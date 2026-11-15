//! From the controller's motor command to the PWM duty: deadband compensation
//! and battery-voltage scaling.
//!
//! The controller ([`crate::control::Controller`]) was tuned on an *ideal*
//! motor, where the command is the wheel's target speed as a fraction of full
//! speed: command 0.1 means 10 % of full speed. The real motors differ in two
//! ways, and [`Drive`] undoes both.
//!
//! # Deadband
//!
//! The TT motors don't move at small duties (see "Known limitations" in
//! CLAUDE.md). A *turning* wheel keeps going down to the *stop* duty: below it
//! the gearbox friction wins. Above it the speed grows roughly linearly with
//! the duty, from 0 at the stop duty to full speed at 100 %. So to get the
//! speed the controller asks for, the command is mapped onto that range:
//!
//! ```text
//! duty = sign(cmd) · (stop + |cmd| · (1 − stop))
//! ```
//!
//! That is the inverse of the motor's behaviour while it turns, so to the
//! controller a turning wheel looks like the ideal motor again. Around 0 the
//! mapping would jump from −stop to +stop with every sign flip of a noisy
//! command, so commands smaller than a *dead zone* give duty 0 (motor off).
//!
//! A *standing* wheel needs more: the higher *start* duty (static friction is
//! larger than sliding friction). This is deliberately **not** compensated.
//! The firmware has no wheel-speed sensor, so it can only guess when a wheel
//! stands. In simulation a "start kick" (at least the start duty for 10-50 ms
//! whenever the command leaves 0, changes direction, or rises out of a small
//! "standing" range) didn't remove the remaining stick-slip, and mapping onto
//! a higher offset (12-15 % instead of the 8 % stop duty) made it worse. See
//! the B2 step in CLAUDE.md for the numbers.
//!
//! # Battery voltage
//!
//! The motor's speed is set by the *voltage* it gets, which is the duty times
//! the battery voltage. A 2S pack goes from 8.4 V full to about 6.4 V empty,
//! so the same duty gives ~30 % more speed on a full pack than on an empty
//! one, and the controller's loop gain changes with it. Scaling the duty by
//! `v_ref / v_battery` keeps the motor voltage, and so the speed, the same.
//! The friction a duty has to overcome is also a voltage (a current, really),
//! so the deadband duties scale the same way: [`Drive`] applies the deadband
//! compensation first, with duties taken as valid at `v_ref`, then scales the
//! result. The deadband was measured at an unknown voltage, so `v_ref` is a
//! convention: the pack's nominal 7.4 V. On an empty pack a full command
//! can't be reached any more: the duty is clamped to 100 %.

use crate::clamp_duty;

/// Deadband compensation settings; see the module docs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Compensation {
    /// Duty (0..=1) below which a turning wheel stops; added to every
    /// command outside the dead zone.
    pub stop_duty: f32,
    /// Commands with `|cmd|` below this give duty 0.
    pub dead_zone: f32,
}

impl Compensation {
    /// The prototype at 100 Hz PWM: stop 8 %, measured 2026-11-15 on all four
    /// motor/direction combinations (wheels in the air, battery voltage not
    /// noted). Dead zone 0.005 (0.5 % of full speed, 2 mm/s): far below the
    /// commands the controller uses while balancing, above the command noise
    /// when the robot stands still.
    pub const PROTOTYPE: Self = Self {
        stop_duty: 0.08,
        dead_zone: 0.005,
    };

    /// No compensation: the duty is the command.
    pub const NONE: Self = Self {
        stop_duty: 0.0,
        dead_zone: 0.0,
    };

    /// `0` inside the dead zone, else `sign(cmd)·(stop + |cmd|·(1 − stop))`.
    /// The command is clamped first; NaN gives 0.
    #[must_use]
    pub fn map(&self, cmd: f32) -> f32 {
        let cmd = clamp_duty(cmd);
        if cmd == 0.0 || cmd.abs() < self.dead_zone {
            0.0
        } else {
            cmd.signum() * (self.stop_duty + cmd.abs() * (1.0 - self.stop_duty))
        }
    }
}

/// Nominal voltage of the 2S Li-ion pack (3.7 V per cell), the reference for
/// [`scale_for_battery`].
pub const V_REF_2S: f32 = 7.4;

/// Scales `duty` so the motor gets the voltage it would get at `v_ref`:
/// `duty · v_ref / battery_v`, clamped to `-1.0..=1.0`. A battery reading of
/// 0 or below (or NaN) is treated as `v_ref`, so a broken ADC reading doesn't
/// switch the motors to full power.
#[must_use]
pub fn scale_for_battery(duty: f32, battery_v: f32, v_ref: f32) -> f32 {
    if battery_v > 0.0 {
        clamp_duty(duty * v_ref / battery_v)
    } else {
        clamp_duty(duty)
    }
}

/// Command → duty: deadband compensation, then battery scaling.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Drive {
    /// Deadband compensation.
    pub compensation: Compensation,
    /// Battery voltage at which the deadband duties hold and no scaling is
    /// applied, in volts.
    pub v_ref: f32,
}

impl Drive {
    /// The prototype: [`Compensation::PROTOTYPE`] at [`V_REF_2S`].
    pub const PROTOTYPE: Self = Self {
        compensation: Compensation::PROTOTYPE,
        v_ref: V_REF_2S,
    };

    /// The duty (−1..=1) for controller command `cmd` with the battery at
    /// `battery_v`.
    #[must_use]
    pub fn duty(&self, cmd: f32, battery_v: f32) -> f32 {
        scale_for_battery(self.compensation.map(cmd), battery_v, self.v_ref)
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "exact values (0, bounds) are compared on purpose"
)]
mod tests {
    use super::{Compensation, Drive, scale_for_battery};

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn maps_the_command_onto_the_turning_range() {
        let c = Compensation::PROTOTYPE;
        assert!(close(c.map(1.0), 1.0));
        assert!(close(c.map(-1.0), -1.0));
        assert!(close(c.map(0.5), 0.08 + 0.5 * 0.92));
        assert!(close(c.map(-0.1), -(0.08 + 0.1 * 0.92)));
    }

    #[test]
    fn dead_zone_and_nan_give_zero() {
        let c = Compensation::PROTOTYPE;
        assert_eq!(c.map(0.004), 0.0);
        assert_eq!(c.map(-0.004), 0.0);
        assert_eq!(c.map(0.0), 0.0);
        assert_eq!(c.map(f32::NAN), 0.0);
        // Just outside the dead zone: the stop duty, not a tiny duty.
        assert!(c.map(0.006) > 0.08);
    }

    #[test]
    fn out_of_range_commands_are_clamped() {
        assert!(close(Compensation::PROTOTYPE.map(3.0), 1.0));
    }

    #[test]
    fn no_compensation_passes_the_command_through() {
        assert!(close(Compensation::NONE.map(0.3), 0.3));
        assert!(close(Compensation::NONE.map(-0.01), -0.01));
    }

    #[test]
    fn battery_scaling_keeps_the_motor_voltage() {
        // At 8.4 V, 74 % duty gives the same 6.2 V as 84 % at 7.4 V.
        assert!(close(scale_for_battery(0.84, 8.4, 7.4), 0.74));
        assert!(close(scale_for_battery(0.5, 7.4, 7.4), 0.5));
        assert!(scale_for_battery(0.5, 6.8, 7.4) > 0.5);
    }

    #[test]
    fn battery_scaling_clamps_and_ignores_bad_readings() {
        assert_eq!(scale_for_battery(0.95, 6.4, 7.4), 1.0);
        assert_eq!(scale_for_battery(-0.95, 6.4, 7.4), -1.0);
        assert!(close(scale_for_battery(0.5, 0.0, 7.4), 0.5));
        assert!(close(scale_for_battery(0.5, f32::NAN, 7.4), 0.5));
    }

    #[test]
    fn drive_compensates_then_scales() {
        let expected = (0.08 + 0.2 * 0.92) * 7.4 / 8.4;
        assert!(close(Drive::PROTOTYPE.duty(0.2, 8.4), expected));
        assert_eq!(Drive::PROTOTYPE.duty(0.001, 8.4), 0.0);
    }
}
