//! DC motor: the command sets a wheel speed, which the wheels reach with a
//! short delay, optionally with the TT motors' PWM deadband.
//!
//! A DC motor is closer to a *speed* source than a force source: the
//! voltage (here the PWM duty) sets how fast it wants to turn. When it turns
//! slower than that it pushes hard; near that speed it barely pushes, and
//! faster than that it brakes (its back-EMF, the voltage it generates
//! itself, then exceeds the supply). That is a first-order lag:
//!
//! ```text
//! a = (target·v_max − v) / τ_m
//! ```
//!
//! `v_max` is the no-load speed at full duty, `τ_m` how long the wheels take
//! to get ~63 % of the way to a new speed. In the ideal motor
//! ([`Motor::ideal`]) the target is the command itself.
//!
//! # Deadband
//!
//! The real TT motors don't move at small duties, and they show hysteresis
//! (see "Known limitations" in CLAUDE.md): a standing wheel needs the
//! *start* duty to break loose (static friction in the gearbox), a turning
//! one keeps going down to the lower *stop* duty (sliding friction is
//! smaller). [`Deadband`] models that with two rules:
//!
//! - **Turning** (|wheel speed| above [`Deadband::standing_speed_m_s`]): the
//!   friction eats the stop duty, the rest drives the wheel. The target speed
//!   is `sign(cmd)·(|cmd| − stop)/(1 − stop)`, or 0 below the stop duty. So
//!   the target is 0 at the stop duty and full speed at full duty, linear in
//!   between.
//! - **Standing**: below the start duty the static friction holds the wheel:
//!   target 0 (the lag pulls the leftover speed to 0). At or above the start
//!   duty the wheel breaks loose and follows the turning rule. Breaking loose
//!   jumps straight to the target of a turning wheel at that duty (≈ 0.05
//!   m/s at the start duty), because the motor is then already pushing that
//!   hard.
//!
//! "Standing" is decided from the wheel speed alone, so the model has no
//! state of its own. A wheel that reverses passes through "standing" and
//! then needs the start duty again, like the real one.
//!
//! Still missing: battery sag, gear backlash, and the load (the body)
//! slowing the motor.

/// First-order motor model for both wheels together.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Motor {
    /// Wheel speed at full duty, no load, in m/s (axle speed over the floor).
    pub max_speed_m_s: f64,
    /// Time constant in s.
    pub time_constant_s: f64,
    /// PWM deadband, or `None` for a linear motor.
    pub deadband: Option<Deadband>,
}

/// The duty a wheel needs to start and to keep turning; see the module docs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Deadband {
    /// Duty (0..=1) a standing wheel needs to start turning.
    pub start_duty: f64,
    /// Duty (0..=1) below which a turning wheel stops.
    pub stop_duty: f64,
    /// Below this |wheel speed| in m/s the wheel counts as standing.
    pub standing_speed_m_s: f64,
}

impl Deadband {
    /// The prototype at 100 Hz PWM, averaged over both motors and directions
    /// (measured 2026-11-15, wheels in the air): start 16-22 % → 19 %, stop
    /// 8 % on all four. The standing threshold, 5 mm/s (≈ 1.5 rpm), is a
    /// choice: well below the slowest turning speed (≈ 0.05 m/s just above
    /// the start duty), well above the speed noise of the physics.
    pub const PROTOTYPE: Self = Self {
        start_duty: 0.19,
        stop_duty: 0.08,
        standing_speed_m_s: 0.005,
    };

    /// Target speed as a fraction of full speed, −1..=1, for command `cmd`
    /// (already clamped) while the wheel moves at `vel_m_s`.
    fn target(&self, cmd: f64, vel_m_s: f64) -> f64 {
        let duty = cmd.abs();
        let standing = vel_m_s.abs() < self.standing_speed_m_s;
        if duty < self.stop_duty || (standing && duty < self.start_duty) {
            0.0
        } else {
            cmd.signum() * (duty - self.stop_duty) / (1.0 - self.stop_duty)
        }
    }
}

impl Motor {
    /// The prototype without deadband: linear, the command is the target
    /// speed.
    ///
    /// - max speed 0.41 m/s: measured 2026-11-15, 10 turns in 5 s at full
    ///   duty with the wheels in the air (120 rpm, 65 mm wheels). Battery
    ///   voltage not noted.
    /// - time constant 50 ms: an estimate. At standstill and full duty it
    ///   gives 0.41 / 0.05 ≈ 8 m/s², about what two TT motors' typical stall
    ///   torque (~0.08 N·m each) can do with this robot's mass plus its
    ///   wheel and rotor inertia (`Body::prototype`).
    #[must_use]
    pub const fn ideal() -> Self {
        Self {
            max_speed_m_s: 0.41,
            time_constant_s: 0.05,
            deadband: None,
        }
    }

    /// The prototype: [`Motor::ideal`] plus [`Deadband::PROTOTYPE`].
    #[must_use]
    pub const fn prototype() -> Self {
        Self {
            deadband: Some(Deadband::PROTOTYPE),
            ..Self::ideal()
        }
    }

    /// Axle acceleration in m/s² with command `cmd` (−1..=1, clamped) while
    /// the axle moves at `axle_vel_m_s`.
    #[must_use]
    pub fn axle_accel(&self, cmd: f64, axle_vel_m_s: f64) -> f64 {
        let cmd = cmd.clamp(-1.0, 1.0);
        let target = self.deadband.map_or(cmd, |d| d.target(cmd, axle_vel_m_s));
        (target * self.max_speed_m_s - axle_vel_m_s) / self.time_constant_s
    }
}

#[cfg(test)]
mod tests {
    use super::Motor;

    #[test]
    fn full_command_from_standstill_accelerates_hardest() {
        let a = Motor::ideal().axle_accel(1.0, 0.0);
        assert!((a - 8.2).abs() < 1e-9, "{a}");
    }

    #[test]
    fn at_the_commanded_speed_it_stops_pushing() {
        let a = Motor::ideal().axle_accel(0.5, 0.205);
        assert!(a.abs() < 1e-9, "{a}");
    }

    #[test]
    fn faster_than_commanded_brakes() {
        assert!(Motor::ideal().axle_accel(0.0, 0.2) < 0.0);
    }

    #[test]
    fn command_is_clamped() {
        let m = Motor::ideal();
        assert!((m.axle_accel(3.0, 0.0) - m.axle_accel(1.0, 0.0)).abs() < 1e-12);
    }

    #[test]
    fn standing_wheel_needs_the_start_duty() {
        let m = Motor::prototype();
        assert!(m.axle_accel(0.18, 0.0).abs() < 1e-12);
        assert!(m.axle_accel(-0.18, 0.0).abs() < 1e-12);
        assert!(m.axle_accel(0.19, 0.0) > 0.0);
        assert!(m.axle_accel(-0.19, 0.0) < 0.0);
    }

    #[test]
    fn turning_wheel_keeps_going_down_to_the_stop_duty() {
        let m = Motor::prototype();
        // Turning forward at 0.1 m/s, command 0.10: above stop, so it pushes
        // towards (0.10 − 0.08) / 0.92 · 0.41 ≈ 0.009 m/s (i.e. slows down),
        // but a standing wheel would not move at all at 0.10.
        let target = (0.10 - 0.08) / 0.92 * 0.41;
        let a = m.axle_accel(0.10, 0.1);
        assert!((a - (target - 0.1) / 0.05).abs() < 1e-9, "{a}");
        assert!(m.axle_accel(0.10, 0.0).abs() < 1e-12);
    }

    #[test]
    fn below_the_stop_duty_it_brakes_to_standstill() {
        let m = Motor::prototype();
        let a = m.axle_accel(0.05, 0.1);
        assert!((a - (-0.1 / 0.05)).abs() < 1e-9, "{a}");
    }

    #[test]
    fn full_duty_reaches_full_speed() {
        let m = Motor::prototype();
        assert!(m.axle_accel(1.0, 0.41).abs() < 1e-9);
        assert!(m.axle_accel(-1.0, -0.41).abs() < 1e-9);
    }
}
