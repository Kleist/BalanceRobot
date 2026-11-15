//! Ideal DC motor: the command sets a wheel speed, which the wheels reach
//! with a short delay.
//!
//! A DC motor is closer to a *speed* source than a force source: the
//! voltage (here the PWM duty) sets how fast it wants to turn. When it turns
//! slower than that it pushes hard; near that speed it barely pushes, and
//! faster than that it brakes (its back-EMF, the voltage it generates
//! itself, then exceeds the supply). That is a first-order lag:
//!
//! ```text
//! a = (cmd·v_max − v) / τ_m
//! ```
//!
//! `v_max` is the no-load speed at full duty, `τ_m` how long the wheels take
//! to get ~63 % of the way to a new speed.
//!
//! "Ideal" here means linear: no deadband, no battery sag, no gear
//! backlash, and the load (the body) doesn't slow the motor. The motor-model
//! step adds those.

/// First-order motor model for both wheels together.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Motor {
    /// Wheel speed at full duty, no load, in m/s (axle speed over the floor).
    pub max_speed_m_s: f64,
    /// Time constant in s.
    pub time_constant_s: f64,
}

impl Motor {
    /// The prototype.
    ///
    /// - max speed 0.41 m/s: measured 2026-11-15, 10 turns in 5 s at full
    ///   duty with the wheels in the air (120 rpm, 65 mm wheels). Battery
    ///   voltage not noted.
    /// - time constant 50 ms: an estimate. At standstill and full duty it
    ///   gives 0.41 / 0.05 ≈ 8 m/s², about what two TT motors' typical stall
    ///   torque (~0.08 N·m each) can do with this robot's mass plus its
    ///   wheel and rotor inertia (`Body::prototype`).
    #[must_use]
    pub const fn prototype() -> Self {
        Self {
            max_speed_m_s: 0.41,
            time_constant_s: 0.05,
        }
    }

    /// Axle acceleration in m/s² with command `cmd` (−1..=1, clamped) while
    /// the axle moves at `axle_vel_m_s`.
    #[must_use]
    pub fn axle_accel(&self, cmd: f64, axle_vel_m_s: f64) -> f64 {
        (cmd.clamp(-1.0, 1.0) * self.max_speed_m_s - axle_vel_m_s) / self.time_constant_s
    }
}

#[cfg(test)]
mod tests {
    use super::Motor;

    #[test]
    fn full_command_from_standstill_accelerates_hardest() {
        let a = Motor::prototype().axle_accel(1.0, 0.0);
        assert!((a - 8.2).abs() < 1e-9, "{a}");
    }

    #[test]
    fn at_the_commanded_speed_it_stops_pushing() {
        let a = Motor::prototype().axle_accel(0.5, 0.205);
        assert!(a.abs() < 1e-9, "{a}");
    }

    #[test]
    fn faster_than_commanded_brakes() {
        assert!(Motor::prototype().axle_accel(0.0, 0.2) < 0.0);
    }

    #[test]
    fn command_is_clamped() {
        let m = Motor::prototype();
        assert!((m.axle_accel(3.0, 0.0) - m.axle_accel(1.0, 0.0)).abs() < 1e-12);
    }
}
