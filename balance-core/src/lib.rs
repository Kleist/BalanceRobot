//! Hardware-independent logic for the balance robot.
//!
//! Everything here is `no_std` so it runs on the ESP32-C3, but it has no
//! hardware dependencies, so it is unit-tested on the host (`cargo test`).

#![no_std]

pub mod attitude;
pub mod battery;
pub mod control;
pub mod drive;
pub mod gyro;
pub mod imu;
pub mod link;
pub mod pid;
pub mod remote;
pub mod safety;

/// Limits a motor duty command to the valid range `-1.0..=1.0`.
///
/// Negative values mean reverse. `NaN` (e.g. from a division by zero
/// upstream) maps to `0.0`, so a bad calculation stops the motor instead of
/// passing garbage to the PWM.
#[must_use]
pub fn clamp_duty(duty: f32) -> f32 {
    if duty.is_nan() {
        0.0
    } else {
        duty.clamp(-1.0, 1.0)
    }
}

/// Splits a signed motor command into a direction and a PWM duty in whole percent.
///
/// Returns `(forward, duty_pct)`. The command is clamped with [`clamp_duty`] first, so
/// the duty is always `0..=100`. A command of 0 gives duty 0 (the caller should stop the
/// motor); the sign of 0 is meaningless then. The LEDC driver takes whole percent, so
/// the command is rounded to the nearest 1 %.
#[must_use]
pub fn split_command(cmd: f32) -> (bool, u8) {
    let cmd = clamp_duty(cmd);
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "|cmd| is in 0..=1, so the rounded percent is in 0..=100"
    )]
    let duty_pct = (cmd.abs() * 100.0 + 0.5) as u8;
    (cmd >= 0.0, duty_pct)
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "clamp returns the exact bound or input, so exact comparison is intended"
)]
mod tests {
    use super::{clamp_duty, split_command};

    #[test]
    fn split_gives_direction_and_percent() {
        assert_eq!(split_command(0.5), (true, 50));
        assert_eq!(split_command(-0.25), (false, 25));
    }

    #[test]
    fn split_rounds_to_nearest_percent() {
        assert_eq!(split_command(0.124), (true, 12));
        assert_eq!(split_command(-0.126), (false, 13));
    }

    #[test]
    fn split_clamps_and_stops_on_nan() {
        assert_eq!(split_command(3.0), (true, 100));
        assert_eq!(split_command(-3.0), (false, 100));
        assert_eq!(split_command(f32::NAN).1, 0);
        assert_eq!(split_command(0.0).1, 0);
    }

    #[test]
    fn passes_values_in_range_through() {
        assert_eq!(clamp_duty(0.5), 0.5);
        assert_eq!(clamp_duty(-0.25), -0.25);
    }

    #[test]
    fn clamps_values_out_of_range() {
        assert_eq!(clamp_duty(1.5), 1.0);
        assert_eq!(clamp_duty(f32::NEG_INFINITY), -1.0);
    }

    #[test]
    fn nan_stops_the_motor() {
        assert_eq!(clamp_duty(f32::NAN), 0.0);
    }
}
