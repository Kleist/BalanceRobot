//! Hardware-independent logic for the balance robot.
//!
//! Everything here is `no_std` so it runs on the ESP32-C3, but it has no
//! hardware dependencies, so it is unit-tested on the host (`cargo test`).

#![no_std]

pub mod attitude;

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

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "clamp returns the exact bound or input, so exact comparison is intended"
)]
mod tests {
    use super::clamp_duty;

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
