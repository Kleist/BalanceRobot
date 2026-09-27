//! Estimating the robot's tilt (pitch) from IMU readings.

/// Pitch angle in degrees from the accelerometer alone.
///
/// At rest an accelerometer measures only gravity's reaction: a 1 g vector
/// pointing *up*. Tilting the robot rotates that vector in the sensor frame,
/// so its direction gives the tilt. Only the direction matters, so any unit
/// works (g, m/s², raw counts).
///
/// - `forward`: acceleration along the robot's forward axis
/// - `up`: acceleration along the robot's up axis (reads +1 g when upright)
///
/// Mapping the IMU's X/Y/Z axes (and signs) to forward/up is up to the
/// caller, because it depends on how the board is mounted.
///
/// Sign convention: **positive pitch = tilted forward**. Tipping forward by θ
/// makes the forward axis point partly down, so it reads −sin θ while `up`
/// reads cos θ, hence `atan2(-forward, up)`.
///
/// Returns a value in −180..=180. A zero vector (free fall) returns 0.
///
/// Any real acceleration (driving, vibration) also rotates the measured
/// vector, so this angle is noisy while moving. That is what the gyro and
/// the complementary filter are for.
#[must_use]
pub fn accel_pitch_deg(forward: f32, up: f32) -> f32 {
    libm::atan2f(-forward, up).to_degrees()
}

#[cfg(test)]
mod tests {
    use super::accel_pitch_deg;

    /// Readings for a robot pitched forward by `deg`, scaled by `g`.
    fn reading(deg: f32, g: f32) -> (f32, f32) {
        let rad = deg.to_radians();
        (-libm::sinf(rad) * g, libm::cosf(rad) * g)
    }

    fn assert_close(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() < 1e-3,
            "expected {expected}°, got {actual}°"
        );
    }

    #[test]
    fn upright_is_zero() {
        assert_close(accel_pitch_deg(0.0, 1.0), 0.0);
    }

    #[test]
    fn lying_on_front_and_back() {
        assert_close(accel_pitch_deg(-1.0, 0.0), 90.0);
        assert_close(accel_pitch_deg(1.0, 0.0), -90.0);
    }

    #[test]
    fn forty_five_degrees_forward() {
        let (forward, up) = reading(45.0, 1.0);
        assert_close(accel_pitch_deg(forward, up), 45.0);
    }

    #[test]
    fn unit_does_not_matter() {
        let (forward, up) = reading(30.0, 9.81);
        assert_close(accel_pitch_deg(forward, up), 30.0);
    }

    #[test]
    fn upside_down_is_plus_or_minus_180() {
        assert_close(accel_pitch_deg(0.0, -1.0).abs(), 180.0);
    }

    #[test]
    fn free_fall_is_zero() {
        assert_close(accel_pitch_deg(0.0, 0.0), 0.0);
    }

    #[test]
    fn round_trips_every_whole_degree() {
        for deg in -179_i16..=179 {
            let deg = f32::from(deg);
            let (forward, up) = reading(deg, 1.0);
            assert_close(accel_pitch_deg(forward, up), deg);
        }
    }
}
