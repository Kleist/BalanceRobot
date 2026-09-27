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

/// Fuses gyro and accelerometer into one pitch estimate.
///
/// The two sensors fail in opposite ways:
/// - the **gyro** is smooth and fast, but integrating its rate adds up every
///   bit of bias, so the angle slowly drifts away;
/// - the **accelerometer** angle ([`accel_pitch_deg`]) never drifts, but every
///   bump and vibration shows up as a jump.
///
/// Each update first predicts with the gyro (`angle + rate·dt`) and then
/// nudges that prediction a little towards the accelerometer angle:
///
/// ```text
/// angle = α·(angle + rate·dt) + (1 − α)·accel_angle,   α = τ / (τ + dt)
/// ```
///
/// The time constant `τ` (seconds) sets the balance: changes faster than `τ`
/// come from the gyro, slower ones from the accelerometer. Computing `α` from
/// `dt` keeps that behaviour the same if the loop rate changes. A leftover
/// gyro bias `b` still shifts the angle by about `b·τ`, so calibrate the bias
/// anyway.
///
/// Units: degrees and degrees per second. The gyro rate must use the same sign
/// convention as the pitch (positive = rotating forward).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ComplementaryFilter {
    tau_s: f32,
    angle_deg: Option<f32>,
}

impl ComplementaryFilter {
    /// A filter with time constant `tau_s` in seconds (typically 0.5–2 s).
    #[must_use]
    pub const fn new(tau_s: f32) -> Self {
        Self {
            tau_s,
            angle_deg: None,
        }
    }

    /// Feeds one sample and returns the new pitch estimate in degrees.
    ///
    /// The first call starts from the accelerometer angle, so the estimate
    /// doesn't have to crawl up from 0° over several `τ`.
    pub fn update(&mut self, accel_angle_deg: f32, gyro_rate_dps: f32, dt_s: f32) -> f32 {
        let angle = match self.angle_deg {
            None => accel_angle_deg,
            Some(prev) => {
                let alpha = self.tau_s / (self.tau_s + dt_s);
                alpha * (prev + gyro_rate_dps * dt_s) + (1.0 - alpha) * accel_angle_deg
            }
        };
        self.angle_deg = Some(angle);
        angle
    }

    /// The current estimate, or `None` before the first [`update`](Self::update).
    #[must_use]
    pub const fn angle_deg(&self) -> Option<f32> {
        self.angle_deg
    }
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

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "the first update returns the accel angle unchanged, so exact comparison is intended"
)]
mod filter_tests {
    use super::ComplementaryFilter;

    const DT: f32 = 0.002; // 500 Hz, the planned control loop rate
    const TAU: f32 = 1.0;

    /// Runs the filter for `steps` samples of `dt`, feeding `sample(t)`.
    fn run(
        filter: &mut ComplementaryFilter,
        steps: u16,
        dt: f32,
        mut sample: impl FnMut(f32) -> (f32, f32),
    ) -> f32 {
        let mut angle = 0.0;
        for i in 0..steps {
            let (accel, rate) = sample(f32::from(i) * dt);
            angle = filter.update(accel, rate, dt);
        }
        angle
    }

    /// Number of 500 Hz samples in `seconds` whole seconds.
    fn steps(seconds: u16) -> u16 {
        seconds * 500
    }

    #[test]
    fn first_update_starts_from_accel_angle() {
        let mut f = ComplementaryFilter::new(TAU);
        assert_eq!(f.angle_deg(), None);
        assert_eq!(f.update(12.0, 100.0, DT), 12.0);
        assert_eq!(f.angle_deg(), Some(12.0));
    }

    #[test]
    fn gyro_bias_drifts_alone_but_stays_bounded_in_filter() {
        // Robot held still at 5°, gyro has 0.5 °/s bias, for one minute.
        let bias = 0.5;

        let mut gyro_only = 5.0;
        for _ in 0..steps(60) {
            gyro_only += bias * DT;
        }
        assert!(gyro_only - 5.0 > 29.0, "pure integration drifts ~30°");

        let mut f = ComplementaryFilter::new(TAU);
        let angle = run(&mut f, steps(60), DT, |_| (5.0, bias));
        // Steady-state offset is about bias·τ = 0.5°.
        assert!((angle - 5.0 - bias * TAU).abs() < 0.01, "got {angle}");
    }

    #[test]
    fn accel_noise_is_smoothed() {
        // True angle 0°, accel alternates ±5° (vibration), gyro is clean.
        // Start at the true angle: starting from one noisy sample would leave
        // a start-up error that takes a few τ to decay.
        let mut f = ComplementaryFilter::new(TAU);
        f.update(0.0, 0.0, DT);
        let mut flip = 1.0;
        let angle = run(&mut f, steps(2), DT, |_| {
            flip = -flip;
            (5.0 * flip, 0.0)
        });
        // Each ±5° sample only moves the estimate by (1 − α)·5° ≈ 0.01°.
        assert!(angle.abs() < 0.05, "got {angle}");
    }

    #[test]
    fn follows_fast_rotation_without_lag() {
        // Tilting at 30 °/s, both sensors agree: the gyro carries the motion.
        let mut f = ComplementaryFilter::new(TAU);
        let angle = run(&mut f, steps(1), DT, |t| (30.0 * t, 30.0));
        assert!((angle - 30.0).abs() < 0.1, "got {angle}");
    }

    #[test]
    fn accel_step_settles_with_time_constant() {
        // Start at 0°, then the accel says 10° while the gyro says "no motion"
        // (e.g. a wrong initial guess). After one τ the estimate has moved
        // ~63% (1 − 1/e) of the way; after 5τ it is essentially there.
        let mut f = ComplementaryFilter::new(TAU);
        f.update(0.0, 0.0, DT);
        let after_tau = run(&mut f, steps(1), DT, |_| (10.0, 0.0));
        assert!((after_tau - 6.32).abs() < 0.05, "got {after_tau}");
        let settled = run(&mut f, steps(4), DT, |_| (10.0, 0.0));
        assert!((settled - 10.0).abs() < 0.1, "got {settled}");
    }

    #[test]
    fn loop_rate_does_not_change_the_response() {
        let mut fast = ComplementaryFilter::new(TAU);
        let mut slow = ComplementaryFilter::new(TAU);
        fast.update(0.0, 0.0, 0.002);
        slow.update(0.0, 0.0, 0.01);
        // One second each: 500 samples at 500 Hz, 100 samples at 100 Hz.
        let a = run(&mut fast, 500, 0.002, |_| (10.0, 0.0));
        let b = run(&mut slow, 100, 0.01, |_| (10.0, 0.0));
        assert!((a - b).abs() < 0.05, "500 Hz: {a}, 100 Hz: {b}");
    }
}
