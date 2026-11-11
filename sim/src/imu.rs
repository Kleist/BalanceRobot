//! Model of the ICM-42670-P: what the IMU would read, in raw LSB, for a given
//! motion of the robot.
//!
//! # What an accelerometer measures
//!
//! Not acceleration, but *specific force*: acceleration minus gravity,
//! `p'' − g⃗`. Lying still it reads +1 g up (the table pushing it up). In free
//! fall it reads 0. So it sees gravity (which gives the tilt) **plus** any
//! acceleration of the point where the chip sits.
//!
//! The chip sits on the body, `h` above the axle. When the body rotates, that
//! point accelerates even if the axle doesn't move:
//!
//! - **tangential** `h·θ''` along the body's forward axis (the body speeding
//!   up its rotation),
//! - **centripetal** `h·θ'²` towards the axle (the body rotating at all).
//!
//! Adding the axle's own acceleration `a` and projecting onto the body axes:
//!
//! ```text
//! forward = a·cos θ + h·θ'' − g·sin θ
//! up      = a·sin θ − h·θ'² + g·cos θ
//! ```
//!
//! With `h = 0` or the robot at rest, this is the pure gravity reading that
//! `balance_core::attitude::accel_pitch_deg` assumes. Otherwise the accel
//! pitch is wrong while the robot moves, which is why the firmware fuses it
//! with the gyro.
//!
//! # What the gyro measures
//!
//! The rotation rate, directly: `θ'` about the axle.
//!
//! # From physics to raw numbers
//!
//! Each reading is scaled to LSB (8192 per g, 65.5 per °/s, the firmware's
//! ranges), then gets a constant **bias** and random **noise** added, and is
//! finally **quantised**: rounded to an integer and clipped to the i16 range.
//!
//! # Axes
//!
//! Same mapping as the firmware's `robot_axes` / `robot_pitch_rate`: IMU X =
//! forward, Z = up, Y = along the axle, and the pitch rate is gyro Y. The
//! robot doesn't move sideways or turn in this model, so accel Y and gyro X/Z
//! read only their bias and noise.

use balance_core::imu::RawSample;

use crate::noise::Gaussian;
use crate::pendulum::{G, State};

/// Accelerometer scale at ±4 g.
pub const ACCEL_LSB_PER_G: f64 = 8192.0;
/// Gyro scale at ±500 °/s.
pub const GYRO_LSB_PER_DPS: f64 = 65.5;

/// How the IMU is mounted and how imperfect it is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuParams {
    /// Height of the chip above the axle in m.
    pub height_above_axle_m: f64,
    /// Standard deviation of the accelerometer noise per sample, in LSB.
    pub accel_noise_lsb: f64,
    /// Standard deviation of the gyro noise per sample, in LSB.
    pub gyro_noise_lsb: f64,
    /// Constant accelerometer offset per axis `[x, y, z]`, in LSB.
    pub accel_bias_lsb: [f64; 3],
    /// Constant gyro offset per axis `[x, y, z]`, in LSB.
    pub gyro_bias_lsb: [f64; 3],
}

impl ImuParams {
    /// A perfect IMU (no noise, no bias) `height_m` above the axle. Readings
    /// are still quantised.
    #[must_use]
    pub const fn ideal(height_m: f64) -> Self {
        Self {
            height_above_axle_m: height_m,
            accel_noise_lsb: 0.0,
            gyro_noise_lsb: 0.0,
            accel_bias_lsb: [0.0; 3],
            gyro_bias_lsb: [0.0; 3],
        }
    }

    /// Placeholder values for the prototype, **not measured yet**:
    ///
    /// - height 10 cm: a guess.
    /// - noise: datasheet-level noise density (about 70 µg/√Hz and
    ///   0.007 °/s/√Hz) over a few hundred Hz of bandwidth, ≈ 1.5 mg and
    ///   ≈ 0.14 °/s per sample.
    /// - bias: typical sizes (a few mg, a few tenths of a °/s), signs made up.
    ///
    /// Replace them with numbers from a still log of the real robot (see the
    /// `log_stats` example).
    #[must_use]
    pub const fn prototype() -> Self {
        Self {
            height_above_axle_m: 0.10,
            accel_noise_lsb: 12.0,
            gyro_noise_lsb: 9.0,
            accel_bias_lsb: [40.0, -20.0, 60.0],
            gyro_bias_lsb: [20.0, -15.0, 5.0],
        }
    }
}

/// Specific force `(forward, up)` in m/s² at a point `height_m` above the
/// axle (the formula in the module docs).
///
/// `pitch_accel_rad_s2` is passed in rather than computed from the physics,
/// so the robot can also be *held* (e.g. still in a hand during calibration,
/// where it doesn't fall).
#[must_use]
pub fn specific_force(
    state: &State,
    pitch_accel_rad_s2: f64,
    axle_accel_m_s2: f64,
    height_m: f64,
) -> (f64, f64) {
    let (sin, cos) = state.pitch_rad.sin_cos();
    let forward = axle_accel_m_s2 * cos + height_m * pitch_accel_rad_s2 - G * sin;
    let up = axle_accel_m_s2 * sin - height_m * state.pitch_rate_rad_s.powi(2) + G * cos;
    (forward, up)
}

/// A simulated IMU: turns motion into [`RawSample`]s, like the firmware's
/// burst read.
#[derive(Debug, Clone)]
pub struct Imu {
    params: ImuParams,
    noise: Gaussian,
}

impl Imu {
    /// An IMU with `params`. The same `seed` gives the same noise every run,
    /// so tests and logs are reproducible.
    #[must_use]
    pub const fn new(params: ImuParams, seed: u64) -> Self {
        Self {
            params,
            noise: Gaussian::new(seed),
        }
    }

    /// What the chip reads for this motion (see [`specific_force`] for the
    /// arguments).
    pub fn sample(
        &mut self,
        state: &State,
        pitch_accel_rad_s2: f64,
        axle_accel_m_s2: f64,
    ) -> RawSample {
        let p = self.params;
        let (forward, up) = specific_force(
            state,
            pitch_accel_rad_s2,
            axle_accel_m_s2,
            p.height_above_axle_m,
        );
        let accel_true = [forward, 0.0, up].map(|a| a / G * ACCEL_LSB_PER_G);
        let pitch_rate_dps = state.pitch_rate_rad_s.to_degrees();
        let gyro_true = [0.0, pitch_rate_dps * GYRO_LSB_PER_DPS, 0.0];

        let mut accel = [0; 3];
        let mut gyro = [0; 3];
        for axis in 0..3 {
            accel[axis] = quantise(
                accel_true[axis] + p.accel_bias_lsb[axis] + p.accel_noise_lsb * self.noise.sample(),
            );
            gyro[axis] = quantise(
                gyro_true[axis] + p.gyro_bias_lsb[axis] + p.gyro_noise_lsb * self.noise.sample(),
            );
        }
        RawSample { accel, gyro }
    }
}

/// Rounds to the nearest integer and clips to what an i16 holds.
///
/// Clips at −32767, not −32768: the real chip uses −32768 to mean "no valid
/// data".
#[expect(
    clippy::cast_possible_truncation,
    reason = "the value is rounded and clamped to the i16 range first"
)]
fn quantise(lsb: f64) -> i16 {
    lsb.round().clamp(-32767.0, 32767.0) as i16
}

#[cfg(test)]
mod tests {
    use balance_core::attitude::accel_pitch_deg;

    use super::{G, Imu, ImuParams, specific_force};
    use crate::pendulum::{Body, State};

    fn ideal(height_m: f64) -> Imu {
        Imu::new(ImuParams::ideal(height_m), 1)
    }

    fn rotating(rate_dps: f64) -> State {
        State {
            pitch_rate_rad_s: rate_dps.to_radians(),
            ..State::default()
        }
    }

    #[test]
    fn upright_and_still_reads_one_g_up() {
        let raw = ideal(0.1).sample(&State::default(), 0.0, 0.0);
        assert_eq!(raw.accel, [0, 0, 8192]);
        assert_eq!(raw.gyro, [0, 0, 0]);
    }

    #[test]
    fn held_at_30_degrees_reads_gravity_rotated() {
        let raw = ideal(0.1).sample(&State::tilted_deg(30.0), 0.0, 0.0);
        // −sin 30° · 8192 = −4096, cos 30° · 8192 = 7094.5 → 7094 after rounding.
        assert_eq!(raw.accel, [-4096, 0, 7094]);
        let pitch = accel_pitch_deg(f32::from(raw.accel[0]), f32::from(raw.accel[2]));
        assert!((pitch - 30.0).abs() < 0.01, "{pitch}");
    }

    #[test]
    fn gyro_reads_the_pitch_rate_on_y() {
        let raw = ideal(0.1).sample(&rotating(10.0), 0.0, 0.0);
        assert_eq!(raw.gyro, [0, 655, 0]);
    }

    #[test]
    fn readings_clip_at_the_range() {
        let mut imu = ideal(0.0);
        assert_eq!(imu.sample(&rotating(1000.0), 0.0, 0.0).gyro[1], 32767);
        assert_eq!(imu.sample(&rotating(-1000.0), 0.0, 0.0).gyro[1], -32767);
    }

    #[test]
    fn axle_acceleration_shows_up_on_forward() {
        // Upright, held, axle speeding up at 0.5 g: forward reads +0.5 g.
        let raw = ideal(0.1).sample(&State::default(), 0.0, 0.5 * G);
        assert_eq!(raw.accel, [4096, 0, 8192]);
    }

    #[test]
    fn rotation_pulls_up_towards_the_axle() {
        // Centripetal: up = g − h·ω². At h = 0.1 m and ω = 5 rad/s: 9.81 − 2.5.
        let state = State {
            pitch_rate_rad_s: 5.0,
            ..State::default()
        };
        let (forward, up) = specific_force(&state, 0.0, 0.0, 0.1);
        assert!(forward.abs() < 1e-12);
        assert!((up - (G - 2.5)).abs() < 1e-12, "{up}");
    }

    #[test]
    fn on_the_axle_a_fall_looks_like_a_static_tilt() {
        // The axle point doesn't accelerate, so only gravity is seen.
        let body = Body::prototype();
        let state = State::tilted_deg(10.0);
        let falling = specific_force(&state, body.pitch_accel(state.pitch_rad, 0.0), 0.0, 0.0);
        let held = specific_force(&state, 0.0, 0.0, 0.0);
        assert!((falling.0 - held.0).abs() < 1e-12);
    }

    #[test]
    fn at_the_centre_of_percussion_a_fall_reads_no_tilt() {
        // At h = I/(m·d) the tangential acceleration h·θ'' exactly cancels
        // gravity's −g·sin θ: an IMU there reads forward = 0 at the start of
        // a fall, so its accel pitch says "upright" while the robot tips over.
        let body = Body::prototype();
        let h = body.inertia_kg_m2 / (body.mass_kg * body.com_height_m);
        let state = State::tilted_deg(10.0);
        let (forward, _) = specific_force(&state, body.pitch_accel(state.pitch_rad, 0.0), 0.0, h);
        assert!(forward.abs() < 1e-9, "{forward}");
    }

    #[test]
    fn still_samples_average_to_the_bias_with_the_set_noise() {
        let params = ImuParams::prototype();
        let mut imu = Imu::new(params, 42);
        let n = 20_000;
        let samples: Vec<_> = (0..n)
            .map(|_| imu.sample(&State::default(), 0.0, 0.0))
            .collect();

        let stats = |values: Vec<f64>| {
            let mean = values.iter().sum::<f64>() / f64::from(n);
            let var = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / f64::from(n);
            (mean, var.sqrt())
        };
        for axis in 0..3 {
            let (mean, std) = stats(samples.iter().map(|s| f64::from(s.gyro[axis])).collect());
            // Mean within ~4 standard errors (9 / √20000 ≈ 0.06 LSB).
            assert!(
                (mean - params.gyro_bias_lsb[axis]).abs() < 0.3,
                "gyro {axis}: {mean}"
            );
            // Rounding adds 1/12 LSB² of variance, negligible next to 81.
            assert!(
                (std / params.gyro_noise_lsb - 1.0).abs() < 0.05,
                "gyro {axis}: {std}"
            );
        }
        let (mean_z, std_z) = stats(samples.iter().map(|s| f64::from(s.accel[2])).collect());
        assert!(
            (mean_z - (8192.0 + params.accel_bias_lsb[2])).abs() < 0.4,
            "{mean_z}"
        );
        assert!(
            (std_z / params.accel_noise_lsb - 1.0).abs() < 0.05,
            "{std_z}"
        );
    }

    #[test]
    fn same_seed_same_readings() {
        let mut a = Imu::new(ImuParams::prototype(), 7);
        let mut b = Imu::new(ImuParams::prototype(), 7);
        for _ in 0..100 {
            assert_eq!(
                a.sample(&State::default(), 0.0, 0.0),
                b.sample(&State::default(), 0.0, 0.0)
            );
        }
    }
}
