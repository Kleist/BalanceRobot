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
//! The specific force is first rotated by the chip's **mounting tilt** (the
//! board doesn't sit exactly square on the body). Each reading is then scaled
//! to LSB (8192 per g, 65.5 per °/s, the firmware's ranges), gets a constant
//! **bias** and random **noise** added, and is finally **quantised**: rounded
//! to an integer and clipped to the i16 range.
//!
//! # Axes
//!
//! Same mapping as the firmware's `robot_axes` / `robot_pitch_rate`: the
//! board stands upright, IMU +X = up, +Z = forward, +Y = right (along the
//! axle). Tipping forward is a negative rotation about +Y, so the gyro reads
//! **minus** the pitch rate on Y. The robot doesn't move sideways or turn in
//! this model, so accel Y and gyro X/Z read only their bias and noise.

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
    /// How far the chip is pitched forward relative to the body, in degrees:
    /// what its accel pitch reads when the body is balanced.
    pub mount_tilt_deg: f64,
    /// Standard deviation of the accelerometer noise per sample and axis
    /// `[x, y, z]`, in LSB.
    pub accel_noise_lsb: [f64; 3],
    /// Standard deviation of the gyro noise per sample and axis `[x, y, z]`,
    /// in LSB.
    pub gyro_noise_lsb: [f64; 3],
    /// Constant accelerometer offset per axis `[x, y, z]`, in LSB.
    pub accel_bias_lsb: [f64; 3],
    /// Constant gyro offset per axis `[x, y, z]`, in LSB.
    pub gyro_bias_lsb: [f64; 3],
}

impl ImuParams {
    /// A perfect IMU (square on the body, no noise, no bias) `height_m` above
    /// the axle. Readings are still quantised.
    #[must_use]
    pub const fn ideal(height_m: f64) -> Self {
        Self {
            height_above_axle_m: height_m,
            mount_tilt_deg: 0.0,
            accel_noise_lsb: [0.0; 3],
            gyro_noise_lsb: [0.0; 3],
            accel_bias_lsb: [0.0; 3],
            gyro_bias_lsb: [0.0; 3],
        }
    }

    /// The prototype's IMU. Measured on 2026-11-15 from a 60 s log of the
    /// robot lying still (`log_stats` example), unless marked as a guess:
    ///
    /// - height 10 cm: a guess, not measured yet.
    /// - mount tilt −3.8°: the accel pitch with the robot held at its balance
    ///   point (the firmware's `BALANCE_PITCH_DEG`). Whether that is the board
    ///   sitting crooked or an accelerometer offset can't be told apart from
    ///   one position; the model puts it all in the tilt.
    /// - noise: the per-sample standard deviation of each axis. It is white:
    ///   the standard deviation of sample-to-sample differences, divided by
    ///   √2, gives the same numbers, and the mean doesn't drift within the
    ///   minute. Y and Z of the accelerometer are ~1.5× noisier than X; the
    ///   gyro (≈ 0.09 °/s on Y) is quieter than the datasheet suggested.
    /// - gyro bias: the means of the still log, matching the firmware's
    ///   startup calibration (−0.148, −0.243, −0.767 °/s).
    /// - accel bias 0: not measurable from a log in one position (it would
    ///   need a 6-position calibration). The log only shows |a| = 1.019 g,
    ///   ≈ 18 mg of offset and scale error combined. At the balance point the
    ///   offsets are part of the mount tilt above.
    #[must_use]
    pub const fn prototype() -> Self {
        Self {
            height_above_axle_m: 0.10,
            mount_tilt_deg: -3.8,
            accel_noise_lsb: [13.0, 20.0, 18.0],
            gyro_noise_lsb: [8.0, 6.0, 6.0],
            accel_bias_lsb: [0.0; 3],
            gyro_bias_lsb: [-10.0, -15.0, -50.0],
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
        // Into the chip's frame, pitched forward by the mounting tilt.
        let (sin, cos) = p.mount_tilt_deg.to_radians().sin_cos();
        let chip_forward = forward * cos - up * sin;
        let chip_up = forward * sin + up * cos;
        let accel_true = [chip_up, 0.0, chip_forward].map(|a| a / G * ACCEL_LSB_PER_G);
        let pitch_rate_dps = state.pitch_rate_rad_s.to_degrees();
        let gyro_true = [0.0, -pitch_rate_dps * GYRO_LSB_PER_DPS, 0.0];

        let mut accel = [0; 3];
        let mut gyro = [0; 3];
        for axis in 0..3 {
            accel[axis] = quantise(
                accel_true[axis]
                    + p.accel_bias_lsb[axis]
                    + p.accel_noise_lsb[axis] * self.noise.sample(),
            );
            gyro[axis] = quantise(
                gyro_true[axis]
                    + p.gyro_bias_lsb[axis]
                    + p.gyro_noise_lsb[axis] * self.noise.sample(),
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
        assert_eq!(raw.accel, [8192, 0, 0]);
        assert_eq!(raw.gyro, [0, 0, 0]);
    }

    #[test]
    fn held_at_30_degrees_reads_gravity_rotated() {
        let raw = ideal(0.1).sample(&State::tilted_deg(30.0), 0.0, 0.0);
        // Up (X): cos 30° · 8192 = 7094.5 → 7094 after rounding.
        // Forward (Z): −sin 30° · 8192 = −4096.
        assert_eq!(raw.accel, [7094, 0, -4096]);
        let pitch = accel_pitch_deg(f32::from(raw.accel[2]), f32::from(raw.accel[0]));
        assert!((pitch - 30.0).abs() < 0.01, "{pitch}");
    }

    #[test]
    fn gyro_reads_minus_the_pitch_rate_on_y() {
        let raw = ideal(0.1).sample(&rotating(10.0), 0.0, 0.0);
        assert_eq!(raw.gyro, [0, -655, 0]);
    }

    #[test]
    fn mount_tilt_is_the_accel_pitch_when_balanced() {
        let params = ImuParams {
            mount_tilt_deg: -3.8,
            ..ImuParams::ideal(0.1)
        };
        let raw = Imu::new(params, 1).sample(&State::default(), 0.0, 0.0);
        let pitch = accel_pitch_deg(f32::from(raw.accel[2]), f32::from(raw.accel[0]));
        assert!((pitch + 3.8).abs() < 0.01, "{pitch}");
    }

    #[test]
    fn readings_clip_at_the_range() {
        let mut imu = ideal(0.0);
        assert_eq!(imu.sample(&rotating(-1000.0), 0.0, 0.0).gyro[1], 32767);
        assert_eq!(imu.sample(&rotating(1000.0), 0.0, 0.0).gyro[1], -32767);
    }

    #[test]
    fn axle_acceleration_shows_up_on_forward() {
        // Upright, held, axle speeding up at 0.5 g: forward (Z) reads +0.5 g.
        let raw = ideal(0.1).sample(&State::default(), 0.0, 0.5 * G);
        assert_eq!(raw.accel, [8192, 0, 4096]);
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
        // Square on the body, so the noise-free accel reading is exactly 1 g up.
        let params = ImuParams {
            mount_tilt_deg: 0.0,
            accel_bias_lsb: [30.0, -20.0, 60.0],
            ..ImuParams::prototype()
        };
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
        let check = |name: &str, values: Vec<f64>, expected_mean: f64, noise: f64| {
            let (mean, std) = stats(values);
            // Mean within ~4 standard errors (20 / √20000 ≈ 0.14 LSB at most).
            let tolerance = 4.0 * noise / f64::from(n).sqrt();
            assert!((mean - expected_mean).abs() < tolerance, "{name}: {mean}");
            // Rounding adds 1/12 LSB² of variance, negligible next to ≥ 36.
            assert!((std / noise - 1.0).abs() < 0.05, "{name}: {std}");
        };
        let one_g_up = [8192.0, 0.0, 0.0];
        for (axis, true_accel) in one_g_up.into_iter().enumerate() {
            check(
                "accel",
                samples.iter().map(|s| f64::from(s.accel[axis])).collect(),
                true_accel + params.accel_bias_lsb[axis],
                params.accel_noise_lsb[axis],
            );
            check(
                "gyro",
                samples.iter().map(|s| f64::from(s.gyro[axis])).collect(),
                params.gyro_bias_lsb[axis],
                params.gyro_noise_lsb[axis],
            );
        }
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
