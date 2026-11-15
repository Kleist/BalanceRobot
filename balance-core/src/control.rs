//! One step of the balance controller: raw IMU sample in, motor command out.
//!
//! The firmware's 500 Hz loop and the simulator both call
//! [`Controller::step`], so the code tuned in simulation is exactly the code
//! that runs on the robot. Everything robot-specific (how the IMU is mounted,
//! the balance point, filter and PID settings) is in one [`Config`].
//!
//! Each step:
//!
//! 1. raw gyro LSB → °/s, minus the bias found at startup;
//! 2. raw accel → robot `(forward, up)` → accelerometer pitch, relative to
//!    the balance point;
//! 3. gyro-only pitch (for the log: shows the drift) and the complementary
//!    filter pitch (the estimate the controller uses);
//! 4. PID on the filter pitch → motor command in −1..=1.

use crate::{
    attitude::{ComplementaryFilter, accel_pitch_deg},
    clamp_duty,
    gyro::GyroBias,
    imu::RawSample,
    pid::Pid,
};

/// PID gains. Units: motor command (−1..=1) per degree of pitch.
///
/// Positive gains mean "tilted forward → drive forward": `kp = 0.05` gives
/// full power at 20° of tilt.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gains {
    /// Proportional: command per degree of tilt.
    pub kp: f32,
    /// Integral: command per degree·second of tilt.
    pub ki: f32,
    /// Derivative: command per degree/second of tipping rate.
    pub kd: f32,
}

/// Everything robot-specific the controller needs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// Gyro scale in LSB per °/s (65.5 at ±500 °/s).
    pub gyro_lsb_per_dps: f32,
    /// The accelerometer pitch at the robot's balance point (centre of mass
    /// above the axle), held by hand. Subtracted from the accelerometer
    /// pitch, so 0° = balanced.
    pub balance_pitch_deg: f32,
    /// Complementary filter time constant in s: changes faster than this
    /// come from the gyro, slower ones from the accelerometer.
    pub filter_tau_s: f32,
    /// Gyro calibration at startup: how many still samples to average.
    pub calibration_samples: u32,
    /// Gyro calibration: largest allowed max − min per axis in °/s; more
    /// means the robot moved and the calibration is repeated.
    pub calibration_max_spread_dps: f32,
    /// PID gains.
    pub gains: Gains,
}

impl Config {
    /// The prototype as measured so far.
    ///
    /// - gyro ±500 °/s: fast enough for a falling robot.
    /// - balance point +3.8°: the accel pitch with the robot held at its
    ///   balance point (the chip is mounted slightly tilted). Refine once the
    ///   robot balances. If it keeps
    ///   rolling forward while holding 0°, it is really leaning forward
    ///   there, so the true balance point is further back: lower this value
    ///   (and vice versa).
    /// - filter τ 0.5 s: a common starting point; tune later.
    /// - calibration 1000 samples (2 s at 500 Hz), spread ≤ 2 °/s: the gyro's
    ///   noise at rest is about 0.5 °/s peak to peak; moving the robot gives
    ///   tens of °/s.
    /// - gains kp 0.12, ki 2.0, kd 0.004: tuned in simulation (`sim` crate,
    ///   `tune` example) with an *ideal* motor, with kd 0.008. The first
    ///   balance attempt stood but shook at ~8 Hz with the command saturating
    ///   (kd 0.008 × the ±150 °/s it swung with is more than full command);
    ///   the simulation reproduces that with a faster motor (30 ms lag) plus
    ///   ~20 ms dead time, and kd halved to 0.004 stays calm there and on the
    ///   nominal model (`compare` example, `closed_loop` tests). The I term is
    ///   essential: with motors
    ///   that behave like speed sources (DC motors do), P or PD alone can't
    ///   balance, because ∫pitch is what tells the controller about the
    ///   wheel speed.
    pub const PROTOTYPE: Self = Self {
        gyro_lsb_per_dps: 65.5,
        balance_pitch_deg: 3.8,
        filter_tau_s: 0.5,
        calibration_samples: 1000,
        calibration_max_spread_dps: 2.0,
        gains: Gains {
            kp: 0.12,
            ki: 2.0,
            kd: 0.004,
        },
    };

    /// Raw gyro LSB `[x, y, z]` to °/s (bias not removed).
    #[must_use]
    pub fn gyro_dps(&self, raw: [i16; 3]) -> [f32; 3] {
        raw.map(|v| f32::from(v) / self.gyro_lsb_per_dps)
    }
}

/// Map the IMU's accelerometer X/Y/Z to the robot's `(forward, up)`.
///
/// The robot's **front is the closed side of the body**: the side the motors
/// drive towards for a positive command (checked on the robot, see
/// `docs/wiring.md`). The board stands upright on the chassis: **+X points up,
/// +Z backward (towards the open side), +Y to the left** (so the axle is Y).
/// Measured with the robot held still (1 g = 8192 LSB):
///
/// | Position | ax | ay | az |
/// |---|---|---|---|
/// | Upright (balance point) | +8169 | −147 | +540 |
/// | Tilted forward ~40° (closed side down) | +5950 | −40 | +5711 |
/// | Tilted backward ~40° (open side down) | +6011 | −196 | −5469 |
///
/// Upright, X reads +1 g, so +X is up. Tipping forward points −Z towards the
/// ground, so `az` goes *positive*: the forward axis is −Z, which makes the
/// forward reading go negative when tipping forward, as [`accel_pitch_deg`]
/// expects.
///
/// (The first axis check, 2026-11-11, took the open side as the front, so the
/// IMU and the motors disagreed: tilting forward drove the wheels backward.
/// Found in the wheels-in-the-air sign check.)
#[must_use]
pub fn robot_axes(accel: [i16; 3]) -> (f32, f32) {
    (-f32::from(accel[2]), f32::from(accel[0]))
}

/// The robot's pitch rate in °/s (positive = tipping forward) from the gyro's
/// `[x, y, z]` in °/s.
///
/// Pitching rotates the robot about its wheel axle, Y (see [`robot_axes`]).
/// Tipping forward turns −Z (forward) towards −X (down). By the right-hand
/// rule a positive rotation about +Y turns +Z towards +X, and so −Z towards
/// −X: tipping forward is a *positive* Y rate.
#[must_use]
pub const fn robot_pitch_rate(rates_dps: [f32; 3]) -> f32 {
    rates_dps[1]
}

/// What one [`Controller::step`] computed. All angles in degrees relative to
/// the balance point, positive = tilted forward.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Output {
    /// Pitch from the accelerometer alone.
    pub accel_pitch_deg: f32,
    /// Bias-corrected gyro rates `[x, y, z]` in °/s (IMU axes).
    pub rates_dps: [f32; 3],
    /// Pitch from integrating the gyro alone (starts at the first accel
    /// pitch, then drifts).
    pub gyro_pitch_deg: f32,
    /// Complementary-filter pitch: the estimate the PID acts on.
    pub filter_pitch_deg: f32,
    /// Motor command for both wheels, −1..=1, positive = drive forward.
    pub motor_cmd: f32,
}

/// The controller's state between steps.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Controller {
    config: Config,
    bias: GyroBias,
    gyro_pitch_deg: Option<f32>,
    filter: ComplementaryFilter,
    pid: Pid,
}

impl Controller {
    /// A controller with `config`, using the gyro `bias` from the startup
    /// calibration.
    #[must_use]
    pub const fn new(config: Config, bias: GyroBias) -> Self {
        let Gains { kp, ki, kd } = config.gains;
        Self {
            config,
            bias,
            gyro_pitch_deg: None,
            filter: ComplementaryFilter::new(config.filter_tau_s),
            pid: Pid::new(kp, ki, kd, -1.0, 1.0),
        }
    }

    /// Processes one IMU sample taken `dt_s` after the previous one.
    pub fn step(&mut self, raw: &RawSample, dt_s: f32) -> Output {
        let (forward, up) = robot_axes(raw.accel);
        let accel_pitch = accel_pitch_deg(forward, up) - self.config.balance_pitch_deg;
        let rates_dps = self.bias.apply(self.config.gyro_dps(raw.gyro));
        let pitch_rate = robot_pitch_rate(rates_dps);

        let gyro_pitch = self.gyro_pitch_deg.get_or_insert(accel_pitch);
        *gyro_pitch += pitch_rate * dt_s;
        let filter_pitch = self.filter.update(accel_pitch, pitch_rate, dt_s);

        // The PID pushes the measurement towards the setpoint (0° = balanced):
        // tilted forward, error = 0 − pitch < 0, so its output is negative.
        // The robot must drive *forward* then, to get the wheels back under
        // the centre of mass, hence the minus sign.
        let motor_cmd = clamp_duty(-self.pid.update(0.0, filter_pitch, dt_s));

        Output {
            accel_pitch_deg: accel_pitch,
            rates_dps,
            gyro_pitch_deg: *gyro_pitch,
            filter_pitch_deg: filter_pitch,
            motor_cmd,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Config, Controller, Gains, Output, robot_axes};
    use crate::{gyro::GyroBias, imu::RawSample};

    const DT: f32 = 0.002;

    /// A raw sample at `pitch_deg` from the balance point (accel only, 1 g),
    /// with gyro Y reading `gyro_y`.
    fn sample(pitch_deg: f32, gyro_y: i16) -> RawSample {
        let chip_pitch = (pitch_deg + Config::PROTOTYPE.balance_pitch_deg).to_radians();
        #[expect(
            clippy::cast_possible_truncation,
            reason = "at most 8192 in magnitude, fits i16"
        )]
        let lsb = |v: f32| libm::roundf(v * 8192.0) as i16;
        RawSample {
            // Tilted forward: forward (−Z) reads −sin, so Z reads +sin; up (X) reads cos.
            accel: [lsb(libm::cosf(chip_pitch)), 0, lsb(libm::sinf(chip_pitch))],
            gyro: [0, gyro_y, 0],
        }
    }

    fn with_gains(kp: f32, ki: f32, kd: f32) -> Controller {
        let config = Config {
            gains: Gains { kp, ki, kd },
            ..Config::PROTOTYPE
        };
        Controller::new(config, GyroBias([0.0; 3]))
    }

    fn assert_close(actual: f32, expected: f32, tol: f32) {
        assert!(
            (actual - expected).abs() <= tol,
            "{actual} is not within {tol} of {expected}"
        );
    }

    #[test]
    fn upright_chip_maps_to_up() {
        assert_eq!(robot_axes([8192, 5, -7]), (7.0, 8192.0));
    }

    #[test]
    fn pitch_is_relative_to_the_balance_point() {
        let out = with_gains(0.0, 0.0, 0.0).step(&sample(0.0, 0), DT);
        assert_close(out.accel_pitch_deg, 0.0, 0.01);
        let out = with_gains(0.0, 0.0, 0.0).step(&sample(10.0, 0), DT);
        assert_close(out.accel_pitch_deg, 10.0, 0.01);
        assert_close(out.filter_pitch_deg, 10.0, 0.01); // first step: accel pitch
        assert_close(out.gyro_pitch_deg, 10.0, 0.01);
    }

    #[test]
    fn bias_is_removed_and_plus_gyro_y_is_tipping_forward() {
        // Gyro Y reads +131 LSB = +2 °/s, of which +1 °/s is bias: +1 °/s on
        // Y is tipping forward at 1 °/s.
        let mut controller = Controller::new(Config::PROTOTYPE, GyroBias([0.0, 1.0, 0.0]));
        let mut out = controller.step(&sample(0.0, 131), DT);
        assert_close(out.rates_dps[1], 1.0, 1e-4);
        for _ in 0..499 {
            out = controller.step(&sample(0.0, 131), DT);
        }
        // 1 s at 1 °/s: the gyro pitch went up by 1°; the filter mostly
        // believes the accelerometer (0°) at this slow rate.
        assert_close(out.gyro_pitch_deg, 1.0, 0.01);
        assert!(
            out.filter_pitch_deg > 0.0 && out.filter_pitch_deg < 0.5,
            "{out:?}"
        );
    }

    #[test]
    fn zero_gains_give_no_command() {
        let out = with_gains(0.0, 0.0, 0.0).step(&sample(10.0, 0), DT);
        assert_close(out.motor_cmd, 0.0, 0.0);
    }

    #[test]
    fn tilted_forward_drives_forward() {
        let out = with_gains(0.05, 0.0, 0.0).step(&sample(10.0, 0), DT);
        assert_close(out.motor_cmd, 0.5, 0.001);
        let out = with_gains(0.05, 0.0, 0.0).step(&sample(-10.0, 0), DT);
        assert_close(out.motor_cmd, -0.5, 0.001);
    }

    #[test]
    fn command_is_limited_to_full_power() {
        let out = with_gains(1.0, 0.0, 0.0).step(&sample(30.0, 0), DT);
        assert_close(out.motor_cmd, 1.0, 0.0);
    }

    #[test]
    fn tipping_forward_faster_drives_forward_harder() {
        // Same tilt, but the second time it is tipping forward at 10 °/s
        // (gyro Y = +655): D adds to the command.
        let run = |gyro_y| {
            let mut controller = with_gains(0.0, 0.0, 0.01);
            let mut out: Option<Output> = None;
            for _ in 0..10 {
                out = Some(controller.step(&sample(0.0, gyro_y), DT));
            }
            out.map_or(f32::NAN, |o| o.motor_cmd)
        };
        assert_close(run(0), 0.0, 1e-4);
        assert!(run(655) > 0.0);
    }
}
