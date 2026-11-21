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
    /// Speed feedback: degrees of pitch per unit of estimated wheel speed
    /// (fraction of full speed, see [`Controller::step`]). The PID's P and I
    /// act on `pitch + kv · speed` instead of the pitch alone. 0 = off.
    ///
    /// Positive, the same sign as the pitch: to stop rolling forward the
    /// robot must first drive forward a little harder, so that it leans back
    /// and then brakes. By itself that is positive feedback (more speed →
    /// more command), stable only while `kp · kv < 1` (0.54 in
    /// [`Config::PROTOTYPE`]).
    pub kv: f32,
    /// Position feedback: degrees of pitch per unit of estimated distance
    /// travelled since the motors came on (in "full-speed seconds": the
    /// integral of the speed estimate, so 1 unit ≈ 0.41 m on the prototype).
    /// P and I act on `pitch + kv · speed + kx · position`. 0 = off.
    ///
    /// Why it holds the position: the I term only settles once its input is
    /// 0 on average. Without `kx`, a balance-point error ε then needs
    /// `kv · speed = −ε`: a steady drift. With `kx`, standing still at
    /// `position = −ε / kx` also satisfies it, so the robot stops a short way
    /// from where it was let go instead of rolling on.
    pub kx: f32,
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
    /// Time constant in s of the wheel-speed estimate: how fast the wheels
    /// follow the command (the motor's lag).
    pub speed_tau_s: f32,
    /// Largest lean in degrees the position feedback may ask for (|kx ·
    /// position| is clamped to this, and the position estimate stops growing
    /// there). After a big push, or when the estimate has drifted from the
    /// real position, the robot then leans back by at most this much
    /// instead of an angle it can't hold.
    pub max_position_lean_deg: f32,
}

impl Config {
    /// The prototype as measured so far.
    ///
    /// - gyro ±500 °/s: fast enough for a falling robot.
    /// - balance point −4.2°: the accel pitch with the robot held still at its
    ///   balance point (2026-11-22: filter pitch −8.0 ± 0.1° with the old
    ///   +3.8°). The first hand measurement (2026-11-11, raw az +540 → +3.8°
    ///   after the axes fix) was 8° off; with it every balance attempt ran
    ///   away forward. If it keeps
    ///   rolling forward while holding 0°, it is really leaning forward
    ///   there, so the true balance point is further back: lower this value
    ///   (and vice versa).
    /// - filter τ 0.5 s: a common starting point; tune later.
    /// - calibration 1000 samples (2 s at 500 Hz), spread ≤ 2 °/s: the gyro's
    ///   noise at rest is about 0.5 °/s peak to peak; moving the robot gives
    ///   tens of °/s.
    /// - gains kp 0.09, ki 0.5, kd 0.003, kv 6 (bring-up step D2, `tune`
    ///   example): history: kp 0.12 / ki 2.0 / kd 0.008 were tuned on an
    ///   ideal motor; the first balance attempt shook at ~8 Hz (D term
    ///   saturating), so kd was halved; the second showed that without speed
    ///   feedback a balance point off by a fraction of a degree makes the I
    ///   term add speed until the motors saturate and the robot falls (in
    ///   simulation 0.25° is enough). The current set survives all 82 stress
    ///   runs of `tune --check` (balance error up to ±1.5°, motor lag 15-80 ms
    ///   with up to 30 ms dead time, ±30 % speed, 6.6 V), drifting at most
    ///   ~0.1 m/s with a 1° error; so do all its neighbours (kp 0.07-0.11,
    ///   ki 0.35-0.75, kd 0.002-0.004, kv 5-7). kd 0.005 already fails.
    /// - position feedback kx 4 (D2, position hold): with a 1° balance error
    ///   the simulated robot stops ~0.1 m from the start instead of drifting
    ///   ~7 cm/s (1.4 m in 20 s); `tune --check 0.09 0.5 0.003 6 4`: no falls
    ///   in 82 stress runs, worst 0.17 m from the start (mean 0.11; kx 0:
    ///   1.04 m), but the robot rocks around its spot (worst pitch over the
    ///   last 5 s ~1.7° on average instead of 0.3°). kx 6-8 hold tighter
    ///   (0.10-0.13 m) but more neighbouring gain sets fall; with kx 4 only
    ///   ki ±30 % do (1 of 82 runs each).
    /// - position lean clamped to 3° (`max_position_lean_deg`).
    /// - speed estimate τ 50 ms: the motor model's lag (estimated).
    pub const PROTOTYPE: Self = Self {
        gyro_lsb_per_dps: 65.5,
        balance_pitch_deg: -4.2,
        filter_tau_s: 0.5,
        calibration_samples: 1000,
        calibration_max_spread_dps: 2.0,
        gains: Gains {
            kp: 0.09,
            ki: 0.5,
            kd: 0.003,
            kv: 6.0,
            kx: 4.0,
        },
        speed_tau_s: 0.05,
        max_position_lean_deg: 3.0,
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
    /// Estimated wheel speed as a fraction of full speed, positive =
    /// forward: the command through a first-order lag (no encoders).
    pub speed_est: f32,
    /// Estimated distance travelled since the motors came on, in full-speed
    /// seconds (the integral of `speed_est`; clamped, see
    /// [`Config::max_position_lean_deg`]).
    pub pos_est: f32,
}

/// The controller's state between steps.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Controller {
    config: Config,
    bias: GyroBias,
    gyro_pitch_deg: Option<f32>,
    filter: ComplementaryFilter,
    pid: Pid,
    speed_est: f32,
    pos_est: f32,
}

impl Controller {
    /// A controller with `config`, using the gyro `bias` from the startup
    /// calibration.
    #[must_use]
    pub const fn new(config: Config, bias: GyroBias) -> Self {
        let Gains { kp, ki, kd, .. } = config.gains;
        Self {
            config,
            bias,
            gyro_pitch_deg: None,
            filter: ComplementaryFilter::new(config.filter_tau_s),
            pid: Pid::new(kp, ki, kd, -1.0, 1.0),
            speed_est: 0.0,
            pos_est: 0.0,
        }
    }

    /// Forgets the PID's I term and D history and sets the speed and position
    /// estimates to 0 (the wheels stand while the motors are off, and the
    /// position is held relative to where the robot is let go); the pitch
    /// estimates keep running.
    ///
    /// Call it while the motors are off (fallen, held, cut off): the pitch
    /// isn't 0 then and nothing corrects it, so the I term would wind up to
    /// full command within seconds (ki 2.0 at 2° reaches 1.0 in 0.25 s), and
    /// the motors would start at full power the moment they are allowed again.
    pub fn reset_pid(&mut self) {
        self.pid.reset();
        self.speed_est = 0.0;
        self.pos_est = 0.0;
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
        //
        // Speed feedback moves the setpoint to −kv · speed, so P and I act on
        // pitch + kv · speed (D stays on the pitch alone, it differentiates
        // the measurement). Without it the I term is the only thing that
        // knows about the speed, and any offset in the measured pitch (a
        // balance point off by a quarter degree, a sloped floor) makes it add
        // speed until the motors saturate and the robot falls. With it, an
        // offset only gives a steady drift.
        //
        // Position feedback adds kx · position: the I term can then settle
        // with the robot standing still (see `Gains::kx`).
        let Gains { kv, kx, .. } = self.config.gains;
        let max_lean = self.config.max_position_lean_deg;
        let position_lean = (kx * self.pos_est).clamp(-max_lean, max_lean);
        let setpoint = -kv * self.speed_est - position_lean;
        let motor_cmd = clamp_duty(-self.pid.update(setpoint, filter_pitch, dt_s));

        // The motors behave like speed sources and the deadband compensation
        // makes the duty roughly proportional to the speed, so the wheels
        // follow the command with the motor's lag.
        self.speed_est += (motor_cmd - self.speed_est) * dt_s / self.config.speed_tau_s;
        self.pos_est += self.speed_est * dt_s;
        // Stop the estimate where the lean it asks for is clamped, so it
        // comes back within seconds once the robot rolls back (like
        // anti-windup for the I term).
        if kx > 0.0 {
            let max_pos = max_lean / kx;
            self.pos_est = self.pos_est.clamp(-max_pos, max_pos);
        }

        Output {
            accel_pitch_deg: accel_pitch,
            rates_dps,
            gyro_pitch_deg: *gyro_pitch,
            filter_pitch_deg: filter_pitch,
            motor_cmd,
            speed_est: self.speed_est,
            pos_est: self.pos_est,
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
            gains: Gains {
                kp,
                ki,
                kd,
                kv: 0.0,
                kx: 0.0,
            },
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
    fn reset_pid_forgets_the_wound_up_i_term() {
        // I only: held at 2° for 2 s, the I term winds up to (nearly) full
        // command; anti-windup stops it one step short of the 1.0 limit.
        let mut controller = with_gains(0.0, 2.0, 0.0);
        let mut out = controller.step(&sample(2.0, 0), DT);
        for _ in 0..1000 {
            out = controller.step(&sample(2.0, 0), DT);
        }
        assert!(out.motor_cmd > 0.98, "{out:?}");
        // After a reset the first step outputs 0 (no history yet), then the
        // I term builds up again from scratch: ki · pitch · dt per step.
        controller.reset_pid();
        out = controller.step(&sample(2.0, 0), DT);
        assert_close(out.motor_cmd, 0.0, 1e-6);
        out = controller.step(&sample(2.0, 0), DT);
        assert_close(out.motor_cmd, 2.0 * 2.0 * DT, 1e-3);
        // The pitch estimate is not reset.
        assert_close(out.filter_pitch_deg, 2.0, 0.01);
    }

    #[test]
    fn speed_estimate_follows_the_command_with_the_motor_lag() {
        // P only, held at 2°: command 0.18. After one time constant (50 ms)
        // the estimate has covered 1 − 1/e ≈ 63 % of the way.
        let mut controller = with_gains(0.09, 0.0, 0.0);
        let mut out = controller.step(&sample(2.0, 0), DT);
        for _ in 1..25 {
            out = controller.step(&sample(2.0, 0), DT);
        }
        assert_close(out.motor_cmd, 0.18, 1e-3);
        assert_close(out.speed_est, 0.18 * (1.0 - (-1.0_f32).exp()), 0.005);
        // A reset sets it back to 0 (the wheels stand while the motors are off).
        controller.reset_pid();
        assert_close(controller.step(&sample(2.0, 0), DT).speed_est, 0.0, 0.01);
    }

    #[test]
    fn speed_feedback_adds_kv_times_speed_to_the_pitch() {
        // P only with kv 6: once the estimate has settled at the command c,
        // c = kp · (pitch + kv · c), so c = kp · pitch / (1 − kp · kv).
        let mut controller = Controller::new(
            Config {
                gains: Gains {
                    kp: 0.05,
                    ki: 0.0,
                    kd: 0.0,
                    kv: 6.0,
                    kx: 0.0,
                },
                ..Config::PROTOTYPE
            },
            GyroBias([0.0; 3]),
        );
        let mut out = controller.step(&sample(1.0, 0), DT);
        for _ in 0..2000 {
            out = controller.step(&sample(1.0, 0), DT);
        }
        assert_close(out.motor_cmd, 0.05 / (1.0 - 0.05 * 6.0), 2e-3);
    }

    #[test]
    fn position_estimate_integrates_the_speed_and_resets() {
        // P only, held at 2°: the speed estimate settles at the command, and
        // the position estimate grows by speed · dt every step.
        let mut controller = with_gains(0.09, 0.0, 0.0);
        let mut prev = controller.step(&sample(2.0, 0), DT);
        for _ in 0..500 {
            let out = controller.step(&sample(2.0, 0), DT);
            assert_close(out.pos_est - prev.pos_est, out.speed_est * DT, 1e-6);
            prev = out;
        }
        assert!(prev.pos_est > 0.15, "{prev:?}");
        controller.reset_pid();
        assert_close(controller.step(&sample(2.0, 0), DT).pos_est, 0.0, 1e-3);
    }

    #[test]
    fn position_feedback_asks_for_at_most_the_max_lean() {
        // kx 20 with the robot driven forward for a long time: the position
        // estimate stops at max_position_lean_deg / kx, so the lean it asks
        // for stays at 3°.
        let config = Config {
            gains: Gains {
                kp: 0.09,
                ki: 0.0,
                kd: 0.0,
                kv: 0.0,
                kx: 20.0,
            },
            ..Config::PROTOTYPE
        };
        let mut controller = Controller::new(config, GyroBias([0.0; 3]));
        let mut out = controller.step(&sample(5.0, 0), DT);
        for _ in 0..5000 {
            out = controller.step(&sample(5.0, 0), DT);
        }
        assert_close(out.pos_est, 3.0 / 20.0, 1e-4);
        // P acts on pitch + 3°: 0.09 · (5 + 3).
        assert_close(out.motor_cmd, 0.09 * 8.0, 0.01);
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
