//! The whole robot in a loop: physics → IMU → the firmware's controller →
//! motor → physics.
//!
//! Timing follows the firmware:
//!
//! - the physics runs in 0.1 ms steps;
//! - every 2 ms (500 Hz) the IMU is sampled and
//!   [`Controller::step`](balance_core::control::Controller::step) — the
//!   exact code the firmware runs — computes a new motor command;
//! - that command reaches the motors [`LATENCY_STEPS`] physics steps later
//!   (1 ms: the firmware's IMU read and maths take about that long), and is
//!   held until the next one.
//!
//! Optionally a [`Drive`] (deadband compensation and battery scaling, from
//! `balance-core`, the same code the firmware will run) sits between the
//! controller and the motor; without it the duty is the command. The drive
//! reads the motor's battery voltage exactly (the firmware's reading is
//! averaged and a little noisy, which doesn't matter here).
//!
//! Before it starts, the gyro is calibrated with the robot held upright and
//! still, like the firmware does at power-up. Then the robot is held at a
//! small tilt for one controller step and let go.

use balance_core::{
    control::{Config, Controller, Gains},
    drive::Drive,
    gyro::CalibrationError,
};

use crate::{
    imu::{Imu, ImuParams},
    log::{CALIBRATION_SAMPLES, DT_S, calibrate_bias},
    motor::Motor,
    pendulum::{Body, State},
};

/// Physics step in s.
pub const PHYSICS_DT_S: f64 = 1e-4;
/// Physics steps per controller step (2 ms / 0.1 ms).
pub const STEPS_PER_TICK: u32 = 20;
/// Physics steps between the IMU sample and the new command taking effect.
pub const LATENCY_STEPS: u32 = 10;
/// Pitch at which the robot counts as fallen, in degrees. The planned
/// safety cut-off turns the motors off there.
pub const FALLEN_DEG: f64 = 45.0;

/// The robot's state at one controller tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tick {
    /// Time since the robot was let go, in s.
    pub t_s: f64,
    /// True pitch in degrees (0 = balanced, positive = forward).
    pub pitch_deg: f64,
    /// The controller's estimate of it (complementary filter).
    pub filter_pitch_deg: f32,
    /// Motor command −1..=1 computed at this tick.
    pub motor_cmd: f32,
    /// Duty sent to the motor for it (the command after the [`Drive`], if
    /// any).
    pub duty: f32,
    /// Axle speed in m/s.
    pub axle_vel_m_s: f64,
    /// Axle position in m from where it was let go.
    pub axle_pos_m: f64,
}

/// One simulated robot.
#[derive(Debug, Clone)]
pub struct Sim {
    body: Body,
    motor: Motor,
    imu: Imu,
    controller: Controller,
    drive: Option<Drive>,
    state: State,
    /// Command the motors are currently getting.
    cmd: f64,
    /// Duties on their way to the motor, oldest first (dead time, see
    /// [`Sim::with_dead_time_ticks`]).
    in_flight: std::collections::VecDeque<f32>,
    dead_time_ticks: usize,
    ticks: u32,
}

impl Sim {
    /// Calibrates the gyro with the robot held upright, then lets it go at
    /// `start_pitch_deg`, with the prototype's controller settings but
    /// `gains`. `seed` picks the IMU noise.
    ///
    /// # Errors
    ///
    /// When the gyro calibration fails (only possible with absurd IMU
    /// noise).
    pub fn new(
        body: Body,
        motor: Motor,
        imu_params: ImuParams,
        gains: Gains,
        start_pitch_deg: f64,
        seed: u64,
    ) -> Result<Self, CalibrationError> {
        let mut imu = Imu::new(imu_params, seed);
        let held = State::default();
        let bias = calibrate_bias((0..CALIBRATION_SAMPLES).map(|_| imu.sample(&held, 0.0, 0.0)))?;
        let config = Config {
            gains,
            ..Config::PROTOTYPE
        };
        let mut controller = Controller::new(config, bias);
        // One sample while still held at the start tilt, so the filter starts
        // from a still accel reading, as in the firmware (its controller runs
        // from power-up on). Once let go, the accel pitch is misleading: the
        // chip is close to the centre of percussion and sees only ~1/5 of the
        // tilt while the robot starts to fall.
        let start = State::tilted_deg(start_pitch_deg);
        let _ = controller.step(&imu.sample(&start, 0.0, 0.0), DT_S);
        Ok(Self {
            body,
            motor,
            imu,
            controller,
            drive: None,
            state: start,
            cmd: 0.0,
            in_flight: std::collections::VecDeque::new(),
            dead_time_ticks: 0,
            ticks: 0,
        })
    }

    /// The prototype robot (motor with deadband) with `gains`, let go at
    /// `start_pitch_deg`.
    ///
    /// # Errors
    ///
    /// See [`Sim::new`].
    pub fn prototype(gains: Gains, start_pitch_deg: f64) -> Result<Self, CalibrationError> {
        Self::with_motor(Motor::prototype(), gains, start_pitch_deg)
    }

    /// The prototype robot, but with `motor` (e.g. [`Motor::ideal`]).
    ///
    /// # Errors
    ///
    /// See [`Sim::new`].
    pub fn with_motor(
        motor: Motor,
        gains: Gains,
        start_pitch_deg: f64,
    ) -> Result<Self, CalibrationError> {
        Self::new(
            Body::prototype(),
            motor,
            ImuParams::prototype(),
            gains,
            start_pitch_deg,
            1,
        )
    }

    /// The same robot with `drive` between the controller and the motor.
    #[must_use]
    pub fn with_drive(self, drive: Drive) -> Self {
        Self {
            drive: Some(drive),
            ..self
        }
    }

    /// The same robot with `ticks` controller periods (2 ms each) of dead
    /// time between the controller and the motor: a pure delay before the
    /// motor reacts at all (PWM period, current build-up, gear backlash),
    /// unlike the motor's lag, which starts reacting at once.
    #[must_use]
    pub fn with_dead_time_ticks(self, ticks: usize) -> Self {
        Self {
            dead_time_ticks: ticks,
            ..self
        }
    }

    /// Advances one controller period (2 ms).
    pub fn tick(&mut self) -> Tick {
        let axle_accel = self.motor.axle_accel(self.cmd, self.state.axle_vel_m_s);
        let pitch_accel = self.body.pitch_accel(&self.state, axle_accel);
        let raw = self.imu.sample(&self.state, pitch_accel, axle_accel);
        let out = self.controller.step(&raw, DT_S);
        #[allow(
            clippy::cast_possible_truncation,
            reason = "a battery voltage fits f32 easily"
        )]
        let battery_v = self.motor.battery_v as f32;
        let duty = self
            .drive
            .as_ref()
            .map_or(out.motor_cmd, |d| d.duty(out.motor_cmd, battery_v));
        let tick = Tick {
            t_s: f64::from(self.ticks) * f64::from(DT_S),
            pitch_deg: self.state.pitch_deg(),
            filter_pitch_deg: out.filter_pitch_deg,
            motor_cmd: out.motor_cmd,
            duty,
            axle_vel_m_s: self.state.axle_vel_m_s,
            axle_pos_m: self.state.axle_pos_m,
        };

        self.in_flight.push_back(duty);
        let arriving = if self.in_flight.len() > self.dead_time_ticks {
            self.in_flight.pop_front()
        } else {
            None
        };
        for step in 0..STEPS_PER_TICK {
            if step == LATENCY_STEPS
                && let Some(d) = arriving
            {
                self.cmd = f64::from(d);
            }
            let a = self.motor.axle_accel(self.cmd, self.state.axle_vel_m_s);
            self.state = self.state.step(&self.body, a, PHYSICS_DT_S);
        }
        self.ticks += 1;
        tick
    }

    /// Runs for up to `duration_s` or until the robot falls.
    pub fn run(&mut self, duration_s: f64) -> Run {
        let mut ticks = Vec::new();
        loop {
            let tick = self.tick();
            ticks.push(tick);
            if tick.pitch_deg.abs() > FALLEN_DEG {
                return Run {
                    ticks,
                    fell_at_s: Some(tick.t_s),
                };
            }
            if tick.t_s >= duration_s {
                return Run {
                    ticks,
                    fell_at_s: None,
                };
            }
        }
    }
}

/// The result of [`Sim::run`].
#[derive(Debug, Clone)]
pub struct Run {
    /// Every controller tick.
    pub ticks: Vec<Tick>,
    /// When the robot fell, or `None` if it stayed up.
    pub fell_at_s: Option<f64>,
}

impl Run {
    /// Largest |pitch| in degrees over the last `last_s` seconds.
    #[must_use]
    pub fn max_abs_pitch_deg(&self, last_s: f64) -> f64 {
        let end = self.ticks.last().map_or(0.0, |t| t.t_s);
        self.ticks
            .iter()
            .filter(|t| t.t_s >= end - last_s)
            .map(|t| t.pitch_deg.abs())
            .fold(0.0, f64::max)
    }

    /// Largest |axle speed| in m/s over the last `last_s` seconds.
    #[must_use]
    pub fn max_abs_speed_m_s(&self, last_s: f64) -> f64 {
        let end = self.ticks.last().map_or(0.0, |t| t.t_s);
        self.ticks
            .iter()
            .filter(|t| t.t_s >= end - last_s)
            .map(|t| t.axle_vel_m_s.abs())
            .fold(0.0, f64::max)
    }

    /// The last tick.
    #[must_use]
    pub fn last(&self) -> Option<&Tick> {
        self.ticks.last()
    }
}

#[cfg(test)]
mod tests {
    use balance_core::control::{Config, Gains};

    use super::Sim;
    use crate::motor::Motor;

    /// The gains the deadband findings (bring-up steps B1/B2) were measured
    /// with, before the first balance attempt halved kd.
    const B_STEP_GAINS: Gains = Gains {
        kp: 0.12,
        ki: 2.0,
        kd: 0.008,
        kv: 0.0,
    };

    /// With the ideal (linear) motor.
    fn run(kp: f32, ki: f32, kd: f32, seconds: f64) -> super::Run {
        Sim::with_motor(
            Motor::ideal(),
            Gains {
                kp,
                ki,
                kd,
                kv: 0.0,
            },
            2.0,
        )
        .unwrap()
        .run(seconds)
    }

    #[test]
    fn without_control_it_falls_like_the_open_loop_model() {
        // Gains 0: command 0, so the motors brake the wheels to standstill
        // (the axle stays put): 2° → 45° in τ·acosh(22.5) ≈ 0.125 · 3.8 s.
        let r = run(0.0, 0.0, 0.0, 2.0);
        let fell = r.fell_at_s.unwrap();
        assert!((fell - 0.48).abs() < 0.05, "{fell}");
    }

    #[test]
    fn p_alone_does_not_balance() {
        assert!(run(0.1, 0.0, 0.0, 10.0).fell_at_s.is_some());
    }

    #[test]
    fn pd_catches_the_fall_but_runs_away() {
        // PD holds it up for a while, then the slow unstable mode (the wheel
        // speed, which a tilt-only PD can't see) takes over.
        let r = run(0.1, 0.0, 0.008, 10.0);
        let fell = r.fell_at_s.unwrap();
        assert!(fell > 0.5, "{fell}");
    }

    #[test]
    fn prototype_pid_balances() {
        let r = Sim::with_motor(Motor::ideal(), Config::PROTOTYPE.gains, 2.0)
            .unwrap()
            .run(10.0);
        assert_eq!(r.fell_at_s, None);
        // Settled: within half a degree over the last 5 s.
        assert!(
            r.max_abs_pitch_deg(5.0) < 0.5,
            "{}",
            r.max_abs_pitch_deg(5.0)
        );
    }

    // With the deadband (`Motor::prototype`). Expected beforehand: the PID,
    // tuned on the ideal motor, falls or limit-cycles, because the commands
    // it needs (mostly < 0.2) are near or inside the deadband. Found instead:
    // it stays up (from 0.5-10°, 5 noise seeds, even with a 40 % start duty),
    // but in one of two ways that both show up as drift on the real robot.

    #[test]
    fn deadband_pid_stays_up_by_rolling_steadily() {
        // Usually it settles into a steady roll at a few cm/s: the wheels
        // never stand still, so only the (low) stop duty matters, and the I
        // term supplies that offset. Constant speed with the body upright is
        // an equilibrium of a tilt-only controller, and the deadband rules
        // out the speeds near 0. (B1 finding: with the B-step gains, no
        // speed feedback.)
        let gains = B_STEP_GAINS;
        let r = Sim::prototype(gains, 2.0).unwrap().run(10.0);
        assert_eq!(r.fell_at_s, None);
        assert!(
            r.max_abs_pitch_deg(5.0) < 0.1,
            "{}",
            r.max_abs_pitch_deg(5.0)
        );
        let slowest = r
            .ticks
            .iter()
            .filter(|t| t.t_s >= 5.0)
            .map(|t| t.axle_vel_m_s.abs())
            .fold(f64::INFINITY, f64::min);
        // Never stops in the last 5 s (seed 1: ~0.027 m/s, ~0.2 m in 10 s).
        assert!(slowest > 0.01, "{slowest}");
    }

    #[test]
    fn deadband_pid_can_stick_slip() {
        // From a small tilt it can instead fall into a stick-slip limit
        // cycle: the wheels stand until the growing command breaks them
        // loose, then turn briefly and stop again. Rare: of seeds 0-29 from
        // 0.5°, only seed 5 does it (8 stops in 1-7 s, pitch up to ±0.5°); the
        // others settle into the steady roll. Which seed depends on how the
        // noise lands, so it changed when the IMU axes were corrected (front
        // = closed side; seed 4 did it before). Measured with the B-step gains
        // (kd 0.008).
        let gains = B_STEP_GAINS;
        let mut sim = Sim::new(
            crate::pendulum::Body::prototype(),
            Motor::prototype(),
            crate::imu::ImuParams::prototype(),
            gains,
            0.5,
            5,
        )
        .unwrap();
        let r = sim.run(10.0);
        assert_eq!(r.fell_at_s, None);
        let window: Vec<_> = r
            .ticks
            .iter()
            .filter(|t| (1.0..7.0).contains(&t.t_s))
            .collect();
        let stops = window
            .windows(2)
            .filter(|w| w[0].axle_vel_m_s.abs() >= 0.005 && w[1].axle_vel_m_s.abs() < 0.005)
            .count();
        // 8 stops in 6 s.
        assert!((5..=12).contains(&stops), "{stops}");
        let swing = window.iter().map(|t| t.pitch_deg.abs()).fold(0.0, f64::max);
        assert!((0.2..1.0).contains(&swing), "{swing}");
    }

    // With the deadband compensation and battery scaling from `balance-core`
    // (`Drive::PROTOTYPE`) between the controller and the deadband motor.

    fn with_drive(start_deg: f64, seed: u64, battery_v: f64) -> super::Run {
        with_drive_gains(Config::PROTOTYPE.gains, start_deg, seed, battery_v)
    }

    fn with_drive_gains(gains: Gains, start_deg: f64, seed: u64, battery_v: f64) -> super::Run {
        Sim::new(
            crate::pendulum::Body::prototype(),
            Motor::prototype().at_battery(battery_v),
            crate::imu::ImuParams::prototype(),
            gains,
            start_deg,
            seed,
        )
        .unwrap()
        .with_drive(balance_core::drive::Drive::PROTOTYPE)
        .run(10.0)
    }

    #[test]
    fn compensated_stays_upright_for_10_s() {
        // The CI check that the whole chain balances: from small and large
        // tilts, on a full, nominal and nearly empty pack. With the
        // speed-feedback gains (D2) the robot settles near zero speed, where
        // the wheels stick-slip: seed 1 from 2° swings up to ±1.3° in the
        // last 5 s (`tune --check`: worst ~2°, mean 0.3° over 82 stress
        // runs). The softer gains trade that wobble for not running away on
        // a balance-point error. 2.5° means "clearly balancing".
        for (start, battery_v) in [(2.0, 7.4), (10.0, 7.4), (5.0, 8.4), (5.0, 6.4)] {
            let r = with_drive(start, 1, battery_v);
            assert_eq!(r.fell_at_s, None, "{start}° at {battery_v} V");
            let p = r.max_abs_pitch_deg(5.0);
            assert!(p < 2.5, "{start}° at {battery_v} V: {p}");
        }
    }

    #[test]
    fn compensation_reduces_the_drift_to_that_of_the_ideal_motor() {
        // Seed 1 from 2°: ideal motor ~0.09 m in 10 s, deadband ~0.19 m,
        // compensated ~0.10 m. Measured with the B-step gains (kd 0.008).
        let gains = B_STEP_GAINS;
        let pos = |r: &super::Run| r.last().unwrap().axle_pos_m.abs();
        let ideal = pos(&Sim::with_motor(Motor::ideal(), gains, 2.0)
            .unwrap()
            .run(10.0));
        let raw = pos(&Sim::prototype(gains, 2.0).unwrap().run(10.0));
        let comp = pos(&with_drive_gains(gains, 2.0, 1, 7.4));
        assert!(comp < 0.7 * raw, "{comp} vs {raw}");
        assert!((comp - ideal).abs() < 0.03, "{comp} vs {ideal}");
    }

    #[test]
    fn compensated_still_stick_slips_near_zero_speed() {
        // When the robot settles at (almost) zero speed, a standing wheel
        // needs the start duty, which the compensation doesn't give. From 2°,
        // 6 of seeds 0-19 end like this (after the IMU axes fix; the seeds
        // changed with it). Seed 4: 18 stops in the last 5 s, pitch up to
        // ±0.6°, but it stays up.
        let r = with_drive(2.0, 4, 7.4);
        assert_eq!(r.fell_at_s, None);
        let stops = r
            .ticks
            .windows(2)
            .filter(|w| {
                w[0].t_s >= 5.0
                    && w[0].axle_vel_m_s.abs() >= 0.005
                    && w[1].axle_vel_m_s.abs() < 0.005
            })
            .count();
        assert!((10..=25).contains(&stops), "{stops}");
        let swing = r.max_abs_pitch_deg(5.0);
        assert!((0.3..1.5).contains(&swing), "{swing}");
    }

    // Dead time and the first balance attempt (bring-up step C4).

    fn fast_motor(gains: Gains, dead_ms: usize) -> super::Run {
        // A motor that responds faster than the 50 ms estimate, plus dead
        // time: the model that reproduces the robot's fast shaking.
        let motor = Motor {
            time_constant_s: 0.030,
            ..Motor::prototype()
        }
        .at_battery(7.75);
        Sim::new(
            crate::pendulum::Body::prototype(),
            motor,
            crate::imu::ImuParams::prototype(),
            gains,
            4.6,
            1,
        )
        .unwrap()
        .with_drive(balance_core::drive::Drive::PROTOTYPE)
        .with_dead_time_ticks(dead_ms / 2)
        .run(10.0)
    }

    #[test]
    fn dead_time_delays_the_first_command() {
        // 10 ticks (20 ms) of dead time: the wheels don't move before.
        let mut sim = Sim::prototype(Config::PROTOTYPE.gains, 5.0)
            .unwrap()
            .with_dead_time_ticks(10);
        for _ in 0..10 {
            assert!(sim.tick().axle_vel_m_s.abs() < 1e-12);
        }
        let mut moved = false;
        for _ in 0..20 {
            moved |= sim.tick().axle_vel_m_s.abs() > 1e-6;
        }
        assert!(moved);
    }

    #[test]
    fn old_kd_shakes_with_a_faster_motor_the_new_one_does_not() {
        // The first balance attempt stood ~10 s but shook at ~8 Hz with the
        // command saturated ~30 % of the time; kd 0.008 alone gave more than
        // full command at the ±150 °/s it swung with. With a 30 ms motor lag
        // and 20 ms dead time the simulation shakes and falls with kd 0.008;
        // kd 0.004 (now in `Config::PROTOTYPE`) stays up and calm.
        assert!(fast_motor(B_STEP_GAINS, 20).fell_at_s.is_some());
        let new = fast_motor(Config::PROTOTYPE.gains, 20);
        assert_eq!(new.fell_at_s, None);
        assert!(new.max_abs_pitch_deg(5.0) < 3.0);
    }

    // Speed feedback (bring-up step D2).

    /// From 4.5° at 7.8 V with the chip's mounting tilt `offset_deg` off the
    /// balance point the controller assumes.
    fn with_balance_error(gains: Gains, offset_deg: f64) -> super::Run {
        let imu = crate::imu::ImuParams {
            mount_tilt_deg: crate::imu::ImuParams::prototype().mount_tilt_deg + offset_deg,
            ..crate::imu::ImuParams::prototype()
        };
        Sim::new(
            crate::pendulum::Body::prototype(),
            Motor::prototype().at_battery(7.8),
            imu,
            gains,
            4.5,
            1,
        )
        .unwrap()
        .with_drive(balance_core::drive::Drive::PROTOTYPE)
        .run(10.0)
    }

    #[test]
    fn without_speed_feedback_a_small_balance_error_runs_away() {
        // The second balance attempt: with a balance point off by half a
        // degree the I term keeps adding speed (it is the only thing that
        // knows about the speed) until the motors saturate at full speed and
        // the robot falls. 0.25° is enough.
        let no_kv = Gains {
            kv: 0.0,
            ..Config::PROTOTYPE.gains
        };
        for offset in [0.5, -0.5] {
            let r = with_balance_error(no_kv, offset);
            assert!(r.fell_at_s.is_some(), "{offset}°");
            let v = r.last().unwrap().axle_vel_m_s.abs();
            assert!(v > 0.35, "{offset}°: {v} m/s");
        }
        let r = with_balance_error(B_STEP_GAINS, 0.25);
        assert!(r.fell_at_s.is_some());
    }

    #[test]
    fn speed_feedback_turns_a_balance_error_into_a_steady_drift() {
        // With kv the I term can settle: pitch + kv · speed = offset, so the
        // robot drifts at a steady speed (~0.1 m/s per degree) and stays up.
        for offset in [1.0, -1.0] {
            let r = with_balance_error(Config::PROTOTYPE.gains, offset);
            assert_eq!(r.fell_at_s, None, "{offset}°");
            let v = r.max_abs_speed_m_s(5.0);
            assert!(v < 0.15, "{offset}°: {v} m/s");
        }
    }
}
