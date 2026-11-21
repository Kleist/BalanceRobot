//! The firmware as a library: [`run`] does everything, for the [`Experiment`] a binary in
//! `src/bin` picks (one per experiment, launched with a cargo alias such as `cargo
//! balance`, see `.cargo/config.toml`), so no constant has to be edited to switch.
//!
//! Read raw accelerometer and gyro samples from the onboard ICM-42670-P and log them
//! over USB-Serial-JTAG as CSV lines, followed by the pitch angle computed from the
//! accelerometer alone, the bias-corrected gyro rates, the pitch from integrating the gyro
//! alone (which drifts) and the complementary-filter pitch that combines both. The IMU is
//! read at 500 Hz, the rate the control loop will run at, but only every
//! `LOG_EVERY_N_SAMPLES`th sample is logged, so the log stays readable. Once a second a
//! `# loop max us: ...` line shows how much of the 2 ms period the loop needed, how late
//! its ticks started, and which part took how long.
//!
//! The sample loop doesn't print: formatting and sending a line takes longer than a whole
//! 2 ms period. It puts the raw numbers into the `LOG` queue instead (`LogMsg`), and the
//! `log_writer` task formats them and sends them with the async USB-Serial-JTAG driver.
//! Setup messages before the loop still use `println!`.
//!
//! The `icm42670` driver only sets the chip up. After that each sample is one burst read
//! of all 12 data bytes (see `read_imu`); the time both ways take is logged at startup.
//!
//! The battery voltage (divider on GPIO4) is logged at startup and then every 0.5 s as
//! `# battery ...` lines, also during the motor test, so the sag under load shows up.
//!
//! At startup the gyro bias is calibrated while the robot stands still.
//! Each sample then goes through `balance_core::control::Controller` (the same code the
//! simulator runs), which computes the pitch estimates and a motor command. In the
//! sign-check and balance experiments the command drives both motors (while the safety cut-offs allow it
//! and no motor test or direction check runs), through the deadband compensation and
//! battery scaling of `balance_core::drive`; otherwise the motors stay off. While the
//! motors are off the PID is held reset, so it starts from scratch when they come on.
//!
//! Runs on two Embassy executors sharing the one CPU core:
//! - the normal (thread-mode) executor runs `main` (setup and calibration), then the `blink`
//!   task (LED on GPIO7, once a second) and the `log_writer` task. They take turns at each
//!   `.await`.
//! - an interrupt executor at priority 2 runs `control_loop`, the 500 Hz loop. It runs
//!   inside an interrupt, so its tick preempts the thread-mode tasks even in the middle of
//!   formatting a log line.
//!
//! Deadband test ([`Experiment::DeadbandTest`]), after calibration: each motor in turn,
//! forward then backward, ramps the PWM duty up from 20 % to 50 % and back down to 10 % in
//! 2 % steps (0.5 s each), then
//! stops for 1 s (see `motor_test_step`). Watch the wheel and note at which logged duty it
//! starts turning on the way up and stops on the way down. The sequence runs twice, then
//! both motors stay off. The IMU CSV is paused during the test, so the `# motor ...` lines
//! are easy to follow. Keep the wheels off the ground.
//!
//! Safety cut-offs (`balance_core::safety`), checked after every IMU read: the motors may
//! only run while the robot is upright (off past ±45°, back on once held within ±5° and
//! still, below 20 °/s, for 0.2 s; off at power-up until held like that once) and the battery isn't empty (warning below 6.6 V,
//! off below 6.4 V for good, each after 1 s below the threshold). Every change is logged
//! as a `# safety: ...` line. The motor test and direction check below obey them, so hold
//! the robot upright (wheels in the air) while they run.
//!
//! Direction check ([`Experiment::MotorCheck`]), after calibration: both motors get
//! `Motor::set(+0.5)` for 2 s, stop for 2 s, `set(-0.5)` for 2 s, then stay off. `+` must
//! turn both wheels the way that drives the robot forward.

#![no_std]

use balance_core::{
    battery::Divider,
    control::{Config, Controller, Gains, Output as ControlOutput, robot_pitch_rate},
    drive::Drive,
    gyro::{BiasCalibrator, GyroBias},
    imu::{DATA_LEN, DATA_START_REGISTER, RawSample},
    safety::{Limits, Safety, Status},
    split_command,
};
use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel as Queue};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::Write as _;
use esp_backtrace as _;
use esp_hal::{
    Async, Blocking,
    analog::adc::{Adc, AdcCalCurve, AdcConfig, AdcPin, Attenuation},
    gpio::{AnyPin, DriveMode, Level, Output, OutputConfig, OutputPin},
    i2c::master::{Config as I2cConfig, Error as I2cError, I2c},
    interrupt::Priority,
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, Channel, ChannelIFace},
        timer::{self, TimerIFace},
    },
    peripherals::{ADC1, GPIO4, LEDC},
    time::Rate,
    timer::timg::TimerGroup,
    usb::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagTx},
};
use esp_println::println;
use esp_rtos::embassy::InterruptExecutor;
use icm42670::{AccelOdr, AccelRange, Address, GyroOdr, GyroRange, Icm42670, prelude::*};
use static_cell::StaticCell;

/// ±4 g: a balancing robot stays well below that, and it keeps 8192 LSB per g.
const ACCEL_RANGE: AccelRange = AccelRange::G4;
/// ±500 °/s: fast enough for a falling robot, 65.5 LSB per °/s.
const GYRO_RANGE: GyroRange = GyroRange::Deg500;
/// I2C address of the onboard ICM-42670-P (AD0 tied low).
const IMU_ADDRESS: u8 = 0x68;
/// The IMU measures at 1.6 kHz, faster than we read it (500 Hz), so every read gets a
/// fresh sample at most 0.6 ms old. 1.6 kHz needs low-noise mode, which the driver
/// switches on.
const ACCEL_ODR: AccelOdr = AccelOdr::Hz1600;
const GYRO_ODR: GyroOdr = GyroOdr::Hz1600;
/// 500 Hz: the rate the balance control loop will run at. Fast enough to catch a falling
/// robot early, slow enough to leave time for I2C, maths and logging in each period.
const SAMPLE_PERIOD: Duration = Duration::from_micros(2000);
/// `SAMPLE_PERIOD` in seconds, for integrating the gyro rate.
const DT_S: f32 = 0.002;
/// Log one CSV line per this many samples: 10 = 50 lines/s, enough to follow a fall
/// (τ ≈ 0.125 s) and a ~2.5 Hz wobble when comparing with the simulation (bring-up step
/// C4). Each line takes the log task ~2 ms to format and send, so 50 lines/s keep it busy
/// ~10 % of the time; check that `dropped` in the `# loop max` lines stays 0. Set to 1 to
/// log every sample (500 lines/s; more than the USB log keeps up with).
const LOG_EVERY_N_SAMPLES: u32 = 10;
/// Log the loop's busy time once per this many samples: 500 = once a second.
const LOOP_STATS_EVERY_N_SAMPLES: u32 = 500;
/// Battery sense divider: battery + → 100 kΩ → GPIO4 → 33 kΩ → GND.
const BATTERY_DIVIDER: Divider = Divider {
    top_ohm: 100_000.0,
    bottom_ohm: 33_000.0,
};
/// ADC readings averaged per battery measurement, to smooth out ADC noise. The loop gets
/// one reading per two iterations (4 ms), so a new average comes every 16 × 4 ms = 64 ms.
const BATTERY_AVERAGE: u32 = 16;
/// Log the battery voltage once per this many samples: 250 = every 0.5 s.
const BATTERY_LOG_EVERY_N_SAMPLES: u32 = 250;

/// PWM frequency for the L298N enable pins. 100 Hz: the TT motors start at a lower duty
/// and keep turning down to a lower duty than at 1 kHz (see "Known limitations" in
/// CLAUDE.md), probably because each pulse is long enough to build up current and torque
/// despite the motor's inductance and the L298N's slow edges. The price is an audible
/// low whine and a ripple at 100 Hz (the control loop runs at 500 Hz, so a command is
/// updated several times per PWM period). The LEDC timer needs a clock divider in range:
/// at 10-bit duty the lowest possible frequency is about 78 Hz. To compare frequencies,
/// change this, reflash and rerun the deadband test (`cargo deadband-test`); the log notes
/// the frequency of each run.
const MOTOR_PWM_FREQUENCY: Rate = Rate::from_hz(100);

/// Motor test: how long each duty step lasts. Long enough to see whether the wheel
/// turns (a TT motor spins up in well under 0.1 s).
const MOTOR_TEST_STEP_MS: u64 = 500;
/// Each ramp goes up from `START` to `PEAK`, then down to `END`, in `STEP` increments.
/// The TT motors start at about 40 %, so the ramp brackets that with some margin. Going
/// back down shows where a turning wheel stops, which is usually lower than where a
/// standing wheel starts (static friction is higher than sliding friction).
const RAMP_START_PCT: u8 = 20;
const RAMP_PEAK_PCT: u8 = 50;
const RAMP_END_PCT: u8 = 10;
const RAMP_STEP_PCT: u8 = 2;
/// Stopped steps after each ramp. The stop means a motor never reverses while turning,
/// which would draw a large current spike and could brown out the battery.
const MOTOR_TEST_STOP_STEPS: u64 = 2;
/// The ramps in one pass of the test.
const MOTOR_TEST_RAMPS: [(Side, Direction); 4] = [
    (Side::Left, Direction::Forward),
    (Side::Left, Direction::Backward),
    (Side::Right, Direction::Forward),
    (Side::Right, Direction::Backward),
];
/// Passes through `MOTOR_TEST_RAMPS` before the test ends and the motors stay off.
const MOTOR_TEST_PASSES: u64 = 2;
/// Direction check: after calibration both motors get `+MOTOR_CHECK_CMD` for
/// `MOTOR_CHECK_STEP_MS`, stop for the same time, run at `-MOTOR_CHECK_CMD`, then stay off.
/// With the wheels in the air, + must turn both wheels the way that drives the robot
/// forward.
const MOTOR_CHECK_CMD: f32 = 0.5;
const MOTOR_CHECK_STEP_MS: u64 = 2000;

/// What the firmware does after setup and calibration. Each binary in `src/bin` runs one
/// (launch with the cargo aliases in `.cargo/config.toml`, e.g. `cargo balance`), so
/// switching experiments never needs a code change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Experiment {
    /// Log the IMU, pitch estimates and the controller's command; the motors stay off.
    ImuLog,
    /// Deadband ramps, one motor and direction at a time (wheels in the air).
    DeadbandTest,
    /// Both motors +0.5, stop, −0.5, then off (wheels in the air): checks the direction.
    MotorCheck,
    /// The controller drives the motors with the P-only `SIGN_CHECK_GAINS` (wheels in the
    /// air): tilting forward must spin the wheels forward.
    SignCheck,
    /// The controller drives the motors with `Config::PROTOTYPE.gains` (on the floor,
    /// hand ready).
    Balance,
}

impl Experiment {
    /// Whether the controller's command is applied to the motors. Only in the sign check
    /// and the balance attempt, so any other experiment never drives the wheels from the
    /// controller by surprise.
    const fn control_drives_motors(self) -> bool {
        matches!(self, Self::SignCheck | Self::Balance)
    }

    /// The gains the controller runs with. The balance gains are wrong for the sign check:
    /// held tilted with the wheels in the air, their I term winds up and the wheels race.
    const fn gains(self) -> Gains {
        if matches!(self, Self::SignCheck) {
            SIGN_CHECK_GAINS
        } else {
            Config::PROTOTYPE.gains
        }
    }
}

/// Gains for the sign check (wheels in the air): P only, so the wheel speed simply follows
/// the tilt and nothing builds up over time. kp 0.03 per degree gives a command of 0.3 at
/// 10° and 0.45 at 15°, clearly above the 100 Hz start duty (16-22 %), so the wheels start
/// at about 6-7° of tilt and keep turning down to about 3° (stop duty 8 %). Full power
/// would need 33°, so the wheels never race. Without I and D a robot on the floor would
/// run away (see "Control pipeline" in CLAUDE.md): this is not for balancing.
const SIGN_CHECK_GAINS: Gains = Gains {
    kp: 0.03,
    ki: 0.0,
    kd: 0.0,
    kv: 0.0,
    kx: 0.0,
};

/// The direction check's command for both motors, by time since the start of the check.
/// 0 while the safety cut-offs don't allow the motors to run.
fn motor_check_cmd(elapsed_ms: u64, motors_allowed: bool) -> f32 {
    if !motors_allowed {
        return 0.0;
    }
    match elapsed_ms / MOTOR_CHECK_STEP_MS {
        0 => MOTOR_CHECK_CMD,
        2 => -MOTOR_CHECK_CMD,
        _ => 0.0,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Side {
    Left,
    Right,
}

/// "Forward" is just IN1 high / IN2 low (IN3 / IN4 for the right motor). Whether that
/// drives the robot forward depends on how each motor is wired to the OUT pins: check it
/// on the robot and swap the motor wires if needed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Direction {
    Forward,
    Backward,
}

/// Whether the duty is going up or down in the current ramp.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ramp {
    Up,
    Down,
}

/// What the motor test does during one step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TestStep {
    /// Run one motor in one direction at this duty cycle (percent); the other is off.
    Run(Side, Direction, Ramp, u8),
    /// Both motors off, between ramps.
    Stop,
    /// The test is over: both motors off for good.
    Done,
}

/// The motor test as a function of the step number (one step per `MOTOR_TEST_STEP_MS`).
fn motor_test_step(step: u64) -> TestStep {
    let up_steps = u64::from((RAMP_PEAK_PCT - RAMP_START_PCT) / RAMP_STEP_PCT) + 1;
    let down_steps = u64::from((RAMP_PEAK_PCT - RAMP_END_PCT) / RAMP_STEP_PCT);
    let steps_per_ramp = up_steps + down_steps + MOTOR_TEST_STOP_STEPS;
    let ramps_per_pass = MOTOR_TEST_RAMPS.len() as u64;
    let ramp = step / steps_per_ramp;
    if ramp >= ramps_per_pass * MOTOR_TEST_PASSES {
        return TestStep::Done;
    }
    let within_ramp = step % steps_per_ramp;
    // All conversions below are of small numbers (< 4, <= 100), so they can't fail.
    let (ramp_dir, duty_pct) = if within_ramp < up_steps {
        let offset = u8::try_from(within_ramp).unwrap_or(0) * RAMP_STEP_PCT;
        (Ramp::Up, RAMP_START_PCT + offset)
    } else if within_ramp < up_steps + down_steps {
        let offset = u8::try_from(within_ramp - up_steps + 1).unwrap_or(0) * RAMP_STEP_PCT;
        (Ramp::Down, RAMP_PEAK_PCT - offset)
    } else {
        return TestStep::Stop;
    };
    let (side, direction) = usize::try_from(ramp % ramps_per_pass)
        .ok()
        .and_then(|i| MOTOR_TEST_RAMPS.get(i).copied())
        .unwrap_or((Side::Left, Direction::Forward));
    TestStep::Run(side, direction, ramp_dir, duty_pct)
}

/// One L298N channel: PWM on EN sets the speed (the duty cycle is the fraction of time
/// the outputs are on), the two IN pins pick the direction.
struct Motor<'d> {
    en: Channel<'d, LowSpeed>,
    in_a: Output<'d>,
    in_b: Output<'d>,
    /// The duty last applied, in signed percent (positive = forward, 0 = stopped), for
    /// the log.
    duty_pct: i8,
}

impl Motor<'_> {
    fn run(&mut self, direction: Direction, duty_pct: u8) -> Result<(), channel::Error> {
        // Switch off while changing the IN pins, so the outputs never see a half-changed
        // combination.
        self.en.set_duty(0)?;
        let forward = direction == Direction::Forward;
        self.in_a.set_level(Level::from(forward));
        self.in_b.set_level(Level::from(!forward));
        self.duty_pct = 0;
        self.en.set_duty(duty_pct)?;
        // `duty_pct` is at most 100 (the LEDC driver rejects more), so it fits an i8.
        let signed = i8::try_from(duty_pct).unwrap_or(i8::MAX);
        self.duty_pct = if forward { signed } else { -signed };
        Ok(())
    }

    /// Signed command, `-1.0..=1.0`: positive = forward, 0 = stop. Out-of-range values are
    /// clamped, NaN stops the motor (see `balance_core::split_command`).
    ///
    /// Called every 2 ms by the control loop, so it only touches the hardware when the
    /// duty changes, and only goes through `run` (EN off while the IN pins switch) when
    /// the direction changes. A duty change in the same direction is a single LEDC write.
    fn set(&mut self, cmd: f32) -> Result<(), channel::Error> {
        let (forward, duty_pct) = split_command(cmd);
        // At most 100 (see `split_command`), so it fits an i8.
        let magnitude = i8::try_from(duty_pct).unwrap_or(i8::MAX);
        let signed = if forward { magnitude } else { -magnitude };
        if signed == self.duty_pct {
            return Ok(());
        }
        if duty_pct == 0 {
            return self.stop();
        }
        if self.duty_pct != 0 && (self.duty_pct > 0) == forward {
            self.en.set_duty(duty_pct)?;
            self.duty_pct = signed;
            return Ok(());
        }
        let direction = if forward {
            Direction::Forward
        } else {
            Direction::Backward
        };
        self.run(direction, duty_pct)
    }

    /// EN low: both outputs off, the motor coasts to a stop.
    fn stop(&mut self) -> Result<(), channel::Error> {
        self.en.set_duty(0)?;
        self.duty_pct = 0;
        self.in_a.set_low();
        self.in_b.set_low();
        Ok(())
    }
}

/// Longest time in µs, per part of the sample loop, since the last `# loop max` line.
#[derive(Debug, Clone, Copy, Default)]
struct LoopTimes {
    /// The whole iteration, from the tick to just before waiting for the next one.
    busy: u64,
    /// How long after its scheduled time an iteration started. Anything else running on
    /// the CPU at the tick (the log task formatting a line) shows up here.
    late: u64,
    imu: u64,
    battery_adc: u64,
    /// Putting one message into the `LOG` queue.
    log_send: u64,
    /// Messages dropped because the `LOG` queue was full (a count, not a time).
    dropped: u32,
}

/// What the sample loop hands to the log task: raw numbers only, because turning them
/// into text is slow (the C3 has no FPU, so float formatting runs in software).
enum LogMsg {
    Sample {
        t_us: u64,
        raw: RawSample,
        out: ControlOutput,
        /// Duty applied to the `[left, right]` motor, signed percent.
        duty_pct: [i8; 2],
    },
    Battery {
        battery_v: f32,
        pin_mv: u16,
    },
    Times(LoopTimes),
    Motor(TestStep),
    MotorError(channel::Error),
    MotorCmd(f32),
    /// The safety cut-offs changed: new status, filter pitch (deg), battery (V).
    Safety(Status, f32, f32),
    /// PWM frequency in Hz, logged once at the start.
    PwmFrequency(u32),
    ImuError(I2cError),
}

/// Queue from the sample loop to `log_writer`. A `static` because both tasks need it and
/// tasks only take `'static` data. `CriticalSectionRawMutex` makes it safe to use from
/// different executors and interrupt priorities (needed once the loop runs on its own,
/// higher-priority executor). 8 messages ≈ half a second of logging.
static LOG: Queue<CriticalSectionRawMutex, LogMsg, 8> = Queue::new();

/// Formats one message as a log line (with newline) into `line`.
fn format_msg(line: &mut heapless::String<192>, msg: &LogMsg) -> core::fmt::Result {
    match msg {
        LogMsg::Sample {
            t_us,
            raw: RawSample { accel: a, gyro: g },
            out:
                ControlOutput {
                    accel_pitch_deg: pitch,
                    rates_dps: [rx, ry, rz],
                    gyro_pitch_deg: gyro_pitch,
                    filter_pitch_deg: filter_pitch,
                    motor_cmd,
                    speed_est,
                    pos_est,
                },
            duty_pct: [duty_l, duty_r],
        } => writeln!(
            line,
            "{t_us:>10},{:>6},{:>6},{:>6},{:>6},{:>6},{:>6},{pitch:>9.1},{rx:>7.2},{ry:>7.2},{rz:>7.2},{gyro_pitch:>14.1},{filter_pitch:>16.1},{motor_cmd:>9.3},{duty_l:>6},{duty_r:>6},{speed_est:>9.3},{pos_est:>9.3}",
            a[0], a[1], a[2], g[0], g[1], g[2]
        ),
        LogMsg::Battery { battery_v, pin_mv } => {
            writeln!(line, "# battery {battery_v:.2} V (pin {pin_mv} mV)")
        }
        LogMsg::Times(LoopTimes {
            busy,
            late,
            imu,
            battery_adc,
            log_send,
            dropped,
        }) => writeln!(
            line,
            "# loop max us: busy {busy} of {}, late {late}, imu {imu}, battery adc {battery_adc}, log send {log_send}; dropped {dropped}",
            SAMPLE_PERIOD.as_micros()
        ),
        LogMsg::Motor(TestStep::Run(side, direction, ramp, duty_pct)) => {
            writeln!(line, "# motor {side:?} {direction:?} {ramp:?} {duty_pct}%")
        }
        LogMsg::Motor(TestStep::Stop) => writeln!(line, "# motor stop"),
        LogMsg::Motor(TestStep::Done) => {
            writeln!(line, "# motor test done (or disabled), motors off")
        }
        LogMsg::PwmFrequency(hz) => writeln!(line, "# motor pwm frequency {hz} Hz"),
        LogMsg::Safety(status, pitch_deg, battery_v) => writeln!(
            line,
            "# safety: motors {} (upright: {}, battery: {:?}; pitch {pitch_deg:+.1} deg, battery {battery_v:.2} V)",
            if status.motors_allowed() { "on" } else { "off" },
            status.upright,
            status.battery,
        ),
        LogMsg::MotorCmd(cmd) => writeln!(line, "# motor cmd {cmd:+.2} (both)"),
        LogMsg::MotorError(e) => writeln!(line, "# motor command failed: {e:?}"),
        LogMsg::ImuError(e) => writeln!(line, "# IMU read failed: {e:?}"),
    }
}

/// Reads accel + gyro in one I2C transaction: write the address of the first data
/// register, then (after a repeated START, without releasing the bus) read 12 bytes.
/// The chip steps through the registers by itself. Within one transaction it doesn't
/// update the data registers (as far as we know for this TDK sensor family; the log
/// would show torn values otherwise), so all 12 bytes come from the same sample.
fn read_imu(i2c: &mut I2c<'_, Blocking>) -> Result<RawSample, I2cError> {
    let mut bytes = [0; DATA_LEN];
    i2c.write_read(IMU_ADDRESS, &[DATA_START_REGISTER], &mut bytes)?;
    Ok(RawSample::from_burst(&bytes))
}

/// The battery voltage divider's ADC input (GPIO4 on ADC1).
struct Battery {
    adc: Adc<'static, ADC1<'static>, Blocking>,
    pin: AdcPin<GPIO4<'static>, ADC1<'static>, AdcCalCurve<ADC1<'static>>>,
    /// Sum and count of the readings `poll` has collected for the next average.
    sum: u32,
    count: u32,
}

impl Battery {
    fn new(
        adc: Adc<'static, ADC1<'static>, Blocking>,
        pin: AdcPin<GPIO4<'static>, ADC1<'static>, AdcCalCurve<ADC1<'static>>>,
    ) -> Self {
        Self {
            adc,
            pin,
            sum: 0,
            count: 0,
        }
    }

    /// Takes `BATTERY_AVERAGE` readings right away and averages them; returns
    /// `(battery_v, pin_mv)`. Busy-waits for each conversion (~45 µs each), so only for
    /// startup, not for the 500 Hz loop.
    fn read_blocking(&mut self) -> (f32, u16) {
        loop {
            if let Some(reading) = self.poll() {
                return reading;
            }
        }
    }

    /// Never waits; call it once per loop iteration. `read_oneshot` starts a conversion
    /// and returns `WouldBlock` until the result is ready; the next call (one iteration
    /// later) picks the result up, and the call after that starts the next conversion. So
    /// a reading costs a few µs instead of a busy-wait. Returns `(battery_v, pin_mv)`
    /// whenever `BATTERY_AVERAGE` readings have been collected, else `None`.
    fn poll(&mut self) -> Option<(f32, u16)> {
        let pin_mv = self.adc.read_oneshot(&mut self.pin).ok()?;
        self.sum += u32::from(pin_mv);
        self.count += 1;
        if self.count < BATTERY_AVERAGE {
            return None;
        }
        // The mean of u16 values always fits in a u16.
        let mean_mv = u16::try_from(self.sum / self.count).unwrap_or(u16::MAX);
        self.sum = 0;
        self.count = 0;
        Some((BATTERY_DIVIDER.battery_v(mean_mv), mean_mv))
    }
}

/// A push-pull output that starts low.
fn output_low<'d>(pin: impl OutputPin + 'd) -> Output<'d> {
    Output::new(pin, Level::Low, OutputConfig::default())
}

/// Sets everything up, calibrates the gyro and starts the tasks for `experiment`.
///
/// Called from each binary's `main`, which `#[esp_rtos::main]` runs as the first task of
/// the Embassy executor; `spawner` starts more tasks. Returns once the tasks run.
#[allow(
    clippy::too_many_lines,
    reason = "setup in order, top to bottom: pins, timers, battery, IMU, calibration, tasks"
)]
pub async fn run(spawner: Spawner, experiment: Experiment) {
    let mut peripherals = esp_hal::init(esp_hal::Config::default());
    println!("# experiment: {experiment:?}");

    // L298N inputs: drive all of them low first thing. Until this point they float, and a
    // floating EN that happens to read high while its IN pins differ spins the motor.
    let left_in = (output_low(peripherals.GPIO1), output_low(peripherals.GPIO3));
    let right_in = (
        output_low(peripherals.GPIO20),
        output_low(peripherals.GPIO21),
    );
    // The EN pins become PWM outputs later, in `control_loop`. Until then, drive them low
    // through a temporary `Output`: `reborrow` lends the pin to it and gets it back when
    // the `Output` is dropped right away (`let _`), and esp-hal leaves a dropped output
    // pin as it was, so it stays low.
    let _ = output_low(peripherals.GPIO0.reborrow());
    let _ = output_low(peripherals.GPIO6.reborrow());

    // Embassy timers need a time source: esp-rtos drives them from hardware timer 0 of
    // timer group 0, and uses a software interrupt to wake the executor when one expires.
    // Started early, because `halt` awaits a timer.
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    let led = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    // Calling a task function only builds it; that fails if the task is already running
    // (each task has one static slot). `spawn` then hands it to the executor.
    match blink(led) {
        Ok(task) => spawner.spawn(task),
        Err(e) => println!("# blink task not started: {e:?}"),
    }

    // Battery voltage on GPIO4 = ADC1 channel 4. 11 dB attenuation gives an input range of
    // about 0-2.5 V on the C3 (a full pack puts ~2.1 V on the pin). `AdcCalCurve` corrects
    // each reading with the chip's factory calibration (stored in eFuses) and returns mV.
    let mut adc_config = AdcConfig::new();
    let pin =
        adc_config.enable_pin_with_cal::<_, AdcCalCurve<_>>(peripherals.GPIO4, Attenuation::_11dB);
    let mut battery = Battery::new(Adc::new(peripherals.ADC1, adc_config), pin);
    let (battery_v, pin_mv) = battery.read_blocking();
    println!("# battery {battery_v:.2} V (pin {pin_mv} mV)");

    // The board wires the IMU (and the SHTC3) to SDA = GPIO10, SCL = GPIO8, with pull-ups.
    // 400 kHz "fast mode": at 500 Hz later, every µs of bus time counts.
    let i2c = match I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    ) {
        Ok(i2c) => i2c.with_sda(peripherals.GPIO10).with_scl(peripherals.GPIO8),
        Err(e) => halt(format_args!("I2C config rejected: {e:?}")).await,
    };

    // `new` checks WHO_AM_I and switches accel + gyro on (the chip powers up asleep).
    let mut imu = match Icm42670::new(i2c, Address::Primary) {
        Ok(imu) => imu,
        Err(e) => halt(format_args!("IMU not found at 0x68: {e:?}")).await,
    };
    if let Err(e) = imu
        .set_accel_range(ACCEL_RANGE)
        .and_then(|()| imu.set_gyro_range(GYRO_RANGE))
        .and_then(|()| imu.set_accel_odr(ACCEL_ODR))
        .and_then(|()| imu.set_gyro_odr(GYRO_ODR))
    {
        halt(format_args!("IMU range/ODR setup failed: {e:?}")).await;
    }
    // Read the rates back, to see that the chip took them.
    match (imu.accel_odr(), imu.gyro_odr()) {
        (Ok(accel), Ok(gyro)) => println!("# IMU output data rate: accel {accel:?}, gyro {gyro:?}"),
        (Err(e), _) | (_, Err(e)) => println!("# IMU ODR read-back failed: {e:?}"),
    }

    // Compare once: the driver's reads (one I2C transaction per byte, 12 per sample) with
    // one burst read. Then take the bus back from the driver for good.
    let start = Instant::now();
    let per_register_ok = imu.accel_raw().is_ok() && imu.gyro_raw().is_ok();
    let per_register_us = start.elapsed().as_micros();
    let mut i2c = imu.free();
    let start = Instant::now();
    let burst_ok = read_imu(&mut i2c).is_ok();
    let burst_us = start.elapsed().as_micros();
    println!(
        "# IMU read time: per-register {per_register_us} us (ok: {per_register_ok}), burst {burst_us} us (ok: {burst_ok})"
    );

    // The controller settings shared with the simulator, with the gyro scale taken from
    // the range the chip was actually set to.
    let config = Config {
        gyro_lsb_per_dps: GYRO_RANGE.scale_factor(),
        gains: experiment.gains(),
        ..Config::PROTOTYPE
    };

    // Gyro bias: average ~2 s of samples while the robot is still, and retry until the
    // robot was still enough.
    let bias: GyroBias = loop {
        println!("# calibrating gyro bias: keep the robot still...");
        let mut calibrator = BiasCalibrator::new();
        // A Ticker fires on a fixed schedule (every SAMPLE_PERIOD since it was created),
        // so the rate stays exact even though each iteration takes a little time.
        let mut ticker = Ticker::every(SAMPLE_PERIOD);
        for _ in 0..config.calibration_samples {
            match read_imu(&mut i2c) {
                Ok(sample) => calibrator.add(config.gyro_dps(sample.gyro)),
                Err(e) => println!("# gyro read failed: {e:?}"),
            }
            ticker.next().await;
        }
        match calibrator.finish(
            config.calibration_samples,
            config.calibration_max_spread_dps,
        ) {
            Ok(bias) => break bias,
            Err(e) => println!("# gyro calibration rejected ({e:?}), retrying"),
        }
    };
    let GyroBias([bx, by, bz]) = bias;
    println!("# gyro bias (deg/s): x={bx:.3} y={by:.3} z={bz:.3}");

    // Header lines start with '#' so a CSV parser can skip them. Columns are right-aligned
    // to fixed widths (i16 needs 6 chars: "-32768") so the raw log is readable by eye;
    // the padding is only spaces, so it is still valid CSV.
    println!(
        "# ICM-42670-P raw samples; accel {ACCEL_RANGE:?} = {} LSB/g, gyro {GYRO_RANGE:?} = {} LSB/(deg/s)",
        ACCEL_RANGE.scale_factor(),
        GYRO_RANGE.scale_factor()
    );
    println!(
        "{:>10},{:>6},{:>6},{:>6},{:>6},{:>6},{:>6},{:>9},{:>7},{:>7},{:>7},{:>14},{:>16},{:>9},{:>6},{:>6},{:>9},{:>9}",
        "t_us",
        "ax",
        "ay",
        "az",
        "gx",
        "gy",
        "gz",
        "pitch_deg",
        "gx_dps",
        "gy_dps",
        "gz_dps",
        "gyro_pitch_deg",
        "filter_pitch_deg",
        "motor_cmd",
        "duty_l",
        "duty_r",
        "speed_est",
        "pos_est"
    );

    // From here on, log lines go through the queue to the log task, which owns the async
    // USB-Serial-JTAG driver. (`println!` writes the same hardware directly; it is only
    // used before this point and in `halt`.)
    let (_, usb_tx) = UsbSerialJtag::new(peripherals.USB_DEVICE)
        .into_async()
        .split();
    match log_writer(usb_tx) {
        Ok(task) => spawner.spawn(task),
        Err(e) => println!("# log task not started: {e:?}"),
    }

    // The control loop runs on its own executor, inside a software interrupt at priority 2.
    // Interrupts preempt normal code, so whenever its tick comes it runs right away, even
    // if the thread-mode tasks (log writer, blink) are in the middle of something. The
    // executor must live forever (`&'static mut`): `StaticCell` hands out that reference
    // once, safely.
    let executor = CONTROL_EXECUTOR.init(InterruptExecutor::new(peripherals.FROM_CPU_INTR1));
    let control_spawner = executor.start(Priority::Priority2);
    let parts = ControlParts {
        experiment,
        i2c,
        battery,
        controller: Controller::new(config, bias),
        ledc: peripherals.LEDC,
        left: MotorPins {
            en: peripherals.GPIO0.into(),
            in_a: left_in.0,
            in_b: left_in.1,
        },
        right: MotorPins {
            en: peripherals.GPIO6.into(),
            in_a: right_in.0,
            in_b: right_in.1,
        },
    };
    match control_loop(parts) {
        Ok(task) => control_spawner.spawn(task),
        Err(e) => println!("# control task not started: {e:?}"),
    }
}

/// Everything the control loop needs, handed over from `main`.
///
/// Spawning onto another executor counts as sending the task to another "thread", so all
/// of this must be `Send`. The motors' LEDC channels aren't (they hold a reference to a
/// timer trait object), so the task builds them itself from the raw peripherals.
struct ControlParts {
    experiment: Experiment,
    i2c: I2c<'static, Blocking>,
    battery: Battery,
    /// Plain numbers only (settings, bias, filter and PID state), so it is `Send`.
    controller: Controller,
    ledc: LEDC<'static>,
    left: MotorPins,
    right: MotorPins,
}

/// The pins of one L298N channel before its EN pin becomes a PWM output.
struct MotorPins {
    en: AnyPin<'static>,
    in_a: Output<'static>,
    in_b: Output<'static>,
}

static CONTROL_EXECUTOR: StaticCell<InterruptExecutor<1>> = StaticCell::new();
static PWM_TIMER: StaticCell<timer::Timer<'static, LowSpeed>> = StaticCell::new();

/// A tripped safety cut-off stops the test motor; the test sequence itself keeps its time.
fn gate_test_step(step: TestStep, motors_allowed: bool) -> TestStep {
    match step {
        TestStep::Run(..) if !motors_allowed => TestStep::Stop,
        other => other,
    }
}

/// Drives the motors for one motor test step.
fn apply_test_step(
    step: TestStep,
    left: &mut Motor<'_>,
    right: &mut Motor<'_>,
) -> Result<(), channel::Error> {
    match step {
        TestStep::Run(side, direction, _, duty_pct) => match side {
            Side::Left => left.run(direction, duty_pct),
            Side::Right => right.run(direction, duty_pct),
        },
        TestStep::Stop | TestStep::Done => left.stop().and_then(|()| right.stop()),
    }
}

/// Drives both motors with the controller's command, if the experiment lets the controller
/// drive them; stopped while the safety cut-offs say no.
///
/// The command goes through `Drive::PROTOTYPE` first (bring-up steps B2/B3, the same code
/// the simulator tests): deadband compensation maps it onto the duty range where a turning
/// wheel actually moves (8 %..100 %), and battery scaling (× 7.4 V / battery) gives the
/// same wheel speed per command on a full and a nearly empty pack.
fn apply_control(
    experiment: Experiment,
    cmd: f32,
    battery_v: f32,
    motors_allowed: bool,
    left: &mut Motor<'_>,
    right: &mut Motor<'_>,
) -> Result<(), channel::Error> {
    if !experiment.control_drives_motors() {
        return Ok(());
    }
    let duty = if motors_allowed {
        Drive::PROTOTYPE.duty(cmd, battery_v)
    } else {
        0.0
    };
    left.set(duty).and(right.set(duty))
}

/// Sets up PWM on the L298N enable pins and returns the `(left, right)` motors, stopped.
async fn setup_motors(
    ledc: LEDC<'static>,
    left: MotorPins,
    right: MotorPins,
) -> (Motor<'static>, Motor<'static>) {
    // PWM on the enable pins with the LEDC peripheral. A LEDC *timer* is a counter that
    // sets the frequency and the duty resolution; each *channel* compares that counter
    // with its own duty value and drives one pin. Both motors share one timer.
    // 10-bit duty: 1024 steps, plenty for 1 % steps (80 MHz / (1 kHz * 1024) ≈ 78 is a
    // valid clock divider).
    let mut ledc = Ledc::new(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    // The channels keep a reference to the timer for as long as they exist, i.e. forever:
    // so the timer must be `'static` too.
    let pwm_timer = PWM_TIMER.init(ledc.timer::<LowSpeed>(timer::Number::Timer0));
    if let Err(e) = pwm_timer.configure(timer::config::Config {
        duty: timer::config::Duty::Duty10Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: MOTOR_PWM_FREQUENCY,
    }) {
        halt(format_args!("PWM timer setup failed: {e:?}")).await;
    }
    let pwm_channel = |number, pin: AnyPin<'static>| {
        let mut channel = ledc.channel::<LowSpeed>(number, pin);
        channel
            .configure(channel::config::Config {
                timer: &*pwm_timer,
                duty_pct: 0,
                drive_mode: DriveMode::PushPull,
            })
            .map(|()| channel)
    };
    let (ena, enb) = match (
        pwm_channel(channel::Number::Channel0, left.en),
        pwm_channel(channel::Number::Channel1, right.en),
    ) {
        (Ok(ena), Ok(enb)) => (ena, enb),
        (Err(e), _) | (_, Err(e)) => halt(format_args!("PWM channel setup failed: {e:?}")).await,
    };
    let left = Motor {
        en: ena,
        in_a: left.in_a,
        in_b: left.in_b,
        duty_pct: 0,
    };
    let right = Motor {
        en: enb,
        in_a: right.in_a,
        in_b: right.in_b,
        duty_pct: 0,
    };
    (left, right)
}

/// The 500 Hz loop: read the IMU, update the filters, run the motor test, queue log
/// messages. Runs on the interrupt executor, so it preempts the thread-mode tasks.
#[embassy_executor::task]
#[allow(
    clippy::too_many_lines,
    reason = "one iteration in order, top to bottom: splitting it would hide the order of battery, tests, IMU, control, safety and logging"
)]
async fn control_loop(parts: ControlParts) {
    let (mut left, mut right) = setup_motors(parts.ledc, parts.left, parts.right).await;
    let ControlParts {
        experiment,
        mut i2c,
        mut battery,
        mut controller,
        ..
    } = parts;

    let motor_test_start = Instant::now();
    let mut motor_step = None;
    let mut last_check_cmd: Option<f32> = None;
    // Safety cut-offs (fallen, battery). Motors start off until the robot is held upright.
    // Updated after each IMU read, so the motor code below uses the previous iteration's
    // status: 2 ms late, which doesn't matter for stopping.
    let mut safety = Safety::new(Limits::PROTOTYPE);
    // The most recent battery average; `poll` updates it about every 64 ms.
    let mut latest_battery = battery.read_blocking();
    // Longest times since the last `# loop max` line: the whole iteration and its parts.
    let mut times = LoopTimes::default();
    // Never waits: if the log task has fallen behind and the queue is full, the message is
    // dropped and counted. The loop's timing matters more than a complete log.
    let log = |times: &mut LoopTimes, msg: LogMsg| {
        let start = Instant::now();
        if LOG.try_send(msg).is_err() {
            times.dropped += 1;
        }
        times.log_send = times.log_send.max(start.elapsed().as_micros());
    };

    log(
        &mut times,
        LogMsg::PwmFrequency(MOTOR_PWM_FREQUENCY.as_hz()),
    );

    let loop_start = Instant::now();
    let mut ticker = Ticker::every(SAMPLE_PERIOD);
    // Iterations since `loop_start`; iteration n is due at `loop_start + n × SAMPLE_PERIOD`.
    let mut iteration: u64 = 0;
    let mut sample: u32 = 0;
    loop {
        let tick = Instant::now();
        let due = loop_start + Duration::from_micros(iteration * SAMPLE_PERIOD.as_micros());
        iteration += 1;
        let late = tick
            .checked_duration_since(due)
            .map_or(0, |d| d.as_micros());
        times.late = times.late.max(late);
        // Wraps after ~100 days at 500 Hz; the only effect is one irregular log gap.
        sample = sample.wrapping_add(1);

        let start = Instant::now();
        if let Some(reading) = battery.poll() {
            latest_battery = reading;
        }
        times.battery_adc = times.battery_adc.max(start.elapsed().as_micros());
        if sample.is_multiple_of(BATTERY_LOG_EVERY_N_SAMPLES) {
            let (battery_v, pin_mv) = latest_battery;
            log(&mut times, LogMsg::Battery { battery_v, pin_mv });
        }

        let step = if experiment == Experiment::DeadbandTest {
            motor_test_step(motor_test_start.elapsed().as_millis() / MOTOR_TEST_STEP_MS)
        } else {
            TestStep::Done
        };
        let step = gate_test_step(step, safety.status().motors_allowed());
        if motor_step != Some(step) {
            motor_step = Some(step);
            log(&mut times, LogMsg::Motor(step));
            if let Err(e) = apply_test_step(step, &mut left, &mut right) {
                log(&mut times, LogMsg::MotorError(e));
            }
        }

        if experiment == Experiment::MotorCheck {
            let elapsed_ms = motor_test_start.elapsed().as_millis();
            let cmd = motor_check_cmd(elapsed_ms, safety.status().motors_allowed());
            if last_check_cmd != Some(cmd) {
                last_check_cmd = Some(cmd);
                log(&mut times, LogMsg::MotorCmd(cmd));
                if let Err(e) = left.set(cmd).and(right.set(cmd)) {
                    log(&mut times, LogMsg::MotorError(e));
                }
            }
        }

        let t_us = Instant::now().as_micros();

        // Read every sample even when it isn't logged: later steps feed each one into the
        // filter and the control loop. Read errors are always logged.
        let start = Instant::now();
        let imu_result = read_imu(&mut i2c);
        times.imu = times.imu.max(start.elapsed().as_micros());
        match imu_result {
            Ok(raw) => {
                // The same code the simulator runs.
                let out = controller.step(&raw, DT_S);
                let (pitch, battery_v) = (out.filter_pitch_deg, latest_battery.0);
                // The bias-corrected pitch rate: re-arming waits until the robot is held
                // still, not just upright (see `balance_core::safety`).
                let rate = robot_pitch_rate(out.rates_dps);
                if let Some(status) = safety.update(pitch, rate, battery_v, DT_S) {
                    log(&mut times, LogMsg::Safety(status, pitch, battery_v));
                }
                // Apply the command after the safety check of this very sample, so a
                // tripped cut-off stops the motors in the same iteration.
                let allowed = safety.status().motors_allowed();
                if let Err(e) = apply_control(
                    experiment,
                    out.motor_cmd,
                    battery_v,
                    allowed,
                    &mut left,
                    &mut right,
                ) {
                    log(&mut times, LogMsg::MotorError(e));
                }
                // While the motors are off nothing corrects the pitch, so the I term would
                // wind up and the motors would start at full power once allowed again.
                if !allowed {
                    controller.reset_pid();
                }
                if motor_step == Some(TestStep::Done) && sample.is_multiple_of(LOG_EVERY_N_SAMPLES)
                {
                    log(
                        &mut times,
                        LogMsg::Sample {
                            t_us,
                            raw,
                            out,
                            duty_pct: [left.duty_pct, right.duty_pct],
                        },
                    );
                }
            }
            Err(e) => log(&mut times, LogMsg::ImuError(e)),
        }

        // Everything above has to fit into one SAMPLE_PERIOD. If it doesn't, the Ticker
        // fires again immediately and the loop falls behind its schedule.
        times.busy = times.busy.max(tick.elapsed().as_micros());
        if sample.is_multiple_of(LOOP_STATS_EVERY_N_SAMPLES) {
            let report = times;
            times = LoopTimes::default();
            log(&mut times, LogMsg::Times(report));
        }

        // Wait for the next tick. While main waits here, the executor runs other tasks.
        ticker.next().await;
    }
}

/// Blinks the LED once a second: 0.5 s on, 0.5 s off, forever.
///
/// The `Output` is `'static` because a task may outlive `main`'s stack frame as far as
/// the compiler knows; the peripherals from `esp_hal::init` are `'static`.
#[embassy_executor::task]
async fn blink(mut led: Output<'static>) {
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}

/// Takes log messages out of the `LOG` queue, formats them and sends them over USB.
///
/// `write_all` sends in 64-byte USB packets and `await`s while the host picks each one
/// up, so the CPU is free for other tasks meanwhile. Formatting has no `.await`, though:
/// while a line is being formatted, nothing else on this executor runs. Write errors are
/// ignored, there is nowhere to report them.
#[embassy_executor::task]
async fn log_writer(mut usb: UsbSerialJtagTx<'static, Async>) {
    let mut line = heapless::String::<192>::new();
    loop {
        let msg = LOG.receive().await;
        line.clear();
        // A line too long for the buffer comes out cut off; better than none.
        let _ = format_msg(&mut line, &msg);
        let _ = usb.write_all(line.as_bytes()).await;
    }
}

/// Setup failed: keep printing why, so the message shows up even if the monitor attaches late.
async fn halt(reason: core::fmt::Arguments<'_>) -> ! {
    loop {
        println!("# FATAL: {reason}");
        Timer::after(Duration::from_secs(1)).await;
    }
}
