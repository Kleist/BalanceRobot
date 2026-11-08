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
//!
//! Runs on two Embassy executors sharing the one CPU core:
//! - the normal (thread-mode) executor runs `main` (setup and calibration), then the `blink`
//!   task (LED on GPIO7, once a second) and the `log_writer` task. They take turns at each
//!   `.await`.
//! - an interrupt executor at priority 2 runs `control_loop`, the 500 Hz loop. It runs
//!   inside an interrupt, so its tick preempts the thread-mode tasks even in the middle of
//!   formatting a log line.
//!
//! Deadband test (off unless `MOTOR_TEST_ENABLED`), after calibration: each motor in turn,
//! forward then backward, ramps the PWM duty up from 20 % to 50 % and back down to 10 % in
//! 2 % steps (0.5 s each), then
//! stops for 1 s (see `motor_test_step`). Watch the wheel and note at which logged duty it
//! starts turning on the way up and stops on the way down. The sequence runs twice, then
//! both motors stay off. The IMU CSV is paused during the test, so the `# motor ...` lines
//! are easy to follow. Keep the wheels off the ground.

#![no_std]
#![no_main]

use balance_core::{
    attitude::{ComplementaryFilter, accel_pitch_deg},
    battery::Divider,
    gyro::{BiasCalibrator, GyroBias},
    imu::{DATA_LEN, DATA_START_REGISTER, RawSample},
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

esp_bootloader_esp_idf::esp_app_desc!();

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
/// Log one CSV line per this many samples: 50 = 10 lines/s. Set to 1 to log every sample
/// (500 lines/s; probably more than the USB log keeps up with).
const LOG_EVERY_N_SAMPLES: u32 = 50;
/// Log the loop's busy time once per this many samples: 500 = once a second.
const LOOP_STATS_EVERY_N_SAMPLES: u32 = 500;
/// Complementary filter time constant: changes faster than this come from the gyro,
/// slower ones from the accelerometer. 0.5 s is a common starting point; tune later.
const FILTER_TAU_S: f32 = 0.5;

/// Battery sense divider: battery + → 100 kΩ → GPIO4 → 33 kΩ → GND.
const BATTERY_DIVIDER: Divider = Divider {
    top_ohm: 100_000.0,
    bottom_ohm: 33_000.0,
};
/// ADC readings averaged per battery measurement, to smooth out ADC noise.
const BATTERY_AVERAGE: u32 = 16;
/// Log the battery voltage once per this many samples: 250 = every 0.5 s.
const BATTERY_LOG_EVERY_N_SAMPLES: u32 = 250;

/// Gyro calibration: 1000 samples at 500 Hz = 2 s of holding still.
const CALIBRATION_SAMPLES: u32 = 1000;
/// Largest allowed max − min per axis during calibration, in °/s. The gyro's noise at
/// rest is about 0.5 °/s peak to peak; moving the robot gives tens of °/s, which rejects
/// the calibration. (With per-register reads this had to be 5 °/s: an axis crossing zero
/// could tear into a single 3.9 °/s spike. Burst reads can't tear.)
const CALIBRATION_MAX_SPREAD_DPS: f32 = 2.0;

/// PWM frequency for the L298N enable pins. 1 kHz is a compromise: the L298N switches
/// slowly (a few µs per edge), so at low frequencies those edges are a small part of
/// the period and even small duty cycles come out clean. The price is an audible whine.
/// Revisit once the deadband has been measured.
const MOTOR_PWM_FREQUENCY: Rate = Rate::from_khz(1);

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
/// Run the deadband test after calibration. Off while working on the IMU, because the
/// test pauses the CSV log; set to `true` to run it again.
const MOTOR_TEST_ENABLED: bool = false;

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
}

impl Motor<'_> {
    fn run(&mut self, direction: Direction, duty_pct: u8) -> Result<(), channel::Error> {
        // Switch off while changing the IN pins, so the outputs never see a half-changed
        // combination.
        self.en.set_duty(0)?;
        let forward = direction == Direction::Forward;
        self.in_a.set_level(Level::from(forward));
        self.in_b.set_level(Level::from(!forward));
        self.en.set_duty(duty_pct)
    }

    /// EN low: both outputs off, the motor coasts to a stop.
    fn stop(&mut self) -> Result<(), channel::Error> {
        self.en.set_duty(0)?;
        self.in_a.set_low();
        self.in_b.set_low();
        Ok(())
    }
}

/// Raw gyro LSB to °/s for the configured range.
fn gyro_dps(g: [i16; 3]) -> [f32; 3] {
    let scale = GYRO_RANGE.scale_factor();
    g.map(|v| f32::from(v) / scale)
}

/// Map the IMU's X/Y/Z to the robot's `(forward, up)`, including signs.
///
/// This depends on how the board is mounted, which we have not checked yet. Best guess:
/// board lying flat with the components on top (so +Z is up) and +X pointing forward.
///
/// How to check it in the log (±4 g range, so 1 g = 8192 LSB):
/// 1. Hold the robot upright and still. The axis reading about +8192 is "up" (if it
///    reads about -8192, use it negated). `pitch_deg` should then be about 0.
/// 2. Tilt the robot forward (nose down). The forward value must go negative and
///    `pitch_deg` positive. If the signs are reversed, negate forward; if a different
///    axis changes, use that one as forward.
fn robot_axes(a: [i16; 3]) -> (f32, f32) {
    let forward = f32::from(a[0]);
    let up = f32::from(a[2]);
    (forward, up)
}

/// The robot's pitch rate in °/s (positive = tipping forward) from the gyro's `[x, y, z]`.
///
/// Pitching rotates the robot about its wheel axle, which is the axis perpendicular to
/// both forward and up: Y, if `robot_axes` is right. By the right-hand rule a positive
/// rotation about +Y turns +X (forward) towards −Z (down), i.e. tips forward, so no sign
/// flip. Check it with the log: tilt the robot slowly forward and back, and
/// `gyro_pitch_deg` must move the same way as `pitch_deg`. If it moves the opposite way,
/// negate the rate.
fn robot_pitch_rate(rates_dps: [f32; 3]) -> f32 {
    rates_dps[1]
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
        pitch: f32,
        rates: [f32; 3],
        gyro_pitch: f32,
        filter_pitch: f32,
    },
    Battery {
        battery_v: f32,
        pin_mv: u16,
    },
    Times(LoopTimes),
    Motor(TestStep),
    MotorError(channel::Error),
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
            pitch,
            rates: [rx, ry, rz],
            gyro_pitch,
            filter_pitch,
        } => writeln!(
            line,
            "{t_us:>10},{:>6},{:>6},{:>6},{:>6},{:>6},{:>6},{pitch:>9.1},{rx:>7.2},{ry:>7.2},{rz:>7.2},{gyro_pitch:>14.1},{filter_pitch:>16.1}",
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
}

impl Battery {
    /// Averages `BATTERY_AVERAGE` readings; returns `(battery_v, pin_mv)`.
    fn read(&mut self) -> (f32, u16) {
        let mut sum: u32 = 0;
        for _ in 0..BATTERY_AVERAGE {
            // `read_oneshot` doesn't wait: it starts a conversion and returns `WouldBlock`
            // until the result is ready (a few µs), so poll it.
            let pin_mv = loop {
                if let Ok(mv) = self.adc.read_oneshot(&mut self.pin) {
                    break mv;
                }
            };
            sum += u32::from(pin_mv);
        }
        // The mean of u16 values always fits in a u16.
        let pin_mv = u16::try_from(sum / BATTERY_AVERAGE).unwrap_or(u16::MAX);
        (BATTERY_DIVIDER.battery_v(pin_mv), pin_mv)
    }
}

/// A push-pull output that starts low.
fn output_low<'d>(pin: impl OutputPin + 'd) -> Output<'d> {
    Output::new(pin, Level::Low, OutputConfig::default())
}

/// `#[esp_rtos::main]` sets up the Embassy executor and runs this as its first task.
/// `spawner` starts more tasks.
#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let mut peripherals = esp_hal::init(esp_hal::Config::default());

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
    let mut battery = Battery {
        adc: Adc::new(peripherals.ADC1, adc_config),
        pin,
    };
    let (battery_v, pin_mv) = battery.read();
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

    // Gyro bias: average ~2 s of samples while the robot is still, and retry until the
    // robot was still enough.
    let bias: GyroBias = loop {
        println!("# calibrating gyro bias: keep the robot still...");
        let mut calibrator = BiasCalibrator::new();
        // A Ticker fires on a fixed schedule (every SAMPLE_PERIOD since it was created),
        // so the rate stays exact even though each iteration takes a little time.
        let mut ticker = Ticker::every(SAMPLE_PERIOD);
        for _ in 0..CALIBRATION_SAMPLES {
            match read_imu(&mut i2c) {
                Ok(sample) => calibrator.add(gyro_dps(sample.gyro)),
                Err(e) => println!("# gyro read failed: {e:?}"),
            }
            ticker.next().await;
        }
        match calibrator.finish(CALIBRATION_SAMPLES, CALIBRATION_MAX_SPREAD_DPS) {
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
        "{:>10},{:>6},{:>6},{:>6},{:>6},{:>6},{:>6},{:>9},{:>7},{:>7},{:>7},{:>14},{:>16}",
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
        "filter_pitch_deg"
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
        i2c,
        battery,
        bias,
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
    i2c: I2c<'static, Blocking>,
    battery: Battery,
    bias: GyroBias,
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
    };
    let right = Motor {
        en: enb,
        in_a: right.in_a,
        in_b: right.in_b,
    };
    (left, right)
}

/// The 500 Hz loop: read the IMU, update the filters, run the motor test, queue log
/// messages. Runs on the interrupt executor, so it preempts the thread-mode tasks.
#[embassy_executor::task]
async fn control_loop(parts: ControlParts) {
    let (mut left, mut right) = setup_motors(parts.ledc, parts.left, parts.right).await;
    let ControlParts {
        mut i2c,
        mut battery,
        bias,
        ..
    } = parts;

    let motor_test_start = Instant::now();
    let mut motor_step = None;
    // Gyro-only pitch: starts at the accelerometer pitch of the first sample, then only
    // ever adds up rate × dt. Any leftover bias adds up too, so this drifts.
    let mut gyro_pitch: Option<f32> = None;
    let mut filter = ComplementaryFilter::new(FILTER_TAU_S);
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

        if sample.is_multiple_of(BATTERY_LOG_EVERY_N_SAMPLES) {
            let start = Instant::now();
            let (battery_v, pin_mv) = battery.read();
            times.battery_adc = times.battery_adc.max(start.elapsed().as_micros());
            log(&mut times, LogMsg::Battery { battery_v, pin_mv });
        }

        let step = if MOTOR_TEST_ENABLED {
            motor_test_step(motor_test_start.elapsed().as_millis() / MOTOR_TEST_STEP_MS)
        } else {
            TestStep::Done
        };
        if motor_step != Some(step) {
            motor_step = Some(step);
            log(&mut times, LogMsg::Motor(step));
            if let Err(e) = apply_test_step(step, &mut left, &mut right) {
                log(&mut times, LogMsg::MotorError(e));
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
                let (forward, up) = robot_axes(raw.accel);
                let pitch = accel_pitch_deg(forward, up);
                let rates = bias.apply(gyro_dps(raw.gyro));
                let gyro_pitch = gyro_pitch.get_or_insert(pitch);
                *gyro_pitch += robot_pitch_rate(rates) * DT_S;
                let filter_pitch = filter.update(pitch, robot_pitch_rate(rates), DT_S);

                if motor_step == Some(TestStep::Done) && sample.is_multiple_of(LOG_EVERY_N_SAMPLES)
                {
                    let msg = LogMsg::Sample {
                        t_us,
                        raw,
                        pitch,
                        rates,
                        gyro_pitch: *gyro_pitch,
                        filter_pitch,
                    };
                    log(&mut times, msg);
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
