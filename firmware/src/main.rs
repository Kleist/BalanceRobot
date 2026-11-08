//! Read raw accelerometer and gyro samples from the onboard ICM-42670-P and log them
//! over USB-Serial-JTAG as CSV lines, followed by the pitch angle computed from the
//! accelerometer alone and the bias-corrected gyro rates. The IMU is read at 100 Hz, but
//! only every `LOG_EVERY_N_SAMPLES`th sample is logged, so the log stays readable.
//!
//! At startup the gyro bias is calibrated while the robot stands still (LED on).
//! After that the LED toggles on every sample, so a steady glow/flicker means the
//! loop is running.
//!
//! Motor test, after calibration, repeating: each motor in turn runs at full speed
//! forward, then backward, 2 s each, with a 2 s stop in between (see `MOTOR_TEST`).
//! `# motor ...` lines in the log mark each phase. Keep the wheels off the ground.

#![no_std]
#![no_main]

use balance_core::{
    attitude::accel_pitch_deg,
    gyro::{BiasCalibrator, GyroBias},
};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig, OutputPin},
    i2c::master::{Config as I2cConfig, I2c},
    main,
    time::{Duration, Instant, Rate},
};
use esp_println::println;
use icm42670::{
    AccelRange, Address, GyroRange, Icm42670, accelerometer::vector::I16x3, prelude::*,
};

esp_bootloader_esp_idf::esp_app_desc!();

/// ±4 g: a balancing robot stays well below that, and it keeps 8192 LSB per g.
const ACCEL_RANGE: AccelRange = AccelRange::G4;
/// ±500 °/s: fast enough for a falling robot, 65.5 LSB per °/s.
const GYRO_RANGE: GyroRange = GyroRange::Deg500;
const SAMPLE_PERIOD: Duration = Duration::from_millis(10);
/// Log one CSV line per this many samples: 10 = 10 lines/s. Set to 1 to log every sample
/// (e.g. to look at vibration or noise), or higher for an even quieter log.
const LOG_EVERY_N_SAMPLES: u32 = 10;

/// Gyro calibration: 200 samples at 100 Hz = 2 s of holding still.
const CALIBRATION_SAMPLES: u32 = 200;
/// Largest allowed max − min per axis during calibration, in °/s. The gyro's noise at
/// rest is about 0.5 °/s peak to peak, but the driver reads high and low bytes in
/// separate transactions, so an axis crossing zero can tear into a single spike of up to
/// 255 LSB ≈ 3.9 °/s. Moving the robot gives tens of °/s, which rejects the calibration.
/// Tighten this once the IMU is burst-read.
const CALIBRATION_MAX_SPREAD_DPS: f32 = 5.0;

/// Motor test: how long each phase of `MOTOR_TEST` lasts.
const MOTOR_TEST_PHASE_MS: u64 = 2000;

#[derive(Debug, Clone, Copy)]
enum Side {
    Left,
    Right,
}

/// "Forward" is just IN1 high / IN2 low (IN3 / IN4 for the right motor). Whether that
/// drives the robot forward depends on how each motor is wired to the OUT pins: check it
/// on the robot and swap the motor wires if needed.
#[derive(Debug, Clone, Copy)]
enum Direction {
    Forward,
    Backward,
}

/// Motor test sequence, one phase per `MOTOR_TEST_PHASE_MS`, repeating. `None` = both
/// motors stopped. The stop between runs means a motor never reverses at full speed,
/// which would draw a large current spike and could brown out the battery.
const MOTOR_TEST: [Option<(Side, Direction)>; 8] = [
    Some((Side::Left, Direction::Forward)),
    None,
    Some((Side::Left, Direction::Backward)),
    None,
    Some((Side::Right, Direction::Forward)),
    None,
    Some((Side::Right, Direction::Backward)),
    None,
];

/// One L298N channel: EN switches the channel's outputs on (full speed, no PWM yet),
/// the two IN pins pick the direction.
struct Motor<'d> {
    en: Output<'d>,
    in_a: Output<'d>,
    in_b: Output<'d>,
}

impl Motor<'_> {
    fn run(&mut self, direction: Direction) {
        // Switch off while changing the IN pins, so the outputs never see a half-changed
        // combination.
        self.en.set_low();
        let forward = matches!(direction, Direction::Forward);
        self.in_a.set_level(Level::from(forward));
        self.in_b.set_level(Level::from(!forward));
        self.en.set_high();
    }

    /// EN low: both outputs off, the motor coasts to a stop.
    fn stop(&mut self) {
        self.en.set_low();
        self.in_a.set_low();
        self.in_b.set_low();
    }
}

/// Raw gyro LSB to °/s for the configured range.
fn gyro_dps(g: I16x3) -> [f32; 3] {
    let scale = GYRO_RANGE.scale_factor();
    [
        f32::from(g.x) / scale,
        f32::from(g.y) / scale,
        f32::from(g.z) / scale,
    ]
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
fn robot_axes(a: I16x3) -> (f32, f32) {
    let forward = f32::from(a.x);
    let up = f32::from(a.z);
    (forward, up)
}

/// A push-pull output that starts low.
fn output_low<'d>(pin: impl OutputPin + 'd) -> Output<'d> {
    Output::new(pin, Level::Low, OutputConfig::default())
}

/// Busy-wait. Fine for now; later steps replace this with async Embassy timers.
fn wait_until(deadline: Instant) {
    while Instant::now() < deadline {}
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // L298N inputs: drive all of them low first thing. Until this point they float, and a
    // floating EN that happens to read high while its IN pins differ spins the motor.
    let mut left = Motor {
        en: output_low(peripherals.GPIO0),
        in_a: output_low(peripherals.GPIO1),
        in_b: output_low(peripherals.GPIO3),
    };
    let mut right = Motor {
        en: output_low(peripherals.GPIO6),
        in_a: output_low(peripherals.GPIO20),
        in_b: output_low(peripherals.GPIO21),
    };

    let mut led = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());

    // The board wires the IMU (and the SHTC3) to SDA = GPIO10, SCL = GPIO8, with pull-ups.
    // 400 kHz "fast mode": the driver does one I2C transaction per register byte, so speed matters.
    let i2c = match I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    ) {
        Ok(i2c) => i2c.with_sda(peripherals.GPIO10).with_scl(peripherals.GPIO8),
        Err(e) => halt(format_args!("I2C config rejected: {e:?}")),
    };

    // `new` checks WHO_AM_I and switches accel + gyro on (the chip powers up asleep).
    let mut imu = match Icm42670::new(i2c, Address::Primary) {
        Ok(imu) => imu,
        Err(e) => halt(format_args!("IMU not found at 0x68: {e:?}")),
    };
    if let Err(e) = imu
        .set_accel_range(ACCEL_RANGE)
        .and_then(|()| imu.set_gyro_range(GYRO_RANGE))
    {
        halt(format_args!("IMU range setup failed: {e:?}"));
    }

    // Gyro bias: average ~2 s of samples while the robot is still, and retry until the
    // robot was still enough. The LED stays on while calibrating.
    led.set_high();
    let bias: GyroBias = loop {
        println!("# calibrating gyro bias: keep the robot still...");
        let mut calibrator = BiasCalibrator::new();
        let mut next = Instant::now();
        for _ in 0..CALIBRATION_SAMPLES {
            next += SAMPLE_PERIOD;
            match imu.gyro_raw() {
                Ok(g) => calibrator.add(gyro_dps(g)),
                Err(e) => println!("# gyro read failed: {e:?}"),
            }
            wait_until(next);
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
        "{:>10},{:>6},{:>6},{:>6},{:>6},{:>6},{:>6},{:>9},{:>7},{:>7},{:>7}",
        "t_us", "ax", "ay", "az", "gx", "gy", "gz", "pitch_deg", "gx_dps", "gy_dps", "gz_dps"
    );

    let motor_test_start = Instant::now();
    let mut motor_phase = None;

    let mut next = Instant::now();
    let mut sample: u32 = 0;
    loop {
        next += SAMPLE_PERIOD;
        // Wraps after ~500 days at 100 Hz; the only effect is one irregular log gap.
        sample = sample.wrapping_add(1);

        let phases = motor_test_start.elapsed().as_millis() / MOTOR_TEST_PHASE_MS;
        // The remainder is < 8, so the conversion can't fail.
        let phase = usize::try_from(phases % MOTOR_TEST.len() as u64).unwrap_or(0);
        if motor_phase != Some(phase) {
            motor_phase = Some(phase);
            left.stop();
            right.stop();
            if let Some((side, direction)) = MOTOR_TEST[phase] {
                match side {
                    Side::Left => left.run(direction),
                    Side::Right => right.run(direction),
                }
                println!("# motor {side:?} {direction:?}");
            } else {
                println!("# motor stop");
            }
        }

        let t_us = Instant::now().duration_since_epoch().as_micros();

        // Read every sample even when it isn't logged: later steps feed each one into the
        // filter and the control loop. Read errors are always logged.
        match (imu.accel_raw(), imu.gyro_raw()) {
            (Ok(a), Ok(g)) if sample.is_multiple_of(LOG_EVERY_N_SAMPLES) => {
                let (forward, up) = robot_axes(a);
                let pitch = accel_pitch_deg(forward, up);
                let [rx, ry, rz] = bias.apply(gyro_dps(g));
                println!(
                    "{t_us:>10},{:>6},{:>6},{:>6},{:>6},{:>6},{:>6},{pitch:>9.1},{rx:>7.2},{ry:>7.2},{rz:>7.2}",
                    a.x, a.y, a.z, g.x, g.y, g.z
                );
            }
            (Ok(_), Ok(_)) => {}
            (Err(e), _) => println!("# accel read failed: {e:?}"),
            (_, Err(e)) => println!("# gyro read failed: {e:?}"),
        }

        led.toggle();
        wait_until(next);
    }
}

/// Setup failed: keep printing why, so the message shows up even if the monitor attaches late.
fn halt(reason: core::fmt::Arguments) -> ! {
    loop {
        println!("# FATAL: {reason}");
        wait_until(Instant::now() + Duration::from_secs(1));
    }
}
