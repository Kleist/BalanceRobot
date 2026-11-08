//! Read raw accelerometer and gyro samples from the onboard ICM-42670-P and log them
//! over USB-Serial-JTAG as CSV lines, ~100 per second, followed by the pitch angle
//! computed from the accelerometer alone and the bias-corrected gyro rates.
//!
//! At startup the gyro bias is calibrated while the robot stands still (LED on).
//! After that the LED toggles on every sample, so a steady glow/flicker means the
//! loop is running.
//!
//! Motor test: the left motor (L298N channel A) runs at full speed in one direction for
//! 2 s, then stops for 2 s, repeating. `# motor on` / `# motor off` lines in the log mark
//! the switches. Keep the wheels off the ground.

#![no_std]
#![no_main]

use balance_core::{
    attitude::accel_pitch_deg,
    gyro::{BiasCalibrator, GyroBias},
};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
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

/// Gyro calibration: 200 samples at 100 Hz = 2 s of holding still.
const CALIBRATION_SAMPLES: u32 = 200;
/// Largest allowed max − min per axis during calibration, in °/s. The gyro's noise at
/// rest is about 0.5 °/s peak to peak, but the driver reads high and low bytes in
/// separate transactions, so an axis crossing zero can tear into a single spike of up to
/// 255 LSB ≈ 3.9 °/s. Moving the robot gives tens of °/s, which rejects the calibration.
/// Tighten this once the IMU is burst-read.
const CALIBRATION_MAX_SPREAD_DPS: f32 = 5.0;

/// Motor test: on for this long, then off for this long.
const MOTOR_TEST_HALF_PERIOD_MS: u64 = 2000;

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

/// Busy-wait. Fine for now; later steps replace this with async Embassy timers.
fn wait_until(deadline: Instant) {
    while Instant::now() < deadline {}
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // L298N inputs: drive all of them low first thing. Until this point they float, and a
    // floating ENA that happens to read high while IN1 != IN2 spins the motor.
    // EN low = both outputs of that channel off (the motor coasts), whatever IN1/IN2 are.
    // The `_name` bindings keep the unused pins alive and low (a plain `_` would drop them).
    let mut ena = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());
    let mut in1 = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());
    let _in2 = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());
    let _enb = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let _in3 = Output::new(peripherals.GPIO20, Level::Low, OutputConfig::default());
    let _in4 = Output::new(peripherals.GPIO21, Level::Low, OutputConfig::default());

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

    // Direction: IN1 high, IN2 low. Which way that turns the wheel depends on how the motor
    // is wired to OUT1/OUT2. Speed is all or nothing for now: ENA high = full speed.
    in1.set_high();
    let motor_test_start = Instant::now();
    let mut motor_on = false;

    let mut next = Instant::now();
    loop {
        next += SAMPLE_PERIOD;

        let half_periods = motor_test_start.elapsed().as_millis() / MOTOR_TEST_HALF_PERIOD_MS;
        let want_on = half_periods.is_multiple_of(2);
        if want_on != motor_on {
            motor_on = want_on;
            if motor_on {
                ena.set_high();
                println!("# motor on");
            } else {
                ena.set_low();
                println!("# motor off");
            }
        }

        let t_us = Instant::now().duration_since_epoch().as_micros();

        match (imu.accel_raw(), imu.gyro_raw()) {
            (Ok(a), Ok(g)) => {
                let (forward, up) = robot_axes(a);
                let pitch = accel_pitch_deg(forward, up);
                let [rx, ry, rz] = bias.apply(gyro_dps(g));
                println!(
                    "{t_us:>10},{:>6},{:>6},{:>6},{:>6},{:>6},{:>6},{pitch:>9.1},{rx:>7.2},{ry:>7.2},{rz:>7.2}",
                    a.x, a.y, a.z, g.x, g.y, g.z
                );
            }
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
