//! Read raw accelerometer and gyro samples from the onboard ICM-42670-P and log them
//! over USB-Serial-JTAG as CSV lines, ~100 per second. The LED on GPIO7 toggles on
//! every sample, so a steady glow/flicker means the loop is running.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    main,
    time::{Duration, Instant, Rate},
};
use esp_println::println;
use icm42670::{AccelRange, Address, GyroRange, Icm42670, prelude::*};

esp_bootloader_esp_idf::esp_app_desc!();

/// ±4 g: a balancing robot stays well below that, and it keeps 8192 LSB per g.
const ACCEL_RANGE: AccelRange = AccelRange::G4;
/// ±500 °/s: fast enough for a falling robot, 65.5 LSB per °/s.
const GYRO_RANGE: GyroRange = GyroRange::Deg500;
const SAMPLE_PERIOD: Duration = Duration::from_millis(10);

/// Busy-wait. Fine for now; later steps replace this with async Embassy timers.
fn wait_until(deadline: Instant) {
    while Instant::now() < deadline {}
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
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

    // Header lines start with '#' so a CSV parser can skip them.
    println!(
        "# ICM-42670-P raw samples; accel {ACCEL_RANGE:?} = {} LSB/g, gyro {GYRO_RANGE:?} = {} LSB/(deg/s)",
        ACCEL_RANGE.scale_factor(),
        GYRO_RANGE.scale_factor()
    );
    println!("t_us,ax,ay,az,gx,gy,gz");

    let mut next = Instant::now();
    loop {
        next += SAMPLE_PERIOD;
        let t_us = Instant::now().duration_since_epoch().as_micros();

        match (imu.accel_raw(), imu.gyro_raw()) {
            (Ok(a), Ok(g)) => println!("{t_us},{},{},{},{},{},{}", a.x, a.y, a.z, g.x, g.y, g.z),
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
