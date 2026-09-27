//! Two async Embassy tasks sharing one CPU:
//! - `heartbeat` double-blinks the LED on GPIO7 ("lub-dub", once a second);
//! - `main` reads raw accelerometer and gyro samples from the onboard ICM-42670-P and
//!   logs them over USB-Serial-JTAG as CSV lines, 100 per second.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    time::{Instant, Rate},
    timer::timg::TimerGroup,
};
use esp_println::println;
use icm42670::{AccelRange, Address, GyroRange, Icm42670, prelude::*};

esp_bootloader_esp_idf::esp_app_desc!();

/// ±4 g: a balancing robot stays well below that, and it keeps 8192 LSB per g.
const ACCEL_RANGE: AccelRange = AccelRange::G4;
/// ±500 °/s: fast enough for a falling robot, 65.5 LSB per °/s.
const GYRO_RANGE: GyroRange = GyroRange::Deg500;
const SAMPLE_PERIOD: Duration = Duration::from_millis(10);

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // esp-rtos drives the Embassy time driver from this hardware timer; the software
    // interrupt lets it wake the executor when a timer expires.
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    let led = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    // Calling a task function only builds it; it fails if its static slot is already in use.
    match heartbeat(led) {
        Ok(task) => spawner.spawn(task),
        Err(e) => println!("# heartbeat task not started: {e:?}"),
    }

    // The board wires the IMU (and the SHTC3) to SDA = GPIO10, SCL = GPIO8, with pull-ups.
    // 400 kHz "fast mode": the driver does one I2C transaction per register byte, so speed matters.
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
    {
        halt(format_args!("IMU range setup failed: {e:?}")).await;
    }

    // Header lines start with '#' so a CSV parser can skip them.
    println!(
        "# ICM-42670-P raw samples; accel {ACCEL_RANGE:?} = {} LSB/g, gyro {GYRO_RANGE:?} = {} LSB/(deg/s)",
        ACCEL_RANGE.scale_factor(),
        GYRO_RANGE.scale_factor()
    );
    println!("t_us,ax,ay,az,gx,gy,gz");

    // A Ticker fires on a fixed schedule (every 10 ms since it was created), so the rate
    // doesn't drift by however long the reads and prints take.
    let mut ticker = Ticker::every(SAMPLE_PERIOD);
    loop {
        let t_us = Instant::now().duration_since_epoch().as_micros();

        match (imu.accel_raw(), imu.gyro_raw()) {
            (Ok(a), Ok(g)) => println!("{t_us},{},{},{},{},{},{}", a.x, a.y, a.z, g.x, g.y, g.z),
            (Err(e), _) => println!("# accel read failed: {e:?}"),
            (_, Err(e)) => println!("# gyro read failed: {e:?}"),
        }

        // `.await` hands the CPU back to the executor, which runs `heartbeat` in the gaps.
        ticker.next().await;
    }
}

/// Double-blink ("lub-dub") once a second, independent of what `main` is doing.
///
/// `'static` because a task outlives `main`'s stack frame as far as the compiler knows.
#[embassy_executor::task]
async fn heartbeat(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        Timer::after_millis(100).await;
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        Timer::after_millis(700).await;
    }
}

/// Setup failed: keep printing why, so the message shows up even if the monitor attaches late.
async fn halt(reason: core::fmt::Arguments<'_>) -> ! {
    loop {
        println!("# FATAL: {reason}");
        Timer::after_secs(1).await;
    }
}
