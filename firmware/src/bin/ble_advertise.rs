//! Like `imu_log` (motors off, IMU CSV), plus BLE advertising: the robot shows up as
//! `BalanceRobot` in a BLE scanner (nRF Connect on a phone). Compare its
//! `# loop max us` lines with `cargo imu-log` to see what the radio costs the 500 Hz loop.
//!
//! `cargo ble-advertise` (alias in `.cargo/config.toml`) builds, flashes and opens the log.

#![no_std]
#![no_main]

use balance_robot::{Experiment, run};
use embassy_executor::Spawner;

// The app descriptor the ESP-IDF 2nd stage bootloader expects (espflash refuses images
// without it). It has to be in the binary itself.
esp_bootloader_esp_idf::esp_app_desc!();

/// `#[esp_rtos::main]` sets up the Embassy executor and runs this as its first task.
#[esp_rtos::main]
async fn main(spawner: Spawner) {
    run(spawner, Experiment::BleAdvertise).await;
}
