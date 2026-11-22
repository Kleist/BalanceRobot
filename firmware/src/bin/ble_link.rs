//! Like `imu_log` (motors off, IMU CSV), plus the BLE link to the remote: the robot
//! advertises as `BalanceRobot` with its GATT service, accepts one connection and logs
//! every packet written to it (`# remote: ...`) and when the link times out. Nothing is
//! applied to the motors. Test it from a phone (nRF Connect: connect, write 5 bytes to the
//! packet characteristic). Compare `# loop max us` with `cargo imu-log`.
//!
//! `cargo ble-link` (alias in `.cargo/config.toml`) builds, flashes and opens the log.

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
    run(spawner, Experiment::BleLink).await;
}
