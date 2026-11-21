//! Deadband test: each motor in turn, forward then backward, ramps the PWM duty up and
//! back down; note at which logged duty the wheel starts and stops. **Wheels in the air.**
//!
//! `cargo deadband-test` (alias in `.cargo/config.toml`) builds, flashes and opens the log.

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
    run(spawner, Experiment::DeadbandTest).await;
}
