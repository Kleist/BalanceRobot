//! Balance attempt: the controller drives the motors with the balance gains
//! (`Config::PROTOTYPE`). **On the floor, hand ready.** Save the log:
//! `cargo balance | tee ../balance-N.log`.
//!
//! `cargo balance` (alias in `.cargo/config.toml`) builds, flashes and opens the log.

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
    run(spawner, Experiment::Balance).await;
}
