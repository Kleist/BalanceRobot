//! Heartbeat: double-blink the LED on GPIO7 ("lub-dub"), pause, repeat,
//! and print a line over USB-Serial-JTAG on every beat.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    main,
    time::{Duration, Instant},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

/// Busy-wait. Fine for now; later steps replace this with async Embassy timers.
fn wait_ms(ms: u64) {
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(ms) {}
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut led = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());

    println!("BalanceRobot: hello from the ESP32-C3");

    let mut beat: u32 = 0;
    loop {
        beat = beat.wrapping_add(1);
        println!("beat {beat}");

        led.set_high();
        wait_ms(100);
        led.set_low();
        wait_ms(100);
        led.set_high();
        wait_ms(100);
        led.set_low();
        wait_ms(700);
    }
}
