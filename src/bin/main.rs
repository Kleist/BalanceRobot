#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::timer::timg::TimerGroup;
use log::info;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

/// Heartbeat on the ESP32-C3-DevKit-RUST-1 status LED (GPIO7).
#[embassy_executor::task]
async fn heartbeat(mut led: Output<'static>) {
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.4.0
    // generator parameters: -o esp32c3 -o esp32c3-mini-1 -o unstable-hal -o embassy -o esp-backtrace -o log -o ci

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Pin plan (see CLAUDE.md). GPIO11-17 are used by the module's flash and must not be touched.
    // Reserved: GPIO2 (strapping, WS2812), GPIO8/10 (I2C), GPIO9 (BOOT), GPIO18/19 (USB).
    let led = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    info!(
        "BalanceRobot firmware {} starting",
        env!("CARGO_PKG_VERSION")
    );

    spawner.spawn(heartbeat(led).unwrap());

    loop {
        info!("alive");
        Timer::after(Duration::from_secs(5)).await;
    }
}
