//! BBC micro:bit v2 tilt remote for the balance robot. So far: Embassy on the micro:bit.
//! The centre LED of the 5x5 matrix blinks from one task, and presses of buttons A and B
//! are logged over defmt/RTT from two more.
//!
//! The LED matrix is wired as 5 row and 5 column lines (micro:bit v2 schematic): an LED
//! lights when its row is driven high and its column low. Showing a pattern means
//! scanning the rows fast (later step); one LED only needs one row and one column pin.
//! The other pins stay inputs after reset, so their LEDs stay dark.
//!
//! Buttons A (P0.14) and B (P0.23) have pull-up resistors on the board and pull the pin
//! low while pressed. `wait_for_low` is async: the GPIOTE peripheral raises an interrupt
//! on the edge, so the task sleeps until then instead of polling.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_time::{Duration, Timer};
use panic_probe as _;

/// A press must stay low this long to count; filters the contact bounce of the button.
const DEBOUNCE: Duration = Duration::from_millis(20);

/// `#[embassy_executor::main]` creates the executor and runs this as its first task.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Clocks, GPIOTE and the RTC1 time driver with default settings; returns every
    // peripheral once, as an owned singleton.
    let p = embassy_nrf::init(embassy_nrf::config::Config::default());
    defmt::info!("hello from the micro:bit remote (Embassy)");

    // Centre LED: row 3 = P0.15, column 3 = P0.31. Starts off: row low, column high.
    let row = Output::new(p.P0_15, Level::Low, OutputDrive::Standard);
    let col = Output::new(p.P0_31, Level::High, OutputDrive::Standard);
    // Calling a task function only builds it; that fails if all its slots are in use.
    // `spawn` then hands it to the executor.
    match blink(row, col) {
        Ok(task) => spawner.spawn(task),
        Err(e) => defmt::error!("blink task not started: {}", e),
    }

    // The board has external pull-ups; the internal one too does no harm.
    let buttons = [
        ("A", Input::new(p.P0_14, Pull::Up)),
        ("B", Input::new(p.P0_23, Pull::Up)),
    ];
    for (name, pin) in buttons {
        match button(name, pin) {
            Ok(task) => spawner.spawn(task),
            Err(e) => defmt::error!("button {} task not started: {}", name, e),
        }
    }
}

/// Toggles the LED twice a second: on = row high + column low, off = row low + column
/// high (the LED is then reverse-biased, so no current flows either way). The task owns
/// both pins: dropping an `Output` would turn the pin back into an input.
#[embassy_executor::task]
async fn blink(mut row: Output<'static>, mut col: Output<'static>) {
    loop {
        row.toggle();
        col.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}

/// Logs each press and release of one button. Two instances run, one per button
/// (`pool_size = 2`: the executor reserves memory for two copies of this task).
#[embassy_executor::task(pool_size = 2)]
async fn button(name: &'static str, mut pin: Input<'static>) {
    let mut presses: u32 = 0;
    loop {
        pin.wait_for_low().await;
        Timer::after(DEBOUNCE).await;
        if pin.is_high() {
            continue; // a bounce or a glitch, not a press
        }
        presses += 1;
        defmt::info!("button {} pressed ({} so far)", name, presses);
        pin.wait_for_high().await;
        Timer::after(DEBOUNCE).await;
        defmt::info!("button {} released", name);
    }
}
