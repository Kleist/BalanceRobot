//! BBC micro:bit v2 tilt remote for the balance robot. So far: the centre LED of the 5x5
//! matrix blinks, presses of buttons A and B are logged, and the accelerometer is read at
//! 50 Hz and logged in milli-g, each from its own Embassy task.
//!
//! The LED matrix is wired as 5 row and 5 column lines (micro:bit v2 schematic): an LED
//! lights when its row is driven high and its column low. Showing a pattern means
//! scanning the rows fast (later step); one LED only needs one row and one column pin.
//! The other pins stay inputs after reset, so their LEDs stay dark.
//!
//! Buttons A (P0.14) and B (P0.23) have pull-up resistors on the board and pull the pin
//! low while pressed. `wait_for_low` is async: the GPIOTE peripheral raises an interrupt
//! on the edge, so the task sleeps until then instead of polling.
//!
//! The LSM303AGR (accelerometer + magnetometer) sits on the micro:bit's internal I2C bus
//! (SCL P0.08, SDA P0.16, pull-ups on the board), separate from the edge connector's.
//! The accelerometer answers at address 0x19. Each sample is logged as
//! `accel mg: x, y, z`; at rest the vector points *up* with ~1000 mg (the sensor measures
//! the force holding it up against gravity, like the robot's IMU).

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    gpio::{Input, Level, Output, OutputDrive, Pull},
    peripherals, twim,
};
use embassy_time::{Delay, Duration, Ticker, Timer};
use lsm303agr::{AccelMode, AccelOutputDataRate, Lsm303agr};
use panic_probe as _;
use static_cell::StaticCell;

/// A press must stay low this long to count; filters the contact bounce of the button.
const DEBOUNCE: Duration = Duration::from_millis(20);

/// How often the accelerometer is read and logged; matches its 50 Hz output data rate.
const ACCEL_PERIOD: Duration = Duration::from_millis(20);

// Connects the TWISPI0 interrupt (shared by the chip's I2C and SPI peripheral 0) to
// embassy-nrf's TWIM handler, which wakes the task waiting for a transfer to finish.
bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

/// RAM copy buffer for I2C writes from flash, see `main`.
static I2C_TX_BUFFER: StaticCell<[u8; 8]> = StaticCell::new();

/// The I2C driver type the accelerometer task owns.
type Accel =
    Lsm303agr<lsm303agr::interface::I2cInterface<twim::Twim<'static>>, lsm303agr::mode::MagOneShot>;

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

    // Internal I2C at 400 kHz (the LSM303AGR's fast mode). TWIM transfers by DMA, which
    // can only read RAM, but the driver writes register addresses from constant slices,
    // which the compiler puts in flash. The driver copies such writes into
    // `tx_ram_buffer` first; it must hold the longest write (a register address plus a
    // value: 2 bytes).
    let mut i2c_config = twim::Config::default();
    i2c_config.frequency = twim::Frequency::K400;
    let tx_ram_buffer = I2C_TX_BUFFER.init([0; 8]);
    let i2c = twim::Twim::new(p.TWISPI0, Irqs, p.P0_16, p.P0_08, i2c_config, tx_ram_buffer);
    match accel(Lsm303agr::new_with_i2c(i2c)) {
        Ok(task) => spawner.spawn(task),
        Err(e) => defmt::error!("accel task not started: {}", e),
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

/// Sets the accelerometer up (12-bit high-resolution mode, 50 Hz, default ±2 g range:
/// 1 mg per LSB) and then logs one sample every `ACCEL_PERIOD`. Stops on a setup error;
/// a failed read is logged and the next one tried.
#[embassy_executor::task]
async fn accel(mut sensor: Accel) {
    match sensor.accelerometer_id().await {
        Ok(id) if id.is_correct() => defmt::info!("LSM303AGR accelerometer found"),
        Ok(id) => defmt::warn!("unexpected accelerometer id {=u8:#x}", id.raw()),
        Err(e) => {
            defmt::error!("accelerometer not answering: {}", defmt::Debug2Format(&e));
            return;
        }
    }
    // `init` turns on block data update: the high and low byte of a value always come
    // from the same sample (no torn values, the problem the robot's IMU driver had).
    if let Err(e) = sensor.init().await {
        defmt::error!("accelerometer init failed: {}", defmt::Debug2Format(&e));
        return;
    }
    // `Delay` waits (busy) for the mode change to settle, a few ms.
    if let Err(e) = sensor
        .set_accel_mode_and_odr(
            &mut Delay,
            AccelMode::HighResolution,
            AccelOutputDataRate::Hz50,
        )
        .await
    {
        defmt::error!("accelerometer mode failed: {}", defmt::Debug2Format(&e));
        return;
    }

    // A `Ticker` fires at fixed times (every 20 ms after the start), unlike
    // `Timer::after`, which would add the time spent reading and logging to each period.
    let mut ticker = Ticker::every(ACCEL_PERIOD);
    loop {
        ticker.next().await;
        match sensor.acceleration().await {
            Ok(a) => {
                let (x, y, z) = a.xyz_mg();
                defmt::info!("accel mg: {=i32}, {=i32}, {=i32}", x, y, z);
            }
            Err(e) => defmt::warn!("accelerometer read failed: {}", defmt::Debug2Format(&e)),
        }
    }
}
