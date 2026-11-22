//! BBC micro:bit v2 tilt remote for the balance robot. So far: the accelerometer is read
//! at 50 Hz and turned into a speed / turn command (`balance_core::remote`), which the
//! 5x5 LED matrix shows as one dot. Every sample also becomes a packet
//! (`balance_core::link`) that goes to the robot over BLE (`ble`). While not connected
//! the dot blinks.
//!
//! Hold it upright in front of you, LEDs facing you, USB edge up, at whatever tilt you
//! like, and press and hold **both** buttons (the grip): that attitude becomes the
//! reference. Tip the top edge further away for forward (the dot moves up), towards you
//! for backward, lower the right side to turn right (the dot moves right). Within ±5° of
//! the reference the dot stays in the centre. Let go of either button and the command
//! is 0 (dot in the centre) whatever the tilt.
//!
//! The LED matrix is wired as 5 row and 5 column lines (micro:bit v2 schematic): an LED
//! lights when its row is driven high and its column low. A pattern needs the rows
//! scanned fast, one at a time; one dot doesn't: one row high, one column low.
//!
//! Buttons A (P0.14) and B (P0.23) have pull-up resistors on the board and pull the pin
//! low while pressed. `wait_for_low` is async: the GPIOTE peripheral raises an interrupt
//! on the edge, so the task sleeps until then instead of polling. Each button task
//! publishes its debounced state in an `AtomicBool` (`A_HELD`, `B_HELD`), which the tilt
//! task reads every sample.
//!
//! The LSM303AGR (accelerometer + magnetometer) sits on the micro:bit's internal I2C bus
//! (SCL P0.08, SDA P0.16, pull-ups on the board), separate from the edge connector's.
//! The accelerometer answers at address 0x19. The log shows `grip on` (with the
//! reference) and `grip off`, and while the grip is held, five times a second
//! `tilt: pitch .. roll .. rel .. .. -> speed .. turn .. (seq ..)` (attitude, relative
//! tilt, command, packet number). A packet is built and sent every sample (50 per
//! second, `seq` counting up).
//!
//! Interrupt priorities: MPSL (under the BLE controller) owns priority 0 for the radio's
//! timing and uses 4 for its low-priority work; everything else here runs at 2, so it
//! can never delay the radio. embassy-nrf drivers don't set a priority themselves, and
//! the reset default is 0, so `main` sets each one before creating the driver.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use balance_core::{
    link::Packet,
    remote::{self, Config, GripChange, TiltRemote},
};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::{
    Peri, bind_interrupts,
    gpio::{AnyPin, Input, Level, Output, OutputDrive, Pull},
    interrupt::{self, InterruptExt, Priority},
    peripherals, rng, twim,
};
use embassy_time::{Delay, Duration, Ticker, Timer};
use lsm303agr::{AccelMode, AccelOutputDataRate, Lsm303agr};
use panic_probe as _;
use static_cell::StaticCell;

mod ble;

/// A press must stay low this long to count; filters the contact bounce of the button.
const DEBOUNCE: Duration = Duration::from_millis(20);

/// How often the accelerometer is read; matches its 50 Hz output data rate.
const ACCEL_PERIOD: Duration = Duration::from_millis(20);
/// `ACCEL_PERIOD` in seconds, for the filter.
const ACCEL_PERIOD_S: f32 = 0.02;
/// Log one line every this many samples (5 per second).
const LOG_EVERY_N_SAMPLES: u32 = 10;
/// While not connected the dot blinks: on for this many samples, then off as long
/// (0.25 s each).
const BLINK_SAMPLES: u32 = 12;

// Connects each interrupt to its handler. TWISPI0 (shared by the chip's I2C and SPI
// peripheral 0) and RNG: embassy-nrf's handlers, which wake the task waiting for a
// transfer / random bytes. The rest belong to MPSL (see `ble`): EGU0_SWI0 is a software
// interrupt for its low-priority work, CLOCK_POWER tells it when a clock has started,
// RADIO, TIMER0 and RTC0 drive the radio's timing.
bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    RNG => rng::InterruptHandler<peripherals::RNG>;
    EGU0_SWI0 => nrf_mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
});

/// Debounced state of button A / B, written by its `button` task. An atomic, so two
/// tasks can share it without a lock: the Cortex-M4 loads and stores a byte in one
/// instruction, and `Relaxed` is enough because nothing else is ordered by it.
static A_HELD: AtomicBool = AtomicBool::new(false);
/// See [`A_HELD`].
static B_HELD: AtomicBool = AtomicBool::new(false);

/// RAM copy buffer for I2C writes from flash, see `main`.
static I2C_TX_BUFFER: StaticCell<[u8; 8]> = StaticCell::new();

/// The I2C driver type the tilt task owns.
type Accel =
    Lsm303agr<lsm303agr::interface::I2cInterface<twim::Twim<'static>>, lsm303agr::mode::MagOneShot>;

/// The 5x5 LED matrix, showing at most one lit LED.
struct Display {
    /// Rows 1-5, top (USB edge) to bottom; high = on.
    rows: [Output<'static>; 5],
    /// Columns 1-5, left (button A) to right; low = on.
    cols: [Output<'static>; 5],
}

impl Display {
    /// Lights the LED at `row`, `col` (0..=4, from the top left) and turns every other
    /// one off: its row low or its column high (or both) keeps it dark. Out-of-range
    /// indices light nothing in that direction.
    fn show_dot(&mut self, row: usize, col: usize) {
        for (i, pin) in self.rows.iter_mut().enumerate() {
            pin.set_level(if i == row { Level::High } else { Level::Low });
        }
        for (i, pin) in self.cols.iter_mut().enumerate() {
            pin.set_level(if i == col { Level::Low } else { Level::High });
        }
    }
}

/// `#[embassy_executor::main]` creates the executor and runs this as its first task.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Clocks, GPIOTE and the RTC1 time driver; returns every peripheral once, as an
    // owned singleton. GPIOTE and the time driver at priority 2, below MPSL's 0.
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);
    defmt::info!("hello from the micro:bit remote (Embassy)");

    interrupt::RNG.set_priority(Priority::P2);
    ble::start(
        spawner,
        ble::Hardware {
            mpsl: nrf_mpsl::Peripherals::new(
                p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31,
            ),
            sdc: nrf_sdc::Peripherals::new(
                p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
                p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
            ),
            rng: p.RNG,
        },
    );

    // Pins from the micro:bit v2 schematic. All LEDs start off: rows low, columns high.
    // `.into()` turns each specific pin type (`P0_21`, ...) into the type-erased `AnyPin`,
    // so pins of different types fit in one array.
    let row = |pin: Peri<'static, AnyPin>| Output::new(pin, Level::Low, OutputDrive::Standard);
    let col = |pin: Peri<'static, AnyPin>| Output::new(pin, Level::High, OutputDrive::Standard);
    let display = Display {
        rows: [
            row(p.P0_21.into()),
            row(p.P0_22.into()),
            row(p.P0_15.into()),
            row(p.P0_24.into()),
            row(p.P0_19.into()),
        ],
        cols: [
            col(p.P0_28.into()),
            col(p.P0_11.into()),
            col(p.P0_31.into()),
            col(p.P1_05.into()),
            col(p.P0_30.into()),
        ],
    };

    // The board has external pull-ups; the internal one too does no harm.
    let buttons = [
        ("A", Input::new(p.P0_14, Pull::Up), &A_HELD),
        ("B", Input::new(p.P0_23, Pull::Up), &B_HELD),
    ];
    // Calling a task function only builds it; that fails if all its slots are in use.
    // `spawn` then hands it to the executor.
    for (name, pin, held) in buttons {
        match button(pin, held) {
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
    interrupt::TWISPI0.set_priority(Priority::P2);
    let i2c = twim::Twim::new(p.TWISPI0, Irqs, p.P0_16, p.P0_08, i2c_config, tx_ram_buffer);
    match tilt(Lsm303agr::new_with_i2c(i2c), display) {
        Ok(task) => spawner.spawn(task),
        Err(e) => defmt::error!("tilt task not started: {}", e),
    }
}

/// Keeps `held` equal to the debounced state of one button. Two instances run, one per
/// button (`pool_size = 2`: the executor reserves memory for two copies of this task).
/// A change only counts once the pin has stayed at its new level for `DEBOUNCE`.
#[embassy_executor::task(pool_size = 2)]
async fn button(mut pin: Input<'static>, held: &'static AtomicBool) {
    loop {
        pin.wait_for_low().await;
        Timer::after(DEBOUNCE).await;
        if pin.is_high() {
            continue; // a bounce or a glitch, not a press
        }
        held.store(true, Ordering::Relaxed);
        loop {
            pin.wait_for_high().await;
            Timer::after(DEBOUNCE).await;
            if pin.is_high() {
                break;
            }
        }
        held.store(false, Ordering::Relaxed);
    }
}

/// Sets the accelerometer up (12-bit high-resolution mode, 50 Hz, default ±2 g range:
/// 1 mg per LSB), then every `ACCEL_PERIOD` reads it, updates the command and shows it
/// as a dot. Stops on a setup error; a failed read is logged and the next one tried.
#[embassy_executor::task]
async fn tilt(mut sensor: Accel, mut display: Display) {
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

    let mut remote = TiltRemote::new(Config::MICROBIT);
    let mut samples: u32 = 0;
    // Sequence number of the next packet; wraps 255 -> 0.
    let mut seq: u8 = 0;
    defmt::info!("hold A+B to drive");
    // A `Ticker` fires at fixed times (every 20 ms after the start), unlike
    // `Timer::after`, which would add the time spent reading to each period.
    let mut ticker = Ticker::every(ACCEL_PERIOD);
    loop {
        ticker.next().await;
        let (x, y, z) = match sensor.acceleration().await {
            Ok(a) => a.xyz_mg(),
            Err(e) => {
                defmt::warn!("accelerometer read failed: {}", defmt::Debug2Format(&e));
                continue;
            }
        };
        let both_held = A_HELD.load(Ordering::Relaxed) && B_HELD.load(Ordering::Relaxed);
        #[allow(
            clippy::cast_precision_loss,
            reason = "readings are within ±2000 mg, exact in f32"
        )]
        let update = remote.step([x as f32, y as f32, z as f32], ACCEL_PERIOD_S, both_held);
        samples += 1;

        match update.grip {
            GripChange::On { reference } => defmt::info!(
                "grip on: reference pitch {=f32} roll {=f32} deg",
                reference.pitch_deg,
                reference.roll_deg
            ),
            GripChange::Off => defmt::info!("grip off"),
            GripChange::None => {}
        }

        let (row, col) = remote::dot(update.command);
        let blink_off = !ble::connected() && (samples / BLINK_SAMPLES).is_multiple_of(2);
        if blink_off {
            display.show_dot(usize::MAX, usize::MAX); // nothing lit
        } else {
            display.show_dot(row, col);
        }

        let active = remote.grip().reference().is_some();
        let packet = Packet::new(seq, update.command, active);
        ble::send(packet);
        seq = seq.wrapping_add(1);

        // Only while the grip is held: otherwise the command is 0 anyway, and the BLE
        // lines (`packets/s`) should stand out.
        if active && samples.is_multiple_of(LOG_EVERY_N_SAMPLES) {
            defmt::info!(
                "tilt: pitch {=f32} roll {=f32} rel {=f32} {=f32} deg -> speed {=f32} turn {=f32} (seq {=u8})",
                update.attitude.pitch_deg,
                update.attitude.roll_deg,
                update.relative.pitch_deg,
                update.relative.roll_deg,
                update.command.speed,
                update.command.turn,
                packet.seq
            );
        }
    }
}
