//! BBC micro:bit v2 tilt remote for the balance robot. For now only a skeleton: it logs
//! "hello" over defmt/RTT once at startup and then a tick counter.
//!
//! No HAL yet: the delay is `cortex_m::asm::delay`, a busy loop that burns *at least* the
//! given number of CPU cycles. The nRF52833 runs at 64 MHz, so 64 million cycles would be
//! a second if each loop iteration took its nominal cycles, but measured on the micro:bit
//! a tick takes ~7.1 s: the loop runs from flash, and with the flash cache off (as after
//! reset) every instruction fetch waits for the flash. A busy loop is no clock; the next
//! step uses Embassy timers, which count a hardware timer instead.

#![no_std]
#![no_main]

// Linked only for their side effects: the RTT transport defmt writes into, and the
// panic handler. `as _` keeps them in the binary without bringing any names into scope.
use defmt_rtt as _;
use panic_probe as _;

/// CPU cycles to burn per tick: one second at 64 MHz if the loop ran at full speed.
const CYCLES_PER_TICK: u32 = 64_000_000;

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("hello from the micro:bit remote");
    let mut ticks: u32 = 0;
    loop {
        cortex_m::asm::delay(CYCLES_PER_TICK);
        ticks += 1;
        defmt::info!("tick {}", ticks);
    }
}
