//! BBC micro:bit v2 tilt remote for the balance robot. For now only a skeleton: it logs
//! "hello" over defmt/RTT once at startup and then a counter once a second.
//!
//! No HAL yet: the delay is `cortex_m::asm::delay`, a busy loop that burns at least the
//! given number of CPU cycles. The nRF52833 runs at 64 MHz, so 64 million cycles are about
//! a second (a bit more: flash wait states slow the loop). Embassy (async tasks and
//! timers) comes in the next step.

#![no_std]
#![no_main]

// Linked only for their side effects: the RTT transport defmt writes into, and the
// panic handler. `as _` keeps them in the binary without bringing any names into scope.
use defmt_rtt as _;
use panic_probe as _;

/// CPU clock after reset: the 64 MHz internal oscillator.
const CPU_HZ: u32 = 64_000_000;

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("hello from the micro:bit remote");
    let mut seconds: u32 = 0;
    loop {
        cortex_m::asm::delay(CPU_HZ);
        seconds += 1;
        defmt::info!("alive for {} s", seconds);
    }
}
