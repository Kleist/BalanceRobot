//! Real time → simulation ticks.
//!
//! The browser calls us once per screen refresh (`requestAnimationFrame`, usually 60 Hz,
//! sometimes 120 Hz, and not at all while the tab is hidden). The simulation advances in
//! fixed 2 ms controller ticks, like the firmware. [`Clock`] turns the time between two
//! frames into a whole number of ticks and carries the remainder over to the next frame, so
//! over many frames the simulation runs exactly at real time whatever the refresh rate.

/// Longest stretch of real time one frame may simulate, in ms. After a hidden tab or a
/// debugger pause the gap can be minutes; catching up on it would freeze the page, so the
/// simulation just pauses for that time instead.
pub const MAX_FRAME_MS: f64 = 100.0;

/// Counts how many fixed-length ticks are due at each frame.
#[derive(Debug, Clone)]
/// All times in ms.
pub struct Clock {
    tick: f64,
    /// Timestamp of the previous frame.
    last: Option<f64>,
    /// Real time not yet simulated (always less than one tick after a frame).
    owed: f64,
}

impl Clock {
    /// A clock for ticks of `tick_ms` milliseconds.
    pub fn new(tick_ms: f64) -> Self {
        Self {
            tick: tick_ms,
            last: None,
            owed: 0.0,
        }
    }

    /// The number of ticks to simulate at a frame drawn at `now_ms` (the timestamp
    /// `requestAnimationFrame` passes, in ms). The first frame only starts the clock.
    pub fn ticks_due(&mut self, now_ms: f64) -> u32 {
        let elapsed = self.last.map_or(0.0, |last| now_ms - last);
        self.last = Some(now_ms);
        self.owed += elapsed.clamp(0.0, MAX_FRAME_MS);
        let ticks = (self.owed / self.tick).floor();
        self.owed -= ticks * self.tick;
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "0 ≤ ticks ≤ MAX_FRAME_MS / tick_ms, a small whole number"
        )]
        let ticks = ticks as u32;
        ticks
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_frame_only_starts_the_clock() {
        let mut clock = Clock::new(2.0);
        assert_eq!(clock.ticks_due(5000.0), 0);
    }

    #[test]
    fn runs_at_real_time_over_many_frames() {
        // 60 Hz frames are 16.67 ms: 8 ticks, sometimes 9, never a whole number.
        let mut clock = Clock::new(2.0);
        let frame_ms = 1000.0 / 60.0;
        let total: u32 = (0..=60)
            .map(|i| clock.ticks_due(f64::from(i) * frame_ms))
            .sum();
        assert_eq!(total, 500, "one second = 500 ticks of 2 ms");
    }

    #[test]
    fn a_long_gap_is_skipped_not_caught_up() {
        let mut clock = Clock::new(2.0);
        clock.ticks_due(0.0);
        assert_eq!(clock.ticks_due(60_000.0), 50, "only MAX_FRAME_MS = 100 ms");
    }

    #[test]
    fn time_going_backwards_simulates_nothing() {
        let mut clock = Clock::new(2.0);
        clock.ticks_due(1000.0);
        assert_eq!(clock.ticks_due(900.0), 0);
        assert_eq!(clock.ticks_due(910.0), 5);
    }
}
