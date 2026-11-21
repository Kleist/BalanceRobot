//! Reading a balance attempt back from the robot's serial log, to compare it
//! with the simulation of the same start (bring-up step C4).
//!
//! The firmware logs one CSV row per logged sample (see [`crate::log`] for
//! the columns) and comment lines starting with `#`. Three kinds of comment
//! matter here:
//!
//! - `# safety: motors on (...)`: the cut-offs allow the motors again (the
//!   robot was held within 5° of upright). An **attempt** starts here; the
//!   firmware resets the PID at this moment, so the controller starts fresh.
//! - `# safety: motors off (upright: false, ...)`: the fallen cut-off
//!   (|pitch| > 45°) ended the attempt. `upright: true` means something else
//!   (the battery) cut the motors.
//! - `# battery 7.60 V (pin 1886 mV)`: the latest battery voltage.
//!
//! Anything else (the CSV header, `# loop max ...`, the monitor's own output)
//! is skipped, so a log pasted straight from the terminal works.

use std::fmt::Write as _;

use crate::closed_loop::Run;

/// The columns of one CSV row that the comparison needs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Row {
    /// Firmware time in µs.
    pub t_us: u64,
    /// Complementary-filter pitch in degrees, relative to the balance point.
    pub filter_pitch_deg: f64,
    /// The controller's command, −1..=1.
    pub motor_cmd: f64,
    /// Duty applied to the `[left, right]` motor in signed percent.
    pub duty_pct: [f64; 2],
}

/// Number of columns in a firmware CSV row.
pub const COLUMNS: usize = 16;

/// Parses one CSV row, or `None` for anything else (comments, the header,
/// monitor output, a line cut off in the middle).
#[must_use]
pub fn parse_row(line: &str) -> Option<Row> {
    let fields: Vec<&str> = line.split(',').map(str::trim).collect();
    if fields.len() != COLUMNS {
        return None;
    }
    let f = |i: usize| fields.get(i).and_then(|v| v.parse::<f64>().ok());
    Some(Row {
        t_us: fields.first()?.parse().ok()?,
        filter_pitch_deg: f(12)?,
        motor_cmd: f(13)?,
        duty_pct: [f(14)?, f(15)?],
    })
}

/// How an attempt ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum End {
    /// The fallen cut-off (|pitch| > 45°).
    Fell,
    /// The motors went off for another reason (low battery).
    Off,
    /// The log ended while the motors were still on.
    LogEnd,
}

/// One balance attempt: from `motors on` to the next `motors off`.
#[derive(Debug, Clone, PartialEq)]
pub struct Attempt {
    /// The latest battery voltage logged before the attempt started.
    pub battery_v: Option<f64>,
    /// The CSV rows while the motors were on.
    pub rows: Vec<Row>,
    /// How it ended.
    pub end: End,
}

impl Attempt {
    /// Filter pitch at the first row, the start of the attempt.
    #[must_use]
    pub fn start_pitch_deg(&self) -> Option<f64> {
        self.rows.first().map(|r| r.filter_pitch_deg)
    }

    /// Time from the first to the last row in s.
    #[must_use]
    pub fn duration_s(&self) -> f64 {
        match (self.rows.first(), self.rows.last()) {
            #[allow(
                clippy::cast_precision_loss,
                reason = "µs over a log of hours stay far below 2^52"
            )]
            (Some(a), Some(b)) => (b.t_us - a.t_us) as f64 * 1e-6,
            _ => 0.0,
        }
    }

    /// The rows as a trace, with t in s since the first row.
    #[must_use]
    pub fn trace(&self) -> Vec<Sample> {
        let t0 = self.rows.first().map_or(0, |r| r.t_us);
        self.rows
            .iter()
            .map(|r| Sample {
                #[allow(
                    clippy::cast_precision_loss,
                    reason = "µs over a log of hours stay far below 2^52"
                )]
                t_s: (r.t_us - t0) as f64 * 1e-6,
                pitch_deg: r.filter_pitch_deg,
                motor_cmd: r.motor_cmd,
            })
            .collect()
    }
}

/// Splits a log into attempts. Attempts without any CSV row are dropped.
pub fn split_attempts<'a>(lines: impl IntoIterator<Item = &'a str>) -> Vec<Attempt> {
    let mut attempts = Vec::new();
    let mut battery_v = None;
    let mut current: Option<Attempt> = None;
    let finish = |attempt: Attempt, attempts: &mut Vec<Attempt>| {
        if !attempt.rows.is_empty() {
            attempts.push(attempt);
        }
    };
    for line in lines {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("# battery ") {
            battery_v = rest
                .split_whitespace()
                .next()
                .and_then(|v| v.parse().ok())
                .or(battery_v);
        } else if line.starts_with("# safety: motors on") {
            if let Some(previous) = current.take() {
                finish(previous, &mut attempts);
            }
            current = Some(Attempt {
                battery_v,
                rows: Vec::new(),
                end: End::LogEnd,
            });
        } else if line.starts_with("# safety: motors off") {
            if let Some(mut attempt) = current.take() {
                attempt.end = if line.contains("upright: false") {
                    End::Fell
                } else {
                    End::Off
                };
                finish(attempt, &mut attempts);
            }
        } else if let (Some(attempt), Some(row)) = (current.as_mut(), parse_row(line)) {
            attempt.rows.push(row);
        }
    }
    if let Some(attempt) = current {
        finish(attempt, &mut attempts);
    }
    attempts
}

/// One point of a trace, robot or simulated.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Time since the start of the attempt in s.
    pub t_s: f64,
    /// Pitch in degrees.
    pub pitch_deg: f64,
    /// Motor command, −1..=1.
    pub motor_cmd: f64,
}

/// A simulated run as a trace (filter pitch, the robot's estimate, so it
/// compares like for like with the log).
#[must_use]
pub fn sim_trace(run: &Run) -> Vec<Sample> {
    run.ticks
        .iter()
        .map(|t| Sample {
            t_s: t.t_s,
            pitch_deg: f64::from(t.filter_pitch_deg),
            motor_cmd: f64::from(t.motor_cmd),
        })
        .collect()
}

/// The sample of `trace` closest in time to `t_s` (traces are sorted).
#[must_use]
pub fn at(trace: &[Sample], t_s: f64) -> Option<Sample> {
    let i = trace.partition_point(|s| s.t_s < t_s);
    let after = trace.get(i);
    let before = i.checked_sub(1).and_then(|j| trace.get(j));
    match (before, after) {
        (Some(b), Some(a)) => Some(if t_s - b.t_s <= a.t_s - t_s { *b } else { *a }),
        (Some(s), None) | (None, Some(s)) => Some(*s),
        (None, None) => None,
    }
}

/// Summary numbers of a trace.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stats {
    /// Length in s.
    pub duration_s: f64,
    /// RMS pitch in degrees.
    pub rms_pitch_deg: f64,
    /// Largest |pitch| in degrees.
    pub max_abs_pitch_deg: f64,
    /// Oscillation period in s from the upward zero crossings of the pitch
    /// (with ±[`CROSSING_HYSTERESIS_DEG`] hysteresis, so noise around 0
    /// doesn't count), or `None` with fewer than two crossings.
    pub period_s: Option<f64>,
    /// Mean |motor command|.
    pub mean_abs_cmd: f64,
    /// Fraction of samples with |command| ≥ 0.99 (saturated).
    pub saturated: f64,
}

/// Hysteresis for counting zero crossings, in degrees.
pub const CROSSING_HYSTERESIS_DEG: f64 = 0.1;

impl Stats {
    /// The numbers for `trace`, or `None` when it is empty.
    #[must_use]
    pub fn of(trace: &[Sample]) -> Option<Self> {
        let first = trace.first()?;
        let last = trace.last()?;
        #[allow(
            clippy::cast_precision_loss,
            reason = "a trace has far fewer than 2^52 samples"
        )]
        let n = trace.len() as f64;
        let rms = (trace.iter().map(|s| s.pitch_deg * s.pitch_deg).sum::<f64>() / n).sqrt();
        let max = trace.iter().map(|s| s.pitch_deg.abs()).fold(0.0, f64::max);
        let mean_abs_cmd = trace.iter().map(|s| s.motor_cmd.abs()).sum::<f64>() / n;
        #[allow(
            clippy::cast_precision_loss,
            reason = "a trace has far fewer than 2^52 samples"
        )]
        let saturated = trace.iter().filter(|s| s.motor_cmd.abs() >= 0.99).count() as f64 / n;

        // Upward crossings: below −h, then above +h.
        let mut crossings = Vec::new();
        let mut below = false;
        for s in trace {
            if s.pitch_deg < -CROSSING_HYSTERESIS_DEG {
                below = true;
            } else if below && s.pitch_deg > CROSSING_HYSTERESIS_DEG {
                crossings.push(s.t_s);
                below = false;
            }
        }
        let period_s = match (crossings.first(), crossings.last()) {
            #[allow(
                clippy::cast_precision_loss,
                reason = "a trace has far fewer than 2^52 crossings"
            )]
            (Some(a), Some(b)) if crossings.len() >= 2 => {
                Some((b - a) / (crossings.len() - 1) as f64)
            }
            _ => None,
        };
        Some(Self {
            duration_s: last.t_s - first.t_s,
            rms_pitch_deg: rms,
            max_abs_pitch_deg: max,
            period_s,
            mean_abs_cmd,
            saturated,
        })
    }
}

/// A log in the firmware's format from a simulated run, as if the robot had
/// made that attempt: battery line, `motors on`, rows every `every` ticks,
/// then `motors off` if it fell. The raw IMU columns are 0 (the comparison
/// doesn't use them). For testing the comparison without a robot.
#[must_use]
pub fn synthetic_log(run: &Run, battery_v: f64, every: usize) -> String {
    let mut log = String::from("ESP-ROM: some monitor junk\n# gyro bias (deg/s): 0 0 0\n");
    let _ = writeln!(log, "# battery {battery_v:.2} V (pin 0 mV)");
    log.push_str(
        "# safety: motors on (upright: true, battery: Ok; pitch +0.0 deg, battery 7.40 V)\n",
    );
    let start_us = 10_000_000_u64;
    for tick in run.ticks.iter().step_by(every.max(1)) {
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "simulated time is positive and short"
        )]
        let t_us = start_us + (tick.t_s * 1e6).round() as u64;
        let duty = (tick.duty * 100.0).round();
        let _ = writeln!(
            log,
            "{t_us:>10},{:>6},{:>6},{:>6},{:>6},{:>6},{:>6},{:>9.1},{:>7.2},{:>7.2},{:>7.2},{:>14.1},{:>16.1},{:>9.3},{duty:>6},{duty:>6}",
            0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, tick.filter_pitch_deg, tick.motor_cmd
        );
    }
    if run.fell_at_s.is_some() {
        log.push_str(
            "# safety: motors off (upright: false, battery: Ok; pitch +45.1 deg, battery 7.40 V)\n",
        );
    }
    log
}

#[cfg(test)]
mod tests {
    use balance_core::{control::Config, drive::Drive};

    use super::{End, Stats, at, parse_row, sim_trace, split_attempts, synthetic_log};
    use crate::{closed_loop::Sim, imu::ImuParams, motor::Motor, pendulum::Body};

    const ROW: &str = "  32657419,  8049,   420,  2943,     0,     5,    77,    -16.3,   0.12,   0.34,   1.94,         -19.3,           -17.1,   -0.514,   -51,   -51";

    #[test]
    fn parses_a_firmware_row() {
        let row = parse_row(ROW).unwrap();
        assert_eq!(row.t_us, 32_657_419);
        assert!((row.filter_pitch_deg + 17.1).abs() < 1e-9);
        assert!((row.motor_cmd + 0.514).abs() < 1e-9);
        assert!(row.duty_pct.iter().all(|d| (d + 51.0).abs() < 1e-9));
    }

    #[test]
    fn skips_everything_else() {
        assert_eq!(parse_row("# loop max us: busy 1170 of 2000"), None);
        assert_eq!(parse_row("      t_us,    ax,    ay"), None);
        assert_eq!(parse_row("  32657419,  8049,   420,  29"), None); // cut off
        assert_eq!(parse_row("[INFO] Flashing has completed!"), None);
    }

    #[test]
    fn splits_attempts_and_tells_how_they_ended() {
        let log = format!(
            "junk\n# battery 7.60 V (pin 1886 mV)\n{ROW}\n\
             # safety: motors on (upright: true, battery: Ok; pitch +4.9 deg, battery 7.60 V)\n\
             {ROW}\n{ROW}\n\
             # safety: motors off (upright: false, battery: Ok; pitch +45.2 deg, battery 7.55 V)\n\
             {ROW}\n# battery 7.50 V (pin 1860 mV)\n\
             # safety: motors on (upright: true, battery: Ok; pitch -4.8 deg, battery 7.50 V)\n\
             {ROW}\n"
        );
        let attempts = split_attempts(log.lines());
        assert_eq!(attempts.len(), 2);
        assert_eq!(attempts[0].rows.len(), 2);
        assert_eq!(attempts[0].end, End::Fell);
        assert_eq!(attempts[0].battery_v, Some(7.6));
        assert_eq!(attempts[1].rows.len(), 1);
        assert_eq!(attempts[1].end, End::LogEnd);
        assert_eq!(attempts[1].battery_v, Some(7.5));
    }

    #[test]
    fn an_empty_attempt_is_dropped() {
        let log = "# safety: motors on (x)\n# safety: motors off (upright: true, battery: Empty)\n";
        assert!(split_attempts(log.lines()).is_empty());
    }

    #[test]
    fn period_from_zero_crossings() {
        // A 2 Hz sine of 1° amplitude, sampled at 50 Hz for 3 s.
        let trace: Vec<_> = (0..150)
            .map(|i| {
                let t = f64::from(i) * 0.02;
                super::Sample {
                    t_s: t,
                    pitch_deg: (2.0 * std::f64::consts::TAU * t).sin(),
                    motor_cmd: 1.0,
                }
            })
            .collect();
        let stats = Stats::of(&trace).unwrap();
        assert!((stats.period_s.unwrap() - 0.5).abs() < 0.02, "{stats:?}");
        assert!((stats.rms_pitch_deg - 0.707).abs() < 0.02, "{stats:?}");
        assert!((stats.saturated - 1.0).abs() < 1e-9);
    }

    #[test]
    fn a_synthetic_log_compares_with_its_own_simulation() {
        // The tool's own check: a log made by the simulation, read back and
        // simulated again from its start, gives (almost) the same trace. Not
        // exactly: the re-run starts from the logged (rounded, filtered)
        // start pitch, not the true one.
        let run = |start| {
            Sim::new(
                Body::prototype(),
                Motor::prototype(),
                ImuParams::prototype(),
                Config::PROTOTYPE.gains,
                start,
                1,
            )
            .unwrap()
            .with_drive(Drive::PROTOTYPE)
            .run(3.0)
        };
        let original = run(3.0);
        let log = synthetic_log(&original, 7.4, 10);
        let attempts = split_attempts(log.lines());
        assert_eq!(attempts.len(), 1);
        let attempt = &attempts[0];
        assert_eq!(attempt.rows.len(), original.ticks.len().div_ceil(10));
        let again = sim_trace(&run(attempt.start_pitch_deg().unwrap()));
        let robot = attempt.trace();
        // The catch from 3° (first 0.6 s) follows closely; after that the
        // stick-slip phase amplifies the small start difference (with the
        // speed-feedback gains it starts at ~0.7 s), so only the summary
        // numbers are compared.
        let worst = robot
            .iter()
            .filter(|r| r.t_s < 0.6)
            .map(|r| (r.pitch_deg - at(&again, r.t_s).unwrap().pitch_deg).abs())
            .fold(0.0, f64::max);
        assert!(worst < 0.2, "{worst}");
        let (a, b) = (Stats::of(&robot).unwrap(), Stats::of(&again).unwrap());
        assert!(
            (a.rms_pitch_deg - b.rms_pitch_deg).abs() < 0.15,
            "{a:?} {b:?}"
        );
        assert!(
            (a.mean_abs_cmd - b.mean_abs_cmd).abs() < 0.05,
            "{a:?} {b:?}"
        );
    }
}
