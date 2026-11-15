//! Compare balance attempts from the robot's log with the simulation of the
//! same start (bring-up step C4).
//!
//! ```text
//! cargo run --release -p sim --example compare -- [OPTIONS] [LOG] < robot.log
//! ```
//!
//! Reads the log from `LOG` or stdin, splits it into attempts (`# safety:
//! motors on` to `motors off`, see `sim::robot_log`), and for each one runs
//! the simulation from the same start pitch at the same battery voltage: the
//! prototype body, IMU and deadband motor, with `Drive::PROTOTYPE` and
//! `Config::PROTOTYPE` gains, as the firmware runs. Prints the two traces
//! side by side and a summary.
//!
//! Options:
//!
//! - `--attempt N`: only attempt N (1 = first).
//! - `--every N`: print every Nth logged row (default 5: 10 rows/s at the
//!   50 Hz log rate). `--every 0` prints only the summaries.
//! - `--seed N`: the simulated IMU's noise (default 1).
//! - `--gains KP KI KD`: other gains than `Config::PROTOTYPE`.
//! - motor parameters to fit the simulation to the robot: `--vmax M_S`
//!   (0.41), `--lag-ms MS` (50), `--dead-ms MS` (0: dead time before the
//!   motor reacts at all, in whole 2 ms controller periods),
//!   `--start-duty D` and `--stop-duty D` (0.19 / 0.08, as fractions),
//!   `--ideal` (no deadband).
//! - `--synthetic START_DEG`: instead of reading a log, make one with the
//!   simulation (from `START_DEG`, prototype motor, seed 2) and compare that:
//!   a check of the tool itself.
//!
//! The pitch columns are the complementary-filter pitch for both, the
//! robot's estimate, so they compare like for like (the true simulated pitch
//! is in the `true` column). The simulation starts with the robot held still
//! at the start pitch; the real robot was let go from a hand, possibly still
//! moving, so the first few tenths of a second differ most.

use std::io::Read;

use balance_core::{
    control::{Config, Gains},
    drive::Drive,
};
use sim::{
    closed_loop::{Run, Sim},
    imu::ImuParams,
    motor::{Deadband, Motor},
    pendulum::Body,
    robot_log::{Attempt, End, Sample, Stats, at, sim_trace, split_attempts, synthetic_log},
};

/// Longest simulated attempt in s.
const MAX_S: f64 = 10.0;

struct Options {
    attempt: Option<usize>,
    every: usize,
    seed: u64,
    gains: Gains,
    motor: Motor,
    dead_time_ticks: usize,
    synthetic: Option<f64>,
    path: Option<String>,
}

fn parse_args() -> Result<Options, String> {
    let mut o = Options {
        attempt: None,
        every: 5,
        seed: 1,
        gains: Config::PROTOTYPE.gains,
        motor: Motor::prototype(),
        dead_time_ticks: 0,
        synthetic: None,
        path: None,
    };
    let mut args = std::env::args().skip(1);
    let mut deadband = Deadband::PROTOTYPE;
    let mut ideal = false;
    while let Some(arg) = args.next() {
        let mut num = |name: &str| -> Result<f64, String> {
            args.next()
                .and_then(|v| v.parse().ok())
                .ok_or(format!("{name} needs a number"))
        };
        match arg.as_str() {
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "small whole numbers from the command line"
            )]
            "--attempt" => o.attempt = Some(num("--attempt")? as usize),
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "small whole numbers from the command line"
            )]
            "--every" => o.every = num("--every")? as usize,
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "small whole numbers from the command line"
            )]
            "--seed" => o.seed = num("--seed")? as u64,
            #[allow(clippy::cast_possible_truncation, reason = "gains are small numbers")]
            "--gains" => {
                o.gains = Gains {
                    kp: num("--gains")? as f32,
                    ki: num("--gains")? as f32,
                    kd: num("--gains")? as f32,
                };
            }
            "--vmax" => o.motor.max_speed_m_s = num("--vmax")?,
            "--lag-ms" => o.motor.time_constant_s = num("--lag-ms")? * 1e-3,
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "a few tens of ms from the command line"
            )]
            "--dead-ms" => o.dead_time_ticks = (num("--dead-ms")? / 2.0).round() as usize,
            "--start-duty" => deadband.start_duty = num("--start-duty")?,
            "--stop-duty" => deadband.stop_duty = num("--stop-duty")?,
            "--ideal" => ideal = true,
            "--synthetic" => o.synthetic = Some(num("--synthetic")?),
            "-h" | "--help" => return Err(String::new()),
            path if !path.starts_with('-') => o.path = Some(path.to_string()),
            other => return Err(format!("unknown option {other}")),
        }
    }
    o.motor.deadband = if ideal { None } else { Some(deadband) };
    Ok(o)
}

fn simulate(o: &Options, motor: Motor, start_deg: f64, duration_s: f64, seed: u64) -> Run {
    match Sim::new(
        Body::prototype(),
        motor,
        ImuParams::prototype(),
        o.gains,
        start_deg,
        seed,
    ) {
        Ok(sim) => sim
            .with_drive(Drive::PROTOTYPE)
            .with_dead_time_ticks(o.dead_time_ticks)
            .run(duration_s),
        Err(e) => {
            eprintln!("simulated gyro calibration failed: {e:?}");
            std::process::exit(1);
        }
    }
}

fn fmt_opt(v: Option<f64>, unit: &str) -> String {
    v.map_or("-".to_string(), |v| format!("{v:.2} {unit}"))
}

fn print_summary(robot: &[Sample], robot_fell: Option<f64>, sim: &[Sample], sim_fell: Option<f64>) {
    let (Some(r), Some(s)) = (Stats::of(robot), Stats::of(sim)) else {
        return;
    };
    println!("                         robot        sim");
    let row = |name: &str, a: String, b: String| println!("{name:<20} {a:>10} {b:>10}");
    row(
        "fell after",
        fmt_opt(robot_fell, "s"),
        fmt_opt(sim_fell, "s"),
    );
    row(
        "duration",
        format!("{:.2} s", r.duration_s),
        format!("{:.2} s", s.duration_s),
    );
    row(
        "RMS pitch",
        format!("{:.2}°", r.rms_pitch_deg),
        format!("{:.2}°", s.rms_pitch_deg),
    );
    row(
        "max |pitch|",
        format!("{:.1}°", r.max_abs_pitch_deg),
        format!("{:.1}°", s.max_abs_pitch_deg),
    );
    row("period", fmt_opt(r.period_s, "s"), fmt_opt(s.period_s, "s"));
    row(
        "frequency",
        fmt_opt(r.period_s.map(|p| 1.0 / p), "Hz"),
        fmt_opt(s.period_s.map(|p| 1.0 / p), "Hz"),
    );
    row(
        "mean |cmd|",
        format!("{:.3}", r.mean_abs_cmd),
        format!("{:.3}", s.mean_abs_cmd),
    );
    row(
        "saturated",
        format!("{:.0} %", r.saturated * 100.0),
        format!("{:.0} %", s.saturated * 100.0),
    );
}

fn compare(o: &Options, index: usize, attempt: &Attempt) {
    let Some(start) = attempt.start_pitch_deg() else {
        return;
    };
    let battery_v = attempt.battery_v.unwrap_or(sim::motor::REFERENCE_V);
    let robot = attempt.trace();
    let robot_fell = (attempt.end == End::Fell).then(|| attempt.duration_s());
    let duration = attempt.duration_s().min(MAX_S);
    let run = simulate(o, o.motor.at_battery(battery_v), start, duration, o.seed);
    let sim = sim_trace(&run);
    let end = match attempt.end {
        End::Fell => "fell (cut-off)",
        End::Off => "motors off (battery)",
        End::LogEnd => "log ended",
    };
    println!(
        "== attempt {index}: start {start:+.1}°, battery {battery_v:.2} V, {} rows, {:.2} s, {end}",
        attempt.rows.len(),
        attempt.duration_s()
    );
    if o.every > 0 {
        println!("     t s   robot°     sim°    true°  robot cmd  sim cmd");
        for r in robot.iter().step_by(o.every).filter(|r| r.t_s <= duration) {
            let s = at(&sim, r.t_s);
            let true_pitch = run
                .ticks
                .get(run.ticks.partition_point(|t| t.t_s < r.t_s))
                .map(|t| t.pitch_deg);
            println!(
                "{:>8.2} {:>8.1} {:>8.1} {:>8.1} {:>10.3} {:>8.3}",
                r.t_s,
                r.pitch_deg,
                s.map_or(f64::NAN, |s| s.pitch_deg),
                true_pitch.unwrap_or(f64::NAN),
                r.motor_cmd,
                s.map_or(f64::NAN, |s| s.motor_cmd),
            );
        }
    }
    let robot: Vec<_> = robot.into_iter().filter(|r| r.t_s <= duration).collect();
    print_summary(&robot, robot_fell, &sim, run.fell_at_s);
    println!();
}

fn main() {
    let o = match parse_args() {
        Ok(o) => o,
        Err(e) => {
            if !e.is_empty() {
                eprintln!("{e}");
            }
            eprintln!(
                "usage: compare [--attempt N] [--every N] [--seed N] [--gains KP KI KD] \
                 [--vmax M_S] [--lag-ms MS] [--dead-ms MS] [--start-duty D] [--stop-duty D] [--ideal] \
                 [--synthetic START_DEG] [LOG]"
            );
            std::process::exit(2);
        }
    };
    let text = if let Some(start) = o.synthetic {
        let run = simulate(&o, Motor::prototype(), start, 5.0, 2);
        synthetic_log(&run, sim::motor::REFERENCE_V, 10)
    } else if let Some(path) = &o.path {
        match std::fs::read_to_string(path) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("{path}: {e}");
                std::process::exit(1);
            }
        }
    } else {
        let mut t = String::new();
        if let Err(e) = std::io::stdin().read_to_string(&mut t) {
            eprintln!("stdin: {e}");
            std::process::exit(1);
        }
        t
    };
    let attempts = split_attempts(text.lines());
    if attempts.is_empty() {
        eprintln!("no attempts found (no '# safety: motors on' followed by CSV rows)");
        std::process::exit(1);
    }
    println!(
        "{} attempt(s); sim: v_max {:.2} m/s, lag {:.0} ms, deadband {}, gains kp {} ki {} kd {}, seed {}\n",
        attempts.len(),
        o.motor.max_speed_m_s,
        o.motor.time_constant_s * 1e3,
        o.motor.deadband.map_or("none".to_string(), |d| format!(
            "start {:.0} % / stop {:.0} %",
            d.start_duty * 100.0,
            d.stop_duty * 100.0
        )),
        o.gains.kp,
        o.gains.ki,
        o.gains.kd,
        o.seed,
    );
    for (i, attempt) in attempts.iter().enumerate() {
        if o.attempt.is_none_or(|n| n == i + 1) {
            compare(&o, i + 1, attempt);
        }
    }
}
