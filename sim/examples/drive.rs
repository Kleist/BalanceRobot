//! Compares what sits between the controller and the deadband motor: nothing,
//! the deadband compensation, and battery scaling at different battery
//! voltages (`balance_core::drive`).
//!
//! `cargo run --release -p sim --example drive`
//!
//! Each variant runs from 0.5°, 2°, 5° and 10° with 5 noise seeds, 10 s
//! each. Printed: how many runs fell; the mean and largest drift (|axle
//! position| after 10 s); how many runs stick-slip (≥ 5 wheel stops, i.e.
//! moving → standing transitions, in the last 5 s); and the largest RMS pitch
//! over the last 5 s.

use balance_core::{
    control::Config,
    drive::{Compensation, Drive},
};
use sim::{
    closed_loop::{Run, Sim},
    imu::ImuParams,
    motor::Motor,
    pendulum::Body,
};

const STARTS_DEG: [f64; 4] = [0.5, 2.0, 5.0, 10.0];
const SEEDS: [u64; 5] = [1, 2, 3, 4, 5];

/// Wheel stops (moving → standing) after `from_s`.
fn stops(run: &Run, from_s: f64) -> usize {
    run.ticks
        .windows(2)
        .filter(|w| {
            w[0].t_s >= from_s
                && w[0].axle_vel_m_s.abs() >= 0.005
                && w[1].axle_vel_m_s.abs() < 0.005
        })
        .count()
}

/// RMS of the true pitch after `from_s`, in degrees.
fn rms_pitch_deg(run: &Run, from_s: f64) -> f64 {
    let p: Vec<f64> = run
        .ticks
        .iter()
        .filter(|t| t.t_s >= from_s)
        .map(|t| t.pitch_deg)
        .collect();
    #[allow(clippy::cast_precision_loss, reason = "a few thousand samples")]
    let n = p.len().max(1) as f64;
    (p.iter().map(|x| x * x).sum::<f64>() / n).sqrt()
}

fn row(name: &str, motor: Motor, drive: Option<Drive>) {
    let (mut fell, mut stick_slip) = (0, 0);
    let (mut drift_sum, mut drift_max, mut rms_max) = (0.0, 0.0_f64, 0.0_f64);
    let mut up = 0;
    for start in STARTS_DEG {
        for seed in SEEDS {
            let Ok(sim) = Sim::new(
                Body::prototype(),
                motor,
                ImuParams::prototype(),
                Config::PROTOTYPE.gains,
                start,
                seed,
            ) else {
                eprintln!("gyro calibration failed");
                std::process::exit(1);
            };
            let mut sim = match drive {
                Some(d) => sim.with_drive(d),
                None => sim,
            };
            let run = sim.run(10.0);
            if run.fell_at_s.is_some() {
                fell += 1;
                continue;
            }
            up += 1;
            let drift = run.last().map_or(0.0, |t| t.axle_pos_m.abs());
            drift_sum += drift;
            drift_max = drift_max.max(drift);
            rms_max = rms_max.max(rms_pitch_deg(&run, 5.0));
            if stops(&run, 5.0) >= 5 {
                stick_slip += 1;
            }
        }
    }
    let runs = STARTS_DEG.len() * SEEDS.len();
    let drift_mean = drift_sum / f64::from(up.max(1));
    println!(
        "{name:<36} {fell:>2}/{runs}  {drift_mean:>5.2}  {drift_max:>5.2}  {stick_slip:>5}  {rms_max:>5.2}"
    );
}

fn main() {
    println!(
        "{:<36} {:>5}  {:>5}  {:>5}  {:>5}  {:>5}",
        "variant", "fell", "drift", "max", "s-s", "rms"
    );
    println!(
        "{:<36} {:>5}  {:>5}  {:>5}  {:>5}  {:>5}",
        "", "", "m", "m", "runs", "deg"
    );
    row("ideal motor", Motor::ideal(), None);
    row("deadband, no compensation", Motor::prototype(), None);
    row(
        "deadband, compensation",
        Motor::prototype(),
        Some(Drive::PROTOTYPE),
    );
    for v in [6.4_f32, 6.8, 8.4] {
        let motor = Motor::prototype().at_battery(f64::from(v));
        // v_ref = the battery voltage: no scaling.
        let unscaled = Drive {
            compensation: Compensation::PROTOTYPE,
            v_ref: v,
        };
        row(
            &format!("{v} V, compensation, no scaling"),
            motor,
            Some(unscaled),
        );
        row(
            &format!("{v} V, compensation + scaling"),
            motor,
            Some(Drive::PROTOTYPE),
        );
    }
}
