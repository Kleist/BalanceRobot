//! Runs the simulated robot with the given PID gains and prints a trace.
//!
//! `cargo run -p sim --example balance -- KP KI KD [START_DEG]`
//!
//! Prints CSV every 10 ms: time, true and estimated pitch, motor command,
//! axle speed and position, and whether/when it fell.

use balance_core::control::{Config, Gains};
use sim::closed_loop::Sim;

fn main() {
    let args: Vec<f32> = std::env::args()
        .skip(1)
        .filter_map(|a| a.parse().ok())
        .collect();
    let gains = match args.as_slice() {
        [kp, ki, kd, ..] => Gains {
            kp: *kp,
            ki: *ki,
            kd: *kd,
        },
        _ => Config::PROTOTYPE.gains,
    };
    let start_deg = args.get(3).copied().map_or(2.0, f64::from);
    let Ok(mut sim) = Sim::prototype(gains, start_deg) else {
        eprintln!("gyro calibration failed");
        std::process::exit(1);
    };
    let run = sim.run(10.0);
    println!(
        "# kp={} ki={} kd={}, let go at {start_deg}°",
        gains.kp, gains.ki, gains.kd
    );
    println!("t_s,pitch_deg,filter_pitch_deg,motor_cmd,axle_vel_m_s,axle_pos_m");
    for t in run.ticks.iter().step_by(5) {
        println!(
            "{:.2},{:.2},{:.2},{:.3},{:.3},{:.3}",
            t.t_s, t.pitch_deg, t.filter_pitch_deg, t.motor_cmd, t.axle_vel_m_s, t.axle_pos_m
        );
    }
    match run.fell_at_s {
        Some(t) => println!("# fell after {t:.2} s"),
        None => println!("# still up after 10 s"),
    }
}
