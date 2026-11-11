//! Prints a simulated firmware IMU log (same CSV as the robot's serial log).
//!
//! - `cargo run -p sim --example imu_log`: held upright and still for 60 s,
//!   to compare with a still log of the real robot (`log_stats` example).
//! - `cargo run -p sim --example imu_log -- fall`: held at 2° for 1 s, then
//!   released (no controller) until it lies at 90°.
//!
//! Like the firmware, it first calibrates the gyro from 1000 still samples.
//! Unlike the firmware, it logs every sample (500 lines/s), not every 50th.

use sim::{
    imu::{Imu, ImuParams},
    log::{CALIBRATION_SAMPLES, DT_US, Pipeline, header},
    pendulum::{Body, State},
};

/// Physics steps per 2 ms IMU sample, of 0.1 ms each.
const SUBSTEPS: u32 = 20;
const PHYSICS_DT_S: f64 = 1e-4;

fn main() {
    let fall = std::env::args().nth(1).as_deref() == Some("fall");
    let body = Body::prototype();
    let mut imu = Imu::new(ImuParams::prototype(), 1);
    let held = if fall {
        State::tilted_deg(2.0)
    } else {
        State::default()
    };

    // Held in the hand: not moving, so no pitch acceleration either.
    let calibration = (0..CALIBRATION_SAMPLES).map(|_| imu.sample(&held, 0.0, 0.0));
    let mut pipeline = match Pipeline::calibrate(calibration.collect::<Vec<_>>()) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("gyro calibration rejected: {e:?}");
            std::process::exit(1);
        }
    };
    println!("{}", header(pipeline.bias()));

    let held_samples: u64 = if fall { 500 } else { 30_000 };
    let mut t_us = 0;
    for _ in 0..held_samples {
        println!("{}", pipeline.row(t_us, &imu.sample(&held, 0.0, 0.0)));
        t_us += DT_US;
    }
    if !fall {
        return;
    }

    let mut state = held;
    while state.pitch_deg() < 90.0 {
        for _ in 0..SUBSTEPS {
            state = state.step(&body, 0.0, PHYSICS_DT_S);
        }
        let pitch_accel = body.pitch_accel(state.pitch_rad, 0.0);
        println!(
            "{}",
            pipeline.row(t_us, &imu.sample(&state, pitch_accel, 0.0))
        );
        t_us += DT_US;
    }
}
