//! Grid search for PID gains in simulation.
//!
//! `cargo run --release -p sim --example tune`
//!
//! Each candidate is let go at 5° and run for 10 s. It must stay up on the
//! prototype's motor model *and* on variations of it (motor half as fast or
//! with twice the lag, latency doubled), because the model parameters are
//! partly guesses.
//!
//! Survivors all hold the tilt within the IMU's noise, so they are ranked by
//! **drift** instead: the worst-case axle speed in the last 5 s. Without
//! encoders the controller can't see the wheel speed, so some drift is
//! unavoidable; less is better (and less likely to run into the motors'
//! speed limit).

use balance_core::control::Gains;
use sim::{closed_loop::Sim, imu::ImuParams, motor::Motor, pendulum::Body};

/// Worst case over the motor variants: (largest |pitch| in degrees, largest
/// |axle speed| in m/s), both over the last 5 s. `None` if any variant fell.
fn worst_case(gains: Gains) -> Option<(f64, f64)> {
    let base = Motor::prototype();
    let variants = [
        base,
        Motor {
            max_speed_m_s: base.max_speed_m_s * 0.6,
            ..base
        },
        Motor {
            max_speed_m_s: base.max_speed_m_s * 1.4,
            ..base
        },
        Motor {
            time_constant_s: base.time_constant_s * 2.0,
            ..base
        },
        Motor {
            time_constant_s: base.time_constant_s * 0.5,
            ..base
        },
    ];
    let (mut pitch, mut speed): (f64, f64) = (0.0, 0.0);
    // A different IMU noise seed per variant.
    for (seed, motor) in (1_u64..).zip(variants) {
        let mut sim = Sim::new(
            Body::prototype(),
            motor,
            ImuParams::prototype(),
            gains,
            5.0,
            seed,
        )
        .ok()?;
        let run = sim.run(10.0);
        if run.fell_at_s.is_some() {
            return None;
        }
        pitch = pitch.max(run.max_abs_pitch_deg(5.0));
        speed = speed.max(run.max_abs_speed_m_s(5.0));
    }
    Some((pitch, speed))
}

fn main() {
    let mut results = Vec::new();
    // kp 0.02..=0.40, ki 0..=8, kd 0..=0.02
    let steps = |n: u8, step: f32| (0..=n).map(move |i| step * f32::from(i));
    for kp in steps(19, 0.02).map(|kp| kp + 0.02) {
        for ki in steps(16, 0.5) {
            for kd in steps(10, 0.002) {
                let gains = Gains { kp, ki, kd };
                if let Some(worst) = worst_case(gains) {
                    results.push((worst, gains));
                }
            }
        }
    }
    results.sort_by(|a, b| a.0.1.total_cmp(&b.0.1));
    println!(
        "{} gain sets survive all variants; least drift:",
        results.len()
    );
    println!("   kp     ki     kd   worst |pitch|  worst |speed|  (last 5 s)");
    for ((pitch, speed), Gains { kp, ki, kd }) in results.iter().take(20) {
        println!("{kp:>5.2} {ki:>6.2} {kd:>6.3}   {pitch:>8.3}°   {speed:>8.3} m/s");
    }
}
