//! Grid search for the controller gains in simulation, including the speed
//! feedback `kv`.
//!
//! `cargo run --release -p sim --example tune`
//!
//! `... --example tune -- --check KP KI KD KV` instead stress-tests one gain
//! set on harder variants (more seeds, 1.5° balance error, 30 ms dead time,
//! motor ±30 % speed, low battery, other starts) and prints each result.
//!
//! Each candidate is let go at 4.5° and run for 10 s on the prototype's
//! deadband motor with `Drive::PROTOTYPE` in between, as on the robot. It must
//! stay up on every variant:
//!
//! - motor: the nominal model (50 ms lag), and a faster motor with dead time
//!   (30 ms lag + 20 ms, 15 ms lag + 20 ms), which reproduced the shaking of
//!   the first balance attempt (bring-up step C4);
//! - balance point: right, and 1° off either way. The second balance attempt
//!   showed that an error of a fraction of a degree is enough for the tilt-only
//!   PID to speed up until the motors saturate and the robot falls.
//!
//! Survivors are ranked by the worst-case **drift speed** in the last 5 s
//! (a balance-point error can only be turned into a steady drift, not
//! removed), then by the worst |pitch|.

use std::thread;

use balance_core::{control::Gains, drive::Drive};
use sim::{closed_loop::Sim, imu::ImuParams, motor::Motor, pendulum::Body};

/// One simulated situation: motor lag and dead time in ms, balance-point
/// error in degrees.
const VARIANTS: [(f64, usize, f64); 9] = [
    (50.0, 0, 0.0),
    (50.0, 0, 1.0),
    (50.0, 0, -1.0),
    (30.0, 20, 0.0),
    (30.0, 20, 1.0),
    (30.0, 20, -1.0),
    (15.0, 20, 0.0),
    (15.0, 20, 1.0),
    (15.0, 20, -1.0),
];

/// Worst case over [`VARIANTS`]: (largest |axle speed| in m/s, largest
/// |pitch| in degrees), both over the last 5 s. `None` if any variant fell.
fn worst_case(gains: Gains) -> Option<(f64, f64)> {
    let (mut speed, mut pitch): (f64, f64) = (0.0, 0.0);
    // A different IMU noise seed per variant.
    for (seed, (lag_ms, dead_ms, offset_deg)) in (1_u64..).zip(VARIANTS) {
        let motor = Motor {
            time_constant_s: lag_ms * 1e-3,
            ..Motor::prototype()
        }
        .at_battery(7.8);
        let imu = ImuParams {
            mount_tilt_deg: ImuParams::prototype().mount_tilt_deg + offset_deg,
            ..ImuParams::prototype()
        };
        let run = Sim::new(Body::prototype(), motor, imu, gains, 4.5, seed)
            .ok()?
            .with_drive(Drive::PROTOTYPE)
            .with_dead_time_ticks(dead_ms / 2)
            .run(10.0);
        if run.fell_at_s.is_some() {
            return None;
        }
        speed = speed.max(run.max_abs_speed_m_s(5.0));
        pitch = pitch.max(run.max_abs_pitch_deg(5.0));
    }
    Some((speed, pitch))
}

/// One stress-test run: (lag ms, dead ms, balance error °, speed factor,
/// battery V, start °, seed).
type Stress = (f64, usize, f64, f64, f64, f64, u64);

fn stress_runs() -> Vec<Stress> {
    let mut v = Vec::new();
    for (lag, dead) in [(50.0, 0), (30.0, 20), (15.0, 20), (30.0, 30), (80.0, 10)] {
        for offset in [-1.5, -1.0, 0.0, 1.0, 1.5] {
            for seed in 1..=3 {
                v.push((lag, dead, offset, 1.0, 7.8, 4.5, seed));
            }
        }
    }
    for speed in [0.7, 1.3] {
        for offset in [-1.0, 1.0] {
            v.push((30.0, 20, offset, speed, 7.8, 4.5, 4));
        }
    }
    for start in [1.0, 8.0, -6.0] {
        v.push((30.0, 20, 0.5, 1.0, 6.6, start, 5));
    }
    v
}

fn check(gains: Gains) {
    println!("lag dead  err  speed  batt  start  -> fell / max|v| last 5 s / max|pitch| / sat %");
    let mut fell = 0;
    let runs = stress_runs();
    for &(lag, dead, offset, speed_factor, battery, start, seed) in &runs {
        let base = Motor::prototype();
        let motor = Motor {
            time_constant_s: lag * 1e-3,
            max_speed_m_s: base.max_speed_m_s * speed_factor,
            ..base
        }
        .at_battery(battery);
        let imu = ImuParams {
            mount_tilt_deg: ImuParams::prototype().mount_tilt_deg + offset,
            ..ImuParams::prototype()
        };
        let Ok(sim) = Sim::new(Body::prototype(), motor, imu, gains, start, seed) else {
            println!("calibration failed");
            continue;
        };
        let run = sim
            .with_drive(Drive::PROTOTYPE)
            .with_dead_time_ticks(dead / 2)
            .run(10.0);
        #[allow(clippy::cast_precision_loss, reason = "a few thousand ticks")]
        let sat = run
            .ticks
            .iter()
            .filter(|t| t.motor_cmd.abs() > 0.99)
            .count() as f64
            / run.ticks.len() as f64;
        if run.fell_at_s.is_some() {
            fell += 1;
        }
        println!(
            "{lag:>3} {dead:>4} {offset:>+4.1} {speed_factor:>6.1} {battery:>5.1} {start:>6.1}  -> {:>6} {:>6.3} m/s {:>6.2}° {:>4.0} %",
            run.fell_at_s
                .map_or("-".to_string(), |t| format!("{t:.1} s")),
            run.max_abs_speed_m_s(5.0),
            run.max_abs_pitch_deg(5.0),
            sat * 100.0
        );
    }
    println!("fell in {fell} of {} runs", runs.len());
}

fn main() {
    let args: Vec<f32> = std::env::args()
        .skip(2)
        .filter_map(|a| a.parse().ok())
        .collect();
    if std::env::args().nth(1).as_deref() == Some("--check") {
        if let [kp, ki, kd, kv] = args[..] {
            check(Gains { kp, ki, kd, kv });
        } else {
            eprintln!("usage: tune [--check KP KI KD KV]");
        }
        return;
    }
    let mut grid = Vec::new();
    for kp in [0.06, 0.09, 0.12, 0.15, 0.2] {
        for ki in [0.5, 1.0, 2.0, 3.0] {
            for kd in [0.002, 0.003, 0.004, 0.006] {
                for kv in [-16.0, -8.0, -4.0, -2.0, 0.0, 2.0, 4.0, 8.0, 16.0] {
                    grid.push(Gains { kp, ki, kd, kv });
                }
            }
        }
    }
    // Split the grid over the CPU cores.
    let cores = thread::available_parallelism().map_or(1, usize::from);
    let chunk = grid.len().div_ceil(cores);
    let mut results: Vec<((f64, f64), Gains)> = thread::scope(|s| {
        let handles: Vec<_> = grid
            .chunks(chunk)
            .map(|part| {
                s.spawn(move || {
                    part.iter()
                        .filter_map(|&g| worst_case(g).map(|w| (w, g)))
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        handles
            .into_iter()
            .flat_map(|h| h.join().unwrap_or_default())
            .collect()
    });
    results.sort_by(|a, b| a.0.0.total_cmp(&b.0.0).then(a.0.1.total_cmp(&b.0.1)));
    println!(
        "{} of {} gain sets survive all {} variants; least drift:",
        results.len(),
        grid.len(),
        VARIANTS.len()
    );
    println!("   kp     ki     kd     kv   worst |speed|  worst |pitch|  (last 5 s)");
    for ((speed, pitch), Gains { kp, ki, kd, kv }) in results.iter().take(25) {
        println!("{kp:>5.2} {ki:>6.2} {kd:>6.3} {kv:>6.1}   {speed:>8.3} m/s   {pitch:>8.2}°");
    }
    let kv_counts: Vec<_> = [-16.0, -8.0, -4.0, -2.0, 0.0, 2.0, 4.0, 8.0, 16.0]
        .iter()
        .map(|&kv| {
            let n = results
                .iter()
                .filter(|(_, g)| (g.kv - kv).abs() < 1e-3)
                .count();
            format!("kv {kv}: {n}")
        })
        .collect();
    println!("survivors per kv: {}", kv_counts.join(", "));
}
