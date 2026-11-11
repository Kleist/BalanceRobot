//! Prints how the prototype falls from 1° with the axle held still, next to
//! the small-angle prediction `θ₀·cosh(t/τ)`.
//!
//! `cargo run -p sim --example fall`

use sim::pendulum::{Body, State};

fn main() {
    const DT: f64 = 1e-4;
    const PRINT_EVERY: u32 = 200; // 20 ms

    let body = Body::prototype();
    let tau = body.fall_time_constant_s();
    println!("# I = {:.5} kg·m², tau = {:.3} s", body.inertia_kg_m2, tau);
    println!("t_s,pitch_deg,cosh_deg");

    let start_deg = 1.0;
    let mut state = State::tilted_deg(start_deg);
    let mut steps = 0_u32;
    while state.pitch_deg() < 90.0 {
        if steps.is_multiple_of(PRINT_EVERY) {
            let t = f64::from(steps) * DT;
            let cosh_deg = start_deg * (t / tau).cosh();
            println!("{t:.2},{:.2},{cosh_deg:.2}", state.pitch_deg());
        }
        state = state.step(&body, 0.0, DT);
        steps += 1;
    }
    println!("# lying down (90°) after {:.3} s", f64::from(steps) * DT);
}
