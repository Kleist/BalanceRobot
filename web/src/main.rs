//! Phone remote and browser simulator for the balance robot (group I in `PLAN.md`).
//!
//! Built with `trunk` into a static page (`index.html` + JS glue + `.wasm`), which GitHub
//! Pages serves. Rust does the work: [`App`] runs the same simulation as
//! `cargo run -p sim --example balance` (physics, IMU and motor models, and the firmware's
//! own controller from `balance-core`) at real time and draws it on a canvas. The page's
//! JavaScript only calls [`App::frame`] once per screen refresh.

mod clock;
mod view;

use balance_core::{control::Config, drive::Drive, gyro::CalibrationError};
use sim::{
    closed_loop::{FALLEN_DEG, Sim, Tick},
    log::DT_S,
    motor::Motor,
    pendulum::Body,
};
use wasm_bindgen::{JsCast, JsValue, prelude::wasm_bindgen};
use web_sys::{CanvasRenderingContext2d, HtmlCanvasElement};

use crate::{clock::Clock, view::Robot};

/// Pitch the simulated robot is let go at, in degrees (as in step I2's check).
const START_PITCH_DEG: f64 = 5.0;

/// The simulator page: one simulated robot and the canvas it is drawn on.
///
/// `#[wasm_bindgen]` exports the type to JavaScript as a class: `new App("robot")` calls
/// [`App::new`], `app.frame(t)` calls [`App::frame`]. The Rust value lives in the wasm
/// memory; JavaScript only holds a handle to it.
#[wasm_bindgen]
pub struct App {
    sim: Sim,
    clock: Clock,
    /// The latest controller tick: what is drawn.
    last: Option<Tick>,
    fell_at_s: Option<f64>,
    canvas: HtmlCanvasElement,
    ctx: CanvasRenderingContext2d,
    robot: Robot,
}

#[wasm_bindgen]
impl App {
    /// Sets up the simulation and finds the canvas with the id `canvas_id`.
    ///
    /// # Errors
    ///
    /// If there is no such canvas, or the simulated gyro calibration fails (it can't with
    /// the prototype's IMU noise). JavaScript sees the error as an exception.
    #[wasm_bindgen(constructor)]
    pub fn new(canvas_id: &str) -> Result<App, JsValue> {
        let canvas: HtmlCanvasElement = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id(canvas_id))
            .ok_or_else(|| JsValue::from_str("canvas not found"))?
            .dyn_into()?;
        let ctx: CanvasRenderingContext2d = canvas
            .get_context("2d")?
            .ok_or_else(|| JsValue::from_str("no 2d context"))?
            .dyn_into()?;
        let sim = robot_sim()
            .map_err(|e| JsValue::from_str(&format!("gyro calibration failed: {e:?}")))?;
        let body = Body::prototype();
        Ok(App {
            sim,
            clock: Clock::new(f64::from(DT_S) * 1000.0),
            last: None,
            fell_at_s: None,
            canvas,
            ctx,
            robot: Robot {
                wheel_radius_m: body.wheel_radius_m,
                com_height_m: body.com_height_m,
                battery_v: Motor::prototype().battery_v,
            },
        })
    }

    /// Advances the simulation to `now_ms` (the timestamp `requestAnimationFrame` passes)
    /// and draws it.
    pub fn frame(&mut self, now_ms: f64) {
        for _ in 0..self.clock.ticks_due(now_ms) {
            if self.fell_at_s.is_some() {
                break;
            }
            let tick = self.sim.tick();
            if tick.pitch_deg.abs() > FALLEN_DEG {
                self.fell_at_s = Some(tick.t_s);
            }
            self.last = Some(tick);
        }
        if let Some(tick) = &self.last {
            view::draw(&self.canvas, &self.ctx, &self.robot, tick, self.fell_at_s);
        }
    }
}

/// The prototype robot as on the floor: deadband motor, the firmware's gains, and the
/// firmware's deadband compensation and battery scaling between them, let go at
/// [`START_PITCH_DEG`].
fn robot_sim() -> Result<Sim, CalibrationError> {
    Ok(Sim::prototype(Config::PROTOTYPE.gains, START_PITCH_DEG)?.with_drive(Drive::PROTOTYPE))
}

/// wasm-bindgen runs a binary's `main` as soon as the module has loaded. All the work
/// happens in [`App`], which the page creates.
fn main() {}

#[cfg(test)]
mod tests {
    use super::*;

    /// Step I2's check, on the host: the robot the page shows stands up from the start
    /// tilt and stays up.
    #[test]
    fn the_page_robot_stands_up_from_the_start_tilt() {
        let run = robot_sim().unwrap().run(10.0);
        assert_eq!(run.fell_at_s, None);
        let wobble = run.max_abs_pitch_deg(5.0);
        assert!(wobble < 3.0, "wobbles ±{wobble:.1}° in the last 5 s");
    }
}
