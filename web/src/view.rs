//! Drawing the simulated robot from the side on a 2D canvas.
//!
//! The camera follows the robot, so the robot stays in the middle and the ground (with a
//! mark every 5 cm) moves under it: that shows it rolling. Forward (positive pitch and
//! speed) is to the right.

use std::f64::consts::TAU;

use sim::closed_loop::Tick;
use web_sys::{CanvasRenderingContext2d, HtmlCanvasElement};

/// Height of the robot's body above the axle, in m (drawing only; the physics uses the
/// measured centre of mass and inertia).
const BODY_HEIGHT_M: f64 = 0.18;
/// Width of the drawn body, in m.
const BODY_WIDTH_M: f64 = 0.05;
/// How much of the world fits on the canvas, top to bottom, in m.
const VIEW_HEIGHT_M: f64 = 0.4;
/// Spacing of the marks on the ground, in m.
const GROUND_MARK_M: f64 = 0.05;

/// What doesn't change between frames.
#[derive(Debug, Clone)]
pub struct Robot {
    /// Wheel radius in m.
    pub wheel_radius_m: f64,
    /// Centre of mass above the axle in m (drawn as a dot).
    pub com_height_m: f64,
    /// The simulated battery voltage.
    pub battery_v: f64,
}

/// Draws `tick` filling the canvas. `fell_at_s` is when the robot fell, if it did.
pub fn draw(
    canvas: &HtmlCanvasElement,
    ctx: &CanvasRenderingContext2d,
    robot: &Robot,
    tick: &Tick,
    fell_at_s: Option<f64>,
) {
    let (width, height) = fit_to_screen(canvas, ctx);
    let ink = text_colour(canvas);
    ctx.clear_rect(0.0, 0.0, width, height);

    let px_per_m = height / VIEW_HEIGHT_M;
    let ground_y = height * 0.85;
    let axle = (width / 2.0, ground_y - robot.wheel_radius_m * px_per_m);

    // Ground and its marks. The robot is at the centre, so a mark at world position x is
    // drawn (x − axle position) to the right of it.
    ctx.set_stroke_style_str(&ink);
    ctx.set_line_width(2.0);
    line(ctx, (0.0, ground_y), (width, ground_y));
    ctx.set_line_width(1.0);
    let half_width_m = width / 2.0 / px_per_m;
    let first = ((tick.axle_pos_m - half_width_m) / GROUND_MARK_M).floor();
    let last = ((tick.axle_pos_m + half_width_m) / GROUND_MARK_M).ceil();
    let mut mark = first;
    while mark <= last {
        let x = width / 2.0 + (mark * GROUND_MARK_M - tick.axle_pos_m) * px_per_m;
        line(ctx, (x, ground_y), (x - 8.0, ground_y + 10.0));
        mark += 1.0;
    }

    // Body: rotated about the axle by the pitch. The canvas's y axis points down, so a
    // positive rotation is clockwise on screen: the top leans right, i.e. forward.
    ctx.save();
    let _ = ctx.translate(axle.0, axle.1);
    let _ = ctx.rotate(tick.pitch_deg.to_radians());
    ctx.set_fill_style_str(if fell_at_s.is_some() {
        "#d0553f"
    } else {
        "#3f7fd0"
    });
    ctx.fill_rect(
        -BODY_WIDTH_M / 2.0 * px_per_m,
        -BODY_HEIGHT_M * px_per_m,
        BODY_WIDTH_M * px_per_m,
        BODY_HEIGHT_M * px_per_m,
    );
    ctx.set_fill_style_str(&ink);
    ctx.begin_path();
    let _ = ctx.arc(0.0, -robot.com_height_m * px_per_m, 4.0, 0.0, TAU);
    ctx.fill();
    ctx.restore();

    // Wheel, with a spoke that turns as it rolls: angle = distance / radius.
    let radius = robot.wheel_radius_m * px_per_m;
    ctx.set_stroke_style_str(&ink);
    ctx.set_line_width(3.0);
    ctx.begin_path();
    let _ = ctx.arc(axle.0, axle.1, radius, 0.0, TAU);
    ctx.stroke();
    let angle = tick.axle_pos_m / robot.wheel_radius_m;
    line(
        ctx,
        axle,
        (axle.0 + radius * angle.cos(), axle.1 + radius * angle.sin()),
    );

    // Readout.
    ctx.set_fill_style_str(&ink);
    ctx.set_font("16px system-ui, sans-serif");
    let lines = [
        format!("t {:.1} s", tick.t_s),
        format!("pitch {:+.1}°", tick.pitch_deg),
        format!("speed {:+.1} cm/s", tick.axle_vel_m_s * 100.0),
        format!("position {:+.1} cm", tick.axle_pos_m * 100.0),
        format!("motor {:+.2}", tick.motor_cmd),
        format!("battery {:.1} V", robot.battery_v),
        fell_at_s.map_or_else(String::new, |t| format!("fell after {t:.2} s")),
    ];
    for (i, text) in lines.iter().enumerate() {
        #[allow(clippy::cast_precision_loss, reason = "a handful of lines")]
        let y = 24.0 + 20.0 * i as f64;
        let _ = ctx.fill_text(text, 12.0, y);
    }
}

/// Makes the canvas's pixel buffer match its size on screen (times the screen's pixel
/// density, so it is sharp on a phone), and scales the drawing so that one unit is one CSS
/// pixel. Returns the size in CSS pixels.
fn fit_to_screen(canvas: &HtmlCanvasElement, ctx: &CanvasRenderingContext2d) -> (f64, f64) {
    let dpr = web_sys::window().map_or(1.0, |w| w.device_pixel_ratio());
    let w = f64::from(canvas.client_width());
    let h = f64::from(canvas.client_height());
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "screen sizes in pixels are small and positive"
    )]
    let (pw, ph) = ((w * dpr).round() as u32, (h * dpr).round() as u32);
    if canvas.width() != pw || canvas.height() != ph {
        // Resizing clears the canvas and resets its transform.
        canvas.set_width(pw);
        canvas.set_height(ph);
    }
    let _ = ctx.set_transform(dpr, 0.0, 0.0, dpr, 0.0, 0.0);
    (w, h)
}

/// The page's text colour (light or dark theme), so the drawing matches it.
fn text_colour(canvas: &HtmlCanvasElement) -> String {
    web_sys::window()
        .and_then(|w| w.get_computed_style(canvas).ok().flatten())
        .and_then(|s| s.get_property_value("color").ok())
        .unwrap_or_else(|| "#888".into())
}

fn line(ctx: &CanvasRenderingContext2d, from: (f64, f64), to: (f64, f64)) {
    ctx.begin_path();
    ctx.move_to(from.0, from.1);
    ctx.line_to(to.0, to.1);
    ctx.stroke();
}
