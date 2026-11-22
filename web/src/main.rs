//! Phone remote and browser simulator for the balance robot (group I in `PLAN.md`).
//!
//! Built with `trunk` into a static page (`index.html` + JS glue + `.wasm`), which GitHub
//! Pages serves. Step I1 only proves the pipeline: Rust runs in the browser and writes
//! into the page.

use wasm_bindgen::JsValue;

/// Runs once when the page has loaded the `.wasm` file: wasm-bindgen calls a binary's
/// `main` as the module's start function.
///
/// # Errors
///
/// Returns the browser's error if the page has no `#app` element. wasm-bindgen turns an
/// `Err` from `main` into a JavaScript exception, so it shows up in the browser console.
fn main() -> Result<(), JsValue> {
    let document = web_sys::window()
        .and_then(|window| window.document())
        .ok_or_else(|| JsValue::from_str("no document"))?;
    let app = document
        .get_element_by_id("app")
        .ok_or_else(|| JsValue::from_str("no #app element"))?;
    app.set_text_content(Some("hello"));
    Ok(())
}
