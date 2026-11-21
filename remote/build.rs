//! Puts `memory.x` where the linker finds it (cortex-m-rt's `link.x` includes it) and
//! relinks when it changes.

use std::{env, fs, path::PathBuf};

fn main() -> std::io::Result<()> {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap_or_default());
    fs::copy("memory.x", out.join("memory.x"))?;
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    Ok(())
}
