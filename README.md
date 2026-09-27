# BalanceRobot

Two-wheel self-balancing robot. `no_std` Rust firmware (esp-hal + Embassy) for the ESP32-C3-DevKit-RUST-1, later steered from Android over BLE.

See [CLAUDE.md](CLAUDE.md) for hardware, pinout, power notes and the bring-up plan.

## Build & flash

```sh
cargo install espflash --locked   # once
cargo run --release               # build, flash over USB and open the log monitor
```
