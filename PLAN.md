# Plan

What is left to do, in order: the open bring-up steps on the prototype, then the step from a prototype held together with rubber bands to a finished robot that can be built a few times and given away. Completed steps, with what was measured and learned on the way, are in [HISTORY.md](HISTORY.md).

## How this file works

- Small steps, one PR each, in order within a group. Groups can run in parallel where noted.
- When a step is done: tick it here in the same PR, then move it with its findings to the matching group in `HISTORY.md` (the next PR may do the move).
- Split a step that turns out too big; insert new steps where they belong.
- Each step ends with a concrete check on the hardware, in simulation or in the browser.

## Open steps on the prototype

### D. After it balances

- [ ] Simulation: gear backlash in the motor model, if the log comparison shows it matters (limit cycle around upright). Not needed so far: the robot wobbles at ~4 Hz (sim ~1.5 Hz) but only ±1°
- [ ] Battery under load: check sag and brownouts while balancing; add bulk capacitance if needed (see "Power notes")

### E. micro:bit remote over BLE (in parallel with C/D; only E12 needs a balancing robot)

- [ ] micro:bit as central: scan for the robot's service UUID, connect, write a fixed packet once a second; LEDs show scanning / connected. Check: the robot logs the packets; the remote reconnects after a robot reset
- [ ] Live tilt commands at 20-50 Hz, `active` from the grip; the robot logs `active`, speed and turn (not applied yet) and packets per second. Check: commands are 0 with `active` 0 unless A+B are held; pressed at an odd angle they start at 0; rate and gaps in the log; range (walk away until it drops)
- [ ] Steering, wheels in the air: differential turn in `balance_core::drive` (`left = cmd + turn`, `right = cmd − turn`, clamped), unit tested; the robot applies `turn`. Check: hold A+B and tilt the micro:bit left, the wheels spin opposite ways; let go and the turn part goes to 0 (with the robot held level, the wheels stop)
- [ ] Speed setpoint, simulation first: target speed in `Controller` (P/I on `pitch + kv·(speed_est − target)`, rate-limited); sim example stepping the target 0 → 0.2 → 0. Check in sim: stays up and follows the target
- [ ] Drive it (needs D2 done, i.e. the robot balances): apply the speed target on the robot. Check: tilt forward and it rolls forward, level the remote or release A+B and it stops; on link loss it stops but stays up

## Product: from prototype to giveaway robot

Goal: a robot that can be played with without fear of breaking it, with a rechargeable battery, no loose wires, and electronics that a service builds, so a few copies can be made and given away. Ease of having it made matters more than price.

### Decisions (2026-11-22)

| Topic | Decision | Why |
|---|---|---|
| MCU | Keep the **ESP32-C3** (ESP32-C3-MINI-1 module) | The micro:bit remote needs BLE, and the C3 is already about the cheapest chip that has it. It also has USB built in. The cheaper ESP32-C2 needs an extra USB-serial chip, which eats the saving, and would mean porting the firmware |
| IMU | Keep the **ICM-42670-P** | Same chip as the dev board: `balance_core::imu` and the burst read carry over |
| Motors | Keep the **TT gear motors** (several on hand) | The 37 mm encoder motors are not part of this plan |
| Motor driver | **DRV8833** instead of the L298N | Tiny, ~0.3 V drop instead of ~2 V, inputs pulled low (no twitch at reset), current limit |
| Battery | **2S 18650** (two cells), USB-C charging | One cell would run the TT motors at ~60 % of today's speed, or need a 2-3 A boost converter |
| Board maker | **JLCPCB**, assembled on one side | Small SMD parts placed by them; only through-hole parts soldered by hand |
| Sound | **Speaker** (PAM8302 amplifier fed by PWM) | Sounds instead of beeps for ~$1-2 per board, and only one pin |
| Printer | Prusa MK3; **PLA** for the first prints, **PETG** from the drop test on | PLA is what the printer's owner knows; PETG survives falls. TPU for bumpers and tires |
| Remote | **micro:bit** and a **phone web app** (group I) | The phone is free for anyone who gets a robot; the micro:bit stays for those who have one |

### Robot board (group F)

One two-layer board of about 80 x 60 mm, all SMD parts on the front so JLCPCB's economic assembly can place them, 18650 holders on the back. One USB-C port does charging, flashing and logging, like the dev board.

Power path: USB-C 5 V → IP2326 boost charger → 2S pack (with S-8252 protection) → slide switch + P-MOSFET (+ fuse) → DRV8833 and the motors. A buck converter makes 3.3 V from the switched battery **or** from USB (diode-OR), so a robot with a flat battery can still be flashed while it charges.

| Block | Part | Placed by | Why |
|---|---|---|---|
| MCU | ESP32-C3-MINI-1 | JLCPCB | Same chip as now, certified antenna. Board edge, no copper or battery under the antenna |
| IMU | ICM-42670-P | JLCPCB | Same chip as now |
| Motor driver | DRV8833 (HTSSOP with thermal pad) | JLCPCB | 2.7-10.8 V, ~1.5 A per motor, current-limit resistors, inputs pulled low |
| Charger | [IP2326](https://www.lcsc.com/product-detail/C2832094.html) 5 V to 2S boost charger | JLCPCB | Charges the 8.4 V pack from plain USB-C 5 V |
| Pack protection | ABLIC [S-8252](https://www.lcsc.com/product-detail/C468215.html) 2-cell protection IC + dual N-MOSFET | JLCPCB | Over-charge, over-discharge, short circuit |
| 3.3 V | Small buck converter with 8.4 V+ input (e.g. AP63203) | JLCPCB | An LDO from 8.4 V would waste ~0.5 W |
| Power path | Two Schottky diodes (battery, USB) into the buck | JLCPCB | Flashing works with the switch off or the battery flat |
| Power switch | Small slide switch driving a P-MOSFET | Hand (switch), JLCPCB (FET) | The switch carries no motor current; the FET also blocks a reversed battery |
| Fuse | 3 A PTC or SMD fuse | JLCPCB | A pinched motor wire won't burn the pack |
| USB-C | 16-pin receptacle with through-hole shell legs, 5.1k CC resistors, USBLC6 ESD diode | JLCPCB | Strong, works with any USB-C charger |
| Bulk capacitor | 470 µF 16 V electrolytic | Hand | Motor current spikes must not reset the MCU |
| Battery holders | 2x PCB-mount 18650 holders | Hand | Swappable cells, no battery wires |
| Motor connectors | 2x JST-XH 2-pin | Hand | Latching plugs |
| Speaker | PAM8302 mono amplifier + 15-20 mm speaker on a 2-pin plug | JLCPCB (amp), hand (plug) | PWM audio from GPIO7 through an RC filter |
| User I/O | WS2812 RGB LED, BOOT and RESET buttons, charge LEDs | JLCPCB | Status without a PC |
| Test points | 3V3, VBAT, GND, I2C, motor outputs | - | Checking a new board with a multimeter |

On each TT motor: a 100 nF ceramic capacitor across the terminals, against brush noise.

Pin map (keeps the dev board's I2C, LED and battery pins; motor pins move off GPIO20/21 because the ROM bootloader toggles GPIO21 at reset):

| Function | GPIO | Note |
|---|---|---|
| Left motor IN1 / IN2 (PWM) | 0 / 1 | DRV8833 takes PWM on the IN pins, no EN pins |
| Right motor IN3 / IN4 (PWM) | 3 / 6 | 4 of the 6 LEDC channels |
| DRV8833 nSLEEP | 5 | 10k pull-down: motors dead until the firmware wakes the driver |
| DRV8833 nFAULT | 20 | Input with pull-up: over-current or over-temperature |
| Battery sense | 4 | Divider 100k / 33k, as now |
| I2C SDA / SCL | 10 / 8 | As now |
| WS2812 RGB LED | 2 | As now |
| Speaker (PWM into the amplifier) | 7 | Replaces the plain LED |
| BOOT button | 9 | As now |
| USB D- / D+ | 18 / 19 | Native USB-Serial-JTAG, as now |
| IMU INT1 | 21 | Through 1k, since the bootloader drives GPIO21 at reset |

Having it made: KiCad with the JLCPCB fabrication plugin (Gerbers, BOM, placement). Prefer JLCPCB "basic" parts; each "extended" part type adds ~$3 setup. Roughly $80-120 for 5 assembled boards including parts and shipping to Denmark (estimate, VAT collected at checkout); fabrication 3-5 days, shipping 3-7 days. Plan for two revisions.

- [ ] F1. Fix the board outline, mounting holes, USB-C and switch positions in one shared file (the contract between board and chassis)
- [ ] F2. Schematic in KiCad; review against the datasheets (IP2326, S-8252, DRV8833, ESP32-C3-MINI-1 reference design)
- [ ] F3. Layout: antenna keep-out, motor current paths short and away from the IMU, thermal pad vias under the DRV8833; JLCPCB DRC and part availability check
- [ ] F4. Order Rev A (5 boards). Check: boards arrive, nothing visibly misplaced
- [ ] F5. Bring-up Rev A without motors or cells: USB only, 3.3 V, flash `cargo imu-log`, IMU and battery-sense readings. Then cells: charge LED, charge current, protection cut-off
- [ ] F6. Motors on the new board (needs H2). Check: `cargo motor-check` with the wheels in the air
- [ ] F7. Order Rev B with the fixes found in F5/F6

### Chassis (group G)

One print a week, so every print must answer as many questions as possible: the first print is a plate of test pieces, and the full body is printed only once its fits are proven.

- CAD in **OpenSCAD** in the repo (`chassis/`): plain text, reviewable in a PR, every dimension a named parameter.
- Keep today's shape so the tuning stays close: motors at the bottom on the axle line, the board upright above them (IMU +X up), the cells on the back of the board, high up (centre of mass at least the current 8 cm above the axle).
- Screws, not glue: M3 heat-set inserts, M3 screws through the board and the TT motors' two mounting holes.
- 4 walls, 30-40 % infill, oriented so crash loads run along the layers. PETG on the MK3: stock PrusaSlicer profile, glue stick or the textured sheet (PETG can stick too hard to smooth PEI).
- For the giveaway robots, consider ordering the final body from JLC3DP (MJF nylon PA12) together with a board order.

- [x] G1a. Fit-test plate design (PLA) in `chassis/fit_test.scad`: TT motor cradle, wheel hubs on the motor shaft, heat-set insert holes in three sizes, the board hole pattern, USB-C and switch cut-outs, a cell seat, a tolerance comb (0.1-0.5 mm clearances). Check: renders to clean solids in CI, slices for the MK3 without supports
- [ ] G1b. Measure a TT motor and the cells (the `MEASURE` values in `chassis/params.scad`), correct them, then print the plate and test each piece as in `chassis/README.md`. Check: the numbers are written down and copied into `params.scad` (insert hole, hub clearance, `fit_slide`, USB-C opening)
- [ ] G2. Body v1 (PLA), with the real motors and the real or a dummy board. Check: goes together without force, cells can be swapped, the USB-C plug reaches
- [ ] G3. Body v2 (PETG) + bumpers and tires (TPU). Check: drop it from upright onto a hard floor 20 times; note what breaks first
- [ ] G4. Body v3 with the drop-test fixes; the Rev A board goes in. Check: it balances in the new body
- [ ] G5. Final body; then one robot per print

### Built for play

| What breaks | Fix |
|---|---|
| Robot lands on the board or battery when it falls | Bumpers at the top front and back that touch the floor first (TPU, or a replaceable PETG part); the board sits inside the body |
| TT gearbox cracks or the motor rips out of its mount | Printed cradle that holds the whole motor body, two M3 screws |
| Wheel pops off the D-shaft | Screw into the shaft's centre hole, through the hub |
| Wires break at solder joints | Short motor leads with JST-XH plugs, a printed zip-tie anchor at each motor, hot glue on the motor tabs |
| Cells shake out of the holders | Printed lid or strap over the cells, screwed |
| USB-C port tears off the board | Through-hole shell legs; the chassis wall supports the plug |
| Stalled wheels overheat the motors | DRV8833 current limit (~1.5 A) and the firmware cut-offs |
| Battery run flat or shorted | Protection IC, firmware low-battery cut-off at 6.4 V, fuse, warning sound from 6.6 V |
| Robot runs away when the remote disconnects | Link loss sets the speed target to 0; the robot keeps balancing (group E) |
| Motors twitch at power-up | DRV8833 pull-downs plus nSLEEP held low until the firmware is ready |

For gifts, add a short card: how to charge, switch off after play, only charge over USB-C.

### Firmware on the robot board (group H)

After group E works on the prototype.

- [ ] H1. Board selection: a `Board` setting (dev board vs robot board) for the pin map, so the prototype keeps working
- [ ] H2. DRV8833 motor driver: PWM on IN1/IN2 instead of EN + direction; nSLEEP high only once the safety gate is armed; log nFAULT as `# safety: driver fault`
- [ ] H3. Re-measure the deadband and top speed. Without the L298N's ~2 V drop the motors get up to 8.4 V, so cap the duty to about 75 % (≈ 6 V average) to protect the 6 V-rated TT motors, then update `Drive` and `Motor::prototype()` in the sim
- [ ] H4. Re-check the IMU axes and the balance point in the new body
- [ ] H5. Speaker and RGB LED: sounds and colours for power-on, armed, fallen, low battery, charging, remote connected (PWM audio from short clips in flash)
- [ ] H6. Pairing for several robots: each remote bonds to one robot (e.g. hold A+B at power-up to pair to the nearest robot by signal strength, store its address in the micro:bit's flash; the phone app picks the robot in the browser's device chooser)

### Phone remote and browser simulator (group I)

A static web app on **GitHub Pages** turns any phone into a tilt remote, and its Simulator mode runs the robot in the browser, so anyone can play without a robot. `sim` and `balance-core` only depend on `libm`, so they compile to WebAssembly as they are.

- One Rust crate, `web/`, built with `wasm-bindgen` (via `trunk` or `wasm-pack`) into a static page. Rust does the work: `sim` for Simulator mode, `balance_core::remote` for tilt → command, `balance_core::link` for the packet. JavaScript only connects the browser APIs.
- Same remote logic as the micro:bit: a big on-screen button replaces A+B (hold to drive; the phone's attitude at that moment is the zero point). Tilt from the DeviceOrientation API.
- Two modes, same controls: Simulator mode drives the simulated robot, drawn from the side on a canvas with pitch, speed and battery. Robot mode sends the same `link` packet over Web Bluetooth at 20-50 Hz to the robot's GATT characteristic.
- Deploy: a GitHub Actions job builds the page on every push to main and publishes it with `actions/deploy-pages`; PRs build it in CI.
- Browser support: the simulator runs everywhere. Web Bluetooth works in Chrome on Android, Windows, macOS and ChromeOS, not in Safari or Firefox; on iPhone use the Bluefy browser. DeviceOrientation on iPhone needs a tap to grant permission; both APIs need HTTPS, which Pages provides.

Steps I1-I4 need no hardware and can start right away.

- [ ] I1. `web/` crate skeleton and the Pages deploy job. Check: the page shows "hello" at the Pages URL
- [ ] I2. Simulator in the browser: run `sim::closed_loop` at real time in wasm and draw the robot. Check: it stands up from a 5° start, as in `cargo run -p sim --example balance`
- [ ] I3. Push the robot: a tap or drag on the canvas gives it a shove; a reset button stands it up. Check: it catches small pushes and falls on big ones
- [ ] I4. Phone tilt into the simulator, with the grip and tilt math from group E. Check: hold the button and tilt, the simulated robot drives; let go and it stops
- [ ] I5. Web Bluetooth to the real robot (needs the robot's GATT step in group E). Check: the robot logs `# remote:` lines from the phone; closing the tab trips the link watchdog
- [ ] I6. Optional: telemetry back from the robot (pitch, battery) over a notify characteristic, shown in the same view as the simulator

### Timeline

One print per week; the board is designed in parallel. Weeks counted from the start.

| Week | Chassis | Robot board | Firmware | Phone web app |
|---|---|---|---|---|
| 1 | G1 fit-test plate | F1-F2 | Group E on the prototype | I1 Pages deploy |
| 2 | G2 body v1 | F3, order Rev A | Group E | I2 simulator |
| 3 | G3 body v2 + bumpers | Rev A at JLCPCB | Group E | I3 push |
| 4 | G4 body v3 | Rev A shipping | Group E | I4 phone tilt |
| 5 | (gate: Rev A in body v3, balances) | F5-F6 bring-up | H1-H4 | I5 Web Bluetooth |
| 6 | G5 final body, then one per week | F7 order Rev B | H5-H6 | I6 |

### Still open

- How many robots: 3 or 10 changes whether JLC3DP nylon bodies are worth it.
- Prices above are estimates from memory; check them in JLCPCB's parts library before ordering.
