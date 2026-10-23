# TW3_V2 — micromouse firmware

Teensy 4.0 main MCU, Teensy 4.0 sensor sub-processor, ESP32-C3 radio pair.
IEEE 16×16 maze, 180 mm cells.

**This is a fresh cut of TW3_V1's known-good code.** Drivers, control loops,
sensor calibration, flood fill and the search mission came across; the fast run
and the localization stack that grew around it did not. Read
[`docs/V2_BASELINE.md`](docs/V2_BASELINE.md) first — it says exactly what was
kept, what was dropped, what was learned from the dropped work, and the rules
that keep this tree from going the same way.

## Layout

```
docs/
  V2_BASELINE.md              what this baseline is, and why    <- start here
  adr/ADR-002-fast-run-geometry.md   the fast run: constant-radius arcs,
                              gated on wall edges, and when to go clothoid
  LOOP_RIG.md                 the autonomous corner experiment: build the
                              ring, run it, read it
  micromouse_spec.md          hardware: pins, geometry, buses
  firmware_architecture_adr.md  ADR-001: C++/Arduino on PlatformIO, layered
  COMMANDS.md                 serial/radio console reference
firmware/
  main_mcu/                   Teensy 4.0 -- the mouse
    lib/HAL                   pins, buses, timers
    lib/Drivers               IMU, encoders, motors, FRAM, sub-MCU link
    lib/Control               500 Hz ISR: fusion, odometry, wheel loops,
                              maneuvers, wall interpretation, fast-run
                              trajectory (Path) and wall gating (WallGate)
    lib/Maze                  wall map + flood fill (pure logic, host-tested)
    lib/Application           search executor, loop sweep, FRAM
                              params/LUT/logs, console, bench harnesses
    src/main.cpp              boot + loop, and nothing else
    test/                     18 host-side tests; ./run_tests.sh
  sensor_subprocessor/        Teensy 4.0 -- 4 S15366 laser sensors
  esp32_radio/                mouse-side ESP-NOW bridge
  esp32_laptop/               laptop-side ESP-NOW dongle
  lib_shared/EspNowLink/      shared by both radios
tools/
  TW3_sensor_cal.py           sensor calibration GUI
  flight_viewer.html          log viewer -- flight recorder ('y') and sweep ('sweep.dump').
                              Map + trail + beams on top; underneath, THE CHANNEL CHART
                              (2026-09-25): any logged column or derived channel (dv/dt,
                              ds/dt, v_imu, slip %, the ABS / brake / arc flags), each on
                              its own scale, value at the cursor, x = record / t_ms / s,
                              ghost runs faint. The selection is remembered per log kind.
                              tune_gui's "View" button opens it on the log just fetched.
```

## Quick start

```
cd firmware/main_mcu
./test/run_tests.sh        # 18 passed, 0 failed -- no hardware needed
pio run                    # build for teensy40
pio run -t upload          # flash
pio device monitor         # 115200; 'p' prints params, 'e' runs a search
```

## Status

| | |
|---|---|
| Host tests | 18 / 18 pass |
| `pio run` | clean, no warnings |
| Flash | 125 KB code, 52 KB data (of 1.9 MB) |
| Hardware | **not yet flown since the cut** — see `docs/V2_BASELINE.md`, "First things to do" |
