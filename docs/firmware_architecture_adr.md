# ADR-001: Firmware Language & Software Architecture for Micromouse MCUs

**Status:** Accepted
**Date:** 2026-07-01 (amended 2026-07-02: reverted build tooling from Arduino IDE back to PlatformIO)
**Deciders:** bert

## Context

`micromouse_spec.md` fixes the on-robot hardware: two Teensy 4.0 boards (Main MCU; Sensor sub-processor) and one ESP32 Super Mini (wireless radio, UART-connected to the Main MCU). The task now is to write the embedded software that programs these three microcontrollers so the mouse can autonomously explore and solve the maze within the competition's 5-minute time limit.

Constraints in play:
- Main MCU must run tight, deterministic control loops (motor PWM, quadrature encoder capture, IMU/wall-sensor fusion) alongside less time-critical work (maze solving, FRAM logging, telemetry).
- Sensor sub-processor must strobe 4 lasers with ~500 µs timing precision and stream 3 wall-distance channels over UART.
- ESP32 radio is a simple UART→ESP-NOW relay with no control logic.
- Earlier project artifacts (now stripped from the hardware spec) referenced a C++/Arduino IDE codebase, indicating the team's existing driver work and familiarity already assume Arduino.
- Single-developer project — toolchain consistency and library availability matter more than large-team process concerns.

## Decision

Use **C++ on the Arduino framework** for all three MCUs — Teensyduino for the two Teensy 4.0 boards, Arduino-ESP32 core for the radio — built with **PlatformIO**, each microcontroller as its own PlatformIO project. Structure each board's firmware in layers (HAL → Drivers → Control → Application) and use a fixed-rate hardware-timer ISR for the Main MCU's control loop instead of a full RTOS.

PlatformIO doesn't change the language or framework, only the build tooling, and it fits a three-board project better: each board's project pins its own platform/library versions in `platformio.ini`, so a library update for one board can't silently break another (Arduino IDE shares one global library folder across all sketches). Its `lib/` convention maps each architecture layer to its own auto-linked library, matching the layering below directly. It's also CLI-first and plain-text-config (scriptable, version-control-friendly, CI-ready) and has a built-in unit test runner for host-side tests of hardware-independent logic like maze solving. The trade-off is a slightly steeper on-ramp than Arduino IDE's open-and-upload workflow; this was accepted (revisited 2026-07-02 after briefly switching to Arduino IDE).

## Consequences

- Faster iteration: existing Arduino libraries cover IMU, FRAM, and motor-driver needs immediately, installable per-project via `platformio.ini`.
- Per-board isolation: each MCU's project pins its own platform and library versions; no shared global library folder.
- Must be deliberate about which code runs in the fixed-rate ISR (control, encoder capture) vs. the main loop (maze solving, logging, telemetry) to avoid jitter — this is a discipline requirement, not a framework guarantee.
- Host-side unit tests (e.g. maze-solving logic) can use PlatformIO's built-in test runner (`pio test`) when wanted.
- Shared code between boards (e.g. common UART packet framing) can live in a shared library folder referenced by each project's `lib_extra_dirs`. Decide the exact location when packet framing is defined (see action items).
- Slightly steeper on-ramp than Arduino IDE (build/upload via PlatformIO IDE extension or `pio run -t upload` rather than a single Upload button).
- UART packet framing between sub-processor↔Main MCU and Main MCU↔ESP32 (removed from the hardware spec along with the old firmware section) will need to be redefined in a firmware design doc before implementation starts.

## Suggested Layering (per board)

Each layer is its own folder under the project's `lib/` (e.g. `lib/HAL/HAL.h` + `HAL.cpp`), auto-linked by PlatformIO's library dependency finder; the entry point is `src/main.cpp`.

**Main MCU:** HAL (SPI, UART, PWM, GPIO, timers/DMA) → Drivers (IMU, FRAM, motor driver, encoder capture) → Control (fixed-rate ISR: wheel velocity control, heading correction, sensor fusion) → Application (maze solve/explore, run state, logging/telemetry — main loop, decoupled from the ISR via lock-free buffers or volatile shared state).

**Sensor sub-processor:** HAL (SPI to sensors, laser strobe timing via IntervalTimer) → Protocol (packetize 3 wall-distance channels, send over UART).

**ESP32 radio:** HAL (UART receive) → Transport (ESP-NOW packetization/send). No control logic.

## Action Items

1. [ ] Set up a PlatformIO project per board (`main_mcu/`, `sensor_subprocessor/`, `esp32_radio/`), each with its `platformio.ini`, `src/main.cpp`, and layer libraries under `lib/` (`HAL`, `Drivers`, `Control`/`Protocol`/`Transport`, `Application`).
2. [ ] Prototype the fixed-rate control ISR and encoder DMA capture first — highest-risk, most timing-critical piece.
3. [ ] Build and bench-test the HAL layer per board before layering drivers on top.
4. [ ] Define UART packet framing (sub-processor↔Main MCU, Main MCU↔ESP32) in a firmware design doc, including whether it's duplicated per sketch or shared via an installed library.
