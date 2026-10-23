# Firmware

Per `../docs/firmware_architecture_adr.md`, each on-robot microcontroller gets
its own PlatformIO project. Each layer (HAL, Drivers, Control,
Application/Protocol/Transport) is its own library under the project's `lib/`,
auto-linked by PlatformIO; the entry point is `src/main.cpp`.

- `main_mcu/` — Teensy 4.0, Main MCU. Maze solving, motion control, sensor
  fusion, logging. **Search mission works end to end** (V1 hardware-proven
  through 2026-08-01): boot-align leg, continuous cell-to-cell straights with
  in-flight extend-or-brake, in-place absolute-cardinal turns, lateral wall
  servo, front-wall re-reference shuffles, flood fill over an edge-locked wall
  map, hand-start protocol, and FRAM persistence of gains, sensor LUTs, the
  goal and the searched map.

  **There is no fast run in this baseline** — see `../docs/V2_BASELINE.md` for
  what was cut and why, and for the structural rules the fast run should come
  back under.

  ```
  cd main_mcu
  pio run                 # build
  pio run -t upload       # build + flash
  pio device monitor      # 115200 console
  ./test/run_tests.sh     # 15 host-side unit tests, no hardware
  ```

- `sensor_subprocessor/` — Teensy 4.0. Reads the 4 S15366-256 laser sensors and
  streams wall packets to the Main MCU over Serial5 @460800. Arduino sketch
  wrapped as a PlatformIO project (`src/*.ino`, code unchanged). **Done.**
- `esp32_radio/` — ESP32-C3 Super Mini (mouse side). Transparent byte pipe:
  UART1 (TX=21, RX=20, 460800) ⟷ ESP-NOW to the laptop dongle. **Done.**
- `esp32_laptop/` — ESP32-C3 Super Mini (laptop USB dongle). Transparent byte
  pipe: native USB serial ⟷ ESP-NOW to the mouse. **Done.**
- `lib_shared/EspNowLink/` — shared ESP-NOW pass-through library used by both
  ESP32 projects (via `lib_extra_dirs`).

The two ESP32 radios are pre-paired by hard-coded MAC (`MOUSE_MAC` /
`LAPTOP_MAC` in each `src/main.cpp`) and must sit on the same Wi-Fi channel
(`ESPNOW_CHANNEL`, default 1). Each prints its own STA MAC on the USB console
at boot so the constants can be verified. Nothing on either side interprets the
stream — it just mirrors the Teensy dual console off-robot.

Hardware reference: `../docs/micromouse_spec.md`.
Language/architecture decision: `../docs/firmware_architecture_adr.md`.
Console commands: `../docs/COMMANDS.md`.
