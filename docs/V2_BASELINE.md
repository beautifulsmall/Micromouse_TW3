# TW3_V2 — the baseline, and why it is this and not more

**Cut from TW3_V1 on 2026-08-18.** V1 is kept intact for reference at
`C:\Users\bob\Claude\Projects\TW3_V1`; nothing in this tree needs it to build.

---

## Why there is a V2

V1 worked. On 2026-08-01 it had 5/5 clean speed runs — search, map, verify,
sprint — after the slip-gate root cause was found. Between 2026-08-06 and
2026-08-15 it acquired self-calibrating sensor LUTs, diagonals, dense
elastic-band paths, a racing line, a beam observer, a pose observer,
`LocalGeom`, then `LocalMap` to replace `LocalGeom`, then `LiveMap`, then
`Horizon v2` to consume both.

Every one of those was a real answer to a real measurement. The problem is
what they did to the shape of the code:

| file | 2026-08-01 | 2026-08-15 |
|---|---|---|
| `Search.cpp` | ~90 KB | **208 KB** |
| `main.cpp` | ~95 KB | **138 KB** |
| `Control.cpp` | ~55 KB | 70 KB |
| `Horizon` + `LiveMap` + `LocalMap` + `LocalGeom` | — | 134 KB |

Two files held 346 KB between them, the localization stack was rebuilt three
times in a week, and the run configuration needed sixteen runtime toggles
(`race=`, `obs=`, `obsh=`, `dpath=`, `hz=`, `hzc=`, `hza=`, `hzp=`, `hzr=`,
`hzg=`, `hzal=`, `ldat=`, `clr=`, `starty=`, `v=`, `alat=`…) to say what the
mouse was even going to do. A `run_settings_line()` function existed purely to
record which of them were on.

So V2 is not a rewrite. It is **the same code with the last two weeks
removed**, plus the structural rules below so it does not happen again.

---

## What came across

Everything here is byte-identical to V1 unless the notes say otherwise. The
bench-measured constants, the dated comments explaining each one, and the
hardware-failure histories are the value — they were not rewritten.

**HAL** — `HAL.h/.cpp`. Pins, SPI, both UARTs, PWM, hardware quadrature,
control timer. Untouched since 2026-07-25.

**Drivers** — `Encoders`, `Motors`, `Fram`, `ImuIcm45686`, `Sensors`.
All five verbatim, including the bench-set sign/channel constants
(`ENC_LEFT_SIGN`, `MOTOR_LEFT_CH`) and the sub-MCU packet protocol.

**Control** — `Fusion` (Madgwick + gyro bias), `Profile` (trapezoid +
`extend`), `Walls` (beam → wall presence, the 165 mm side threshold and the
geometry argument for it), `WallAlign`, and `Control` itself: the 500 Hz ISR,
wheel velocity loops, fused motion estimator with the slip gate and its 120 ms
timeout, and the profiled maneuver executor (move / turn / arc) with the
lateral wall servo.

**Maze** — `Maze.h/.cpp` verbatim. Edge-keyed wall storage with unknown /
known-open / known-wall / LOCKED, optimistic and pessimistic flood, `best_step`
and `best_step_known`, raycast, save/load state.

**Application** — `Params` (FRAM gains, v10, all migrations), `SensorLut`
(FRAM LUT store + boot upload to the sub-MCU), `FlightLog` (FRAM flight
recorder), `Search` (the search mission), and two new files that are pure
file moves: `Console` and `Bench`.

**Other boards** — `sensor_subprocessor`, `esp32_radio`, `esp32_laptop`,
`lib_shared/EspNowLink`, all verbatim.

**Tools** — `TW3_sensor_cal.py` + its config, and `flight_viewer.html`.

---

## What was left in V1

| dropped | why |
|---|---|
| `Path.h/.cpp` (72 KB) | Clothoid + dense-diagonal + elastic-band + racing-line + reshape-splice planner. The whole fast run goes at once; it comes back as its own module. |
| `Horizon.h/.cpp` (79 KB) | Short-horizon replanner. Two full rewrites in ten days and it was still being bisected by console flag at the cut. |
| `LiveMap`, `LocalMap`, `LocalGeom` (55 KB) | The map-matching / wall-reconstruction stack. `LocalGeom` was superseded by `LocalMap` five days after it landed. |
| Speed-run half of `Search.cpp` (~2700 lines) | `S_PATH`, `S_SPEED_*`, `S_GOAL_CREEP`, `S_IMPACT_HOLD`, `S_DIAG`, the route compiler, the wall-edge / corner-front / finish gates, the beam and pose observers. |
| Speed-path executor in `Control` (~450 lines) | `control_path_*`, the cross-track hold, the launch datum, the impact latch, the clearance speed cap. |
| `'k'` corner-test harness | Built directly on `SpeedPath`. |
| 16 runtime toggles | Every one configured something that is no longer here. |

**None of it is lost** — V1 is on disk, and the findings worth keeping are
recorded below.

---

## Findings from the dropped work, so they need not be re-derived

1. **bert's maze is crooked, repeatably.** The first corridor is 1.86° off the
   power-on yaw reference, sd 0.12° across four careful placements (runs
   093/096/097/098). That is 30 mm over 900 mm. It is a property of a
   hand-built maze, not a calibration error, and a competition maze will have
   its own version. Any future localization must fly the maze in the room, not
   the one on the drawing.

2. **A y-on-x wall fit dies on diagonals.** Measured over logs 081/086/087:
   0.33 mm median residual axis-aligned, **32 mm** on a diagonal, width gate
   0% pass. Total least squares in normal form (minor eigenvector of the
   scatter matrix) gave 0.92 mm on the same diagonal — a 34× improvement — and
   is exact at every angle. If wall fitting comes back, start there.

3. **Wall pose error decouples exactly to first order.** For a wall in normal
   form `(n, c)`: `dc = -n·t` (translation only) and `dangle = -dθ` (rotation
   only). The first-order cross terms cancel identically. Heading falls out of
   a mean of angle residuals, translation out of a 2×2 least squares. No
   iteration, no Jacobian. Two parallel walls pin only the lateral component,
   which is the honest answer, not a failure.

4. **Dead reckoning does not scale to 16×16.** Measured wheel drift is
   3.8 mm/m (median, logs 066–081). A 16×16 route is 8–15 m, so 30–60 mm of
   accumulated error before slip — a third of a cell.

5. **Applying wall measurements as pose steps to a moving mouse diverges.**
   That was log 069, and it was a near-singular solve trusted as if full rank.
   A once-only measurement while stationary (the launch datum) has no loop to
   close and did not have this problem.

6. **The lateral servo gain is a length constant, not a time constant.**
   `1/k` in radians = 191 mm at 0.3 deg/mm. Heading → lateral position is a
   double integrator with no damping of its own, so as speed rises the
   available time shrinks and the phase margin goes. 0.15 (382 mm) is what is
   in this baseline.

7. **`ka` = 0 is policy.** A corrupted `ka` (~1.5e-4) in FRAM caused accel
   surge, braking wheel-lock and ~12 mm decel-tail rollback on every profiled
   move, and was misdiagnosed as stiction for two days. Check `ka` in `p`
   before diagnosing anything else about a move.

8. **255 means INVALID, not absence.** Both over-range and under-range read
   255. A non-return carries no information.

9. **Walls repeat every 180 mm, so a wide correction window aliases to the
   wrong cell** — and produces a *confident* half-cell error, which is worse
   than no correction.

---

## Structural rules for V2

These exist because of the table at the top of this document.

1. **`src/main.cpp` is boot and loop.** Nothing else. It is 160 lines; if it
   grows past ~250, something belongs in a library.

2. **`Bench` is not flight code.** Console UI and calibration harnesses only.
   Nothing in `Bench.cpp` may run during a mission. When it passes ~1000
   lines, split it per harness rather than letting it accrete.

3. **A module that reaches 1500 lines gets split before the next feature goes
   into it.** `Search.cpp` is 1430 today. That is the ceiling, not the target.

4. **The fast run arrives as its own module** (`lib/Control/Path` +
   `lib/Application/SpeedRun`), with the search executor untouched. If a fast
   run change needs an edit inside `Search.cpp`, the boundary is wrong.

5. **One experiment at a time, and it either graduates or is deleted.** V1's
   sixteen toggles were sixteen half-finished experiments kept alive by flag.
   A runtime flag is for a bisect that ends this week, not a permanent choice.

6. **A replacement deletes what it replaces, in the same change.**
   `LocalGeom` and `LocalMap` answered the same question for five days.

7. **`test/run_tests.sh` passes before anything is called done.** Fifteen
   host tests, no hardware, a few seconds. It caught a stale assertion in
   `test_move` that had been silently wrong since 2026-07-16 (see below).

---

## Changes that are NOT pure deletion

Three, and only three:

1. **`Console` and `Bench` split out of `main.cpp`.** Pure file moves; the
   harness code is unchanged. `print_threshold_report` → `bench_print_threshold_report`
   and `handle_line` → `bench_handle_line`, otherwise identical.

2. **`control_yaw_integral_enable(bool)` added** (`Control.h`). The
   heading-hold integral added on 2026-07-16 learns a trim coefficient and
   applies `dv_hold = c·v`, so the mouse holds heading even at `kyaw = 0`. That
   silently invalidated `test_move`'s contrast case ("kyaw = 0 visibly veers"),
   which had been failing ever since. The switch lets the test turn off *all*
   heading authority and see the weak wheel again, and it is also the right
   thing for any calibration that must measure the raw asymmetry rather than
   have it corrected underneath.

3. **`test/run_tests.sh` added.** V1's tests were fifteen hand-copied `g++`
   command lines in `test/README.md`, three of which had drifted out of date
   (missing `Profile.cpp`). One script, one command, all fifteen.

---

## One thing that changed meaning, not code

`search_map_save/load/wipe` and the FRAM map block came across intact, and the
map still survives a power cycle. But in V1 the **only consumer** of a restored
map was the speed run: `search_start()` calls `maze.reset()`, so a search has
always begun from a blank model. With the fast run gone, the boot-time
`search_map_load()` no longer changes what the mouse does.

It is kept anyway — `M` can show what the last search found after a power
cycle, and the data is there when the fast run returns. The console lines that
promised "wave to SPEED RUN" were corrected to say so. Making a search *resume*
from a stored map would be new behaviour, never flown in V1 either, and needs
its own thinking about where the mouse believes it is standing.

## State at the cut

- `pio run` (env `teensy40`): **clean**, no warnings.
  FLASH 109 KB code / 48 KB data; RAM1 62 KB variables.
- `test/run_tests.sh`: **15 passed, 0 failed**.
- Not yet run on hardware. Nothing here is new code, but the search mission has
  not been flown since the cut — first bench session should be
  `m` → `p` (check the FRAM gains loaded) → `n` → `f` → `e`.

---

## What has happened since the cut

**2026-08-18 — the fast run restarted.** `lib/Control/Path` (straights +
constant-radius arcs on one arclength coordinate, per-tick velocity planning
from measured position), a path executor in `Control`, `lib/Control/WallGate`
(the inside-wall edge gate), and the `k` single-corner bench harness. Nothing
of V1's clothoid / diagonal / observer work came back. Reasoning, and the
measurement that decides whether clothoids are needed, are in
`adr/ADR-002-fast-run-geometry.md`. Still unflown.

**2026-08-18 (later) — the loop rig.** `lib/Application/LoopSweep`: an
autonomous parameter experiment on a closed 6x6 ring. One (radius, arc speed,
slew rail) combination per lap, four corners per lap as four repeats, every
corner logged to FRAM as a 32-byte summary plus a distance-sampled trace of
the corner cell and its two neighbours. Resumable across battery swaps, and
guarded so it stops itself rather than hitting a wall. `docs/LOOP_RIG.md` has
the build sheet. The slew rail is a swept axis on purpose: it is the only way
to tell grip-limited from trackability-limited, and therefore the only way to
answer ADR-002's clothoid question with data instead of judgement.

**2026-08-19 — terminal-less operation and the radio made optional.** The
sweep's whole configuration is compiled in and it arms itself at boot, so a
session is flash-place-wave with nothing typed; the status LEDs say whether a
wave will start, resume, or is refusing to overwrite a stored set. The ESP32
radio is now compile-time optional (`HAL_RADIO_ENABLED`, default off) because
it costs battery and, more importantly, its floating RX pin would inject noise
characters into the console parser during an unattended run. Radio writes are
non-blocking and USB writes only happen when a terminal is open.

## First things to do

1. Flash it and confirm the boot banner, `p` (gains from FRAM), and `thr`.
2. Run a search on the practice rig end to end. That is the baseline claim.
3. Only then start the fast run, as `lib/Control/Path` — one corner geometry,
   proven on the floor, before anything is layered on it.
