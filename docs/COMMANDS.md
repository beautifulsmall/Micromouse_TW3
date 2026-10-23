# TW3_V2 — Serial / Radio Command Reference

Every command is a line sent over USB or the radio bridge to the main MCU,
**terminated by CR or LF** — `main.cpp` dispatches only on a line ending, so a
bare character sits in the buffer until one arrives. (That is also the second
line of defence on the radio: one stray byte cannot execute anything.)

**The radio is enabled by default** (`HAL_RADIO_ENABLED`, `lib/HAL/HAL.h`), so
floor tests can be started without a hand on the mouse — which matters for `j`
and the other calibrations, where touching the shell at t=0 is a disturbance
the measurement then carries. Leaving the ESP32 off the robot stays safe: RX5
(pin 21) is pulled up before `Serial5.begin()`, so an absent module idles high.
It does cost battery, so keep it consistent across any runs you intend to
compare.

Any command disarms a pending hand-start except `h`, `D`, `M`, and any
`name=value` assignment.

Setup tags: **[blocks]** = wheels off the ground · **[floor]** = on the
maze floor · **[clear]** = clear of walls · **[gyro]** = needs the IMU
calibrated first (it will say "attitude not ready" otherwise).

> The fast run is being rebuilt (see `adr/ADR-002-fast-run-geometry.md`).
> The single-corner harness `k` is here; the route runner (`x`) is not yet.
> V1's `diag` / `race=` / `obs=` / `hz*=` / `dclr=` / `ldat=` / `leds=` /
> `clr=` / `chain=` / `home=` / `starty=` toggles are not in this baseline
> and are not coming back as a group — see `V2_BASELINE.md`.

---

## Missions

| cmd | what it does |
|-----|--------------|
| `h` | Arm the hand-start THE WAY BOOT DOES (2026-09-21): the ROUTE if FRAM holds a proven map, else the SEARCH — before that day `h` always armed the search, which after MR3 re-searched the rig and threw away the ladder and the goff table. `h.search` forces the search hand-start regardless. Typing at the console stands the armed wave down (you are at a terminal, not about to wave) EXCEPT for assignments (`x=..`) and reads — `y`, `lr`, `lr,N`, `lr.all`, `mr`, `p`, `D`, `M` — so the between-runs routine (lap ends armed, fetch the log, read the report, wave) never needs a re-arm (2026-09-21). Wave a hand <180 mm in front to launch; the IMU calibrates after the hand clears. Hold the hand there 5 s to WIPE the stored map first. The mouse arms itself at power-on — a proven map in FRAM arms the ROUTE instead, and since 2026-09-15 the same 5 s hold on the route's hand start wipes the map, clears the goff table and hands the wave to the search (remove the hand: MR1 launches), so the gesture means "new unknown maze" whichever mission is armed. **THE START GATE (2026-09-30):** a wave only counts once BOTH side beams have read the start cell's walls, steady (no packet-to-packet jump over 12 mm), for 300 ms. Until then the LEDs **alternate** (LED1, LED2, 150 ms each) — in the hand, or a sensor that came up wrong after a quick power cycle; still alternating with the mouse on the floor in the start cell = the power-on fault: power off, wait a few seconds, power on. The console says `start gate: side walls L .. R .. -- ready for the wave` when it passes, and why not after 2 s. Ready: the search shows LED1 steady, the route its usual both-LEDs slow blink. Lifting the mouse (both side beams gone) cancels a wave in progress, so it can be picked up after a run to clean the wheels. The route takes the sides to judge from the map at the launch pose (a practice start cell open to one side judges only the other); the search assumes the rules' start cell, walls both sides. **No power cycle after the search (2026-09-30):** a search that completes (map proven, back on the start cell) hands straight to the route — the lap end's datum sequence squares the mouse on the start cell's walls (side wall, centreline, rear wall, facing NORTH) and arms MR2 with the lasers still on. `y` dumps the search's log until MR2 launches. Pick it up for the wheels if wanted: the start gate holds the wave until it is back in the start cell. An ABORTED search still ends idle (the mouse is not in the start cell): carry it back and power cycle, or `h`. |
| `e` | Start the ROLLING search (Explore.h, 2026-09-12): a SpeedPath extended a cell at a time at `sv` mm/s, R60 turns inside the cell, wall-end and front-wall anchors every cell, explore-all as configured, map saved at the end, flight log recorded (decimated, 'y' dumps it with search column names). Pre-flight (2026-09-18): while the mouse stands on the start cell the side beams are tallied — left must see the outer wall, right the start cell's east wall; a beam stuck at 255 (powered on too soon) refuses the search BEFORE the flight log is armed, so the previous log survives. If a run is active, `e` aborts it. |
| `search.old` | The stop-and-sense search (Search.cpp: boot-align leg → goal → explore → home). Kept for comparison. |
| `E` | Toggle **explore-all** (compiled default: ON; RAM only). **Route-aware since 2026-10-02:** after the goal the search visits only unknown cells that could lie on a route within 4 cells (`EX_ROUTE_SLACK_CELLS`) of the best KNOWN route — dead ends and pockets off every such route are never entered, and it ends with `-- ROUTES PROVEN ... --` when none are left. On random 16×16s: the true shortest route every time, in ~170 steps against ~370 to map every wall. On, `e` maps every *reachable* wall before coming home; off, it stops the moment a route to the goal is proven — the competition behaviour, which routinely leaves a third of the maze unseen. *Reachable* is the operative word: an enclosed 6×6 inside the 16×16 model leaves 220 cells unknown forever, and a completion test that waited for those would never fire. |
| `goal=x,y` | Set the goal to one cell. `goal=x0,y0,x1,y1` for a block; `goal=std` restores the competition 2×2 centre. Persisted in FRAM. |
| `map=save` / `map=wipe` / `map=load` | The searched map in FRAM. A completed search saves by itself and boot restores it, so a power cycle resumes rather than starting blind. |

## Calibration & tuning tests

| cmd | setup | what it does |
|-----|-------|--------------|
| `m` | [blocks] | Motor self-test — spins each wheel, checks direction. |
| `s` | [blocks] | Duty sweep 0.05→0.45 — fits `vmaxl`/`vmaxr` wheel-speed feedforward. |
| `v` | [floor] | Velocity-loop test — commands target speeds, logs left/right (CSV). |
| `c` | [floor][clear][gyro] | Straight-line calibration — alternating ±legs fit `strim` (heading trim). |
| `t` / `t.track` | [floor][clear][gyro] | Turn auto-tune — 90° turns in place at the registry's `tomg`/`talp`, adjusting `kpt` (turn-only wheel gain) until the cruise reaches 82 % of `tomg`, settles within 5°, and the plateau is slip-free. SLIP-AWARE since 2026-09-16: undershoot with slip on ≥15 % of the plateau LOWERS kpt (÷1.3) — raising the wheel gain on a slipping turn only buzzes the tyres loose harder (kpt 0.0088–0.016 buzzed and ran 220–330 of 450; 0.004 ran 426, silent); clean undershoot raises it (×1.25); range 0.002–0.012. Prints a CSV per attempt (yaw_ref, yaw, yaw_err, omega_ref, yaw_rate, wheel_diff = half the wheel differential, slip) and the attempt's slip %. The `track` fit (clean turns: 67–69 mm vs the stored 72.3) is REPORTED, not applied — `track` is also the arc feedforward and the corners were tuned with 72.3; `t.track` applies it. `w` persists kpt. |
| `a` | [floor][gyro] | Arc test — one radius-90 arc, logs tracking (CSV). |
| `r` | [floor][gyro] | Repeated-arc test — several 90° arcs back-to-back. |
| `g` | [floor] | Wall-sensor geometry capture — averaged snapshot at a held pose. |
| `f` | [floor][gyro] | Wall-align run — drives beside walls, fits the corridor axis (CSV). |
| `l` | [floor][gyro] | Lateral-servo run — start off-centre; logs how it recenters. |
| `k` | [floor][gyro] | **Corner harness** — flies ONE gated constant-radius corner in an L-corridor (2 cells in, 90°, 2 cells out, wall at the far end; mouse at the first cell centre). Reports exit yaw error, exit lateral, front path error and what the edge gate did. `kl`/`kr` = left/right, `kr80`/`kl100` = radius (45–125). See `adr/ADR-002-fast-run-geometry.md`. |
| `absb=0.12` | | **The ABS** (2026-09-24). Registry #63. Each wheel's velocity TARGET is clamped to within this fraction of body speed (the fused/IMU estimate ± the commanded differential; 250 mm/s floor) before the wheel loop sees it — on straights; in an arc (v7, 2026-09-26) the wheel MEAN is judged against the body and both targets shift together, so the turn's differential is never touched (the lateral test's circles had shown clamps on every yaw twitch), so the loop never asks for more slip than the tyre can deliver — a braking wheel is held at body − band, still rolling, still counting; a launching wheel at body + band. While clamped the acceleration feedforward is the body's measured acceleration and the estimator stops fusing the encoders. (The first version, a duty override, limit-cycled against the loop and coasted into a wall — v2 the same evening.) Exempt in in-place turns. `absb=0` switches it off. The route's leg end and the slip test print how often it acted. Built after the second slip ladder's collisions, which were locked-wheel slides at 10000+ mm/s². |
| `slip A[,mm]` | [floor][gyro] | **Launch slip test** (2026-09-23) — a triangular profile on a straight: accelerate at A mm/s² to the peak, brake at A to rest, over `mm` (default 900 = 5 cells). Reports the tyre limit three ways: ACCEL and BRAKE each as wheel Δv vs the IMU's body Δv (slip %) with that phase's duty peak (`<< MOTOR LIMIT` when it reached 0.85 — a different fix from grip); and the RULER, believed distance vs the front wall before/after (start about mm+100 from a wall so it is in beam range at the end). Ladder A from 1200 (the flown ralon) upward: `slip 2000`, `slip 3000`, `slip 4000`… until slip % leaves ~0 or the motor limit prints. The IMU integral carries 1–2 % of tilt error, so read the trend and the ruler. Pure longitudinal from rest = the launch case; the mid-run (cornering-loaded) limit is SLIP_LOOP.md's ralon ladder. The `obs_head` column carries left − right wheel velocity (÷100 in the dump; the viewer's `vl-vr` channel restores it) — which tyre let go. |
| `lat A[,R][,deg]` | [floor][gyro] | **Lateral limit test** (2026-09-25) — the slip ladder for the corners, in one run. A circle of radius R (default 84, the route's) whose yaw rate ramps to the rate that gives lateral A (default 6000 mm/s²) over deg (default 720, two circles; sign = direction), so the lateral load v²/R climbs from 0 to A through the sweep. The report bins the sweep by commanded lateral: measured ay ÷ commanded, measured yaw rate ÷ commanded, yaw_err — and names the **knee**, the first bin where the tyre stopped following (ay under 85 % or omega under 90 % of the command: the mouse running wide). A race `ralat` is ~0.8 of the knee. Flight log (`y`): ay vs `lat_cmd`, omega vs `omega_cmd`, `ay/cmd`, `om/cmd`, `R_actual`. Needs ~2R + 120 mm of clear floor, no walls. The slip test's peak is now capped at 3500 (was 2500) for long floors: `slip 6000,2200` reaches it. |
| `j` | [floor][gyro] | **Scale test** — drives 5 cells (900 mm) straight at slip-free speed; reports believed L/R distance. Measure physical travel, then `circl = circl × phys/believedL`, `circr` likewise. `j720` = any distance in mm. |
| `b` | [floor] | Front-wall stop test — face a wall ≤220 mm; checks dead-stop accuracy. |
| `n` | [floor] | Sensor readout — free-run beam stream (CSV), with yaw and IMU temperature. `n` again stops. |
| `u` | [floor][clear][gyro] | Contact probe — creeps to a front wall and stops on wheel-stall/accel contact (±0.2 mm). Self-cal rung 1. |
| `C` | [floor][clear][gyro] | **Front LUT sweep** (uppercase). Face a wall ~250–300 mm back: creeps in logging raw front px→odometry, stalls at contact for the zero (centre 45 mm), then prints + holds pending. Press `W` to commit. |
| `S` | [floor][gyro] | **Side LUT sweep** (2×4 special rig, outer walls only). Rear against south wall, laterally centred, facing north. Runs two diagonals, raycasting each side beam to the known walls → perpendicular-from-centre tables. Press `W` to commit. |
| `R` | [floor] | **Raw-px monitor** (uppercase). Streams raw pixel + signal per beam at ~10 Hz. `R` again stops. |
| `enc` | — | **Wheel velocity source monitor** (2026-09-25). Prints, at 20 Hz, the count-per-tick velocity, the edge-timed velocity and the filtered value the wheel loop is using, per wheel, the direction as the ENC count and as the A/B edge order see it (`d+1/+1`), plus edges captured per phase (`e 820+820/820+820` after exactly ten turns of each wheel — a phase that reads 810 is missing one slot per rev: dust or a damaged slot), time since the last edge, DMA overruns and direction mismatches. The counts restart from zero each time the monitor is switched on. Spin a wheel by hand: the edge column should read a steady number where the count column shows a 0/103/206 staircase, both with the same sign. A read — it never disarms the wave, and it keeps printing through a harness. `enc` again stops. |
| `vsrc=0` | | **Wheel velocity source** (2026-09-25). Registry #64. `0` = counts per tick (0.2058 mm per count at 500 Hz, a 103 mm/s LSB softened by `vfilt`), as always. `1` = **edge timing**: the four encoder signals are also routed through the XBAR to four DMA channels that stamp every rising edge with an 8 MHz timer, and each tick the velocity is edges × 0.823 mm ÷ the time they span (M/T method) — timer resolution instead of loop resolution, one edge of lag, and a stopped wheel reads zero within an edge-time. With no edge in a tick the estimate decays as one edge over the time since the last. `vfilt` still applies. Check it with `enc` before switching. Bert's XBAR/DMA capture (`firmware/example_code`), made a driver in `lib/Drivers/EncoderTiming`. |
| `kaa=0` | | **Angular acceleration feedforward** (2026-09-25). Registry #65. `ka` used to multiply the whole per-wheel acceleration reference — the launch's linear part AND the differential the omega rail commands at every arc entry and exit (rslew 7000 °/s² × half-track ≈ 4400 mm/s² per wheel, 0.22 of duty at ka 5e-5, reversed at the exit). `ka` was fitted on straights, and the first D9 laps with it under-rotated every corner (+3.8/−5.4/+10.2° exit error against +0.8/+0.1/+2.8 on 09-22) and hit the 45-out post twice. Now `ka` is the linear gain only and `kaa` the differential one; `kaa=0` (default) flies the arcs exactly as 09-22 did. Swept the same evening: 1e-5 flew a complete lap, and 2e-5 → 1e-4 under-rotated more at every step and hit the 45-out post every time — the pulse costs the inner tyre its grip. Leave at 0. |
| `rbrk=0` | | **Route brake deceleration** (2026-09-25). Registry #66. The planner's braking curves into corners and to the leg end use this; `0` (default) = `ralon`, as it always was. Why separate: a launch has no lateral load and takes 5000+, but the brake into a corner runs into the clothoid where the lateral is building — at `ralon=5000` the 45-in read ax −8400 with the arc already on, the tyre let go, the yaw lagged 6.6° then overshot, and the V after it was entered 13° behind and hit the post; `ralon=4000` flew MR2–MR5 clean. So `ralon` high for the launch and straights, `rbrk` at the number the corner entries have proven (4000). |
| `mra=1` | | **Ladder acceleration scale** (2026-09-26). Registry #67. The run ladder MR2→MR5 steps the out-leg peak (`rv2`..`rv5`) and nothing else; with `mra` under 1 it steps the CORNER speeds too: `ralat` is multiplied by `mra` at MR2, rising linearly to ×1.0 at MR5 (`mra=0.4`: ×0.4, ×0.6, ×0.8, ×1.0). Until 2026-10-04 `ralon` and `rbrk` were scaled as well; now they are the registry's on every rung (bert: "the long straights should be at max accel, not limiting by max V" -- the risk is in the corners, so the ladder is the corners'; set `rv2`..`rv5` to the straight-line ceiling you trust and let `mra` and `rvv` make MR2 the safest fast run). The rung line at launch prints the numbers it flies. The HOME leg (rvh, 500) is not scaled — it flies the registry's accelerations on every rung (2026-09-26: scaling it made it the slow half of every lap for no safety it needed). `1` (default) = every rung at the registry's accelerations. For race day, so the banker runs are conservative and only MR5 asks for everything; for a tuning ladder keep it at 1, so one variable stays one variable. |
| `vref=0` | | **Battery compensation** (2026-09-26). Registry #68. A DC motor's torque is set by the voltage across it, V_batt × duty; the duty law (`vmaxl/r`, `fricl/r`, `ka`) was fitted at one pack voltage and the pack swings ~15 % over a session. With `vref` > 0 every wheel duty — feedforward, friction, acceleration and the PD trim together — is multiplied by `vref / vbat` before the `dmax` ceiling, so the torque delivered is what the law asked for at the calibration voltage. `vref` is in the `vbat` instrument's own units (what `vbat` and LAPREP print; the divider need not be calibrated, only the ratio matters): set it to the reading the current tuning was made at (2.00–2.15 for the September ladders). Clamped ×0.75..×1.35, off below 1 V (no sample). `0` (default) = off. `vbat` prints the factor in force. |
| `chold=1` | | **Clothoid hold** (2026-09-26). Registry #69. The planner used to brake to a corner's speed *at the arc* — through the ramp-in clothoid, while the lateral load was coming on — and launch out of it through the ramp-out. On the new maze's MR5 that put 4400 mm/s² of brake under 4000 of lateral at the V (2,2), the tyre let go, the yaw arrived 15° late and the mouse hit the post (the same mechanism as the 09-25 ralon-5000 crash). With `chold=1` the entry clothoid's speed cap is the arc's and the braking target is the clothoid *start*, so the brake is finished before the curvature begins; the exit clothoid holds the arc speed until the curvature is gone. Costs ~15 ms a corner. `0` = the old planner. The rung line prints "clothoid hold" when on. |
| `rdiag=1` | | **Diagonals on/off** (2026-09-26). Registry #70. `1` = the route compiler may fly diagonals — 45s, 135s and Vs — as it always has. `0` = ORTHOGONAL ONLY: every staircase compiles as the 90s it was, on the R84 geometry with 43 mm of inside clearance and the most proof behind it. The banker's switch: the evening the new maze's home leg hit posts twice on MR2 (a same-hand 45-out → 90 mm → 135-in → V, entered 20 mm inside off the diagonal's exit offset with no straight to recentre in), MR2–MR4 should bank a time on 90s and the diagonal form be asked for at MR5 only — or not at all on a maze whose diagonals pack same-hand turns. Compiled per leg at arm, like `r135`; the leg line says "ORTHOGONAL ONLY" when off. **The home leg of a two-leg run is always orthogonal** (2026-09-27): the return is not timed, and the home diagonal is where the first `rdiag=1` lap on the 09-27 map clipped a post. |
| `rdv=1000` | | **Diagonal speed ceiling** (2026-10-03). Registry #74. The out leg flies diagonals only when its peak speed is at or under `rdv`; above it the leg is compiled orthogonal (`route: peak .. is above rdv .. -- this leg flies ORTHOGONAL`). At Hazlemere both MR4 (1500) out legs hit a post leaving a V, where MR2 and MR3 had flown the same diagonals to the goal: the V-exit lane change cannot walk `vout` back at 500–760 mm/s. 1000 = diagonals at MR2/MR3, 90s at MR4/MR5 (on the test maze: 41.5 / 34.1 / 30.7 / 27.6 s predicted). `0` = no limit. The diagonal-into-U rule of that day (`ROUTE_DIAG_EXIT_TO_UTURN_MM`) is OFF since 2026-10-04: the U it protected had been handed a 44 mm error by the V before the 135, and its replacement zigzag crashed at once (hazel10); ~44 U-turns flew clean that day. **Race day 2026-10-04: `rdv=0`** -- the orthogonal MR4/MR5 is the worst route type (0/4 un-merged zigzags) and on the second test maze was slower than the diagonal MR3. A V-into-opposite-135 rule (`ROUTE_V_TO_OPP_135_MM`) exists but is OFF (0): its un-merged chain is an R60 zigzag that ran wide and was governor-stopped at MR2 (hazzel8), where the V -> 135 pair itself had flown MR2/MR3. |
| `rvv=320` | | **V speed cap** (2026-10-04). Registry #75. A V, and the turn it hands into within 120 mm of straight (`ROUTE_V_CAP_AFTER_MM`), are built and flown at no more than `rvv` mm/s on every rung -- the cap travels in the corner like the fit's own speed rungs, so the planner brakes for it and the clothoid is sized to it. From the Hazlemere analysis: the V arc itself was never hit (9 clean passes), every V crash was at its exit or at the 135 65 mm after it, and all of them above MR3 corner speeds; 320 is the V's own speed at MR2 (R64 at ralat x0.4), where it does not bind. Costs 0.6-1.3 s at MR5 on the two test mazes. `0` = no cap. |
| `rvu=0` | | **U-turn speed cap** (2026-10-04). Registry #76. The same cap for a U-turn (its clothoid is solved at the corner speed; slower follows it with room to spare). ~44 Us flew clean at 338-476 mm/s on the test day, so it ships at 0; 340 (the MR2 U speed) costs 1.5-3.6 s at MR5 on a six-U maze. `0` = no cap. |
| `vlearn=0` | | **V timing learner on/off** (2026-10-04). Registry #77. `1` = the V's first exit post teaches its timing for the next run (DiagLane.h, the `h=Ev` goff rows); `0` = nothing is learned and a stored V timing is never applied. Off for the race: on the small maze's chain of three Vs the dead reckoning (seeded `vout` outside) was 24 mm wrong at every V, and the learner moved all three +12 mm late after MR2 and to the +20 clamp after MR3; at Hazlemere it moved (6,11) +7 late from a post 165 mm out and MR4 hit the outer post leaving it. The 90-corner goff learning (`glearn`) is separate and stays. |
| `ghome=0` | | **How the home leg leaves the goal** (2026-10-04). Registry #78. `0` = stop on the goal cell, turn in place, press back onto the first known wall behind within 3 cells (the goal datum), fly home. `1` = stop, then fly a LOOP: one step ahead into the 2x2 goal block and out through the block's OTHER door, following the home flood — no turn in place, no press (bert: "the goal may have another entrance opposite the one we come in so there will be no wall to back up to"; set it when you see the maze). A goal whose only door is the one the mouse came in by REFUSES the loop (the walk would come back through the arrival cell: a 270 in two cells of room, four tight arcs) and the mouse turns in place as for 0, saying so. The loop launches from the arrival pose, so a mapped side wall unseen there (the stop fell short) is tolerated by the pre-flight when that beam was live on the leg just flown. Console: `GOAL LOOP (ghome 1): the home leg flies on through the 2x2 block and out of its other door`. Growing the registry past 77 moved the flight log's record base (FLOG_PARAMS_BYTES 320 → 416): dump any stored log before flashing this. |
| `rturnh=8` | | **Home-leg turn cost** (2026-10-01). Registry #73. The home leg chooses its route like the out leg (fewest cells vs fewest turns, scored by time + cost per turn) but with `rturnh` cells per turn instead of `rturn`: it is not timed, so a longer route with fewer turns -- fewer posts to miss -- wins easily. `0` = use `rturn`. On the 09-27 and 10-01 mazes the way home is forced (one corridor network), so it changes nothing there; `rvh` is the lever for the zigzag. |
| `dwheel=1` | | **The duty mixer** (2026-10-01). Registry #72. `dmax` (0.85) now caps the COMMON MODE -- the mean of the two wheels, i.e. speed -- and each wheel may go to `dwheel` to keep the steering differential: at full throttle a yaw error can still be corrected (one wheel to 1.0, the other below 0.85, never both at 1.0). If a wheel would pass `dwheel`, the speed gives way, not the steering. Before, each wheel was clipped at 0.85 on its own, and at the top of every launch/brake both sat there with zero differential (the 09-30 MR5 held 13 deg of heading error for 150 mm at 1500 mm/s and hit a post). The leg-end line reads `duty peak speed .. of 0.85, wheel .. of 1.00`. `0` = the old per-wheel clip. |
| `rturn=3` | | **Turn-weighted route choice** (2026-09-26). Registry #71. The cell flood counts cells and nothing else; when a wall moved on the new maze it sent MR3 down an 18-cell route with eleven R45/R60 zigzag turns (the mouse hit a wall on the fourth) in preference to the 20-cell route with six turns and a U it had flown all evening. With `rturn` > 0 the route walk is a search over (cell, heading) with cost = cells + rturn × turns (a 180 counts twice), over known-open edges only. At 2000 mm/s a 90 with its brake and launch costs three to four cells of time, and every turn is a post to miss. `0` = the plain cell flood. Set at arm, per leg. **Since 2026-09-27 the route is chosen by flight time:** with `rturn` > 0 the compiler builds BOTH the fewest-cells walk and the fewest-turns walk (diagonals merged, radii fitted), flies each through the planner at the leg's own speed and accelerations, and keeps the lower of *time + rturn cells of time per compiled turn* (a U counts two). The leg line prints both candidates, `>>` on the one flown. So a staircase that compiles to one diagonal is scored as the 2–3 arcs it flies, not the 8 grid turns it walks — and with `rdiag=0` the zigzag still loses to the fewer-turn route. |
| `W` | — | **Commit sensor LUT** (uppercase; distinct from lowercase `w`). Writes whatever `C`/`S` left pending to FRAM and uploads live to the sub-MCU. Boot re-uploads from FRAM. |
| `sweep.restart` | — | Start a new sweep session, clearing the log. **REFUSES if the region holds a flight log** (i.e. after a search) — `y` and `M` read that instead. `sweep.restart!` overrides and erases it. |
| `lutpull` | — | **Persist a GUI cal.** Pulls the sub-MCU's LIVE LUTs back over Serial5 into main FRAM. Stop the GUI stream (`q`) first, or the next boot overwrites your cal with the old FRAM. |
| `lutclear` | — | Invalidate the stored LUT block → next boot uploads nothing and the sub-MCU keeps its compiled defaults. |
| `thr` / `thr!` | — | Sub-MCU threshold self-test report (`thr!` re-measures the room first). Also printed at every boot. |

## Loop sweep — autonomous corner experiment

Drives a closed ring for minutes, changing one corner parameter per lap and
logging every corner to FRAM. Build sheet and analysis notes: `LOOP_RIG.md`.

**None of these commands are required.** The whole session is compiled in
(`lib/Application/LoopSweep.h`) and the sweep arms itself at power-on, so the
workflow at the maze is flash → place → wave, with the status LEDs saying
what a wave will do. Every boot returns to the compiled values; these
commands exist for trying something while tethered. Verbs are words, not
single characters — this one drives the mouse unattended and should not be
one keystroke away from a mistyped calibration.

| cmd | what it does |
|-----|--------------|
| `sweep` | Print the grid, the implied `a_lat` for every (R, v) pair, the FRAM budget it needs, and how far the session has got. **Read this before waving.** |
| `sweep.r=125,100,80,60,45` | Radii to sweep (1–8 values, 45–125 mm). Largest first, and **radius varies fastest** in combo order, so the first loops of a battery cover every radius. |
| `sweep.v=500,600` | Arc speeds, mm/s (1–8 values, 80–900). `a_lat` is derived as v²/R, so the grid reaches 0.82 g at R45/v600 — deliberately past the tyres. **The slow rows were cut in v13**: run 128's mean `exit_lat` is monotonic in v and crosses zero near 400, so v200–v400 were both the least informative and the worst-behaved cells in the grid. |
| `sweep.slew=2500,6000` | Angular slew rails, deg/s² (1–4 values). **The axis that separates grip from trackability** — see `LOOP_RIG.md`. |
| `sweep.dir=r` / `sweep.dir=l` | Ring direction: `r` = clockwise = right turns. One ring gives one direction, so run both. **Compiled default is `l` from v14** — every corner before run 133 was a right turn and `tasym` had never been tested. Place the mouse with the inner block on its **left**. |
| `sweep.trace=4` | Trace spacing in mm of path (0 = summaries only). Distance-based, so the cost does not depend on speed. |
| `sweep.arm` | Arm. A wave launches the whole remaining grid; it runs to completion or a guard. |
| `sweep.stop` | Abort a run, or disarm if only armed. Records written so far are kept. |
| `sweep.dump` | Dump the log as CSV — summaries (`S,`), stop records (`X,`), then traces (`T,`). **Use USB.** |
| `route` | Print what the mission believes (next leg, believed pose), then compile the stored map into a path **without flying it**. Floods pessimistically from (0,0) N to the goal, merges any two same-direction turns in **adjacent** cells into a single **180 U-turn** (R89.7 at v500 — the maze fixes the radius, since the exit corridor is one cell from the entry), then picks a radius per turn off the ladder (R84 → R73 → R60 → R45) for what is left, applying the relaxed same-direction minimum to any same-direction pair the U-turn could not take. A U-turn that will not fit falls back to two 90s, so this can only make a route better or leave it unchanged, and prints every turn with the radius it got and whether it had to drop. Answers the 2T question on the bench: two consecutive arcs need `T(a) + T(b) + 20 mm` between their cell centres, and cells are 180 mm apart, so a hairpin cannot be flown at R84. Read-only — it copies the maze and never touches the mission's own model. |
| `route.arm` | Arm the route run on the hand start. A wave flies **start → goal → start** as one continuous path (`ROUTE_RUN_OUT_AND_BACK`) — the goal is not a special case, it is compiled as one walk so whatever heading change sits there becomes an ordinary turn, a U-turn, or nothing at all. A lap that lands back on the start cell self-turns to the launch heading and re-arms — **do not reposition the mouse between laps**; it knows where it is. Refuses when the stored map has no pessimistically-proven start→goal route. Stands the search's hand start down — one mission owns the wave. |
| `route.stop` | Abort a flying leg (brakes — reverse duty, not a coast) or stand the arming down. |
| *(recovery re-arm)* | Automatic (2026-10-04, race morning; bert: "after a crash / abort wait 20 s and then get ready for another start ... so don't power off the sensors -- this way there is no power off/on problem"). An aborted leg, a stop between legs and a pre-flight sensor fault no longer park the mouse until a power cycle. They print `route: RECOVERY -- 10 s to put the mouse back on the start cell...`, keep the sensor stream running, pulse both LEDs together 1 s on / 1 s off for 10 s (20 s until 09:40 that day), then reset the pose to the start-cell convention (0,0 facing NORTH, against the rear wall, hand placed) and re-arm the wave at the SAME rung (the ladder climbs only on a goal reached). The hand start then runs the IMU calibration, the pre-flight against the map and the start gate's two walls as usual — a beam still dead is refused again and the window repeats. `route.stop` during the window cancels it (idle). Before this a quick power cycle was the recovery, and a quick power cycle is what brings the sensor MCU up stuck at 255. |
| *(pose not confirmed)* | Automatic (2026-10-04). The pre-flight before a leg compares the side beams with the map at the pose the leg starts from. A beam that reads 255 where the map has a wall used to be a SENSOR FAULT; if that beam read live at least 100 times on the leg just flown it is not dead — the mouse is not where the leg thinks it stopped (the goal stop is dead-reckoned from the last corner; on the small maze MR3/MR4/MR5 all stopped past the (2,4) wall end with 1800 live readings behind them). The console says `route: POSE NOT CONFIRMED ... This leg is NOT flown.` and the recovery window starts at once; the leg before it is complete, reported and has climbed the ladder. A hand start is judged on fresh evidence only (the counters reset at arm), so a sensor that comes up dead at the start cell is still a fault. |
| `sweep.recover` | Read the summary region **without trusting the header**. A wipe rewrites 256 header bytes and leaves every record where it was, so a session whose index was lost is still readable until a new session writes over it from record 0. Prints how many contiguous plausible records survived, then dumps them in the normal CSV form. Read-only — it never writes FRAM. Use it whenever `sweep.dump` reports a log you know you flew. |
| `sweep.restart` | New session, log cleared. Dump first. |
| `sweep.probe` | Stand still and read both side beams for 40 packets. Prints L, R, their sum, the raw `(R−L)/2` and the wall model's lateral. A hand-centred mouse in a 168 mm corridor reads 84 / 84 / 168 — anything else is the ruler the whole experiment is measured with, being wrong. Disarms the sweep, like `sweep.dump`. |

**Why a session stopped.** Every failed loop writes an `X,` row — fail reason,
path `s`, which corner it was heading for, lateral **and the lateral one packet
earlier**, yaw error, both raw beams, and how long it had been running. The
pair of laterals is the point: a stop that arrives from nowhere is an artefact,
one that arrives from an already-large lateral is the mouse genuinely running
out of corridor. Traces also drop
a breadcrumb every 60 mm of corridor, so the trail no longer goes dark between
corners. If a run ends with **no** `X,` row and `loops_done` in the dump
header has not advanced past the last logged loop, the firmware never noticed
anything was wrong — suspect power, not path.

Changing any grid setting resets progress: a new grid is a new session.
Progress lives in FRAM, so a flat battery mid-session means swap, power on,
wave, and it continues at the combo it reached.

**A stored session is never discarded automatically.** If the log holds a set
this firmware did not produce — you reflashed with a different grid or the
other direction — the mouse blinks both LEDs fast, a wave does nothing, and
only a 5-second hand-hold discards it. Same gesture vocabulary as the
search's map wipe: the benign outcome is the short one.

## Radio / power

`HAL_RADIO_ENABLED` in `lib/HAL/HAL.h` defaults to **0** — the ESP32 module
is not required and not read. With it removed, RX5 would otherwise float and
decode noise into console characters (a stray `m` is a motor test). Console
output still goes to USB when a terminal is open. See `LOOP_RIG.md`.

## Observability & data

| cmd | what it does |
|-----|--------------|
| `p` | Print all runtime parameters + FRAM status. |
| `M` | Dump the current maze map (ASCII), any time. |
| `D` | Toggle the 50 Hz debug telemetry stream. |
| `y` | Dump the FRAM flight recorder as CSV. Survives power cycles. Paced for the radio link. Since 2026-09-14 the ROUTE log is decimated by DISTANCE like the search's: one record per `ds` mm of path (banner `ds2.0` etc.; 2 mm floor = every packet on a rig, sized up at launch so a whole 16x16 two-leg run fits 90 % of the region, ~2.7 mm typical) plus every event packet (gate correction, front gate, diagonal lock, arc entry/exit — everything that flashes an LED) plus one per 100 ms stationary. The launch never gets overwritten: past 95 % full the spacing quadruples; the search widens its 5 mm at 50 % and 75 % full. Log size is a function of distance, not time. |
| `z` | Zero the encoder counts. |
| `w` | Save the current runtime parameters to FRAM. |
| `param.diff` | Only the parameters that differ from compiled defaults — the **shadowing detector**. Run before believing any analysis quoting a source constant. |
| `param.dump` | Machine-readable registry: `PARAMDEF,<name>,<value>,<min>,<max>` per line, then `PARAMDEF,end`. The GUI's discovery call. |
| `mz.v=<i>,<hex4>` / `mz.h=<i>,<hex4>` | Stage a wall-bitmap plane for a map push (i = 0..16; v bit y = wall between (i-1,y),(i,y); h bit x = wall between (x,i-1),(x,i)). Silent ack. |
| `fram.info` | The FRAM chip: family (Ramxeed/Fujitsu or Infineon/Cypress from the RDID), size measured by an aliasing probe, the size this build is laid out for (`FRAM_MBIT`), flight-log capacity. |
| `fram.dump` | The stored variables -- params, goal, sensor LUTs, searched map, goff table: the 2 KB small-block window -- as 64 lines `FRAM,cfg,<addr>,<64 hex>` and `FRAM,end,cfg,2048,<crc32>`. The GUI's **FRAM backup** button files it as `.fram.json`. Not included: the flight log (`y`) and the lap report (`lr`). |
| `fram.clear` / `fram.stage,<addr>,<hex>` / `fram.commit,<crc32>` | Write a backup back: staged in RAM line by line, committed only when all 64 lines are present and the CRC matches, then read back and verified. **Power cycle after a commit** -- every module reloads its block at boot. The GUI's **FRAM restore...** button does the whole sequence from a `.fram.json`. Chip swap: backup on the old chip, fit the new one, flash `pio run -e teensy40_fram8`, restore, power cycle. |
| `mz.commit` | Adopt the staged 34 planes as the searched map (every edge KNOWN, nothing locked/visited), persist to FRAM, report goal reachability + route availability. Refuses an incomplete stage. |
| `mz.clear` | Drop a half-staged push. |
| *(two-leg run)* | Automatic. A route wave first tries the continuous loop S->G->S; on a map with no loop through the goal (a dead-end goal, a 2x2 goal with one way in — every rules-compliant maze from D11 on) the console says `no continuous loop through the goal (...) -- TWO LEGS` and the mouse flies S->G, stops on the goal cell's centre, turns in place to the home flood's first step (450 deg/s), and flies G->S compiled from the goal — one wave, one flight log (banner `route:S->G|G->S`, flags bit6 on the home leg, `s` restarts at 0), two lap reports (`dir=o` out, `dir=h` home), then the usual lap-end datum at (0,0). After the goal turn, if the map knows a wall behind the mouse — directly behind, or (2026-10-04) the first known wall within 3 cells of known-open cells, which in a 2x2 goal is the block's far side — it backs onto it at 120 mm/s (the goal datum, same press as the lap end; `goal datum -- the known wall behind is 2 cells back: pressing up to 310 mm`), moves its pose to the cell against that wall (`pressed 2 cells back: the home leg starts from (x,y)`) and the home leg compiles from there, launching from the setback with the 39 mm lead-in. This is what rescues the home leg from the goal stop falling short at the faster rungs (the wheels over-count the accelerations out of fast corners; the stop is dead-reckoned from the last corner). No known wall behind within 3 cells → dead reckoning, and it says so. A press that finds no wall, or a refused goal turn, now goes to the recovery window instead of asking for a power cycle. The goal stop keeps the velocity loop holding zero (no brake, no standby), settles 150 ms, turns at the search's 450/4000; the turn and the press are in the flight log (flags bit4 turn / bit5 press, `s` = translation since the stop, `obs_along`/`obs_cross` = left/right wheel mm/s), and the press distance is checked against the front beam read at the stop (`WARNING -- press stalled ... from where the front beam put the wall`). `route.stop` between the legs prints `STOPPED between legs` (no abort report). |
| `route.go` | Start a route run NOW, no hand wave (arms first if idle; same 700 ms settle + ~3 s IMU recalibration as a wave — **keep clear, it moves ~4 s after the command**). With the lap-end self-turn this gives hands-free repeated laps: lap → fetch log → tune → `route.go`. |
| `tick` / `tick=1` / `tick=0` | The idle 10 Hz status line (`tick=... \| yaw ... \| L R v \| C \| V`). OFF by default since 2026-09-16 — it buried the test data. `tick` toggles; the gyro-calibration "hold still" line at boot prints regardless, and the LEDs still heartbeat. |
| `mr` / `mr=N` | **The run ladder** (RunLadder.h, 2026-09-18). A competition is five runs and no laptop: MR1 is the search (at `sv`); every route run after it climbs a rung, the OUT leg's peak velocity coming from `rv2`/`rv3`/`rv4`/`rv5` (800/1000/1500/2000) by run number and the unscored HOME leg flying at `rvh` (500). `ralat` stays frozen so the corners are the same arcs on every rung. The next run's number lives in FRAM: reset to MR2 by every map save (a completed search or `mz.commit`), advanced **only when a run reaches the goal** (2026-09-30: the out leg stops on the goal, or a circuit through it completes — the console says `route: MRn reached the goal -- next wave is MRm`). A launch alone, an abort, a crash or a mouse carried off leaves the rung where it was: the next wave flies the same speed again. Power cycles never move it. Held at MR5. `mr` prints it; `mr=N` sets it (a false start you don't want to pay for); `mr=0` switches the ladder OFF and every leg flies `rv` as before — the rig-session mode. The rung and peaks go into the flight-log banner (`MR3 v1000/500`) and the lap report (`mr=3,v=1000`). A completed SEARCH also clears the goff table (maze property); a pushed map does not. **The V timing learner (2026-09-30):** every V's exit is walked back to the diagonal's centre line on dead reckoning (the `vout` offset the arc leaves is known), and the exit diagonal's first post then measures what the dead reckoning did not know -- the V's own timing. Flight prints `V exit (x,y): post .., dead reckoning .. -> .. mm OUTSIDE (late)`; the leg end folds it into the goff table under the V key (`goffc` lists it as `h=Ev` etc.) with `glearn`'s gain and slew, and the NEXT run fires that V earlier/later by it (clamped +/-20; the lever prints `learned`). MR2 learns, MR3-5 spend -- no maze-specific number from the laptop. `glearn=0` freezes it with the gates. |
| `lr` / `lr,N` / `lr.all` | Reprint a stored **lap report**: per-corner bookkeeping the mouse keeps for itself each leg — cell, commanded sweep + radius, sweep error, worst \|yaw_err\|, apex outer-beam minimum, exit yaw/lateral, gate outcome + correction, front correction, slip instrument. FRAM holds a RING of the last 8 (2026-09-14: the top 8 KB of the chip, slot = seq % 8), so a competition's MR2..MR5 all survive with no laptop between runs. `lr` = the latest, `lr,3` = three laps back, `lr.all` = every stored report oldest first (the GUI scoreboard can ingest a whole event from it). Survives power cycles; `lr` never stamps today's LPARAM on an old lap. |
| `route.dir=n\|e\|a` | Which way round a closed-circuit rig the lap runs — and therefore the **launch heading** (a launch cannot begin with a turn, so exit side = launch facing). `n` = launch north, west arm out (CAL-6: CW, right-handers); `e` = launch east, east arm out (CCW, left-handers); `a` = auto (legacy tie-break, flies like `n`). Setting it re-orients a mouse parked at the start cell **in place, by itself** (keep clear); hand-placed at the setback it prints which wall to back onto instead (`e`: back to the WEST wall). Falls back to the default direction with a console note if the map can't be flown that way round. Volatile — resets to `a` at power-on; the GUI re-sends its toggle. Bare `route.dir` reports. The direction flown is stamped in the flight-log banner (`dirn`/`dire`/`dira`). |

## Parameters — `name=value` (then `w` to persist, `p` to review)

**The registry rule (2026-08-27, Params.h):** a parameter is either compiled
or tunable, never both. Everything below lives in ONE table
(`PARAM_REGISTRY`), which also drives `p`, FRAM save/load, hard min/max
clamping at every door, and the `PARAM,...` snapshot stamped into every
flight log at arm — the log records what actually flew. Values set out of
range are clamped and say so. New parameters are APPENDED to the table
(never inserted or reordered — the FRAM block is values in table order);
appending needs no FRAM version bump.

| name | meaning |
|------|---------|
| `kp` / `kpt` | wheel P gain — straight / turn-stiff |
| `kd` / `ka` | wheel D / accel feedforward (**`ka` = 0 by policy** — see V2_BASELINE.md finding 7. Clamp caps it at 2e-5: the 2026-07-09 corruption value can never reach the ISR again) |
| `kx` / `kyaw` | position P (on fused distance) / heading-hold P |
| `strim` | straight-line differential trim |
| `track` | effective track width (mm) for the rotation feedforward |
| `tasym` | turn-asymmetry feedforward (CW-vs-CCW; half-weighted on rights) |
| `us` | arc-understeer feedforward — fractional boost on the path executor's rate differential, arcs only |
| `kslip` | slip-angle feedforward (deg per mm/s²) |
| `circl` / `circr` / `circ` | per-wheel measured circumference (mm) — odometry scale; `circ` sets both |
| `vmaxl` / `vmaxr` | per-wheel free-speed feedforward |
| `fricl` / `fricr` | per-wheel static friction feedforward |
| `vfilt` | wheel-velocity IIR alpha (0.2 ≈ 10 ms lag **inside the wheel loop** — part of the measured arc transport lag) |
| `dvmax` | heading-correction differential clamp, mm/s |
| `kyawi` / `kyawimax` | yaw-trim integral learn rate / coefficient cap |
| `rv` / `ralon` / `ralat` / `rslew` | route mission: cruise speed, longitudinal accel, lateral cap, rate rail. **rv and rslew are part of the path's geometry** — the leg recompiles at every arm, so changes take effect on the next wave. `rslew` below the U-turn geometry's floor is raised to it, loudly. |

### Corner-harness rungs — runtime only, not persisted

Every boot returns to the compiled defaults, so a rung being *tried* cannot
quietly become the value everything else is judged against.

| name | meaning |
|------|---------|
| `ky=` | where the mouse is put down: mm from the start cell's REAR boundary to the mouse reference point. **90 = cell centre (default), 51 = back against the rear wall.** Get this wrong and the 39 mm difference exceeds the gate's own ±35 mm window, so the gate skips every run. |
| `kv=` | harness cruise speed, mm/s (default 350) |
| `kalon=` | harness longitudinal accel, mm/s² (default 700) |
| `kalat=` | harness lateral budget, mm/s² (default 1000). Prints the R60 arc speed it implies. |

---

# Sensor sub-MCU (direct USB) — S15366 laser sensors

A **separate** command set, spoken to the sensor Teensy over **its own USB
cable at 115200 baud** (not the main MCU, not the radio). Use a terminal
or `tools/TW3_sensor_cal.py`.

Beam/LUT channel index for `L` and `W` = the transmit / `d`-stream order:
**0=Left, 1=Front-R, 2=Front-L, 3=Right**. The `d` stream prints the same
order: `left, fr, fl, right, loop_us`.

## Streaming / modes

| cmd | what it does |
|-----|--------------|
| `r` | Raw stream — 41 CSV fields (all per-channel S15366 registers ×4 + `loop_us`). |
| `d` | Distance stream — `left,fr,fl,right,loop_us` (mm; 255 = out of range / invalid). |
| `I` | Interlaced distance stream — the ~580 Hz mode (fire L,FR,R,FL, read one slot late). |
| `s` | Single shot — one raw line, then idle. |
| `q` | Quiet — stop streaming, lasers off. |

## Detection

| cmd | what it does |
|-----|--------------|
| `R` | Re-detect the three sensor ICs (after power-cycling a sensor board). |
| `F` | Force all ICs "present" (bypass detection). |
| `E` | Toggle **explore-all**. On (the compiled default; route-aware since 2026-10-02, see above) `e` maps every *reachable* wall before coming home; off it stops the moment a route to the goal is proven. Reachable matters: an enclosed 6×6 inside the 16×16 model leaves 220 cells unknown forever, and waiting for those would never terminate. |
| `h` or `?` | Help — also prints the current integration/gain/adc/area/threshold config. |

## Per-channel / per-IC config (IC = 0..2: right, front, left)

| cmd | meaning |
|-----|---------|
| `is<µs>` | Integration time, SIDE beams (1–2047; default 400). |
| `if<µs>` | Integration time, FRONT beams. |
| `L<ch><0/1>` | Enable/disable one laser channel (0=L,1=FR,2=FL,3=R). |
| `W<ch>,<px>,<mm>` | Set one distance-LUT entry (volatile RAM patch). |
| `g<ic> <v>` | Gain for IC. |
| `a<ic> <v>` | ADC reference for IC. |
| `c<ic> <v>` | Calc area for IC (0=5px, 1=9px, 2=19px). |
| `t<ic> <v>` | Peak threshold for IC (0–255). |
| `b<v>` / `x<v>` / `n<v>` | Blanking / readout start X / readout pixel count (global). |

*Sensor config lives in the sensor firmware, not FRAM — it resets on power
cycle unless baked into `TW3_sensor_sub_mcu.ino` defaults. Full LUT tables
are the exception: the main MCU stores swept tables in its FRAM and streams
them to the sub-MCU at every boot.*

---

## Typical workflows (main MCU)

- **First bring-up:** `m` (direction) → `s` [blocks] (vmax) → `c` (strim) → `t` (kpt/track) → `j` (per-wheel circ) → `w`.
- **Scale calibration:** `j`, measure physical travel, set `circl`/`circr`, `j` to verify, `w`.
- **Sensor calibration:** `u` (contact zero) → `C` (front sweep) → `W`; `S` (side sweep on the 2×4 rig) → `W`.
- **Diagnose a search:** run it with `D` on, or `y` afterwards, then open `tools/flight_viewer.html`.
- **Diagnose a sweep:** `sweep.dump`, save the text, and drop it on the same
  viewer. It detects the format and loads **one run per loop** — the ring
  closes, so every loop starts from the same physical place and they overlay
  directly. That overlay is the point: the ring is hand-built and uneven but
  repeatable, so what repeats across loops is the maze and what does not is
  the mouse. The commanded path is drawn dashed under the trail, the chart
  shows `ay` against `v x omega`, and the panel under the readout prints the
  arc-trim result — the mean `exit_lat` when the trim is on every corner, or
  the odd/even crossover when it alternates, with a standard error either way.
  It reads which from the dump header rather than assuming, because a
  crossover computed over an all-corners run returns a confident zero that
  means nothing.
