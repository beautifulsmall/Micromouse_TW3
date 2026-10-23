# test/

Host-side tests. No hardware, no PlatformIO — plain `g++`, a few seconds.

18 tests at the last count; `run_tests.sh` prints a pass/fail line each.

```
./run_tests.sh              # all of them
./run_tests.sh maze walls   # just these
```

Each test is a standalone `main()` that returns non-zero on failure and
prints what it measured. `run_tests.sh` holds the link line for each one; when
a test grows a dependency, add it there. In V1 these link lines lived only in
this README as copy-paste command lines, and three of them had silently drifted
out of date — that is why the script exists.

## What each test covers

| test | module(s) | what it proves |
|---|---|---|
| `fifo_decode` | `Drivers/ImuIcm45686` | ICM-45686 FIFO packet decode, header validation, unit conversion. Mocks the HAL. |
| `fusion` | `Control/Fusion` | Madgwick fusion: gyro bias calibration + motion guard, stationary hold, level and tilted 90° turns (tilt compensation vs naive `gz` integration), yaw wrap, wall re-referencing. |
| `profile` | `Control/Profile` | Trapezoid: full profile, short-move triangle, reverse moves, degenerate requests, and that `a_ref` integrates to `v_ref` integrates to the requested distance. |
| `path` | `Control/Path` | Straight/arc geometry: quarter-arc endpoints, segment joints agreeing from both sides, corner metadata, and the path-s vs spatial tangent-length gap that is the anchor trap. Planner: lateral cap, the sqrt braking curve, the end constraint, the accel ramp, and a forward simulation driven the way the ISR drives it. |
| `wall_gate` | `Control/WallGate` | Inside-wall edge detection: clean edge, early/late edges giving the right correction sign, a step onto the next wall, out-of-window and off-centre skips, a spurious dropout not firing, and firing at most once per arming. |
| `sweep` | `Application/LoopSweep` | The parameter grid covers every combination exactly once and in the intended order (speed fastest, slew rail switching once); record layouts are the asserted on-FRAM sizes; the FRAM regions do not overlap; the planned 50-loop session fits the trace region; and distance-based sampling costs the same at every speed where time-based would more than double. |
| `maze` | `Maze` | Edge-keyed wall storage, three-state edges + LOCK, optimistic/pessimistic flood, `best_step` tie-breaking, save/load round-trip. |
| `walls` | `Control/Walls` | Beam ranges → wall presence, front distance, lateral offset; out-of-range handling. |
| `wall_align` | `Control/WallAlign` | Corridor-axis fit from (distance, lateral) samples; outlier rejection. |
| `sensors` | `Drivers/Sensors` | Sub-MCU packet framing, the front-beam un-crossing, sequence/freshness. |
| `params` | `Application/Params` | FRAM block: CRC-32, version migration, defaults on a bad block. |
| `motors` | `Drivers/Motors` | Channel/sign mapping, stop and standby. |
| `odometry` | `Control` + drivers | Sign normalisation, counts→mm, velocity filter. Mocks the HAL, drives the ISR by hand. |
| `velocity_control` | `Control` | Wheel loop (feedforward + PD, duty clamp, enable/disable) against a first-order wheel plant. |
| `motion` | `Control` | Fused forward motion: grip tracking, wheelspin detection, IMU coasting during slip, recovery. Provides its own `imu_read_fifo`. |
| `move` | `Control` | Profiled move with an asymmetric plant (right wheel 5% weak): distance reached, heading held; the integral holds it even at `kyaw = 0`; with all heading authority off it visibly veers. |
| `turn` | `Control` | Closed-loop in-place turn: lands inside the finish band from either direction. |
| `straight_cal` | `Control` | The `'c'` update law (`strim += avg_dv/avg_v` over alternating ±legs) with an asymmetric plant and residual gyro bias. |

## Note on `test_move`

Its third case turns the heading-hold integral off with
`control_yaw_integral_enable(false)`. That switch was added in the V2 cut: the
integral (added 2026-07-16) holds heading even at `kyaw = 0`, which quietly
invalidated the original "kyaw = 0 visibly veers" contrast case. It had been
failing ever since and nobody was running the suite. See
`docs/V2_BASELINE.md`.
