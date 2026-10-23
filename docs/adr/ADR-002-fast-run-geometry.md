# ADR-002: Fast-run trajectory — constant-radius arcs, gated on wall edges

**Status:** Accepted
**Date:** 2026-08-18
**Deciders:** bert
**Supersedes:** nothing. TW3_V1's clothoid planner is not in this baseline
(see `../V2_BASELINE.md`); this is the fast run being started again from the
simplest thing that has actually flown.

## Context

TW3_V2 has a working search: it explores, maps, and saves a proven route to
FRAM. It has no fast run. The question is what the first one should be.

V1's history is the evidence available, and it is not a straight line:

- **Quarter arcs (constant radius), 2026-07.** Fixed radius, an angular slew
  rail, and a lead hack. Recurring wall strikes at zigzag corners until a
  closed-loop turn planner fixed the pure turns. This era ended at ~4.5 s on
  the test maze.
- **Wall-edge gates, 2026-07-16 onward.** The corner's inside wall ends at a
  known place; seeing that end anchors along-track position without a pose
  estimator. This is what made the arc era work at all, and it survived
  every later rewrite unchanged in principle.
- **Clothoids, 2026-07-20.** Curvature ramps linearly instead of stepping, so
  the reference is trackable and the entry transient goes away. This produced
  the 5/5 clean runs of 2026-08-01.
- **Everything after 2026-08-06** — diagonals, elastic bands, racing lines,
  three localization stacks — is what `V2_BASELINE.md` is about, and is out
  of scope here.

So constant-radius arcs are known to fly, and are known to have been replaced
for a specific, measured reason.

Three options were considered:

1. **Compiled continuous path of straights and constant-radius arcs**, one
   arclength coordinate, per-tick velocity planning from measured position.
2. **Chain the existing maneuver executor** so straight → arc → straight hand
   off without stopping.
3. **Arcs inside the search executor** — keep stop-and-sense, replace in-place
   turns with arcs.

## Decision

**Option 1.** A new `lib/Control/Path` compiles a route into segments of
constant curvature — `kappa == 0` is a straight, `kappa != 0` a circular arc —
on a single arclength coordinate `s`. A new path executor in `Control` treats
the fused forward distance as `s` and recomputes the entire velocity and
heading reference from it every 500 Hz tick. Corners are all one fixed radius
(default R60) for now. The inside-wall edge gate (`lib/Control/WallGate`)
corrects `s` from the end of the wall each corner turns around.

Bring-up is a single-corner bench harness (`k`), not a route.

### Why option 1

**Because every brake has to be feedback, not a clock.** This is the property
that makes gating possible at all. `sample()` takes the measured `s` and
re-solves every constraint from it: the pointwise lateral cap, a braking curve
to each upcoming arc, the zero-velocity constraint at the end, and the accel
ramp. Nothing is integrated forward, so when a gate moves `s` by 20 mm the
next tick simply produces the correct answer for the new position. There is no
schedule to resynchronise and no discontinuity to smooth over.

Option 2 fails exactly here. The maneuver executor is time-profiled: it samples
`TrapProfile` at elapsed time. An along-track correction has nowhere to land —
the profile has already decided when it will brake. Gating would have to be
bolted on as a distortion of the time base, which is the kind of second
mechanism that gets debugged for a fortnight.

Option 3 is not a fast run. Stopping at every decision is the cost the search
pays for certainty, and the whole point here is to stop paying it.

### Why constant radius rather than clothoids

> **RESOLVED 2026-08-21, runs 130–135. The measurement was taken and it says
> clothoids.** The decision criterion below was written before the data and
> the data answered it cleanly — see "The clothoid decision" at the end of
> this document. Clothoid transitions are now built into `Path` (opt-in per
> path, off by default) and enabled for the sweep. The reasoning below is
> preserved as written, because it is why the sequence was right.

Because the clothoid is a *fix for a measured problem*, and the measurement
has not been taken on this baseline. Adopting the fix first means carrying
its complexity — Fresnel series, tangent-length fitting, per-corner geometry,
the spatial-versus-path-`s` bookkeeping that produced V1's ~0.2·R anchor
bugs — without knowing whether this plant, at these speeds, needs it.

The honest sequence is: fly the simple geometry, measure the entry transient
one corner at a time, and let the number decide. `Path.h` records the
criterion and `k` is the instrument:

- a bias that **stays put** as speed rises is trim — fix it with `tasym` /
  `track` and keep this geometry;
- a bias or scatter that **grows** with speed is the curvature step itself,
  and that is when to make the reference curvature-continuous.

### Why one fixed radius

A per-corner budget fitter (each corner taking the largest radius its
straights afford) is worth real time — speed through a corner goes as √R. It
is also where the tangent-length bookkeeping lives, and therefore where the
anchor bugs live. One radius everywhere makes the gate arithmetic a single
subtraction and makes every corner directly comparable to every other. The
fitter is a later increment, added once the corner itself is trusted.

### Why the bench harness first

A route measures the *sum* of its corners. V1 spent two weeks reading four
corners' accumulated error as one number, and the harness is what finally
separated them — it is how the CW/CCW asymmetry that `tasym` now corrects was
found. Building the route compiler before one corner is proven would repeat
that mistake with better documentation.

## Consequences

**The curvature step is real and is mitigated, not removed.** Constant-radius
corners hand the executor a step in commanded angular rate at every tangent
point. `PATH_OMEGA_SLEW_DPS2` (4000 deg/s², ~59 ms / ~24 mm to the R60 arc
rate at 400 mm/s) bounds how fast the command may change. This converts an
untrackable step into a bounded, repeatable entry transient — the mouse still
enters slightly late and runs slightly wide, and that error is what `k`
measures. Repeatability is the useful property, not smallness.

**Gate corrections are drained, never stepped.** `s` feeds the braking curves,
so a step in `s` is a step in commanded speed and — through `omega = v·kappa` —
in commanded rate. V1 applied a raw −53 mm edge correction into an approach
and got a +12 mm entry transient and a crash (2026-07-26). `control_path_correct`
clamps to ±40 mm and drains at 0.25 mm per tick, so a 25 mm correction lands
over ~200 ms, inside the ~100 mm of approach a gate typically has left.

**Silent or certain.** A gate whose implied correction falls outside its
plausibility window, or which fires while the mouse is too far off the
corridor centreline, is dropped entirely. The corner then flies on odometry,
which is what it did before gates existed. A wrong correction is worse than
none — and a skip does not consume the gate, so a later clean approach can
still anchor.

**Speed limits start below V1's finish.** `PATH_V_MAX_DEFAULT` is 400 mm/s
against the 650 V1 ended at, and `a_lat` 1000 against 2000. The trajectory
generator underneath those numbers is different, so they are not inherited —
they are walked back up by hand on the harness with `kv=` / `kalat=`.

**No impact latch.** V1's accelerometer-based wall-contact detector went with
the speed run, and its thresholds were never reliably calibrated (measured
post strikes were 1.98–2.02 g against a 3.0 g hard trip that fired on none of
five logged strikes). The harness has a front-beam panic guard instead. An
impact latch should come back before a route runs unattended at speed, and
should be sized from logged strikes rather than from a guess.

**A second executor now exists in `Control`.** `move_tick` and `path_tick` are
mutually exclusive by their start guards, and `control_move_abort` stops
either. Two executors is one more than ideal; the alternative was one executor
serving two incompatible parameterisations (time and arclength), which is
worse.

**Host-testable.** `Path` and `WallGate` are pure logic with no HAL, and both
have test files. The forward-simulation test drives the planner the way the
ISR does and checks the lateral budget is respected through the arc, so the
speed schedule is verified before it ever reaches a motor.

## The clothoid decision — resolved by runs 130–135

The criterion above was *"a bias that stays put as speed rises is trim; a bias
or scatter that grows with speed is the curvature step."* The loop rig
answered it on 250+ corners, and the answer was not the one the sequence
expected.

**The instrument had to be fixed first.** `arc_turn` spans only the `in_arc`
window, and the overshoot happens after the arc closes — so for four runs the
summary column said the mouse *under*-turned while it was visibly cutting in.
Measured straight-to-straight with yaw unwrapped, total heading is **over** 90°
everywhere, growing as the turn tightens, and it decays back to zero within
one cell. A transient, not a steady-state error.

**It has the closed form of a rate ramp.** When the arc ends, commanded
curvature steps to zero and the rail bounds how fast ω can fall, so the mouse
keeps rotating by `ω²/2α`:

| cell | measured | ω²/2α |
|---|---|---|
| R125 v500 slew2500 | 5.33° | 10.50 |
| R100 v500 slew2500 | 7.61° | 16.41 |
| R80 v500 slew6000 | 6.18° | 10.69 |
| R60 v500 slew6000 | 13.69° | 19.00 |

Consistently ~60 % of the open-loop form — the yaw loop absorbs the rest —
and the same shape in every cell and both directions.

**The rail A/B is the deciding test, and it was specified in advance.**

| | slew 2500 | slew 6000 |
|---|---|---|
| R125 v500 | 5.33° | 0.84° |
| R100 v500 | 7.61° | 3.04° |
| R80 v500 | 11.86° | 6.18° |

Halve the ramp, halve the overshoot. **The error moves with the rail**, which
by the criterion above is the curvature step and not grip.

**Both alternatives were falsified on the floor, not argued away.**

- *Grip.* `|ay|/a_dem` never saturates; it reads 1.085 at the very cell that
  crashed, and it differs by **turn direction** at identical demand — which
  grip cannot do. (An earlier reading of that column as a "tyre cliff at
  0.61 g" was wrong: the ratio is `R_commanded/R_actual`, a curvature meter.)
- *Feedforward magnitude.* `track_mm × 0.90` was flown twice — runs 134 and
  135, same firmware, a clean repeat — and moved the paired overshoot by
  **+0.31°**, slightly the wrong way, with 8 of 12 cells up. `kyaw` closes on
  heading error, so cutting the feedforward just makes feedback contribute
  more: the *split* moves, the trajectory does not.

**What was implemented.** `PathSeg` gained an end curvature, so one struct
covers straight / arc / clothoid as a linear curvature ramp; `pose()`
integrates ramp segments with Simpson over `θ(u)`; `add_arc` emits
clothoid→arc→clothoid. Off by default, so a path that does not ask for
transitions is bit-identical to the geometry that flew runs 116–135.

**The constraint that shaped the design, and it was not anticipated.** A
clothoid corner consumes `T = R + m + x₀` of corridor instead of `R`, and the
gate must fire before the tangent point. The inside wall ends 90 mm before the
corner point and the beam leads ~45 mm, so `T ≲ 135`. A pure arc already has
`T = R`, leaving `135 − R` of room: **10 mm at R125, 55 at R80, 75 at R60.**
The clothoid budget therefore *grows as the radius tightens* — exactly where
the overshoot is worst. R125 gets no transition and needs none (0.84° at slew
6000); R60 gets the full rail-sized ramp with 54 mm to spare.

This is also why `gate_expect_s` must read `corner_tangent_mm()` rather than
assuming `R`. Assuming `R` would put every gate expectation `T − R` too early
— the same class of error that cost runs 120–135.

**Sizing policy:** `L = min(v²/(R·α), room)`. The rail term is the shortest
transition the executor can already follow; shorter and the ramp stays
rate-limited, so the geometry gains nothing.

**The prediction to falsify:** clothoids should collapse the slew-2500 column
onto the slew-6000 column. If the reference never asks for a step, the rail
stops mattering, and the −7 mm exit bias that only slew 2500 shows should go
with it. Run 135 is the baseline — first complete grid, 52 corners, zero guard
trips, 100 % gate lock, mean |exit_lat| 6.4 mm.

## Action items

1. [x] ~~Fly `k` both directions at R60/80/100 on the bench rig.~~ Superseded:
       the loop rig measured it at scale instead (250+ corners, both
       directions, 13 cells) — see above.
2. [x] ~~Decide clothoids-or-trim from whether the error scales with speed.~~
       **Decided: clothoids.** The rail A/B was decisive and both trim
       candidates were falsified.
3. [ ] Route compiler: proven route → `SpeedPath`, with the per-corner gate
       anchors derived from the map. Only after (1) and (2).
4. [ ] Corner front gate and finish gate — the other two anchors V1 used —
       once the edge gate is trusted on hardware.
5. [ ] Re-size and reinstate an impact latch before any unattended route run.
