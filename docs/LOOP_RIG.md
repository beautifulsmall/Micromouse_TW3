# The loop rig — an autonomous corner experiment

A closed ring of corridor that the mouse drives round for minutes at a time,
changing one corner parameter per lap and logging every corner to FRAM. It
exists to answer one question: **at what combination of radius and speed does
the mouse come out of a corner centred?**

Firmware: `lib/Application/LoopSweep`. Design reasoning:
`adr/ADR-002-fast-run-geometry.md`.

---

## Build it

A one-cell-wide corridor round the perimeter of a **6×6** maze: 20 cells, four
90° corners, corner centres five cells (900 mm) apart, four cells of straight
between arcs. **Walls on both sides all the way round** — the outer boundary
plus a walled 4×4 block in the middle.

```
        +---+---+---+---+---+---+
    y=5 |   →   →   →   →   →   ↓|      ← corner cells are the four corners
        +   +---+---+---+---+   +
    y=4 | ↑ |███████████████| ↓ |
        +   +               +   +
    y=3 | ↑ |███ inner ████| ↓ |       the 4x4 block only needs its
        +   +               +   +       PERIMETER walled -- nothing goes
    y=2 | ↑ |███ block ████| ↓ |       inside it
        +   +               +   +
    y=1 | ↑ |███████████████| ↓ |
        +   +---+---+---+---+   +
    y=0 | ↑   ←   ←   ←   ←   ← |
        +---+---+---+---+---+---+
         x=0  1   2   3   4   5
```

Wall count: 24 boundary segments + 16 around the inner block = **40 pieces**.

**Why walls on both sides matters.** Three things depend on it:

1. `lateral_valid` is true every packet, so the exit-lateral measurement —
   the actual objective — is never missing.
2. Both walls give the *averaged* lateral, which is immune to corridor-width
   calibration error; one wall alone leans on `WALL_CORRIDOR_HALF_MM`.
3. The lateral guard (`|lateral| > 40 mm for 2 packets → abort`) is available
   continuously,
   pose-free and map-free. On a rig that runs unattended for minutes, that is
   the difference between a stopped experiment and a broken mouse.

**Where the wall edge is.** The inside wall of each corner runs continuously
along the side and **ends at the corner cell's near boundary**, because the
corridor turns there. That end is the landmark the gate anchors on. Past it
the beam looks down a long open corridor and reads 255 — a clean non-return,
confirmed in 3 samples.

**Placing the mouse.** Anywhere on a side, two cells before a corner, facing
along the corridor. Clockwise (the default) means the inner block is on the
mouse's **right** and every corner is a right turn.

---

## Run it

**Flash, place, wave. Nothing typed.** The whole session is compiled in —
see the block at the top of `lib/Application/LoopSweep.h`:

```c++
constexpr bool SWEEP_ARM_AT_BOOT      = true;      // the sweep owns the wave
constexpr bool SWEEP_DIR_LEFT_DEFAULT = false;     // v15: back to R for the tasym pair
constexpr float    SWEEP_TASYM_OVERRIDE = 0.0f;    // v15: null the asymmetry ff, RAM only
constexpr float    SWEEP_TRACK_SCALE = 1.0f;       // v16 x0.9 FALSIFIED, reverted
constexpr bool     SWEEP_CLOTHOID = true;          // v17: clothoid transitions
constexpr uint8_t  SWEEP_DEF_R[]      = {125, 100, 80, 60, 45};
constexpr uint16_t SWEEP_DEF_V[]      = {500, 600};
constexpr uint16_t SWEEP_DEF_SLEW[]   = {2500, 6000};
constexpr float    SWEEP_DEF_TRACE_MM = 4.0f;
constexpr float    SWEEP_ARC_TRIM_MM  = -2.0f;     // moves WITH the look-ahead, see v14
constexpr float    SWEEP_MAX_RAMP_FRAC = 0.75f;    // decline what the rail cannot deliver
```

20 combos (7 of them declined by the ramp rule, so 13 flown) ≈ 2 minutes of
driving, ~110 KB of traces (including breadcrumbs) and 2 KB of summaries. The mouse writes those numbers into the log header, so the data
always says which firmware produced it.

**Radius varies fastest**, then speed, then the slew rail. It was
speed-fastest for runs 116–119 and that buried the axis that mattered: no
session has yet survived past loop 8 of 50 — a guard trip needs a human to
pick the mouse up and wave — so R80, R60 and R45 have **never been flown**.
Now the first five loops of any battery sweep every radius, which is what the
open question needs, because arc length runs 196 mm at R125 down to 71 mm at
R45.

**The speed axis starts at 500 on purpose.** It has been cut from the bottom
twice, both times because the slow end resolved nothing. Runs 116–119 swept
200–400 and never reached a speed the mouse struggled at; runs 120–129 swept
200–600 and the answer came back the other way round — the fast cells were the
clean ones, and run 129 spent an entire session in the v200 row before ending
in it. Run 128's mean `exit_lat` is monotonic in v at every radius and crosses
zero near 400. R45/v600 asks 8000 mm/s² — 0.82 g — and will almost certainly
trip a guard; that trip is the measurement.

The second reason is the better one. `SWEEP_ARC_TRIM_MM` corrects a placement
error that scales with the distance covered during the gate's detection lag,
so it scales with `v`, and **one constant can only be right at one speed**.
Narrowing the axis to 500–600 shrinks the span it must cover from ~40 mm of
required trim to ~10 mm, which is inside the run-to-run noise. One thing to
watch when reading the log: the straight speed is `max(v_arc, 400)`, so the
approach now always runs at arc speed and the edge gate gets ~6 mm per packet
rather than 4. Its 2–3 sample confirmation lags ~15–18 mm everywhere instead
of varying down the column — that uniformity is the point.

The console commands in `COMMANDS.md` still work when you are tethered, but
nothing requires them, and **every boot returns to exactly what is compiled
above**. A session that depends on six typed settings is a session that
silently runs with yesterday's typo in it.

### At the maze

Power on. The status LEDs tell you what a wave will do — this is the only UI
out there:

| LEDs | meaning | what a wave does |
|---|---|---|
| slow alternate | fresh session, or a part-done one with this same grid | starts / continues it |
| **fast, both together** | the log holds a set this firmware did not produce, or a finished set | **nothing** — hold the hand 5 s to discard and start fresh |

So: wave → it calibrates the gyro (~3 s, keep it still) and sets off.

Battery flat halfway? Swap it, power on, wave. Grid position lives in FRAM
and it resumes at the combo it reached.

### Getting the data off

```
sweep.dump
```

Tether USB, open a terminal (the firmware only writes to USB when DTR is
asserted, i.e. a terminal is actually open), and type it. Typing anything
stands the hand start down first, so the mouse will not set off while you are
at the keyboard; `sweep.arm` re-arms it.

Output is CSV in three parts:

```
# sweep session 3  dir=R  corners=200  traces=421888 B  ay_zero=+0 mm/s2  \
#   arc_trim=+25 mm on ALL corners  combo_ix=50  loops_done=50
# gains kp=... kpt=... kyaw=... track=... tasym=... strim=... circl=... circr=...
S,loop,corner,dir,R,v_arc,slew_ix,entry_lat,exit_lat,entry_yaw_err,...
S,0,0,R,45,200,0,-1.2,3.4,...
...
X,loop,dir,R,v_arc,slew_ix,at_s,to_corner,lat,lat_prev,lat_src,yaw_err,left,right,...
X,7,R,80,400,0,2452,3,-44.6,-38.2,one,0.31,255,39,9840,1.2,lateral
...
T,off,s,v,yaw,yaw_err,omega,ay,a_dem,left,right,in_arc,corner
T,0,318,398,0.02,-0.11,0,-14,0,84,83,0,0
...
```

The `#` lines record which gains were in force when the session started —
the sweep is meaningless without them, so they travel with the data rather
than in your notebook. `S,` rows are the experiment (one per corner); `T,`
rows are the diagnostic detail, and `trace_off` on each `S,` row points at
where that corner's `T,` rows begin. `X,` rows say why a loop stopped — one per failed loop, whatever it was
doing at the time — and `T,` rows with `corner = 255` are the sparse
corridor breadcrumbs that show where it got to.

Roughly 22,000 lines / 1.3 MB for a full session: seconds over USB, about
half a minute over the radio if it is fitted.

### The second direction

**One ring gives one turn direction**, and the CW/CCW asymmetry that `tasym`
corrects is first-order — so a complete experiment is this session flown
twice. Flip `SWEEP_DIR_LEFT_DEFAULT` to `true`, reflash, and the mouse will
see a stored set from a different grid and **refuse to overwrite it**: fast
both-LED blink, and a wave does nothing until you have dumped the first set
and held the hand for 5 s. The data cannot be lost by accident.

### If you want to run a SEARCH instead

`SWEEP_ARM_AT_BOOT = true` means the sweep owns the hand start and the search
no longer arms at power-on. Type `h` to arm the search for one run, or set
the constant to `false` and reflash. Only one mission may own the wave —
otherwise a wave is ambiguous, and both of these are multi-minute autonomous
runs.

---

## Read it

Two exit signatures, and they mean different things:

| exit yaw error | exit lateral | diagnosis | lever |
|---|---|---|---|
| ≠ 0 | ≈ 0 | the rotation did not complete | `track`, `tasym` |
| ≈ 0 | ≠ 0 | the path was displaced | **this is the R/v matching** |

The zero-crossing of exit lateral against speed, for each radius, is the
answer to the original question.

**And then the slew axis tells you why.** Exit displacement has two causes
and both scale as v²/R — understeer (grip) and the curvature step at the
tangent point (reference trackability). They are collinear in the (R, v)
plane, so no amount of sweeping radius and speed can separate them. The slew
rail divides the second and leaves the first alone:

- exit error **moves** between slew 2500 and 6000 → the curvature step is
  dominant → **clothoids will fix it**, and ADR-002's open question is
  answered;
- exit error **does not move** → it is grip → clothoids will not help, and
  the lever is slide compensation or slower corners.

That is the single most valuable thing this rig measures, and it costs one
extra factor of two in loops.

Scatter across the four corners of a loop is your noise estimate — free,
because the four corners of a lap are four repeats of the same combo.

---

## What gets logged, and why it fits

**Summaries — 36 B per corner, and these ARE the experiment.** Written the
instant the corner completes, so a pulled battery loses at most the corner in
progress. 72 KB reserved = 2048 corners = 512 loops; they never wrap.

```
loop, corner, dir, R, v_arc, slew_ix,
entry_lat, exit_lat, entry_yaw_err, exit_yaw_err,
gate_corr, gate_drop_s, arc_min_left, arc_min_right,
corner_ms, wheel_minus_s, flags, trace_off, arc_turn
```

`arc_turn` is the heading actually turned between the two tangent points.
The command is always 90.00. Anything else is an effective radius that is not
the commanded one, and it is on *every* corner rather than only the traced
ones because it is the number that separates "right arc, something else
pushes me wide" from "the mouse does not turn as far as it is told".

**Traces — 16 B per record, one every 4 mm of path, in the corner window
only** (the corner cell and one cell either side). ~128 records per corner,
2 KB; a 50-loop session is ~400 KB and fits the trace region without wrapping.

```
off, s, v, yaw, yaw_err, omega, ay, a_dem, left, right, in_arc, corner
```

`ay` is the **lateral specific force**, body +Y, mm/s², filtered, gravity
removed, minus the stationary zero taken during the pre-loop IMU calibration
(printed in the dump header as `ay_zero`). `a_dem` is `v × omega` — what the
tyres have to supply to hold the commanded arc — computed at dump time from
the two columns beside it.

**These two columns are the whole reason the record changed.** A broadside
skid turns no wheel and changes no heading, so the encoders and the gyro are
both blind to it; the accelerometer is the only instrument on the mouse that
can see one. `ay ≈ a_dem` is a mouse going round. `|ay| < |a_dem|` is a mouse
going straighter than it was told, and the deficit integrated twice is how far
it slid. `arc_ay` on each summary row is the same measurement averaged over
the steady part of the arc, so it exists for every corner rather than only
the traced ones.

The record stayed 16 B: `ay` took `lat_x10`'s place, because `lat` was the one
field fully recoverable from what is left — it is `(right − left) / 2` with
both beams valid and `84 − left` / `right − 84` with one, and both raw beams
are still in the record.

**The traces are sampled on DISTANCE, not time, and that is the whole reason
the budget works.** At a fixed 616 Hz the same window costs 16 KB at
200 mm/s and 7 KB at 450 mm/s — so in a *speed* sweep the slow combos eat the
FRAM, which is exactly backwards. Sampling per millimetre makes every combo
cost the same and gives uniform spatial resolution, which is what geometry
work wants anyway. (Third time this project has met the same lesson: the
lateral servo, the gate-correction drain, and now the logger all want a
length constant, not a time constant.)

`sweep` prints the projected usage before you start, so an over-ambitious
grid tells you at configuration time rather than by silently wrapping.

---

## Field notes

### Run 116 — first hardware outing, 2026-08-19

Stopped after one corner, mid-corridor, no collision. **The lateral guard
fired at the corner exit and it was right to** — the mouse came out of an R45
corner with the outside wall 43 mm from its centre, i.e. about 8 mm from the
body. What the log then exposed:

**The grid opened on its own worst cell.** R45 / v200 / slew 2500 spends
**29 % of the whole arc** just ramping the commanded rate up to v/R (255 deg/s
at 2500 deg/s² takes 102 ms = 20 mm of a 71 mm arc). At R45/v300 it is 65 % —
not a corner at all, a continuous ramp. Radii now run **largest first**, and
`sweep` prints the ramp-fraction table so the ugly cells are visible before
you fly.

**A guard trip ended the whole session.** One trip on combo 0 cost the other
49. A guard trip is a *result* — "this combination cannot hold the corridor"
is what the sweep exists to find out — so it now ends the **loop**, records
the combo with a `fail` reason, advances the grid, and re-arms. Put the mouse
back on a straight and wave. Three failures in a row still stops for good:
that is systemic, not a property of the grid.

**The exit lateral was being measured inside the junction.** The old window
opened 20 mm after the exit tangent — where the mouse is still in the corner
*cell*, with the inside beam looking straight down the far corridor. Run 116
read left 43 / right 147 there: a sum of 190 against a 168 mm corridor, so the
two beams were not a pair, and the averaged "lateral" of 52 mm was part
artefact. Now: a **corridor-sum check** (reject the pair, use the nearer wall,
and record which in `lat_src`), and the exit window moved past the junction.

**In the junction the corridor model does not apply at all**, so the
model-free guard took over there: a raw side beam under 30 mm for three
packets. Run 116 saw a legitimate 40 mm mid-arc with the body clear — a beam
sweeping a concave corner at a shallow angle — so 30 sits below anything the
geometry produces.

**Two logging bugs, both silent.** Entry and exit yaw error came out identical
(−4.67) with `corner_ms = 0`, because the latches re-armed after the arc and
re-stamped themselves every poll. And an aborted corner stored `entry_lat` and
`exit_lat` as 0.0 — the abort path wrote the record without computing the
derived fields, so the two numbers the experiment is about went missing
exactly when something interesting happened. Both fixed; `SWEEP_VERSION` is
now 2 so an old header is rejected rather than misread.

**Still open.** Heading tracking through that corner was good — peak lag
12.3°, 5.2° at the exit — and integrating the lag accounts for only about
**8 mm** of the ~43 mm displacement. So either the mouse really did slide, or
the beam reading in the junction was still not telling the truth. The next
run's data will say, because the exit lateral is now sampled somewhere the
wall model means something. Do not tune anything on the strength of one
corner.

### Run 117 — eight corners, and the first real measurement, 2026-08-20

Two loops' worth of geometry, then a stop. Nothing hit anything.

**Loop 0 flew perfectly and was still wrong by 20 mm.** All four R125 / v200
corners came out `ok`, `lat_src=both`, exit yaw error under 1.4°. And all four
exited **+22 to +29 mm toward the outside wall**, having entered at +2 to +8.
Yaw ≈ 0 with lateral ≠ 0 is the signature of a *displaced* path, not a
mistracked one, and at R125/v200 the lateral demand is 320 mm/s² and the slew
ramp is 4 % of the arc — so it is neither grip nor the curvature step. The
lateral servo pulls it back over the following 650 mm of straight, which is
why it repeats instead of accumulating. **This is the number the rig was built
to produce.** What it is not yet is explained:

| candidate | evidence | accounts for |
|---|---|---|
| in-arc heading lag | mean 0.75–2.5°, integrated along the arc | 3–9 mm |
| effective radius too large | arc delivered **84.9–87.3°**, not 90 — R_eff 126–130 mm against 125 commanded | ~3 mm |
| exit heading, junction, servo suppressed | leaves the arc 3–5° short, runs 60 mm before the window | ~4 mm |
| | | **~12 of ~20 mm** |

So roughly half is understood. `arc_turn` is now logged on every corner so the
radius half of it stops needing a trace to see.

**The corridor is not 168 mm.** Left + right summed to **175–182** everywhere
— on the entry straight, through the arc, and in the exit window. A
*differential* lateral is immune to a common offset, so this does not bias the
20 mm above, but `SWEEP_CORRIDOR_SUM_TOL_MM = 20` is currently spending half
its budget on a known bias, leaving only ~10 mm to catch a genuinely
mismatched pair. Measure the rig with a rule before changing
`SWEEP_CORRIDOR_MM`: it is either a wide corridor or two beams reading ~5 mm
long, and those want different fixes.

**Why it stopped: `sweep_arm()` refuses unless the state is idle.** Its first
line is `if (state != W_IDLE && state != W_DONE) return;`, and `loop_failed()`
never left `W_FLY`. So the re-arm after a guard trip silently did nothing and
the mouse stayed in the flying state holding a dead path. Every poll after
that re-entered the flying body, found `corner_ix` still on the failed corner
with `s_mm` frozen past its window, opened a fresh record, latched entry and
exit in the same tick, wrote the empty record, then hit `!ps.done` and failed
the loop *again* — one combo burned per tick until the three-in-a-row streak
stopped the session. One real lateral trip on combo 1 therefore ate combos 2
and 3 and left `S,2,1` and `S,3,1` in the log with nothing in them.
`loop_failed()` now sets `W_IDLE` itself, and a record cannot be opened
against an inactive path.

**`SWEEP_F_SLIP` meant nothing in runs 116 and 117.** `slip0` was read
*before* `control_motion_zero()` cleared the counter, so the baseline and the
counter could never agree and the flag came out set on every corner of every
loop. Zero first, then baseline — and baseline per corner, since a slip in
corner 0 is not evidence about corner 3.

`SWEEP_VERSION` is now 3 (the record grew); an old log reads as NEEDS_WIPE.

**Next.** Fly the same grid with `dir = L`. If the offset flips to the other
physical wall it is geometry — radius, heading lag, arc timing — and
symmetric. If it stays on the same wall it is a steering or sensor trim bias.
That one run separates the two families, and nothing should be tuned before
it.

### Run 118 — the contradiction, 2026-08-20

Three loops, ten corners, all three fixes confirmed: no phantom records, the
`SLIP` flag now varies instead of being always set, `arc_turn` logged on every
corner. It stopped at loop 2 corner 1 on the lateral guard and **the guard was
right again** — the left beam read 41 mm in the new corridor, and the body is
35 mm half-width with contact at 49. Then it re-armed and stood waiting for a
wave, which is the intended behaviour and looks exactly like a fault.

**The offset does not depend on speed.** Exit lateral by corner, across
v200 / v250 / v300:

| corner | v200 | v250 | v300 |
|---|---|---|---|
| 0 | 28.5 | 27.2 | 29.3 |
| 1 | 32.0 | 31.5 | (guard) |
| 2 | 24.3 | 23.6 | — |
| 3 | 29.9 | 21.7 | — |

Lateral demand rises 2.25× from v200 to v300 and the number does not move.
**That kills grip.** It is a geometric or metrological constant, and a
constant can be found standing still.

**The mouse's own dead reckoning says the corner was fine.** Integrating the
logged heading over the logged distance from the entry tangent, against a
perfect arc of the commanded radius, predicts the lateral change through each
corner as:

| | predicted | measured |
|---|---|---|
| L0C0 | +2.8 | +23.5 |
| L0C1 | +2.3 | +27.2 |
| L1C0 | +6.6 | +23.7 |
| L1C1 | +1.1 | +38.6 |
| L1C2 | −0.5 | +25.8 |
| L2C0 | +3.4 | +34.8 |

Encoders and gyro: the arc was the right shape, within about 5 mm. Walls:
25–39 mm. **Those cannot both be true**, and that is a far more useful
statement than the half-explanation run 117 produced.

Three ways out, and only one survives:

1. *It slid.* A broadside skid is invisible to both encoders and gyro, and
   produces exactly this signature. But it would scale with v²/R, and the
   table above is flat. Dead.
2. *The arc was placed wrong along the corridor.* Also invisible to dead
   reckoning — s is measured from the launch point, not from the wall — and
   starting an arc d late puts the exit d to the outside. But the gate's
   arithmetic checks out where it can be checked: corner 1's tangent sits at
   s = 1081, its cell boundary at 1116, and with a 72 mm beam look-ahead the
   drop is predicted at 1044. Measured: 1045, 1042, 1036. The gate is anchored
   correctly at corner 1, and corner 1 is the *worst* corner. Mostly dead.
3. **The lateral measurement is not taken against a fixed ruler.** Left + right
   sums to 165–179 in the entry windows and 176–182 in the exit windows. A
   corridor is one width everywhere. Both beams agree with each other in each
   window — in the exit window L ≈ 61 and R ≈ 117, and against a half-width of
   89 both independently say "28 mm left" — so it is not one bad beam, it is
   the pair reading a different geometry in the two places.

**So stop flying.** `sweep.probe` stands the mouse still and prints both raw
side beams, their sum, and the wall model's lateral. Set it down by hand,
centred, and take a reading in three places: mid-straight, in the entry window
before a corner, and in the exit window after one. A centred mouse reads
84 / 84 / 168. If it reads 60 / 117 in the exit window then three runs have
been measuring an artefact; if it reads 84 / 84 there, the mouse really was
28 mm off and something moved it that neither the encoders nor the gyro saw.
Either answer closes the question. Bring a rule and measure the rig's actual
corridor width at the same three places while you are there.

### Run 119 — the same grid, with the lateral accelerometer, 2026-08-20

bert watched the mouse and it visibly moved out on the turns. That settles it:
it slid, and the analysis after run 118 argued slip was dead because the exit
offset is flat with speed. **An eye beats an inference.** Firmware measures
what it is told to measure, and nothing on this mouse was measuring the
direction it was moving in.

So run 119 is run 118 again — same grid, same direction, nothing else changed
— with body **+Y acceleration logged**, and one number to look at:

    a_dem = v × omega   (what the arc demands)
    ay                  (what the tyres actually delivered)

If `ay` tracks `a_dem`, it did not slide and the offset is metrological after
all. If `ay` falls short, the shortfall integrated twice is the distance it
slid, and it should come out around 25 mm.

What "flat with speed" then has to mean, if the tyres are slipping: a
Coulomb-limited breakaway would scale hard with v²/R and does not fit, but a
**constant slip angle** does — lateral displacement is arc length × tan β, and
arc length is fixed by R, not by v. That points at a steady scrub rather than
a breakaway, which is a different fix (weight distribution, tyre compound,
wheelbase) and a different reason the exit is always outside.

**Flashing this wipes the run-118 log.** `SWEEP_VERSION` is now 4 — the trace's
`lat_x10` became `ay_mm_s2` and the summary's spare word became `arc_ay` —
so the stored header is rejected at boot and a fresh session starts on the
first wave. Run 118 is already dumped; nothing else is at risk.

### Run 119 result — the accelerometer says the tyres held, 2026-08-20

31 corners, 8 loops, one lateral guard trip at R100/v300. `ay_zero` came out
at 0 mm/s², so the Y bias is negligible and the numbers are readable as-is.

**The tyres deliver the cornering force.** `arc_ay` against `a_dem = v²/R`,
averaged over the four corners of each loop:

| R | v | \|ay\|/a_dem | R from ay | R commanded |
|---|---|---|---|---|
| 125 | 200 | 1.16 | 124 | 125 |
| 125 | 250 | 1.04 | 125 | 125 |
| 125 | 300 | 1.04 | 124 | 125 |
| 125 | 350 | 1.02 | 124 | 125 |
| 125 | 400 | 1.10 | 114 | 125 |
| 100 | 200 | 1.21 | 84 | 100 |
| 100 | 250 | 1.03 | 103 | 100 |
| 100 | 300 | 0.96 | 105 | 100 |

Per-corner scatter is huge — the accelerometer sits in 700–1400 mm/s² of
gearbox and motor vibration — so single corners mean nothing and the
four-corner means mean a lot. There is no deficit anywhere, including the
highest demand flown (894 mm/s² at R100/v300).

**And there is no crab velocity either.** `w = ∫(ay − v·ω)dt` through the arc
is the lateral velocity the body picks up relative to a coordinated turn.
Producing 25 mm of slide over a ~1 s arc needs a sustained w of about
50 mm/s. Measured, by speed: −26, −15, +2, +14, −48 mm/s, standard errors
34, 34, 29, 14, 17. Straddling zero, and short of what is needed.

**The arc also starts in the right place.** Measured from the trace itself —
the last valid inside-beam sample before the arc, to the first `in_arc`
sample — against the geometric ideal of `162 − R`:

- R125: **−0.8 mm** late (n = 20, sd 7.2)
- R100: **+6.3 mm** late (n = 11, sd 8.2)

**So four independent checks now agree the corner was flown correctly**, and
the walls still measure the exit 25 mm outside (mean measured +25.0 mm against
+(−1.3) mm predicted from heading over distance, n = 30):

| | says |
|---|---|
| encoders + gyro, integrated | the arc was the right shape |
| accelerometer, force | the tyres made the full lateral demand |
| accelerometer, ∫(ay − v·ω) | no outward crab above the noise |
| trace, wall-end to arc-start | the arc began where it should |
| **walls, and bert's eye** | **25 mm outside** |

Three inertial instruments cannot all be blind to the same motion. What they
*can* all be blind to is a disagreement between the mouse's frame and the
maze's — which is what is left.

**A caution about the eye.** "It visibly moved out on the turns" and "it slid"
are different claims. The eye sees POSITION, and on position the eye and the
walls agree completely. The accelerometer tested SLIP, and said no. Both
observations are sound; they are answers to different questions.

**A design fault in the grid, found while analysing this.** Combo order is
speed-fastest, so within a radius block **loop index is a proxy for speed** —
and `entry_lat` drifted from +5 mm in loop 0 to −12 mm in loop 6. Any quantity
compared across loops is confounded with that drift. Run 119's apparent
"exit_lat falls with speed" is partly the drift. Comparisons must be made
**within** a loop from here on — and the grid has been reordered to radius-
fastest so the axis read first is no longer the one aliased onto time.

### Next: the arc-placement probe

`SWEEP_ARC_TRIM_MM` (default −20 mm) adds a fixed along-path offset to the arc
start, on top of the gate's correction and spread over the same room, when
**`(loop_ix + corner_ix)` is odd**.

Starting an arc *d* mm late puts the exit *d* mm outside, so:

- `mean exit_lat(trimmed) − mean exit_lat(clean) ≈ −20 mm` → **placement**.
  Some geometry constant is wrong and calibration fixes it.
- no difference → **the exit lateral is not measuring position**, and no
  amount of flying will fix that.

**It is a crossover, not a fixed set of corners, and that matters.** Two
confounds have to be beaten at once:

- *Across loops* — the drift above.
- *Across corners* — **the ring is hand-built.** It is repeatable, which is the
  whole reason many loops of an imperfect maze beat one loop of a perfect one,
  but it is not uniform. At R125 run 119's four corners exited at 17.0, 22.1,
  14.8 and 19.5 mm, so odd corners run **4.9 mm wider than even ones** before
  anything is done to them. A trim fixed to corner parity would have carried a
  quarter of its own expected answer in as bias.

On the parity of the sum, each physical corner is trimmed on half the loops
and flown clean on the other half; every loop still holds two trimmed and two
untrimmed corners seconds apart; and both effects cancel in the paired
difference. Nothing extra is logged — `loop_ix` and `corner_ix` are already on
every summary row.

`SWEEP_VERSION` is 5. The record layout did not change, which is exactly why
the version had to — a v4 log would decode perfectly and mean something else.

**Note on the per-corner spread.** The corner-to-corner variation is ±4 mm
about a common ~18 mm. The maze's imperfection *modulates* the offset; it does
not cause it. All four corners exit wide.

**`sweep.probe` is deliberately not being used.** A hand-built ring gives
different corridor widths at different points, so a stationary absolute
reading would vary from spot to spot and prove little (bert, 2026-08-20). The
rig is treated as a fixed unknown and differenced away instead — which is what
the crossover above does.

### Runs 120–121 — placement moves the exit one for one, 2026-08-21

Both sessions died after two corners, for the same reason both times, and both
still answered the question they were built to ask.

**The measurement.** Run 121, taken from the traces rather than the summary —
the last valid inside-beam sample before the arc, to the first `in_arc`
sample, with the gate's own correction added back so both numbers are in the
same frame:

| | arc starts after the wall end | exit, outside beam |
|---|---|---|
| corner 0, clean | 38 mm (nominal 37) | **63 mm** from the wall |
| corner 1, trim −20 mm | 59 mm | **42 mm** from the wall |

**21 mm of exit for 20 mm of trim**, on the outside beam alone so no
corridor-sum ambiguity. Run 120 said the same thing more crudely: three
trimmed corners, three guard trips; two clean corners, two survivals.

**Arc placement moves the exit one for one.** That is settled and does not
need testing again.

**What it leaves is an equivalence, not an answer.** The clean corner starts
its arc where the geometry says it should — within about a millimetre of the
wall end, in runs 119 and 121 both — and still exits ~25 mm outside. With a
1:1 response, *"the mouse turns 25 mm too late"* and *"the gate's idea of
where the corner is, is 25 mm out"* are **the same statement**. The gate's
expectation is `corner_entry_s + R − 90 − WALL_GATE_BEAM_AHEAD_MM`, and that
72 mm look-ahead has never been measured independently.

**Why both sessions died at two corners.** Alternating a −20 mm trim put a
deliberately marginal corner in *every* loop. Every loop tripped the lateral
guard, and three failed loops end a session. The crossover design and the
fail-streak guard are incompatible: the crossover guarantees the streak.

### Run 122 — the trim on every corner

`SWEEP_ARC_TRIM_MM` is now **+25 mm on every corner**, not −20 alternating.
Positive starts the arc *earlier* (`control_path_correct` adds to `path_bias`,
`s = (d_est − d0) + path_bias`, so raising `s` brings the tangent point
forward). One question:

- **exits fall to ~0** → the standing 25 mm is one wrong constant in the gate
  geometry. Fold it into the look-ahead and the experiment can finally get on
  with radius and speed.
- **exits unchanged** → placement is a lever that works but is not the cause,
  and the search moves elsewhere.

It should also unstick the sessions. Pulling ~25 mm off a ~25 mm offset puts
every exit well inside the 40 mm guard, so a full grid may run for the first
time since this started — which matters more than it sounds, because **the
slew axis has never once been reached**. Every corner in runs 116–121 is
`slew_ix 0`.

The trim rides on the *gate's* correction rather than being applied at
`corner_open`, deliberately: the hypothesis is that the gate's constant is
wrong, so the trim belongs where that constant is used — and a corner whose
gate never locked gets no trim, which is right, since there is nothing to
correct against.

`SWEEP_VERSION` is 6. The record layout did not change, which is exactly why
the version had to: a v5 log would decode perfectly and mean something else.

### Run 122 — it was a placement error, and it is fixed

`exit_lat` on all seven corners: **0.5, 1.4, 1.1, 2.7, 0.4, 6.9, −1.5** —
mean **+1.6 ± 0.9 mm**, from ~25 mm. The `+25 mm` trim removed the whole
standing offset.

The exit window is not the only witness, which matters after five runs of
arguing about whether it measures anything. The closest approach *inside the
arc*, straight off the raw beams:

| | runs 117–121 | run 122 |
|---|---|---|
| `arc_min_left` | 40–58 | **70–80** |
| `arc_min_right` | 110–150 | **85–97** |

The mouse now passes through the corner cell centred instead of hugging the
outside wall.

**The constant.** Measuring the physical arc start from the wall end, with the
gate correction and the trim added back: 15–17 mm at R125 against a nominal
37, and 38–42 at R100 against 62. **Consistently ~21 mm earlier than the
model, at both radii.** Nominal is `(90 + WALL_GATE_BEAM_AHEAD_MM) − R`, so the
effective look-ahead is about **51 mm, not 72**.

**It is not purely geometry.** The gate stamps `drop_s` when the detector
*confirms*, two or three samples after the beam loses the wall — 10–15 mm of
the 21 at a 400 mm/s approach. Approach speed is `max(v_arc, 400)`, so it has
been pinned at 400 for every corner ever flown, which is exactly why one fixed
number fitted so well. At v500 and v600 the approach speeds up, sample spacing
goes 4 → 6 mm, and the lag grows another 4–6 mm. **Prediction: exits drift
~5 mm outside at v600.** If they do it is lag and wants a speed-dependent
term; if they do not it is geometry and folds into the constant.

**A side effect worth recording.** The corridor-sum anomaly mostly dissolved.
Over corridor samples only: mean sum **175.4** in run 119 (mean |lat| 15.1 mm)
against **172.6** in run 122 (mean |lat| 3.4 mm). Most of "the corridor is 178
not 168" was a *consequence* of sitting off-centre, not a cause of it.

`arc_turn` is unchanged at 87.5–88.5° — that is the slew ramp, which costs
angle rather than position.

### And it still stopped, silently — so now it can explain itself

Loop 1 has three corners and no failure row. `loop_failed()` only wrote a
summary when a corner record happened to be open, and the mouse stopped
between corner 2's window closing (s≈2301) and corner 3's opening (s≈2661).
The console said something; the console was not connected.

Three additions, `SWEEP_VERSION` 7:

**Loop-event rows.** A loop that ends with no corner in progress writes a
record with `corner_ix = SWEEP_CORNER_EVENT`, carrying the state at the stop —
path `s`, the corner it was heading for, lateral and its source, yaw error,
both raw beams, ms into the loop, odometry-minus-`s`. `sweep.dump` prints
these as `X,` lines with their own column names rather than pretending they
are corners, and the viewer puts them at the top of the panel.

**Header counters.** `combo_ix` and `loops_done` now print in the dump header.
They are what separates *"the firmware failed the loop"* from *"the mouse died
and never noticed"* — for run 122 those two numbers alone would have answered
it.

**Corridor breadcrumbs.** Tracing only ran inside the capture windows, so
between them there was no record of where the mouse was at all — precisely
where run 122 stopped. One trace record every 60 mm of corridor, tagged
`SWEEP_CORNER_EVENT`, costs ~29 records a loop (24 KB over a 50-loop session)
and closes the hole. The viewer draws them on the trail and keeps them out of
the per-corner fold and the charts.

None of this covers a mouse that simply loses power — that writes nothing at
all. The header counters are the tell for that case.

### Run 123 — the guard was firing on one packet

Three corners, exits **−1.4, −0.9, −1.5 mm**. The placement fix holds. Then it
stopped, and the log looked empty — but it was not. Corner 3's row says
`fail=lateral`: a corner record *was* open, so `loop_failed()` took the
corner-row branch and the `X,` row never appeared. The trouble is that a
corner which never reached its tangent point has nothing in it, so the row
read `entry_lat 5.2` and then zeros. **`fail=lateral` with no `s`, no lateral
and no beams is not a stop reason.**

Reconstructing it from the trace:

- last logged sample **s=2739**, `L=82 R=93`, lateral **+5.5 mm**, `yaw_err`
  0.07°, braking normally for the corner. Dead centre.
- corner 3's entry tangent is at s=2774, and the inside wall ends — by the
  gate's own arithmetic, `entry_s + R − 90 − 72` — at **s=2737**.
- the guard fired within 4 mm of that.

So the mouse aborted **on the packet where the inside wall ends**, four
millimetres after reading centred. And there is a mechanism: when the pair
fails the corridor-sum check, `corridor_lat()` falls back to the **nearer**
wall — so a beam reading short for one packet is both the reason the pair was
rejected and the number that then gets trusted. `84 − 44 = +40` and the guard
is at 40.

Two things made this invisible. **The tripping packet is never traced** —
guard 1 returns before the trace-write block, so the log always goes dark one
sample early, in every run. And the guard had **no run-length filter at all**,
while guard 2 (raw beam proximity) has always required three consecutive
packets.

Fixed, `SWEEP_VERSION` 8:

- **`SWEEP_LAT_ABORT_N = 2`.** The lateral limit must persist for two packets.
  The margin cost is under a millimetre: the abort is at 40 mm, contact at 49,
  and a mouse in a corridor closes laterally at a few mm per hundred of
  travel — about 5 mm of forward travel per packet at cruise.
- **A loop-event row on *every* failure**, not only when no corner record is
  open. The corner row keeps its own meaning; the `X,` row carries the state
  at the stop.
- **`lat_prev` on the event row** — the lateral one packet earlier. That single
  number separates an artefact (centred, then suddenly not) from a real drift
  (already going), and the viewer says which in words.
- **`sweep_abort` takes a fail code.** A lost edge gate and a human pressing
  stop both ended up labelled `user`; they are not the same finding.

### Run 124 — 58 corners, and the slew rail finally shows itself

**Fifteen loops. The first session ever to get past loop 8, and the first time
R80, R60 and R45 have been flown at all.** Mean exit **+2.0 ± 1.2 mm** over 56
clean corners of 57.

**The under-turn is the slew ramp, quantitatively.** The rail takes
`T = ω/α` to bring the commanded rate to `v/R`, and the mouse leaves that ramp
a heading deficit of `v²/(2R²α)` behind — which it never fully recovers inside
the arc. Predicted against measured `90 − |arc_turn|`:

| cell | ramp/arc | predicted | measured |
|---|---|---|---|
| R125/v200 | 0.04 | 1.7° | **1.7°** |
| R100/v200 | 0.06 | 2.6° | **2.4°** |
| R125/v300 | 0.08 | 3.8° | **3.1°** |
| R100/v300 | 0.13 | 5.9° | **6.9°** |
| R80/v300 | 0.21 | 9.2° | **9.1°** |
| R60/v300 | 0.36 | 16.4° | 8.0° |
| R45/v300 | 0.65 | 29.2° | 10.1° |

Tight wherever the fraction is small; the simple formula over-predicts at the
top end because the **exit** ramp hands some of the angle back — it does not
hand back the displacement, which is the whole point.

**The crash was R45/v400, and it was not flyable.** `ramp / arc = 2v²/(πR²α)`
comes to **1.15** there: the commanded turn rate is *never reached before the
arc ends*. The mouse is being asked for a corner it is not being told how to
drive. The X row caught it exactly —

```
X,14,R,45,400,0,351,0,-16.0,-18.0,one,-25.00,100,255,1281,12.8,yaw
```

— YAW guard at s=351, which is 36 mm into a 70.7 mm arc, `yaw_err` at the
25° limit, `lat_prev` −18 so a genuine drift and not an artefact, and
`wheel_minus_s` 12.8 mm of scrub. "Appeared to not complete the turn" is
precisely right: it could not.

**So the grid now declines cells the rail cannot deliver.** `SWEEP_MAX_RAMP_FRAC
= 0.75` — logged as `SWEEP_FAIL_RAMP`, which is a *result* in the same sense a
guard trip is: "this rail cannot produce this corner" is exactly what the slew
axis exists to find out. The threshold keeps every cell that has actually
flown (R60/v400 at 0.65 is the closest survivor) and rejects R45/v400 and
everything worse. `sweep` marks declined cells with `*` in the ramp table.

At slew 6000 nearly all of them come back — which is the comparison the
experiment was built for, and which no session has yet reached: **every one of
these 58 corners is still `slew_ix 0`**, and the rail does not switch until
combo 25.

**Gate health is starting to matter.** Three corners skipped the gate on
lateral (`entry_lat` over the 13 mm limit, at R125/v400) and four on the
window (R45/v300, R60/v400). Those corners flew on odometry alone. Six misses
in a row aborts the session, so this is the next thing to watch.

### Run 125 — the fallback rule was backwards

Three clean corners, then a stop mid-corridor with the motors simply off. The
X row said exactly what happened, which is what it was built for:

```
X,0,R,125,200,0,2747,3,-45.0,-45.0,one,0.86,82,39,8846,-70.8,lateral
```

Lateral −45 mm, **`lat_prev` also −45**, so not an artefact — and `yaw_err`
0.86°, dead straight. The beams tell the real story:

| s | left | right | sum |
|---|---|---|---|
| 2732 | 82 | 92 | 174 |
| 2736 | 82 | 91 | 173 |
| 2740 | 82 | 91 | 173 |
| 2744 | 82 | 92 | 174 |
| **2747** | **82** | **39** | **121** |

The left beam sits at 82 for the whole corridor. The right drops 92 → 39 in
three millimetres, 27 mm before corner 3's tangent — **the post at the end of
the inside wall**. The sum-check correctly rejected the pair, and then the
fallback took the *nearer* wall, which was the post, called it −45 mm of
lateral and aborted a loop that was centred.

**Which beam is lying follows from the sign of the sum error, and the old rule
had it backwards.** A mouse genuinely 45 mm right reads 129 / 39 — the sum
*survives*, because both beams move together. Only a beam looking at something
that is not the corridor wall breaks it. So:

- **sum too small** → a beam is reading **short** → trust the **farther**
- **sum too large** → a beam is reading **long** → trust the **nearer**

Taking the nearer is right for *clearance* and wrong for *position*, and the
case that actually occurs is the short one. On run 125's packet the new rule
returns +2 mm from the left beam and the guard never fires.

This is not a glitch and the two-packet filter could never have caught it —
the beam sweeps that post at **every corner of every lap**. `lat_prev` is what
proved it, one run after it was added.

A contested pair now reports `lat_src = suspect` rather than hiding inside the
ordinary single-wall case, so it can be counted. `SWEEP_VERSION` 10.

**Also worth knowing:** `wheel_minus_s` is no longer a slip figure. The
`+25 mm` arc trim adds to `s` at every corner, so a clean four-corner loop
reads about −100 by construction. Run 125's −70.8 after three corners is −75
of trim plus a few millimetres of real difference. Read the change across a
loop, not the value.

### Run 126 — the right beam has a fault

Not a firmware regression: **the right side sensor is intermittently stuck.**

| | run 124 | run 126 |
|---|---|---|
| corridor packets with both beams | 4238 | 1640 |
| pairs summing to less than 150 | **0** | **126 (7.7 %)** |
| most common right-beam value | 89 (333×) | **38 (88×)** |

Approaching corner 3, the right beam sat locked at **39 for 150 mm of
corridor** — s=2629 to s=2783 — while the left ran smoothly at 78–80. It
flickered back to the true ~96 four times in that stretch and returned to 39.
That is a channel latching, not geometry: a post is a few packets, not a
hundred and fifty millimetres.

**The sign rule handled it correctly** — sum 118, too small, so trust the
farther beam: +5 mm, which the left beam confirms. But two things followed
from it anyway.

**The gate was still being fed the raw pair.** `corridor_lat()` had the sign
rule; `gate.update()` was handed `wv.lateral_mm` directly, which read −20 mm on
those packets. Over the gate's 13 mm limit, so **corner 3 skipped its gate on
every single lap** (`gate_drop_s = -1`, `SKIP_LAT`) while the guard, looking at
the same packet, correctly saw +5. One lateral, one rule — the gate now gets
the resolved one.

**And that is what unravelled the run.** A skipped gate means the arc is placed
on odometry alone, so the exit is wide; the servo then has more to drain over
the following straight and overshoots further. `entry_lat` walked from +2.7 in
loop 0 to −19.0 by loop 3, and `exit_lat` on the ungated corners reached
+21.9, +21.0, +32.7, +34.3. The collision was loop 5 corner 1 at R125/v300,
`exit_lat` **+47.5** with `lat_src = both` and a plausible sum of 185 — so that
one was real, not an artefact. The guard fired correctly and, again, too late.

**Beam health is now counted.** The session header carries
`suspect_pairs=N/M (x.x%)` — how many two-beam packets did not describe one
corridor. A few per lap is the posts at wall ends. Run 126 would have printed
7.7 % on the first line of the dump instead of needing a packet-by-packet
read. `SWEEP_VERSION` 11.

**Before the next run: check the right sensor.** Nothing in the firmware can
substitute for a channel that reports 38 when the wall is at 96, and every
number in this run is downstream of it.

### Run 127 — 83 corners, the slew rail reached, and the gate is the lever

A resumed session (loops 0–5 are run 126's pre-clean data; 6–27 are after),
28 loops, **combo 28 of 50 — past combo 25, so the slew rail switched for the
first time in the project's history.**

**The ramp rule did its job.** Six combos declined and logged, no crash on any
of them: R45/v400, R60/v500, R45/v500, R80/v600, R60/v600, R45/v600.

**The gate is the single biggest determinant of exit quality**, and it was
refusing a third of the time:

| | corners | mean \|exit_lat\| |
|---|---|---|
| gate LOCKED | 48 (58 %) | **7.7 mm** |
| gate SKIPPED on lateral | 29 (35 %) | **17.2 mm** |

Better than two to one — and the skip is a **latch**. No correction means the
arc is placed on odometry alone, so the exit is wide, so the servo has more to
drain over the following straight and overshoots further, so the next entry is
further off and skips again. Four sessions have now ended inside that loop.
Run 127's last corner entered 13.6 mm off, skipped the gate, and exited
**66 mm** into the outer wall.

So `lat_max_mm` goes from **13 to 25**. The reason for the limit is real — a
mouse off centre sees the wall end at a different point along the beam, biasing
the drop by roughly `0.6 × lat`, so ~15 mm at 25. But the alternative is not
zero error, it is *no correction at all*, and that error is unbounded and
compounds lap over lap. **A bounded bias beats an unbounded one.** The
principled fix is to compensate rather than refuse, since the gate already
knows the lateral — that wants the beam geometry measured on the bench first
(the effective look-ahead came out ~51 mm in run 122 against the 72 assumed),
so it waits for a number rather than a guess.

**Cleaning the sensors was not the fix, and my run-126 diagnosis was wrong.**

```
arc_min_right at CORNER 3, every lap of run 127:
  L0:38 L1:38 L2:38 L3:38 L4:38 L6:38 L7:38 L8:38 L9:38 L10:38
  L11:38 L12:38 L13:38 L15:38 L16:38 L17:38 L20:38 L21:38 L25:38 L26:38
run 124, same corner:
  86 88 110 93 110 91 95 113 107 95 99 92 102 81
```

**Exactly 38, twenty laps running, before and after the clean.** A dirty lens
does not produce the same value to the millimetre for twenty laps, and it does
not survive cleaning. This is a fixed feature at corner 3 that appeared between
runs 124 and 126 — either something physical in that corridor, or a bad LUT bin
that the beam only lands in at that geometry. `sweep.probe` parked on that
approach answers it in seconds, and it is worth doing before reading anything
else from corner 3.

**On "faster then slower":** that is the grid, not the battery. Speed climbs
v200 → v600 as the combo advances, then **resets to v200 at combo 25** when the
slew rail switches. Loops 25–27 are slow because they are the fast rail
starting the speed axis again.

**The slew comparison is not yet usable.** Loops 25–27 are the only fast-rail
data and they sit at the end of a session that had drifted, against slow-rail
loops taken before the clean. Same-cell comparison needs a session that starts
at combo 25.

### The 38 was never a distance (bert, 2026-08-21)

> *"a wall reading of 38 would be almost a wall scrape which didn't happen so
> should be regarded as an outlier"*

Correct, and it reframes two runs of analysis. A side reading is the
**perpendicular distance from the mouse centre** to the wall alongside it, and
the body is 70 mm wide:

```
   35 mm   wall touching the bodywork
   38 mm   three millimetres of air, for 150 mm of corridor at 200 mm/s
   44 mm   the near beam when the mouse is at the 40 mm lateral abort
   84 mm   centred
```

38 is not a near miss to be reasoned about downstream. **It is not a length.**
And the mouse drove on quietly, which settles it.

Everything clever built since run 125 — the corridor-sum check, the sign
resolution, `lat_src = suspect` — exists to cope with a number that should
never have been admitted as a distance in the first place. None of those rules
can recover the beam that was thrown away deciding between them.

So there is now a **validity floor at the source**: `SWEEP_SIDE_MIN_MM = 40`.
A corridor side reading below it is rejected as invalid, exactly like a 255,
before anything consumes it.

**It costs nothing real.** The lateral abort is at 40 mm of offset, which puts
the near beam at `84 − 40 = 44`. Anything the mouse can be at while still
driving reads 44 or more, so a corridor sample under 40 is already past a
threshold that would have fired a packet earlier on a larger, more plausible
number. And it cannot hide a real wall: at a genuine 45 mm offset the near beam
reads 39 and is rejected, but the **far** beam reads 129 and still gives −45, so
the guard still fires. That is asserted in the tests.

**Corridor only.** Inside a junction the beam is not looking at a wall beside
the mouse — it sweeps posts and corner faces at shallow angles, and run 116 saw
a legitimate 40 mm there with the body clear. The junction keeps the model-free
proximity guard, which needs no geometry to be true.

The floor also subsumes the sign rule for the case that has been biting us:
run 125's `82 / 39` is now resolved by rejecting the 39 outright rather than by
reasoning about which beam to believe. The sign rule stays for genuinely
contested pairs where both readings are plausible lengths.

`SWEEP_VERSION` 12.

### Run 128 — lights ruled out, and the trim cannot be a constant

Two sessions in one log, lights off then on. **`arc_min_right` at corner 3 read
exactly 38 on all seventeen laps, both ways.** Ambient light is not the cause,
and the hypothesis is properly dead rather than merely unsupported. Whatever
that channel does at corner 3, it does in the dark.

The new beam-health counter gave its first baseline: **`suspect_pairs =
5062/92084 (5.5 %)`** — a number to watch rather than to act on yet.

**The gate is worth three to one, again.** 49 corners locked at 5.7 mm mean
\|exit_lat\|, 17 skipped at 16.0 mm. Run 127 said 7.7 against 17.2 on a
different sample. That is the strongest lever in the system and the reason
`lat_max_mm` went to 25.

**And here is the thing the run was really for.** Mean `exit_lat` by cell,
negative meaning toward the inner block:

| | v200 | v300 | v400 | v500 |
|---|---|---|---|---|
| **R125** | +20.2 | +7.5 | +6.9 | −0.8 |
| **R100** | +13.7 | +5.6 | −1.2 | −5.5 |
| **R80** | +0.7 | +4.2 | −3.2 | −16.5 |
| **R60** | +5.4 | +1.7 | −8.3 | — |
| **R45** | +7.6 | −8.6 | — | — |

Monotonic in both axes, crossing zero along a diagonal. **A single fixed
`SWEEP_ARC_TRIM_MM` cannot be right everywhere** — +25 is well chosen for the
middle of the grid and wrong at both corners of it, late at R125/v200 and early
at R80/v500. The last failure is the bottom-right of that table: entered 14 mm
outside, turned early, and scraped the **inner** wall — the opposite sign from
every failure before it.

That table is the correction surface, read directly: the trim a cell wants is
`25 + exit_lat(R, v)`, ranging from about +8 to +45. Fitting it is the next
substantive step, and it wants one more session — ideally after corner 3's 38
is resolved, since that corner is a quarter of every average above.

**The failure itself was real, not an artefact.** `left 135 / right 255` with
`lat_prev` also −51: the right beam gives no return because the wall is inside
its minimum range, and the left independently puts the mouse 51 mm toward the
block. Two beams, one story, and the guard fired on the honest one.

### Run 129 — corner 3 in the clear, and the slow rows go

Thirteen corners, three loops, `suspect_pairs = 1083/16683 (6.5 %)`. A short
session, but the shortest and most conclusive table the rig has produced,
because every one of those three loops flew the *same four physical corners*
and the pattern does not move:

| corner | `gate_corr` | `arc_min_r` | flags | exit_lat, loops 0/1/2 |
|---|---|---|---|---|
| 0 | −8.0 / −15.4 / +7.0 | 98 / 95 / 106 | `0xD2` | +8.4 +0.8 +8.3 |
| 1 | −1.6 / −0.4 / +1.8 | 101 / 101 / 103 | `0x92` | +9.3 +5.0 +1.4 |
| 2 | +0.8 / +14.0 / +18.9 | 89 / 85 / 123 | `0x92` | +2.7 −6.4 +23.8 |
| **3** | **0.0 / 0.0 / 0.0** | **38 / 38 / 38** | `0x88` | **+27.5 +20.9 +63.0** |

Corner 3 never gated once. `gate_corr` is exactly `0.0` on all three laps and
bit `0x08` (SKIP_WIN) is set on all three. It is also the corner with the 38,
and it produced the worst exit of every lap by a wide margin. The session
ended there: entered −8.9, exited **+63.0**, lateral guard.

**What the beam is actually doing, from the traces** (bert asked the right
question: step or gradual, and does it track the left sensor?). It is a
**step**, it is **binary**, and it has **nothing to do with the left beam**.
124 corridor packets in the corner-3 approach, all three laps, arc excluded:

| | n | inside beam | left beam | L+R | d(right)/d(left) |
|---|---|---|---|---|---|
| true branch | 33 | 81.2 ± 4.4 | 90.3 ± 3.3 | **171.4** | **−1.32** |
| floor branch | 91 | **38.6 ± 0.6** | 86.6 ± 7.4 | 125.2 | **−0.05** |

Three things settle it:

- **Nothing in between.** The right beam reports 38, 39, 40 — or 68 and up.
  There is no reading between 41 and 67 anywhere in the run. It does not walk
  down to 38, it jumps.
- **The floor does not move when the mouse does.** In the true branch the two
  beams trade off as a corridor forces them to (slope −1.32, sum 171.4 against
  a nominal 168 — that branch is the wall). In the floor branch the mouse
  travels 7 mm sideways and the reading changes by 0.6 mm. **A distance that
  does not change when you move is not a distance.**
- **The left beam is healthy throughout.** One sub-40 left packet in the whole
  run, and that one is inside an arc. No correlation, no common mode.

The duty cycle then says what the floor *is*:

| lap | mean left | implied true right | packets on the floor |
|---|---|---|---|
| 0 | 77.4 | ~93.6 | 35/36 (**97 %**) |
| 1 | 92.4 | ~78.6 | 26/42 (62 %) |
| 2 | 91.1 | ~79.9 | 30/46 (65 %) |

Further wall, weaker return, more floor. That is the signature of a **fixed
crosstalk/noise floor being reported as a range** — what the channel emits
when it has no usable return. It also disposes of the two dead hypotheses at a
stroke: cleaning the optics (run 127) and running lights-out (run 128) changed
nothing because **the deficient reflector is that stretch of wall**, not the
sensor. Corner 3's inside wall is dark, angled, or dirty over roughly 400 mm.

**So the harm is not a missed edge — it is that the floor OUTLIVES the wall.**
The true branch stops at s ≈ 2760–2785; the 38s run on to 2845–2888.
Corner-to-corner drop cadence, from `gate_drop_s`:

| lap | 0→1 | 1→2 | **2→3** |
|---|---|---|---|
| 0 | 839 | 844 | **901** |
| 1 | 842 | 843 | **945** |
| 2 | 871 | 849 | **978** |

Corner 3's drop is stamped 57 / 102 / 129 mm late against a ±35 mm window. The
gate finds an edge, in the wrong place, and **silent-or-certain correctly
throws it away** — the system working, not failing. (The lateness growing lap
over lap is the skip latch: ungated corner, wide exit, worse entry, worse
again.)

**Which makes v13's filter a guard, not a cure — and the traces say so.**
Replaying the real packets through `WallGate` under both policies: lap 0 raw
drops at 2794 and filtered finds nothing at all; laps 1 and 2 drop at 2845 and
2889 **either way**. Skipped packets do not break the presence run, so the drop
still stamps on the first 255. Keep the filter — a floor value must never be
believed as a length, and `corridor_lat()` needs it regardless — but it does
not gate corner 3.

*Two earlier explanations of the 38 are recorded here as wrong: a poisoned
`track_mm_` widening the step test, and 38s landing in the opening to clear
`absent_run_`. Both were plausible from the summary rows; the per-packet
traces refuted both. The summary rows say a corner did not gate. Only the
traces say why.*

**The decisive next test is physical, not firmware.** Move that wall panel —
swap it with another side, or rotate the whole ring — and re-run. If the 38
travels with the panel it is the reflector, and the fix is a strip of matt
white card. If it stays at corner 3 it is the geometry or lighting of that
location. Either way it costs a minute and settles what four runs of firmware
analysis could not. `R` (raw px/sig) parked on that approach is the confirming
instrument: a crosstalk floor shows low signal amplitude, a genuine target at
38 mm shows high.

**And the velocity axis.** Run 129 reached `combo_ix = 3`, which with radius
varying fastest means the *entire session* was the v200 row: R125, R100, R80,
all at 200 mm/s. Set against run 128's surface — where mean `exit_lat` is
monotonic in v and crosses zero near 400, and the v500 column is the tightest
in the grid — the rig has been spending most of its floor time and most of its
FRAM in the cells that behave worst and teach least. bert's read of the
hardware, and his call: *"at 600 mm/s it went very clean… the lower velocity
settings for the loop tests are not needed."*

`SWEEP_DEF_V` is now `{500, 600}`. Grid = 5 R × 2 v × 2 slew = 20 combos, 7
declined by the ramp rule (slew 2500: R80/v600, both R60, both R45; slew 6000:
both R45), 13 flown.

There is a second reason beyond time, and it is the better one. The single
`SWEEP_ARC_TRIM_MM` corrects a placement error that scales with how far the
mouse travels during the gate's detection lag — which is proportional to `v`.
One constant can only be right at one speed. Narrowing the axis from 200–600
to 500–600 shrinks the span that constant has to cover from ~40 mm of required
trim to ~10 mm, which is inside the run-to-run noise. Fitting `trim(R, v)`
properly is still the real answer and is still open; this makes the interim
constant defensible in the meantime.

*(Flashed as `SWEEP_VERSION = 13`.)*

### Runs 130-132 — one wrong constant, three symptoms

Three sessions on identical firmware, 19 of 20 combos each, 147 corners. All
three ended in the same cell (R60/v600/slew6000, heading guard at ~25°) — the
repeatability is itself a result.

**The 38 mm floor at corner 3 is gone.** Zero sub-45 packets on either beam in
5,596 traces; `suspect_pairs` 6.5% → 2.5%. Speed does not explain it (run 127
shows `arc_min_r = 38` at v600 as well as v200), so it was not a slow-corner
artefact. What changed is the line through that corridor — the mouse now sits
~100 mm off the right wall there against ~80 mm before, a different incidence
angle on the same panel. Consistent with the reflectance story, unproven, and
**it may come back**. `suspect_pairs` is the tell-tale.

**The apparent oversteer was a mixing effect, not a cornering error.** Split
every corner by whether the gate actually fired:

| | gate LOCKED | gate SKIPPED | skip rate |
|---|---|---|---|
| R125 | **+1.1** | −16.7 | 25 % |
| R100 | **−0.3** | −18.2 | 40 % |
| R80 | **−0.3** | −19.8 | 61 % |
| R60 | **+1.6** | −17.2 | 87 % |

**Both columns are flat in R.** A gated corner exits within 2 mm at every
radius from 125 down to 60; an ungated one cuts 18 mm inside at every radius.
Only the *mix* changes — which produces a smooth pooled gradient (−3.4 → −7.4
→ −12.2 → −14.7) that looks exactly like oversteer growing as the turns
tighten. Chasing that with a steering-gain correction would have been fixing a
statistic.

**And it is a fixed offset, not a scale error.** Corner 0 has a short approach;
corners 1–3 have the full ~845 mm. Ungated exits: −20.0, −18.0, −21.0, −15.7.
`gate_corr`: −26.3 on corner 0, −23.8 on the rest. Identical. **The odometry is
sound** — the entire error is one wrong constant in the gate geometry.

#### The fix (v14): the look-ahead and the trim are ONE lever

`WALL_GATE_BEAM_AHEAD_MM = 72` was a V1 desk calculation, never checked against
a wall the mouse had driven past. Three independent measurements:

| method | effective look-ahead |
|---|---|
| run 122, arc-start geometry | ~51 mm |
| runs 130–132, drop → first in-arc sample, 81 corners | ~48 mm |
| runs 130–132, censored-normal fit to `gate_corr`, n=147 | ~39 mm |

Set to **45**, deliberately under-correcting: a residual negative bias still
gates, whereas over-correcting flips the sign of every correction in the log.

The code applies `control_path_correct(corr + SWEEP_ARC_TRIM_MM, ...)`, so the
trim and the look-ahead **add** — they are the same lever and must move
together:

```
before   corr ~ -24.5 (gated mean) + 25.0 trim  =  +0.5 applied
after    corr ~  +2.5              -  2.0 trim  =  +0.5 applied
```

Placement is unchanged by construction; only `trim − look_ahead = −47` matters,
and `test_sweep.cpp` now asserts it so a half-change cannot slip through. The
window goes 35 → 45 (the residual spread is sd 12.5, so 35 was only 2.8σ).
**The measure of success in run 133 is the SKIP RATE, not the exit.**

#### Two things the grid measured that are worth keeping

**Tyre limit — RETRACTED, see run 133.** This section originally read the
`|ay|/a_dem` column as a grip measurement and concluded the tyres delivered
full demand to 0.46 g and then failed as a cliff at 0.61 g (ratio 0.876 at
R60/v600). **That was wrong.** `ay` is the measured lateral specific force and
`a_dem` is `v²/R` commanded, so the ratio is `R_commanded / R_actual` — it
measures how tightly the mouse actually turned, not how much grip it had. Two
things prove it:

- The same cell in the other direction reads **1.085**, not 0.876. Grip is not
  direction-dependent; a feedforward asymmetry is.
- The ratio **exceeds 1** at every demand from 0.32 g upward in run 133. A tyre
  cannot deliver more lateral force than was asked of it because of grip. That
  is the mouse turning tighter than commanded.

**There is no evidence of tyre saturation anywhere in this dataset.** The
column is a curvature-error meter and should be read as one. Sideslip, the
thing that would actually indicate grip loss, needs heading rate compared
against path direction, which run 119 did and found nothing.

**Geometry: R = 84 mm is the optimal single-arc radius.** For a 90° arc tangent
to both corridor centrelines, the arc centre sits at (R, R) from the centreline
intersection and the inside post at (84, 84). Closest approach of the mouse
*centre* to the post:

| R | clearance | body margin (needs 35) |
|---|---|---|
| 125 | 67.0 | 32 |
| 100 | 77.4 | 42 |
| **84** | **84.0** | **49** — the arc is centred ON the post |
| 80 | 74.3 | 39 |
| 60 | 26.1 | **impossible** |
| 45 | 10.2 | **impossible** |

Usable band for a 70 mm mouse: **R between 63.7 and 202 mm**. R60 is below the
minimum — there is no path at that radius that clears the post, so the R60
failures are geometry, not control.

**Where to start the turn**, measured, gated corners only:

| | drop → arc start | v500 | v600 |
|---|---|---|---|
| R125 | 17.3 ± 9.0 | 19.8 | 14.3 |
| R100 | 39.5 ± 6.6 | 39.4 | 39.9 |
| R80 | 55.8 ± 4.0 | | |
| R60 | 76.0 ± 7.0 | | |

Speed-independent (R100 is 39.4 at 500 and 39.9 at 600), slope −1 in R. The
rule is pure geometry: **arc start = (90 − R) mm past the wall end**, with the
wall end ~48 mm ahead of where the beam reports the drop. Length constant, not
time constant, again.

**Gyro: no measurable drift over 19 loops.** Run 132's R125 corners run 87.4,
89.8, 89.7, 88.4 … 89.2, 90.7, 87.3, 87.8 — no trend; entry heading error at
corners 1–3 sits at +0.3 to +0.9° after every straight. The alarming-looking
347°-per-lap closure is *not* a gyro result: `arc_turn` spans only the `in_arc`
window, so heading swept during the entry and exit blends is uncounted, and the
deficit tracks radius (−2.4° at R125 → −5.8° at R60) rather than being a
constant percentage.

### Run 133 — v14 confirmed, and the oversteer located in the driver

**The gate change worked, and the margin is not subtle.**

| | corners | gate LOCKED | window skips |
|---|---|---|---|
| dir=R, v13 (runs 130–132) | 147 | 81 (55 %) | 65 |
| **dir=L, v14 (run 133)** | 50 | **48 (96 %)** | **1** |

`gate_corr` went −24.5 → −10.2. The v13 uncensored estimate was −33, the
constant moved +27, and −33 + 27 = −6 against −10.2 measured on a ring the
constant had never seen. The model survived a direction flip.

#### The radius-dependent "oversteer" is the tasym feedforward

bert, watching the viewer: *"the mouse is tuned to R125 but is oversteering at
every smaller radius, this means the tyres are still gripping, it looks like a
driver calibration error."* Correct, and the pairing locates it exactly.
Every cell flown in both directions:

| R | common mode (both dirs) | asymmetry (L−R)/2 |
|---|---|---|
| R125 | −2.15° | +0.22° |
| R100 | −2.30° | +0.81° |
| R80 | −2.53° | +1.32° |
| R60 | −3.74° | **+3.00°** |

The common mode is **flat in R** — the arc-window artefact. **All** the radius
dependence is in the asymmetry, and against turn rate it is a straight line:
**+0.00915° per °/s, r = 0.74** over 13 paired cells. Rate-proportional and
direction-antisymmetric has one owner. `Control.cpp:589`:

```c++
float dv_ff = (w * DEG2RAD) * half_track;
float asym  = vctrl_gains.tasym * fabsf(w * DEG2RAD) * half_track;
dv_ff += (w >= 0.0f) ? asym : 0.5f * asym;
```

At `tasym = 0.200` the commanded differential is **×1.200 left, ×0.900 right** —
a 15 % spread, open loop, scaling with |ω|. The yaw loop absorbs ~94 %; the 6 %
that leaks is the measured 0.92 %. At R60/v600 that is **5.2° of extra
rotation**, which is what put the mouse into the inside wall.

The value came from the procedure recorded in `Bench.cpp`: *"raise until `kl`
exit yaw_err ~ 0, then check `kr` stays near 0."* Sequential, not joint —
raising `tasym` drives left to zero and pushes right away, and the procedure
never goes back. The `0.5f` weighting on right turns is a second patch on the
same wound.

**Note the direction trap.** A global `omega × 0.9` would have been wrong: in
dir=R the mouse *under*-steers at every smaller radius (R60/v500 came in at
84.79°). The fault is an asymmetry, not a gain that is too high.

#### v15: a session gain override, and the null

`tasym` lives in FRAM and **overrides** the compiled `VCTRL_TASYM` — run 133
flew at 0.200 while the constant said 0.10. Editing the constant and reflashing
would have changed nothing. Rather than require two typed commands before every
session (which this rig exists to avoid), `SWEEP_TASYM_OVERRIDE` applies the
value to the live gains at arm time, **RAM only, never persisted**, and
re-snapshots the header gains so the dump reports what actually ran.

Set to **0** for the next pair. That does not assert the physical asymmetry is
zero — it removes the *known* asymmetric term so that whatever (L−R)/2 survives
is the real one, measured on arcs for the first time and fittable jointly.

**The experiment is the PAIR.** v15 flies dir=R (the 147-corner baseline); then
flip `SWEEP_DIR_LEFT_DEFAULT` back to `true` and repeat. One direction alone
says nothing about an asymmetry.

### v16 — the exit overshoot, and a x0.9 on the angular feedforward

**`arc_turn` was the wrong instrument and I had the sign backwards for four
runs.** It spans only the `in_arc` window, and the overshoot happens after the
arc closes. Total heading, straight-to-straight, yaw unwrapped:

| | R125 | R100 | R80 | R60 |
|---|---|---|---|---|
| run 130 (R) | +3.03 | +5.10 | +7.21 | +8.54 |
| run 131 (R) | +2.69 | +4.79 | +6.51 | +8.55 |
| run 132 (R) | +2.99 | +4.67 | +5.84 | +7.88 |
| run 133 (L) | +1.10 | +3.49 | +5.54 | +10.00 |

**Over** 90° everywhere, both directions, growing as the turn tightens — the
opposite of what `arc_turn` said. bert called it from the viewer.

It is a transient, not a steady-state error. Measured with the post window
walked outward, it decays to zero inside one cell:

| past the arc end | +32 mm | +80 mm | +140 mm |
|---|---|---|---|
| R125 | 3.94 | 1.92 | −1.14 |
| R80 | 9.29 | 4.48 | +1.54 |
| R60 | 11.30 | 5.15 | +1.50 |

So `track_mm` is not systematically wrong — the mouse settles on exactly 90.
What is wrong is a **heading overshoot at the arc exit** of up to 11–15° at
R60, and at 500 mm/s the mouse covers a lot of ground toward the inside wall
while the servo drags it back.

#### It is the slew ramp, in closed form — and the rail A/B says clothoids

When the arc ends, commanded curvature steps to zero and the rail bounds how
fast ω can fall, so the mouse keeps rotating by `ω²/2α`:

| cell | measured | ω²/2α |
|---|---|---|
| R125 v500 slew2500 | 5.33 | 10.50 |
| R100 v500 slew2500 | 7.61 | 16.41 |
| R80 v500 slew6000 | 6.18 | 10.69 |
| R60 v500 slew6000 | 13.69 | 19.00 |
| R60 v600 slew6000 | 24.27 | 27.36 |

Consistently ~60 % of the open-loop prediction (the yaw loop eats the rest),
but the shape is exact. It is the mirror of the entry ramp — rate too low
going in, too high coming out. They cancel in heading, which is why total
turn settles at 90. **They do not cancel in position.**

**ADR-002 wrote the decision rule in advance:** *error moves with the rail →
clothoids will help; error does not → it is grip.*

| | slew 2500 | slew 6000 |
|---|---|---|
| R125 v500 | 5.33 | 0.84 |
| R100 v500 | 7.61 | 3.04 |
| R80 v500 | 11.86 | 6.18 |

Halving the ramp roughly halves the overshoot. **The error moves with the
rail. It is the curvature step at the tangent point — not grip, not a gain.**

#### The x0.9, and why `track_mm` is the knob

bert: *"a x0.9 correction be applied to the feed forward angular acceleration
or velocity would reduce overshoot."* Both terms are built from the same
half-track — `dv_ff = ω·half_track` (Control.cpp:588) and
`a_ang = α·half_track` (:606) — and `track_mm` appears **nowhere else in the
control path**: not in odometry, not in fusion, not in any measurement. It is
a pure command scale, so ×0.9 on the feedforward *is* `track_mm × 0.9`, for
both terms at once, with no new gain and no FRAM layout change.
`SWEEP_TRACK_SCALE = 0.90` takes 72.31 → 65.08, RAM only, session-scoped.

Predictions, so run 135 can falsify them:

- exit overshoot at R60/v500/slew6000 falls from 13.7° to **10–12°**
- total turn still settles at 90 (feedback owns steady state)
- the **entry** rate deficit gets slightly worse — the first fifth of an R60
  arc already runs at 0.23 of commanded

If the overshoot does not move, the feedforward is not driving it and the
curvature step is the whole story — which is the clothoid case, and the
clothoid work is queued behind this run precisely so the two can be
attributed separately.


### Run 135 — the first complete grid, and the answer

**All 20 combos.** 13 flown, 7 declined by the ramp rule exactly as predicted,
**52 corners, zero guard trips, 100 % gate lock.** R60/v600/slew6000 — the cell
that ended runs 130, 131, 132, 133 and 134 — flew all four corners.

Mean `exit_lat`, mm (negative = toward the inner block):

| | v500 | v600 | | v500 | v600 |
|---|---|---|---|---|---|
| **slew 2500** | | | **slew 6000** | | |
| R125 | −7.2 | −5.6 | R125 | **+0.1** | −0.9 |
| R100 | −7.5 | −6.5 | R100 | **−0.6** | −6.0 |
| R80 | −7.8 | — | R80 | **−0.6** | −4.5 |
| R60 | — | — | R60 | −6.5 | −8.1 |

52 corners: mean |exit_lat| **6.4 mm**, sd 4.6, worst 17.4; mean |exit_yaw|
2.07°. **The whole slew-6000 / v500 row is inside 1 mm**, down to R80 at
0.32 g.

Note the shape: the slew-2500 column carries a uniform ≈ −7 mm bias that the
slew-6000 column does not. That is the curvature step, visible directly in the
objective.

#### The x0.9 falsified, on a clean repeat

Runs 134 and 135 are the same firmware, and agree cell by cell. Paired change
in exit overshoot across twelve cells: **+0.31°** — slightly the wrong way,
8 of 12 cells up. `kyaw` closes on heading error, so cutting the feedforward
just makes feedback contribute more; the split moves, the trajectory does not.
Reverted to 1.0.

The other two predictions held, which is what makes it a measurement rather
than noise: total turn unchanged, and the entry deficit slightly worse exactly
as forecast (R80 0.289 → 0.278).

**What actually fixed the session was v14:**

| | gate LOCKED | R80 exit_lat |
|---|---|---|
| runs 130–132 | 51–57 % | −12.2 (sd 12.7) |
| runs 134–135 | 96 %, **100 %** | −4.4 (sd 5.4) |

### v17 — clothoid corners

`PathSeg` gained an end curvature, so one struct covers straight / arc /
clothoid as a linear curvature ramp. `pose()` integrates ramp segments with
Simpson over θ(u) — arithmetic rather than a case analysis, because the case
analysis is where sign errors live. `add_arc` emits clothoid→arc→clothoid.
**Off by default**, so a path that does not ask for transitions is
bit-identical to the geometry that flew runs 116–135, and every existing test
passes unchanged.

Measured effect on the reference: worst curvature change per mm goes
**0.01667 → 0.00042**, a factor of 40.

#### The budget, which was not anticipated and shaped the design

A clothoid corner consumes `T = R + m + x₀` of corridor instead of `R`, and
the gate must fire before the tangent point. The wall ends 90 mm before the
corner point, the beam leads ~45, so `T ≲ 135`:

| R | T (pure arc) | room | rail-sized L | T with it |
|---|---|---|---|---|
| 125 | 125 | 10 | 19–66 | 135–159 ✗ |
| 100 | 100 | 35 | 24–83 | 112–144 |
| 80 | 80 | 55 | 30–103 | 95–135 |
| 60 | 60 | 75 | 40–138 | 81–101 |

**The budget grows as the radius tightens — exactly where the overshoot is
worst.** R125 gets no transition and needs none (0.84° at slew 6000); R60 gets
the full rail-sized ramp with 54 mm to spare. Sizing is
`L = min(v²/(R·α), room)`: the rail term is the shortest transition the
executor can already follow, and anything shorter stays rate-limited.

`gate_expect_s` now reads `corner_tangent_mm()` instead of assuming `R` —
assuming `R` would put every gate expectation `T − R` too early, which is the
same class of error that cost runs 120–135.

Each launch prints what it chose:

```
loop 7: R80 v600 slew 6000  clothoid 43.0 mm (rail 43.0, room 65.9)
        T 102.4  gate room 32.6
```

**The prediction to falsify in run 136:** clothoids should collapse the
slew-2500 column onto the slew-6000 column. If the reference never asks for a
step, the rail stops mattering, and the −7 mm bias that only slew 2500 shows
should go with it. Run 135's table above is the baseline.


### Runs 136–137 — the clothoid confirmed, with a built-in control

Two full grids on identical firmware, so this is a repeat rather than a
result. They agree cell by cell, and the split falls exactly on whether the
corner got a transition at all:

| cell | 135 arc | 136 | 137 | 136+137 vs 135 |
|---|---|---|---|---|
| R100 v600 slew6000 | 7.62 | 3.32 | 1.80 | **−5.06** |
| R80 v500 slew2500 | 10.85 | 1.54 | 1.67 | **−9.24** |
| R80 v600 slew6000 | 11.16 | 2.54 | 3.74 | **−8.02** |
| R60 v500 slew6000 | 13.05 | 3.16 | 2.85 | **−10.04** |
| R125 v500 slew2500 | 6.78 | 7.31 | 6.20 | −0.03 |
| R125 v500 slew6000 | 2.47 | 2.75 | 2.26 | +0.03 |

```
cells WITH a clothoid (R <= 100):  -5.70 deg   n=8
cells WITHOUT one    (R125):       +0.45 deg   n=4
```

**R125 is a control channel by construction** — `clothoid_for_room` returns 0
there, so it flies the same pure arc it always did. It moved +0.45° while its
neighbours moved −5.70°. That is about as clean an attribution as this rig can
produce, and it was not designed in: it fell out of the corridor budget.

By radius, repeats pooled:

| R | arc (134+135) | clothoid (136+137) |
|---|---|---|
| R125 | −4.58 (sd 8.68) | −6.80 (sd 7.57) |
| R100 | −4.88 (sd 7.00) | −3.82 (sd 6.55) |
| R80 | −4.44 (sd 5.38) | **−1.86 (sd 5.51)** |
| R60 | −11.67 (sd 16.69) | **−3.38 (sd 7.70)** |

R60 is the big change — mean cut to a third and the scatter halved. R80 is the
best-centred radius in the grid, which is bert's read from the viewer and the
same radius the post-clearance geometry puts nearest the optimum.

**The pre-registered prediction FAILED, and the reason is worth more.** The
prediction was that clothoids collapse the slew-2500 column onto slew-6000.
Mean rail gap: **−5.33 → −5.38 mm. Unchanged.** But at R80/v500 the gap
inverted (−7.2 → +2.2), which is the tell: at the lazy rail the required
transition is `L = v²/(R·α)` with α small, so it is *longer*, and the room
budget clips it — R100/v500/slew2500 wants 57 mm and can have ~40. The
mechanism is confirmed; the prediction was too strong because it assumed the
budget never binds. It binds exactly where the rail is laziest.

**The R125 drift is not monotonic after all.** Tracking the control channel
across every session:

| run | 130 | 131 | 132 | 134 | 135 | 136 | 137 |
|---|---|---|---|---|---|---|---|
| R125 overshoot | 4.48 | 4.31 | 4.72 | 5.22 | 5.42 | 6.32 | **5.42** |

Run 137 came back down. Something that rises and then recovers between
sessions is a charge cycle, not wear — wear does not un-wear. bert's call, and
it retires the "monotonic drift over six sessions" reading, which was
over-fitted to five points. **Battery voltage needs logging before any further
cross-session comparison is trusted**; the analog pin exists on the board but
no firmware reads it and `micromouse_spec.md` §5.6 does not list it.

**Watch:** gate lock is easing off across the clothoid runs — 52/52, 48/52,
44/49. A longer `T` eats gate room by design, and `SWEEP_GATE_ROOM_MIN_MM`
bounds it at 20 mm, but that is the trade the clothoid makes and it is worth
watching rather than assuming.

Run 137's stop was not a cornering failure: `at_s 82`, i.e. 82 mm into the
loop, `lat +41.0`, `lat_src suspect`, beams reading 43 and 150 (sum 193 against
a 168 corridor). That is a launch placement, not a corner.

### Run 138 — dir=L, closing the tasym pair

`SWEEP_DIR_LEFT_DEFAULT = true`, and **nothing else changes** — clothoid on,
tasym 0, track 1.0, the same constants runs 136 and 137 flew. The only
difference between this session and those is the turn direction, which is what
makes `(L−R)/2` the physical asymmetry rather than a mix of asymmetry and
geometry.

**Place the mouse with the inner block on its LEFT.** The stored clockwise set
is a different grid, so both LEDs fast-blink and a wave does nothing until the
old data is dumped and the hand held 5 s.

#### Why this ring and not a rig with two loops in it

A theta rig — the outer ring plus a cross bar — gives both turn directions in
one session, and it is the obvious next rig. It does not replace this
measurement, because **its left and right corners are different pieces of
hand-built wall**, which hands back the confound the pair exists to remove.
Flying this ring the other way keeps all four physical corners.

Three things to know before cutting the bar in:

- **The short cell is marginal.** Corner cells sit at x=0 and x=5, so a bar
  never splits evenly: column 2 gives cells 360 mm and 540 mm. On the 360 mm
  side the corner consumes T = 101–125 mm each end, leaving 110–158 mm of
  straight, while the gate needs ~250–274 mm of clear run before the entry
  tangent to arm. **The gate would arm inside the previous arc on every
  short-side corner.** Put the bar at column 3 and fly the 540 mm cell as the
  primary loop; keep the 360 mm cell as a deliberate back-to-back stress test.
- **The bar punches openings in the inner wall**, which for a clockwise loop
  is the gate's landmark wall. That is the decoy-edge test worth doing — the
  gate has never had to *reject* a false wall-end — but it needs a rejection
  rule designed, not discovered.
- **It is a one-way door on the baseline.** The unbroken ring is what made
  runs 116–137 comparable. Build the theta as a second rig, or bank the ring's
  outstanding questions first.

What the theta genuinely buys that nothing else can: alternating cells give
net-zero turning per double lap, which cancels gyro scale error.


## Guards

It runs unattended, so it stops itself:

| guard | threshold | why |
|---|---|---|
| lateral limit | 40 mm | contact at 49 mm (corridor half 84, mouse half 35). Pose-free — cannot be fooled by odometry drift. |
| heading error | 25° | the reference and the mouse have parted company |
| gate-miss streak | 6 corners | it no longer knows where it is along the corridor |
| session time | 15 min | backstop |

An abort still writes every record up to that point, and prints why.

**There is no impact latch.** V1's accelerometer one went with the speed run,
and its thresholds fired on none of five logged strikes. The lateral guard
does that job here and does it better, because this rig has walls in view
continuously. Re-sizing an impact latch from logged strikes is still worth
doing before anything runs fast in a maze without both walls.

---

## Running with the radio removed

The ESP32 module is worth real battery on a 350 mAh pack, and the firmware is
built to run without it. `HAL_RADIO_ENABLED` in `lib/HAL/HAL.h` now defaults to
**1** (2026-08-21) so floor calibrations can be triggered without touching the
mouse; set it to **0** and Serial5 is never begun, writes are discarded, reads
always return −1.

**If you are comparing sweep sessions, keep this setting constant across them.**
A powered ESP32 changes the discharge profile, and the R125 control channel
already shows a battery-shaped wobble across runs 130–137. Mixing radio-on and
radio-off sessions would add a second unexplained variable to the one we are
still trying to remove.

That is not only about power. **With the module gone, RX5 (pin 21) floats.**
Noise makes start bits, the UART decodes garbage characters, and those go
straight into the console parser — where a single stray `m` is a motor
self-test and `e` aborts a mission. Six minutes of unattended running is
plenty of time for that to happen. Disabled means genuinely not read; and
even with `HAL_RADIO_ENABLED 1` the RX pin is now pulled up, so an
unpowered module cannot inject either.

Two related things were tightened at the same time:

- **Radio writes are non-blocking.** `Serial5.write()` blocks once its buffer
  fills; at 460800 baud a chatty moment would hold the main loop for
  milliseconds while a control loop is flying the mouse. It now takes what
  the UART has room for and drops the rest. Console output is diagnostic —
  losing a line beats stalling the loop. Bulk dumps go over USB anyway.
- **USB writes only happen when a terminal is actually open** (`if (Serial)`,
  i.e. the host has asserted DTR). Writing into a configured-but-unread
  endpoint is the one remaining way the USB stack could hold up the loop.

Nothing else in the run path blocks: no `delay()`, no spin-waits. FRAM writes
briefly mask interrupts, but they are 16 and 32 bytes — at 4 mm spacing and
400 mm/s that is ~100 writes/s of ~20 µs against a 2 ms control tick.

## FRAM note

The sweep and the flight recorder **share** the 510 KB bulk region — they
instrument different things and are used months apart, so splitting it would
halve both. Each writes its own magic and each refuses to write when the
other owns the region: after a sweep, a search abort will decline to record
its outcome string rather than scribble over the experiment, and `y` will say
so rather than print garbage. Dump one before starting the other.
## RUNS 138-140: the wheel constants, and the between-loop turn that broke a lap

Three dir=L sessions on v18. 138 had missing internal walls (posts all
present), 139 and 140 were complete. All three ended the same way: a late lap
launching badly and grazing the outer wall.

### The `j` differential found `circr` out by 0.24%

Three `j` runs, last two identical: believed L 907.1 / R 904.3-904.6 against
an actual 906-907 mm. The mouse believed the two wheels had gone different
distances on a straight line, which is a wheel-constant error and nothing
else. Set as compiled defaults **and** saved to FRAM:

    circ_left_mm  = 67.788
    circ_right_mm = 68.103

**FRAM overrides compiled defaults.** This bit us twice: `tasym` in run 133,
then `circl`/`circr` in run 140, where the new constants were compiled but
never flew, and `strim` was then re-fitted against the OLD ones. Re-running
the C test after saving gave **-0.0886, essentially unchanged from -0.0823**.
That is the useful result: `strim` is not absorbing a circumference error.
An 8.2% differential is ~10x any plausible circumference mismatch, so it is
torque or friction asymmetry, and the wheel constants were never going to
touch it.

### The failed lap was `square_up()`, firing for the only time in the session

bert, watching: *"starting the last lap the mouse visibly adjusted its pose,
went into the first corner badly and came out worse."* That adjustment was a
named function in our own code.

`square_up()` compared the fused yaw against the nearest cardinal between
loops and, past a 1.5 deg deadband, turned the body in place to remove the
difference. Every loop boundary in run 140, last sample of one lap against
first of the next:

| final yaw | beams before | beams after |
|---|---|---|
| -0.99 | L81 R90 | L82 R90 |
| +0.53 | L89 R82 | L90 R82 |
| +0.32 | L88 R83 | L89 R83 |
| -0.19 | L87 R86 | L86 R85 |
| +0.32 | L89 R82 | L89 R82 |
| +0.17 | L88 R83 | L88 R82 |
| +1.15 | L91 R80 | L90 R80 |
| +0.48 | L88 R84 | L87 R84 |
| +0.66 | L88 R84 | L88 R84 |
| -0.07 | L86 R86 | L86 R85 |
| **+1.02** | **L89 R81** | **L98 R75** |

Ten boundaries hold position to within a millimetre. One moves 7 mm, and it
is the boundary into the lap that failed. Loop 17 corner 0 then entered at
**-25.7** (every other loop launched between -1.2 and -8.8), missed
`lat_max = 25` by 0.7 mm, hit SKIP_LAT, flew ungated, exited -23.3, drifted
to -40.5 and tripped the guard.

**Why that boundary.** The mouse yaws while braking. Loop 16's final straight,
s 3006 -> 3170 as v falls 600 -> 131: yaw -0.40 -> +1.02 and the beams walk
86/86 (dead centre) to 89/81. It was still turning at the last logged sample,
so at rest it was past the 1.5 deg deadband. The +1.15 lap above sat just
under it. The deadband was being grazed, and which side it landed on was
effectively a coin flip.

**Why the correction hurt.** Two independent faults, and both are structural:

1. **The reference drifts.** Fused yaw accumulates error over a 17-loop
   session; the mouse's physical alignment does not (corner 0's entry lateral
   held flat near -6 mm all run). `square_up()` read accumulated gyro drift as
   a physical misalignment. A deadband makes this worse, not safer: every
   silent lap lets the drift grow, and the lap that finally trips dumps the
   whole accumulation into the body at once.
2. **An in-place turn is the wrong actuator.** At ~2 deg with a 2000 deg/s^2
   ramp the manoeuvre is ~60 ms and never leaves the acceleration phase, so
   the wheels break out of stiction unevenly and the body translates. It moved
   the mouse 7 mm FURTHER off centre than the error it was called to fix.

**And the launch made it permanent.** `control_path_start` captured
`path_yaw0 = fusion_yaw_deg()`, so the path frame is the BODY. A mouse
standing a few degrees off the corridor flies the whole first leg a few
degrees off it, and the lateral servo can only trade angle for offset - it
never converges. The trace shows exactly that: `yaw_err` pinned at +1.4 to
+2.3 for the entire leg while the beams diverged 101/70 -> 110/62. Corner 0
has the short approach, so there was never enough leg to drain it.

### v19/v20: what changed

- **`lat_max_mm` 25 -> 35.** The refusal exists because the drop is biased by
  `lat * (beam_ahead / corridor_half) = 0.54 * lat`; at 35 that is 19 mm,
  still inside the +/-45 window, so a bad lateral gets caught by the window
  test on its merits rather than pre-emptively by a proxy. Both terms are now
  measured, so **compensating instead of refusing is finally buildable** and
  would let the limit come back down.
- **`square_up()` retired.** Replaced by a `WallAlign` fit over each straight
  leg: `corridor = yaw_bar - eps`, with both terms averaged over the same
  samples (the lateral servo is steering during the fit, so a one-instant yaw
  would measure the transient and call it a misalignment - pinned in
  `test_wall_align.cpp`). Between loops the FRAME is shifted so the corridor
  reads an exact cardinal. **Nothing moves.** That is `square_up()`'s stated
  purpose - stopping the frame rotating over a session - applied to the datum
  instead of to the body, which is where the error actually lived.
- **`control_path_start` gained a `yaw0_offset_deg`** (defaulted 0, so every
  other caller is unchanged). The sweep passes `corridor - yaw`, so the path
  is built on the corridor and the body's residual misalignment drains as a
  heading transient while the mouse accelerates - rolling, not scrubbing.
  Clamped at 6 deg; past that it launches on the body and says so.

**What run 141 should show.** No pose change at any loop boundary, a `square:`
line every lap reporting eps and the frame correction, and corner 0 entry
laterals that stay in the -1 to -9 band for the whole session including the
last lap. If eps comes back consistently non-zero and same-signed lap over
lap, that is the gyro drift rate, measured for the first time.

### `ch1 front_r` reads a wall in the dark

From the C-test boot report: `ch1 front_r dark 88 thresh 60 margin -28`. The
other three channels are +22, +60, +22. With the emitter off that channel
says "wall present". Ambient leak or a reflector in its field. It does not
touch the loop rig (side beams only) but it will affect anything front-wall
dependent, the search included.


### v21: launch rows (`Q,`)

Run 141 completed and the measurement it was built to produce did not survive
it. The corridor fit's `eps` went to the console; the console is what happens
to be connected, and the dump is what gets kept. So every loop now writes one
record at launch, before it moves:

    Q,loop,dir,R,v_arc,slew_ix,eps,frame_corr,body_off,cardinal,lat0,lat_src,
      left,right,fit_n,fit_span,fit_rej,square

- `eps` - fitted heading error to the wall. **This is the number to read
  across a session.** Same sign lap after lap is the gyro drift rate; scatter
  about zero is fit noise. The viewer reports mean +/- se and says which it is,
  because the question is whether the centre is off zero, not how wide it is.
- `frame_corr` - what squaring shifted the fused frame by.
- `body_off` - how far the body was from the corridor at launch, i.e. what the
  path frame was offset by. Bounded by `SWEEP_SQ_MAX_DEG`; past that the path
  is built on the body and `square` reads `body`.
- `lat0` / `left` / `right` - the standing pose, which the fit cannot see (it
  ends while the mouse is still moving and the brake then translates it).
- `fit_n` / `fit_span` / `fit_rej` - what the fit was worth AT THE MOMENT IT
  WAS USED. `lane` keeps accumulating into the next leg, so reading its size
  later would report a different fit from the one that produced the answer.
- `square` - `nofit` | `squared` | `refused` | `body`.

Same 36-byte record, `corner_ix = SWEEP_CORNER_LAUNCH` (254), kept out of the
corner statistics exactly as `X,` rows are - pooling a launch row into a
corner mean would be averaging a standing pose with a measurement. Costs one
record per loop; the summary region holds 2048, so a session would need 340
loops to notice.

## RUN 142: a clean repeat, the first drift number, and a false 40

Same firmware as 141 plus the launch rows. **141 had a fresh battery, 142 did
not** (bert), which makes the pair a battery A/B nobody designed.

### It repeats

| | 141 (fresh) | 142 |
|---|---|---|
| combos | 20/20 | 20/20 |
| corners, failures | 52, none | 52, none |
| gate LOCKED | 52/52 | 52/52 |
| mean \|entry_lat\| | 4.80 | 4.74 |
| corner-0 entry | -2.44 sd 2.21 | -2.11 sd 2.17 |
| gate_corr | -0.79 sd 6.5 | -0.54 sd 7.3 |
| mean \|exit_lat\| | 7.74 | 8.73 |

Twelve loop boundaries, maximum movement **1 mm**, one of them at yaw +1.31 --
higher than the +1.02 that tripped the old deadband in run 140. Nothing moved.

Per-corner exit_lat repeats to a degree the rig has never shown before:
corner 2 reads **+10.79 in both sessions**, corner 3 +10.46 / +11.89. The
hand-built ring is repeatable, as bert said; these are wall positions, not
control error.

**Battery is not driving the corner-0 v600/slew1 group.** That group held
-12.7/-12.3/-13.4/-4.5 on the fresh pack and -11.1/-6.1/-7.9/-2.8 on the
tired one. Less charge, smaller error. Whatever it is, depletion is not it --
which also means the grid's slew/battery confound (slew_ix is the slowest
axis, so slew1 always flies on a more depleted pack) did not cause it.

### THE FIRST DRIFT NUMBER -- and it is not the column I said

I wrote that `eps` was the drift measurement. The first session to produce any
says otherwise, and the correction is more interesting than the claim.

**eps is the TRAVEL direction relative to the wall.** On the final leg the
lateral servo is still converging, so eps mostly measures that. Run 142:
**eps -1.62 +/- 0.14 deg, negative on 12 of 12** -- and `lat0` was **+3.6 mm**,
so the mouse ended each lap left of centre and travelling right. Those two
agree with each other. It is a servo settling, not a gyro.

**frame_corr is the drift channel.** It is `cardinal - (yaw_bar - eps)`, and
yaw_bar and eps are averaged over the same samples, so the convergence enters
both and cancels -- which is exactly what `test_wall_align.cpp`'s sweep-law
test pins. What survives is the angle between the fused frame and the
corridor, and squaring resets that to zero at the start of every lap. So:

    frame_corr = gyro error accumulated over ONE lap
               = -0.708 +/- 0.135 deg,  negative 12/12,  t = 5.2

Twelve of twelve on one sign is p = 1/2048 against a zero-mean null. **This
rig has never been able to state a gyro error before.** Run 133's "no
measurable drift over 19 loops" came from `arc_turn`, which spans only the
`in_arc` window and cannot see it.

**Bias or scale is NOT yet settled**, and one direction cannot settle it:

- bias accumulates with TIME, scale with ANGLE TURNED
- the v500 laps are ~20 % longer in time than the v600 laps
- difference came out **+0.32 +/- 0.27 deg (t = 1.2)** -- decides nothing,
  and the fixed inter-lap stop dilutes the time axis further

The clean test is **DIRECTION**. A bias keeps its sign flown clockwise; a
scale error flips it, because the lap then turns -360 instead of +360. If it
is scale, it is 0.197 %. One dir=R session answers it, and it is the same
session the tasym pair already wants.

### `arc_min` is lying at corners 2 and 3, and the run-129 test says so

Corner 2's `arc_min_left` reads **40 or 41 in all thirteen loops**. Corner 3's
`arc_min_right` reads **38 to 40 in all thirteen**. Radius runs 125 down to 60
across those loops -- completely different lines through the corner cell --
and the number does not move.

That is the run-129 discriminator, verbatim: *a distance that does not change
when the mouse moves is not a distance.* bert watched this session and called
the clearance excellent; 40 mm would be 5 mm from the bodywork.

What separates it from run 129's floor:

- values spread **39-44**, not pinned at 38.6 +/- 0.6
- **inside the arcs only** (`in_arc = 1`), never in the corridor
- two DIFFERENT beams at two DIFFERENT corners -- left at corner 2, right at
  corner 3 -- so it is not one sick channel
- **run 141 was completely clean at the same corners** (73-84 / 80-91,
  varying with radius as it should), and run 140 showed it on loop 0 only

Something about the rig changed between 141 and 142. Panel moved, ring
rotated, lighting -- unknown.

**What it does and does not break.** `corridor_lat()` exempts junctions from
the `SWEEP_SIDE_MIN_MM` floor, so this does NOT poison the lateral the mouse
steers on, and nothing in the run misbehaved. What it breaks is `arc_min` as a
CLEARANCE instrument, which is what it exists for -- read literally it says
the mouse spent every corner 2 five millimetres off the wall. And the beam
proximity guard sits at `SWEEP_BEAM_ABORT_MM = 30`, so the whole session ran
**8 mm from a session-ending trip on a reading that is not a distance.**

## RUN 143 (dir=R): the prediction confirmed, and a clockwise instability

### SCALE, not bias -- pre-registered and confirmed

|  | frame_corr | same sign |
|---|---|---|
| 142 dir=L | **-0.708 +/- 0.135** | 0 of 12 positive |
| 143 dir=R | **+0.679 +/- 0.208** | 8 of 9 positive |

Bias predicted -0.708 (same sign, because a clockwise lap takes the same
time). Scale predicted +0.708 (flipped, because the lap turns -360 instead of
+360). **Observed +0.679.** The sign flipped and the magnitudes agree to 4 %.

    frame_corr = -phi * (s - 1),  phi = +/-360 deg per lap
    s - 1 = +0.00193   ->  the gyro OVER-READS rotation by 0.193 %

This is the first physical constant the ring rig has measured that is not
about the ring. It took the launch rows, the corridor squaring (which zeroes
the frame each lap so the error cannot hide in an accumulating offset), and
one line changed between two sessions.

**Where the error is NOT.** `ImuIcm45686.h` computes `raw / 8.2f`, while
+/-4000 dps over int16 is 8.192 -- so the driver under-reads by 0.098 % and
correcting that constant alone makes the measured error WORSE (0.193 ->
0.291 %). The part's own scale factor is out by about +0.29 %, which is
ordinary part-to-part variation for a MEMS gyro. The fix is a measured scale
correction. It touches every turn the mouse makes, search included, so it
wants one confirming session before it goes in.

### The run failed, and the clockwise direction is not repeatable

Ended at combo 16 of 20, loop 15 corner 0, lateral guard at -41.0 mm. bert
saw the graze and called the oversteer from the floor. 37/37 gated, but
**14 of 37 corners flagged SLIP** against 7 of 52 in run 142.

Corner 0 exits, dir=R: -18.4, -23.5, -19.0, -26.6, -27.1, -14.2, +6.2, -6.0,
-21.5, -41.0. Mean **-19.1** against **-1.8** for the same corner in 142.
Negative is toward the INSIDE for a clockwise loop, so the mouse is cutting
in: oversteer, as seen.

**But the cause is not settled, and the reason is that dir=R does not
repeat.** Five clockwise sessions against five anticlockwise ones:

|  | \|arc_turn\| spread | gate_corr |
|---|---|---|
| dir=R (134,135,136,137,143) | 87.80 - 92.10 (**4.30**, sd 2.10) | -0.47 .. **+17.93** (sd 7.43) |
| dir=L (138,139,140,141,142) | 87.47 - 88.25 (**0.78**, sd 0.30) | -5.28 .. -0.54 (sd 2.33) |

Anticlockwise is rock steady across five sessions. Clockwise wanders by 4.3
degrees of arc rotation and 18 mm of gate correction, and the two move
together (**r = +0.81**). Run 143's +17.93 is far outside anything either
direction has shown; the previous clockwise runs with the same clothoid
geometry read +5.02 and +3.13.

The +18 is real, not a summary-field artefact -- the traces confirm it
through a second window. Drop to first `in_arc` sample:

    142 dir=L   26.0 mm      137 dir=R   28.7 mm
    143 dir=R   43.5 mm      136 dir=R   29.5 mm

**What it is NOT.** Not a fixed per-beam asymmetry in the look-ahead: runs
134 and 135 flew clockwise on the same right-hand beam and read +0.50 and
-0.47. A constant optical difference would have shown there.

**What was tried and did not decide it.** Within run 143, exit_lat against
gate_corr gives slope +0.19 +/- 0.20 (r = +0.16, t = 0.9) -- no relationship.
That test is the wrong instrument and it is worth saying why: the gate
REMOVES the odometry error it measures, so a correctly-applied correction
leaves no exit signature however large it is. Only the CONSTANT part of the
bias misplaces every arc equally, and a regression cannot see a constant.

**Nothing in firmware changed but the direction flag** (v22 flips one line;
141, 142 and 143 are otherwise the same build). So the +13 mm that separates
143 from 136/137 is either the rig, something about clockwise interacting
with the new launch frame, or drift in the optics -- and this data cannot
separate them.

**The cheapest next step needs no flash: re-run 143 unchanged.** If gate_corr
returns +18 it is structural and belongs to the direction; if it returns to
+3 to +5 like 136/137, run 143 sat on a rig that had moved. bert has said
walls shift a few mm between runs but never during one, and a moved inside
panel would do exactly this. Resisting the temptation to add a
direction-specific trim first is deliberate -- that is how the -24.5 mm
offset survived seventeen sessions.

## RUN 144 (dir=R, same binary as 143): the scale error stands, and the
## oversteer is PLACEMENT

bert did not re-flash, so 143 -> 144 is a pure RIG A/B: identical firmware,
one wall-alignment cleanup between them. That is a better experiment than the
repeat that was asked for.

### The scale error, measured a third time

| | frame_corr | n |
|---|---|---|
| 142 dir=L | -0.708 +/- 0.135 | 12 |
| 143 dir=R | +0.679 +/- 0.208 | 9 |
| 144 dir=R | **+0.693 +/- 0.148** | 12 |

    pooled |frame_corr| = 0.710 +/- 0.086 deg per lap  (n = 33)
    s - 1 = 0.1971 % +/- 0.0238 %

Three sessions, two directions, agreeing to within 2 %. This is now a
measured constant and not a result. Correcting it is a driver change that
touches every turn the mouse makes, so it wants doing deliberately -- but the
number is no longer in doubt.

### The session completed

20/20 combos, 13 flown, 52 corners, **52/52 gate lock, zero failures** --
the first complete clockwise session. Mean |exit_lat| 11.02 -> **7.09**.
The wall cleanup did real work.

### THE OVERSTEER IS ARC PLACEMENT, NOT STEERING

The same lever runs 120-121 measured (21 mm of exit per 20 mm of trim, 1:1)
accounts for the whole direction difference:

    delta gate_corr   142 -> 144   -0.54 -> +13.12   = +13.66
    delta exit_lat    142 -> 144   +7.12 ->  -5.09   = -12.21
    ratio                                              -0.89   (lever: -1.0)

A +13 mm gate correction starts every arc 13 mm early, and an arc started
early exits toward the inside. That is exactly what bert sees. **No steering
gain is implicated**, which is the same conclusion runs 130-132 reached in
the other direction and the same one that was nearly missed there.

### What the 13 mm is NOT -- two candidates killed cheaply

**Not the path prediction.** Checked on the host, no flying:
`clothoid_for_room`, `tangent_for` and `begin_ring`'s corner entry/tangent
are **bit-identical** for +90 and -90 at every radius in the grid. So
`gate_expect_s` is direction-symmetric and cannot be the source.

**Not odometry.** Measured corner-to-corner pitch from `gate_drop_s`:
858.7 mm (142 dir=L) against 858.1 mm (144 dir=R), n = 39 each. The mouse
measures the same distance round the ring both ways to within 0.6 mm --
so the 8.2 % drivetrain differential in `strim` is not leaking into distance.

**Not the walls, mostly.** The cleanup moved gate_corr +17.93 -> +13.12.
About 5 mm of the bias was rig; **13 mm survives a wall alignment.**

### What is left, and the history that points at it

    dir=R   134 +0.50   135 -0.47   136 +5.02   137 +3.13   143 +17.93  144 +13.12
    dir=L   133 -10.18  138 -5.21   139 -5.28   140 -2.05   141 -0.79   142 -0.54

Anticlockwise has sat near -1 for three sessions. Clockwise has moved from
about +2 across runs 134-137 to about +15 across 143-144, with no firmware
change that explains it. The remaining candidate is the RIGHT side beam's
effective look-ahead having changed -- `WALL_GATE_BEAM_AHEAD_MM` is one
constant serving two beams, and its own comment says to re-measure if the
optics move. That is a bench measurement, not another session.

### `arc_turn` is placement-sensitive -- do NOT set tasym from it yet

|arc_turn| moved **92.10 -> 93.13** between two runs of the SAME BINARY that
differed only by a wall cleanup. A gyro-measured body rotation cannot depend
on wall positions except through where the arc is placed, so `arc_turn` is
reading placement as well as rotation.

That contaminates the CW/CCW pairing: 142 vs 143 gives (L-R)/2 = -1.93,
142 vs 144 gives **-2.44**, and the two-direction mean is 90.18 and 90.69
respectively rather than the 90.00 that would make the window artefact zero.
The physical asymmetry is real -- it agrees in sign and rough size with the
-1.83 +/- 0.53 from run 133 -- but it is not yet measured to a precision
worth setting a gain from. Fix the placement bias first; the asymmetry
measurement gets cleaner for free.

### Watch

SLIP flags: **19 of 52** in dir=R against 7 of 52 in dir=L. Nothing failed,
but that is a large direction asymmetry in the slip detector and it is new.

## RUN 148: what a POST looks like -- and it is not what I predicted

Two inner walls removed. Clockwise (147) they sat on the turn exits, so every
F row came back `runs_seen = 1`; anticlockwise (148) the same two gaps land on
the APPROACHES, and corners 0 and 2 saw a bare post.

### The measurement

| | packets | span mm | min range |
|---|---|---|---|
| **post** (n=26) | **30.8 +/- 5.1** | **22.1 +/- 4.0** | 87.3 +/- 2.2 |
| wall (n=26) | 199 | 151 | 86 |

**A post is 2.5x LONGER than `present_n = 12`, not shorter.** I predicted four
to six packets from a guessed 2-3 mm packet spacing, then revised to 15-18
from run 147's measured density, and the real answer is 24 to 41. Both
predictions were wrong and in the same direction, for the same reason: the
beam sweeps across a post at a shallow angle, so 12 mm of physical post
subtends 22 mm of travel.

### THE EXISTING GATE ALREADY WORKS ON POSTS

No new detector. Run 148 gated **52 of 52**, and the stripped corners flew as
well as the intact ones:

    corner 0 (post)   LOCK 13/13   gate_corr +14.51   exit -6.17
    corner 1 (wall)   LOCK 13/13   gate_corr  -2.93   exit -0.49
    corner 2 (post)   LOCK 13/13   gate_corr  -4.85   exit -1.02
    corner 3 (wall)   LOCK 13/13   gate_corr  -3.64   exit -0.55

Whole session |exit_lat| **3.72 mm**, the best of any session on this rig
(142 was 8.73, 146 was 11.62). The step detector acquires the post as a short
wall and drops cleanly off its far end. bert's thesis -- "the maze is known,
only posts matter" -- is what the data says.

### Post against wall is trivially separable

22 mm against 151 mm, with nothing in between. A feature can be CLASSIFIED by
its run length alone, which is what lets an observed sequence be matched
against a map. That is the piece the counting gate needs and it comes free.

### What did need fixing: the COUNT

The raw stream splits a post into two or three pieces -- 39 precursor blips of
2.5 packets at 107-140 mm, ahead of the solid run, separated from it by 1.9 mm
on average and **4 mm at worst**. Those are dropouts inside one feature, not
features, and they are the beam grazing the post's edge before it settles.

`runs_seen` therefore read 2, 3 or 4 where the truth is 1 -- and the count is
exactly what bert's gate is to be built on. **`WallGate::BRIDGE_MM = 8`**: a
run stays open across a gap of up to 8 mm. All 26 approaches collapse to
exactly one run, with twice the margin over the worst gap observed, and a real
corridor opening (>= one cell) is nowhere near it.

### The question this rig still cannot answer

bert asked for "the gap from the previous post". It is not in this data and
cannot be: posts flush with a continuous wall produce no feature at all, so
the only posts that stand out are the two at the stripped corners, one per
approach. Post-to-post spacing needs a maze with several gaps AND a wider
observation window -- the observer currently sees only
`(SWEEP_TRACE_HALF_MM - R)` before the entry tangent, 145 mm at R125, because
the gate is fed only while a corner RECORD is open. Decoupling that is now
the blocking piece of work for the counting gate.

## RUN 149: the window works; the RETURNS are what ran out

More inner walls removed, dir=L, same settings as 148. Clean: 52/52 gated,
no failures, |exit_lat| 3.78 (148 was 3.72).

**The 500 mm observation window is live.** Corner 1's run begins at s = 936,
while the corner RECORD only opens at entry - (SWEEP_TRACE_HALF_MM - R) =
1426.9 - 145 = 1282. A feature recorded 346 mm before the record exists can
only have come from v26's distance-keyed arming. The log is capturing the
whole corridor.

**What it is capturing is nothing.** The inside (left) beam returns a valid
reading on only **62 %** of corridor packets, down from 76.7 % in 148, and the
blind stretches are long:

    n = 77 blind stretches   median 186 mm   90th pct 263 mm   max 298 mm

A median blind stretch is longer than a cell. That is why an approach shows
one to three features and not five: the posts are not returning.

### And the reason is a feedback loop worth naming

| | 148 | 149 |
|---|---|---|
| inside beam valid | 76.7 % | **62.0 %** |
| mean lateral | -3.74 (sd 5.4) | **-7.82 (sd 8.5)** |
| left beam when valid | 85.0 | 89.4 |
| right beam when valid | 77.7 | 73.2 |

The lateral servo only acts on `SWEEP_LAT_BOTH`; with one wall it sets the
bias to ZERO. So: walls removed -> inside beam blind 38 % of the time in
186 mm stretches -> servo unpowered -> the mouse drifts ~8 mm toward the OUTER
wall -> the inside beam is further from the posts -> weaker returns -> fewer
features still. It is self-reinforcing, and it gets worse the more walls come
out, which is the direction the rig is heading.

Note the run was still CLEAN -- 52/52, |exit| unchanged. The drift does not
cost cornering. It costs the post data that post-based navigation depends on.

### The available fix, not yet made

The OUTER beam was valid on **100 %** of corridor packets in both runs, and
`corridor_lat()` already computes a single-wall lateral (`SWEEP_LAT_ONE`,
`HALF - L` or `R - HALF`). Letting the servo steer on one wall would give it
continuous authority instead of 62 %.

The reason it was restricted to BOTH is real: the single-wall branch has no
common-mode cancellation, so the ~5 mm absolute error measured after the run
145 recalibration becomes 5 mm of lateral error directly, and runs 125 and 126
both died on a bad single beam. But the trade has changed. A bounded 5 mm bias
against an unbounded 8 mm drift is the same argument that took `lat_max` from
13 to 25 to 35, and it wins for the same reason.

---

# CAL-6 — the corner-calibration circuit (2026-08-27)

> **The maps below are better viewed graphically**: open
> `tools/tune_gui.html` in Chrome/Edge (no connection needed) and pick the
> rig in the dropdown — walls, flown path, turn types, sealed cells and a
> wall-by-wall build list, rendered to scale. The ASCII here is the
> fallback for terminals.

The successor to the outer ring as the tuning rig: a closed 28-cell circuit
in the 6x6 that contains every corner type in the L3 trim table
(TUNING_PLAN.md) and repeats identically every lap. Designed with bert
2026-08-27; verified to close, no revisits, all tangent/setback fits clear.

## rev A — the build spec (start here)

```
+--+--+--+--+--+--+          CW lap (start (0,0), N):
| o  o| o  o  o  o|            U-RIGHT   NW hairpin (0,5)/(1,5)
+  +  +  +--+--+  +            U-LEFT    tooth (1,3)/(2,3)
| o| o| o| .| .| o|            R84 x3    (2,5) (5,5) (5,3)
+  +  +  +--+--+  +            U-LEFT    dent (3,3)/(3,2)
| o| o  o| o  o  o|            R84 x3    (5,2) (5,0) (0,0)
+  +--+--+  +--+--+
| o| .| .| o  o  o|          CCW lap = mirror: 6x R84-LEFT,
+  +--+--+--+--+  +            2x U-RIGHT, 1x U-LEFT.
| o| .| .| .| .| o|
+  +  o = corridor           28 cells, ~5.1 m, ~10 s at v500.
| o  o  o  o  o  o|          Sealed: (1,1)(1,2)(2,1)(2,2)
+--+--+--+--+--+--+                  (3,1)(3,4)(4,1)(4,4)
```

Coordinates are (x, y) from (0,0) at bottom-left, y up — same convention as
the test8 transcription in test_route.cpp. Two 900 mm straights (west,
bottom) carry the L1 work (strim, accel/brake, the tau fit).

## rev C — bert's wall move (build after rev A is proven)

One wall: (2,4)'s EAST wall relocates to (2,5)'s east. The tooth-exit corner
drops to (2,4), (2,5) becomes an open alcove over the corner's outside, the
top straight moves to row 4, and (5,4)/(5,3) merge into a U-RIGHT:

```
+--+--+--+--+--+--+          CW lap: 2x U-R, 2x U-L,
| o  o| a| .| .| .|            1x R73-R at (2,4), 3x R84-R.
+  +  +  +--+--+--+          U hands BALANCED every lap;
| o| o| o  o  o  o|          R73 joins the table (the U exit
+  +  +  +--+--+  +          setback 85.8 + T73 89.9 = 175.7
| o| o  o| o  o  o|          fits the 180 gap; R84's 98.6
+  +--+--+  +--+--+          does not).
| o| .| .| o  o  o|
+  +--+--+--+--+  +          a = alcove, open south: a full
| o| .| .| .| .| o|          cell of outer clearance at the
| o  o  o  o  o  o|          corner after two consecutive Us
+--+--+--+--+--+--+          -- the highest-error point.
```

26 cells, ~4.7 m. (2,4) flies ungated CW (inside wall ends 180 early;
inherits the tooth U's anchor); CCW gating unaffected.

## Search discovery procedure (before lap work)

The circuit has no natural goal. To map the FULL loop with the standard
hand-start search: set goal (5,5) — the exact antipode of (0,0) along the
circuit, 14 cells each way — wave, let it search out and return. If the far
arm is still unknown afterwards (`M` to check: the two arms are equal length,
so the return may retrace), move the goal to the middle of the unknown arm
and wave again. Two searches guarantee the whole loop. Then `map=save`.

## D6 — the diagonal rig (later phase, separate wall build)

Perimeter west/top + a 3-step 45-degree staircase from ~(2,0) to (5,3):
764 mm of diagonal line, 45-in/45-out each lap, hands mirror with direction.
Lane math: usable half-lane between staircase posts is 55 mm against our
35 mm half-width — 20 mm per side at perfect centring, which is why the
diagonal phase comes AFTER the corner table and the tau fix, not alongside.
