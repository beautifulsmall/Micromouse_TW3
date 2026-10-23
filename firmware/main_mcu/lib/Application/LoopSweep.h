// LoopSweep -- an autonomous corner-parameter experiment on a closed ring.
//
// THE RIG. A one-cell-wide corridor round the perimeter of a 6x6 maze:
// 20 cells, four 90-degree corners, corner centres five cells (900 mm)
// apart, so four cells of straight between arcs. Walls on BOTH sides all
// the way round (outer boundary plus a walled 4x4 block in the middle), so
// side-beam data is continuous and the lateral offset is always observed.
//
// WHAT IT MEASURES. For each (radius, arc speed, slew rail) it flies one
// loop -- four corners, four repeats -- and records, per corner: the
// lateral offset going in, the lateral offset coming out, the yaw error
// coming out, what the wall-edge gate saw, and how close the body came to
// either wall inside the arc. The question it answers is bert's: at what
// combination of radius and speed does the mouse come OUT of a corner
// centred?
//
// The two exit signatures mean different things and the record keeps them
// apart:
//   exit yaw err != 0, exit lateral ~ 0  -> the rotation did not complete
//                                           -> `track` / `tasym`
//   exit yaw err ~ 0, exit lateral != 0  -> the PATH was displaced
//                                           -> this is the R/v matching
//
// WHY THE SLEW RAIL IS A SWEPT AXIS. Exit displacement has two causes and
// both scale as v^2/R: understeer (grip) and the curvature step at the
// tangent point (reference trackability). They are COLLINEAR in the (R, v)
// plane, so sweeping radius and speed alone cannot separate them. The slew
// rail can: it divides the second and leaves the first alone. Run the grid
// at two rails and the answer to ADR-002's open question falls out --
// error moves with the rail -> clothoids will help; error does not ->
// it is grip and they will not.
//
// ODOMETRY THEN EDGE DETECT (bert, 2026-08-18). Odometry is good for the
// first three cells out of a corner; the gate is armed only for the last
// stretch of the approach. Arming late keeps the detector away from the
// beam-sweep region just after the previous arc, where false drops live.
//
// AUTONOMY. Wave to start; it flies the whole grid and stops. Grid progress
// lives in FRAM, so a flat battery means swap, wave, and it carries on at
// the combo it reached. Guards abort on |lateral| over limit, a run of
// skipped gates, heading error, or the loop/time caps -- and an abort still
// leaves every record written so far.

#pragma once

#include <Arduino.h>
#include "Fram.h"

// -- The rig ------------------------------------------------------------------
constexpr float SWEEP_CELL_MM = 180.0f;
// Corner point to corner point along one side of the ring: 5 cells for a
// 6x6 perimeter.
constexpr float SWEEP_SIDE_MM = 5.0f * SWEEP_CELL_MM;
// Where the mouse is placed on the first side, in cells before the first
// corner point. 2 cells leaves 3 cells of run-down at the end of the loop.
constexpr float SWEEP_FIRST_LEG_CELLS = 2.0f;

// -- THE COMPILED-IN SESSION --------------------------------------------------
// Everything the sweep needs is here, so the workflow is: flash, place the
// mouse on a side of the ring, wave. No terminal, no settings, nothing to
// remember at the maze. The console commands still exist (COMMANDS.md) for
// when you are tethered and want to try something, but they are never
// REQUIRED and every boot returns to exactly what is compiled below.
//
// That is deliberate. A session that depends on six typed commands is a
// session that silently runs with yesterday's typo in it, and the mouse
// cannot tell you which numbers it actually used -- so the numbers live in
// the firmware, and the firmware writes them into the log header.

// Arm the SWEEP at power-on instead of the search. Exactly one mission may
// own the hand start: both watch the same front beam, and both arming would
// make a wave ambiguous.
//
// FALSE from v28: the wave now starts a SEARCH. The loop sweep has answered
// what it was built to answer -- see SWEEP_VERSION -- and the programme has
// moved to bert's three phases: solve the maze slowly, run it slowly while
// recording, then run it fast against that recording.
//
// This flag is why run 138 flew a loop test when a search was asked for. It
// is a one-line switch back if the ring is ever wanted again, and the ring's
// configuration below is left intact and correct for that.
//
// FALSE again from v32 (bert, 2026-08-23): the acceleration axis is answered
// (run 153, see SWEEP_DEF_A_LON) and the programme goes back to phase 1 --
// map the maze, then gather over it. The wave starts a SEARCH; the sweep is
// started deliberately with 'sweep.arm' once the map is in FRAM.
constexpr bool SWEEP_ARM_AT_BOOT = false;

// Ring direction. false = clockwise = all RIGHT turns (inner block on the
// mouse's right); true = anticlockwise = all LEFT turns.
//
// ONE RING GIVES ONE TURN DIRECTION, and the CW/CCW asymmetry that `tasym`
// corrects is a first-order effect -- so a complete experiment is this
// session flown twice, flipping this line between them. Flip it, reflash,
// and the mouse will ask for the 5 s hold before it discards the first
// set (see the wipe gesture below), so the data cannot be lost by accident.
//
// RIGHT (v22) -- and this one has a PRE-REGISTERED PREDICTION, which is new.
//
// Runs 141 and 142 flew dir=L on v20/v21: both completed 20/20 combos with
// 52/52 gate lock and no failures, and 142's launch rows produced the first
// gyro number this rig has ever stated:
//
//     frame_corr = -0.708 +/- 0.135 deg per lap,  negative on 12 of 12
//
// That is the fused frame rotating against the corridor over one lap, with
// squaring zeroing it at the start of each. It is real (t = 5.2) and it is
// unexplained. Two mechanisms fit, and they are told apart by DIRECTION:
//
//     BIAS   accumulates with TIME.  A clockwise lap takes the same time as
//            an anticlockwise one, so frame_corr KEEPS ITS SIGN: negative.
//     SCALE  accumulates with ANGLE TURNED. A clockwise lap turns -360
//            instead of +360, so frame_corr FLIPS: positive, +0.708.
//
// ==> RUN 143 ANSWERED IT: SCALE. <==
//
//     dir=L (142)   frame_corr  -0.708 +/- 0.135    0 of 12 positive
//     dir=R (143)   frame_corr  +0.679 +/- 0.208    8 of  9 positive
//
//     bias predicted -0.708 (same sign); scale predicted +0.708 (flipped).
//     Observed +0.679 -- the sign flipped and the magnitudes agree to 4 %.
//
// The gyro OVER-READS rotation by 0.193 %. Note where that has to live:
// ImuIcm45686.h divides raw by 8.2, while +/-4000 dps over int16 is 8.192,
// so the DRIVER already under-reads by 0.098 % -- correcting that constant
// alone makes this worse, not better. The part's own scale factor is out by
// about +0.29 %, which is ordinary for a MEMS gyro and is exactly the kind
// of thing only a rig like this can measure. The fix is a measured scale
// correction, not a datasheet one.
//
// The original reasoning, kept because it is the reason the test worked:
//
// Write the prediction down before flying it. If it comes back negative the
// gyro has a bias of about 0.09 deg/s and the fix is a better still-detect
// and re-zero; if positive, the scale factor is out by 0.197 % and the fix
// is a scale calibration -- and bert's figure-of-eight rig, whose whole
// argument is that net-zero turning per lap cancels scale error, becomes
// worth building. If it comes back near zero, something about the direction
// itself is involved and both mechanisms are wrong.
//
// The within-run test was tried first and could not decide it: the v500 laps
// are ~20 % longer in time than the v600 laps, and the difference came out
// +0.32 +/- 0.27 deg (t = 1.2). The fixed inter-lap stop dilutes the time
// axis, which is why direction is the instrument and not speed.
//
// SECOND QUESTION, free with the same session: (L-R)/2 at tasym=0 is the
// PHYSICAL CW/CCW asymmetry, owed since v15. It needs both directions over
// the SAME FOUR PHYSICAL CORNERS -- which is why it is this ring flown the
// other way and not a rig with two loops in it. A rig whose left and right
// corners are different pieces of hand-built wall gives back the confound
// the pair exists to remove. Runs 141/142 are the dir=L half and they are
// unusually clean halves to have.
//
// THIRD, and watch for it: run 142's arc_min read a FALSE 40 at corner 2 and
// a false 38-40 at corner 3, on every lap, unmoved by radius -- while run
// 141 was clean at the same corners. Flown clockwise those are different
// beams at different angles, so whether the 40s travel with the corner or
// with the beam is answered for nothing.
//
// LEFT (v24) -- the stripped-wall approaches.
//
// Two inner walls are out. Clockwise they sat on the turn EXITS, so run 147
// saw four intact approaches and every F row came back runs_seen = 1: no
// post data at all, but 50 walls of known length, which is what measured the
// sample density above. Anticlockwise puts the same two gaps on the
// APPROACHES, where the observer is looking.
//
// EXPECT TWO CORNERS TO GO UNGATED, or possibly not -- see the packet-rate
// note: a bare post may give 15-18 packets and clear present_n after all.
// Either answer is the measurement. An ungated corner costs a bounded ~18 mm
// with no latch (runs 130-132), and the two intact corners still anchor the
// lap, so nothing here needs the gate to succeed.
//
// Place the mouse with the inner block on its LEFT. The stored clockwise set
// is a different grid, so the mouse will fast-blink both LEDs and ignore a
// wave until the old data is dumped and the hand held for 5 s -- dump run
// 147 before flashing this.
constexpr bool SWEEP_DIR_LEFT_DEFAULT = false;

// The grid. 5 radii x 5 arc speeds x 2 slew rails = 50 loops, ~6 minutes of
// driving, ~400 KB of traces. Sized to one battery.
// LARGEST RADIUS FIRST. The slew rail has to ramp the commanded rate from 0
// to v/R at the tangent point, and that ramp costs a fixed TIME -- so as a
// fraction of the arc it is worst at small R and high v. At R45/v200/slew
// 2500 the ramp is 29% of the whole arc; at R45/v300 it is 65%, which is not
// a corner at all, it is a continuous ramp.
//
// Run 116 opened on R45/v200/slew2500 -- the single worst cell in the grid --
// tripped the lateral guard at the corner exit, and (as the firmware then
// stood) ended the session. Working down from the largest radius means the
// mouse banks the benign combinations first and reaches the marginal ones
// with 40 loops of data already in FRAM. 'sweep' prints the ramp fraction
// for every cell so you can see which ones are expected to be ugly.
// R84 -- THE MAXIMUM-CLEARANCE RADIUS, and the loop sweep's last word on
// radius. The arc centre sits at (R, R) from where the two centrelines cross
// and the inside post sits at (84, 84), so at R = 84 the arc is centred ON
// THE POST and holds 84 mm from it the whole way round. Clearance by radius:
//
//     R125 67.0   R100 77.4   R84 84.0   R80 74.3   R60 26.1   R45 10.2
//
// Body half-width is 35, so R84 leaves 49 mm of margin -- the most any single
// arc can. R60 and R45 are below the 63.7 mm geometric minimum and were only
// ever measuring the post.
//
// AND THE POST GATE CAN TRIGGER IT, which is the other half of bert's
// requirement (2026-08-22). At v500 on the fast rail:
//
//     clothoid 28.4 mm (rail-limited, room allows 58.8 -- not clipped)
//     tangent  98.6 mm
//     gate room 36.4 mm  against SWEEP_GATE_ROOM_MIN_MM = 20
//     ramp     0.22 of the arc against SWEEP_MAX_RAMP_FRAC = 0.75
//
// Every one of those has margin, and unlike R125 -- where clothoid_for_room
// returns 0 and the corner flies a bare arc -- R84 gets a full rail-sized
// transition. It is the best corner this mouse can fly, on every axis at once.
constexpr uint8_t SWEEP_DEF_R[] = {84};
// 500 and 600 ONLY. The axis has been cut from the bottom twice now, each
// time for the same reason: the slow end resolves nothing.
//
// Runs 116-119 swept 200..400 in steps of 50 and never reached a speed the
// mouse struggled at. Runs 120-129 swept 200..600 in steps of 100, and the
// answer came back the other way round -- the FAST cells were the clean ones.
// Run 128's mean exit_lat is monotonic in v across every radius, and it
// crosses zero somewhere around 400:
//
//     v200  +20.2 (R125) ... +8 (R80)     turning too late, exits outside
//     v500   -0.8 (R125)  -5.5 (R100)     the tightest column in the run
//     v600                                bert: "at 600 it went very clean"
//
// and run 129 ended on a v200 corner. So the low-v rows were not merely
// uninformative, they were actively the worst-behaved, and they were eating
// three fifths of every session's FRAM and floor time to say so. bert,
// 2026-08-21: "the lower velocity settings for the loop tests are not needed,
// maybe set a min velocity at 500mm/s, the mouse has proven it can corner at
// these speeds."
//
// WHY THE SLOW CELLS EXIT WIDE is worth stating, because it is the trim and
// not the tyres. SWEEP_ARC_TRIM_MM is a single constant tuned on run 122's
// mix; the placement error it corrects scales with how far the mouse travels
// during the gate's detection lag, and that distance is proportional to v.
// One constant can therefore only be right at one speed. Cutting the axis to
// 500/600 narrows the span the constant has to cover from ~40 mm to ~10 mm,
// which is inside the noise -- so this change also buys accuracy, not just
// time. Fitting trim as a function of (R, v) is still the real answer and is
// still open.
//
// The grid is now 5 R x 2 v x 2 slew = 20 combos. Seven are declined by the
// ramp rule (slew 2500: R80/v600, both R60, both R45; slew 6000: both R45),
// leaving 13 flown -- about the same session length as before at four times
// the data density in the range that matters.
//
// One consequence to keep in mind when reading the log: the straight speed is
// max(v_arc, 400), so the APPROACH is now always at arc speed, and the edge
// gate sees fewer packets per mm (about 6 mm per sample at 600 against 4 at
// 400). The gate needs 2-3 samples to confirm, so its detection lag is ~15-18
// mm everywhere now rather than varying down the column. That uniformity is
// the point. Watch gate_corr and gate_drop_s before trusting anything else.
// 500 for data gathering (bert, 2026-08-22): "the mouse can corner inside a
// cell after seeing a post at 600 mm/s", so the cornering question is closed
// and speed is no longer an axis. What these runs are for now is WALL AND
// POST DATA, and a second speed would only halve the laps at each layout.
//
// One caveat to carry forward: sample density scales with speed -- 0.677
// mm/packet at v500, 0.792 at v600. Anything the library stores about a
// feature must be in MILLIMETRES, not packets, or it will not transfer from a
// slow gathering run to a fast one.
// v32: 500, as the gathering plan said all along. 700 was borrowed for the
// acceleration test and there is no longer a reason to hold it.
constexpr uint16_t SWEEP_DEF_V[] = {500};

// -- THE CORRIDOR SWEEP (v29) -------------------------------------------------
//
// Every session up to 148 flew a CONSTANT-SPEED CRUISE. `set_limits` was
// called with v_max = the arc speed and a_lon = PATH_A_LON_DEFAULT = 700
// mm/s^2, so the mouse never went faster on a straight than it did round a
// corner, and it never asked the tyres for more than 0.07 g longitudinally.
// Thirty sessions of cornering data and not one number about acceleration.
//
// This sweeps the corridor instead. R84 and v_arc 700 are FIXED at the best
// corner the mouse has (0.595 g lateral, comfortably inside a guessed
// mu = 0.8), v_max is lifted clear so it never binds, and a_lon is the axis.
//
// THE STRAIGHT IS 703 mm: 900 between corner points, less 2 x 98.6 of
// tangent. Accelerating over half of it from 700 mm/s:
//
//     a_lon      g       peak on the straight
//       700    0.07       991 mm/s   <- the baseline every past session flew
//      2000    0.20      1376
//      4000    0.41      1816
//      6000    0.61      2170
//      8000    0.82      2470       <- about the mu = 0.8 guess
//     10000    1.02      2740       <- deliberately past it
//
// The grid must CONTAIN the limit or it cannot find it, which is the same
// argument the radius grid used to make before it became a gathering rig.
//
// AND THE INSTRUMENT ALREADY EXISTS. Wheel slip under acceleration makes
// odometry run ahead of truth, and the wall gate measures exactly that
// against a landmark at every corner. gate_corr IS the longitudinal slip
// meter: if it grows with a_lon, that is the traction limit, measured. The
// lateral and beam guards are unchanged and have thirty sessions on them.
// RUN 153 ANSWERED THIS, AND THE ANSWER IS MUCH LOWER THAN THE FRICTION
// GUESS. The axis was {700, 2000, 4000, 6000, 8000, 10000} and it got three
// loops in:
//
//   a_lon  700   four clean corners, wheel_minus_s 0.0
//   a_lon 2000   failed on the lateral at s=503, wheel_minus_s +27.4 mm
//   a_lon 4000   failed at s=211, wheel_minus_s +8.8 mm by then
//
// The instrument worked exactly as the note below predicted: odometry ran
// AHEAD of the path, by 27 mm in half a lap. At 4000 the trace reads
// v = 1336 mm/s at s = 139, where sqrt(2*a*s) allows at most 1054 -- the
// wheels were turning a quarter faster than the mouse was moving. The mouse
// then steers on a lie: yaw_err climbs monotonically 2.4 -> 12.0 deg over
// those same 139 mm and the lateral guard trips.
//
// So the binding limit is NOT the friction circle. mu ~ 0.8 would allow
// 7848 mm/s^2; the mouse breaks somewhere between 700 and 2000, a tenth of
// that. Longitudinal traction is worse than lateral because the drive torque
// and the weight transfer both act on the same two wheels, and because
// nothing here closes the loop on slip -- the odometry that would detect it
// is the thing that is wrong.
//
// 700 until slip compensation exists. The axis is kept in the comment above
// so re-running it is a one-line edit once there is something to measure.
constexpr uint16_t SWEEP_DEF_A_LON[] = {700};

// v32: FLAT AT THE ARC SPEED. 3200 existed to let a_lon be the only binding
// limit while acceleration was the subject. It is not the subject any more --
// wall and post signatures are -- and a corridor that ramps changes sample
// density along the very corridor being characterised (0.677 mm/packet at
// v500 against 0.53 at v640). A gathering lap should read the same at both
// ends of the straight, so the whole lap flies at SWEEP_DEF_V.
constexpr float SWEEP_STRAIGHT_V_MAX = 500.0f;
// The two rails that separate grip from trackability (see the note at the
// top of this file). 2500 is a lazy entry, 6000 a sharp one.
// Fast rail only. The rail A/B is answered (runs 136-137) and on the lazy
// rail the room budget CLIPS the clothoid, which is a nuisance variable in
// data whose subject is the walls rather than the corner.
constexpr uint16_t SWEEP_DEF_SLEW[] = {6000};

// One cell, flown this many times. With a single R, v and rail the grid is
// one combo, and what a gathering session wants is laps over a maze layout,
// not cells. 16 is a little more than the 13 a battery has been delivering,
// and progress lives in FRAM -- so a flat cell means swap, wave, carry on.
// v32: one combo, so reps ARE the session. 12 laps, which is what a battery
// has been delivering, and progress lives in FRAM -- a flat cell means swap,
// wave, carry on.
constexpr uint8_t SWEEP_DEF_REPS = 12;
// Trace sampling, in mm of path travelled. 0 disables tracing.
constexpr float SWEEP_DEF_TRACE_MM = 4.0f;

// HOW MUCH OF AN ARC MAY BE SPENT RAMPING. The rail takes T = omega/alpha to
// bring the commanded rate to v/R, and in that time the mouse covers
// v^2/(R*alpha) of a v*pi*R/2 arc. As a fraction:
//
//     ramp / arc = 2 v^2 / (pi R^2 alpha)
//
// Above 1.0 the commanded rate is NEVER reached before the arc ends: the
// mouse is asked for a corner it is not physically being told how to drive,
// and it runs wide into the outer wall. That is not a measurement, it is a
// crash with a plausible-looking log line.
//
// Run 124 is the evidence. It reached the small radii for the first time and
// the measured shortfall in arc_turn tracked the ramp prediction closely
// wherever the fraction was small (R125/v200: predicted 1.7 deg, measured
// 1.7; R80/v300: predicted 9.2, measured 9.1). Then combo 14 -- R45 at
// v400, ramp/arc = 1.15 -- tripped the yaw guard at 25 degrees of error a
// third of the way through the arc and hit the outer wall.
//
// So the grid now declines those cells rather than discovering them with the
// bodywork. They are logged as SWEEP_FAIL_RAMP, which is a RESULT: "this rail
// cannot deliver this corner" is exactly what the slew axis exists to find
// out. At 0.75 the rule excludes R45/v400 and everything worse while keeping
// every cell that has actually flown (R60/v400 at 0.65 is the closest
// survivor), and at slew 6000 nearly all of them come back -- which is the
// comparison the experiment wants.
constexpr float SWEEP_MAX_RAMP_FRAC = 0.75f;

// ARC-PLACEMENT TRIM. An extra along-path offset added to the arc start on
// top of the edge gate's correction, spread over the same room, applied to
// EVERY corner.
//
// POSITIVE STARTS THE ARC EARLIER. control_path_correct() adds to path_bias,
// s = (d_est - d0) + path_bias, so raising s brings the tangent point's s
// value forward and the mouse turns sooner. Getting this backwards costs a
// battery, so: +25 turns 25 mm earlier and pulls the exit 25 mm INWARD.
//
// WHY A FIXED OFFSET NOW, AND NOT THE CROSSOVER IT REPLACES.
// Runs 120 and 121 flew a -20 mm trim alternating on (loop + corner) parity,
// to beat the session drift and the hand-built ring's per-corner bias at the
// same time. It answered its question on the second corner it ever flew.
// Run 121, from the traces, measuring the inside wall's end to the first
// in_arc sample:
//
//       corner 0, clean   arc 38 mm after the wall end (nominal 37)
//                         exit: 63 mm from the outside wall
//       corner 1, -20 mm  arc 59 mm after the wall end
//                         exit: 42 mm from the outside wall
//
// 21 mm of exit for 20 mm of trim, on the outside beam alone so no
// corridor-sum ambiguity. ARC PLACEMENT MOVES THE EXIT ONE FOR ONE. That is
// no longer a hypothesis and does not need testing again.
//
// What it leaves is an equivalence. The clean corner starts its arc where the
// geometry says it should -- within about a millimetre of the wall end, in
// runs 119 and 121 both -- and still exits ~25 mm outside. With a 1:1
// response, "the mouse turns 25 mm too late" and "the gate's idea of where
// the corner is, is 25 mm out" are THE SAME STATEMENT. The gate's expectation
// is corner_entry_s + R - 90 - WALL_GATE_BEAM_AHEAD_MM, and that 72 mm
// look-ahead has never been measured independently.
//
// So this run applies the correction to every corner and asks one question:
//   exits fall to ~0   -> the standing 25 mm is ONE WRONG CONSTANT in the
//                         gate geometry. Fold it into the look-ahead and the
//                         experiment can finally get on with radius and speed.
//   exits unchanged    -> placement is a lever that works but is not the
//                         cause, and the search moves elsewhere.
//
// EXITS FELL TO ~0 (run 122, +1.6 +/- 0.9 mm), so it was one wrong constant,
// and v14 finally folds it into the look-ahead where it belongs.
// WALL_GATE_BEAM_AHEAD_MM has gone 72 -> 45. Read the long note there for the
// three independent measurements behind that.
//
// THIS TRIM AND THAT CONSTANT ARE THE SAME LEVER -- the code applies
// `control_path_correct(corr + SWEEP_ARC_TRIM_MM, ...)`, so they simply add.
// Which is why the two MUST move together, and why the pair is arithmetic,
// not tuning:
//
//     before   corr ~ -24.5 (gated mean) + 25.0 trim  =  +0.5 applied
//     after    corr ~  +2.5              -  2.0 trim  =  +0.5 applied
//
// Placement is unchanged BY CONSTRUCTION. Gated corners were exiting at
// +1.5 mm and should keep doing exactly that. Nothing about this pair is
// meant to make a good corner better.
//
// What it buys is the corners that were never gated at all. With the centre
// moved onto zero and the window widened to 45, the 21-of-49 that fell
// outside +/-35 should now land inside it -- and an ungated corner is not a
// slightly worse corner, it is a -18 mm one. That is the entire prize, and
// the measure of success in run 133 is the SKIP RATE, not the exit.
//
// If exits move materially away from +1.5, the pair was not neutral and the
// arithmetic above is where to look first.
//
// Set to 0 to disable the trim and fly the plain grid.
// -2 -> +11 with WALL_GATE_BEAM_AHEAD_MM 45 -> 58 (2026-08-30). In the ring
// rig the trim and the look-ahead ADD, so only their DIFFERENCE places the
// arc: the run-122 tuning is (trim - look_ahead) = -47, and moving the
// look-ahead alone would silently re-place every ring corner by 13 mm.
// test_sweep pins the difference for exactly this reason.
constexpr float SWEEP_ARC_TRIM_MM = 11.0f;

// SESSION GAIN OVERRIDE: turn-asymmetry feedforward.
//
// Negative = leave whatever is in FRAM alone. Any value >= 0 is APPLIED TO THE
// LIVE GAINS when the sweep arms, in RAM only -- it is never persisted, so a
// power cycle restores the stored value and nothing here can quietly become
// the mouse's permanent tuning.
//
// WHY THIS EXISTS AT ALL. `tasym` lives in FRAM and OVERRIDES the compiled
// VCTRL_TASYM, so editing that constant and reflashing changes nothing on a
// mouse that has ever had params saved -- run 133 flew with tasym=0.200 while
// the constant said 0.10. The alternative is asking for two typed commands
// before every session, and this file's whole premise (see SWEEP_ARM_AT_BOOT)
// is that a session which depends on typed settings is a session that
// silently runs with yesterday's typo in it. Compiled in, it also lands in
// the dump's gains line, so the data says what actually flew.
//
// WHAT IT IS FOR RIGHT NOW. Control.cpp applies
//
//     dv_ff = w*ht ;  asym = tasym*|w|*ht ;  dv_ff += (w>=0) ? asym : 0.5*asym
//
// so at tasym=0.200 the commanded differential is x1.200 on LEFT turns and
// x0.900 on RIGHT -- a 15 % direction spread, open loop, scaling with |w|.
// Pairing run 133 (dir=L) against runs 130-132 (dir=R) cell by cell:
//
//     R      common mode        asymmetry (L-R)/2
//     R125     -2.15 deg             +0.22
//     R100     -2.30                 +0.81
//     R80      -2.53                 +1.32
//     R60      -3.74                 +3.00
//
// The common mode is FLAT in R -- that is the arc-window artefact. Every bit
// of the radius dependence is in the asymmetry, and against turn rate it is a
// line: +0.00915 deg per deg/s, r = 0.74 over 13 paired cells. Rate-
// proportional and direction-antisymmetric has exactly one owner, and this is
// it. Left over-turns, right under-turns, worse the tighter and faster the
// corner. At R60/v600 that is 5.2 deg of extra rotation, which is what put
// the mouse into the inside wall.
//
// The value came from a sequential procedure recorded in Bench.cpp: "raise
// until 'kl' exit yaw_err ~ 0, then check 'kr' stays near 0". Raising tasym
// drives left to zero and pushes right away, and the procedure never goes
// back -- which is how 0.200 happened, and why the 0.5 weighting on right
// turns is a second patch on the same wound.
//
// SO: 0 for the next pair of sessions. That does not assert the physical
// asymmetry is zero -- it removes the KNOWN asymmetric term so that whatever
// (L-R)/2 survives is the real one, measured on arcs for the first time and
// fittable jointly instead of sequentially. Set tasym from THAT, not from
// this. Set back to -1 to hand control to FRAM again.
constexpr float SWEEP_TASYM_OVERRIDE = 0.0f;

// SESSION GAIN OVERRIDE: the angular feedforward, as a SCALE on track_mm.
// 1.0 = leave FRAM alone. Same rules as the tasym override -- applied to the
// live gains at arm time, RAM only, never persisted, and it lands in the
// dump's gains line so the data says what flew.
//
// WHY track_mm IS THE RIGHT KNOB. bert asked for "a x0.9 correction applied
// to the feed forward angular acceleration or velocity". Both of those terms
// are built from the same half-track:
//
//     dv_ff = omega * half_track          (Control.cpp:588, angular VELOCITY)
//     a_ang = alpha * half_track          (Control.cpp:606, angular ACCEL)
//
// and a grep says track_mm appears NOWHERE else in the control path -- not in
// odometry, not in the fusion, not in any measurement. It is a pure command
// scale. So x0.9 on the feedforward IS track_mm x 0.9, exactly, for both
// terms at once, with no new gain and no FRAM layout change. 72.31 -> 65.08.
//
// (The same half-track also scales the in-place turn feedforward at
// Control.cpp:465-498. A sweep session never turns in place, so for this
// experiment that is moot -- but do not persist this value, because a search
// run WOULD feel it.)
//
// WHAT IT IS EXPECTED TO DO, so run 135 can falsify it. The exit overshoot
// measured over runs 130-133 is about 60 % of the open-loop ramp prediction
// omega^2/(2*alpha), which means the yaw loop is already absorbing ~40 %.
// Taking the feedforward down 10 % should cut the FF-driven part of the
// rotation by 10 % and let feedback recover more of the rest, so:
//
//     PREDICT exit overshoot at R60/v500/slew6000 falls 13.7 deg -> 10-12
//     PREDICT total turn still settles at 90 (feedback owns steady state)
//     PREDICT the ENTRY rate deficit gets slightly WORSE -- the first fifth
//             of an R60 arc already runs at 0.23 of commanded
//
// If the overshoot does not move, the feedforward is not what is driving it
// and the curvature step is the whole story -- which is the clothoid case.
constexpr float SWEEP_TRACK_SCALE = 1.0f;

// CLOTHOID CORNERS. false = the constant-radius geometry runs 116-135 flew.
//
// Turned on because three independent lines of evidence converged on the
// curvature step at the tangent point, and nothing else survived:
//
//   1. The overshoot has the closed form of a rate ramp. Measured
//      straight-to-straight it is ~60 % of omega^2/(2*alpha) in every cell.
//   2. The RAIL A/B, which ADR-002 specified in advance as the deciding
//      test: R80/v500 overshoots 11.86 deg at slew 2500 and 6.18 at 6000.
//      Halve the ramp, halve the error -- so it is the reference, not grip.
//   3. Both alternatives were falsified on the floor. Grip: |ay|/a_dem
//      never saturates and differs by turn DIRECTION at identical demand.
//      Feedforward magnitude: track x0.90 over runs 134 and 135 (a clean
//      repeat) moved the overshoot +0.31 deg -- the wrong way -- because
//      kyaw just took up the slack.
//
// Run 135 is the baseline to beat: the first complete grid, 52 corners,
// zero guard trips, 100 % gate lock, mean |exit_lat| 6.4 mm. THE PREDICTION
// is that clothoids collapse the slew-2500 column onto the slew-6000 one --
// if the reference never asks for a step, the rail stops mattering, and the
// -7 mm bias that only slew 2500 shows should go with it.
constexpr bool SWEEP_CLOTHOID = true;

// Corridor the gate must keep between its firing point and the tangent, for
// the correction to be spread over rather than applied as a step. This is
// what bounds how long a clothoid may be -- see the sizing note in
// loop_launch().
constexpr float SWEEP_GATE_ROOM_MIN_MM = 20.0f;

// Hold the hand in front for this long, instead of waving, to DISCARD the
// stored session and start a fresh one. Same vocabulary as the search's map
// wipe: a short gesture is the benign outcome, the destructive one has to be
// deliberate. Without a terminal this is the only way to say "yes, throw the
// old data away", so the mouse refuses to do it on its own.
constexpr uint32_t SWEEP_WIPE_HOLD_MS = 5000;

// -- Grid ---------------------------------------------------------------------
constexpr int SWEEP_MAX_R = 8;
constexpr int SWEEP_MAX_V = 8;
constexpr int SWEEP_MAX_SLEW = 4;
constexpr int SWEEP_MAX_ALON = 8;

// A pure, testable description of the parameter grid and where the session
// has got to in it. Combo order is (slew slowest, then radius, then speed):
// consecutive loops therefore differ only in speed, which keeps the biggest
// mechanical change -- the radius -- as infrequent as possible.
struct SweepGrid {
    uint8_t n_r = 0, n_v = 0, n_slew = 0;
    // How many times the whole (R, v, slew) sweep is flown. 1 for a
    // parameter sweep, where every cell is a different question. More for
    // DATA GATHERING, where every cell is the same question asked of a
    // different piece of maze and what is wanted is laps.
    uint8_t n_rep = 1;
    uint8_t n_alon = 1;
    uint8_t r_mm[SWEEP_MAX_R] = {0};
    uint16_t v_arc[SWEEP_MAX_V] = {0};
    uint16_t slew[SWEEP_MAX_SLEW] = {0};
    // LONGITUDINAL acceleration, mm/s^2. The fourth axis, added in v29.
    // Everything before it swept the CORNER; this sweeps the CORRIDOR, and
    // until now the corridor was never swept at all -- v_max was pinned to
    // the arc speed and a_lon to PATH_A_LON_DEFAULT = 700 mm/s^2, so every
    // lap of every session was a constant-speed cruise at 0.07 g.
    uint16_t a_lon[SWEEP_MAX_ALON] = {0};

    int combos() const {
        return (int)n_r * (int)n_v * (int)n_slew * (int)(n_alon ? n_alon : 1) *
               (int)(n_rep ? n_rep : 1);
    }
    bool valid() const {
        return n_r && n_v && n_slew && n_alon && n_rep && combos() <= 512;
    }

    // Decompose a combo index. Returns false if out of range.
    //
    // RADIUS varies fastest, then speed, then the slew rail (bert,
    // 2026-08-20). It was speed-fastest for runs 116-119 and that was wrong
    // for two reasons, both learned the hard way:
    //
    //   COVERAGE. No session has yet survived past loop 8 of 50 -- a guard
    //   trip needs a human to pick the mouse up and wave -- so with radius
    //   moving once every five loops, R80, R60 and R45 have NEVER BEEN FLOWN.
    //   And the live question is whether the 25 mm exit offset scales with
    //   arc length, which runs 196 mm at R125 down to 71 mm at R45. The one
    //   axis that could answer it was the one the ordering buried. Now the
    //   first five loops of any battery sweep every radius.
    //
    //   CONFOUNDING. With speed fastest, loop index was a proxy for speed
    //   within each radius block -- and entry_lat drifted 17 mm across run
    //   119's session. Every apparent speed effect carried that drift.
    //   Radius-fastest does not remove drift, it just stops it aliasing onto
    //   the axis being read first.
    //
    // The slew rail still changes exactly once in the session, which is what
    // its A/B needs.
    //
    // Inline so it is host-testable without dragging in the HAL.
    bool at(int ix, float *radius_mm, float *v_arc_mm_s, float *slew_dps2,
            float *a_lon_mm_s2 = nullptr) const {
        if (!valid() || ix < 0 || ix >= combos()) return false;
        int r = ix % n_r;
        int v = (ix / n_r) % n_v;
        int s = (ix / ((int)n_v * (int)n_r)) % n_slew;
        int a = (ix / ((int)n_slew * (int)n_v * (int)n_r)) % n_alon;
        if (a_lon_mm_s2) *a_lon_mm_s2 = (float)a_lon[a];
        // Repeat is the SLOWEST axis, so a whole sweep completes before any
        // of it is flown twice -- a session cut short by a battery is then a
        // complete sweep with fewer repeats, not a partial one.
        if (radius_mm) *radius_mm = (float)r_mm[r];
        if (v_arc_mm_s) *v_arc_mm_s = (float)v_arc[v];
        if (slew_dps2) *slew_dps2 = (float)slew[s];
        return true;
    }
};

// -- FRAM layout --------------------------------------------------------------
// The sweep and the flight recorder SHARE the bulk region -- they are never
// both wanted (one instruments a speed run, the other a parameter sweep),
// and splitting 510 KB between two logs that are used months apart would
// halve both. Each writes its own magic; each dump refuses politely if the
// region belongs to the other.
constexpr uint32_t SWEEP_BASE = 0x00800;
// 256 -> 384 (2026-08-30): the header embeds a full ControlGains, and param
// 39 pushed it past 256. Headroom to ~70 params. Shifts SWEEP_SUM_BASE, so
// a sweep session stored by older firmware cannot be resumed or dumped on
// this one -- dump before reflashing across this line (route laps and the
// flight log are unaffected; they live in their own regions).
// 384 -> 512 (2026-09-26): param 71 (rturn) pushed it past 384. Headroom to
// ~100 params. Same consequence as last time: a sweep session stored by
// older firmware cannot be resumed or dumped on this one.
constexpr uint32_t SWEEP_HDR_BYTES = 512;
constexpr uint32_t SWEEP_SUM_BASE = SWEEP_BASE + SWEEP_HDR_BYTES;
// 72 KB = 2048 corner records of 36 B = 512 loops. The summaries ARE the
// experiment, so they get a region that a realistic session cannot overflow,
// and they never wrap. Traces are the diagnostic and may. (Was 64 KB at 32 B
// per record; the region grew with the record so the loop count did not
// shrink -- 8 KB off the trace ring costs about two corners of trace.)
constexpr uint32_t SWEEP_SUM_BYTES = 72UL * 1024;
constexpr uint32_t SWEEP_TRACE_BASE = SWEEP_SUM_BASE + SWEEP_SUM_BYTES;
constexpr uint32_t SWEEP_TRACE_END = FRAM_BULK_END;   // lap report above
constexpr uint32_t SWEEP_MAGIC = 0x54573357UL;  // 'TW3W'
// v3: SweepSummary grew arc_turn_cdeg (32 -> 36 B). A v2 log in FRAM will
// read as NEEDS_WIPE, which is the correct outcome -- the records would
// otherwise be decoded at the wrong stride.
// v4: the trace's lat_x10 became ay_mm_s2 (same 16 B) and the summary's
// reserved word became arc_ay_mm_s2. Same size, different meaning, so the
// version bump is what stops a v3 log being read as if it were this one.
// v5: odd corners now fly with SWEEP_ARC_TRIM_MM added to the arc start.
// The layout is unchanged, which is exactly why the version has to move --
// a v4 log would decode perfectly and mean something else.
// v6: the trim went from -20 mm on odd corners to +25 mm on EVERY corner.
// Same reason again: identical layout, different experiment.
// v7: loop-event rows (corner_ix == SWEEP_CORNER_EVENT) and corridor
// breadcrumbs now share the summary and trace regions with real corners. A v6
// reader would decode both as corners and believe them.
// v8: a loop-event row is now written on EVERY failure, not only when no
// corner record was open, and it carries the previous packet's lateral. The
// lateral guard also needs two consecutive packets.
// v9: combos whose slew ramp does not fit inside the arc are declined rather
// than flown (SWEEP_FAIL_RAMP), so a session's combo sequence is no longer
// the same as the grid's.
// v10: a rejected beam pair now resolves by the SIGN of the sum error rather
// than always taking the nearer wall, and reports SWEEP_LAT_SUSPECT. The
// lateral column means something different wherever that fires.
// v11: the edge gate is fed that same resolved lateral instead of the raw
// pair, and the header carries a beam-health count.
// v12: a corridor side reading below SWEEP_SIDE_MIN_MM is rejected as not a
// distance before anything consumes it, so lat_src and the lateral mean
// something different wherever that fires.
// v13: the edge gate SKIPS a sub-floor side packet rather than seeing it, so
// a stuck low beam can no longer capture track_mm_ and blind the step test
// (this is what made corner 3 skip its gate every lap in runs 127-129); and
// the velocity axis is 500/600 only.
// v14: the beam look-ahead is measured rather than calculated (72 -> 45), the
// arc trim moves with it to keep placement identical (+25 -> -2), the
// plausibility window widens to 45 now that its centre is right, and the ring
// is flown ANTICLOCKWISE for the first time.
// v15: the sweep can override a control gain for its own session (RAM only),
// and uses it to take the turn-asymmetry feedforward to zero. Back to dir=R
// for the first half of the pair.
// v16: the angular feedforward is scaled x0.90 for the session (track 72.31
// -> 65.08), against the exit heading overshoot measured over runs 130-133.
// v17: v16 FALSIFIED on a clean repeat (runs 134/135 moved the overshoot
// +0.31 deg, the wrong way) and reverted. Clothoid transitions instead --
// the rail A/B and both falsified alternatives all point at the curvature
// step. See ADR-002, "The clothoid decision".
// v18: dir=L, closing the tasym pair. No other change -- the clothoid, the
// gate constants and tasym=0 are exactly what runs 136 and 137 flew, so the
// only difference between this session and those is the turn direction.
// v19: circl/circr as measured by the 'j' differential are now compiled
// defaults AND saved to FRAM, so for the first time the wheel constants in
// flight are the measured ones. lat_max 25 -> 35 (WallGate.h): run 140's one
// failure was refused by 0.7 mm and the ungated corner cost the lap.
// v20: square_up() RETIRED. The between-loop in-place turn referenced the
// drifting fused yaw and moved the body to fix it; run 140's single firing
// was the only pose change at any of the eleven loop boundaries and it put
// the mouse 7 mm further off centre, which is the whole of why the last lap
// failed. Replaced by a corridor fit (WallAlign over each straight leg) that
// re-references the FRAME between loops and launches the next path on the
// corridor rather than on the body. Nothing turns in place any more.
// v21: launch rows ('Q,'). Run 141 completed 20/20 with 100 % gate lock and
// no pose change at any of its twelve loop boundaries -- and the corridor
// fit's eps, the reason to believe any of that, existed only on the console.
// Every loop now records what the squaring measured before it moves.
// v22: dir=R. Nothing else changes -- the clothoid, tasym 0, the gate
// constants, lat_max 35 and the squaring are exactly what runs 141 and 142
// flew, so the only difference between this session and those is the turn
// direction. That is what makes frame_corr's SIGN a clean test of bias
// against scale, and (L-R)/2 the physical asymmetry rather than a mix of
// asymmetry and geometry.
// v23: the FEATURE OBSERVER. Passive -- no decision changes. Every corner
// approach now records what the inside beam did as a list of RUNS (a wall is
// a long run, a post a short one, a gap the space between), written as an
// 'F,' row. And the gate's single 240 mm arming distance is split into
// OBSERVE (600) and DECIDE (240, unchanged), because counting features wants
// the opposite of a tight window.
//
// The rig for it: two inner walls removed, leaving a bare post before two of
// the four turns. Those corners will go UNGATED -- 12 mm of post cannot
// supply present_n = 12 packets of continuous wall -- and that is the correct
// behaviour, not a fault: an ungated corner is flown on odometry and costs a
// bounded ~18 mm (runs 130-132, 147 corners). The other two corners still
// anchor the lap. Nothing is being asked to work that has not been measured.
// v24: dir=L. The stripped walls sit on the turn EXITS clockwise (bert), so
// run 147 saw four intact approaches and every F row came back runs_seen=1.
// Flown anticlockwise the same two gaps land on the APPROACHES, which is
// where the observer is looking. Same firmware otherwise.
// v25: run-merging in the feature observer (WallGate::BRIDGE_MM = 8). Run
// 148 answered the post question and the answer was the opposite of what I
// predicted twice: a post gives 30.8 packets over 22.1 mm, not four over
// twelve. The step detector already gates on one -- 52/52 including both
// stripped corners, exits -6.2 and -1.0 against -0.5 intact. What needed
// fixing was only the COUNT: the raw stream splits a post into two or three
// pieces separated by at most 4 mm, and the count is what bert's gate is to
// be built on. Bridging at 8 mm collapses all 26 to exactly one.
// v26: the observer is unchained from the corner record. The gate is armed
// on DISTANCE (gate_arm_for) rather than inside corner_open, so it now sees
// SWEEP_GATE_OBSERVE_MM = 500 of approach instead of the record's 145 at
// R125. Decisions are unmoved: live_from_s_mm is held at the later of the
// decide distance and the record opening, so nothing decides before there is
// a record to write it into and every decision lands where runs 141-148 put
// it. Needed because bert is stripping more inner walls: counting posts is
// pointless through a window that only shows the last one.
// v27: the loop sweep stops being a parameter sweep. R84 (max clearance, and
// the post gate has 36 mm of room to trigger it), v500, fast rail, 16 laps of
// it. Every lap is now the same corner asked of a different piece of maze,
// which is what wall and post data gathering needs. SweepGrid gained n_rep.
// v28: the wave starts a SEARCH, not a sweep (SWEEP_ARM_AT_BOOT = false).
// v29: THE CORRIDOR SWEEP. R84 / v_arc 700 fixed, v_max lifted to 3200, and
// a_lon swept 700 -> 10000 (0.07 g -> 1.02 g). The first sweep this rig has
// ever run on the straights. dir=L. The wave arms the SWEEP again -- flip
// SWEEP_ARM_AT_BOOT back to false to return to searching.
// v30, after run 151 hit a wall at speed. Three things:
//   1. ABORTS NOW BRAKE. motors_stop() is zero duty and zero duty is COAST,
//      so every guard trip up to 151 handed a moving mouse to rolling
//      resistance. control_brake_to_stop() holds the wheel loop up with zero
//      targets first -- which is reverse duty, measured at -0.45 (the limit)
//      from 1148 mm/s in test_velocity_control.
//   2. The corridor squaring no longer carries a STALE bearing across a
//      junction. sq_corridor_yaw is absolute and the ring's legs are 90 deg
//      apart, so a fit latched on one leg and left standing when the next
//      fails is wrong by a right angle -- run 151 squared onto a cardinal
//      90 deg out (body_off 89.70) and only the launch clamp saved it.
//   3. a_lon axis unchanged; the sweep has not yet been past 2000.
// v31: dir=R. The maze is still the stripped one and the two sides are not
// alike -- posts and single wall ends on one, a wall/gap mix on the other --
// so which side the INSIDE beam looks at is set by the direction (bert).
// Flown left, run 152 spent its last 46 mm of loop 1 with the inside beam
// blind (255) and the guard reading the OUTSIDE beam pinned at 130.
// v32: the acceleration axis is ANSWERED and collapsed (run 153 -- see
//   SWEEP_DEF_A_LON: 700 clean, 2000 slips 27 mm, 4000 breaks away at
//   launch). The session goes back to what it was built for: one gathering
//   combo, R84 / v500 / a_lon 700, flat corridor, 12 laps. The wave starts a
//   SEARCH again (SWEEP_ARM_AT_BOOT false); the sweep is armed deliberately
//   once the map is in FRAM.
constexpr uint16_t SWEEP_VERSION = 32;

// One record per corner. 36 bytes, written the moment the corner completes,
// so a battery pulled mid-loop loses at most the corner in progress.
struct SweepSummary {
    uint16_t loop_ix;
    uint8_t corner_ix;   // 0..3 round the ring
    uint8_t flags;       // see SWEEP_F_*
    uint8_t r_mm;
    uint8_t slew_ix;
    uint16_t v_arc;      // commanded arc speed, mm/s
    int16_t entry_lat_x10;   // averaged just before the tangent point
    int16_t exit_lat_x10;    // averaged just after it
    int16_t entry_yaw_err_cdeg;
    int16_t exit_yaw_err_cdeg;
    int16_t gate_corr_x10;   // along-track correction the edge gate applied
    int16_t gate_drop_s;     // path s at which the drop was seen
    uint8_t arc_min_left;    // closest raw side beam inside the arc
    uint8_t arc_min_right;
    uint16_t corner_ms;      // tangent point to tangent point
    // Odometry minus path s. NOT a slip figure any more: SWEEP_ARC_TRIM_MM
    // adds 25 mm to s at every corner, so a clean four-corner loop reads
    // about -100 here by construction. Run 125's -70.8 after three corners is
    // -75 of trim plus a few mm of real difference. Read the CHANGE across a
    // loop, not the value.
    int16_t wheel_minus_s_x10;
    uint8_t fail;            // SWEEP_FAIL_* -- why the loop ended here
    uint8_t lat_source;      // 0 none, 1 both walls, 2 one wall (see below)
    uint32_t trace_off;      // byte offset of this corner's trace, or 0
    // HOW FAR THE MOUSE ACTUALLY TURNED, tangent point to tangent point, in
    // centidegrees. The commanded value is always 9000. Run 117 delivered
    // 8490 to 8730 -- an effective radius of 126 to 130 mm against a
    // commanded 125 -- and that only came out by reconstructing it from the
    // trace, which exists for a handful of corners. It is the one number
    // that separates "the arc is the right shape and something else pushes
    // me wide" from "the mouse does not turn as far as it is told", so it
    // belongs on every corner of every loop, not just the traced ones.
    int16_t arc_turn_cdeg;
    // Mean measured lateral specific force over the STEADY part of the arc
    // (|omega| within 10 % of v/R, so the slew ramp does not drag it down),
    // mm/s^2, zeroed. Compare against v*v/R, which is fixed by the combo:
    // 320 at R125/v200, 720 at R125/v300. A tyre that cannot make the demand
    // is a mouse on a wider arc than it was told, and this is the one number
    // that says so for EVERY corner, traced or not.
    int16_t arc_ay_mm_s2;
};
//
// ON A LOOP EVENT ROW (corner_ix == SWEEP_CORNER_EVENT) the per-corner fields
// have no corner to describe, so they carry the state at the moment the loop
// stopped instead. Nothing is reinterpreted silently: sweep_dump prints these
// rows as 'X,' with their own column names.
//
//     gate_drop_s          path s where it stopped, mm
//     entry_lat_x10        lateral there, mm x10
//     entry_yaw_err_cdeg   yaw error there, centidegrees
//     arc_min_left/right   raw side beams there
//     lat_source           SWEEP_LAT_* for that lateral
//     exit_lat_x10         index of the corner it was heading TO
//     corner_ms            ms since the loop launched
//     wheel_minus_s_x10    odometry minus path s, as ever
//     arc_turn_cdeg        the PREVIOUS packet's lateral, mm x10 -- the one
//                          number that separates a glitch (centred, then
//                          suddenly not) from a real drift (already going)
//     r_mm / v_arc / slew_ix / flags / fail   as ever
static_assert(sizeof(SweepSummary) == 36, "sweep summary must stay 36 B");

// Why a loop ended early. A guard trip is a RESULT -- "this combination is
// not flyable" is exactly the kind of thing the sweep exists to find out --
// so it is recorded against the combo rather than thrown away as an error.
constexpr uint8_t SWEEP_FAIL_NONE = 0;
constexpr uint8_t SWEEP_FAIL_LATERAL = 1;   // corridor lateral over limit
constexpr uint8_t SWEEP_FAIL_BEAM = 2;      // raw beam proximity, in a junction
constexpr uint8_t SWEEP_FAIL_YAW = 3;
constexpr uint8_t SWEEP_FAIL_TIMEOUT = 4;
constexpr uint8_t SWEEP_FAIL_GATE = 5;      // edge gate lost: position unknown
constexpr uint8_t SWEEP_FAIL_USER = 6;
// NOT FLOWN: the angular slew rail cannot deliver this corner's turn rate
// inside this corner's arc, so the commanded trajectory is not achievable and
// flying it would only find that out by hitting a wall. See
// SWEEP_MAX_RAMP_FRAC.
constexpr uint8_t SWEEP_FAIL_RAMP = 7;

// ---- WHY A SESSION STOPPED --------------------------------------------------
//
// Run 122 flew the cleanest set yet -- seven corners, every exit within a
// couple of millimetres of centred -- and then stopped mid-corridor with
// NOTHING IN THE LOG SAYING WHY. loop_failed() only wrote a record when a
// corner record happened to be open, and the mouse stopped between corner 2's
// window closing and corner 3's opening. The console said something; the
// console was not connected.
//
// A rig that runs unattended has to be able to explain itself afterwards, so
// a loop that ends with no corner in progress now writes a LOOP EVENT row:
// the same 36-byte record with corner_ix set to this sentinel and several
// fields carrying the state at the moment it stopped. sweep_dump prints those
// as 'X,' lines with their own header rather than pretending they are corners.
constexpr uint8_t SWEEP_CORNER_EVENT = 255;

// ---- LAUNCH ROWS -------------------------------------------------------------
//
// Run 141 was the first session to complete, and the one measurement it was
// built to produce did not survive it: the corridor fit's `eps` -- the mouse's
// heading error against the wall, which is the gyro's drift made visible --
// was printed to the console and never written down. The dump is what gets
// kept, mailed and re-read months later; the console is what happens to be
// connected. A number that only exists on the console does not exist.
//
// So every loop now writes ONE record at launch, before it moves, carrying
// what the squaring measured and the pose it is launching from. Same 36-byte
// record, corner_ix set to this sentinel, printed as 'Q,' with its own
// column names. Cost is one record per loop against 4 corners plus a handful
// of events, so under 2 % more summary space.
//
// Fields, re-purposed exactly as the X row re-purposes them:
//
//     entry_yaw_err_cdeg   eps: fitted heading error to the wall, cdeg
//     exit_yaw_err_cdeg    frame correction applied by squaring, cdeg
//     arc_turn_cdeg        body offset from the corridor at launch, cdeg --
//                          what the path frame was shifted BY
//     exit_lat_x10         the cardinal the corridor was squared onto, deg x10
//     entry_lat_x10        standing lateral at launch, mm x10
//     arc_min_left/right   raw side beams while standing
//     lat_source           SWEEP_LAT_* for that standing lateral
//     corner_ms            fit sample count
//     gate_drop_s          fit span, mm
//     gate_corr_x10        fit samples rejected by the outlier gate (a COUNT,
//                          not a x10 length -- the field name is the record's,
//                          the meaning is this row's)
//     fail                 SWEEP_SQ_* below, NOT a SWEEP_FAIL_*
//
// WHICH COLUMN IS THE DRIFT. Not eps -- I said eps when these rows were
// added and run 142, the first session to produce any, said otherwise.
//
// eps is the mouse's TRAVEL DIRECTION relative to the wall. On the final leg
// the lateral servo is still converging, so eps mostly measures that
// convergence. Run 142: eps -1.62 +/- 0.14 deg, negative on 12 of 12 laps --
// and lat0 was +3.6 mm, i.e. the mouse ended each lap left of centre and
// travelling right. Those two agree. It is a servo settling, not a gyro.
//
// frame_corr is the drift channel. It is cardinal - (yaw_bar - eps), and
// yaw_bar and eps are averaged over the SAME samples, so the convergence
// enters both and cancels. What survives is the angle between the fused
// frame and the corridor -- and since squaring resets that to zero at the
// start of every lap, frame_corr is the gyro error accumulated over ONE lap.
// Run 142: -0.708 +/- 0.135 deg, negative on 12 of 12 (t = 5.2).
//
// BIAS OR SCALE is not yet settled and this rig cannot settle it in one
// direction. Bias accumulates with time, scale with angle turned; the v500
// laps are ~20 % longer in time than the v600 laps and the difference came
// out +0.32 +/- 0.27 deg (t = 1.2), which decides nothing. The clean test is
// DIRECTION: a bias keeps its sign when the ring is flown clockwise, a scale
// error flips it, because the lap then turns -360 instead of +360. One
// dir=R session answers it, and it is the same session the tasym pair wants.
constexpr uint8_t SWEEP_CORNER_LAUNCH = 254;

// ---- FEATURE ROWS ------------------------------------------------------------
//
// One per corner approach, printed as 'F,'. Carries what the inside beam
// actually did on the way in: every unbroken run of "something near", with
// where it started and ended, how long it lasted in packets, and how close it
// got. See WallGate's feature observer for why.
//
// This one does NOT re-purpose SweepSummary's fields -- five runs do not map
// onto them in any way a reader could follow. It overlays its own struct on
// the same 36 bytes, with `kind` sitting exactly where `corner_ix` sits so the
// dump can tell the rows apart before it decides how to read them.
struct SweepFeatureRow {
    uint16_t loop_ix;
    uint8_t kind;        // SWEEP_CORNER_FEATURE, aliases corner_ix
    uint8_t flags;       // SWEEP_F_LEFT
    uint8_t corner_ix;   // which approach this describes
    uint8_t runs_seen;   // total runs on the approach, saturating
    struct {
        int16_t s0_mm;
        int16_t s1_mm;
        uint8_t min_mm;
        uint8_t n;       // packets
    } run[5];
};
static_assert(sizeof(SweepFeatureRow) == 36, "feature row must stay 36 B");
constexpr uint8_t SWEEP_CORNER_FEATURE = 253;

// How far out the gate is ARMED, and how far out it may DECIDE. These were
// one number (240) doing both jobs. Decision runway wants to be tight -- a
// wall end far from where the map says it is is not that wall end -- and
// feature counting wants to be generous, because the point of counting is to
// see all of them. Splitting them changes no decision: DECIDE is still 240.
//
// RUN 147 SHOWED THIS NUMBER DOING NOTHING, and v26 fixed the reason. The
// gate used to be armed inside corner_open() and fed only while a corner
// RECORD was open, so the observer's real window was the record's --
// (SWEEP_TRACE_HALF_MM - R), 145 mm at R125 -- and run 147's runs all came
// out 123-141 mm long against a 600 mm arming distance that never once bit.
//
// Arming is now keyed on distance (gate_arm_for), independent of the record,
// so this is the real window at last. 500 mm is over three times what the
// record allowed and covers most of a 900 mm leg, which is what a corridor
// with several walls removed needs: the point of counting posts is to see
// more than one. It cannot collide with the previous corner, because arming
// requires the gate to be DISARMED and corner_close() is what disarms it --
// structural, not a margin that would have to be recomputed per radius.
constexpr float SWEEP_GATE_OBSERVE_MM = 500.0f;
constexpr float SWEEP_GATE_DECIDE_MM = 240.0f;
static_assert(SWEEP_GATE_OBSERVE_MM >= SWEEP_GATE_DECIDE_MM,
              "observing must start no later than deciding");

// SAMPLE DENSITY ON THE INSIDE BEAM, measured in run 147 from 50 walls of
// known length (span in mm divided by packets):
//
//     v500 approach   0.677 mm/packet      v600 approach   0.792 mm/packet
//
// This matters more than it looks. The post detector was designed on my
// guess of 2-3 mm/packet, which made a 12 mm post four to six packets and
// therefore invisible to present_n = 12. The measurement says 15 to 18
// packets -- ABOVE the threshold. The existing step detector may well see a
// bare post as a short wall and drop cleanly off its end, which would make
// post-only corners gateable with no new detector at all.
//
// That is a prediction, not a result: 12 mm is the post's physical width and
// the beam has a footprint and a sweep angle, so the observed run could be
// either longer or shorter. The dir=L run measures it.
constexpr float SWEEP_MM_PER_PACKET_V500 = 0.677f;
constexpr float SWEEP_MM_PER_PACKET_V600 = 0.792f;

// What the squaring did, reported on the launch row's `fail` field.
constexpr uint8_t SWEEP_SQ_NONE = 0;      // no usable corridor fit
constexpr uint8_t SWEEP_SQ_SQUARED = 1;   // frame shifted, path on corridor
constexpr uint8_t SWEEP_SQ_REFUSED = 2;   // fit valid but implausibly far off
constexpr uint8_t SWEEP_SQ_BODY = 3;      // squared, but the launch offset
                                          // exceeded SWEEP_SQ_MAX_DEG so the
                                          // path was built on the body anyway

// BREADCRUMBS. Tracing only runs inside the corner windows, so between them
// there is no record of where the mouse was at all -- which is exactly where
// run 122 stopped. One trace record every SWEEP_BREADCRUMB_MM of corridor,
// tagged with SWEEP_CORNER_EVENT, closes that hole. The cost is about 30
// records per loop (480 B), so a 50-loop session spends 24 KB of a 439 KB
// trace region to never again be blind about where a run ended.
//
// This does NOT cover a mouse that simply dies -- a flat cell writes no
// record of any kind. For that, read loops_done and combo_ix out of the dump
// header: if they have not advanced past the last logged corner, the firmware
// never noticed anything was wrong, and the answer is power, not path.
constexpr float SWEEP_BREADCRUMB_MM = 60.0f;

constexpr uint8_t SWEEP_F_LEFT = 0x01;         // left turn (else right)
constexpr uint8_t SWEEP_F_GATE_LOCK = 0x02;
constexpr uint8_t SWEEP_F_GATE_SKIP_LAT = 0x04;
constexpr uint8_t SWEEP_F_GATE_SKIP_WIN = 0x08;
constexpr uint8_t SWEEP_F_SLIP = 0x10;
constexpr uint8_t SWEEP_F_ABORTED = 0x20;
constexpr uint8_t SWEEP_F_SHORT_APPROACH = 0x40;  // first corner of a loop
constexpr uint8_t SWEEP_F_TRACED = 0x80;

// -- Guards -------------------------------------------------------------------
// Corridor half-width 84 mm, mouse half-width 35 mm, so the body touches at
// 49 mm off centre. 40 leaves 9 mm and is measured from data the mouse has
// every packet with walls on both sides -- pose-free, map-free, and it
// cannot be fooled by odometry drift.
// A SIDE BEAM THAT CANNOT BE A DISTANCE.
//
// The side reading is the perpendicular distance from the mouse CENTRE to the
// wall running alongside it, and the body is 70 mm wide. So 35 mm has the
// wall touching the bodywork and anything below that has it INSIDE the mouse.
// A reading of 38 puts three millimetres between wall and chassis -- for
// 150 mm of corridor at 200 mm/s, which does not happen quietly (bert,
// 2026-08-21). It is not a near miss to be reasoned about downstream; it is
// not a distance at all.
//
// 40 is the floor, and it costs nothing real: the lateral abort is at 40 mm
// of offset, which puts the near beam at 84 - 40 = 44. Anything the mouse can
// be at while still driving reads 44 or more, so a corridor sample under 40
// is already past a threshold that would have fired a packet earlier on a
// larger, more plausible number.
//
// CORRIDOR ONLY. Inside a junction the beam is not looking at a wall beside
// the mouse -- it sweeps posts and corner faces at shallow angles, and run
// 116 saw a legitimate 40 mm there with the body clear. The model-free
// proximity guard (SWEEP_BEAM_ABORT_MM, three packets) covers the junction,
// and it needs no geometry to be true.
constexpr float SWEEP_SIDE_MIN_MM = 40.0f;

constexpr float SWEEP_LAT_ABORT_MM = 40.0f;
// ...and it must PERSIST. Guard 2 (raw beam proximity) has always required
// SWEEP_BEAM_ABORT_N consecutive packets; guard 1 fired on a single one, and
// a single packet is exactly what goes wrong at a wall END.
//
// Run 123 died 4 mm after a sample reading L82 / R93 -- dead centre, lateral
// +5.5 -- at s 2739, with the inside wall ending at s 2737 by the gate's own
// arithmetic. One packet at the transition is enough: if the pair fails the
// corridor-sum check the code falls back to the NEARER wall, so a beam that
// reads short for one packet is both the reason the pair was rejected and the
// number that then gets trusted.
//
// Two packets costs almost nothing in margin. The abort sits at 40 mm and
// contact is at 49, and a mouse in a corridor closes laterally at a few mm
// per hundred of travel -- roughly 5 mm of forward travel per packet at
// cruise, so under a millimetre of lateral. A real approach to a wall still
// trips; a one-packet artefact no longer ends a six-minute session.
constexpr int SWEEP_LAT_ABORT_N = 2;
constexpr float SWEEP_YAW_ABORT_DEG = 25.0f;

// CORRIDOR-SUM SANITY (2026-08-19, from the first hardware run). The two
// side beams should sum to the corridor's face-to-face width, 168 mm,
// wherever the mouse sits across it -- that is what makes the two-wall
// average immune to width error. When they do NOT sum to it, one beam is
// not looking at the opposite wall: at a corner exit the mouse is still
// inside the junction and the inside beam sees straight down the far
// corridor. Run 116 exited a corner reading left 43 / right 147 -- a sum of
// 190 -- and the averaged "lateral" of 52 mm was part artefact.
//
// Averaging two beams that are not a pair is worse than using one, so above
// this tolerance the nearer wall alone is used and the record says so.
// The corridor in BEAM units, not maze units: the walls are 168 face to
// face, but the two side beams sum to 165 through the middle of it (bench
// 'wcal s', three sweeps 2026-09-06: L+R 164.5-165.3 mean, sd 1.3), so 165
// is what a centred mouse's beams add up to and what the one-wall
// fallback (HALF - reading) must be measured against -- at 168 a centred
// mouse by its left beam alone looked 2-3 mm right.
constexpr float SWEEP_CORRIDOR_MM = 165.0f;
constexpr float SWEEP_CORRIDOR_SUM_TOL_MM = 20.0f;
constexpr uint8_t SWEEP_LAT_NONE = 0, SWEEP_LAT_BOTH = 1, SWEEP_LAT_ONE = 2;
// Both beams returned, but they do not describe the same corridor -- their
// sum is not the corridor width. One of them is looking at something that is
// not the wall beside the mouse (a post at a wall end, most often), and the
// sign of the sum error says which. See corridor_lat(). Kept distinct from
// SWEEP_LAT_ONE so a contested pair can be counted in the log rather than
// disappearing into the ordinary single-wall case.
constexpr uint8_t SWEEP_LAT_SUSPECT = 3;

// THE JUNCTION. From the arc's entry tangent to this far past its exit
// tangent, the mouse is inside the corner cell and there is no corridor for
// walls_evaluate's model to describe -- the beams sweep across a concave
// outer corner and down an open corridor. The corridor-model guard is
// therefore suspended here, and the model-free one below takes over.
//
// 60 mm: the exit tangent sits R before the corner point... the mouse
// leaves the corner cell half a cell (90 mm) past the corner point, i.e.
// 90 - R past the tangent. 60 covers every radius in the grid with margin.
constexpr float SWEEP_JUNCTION_EXIT_MM = 60.0f;

// Last-ditch guard for inside the junction: a RAW side beam this close is
// something genuinely against the body (half-width 35 mm), whatever angle
// it is being seen at. Deliberately below anything the geometry produces --
// run 116 saw a legitimate 40 mm mid-arc while the body was clear -- so
// this cannot false-trip on a beam sweeping a corner.
constexpr float SWEEP_BEAM_ABORT_MM = 30.0f;
constexpr int SWEEP_BEAM_ABORT_N = 3;

// A guard trip ends the LOOP, not the session: the combo is marked and the
// mouse waits for a wave to carry on with the next one. This many trips in
// a row is systemic, and then it does stop.
constexpr int SWEEP_FAIL_STREAK_ABORT = 3;
// If the gate misses this many corners running, the mouse no longer knows
// where it is along the corridor. Stop rather than guess.
constexpr int SWEEP_GATE_MISS_ABORT = 6;
constexpr uint32_t SWEEP_TIME_LIMIT_MS = 15UL * 60 * 1000;

// One trace record per SWEEP_TRACE_STEP_MM of PATH, inside the capture
// window only.
//
// DISTANCE-BASED, NOT TIME-BASED, and that is the whole reason the budget
// works. At a fixed 616 Hz a corner costs 16 KB at 200 mm/s and 7 KB at
// 450 mm/s -- so in a SPEED sweep the slow combos eat the FRAM, which is
// exactly backwards. Sampling on distance makes every combo cost the same
// and gives uniform spatial resolution, which is what geometry work wants.
struct SweepTrace {
    int16_t s_mm;
    int16_t v_mm_s;
    int16_t yaw_cdeg;      // relative to the loop's launch heading
    int16_t yaw_err_cdeg;
    int16_t omega_dps;     // commanded, post-slew
    // LATERAL SPECIFIC FORCE, body +Y, mm/s^2, filtered, gravity removed,
    // MINUS the stationary baseline in the header.
    //
    // This replaced lat_x10 rather than joining it, because the trace record
    // has to stay 16 B for the FRAM budget to work and lat_x10 was the one
    // field that is fully recoverable from what is left: the wall model's
    // lateral is (right - left) / 2 when both beams are valid and
    // 84 - left / right - 84 when one is, and both raw beams are still here.
    // bit1 of flags still carries lateral_valid.
    //
    // What to do with it: in a turn the tyres must supply v * omega to hold
    // the arc, and both of those are in this same record. a_lat ~ v*omega is
    // a mouse going round; |a_lat| < |v*omega| is a mouse going straighter
    // than it was told, and the deficit integrated twice IS the distance it
    // slid. Nothing else on this mouse can see that -- a broadside skid
    // turns no wheel and changes no heading.
    int16_t ay_mm_s2;
    uint8_t left_mm;       // raw beams
    uint8_t right_mm;
    uint8_t flags;         // bit0 in_arc, bit1 lat_valid
    uint8_t corner_ix;
};
static_assert(sizeof(SweepTrace) == 16, "sweep trace must stay 16 B");

// Capture window: the corner cell and one cell either side, i.e. the corner
// point +/- 270 mm of SPATIAL corridor. At 4 mm that is ~128 records
// (2.0 KB) per corner, so 50 loops x 4 corners = 400 KB -- inside the
// 446 KB trace region with headroom.
constexpr float SWEEP_TRACE_HALF_MM = 1.5f * SWEEP_CELL_MM;
// Where the entry and exit lateral are averaged, relative to the tangent
// points. The EXIT window starts past the junction (above): measuring 20 mm
// after the tangent, as the first build did, samples the mouse while it is
// still in the corner cell with no corridor around it.
constexpr float SWEEP_ENTRY_WIN_FAR_MM = 80.0f;
constexpr float SWEEP_ENTRY_WIN_NEAR_MM = 20.0f;
constexpr float SWEEP_EXIT_WIN_NEAR_MM = SWEEP_JUNCTION_EXIT_MM;
constexpr float SWEEP_EXIT_WIN_FAR_MM = SWEEP_JUNCTION_EXIT_MM + 80.0f;
constexpr float SWEEP_TRACE_STEP_DEFAULT_MM = 4.0f;

// -- API ----------------------------------------------------------------------
void sweep_set_printer(void (*printer)(const char *line));

// Install the compiled-in grid above. Call at boot BEFORE sweep_resume_load()
// so the loader can tell whether the stored session was flown with this
// firmware's numbers.
void sweep_load_defaults();

// What a wave will do right now -- the mouse has no terminal at the maze, so
// the armed LED pattern says this out loud (see sweep_poll).
enum SweepReadiness : uint8_t {
    SWEEP_READY_FRESH,    // no stored session: a wave starts one
    SWEEP_READY_RESUME,   // same grid, part done: a wave continues it
    SWEEP_READY_NEEDS_WIPE  // stored data from a DIFFERENT grid, or a
                            // finished session: a wave does nothing, a 5 s
                            // hold discards it and starts fresh
};
SweepReadiness sweep_readiness();

// Replace the grid. Any of the three lists may be left alone by passing
// n == 0. Resets progress: a new grid is a new session.
bool sweep_set_radii(const uint8_t *r_mm, int n);
bool sweep_set_speeds(const uint16_t *v, int n);
bool sweep_set_slews(const uint16_t *s, int n);
void sweep_set_direction(bool left_turns);
void sweep_set_trace_step(float mm);   // 0 disables tracing
const SweepGrid &sweep_grid();
bool sweep_left_turns();
float sweep_trace_step();

// Print the grid, the FRAM budget it implies, and how far the session has
// got. Refuses nothing -- this is what you read before waving.
void sweep_report();

// Arm the hand start. A wave launches the whole remaining grid.
void sweep_arm();
void sweep_disarm();
bool sweep_armed();
bool sweep_running();
// Stop the session. `code` is what goes in the loop-event row, so an abort
// says which KIND it was -- a lost edge gate and a human pressing stop are
// not the same finding, and both end up here.
void sweep_abort(const char *why, uint8_t code = SWEEP_FAIL_USER);
void sweep_poll();

// Start the grid from the beginning: wipes the log and the progress. The
// existing records are gone, so dump before calling this.
void sweep_restart();

// Load the session header at boot. Returns true if a valid session (from
// this firmware version) was found, so a battery swap resumes.
bool sweep_resume_load();

// Dump everything as CSV: a header line, then one 'S,' line per corner
// summary, then one 'T,' line per trace record. Blocking; USB is the sane
// place to do this.
void sweep_dump(void (*println)(const char *line));

// THE CORRIDOR LATERAL RULE, shared. Returns SWEEP_LAT_* and writes the
// signed offset from corridor centre (+ve toward the LEFT wall) to *lat.
// Exposed because the route runner steers on the same number and a second
// implementation would not carry the run-125/126/129 reasoning that shaped
// this one: the 40 mm floor on a reading that cannot be a distance, the
// corridor-sum check that decides WHICH beam is lying, and the sign rule that
// follows from it. One lateral, one rule.
struct WallView;
uint8_t sweep_corridor_lat(const WallView &wv, bool in_junction, float *lat);

// Read the summary region back WITHOUT trusting the header. A cleared or
// foreign header leaves every record where it was, so a session whose index
// was lost is still readable until something writes over it. Returns how many
// contiguous plausible records were found; sweep_recover() prints them in the
// normal CSV form. Neither writes to FRAM.
uint32_t sweep_recover_scan();
void sweep_recover(void (*println)(const char *line));

// The rule the recovery scan stops on. Inline and here rather than static in
// the .cpp so it can be tested against real record bytes: it is the whole of
// the judgement about what counts as surviving data, and getting it wrong
// either truncates a recoverable session or prints garbage as if it were
// corners. Deliberately conservative -- it only reads the two fields every
// row type shares, because feature rows overlay a different struct on the
// same 36 bytes and mean something else by everything after them.
inline bool sweep_rec_plausible(const SweepSummary &s) {
    const uint8_t *p = (const uint8_t *)&s;
    bool all0 = true, allf = true;      // blank FRAM reads as one of these
    for (unsigned i = 0; i < sizeof(s); i++) {
        if (p[i] != 0x00) all0 = false;
        if (p[i] != 0xFF) allf = false;
    }
    if (all0 || allf) return false;
    if (!(s.corner_ix < 8 || s.corner_ix == SWEEP_CORNER_EVENT ||
          s.corner_ix == SWEEP_CORNER_LAUNCH ||
          s.corner_ix == SWEEP_CORNER_FEATURE))
        return false;
    if (s.loop_ix > 2000) return false;
    return true;
}
