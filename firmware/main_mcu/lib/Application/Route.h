// Route compiler: a solved maze -> a SpeedPath.
//
// This is the piece phase 3 has always been blocked on. `SpeedPath` could
// build exactly two things -- one corner in an L-corridor (`begin_corner`)
// and the square loop rig (`begin_ring`) -- and neither of them has any idea
// what maze it is in. That is why run 154 flew a 900 mm square through the
// walls of a freshly-searched test8: `sweep.arm` does not read the map,
// because until this file there was nothing that could.
//
// Pure logic. No HAL, no FRAM, no sensors -- it reads a `Maze` and writes a
// `SpeedPath`, so it is host-testable in full (test_route.cpp) and the same
// code answers "would this route fly?" on the bench as flies it at the maze.
//
// ---------------------------------------------------------------------------
// THE GEOMETRY THAT DECIDES EVERYTHING: 2T
//
// Each arc consumes a tangent length T either side of the turn's cell centre,
// so two consecutive turns need T(a) + T(b) of straight between their cell
// centres or the arcs OVERLAP -- and an overlapped pair is not a path, it is
// two commands fighting over the same metres of corridor.
//
//     R125 T=139   R100 T=115   R84 T=98.6   R60 T=91   R45 T=76
//
// At R84 a back-to-back pair therefore needs 197 mm, and cells are 180 mm
// apart. So a hairpin -- turn, one cell, turn back -- CANNOT be flown at the
// chosen radius. This is not a corner case: bert's test8 route hits one on
// its first two turns (N->E at (0,5), E->S at (1,5), 180 mm apart).
//
// bert's policy, 2026-08-23: DROP THE RADIUS LOCALLY. Fly the tight pair at
// whatever the ladder can fit and R84 everywhere else, rather than stopping.
// The cost is real and worth stating: the mouse takes a different line
// through those cells than it would at R84, so a sensor signature gathered
// there does not transfer to a cell flown at the preferred radius. Anything
// the feature library stores must therefore be keyed by the radius that flew
// it, not just by the cell.
//
// ---------------------------------------------------------------------------
// WHAT THIS DOES NOT DO
//
// It does not fly. It builds a path and reports what it had to compromise.
// Nothing here brakes, gates, or reads a beam -- hanging the gathering
// instruments off the result is the next piece, and keeping that separate is
// what lets the route be checked on the bench before it is ever flown.

#pragma once

#include <stdint.h>

#include "Maze.h"
#include "Path.h"

// Cell pitch. The maze spec's 180 mm, restated here so the compiler does not
// depend on a search tunable that exists for a different reason.
constexpr float ROUTE_CELL_MM = 180.0f;

// The radius ladder, largest first. R84 is the max-clearance radius (the arc
// is centred ON the inside post and holds 84 mm throughout); the rest are the
// measured fallbacks from the tangent table above. A turn takes the largest
// entry that fits, so a route with room everywhere is flown entirely at R84
// and the ladder never shows up in the result.
// 73 is here because of the hairpin. Two SAME-DIRECTION 90s one cell apart
// are a 180 turn that the compiler chops in half, and 2*T(73) = 179.9 mm --
// it is the largest radius whose pair fits inside one cell with the arcs
// touching. Runs 156-163 measured what the old R60+R45 pair costs: a peak
// commanded rate of 598 deg/s and a heading error that reached 23.9 deg
// against a 25 deg abort, twice in eight passes. R73+R73 asks 392 deg/s.
constexpr uint8_t ROUTE_RADII[] = {84, 73, 60, 45};
constexpr int ROUTE_N_RADII = sizeof(ROUTE_RADII) / sizeof(ROUTE_RADII[0]);

// NEVER REFUSE A VALID MAZE (bert, 2026-09-11, on D10: "the mouse can always
// slow down but should never refuse any valid maze"). Two rungs sit under
// the radius ladder, both LOCAL to the turn or feature that is short of room:
//
// THE SPEED RUNG. A corner's tangent is R tan(sweep/2) plus a clothoid lead
// that shrinks with the speed it is flown at: an R45 90 needs 62 mm at its
// 424 mm/s corner speed and 47 at 150; an R45 45 goes 33 -> 21. When a pair
// still does not fit with both turns at ROUTE_RADII[last], the fit lowers
// the corner speed of the turn with the larger tangent by ROUTE_V_RUNG per
// step, down to ROUTE_V_FLOOR_MM_S, and the emitted corner carries that
// speed as a cap the planner brakes to (PathSeg::vcap_mm_s). Two 90s one
// cell apart -- the tightest thing a maze can ask -- then always fit:
// 47 + 47 + 20 = 114 in 180 mm. Only a U-turn's setback is speed-blind.
//
// THE VETO (local un-merge). If a pair still refuses, the pair's U-turn
// (first) or staircase (second) is VETOED -- marked not-to-merge, keyed by
// walk cell -- and the walk is recompiled from scratch, up to
// ROUTE_MAX_VETO times: a multi-pass compiler, cheap while the mouse waits
// at the start. A vetoed U becomes the two same-way 90s it also is, which
// the speed rung always fits; a vetoed staircase becomes its 90s. Nothing
// else in the route changes, which is the point: until today one starved
// pair threw away every diagonal and U in the lap (COMP_GAPS A). The old
// global rungs (no U, then no diagonal) remain behind these as the last
// resort and should now be unreachable.
constexpr float ROUTE_V_RUNG = 0.8f;
constexpr float ROUTE_V_FLOOR_MM_S = 150.0f;
constexpr int ROUTE_N_V_RUNGS = 8;          // 0.8^8 = 0.17: 580 -> floor
constexpr int ROUTE_MAX_VETO = 48;   // 12 until 2026-10-03: a vetoed diagonal chain takes two entries per cell

// Straight the compiler insists on leaving between two arcs, on top of the
// two tangents. Zero is geometrically legal -- the arcs merely touch -- but a
// path with no straight between corners gives the wall gate nothing to see
// and the lateral servo nothing to settle on, and every landmark correction
// this mouse makes happens on a straight. 20 mm is the same minimum the ring
// rig's gate room was sized against.
constexpr float ROUTE_MIN_STRAIGHT_MM = 20.0f;

// EXCEPT BETWEEN TWO TURNS THE SAME WAY, WHERE THE STRAIGHT IS PURE HARM.
//
// A left followed by a left (or right then right) one cell apart is a 180
// turn. The curvature never changes sign across it, so a straight in the
// middle is not a rest -- it is a demand that the mouse unwind its turn rate
// all the way to zero and immediately wind it back up.
//
// Runs 156-163, eight passes over the same hairpin, measured what that costs.
// From the packet trace of run 162:
//
//   s= 992  end of arc 0, the mouse is 6.7 deg OVER-rotated
//   s=1000  in the straight, 8.9 deg
//   s=1016  end of the straight, 7.8 deg   <- 24 mm bought back 1.1 deg
//   s=1112  deep in arc 1, -23.9 deg       <- against a 25 deg abort
//
// Three things compound in those 25 mm and every one of them is the
// straight's fault:
//
//   1. the rate unwinds 477 deg/s to zero and back up to 598, over 51 ms;
//   2. the R45 arc wants 470 mm/s and needs 21.1 mm to brake into from 500,
//      against 25.3 mm available -- so the deceleration spills INTO the arc
//      and the rate demand is moving while the loop is at its hardest;
//   3. 24 mm at 500 mm/s is 48 ms, which recovers nothing. arc 1 is entered
//      about 8 deg wrong before it has begun.
//
// Setting the minimum to zero for a same-direction pair lets the radius fit
// keep both turns at R73 instead of demoting to R60 and R45. Peak rate falls
// 598 -> 392 deg/s, the lateral demand falls 0.50 g -> 0.36 g, and neither
// arc needs to brake at all.
//
// NOTHING IS LOST BY REMOVING IT. The straight exists to give the gate and
// the lateral servo somewhere to work, and between a tight same-direction
// pair there is nothing to gate anyway -- the second turn is always the
// `inherited` case (see RouteRun.h), because clearing the first turn's
// junction already puts the mouse past the second turn's post.
//
// This was the cheap experiment, and it CONFIRMED the mechanism: runs 164-167
// flew R73+R73 and the hairpin's heading error fell from 14.9 to 10.1 against
// a prediction of 9.8. Rate-tracking lag, nothing subtler. The relaxed
// minimum stays because it is still what lets a same-direction pair be flown
// symmetrically when the U-turn below cannot be used.
constexpr float ROUTE_MIN_STRAIGHT_SAME_MM = 0.0f;

// -- THE U-TURN ---------------------------------------------------------------
//
// Two same-direction 90s in ADJACENT cells are not two corners. They are a
// 180, and chopping them in half is what created every problem the hairpin
// had. Emitted as one primitive (SpeedPath::add_uturn) the radius is forced
// by the maze -- the exit corridor is one cell from the entry corridor, so
// the lateral offset is exactly ROUTE_CELL_MM and R follows:
//
//     v500, 6000 deg/s^2 rail  ->  R 89.7, clothoids 26.6 mm, 163 deg of arc
//     peak rate 319 deg/s      (R84 corner: 341.  R73+R73: 392.  R60+R45: 598)
//
// Clearance improves as well: 61.9 mm of body margin at the inside post
// against 49 mm for an R84 corner. And it collapses the gating problem --
// the U-turn's inside post is the same post that gated the FIRST of the two
// 90s, so one anchor serves the whole manoeuvre and the `inherited` second
// turn (see RouteRun.h) stops existing.
//
// WHAT BOUNDS IT, AND THE MISTAKE THE FIRST VERSION MADE.
//
// The turn reaches along the entry corridor before it comes back, and the
// mouse must clear the far wall of the turn row. The first version sized that
// from the CENTRELINE apex plus the body half-width:
//
//     setback = extent - (WALL_FACE - BODY_HALF - MARGIN)      <- WRONG
//
// which gave 103.0 - 39 = 64.0 mm and looked like 10 mm of margin. Runs
// 168-171 scraped the north wall on every pass, with the inside beam showing
// excessive room at the post -- the whole turn displaced outward (bert).
//
// A MOUSE IS A RECTANGLE, NOT A POINT. Swept round an arc, its outermost
// point is the outer front corner, and that corner is further from the arc
// centre than the flank is:
//
//     outer sweep = sqrt((R + BODY_HALF)^2 + BODY_FRONT^2)
//
// not R + BODY_HALF. At R89.7 that is 6 mm extra for a 40 mm overhang and
// 18 mm for 70 -- so the 10 mm of "margin" was somewhere between thin and
// negative, which is exactly what it did.
//
// It never bit on a 90-degree corner because there the binding clearance is
// the inside POST with 49 mm to give. The U-turn is the first primitive that
// runs against the outer wall with single-digit margin, so it is the first
// place the rectangle matters.
//
// The setback is now built from three terms, each defensible on its own:
//
//     setback = sqrt((R + BODY_HALF)^2 + BODY_HALF_LEN^2)   the envelope
//               - WALL_FACE + MARGIN                        the clearance
//               + LATE_MM                                   the measured late start
//
// and the clearance is ASSERTED rather than assumed -- a U-turn that cannot
// prove it clears is refused and falls back to two 90s.
//
// WITH THE MEASURED HALF-LENGTH THE ENVELOPE ALONE DOES NOT EXPLAIN THE
// SCRAPE. At 45 mm it adds 7.9 mm, so the 64 mm setback had 15.5 mm of real
// clearance and should have cleared. It is the 12 mm late start (below) that
// took it to 3.5 and put the flank on the wall. Both terms are in the sum
// now, and it was worth finding out that the smaller one was not the culprit
// -- a bigger margin would have hidden the late start rather than measuring
// it.
//
// The setback is also what the wall gate uses as this corner's reported
// tangent, so gate room is 135 - setback. That is what a bigger setback
// spends; the post clearance barely moves, because it is dominated by the
// 90 mm column offset and never drops below 90 mm against 84 for an R84
// corner. bert's "excessive post clearance" is real and cannot be reduced:
// R is forced by the one-cell offset.
constexpr float ROUTE_BODY_HALF_MM = 35.0f;   // mouse half-width
constexpr float ROUTE_WALL_FACE_MM = 84.0f;   // wall face from a cell centre
// HALF THE CHASSIS LENGTH. Measured (bert, 2026-08-26): the mouse is 90 mm
// long with the wheel axle CENTRAL, so both ends reach 45 mm from it -- and
// the axle centre is the datum for everything else too (the forward sensors
// are calibrated from it, the odometry integrates it, and it is the centre of
// rotation, so it is what the path follows). One origin, no offsets to carry.
//
// Central means the REAR sweeps as far as the front, so this bounds both ends
// of the U-turn: the front-outer corner going in, the rear-outer coming out.
constexpr float ROUTE_BODY_HALF_LEN_MM = 45.0f;
// Slack at the apex ON TOP of the swept envelope.
constexpr float ROUTE_UTURN_MARGIN_MM = 12.0f;

// HOW LATE THE MOUSE ACTUALLY STARTS THE TURN, and why that is a clearance
// term rather than a bug to be fixed here.
//
// The gate's correction at the first corner is +12 mm and has been for every
// run of the last three geometries:
//
//     runs 156-163  R60+R45   +12.2 mean, sd 1.8, n=8
//     runs 164-167  R73+R73   +11.2 mean, sd 1.0, n=4
//     runs 168-171  U-turn    +12.0  +11.0  +13.0  +12.0
//
// If the odometry is sound, that correction IS
//     true beam lead - WALL_GATE_BEAM_AHEAD_MM
// so a constant of 45 against a true lead near 57 makes the gate push s
// forward by 12 when the mouse was already right. The mouse then believes it
// is 12 mm further along than it is and delays the turn by exactly that --
// which is bert's "the corner was started too late", and it moves the apex
// 1:1 toward the wall.
//
// IT COULD ALSO BE THE MAZE. A repeatable offset between where the model puts
// that post and where it physically is produces the identical signature, and
// this data cannot separate the two: the later corners' corrections are mixed
// in sign, which a pure lead error would not be. Separating them needs the
// pre-registered test the WALL_GATE_BEAM_AHEAD_MM note asks for, and moving
// that constant on a hypothesis would re-bias every corner on every route.
//
// It is carried HERE as an allowance on the one primitive whose clearance is
// small enough to care. It is a STOPGAP, and it is still one: the beam-lead /
// maze-offset question is untouched by anything below, because a lead error
// moves WHERE the arc starts, not what shape the mouse then flies.
//
// WHAT THE 2026-08-27 INSTRUMENT SETTLED. The duty budget named in the old
// version of this note is gone, and it was never the culprit -- at 500 mm/s
// the corner's whole demand is ~0.2 duty against a 0.45 fold. The omega
// column, live from run 180, said something else instead; see
// ROUTE_UTURN_TRACK_MM below.
constexpr float ROUTE_UTURN_LATE_MM = 12.0f;

// HOW FAR OUTSIDE ITS OWN ARC THE MOUSE ACTUALLY FLIES -- AND WHY THE
// ALLOWANCE FOR IT IS ZERO.
//
// Runs 180-182, cross-track reconstructed from the flown yaw against the
// compiled clothoid-arc-clothoid, at the U-turn: +12.4, +9.7, +19.2 mm, all
// OUTSIDE. Run 181's LEFT-hand U-turn came out at +10.0 against its own
// right-hand's +9.7, and its four R84 nineties at +10.2, +4.9, +5.9, +8.3.
// Six arcs, both hands, both sweeps, every one outside. Both hands wide by the
// same amount rules out every signed term (the learned yaw trim, VCTRL_TASYM)
// and leaves a symmetric tracking lag.
//
// AND THE SIZE DOES NOT DEPEND ON THE RADIUS. Cross-track is the integral of
// the heading lag, the lag is proportional to commanded rate, so
//
//     integral(lag ds) = k * integral(omega*v dt) = k * v * total_sweep
//
// and R cancels. A 180 bulges twice as far as a 90 because it turns twice as
// far, not because it is tighter -- which is why tightening the radius is not
// the lever it looks like.
//
// THE ALLOWANCE IS ZERO BECAUSE THE CORRIDOR HAS NOTHING TO GIVE. Bisected on
// the test8 loop, 2026-08-27, against the CORRECTED setback (uturn_body_reach
// in Route.cpp -- the old one understated the forward reach by 13.3 mm):
//
//     TRACK 0    setback 85.8    3 U-turns,  9 turns, 2 demotions
//     TRACK 2    setback 87.8    0 U-turns, 12 turns, 8 demotions
//
// Two millimetres. Past that the U-turn stops leaving its neighbour a legal
// straight and the compiler falls back to the R73/R73 hairpin bert replaced at
// run 168 -- silently, so watch `uturns` in the route report.
//
// That is the real state of this primitive: it fits, with the 12 mm build
// margin and not one millimetre more. Every millimetre of flight error goes
// into the wall. Padding is finished as a strategy here; the error has to come
// out of the CONTROL, which is what the corrected setback and the rate-rail
// fix in RouteRun.cpp are for.
constexpr float ROUTE_UTURN_TRACK_MM = 0.0f;

// -- THE U-TURN RADIUS, FORCED BY HAND -----------------------------------------
//
// bert, 2026-08-27, after the first run with the understeer feedforward still
// went wide: "humour me and reduce the radius by 20% on the first corner 0 U
// turn. if it crashes we will go your way."
//
// 0.8 * 89.67 = 71.74. It is an experiment with an explicit stop condition,
// and it is worth flying even though the bulge analysis says the radius is
// not the lever, because that analysis makes a prediction the run can falsify:
//
//     the bulge should NOT shrink       (it is k*v*sweep; R cancels)
//     the apex clearance SHOULD improve by 16.7 mm
//                                       (envelope 132.5 -> 115.8)
//
// So a run that clears tells us the extra clearance was enough. A run that
// clears AND shows a smaller bulge says the model is wrong and the radius
// matters after all -- which is worth knowing and is exactly what bert is
// buying. Read the flight log's cross-track before declaring either.
//
// HOW IT IS REACHED. Not by lowering the rail -- that path is blocked, see
// SpeedPath::uturn_solve_radius. The radius is fixed and the CLOTHOID is
// solved to make the offset come out at exactly one cell, so the geometry
// stays exact: the mouse still lands on the next corridor's centreline. The
// rail falls out at roughly 1090 dps^2, well inside PATH_OMEGA_SLEW_DPS2.
//
// WHAT IT COSTS, stated so the crash is diagnosable if it comes:
//
//     clothoid    26.6 -> ~185 mm       the turn is nearly all transition
//     arc          255 -> ~40 mm
//     length       308 -> ~408 mm       +100 mm per U-turn
//     peak omega   319 -> 400 deg/s     +25 %, so +25 % of bulge
//     a_lat       2788 -> 3486 mm/s2    0.28 g -> 0.36 g
//     envelope   132.5 -> 115.8 mm      -16.7, the point of the exercise
//     setback     85.5 -> 68.8 mm       frees 16.7 mm of corridor
//
// The lateral acceleration is the number to watch. Nothing on this mouse
// closes the loop on slip, and 0.36 g is the highest any corner has been
// asked for. If it lets go, it lets go at the apex.
//
// IT APPLIES TO EVERY U-TURN ON THE ROUTE, not just corner 0 -- the compiler
// solves one U-turn geometry and reuses it. On the test8 loop that is three.
//
// Set to 0 to go back to solving the radius from the rslew rail, which
// is the R89.67 that flew runs 168-182.
constexpr float ROUTE_UTURN_RADIUS_MM = 71.74f;

// A U-turn is tried first and FALLS BACK to two 90s when it does not fit --
// a neighbouring turn too close, or no radius that offsets one cell. The
// fallback matters: it means adding this cannot make any route worse than it
// was, only better or identical.
constexpr bool ROUTE_UTURN_ENABLED = true;

// ---------------------------------------------------------------------------
// DIAGONALS: A STAIRCASE IS ONE STRAIGHT, NOT SIX CORNERS
//
// Runs 284-290 (2026-08-31) flew D6's diagonal corridor as what the flood
// sees -- (5,3)->(4,3)->(4,2)->(3,2)->(3,1)->(2,1)->(2,0), six alternating
// 90s one cell apart, demoted to R60/R45 because two R84 tangents do not fit
// a 180 mm cell. Measured: 19-21 mm of straight between arcs, an omega
// reversal of ~1300 deg/s across it (~32,500 deg/s^2 demanded against a
// 7,000 rail), every corner after the first running 12.7+ deg late at its
// tangent point, one crash in four, and no gate able to arm anywhere in it.
// Arc-straight-arc cannot fly that geometry at a sensible speed. What the
// corridor physically IS is a straight at 45 degrees.
//
// THE GEOMETRY. A run of k alternating 90s, each exactly one cell after the
// last, is replaced by: a 45-degree arc at the FIRST turn cell, one straight
// along the corridor's centre line, a 45-degree arc at the LAST. The centre
// line runs through the aperture midpoints of the staircase, NOT from cell
// centre to cell centre -- centre-to-centre is parallel but half a cell
// off, and lands exactly on the corner posts (the GUI drew it that way until
// today). The entry corner point is where the entry corridor's centre line
// meets the diagonal: half a cell BEFORE the first turn cell's centre. The
// exit corner point is half a cell AFTER the last turn cell's centre, along
// the exit heading. On D6 that is (5.5,4.0) -> (2.0,0.5) in post units, a
// displacement of (3.5,3.5) cells -- 891 mm, exactly 45 degrees. The
// compiler computes both points from the cells and REFUSES the merge if the
// displacement is not exactly diagonal, rather than trusting the derivation.
//
// THE SIDE-GATE LANDMARK SURVIVES. A 45-entry turns around the same post a
// 90 would -- the near corner of the turn cell -- and the beam loses the wall
// at that post's far face, 84 mm before the cell centre, exactly as for a
// 90. Only the corner point moved (90 mm earlier), so RouteTurn carries the
// corner point's offset from the cell centre and the gate reckons from the
// centre. Nothing else about the expectation changes.
//
// STAGE ONE IS DATA (bert, 2026-08-31: "data can be collected initially
// similar to fgate=1 to discover what data is available repeatably"). The
// diagonal is FLOWN, but no gate on a 45 or on the diagonal steers: the
// entry 45 records what the side edge and the front wall would have said,
// the exit 45 records nothing, and every side/front beam episode along the
// diagonal is logged against the post lattice the geometry predicts. What
// repeats becomes a gate; what does not stays a log line.
//
// Fallback, like the U-turn: a staircase the ladder cannot fit as a diagonal
// is recompiled as the zigzag it was, so this cannot make a route worse.
constexpr bool ROUTE_DIAG_ENABLED = true;
// Fewest one-cell gaps in an alternating run to merge. 1 gap = the dogleg
// (R then L in adjacent cells), a 45-in, 254.6 mm of diagonal at 45 degrees,
// a 45-out: room for T45 + T45 + 20 = 122 mm at R84 with 130 to spare.
// Until 2026-09-12 this was 2 and a dogleg compiled as two crowded 90s at
// R60/R45 with no straight between them -- the D6 crash geometry in
// miniature, and the commonest small feature of a real maze (COMP_GAPS C).
// D9's (5,2)/(4,2): R60+R60, min straight 25 -> R84 45s, min straight 88.
constexpr int ROUTE_DIAG_MIN_STEPS = 1;
// Corner-point offset from the turn cell centre, along the walk: a 45-entry
// sits half a cell before, a 45-exit half a cell after.
constexpr float ROUTE_DIAG_CP_OFF_MM = 0.5f * ROUTE_CELL_MM;
// Along the diagonal, aperture midpoints repeat every half-cell diagonal
// (180/sqrt2 = 127.3 mm) starting one of them past the entry corner point;
// the posts sit midway between, 63.6 mm off the centre line, alternating
// sides. Their faces are 57.6 mm from the centre line: 22.6 mm of clearance
// for a 70 mm mouse, against 49 in a corridor. The diagonal observer uses
// these to label what each beam saw.
constexpr float ROUTE_DIAG_PITCH_MM = ROUTE_CELL_MM * 0.70710678f;
constexpr float ROUTE_DIAG_POST_OFF_MM = 0.5f * ROUTE_DIAG_PITCH_MM;

// The largest route this will compile. A 16x16 flood route cannot exceed the
// cell count, and turns cannot exceed cells.
constexpr int ROUTE_MAX_CELLS = MAZE_SIZE * MAZE_SIZE;
constexpr int ROUTE_MAX_TURNS = ROUTE_MAX_CELLS;   // any walk's grid turns (Path.h)

enum RouteResult : uint8_t {
    ROUTE_OK = 0,
    ROUTE_NO_PATH,        // the flood does not reach, or the map is not known
    ROUTE_TOO_LONG,       // more cells or turns than the tables hold
    ROUTE_BAD_TURN,       // a heading change that is not +/-90 (a U-turn)
    ROUTE_NO_RADIUS,      // a turn pair that not even the smallest radius fits
    ROUTE_PATH_FULL,      // SpeedPath ran out of segments
};

struct RouteTurn {
    uint8_t cx, cy;      // cell the turn is centred on
    // MAZE-frame heading the mouse ARRIVES on (a Heading, 0-3). Same cell,
    // opposite approach = two different physical landmark edges, so this is
    // half of the per-corner goff table's key (GoffTable.h). Survives the
    // U-merge because the merge keeps the pair's FIRST record.
    uint8_t entry_h;
    // 1 = the maze has a wall directly ahead of the turn cell on the
    // approach heading -- its face sits at the corner datum + 84 mm, seen
    // by the FRONT beams well before the side wall-end and nearly
    // orthogonal to the path, so its range barely moves with lateral error
    // (bert, 2026-08-30). Such corners can be gated by front wall instead
    // of side edge -- see the front gate in RouteRun.
    uint8_t front_ok;
    uint8_t r_mm;        // radius actually used
    // +90 = left (CCW), -90 = right, +/-180 = a U-turn. int16 because 180
    // does not fit a signed byte -- and a U-turn IS the sweep, not a flag
    // bolted onto a 90.
    int16_t sweep_deg;
    uint8_t cell_ix;     // index along the route
    // U-turn only: the route index of the cell it EXITS through. Equal to
    // cell_ix for a 90.
    uint8_t exit_ix;
    // Corner point offset from the cell centre along the walk, mm: 0 for a
    // 90 or a U-turn, -90 for a diagonal ENTRY (+/-45), +90 for a diagonal
    // EXIT. See the diagonal note above ROUTE_DIAG_ENABLED.
    int8_t cp_off_mm;
    // Diagonal EXIT only: the corner-point-to-corner-point length of the
    // diagonal it ends, mm. 0 otherwise. The exit's partner is the turn
    // immediately before it.
    uint16_t diag_mm;
    // Speed rung: the corner speed the fit had to drop this turn to so its
    // pair fits, mm/s; 0 = flown at the normal sqrt(ralat R) cap. Carried
    // into the path as the arc's vcap.
    uint16_t v_cap_mm_s;
};

// What the fit could not place, reported so the retry loop can veto the
// feature responsible rather than the whole route.
struct RouteRefusal {
    uint8_t cell_a, cell_b;    // walk cells of the pair (a before b); a == b
                               // for the lead-in or the tail
    uint8_t uturn_a, uturn_b;  // 1 = that turn is a merged U
    uint8_t diag_a, diag_b;    // 1 = that turn belongs to a merged staircase
};
struct RouteVeto {
    uint8_t cell_ix;
    uint8_t kind;              // ROUTE_VETO_UTURN / ROUTE_VETO_DIAG
};
constexpr uint8_t ROUTE_VETO_UTURN = 1;
constexpr uint8_t ROUTE_VETO_DIAG = 2;
// A DIAGONAL MAY NOT HAND STRAIGHT INTO A U-TURN (2026-10-03, Hazlemere,
// the second session's MR2: a 135-out at (8,3) landed 44 mm inside its
// corridor -- a diagonal's exit is observed, never gated -- and the U-turn
// at (6,3) began 98 mm later, before the lateral servo had done anything;
// the U's second arc put the mouse 48 mm into the dividing wall. The same
// U flown from a corridor 90 with 320 mm of straight before it was clean
// on both laps of the first session. A diagonal exit (45-out, 135-out)
// followed by a U within this much straight is un-merged -- its staircase
// flies as 90s -- and the U gets a corridor approach. A plain 90 after a
// diagonal exit is not touched: six of those flew clean the same lap.
// WITHDRAWN 2026-10-04 (the test-day analysis, RACE_PLAN): the 135 at
// (8,3) landed 44 mm inside because the V 65 mm before it had left the
// mouse 12 mm to its outside (vout 10 + 2) -- the U was handed the error,
// it did not make it. The rule's replacement route was a five-arc R73
// zigzag at (8,0)..(8,3) that crashed at once (hazel10); every un-merged
// zigzag flown that day did (0/4). Meanwhile ~44 U-turns were flown clean
// at 338-476 mm/s. 0 = off; the code path stays for a maze that proves
// the rule.
constexpr float ROUTE_DIAG_EXIT_TO_UTURN_MM = 0.0f;
// ...NOR MAY A V HAND STRAIGHT INTO A 135-OUT OF THE OPPOSITE HAND (same
// day, maze 1: right V (5,13) -> 255 mm diagonal -> left 135-out (3,14).
// A V leaves the mouse vout OUTSIDE its exit line and the lane change
// walks that back over ~90 mm; a 135's arc begins 2.4 R = 203 mm before
// its corner point, so here it began ~50 mm after the V's arc ended --
// with the mouse still on the outside of the V, which is the INSIDE of
// the opposite-hand 135. Brushed at MR3 (hazel4, ye 12), hit at MR4
// (hazel6, ye 24), hit at MR2 with vout 15 (hazel_3, the other maze).
// Below this much diagonal between the V's arc end and the 135's arc
// start the chain is un-merged. Same-hand pairs are not touched: the
// mouse is then on the 135's OUTSIDE.
// WITHDRAWN THE SAME EVENING (hazzel8, MR2): the un-merged chain is a
// one-cell staircase, which can only be flown as an R60 zigzag -- and that
// zigzag ran both arcs ~40 mm wide at 289 mm/s, arrived at the (3,14)
// front wall 70 mm out of place and was stopped by the front governor 60
// mm short of it. The V -> 135 pair had flown clean at MR2 twice and MR3
// once (one brush) that day; the hit was at MR4, which rdv now flies
// orthogonal. 0 = off. Kept for a maze where the chain is longer.
constexpr float ROUTE_V_TO_OPP_135_MM = 0.0f;

// THE V (2026-09-05, bert: "we need to concentrate on the turns, there can
// be back to back 45's in opposite directions"). One diagonal handing
// straight into another -- NE into NW -- is, in cells, two SAME-sign 90s
// one cell apart sitting INSIDE a staircase: R L R L | L | R L R. The
// U-merge used to eat that pair as a 180 (it is one, in heading terms: E
// -> N -> W), the diagonal on each side then had no tangent room against
// it, and the whole run fell back to the zigzag. Physically the exit
// corner point of the first diagonal (half a cell after the pair's first
// cell, along its departure heading) IS the entry corner point of the
// second (half a cell before the pair's second cell, along its arrival
// heading): the aperture between the two cells. So the pair is ONE 90
// degree arc at that point, from one diagonal's heading to the other's,
// with the aperture's inside post 90 mm away along the bisector. Its
// record is the pair's FIRST turn with sweep +/-90, cp_off_mm +90 (like
// a 45-out) and diag_mm = the diagonal it ends (like a 45-out); the next
// diagonal's exit measures ITS diag_mm from the same point. That is how
// the runner and the fit tell a V from a corridor 90: a 90 with a
// corner-point offset. Inside clearance at R84: the arc's apex sits
// R(sqrt2 - 1) = 35 mm from the corner point toward a post whose face is
// 84 mm away -- 12 mm for a 37 mm half-width; R60 gives 22. (Wrong --
// corrected the same day: the post sits near the arc's CENTRE, not its
// apex; see ROUTE_V_RADIUS_MAX_MM for the real clearance.)
inline bool route_turn_is_v(const RouteTurn &t) {
    return (t.sweep_deg == 90 || t.sweep_deg == -90) && t.cp_off_mm != 0;
}

// THE 135 (2026-09-11, D10; bert: "the mouse can always slow down but
// should never refuse any valid maze"). A diagonal that hands straight into
// a corridor -- or a corridor straight into a diagonal -- is, in cells, a
// staircase whose last (first) turn is followed (preceded) one cell on by a
// 90 the SAME way, with a straight beyond it:
//     R L R L R | R | straight       diagonal NE, then south: one 135 right
// The old parse made a U of the pair (2b) and left the 45-out 90 mm short of
// the U's setback: refused, and the whole lap fell back to the zigzag.
// Physically the diagonal's centre line and the corridor's meet at ONE
// point, half a cell PAST the lone 90's centre against its departure (a
// 135-out) or half a cell past it along its arrival (a 135-in) -- which is
// the corridor's end-wall face if that side is walled, an aperture if not.
// One arc of 135 degrees joins the two lines there: tangent 2.414 R, entry
// on the diagonal 2.414 R before the point, exit on the corridor centre
// line 2.414 R after it. The record is the lone 90's (its cell, its arrival
// heading, its front wall) with sweep +/-135, cp_off_mm -90 for an out
// (the point is before the cell centre on the departure axis) and +90 for
// an in, and diag_mm on the out as for a 45-out -- the diagonal it ends is
// one post pitch longer than the 45 would have measured.
//
// Clearance (arc centre 2.613 R from the corner point on the bisector; body
// 70 x 90, axle 45 from the rear): to the end wall, when there is one, 68 mm
// at R84, 32 at R60, 9 at R45; the inside post -- the lone 90's own inside
// post -- sits 24 mm from the R84 centre, INSIDE the body's inner sweep
// radius of 47, so the mouse pivots round it 15 mm clear; at R73 10 mm; at
// R60 and below the post is outside the inner radius but behind the swept
// angle. Stage one is data, as for the 45s: the 135 is flown, its entry
// side is placed by the corridor (a 135-in) or by the diagonal's posts (a
// 135-out), no gate steers on it, and the runner treats it as diagonal
// family (RouteRun is_diag_turn).
constexpr bool ROUTE_135_ENABLED = true;
// r135: the registry switch (Control.h route_135), read at compile time by
// the runner; ROUTE_135_ENABLED is the compiled ceiling.
void route_set_135(bool on);
// rdiag: the runner's switch for diagonals as a whole (45s, 135s and Vs);
// off, every staircase compiles as its 90s. ROUTE_DIAG_ENABLED is the
// compiled ceiling.
void route_set_diag(bool on);
bool route_diag_enabled();
// rturn: the turn-weighted walk (Route.cpp, walk_weighted) -- cells of cost
// per 90 degree turn when choosing the route; 0 = the plain cell flood.
void route_set_turn_cost(float cells_per_turn);
float route_turn_cost();
// Where the walk the last route_build flew (from (x, y, h), on the CURRENT
// flood) ends: the runner's leg-end pose.
bool route_walk_end(Maze &maze, int x, int y, Heading h, int *ex, int *ey, Heading *eh);

// THE ROUTE IS CHOSEN BY FLIGHT TIME (Route.cpp, "compare"). With rturn > 0
// and a time context set, route_build compiles both the fewest-cells walk
// and the fewest-turns walk (diagonals merged, radii fitted), flies each
// through the planner at these limits, and keeps the lower of
// time + rturn cells of time per compiled turn (a U counts two). The runner
// sets the context before every compile; route_home_heading uses it too.
void route_set_time_context(float v_peak, float a_lon, float a_lat, float a_brk,
                            bool clothoid_hold, float slew_dps2);
void route_clear_time_context();
// The commanded flight time of a compiled path at the context's limits, ms
// (the planner walked from rest to the end; sets those limits on p).
float route_path_time_ms(SpeedPath &p);
struct RouteChoice {
    bool compared;       // both candidates compiled and were scored
    bool same_walk;      // the two walks were the same cells
    uint8_t chosen;      // 0 = fewest cells, 1 = fewest turns
    struct {
        bool valid;
        uint8_t cells, turns, uturns, diags;
        float length_mm, ms, score;
    } cand[2];
};
const RouteChoice &route_last_choice();
bool route_135_enabled();
inline bool route_turn_is_135(const RouteTurn &t) {
    return t.sweep_deg == 135 || t.sweep_deg == -135;
}
// A run bounded by a V on either side may be as short as one step (a
// diagonal zigzag: NE, NW, NE ... each 254 mm); a run bounded by 45s on
// both ends keeps ROUTE_DIAG_MIN_STEPS.
constexpr int ROUTE_DIAG_MIN_STEPS_AT_V = 1;
// THE V'S RADIUS IS 64, AND THE POST IS ITS CENTRE. The inside post sits
// 90 mm from the corner point on the bisector; a 90 arc's centre sits
// R*sqrt2 along the same bisector. The body's inner side sweeps a circle
// of radius R - 37 about that centre, so the clearance to the post is
//     (R - 37) - |R sqrt2 - 90| - 6
// which is 12 mm at R84, 12 at R60 (the first cap, 2026-09-06 morning --
// "no observable change", bert, and none was due), and peaks at 20.5 mm
// at R = 90/sqrt2 = 63.6, where the mouse pivots on the post itself. An
// entry offset e toward the inside moves the centre e off the post and
// costs ~e of clearance at every radius (e = 15: 5 mm at 60, 64 and 73,
// nothing at 84) -- which is why no radius flew clean until the diagonal
// delivers the mouse on its centre line, and why firing the V later or
// earlier cannot help (it moves the centre off the post too). The ladder
// below 64 is still available to the fit.
// -- unless the diagonal is flown OUTSIDE its centre line on purpose:
// with `vout` (route_set_v_out) the lines on both sides of the V are offset
// outward by vout, the arc that joins them is still centred on the post,
// radius 64 + vout, tangent to the offset lines at the SAME stations, and
// the clearance grows by vout. The outer posts of the two diagonals are a
// post pitch (127 mm) before and after the V station; between them the
// outside is empty, which is the room this uses.
constexpr float ROUTE_V_RADIUS_MAX_MM = 64.0f;

struct RouteStats {
    uint8_t cells;         // route length, cells stepped
    uint8_t turns;
    uint8_t dropped;       // turns that could not take ROUTE_RADII[0]
    float   length_mm;     // total path arclength
    float   min_straight;  // shortest straight between two arcs, mm
    uint8_t same_pairs;    // turn pairs that took the same-direction minimum
    uint8_t uturns;        // pairs emitted as ONE 180 instead of two 90s
    uint8_t diags;         // staircases emitted as 45 / straight / 45
    uint8_t goal_ix;       // route index of the goal cell on an out-and-back,
                           // 0 on a one-way route
    uint8_t turns135;      // 135s emitted (diagonal <-> corridor in one arc)
    uint8_t slowed;        // turns carrying a speed cap (the speed rung)
    uint8_t vetoed;        // features un-merged by the local fallback
    uint8_t passes;        // compile passes the fallback loop took (1 = clean)
};

// Compile the route the maze's CURRENT flood points along, starting at
// (x0, y0) heading h0, into `path`. Call maze.flood_to_goal(false) (or
// flood_to_start) first -- this walks that flood downhill and does not
// choose a destination of its own.
//
// v_mm_s and slew_dps2 size the clothoids (L = v^2 / (R * alpha)), so a route
// compiled for one speed is not the path for another. That is deliberate: the
// transition length IS a function of speed, and pretending otherwise is what
// makes a corner that tracks at 500 stop tracking at 700.
//
// `path` is cleared. On anything but ROUTE_OK its contents are undefined.
// lead_in_mm is straight laid down BEFORE the first cell centre, for a mouse
// that does not start on one. The competition start is the case: the mouse
// begins with its back to the rear wall, 51 mm in, and the cell centre is at
// 90 -- so the start leg needs 39 mm of lead-in or every landmark on the
// route sits 39 mm from where the path thinks it is. A leg that begins on a
// cell centre passes 0.
RouteResult route_build(Maze &maze, int x0, int y0, Heading h0,
                        SpeedPath &path, float v_mm_s, float slew_dps2,
                        RouteStats *stats = nullptr,
                        RouteTurn *turns_out = nullptr,
                        int max_turns_out = 0,
                        float lead_in_mm = 0.0f);

// ONE CONTINUOUS OUT-AND-BACK: start -> goal -> start, no stop at the goal.
//
// Compiled as a SINGLE walk -- the flood to the goal, then the flood back to
// the start, joined end to end -- and handed to the same compiler as a
// one-way route. That is the whole trick: the goal stops being a special case
// and becomes whatever heading change happens to sit there.
//
//   the return leaves on the SAME heading    -> no turn at all, it flies through
//   the return leaves PERPENDICULAR          -> an ordinary 90
//   the return leaves via the next corridor  -> two same-direction adjacent
//                                               turns, which the U-turn merger
//                                               already collapses into one 180
//   the return leaves BACKWARDS (a dead-end
//   goal cell)                               -> ROUTE_BAD_TURN, honestly. No
//                                               finite radius reverses in a
//                                               168 mm corridor; that needs a
//                                               spin, which this does not do.
//
// On test8 it is the first case: the mouse arrives at (2,0) heading WEST and
// the return leaves (2,0) heading WEST. There is nothing to turn -- it just
// keeps going, and the "stop at the goal" was only ever the mission choosing
// to end the leg there.
//
// Call it INSTEAD of route_build. It floods twice internally, so the maze's
// flood is left pointing at the start when it returns.
//
// force_exit (route.dir): on a CLOSED-CIRCUIT rig both arms reach the goal,
// and which one the walk takes is decided by tie-breaks -- deterministic but
// not chosen. Pass a Heading (as int) to choose: the OUT leg leaves the
// start cell through that side, and the HOME leg is forbidden from entering
// through it -- so the lap runs the circuit in the chosen rotational
// direction and still covers every cell. Implemented as temporary walls on
// this COPY of the maze (closed for one flood, reopened for the other), so
// the stored map is never touched. -1 = legacy tie-break behaviour. On a
// maze where the forced side is not a KNOWN-OPEN exit of the start cell the
// force is impossible and the build reports ROUTE_NO_PATH -- the caller
// (RouteRun) retries unforced and says so.
RouteResult route_build_loop(Maze &maze, int x0, int y0, Heading h0,
                             SpeedPath &path, float v_mm_s, float slew_dps2,
                             RouteStats *stats = nullptr,
                             RouteTurn *turns_out = nullptr,
                             int max_turns_out = 0, float lead_in_mm = 0.0f,
                             int force_exit = -1);

// THE GOAL LOOP (bert, 2026-10-04, race morning: "it would be better for
// the mouse to do a loop of the goal, just like it does in the search,
// because the goal may have another entrance opposite the one we come in
// so there will be no wall to back up to"). The alternative to the in-place
// 180 for the home leg of a two-leg run (ghome, Control.h). From the
// arrival cell (gx, gy) heading h_in the walk goes AHEAD into the goal
// block's next cell and from there follows the home flood out through a
// door of the block's far row or far column -- the goal with a second
// entrance, where there is no wall behind the turned mouse to press onto.
// No in-place turn, no datum press: the corners through the block are
// gated on its outer walls and posts as corners always are. A block whose
// only door is the arrival door is REFUSED (the walk would come back
// through the arrival cell: a 270 in two cells of room, which the fit can
// only make as four tight arcs) and the runner turns in place instead.
// Possible only when the block is 2x2 around the arrival cell with
// known-open interior walls (route_goal_loop_possible).
bool route_goal_loop_possible(const Maze &maze, int gx, int gy, Heading h_in);
RouteResult route_build_goal_loop(Maze &maze, int gx, int gy, Heading h_in,
                                  SpeedPath &path, float v_mm_s, float slew_dps2,
                                  RouteStats *stats, RouteTurn *turns_out,
                                  int max_turns_out);
// Where the last route_build_goal_loop's walk ended, and facing which way.
bool route_goal_loop_end(int *ex, int *ey, Heading *eh);
// Would the loop compile from here (a dry run into a scratch path, with the
// time context the home leg will fly)? The runner's loop-or-turn decision.
bool route_goal_loop_viable(Maze &maze, int gx, int gy, Heading h_in,
                            float v_mm_s, float slew_dps2);

// THE TWO-LEG RUN (2026-09-13, "never get a refusal on the known map"):
// when route_build_loop cannot make one walk of it -- a dead-end goal, a
// 2x2 goal with one way in, any maze that is not a circuit through the
// start, which from D11 on is EVERY rules-compliant maze -- the runner flies
// S->G as a leg, stops on the goal cell's centre, turns in place, and flies
// G->S as a second leg compiled from there. This decides the heading that
// second leg launches on: the first step of the home flood from the goal
// cell, given the heading the mouse arrived on (best_step_known prefers
// straight on ties, so a goal block with a through-opening leaves without
// turning). Floods the caller's maze to the start and leaves it so. False
// when there is no known way home from that cell.
bool route_home_heading(Maze &maze, int gx, int gy, Heading arrived,
                        Heading &out);

// Human-readable name for a result, for console and test output.
const char *route_result_name(RouteResult r);

// The tangent a turn of this radius consumes either side of its cell centre,
// at the speed the route is being compiled for. Exposed because the 2T
// question is worth asking BEFORE committing to a radius -- it is what
// tools/route_check.py asks of a map dump.
float route_tangent_mm(float radius_mm, float v_mm_s, float slew_dps2,
                       float sweep_deg = 90.0f);

// Corner-geometry context for subsequent builds (set-before-build state,
// like control_set_path_slew). alat_mm_s2: solve each corner's tangents and
// the U-turn geometry at the speed that corner will actually be flown,
// min(v, sqrt(alat x R)) -- 0 keeps the legacy solve-at-route-speed. At
// rv=500 the difference is negligible; at rv=1300 it is the difference
// between single clothoid 180s and every U decomposing into two squared
// 90s (2026-08-29 lap). uturn_r_mm: the `rur` registry param -- -1 = the
// compiled ROUTE_UTURN_RADIUS_MM, 0 = solve the radius from the rail,
// >0 = force that radius. RouteRun re-sets this from the registry at every
// compile; tests that do not call it get legacy behaviour.
void route_set_corner_context(float alat_mm_s2, float uturn_r_mm);

// Per-direction 90-degree radius override (`r90`/`l90`): replaces the
// ladder's TOP rung only -- 0 = compiled ROUTE_RADII[0], demotions walk the
// compiled ladder below, U-turns keep rur. Set-before-build state like the
// corner context; RouteRun sets it from the registry at every compile.
void route_set_r90(float right_mm, float left_mm);
// The V's outward offset (`vout`, ControlGains::v_out_mm): every V is
// compiled at ROUTE_V_RADIUS_MAX_MM + mm, and its tangents are reported as
// if from the CENTRELINE corner point (the offset lines' own corner point
// is mm further along each leg, so the arc's entry and exit stations do not
// move). The runner pushes the diagonal set point outward by the same mm
// on either side of the V. 0 = the plain V.
void route_set_v_out(float mm);
// rvv / rvu (2026-10-04, the Hazlemere analysis): speed caps on the two
// turn types the day's crashes gathered round. A V and the turn that
// follows it within ROUTE_V_CAP_AFTER_MM of straight (the 135 a V hands
// its 10-12 mm of lateral to -- six of the day's crashes) are built and
// flown at no more than v_cap_mm_s; a U-turn at no more than u_cap_mm_s.
// 0 = no cap. A capped corner carries the cap in RouteTurn.v_cap_mm_s
// and the path's segment vcap, exactly as the fit's own speed rungs do,
// so the planner brakes for it and the clothoid is sized to it. At MR2
// (ralat x0.4: a V flies 320, an R84 367) a 320 cap does not bind; the
// cap is what stops the V climbing the ladder with the straights.
constexpr float ROUTE_V_CAP_AFTER_MM = 120.0f;
void route_set_turn_caps(float v_cap_mm_s, float u_cap_mm_s);
float route_v_cap();
float route_u_cap();

// THE U-TURN THE COMPILER WILL ACTUALLY USE. One place decides whether the
// radius is forced (ROUTE_UTURN_RADIUS_MM) or solved from the rail, and
// everything else -- the compiler, the tests, the console report -- asks
// here. The alternative is what just happened: the test solved its own
// geometry from the rail, the compiler solved a different one from a forced
// radius, and the extent assertion failed on a disagreement that was never
// about the extent.
//
// slew_out, when given, receives the rail the geometry implies. For a forced
// radius that is an OUTPUT worth printing -- it must stay inside
// PATH_OMEGA_SLEW_DPS2 or the executor will clip the ramp the geometry
// assumes.
bool route_uturn_geometry(float v_mm_s, float slew_dps2, SpeedPath::UTurn *out,
                          float *slew_out = nullptr);

// The setback a U-turn of this radius needs before the cell centre it is
// named by: swept envelope, less the wall face, plus the three allowances.
// Exposed so a test or a report quotes the same number the compiler places
// the turn with.
float route_uturn_setback_mm(const SpeedPath::UTurn &u);
