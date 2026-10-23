// Fly a compiled route on the hand start.
//
// This is phase 2: the maze is solved and stored, and the mouse flies it as
// ONE continuous path -- arcs, no stops -- so the sensor stream can be
// recorded against a route it will later race. Until now the only things
// that could fly were the search (stop-turn-go, 300 mm/s, no arcs) and the
// loop sweep (a fixed 900 mm square that has never known what maze it is in;
// see run 154, where `sweep.arm` drove that square straight through the walls
// of a freshly-searched test8).
//
// ---------------------------------------------------------------------------
// WHAT ARMS, AND WHEN
//
// One mission owns the hand start, because both watch the same front beam and
// arming two would make a wave ambiguous. The rule at boot:
//
//     a proven route in FRAM  -> the wave flies the ROUTE
//     no proven route         -> the wave runs a SEARCH
//
// "Proven" means Maze::flood_to_goal(false) -- PESSIMISTIC, unknown edges
// count as walls -- reaches the start. A map with a hole in it anywhere on
// every path to the goal is not a route, and the honest response is to search
// again rather than fly through an edge nobody has looked at.
//
// So the workflow at the maze is: power on, wave (it searches), power cycle,
// wave (it flies what it found). Nothing typed. `h` forces a search back onto
// the hand start when the route is not what is wanted.
//
// ---------------------------------------------------------------------------
// THE RUN: OUT AND BACK, IN ONE GO
//
// A wave flies start -> goal -> start as ONE continuous path with no stop at
// the goal. It used to fly one leg and stop, alternating direction on each
// wave, which was the right way to take the first flights -- but the route is
// proven now (runs 156-175) and half the data was being left on the floor:
// every landmark on the route has a second, different signature seen from the
// other direction, and a wave was only ever collecting one of them.
//
// THE GOAL IS NOT A SPECIAL CASE. The whole out-and-back is compiled as one
// walk -- flood to the goal, then flood home, joined end to end -- so whatever
// heading change sits at the goal is found by the same turn finder as every
// other corner. On test8 there is none at all: the mouse arrives at (2,0)
// heading WEST and the return leaves (2,0) heading WEST, so it simply keeps
// going. Where a goal does need turning, it comes out as a 90 or, if the
// return leaves through the neighbouring corridor, as a U-turn the merger
// picks up like any other. A goal that can only be left BACKWARDS is refused
// (ROUTE_BAD_TURN): no finite radius reverses in a 168 mm corridor.
//
// WHERE IT ENDS, AND WHY THAT NEEDS A HAND. The competition start cell opens
// only to the north, so a mouse returning to it arrives facing SOUTH -- nose
// to the back wall. It cannot set off again from there, and a mission that
// re-armed assuming the start pose would fly the next route 180 degrees
// wrong. So a completed run resets the believed pose to the start convention
// and asks for the mouse to be put back on the start cell facing north. It is
// one physical action, and the mouse has driven itself to within a cell of
// your hand to ask for it.
//
// ---------------------------------------------------------------------------
// THE POST GATE
//
// Every turn is anchored on the END of the wall it turns around -- and in a
// maze that end is a POST, which is the point. Walls come and go between
// layouts; posts are fitted at every cell corner by the regulations, so the
// landmark the mouse navigates by is the one thing guaranteed to be there.
//
// The geometry falls out with nothing left over. The inside wall ends at the
// turn cell's near boundary, half a cell before the corner point. The beam's
// hit point leads the mouse by WALL_GATE_BEAM_AHEAD_MM. So the drop is
// expected at
//
//     corner point - 90 mm (half a cell) - 45 mm (beam lead)
//
// and what is left between seeing it and the arc starting is
//
//     room = 90 + 45 - T
//
// where T is the clothoid-aware tangent. At the operating point -- R84, v500
// -- that is 36.4 mm, and the correction is drained over exactly that, not
// over a fixed fraction. The ladder's other radii have more (R60 54.1,
// R45 61.2), which is a happy accident: the corners forced onto a smaller
// radius by a tight pair are also the ones with the most room to correct in.
//
// SILENT OR CERTAIN. A correction outside the +/-45 mm window is dropped, not
// half-applied. An ungated corner is flown on odometry, which is what every
// corner did before this existed; a WRONG correction is worse than none. V1
// learned that at the cost of a crash.
//
// WHAT A ROUTE ADDS THAT THE RING DID NOT. On the ring the inside wall ran
// continuously to every corner. In a maze it may be absent entirely, leaving
// the bare post -- 22.1 +/- 4.0 mm, about 33 packets at 500 mm/s against a
// present_n of 12. Run 148 gated 52 of 52 post-only approaches, so this is
// measured rather than hoped for. What a route also adds is more chances to
// be wrong: gaps and wall ends earlier on the approach look like edges too,
// which is what live_from_s_mm is for -- the gate OBSERVES from far out and
// is only allowed to DECIDE inside ROUTE_GATE_DECIDE_MM of where it expects
// the post to be.
//
// ---------------------------------------------------------------------------
// THE SECOND TURN OF A TIGHT PAIR CANNOT BE GATED, AND DOES NOT NEED TO BE
//
// The gate for corner k cannot arm until the mouse is clear of corner k-1 --
// during an arc the side beam is not looking at a corridor wall, so feeding
// it there is feeding it junk. That clearance point is
//
//     corner_exit_s(k-1) + SWEEP_JUNCTION_EXIT_MM
//
// and on a pair 180 mm apart it lands PAST corner k's expected drop. Measured
// on test8 at v500:
//
//     outbound  corner 1 (the hairpin's second half): armable at 1052,
//               drop expected at 956 -- 96 mm too late
//     return    4 of its 7 corners, the whole staircase
//
// So the gate is not armed for those corners at all. That is the right
// answer, not a defeat. A gate exists because odometry DRIFTS WITH DISTANCE,
// and a corner 180 mm downstream of one that was just anchored has had 180 mm
// to drift -- which at any slip rate this mouse has ever shown is nothing.
// The position is inherited from the previous corner's correction and is
// still fresh.
//
// It is counted as `inherited` rather than as a miss, because reporting it as
// a failure would train the reader to ignore the one number that matters:
// `no_edge`, a corner that had the room to anchor itself and found nothing.

#pragma once

#include <stdint.h>

#include "Maze.h"   // Heading

// SPEED, RAIL AND ACCELERATION ARE REGISTRY PARAMETERS (2026-08-27):
// rv, rslew, ralon, ralat -- settable over the console, persisted in FRAM,
// compiled defaults in ControlGains (Control.h). The clothoid is
// v^2/(R*alpha), so rv and rslew ARE part of the path's geometry -- which is
// why the leg is compiled fresh at every arm: a route compiled at 500 is not
// the path for 700, and changing rv takes effect on the next wave with the
// geometry recomputed to match.
//
// The constants that used to sit here were DELETED, not kept as
// documentation: a constant named ROUTE_RUN_V_MM_S that the route run no
// longer reads is the compiled-vs-tuned shadowing trap in a new coat.
// Context worth keeping from their comments: ralon defaults to 700 because
// run 153 measured wheel breakaway between 700 and 2000 with nothing closing
// the loop on slip; ralat defaults to 0.5 g as the profile's planning
// ceiling so a dropped-radius corner slows itself rather than asking for
// grip that is not there.

// How far out the gate is fed, and how far out it may act. Observing wide
// costs nothing and lets the detector build its presence run over real wall;
// deciding narrow is what stops an earlier wall end being mistaken for the
// post that gates the turn.
constexpr float ROUTE_GATE_OBSERVE_MM = 500.0f;
constexpr float ROUTE_GATE_DECIDE_MM = 240.0f;

// Fly the whole out-and-back on one wave (see the note above). False reverts
// to the original behaviour: one leg per wave, direction alternating, with a
// controlled stop at each end -- which is still the right mode for the first
// flight of an unproven route or a maze nobody has watched the mouse cross.
// Re-enabled 2026-08-28. It was switched off for the forced-radius
// experiment (whose tight U-turn could not compile on the loop) and the
// flag outlived the experiment: run 188 -- the first CAL-6 flight -- flew a
// perfect S->G leg and then stopped at the goal, which read as "gave up mid
// corridor" until the settings banner said route:S->G. The experiment's
// radius went back to 0 days ago; this goes back with it.
constexpr bool ROUTE_RUN_OUT_AND_BACK = true;

// Runs per wave. Only meaningful with OUT_AND_BACK false, where it counts
// legs; a loop always ends at the start cell needing a hand, so it is one.
constexpr uint8_t ROUTE_RUN_LEGS_PER_WAVE = 1;

// Does FRAM hold a map with a PESSIMISTICALLY proven start->goal route? This
// is what decides whether the hand start flies or searches.
bool route_run_available();

void route_run_set_printer(void (*printer)(const char *line));

// Arm on the hand start. Refuses (and says so) when no proven route exists.
void route_run_arm();
// AFTER A SEARCH (bert, 2026-09-30: "the mouse can stay powered on after
// the search, the lasers can stay on, and the mouse can be ready to re-arm
// for MR2 without a power cycle"). The search has just stopped on the start
// cell's centre facing `arrived`, the map saved and proven: fly the lap
// end's datum sequence from there (turn to the cross heading, back onto the
// side wall, forward to the centreline, turn to the launch heading, back
// onto the rear wall) and arm the route -- MR2, the start gate holding the
// wave. Falls back to a plain arm (placed by hand) if the map is not
// proven or a mission is running. Returns true if it took over.
bool route_run_arm_after_search(Heading arrived);
// Start a run NOW, no hand wave: the GUI's "Start lap". Arms first if idle;
// same settle + IMU recalibration pipeline as a wave. Keep clear of the
// mouse -- it moves ~4 s after this command.
void route_run_go();
void route_run_disarm();
bool route_run_armed();
// route.dir -- which way round a closed-circuit rig the next lap runs, and
// therefore which way the lap LAUNCHES (a launch cannot begin with a turn,
// so the exit side and the launch heading are the same choice).
// 'n' = launch north, west arm out (CAL-6: CW, right-handers),
// 'e' = launch east, east arm out (CAL-6: CCW, left-handers),
// 'a' = auto (legacy tie-break; flies like 'n' on CAL-6).
// Setting it re-orients a mouse parked at the start cell: in place, by
// itself, when it is there after a lap; by instruction to the hand when it
// was hand-placed. Volatile: resets to auto at power-on; the GUI re-sends
// its choice.
void route_run_set_dir(char c);
char route_run_get_dir();
// True from the wave until the leg ends -- the same predicate shape the
// sweep and the search use, so the console can refuse the same way.
bool route_run_running();

// Stop now. Brakes if moving (reverse duty, not a coast -- see
// control_brake_to_stop).
void route_run_abort(const char *why);

// Call every main loop.
void route_run_poll();

// What the mission believes: which leg is next, where the mouse is thought to
// be, and what the compiled route looks like.
void route_run_report();
// The lane-change step test: on the next laps, on the first long two-wall
// straight, step the corridor set point by mm (+ = left) for 250 mm and
// report the response (LSTEP / LS lines). 0 = off. Console 'lstep <mm>'.
void route_run_set_lstep(float mm);

// Gate health for the last leg: how many corners were anchored on a post and
// how many flew on odometry alone. A leg where this is mostly misses is a leg
// whose along-track position is a guess, whatever else the log says.
struct RouteGateStats {
    uint8_t corners;
    uint8_t locked;      // a correction was applied
    uint8_t skipped_lat; // an edge was seen but the mouse was too far off centre
    uint8_t no_edge;     // nothing that could be a wall end was found
    uint8_t inherited;   // too close behind the previous corner to gate, and
                         // not expected to -- see the note above
    float   corr_sum_mm; // signed, for the mean
    float   corr_min_mm;
    float   corr_max_mm;
};
RouteGateStats route_run_gate_stats();
