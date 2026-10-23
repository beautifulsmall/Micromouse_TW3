// THE ROLLING SEARCH (bert, 2026-09-12): "start the re-write of the search,
// 100 mm/s is enough for a search, the corners can be tight, always inside
// a cell so the walls are seen, and add the log."
//
// WHY A REWRITE. The stop-and-sense search (Search.cpp, July 2026) restarts
// dead reckoning at every stop-turn-go and has no way to notice it has lost
// the cell: on D9 (2026-09-12) it declared the map complete, came home
// along the top row one cell east of where it believed, turned into a wall
// and shuffled against it for ever -- the sensed wall sat on a traversed,
// LOCKED edge, so the map ignored it and the flood kept saying "south". The
// route mission is the mature machine: it flies a SpeedPath, re-anchors on
// every wall end and front wall, and holds the corridor with the lateral
// servo. This module makes the search THAT machine with a one-cell horizon.
//
// THE PATH IS BUILT AS THE WALLS COME IN. The mouse flies a SpeedPath that is
// extended a cell at a time (SpeedPath::add_straight / add_arc are safe to
// call while the executor samples the path: a segment is complete before the
// count that exposes it moves, and the planner re-solves its braking from s
// every tick -- Path.h). The path always ends at the TURN POINT of the cell
// being entered; that is a v = 0 constraint, so an undecided cell is a stop
// inside that cell, never a collision. The decision for the cell -- straight,
// left, right, or back -- extends the path past that point before the mouse
// gets there, and the stop never happens.
//
// GEOMETRY, ALL INSIDE THE CELL. A 90 is an R55 arc with 16 mm clothoids
// between two ~27 mm straights: enter the cell at its boundary, straight to
// the turn point (cell centre - T), the turn, straight to the exit
// boundary. Nothing of the turn leaves the cell, so the side beams are
// looking at THIS cell's walls throughout, and the two straights are where
// they read them. At 100 mm/s the clothoid the rail would ask for is 1.4 mm,
// so the arcs are plain (clothoid length 0) and the rail rounds the step.
// A straight-through cell is 180 mm from turn point to turn point.
//
// WHEN THE DECISION IS MADE. The side lasers' hit point rides ~58-70 mm
// ahead of the mouse (WallGate.h, Search.cpp), so a cell's side walls are in
// view from the mouse at (boundary - 45) -- posts excluded -- and the front
// wall of the cell, 154 mm ahead at (boundary + 20), is well inside the
// front pair's 200 mm. The decision is taken at boundary + EX_DECIDE_MM
// (15): ~60 mm of side evidence, ~130 packets at 100 mm/s, before the
// planner would start braking for the turn point at boundary + 30. If the
// evidence is thin (fewer than EX_MIN_VOTES packets, sensors stale) the
// cell falls back to the old search: straight on to the centre, STOP, a
// stationary burst, turn in place, go -- smooth when it can be, safe when
// it cannot.
//
// LOCALISATION, EVERY CELL. Two WallGates (one per side) are armed at every
// boundary the mouse will cross straight: a corridor wall that ends at that
// boundary drops out of the beam at (boundary + 6 - 58), and the drop is an
// along-track anchor exactly as the route's side gate uses it (silent or
// certain: outside +/-45 it is dropped). A front wall in the cell ahead is
// the second anchor -- the median of (expected - measured) over the approach
// window -- applied once at the decision. Both go through
// control_path_correct, drained over the room left before the turn point.
// The lateral servo holds the corridor from both walls, or one.
//
// THE CONTRADICTION GUARD, which the old search lacked: a front wall inside
// EX_FRONT_PANIC_MM where the path is committed to drive on, both side
// beams under EX_JAM_MM, or a gate whose edge lands outside its window twice
// running, stops the mouse and ends the mission with "lost at (x,y)" --
// the map so far is saved, the flight log says where it went wrong. Never
// the shuffle loop.
//
// THE FRONT WALL IS JUDGED AGAINST THE MAP, NOT A FIXED RANGE (2026-09-26,
// the first search of a new maze: goal (5,5) voted f38/87, "F0", and the
// mouse drove into the goal's front wall at 300 mm/s). The wall stood 31 mm
// farther than the grid put it (the belief was 31 mm ahead of the mouse:
// the right-wall-end anchor two cells back read +9, and every side anchor
// of every flight reads +8..+20 against front anchors of ~0). In the vote
// window the wall is expected 161-194 mm ahead; the old vote needed a beam
// UNDER 200 mm (walls_evaluate's "present" rule), so a wall 31 mm late
// crossed 200 only in the last 40 % of the window -- 38 of 87 packets --
// and lost. FR saturates at ~185 mm, FL reads true to 250 (this log: 250 at
// 256 mm, 225 at 235, 220 at 224). So the vote now takes the nearest RAW
// front beam (255 = none) against expected + EX_FRONT_TOL_MM: the only
// thing that can stand 160-250 mm ahead in a corridor is this cell's front
// wall (the next cell's is 180 mm behind it), so the tolerance costs
// nothing. Three consequences follow:
//   - the front anchor beyond EX_ANCHOR_MAX_MM no longer rolls the turn on
//     the bad belief: the path is left ending at the turn point, the mouse
//     stops there and the stationary burst re-references it (stopped_at_tp
//     already exists for thin votes) -- EXE_FRONT_HOLD;
//   - a SOFT front guard, EX_FRONT_SOFT_MM: a front beam that close where
//     the path is committed straight (or not yet decided) is a contradiction
//     with 40+ mm of margin over anything a legitimate path shows (a wall
//     ahead of a turn cell is 147 mm at the turn point, and the guard is
//     off inside arcs, where the beams sweep the outer corner at ~125).
//     It does not abort: the path is cut to a stop from here (v^2/2a), the
//     wall is written, the burst re-references and decides -- a wasted
//     stop, never a wall -- EXE_FRONT_STOP;
//   - the hard guard EX_FRONT_PANIC_MM moves 55 -> 70: 55 was contact (45)
//     + 10, i.e. the wall already touching at any speed; 70 leaves the
//     hard brake 25 mm at 300 mm/s.
//
// THE HARD GUARD CARRIES ON (bert, 2026-09-26, after it ended the second
// search inside the left turn at (5,5) on FL 68 / FR 255 -- something
// narrow at the (4,5) opening -- with the map complete and the mouse on
// its way home: "the hard front guard should not end in an abort, the
// mouse should carry on if it is going to goal or start"). It is now an
// EMERGENCY STOP, not a verdict: the full-duty brake, then the mouse
// works out where it stands in the cell from the path's own pose at s
// (mid-arc included), turns in place to the heading it was going to leave
// the cell by, and drives on from there with the first straight shortened
// by how far past the centre it already is (path_from_centre's past_mm,
// along the NEW heading). No burst -- at 66 deg of yaw the beams say
// nothing the map can use -- and no wall is written for one stop: a
// single beam at 68 mm is not a wall face. A SECOND stop in the same cell
// is: the edge the mouse keeps trying to leave by is written as a wall
// and the flood goes round; if that edge is one the mouse has driven
// through (locked) the map and the world disagree and the mission ends
// "lost" -- the only abort left in the front guards. EXE_FRONT_HALT.
//
// THE FLIGHT LOG. Same 40-byte FlogRec, same 'y' dump, same FRAM region as
// the route -- but a search at 100 mm/s runs for minutes, and FRAM holds
// ~13k records. So the record is DECIMATED BY DISTANCE: one record per
// EX_LOG_DS_MM (5 mm) of path -- 50 ms apart at 100 mm/s, ~9k records for a
// 45 m search -- plus one record at every EVENT (cell entry, decision,
// anchor, stop, abort) so nothing that changed the mission's mind is between
// samples, plus one every EX_LOG_STILL_MS while stationary so a stop and a
// turn are visible. The observer/prediction fields the route never fills
// carry the search's own state; flog_dump names them by the banner:
//
//    corr   obs_along_mm    along-track correction applied at this record
//    lat    obs_cross_mm    lateral fed to the servo (+ left), 0 if none
//    ev     obs_n           event code (ExploreEvent below), 0 = sample
//    cx,cy  pred_axis_l/r   the believed cell
//    hw     pred_coord_l    heading (bits 8-9) | wall bits: 1 L 2 F 4 R
//                           present, 16 L 32 F 64 R known (this cell)
//    dist   pred_coord_r    flood distance of the believed cell
//    wl,wr  pred_rng_l/r    side votes for THIS cell, % of packets seeing a
//                           wall (255 = no samples yet)
//    fexp   obs_head_cdeg   expected front distance (mm) when the cell is
//                           known to have a front wall, else 0
//    flags                  bit0 in_arc, bit2 correction applied, bit3
//                           lateral valid, bit4 stationary, bit5 sense burst
//
// Registry: `sv` (#55) is the search speed, mm/s (default 100). Everything
// else is compiled below -- geometry, not tuning.

#pragma once

#include <Arduino.h>

#include "Maze.h"
#include "Path.h"

// Turn geometry: the arc, with a clothoid ramp at each end, sits between
// two straights of (90 - T) where T is the primitive's tangent, so the
// whole turn stays inside one cell and the exit lands on the exit
// boundary's centre. R60 plain arcs flew the first two searches (100 and
// 200 mm/s, 2026-09-12); at 200 the curvature STEPS showed: 8-11 deg of
// heading lag at the arc entry and 5-6 deg of overshoot at the exit, held
// for half a cell -- the visible "overturn". The 16 mm clothoids ramp the
// rate over 80 ms at 200 mm/s, which the wheel loop can follow; R55 keeps
// the tangent (63.2) short enough for 26.8 mm of lead, so the decision at
// boundary + 13 still precedes the turn point by more than the 13 mm the
// planner brakes over at 200 mm/s.
constexpr float EX_CELL_MM = 180.0f;
constexpr float EX_TURN_R_MM = 55.0f;
constexpr float EX_TURN_CLOTHOID_MM = 16.0f;
inline float ex_turn_lead_mm() {
    return 0.5f * EX_CELL_MM -
           SpeedPath::tangent_for(EX_TURN_R_MM, EX_TURN_CLOTHOID_MM, 90.0f);
}
// The decision point past the entry boundary, and the vote window that
// feeds it (mouse positions relative to the boundary; the beams lead).
// 20 -> 15 after the first flight (2026-09-12, D9: 41 of 41 decisions
// rolling, votes 0-8 % against 100 %, 55 mm of window is plenty): with
// EX_ACC_MM_S2 1500 the planner starts braking for the turn point 13 mm
// before it at 200 mm/s, i.e. at boundary + 17 -- after the decision, so
// sv 200 rolls through every cell without a dip. 250 dips a little.
constexpr float EX_DECIDE_MM = 13.0f;
constexpr float EX_SIDE_WIN_LO_MM = -45.0f;
constexpr float EX_SIDE_WIN_HI_MM = 13.0f;
constexpr float EX_FRONT_WIN_LO_MM = -20.0f;
constexpr int EX_MIN_VOTES = 6;
// A front beam counts as this cell's front wall when it reads within this
// of the map's expectation (rolling window and stationary burst alike). The
// next cell's wall is 180 mm beyond, so 60 is unambiguous; a belief error
// past it stops the mouse (EXE_FRONT_HOLD / EXE_FRONT_STOP), never a turn.
constexpr float EX_FRONT_TOL_MM = 60.0f;
// Profile limits for the search path. 1500 mm/s^2 is what the route flies
// as ralon; a_lat caps the R60 arc at sqrt(1000 x 60) = 245 mm/s, so above
// sv ~240 the planner slows for the turns itself.
constexpr float EX_ACC_MM_S2 = 1500.0f;
constexpr float EX_ALAT_MM_S2 = 1000.0f;
// In-place turns (dead ends): the registry's `tomg` / `talp`
// (ControlGains::turn_omega_dps / turn_alpha_dps2, default 450/4000 --
// the old search's rates). Shared with the route's goal turn; tunable
// since 2026-09-15 (the recorded goal turn reached 298 of 450 deg/s).
// Wall-end anchor geometry: the drop is seen EX_BEAM_LEAD before the
// post's far face, which is half a post past the grid line. The route's
// WALL_GATE_BEAM_AHEAD_MM (58) was measured at ~1000 mm/s and carries the
// sensor pipeline's latency, ~10 ms = 10 mm at that speed, folded in; at
// 100 mm/s the same latency is 1 mm, and the first flight read every side
// anchor +6..+18 (mean +10.6) against front anchors of -1 +/- 3 where no
// side anchor preceded them. So the lead is 68 less the latency at the
// search speed: 67 at 100, 66 at 200.
constexpr float EX_BEAM_LEAD_BASE_MM = 68.0f;
constexpr float EX_BEAM_LEAD_LATENCY_S = 0.010f;
inline float ex_beam_lead_mm(float v_mm_s) { return EX_BEAM_LEAD_BASE_MM - EX_BEAM_LEAD_LATENCY_S * v_mm_s; }
constexpr float EX_HALF_POST_MM = 6.0f;
constexpr float EX_GATE_ARM_MM = 120.0f;      // before the expected drop
constexpr float EX_ANCHOR_MAX_MM = 40.0f;     // silent or certain
// Guards. SOFT: a front beam (walls_evaluate "present", <= 200) this close
// where the path drives straight on, EX_FRONT_SOFT_PKTS packets running,
// cuts the path to a stop and re-senses. PANIC: the hard brake and "lost".
constexpr float EX_FRONT_SOFT_MM = 130.0f;
constexpr int EX_FRONT_SOFT_PKTS = 3;
constexpr float EX_FRONT_PANIC_MM = 70.0f;
constexpr float EX_JAM_MM = 40.0f;
constexpr int EX_MAX_STEPS = 2000;
// EXPLORE-ALL, ROUTE-AWARE (2026-10-02, Maze::flood_to_useful_unknown): after
// the goal the search visits only unknown cells that could lie on a route no
// more than this many cells longer than the best known one. The slack lets
// a longer route with fewer turns (rturn) be found; dead ends and pockets
// off every such route are never entered.
constexpr int EX_ROUTE_SLACK_CELLS = 4;   // 600 until 2026-10-02: a 16x16 can need more
constexpr uint32_t EX_TIMEOUT_MS = 1200000;   // 20 min: a 16x16 explore-all at 100 mm/s
// Path housekeeping: rebuild the path from the current cell once it holds
// this many segments (sized when the path held 96 / 24 corners; the
// limits are far larger now and the rebase cadence is unchanged).
constexpr int EX_REBASE_SEGS = 40;
// Flight log decimation.
constexpr float EX_LOG_DS_MM = 5.0f;
constexpr uint32_t EX_LOG_STILL_MS = 100;

enum ExploreEvent : uint8_t {
    EXE_SAMPLE = 0,
    EXE_LAUNCH = 1,
    EXE_ENTER = 2,       // crossed into a cell (believed)
    EXE_STRAIGHT = 3,    // decision: on through
    EXE_LEFT = 4,
    EXE_RIGHT = 5,
    EXE_BACK = 6,        // decision: stop at the centre and turn about
    EXE_UNDECIDED = 7,   // thin evidence: stop at the centre and sense
    EXE_ANCHOR_L = 8,    // left wall end locked
    EXE_ANCHOR_R = 9,
    EXE_ANCHOR_F = 10,   // front wall anchor
    EXE_ANCHOR_SKIP = 11, // an edge outside its window, dropped
    EXE_STOP = 12,       // path ended (stationary)
    EXE_SENSE = 13,      // stationary burst result
    EXE_TURN = 14,       // in-place turn started
    EXE_GOAL = 15,
    EXE_MAPPED = 16,     // no reachable unknown edge left
    EXE_HOME = 17,       // mission complete
    EXE_REBASE = 18,     // path rebuilt from the current cell
    EXE_ABORT = 19,
    EXE_FRONT_HOLD = 20, // front anchor beyond the window: stop at the turn point, re-sense
    EXE_FRONT_STOP = 21, // soft front guard: path cut to a stop, re-sense
    EXE_FRONT_HALT = 22, // hard front guard: emergency stop, turn to the exit heading, carry on
};

// Start the rolling search from the start cell, facing north, with the
// mouse's back to the rear wall (starty). Recalibrates the IMU first (3 s
// stationary). False, with a printed reason, if it cannot.
// imu_ready: the caller has just recalibrated (the hand start does), so
// skip this module's own 3 s recalibration.
bool explore_start(bool imu_ready = false);
void explore_abort(const char *why);
bool explore_running();
void explore_poll();
void explore_set_printer(void (*printer)(const char *line));
