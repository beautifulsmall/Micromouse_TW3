// Search executor: autonomous flood-fill maze search.
//
// Walks the maze with the classic stop-and-sense loop, wiring the Maze model
// (lib/Maze) to the motion layer (lib/Control). One full 'e' mission is:
// BOOT-ALIGN LEG (below), then search to the goal block, then search back
// to the start (still discovering), then stop, save the map and dump it.
// After a successful mission the PESSIMISTIC flood to the goal tells whether
// a fully-known route exists -- which is what a future fast run will need.
//
// V2: this file is the SEARCH mission only. V1's speed-run executor
// (S_PATH, the route compiler, the wall gates, the observers) is not in this
// baseline -- see docs/V2_BASELINE.md.
//
// Boot-align leg (the standalone 'f' folded into the mission): the mission
// opens with a (0,0)->(0,1) straight up the start corridor, fitting
// (distance, lateral) samples against the continuous left wall (WallAlign)
// with the lateral servo OFF, and re-references the IMU at the far end so
// the corridor axis reads exactly 0. The search proper then starts from
// (0,1). Cell (0,0) needs no sensing: E wall by rules (preset), W/S are
// boundary, N provably open. No separate 'f' required before 'e'.
//
// Straights run CONTINUOUSLY: a leg starts as a one-cell move and is
// extended in flight cell by cell (control_move_extend) for as long as the
// flood keeps pointing straight -- the extend-or-brake decision for each
// next cell happens SEARCH_DECIDE_MARGIN_MM before its centre, one cell
// ahead of the braking point, using the front beam (reliable at that
// range) plus best-effort side evidence. The mouse stops only for turns,
// walls ahead, or phase targets. Bookkeeping happens in flight at each
// cell boundary: traversed-edge lock, position, step budget; side-wall
// windows commit per cell as they complete ("fly" lines in the log).
//
// The stationary cycle below runs at each STOP:
//
//   SETTLE   150 ms stationary (sensor data clean, mouse still)
//   SENSE    burst of packets -> wall presence by majority vote; update the
//            Maze (body-relative -> absolute via current heading), mark
//            visited; phase bookkeeping (goal reached / home again)
//   SHUFFLE  if a front wall is visible and the stationary front distance is
//            off cell-centre by 8..40 mm, a short corrective straight
//            re-centres LONGITUDINALLY (odometry drift reset -- the exact
//            front-wall re-reference the 'b' test proved). > 40 mm means the
//            cell estimate itself is suspect -> abort.
//   TURN     in-place to the ABSOLUTE cardinal of the chosen step (angle =
//            wrap(cardinal - yaw), skipped when < 3 deg). Absolute, not
//            relative: this also absorbs turn undershoot and the standing
//            angle the lateral servo leaves at leg end (the executor
//            captures current yaw at move start -- maze legs must reference
//            the MAZE frame, hardware finding 2026-07-10).
//   MOVE     one cell (180 mm) straight with the lateral wall servo fed per
//            fresh packet (Control.h LAT_SERVO_*); front-wall panic guard
//            (< 55 mm mid-move = unexpected wall = abort).
//
// Heading frame: maze north (+y) = IMU yaw 0 -- RUN 'f' IN THE START
// CORRIDOR FIRST so the corridor axis reads zero. Maze east = yaw -90
// (right-hand turns are CW = negative yaw).
//
// Sensors: TRIGGER mode at 50 Hz during search and returns; FREE-RUN
// during the sprint (S_PATH) for max packet rate. The old <= 5 s free-run
// thermal cap was disproven by bench testing (2026-07-19) -- free-run
// duration is unlimited; triggered search remains as laser-duty hygiene
// and could move to free-run (or interlaced free-run) if the search ever
// wants the rate.
//
// Yaw is NOT re-referenced during the search (V1): the wall-align fit
// assumes a straight path and the lateral servo bends it (mutually
// exclusive by design, Control.h). Boot 'f' + bounded gyro drift
// (~0.01 deg/s) + the servo keeping the mouse centred carry a full search.
//
// Abort: 'e' while running, any move that ends unsettled, a blocked flood,
// the front panic guard, or the 400-step budget. All stop the motors, stop
// the sensors, and print the reason.

#pragma once

#include <Arduino.h>

#include "Maze.h"
#include "Params.h"   // PARAMS_FRAM_ADDR / PARAMS_BLOCK_BYTES: the goal block sits above them

// Line printer injection: Search formats its own lines (no trailing \n
// needed) and hands them to main.cpp's console (USB + radio).
void search_set_printer(void (*printer)(const char *line));

// Debug stream toggle ('D' over serial). While ON, every 20 ms of a
// mission emits one CSV line prefixed "D," with wheel odometry, fused
// motion, IMU heading, servo state, and RAW wall-sensor beams -- the data
// needed to diagnose false walls (raw beams at the moment of the lie) and
// stop overshoot (wheel vs fused vs wall-implied distance):
//   D,ms,state,d,v_enc,v_est,yaw,yaw_err,bias,slip,rawL,rawFL,rawFR,rawR,front,lat
// Additionally (always on) each leg ends with a one-line summary:
//   leg: prof <target> fused <d> wheels <d> | yaw_err | slip <n>
// fused-vs-wheels separates slip/fusion issues from encoder scale; the
// following shuffle line is the wall's verdict on the same stop.
void search_set_debug(bool on);
bool search_debug();

// Start a SEARCH mission from (0,0) facing north (roughly -- the
// boot-align leg measures and corrects the placement error). False (with
// a printed reason) if the attitude isn't ready.
bool search_start();

// START Y in mm from the start cell's south boundary ('starty=', default 51).
// 51 is the back-to-the-rear-wall geometry: 6 mm of wall face plus 45 mm of
// half body. The path carries a lead-in of (90 - starty) so path s=0 lands on
// the mouse's real start pose rather than the cell centre.
//
// WHY IT IS A KNOB. bert's flight viewer reconstructs the wall cloud by
// integrating the logged (s, yaw) from an assumed start pose, and on runs
// 111/112 the cloud lands on the drawn maze at y = 90 -- the CELL CENTRE --
// not at 51. That is a 39 mm along-track bias from the first packet of every
// run, and on the dense path NOTHING corrects it (corner_count = 0,
// path_front_ref = false). The firmware's own geometry is self-consistent and
// host-verified; the disputed quantity is 51 itself, so 51 is what moves.
//
// READ AT PATH COMPILE TIME. Set it before the run, not during.
void search_set_start_y_mm(float mm);
float search_start_y_mm();

// Stop everything mid-mission (motors, sensors); prints the reason.
void search_abort(const char *why);

bool search_running();

// Call every main-loop iteration (drives the state machine + sensor
// triggering; cheap when idle).
void search_poll();

// ASCII map of the model: walls, visited cells, start/goal markers.
// Callable any time (also runs automatically at mission end).
void search_dump_map();

// The maze model.
const Maze &search_maze();
// ...and writable, for the rolling search (Explore.h), which builds the
// same map with the same goal, saves it through search_map_save(), and
// leaves it where route.arm and 'M' expect it.
Maze &search_maze_mut();

// -- Map persistence -----------------------------------------------------------
//
// The searched map survives a power cycle. In V1 that was what made the
// competition flow work -- search once, power off, power on, wave, sprint --
// because the restored map was what the speed run flew.
//
// V2 HAS NO FAST RUN, so read this carefully: the map is still saved and
// still restored at boot, but nothing consumes it yet. search_start() calls
// maze.reset(), so a wave always searches from scratch whatever is in FRAM.
// What the stored map is good for today is 'M' (see what the last search
// found, after a power cycle) and not losing the data before the fast run
// returns. Making a search RESUME from a stored map would be a new
// behaviour -- V1 never did it either -- and wants its own thinking about
// where the mouse believes it is standing.
//
// Saved automatically when a search mission completes, whether or not it
// found a proven route, and the search always ends back at the start, so the
// mouse and the map agree about where it is.
//
// The whole model is stored, lock bits and all (Maze::State), not the
// flattened wall list the flight log carries: whatever eventually flies on a
// restored map must find it exactly as trusted as when it was saved.
void search_map_save();

// Load at boot, after fram_init() and search_set_printer(). A missing,
// stale-version or corrupt block leaves the empty compiled-in map -- never
// half-interprets bytes (same policy as Params and the goal block). Returns
// true if a map was restored.
bool search_map_load();

// Forget the maze: clears the model in RAM and invalidates the FRAM block, so
// the next run is a fresh search. This is what the long hand-start gesture
// does, and 'map=wipe' on the console.
void search_map_wipe();

// Replace the whole wall model with an externally supplied one (the GUI's
// map push -- see the mz.* console commands in Bench.cpp) and persist it,
// exactly as if a search had produced it. The current goal is preserved.
// After this, route_run_available() answers for the pushed map.
void search_maze_adopt(const Maze &m);

// True if the model currently holds a fully-known route start->goal, i.e. a
// speed run is possible right now.
bool search_map_has_route();

// -- Full exploration ---------------------------------------------------------
// OFF gives the classic mission: to the goal, then home, stopping the moment
// a route is proven. That is the right thing in a competition run and the
// wrong thing for a rig, because it routinely leaves a third of the maze
// never looked at.
//
// ON inserts a sweep between the two: after the goal, keep visiting the
// nearest cell that still has an unknown edge until none is REACHABLE, then
// come home. Reachable is the operative word -- an enclosed 6x6 inside a
// 16x16 model leaves 220 cells permanently unknown, and a completion test
// that waited for them would never fire.
//
// Goal first, then explore, then home: a flat battery partway through still
// leaves a proven route, which is the outcome worth protecting.
void search_set_explore_all(bool on);
bool search_explore_all();

// Compiled default: ON (bert, 2026-10-02: "explore all should be default on,
// there are often multiple routes to the goal; known dead ends do not need
// to be searched"). Since that day the rolling search's explore phase is
// ROUTE-AWARE (Explore.h EX_ROUTE_SLACK_CELLS): it visits only unknown cells
// that could lie on a route within the slack of the best known one, so a
// 16x16 is not mapped wall-for-wall. 'E' toggles it (RAM only).
constexpr bool SEARCH_EXPLORE_ALL_DEFAULT = true;

// FRAM block for the map (see the allocation registry in Fram.h).
constexpr uint32_t SEARCH_MAP_FRAM_ADDR = 0x00610;

// Reconfigure the goal rectangle (inclusive; single cell = x0==x1, y0==y1).
// Set once for a practice rig ('goal=x,y' over serial); PERSISTED to FRAM
// immediately (versioned + CRC block at SEARCH_GOAL_FRAM_ADDR), so it
// survives power cycles. Default is the competition block (7..8, 7..8).
void search_set_goal(int x0, int y0, int x1, int y1);

// FRAM block for the goal (see the map registry in Fram.h). 0x100 until
// 2026-09-21, when the params block grew onto it (Fram.h).
constexpr uint32_t SEARCH_GOAL_FRAM_ADDR = 0x00180;
static_assert(PARAMS_FRAM_ADDR + PARAMS_BLOCK_BYTES <= SEARCH_GOAL_FRAM_ADDR,
              "the params block has grown onto the goal block: move the "
              "goal (Fram.h map) before adding this param");

// Load the persisted goal at boot (call after fram_init(), and after
// search_set_printer() so the confirmation line is visible). A missing,
// stale-version, or corrupt block leaves the compiled-in default -- never
// half-interprets bytes (same policy as Params).
void search_goal_load();

// Hand start (race-start protocol; armed at power-on from setup()). While
// armed the front beam streams: a hand held closer than SEARCH_HAND_START_MM
// (debounced) is the gesture. TWO gestures, told apart by HOW LONG the hand
// stays there:
//
//   SHORT  hand in, hand out       -> SEARCH. (V1 launched a speed run here
//                                    instead when the map held a proven
//                                    route; V2 has no fast run yet.)
//   LONG   hand held >= 5 s        -> WIPE the stored map, then search.
//
// Anything removed before the 5 s mark is a RUN, not just the sub-second
// wave: a run is the benign outcome, and wiping a searched maze by accident
// is the expensive one, so the destructive gesture is the one that has to be
// deliberate. The wipe fires at the 5 s mark while the hand is still there,
// so the mouse setting off on a SEARCH rather than a sprint is the
// confirmation that it took -- there is no indicator to flash and the console
// is not where bert's eyes are at a race start.
//
// After the gesture the mouse waits for the hand to CLEAR, then
// SEARCH_HAND_SETTLE_MS of stillness, then RE-RUNS the IMU gyro calibration
// -- the bias window thus runs after the last human touch, superseding the
// power-on cal that happens mid-handling -- then starts the mission.
//
// ARMING HAPPENS AT POWER-ON ONLY (bert, 2026-08-13). A finished mission
// leaves the mouse IDLE: with the map in FRAM there is nothing left to lose by
// switching it off, and switching it off is what you want to do anyway -- to
// clean the wheels between the search and the run that counts. It also
// removes a hazard the old auto-re-arm had, that a hand reaching for a mouse
// stopped in the goal cell reads exactly like the starting gesture.
// Any serial command disarms for bench work; 'h' arms again by hand.
void search_arm_handstart();
void search_disarm_handstart();
bool search_armed();
