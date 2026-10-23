#include "RouteRun.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "Bench.h"       // bench_params_from_fram
#include "Control.h"
#include "DiagLane.h"     // the V exit lane change + the V timing learner
#include "FlightLog.h"
#include "GoffTable.h"   // per-corner learned gate offsets (the mouse's map)
#include "RunLadder.h"   // MR2..MR5: the out-leg peak by run number
#include "HAL.h"
#include "LapReport.h"   // per-corner bookkeeping, written to FRAM at lap end
#include "LoopSweep.h"   // sweep_corridor_lat + the guard constants
#include "Maze.h"
#include "Path.h"
#include "Route.h"
#include "Search.h"
#include "Sensors.h"
#include "StartGate.h"
#include "Vbat.h"
#include "WallGate.h"
#include "Walls.h"

// ---------------------------------------------------------------- printer
static void (*prn)(const char *) = nullptr;
static void printf_line(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void printf_line(const char *fmt, ...) {
    if (!prn) return;
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    prn(buf);
}
void route_run_set_printer(void (*printer)(const char *line)) { prn = printer; }

// ---------------------------------------------------------------- state
//
// The hand protocol is the sweep's, deliberately: same beam, same trigger
// counts, same settle. Two missions that ask for a wave should not feel like
// two different mice.
// The route's speed/accel/rail are REGISTRY PARAMETERS now (rv, ralon,
// ralat, rslew -- bert, 2026-08-27: "all motor, feed forward and pid
// parameters" belong in the registry, and the mission numbers he retunes
// between sessions belong with them). The ROUTE_RUN_* constants in
// RouteRun.h are the compiled DEFAULTS only. The compile-time rail contract
// that used to live here as a static_assert is enforced at runtime in
// compile_leg instead: a rail below what the U-turn geometry assumes is
// floored, loudly.

static constexpr float RR_HAND_START_MM = 100.0f;
static constexpr float RR_HAND_CLEAR_MM = 150.0f;
static constexpr int RR_HAND_TRIGGER_N = 30;
static constexpr int RR_HAND_CLEAR_N = 30;
static constexpr uint32_t RR_HAND_SETTLE_MS = 700;
static constexpr uint32_t RR_TYRE_SETTLE_MS = 300;
// A leg that has not finished this long after launch is not going to. Sized
// off the longest route the compiler will emit (256 cells at 500 mm/s is
// 92 s) with slack, so it only ever catches a genuinely stuck path.
static constexpr uint32_t RR_LEG_TIMEOUT_MS = 120000;
// BLIND-FLIGHT GUARDS (run 208, 2026-08-29). The sub-MCU streams all-255
// packets when its lasers are disarmed/unpowered; run 208's lasers died in
// the launch window and the mouse flew 2.6 m of CAL-6 open loop -- every
// guard below the fresh-packet check needs beams, so nothing objected until
// it hit a wall. Two failure shapes, two guards:
//   SILENT  no packets at all. The seq gate then starves the whole flying
//           body -- lateral/beam/heading aborts AND the flight log -- while
//           the path keeps executing. The stream runs ~580 Hz; 150 ms of
//           silence is a dead link, not jitter.
//   BLIND   packets arrive but every beam reads invalid. A corridor rig
//           always returns something within a cell or two; 600 mm (3 cells)
//           with all four channels dark is a sensor failure, not geometry.
//           (An open-centre full-size maze could near this legitimately --
//           revisit the threshold before flying one.)
static constexpr uint32_t RR_SENS_SILENT_MS = 150;
static constexpr float RR_BLIND_ABORT_MM = 600.0f;
// PRE-FLIGHT SENSOR CHECK (bert, 2026-09-05): "the 255 wall readings are
// from a power-on hardware fault -- if I re-power the mouse too quickly
// after power off the sensors can become inoperative. If any wall sensors
// read 255 at the go, when there should be walls available, the mouse
// should abort and flash the LEDs for 100 ms every 2 s." Laps 53/54 flew
// 600 mm blind before the in-flight guard caught them; the check belongs
// BEFORE the wheels turn. While the mouse stands on its launch cell
// (IMU calibration + tyre settle, ~3.3 s stationary) every packet's side
// beams are tallied against the MAP: a side the map says has a wall must
// return live readings (1..254). A side that never does, over at least
// RR_PREFLIGHT_MIN_PKTS packets, is a dead sensor and the leg is refused
// into RR_FAULT -- LEDs on for RR_FAULT_ON_MS every RR_FAULT_PERIOD_MS
// until route.stop. Only sides with a KNOWN wall are judged: the front
// beams look down an open corridor at launch and cannot be checked here,
// and an unknown or open side proves nothing either way.
static constexpr int RR_PREFLIGHT_MIN_PKTS = 20;
// Too few packets is NOT a pass (bert, 2026-09-18: "only wipe the log after
// a good sensor reading and all sensors valid"). The launch waits in
// RR_SETTLE for the tally to fill -- ~30 ms at the stream rate -- and a
// stream that never delivers is itself a fault after this long.
static constexpr uint32_t RR_PREFLIGHT_WAIT_MS = 2000;
static constexpr uint32_t RR_FAULT_ON_MS = 100;
static constexpr uint32_t RR_FAULT_PERIOD_MS = 2000;
// THE RECOVERY RE-ARM (bert, 2026-10-04, race morning: "after a crash /
// abort wait 20 s and then get ready for another start. This gives me time
// to recover the mouse from the maze and place it back at start for the
// next hand start, so don't power off the sensors -- this way there is no
// power off/on problem"). An aborted leg, a stop between legs and a
// pre-flight sensor fault all used to park the mouse in RR_IDLE (or
// RR_FAULT) until a power cycle -- and a quick power cycle is exactly what
// brings the sensor MCU up stuck at 255 (the start-gate fault, twice this
// morning). Now they enter RR_RECOVER: motors off, sensors left running,
// both LEDs pulsing slowly for RR_RECOVER_MS, then the pose is reset to the
// start-cell convention (0,0 facing NORTH, at the rear-wall setback, a
// hand must place it) and the wave re-arms at the SAME rung -- the ladder
// climbs only on a goal reached, so a crashed MR3 is MR3 again. The hand
// start then does what it always does: IMU calibration on the placed
// mouse, the pre-flight against the map, the start gate's two walls.
static constexpr uint32_t RR_RECOVER_MS = 10000;   // 20 -> 10 s (bert, 10-04 09:40)
static constexpr uint32_t RR_RECOVER_BLINK_MS = 1000;
// EVENT FLASHES IN FLIGHT (bert, 2026-09-12): "use the LEDs to signal key
// events during the run ... default off for the run except for signalling
// flashes." Both LEDs are OFF from launch to the leg end. LED1 flashes
// when an along-track anchor is taken from a WALL END -- a side gate lock,
// or the front gate acting (fgate 2, incl. its fallback); LED2 flashes when
// the diagonal observer locks a POST. 40 ms is ~40 mm at speed, long enough
// for an eye or a camera frame; a second event inside the window restarts
// it. Serviced once per flight packet in route_run_poll.
static constexpr uint32_t RR_LED_FLASH_MS = 40;
// THE FLIGHT LOG, DECIMATED BY DISTANCE (bert, 2026-09-14: "is there a way
// to fit a single flight log of a 16x16 maze into the 4 Mbit memory").
// Every sensor packet (~500 Hz) was a record: 26 s of region, and a 16x16
// two-leg run at MR2's rv is 35-80 s -- the ring overwrote its own launch.
// Now, as the search already does (Explore.h): one record per RR_LOG_DS
// mm of path, plus EVERY event packet (a gate correction, a front gate
// firing, a diagonal lock, an arc entry or exit -- the packets that
// changed something), plus one per RR_LOG_STILL_MS while stationary. The
// spacing is sized at launch from the compiled leg so the whole run takes
// RR_LOG_BUDGET of the region: ds = max(floor, legs x length / budget). At
// the 2 mm floor a 1000 mm/s straight is every packet and a 640 mm/s
// corner every 1.6 ms, so a 6x6 rig's log is unchanged from full rate; a
// typical 16x16 two-leg run (30 m) sits at ~2.7 mm; a pathological 72 m
// run gets 6.2 mm and still fits. The capacity ladder (FlightLog.h)
// backs it up. Log size is now a function of DISTANCE, which the maze
// bounds, not of time, which the rung does not.
static constexpr float RR_LOG_DS_MIN_MM = 2.0f;
static constexpr uint32_t RR_LOG_STILL_MS = 100;
static constexpr float RR_LOG_BUDGET = 0.90f;
static constexpr float RR_LOG_PANIC_FILL = 0.95f;   // past this, spacing x4
static float log_ds_mm = RR_LOG_DS_MIN_MM;
static float log_last_s = -1e9f;
static uint32_t log_last_ms = 0;
static bool log_event = false;      // set by anything worth a record
static bool log_last_in_arc = false;
// ANCHOR-AGE WINDOW (bert, 2026-08-29): "the gate accept window can be half
// a cell, no benefit in making it too small... it should widen with
// accumulated odometry uncertainty." The map is pinned to the last locked
// post; everything since rides odometry, so the accept window starts at the
// proven 45 and grows 4% of the distance dead-reckoned since that post,
// capped at half a cell -- edges repeat at the 180 mm cell pitch, so +/-90
// is the aliasing limit where a wrong cell's edge becomes admissible. A
// fresh anchor keeps the tight window (the decoy-edge defence); a stale one
// stops refusing the very correction it needs most. 4% is generous against
// the 0.24%-calibrated wheels; the margin covers braking slip and corner
// scrub, which the j-runs never measured.
// ---- REAR-WALL DATUM AT THE LAP BOUNDARY -------------------------------
//
// A lap used to end "on the cell centre" -- by the mouse's own reckoning,
// which is exactly the quantity in doubt. corner 0's corr IS the launch
// error (it is the first gate of the lap, so it measures everything that
// accumulated since s=0), and pooled over the three runs of 2026-08-31 by
// lap index within the run:
//
//   lap 1  +20.3   <- hand placed at the rear wall
//   lap 2  -25.2   <- "cell centre", assumed
//   lap 3  -16.8
//   lap 4   -8.3
//   lap 5   -6.3
//   lap 6   -2.0
//
// Lap 2 is the worst lap of every run -- worse than the hand-placed one --
// and it takes four laps to wash out. Every one of those laps teaches the
// goff table. The yaw residual was already trued up at the lap boundary;
// the along-track one never was.
//
// bert's fix, and it is better than measuring: "a new lap should always be
// from the rear wall, its the most repeatable." The rear wall is a
// MECHANICAL datum -- it does not depend on odometry, on a LUT, or on a
// beam, and pressing the back panel flat against it is exactly what a hand
// placement does. So the mouse now backs into it after the lap turn and
// launches from at_start_setback like lap 1 does, which merges the two
// start populations the table was being asked to straddle.
//
// Commanded distance covers the nominal 39 mm gap plus the worst observed
// stop error, so the wall is always reached; contact is detected as a
// stall (the wheels stop while the profile still asks for speed) and the
// press is cut ~60 ms later, so the motors never sit into the duty limit.
// If no stall arrives in the full RR_BACKUP_MM, there was no wall where
// one is guaranteed -- the mouse does not know where it is, and says so
// rather than claiming a datum it did not touch.
// 85 until 2026-09-10: with the braking-slip model on (sbrk 40) the lap's
// final stop lands ~25-45 mm short of the cell centre (the stopping
// deceleration is modelled as slipping like the 1070 -> 580 corner
// braking does, which it does not), the cross-wall press then ran out of
// travel 85 mm in -- "no WEST wall inside 85 mm" on every lap -- and the
// forward-to-centreline step was skipped. The press is the mechanical
// datum: it must reach the wall from any plausible stop, so it may travel
// 130 (1.1 s at 120 mm/s); a wall further than that really is missing.
static constexpr float RR_BACKUP_MM = 130.0f;
static constexpr float RR_BACKUP_V = 120.0f;
static constexpr float RR_BACKUP_ACC = 1500.0f;
static constexpr float RR_BACKUP_STALL_MM_S = 25.0f;
static constexpr uint32_t RR_BACKUP_STALL_MS = 60;
static constexpr uint32_t RR_BACKUP_MAX_MS = 2000;
// Ignore the first few mm: the profile starts from rest, so v is legitimately
// under the stall threshold before it has accelerated.
static constexpr float RR_BACKUP_ARM_MM = 6.0f;
// THE FAR GOAL DATUM (bert, 2026-10-04, race morning: "the mouse always
// stops short in the goal cell, so after the turn on this maze it sees no
// walls"). The goal stop is dead-reckoned from the last corner, and the
// wheels over-count the accelerations out of the faster corners (the
// corner slip instrument at the last corner read -2 at MR2, -6.5 at MR3,
// -8 at MR4, -12.5 at MR5), so the stop falls short by a growing amount
// and the home leg -- launched from that pose -- flies every corner early
// (Hazlemere M1 MR3 home: "arc ~60 mm early"). A wall directly behind the
// turned mouse was already pressed for (the goal datum); now the first
// KNOWN wall behind within RR_GOAL_DATUM_MAX_CELLS cells of known-open
// cells is pressed for too -- in a 2x2 goal that is the block's far side,
// one cell back. The press is slow (RR_BACKUP_V) and self-correcting: it
// ends where the wall is, whatever the stop did. The pose moves to the
// cell against that wall and the home leg compiles from there with the
// lead-in, exactly as a lap end does at (0,0).
static constexpr int RR_GOAL_DATUM_MAX_CELLS = 3;
static float backup_mm = RR_BACKUP_MM;   // this press's travel allowance
static int goal_datum_cells = 1;         // cells back to the wall being pressed for

static constexpr float RR_GATE_WINDOW_BASE_MM = 45.0f;
static constexpr float RR_GATE_WINDOW_GROW = 0.04f;
static constexpr float RR_GATE_WINDOW_MAX_MM = 90.0f;
// FRONT-WALL GATE (bert, 2026-08-30): "the front wall is seen before a wall
// end and is more orthogonal so data is more consistent across lateral
// error." Corners whose turn cell has a wall dead ahead (RouteTurn
// front_ok) anchor on that wall's RANGE instead of the side wall-end EDGE:
// front_mm is forward-from-centre (the LUT bakes in mount + angle), so the
// believed-frame error is simply (face_s - s) - front_mm, sampled every
// packet on the approach -- a ramp, not an event. The median of the last
// FGATE_RING samples fires once the wall is FGATE_APPLY_MM out; the spread
// test is the phantom killer (ch1's dark-current wall reads a CONSTANT
// range, so its implied error slides 1:1 with s and the spread blows up).
// fgate=0 disables (registry); side-edge gating is untouched for corners
// with no front wall or when the front data fails its own checks.
// THE GEOMETRY, MEASURED (2026-08-30, after laps 189-195 fired the front
// gate exactly zero times): at rv1500 the clothoid tangent is 132.6 mm, so
// the ARC ENTRY sits 132.6 mm before the cell centre and the wall face is
// 84 mm beyond it -- the wall is still 216.6 mm away when the turn starts,
// and the mouse never gets closer while going straight. The first build
// sampled only under walls_evaluate's 200 mm presence flag and decided at
// 110-150 mm: a window the approach does not contain. Zero samples, ever.
//
// The window that EXISTS is 216 mm out to the sensor's own ceiling (front
// beams are uint8 mm; 255 = out of range), so the gate reads the RAW front
// beams with its own cap rather than wv.front, and fires as soon as the
// ring is full -- which is ~30 mm before the arc entry, EARLIER than the
// side gate manages at this speed (the side edge's expected drop has crept
// to within ~2 mm of the entry as the tangent grew, floored to 5 mm of
// drain room; the front gate drains over its own real remaining room).
// At lower speeds the tangent shrinks and the window only gets wider.
static constexpr float FGATE_MAX_RANGE_MM = 250.0f;  // sensor ceiling, off the rail
//
// WHICH INSTRUMENT WAS WRONG -- the SIDE gate, resolved 2026-08-30.
// The front beam is honest to ~3 mm at 216-254 mm; the 13.5 mm belonged to
// WALL_GATE_BEAM_AHEAD_MM, which was 45 (the chassis half-length) against a
// measured spot lead of 64. See the derivation at that constant. With the
// lead corrected the two gates share a datum and ftrim goes to 0.
static constexpr int FGATE_RING = 8;
static constexpr int FGATE_MIN_N = 5;
static constexpr float FGATE_SPAN_MM = 15.0f;    // travel since acquisition
static constexpr float FGATE_DRIFT_MM = 10.0f;   // error movement over it
static constexpr float FGATE_SPREAD_MM = 12.0f;  // scatter within the ring
// HOW LATE THE FALLBACK MAY STILL ACT: mm before the arc entry. The side
// gate is the PRIMARY key (bert, 2026-08-30, from the flight viewer: "the
// front data does only come in after the side wall gate, so unless the
// front sensor can measure better at ~230mm the side gate is the more
// reliable key"). The timing bears it out -- with the lead at 58 the side
// drop lands at datum-148 while the front wall only becomes readable around
// datum-156..-166, so they are near-simultaneous and the front is often
// second. So is the precision: on the same population (corner 0, hand
// placed, true error near constant) the side repeats to sd 0.8 mm and the
// front to 1.4. The earlier "front is far better" reading compared the
// front against side scatter that was mostly REAL lap-to-lap position
// variation, not measurement noise.
//
// What the front does own is COVERAGE: the corners where the side gate
// finds no edge at all -- (1,3), (5,0), (5,5) CCW -- currently fly on an
// inherited anchor, and the front wall has an answer for them. Hence the
// fallback: measure always, act only where the side gate ran out of
// approach without finding its edge. "the front is still seen before a
// turn is started sometimes" -- the arc entry is at datum-133, so there is
// ~25 mm of margin to act in, which is what this deadline spends.
static constexpr float FGATE_FALLBACK_MM = 12.0f;
// FUSION (bert, 2026-08-30: "both methods... can we fuse them"). After the
// front gate corrects the frame, the SIDE gate stays armed: its edge
// should now drop almost exactly on expectation (offset only by the
// deliberate atrim, which the side branch subtracts again as placement).
// If it drops, the residual refines the anchor and the corner reports
// FUSED; the printed front-vs-side residual is the standing measurement of
// the two landmarks against each other -- and a loud residual is a decoy
// or map error caught by cross-check. If the side never drops, FRONT
// stands alone. The learner learns from the FRONT error only: the two
// features are different physical objects, and one table entry must chase
// one target.

enum RrState : uint8_t {
    RR_IDLE,
    RR_ARMED,
    RR_HAND_CLEAR,
    RR_CAL,
    RR_SETTLE,
    RR_FLY,
    RR_LAPTURN,   // in-place turn to the next launch heading (lap end, or
                  // a route.dir change while parked at the start cell)
    RR_GOALTURN,  // two-leg run: in-place turn on the goal cell's centre to
                  // the home leg's launch heading (then RR_SETTLE -> leg 2)
    RR_BACKUP,    // pressing back onto the rear wall for the launch datum
    RR_DATUM_FWD, // off the cross wall to the centreline (two-wall datum)
    RR_FAULT,     // pre-flight sensor check failed: LEDs flash, nothing
                  // flies until route.stop (see preflight_* below)
    RR_RECOVER,   // after an abort or a fault: RR_RECOVER_MS for a hand to
                  // put the mouse back on the start cell, then re-arm
};
static RrState state = RR_IDLE;
static uint32_t state_t0 = 0;
static uint32_t last_seq = 0;
static int hand_run = 0;
static bool hand_active = false;
static uint32_t hand_seen_ms = 0;
static bool hand_cal_started = false;
static uint32_t arm_t0 = 0;             // when the wave was armed
static bool arm_silent_said = false;    // the "no packets" warning, once per arm
// HOLD TO WIPE, on the route's hand start too (bert, 2026-09-15: held a
// hand 7 s expecting a search and got MR2 -- a proven map arms the ROUTE
// at power-on, and only the search's hand start knew the 5 s gesture).
// Now the gesture means the same whichever mission owns the wave: hold
// RR_HAND_WIPE_MS and the mouse wipes the stored map, clears the learned
// gate offsets (maze property), and hands the wave to the SEARCH with the
// hand still there -- remove it and MR1 launches. "Prepare for a new
// unknown maze": the safest reading of a long hold.
static constexpr uint32_t RR_HAND_WIPE_MS = 5000;
static uint32_t hand_t0 = 0;
static uint32_t hand_next_tick = 1;
static uint32_t led_last = 0;
static bool led_phase = false;
static uint32_t led1_off_ms = 0, led2_off_ms = 0;   // flash deadlines, 0 = dark
static void led_flash(int which) {
    log_event = true;   // an LED flash IS an event: log this packet
    uint32_t off = millis() + RR_LED_FLASH_MS;
    if (which == 1) { hal_set_led1(true); led1_off_ms = off; }
    else            { hal_set_led2(true); led2_off_ms = off; }
}
static void led_flash_service(uint32_t now) {
    if (led1_off_ms && (int32_t)(now - led1_off_ms) >= 0) { hal_set_led1(false); led1_off_ms = 0; }
    if (led2_off_ms && (int32_t)(now - led2_off_ms) >= 0) { hal_set_led2(false); led2_off_ms = 0; }
}
static void led_dark() {
    hal_set_led1(false); hal_set_led2(false);
    led1_off_ms = led2_off_ms = 0;
    led_phase = false;
}
static int pf_pkts = 0;          // packets tallied while stationary
static int pf_live_l = 0, pf_live_r = 0;   // packets with a live beam
static void preflight_reset() { pf_pkts = pf_live_l = pf_live_r = 0; }
// Live side readings on the leg just FLOWN (reset at arm and at launch).
// A beam that read walls all the way to the goal and reads 255 at the
// goal stop is not dead: the mouse is not where the leg thinks it is
// (2026-10-04, the small maze: MR3/MR4/MR5 all stopped at the goal with
// the left beam on 255 where the map has a wall, after 1800 live readings
// on the way there; MR2 read 85 at the same spot).
static int fly_live_l = 0, fly_live_r = 0;
static constexpr int RR_BEAM_PROVEN_PKTS = 100;
static void preflight_sample(const WallRanges &w) {
    pf_pkts++;
    if (w.left_mm > 0 && w.left_mm < 255) pf_live_l++;
    if (w.right_mm > 0 && w.right_mm < 255) pf_live_r++;
}

// Where the mission believes the mouse is, and which way the next leg goes.
// The start pose is the competition one and is not negotiable: (0,0) facing
// NORTH, back to the rear wall. Everything after that the mission derives
// from the route it just flew.
static uint8_t pose_x = 0, pose_y = 0;
static Heading pose_h = HEADING_N;
static bool leg_to_goal = true;
static uint8_t legs_left = 0;
// THE RUN LADDER (RunLadder.h). cur_mr is this run's rung, taken from the
// FRAM counter at the first leg's launch, 0 = ladder off; the out leg
// flies at rv2..rv5 by rung, the home leg at rvh, and with the ladder off
// every leg flies at the hand-set rv as before.
//
// THE RUNG IS TAKEN AFTER PRE-FLIGHT, NOT AT THE WAVE (bert, 2026-09-18).
// The first MR1->MR5 test lost MR3: the mouse was powered on too soon, the
// left beam read 255 for every packet, pre-flight refused the launch --
// and the counter had already advanced, so the next wave flew MR4 and the
// 1000 mm/s run was never flown. A run that never left the line was not a
// run. So RR_CAL only marks the wave as owing a rung (rung_pending); the
// take happens in leg_launch once preflight_ok() has passed, and the same
// gate is what arms the flight log -- nothing in FRAM (log, ladder) changes
// on a wave the sensors then veto. A crash IN FLIGHT still costs the rung,
// as on the day.
static uint8_t cur_mr = 0;
static bool rung_pending = false;
static float leg_peak_v = 0.0f;   // the peak this leg was compiled for
static bool home_leg();
static float leg_speed() {
    ControlGains g = control_get_gains();
    if (cur_mr == 0) return g.route_v;
    return home_leg() ? run_ladder_home_v(cur_mr, g) : run_ladder_out_v(cur_mr, g);
}
// THE TWO-LEG RUN (2026-09-13). ROUTE_RUN_OUT_AND_BACK asks for one walk
// S->G->S; on a maze that is not a circuit through the start -- a dead-end
// goal, a 2x2 goal with one way in, every rules-compliant maze from D11 on
// -- route_build_loop ends in an about-face at the goal (ROUTE_BAD_TURN)
// and the mouse used to refuse to fly. Now the compile falls back to two
// legs: S->G compiled one-way (the leg machinery from the CAL-6 days,
// leg_to_goal / legs_left, which a loop never used), a stop on the goal
// cell's centre, an in-place turn to the home flood's first step
// (route_home_heading), and G->S compiled FROM THE GOAL and flown as the
// second leg. One wave, one flight log (flog_resume across the turn), two
// lap reports (dir 'o' and 'h'), and the lap end at (0,0) is the usual
// datum sequence. The circuit is still tried first: a maze that has one
// flies through the goal as before.
// two_leg: this run took the fallback. leg_to_goal says which leg is next.
static bool two_leg = false;
static Heading goal_turn_h = HEADING_N;   // where RR_GOALTURN is turning TO
// The goal turn: the old search's in-place rates (Search.cpp, 450/4000:
// faster turns scrub symmetrically), on a cell centre with the central post
// or a front wall 90 mm away.
// 450/4000 on the first two flights (D9, 2026-09-15): bert -- "the 180 at
// the goal cell drifted to one side; it looked slower than the 180 in the
// alcove, which stayed more central". The alcove 180 is the lap-end datum
// turn at 360/2000 (datum_turn_to), so the goal turn now uses the same.
// 2026-09-15 evening, third and fourth flights: 360/2000 drifted MORE
// ("very offset drift", the second run's 180 failed) and bert's eye says
// the SEARCH's dead-end 180s (Explore.cpp: 450/4000, velocity loop never
// released, 150 ms settle) are the best turns the mouse makes. So the goal
// stop is now the search's stop: the loop stays enabled holding zero (no
// brake pulse, no STBY cycle), RR_GOAL_SETTLE_MS of settle, then the turn
// at the search's rates -- and the turn and the press are RECORDED (flags
// bit4, obs_along/obs_cross = left/right wheel velocity, s = odometry) so
// the drift can be read instead of guessed at.
// The rates are the registry's `tomg` / `talp` (shared with the search's
// dead-end turns; ControlGains) -- the first RECORDED goal turn (D9,
// 2026-09-15, 450/4000): plateau mean 298 deg/s, sd 99, min 124, against
// 450 asked, wheels at the odometric equivalent of 428 -- ~30 % slip in
// place; the side beam put the body 26 mm further from the north wall
// after the turn than before, and the press then ran 110 mm where the
// front beam had promised 77: ~30 mm forward as well. A rotation about a
// point ~20 mm off the body centre. The rate the floor delivers is what
// tomorrow's session tunes.
static constexpr uint32_t RR_GOAL_TURN_MAX_MS = 4000;
static constexpr uint32_t RR_GOAL_SETTLE_MS = 150;
// The front beam at the out leg's stop (centre-referenced LUT), so the
// press can be checked: the panel should meet the wall after
// (front_at_stop - starty) mm, and a stall well before that is the body
// caught on something else, not a datum.
static float goal_front_mm = 0.0f;
static constexpr float RR_GOAL_PRESS_TOL_MM = 25.0f;
static uint32_t goal_settle_t0 = 0;
static bool goal_turn_pending = false;   // settle running, turn not yet started
static float goal_turn_deg = 0.0f;
static float goal_stop_d_mm = 0.0f;      // odometry at the goal stop (turn log datum)
// THE GOAL DATUM (2026-09-15). The out leg's final stop lands SHORT of the
// cell centre (the braking-slip model, sbrk -- see RR_BACKUP_MM's note:
// 25-45 mm on a lap end), and the first two two-leg flights measured it
// on the home leg's first corner: LOCK +37 / +16 mm with the front gate
// agreeing (+34 / +15.5), lateral -23 / -39 -- and three corners later
// the home V had no margin left ("also hit inner V on return"). The lap
// end fixes the same error with a wall press; the goal gets one too: after
// the turn, if the map KNOWS a wall on the side now behind the mouse, back
// onto it exactly as the lap end does (RR_BACKUP), and the home leg
// launches from the setback with the same 39 mm lead-in a hand-placed
// start takes -- along-track and yaw datumed by the wall. No known wall
// behind: launch dead-reckoned as before, and say so. The home leg is not
// the scored leg, so the second and a half this costs is free.
static bool goal_datum = false;   // RR_BACKUP is the goal press, not a lap end
static bool goal_loop = false;    // ghome 1: this home leg loops the goal block (Route.h)
// A leg's name for the console and the banner.
static const char *leg_name_long() {
    if (!ROUTE_RUN_OUT_AND_BACK || two_leg)
        return leg_to_goal ? "START -> GOAL" : "GOAL -> START";
    return "START -> GOAL -> START";
}
static const char *leg_name_short() {
    if (!ROUTE_RUN_OUT_AND_BACK || two_leg)
        return leg_to_goal ? "route:S->G" : "route:G->S";
    return "route:S->G->S";
}
// The home leg of a two-leg run: launched from the goal, no recalibration,
// the yaw frame still the out leg's.
static bool home_leg() { return two_leg && !leg_to_goal; }
// TRUE only while the mouse is where a human put it: on the start cell with
// its back to the rear wall, 51 mm in rather than on the 90 mm centre. The
// compiled path starts at a CELL CENTRE, so that first leg needs 39 mm of
// lead-in or every landmark on the route sits 39 mm from where the path
// believes it is. A leg that ENDS somewhere ends on a centre, so this is
// cleared the moment the mouse moves under its own power and is only ever set
// again by a power cycle.
static bool at_start_setback = true;

// route.dir -- which way round a closed-circuit rig the lap runs. -1 = auto
// (legacy tie-break, west arm out on CAL-6). Otherwise the Heading the lap
// LAUNCHES on and the OUT leg exits the start cell through: HEADING_N or
// HEADING_E (the start is a corner cell; S and W are the outside). The two
// are the same thing on purpose: a launch cannot begin with a turn (no
// run-up in the start cell), so choosing the exit side IS choosing which
// way the nose points at launch -- the lap-end turn aims there, and a
// hand-placed first lap must be placed facing it (dir=e: back to the WEST
// wall). Volatile by design: the GUI sets it per session, and a power cycle
// returns to the known default rather than a stale saved preference. See
// route_build_loop's force_exit.
static int route_dir = -1;

char route_run_get_dir() {
    return route_dir < 0 ? 'a' : (route_dir == HEADING_N ? 'n' : 'e');
}

PATH_STORAGE static SpeedPath path;   // RAM2 (Path.h)
static RouteStats stats;
static RouteTurn turns[ROUTE_MAX_TURNS];
static uint8_t n_turns = 0;
// Cell and heading the compiled leg ENDS on -- applied to the pose only once
// the leg actually completes, so an aborted leg leaves the mission believing
// what it can still see rather than where it meant to get to.
static uint8_t end_x = 0, end_y = 0;
static Heading end_h = HEADING_N;
// The cardinal the CURRENT leg launched on. The fused yaw is re-zeroed at
// every launch with the nose on this cardinal, so every yaw read during and
// after the leg is RELATIVE TO IT -- all frame arithmetic below rotates
// through launch_h, never assumes 0 = north.
static Heading launch_h = HEADING_N;
// Where the in-place RR_LAPTURN is turning TO (a lap end turns to the
// route.dir launch heading; a route.dir change at the start cell turns
// likewise). Applied to pose_h only when the turn completes.
static Heading lapturn_target = HEADING_N;
// A lap end presses back onto the rear wall afterwards; a route.dir change
// at the start cell does not (the mouse is parked, wherever a hand left it).
static bool lapturn_backup = false;
// Rear-wall press bookkeeping (see the RR_BACKUP constants).
static uint32_t backup_stall_t0 = 0;
// THE TWO-WALL DATUM (2026-09-05). A single rear-wall press datums the
// launch axis only; the lateral is whatever the pivot inherited. For the
// north launch that is the x of the west-approach stop, anchored by the
// 45-out posts 300 mm earlier -- corner-0 lat +/-3. For the EAST launch it
// is y: after a CW lap that is the SOUTH-wall press datum itself, 39 mm
// south of the centreline, and after a CCW lap the dead-reckoned stop from
// a usually-NO_EDGE (0,5). Laps 34-36 entered the diagonal at lat -33..-35
// every time, lost posts 1-3, and dragged the whole CCW lap down with it.
// So a lap end now presses BOTH walls: turn to the cross heading, press
// that wall, drive forward to the centreline (the same 39 mm the lead-in
// uses), turn to the launch heading, press the rear wall. The start cell
// has walls on three sides by rule, so this holds on any maze. Phases:
// 1 = cross press, 2 = forward, 3 = final press; 0 = legacy single press.
static int datum_phase = 0;
static Heading datum_want = HEADING_N;
static Heading cross_heading(Heading want) {
    return want == HEADING_E ? HEADING_N : HEADING_E;
}
static bool start_rear_wall_press(uint32_t now, float extra_mm = 0.0f);
static float yaw_of_heading(Heading h);
static const char *h_name(Heading h);
// Turn in place to `target` (or press straight away if already facing it)
// as datum phase `phase`; the press follows the turn (RR_LAPTURN -> RR_BACKUP).
static void datum_turn_to(Heading target, int phase, uint32_t now) {
    datum_phase = phase;
    float yaw_now = control_heading().yaw_deg;
    while (yaw_now > 180.0f) yaw_now -= 360.0f;
    while (yaw_now < -180.0f) yaw_now += 360.0f;
    float turn = yaw_of_heading(target) - yaw_now;
    while (turn > 180.0f) turn -= 360.0f;
    while (turn < -180.0f) turn += 360.0f;
    control_velocity_enable(true);
    if (fabsf(turn) > 1.0f) {
        if (control_turn_start(turn, 360.0f, 2000.0f)) {
            printf_line("route: datum %d/3 -- turning %.0f deg to %s",
                        phase == 1 ? 1 : 3, (double)turn, h_name(target));
            lapturn_target = target;
            lapturn_backup = true;
            state = RR_LAPTURN;
            state_t0 = now;
            return;
        }
        control_velocity_enable(false);
        state = RR_IDLE;
        printf_line("route: datum turn REFUSED -- facing %s. Place the "
                    "mouse facing %s before the next lap.", h_name(pose_h),
                    h_name(datum_want));
        route_run_arm();
        return;
    }
    pose_h = target;
    if (start_rear_wall_press(now)) return;
    control_velocity_enable(false);
    state = RR_IDLE;
    route_run_arm();
}
// Begin the whole sequence toward launch heading `want`.
static void datum_begin(Heading want, uint32_t now) {
    datum_want = want;
    datum_turn_to(cross_heading(want), 1, now);
}

// YAW AT THE ARC'S TRUE END, for the lap report's sweep measurement.
//
// The corner CLOSES at exit + SWEEP_JUNCTION_EXIT_MM (60 mm), which is the
// right boundary for the wall model -- it is sized so the mouse is clear of
// the corner cell whatever the radius. It is the wrong place to measure the
// SWEEP, because it assumes the next corner is far away.
//
// On the D6 zigzag it is not: R60 and R45 arcs one cell apart leave 19-21 mm
// of straight between them (measured, laps 284-287), so the close boundary
// lands ~40 mm INSIDE the next arc -- about 50 deg of it at 650 deg/s. The
// reported sweep error then picked up ~50 deg of the NEXT, opposite-sign
// arc and pinned the +-12.7 clamp, alternating with the turn hand, on every
// zigzag corner of every lap. The arcs themselves were fine: integrating
// yaw over each one from the flight log gives -83, +97, -96, +92, -93, +94,
// -93 against a +-90 command. The mouse was flying it; the ruler was bent.
//
// So sample the yaw where the arc actually ends and report the sweep from
// that. Measurement only -- the close boundary, the gate and the junction
// rules are untouched.
static float corner_exit_yaw = 0.0f;
static bool corner_exit_yaw_valid = false;


static const char *h_name(Heading h) {
    return h == HEADING_N ? "NORTH"
           : h == HEADING_E ? "EAST"
           : h == HEADING_S ? "SOUTH"
                            : "WEST";
}

// Press back onto the rear wall to re-establish the launch datum (see the
// RR_BACKUP constants). The caller has already set pose_h. Returns true if
// the press started and RR_BACKUP now owns the sequence.
static bool start_rear_wall_press(uint32_t now, float extra_mm) {
    control_motion_zero();
    control_velocity_enable(true);
    backup_mm = RR_BACKUP_MM + extra_mm;
    if (!control_move_start(-backup_mm, RR_BACKUP_V, RR_BACKUP_ACC)) {
        control_velocity_enable(false);
        printf_line("route: rear-wall press REFUSED -- this lap keeps the "
                    "old cell-centre datum");
        return false;
    }
    backup_stall_t0 = 0;
    state = RR_BACKUP;
    state_t0 = now;
    printf_line("route: backing onto the rear wall for the launch datum");
    return true;
}
// The launch heading route.dir asks for ('a' behaves as north).
static Heading dir_heading() {
    return route_dir == HEADING_E ? HEADING_E : HEADING_N;
}
// Yaw (current frame: 0 = launch_h, left positive) that points along `want`.
static float yaw_of_heading(Heading want) {
    int k = (want - launch_h) & 3;          // clockwise quarter-turns
    float y = -90.0f * (float)k;
    if (y < -180.0f) y += 360.0f;           // k=3 -> +90 (one left)
    return y;
}

// Guards
static int lat_over_run = 0;
static int beam_close_run = 0;
// s at the last packet with at least one live beam (blind-flight guard).
static float last_sighted_s = 0;

// s of the last known good post -- the position the map is pinned to. The
// launch cell centre is itself an anchor (s = 0); every LOCK moves it.
static float anchor_s_mm = 0;
// THE SLIP INSTRUMENT (LapCorner v4, LapReport.h): over each anchor-to-
// anchor segment, the commanded speed GAINED and SHED, accumulated every
// packet. As the next gate anchors, the segment's dead-reckoned distance
// ("dr"), these two, and what the slip model (sacc/sbrk) already added are
// written to that corner's lap record -- the three columns sacc/sbrk are
// fitted from. The fold (control_path_slip_take) happens at EVERY anchor,
// recorded or not, so the model's running term stays bounded.
static float seg_dv_acc = 0.0f, seg_dv_brk = 0.0f, seg_v_last = 0.0f;
static void seg_tick(float v_cmd) {
    float dv = v_cmd - seg_v_last;
    if (dv > 0.0f) seg_dv_acc += dv;
    else seg_dv_brk -= dv;
    seg_v_last = v_cmd;
}
static bool slip_armed = false;   // launch straight behind us (Control.h)
static void seg_anchor(int k, float rode, bool record) {
    float slip = control_path_slip_take();
    slip_armed = true;   // the launch straight is behind us
    if (record) lap_report_segment(k, rode, seg_dv_acc, seg_dv_brk, slip);
    seg_dv_acc = seg_dv_brk = 0.0f;
    seg_v_last = control_path_status().v_mm_s;
}

// Corners that LOCKED this leg, queued for the lap-end learning fold into
// the per-corner goff table (GoffTable.h). Queued rather than learned in
// flight because learning writes FRAM and nothing writes FRAM while flying
// -- the same rule that batches the lap report.
static uint8_t learn_n = 0;
static struct {
    uint8_t cx, cy, h;
    float corr;
} learn_q[ROUTE_MAX_TURNS];
// THE V TIMING LEARNER (DiagLane.h; bert 2026-09-30: "MR2 being a safe
// angle intent run to trim further runs"). Each V's exit is flown back to
// the line on dead reckoning; the exit diagonal's first post then says how
// far off the dead reckoning was, and that residual is the V's along-track
// timing (1:1). Queued per V, folded at leg end into the goff table under
// the V key (cell, arrival heading | GOFF_KEY_V) with the same glearn gain
// and slew as the gates, spent at the V's lever from the next run on. The
// search clears the table, so every new maze starts from zero.
static uint8_t vlearn_n = 0;
static struct {
    uint8_t cx, cy, h;
    float out_mm;       // exit residual, + = OUTSIDE (the V was late)
} vlearn_q[ROUTE_MAX_TURNS];
// A hand-placed launch (at_start_setback) has an eyeball datum: bert places
// at the cell centre, the compile assumes the rear-wall setback, and the
// ~42 mm difference lands whole in the FIRST lock's corr -- (0,5) reads
// +24 after a hand start and -18 after a lap turn (runs 101/111 vs
// 102/103). That corr is placement, not corner geometry, so the first lock
// of a setback-launched leg is excluded from learning.
static bool launch_was_setback = false;
// ...but a MACHINE press is not an eyeball. Once the mouse backs onto the
// wall itself (RR_BACKUP) the setback pose is a mechanical datum, and the
// first 10 laps flown that way put corner 0's corr at +10.5 +10.5 +12.0
// +12.0 +13.5 +13.0 +11.5 +12.0 +14.0 +15.5 -- mean +12.45, sd 1.49, which
// is tighter than any gate on the mouse. Excluding that from the learner
// would be throwing away the best measurement in the lap; worse, it left
// (0,5) as the ONE corner that could never learn, and it sat 13 mm outside
// on every lap of the session while the rest converged. So the exclusion
// now keys on a HAND placement specifically, not on the setback pose.
static bool launch_by_press = false;
static bool first_lock_pending = false;

// THE LEARNER MAY ONLY LEARN FROM A CORNER WHOSE ANCHOR WAS FRESH.
//
// corr at corner k is (truth - expectation) accumulated since the LAST
// anchor. When corner k-1 gates, that is one cell of odometry and corr is
// corner k's own geometry -- which is what the table is for. When corner
// k-1 goes NO_EDGE, the anchor is two corners old and corr carries k-1's
// whole uncorrected error as well. Folding that into the table teaches
// cell k a permanent offset for a miss that happened somewhere else.
//
// This is not theoretical. Session 251-273 (2026-08-31), |corr| at a
// corner whose predecessor gated: 11.0 mm mean over 123 samples. |corr|
// at a corner whose predecessor MISSED: 24.2 mm over 10. The five largest
// corrections of the entire session -- 34.0, 35.0, 37.0, 39.5, 40.0 -- all
// follow a miss, and three of them are the same corner:
//
//   seq 265  (1,3) NO_EDGE -> (2,5) corr -39.5
//   seq 270  (1,3) NO_EDGE -> (2,5) corr -34.0
//   seq 271  (1,3) NO_EDGE -> (2,5) corr -37.0
//
// The table did exactly what it was told: goff(2,5) walked +7 -> +46 over
// those laps (10 mm/lap slew cap, three laps of it), which moved the arc
// entry 39 mm and dragged (2,5)'s exit lateral from +24 (inside) to -20
// (outside). The gate converged -- corr at (2,5) is ~0 by seq 273 -- on an
// expectation that is 39 mm wrong. A learner optimising corr to zero will
// happily fly the mouse into the outside wall to get there.
//
// So: a lock learns only if the anchor it rode from was set at the
// previous corner. A miss poisons exactly one corner's lesson, not the
// table. (It costs nothing: the correction is still APPLIED in flight --
// only the permanent fold is skipped.)
static bool anchor_fresh = false;
static uint8_t learn_skipped = 0;

// ---- THE DIAGONAL GATE (dgate; Route.h, the diagonal note) ------------
//
// On a diagonal the posts flick past on ALTERNATE sides every 127.3 mm, each
// seen by that side's beam as a descending ramp (~130 -> ~50 over ~100 mm
// of travel) and then a sharp drop-out. The drop-out is the edge. The first
// flight (laps 291-296, 2026-09-01) put six of them within 2-9 mm of the
// geometry's prediction, mean +6.2, sd 2.6 -- a better landmark than any
// corner post on CAL-6, and with the rest of the diagonal to drain into
// rather than 5 mm.
//
// The closest reading is a LATERAL. The post's near corner is
// ROUTE_DIAG_POST_OFF_MM less a half post diagonal -- 55.1 mm -- from the
// centre line. What each beam READS for a centred post was measured on the
// bench 2026-09-06 (Bench 'wcal lp' / 'wcal rp', FLINCH.md): the LEFT beam
// 57.0, the RIGHT beam 54.7, both to a count over 2 s -- so a left post read
// at m says the mouse is 57.0 - m to the left; a right post says m - 54.7.
// (Before that both used 57.6, the geometric face; the 2.3 mm L-R skew was
// in the estimate.) The response is not 1:1 either: 20 mm toward a post
// read 40 on both sides (the beam then catches the post's lower 45-deg
// face), a slope of ~0.85; DGATE_LAT_MAX_MM bounds what is acted on.
// First flight: L 60.3, R 53.0 -> 3.7 mm right.
//
// Neither 45 is gated. The 45-in's edge drops as its arc begins (a 45's
// tangent is 49 mm, a 90's 98.6, so the arc entry sits 3 mm after the drop)
// and the beam swings onto the post it is turning around; the 45-out is
// met at 45 degrees to every wall. The diagonal's own posts place both.
//
// Post n: foot at cp + OFF + n*PITCH along the line, side alternating from
// the INSIDE of the entry turn (n = 0). Post 0 drops inside the 45-in's
// arc and is skipped; the last post considered is the last whose drop lands
// before the 45-out's arc with room to drain.
static constexpr int DGATE_MAX_POSTS = 12;
static constexpr float DGATE_SEE_MM = 110.0f;      // a post reads under this
static constexpr float DGATE_MIN_LEN_MM = 30.0f;   // ...for at least this
// A centred mouse reads a post at ~57.6; displaced 15 mm away from it, ~73.
// 80 rejected exactly the posts an off-centre lap most needs (lap 323
// entered 16 mm right: post 1 missed, post 3 scraped through at min 70).
static constexpr float DGATE_MIN_READ_MM = 90.0f;  // ...getting this close
static constexpr float DGATE_WINDOW_MM = 30.0f;    // drop-out plausibility
// 45-IN ENTRY-LATERAL COMPENSATION: mm of extra "later" per mm the mouse is
// on the INSIDE of the coming 45 as it resolves (measured in the feeding
// corridor). 1.0 = fire later by the whole offset; capped so a bad lateral
// cannot throw the 45 into the far wall.
static constexpr float DIAG_IN_LAT_GAIN = 1.0f;
static constexpr float DIAG_IN_LAT_MAX_MM = 30.0f;
// 0 since 2026-09-06 afternoon: for a V the inside post sits at the arc's
// CENTRE (Route.h ROUTE_V_RADIUS_MAX_MM), and moving the arc along the
// entry diagonal moves the centre off the post in either direction --
// clearance can only come from the entry lateral itself. The hand trim
// on the V's cell stays as a lever for the along-track, not the post.
static constexpr float V_IN_LAT_GAIN = 0.0f;
static constexpr float DGATE_ROOM_MM = 80.0f;      // drain before the next ramp
static constexpr float DGATE_HALF_POST_MM = 6.0f;
// Per-beam post constants, MEASURED (wcal, 2026-09-06), not the geometric
// ROUTE_DIAG_POST_OFF_MM - DGATE_HALF_POST_MM = 57.6.
static constexpr float DGATE_FACE_L_MM = 57.0f;    // LEFT beam on a left post, centred
static constexpr float DGATE_FACE_R_MM = 54.7f;    // RIGHT beam on a right post, centred
static constexpr float DGATE_LAT_MAX_MM = 20.0f;   // do not steer on more
// THE DIAGONAL LATERAL SERVO IS A LANE CHANGE, NOT A CORRIDOR SERVO
// (2026-09-06). It gets ONE sample per post, 127.3 mm apart, and the next
// hazard is the next post -- so the response to a sample must move most of
// the error before the next post, then be refreshed by that post's own
// sample. A heading offset theta held over one pitch moves the mouse
// 127 * theta(rad); to take out ~80 % of an error e in one pitch needs
// theta = 0.36 deg per mm of e -- 2.4x the corridor servo's gain, capped
// by the bias clamp (5 deg -> ~11 mm per post) for anything over 14 mm.
// The old value here was 0.5 (0.075 deg/mm): from a 15 mm error it moved
// ~7 mm per post and both D8 laps of 2026-09-06 hit the V's inside post
// after riding the first diagonal 13.6 mm inside for three posts, the
// third post read at 41 -- the bodywork -- before the arc even began.
// The 2026-09-04 overshoot that argued for 0.5 (11 right -> 10 left over
// four posts) was a LAG problem, not a gain problem: the estimate was the
// average of the last LEFT and last RIGHT samples, a post and a half old.
// The estimate is now the latest sample alone; each post is an
// independent measurement of the same centre line.
// Since 2026-09-07 an ABSOLUTE gain, not a multiple of the corridor's --
// the corridor gain became tunable (`latk`) and the diagonal must not
// follow it: this servo is sample-and-hold per post (127 mm), so its
// per-post gain is 127 x k(rad) = 0.8 at 0.36 deg/mm, and anything past
// ~0.45 overshoots between posts by construction.
static constexpr float DGATE_SERVO_DEG_PER_MM = 0.36f;   // was 2.4 x 0.15
// The lane change a sample commands is designed to be done in one pitch;
// the bias ends there unless a post refreshes it (see the servo).
static constexpr float DGATE_HOLD_MM = ROUTE_DIAG_PITCH_MM;
// BEAM-LEAD COUPLING, the side gate's gklat in diagonal form. The side
// beam is angled ~45 deg forward (58 mm lead onto a face 57.6 away), so a
// post that sits FARTHER across the line is lost LATER along-track and a
// nearer one earlier -- lateral error masquerades as along-track error.
// Lap 323 (2026-09-02) entered the diagonal 16 mm right and the raw
// corrections swung -22.6 / +28.2 on alternate sides while the exit was
// fine. Fitted from all six DGATE laps (310-312, 323-325, n=31):
//     L: corr = +1.5 - 0.68*lat      residual sd 2.3
//     R: corr = +2.7 + 0.39*lat      residual sd 3.7
// The applied correction subtracts the slope term, using the SAME post's
// own lateral (one measurement, two projections). Slopes are compiled,
// like the beam geometry they express; refit from the DGATE table if the
// sensor pose ever changes. The intercepts live in dtrim.
static constexpr float DGATE_KLAT_L = -0.68f;
static constexpr float DGATE_KLAT_R = +0.39f;
static constexpr float DGATE_EXIT_MARGIN_MM = 10.0f;
struct DgPost {
    int8_t n;
    int8_t diag;         // which diagonal (its exit turn index): the table spans the lap
    uint8_t side_right;
    uint8_t seen;        // 1 = a qualifying drop-out landed in the window
    uint8_t applied;
    uint8_t min_mm;
    int16_t pred_mm;     // from the entry corner point
    int16_t obs_mm;
    float corr_mm;
    float lat_mm;        // single-post lateral estimate (+ left)
};
static DgPost dg_post[DGATE_MAX_POSTS];
static int dg_np = 0;             // posts recorded this diagonal
static int dg_exit_k = -1;        // the diagonal being flown (its exit turn)
static int dg_n = 0;              // the post being watched
static int dg_last_n = 0;         // last post worth watching
static bool dg_first_right = false;
static float dg_cp_s = 0.0f;      // path-s of the entry corner point
static float dg_exit_arc_s = 0.0f;
static bool dg_ep_open = false;
static float dg_ep_s0 = 0.0f;
static uint8_t dg_ep_min = 255;
static float dg_lat_l = 0.0f, dg_lat_r = 0.0f;
static bool dg_lat_l_ok = false, dg_lat_r_ok = false;
static float dg_lat_est = 0.0f;
static bool dg_lat_valid = false;
static float dg_lat_s = 0.0f;   // path s the latest post sample was taken at
static float dg_last_corr = 0.0f;
// The dead-reckoned lateral (DiagLane.h): seeded at a V's arc end with the
// V's known outward offset, re-seeded by every accepted post sample.
static DiagDR dg_dr;
static int dg_v_learn_k = -1;     // the V whose first exit post is pending
// The whole lateral set point of the diagonal being flown (DiagLane.h VLane):
// the V behind's exit walk-back, the V ahead's entry push, dlat.
static VLane dg_lane;
// A V's timing shift (hand trim + learned) is a path-s LEAD given before its
// arc; the arc's exit does not move along the exit line (it moves across
// it), so the lead is handed back at the arc end. Before 2026-09-30 it rode
// on into every corner after the V: MR5's (4,4) turned ~40 mm early after
// two learned Vs, came out 42 mm inside and the lateral guard stopped the
// run in the goal row.
static int v_shift_k = -1;
static float v_shift_mm = 0.0f;
static int dg_locked = 0, dg_missed = 0;
// WHERE THE 45-IN REALLY JOINED THE DIAGONAL (2026-09-12, D9 first flight).
// The lattice is laid out from the compiled corner point, but the mouse
// joins the diagonal from wherever it was in the feeding corridor: a
// lateral offset d (+ = inside of the turn) lands it d/sqrt2 further along
// the diagonal, and the 45-in's lever (the arc fired `shift` later in the
// world) adds shift/sqrt2 more. D9 CCW: 22 mm inside after the (4,0) 90,
// lever +20 -> the mouse rode the dogleg diagonal 30 mm ahead of its
// lattice and post 1 read 41-44 early on all three laps; CW the same
// corner was entered 8 mm OUTSIDE with no lever and post 1 read 1-11 late.
// D10's 45-ins (levers 2-18) fit the same line. The DGATE correction was
// re-anchoring the frame to the physical diagonal correctly -- the "45's
// own ~30 mm early bias" was mostly this -- but a MISSED post 1 left the
// whole error in place, and the corr numbers said "odometry" when they
// meant "geometry". Recorded at the lever, consumed by dg_begin.
static int dg_join_corner = -1;
static float dg_join_ahead_mm = 0.0f;
static float diag_gate_live = 0.0f;   // dgate
static float diag_trim_live = 0.0f;   // dtrim
static float diag_lat_live = 0.0f;
static float v_out_live = 0.0f;     // vout: the V's outward offset, mm
static float v_learn_live = 0.0f;   // vlearn: the V timing learner on/off
static float latk_live = LAT_SERVO_DEG_PER_MM;   // latk: corridor servo gain, deg/mm
static float flinch_live = 0.0f;      // flinch (mm), 0 = off
static float ftau_live = 0.0f;        // ftau (ms), 0 = off
static float walign_live = 0.0f;      // walign (gain), 0 = off

// ---- WALL-FRAME ALIGNMENT (Control.h ControlGains::wall_align) ----------
// On a straight with BOTH walls, the corridor's direction in the path frame
// is observable: the lateral drifts at sin(heading error) per mm of travel,
// and the heading (relative to the path's cardinal, in the frame) is
// measured. Fit lat vs s over FRAME_FIT_MM, take the mean heading, and
//     frame error = mean(heading) - asin(d lat / d s)
// is where the corridor really points. An OFFSET does not drift, so it
// cannot be mistaken for an angle -- which is what a plain integral on the
// lateral would do. The estimate is applied through
// control_path_frame_adjust, a fraction (walign) per window, rate- and
// lap-limited, and the window restarts. Heading in the frame is recovered
// from the executor's yaw error and the biases this file added to the
// reference: yaw - path_yaw = -(yaw_err) + lat_bias + reflex_bias.
static float fr_s0 = 0.0f;
static int fr_n = 0;
static double fr_sx = 0, fr_sy = 0, fr_sxx = 0, fr_sxy = 0, fr_sh = 0;
static bool fr_open = false;
static float fr_applied = 0.0f;    // this lap, deg
static int fr_windows = 0;
static float fr_last_est = 0.0f;
static float lat_bias_last = 0.0f;
static void fr_window_reset() {
    fr_open = false;
    fr_n = 0;
    fr_sx = fr_sy = fr_sxx = fr_sxy = fr_sh = 0;
}
static void fr_lap_reset() {
    fr_window_reset();
    fr_applied = 0.0f;
    fr_windows = 0;
    fr_last_est = 0.0f;
}
// Called on every packet where the corridor is valid (both walls, model
// on, not in an arc); `off` closes the window on every other packet.
static void fr_sample(float s, float lat, float hdg_deg) {
    if (!fr_open) { fr_open = true; fr_s0 = s; }
    double x = s - fr_s0;
    fr_n++;
    fr_sx += x; fr_sy += lat; fr_sxx += x * x; fr_sxy += x * lat;
    fr_sh += hdg_deg;
    if (x < FRAME_FIT_MM || fr_n < 20) return;
    double den = fr_n * fr_sxx - fr_sx * fr_sx;
    if (den <= 0) { fr_window_reset(); return; }
    double slope = (fr_n * fr_sxy - fr_sx * fr_sy) / den;   // mm per mm
    if (slope > 0.2) slope = 0.2;
    if (slope < -0.2) slope = -0.2;
    float drift_deg = (float)(asin(slope) * 57.29577951308232);
    float hdg_mean = (float)(fr_sh / fr_n);
    float est = hdg_mean - drift_deg;   // corridor direction in the frame
    fr_last_est = est;
    float step = walign_live * est;
    // The lap's FIRST window carries the launch error, which is the big
    // one, and CCW on D7 the launch straight is ~250 mm before the 45-in
    // -- one window. Let that one apply twice the per-window cap.
    float cap = fr_windows == 0 ? 2.0f * FRAME_STEP_MAX_DEG : FRAME_STEP_MAX_DEG;
    if (step > cap) step = cap;
    if (step < -cap) step = -cap;
    float total = fr_applied + step;
    if (total > FRAME_LAP_MAX_DEG) step = FRAME_LAP_MAX_DEG - fr_applied;
    if (total < -FRAME_LAP_MAX_DEG) step = -FRAME_LAP_MAX_DEG - fr_applied;
    if (step != 0.0f) {
        control_path_frame_adjust(step);
        fr_applied += step;
        fr_windows++;
    }
    fr_window_reset();
}
static void fr_report() {
    if (walign_live <= 0.0f) return;
    hal_radio_wait_room(96, 20);
    printf_line("FRAME,applied=%+.1f,windows=%d,last_est=%+.1f",
                (double)fr_applied, fr_windows, (double)fr_last_est);
}

// ---- THE VECTOR REFLEX: beam rate ---------------------------------------
// "Vector data" (bert, 2026-09-05): each side beam's closing rate, measured
// as mm of range lost per mm of TRAVEL over the last REFLEX_RATE_SPAN_MM,
// from a short ring of (s, left, right) samples. Per mm of travel rather
// than per second so the geometry is speed-invariant; multiplied by v where
// a time is wanted (tau). A rate is valid only when both ends of the span
// are live readings (1..254): an edge appearing or vanishing is not a rate.
static constexpr float REFLEX_RATE_SPAN_MM = 4.0f;
// A beam RECEDING faster than this (mm per mm) is a wall being left behind,
// and the straight/diagonal range rule does not fire on it.
static constexpr float REFLEX_RECEDE_MM_PER_MM = 0.2f;
static constexpr int REFLEX_RING_N = 16;
struct BeamHist { float s; uint8_t l, r; };
static BeamHist rfx_ring[REFLEX_RING_N];
static int rfx_head = 0, rfx_n = 0;
// Per-lap instrument, printed as a REFLEX line after the lap block.
static int rfx_fires = 0;            // packets with a non-zero bias
static float rfx_max_deg = 0.0f;     // peak |bias|
static float rfx_first_s = 0.0f;     // s of the first fire this lap
static char rfx_first_ctx = '-';     // 's'traight / 'a'rc / 'd'iagonal
static float rfx_bias_prev = 0.0f;

static void rfx_reset() {
    rfx_head = rfx_n = 0;
    rfx_fires = 0;
    rfx_max_deg = 0.0f;
    rfx_first_s = 0.0f;
    rfx_first_ctx = '-';
    rfx_bias_prev = 0.0f;
}

static inline bool beam_live(uint8_t b) { return b > 0 && b < 255; }

// Push this packet's beams, then return the closing rate (mm per mm of
// travel, + = closing) of the given side against the sample >=
// REFLEX_RATE_SPAN_MM back. false when no such sample or either end dead.
static void rfx_push(float s, uint8_t l, uint8_t r) {
    rfx_ring[rfx_head] = BeamHist{s, l, r};
    rfx_head = (rfx_head + 1) % REFLEX_RING_N;
    if (rfx_n < REFLEX_RING_N) rfx_n++;
}
static bool rfx_closing(bool right, float s, uint8_t cur, float *per_mm) {
    if (!beam_live(cur)) return false;
    // Walk back from the newest; the first sample at least the span behind
    // is the one (the ring holds ~16 packets, ~30 mm at 700 mm/s).
    for (int i = 1; i <= rfx_n; i++) {
        const BeamHist &h = rfx_ring[(rfx_head - i + REFLEX_RING_N) % REFLEX_RING_N];
        float ds = s - h.s;
        if (ds < REFLEX_RATE_SPAN_MM) continue;
        uint8_t old = right ? h.r : h.l;
        if (!beam_live(old)) return false;
        *per_mm = ((float)old - (float)cur) / ds;
        return true;
    }
    return false;
}

static void rfx_report() {
    if (flinch_live <= 0.0f && ftau_live <= 0.0f) return;
    hal_radio_wait_room(96, 20);
    if (rfx_fires == 0)
        printf_line("REFLEX,n=0");
    else
        printf_line("REFLEX,n=%d,max=%.1f,first=%c@%d", rfx_fires,
                    (double)rfx_max_deg, rfx_first_ctx, (int)rfx_first_s);
}

// -- THE LANE-CHANGE STEP TEST ('lstep <mm>', bert 2026-09-07: "the dgate
// servo 5 deg / 100 mm seems very low compared to what the mouse is
// capable of. we should test those limits") ------------------------------
// One shot per lap: on the first two-wall straight that has been valid for
// LS_LEAD_MM and still has LS_NEED_MM to its next arc, the corridor
// servo's set point steps to +mm (left) for LS_HOLD_MM of travel, then back
// to 0. The lateral (the walls' own estimate) is sampled against travel
// from the step and the lap report gets LSTEP,mm=,v=,rise90=,peak=,over=,
// back90= plus an LS,ds,lat,yerr table -- the plant's lane-change response
// measured directly, which is what sets latk, DGATE_SERVO_DEG_PER_MM, the clamp and
// the feedforward. Wall-frame alignment is suspended for the window (a
// commanded lateral ramp looks exactly like a heading error to it).
static constexpr float LS_LEAD_MM = 100.0f;
static constexpr float LS_NEED_MM = 450.0f;
static constexpr float LS_HOLD_MM = 250.0f;
static constexpr float LS_LOG_MM = 500.0f;
static constexpr int LS_MAX = 64;
static int corner_ix;                // the gate's next corner (defined with the gate below)
static float ls_mm = 0.0f;           // the commanded step; 0 = test off
static bool ls_done = false, ls_on = false;
static float ls_s0 = 0.0f;           // path-s the step began at
static float ls_stretch_s0 = -1.0f;  // where the current two-wall stretch began
static float ls_v = 0.0f;
static int16_t ls_ds[LS_MAX], ls_lat[LS_MAX], ls_ye[LS_MAX];
static int ls_n = 0;
static float ls_next_log = 0.0f;
static void ls_reset() {
    ls_done = ls_on = false; ls_n = 0; ls_stretch_s0 = -1.0f; ls_v = 0.0f;
}
// The set point for the corridor servo at path-s; also records samples.
static float ls_set_point(float s_now, float lat, float yaw_err, float v, bool two_wall_valid) {
    if (ls_mm == 0.0f) return 0.0f;
    if (!two_wall_valid) { ls_stretch_s0 = -1.0f; return ls_on ? ls_mm : 0.0f; }
    if (ls_stretch_s0 < 0.0f) ls_stretch_s0 = s_now;
    if (!ls_done && !ls_on && s_now - ls_stretch_s0 >= LS_LEAD_MM &&
        corner_ix < n_turns &&
        path.corner_entry_s_mm(corner_ix) - s_now >= LS_NEED_MM) {
        ls_on = true; ls_done = true; ls_s0 = s_now; ls_v = v; ls_next_log = 0.0f; ls_n = 0;
        printf_line("LSTEP: %+.0f mm at s%d, v %.0f, hold %.0f mm", (double)ls_mm,
                    (int)s_now, (double)v, (double)LS_HOLD_MM);
    }
    if (!ls_done) return 0.0f;
    float ds = s_now - ls_s0;
    if (ds <= LS_LOG_MM && ds >= ls_next_log && ls_n < LS_MAX) {
        ls_ds[ls_n] = (int16_t)lroundf(ds);
        ls_lat[ls_n] = (int16_t)lroundf(lat);
        ls_ye[ls_n] = (int16_t)lroundf(yaw_err * 10.0f);
        ls_n++;
        ls_next_log = ds + LS_LOG_MM / (float)LS_MAX;
    }
    if (ls_on && ds >= LS_HOLD_MM) ls_on = false;
    return ls_on ? ls_mm : 0.0f;
}
static void ls_report() {
    if (ls_mm == 0.0f || !ls_done || ls_n < 4) return;
    // rise90: travel until the lateral first reaches 90 % of the step;
    // peak: the extreme lateral while held; back90: travel after release
    // until within 10 % of the step of zero.
    float lat0 = (float)ls_lat[0];
    float target = lat0 + ls_mm, thr = lat0 + 0.9f * ls_mm;
    int rise = -1, back = -1; float peak = lat0;
    for (int i = 0; i < ls_n; i++) {
        float l = (float)ls_lat[i];
        bool up = ls_mm > 0.0f;
        if (rise < 0 && (up ? l >= thr : l <= thr)) rise = ls_ds[i];
        if (ls_ds[i] <= LS_HOLD_MM && (up ? l > peak : l < peak)) peak = l;
        if (ls_ds[i] > LS_HOLD_MM && back < 0 && fabsf(l - lat0) <= 0.1f * fabsf(ls_mm))
            back = ls_ds[i] - (int)LS_HOLD_MM;
    }
    hal_radio_wait_room(128, 20);
    printf_line("LSTEP,mm=%.0f,v=%.0f,rise90=%d,peak=%.0f,over=%.0f%%,back90=%d,n=%d",
                (double)ls_mm, (double)ls_v, rise, (double)(peak - lat0),
                (double)(100.0f * ((peak - lat0) - ls_mm) / ls_mm), back, ls_n);
    (void)target;
    for (int i = 0; i < ls_n; i++) {
        hal_radio_wait_room(48, 10);
        printf_line("LS,%d,%d,%.1f", ls_ds[i], ls_lat[i], (double)ls_ye[i] * 0.1);
    }
}
void route_run_set_lstep(float mm) { ls_mm = mm; ls_reset(); }

static float dg_pred_drop(int n) {
    return dg_cp_s + ROUTE_DIAG_POST_OFF_MM + (float)n * ROUTE_DIAG_PITCH_MM -
           WALL_GATE_BEAM_AHEAD_MM + DGATE_HALF_POST_MM + diag_trim_live;
}
static bool dg_side_right(int n) { return ((n & 1) == 0) == dg_first_right; }

static bool is_v_turn(int k);   // below, with the corner types

static void dg_begin(int exit_k) {
    dg_exit_k = exit_k;
    // dg_np is NOT reset here (2026-09-10): the post table spans the lap, so
    // a lap with two diagonals (D8: 509 mm A, then the V, then 255 mm B)
    // reports both -- until now only the last one survived to the lap end,
    // and diagonal A's posts, the ones that say how the 45-in delivered the
    // mouse, were never in a laps file.
    // THE ANCHOR TRAP, ON THE DIAGONAL (Path.h header; 2026-09-11, D10 CCW).
    // The post lattice is laid out from the entry corner point ALONG THE
    // DIAGONAL, so the corner point's path-s must be the one on the exit
    // side of the arc: exit_s - T. It used to be entry_s + T, which is the
    // same point on the APPROACH line but 2T - arclength further along the
    // path: 5 mm for a 45 (harmless), ~31 for a V (diagonal B's post 1 on
    // D8/D9 reading 15-35 early), 210 for a 135-in -- on D10 CCW the whole
    // lattice sat a post pitch and more off after each 135-in: post 1 "seen"
    // at +65 against +155, +82 applied, the 45-out fired into the wall
    // (lap 157). The 45's own ~30 mm early was NOT this: see
    // dg_join_ahead_mm (where the 45-in really joined the line).
    dg_cp_s = path.corner_exit_s_mm(exit_k - 1) -
              path.corner_tangent_mm(exit_k - 1);
    // ...moved back by however far ahead the 45-in actually joined the
    // line (dg_join_ahead_mm, recorded at its lever): the posts are where
    // they are, the mouse reaches them sooner.
    if (dg_join_corner == exit_k - 1) dg_cp_s -= dg_join_ahead_mm;
    dg_join_corner = -1;
    dg_exit_arc_s = path.corner_entry_s_mm(exit_k);
    // Post 0 sits on the inside of the entry turn for a 45 or a V. A 135's
    // corner point is one post pitch further back along the line (Route.h,
    // THE 135), so its post 0 is on the OTHER side.
    bool e135 = route_turn_is_135(turns[exit_k - 1]);
    dg_first_right = (turns[exit_k - 1].sweep_deg < 0) != e135;
    // Post 0 always drops inside (or hard against the end of) the entry
    // arc -- the beam swings onto the very post the 45 turns around -- so
    // start at 1, and skip on past any post the arc still covers: a 135's
    // arc is 220 mm long against the lattice's 127, so it starts at 2.
    dg_n = 1;
    {
        float arc_end = path.corner_exit_s_mm(exit_k - 1);
        while (dg_n + 1 < DGATE_MAX_POSTS && dg_pred_drop(dg_n) < arc_end) dg_n++;
    }
    dg_last_n = 0;
    for (int n = 1; n < DGATE_MAX_POSTS; n++)
        if (dg_pred_drop(n) < dg_exit_arc_s - DGATE_EXIT_MARGIN_MM) dg_last_n = n;
    dg_ep_open = false;
    dg_lat_l_ok = dg_lat_r_ok = dg_lat_valid = false;
    dg_last_corr = 0.0f;
    // THE V EXIT (DiagLane.h). A V's arc lands the mouse vout OUTSIDE this
    // diagonal's centre line whatever the entry did -- a known start, so the
    // walk back is flown on dead reckoning from here, not on a post sample
    // that does not come until the post itself.
    dg_v_learn_k = -1;
    float vo = v_out_live > 0.0f ? v_out_live : 0.0f;
    // THE ENTRY THAT FELL SHORT (DiagLane.h, 2026-10-01). The arc just
    // flown is a V compiled tangent to lines vout outside; the dead
    // reckoning (stopped at the arc's start -- it is not stepped in arcs)
    // says how far outside the mouse really was. The shortfall puts it
    // AHEAD of path-s by the same amount: hand it to path-s now, before
    // the next corner fires on it.
    // ...but only when the dead reckoning was SEEDED BY A POST. In a chain
    // of Vs (the small maze, 2026-10-04) the DR entering V2 was seeded by
    // V1's assumed exit and never checked against a post: it read "entered
    // -7.9 / 17 / 22 mm outside" and the fix sat on its +/-12 clamp at
    // every V of every run; the one MR4 abort of the morning was V (5,2)
    // taking a +12 along fix and leaving 12.7 mm off (exitye -12.7), then
    // V (4,3) at maxye 25 -> heading-error abort. A 12 mm along error at a
    // 90 degree V IS a 12 mm lateral error at its exit. A single V after a
    // 45-in has post-seeded DR and keeps the fix (Hazlemere: "entered 5.4
    // mm outside of 10 -> +4.6", flown clean four times).
    if (is_v_turn(exit_k - 1) && dg_dr.valid && !dg_dr.from_v &&
        path.corner_entry_s_mm(exit_k - 1) - dg_dr.s0 <= V_DR_MAX_AGE_MM) {
        float out = turns[exit_k - 1].sweep_deg > 0 ? -1.0f : 1.0f;   // + left
        float e_out = out * (dg_dr.lat - dg_lane.base);
        float ahead = v_exit_along_mm(vo, e_out);
        if (ahead > V_ALONG_FIX_MAX_MM) ahead = V_ALONG_FIX_MAX_MM;
        if (ahead < -V_ALONG_FIX_MAX_MM) ahead = -V_ALONG_FIX_MAX_MM;
        if (fabsf(ahead) >= 1.0f) {
            control_path_correct(ahead, 30.0f);
            printf_line("  V (%u,%u): entered %.1f mm outside of %.0f -> %+.1f mm "
                        "along at its exit",
                        turns[exit_k - 1].cx, turns[exit_k - 1].cy, (double)e_out,
                        (double)vo, (double)ahead);
        }
    }
    dg_lane = VLane{};
    dg_lane.base = diag_lat_live;
    dg_lane.next_arc_s = dg_exit_arc_s;
    if (is_v_turn(exit_k - 1) && vo > 0.0f) {
        dg_lane.exit_v = true;
        dg_lane.exit_s0 = path.corner_exit_s_mm(exit_k - 1);
        dg_lane.exit_off = (turns[exit_k - 1].sweep_deg > 0 ? -1.0f : 1.0f) * vo;  // + left
        dg_lane.fit_exit();
    }
    if (is_v_turn(exit_k) && vo > 0.0f) {
        dg_lane.entry_v = true;
        dg_lane.entry_arc_s = dg_exit_arc_s;
        dg_lane.entry_off = (turns[exit_k].sweep_deg > 0 ? -1.0f : 1.0f) * vo;
    }
    if (is_v_turn(exit_k - 1)) {
        float arc_end = path.corner_exit_s_mm(exit_k - 1);
        dg_dr.seed(dg_lane.sp(arc_end), arc_end, true);
        dg_v_learn_k = exit_k - 1;
        // Hand back the V's timing lead (see v_shift_k).
        if (v_shift_k == exit_k - 1 && v_shift_mm != 0.0f) {
            control_path_correct(v_shift_mm, 30.0f);
            v_shift_mm = 0.0f;
        }
        v_shift_k = -1;
    } else {
        dg_dr.clear();
    }
}

// One packet on the diagonal. Returns true if a correction was APPLIED.
static bool dg_update(const WallRanges &w, float s_now) {
    if (dg_n > dg_last_n) return false;
    bool right = dg_side_right(dg_n);
    uint8_t v = right ? w.right_mm : w.left_mm;
    float pred = dg_pred_drop(dg_n);
    bool seeing = v != 0 && v != 255 && (float)v < DGATE_SEE_MM;
    bool applied = false;
    if (seeing) {
        if (!dg_ep_open) {
            dg_ep_open = true;
            dg_ep_s0 = s_now;
            dg_ep_min = v;
        } else if (v < dg_ep_min) {
            dg_ep_min = v;
        }
        // An episode that outlives the window is not this post.
        if (s_now > pred + DGATE_WINDOW_MM + 40.0f) dg_ep_open = false;
        return false;
    }
    if (dg_ep_open) {
        dg_ep_open = false;
        float len = s_now - dg_ep_s0;
        bool post = len >= DGATE_MIN_LEN_MM && (float)dg_ep_min < DGATE_MIN_READ_MM;
        float lat_pre = right ? ((float)dg_ep_min - DGATE_FACE_R_MM)
                              : (DGATE_FACE_L_MM - (float)dg_ep_min);
        // Remove the beam-lead coupling BEFORE the window test: a
        // displaced mouse moves the drop-out genuinely, and judging the
        // raw edge against the window rejected exactly those posts.
        float corr_c = (pred - s_now) -
                       (right ? DGATE_KLAT_R : DGATE_KLAT_L) * lat_pre;
        // The window grows with the distance this expectation has ridden
        // since the last anchor, exactly as the side gate's does: CCW the
        // first post is the lap's first anchor after ~500 mm of launch
        // and 45-in, and +38 there (2026-09-05 laps 44-46: a +20 post-face
        // baseline, the +8 trim, the model) fell outside the flat +/-30 --
        // nothing re-anchored and all six posts went "none".
        float win = DGATE_WINDOW_MM +
                    RR_GATE_WINDOW_GROW * (pred - anchor_s_mm);
        if (win > RR_GATE_WINDOW_MAX_MM) win = RR_GATE_WINDOW_MAX_MM;
        if (win < DGATE_WINDOW_MM) win = DGATE_WINDOW_MM;
        if (post && fabsf(corr_c) < win) {
            float corr = corr_c;
            float lat = lat_pre;
            if (right) { dg_lat_r = lat; dg_lat_r_ok = true; }
            else       { dg_lat_l = lat; dg_lat_l_ok = true; }
            // The LATEST sample (see DGATE_SERVO_DEG_PER_MM). Beyond
            // DGATE_LAT_MAX_MM it is CLAMPED, not discarded (2026-09-27, the
            // home leg that clipped the 135-out's post: a right post read 82,
            // 27 mm left of the line, and the old rule stopped steering at
            // exactly the sample that most needed it -- the servo went to 0
            // with the mouse 24 mm inside and 170 mm from the arc). The bias
            // clamp already bounds the response; a clean post read far off
            // is the mouse far off.
            dg_lat_est = lat;
            if (dg_lat_est > DGATE_LAT_MAX_MM) dg_lat_est = DGATE_LAT_MAX_MM;
            if (dg_lat_est < -DGATE_LAT_MAX_MM) dg_lat_est = -DGATE_LAT_MAX_MM;
            dg_lat_valid = true;
            dg_lat_s = s_now;
            // The V's first exit post: what the dead reckoning did not know
            // is the V's own timing (DiagLane.h). + outside = late.
            if (v_learn_live >= 0.5f && dg_v_learn_k >= 0 && dg_dr.valid && dg_dr.from_v &&
                s_now - dg_dr.s0 <= V_LEARN_MAX_S_MM) {
                float resid = dg_lat_est - dg_dr.lat;                 // + left
                float out = turns[dg_v_learn_k].sweep_deg > 0 ? -resid : resid;
                if (vlearn_n < ROUTE_MAX_TURNS) {
                    vlearn_q[vlearn_n].cx = turns[dg_v_learn_k].cx;
                    vlearn_q[vlearn_n].cy = turns[dg_v_learn_k].cy;
                    vlearn_q[vlearn_n].h = turns[dg_v_learn_k].entry_h;
                    vlearn_q[vlearn_n].out_mm = out;
                    vlearn_n++;
                }
                printf_line("  V exit (%u,%u): post %+.1f, dead reckoning %+.1f "
                            "-> %+.1f mm %s, %.0f mm past the arc",
                            turns[dg_v_learn_k].cx, turns[dg_v_learn_k].cy,
                            (double)dg_lat_est, (double)dg_dr.lat,
                            (double)fabsf(out), out >= 0.0f ? "OUTSIDE (late)" : "inside (early)",
                            (double)(s_now - dg_dr.s0));
            }
            dg_v_learn_k = -1;
            dg_dr.seed(dg_lat_est, s_now, false);
            if (diag_gate_live >= 1.0f) {
                float room = dg_exit_arc_s - s_now - 5.0f;
                if (room > DGATE_ROOM_MM) room = DGATE_ROOM_MM;
                if (room < 5.0f) room = 5.0f;
                control_path_correct(corr, room);
                anchor_s_mm = pred;
                seg_anchor(-1, 0.0f, false);   // fold the slip term; posts don't record
                anchor_fresh = true;
                applied = true;
            }
            dg_last_corr = corr;
            dg_locked++;
            led_flash(2);   // a post, seen
            if (dg_np < DGATE_MAX_POSTS) {
                DgPost &P = dg_post[dg_np++];
                P.diag = (int8_t)dg_exit_k;
                P.n = (int8_t)dg_n; P.side_right = right; P.seen = 1;
                P.applied = applied; P.min_mm = dg_ep_min;
                P.pred_mm = (int16_t)lroundf(pred - dg_cp_s);
                P.obs_mm = (int16_t)lroundf(s_now - dg_cp_s);
                P.corr_mm = corr; P.lat_mm = lat;
            }
            dg_n++;
        }
        return applied;
    }
    // Past the window with nothing qualifying: this post is missing (or
    // misaligned beyond the window). Say so and watch the next one.
    if (s_now > pred + DGATE_WINDOW_MM) {
        dg_missed++;
        if (dg_np < DGATE_MAX_POSTS) {
            DgPost &P = dg_post[dg_np++];
            P.diag = (int8_t)dg_exit_k;
            P.n = (int8_t)dg_n; P.side_right = right; P.seen = 0; P.applied = 0;
            P.min_mm = 255; P.pred_mm = (int16_t)lroundf(pred - dg_cp_s);
            P.obs_mm = 0; P.corr_mm = 0.0f; P.lat_mm = 0.0f;
        }
        dg_n++;
    }
    return false;
}

static void dg_report() {
    if (dg_exit_k < 0) return;
    unsigned D = turns[dg_exit_k].diag_mm;
    printf_line("route: dgate %s -- diagonal %u mm, posts 1..%d: %d locked, "
                "%d missed, last corr %+.1f, lateral %+.1f mm%s  "
                "(pred = foot - %.0f lead + %.0f + dtrim %.1f)",
                diag_gate_live >= 2.0f   ? "S+LATERAL"
                : diag_gate_live >= 1.0f ? "ALONG-TRACK"
                                         : "OBSERVE",
                D, dg_last_n, dg_locked, dg_missed, (double)dg_last_corr,
                (double)dg_lat_est, dg_lat_valid ? "" : " (not steered)",
                (double)WALL_GATE_BEAM_AHEAD_MM, (double)DGATE_HALF_POST_MM,
                (double)diag_trim_live);
    for (int i = 0; i < dg_np; i++) {
        const DgPost &P = dg_post[i];
        hal_radio_wait_room(96, 20);
        if (P.seen)
            printf_line("DGATE,diag=%d,post=%d,side=%s,pred=%+d,obs=%+d,corr=%+.1f,"
                        "min=%u,lat=%+.1f,applied=%u",
                        P.diag, P.n, P.side_right ? "R" : "L", P.pred_mm, P.obs_mm,
                        (double)P.corr_mm, (unsigned)P.min_mm,
                        (double)P.lat_mm, (unsigned)P.applied);
        else
            printf_line("DGATE,diag=%d,post=%d,side=%s,pred=%+d,obs=none", P.diag, P.n,
                        P.side_right ? "R" : "L", P.pred_mm);
    }
    dg_exit_k = -1;
    dg_np = 0;
    dg_join_corner = -1;
    dg_locked = dg_missed = 0;
}

// Last packet's corridor lateral, kept for the lap report: the corner-close
// boundary is tested BEFORE this packet's lateral is computed, so the close
// records the estimate from one packet (~2 ms) earlier -- indistinguishable
// at these speeds, and it keeps the close inside the same walk block.
static float lr_lat_mm = 0.0f;
static bool lr_lat_valid = false;

// ---------------------------------------------------------------- the gate
//
// One gate, walked forward along the route. corner_ix is the corner it is
// currently working on; it advances once the mouse is past that corner's exit.
// Arming is keyed on DISTANCE to the expected drop, and disarming is what
// lets the next corner arm -- so "the next corner cannot arm until this one
// has closed" is structural rather than arithmetic.
static WallGate gate;
// (corner_ix is declared above, beside the lane-change step test)
static RouteGateStats gstats;
// Outcome of the corner currently open, resolved when it closes. Counting at
// the packet would count a SKIP once per packet -- the gate stays armed after
// one so a cleaner reading can still anchor the corner -- and a corner is one
// event, not a rate.
static bool cur_skipped_lat = false;
// This corner's outcome is already decided -- it was too late to gate at all.
// Without it the arm test would re-fire on every packet of the approach.
static bool gate_resolved = false;

// Path-s at which the drop is EXPECTED to be observed.
//
// corner_tangent_mm(), NOT the radius. They are the same number only for a
// pure arc; with a clothoid the primitive consumes T = R + m + x0 of corridor.
// Using R would put every expectation T-R too early and turn the anchor into
// a systematic bias -- the exact class of error that cost the ring runs
// 120-135.
// `goff` (registry): one signed offset on every expectation, captured at
// launch, PLUS the per-corner-type offsets (goffr/goffl/goffur/gofful) --
// the corner-type database's first column, applied here in the follower.
// Positive = expect the drop later = the gate's correction leaves the mouse
// believing it is that much behind = the turn starts LATER in the world.
// This steers GATED corners only: a NO_EDGE corner takes no correction, so
// its type offset never engages -- honest by construction. It shifts expect
// and live_from together (both derive from this function), so the decide
// window moves with it.
static float gate_off_live = 0.0f;
static float goff_type_live[4] = {0, 0, 0, 0};  // R90, L90, UR, UL
static float gate_klat_live[2] = {0, 0};        // [left, right] beam, per launch
static float arc_trim_live[2] = {0, 0};         // [90s, Us] -- atrim/utrim
static float fgate_live = 0.0f;                 // fgate (0 = off)
static float front_trim_live = 0.0f;            // ftrim
static float gate_window_live = 0.0f;           // accept window of the armed gate
static float fg_ring[FGATE_RING];     // implied along-track error, mm
static int fg_n = 0, fg_ix = 0;
static float fg_first_s = 0.0f;       // s at acquisition (span anchor)
static float fg_last_range = 0.0f;    // last beam range folded in (report)
// WHY the front gate stayed silent, counted per leg and reported once at
// the end (stationary). Coverage is the open question: all 8 CAL-6 corners
// have a front wall, yet only 1-2 fire per lap, and the corners that go
// begging are exactly the NO_EDGE ones the side gate cannot do either.
// 0 = none yet, 1 = ring spread, 2 = drift over travel, 3 = outside window.
static uint8_t fg_last_fail = 0;
static uint8_t fg_stat_fired = 0, fg_stat_nosamp = 0, fg_stat_short = 0,
               fg_stat_spread = 0, fg_stat_drift = 0, fg_stat_window = 0;
static float fg_first_err = 0.0f;     // its error (drift anchor)
static bool fg_fired = false;     // front gate resolved this corner
static bool fg_applied = false;   // ...and actually steered (fgate >= 2)
static float fg_med = 0.0f;       // the front correction that was applied
static int corner_type_ix(int k) {
    if (k < 0 || k >= n_turns) return -1;
    // A 45 is neither a 90 nor a U: no type offset, no atrim/utrim -- those
    // were fitted on 90s and 180s and mean nothing here. Stage one of the
    // diagonal (Route.h) collects data on 45s; it does not tune them.
    if (turns[k].sweep_deg == 45 || turns[k].sweep_deg == -45) return -1;
    // A V (Route.h) is a 90 between two diagonals: no corridor either side,
    // so the 90's trims and offsets do not apply to it either. Nor to a 135
    // (one side is the diagonal).
    if (route_turn_is_v(turns[k]) || route_turn_is_135(turns[k])) return -1;
    bool u = turns[k].sweep_deg == 180 || turns[k].sweep_deg == -180;
    bool left = turns[k].sweep_deg > 0;
    return u ? (left ? 3 : 2) : (left ? 1 : 0);
}
// A turn that is one end of a diagonal (Route.h, ROUTE_DIAG_ENABLED).
static bool is_diag_turn(int k) {
    return k >= 0 && k < n_turns &&
           (turns[k].sweep_deg == 45 || turns[k].sweep_deg == -45 ||
            route_turn_is_v(turns[k]) || route_turn_is_135(turns[k]));
}
static bool is_v_turn(int k) {
    return k >= 0 && k < n_turns && route_turn_is_v(turns[k]);
}
static bool is_diag_exit(int k) {
    return k >= 0 && k < n_turns && turns[k].diag_mm > 0;
}
// A 135-IN arrives along a corridor (Route.h, THE 135): it has the same
// inside wall-end and the same front face a 90 in that cell would have, and
// unlike a 45-in its arc begins AFTER the side edge drops (130 mm before the
// cell centre against the drop at 148), so both gates can place it. It is
// still diagonal family for everything downstream (no type trims, the post
// observer wakes at its arc end).
static bool is_135_in(int k) {
    return k >= 0 && k < n_turns && route_turn_is_135(turns[k]) && turns[k].diag_mm == 0;
}
// The diagonal lateral SET POINT at path-s: dlat, pushed OUTWARD by vout
// around a V (Control.h v_out_mm) -- ramped in over the post pitch before
// the V's arc (from the last outer post) and back out over the post pitch
// after it (to the first outer post). Outward = away from the V's inside,
// i.e. right of a left V (+ lateral is left). exit_k is the diagonal's
// exit turn; its entry turn is exit_k - 1.
static float diag_set_point(float s_now, int exit_k) {
    (void)exit_k;             // dg_lane is built for dg_exit_k by dg_begin
    dg_lane.base = diag_lat_live;
    return dg_lane.sp(s_now);
}
// The arc trim a corner takes: atrim for 90s, utrim for Us, nothing for a 45.
static float trim_for(int k) {
    int ti = corner_type_ix(k);
    if (ti < 0) return 0.0f;
    float t = (ti == 2 || ti == 3) ? arc_trim_live[1] : arc_trim_live[0];
    // ...plus this corner's own hand-set trim (GoffTable.h): the lever
    // that moves ONE arc without the learner cancelling it.
    if (k < n_turns)
        t += goff_table_trim_mm(turns[k].cx, turns[k].cy, turns[k].entry_h);
    return t;
}
// PATH-s of the turn cell's CENTRE. For a 90 that is the corner point
// (entry + tangent); a diagonal's corner point sits cp_off from the centre
// (-90 on the way in, +90 on the way out), and both the side-gate post and
// the front face are reckoned from the centre, not the corner point -- see
// the diagonal note in Route.h.
static float corner_centre_s(int k) {
    float cp = path.corner_entry_s_mm(k) + path.corner_tangent_mm(k);
    if (k >= 0 && k < n_turns) cp -= (float)turns[k].cp_off_mm;
    return cp;
}
// The full tunable/learned offset stack for corner k -- shared by the side
// gate's expectation AND the front gate's wall-face position, so the glearn
// integrator converges to zero through WHICHEVER gate fires.
static float gate_off_all(int k) {
    float off = gate_off_live;
    int ti = corner_type_ix(k);
    if (ti >= 0) off += goff_type_live[ti];
    // The learned per-corner layer, keyed by cell + maze-frame approach
    // heading (GoffTable.h) -- what turns the grid into THIS rig's grid.
    if (k >= 0 && k < n_turns)
        off += goff_table_mm(turns[k].cx, turns[k].cy, turns[k].entry_h);
    return off;
}
static float gate_expect_s(int k) {
    return corner_centre_s(k) - 0.5f * ROUTE_CELL_MM -
           WALL_GATE_BEAM_AHEAD_MM + gate_off_all(k);
}
// Where the FRONT wall's face sits along the path: the corner datum (cell
// centre crossing) + 84 (cell boundary 90 minus half a 12 mm wall). Same
// offset stack as the side expectation -- see gate_off_all.
static float fgate_face_s(int k) {
    // ftrim moves the believed FACE nearer by its own value, so a standing
    // FRONT corr is cancelled by setting ftrim to it (bert, 2026-08-30:
    // "needs a trim in the parameter table"). The front beams read the
    // face at 216-254 mm, well beyond where the front LUT was calibrated
    // (contact, 45 mm), so a long-range scale/offset error is expected and
    // is NOT something the learner may absorb -- see the learning note.
    return corner_centre_s(k) + (0.5f * ROUTE_CELL_MM - 6.0f) +
           gate_off_all(k) - front_trim_live;
}

// Run left between the drop and the arc: 90 + 45 - T. The correction is
// drained over exactly this, because it shrinks with radius and a fixed rate
// would spill a large-R correction into the arc it was meant to set up.
static float gate_room(int k) {
    float room = path.corner_entry_s_mm(k) - gate_expect_s(k);
    return room > 5.0f ? room : 5.0f;
}

static void gate_arm_for(int k) {
    WallGateConfig gc;
    gc.expect_s_mm = gate_expect_s(k);
    // Accept window scaled by anchor age -- how far this expectation rides
    // on odometry since the last known good post (see the RR_GATE_WINDOW
    // constants). Well inside ROUTE_GATE_DECIDE_MM, so the gate is always
    // live before the window opens.
    {
        float age = gc.expect_s_mm - anchor_s_mm;
        if (age < 0) age = 0;
        float w = RR_GATE_WINDOW_BASE_MM + RR_GATE_WINDOW_GROW * age;
        gc.window_mm = w < RR_GATE_WINDOW_MAX_MM ? w : RR_GATE_WINDOW_MAX_MM;
        gate_window_live = gc.window_mm;   // the front gate shares the window
        fg_n = 0;
        fg_ix = 0;
        fg_fired = false;
        fg_applied = false;
        fg_first_s = 0.0f;
        fg_last_range = 0.0f;
        fg_last_fail = 0;
    }
    // Lateral drop compensation, PER BEAM: a left turn watches the left
    // beam (slope ~0.18), a right turn the right beam (~0.68) -- bert's
    // 20 mm bench offset measured them 4x apart (2026-08-29).
    gc.klat = path.corner_dir(k) > 0 ? gate_klat_live[0] : gate_klat_live[1];
    // OBSERVE WIDE, DECIDE NARROW. The gate is fed from far out so its
    // presence run is built on real wall, but it may only ACT within
    // ROUTE_GATE_DECIDE_MM of the expectation -- otherwise a gap or a wall end
    // earlier on the approach is an edge too, and a maze approach has far more
    // of those than a ring leg ever did.
    gc.live_from_s_mm = gc.expect_s_mm - ROUTE_GATE_DECIDE_MM;
    gate.arm(gc);
}

// ---------------------------------------------------------------- helpers

// A PESSIMISTIC flood is the whole definition of "a good maze is stored":
// unknown edges count as walls, so a finite distance at the start means every
// edge on the route has been looked at.
static bool flood_for_leg(Maze &m, bool to_goal) {
    if (to_goal) m.flood_to_goal(false);
    else m.flood_to_start(false);
    return true;
}

bool route_run_available() {
    Maze m = search_maze();          // a COPY: a query must not move the model
    m.flood_to_goal(false);
    return m.distance(0, 0) != MAZE_DIST_UNREACHABLE;
}

// Compile the leg the mission is due to fly. Returns ROUTE_OK and leaves the
// path ready, or reports why not.
static RouteResult compile_leg() {
    Maze m = search_maze();
    // The launch datum (`starty`, ControlGains) -- read here so a change
    // takes effect at the next compile, and the datum sequence's
    // forward-to-centreline move (search_start_y_mm) agrees with it.
    search_set_start_y_mm(control_get_gains().start_y_mm);
    float lead = at_start_setback
                     ? 0.5f * ROUTE_CELL_MM - search_start_y_mm()
                     : 0.0f;
    // The mission numbers come from the registry (rv/ralon/ralat/rslew), so
    // a speed step during a tuning session is `rv=600`, not a reflash -- and
    // the leg is compiled fresh each arm, so the geometry always matches.
    ControlGains g = control_get_gains();
    // The rail contract, at runtime now that the rail is tunable: the U-turn
    // geometry assumes at least PATH_UTURN_MAX_RAIL_DPS2 of ramp, and a rail
    // below it would clip every clothoid in the leg. Floor it and say so.
    float slew = g.route_slew;
    if (slew < PATH_UTURN_MAX_RAIL_DPS2) {
        printf_line("route: rslew %.0f is below the geometry's %.0f -- using "
                    "the floor", (double)slew,
                    (double)PATH_UTURN_MAX_RAIL_DPS2);
        slew = PATH_UTURN_MAX_RAIL_DPS2;
    }
    // Corner geometry solves at the speed each corner is actually flown
    // (ralat cap), and the U radius comes from the registry (`rur`) --
    // re-set every compile so a stale context cannot outlive a leg.
    route_set_corner_context(g.route_alat, g.route_uturn_r);
    // Per-direction 90 radius overrides (`r90`/`l90`, 0 = compiled ladder).
    route_set_r90(g.r90_right, g.r90_left);
    // The V's outward offset (`vout`): a bigger V arc on offset lines.
    route_set_v_out(g.v_out_mm);
    route_set_135(g.route_135 >= 0.5f);
    // rvv / rvu (Route.h): the V (and what it hands into) and the U never
    // fly faster than these, whatever the rung.
    route_set_turn_caps(g.v_cap_v, g.v_cap_u);
    // rdiag: 0 = orthogonal only. THE HOME LEG IS ALWAYS ORTHOGONAL (bert,
    // 2026-09-27, after the home diagonal clipped the 135-out's post: "maybe
    // the return should be smooth 90's if that's safer"). The return is not
    // timed; a crash on it costs the next run. The out leg keeps rdiag.
    // rdv (2026-10-03, Hazlemere): diagonals only when the leg's peak is at
    // or under rdv. Both MR4 out legs on the full-size maze hit a post at a
    // V's exit (the lane change cannot walk vout back at 500-760 mm/s); MR2
    // and MR3 flew the same diagonals to the goal. 0 = no limit.
    const bool diag_ok = g.route_diag >= 0.5f && !home_leg() &&
                         (g.diag_v_max <= 0.0f || leg_speed() <= g.diag_v_max);
    route_set_diag(diag_ok);
    if (g.route_diag >= 0.5f && !home_leg() && !diag_ok)
        printf_line("route: peak %.0f is above rdv %.0f -- this leg flies ORTHOGONAL",
                    (double)leg_speed(), (double)g.diag_v_max);
    // rturn: turns cost cells when the route is chosen -- rturnh on the
    // home leg (untimed: a turn avoided is worth a long detour there).
    route_set_turn_cost(home_leg() && g.route_turn_cost_home > 0.0f ? g.route_turn_cost_home
                                                                     : g.route_turn_cost);
    RouteResult r;
    leg_peak_v = leg_speed();
    // The limits this leg will fly at, so the compiler can time the
    // candidate routes the way they will be flown (Route.h, "chosen by
    // flight time"). Same k as the set_limits after the compile.
    {
        float kk = home_leg() ? 1.0f : run_ladder_accel_scale(cur_mr, g);
        route_set_time_context(leg_peak_v, g.route_alon, g.route_alat * kk,
                               g.route_brk, g.clothoid_hold > 0.5f, slew);
    }
    // The circuit is tried on a run's FIRST leg only: the home leg of a
    // two-leg run is a one-way compile from the goal by definition.
    bool as_loop = ROUTE_RUN_OUT_AND_BACK && !home_leg();
    if (as_loop) {
        // route_build_loop does both floods itself, on this COPY of the maze.
        // route.dir chooses which way round the circuit; if the maze cannot
        // honour it (not a loop, or the forced side is walled) fall back to
        // the unforced build rather than refusing to fly.
        // route.dir=auto exits the way the nose points: a launch cannot
        // begin with a turn (no run-up in the start cell), so the pose
        // heading IS the only honest exit -- and forcing it keeps the out
        // flood off a shorter arm that leaves sideways. On D7 auto used to
        // let the out leg take the 10-cell east arm from a north-facing
        // pose (impossible in-cell 90) and the home leg walk straight back
        // (ROUTE_BAD_TURN) -- "mouse wouldn't run" (bert, 2026-09-04).
        const int force = route_dir >= 0 ? route_dir : (int)pose_h;
        r = route_build_loop(m, pose_x, pose_y, pose_h, path, leg_peak_v,
                             slew, &stats, turns,
                             ROUTE_MAX_TURNS, lead, force);
        if (r != ROUTE_OK) {
            printf_line("route.dir=%c: no route out the %c side -- "
                        "compiling unforced", route_run_get_dir(),
                        "NESW"[force & 3]);
            r = route_build_loop(m, pose_x, pose_y, pose_h, path, leg_peak_v,
                                 slew, &stats, turns,
                                 ROUTE_MAX_TURNS, lead, -1);
        }
        if (r != ROUTE_OK) {
            // NOT A CIRCUIT (or one the compiler cannot fly as one walk):
            // two legs. The maze is a copy; the floods it holds are
            // whatever the loop left, so flood again below.
            printf_line("route: no continuous loop through the goal (%s) -- "
                        "TWO LEGS: fly to the goal, turn in place, fly home",
                        route_result_name(r));
            two_leg = true;
            leg_to_goal = true;
            as_loop = false;
        }
    }
    if (!as_loop && home_leg() && goal_loop) {
        // THE GOAL LOOP (Route.h): the home walk starts through the block.
        r = route_build_goal_loop(m, pose_x, pose_y, pose_h, path, leg_peak_v,
                                  slew, &stats, turns, ROUTE_MAX_TURNS);
        if (r != ROUTE_OK) {
            printf_line("route: the goal loop did not compile (%s)", route_result_name(r));
            n_turns = 0;
            return r;
        }
        n_turns = stats.turns;
        float k = 1.0f;   // the home leg flies the registry's accelerations
        path.set_limits(leg_peak_v, g.route_alon, g.route_alat * k, g.route_brk);
        path.set_clothoid_hold(g.clothoid_hold > 0.5f);
        int ex, ey; Heading eh;
        if (!route_goal_loop_end(&ex, &ey, &eh)) { n_turns = 0; return ROUTE_NO_PATH; }
        end_x = (uint8_t)ex; end_y = (uint8_t)ey; end_h = eh;
        return ROUTE_OK;
    }
    if (!as_loop) {
        flood_for_leg(m, leg_to_goal);
        r = route_build(m, pose_x, pose_y, pose_h, path, leg_peak_v,
                        slew, &stats, turns, ROUTE_MAX_TURNS,
                        lead);
        // A HOME LEG TOO LONG TO FLY ORTHOGONAL (2026-10-02, the eve of the
        // full-size test day): on a 16x16 the orthogonal home leg can need
        // more than PATH_MAX_CORNERS arcs -- every staircase step is one --
        // and "cannot compile this leg" would strand the mouse at the goal.
        // 19 of 300 random 16x16 mazes did; with diagonals every one of them
        // compiled (40-51 arcs). The orthogonal rule is a preference, not a
        // reason to stay at the goal: fly it with diagonals.
        if (r == ROUTE_PATH_FULL && home_leg() && g.route_diag >= 0.5f) {
            printf_line("route: the orthogonal home leg needs more than %d arcs -- "
                        "compiling it WITH diagonals", PATH_MAX_CORNERS);
            route_set_diag(true);
            flood_for_leg(m, leg_to_goal);
            r = route_build(m, pose_x, pose_y, pose_h, path, leg_peak_v,
                            slew, &stats, turns, ROUTE_MAX_TURNS, lead);
        }
    }
    if (r != ROUTE_OK) {
        n_turns = 0;
        return r;
    }
    n_turns = stats.turns;
    {
        // The rung's accelerations (RunLadder.h, mra): rbrk 0 stays "= ralon"
        // through the scale since 0 x k is 0. THE HOME LEG DOES NOT FOLLOW
        // THE LADDER (bert, 2026-09-26, the first clean MR2-MR5 on the new
        // maze: "the return leg should not follow the acceleration ladder").
        // It flies at rvh, the banker's 500, on every rung; scaling its
        // accelerations too made it the slow half of every lap (8116 ms at
        // MR2 against 6076 at MR5 for the same 500 mm/s peak) for no
        // safety it needed -- at 500 the corners are at their radius speed
        // whatever the scale. The out leg is where the ladder climbs.
        // THE LADDER IS THE CORNERS' (bert, 2026-10-04: "the long straights
        // should be at max accel, not limiting by max V"): the rung scales
        // ralat only; ralon and rbrk are the registry's on every rung.
        float k = home_leg() ? 1.0f : run_ladder_accel_scale(cur_mr, g);
        path.set_limits(leg_peak_v, g.route_alon, g.route_alat * k, g.route_brk);
        path.set_clothoid_hold(g.clothoid_hold > 0.5f);   // chold (Path.h)
    }

    if (as_loop) {
        // A loop ends where it began. The FACING it ends on depends on the
        // rig (the original outer loop came home southbound; CAL-6 CW comes
        // home WESTBOUND; CCW comes home southbound again) -- so end_h here
        // is only a placeholder. leg_finished() overwrites pose_h from the
        // LIVE fused yaw, which knows the truth without caring which rig or
        // direction was flown. (This placeholder being trusted is what would
        // have turned the first hands-free lap chain into a crash: the old
        // hardcoded SOUTH met a CAL-6 lap that actually ends facing WEST.)
        end_x = pose_x;
        end_y = pose_y;
        end_h = heading_back(HEADING_N);
        return ROUTE_OK;
    }

    // Walk the same flood again to learn where the leg ENDS. route_build
    // already did this walk; re-doing it here rather than plumbing the answer
    // out keeps the compiler's interface about geometry and this about the
    // mission's bookkeeping.
    // ...by the SAME walk (rturn's weighted walk enters a goal region from
    // whichever side costs least, which need not be the plain flood's side).
    int cx = pose_x, cy = pose_y;
    Heading h = pose_h;
    if (!route_walk_end(m, cx, cy, h, &cx, &cy, &h)) {
        n_turns = 0;
        return ROUTE_NO_PATH;
    }
    end_x = (uint8_t)cx;
    end_y = (uint8_t)cy;
    end_h = h;
    return ROUTE_OK;
}

static void stop_everything() {
    // BRAKE, do not coast. control_velocity_enable(false) alone leaves a
    // moving mouse rolling on its own momentum into whatever is ahead -- run
    // 151 is what that looks like at speed.
    control_brake_to_stop();
    control_move_abort();
    control_velocity_enable(false);
    control_set_lateral_bias_deg(0.0f);
    control_set_reflex_bias_deg(0.0f);
    flog_stop();
    sensors_stop();
}

static void recover_begin(uint32_t now) {
    state = RR_RECOVER;
    state_t0 = now;
    led_phase = false;
    hal_set_led1(false);
    hal_set_led2(false);
    sensors_free_run_interlaced();   // the stream stays up: no power cycle
    printf_line("route: RECOVERY -- %lu s to put the mouse back on the start "
                "cell, against the rear wall facing NORTH; then the wave "
                "re-arms by itself at the same rung (no power cycle). "
                "'route.stop' cancels.", (unsigned long)(RR_RECOVER_MS / 1000));
}

void route_run_abort(const char *why) {
    if (state == RR_IDLE) return;
    if (state == RR_RECOVER) {
        hal_set_led1(false);
        hal_set_led2(false);
        sensors_stop();
        state = RR_IDLE;
        printf_line("route: recovery cancelled (%s) -- idle", why);
        return;
    }
    if (state == RR_FAULT) {
        hal_set_led1(false);
        hal_set_led2(false);
        sensors_stop();
        state = RR_IDLE;
        printf_line("route: sensor fault cleared (%s) -- idle; arm again, or "
                    "power the mouse off for a good few seconds first if the "
                    "start gate still reads 255", why);
        return;
    }
    // Where the leg died, captured BEFORE stop_everything tears the path
    // down -- the abort report's mm field.
    float abort_s = control_path_status().s_mm;
    const bool was_flying = state == RR_FLY;
    uint32_t abort_ms = was_flying ? millis() - state_t0 : 0;
    stop_everything();
    state = RR_IDLE;
    hand_active = false;
    hand_run = 0;
    rung_pending = false;
    goal_datum = false;
    goal_loop = false;
    goal_turn_pending = false;
    // The abort report belongs to a leg that was FLYING. Stopped between
    // legs (the goal turn, the lap-end datum), the last leg's report has
    // already been filed and a second, empty one would only confuse the
    // scoreboard.
    if (was_flying) {
        ControlVelStatus vs = control_velocity_status();
        printf_line("route: leg ABORTED -- %s  (duty peak %.2f of %.2f)", why,
                    (double)vs.duty_peak,
                    (double)control_get_gains().duty_limit);
        // The unfinished lap's report, marked abort=1 (bert, 2026-08-30) --
        // corners recorded so far, gates as they resolved, LPARAM as flown.
        // The mouse is stopped; the stationary transmit rule holds.
        lap_report_set_vbat(vbat_mv());
        lap_report_abort(abort_ms, abort_s, vs.duty_peak);
        // The diagonal's post table too: an aborted lap on a diagonal is
        // exactly the lap whose posts need reading (D8, 2026-09-06).
        dg_report();
        rfx_report();
        fr_report();
        ls_report();
    } else {
        printf_line("route: STOPPED between legs -- %s", why);
    }
    printf_line("  the mouse is NOT where the mission thinks it is. Put it "
                "back on the start cell facing NORTH.");
    // It does not re-arm NOW: a leg that ended early ended somewhere
    // unknown, and arming from an unknown pose is how a recovery becomes a
    // second crash. It re-arms after RR_RECOVER_MS, from the start-cell
    // convention, for a hand start (RR_RECOVER).
    recover_begin(millis());
}

// ---------------------------------------------------------------- launch
// The map's verdict on the launch cell's sides, against the tally. A side
// with a KNOWN wall that never returned a live reading is a dead sensor.
enum PreflightVerdict { PF_OK, PF_WAIT, PF_FAULT, PF_POSE };
static PreflightVerdict preflight_check(uint32_t now) {
    const Maze &m = search_maze();
    Heading hl = heading_left(pose_h), hr = heading_right(pose_h);
    bool wall_l = m.wall_known(pose_x, pose_y, hl) &&
                  m.wall_present(pose_x, pose_y, hl);
    bool wall_r = m.wall_known(pose_x, pose_y, hr) &&
                  m.wall_present(pose_x, pose_y, hr);
    if (pf_pkts < RR_PREFLIGHT_MIN_PKTS) {
        // No evidence yet is not a pass: wait for the stream (it fills the
        // tally in tens of ms), and a stream that stays silent is a fault
        // of its own -- the log and the rung are not spent on it.
        if (now - state_t0 < RR_TYRE_SETTLE_MS + RR_PREFLIGHT_WAIT_MS)
            return PF_WAIT;
        printf_line("route: PRE-FLIGHT FAILED -- only %d sensor packets in "
                    "%u ms stationary: the sensor stream is not running",
                    pf_pkts, (unsigned)(now - state_t0));
        return PF_FAULT;
    }
    bool dead_l = wall_l && pf_live_l == 0;
    bool dead_r = wall_r && pf_live_r == 0;
    if (goal_loop && (dead_l || dead_r) &&
        (!dead_l || fly_live_l >= RR_BEAM_PROVEN_PKTS) &&
        (!dead_r || fly_live_r >= RR_BEAM_PROVEN_PKTS)) {
        // THE GOAL LOOP launches from the arrival pose, where the stop may
        // have fallen short of the cell and a side beam looks past a wall
        // end; the beams proved themselves on the leg just flown, and the
        // loop's first corners are gated on the block's walls.
        printf_line("route: pre-flight (goal loop) -- %d packets, left %s (%d live), "
                    "right %s (%d live); a mapped wall unseen at the stop is "
                    "tolerated here", pf_pkts,
                    wall_l ? "wall" : "open/unknown", pf_live_l,
                    wall_r ? "wall" : "open/unknown", pf_live_r);
        return PF_OK;
    }
    if (!dead_l && !dead_r) {
        printf_line("route: pre-flight OK -- %d packets, left %s (%d live), "
                    "right %s (%d live)", pf_pkts,
                    wall_l ? "wall" : "open/unknown", pf_live_l,
                    wall_r ? "wall" : "open/unknown", pf_live_r);
        return PF_OK;
    }
    // A beam proven alive on the leg just flown is not a dead sensor: the
    // map and the mouse disagree about WHERE it is (the goal stop is
    // dead-reckoned from the last corner). That leg is not flown.
    bool proven_l = !dead_l || fly_live_l >= RR_BEAM_PROVEN_PKTS;
    bool proven_r = !dead_r || fly_live_r >= RR_BEAM_PROVEN_PKTS;
    if (proven_l && proven_r) {
        printf_line("route: POSE NOT CONFIRMED -- the map has a wall on the %s%s%s "
                    "of (%u,%u) facing %s and that beam reads 255 (%d packets), "
                    "yet it read live %d/%d times on the leg just flown: the "
                    "mouse is not where the leg thinks it stopped. This leg is "
                    "NOT flown.",
                    dead_l ? "LEFT" : "", (dead_l && dead_r) ? " and " : "",
                    dead_r ? "RIGHT" : "", pose_x, pose_y, h_name(pose_h), pf_pkts,
                    dead_l ? fly_live_l : fly_live_r,
                    (dead_l && dead_r) ? fly_live_r : (dead_l ? fly_live_l : fly_live_r));
        return PF_POSE;
    }
    printf_line("route: PRE-FLIGHT FAILED -- the map has a wall on the %s%s%s "
                "of (%u,%u) facing %s and that beam read 255 for all %d "
                "packets: SENSOR FAULT (power-on too soon after power-off?)",
                dead_l ? "LEFT" : "", (dead_l && dead_r) ? " and " : "",
                dead_r ? "RIGHT" : "", pose_x, pose_y, h_name(pose_h), pf_pkts);
    return PF_FAULT;
}

static void fault_enter(uint32_t now) {
    stop_everything();
    state = RR_FAULT;
    state_t0 = now;
    led_phase = false;
    hal_set_led1(false);
    hal_set_led2(false);
    printf_line("  LEDs flash 100 ms every 2 s for %lu s, then the wave re-arms "
                "for another hand start (the pre-flight runs again). "
                "'route.stop' clears the fault now.",
                (unsigned long)(RR_RECOVER_MS / 1000));
}

static void leg_launch(uint32_t now) {
    // THE SENSOR GATE. Everything that costs something -- the ladder rung,
    // the flight log (flog_arm below overwrites the last run's log) -- sits
    // BEHIND this check. A wave the sensors veto leaves FRAM exactly as it
    // was: the previous log still dumps with 'y', the rung is still owed.
    PreflightVerdict pf = preflight_check(now);
    if (pf == PF_WAIT) return;   // still RR_SETTLE: the poll calls again
    if (pf == PF_FAULT) {
        fault_enter(now);
        return;
    }
    if (pf == PF_POSE) {
        // Not a fault: straight to the recovery window. The leg before this
        // one is complete and reported (the rung climbed on its goal).
        stop_everything();
        hand_active = false;
        hand_run = 0;
        rung_pending = false;
        goal_datum = false;
        goal_turn_pending = false;
        recover_begin(now);
        return;
    }
    fly_live_l = fly_live_r = 0;
    if (rung_pending) {
        rung_pending = false;
        cur_mr = run_ladder_take();
        if (cur_mr) {
            ControlGains g = control_get_gains();
            float k = run_ladder_accel_scale(cur_mr, g);
            printf_line("route: this is MR%u -- out at %.0f, home at %.0f, "
                        "corners x%.2f (ralat %.0f; ralon %.0f rbrk %.0f every rung%s; "
                        "V cap %.0f, U cap %.0f; home leg x1.00) "
                        "(climbs to the next rung only if it reaches the goal)", (unsigned)cur_mr,
                        (double)run_ladder_out_v(cur_mr, g),
                        (double)run_ladder_home_v(cur_mr, g), (double)k,
                        (double)(g.route_alat * k), (double)g.route_alon,
                        (double)(g.route_brk > 0.0f ? g.route_brk : g.route_alon),
                        g.clothoid_hold > 0.5f ? ", clothoid hold" : "",
                        (double)g.v_cap_v, (double)g.v_cap_u);
        }
    }
    led_dark();   // in flight the LEDs speak only in event flashes
    RouteResult r = compile_leg();
    if (r != ROUTE_OK) {
        printf_line("route: cannot compile this leg -- %s",
                    route_result_name(r));
        stop_everything();
        goal_loop = false;
        if (home_leg()) { recover_begin(now); return; }   // the out leg is scored; recover
        state = RR_IDLE;
        return;
    }
    printf_line("route: %s at peak %.0f mm/s%s, %u cells, %u turns (%u below "
                "R%u, %u U-turn%s, %u diagonal%s%s, %u x 135), %.0f mm%s",
                leg_name_long(), (double)leg_peak_v,
                cur_mr ? (home_leg() ? " (rvh)" : " (ladder)") : " (rv)",
                stats.cells, stats.turns, stats.dropped, ROUTE_RADII[0],
                stats.uturns, stats.uturns == 1 ? "" : "s",
                stats.diags, stats.diags == 1 ? "" : "s",
                route_diag_enabled() ? "" : " -- rdiag 0, ORTHOGONAL ONLY",
                stats.turns135,
                (double)stats.length_mm,
                at_start_setback ? ", +lead-in from the rear wall" : "");
    {
        // Which route, and why (Route.h, "chosen by flight time").
        const RouteChoice &ch = route_last_choice();
        if (ch.compared) {
            for (int c = 0; c < 2; c++)
                printf_line("   %s %-12s %2u cells %2u turns%s %u diag, %4.0f mm, "
                            "flies %4.0f ms, score %4.0f",
                            ch.chosen == c ? ">>" : "  ",
                            c == 0 ? "fewest cells" : "fewest turns",
                            ch.cand[c].cells, ch.cand[c].turns,
                            ch.cand[c].uturns ? " (U x2)" : "",
                            ch.cand[c].diags, (double)ch.cand[c].length_mm,
                            (double)ch.cand[c].ms, (double)ch.cand[c].score);
        } else if (ch.same_walk) {
            printf_line("   fewest cells and fewest turns are the same route");
        }
    }
    if (stats.passes > 1 || stats.slowed)
        // The multi-pass compile (Route.h, NEVER REFUSE A VALID MAZE): what
        // it had to give up, and where, so a slow lap is explained before
        // it is flown.
        printf_line("   %u compile pass%s: %u feature%s un-merged, %u corner%s "
                    "speed-capped (@mm/s below)",
                    stats.passes, stats.passes == 1 ? "" : "es",
                    stats.vetoed, stats.vetoed == 1 ? "" : "s",
                    stats.slowed, stats.slowed == 1 ? "" : "s");
    if (ROUTE_RUN_OUT_AND_BACK && !two_leg)
        printf_line("   the goal is cell %u of %u -- flown straight through",
                    stats.goal_ix, stats.cells);
    else if (two_leg)
        printf_line("   leg %d of 2: ends at (%u,%u) facing %s%s",
                    leg_to_goal ? 1 : 2, end_x, end_y, h_name(end_h),
                    leg_to_goal ? " -- stop, turn in place, fly home"
                                : " -- the lap end datum follows");
    // A full-size leg can have 100+ turns: list the first 40 (the launch
    // waits for the radio on every line), then a count.
    const int list_n = n_turns < 40 ? n_turns : 40;
    if (n_turns > list_n)
        printf_line("   (%d turns -- the first %d listed)", n_turns, list_n);
    for (int j = 0; j < list_n; j++) {
        bool u = turns[j].sweep_deg == 180 || turns[j].sweep_deg == -180;
        bool d = is_diag_turn(j);
        char cap[16] = "";
        if (turns[j].v_cap_mm_s)
            snprintf(cap, sizeof cap, "  @%u", (unsigned)turns[j].v_cap_mm_s);
        printf_line("   turn %d (%u,%u) R%u %s%s%s%s%s", j, turns[j].cx,
                    turns[j].cy, turns[j].r_mm,
                    turns[j].sweep_deg > 0 ? "LEFT " : "RIGHT",
                    u ? "  180 U-TURN" : "",
                    d ? (is_v_turn(j) ? "  90 V: DIAGONAL TO DIAGONAL"
                         : route_turn_is_135(turns[j])
                             ? (is_diag_exit(j) ? "  135 DIAGONAL OUT (one arc)"
                                                : "  135 DIAGONAL IN (one arc)")
                         : is_diag_exit(j) ? "  45 DIAGONAL OUT"
                                           : "  45 DIAGONAL IN")
                      : "",
                    (!u && turns[j].r_mm < ROUTE_RADII[0]) ? "  << dropped"
                                                           : "",
                    cap);
        if (is_diag_exit(j))
            printf_line("      %u mm of diagonal, corner point to corner "
                        "point -- OBSERVED, not gated (stage one)",
                        (unsigned)turns[j].diag_mm);
    }

    // Size the flight log's record spacing from the compiled leg (see
    // RR_LOG_DS_MIN_MM): the whole run -- both legs of a two-leg run --
    // must fit RR_LOG_BUDGET of the region, launch included.
    {
        float legs = (two_leg && leg_to_goal) ? 2.0f : 1.0f;
        float budget = RR_LOG_BUDGET * (float)FLOG_CAPACITY -
                       (home_leg() ? (float)flog_count() : 0.0f);
        if (budget < 1000.0f) budget = 1000.0f;
        log_ds_mm = legs * stats.length_mm / budget;
        if (log_ds_mm < RR_LOG_DS_MIN_MM) log_ds_mm = RR_LOG_DS_MIN_MM;
        log_last_s = -1e9f;
        log_last_ms = 0;
        log_event = false;
        log_last_in_arc = false;
        printf_line("route: flight log one record per %.1f mm + events "
                    "(~%.0f records this leg, region %lu, %lu used)",
                    (double)log_ds_mm, (double)(stats.length_mm / log_ds_mm),
                    (unsigned long)FLOG_CAPACITY,
                    (unsigned long)(home_leg() ? flog_count() : 0));
    }

    // Zero the duty peak so it measures THIS leg. The ceiling is a constant
    // now (the thermal budget went on 2026-08-27), so there is nothing to
    // report before the run -- only afterwards, and only one number: how close
    // the clamp came to binding.
    control_duty_peak_reset();

    // TELL THE EXECUTOR WHAT THE PATH WAS COMPILED WITH. It was never told,
    // and the two did not agree.
    //
    // A clothoid IS a promise about the rate ramp: its length is
    // v^2/(R*alpha), so a corner compiled at the rslew param (default 6000) hands
    // the executor a reference whose rate rises at 6000 deg/s^2. The executor
    // was railing it at PATH_OMEGA_SLEW_DPS2 = 4000 -- or at whatever the last
    // `sweep` left behind, because LoopSweep is the only other caller of
    // control_set_path_slew and nothing restores it.
    //
    // At R84/v500 the arc rate is 341 deg/s. The geometry allows 28.4 mm of
    // clothoid to reach it; a 4000 rail needs 43 mm. So the mouse was still
    // winding up 15 mm INTO the constant-radius arc, on every corner, having
    // under-rotated for the whole transition -- which is sign-independent,
    // proportional to commanded rate, and integrates into cross-track. That is
    // the bulge's signature exactly, and Control.h has been describing the
    // mechanism since the rail was introduced: "ask the plant for 234 deg/s
    // instantly and it will not deliver, it will enter late and run wide."
    //
    // The clothoids exist so the rail never has to bind. Setting it here is
    // what makes that true.
    {
        // The rail the leg was COMPILED with (compile_leg floors it the same
        // way), so geometry and executor cannot disagree.
        float slew = control_get_gains().route_slew;
        if (slew < PATH_UTURN_MAX_RAIL_DPS2) slew = PATH_UTURN_MAX_RAIL_DPS2;
        control_set_path_slew(slew);
    }

    lat_over_run = 0;
    beam_close_run = 0;
    last_sighted_s = 0;
    anchor_s_mm = 0;
    seg_dv_acc = seg_dv_brk = seg_v_last = 0.0f;   // launch: a fresh segment
    slip_armed = false;
    learn_n = 0;
    vlearn_n = 0;
    dg_dr.clear();
    dg_v_learn_k = -1;
    v_shift_k = -1;
    v_shift_mm = 0.0f;
    fg_stat_fired = fg_stat_nosamp = fg_stat_short = 0;
    fg_stat_spread = fg_stat_drift = fg_stat_window = 0;
    // Trustworthy datum = the mouse pressed itself onto the wall. A hand
    // placement (boot, or a leg that ended off the start cell) is not.
    launch_was_setback = at_start_setback && !launch_by_press;
    first_lock_pending = true;
    // A rear-wall setback launch has a machined datum; a hand placement is
    // an eyeball one, so corner 0 rides from an anchor we do not trust.
    anchor_fresh = !launch_was_setback;
    learn_skipped = 0;
    corner_ix = 0;
    corner_exit_yaw_valid = false;
    dg_exit_k = -1;
    dg_np = 0;
    dg_join_corner = -1;
    dg_locked = dg_missed = 0;
    dg_lat_valid = false;
    gate.disarm();
    cur_skipped_lat = false;
    gate_resolved = false;
    gstats = RouteGateStats{};
    gstats.corr_min_mm = 1e9f;
    gstats.corr_max_mm = -1e9f;
    control_motion_zero();
    control_velocity_enable(true);
    sensors_free_run_interlaced();
    if (home_leg()) {
        // The second leg of a two-leg run continues the out leg's log:
        // one run, one banner, one params block, the goal turn the gap.
        flog_resume();
    } else {
        flog_arm();
        // THE BANNER CARRIES THE GAINS, not just the mission name. Runs
        // 180-182 said "route: start->goal->start" and nothing else, and
        // the whole understeer analysis had to assume the COMPILED track
        // of 59 because params in FRAM override compiled defaults and no
        // log has ever recorded which one was live. The banner is 160
        // bytes; this fits in ~110.
        ControlGains g = control_get_gains();
        char b[160];
        // launch=N/E/S/W is the heading the leg actually starts on (dir
        // may say 'a' = auto): the flight viewer needs it to place a CCW
        // lap's trail on the same map as a CW one (bert, 2026-09-11).
        // A two-leg run says so ("route:S->G|G->S"): the viewer then
        // expects the s column to restart at the goal.
        // MRn v<out>/<home>: the rung and the peaks it chose (ladder off:
        // MR0 and one v).
        char mrs[24];
        if (cur_mr)
            snprintf(mrs, sizeof mrs, "MR%u v%.0f/%.0f", (unsigned)cur_mr,
                     (double)run_ladder_out_v(cur_mr, g),
                     (double)run_ladder_home_v(cur_mr, g));
        else
            snprintf(mrs, sizeof mrs, "MR0 v%.0f", (double)g.route_v);
        snprintf(b, sizeof b,
                 "%s %s trk%.1f us%.2f ta%.2f ky%.1f dmax%.2f dir%c launch=%c ds%.1f",
                 two_leg ? "route:S->G|G->S" : leg_name_short(), mrs,
                 (double)g.track_mm,
                 (double)g.arc_understeer, (double)g.tasym, (double)g.kyaw,
                 (double)g.duty_limit, route_run_get_dir(),
                 "NESW"[(int)pose_h & 3], (double)log_ds_mm);
        flog_set_banner(b);
    }
    // The cardinal the leg starts on, as an offset from the CURRENT fused
    // yaw: the IMU was just recalibrated to zero, and the mouse is square to
    // the corridor by construction on the start cell. Any later leg launches
    // from wherever the previous one left the heading, which is why the path
    // datum is taken from the live yaw rather than assumed.
    //
    // THE HOME LEG OF A TWO-LEG RUN launches from the goal after an in-place
    // turn, with NO wall to press and no recalibration: the fused yaw is
    // still the out leg's frame (0 = launch_h), which is exactly what makes
    // this leg possible. The path is built on the CORRIDOR, not the body:
    // the datum offset is (corridor yaw - live yaw), so whatever the turn
    // left over (a degree or two, the settle gate's usual) drains as a
    // heading transient on the first straight instead of rotating the whole
    // leg (Control.h, control_path_start's yaw0_offset). launch_h is NOT
    // moved -- every frame rotation below stays in the out leg's frame.
    float yaw0_off = 0.0f;
    if (home_leg()) {
        float yw = control_heading().yaw_deg;
        yaw0_off = yaw_of_heading(pose_h) - yw;
        while (yaw0_off > 180.0f) yaw0_off -= 360.0f;
        while (yaw0_off < -180.0f) yaw0_off += 360.0f;
        printf_line("route: home leg launches on %s from the goal, %+.1f deg "
                    "of turn residual to drain", h_name(pose_h),
                    (double)-yaw0_off);
    }
    if (!control_path_start(&path, yaw0_off)) {
        printf_line("route: path refused (fusion not ready?)");
        stop_everything();
        state = RR_IDLE;
        return;
    }
    if (!home_leg())
        launch_h = pose_h;   // yaw 0 = this cardinal until the next launch
    {   // gate offsets, global + per type, captured per launch
        ControlGains gg = control_get_gains();
        gate_off_live = gg.gate_off_mm;
        goff_type_live[0] = gg.goff_r90;
        goff_type_live[1] = gg.goff_l90;
        goff_type_live[2] = gg.goff_ur;
        goff_type_live[3] = gg.goff_ul;
        gate_klat_live[0] = gg.gate_klat;     // left beam (left turns)
        gate_klat_live[1] = gg.gate_klat_r;   // right beam (right turns)
        arc_trim_live[0] = gg.arc_trim_90;    // atrim
        arc_trim_live[1] = gg.arc_trim_u;     // utrim
        fgate_live = gg.front_gate;           // fgate
        front_trim_live = gg.front_trim;      // ftrim
        diag_gate_live = gg.diag_gate;        // dgate
        diag_trim_live = gg.diag_trim;        // dtrim
        diag_lat_live = gg.diag_lat;
        v_out_live = gg.v_out_mm;
        v_learn_live = gg.v_learn;
        latk_live = gg.lat_servo_k;
        flinch_live = gg.flinch_mm;
        ftau_live = gg.flinch_tau_ms;
        walign_live = gg.wall_align;
    }
    rfx_reset();
    ls_reset();
    fr_lap_reset();
    // A two-leg run files one report per leg: dir 'o' (out, S->G) and 'h'
    // (home, G->S) -- the GUI scoreboard groups by this character.
    lap_report_begin(two_leg ? (leg_to_goal ? 'o' : 'h') : route_run_get_dir());
    lap_report_set_run(cur_mr, leg_peak_v);
    state = RR_FLY;
    state_t0 = now;
    last_seq = sensors_latest().seq;
}

static void leg_finished(uint32_t now) {
    if (two_leg && leg_to_goal) {
        // THE GOAL STOP IS THE SEARCH'S STOP: the path brought v to zero;
        // the velocity loop keeps holding zero through the settle, the
        // report and the turn. No brake pulse, no motor STBY cycle, the
        // sensors keep streaming, the log stays open (the turn is recorded).
        {
            WallRanges w = sensors_latest();
            WallView wv = walls_evaluate(w, sensors_ms_since_packet());
            goal_front_mm = (wv.valid && wv.front) ? wv.front_mm : 0.0f;
        }
        control_move_abort();
        control_set_lateral_bias_deg(0.0f);
        control_set_reflex_bias_deg(0.0f);
        goal_stop_d_mm = control_motion().d_est_mm;
    } else {
        stop_everything();
    }
    // Only NOW does the mission move its idea of where the mouse is. An
    // aborted leg never gets here.
    pose_x = end_x;
    pose_y = end_y;
    pose_h = end_h;
    float end_resid = 0.0f;   // yaw residual vs the parked cardinal
    if (ROUTE_RUN_OUT_AND_BACK) {
        // The compiled end_h is a placeholder for loops (see compile_leg) --
        // the LIVE fused yaw knows which way the lap actually finished,
        // whatever the rig and whatever route.dir chose. Frame: yaw was
        // zeroed at launch with the nose on launch_h, so the facing is
        // launch_h rotated by the yaw's clockwise quarter-turns. A two-leg
        // run's legs are one-way compiles with a real end_h; the live yaw
        // is read the same way (the frame is the out leg's throughout) and
        // a disagreement is said out loud -- it means the leg did not fly
        // the heading sequence it was compiled to.
        float yw = control_heading().yaw_deg;
        while (yw > 180.0f) yw -= 360.0f;
        while (yw < -180.0f) yw += 360.0f;
        int q = (int)lroundf(-yw / 90.0f) & 3;   // clockwise quarters
        Heading live_h = Heading((launch_h + q) & 3);
        if (two_leg && live_h != end_h)
            printf_line("route: the leg was compiled to end facing %s but "
                        "the yaw says %s (%.1f deg) -- trusting the yaw",
                        h_name(end_h), h_name(live_h), (double)yw);
        pose_h = live_h;
        end_resid = yw - yaw_of_heading(pose_h);
        while (end_resid > 180.0f) end_resid -= 360.0f;
        while (end_resid < -180.0f) end_resid += 360.0f;
    }
    const bool out_leg_done = two_leg && leg_to_goal;   // S->G just ended
    // THE LADDER CLIMBS ON THE GOAL, not on the launch (2026-09-30): the out
    // leg of a two-leg run just stopped on the goal, or a one-leg circuit
    // through the goal just came home. An abort never gets here.
    if (cur_mr && (out_leg_done || !two_leg)) {
        run_ladder_climb(cur_mr);
        printf_line("route: MR%u reached the goal -- next wave is MR%u",
                    (unsigned)cur_mr, (unsigned)run_ladder_next());
    }
    leg_to_goal = !leg_to_goal;
    at_start_setback = false;   // it is on a cell centre now, not a wall
    ControlVelStatus vs = control_velocity_status();
    const float dmax_live = control_get_gains().duty_limit;
    {
        // Two ceilings since the duty mixer (DutyMix.h, 2026-10-01): the
        // SPEED (common mode) at dmax, each WHEEL at dwheel for steering.
        float dw = control_get_gains().wheel_duty_limit;
        float wmax = dw <= 0.0f ? dmax_live : (dw > dmax_live ? dw : dmax_live);
        bool cm_sat = vs.duty_cm_peak > dmax_live - 0.02f;
        bool wh_sat = vs.duty_peak > wmax - 0.02f;
        printf_line("route: duty peak speed %.2f of %.2f, wheel %.2f of %.2f%s%s",
                    (double)vs.duty_cm_peak, (double)dmax_live,
                    (double)vs.duty_peak, (double)wmax,
                    cm_sat ? "  << speed at the cap (straight-line limit)" : "",
                    wh_sat ? "  << a WHEEL at its limit -- steering gave way to "
                             "nothing, speed was cut to keep it" : "");
    }
    {
        // THE ABS (Control.h abs_band): how often a wheel left the ground's
        // speed and was pulled back. Zero on a clean leg; a count says
        // where the tyre limit was met (the slip test's job to say why).
        ControlMotion mo = control_motion();
        if (mo.abs_events)
            printf_line("route: ABS acted %lu time(s), %lu ticks (%.0f ms) -- a "
                        "wheel target was clamped to the slip band",
                        (unsigned long)mo.abs_events, (unsigned long)mo.abs_ticks,
                        (double)mo.abs_ticks * 2.0);
    }
    // The last corner never gets a "past its exit" tick, so resolve it here
    // or every leg would under-report by one -- the lap report included.
    if (gate.armed() && !gate.fired() && !fg_applied) {
        if (cur_skipped_lat) gstats.skipped_lat++;
        else gstats.no_edge++;
        lap_report_gate(corner_ix,
                        cur_skipped_lat ? LR_GATE_SKIP_LAT : LR_GATE_NO_EDGE,
                        0.0f, gate.last_lat_mm());
    }
    lap_report_corner_close(corner_ix,
                            corner_exit_yaw_valid ? corner_exit_yaw
                                                  : control_heading().yaw_deg,
                            0.0f, false, 0.0f);
    corner_exit_yaw_valid = false;
    gate.disarm();
    if (gstats.corners > 0) {
        printf_line("route: gate %u/%u corners anchored on a post "
                    "(%u off-centre, %u NO EDGE, %u inherited)",
                    gstats.locked, gstats.corners, gstats.skipped_lat,
                    gstats.no_edge, gstats.inherited);
        if (gstats.locked > 0)
            printf_line("   corr mean %+.1f mm, range %+.1f to %+.1f",
                        (double)(gstats.corr_sum_mm / gstats.locked),
                        (double)gstats.corr_min_mm, (double)gstats.corr_max_mm);
        else if (gstats.locked == 0 && gstats.inherited < gstats.corners)
            printf_line("   EVERY corner that could gate found NOTHING -- the "
                        "along-track position of this leg is a guess");
    }
    if (control_get_gains().front_gate > 0.0f)
        printf_line("route: fgate %u fired -- declined %u no-wall, %u short, "
                    "%u spread, %u drift, %u window",
                    (unsigned)fg_stat_fired, (unsigned)fg_stat_nosamp,
                    (unsigned)fg_stat_short, (unsigned)fg_stat_spread,
                    (unsigned)fg_stat_drift, (unsigned)fg_stat_window);
    // LEARN. Fold every LOCKED corner's correction into the per-corner
    // goff table (GoffTable.h) and save once -- stationary, like the lap
    // report. glearn=0 turns this off (race day: spend the table, don't
    // move it).
    {
        float alpha = control_get_gains().gate_learn;
        if (alpha > 0.0f && learn_n > 0) {
            for (int i = 0; i < learn_n; i++)
                goff_table_learn(learn_q[i].cx, learn_q[i].cy, learn_q[i].h,
                                 learn_q[i].corr, alpha);
            goff_table_save();
            printf_line("route: goff table updated from %u corner(s), %u "
                        "skipped (stale anchor), %d held ('goffc' lists "
                        "them)",
                        (unsigned)learn_n, (unsigned)learn_skipped,
                        goff_table_count());
        }
        learn_n = 0;
        // THE V TIMING (DiagLane.h): each V's exit residual, + outside =
        // late, so the learned timing moves EARLIER (goff_table_learn adds
        // alpha x -corr). Same gain, same per-run slew as the gates.
        if (v_learn_live >= 0.5f && alpha > 0.0f && vlearn_n > 0) {
            for (int i = 0; i < vlearn_n; i++) {
                uint8_t hk = (uint8_t)(vlearn_q[i].h | GOFF_KEY_V);
                goff_table_learn(vlearn_q[i].cx, vlearn_q[i].cy, hk,
                                 vlearn_q[i].out_mm, alpha);
                printf_line("route: V (%u,%u) exit %+.1f mm outside -> timing "
                            "%+.1f mm (next run)",
                            vlearn_q[i].cx, vlearn_q[i].cy,
                            (double)vlearn_q[i].out_mm,
                            (double)goff_table_mm(vlearn_q[i].cx, vlearn_q[i].cy, hk));
            }
            goff_table_save();
        }
        vlearn_n = 0;
        learn_skipped = 0;
    }
    printf_line("route: leg COMPLETE at (%u,%u) facing %s", pose_x, pose_y,
                pose_h == HEADING_N   ? "N"
                : pose_h == HEADING_E ? "E"
                : pose_h == HEADING_S ? "S"
                                      : "W");
    // Stamp, store, transmit -- the mouse is stopped, so this is exactly
    // "save it to FRAM and report once a lap" (bert, 2026-08-27). The GUI
    // scoreboard keys on the LAPREP/LC lines; 'lr' reprints the block.
    lap_report_set_vbat(vbat_mv());
    lap_report_finish(now - state_t0, stats.length_mm, vs.duty_peak,
                      end_resid);
    // The diagonal gate's per-post table follows the lap's LAPREP block, so
    // the GUI's lap recorder files it with the lap it belongs to.
    dg_report();
    rfx_report();
    fr_report();
    ls_report();
    if (out_leg_done) {
        // AT THE GOAL, on the cell's centre, stopped. Which way is home?
        // The home flood's first step from here, given the arrival heading
        // (straight on wins a tie: a goal block with a through-opening on
        // the way home leaves without turning at all).
        Maze m = search_maze();
        Heading want;
        goal_loop = false;
        {
            // The home leg's own limits (rvh, the registry's accelerations:
            // it does not follow the ladder), so the door the goal turn
            // faces is the one the home comparison would choose.
            ControlGains g = control_get_gains();
            float vh = cur_mr ? run_ladder_home_v(cur_mr, g) : g.route_v;
            float sl = g.route_slew < PATH_UTURN_MAX_RAIL_DPS2 ? PATH_UTURN_MAX_RAIL_DPS2
                                                               : g.route_slew;
            route_set_time_context(vh, g.route_alon, g.route_alat, g.route_brk,
                                   g.clothoid_hold > 0.5f, sl);
            route_set_diag(false);   // the home leg is orthogonal (compile_leg)
            route_set_turn_cost(g.route_turn_cost_home > 0.0f ? g.route_turn_cost_home
                                                              : g.route_turn_cost);
            if (g.goal_home >= 0.5f) {
                // ghome 1 (Control.h, Route.h THE GOAL LOOP): fly on through
                // the block and out of its second door instead of turning.
                if (route_goal_loop_viable(m, pose_x, pose_y, pose_h, vh, sl)) {
                    goal_loop = true;
                    preflight_reset();
                    printf_line("route: at the goal (%u,%u) facing %s -- GOAL LOOP "
                                "(ghome 1): the home leg flies on through the 2x2 "
                                "block and out of its other door, no turn in place, "
                                "no press", pose_x, pose_y, h_name(pose_h));
                    state = RR_SETTLE;
                    state_t0 = now;
                    return;
                }
                printf_line("route: ghome 1 but the goal block around (%u,%u) facing %s "
                            "has no second door the loop can leave by (or is not a 2x2 "
                            "of known-open cells) -- turning in place instead",
                            pose_x, pose_y, h_name(pose_h));
            }
        }
        if (!route_home_heading(m, pose_x, pose_y, pose_h, want)) {
            printf_line("route: at the goal (%u,%u) and the map has NO KNOWN "
                        "WAY HOME from it -- stopping here. Carry the mouse "
                        "back to the start cell and power cycle.", pose_x,
                        pose_y);
            state = RR_IDLE;
            return;
        }
        float yaw_now = control_heading().yaw_deg;
        while (yaw_now > 180.0f) yaw_now -= 360.0f;
        while (yaw_now < -180.0f) yaw_now += 360.0f;
        float turn = yaw_of_heading(want) - yaw_now;
        while (turn > 180.0f) turn -= 360.0f;
        while (turn < -180.0f) turn += 360.0f;
        goal_turn_h = want;
        preflight_reset();
        if (fabsf(turn) <= 1.0f) {
            printf_line("route: at the goal facing %s -- home leg leaves "
                        "straight on, no turn", h_name(want));
            pose_h = want;
            state = RR_SETTLE;
            state_t0 = now;
            return;
        }
        // Settle with the loop holding zero (the search's 150 ms), then
        // the turn starts from the poll -- RR_GOALTURN records both.
        printf_line("route: at the goal (%u,%u) facing %s, front %.0f mm -- "
                    "settle %u ms, then %+.0f deg in place to %s for the "
                    "home leg", pose_x, pose_y, h_name(pose_h),
                    (double)goal_front_mm, (unsigned)RR_GOAL_SETTLE_MS,
                    (double)turn, h_name(want));
        goal_turn_deg = turn;
        goal_turn_pending = true;
        goal_settle_t0 = now;
        state = RR_GOALTURN;
        state_t0 = now;
        return;
    }
    if (legs_left > 0) legs_left--;
    if (legs_left > 0) {
        printf_line("  %u more leg(s) this wave", legs_left);
        preflight_reset();
        state = RR_SETTLE;
        state_t0 = now;
        return;
    }
    if (ROUTE_RUN_OUT_AND_BACK && pose_x == 0 && pose_y == 0) {
        // THE LAP ENDS READY FOR THE NEXT ONE (bert, 2026-08-28: "repeated
        // laps untouched by human hands"). On CAL-6 the loop lands back on
        // the start cell facing WEST; the launch convention is NORTH; so the
        // 90 the next lap needs is flown here, in place, as part of the lap
        // end -- not delegated to a hand. The pose stays exactly what it is
        // (cell CENTRE, not the rear-wall setback a hand placement gives),
        // which is why at_start_setback stays false: the next compile takes
        // no lead-in, because none is true.
        // Turn to the NEXT lap's launch heading (route.dir's choice), so a
        // CW session parks facing north and a CCW one facing east. The turn
        // amount comes from the LIVE yaw, not the quantized pose: the lap's
        // accumulated residual (half a degree of corner debt) is trued up
        // here for free, so every lap launches from the same datum.
        Heading want = dir_heading();
        datum_begin(want, now);
        return;
    }
    state = RR_IDLE;
    if (ROUTE_RUN_OUT_AND_BACK) {
        // A loop that ends anywhere OTHER than the start cell keeps the old
        // contract: pose reset to the convention, human told what to make
        // true.
        pose_x = 0;
        pose_y = 0;
        pose_h = HEADING_N;
        at_start_setback = true;
        launch_by_press = false;   // a hand must fix this one
        printf_line("-- 'y' dumps the flight log. PUT THE MOUSE BACK on the "
                    "start cell facing NORTH, then wave for another lap --");
    } else {
        printf_line("-- 'y' dumps the flight log. Wave to fly %s --",
                    leg_name_long());
    }
    route_run_arm();
}

// ---------------------------------------------------------------- arming
bool route_run_arm_after_search(Heading arrived) {
    if (state != RR_IDLE) return false;
    if (!route_run_available()) return false;
    // The route's own idea of where it is, as a lap end leaves it: the
    // start cell's centre, facing the way the search arrived. The search's
    // yaw frame has north at 0 (Explore.h), which is the frame
    // yaw_of_heading expects with launch_h north.
    pose_x = 0;
    pose_y = 0;
    pose_h = arrived;
    launch_h = HEADING_N;
    leg_to_goal = true;
    legs_left = 0;
    two_leg = false;
    goal_datum = false;
    at_start_setback = false;
    launch_by_press = false;
    printf_line("route: search complete -- squaring up on the start cell's walls "
                "for MR2 (no power cycle needed)");
    datum_begin(dir_heading(), millis());
    return true;
}

void route_run_arm() {
    if (state != RR_IDLE) return;
    if (!route_run_available()) {
        printf_line("route: no proven route in the stored map -- search "
                    "first ('h' arms a search)");
        return;
    }
    // ONE MISSION OWNS THE WAVE -- both watch the same front beam, and arming
    // both would make a wave ambiguous. 'h' does the reverse.
    search_disarm_handstart();
    sensors_free_run_interlaced();
    hand_run = 0;
    hand_active = false;
    hand_cal_started = false;
    fly_live_l = fly_live_r = 0;   // a hand start is judged on fresh evidence
    goal_loop = false;
    last_seq = sensors_latest().seq;
    arm_t0 = millis();
    arm_silent_said = false;
    {
        // The sides the map says the launch cell has walls on (StartGate.h).
        StartGate &gate = start_gate_shared();
        Maze m = search_maze();
        Heading hl = heading_left(pose_h), hr = heading_right(pose_h);
        bool carry = gate.carry_next_reset;
        gate.reset(arm_t0);
        if (!carry) {
            gate.need_l = m.wall_known(pose_x, pose_y, hl) && m.wall_present(pose_x, pose_y, hl);
            gate.need_r = m.wall_known(pose_x, pose_y, hr) && m.wall_present(pose_x, pose_y, hr);
        }
    }
    state = RR_ARMED;
    if (!fram_ready())
        // Every trim, LUT and learned offset is a compiled default right
        // now -- the mouse WILL fly, and it will fly badly (D9, 2026-09-15:
        // a fram8 build on the 4 Mbit chip, first post at the V).
        printf_line("route: WARNING -- FRAM NOT LIVE (%s): flying on COMPILED "
                    "DEFAULTS, no sensor LUTs, nothing will be saved",
                    fram_status_text());
    else if (!bench_params_from_fram())
        // FRAM answers but the params block did not load (blank chip after
        // a swap, a failed CRC): the same compiled defaults, said as loudly.
        printf_line("route: WARNING -- REGISTRY IS COMPILED DEFAULTS (kyaw %.1f, "
                    "track %.1f): the params block did not load from FRAM. "
                    "Restore the .fram.json backup or re-enter and 'w'",
                    (double)control_get_gains().kyaw,
                    (double)control_get_gains().track_mm);
    printf_line("route ARMED: wave to fly %s at %.0f mm/s%s",
                leg_name_long(),
                (double)(run_ladder_next()
                             ? run_ladder_out_v(run_ladder_next(), control_get_gains())
                             : control_get_gains().route_v),
                ROUTE_RUN_OUT_AND_BACK && !two_leg
                    ? " (two legs with a turn at the goal if the map has no "
                      "loop)"
                    : "");
    route_run_report();
}

void route_run_disarm() {
    if (state == RR_FAULT || state == RR_RECOVER) { route_run_abort("disarm"); return; }
    if (state == RR_ARMED || state == RR_HAND_CLEAR || state == RR_CAL) {
        sensors_stop();
        state = RR_IDLE;
        printf_line("route disarmed");
    }
}

bool route_run_armed() {
    return state == RR_ARMED || state == RR_HAND_CLEAR || state == RR_CAL;
}
bool route_run_running() { return state != RR_IDLE; }

// ---------------------------------------------------------------- report
void route_run_report() {
    if (!route_run_available()) {
        printf_line("route: no proven start->goal route in the stored map");
        return;
    }
    if (run_ladder_next())
        printf_line("route: run ladder ON -- next wave is MR%u: out %.0f, "
                    "home %.0f ('mr=0' flies rv)", (unsigned)run_ladder_next(),
                    (double)run_ladder_out_v(run_ladder_next(), control_get_gains()),
                    (double)run_ladder_home_v(run_ladder_next(), control_get_gains()));
    printf_line("route: next run %s from (%u,%u) facing %s, v%.0f R%u",
                leg_name_long(),
                pose_x, pose_y,
                pose_h == HEADING_N   ? "N"
                : pose_h == HEADING_E ? "E"
                : pose_h == HEADING_S ? "S"
                                      : "W",
                (double)control_get_gains().route_v, ROUTE_RADII[0]);
}

// Start a run WITHOUT the hand wave -- the GUI's "Start lap" button, and the
// second half of "repeated laps untouched by human hands". Valid only while
// ARMED (the same guard the wave gets); it enters the exact pipeline the
// wave enters after the hand clears -- the 700 ms settle, the ~3 s IMU
// recalibration with the mouse provably stationary, the tyre settle -- so a
// GUI-started lap is calibrated identically to a hand-started one.
void route_run_go() {
    if (state == RR_IDLE) route_run_arm();   // convenience: go implies arm
    if (state != RR_ARMED) {
        printf_line("route.go: not armed (state busy) -- 'route.stop' first "
                    "if a run is stuck");
        return;
    }
    hand_active = false;
    hand_run = 0;
    hand_cal_started = false;
    preflight_reset();
    state = RR_CAL;
    state_t0 = millis();
    printf_line("route.go: launching -- keep clear, IMU calibrates ~3 s "
                "then it flies");
}

RouteGateStats route_run_gate_stats() { return gstats; }

// route.dir=n|e|a -- see the route_dir comment at the top. Setting it also
// makes the PARKED mouse match the new launch heading, because "the GUI
// changes direction between laps" is the whole use case: hand-placed at the
// setback, the human is told which way to face it; parked at the cell
// centre after a lap, the mouse re-orients itself with the same in-place
// turn a lap end uses.
void route_run_set_dir(char c) {
    switch (c) {
        case 'n': case 'N': route_dir = HEADING_N; break;
        case 'e': case 'E': route_dir = HEADING_E; break;
        case 'a': case 'A': route_dir = -1; break;
        default:
            printf_line("route.dir: n (launch north), e (launch east) or "
                        "a (auto)");
            return;
    }
    printf_line("route.dir=%c -- %s", route_run_get_dir(),
                route_dir < 0
                    ? "auto (tie-break; CAL-6 flies CW, launch north)"
                    : (route_dir == HEADING_N
                           ? "launch NORTH, west arm out (CAL-6: CW, "
                             "right-handers)"
                           : "launch EAST, east arm out (CAL-6: CCW, "
                             "left-handers)"));
    Heading want = dir_heading();
    if (pose_x != 0 || pose_y != 0 || pose_h == want) return;
    if (at_start_setback && launch_by_press &&
        (state == RR_IDLE || state == RR_ARMED)) {
        // Parked by the machine's own press: it knows where it is, so the
        // two-wall datum re-parks it for the new heading, hands-free.
        datum_begin(want, millis());
        return;
    }
    if (at_start_setback) {
        // The mouse is wherever a hand put it -- tell the hand.
        pose_h = want;
        printf_line("  place the mouse on the start cell facing %s, back "
                    "to the %s wall", h_name(want),
                    want == HEADING_E ? "WEST" : "SOUTH");
        return;
    }
    if (state == RR_IDLE || state == RR_ARMED) {
        // Parked on the start-cell centre after a lap: re-orient in place.
        // The yaw frame is still the last lap's (nothing re-zeroes it until
        // the next launch), so yaw_of_heading() is valid.
        float yaw_now = control_heading().yaw_deg;
        while (yaw_now > 180.0f) yaw_now -= 360.0f;
        while (yaw_now < -180.0f) yaw_now += 360.0f;
        float turn = yaw_of_heading(want) - yaw_now;
        while (turn > 180.0f) turn -= 360.0f;
        while (turn < -180.0f) turn += 360.0f;
        control_velocity_enable(true);
        if (control_turn_start(turn, 360.0f, 2000.0f)) {
            printf_line("  turning %.0f deg to %s -- keep clear",
                        (double)turn, h_name(want));
            lapturn_target = want;
            lapturn_backup = false;   // parked by a hand; nothing to re-datum
            state = RR_LAPTURN;
            state_t0 = millis();
        } else {
            control_velocity_enable(false);
            pose_h = want;
            printf_line("  turn refused -- place the mouse facing %s by "
                        "hand", h_name(want));
        }
        return;
    }
    printf_line("  (a run is in progress -- the new direction applies from "
                "the next lap that starts at the start cell)");
}

// THE TURN AND THE PRESS, RECORDED (2026-09-15). The goal turn's sideways
// drift could only be described, never measured: nothing logged between
// the out leg's last record and the home leg's first. One record per
// sensor packet while turning or pressing: flags bit4, s = odometry
// distance since the stop (the translation the turn should not have),
// v = the commanded linear velocity, yaw and omega as always, and the two
// wheel velocities in the observer fields the route never uses
// (obs_along = left, obs_cross = right, mm/s) -- their SUM is the drift's
// engine, their DIFFERENCE the turn.
static uint32_t turn_log_seq = 0;
static void log_turn_packet(uint32_t now) {
    WallRanges w = sensors_latest();
    if (w.seq == turn_log_seq) return;
    turn_log_seq = w.seq;
    ControlMotion mo = control_motion();
    ControlOdometry od = control_odometry();
    ControlHeading hd = control_heading();
    ControlMoveStatus ms = control_move_status();
    FlogRec r;
    memset(&r, 0, sizeof r);
    r.t_ms = (uint16_t)(now - state_t0);
    // Translation since the stop (the press zeroes the odometry itself).
    r.s_mm = (int16_t)(state == RR_BACKUP ? mo.d_est_mm
                                          : mo.d_est_mm - goal_stop_d_mm);
    r.v_mm_s = (int16_t)ms.v_cmd;
    r.yaw_cdeg = (int16_t)(hd.yaw_deg * 10.0f);
    r.yaw_err_cdeg = (int16_t)(ms.yaw_err_deg * 10.0f);
    r.ay_mm_s2 = (int16_t)mo.a_lat_mm_s2;
    r.ax_mm_s2 = (int16_t)mo.a_fwd_mm_s2;
    r.flags = (uint8_t)(0x10 | (state == RR_BACKUP ? 0x20 : 0));
    r.left_mm = w.left_mm;
    r.right_mm = w.right_mm;
    r.front_left_mm = w.front_left_mm;
    r.front_right_mm = w.front_right_mm;
    r.wheel_fused_mm = (int16_t)od.dist_mm;
    r.omega_dps = (int16_t)ms.yaw_rate_dps;
    r.obs_along_mm = (int16_t)od.vel_left_mm_s;
    r.obs_cross_mm = (int16_t)od.vel_right_mm_s;
    flog_record(r);
}

// ---------------------------------------------------------------- poll
void route_run_poll() {
    if (state == RR_IDLE) return;
    uint32_t now = millis();
    sensors_poll();

    switch (state) {
        case RR_ARMED: {
            StartGate &gate = start_gate_shared();
            gate.tick(sensors_ms_since_packet(), now);
            if (!gate.ready && !hand_active) {
                // THE START GATE (StartGate.h): LEDs one then the other until
                // both side walls read steady -- in the hand, or a sensor
                // that came up wrong after a quick power cycle.
                bool p1 = StartGate::led1_phase(now);
                hal_set_led1(p1);
                hal_set_led2(!p1);
                if (gate.time_to_say(now))
                    printf_line("start gate: side walls not both seen (L %u R %u%s) -- "
                                "LEDs alternate until the mouse sits in the start cell "
                                "reading both. Still alternating on the floor = the "
                                "sensor fault: power off, wait a few seconds, power on.",
                                gate.last_l, gate.last_r,
                                sensors_ms_since_packet() > START_GATE_STALE_MS
                                    ? ", NO PACKETS" : "");
            } else if (!hand_active && now - led_last >= 250) {
                led_last = now;
                led_phase = !led_phase;
                // Both LEDs together, slowly: distinct from the sweep's
                // alternating pattern, because at the maze the LEDs are the
                // only thing that says which mission a wave will start.
                hal_set_led1(led_phase);
                hal_set_led2(led_phase);
            }
            WallRanges w = sensors_latest();
            // A wave nobody sees (2026-09-27, 11:50: armed twice, no "hand
            // seen" for a minute each time; a longer power-off fixed it --
            // the sub-MCU comes up mute after a quick power cycle, and the
            // pre-flight only says so once a hand has already been waved).
            // Say it here, once, so the mouse does not just sit there.
            if (!arm_silent_said && now - arm_t0 > 3000 &&
                sensors_ms_since_packet() > 2000) {
                arm_silent_said = true;
                printf_line("route: armed but NO SENSOR PACKETS for %lu ms -- the wave "
                            "cannot be seen. Power off, wait a few seconds, power on.",
                            (unsigned long)sensors_ms_since_packet());
            }
            if (w.seq == last_seq) return;
            last_seq = w.seq;
            WallView wv = walls_evaluate(w, sensors_ms_since_packet());
            if (gate.packet(w.left_mm, w.right_mm, now))
                printf_line("start gate: side walls L %u R %u -- ready for the wave",
                            w.left_mm, w.right_mm);
            if (!gate.ready && !hand_active) {
                hand_run = 0;          // no gesture begins off the floor
                return;
            }
            if (hand_active && w.left_mm == 255 && w.right_mm == 255) {
                // Lifted mid-gesture: drop it (a hand in front may nudge one
                // side beam; both gone is a lift).
                hand_active = false;
                hand_run = 0;
                printf_line("route: wave cancelled -- both side walls gone (picked up?)");
                return;
            }
            float lim = hand_active ? RR_HAND_CLEAR_MM : RR_HAND_START_MM;
            if (wv.valid && wv.front && wv.front_mm < lim) {
                if (hand_run < RR_HAND_TRIGGER_N) hand_run++;
                if (hand_run >= RR_HAND_TRIGGER_N && !hand_active) {
                    hand_active = true;
                    hand_t0 = now;
                    hand_next_tick = 1;
                    printf_line("route: hand seen -- remove to fly, or hold "
                                "%lu s to WIPE the map and search instead",
                                (unsigned long)(RR_HAND_WIPE_MS / 1000));
                }
                if (hand_active) hand_seen_ms = now;
            } else if (!hand_active) {
                hand_run = 0;
            }
            if (hand_active) {
                uint32_t held = now - hand_t0;
                if (hand_next_tick * 1000 <= held &&
                    hand_next_tick * 1000 < RR_HAND_WIPE_MS) {
                    // Count the hold out loud and on the LEDs, so it is
                    // visible at the maze while it is happening.
                    led_flash(1);
                    led_flash(2);
                    printf_line("   holding %lu s -- %lu more to WIPE",
                                (unsigned long)hand_next_tick,
                                (unsigned long)(RR_HAND_WIPE_MS / 1000 -
                                                hand_next_tick));
                    hand_next_tick++;
                }
                if (held >= RR_HAND_WIPE_MS) {
                    // New unknown maze: map gone, learned offsets gone, and
                    // the wave belongs to the search from here. The hand is
                    // still there, so the search's own hand start picks the
                    // gesture up and launches when it is removed.
                    search_map_wipe();
                    goff_table_clear();
                    goff_table_save();
                    hand_active = false;
                    hand_run = 0;
                    led_dark();
                    state = RR_IDLE;
                    printf_line("route: MAP WIPED, goff cleared -- the wave "
                                "is the SEARCH's now: remove your hand to "
                                "launch MR1");
                    start_gate_shared().carry_next_reset = true;   // same mouse, same place
                    search_arm_handstart();
                    return;
                }
            }
            if (hand_active && now - hand_seen_ms >= 400) {
                hand_active = false;
                hand_run = 0;
                state = RR_HAND_CLEAR;
            }
            return;
        }

        case RR_HAND_CLEAR: {
            WallRanges w = sensors_latest();
            if (w.seq == last_seq) return;
            last_seq = w.seq;
            WallView wv = walls_evaluate(w, sensors_ms_since_packet());
            bool clear = !wv.front || wv.front_mm > RR_HAND_CLEAR_MM;
            if (wv.valid && clear) {
                if (++hand_run >= RR_HAND_CLEAR_N) {
                    printf_line("route: hand clear -- keep the mouse still");
                    hand_run = 0;
                    hand_cal_started = false;
                    preflight_reset();
                    state = RR_CAL;
                    state_t0 = now;
                }
            } else {
                hand_run = 0;
            }
            return;
        }

        case RR_CAL:
            {
                WallRanges w = sensors_latest();
                if (w.seq != last_seq) { last_seq = w.seq; preflight_sample(w); }
            }
            // The IMU zero is taken HERE, with the mouse provably stationary
            // on the start cell, and nowhere else in the mission.
            if (!hand_cal_started) {
                if (now - state_t0 < RR_HAND_SETTLE_MS) return;
                control_recalibrate();
                hand_cal_started = true;
                printf_line("route: IMU calibrating (~3 s, stationary)...");
                return;
            }
            if (!control_heading().ready) return;
            legs_left = ROUTE_RUN_LEGS_PER_WAVE;
            // The run's rung is owed, not yet taken: leg_launch takes it
            // once pre-flight has passed (see rung_pending). Until then
            // cur_mr says what the wave WOULD fly, for the compile and
            // the prints, without touching the FRAM counter.
            rung_pending = true;
            cur_mr = run_ladder_next();
            if (ROUTE_RUN_OUT_AND_BACK) {
                // Every wave starts a fresh run from the start cell: the
                // circuit is tried first, and the two-leg fallback is
                // decided by THIS compile, not remembered from the last.
                two_leg = false;
                leg_to_goal = true;
                goal_datum = false;
            }
            state = RR_SETTLE;
            state_t0 = now;
            return;

        case RR_SETTLE:
            {
                WallRanges w = sensors_latest();
                if (w.seq != last_seq) { last_seq = w.seq; preflight_sample(w); }
            }
            if (now - state_t0 < RR_TYRE_SETTLE_MS) return;
            leg_launch(now);
            return;

        case RR_FAULT: {
            // 100 ms on, every 2 s, both LEDs; then the recovery window --
            // the next hand start runs the pre-flight again, and a beam
            // still dead is refused again (the start gate will not even
            // offer the wave while a mapped wall reads 255).
            if (now - state_t0 >= RR_RECOVER_MS) {
                printf_line("route: sensor fault -- trying again: the next hand "
                            "start re-runs the pre-flight");
                recover_begin(now);
                return;
            }
            uint32_t ph = (now - state_t0) % RR_FAULT_PERIOD_MS;
            bool on = ph < RR_FAULT_ON_MS;
            if (on != led_phase) {
                led_phase = on;
                hal_set_led1(on);
                hal_set_led2(on);
            }
            return;
        }

        case RR_RECOVER: {
            if (now - state_t0 < RR_RECOVER_MS) {
                // Both LEDs together, 1 s on / 1 s off: "recovering, do
                // not wave yet" -- slower than the armed 250 ms pulse.
                bool on = ((now - state_t0) / RR_RECOVER_BLINK_MS) % 2 == 0;
                if (on != led_phase) {
                    led_phase = on;
                    hal_set_led1(on);
                    hal_set_led2(on);
                }
                return;
            }
            // The start-cell convention, as a lap end that finished
            // elsewhere leaves it (route_run lap end): a hand placed it.
            state = RR_IDLE;
            pose_x = 0;
            pose_y = 0;
            pose_h = HEADING_N;
            launch_h = HEADING_N;
            leg_to_goal = true;
            legs_left = 0;
            two_leg = false;
            goal_datum = false;
            goal_turn_pending = false;
            rung_pending = false;
            at_start_setback = true;
            launch_by_press = false;   // a hand must fix this one
            printf_line("route: recovery window over -- arming for a hand start "
                        "from the start cell facing NORTH%s",
                        run_ladder_next() ? " (same rung)" : "");
            route_run_arm();
            return;
        }

        case RR_LAPTURN: {
            ControlMoveStatus ms = control_move_status();
            if (ms.active && now - state_t0 < 4000) return;
            // Done, or the settle gate hung (a known in-place-turn failure
            // mode) -- either way the wheels are stopped and the next lap's
            // launch reads the LIVE yaw as its datum, so a degree or two of
            // turn residue drains as a heading transient on the first
            // straight rather than poisoning the lap.
            if (!ms.active && !ms.done)
                printf_line("route: lap turn did not settle cleanly -- "
                            "launch datum comes from the live yaw, so this "
                            "costs nothing");
            control_move_abort();
            pose_h = lapturn_target;
            // THE LAP END IS NOT DONE YET. The turn put the mouse on the
            // launch heading; the press puts it on the launch DATUM.
            if (lapturn_backup) {
                lapturn_backup = false;
                if (start_rear_wall_press(now)) return;
            }
            control_velocity_enable(false);   // enabled just for this turn
            state = RR_IDLE;
            printf_line("-- lap ready facing %s: fetch the log, tune, then "
                        "'route.go' or wave. Do NOT reposition the mouse: "
                        "it knows where it is. --", h_name(pose_h));
            route_run_arm();
            return;
        }

        case RR_GOALTURN: {
            log_turn_packet(now);
            if (goal_turn_pending) {
                if (now - goal_settle_t0 < RR_GOAL_SETTLE_MS) return;
                goal_turn_pending = false;
                ControlGains tg = control_get_gains();
                if (!control_turn_start(goal_turn_deg, tg.turn_omega_dps,
                                        tg.turn_alpha_dps2)) {
                    control_velocity_enable(false);
                    printf_line("route: goal turn REFUSED -- stopping at the "
                                "goal. Home leg NOT flown.");
                    stop_everything();
                    recover_begin(now);
                    return;
                }
                state_t0 = now;
                return;
            }
            ControlMoveStatus ms = control_move_status();
            if (ms.active && now - state_t0 < RR_GOAL_TURN_MAX_MS) return;
            // Done, or the settle gate hung (the in-place turn's known
            // failure mode): either way the wheels are stopped, and the
            // home leg builds its path on the corridor from the LIVE yaw,
            // so a degree or two of residue drains on the first straight.
            if (!ms.active && !ms.done)
                printf_line("route: goal turn did not settle cleanly -- the "
                            "home leg launches on the live yaw, so this "
                            "costs nothing");
            control_move_abort();   // the loop stays enabled, holding zero
            pose_h = goal_turn_h;
            {
                float yw = control_heading().yaw_deg;
                float resid = yw - yaw_of_heading(pose_h);
                while (resid > 180.0f) resid -= 360.0f;
                while (resid < -180.0f) resid += 360.0f;
                printf_line("route: goal turn done, facing %s (%+.1f deg off "
                            "the corridor) -- home leg in %u ms",
                            h_name(pose_h), (double)resid,
                            (unsigned)RR_TYRE_SETTLE_MS);
            }
            preflight_reset();
            {
                // The goal datum: the first KNOWN wall behind the new
                // heading, through known-open cells, within
                // RR_GOAL_DATUM_MAX_CELLS (RR_GOAL_DATUM_MAX_CELLS note).
                const Maze &m = search_maze();
                Heading behind = heading_back(pose_h);
                int cells = 0;
                {
                    int cx = pose_x, cy = pose_y;
                    for (int k = 1; k <= RR_GOAL_DATUM_MAX_CELLS; k++) {
                        if (!m.wall_known(cx, cy, behind)) break;
                        if (m.wall_present(cx, cy, behind)) { cells = k; break; }
                        cx += heading_dx(behind); cy += heading_dy(behind);
                        if (cx < 0 || cy < 0 || cx >= MAZE_SIZE || cy >= MAZE_SIZE) break;
                    }
                }
                if (cells > 0) {
                    goal_datum = true;
                    goal_datum_cells = cells;
                    if (start_rear_wall_press(now, (float)(cells - 1) * ROUTE_CELL_MM)) {
                        if (cells > 1)
                            printf_line("route: goal datum -- the known wall behind is "
                                        "%d cells back: pressing up to %.0f mm",
                                        cells, (double)backup_mm);
                        return;
                    }
                    goal_datum = false;
                    control_velocity_enable(false);
                }
                else
                    printf_line("route: no known wall behind at the goal within "
                                "%d cells -- home leg launches on dead reckoning",
                                RR_GOAL_DATUM_MAX_CELLS);
            }
            state = RR_SETTLE;
            state_t0 = now;
            return;
        }

        case RR_BACKUP: {
            if (goal_datum) log_turn_packet(now);
            ControlMoveStatus ms = control_move_status();
            ControlMotion mo = control_motion();
            float went = fabsf(mo.d_est_mm);
            // CONTACT = the profile still asks for speed and the mouse has
            // stopped anyway. Held for RR_BACKUP_STALL_MS so a stiction
            // hesitation on the way back is not mistaken for the wall.
            bool touched = false;
            if (went > RR_BACKUP_ARM_MM &&
                fabsf(mo.v_est_mm_s) < RR_BACKUP_STALL_MM_S) {
                if (backup_stall_t0 == 0) backup_stall_t0 = now;
                else if (now - backup_stall_t0 >= RR_BACKUP_STALL_MS)
                    touched = true;
            } else {
                backup_stall_t0 = 0;
            }
            // The time allowance grows with the travel (a far goal datum).
            uint32_t max_ms = RR_BACKUP_MAX_MS +
                              (uint32_t)((backup_mm - RR_BACKUP_MM) / RR_BACKUP_V * 1000.0f);
            if (!touched && ms.active && now - state_t0 < max_ms)
                return;
            control_move_abort();
            control_motion_zero();
            if (datum_phase == 1) {
                // Cross wall done (or not): off it to the centreline, then
                // the launch axis. A missing cross wall is not fatal -- the
                // lateral just goes un-datumed as it always used to.
                if (touched) {
                    float fwd = 0.5f * ROUTE_CELL_MM - search_start_y_mm();
                    if (control_move_start(fwd, RR_BACKUP_V, RR_BACKUP_ACC)) {
                        printf_line("route: datum 1/3 -- %s wall at %.0f mm, "
                                    "forward %.0f to the centreline",
                                    pose_h == HEADING_N ? "SOUTH" : "WEST",
                                    (double)went, (double)fwd);
                        datum_phase = 2;
                        state = RR_DATUM_FWD;
                        state_t0 = now;
                        return;
                    }
                } else {
                    printf_line("route: datum 1/3 -- no %s wall inside %.0f "
                                "mm; lateral stays un-datumed this lap",
                                pose_h == HEADING_N ? "SOUTH" : "WEST",
                                (double)went);
                }
                datum_turn_to(datum_want, 3, now);
                return;
            }
            datum_phase = 0;
            control_velocity_enable(false);
            if (goal_datum) {
                goal_datum = false;
                if (touched) {
                    // Same contract as the start cell: back panel on the
                    // wall, so the home leg compiles with the lead-in. A far
                    // wall: the pose is now the cell against it.
                    if (goal_datum_cells > 1) {
                        Heading behind = heading_back(pose_h);
                        pose_x = (uint8_t)((int)pose_x + heading_dx(behind) * (goal_datum_cells - 1));
                        pose_y = (uint8_t)((int)pose_y + heading_dy(behind) * (goal_datum_cells - 1));
                        printf_line("route: goal datum -- pressed %d cells back: the home "
                                    "leg starts from (%u,%u)", goal_datum_cells, pose_x, pose_y);
                    }
                    at_start_setback = true;
                    launch_by_press = true;
                    // Cross-check against the front beam read before the
                    // turn: the panel should have met the wall after
                    // (front - starty) mm. A stall much earlier is the body
                    // caught on a side wall or post after a drifting turn,
                    // and the datum is then wrong by the difference.
                    float expect = (goal_front_mm > 0.0f && goal_datum_cells == 1)
                                       ? goal_front_mm - search_start_y_mm()
                                       : -1.0f;
                    printf_line("route: goal datum -- wall behind at %.0f mm "
                                "(front beam said %.0f); home leg launches "
                                "from the setback (lead-in %.0f)",
                                (double)went, (double)expect,
                                (double)(0.5f * ROUTE_CELL_MM - search_start_y_mm()));
                    if (expect > 0.0f && fabsf(went - expect) > RR_GOAL_PRESS_TOL_MM)
                        printf_line("route: WARNING -- press stalled %+.0f mm "
                                    "from where the front beam put the wall: "
                                    "the datum may be off by that much",
                                    (double)(went - expect));
                    preflight_reset();
                    state = RR_SETTLE;
                    state_t0 = now;
                    return;
                }
                // The map said wall; the wheels found none in RR_BACKUP_MM.
                // The mouse is not where it believes it is -- and it has
                // just reversed up to 130 mm from wherever that was.
                printf_line("route: goal datum -- NO WALL behind inside %.0f "
                            "mm at (%u,%u): the mouse is not where it thinks. "
                            "Home leg NOT flown.", (double)went, pose_x, pose_y);
                stop_everything();
                recover_begin(now);
                return;
            }
            state = RR_IDLE;
            if (touched) {
                // The back panel is against the wall, which is where a hand
                // places it -- so the next compile takes the same lead-in
                // lap 1 takes, and both start populations become one.
                at_start_setback = true;
                launch_by_press = true;
                printf_line("route: rear wall at %.0f mm -- launch datum is "
                            "the WALL, not the odometry (nominal 39)",
                            (double)went);
                printf_line("-- lap ready facing %s: fetch the log, tune, "
                            "then 'route.go' or wave. Do NOT reposition the "
                            "mouse: it knows where it is. --",
                            h_name(pose_h));
            } else {
                // The maze boundary guarantees a wall here. Not finding one
                // means the mouse is not where it believes it is, and a
                // datum it never touched is worse than no datum at all.
                pose_x = 0;
                pose_y = 0;
                at_start_setback = true;
                launch_by_press = false;   // a hand must fix this one
                printf_line("route: NO REAR WALL inside %.0f mm -- the mouse "
                            "is not on the start cell. PUT IT BACK against "
                            "the rear wall facing %s before the next lap.",
                            (double)went, h_name(pose_h));
            }
            route_run_arm();
            return;
        }

        case RR_DATUM_FWD: {
            ControlMoveStatus ms = control_move_status();
            if (ms.active && now - state_t0 < 2000) return;
            control_move_abort();
            control_motion_zero();
            datum_turn_to(datum_want, 3, now);
            return;
        }

        case RR_IDLE:    // returned above
        case RR_FLY:
            break;   // the body below
    }

    // ---- flying -----------------------------------------------------------
    led_flash_service(now);
    ControlPathStatus ps = control_path_status();
    seg_tick(ps.v_mm_s);

    if (!ps.active) {
        if (ps.done) {
            leg_finished(now);
        } else {
            ControlFrontGuard fg = control_front_guard();
            route_run_abort(fg.cap_ticks > 100
                            ? "the front governor held the mouse off a wall the path drove at"
                            : "path ended without completing");
        }
        return;
    }
    if (now - state_t0 > RR_LEG_TIMEOUT_MS) {
        route_run_abort("leg timeout");
        return;
    }

    // SILENT-LINK GUARD -- must sit ABOVE the seq gate: with the sub-MCU
    // quiet, seq never changes and everything below (including every other
    // abort) is starved while the path keeps flying (run 208's worse twin).
    if (sensors_ms_since_packet() > RR_SENS_SILENT_MS) {
        route_run_abort("sensor link silent -- no packets from the sub-MCU");
        return;
    }

    WallRanges w = sensors_latest();
    if (w.seq == last_seq) return;
    last_seq = w.seq;
    if (w.left_mm > 0 && w.left_mm < 255 && fly_live_l < 30000) fly_live_l++;
    if (w.right_mm > 0 && w.right_mm < 255 && fly_live_r < 30000) fly_live_r++;
    WallView wv = walls_evaluate(w, sensors_ms_since_packet());

    // BLIND-FLIGHT GUARD (run 208: lasers died at launch, the sub streamed
    // all-255 packets, and the mouse flew 2.6 m open loop into a wall). A
    // live beam is 1..254 -- 255 is out of range, 0 an IC error.
    {
        auto live = [](uint8_t b) { return b > 0 && b < 255; };
        if (live(w.left_mm) || live(w.right_mm) || live(w.front_left_mm) ||
            live(w.front_right_mm))
            last_sighted_s = ps.s_mm;
        else if (ps.s_mm - last_sighted_s > RR_BLIND_ABORT_MM) {
            route_run_abort("wall sensors blind -- every beam invalid for "
                            "600 mm (lasers dead?)");
            return;
        }
    }

    // IN A JUNCTION? From the entry tangent of the corner being approached to
    // SWEEP_JUNCTION_EXIT_MM past its exit. Inside that stretch there is no
    // corridor for the wall model to describe, so the model-based rules stand
    // down and the raw-beam guard takes over. The corner index is found by
    // scanning the path's own corner list rather than tracked in a counter --
    // a counter is one more thing that can be left pointing at the wrong
    // corner when something goes wrong mid-leg.
    int nc = path.corner_count();

    // WALK THE GATE FORWARD. corner_ix is the corner being worked on; once
    // the mouse is past its exit the corner is closed, its outcome counted,
    // and the gate disarmed so the next one may arm. The lap report closes
    // its corner at the same boundary, from the same walk -- one position
    // tracker, two consumers.
    if (corner_ix < nc && !corner_exit_yaw_valid &&
        ps.s_mm >= path.corner_exit_s_mm(corner_ix)) {
        corner_exit_yaw = control_heading().yaw_deg;
        corner_exit_yaw_valid = true;
    }
    if (corner_ix < nc &&
        ps.s_mm > path.corner_exit_s_mm(corner_ix) + SWEEP_JUNCTION_EXIT_MM) {
        // A corner the FRONT gate anchored but whose side edge never
        // dropped is FRONT, not NO_EDGE -- the front already reported
        // it. In OBSERVE nothing was applied, so NO_EDGE stands.
        if (gate.armed() && !gate.fired() && !fg_applied) {
            if (cur_skipped_lat) gstats.skipped_lat++;
            else gstats.no_edge++;
            lap_report_gate(corner_ix,
                            cur_skipped_lat ? LR_GATE_SKIP_LAT
                                            : LR_GATE_NO_EDGE,
                            0.0f, gate.last_lat_mm());
            // This corner set no anchor, so the NEXT corner's corr will
            // carry this corner's error too -- it may fly on it, but it
            // may not teach on it (see anchor_fresh).
            anchor_fresh = false;
        } else {
            anchor_fresh = true;
        }
        // A 45-in anchors nothing. A 45-out is placed by the diagonal's
        // posts: if the gate applied at least one, the anchor is fresh and
        // the exit record carries the last correction and the lateral the
        // posts measured; otherwise the next corner flies the diagonal's
        // drift and may not teach on it.
        if (is_diag_exit(corner_ix)) {
            lap_report_gate(corner_ix, LR_GATE_DIAG, dg_last_corr,
                            dg_lat_valid ? dg_lat_est : 127.0f);
            anchor_fresh = diag_gate_live >= 1.0f && dg_locked > 0;
        } else if (is_diag_turn(corner_ix) && !is_135_in(corner_ix)) {
            anchor_fresh = false;   // (a gated 135-in keeps the 90's rule above)
        }
        lap_report_corner_close(corner_ix,
                                corner_exit_yaw_valid ? corner_exit_yaw
                                                      : control_heading().yaw_deg,
                                ps.yaw_err_deg, lr_lat_valid, lr_lat_mm);
        corner_exit_yaw_valid = false;
        cur_skipped_lat = false;
        if (fgate_live > 0.0f) {
            if (fg_fired) fg_stat_fired++;
            else if (fg_n == 0) fg_stat_nosamp++;
            else if (fg_last_fail == 1) fg_stat_spread++;
            else if (fg_last_fail == 2) fg_stat_drift++;
            else if (fg_last_fail == 3) fg_stat_window++;
            else fg_stat_short++;   // never reached count+span to be judged
        }
        fg_fired = false;
        fg_applied = false;
        fg_last_range = 0.0f;
        fg_last_fail = 0;
        gate_resolved = false;
        gate.disarm();
        corner_ix++;
        // The launch straight is behind us once the FIRST corner is done,
        // anchor or no anchor. CCW the first anchor is diagonal post 1, and
        // lap 36 (2026-09-05) missed every post: the slip model stayed
        // disarmed for 1619 mm and (3,5) needed -50 with slip=+0.0.
        slip_armed = true;
    }

    bool in_junction = false;
    if (corner_ix < nc) {
        in_junction = ps.s_mm >= path.corner_entry_s_mm(corner_ix) &&
                      ps.s_mm < path.corner_exit_s_mm(corner_ix) +
                                    SWEEP_JUNCTION_EXIT_MM;
    }
    // ON THE DIAGONAL: the corner being worked on is a diagonal EXIT and its
    // arc has not begun. There is no corridor here -- posts flick past at
    // 45 degrees on alternate sides, 58 mm from the centre line -- so the
    // corridor model, its lateral servo and its lateral guard all stand
    // down exactly as they do in a junction, and the raw-beam proximity
    // guard covers the body. The observer records what the beams see.
    bool on_diag = corner_ix < nc && is_diag_exit(corner_ix) &&
                   ps.s_mm < path.corner_entry_s_mm(corner_ix);
    // ...and the TAIL of the entry 45's junction. The observer used to wake
    // only when corner_ix advanced -- 60 mm after the entry arc ends -- and
    // that 60 mm is post 1's ramp. Flown CCW (laps 327-332, 2026-09-04) the
    // launch error reaches the diagonal ungated (the diagonal is the FIRST
    // gate on that lap order), the edge arrives ~26 mm early, and only
    // ~20 mm of post 1's episode was visible: missed, deterministically,
    // every lap. Observing from the entry arc's END gives post 1 its ramp
    // back. The corridor model stays down here anyway (still in_junction).
    // A V's arc end is the next diagonal's start as well (a V is an exit
    // AND an entry), so only "the next corner is a diagonal exit" is asked.
    bool on_diag_early = !on_diag && corner_ix + 1 < nc &&
                         is_diag_turn(corner_ix) &&
                         is_diag_exit(corner_ix + 1) &&
                         ps.s_mm >= path.corner_exit_s_mm(corner_ix);
    if (on_diag || on_diag_early) {
        int ek = on_diag ? corner_ix : corner_ix + 1;
        if (dg_exit_k != ek) dg_begin(ek);
        dg_update(w, ps.s_mm);
        // The dead reckoning: heading relative to the line, along path-s.
        if (!ps.in_arc) dg_dr.step(ps.s_mm, ps.bias_deg - ps.yaw_err_deg);
    }
    bool model_off = in_junction || on_diag;
    // The slip model rides the ARMED flag but stands down on the diagonal:
    // the diagonal straight does not slip (CW posts read a steady +5 of
    // over-subtraction; CCW it walked every post out of its window,
    // 2026-09-05), and the posts re-anchor every 127 mm anyway.
    control_path_slip_enable(slip_armed && !(on_diag || on_diag_early));

    float lat = 0.0f;
    uint8_t lat_src = sweep_corridor_lat(wv, model_off, &lat);
    lr_lat_mm = lat;
    lr_lat_valid = lat_src != SWEEP_LAT_NONE;

    // Lap report: open the corner window on first entry, then sample the
    // path yaw error and the OUTER beam (the wall the mouse would hit going
    // wide) every packet inside it. Identity comes from turns[] -- one arc
    // per turn, in order, same defensive guard the gate print uses.
    if (in_junction && corner_ix < n_turns) {
        lap_report_corner_enter(corner_ix, turns[corner_ix].cx,
                                turns[corner_ix].cy, turns[corner_ix].r_mm,
                                (int)turns[corner_ix].sweep_deg,
                                control_heading().yaw_deg);
        bool lr_left = path.corner_dir(corner_ix) > 0;
        lap_report_corner_sample(corner_ix, ps.yaw_err_deg,
                                 lr_left ? w.right_mm : w.left_mm);
    }

    // Lateral servo on the corridor only, and only on a lateral good enough
    // to steer on. SWEEP_LAT_BOTH means two beams that agree the corridor is
    // 168 wide; anything less is not steered on, because a wrong correction
    // is worse than none.
    // THE FLINCH, vector form (Control.h: flinch_mm, flinch_tau_ms). Raw
    // beams and their closing rate, no model, on its own yaw-reference
    // channel (control_set_reflex_bias_deg) so it works INSIDE ARCS too.
    // Sign: yaw is left-positive, so a wall close on the LEFT steers right
    // (negative). Three contexts:
    //   STRAIGHT  either beam inside `flinch`, and not receding: the P law,
    //             FLINCH_DEG_PER_MM per mm inside, cap FLINCH_MAX_DEG.
    //   ARC       the INSIDE beam only, on time-to-contact: range under
    //             REFLEX_ARC_RANGE_MM and range/closing-speed under `ftau`
    //             (reflex_arc_bias_deg). Opens the arc away from the post.
    //   DIAGONAL  either beam inside REFLEX_DIAG_RANGE_MM (posts pass at
    //             41-48 on a centred run), the P law; NOT tau -- a post's
    //             ramp is 130 -> 45 in 60 mm, tau ~50 ms on a clean pass.
    float reflex_bias = 0.0f;
    char reflex_ctx = '-';
    rfx_push(ps.s_mm, w.left_mm, w.right_mm);
    if (flinch_live > 0.0f || ftau_live > 0.0f) {
        float cl = 0.0f, cr = 0.0f;
        bool lrate = rfx_closing(false, ps.s_mm, w.left_mm, &cl);
        bool rrate = rfx_closing(true, ps.s_mm, w.right_mm, &cr);
        if (ps.in_arc && corner_ix < nc) {
            bool left_turn = path.corner_dir(corner_ix) > 0;
            uint8_t in_mm = left_turn ? w.left_mm : w.right_mm;
            bool ok = left_turn ? lrate : rrate;
            float per_mm = left_turn ? cl : cr;
            if (ok && beam_live(in_mm)) {
                float mag = reflex_arc_bias_deg((float)in_mm,
                                                per_mm * ps.v_mm_s, ftau_live);
                reflex_bias = left_turn ? -mag : mag;   // away from the inside
                if (mag > 0.0f) reflex_ctx = 'a';
            }
        } else if (flinch_live > 0.0f) {
            bool diag = on_diag || on_diag_early;
            float lim = diag ? REFLEX_DIAG_RANGE_MM : flinch_live;
            if (diag || !model_off) {
                // receding = rate valid and more negative than the limit;
                // no rate (edge, dead beam) does not veto a close reading.
                bool l_ok = beam_live(w.left_mm) && (float)w.left_mm < lim &&
                            !(lrate && cl < -REFLEX_RECEDE_MM_PER_MM);
                bool r_ok = beam_live(w.right_mm) && (float)w.right_mm < lim &&
                            !(rrate && cr < -REFLEX_RECEDE_MM_PER_MM);
                if (l_ok) reflex_bias -= FLINCH_DEG_PER_MM * (lim - (float)w.left_mm);
                if (r_ok) reflex_bias += FLINCH_DEG_PER_MM * (lim - (float)w.right_mm);
                if (reflex_bias > FLINCH_MAX_DEG) reflex_bias = FLINCH_MAX_DEG;
                if (reflex_bias < -FLINCH_MAX_DEG) reflex_bias = -FLINCH_MAX_DEG;
                if (reflex_bias != 0.0f) reflex_ctx = diag ? 'd' : 's';
            }
        }
    }
    if (reflex_bias != 0.0f) {
        if (rfx_fires == 0) { rfx_first_s = ps.s_mm; rfx_first_ctx = reflex_ctx; }
        rfx_fires++;
        if (fabsf(reflex_bias) > rfx_max_deg) rfx_max_deg = fabsf(reflex_bias);
    }
    if (reflex_bias != rfx_bias_prev) {
        control_set_reflex_bias_deg(reflex_bias);
        rfx_bias_prev = reflex_bias;
    }
    // THE V LANE's window (DiagLane.h): after a V until the next arc, or in
    // the pitch before a V. The push's dead reckoning is seeded by the last
    // post sample (dg_update re-seeds it on every one) -- or, with none on
    // this diagonal, from the centre line where the push begins.
    bool lane_servo = false;
    if ((on_diag || on_diag_early) && diag_gate_live >= 2.0f && !ps.in_arc &&
        ps.s_mm < dg_exit_arc_s) {
        bool after_v = dg_dr.valid && dg_dr.from_v && ps.s_mm - dg_dr.s0 <= V_LANE_MAX_MM;
        bool before_v = dg_lane.in_entry_push(ps.s_mm);
        if (before_v && !dg_dr.valid)
            dg_dr.seed(dg_lat_valid ? dg_lat_est : diag_lat_live, ps.s_mm, false);
        if (before_v && ps.s_mm - dg_dr.s0 > V_DR_MAX_AGE_MM) before_v = false;
        lane_servo = after_v || before_v;
    }
    float ls_sp = ls_set_point(ps.s_mm, lat, ps.yaw_err_deg, ps.v_mm_s,
                               !model_off && lat_src == SWEEP_LAT_BOTH && !ps.in_arc);
    if (!model_off && lat_src == SWEEP_LAT_BOTH) {
        lat_bias_last = -latk_live * (lat - ls_sp);
        if (lat_bias_last > LAT_SERVO_BIAS_MAX_DEG) lat_bias_last = LAT_SERVO_BIAS_MAX_DEG;
        if (lat_bias_last < -LAT_SERVO_BIAS_MAX_DEG) lat_bias_last = -LAT_SERVO_BIAS_MAX_DEG;
        control_set_lateral_bias_deg(lat_bias_last);
        // THE WALLS OWN THE FRAME (wall_align): a straight with both walls
        // is where the corridor's direction can be measured; feed it --
        // not during a commanded lane change, which looks like a heading
        // error to it (the step test's window: from the step to LS_LOG_MM).
        bool ls_window = ls_mm != 0.0f && ls_done && ps.s_mm - ls_s0 <= LS_LOG_MM;
        if (walign_live > 0.0f && !ps.in_arc && !ls_window)
            fr_sample(ps.s_mm, lat,
                      -ps.yaw_err_deg + lat_bias_last + rfx_bias_prev);
        else
            fr_window_reset();
    } else if (lane_servo) {
        // THE V LANE (DiagLane.h): the walk back after a V and the push
        // before one, on the dead-reckoned lateral -- the set point's slope
        // as feedforward, the lane gain on what the estimate says is left.
        // Before 2026-09-30 nothing steered after a V (every V left the
        // mouse vout outside at its first outer post: 09-30 MR2 read 37 and
        // hit); before 10-01 the push before a V rode the post servo's held
        // sample with no feedforward and made ~3 of its 10 mm. The whole set
        // point is followed (a second V's push overlaps the first's walk
        // back), looked up a heading-lag ahead, and ends before the next arc.
        dg_lane.base = diag_lat_live;
        lat_bias_last = v_lane_bias_deg(dg_lane, ps.s_mm, ps.v_mm_s, dg_dr.lat,
                                        V_LANE_K_DEG_PER_MM, LAT_SERVO_BIAS_MAX_DEG);
        control_set_lateral_bias_deg(lat_bias_last);
        fr_window_reset();
    } else if ((on_diag || on_diag_early) && diag_gate_live >= 2.0f &&
               dg_lat_valid && ps.s_mm - dg_lat_s <= DGATE_HOLD_MM) {
        // THE POSTS STEER. One lateral sample per post, held until the
        // next post refreshes it -- the same servo the corridor uses, fed
        // every 127 mm instead of every packet. dlat is the set point:
        // the servo pulls the measured lateral toward it, not toward a
        // zero the beams' own asymmetry has already bent (Control.h).
        // ...and held for ONE PITCH, no longer (DGATE_HOLD_MM): the gain is
        // sized so the bias moves ~80 % of the error in one pitch, so a
        // hold that outlives it keeps steering past the line. 2026-09-27's
        // home diagonal read a left post 11 mm right, missed the next two
        // posts, and held +3.6 deg for 360 mm -- 24 mm LEFT, into the
        // 135-out's inside post.
        lat_bias_last = -DGATE_SERVO_DEG_PER_MM *
                        (dg_lat_est - diag_set_point(ps.s_mm, dg_exit_k));
        control_set_lateral_bias_deg(lat_bias_last);
        fr_window_reset();
    } else {
        lat_bias_last = 0.0f;
        control_set_lateral_bias_deg(0.0f);
        fr_window_reset();
    }

    // SIDE WALLS NEVER STOP A RUN (2026-10-02, race day; bert: "this wasn't
    // a crash, so even if the mouse scrapes a side wall it can carry on --
    // it's not like a front impact. The side wall limit abort should be
    // removed."). The 10-01 MR5 on the new maze left the (4,4) turn 43 mm
    // off centre, 41 mm from the north wall, and GUARD 1 ("lateral limit",
    // 40 mm toward a seen wall, twice) ended a run that was in the goal
    // block. Both side-beam aborts -- the corridor lateral limit and the
    // corner beam-proximity limit -- are now NOTES, printed once per leg;
    // the corridor servo and the flinch keep steering away. What still
    // stops a run: the front governor (a wall AHEAD), the heading-error
    // limit (a spin), the leg timeout, a silent or blind sensor link.
    if (!model_off && lat_src != SWEEP_LAT_NONE && fabsf(lat) > SWEEP_LAT_ABORT_MM) {
        if (lat_over_run++ == 0)
            printf_line("route: %.0f mm off centre at s %.0f (L %u R %u) -- side "
                        "walls do not stop the run, carrying on",
                        (double)fabsf(lat), (double)ps.s_mm, w.left_mm, w.right_mm);
    }
    if (model_off) {
        uint8_t nearest = w.left_mm < w.right_mm ? w.left_mm : w.right_mm;
        bool valid = nearest != 0 && nearest != 255;
        if (valid && (float)nearest < SWEEP_BEAM_ABORT_MM && beam_close_run++ == 0)
            printf_line("route: side beam %u mm in a corner at s %.0f -- "
                        "carrying on", nearest, (double)ps.s_mm);
    }
    if (fabsf(ps.yaw_err_deg) > SWEEP_YAW_ABORT_DEG) {
        route_run_abort("heading error limit");
        return;
    }

    // ---- THE POST GATE ----------------------------------------------------
    //
    // Anchor the corner on the end of the wall it turns around. In a maze
    // that end is a post, and posts are the one feature the regulations
    // guarantee at every cell corner -- which is exactly why the mouse
    // navigates by them rather than by walls.
    bool gate_applied = false;
    if (corner_ix < nc) {
        if (!gate.armed() && !gate_resolved &&
            ps.s_mm >= gate_expect_s(corner_ix) - ROUTE_GATE_OBSERVE_MM) {
            if (is_diag_turn(corner_ix) && !is_135_in(corner_ix)) {
                // NEITHER END OF A DIAGONAL IS GATED. The 45-out is met at
                // 45 degrees to every wall, so the post-and-edge geometry
                // does not exist. The 45-in LOOKS gateable -- it turns
                // around the same post a 90 would -- but its tangent is
                // 49 mm against a 90's 98.6, so the arc begins 3 mm after
                // the edge drops and the beam swings onto the very post it
                // is turning around: three of five first-flight laps read
                // SKIP_LAT with a "lateral" of 30-37 mm on a perfectly
                // centred approach (L 82 / R 85). The diagonal's own posts
                // place both ends (the DGATE block).
                gstats.corners++;
                lap_report_gate(corner_ix, LR_GATE_DIAG, 0.0f, 127.0f);
                gate_resolved = true;
                // THE 45-IN'S ONLY LEVER. Ungated, it fires on the previous
                // anchor plus dead reckoning, and its arc hugs the inside
                // post by design (~5 mm on a clean lap 29, 2026-09-05; lap
                // 30 entered 6 mm further inside and clipped it). A hand
                // trim on its cell (goffc.trim cx cy h mm) is applied here
                // as a one-shot correction, same sign as the 90s': + = arc
                // later in the world = more room from the inside post. The
                // diagonal's own posts re-anchor 155 mm on, so a late 45-in
                // costs nothing downstream.
                if (!is_diag_exit(corner_ix) && corner_ix < n_turns) {
                    float trim = goff_table_trim_mm(turns[corner_ix].cx,
                                                    turns[corner_ix].cy,
                                                    turns[corner_ix].entry_h);
                    // ...PLUS what the beams say right now. The mouse has
                    // just measured its lateral in the corridor feeding the
                    // 45 (both walls -- lr_lat_valid), and an entry on the
                    // INSIDE is the clip: laps 30 and 15:07 (2026-09-05)
                    // both entered 6-33 mm inside and hit post (5,4) with
                    // the arc otherwise perfect. Firing later by the inside
                    // offset moves the body off that post by ~0.7x of it.
                    // Outside offsets are not compensated: a late-and-wide
                    // 45 is harmless, the diagonal posts re-anchor.
                    float inside = 0.0f, lat_in = 0.0f;
                    if (lr_lat_valid) {
                        float lat = lr_lat_mm;                 // + = left
                        inside = path.corner_dir(corner_ix) > 0 ? lat : -lat;
                        lat_in = inside;                       // signed, for the lattice
                        if (lat_in > DIAG_IN_LAT_MAX_MM) lat_in = DIAG_IN_LAT_MAX_MM;
                        if (lat_in < -DIAG_IN_LAT_MAX_MM) lat_in = -DIAG_IN_LAT_MAX_MM;
                        if (inside < 0.0f) inside = 0.0f;
                        if (inside > DIAG_IN_LAT_MAX_MM) inside = DIAG_IN_LAT_MAX_MM;
                    }
                    float shift = trim + DIAG_IN_LAT_GAIN * inside;
                    // Where this puts the mouse on the diagonal, for the post
                    // observer (dg_join_ahead_mm): an inside offset and a
                    // later arc both project onto the 45-degree line at
                    // 1/sqrt2; an outside entry (lat_in < 0) lands it short.
                    dg_join_corner = corner_ix;
                    dg_join_ahead_mm = (lat_in + shift) * 0.70710678f;
                    if (shift != 0.0f) {
                        float room = path.corner_entry_s_mm(corner_ix) - ps.s_mm;
                        if (room < 5.0f) room = 5.0f;
                        control_path_correct(-shift, room);
                    }
                    if (shift != 0.0f || fabsf(lat_in) >= 3.0f)
                        printf_line("  45-in: corner %d (%u,%u) trim %+.1f, "
                                    "lat %+.0f mm (+ inside) -> %+.1f later, "
                                    "joins the diagonal %+.0f mm ahead",
                                    corner_ix, turns[corner_ix].cx,
                                    turns[corner_ix].cy, (double)trim,
                                    (double)lat_in, (double)shift,
                                    (double)dg_join_ahead_mm);
                } else if (is_v_turn(corner_ix) && corner_ix < n_turns) {
                    // THE V'S LEVER (2026-09-06). Same idea, measured by
                    // the diagonal's own posts instead of a corridor: the
                    // latest post sample says how far INSIDE the mouse is
                    // riding the entry diagonal. The V's inside post sits
                    // on the bisector, so firing the arc later by d moves
                    // the apex off it by 0.7 d -- hence the 1.4 gain --
                    // and puts the exit 0.7 d toward the OUTER side of the
                    // next diagonal, where the nearest post is 90 mm away.
                    // Laps 80-85 on D8 all arrived 13-16 mm inside and
                    // read the post at 40-44 through the arc.
                    float trim = goff_table_trim_mm(turns[corner_ix].cx,
                                                    turns[corner_ix].cy,
                                                    turns[corner_ix].entry_h);
                    float inside = 0.0f;
                    if (dg_lat_valid) {
                        inside = path.corner_dir(corner_ix) > 0 ? dg_lat_est
                                                                : -dg_lat_est;
                        if (inside < 0.0f) inside = 0.0f;
                        if (inside > DIAG_IN_LAT_MAX_MM) inside = DIAG_IN_LAT_MAX_MM;
                    }
                    // ...plus what this V's own exit taught the mouse on an
                    // earlier run of THIS maze (DiagLane.h, the V timing
                    // learner): + = later. Zero on MR2 after a search.
                    float learned = v_learn_live >= 0.5f
                                  ? goff_table_mm(turns[corner_ix].cx,
                                                  turns[corner_ix].cy,
                                                  (uint8_t)(turns[corner_ix].entry_h |
                                                            GOFF_KEY_V))
                                  : 0.0f;   // vlearn 0: a stored timing is never applied
                    if (learned > V_LEARN_MAX_MM) learned = V_LEARN_MAX_MM;
                    if (learned < -V_LEARN_MAX_MM) learned = -V_LEARN_MAX_MM;
                    float shift = trim + learned + V_IN_LAT_GAIN * inside;
                    v_shift_k = corner_ix;
                    v_shift_mm = shift;     // handed back at the arc end (dg_begin)
                    if (shift != 0.0f) {
                        float room = path.corner_entry_s_mm(corner_ix) - ps.s_mm;
                        if (room < 5.0f) room = 5.0f;
                        control_path_correct(-shift, room);
                        printf_line("  V: corner %d (%u,%u) trim %+.1f, learned %+.1f, "
                                    "inside %.0f mm -> %+.1f later",
                                    corner_ix, turns[corner_ix].cx,
                                    turns[corner_ix].cy, (double)trim,
                                    (double)learned, (double)inside, (double)shift);
                    }
                }
            } else if (ps.s_mm > gate_expect_s(corner_ix)) {
                // ALREADY PAST THE DROP. This corner sits so close behind the
                // previous one that clearing its junction put the mouse past
                // its own post. Do not arm: there is nothing left to see, and
                // an armed gate here would spend its decision on whatever the
                // beam happens to be looking at. The position is inherited
                // from the previous corner's anchor, 180 mm ago, and is still
                // fresh -- see the note in RouteRun.h.
                gstats.corners++;
                gstats.inherited++;
                lap_report_gate(corner_ix, LR_GATE_INHERITED, 0.0f, 127.0f);
                gate_resolved = true;
            } else {
                gate_arm_for(corner_ix);
                gstats.corners++;
                cur_skipped_lat = false;
            }
        }
        if (gate.armed() && !gate.fired()) {
            // ---- FRONT-WALL GATE (see the FGATE constants) --------------
            bool front_handled = false;
            // RAW front beams, not wv.front: the usable range window sits
            // ABOVE walls_evaluate's 200 mm presence flag (see the FGATE
            // constants). Beams are forward-from-centre mm; 255 = OOR.
            // The last beam range that actually went into the ring. It must
            // PERSIST across packets: the fire test (count + span) is
            // re-evaluated every packet, and span grows with s, so the gate
            // routinely fires on a packet where neither beam contributed --
            // printing a per-packet value gave "at front 0" (seq 273,
            // corner 6). The median is taken from the ring either way, so
            // this was only ever the report, but a zero in the log is a
            // zero someone has to chase.
            float &fr = fg_last_range;
            if (!fg_fired && fgate_live > 0.0f && corner_ix < n_turns &&
                turns[corner_ix].front_ok && wv.valid &&
                ps.s_mm < path.corner_entry_s_mm(corner_ix)) {
                float expect_range = fgate_face_s(corner_ix) - ps.s_mm;
                const uint8_t beams[2] = {w.front_left_mm, w.front_right_mm};
                for (int b = 0; b < 2; b++) {
                    if (beams[b] == 0 || beams[b] > FGATE_MAX_RANGE_MM)
                        continue;
                    float err = expect_range - (float)beams[b];
                    // Plausibility: a sample the gate could not act on
                    // anyway is a phantom or another wall -- drop it rather
                    // than let it poison the median.
                    if (fabsf(err) >= gate_window_live) continue;
                    if (fg_n == 0) {
                        fg_first_s = ps.s_mm;
                        fg_first_err = err;
                    }
                    fg_ring[fg_ix] = err;
                    fg_ix = (fg_ix + 1) % FGATE_RING;
                    if (fg_n < FGATE_RING) fg_n++;
                    fr = (float)beams[b];
                }
                float span = fg_n > 0 ? ps.s_mm - fg_first_s : 0.0f;
                if (fg_n >= FGATE_MIN_N && span >= FGATE_SPAN_MM) {
                    // Median + spread of the ring. A phantom (constant
                    // range) makes the implied error slide 1:1 with s --
                    // spread rejects it; a real wall's samples agree.
                    float srt[FGATE_RING];
                    for (int i = 0; i < fg_n; i++) srt[i] = fg_ring[i];
                    for (int i = 1; i < fg_n; i++)
                        for (int j = i; j > 0 && srt[j] < srt[j - 1]; j--) {
                            float t = srt[j];
                            srt[j] = srt[j - 1];
                            srt[j - 1] = t;
                        }
                    float med = srt[fg_n / 2];
                    float spread = srt[fg_n - 1] - srt[0];
                    fg_last_fail = spread > FGATE_SPREAD_MM      ? 1
                                   : fabsf(med - fg_first_err) >
                                             FGATE_DRIFT_MM      ? 2
                                   : fabsf(med) >= gate_window_live ? 3
                                                                    : 0;
                    if (fg_last_fail == 0) {
                        float trim = trim_for(corner_ix);
                        // Drain over the room the FRONT gate actually has
                        // left -- it fires earlier than the side edge, so
                        // gate_room's side-gate figure would under-state it
                        // and dump the correction as a step into the arc.
                        float froom = path.corner_entry_s_mm(corner_ix) -
                                      ps.s_mm;
                        if (froom < 5.0f) froom = 5.0f;
                        // fgate 1 = OBSERVE (measure, log, do not steer);
                        // 2 = ACT. Observe is the safe default because the
                        // front bias proved to be PER CORNER, not global:
                        // laps 196-214 measured (0,5) at +24.4 (sd 1.4!)
                        // and (1,3) at -3.1 in the same session, so any one
                        // ftrim is wrong at one of them -- "some crashed
                        // head on, others turned too tight" (bert). Observe
                        // mode flies exactly as fgate=0 while recording
                        // what the front wall would have said, which is the
                        // dataset a per-corner front trim needs.
                        // ALWAYS record what the front wall said, so the
                        // lap carries front and side for the SAME corner.
                        lap_report_front(corner_ix, med);
                        if (fgate_live >= 2.0f &&
                            (!is_diag_turn(corner_ix) || is_135_in(corner_ix))) {
                            control_path_correct(med - trim, froom);
                            gate_applied = true;
                            fg_applied = true;
                            led_flash(1);   // the front wall placed this arc
                        }
                        lap_report_gate(corner_ix, LR_GATE_FRONT, med, lat);
                        gstats.locked++;
                        gstats.corr_sum_mm += med;
                        if (med < gstats.corr_min_mm) gstats.corr_min_mm = med;
                        if (med > gstats.corr_max_mm) gstats.corr_max_mm = med;
                        // THE LEARNER DOES NOT LEARN FROM THE FRONT.
                        // The table shifts the SIDE expectation too, so a
                        // front-only bias (long-range LUT error) fed in
                        // here would walk the side gate off its own post
                        // within two laps -- the goff-cancellation
                        // mechanism, pointed at the wrong landmark. Front
                        // bias belongs in ftrim, which is measured, not
                        // learned. A FUSED corner still learns, from its
                        // SIDE residual (below): that one IS a side-gate
                        // measurement, taken in a corrected frame.
                        first_lock_pending = false;
                        float rode = gate_expect_s(corner_ix) - anchor_s_mm;
                        if (fg_applied) {
                            anchor_s_mm = gate_expect_s(corner_ix);
                            seg_anchor(corner_ix, rode, true);
                        }
                        printf_line("  fgate%s: corner %d (%u,%u)  corr %+.1f "
                                    "mm at front %.0f (n=%d span %.0f "
                                    "spread %.1f, room %.0f), dr %.0f",
                                    fg_applied ? "" : " OBSERVED",
                                    corner_ix, turns[corner_ix].cx,
                                    turns[corner_ix].cy, (double)med,
                                    (double)fr, fg_n, (double)span,
                                    (double)spread, (double)froom,
                                    (double)rode);
                        // FUSION: the side gate stays ARMED. Its edge now
                        // drops against a front-corrected frame; if it
                        // fires, the residual refines the anchor and the
                        // corner reports FUSED (see the constants note).
                        fg_fired = true;
                        fg_med = med;
                        // In OBSERVE the side gate keeps the corner: it
                        // still watches, still fires, still owns the LC
                        // gate/corr, and the lap flies exactly as fgate=0.
                        // Only an APPLIED front anchor consumes the packet.
                        front_handled = fg_applied;
                    }
                    // spread or window failed: keep sampling; the side gate
                    // below still watches its edge as if we were never here.
                }
            }
            if (front_handled) {
                // fall through to the record section -- this packet's gate
                // work is done.
            } else {
            // The INSIDE beam: left for a left turn, right for a right one.
            // corner_dir comes from the path's own curvature, so it cannot
            // disagree with the arc that is about to be flown.
            bool left_turn = path.corner_dir(corner_ix) > 0;
            uint8_t inside = left_turn ? w.left_mm : w.right_mm;

            // DO NOT FEED THE DETECTOR A READING THAT CANNOT BE A WALL.
            // Under SWEEP_SIDE_MIN_MM the wall would be within three
            // millimetres of the bodywork, which does not happen without a
            // mark on the mouse -- it is the channel's floor when it has no
            // usable return. Skipping the packet outright leaves every run
            // counter untouched: it cannot invent an opening and it cannot
            // hold a wall open either. (Runs 127-129: cleaning the optics and
            // killing the lights changed nothing, because the deficient
            // reflector was the WALL.)
            if (inside != 0 && inside != 255 &&
                (float)inside < SWEEP_SIDE_MIN_MM)
                inside = 0;   // sentinel: skip this packet

            float corr = 0.0f;
            // The gate wants lateral signed TOWARD THE INSIDE wall (the one
            // it watches): +lat is a leftward displacement in the corridor
            // frame, which is toward the inside only for a left turn.
            float lat_inside = left_turn ? lat : -lat;
            WallGate::Result gr =
                inside == 0
                    ? WallGate::NONE
                    : gate.update(ps.s_mm, inside,
                                  lat_src != SWEEP_LAT_NONE, lat_inside,
                                  &corr);
            if (gr == WallGate::CORRECTION) {
                led_flash(1);   // a wall end, seen and locked
                // Drain it over the room actually left before the arc. That
                // room shrinks with radius (36.4 mm at R84/v500, 54.1 at R60,
                // 61.2 at R45), and a fixed rate would spill a large-R
                // correction into the very arc it was meant to set up.
                //
                // ARC PLACEMENT TRIM (atrim/utrim): subtracted from the
                // APPLIED correction only -- the gate's measurement (corr,
                // logged below) stays honest, and the arc physically slides
                // +trim mm LATER relative to the locked edge. This is the
                // ring rig's exit lever (21 mm of exit per 20 mm of trim,
                // runs 120-122) in maze form, and the only placement knob
                // the glearn loop cannot cancel: expectation-side offsets
                // (the goff family) converge back to the physical edge
                // within two laps. The believed-frame offset it introduces
                // is re-anchored by the next gate; the table absorbs it
                // once, capped by GOFF_TABLE_STEP_MM.
                {
                    float trim = trim_for(corner_ix);
                    // ONCE PER CORNER, not once per gate. When the front
                    // gate already placed this arc, this drop is a
                    // REFINEMENT: applying the trim again slid the arc a
                    // second 30 mm early -- ~21 mm further toward the
                    // inside wall -- which is exactly what wrecked every
                    // FUSED corner in laps 196/198 while the fgate=0 lap
                    // between them flew clean.
                    if (fg_applied) trim = 0.0f;
                    control_path_correct(corr - trim, gate_room(corner_ix));
                }
                gate_applied = true;
                if (fg_applied) {
                    // FUSED: the front anchor already fixed the frame and
                    // fed the learner; this drop is the refinement. corr
                    // here =~ atrim + (front-vs-edge disagreement) -- the
                    // subtraction above turns it into exactly the residual
                    // applied. Report the front error as the corner's corr
                    // (comparable across laps); print the disagreement.
                    lap_report_gate(corner_ix, LR_GATE_FUSED, fg_med,
                                    gate.last_lat_mm());
                    // In OBSERVE mode nothing was applied, so this is an
                    // ordinary side-gate lock and learns as one.
                    if (corner_ix < n_turns && learn_n < ROUTE_MAX_TURNS &&
                        !(launch_was_setback && first_lock_pending) &&
                        anchor_fresh) {
                        learn_q[learn_n].cx = turns[corner_ix].cx;
                        learn_q[learn_n].cy = turns[corner_ix].cy;
                        learn_q[learn_n].h = turns[corner_ix].entry_h;
                        learn_q[learn_n].corr = corr;
                        learn_n++;
                    } else if (corner_ix < n_turns) {
                        learn_skipped++;
                    }
                    first_lock_pending = false;
                    printf_line("  fuse: corner %d (%u,%u)  front %+.1f, "
                                "side resid %+.1f mm (vs trim)",
                                corner_ix,
                                corner_ix < n_turns ? turns[corner_ix].cx : 0,
                                corner_ix < n_turns ? turns[corner_ix].cy : 0,
                                (double)fg_med, (double)corr);
                } else {
                lap_report_gate(corner_ix, LR_GATE_LOCK, corr, gate.last_lat_mm());
                // Queue for the lap-end fold into the per-corner table --
                // except the first lock after a hand-placed launch, whose
                // corr is the placement datum, not corner geometry (see
                // launch_was_setback above).
                bool skip_learn = (first_lock_pending && launch_was_setback)
                                  || !anchor_fresh;
                first_lock_pending = false;
                if (!skip_learn && corner_ix < n_turns &&
                    learn_n < ROUTE_MAX_TURNS) {
                    learn_q[learn_n].cx = turns[corner_ix].cx;
                    learn_q[learn_n].cy = turns[corner_ix].cy;
                    learn_q[learn_n].h = turns[corner_ix].entry_h;
                    learn_q[learn_n].corr = corr;
                    learn_n++;
                } else if (corner_ix < n_turns) {
                    learn_skipped++;
                }
                gstats.locked++;
                gstats.corr_sum_mm += corr;
                if (corr < gstats.corr_min_mm) gstats.corr_min_mm = corr;
                if (corr > gstats.corr_max_mm) gstats.corr_max_mm = corr;
                // route_build emits exactly one arc per turn, in order, so
                // corner k is turns[k] -- but print defensively rather than
                // index a stale array if that ever stops being true.
                // This post is now the last known good one: the map pins
                // here, and the next gate's window is sized by how far it
                // rides from this spot. dr = the dead-reckoned distance
                // THIS lock rode -- the number that separates odometry
                // drift from build geometry when a corr surprises.
                float rode = gate_expect_s(corner_ix) - anchor_s_mm;
                anchor_s_mm = gate_expect_s(corner_ix);
                // A FUSED corner keeps the front's record (the long segment);
                // this refinement rode only the front-to-edge gap.
                seg_anchor(corner_ix, rode, !fg_applied);
                if (corner_ix < n_turns)
                    printf_line("  gate: corner %d (%u,%u) R%u  corr %+.1f mm "
                                "at s %.0f, %.0f mm to the arc, dr %.0f%s",
                                corner_ix, turns[corner_ix].cx,
                                turns[corner_ix].cy, turns[corner_ix].r_mm,
                                (double)corr, (double)gate.drop_s_mm(),
                                (double)gate_room(corner_ix), (double)rode,
                                skip_learn ? "  [stale anchor: flown, not "
                                             "learned]"
                                           : "");
                else
                    printf_line("  gate: corner %d  corr %+.1f mm at s %.0f",
                                corner_ix, (double)corr,
                                (double)gate.drop_s_mm());
                }   // !fg_applied (plain side lock -- also the OBSERVE path)
            } else if (gr == WallGate::SKIPPED_LAT) {
                // An edge WAS seen, but the mouse was too far off centre for
                // the reading to mean what it looks like it means. The corner
                // goes ungated rather than half-corrected; the gate stays
                // armed, so a cleaner packet can still anchor it. Recorded
                // as the corner's outcome, not counted here -- see
                // cur_skipped_lat.
                cur_skipped_lat = true;
            }
            }   // !front_handled (side gate)
        }

        // FRONT AS FALLBACK (fgate >= 2). The side gate has had its whole
        // approach and found nothing; the arc is about to start; the front
        // wall has a measurement. Take it rather than fly the corner on an
        // inherited anchor. At fgate >= 3 the front leads instead and this
        // never triggers (fg_applied already set).
        if (fgate_live >= 2.0f && fg_fired && !fg_applied &&
            (!is_diag_turn(corner_ix) || is_135_in(corner_ix)) &&
            gate.armed() && !gate.fired() && corner_ix < n_turns &&
            ps.s_mm >= path.corner_entry_s_mm(corner_ix) -
                           FGATE_FALLBACK_MM) {
            float trim = trim_for(corner_ix);
            float froom = path.corner_entry_s_mm(corner_ix) - ps.s_mm;
            if (froom < 5.0f) froom = 5.0f;
            control_path_correct(fg_med - trim, froom);
            gate_applied = true;
            fg_applied = true;
            led_flash(1);   // the front wall placed this arc (fallback)
            lap_report_gate(corner_ix, LR_GATE_FRONT, fg_med,
                            gate.last_lat_mm());
            gstats.locked++;
            gstats.corr_sum_mm += fg_med;
            if (fg_med < gstats.corr_min_mm) gstats.corr_min_mm = fg_med;
            if (fg_med > gstats.corr_max_mm) gstats.corr_max_mm = fg_med;
            first_lock_pending = false;
            {
                float rode = gate_expect_s(corner_ix) - anchor_s_mm;
                anchor_s_mm = gate_expect_s(corner_ix);
                seg_anchor(corner_ix, rode, true);
            }
            gate.disarm();
            gate_resolved = true;
            printf_line("  fgate FALLBACK: corner %d (%u,%u) no side edge -- "
                        "front corr %+.1f mm, room %.0f",
                        corner_ix, turns[corner_ix].cx, turns[corner_ix].cy,
                        (double)fg_med, (double)froom);
        }
    }

    // RECORD. The flight recorder, not a new format: 'y' already reads it and
    // the offline tools already parse it. wheel_fused_mm is the one to watch
    // -- with no wall gate on this path it is the ONLY thing that says how
    // far the odometry has drifted from truth over the leg.
    //
    // DECIMATED BY DISTANCE (RR_LOG_DS_MIN_MM): this packet is written if
    // it is an event (a flash, a gate correction, an arc boundary), or the
    // path has moved log_ds since the last record (times the capacity
    // ladder), or the mouse is stationary and RR_LOG_STILL_MS have passed.
    {
        bool arc_edge = ps.in_arc != log_last_in_arc;
        log_last_in_arc = ps.in_arc;
        bool event = log_event || gate_applied || arc_edge;
        // The route sized its spacing to the leg, so the search's ladder
        // (x2 at half) would only coarsen a run that is on plan; here the
        // backstop is the last 5 %: past it, the spacing quadruples rather
        // than let the ring eat the launch.
        float scale = flog_fill() >= RR_LOG_PANIC_FILL ? 4.0f : 1.0f;
        bool moved = ps.s_mm - log_last_s >= log_ds_mm * scale ||
                     ps.s_mm < log_last_s;
        bool still = ps.v_mm_s == 0.0f && ps.s_mm == log_last_s &&
                     now - log_last_ms >= RR_LOG_STILL_MS;
        log_event = false;
        if (!event && !moved && !still) return;
        log_last_s = ps.s_mm;
        log_last_ms = now;
    }
    ControlMotion mo = control_motion();
    ControlOdometry od = control_odometry();
    ControlHeading hd = control_heading();
    FlogRec r;
    memset(&r, 0, sizeof r);
    r.t_ms = (uint16_t)(now - state_t0);
    r.s_mm = (int16_t)ps.s_mm;
    r.v_mm_s = (int16_t)ps.v_mm_s;
    r.yaw_cdeg = (int16_t)(hd.yaw_deg * 10.0f);
    r.yaw_err_cdeg = (int16_t)(ps.yaw_err_deg * 10.0f);
    r.ay_mm_s2 = (int16_t)mo.a_lat_mm_s2;
    r.ax_mm_s2 = (int16_t)mo.a_fwd_mm_s2;
    // bit6: the HOME leg of a two-leg run (s restarts at 0 from the goal;
    // the viewer needs the boundary).
    // bit5 ABS acting, bit7 slip gate open (2026-09-25: the first route
    // flight with the ABS and the edge velocity ran its path position 18 %
    // ahead of the wheels on a straight and the log could not say which of
    // the estimator's gates was open -- now it can).
    r.flags = (uint8_t)((ps.in_arc ? 1 : 0) |
                        (ps.front_gov ? 2 : 0) |       // the front governor capped v (Control.h)
                        (gate_applied ? 4 : 0) |
                        (lat_src != SWEEP_LAT_NONE ? 8 : 0) |
                        (mo.abs_active ? 0x20 : 0) |
                        (home_leg() ? 0x40 : 0) |
                        (mo.slip ? 0x80 : 0));
    r.left_mm = w.left_mm;
    r.right_mm = w.right_mm;
    r.front_left_mm = w.front_left_mm;
    r.front_right_mm = w.front_right_mm;
    r.wheel_fused_mm = (int16_t)(od.dist_mm - ps.s_mm);
    // MEASURED rate, not the commanded one in ps.omega_dps: omega/v is the
    // curvature the mouse actually flew, and comparing it against the
    // planned kappa is what separates "the arc is wrong" from "the arc is
    // right and something pushed me off it".
    r.omega_dps = (int16_t)control_move_status().yaw_rate_dps;
    flog_record(r);
}
