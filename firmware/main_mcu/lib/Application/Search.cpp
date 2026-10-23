#include "Search.h"
#include "RunLadder.h"
#include "Explore.h"

#include "HAL.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "Control.h"
#include "FlightLog.h"
#include "Fram.h"
#include "Params.h"  // params_crc32 -- shared CRC for FRAM blocks
#include "Sensors.h"
#include "StartGate.h"
#include "WallAlign.h"
#include "Walls.h"

// -- Tunables ---------------------------------------------------------------------

static constexpr float SEARCH_CELL_MM = 180.0f;
static constexpr float SEARCH_V = 300.0f;        // mm/s ('b'/'l' proven)
// Speed run: 400 out (braking 114 mm at 700 mm/s^2, inside the final
// cell; wall-stop guard margins unchanged), search speed home -- the time
// is earned outbound, the return is zero-risk recovery on the same
// proven route.
static constexpr float SEARCH_SPEED_V = 400.0f;
// 700, not 500: the extend-or-brake decision must WAIT for real side data.
// At 500 the decel starts 90 mm out, forcing the decision to d=105 with
// only ~4 side samples (< SEARCH_SIDE_MIN_N) -- optimistic sides then said
// "maybe an opening, turn" and the mouse braked EVERY corridor cell
// (hardware 2026-07-10, first goal run: no extension ever fired on the
// north corridor). At 700 the decel needs 64 mm (starts at 116 for a
// 1-cell leg), so the d=105 decision has ~9 samples of real wall evidence
// plus 2 packets of slack. The plant has done 800 mm/s^2 ramps ('c',
// 2026-07-08); watch stop slide -- if it grows, the shuffle absorbs it.
static constexpr float SEARCH_ACC = 700.0f;      // mm/s^2
// Legs stop 5-8 mm PAST the profile end -- the FF-dominated wheel loop
// underbrakes (ka deliberately 0) and momentum carries through; fused and
// wheel odometry agree on it to 0.2 mm across every leg of the 2026-07-10
// telemetry run. Aim the profile short by the measured overshoot so the
// physical stop lands on the cell centre; wall-referenced stops and the
// shuffle absorb the remaining spread. (Decision timing unaffected: the
// decel start moves 6 mm earlier, still past the d-76 decision point.)
static constexpr float SEARCH_STOP_TRIM_MM = 6.0f;
// 450/4000, up from 300/2500: in-place turns WALK laterally at low
// rotation rates -- stick-slip breaks asymmetrically and the gripping
// wheel drags the mouse sideways (measured ~40 mm on a goal 180,
// 2026-07-10). Faster turns put both tyres into kinetic friction
// together: symmetric scrub, rotation about the true centre. The
// closed-loop planner is position-feedback so the higher rate costs no
// accuracy; the finish band still lands it. If tracking lags at 450, a
// 't' re-tune of kpt at the new rate is the follow-up.
static constexpr float SEARCH_TURN_OMEGA = 450.0f;   // deg/s
static constexpr float SEARCH_TURN_ALPHA = 4000.0f;  // deg/s^2
static constexpr float SEARCH_TURN_SKIP_DEG = 3.0f;  // already close enough

// Front-wall stop reference: forward distance (mouse centre -> wall face) at
// a cell centre. Dead-centre = half a cell minus the wall inset: 90 - 6 =
// 84 mm (face-to-face across a cell = 180 - 2*6 = 168). Keep == BTEST_TARGET_MM.
// (bert 2026-08-08: hand-start run lands dead-centre at 84.)
static constexpr float SEARCH_FRONT_TARGET_MM = 84.0f;
// START POSE (bert, 2026-07-28). The mouse is placed with its BACK TO THE REAR
// WALL, so its reference point does NOT sit at the start-cell centre:
//     rear wall face      = 6 mm   (12 mm wall, half-thickness, spec 3.1)
//   + half body length    = 45 mm  (90 mm footprint, spec table 255)
//   -------------------------------
//     start y             = 51 mm  from the cell boundary
// The path is compiled from CELL CENTRES, so s=0 used to mean y=90 and the
// mouse began every run SEARCH_START_SETBACK_MM short of its own path origin.
// A straight lead-in of that setback is prepended to the path so
// s=0 lands on the real start pose.
//
// This matters MORE on the dense diagonal path than it did on the clothoid one:
// path_compile sets corner_count = 0 and path_front_ref = false there, so every
// gate is inert and along-track position is pure fused odometry from the start.
// Nothing downstream absorbs a placement error, and nothing OBSERVES along-track
// at all until the first wall ahead (~s 530-655 mm in logs 059/061), because a
// corridor of side walls carries no along-track information.
//
// VALIDATED on hardware 2026-07-28 via obs_along (map-referenced, shadow-only,
// never applied -- the one honest read of along-track error):
//     055/056/059  centre-placed, firmware assumed centre    obs_along -11.9 mm
//     061          south-wall placed, firmware assumes 51    obs_along -10.2 mm
// Both self-consistent; difference 1.7 +- 0.6 mm where a wrong lead-in would
// have shifted it by ~39. So 51/39 is right to within a few mm.
//
// Do NOT try to measure this with (s - integral of the v column): v is the
// COMMANDED velocity, so that difference is velocity tracking lag, not position
// error. It plateaus around -58 mm and looks convincingly like a start offset.
// It is not one.
//
// Unexplained residual: both configurations carry a -10 to -12 mm along-track
// bias that does not grow with distance (059 by quarter: -12.7/-6.1/-10.5
// /-14.8), so it is not an odometry scale error. Front-beam LUT zero is the
// prime suspect.
static void printf_line(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// START Y, RUNTIME-SETTABLE ('starty='). The compiled default is the
// back-to-the-rear-wall geometry above: 6 mm of wall face plus 45 mm of half
// body = 51 mm from the cell boundary, so the mouse begins 39 mm short of the
// cell centre and the path carries a 39 mm lead-in to match.
//
// IT IS A KNOB BECAUSE THE RUN DATA DISAGREES WITH THE GEOMETRY. bert, run
// 111/112: the flight viewer reconstructs the wall cloud by integrating the
// logged (s, yaw) from a start pose, and the cloud lands on the drawn maze when
// that pose is y = 90 -- the CELL CENTRE -- not y = 51. That is a 39 mm
// along-track bias present from the first packet of every run, in the direction
// that puts the mouse FURTHER ALONG than the firmware believes, and nothing
// corrects it: the dense path sets path_front_ref = false and corner_count = 0,
// so there are no gates, and the along-track landmark gate only fires where a
// wall across the direction of travel is in view.
//
// The firmware side is NOT the bug -- measured on the host, begin_route_diag
// with lead_in 39 puts path s=0 at path y 0 and the first cell centre at path
// y 39, and Search adds maze_dy = 51, so s=0 lands on maze y 51 and the cell
// centre on maze y 90 exactly as intended. The number 51 is the thing in
// question, so it is the thing that moves.
//
// Setting this rebuilds nothing by itself: the value is read when the path is
// COMPILED, so set it before the run.
static constexpr float SEARCH_START_Y_DEFAULT_MM = 51.0f;
static float g_start_y_mm = SEARCH_START_Y_DEFAULT_MM;
static inline float search_start_setback_mm() {
    return 0.5f * SEARCH_CELL_MM - g_start_y_mm;
}
void search_set_start_y_mm(float mm) {
    if (mm < 0.0f) mm = 0.0f;
    if (mm > SEARCH_CELL_MM) mm = SEARCH_CELL_MM;
    g_start_y_mm = mm;
}
float search_start_y_mm() { return g_start_y_mm; }
static constexpr float SEARCH_SHUFFLE_MIN_MM = 5.0f;   // below: not worth it
// (was 8: with the 8 mm settle tolerance, stop errors of 5-8 mm must be
// corrected whenever a front wall offers ground truth, or they stand --
// hardware 2026-07-10: 5.8 mm slide left the mouse visibly near the wall.)
// MAX was 40: in-place 180s WALK the mouse under scrub (encoders blind to
// it -- wheels counter-rotate, slip-gated), and the goal-cell turn walked
// ~40 mm on the new rig; the next stop's honest 43 mm correction was then
// ruled "implausible" and aborted the mission (2026-07-10). A genuinely
// lost cell estimate is ~180 mm off; 60 leaves a decade of daylight
// between turn-walk and true confusion.
static constexpr float SEARCH_SHUFFLE_MAX_MM = 60.0f;  // above: lost -> abort
static constexpr float SEARCH_SHUFFLE_V = 150.0f;      // gentle
static constexpr float SEARCH_FRONT_PANIC_MM = 55.0f;  // last-resort guard
// Continuous front-wall engagement while flying. The per-cell decision
// checks the front beam exactly ONCE, at a range (~187 mm) right at the
// 200 mm presence threshold -- one blink and the leg extends into a wall
// with nothing left but the panic guard, which at 55 mm cannot out-brake
// 300 mm/s (64 mm needed; hardware 2026-07-10: wall contact at (1,2)).
// So: EVERY packet, a front wall seen within ENGAGE range (debounced
// SEARCH_FRONT_ENGAGE_N consecutive packets) arms the stop; and if the
// current plan's endpoint overruns the wall-implied stop by more than
// SEARCH_FRONT_OVERRUN_MM, brake immediately -- the stop lands short of
// the cell centre and the stationary shuffle recovers the offset.
static constexpr float SEARCH_FRONT_ENGAGE_MM = 170.0f;
// Packet count at the 616 Hz stream (was 2 at the old 50 Hz = 40 ms;
// 24 keeps the same real debounce time -- 2 packets at 616 Hz is 3 ms,
// single-flicker territory).
static constexpr int SEARCH_FRONT_ENGAGE_N = 24;
// 45, not 20: the single-beam front under-reads by ~30 mm near its range
// limit (2026-07-10: a wall truly 195 mm ahead read ~166, tripping a FALSE
// overrun brake mid-corridor). A REAL missed-extend overrun is a whole
// cell (180 mm), so 45 keeps a decisive gap between beam error and
// genuine emergency.
static constexpr float SEARCH_FRONT_OVERRUN_MM = 45.0f;

// Boot-align leg (folded-in 'f', same numbers): sample gating for the
// WallAlign fit. The rig's start corridor has a continuous left wall across
// (0,0)->(0,1), so the fit always has clean geometry.
static constexpr float SEARCH_ALIGN_STEP_MM = 4.0f;   // min sample spacing
static constexpr float SEARCH_ALIGN_YAW_GATE = 1.5f;  // fit only on-reference
static constexpr float SEARCH_PRETURN_DEG = 1.0f;

// In-move side-wall evidence window (mm of leg distance). The side beams
// sweep the ENTERED cell's side walls through this stretch at many
// longitudinal positions -- position diversity defeats the systematic
// misreads a single stationary burst cannot (hardware 2026-07-10: phantom
// N+S walls at (1,4) from beams on the far corner posts).
//
// Window placement is set by the DOCUMENTED beam geometry (sensor sub-MCU
// source, laser_* tables): side lasers mount 40 mm forward, +/-15 mm
// lateral, at +72.8 / -71.0 deg. To a wall face 82.5 mm off-centre the
// beam adds ~30-34 mm forward over ~97 mm lateral, so the HIT lands
// ~70-74 mm AHEAD of the mouse centre. The entered cell's side walls span
// leg-d 90..270 with 12 mm posts at each end; with ~25 mm post margins
// the hits must land in ~120..240, i.e. the MOUSE must be at leg-d
// ~50..165. (This also convicts the stationary burst: parked at a cell
// centre the side beams strike ~18 mm from the FAR boundary -- on the
// next corner's posts, which exist whether walls do or not.)
// When >= SEARCH_SIDE_MIN_N samples exist, this evidence OVERRIDES the
// stationary burst for left/right; front stays with the stationary burst
// (near-axial beams, geometry-independent).
static constexpr float SEARCH_SIDE_WIN_LO_MM = 50.0f;
static constexpr float SEARCH_SIDE_WIN_HI_MM = 165.0f;
static constexpr int SEARCH_SIDE_MIN_N = 5;

// Continuous legs: the extend-or-brake decision for cell j happens this
// many mm before j's centre. The margin must satisfy BOTH sides: late
// enough for real side evidence, early enough to brake. At 700 mm/s^2 the
// decel needs 64 mm (starts 116 mm before the centre); deciding at 104
// (margin 76) leaves ~2 packets of slack AND ~9 side-window samples
// (window opens at 50, packets every ~6 mm). History: margin 105 decided
// at d=75 with only ~4 samples -- below SEARCH_SIDE_MIN_N, so corridors
// were judged on OPTIMISTIC sides ("maybe an opening, turn") and the
// mouse braked every cell (hardware 2026-07-10, two runs).
// 92, not 76: the decision triggers on PHYSICAL distance but the profile's
// decel gate runs on PROFILE time, and the plant lags the profile ~10 mm
// at cruise -- with the stop trim shortening the first cell to 174, every
// extension at d=104 arrived after t_decel and was refused ("decel
// committed"), so corridors still stopped per cell (2026-07-10 telemetry:
// d=104 at t=620 ms vs decel start t=583 ms). At 92 the decision runs
// ~25 ms of profile time before the decel with ~6 side samples in hand.
static constexpr float SEARCH_DECIDE_MARGIN_MM = 92.0f;

// (SEARCH_TURN_TRIM_DEG retired 2026-07-10: superseded by Control's
// closed-loop turn deceleration planner -- see control_turn_start.)

// Hand-start (armed at power-on): a hand held under HAND_START_MM in front
// of the front beam (debounced) triggers the run. The mouse waits for the
// hand to CLEAR (above HAND_CLEAR_MM), then SEARCH_HAND_SETTLE_MS of
// stillness, then re-runs the IMU gyro calibration -- so the bias window
// runs after the last human touch, not at power-on mid-handling -- then
// starts the mission. The sensors stream in INTERLACED FREE-RUN (~616 Hz)
// whenever the state machine is live -- armed states included (the old
// 10/50 Hz triggered cadences died with the disproven thermal cap,
// 2026-07-19).
// The hand thresholds are start-protocol constants only -- entirely
// separate from wall mapping (the armed states never write to the Maze).
// By the rules the start cell's exit is open straight ahead, so an armed
// mouse at the start never has a wall inside beam range: anything the
// front beam sees while armed IS a hand. CLEAR must exceed START so the
// trigger and clear debounces cannot fight; a removed hand normally reads
// as no return at all (255).
static constexpr float SEARCH_HAND_START_MM = 100.0f;
// Once a gesture is live, a HYSTERESIS band: anything nearer than this still
// counts as the hand. A bare hand is a poor diffuse target for a laser
// triangulation sensor and it does not hold still, so a single threshold that
// is right for triggering is far too tight for holding. CLEAR must exceed
// HOLD so the two cannot fight.
static constexpr float SEARCH_HAND_HOLD_MM = 140.0f;
static constexpr float SEARCH_HAND_CLEAR_MM = 150.0f;
// Packet counts at the 616 Hz stream. The trigger debounce is SHORT (~50 ms)
// because it now only has to reject a stray packet: the gesture is no longer
// "was the hand there at all" but "how long was it there", and that is timed
// from the moment the debounce passes. A 300 ms debounce would have eaten a
// third of a sub-second wave.
static constexpr int SEARCH_HAND_TRIGGER_N = 30;
static constexpr int SEARCH_HAND_CLEAR_N = 300;
static constexpr uint32_t SEARCH_HAND_SETTLE_MS = 1500;

// Hold the hand under SEARCH_HAND_START_MM for this long and the map is
// WIPED (bert, 2026-08-13): short gesture = run, long gesture = forget the
// maze and search it again. Anything removed sooner is a RUN -- see the
// gesture note in Search.h for why the boundary is that way round.
static constexpr uint32_t SEARCH_HAND_WIPE_MS = 5000;

// The hand is GONE only after this long unseen. THE BUG THIS FIXES (bert,
// 2026-08-14: "the >5s hand isn't working, it only does the speed run"): the
// release debounce was SEARCH_HAND_TRIGGER_N packets, which at the 616 Hz
// stream is 49 ms. Any dropout longer than that ended the gesture, so a
// 5-second hold became a speed run essentially every time -- and a speed run
// is the one outcome you did not want when you were asking for a wipe. The
// hold is now dropout-tolerant: elapsed time is counted from the FIRST
// sighting and brief losses do not reset it.
static constexpr uint32_t SEARCH_HAND_LOST_MS = 400;

// NOTE there is deliberately no post-mission re-arm delay here any more.
// ARMING HAPPENS AT POWER-ON ONLY (bert, 2026-08-13). Once the map persists,
// the natural thing between runs is to switch the mouse off and clean the
// wheels, so a mission ending leaves the mouse IDLE and the pick-up-looks-
// like-a-wave hazard disappears with it. 'h' still arms by hand on the bench.

static constexpr uint32_t SEARCH_SETTLE_MS = 150;
// Free-run keepalive: re-send CMD_FREE_RUN_IL periodically so a dropped
// command byte cannot silently starve a whole mission of packets
// (idempotent on the sub-MCU).
static constexpr uint32_t SEARCH_SENSOR_KEEPALIVE_MS = 1000;
static constexpr int SEARCH_SENSE_PACKETS = 8;      // stationary burst
static constexpr uint32_t SEARCH_SENSE_TIMEOUT_MS = 1000;
static constexpr int SEARCH_STEP_BUDGET = 2000;   // 400 until 2026-10-02 (16x16)

// -- State -------------------------------------------------------------------------

// PHASE_EXPLORE sits between them when explore-all is on: reach the goal,
// then sweep up every reachable unknown edge, then come home. Ordering
// matters -- going to the goal FIRST means a flat battery still leaves a
// proven route, which is the outcome that matters most.
enum SearchPhase : uint8_t { PHASE_TO_GOAL, PHASE_EXPLORE, PHASE_TO_START };

enum SearchState : uint8_t {
    S_IDLE,
    S_ARMED,         // hand-start: watching the front beam
    S_HAND_CLEAR,    // hand seen: wait for it to be removed
    S_CAL,           // stillness delay, then IMU recal, then mission
    S_ALIGN_TURN,    // pre-turn to yaw 0 before the boot-align leg
    S_ALIGN_SETTLE,  // pause after the pre-turn
    S_ALIGN,         // boot-align leg (0,0)->(0,1): WallAlign fit, NO servo
    S_SETTLE,        // stationary pause before sensing
    S_SENSE,         // packet burst -> maze update -> decision
    S_TURN,          // in-place turn to the chosen cardinal
    S_TURN_SETTLE,   // tyres relax after the turn, before the straight
    S_SHUFFLE,       // short front-referenced corrective straight
    S_MOVE,          // one-cell straight, lateral servo live
};

static void (*prn)(const char *) = nullptr;
static void printf_line(const char *fmt, ...) {
    if (!prn) return;
    // 192, not 128: the divergence and side-sum lines both land in the
    // 110-120 char range, which fit only by luck. Silent truncation is the
    // worst possible failure for a line whose whole job is to explain a fault.
    char buf[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    prn(buf);
}

// V2 runs the SEARCH mission only. V1 carried a second mode here that
// executed a compiled speed path; it is not in this baseline (see
// docs/V2_BASELINE.md). The fast run gets rebuilt on top of this, as its
// own module, once the search is proven end to end again.
static int leg_cells = 1;        // straight length for the next leg

static Maze maze;
static SearchState state = S_IDLE;
static SearchPhase phase = PHASE_TO_GOAL;
static int cell_x = 0, cell_y = 0;
static Heading heading = HEADING_N;
static Heading pending_step = HEADING_N;  // decided in SENSE, executed after
static int steps = 0;
static uint32_t state_t0 = 0;
static uint32_t last_trigger = 0;
static uint32_t last_seq = 0;

// SENSE burst accumulators.
static int sense_n = 0;
static int sense_left = 0, sense_right = 0, sense_front = 0;
static float sense_front_sum = 0.0f;
static int sense_front_n = 0;

// Boot-align leg state.
static WallAlign align;
static float align_yaw0 = 0.0f;
static float align_last_d = 0.0f;

// In-move side-wall evidence (see SEARCH_SIDE_WIN_* above).
static int side_n = 0, side_left = 0, side_right = 0;

static void side_reset() { side_n = side_left = side_right = 0; }

static void side_sample(float d_mm, const WallView &wv) {
    if (!wv.valid) return;
    if (d_mm < SEARCH_SIDE_WIN_LO_MM || d_mm > SEARCH_SIDE_WIN_HI_MM) return;
    side_n++;
    if (wv.left) side_left++;
    if (wv.right) side_right++;
}

// Continuous-leg state. A leg is one straight of 1..N cells from origin
// (leg_ax, leg_ay). Three independent trackers walk the leg:
//   fly_w       cell index whose side window is being accumulated
//   fly_dec     cell index of the next extend-or-brake decision
//   fly_entered cells whose entry boundary has been crossed (bookkeeping:
//               traversed-edge lock, cell position, step budget)
// Counters are reset lazily on the next window's first sample, so at a
// stop they still hold the stop cell's full window for sense_decide.
static int fly_w = 1;
static int fly_dec = 1;
static int fly_entered = 0;
static int leg_ax = 0, leg_ay = 0;
static bool fly_stop_planned = false;  // decision said brake: no more extends
static bool side_pending_reset = false;
static int front_engage_run = 0;       // consecutive packets with front seen
                                       // inside SEARCH_FRONT_ENGAGE_MM

// Debug stream + per-leg diagnostics (see Search.h).
static bool dbg_stream = false;
static uint32_t dbg_last = 0;
static float leg_odo0 = 0.0f;      // wheel odometry at leg start
static uint32_t leg_slip0 = 0;     // slip-event count at leg start

static void dbg_tick(uint32_t now) {
    if (!dbg_stream || now - dbg_last < 20) return;
    dbg_last = now;
    ControlMoveStatus st = control_move_status();
    ControlOdometry o = control_odometry();
    ControlMotion m = control_motion();
    WallRanges w = sensors_latest();
    WallView wv = walls_evaluate(w, sensors_ms_since_packet());
    float d = st.d_mm;
    float ye = st.yaw_err_deg;
    float col8 = st.lat_bias_deg;
    printf_line("D,%lu,%d,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%u,%u,%u,%u,"
                "%.1f,%.1f",
                (unsigned long)now, (int)state, d, o.vel_mm_s,
                m.v_est_mm_s, control_heading().yaw_deg, ye, col8,
                m.slip ? 1 : 0, w.left_mm, w.front_left_mm,
                w.front_right_mm, w.right_mm,
                wv.front ? wv.front_mm : -1.0f,
                wv.lateral_valid ? wv.lateral_mm : -99.9f);
}

// One line per completed leg: profile target vs fused distance vs raw
// wheel odometry, end heading error, slip events. The shuffle line that
// may follow is the WALL's verdict on the same stop -- three independent
// yardsticks for the overshoot hunt.
static void leg_summary(const ControlMoveStatus &st) {
    ControlOdometry o = control_odometry();
    ControlMotion m = control_motion();
    printf_line("leg: prof %.0f fused %.1f wheels %.1f | yaw_err %+.2f | "
                "slip %lu",
                st.d_ref, st.d_mm, o.dist_mm - leg_odo0, st.yaw_err_deg,
                (unsigned long)(m.slip_events - leg_slip0));
}

// -- Helpers -----------------------------------------------------------------------

// Maze north = IMU yaw 0 ('f' in the start corridor first). East = -90
// (turning right is CW = negative yaw in the fusion frame).
static float cardinal_yaw_deg(Heading h) {
    switch (h) {
        case HEADING_N: return 0.0f;
        case HEADING_E: return -90.0f;
        case HEADING_S: return 180.0f;
        default:        return 90.0f;  // W
    }
}

static float wrap180f(float a);  // defined below

static float wrap180f(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static const char *heading_name(Heading h) {
    static const char *names[4] = {"N", "E", "S", "W"};
    return names[h];
}

static void stop_everything() {
    // Brake before cutting -- zero duty is coast. See
    // control_brake_to_stop(); run 151 coasted into a wall.
    control_brake_to_stop();
    control_move_abort();
    control_velocity_enable(false);
    sensors_stop();
    flog_stop();  // recorder freezes with the mission ('y' dumps it)
    state = S_IDLE;
}

static int hand_run = 0;            // debounce counter (trigger and clear)
static bool hand_cal_started = false;
static uint32_t hand_t0 = 0;        // millis the hand passed the debounce
static uint32_t hand_seen_ms = 0;   // millis of the last sighting
static bool hand_active = false;    // a gesture is in progress
static uint32_t hand_next_tick = 0; // next countdown line to print
static bool hand_wiped = false;     // the 5 s wipe already fired this gesture
static StartGate &start_gate = start_gate_shared();   // both side walls steady before a wave counts

// Interlaced free-run everywhere (2026-07-19): 616 Hz bench-proven, no
// thermal limit, so the whole state machine -- armed, search, sprint,
// return -- rides one continuous stream. (History: the sprint flew its
// first weeks on a 50 Hz TRIGGERED cadence nobody had switched over --
// 9 mm between packets at 450 mm/s, run-to-run scatter to match.)
// This tick is now only a KEEPALIVE: the re-send is idempotent on the
// sub-MCU and rescues a dropped command byte instead of letting it
// silently starve a mission. stop_everything still sends CMD_STOP.
static void trigger_tick(uint32_t now) {
    if (now - last_trigger >= SEARCH_SENSOR_KEEPALIVE_MS) {
        last_trigger = now;
        sensors_free_run_interlaced();
    }
}

// One-shot extension of the next stationary settle (goal dead-stop pause).
static uint32_t settle_extra_ms = 0;

static void enter_settle(uint32_t now) {
    state = S_SETTLE;
    state_t0 = now;
}

// -- Mission bookkeeping -------------------------------------------------------------

// FULL EXPLORATION. A goal search stops as soon as a route is proven, which
// routinely leaves a third of the maze unseen. Set this and the mission adds
// a sweep that keeps going until no reachable edge is unknown -- which is
// what a mouse that will later fly a route it discovered itself needs, and
// what an enclosed 6x6 rig needs if the map is to be complete.
static bool explore_all = SEARCH_EXPLORE_ALL_DEFAULT;

void search_set_explore_all(bool on) { explore_all = on; }
bool search_explore_all() { return explore_all; }

static bool at_phase_target() {
    if (phase == PHASE_TO_GOAL) return maze.in_goal(cell_x, cell_y);
    if (phase == PHASE_EXPLORE) {
        // "Done" is NOT "every wall known" -- walls sealed outside an
        // enclosed sub-maze can never be resolved and waiting for them would
        // never terminate. It is "nothing unknown that can be REACHED",
        // which is the question the flood can actually answer.
        return !maze.useful_unknown_reachable(cell_x, cell_y, EX_ROUTE_SLACK_CELLS);
    }
    return cell_x == 0 && cell_y == 0;
}

static void flood_for_phase() {
    if (phase == PHASE_TO_GOAL) maze.flood_to_goal(true);
    else if (phase == PHASE_EXPLORE) maze.flood_to_useful_unknown(EX_ROUTE_SLACK_CELLS);
    else maze.flood_to_start(true);
}

// Cells still holding an unknown edge, for the progress line. Cheap, and it
// is the number that tells you whether a session is converging or stuck.
static int unknown_cells() {
    int n = 0;
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            if (maze.has_unknown_wall(x, y)) n++;
    return n;
}

static void finish_mission() {
    stop_everything();
    printf_line("-- search complete: back at start, %d steps total --",
                steps);
    // PERSIST THE MAP HERE, not after the route test: a partial map is worth
    // keeping too. It means the next power-up RESUMES the search instead of
    // starting blind, and a search mission always ends back at the start, so
    // the mouse and the saved map agree about where it is standing.
    search_map_save();
    maze.flood_to_goal(false);  // pessimistic: fully-known route?
    uint8_t d = maze.distance(0, 0);
    if (d != MAZE_DIST_UNREACHABLE) {
        printf_line("known route start->goal: %u cells", d);
        search_dump_map();
        // Default: STOP. The map is in FRAM now, so the run that counts
        // should start from a clean power-on with clean wheels rather than
        // inherit whatever the search left on the tyres (bert, 2026-08-13).
        printf_line("-- IDLE. Clean the wheels, power-cycle, wave: the map "
                    "is saved and the next wave is a SPEED RUN --");
        return;
    }
    printf_line("no fully-known route yet -- another search pass needed");
    search_dump_map();
    printf_line("-- IDLE. Power-cycle to arm; the partial map is saved, so "
                "the next search RESUMES --");
}

// -- State machine ------------------------------------------------------------------

// Boot-align leg: (0,0) -> (0,1) straight with the WallAlign fit running
// and the lateral servo OFF (the fit assumes a straight path). On success
// the IMU is re-referenced so the corridor axis reads exactly 0.
static void start_align_leg() {
    align.reset();
    control_motion_zero();
    side_reset();
    align_yaw0 = control_heading().yaw_deg;
    align_last_d = -1e9f;
    last_seq = sensors_latest().seq;
    // One cell PLUS the start setback: the mouse begins with its back to the
    // rear wall (SEARCH_START_Y_MM = 51), not at the cell centre, so a bare
    // SEARCH_CELL_MM would stop it the start setback short of the centre
    // of (0,1) and hand that error to every cell after it. The speed run needs
    // no equivalent -- it has no boot-align leg and its path carries the
    // setback as a lead-in instead (see SEARCH_START_Y_MM).
    if (!control_move_start(SEARCH_CELL_MM + search_start_setback_mm(),
                            SEARCH_V, SEARCH_ACC)) {
        search_abort("align move refused (fusion not ready?)");
        return;
    }
    state = S_ALIGN;
}

static void align_finish(uint32_t now) {
    if (!align.valid()) {
        printf_line("align fit: n=%d rejected=%d span=%.0f mm",
                    align.samples(), align.rejected(), align.span_mm());
        search_abort("boot align fit invalid -- left wall continuous?");
        return;
    }
    float eps = align.yaw_err_deg();
    if (fabsf(eps) > WALL_ALIGN_MAX_CORR_DEG) {
        printf_line("align fit: %+.2f deg", eps);
        search_abort("boot align correction implausible");
        return;
    }
    // Wall direction in the current IMU frame is (yaw0 - eps); subtract it
    // so the corridor reads 0 (same law as the standalone 'f').
    float yaw_now = control_heading().yaw_deg;
    control_set_heading_deg(yaw_now - (align_yaw0 - eps));
    printf_line("boot align: heading err %+.2f deg -> corridor = 0", eps);
    maze.set_wall_traversed(0, 0, HEADING_N);  // just drove through it
    cell_x = 0;
    cell_y = 1;  // the align leg ends one cell up the corridor
    heading = HEADING_N;
    // Disarm the late-boundary check until a real leg arms it: the align
    // leg's d frame reads ~185 at its stationary sense (~224 since the leg
    // gained the start setback), and stale fly state let the check
    // register a PHANTOM crossing to (0,2) -- the whole mission then
    // navigated one cell north of reality (2026-07-10). The longer leg makes
    // that disarm MORE necessary, not less.
    fly_dec = 0;
    fly_entered = 0;
    enter_settle(now);
}

// Start a leg. SEARCH: one cell, extended in flight while the flood keeps
// pointing straight (see S_MOVE / fly_decide). SPEED outbound: the whole
// route is ONE continuous path (arcs, no stops) -- divert to it. SPEED
// return: leg_cells was pre-computed; whole straight, one profile.
static void start_move_one_cell() {
    control_motion_zero();
    side_reset();
    leg_cells = 1;
    fly_w = 1;
    fly_dec = leg_cells;
    fly_entered = 0;
    leg_ax = cell_x;
    leg_ay = cell_y;
    fly_stop_planned = false;
    side_pending_reset = false;
    front_engage_run = 0;
    leg_odo0 = control_odometry().dist_mm;
    leg_slip0 = control_motion().slip_events;
    float v = SEARCH_V;
    // ABSOLUTE cardinal reference: the rolling heading loop absorbs the
    // previous turn's undershoot; the lateral servo keeps its full clamp
    // for actual centring (see control_move_start_heading). Profile aims
    // SEARCH_STOP_TRIM_MM short; the braking overshoot lands on centre.
    if (!control_move_start_heading(
            SEARCH_CELL_MM * leg_cells - SEARCH_STOP_TRIM_MM, v, SEARCH_ACC,
            cardinal_yaw_deg(heading))) {
        search_abort("move refused (fusion not ready?)");
        return;
    }
    last_seq = sensors_latest().seq;
    state = S_MOVE;
}

// Commit the completed side window for cell fly_w into the maze.
static void fly_commit_walls() {
    int tx = leg_ax + fly_w * heading_dx(heading);
    int ty = leg_ay + fly_w * heading_dy(heading);
    if (side_n >= SEARCH_SIDE_MIN_N) {
        bool l = side_left * 2 > side_n;
        bool r = side_right * 2 > side_n;
        maze.set_wall(tx, ty, heading_left(heading), l);
        maze.set_wall(tx, ty, heading_right(heading), r);
        maze.set_visited(tx, ty);
        printf_line("fly (%d,%d) %s | L%d R%d | mv %d/%d/%d | step %d", tx,
                    ty, heading_name(heading), l, r, side_left, side_right,
                    side_n, steps);
    }
    fly_w++;
    side_pending_reset = true;  // reset on the next window's first sample
}

// Extend-or-brake decision for cell fly_dec (the current profile target).
static void fly_decide(float d, const WallView &wv) {
    int tx = leg_ax + fly_dec * heading_dx(heading);
    int ty = leg_ay + fly_dec * heading_dy(heading);

    // Phase target inside this cell: stop there.
    bool target = (phase == PHASE_TO_GOAL) ? maze.in_goal(tx, ty)
                                           : (tx == 0 && ty == 0);
    if (target) { fly_stop_planned = true; return; }

    // Front wall belonging to cell fly_dec: stop at its centre.
    if (wv.valid && wv.front &&
        d + wv.front_mm < SEARCH_CELL_MM * fly_dec + 120.0f) {
        fly_stop_planned = true;
        return;
    }

    // Best-effort partial side evidence (refined at window commit).
    if (side_n >= SEARCH_SIDE_MIN_N) {
        maze.set_wall(tx, ty, heading_left(heading),
                      side_left * 2 > side_n);
        maze.set_wall(tx, ty, heading_right(heading),
                      side_right * 2 > side_n);
    }

    flood_for_phase();
    Heading next;
    if (!maze.best_step(tx, ty, heading, next) || next != heading) {
        fly_stop_planned = true;  // turn (or dead end) at fly_dec: brake
        return;
    }
    if (!control_move_extend(SEARCH_CELL_MM)) {
        fly_stop_planned = true;  // decel already committed
        return;
    }
    fly_dec++;  // rolling on: next decision one cell later
}

static void start_turn_or_move(uint32_t now) {
    float want = cardinal_yaw_deg(pending_step);
    float angle = wrap180f(want - control_heading().yaw_deg);
    if (fabsf(angle) > SEARCH_TURN_SKIP_DEG) {
        // No trim: turns now run Control's closed-loop deceleration
        // planner (move_tick), which re-plans the decel every tick from
        // MEASURED remaining angle and lands within MOVE_TURN_FINISH_DEG
        // regardless of scrub variance. (The open-loop era needed a
        // +3.5 deg aim-past on 90s; with the planner that would be a
        // deliberate overshoot.)
        if (!control_turn_start(angle, SEARCH_TURN_OMEGA, SEARCH_TURN_ALPHA)) {
            search_abort("turn refused (fusion not ready?)");
            return;
        }
        state = S_TURN;
        state_t0 = now;
        return;
    }
    heading = pending_step;
    start_move_one_cell();
}

static void sense_decide(uint32_t now) {
    // Late boundary bookkeeping: an engage-brake jumps to SETTLE and the
    // mouse can COAST across a cell boundary with no S_MOVE polls running
    // (2026-07-10: braked at d=75, coasted to d=92 past the 90 mm
    // boundary; the stale cell estimate then failed the front sanity
    // check). The leg's d frame is valid until the next motion_zero, so
    // register any missed crossings now -- normal stops are a no-op here.
    {
        ControlMoveStatus st = control_move_status();
        while (fly_entered < fly_dec &&
               st.d_mm >= SEARCH_CELL_MM * (fly_entered + 1) -
                              0.5f * SEARCH_CELL_MM) {
            maze.set_wall_traversed(cell_x, cell_y, heading);
            cell_x += heading_dx(heading);
            cell_y += heading_dy(heading);
            fly_entered++;
            steps++;
        }
    }

    bool left = sense_left * 2 > SEARCH_SENSE_PACKETS;
    bool right = sense_right * 2 > SEARCH_SENSE_PACKETS;
    bool front = sense_front * 2 > SEARCH_SENSE_PACKETS;
    float front_mm =
        sense_front_n ? sense_front_sum / sense_front_n : WALL_FRONT_NONE_MM;

    // Position-diverse in-move evidence overrides the stationary burst for
    // the SIDE walls when available (see SEARCH_SIDE_WIN_*).
    if (side_n >= SEARCH_SIDE_MIN_N) {
        left = side_left * 2 > side_n;
        right = side_right * 2 > side_n;
    }

    // Body-relative -> absolute walls; opening confirmations matter as much
    // as walls (they unblock the flood). Locked (traversed) edges ignore
    // these writes by design.
    maze.set_wall(cell_x, cell_y, heading, front);
    maze.set_wall(cell_x, cell_y, heading_left(heading), left);
    maze.set_wall(cell_x, cell_y, heading_right(heading), right);
    maze.set_visited(cell_x, cell_y);
    printf_line("cell (%d,%d) %s | walls L%d F%d R%d | mv %d/%d/%d | "
                "step %d", cell_x, cell_y, heading_name(heading), left,
                front, right, side_left, side_right, side_n, steps);

    if (at_phase_target()) {
        if (phase == PHASE_TO_GOAL) {
            phase = explore_all ? PHASE_EXPLORE : PHASE_TO_START;
            printf_line("-- GOAL reached in %d steps -- %s --", steps,
                        explore_all ? "exploring the rest" : "returning to "
                                                             "start");
            if (explore_all)
                printf_line("   %d cells still hold an unknown wall",
                            unknown_cells());
            // Stop dead in the goal cell: a deliberate pause, then re-sense
            // and decide what comes next.
            settle_extra_ms = 600;
            enter_settle(now);
            return;
        }
        if (phase == PHASE_EXPLORE) {
            phase = PHASE_TO_START;
            printf_line("-- MAP COMPLETE in %d steps: no reachable unknown "
                        "wall left (%d cells unknown but sealed off) -- "
                        "returning to start --",
                        steps, unknown_cells());
            settle_extra_ms = 600;
            enter_settle(now);
            return;
        }
        finish_mission();
        return;
    }
    if (phase == PHASE_EXPLORE && (steps % 10) == 0)
        printf_line("   explore: %d cells still unknown", unknown_cells());

    // (Step counting moved to the in-flight boundary crossings.)

    Heading next;
    flood_for_phase();
    if (!maze.best_step(cell_x, cell_y, heading, next)) {
        search_abort("no route (flood blocked)");
        return;
    }
    pending_step = next;

    // Longitudinal re-reference off a visible front wall (see Search.h).
    if (front && front_mm < WALL_FRONT_NONE_MM) {
        float err = front_mm - SEARCH_FRONT_TARGET_MM;
        if (fabsf(err) > SEARCH_SHUFFLE_MAX_MM) {
            printf_line("front %.1f mm vs %.1f expected", front_mm,
                        SEARCH_FRONT_TARGET_MM);
            search_abort("front distance implausible -- cell estimate lost?");
            return;
        }
        if (fabsf(err) >= SEARCH_SHUFFLE_MIN_MM) {
            printf_line("front re-ref: %+.1f mm shuffle", err);
            control_motion_zero();
            if (!control_move_start(err, SEARCH_SHUFFLE_V, SEARCH_ACC)) {
                search_abort("shuffle refused");
                return;
            }
            state = S_SHUFFLE;
            return;
        }
    }
    start_turn_or_move(now);
}

void search_poll() {
    if (state == S_IDLE) return;
    uint32_t now = millis();
    trigger_tick(now);
    sensors_poll();
    dbg_tick(now);

    switch (state) {
        case S_ARMED: {
            // THE PLACEMENT GATE. This is where the mouse actually sits while
            // bert positions it after a power-on -- S_SPEED_ARMED, where this
            // aid was first put, is a SECOND gate reached only after the first
            // wave. bert, 2026-08-15: "the datum still only starts after a
            // hand start", which is exactly that mistake seen from outside.
            //
            // Running it in both is deliberate rather than untidy: whichever
            // armed state the mouse was in last before launching is the one
            // whose measurement gets used, so if anything moves the mouse
            // between the two gates the later measurement simply wins.
            WallRanges w = sensors_latest();
            // THE START GATE (StartGate.h): both side walls steady, or the
            // wave does not count -- a mouse in the hand, or a sensor that
            // came up wrong after a quick power cycle. The LEDs alternate
            // while it is not ready; LED1 steady says "ready for the wave".
            start_gate.tick(sensors_ms_since_packet(), now);
            if (!start_gate.ready && !hand_active) {
                bool p1 = StartGate::led1_phase(now);
                hal_set_led1(p1);
                hal_set_led2(!p1);
                if (start_gate.time_to_say(now))
                    printf_line("start gate: side walls not both seen (L %u R %u%s) -- "
                                "LEDs alternate until the mouse sits in the start cell "
                                "reading both. Still alternating on the floor = the "
                                "sensor fault: power off, wait a few seconds, power on.",
                                start_gate.last_l, start_gate.last_r,
                                sensors_ms_since_packet() > START_GATE_STALE_MS
                                    ? ", NO PACKETS" : "");
            }
            if (w.seq != last_seq) {
                last_seq = w.seq;
                WallView wv = walls_evaluate(w, sensors_ms_since_packet());
                if (start_gate.packet(w.left_mm, w.right_mm, now)) {
                    hal_set_led1(true);
                    hal_set_led2(false);
                    printf_line("start gate: side walls L %u R %u -- ready for the wave",
                                w.left_mm, w.right_mm);
                }
                if (!start_gate.ready && !hand_active) {
                    // Not on the floor in the start cell: no gesture begins.
                    hand_run = 0;
                    return;
                }
                if (hand_active && !hand_wiped && w.left_mm == 255 && w.right_mm == 255) {
                    // Lifted mid-gesture (both walls gone; a hand in front
                    // may nudge one side beam, which is not a lift): drop it.
                    hand_active = false;
                    hand_run = 0;
                    printf_line("hand start cancelled -- both side walls gone (picked up?)");
                    return;
                }
                // Schmitt trigger: START_MM to begin a gesture, the wider
                // HOLD_MM to keep one alive.
                float lim = hand_active ? SEARCH_HAND_HOLD_MM
                                        : SEARCH_HAND_START_MM;
                if (wv.valid && wv.front && wv.front_mm < lim) {
                    if (hand_run < SEARCH_HAND_TRIGGER_N) hand_run++;
                    if (hand_run >= SEARCH_HAND_TRIGGER_N && !hand_active) {
                        hand_active = true;
                        hand_t0 = now;
                        hand_wiped = false;
                        hand_next_tick = 1;
                        printf_line("hand seen -- remove to run, or hold "
                                    "%lu s to WIPE the map",
                                    (unsigned long)
                                    (SEARCH_HAND_WIPE_MS / 1000));
                    }
                    if (hand_active) hand_seen_ms = now;
                } else if (!hand_active) {
                    hand_run = 0;
                }
            }
            if (!hand_active) return;

            // Everything below runs on the MILLIS clock, not the packet
            // clock, so the answer does not depend on the stream rate.
            uint32_t held = now - hand_t0;

            // Countdown, so a hold is visible while it is happening rather
            // than only in its outcome.
            if (!hand_wiped && hand_next_tick * 1000 <= held &&
                hand_next_tick * 1000 < SEARCH_HAND_WIPE_MS) {
                printf_line("   holding %lu s -- %lu more to WIPE",
                            (unsigned long)hand_next_tick,
                            (unsigned long)(SEARCH_HAND_WIPE_MS / 1000
                                            - hand_next_tick));
                hand_next_tick++;
            }

            // Fire the wipe AT the mark with the hand still there, so the
            // mouse setting off on a SEARCH is the confirmation it took.
            if (!hand_wiped && held >= SEARCH_HAND_WIPE_MS) {
                hand_wiped = true;
                search_map_wipe();
            }

            // Gesture over only after a real absence, not a flicker.
            if (now - hand_seen_ms >= SEARCH_HAND_LOST_MS) {
                printf_line("hand start: %s (held %lu ms) -- remove hand",
                            hand_wiped ? "MAP WIPED, will search"
                                       : "triggered",
                            (unsigned long)(hand_seen_ms - hand_t0));
                hand_run = 0;
                hand_active = false;
                state = S_HAND_CLEAR;
            }
            return;
        }

        case S_HAND_CLEAR: {
            WallRanges w = sensors_latest();
            if (w.seq != last_seq) {
                last_seq = w.seq;
                WallView wv = walls_evaluate(w, sensors_ms_since_packet());
                bool clear = !wv.front || wv.front_mm > SEARCH_HAND_CLEAR_MM;
                if (wv.valid && clear) {
                    if (++hand_run >= SEARCH_HAND_CLEAR_N) {
                        printf_line("hand clear -- keep the mouse still");
                        hand_run = 0;
                        hand_cal_started = false;
                        state = S_CAL;
                        state_t0 = now;
                    }
                } else {
                    hand_run = 0;
                }
            }
            return;
        }

        case S_CAL:
            if (!hand_cal_started) {
                // Stillness delay after the hand cleared, THEN the recal:
                // the bias window must run untouched.
                if (now - state_t0 < SEARCH_HAND_SETTLE_MS) return;
                control_recalibrate();
                hand_cal_started = true;
                printf_line("IMU calibrating (~3 s, stationary)...");
                return;
            }
            if (!control_heading().ready) return;  // motion guard may extend
            state = S_IDLE;  // the mission start rebuilds the state
            // (free-run keeps streaming straight through into the mission)
            // V2: a wave always starts a SEARCH. V1 asked the map here and
            // handed off to the speed run when a proven route existed; there
            // is no fast run to hand off to yet. Note search_start() calls
            // maze.reset(), so a map restored at boot is NOT resumed -- that
            // was never how V1 used it either (only the sprint read it), and
            // resuming a search from a stored map has never been flown.
            // 2026-09-12: the wave starts the ROLLING search (Explore.h);
            // it recalibrates again itself (a second 3 s, stationary), which
            // is cheap insurance at a race start. 'search.old' on the console
            // is the only way to this file's mission now.
            printf_line("calibrated -- searching (rolling)");
            explore_start(true);
            return;

        case S_ALIGN_TURN: {
            ControlMoveStatus st = control_move_status();
            if (st.active) return;
            if (!st.done) {
                search_abort("align pre-turn did not settle");
                return;
            }
            state = S_ALIGN_SETTLE;
            state_t0 = now;
            return;
        }

        case S_ALIGN_SETTLE:
            if (now - state_t0 < 400) return;
            start_align_leg();
            return;

        case S_ALIGN: {
            ControlMoveStatus st = control_move_status();
            WallRanges w = sensors_latest();
            if (w.seq != last_seq) {
                last_seq = w.seq;
                WallView wv = walls_evaluate(w, sensors_ms_since_packet());
                // Feed the fit exactly like the standalone 'f': fresh
                // packet, walls seen, heading on-reference, 4 mm spacing.
                // NO lateral servo -- the fit needs a straight path.
                if (wv.lateral_valid &&
                    fabsf(st.yaw_err_deg) < SEARCH_ALIGN_YAW_GATE &&
                    st.d_mm - align_last_d >= SEARCH_ALIGN_STEP_MM) {
                    align_last_d = st.d_mm;
                    align.add(st.d_mm, wv.lateral_mm);
                }
                // Evidence for (0,1)'s sides. The window (SEARCH_SIDE_WIN_*)
                // is measured from the LEG START, so it must be shifted back
                // into the old frame now that the leg carries the start
                // setback -- otherwise the same [50,165] window sits 39 mm
                // earlier against the cell grid and samples mostly (0,0):
                //   before  y 140..255  ->  65% on cell (0,1)
                //   naive   y 101..216  ->  31% on cell (0,1)   <- wrong cell
                // That is the first mapping decision of the run, so getting it
                // wrong takes the whole search with it (2026-07-30 regression).
                side_sample(st.d_mm - search_start_setback_mm(), wv);
                if (wv.front && wv.front_mm < SEARCH_FRONT_PANIC_MM) {
                    search_abort("front panic during align leg");
                    return;
                }
            }
            if (st.active) return;
            if (!st.done) {
                printf_line("move end: pos_err %+.1f mm, v %.1f mm/s, "
                            "yaw_err %+.2f deg",
                            st.d_ref - st.d_mm, control_odometry().vel_mm_s,
                            st.yaw_err_deg);
                search_abort("align leg did not settle");
                return;
            }
            align_finish(now);
            return;
        }

        case S_SETTLE:
            if (now - state_t0 < SEARCH_SETTLE_MS + settle_extra_ms) return;
            settle_extra_ms = 0;
            sense_n = 0;
            sense_left = sense_right = sense_front = 0;
            sense_front_sum = 0.0f;
            sense_front_n = 0;
            last_seq = sensors_latest().seq;
            state = S_SENSE;
            state_t0 = now;
            return;

        case S_SENSE: {
            WallRanges w = sensors_latest();
            if (w.seq != last_seq) {
                last_seq = w.seq;
                WallView wv = walls_evaluate(w, sensors_ms_since_packet());
                if (wv.valid) {
                    sense_n++;
                    if (wv.left) sense_left++;
                    if (wv.right) sense_right++;
                    if (wv.front) {
                        sense_front++;
                        sense_front_sum += wv.front_mm;
                        sense_front_n++;
                    }
                }
            }
            if (sense_n >= SEARCH_SENSE_PACKETS) {
                sense_decide(now);
                return;
            }
            if (now - state_t0 > SEARCH_SENSE_TIMEOUT_MS) {
                search_abort("sensor silence (sub-MCU down / trigger cold?)");
            }
            return;
        }

        case S_TURN: {
            ControlMoveStatus st = control_move_status();
            if (st.active) return;
            if (!st.done) {
                printf_line("turn end: yaw_err %+.2f deg, rate %.1f dps",
                            st.yaw_err_deg, st.yaw_rate_dps);
                search_abort("turn did not settle");
                return;
            }
            heading = pending_step;
            state = S_TURN_SETTLE;  // measured from turn END, not start
            state_t0 = now;
            return;
        }

        case S_TURN_SETTLE:
            // Brief pause lets the tyres relax before the straight.
            if (now - state_t0 < 100) return;
            start_move_one_cell();
            return;

        case S_SHUFFLE: {
            ControlMoveStatus st = control_move_status();
            if (st.active) return;
            // A shuffle that times out unsettled is tolerable (it is itself
            // a correction) -- continue either way.
            start_turn_or_move(now);
            return;
        }

        case S_MOVE: {
            ControlMoveStatus st = control_move_status();
            float d = st.d_mm;
            WallRanges w = sensors_latest();
            WallView wv = walls_evaluate(w, sensors_ms_since_packet());

            if (w.seq != last_seq) {  // fresh packet: servo, window, panic
                last_seq = w.seq;
                // Lateral servo: same feed the 'l' test proved.
                if (wv.lateral_valid) {
                    control_set_lateral_bias_deg(-LAT_SERVO_DEG_PER_MM *
                                                 wv.lateral_mm);
                } else {
                    control_set_lateral_bias_deg(0.0f);
                }
                // Side window for cell fly_w (lazy counter reset: at a stop
                // the counters still hold the stop cell's full window for
                // sense_decide's override).
                float rel = d - SEARCH_CELL_MM * (fly_w - 1);
                if (wv.valid && rel >= SEARCH_SIDE_WIN_LO_MM &&
                    rel <= SEARCH_SIDE_WIN_HI_MM) {
                    if (side_pending_reset) {
                        side_reset();
                        side_pending_reset = false;
                    }
                    side_n++;
                    if (wv.left) side_left++;
                    if (wv.right) side_right++;
                }
                if (wv.front && wv.front_mm <= SEARCH_FRONT_ENGAGE_MM) {
                    front_engage_run++;
                } else {
                    front_engage_run = 0;
                }
                if (front_engage_run >= SEARCH_FRONT_ENGAGE_N) {
                    // A real wall ahead (debounced). Arm the stop; if the
                    // current plan overruns the wall-implied stop point (a
                    // missed no-extend at the marginal beam range), brake
                    // NOW -- the shuffle recovers the centre offset.
                    fly_stop_planned = true;
                    float wall_stop =
                        d + wv.front_mm - SEARCH_FRONT_TARGET_MM;
                    float plan_stop = SEARCH_CELL_MM * fly_dec;
                    if (plan_stop > wall_stop + SEARCH_FRONT_OVERRUN_MM) {
                        printf_line("front engage: wall stop %.0f, plan "
                                    "%.0f -- braking early", wall_stop,
                                    plan_stop);
                        leg_summary(st);
                        control_move_abort();
                        enter_settle(now);
                        return;
                    }
                }
                if (wv.front) {
                    // Wall in sight at a planned stop: the wall is the
                    // position ground truth, not odometry -- end the move
                    // where the wall says the cell centre is (drift made
                    // the pure-odometry stop overrun into the panic guard
                    // on the first goal run, 2026-07-10).
                    if (fly_stop_planned &&
                        wv.front_mm <= SEARCH_FRONT_TARGET_MM) {
                        leg_summary(st);
                        control_move_abort();
                        enter_settle(now);
                        return;
                    }
                    // Last resort: unexpected wall dead ahead.
                    if (wv.front_mm < SEARCH_FRONT_PANIC_MM) {
                        search_abort("front panic -- unexpected wall");
                        return;
                    }
                }
            }

            // Side window for cell fly_w complete: commit it to the maze.
            if (d - SEARCH_CELL_MM * (fly_w - 1) > SEARCH_SIDE_WIN_HI_MM) {
                fly_commit_walls();
            }
            // Boundary crossing: traversed lock, position, step budget.
            // Wheels outrank beams -- the edge just driven through is open,
            // permanently.
            if (fly_entered < fly_dec &&
                d >= SEARCH_CELL_MM * (fly_entered + 1) -
                         0.5f * SEARCH_CELL_MM) {
                maze.set_wall_traversed(cell_x, cell_y, heading);
                cell_x += heading_dx(heading);
                cell_y += heading_dy(heading);
                fly_entered++;
                if (++steps > SEARCH_STEP_BUDGET) {
                    search_abort("step budget exhausted");
                    return;
                }
            }
            // Extend-or-brake decision, one cell ahead of the braking point.
            if (!fly_stop_planned &&
                d >= SEARCH_CELL_MM * fly_dec - SEARCH_DECIDE_MARGIN_MM) {
                fly_decide(d, wv);
            }

            if (st.active) return;
            if (!st.done) {
                // Which settle gate failed? (Control.h MOVE_DONE_*.)
                printf_line("move end: pos_err %+.1f mm, v %.1f mm/s, "
                            "yaw_err %+.2f deg (bias %+.2f)",
                            st.d_ref - st.d_mm, control_odometry().vel_mm_s,
                            st.yaw_err_deg, st.lat_bias_deg);
                search_abort("move did not settle");
                return;
            }
            leg_summary(st);
            enter_settle(now);
            return;
        }

        default:
            return;
    }
}

// -- Mission control ----------------------------------------------------------------

void search_set_printer(void (*printer)(const char *line)) { prn = printer; }

void search_set_debug(bool on) {
    dbg_stream = on;
    if (on) {
        printf_line("debug stream ON: D,ms,state,d,v_enc,v_est,yaw,yaw_err,"
                    "bias,slip,rawL,rawFL,rawFR,rawR,front,lat");
    } else {
        printf_line("debug stream OFF");
    }
}

bool search_debug() { return dbg_stream; }

bool search_running() { return state != S_IDLE; }

bool search_armed() {
    return state == S_ARMED || state == S_HAND_CLEAR || state == S_CAL;
}

// Called from setup() at power-on, and by 'h' on the bench. NOT from mission
// completion -- see the note by the hand-start constants.
void search_arm_handstart() {
    if (state != S_IDLE) return;              // never interrupt a mission
    sensors_free_run_interlaced();
    hand_run = 0;
    hand_active = false;
    hand_seen_ms = 0;
    hand_cal_started = false;
    hand_wiped = false;
    last_seq = sensors_latest().seq;
    last_trigger = 0;
    {
        bool carry = start_gate.carry_next_reset;
        start_gate.reset(millis());
        if (!carry) start_gate.need_l = start_gate.need_r = true;   // a start cell, by the rules
    }
    state = S_ARMED;
    printf_line("armed: hand <%.0f mm in front = SEARCH; hold %lu s = wipe "
                "the stored map first%s",
                SEARCH_HAND_START_MM,
                (unsigned long)(SEARCH_HAND_WIPE_MS / 1000),
                search_map_has_route()
                    ? " (FRAM holds a proven route -- 'M' to see it)"
                    : "");
}

void search_disarm_handstart() {
    if (!search_armed()) return;
    sensors_stop();
    state = S_IDLE;
    printf_line("hand start disarmed ('h' re-arms the wave: the route if the "
                "map is proven, else the search)");
}

// Common mission launch: position/state init, then the pre-turn + boot
// align leg. The caller has prepared the map/flood.
static bool mission_begin() {
    cell_x = 0;
    cell_y = 0;
    heading = HEADING_N;
    phase = PHASE_TO_GOAL;
    steps = 0;
    leg_cells = 1;
    fly_dec = 0;       // late-boundary check disarmed until a leg arms it
    fly_entered = 0;
    sensors_free_run_interlaced();  // (re)assert the mission stream
    control_velocity_enable(true);
    control_motion_zero();
    last_trigger = 0;
    // Re-runs start with a maze-referenced yaw: face the corridor first.
    // First boot: fusion yaw starts at the placement orientation, already
    // ~0, so the pre-turn skips and the align leg starts immediately.
    float yaw_now = control_heading().yaw_deg;
    if (fabsf(yaw_now) > SEARCH_PRETURN_DEG) {
        if (!control_turn_start(-yaw_now, SEARCH_TURN_OMEGA,
                                SEARCH_TURN_ALPHA)) {
            printf_line("align pre-turn refused");
            control_velocity_enable(false);
            return false;
        }
        state = S_ALIGN_TURN;
        state_t0 = millis();
        return true;
    }
    start_align_leg();
    return search_running();  // start_align_leg may itself have aborted
}

bool search_start() {
    if (!control_heading().ready) {
        printf_line("attitude not ready -- wait for gyro calibration");
        return false;
    }
    printf_line("search: boot-align leg (0,0)->(0,1), then search to goal "
                "(%d,%d)..(%d,%d). 'e' again aborts.",
                maze.goal_x0(), maze.goal_y0(), maze.goal_x1(),
                maze.goal_y1());
    maze.reset();
    // (0,0) is fully known without sensing: E wall by rules (this rig
    // always has it -- reset() presets it), W/S are boundary, and N is
    // provably open because the align leg drives through it.
    maze.set_wall(0, 0, HEADING_N, false);
    maze.set_visited(0, 0);
    return mission_begin();
}

void search_abort(const char *why) {
    flog_set_outcome(why);   // persist WHY, next to the data that shows it
    stop_everything();
    printf_line("search ABORT at (%d,%d) %s: %s", cell_x, cell_y,
                heading_name(heading), why);
    search_dump_map();
    // Deliberately NOT saved. An abort usually means the mouse stopped
    // believing where it was, and walls sensed from a wrong cell are worse
    // than no walls at all -- they are wrong AND they are trusted. Read the
    // dump above and use 'map=save' if the map looks sound.
    printf_line("map NOT saved (abort) -- 'map=save' to keep it anyway");
}

// -- Map dump -----------------------------------------------------------------------

void search_dump_map() {
    // Rows top (y=15) to bottom. Cell interior: 'G' goal, 'S' start,
    // '*' visited, ' ' untouched. Unknown edges print as open.
    //
    // Paced: 33 lines x ~50 bytes is ~1.7 KB against the radio's 1 KB TX
    // ring, which DROPS NEW BYTES when full -- an unpaced 'M' over the
    // radio delivers only the top ~10 rows, all solid by design on a
    // 6x6 rig, and reads as "100% walls" (bert, 2026-09-04: a correct
    // D6/D7 push looked destroyed; the commit's own "goal 10 cells from
    // start" on the same maze was the tell). Every call site is
    // stationary (bench 'M', search end), so waiting is safe -- same
    // treatment as the LAPREP and PARAMDEF dumps.
    const auto out = [](const char *l) {
        if (!prn) return;
        hal_radio_wait_room(64, 20);
        prn(l);
    };
    char line[MAZE_SIZE * 3 + 2];
    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        // North walls of this row.
        int p = 0;
        for (int x = 0; x < MAZE_SIZE; x++) {
            line[p++] = '+';
            const char c = maze.wall_present(x, y, HEADING_N) ? '-' : ' ';
            line[p++] = c;
            line[p++] = c;
        }
        line[p++] = '+';
        line[p] = '\0';
        out(line);
        // Cell row: west walls + interiors (+ east boundary).
        p = 0;
        for (int x = 0; x < MAZE_SIZE; x++) {
            line[p++] = maze.wall_present(x, y, HEADING_W) ? '|' : ' ';
            char c = ' ';
            if (maze.in_goal(x, y)) c = 'G';
            else if (x == 0 && y == 0) c = 'S';
            else if (maze.visited(x, y)) c = '*';
            line[p++] = c;
            line[p++] = ' ';
        }
        line[p++] = maze.wall_present(MAZE_SIZE - 1, y, HEADING_E) ? '|' : ' ';
        line[p] = '\0';
        out(line);
    }
    // South boundary.
    int p = 0;
    for (int x = 0; x < MAZE_SIZE; x++) {
        line[p++] = '+';
        const char c = maze.wall_present(x, 0, HEADING_S) ? '-' : ' ';
        line[p++] = c;
        line[p++] = c;
    }
    line[p++] = '+';
    line[p] = '\0';
    out(line);
}

const Maze &search_maze() { return maze; }
Maze &search_maze_mut() { return maze; }

void search_maze_adopt(const Maze &m) {
    maze = m;
    search_map_save();
}

// -- Goal persistence (FRAM block, Params-style: versioned + CRC) ---------------

struct SearchGoalBlock {
    uint32_t magic;    // "TW3G"
    uint16_t version;
    uint8_t x0, y0, x1, y1;
    uint16_t reserved;  // keeps the CRC 4-byte aligned
    uint32_t crc;       // params_crc32 over everything above
};
static constexpr uint32_t SEARCH_GOAL_MAGIC = 0x54573347;  // "TW3G"
static constexpr uint16_t SEARCH_GOAL_VERSION = 1;
static constexpr size_t SEARCH_GOAL_CRC_SPAN =
    sizeof(SearchGoalBlock) - sizeof(uint32_t);

static void goal_save() {
    SearchGoalBlock b;
    b.magic = SEARCH_GOAL_MAGIC;
    b.version = SEARCH_GOAL_VERSION;
    b.x0 = uint8_t(maze.goal_x0());
    b.y0 = uint8_t(maze.goal_y0());
    b.x1 = uint8_t(maze.goal_x1());
    b.y1 = uint8_t(maze.goal_y1());
    b.reserved = 0;
    b.crc = params_crc32(&b, SEARCH_GOAL_CRC_SPAN);
    fram_write(SEARCH_GOAL_FRAM_ADDR, &b, sizeof(b));
}

void search_goal_load() {
    SearchGoalBlock b;
    fram_read(SEARCH_GOAL_FRAM_ADDR, &b, sizeof(b));
    if (b.magic != SEARCH_GOAL_MAGIC || b.version != SEARCH_GOAL_VERSION ||
        b.crc != params_crc32(&b, SEARCH_GOAL_CRC_SPAN)) {
        return;  // no/invalid block: compiled-in default stands
    }
    maze.set_goal(b.x0, b.y0, b.x1, b.y1);
    printf_line("goal from FRAM: (%d,%d)..(%d,%d)", maze.goal_x0(),
                maze.goal_y0(), maze.goal_x1(), maze.goal_y1());
}

// -- Map persistence (FRAM block; same versioned + CRC policy) -----------------
//
// Lives in the tail of the sensor-LUT region: the LUT block is 8 B of header
// plus 4x256 B of table and ends at 0x0608, so 0x0610 upward was already
// spare and nothing has to move. 248 bytes of a 496-byte hole.
//
// FeRAM, so there is no erase ceremony and no wear budget worth managing --
// see Fram.h. The block is written whole, and a torn write shows up as a CRC
// failure, which is treated exactly like no map at all.

struct SearchMapBlock {
    uint32_t magic;      // "TW3M"
    uint16_t version;
    uint16_t flags;      // bit 0: a proven start->goal route existed at save
    Maze::State st;
    uint32_t crc;        // params_crc32 over everything above
};
static constexpr uint32_t SEARCH_MAP_MAGIC = 0x5457334D;  // "TW3M"
static constexpr uint16_t SEARCH_MAP_VERSION = 1;
static constexpr size_t SEARCH_MAP_CRC_SPAN =
    sizeof(SearchMapBlock) - sizeof(uint32_t);

bool search_map_has_route() {
    // Recomputes the flood, so never call this mid-mission -- the callers
    // are all in idle/armed states.
    maze.flood_to_goal(false);   // pessimistic: only fully-known edges
    return maze.distance(0, 0) != MAZE_DIST_UNREACHABLE;
}

void search_map_save() {
    if (!fram_ready()) {
        printf_line("map NOT saved -- FRAM missing");
        return;
    }
    SearchMapBlock b;
    b.magic = SEARCH_MAP_MAGIC;
    b.version = SEARCH_MAP_VERSION;
    b.flags = search_map_has_route() ? 1 : 0;
    maze.save_state(b.st);
    b.crc = params_crc32(&b, SEARCH_MAP_CRC_SPAN);
    fram_write(SEARCH_MAP_FRAM_ADDR, &b, sizeof(b));
    // A saved map is a new ladder: the next route run is MR2 (RunLadder.h).
    run_ladder_reset();
    printf_line("map saved to FRAM (%s route) -- run ladder reset: next "
                "route run is MR2",
                (b.flags & 1) ? "proven" : "no proven");
}

bool search_map_load() {
    SearchMapBlock b;
    fram_read(SEARCH_MAP_FRAM_ADDR, &b, sizeof(b));
    if (b.magic != SEARCH_MAP_MAGIC || b.version != SEARCH_MAP_VERSION ||
        b.crc != params_crc32(&b, SEARCH_MAP_CRC_SPAN)) {
        return false;   // no/invalid block: the empty compiled map stands
    }
    maze.load_state(b.st);
    // Do NOT trust the stored flag: the goal may have been moved since
    // (its own FRAM block, loaded separately), which changes the answer.
    // Re-derive it from the map that is actually loaded.
    bool route = search_map_has_route();
    // V2 has no fast run, and search_start() resets the model, so a restored
    // map does NOT shorten the next search -- it is kept so 'M' can show what
    // the last search found, and so the data is still there when the fast run
    // comes back. Say that plainly rather than promising a sprint.
    printf_line("map from FRAM: %s ('M' to view; a search still starts fresh)",
                route ? "proven route to goal" : "partial, no proven route");
    return true;
}

void search_map_wipe() {
    maze.reset();          // keeps the goal rectangle, by Maze::reset policy
    if (fram_ready()) {
        // Invalidate rather than rewrite: a zeroed magic can never be
        // mistaken for a map, and it does not depend on the struct layout
        // staying put across a firmware change.
        uint32_t zero = 0;
        fram_write(SEARCH_MAP_FRAM_ADDR, &zero, sizeof(zero));
    }
    printf_line("MAP WIPED -- next run is a fresh search");
}

void search_set_goal(int x0, int y0, int x1, int y1) {
    maze.set_goal(x0, y0, x1, y1);
    goal_save();
    printf_line("goal set: (%d,%d)..(%d,%d)%s", maze.goal_x0(),
                maze.goal_y0(), maze.goal_x1(), maze.goal_y1(),
                fram_ready() ? " -- saved to FRAM" : " (FRAM missing: RAM only)");
}
