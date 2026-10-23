#include "Explore.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "Bench.h"       // bench_params_from_fram: are these gains the tuned ones?
#include "Control.h"
#include "FlightLog.h"
#include "GoffTable.h"
#include "HAL.h"
#include "Path.h"
#include "RouteRun.h"
#include "Search.h"
#include "Sensors.h"
#include "WallGate.h"
#include "Walls.h"

// ---------------------------------------------------------------- printing
static void (*prn)(const char *) = nullptr;
void explore_set_printer(void (*printer)(const char *line)) { prn = printer; }
static void printf_line(const char *fmt, ...) {
    if (!prn) return;
    char buf[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    prn(buf);
}
static inline float wrap180f(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}
static const char *heading_name(Heading h) {
    switch (h) { case HEADING_N: return "N"; case HEADING_E: return "E";
                 case HEADING_S: return "S"; default: return "W"; }
}
// Maze north = fusion yaw 0 (the IMU is zeroed on the start cell facing
// north); +CCW, so west is +90 and east -90 (Search.h).
static float cardinal_yaw_deg(Heading h) {
    switch (h) {
        case HEADING_N: return 0.0f;
        case HEADING_E: return -90.0f;
        case HEADING_S: return 180.0f;
        default:        return 90.0f;
    }
}

// ---------------------------------------------------------------- state
enum ExState : uint8_t {
    EX_IDLE,
    EX_CAL,          // IMU recalibration, stationary
    EX_ROLL,         // flying the rolling path
    EX_STOPPED,      // path ended at a planned stop (cell centre or turn point)
    EX_SENSE,        // stationary burst
    EX_TURN,         // in-place turn
    EX_TURN_SETTLE,  // tyres relax before the next path
    EX_RECENTRE,     // after a hard front stop: driving back to the cell centre
};
enum ExPhase : uint8_t { PH_TO_GOAL, PH_EXPLORE, PH_TO_START };
enum CellKind : uint8_t {
    CK_PENDING = 0,  // entered, not yet decided
    CK_STRAIGHT,
    CK_LEFT,
    CK_RIGHT,
    CK_STOP,         // path ends at this cell's centre
};

// One cell the path passes through. s_b is the path s of its ENTRY
// boundary, s_p the turn point (s_b + EX_TURN_LEAD_MM) where the path ends
// until the cell is decided.
struct CellRec {
    int8_t cx, cy;
    Heading h;        // heading on entry
    CellKind kind;
    float s_b, s_p;
    // Side votes while the mouse is in the window, front votes likewise.
    uint16_t n_side, n_left, n_right;
    uint16_t n_front, n_fwall;
    float front_err_sum;   // (expected - measured), front packets
    bool traversed_in;     // the entry edge has been locked
    bool gates_done;       // wall-end gates armed (or judged unarmable)
    bool hold_tp;          // decision withheld: stop at the turn point, burst
};
constexpr int EX_RING = 8;
static CellRec ring[EX_RING];
static int ring_n = 0;      // records ever pushed (index = ring_n - 1 is last)
static inline CellRec &rec(int i) { return ring[((i % EX_RING) + EX_RING) % EX_RING]; }
static inline CellRec &last_rec() { return rec(ring_n - 1); }

static ExState state = EX_IDLE;
static ExPhase phase = PH_TO_GOAL;
static uint32_t state_t0 = 0, t_launch = 0;
static uint32_t last_seq = 0;
static bool cal_started = false;
// THE SEARCH'S PRE-FLIGHT (bert, 2026-09-18: "only wipe the log after a
// good sensor reading and all sensors valid"). The route run has had one
// since the CAL-6 days (RouteRun.cpp, RR_PREFLIGHT_MIN_PKTS); the search
// had none and armed the flight log -- overwriting the last run's -- on
// the IMU's say-so alone. A mouse powered on too soon after power-off
// comes up with a beam stuck at 255 (MR3 of the first ladder test), and a
// search launched like that flies one-sided into the first wall. So while
// the mouse stands on the start cell every packet's side beams are
// tallied. The start cell is (0,0) facing NORTH: the LEFT beam looks at
// the maze's outer wall, which every maze has, and the RIGHT at the start
// cell's east wall, which the rules put there (RACE_PLAN.md, D11 on). A
// beam that never reads live over EX_PREFLIGHT_MIN_PKTS packets is dead
// and the search is refused BEFORE flog_arm -- FRAM untouched, the
// previous log still dumps with 'y'. On a rig whose start cell is open on
// the right, put the wall back: the day's maze will have it.
static constexpr int EX_PREFLIGHT_MIN_PKTS = 20;
static constexpr uint32_t EX_PREFLIGHT_WAIT_MS = 2000;
static int pf_pkts = 0, pf_live_l = 0, pf_live_r = 0;
static Maze *mz = nullptr;
PATH_STORAGE static SpeedPath pathA;
PATH_STORAGE static SpeedPath pathB;
static SpeedPath *path = &pathA;
static float v_search = 100.0f;
static int steps = 0;
static int stops = 0;
static int anchors = 0, anchors_skipped = 0, skip_run = 0;
static bool stop_at_centre = false;   // the current path's end is a centre
static Heading stop_turn_to = HEADING_N;
static bool stop_then_sense = false;  // stopped undecided: burst first
static bool stopped_at_tp = false;    // ...at the turn point, not the centre
static bool finishing = false;        // home: the last stop ends the mission
static WallGate gate_l, gate_r;
static bool gate_armed_l = false, gate_armed_r = false;
static float anchor_corr_this = 0.0f; // for the log record
static float lat_this = 0.0f;
static int soft_run = 0;              // consecutive packets inside EX_FRONT_SOFT_MM
static bool soft_stop = false;        // the coming stop is the soft guard's, not a centre
static float soft_stop_past_mm = 0.0f; // ...and stands this far past the believed centre
static float resume_past_mm = 0.0f;   // after an in-place turn: past the centre along the NEW heading
static int hard_cx = -1, hard_cy = -1, hard_n = 0;   // hard front stops in the same cell
// THE RECENTRE (2026-09-27, the (5,0) alcove): after a hard stop the mouse
// goes back to the cell centre before it turns to leave -- see front_hard_stop.
static uint8_t rc_phase = 0;       // 0 none, 1 turn toward the centre pending, 2 driving
static float rc_yaw_deg = 0.0f;    // maze-frame yaw that points at the centre
static float rc_mm = 0.0f;         // how far
static constexpr float EX_RECENTRE_MIN_MM = 20.0f;
static constexpr float EX_RECENTRE_V = 150.0f;
static constexpr float EX_RECENTRE_ACC = 1000.0f;
static float centre_err_mm = 0.0f;    // burst: the mouse is this far past the centre
static bool lat_valid_this = false;
static uint8_t event_pending = EXE_SAMPLE;
static uint32_t led1_off_ms = 0;
static void led_service(uint32_t now) {
    if (led1_off_ms && (int32_t)(now - led1_off_ms) >= 0) { hal_set_led1(false); led1_off_ms = 0; }
}
static float log_last_s = -1e9f;
static float s_total_base = 0.0f;     // path s of earlier paths this mission
static uint32_t log_last_ms = 0;
// Stationary sensing burst.
static int sn_n = 0, sn_l = 0, sn_r = 0, sn_f = 0;
static float sn_front_sum = 0.0f;
static int sn_front_n = 0;

// ---------------------------------------------------------------- helpers
static int unknown_cells() {
    int n = 0;
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            if (mz->has_unknown_wall(x, y)) n++;
    return n;
}
static void flood_for_phase() {
    if (phase == PH_TO_GOAL) mz->flood_to_goal(true);
    // Explore only what could still improve the route (Maze.h,
    // flood_to_useful_unknown): dead ends and far-off pockets are skipped.
    else if (phase == PH_EXPLORE) mz->flood_to_useful_unknown(EX_ROUTE_SLACK_CELLS);
    else mz->flood_to_start(true);
}
static uint8_t wall_bits(int cx, int cy, Heading h) {
    uint8_t b = 0;
    Heading L = heading_left(h), R = heading_right(h);
    if (mz->wall_present(cx, cy, L)) b |= 1;
    if (mz->wall_present(cx, cy, h)) b |= 2;
    if (mz->wall_present(cx, cy, R)) b |= 4;
    if (mz->wall_known(cx, cy, L)) b |= 16;
    if (mz->wall_known(cx, cy, h)) b |= 32;
    if (mz->wall_known(cx, cy, R)) b |= 64;
    return b;
}

// ---------------------------------------------------------------- log
static void log_record(uint8_t ev, bool stationary) {
    if (!flog_active()) return;
    uint32_t now = millis();
    ControlPathStatus ps = control_path_status();
    ControlMotion mo = control_motion();
    ControlOdometry od = control_odometry();
    ControlHeading hd = control_heading();
    WallRanges w = sensors_latest();
    const CellRec &c = ring_n ? last_rec() : ring[0];
    // The believed cell is the last record whose entry boundary is behind
    // the mouse; the last record itself while stationary.
    const CellRec *cur = &c;
    if (!stationary && ring_n >= 2 && ps.s_mm < c.s_b) cur = &rec(ring_n - 2);
    FlogRec r;
    memset(&r, 0, sizeof r);
    r.t_ms = (uint16_t)(now - t_launch);
    r.s_mm = (int16_t)ps.s_mm;
    r.v_mm_s = (int16_t)ps.v_mm_s;
    r.yaw_cdeg = (int16_t)(hd.yaw_deg * 10.0f);
    r.yaw_err_cdeg = (int16_t)(ps.yaw_err_deg * 10.0f);
    r.ay_mm_s2 = (int16_t)mo.a_lat_mm_s2;
    r.ax_mm_s2 = (int16_t)mo.a_fwd_mm_s2;
    r.flags = (uint8_t)((ps.in_arc ? 1 : 0) |
                        (ps.front_gov ? 2 : 0) |       // the front governor capped v
                        (anchor_corr_this != 0.0f ? 4 : 0) |
                        (lat_valid_this ? 8 : 0) |
                        (stationary ? 16 : 0) |
                        (state == EX_SENSE ? 32 : 0));
    r.left_mm = w.left_mm;
    r.right_mm = w.right_mm;
    r.front_left_mm = w.front_left_mm;
    r.front_right_mm = w.front_right_mm;
    r.wheel_fused_mm = (int16_t)(od.dist_mm - (s_total_base + ps.s_mm));
    r.omega_dps = (int16_t)control_move_status().yaw_rate_dps;
    r.obs_along_mm = (int16_t)lroundf(anchor_corr_this);
    r.obs_cross_mm = (int16_t)lroundf(lat_valid_this ? lat_this : 0.0f);
    r.obs_n = ev;
    r.pred_axis_l = (uint8_t)cur->cx;
    r.pred_axis_r = (uint8_t)cur->cy;
    r.pred_coord_l = (int16_t)(((int)cur->h << 8) | wall_bits(cur->cx, cur->cy, cur->h));
    r.pred_coord_r = (int16_t)mz->distance(cur->cx, cur->cy);
    r.pred_rng_l = cur->n_side ? (uint8_t)(100 * cur->n_left / cur->n_side) : 255;
    r.pred_rng_r = cur->n_side ? (uint8_t)(100 * cur->n_right / cur->n_side) : 255;
    if (mz->wall_known(cur->cx, cur->cy, cur->h) &&
        mz->wall_present(cur->cx, cur->cy, cur->h)) {
        float exp_f = (cur->s_b + EX_CELL_MM - EX_HALF_POST_MM) - ps.s_mm;
        if (exp_f > 0.0f && exp_f < 320.0f)
            r.obs_head_cdeg = (int16_t)(exp_f * 100.0f);
    }
    flog_record(r);
    anchor_corr_this = 0.0f;
    log_last_s = ps.s_mm;
    log_last_ms = now;
    event_pending = EXE_SAMPLE;
}
static void event(uint8_t ev) {
    if (event_pending == EXE_SAMPLE || ev >= event_pending) event_pending = ev;
    log_record(event_pending, state != EX_ROLL);
}

// ---------------------------------------------------------------- wall frame
// WALL-FRAME ALIGNMENT, ported from RouteRun (walign). A search runs for
// minutes and the gyro drifts ~0.01 deg/s: after ten minutes every
// in-place turn would be several degrees off and the servo would be
// holding a permanent bias on every straight. On a straight with both
// walls the corridor's direction in the path frame is observable
// (heading minus the lateral's drift angle); walign of each 150 mm
// window's estimate is applied to the executor's datum, 1 deg per window
// at most, and the total is carried across paths and into the turns.
static float frame_deg = 0.0f;      // the frame correction so far, deg
static int fr_windows = 0;
static float fr_last_est = 0.0f;
static float fr_s0 = 0.0f;
static int fr_n = 0;
static double fr_sx = 0, fr_sy = 0, fr_sxx = 0, fr_sxy = 0, fr_sh = 0;
static bool fr_open = false;
static float lat_bias_cmd = 0.0f;
constexpr float EX_FRAME_FIT_MM = 150.0f;
constexpr float EX_FRAME_STEP_MAX_DEG = 1.0f;
constexpr float EX_FRAME_MAX_DEG = 12.0f;
static void fr_window_reset() {
    fr_open = false; fr_n = 0;
    fr_sx = fr_sy = fr_sxx = fr_sxy = fr_sh = 0;
}
static void fr_sample(float s, float lat, float hdg_deg) {
    float k = control_get_gains().wall_align;
    if (k <= 0.0f) return;
    if (!fr_open) { fr_open = true; fr_s0 = s; }
    double x = s - fr_s0;
    fr_n++;
    fr_sx += x; fr_sy += lat; fr_sxx += x * x; fr_sxy += x * lat;
    fr_sh += hdg_deg;
    if (x < EX_FRAME_FIT_MM || fr_n < 20) return;
    double den = fr_n * fr_sxx - fr_sx * fr_sx;
    if (den <= 0) { fr_window_reset(); return; }
    double slope = (fr_n * fr_sxy - fr_sx * fr_sy) / den;
    if (slope > 0.2) slope = 0.2;
    if (slope < -0.2) slope = -0.2;
    float drift_deg = (float)(asin(slope) * 57.29577951308232);
    float est = (float)(fr_sh / fr_n) - drift_deg;
    fr_last_est = est;
    float step = k * est;
    if (step > EX_FRAME_STEP_MAX_DEG) step = EX_FRAME_STEP_MAX_DEG;
    if (step < -EX_FRAME_STEP_MAX_DEG) step = -EX_FRAME_STEP_MAX_DEG;
    if (frame_deg + step > EX_FRAME_MAX_DEG) step = EX_FRAME_MAX_DEG - frame_deg;
    if (frame_deg + step < -EX_FRAME_MAX_DEG) step = -EX_FRAME_MAX_DEG - frame_deg;
    if (step != 0.0f) {
        control_path_frame_adjust(step);
        frame_deg += step;
        fr_windows++;
    }
    fr_window_reset();
}

// ---------------------------------------------------------------- stop
static void stop_everything() {
    control_brake_to_stop();
    control_move_abort();
    control_velocity_enable(false);
    control_set_lateral_bias_deg(0.0f);
    control_set_reflex_bias_deg(0.0f);
    flog_stop();
    sensors_stop();
    hal_set_led1(false);
    hal_set_led2(false);
}

static void save_map_and_report(const char *how) {
    search_map_save();
    // ARRIVE CLEARED, AUTOMATICALLY (RACE_PLAN.md item 4): the learned gate
    // offsets are maze property and the maze just changed. Trims survive
    // (goff_table_clear keeps them). A pushed map does not clear -- the
    // rigs keep their learning across sessions; a SEARCHED maze is new.
    goff_table_clear();
    goff_table_save();
    printf_line("search: goff table cleared (new maze) -- MR2 learns fresh");
    {
        uint16_t v[MAZE_SIZE + 1], h[MAZE_SIZE + 1];
        mz->walls_pack(v, h);
        flog_set_walls(v, h, MAZE_SIZE + 1);
    }
    printf_line("search: %s -- %d steps, %d stops, %d anchors (%d dropped), "
                "frame %+.1f deg over %d windows, %.1f m in %lu s, %d cells "
                "still hold an unknown wall",
                how, steps, stops, anchors, anchors_skipped, (double)frame_deg,
                fr_windows,
                (double)(s_total_base / 1000.0f),
                (unsigned long)((millis() - t_launch) / 1000),
                unknown_cells());
    mz->flood_to_goal(false);
    uint8_t d = mz->distance(0, 0);
    if (d != MAZE_DIST_UNREACHABLE)
        printf_line("known route start->goal: %u cells -- the map is saved, "
                    "'M' shows it, route.arm flies it", d);
    else
        printf_line("no fully-known route start->goal yet -- the partial map "
                    "is saved");
    search_dump_map();
}

void explore_abort(const char *why) {
    if (state == EX_IDLE) return;
    const CellRec &c = ring_n ? last_rec() : ring[0];
    printf_line("search ABORT at (%d,%d) %s: %s", c.cx, c.cy,
                heading_name(c.h), why);
    event(EXE_ABORT);
    flog_set_outcome(why);
    if (state == EX_ROLL) s_total_base += control_path_status().s_mm;
    stop_everything();
    state = EX_IDLE;
    save_map_and_report("aborted");
}

// ---------------------------------------------------------------- path builder
// The cursor is the path end: always the turn point of the last record
// (or its centre when the record is a STOP).
static void push_rec(int cx, int cy, Heading h, float s_b) {
    CellRec &r = rec(ring_n);
    memset(&r, 0, sizeof r);
    r.cx = (int8_t)cx; r.cy = (int8_t)cy; r.h = h;
    r.kind = CK_PENDING;
    r.s_b = s_b;
    r.s_p = s_b + ex_turn_lead_mm();
    ring_n++;
}

// Extend the path from the last record's turn point by its decision, and
// push the record of the cell that puts the mouse in. Returns false if the
// path is full (caller rebases first).
static bool extend_by(CellKind k) {
    CellRec &c = last_rec();
    c.kind = k;
    // The wall-end gates armed for THIS cell's entry are done: the drop was
    // seen or it was not. Left armed into a turn they read the outside beam
    // swinging off the wall as a wall end 150 mm from its prediction (four
    // "dropped anchors" on the 200 mm/s flight, all inside arcs) -- and two
    // of those in a row would have ended the mission as "lost".
    gate_l.disarm(); gate_r.disarm();
    gate_armed_l = gate_armed_r = false;
    float s_end = path->length_mm();   // == c.s_p by construction
    float lead = ex_turn_lead_mm();
    if (k == CK_STRAIGHT) {
        if (!path->add_straight(EX_CELL_MM)) return false;
        int nx = c.cx + heading_dx(c.h), ny = c.cy + heading_dy(c.h);
        push_rec(nx, ny, c.h, s_end + EX_CELL_MM - lead);
    } else if (k == CK_LEFT || k == CK_RIGHT) {
        float ang = (k == CK_LEFT) ? 90.0f : -90.0f;
        if (!path->add_arc(EX_TURN_R_MM, ang)) return false;
        if (!path->add_straight(2.0f * lead)) return false;
        Heading nh = (k == CK_LEFT) ? heading_left(c.h) : heading_right(c.h);
        // The primitive's own length (clothoids included) puts the next
        // turn point where the path now ends.
        float s_next = path->length_mm();
        int nx = c.cx + heading_dx(nh), ny = c.cy + heading_dy(nh);
        push_rec(nx, ny, nh, s_next - lead);
    } else {  // CK_STOP: on to the centre
        if (!path->add_straight(0.5f * EX_CELL_MM - lead)) return false;
    }
    control_path_extended();
    return true;
}

// Rebuild the path from the LAST record's entry boundary into the spare
// object and swap the executor onto it. Called before a decision, with the
// mouse between the boundary and the turn point of that record.
static bool rebase() {
    CellRec &c = last_rec();
    ControlPathStatus ps = control_path_status();
    float s_now = ps.s_mm;
    if (s_now < c.s_b - 5.0f || s_now > c.s_p) return false;  // not on the lead-in
    SpeedPath *np = (path == &pathA) ? &pathB : &pathA;
    np->clear(0.0f, 0.0f, cardinal_yaw_deg(c.h));
    np->set_clothoid_len(EX_TURN_CLOTHOID_MM);
    np->set_limits(v_search, EX_ACC_MM_S2, EX_ALAT_MM_S2);
    if (!np->add_straight(ex_turn_lead_mm())) return false;
    float shift = c.s_b;
    if (!control_path_swap(np, s_now - shift)) return false;
    path = np;
    for (int i = 0; i < EX_RING && i < ring_n; i++) {
        CellRec &r = rec(ring_n - 1 - i);
        r.s_b -= shift;
        r.s_p -= shift;
    }
    gate_l.disarm(); gate_r.disarm();
    gate_armed_l = gate_armed_r = false;
    log_last_s -= shift;
    event(EXE_REBASE);
    return true;
}

// Start (or restart) the path from the cursor. The path already holds its
// first segments; the mouse stands at s = 0 of it.
static bool launch_path() {
    // Datum: the path's yaw is ABSOLUTE (north = 0) plus the wall-frame
    // correction learned so far, whatever the mouse is standing at.
    float yaw = control_heading().yaw_deg;
    if (!control_path_start(path, wrap180f(frame_deg - yaw))) return false;
    fr_window_reset();
    control_path_slip_enable(false);
    soft_run = 0;
    state = EX_ROLL;
    state_t0 = millis();
    log_last_s = -1e9f;
    return true;
}

// A fresh path from a cell CENTRE heading h: straight to the next cell's
// turn point. Used after every stop.
static bool path_from_centre(int cx, int cy, Heading h, float past_mm) {
    // past_mm: how far past the centre the mouse actually stands along h
    // (a stationary front reading said so); the first straight is shorter
    // by that much, so the next boundary lands where the wall is.
    if (past_mm > EX_FRONT_TOL_MM) past_mm = EX_FRONT_TOL_MM;
    if (past_mm < -EX_FRONT_TOL_MM) past_mm = -EX_FRONT_TOL_MM;
    path->clear(0.0f, 0.0f, cardinal_yaw_deg(h));
    path->set_clothoid_len(EX_TURN_CLOTHOID_MM);
    path->set_limits(v_search, EX_ACC_MM_S2, EX_ALAT_MM_S2);
    if (!path->add_straight(0.5f * EX_CELL_MM + ex_turn_lead_mm() - past_mm)) return false;
    // The cell we stand in is decided (straight out of it).
    push_rec(cx, cy, h, -0.5f * EX_CELL_MM - past_mm);
    last_rec().kind = CK_STRAIGHT;
    last_rec().traversed_in = true;
    push_rec(cx + heading_dx(h), cy + heading_dy(h), h, 0.5f * EX_CELL_MM - past_mm);
    gate_l.disarm(); gate_r.disarm();
    gate_armed_l = gate_armed_r = false;
    stop_at_centre = false;
    return true;
}

// ---------------------------------------------------------------- gates
static void arm_gates_for(const CellRec &c) {
    // The cell being LEFT is the previous record; a wall on its side that
    // ends at this boundary is the landmark. Only after a straight
    // traversal: the beams swing through an arc.
    if (ring_n < 2) return;
    const CellRec &p = rec(ring_n - 2);
    if (p.kind != CK_STRAIGHT) return;
    float expect = c.s_b + EX_HALF_POST_MM - ex_beam_lead_mm(v_search);
    WallGateConfig cfg;
    cfg.expect_s_mm = expect;
    cfg.window_mm = EX_ANCHOR_MAX_MM;
    cfg.live_from_s_mm = expect - 60.0f;
    cfg.klat = 0.0f;
    Heading L = heading_left(p.h), R = heading_right(p.h);
    if (mz->wall_known(p.cx, p.cy, L) && mz->wall_present(p.cx, p.cy, L)) {
        gate_l.arm(cfg); gate_armed_l = true;
    }
    if (mz->wall_known(p.cx, p.cy, R) && mz->wall_present(p.cx, p.cy, R)) {
        gate_r.arm(cfg); gate_armed_r = true;
    }
}

static void apply_anchor(float corr, float s_now, uint8_t ev) {
    const CellRec &c = last_rec();
    float room = c.s_p - s_now;
    if (room < 5.0f) room = 5.0f;
    if (room > 60.0f) room = 60.0f;
    control_path_correct(corr, room);
    anchor_corr_this = corr;
    anchors++;
    skip_run = 0;
    hal_set_led1(true);          // a 40 ms flash, as the route mission does
    led1_off_ms = millis() + 40;
    event(ev);
}

static void gate_update(WallGate &g, bool &armed, bool right, float s_now,
                        const WallRanges &w, const WallView &wv) {
    if (!armed) return;
    uint8_t raw = right ? w.right_mm : w.left_mm;
    float lat_in = wv.lateral_valid ? (right ? -wv.lateral_mm : wv.lateral_mm) : 0.0f;
    float corr = 0.0f;
    WallGate::Result gr = g.update(s_now, raw, wv.lateral_valid, lat_in, &corr);
    if (gr == WallGate::CORRECTION) {
        armed = false;
        apply_anchor(corr, s_now, right ? EXE_ANCHOR_R : EXE_ANCHOR_L);
    } else if (gr == WallGate::SKIPPED_WINDOW) {
        armed = false;
        anchors_skipped++;
        skip_run++;
        event(EXE_ANCHOR_SKIP);
        printf_line("  anchor dropped: %s wall end %+.0f mm from where the "
                    "map put it (drop at s %.0f)", right ? "right" : "left",
                    (double)corr, (double)g.drop_s_mm());
        if (skip_run >= 2) explore_abort("lost: two wall ends in a row far from the map");
    } else if (gr == WallGate::SKIPPED_LAT) {
        armed = false;
    }
}

// ---------------------------------------------------------------- decision
static bool at_phase_target(int cx, int cy) {
    if (phase == PH_TO_GOAL) return mz->in_goal(cx, cy);
    if (phase == PH_EXPLORE) return !mz->useful_unknown_reachable(cx, cy, EX_ROUTE_SLACK_CELLS);
    return cx == 0 && cy == 0;
}

// Decide the last record from its votes (or a stationary burst), extend
// the path or plan a stop. `L,F,R` are the wall verdicts, `known` says the
// votes were conclusive.
static void decide(bool L, bool F, bool R, bool known, float s_now, bool stationary) {
    CellRec &c = last_rec();
    Heading HL = heading_left(c.h), HR = heading_right(c.h);
    // A FRONT WALL THE MAP ALREADY HAS IS NOT REMOVED BY A ROLLING VOTE
    // (2026-09-26 18:24, (4,2) W twice: the vote read f9/86 then f42/86 on
    // a wall the soft guard then found at 128 mm and the burst placed at
    // exactly 174 -- a face that reads long or not at all from 160-250 mm,
    // a knocked wall most likely -- and the second pass re-opened the
    // burst-confirmed wall on the marginal vote, drove at it and stopped
    // again). Rolling votes may add walls; only a stationary burst, 84 mm
    // from an unmistakable face, may take a front wall away. A wrongly
    // kept wall costs a route its shortest line; a wrongly removed one
    // costs the mouse a stop, or worse.
    if (known && !stationary && !F && mz->wall_known(c.cx, c.cy, c.h) &&
        mz->wall_present(c.cx, c.cy, c.h)) {
        printf_line("  front wall of (%d,%d) %s kept from the map: the rolling vote read %u/%u",
                    c.cx, c.cy, heading_name(c.h), c.n_fwall, c.n_front);
        F = true;
    }
    if (known) {
        mz->set_wall(c.cx, c.cy, HL, L);
        mz->set_wall(c.cx, c.cy, c.h, F);
        mz->set_wall(c.cx, c.cy, HR, R);
    }
    mz->set_visited(c.cx, c.cy);
    // THE CONTRADICTION GUARD: a sensed wall on an edge the mouse has
    // driven through (locked) is not a wall, it is a lost cell.
    if (known) {
        bool bad = false;
        if (L && mz->wall_known(c.cx, c.cy, HL) && !mz->wall_present(c.cx, c.cy, HL)) bad = true;
        if (F && mz->wall_known(c.cx, c.cy, c.h) && !mz->wall_present(c.cx, c.cy, c.h)) bad = true;
        if (R && mz->wall_known(c.cx, c.cy, HR) && !mz->wall_present(c.cx, c.cy, HR)) bad = true;
        if (bad) {
            explore_abort("lost: a wall where the map has a traversed opening");
            return;
        }
    }
    printf_line("cell (%d,%d) %s | walls L%d F%d R%d | votes %u/%u/%u f%u/%u%s | step %d",
                c.cx, c.cy, heading_name(c.h), L, F, R, c.n_left, c.n_right,
                c.n_side, c.n_fwall, c.n_front, stationary ? " (burst)" : "",
                steps);
    // Phase bookkeeping.
    if (at_phase_target(c.cx, c.cy)) {
        if (phase == PH_TO_GOAL) {
            phase = search_explore_all() ? PH_EXPLORE : PH_TO_START;
            printf_line("-- GOAL reached in %d steps -- %s --", steps,
                        search_explore_all() ? "exploring the rest" : "returning to start");
            event(EXE_GOAL);
            hal_set_led2(true);
        } else if (phase == PH_EXPLORE) {
            phase = PH_TO_START;
            {
                int known = 0;
                mz->flood_to_useful_unknown(EX_ROUTE_SLACK_CELLS, &known);
                printf_line("-- ROUTES PROVEN in %d steps: best known route %d cells, no "
                            "unknown wall left on any route within %d cells of it (%d "
                            "cells unknown, dead ends and off-route) -- returning to start --",
                            steps, known, EX_ROUTE_SLACK_CELLS, unknown_cells());
            }
            event(EXE_MAPPED);
        } else {
            // Home: stop at the centre and finish.
            finishing = true;
            stop_at_centre = true;
            stop_then_sense = false;
            if (!extend_by(CK_STOP)) { explore_abort("path full at home"); return; }
            event(EXE_HOME);
            return;
        }
    }
    // Re-check the target after a phase change (a goal that is also
    // fully mapped, or explore-all off at the start cell).
    if (at_phase_target(c.cx, c.cy) && phase == PH_TO_START && c.cx == 0 && c.cy == 0) {
        finishing = true; stop_at_centre = true; stop_then_sense = false;
        if (!extend_by(CK_STOP)) { explore_abort("path full at home"); return; }
        event(EXE_HOME);
        return;
    }
    if (phase == PH_EXPLORE && (steps % 10) == 0) {
        int known = 0;
        int n = mz->flood_to_useful_unknown(EX_ROUTE_SLACK_CELLS, &known);
        printf_line("   explore: %d cells still worth a look (best known route %d cells), "
                    "%d unknown in all", n, known, unknown_cells());
    }

    Heading next;
    flood_for_phase();
    if (!mz->best_step(c.cx, c.cy, c.h, next)) {
        explore_abort("no route (flood blocked)");
        return;
    }
    // Front anchor: the wall ahead, when there is one, places the mouse
    // before it turns or stops -- the second along-track instrument.
    // Beyond the anchor window the belief is too far out to roll the turn
    // on (a +50 would start the arc behind the mouse, a -50 exit it 50 mm
    // into the wrong lane): the path is left ending at the turn point, the
    // mouse stops there and the burst re-references it on this same wall
    // (stopped_at_tp, exp 144) before the turn resumes.
    if (F && c.n_fwall >= 4 && !stationary) {
        float corr = c.front_err_sum / (float)c.n_fwall;
        if (fabsf(corr) <= EX_ANCHOR_MAX_MM) apply_anchor(corr, s_now, EXE_ANCHOR_F);
        else {
            anchors_skipped++;
            event(EXE_ANCHOR_SKIP);
            printf_line("  front wall %+.0f mm from where the map put it: stopping at the "
                        "turn point to re-sense", (double)corr);
            c.hold_tp = true;
            event(EXE_FRONT_HOLD);
            return;
        }
    }
    if (path->seg_count() >= EX_REBASE_SEGS && !stationary) rebase();
    steps++;
    if (next == c.h) {
        if (!extend_by(CK_STRAIGHT)) { explore_abort("path full"); return; }
        event(EXE_STRAIGHT);
    } else if (next == HL) {
        if (!extend_by(CK_LEFT)) { explore_abort("path full"); return; }
        event(EXE_LEFT);
    } else if (next == HR) {
        if (!extend_by(CK_RIGHT)) { explore_abort("path full"); return; }
        event(EXE_RIGHT);
    } else {
        // Back: on to the centre, stop, turn about.
        stop_at_centre = true;
        stop_turn_to = next;
        stop_then_sense = false;
        if (!extend_by(CK_STOP)) { explore_abort("path full"); return; }
        event(EXE_BACK);
    }
}

// Thin evidence: on to the centre and stop; the burst decides.
static void defer_to_centre() {
    stop_at_centre = true;
    stop_then_sense = true;
    if (!extend_by(CK_STOP)) { explore_abort("path full"); return; }
    event(EXE_UNDECIDED);
}

// ---------------------------------------------------------------- start
bool explore_start(bool imu_ready) {
    if (state != EX_IDLE) { printf_line("search: already running"); return false; }
    if (search_running()) { printf_line("search: the old mission owns the mouse"); return false; }
    mz = &search_maze_mut();
    v_search = control_get_gains().search_v_mm_s;
    if (v_search < 50.0f) v_search = 50.0f;
    if (v_search > 400.0f) v_search = 400.0f;
    mz->reset();
    mz->set_wall(0, 0, HEADING_N, false);   // the launch drives through it
    ring_n = 0;
    steps = stops = anchors = anchors_skipped = skip_run = 0;
    phase = PH_TO_GOAL;
    finishing = false;
    stop_at_centre = stop_then_sense = stopped_at_tp = false;
    soft_run = 0; soft_stop = false; soft_stop_past_mm = 0.0f;
    resume_past_mm = 0.0f; hard_cx = hard_cy = -1; hard_n = 0; rc_phase = 0;
    gate_l.disarm(); gate_r.disarm();
    gate_armed_l = gate_armed_r = false;
    anchor_corr_this = 0.0f;
    centre_err_mm = 0.0f;
    frame_deg = 0.0f; fr_windows = 0; fr_last_est = 0.0f; lat_bias_cmd = 0.0f;
    s_total_base = 0.0f;
    fr_window_reset();
    led1_off_ms = 0;
    event_pending = EXE_SAMPLE;
    // The mouse stands with its back to the rear wall: starty from the
    // south boundary of (0,0). Path s = 0 there; (0,1)'s entry boundary is
    // (180 - starty) ahead.
    float starty = control_get_gains().start_y_mm;
    path = &pathA;
    path->clear(0.0f, 0.0f, cardinal_yaw_deg(HEADING_N));
    path->set_clothoid_len(EX_TURN_CLOTHOID_MM);
    path->set_limits(v_search, EX_ACC_MM_S2, EX_ALAT_MM_S2);
    float to_b = EX_CELL_MM - starty;
    path->add_straight(to_b + ex_turn_lead_mm());
    push_rec(0, 0, HEADING_N, -starty);
    last_rec().kind = CK_STRAIGHT;
    last_rec().traversed_in = true;
    push_rec(0, 1, HEADING_N, to_b);
    printf_line("search: rolling, %.0f mm/s, R%.0f turns inside the cell, "
                "goal (%d,%d)..(%d,%d), explore-all %s. 'e' again aborts.",
                (double)v_search, (double)EX_TURN_R_MM, mz->goal_x0(),
                mz->goal_y0(), mz->goal_x1(), mz->goal_y1(),
                search_explore_all() ? "ON" : "off");
    if (!bench_params_from_fram()) {
        // 2026-09-21: a 16x16 search flown on the compiled defaults (kyaw
        // 2 for 10, track 59 for 72.3, kpt 0.0004 for 0.004) turned 75 deg
        // for a 90, held a 22 deg heading error down the next cell and
        // drove into the wall. The mouse still flies -- on the day a blank
        // FRAM must not stop the run -- but it says so, in capitals.
        ControlGains g = control_get_gains();
        printf_line("search: WARNING -- REGISTRY IS COMPILED DEFAULTS, not the "
                    "tuned values (kyaw %.1f, track %.1f, kpt %.4f): FRAM %s, "
                    "params block not loaded. Restore the .fram.json backup "
                    "or re-enter and 'w' before flying",
                    (double)g.kyaw, (double)g.track_mm, (double)g.kp_turn,
                    fram_ready() ? "live" : "NOT LIVE");
    }
    sensors_free_run_interlaced();
    control_motion_zero();
    control_velocity_enable(true);
    hal_set_led1(false); hal_set_led2(false);
    cal_started = imu_ready;   // already calibrated: EX_CAL passes straight through
    pf_pkts = pf_live_l = pf_live_r = 0;
    last_seq = sensors_latest().seq;
    state = EX_CAL;
    state_t0 = millis();
    return true;
}

// The launch gate: true = go, false = still waiting; a dead beam or a
// silent stream refuses the search from here (state back to EX_IDLE).
static void preflight_sample() {
    WallRanges w = sensors_latest();
    if (w.seq == last_seq) return;
    last_seq = w.seq;
    pf_pkts++;
    if (w.left_mm > 0 && w.left_mm < 255) pf_live_l++;
    if (w.right_mm > 0 && w.right_mm < 255) pf_live_r++;
}
static bool preflight_pass(uint32_t now) {
    preflight_sample();
    if (pf_pkts < EX_PREFLIGHT_MIN_PKTS) {
        if (now - state_t0 < EX_PREFLIGHT_WAIT_MS) return false;
        printf_line("search: PRE-FLIGHT FAILED -- only %d sensor packets in "
                    "%u ms stationary: the sensor stream is not running",
                    pf_pkts, (unsigned)(now - state_t0));
    } else if (pf_live_l > 0 && pf_live_r > 0) {
        printf_line("search: pre-flight OK -- %d packets, left %d live, "
                    "right %d live", pf_pkts, pf_live_l, pf_live_r);
        return true;
    } else {
        bool dead_l = pf_live_l == 0, dead_r = pf_live_r == 0;
        printf_line("search: PRE-FLIGHT FAILED -- the %s%s%s beam read 255 "
                    "for all %d packets on the start cell (outer wall left, "
                    "start-cell wall right): SENSOR FAULT (power-on too soon "
                    "after power-off?) -- or the rig's start cell is missing "
                    "that wall", dead_l ? "LEFT" : "",
                    (dead_l && dead_r) ? " and " : "", dead_r ? "RIGHT" : "",
                    pf_pkts);
    }
    // Refused before flog_arm: nothing in FRAM has changed. Not an
    // explore_abort -- that would stamp an outcome on the LAST run's log
    // and save an empty map over the wiped one.
    printf_line("  power the mouse off, wait a few seconds, power on, and "
                "search again");
    stop_everything();
    state = EX_IDLE;
    return false;
}

static void launch(uint32_t now) {
    t_launch = now;
    flog_arm();
    {
        ControlGains g = control_get_gains();
        char b[160];
        snprintf(b, sizeof b, "search:rolling v%.0f R%.0f trk%.1f ky%.1f latk%.2f "
                 "starty%.0f goal=%d,%d explore=%d launch=N",
                 (double)v_search, (double)EX_TURN_R_MM, (double)g.track_mm,
                 (double)g.kyaw, (double)g.lat_servo_k, (double)g.start_y_mm,
                 mz->goal_x0(), mz->goal_y0(), search_explore_all() ? 1 : 0);
        flog_set_banner(b);
    }
    if (!launch_path()) { explore_abort("path refused (fusion not ready?)"); return; }
    last_seq = sensors_latest().seq;
    event(EXE_LAUNCH);
}

// ---------------------------------------------------------------- front wall
// The nearest raw front beam, mm forward from the centre (the LUT's
// convention), or WALL_FRONT_NONE_MM when both read 255 (nothing in range).
// FR saturates at ~185 mm, FL reads true to ~250, so beyond 185 this is FL.
static float front_raw_mm(const WallRanges &w) {
    float best = WALL_FRONT_NONE_MM;
    if (w.front_left_mm != 255 && (float)w.front_left_mm < best) best = (float)w.front_left_mm;
    if (w.front_right_mm != 255 && (float)w.front_right_mm < best) best = (float)w.front_right_mm;
    return best;
}

// The soft front guard fired: a wall front_mm ahead where the path drives
// straight on. Cut the path to a stop from here (the planner brakes at
// EX_ACC from the next tick), make this cell a STOP cell whose burst
// re-references and decides it, and write the wall we are looking at so the
// burst's F does not read as a contradiction of our own wrong vote. The
// next cell's record, if the decision had already pushed one, is dropped:
// the path no longer reaches it. Like rebase(), the new path's s = 0 is
// this cell's entry boundary.
static void front_stop(float s_now, float v_now, float front_mm) {
    if (ring_n >= 2 && s_now < last_rec().s_b) ring_n--;
    CellRec &c = last_rec();
    float rel = s_now - c.s_b;
    float stop_mm = v_now * v_now / (2.0f * EX_ACC_MM_S2) + 5.0f;
    SpeedPath *np = (path == &pathA) ? &pathB : &pathA;
    np->clear(0.0f, 0.0f, cardinal_yaw_deg(c.h));
    np->set_clothoid_len(EX_TURN_CLOTHOID_MM);
    np->set_limits(v_search, EX_ACC_MM_S2, EX_ALAT_MM_S2);
    if (!np->add_straight(rel + stop_mm) || !control_path_swap(np, rel)) {
        explore_abort("lost: a wall in the face, and the stop was refused");
        return;
    }
    path = np;
    float shift = c.s_b;
    for (int i = 0; i < EX_RING && i < ring_n; i++) {
        CellRec &r = rec(ring_n - 1 - i);
        r.s_b -= shift;
        r.s_p -= shift;
    }
    log_last_s -= shift;
    gate_l.disarm(); gate_r.disarm();
    gate_armed_l = gate_armed_r = false;
    c.kind = CK_STOP;
    mz->set_wall(c.cx, c.cy, c.h, true);
    stop_at_centre = true;
    stop_then_sense = true;
    stopped_at_tp = false;
    // The burst's front reading re-references the cell; should it see no
    // wall after all (a glitch fired us), the next path still starts from
    // where the mouse stands, not from a centre it is not at.
    soft_stop = true;
    soft_stop_past_mm = rel + stop_mm - 0.5f * EX_CELL_MM;
    printf_line("  front wall %.0f mm ahead where the path drives on at (%d,%d) %s: "
                "stopping to re-sense", (double)front_mm, c.cx, c.cy, heading_name(c.h));
    event(EXE_FRONT_STOP);
}

// The hard front guard fired: something inside EX_FRONT_PANIC_MM where the
// path drives on (Explore.h, "THE HARD GUARD CARRIES ON"). Emergency stop,
// then resume from where the mouse stands.
static Heading exit_heading_of(const CellRec &c) {
    switch (c.kind) {
        case CK_LEFT:  return heading_left(c.h);
        case CK_RIGHT: return heading_right(c.h);
        default:       return c.h;
    }
}
static void front_hard_stop(float s_now, const ControlPathStatus &ps, const WallRanges &w) {
    // Where the mouse stands, in the frame of the cell it is in: the path's
    // own pose at s against its pose at the cell's entry boundary, rotated
    // into the entry heading. along = 0 at the boundary, 90 at the centre;
    // lateral + = left of the entry axis.
    if (ring_n >= 2 && s_now < last_rec().s_b) ring_n--;   // the pending next cell is never reached
    CellRec &c = last_rec();
    // (A cell launched from its centre has its boundary BEFORE the path's
    // start, s_b < 0: the first segment is a straight, so extrapolate.)
    PathPose pn = path->pose(s_now);
    PathPose pb = path->pose(c.s_b < 0.0f ? 0.0f : c.s_b);
    float thb = pb.yaw_deg * DEG2RAD;
    if (c.s_b < 0.0f) { pb.x_mm += c.s_b * cosf(thb); pb.y_mm += c.s_b * sinf(thb); }
    float dx = pn.x_mm - pb.x_mm, dy = pn.y_mm - pb.y_mm;
    float along = dx * cosf(thb) + dy * sinf(thb) - 0.5f * EX_CELL_MM;   // past the centre, entry axis
    float lateral = -dx * sinf(thb) + dy * cosf(thb);
    Heading hx = exit_heading_of(c);
    float yaw_now = control_heading().yaw_deg;
    printf_line("  FRONT STOP at (%d,%d) %s: %.0f mm in the face (FL %u FR %u L %u R %u) at yaw %.0f%s, "
                "standing %+.0f along %+.0f left of the centre",
                c.cx, c.cy, heading_name(c.h), (double)walls_evaluate(w, 0).front_mm,
                w.front_left_mm, w.front_right_mm, w.left_mm, w.right_mm,
                (double)yaw_now, ps.in_arc ? " (in arc)" : "", (double)along, (double)lateral);
    // The stop itself: full-duty brake, then the path is dropped.
    control_brake_to_stop();
    control_move_abort();
    control_set_lateral_bias_deg(0.0f);
    s_total_base += s_now;
    stops++;
    // A second stop in the same cell: the way out is blocked, whatever the
    // votes said. Write the wall and let the flood find another way; an
    // edge the mouse has driven through cannot be written, and then the
    // map and the world disagree.
    if (c.cx == hard_cx && c.cy == hard_cy) hard_n++; else { hard_cx = c.cx; hard_cy = c.cy; hard_n = 1; }
    // BOTH front beams on one face (the governor's definition of a wall:
    // both in range, within FRONT_GOV_AGREE_MM) is a wall on the FIRST
    // stop. 2026-09-27, the (5,0) alcove on every search: the side beam
    // never sees the W wall of (5,0) (R 255 the whole way in -- a surface
    // it does not return from), the vote says open, the turn meets the
    // wall at FL 69 FR 74, and the mouse used to turn back to it and drive
    // at it a second time before believing it. One beam alone (the narrow
    // thing at (4,5)|(5,5): FL 54 FR 255) still needs the second stop.
    bool face = w.front_left_mm != 255 && w.front_right_mm != 255 &&
                fabsf((float)w.front_left_mm - (float)w.front_right_mm) <= FRONT_GOV_AGREE_MM;
    if (face && hard_n < 2) {
        printf_line("  both front beams on one face: %s of (%d,%d) is a wall", heading_name(hx), c.cx, c.cy);
        hard_n = 2;
    }
    if (hard_n >= 2) {
        mz->set_wall(c.cx, c.cy, hx, true);
        if (!mz->wall_present(c.cx, c.cy, hx)) {
            explore_abort("lost: stopped twice for an opening the mouse has driven through");
            return;
        }
        printf_line("  second stop here: %s of (%d,%d) written as a wall, re-routing", heading_name(hx), c.cx, c.cy);
        Heading next;
        flood_for_phase();
        if (!mz->best_step(c.cx, c.cy, hx, next)) { explore_abort("no route (flood blocked)"); return; }
        hx = next;
    }
    // How far past the centre along the heading we will leave by: the
    // first straight of the new path is shorter by this much.
    float past = (hx == c.h) ? along : (hx == heading_left(c.h)) ? lateral
                 : (hx == heading_right(c.h)) ? -lateral : -along;
    // ...but only the component ALONG the exit heading can be absorbed by
    // the path; the one ACROSS it cannot (2026-09-27: two stops in the
    // (5,0) doorway left the mouse 77 mm west of the column's centre line,
    // it turned north and launched into the corner post). Off the centre
    // by more than EX_RECENTRE_MIN_MM, the mouse first drives back to the
    // centre -- turn to face it, drive, then turn to the exit and launch
    // from the centre as a planned stop does.
    float off = sqrtf(along * along + lateral * lateral);
    if (off > EX_RECENTRE_MIN_MM) {
        // Entry frame: the centre is at (-along forward, -lateral left).
        rc_yaw_deg = wrap180f(cardinal_yaw_deg(c.h) + atan2f(-lateral, -along) / DEG2RAD);
        rc_mm = off;
        rc_phase = 1;
        past = 0.0f;
        printf_line("  %.0f mm off the centre of (%d,%d): back to it first", (double)off, c.cx, c.cy);
    } else {
        rc_phase = 0;
    }
    c.kind = CK_STOP;
    stop_turn_to = hx;
    stop_at_centre = true;
    stop_then_sense = false;      // no burst: the heading is whatever the arc left it
    stopped_at_tp = false;
    centre_err_mm = past;         // used if already facing hx (stopped_tick)
    resume_past_mm = past;        // used after the in-place turn (turn_settle_tick)
    fr_window_reset();
    state = EX_STOPPED;
    state_t0 = millis();
    event(EXE_FRONT_HALT);
}

// ---------------------------------------------------------------- roll
static void roll_tick(uint32_t now) {
    ControlPathStatus ps = control_path_status();
    if (!ps.active) {
        if (!ps.done) {
            ControlFrontGuard fg = control_front_guard();
            explore_abort(fg.cap_ticks > 100
                          ? "lost: the front governor held the mouse off a wall the path drove at"
                          : "path stalled");
            return;
        }
        // Planned stop reached: a centre (decided), or the turn point of a
        // cell that never got its decision (stale sensors, thin votes) --
        // then the burst decides from there.
        stops++;
        s_total_base += ps.s_mm;   // this path's distance joins the mission total
        control_set_lateral_bias_deg(0.0f);
        if (ring_n && last_rec().kind == CK_PENDING) {
            stopped_at_tp = true;
            stop_then_sense = true;
        }
        state = EX_STOPPED;
        state_t0 = now;
        event(EXE_STOP);
        return;
    }
    if (now - t_launch > EX_TIMEOUT_MS) { explore_abort("mission timeout"); return; }
    if (steps > EX_MAX_STEPS) { explore_abort("step budget"); return; }
    if (!ring_n) return;
    CellRec &c = last_rec();
    float s = ps.s_mm;

    WallRanges w = sensors_latest();
    if (w.seq == last_seq) {
        return;
    }
    last_seq = w.seq;
    WallView wv = walls_evaluate(w, sensors_ms_since_packet());

    // Lateral servo, straights only (the executor drops it in arcs).
    lat_valid_this = wv.valid && wv.lateral_valid;
    lat_this = lat_valid_this ? wv.lateral_mm : 0.0f;
    float bias_prev = lat_bias_cmd;
    lat_bias_cmd = lat_valid_this ? -control_get_gains().lat_servo_k * wv.lateral_mm : 0.0f;
    control_set_lateral_bias_deg(lat_bias_cmd);
    if (lat_valid_this && wv.left && wv.right && !ps.in_arc)
        fr_sample(s, wv.lateral_mm, -ps.yaw_err_deg + bias_prev);
    else
        fr_window_reset();

    // Guards that do not wait for a decision.
    if (wv.valid && w.left_mm != 255 && w.right_mm != 255 &&
        w.left_mm < EX_JAM_MM && w.right_mm < EX_JAM_MM) {
        explore_abort("lost: both side beams inside the body clearance");
        return;
    }
    if (wv.valid && wv.front && wv.front_mm < EX_FRONT_PANIC_MM &&
        !(c.kind == CK_STOP)) {
        // Say what the beams saw: 2026-09-26 13:11 the guard fired inside
        // the left turn at (5,5) on FL 68 with FR 255 and L 255 -- not a
        // wall face (the other front beam 18 deg away would have seen a
        // face 25 mm from the nose), something narrow at the (4,5) opening.
        // The console line is the only place that distinction is visible.
        front_hard_stop(s, ps, w);
        return;
    }
    // THE SOFT FRONT GUARD (Explore.h). The cell the mouse is IN: the last
    // record once past its boundary, else the one before. Only where the
    // path is committed straight through it, or not yet decided, and never
    // inside an arc or on a stop already planned.
    {
        const CellRec *cur = (s >= c.s_b || ring_n < 2) ? &c : &rec(ring_n - 2);
        bool straight_on = !ps.in_arc && !c.hold_tp && c.kind != CK_STOP &&
                           (cur->kind == CK_STRAIGHT || cur->kind == CK_PENDING);
        if (straight_on && wv.valid && wv.front && wv.front_mm < EX_FRONT_SOFT_MM) {
            if (++soft_run >= EX_FRONT_SOFT_PKTS) {
                front_stop(s, ps.v_mm_s, wv.front_mm);
                return;
            }
        } else {
            soft_run = 0;
        }
    }

    // Boundary crossing into the last record: lock the entry edge, arm the
    // wall-end gates for this boundary.
    if (!c.traversed_in && s >= c.s_b) {
        c.traversed_in = true;
        if (ring_n >= 2) {
            const CellRec &p = rec(ring_n - 2);
            mz->set_wall_traversed(p.cx, p.cy, c.h);
        }
        event(EXE_ENTER);
    }
    if (!c.gates_done && c.kind == CK_PENDING &&
        s >= c.s_b + EX_HALF_POST_MM - ex_beam_lead_mm(v_search) - EX_GATE_ARM_MM &&
        s < c.s_b - 40.0f) {
        c.gates_done = true;
        arm_gates_for(c);
    }
    if (wv.valid) {
        gate_update(gate_l, gate_armed_l, false, s, w, wv);
        gate_update(gate_r, gate_armed_r, true, s, w, wv);
    }
    // Stand the gates down once the drop window is well past.
    if (s > c.s_b + 20.0f) {
        if (gate_armed_l) { gate_l.disarm(); gate_armed_l = false; }
        if (gate_armed_r) { gate_r.disarm(); gate_armed_r = false; }
    }

    // Votes for the cell being entered.
    if (c.kind == CK_PENDING && wv.valid) {
        float rel = s - c.s_b;
        if (rel >= EX_SIDE_WIN_LO_MM && rel <= EX_SIDE_WIN_HI_MM) {
            c.n_side++;
            if (wv.left) c.n_left++;
            if (wv.right) c.n_right++;
        }
        if (rel >= EX_FRONT_WIN_LO_MM && rel <= EX_SIDE_WIN_HI_MM) {
            float exp_f = (c.s_b + EX_CELL_MM - EX_HALF_POST_MM) - s;
            c.n_front++;
            // The nearest RAW front beam against the map's expectation, not
            // the 200 mm "present" rule (Explore.h: the wall 31 mm late that
            // lost the vote). 255 is the sub-MCU's "nothing in range".
            float raw_f = front_raw_mm(w);
            if (raw_f < exp_f + EX_FRONT_TOL_MM) {
                c.n_fwall++;
                c.front_err_sum += exp_f - raw_f;
            }
        }
        // THE DECISION.
        if (rel >= EX_DECIDE_MM && !c.hold_tp) {
            if (c.n_side >= EX_MIN_VOTES && c.n_front >= 3) {
                bool L = c.n_left * 2 > c.n_side;
                bool R = c.n_right * 2 > c.n_side;
                bool F = c.n_fwall * 2 > c.n_front;
                decide(L, F, R, true, s, false);
            } else {
                defer_to_centre();
            }
        }
    }

    // The record: decimated by distance, plus events (event() records
    // immediately) -- so here only the distance sample.
    // The spacing widens as the region fills (FlightLog.h, the capacity
    // ladder): an explore-all that outruns 45 m keeps its launch.
    if (s - log_last_s >= EX_LOG_DS_MM * flog_ds_scale() || s < log_last_s)
        log_record(EXE_SAMPLE, false);
}

// ---------------------------------------------------------------- stopped
static void sense_reset() { sn_n = sn_l = sn_r = sn_f = 0; sn_front_sum = 0.0f; sn_front_n = 0; }

static void stopped_tick(uint32_t now) {
    if (now - state_t0 < 150) {   // settle
        if (now - log_last_ms >= EX_LOG_STILL_MS) log_record(EXE_SAMPLE, true);
        return;
    }
    if (finishing) {
        Heading arrived = last_rec().h;
        stop_everything();
        state = EX_IDLE;
        flog_set_outcome("completed");
        save_map_and_report("complete, back at start");
        // STRAIGHT ON TO MR2, NO POWER CYCLE (bert, 2026-09-30): the route
        // squares up on the start cell's walls exactly as a lap end does and
        // arms; the start gate (StartGate.h) holds the wave while the mouse
        // is picked up for the wheels. 'y' still dumps this search's log
        // until MR2 launches.
        if (route_run_arm_after_search(arrived)) {
            printf_line("-- search log: 'y' before the wave. Clean the wheels if you like "
                        "(pick it up: the wave is held until it is back in the start cell, "
                        "facing NORTH, back to the wall), then wave for MR2 --");
        } else {
            printf_line("-- IDLE. Clean the wheels, power-cycle: the map is saved --");
        }
        return;
    }
    if (stop_then_sense) {
        sense_reset();
        state = EX_SENSE;
        state_t0 = now;
        last_seq = sensors_latest().seq;
        return;
    }
    // After a hard stop off the centre: face the centre first (the drive
    // follows the turn, in turn_settle_tick).
    if (rc_phase == 1) {
        float yaw_rc = control_heading().yaw_deg;
        float a = wrap180f(rc_yaw_deg + frame_deg - yaw_rc);
        if (fabsf(a) > 3.0f) {
            ControlGains tg = control_get_gains();
            if (!control_turn_start(a, tg.turn_omega_dps, tg.turn_alpha_dps2)) {
                explore_abort("turn refused"); return;
            }
            state = EX_TURN;
            state_t0 = now;
            event(EXE_TURN);
            return;
        }
        state = EX_TURN_SETTLE;         // already facing it: straight to the drive
        state_t0 = now - 150;
        return;
    }
    // A planned about-face (or a straight-on after a burst decided so).
    CellRec &c = last_rec();
    float yaw_now = control_heading().yaw_deg;
    float ang = wrap180f(cardinal_yaw_deg(stop_turn_to) + frame_deg - yaw_now);
    if (fabsf(ang) > 3.0f) {
        ControlGains tg = control_get_gains();
        if (!control_turn_start(ang, tg.turn_omega_dps, tg.turn_alpha_dps2)) {
            explore_abort("turn refused"); return;
        }
        state = EX_TURN;
        state_t0 = now;
        printf_line("  turn %+.0f deg to %s at (%d,%d)", (double)ang,
                    heading_name(stop_turn_to), c.cx, c.cy);
        event(EXE_TURN);
        return;
    }
    // Already facing it: new path from the centre, re-referenced on the
    // front wall if the burst saw one.
    if (!path_from_centre(c.cx, c.cy, stop_turn_to, centre_err_mm)) { explore_abort("path refused"); return; }
    centre_err_mm = 0.0f;
    resume_past_mm = 0.0f;   // a hard stop that needed no turn used centre_err_mm instead
    if (!launch_path()) { explore_abort("path refused (fusion not ready?)"); return; }
    last_seq = sensors_latest().seq;
}

static void sense_tick(uint32_t now) {
    WallRanges w = sensors_latest();
    if (w.seq != last_seq) {
        last_seq = w.seq;
        WallView wv = walls_evaluate(w, sensors_ms_since_packet());
        if (wv.valid) {
            sn_n++;
            if (wv.left) sn_l++;
            if (wv.right) sn_r++;
            // 84 at the centre; at the turn point (boundary + lead) the post
            // face is CELL - lead - half post = 147 (was written R + 84 =
            // 139, an 8 mm bias on every turn-point re-reference; 2026-09-26).
            float exp_f = (stopped_at_tp ? EX_CELL_MM - ex_turn_lead_mm() : 0.5f * EX_CELL_MM) -
                          EX_HALF_POST_MM;
            float raw_f = front_raw_mm(w);   // FL reaches the 144 + 60 the turn point needs; FR does not
            if (raw_f < exp_f + EX_FRONT_TOL_MM) {
                sn_f++;
                sn_front_sum += exp_f - raw_f;
                sn_front_n++;
            }
        }
    }
    if (now - log_last_ms >= EX_LOG_STILL_MS) log_record(EXE_SAMPLE, true);
    if (now - state_t0 < 200 || sn_n < 12) {
        if (now - state_t0 > 1500) { explore_abort("no sensor packets while sensing"); }
        return;
    }
    bool L = sn_l * 2 > sn_n, R = sn_r * 2 > sn_n, F = sn_f * 2 > sn_n;
    CellRec &c = last_rec();
    // The stationary front distance re-references the cell (the old
    // search's shuffle, applied to the map's idea of where the centre is
    // rather than by driving): reported, and folded into the next path
    // as a shorter or longer first straight.
    float centre_err = (F && sn_front_n) ? sn_front_sum / (float)sn_front_n
                       : (soft_stop ? soft_stop_past_mm : 0.0f);
    soft_stop = false;
    centre_err_mm = centre_err;
    printf_line("  burst at (%d,%d) %s: L%d F%d R%d (%d pkts), front says %+.0f mm past centre",
                c.cx, c.cy, heading_name(c.h), L, F, R, sn_n, (double)centre_err);
    event(EXE_SENSE);
    // Decide as if rolling, but from the centre: a straight/left/right
    // becomes a turn-in-place (or none) then a fresh path.
    Heading HL = heading_left(c.h), HR = heading_right(c.h);
    mz->set_wall(c.cx, c.cy, HL, L);
    mz->set_wall(c.cx, c.cy, c.h, F);
    mz->set_wall(c.cx, c.cy, HR, R);
    mz->set_visited(c.cx, c.cy);
    bool bad = (L && mz->wall_known(c.cx, c.cy, HL) && !mz->wall_present(c.cx, c.cy, HL)) ||
               (F && mz->wall_known(c.cx, c.cy, c.h) && !mz->wall_present(c.cx, c.cy, c.h)) ||
               (R && mz->wall_known(c.cx, c.cy, HR) && !mz->wall_present(c.cx, c.cy, HR));
    if (bad) { explore_abort("lost: a wall where the map has a traversed opening"); return; }
    if (at_phase_target(c.cx, c.cy)) {
        if (phase == PH_TO_GOAL) {
            phase = search_explore_all() ? PH_EXPLORE : PH_TO_START;
            printf_line("-- GOAL reached in %d steps --", steps);
            event(EXE_GOAL);
        } else if (phase == PH_EXPLORE) {
            phase = PH_TO_START;
            printf_line("-- MAP COMPLETE in %d steps -- returning to start --", steps);
            event(EXE_MAPPED);
        } else {
            finishing = true;
            state = EX_STOPPED; state_t0 = now - 150;
            event(EXE_HOME);
            return;
        }
    }
    Heading next;
    flood_for_phase();
    if (!mz->best_step(c.cx, c.cy, c.h, next)) { explore_abort("no route (flood blocked)"); return; }
    steps++;
    stop_then_sense = false;
    if (stopped_at_tp) {
        // Standing at the turn point: the path resumes from here exactly as
        // a rolling decision would have extended it. The burst's front
        // reading says how far PAST the true turn point we stand (+) or
        // short of it (-): short, the extension begins with that much
        // straight so the arc starts where it should; past, the arc starts
        // here and the exit lands that much early -- the next boundary's
        // anchor absorbs it -- but a straight-on cell keeps the true
        // boundary in its record.
        stopped_at_tp = false;
        float past = centre_err;
        if (past > EX_FRONT_TOL_MM) past = EX_FRONT_TOL_MM;
        if (past < -EX_FRONT_TOL_MM) past = -EX_FRONT_TOL_MM;
        centre_err_mm = 0.0f;
        path->clear(0.0f, 0.0f, cardinal_yaw_deg(c.h));
        path->set_clothoid_len(EX_TURN_CLOTHOID_MM);
        path->set_limits(v_search, EX_ACC_MM_S2, EX_ALAT_MM_S2);
        gate_l.disarm(); gate_r.disarm();
        gate_armed_l = gate_armed_r = false;
        if (past < 0.0f) path->add_straight(-past);
        c.s_p = path->length_mm();
        c.s_b = c.s_p - ex_turn_lead_mm();
        c.kind = CK_PENDING;
        Heading HL2 = heading_left(c.h), HR2 = heading_right(c.h);
        CellKind k = (next == c.h) ? CK_STRAIGHT : (next == HL2) ? CK_LEFT
                     : (next == HR2) ? CK_RIGHT : CK_STOP;
        if (!extend_by(k)) { explore_abort("path full"); return; }
        if (k == CK_STRAIGHT && past > 0.0f) {
            last_rec().s_b -= past;
            last_rec().s_p -= past;
        }
        if (k == CK_STOP) { stop_turn_to = next; stop_at_centre = true; }
        event(k == CK_STRAIGHT ? EXE_STRAIGHT : k == CK_LEFT ? EXE_LEFT
              : k == CK_RIGHT ? EXE_RIGHT : EXE_BACK);
        if (!launch_path()) { explore_abort("path refused (fusion not ready?)"); return; }
        last_seq = sensors_latest().seq;
        return;
    }
    stop_turn_to = next;
    c.kind = CK_STOP;
    state = EX_STOPPED;
    state_t0 = now - 150;   // no second settle
}

static void turn_tick(uint32_t now) {
    ControlMoveStatus st = control_move_status();
    if (now - log_last_ms >= EX_LOG_STILL_MS) log_record(EXE_SAMPLE, true);
    if (st.active) return;
    if (!st.done) { explore_abort("turn did not settle"); return; }
    state = EX_TURN_SETTLE;
    state_t0 = now;
}

static void recentre_tick(uint32_t now) {
    ControlMoveStatus st = control_move_status();
    if (now - log_last_ms >= EX_LOG_STILL_MS) log_record(EXE_SAMPLE, true);
    if (st.active && now - state_t0 < 3000) return;
    if (st.active) control_move_abort();   // the governor held it short: good enough
    rc_phase = 0;
    centre_err_mm = 0.0f;
    resume_past_mm = 0.0f;
    state = EX_STOPPED;                    // -> the turn to the exit, then the path
    state_t0 = now - 150;
}

static void turn_settle_tick(uint32_t now) {
    if (now - state_t0 < 150) return;
    if (rc_phase == 1) {
        // Facing the centre: drive to it.
        if (!control_move_start(rc_mm, EX_RECENTRE_V, EX_RECENTRE_ACC)) {
            explore_abort("recentre refused"); return;
        }
        rc_phase = 2;
        state = EX_RECENTRE;
        state_t0 = now;
        return;
    }
    CellRec &c = last_rec();
    // After a turn the along-track error became lateral; the servo owns it
    // -- except after a hard front stop, which computed where it stands
    // along the NEW heading (front_hard_stop, resume_past_mm).
    centre_err_mm = 0.0f;
    float past = resume_past_mm;
    resume_past_mm = 0.0f;
    if (!path_from_centre(c.cx, c.cy, stop_turn_to, past)) { explore_abort("path refused"); return; }
    if (!launch_path()) { explore_abort("path refused (fusion not ready?)"); return; }
    last_seq = sensors_latest().seq;
}

// ---------------------------------------------------------------- poll
void explore_poll() {
    if (state == EX_IDLE) return;
    uint32_t now = millis();
    sensors_poll();
    led_service(now);
    switch (state) {
        case EX_CAL:
            if (!cal_started) {
                if (now - state_t0 < 300) return;
                control_recalibrate();
                cal_started = true;
                printf_line("search: IMU calibrating (~3 s, stationary)...");
                return;
            }
            // The tally runs through the whole stationary window; the
            // verdict is asked for once the IMU is ready, and go is
            // conditional on it (see preflight_pass).
            if (!control_heading().ready) { preflight_sample(); return; }
            if (!preflight_pass(now)) return;
            launch(now);
            return;
        case EX_ROLL: roll_tick(now); return;
        case EX_STOPPED: stopped_tick(now); return;
        case EX_SENSE: sense_tick(now); return;
        case EX_TURN: turn_tick(now); return;
        case EX_TURN_SETTLE: turn_settle_tick(now); return;
        case EX_RECENTRE: recentre_tick(now); return;
        default: return;
    }
}

bool explore_running() { return state != EX_IDLE; }
