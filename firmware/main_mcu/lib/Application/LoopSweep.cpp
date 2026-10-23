#include "LoopSweep.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "Control.h"
#include "Fram.h"
#include "HAL.h"
#include "Params.h"  // params_crc32
#include "Path.h"
#include "Sensors.h"
#include "WallAlign.h"
#include "WallGate.h"
#include "Walls.h"

// ---------------------------------------------------------------- config
static SweepGrid g_grid;
static bool g_left = false;  // false = clockwise = right turns
static float g_trace_step = SWEEP_TRACE_STEP_DEFAULT_MM;

static void (*prn)(const char *) = nullptr;
static void printf_line(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void printf_line(const char *fmt, ...) {
    if (!prn) return;
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    prn(buf);
}

void sweep_set_printer(void (*printer)(const char *line)) { prn = printer; }

// ---------------------------------------------------------------- FRAM header
struct SweepHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;  // bit0 left turns
    uint32_t session;
    SweepGrid grid;
    float trace_step;
    uint16_t combo_ix;   // NEXT combo to fly
    uint16_t loops_done;
    uint32_t corners;    // summary records written
    uint32_t trace_off;  // next free byte offset in the trace region
    uint8_t trace_wrapped;
    uint8_t pad[3];
    // Stationary lateral-accelerometer reading, mm/s^2, measured at the end
    // of the IMU calibration when the mouse is provably not moving. Gravity
    // removal leaves the sensor's own Y bias behind, and 0.01 g of it is
    // 98 mm/s^2 -- a third of the lateral demand at R125/v200. Subtracting a
    // measured zero is the difference between "the tyres gave 250 of the 320
    // asked for" and a number that cannot be read at all.
    int16_t ay_bias_mm_s2;
    uint8_t pad2[2];
    // Beam health for the session: how many two-beam packets did not
    // describe one corridor. See the note where these are counted.
    uint32_t pair_samples;
    uint32_t suspect_pairs;
    ControlGains gains;  // what the trim was when the session started
    uint32_t crc;
};
static_assert(sizeof(SweepHeader) <= SWEEP_HDR_BYTES, "sweep header too big");

static SweepHeader hdr;

static void hdr_write() {
    hdr.magic = SWEEP_MAGIC;
    hdr.version = SWEEP_VERSION;
    hdr.flags = g_left ? 1 : 0;
    hdr.grid = g_grid;
    hdr.trace_step = g_trace_step;
    hdr.crc = params_crc32(&hdr, sizeof(hdr) - sizeof(uint32_t));
    fram_write(SWEEP_BASE, &hdr, sizeof(hdr));
}

void sweep_load_defaults() {
    g_grid.n_r = sizeof(SWEEP_DEF_R) / sizeof(SWEEP_DEF_R[0]);
    for (int i = 0; i < g_grid.n_r; i++) g_grid.r_mm[i] = SWEEP_DEF_R[i];
    g_grid.n_v = sizeof(SWEEP_DEF_V) / sizeof(SWEEP_DEF_V[0]);
    for (int i = 0; i < g_grid.n_v; i++) g_grid.v_arc[i] = SWEEP_DEF_V[i];
    g_grid.n_slew = sizeof(SWEEP_DEF_SLEW) / sizeof(SWEEP_DEF_SLEW[0]);
    for (int i = 0; i < g_grid.n_slew; i++) g_grid.slew[i] = SWEEP_DEF_SLEW[i];
    g_grid.n_alon = sizeof(SWEEP_DEF_A_LON) / sizeof(SWEEP_DEF_A_LON[0]);
    for (int i = 0; i < g_grid.n_alon; i++) g_grid.a_lon[i] = SWEEP_DEF_A_LON[i];
    g_grid.n_rep = SWEEP_DEF_REPS;
    g_left = SWEEP_DIR_LEFT_DEFAULT;
    g_trace_step = SWEEP_DEF_TRACE_MM;
    // NOTE: no grid_changed() here. This is the boot path, and wiping the
    // stored progress before the loader has looked at it would throw away
    // the session a battery swap is meant to continue.
}

// Does a stored session describe the same experiment the firmware is
// currently compiled for? If not, its records were taken under different
// numbers and must not be appended to or silently discarded.
static bool grid_matches(const SweepHeader &h) {
    if (h.grid.n_r != g_grid.n_r || h.grid.n_v != g_grid.n_v ||
        h.grid.n_slew != g_grid.n_slew || h.grid.n_rep != g_grid.n_rep ||
        h.grid.n_alon != g_grid.n_alon)
        return false;
    for (int i = 0; i < g_grid.n_r; i++)
        if (h.grid.r_mm[i] != g_grid.r_mm[i]) return false;
    for (int i = 0; i < g_grid.n_v; i++)
        if (h.grid.v_arc[i] != g_grid.v_arc[i]) return false;
    for (int i = 0; i < g_grid.n_slew; i++)
        if (h.grid.slew[i] != g_grid.slew[i]) return false;
    for (int i = 0; i < g_grid.n_alon; i++)
        if (h.grid.a_lon[i] != g_grid.a_lon[i]) return false;
    if (((h.flags & 1) != 0) != g_left) return false;   // direction counts
    if (fabsf(h.trace_step - g_trace_step) > 0.01f) return false;
    return true;
}

// True once a valid stored session exists, whatever grid it used.
static bool stored_valid = false;
static bool stored_matches = false;
// Something IS in the region but this firmware cannot read it: a sweep header
// from another version, a corrupt one, or a flight log from a search. Not
// "empty" -- see the note in sweep_resume_load.
static bool stored_foreign = false;

// The flight recorder's magic, checked here rather than by calling into
// FlightLog: the two share SWEEP_BASE (FLOG_BASE is the same address), and
// the sweep must be able to recognise a search log without depending on the
// recorder's translation unit -- the host test build links this file alone.
constexpr uint32_t SWEEP_FLOG_MAGIC = 0x5457334C;  // "TW3L"

bool sweep_resume_load() {
    SweepHeader h;
    fram_read(SWEEP_BASE, &h, sizeof(h));
    stored_valid = false;
    stored_matches = false;
    stored_foreign = false;
    if (h.magic != SWEEP_MAGIC || h.version != SWEEP_VERSION ||
        h.crc != params_crc32(&h, sizeof(h) - sizeof(uint32_t))) {
        // UNREADABLE IS NOT EMPTY. This used to report "no stored session"
        // for every one of these, and sweep_arm() then called sweep_restart()
        // at boot -- stamping a fresh empty sweep header over whatever was
        // there. Two ways that destroys data with no gesture from anyone:
        //   - a flight log from a search (run 150). FLOG_BASE == SWEEP_BASE,
        //     so booting a SWEEP_ARM_AT_BOOT build after a search wiped the
        //     search's log before 'y' could ever read it.
        //   - a sweep session from the previous SWEEP_VERSION. Bump the
        //     version to change the experiment and the first boot after the
        //     reflash throws away the set that motivated the change.
        // Only a region that reads as neither is treated as free.
        if (h.magic == SWEEP_MAGIC || h.magic == SWEEP_FLOG_MAGIC) {
            stored_foreign = true;
            printf_line("sweep: FRAM holds %s this firmware cannot append to. "
                        "It is NOT overwritten: dump it ('sweep.dump' / 'y'), "
                        "or hold the hand %lu s to discard and start fresh.",
                        h.magic == SWEEP_FLOG_MAGIC
                            ? "a FLIGHT log (a search)"
                            : "a sweep session from another version",
                        (unsigned long)(SWEEP_WIPE_HOLD_MS / 1000));
            return false;
        }
        printf_line("sweep: no stored session -- a wave starts a fresh one");
        return false;
    }
    stored_valid = true;
    stored_matches = grid_matches(h);
    if (!stored_matches) {
        // Keep the compiled grid in force but DO NOT touch the stored data.
        // The header stays as it is until a deliberate wipe, so a reflash
        // with different numbers can never quietly destroy a set.
        hdr = h;
        printf_line("sweep: FRAM holds session %lu from a DIFFERENT grid "
                    "(%lu corners). Dump it, or hold the hand %lu s to "
                    "discard and start fresh.",
                    (unsigned long)h.session, (unsigned long)h.corners,
                    (unsigned long)(SWEEP_WIPE_HOLD_MS / 1000));
        return true;
    }
    hdr = h;
    if (hdr.combo_ix < g_grid.combos()) {
        printf_line("sweep: session %lu resumes at combo %u of %d "
                    "(%lu corners logged) -- wave to continue",
                    (unsigned long)hdr.session, (unsigned)hdr.combo_ix,
                    g_grid.combos(), (unsigned long)hdr.corners);
    } else {
        printf_line("sweep: session %lu COMPLETE (%lu corners). Dump it, or "
                    "hold the hand %lu s to start another set.",
                    (unsigned long)hdr.session, (unsigned long)hdr.corners,
                    (unsigned long)(SWEEP_WIPE_HOLD_MS / 1000));
    }
    return true;
}

SweepReadiness sweep_readiness() {
    // A region we cannot read still belongs to someone. NEEDS_WIPE means a
    // wave does nothing and only the deliberate 5 s hold clears it, which is
    // what protects a search log or a previous version's session.
    if (stored_foreign) return SWEEP_READY_NEEDS_WIPE;
    if (!stored_valid) return SWEEP_READY_FRESH;
    if (!stored_matches) return SWEEP_READY_NEEDS_WIPE;
    if (hdr.combo_ix >= g_grid.combos()) return SWEEP_READY_NEEDS_WIPE;
    return SWEEP_READY_RESUME;
}

void sweep_restart() {
    uint32_t session = hdr.magic == SWEEP_MAGIC ? hdr.session + 1 : 1;
    hdr = SweepHeader{};   // value-init: SweepGrid and ControlGains have
                           // default member initialisers, so memset is wrong
    hdr.session = session;
    hdr.trace_off = 0;
    hdr.gains = control_get_gains();
    hdr_write();
    stored_valid = true;
    stored_matches = true;
    stored_foreign = false;
    printf_line("sweep: session %lu started, log cleared",
                (unsigned long)session);
}

// ---------------------------------------------------------------- setters
static void grid_changed() {
    hdr.combo_ix = 0;
    hdr.loops_done = 0;
    hdr.corners = 0;
    hdr.trace_off = 0;
    hdr.trace_wrapped = 0;
}

bool sweep_set_radii(const uint8_t *r, int n) {
    if (n < 1 || n > SWEEP_MAX_R) return false;
    for (int i = 0; i < n; i++) {
        if (r[i] < 45 || r[i] > 125) return false;
    }
    g_grid.n_r = n;
    for (int i = 0; i < n; i++) g_grid.r_mm[i] = r[i];
    grid_changed();
    return true;
}

bool sweep_set_speeds(const uint16_t *v, int n) {
    if (n < 1 || n > SWEEP_MAX_V) return false;
    for (int i = 0; i < n; i++) {
        if (v[i] < 80 || v[i] > 900) return false;
    }
    g_grid.n_v = n;
    for (int i = 0; i < n; i++) g_grid.v_arc[i] = v[i];
    grid_changed();
    return true;
}

bool sweep_set_slews(const uint16_t *s, int n) {
    if (n < 1 || n > SWEEP_MAX_SLEW) return false;
    for (int i = 0; i < n; i++) {
        if (s[i] < 500 || s[i] > 20000) return false;
    }
    g_grid.n_slew = n;
    for (int i = 0; i < n; i++) g_grid.slew[i] = s[i];
    grid_changed();
    return true;
}

void sweep_set_direction(bool left_turns) {
    g_left = left_turns;
    grid_changed();
}

void sweep_set_trace_step(float mm) {
    if (mm < 0.0f) mm = 0.0f;
    if (mm > 0.0f && mm < 1.0f) mm = 1.0f;
    g_trace_step = mm;
    grid_changed();
}

const SweepGrid &sweep_grid() { return g_grid; }
bool sweep_left_turns() { return g_left; }
float sweep_trace_step() { return g_trace_step; }

// ---------------------------------------------------------------- budget
// Records per corner at the configured step, for the largest radius in the
// grid (the shortest window, so this UNDER-estimates -- use the smallest
// radius for the worst case).
static int trace_recs_per_corner(float radius_mm) {
    if (g_trace_step <= 0.0f) return 0;
    float win = 2.0f * (SWEEP_TRACE_HALF_MM - radius_mm) +
                radius_mm * (float)M_PI / 2.0f;
    return (int)(win / g_trace_step) + 2;
}

// Fraction of an arc spent slewing the commanded rate up to v/R. See
// SWEEP_MAX_RAMP_FRAC -- this is the number the report prints and the number
// loop_launch declines on, computed once so they cannot drift apart.
static float ramp_frac(float R, float v, float slew) {
    if (R <= 0.0f || slew <= 0.0f) return 0.0f;
    float w = v / R * 57.29578f;          // deg/s
    float ramp = v * (w / slew);          // mm covered while ramping
    return ramp / (R * (float)M_PI / 2.0f);
}

void sweep_report() {
    if (!g_grid.valid()) {
        printf_line("sweep: grid not configured (need radii, speeds, slews)");
        return;
    }
    int combos = g_grid.combos();
    printf_line("sweep grid: %d radii x %d speeds x %d slews = %d combos "
                "= %d loops, %s turns",
                g_grid.n_r, g_grid.n_v, g_grid.n_slew, combos, combos,
                g_left ? "LEFT (anticlockwise)" : "RIGHT (clockwise)");
    if (SWEEP_ARC_TRIM_MM != 0.0f) {
        printf_line("  ARC TRIM ACTIVE: every arc starts %+.0f mm along the "
                    "path (positive = turns EARLIER = exit pulled inward). "
                    "Watch mean exit_lat: near 0 means the standing offset "
                    "was one wrong constant in the gate geometry.",
                    (double)SWEEP_ARC_TRIM_MM);
    }
    {
        char b[160];
        int o = snprintf(b, sizeof b, "  R:");
        for (int i = 0; i < g_grid.n_r; i++)
            o += snprintf(b + o, sizeof(b) - o, " %u", g_grid.r_mm[i]);
        o += snprintf(b + o, sizeof(b) - o, "   v_arc:");
        for (int i = 0; i < g_grid.n_v; i++)
            o += snprintf(b + o, sizeof(b) - o, " %u", g_grid.v_arc[i]);
        o += snprintf(b + o, sizeof(b) - o, "   slew:");
        for (int i = 0; i < g_grid.n_slew; i++)
            o += snprintf(b + o, sizeof(b) - o, " %u", g_grid.slew[i]);
        if (prn) prn(b);
    }
    // Implied lateral budget per combination -- the number that decides
    // whether a combo is even physically sane.
    printf_line("  implied a_lat = v^2/R:");
    for (int ri = 0; ri < g_grid.n_r; ri++) {
        char b[160];
        int o = snprintf(b, sizeof b, "    R%3u:", g_grid.r_mm[ri]);
        for (int vi = 0; vi < g_grid.n_v; vi++) {
            float v = g_grid.v_arc[vi];
            float a = v * v / (float)g_grid.r_mm[ri];
            o += snprintf(b + o, sizeof(b) - o, " %5.0f", a);
        }
        o += snprintf(b + o, sizeof(b) - o, "  mm/s2");
        if (prn) prn(b);
    }
    // The corridor axis, and what it buys. Printed at arm time so the
    // console says what the session is actually about before it moves.
    if (g_grid.n_alon > 0) {
        char b[200];
        int o = snprintf(b, sizeof b, "  a_lon (corridor):");
        for (int i = 0; i < g_grid.n_alon; i++)
            o += snprintf(b + o, sizeof(b) - o, " %u", g_grid.a_lon[i]);
        o += snprintf(b + o, sizeof(b) - o, " mm/s2   v_max %.0f",
                      (double)SWEEP_STRAIGHT_V_MAX);
        if (prn) prn(b);
        o = snprintf(b, sizeof b, "    peak on the straight:");
        for (int i = 0; i < g_grid.n_alon; i++) {
            float v0 = g_grid.v_arc[0];
            float st = SWEEP_SIDE_MM - 2.0f * 98.6f;   // R84 tangent
            float pk = sqrtf(v0 * v0 + (float)g_grid.a_lon[i] * st);
            if (pk > SWEEP_STRAIGHT_V_MAX) pk = SWEEP_STRAIGHT_V_MAX;
            o += snprintf(b + o, sizeof(b) - o, " %4.0f", pk);
        }
        o += snprintf(b + o, sizeof(b) - o, " mm/s");
        if (prn) prn(b);
    }
    // Ramp fraction: how much of each arc is spent slewing the commanded
    // rate up to v/R rather than holding it. This is the curvature step made
    // visible, and it is why the slew rail is a swept axis at all. Anything
    // over ~50% is barely a corner; anything marked * is over
    // SWEEP_MAX_RAMP_FRAC and will be declined rather than flown.
    for (int si = 0; si < g_grid.n_slew; si++) {
        printf_line("  ramp %% of arc at slew %u:", g_grid.slew[si]);
        for (int ri = 0; ri < g_grid.n_r; ri++) {
            char b[160];
            int o = snprintf(b, sizeof b, "    R%3u:", g_grid.r_mm[ri]);
            for (int vi = 0; vi < g_grid.n_v; vi++) {
                float f = ramp_frac((float)g_grid.r_mm[ri],
                                    (float)g_grid.v_arc[vi],
                                    (float)g_grid.slew[si]);
                // A cell the grid will DECLINE is marked, not just printed:
                // reading 118 and reading "this one will not be flown" are
                // different pieces of information.
                o += snprintf(b + o, sizeof(b) - o, "%4.0f%c", 100.0f * f,
                              f > SWEEP_MAX_RAMP_FRAC ? '*' : ' ');
            }
            if (prn) prn(b);
        }
    }
    // FRAM budget. Worst case = smallest radius (longest capture window).
    uint8_t rmin = 255;
    for (int i = 0; i < g_grid.n_r; i++)
        if (g_grid.r_mm[i] < rmin) rmin = g_grid.r_mm[i];
    int per = trace_recs_per_corner((float)rmin);
    // 4 corners + 1 launch row + 1 spare for a loop-event row if it stops
    // between corners. 216 B a loop, and it means a session can always
    // explain both how it launched and how it ended.
    uint32_t sum_need = (uint32_t)combos * 6 * sizeof(SweepSummary);
    // Breadcrumbs: the loop's corridor is everything that is not a capture
    // window, sampled every SWEEP_BREADCRUMB_MM.
    float lap = 4.0f * SWEEP_SIDE_MM;
    float windows = 4.0f * (2.0f * (SWEEP_TRACE_HALF_MM - (float)rmin) +
                            (float)rmin * (float)M_PI / 2.0f);
    float corridor = lap - windows;
    int crumbs = corridor > 0.0f ? (int)(corridor / SWEEP_BREADCRUMB_MM) + 4 : 0;
    uint32_t tr_need =
        (uint32_t)combos * (4 * per + crumbs) * sizeof(SweepTrace);
    uint32_t tr_have = SWEEP_TRACE_END - SWEEP_TRACE_BASE;
    printf_line("  summaries: %lu B of %lu (%s)",
                (unsigned long)sum_need, (unsigned long)SWEEP_SUM_BYTES,
                sum_need <= SWEEP_SUM_BYTES ? "fits" : "OVERFLOWS");
    if (g_trace_step > 0.0f) {
        printf_line("  traces: %d rec/corner at %.1f mm + %d breadcrumbs/loop "
                    "-> %lu KB of %lu KB (%s)",
                    per, (double)g_trace_step, crumbs,
                    (unsigned long)(tr_need / 1024),
                    (unsigned long)(tr_have / 1024),
                    tr_need <= tr_have ? "fits"
                                       : "WILL WRAP -- oldest traces lost");
    } else {
        printf_line("  traces: disabled");
    }
    printf_line("  progress: combo %u of %d, %u loops, %lu corners logged",
                (unsigned)hdr.combo_ix, combos, (unsigned)hdr.loops_done,
                (unsigned long)hdr.corners);
}

// ---------------------------------------------------------------- run state
enum SweepState : uint8_t {
    W_IDLE,
    W_ARMED,       // watching the front beam for a hand
    W_HAND_CLEAR,  // hand seen: wait for it to go
    W_CAL,         // stillness, then IMU recalibration
    W_SQUARE,      // RETIRED (run 140): squaring turned the body in place and
                   // cost more than it fixed. See the note on `lane`. Kept as
                   // a slot so the enum's numbering does not shift.
    W_SETTLE,      // tyres relax
    W_FLY,         // flying one loop
    W_DONE,
};
static SweepState state = W_IDLE;
static uint32_t state_t0 = 0;
static uint32_t session_t0 = 0;
static uint32_t last_seq = 0;
static int hand_run = 0;
static bool hand_cal_started = false;
// Lateral-accelerometer zero, taken during W_CAL while the mouse is still.
constexpr uint32_t SWEEP_AY_CAL_MS = 500;
static uint32_t ay_cal_t0 = 0;
static int ay_cal_n = 0;
static float ay_cal_sum = 0.0f;
static bool hand_active = false;      // a gesture is in progress
static uint32_t hand_t0 = 0;          // millis the hand passed the debounce
static uint32_t hand_seen_ms = 0;
static uint32_t hand_next_tick = 0;
static bool hand_wiped = false;       // the 5 s wipe already fired
static uint32_t led_last = 0;
static bool led_phase = false;

// Hand-start thresholds. Deliberately the same numbers as the search's
// (Search.cpp): one gesture, one meaning, whatever the mouse is about to do.
static constexpr float W_HAND_START_MM = 100.0f;
static constexpr float W_HAND_CLEAR_MM = 150.0f;
static constexpr int W_HAND_TRIGGER_N = 30;
static constexpr int W_HAND_CLEAR_N = 30;
static constexpr uint32_t W_HAND_SETTLE_MS = 700;

PATH_STORAGE static SpeedPath path;
static WallGate gate;
static float cur_R = 60.0f, cur_v = 245.0f, cur_slew = 4000.0f;
static float cur_a_lon = PATH_A_LON_DEFAULT;
static int cur_slew_ix = 0;

// Per-loop / per-corner bookkeeping.
static int corner_ix = 0;         // next corner to be gated/measured
static int gate_miss_streak = 0;
static int lat_over_run = 0;   // consecutive packets over the lateral limit
static float loop_yaw0 = 0.0f;
static float next_trace_s = 0.0f;
static float next_crumb_s = 0.0f;   // corridor breadcrumbs, between windows
static SweepSummary rec;
static bool rec_open = false;
static int entry_n = 0, exit_n = 0;
static float entry_sum = 0.0f, exit_sum = 0.0f;
static uint32_t arc_t0 = 0;
static float arc_yaw0 = 0.0f;   // heading at the entry tangent point
static float arc_ay_sum = 0.0f; // steady-arc lateral accel accumulator
static int arc_ay_n = 0;

// ---- corridor squaring -------------------------------------------------------
// WHAT REPLACED square_up(), and why the old one had to go.
//
// The ring closes 360 degrees, so a clean loop returns to its launch heading
// and any residual would rotate the next loop's frame. square_up() addressed
// that by comparing the FUSED yaw against the nearest cardinal and, past a
// 1.5 degree deadband, turning the body in place to remove the difference.
//
// Both halves of that are wrong, and run 140 charged for both.
//
// THE REFERENCE. Fused yaw drifts. Over a 17-loop session the drift is a
// property of the gyro, not of where the mouse is standing -- corner 0's
// entry lateral held flat at about -6 mm for the whole run, so the mouse's
// PHYSICAL alignment was never the thing going wrong. square_up() read the
// accumulated drift as a physical error. A deadband makes that worse rather
// than safer: every silent lap lets the drift grow, and the one lap that
// finally trips the threshold dumps the whole accumulation into the body in
// a single turn.
//
// THE ACTUATION. An in-place turn is the most expensive way to change a
// heading. At two degrees with a 2000 deg/s^2 ramp the whole manoeuvre is
// about 60 ms and never leaves the acceleration phase, so the wheels break
// out of stiction unevenly and the body translates. Run 140's one firing
// moved the mouse from L89/R81 to L98/R75 -- seven millimetres FURTHER off
// centre than the error it was called to fix. Ten other boundaries in the
// same run, where it stayed silent, held position to within a millimetre.
//
// WHAT REPLACES IT. The corridor walls are the reference the gyro was
// pretending to be, and WallAlign already fits them: (distance, lateral)
// least-squares over a straight leg gives the travel direction relative to
// the wall, so corridor_yaw = yaw - eps. Two consequences follow, and
// neither of them moves the mouse:
//
//   1. The fused frame is re-referenced between loops so the corridor reads
//      an exact cardinal. That is square_up()'s STATED purpose -- stopping
//      the frame rotating over a session -- done to the datum instead of to
//      the body, which is where the error actually lived.
//   2. The next loop's path is launched on the corridor rather than on the
//      body (control_path_start's yaw0 offset). A body that is standing a
//      degree or two off then flies a path that is straight down the
//      corridor, and the misalignment drains as a heading transient while
//      the mouse accelerates -- rolling, not scrubbing.
//
// The fit runs on every corridor leg and resets at each junction, so the
// value carried into the launch is the last CLEAN leg of the lap. Using the
// mouse's own wheel odometry as the fit's distance axis, not path s, keeps a
// mid-leg gate correction from injecting a step into the slope.
static WallAlign lane;
static float lane_yaw_ref = 0.0f;   // unwrap datum for the leg's yaw mean
static float lane_yaw_sum = 0.0f;
static int lane_yaw_n = 0;
static bool lane_open = false;      // a leg's fit is accumulating
static bool sq_valid = false;         // a corridor direction is known
static float sq_corridor_yaw = 0.0f;  // corridor's bearing in the fused frame
static float sq_cardinal = 0.0f;      // what the corridor reads after squaring
static float sq_eps_deg = 0.0f;     // last fitted heading error, for the log
static float sq_corr_deg = 0.0f;    // last re-reference applied, for the log
static uint8_t sq_status = SWEEP_SQ_NONE;   // what squaring did, for the log
static int sq_fit_n = 0;            // fit size at the moment it was used
static float sq_fit_span = 0.0f;
static int sq_fit_rej = 0;

// Largest launch frame offset worth applying. Beyond this the fit is more
// likely wrong than the mouse is crooked, and a bad frame offset rotates the
// entire trajectory -- the one failure mode the body-relative default was
// chosen to avoid. WALL_ALIGN_MAX_CORR_DEG (10) bounds the fit itself; this
// is the tighter operational bound on acting for real.
constexpr float SWEEP_SQ_MAX_DEG = 6.0f;

static float wrap180(float d) {
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}
// Latch ONCE per corner. The first build keyed these off (s >= entry_s) with
// an arc_t0 == 0 guard, which re-armed the moment the exit cleared arc_t0 --
// so every poll after the arc re-stamped both fields and reset the clock.
// Run 116 shows the result: entry and exit yaw error identical (-4.67) and
// corner_ms = 0, on a corner that plainly took a third of a second.
static bool entry_latched = false;
static bool exit_latched = false;
static int beam_close_run = 0;
static int fail_streak = 0;
static uint32_t slip0 = 0;
static float odo0 = 0.0f;
static uint32_t rec_trace_off = 0;
static bool rec_tracing = false;

// -- FRAM writes ---------------------------------------------------------------
static void summary_write(const SweepSummary &s) {
    uint32_t slot = hdr.corners;
    uint32_t max = SWEEP_SUM_BYTES / sizeof(SweepSummary);
    if (slot >= max) return;  // summaries never wrap: stop rather than lose
    fram_write(SWEEP_SUM_BASE + slot * sizeof(SweepSummary), &s, sizeof(s));
    hdr.corners++;
}

static void trace_write(const SweepTrace &t) {
    uint32_t cap = SWEEP_TRACE_END - SWEEP_TRACE_BASE;
    if (hdr.trace_off + sizeof(t) > cap) {
        hdr.trace_off = 0;  // ring: the oldest diagnostic is the cheapest
        hdr.trace_wrapped = 1;
    }
    fram_write(SWEEP_TRACE_BASE + hdr.trace_off, &t, sizeof(t));
    hdr.trace_off += sizeof(t);
}

// -- geometry ------------------------------------------------------------------
// Path s at which the inside wall's END should be SEEN for corner k.
//
// The wall ends at the corner cell's near boundary, half a cell before the
// corner point; the beam leads the mouse by WALL_GATE_BEAM_AHEAD_MM; the
// tangent point is R before the corner point. All three lie on the approach
// STRAIGHT, where path s and spatial distance are the same number -- which
// is the only reason this is a subtraction (see Path.h, the anchor trap).
static float gate_expect_s(int k) {
    // corner_tangent_mm(), NOT cur_R. They are the same number only while
    // the corner is a pure arc; with a clothoid the primitive consumes
    // T = R + m + x0 of corridor, ~10 mm more per 20 mm of transition. Using
    // R here would put every gate expectation T-R too early and turn a
    // working anchor into a systematic bias -- exactly the class of error
    // that cost runs 120-135.
    return path.corner_entry_s_mm(k) + path.corner_tangent_mm(k) -
           0.5f * SWEEP_CELL_MM - WALL_GATE_BEAM_AHEAD_MM;
}

// Room between the gate's firing point and the tangent point -- the distance
// the mouse still has to run after seeing the wall end, and therefore the
// distance the correction is spread over.
//
// It falls out of the geometry with nothing left over:
//
//     room = (half a cell) + (beam lead) - T  =  90 + 45 - T
//
// because the wall ends at the corner cell's near boundary (90 mm before the
// corner point), the beam sees that end WALL_GATE_BEAM_AHEAD_MM early, and
// the arc starts T before the corner point.
//
// THE STALE NUMBERS THAT USED TO BE HERE said "117 mm at R45, 37 mm at R125".
// Both were computed with the old beam lead of 72 mm and with T = R -- i.e.
// before the lead was corrected to 45 (see WALL_GATE_BEAM_AHEAD_MM) and
// before clothoids made T bigger than R. Measured through the real
// tangent_for at v500:
//
//     R125  T 134.7  room   0.3      R100  T 112.2  room  22.8
//     R84   T  98.6  room  36.4      R60   T  80.9  room  54.1
//     R45   T  73.8  room  61.2
//
// AND IT SHRINKS WITH SPEED, because the clothoid is v^2/(R*alpha) and T
// grows with it. At v700: R84 falls to 21.7 mm, R100 to 10.7, and R125 goes
// NEGATIVE (-9.2) -- the gate would fire after the arc had already started,
// with nothing left to correct over. R125 is not in the ladder and v700 is
// not the gathering speed, but this is the wall the combination runs into,
// and it is why a route compiled at one speed is not the path for another.
static float gate_room(int k) {
    float room = path.corner_entry_s_mm(k) - gate_expect_s(k);
    return room > 5.0f ? room : 5.0f;
}

// The lateral offset, with the two beams checked against each other first.
// Returns SWEEP_LAT_* saying where the number came from; *lat is only
// meaningful when that is not SWEEP_LAT_NONE.
// NO LONGER STATIC (2026-08-23). The route runner needs the same rule, and a
// second copy of it would drift: every hard-won line below -- the 40 mm
// floor, the corridor-sum discriminator, the sign resolution -- came out of a
// specific run that cost a session, and none of that would be re-derived in a
// duplicate. Declared in LoopSweep.h as sweep_corridor_lat.
uint8_t sweep_corridor_lat(const WallView &wv, bool in_junction, float *lat) {
    if (!wv.lateral_valid) return SWEEP_LAT_NONE;
    const float HALF = 0.5f * SWEEP_CORRIDOR_MM;
    bool lok = wv.left, rok = wv.right;
    float L = wv.left_mm, R = wv.right_mm;

    // FIRST, THROW OUT WHAT CANNOT BE A DISTANCE. See SWEEP_SIDE_MIN_MM: a
    // corridor reading under 40 mm has the wall within three millimetres of
    // the bodywork, which does not happen without a mark on the mouse. Doing
    // this here rather than downstream matters -- every clever rule below
    // (the corridor-sum check, the sign resolution) exists to cope with a
    // reading that should never have been treated as a length in the first
    // place, and none of them can recover the beam that was thrown away.
    if (!in_junction) {
        if (lok && L < SWEEP_SIDE_MIN_MM) lok = false;
        if (rok && R < SWEEP_SIDE_MIN_MM) rok = false;
    }
    if (!lok && !rok) return SWEEP_LAT_NONE;

    if (lok && rok) {
        float sum = L + R;
        if (fabsf(sum - SWEEP_CORRIDOR_MM) <= SWEEP_CORRIDOR_SUM_TOL_MM) {
            *lat = 0.5f * (R - L);         // a genuine pair: average them
            return SWEEP_LAT_BOTH;
        }
        // NOT A PAIR -- and WHICH ONE IS LYING FOLLOWS FROM THE SIGN.
        //
        // A mouse genuinely off centre does not break the sum: move 45 mm
        // right and the readings go 129 / 39, which still sums to 168. Only
        // a beam reporting something that is not the corridor wall breaks
        // it, so the sum error says which beam to drop:
        //
        //   sum TOO SMALL -> a beam is reading SHORT -> trust the FARTHER
        //   sum TOO LARGE -> a beam is reading LONG  -> trust the NEARER
        //
        // The original rule always took the nearer, on the reasoning that
        // the near wall is the one you hit. Right for clearance, wrong for
        // position, and backwards for the case that actually occurs: run 125
        // died taking a 39 from a post and calling it -45 mm of lateral
        // while the other beam sat steady at 82.
        bool small = sum < SWEEP_CORRIDOR_MM;
        bool take_left = small ? (L >= R) : (L <= R);
        *lat = take_left ? HALF - L : R - HALF;
        return SWEEP_LAT_SUSPECT;
    }
    // One wall. Computed here rather than taken from wv.lateral_mm, which is
    // derived before the floor above and can be built on a beam this has
    // just rejected.
    *lat = lok ? HALF - L : R - HALF;
    return SWEEP_LAT_ONE;
}


// ---- state at the moment a loop stops ---------------------------------------
// Refreshed on every sensor packet while flying, so whichever guard fires --
// or the path timing out -- has a truthful snapshot to write without every
// call site having to gather one. Cheap: six words per packet.
struct StopCtx {
    float s_mm;
    float lat_mm;
    float yaw_err_deg;
    uint8_t left, right, lat_src;
    int corner_to;
    uint32_t loop_ms;
    float lat_prev_mm;   // the packet BEFORE: glitch or genuine drift?
};
static StopCtx stopc;
static void stop_ctx_clear() {
    stopc = StopCtx{};
    stopc.left = 255;
    stopc.right = 255;
}

static void corner_finalise();   // defined with the corner bookkeeping

// A loop ended with no corner record open, so there is no corner to blame and
// nothing would otherwise be written at all. Record the stop itself.
static void loop_event_write(uint8_t fail_code) {
    SweepSummary e;
    memset(&e, 0, sizeof(e));
    e.loop_ix = hdr.loops_done;
    e.corner_ix = SWEEP_CORNER_EVENT;
    e.flags = (g_left ? SWEEP_F_LEFT : 0) | SWEEP_F_ABORTED;
    if (control_motion().slip_events != slip0) e.flags |= SWEEP_F_SLIP;
    e.r_mm = (uint8_t)(cur_R + 0.5f);
    e.slew_ix = (uint8_t)cur_slew_ix;
    e.v_arc = (uint16_t)cur_v;
    e.fail = fail_code;
    e.gate_drop_s = (int16_t)stopc.s_mm;
    e.entry_lat_x10 = (int16_t)(stopc.lat_mm * 10.0f);
    e.entry_yaw_err_cdeg = (int16_t)(stopc.yaw_err_deg * 100.0f);
    e.arc_min_left = stopc.left;
    e.arc_min_right = stopc.right;
    e.lat_source = stopc.lat_src;
    e.exit_lat_x10 = (int16_t)stopc.corner_to;
    e.corner_ms = (uint16_t)(stopc.loop_ms > 65535 ? 65535 : stopc.loop_ms);
    e.wheel_minus_s_x10 =
        (int16_t)(((control_odometry().dist_mm - odo0) - stopc.s_mm) * 10.0f);
    e.arc_turn_cdeg = (int16_t)(stopc.lat_prev_mm * 10.0f);   // lat_prev
    summary_write(e);
}

static void stop_everything() {
    control_move_abort();
    // BRAKE BEFORE CUTTING. control_velocity_enable(false) ends in
    // motors_stop(), and zero duty is COAST -- so every abort up to run 151
    // handed a moving mouse to rolling resistance. At the corridor sweep's
    // speeds that is most of a maze: run 151 tripped the lateral guard at
    // 1143 mm/s and coasted into a wall.
    float v_at_stop = control_brake_to_stop();
    if (fabsf(v_at_stop) > 100.0f)
        printf_line("stop: braked to %.0f mm/s before cutting (timeout)",
                    (double)v_at_stop);
    control_velocity_enable(false);
    sensors_stop();
    hdr_write();
    state = W_IDLE;
}

void sweep_abort(const char *why, uint8_t code) {
    if (rec_open) {
        corner_finalise();
        rec.flags |= SWEEP_F_ABORTED;
        if (rec.fail == SWEEP_FAIL_NONE) rec.fail = code;
        summary_write(rec);
        rec_open = false;
    }
    if (state == W_FLY) loop_event_write(code);
    stop_everything();
    printf_line("sweep ABORT at combo %u loop %u corner %d: %s",
                (unsigned)hdr.combo_ix, (unsigned)hdr.loops_done, corner_ix,
                why);
    printf_line("-- %lu corners logged. 'Wd' to dump, wave to resume --",
                (unsigned long)hdr.corners);
}

// A guard tripped, or the loop could not finish. THIS IS A RESULT, NOT AN
// ERROR: "R45 at 200 mm/s with a lazy slew rail cannot hold the corridor" is
// precisely what the sweep is for. So the combo is marked, the grid moves on,
// and the mouse re-arms and waits for a wave.
//
// The first build ended the whole SESSION on any guard trip -- and the grid
// happens to open on its most marginal cell (R45/v200/slew 2500 spends 29% of
// the arc just ramping the rate), so one trip on combo 0 cost the other 49.
//
// It does not carry on by itself: after a trip the mouse is somewhere in a
// junction, off its path, and nothing here can put it back. A human picks it
// up, sets it on a straight, and waves.
static void loop_failed(const char *why, uint8_t fail_code) {
    if (rec_open) {
        corner_finalise();
        rec.flags |= SWEEP_F_ABORTED;
        rec.fail = fail_code;
        summary_write(rec);
        rec_open = false;
    }
    // ALWAYS, whether or not a corner was open. Run 123 tripped a guard while
    // corner 3's record WAS open, so the failure went into that corner row --
    // which, for a corner that never reached its tangent point, is almost
    // entirely zeros. "fail=lateral" with no s, no lateral and no beams is
    // not a stop reason. The event row carries the state; the corner row goes
    // on meaning what it always meant.
    loop_event_write(fail_code);
    control_move_abort();
    control_velocity_enable(false);
    control_set_lateral_bias_deg(0.0f);

    // LEAVE W_FLY HERE, not in sweep_arm(). sweep_arm() opens with
    //     if (state != W_IDLE && state != W_DONE) return;
    // so the first build's call at the bottom of this function did nothing at
    // all: the mouse stayed in W_FLY holding a dead path. Every subsequent
    // poll then re-entered the flying body, saw !rec_open with corner_ix
    // still on the failed corner and s_mm frozen past its window, opened a
    // fresh record, latched entry and exit in the same tick (corner_ms 0),
    // fell through "if (ps.active) return" and wrote the empty record -- then
    // hit !ps.done and failed the loop AGAIN. One guard trip therefore burned
    // a combo per tick until the fail streak stopped the session.
    // Run 117: one lateral trip on combo 1 ate combos 2 and 3 and logged
    // S,2,1 and S,3,1 with no data in them.
    state = W_IDLE;
    corner_ix = 0;
    rec_open = false;
    gate.disarm();   // a loop that ends mid-approach must not carry an
                     // arming, or a half-observed corner, into the next

    printf_line("loop %u FAILED (combo %u: R%.0f v%.0f slew %.0f): %s",
                (unsigned)hdr.loops_done, (unsigned)hdr.combo_ix,
                (double)cur_R, (double)cur_v, (double)cur_slew, why);
    hdr.loops_done++;
    hdr.combo_ix++;          // the combo is answered: it does not fly
    hdr_write();
    fail_streak++;

    if (fail_streak >= SWEEP_FAIL_STREAK_ABORT) {
        sensors_stop();
        state = W_IDLE;
        printf_line("== %d combos failed in a row -- stopping. Something is "
                    "wrong with the rig or the tuning, not with the grid. ==",
                    fail_streak);
        printf_line("-- %lu corners logged. 'sweep.dump' to read them --",
                    (unsigned long)hdr.corners);
        return;
    }
    if (hdr.combo_ix >= g_grid.combos()) {
        sensors_stop();
        state = W_DONE;
        printf_line("== sweep COMPLETE: %lu corners ==",
                    (unsigned long)hdr.corners);
        return;
    }
    printf_line("-- put the mouse back on a straight and wave: next is "
                "combo %u of %d --",
                (unsigned)hdr.combo_ix, g_grid.combos());
    sweep_arm();
}

// -- launching a loop ----------------------------------------------------------
// Defined next to square_frame(), which owns the values it writes.
static void launch_row_write(float body_off_deg, float lat0_mm,
                             uint8_t lat0_src, const WallRanges &w0);

static bool loop_launch() {
    // Walk forward past any combo the slew rail cannot deliver. Bounded by
    // the grid: every pass either flies or advances combo_ix, and combo_ix
    // running off the end is the COMPLETE exit above.
    for (;;) {
        if (!g_grid.at(hdr.combo_ix, &cur_R, &cur_v, &cur_slew, &cur_a_lon)) {
            stop_everything();
            state = W_DONE;
            printf_line("== sweep COMPLETE: %d combos, %lu corners ==",
                        g_grid.combos(), (unsigned long)hdr.corners);
            printf_line("-- 'sweep.dump' to read the log over USB --");
            return false;
        }
        cur_slew_ix = hdr.combo_ix / (g_grid.n_v * g_grid.n_r);

        // DECLINE a corner the rail cannot deliver. The ramp has to fit
        // inside the arc or the commanded rate is never reached, and run 124
        // showed what that costs: R45/v400 at 1.15 tripped the yaw guard a
        // third of the way round and hit the outer wall. Recorded as a
        // result, not skipped silently.
        float rf = ramp_frac(cur_R, cur_v, cur_slew);
        if (rf <= SWEEP_MAX_RAMP_FRAC) break;

        printf_line("combo %u DECLINED (R%.0f v%.0f slew %.0f): the rate ramp "
                    "is %.0f%% of the arc -- the commanded turn rate is never "
                    "reached, so this is a crash, not a corner",
                    (unsigned)hdr.combo_ix, (double)cur_R, (double)cur_v,
                    (double)cur_slew, (double)(100.0f * rf));
        // Nothing moved, so nothing movement-derived should be reported.
        stop_ctx_clear();
        odo0 = control_odometry().dist_mm;
        loop_event_write(SWEEP_FAIL_RAMP);
        hdr.loops_done++;
        hdr.combo_ix++;
        hdr_write();
    }

    // CLOTHOID SIZING, per combo. Two limits, and the smaller wins.
    //
    //   rail  L = v^2/(R*alpha) -- the shortest transition the slew rail can
    //         already follow. Shorter and the ramp stays rate-limited, so
    //         the geometry gains nothing; longer and the corner eats
    //         corridor for no benefit.
    //   room  the corner consumes T either side of the corner point, and the
    //         gate has to fire before the tangent. The inside wall ends
    //         90 mm before the corner point and the beam leads by
    //         WALL_GATE_BEAM_AHEAD_MM, so T must leave SWEEP_GATE_ROOM_MIN_MM
    //         for the correction to be spread over.
    //
    // The room limit bites hardest at LARGE radius, which is where the
    // overshoot is smallest -- a pure R125 arc already eats 125 of the 135
    // available. So R125 gets little or no transition and does not need one
    // (0.84 deg of overshoot at slew 6000), while R60 gets the full
    // rail-sized ramp with 54 mm to spare. The budget happens to grow
    // exactly where the problem is.
    float sweep_deg = g_left ? 90.0f : -90.0f;
    float t_max = 0.5f * SWEEP_CELL_MM + WALL_GATE_BEAM_AHEAD_MM -
                  SWEEP_GATE_ROOM_MIN_MM;
    float l_rail = SpeedPath::clothoid_for_rail(cur_R, cur_v, cur_slew);
    float l_room = SpeedPath::clothoid_for_room(cur_R, t_max, sweep_deg);
    float l_cl = (l_rail < l_room) ? l_rail : l_room;
    if (!SWEEP_CLOTHOID) l_cl = 0.0f;
    path.set_clothoid_len(l_cl);

    float first_leg = SWEEP_FIRST_LEG_CELLS * SWEEP_CELL_MM;
    if (!path.begin_ring(first_leg, SWEEP_SIDE_MM, cur_R, g_left)) {
        sweep_abort("ring path build failed");
        return false;
    }
    // a_lat is DERIVED from the (R, v) pair being tested: the point of the
    // grid is to command a specific arc speed at a specific radius, not to
    // command a lateral budget and see what speed falls out.
    float a_lat = cur_v * cur_v / cur_R;
    // v_max is NOT the arc speed any more. The profile takes
    // min(sqrt(a_lat/kappa), v_max), so the arc still flies at sqrt(a_lat*R)
    // = cur_v while the straight is free to run up to SWEEP_STRAIGHT_V_MAX at
    // cur_a_lon. That separation is the whole of this experiment.
    path.set_limits(SWEEP_STRAIGHT_V_MAX, cur_a_lon, a_lat);
    control_set_path_slew(cur_slew);
    printf_line("loop %u: R%.0f v%.0f slew %.0f  clothoid %.1f mm "
                "(rail %.1f, room %.1f)  T %.1f  gate room %.1f",
                (unsigned)hdr.loops_done, (double)cur_R, (double)cur_v,
                (double)cur_slew, (double)path.clothoid_len(),
                (double)l_rail, (double)l_room,
                (double)path.corner_tangent_mm(0), (double)gate_room(0));

    corner_ix = 0;
    rec_open = false;
    gate.disarm();   // a loop that ends mid-approach must not carry an
                     // arming, or a half-observed corner, into the next
    next_trace_s = -1.0f;
    next_crumb_s = -1.0f;
    lat_over_run = 0;
    stop_ctx_clear();
    odo0 = control_odometry().dist_mm;
    // ZERO FIRST, THEN BASELINE. The first build read slip_events before
    // control_motion_zero() cleared it, so slip0 held the previous loop's
    // count and the counter it was compared against started from 0 -- they
    // could never agree, and SWEEP_F_SLIP came out set on every corner of
    // every loop in runs 116 and 117. The flag carried no information at all.
    control_motion_zero();
    slip0 = control_motion().slip_events;
    loop_yaw0 = control_heading().yaw_deg;
    last_seq = sensors_latest().seq;
    lane_open = false;   // the fit spans one leg; the launch starts a new one

    // LAUNCH ON THE CORRIDOR, NOT ON THE BODY. square_frame() put the
    // corridor on sq_cardinal, so whatever the mouse has drifted to while
    // stopped reads directly as (yaw - sq_cardinal). Handing the negative of
    // that to control_path_start builds the whole trajectory on the corridor
    // and leaves the misalignment as a heading transient.
    // Where the mouse is STANDING, which the fit cannot see (it ends while
    // the mouse is still moving, and the brake translates it). Read whether
    // or not the squaring worked -- a launch row with no fit is still worth
    // having, because it says what the pose was when the fit was missing.
    WallRanges w0 = sensors_latest();
    WallView v0 = walls_evaluate(w0, sensors_ms_since_packet());
    float lat0 = 0.0f;
    uint8_t src0 = sweep_corridor_lat(v0, false, &lat0);

    float yaw_off = 0.0f;
    float delta = 0.0f;
    if (sq_valid) {
        delta = wrap180(control_heading().yaw_deg - sq_cardinal);
        if (fabsf(delta) <= SWEEP_SQ_MAX_DEG) {
            yaw_off = -delta;
        } else {
            sq_status = SWEEP_SQ_BODY;
            printf_line("launch: body %+.2f deg off the corridor -- past the "
                        "%.0f deg limit, launching on the body instead",
                        (double)delta, (double)SWEEP_SQ_MAX_DEG);
        }
        // Logged, not acted on: with an honest frame the lateral servo
        // drains the standing offset over the first leg.
        printf_line("launch: body %+.2f deg off corridor (frame offset "
                    "%+.2f), standing lat %s%.1f mm (L%u R%u)",
                    (double)delta, (double)yaw_off,
                    src0 == SWEEP_LAT_NONE ? "n/a " : "", (double)lat0,
                    (unsigned)w0.left_mm, (unsigned)w0.right_mm);
    }
    // BEFORE the path starts, so a mouse that dies on its first metre still
    // leaves the pose it launched from.
    launch_row_write(delta, lat0, src0, w0);

    if (!control_path_start(&path, yaw_off)) {
        sweep_abort("path start refused");
        return false;
    }
    // Peak reachable on the straight: 900 between corner points less two
    // tangents, half of it spent accelerating from the arc speed.
    float straight = SWEEP_SIDE_MM - 2.0f * path.corner_tangent_mm(0);
    float peak = sqrtf(cur_v * cur_v + cur_a_lon * straight);
    if (peak > SWEEP_STRAIGHT_V_MAX) peak = SWEEP_STRAIGHT_V_MAX;
    printf_line("loop %u: combo %u/%d  R%.0f v_arc %.0f a_lat %.0f slew %.0f"
                "  |  a_lon %.0f (%.2f g), straight %.0f mm, peak ~%.0f mm/s",
                (unsigned)hdr.loops_done, (unsigned)hdr.combo_ix,
                g_grid.combos(), (double)cur_R, (double)cur_v, (double)a_lat,
                (double)cur_slew, (double)cur_a_lon,
                (double)(cur_a_lon / 9810.0f), (double)straight, (double)peak);
    state = W_FLY;
    state_t0 = millis();
    return true;
}

// Square the FRAME onto the corridor between loops. Nothing moves: the fit
// from the last clean leg says where the corridor lies in the fused frame,
// and the fused frame is shifted so it lies on an exact cardinal. See the
// note on `lane` for what this replaces and why an in-place turn was the
// wrong instrument.
//
// After this, sq_cardinal IS the corridor, by construction, and stays the
// corridor through the stop -- any further twist while the tyres relax shows
// up as (yaw - sq_cardinal) and is picked up at launch rather than baked in.
static void square_frame() {
    // Snapshot what the fit was worth AT THE MOMENT IT WAS USED. `lane` keeps
    // accumulating into the next leg, so reading its size later would report
    // a different fit from the one that produced sq_corridor_yaw.
    sq_fit_n = lane.samples();
    sq_fit_span = lane.span_mm();
    sq_fit_rej = lane.rejected();
    if (!sq_valid) {
        sq_status = SWEEP_SQ_NONE;
        sq_corr_deg = 0.0f;
        printf_line("square: no corridor fit this loop (n=%d span=%.0f mm) "
                    "-- frame left as it stands, launching on the body",
                    sq_fit_n, (double)sq_fit_span);
        return;
    }
    float yaw = control_heading().yaw_deg;
    float cardinal = roundf(sq_corridor_yaw / 90.0f) * 90.0f;
    float corr = wrap180(cardinal - sq_corridor_yaw);
    if (fabsf(corr) > WALL_ALIGN_MAX_CORR_DEG) {
        printf_line("square: corridor fit %+.2f deg off cardinal -- refused "
                    "(eps %+.2f, n=%d)", (double)corr, (double)sq_eps_deg,
                    sq_fit_n);
        sq_valid = false;
        sq_status = SWEEP_SQ_REFUSED;
        sq_corr_deg = corr;
        return;
    }
    // Move the datum, not the mouse: everything the fused frame reports
    // shifts by the same corr, so the corridor lands on the cardinal and the
    // body keeps whatever angle to it that it actually has.
    control_set_heading_deg(yaw + corr);
    sq_cardinal = cardinal;
    sq_corr_deg = corr;
    sq_status = SWEEP_SQ_SQUARED;
    printf_line("square: corridor eps %+.2f deg, frame %+.2f deg -> corridor "
                "= %.0f  (fit n=%d span=%.0f mm, %d rejected)",
                (double)sq_eps_deg, (double)corr, (double)cardinal,
                sq_fit_n, (double)sq_fit_span, sq_fit_rej);
}

// One record per loop, written BEFORE it moves. See SWEEP_CORNER_LAUNCH for
// what each re-purposed field carries and why this row exists at all.
static void launch_row_write(float body_off_deg, float lat0_mm,
                             uint8_t lat0_src, const WallRanges &w0) {
    SweepSummary e;
    memset(&e, 0, sizeof(e));
    e.loop_ix = hdr.loops_done;
    e.corner_ix = SWEEP_CORNER_LAUNCH;
    e.flags = g_left ? SWEEP_F_LEFT : 0;
    e.r_mm = (uint8_t)(cur_R + 0.5f);
    e.slew_ix = (uint8_t)cur_slew_ix;
    e.v_arc = (uint16_t)cur_v;
    e.fail = sq_status;
    e.entry_yaw_err_cdeg = (int16_t)(sq_eps_deg * 100.0f);
    e.exit_yaw_err_cdeg = (int16_t)(sq_corr_deg * 100.0f);
    e.arc_turn_cdeg = (int16_t)(body_off_deg * 100.0f);
    e.exit_lat_x10 = (int16_t)(sq_cardinal * 10.0f);
    e.entry_lat_x10 = (int16_t)(lat0_mm * 10.0f);
    e.lat_source = lat0_src;
    e.arc_min_left = w0.left_mm;
    e.arc_min_right = w0.right_mm;
    e.corner_ms = (uint16_t)(sq_fit_n > 65535 ? 65535 : sq_fit_n);
    e.gate_drop_s = (int16_t)sq_fit_span;
    e.gate_corr_x10 = (int16_t)sq_fit_rej;
    // The corridor sweep's axis. arc_ay_mm_s2 carries no meaning on a launch
    // row, and a_lon is per-LOOP, which is exactly what this row is.
    e.arc_ay_mm_s2 = (int16_t)cur_a_lon;
    summary_write(e);
}

static void loop_begin() {
    if (millis() - session_t0 > SWEEP_TIME_LIMIT_MS) {
        sweep_abort("session time limit", SWEEP_FAIL_TIMEOUT);
        return;
    }
    square_frame();   // costs no motion, so there is nothing to wait for
    if (!loop_launch()) return;
}

// -- per-corner measurement ----------------------------------------------------
// ARM THE GATE, EARLY AND INDEPENDENTLY OF THE CORNER RECORD.
//
// Until run 148 the gate was armed inside corner_open() and fed only while a
// record was open, so its whole world was the capture window -- which starts
// (SWEEP_TRACE_HALF_MM - R) before the entry tangent, 145 mm at R125. That is
// fine for finding ONE wall end. It is useless for what bert is building
// next: a corridor with several walls removed, where the mouse counts the
// posts it passes and gates from the last one. Over a 900 mm leg the
// instrument could see the last 16 % of it.
//
// So arming moves here, keyed on distance to the expected drop rather than on
// the record. The observer then sees SWEEP_GATE_OBSERVE_MM of approach.
//
// WHAT DOES NOT CHANGE: where the gate is allowed to DECIDE. live_from_s_mm
// is held at whichever is LATER -- the decide distance, or the moment the
// record opens -- so no decision can land before there is a record to write
// it into, and every decision happens exactly where it happened in runs 141
// to 148. Widening observation must not quietly widen the thing that has
// taken thirty sessions to make trustworthy.
static void gate_arm_for(int k) {
    WallGateConfig gc;
    gc.expect_s_mm = gate_expect_s(k);
    float decide = gc.expect_s_mm - SWEEP_GATE_DECIDE_MM;
    float rec_opens = path.corner_entry_s_mm(k) - (SWEEP_TRACE_HALF_MM - cur_R);
    gc.live_from_s_mm = decide > rec_opens ? decide : rec_opens;
    gate.arm(gc);
}

static void corner_open(int k) {
    memset(&rec, 0, sizeof(rec));
    rec.loop_ix = hdr.loops_done;
    rec.corner_ix = (uint8_t)k;
    rec.flags = g_left ? SWEEP_F_LEFT : 0;
    if (k == 0) rec.flags |= SWEEP_F_SHORT_APPROACH;
    rec.r_mm = (uint8_t)(cur_R + 0.5f);
    rec.slew_ix = (uint8_t)cur_slew_ix;
    rec.v_arc = (uint16_t)cur_v;
    rec.arc_min_left = 255;
    rec.arc_min_right = 255;
    rec.gate_drop_s = -1;
    arc_t0 = 0;
    arc_yaw0 = 0.0f;
    arc_ay_sum = 0.0f;
    arc_ay_n = 0;
    entry_latched = false;
    exit_latched = false;
    beam_close_run = 0;
    rec.fail = SWEEP_FAIL_NONE;
    entry_n = exit_n = 0;
    entry_sum = exit_sum = 0.0f;
    rec_tracing = false;
    rec_trace_off = 0;
    // Per-CORNER slip baseline: a slip in corner 0 is not evidence about
    // corner 3, and the loop-level baseline made every later corner inherit
    // the earlier ones' flag.
    slip0 = control_motion().slip_events;
    rec_open = true;

    // The gate is NOT armed here any more -- gate_arm_for() does it, earlier
    // and independently of this record. See the note there.
}

// Fill the derived fields. MUST run before any path that writes the record
// -- the first build only did it in corner_close(), so an aborted corner
// stored entry_lat and exit_lat as 0.0. Those are the two numbers the whole
// experiment is about, and they went missing in exactly the case worth
// looking at (run 116).
static void corner_finalise() {
    rec.arc_ay_mm_s2 = arc_ay_n ? (int16_t)(arc_ay_sum / arc_ay_n) : 0;
    rec.entry_lat_x10 = entry_n ? (int16_t)(entry_sum / entry_n * 10.0f) : 0;
    rec.exit_lat_x10 = exit_n ? (int16_t)(exit_sum / exit_n * 10.0f) : 0;
    if (control_motion().slip_events != slip0) rec.flags |= SWEEP_F_SLIP;
    if (rec_tracing) {
        rec.flags |= SWEEP_F_TRACED;
        rec.trace_off = rec_trace_off;
    }
}

// What the inside beam did on the approach we have just finished. Written
// for EVERY corner, not only stripped ones: on an intact corner the wall is
// a run of known length, which is what calibrates packets-per-millimetre and
// makes the short runs interpretable.
static void feature_row_write() {
    SweepFeatureRow f;
    memset(&f, 0, sizeof(f));
    f.loop_ix = hdr.loops_done;
    f.kind = SWEEP_CORNER_FEATURE;
    f.flags = g_left ? SWEEP_F_LEFT : 0;
    f.corner_ix = rec.corner_ix;
    int seen = gate.run_count();
    f.runs_seen = (uint8_t)(seen > 255 ? 255 : seen);
    int n = gate.runs_stored();
    if (n > 5) n = 5;
    for (int i = 0; i < n; i++) {
        const WallGate::Run &r = gate.run(i);
        f.run[i].s0_mm = (int16_t)r.s0_mm;
        f.run[i].s1_mm = (int16_t)r.s1_mm;
        f.run[i].min_mm = r.min_mm;
        f.run[i].n = r.n;
    }
    SweepSummary s;
    memcpy(&s, &f, sizeof(s));   // same 36 bytes; kind aliases corner_ix
    summary_write(s);
}

static void corner_close() {
    if (!rec_open) return;
    corner_finalise();
    // Gate health. A corner that goes ungated is flown on odometry, which is
    // survivable once and a reason to stop if it keeps happening -- the
    // mouse is then navigating a 20-cell ring on dead reckoning alone.
    if (rec.flags & SWEEP_F_GATE_LOCK) gate_miss_streak = 0;
    else gate_miss_streak++;
    summary_write(rec);
    feature_row_write();          // reads the observer, so before disarm
    gate.disarm();                // lets the NEXT corner arm; see gate_arm_for
    rec_open = false;
}

// One trace record from the current state. Shared by corner tracing and the
// corridor breadcrumbs so the two can never drift apart in what they mean.
static SweepTrace trace_fill(const ControlPathStatus &ps, const WallRanges &w,
                             const WallView &wv, int corner) {
    SweepTrace t;
    t.s_mm = (int16_t)ps.s_mm;
    t.v_mm_s = (int16_t)ps.v_mm_s;
    t.yaw_cdeg = (int16_t)((control_heading().yaw_deg - loop_yaw0) * 100.0f);
    t.yaw_err_cdeg = (int16_t)(ps.yaw_err_deg * 100.0f);
    t.omega_dps = (int16_t)ps.omega_dps;
    t.ay_mm_s2 =
        (int16_t)(control_motion().a_lat_mm_s2 - (float)hdr.ay_bias_mm_s2);
    t.left_mm = w.left_mm;
    t.right_mm = w.right_mm;
    t.flags = (ps.in_arc ? 1 : 0) | (wv.lateral_valid ? 2 : 0);
    t.corner_ix = (uint8_t)corner;
    return t;
}

// ---------------------------------------------------------------- poll
void sweep_poll() {
    if (state == W_IDLE || state == W_DONE) return;
    uint32_t now = millis();
    sensors_poll();

    switch (state) {
        case W_ARMED: {
            // THE LEDS ARE THE ONLY UI AT THE MAZE. No terminal is required
            // to run this thing, so the armed pattern has to say what a wave
            // will actually do:
            //   slow alternate  -- fresh session or resumable: wave to go
            //   fast together   -- the log holds data this firmware did not
            //                      produce, or a finished set. A wave does
            //                      NOTHING; hold 5 s to discard and start.
            if (!hand_active) {
                SweepReadiness rd = sweep_readiness();
                uint32_t period = (rd == SWEEP_READY_NEEDS_WIPE) ? 120 : 500;
                if (now - led_last >= period) {
                    led_last = now;
                    led_phase = !led_phase;
                    if (rd == SWEEP_READY_NEEDS_WIPE) {
                        hal_set_led1(led_phase);
                        hal_set_led2(led_phase);
                    } else {
                        hal_set_led1(led_phase);
                        hal_set_led2(!led_phase);
                    }
                }
            }

            WallRanges w = sensors_latest();
            if (w.seq != last_seq) {
                last_seq = w.seq;
                WallView wv = walls_evaluate(w, sensors_ms_since_packet());
                // Schmitt trigger: START_MM to begin a gesture, the wider
                // CLEAR_MM to keep one alive.
                float lim = hand_active ? W_HAND_CLEAR_MM : W_HAND_START_MM;
                if (wv.valid && wv.front && wv.front_mm < lim) {
                    if (hand_run < W_HAND_TRIGGER_N) hand_run++;
                    if (hand_run >= W_HAND_TRIGGER_N && !hand_active) {
                        hand_active = true;
                        hand_t0 = now;
                        hand_wiped = false;
                        hand_next_tick = 1;
                        printf_line("sweep: hand seen -- remove to run, or "
                                    "hold %lu s to DISCARD the stored session",
                                    (unsigned long)(SWEEP_WIPE_HOLD_MS / 1000));
                    }
                    if (hand_active) hand_seen_ms = now;
                } else if (!hand_active) {
                    hand_run = 0;
                }
            }
            if (!hand_active) return;

            // Everything below runs on the millis clock, not the packet
            // clock, so the answer does not depend on the stream rate.
            uint32_t held = now - hand_t0;
            // THE HOLD ONLY DISCARDS WHAT A WAVE CANNOT START.
            //
            // It used to fire on any held hand, whatever was in FRAM. That
            // is a destroyer, and it is almost certainly what ate the good
            // high-accel lap (bert, 2026-08-22, no log under 'y' or
            // 'sweep.dump'). The sweep RE-ARMS after every combo, so between
            // loops the mouse sits in W_ARMED with the front beam live and a
            // full session in FRAM. Reach in to pick it up or straighten it,
            // linger 5 s, and the session is gone -- announced only by two
            // LEDs and a console line nobody is reading at the maze.
            //
            // A resumable session needs no discard: a wave already continues
            // it. The gesture exists solely for the case where a wave does
            // NOTHING -- a finished set, or a log this firmware cannot append
            // to. So it is gated on exactly that state now, and a hold over a
            // resumable session says so instead of wiping it.
            bool wipeable = sweep_readiness() == SWEEP_READY_NEEDS_WIPE;
            if (!hand_wiped && hand_next_tick * 1000 <= held &&
                hand_next_tick * 1000 < SWEEP_WIPE_HOLD_MS) {
                if (wipeable)
                    printf_line("   holding %lu s -- %lu more to DISCARD",
                                (unsigned long)hand_next_tick,
                                (unsigned long)(SWEEP_WIPE_HOLD_MS / 1000 -
                                                hand_next_tick));
                else
                    printf_line("   holding %lu s -- nothing to discard, the "
                                "session resumes on a wave",
                                (unsigned long)hand_next_tick);
                hand_next_tick++;
            }
            // Fire the wipe AT the mark with the hand still there, so the
            // mouse setting off IS the confirmation that it took.
            if (!hand_wiped && wipeable && held >= SWEEP_WIPE_HOLD_MS) {
                hand_wiped = true;
                hal_set_led1(true);
                hal_set_led2(true);
                sweep_restart();
            }
            if (now - hand_seen_ms >= 400) {   // a real absence, not a flicker
                if (!hand_wiped && sweep_readiness() == SWEEP_READY_NEEDS_WIPE) {
                    // A wave cannot discard data. Say so and stay armed.
                    printf_line("sweep: nothing to do -- the stored session "
                                "must be discarded first (hold %lu s)",
                                (unsigned long)(SWEEP_WIPE_HOLD_MS / 1000));
                    hand_active = false;
                    hand_run = 0;
                    return;
                }
                printf_line("sweep: %s -- remove hand",
                            hand_wiped ? "LOG DISCARDED, starting fresh"
                                       : "triggered");
                hand_active = false;
                hand_run = 0;
                state = W_HAND_CLEAR;
            }
            return;
        }

        case W_HAND_CLEAR: {
            WallRanges w = sensors_latest();
            if (w.seq == last_seq) return;
            last_seq = w.seq;
            WallView wv = walls_evaluate(w, sensors_ms_since_packet());
            bool clear = !wv.front || wv.front_mm > W_HAND_CLEAR_MM;
            if (wv.valid && clear) {
                if (++hand_run >= W_HAND_CLEAR_N) {
                    printf_line("sweep: hand clear -- keep the mouse still");
                    hand_run = 0;
                    hand_cal_started = false;
                    ay_cal_n = 0;
                    state = W_CAL;
                    state_t0 = now;
                }
            } else {
                hand_run = 0;
            }
            return;
        }

        case W_CAL:
            if (!hand_cal_started) {
                if (now - state_t0 < W_HAND_SETTLE_MS) return;
                control_recalibrate();
                hand_cal_started = true;
                printf_line("sweep: IMU calibrating (~3 s, stationary)...");
                return;
            }
            if (!control_heading().ready) return;
            // LATERAL ACCELEROMETER ZERO. The mouse is provably stationary
            // here and provably not stationary anywhere else in a session,
            // so this is the only place the Y bias can be measured. Without
            // it the trace's ay is an unknown constant plus the signal, and
            // the constant is the same order as the signal.
            if (ay_cal_n == 0) {
                ay_cal_t0 = now;
                ay_cal_sum = 0.0f;
            }
            if (now - ay_cal_t0 < SWEEP_AY_CAL_MS) {
                ay_cal_sum += control_motion().a_lat_mm_s2;
                ay_cal_n++;
                return;
            }
            hdr.ay_bias_mm_s2 = (int16_t)(ay_cal_sum / ay_cal_n);
            hdr_write();
            printf_line("sweep: lateral accel zero %+d mm/s2 (%d samples)",
                        (int)hdr.ay_bias_mm_s2, ay_cal_n);
            session_t0 = now;
            control_velocity_enable(true);
            gate_miss_streak = 0;
            sensors_free_run_interlaced();
            sweep_report();
            loop_begin();
            return;

        case W_SQUARE:
            // Unreachable: squaring no longer moves the mouse, so nothing
            // enters this state. Fall through to W_SETTLE rather than
            // return, so a mouse that somehow lands here still launches
            // instead of sitting in a state with no exit.
        case W_SETTLE:
            if (now - state_t0 < 250) return;
            loop_launch();
            return;

        case W_FLY:
            break;  // the body of the function

        default:
            return;
    }

    // ---- flying a loop -------------------------------------------------------
    ControlPathStatus ps = control_path_status();
    int nc = path.corner_count();

    // Open the record for a corner once the mouse is within the capture
    // window of it; close it once past the window.
    if (rec_open) {
        float win_end = path.corner_exit_s_mm(corner_ix) +
                        (SWEEP_TRACE_HALF_MM - cur_R);
        if (ps.s_mm > win_end) {
            corner_close();
            corner_ix++;
        }
    }
    // ps.active in the condition: never open a record against a path that is
    // no longer running. Belt and braces behind the state fix in
    // loop_failed() -- a record opened on a frozen s is not a corner.
    if (!rec_open && ps.active && corner_ix < nc) {
        float win_start = path.corner_entry_s_mm(corner_ix) -
                          (SWEEP_TRACE_HALF_MM - cur_R);
        if (ps.s_mm >= win_start) {
            corner_open(corner_ix);
            next_trace_s = ps.s_mm;
        }
    }

    WallRanges w = sensors_latest();
    if (w.seq != last_seq) {
        last_seq = w.seq;
        WallView wv = walls_evaluate(w, sensors_ms_since_packet());

        float lat = 0.0f;

        // Are we in the junction -- inside the corner cell, where there is
        // no corridor for the wall model to describe? From the entry tangent
        // to SWEEP_JUNCTION_EXIT_MM past the exit tangent.
        bool in_junction = false;
        if (corner_ix < nc) {
            in_junction = ps.s_mm >= path.corner_entry_s_mm(corner_ix) &&
                          ps.s_mm < path.corner_exit_s_mm(corner_ix) +
                                        SWEEP_JUNCTION_EXIT_MM;
        }

        // AFTER in_junction, because the validity floor on a side beam is a
        // corridor rule -- in a junction the beam is not looking at a wall
        // beside the mouse and the body geometry does not bound it.
        uint8_t lat_src = sweep_corridor_lat(wv, in_junction, &lat);

        // Keep the stop snapshot current. Whatever ends this loop -- a guard,
        // the path timing out, a human -- gets a truthful picture of the last
        // packet without every call site having to assemble one.
        stopc.lat_prev_mm = stopc.lat_mm;
        stopc.s_mm = ps.s_mm;
        stopc.lat_mm = lat;
        stopc.yaw_err_deg = ps.yaw_err_deg;
        stopc.left = w.left_mm;
        stopc.right = w.right_mm;
        stopc.lat_src = lat_src;
        // BEAM HEALTH. A pair whose sum is not the corridor width means one
        // beam is not looking at the wall beside the mouse. A few per lap is
        // the posts at wall ends; run 126 hit 7.7 % because the right channel
        // was intermittently stuck at 38, and that took a trace-by-trace read
        // to find. Counted here so the next one announces itself.
        if (hdr.pair_samples < 0xFFFFFFFFUL) {
            if (wv.left && wv.right) {
                hdr.pair_samples++;
                if (lat_src == SWEEP_LAT_SUSPECT) hdr.suspect_pairs++;
            }
        }
        stopc.corner_to = corner_ix;
        stopc.loop_ms = now - state_t0;


        // Lateral wall servo on the corridor only, and only on a lateral we
        // trust enough to steer on.
        if (!in_junction && lat_src == SWEEP_LAT_BOTH) {
            control_set_lateral_bias_deg(-LAT_SERVO_DEG_PER_MM * lat);
        } else {
            control_set_lateral_bias_deg(0.0f);
        }

        // CORRIDOR SQUARING FIT. Same gate as the servo -- a lateral good
        // enough to steer on is good enough to fit a wall direction to --
        // and reset at every junction so the fit only ever spans one
        // straight leg.
        //
        // Both terms of the average are taken over the SAME samples on
        // purpose. The lateral servo is steering during the fit, so the
        // mouse's heading and its direction of travel both move; eps is the
        // angle between travel and the wall, and the yaw mean is the heading
        // over the same stretch, so (yaw_bar - eps) is the wall's direction
        // in the fused frame whether the mouse was converging or already
        // straight. Taking yaw at one instant instead would measure the
        // servo's transient and call it a misalignment.
        if (in_junction) {
            lane_open = false;
            // INVALIDATE THE LATCH, not just the accumulator. sq_corridor_yaw
            // is an ABSOLUTE bearing, and the ring's four legs are 90 apart --
            // so a value latched on an earlier leg and left standing when a
            // later leg fails to fit is not stale by a little, it is stale by
            // a right angle. Run 151 did exactly that: the last leg managed
            // n=21 over 20 mm with 30 samples rejected (walls removed for the
            // post work), the fit never became valid, and the previous leg's
            // bearing squared the frame onto a cardinal 90 deg out. The launch
            // clamp caught it -- body_off 89.7 against SWEEP_SQ_MAX_DEG 6, so
            // it fell back to the body and flew -- but a safety net firing is
            // not a design.
            sq_valid = false;
        } else if (lat_src == SWEEP_LAT_BOTH) {
            float yaw = control_heading().yaw_deg;
            if (!lane_open) {
                lane.reset();
                lane_yaw_ref = yaw;
                lane_yaw_sum = 0.0f;
                lane_yaw_n = 0;
                lane_open = true;
            }
            if (lane.add(control_odometry().dist_mm, lat)) {
                lane_yaw_sum += wrap180(yaw - lane_yaw_ref);
                lane_yaw_n++;
            }
            if (lane.valid() && lane_yaw_n > 0) {
                float eps = lane.yaw_err_deg();
                if (fabsf(eps) <= WALL_ALIGN_MAX_CORR_DEG) {
                    float yaw_bar = lane_yaw_ref + lane_yaw_sum / lane_yaw_n;
                    sq_eps_deg = eps;
                    sq_corridor_yaw = yaw_bar - eps;
                    sq_valid = true;
                }
            }
        }

        // GUARD 1, in the corridor: the wall model applies, so use it.
        // It must persist for SWEEP_LAT_ABORT_N packets -- see the note
        // there. Run 123 ended on a single packet at a wall-end transition
        // four millimetres after reading dead centre.
        if (!in_junction && lat_src != SWEEP_LAT_NONE &&
            fabsf(lat) > SWEEP_LAT_ABORT_MM) {
            if (++lat_over_run >= SWEEP_LAT_ABORT_N) {
                loop_failed("lateral limit -- about to touch a wall",
                            SWEEP_FAIL_LATERAL);
                return;
            }
        } else {
            lat_over_run = 0;
        }
        // GUARD 2, in the junction: model-free. A RAW beam this close is
        // something against the body whatever angle it is seen at, and it is
        // set below anything the corner geometry legitimately produces.
        if (in_junction) {
            uint8_t nearest = w.left_mm < w.right_mm ? w.left_mm : w.right_mm;
            bool valid = nearest != 0 && nearest != 255;
            if (valid && (float)nearest < SWEEP_BEAM_ABORT_MM) {
                if (++beam_close_run >= SWEEP_BEAM_ABORT_N) {
                    loop_failed("beam proximity in the corner", SWEEP_FAIL_BEAM);
                    return;
                }
            } else {
                beam_close_run = 0;
            }
        }
        if (fabsf(ps.yaw_err_deg) > SWEEP_YAW_ABORT_DEG) {
            loop_failed("heading error limit", SWEEP_FAIL_YAW);
            return;
        }

        // OBSERVE WIDE, DECIDE NARROW -- and observe OUTSIDE the record.
        //
        // Arming is keyed on distance to the expected drop, not on the corner
        // record, so the feature observer sees SWEEP_GATE_OBSERVE_MM of
        // approach instead of the record's (SWEEP_TRACE_HALF_MM - R). The
        // DECISION still happens exactly where it always has: gate_arm_for()
        // holds live_from_s_mm at the later of the decide distance and the
        // record opening, so nothing can be decided before there is a record
        // to write it into.
        //
        // Arming requires the gate to be DISARMED, and corner_close() is what
        // disarms it. That makes "the next corner cannot arm until this one
        // has closed" structural rather than arithmetic -- worth more than a
        // margin calculation, because the margin depends on radius.
        if (ps.active && corner_ix < nc) {
            if (!gate.armed() &&
                ps.s_mm >= gate_expect_s(corner_ix) - SWEEP_GATE_OBSERVE_MM) {
                gate_arm_for(corner_ix);
            }
            if (gate.armed() && !gate.fired()) {
                uint8_t inside = g_left ? w.left_mm : w.right_mm;
                // DO NOT FEED THE DETECTOR A READING THAT CANNOT BE A WALL.
                // Skipping the packet outright leaves every run counter
                // untouched -- no false "present", no false "absent". That is
                // the conservative choice: it cannot invent an opening, and
                // it cannot hold a wall open either.
                //
                // BE CLEAR ABOUT WHAT THIS DOES AND DOES NOT FIX. It is a
                // guard against believing a number that is not a length. It
                // is NOT the cure for corner 3, and the run-129 traces say so
                // outright -- replaying the real packets through the gate,
                // the drop lands in exactly the same place with the filter as
                // without it (loops 1 and 2), because skipped packets do not
                // break the presence run and the drop still stamps on the
                // first 255.
                //
                // What corner 3 actually does, measured over 124 corridor
                // packets in three laps: the inside beam is BIMODAL. It
                // reports either the true wall (mean 81.2, and left+right =
                // 171.4 against a 168 corridor, so it is right) or 38-40, and
                // nothing between 41 and 67 anywhere in the run. In the true
                // branch d(right)/d(left) = -1.32 -- the two beams trade off
                // like a corridor must. In the low branch it is -0.05: the
                // mouse moves 7 mm sideways and the reading moves 0.6. A
                // distance that does not change when you move is not a
                // distance, it is the channel's floor when it has no usable
                // return. Which is why cleaning the sensor (run 127) and
                // killing the lights (run 128) changed nothing: the deficient
                // reflector is that stretch of WALL, not the optics.
                //
                // The harm is that the floor OUTLIVES the wall. The true
                // branch stops at s ~ 2760-2785; the 38s run on to 2845-2888.
                // So the drop is stamped 57 / 102 / 129 mm later than the
                // 840-850 mm corner-to-corner cadence of corners 0-2, the
                // implied correction is far outside the +/-35 window, and the
                // gate refuses it. Corner 3 is not a gate that fails to find
                // an edge -- it finds one, in the wrong place, and
                // silent-or-certain correctly throws it away. Modelled from
                // the real numbers in test_wall_gate.cpp.
                if (inside != 0 && inside != 255 &&
                    (float)inside < SWEEP_SIDE_MIN_MM) {
                    inside = 0;   // sentinel; the call is skipped below
                }
                float corr = 0.0f;
                // The SIGN-RESOLVED lateral, not wv.lateral_mm. The gate
                // rejects a drop when the mouse is more than lat_max_mm off
                // centre, and feeding it the raw pair meant a single bad
                // beam vetoed the correction: run 126's right beam sat stuck
                // at 39 through 150 mm of corridor, wv.lateral_mm read -20,
                // and corner 3 skipped its gate on EVERY lap while
                // corridor_lat() -- looking at the same packet -- correctly
                // read +5 from the left beam. One lateral, one rule.
                WallGate::Result r =
                    inside == 0
                        ? WallGate::NONE
                        : gate.update(ps.s_mm, inside,
                                      lat_src != SWEEP_LAT_NONE, lat, &corr);
                // ACT only with a record open. Before it, live_from_s_mm
                // has already suppressed the decision, so r is NONE here --
                // this is the belt to that braces.
                if (!rec_open) {
                    /* observed only */
                } else if (r == WallGate::CORRECTION) {
                    // ARC-PLACEMENT TRIM, on EVERY corner. See the long
                    // note at SWEEP_ARC_TRIM_MM for why this stopped being an
                    // alternating probe: run 121 measured 21 mm of exit per
                    // 20 mm of trim, on the outside beam alone, so placement
                    // moves the exit one for one and the crossover that
                    // proved it has done its job. Positive turns EARLIER and
                    // pulls the exit inward.
                    //
                    // It rides on the gate's own correction rather than being
                    // applied at corner_open, and that is deliberate twice
                    // over. The hypothesis under test is that the gate's
                    // geometry constant is wrong, so the trim belongs at
                    // exactly the point that constant is used. And a corner
                    // where the gate never locked gets no trim -- which is
                    // right, because there is nothing to correct against, and
                    // the record already says so by the absence of
                    // SWEEP_F_GATE_LOCK.
                    // Spread it over the room actually left before the arc:
                    // that room shrinks with radius, and a fixed rate would
                    // drain a large-R correction into the arc it set up.
                    control_path_correct(corr + SWEEP_ARC_TRIM_MM,
                                         gate_room(corner_ix));
                    rec.flags |= SWEEP_F_GATE_LOCK;
                    rec.gate_corr_x10 = (int16_t)(corr * 10.0f);
                    rec.gate_drop_s = (int16_t)gate.drop_s_mm();
                    gate_miss_streak = 0;
                } else if (r == WallGate::SKIPPED_LAT) {
                    rec.flags |= SWEEP_F_GATE_SKIP_LAT;
                } else if (r == WallGate::SKIPPED_WINDOW) {
                    rec.flags |= SWEEP_F_GATE_SKIP_WIN;
                    rec.gate_drop_s = (int16_t)gate.drop_s_mm();
                }
            }
        }

        // Everything below needs a corner record to write into.
        if (rec_open) {
            float entry_s = path.corner_entry_s_mm(corner_ix);
            float exit_s = path.corner_exit_s_mm(corner_ix);
            // Entry lateral: the last stretch of straight before the tangent.
            if (lat_src != SWEEP_LAT_NONE &&
                ps.s_mm > entry_s - SWEEP_ENTRY_WIN_FAR_MM &&
                ps.s_mm < entry_s - SWEEP_ENTRY_WIN_NEAR_MM) {
                entry_sum += lat;
                entry_n++;
                rec.lat_source = lat_src;
            }
            // Exit lateral: PAST THE JUNCTION, once the mouse is in the new
            // corridor and the wall model means something again -- but still
            // early enough that the servo has not had room to hide the error
            // (its length constant is 382 mm).
            if (lat_src != SWEEP_LAT_NONE &&
                ps.s_mm > exit_s + SWEEP_EXIT_WIN_NEAR_MM &&
                ps.s_mm < exit_s + SWEEP_EXIT_WIN_FAR_MM) {
                exit_sum += lat;
                exit_n++;
                rec.lat_source = lat_src;
            }
            if (ps.in_arc) {
                if (w.left_mm && w.left_mm < rec.arc_min_left)
                    rec.arc_min_left = w.left_mm;
                if (w.right_mm && w.right_mm < rec.arc_min_right)
                    rec.arc_min_right = w.right_mm;
                // Steady arc only: while the rate is still slewing up, the
                // lateral demand is not yet v*v/R and averaging over the
                // ramp would understate the tyres. At R45/slew2500 the ramp
                // is 29 % of the arc, which is exactly the case where a
                // ramp-contaminated mean would mislead.
                float om_full = cur_v / cur_R * 57.29578f;
                if (fabsf(ps.omega_dps) >= 0.9f * om_full) {
                    arc_ay_sum += control_motion().a_lat_mm_s2 -
                                  (float)hdr.ay_bias_mm_s2;
                    arc_ay_n++;
                }
            }

            // Distance-based trace sampling (see SweepTrace).
            if (g_trace_step > 0.0f && ps.s_mm >= next_trace_s) {
                next_trace_s = ps.s_mm + g_trace_step;
                if (!rec_tracing) {
                    rec_tracing = true;
                    rec_trace_off = hdr.trace_off;
                }
                trace_write(trace_fill(ps, w, wv, corner_ix));
            }
        } else if (g_trace_step > 0.0f && ps.s_mm >= next_crumb_s) {
            // BREADCRUMB, between corner windows. Sparse on purpose: enough
            // to say where a loop ended and in what state, not enough to
            // matter against the trace budget (~30 per loop, 480 B).
            // Tagged SWEEP_CORNER_EVENT so nothing mistakes one for corner
            // data -- the viewer draws them on the trail and excludes them
            // from the per-corner fold and the charts.
            next_crumb_s = ps.s_mm + SWEEP_BREADCRUMB_MM;
            trace_write(trace_fill(ps, w, wv, SWEEP_CORNER_EVENT));
        }
    }

    // Corner timing + yaw error, sampled off the path status rather than a
    // packet so it does not depend on the sensor stream.
    if (rec_open) {
        float entry_s = path.corner_entry_s_mm(corner_ix);
        float exit_s = path.corner_exit_s_mm(corner_ix);
        if (!entry_latched && ps.s_mm >= entry_s) {
            entry_latched = true;
            arc_t0 = now;
            arc_yaw0 = control_heading().yaw_deg;
            rec.entry_yaw_err_cdeg = (int16_t)(ps.yaw_err_deg * 100.0f);
        }
        if (!exit_latched && entry_latched && ps.s_mm >= exit_s) {
            exit_latched = true;
            rec.corner_ms = (uint16_t)(now - arc_t0);
            rec.exit_yaw_err_cdeg = (int16_t)(ps.yaw_err_deg * 100.0f);
            float turned = control_heading().yaw_deg - arc_yaw0;
            while (turned > 180.0f) turned -= 360.0f;
            while (turned < -180.0f) turned += 360.0f;
            rec.arc_turn_cdeg = (int16_t)(turned * 100.0f);
        }
    }

    if (ps.active) return;

    // ---- loop finished -------------------------------------------------------
    if (rec_open) {
        rec.wheel_minus_s_x10 =
            (int16_t)(((control_odometry().dist_mm - odo0) - ps.s_mm) * 10.0f);
        corner_close();
    }
    control_set_lateral_bias_deg(0.0f);
    if (!ps.done) {
        loop_failed("loop did not finish (timeout)", SWEEP_FAIL_TIMEOUT);
        return;
    }
    fail_streak = 0;   // a clean loop clears the streak

    // Gate health across the whole loop.
    hdr.loops_done++;
    hdr.combo_ix++;
    hdr_write();  // progress persisted BEFORE the next loop can fail
    if (gate_miss_streak >= SWEEP_GATE_MISS_ABORT) {
        sweep_abort("edge gate lost -- position no longer known",
                    SWEEP_FAIL_GATE);
        return;
    }
    (void)0;
    if (now - session_t0 > SWEEP_TIME_LIMIT_MS) {
        sweep_abort("session time limit", SWEEP_FAIL_TIMEOUT);
        return;
    }
    // Square the frame onto the corridor, then let the tyres relax before
    // the next combo launches. Squaring no longer moves the mouse, so there
    // is no manoeuvre to wait on and W_SETTLE is entered unconditionally.
    square_frame();
    state = W_SETTLE;
    state_t0 = now;
}

// ---------------------------------------------------------------- arming
void sweep_arm() {
    if (state != W_IDLE && state != W_DONE) return;
    if (!g_grid.valid()) {
        printf_line("sweep: configure the grid first ('W' prints it)");
        return;
    }
    // A fresh chip starts a session straight away; anything else waits for
    // the gesture to say whether it may be discarded.
    if (sweep_readiness() == SWEEP_READY_FRESH && hdr.magic != SWEEP_MAGIC) {
        sweep_restart();
    }
    // Apply the compiled gain override before anything flies, and re-snapshot
    // the header gains so the dump reports what ACTUALLY ran rather than what
    // was in FRAM at session start. See SWEEP_TASYM_OVERRIDE -- RAM only,
    // never persisted.
    {
        ControlGains g = control_get_gains();
        bool changed = false;
        if (SWEEP_TASYM_OVERRIDE >= 0.0f && g.tasym != SWEEP_TASYM_OVERRIDE) {
            printf_line("sweep: tasym %.3f -> %.3f for this session "
                        "(RAM only, not saved)",
                        (double)g.tasym, (double)SWEEP_TASYM_OVERRIDE);
            g.tasym = SWEEP_TASYM_OVERRIDE;
            changed = true;
        }
        // The angular feedforward scale. Applied to the value ALREADY in the
        // gains, so it composes with whatever the bench last tuned rather
        // than overwriting it with a number from this file.
        if (SWEEP_TRACK_SCALE != 1.0f && SWEEP_TRACK_SCALE > 0.0f) {
            float t = g.track_mm * SWEEP_TRACK_SCALE;
            printf_line("sweep: angular ff track %.2f -> %.2f (x%.2f) for "
                        "this session (RAM only, not saved)",
                        (double)g.track_mm, (double)t,
                        (double)SWEEP_TRACK_SCALE);
            g.track_mm = t;
            changed = true;
        }
        if (changed) control_set_gains(g);
    }
    hdr.gains = control_get_gains();
    hdr_write();

    sensors_free_run_interlaced();
    hand_run = 0;
    hand_active = false;
    hand_cal_started = false;
    ay_cal_n = 0;
    hand_wiped = false;
    last_seq = sensors_latest().seq;
    // The corridor fit is a measurement of THIS session's frame, and W_CAL
    // recalibrates the IMU after arming -- so a value carried in from a
    // previous session names a bearing that no longer exists. Loop 0 flies on
    // the body, as it did before any of this, and squares from loop 1.
    lane.reset();
    lane_open = false;
    sq_valid = false;
    sq_eps_deg = 0.0f;
    sq_corr_deg = 0.0f;
    state = W_ARMED;
    SweepReadiness rd = sweep_readiness();
    printf_line("sweep ARMED: %s",
                rd == SWEEP_READY_FRESH ? "fresh session -- wave to start"
                : rd == SWEEP_READY_RESUME
                    ? "resuming -- wave to continue"
                    : "log holds another set -- wave does nothing, hold to "
                      "discard");
    sweep_report();
}

void sweep_disarm() {
    if (state == W_ARMED || state == W_HAND_CLEAR || state == W_CAL) {
        sensors_stop();
        state = W_IDLE;
        printf_line("sweep disarmed");
    }
}

bool sweep_armed() {
    return state == W_ARMED || state == W_HAND_CLEAR || state == W_CAL;
}

bool sweep_running() { return state != W_IDLE && state != W_DONE; }

// ---------------------------------------------------------------- dump
// ---------------------------------------------------------------- recover
//
// A cleared header is not a cleared log. sweep_restart() rewrites 256 bytes
// at SWEEP_BASE and touches nothing in the summary region, so every record
// from the previous session is still sitting at SWEEP_SUM_BASE until a new
// session writes over it from index 0. That is the difference between "the
// data is gone" and "the index to the data is gone", and until now there was
// no way to tell them apart from the console.
//
// The scan walks records forward from index 0 and stops at the first one that
// cannot be a record. It deliberately does not try to be clever: a stale row
// from an older, longer session may follow a shorter new one, and the count
// is reported so the reader can see where the plausible run ends rather than
// having a guess made for them.
uint32_t sweep_recover_scan() {
    uint32_t max_rec = SWEEP_SUM_BYTES / sizeof(SweepSummary);
    uint32_t n = 0;
    for (; n < max_rec; n++) {
        SweepSummary s;
        fram_read(SWEEP_SUM_BASE + n * sizeof(s), &s, sizeof(s));
        if (!sweep_rec_plausible(s)) break;
    }
    return n;
}

void sweep_recover(void (*println)(const char *line)) {
    if (!println) return;
    uint32_t n = sweep_recover_scan();
    char b[200];
    snprintf(b, sizeof b,
             "# RECOVER: %lu plausible record(s) in the summary region "
             "(header says %lu). The header is NOT rewritten -- this is a "
             "read of what survived.",
             (unsigned long)n,
             hdr.magic == SWEEP_MAGIC ? (unsigned long)hdr.corners : 0UL);
    println(b);
    if (n == 0) {
        println("# nothing to recover: the records really are gone.");
        return;
    }
    // Dump through the normal printer by lending it a header. RAM only --
    // no hdr_write on this path, so a recover can never itself become the
    // thing that changes what is in FRAM.
    SweepHeader save = hdr;
    hdr.magic = SWEEP_MAGIC;
    hdr.corners = n;
    sweep_dump(println);
    hdr = save;
}

void sweep_dump(void (*println)(const char *line)) {
    if (!println) return;
    char b[200];
    if (hdr.magic != SWEEP_MAGIC) {
        println("no sweep log in FRAM (the region may hold a flight log)");
        return;
    }
    snprintf(b, sizeof b,
             "# sweep session %lu  dir=%s  corners=%lu  traces=%lu B%s  "
             "ay_zero=%+d mm/s2  arc_trim=%+.0f mm on ALL corners  "
             "combo_ix=%u  loops_done=%u  suspect_pairs=%lu/%lu (%.1f%%)",
             (unsigned long)hdr.session, (hdr.flags & 1) ? "L" : "R",
             (unsigned long)hdr.corners, (unsigned long)hdr.trace_off,
             hdr.trace_wrapped ? " (WRAPPED)" : "",
             (int)hdr.ay_bias_mm_s2, (double)SWEEP_ARC_TRIM_MM,
             (unsigned)hdr.combo_ix, (unsigned)hdr.loops_done,
             (unsigned long)hdr.suspect_pairs,
             (unsigned long)hdr.pair_samples,
             hdr.pair_samples
                 ? 100.0 * hdr.suspect_pairs / hdr.pair_samples : 0.0);
    println(b);
    snprintf(b, sizeof b,
             "# gains kp=%.5f kpt=%.5f kyaw=%.2f track=%.2f tasym=%.3f "
             "strim=%.5f circl=%.2f circr=%.2f",
             (double)hdr.gains.kp, (double)hdr.gains.kp_turn,
             (double)hdr.gains.kyaw, (double)hdr.gains.track_mm,
             (double)hdr.gains.tasym, (double)hdr.gains.strim,
             (double)hdr.gains.circ_left_mm, (double)hdr.gains.circ_right_mm);
    println(b);
    println("S,loop,corner,dir,R,v_arc,slew_ix,entry_lat,exit_lat,"
            "entry_yaw_err,exit_yaw_err,gate_corr,gate_drop_s,arc_min_l,"
            "arc_min_r,corner_ms,wheel_minus_s,fail,lat_src,flags,trace_off,"
            "arc_turn,arc_ay,a_dem");

    static const char *FAIL[] = {"ok", "lateral", "beam", "yaw",
                                 "timeout", "gate", "user", "ramp"};
    static const char *LSRC[] = {"none", "both", "one", "suspect"};
    static const char *SQ[] = {"nofit", "squared", "refused", "body"};
    bool xhdr = false;   // the X header prints only if there is an X row
    bool qhdr = false;   // ditto the Q header
    bool fhdr = false;   // ditto the F header
    for (uint32_t i = 0; i < hdr.corners; i++) {
        SweepSummary s;
        fram_read(SWEEP_SUM_BASE + i * sizeof(s), &s, sizeof(s));
        // A loop event is not a corner and must not be read as one. Its
        // fields carry the state at the stop (see SweepSummary in the
        // header), so it gets its own row type and its own column names.
        // A feature row is not a corner and does not even share the corner
        // record's field layout -- it overlays its own struct. Decoded via
        // SweepFeatureRow rather than by re-reading fields.
        if (s.corner_ix == SWEEP_CORNER_FEATURE) {
            SweepFeatureRow f;
            memcpy(&f, &s, sizeof(f));
            if (!fhdr) {
                fhdr = true;
                println("# F rows: what the INSIDE beam did on the approach. "
                        "One run = an unbroken stretch of something near. A "
                        "wall is a long run, a post is a short one, a gap is "
                        "the space between two. n is packets, not mm.");
                println("F,loop,corner,dir,runs_seen,r0_s0,r0_s1,r0_min,r0_n,"
                        "r1_s0,r1_s1,r1_min,r1_n,r2_s0,r2_s1,r2_min,r2_n,"
                        "r3_s0,r3_s1,r3_min,r3_n,r4_s0,r4_s1,r4_min,r4_n");
            }
            int off = snprintf(b, sizeof b, "F,%u,%u,%c,%u",
                               f.loop_ix, f.corner_ix,
                               (f.flags & SWEEP_F_LEFT) ? 'L' : 'R',
                               f.runs_seen);
            for (int k = 0; k < 5 && off > 0 && off < (int)sizeof b; k++)
                off += snprintf(b + off, sizeof b - off, ",%d,%d,%u,%u",
                                f.run[k].s0_mm, f.run[k].s1_mm,
                                f.run[k].min_mm, f.run[k].n);
            println(b);
            continue;
        }
        // A launch row is not a corner either. It carries what the corridor
        // squaring measured and the pose the loop launched from -- see
        // SWEEP_CORNER_LAUNCH for the field mapping.
        if (s.corner_ix == SWEEP_CORNER_LAUNCH) {
            if (!qhdr) {
                qhdr = true;
                println("# Q rows: one per loop, written at launch. "
                        "frame_corr is the gyro error accumulated over ONE "
                        "lap (squaring zeroes it each lap); same sign lap "
                        "over lap is a drift. eps is the TRAVEL angle and "
                        "reads the lateral servo still converging -- not a "
                        "drift. See SWEEP_CORNER_LAUNCH.");
                // Pure CSV on the header line: an explanatory tail after the
                // final comma silently becomes part of the last field name.
                println("Q,loop,dir,R,v_arc,slew_ix,eps,frame_corr,body_off,"
                        "cardinal,lat0,lat_src,left,right,fit_n,fit_span,"
                        "fit_rej,square,a_lon");
            }
            snprintf(b, sizeof b,
                     "Q,%u,%c,%u,%u,%u,%.2f,%.2f,%.2f,%.0f,%.1f,%s,%u,%u,%u,"
                     "%d,%d,%s,%d",
                     s.loop_ix, (s.flags & SWEEP_F_LEFT) ? 'L' : 'R',
                     s.r_mm, s.v_arc, s.slew_ix,
                     s.entry_yaw_err_cdeg / 100.0,
                     s.exit_yaw_err_cdeg / 100.0,
                     s.arc_turn_cdeg / 100.0,
                     s.exit_lat_x10 / 10.0,
                     s.entry_lat_x10 / 10.0,
                     LSRC[s.lat_source < 4 ? s.lat_source : 0],
                     s.arc_min_left, s.arc_min_right, s.corner_ms,
                     s.gate_drop_s, s.gate_corr_x10,
                     SQ[s.fail < 4 ? s.fail : 0], s.arc_ay_mm_s2);
            println(b);
            continue;
        }
        if (s.corner_ix == SWEEP_CORNER_EVENT) {
            if (!xhdr) {
                xhdr = true;
                // Pure CSV on the header line -- the last field name is
                // parsed by column name at the other end, and an explanatory
                // tail after the final comma silently becomes part of it.
                println("# X rows: a loop that ended BETWEEN corner windows. "
                        "No corner to describe, so these carry the state at "
                        "the stop.");
                println("X,loop,dir,R,v_arc,slew_ix,at_s,to_corner,lat,"
                        "lat_prev,lat_src,yaw_err,left,right,loop_ms,"
                        "wheel_minus_s,fail");
            }
            snprintf(b, sizeof b,
                     "X,%u,%c,%u,%u,%u,%d,%d,%.1f,%.1f,%s,%.2f,%u,%u,%u,"
                     "%.1f,%s",
                     s.loop_ix, (s.flags & SWEEP_F_LEFT) ? 'L' : 'R',
                     s.r_mm, s.v_arc, s.slew_ix, s.gate_drop_s,
                     s.exit_lat_x10, s.entry_lat_x10 / 10.0,
                     s.arc_turn_cdeg / 10.0,
                     LSRC[s.lat_source < 4 ? s.lat_source : 0],
                     s.entry_yaw_err_cdeg / 100.0, s.arc_min_left,
                     s.arc_min_right, s.corner_ms,
                     s.wheel_minus_s_x10 / 10.0,
                     FAIL[s.fail < 8 ? s.fail : 0]);
            println(b);
            continue;
        }
        snprintf(b, sizeof b,
                 "S,%u,%u,%c,%u,%u,%u,%.1f,%.1f,%.2f,%.2f,%.1f,%d,%u,%u,%u,"
                 "%.1f,%s,%s,0x%02X,%lu,%.2f,%d,%.0f",
                 s.loop_ix, s.corner_ix, (s.flags & SWEEP_F_LEFT) ? 'L' : 'R',
                 s.r_mm, s.v_arc, s.slew_ix, s.entry_lat_x10 / 10.0,
                 s.exit_lat_x10 / 10.0, s.entry_yaw_err_cdeg / 100.0,
                 s.exit_yaw_err_cdeg / 100.0, s.gate_corr_x10 / 10.0,
                 s.gate_drop_s, s.arc_min_left, s.arc_min_right, s.corner_ms,
                 s.wheel_minus_s_x10 / 10.0,
                 FAIL[s.fail < 8 ? s.fail : 0],
                 LSRC[s.lat_source < 4 ? s.lat_source : 0], s.flags,
                 (unsigned long)s.trace_off, s.arc_turn_cdeg / 100.0,
                 s.arc_ay_mm_s2,
                 s.r_mm ? (double)s.v_arc * s.v_arc / s.r_mm : 0.0);
        println(b);
    }

    if (hdr.trace_off == 0 && !hdr.trace_wrapped) return;
    // a_dem is computed here rather than stored: it is exactly v * omega,
    // both of which are already in the record, and the whole question is
    // whether ay kept up with it. Printing them side by side means the
    // answer is visible in the dump without a spreadsheet.
    //   ay ~ a_dem            -> the tyres held the arc
    //   |ay| < |a_dem|        -> the mouse went straighter than commanded,
    //                            and the deficit integrated twice is how far
    //                            it slid.
    // lat is gone from the record; it was (right - left) / 2, and both raw
    // beams are still here.
    println("T,off,s,v,yaw,yaw_err,omega,ay,a_dem,left,right,in_arc,corner");
    uint32_t cap = SWEEP_TRACE_END - SWEEP_TRACE_BASE;
    uint32_t end = hdr.trace_wrapped ? cap : hdr.trace_off;
    for (uint32_t off = 0; off + sizeof(SweepTrace) <= end;
         off += sizeof(SweepTrace)) {
        SweepTrace t;
        fram_read(SWEEP_TRACE_BASE + off, &t, sizeof(t));
        double a_dem = (double)t.v_mm_s * t.omega_dps * (M_PI / 180.0);
        snprintf(b, sizeof b, "T,%lu,%d,%d,%.2f,%.2f,%d,%d,%.0f,%u,%u,%u,%u",
                 (unsigned long)off, t.s_mm, t.v_mm_s, t.yaw_cdeg / 100.0,
                 t.yaw_err_cdeg / 100.0, t.omega_dps, t.ay_mm_s2, a_dem,
                 t.left_mm, t.right_mm, (unsigned)(t.flags & 1),
                 t.corner_ix);
        println(b);
    }
}
