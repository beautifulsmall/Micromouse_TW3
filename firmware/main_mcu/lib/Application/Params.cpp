#include "Params.h"

#include <math.h>
#include <string.h>

#include "Drivers.h"

// Previous layout (version 10), the last of the frozen-history packed
// structs -- v11 replaced the whole parade with a registry-order value
// array (see Params.h). Still accepted at boot; do NOT edit.
struct ParamsBlockV10 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;  // sizeof(ParamsBlock), guards against layout drift
    float vmax_left_mm_s;
    float vmax_right_mm_s;
    float friction_left;
    float friction_right;
    float kp;
    float kp_turn;
    float kd;
    float ka;
    float kx;
    float kyaw;
    float strim;
    float track_mm;
    float tasym;
    float circ_left_mm;
    float circ_right_mm;
    float kslip;      // v10: slip-angle feedforward
    uint32_t crc;  // CRC-32 over everything above
};

// Previous layout (version 8): single circ_mm. Migrated -> both wheels.
struct ParamsBlockV9 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;  // sizeof(ParamsBlock), guards against layout drift
    float vmax_left_mm_s;
    float vmax_right_mm_s;
    float friction_left;
    float friction_right;
    float kp;
    float kp_turn;
    float kd;
    float ka;
    float kx;
    float kyaw;
    float strim;
    float track_mm;
    float tasym;
    float circ_left_mm;
    float circ_right_mm;
    uint32_t crc;  // CRC-32 over everything above
};

// Previous layout (version 8): single circ_mm. Migrated -> both wheels.
struct ParamsBlockV8 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float vmax_left_mm_s;
    float vmax_right_mm_s;
    float friction_left;
    float friction_right;
    float kp;
    float kp_turn;
    float kd;
    float ka;
    float kx;
    float kyaw;
    float strim;
    float track_mm;
    float tasym;
    float circ_mm;
    uint32_t crc;
};

// Previous layout (version 7): pre-circ_mm. Migrated (circ defaults).
struct ParamsBlockV7 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float vmax_left_mm_s;
    float vmax_right_mm_s;
    float friction_left;
    float friction_right;
    float kp;
    float kp_turn;
    float kd;
    float ka;
    float kx;
    float kyaw;
    float strim;
    float track_mm;
    float tasym;
    uint32_t crc;
};

// Previous layout (version 6), still accepted by params_load() so a saved
// block survives the firmware upgrade. Do NOT edit -- frozen history.
struct ParamsBlockV6 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float vmax_left_mm_s;
    float vmax_right_mm_s;
    float friction_left;
    float friction_right;
    float kp;
    float kp_turn;
    float kd;
    float ka;
    float kx;
    float kyaw;
    float strim;
    float track_mm;
    uint32_t crc;
};

// Previous layout (version 5), still accepted by params_load() so a saved
// block survives the firmware upgrade. Do NOT edit -- frozen history.
struct ParamsBlockV5 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float vmax_left_mm_s;
    float vmax_right_mm_s;
    float friction_left;
    float friction_right;
    float kp;
    float kd;
    float ka;
    float kx;
    float kyaw;
    float strim;
    float track_mm;
    uint32_t crc;
};

// Previous layout (version 4), still accepted by params_load() so a saved
// block survives the firmware upgrade. Do NOT edit -- frozen history.
struct ParamsBlockV4 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float vmax_left_mm_s;
    float vmax_right_mm_s;
    float friction_left;
    float friction_right;
    float kp;
    float kd;
    float ka;
    float kx;
    float kyaw;
    float strim;
    uint32_t crc;
};

// Previous layout (version 3), still accepted by params_load() so a saved
// block survives the firmware upgrade. Do NOT edit -- frozen history.
struct ParamsBlockV3 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float vmax_left_mm_s;
    float vmax_right_mm_s;
    float friction_left;
    float friction_right;
    float kp;
    float kd;
    float ka;
    float kx;
    float kyaw;
    uint32_t crc;
};

// Previous layout (version 2), still accepted by params_load() so a saved
// block survives the firmware upgrade. Do NOT edit -- frozen history.
struct ParamsBlockV2 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float vmax_left_mm_s;
    float vmax_right_mm_s;
    float friction_left;
    float friction_right;
    float kp;
    float kd;
    float ka;
    uint32_t crc;
};

uint32_t params_crc32(const void *data, size_t len) {
    const uint8_t *p = static_cast<const uint8_t *>(data);
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320UL & (0 - (crc & 1)));
        }
    }
    return ~crc;
}

static constexpr size_t CRC_SPAN_V10 =
    sizeof(ParamsBlockV10) - sizeof(uint32_t);
static constexpr size_t CRC_SPAN_V9 =
    sizeof(ParamsBlockV9) - sizeof(uint32_t);
static constexpr size_t CRC_SPAN_V8 = sizeof(ParamsBlockV8) - sizeof(uint32_t);
static constexpr size_t CRC_SPAN_V7 = sizeof(ParamsBlockV7) - sizeof(uint32_t);
static constexpr size_t CRC_SPAN_V6 = sizeof(ParamsBlockV6) - sizeof(uint32_t);
static constexpr size_t CRC_SPAN_V5 = sizeof(ParamsBlockV5) - sizeof(uint32_t);
static constexpr size_t CRC_SPAN_V4 = sizeof(ParamsBlockV4) - sizeof(uint32_t);
static constexpr size_t CRC_SPAN_V3 = sizeof(ParamsBlockV3) - sizeof(uint32_t);
static constexpr size_t CRC_SPAN_V2 = sizeof(ParamsBlockV2) - sizeof(uint32_t);

// Migration: accept a v8 block (single circ) -> seed both wheels from it.
// v9 -> v10 migration: everything carries over, kslip starts at its compiled
// default (0 = inert). Without this, adding the gain would fail the size guard
// and silently drop every tuned value back to defaults on the next boot.
static bool params_load_v9(ControlGains &g) {
    ParamsBlockV9 b;
    fram_read(PARAMS_FRAM_ADDR, &b, sizeof(b));
    if (b.magic != PARAMS_MAGIC) return false;
    if (b.version != 9 || b.size != sizeof(ParamsBlockV9)) return false;
    if (b.crc != params_crc32(&b, CRC_SPAN_V9)) return false;
    g.vmax_left_mm_s = b.vmax_left_mm_s;
    g.vmax_right_mm_s = b.vmax_right_mm_s;
    g.friction_left = b.friction_left;
    g.friction_right = b.friction_right;
    g.kp = b.kp;
    g.kp_turn = b.kp_turn;
    g.kd = b.kd;
    g.ka = b.ka;
    g.kx = b.kx;
    g.kyaw = b.kyaw;
    g.strim = b.strim;
    g.track_mm = b.track_mm;
    g.tasym = b.tasym;
    g.circ_left_mm = b.circ_left_mm;
    g.circ_right_mm = b.circ_right_mm;
    return true;   // kslip keeps its default
}

static bool params_load_v8(ControlGains &g) {
    ParamsBlockV8 b;
    fram_read(PARAMS_FRAM_ADDR, &b, sizeof(b));
    if (b.magic != PARAMS_MAGIC) return false;
    if (b.version != 8 || b.size != sizeof(ParamsBlockV8)) return false;
    if (b.crc != params_crc32(&b, CRC_SPAN_V8)) return false;
    g.vmax_left_mm_s = b.vmax_left_mm_s;
    g.vmax_right_mm_s = b.vmax_right_mm_s;
    g.friction_left = b.friction_left;
    g.friction_right = b.friction_right;
    g.kp = b.kp;
    g.kp_turn = b.kp_turn;
    g.kd = b.kd;
    g.ka = b.ka;
    g.kx = b.kx;
    g.kyaw = b.kyaw;
    g.strim = b.strim;
    g.track_mm = b.track_mm;
    g.tasym = b.tasym;
    if (b.circ_mm > 1.0f) { g.circ_left_mm = b.circ_mm; g.circ_right_mm = b.circ_mm; }
    return true;
}

// Migration: accept a valid v7 block; circ_mm takes the compiled default.
static bool params_load_v7(ControlGains &g) {
    ParamsBlockV7 b;
    fram_read(PARAMS_FRAM_ADDR, &b, sizeof(b));
    if (b.magic != PARAMS_MAGIC) return false;
    if (b.version != 7 || b.size != sizeof(ParamsBlockV7)) return false;
    if (b.crc != params_crc32(&b, CRC_SPAN_V7)) return false;
    g.vmax_left_mm_s = b.vmax_left_mm_s;
    g.vmax_right_mm_s = b.vmax_right_mm_s;
    g.friction_left = b.friction_left;
    g.friction_right = b.friction_right;
    g.kp = b.kp;
    g.kp_turn = b.kp_turn;
    g.kd = b.kd;
    g.ka = b.ka;
    g.kx = b.kx;
    g.kyaw = b.kyaw;
    g.strim = b.strim;
    g.track_mm = b.track_mm;
    g.tasym = b.tasym;
    return true;
}

// Migration: accept a valid v6 block, keeping its fields and leaving the
// field it predates (tasym) at the compiled default.
static bool params_load_v6(ControlGains &g) {
    ParamsBlockV6 b;
    fram_read(PARAMS_FRAM_ADDR, &b, sizeof(b));
    if (b.magic != PARAMS_MAGIC) return false;
    if (b.version != 6 || b.size != sizeof(ParamsBlockV6)) return false;
    if (b.crc != params_crc32(&b, CRC_SPAN_V6)) return false;

    g.vmax_left_mm_s = b.vmax_left_mm_s;
    g.vmax_right_mm_s = b.vmax_right_mm_s;
    g.friction_left = b.friction_left;
    g.friction_right = b.friction_right;
    g.kp = b.kp;
    g.kp_turn = b.kp_turn;
    g.kd = b.kd;
    g.ka = b.ka;
    g.kx = b.kx;
    g.kyaw = b.kyaw;
    g.strim = b.strim;
    g.track_mm = b.track_mm;
    return true;
}

// Migration: accept a valid v5 block. v5 predates the kp split; a block saved
// after a 't' auto-tune holds the TURN-stiff kp, so seed kp_turn from the
// stored kp (kp itself is kept -- restore the straight value by hand if it
// was left turn-stiff).
static bool params_load_v5(ControlGains &g) {
    ParamsBlockV5 b;
    fram_read(PARAMS_FRAM_ADDR, &b, sizeof(b));
    if (b.magic != PARAMS_MAGIC) return false;
    if (b.version != 5 || b.size != sizeof(ParamsBlockV5)) return false;
    if (b.crc != params_crc32(&b, CRC_SPAN_V5)) return false;

    g.vmax_left_mm_s = b.vmax_left_mm_s;
    g.vmax_right_mm_s = b.vmax_right_mm_s;
    g.friction_left = b.friction_left;
    g.friction_right = b.friction_right;
    g.kp = b.kp;
    g.kp_turn = b.kp;  // best available seed (see comment above)
    g.kd = b.kd;
    g.ka = b.ka;
    g.kx = b.kx;
    g.kyaw = b.kyaw;
    g.strim = b.strim;
    g.track_mm = b.track_mm;
    return true;
}

// Migration: accept a valid v4 block, keeping its fields and leaving the
// fields it predates (track_mm) at compiled defaults.
static bool params_load_v4(ControlGains &g) {
    ParamsBlockV4 b;
    fram_read(PARAMS_FRAM_ADDR, &b, sizeof(b));
    if (b.magic != PARAMS_MAGIC) return false;
    if (b.version != 4 || b.size != sizeof(ParamsBlockV4)) return false;
    if (b.crc != params_crc32(&b, CRC_SPAN_V4)) return false;

    g.vmax_left_mm_s = b.vmax_left_mm_s;
    g.vmax_right_mm_s = b.vmax_right_mm_s;
    g.friction_left = b.friction_left;
    g.friction_right = b.friction_right;
    g.kp = b.kp;
    g.kd = b.kd;
    g.ka = b.ka;
    g.kx = b.kx;
    g.kyaw = b.kyaw;
    g.strim = b.strim;
    return true;
}

static bool params_load_v3(ControlGains &g) {
    ParamsBlockV3 b;
    fram_read(PARAMS_FRAM_ADDR, &b, sizeof(b));
    if (b.magic != PARAMS_MAGIC) return false;
    if (b.version != 3 || b.size != sizeof(ParamsBlockV3)) return false;
    if (b.crc != params_crc32(&b, CRC_SPAN_V3)) return false;

    g.vmax_left_mm_s = b.vmax_left_mm_s;
    g.vmax_right_mm_s = b.vmax_right_mm_s;
    g.friction_left = b.friction_left;
    g.friction_right = b.friction_right;
    g.kp = b.kp;
    g.kd = b.kd;
    g.ka = b.ka;
    g.kx = b.kx;
    g.kyaw = b.kyaw;
    return true;
}

// Migration: accept a valid v2 block, keeping its fields and leaving the
// fields it predates (kx, kyaw) at compiled defaults.
static bool params_load_v2(ControlGains &g) {
    ParamsBlockV2 b;
    fram_read(PARAMS_FRAM_ADDR, &b, sizeof(b));
    if (b.magic != PARAMS_MAGIC) return false;
    if (b.version != 2 || b.size != sizeof(ParamsBlockV2)) return false;
    if (b.crc != params_crc32(&b, CRC_SPAN_V2)) return false;

    g.vmax_left_mm_s = b.vmax_left_mm_s;
    g.vmax_right_mm_s = b.vmax_right_mm_s;
    g.friction_left = b.friction_left;
    g.friction_right = b.friction_right;
    g.kp = b.kp;
    g.kd = b.kd;
    g.ka = b.ka;
    return true;
}

static bool params_load_v10(ControlGains &g) {
    ParamsBlockV10 b;
    fram_read(PARAMS_FRAM_ADDR, &b, sizeof(b));
    if (b.magic != PARAMS_MAGIC) return false;
    if (b.version != 10 || b.size != sizeof(ParamsBlockV10)) return false;
    if (b.crc != params_crc32(&b, CRC_SPAN_V10)) return false;
    g.vmax_left_mm_s = b.vmax_left_mm_s;
    g.vmax_right_mm_s = b.vmax_right_mm_s;
    g.friction_left = b.friction_left;
    g.friction_right = b.friction_right;
    g.kp = b.kp;
    g.kp_turn = b.kp_turn;
    g.kd = b.kd;
    g.ka = b.ka;
    g.kx = b.kx;
    g.kyaw = b.kyaw;
    g.strim = b.strim;
    g.track_mm = b.track_mm;
    g.tasym = b.tasym;
    g.circ_left_mm = b.circ_left_mm;
    g.circ_right_mm = b.circ_right_mm;
    g.kslip = b.kslip;
    return true;   // arc_understeer keeps its default (v10 predates it)
}

// -- THE REGISTRY (see Params.h for the rules: append-only, clamp at every
// door, one table that everything walks) --------------------------------------
//
// Ranges are HARD physical clamps, not tuning advice -- generous enough that
// no deliberate experiment hits them, tight enough that a corrupted value
// cannot reach the ISR. Two are worth their own line:
//
//   ka:   2e-5 max. The physically correct value is tau_plant/vmax ~ 2.5e-6;
//         the 2026-07-09 corruption that cost two days was 1.5e-4. The clamp
//         sits 8x above physics and 7x below the corruption.
//   circ: 60..75 mm brackets the real wheel by ~10 %; a value outside that
//         is not a wheel, it is a bad byte.
const ParamDef PARAM_REGISTRY[PARAM_COUNT] = {
    {"vmaxl", &ControlGains::vmax_left_mm_s, 1000.0f, 20000.0f},
    {"vmaxr", &ControlGains::vmax_right_mm_s, 1000.0f, 20000.0f},
    {"fricl", &ControlGains::friction_left, 0.0f, 0.2f},
    {"fricr", &ControlGains::friction_right, 0.0f, 0.2f},
    {"kp", &ControlGains::kp, 0.0f, 0.01f},
    // kpt 0..0.1 and strim +/-0.2: the first cut of these ranges (0.01,
    // 0.05) was TOO TIGHT and clamped bert's real tuned values on the very
    // first param.diff (kpt printed exactly 0.01, strim exactly -0.05 --
    // values sitting exactly on a clamp boundary are the clamp, not the
    // tuning). Ranges are for stopping corruption, not for encoding tuning
    // opinions; when in doubt, 10x past anything ever deliberately used.
    {"kpt", &ControlGains::kp_turn, 0.0f, 0.1f},
    {"kd", &ControlGains::kd, 0.0f, 0.01f},
    // ka 2e-5 -> 1e-4 (2026-09-23): the first slip ladder put tomorrow's
    // value at ~5e-5 (0.55 of duty made ~9500 mm/s2 in the 16000 log), and
    // the old ceiling would have clipped it to 2e-5. 1e-4 keeps the
    // 2026-07-09 corruption value (1.5e-4) out of the ISR.
    {"ka", &ControlGains::ka, 0.0f, 1e-4f},
    {"kx", &ControlGains::kx, 0.0f, 20.0f},
    {"kyaw", &ControlGains::kyaw, 0.0f, 50.0f},
    {"strim", &ControlGains::strim, -0.2f, 0.2f},
    {"track", &ControlGains::track_mm, 40.0f, 120.0f},
    {"tasym", &ControlGains::tasym, -0.5f, 0.5f},
    {"us", &ControlGains::arc_understeer, -0.5f, 0.5f},
    {"kslip", &ControlGains::kslip, 0.0f, 0.01f},
    {"circl", &ControlGains::circ_left_mm, 60.0f, 75.0f},
    {"circr", &ControlGains::circ_right_mm, 60.0f, 75.0f},
    // -- appended 2026-08-27 (see the note in Control.h at these fields) ----
    {"vfilt", &ControlGains::vfilt, 0.02f, 1.0f},
    {"dvmax", &ControlGains::dvmax, 20.0f, 1000.0f},
    {"kyawi", &ControlGains::kyawi, 0.0f, 0.5f},
    {"kyawimax", &ControlGains::kyawimax, 0.0f, 1.0f},
    {"rv", &ControlGains::route_v, 100.0f, 3000.0f},
    {"ralon", &ControlGains::route_alon, 100.0f, 20000.0f},
    {"ralat", &ControlGains::route_alat, 500.0f, 20000.0f},
    {"rslew", &ControlGains::route_slew, 1000.0f, 20000.0f},
    {"kparc", &ControlGains::kp_arc, 0.0f, 0.1f},
    {"goff", &ControlGains::gate_off_mm, -20.0f, 20.0f},
    // rur: 0 is MEANINGFUL (solve the radius from the rail), so the range
    // must include it; anything else below 45 mm cannot be flown.
    {"rur", &ControlGains::route_uturn_r, 0.0f, 95.0f},
    // Per-corner-type gate offsets (added to goff for corners of the type).
    {"goffr", &ControlGains::goff_r90, -20.0f, 20.0f},
    {"goffl", &ControlGains::goff_l90, -20.0f, 20.0f},
    {"goffur", &ControlGains::goff_ur, -20.0f, 20.0f},
    {"gofful", &ControlGains::goff_ul, -20.0f, 20.0f},
    // Gate lateral compensation coefficient (WallGate klat), mm/mm.
    // Clamp widened +/-1.5 -> +/-3 (2026-09-05): the (3,5) CW inside beam
    // measured ~2.3 mm of drop shift per mm of lateral across laps 28-30.
    {"gklat", &ControlGains::gate_klat, -3.0f, 3.0f},
    {"gklatr", &ControlGains::gate_klat_r, -3.0f, 3.0f},
    {"glearn", &ControlGains::gate_learn, 0.0f, 1.0f},
    {"r90", &ControlGains::r90_right, 0.0f, 200.0f},
    {"l90", &ControlGains::r90_left, 0.0f, 200.0f},
    {"atrim", &ControlGains::arc_trim_90, -30.0f, 30.0f},
    {"utrim", &ControlGains::arc_trim_u, -30.0f, 30.0f},
    {"fgate", &ControlGains::front_gate, 0.0f, 3.0f},
    {"ftrim", &ControlGains::front_trim, -40.0f, 40.0f},
    {"dgate", &ControlGains::diag_gate, 0.0f, 2.0f},
    {"dtrim", &ControlGains::diag_trim, -40.0f, 40.0f},
    {"dlat", &ControlGains::diag_lat, -20.0f, 20.0f},
    // Floor 0.30: a mouse that cannot reach cruise feedforward + one arc's
    // differential is undrivable, not protected. Ceiling 1.0: full battery
    // voltage; above 0.85 the stall margin shrinks, raise knowingly.
    {"dmax", &ControlGains::duty_limit, 0.30f, 1.0f},
    // Slip model, mm per m/s of commanded speed change (ControlGains).
    {"sacc", &ControlGains::slip_acc, 0.0f, 200.0f},
    {"sbrk", &ControlGains::slip_brk, 0.0f, 200.0f},
    {"flinch", &ControlGains::flinch_mm, 0.0f, 70.0f},
    {"ftau", &ControlGains::flinch_tau_ms, 0.0f, 500.0f},
    {"walign", &ControlGains::wall_align, 0.0f, 1.0f},
    {"vout", &ControlGains::v_out_mm, 0.0f, 15.0f},
    {"latk", &ControlGains::lat_servo_k, 0.05f, 0.6f},
    {"starty", &ControlGains::start_y_mm, 30.0f, 90.0f},
    {"r135", &ControlGains::route_135, 0.0f, 1.0f},
    {"sv", &ControlGains::search_v_mm_s, 50.0f, 400.0f},
    {"tomg", &ControlGains::turn_omega_dps, 100.0f, 720.0f},
    {"talp", &ControlGains::turn_alpha_dps2, 500.0f, 10000.0f},
    {"rv2", &ControlGains::rv_mr2, 200.0f, 2500.0f},
    {"rv3", &ControlGains::rv_mr3, 200.0f, 2500.0f},
    {"rv4", &ControlGains::rv_mr4, 200.0f, 2500.0f},
    {"rv5", &ControlGains::rv_mr5, 200.0f, 2500.0f},
    {"rvh", &ControlGains::rv_home, 200.0f, 2500.0f},
    {"absb", &ControlGains::abs_band, 0.0f, 0.5f},   // ABS band, 0 = off
    {"vsrc", &ControlGains::vel_source, 0.0f, 1.0f},  // wheel velocity: 0 counts, 1 edge timing
    {"kaa", &ControlGains::kaa, 0.0f, 1e-4f},         // angular accel FF (arcs); 0 = the 09-22 arcs
    {"rbrk", &ControlGains::route_brk, 0.0f, 20000.0f}, // route brake decel; 0 = ralon
    {"mra", &ControlGains::ladder_accel_mr2, 0.1f, 1.0f},  // ladder accel scale at MR2 (1 = flat)
    {"vref", &ControlGains::vbat_ref, 0.0f, 20.0f},        // vbat reading the duty law was fitted at (0 = off)
    {"chold", &ControlGains::clothoid_hold, 0.0f, 1.0f},   // planner holds arc speed through clothoids (1) or brakes/launches through them (0)
    {"rdiag", &ControlGains::route_diag, 0.0f, 1.0f},      // route compiler may fly diagonals (1) or orthogonal 90s only (0)
    {"rturn", &ControlGains::route_turn_cost, 0.0f, 30.0f}, // route choice: cells of cost per 90 turn (0 = plain cell flood)
    {"dwheel", &ControlGains::wheel_duty_limit, 0.0f, 1.0f}, // per-wheel duty ceiling for steering above dmax (0 = old per-wheel clip)
    {"rturnh", &ControlGains::route_turn_cost_home, 0.0f, 30.0f}, // HOME leg route choice: cells per turn (0 = rturn)
    {"rdv", &ControlGains::diag_v_max, 0.0f, 3000.0f},             // diagonals only at out-leg peaks <= this (0 = no limit)
    {"rvv", &ControlGains::v_cap_v, 0.0f, 2500.0f},                // speed cap on a V and the turn within 120 mm after it (0 = none)
    {"rvu", &ControlGains::v_cap_u, 0.0f, 2500.0f},                // speed cap on a U-turn (0 = none)
    {"vlearn", &ControlGains::v_learn, 0.0f, 1.0f},                 // V timing learner on (1) / off (0)
    {"ghome", &ControlGains::goal_home, 0.0f, 1.0f},                // home leg leaves the goal: 0 turn in place + press back, 1 loop the 2x2 block
    // APPEND new rows HERE, never above -- the stored block is these values
    // in this order, and test_params pins the order with a golden list.
};

const ParamDef *param_find(const char *name) {
    for (int i = 0; i < PARAM_COUNT; i++)
        if (strcmp(PARAM_REGISTRY[i].name, name) == 0) return &PARAM_REGISTRY[i];
    return nullptr;
}

int params_clamp(ControlGains &g) {
    int hit = 0;
    for (int i = 0; i < PARAM_COUNT; i++) {
        const ParamDef &d = PARAM_REGISTRY[i];
        float &v = g.*(d.field);
        // NaN fails both comparisons below and would sail through untouched,
        // straight into the ISR -- and FRAM corruption is exactly where NaN
        // bit patterns come from. Send it to the compiled default.
        if (isnan(v)) { v = ControlGains{}.*(d.field); hit++; continue; }
        if (v < d.min) { v = d.min; hit++; }
        else if (v > d.max) { v = d.max; hit++; }
    }
    return hit;
}

// -- v11: values in registry order at an arbitrary address --------------------

struct ParamsHdrV11 {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
};

bool params_save_at(uint32_t addr, const ControlGains &g) {
    uint8_t buf[PARAMS_BLOCK_BYTES];
    ParamsHdrV11 h{PARAMS_MAGIC, PARAMS_VERSION, (uint16_t)PARAM_COUNT};
    memcpy(buf, &h, sizeof h);
    float v[PARAM_COUNT];
    for (int i = 0; i < PARAM_COUNT; i++) v[i] = g.*(PARAM_REGISTRY[i].field);
    memcpy(buf + sizeof h, v, sizeof v);
    uint32_t crc = params_crc32(buf, sizeof h + sizeof v);
    memcpy(buf + sizeof h + sizeof v, &crc, sizeof crc);
    fram_write(addr, buf, sizeof buf);
    return fram_ready();
}

bool params_load_at(uint32_t addr, ControlGains &g) {
    ParamsHdrV11 h;
    fram_read(addr, &h, sizeof h);
    if (h.magic != PARAMS_MAGIC || h.version != PARAMS_VERSION) return false;
    // A block from a FUTURE firmware (more params than we know) is refused
    // outright rather than half-read: the first PARAM_COUNT values would load
    // correctly (append-only guarantees that), but the CRC spans values we
    // cannot check, and "probably fine" is not a standard for ISR inputs.
    if (h.count == 0 || h.count > PARAM_COUNT) return false;
    const size_t span = sizeof(ParamsHdrV11) + h.count * 4;
    uint8_t buf[sizeof(ParamsHdrV11) + PARAM_COUNT * 4];
    fram_read(addr, buf, span);
    uint32_t crc;
    fram_read(addr + span, &crc, sizeof crc);
    if (crc != params_crc32(buf, span)) return false;
    const float *v = (const float *)(buf + sizeof(ParamsHdrV11));
    // An OLDER block (fewer params) loads what it has; the appended
    // parameters keep their compiled defaults. This is the whole migration
    // story from v11 on.
    for (int i = 0; i < (int)h.count; i++) g.*(PARAM_REGISTRY[i].field) = v[i];
    return true;
}

bool params_load(ControlGains &g) {
    g = ControlGains{};  // defaults unless a valid block replaces them

    ParamsHdrV11 h;
    fram_read(PARAMS_FRAM_ADDR, &h, sizeof h);
    if (h.magic != PARAMS_MAGIC) return false;

    bool ok;
    if (h.version == PARAMS_VERSION) {
        ok = params_load_at(PARAMS_FRAM_ADDR, g);
    } else {
        // The legacy parade, one last generation. The next 'w' writes v11.
        switch (h.version) {
            case 2: ok = params_load_v2(g); break;
            case 3: ok = params_load_v3(g); break;
            case 4: ok = params_load_v4(g); break;
            case 5: ok = params_load_v5(g); break;
            case 6: ok = params_load_v6(g); break;
            case 7: ok = params_load_v7(g); break;
            case 8: ok = params_load_v8(g); break;
            case 9: ok = params_load_v9(g); break;
            case 10: ok = params_load_v10(g); break;
            default: ok = false; break;
        }
    }
    if (!ok) g = ControlGains{};  // never half-interpret
    // Clamped EVEN ON SUCCESS: CRC proves the bytes are the bytes that were
    // written, not that what was written was sane (the ka incident had 18
    // months of valid CRCs). The registry range is the sanity check.
    params_clamp(g);
    return ok;
}

bool params_save(const ControlGains &g) {
    return params_save_at(PARAMS_FRAM_ADDR, g);
}
