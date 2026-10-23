// FRAM flight recorder (bert's design, 2026-07-29): every sensor packet
// of the SPEED RUN is written to FeRAM as a 20-byte record -- full-rate
// wall + IMU + estimator truth, surviving any abort, dumped to the host
// on demand. FeRAM (MB85RS4MTY, 512 KB, no write latency, 10^13
// endurance) makes per-loop logging free; only the sprint is captured.
//
// FRAM map: records at FLOG_BASE.., ring-buffered (a 42 s capacity ring
// never wraps on a sprint, but the ring makes long experiments safe).
//
// Usage: flog_arm() at path launch; flog_record() once per fresh sensor
// packet in S_PATH; flog_stop() at completion/abort (idempotent);
// 'y' on the console dumps CSV (oldest -> newest) over both links.

#pragma once

#include <Arduino.h>
#include "Fram.h"

// SHARED REGION. The loop sweep (LoopSweep.h) uses the same bytes; the two
// are never wanted at once. flog_owns_region() says whether what is in FRAM
// right now is a flight log, and every write in this module is gated on it.
constexpr uint32_t FLOG_BASE = 0x00800;      // after params + goal + LUT blocks
// Persistent header (bert's housekeeping, 2026-08-01): the records were
// always in FeRAM; the COUNT lived in RAM and died at power-off. The
// header persists it -- refreshed every 16 records in flight, finalized
// at stop -- so 'y' retrieves the previous sprint after a power cycle,
// even one that ended with the battery pulled mid-crash.
constexpr uint32_t FLOG_HDR_BYTES = 16;
// Wall-map preamble (bert, 2026-08-03): the exact searched map, stored at
// speed-run start, so offline tools use ground truth instead of inferring
// walls from the noisy beam cloud. 17 vertical + 17 horizontal uint16 = 68
// bytes; 96 reserved.
constexpr uint32_t FLOG_WALL_ADDR = FLOG_BASE + FLOG_HDR_BYTES;
constexpr uint32_t FLOG_WALL_BYTES = 96;
// Settings banner, PERSISTED (2026-08-11). It first lived in a RAM string and
// never once appeared in a dump: the mouse runs untethered on the radio, so
// power cycles between the sprint and 'y' are normal -- which is the whole
// reason the count header is persisted in the first place. The records
// survived; the banner did not. Anything that has to survive to the dump has
// to live in FeRAM, same rule as the count and the wall map.
constexpr uint32_t FLOG_BANNER_ADDR = FLOG_WALL_ADDR + FLOG_WALL_BYTES;
// Two SEPARATE fields, not one shared buffer. They competed when they shared:
// the settings line is ~130 chars, each outcome append ~40, and log 078 lost
// the tail of "front panic on speed path" because the observer had already
// appended. Waiting longer before power-down does nothing -- FeRAM commits
// immediately; this was always snprintf running out of room, not time.
// The outcome is the more important field, so it gets its own space.
constexpr uint32_t FLOG_BANNER_BYTES = 160;   // settings, written at arm
constexpr uint32_t FLOG_OUTCOME_ADDR = FLOG_BANNER_ADDR + FLOG_BANNER_BYTES;
constexpr uint32_t FLOG_OUTCOME_BYTES = 96;   // appended during the run
// The short horizon gets its OWN field, for the same reason the outcome got
// one: at 96 bytes the outcome is already ~75 full after an abort reason and
// the side-sum line, and flog_set_outcome deliberately keeps the FIRST reason
// when it fills. A horizon report appended there would be dropped silently on
// exactly the runs that ended badly -- the runs worth reading. Written once at
// the end of a run (see flog_set_horizon).
constexpr uint32_t FLOG_HZ_ADDR = FLOG_OUTCOME_ADDR + FLOG_OUTCOME_BYTES;
constexpr uint32_t FLOG_HZ_BYTES = 128;
// FULL PARAMETER SNAPSHOT (2026-08-27), stamped at flog_arm from the live
// gains, printed by the dump as PARAM,name=value lines. The settings banner
// carried seven hand-picked values and the other ten stayed invisible --
// which is how a steering-lag analysis got built on a compiled kyaw of 2.0
// while the mouse flew 10.0. The snapshot is the registry serialization
// (Params.h, params_save_at), so a parameter added to the registry appears
// in every subsequent log with no change here. 160 reserved against the
// current 112 (25 params) so appended params do not shift the record base
// every time.
//
// NOTE: adding this block SHIFTED FLOG_REC_BASE. A log recorded by firmware
// older than 2026-08-27 will not dump correctly on this firmware (records
// begin 96 bytes earlier than we now look): dump before reflashing, not
// after.
// 2026-08-30: 160 -> 224 when param 39 (utrim) overflowed 160 (8 + 39x4 +
// 4 = 168). Headroom to 53 params this time, same rule: dump any stored
// log BEFORE reflashing across this line -- FLOG_REC_BASE moved again.
// 2026-09-11: 224 -> 320 when param 54 (r135) overflowed 224 (8 + 54x4 +
// 4 = 228). Headroom to 77 params. Same rule again: DUMP ANY STORED FLIGHT
// LOG BEFORE REFLASHING -- FLOG_REC_BASE moved by 96 bytes.
constexpr uint32_t FLOG_PARAMS_ADDR = FLOG_HZ_ADDR + FLOG_HZ_BYTES;
// 2026-10-04 (race morning): 320 -> 416 when param 78 (ghome) overflowed
// 320 (8 + 78x4 + 4 = 324). Headroom to 101 params. SAME RULE: dump any
// stored flight log BEFORE reflashing -- FLOG_REC_BASE moved by 96 bytes.
constexpr uint32_t FLOG_PARAMS_BYTES = 416;
constexpr uint32_t FLOG_REC_BASE = FLOG_PARAMS_ADDR + FLOG_PARAMS_BYTES;
constexpr uint32_t FLOG_REC_BYTES = 40;
constexpr uint32_t FLOG_CAPACITY =
    (FRAM_BULK_END - FLOG_REC_BASE) / FLOG_REC_BYTES;  // ~13k rec / ~22 s

// One record. Packed layout, 20 bytes -- keep in sync with flog_dump().
struct FlogRec {
    uint16_t t_ms;       // since flog_arm (wraps at 65 s -- fine)
    int16_t s_mm;        // believed path position
    int16_t v_mm_s;      // commanded velocity
    int16_t yaw_cdeg;    // fusion yaw * 10
    int16_t yaw_err_cdeg;  // path yaw error * 10
    int16_t ay_mm_s2;    // body-Y accel, gravity-compensated
    int16_t ax_mm_s2;    // body-X accel (filtered fwd)
    int8_t slide_mm;     // per-corner slide integral (clamped +/-127)
    uint8_t flags;       // bit0 in_arc, bit1 impact latched, bit2 a
                         // correction step applied this packet, bit3
                         // lateral_valid
    uint8_t left_mm;     // raw beam bytes (255 invalid, 0 IC error)
    uint8_t front_left_mm;
    uint8_t front_right_mm;
    uint8_t right_mm;
    // Drift discriminators (2026-07-30): wheel-vs-fused separates
    // estimator artifacts from real path length; measured omega gives
    // curvature truth (omega/v = real kappa vs planned).
    int16_t wheel_fused_mm;  // (wheel odometer - fused s), cumulative
    int16_t omega_dps;       // measured yaw rate
    // Beam observer, SHADOW mode (2026-08-04): proposed corrections from
    // raycasting the known map vs the beams -- LOGGED, not applied. Offline
    // we compare obs_along to the gates' actual s corrections.
    int16_t obs_along_mm;    // along-track position error estimate
    int16_t obs_cross_mm;    // cross-track (lateral) error estimate
    uint8_t obs_n;           // beams that matched a wall this packet
    // Collision clearance (2026-07-30): predicted body-EDGE room from the two
    // side beams (Walls.h walls_clearance_mm) -- pose-free and map-free, so an
    // accumulated position error cannot fool it. Negative = the flank sweeps
    // through the detected point. Clamped to +/-127; +127 also means "nothing
    // relevant in view". Took over the spare pad byte, so the record stays
    // 30 bytes and the FRAM layout is unchanged.
    int8_t clear_mm;
    // ---- PREDICTED WALL VECTOR (bert 2026-08-03) ------------------------
    // What the map says each side beam should be looking at, from the raycast
    // observe() already performs. Logging the PREDICTION alongside the raw
    // beam byte is what makes the observer debuggable offline: the viewer can
    // draw the wall the mouse believed it was measuring, next to the hit it
    // actually got, and a mis-association becomes visible instead of being
    // buried in a residual.
    //
    // axis: 0 = vertical wall (x-normal), 1 = horizontal (y-normal),
    //       255 = no wall predicted for that beam this packet.
    // coord: the wall FACE coordinate in mm (x for vertical, y for horizontal).
    //       Confirmed on log 068: values land on 180k +/- 6, i.e. the face, not
    //       the grid line -- walls are 12 mm thick and the beam stops on the
    //       near surface. With axis this IS the wall vector, since maze walls
    //       are axis-aligned on the 180 mm grid. The viewer must draw it 6 mm
    //       inside the grey map line, not on it.
    // rng:  predicted along-beam range, so observed-minus-predicted is
    //       recoverable without re-running the raycast offline.
    uint8_t pred_axis_l;
    uint8_t pred_axis_r;
    int16_t pred_coord_l;
    int16_t pred_coord_r;
    uint8_t pred_rng_l;      // mm, 255 = none
    uint8_t pred_rng_r;
    // Accumulated heading correction from the pose observer, deg * 100.
    // Log 068 exposed the gap this fills: obs_along/obs_cross carry the OLD
    // shadow decomposition, and the SOLVED correction went straight to Control
    // without ever being recorded -- so there was no way to check the firmware
    // observer against the offline replay of the same run. With the observer
    // off this stays 0.
    int16_t obs_head_cdeg;
};
static_assert(sizeof(FlogRec) == 40, "flight record must stay 40 bytes");

// Restore the previous run's header from FRAM (call once at boot, after
// fram_init). Leaves the recorder idle with the old count readable.
void flog_init();
// True if the bulk FRAM region currently holds a flight log rather than a
// sweep log. Writes are no-ops when false; flog_arm() takes ownership.
bool flog_owns_region();

// Store the known wall map (17 v + 17 h uint16) for the dump preamble.
// Call once at speed-run start, before flog_arm.
void flog_set_walls(const uint16_t *v, const uint16_t *h, int n);

void flog_arm();
void flog_stop();
// Continue the SAME log after a flog_stop: records append, t_ms keeps its
// origin, the banner and the params block stand. The two-leg route uses
// this so S->G, the stop and turn at the goal, and G->S read as ONE run in
// the viewer (the in-place turn is the gap between the legs; the yaw frame
// is continuous across it because nothing re-zeroes the fusion between
// legs).
void flog_resume();
bool flog_active();
void flog_record(const FlogRec &r);
uint32_t flog_count();

// THE CAPACITY LADDER (2026-09-14, "fit a single flight log of a 16x16
// into the 4 Mbit part"). The region is a ring: a run longer than it
// overwrites its own LAUNCH, the part every investigation starts from. So
// both missions decimate by DISTANCE (one record per few mm of path, plus
// every event, plus one per 100 ms stationary) and scale that spacing by
// how full the region already is: x1 below half, x2 from half, x4 from
// three quarters. A run that would have wrapped instead loses resolution
// over its tail; the launch and everything logged so far stand. The
// search, which cannot know its length in advance, uses this ladder (an
// explore-all can run 32 m at 5 mm, 32 more at 10, 64 more at 20 -- past
// the 108 m EX_MAX_STEPS bound). The route sizes its spacing from the
// compiled leg instead (RouteRun.cpp, RR_LOG_DS_MIN_MM) and uses only the
// fill fraction, quadrupling past 95 %.
constexpr float FLOG_LADDER_HALF = 0.50f;
constexpr float FLOG_LADDER_THREEQ = 0.75f;
inline float flog_fill() { return (float)flog_count() / (float)FLOG_CAPACITY; }
inline float flog_ds_scale() {
    float f = flog_fill();
    return f >= FLOG_LADDER_THREEQ ? 4.0f : f >= FLOG_LADDER_HALF ? 2.0f : 1.0f;
}

// Dump CSV oldest -> newest through the provided line printer. Blocking
// (call from the console handler, not mid-mission).
void flog_dump(void (*println_fn)(const char *));

// Settings banner (bert 2026-08-11): the one-line record of HOW the run was
// configured, captured at flog_arm and replayed at the top of every dump.
// Without it a log is unreadable a week later -- 069 and 070 differ only in
// firmware settings, and working out which was which cost real time. Captured
// at ARM, not at dump, so a runtime change made after the crash cannot rewrite
// history. Search fills it in; the dump just prints it.
void flog_set_banner(const char *text);

// Append the run's OUTCOME to the persisted banner -- the abort reason, or
// "completed". Runs 073/074/075 all ended by search_abort() mid-corner and the
// reason went only to the console, which is not what gets pasted back: three
// logs, three unexplained stops, and 21 possible search_abort() call sites to
// guess between. The reason belongs with the data it explains.
void flog_set_outcome(const char *why);

// The short-horizon summary for this run, dumped as "HORIZON,...". Overwrites
// rather than appends: there is exactly one per run and the last word is the
// whole run's tally.
void flog_set_horizon(const char *text);
