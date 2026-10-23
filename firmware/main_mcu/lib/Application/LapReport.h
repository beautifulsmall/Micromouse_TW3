// LAP REPORT -- the mouse does its own corner bookkeeping.
//
// bert, 2026-08-27 (TUNING_PLAN.md): "The lap report should not report live
// data but save it to FRAM and report once a lap." And 2026-08-29
// (CORNER_SET.md): Harrison made hundreds of measurements per corner by
// hand; the lap report makes them automatic, so the GUI scoreboard fills
// itself instead of waiting on offline scripts.
//
// WHAT IS MEASURED, per corner, onboard, with the same definitions the
// offline analysis used (so the numbers are continuous with runs 193-201):
//
//   sweep_err   yaw swept from the corner's entry tangent to its close
//               (exit + SWEEP_JUNCTION_EXIT_MM -- the same boundary the
//               gate walk uses), minus the commanded sweep. Degrees.
//   max|ye|     worst path yaw error inside that window.
//   apex_min    smallest valid OUTER side beam in the window -- the wall
//               the mouse would hit going wide. mm, 255 = never seen.
//   exit_ye     path yaw error at the close.
//   exit_lat    corridor lateral at the close (127 = no estimate).
//   gate        how the corner was anchored (LOCK/corr mm, NO_EDGE,
//               SKIP_LAT, INHERITED, NONE) -- same outcomes the gate
//               stats aggregate, kept per corner here.
//
// LIFE CYCLE. begin() at launch; RouteRun feeds enter/sample/gate/close as
// its own gate walk crosses the same boundaries (no second position
// tracker); finish() at leg end writes the block to FRAM and prints it --
// the mouse is stationary by then, which is exactly bert's rule: nothing
// is transmitted while flying. 'lr' reprints the stored block any time,
// including after a power cycle.
//
// STORAGE. A RING of eight 1 KB slots at the top of the chip (LapReport
// ring, Fram.h map), slot = seq % 8. Was one block, overwritten every
// lap, on the grounds that the GUI fetches between laps -- which is true
// on a rig and false at a competition, where nothing is downloaded
// between MR2 and MR5 and the lap report is exactly the "data needed to
// calculate the next run" (bert, 2026-09-13). Eight covers the five runs
// with room for a false start. 'lr' prints the latest, 'lr,N' the one N
// laps back, 'lr.all' every stored report oldest first (so the GUI
// scoreboard can ingest a whole event after the fact). The seq counter
// continues from the newest stored slot, as it always has.

#pragma once

#include <Arduino.h>

#include "Fram.h"

// The top 8 KB of the chip, whatever size the build is laid out for
// (Fram.h FRAM_MBIT): 0x7E000 on the 4 Mbit part, 0xFE000 on the 8 Mbit.
// Slot k is at LAPREP_FRAM_ADDR + k * LAPREP_FRAM_BYTES; the pre-ring
// single block (0x7FC00 on 4 Mbit) is slot 7 and still reads.
constexpr uint32_t LAPREP_FRAM_ADDR = FRAM_BULK_END;  // see map in Fram.h
constexpr uint32_t LAPREP_FRAM_BYTES = 1024;
constexpr int LAPREP_RING_N = 8;
static_assert(LAPREP_RING_N * LAPREP_FRAM_BYTES == FRAM_TOP_BLOCKS_BYTES,
              "the lap report ring must fill the top blocks exactly");
constexpr uint32_t LAPREP_MAGIC = 0x54573352;   // "TW3R"
// v3: sweep_err widened to int16. v4 (2026-09-05): the SLIP INSTRUMENT --
// per corner, the distance dead-reckoned since the previous anchor, the
// speed gained and shed over it, and what the slip model already took off
// -- and the block moved to its own 1 KB at the top of FRAM with the corner
// cap raised to 32 (a 16x16 speed run has 20-30 corners; 14 was the rigs).
// v5 (2026-09-18): the RUN LADDER -- the header carries the run number
// (MR2..MR5, 0 = ladder off) and the peak velocity the leg was compiled
// for, so a stored report says which rung it was without a diary. 24 B
// header; v4 blocks (20 B) still read, with mr 0 / v 0.
constexpr uint8_t LAPREP_VERSION = 5;
constexpr int LAPREP_MAX_CORNERS = 32;

enum LapGate : uint8_t {
    LR_GATE_NONE = 0,       // never armed (no corner window reached?)
    LR_GATE_LOCK = 1,       // post found, correction applied
    LR_GATE_NO_EDGE = 2,    // armed, watched, saw nothing
    LR_GATE_SKIP_LAT = 3,   // edge seen but mouse too far off-centre
    LR_GATE_INHERITED = 4,  // too close behind the previous corner to arm
    LR_GATE_FRONT = 5,      // anchored on the FRONT wall's range (fgate)
    LR_GATE_FUSED = 6,      // front anchored, side edge confirmed/refined
    LR_GATE_DIAG = 7,       // one end of a diagonal: observed, never steered
                            // (corr = what the side edge measured on a
                            // 45-in, 0 on a 45-out). Stage one, Route.h.
};

struct LapCorner {          // 14 bytes, packed scaling documented per field
    uint8_t cx, cy;         // cell
    uint8_t r_mm;           // commanded radius
    int8_t sweep_d3;        // commanded sweep / 3: 45, 90 and 180 all exact
    // Measured - commanded sweep, 0.1 deg, sampled where the ARC ENDS (see
    // corner_exit_yaw in RouteRun.cpp). int8 (+-12.7) was enough for a corner
    // entered off a straight -- CAL-6 never left +-3 -- but the D6 zigzag pins
    // it on every corner after the first: with 20 mm between arcs the omega
    // slew rail cannot keep up and the mouse is running late by more than the
    // clamp could say. A field that saturates exactly where the interesting
    // physics lives is not a measurement, so: int16.
    int16_t sweep_err_x10;
    uint8_t max_ye_x10;     // 0.1 deg, clamped 25.5
    uint8_t apex_min_mm;    // outer beam min, 255 = never valid
    int8_t exit_ye_x10;     // 0.1 deg
    int8_t exit_lat_mm;     // 127 = no estimate
    uint8_t gate;           // LapGate
    int8_t gate_corr_x2;    // 0.5 mm, clamped +-63.5
    // The lateral the GATE saw at the last wall-present sample of the
    // APPROACH (WallGate last_lat_mm) -- not the exit lateral in exit_lat.
    // This is the x-axis for measuring the beam-lead slope k (gklat) from
    // flight data: corr vs glat across laps. 127 = not gated.
    int8_t gate_lat_mm;
    // What the FRONT wall said about this corner, mm, 0.5 resolution
    // (127 = the front gate never produced a measurement here). Recorded
    // INDEPENDENTLY of how the corner was actually gated, so a lap carries
    // the front reading and the side reading for the SAME corner -- which
    // is the pairing a per-corner front trim has to be fitted from, and
    // the thing observe mode existed to collect (bert, 2026-08-30:
    // "didn't see any new data columns in log files").
    int8_t front_corr_x2;
    // THE SLIP INSTRUMENT (v4). Distance dead-reckoned from the previous
    // anchor to this corner's gate, mm ("dr" in the gate print) -- the
    // baseline the correction accumulated over. 0 = this corner's gate did
    // not anchor (INHERIT / NO_EDGE / a 45). Divided into corr it is the
    // slip FRACTION over the segment; against dv_acc / dv_brk it separates
    // drive-slip (wheels over-read while accelerating) from brake-slide
    // (under-read while braking): fit sacc / sbrk offline from these three
    // columns across a session, then set them in the registry. Learned
    // weights, deterministic flight (TUNING_PLAN.md).
    int16_t ds_mm;
    uint8_t dv_acc_8;       // speed GAINED since the anchor, mm/s / 8 (cap 2040)
    uint8_t dv_brk_8;       // speed SHED since the anchor, mm/s / 8
    // What the slip model (sacc/sbrk) had already added to s over this
    // segment when the gate fired, 0.5 mm. With the model fitted right the
    // corr column shrinks toward the noise floor and this one carries what
    // the corr used to.
    int16_t slip_x2;
};
static_assert(sizeof(LapCorner) == 20, "LapCorner must stay 20 B");

struct LapHeader {          // 20 bytes incl CRC over header+corners
    uint32_t magic;
    uint8_t version;
    uint8_t n;              // corners recorded
    uint8_t dir;            // 'n'/'e'/'a' as flown
    uint8_t dutypk_x100;    // duty peak, clamped 2.55
    uint16_t seq;           // lap counter, survives power cycles
    uint16_t lap_ms_x2;     // lap time / 2 ms (131 s max)
    uint16_t len_mm;
    int8_t end_ye_x10;      // yaw residual vs the parked cardinal
    uint8_t vbat_x50;       // battery at lap end, mV/50 (8.4 V = 168);
                            // 0 = not sampled (pre-vbat firmware blocks)
    uint8_t mr;             // v5: run number MR2..MR5, 0 = ladder off
    uint8_t v_peak_x10;     // v5: peak velocity / 10 mm/s (2550 max)
    uint8_t spare[2];
    uint32_t crc;           // over everything above + all corner records
};
static_assert(sizeof(LapHeader) == 24, "LapHeader must stay 24 B");
// The v4 layout, for reading blocks stored before 2026-09-18.
struct LapHeaderV4 {
    uint32_t magic; uint8_t version, n, dir, dutypk_x100;
    uint16_t seq, lap_ms_x2, len_mm; int8_t end_ye_x10; uint8_t vbat_x50;
    uint32_t crc;
};
static_assert(sizeof(LapHeaderV4) == 20, "v4 header is 20 B");
static_assert(sizeof(LapHeader) + LAPREP_MAX_CORNERS * sizeof(LapCorner)
                  <= LAPREP_FRAM_BYTES,
              "lap report outgrew its 1 KB block at the top of FRAM");

// At launch. Resets the working state; n_corners > cap is clamped.
void lap_report_begin(char dir);

// Corner k's window opened (entry tangent crossed). Records the entry yaw
// and the corner's identity.
void lap_report_corner_enter(int k, uint8_t cx, uint8_t cy, uint8_t r_mm,
                             int sweep_deg, float yaw_now);

// One sensor-packet sample inside corner k's window.
void lap_report_corner_sample(int k, float yaw_err_deg, uint8_t outer_beam);

// The gate resolved corner k (LOCK carries the correction; glat_mm is the
// approach lateral the gate judged it at -- WallGate last_lat_mm).
void lap_report_gate(int k, LapGate g, float corr_mm, float glat_mm);

// The FRONT wall's measurement for corner k, whether or not it steered.
void lap_report_front(int k, float corr_mm);
// The slip instrument for corner k, taken as its gate ANCHORS: distance
// dead-reckoned since the previous anchor, speed gained / shed over it,
// and the slip model's contribution over the same segment (see LapCorner).
void lap_report_segment(int k, float ds_mm, float dv_acc_mm_s,
                        float dv_brk_mm_s, float slip_mm);

// Corner k closed (past exit + junction margin, or leg end).
void lap_report_corner_close(int k, float yaw_now, float exit_ye_deg,
                             bool lat_valid, float lat_mm);

// Battery at lap end, mV (call just before finish; 0 = leave unsampled).
// Kept out of finish()'s signature so the host tests need no vbat mock.
void lap_report_set_vbat(uint16_t mv);
// The run ladder's rung (0 = off) and the peak this leg was compiled for
// (RunLadder.h) -- call after begin().
void lap_report_set_run(uint8_t mr, float v_peak_mm_s);

// Leg ABORTED (bert, 2026-08-30: "when the mouse aborts it should then send
// the lap report, these are missing for unfinished laps"). Stamps what was
// recorded up to the abort, writes FRAM, prints -- the mouse has just been
// braked to a stop, so the stationary rule holds. The header's dir byte
// carries bit7 as the abort mark (ASCII dirs never set it; old stored
// blocks reprint unchanged), and the printed header gains ",abort=1" so
// the GUI can keep crash laps out of the scoreboard statistics while the
// corner rows still say exactly how far the lap got and what each gate saw.
void lap_report_abort(uint32_t lap_ms, float len_mm, float duty_peak);

// Leg complete: stamp the header, write FRAM, print the report (the mouse
// is stationary -- this is the "transmit at stop").
void lap_report_finish(uint32_t lap_ms, float len_mm, float duty_peak,
                       float end_ye_deg);

// Print a stored report ('lr'): back = 0 the latest, 1 the one before,
// ... up to LAPREP_RING_N - 1. Says so if that slot holds nothing valid.
void lap_report_print(int back = 0);
// Every stored report, oldest first ('lr.all'); returns how many printed.
int lap_report_print_all();
// The FRAM address of the slot the latest report occupies (0 = none) --
// diagnostics and the host test.
uint32_t lap_report_latest_addr();
