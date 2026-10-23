// Persistent tunable parameters (FRAM-backed), and THE PARAMETER REGISTRY.
//
// -- THE RULE (bert, 2026-08-27, TUNING_PLAN.md) ------------------------------
//
// A parameter is either COMPILED or TUNABLE, never both. Anything anyone will
// ever want to move joins the registry below; the compiled defaults in
// ControlGains exist only to seed an empty FRAM. Once a parameter is in the
// registry, editing its compiled default and reflashing is EXPECTED to do
// nothing on a mouse with a saved block -- that is the design, not a bug.
//
// This rule exists because the alternative was lived, four times. The tuned
// kyaw was 10.0 while the compiled default said 2.0 and a whole steering-lag
// model was built on 2.0; the live track was 72.3 against a compiled 59; a
// corrupted ka cost two days as "stiction"; and arc_understeer was added to
// ControlGains without ever being added to the FRAM block or the console's
// set_param -- so it could be tuned in RAM and was silently thrown away at
// every power cycle. Three hand-maintained copies of the parameter list
// (struct, FRAM block, console if-chain) is how that happens. The registry is
// ONE table, and everything else -- console set/get/list, FRAM save/load, the
// flight log's snapshot, min/max clamping, the future GUI -- walks it.
//
// -- THE REGISTRY -------------------------------------------------------------
//
// One row per tunable: console name, pointer-to-member into ControlGains, and
// a hard [min, max]. The clamp is applied at EVERY door -- console set, FRAM
// load, migration -- so a corrupted-but-CRC-valid value can never again reach
// the ISR (the ka incident was 1.5e-4 against a physical ~2.5e-6; its clamp
// now tops out at 2e-5).
//
// APPEND-ONLY. New parameters go at the END of the table, never in the
// middle, never reordered, never deleted (retire a dead one by clamping it to
// its default). The stored block is "values in registry order", so this rule
// is what makes old blocks load into new firmware without a migration
// function per version. test_params pins the order with a golden list.
//
// -- THE STORED BLOCK (v11) ---------------------------------------------------
//
//     u32 magic, u16 version, u16 count, float value[count], u32 crc
//
// `count` is PARAM_COUNT at the time of saving. A newer firmware loading an
// older block reads the first `count` values and leaves the rest at compiled
// defaults; an older firmware refuses a newer block (count too large) and
// boots on defaults rather than misassigning values. Appending a parameter
// therefore needs NO version bump and NO frozen-history struct -- the parade
// of ParamsBlockV2..V10 in Params.cpp ends at v11. The version number now
// only changes if the format itself does.
//
// Blocks v2..v10 (packed structs, one frozen copy each) are still accepted at
// boot so the mouse's saved tuning survives this upgrade; the next 'w' writes
// v11 and the legacy loaders become dead code to delete in some later spring
// clean.
//
// save/load also take an explicit address so the SAME serialization stamps a
// snapshot of the live gains into every flight log (FlightLog.h,
// FLOG_PARAMS_ADDR) -- the log then records what actually flew, which the
// settings banner's seven values only gestured at.

#pragma once

#include <Arduino.h>

#include "Control.h"

constexpr uint32_t PARAMS_FRAM_ADDR = 0x00000;  // see map in Fram.h
constexpr uint32_t PARAMS_MAGIC = 0x54573350;   // "TW3P"
constexpr uint16_t PARAMS_VERSION = 11;  // v11: registry-order value array
// History: v2 added ka; v3 added kx/kyaw; v4 added strim; v5 added track_mm;
// v6 split the turn P gain (kp_turn); v7 added tasym; v8 added a single
// circ_mm (superseded by the per-wheel split in v9); v10 added kslip.
// v5 -> v6 note: v5 predates the kp split, and a block saved after a 't'
// auto-tune holds the TURN-stiff kp -- migration seeds kp_turn from the
// stored kp; if straights buzz, restore the straight kp by hand and 'w'.

// One registry row. `name` is the console name ('kyaw=2' etc), kept short
// because it is typed over a radio link; NUL-terminated.
struct ParamDef {
    const char *name;
    float ControlGains::*field;
    float min;
    float max;
};

// The table (Params.cpp) and its size. PARAM_COUNT is a constant, not a
// sentinel scan, so the coverage tripwire below can see it.
constexpr int PARAM_COUNT = 78;
extern const ParamDef PARAM_REGISTRY[PARAM_COUNT];

// THE COVERAGE TRIPWIRE. ControlGains is all-float by construction; if a
// field is added to the struct without a row being appended to the registry,
// this fails the build instead of the field silently joining the untunable,
// unsaved, unlogged limbo that arc_understeer sat in.
static_assert(sizeof(ControlGains) == PARAM_COUNT * sizeof(float),
              "ControlGains and PARAM_REGISTRY disagree -- append the new "
              "field to the registry (END of the table) and bump PARAM_COUNT");

// Registry lookup by console name. nullptr if unknown.
const ParamDef *param_find(const char *name);

// Clamp every field of `g` into its registry range. Returns how many fields
// were out of range (0 = untouched). Called on every load and every set.
int params_clamp(ControlGains &g);

// Loads the block from FRAM into `g`. Returns true if a valid block was
// found; on false (no FRAM, bad magic/CRC, future format) `g` is left at
// compiled-in defaults. The result is always clamped.
bool params_load(ControlGains &g);

// Writes `g` to FRAM at the params address (no-op if FRAM missing; returns
// fram_ready()).
bool params_save(const ControlGains &g);

// The same v11 serialization at an arbitrary FRAM address -- the flight log
// uses these to stamp each run with the gains that flew it. Needs
// PARAMS_BLOCK_BYTES of room.
constexpr uint32_t PARAMS_BLOCK_BYTES = 8 + PARAM_COUNT * 4 + 4;
bool params_save_at(uint32_t addr, const ControlGains &g);
bool params_load_at(uint32_t addr, ControlGains &g);

// CRC-32 (IEEE, reflected) over an arbitrary buffer -- exposed for tests
// and future FRAM blocks (maze store will want it too).
uint32_t params_crc32(const void *data, size_t len);
