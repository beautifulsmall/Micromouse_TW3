// PER-CORNER GATE OFFSET TABLE -- the map the mouse learns about its maze.
//
// bert, 2026-08-29: "on the speed run the mouse doesnt need to know where it
// is, just counts walls and gaps and turns round posts... a per corner Goff
// table the mouse holds alongside the map." The competition shape: run 1
// searches, run 2 flies the speed-run line at moderate speed and COLLECTS
// this table from its own gate corrections, later runs spend it at speed.
//
// WHY PER CORNER. The 2026-08-29 goffr=0 baseline (laps 101-106) put every
// generic R90 at ~0 mm while (5,5) sat at -28 and (5,0) at -40/-28 -- in
// BOTH travel directions, sigma ~1.5 mm across laps. That is not a corner
// TYPE (the registry's goffr/goffl/goffur/gofful cover those); it is the
// hand-built rig's actual walls. A table keyed by (cell, approach heading)
// absorbs exactly that layer. Approach heading matters because the same
// cell entered from opposite directions gates on two different physical
// edges -- (5,0) measured -39.5 down the east column and -28.5 along the
// south row.
//
// LAYERING. gate_expect_s = geometry + goff (global) + goff<type> + THIS.
// The registry offsets stay the hand-tuned per-type layer; the table is the
// learned per-corner residual on top. Zeroing the table returns the mouse
// to the registry behaviour.
//
// LEARNING. At leg end (mouse stationary -- same rule as the lap report),
// every corner that LOCKED folds its residual correction in:
//
//     g += glearn * (-corr)          clamped to +/-GOFF_TABLE_MAX_MM
//
// corr is the gate's applied correction (err = expect - drop): a NEGATIVE
// corr means the edge came LATER than expected, so the expectation must
// GROW -- hence the sign flip. With the table applied, the next lap's corr
// IS the residual, so this is an integrator; glearn (registry, default 0.5)
// is its gain. glearn=0 freezes the table for race day. NO_EDGE, SKIP_LAT
// and INHERITED corners never learn -- no drop, no evidence.
//
// STORAGE. Sparse block at GOFF_TABLE_FRAM_ADDR (see the map in Fram.h):
// 8 B header + 4 B/entry {cx, cy, entry_h, off_mm} + CRC. 57-entry cap --
// CAL-6 both directions is 16, a full-maze speed route is well under. A
// full table refuses new corners loudly rather than evicting one silently.
// Saved once per leg end, loaded at boot alongside the maze.

#pragma once

#include <Arduino.h>

constexpr uint32_t GOFF_TABLE_FRAM_ADDR = 0x00710;  // see map in Fram.h
constexpr uint32_t GOFF_TABLE_FRAM_END = 0x00800;   // exclusive
constexpr uint32_t GOFF_TABLE_MAGIC = 0x54573347;   // "TW3G"
// v2 (2026-09-01): each entry carries a hand-set TRIM alongside the learned
// offset -- see goff_table_trim_mm below. Old tables load empty.
constexpr uint8_t GOFF_TABLE_VERSION = 2;
// GEOMETRY REVISION -- the auto-staleness guard (bert, 2026-08-30: "should
// I be doing a goff clear after each flash?"). No: entries measure the
// PHYSICAL rig against the gate's expectation formula, so they survive any
// reflash that leaves that formula alone. What silently invalidates them is
// a CODE change to the formula itself -- BEAM_AHEAD, the tangent math, the
// expectation composition in gate_expect_s. BUMP THIS NUMBER in the same
// commit as any such change: a stored table with a different rev declares
// itself stale at boot and starts empty, loudly, instead of steering the
// mouse with offsets measured against geometry that no longer exists.
// (Parameter-level changes -- rv, r90, rur -- do NOT need a bump: the
// 10 mm/lap slew re-converges the table in 2-3 laps, and the GUI's "Clear
// corner table" button covers the deliberate fresh start.)
// rev 2 (2026-08-30): WALL_GATE_BEAM_AHEAD_MM 45 -> 58. Every stored
// entry was fitted against the old expectation and is 13 mm stale.
constexpr uint16_t GOFF_TABLE_GEOM_REV = 2;
constexpr int GOFF_TABLE_MAX =
    (int)((GOFF_TABLE_FRAM_END - GOFF_TABLE_FRAM_ADDR - 8 - 4) / 5);  // 45
constexpr float GOFF_TABLE_MAX_MM = 60.0f;  // entry clamp, +/-
// Per-lap slew for an entry that already EXISTS: one decoy lock (the
// (2,5)+21.5-poisons-(5,5) pattern) or one sagging-battery lap may drag a
// converged corner at most this far, while a NEW corner still takes the
// full alpha step so the first lap fixes most of the offset.
constexpr float GOFF_TABLE_STEP_MM = 10.0f;

// Boot: restore the stored table (bad magic/CRC -> empty, says so).
void goff_table_load();

// The learned offset for a corner approached on maze-frame heading h
// (0-3). 0.0 for a corner the table has never seen.
float goff_table_mm(uint8_t cx, uint8_t cy, uint8_t h);

// Fold one LOCKED corner's applied correction in: g += alpha * (-corr_mm),
// creating the entry if new. False (and a console line) if the table is
// full and the corner is unknown. Does NOT save -- call goff_table_save()
// once per leg, after the last learn, with the mouse stationary.
bool goff_table_learn(uint8_t cx, uint8_t cy, uint8_t h, float corr_mm,
                      float alpha);

// Overwrite one entry by hand ('goffc.set') -- for a corner that cannot
// learn because it never drops (a NO_EDGE case may gate once its window
// finally covers the real edge). Same clamp, same full-table refusal.
bool goff_table_set(uint8_t cx, uint8_t cy, uint8_t h, float off_mm);

void goff_table_save();
// Forget the LEARNED offsets and save. Hand-set trims survive: they are a
// property of the rig the human measured, not of the run that learned the
// offsets, and the usual reason to clear is a geometry or firmware change
// that invalidates the learned layer only. goff_table_clear_all drops both.
void goff_table_clear();
void goff_table_clear_all();

// ---- THE PER-CORNER TRIM (bert, 2026-09-01: "the first right needs
// starting earlier") -----------------------------------------------------
//
// The learned offset above cannot hold an exit position: the learner drives
// each corner's corr to zero, so every corner ends up with whatever
// ALONG-TRACK offset lands its own post on expectation -- and that is not
// the offset that centres its exit. Measured on D6 (laps 297-304): (5,5)
// begins its arc ~0 mm after the inside beam loses the wall and exits
// centred; (0,5), same radius, same speed, begins 32 mm after and exits 35
// mm outside. Exit lateral tracks arc timing 1:1. atrim is the lever that
// moves an arc without the learner cancelling it -- it acts on the APPLIED
// correction, not the expectation -- but it is one number for every 90.
// This is atrim per corner: added to atrim/utrim for that (cell, heading),
// same sign (+ = arc later = exit further outside), never learned, set by
// hand ('goffc.trim'), kept across goffc.clear.
float goff_table_trim_mm(uint8_t cx, uint8_t cy, uint8_t h);
bool goff_table_set_trim(uint8_t cx, uint8_t cy, uint8_t h, float trim_mm);

// Console: one line per entry, GOFFC,cell=x:y,h=N|E|S|W,off=+n.n -- key=value
// like the lap report, so the GUI can adopt it later without a new parser.
void goff_table_print();

int goff_table_count();
