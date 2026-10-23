// THE RUN LADDER (bert, 2026-09-18: "we should set velocity limits per leg
// ... MR2,3,4,5 out at 800, 1000, 1500, 2000; MR2,3,4,5 return all at 400;
// these are peak velocities").
//
// A competition is five runs and no laptop. The search (MR1) flies at `sv`.
// Every route run after it climbs one rung: the OUT leg's peak velocity
// comes from the registry table rv2 / rv3 / rv4 / rv5 by the run's number,
// the HOME leg (unscored -- only start->goal is timed) flies at rvh, and
// ralat stays frozen so the corners are the same arcs on every rung and
// the learned gate timings transfer (RACE_PLAN.md, the five-run doctrine).
// Which run is next is REMEMBERED IN FRAM, not in RAM: the counter resets
// to MR2 whenever a map is saved (a completed search, or a pushed map --
// a new maze is a new ladder), advances by one each time a route run
// launches from the start cell (a crash costs the run, as it does on the
// day), and holds at MR5 after that. Power cycles do not touch it: search,
// power off, wave, power off, wave, and the speed climbs by itself.
//
// `mr` on the console prints the next rung; `mr=N` sets it (a false start
// you do not want to pay for); `mr=0` turns the ladder OFF, and the route
// flies at the hand-set `rv` on every leg as it always did -- the rig
// sessions' mode. The rung and the peaks it chose go into the flight-log
// banner (MR3 v1000/500) and the lap report header.
//
// STORAGE. 12 B at RUN_LADDER_FRAM_ADDR in the free small-block window
// (Fram.h map): magic, version, next rung, CRC. No block = ladder OFF
// (a mouse flashed with this firmware behaves exactly as before until a
// map is saved).

#pragma once

#include <Arduino.h>

#include "Control.h"
#include "Params.h"

constexpr uint32_t RUN_LADDER_FRAM_ADDR = 0x00190;   // see map in Fram.h (0x110 until 2026-09-21)
static_assert(PARAMS_FRAM_ADDR + PARAMS_BLOCK_BYTES <= RUN_LADDER_FRAM_ADDR,
              "the params block has grown onto the run ladder block: move "
              "it (Fram.h map) before adding this param");
constexpr uint32_t RUN_LADDER_MAGIC = 0x4C4E5552;   // "RUNL"
constexpr uint8_t RUN_LADDER_VERSION = 1;
constexpr uint8_t RUN_LADDER_FIRST = 2;   // the first route run is MR2
constexpr uint8_t RUN_LADDER_LAST = 5;    // and everything past MR5 is MR5

// Restore the counter from FRAM (boot, after fram_init).
void run_ladder_init();
// The next run's rung: 2..5, or 0 = ladder off (fly at rv).
uint8_t run_ladder_next();
// A map was saved: the ladder starts again at MR2.
void run_ladder_reset();
// A route run is launching from the start cell: returns its rung (0 = off).
// It does NOT advance the counter (2026-09-30; bert: "the MR ladder shouldn't
// climb unless a good run and a goal reached is successful") -- a run that
// aborts, crashes or is carried off flies the same rung again next time.
uint8_t run_ladder_take();
// The run flown at rung `flown` REACHED THE GOAL (the out leg ended there, or
// the circuit through it completed): the next run climbs one rung (capped at
// RUN_LADDER_LAST). Only if the counter still says `flown` -- an 'mr=N' typed
// in between wins.
void run_ladder_climb(uint8_t flown);
// Console: set the next rung (2..5), or 0 to switch the ladder off.
void run_ladder_set(uint8_t next_mr);
// The peaks a rung flies: out leg from rv2..rv5 (0 = use rv), home from rvh.
float run_ladder_out_v(uint8_t mr, const ControlGains &g);
float run_ladder_home_v(uint8_t mr, const ControlGains &g);
// THE ACCELERATION LADDER (bert, 2026-09-26: "if it is just velocity
// should we also ladder the acceleration, so only MR5 has 5000"). The
// rung's ralat is scaled by this: `mra` at MR2, rising linearly to 1.0 at
// MR5 (mra 0.4: x0.4, x0.6, x0.8, x1.0). mra 1.0 is the old ladder --
// every rung at the registry's corner speeds. 1.0 for a run off the
// ladder (mr 0) and for MR5. Until 2026-10-04 ralon and rbrk were scaled
// too; bert, after Hazlemere: "the long straights should be at max accel,
// not limiting by max V" -- the risk is in the corners, so the ladder is
// the corners': ralon and rbrk are the registry's on every rung and the
// rung's rv is where the straights stop climbing (RouteRun).
// The OUT leg's scale; the home leg flies x1.0 on every rung (RouteRun).
float run_ladder_accel_scale(uint8_t mr, const ControlGains &g);
