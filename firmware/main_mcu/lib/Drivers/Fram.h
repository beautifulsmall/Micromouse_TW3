// FRAM driver (MB85RS4MTY, 512 KB SPI FeRAM on CS1).
//
// Thin layer over the HAL: verifies the device ID at init and exposes
// typed read/write. FeRAM has no write latency, no erase pages, and 10^13
// write endurance, so callers can treat it like battery-backed SRAM --
// no wear-levelling or commit ceremony needed.
//
// Current map (keep this comment as the allocation registry):
//   0x00000 - 0x0017F   tunable parameter block (Params.h): 8 B header +
//                       PARAM_COUNT x 4 B + CRC. 384 B = room for 93
//                       params. THE BLOCK GROWS: the registry is
//                       append-only, and on 2026-09-18 param #62 took it
//                       to 260 B -- 4 B past the 256 it had, onto the goal
//                       block's magic at 0x100. Every 'w' then killed the
//                       goal and every 'goal=' killed the params CRC
//                       (2026-09-21: a mouse flying on compiled defaults,
//                       then a search re-run to the 16x16 centre). The
//                       static_asserts in Search.h and RunLadder.h now
//                       refuse to compile a registry that reaches them.
//   0x00180 - 0x0018F   search goal block (Search.h / Search.cpp).
//                       Moved from 0x100 on 2026-09-21 -- re-enter 'goal='
//                       once after flashing.
//   0x00190 - 0x0019B   run ladder block (RunLadder.h): magic, next run
//                       number MR2..MR5 (0 = ladder off), CRC. Reset by
//                       every map save, advanced when a route run reaches the goal (2026-09-30).
//                       Moved from 0x110 on 2026-09-21 -- the ladder is OFF
//                       until the next map save (or 'mr=2').
//   0x0019C - 0x001EF   free.
//   0x001F0 - 0x001FF   free (small blocks; 0x1F0 is the size probe's byte).
//   0x00200 - 0x0060F   sensor LUT block: 8 B header + 4x256 B LUTs
//                       (SensorLut.h; uploaded to the sub-MCU at boot)
//   0x00610 - 0x0070F   searched maze map: 8 B header + Maze::State (236 B)
//                       + CRC = 248 B of the 256 available. Restored at
//                       boot, so a power cycle keeps what the last search
//                       discovered.
//   0x00710 - 0x007FF   per-corner learned gate offset table (GoffTable.h):
//                       8 B header + up to 57 x 4 B {cell, approach
//                       heading, mm} + CRC. The mouse's own corrections to
//                       the grid, learned at moderate speed and spent at
//                       race speed -- it lives beside the maze because it
//                       IS part of the map (bert, 2026-08-29).
//
//                       The WALLS are stored, not the SOLUTION: the flood
//                       distance field is derived, and load_state() clears
//                       it rather than restoring a stale one (a stale flood
//                       is a quiet way to walk a route the current goal
//                       never asked for -- and the goal lives in its own
//                       block above, so it can have moved since the save).
//   0x00800 - 0x7FBFF   BULK LOG REGION, 509 KB, shared by two writers that
//                       are never wanted at once (both end at FRAM_BULK_END):
//                         - flight recorder (FlightLog.h): 16 B header +
//                           30 B records, for a speed run
//                         - loop sweep (LoopSweep.h): 256 B header, then
//                           64 KB of 32 B per-corner summaries that never
//                           wrap, then 446 KB of 16 B traces that ring
//                       Each writes its own magic and each REFUSES to write
//                       when the other owns the region, so a search abort
//                       cannot scribble an outcome string over an
//                       experiment. Dump one before starting the other.
//   0x7E000 - 0x7FFFF   LAP REPORT RING (LapReport.h), 8 x 1 KB: 20 B
//                       header + up to 32 x 20 B per-corner records per
//                       slot, slot = seq % 8. Was one block at 0x7FC00
//                       (now slot 7, so the last report before this layout
//                       still reads). Carved off the TOP of the bulk region
//                       2026-09-05 (1 KB) and widened to eight 2026-09-14
//                       so a competition's MR2..MR5 reports all survive
//                       with no laptop between runs -- the lap report IS
//                       the "data needed to calculate the next run"
//                       (RACE_PLAN.md). Costs the flight log 179 records.

#pragma once

#include <Arduino.h>

// THE PART (bert, 2026-09-13): an 8 Mbit (1M x 8) Infineon SPI FRAM is
// on order to replace the 4 Mbit Ramxeed/Fujitsu MB85RS4MTY -- same
// opcodes, same 3-byte addressing, twice the space, so the flight log can
// hold a whole event (RACE_PLAN.md, "FRAM and the flight log"). The SIZE
// is a build-time choice because the map above is laid out from it (the
// lap report block sits 1 KB below the top, the bulk log ends there):
// build with -DFRAM_MBIT=8 once the new chip is fitted. fram_init()
// checks the chip is at least that big (a 4 Mbit part ignores A19 and
// aliases the upper half onto the lower -- the probe catches that) and
// refuses to run on a chip smaller than the build expects, rather than
// let two blocks land on the same bytes.
#ifndef FRAM_MBIT
#define FRAM_MBIT 4
#endif
constexpr uint32_t FRAM_SIZE_BYTES = (uint32_t)FRAM_MBIT * 128UL * 1024;
// Where the bulk log writers stop and the small blocks at the top begin:
// the lap report ring, 8 x 1 KB (LapReport.h).
constexpr uint32_t FRAM_TOP_BLOCKS_BYTES = 8 * 1024;
constexpr uint32_t FRAM_BULK_END = FRAM_SIZE_BYTES - FRAM_TOP_BLOCKS_BYTES;   // 0x7E000 at 4 Mbit
constexpr uint32_t FRAM_DEVICE_ID = 0x047F490B;  // RDID per datasheet (Fujitsu/Ramxeed)
constexpr uint32_t FRAM_DEVICE_ID_JEDEC_CONT = 0x7F7F7F7F;  // Infineon/Cypress: continuation codes first
// The migration region: everything the mouse keeps between power cycles
// that is not a log -- params, goal, sensor LUTs, the searched map, the
// goff table (the small blocks 0x000-0x7FF above). 'fram.dump' prints it,
// 'fram.stage' + 'fram.commit' write it back, so a chip swap carries the
// calibration and the tuning across (Bench.cpp). The lap report and the
// flight log are NOT migrated: dump them ('lr', 'y') before the swap.
constexpr uint32_t FRAM_CFG_BYTES = 0x800;

// Reads the device ID; returns false (and leaves fram_ready() false) if it
// doesn't match -- wiring fault or missing chip. Call after hal_spi_init().
bool fram_init();
bool fram_ready();
// What fram_init found: the RDID bytes as text, the chip size the probe
// measured (512 KB or 1 MB; 0 if no chip answered), and why it is not
// ready when it is not.
const char *fram_id_text();
uint32_t fram_size_detected();
const char *fram_status_text();

// Plain byte access. No-ops when fram_ready() is false (reads zero-fill),
// so callers degrade gracefully on a board without the chip populated.
void fram_read(uint32_t addr, void *buf, size_t len);
void fram_write(uint32_t addr, const void *buf, size_t len);
