// Bench: the line-based console UI and every calibration / self-test
// harness behind it.
//
// THIS IS NOT FLIGHT CODE. Nothing here runs during a mission -- the search
// executor (Search.h) owns the mouse once a run starts, and every harness in
// here refuses to start while another one, or a mission, is active. Keeping
// that boundary as a FILE boundary is deliberate: in V1 the harnesses, the
// console, the parameter UI and the boot sequence all lived in one 3200-line
// main.cpp, and each new experiment went into it because that was where
// everything already was.
//
// The rule for this module: a harness is a `*_start()` plus a `*_poll()`
// driven from bench_poll(), owning only its own file-static state, and it
// must be startable and abortable from the console. When this file gets past
// about a thousand lines, split it per harness (Bench/Motion.cpp,
// Bench/SensorCal.cpp, ...) rather than letting it grow the way V1's
// main.cpp did.
//
// Commands are documented in docs/COMMANDS.md.

#pragma once

#include <Arduino.h>

// Boot status for the 'p' readout (FRAM present? gains from FRAM or from the
// compiled defaults?). Called once from setup().
void bench_set_boot_status(bool fram_ok, bool params_loaded);
// True once the registry in RAM came from FRAM (loaded at boot, or saved
// with 'w' since). False = every gain is a COMPILED DEFAULT (kyaw 2, track
// 59, kpt 0.0004 ...): the mouse flies, and flies into the first wall
// (2026-09-21, the 16x16 search that turned 75 deg and drove on). The
// search and the route both say so at the line.
bool bench_params_from_fram();

// Print the sub-MCU's power-on threshold self-test. `rerun` re-measures the
// room first. Called at boot and by the 'thr' / 'thr!' commands.
void bench_print_threshold_report(bool rerun);

// Handle one console line (no trailing newline). Empty lines are ignored.
void bench_handle_line(char *line);

// Poll every harness; call once per main-loop pass.
void bench_poll();

// True while any harness OR a mission is running -- the status heartbeat and
// the boot banner stay quiet so they cannot corrupt a CSV stream.
bool bench_busy();
// The idle 10 Hz status line ("tick=... | yaw ... | L R v | C | V"). OFF
// by default since 2026-09-16 (bert: it buries the test data in the
// terminal); 'tick' toggles it, 'tick=1' / 'tick=0' set it. The gyro
// calibration "hold still" line at boot prints regardless.
bool bench_tick_enabled();
