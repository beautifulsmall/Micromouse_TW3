// RunLadder host test: the five-run counter the competition rides on.
// Mocks fram_* (the small-block window) and the Arduino touches; links
// Params.cpp for the real CRC.
//
//   ./run_tests.sh ladder

#include <cstdio>
#include <cstring>

#include "RunLadder.h"
#include "Params.h"

static int failures = 0;
static void expect(const char *what, bool ok) {
    if (!ok) { std::printf("    FAIL: %s\n", what); failures++; }
}

// -- Mocks --------------------------------------------------------------------
static uint8_t fram_mem[0x800];
static bool fram_present = true;
bool fram_init() { return fram_present; }
bool fram_ready() { return fram_present; }
void fram_read(uint32_t addr, void *buf, size_t len) {
    if (!fram_present) { std::memset(buf, 0, len); return; }
    std::memcpy(buf, fram_mem + (addr & 0x7FF), len);
}
void fram_write(uint32_t addr, const void *buf, size_t len) {
    if (!fram_present) return;
    std::memcpy(fram_mem + (addr & 0x7FF), buf, len);
}
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void delay(unsigned long) {}
void noInterrupts() {}
void interrupts() {}

int main() {
    ControlGains g;   // compiled defaults: rv 500? -- whatever they are, read them
    std::memset(fram_mem, 0, sizeof fram_mem);

    // Fresh chip: no block -> the ladder is OFF and every leg flies rv.
    run_ladder_init();
    expect("no block: ladder off", run_ladder_next() == 0);
    expect("off: take returns 0", run_ladder_take() == 0);
    expect("off: out leg flies rv", run_ladder_out_v(0, g) == g.route_v);
    expect("off: home leg flies rv", run_ladder_home_v(0, g) == g.route_v);

    // A map save starts the ladder at MR2. A launch TAKES the rung without
    // moving it; only reaching the goal climbs (2026-09-30: a crash, an
    // abort or a mouse carried off flies the same rung again). MR5 holds.
    run_ladder_reset();
    expect("after a map save: next is MR2", run_ladder_next() == 2);
    expect("first launch is MR2", run_ladder_take() == 2);
    expect("a launch alone does not climb", run_ladder_next() == 2);
    expect("an aborted MR2: the next launch is MR2 again", run_ladder_take() == 2);
    run_ladder_climb(2);
    expect("MR2 reached the goal: next is MR3", run_ladder_next() == 3);
    run_ladder_climb(2);
    expect("a stale climb (rung already moved) does nothing", run_ladder_next() == 3);
    expect("second launch is MR3", run_ladder_take() == 3);
    run_ladder_climb(3);
    expect("third is MR4", run_ladder_take() == 4);
    run_ladder_climb(4);
    expect("fourth is MR5", run_ladder_take() == 5);
    run_ladder_climb(5);
    expect("fifth is still MR5", run_ladder_take() == 5);
    expect("and the next stays MR5", run_ladder_next() == 5);
    run_ladder_set(3);
    run_ladder_climb(5);
    expect("mr=3 typed after an MR5 launch: the goal does not override it",
           run_ladder_next() == 3);
    run_ladder_climb(0);
    expect("ladder-off run: climb(0) does nothing", run_ladder_next() == 3);

    // The rungs map to the table, the home leg to rvh.
    expect("MR2 out = rv2", run_ladder_out_v(2, g) == g.rv_mr2);
    expect("MR3 out = rv3", run_ladder_out_v(3, g) == g.rv_mr3);
    expect("MR4 out = rv4", run_ladder_out_v(4, g) == g.rv_mr4);
    expect("MR5 out = rv5", run_ladder_out_v(5, g) == g.rv_mr5);
    expect("home = rvh on every rung", run_ladder_home_v(3, g) == g.rv_home &&
                                       run_ladder_home_v(5, g) == g.rv_home);
    expect("defaults: 800/1000/1500/2000 out, 500 home",
           g.rv_mr2 == 800.0f && g.rv_mr3 == 1000.0f && g.rv_mr4 == 1500.0f &&
               g.rv_mr5 == 2000.0f && g.rv_home == 500.0f);

    // The counter survives a power cycle (re-init from FRAM).
    run_ladder_reset();
    run_ladder_take();               // MR2 flown...
    run_ladder_climb(2);             // ...to the goal, next MR3
    run_ladder_init();               // "power cycle"
    expect("counter restored from FRAM", run_ladder_next() == 3);

    // mr=N sets; mr=0 switches off; out of range clamps.
    run_ladder_set(5);
    expect("mr=5", run_ladder_next() == 5);
    run_ladder_set(9);
    expect("mr=9 clamps to 5", run_ladder_next() == 5);
    run_ladder_set(1);
    expect("mr=1 clamps to 2 (MR1 is the search)", run_ladder_next() == 2);
    run_ladder_set(0);
    expect("mr=0 switches the ladder off", run_ladder_next() == 0);
    run_ladder_init();
    expect("off survives a power cycle", run_ladder_next() == 0);

    // A corrupted block reads as OFF, never as a wrong rung.
    run_ladder_reset();
    fram_mem[(RUN_LADDER_FRAM_ADDR & 0x7FF) + 5] ^= 0x01;   // the rung byte
    run_ladder_init();
    expect("corrupt block: ladder off", run_ladder_next() == 0);

    // No FRAM: everything is RAM only and off at boot.
    fram_present = false;
    run_ladder_init();
    expect("no FRAM: off", run_ladder_next() == 0);
    run_ladder_reset();
    expect("no FRAM: reset still works in RAM", run_ladder_next() == 2);

    if (failures == 0) { std::printf("ALL LADDER TESTS PASSED\n"); return 0; }
    std::printf("%d LADDER TEST(S) FAILED\n", failures);
    return 1;
}
