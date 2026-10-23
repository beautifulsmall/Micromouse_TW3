// GoffTable host test: the learned per-corner offsets must converge with
// the right SIGN, survive a FRAM round trip, refuse corruption, and refuse
// overflow loudly. Mocks fram_* (2 KB array covering the 0x710 window) and
// the console; links Params.cpp for the real CRC.

#include <cstdio>
#include <cstring>
#include <string>
#include <cstdarg>

#include "GoffTable.h"

static int failures = 0;
static void expect(const char *what, bool ok) {
    if (!ok) {
        std::printf("    FAIL: %s\n", what);
        failures++;
    }
}

// -- Mock Fram ----------------------------------------------------------------
static uint8_t fram_mem[2048];
static bool fram_ok = true;
bool fram_init() { return true; }
bool fram_ready() { return fram_ok; }
void fram_read(uint32_t addr, void *buf, size_t len) {
    std::memcpy(buf, fram_mem + (addr & 0x7FF), len);
}
void fram_write(uint32_t addr, const void *buf, size_t len) {
    std::memcpy(fram_mem + (addr & 0x7FF), buf, len);
}

// -- Mock console -------------------------------------------------------------
static std::string out;
void con_write(const char *s) { out += s; }
void con_println(const char *s) { out += s; out += "\n"; }
void hal_radio_wait_room(uint16_t, uint32_t) {}
void con_printf(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    out += buf;
}
static bool has(const char *needle) {
    return out.find(needle) != std::string::npos;
}

// -----------------------------------------------------------------------------
static void test_learning_sign_and_convergence() {
    goff_table_clear();
    // Corner (5,5) approached heading S: the gate keeps reporting corr -28
    // (edge LATER than expected), as laps 101-106 actually did. The entry
    // must go POSITIVE (expectation must grow) and converge on +28.
    goff_table_learn(5, 5, 2, -28.0f, 0.5f);
    expect("first fold is alpha * -corr",
           goff_table_mm(5, 5, 2) == 14.0f);
    // Next lap the table is applied, so the residual corr shrinks to -14.
    goff_table_learn(5, 5, 2, -14.0f, 0.5f);
    expect("integrator accumulates", goff_table_mm(5, 5, 2) == 21.0f);
    goff_table_learn(5, 5, 2, -7.0f, 0.5f);
    expect("three laps converge within 4 mm",
           goff_table_mm(5, 5, 2) >= 24.0f && goff_table_mm(5, 5, 2) <= 28.0f);
    // Same cell, opposite approach: a DIFFERENT physical edge, so a
    // different entry -- (5,0) measured -39.5 one way, -28.5 the other.
    goff_table_learn(5, 5, 0, 10.0f, 1.0f);
    expect("opposite approach is its own entry",
           goff_table_mm(5, 5, 0) == -10.0f &&
               goff_table_mm(5, 5, 2) >= 24.0f);
    expect("unknown corner reads zero", goff_table_mm(3, 3, 1) == 0.0f);
    // alpha = 0 (glearn off) must not create or move anything.
    goff_table_learn(2, 2, 1, -30.0f, 0.0f);
    expect("glearn=0 learns nothing", goff_table_mm(2, 2, 1) == 0.0f);
    // Clamp: a runaway correction cannot poison the map.
    goff_table_learn(4, 4, 1, -500.0f, 1.0f);
    expect("entries clamp at the cap",
           goff_table_mm(4, 4, 1) == GOFF_TABLE_MAX_MM);
}

static void test_fram_round_trip() {
    goff_table_clear();
    goff_table_learn(5, 5, 2, -28.0f, 1.0f);   // +28
    goff_table_learn(5, 0, 2, -40.0f, 1.0f);   // +40
    goff_table_set(5, 3, 1, -12.0f);           // the hand-set verb
    goff_table_save();
    // Reload from FRAM as a boot would.
    goff_table_load();
    expect("round trip keeps count", goff_table_count() == 3);
    expect("round trip keeps values",
           goff_table_mm(5, 5, 2) == 28.0f &&
               goff_table_mm(5, 0, 2) == 40.0f &&
               goff_table_mm(5, 3, 1) == -12.0f);
    // Corrupt one stored byte: the CRC must refuse, not hallucinate.
    fram_mem[(GOFF_TABLE_FRAM_ADDR & 0x7FF) + 9] ^= 0x40;
    out.clear();
    goff_table_load();
    expect("corrupted block starts empty", goff_table_count() == 0);
    expect("and says so", has("failed CRC"));
    fram_mem[(GOFF_TABLE_FRAM_ADDR & 0x7FF) + 9] ^= 0x40;
    goff_table_load();
    expect("uncorrupted block loads again", goff_table_count() == 3);
}

static void test_full_table_refuses() {
    goff_table_clear_all();
    for (int i = 0; i < GOFF_TABLE_MAX; i++)
        goff_table_set((uint8_t)(i & 15), (uint8_t)(i >> 4), 0, 5.0f);
    expect("table fills to its cap", goff_table_count() == GOFF_TABLE_MAX);
    out.clear();
    bool ok = goff_table_learn(15, 15, 3, -20.0f, 0.5f);
    expect("a full table refuses a NEW corner", !ok && has("FULL"));
    // ...but an EXISTING corner still learns.
    ok = goff_table_learn(0, 0, 0, -10.0f, 1.0f);
    expect("existing corners still learn when full",
           ok && goff_table_mm(0, 0, 0) == 15.0f);
}

static void test_print_format() {
    goff_table_clear_all();
    goff_table_set(5, 5, 2, 28.0f);
    out.clear();
    goff_table_print();
    expect("GOFFC line is key=value, colon cell, heading letter, trim column",
           has("GOFFC,cell=5:5,h=S,off=+28,trim=+0"));
}

// The per-corner trim: hand-set, never learned, kept across a clear.
// (bert, 2026-09-01: "the first right needs starting earlier" -- (0,5)
// began its arc 32 mm later than (5,5) and exited 35 mm outside; the
// learner cannot hold an exit position, atrim is global, so this.)
static void test_corner_trim() {
    goff_table_clear_all();
    expect("no trim on an unknown corner", goff_table_trim_mm(0, 5, 0) == 0.0f);
    expect("setting a trim creates the entry",
           goff_table_set_trim(0, 5, 0, -30.0f) && goff_table_count() == 1);
    expect("and reads back", goff_table_trim_mm(0, 5, 0) == -30.0f);
    expect("with the learned offset untouched", goff_table_mm(0, 5, 0) == 0.0f);
    // The learner moves the offset, never the trim.
    goff_table_learn(0, 5, 0, -12.0f, 1.0f);   // existing entry: slewed to 10
    expect("learning moves the offset", goff_table_mm(0, 5, 0) == 10.0f);
    expect("learning leaves the trim alone", goff_table_trim_mm(0, 5, 0) == -30.0f);
    // A plain clear forgets what was learned and keeps what was set by hand.
    goff_table_set(5, 5, 1, 20.0f);   // a learned-only corner
    out.clear();
    goff_table_clear();
    expect("clear keeps the trimmed corner", goff_table_count() == 1);
    expect("with its offset zeroed", goff_table_mm(0, 5, 0) == 0.0f);
    expect("and its trim intact", goff_table_trim_mm(0, 5, 0) == -30.0f);
    expect("and says so", has("hand trim") && has("kept"));
    expect("the learned-only corner is gone", goff_table_mm(5, 5, 1) == 0.0f);
    // It survives FRAM.
    goff_table_save();
    goff_table_load();
    expect("the trim round-trips through FRAM",
           goff_table_trim_mm(0, 5, 0) == -30.0f);
    out.clear();
    goff_table_print();
    expect("and prints", has("GOFFC,cell=0:5,h=N,off=+0,trim=-30"));
    goff_table_clear_all();
    expect("clear all drops it", goff_table_count() == 0);
}

int main() {
    test_learning_sign_and_convergence();
    test_fram_round_trip();
    test_full_table_refuses();
    test_print_format();
    test_corner_trim();
    if (failures == 0) {
        std::printf("ALL GOFF TABLE TESTS PASSED\n");
        return 0;
    }
    std::printf("%d GOFF TABLE TEST(S) FAILED\n", failures);
    return 1;
}
