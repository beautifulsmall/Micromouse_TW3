// LapReport host test: the bookkeeping the scoreboard trusts had better
// survive a round trip. Mocks fram_* (1 KB array), the console (captured
// into a string) and the two HAL touches (millis/hal_radio_poll), links
// Params.cpp for the real CRC.

#include <cstdio>
#include <cstring>
#include <string>
#include <cstdarg>

#include "LapReport.h"
#include "Params.h"

static int failures = 0;
static void expect(const char *what, bool ok) {
    if (!ok) {
        std::printf("    FAIL: %s\n", what);
        failures++;
    }
}

// -- Mock Fram: the top blocks (the 8 KB lap report ring) ---------------------
static uint8_t fram_mem[FRAM_TOP_BLOCKS_BYTES];
bool fram_init() { return true; }
bool fram_ready() { return true; }
static size_t off(uint32_t addr) {
    // Every access the module makes must land in the ring.
    if (addr < LAPREP_FRAM_ADDR || addr >= LAPREP_FRAM_ADDR + sizeof fram_mem) {
        std::printf("    FAIL: FRAM access outside the lap report ring: 0x%lx\n",
                    (unsigned long)addr);
        failures++;
        return 0;
    }
    return addr - LAPREP_FRAM_ADDR;
}
void fram_read(uint32_t addr, void *buf, size_t len) {
    std::memcpy(buf, fram_mem + off(addr), len);
}
void fram_write(uint32_t addr, const void *buf, size_t len) {
    std::memcpy(fram_mem + off(addr), buf, len);
}

// -- Mock console + HAL -------------------------------------------------------
static std::string out;
void con_write(const char *s) { out += s; }
void con_println(const char *s) { out += s; out += "\n"; }
void con_printf(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    out += buf;
}
static unsigned long fake_ms = 0;
unsigned long millis() { return fake_ms += 4; }  // pace loops terminate fast
void delay(unsigned long) {}
void hal_radio_poll() {}
void hal_radio_wait_room(uint16_t, uint32_t) {}

// -- Mock live gains (the LPARAM lines print what flew) -----------------------
#include "Control.h"
ControlGains control_get_gains() {
    ControlGains g;      // compiled defaults
    g.kp_turn = 0.005f;  // a recognisable tuned value
    return g;
}

// -----------------------------------------------------------------------------
static bool has(const char *needle) {
    return out.find(needle) != std::string::npos;
}

static void fly_one_lap() {
    lap_report_begin('e');
    // THE FRONT GATE FIRES BEFORE THE ARC ENTRY, so its measurement is
    // recorded BEFORE the corner window opens. This ordering is the real
    // one and it must not lose the reading.
    lap_report_front(0, -7.5f);
    // corner 0: an L90 that over-rotates 0.4 with a locked gate.
    lap_report_corner_enter(0, 5, 0, 84, 90, 10.0f);
    lap_report_corner_sample(0, 1.3f, 120);
    lap_report_corner_sample(0, -2.1f, 63);   // worst ye, apex min
    lap_report_corner_sample(0, 0.5f, 255);   // invalid beam ignored
    lap_report_gate(0, LR_GATE_LOCK, -1.5f, 9.0f);
    // The slip instrument as the gate anchors: rode 840 mm from the
    // launch, gained 1000 mm/s and shed 360 on the way, model added -3.
    lap_report_segment(0, 840.0f, 1000.0f, 360.0f, -3.0f);
    lap_report_corner_close(0, 100.4f, 0.3f, true, 4.0f);
    // corner 1: a 180 whose sweep crosses the wrap: entry 170, cmd +180,
    // close reads -10.5 -> swept 179.5, err -0.5.
    lap_report_corner_enter(1, 0, 5, 71, 180, 170.0f);
    lap_report_corner_sample(1, 6.0f, 80);
    lap_report_corner_close(1, -10.5f, -0.8f, false, 0.0f);
    // corner 2: NO EDGE, and a sweep error past the OLD int8 clamp. The D6
    // zigzag pins +-12.7 on every corner after the first (2026-08-31), so the
    // field is int16 now and this must report the real number, not the rail.
    lap_report_corner_enter(2, 2, 5, 84, -90, 0.0f);
    lap_report_gate(2, LR_GATE_NO_EDGE, 0.0f, -4.0f);
    lap_report_corner_close(2, -70.0f, 0.0f, false, 0.0f);  // err +20
    lap_report_finish(10111, 4728.0f, 0.71f, 0.4f);
}

static void test_report_content() {
    out.clear();
    fly_one_lap();
    expect("header line printed",
           has("dir=e,ms=10110,mm=4728,corners=3,dutypk=0.71,endye=+0.4"));
    expect("corner 0 identity", has("LC,i=0,cell=5:0,r=84,sweep=90"));
    expect("corner 0 sweep err", has("err=+0.4"));
    expect("corner 0 worst ye kept", has("maxye=2.1"));
    expect("corner 0 apex is the min valid beam", has("apex=63"));
    expect("corner 0 gate", has("gate=LOCK,corr=-1.5,glat=9"));
    expect("front column rides alongside the side gate's own number",
           has("gate=LOCK,corr=-1.5,glat=9,fcorr=-7.5"));
    expect("corners the front never saw carry no column",
           out.find("cell=0:5") != std::string::npos &&
               out.find(",fcorr", out.find("cell=0:5") + 200) ==
                   std::string::npos);
    expect("corner 0 exit lat", has("lat=4"));
    // v4: the slip instrument rides after the front column, only on the
    // corner whose gate anchored (dva/dvb round to 8 mm/s).
    expect("slip instrument columns on the anchoring corner",
           has("fcorr=-7.5,ds=840,dva=1000,dvb=360,slip=-3.0"));
    expect("no slip columns where no gate anchored",
           out.find(",ds=", out.find("cell=0:5")) == std::string::npos);
    expect("corner 1 wraps the sweep correctly", has("err=-0.5"));
    expect("corner 2 reports the real +20, not the old int8 rail",
           has("err=+20.0") && !has("err=+12.7"));
    expect("corner 2 outcome", has("gate=NO_EDGE"));
    expect("terminator", has("LAPREP,end"));
    // The registry rides ahead of the report, as flown.
    expect("LPARAM lines carry the registry", has("LPARAM,vmaxl="));
    expect("with the live (not compiled) values", has("kpt=0.005"));
    expect("through the appended params", has("rur=71.74"));
}

static void test_fram_round_trip() {
    out.clear();
    lap_report_print();
    expect("stored block reprints the same corners",
           has("LC,i=0,cell=5:0,r=84") && has("err=-0.5") &&
               has("LAPREP,end"));
    expect("a reprint does NOT stamp today's gains on an old lap",
           !has("LPARAM"));
    // Corrupt one stored byte: the CRC must refuse, not hallucinate.
    size_t at = lap_report_latest_addr() - LAPREP_FRAM_ADDR + sizeof(LapHeader) + 3;
    fram_mem[at] ^= 0x40;
    out.clear();
    lap_report_print();
    expect("corrupted block is refused", has("no report 0 back"));
    fram_mem[at] ^= 0x40;
}

static void test_seq_survives() {
    out.clear();
    fly_one_lap();
    fly_one_lap();
    // Two more laps on top of the first: seq must be 3 by now.
    expect("lap counter continues from FRAM", has("LAPREP,seq=3,"));
}

static void test_the_ring() {
    // Eleven laps on top of the three so far: the ring holds the last
    // eight (seq 7..14), oldest first from lr.all, and 'lr,N' walks back.
    for (int i = 0; i < 11; i++) fly_one_lap();
    out.clear();
    lap_report_print(0);
    expect("latest is seq 14", has("LAPREP,seq=14,"));
    out.clear();
    lap_report_print(3);
    expect("three back is seq 11", has("LAPREP,seq=11,"));
    out.clear();
    lap_report_print(7);
    expect("seven back is seq 7 (the oldest kept)", has("LAPREP,seq=7,"));
    out.clear();
    lap_report_print(8);
    expect("eight back is beyond the ring", has("the ring keeps 8"));
    out.clear();
    int n = lap_report_print_all();
    expect("lr.all prints the whole ring", n == 8);
    size_t p7 = out.find("LAPREP,seq=7,"), p14 = out.find("LAPREP,seq=14,");
    expect("oldest first", p7 != std::string::npos && p14 != std::string::npos && p7 < p14);
    expect("seq 6 is gone", !has("LAPREP,seq=6,"));
}

static void test_guards() {
    out.clear();
    // Events outside a lap, out-of-range corners, double closes: all no-ops.
    lap_report_corner_enter(99, 0, 0, 84, 90, 0.0f);
    lap_report_corner_sample(-1, 1.0f, 50);
    lap_report_gate(20, LR_GATE_LOCK, 3.0f, 0.0f);
    lap_report_begin('n');
    lap_report_corner_enter(0, 1, 1, 84, 90, 0.0f);
    lap_report_corner_close(0, 90.0f, 0.0f, false, 0.0f);
    lap_report_corner_close(0, 45.0f, 9.9f, true, 9.0f);  // second close ignored
    lap_report_finish(5000, 1000.0f, 0.5f, 0.0f);
    expect("double close keeps the first", has("err=+0.0") && !has("err=-45"));
    expect("closed corner stops sampling", !has("maxye=9.9"));
}

static void test_abort_report() {
    out.clear();
    // A lap that dies in corner 2: begin, two corners of evidence, abort.
    lap_report_begin('a');
    lap_report_corner_enter(0, 0, 5, 82, -180, 0.0f);
    lap_report_gate(0, LR_GATE_LOCK, -3.0f, 5.0f);
    lap_report_corner_close(0, -180.2f, 0.1f, true, -8.0f);
    lap_report_corner_enter(1, 2, 5, 84, -90, -180.0f);
    lap_report_abort(3464, 2447.0f, 0.61f);
    expect("abort prints a marked header",
           has("dir=a") && has("abort=1") && has("mm=2447"));
    expect("recorded corners survive the abort",
           has("LC,i=0,cell=0:5") && has("gate=LOCK"));
    expect("abort carries the flown params", has("LPARAM,vmaxl="));
    expect("terminator still printed", has("LAPREP,end"));
    // The stored block keeps the mark: 'lr' after the crash says abort=1.
    out.clear();
    lap_report_print();
    expect("reprint keeps the abort mark", has("abort=1"));
    // A second abort (or finish) after the first is a no-op.
    out.clear();
    lap_report_abort(9999, 9.0f, 0.1f);
    expect("double abort is a no-op", out.empty());
}

int main() {
    test_report_content();
    test_fram_round_trip();
    test_seq_survives();
    test_the_ring();
    test_guards();
    test_abort_report();
    if (failures == 0) {
        std::printf("ALL LAPREPORT TESTS PASSED\n");
        return 0;
    }
    std::printf("%d LAPREPORT TEST(S) FAILED\n", failures);
    return 1;
}
