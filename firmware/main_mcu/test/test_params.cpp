// Host-side test: mock the Fram driver with a byte array and verify the
// persistent parameter block (Params.h): save/load round-trip, CRC and
// magic/version rejection, graceful defaults when FRAM is absent.
// Build (one line):
//
//   g++ -std=c++17 -Istubs -I../lib/Application -I../lib/Control
//     -I../lib/Drivers -I../lib/HAL ../lib/Application/Params.cpp
//     test_params.cpp -o test_params && ./test_params

#include "Params.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

// -- Arduino stub definitions -------------------------------------------------
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void delay(unsigned long) {}
unsigned long millis() { return 0; }
void noInterrupts() {}
void interrupts() {}

// -- Mock Fram driver (Params links against fram_*, not the HAL) ----------------
static uint8_t fram_mem[1024];
static bool fram_present = true;

bool fram_init() { return fram_present; }
bool fram_ready() { return fram_present; }
void fram_read(uint32_t addr, void *buf, size_t len) {
    if (!fram_present) {
        memset(buf, 0, len);
        return;
    }
    memcpy(buf, fram_mem + addr, len);
}
void fram_write(uint32_t addr, const void *buf, size_t len) {
    if (!fram_present) return;
    memcpy(fram_mem + addr, buf, len);
}

static void expect_near(const char *what, float got, float want) {
    if (std::fabs(got - want) > 1e-6f * (1.0f + std::fabs(want))) {
        std::printf("FAIL: %s = %g, expected %g\n", what, got, want);
        assert(false);
    }
}

int main() {
    // CRC-32 known answer ("123456789" -> 0xCBF43926, the standard check).
    assert(params_crc32("123456789", 9) == 0xCBF43926);

    // ---- THE REGISTRY ITSELF ------------------------------------------------
    // Golden name list, IN ORDER. The stored block is "values in registry
    // order", so the order IS the file format: appending to this list is how
    // a parameter is born; editing the middle of it is how every mouse's
    // saved tuning gets scrambled into the wrong fields. If this assert
    // fires, you reordered or renamed -- don't retarget the list, undo.
    static const char *const GOLDEN[PARAM_COUNT] = {
        "vmaxl", "vmaxr", "fricl", "fricr", "kp", "kpt", "kd", "ka",
        "kx", "kyaw", "strim", "track", "tasym", "us", "kslip",
        "circl", "circr",
        // appended 2026-08-27: velocity filter, heading clamp, yaw integral,
        // route mission numbers
        "vfilt", "dvmax", "kyawi", "kyawimax", "rv", "ralon", "ralat",
        "rslew",
        // appended 2026-08-28: the arc wheel gain (runs 188/189 A/B)
        "kparc",
        // appended 2026-08-29: gate expectation offset + tunable U radius
        "goff", "rur",
        // appended 2026-08-29 later: per-corner-type gate offsets
        "goffr", "goffl", "goffur", "gofful",
        // appended 2026-08-29 later still: gate lateral compensation,
        // per beam (left, right)
        "gklat", "gklatr", "glearn", "r90", "l90", "atrim", "utrim", "fgate", "ftrim",
        "dgate", "dtrim", "dlat", "dmax", "sacc", "sbrk", "flinch", "ftau",
        "walign", "vout", "latk", "starty", "r135", "sv", "tomg", "talp", "rv2", "rv3", "rv4", "rv5", "rvh", "absb", "vsrc", "kaa", "rbrk", "mra", "vref", "chold", "rdiag", "rturn", "dwheel", "rturnh", "rdv", "rvv", "rvu", "vlearn", "ghome"};
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (std::strcmp(PARAM_REGISTRY[i].name, GOLDEN[i]) != 0) {
            std::printf("FAIL: registry[%d] is '%s', golden order says '%s'\n",
                        i, PARAM_REGISTRY[i].name, GOLDEN[i]);
            assert(false);
        }
        assert(PARAM_REGISTRY[i].min < PARAM_REGISTRY[i].max);
        // The compiled default must sit inside its own clamp, or a fresh
        // mouse boots pre-clamped and the default is a lie.
        float d = ControlGains{}.*(PARAM_REGISTRY[i].field);
        assert(d >= PARAM_REGISTRY[i].min && d <= PARAM_REGISTRY[i].max);
        // Names and fields are unique: two rows sharing a field would make
        // the last one win silently on load.
        for (int j = 0; j < i; j++) {
            assert(std::strcmp(PARAM_REGISTRY[i].name,
                               PARAM_REGISTRY[j].name) != 0);
            assert(PARAM_REGISTRY[i].field != PARAM_REGISTRY[j].field);
        }
    }
    assert(param_find("kyaw") == &PARAM_REGISTRY[9]);
    assert(param_find("nonsense") == nullptr);

    // Every float in ControlGains is reachable through the registry: 17
    // distinct member pointers over a 17-float struct is full coverage (the
    // static_assert in Params.h pins the sizes).

    // ---- Empty FRAM: load fails, gains = compiled defaults ------------------
    ControlGains g;
    assert(!params_load(g));
    expect_near("default vmaxl", g.vmax_left_mm_s, VCTRL_VMAX_LEFT_MM_S);
    expect_near("default kp", g.kp, VCTRL_KP);

    // ---- Save -> load round-trip preserves every field ----------------------
    // Values chosen INSIDE every clamp; out-of-range is its own test below.
    ControlGains saved;
    saved.vmax_left_mm_s = 7515.0f;
    saved.vmax_right_mm_s = 7591.0f;
    saved.friction_left = 0.0021f;
    saved.friction_right = 0.0019f;
    saved.kp = 0.00055f;
    saved.kp_turn = 0.0026f;
    saved.kd = 0.000012f;
    saved.ka = 2.5e-6f;      // the physically-derived value, not the corruption
    saved.kx = 2.5f;
    saved.kyaw = 10.0f;      // what the real mouse actually runs (2026-08-27)
    saved.track_mm = 72.31f; // ditto
    saved.arc_understeer = 0.12f;
    assert(params_save(saved));

    ControlGains loaded;
    assert(params_load(loaded));
    for (int i = 0; i < PARAM_COUNT; i++)
        expect_near(PARAM_REGISTRY[i].name, loaded.*(PARAM_REGISTRY[i].field),
                    saved.*(PARAM_REGISTRY[i].field));

    // us round-tripping matters historically: arc_understeer was tunable in
    // RAM but absent from the v10 block, so it silently died at power-off.
    expect_near("us survives the power cycle", loaded.arc_understeer, 0.12f);

    // ---- The clamp, at both doors -------------------------------------------
    // params_clamp reports and repairs; the ka ceiling is sized so the
    // 2026-07-09 corruption value (1.5e-4) can never reach the ISR again.
    ControlGains hot = saved;
    hot.ka = 1.5e-4f;
    hot.kyaw = 400.0f;
    assert(params_clamp(hot) == 2);
    expect_near("ka clamped to its ceiling", hot.ka, 1e-4f);
    expect_near("kyaw clamped", hot.kyaw, 50.0f);
    assert(params_clamp(hot) == 0);  // idempotent

    // And the load door: a block whose bytes are intact (CRC valid) but whose
    // VALUES are insane loads clamped -- CRC proves the write, not the sanity.
    hot.ka = 1.5e-4f;
    assert(params_save(hot));   // save does not judge; load does
    assert(params_load(loaded));
    expect_near("insane stored ka arrives clamped", loaded.ka, 1e-4f);

    // NaN is FRAM corruption's favourite float; it must land on the compiled
    // default, not sail through comparisons into the ISR.
    ControlGains nang = saved;
    nang.kyaw = NAN;
    assert(params_clamp(nang) == 1);
    expect_near("NaN -> compiled default", nang.kyaw, VCTRL_KYAW);

    // ---- Corruption: flip one payload byte -> CRC rejects, defaults ---------
    assert(params_save(saved));
    fram_mem[PARAMS_FRAM_ADDR + 10] ^= 0x40;
    assert(!params_load(loaded));
    expect_near("post-corruption vmaxl", loaded.vmax_left_mm_s,
                VCTRL_VMAX_LEFT_MM_S);
    assert(params_save(saved));  // re-save repairs
    assert(params_load(loaded));

    // ---- The v11 count field IS the migration -------------------------------
    // An OLDER v11 block (fewer params -- here, one saved before "circr"
    // existed... which cannot happen for these 17, so simulate the future:
    // a 15-count block) loads its 15 and defaults the rest.
    {
        struct H { uint32_t magic; uint16_t version, count; } h{
            PARAMS_MAGIC, PARAMS_VERSION, 15};
        uint8_t buf[8 + 15 * 4 + 4];
        memcpy(buf, &h, 8);
        float v[15];
        for (int i = 0; i < 15; i++) v[i] = saved.*(PARAM_REGISTRY[i].field);
        v[9] = 7.5f;  // kyaw, to prove the stored value lands
        memcpy(buf + 8, v, sizeof v);
        uint32_t crc = params_crc32(buf, 8 + sizeof v);
        memcpy(buf + 8 + sizeof v, &crc, 4);
        memcpy(fram_mem + PARAMS_FRAM_ADDR, buf, sizeof buf);
        assert(params_load(loaded));
        expect_near("short block: stored kyaw lands", loaded.kyaw, 7.5f);
        expect_near("short block: circl defaults", loaded.circ_left_mm,
                    ControlGains{}.circ_left_mm);
    }
    // A FUTURE block (count > PARAM_COUNT) is refused whole, not half-read.
    {
        struct H { uint32_t magic; uint16_t version, count; } h{
            PARAMS_MAGIC, PARAMS_VERSION, PARAM_COUNT + 3};
        memcpy(fram_mem + PARAMS_FRAM_ADDR, &h, 8);
        assert(!params_load(loaded));
        expect_near("future block -> defaults", loaded.kp, VCTRL_KP);
    }

    // ---- save_at/load_at: the flight-log snapshot path ----------------------
    {
        const uint32_t SNAP = 512;  // inside the mock, away from the block
        assert(params_save_at(SNAP, saved));
        ControlGains snap = ControlGains{};
        assert(params_load_at(SNAP, snap));
        expect_near("snapshot kyaw", snap.kyaw, saved.kyaw);
        expect_near("snapshot us", snap.arc_understeer, saved.arc_understeer);
        fram_mem[SNAP + 12] ^= 0x01;
        assert(!params_load_at(SNAP, snap));
    }

    // ---- Legacy blocks still load (one last generation) ---------------------
    // v2: fields survive, fields it predates take defaults. Its ka of 1.9e-4
    // is era-authentic -- and must now arrive CLAMPED.
    struct V2 {
        uint32_t magic;
        uint16_t version;
        uint16_t size;
        float vmaxl, vmaxr, fricl, fricr, kp, kd, ka;
        uint32_t crc;
    } v2{};
    v2.magic = PARAMS_MAGIC;
    v2.version = 2;
    v2.size = sizeof(V2);
    v2.vmaxl = 7000.0f;
    v2.vmaxr = 7100.0f;
    v2.fricl = 0.0030f;
    v2.fricr = 0.0031f;
    v2.kp = 0.0005f;
    v2.kd = 0.0f;
    v2.ka = 0.00019f;
    v2.crc = params_crc32(&v2, sizeof(V2) - sizeof(uint32_t));
    memcpy(fram_mem + PARAMS_FRAM_ADDR, &v2, sizeof(v2));
    assert(params_load(loaded));
    expect_near("v2 vmaxl kept", loaded.vmax_left_mm_s, 7000.0f);
    expect_near("v2 ka arrives clamped", loaded.ka, 1e-4f);
    expect_near("v2 kx defaulted", loaded.kx, VCTRL_KX);
    expect_near("v2 kyaw defaulted", loaded.kyaw, VCTRL_KYAW);

    // v5 migration: kp_turn predated by the block -> seeded from stored kp.
    struct V5 {
        uint32_t magic;
        uint16_t version;
        uint16_t size;
        float vmaxl, vmaxr, fricl, fricr, kp, kd, ka, kx, kyaw, strim, track;
        uint32_t crc;
    } v5{};
    v5.magic = PARAMS_MAGIC;
    v5.version = 5;
    v5.size = sizeof(V5);
    v5.vmaxl = 7515.0f;
    v5.vmaxr = 7591.0f;
    v5.fricl = 0.0021f;
    v5.fricr = 0.0019f;
    v5.kp = 0.0041f;  // turn-stiff (saved after a 't' run)
    v5.kd = 0.0005f;
    v5.ka = 2e-6f;
    v5.kx = 3.0f;
    v5.kyaw = 2.0f;
    v5.strim = -0.0071f;
    v5.track = 66.2f;
    v5.crc = params_crc32(&v5, sizeof(V5) - sizeof(uint32_t));
    memcpy(fram_mem + PARAMS_FRAM_ADDR, &v5, sizeof(v5));
    assert(params_load(loaded));
    expect_near("v5 kp kept", loaded.kp, 0.0041f);
    expect_near("v5 kpt seeded from kp", loaded.kp_turn, 0.0041f);
    expect_near("v5 strim kept", loaded.strim, -0.0071f);
    expect_near("v5 track kept", loaded.track_mm, 66.2f);

    // And a v10 block -- the last packed struct, what bert's mouse holds
    // right now. It must load, and the field it predates (us) must default.
    struct V10 {
        uint32_t magic;
        uint16_t version;
        uint16_t size;
        float vmaxl, vmaxr, fricl, fricr, kp, kpt, kd, ka, kx, kyaw, strim,
            track, tasym, circl, circr, kslip;
        uint32_t crc;
    } v10{};
    v10.magic = PARAMS_MAGIC;
    v10.version = 10;
    v10.size = sizeof(V10);
    v10.vmaxl = 7515.0f;
    v10.vmaxr = 7591.0f;
    v10.fricl = 0.002f;
    v10.fricr = 0.002f;
    v10.kp = 0.0004f;
    v10.kpt = 0.0004f;
    v10.kd = 0.0f;
    v10.ka = 0.0f;
    v10.kx = 3.0f;
    v10.kyaw = 10.0f;
    v10.strim = 0.0f;
    v10.track = 72.31f;
    v10.tasym = 0.0f;
    v10.circl = 67.788f;
    v10.circr = 68.103f;
    v10.kslip = 0.0f;
    v10.crc = params_crc32(&v10, sizeof(V10) - sizeof(uint32_t));
    memcpy(fram_mem + PARAMS_FRAM_ADDR, &v10, sizeof(v10));
    assert(params_load(loaded));
    expect_near("v10 kyaw kept", loaded.kyaw, 10.0f);
    expect_near("v10 track kept", loaded.track_mm, 72.31f);
    expect_near("v10 us defaulted", loaded.arc_understeer,
                VCTRL_ARC_UNDERSTEER);
    // ...and the next save writes v11, ending the parade.
    assert(params_save(loaded));
    uint16_t ver;
    memcpy(&ver, fram_mem + PARAMS_FRAM_ADDR + 4, 2);
    assert(ver == PARAMS_VERSION);

    // ---- FRAM missing: save reports failure, load returns defaults ----------
    fram_present = false;
    assert(!params_save(saved));
    assert(!params_load(loaded));
    expect_near("no-FRAM kp", loaded.kp, VCTRL_KP);

    std::printf("ALL PARAMS TESTS PASSED\n");
    return 0;
}
