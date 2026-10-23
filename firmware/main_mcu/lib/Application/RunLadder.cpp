#include "RunLadder.h"

#include <string.h>

#include "Drivers.h"   // fram_read / fram_write / fram_ready
#include "Params.h"    // params_crc32

struct RunLadderBlock {
    uint32_t magic;
    uint8_t version;
    uint8_t next_mr;   // 2..5, or 0 = ladder off
    uint8_t pad[2];
    uint32_t crc;
};
static_assert(sizeof(RunLadderBlock) == 12, "run ladder block must stay 12 B");

static uint8_t next_mr = 0;   // 0 until a map is saved on this firmware

static void save() {
    if (!fram_ready()) return;
    RunLadderBlock b;
    memset(&b, 0, sizeof b);
    b.magic = RUN_LADDER_MAGIC;
    b.version = RUN_LADDER_VERSION;
    b.next_mr = next_mr;
    b.crc = params_crc32(&b, sizeof(RunLadderBlock) - sizeof(uint32_t));
    fram_write(RUN_LADDER_FRAM_ADDR, &b, sizeof b);
}

void run_ladder_init() {
    RunLadderBlock b;
    fram_read(RUN_LADDER_FRAM_ADDR, &b, sizeof b);
    if (b.magic != RUN_LADDER_MAGIC || b.version != RUN_LADDER_VERSION ||
        b.crc != params_crc32(&b, sizeof(RunLadderBlock) - sizeof(uint32_t))) {
        next_mr = 0;   // no block: the ladder is off until a map is saved
        return;
    }
    next_mr = b.next_mr > RUN_LADDER_LAST ? RUN_LADDER_LAST : b.next_mr;
}

uint8_t run_ladder_next() { return next_mr; }

void run_ladder_reset() {
    next_mr = RUN_LADDER_FIRST;
    save();
}

uint8_t run_ladder_take() { return next_mr; }

void run_ladder_climb(uint8_t flown) {
    if (flown == 0 || next_mr != flown) return;
    if (next_mr < RUN_LADDER_LAST) {
        next_mr++;
        save();
    }
}

void run_ladder_set(uint8_t mr) {
    if (mr != 0 && mr < RUN_LADDER_FIRST) mr = RUN_LADDER_FIRST;
    if (mr > RUN_LADDER_LAST) mr = RUN_LADDER_LAST;
    next_mr = mr;
    save();
}

float run_ladder_out_v(uint8_t mr, const ControlGains &g) {
    switch (mr) {
        case 2: return g.rv_mr2;
        case 3: return g.rv_mr3;
        case 4: return g.rv_mr4;
        case 5: return g.rv_mr5;
        default: return g.route_v;
    }
}

float run_ladder_home_v(uint8_t mr, const ControlGains &g) {
    return mr == 0 ? g.route_v : g.rv_home;
}

float run_ladder_accel_scale(uint8_t mr, const ControlGains &g) {
    if (mr < 2 || mr >= 5) return 1.0f;
    float m = g.ladder_accel_mr2;
    if (m < 0.1f) m = 0.1f;
    if (m > 1.0f) m = 1.0f;
    return m + (1.0f - m) * (float)(mr - 2) / 3.0f;
}
