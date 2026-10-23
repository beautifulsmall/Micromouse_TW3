// Host-side test: wall interpretation (lib/Control/Walls.*). Pure logic over
// WallRanges -- presence thresholds, 255 = absent, front-distance projection,
// closest-front selection, and staleness. No HAL/Arduino beyond the uint stub.
//
// Build:
//   g++ -std=c++17 -Istubs -I../lib/Control -I../lib/Drivers ../lib/Control/Walls.cpp test_walls.cpp -o test_walls && ./test_walls

#include "Walls.h"

#include <cmath>
#include <cstdio>

static int failures = 0;
static void expect(const char *what, bool cond) {
    if (!cond) { std::printf("FAIL: %s\n", what); failures++; }
}

// Args in WallRanges field order: left, front_right, front_left, right.
static WallRanges mk(uint8_t l, uint8_t fr, uint8_t fl, uint8_t r) {
    WallRanges w;
    w.left_mm = l; w.front_right_mm = fr; w.front_left_mm = fl; w.right_mm = r;
    w.seq = 1;
    return w;
}

int main() {
    // -- All walls present -----------------------------------------------------
    WallView v = walls_evaluate(mk(90, 100, 100, 95), 5);
    expect("fresh valid", v.valid);
    expect("left present", v.left);
    expect("right present", v.right);
    expect("front present", v.front);
    // front LUTs are forward-from-centre: front_mm = closest beam directly
    expect("front_mm = 100 (closest beam)", std::fabs(v.front_mm - 100.0f) < 1.5f);

    // -- No walls (out of range) -----------------------------------------------
    v = walls_evaluate(mk(255, 255, 255, 255), 5);
    expect("no left", !v.left);
    expect("no right", !v.right);
    expect("no front", !v.front);
    expect("front_mm none", v.front_mm == WALL_FRONT_NONE_MM);

    // -- One side only ---------------------------------------------------------
    v = walls_evaluate(mk(80, 255, 255, 255), 5);
    expect("left only", v.left && !v.right && !v.front);

    // -- Threshold boundary ----------------------------------------------------
    v = walls_evaluate(mk(WALL_LEFT_PRESENT_MM, 255, 255, 255), 5);
    expect("at threshold present", v.left);
    v = walls_evaluate(mk((uint8_t)(WALL_LEFT_PRESENT_MM + 1), 255, 255, 255), 5);
    expect("above threshold absent", !v.left);

    // -- Front distance uses the CLOSEST beam ----------------------------------
    v = walls_evaluate(mk(255, 60, 120, 255), 5);  // fr=60, fl=120
    // forward-from-centre pass-through: min(60, 120) = 60
    expect("front uses closest beam", std::fabs(v.front_mm - 60.0f) < 1.5f);

    // -- Side calibration: LUTs are now PERPENDICULAR-from-centre (auto-cal
    // 'S'), so left_mm/right_mm pass through and lateral = (right-left)/2.
    // Centred corridor: each wall 84 mm (WALL_CORRIDOR_HALF_MM).

    // 20 mm toward the LEFT wall: d_left = 64, d_right = 104, lateral +20.
    v = walls_evaluate(mk(64, 255, 255, 104), 5);
    expect("pose L: left_mm 64", std::fabs(v.left_mm - 64.0f) < 1.0f);
    expect("pose L: right_mm 104", std::fabs(v.right_mm - 104.0f) < 1.0f);
    expect("pose L: lateral ~+20", v.lateral_valid &&
                                       std::fabs(v.lateral_mm - 20.0f) < 1.0f);

    // 20 mm toward the RIGHT wall: mirrored, lateral -20.
    v = walls_evaluate(mk(104, 255, 255, 64), 5);
    expect("pose R: left_mm 104", std::fabs(v.left_mm - 104.0f) < 1.0f);
    expect("pose R: right_mm 64", std::fabs(v.right_mm - 64.0f) < 1.0f);
    expect("pose R: lateral ~-20", v.lateral_valid &&
                                       std::fabs(v.lateral_mm + 20.0f) < 1.0f);

    // Centre pose: width recovered 168 (2*84), lateral 0.
    v = walls_evaluate(mk(84, 255, 255, 84), 5);
    expect("pose C: width 168", std::fabs((v.left_mm + v.right_mm) - 168.0f) < 1.5f);
    expect("pose C: lateral 0", v.lateral_valid && std::fabs(v.lateral_mm) < 1.0f);

    // One wall only: lateral falls back to the corridor-half constant (84).
    v = walls_evaluate(mk(64, 255, 255, 255), 5);  // left wall at 64
    expect("left-only lateral ~+20", v.lateral_valid &&
                                         std::fabs(v.lateral_mm - 20.0f) < 1.0f);
    v = walls_evaluate(mk(255, 255, 255, 64), 5);  // right wall at 64
    expect("right-only lateral ~-20", v.lateral_valid &&
                                          std::fabs(v.lateral_mm + 20.0f) < 1.0f);

    // No side walls: lateral invalid.
    v = walls_evaluate(mk(255, 100, 100, 255), 5);
    expect("no sides: lateral invalid", !v.lateral_valid && v.lateral_mm == 0.0f);

    // -- Stale data -> invalid, flags false ------------------------------------
    v = walls_evaluate(mk(80, 80, 80, 80), 150);
    expect("stale invalid", !v.valid);
    expect("stale no flags", !v.left && !v.right && !v.front);
    expect("stale front none", v.front_mm == WALL_FRONT_NONE_MM);
    expect("stale lateral invalid", !v.lateral_valid);

    if (failures == 0) { std::printf("ALL WALL TESTS PASSED\n"); return 0; }
    std::printf("%d WALL TEST(S) FAILED\n", failures);
    return 1;
}
