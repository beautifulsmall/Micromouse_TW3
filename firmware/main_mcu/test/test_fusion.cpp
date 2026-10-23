// Host-side test for the Madgwick fusion module (lib/Control/Fusion.*).
// Pure math, no Arduino stubs needed:
//
//   g++ -std=c++17 -I../lib/Control ../lib/Control/Fusion.cpp
//       test_fusion.cpp -o test_fusion && ./test_fusion   (one line)
//
// Scenarios: gyro bias calibration (+ motion guard), stationary heading
// hold, a level turn, a tilted turn (the tilt-compensation case that
// motivates fusing accel with gyro -- naive gz integration under-reads a
// 90 deg turn by ~1.4 deg at 10 deg of tilt), and wall-based heading
// re-referencing via fusion_set_yaw_deg().

#include "Fusion.h"

#include <cassert>
#include <cmath>
#include <cstdio>

// Gyro bias injected into every synthetic sample, dps. Well within the
// FUSION_CAL_MAX_DPS motion guard.
static constexpr float BX = 0.5f, BY = -0.3f, BZ = 0.8f;

static void feed(int n, float gx, float gy, float gz,
                 float ax, float ay, float az) {
    for (int i = 0; i < n; i++) {
        fusion_update(gx + BX, gy + BY, gz + BZ, ax, ay, az);
    }
}

static void expect_near(const char *what, float got, float want, float tol) {
    if (std::fabs(got - want) > tol) {
        std::printf("FAIL: %s = %.3f, expected %.3f +/- %.3f\n",
                    what, got, want, tol);
        assert(false);
    }
}

int main() {
    const int one_sec = static_cast<int>(FUSION_SAMPLE_HZ);

    // -- Settle delay ---------------------------------------------------------
    // The first FUSION_SETTLE_SAMPLES after reset are discarded (mouse
    // settling); even wild motion during settle must not delay readiness.
    fusion_reset();
    assert(!fusion_ready());
    feed(FUSION_SETTLE_SAMPLES, 200.0f, -150.0f, 300.0f, 0.5f, -0.5f, 0.7f);
    assert(!fusion_ready());

    // -- Calibration + motion guard ------------------------------------------
    feed(one_sec / 2, 0, 0, 0, 0, 0, 1);  // half a window...
    fusion_update(BX, BY, 50.0f, 0, 0, 1);  // ...then a movement spike
    feed(one_sec - 1, 0, 0, 0, 0, 0, 1);   // guard restarted: not enough yet
    assert(!fusion_ready());
    feed(1, 0, 0, 0, 0, 0, 1);
    assert(fusion_ready());

    float bias[3];
    fusion_gyro_bias_dps(bias);
    expect_near("bias x", bias[0], BX, 0.01f);
    expect_near("bias y", bias[1], BY, 0.01f);
    expect_near("bias z", bias[2], BZ, 0.01f);

    // -- Stationary: heading holds despite bias ------------------------------
    feed(10 * one_sec, 0, 0, 0, 0, 0, 1);
    expect_near("stationary yaw after 10 s", fusion_yaw_deg(), 0.0f, 0.05f);
    expect_near("stationary pitch", fusion_pitch_deg(), 0.0f, 0.1f);
    expect_near("stationary roll", fusion_roll_deg(), 0.0f, 0.1f);

    // -- Level 90 deg left turn (gz = +90 dps for 1 s) ------------------------
    feed(one_sec, 0, 0, 90.0f, 0, 0, 1);
    expect_near("yaw after level +90 turn", fusion_yaw_deg(), 90.0f, 0.5f);

    // -- Tilted turn: the tilt-compensation case ------------------------------
    // Mouse rolled 10 deg; rotation about the TRUE vertical at 90 dps.
    // Body-frame rates: (0, sin(10)*90, cos(10)*90); gravity in body frame:
    // (0, sin(10), cos(10)). Naive gz integration would read only
    // cos(10)*90 = 88.6 deg; the filter must report 90 about true vertical.
    fusion_reset();
    const float t = 10.0f * static_cast<float>(M_PI) / 180.0f;
    feed(FUSION_SETTLE_SAMPLES + one_sec,  // settle, then calibrate tilted
         0, 0, 0, 0, std::sin(t), std::cos(t));
    assert(fusion_ready());
    expect_near("tilted roll", fusion_roll_deg(), 10.0f, 0.2f);
    feed(one_sec, 0, std::sin(t) * 90.0f, std::cos(t) * 90.0f,
         0, std::sin(t), std::cos(t));
    expect_near("yaw after tilted +90 turn", fusion_yaw_deg(), 90.0f, 0.5f);
    expect_near("roll preserved through turn", fusion_roll_deg(), 10.0f, 0.5f);

    // -- Wall-based re-reference ----------------------------------------------
    fusion_set_yaw_deg(45.0f);
    expect_near("yaw after set_yaw(45)", fusion_yaw_deg(), 45.0f, 0.01f);
    expect_near("roll untouched by set_yaw", fusion_roll_deg(), 10.0f, 0.5f);

    // Wrap: +170 deg more puts us past +180 -> wraps negative.
    feed(one_sec, 0, std::sin(t) * 170.0f, std::cos(t) * 170.0f,
         0, std::sin(t), std::cos(t));
    expect_near("yaw wrap past +180", fusion_yaw_deg(), -145.0f, 1.0f);

    std::printf("ALL FUSION TESTS PASSED\n");
    return 0;
}
