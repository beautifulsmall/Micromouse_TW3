// THE DUTY MIXER (DutyMix.h): speed capped at dmax, steering to dwheel.
#include <cmath>
#include <cstdio>

#include "DutyMix.h"

static int fails = 0;
static void expect(const char *what, bool ok) {
    std::printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) fails++;
}
static bool near(float a, float b) { return std::fabs(a - b) < 1e-5f; }

int main() {
    std::printf("[the old law, dwheel 0]\n");
    DutyMixOut o = duty_mix(1.2f, 1.0f, 0.85f, 0.0f);
    expect("both wheels clipped at 0.85: the steering is gone",
           near(o.left, 0.85f) && near(o.right, 0.85f));

    std::printf("[dwheel 1.0, dmax 0.85]\n");
    o = duty_mix(0.5f, 0.6f, 0.85f, 1.0f);
    expect("inside every limit: untouched", near(o.left, 0.5f) && near(o.right, 0.6f));
    o = duty_mix(1.2f, 1.2f, 0.85f, 1.0f);
    expect("flat out straight: both at 0.85 (the straight-line cap kept)",
           near(o.left, 0.85f) && near(o.right, 0.85f) && near(o.common, 0.85f));
    // The 09-30 MR5: full throttle and a yaw error asking 0.1 of differential.
    o = duty_mix(1.10f, 1.30f, 0.85f, 1.0f);
    expect("full throttle + steering: the differential survives (0.75 / 0.95)",
           near(o.left, 0.75f) && near(o.right, 0.95f));
    expect("...speed still at the cap", near(o.common, 0.85f));
    o = duty_mix(0.9f, 1.5f, 0.85f, 1.0f);
    expect("a steer bigger than the headroom: one wheel at 1.0, speed gives way",
           near(o.right, 1.0f) && near(o.right - o.left, 0.6f) && o.common < 0.85f);
    expect("never both wheels at 1.0", !(near(o.left, 1.0f) && near(o.right, 1.0f)));
    o = duty_mix(-1.3f, -1.1f, 0.85f, 1.0f);
    expect("braking the same, mirrored", near(o.left, -0.95f) && near(o.right, -0.75f));
    o = duty_mix(-1.4f, 1.4f, 0.85f, 1.0f);
    expect("an in-place spin: differential capped at the wheel limit",
           near(o.left, -1.0f) && near(o.right, 1.0f) && near(o.common, 0.0f));

    std::printf("[dwheel at or below dmax: mixed inside dmax]\n");
    o = duty_mix(1.10f, 1.30f, 0.85f, 0.5f);
    expect("the wheel ceiling is never below dmax; steering first inside it",
           near(o.right, 0.85f) && near(o.right - o.left, 0.2f));

    std::printf(fails ? "duty_mix: %d FAILED\n" : "duty_mix: all passed\n", fails);
    return fails ? 1 : 0;
}
