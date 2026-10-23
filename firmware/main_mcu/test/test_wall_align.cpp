// Host-side test: wall-referenced IMU alignment (lib/Control/WallAlign.*).
// Synthesizes a run beside a wall with a known heading error, integer-mm
// quantisation (the real sensor noise floor), and a missing-wall-segment
// spike (posts are flush with the walls and cause no disturbance -- only an
// absent wall does), and checks the fit recovers the heading error, the
// lateral offset, and rejects the spike.
//
// Build:
//   g++ -std=c++17 -I../lib/Control ../lib/Control/WallAlign.cpp test_wall_align.cpp -o test_wall_align && ./test_wall_align

#include "WallAlign.h"

#include <cmath>
#include <cstdio>

static int failures = 0;
static void expect(const char *what, bool cond) {
    if (!cond) { std::printf("FAIL: %s\n", what); failures++; }
}

static constexpr float DEG2RAD_T = 0.017453292519943295f;

int main() {
    // -- Clean run: 2 deg heading error, 5 mm start offset, 330 mm of wall
    // (the boot double-wall span), samples every 5 mm, quantised to 1 mm.
    {
        WallAlign wa;
        wa.reset();
        float eps = 2.0f * DEG2RAD_T;
        for (float d = 0.0f; d <= 330.0f; d += 5.0f) {
            float lat = 5.0f + std::sin(eps) * d;
            lat = std::round(lat);  // integer-mm quantisation
            wa.add(d, lat);
        }
        expect("clean: valid", wa.valid());
        expect("clean: yaw within 0.2 deg",
               std::fabs(wa.yaw_err_deg() - 2.0f) < 0.2f);
        expect("clean: offset at end within 1 mm",
               std::fabs(wa.offset_mm(330.0f) - (5.0f + std::sin(eps) * 330.0f)) < 1.0f);
        expect("clean: nothing rejected", wa.rejected() == 0);
    }

    // -- Negative error: drifting toward the right wall.
    {
        WallAlign wa;
        wa.reset();
        float eps = -1.5f * DEG2RAD_T;
        for (float d = 0.0f; d <= 200.0f; d += 5.0f) {
            wa.add(d, std::round(-3.0f + std::sin(eps) * d));
        }
        expect("neg: valid", wa.valid());
        expect("neg: yaw within 0.25 deg",
               std::fabs(wa.yaw_err_deg() + 1.5f) < 0.25f);
    }

    // -- Missing wall segment: the beam sees the next corridor's wall for a
    // stretch; the spiked laterals must be rejected and not disturb the fit.
    {
        WallAlign wa;
        wa.reset();
        float eps = 1.0f * DEG2RAD_T;
        int spikes = 0;
        for (float d = 0.0f; d <= 330.0f; d += 5.0f) {
            float lat = std::sin(eps) * d;
            if (d >= 150.0f && d < 180.0f) {  // absent wall: far-wall return
                lat += 25.0f;
                spikes++;
            }
            wa.add(d, std::round(lat));
        }
        expect("gap: valid", wa.valid());
        expect("gap: spikes rejected", wa.rejected() == spikes);
        expect("gap: yaw within 0.25 deg",
               std::fabs(wa.yaw_err_deg() - 1.0f) < 0.25f);
    }

    // -- Too short a span: not valid, even with many samples.
    {
        WallAlign wa;
        wa.reset();
        for (float d = 0.0f; d <= 50.0f; d += 1.0f) wa.add(d, 2.0f);
        expect("short: invalid", !wa.valid());
    }

    // -- Perfectly parallel run: slope ~0.
    {
        WallAlign wa;
        wa.reset();
        for (float d = 0.0f; d <= 200.0f; d += 5.0f) wa.add(d, 7.0f);
        expect("parallel: valid", wa.valid());
        expect("parallel: yaw ~0", std::fabs(wa.yaw_err_deg()) < 0.05f);
        expect("parallel: offset ~7", std::fabs(wa.offset_mm(200.0f) - 7.0f) < 0.5f);
    }

    // -- THE SWEEP'S CORRIDOR-SQUARING LAW, which is the bit that is easy to
    // get wrong. The sweep is not measuring a heading error to correct; it
    // wants the CORRIDOR'S BEARING in the fused frame, so that the next
    // loop's path can be built on the corridor instead of on the body.
    //
    // The law is corridor = yaw_bar - eps, with both terms averaged over the
    // SAME samples. The trap it has to survive is that the lateral servo is
    // steering throughout the fit, so the mouse is converging: its heading
    // is NOT constant, and eps is not a static misalignment. Taking yaw at
    // one instant would measure the servo's transient instead.
    //
    // Simulated here: a corridor at a true bearing of +37.4 deg in the fused
    // frame, and a mouse starting 8 mm off centre with a 3 deg heading error
    // that the servo bleeds out linearly over the leg. Both the lateral and
    // the heading move the whole way; the recovered bearing must still be
    // +37.4.
    {
        const float TRUE_CORRIDOR = 37.4f;
        const float DEG = 3.14159265f / 180.0f;
        WallAlign wa;
        wa.reset();
        float yaw_ref = 0.0f, yaw_sum = 0.0f;
        int yaw_n = 0;
        float lat = 8.0f;
        bool first = true;
        for (float d = 0.0f; d <= 300.0f; d += 3.0f) {
            // Heading error to the corridor, decaying 3 -> 0 across the leg.
            float err = 3.0f * (1.0f - d / 300.0f);
            float yaw = TRUE_CORRIDOR + err;
            if (first) { yaw_ref = yaw; first = false; }
            wa.add(d, lat);
            yaw_sum += yaw - yaw_ref;
            yaw_n++;
            lat += std::sin(err * DEG) * 3.0f;   // travel closes on the wall
        }
        expect("sweep law: fit valid", wa.valid());
        float yaw_bar = yaw_ref + yaw_sum / yaw_n;
        float corridor = yaw_bar - wa.yaw_err_deg();
        expect("sweep law: corridor bearing recovered within 0.1 deg",
               std::fabs(corridor - TRUE_CORRIDOR) < 0.1f);
        // And pin that the average is doing real work. The naive version --
        // yaw at ONE instant, here the last sample, where the mouse has
        // already converged and yaw reads the corridor exactly -- lands a
        // full eps away from the truth. Without this the test would still
        // pass if someone "simplified" yaw_bar down to a single reading and
        // the sim happened to be symmetric.
        float yaw_last = TRUE_CORRIDOR;   // err has decayed to 0 by d = 300
        float naive = yaw_last - wa.yaw_err_deg();
        expect("sweep law: a one-instant yaw is wrong by the servo transient",
               std::fabs(naive - TRUE_CORRIDOR) > 1.0f);
    }

    if (failures == 0) { std::printf("ALL WALL-ALIGN TESTS PASSED\n"); return 0; }
    std::printf("%d WALL-ALIGN TEST(S) FAILED\n", failures);
    return 1;
}
