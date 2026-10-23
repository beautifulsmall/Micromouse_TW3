// Host-side test: trapezoidal motion profile generator (Profile.h). Pure math,
// no HAL/Arduino stubs needed. Build:
//
//   g++ -std=c++17 -I../lib/Control ../lib/Control/Profile.cpp test_profile.cpp -o test_profile && ./test_profile

#include "Profile.h"

#include <cassert>
#include <cmath>
#include <cstdio>

static int failures = 0;
static void expect(const char *what, bool cond) {
    if (!cond) {
        std::printf("FAIL: %s\n", what);
        failures++;
    }
}

// Numerically integrate a_ref -> v_ref -> d_ref over the whole profile and
// confirm internal consistency plus the endpoint distance. dt = 0.1 ms.
static void check_consistency(const char *tag, const TrapProfile &p,
                              float expect_dist, float expect_peak) {
    const float dt = 0.0001f;
    float v_int = 0.0f, d_int = 0.0f, prev_d = 0.0f, peak = 0.0f;
    bool monotonic = true;
    for (float t = 0.0f; t <= p.duration_s() + 0.5f; t += dt) {
        ProfileSample s = p.sample(t);
        // Integral of commanded accel should track commanded velocity.
        if (t < p.duration_s()) {
            v_int += s.a_ref * dt;
            d_int += s.v_ref * dt;
        }
        if (std::fabs(s.v_ref) > peak) peak = std::fabs(s.v_ref);
        // Distance magnitude must never retreat (holds for either direction).
        if (std::fabs(s.d_ref) + 1e-3f < std::fabs(prev_d)) monotonic = false;
        prev_d = s.d_ref;
    }
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s: a_ref integrates to v_ref", tag);
    expect(buf, std::fabs(v_int) < 5.0f);  // ends back at rest
    std::snprintf(buf, sizeof(buf), "%s: v_ref integrates to distance", tag);
    expect(buf, std::fabs(std::fabs(d_int) - std::fabs(expect_dist)) < 2.0f);
    std::snprintf(buf, sizeof(buf), "%s: endpoint d_ref == target", tag);
    expect(buf, std::fabs(p.sample(p.duration_s()).d_ref - expect_dist) < 1e-2f);
    std::snprintf(buf, sizeof(buf), "%s: peak velocity", tag);
    expect(buf, std::fabs(peak - std::fabs(expect_peak)) < 1.0f);
    std::snprintf(buf, sizeof(buf), "%s: d_ref monotonic in move direction", tag);
    expect(buf, monotonic);
}

int main() {
    // -- Full trapezoid: reaches cruise, holds, decelerates ---------------------
    {
        TrapProfile p;
        p.begin(720.0f, 400.0f, 500.0f);
        // t_acc = 0.8 s, cruise dist = 400 mm -> t_cruise = 1.0 s, total 2.6 s.
        expect("trap duration ~2.6 s", std::fabs(p.duration_s() - 2.6f) < 1e-3f);
        expect("trap peak == cruise", std::fabs(p.peak_velocity_mm_s() - 400.0f) < 1e-3f);
        ProfileSample mid = p.sample(0.4f);  // mid accel
        expect("accel phase a_ref", std::fabs(mid.a_ref - 500.0f) < 1e-2f);
        expect("accel phase v_ref", std::fabs(mid.v_ref - 200.0f) < 1e-1f);
        ProfileSample cruise = p.sample(1.3f);
        expect("cruise a_ref 0", std::fabs(cruise.a_ref) < 1e-3f);
        expect("cruise v_ref", std::fabs(cruise.v_ref - 400.0f) < 1e-2f);
        ProfileSample decel = p.sample(2.2f);  // 0.4 s into decel
        expect("decel a_ref negative", decel.a_ref < -1.0f);
        expect("decel v_ref", std::fabs(decel.v_ref - 200.0f) < 1e-1f);
        expect("done at end", p.sample(2.6f).done);
        expect("not done mid-run", !p.sample(1.0f).done);
        check_consistency("trap", p, 720.0f, 400.0f);
    }

    // -- Triangle: too short to reach cruise ------------------------------------
    {
        TrapProfile p;
        p.begin(100.0f, 400.0f, 500.0f);
        float vp = std::sqrt(500.0f * 100.0f);  // ~223.6 mm/s
        expect("triangle peak < cruise", p.peak_velocity_mm_s() < 400.0f);
        expect("triangle peak == sqrt(a*d)",
               std::fabs(p.peak_velocity_mm_s() - vp) < 1e-1f);
        check_consistency("triangle", p, 100.0f, vp);
    }

    // -- Negative distance: mirror of the positive case -------------------------
    {
        TrapProfile p;
        p.begin(-300.0f, 400.0f, 500.0f);
        expect("negative target", std::fabs(p.target_mm() + 300.0f) < 1e-3f);
        expect("negative peak sign", p.peak_velocity_mm_s() < 0.0f);
        ProfileSample s = p.sample(0.2f);
        expect("negative v_ref", s.v_ref < 0.0f);
        expect("negative a_ref", s.a_ref < 0.0f);
        check_consistency("neg", p, -300.0f, p.peak_velocity_mm_s());
    }

    // -- Degenerate requests park immediately -----------------------------------
    {
        TrapProfile p;
        p.begin(0.0f, 400.0f, 500.0f);
        expect("zero-dist done", p.sample(0.0f).done);
        expect("zero-dist duration 0", p.duration_s() == 0.0f);

        TrapProfile q;
        q.begin(300.0f, 400.0f, 0.0f);  // no accel -> degenerate
        expect("zero-accel done", q.sample(0.0f).done);
    }

    // -- KaTuner: recovers the ka a run needed from ramp tracking error ---------
    {
        const float kp = 0.0004f;
        const float ka_true = 0.00025f;  // duty per mm/s^2 the plant needs
        const float ka_now = 0.0f;       // gain the simulated run ran with

        TrapProfile p;
        p.begin(720.0f, 400.0f, 500.0f);

        // Simulate the 50 Hz sampling of the 'a' test. Steady-state error
        // model: the P term supplies the missing accel duty, so
        // err = (ka_true - ka_now) * a_ref / kp. Add a disturbance that
        // correlates with velocity (a vmax/friction mismatch would look
        // like this) -- the two-ramp fit must reject it -- plus
        // deterministic "encoder noise".
        KaTuner tuner;
        int i = 0;
        for (float t = 0.0f; t < p.duration_s(); t += 0.02f, i++) {
            ProfileSample s = p.sample(t);
            if (std::fabs(s.a_ref) < 100.0f) continue;  // ramps only
            float err = (ka_true - ka_now) * s.a_ref / kp;
            err += 0.05f * s.v_ref;              // v-correlated model error
            err += ((i % 3) - 1.0f) * 8.0f;      // +/-8 mm/s pseudo-noise
            tuner.add(s.a_ref, err);
        }

        expect("tuner valid after run", tuner.valid());
        float ka_fit = tuner.suggest(ka_now, kp);
        std::printf("KaTuner: true %.6f fit %.6f (n=%d)\n", ka_true, ka_fit,
                    tuner.samples());
        expect("tuner recovers ka within 10%",
               std::fabs(ka_fit - ka_true) < 0.1f * ka_true);

        // Second run at the fitted gain: suggestion should barely move.
        KaTuner tuner2;
        for (float t = 0.0f; t < p.duration_s(); t += 0.02f) {
            ProfileSample s = p.sample(t);
            if (std::fabs(s.a_ref) < 100.0f) continue;
            float err = (ka_true - ka_fit) * s.a_ref / kp + 0.05f * s.v_ref;
            tuner2.add(s.a_ref, err);
        }
        float ka_fit2 = tuner2.suggest(ka_fit, kp);
        expect("second run converged",
               std::fabs(ka_fit2 - ka_true) < 0.02f * ka_true);

        // Too few samples -> not valid.
        KaTuner sparse;
        for (int k = 0; k < 5; k++) sparse.add(500.0f, 10.0f);
        expect("sparse tuner invalid", !sparse.valid());
    }

    // -- TrackTuner: recover a known effective track from synthetic turn data --
    {
        const float DEG2RAD = 0.017453292519943295f;
        const float true_track = 63.5f;  // effective (scrub) > 59 mm geometric
        KaTuner unused;
        (void)unused;
        TrackTuner tt;
        tt.reset();
        expect("track: empty is invalid", !tt.valid());
        // Synthesize a turn: various yaw rates, wheel_diff = omega * track/2,
        // plus a little symmetric noise that least-squares should reject.
        for (int i = 0; i < 200; i++) {
            float omega_dps = 60.0f + (i % 40) * 3.0f;  // 60..177 deg/s
            float omega = omega_dps * DEG2RAD;
            float noise = ((i % 2) ? 1.0f : -1.0f) * 0.5f;  // zero-mean
            float wheel_diff = omega * (true_track * 0.5f) + noise;
            tt.add(omega, wheel_diff);
        }
        expect("track: valid after samples", tt.valid());
        expect("track: recovers effective track",
               std::fabs(tt.suggest_track_mm() - true_track) < 1.0f);
    }

    // -- extend(): continuous multi-cell legs (maze search) --------------------
    {
        // One maze cell at search speed: 180 mm at 300/500 peaks exactly at
        // cruise. Extending during accel inserts cruise time; velocity is
        // continuous and the integral lands exactly on the new target.
        TrapProfile p;
        p.begin(180.0f, 300.0f, 500.0f);
        float dur1 = p.duration_s();
        expect("extend: ok during accel", p.extend(180.0f, 0.548f));
        expect("extend: target 360", std::fabs(p.target_mm() - 360.0f) < 0.01f);
        expect("extend: duration +0.6 s",
               std::fabs(p.duration_s() - (dur1 + 0.6f)) < 0.01f);
        ProfileSample a = p.sample(0.599f), b = p.sample(0.601f);
        expect("extend: velocity continuous at old decel point",
               std::fabs(a.v_ref - b.v_ref) < 2.0f);
        float d = 0.0f;
        for (float t = 0.0f; t < p.duration_s(); t += 0.0005f)
            d += p.sample(t).v_ref * 0.0005f;
        expect("extend: v integrates to 360", std::fabs(d - 360.0f) < 1.0f);

        TrapProfile q;
        q.begin(180.0f, 300.0f, 500.0f);
        expect("extend: refused in decel", !q.extend(180.0f, 0.9f));
        expect("extend: unchanged after refusal",
               std::fabs(q.target_mm() - 180.0f) < 0.01f);

        TrapProfile r;  // chain 4 cells one at a time, like the search does
        r.begin(180.0f, 300.0f, 500.0f);
        expect("extend: chain 1", r.extend(180.0f, 0.5f));
        expect("extend: chain 2", r.extend(180.0f, 1.0f));
        expect("extend: chain 3", r.extend(180.0f, 1.5f));
        expect("extend: chained target 720",
               std::fabs(r.target_mm() - 720.0f) < 0.01f);
        float d2 = 0.0f;
        for (float t = 0.0f; t < r.duration_s(); t += 0.0005f)
            d2 += r.sample(t).v_ref * 0.0005f;
        expect("extend: chained v integrates to 720",
               std::fabs(d2 - 720.0f) < 1.5f);
    }

    if (failures == 0) {
        std::printf("ALL PROFILE TESTS PASSED\n");
        return 0;
    }
    std::printf("%d PROFILE TEST(S) FAILED\n", failures);
    return 1;
}
