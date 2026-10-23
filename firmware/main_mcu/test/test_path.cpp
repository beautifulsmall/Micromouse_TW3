// SpeedPath: geometry, corner metadata, and the per-tick velocity planner.
//
// Pure math, no Arduino stub needed:
//   g++ -std=c++17 -I../lib/Control ../lib/Control/Path.cpp test_path.cpp \
//       -o test_path && ./test_path
// (or just ./run_tests.sh path)

#include "Path.h"

#include <cmath>
#include <cstdio>

static int failures = 0;

static void expect(const char *what, bool ok) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        failures++;
    }
}

static void expect_near(const char *what, float got, float want, float tol) {
    if (!(std::fabs(got - want) <= tol)) {
        std::printf("FAIL: %s -- got %.4f, want %.4f (tol %.4f)\n", what, got,
                    want, tol);
        failures++;
    }
}

// ---------------------------------------------------------------- geometry
static void test_straight() {
    SpeedPath p;
    p.clear(0.0f, 0.0f, 0.0f);
    expect("straight added", p.add_straight(500.0f));
    expect_near("length", p.length_mm(), 500.0f, 1e-3f);
    expect("one segment", p.seg_count() == 1);
    expect("no corners", p.corner_count() == 0);

    PathPose a = p.pose(0.0f);
    expect_near("start x", a.x_mm, 0.0f, 1e-3f);
    PathPose b = p.pose(250.0f);
    expect_near("mid x", b.x_mm, 250.0f, 1e-3f);
    expect_near("mid y", b.y_mm, 0.0f, 1e-3f);
    expect_near("mid yaw", b.yaw_deg, 0.0f, 1e-3f);
    // Clamped past the end rather than extrapolated.
    PathPose c = p.pose(9999.0f);
    expect_near("clamped x", c.x_mm, 500.0f, 1e-3f);

    // Consecutive straights merge into one segment.
    p.add_straight(100.0f);
    expect("straights merged", p.seg_count() == 1);
    expect_near("merged length", p.length_mm(), 600.0f, 1e-3f);
}

static void test_arc_geometry() {
    const float R = 60.0f;
    SpeedPath p;
    p.clear(0.0f, 0.0f, 0.0f);
    p.add_arc(R, 90.0f);  // +90 = left / CCW

    // Arclength of a quarter circle.
    expect_near("arc length", p.length_mm(), R * M_PI / 2.0f, 1e-2f);

    // Starting at the origin heading +x and curving left, the quarter arc
    // ends at (R, R) heading +y. That is the whole test: if this is wrong,
    // every gate anchor downstream is wrong by the same amount.
    PathPose e = p.pose(p.length_mm());
    expect_near("arc end x", e.x_mm, R, 1e-2f);
    expect_near("arc end y", e.y_mm, R, 1e-2f);
    expect_near("arc end yaw", e.yaw_deg, 90.0f, 1e-2f);

    // Halfway round: 45 degrees, on the circle centred at (0, R).
    PathPose m = p.pose(0.5f * p.length_mm());
    expect_near("arc mid yaw", m.yaw_deg, 45.0f, 1e-2f);
    float dx = m.x_mm - 0.0f, dy = m.y_mm - R;
    expect_near("arc mid on circle", std::sqrt(dx * dx + dy * dy), R, 1e-2f);

    // A right turn mirrors it.
    SpeedPath q;
    q.clear(0.0f, 0.0f, 0.0f);
    q.add_arc(R, -90.0f);
    PathPose re = q.pose(q.length_mm());
    expect_near("right arc end x", re.x_mm, R, 1e-2f);
    expect_near("right arc end y", re.y_mm, -R, 1e-2f);
    expect_near("right arc end yaw", re.yaw_deg, -90.0f, 1e-2f);
    expect("right corner dir", q.corner_dir(0) == -1);
}

static void test_corner_metadata() {
    const float R = 60.0f, lead = 300.0f, exit_ = 300.0f;
    SpeedPath p;
    expect("corner built", p.begin_corner(lead, R, -90.0f, exit_));
    expect("three segments", p.seg_count() == 3);
    expect("one corner", p.corner_count() == 1);
    expect_near("entry s", p.corner_entry_s_mm(0), lead, 1e-3f);
    expect_near("exit s", p.corner_exit_s_mm(0), lead + R * M_PI / 2.0f, 1e-2f);
    expect_near("radius", p.corner_radius_mm(0), R, 1e-3f);
    expect("dir right", p.corner_dir(0) == -1);

    // THE ANCHOR TRAP (Path.h): for a 90-degree arc the spatial tangent
    // length is exactly R, while the arc's PATH length is pi*R/2 = 1.57*R.
    // A gate that confuses the two picks up ~0.2*R of systematic error.
    expect_near("tangent length", p.corner_tangent_mm(0), R, 1e-2f);
    float path_len = p.corner_exit_s_mm(0) - p.corner_entry_s_mm(0);
    float spatial = 2.0f * p.corner_tangent_mm(0);
    expect("arc path shorter than the room it eats", path_len < spatial);
    expect_near("the gap is ~0.2R", spatial - path_len, 0.4292f * R, 0.5f);

    // Total corridor run either side, which is what a route compiler has to
    // budget: lead + R before the corner point, R + exit after it.
    expect_near("approach corridor", lead + p.corner_tangent_mm(0), 360.0f,
                1e-2f);

    // End pose: 300 straight, quarter right, 300 straight, from the origin
    // heading +x, lands at (360, -360) heading -90.
    PathPose e = p.pose(p.length_mm());
    expect_near("corner end x", e.x_mm, 360.0f, 1e-2f);
    expect_near("corner end y", e.y_mm, -360.0f, 1e-2f);
    expect_near("corner end yaw", e.yaw_deg, -90.0f, 1e-2f);

    // Curvature is a STEP at the tangent points -- the known limitation the
    // executor's slew rail exists to bound. Assert it so nobody "fixes" the
    // symptom by smoothing kappa here instead of changing the geometry.
    expect("straight before entry", p.kappa_at(lead - 1.0f) == 0.0f);
    expect("curved after entry", p.kappa_at(lead + 1.0f) != 0.0f);
}

static void test_seg_joints_agree() {
    // Segment start poses are precomputed by walking the path once. Verify
    // each one matches pose() evaluated at the joint from the other side --
    // if these ever diverge, a corner's entry and the preceding straight's
    // end disagree about where the mouse is.
    SpeedPath p;
    p.clear(0.0f, 0.0f, 0.0f);
    p.add_straight(180.0f);
    p.add_arc(60.0f, 90.0f);
    p.add_straight(180.0f);
    p.add_arc(75.0f, -90.0f);
    p.add_straight(90.0f);
    for (int i = 1; i < p.seg_count(); i++) {
        const PathSeg &s = p.seg(i);
        PathPose from_prev = p.pose(s.s0_mm);
        expect_near("joint x", s.x0_mm, from_prev.x_mm, 1e-2f);
        expect_near("joint y", s.y0_mm, from_prev.y_mm, 1e-2f);
    }
}

// ---------------------------------------------------------------- planner
// The speed rung's cap (Route.h, NEVER REFUSE A VALID MAZE): a corner the
// route compiler had to slow carries a ceiling on its curved segments, and
// the planner brakes to it exactly as it brakes to the lateral cap.
static void test_corner_speed_cap() {
    const float R = 60.0f;
    SpeedPath p;
    p.clear(0.0f, 0.0f, 0.0f);
    p.add_straight(600.0f);
    p.set_next_vcap(150.0f);
    p.add_arc(R, -90.0f);
    p.set_next_vcap(0.0f);
    p.add_straight(600.0f);
    p.add_arc(R, -90.0f);          // an uncapped corner after it
    p.add_straight(300.0f);
    p.set_limits(600.0f, 700.0f, 1000.0f);

    float v_lat = std::sqrt(1000.0f * R);   // 245: the lateral cap alone
    float mid0 = 0.5f * (p.corner_entry_s_mm(0) + p.corner_exit_s_mm(0));
    float mid1 = 0.5f * (p.corner_entry_s_mm(1) + p.corner_exit_s_mm(1));
    expect_near("capped arc holds the cap", p.v_cap_at(mid0), 150.0f, 1e-3f);
    expect_near("uncapped arc keeps the lateral cap", p.v_cap_at(mid1), v_lat, 0.5f);
    expect_near("straight between them is uncapped", p.v_cap_at(p.corner_exit_s_mm(0) + 50.0f), 600.0f, 1e-3f);

    // Braking curve into the capped corner aims at 150, not 245.
    float d = 100.0f;
    SpeedPath::Sample s = p.sample(p.corner_entry_s_mm(0) - d, 600.0f, 0.002f);
    float want = std::sqrt(150.0f * 150.0f + 2.0f * 700.0f * d);
    expect_near("brakes to the cap", s.v_mm_s, want, 1.0f);
    SpeedPath::Sample s1 = p.sample(p.corner_entry_s_mm(0) - 1.0f, 600.0f, 0.002f);
    expect_near("arrives at the cap", s1.v_mm_s, 150.0f, 15.0f);
    // And the cap does not leak onto the next corner.
    SpeedPath::Sample s2 = p.sample(p.corner_entry_s_mm(1) - 1.0f, 600.0f, 0.002f);
    expect_near("next corner at its lateral cap", s2.v_mm_s, v_lat, 15.0f);
}

static void test_velocity_caps() {
    const float R = 60.0f;
    SpeedPath p;
    p.begin_corner(600.0f, R, -90.0f, 600.0f);
    p.set_limits(600.0f, 700.0f, 1000.0f);

    // Inside the arc: v <= sqrt(a_lat / kappa) = sqrt(a_lat * R).
    float v_arc = std::sqrt(1000.0f * R);
    float mid = 0.5f * (p.corner_entry_s_mm(0) + p.corner_exit_s_mm(0));
    expect_near("arc pointwise cap", p.v_cap_at(mid), v_arc, 0.5f);
    expect_near("straight cap is v_max", p.v_cap_at(10.0f), 600.0f, 1e-3f);

    // On the approach, well clear of the corner, the ceiling applies.
    SpeedPath::Sample s0 = p.sample(10.0f, 600.0f, 0.002f);
    expect_near("cruise on the lead", s0.v_mm_s, 600.0f, 1e-3f);

    // Just before the arc, braking has taken over and the reference has
    // converged onto the arc cap.
    SpeedPath::Sample s1 =
        p.sample(p.corner_entry_s_mm(0) - 1.0f, 600.0f, 0.002f);
    expect("braked into the corner", s1.v_mm_s < 600.0f);
    expect_near("arrives at the arc cap", s1.v_mm_s, v_arc, 15.0f);

    // The braking curve is exactly sqrt(v_arc^2 + 2*a_lon*d) at a distance
    // where nothing else binds.
    float d = 100.0f;
    SpeedPath::Sample s2 =
        p.sample(p.corner_entry_s_mm(0) - d, 600.0f, 0.002f);
    float want = std::sqrt(v_arc * v_arc + 2.0f * 700.0f * d);
    if (want > 600.0f) want = 600.0f;
    expect_near("braking curve", s2.v_mm_s, want, 1.0f);

    // The end is a v = 0 constraint.
    SpeedPath::Sample s3 = p.sample(p.length_mm(), 100.0f, 0.002f);
    expect_near("stops at the end", s3.v_mm_s, 0.0f, 1e-3f);
    expect("at_end flagged", s3.at_end);

    // Accel ramp: from rest the first tick may only reach a_lon*dt.
    SpeedPath::Sample s4 = p.sample(0.0f, 0.0f, 0.002f);
    expect_near("accel ramp from rest", s4.v_mm_s, 700.0f * 0.002f, 1e-3f);
}

static void test_omega_reference() {
    const float R = 60.0f;
    SpeedPath p;
    p.begin_corner(300.0f, R, 90.0f, 300.0f);  // left
    p.set_limits(400.0f, 700.0f, 1000.0f);
    float mid = 0.5f * (p.corner_entry_s_mm(0) + p.corner_exit_s_mm(0));
    SpeedPath::Sample s = p.sample(mid, 245.0f, 0.002f);
    expect("in_arc", s.in_arc);
    // omega = v/R, positive for a left turn (+CCW), in deg/s.
    expect_near("omega ff", s.omega_dps, s.v_mm_s / R * 57.29578f, 0.5f);
    expect("left turn is +omega", s.omega_dps > 0.0f);

    SpeedPath q;
    q.begin_corner(300.0f, R, -90.0f, 300.0f);
    q.set_limits(400.0f, 700.0f, 1000.0f);
    float qmid = 0.5f * (q.corner_entry_s_mm(0) + q.corner_exit_s_mm(0));
    expect("right turn is -omega",
           q.sample(qmid, 245.0f, 0.002f).omega_dps < 0.0f);

    // Off the path there is no curvature left to command.
    expect_near("no omega past the end", p.sample(p.length_mm() + 5.0f, 0.0f,
                                                  0.002f).omega_dps,
                0.0f, 1e-3f);
}

static void test_forward_simulation() {
    // Drive the planner the way the ISR does -- feed back the previous v,
    // integrate s at the commanded speed -- and check it completes, never
    // exceeds the lateral budget through the arc, and stops at the end.
    const float R = 60.0f, dt = 1.0f / 500.0f;
    SpeedPath p;
    p.begin_corner(500.0f, R, -90.0f, 500.0f);
    p.set_limits(500.0f, 700.0f, 1000.0f);

    float s = 0.0f, v = 0.0f;
    float worst_lat = 0.0f, v_peak = 0.0f;
    int ticks = 0;
    for (; ticks < 20000; ticks++) {
        SpeedPath::Sample smp = p.sample(s, v, dt);
        v = smp.v_mm_s;
        if (v > v_peak) v_peak = v;
        if (smp.in_arc) {
            float lat = v * v / R;
            if (lat > worst_lat) worst_lat = lat;
        }
        s += v * dt;
        if (s >= p.length_mm() && v < 1.0f) break;
    }
    expect("run completed", ticks < 20000);
    expect_near("finished at the path end", s, p.length_mm(), 2.0f);
    expect("lateral budget respected", worst_lat <= 1000.0f * 1.02f);
    expect("reached a useful cruise", v_peak > 400.0f);

    // Sanity on duration: 1094 mm of path, most of it near 500 mm/s, with a
    // slow arc in the middle. Anything outside a few seconds means the
    // planner is not converging the way the ISR would drive it.
    float secs = ticks * dt;
    expect("plausible duration", secs > 1.5f && secs < 6.0f);
}

static void test_capacity_and_rejects() {
    SpeedPath p;
    p.clear(0.0f, 0.0f, 0.0f);
    expect("zero-length straight is a no-op", p.add_straight(0.0f));
    expect("no segment created", p.seg_count() == 0);
    expect("negative radius rejected", !p.add_arc(-10.0f, 90.0f));
    expect("zero angle rejected", !p.add_arc(60.0f, 0.0f));

    // Fill the corner table and confirm it refuses rather than overruns.
    SpeedPath q;
    q.clear(0.0f, 0.0f, 0.0f);
    int made = 0;
    for (int i = 0; i < PATH_MAX_CORNERS + 8; i++) {
        if (q.add_arc(60.0f, (i % 2) ? 90.0f : -90.0f)) made++;
    }
    expect("corner table bounded", made <= PATH_MAX_CORNERS);
    expect("corner count bounded", q.corner_count() <= PATH_MAX_CORNERS);
    expect("segment table bounded", q.seg_count() <= PATH_MAX_SEGS);
}

static void test_ring_closes() {
    // THE test for the loop rig: a ring that does not close walks the start
    // pose a little further round the maze every loop, and that looks exactly
    // like drift in the data it is meant to measure.
    const float SIDE = 900.0f;   // 5 cells, corner point to corner point
    const float FIRST = 360.0f;  // start 2 cells before the first corner
    for (int ri = 0; ri < 3; ri++) {
        const float R[3] = {45.0f, 60.0f, 125.0f};
        for (int dir = 0; dir < 2; dir++) {
            SpeedPath p;
            expect("ring built", p.begin_ring(FIRST, SIDE, R[ri], dir != 0));
            expect("four corners", p.corner_count() == 4);

            // Length: four sides less 8R of straight, plus four quarter arcs.
            float want = 4.0f * SIDE - 8.0f * R[ri] + 4.0f * (R[ri] * M_PI / 2.0f);
            expect_near("ring length", p.length_mm(), want, 0.05f);

            // Closes in position AND heading (heading comes back to 0 mod 360).
            PathPose s = p.pose(0.0f);
            PathPose e = p.pose(p.length_mm());
            expect_near("ring closes x", e.x_mm, s.x_mm, 0.05f);
            expect_near("ring closes y", e.y_mm, s.y_mm, 0.05f);
            expect_near("ring closes yaw", e.yaw_deg, s.yaw_deg, 0.05f);

            // Every corner is the same radius and the same sense.
            for (int k = 0; k < 4; k++) {
                expect_near("corner radius", p.corner_radius_mm(k), R[ri], 1e-3f);
                expect("corner sense", p.corner_dir(k) == (dir ? 1 : -1));
            }
            // Corners are evenly spaced by one side length of spatial run.
            for (int k = 1; k < 4; k++) {
                float gap = p.corner_entry_s_mm(k) - p.corner_exit_s_mm(k - 1);
                expect_near("straight between corners", gap,
                            SIDE - 2.0f * R[ri], 0.05f);
            }
            // First and last straights sum to a full side less 2R -- the
            // loop boundary sits in the middle of a side, not in a corner.
            float first = p.corner_entry_s_mm(0);
            float last = p.length_mm() - p.corner_exit_s_mm(3);
            expect_near("boundary splits a side", first + last,
                        SIDE - 2.0f * R[ri], 0.05f);
        }
    }
}

static void test_ring_gate_anchor() {
    // Where the inside wall ends, in path s, for each corner of the ring.
    // The wall ends at the corner cell's near boundary = 90 mm before the
    // corner point; the beam leads the mouse by 72 mm; the tangent point is
    // R before the corner point. All three are on the approach STRAIGHT,
    // where path s and spatial distance agree -- which is the only reason
    // this is a subtraction and not a projection.
    const float SIDE = 900.0f, CELL = 180.0f, BEAM = 72.0f;
    for (int ri = 0; ri < 3; ri++) {
        const float R[3] = {45.0f, 60.0f, 125.0f};
        SpeedPath p;
        p.begin_ring(360.0f, SIDE, R[ri], false);
        for (int k = 0; k < 4; k++) {
            float entry = p.corner_entry_s_mm(k);
            float expect_s = entry + R[ri] - 0.5f * CELL - BEAM;
            // The gate must fire BEFORE the tangent point, or there is
            // nothing left to correct.
            expect("gate anchor precedes the arc", expect_s < entry);
            // And after the previous arc's exit, or the beam is sweeping.
            if (k > 0) {
                expect("gate anchor follows the previous arc",
                       expect_s > p.corner_exit_s_mm(k - 1));
            }
        }
        // Room to drain shrinks as R grows: R - 162 mm relative to the
        // tangent. At R125 that is only 37 mm, which is why the correction
        // has to be spread over the room available rather than a fixed rate.
        float room = 0.5f * CELL + BEAM - R[ri];
        expect_near("room to the tangent", room, 162.0f - R[ri], 1e-3f);
    }
}

// ---------------------------------------------------------------- clothoids
//
// A clothoid corner exists to remove the curvature STEP at the tangent
// point. Runs 130-135 measured what the step costs -- an exit heading
// overshoot of omega^2/(2*alpha), 13.7 deg at R60/v500 -- and the rail A/B
// that ADR-002 specified in advance came back decisive: halve the ramp,
// halve the error. These tests check the PROPERTIES that has to give,
// not the series expansion that implements it.

static void test_clothoid_off_is_bit_identical() {
    // The whole safety of this change: a path that does not ask for
    // transitions must be exactly the geometry that flew runs 116-135.
    SpeedPath a, b;
    b.set_clothoid_len(0.0f);
    a.begin_ring(360.0f, 900.0f, 80.0f, false);
    b.begin_ring(360.0f, 900.0f, 80.0f, false);
    expect("same segment count", a.seg_count() == b.seg_count());
    expect("same length", std::fabs(a.length_mm() - b.length_mm()) < 1e-4f);
    expect("corner is one segment", a.corner_count() == 4);
    expect("tangent is R for a pure arc",
           std::fabs(a.corner_tangent_mm(0) - 80.0f) < 1e-3f);
}

static void test_clothoid_curvature_is_continuous() {
    // The point of the exercise. Sample across the entry tangent: with a
    // pure arc kappa jumps 0 -> 1/R in one step; with a clothoid it ramps.
    const float R = 60.0f, L = 40.0f;
    SpeedPath step, ramp;
    step.begin_corner(300.0f, R, -90.0f, 300.0f);
    ramp.set_clothoid_len(L);
    ramp.begin_corner(300.0f, R, -90.0f, 300.0f);

    float worst_step = 0.0f, worst_ramp = 0.0f;
    float prev_s = step.kappa_at(0.0f), prev_r = ramp.kappa_at(0.0f);
    for (float s = 1.0f; s < 400.0f; s += 1.0f) {
        float ks = step.kappa_at(s), kr = ramp.kappa_at(s);
        float ds = std::fabs(ks - prev_s), dr = std::fabs(kr - prev_r);
        if (ds > worst_step) worst_step = ds;
        if (dr > worst_ramp) worst_ramp = dr;
        prev_s = ks;
        prev_r = kr;
    }
    std::printf("  worst dkappa per mm: step %.5f  clothoid %.5f\n",
                (double)worst_step, (double)worst_ramp);
    expect("the pure arc really does step the full 1/R",
           std::fabs(worst_step - 1.0f / R) < 1e-4f);
    // Over L mm the ramp spreads 1/R, so each 1 mm step is 1/(R*L).
    expect("the clothoid spreads it over its length",
           worst_ramp < 1.5f / (R * L));
    expect("and that is a big reduction", worst_ramp * 10.0f < worst_step);
}

static void test_clothoid_still_turns_ninety_and_closes() {
    // A transition that quietly changed the turn angle, or left the ring
    // open, would look exactly like drift on the floor -- which is why the
    // pure-arc ring has always asserted closure.
    const float SIDE = 900.0f;
    for (int ri = 0; ri < 3; ri++) {
        const float R[3] = {60.0f, 80.0f, 100.0f};
        for (int li = 0; li < 3; li++) {
            const float L[3] = {20.0f, 40.0f, 60.0f};
            for (int dir = 0; dir < 2; dir++) {
                SpeedPath p;
                p.set_clothoid_len(L[li]);
                expect("clothoid ring built",
                       p.begin_ring(360.0f, SIDE, R[ri], dir != 0));
                expect("four corners", p.corner_count() == 4);
                PathPose a = p.pose(0.0f), b = p.pose(p.length_mm());
                float dx = a.x_mm - b.x_mm, dy = a.y_mm - b.y_mm;
                expect("ring closes", std::sqrt(dx * dx + dy * dy) < 0.05f);
                // Each corner sweeps exactly 90, clothoids included.
                for (int k = 0; k < 4; k++) {
                    float y0 = p.pose(p.corner_entry_s_mm(k)).yaw_deg;
                    float y1 = p.pose(p.corner_exit_s_mm(k)).yaw_deg;
                    float d = y1 - y0;
                    while (d > 180.0f) d -= 360.0f;
                    while (d < -180.0f) d += 360.0f;
                    expect_near("corner sweeps 90", std::fabs(d), 90.0f, 0.05f);
                }
            }
        }
    }
}

static void test_clothoid_eats_more_corridor() {
    // T = R + m + x0 > R. This is the cost, and it is the reason the gate
    // must read corner_tangent_mm() rather than assuming R -- an assumption
    // that would put every gate expectation T-R too early.
    const float R = 60.0f;
    float t0 = SpeedPath::tangent_for(R, 0.0f, -90.0f);
    expect_near("no clothoid, T is R", t0, R, 1e-3f);
    float prev = t0;
    for (float L = 10.0f; L <= 70.0f; L += 10.0f) {
        float t = SpeedPath::tangent_for(R, L, -90.0f);
        expect("T grows with the transition", t > prev);
        prev = t;
    }
    std::printf("  R60: T = %.1f at L=0, %.1f at L=40\n", (double)t0,
                (double)SpeedPath::tangent_for(R, 40.0f, -90.0f));

    // And the built path agrees with the closed form -- if these ever
    // disagree the ring stops closing, so this is the cheap early warning.
    SpeedPath p;
    p.set_clothoid_len(40.0f);
    p.begin_ring(360.0f, 900.0f, R, false);
    expect_near("built path matches tangent_for()", p.corner_tangent_mm(0),
                SpeedPath::tangent_for(R, 40.0f, -90.0f), 1e-3f);
}

static void test_clothoid_sizing_policy() {
    // RAIL: the shortest transition the slew rail can already follow,
    // L = v^2/(R*alpha). Shorter and the ramp stays rate-limited.
    float l = SpeedPath::clothoid_for_rail(60.0f, 500.0f, 6000.0f);
    expect_near("rail-sized L at R60/v500/slew6000", l, 39.8f, 0.5f);
    expect("a lazier rail wants a longer transition",
           SpeedPath::clothoid_for_rail(60.0f, 500.0f, 2500.0f) > l);
    expect("more speed wants a longer transition",
           SpeedPath::clothoid_for_rail(60.0f, 600.0f, 6000.0f) > l);

    // ROOM: the landmark budget. The inside wall ends 90 mm before the
    // corner point and the beam leads ~45, so T must fit inside ~135 less
    // whatever the gate needs to spread its correction over.
    const float T_MAX = 135.0f - 20.0f;
    expect("R125 has no room for a transition at all",
           SpeedPath::clothoid_for_room(125.0f, T_MAX, -90.0f) == 0.0f);
    float r80 = SpeedPath::clothoid_for_room(80.0f, T_MAX, -90.0f);
    float r60 = SpeedPath::clothoid_for_room(60.0f, T_MAX, -90.0f);
    std::printf("  room-limited L: R80 %.1f  R60 %.1f  R125 0.0\n",
                (double)r80, (double)r60);
    expect("R80 has room", r80 > 20.0f);
    // THE USEFUL PROPERTY: the budget grows as the radius tightens, which is
    // exactly where the overshoot is worst. R125 needs no transition (0.84
    // deg of overshoot at slew 6000) and has none available; R60 needs 13.7
    // deg worth and has room for it.
    expect("the budget grows as the radius tightens", r60 > r80);

    // Whatever the policy picks must actually fit.
    for (int i = 0; i < 3; i++) {
        const float R[3] = {60.0f, 80.0f, 100.0f};
        float L = SpeedPath::clothoid_for_room(R[i], T_MAX, -90.0f);
        expect("room-limited L really fits the budget",
               SpeedPath::tangent_for(R[i], L, -90.0f) <= T_MAX + 1e-3f);
    }
}

static void test_clothoid_pose_matches_numeric_integration() {
    // pose() through a ramp is Simpson over theta(u); an independent
    // fine-grained integration must land in the same place, or every
    // downstream pose is quietly wrong.
    SpeedPath p;
    p.set_clothoid_len(50.0f);
    p.begin_corner(200.0f, 70.0f, -90.0f, 200.0f);
    float x = 0.0f, y = 0.0f, th = 0.0f;
    const float h = 0.02f;
    float target = p.length_mm() - 1.0f;
    for (float s = 0.0f; s < target; s += h) {
        float k = p.kappa_at(s + 0.5f * h);
        th += k * h;
        x += h * std::cos(th - 0.5f * k * h);
        y += h * std::sin(th - 0.5f * k * h);
    }
    PathPose e = p.pose(target);
    std::printf("  pose vs fine integration: dx %.3f dy %.3f mm\n",
                (double)(e.x_mm - x), (double)(e.y_mm - y));
    expect("pose x agrees", std::fabs(e.x_mm - x) < 0.5f);
    expect("pose y agrees", std::fabs(e.y_mm - y) < 0.5f);
}

// THE CLOTHOID HOLD (Path.h, set_clothoid_hold; the 2026-09-26 MR5 crash).
// A fast straight into a clothoid corner: with the hold ON the commanded
// speed must already be the arc's at the ramp-in START and stay there
// through both ramps; with it OFF the planner is still braking inside the
// ramp-in (the old behaviour, the one that met the lateral load coming on).
static void test_clothoid_hold() {
    const float R = 72.0f, L = 40.0f, A_LAT = 4000.0f, A_BRK = 4000.0f;
    const float v_arc = std::sqrt(A_LAT * R);   // 537 (v^2 kappa = a_lat)
    for (int hold = 0; hold < 2; hold++) {
        SpeedPath p;
        p.clear(0, 0, 0);
        p.set_clothoid_len(L);
        p.set_limits(2000.0f, 4000.0f, A_LAT, A_BRK);
        p.set_clothoid_hold(hold == 1);
        p.add_straight(600.0f);
        p.add_arc(R, 90.0f);
        p.add_straight(400.0f);
        // Find the ramp-in and ramp-out segments.
        float in0 = -1, in1 = -1, out0 = -1, out1 = -1;
        for (int i = 0; i < p.seg_count(); i++) {
            const PathSeg &sg = p.seg(i);
            if (sg.kappa != sg.kappa1) {
                if (std::fabs(sg.kappa1) > std::fabs(sg.kappa)) { in0 = sg.s0_mm; in1 = sg.s0_mm + sg.len_mm; }
                else { out0 = sg.s0_mm; out1 = sg.s0_mm + sg.len_mm; }
            }
        }
        expect("clothoid segments found", in0 >= 0 && out0 >= 0);
        // Walk the planner at 1 kHz from rest.
        float v_prev = 0.0f, s = 0.0f, dt = 0.001f;
        float v_at_in0 = -1, vmax_in = 0, vmax_out = 0, v_at_out1 = -1;
        while (s < p.length_mm() - 1.0f) {
            SpeedPath::Sample sm = p.sample(s, v_prev, dt);
            if (v_at_in0 < 0 && s >= in0) v_at_in0 = sm.v_mm_s;
            if (s >= in0 && s <= in1 && sm.v_mm_s > vmax_in) vmax_in = sm.v_mm_s;
            if (s >= out0 && s <= out1 && sm.v_mm_s > vmax_out) vmax_out = sm.v_mm_s;
            if (v_at_out1 < 0 && s >= out1) v_at_out1 = sm.v_mm_s;
            v_prev = sm.v_mm_s;
            s += v_prev * dt;
            if (v_prev < 1.0f) s += 0.5f;   // never stall the walk
        }
        std::printf("  clothoid hold %s: v at ramp-in start %.0f, max in ramp-in %.0f, max in ramp-out %.0f, arc %.0f\n",
                    hold ? "ON " : "OFF", v_at_in0, vmax_in, vmax_out, v_arc);
        if (hold) {
            expect("hold ON: arc speed reached by the ramp-in start", v_at_in0 <= v_arc + 5.0f);
            expect("hold ON: no brake inside the ramp-in", vmax_in <= v_arc + 5.0f);
            expect("hold ON: no launch inside the ramp-out", vmax_out <= v_arc + 5.0f);
        } else {
            expect("hold OFF: still braking inside the ramp-in (the old planner)", v_at_in0 > v_arc + 50.0f);
        }
    }
}

int main() {
    test_clothoid_hold();
    test_straight();
    test_arc_geometry();
    test_corner_metadata();
    test_seg_joints_agree();
    test_velocity_caps();
    test_corner_speed_cap();
    test_omega_reference();
    test_forward_simulation();
    test_ring_closes();
    test_ring_gate_anchor();
    test_clothoid_off_is_bit_identical();
    test_clothoid_curvature_is_continuous();
    test_clothoid_still_turns_ninety_and_closes();
    test_clothoid_eats_more_corridor();
    test_clothoid_sizing_policy();
    test_clothoid_pose_matches_numeric_integration();
    test_capacity_and_rejects();

    if (failures == 0) {
        std::printf("ALL PATH TESTS PASSED\n");
        return 0;
    }
    std::printf("%d PATH TEST(S) FAILED\n", failures);
    return 1;
}
