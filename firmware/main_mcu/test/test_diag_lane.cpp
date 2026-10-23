// THE V EXIT LANE CHANGE (DiagLane.h): the geometry that makes a V land
// vout outside its exit line whatever the entry did, the diagonal's whole
// lateral set point, and plant runs of the dead-reckoned lane change on the
// three shapes the 2026-09-30 laps flew: a lone V, a V with a 45-out 82 mm
// after it, and two same-hand Vs 159 mm apart.
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "DiagLane.h"

static int fails = 0;
static void expect(const char *what, bool ok) {
    std::printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) fails++;
}

// A LEFT V, post at the origin, entry heading +x, exit heading +y. The entry
// line sits (64 + e) to the right of the post (e = how far the mouse really
// is outside the centre line, + outward); the compiled arc, radius 64 + vout,
// starts where the path says (tangent to the OFFSET line at x = 0) and the
// mouse flies it from wherever it is. Returns the exit (lateral outward of
// the exit centre line x = 64, along the exit line relative to the planned
// arc end).
static void v_exit(float vout, float e, float *lat_out, float *along) {
    const float R = 64.0f + vout;
    float cx = 0.0f, cy = -(64.0f + e) + R;
    float xe = cx + R, ye = cy;
    *lat_out = xe - 64.0f;
    *along = ye - 0.0f;
}

static void test_the_v_lands_vout_outside_whatever_the_entry() {
    std::printf("[the V's exit, by geometry]\n");
    for (float vout : {0.0f, 10.0f, 15.0f}) {
        for (float e : {0.0f, 5.0f, 10.0f, -8.0f}) {
            float lat, along;
            v_exit(vout, e, &lat, &along);
            expect("exit lateral is vout, whatever the entry lateral",
                   std::fabs(lat - vout) < 1e-3f);
            expect("the entry lateral becomes an ALONG error (vout - e)",
                   std::fabs(along - (vout - e)) < 1e-3f);
        }
    }
    // ...and the TIMING of the arc: fired early by a, the whole arc moves a
    // back along the ENTRY heading -- perpendicular to the exit. The exit
    // moves a to the inside and not at all along: the path-s lead the lever
    // gave the V must be handed back at its arc end (RouteRun), or it rides
    // on into every corner after it (09-30 MR5: (4,4) turned ~40 mm early).
    const float a = 12.0f;
    float ex = 64.0f + 10.0f, ey = 0.0f;           // planned exit point (vout 10)
    float ex_e = ex - a, ey_e = ey;                // translated by -a along +x
    expect("early V: exit moves inside by a", std::fabs((ex - ex_e) - a) < 1e-4f);
    expect("early V: exit does not move along the exit line", std::fabs(ey_e - ey) < 1e-4f);
}

static void test_the_set_point() {
    std::printf("[the set point]\n");
    VLane ln;
    ln.exit_v = true; ln.exit_s0 = 0.0f; ln.exit_off = -10.0f;
    expect("full offset at the arc end", std::fabs(ln.sp(0.0f) + 10.0f) < 1e-4f);
    expect("gone at the ramp's end", ln.sp(V_EXIT_RAMP_MM) == 0.0f);
    expect("~1 mm of 10 left 72 mm out", std::fabs(ln.sp(72.0f)) < 1.0f);
    expect("slope points back at the line", ln.slope(45.0f) > 0.0f);
    ln.next_arc_s = 82.0f; ln.fit_exit();
    expect("a 45-out 82 mm on: the ramp ends 10 mm before it (72)",
           std::fabs(ln.exit_len - 72.0f) < 0.01f);
    ln.next_arc_s = 30.0f; ln.fit_exit();
    expect("...never under the minimum", ln.exit_len == V_EXIT_RAMP_MIN_MM);
    VLane vv;
    vv.exit_v = true; vv.exit_s0 = 0.0f; vv.exit_off = -10.0f;
    vv.entry_v = true; vv.entry_arc_s = 159.0f; vv.entry_off = -10.0f;
    vv.next_arc_s = 159.0f; vv.fit_exit();
    float hi = -1e9f;
    for (float x = 0.0f; x <= 159.0f; x += 1.0f)
        if (vv.sp(x) > hi) hi = vv.sp(x);
    std::printf("    two left Vs 159 mm apart: set point never above %+.1f\n", (double)hi);
    expect("V-V the same way: the set point stays outside (below -4)", hi < -4.0f);
}

static void test_the_dead_reckoning() {
    std::printf("[the dead reckoning]\n");
    DiagDR dr;
    dr.step(10.0f, 5.0f);
    expect("unseeded: does nothing", !dr.valid && dr.lat == 0.0f);
    dr.seed(-10.0f, 100.0f, true);
    for (float s = 101.0f; s <= 200.0f; s += 1.0f) dr.step(s, 2.0f);
    expect("2 deg left over 100 mm is +3.5 mm", std::fabs(dr.lat - (-10.0f + 3.49f)) < 0.05f);
    float before = dr.lat;
    dr.step(150.0f, 30.0f);
    dr.step(260.0f, 30.0f);
    expect("backwards and absurd steps are not integrated", dr.lat == before);
    dr.clear();
    expect("clear", !dr.valid && !dr.from_v);
}

// The plant, FITTED to the 2026-09-30 MR5 first-V exit (the old law replayed
// against the logged speeds reproduces the logged heading to 0.7 deg RMS:
// +10 at d 85, -4 at d 150): bias slewed at 120 deg/s, yaw-rate demand =
// bias rate + (bias - heading) / 30 ms, delivered through a 40 ms lag.
struct Run { float lat_at[2]; float head_at_arc, lat_at_arc, sp_at_arc, max_in, max_head; };
static Run fly(const VLane &ln, float v, float yaw0_deg, bool servo, float tau_s,
               float probe0, float probe1) {
    DiagDR dr;
    dr.seed(ln.sp(ln.exit_s0), ln.exit_s0, true);
    float lat = ln.sp(ln.exit_s0), head = yaw0_deg, bias_app = 0.0f, wrate = 0.0f;
    Run r = {{0, 0}, 0, 0, 0, -1e9f, 0};
    const float ds = 0.5f, dt = ds / v;
    const float end = ln.next_arc_s;
    for (float d = ln.exit_s0; d < end; d += ds) {
        float cmd = servo ? v_lane_bias_deg(ln, d, v, dr.lat, V_LANE_K_DEG_PER_MM, 10.0f) : 0.0f;
        float db = cmd - bias_app, dbmax = 120.0f * dt;
        if (db > dbmax) db = dbmax;
        if (db < -dbmax) db = -dbmax;
        bias_app += db;
        float u = db / dt + (bias_app - head) / 0.03f;
        wrate += (u - wrate) * (dt / tau_s);
        head += wrate * dt;
        lat += ds * std::sin(head * 0.01745329f);
        dr.step(d + ds, head);
        float x = d + ds;
        if (std::fabs(x - probe0) < 0.26f) r.lat_at[0] = lat;
        if (std::fabs(x - probe1) < 0.26f) r.lat_at[1] = lat;
        if (lat - ln.sp(x) > r.max_in) r.max_in = lat - ln.sp(x);   // + = inside (left V)
        if (std::fabs(head) > r.max_head) r.max_head = std::fabs(head);
    }
    r.head_at_arc = head;
    r.lat_at_arc = lat;
    r.sp_at_arc = ln.sp(end);
    return r;
}

static void test_a_lone_v() {
    std::printf("[a lone left V, vout 10, arc ending 1.8 deg short, a long diagonal on]\n");
    VLane ln;
    ln.exit_v = true; ln.exit_s0 = 0.0f; ln.exit_off = -10.0f; ln.next_arc_s = 300.0f;
    ln.fit_exit();
    for (float v : {344.0f, 544.0f, 800.0f}) {
        Run old = fly(ln, v, -1.8f, false, 0.04f, 60.0f, 117.0f);
        Run now = fly(ln, v, -1.8f, true, 0.04f, 60.0f, 117.0f);
        std::printf("    v %4.0f: no return %+5.1f at 60 / %+5.1f at the post (117); lane change "
                    "%+5.1f / %+5.1f, max heading %.1f, max inside %+.1f\n",
                    (double)v, (double)old.lat_at[0], (double)old.lat_at[1],
                    (double)now.lat_at[0], (double)now.lat_at[1], (double)now.max_head,
                    (double)now.max_in);
        expect("old: still >= 10 outside at the post", old.lat_at[1] <= -10.0f);
        // What the plant can do (DiagLane.h): 10 mm in 60 mm only at MR2's
        // V speeds; at the V ceiling ~6 mm is still out at 60.
        float at60 = v <= 400.0f ? 3.5f : v <= 600.0f ? 6.5f : 9.0f;
        expect("walked back at 60 mm as far as the plant allows",
               std::fabs(now.lat_at[0]) <= at60);
        expect("within 3.5 mm at the post (117)", std::fabs(now.lat_at[1]) <= 3.5f);
        expect("no overshoot past 3 mm to the inside", now.max_in <= 3.0f);
    }
}

static void test_the_v_then_a_45_out() {
    std::printf("[the 09-30 second V: a 45-out 82 mm after it]\n");
    VLane ln;
    ln.exit_v = true; ln.exit_s0 = 0.0f; ln.exit_off = -10.0f; ln.next_arc_s = 82.0f;
    ln.fit_exit();
    for (float v : {344.0f, 544.0f, 700.0f}) {
        Run now = fly(ln, v, 0.0f, true, 0.04f, 40.0f, 60.0f);
        std::printf("    v %4.0f: %+5.1f at 40, %+5.1f at 60; at the 45-out heading %+.1f, "
                    "lateral %+.1f; max heading %.1f\n",
                    (double)v, (double)now.lat_at[0], (double)now.lat_at[1],
                    (double)now.head_at_arc, (double)now.lat_at_arc, (double)now.max_head);
        expect("heading within 6 deg of the line when the 45-out starts (MR5: 12)",
               std::fabs(now.head_at_arc) <= 6.0f);
        expect("lateral within 5.5 mm of the line at the 45-out",
               std::fabs(now.lat_at_arc) <= 5.5f);
    }
}

static void test_two_vs_the_same_way() {
    std::printf("[the 09-30 first V: the second V (same hand) 159 mm on]\n");
    VLane ln;
    ln.exit_v = true; ln.exit_s0 = 0.0f; ln.exit_off = -10.0f;
    ln.entry_v = true; ln.entry_arc_s = 159.0f; ln.entry_off = -10.0f;
    ln.next_arc_s = 159.0f; ln.fit_exit();
    for (float v : {544.0f, 800.0f}) {
        Run now = fly(ln, v, 0.0f, true, 0.04f, 60.0f, 117.0f);
        std::printf("    v %4.0f: at the next V heading %+.1f, lateral %+.1f (set point %+.1f); "
                    "max heading %.1f (MR5 swung +10 / -7)\n",
                    (double)v, (double)now.head_at_arc, (double)now.lat_at_arc,
                    (double)now.sp_at_arc, (double)now.max_head);
        expect("no wiggle: heading never past 6.5 deg (MR5: +10 then -7)",
               now.max_head <= 6.5f);
        expect("heading within 2 deg at the next V", std::fabs(now.head_at_arc) <= 2.0f);
        expect("lateral within 2 mm of the set point at the next V",
               std::fabs(now.lat_at_arc - now.sp_at_arc) <= 2.0f);
    }
}

static void test_the_entry_push() {
    std::printf("[the push before a V: 10 mm outward over the last pitch]\n");
    VLane ln;
    ln.entry_v = true; ln.entry_arc_s = 300.0f; ln.entry_off = -10.0f; ln.next_arc_s = 300.0f;
    for (float v : {344.0f, 544.0f, 800.0f}) {
        Run now = fly(ln, v, 0.0f, true, 0.04f, 250.0f, 280.0f);
        float e_out = -now.lat_at_arc;
        std::printf("    v %4.0f: at the arc lateral %+5.1f (set point %+.1f), heading %+.1f "
                    "-> exit %+.1f mm along\n",
                    (double)v, (double)now.lat_at_arc, (double)now.sp_at_arc,
                    (double)now.head_at_arc, (double)v_exit_along_mm(10.0f, e_out));
        expect("the push is made: within 3 mm of vout at the arc (10-01 MR2 made ~3 of 10)",
               std::fabs(now.lat_at_arc - now.sp_at_arc) <= 3.0f);
        expect("heading within 5 deg of the line when the arc takes over",
               std::fabs(now.head_at_arc) <= 5.0f);
    }
    // The shortfall's along error, and its sign: 3 of 10 made = 7 ahead.
    expect("entered 3 outside of 10: 7 mm AHEAD at the exit",
           std::fabs(v_exit_along_mm(10.0f, 3.0f) - 7.0f) < 1e-5f);
    expect("entered on the offset line: no along error", v_exit_along_mm(10.0f, 10.0f) == 0.0f);
}

int main() {
    test_the_v_lands_vout_outside_whatever_the_entry();
    test_the_set_point();
    test_the_dead_reckoning();
    test_a_lone_v();
    test_the_v_then_a_45_out();
    test_two_vs_the_same_way();
    test_the_entry_push();
    std::printf(fails ? "diag_lane: %d FAILED\n" : "diag_lane: all passed\n", fails);
    return fails ? 1 : 0;
}
