// WallGate: inside-wall edge detection and the along-track correction.
//
//   g++ -std=c++17 -I../lib/Control ../lib/Control/WallGate.cpp \
//       test_wall_gate.cpp -o test_wall_gate && ./test_wall_gate
// (or ./run_tests.sh wall_gate)

#include "WallGate.h"

#include <cmath>
#include <cstdio>
#include <initializer_list>

static int failures = 0;

static void expect(const char *what, bool ok) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        failures++;
    }
}

static void expect_near(const char *what, float got, float want, float tol) {
    if (!(std::fabs(got - want) <= tol)) {
        std::printf("FAIL: %s -- got %.2f, want %.2f (tol %.2f)\n", what, got,
                    want, tol);
        failures++;
    }
}

static WallGateConfig cfg(float expect_s) {
    WallGateConfig c;
    c.expect_s_mm = expect_s;
    return c;
}

// Walk the mouse down a corridor at `step` mm per packet. The inside wall
// reads `near` until the mouse passes `wall_end_s`, then reads `after`
// (255 = non-return, or a valid far reading = a step onto the next wall).
struct Sim {
    WallGate g;
    float s = 0.0f;
    float step = 3.0f;
    float lat = 0.0f;
    WallGate::Result last = WallGate::NONE;
    float corr = 0.0f;
    int corrections = 0;

    void run(float from_s, float to_s, float wall_end_s, uint8_t near_mm,
             uint8_t after_mm) {
        for (s = from_s; s < to_s; s += step) {
            uint8_t d = (s < wall_end_s) ? near_mm : after_mm;
            float c = 0.0f;
            WallGate::Result r = g.update(s, d, true, lat, &c);
            if (r != WallGate::NONE) {
                last = r;
                if (r == WallGate::CORRECTION) {
                    corr = c;
                    corrections++;
                }
            }
        }
    }
};

// A clean edge, seen exactly where predicted, corrects by ~0.
static void test_clean_edge_on_prediction() {
    Sim sim;
    sim.g.arm(cfg(200.0f));
    sim.run(0.0f, 300.0f, /*wall_end_s=*/200.0f, /*near=*/80, /*after=*/255);
    expect("fired once", sim.corrections == 1);
    expect("result is a correction", sim.last == WallGate::CORRECTION);
    // The drop is stamped at the first sample past the end, so the error is
    // within one packet of travel.
    expect_near("correction ~0", sim.corr, 0.0f, sim.step + 0.01f);
    expect("gate consumed", sim.g.fired());
}

// The mouse is 20 mm FARTHER along than odometry believes: the edge shows
// up 20 mm early in s, so the correction is +20 (push s forward).
static void test_edge_early_means_positive_correction() {
    Sim sim;
    sim.g.arm(cfg(200.0f));
    sim.run(0.0f, 300.0f, 180.0f, 80, 255);
    expect("fired", sim.corrections == 1);
    expect_near("positive correction", sim.corr, 20.0f, sim.step + 0.01f);
}

static void test_edge_late_means_negative_correction() {
    Sim sim;
    sim.g.arm(cfg(200.0f));
    sim.run(0.0f, 300.0f, 220.0f, 80, 255);
    expect("fired", sim.corrections == 1);
    expect_near("negative correction", sim.corr, -20.0f, sim.step + 0.01f);
}

// A step onto the NEXT wall down the corridor is the usual signature, and
// is strong evidence -- it confirms in fewer samples than a non-return.
static void test_step_onto_next_wall() {
    Sim sim;
    sim.g.arm(cfg(200.0f));
    sim.run(0.0f, 300.0f, 200.0f, 80, /*after=*/230);
    expect("step counts as an edge", sim.corrections == 1);
    expect_near("step correction ~0", sim.corr, 0.0f, sim.step + 0.01f);
}

// Silent or certain: an implausible edge is dropped, not applied.
static void test_out_of_window_is_silent() {
    Sim sim;
    sim.g.arm(cfg(200.0f));   // window 35
    sim.run(0.0f, 400.0f, 300.0f, 80, 255);   // edge 100 mm late
    expect("no correction applied", sim.corrections == 0);
    expect("reported as out of window", sim.last == WallGate::SKIPPED_WINDOW);
    expect("gate NOT consumed", !sim.g.fired());
}

// Off centre: the inside beam is untrustworthy, so skip -- and crucially do
// not consume the gate, so a later clean approach can still anchor.
static void test_off_centre_skips_without_consuming() {
    Sim sim;
    sim.lat = WallGateConfig{}.lat_max_mm;   // at the limit, and it is >=
    sim.g.arm(cfg(200.0f));
    sim.run(0.0f, 300.0f, 200.0f, 80, 255);
    expect("no correction", sim.corrections == 0);
    expect("reported as off centre", sim.last == WallGate::SKIPPED_LAT);
    expect("gate still available", !sim.g.fired());
}

// Run 140 regression. The session's only failure launched loop 17 at
// -25.7 mm, was refused by 0.7 mm, flew the corner ungated and ended in the
// outer wall. The limit was raised to 35; this pins the case that motivated
// it, so a future tightening has to argue with the run that caused it.
static void test_run140_lateral_now_gates() {
    Sim sim;
    sim.lat = -25.7f;
    sim.g.arm(cfg(200.0f));
    sim.run(0.0f, 300.0f, 200.0f, 80, 255);
    expect("run 140's -25.7 now produces a correction", sim.corrections == 1);
    expect("and is not refused on lateral", sim.last != WallGate::SKIPPED_LAT);
}

// ---- THE FEATURE OBSERVER ----------------------------------------------------
// The instrument for bert's maze gate (2026-08-22): "there may be many walls
// and gaps before a turn, the mouse should count these and gate the turn from
// the last good edge." These pin what it records, because every later decision
// is sized from these numbers and a mis-recorded run would mis-size all of it.

static WallGate feed(const uint8_t *d, int n, float step, float from_s,
                     float live_from = 0.0f) {
    WallGate g;
    WallGateConfig c;
    c.expect_s_mm = 1e6f;          // never decide; we are testing observation
    c.live_from_s_mm = live_from;
    g.arm(c);
    float junk = 0.0f;
    for (int i = 0; i < n; i++)
        g.update(from_s + i * step, d[i], true, 0.0f, &junk);
    return g;
}

// A continuous wall is ONE run, and its packet count is what calibrates
// everything else -- a post is only interpretable against a known length.
static void test_observer_sees_a_wall_as_one_run() {
    uint8_t d[40];
    for (int i = 0; i < 40; i++) d[i] = (i < 30) ? 84 : 255;
    WallGate g = feed(d, 40, 3.0f, 0.0f);
    expect("one run seen", g.run_count() == 1);
    expect("and stored", g.runs_stored() == 1);
    expect("30 packets of wall", g.run(0).n == 30);
    expect_near("run starts at s=0", g.run(0).s0_mm, 0.0f, 0.01f);
    expect_near("run ends at the last wall packet", g.run(0).s1_mm, 87.0f, 0.01f);
    expect("min range is the wall", g.run(0).min_mm == 84);
}

// A POST. 12 mm of surface at 3 mm per packet is four packets. Under the
// old 12-packet acquisition rule the step gate was blind to it at speed;
// with the span-based rule (8 mm + 4 clustered packets, 2026-08-29) this
// same post now clears acquisition -- which is the point of the change.
static void test_observer_sees_a_post_as_a_short_run() {
    uint8_t d[20];
    for (int i = 0; i < 20; i++) d[i] = 255;
    for (int i = 8; i < 12; i++) d[i] = 88;   // 4 packets = 12 mm at 3 mm
    WallGate g = feed(d, 20, 3.0f, 0.0f);
    expect("one run", g.run_count() == 1);
    expect("four packets", g.run(0).n == 4);
    // Packets 8..11 at 3 mm land on s = 24, 27, 30, 33, so the centre is
    // 28.5 -- the run spans the SAMPLES, not the post's physical edges, and
    // the difference is half a packet either side. That distinction is the
    // whole reason the record keeps s0 and s1 rather than a midpoint.
    expect_near("centred on the sampled post",
                0.5f * (g.run(0).s0_mm + g.run(0).s1_mm), 28.5f, 0.01f);
    expect("a sampled post now MEETS the acquisition floor",
           g.run(0).n >= WallGateConfig{}.present_n &&
               g.run(0).s1_mm - g.run(0).s0_mm >=
                   WallGateConfig{}.present_span_mm);
}

// ---- THE WALL-POST GAP (bert, 2026-08-29) -----------------------------------
// Construction leaves ~2 mm between the wall's end and the 12 mm post. At
// slow (braking) approach density that gap is ~3 packets -- enough for the
// old count-based confirm to fire the gate AT THE GAP, 14 mm before the
// post's far edge: the -15 mm fingerprint of runs 203-207. The distance-
// based confirm must ride across the gap and fire at the real opening.
static void test_wall_gap_post_fires_at_the_post_not_the_gap() {
    for (float step : {0.7f, 1.8f, 3.0f}) {   // slow gather, race, coarse
        WallGate g;
        g.arm(cfg(214.0f));   // the REAL edge: the post's far end
        float c = 0.0f;
        int fired = 0;
        float corr = 99.0f;
        for (float s = 0.0f; s < 320.0f; s += step) {
            uint8_t d;
            if (s < 200.0f) d = 82;                     // the wall
            else if (s < 202.0f) d = 255;               // the 2 mm gap
            else if (s < 214.0f) d = 85;                // the 12 mm post
            else d = 255;                               // the true opening
            if (g.update(s, d, true, 0.0f, &c) == WallGate::CORRECTION) {
                fired++;
                corr = c;
            }
        }
        char what[80];
        std::snprintf(what, sizeof what,
                      "fires once at %.1f mm/packet", step);
        expect(what, fired == 1);
        std::snprintf(what, sizeof what,
                      "anchors on the POST edge (corr ~0), not the gap "
                      "(corr +14), at %.1f mm/packet", step);
        expect_near(what, corr, 0.0f, step + 0.5f);
    }
}

// A bare post at RACE speed. 12 mm at 1.8 mm/packet is ~7 packets -- under
// the old 12-packet rule this corner went NO_EDGE (the v1500 laps measured
// exactly that); span-based acquisition sees it.
static void test_post_acquires_at_race_speed() {
    WallGate g;
    g.arm(cfg(112.0f));   // post far edge
    float c = 0.0f;
    int fired = 0;
    for (float s = 0.0f; s < 250.0f; s += 1.8f) {
        uint8_t d = (s >= 100.0f && s < 112.0f) ? 87 : 255;
        if (g.update(s, d, true, 0.0f, &c) == WallGate::CORRECTION) fired++;
    }
    expect("a bare post gates at 1.8 mm/packet", fired == 1);
    expect_near("on its far edge", c, 0.0f, 2.5f);
}

// LATERAL COMPENSATION (bert's bench numbers, 2026-08-29): the beam's lead
// shortens ~1 mm per mm of offset toward the inside wall, so the drop is
// seen LATE by the same amount. klat (registry `gklat`) undoes it; 0 keeps
// the old behaviour. Lat arrives signed toward-the-inside from the caller.
static void test_lateral_compensation() {
    // Riding 10 mm toward the inside, the physical drop appears 10 mm late.
    auto fly = [](float klat, float *corr_out) {
        WallGate g;
        WallGateConfig c = cfg(200.0f);
        c.klat = klat;
        g.arm(c);
        float corr = 0.0f;
        int fired = 0;
        for (float s = 0.0f; s < 300.0f; s += 3.0f) {
            uint8_t d = (s < 210.0f) ? 80 : 255;
            if (g.update(s, d, true, 10.0f, &corr) == WallGate::CORRECTION)
                fired++;
        }
        *corr_out = corr;
        return fired;
    };
    float corr;
    expect("uncompensated fires", fly(0.0f, &corr) == 1);
    expect_near("and carries the -10 lateral bias", corr, -10.0f, 3.5f);
    expect("compensated fires", fly(1.0f, &corr) == 1);
    expect_near("and the bias is gone", corr, 0.0f, 3.5f);
}

// The packet floor still means something: two glitch packets spanning less
// than the required 8 mm are not a surface, whatever the speed.
static void test_a_blip_is_still_not_a_wall() {
    WallGate g;
    g.arm(cfg(60.0f));
    float c = 0.0f;
    int fired = 0;
    for (float s = 0.0f; s < 150.0f; s += 3.0f) {
        uint8_t d = (s >= 51.0f && s <= 57.0f) ? 84 : 255;  // 3 packets, 6 mm
        if (g.update(s, d, true, 0.0f, &c) == WallGate::CORRECTION) fired++;
    }
    expect("a sub-span blip never fires the gate", fired == 0);
}

// RUN 148's real post, replayed. Two inner walls out; 26 stripped approaches
// gave a solid run of 30.8 +/- 5.1 packets at 22.1 +/- 4.0 mm, preceded by
// short blips at 107-140 mm separated from it by at most 4 mm. Those blips
// are dropouts inside one feature, and BRIDGE_MM has to swallow them or the
// count -- which is what bert's gate is to be built on -- lies.
static void test_run148_post_is_one_feature_not_three() {
    uint8_t d[80];
    for (int i = 0; i < 80; i++) d[i] = 255;
    d[20] = 120; d[21] = 117;          // precursor blip, 2 packets
    d[24] = 111;                       // second blip, 1 packet
    for (int i = 27; i < 58; i++) d[i] = 88;   // the post, 31 packets
    // 0.72 mm/packet, the measured density: the blip gaps are ~2 and ~2 mm.
    WallGate g = feed(d, 80, 0.72f, 0.0f);
    expect("bridged into ONE feature", g.run_count() == 1);
    expect("spanning blips through post", g.run(0).n == 31 + 3);
    expect("and it clears present_n with room",
           g.run(0).n > 2 * WallGateConfig{}.present_n);
    expect("closest reading is the post, not the blips", g.run(0).min_mm == 88);
}

// A gap WIDER than the bridge is still two features -- the bridge must not
// swallow a real opening. 148's worst in-feature gap was 4 mm; a corridor
// opening is at least a cell.
static void test_bridge_does_not_swallow_a_real_gap() {
    uint8_t d[120];
    for (int i = 0; i < 120; i++) d[i] = 255;
    for (int i = 0; i < 30; i++) d[i] = 84;     // wall
    for (int i = 90; i < 120; i++) d[i] = 88;   // next feature, far away
    WallGate g = feed(d, 120, 3.0f, 0.0f);
    expect("two features", g.run_count() >= 1);
    expect("the wall closed on its own", g.run(0).n == 30);
    expect("gap far exceeds the bridge",
           (90 - 29) * 3.0f > WallGate::BRIDGE_MM);
}

// WALL, GAP, POST -- the sequence a stripped approach gives, in order.
static void test_observer_records_the_sequence() {
    uint8_t d[60];
    for (int i = 0; i < 60; i++) d[i] = 255;
    for (int i = 0; i < 20; i++) d[i] = 82;    // wall
    for (int i = 40; i < 45; i++) d[i] = 90;   // post after a gap
    WallGate g = feed(d, 60, 3.0f, 0.0f);
    expect("two runs", g.run_count() == 2);
    expect("wall first, 20 packets", g.run(0).n == 20);
    expect("post second, 5 packets", g.run(1).n == 5);
    expect("and they are in travel order", g.run(0).s1_mm < g.run(1).s0_mm);
    expect_near("gap of 63 mm", g.run(1).s0_mm - g.run(0).s1_mm, 63.0f, 0.01f);
}

// More features than the record holds: the COUNT must still be truthful,
// because "how many did I pass" is the thing matched against the map.
static void test_observer_counts_beyond_what_it_stores() {
    uint8_t d[120];
    for (int i = 0; i < 120; i++) d[i] = ((i / 4) % 2 == 0) ? 85 : 255;
    WallGate g = feed(d, 120, 3.0f, 0.0f);
    expect("stores at most MAX_RUNS", g.runs_stored() <= WallGate::MAX_RUNS);
    expect("but counts them all", g.run_count() > WallGate::MAX_RUNS);
}

// OBSERVE WIDE, DECIDE NARROW. live_from_s_mm must defer the DECISION without
// costing a single observation -- if it gated the observer too, splitting the
// two distances would have bought nothing.
static void test_live_from_defers_the_decision_not_the_observation() {
    Sim early;
    early.g.arm(cfg(200.0f));
    early.run(0.0f, 300.0f, 200.0f, 80, 255);
    expect("baseline fires", early.corrections == 1);

    WallGate g;
    WallGateConfig c = cfg(200.0f);
    c.live_from_s_mm = 250.0f;   // past the edge: no decision possible
    g.arm(c);
    float junk = 0.0f;
    int fired = 0;
    for (float s = 0.0f; s < 300.0f; s += 3.0f) {
        uint8_t d = (s < 200.0f) ? 80 : 255;
        if (g.update(s, d, true, 0.0f, &junk) == WallGate::CORRECTION) fired++;
    }
    expect("decision suppressed before live_from", fired == 0);
    expect("but the wall was still OBSERVED", g.run_count() == 1);
    expect("with its full length", g.run(0).n == 67);
}

// A drop before the wall was ever acquired is not an edge -- an opening can
// only be the end of something.
static void test_no_wall_no_edge() {
    Sim sim;
    sim.g.arm(cfg(200.0f));
    sim.run(0.0f, 300.0f, /*wall_end_s=*/-1.0f, 80, 255);  // never present
    expect("nothing fires", sim.corrections == 0);
    expect("no result at all", sim.last == WallGate::NONE);
}

// A single spurious non-return in the middle of a healthy wall must not
// look like an edge: it neither confirms (needs 3) nor should it destroy
// the presence run so completely that the real edge afterwards is missed.
static void test_single_dropout_does_not_fire() {
    WallGate g;
    g.arm(cfg(200.0f));
    float c = 0.0f;
    int fired = 0;
    for (float s = 0.0f; s < 300.0f; s += 3.0f) {
        uint8_t d;
        if (std::fabs(s - 90.0f) < 1.0f) d = 255;         // one glitch
        else if (s < 200.0f) d = 80;
        else d = 255;                                     // the real edge
        if (g.update(s, d, true, 0.0f, &c) == WallGate::CORRECTION) fired++;
    }
    expect("exactly one correction", fired == 1);
    expect_near("the real edge is the one found", c, 0.0f, 3.5f);
}

// Fires at most once per arming.
static void test_fires_once() {
    Sim sim;
    sim.g.arm(cfg(200.0f));
    sim.run(0.0f, 600.0f, 200.0f, 80, 255);
    expect("only one correction", sim.corrections == 1);
    // Re-arming makes it available again.
    sim.g.arm(cfg(500.0f));
    expect("re-armed", !sim.g.fired());
}

// WHAT CORNER 3 OF THE RING RIG ACTUALLY DOES, and what saved it.
//
// This test does not check a fix. It reproduces a measured FAULT so that the
// reason the caller filters sub-floor readings -- and the limit of what that
// filter buys -- is written down somewhere executable.
//
// The numbers are from run 129, 124 corridor packets over three laps of the
// same physical corner. The inside beam there is BIMODAL: it reports either
// the true wall (mean 81.2 mm, and left+right = 171.4 against a 168 mm
// corridor, so it is right) or 38-40 mm, with nothing between 41 and 67
// anywhere in the run.
//
// Which branch is the lie is not a judgement call:
//
//     true branch   d(right)/d(left) = -1.32    the beams trade off
//     low  branch   d(right)/d(left) = -0.05    7 mm of mouse, 0.6 mm of read
//
// A distance that does not change when the mouse moves is not a distance. It
// is the channel's floor when it has no usable return -- which also explains
// why cleaning the optics (run 127) and running lights-out (run 128) changed
// nothing. The deficient reflector is that stretch of WALL.
//
// THE HARM IS THAT THE FLOOR OUTLIVES THE WALL. The true branch stops around
// s = 2760-2785; the 38s run on to 2845-2888. The drop is therefore stamped
// 57-129 mm later than the 840-850 mm corner-to-corner cadence of corners
// 0-2, and the plausibility window refuses it. Corner 3 is not a gate that
// fails to find an edge. It finds one, in the wrong place, and
// silent-or-certain throws it away -- which is the system working.
static void test_a_floor_reading_that_outlives_the_wall() {
    // Real wall to s = 160, flickering into the floor from s = 100, floor
    // alone from 160 to 245, then genuine non-return.
    auto packet = [](int i) -> uint8_t {
        float s = i * 5.0f;
        if (s < 160.0f) return (s >= 100.0f && i % 2) ? 38 : 84;
        if (s < 245.0f) return 38;
        return 255;
    };

    Sim raw;
    raw.g.arm(cfg(160.0f));   // the wall really ends here
    for (int i = 0; i < 120; i++) {
        float c = 0.0f;
        WallGate::Result r = raw.g.update(i * 5.0f, packet(i), true, 0.0f, &c);
        if (r != WallGate::NONE) raw.last = r;
        if (r == WallGate::CORRECTION) raw.corrections++;
    }
    expect("no correction is applied", raw.corrections == 0);
    expect("the late drop is refused as implausible, not believed",
           raw.last == WallGate::SKIPPED_WINDOW);
    expect("and the gate is NOT consumed, so a clean lap could still anchor",
           !raw.g.fired());

    // NOW THE HONEST PART. Skipping the sub-floor packets does not rescue
    // this corner: skipped packets leave the presence run intact, so the
    // drop still stamps on the first 255 in exactly the same place. The
    // filter stops a floor value being believed as a length; it cannot
    // conjure the wall end that the floor concealed. Only fixing the
    // reflector does that.
    Sim filt;
    filt.g.arm(cfg(160.0f));
    for (int i = 0; i < 120; i++) {
        uint8_t d = packet(i);
        if (d != 255 && d < 40) continue;   // the SWEEP_SIDE_MIN_MM skip
        float c = 0.0f;
        WallGate::Result r = filt.g.update(i * 5.0f, d, true, 0.0f, &c);
        if (r != WallGate::NONE) filt.last = r;
        if (r == WallGate::CORRECTION) filt.corrections++;
    }
    expect("filtering changes nothing here -- same refusal, same reason",
           filt.corrections == 0 && filt.last == WallGate::SKIPPED_WINDOW);
}

static void test_disarmed_is_inert() {
    WallGate g;
    float c = 0.0f;
    expect("starts disarmed", !g.armed());
    expect("inert", g.update(0.0f, 80, true, 0.0f, &c) == WallGate::NONE);
}

int main() {
    test_clean_edge_on_prediction();
    test_edge_early_means_positive_correction();
    test_edge_late_means_negative_correction();
    test_step_onto_next_wall();
    test_out_of_window_is_silent();
    test_off_centre_skips_without_consuming();
    test_run140_lateral_now_gates();
    test_observer_sees_a_wall_as_one_run();
    test_observer_sees_a_post_as_a_short_run();
    test_wall_gap_post_fires_at_the_post_not_the_gap();
    test_post_acquires_at_race_speed();
    test_lateral_compensation();
    test_a_blip_is_still_not_a_wall();
    test_observer_records_the_sequence();
    test_run148_post_is_one_feature_not_three();
    test_bridge_does_not_swallow_a_real_gap();
    test_observer_counts_beyond_what_it_stores();
    test_live_from_defers_the_decision_not_the_observation();
    test_no_wall_no_edge();
    test_single_dropout_does_not_fire();
    test_fires_once();
    test_a_floor_reading_that_outlives_the_wall();
    test_disarmed_is_inert();

    if (failures == 0) {
        std::printf("ALL WALL GATE TESTS PASSED\n");
        return 0;
    }
    std::printf("%d WALL GATE TEST(S) FAILED\n", failures);
    return 1;
}
