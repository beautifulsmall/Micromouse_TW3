// THE V EXIT LANE CHANGE (2026-09-30; bert, after the second V of the MR2
// out leg clipped the first outer post after it: "we can't have a specific
// maze trim, this won't work for an unknown maze ... MR2 needs to fly more
// safely").
//
// WHAT WAS WRONG. `vout` flies a V on lines offset OUTWARD by vout, joined
// by an arc of radius 64 + vout centred on the V's inside post. Whatever the
// mouse did on the entry diagonal, that arc lands it vout OUTSIDE the exit
// diagonal's centre line (an entry lateral e becomes an exit ALONG error of
// vout - e, never a lateral one -- see test_diag_lane). The compiler leaves
// the walk back to the runner (test_route: "the runner's job ... not the
// compiler's"), and the runner's diagonal servo only steers for one pitch
// after a FRESH POST SAMPLE (DGATE_HOLD_MM, 2026-09-27). After a V there is
// no fresh sample: the first post the exit diagonal offers IS that outer
// post. So the offset was never walked back, on any maze: the 09-30 lap's
// second V read that post at 37 (centred ~55) and hit it 60 mm after its
// arc, with vout 10 of the 18 mm coming straight from the offset. The
// linear 127 mm ramp was too slow anyway -- the mouse's front corner meets
// the first outer post ~72 mm after the arc ends (191 mm from the V station
// less the R74 tangent less the 45 mm nose).
//
// THE FIX, maze-independent. The exit is a PLANNED lane change from a KNOWN
// start (vout outward, by geometry), so it is flown on a dead-reckoned
// lateral, not on a post sample: seed the estimate with the offset at the
// arc's end, integrate the heading relative to the line (the applied bias
// minus the yaw error) along path-s, and steer with feedforward (the set
// point's own slope) plus the diagonal servo's feedback on the estimate. The
// set point returns to the line on a half-cosine over V_EXIT_RAMP_MM, done
// before the nose reaches the post. The first post sample then measures
// what the dead reckoning did NOT know -- the V's own along-track timing
// error, which moves the exit 1:1 -- and that residual is what the mouse
// learns per V on MR2 and spends from MR3 (RouteRun, the goff table's V
// layer). No hand trim, no maze-specific number.
//
// Header-only and free of the HAL so the host tests can drive it.

#pragma once

#include <math.h>

// The half-cosine return, at most this long; shorter when the next arc
// comes sooner (VLane::fit_exit). 90 mm: slope peaks at vout x pi / 180 rad
// -- 10 deg at vout 10, the lateral bias cap.
constexpr float V_EXIT_RAMP_MM = 90.0f;
constexpr float V_EXIT_RAMP_MIN_MM = 45.0f;
// The ramp must end this far before the next arc begins, so the heading is
// back on the line when the arc takes the reference (Control zeroes the
// lateral bias inside an arc): 2026-09-30 MR5 entered a 45-out 82 mm after
// a V with the heading still 12 deg off the line.
constexpr float V_EXIT_ARC_MARGIN_MM = 10.0f;
// The dead-reckoned servo flies at most this far past a V (two post
// pitches), and never into the next arc.
constexpr float V_LANE_MAX_MM = 255.0f;
// THE HEADING PLANT, fitted to the 2026-09-30 MR5 first-V exit (0.7 deg
// RMS over 160 mm, test_diag_lane): the applied bias slews at 120 deg/s
// (Control.h PATH_LAT_BIAS_SLEW_DPS), the yaw-rate demand is the bias rate
// plus (bias - heading) / 30 ms, and the wheels deliver it with a ~40 ms
// lag. The feedforward is therefore the set point's slope looked up AHEAD
// (v x lead), and the feedback on the dead reckoning is stiffer than the
// post servo's (0.36 is sized for one sample per 127 mm; this estimate is
// continuous). What the plant CANNOT do: walk 10 mm back in 60 mm above
// ~450 mm/s -- at 544 (a V's own ceiling at ralat 4000) ~6 mm is still out
// there, at 344 ~3. The learner and a smaller vout are what close the rest.
constexpr float V_LANE_LEAD_S = 0.05f;
constexpr float V_LANE_LEAD_MAX_MM = 45.0f;
constexpr float V_LANE_K_DEG_PER_MM = 0.7f;
// A post sample this far from the V's arc end still measures the V (the
// first outer post of the exit diagonal sits ~117 mm out). 260 -> 130 on
// 2026-10-04: at Hazlemere the (6,11) V was "learned" +7 mm late from a
// post read 165 mm past its arc ("post -11.7, dead reckoning +2.5"), a
// lateral error by then, and MR4 flew that V late into the outer post
// (hazel5). The day's true timing reads came 34-42 mm past the arc.
constexpr float V_LEARN_MAX_S_MM = 130.0f;
// What a learned V timing may ever do, either way (the table's own clamp is
// 60): half the diagonal lane's clearance.
constexpr float V_LEARN_MAX_MM = 20.0f;
// The goff table key for a V's learned TIMING: its cell and arrival heading,
// with this bit set -- so a 90 that another route flies through the same
// cell on the same heading never shares its entry.
constexpr unsigned GOFF_KEY_V = 4u;
constexpr float V_DIAG_PITCH_MM = 127.279f;   // Route.h ROUTE_DIAG_PITCH_MM

// THE ENTRY PUSH, flown too (2026-10-01, MR2). The V's arc is compiled
// tangent to the OFFSET lines; a mouse that arrives e outside instead of
// vout leaves the arc (vout - e) AHEAD of its path-s (test_diag_lane, the
// geometry) -- harmless if a post re-anchors before the next corner, but
// two Vs 382 mm apart have none: the 10-01 MR2 pushed only ~3 of 10 before
// the first V (the post servo's held sample, no feedforward), left it ~7
// ahead, and the second V fired ~7 late and exited 17 outside (its outer
// post read 37 -- as on 09-27 and 09-30). So the push is flown on the same
// dead-reckoned lane servo as the walk back, from the pitch before the arc
// (seeded by the last post sample, or the V before), and whatever is still
// missing at the arc's start is handed to path-s at its end.
constexpr float V_DR_MAX_AGE_MM = 400.0f;     // dead reckoning trusted this far
constexpr float V_ALONG_FIX_MAX_MM = 12.0f;   // the arc-end along correction's clamp
// Along error at a V's arc end (+ = the mouse AHEAD of path-s) when it
// started the arc e_out outside its centre line (+ outward) instead of vout.
inline float v_exit_along_mm(float vout, float e_out) { return vout - e_out; }

// Fraction of the exit offset still to take out d mm past the arc end, over
// a ramp of len mm (1 at the arc end, 0 from len on).
inline float v_exit_frac(float d, float len = V_EXIT_RAMP_MM) {
    if (d <= 0.0f) return 1.0f;
    if (d >= len) return 0.0f;
    return 0.5f * (1.0f + cosf(3.14159265f * d / len));
}

// THE DIAGONAL'S LATERAL SET POINT (+ left, the post samples' frame), the
// whole of it: dlat, plus the V EXIT's offset walked back on a half-cosine
// (the V behind), plus the NEXT V's entry offset ramped in linearly over the
// last pitch before its arc (the V ahead). On a short diagonal between two
// same-hand Vs the two overlap and the set point barely moves -- which is
// what the feedforward must follow: its slope, not one ramp's.
struct VLane {
    float base = 0.0f;
    bool exit_v = false;
    float exit_s0 = 0.0f, exit_off = 0.0f, exit_len = V_EXIT_RAMP_MM;
    bool entry_v = false;
    float entry_arc_s = 0.0f, entry_off = 0.0f;
    float next_arc_s = 1e9f;     // where this diagonal's exit arc begins

    // Shorten the exit ramp to end V_EXIT_ARC_MARGIN_MM before the next arc.
    void fit_exit() {
        float room = next_arc_s - exit_s0 - V_EXIT_ARC_MARGIN_MM;
        exit_len = room < V_EXIT_RAMP_MM ? room : V_EXIT_RAMP_MM;
        if (exit_len < V_EXIT_RAMP_MIN_MM) exit_len = V_EXIT_RAMP_MIN_MM;
    }
    float sp(float s) const {
        float v = base;
        if (exit_v) v += exit_off * v_exit_frac(s - exit_s0, exit_len);
        if (entry_v) {
            float t = (s - (entry_arc_s - V_DIAG_PITCH_MM)) / V_DIAG_PITCH_MM;
            if (t > 1.0f) t = 1.0f;
            if (t > 0.0f) v += entry_off * t;
        }
        return v;
    }
    float slope(float s) const { return 0.25f * (sp(s + 2.0f) - sp(s - 2.0f)); }
    // Inside the next V's entry push (or its feedforward's lead-in)?
    bool in_entry_push(float s) const {
        return entry_v && s < entry_arc_s &&
               s >= entry_arc_s - V_DIAG_PITCH_MM - V_LANE_LEAD_MAX_MM;
    }
};

// The lane servo's bias (deg, + left): the set point's slope LEAD mm ahead
// as feedforward (never past the next arc), plus the diagonal servo's gain
// on the estimate's error from the set point, clamped.
inline float v_lane_bias_deg(const VLane &ln, float s, float v_mm_s, float lat_est,
                             float k_deg_per_mm, float cap_deg) {
    float lead = v_mm_s * V_LANE_LEAD_S;
    if (lead > V_LANE_LEAD_MAX_MM) lead = V_LANE_LEAD_MAX_MM;
    if (lead < 0.0f) lead = 0.0f;
    float sl = s + lead < ln.next_arc_s ? ln.slope(s + lead) : 0.0f;
    float b = atanf(sl) * 57.2957795f - k_deg_per_mm * (lat_est - ln.sp(s));
    if (b > cap_deg) b = cap_deg;
    if (b < -cap_deg) b = -cap_deg;
    return b;
}

// Dead-reckoned lateral on a diagonal, + left, in the post samples' frame.
struct DiagDR {
    bool valid = false;
    bool from_v = false;   // seeded by a V's geometry, not by a post
    float lat = 0.0f;
    float s0 = 0.0f;       // path-s of the seed
    float s_prev = 0.0f;

    void seed(float lat0, float s, bool v) {
        valid = true;
        from_v = v;
        lat = lat0;
        s0 = s_prev = s;
    }
    void clear() { valid = false; from_v = false; }
    // One step: theta_deg = heading relative to the line (+ left) = the
    // applied path bias minus the yaw error. A backwards or absurd step
    // (a path correction, a stall) is not integrated.
    void step(float s, float theta_deg) {
        if (!valid) return;
        float ds = s - s_prev;
        s_prev = s;
        if (ds <= 0.0f || ds > 40.0f) return;
        lat += ds * sinf(theta_deg * 0.01745329f);
    }
};
