#include "Path.h"

#include <math.h>

namespace {
constexpr float DEG2RAD = 0.017453292519943295f;
constexpr float RAD2DEG = 57.29577951308232f;

float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// Clothoid endpoint in its own entry frame, from the Fresnel series. t is
// the total heading the clothoid sweeps (rad) and w the fraction of its
// length travelled, so w = 1 is the endpoint:
//     x = L (w - t^2 w^5/10 + t^4 w^9/216)
//     y = L (t w^3/3 - t^3 w^7/42 + t^5 w^11/1320)
// These are just the term-by-term integrals of cos/sin of theta = t*w^2.
// The (L, t) form stays well behaved as t -> 0, which the straight limit
// needs. Same series V1 used; kept because it is exact enough (relative
// error < 1e-4 for t below ~0.35 rad) and has no branch.
void clothoid_xy(float L, float t, float w, float &x, float &y) {
    float w2 = w * w, w3 = w2 * w, t2 = t * t;
    x = L * (w - t2 * w3 * w2 / 10.0f + t2 * t2 * w3 * w3 * w3 / 216.0f);
    y = L * (t * w3 / 3.0f - t2 * t * w3 * w3 * w / 42.0f +
             t2 * t2 * t * w3 * w3 * w3 * w2 / 1320.0f);
}

// Position along a LINEAR-CURVATURE segment, integrated numerically.
//
// The closed Fresnel form above only covers a clothoid that starts straight.
// The exit ramp starts curved and ends straight, and a general route could
// ask for anything, so pose() integrates
//     theta(u) = theta0 + k0*u + (k1 - k0) u^2 / (2 L)
// with Simpson over a fixed 8 intervals. Segments here are tens of
// millimetres and theta is smooth, so the h^4 error is far below a micron;
// the cost is 9 sin/cos pairs on a 600 MHz FPU, once per tick. Choosing
// arithmetic over a case analysis is deliberate -- the case analysis is
// where sign errors live, and this has none.
void ramp_xy(float len, float k0, float k1, float th0, float ds,
             float &dx, float &dy) {
    const int N = 8;  // even, for Simpson
    float h = ds / (float)N;
    float b = (k1 - k0) / (2.0f * len);
    dx = 0.0f;
    dy = 0.0f;
    for (int i = 0; i <= N; i++) {
        float u = h * (float)i;
        float th = th0 + k0 * u + b * u * u;
        float wgt = (i == 0 || i == N) ? 1.0f : ((i & 1) ? 4.0f : 2.0f);
        dx += wgt * cosf(th);
        dy += wgt * sinf(th);
    }
    dx *= h / 3.0f;
    dy *= h / 3.0f;
}
}  // namespace

void SpeedPath::clear(float x0_mm, float y0_mm, float yaw0_deg) {
    n_ = 0;
    nc_ = 0;
    len_ = 0.0f;
    next_vcap_ = 0.0f;
    x0_ = x0_mm;
    y0_ = y0_mm;
    yaw0_rad_ = yaw0_deg * DEG2RAD;
}

void SpeedPath::set_limits(float v_max_mm_s, float a_lon_mm_s2,
                           float a_lat_mm_s2, float a_brk_mm_s2) {
    if (v_max_mm_s > 0.0f) v_max_ = v_max_mm_s;
    if (a_lon_mm_s2 > 0.0f) a_lon_ = a_lon_mm_s2;
    if (a_lat_mm_s2 > 0.0f) a_lat_ = a_lat_mm_s2;
    a_brk_ = a_brk_mm_s2 > 0.0f ? a_brk_mm_s2 : 0.0f;
}

bool SpeedPath::push(float len_mm, float kappa0, float kappa1) {
    if (n_ >= PATH_MAX_SEGS || len_mm <= 0.0f) return false;
    PathSeg &s = seg_[n_];
    s.s0_mm = len_;
    s.len_mm = len_mm;
    s.kappa = kappa0;
    s.kappa1 = kappa1;
    s.vcap_mm_s = (kappa0 != 0.0f || kappa1 != 0.0f) ? next_vcap_ : 0.0f;
    if (n_ == 0) {
        s.x0_mm = x0_;
        s.y0_mm = y0_;
        s.yaw0_rad = yaw0_rad_;
    } else {
        // Start pose of this segment = end pose of the previous one. Walking
        // it forward once at build time keeps pose(s) O(1) at run time, and
        // means a rounding error can never make two segments disagree about
        // where the joint is.
        const PathSeg &p = seg_[n_ - 1];
        PathPose e = pose(p.s0_mm + p.len_mm);
        s.x0_mm = e.x_mm;
        s.y0_mm = e.y_mm;
        s.yaw0_rad = e.yaw_deg * DEG2RAD;
    }
    len_ += len_mm;
    n_++;
    return true;
}

bool SpeedPath::add_straight(float len_mm) {
    if (len_mm <= 0.0f) return true;  // a zero-length straight is a no-op
    // Merge with a preceding straight: a route compiler emits one per cell
    // and there is no reason to spend segment slots (or lookahead loop
    // iterations) on joints that are not joints.
    if (n_ > 0 && seg_[n_ - 1].kappa == 0.0f) {
        seg_[n_ - 1].len_mm += len_mm;
        len_ += len_mm;
        return true;
    }
    return push(len_mm, 0.0f, 0.0f);
}

bool SpeedPath::add_arc_plain(float radius_mm, float angle_deg) {
    float kappa = (angle_deg > 0.0f) ? 1.0f / radius_mm : -1.0f / radius_mm;
    float len = radius_mm * fabsf(angle_deg) * DEG2RAD;
    int idx = n_;
    if (!push(len, kappa, kappa)) return false;
    corner_seg_[nc_] = idx;
    corner_nseg_[nc_] = 1;
    corner_t_mm_[nc_] = radius_mm * tanf(0.5f * fabsf(angle_deg) * DEG2RAD);
    nc_++;
    return true;
}

float SpeedPath::tangent_for(float radius_mm, float clothoid_mm,
                             float sweep_deg) {
    float phi = fabsf(sweep_deg) * DEG2RAD;
    if (radius_mm <= 0.0f || phi <= 0.0f) return 0.0f;
    float half = tanf(0.5f * phi);
    if (half <= 0.0f) return 0.0f;
    if (clothoid_mm <= 0.0f) return radius_mm * half;
    float L = clothoid_mm;
    float t = L / (2.0f * radius_mm);           // sweep of ONE clothoid
    if (2.0f * t > PATH_CLOTHOID_MAX_SWEEP_FRAC * phi) {
        // Does not fit; the builder clamps L the same way, so report the
        // geometry the builder will actually produce.
        L = PATH_CLOTHOID_MAX_SWEEP_FRAC * phi * radius_mm;
        t = L / (2.0f * radius_mm);
    }
    float xc, yc;
    clothoid_xy(L, t, 1.0f, xc, yc);
    float m = yc - radius_mm * (1.0f - cosf(t));   // shift of the arc centre
    float x0 = xc - radius_mm * sinf(t);           // tangent offset
    // The shifted arc is a circle of effective radius R + m tangent to both
    // legs, whose tangent length is (R + m) * tan(phi/2); the clothoid adds
    // x0 along the leg. This MULTIPLIES by tan(phi/2), as the pure-arc form
    // above does. It divided until 2026-08-31 -- invisible at the only sweep
    // ever compiled (90 degrees, tan 45 = 1), and reporting 218 mm for an R84
    // 45-degree corner whose true consumption is ~50 the day one was asked.
    return x0 + (radius_mm + m) * half;
}

float SpeedPath::clothoid_for_rail(float radius_mm, float v_mm_s,
                                   float slew_dps2) {
    if (radius_mm <= 0.0f || v_mm_s <= 0.0f || slew_dps2 <= 0.0f) return 0.0f;
    // Inside a clothoid omega_dot = v^2 * dkappa/ds = v^2 / (R * L), so the
    // shortest L whose ramp the rail can already follow is v^2/(R*alpha).
    return v_mm_s * v_mm_s / (radius_mm * (slew_dps2 * DEG2RAD));
}

float SpeedPath::clothoid_for_room(float radius_mm, float t_max_mm,
                                   float sweep_deg) {
    if (radius_mm <= 0.0f) return 0.0f;
    if (tangent_for(radius_mm, 0.0f, sweep_deg) > t_max_mm) return 0.0f;
    // T is monotone increasing in L, so bisect. 24 halvings of a 0..90 mm
    // bracket lands well inside a micron, and this runs once per path build.
    float lo = 0.0f, hi = PATH_CLOTHOID_MAX_MM;
    if (tangent_for(radius_mm, hi, sweep_deg) <= t_max_mm) return hi;
    for (int i = 0; i < 24; i++) {
        float mid = 0.5f * (lo + hi);
        if (tangent_for(radius_mm, mid, sweep_deg) <= t_max_mm) lo = mid;
        else hi = mid;
    }
    return lo;
}

bool SpeedPath::add_arc(float radius_mm, float angle_deg) {
    if (radius_mm <= 0.0f || angle_deg == 0.0f) return false;
    if (nc_ >= PATH_MAX_CORNERS) return false;
    if (clothoid_len_ <= 0.0f) return add_arc_plain(radius_mm, angle_deg);

    float phi = fabsf(angle_deg) * DEG2RAD;
    float L = clothoid_len_;
    // The clothoid PAIR cannot sweep more than the corner does, or there is
    // no central arc left to be the corner. Clamp rather than refuse: a
    // caller asking for a long transition on a tight corner wants the
    // longest one that fits, not an error.
    float t = L / (2.0f * radius_mm);
    if (2.0f * t > PATH_CLOTHOID_MAX_SWEEP_FRAC * phi) {
        L = PATH_CLOTHOID_MAX_SWEEP_FRAC * phi * radius_mm;
        t = L / (2.0f * radius_mm);
    }
    if (L <= 0.0f) return add_arc_plain(radius_mm, angle_deg);

    float kappa = (angle_deg > 0.0f) ? 1.0f / radius_mm : -1.0f / radius_mm;
    float arc_len = radius_mm * (phi - 2.0f * t);
    if (arc_len <= 0.0f) return add_arc_plain(radius_mm, angle_deg);

    int idx = n_;
    if (!push(L, 0.0f, kappa)) return false;         // ramp in
    if (!push(arc_len, kappa, kappa)) return false;  // hold
    if (!push(L, kappa, 0.0f)) return false;         // ramp out
    corner_seg_[nc_] = idx;
    corner_nseg_[nc_] = 3;
    corner_t_mm_[nc_] = tangent_for(radius_mm, L, angle_deg);
    nc_++;
    return true;
}

// ---------------------------------------------------------------- U-turn
//
// The geometry, all of it, in closed form. For a symmetric
// clothoid-arc-clothoid 180 with clothoid length L sweeping t = L/(2R):
//
//   the clothoid ends at (x1, y1) = clothoid_xy(L, t)
//   the arc centre is then at (x1 - R sin t, y1 + R cos t)
//   the SHIFT the clothoid adds is  p = y1 - R(1 - cos t)
//   the total lateral offset of the 180 is  2 (R + p)
//   the apex -- heading exactly 90 from entry, by symmetry the midpoint --
//   sits  x1 - R sin t + R  along the entry direction
//
// Both are exact, so no marching is needed at run time. The offset is
// monotonic in R (bigger radius, wider turn), which is what lets a bisection
// find the radius the corridor demands.
static void uturn_geometry(float R, float v_mm_s, float slew_dps2,
                           float *L_out, float *offset_out, float *extent_out,
                           float *len_out) {
    float L = SpeedPath::clothoid_for_rail(R, v_mm_s, slew_dps2);
    float t = L / (2.0f * R);
    float x1, y1;
    clothoid_xy(L, t, 1.0f, x1, y1);
    float p = y1 - R * (1.0f - cosf(t));
    if (L_out) *L_out = L;
    if (offset_out) *offset_out = 2.0f * (R + p);
    if (extent_out) *extent_out = x1 - R * sinf(t) + R;
    // Two clothoids plus the arc that is left once they have taken 2t of it.
    if (len_out) *len_out = 2.0f * L + R * ((float)M_PI - 2.0f * t);
}

bool SpeedPath::uturn_solve(float offset_mm, float v_mm_s, float slew_dps2,
                            UTurn *out) {
    if (offset_mm <= 0.0f || v_mm_s <= 0.0f || slew_dps2 <= 0.0f) return false;
    // Bracket generously and bisect. 30 iterations takes a 150 mm bracket to
    // well under a micron, and the whole thing runs once per route compile.
    float lo = 20.0f, hi = 200.0f;
    float o_lo, o_hi;
    uturn_geometry(lo, v_mm_s, slew_dps2, nullptr, &o_lo, nullptr, nullptr);
    uturn_geometry(hi, v_mm_s, slew_dps2, nullptr, &o_hi, nullptr, nullptr);
    if (offset_mm < o_lo || offset_mm > o_hi) return false;
    for (int i = 0; i < 40; i++) {
        float mid = 0.5f * (lo + hi);
        float o;
        uturn_geometry(mid, v_mm_s, slew_dps2, nullptr, &o, nullptr, nullptr);
        if (o < offset_mm) lo = mid;
        else hi = mid;
    }
    float R = 0.5f * (lo + hi);
    float L, off, ext, len;
    uturn_geometry(R, v_mm_s, slew_dps2, &L, &off, &ext, &len);
    if (L <= 0.0f || L > PATH_CLOTHOID_MAX_MM) return false;
    if (out) {
        out->radius_mm = R;
        out->clothoid_mm = L;
        out->extent_mm = ext;
        out->length_mm = len;
    }
    return true;
}

// Geometry at a FIXED radius and an explicitly chosen clothoid, rather than
// one derived from a rail. Same closed forms as uturn_geometry; only where L
// comes from differs.
static void uturn_geometry_L(float R, float L, float *offset_out,
                             float *extent_out, float *len_out) {
    float t = L / (2.0f * R);
    float x1, y1;
    clothoid_xy(L, t, 1.0f, x1, y1);
    float p = y1 - R * (1.0f - cosf(t));
    if (offset_out) *offset_out = 2.0f * (R + p);
    if (extent_out) *extent_out = x1 - R * sinf(t) + R;
    if (len_out) *len_out = 2.0f * L + R * ((float)M_PI - 2.0f * t);
}

bool SpeedPath::uturn_solve_radius(float offset_mm, float radius_mm,
                                   float v_mm_s, UTurn *out, float *slew_out) {
    if (offset_mm <= 0.0f || radius_mm <= 0.0f || v_mm_s <= 0.0f) return false;
    // 2R alone must not already overshoot: p is positive, so no clothoid can
    // bring the offset back DOWN.
    if (2.0f * radius_mm > offset_mm) return false;
    // Upper bound on L: the clothoid may sweep at most 90 degrees per end
    // (t = L/2R <= pi/2), or the arc between them is negative. Back off a
    // little so add_uturn's arc_len > 0 check has something to hold.
    float l_hi = 0.98f * (float)M_PI * radius_mm;
    if (l_hi > PATH_UTURN_CLOTHOID_MAX_MM) l_hi = PATH_UTURN_CLOTHOID_MAX_MM;
    float o_hi;
    uturn_geometry_L(radius_mm, l_hi, &o_hi, nullptr, nullptr);
    if (o_hi < offset_mm) return false;   // cannot reach it at this radius
    float lo = 0.0f, hi = l_hi;
    for (int i = 0; i < 40; i++) {
        float mid = 0.5f * (lo + hi);
        float o;
        uturn_geometry_L(radius_mm, mid, &o, nullptr, nullptr);
        if (o < offset_mm) lo = mid;
        else hi = mid;
    }
    float L = 0.5f * (lo + hi);
    float off, ext, len;
    uturn_geometry_L(radius_mm, L, &off, &ext, &len);
    if (L <= 0.0f || L > PATH_UTURN_CLOTHOID_MAX_MM) return false;
    if (radius_mm * ((float)M_PI - L / radius_mm) <= 0.0f) return false;
    if (out) {
        out->radius_mm = radius_mm;
        out->clothoid_mm = L;
        out->extent_mm = ext;
        out->length_mm = len;
    }
    // The rail this implies. Refuse rather than report a geometry the
    // executor cannot follow -- a clipped ramp is a path nobody solved.
    float rail = v_mm_s * v_mm_s / (radius_mm * L) * RAD2DEG;
    if (rail > PATH_UTURN_MAX_RAIL_DPS2) return false;
    if (slew_out) *slew_out = rail;
    return true;
}

bool SpeedPath::add_uturn(const UTurn &u, float sign, float entry_setback_mm) {
    if (u.radius_mm <= 0.0f || u.clothoid_mm <= 0.0f) return false;
    if (nc_ >= PATH_MAX_CORNERS) return false;
    float kappa = (sign > 0.0f) ? 1.0f / u.radius_mm : -1.0f / u.radius_mm;
    float t = u.clothoid_mm / (2.0f * u.radius_mm);
    float arc_len = u.radius_mm * ((float)M_PI - 2.0f * t);
    if (arc_len <= 0.0f) return false;
    int idx = n_;
    if (!push(u.clothoid_mm, 0.0f, kappa)) return false;   // ramp in
    if (!push(arc_len, kappa, kappa)) return false;        // hold
    if (!push(u.clothoid_mm, kappa, 0.0f)) return false;   // ramp out
    corner_seg_[nc_] = idx;
    corner_nseg_[nc_] = 3;
    // NOT a tangent -- see the note in Path.h. This is the distance back to
    // the cell centre the turn is named by, so the gate's expectation
    // arithmetic needs no special case for a U-turn.
    corner_t_mm_[nc_] = entry_setback_mm;
    nc_++;
    return true;
}

bool SpeedPath::begin_corner(float lead_mm, float radius_mm, float angle_deg,
                             float exit_mm) {
    clear(0.0f, 0.0f, 0.0f);
    if (!add_straight(lead_mm)) return false;
    if (!add_arc(radius_mm, angle_deg)) return false;
    if (!add_straight(exit_mm)) return false;
    return true;
}

bool SpeedPath::begin_ring(float first_leg_mm, float side_mm,
                           float radius_mm, bool left_turns) {
    float angle = left_turns ? 90.0f : -90.0f;
    // The straight between two corner points is side - 2T, and T is R only
    // when there is no clothoid. Using R here with clothoids on would leave
    // the ring 8*(T-R) too long and it would not close -- which looks
    // exactly like drift, and is the reason test_path asserts closure.
    float T = tangent_for(radius_mm, clothoid_len_, angle);
    if (radius_mm <= 0.0f || side_mm <= 2.0f * T) return false;
    if (first_leg_mm <= T) return false;
    float last_leg = side_mm - first_leg_mm;
    if (last_leg <= T) return false;

    clear(0.0f, 0.0f, 0.0f);
    if (!add_straight(first_leg_mm - T)) return false;
    for (int k = 0; k < 4; k++) {
        if (!add_arc(radius_mm, angle)) return false;
        if (k < 3) {
            if (!add_straight(side_mm - 2.0f * T)) return false;
        }
    }
    return add_straight(last_leg - T);
}

int SpeedPath::seg_at(float s_mm) const {
    if (n_ == 0) return 0;
    if (s_mm <= 0.0f) return 0;
    // The last segment starting at or before s. s0 is strictly increasing
    // (push refuses a zero length), so a binary search finds the same index
    // the old backwards scan did -- in 10 steps instead of up to 1025.
    int lo = 0, hi = n_ - 1;
    while (lo < hi) {
        int mid = (lo + hi + 1) / 2;
        if (seg_[mid].s0_mm <= s_mm) lo = mid;
        else hi = mid - 1;
    }
    return lo;
}

float SpeedPath::kappa_at(float s_mm) const {
    if (n_ == 0) return 0.0f;
    // Past the end the path is straight: the executor is stopping, and a
    // residual curvature there would keep commanding a differential into a
    // mouse that has nowhere left to go.
    if (s_mm >= len_) return 0.0f;
    const PathSeg &sg = seg_[seg_at(s_mm)];
    if (sg.kappa1 == sg.kappa) return sg.kappa;   // straight or arc
    float u = (s_mm - sg.s0_mm) / sg.len_mm;
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
    return sg.kappa + (sg.kappa1 - sg.kappa) * u;
}

// Rate of change of curvature along the path, 1/mm^2. Zero on straights and
// arcs; constant along a clothoid, which is the whole point of one. The
// executor's angular acceleration inside a clothoid is v^2 * this.
float SpeedPath::dkappa_at(float s_mm) const {
    if (n_ == 0 || s_mm >= len_ || s_mm < 0.0f) return 0.0f;
    const PathSeg &sg = seg_[seg_at(s_mm)];
    if (sg.kappa1 == sg.kappa || sg.len_mm <= 0.0f) return 0.0f;
    return (sg.kappa1 - sg.kappa) / sg.len_mm;
}

PathPose SpeedPath::pose(float s_mm) const {
    PathPose p;
    if (n_ == 0) {
        p.x_mm = x0_;
        p.y_mm = y0_;
        p.yaw_deg = yaw0_rad_ * RAD2DEG;
        return p;
    }
    if (s_mm < 0.0f) s_mm = 0.0f;
    if (s_mm > len_) s_mm = len_;
    int i = seg_at(s_mm);
    const PathSeg &sg = seg_[i];
    float ds = s_mm - sg.s0_mm;
    if (ds > sg.len_mm) ds = sg.len_mm;
    float th0 = sg.yaw0_rad;
    if (sg.kappa != sg.kappa1) {
        // Linear curvature ramp -- a clothoid. Heading is the exact integral
        // of kappa; position comes from ramp_xy().
        float b = (sg.kappa1 - sg.kappa) / (2.0f * sg.len_mm);
        float th = th0 + sg.kappa * ds + b * ds * ds;
        float dx, dy;
        ramp_xy(sg.len_mm, sg.kappa, sg.kappa1, th0, ds, dx, dy);
        p.x_mm = sg.x0_mm + dx;
        p.y_mm = sg.y0_mm + dy;
        p.yaw_deg = wrap180(th * RAD2DEG);
        return p;
    }
    if (sg.kappa == 0.0f) {
        p.x_mm = sg.x0_mm + ds * cosf(th0);
        p.y_mm = sg.y0_mm + ds * sinf(th0);
        p.yaw_deg = wrap180(th0 * RAD2DEG);
        return p;
    }
    // Circular arc. Integrating the unicycle at constant curvature:
    //   x = x0 + (sin(th0 + k*ds) - sin(th0)) / k
    //   y = y0 - (cos(th0 + k*ds) - cos(th0)) / k
    // which degenerates correctly to the straight above as k -> 0.
    float th = th0 + sg.kappa * ds;
    p.x_mm = sg.x0_mm + (sinf(th) - sinf(th0)) / sg.kappa;
    p.y_mm = sg.y0_mm - (cosf(th) - cosf(th0)) / sg.kappa;
    p.yaw_deg = wrap180(th * RAD2DEG);
    return p;
}

float SpeedPath::corner_entry_s_mm(int k) const {
    if (k < 0 || k >= nc_) return 0.0f;
    return seg_[corner_seg_[k]].s0_mm;
}

float SpeedPath::corner_exit_s_mm(int k) const {
    if (k < 0 || k >= nc_) return 0.0f;
    // The LAST segment of the primitive -- with clothoids that is the exit
    // ramp, not the arc, and a caller asking where the corner ends means
    // where the mouse is straight again.
    const PathSeg &s = seg_[corner_seg_[k] + corner_nseg_[k] - 1];
    return s.s0_mm + s.len_mm;
}

float SpeedPath::corner_radius_mm(int k) const {
    if (k < 0 || k >= nc_) return 0.0f;
    // The CENTRAL arc's radius: the tightest curvature the corner reaches,
    // which is what a speed cap or a clearance check wants. For a clothoid
    // corner the first segment starts straight, so read the middle one.
    const PathSeg &s = seg_[corner_seg_[k] + corner_nseg_[k] / 2];
    float kap = (fabsf(s.kappa) > fabsf(s.kappa1)) ? s.kappa : s.kappa1;
    return (kap != 0.0f) ? 1.0f / fabsf(kap) : 0.0f;
}

int SpeedPath::corner_dir(int k) const {
    if (k < 0 || k >= nc_) return 0;
    const PathSeg &s = seg_[corner_seg_[k] + corner_nseg_[k] / 2];
    float kap = (fabsf(s.kappa) > fabsf(s.kappa1)) ? s.kappa : s.kappa1;
    return kap > 0.0f ? 1 : -1;
}

float SpeedPath::corner_tangent_mm(int k) const {
    if (k < 0 || k >= nc_) return 0.0f;
    return corner_t_mm_[k];
}

float SpeedPath::v_cap_at(float s_mm) const {
    float k = fabsf(kappa_at(s_mm));
    const PathSeg *sg = (n_ > 0 && s_mm < len_) ? &seg_[seg_at(s_mm)] : nullptr;
    // THE CLOTHOID HOLD (Path.h): inside a curvature ramp the cap is the
    // tighter end's, so the ramp is driven at the arc's speed -- no brake
    // overlapping the lateral coming on, no launch overlapping it going off.
    if (clothoid_hold_ && sg && sg->kappa != sg->kappa1) {
        float ka = fabsf(sg->kappa), kb = fabsf(sg->kappa1);
        k = (ka > kb) ? ka : kb;
    }
    if (k <= 0.0f) return v_max_;
    float v = sqrtf(a_lat_ / k);
    if (v > v_max_) v = v_max_;
    float cap = sg ? sg->vcap_mm_s : 0.0f;
    if (cap > 0.0f && cap < v) v = cap;
    return v;
}

SpeedPath::Sample SpeedPath::sample(float s_mm, float v_prev_mm_s,
                                    float dt_s) const {
    Sample out;
    out.kappa = kappa_at(s_mm);
    out.in_arc = out.kappa != 0.0f;
    out.at_end = s_mm >= len_;
    out.yaw_deg = pose(s_mm).yaw_deg;

    // 1. Pointwise lateral cap at the curvature the mouse is ON.
    float v = v_cap_at(s_mm);

    // 2. Every upcoming arc is a braking target AT ITS ENTRY POINT. This is
    //    the whole planner: no schedule, no committed decel, just "how fast
    //    may I be here so that I can still be slow enough there". Because it
    //    is re-solved from measured s every tick, a gate correction, a slip,
    //    or a long corner simply changes the answer next tick.
    for (int i = 0; i < n_; i++) {
        const PathSeg &sg = seg_[i];
        // The binding point of a segment is where its curvature is TIGHTEST,
        // which for a clothoid is one end or the other rather than the
        // start. Braking to the segment start would be early on a ramp-in
        // (the curvature is still 0 there) and pointless on a ramp-out.
        float ka = fabsf(sg.kappa), kb = fabsf(sg.kappa1);
        float kmax = (ka > kb) ? ka : kb;
        if (kmax == 0.0f) continue;
        // With the clothoid hold the arc's speed is due at the ramp's START
        // (Path.h): the brake finishes before the lateral load begins.
        float s_at = (kb > ka && !clothoid_hold_) ? (sg.s0_mm + sg.len_mm) : sg.s0_mm;
        float d = s_at - s_mm;
        if (d <= 0.0f) continue;  // passed; the pointwise cap owns it
        // Too far ahead to bind: lim >= sqrt(2 a d) >= v already, and every
        // later arc is further still (segments are in s order) -- stop.
        if (2.0f * a_brk() * d >= v * v) break;
        float v_arc = sqrtf(a_lat_ / kmax);
        if (v_arc > v_max_) v_arc = v_max_;
        if (sg.vcap_mm_s > 0.0f && sg.vcap_mm_s < v_arc) v_arc = sg.vcap_mm_s;
        float lim = sqrtf(v_arc * v_arc + 2.0f * a_brk() * d);
        if (lim < v) v = lim;
    }

    // 3. The path end is a v = 0 constraint.
    float d_end = len_ - s_mm;
    if (d_end < 0.0f) d_end = 0.0f;
    float lim_end = sqrtf(2.0f * a_brk() * d_end);
    if (lim_end < v) v = lim_end;

    // 4. Accel ramp. Only ever limits: braking is allowed to be as sharp as
    //    the constraints above demand, because they were computed with
    //    a_lon in the first place.
    float ramp = v_prev_mm_s + a_lon_ * dt_s;
    if (v > ramp) v = ramp;
    if (v < 0.0f) v = 0.0f;

    out.v_mm_s = v;
    // Rate feedforward: omega = v * kappa, in deg/s. The executor slews this
    // (it steps at every tangent point -- see Path.h) before using it.
    out.omega_dps = v * out.kappa * RAD2DEG;
    return out;
}
