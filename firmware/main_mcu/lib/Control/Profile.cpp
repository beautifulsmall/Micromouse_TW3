#include "Profile.h"

#include <math.h>

void TrapProfile::begin(float distance_mm, float v_cruise_mm_s,
                        float a_max_mm_s2) {
    sign_ = distance_mm < 0.0f ? -1.0f : 1.0f;
    dist_ = fabsf(distance_mm);
    a_ = fabsf(a_max_mm_s2);
    float vc = fabsf(v_cruise_mm_s);

    // Degenerate requests -> an already-parked profile (sample() returns the
    // target at rest for any t).
    if (dist_ <= 0.0f || a_ <= 0.0f || vc <= 0.0f) {
        vp_ = 0.0f;
        d_acc_ = 0.0f;
        t_acc_ = t_cruise_ = t_total_ = 0.0f;
        return;
    }

    // Distance to reach the cruise speed (accel phase alone).
    float d_to_cruise = vc * vc / (2.0f * a_);

    if (2.0f * d_to_cruise <= dist_) {
        // Full trapezoid: accelerate to vc, cruise, decelerate.
        vp_ = vc;
        d_acc_ = d_to_cruise;
        t_acc_ = vc / a_;
        float d_cruise = dist_ - 2.0f * d_acc_;
        t_cruise_ = d_cruise / vc;
    } else {
        // Triangle: never reach vc. Peak vp where 2 * vp^2/(2a) = dist_.
        vp_ = sqrtf(a_ * dist_);
        d_acc_ = dist_ * 0.5f;
        t_acc_ = vp_ / a_;
        t_cruise_ = 0.0f;
    }
    t_total_ = 2.0f * t_acc_ + t_cruise_;
}

bool TrapProfile::extend(float extra_mm, float t_now_s) {
    float extra = fabsf(extra_mm);
    if (extra <= 0.0f) return false;
    if (vp_ <= 0.0f || t_total_ <= 0.0f) return false;  // degenerate/parked
    if (t_now_s >= t_acc_ + t_cruise_) return false;    // decel committed
    // Insert cruise time at the peak speed; the decel phase is unchanged,
    // just later. For a triangular profile this cruises at the triangle's
    // peak (slightly under v_cruise) -- correct, if not time-optimal.
    dist_ += extra;
    t_cruise_ += extra / vp_;
    t_total_ += extra / vp_;
    return true;
}

ProfileSample TrapProfile::sample(float t_s) const {
    ProfileSample s;

    if (t_total_ <= 0.0f || t_s >= t_total_) {
        // Parked at the target (also covers the degenerate profile).
        s.v_ref = 0.0f;
        s.a_ref = 0.0f;
        s.d_ref = sign_ * dist_;
        s.done = true;
        return s;
    }
    if (t_s <= 0.0f) {
        s.v_ref = 0.0f;
        s.a_ref = sign_ * a_;  // about to launch into the accel phase
        s.d_ref = 0.0f;
        s.done = false;
        return s;
    }

    float v, a, d;
    if (t_s < t_acc_) {
        // Accel phase.
        a = a_;
        v = a_ * t_s;
        d = 0.5f * a_ * t_s * t_s;
    } else if (t_s < t_acc_ + t_cruise_) {
        // Cruise phase.
        a = 0.0f;
        v = vp_;
        d = d_acc_ + vp_ * (t_s - t_acc_);
    } else {
        // Decel phase.
        float td = t_s - t_acc_ - t_cruise_;
        a = -a_;
        v = vp_ - a_ * td;
        if (v < 0.0f) v = 0.0f;
        d = d_acc_ + vp_ * t_cruise_ + vp_ * td - 0.5f * a_ * td * td;
    }

    s.v_ref = sign_ * v;
    s.a_ref = sign_ * a;
    s.d_ref = sign_ * d;
    s.done = false;
    return s;
}
