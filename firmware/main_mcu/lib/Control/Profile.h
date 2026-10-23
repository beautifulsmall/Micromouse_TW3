// Trapezoidal motion profile generator.
//
// Turns a move request (distance, cruise speed, accel limit) into smooth
// per-tick velocity AND acceleration references. The velocity controller
// tracks v_ref with feedforward + PD as before; a_ref is the extra term the
// acceleration feedforward (ControlGains::ka) uses to pre-empt the inertia of
// a ramp instead of waiting for the PD to build error.
//
// Shape (symmetric accel/decel): a ramp up to the cruise speed vp, a flat
// cruise, then a mirror-image ramp down to rest.
//
// If the move is too short to reach the cruise speed the profile degrades to a
// triangle (t_cruise = 0, peak speed vp = sqrt(a*D) < v_cruise) -- the integral
// still equals the requested distance exactly.
//
// Pure math, no Arduino/HAL dependency, so it is exercised host-side in
// test/test_profile.cpp. All quantities are signed by the move direction:
// a negative distance yields negative v_ref/a_ref/d_ref throughout.

#pragma once

#include <stdint.h>

struct ProfileSample {
    float v_ref;  // mm/s, commanded velocity
    float a_ref;  // mm/s^2, commanded acceleration (+accel, -decel, 0 cruise)
    float d_ref;  // mm, commanded distance travelled so far
    bool done;    // true once t >= total duration (at rest at the target)
};

class TrapProfile {
   public:
    // Plan a move of `distance_mm` (signed) capped at `v_cruise_mm_s` with
    // accel/decel magnitude `a_max_mm_s2`. v_cruise and a_max are treated as
    // magnitudes (their sign is ignored). A zero/degenerate request yields an
    // already-done profile that parks at the target.
    void begin(float distance_mm, float v_cruise_mm_s, float a_max_mm_s2);

    // Sample the profile at elapsed time `t_s` seconds from begin(). Clamped:
    // t <= 0 sits at the start, t >= duration() sits at the target.
    ProfileSample sample(float t_s) const;

    // Extend the move by `extra_mm` (magnitude, same direction) WITHOUT
    // stopping: inserts cruise time at the current peak speed, pushing the
    // decel phase out. Only legal while the profile is still in its accel
    // or cruise phase at time `t_now_s` -- once decel has begun the stop is
    // committed (returns false, profile unchanged). This is what lets the
    // maze search chain cell after cell into one continuous run, deciding
    // one cell ahead of the braking point.
    bool extend(float extra_mm, float t_now_s);

    float duration_s() const { return t_total_; }
    float peak_velocity_mm_s() const { return sign_ * vp_; }
    float target_mm() const { return sign_ * dist_; }

   private:
    float sign_ = 1.0f;   // move direction
    float dist_ = 0.0f;   // |distance|
    float a_ = 0.0f;      // |accel|
    float vp_ = 0.0f;     // peak (cruise) speed reached, magnitude
    float d_acc_ = 0.0f;  // |distance| covered during the accel phase
    float t_acc_ = 0.0f;
    float t_cruise_ = 0.0f;
    float t_total_ = 0.0f;
};

// Auto-fit for the acceleration feedforward gain (ControlGains::ka).
//
// During a profiled run, whatever accel duty the current ka does not
// provide, the P term must supply once error has built:
//
//     kp * err  ~=  (ka_true - ka) * a_ref
//
// so least squares over the ramp samples gives the correction directly:
//
//     ka_true = ka + kp * sum(err * a_ref) / sum(a_ref^2)
//
// Feed it (a_ref, v_ref - v_enc) pairs sampled during BOTH ramps (a_ref of
// both signs, skipping cruise and any slip-flagged ticks): error terms that
// correlate with velocity instead of acceleration -- vmax or friction
// mismatch -- have the same sign in both ramps while a_ref flips, so they
// cancel out of the fit. The D term is ignored (kd is normally 0 while
// tuning ka). One run lands close; a second run verifies / polishes.
class KaTuner {
   public:
    void reset() {
        s_aa_ = s_ae_ = 0.0f;
        n_ = 0;
    }
    void add(float a_ref, float v_err) {
        s_aa_ += a_ref * a_ref;
        s_ae_ += a_ref * v_err;
        n_++;
    }
    int samples() const { return n_; }
    // Enough ramp samples for a meaningful fit?
    bool valid() const { return n_ >= 20 && s_aa_ > 1.0f; }
    // The ka this run implies, given the gains it ran with.
    float suggest(float ka_now, float kp) const {
        return ka_now + kp * s_ae_ / s_aa_;
    }

   private:
    float s_aa_ = 0.0f;  // sum a_ref^2
    float s_ae_ = 0.0f;  // sum a_ref * err
    int n_ = 0;
};

// Auto-fit for the effective wheel track (ControlGains::track_mm), used only
// as the feedforward that converts a commanded body rotation rate into a
// wheel-speed differential during a turn.
//
// It is a direct kinematic measurement, independent of any control gain: for
// pure rotation the wheels counter-rotate about the centre, so
//
//     0.5 * (v_right - v_left)  =  omega * (track / 2)
//
// with v_* the encoder wheel speeds (mm/s) and omega the gyro yaw rate
// (rad/s). Least squares of the measured differential against omega gives the
// half-track slope directly; track = 2 * slope. Because it reads encoders vs
// gyro (not the loop's own command), it captures the real EFFECTIVE track --
// tyre scrub during an in-place turn makes it larger than the 59 mm geometric
// contact separation. Feed samples only where |omega| is well above noise and
// no slip is flagged.
class TrackTuner {
   public:
    void reset() {
        s_ww_ = s_wd_ = 0.0f;
        n_ = 0;
    }
    // omega in rad/s (gyro), wheel_diff = 0.5*(v_right - v_left) in mm/s.
    void add(float omega_rad_s, float wheel_diff_mm_s) {
        s_ww_ += omega_rad_s * omega_rad_s;
        s_wd_ += omega_rad_s * wheel_diff_mm_s;
        n_++;
    }
    int samples() const { return n_; }
    bool valid() const { return n_ >= 20 && s_ww_ > 0.01f; }
    // Effective track in mm implied by this turn (slope = half-track).
    float suggest_track_mm() const { return 2.0f * s_wd_ / s_ww_; }

   private:
    float s_ww_ = 0.0f;  // sum omega^2
    float s_wd_ = 0.0f;  // sum omega * wheel_diff
    int n_ = 0;
};
