#include "WallGate.h"

#include <math.h>

void WallGate::reset_runs() {
    present_run_ = 0;
    absent_run_ = 0;
    step_run_ = 0;
    track_mm_ = 255.0f;
    present_s0_ = 0.0f;
    present_s1_ = 0.0f;
}

void WallGate::arm(const WallGateConfig &cfg) {
    cfg_ = cfg;
    armed_ = true;
    fired_ = false;
    drop_s_ = 0.0f;
    lat_at_wall_ = 0.0f;
    reset_runs();
    for (int i = 0; i < MAX_RUNS; i++) runs_[i] = Run{};
    cur_ = Run{};
    in_run_ = false;
    runs_n_ = 0;
    runs_seen_ = 0;
}

// One sample into the feature observer. A "run" is an unbroken stretch of
// the beam seeing something near enough to be a wall or a post. Closing a
// run is what records it -- so a run still open when the corner ends is
// deliberately not stored: it has no end, and its length is the one thing
// the observer exists to measure.
void WallGate::observe(float s_mm, bool valid, float d) {
    bool near = valid && d <= (float)cfg_.present_mm;
    if (near) {
        if (!in_run_) {
            in_run_ = true;
            cur_.s0_mm = s_mm;
            cur_.min_mm = 255;
            cur_.n = 0;
        }
        cur_.s1_mm = s_mm;
        if (cur_.n < 255) cur_.n++;
        if (d < (float)cur_.min_mm) cur_.min_mm = (uint8_t)d;
        return;
    }
    if (!in_run_) return;
    // Do not close on the first miss. A run stays open across a gap of up to
    // BRIDGE_MM, because a dropout inside a feature is not the end of it --
    // see the note there for the measurement that sets the number.
    if (s_mm - cur_.s1_mm <= BRIDGE_MM) return;
    in_run_ = false;
    runs_seen_++;
    if (runs_n_ < MAX_RUNS) runs_[runs_n_++] = cur_;
}

WallGate::Result WallGate::update(float s_mm, uint8_t inside_raw_mm,
                                  bool lat_valid, float lat_mm,
                                  float *corr_mm) {
    if (!armed_ || fired_) return NONE;

    // 0 is an IC error, 255 is "no valid measurement" -- neither is a
    // distance, and neither is evidence of an opening on its own.
    bool valid = inside_raw_mm != 255 && inside_raw_mm != 0;
    float d = (float)inside_raw_mm;

    // OBSERVE FIRST, and unconditionally. The observer must see the whole
    // approach including whatever happens before the gate is allowed to
    // decide -- that is the entire point of it.
    observe(s_mm, valid, d);
    if (s_mm < cfg_.live_from_s_mm) return NONE;

    // Is the beam still on the wall we have been tracking? "Still on it"
    // means valid AND not stepped out beyond the tracked minimum. The
    // presence threshold only has to hold while ACQUIRING the wall; once
    // tracking, the step test owns the decision, because a wall seen at a
    // shallow angle can read past the presence threshold without ending.
    bool tracking;
    if (present_run_ == 0) {
        tracking = valid && d <= (float)cfg_.present_mm;
    } else {
        tracking = valid && d < track_mm_ + cfg_.step_mm;
    }

    if (tracking) {
        if (present_run_ == 0) present_s0_ = s_mm;
        present_s1_ = s_mm;
        present_run_++;
        // A return from the wall while drop evidence was pending is the
        // wall-post GAP being crossed (2 mm of construction between the
        // wall's end and the post): the same feature continues, so the
        // pending evidence is discarded and the run keeps its history.
        absent_run_ = 0;
        step_run_ = 0;
        if (d < track_mm_) track_mm_ = d;
        // Remember the lateral seen WITH the wall still there -- see the
        // off-centre guard note in WallGate.h.
        if (lat_valid) lat_at_wall_ = lat_mm;
        return NONE;
    }

    // Not on the wall. Only meaningful if we had actually acquired it: an
    // opening can only be the END of something. Acquisition is a SPAN of
    // travel plus a packet floor, so it means the same wall at every speed
    // -- see the note at present_span_mm.
    if (present_run_ < cfg_.present_n ||
        present_s1_ - present_s0_ < cfg_.present_span_mm) {
        reset_runs();
        return NONE;
    }

    // First evidence stamps the position; confirmation only decides whether
    // to believe it. Taking s at confirmation instead would bias every
    // correction by the confirmation distance.
    if (absent_run_ == 0 && step_run_ == 0) drop_s_ = s_mm;

    // Confirmation needs the evidence packets AND the evidence to persist
    // drop_confirm_mm of travel past the stamp -- the distance is what
    // stops the 2 mm wall-post gap confirming at slow approach speeds
    // (see the note at drop_confirm_mm).
    bool packets_ok;
    if (valid) {
        step_run_++;                              // strong: beam stepped out
        packets_ok = step_run_ >= cfg_.step_n;
    } else {
        absent_run_++;                            // weak: non-return
        packets_ok = absent_run_ >= cfg_.absent_n;
    }
    if (!packets_ok || s_mm - drop_s_ < cfg_.drop_confirm_mm) return NONE;

    // Lateral compensation: offset toward the inside wall shortens the
    // beam's lead, so the drop is observed late by klat x lat -- undo it
    // before judging the error (see the note at klat; lat arrives signed
    // toward-the-inside from the caller).
    float drop_eff = drop_s_ - cfg_.klat * lat_at_wall_;
    float err = cfg_.expect_s_mm - drop_eff;
    Result r;
    if (fabsf(lat_at_wall_) >= cfg_.lat_max_mm) {
        // Skip WITHOUT consuming the gate: the presence debounce restarts
        // and a later clean stretch may still anchor this corner. V1 burned
        // a real gate here by treating a skip as a use (2026-07-18).
        r = SKIPPED_LAT;
    } else if (fabsf(err) >= cfg_.window_mm) {
        r = SKIPPED_WINDOW;
    } else {
        if (corr_mm) *corr_mm = err;
        fired_ = true;
        r = CORRECTION;
    }
    reset_runs();
    return r;
}
