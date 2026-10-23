#include "WallAlign.h"

#include <math.h>

static constexpr float RAD2DEG_WA = 57.29577951308232f;

void WallAlign::reset() {
    x0_ = sx_ = sy_ = sxx_ = sxy_ = 0.0f;
    x_min_ = x_max_ = 0.0f;
    n_ = 0;
    rejected_ = 0;
}

bool WallAlign::fit(float &a, float &b) const {
    if (n_ < 2) return false;
    float det = n_ * sxx_ - sx_ * sx_;
    if (fabsf(det) < 1e-6f) return false;  // no x spread
    b = (n_ * sxy_ - sx_ * sy_) / det;
    a = (sy_ - b * sx_) / n_;  // intercept in shifted-x frame
    return true;
}

bool WallAlign::add(float dist_mm, float lateral_mm) {
    if (n_ == 0) x0_ = dist_mm;
    float x = dist_mm - x0_;

    // Outlier gate: once the fit is established, reject samples far off it.
    if (n_ >= WALL_ALIGN_GATE_MIN_N) {
        float a, b;
        if (fit(a, b) && fabsf(lateral_mm - (a + b * x)) > WALL_ALIGN_RESID_MM) {
            rejected_++;
            return false;
        }
    }

    sx_ += x;
    sy_ += lateral_mm;
    sxx_ += x * x;
    sxy_ += x * lateral_mm;
    if (n_ == 0) {
        x_min_ = x_max_ = dist_mm;
    } else {
        if (dist_mm < x_min_) x_min_ = dist_mm;
        if (dist_mm > x_max_) x_max_ = dist_mm;
    }
    n_++;
    return true;
}

bool WallAlign::valid() const {
    return n_ >= WALL_ALIGN_MIN_N && span_mm() >= WALL_ALIGN_MIN_SPAN_MM;
}

float WallAlign::slope() const {
    float a, b;
    if (!fit(a, b)) return 0.0f;
    return b;
}

float WallAlign::yaw_err_deg() const {
    float b = slope();
    if (b > 1.0f) b = 1.0f;
    if (b < -1.0f) b = -1.0f;
    return asinf(b) * RAD2DEG_WA;
}

float WallAlign::offset_mm(float at_dist_mm) const {
    float a, b;
    if (!fit(a, b)) return 0.0f;
    return a + b * (at_dist_mm - x0_);
}
