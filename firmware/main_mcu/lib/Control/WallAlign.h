// WallAlign: wall-referenced IMU heading correction.
//
// The mouse STEERS on the IMU (stable over a straight, ~0.01 deg/s drift) and
// uses the walls as ground truth to occasionally re-reference it. While moving
// beside a wall, (distance, lateral offset) samples are accumulated and
// least-squares fitted:  lateral = a + b * dist.  The slope is the sine of the
// travel direction relative to the wall, so
//
//   heading error to the wall  eps = asin(b)   (+CCW: drifting toward the
//                                               LEFT wall means eps > 0)
//
// and the intercept gives the lateral offset for a position fix. Corrections
// are applied BETWEEN moves (control_set_heading_deg) -- never mid-move, which
// would step the heading loop.
//
// Boot alignment: the mouse may be placed slightly misaligned. The first move
// (start cell -> cell 2) is always forward with a continuous double wall on
// the left, so this fit runs there and re-references the IMU to the maze.
//
// Outlier gate: posts are 12x12 mm and FLUSH with the walls, so a continuous
// wall reads as one unbroken surface -- no blanking needed when passing a
// post. The gate exists for MISSING wall segments (open cell to the side):
// there the beam sees the next corridor's wall (a big lateral jump) or drops
// out. Once the fit has WALL_ALIGN_GATE_MIN_N samples, a new sample whose
// residual from the current fit exceeds WALL_ALIGN_RESID_MM is rejected
// (counted, not added).
//
// Pure logic (running least-squares sums), no HAL -- host-tested in
// test_wall_align.cpp.

#pragma once

#include <stdint.h>

// Fit acceptance: at least this many samples spanning at least this distance.
constexpr int WALL_ALIGN_MIN_N = 20;
constexpr float WALL_ALIGN_MIN_SPAN_MM = 100.0f;

// Outlier gate (see above).
constexpr int WALL_ALIGN_GATE_MIN_N = 8;
constexpr float WALL_ALIGN_RESID_MM = 4.0f;

// Corrections larger than this are implausible (bad wall data, wrong wall) --
// callers should refuse to apply them.
constexpr float WALL_ALIGN_MAX_CORR_DEG = 10.0f;

class WallAlign {
public:
    void reset();

    // Add one sample: fused distance since move start (mm) and the wall
    // lateral offset (mm, +ve toward the LEFT wall). Caller gates on
    // WallView::lateral_valid. Returns false if the outlier gate rejected it.
    bool add(float dist_mm, float lateral_mm);

    // True when the fit is usable (WALL_ALIGN_MIN_N / MIN_SPAN reached).
    bool valid() const;

    float slope() const;          // b: d(lateral)/d(dist)
    float yaw_err_deg() const;    // asin(slope), +CCW relative to the wall
    float offset_mm(float at_dist_mm) const;  // fitted lateral at a distance

    int samples() const { return n_; }
    int rejected() const { return rejected_; }
    float span_mm() const { return n_ ? x_max_ - x_min_ : 0.0f; }

private:
    // Running least-squares sums, x shifted by the first sample for
    // conditioning (distances can be large relative to the span).
    float x0_ = 0.0f;
    float sx_ = 0.0f, sy_ = 0.0f, sxx_ = 0.0f, sxy_ = 0.0f;
    float x_min_ = 0.0f, x_max_ = 0.0f;
    int n_ = 0;
    int rejected_ = 0;

    bool fit(float &a, float &b) const;  // false if degenerate
};
