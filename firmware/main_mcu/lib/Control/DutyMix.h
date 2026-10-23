// THE DUTY MIXER (2026-10-01; bert, after MR2-MR5 all read "duty peak 0.85
// of 0.85 << SATURATED": "keep the straight-line cap at 0.85 but allow + and
// - that for steering, so one wheel could go to 100 but not both").
//
// Before: each wheel's duty was clipped at dmax on its own. With both wheels
// asking for more than dmax -- every launch and every brake at the top of a
// rung -- both sat at 0.85 and the DIFFERENCE between them, which is the
// only thing that steers, was zero. The 09-30 MR5 took a 300 deg/s yaw kick
// accelerating through ~940 mm/s on the long diagonal and held 12-13 deg of
// heading error for 150 mm while the speed climbed, then hit a post: there
// was no duty left to turn with.
//
// Now the two duties are split into common mode (the mean: speed) and
// differential (half the difference: steering). The COMMON MODE is capped at
// dmax -- the straight-line motor protection, unchanged -- and each WHEEL at
// dwheel. If a wheel would pass dwheel, the common mode gives way, never the
// steering: a mouse that cannot hold its line at full throttle should go a
// little slower, not off the line.
//
// dwheel = 0 is the old per-wheel clip at dmax, exactly (the A/B switch).
// dwheel at or below dmax still mixes (steering first) inside dmax.
//
// Header-only and free of the HAL so the host tests can drive it.
#pragma once

#include <math.h>

struct DutyMixOut {
    float left, right;
    float common;      // |common mode| applied
};

inline DutyMixOut duty_mix(float l, float r, float cm_max, float wheel_max_param) {
    DutyMixOut o;
    if (wheel_max_param <= 0.0f) {
        // The old law: each wheel clipped on its own.
        if (l > cm_max) l = cm_max;
        if (l < -cm_max) l = -cm_max;
        if (r > cm_max) r = cm_max;
        if (r < -cm_max) r = -cm_max;
        o.left = l; o.right = r; o.common = fabsf(0.5f * (l + r));
        return o;
    }
    float wheel_max = wheel_max_param > cm_max ? wheel_max_param : cm_max;
    if (wheel_max > 1.0f) wheel_max = 1.0f;
    float m = 0.5f * (l + r);
    float d = 0.5f * (r - l);
    if (d > wheel_max) d = wheel_max;
    if (d < -wheel_max) d = -wheel_max;
    if (m > cm_max) m = cm_max;
    if (m < -cm_max) m = -cm_max;
    float ad = fabsf(d);
    if (m + ad > wheel_max) m = wheel_max - ad;     // steering first
    if (m - ad < -wheel_max) m = -wheel_max + ad;
    o.left = m - d;
    o.right = m + d;
    o.common = fabsf(m);
    return o;
}
