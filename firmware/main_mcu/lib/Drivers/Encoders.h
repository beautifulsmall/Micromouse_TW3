// Quadrature encoder driver (left/right wheel odometry).
//
// The heavy lifting happens in hardware (i.MX RT1062 ENC modules, see
// HAL.h): this layer normalises direction and converts counts to
// millimetres using the geometry from micromouse_spec.md section 6.4.
//
// Convention: forward motion => positive counts/mm on BOTH wheels. The
// motors are mirrored left/right, so one side's raw hardware count runs
// backwards; ENC_LEFT_SIGN / ENC_RIGHT_SIGN normalise that. Verify on the
// bench (push the mouse forward, both readouts must increase) and flip the
// sign constants if needed.

#pragma once

#include <Arduino.h>

// Geometry (spec 6.4): 82-spoke disc on the wheel, x4 quadrature decode
// => 328 counts per wheel revolution; wheel circumference 67.50 mm
// => 0.2058 mm per count.
constexpr float ENC_COUNTS_PER_REV = 328.0f;
constexpr float ENC_WHEEL_CIRCUMFERENCE_MM = 67.50f;
constexpr float ENC_MM_PER_COUNT = ENC_WHEEL_CIRCUMFERENCE_MM / ENC_COUNTS_PER_REV;

// Direction normalisation -- set from bench test 2026-07-02: both raw
// counts ran negative going forward, so both are inverted here.
constexpr int32_t ENC_LEFT_SIGN = -1;
constexpr int32_t ENC_RIGHT_SIGN = 1;

// Configures the hardware decoders and zeroes both counts.
void encoders_init();

// Direction-normalised counts since last zero (forward = positive).
int32_t encoder_left_counts();
int32_t encoder_right_counts();

// Zero both counters (e.g. at the start of a run).
void encoders_zero();

// Counts -> millimetres.
inline float encoder_mm(int32_t counts) { return counts * ENC_MM_PER_COUNT; }
