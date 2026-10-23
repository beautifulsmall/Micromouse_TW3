// 6-DOF orientation filter: Madgwick gradient-descent fusion of the
// ICM-45686 accel + gyro (no magnetometer).
//
// Purpose (per micromouse_spec.md section on the IMU): true heading for
// straight-line correction and turn accuracy. Fusing accel with gyro keeps
// the gravity (tilt) reference locked so heading is measured about the
// true vertical, not the body Z axis -- integrating gz alone under-reads
// heading by cos(tilt) (about 1.5% at 10 degrees of tilt).
//
// Frames and conventions:
//   - Body frame = IMU frame = mouse frame: X forward, Y left, Z up
//     (right-handed; IMU is mounted at the centre of rotation).
//   - At rest the accelerometer reads +1 g on Z.
//   - Yaw/heading is positive counterclockwise viewed from above
//     (a left turn increases yaw), reported wrapped to (-180, +180].
//
// Yaw observability: with no magnetometer the accelerometer corrects
// pitch/roll only; yaw is bias-corrected gyro integration about the true
// vertical. Gyro Z bias is measured at startup (mouse must be stationary),
// and fusion_set_yaw_deg() lets higher layers re-reference heading from
// maze-wall ground truth between runs / down straights.
//
// This module is pure math -- no HAL, no Arduino -- so it is host-testable
// (see test/test_fusion.cpp). It is NOT interrupt-safe by itself: all
// functions are expected to be called from the control ISR, or from code
// that masks the control ISR (Control.cpp wraps the main-loop accessors).

#pragma once

// One filter update per IMU sample: the nominal sample interval is the IMU
// ODR set in ImuIcm45686.cpp. Keep in sync if the ODR changes.
constexpr float FUSION_SAMPLE_HZ = 6400.0f;
constexpr float FUSION_SAMPLE_DT = 1.0f / FUSION_SAMPLE_HZ;

// Madgwick filter gain. Trades gyro trust against accel trust for the tilt
// reference. The gyro is excellent when calibrated, and the mouse spends
// its time accelerating (which corrupts the gravity reference), so lean on
// the gyro: 0.033 is the paper's optimum for 6-DOF. Raise it if tilt drifts;
// lower it if pitch/roll wobble with translational acceleration.
constexpr float FUSION_BETA = 0.033f;

// Settle delay: samples discarded after fusion_reset() before bias
// accumulation starts (2 s at 6.4 kHz), letting the mouse stop rocking
// after being placed / powered on.
constexpr int FUSION_SETTLE_SAMPLES = 2 * 6400;

// Gyro bias calibration: samples averaged (1 s at 6.4 kHz), and the motion
// guard -- any |rate| above this restarts the accumulation, so calibration
// only completes once the mouse has been genuinely still for the window.
constexpr int FUSION_CAL_SAMPLES = 6400;
constexpr float FUSION_CAL_MAX_DPS = 10.0f;

// Reset the filter and start the settle delay + gyro bias calibration.
// The mouse must be stationary until fusion_ready() returns true (~3 s:
// 2 s settle + 1 s bias window). Call this LAST during startup once other
// initialisations exist, so their mechanical/electrical disturbance is over
// before the bias window runs.
void fusion_reset();

// Feed one IMU sample (gyro in dps, accel in g, body frame as above).
// During calibration this accumulates bias; afterwards it runs Madgwick
// with dt = FUSION_SAMPLE_DT. Cost: ~40 flops + 2 sqrt -- negligible on
// Teensy 4.0 even at 13 samples per 500 Hz tick.
void fusion_update(float gx_dps, float gy_dps, float gz_dps,
                   float ax_g, float ay_g, float az_g);

// False while gyro bias calibration is still accumulating.
bool fusion_ready();

// Euler angles, degrees. Yaw wrapped to (-180, +180], +CCW from above.
float fusion_yaw_deg();
float fusion_pitch_deg();  // +nose up
float fusion_roll_deg();   // +right side down

// Re-reference heading to external ground truth (maze walls): rotates the
// attitude about the true vertical so yaw reads `yaw_deg`; pitch/roll are
// untouched.
void fusion_set_yaw_deg(float yaw_deg);

// Calibrated gyro bias estimate, dps (x, y, z). Valid once fusion_ready().
void fusion_gyro_bias_dps(float out[3]);

// Gravity unit vector in the body frame from the current attitude (what a
// perfect accelerometer would read at rest, in g). Subtract it from the
// accel reading to get true body acceleration: a = g0 * (f - gravity).
// Identity attitude returns (0, 0, 1).
void fusion_gravity_body(float out[3]);
