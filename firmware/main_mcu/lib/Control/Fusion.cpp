#include "Fusion.h"

#include <math.h>

// Implementation of Madgwick's IMU (6-DOF, no magnetometer) gradient-descent
// filter, from "An efficient orientation filter for inertial and
// inertial/magnetic sensor arrays" (Madgwick, 2010), MadgwickAHRSupdateIMU.
// The quaternion q = (q0, q1, q2, q3) rotates the earth frame into the body
// frame; earth Z is up and the accelerometer reads +1 g on body Z at rest,
// which matches the paper's convention -- no axis remapping needed for our
// X-forward / Y-left / Z-up mounting.

static constexpr float DEG2RAD = 0.017453292519943295f;
static constexpr float RAD2DEG = 57.29577951308232f;

// -- Filter state ------------------------------------------------------------
static float q0, q1, q2, q3;

// -- Gyro bias calibration state ---------------------------------------------
static bool ready;
static int settle_count;
static int cal_count;
static float gyro_sum[3];   // dps
static float accel_sum[3];  // g
static float bias[3];       // dps

// True 1/sqrt. Teensy 4.0 (Cortex-M7) has a hardware FPU with fast vsqrt;
// the classic fast-inverse-sqrt hack costs accuracy for no real gain here.
static inline float inv_sqrt(float x) { return 1.0f / sqrtf(x); }

void fusion_reset() {
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    ready = false;
    settle_count = 0;
    cal_count = 0;
    gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0f;
    accel_sum[0] = accel_sum[1] = accel_sum[2] = 0.0f;
    bias[0] = bias[1] = bias[2] = 0.0f;
}

bool fusion_ready() { return ready; }

void fusion_gyro_bias_dps(float out[3]) {
    out[0] = bias[0]; out[1] = bias[1]; out[2] = bias[2];
}

// Finish calibration: average the sums into the bias, and initialise the
// quaternion tilt from the averaged gravity vector (yaw = 0 by definition;
// walls give the real reference later).
static void cal_finish() {
    bias[0] = gyro_sum[0] / FUSION_CAL_SAMPLES;
    bias[1] = gyro_sum[1] / FUSION_CAL_SAMPLES;
    bias[2] = gyro_sum[2] / FUSION_CAL_SAMPLES;

    float ax = accel_sum[0] / FUSION_CAL_SAMPLES;
    float ay = accel_sum[1] / FUSION_CAL_SAMPLES;
    float az = accel_sum[2] / FUSION_CAL_SAMPLES;

    // Tilt from gravity (ZYX convention, yaw = 0):
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    // q = qy(pitch) * qx(roll)
    float cy = cosf(pitch * 0.5f), sy = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    q0 = cy * cr;
    q1 = cy * sr;
    q2 = sy * cr;
    q3 = -sy * sr;

    ready = true;
}

static void cal_update(float gx, float gy, float gz,
                       float ax, float ay, float az) {
    // Motion guard: rates beyond any plausible bias mean the mouse moved --
    // restart so the window only completes after genuine stillness.
    if (fabsf(gx) > FUSION_CAL_MAX_DPS || fabsf(gy) > FUSION_CAL_MAX_DPS ||
        fabsf(gz) > FUSION_CAL_MAX_DPS) {
        cal_count = 0;
        gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0f;
        accel_sum[0] = accel_sum[1] = accel_sum[2] = 0.0f;
        return;
    }
    gyro_sum[0] += gx; gyro_sum[1] += gy; gyro_sum[2] += gz;
    accel_sum[0] += ax; accel_sum[1] += ay; accel_sum[2] += az;
    if (++cal_count >= FUSION_CAL_SAMPLES) cal_finish();
}

void fusion_update(float gx_dps, float gy_dps, float gz_dps,
                   float ax_g, float ay_g, float az_g) {
    if (!ready) {
        // Settle delay: discard samples while the mouse stops rocking after
        // placement/power-on, then accumulate the bias window.
        if (settle_count < FUSION_SETTLE_SAMPLES) {
            settle_count++;
            return;
        }
        cal_update(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);
        return;
    }

    float gx = (gx_dps - bias[0]) * DEG2RAD;
    float gy = (gy_dps - bias[1]) * DEG2RAD;
    float gz = (gz_dps - bias[2]) * DEG2RAD;
    float ax = ax_g, ay = ay_g, az = az_g;

    // Rate of change of quaternion from gyroscope.
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Gradient-descent corrective step from the accelerometer, skipped if
    // the accel reading is invalid (all zeros).
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        float recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay -
                   _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay -
                   _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        // Guard: when the attitude already agrees exactly with gravity the
        // gradient is zero and normalising it would produce inf * 0 = NaN.
        float snorm2 = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (snorm2 > 1e-12f) {
            recipNorm = inv_sqrt(snorm2);
            s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

            qDot1 -= FUSION_BETA * s0;
            qDot2 -= FUSION_BETA * s1;
            qDot3 -= FUSION_BETA * s2;
            qDot4 -= FUSION_BETA * s3;
        }
    }

    q0 += qDot1 * FUSION_SAMPLE_DT;
    q1 += qDot2 * FUSION_SAMPLE_DT;
    q2 += qDot3 * FUSION_SAMPLE_DT;
    q3 += qDot4 * FUSION_SAMPLE_DT;

    float recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void fusion_gravity_body(float out[3]) {
    // Third row of the earth->body rotation: the same predicted-gravity
    // expression Madgwick's objective function minimises against.
    out[0] = 2.0f * (q1 * q3 - q0 * q2);
    out[1] = 2.0f * (q0 * q1 + q2 * q3);
    out[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

// -- Euler extraction (ZYX) --------------------------------------------------
float fusion_yaw_deg() {
    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;
}

float fusion_pitch_deg() {
    float s = 2.0f * (q0 * q2 - q3 * q1);
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    return asinf(s) * RAD2DEG;
}

float fusion_roll_deg() {
    return atan2f(2.0f * (q0 * q1 + q2 * q3),
                  1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
}

void fusion_set_yaw_deg(float yaw_deg) {
    // Pre-multiply by a rotation about earth (true vertical) Z. In the ZYX
    // decomposition q = qz(yaw)*qy(pitch)*qx(roll), this changes only yaw.
    float delta = (yaw_deg - fusion_yaw_deg()) * DEG2RAD;
    float c = cosf(delta * 0.5f), s = sinf(delta * 0.5f);
    float n0 = c * q0 - s * q3;
    float n1 = c * q1 - s * q2;
    float n2 = c * q2 + s * q1;
    float n3 = c * q3 + s * q0;
    q0 = n0; q1 = n1; q2 = n2; q3 = n3;
}
