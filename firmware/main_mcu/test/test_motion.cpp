// Host-side test: fused forward-motion estimator (Control.h) -- grip,
// wheelspin, and recovery. Mocks the HAL, the wheel plant, AND
// imu_read_fifo (so the estimator can be fed accel that agrees or
// disagrees with the encoders). Build (one line; note ImuIcm45686.cpp is
// NOT linked -- this test provides its own imu_read_fifo):
//
//   g++ -std=c++17 -Istubs -I../lib/Control -I../lib/Drivers -I../lib/HAL
//     ../lib/Control/Control.cpp ../lib/Control/Fusion.cpp
//     ../lib/Drivers/Encoders.cpp ../lib/Drivers/Motors.cpp
//     test_motion.cpp -o test_motion && ./test_motion

#include "Control.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

// -- Arduino stub definitions -------------------------------------------------
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void delay(unsigned long) {}
unsigned long millis() { return 0; }
void noInterrupts() {}
void interrupts() {}

// -- Mock HAL: motors ----------------------------------------------------------
static float duty_ch[2] = {0, 0};
static bool standby = true;
void hal_motors_init() {}
void hal_motor_standby(bool s) { standby = s; }
void hal_motor_set(uint8_t channel, float duty) { duty_ch[channel] = duty; }

// -- Mock HAL: encoders + wheel plant -------------------------------------------
static float plant_vel[2] = {0, 0};  // mm/s
static double plant_mm[2] = {0, 0};
static float plant_acc = 0;  // body accel this tick, mm/s^2 (both wheels)
static constexpr float PLANT_LAG_ALPHA = 0.1f;

static float wheel_duty(int w) {
    return w == 0 ? duty_ch[MOTOR_LEFT_CH] * MOTOR_LEFT_SIGN
                  : duty_ch[MOTOR_RIGHT_CH] * MOTOR_RIGHT_SIGN;
}

static void plant_step() {
    float v_before = 0.5f * (plant_vel[0] + plant_vel[1]);
    for (int w = 0; w < 2; w++) {
        float vmax = w == 0 ? VCTRL_VMAX_LEFT_MM_S : VCTRL_VMAX_RIGHT_MM_S;
        float fric = w == 0 ? VCTRL_FRICTION_LEFT : VCTRL_FRICTION_RIGHT;
        float duty = standby ? 0.0f : wheel_duty(w);
        float mag = std::fabs(duty) - fric;
        float v_ss = mag > 0 ? vmax * mag * (duty > 0 ? 1 : -1) : 0;
        plant_vel[w] += PLANT_LAG_ALPHA * (v_ss - plant_vel[w]);
        plant_mm[w] += plant_vel[w] / CONTROL_LOOP_HZ;
    }
    plant_acc =
        (0.5f * (plant_vel[0] + plant_vel[1]) - v_before) * CONTROL_LOOP_HZ;
}

int32_t hal_encoder_left() {
    return (int32_t)std::llround(plant_mm[0] / ENC_MM_PER_COUNT) * ENC_LEFT_SIGN;
}
int32_t hal_encoder_right() {
    return (int32_t)std::llround(plant_mm[1] / ENC_MM_PER_COUNT) * ENC_RIGHT_SIGN;
}
void hal_encoders_init() {}
void hal_encoders_zero() { plant_mm[0] = plant_mm[1] = 0; }

// Unused IMU register HAL (imu_read_fifo below bypasses the real driver).
uint8_t hal_imu_read_reg(uint8_t) { return 0; }
void hal_imu_read_regs(uint8_t, uint8_t *buf, size_t len) { memset(buf, 0, len); }
void hal_imu_write_reg(uint8_t, uint8_t) {}

// -- Mock IMU FIFO ---------------------------------------------------------------
// 13 packets per tick (nominal 6.4 kHz / 500 Hz). sim_body_acc is what the
// "IMU" reports as forward accel -- equal to plant_acc when gripping, held
// at 0 to simulate the body not following the wheels (wheelspin).
static float sim_body_acc = 0;  // mm/s^2
static constexpr int FIFO_PER_TICK = 13;

size_t imu_read_fifo(ImuSample *out, size_t max_samples) {
    size_t n = FIFO_PER_TICK < max_samples ? FIFO_PER_TICK : max_samples;
    int16_t ax = (int16_t)std::lround(sim_body_acc / GRAVITY_MM_S2 * 8192.0f);
    for (size_t i = 0; i < n; i++) {
        out[i] = ImuSample{};
        out[i].ax = ax;
        out[i].az = 8192;  // +1 g: level, stationary gravity reference
    }
    return n;
}
uint16_t imu_fifo_count() { return 0; }
bool imu_init() { return true; }

static void (*tick_isr)() = nullptr;
bool hal_control_timer_begin(void (*isr)(), float) {
    tick_isr = isr;
    return true;
}

static void expect(const char *what, bool cond) {
    if (!cond) {
        std::printf("FAIL: %s\n", what);
        assert(false);
    }
}

int main() {
    encoders_init();
    motors_init();
    assert(control_init());
    assert(tick_isr != nullptr);

    // Run the fusion settle + gyro bias calibration on stationary packets
    // (2 s settle + 1 s window at 6.4 kHz = 19200 samples / 13 per tick).
    for (int i = 0; i < 1600; i++) tick_isr();
    ControlHeading h = control_heading();
    expect("fusion ready after calibration", h.ready);

    control_motion_zero();
    control_velocity_enable(true);

    // -- Grip: gentle ramp, IMU agrees with wheels ------------------------------
    for (int i = 0; i < 400; i++) {  // 0 -> 400 mm/s at 500 mm/s^2
        float target = 500.0f * (i / (float)CONTROL_LOOP_HZ);
        if (target > 400.0f) target = 400.0f;
        control_set_wheel_velocity(target, target);
        plant_step();
        sim_body_acc = plant_acc;  // body follows wheels: grip
        tick_isr();
    }
    for (int i = 0; i < 500; i++) {  // cruise 1 s
        control_set_wheel_velocity(400.0f, 400.0f);
        plant_step();
        sim_body_acc = plant_acc;
        tick_isr();
    }
    ControlOdometry o = control_odometry();
    ControlMotion m = control_motion();
    expect("no slip during grip ramp", m.slip_events == 0);
    expect("v_est tracks v_enc at cruise",
           std::fabs(m.v_est_mm_s - o.vel_mm_s) < 30.0f);
    expect("v_est near 400", std::fabs(m.v_est_mm_s - 400.0f) < 40.0f);

    // -- Wheelspin: wheels jump, body (IMU) does not ----------------------------
    float v_est_before = m.v_est_mm_s;
    plant_vel[0] += 800.0f;  // wheelspin: encoders leap...
    plant_vel[1] += 800.0f;
    for (int i = 0; i < 25; i++) {  // 50 ms
        control_set_wheel_velocity(400.0f, 400.0f);
        plant_step();
        sim_body_acc = 0.0f;  // ...but the body doesn't accelerate
        tick_isr();
    }
    m = control_motion();
    expect("slip detected", m.slip && m.slip_events >= 1);
    expect("v_est ignored the encoder jump",
           std::fabs(m.v_est_mm_s - v_est_before) < 100.0f);

    // -- Recovery: wheels regain grip near the estimate -------------------------
    plant_vel[0] = m.v_est_mm_s;  // grip regained at body speed
    plant_vel[1] = m.v_est_mm_s;
    for (int i = 0; i < 200; i++) {  // plenty > MOTION_RECOVER_TICKS
        control_set_wheel_velocity(400.0f, 400.0f);
        plant_step();
        sim_body_acc = plant_acc;
        tick_isr();
    }
    m = control_motion();
    expect("slip cleared after recovery", !m.slip);
    o = control_odometry();
    expect("v_est re-tracks encoders",
           std::fabs(m.v_est_mm_s - o.vel_mm_s) < 30.0f);

    control_velocity_enable(false);
    std::printf("ALL MOTION TESTS PASSED\n");
    return 0;
}
