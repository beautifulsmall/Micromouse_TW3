// Host-side test: straight-line self-calibration -- replicates the 'c'
// command's update law (strim += avg_dv/avg_v over alternating +/-200 mm
// legs) against a plant with a 5% weak right wheel AND a residual gyro
// bias switched on after fusion calibration. Verifies:
//   - the loop converges (a fwd+rev pair inside 1 deg / 5 mm) within 16 legs
//   - strim lands positive (right wheel needs speeding) and sane
//   - the gyro-bias component does NOT poison the trim (it flips sign with
//     direction and averages out over pairs)
// Build (one line; provides its own imu_read_fifo):
//
//   g++ -std=c++17 -Istubs -I../lib/Control -I../lib/Drivers -I../lib/HAL
//     ../lib/Control/Control.cpp ../lib/Control/Fusion.cpp
//     ../lib/Control/Profile.cpp ../lib/Drivers/Encoders.cpp
//     ../lib/Drivers/Motors.cpp test_straight_cal.cpp
//     -o test_straight_cal && ./test_straight_cal

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

// -- Plant: asymmetric wheels + rigid-body yaw (as test_move.cpp) ---------------
static constexpr float TRACK_MM = 65.0f;
static constexpr float PLANT_LAG_ALPHA = 0.1f;
static float plant_gain[2] = {1.0f, 0.95f};  // right wheel 5% weak
static float plant_vel[2] = {0, 0};
static double plant_mm[2] = {0, 0};
static float plant_acc = 0;
static float plant_yaw_dps = 0;

static float wheel_duty(int w) {
    return w == 0 ? duty_ch[MOTOR_LEFT_CH] * MOTOR_LEFT_SIGN
                  : duty_ch[MOTOR_RIGHT_CH] * MOTOR_RIGHT_SIGN;
}

static void plant_step() {
    float v_before = 0.5f * (plant_vel[0] + plant_vel[1]);
    for (int w = 0; w < 2; w++) {
        float vmax = (w == 0 ? VCTRL_VMAX_LEFT_MM_S : VCTRL_VMAX_RIGHT_MM_S) *
                     plant_gain[w];
        float fric = w == 0 ? VCTRL_FRICTION_LEFT : VCTRL_FRICTION_RIGHT;
        float duty = standby ? 0.0f : wheel_duty(w);
        float mag = std::fabs(duty) - fric;
        float v_ss = mag > 0 ? vmax * mag * (duty > 0 ? 1 : -1) : 0;
        plant_vel[w] += PLANT_LAG_ALPHA * (v_ss - plant_vel[w]);
        plant_mm[w] += plant_vel[w] / CONTROL_LOOP_HZ;
    }
    plant_acc =
        (0.5f * (plant_vel[0] + plant_vel[1]) - v_before) * CONTROL_LOOP_HZ;
    plant_yaw_dps =
        (plant_vel[1] - plant_vel[0]) / TRACK_MM * 57.29577951308232f;
}

int32_t hal_encoder_left() {
    return (int32_t)std::llround(plant_mm[0] / ENC_MM_PER_COUNT) * ENC_LEFT_SIGN;
}
int32_t hal_encoder_right() {
    return (int32_t)std::llround(plant_mm[1] / ENC_MM_PER_COUNT) * ENC_RIGHT_SIGN;
}
void hal_encoders_init() {}
void hal_encoders_zero() { plant_mm[0] = plant_mm[1] = 0; }

uint8_t hal_imu_read_reg(uint8_t) { return 0; }
void hal_imu_read_regs(uint8_t, uint8_t *buf, size_t len) { memset(buf, 0, len); }
void hal_imu_write_reg(uint8_t, uint8_t) {}

// -- Mock IMU FIFO: accel + yaw rate + switchable residual gyro bias ------------
static constexpr int FIFO_PER_TICK = 13;
static float sim_gyro_bias_dps = 0.0f;  // enabled AFTER fusion calibration
size_t imu_read_fifo(ImuSample *out, size_t max_samples) {
    size_t n = FIFO_PER_TICK < max_samples ? FIFO_PER_TICK : max_samples;
    int16_t ax = (int16_t)std::lround(plant_acc / GRAVITY_MM_S2 * 8192.0f);
    int16_t gz =
        (int16_t)std::lround((plant_yaw_dps + sim_gyro_bias_dps) * 8.2f);
    for (size_t i = 0; i < n; i++) {
        out[i] = ImuSample{};
        out[i].ax = ax;
        out[i].az = 8192;
        out[i].gz = gz;
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

static int failures = 0;
static void expect(const char *what, bool cond) {
    if (!cond) {
        std::printf("FAIL: %s\n", what);
        failures++;
    }
}

// One calibration leg; returns false if the move never finished.
static bool run_leg(float dist, ControlMoveStatus *st_out) {
    if (!control_move_start(dist, 250.0f, 800.0f)) return false;
    for (int i = 0; i < 2500; i++) {
        plant_step();
        tick_isr();
        if (!control_move_status().active) {
            *st_out = control_move_status();
            return true;
        }
    }
    return false;
}

int main() {
    encoders_init();
    motors_init();
    assert(control_init());
    assert(tick_isr != nullptr);

    // Fusion settle + calibration on clean stationary packets, THEN switch
    // on a residual gyro bias the calibration never saw.
    for (int i = 0; i < 1600; i++) tick_isr();
    assert(control_heading().ready);
    sim_gyro_bias_dps = 0.3f;

    control_motion_zero();
    control_velocity_enable(true);

    // -- The 'c' loop: alternating legs, strim += avg_dv/avg_v ------------------
    bool converged = false;
    bool fwd_ok = false;
    int legs = 0;
    for (; legs < 16 && !converged; legs++) {
        float dir = (legs % 2 == 0) ? 1.0f : -1.0f;
        ControlMoveStatus st;
        expect("leg completes", run_leg(dir * 200.0f, &st));

        ControlMoveTrim tr = control_move_trim();
        ControlGains g = control_get_gains();
        expect("trim measurement valid", tr.valid);
        if (tr.valid && std::fabs(tr.avg_v_mm_s) > 50.0f) {
            g.strim += tr.avg_dv_mm_s / tr.avg_v_mm_s;
            control_set_gains(g);
        }

        float d_err = st.d_ref - st.d_mm;
        bool leg_ok = st.done && std::fabs(st.yaw_err_deg) < 1.0f &&
                      std::fabs(d_err) < 5.0f;
        std::printf("leg %2d %s: yaw %+.2f deg, pos %+.1f mm, strim %+.5f %s\n",
                    legs, dir > 0 ? "FWD" : "REV", st.yaw_err_deg, d_err,
                    g.strim, leg_ok ? "ok" : "");

        if (dir > 0) {
            fwd_ok = leg_ok;
        } else if (leg_ok && fwd_ok) {
            converged = true;
        }
    }
    control_velocity_enable(false);

    ControlGains g = control_get_gains();
    std::printf("converged=%d after %d legs, strim=%.5f\n", converged, legs,
                g.strim);
    expect("calibration converged within 16 legs", converged);
    expect("strim positive (weak right wheel needs speeding)", g.strim > 0.001f);
    expect("strim sane", g.strim < 0.05f);

    if (failures == 0) {
        std::printf("ALL STRAIGHT-CAL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d STRAIGHT-CAL TEST(S) FAILED\n", failures);
    return 1;
}
