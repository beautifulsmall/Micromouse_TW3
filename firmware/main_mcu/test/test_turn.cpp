// Host-side test: on-the-spot turn via the maneuver executor (control_turn_start
// in lib/Control) plus the effective-track auto-fit (TrackTuner, Profile.h).
//
// The mock plant models each wheel as a first-order lag AND integrates body yaw
// kinematically from the wheel-speed differential (omega = (v_r - v_l)/track),
// then feeds that rate back as the gyro's gz so the real Madgwick fusion tracks
// heading. This closes the same loop the robot runs: executor -> wheels ->
// body rotation -> gyro -> fusion yaw -> executor. A sign error anywhere makes
// it positive feedback and the turn diverges, so the test is meaningful.
//
// Build (one line; links Profile.cpp since the executor uses TrapProfile):
//
//   g++ -std=c++17 -include cmath -Istubs -I../lib/Control -I../lib/Drivers -I../lib/HAL ../lib/Control/Control.cpp ../lib/Control/Fusion.cpp ../lib/Control/Profile.cpp ../lib/Drivers/Encoders.cpp ../lib/Drivers/Motors.cpp test_turn.cpp -o test_turn && ./test_turn

#include "Control.h"
#include "Profile.h"

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

// -- Mock plant: per-wheel first-order lag + kinematic yaw ----------------------
static constexpr float PLANT_TRACK_MM = 59.0f;  // the plant's true track
static constexpr float RAD2DEG = 57.29577951308232f;
static float plant_vel[2] = {0, 0};  // mm/s, [0]=left [1]=right
static double plant_mm[2] = {0, 0};
static double plant_yaw_deg = 0.0;   // integrated from the wheel differential
static float sim_yaw_rate_dps = 0.0f;
static constexpr float PLANT_LAG_ALPHA = 0.1f;

static float wheel_duty(int w) {
    return w == 0 ? duty_ch[MOTOR_LEFT_CH] * MOTOR_LEFT_SIGN
                  : duty_ch[MOTOR_RIGHT_CH] * MOTOR_RIGHT_SIGN;
}

static void plant_step() {
    for (int w = 0; w < 2; w++) {
        float vmax = w == 0 ? VCTRL_VMAX_LEFT_MM_S : VCTRL_VMAX_RIGHT_MM_S;
        float fric = w == 0 ? VCTRL_FRICTION_LEFT : VCTRL_FRICTION_RIGHT;
        float duty = standby ? 0.0f : wheel_duty(w);
        float mag = std::fabs(duty) - fric;
        float v_ss = mag > 0 ? vmax * mag * (duty > 0 ? 1 : -1) : 0;
        plant_vel[w] += PLANT_LAG_ALPHA * (v_ss - plant_vel[w]);
        plant_mm[w] += plant_vel[w] / CONTROL_LOOP_HZ;
    }
    // Kinematic body yaw: right wheel faster => CCW (+yaw).
    float omega_rad = (plant_vel[1] - plant_vel[0]) / PLANT_TRACK_MM;
    sim_yaw_rate_dps = omega_rad * RAD2DEG;
    plant_yaw_deg += sim_yaw_rate_dps / CONTROL_LOOP_HZ;
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

// -- Mock IMU FIFO: level attitude + gz from the plant yaw rate -----------------
static constexpr int FIFO_PER_TICK = 13;
size_t imu_read_fifo(ImuSample *out, size_t max_samples) {
    size_t n = FIFO_PER_TICK < max_samples ? FIFO_PER_TICK : max_samples;
    int16_t gz = (int16_t)std::lround(sim_yaw_rate_dps * 8.2f);  // 8.2 LSB/dps
    for (size_t i = 0; i < n; i++) {
        out[i] = ImuSample{};
        out[i].az = 8192;  // +1 g: level
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
    if (!cond) { std::printf("FAIL: %s\n", what); failures++; }
}

// Run the executor + plant until the maneuver reports inactive, or a cap.
static ControlMoveStatus run_until_done(int max_ticks) {
    ControlMoveStatus st{};
    for (int i = 0; i < max_ticks; i++) {
        plant_step();
        tick_isr();
        st = control_move_status();
        if (!st.active) break;
    }
    return st;
}

int main() {
    encoders_init();
    motors_init();
    assert(control_init());
    assert(tick_isr != nullptr);

    // Gyro-bias calibration window: stay stationary (gz = 0).
    for (int i = 0; i < 1600; i++) tick_isr();
    expect("fusion ready after calibration", control_heading().ready);

    control_motion_zero();
    control_velocity_enable(true);

    // The mock plant is a crude first-order lag with no inertia, so the
    // hardware-tuned inner-loop gains (high kp/kd) and the accel feedforward
    // overdrive it into numeric ringing on a velocity step -- a fidelity limit
    // of the mock, not the executor. Use gentle inner-loop gains here so the
    // plant is stable and the test exercises the TURN executor logic (profile
    // tracking, heading closure, direction, settle) rather than plant numerics.
    {
        ControlGains g = control_get_gains();
        g.kp = 0.0004f;
        g.kp_turn = 0.0004f;  // pure turns select kp_turn (split gain)
        g.kd = 0.0f;
        g.ka = 0.0f;
        control_set_gains(g);
    }

    // -- +90 deg in-place turn --------------------------------------------------
    double yaw0 = plant_yaw_deg;
    float fyaw0 = control_heading().yaw_deg;
    assert(control_turn_start(90.0f, 180.0f, 1200.0f));
    ControlMoveStatus st = run_until_done(2000);  // 4 s cap at 500 Hz

    expect("turn reports done (settled)", st.done);
    expect("fusion yaw reached ~+90",
           std::fabs((control_heading().yaw_deg - fyaw0) - 90.0f) < 3.0f);
    expect("plant physically rotated ~+90",
           std::fabs((plant_yaw_deg - yaw0) - 90.0) < 5.0);
    expect("final heading error small", std::fabs(st.yaw_err_deg) < 2.0f);
    // In-place: net forward travel stays ~0 (wheels counter-rotate).
    ControlOdometry o = control_odometry();
    expect("no net translation during turn", std::fabs(o.dist_mm) < 15.0f);

    // -- -90 deg turn: direction reverses --------------------------------------
    fyaw0 = control_heading().yaw_deg;
    assert(control_turn_start(-90.0f, 180.0f, 1200.0f));
    st = run_until_done(2000);
    expect("reverse turn done", st.done);
    expect("fusion yaw reached ~-90",
           std::fabs((control_heading().yaw_deg - fyaw0) + 90.0f) < 3.0f);

    // -- TrackTuner recovers the plant's 59 mm track from a turn ----------------
    {
        TrackTuner tt;
        tt.reset();
        const float DEG2RAD_L = 0.017453292519943295f;
        const float FIT_MIN_DPS = 0.3f * 180.0f;
        assert(control_turn_start(180.0f, 180.0f, 1200.0f));
        for (int i = 0; i < 2000; i++) {
            plant_step();
            tick_isr();
            ControlMoveStatus s = control_move_status();
            ControlOdometry od = control_odometry();
            float wheel_diff = 0.5f * (od.vel_right_mm_s - od.vel_left_mm_s);
            if (std::fabs(s.omega_ref_dps) > FIT_MIN_DPS) {
                tt.add(s.yaw_rate_dps * DEG2RAD_L, wheel_diff);
            }
            if (!s.active) break;
        }
        expect("TrackTuner valid", tt.valid());
        expect("TrackTuner recovers ~59 mm track",
               std::fabs(tt.suggest_track_mm() - PLANT_TRACK_MM) < 3.0f);
    }

    control_velocity_enable(false);
    if (failures == 0) {
        std::printf("ALL TURN TESTS PASSED\n");
        return 0;
    }
    std::printf("%d TURN TEST(S) FAILED\n", failures);
    return 1;
}
