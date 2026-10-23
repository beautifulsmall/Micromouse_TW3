// Host-side test: profiled move executor (Control.h) -- closed-loop sim of
// a straight move with an ASYMMETRIC plant (right wheel 5% weak) and a
// simulated gyro (yaw rate = wheel speed differential / track width). This
// is the failure mode seen on hardware 2026-07-03: a time-based open-loop
// profile undershot the distance and curved off heading. Verifies:
//   - position loop: fused distance reaches the 720 mm target (move done)
//   - heading hold: final yaw error small despite the weak wheel
//   - contrast: same plant with kyaw = 0 visibly veers
// Build (one line; provides its own imu_read_fifo -- no ImuIcm45686.cpp):
//
//   g++ -std=c++17 -Istubs -I../lib/Control -I../lib/Drivers -I../lib/HAL
//     ../lib/Control/Control.cpp ../lib/Control/Fusion.cpp
//     ../lib/Control/Profile.cpp ../lib/Drivers/Encoders.cpp
//     ../lib/Drivers/Motors.cpp test_move.cpp -o test_move && ./test_move

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

// -- Plant: asymmetric wheels + rigid-body yaw ----------------------------------
static constexpr float TRACK_MM = 65.0f;   // wheel separation
static constexpr float PLANT_LAG_ALPHA = 0.1f;
static float plant_gain[2] = {1.0f, 0.95f};  // right wheel 5% weak
static float plant_vel[2] = {0, 0};          // mm/s
static double plant_mm[2] = {0, 0};
static float plant_acc = 0;      // body forward accel, mm/s^2
static float plant_yaw_dps = 0;  // body yaw rate from wheel differential

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
    // +CCW yaw: right wheel faster than left turns left (CCW, positive).
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

// -- Mock IMU FIFO: forward accel + yaw rate from the plant ---------------------
static constexpr int FIFO_PER_TICK = 13;
size_t imu_read_fifo(ImuSample *out, size_t max_samples) {
    size_t n = FIFO_PER_TICK < max_samples ? FIFO_PER_TICK : max_samples;
    int16_t ax = (int16_t)std::lround(plant_acc / GRAVITY_MM_S2 * 8192.0f);
    int16_t gz = (int16_t)std::lround(plant_yaw_dps * 8.2f);
    for (size_t i = 0; i < n; i++) {
        out[i] = ImuSample{};
        out[i].ax = ax;
        out[i].az = 8192;  // +1 g
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

// Runs a full 720 mm move; returns ticks consumed (0 = never finished).
static int run_move() {
    if (!control_move_start(720.0f, 400.0f, 500.0f)) return 0;
    for (int i = 1; i <= 2500; i++) {  // profile is 1300 ticks + settle
        plant_step();
        tick_isr();
        if (!control_move_status().active) return i;
    }
    return 0;
}

int main() {
    encoders_init();
    motors_init();
    assert(control_init());
    assert(tick_isr != nullptr);

    // Fusion settle + gyro bias calibration on stationary packets.
    for (int i = 0; i < 1600; i++) tick_isr();
    assert(control_heading().ready);

    // Refused while the velocity controller is off.
    expect("move refused when disabled", !control_move_start(720, 400, 500));

    // -- Move with heading hold on the asymmetric plant --------------------------
    control_motion_zero();
    control_velocity_enable(true);
    int ticks = run_move();
    ControlMoveStatus st = control_move_status();
    ControlHeading h = control_heading();
    std::printf("move: %d ticks, done=%d, d=%.1f mm, yaw=%.2f deg\n",
                ticks, st.done, st.d_mm, h.yaw_deg);
    expect("move finished", ticks > 0);
    expect("move settled (done)", st.done);
    expect("distance reached", std::fabs(st.d_mm - 720.0f) < 5.0f);
    expect("heading held despite weak wheel", std::fabs(h.yaw_deg) < 2.0f);
    control_velocity_enable(false);

    // -- Contrast A: integral only (kyaw = 0) ----------------------------------
    // The heading-hold INTEGRAL (added 2026-07-16, YAW_INT_* in Control.cpp)
    // learns a trim coefficient and applies dv_hold = c * v, so it holds
    // heading on its own. Zeroing kyaw alone therefore does NOT veer -- which
    // is what this contrast case used to assert, and why it failed silently
    // from the day the integral landed until the V2 cut re-ran the suite.
    // Assert the real property instead: the integral alone still holds.
    ControlGains g = control_get_gains();
    float kyaw_saved = g.kyaw;
    g.kyaw = 0.0f;
    control_set_gains(g);

    control_motion_zero();
    control_set_heading_deg(0.0f);
    control_velocity_enable(true);
    ticks = run_move();
    h = control_heading();
    st = control_move_status();
    std::printf("kyaw=0 (integral on):  %d ticks, d=%.1f mm, yaw=%.2f deg\n",
                ticks, st.d_mm, h.yaw_deg);
    expect("kyaw=0 still reaches distance (position loop)",
           ticks > 0 && std::fabs(st.d_mm - 720.0f) < 8.0f);
    expect("kyaw=0 heading still held by the integral",
           std::fabs(h.yaw_deg) < 2.0f);
    control_velocity_enable(false);

    // -- Contrast B: no heading authority at all -> the weak wheel veers --------
    // The original sanity case, restored: with BOTH the proportional hold and
    // the integral off, nothing opposes the 5% weak right wheel and the move
    // curves off heading. If this ever stops veering, the plant model in this
    // test has gone symmetric and the two cases above prove nothing.
    control_yaw_integral_enable(false);
    control_motion_zero();
    control_set_heading_deg(0.0f);
    control_velocity_enable(true);
    ticks = run_move();
    h = control_heading();
    st = control_move_status();
    std::printf("no heading authority:  %d ticks, d=%.1f mm, yaw=%.2f deg\n",
                ticks, st.d_mm, h.yaw_deg);
    expect("no heading authority visibly veers (sanity)",
           std::fabs(h.yaw_deg) > 3.0f);
    control_velocity_enable(false);
    control_yaw_integral_enable(true);

    g.kyaw = kyaw_saved;
    control_set_gains(g);

    // -- THE FRONT GOVERNOR (Control.h, 2026-09-26) ------------------------------
    // A wall stands 750 mm from the start; the move asks for 720 (it would
    // park 30 mm from the wall, contact at 45). Both front beams report the
    // wall from 200 mm out. The governor must hold the mouse at >= 60 mm and
    // the move must end NOT done -- that is how a mission learns.
    {
        const float WALL_MM = 750.0f;
        auto beams = [&](float d, uint8_t *fl, uint8_t *fr, float skew) {
            float a = d, b = d + skew;
            *fl = (a >= 1.0f && a <= 200.0f) ? (uint8_t)std::lround(a) : 255;
            *fr = (b >= 1.0f && b <= 200.0f) ? (uint8_t)std::lround(b) : 255;
        };
        auto run_with_wall = [&](float skew, float *closest, uint32_t *cap_ticks) -> ControlMoveStatus {
            control_motion_zero();
            control_set_heading_deg(0.0f);
            control_velocity_enable(true);
            *closest = 1e9f;
            double p0 = 0.5 * (plant_mm[0] + plant_mm[1]);   // the plant is not re-zeroed between runs
            bool started = control_move_start(720.0f, 400.0f, 500.0f);
            expect("governor test: move started", started);
            for (int i = 1; i <= 4000 && started; i++) {
                plant_step();
                float d_now = WALL_MM - (float)(0.5 * (plant_mm[0] + plant_mm[1]) - p0);
                if (d_now < *closest) *closest = d_now;
                uint8_t fl, fr;
                beams(d_now, &fl, &fr, skew);
                control_front_guard_feed(fl, fr, 0);
                tick_isr();
                if (!control_move_status().active) break;
            }
            *cap_ticks = control_front_guard().cap_ticks;
            control_front_guard_feed(255, 255, 0);
            control_velocity_enable(false);
            return control_move_status();
        };
        float closest; uint32_t capt;
        ControlMoveStatus gs = run_with_wall(2.0f, &closest, &capt);
        std::printf("governor, wall at 750, move 720: closest %.1f mm, d=%.1f, done=%d, capped %lu ticks\n",
                    closest, gs.d_mm, gs.done, (unsigned long)capt);
        expect("governor held the mouse outside the 60 mm floor", closest >= FRONT_GOV_FLOOR_MM - 3.0f);
        expect("governor engaged (capped ticks)", capt > 50);
        expect("move against the governor ends NOT done", !gs.done);
        expect("move against the governor still got close (>= 640)", gs.d_mm > 640.0f);

        // Disagreeing beams (one clips a post 60 mm nearer): no wall, no cap.
        gs = run_with_wall(60.0f, &closest, &capt);
        std::printf("governor, beams disagree by 60: closest %.1f mm, d=%.1f, done=%d, capped %lu ticks\n",
                    closest, gs.d_mm, gs.done, (unsigned long)capt);
        expect("disagreeing beams do not cap", capt == 0);
        expect("move completes when the beams disagree", gs.done && std::fabs(gs.d_mm - 720.0f) < 5.0f);

        // Stale packet: no cap.
        control_motion_zero(); control_set_heading_deg(0.0f);
        control_front_guard_feed(70, 70, 500);
        expect("stale packet is not a wall", !control_front_guard().wall);
        control_front_guard_feed(70, 70, 10);
        expect("fresh agreeing pair is a wall at the nearer beam",
               control_front_guard().wall && std::fabs(control_front_guard().d_mm - 70.0f) < 0.5f);
        control_front_guard_feed(255, 255, 0);
    }

    if (failures == 0) {
        std::printf("ALL MOVE TESTS PASSED\n");
        return 0;
    }
    std::printf("%d MOVE TEST(S) FAILED\n", failures);
    return 1;
}
