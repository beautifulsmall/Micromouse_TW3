// Host-side test: mock HAL + simple motor/encoder plant model, drive the
// control tick by hand, verify the wheel velocity controller (feedforward
// + PD, duty clamp, enable/disable semantics). Build (one line):
//
//   g++ -std=c++17 -Istubs -I../lib/Control -I../lib/Drivers -I../lib/HAL
//     ../lib/Control/Control.cpp ../lib/Control/Fusion.cpp
//     ../lib/Drivers/Encoders.cpp ../lib/Drivers/ImuIcm45686.cpp
//     ../lib/Drivers/Motors.cpp test_velocity_control.cpp
//     -o test_velocity_control && ./test_velocity_control

#include "Control.h"
#include "Motors.h"

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
static float duty_ch[2] = {0, 0};  // last commanded duty per TB6612 channel
static bool standby = true;

void hal_motors_init() {}
void hal_motor_standby(bool s) { standby = s; }
void hal_motor_set(uint8_t channel, float duty) { duty_ch[channel] = duty; }

// -- Mock HAL: encoders + plant model -------------------------------------------
// Wheel plant matching the controller's model exactly (v_ss = VMAX * (duty
// - friction)), with a first-order lag so the PD has a transient to work
// on. Positions integrate into raw hardware counts (inverse-normalised so
// the driver's ENC_*_SIGN maps them back to forward-positive).
static float plant_vel[2] = {0, 0};  // mm/s, [left, right]
static double plant_mm[2] = {0, 0};
static constexpr float PLANT_LAG_ALPHA = 0.1f;  // ~20 ms time constant

static float wheel_duty(int wheel) {  // forward-positive duty per wheel
    return wheel == 0 ? duty_ch[MOTOR_LEFT_CH] * MOTOR_LEFT_SIGN
                      : duty_ch[MOTOR_RIGHT_CH] * MOTOR_RIGHT_SIGN;
}

static void plant_step() {
    for (int w = 0; w < 2; w++) {
        float vmax = w == 0 ? VCTRL_VMAX_LEFT_MM_S : VCTRL_VMAX_RIGHT_MM_S;
        float friction = w == 0 ? VCTRL_FRICTION_LEFT : VCTRL_FRICTION_RIGHT;
        float duty = standby ? 0.0f : wheel_duty(w);
        float mag = std::fabs(duty) - friction;
        float v_ss = mag > 0 ? vmax * mag * (duty > 0 ? 1 : -1) : 0;
        plant_vel[w] += PLANT_LAG_ALPHA * (v_ss - plant_vel[w]);
        plant_mm[w] += plant_vel[w] / CONTROL_LOOP_HZ;
    }
}

int32_t hal_encoder_left() {
    return (int32_t)std::llround(plant_mm[0] / ENC_MM_PER_COUNT) * ENC_LEFT_SIGN;
}
int32_t hal_encoder_right() {
    return (int32_t)std::llround(plant_mm[1] / ENC_MM_PER_COUNT) * ENC_RIGHT_SIGN;
}
void hal_encoders_init() {}
void hal_encoders_zero() { plant_mm[0] = plant_mm[1] = 0; }

// -- Mock HAL: IMU (empty FIFO, tick's IMU path is a no-op) ---------------------
uint8_t hal_imu_read_reg(uint8_t) { return 0; }
void hal_imu_read_regs(uint8_t, uint8_t *buf, size_t len) { memset(buf, 0, len); }
void hal_imu_write_reg(uint8_t, uint8_t) {}

// Capture the control ISR so the test can invoke ticks directly.
static void (*tick_isr)() = nullptr;
bool hal_control_timer_begin(void (*isr)(), float) {
    tick_isr = isr;
    return true;
}

static void run_ticks(int n) {
    for (int i = 0; i < n; i++) {
        plant_step();
        tick_isr();
    }
}

static void expect_near(const char *what, float got, float want, float tol) {
    if (std::fabs(got - want) > tol) {
        std::printf("FAIL: %s = %.3f, expected %.3f +/- %.3f\n",
                    what, got, want, tol);
        assert(false);
    }
}

// THE DUTY CEILING DOES NOT MOVE (bert, 2026-08-27: "there should be no J
// budget or limits that change related to energy budget").
//
// This test used to assert the opposite. It pinned a burst budget: PEAK 0.85
// reachable for ~1.9 s, folding to CONT 0.45, recovering over 30 s. That
// budget is gone, and the test inverts with it -- which is the point of
// keeping it rather than deleting it. The property worth defending is no
// longer "the fold happens and is a useful length"; it is that NOTHING
// happens, however long the demand is held.
//
// A moving ceiling was not just an unwanted limit, it was an unfalsifiable
// explanation. Every wide corner and soft launch had "maybe it folded" as a
// live hypothesis that cost real time to rule out. A constant is checkable.
static void test_duty_ceiling_never_moves() {
    control_set_gains(ControlGains{});
    control_velocity_enable(true);
    control_duty_peak_reset();

    // Ask for far more than the motors can give: duty pins at the ceiling.
    control_set_wheel_velocity(9000.0f, 9000.0f);
    run_ticks(5);
    ControlVelStatus v = control_velocity_status();
    expect_near("a hard demand reaches the limit", v.duty_left,
                VCTRL_DUTY_LIMIT, 1e-3f);

    // Hold it for a minute of control time -- twice the old TAU, six times
    // the whole 10-minute-battery mouse spends in any one sprint. Nothing may
    // change.
    run_ticks((int)(60.0f * CONTROL_LOOP_HZ));
    v = control_velocity_status();
    expect_near("still at the limit after 60 s", v.duty_left,
                VCTRL_DUTY_LIMIT, 1e-3f);
    expect_near("the peak recorded it", v.duty_peak, VCTRL_DUTY_LIMIT, 1e-3f);

    // And the peak is a measurement, not a latch that survives the next run.
    // Let it come to rest first: zero target while rolling is a hard BRAKE
    // (see test_zero_target_while_rolling_brakes), and a brake is a duty like
    // any other -- resetting mid-brake would immediately re-arm the peak.
    control_set_wheel_velocity(0.0f, 0.0f);
    run_ticks(500);
    control_duty_peak_reset();
    run_ticks(5);
    v = control_velocity_status();
    if (!(v.duty_peak < 0.2f)) {
        std::printf("FAIL: duty peak %.3f did not reset\n", (double)v.duty_peak);
        assert(false);
    }
    std::printf("  duty: fixed ceiling %.2f, held 60 s, no fold\n",
                (double)VCTRL_DUTY_LIMIT);
    control_velocity_enable(false);
}

// BRAKING IS REVERSE DUTY, and it falls out of the loop already there.
//
// Run 151 coasted into a wall because every abort ended in motors_stop(), and
// zero duty is COAST. The fix needs no new actuator model: with the target at
// zero and the wheel still rolling forward, err = 0 - v is large and negative,
// so kp*err drives the duty NEGATIVE and the bridge brakes. The whole fix
// rests on that, so it is pinned here against the real plant.
static void test_zero_target_while_rolling_brakes() {
    control_set_gains(ControlGains{});
    control_velocity_enable(true);
    // Wind the plant up to the speed run 151 tripped its guard at.
    control_set_wheel_velocity(1143.0f, 1143.0f);
    run_ticks(600);
    float v_before = 0.5f * (control_odometry().vel_left_mm_s +
                             control_odometry().vel_right_mm_s);
    if (v_before < 900.0f) {
        std::printf("FAIL: plant never reached speed (%.0f)\n", v_before);
        assert(false);
    }
    // Ask for zero while still rolling: the duty must go NEGATIVE.
    control_set_wheel_velocity(0.0f, 0.0f);
    plant_step();
    tick_isr();
    float dl = wheel_duty(0), dr = wheel_duty(1);
    if (!(dl < -0.05f && dr < -0.05f)) {
        std::printf("FAIL: braking duty not negative (%.3f, %.3f)\n", dl, dr);
        assert(false);
    }
    if (dl < -VCTRL_DUTY_LIMIT - 1e-6f || dr < -VCTRL_DUTY_LIMIT - 1e-6f) {
        std::printf("FAIL: braking exceeded the duty limit\n");
        assert(false);
    }
    // And it must actually stop the thing, not merely point the right way.
    run_ticks(400);
    float v_after = 0.5f * (control_odometry().vel_left_mm_s +
                            control_odometry().vel_right_mm_s);
    expect_near("braked to rest", v_after, 0.0f, 30.0f);
    std::printf("  brake: %.0f mm/s -> %.0f mm/s, first duty %.3f\n",
                v_before, v_after, dl);

    // control_brake_to_stop() must return promptly once stopped, and must be
    // safe to call when the loop is already disabled (every abort path does).
    control_velocity_enable(false);
    float v = control_brake_to_stop();
    expect_near("brake with the loop disabled is a no-op", v, 0.0f, 0.001f);
}

int main() {
    encoders_init();
    motors_init();
    assert(control_init());
    assert(tick_isr != nullptr);

    // Disabled: ticks must not drive the motors or leave standby.
    run_ticks(50);
    assert(standby);
    expect_near("disabled duty L", duty_ch[MOTOR_LEFT_CH], 0.0f, 1e-6f);

    // Enable + step to 500 mm/s: must converge and hold within 2%.
    control_velocity_enable(true);
    assert(!standby);
    control_set_wheel_velocity(500.0f, 500.0f);
    run_ticks(500);  // 1 s
    ControlOdometry o = control_odometry();
    expect_near("vel L @500", o.vel_left_mm_s, 500.0f, 10.0f);
    expect_near("vel R @500", o.vel_right_mm_s, 500.0f, 10.0f);
    ControlVelStatus v = control_velocity_status();
    // Duty should sit near the feedforward value (500/VMAX + friction).
    float duty_ff = 500.0f / VCTRL_VMAX_LEFT_MM_S + VCTRL_FRICTION_LEFT;
    expect_near("duty L @500", v.duty_left, duty_ff, 0.05f);

    // Asymmetric targets (a turn): each wheel tracks its own setpoint.
    control_set_wheel_velocity(300.0f, 600.0f);
    run_ticks(500);
    o = control_odometry();
    expect_near("vel L asym", o.vel_left_mm_s, 300.0f, 10.0f);
    expect_near("vel R asym", o.vel_right_mm_s, 600.0f, 12.0f);

    // Reverse: negative setpoint tracks negative velocity.
    control_set_wheel_velocity(-400.0f, -400.0f);
    run_ticks(500);
    o = control_odometry();
    expect_near("vel L reverse", o.vel_left_mm_s, -400.0f, 10.0f);

    // Unreachable setpoint: duty must clamp at VCTRL_DUTY_LIMIT and the
    // plant saturates at VMAX * (limit - friction).
    control_set_wheel_velocity(100000.0f, 100000.0f);
    run_ticks(500);
    v = control_velocity_status();
    expect_near("duty clamp", v.duty_left, VCTRL_DUTY_LIMIT, 1e-4f);
    o = control_odometry();
    float v_sat = VCTRL_VMAX_LEFT_MM_S * (VCTRL_DUTY_LIMIT - VCTRL_FRICTION_LEFT);
    expect_near("vel at clamp", o.vel_left_mm_s, v_sat, v_sat * 0.02f);

    // Disable: coast + standby, duties reported zero.
    control_velocity_enable(false);
    assert(standby);
    v = control_velocity_status();
    assert(!v.enabled);
    expect_near("duty after disable L", duty_ch[MOTOR_LEFT_CH], 0.0f, 1e-6f);
    expect_near("duty after disable R", duty_ch[MOTOR_RIGHT_CH], 0.0f, 1e-6f);
    run_ticks(200);  // plant coasts down; controller must stay quiet
    v = control_velocity_status();
    expect_near("status duty after disable", v.duty_left, 0.0f, 1e-6f);

    test_duty_ceiling_never_moves();
    test_zero_target_while_rolling_brakes();

    // ---- The slip model's per-tick step (Control.h) ------------------------
    // sacc = 50 mm per m/s gained: +1000 mm/s^2 for one 2 ms tick is +2 mm/s
    // = +0.002 m/s -> the wheels are 0.1 mm ahead of the world, so the
    // correction is -0.1. Braking uses sbrk with the opposite sign; zero
    // accel is zero; coefficients at zero make the model inert.
    expect_near("slip step, accelerating", path_slip_step(1000.0f, 0.002f, 50.0f, 20.0f),
                -0.1f, 1e-6f);
    expect_near("slip step, braking", path_slip_step(-1000.0f, 0.002f, 50.0f, 20.0f),
                +0.04f, 1e-6f);
    expect_near("slip step, cruise", path_slip_step(0.0f, 0.002f, 50.0f, 20.0f),
                0.0f, 1e-9f);
    expect_near("slip step, inert", path_slip_step(1000.0f, 0.002f, 0.0f, 0.0f),
                0.0f, 1e-9f);
    // Over a whole launch: 0 -> 1000 mm/s at any profile sums to sacc x 1.0.
    {
        float acc = 0.0f, v = 0.0f;
        for (int i = 0; i < 500; i++) {   // 1 s at 500 Hz, a = 1000
            acc += path_slip_step(1000.0f, 0.002f, 50.0f, 20.0f);
            v += 2.0f;
        }
        expect_near("a full ramp integrates to sacc x dv", acc, -50.0f, 1e-3f);
        (void)v;
    }

    // ---- The vector reflex's arc law (Control.h reflex_arc_bias_deg) ------
    // Numbers from the 2026-09-05 D7 CW logs. Lap 51's inside beam closed
    // at ~0.55 mm per mm of travel at ~690 mm/s (380 mm/s): 59 mm is 155 ms
    // from contact -> fires under ftau 200 (ramp), 40 mm is 105 ms -> nearly
    // the full step. Lap 50's beam let go at 64 (no rate) -> nothing. The
    // (5,5) 90 reads its inside beam at 72 closing slowly -> nothing.
    expect_near("reflex: off when ftau = 0",
                reflex_arc_bias_deg(40.0f, 380.0f, 0.0f), 0.0f, 1e-9f);
    expect_near("reflex: not closing",
                reflex_arc_bias_deg(40.0f, 0.0f, 200.0f), 0.0f, 1e-9f);
    expect_near("reflex: receding",
                reflex_arc_bias_deg(40.0f, -100.0f, 200.0f), 0.0f, 1e-9f);
    expect_near("reflex: outside the range limit",
                reflex_arc_bias_deg(72.0f, 380.0f, 200.0f), 0.0f, 1e-9f);
    expect_near("reflex: 48 mm closing slowly (a normal 90's apex)",
                reflex_arc_bias_deg(48.0f, 150.0f, 200.0f), 0.0f, 1e-9f);
    // 59 mm is outside REFLEX_ARC_RANGE_MM (50) since 2026-09-06: a V at
    // R60 reads its inside post at ~55 near the apex on a clean pass.
    expect_near("reflex: 59 mm is outside the range limit now",
                reflex_arc_bias_deg(59.0f, 380.0f, 200.0f), 0.0f, 1e-9f);
    // 48 / 380 = 126 ms -> k = 2 (1 - 0.632) = 0.737 -> 5.9 deg
    expect_near("reflex: 48 mm at 380 mm/s ramps in",
                reflex_arc_bias_deg(48.0f, 380.0f, 200.0f), 5.89f, 0.05f);
    // 40 / 380 = 105 ms -> k = 0.947 -> 7.6 deg
    expect_near("reflex: 40 mm at 380 mm/s is nearly the full step",
                reflex_arc_bias_deg(40.0f, 380.0f, 200.0f), 7.58f, 0.05f);
    expect_near("reflex: saturates at FLINCH_MAX_DEG",
                reflex_arc_bias_deg(30.0f, 1000.0f, 200.0f), FLINCH_MAX_DEG, 1e-6f);
    // tau exactly at ftau is the boundary: nothing.
    expect_near("reflex: tau == ftau is off",
                reflex_arc_bias_deg(38.0f, 190.0f, 200.0f), 0.0f, 1e-9f);

    std::printf("ALL VELOCITY CONTROL TESTS PASSED\n");
    return 0;
}
