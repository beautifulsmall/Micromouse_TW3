// Host-side test: mock HAL, drive the control tick by hand, verify the
// encoder driver sign normalisation and the Control-layer odometry
// (mm conversion, centre distance, velocity filter). Build (one line):
//
//   g++ -std=c++17 -Istubs -I../lib/Control -I../lib/Drivers -I../lib/HAL
//     ../lib/Control/Control.cpp ../lib/Control/Fusion.cpp
//     ../lib/Drivers/Encoders.cpp ../lib/Drivers/ImuIcm45686.cpp
//     ../lib/Drivers/Motors.cpp test_odometry.cpp
//     -o test_odometry && ./test_odometry

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

// -- Mock HAL ------------------------------------------------------------------
// Raw hardware counts (pre sign normalisation).
static int32_t fake_left = 0;
static int32_t fake_right = 0;

void hal_encoders_init() {}
int32_t hal_encoder_left() { return fake_left; }
int32_t hal_encoder_right() { return fake_right; }
void hal_encoders_zero() { fake_left = fake_right = 0; }

// Motors: Control links against the velocity controller, which calls into
// the Motors driver; the controller stays disabled here, so no-ops suffice.
void hal_motors_init() {}
void hal_motor_standby(bool) {}
void hal_motor_set(uint8_t, float) {}

// IMU: empty FIFO so the tick's IMU path is a no-op.
uint8_t hal_imu_read_reg(uint8_t) { return 0; }
void hal_imu_read_regs(uint8_t, uint8_t *buf, size_t len) { memset(buf, 0, len); }
void hal_imu_write_reg(uint8_t, uint8_t) {}

// Capture the control ISR so the test can invoke ticks directly.
static void (*tick_isr)() = nullptr;
bool hal_control_timer_begin(void (*isr)(), float) {
    tick_isr = isr;
    return true;
}

static void expect_near(const char *what, float got, float want, float tol) {
    if (std::fabs(got - want) > tol) {
        std::printf("FAIL: %s = %.3f, expected %.3f +/- %.3f\n",
                    what, got, want, tol);
        assert(false);
    }
}

int main() {
    encoders_init();
    assert(control_init());
    assert(tick_isr != nullptr);

    // Sign normalisation: forward motion = raw +left / -right (per
    // ENC_*_SIGN in Encoders.h) must read positive on both sides.
    fake_left = 1000 * ENC_LEFT_SIGN;
    fake_right = 1000 * ENC_RIGHT_SIGN;
    tick_isr();
    ControlOdometry o = control_odometry();
    expect_near("left_mm", o.left_mm, 1000 * ENC_MM_PER_COUNT, 0.01f);
    expect_near("right_mm", o.right_mm, 1000 * ENC_MM_PER_COUNT, 0.01f);
    expect_near("dist_mm", o.dist_mm, 1000 * ENC_MM_PER_COUNT, 0.01f);

    // Reverse: negative on both sides.
    fake_left = -500 * ENC_LEFT_SIGN;
    fake_right = -500 * ENC_RIGHT_SIGN;
    tick_isr();
    o = control_odometry();
    expect_near("reverse left_mm", o.left_mm, -500 * ENC_MM_PER_COUNT, 0.01f);
    expect_near("reverse right_mm", o.right_mm, -500 * ENC_MM_PER_COUNT, 0.01f);

    // Velocity: +10 counts per tick on both wheels = 10 * mm/count * 500 Hz
    // (~1029 mm/s). EMA (alpha 0.2, tau ~5 ticks) must converge by 100 ticks.
    int32_t l = 0, r = 0;
    fake_left = fake_right = 0;
    tick_isr();  // settle deltas at the zeroed baseline
    tick_isr();
    for (int i = 0; i < 100; i++) {
        l += 10;
        r += 10;
        fake_left = l * ENC_LEFT_SIGN;
        fake_right = r * ENC_RIGHT_SIGN;
        tick_isr();
    }
    o = control_odometry();
    const float v = 10.0f * ENC_MM_PER_COUNT * CONTROL_LOOP_HZ;
    expect_near("vel_left_mm_s", o.vel_left_mm_s, v, v * 0.01f);
    expect_near("vel_right_mm_s", o.vel_right_mm_s, v, v * 0.01f);
    expect_near("vel_mm_s", o.vel_mm_s, v, v * 0.01f);

    // Stop: velocity decays back toward zero.
    for (int i = 0; i < 100; i++) tick_isr();
    o = control_odometry();
    expect_near("vel after stop", o.vel_mm_s, 0.0f, 1.0f);

    std::printf("ALL ODOMETRY TESTS PASSED\n");
    return 0;
}
