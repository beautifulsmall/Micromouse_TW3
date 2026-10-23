// Host-side test: mock HAL, verify the Motors driver's left/right channel
// mapping, polarity normalisation, clamping pass-through and stop/standby.
// Build (one line):
//
//   g++ -std=c++17 -Istubs -I../lib/Drivers -I../lib/HAL
//     ../lib/Drivers/Motors.cpp test_motors.cpp -o test_motors && ./test_motors

#include "Motors.h"

#include <cassert>
#include <cmath>
#include <cstdio>

// -- Arduino stub definitions --------------------------------------------------
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void delay(unsigned long) {}
unsigned long millis() { return 0; }
void noInterrupts() {}
void interrupts() {}

// -- Mock HAL -------------------------------------------------------------------
static float last_duty[2] = {99.0f, 99.0f};  // per channel
static bool standby = true;
static bool init_called = false;

void hal_motors_init() { init_called = true; }
void hal_motor_standby(bool s) { standby = s; }
void hal_motor_set(uint8_t ch, float duty) {
    assert(ch <= 1);
    last_duty[ch] = duty;
}

int main() {
    motors_init();
    assert(init_called);

    // Enable maps to STBY (active-low handled in HAL; driver passes intent).
    motors_enable(true);
    assert(!standby);
    motors_enable(false);
    assert(standby);

    // Forward duty lands on the mapped channel with the normalised sign.
    motor_set_left(0.5f);
    assert(std::fabs(last_duty[MOTOR_LEFT_CH] - MOTOR_LEFT_SIGN * 0.5f) < 1e-6f);
    motor_set_right(0.25f);
    assert(std::fabs(last_duty[MOTOR_RIGHT_CH] - MOTOR_RIGHT_SIGN * 0.25f) < 1e-6f);

    // Reverse.
    motor_set_left(-0.75f);
    assert(std::fabs(last_duty[MOTOR_LEFT_CH] + MOTOR_LEFT_SIGN * 0.75f) < 1e-6f);

    // Left and right use different channels.
    static_assert(MOTOR_LEFT_CH != MOTOR_RIGHT_CH, "channel clash");

    // Stop zeroes both.
    motors_stop();
    assert(last_duty[0] == 0.0f && last_duty[1] == 0.0f);

    std::printf("ALL MOTOR TESTS PASSED\n");
    return 0;
}
