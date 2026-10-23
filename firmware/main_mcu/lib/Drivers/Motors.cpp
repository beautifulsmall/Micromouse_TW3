#include "Motors.h"

#include "HAL.h"

void motors_init() {
    hal_motors_init();
}

void motors_enable(bool enable) {
    hal_motor_standby(!enable);
}

void motor_set_left(float duty) {
    hal_motor_set(MOTOR_LEFT_CH, MOTOR_LEFT_SIGN * duty);
}

void motor_set_right(float duty) {
    hal_motor_set(MOTOR_RIGHT_CH, MOTOR_RIGHT_SIGN * duty);
}

void motors_stop() {
    motor_set_left(0.0f);
    motor_set_right(0.0f);
}
