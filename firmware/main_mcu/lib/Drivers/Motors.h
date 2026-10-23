// Motor driver (TB6612FNG dual H-bridge, left/right wheel mapping).
//
// The HAL provides raw per-channel PWM + direction (hal_motor_set); this
// layer maps TB6612 channels to left/right wheels and normalises polarity
// so that positive duty = forward on both sides, mirroring Encoders.h.
//
// Bench verification (run the 'm' self-test over serial, see main.cpp):
// it pulses each wheel and checks the encoders. If a wheel runs backwards,
// flip that side's MOTOR_*_SIGN; if the wrong wheel moves, swap
// MOTOR_LEFT_CH / MOTOR_RIGHT_CH.
//
// Open-loop duty only for now: PD wheel-velocity control + feedforward
// will layer on top in Control, calling motor_set_left/right each tick.
//
// NOTE on supply: NFP D1618 motors are rated 3 V but run from 7.4 V (2S),
// so full duty is ~2.5x rated voltage. Keep bench-test duties low; the
// velocity controller will own duty limits later.

#pragma once

#include <Arduino.h>

// Left/right mapping -- set from bench self-test 2026-07-02: channels were
// swapped (left wheel is TB6612 "B"); directions were already correct.
constexpr uint8_t MOTOR_LEFT_CH = 1;   // TB6612 channel B
constexpr uint8_t MOTOR_RIGHT_CH = 0;  // TB6612 channel A
constexpr float MOTOR_LEFT_SIGN = 1.0f;
constexpr float MOTOR_RIGHT_SIGN = 1.0f;

// Configures pins; leaves the driver in standby (outputs off).
void motors_init();

// Enable/disable the H-bridge outputs (TB6612 STBY).
void motors_enable(bool enable);

// Duty in [-1, +1], positive = forward. 0 = coast.
void motor_set_left(float duty);
void motor_set_right(float duty);

// Coast both motors (duty 0, outputs still enabled).
void motors_stop();
