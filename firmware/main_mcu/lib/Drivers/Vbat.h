// BATTERY VOLTAGE SENSE -- the instrument that has been missing since run
// 137 ("Battery voltage needs logging before any further cross-session
// comparison", SWEEP_FIELD_NOTES) and that run 210 made urgent: dutypk
// 0.65 on the last lap of a session against 0.40-0.49 fresh is a sagging
// pack asking for more PWM to make the same watts (bert, 2026-08-30).
//
// Hardware: 2S LiPo (8.4 V full) through a resistive divider into pin 23
// (A9) -- the one free analog pin (spec 5.6 never listed it; pin 21 = A7
// is RX5 and must not be touched). VBAT_DIVIDER is NOMINAL until
// calibrated: type 'vbat' with a multimeter on the pack, then set the
// constant to (meter_mV / reported_pin_mV) and reflash. The divider ratio
// is a property of two resistors; it will never need calibrating twice.
//
// MONITORING ONLY, deliberately. Battery voltage also belongs in the PWM
// feedforward (duty x VBAT_NOM/vbat would make the wheel gains
// voltage-invariant), but that changes the control plant -- it ships
// separately, after a session of vbat-vs-duty data says how big the lever
// is. One variable at a time.
//
// Reads are main-loop only (analogRead is not ISR-safe here), lightly
// EMA-filtered, refreshed at most every VBAT_PERIOD_MS so the poll can be
// called as often as convenient.

#pragma once

#include <Arduino.h>

constexpr uint8_t VBAT_PIN = 23;         // A9 (bert, 2026-08-30)
constexpr float VBAT_DIVIDER = 3.0f;     // NOMINAL -- calibrate via 'vbat'
constexpr float VBAT_ADC_REF_MV = 3300.0f;
constexpr int VBAT_ADC_MAX = 1023;       // Teensy default 10-bit
constexpr uint32_t VBAT_PERIOD_MS = 100;
constexpr float VBAT_EMA = 0.3f;         // per sample at 10 Hz

void vbat_init();

// Refresh if due (main loop). Cheap when not due.
void vbat_poll();

// Filtered pack voltage, mV. 0 until the first sample lands.
uint16_t vbat_mv();

// Console 'vbat': raw counts, pin mV, pack mV -- the calibration line.
void vbat_print();
