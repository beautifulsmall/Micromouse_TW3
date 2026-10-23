// Minimal Arduino.h stub for host-side tests (no hardware). Declarations
// only -- each test supplies its own definitions/mocks.
#pragma once

#include <stddef.h>
#include <stdint.h>

#define HIGH 1
#define LOW 0
#define OUTPUT 1

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
void delay(unsigned long);
unsigned long millis();
unsigned long micros();
void noInterrupts();
void interrupts();
