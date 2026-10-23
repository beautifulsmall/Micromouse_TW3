// Dual console: USB serial + the ESP32 radio link, as one stream.
//
// Everything the firmware prints goes to BOTH USB serial and Serial5
// (-> ESP32 -> ESP-NOW, spec section 7), and commands are accepted from
// either, so bench runs work untethered.
//
// This is its own module in V2 because three separate things print through
// it -- boot (src/main.cpp), the bench harnesses (Bench.h) and the search
// executor (Search.h, via search_set_printer) -- and in V1 it lived as
// file-static functions inside main.cpp, which is one of the reasons
// everything else ended up inside main.cpp too.

#pragma once

#include <Arduino.h>

// Raw write (no newline added).
void con_write(const char *s);

// printf into a 256-byte buffer, then con_write. Not reentrant: it uses one
// static buffer, so do not call it from an ISR.
void con_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Write plus a trailing newline. Matches the void(*)(const char*) signature
// Search and FlightLog take as their line printer.
void con_println(const char *s);
