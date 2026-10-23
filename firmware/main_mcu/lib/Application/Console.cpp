#include "Console.h"

#include <stdarg.h>
#include <string.h>

#include "HAL.h"

void con_write(const char *s) {
    // ONLY write to USB when a terminal is actually open. `if (Serial)` is
    // true on Teensy once the host asserts DTR, i.e. something is listening
    // and draining the CDC endpoint. Writing into a configured-but-unread
    // endpoint is the one way the USB stack can hold up the main loop, and
    // this mouse spends its time flying autonomous runs with nobody
    // attached. The radio path is separately non-blocking (HAL.h) and
    // compiles out entirely when the module is not fitted.
    if (Serial) Serial.print(s);
    hal_radio_write(reinterpret_cast<const uint8_t *>(s), strlen(s));
}

void con_printf(const char *fmt, ...) {
    static char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    con_write(buf);
}

void con_println(const char *s) {
    con_write(s);
    con_write("\n");
}
