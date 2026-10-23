#include "Fram.h"

#include <string.h>

#include "HAL.h"

#include <stdio.h>

static bool ready = false;
static char id_text[40] = "none";
static char status_text[96] = "not initialised";
static uint32_t size_detected = 0;

// Does the chip have a 20th address bit? A 4 Mbit part ignores A19, so a
// byte written at (0x80000 + x) reads back at x. 0x1F0 is a free byte in
// the small-block window (Fram.h map); on a 1 MB chip 0x801F0 is inside
// the bulk log region. Both are restored.
static uint32_t probe_size() {
    const uint32_t lo = 0x1F0, hi = 0x80000 + 0x1F0;
    uint8_t a0, b0, t;
    hal_fram_read(lo, &a0, 1);
    hal_fram_read(hi, &b0, 1);
    uint8_t pat = (uint8_t)(~a0);
    hal_fram_write(hi, &pat, 1);
    hal_fram_read(lo, &t, 1);
    bool aliased = (t == pat);
    hal_fram_write(hi, &b0, 1);
    hal_fram_write(lo, &a0, 1);
    return aliased ? 512UL * 1024 : 1024UL * 1024;
}

bool fram_init() {
    hal_fram_init();
    uint32_t id4 = hal_fram_read_id();
    uint8_t idb[9];
    hal_fram_read_id_bytes(idb, 9);
    snprintf(id_text, sizeof id_text, "%02X %02X %02X %02X %02X %02X %02X %02X %02X",
             idb[0], idb[1], idb[2], idb[3], idb[4], idb[5], idb[6], idb[7], idb[8]);
    bool known = (id4 == FRAM_DEVICE_ID) || (id4 == FRAM_DEVICE_ID_JEDEC_CONT);
    if (!known) {
        ready = false;
        size_detected = 0;
        snprintf(status_text, sizeof status_text,
                 "no FRAM: RDID %08lX is neither Ramxeed nor Infineon",
                 (unsigned long)id4);
        return false;
    }
    size_detected = probe_size();
    if (size_detected < FRAM_SIZE_BYTES) {
        ready = false;
        snprintf(status_text, sizeof status_text,
                 "FRAM is %lu KB but this build expects %lu KB (FRAM_MBIT) -- "
                 "refusing to run on it",
                 (unsigned long)(size_detected / 1024),
                 (unsigned long)(FRAM_SIZE_BYTES / 1024));
        return false;
    }
    ready = true;
    snprintf(status_text, sizeof status_text, "%s %lu KB%s",
             id4 == FRAM_DEVICE_ID ? "Ramxeed/Fujitsu" : "Infineon/Cypress",
             (unsigned long)(size_detected / 1024),
             size_detected > FRAM_SIZE_BYTES ? " (build uses the lower half: FRAM_MBIT)" : "");
    return true;
}

bool fram_ready() { return ready; }
const char *fram_id_text() { return id_text; }
uint32_t fram_size_detected() { return size_detected; }
const char *fram_status_text() { return status_text; }

void fram_read(uint32_t addr, void *buf, size_t len) {
    if (!ready) {
        memset(buf, 0, len);
        return;
    }
    hal_fram_read(addr, static_cast<uint8_t *>(buf), len);
}

void fram_write(uint32_t addr, const void *buf, size_t len) {
    if (!ready) return;
    hal_fram_write(addr, static_cast<const uint8_t *>(buf), len);
}
