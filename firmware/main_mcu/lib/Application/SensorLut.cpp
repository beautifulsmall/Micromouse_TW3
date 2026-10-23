#include "SensorLut.h"

#include "Fram.h"
#include "Sensors.h"

// Header lives at SENSOR_LUT_FRAM_ADDR; data follows.
static constexpr uint32_t HDR_ADDR = SENSOR_LUT_FRAM_ADDR;
static constexpr uint32_t DATA_ADDR = SENSOR_LUT_FRAM_ADDR + 8;
static constexpr uint32_t DATA_BYTES = SENSOR_LUT_CHANNELS * 256;  // 1024

struct LutHeader {
    uint32_t magic;
    uint8_t version;
    uint8_t valid;    // bitmask
    uint16_t crc;
};

// CRC-16/CCITT over the whole 4x256 data region (read from FRAM).
static uint16_t data_crc() {
    uint16_t crc = 0xFFFF;
    uint8_t buf[256];
    for (int ch = 0; ch < SENSOR_LUT_CHANNELS; ch++) {
        fram_read(DATA_ADDR + ch * 256, buf, 256);
        for (int i = 0; i < 256; i++) {
            crc ^= (uint16_t)buf[i] << 8;
            for (int b = 0; b < 8; b++)
                crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

static void read_hdr(LutHeader &h) { fram_read(HDR_ADDR, &h, sizeof(h)); }

static bool hdr_ok(const LutHeader &h) {
    return h.magic == SENSOR_LUT_MAGIC && h.version == SENSOR_LUT_VERSION;
}

void sensor_lut_store(uint8_t ch, const uint8_t lut[256]) {
    if (ch >= SENSOR_LUT_CHANNELS || !fram_ready()) return;
    LutHeader h;
    read_hdr(h);
    if (!hdr_ok(h)) {                 // fresh block: zero all channels
        uint8_t zero[256];
        for (int i = 0; i < 256; i++) zero[i] = 255;  // 255 = out of range
        for (int c = 0; c < SENSOR_LUT_CHANNELS; c++)
            fram_write(DATA_ADDR + c * 256, zero, 256);
        h.magic = SENSOR_LUT_MAGIC;
        h.version = SENSOR_LUT_VERSION;
        h.valid = 0;
    }
    fram_write(DATA_ADDR + ch * 256, lut, 256);
    h.valid |= (uint8_t)(1u << ch);
    h.crc = data_crc();              // recompute over the updated region
    fram_write(HDR_ADDR, &h, sizeof(h));
}

bool sensor_lut_get(uint8_t ch, uint8_t lut[256]) {
    if (ch >= SENSOR_LUT_CHANNELS || !fram_ready()) return false;
    LutHeader h;
    read_hdr(h);
    if (!hdr_ok(h) || !(h.valid & (1u << ch))) return false;
    fram_read(DATA_ADDR + ch * 256, lut, 256);
    return true;
}

uint8_t sensor_lut_valid_mask() {
    if (!fram_ready()) return 0;
    LutHeader h;
    read_hdr(h);
    if (!hdr_ok(h)) return 0;
    if (h.crc != data_crc()) return 0;   // corrupt -> treat as absent
    return h.valid;
}

bool sensor_lut_present() { return sensor_lut_valid_mask() != 0; }

void sensor_lut_clear() {
    if (!fram_ready()) return;
    LutHeader h;
    read_hdr(h);
    h.magic = 0;                     // invalidate; leave data in place
    fram_write(HDR_ADDR, &h, sizeof(h));
}

int sensor_lut_pull_all() {
    if (!fram_ready()) return -1;
    bool up = false;
    for (int i = 0; i < 6 && !up; i++) up = sensors_ping(500);
    if (!up) return -1;
    int n = 0;
    uint8_t lut[256];
    for (uint8_t ch = 0; ch < SENSOR_LUT_CHANNELS; ch++) {
        if (sensors_download_lut(ch, lut)) { sensor_lut_store(ch, lut); n++; }
    }
    return n;
}

int sensor_lut_upload_all() {
    uint8_t mask = sensor_lut_valid_mask();
    if (mask == 0) return 0;         // nothing stored -> compiled defaults
    bool up = false;                 // the sub-MCU may still be detecting
    for (int i = 0; i < 6 && !up; i++) up = sensors_ping(500);  // ~3 s budget
    if (!up) return -1;              // sub-MCU not answering -> skip
    int n = 0;
    uint8_t lut[256];
    for (uint8_t ch = 0; ch < SENSOR_LUT_CHANNELS; ch++) {
        if (!(mask & (1u << ch))) continue;
        if (!sensor_lut_get(ch, lut)) continue;
        bool ok = false;
        for (int tries = 0; tries < 3 && !ok; tries++)
            ok = sensors_upload_lut(ch, lut);
        if (ok) n++;
    }
    return n;
}
