#include "Sensors.h"

#include <string.h>

#include "HAL.h"

// Normal packet: 5 bytes [L,FR,FL,R, 0x0D]. RAW-cal packet: 13 bytes
// [L,FR,FL,R, px0,sig0,px1,sig1,px2,sig2,px3,sig3, 0x0D] -- same 4 mm
// bytes plus raw px+signal per channel (auto-cal, 2026-08-06). The
// active length switches with raw_mode (set by sensors_free_run_raw).
static constexpr size_t PACKET_LEN = 5;
static constexpr size_t PACKET_LEN_RAW = 13;
static constexpr uint8_t PACKET_TERM = 0x0D;  // packet terminator

static uint8_t rx_buf[PACKET_LEN_RAW];
static size_t rx_len = 0;
static bool raw_mode = false;

static WallRanges latest = {SENSOR_OUT_OF_RANGE, SENSOR_OUT_OF_RANGE,
                            SENSOR_OUT_OF_RANGE, SENSOR_OUT_OF_RANGE, 0};
static WallRaw latest_raw = {{0, 0, 0, 0}, {0, 0, 0, 0}, 0};
static uint32_t last_packet_ms = 0;

void sensors_init() {
    hal_uart_sensor_init();
    rx_len = 0;
    last_packet_ms = millis();
}

// Ready = a complete packet arrived recently (sub-MCU alive and framed).
bool sensors_ready() {
    return latest.seq > 0 && (millis() - last_packet_ms) < 500;
}

static void commit_packet() {
    latest.left_mm = rx_buf[0];
    // The two front beams angle inward and cross ~100 mm ahead. Beyond the
    // crossover -- where maze walls actually sit -- the physically-right sensor
    // (packet[1]) sees the LEFT-front region and the physically-left sensor
    // (packet[2]) sees the RIGHT-front region. Swap them here so front_left_mm /
    // front_right_mm carry the correct side at wall distances.
    latest.front_left_mm = rx_buf[1];
    latest.front_right_mm = rx_buf[2];
    latest.right_mm = rx_buf[3];
    if (raw_mode) {
        // Raw px/sig follow the 4 mm bytes, in the SAME channel order
        // (0=left, 1=fr, 2=fl, 3=right). NOTE: the mm fields above swap
        // fr/fl for the crossover; the raw arrays keep the wire order so
        // the cal pairs px with the channel that produced it.
        for (int i = 0; i < 4; i++) {
            latest_raw.px[i] = rx_buf[4 + 2 * i];
            latest_raw.sig[i] = rx_buf[5 + 2 * i];
        }
        latest_raw.seq = latest.seq + 1;
    }
    latest.seq++;
    last_packet_ms = millis();
}

static void send_cmd(uint8_t cmd) { hal_sensor_write(&cmd, 1); }

void sensors_free_run() { raw_mode = false; send_cmd(SENSOR_CMD_FREE_RUN); }
void sensors_free_run_interlaced() {
    raw_mode = false; send_cmd(SENSOR_CMD_FREE_RUN_IL);
}
void sensors_free_run_raw() { raw_mode = true; send_cmd(SENSOR_CMD_FREE_RUN_RAW); }
void sensors_stop() { raw_mode = false; send_cmd(SENSOR_CMD_STOP); }
void sensors_trigger() { raw_mode = false; send_cmd(SENSOR_CMD_TRIGGER); }

void sensors_poll() {
    // Drain the RX FIFO, keep the most recent complete packet. Length is
    // 5 normally, 13 in raw-cal mode; a full buffer not ending in 0x0D
    // means lost alignment (dropped byte) -> drop oldest and slide, which
    // re-locks within a packet or two.
    const size_t plen = raw_mode ? PACKET_LEN_RAW : PACKET_LEN;
    for (int c = hal_sensor_read(); c >= 0; c = hal_sensor_read()) {
        rx_buf[rx_len++] = (uint8_t)c;
        if (rx_len < plen) continue;
        if (rx_buf[plen - 1] == PACKET_TERM) {
            commit_packet();
            rx_len = 0;
        } else {
            memmove(rx_buf, rx_buf + 1, plen - 1);
            rx_len = plen - 1;
        }
    }
}

WallRanges sensors_latest() { return latest; }
WallRaw sensors_latest_raw() { return latest_raw; }

// Wait up to timeout_ms for a specific byte from the sub-MCU (draining and
// discarding anything else). Boot-time blocking helper.
static bool wait_for_byte(uint8_t want, uint32_t timeout_ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms) {
        int c = hal_sensor_read();
        if (c >= 0 && (uint8_t)c == want) return true;
    }
    return false;
}

bool sensors_ping(uint32_t timeout_ms) {
    while (hal_sensor_read() >= 0) { /* drain stale bytes */ }
    send_cmd(SENSOR_CMD_PING);
    return wait_for_byte(SENSOR_CMD_PING, timeout_ms);
}

bool sensors_upload_lut(uint8_t ch, const uint8_t lut[256]) {
    if (ch >= 4) return false;
    while (hal_sensor_read() >= 0) { /* drain */ }
    uint8_t pkt[259];
    pkt[0] = SENSOR_CMD_LOAD_LUT;
    pkt[1] = ch;
    uint8_t sum = ch;
    for (int i = 0; i < 256; i++) { pkt[2 + i] = lut[i]; sum += lut[i]; }
    pkt[258] = sum;
    hal_sensor_write(pkt, sizeof(pkt));
    // The sub replies ACK (written) or NAK (bad checksum). Treat NAK / no
    // reply as failure so the caller retries.
    uint32_t t0 = millis();
    while (millis() - t0 < 300) {
        int c = hal_sensor_read();
        if (c < 0) continue;
        if ((uint8_t)c == SENSOR_LUT_ACK) return true;
        if ((uint8_t)c == SENSOR_LUT_NAK) return false;
    }
    return false;
}

bool sensors_download_lut(uint8_t ch, uint8_t out[256]) {
    if (ch >= 4) return false;
    while (hal_sensor_read() >= 0) { /* drain */ }
    uint8_t cmd[2] = {SENSOR_CMD_DUMP_LUT, ch};
    hal_sensor_write(cmd, 2);
    uint8_t buf[257];
    int got = 0;
    uint32_t t0 = millis();
    while (got < 257 && millis() - t0 < 400) {
        int c = hal_sensor_read();
        if (c >= 0) buf[got++] = (uint8_t)c;
    }
    if (got < 257) return false;
    uint8_t sum = ch;
    for (int i = 0; i < 256; i++) sum += buf[i];
    if (sum != buf[256]) return false;
    for (int i = 0; i < 256; i++) out[i] = buf[i];
    return true;
}

bool sensors_threshold_report(SensorThresholds *out, bool rerun,
                              uint32_t timeout_ms) {
    if (!out) return false;
    *out = SensorThresholds();
    out->valid = false;
    while (hal_sensor_read() >= 0) { /* drain */ }
    uint8_t cmd[2] = {SENSOR_CMD_THRESH_REP, (uint8_t)(rerun ? 1 : 0)};
    hal_sensor_write(cmd, 2);
    // A re-run dark-reads four channels at up to 400 us each, twice over, plus
    // SPI writes -- so allow the caller's timeout rather than a fixed one.
    uint8_t buf[SENSOR_THRESH_REP_BYTES];
    int got = 0;
    uint32_t t0 = millis();
    while (got < SENSOR_THRESH_REP_BYTES && millis() - t0 < timeout_ms) {
        int c = hal_sensor_read();
        if (c >= 0) buf[got++] = (uint8_t)c;
    }
    if (got < SENSOR_THRESH_REP_BYTES) return false;
    uint8_t sum = 0;
    for (int i = 0; i < SENSOR_THRESH_REP_BYTES - 1; i++) sum += buf[i];
    if (sum != buf[SENSOR_THRESH_REP_BYTES - 1]) return false;
    for (int i = 0; i < 4; i++) {
        out->dark[i] = buf[i];
        out->thresh[i] = buf[4 + i];
        out->vmin[i] = buf[8 + i];
    }
    out->flags = buf[12];
    out->valid = true;
    return true;
}

uint32_t sensors_ms_since_packet() { return millis() - last_packet_ms; }
