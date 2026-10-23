// Host-side test: mock HAL, feed synthetic FIFO packets, verify decode.
#include "ImuIcm45686.h"
#include <cassert>
#include <cstring>
#include <cstdio>
#include <vector>

static std::vector<uint8_t> fake_fifo;
static uint16_t fake_count = 0;

void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void delay(unsigned long) {}
unsigned long millis() { return 0; }
void noInterrupts() {}
void interrupts() {}
struct HWSerial_ {} SerialObj;

uint8_t hal_imu_read_reg(uint8_t reg) {
    if (reg == 0x72) return 0xE9;
    if (reg == 0x20) return 0x20;
    return 0;
}
void hal_imu_read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    if (reg == 0x12) { buf[0] = fake_count & 0xFF; buf[1] = fake_count >> 8; return; }
    if (reg == 0x14) { memcpy(buf, fake_fifo.data(), len); return; }
    memset(buf, 0, len);
}
void hal_imu_write_reg(uint8_t, uint8_t) {}
bool hal_control_timer_begin(void (*)(), float) { return true; }

static void push_packet(int16_t ax, int16_t ay, int16_t az,
                        int16_t gx, int16_t gy, int16_t gz,
                        int8_t temp, uint16_t ts, uint8_t hdr = 0x68) {
    uint8_t p[16];
    p[0] = hdr;
    auto le = [&](int idx, int16_t v) { p[idx] = v & 0xFF; p[idx+1] = (uint16_t)v >> 8; };
    le(1, ax); le(3, ay); le(5, az); le(7, gx); le(9, gy); le(11, gz);
    p[13] = (uint8_t)temp; p[14] = ts & 0xFF; p[15] = ts >> 8;
    fake_fifo.insert(fake_fifo.end(), p, p + 16);
    fake_count++;
}

int main() {
    assert(imu_init());

    // 2 packets: 1g on Z (8192), -1g (-8192); gyro raw values round-trip
    push_packet(0, 0, 8192, 1641, -1641, 0, 10, 0x1234);
    push_packet(0, 0, -8192, 0, 0, 16400, -20, 0x1235);

    ImuSample s[4];
    size_t n = imu_read_fifo(s, 4);
    assert(n == 2);
    assert(s[0].az == 8192 && s[1].az == -8192);
    assert(s[0].gx == 1641 && s[0].gy == -1641);
    assert(s[1].gz == 16400);
    assert(s[0].temp_raw == 10 && s[1].temp_raw == -20);
    assert(s[0].timestamp == 0x1234);

    // Conversions
    assert(imu_accel_g(8192) == 1.0f);
    // 8200 LSB is the datasheet's 1000 dps; the measured scale (the part
    // over-reads by 0.1875 %) makes it 998.13 dps of world rotation.
    float dps = imu_gyro_dps(8200);
    assert(dps > 998.0f && dps < 998.3f);
    assert(IMU_GYRO_SCALE_MEAS > 1.0018f && IMU_GYRO_SCALE_MEAS < 1.0020f);
    assert(imu_temp_c(10) == 30.0f && imu_temp_c(-20) == 15.0f);

    // Bad header -> stops decoding, returns 0
    fake_fifo.clear(); fake_count = 0;
    push_packet(1, 2, 3, 4, 5, 6, 7, 8, /*hdr=*/0x80);  // EXT_HEADER set
    n = imu_read_fifo(s, 4);
    assert(n == 0);

    // Empty FIFO
    fake_fifo.clear(); fake_count = 0;
    assert(imu_read_fifo(s, 4) == 0);

    printf("ALL FIFO DECODE TESTS PASSED\n");
    return 0;
}
