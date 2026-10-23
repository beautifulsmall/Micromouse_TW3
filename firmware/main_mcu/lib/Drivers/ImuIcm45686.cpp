#include "ImuIcm45686.h"

#include "HAL.h"

// -- Register map (ICM-45686 datasheet, user bank 0) -------------------------
static constexpr uint8_t REG_PWR_MGMT0 = 0x10;      // 17.17
static constexpr uint8_t REG_FIFO_COUNT_0 = 0x12;   // 17.18
static constexpr uint8_t REG_FIFO_DATA = 0x14;      // 17.20
static constexpr uint8_t REG_ACCEL_CONFIG0 = 0x1B;  // 17.26
static constexpr uint8_t REG_GYRO_CONFIG0 = 0x1C;   // 17.27
static constexpr uint8_t REG_FIFO_CONFIG0 = 0x1D;   // 17.28
static constexpr uint8_t REG_FIFO_CONFIG2 = 0x20;   // 17.31
static constexpr uint8_t REG_FIFO_CONFIG3 = 0x21;   // 17.32
static constexpr uint8_t REG_WHO_AM_I = 0x72;       // 17.79
static constexpr uint8_t REG_MISC2 = 0x7F;          // 17.84

static constexpr uint8_t WHO_AM_I_VALUE = 0xE9;  // 17.79

// PWR_MGMT0 (17.17): GYRO_MODE[3:2]=11 (Low Noise), ACCEL_MODE[1:0]=11 (LN)
static constexpr uint8_t PWR_MGMT0_ACCEL_GYRO_LN = 0x0F;

// ACCEL_CONFIG0 (17.26): FS_SEL[6:4]=011 (+/-4 g), ODR[3:0]=0011 (6.4 kHz LN, max)
// If you change these, update imu_accel_g() in the header to match.
static constexpr uint8_t ACCEL_CONFIG0_4G_6400HZ = 0x33;

// GYRO_CONFIG0 (17.27): FS_SEL[7:4]=0000 (+/-4000 dps, max), ODR[3:0]=0011 (6.4 kHz LN, max)
// If you change these, update imu_gyro_dps() in the header to match.
static constexpr uint8_t GYRO_CONFIG0_4000DPS_6400HZ = 0x03;

// FIFO_CONFIG0 (17.28): FIFO_MODE[7:6]=01 (stream), FIFO_DEPTH[5:0]=000111
// (2 KB, the datasheet-recommended setting)
static constexpr uint8_t FIFO_CONFIG0_STREAM_2K = 0x47;

// FIFO_CONFIG2 (17.31): bit 7 = FIFO_FLUSH (self-clearing)
static constexpr uint8_t FIFO_CONFIG2_FLUSH = 0x80;

// FIFO_CONFIG3 (17.32): bit2 FIFO_GYRO_EN | bit1 FIFO_ACCEL_EN | bit0 FIFO_IF_EN
static constexpr uint8_t FIFO_CONFIG3_ACCEL_GYRO_EN = 0x07;

// REG_MISC2 (17.84): bit 1 = SOFT_RST (self-clearing)
static constexpr uint8_t MISC2_SOFT_RST = 0x02;

// FIFO header byte ("FIFO Header" section): with accel+gyro enabled and no
// high-res / external sensors, expect EXT_HEADER(7)=0, ACCEL_EN(6)=1,
// GYRO_EN(5)=1, HIRES_EN(4)=0. Bits 3:0 (timestamp/FSYNC/ODR-change flags)
// vary, so mask them off for validation.
static constexpr uint8_t FIFO_HDR_MASK = 0xF0;
static constexpr uint8_t FIFO_HDR_ACCEL_GYRO = 0x60;

static constexpr size_t FIFO_PACKET_SIZE = 16;  // accel+gyro packet

// Max packets pulled per burst read; bounds ISR time. At 6.4 kHz ODR and a
// 500 Hz drain rate the steady state is ~13 packets per call; 32 gives
// headroom to catch up after a delayed tick (32 x 16 B at 12 MHz SPI is
// ~350 us, well inside the 2 ms tick).
static constexpr size_t FIFO_MAX_BURST = 32;

bool imu_init() {
    // Soft reset, then wait for the device to come back (datasheet 17.84;
    // reset completes well within a few ms).
    hal_imu_write_reg(REG_MISC2, MISC2_SOFT_RST);
    delay(5);

    if (hal_imu_read_reg(REG_WHO_AM_I) != WHO_AM_I_VALUE) {
        return false;
    }

    // Sensor config: full scale + ODR, then FIFO, then power modes.
    hal_imu_write_reg(REG_ACCEL_CONFIG0, ACCEL_CONFIG0_4G_6400HZ);
    hal_imu_write_reg(REG_GYRO_CONFIG0, GYRO_CONFIG0_4000DPS_6400HZ);

    // FIFO enable sequence per datasheet 17.32: (1) enable FIFO (stream
    // mode), (2) enable the sensor-registers-to-FIFO interface.
    hal_imu_write_reg(REG_FIFO_CONFIG0, FIFO_CONFIG0_STREAM_2K);
    hal_imu_write_reg(REG_FIFO_CONFIG3, FIFO_CONFIG3_ACCEL_GYRO_EN);

    // Accel + gyro on, Low Noise mode. Gyro output needs tens of ms to
    // settle after enable; first samples will show that ramp.
    hal_imu_write_reg(REG_PWR_MGMT0, PWR_MGMT0_ACCEL_GYRO_LN);

    // Discard anything captured during configuration.
    uint8_t cfg2 = hal_imu_read_reg(REG_FIFO_CONFIG2);
    hal_imu_write_reg(REG_FIFO_CONFIG2, cfg2 | FIFO_CONFIG2_FLUSH);

    return true;
}

uint16_t imu_fifo_count() {
    // FIFO_COUNT_0/1 (17.18/17.19) hold the count *in packets*. With the
    // device's default Little Endian data mode, the low byte comes first
    // (the register-map tables document the Big Endian layout -- see the
    // "Device Configuration for Data Endianness" section).
    uint8_t raw[2];
    hal_imu_read_regs(REG_FIFO_COUNT_0, raw, 2);
    return static_cast<uint16_t>(raw[0]) | (static_cast<uint16_t>(raw[1]) << 8);
}

// Decode one 16-byte packet (little-endian field order: low byte first).
// Layout ("Packet Structure" section): header, Ax, Ay, Az, Gx, Gy, Gz
// (2 bytes each), temp (1 byte), timestamp (2 bytes).
static void decode_packet(const uint8_t *p, ImuSample &s) {
    auto le16 = [](const uint8_t *b) {
        return static_cast<int16_t>(static_cast<uint16_t>(b[0]) |
                                    (static_cast<uint16_t>(b[1]) << 8));
    };
    s.ax = le16(p + 1);
    s.ay = le16(p + 3);
    s.az = le16(p + 5);
    s.gx = le16(p + 7);
    s.gy = le16(p + 9);
    s.gz = le16(p + 11);
    s.temp_raw = static_cast<int8_t>(p[13]);
    s.timestamp = static_cast<uint16_t>(p[14]) | (static_cast<uint16_t>(p[15]) << 8);
}

size_t imu_read_fifo(ImuSample *out, size_t max_samples) {
    uint16_t available = imu_fifo_count();
    size_t n = available;
    if (n > max_samples) n = max_samples;
    if (n > FIFO_MAX_BURST) n = FIFO_MAX_BURST;
    if (n == 0) return 0;

    uint8_t buf[FIFO_MAX_BURST * FIFO_PACKET_SIZE];
    hal_imu_read_regs(REG_FIFO_DATA, buf, n * FIFO_PACKET_SIZE);

    size_t decoded = 0;
    for (size_t i = 0; i < n; i++) {
        const uint8_t *p = buf + i * FIFO_PACKET_SIZE;
        if ((p[0] & FIFO_HDR_MASK) != FIFO_HDR_ACCEL_GYRO) {
            // Unexpected packet type -- FIFO framing is off. Flush so the
            // next read starts clean rather than decoding garbage.
            uint8_t cfg2 = hal_imu_read_reg(REG_FIFO_CONFIG2);
            hal_imu_write_reg(REG_FIFO_CONFIG2, cfg2 | FIFO_CONFIG2_FLUSH);
            break;
        }
        decode_packet(p, out[decoded]);
        decoded++;
    }
    return decoded;
}
