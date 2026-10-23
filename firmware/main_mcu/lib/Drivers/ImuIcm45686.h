// ICM-45686 IMU driver (6-DOF accel + gyro), FIFO-based.
//
// Reads accel XYZ + gyro XYZ through the sensor's FIFO so no samples are
// missed between control-loop ticks. All register addresses, bitfields, and
// scale factors are from the ICM-45686 datasheet (DS-000577 rev 1.0);
// section references are noted at each constant in the .cpp.
//
// Configuration chosen here (adjust the constants in the .cpp):
//   - Accel: +/-4 g full scale, 6.4 kHz ODR (max), Low Noise mode
//   - Gyro:  +/-4000 dps full scale (max), 6.4 kHz ODR (max), Low Noise mode
//   - FIFO: stream mode, 2 KB depth, 16-byte accel+gyro packets
//
// With the control loop draining the FIFO at 500 Hz, each tick sees ~13
// packets (6.4 kHz ODR / 500 Hz drain); the 2 KB FIFO (128 packets, ~20 ms)
// absorbs several delayed ticks before overflow.
//
// Device data endianness is left at its power-on default (Little Endian),
// so multi-byte FIFO fields arrive low byte first.

#pragma once

#include <Arduino.h>

// One decoded 16-byte accel+gyro FIFO packet.
struct ImuSample {
    int16_t ax, ay, az;   // raw accel, 8192 LSB/g  (+/-4 g)
    int16_t gx, gy, gz;   // raw gyro,  8.2 LSB/dps (+/-4000 dps)
    int8_t temp_raw;      // die temp, degC = raw/2 + 25
    uint16_t timestamp;   // 16-bit sensor timestamp field
};

// Resets and configures the device. Requires hal_spi_init() and
// hal_imu_init() to have been called. Returns false if the WHO_AM_I check
// fails (wiring/SPI problem).
bool imu_init();

// Drains up to `max_samples` packets from the FIFO into `out`.
// Returns the number of packets read. Safe to call from the control ISR.
size_t imu_read_fifo(ImuSample *out, size_t max_samples);

// Number of packets currently waiting in the FIFO.
uint16_t imu_fifo_count();

// Raw-to-physical-unit conversions for the configuration above.
inline float imu_accel_g(int16_t raw) { return raw / 8192.0f; }     // +/-4 g

// THE MEASURED GYRO SCALE (2026-09-22). The datasheet's 8.2 LSB/dps is the
// nominal; this part reads 0.1875 % MORE rotation than the world. Measured
// five times, by fitting the fused yaw to the walls' corridor direction
// over 45 flight logs (WALL_FRAME.md, 2026-09-05: the launch straight's
// heading against the late straights' after a net 270 of rotation, CW and
// CCW -- the shift flips sign with the hand of the lap, which is a scale
// error, not a bias), 0.1875 % +/- 0.019. It is MOUSE property, so it is
// compiled, not a registry row (the registry rule): 0.17 deg per 90, 0.34
// per 180, ~0.5 deg per lap of same-hand turns, which the wall frame was
// re-trimming at the next two-wall straight. Now the turns land on the
// world's 90, not the gyro's. This is the measured correction and NOT a
// re-derived datasheet sensitivity (32768/4000 = 8.192 would move it the
// wrong way).
constexpr float IMU_GYRO_SCALE_MEAS = 1.001875f;   // gyro / world
inline float imu_gyro_dps(int16_t raw) {
    return raw / (8.2f * IMU_GYRO_SCALE_MEAS);        // +/-4000 dps
}
inline float imu_temp_c(int8_t raw) { return raw / 2.0f + 25.0f; }  // datasheet 17.16
