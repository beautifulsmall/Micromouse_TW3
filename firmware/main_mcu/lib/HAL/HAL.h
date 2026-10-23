// HAL / BSP layer.
//
// Thin wrappers around the Main MCU's peripherals, one function group per
// bus/interface defined in micromouse_spec.md section 5 (Pin Connections):
//
//   - SPI bus (ICM-45686 IMU, FRAM)               -- section 5.1
//   - UART2 -- sensor sub-processor link           -- section 5.2
//   - UART5 -- ESP32 radio link                    -- section 5.3
//   - PWM / GPIO -- TB6612FNG motor driver          -- section 5.4
//   - Quadrature decoder inputs (encoders)          -- section 5.5
//   - Status LEDs                                   -- section 5.6
//   - Fixed-rate control timer (IntervalTimer)
//
// No sensor- or device-specific logic belongs here -- that's the Drivers
// layer. This layer only wraps the raw pins/buses (init, read, write) so
// drivers don't touch Arduino's SPI/Serial/register access directly.

#pragma once

#include <Arduino.h>

// -- Status LEDs (micromouse_spec.md section 5.6) --------------------------
void hal_init();
void hal_set_led1(bool on);
void hal_set_led2(bool on);

// -- SPI bus + IMU chip select (micromouse_spec.md section 5.1) ------------
// Teensy 4.0 default SPI pins: SCK=13, MISO=12, MOSI=11. IMU CS = pin 10.
// ICM-45686 SPI: mode 0, max 24 MHz, MSB first; register address byte has
// the R/W bit in bit 7 (1 = read) -- datasheet "SPI Operational Features".
void hal_spi_init();
void hal_imu_init();  // CS0 (pin 10) idle-high

uint8_t hal_imu_read_reg(uint8_t reg);
void hal_imu_read_regs(uint8_t reg, uint8_t *buf, size_t len);  // burst read
void hal_imu_write_reg(uint8_t reg, uint8_t val);

// -- Motor driver, TB6612FNG (micromouse_spec.md section 5.4) ----------------
// Channel 0 = TB6612 "A" (PWMA=9, AIN1=15, AIN2=14),
// channel 1 = TB6612 "B" (PWMB=19, BIN1=17, BIN2=18), STBY=16.
// PWM: 20 kHz (above audible; the NFP D1618 coreless motors have low
// inductance, so a high carrier keeps ripple current down), 10-bit duty.
// Which channel is the left wheel, and which polarity is forward, is the
// Drivers layer's job (Motors.h).
void hal_motors_init();               // configures pins; starts in standby
void hal_motor_standby(bool standby); // true = driver outputs disabled
// duty in [-1, +1]: sign selects IN1/IN2 polarity, magnitude the PWM.
// duty == 0 -> coast (both INs low). Safe to call from the control ISR.
void hal_motor_set(uint8_t channel, float duty);

// -- FRAM, MB85RS4MTY (micromouse_spec.md section 5.1, CS1 = pin 6) ----------
// 512 KB SPI FeRAM: no write latency, no page limits, 10^13 endurance.
// SPI mode 0, READ command max 40 MHz (datasheet AC characteristics); runs
// on the shared SPI bus at the same conservative 12 MHz as the IMU.
//
// NOTE: the 500 Hz control ISR owns the SPI bus (IMU FIFO drain), so every
// FRAM transaction here briefly masks interrupts. Keep transfers small
// (tens of bytes) to bound control-tick jitter.
void hal_fram_init();       // CS1 (pin 6) idle-high
uint32_t hal_fram_read_id();  // RDID, first 4 bytes: 0x047F490B (Fujitsu/Ramxeed 4Mbit)
// RDID, the first n bytes (Infineon/Cypress parts answer 9: six 0x7F
// JEDEC continuation codes, then manufacturer, then two of part/density).
void hal_fram_read_id_bytes(uint8_t *out, int n);
void hal_fram_read(uint32_t addr, uint8_t *buf, size_t len);
void hal_fram_write(uint32_t addr, const uint8_t *buf, size_t len);

// -- Quadrature encoders (micromouse_spec.md section 5.5) -------------------
// Left encoder: pins 0 (A) / 1 (B); right encoder: pins 2 (A) / 3 (B).
// Decoded by the i.MX RT1062's hardware ENC modules (via the Teensyduino
// QuadEncoder library) -- x4 quadrature counting in silicon, no interrupts,
// no CPU cost per edge. Counts are raw hardware direction: sign
// normalisation (mirrored motors) is the Drivers layer's job.
void hal_encoders_init();
int32_t hal_encoder_left();   // raw signed count
int32_t hal_encoder_right();
void hal_encoders_zero();

// -- Fixed-rate control timer ----------------------------------------------
// Runs `isr` at `hz` from a hardware IntervalTimer (per the ADR: fixed-rate
// ISR instead of an RTOS). Returns false if no timer slot was available.
bool hal_control_timer_begin(void (*isr)(), float hz);

// -- ESP32 radio link (micromouse_spec.md sections 5.3 and 7) ---------------
// Serial5 (pins 20/21) at 460800 baud to the ESP32 Super Mini, which
// bridges to ESP-NOW off-robot. Byte-pipe only: framing/meaning is the
// caller's business (Console.cpp mirrors the bench console over it).
//
// COMPILE-TIME DISABLE. Set HAL_RADIO_ENABLED to 0 when the ESP32 module is
// not fitted (it is worth real battery on a 350 mAh pack). With it off,
// Serial5 is never begun, writes are discarded and reads always return -1.
//
// This matters for more than power. With the module removed, RX5 (pin 21)
// FLOATS: noise makes start bits, the UART decodes garbage characters, and
// those characters go straight into the console parser. A single stray 'm'
// is a motor self-test; 'e' aborts a mission. On a mouse running an
// unattended experiment for six minutes that is a real failure mode, not a
// theoretical one. So: disabled means genuinely not read, and even when
// ENABLED the RX pin is pulled up so an unpowered module cannot inject.
//
// ENABLED BY DEFAULT from 2026-08-21 (bert). The reason is calibration, not
// telemetry: 'j' (the wheel-scale test) and the other floor tests want the
// mouse UNTOUCHED when they start, because a hand on the shell at t=0 is a
// disturbance the measurement then has to live with. Triggering over the
// radio removes the operator from the experiment.
//
// Leaving the module OFF the robot is still safe with this enabled, which is
// what makes the default defensible: hal_uart_radio_init() sets pin 21
// INPUT_PULLUP before Serial5.begin(), so an absent or unpowered module
// leaves the line idle-high and no start bit is ever framed.
//
// A second layer, worth knowing because it is not obvious from the parser:
// main.cpp only dispatches a line on CR or LF. A lone noise byte therefore
// cannot execute anything -- it sits in the buffer until a terminator
// arrives. Two separate corruptions are needed, not one.
//
// THE COST IS BATTERY, and that is not a footnote right now. The R125
// control channel drifted across runs 130-137 and came back down at 137,
// which points at charge state; adding a powered ESP32 to the load changes
// the discharge profile. Fine for bench and calibration work. For sweep
// sessions being compared against each other, either leave it on for all of
// them or off for all of them -- do not mix.
#ifndef HAL_RADIO_ENABLED
#define HAL_RADIO_ENABLED 1
#endif
void hal_uart_radio_init();
// NON-BLOCKING, and BUFFERED since 2026-08-21. Writes land in a 1 KB ring;
// hal_radio_poll() drains whatever the UART has room for each main-loop
// pass. Never stalls the main loop behind a 460800-baud drain while a
// control loop is flying the mouse.
//
// It buffers because the earlier version discarded whatever did not fit
// RIGHT NOW, and the Teensy TX buffer is only a few dozen bytes -- so any
// console line longer than that silently lost its tail. That is harmless for
// telemetry and destructive for a measurement: the scale test printed
// "believed L 906.7  R 90" and the R value, plus the whole correction line
// after it, went in the bin. Bulk dumps still belong on USB, which is faster
// and is where you are standing anyway.
size_t hal_radio_write(const uint8_t *buf, size_t len);
// Drain the ring into the UART. Call once per main-loop pass. Cheap when
// there is nothing to send.
void hal_radio_poll();
// BULK DUMPS PACE THEMSELVES. The ring is 1 KB and drops NEW bytes on
// overrun, so any burst over ~1.1 KB loses its tail: the 43-line PARAMDEF
// dump lost 'dtrim' (2026-09-01), the first lap report after a reflash lost
// six of nine LC lines (seq 288). A dump that runs while the mouse is
// STATIONARY may simply wait for room -- this drains the ring until at least
// `bytes` are free or `timeout_ms` passes. Never call it from anything that
// can run while flying: the flight loop must not block, and the per-packet
// console lines are sized to fit.
void hal_radio_wait_room(uint16_t bytes, uint32_t timeout_ms);
// Bytes lost to a full ring since boot -- 0 unless something is generating
// output faster than 46 kB/s can carry it away.
uint32_t hal_radio_dropped();
int hal_radio_read();  // next received byte, or -1 if none pending

// -- Sensor sub-processor link (micromouse_spec.md section 5.2) --------------
// Serial2 (pins 7=RX2, 8=TX2) to the sensor sub-MCU. Runs at 460800 to match
// the sub-MCU sketch (TW3_sensor_sub_mcu.ino), not the 115200 the spec lists.
// Byte-pipe only: the command/packet protocol is the Drivers layer (Sensors.h).
void hal_uart_sensor_init();
size_t hal_sensor_write(const uint8_t *buf, size_t len);
int hal_sensor_read();  // next received byte, or -1 if none pending
