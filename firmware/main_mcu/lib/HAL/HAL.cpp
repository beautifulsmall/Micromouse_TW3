#include "HAL.h"

#include <SPI.h>

// Pin assignments from micromouse_spec.md, sections 5.1 and 5.6.
static constexpr uint8_t PIN_LED1 = 4;
static constexpr uint8_t PIN_LED2 = 5;
static constexpr uint8_t PIN_IMU_CS = 10;  // CS0 (IMU), section 5.1

// ICM-45686 supports up to 24 MHz SPI. Set to 10 MHz per bert 2026-08-06.
// NOTE: this was 12 MHz (higher) -- 10 is a slight de-rate; if the intent
// was "faster IMU reads", raise toward 20-24 MHz instead. Mode 0 (data
// latched on rising edge), MSB first -- datasheet "SPI Operational Features".
static const SPISettings IMU_SPI_SETTINGS(10000000, MSBFIRST, SPI_MODE0);

static constexpr uint8_t SPI_READ_FLAG = 0x80;  // bit 7 of address byte

void hal_init() {
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
}

void hal_set_led1(bool on) {
    digitalWrite(PIN_LED1, on ? HIGH : LOW);
}

void hal_set_led2(bool on) {
    digitalWrite(PIN_LED2, on ? HIGH : LOW);
}

// -- SPI + IMU ---------------------------------------------------------------

void hal_spi_init() {
    SPI.begin();  // Teensy 4.0 default pins: SCK=13, MISO=12, MOSI=11
}

void hal_imu_init() {
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);  // idle deselected
}

uint8_t hal_imu_read_reg(uint8_t reg) {
    uint8_t val;
    hal_imu_read_regs(reg, &val, 1);
    return val;
}

void hal_imu_read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    SPI.beginTransaction(IMU_SPI_SETTINGS);
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.transfer(reg | SPI_READ_FLAG);
    for (size_t i = 0; i < len; i++) {
        buf[i] = SPI.transfer(0x00);
    }
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.endTransaction();
}

void hal_imu_write_reg(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(IMU_SPI_SETTINGS);
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.transfer(reg);  // bit 7 = 0 -> write
    SPI.transfer(val);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.endTransaction();
}

// -- FRAM (MB85RS4MTY, 512 KB SPI FeRAM) --------------------------------------
// Opcodes and timing from the RAMXEED DS501-00065-3v0-E datasheet. Writes
// need a WREN frame first (WEL auto-clears after each WRITE); there is no
// write-completion wait -- FeRAM commits at bus speed. 24-bit address, upper
// 5 bits don't care.

static constexpr uint8_t PIN_FRAM_CS = 6;  // CS1, spec section 5.1
static const SPISettings FRAM_SPI_SETTINGS(12000000, MSBFIRST, SPI_MODE0);

static constexpr uint8_t FRAM_OP_WREN = 0x06;
static constexpr uint8_t FRAM_OP_READ = 0x03;
static constexpr uint8_t FRAM_OP_WRITE = 0x02;
static constexpr uint8_t FRAM_OP_RDID = 0x9F;

void hal_fram_init() {
    pinMode(PIN_FRAM_CS, OUTPUT);
    digitalWrite(PIN_FRAM_CS, HIGH);  // idle deselected
}

static void fram_send_addr(uint32_t addr) {
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >> 8) & 0xFF);
    SPI.transfer(addr & 0xFF);
}

// All transactions mask interrupts: the control ISR owns the SPI bus (see
// HAL.h). A 64-byte transfer at 12 MHz is ~45 us -- small vs the 2 ms tick.

uint32_t hal_fram_read_id() {
    noInterrupts();
    SPI.beginTransaction(FRAM_SPI_SETTINGS);
    digitalWrite(PIN_FRAM_CS, LOW);
    SPI.transfer(FRAM_OP_RDID);
    uint32_t id = 0;
    for (int i = 0; i < 4; i++) {
        id = (id << 8) | SPI.transfer(0x00);
    }
    digitalWrite(PIN_FRAM_CS, HIGH);
    SPI.endTransaction();
    interrupts();
    return id;
}

void hal_fram_read_id_bytes(uint8_t *out, int n) {
    noInterrupts();
    SPI.beginTransaction(FRAM_SPI_SETTINGS);
    digitalWrite(PIN_FRAM_CS, LOW);
    SPI.transfer(FRAM_OP_RDID);
    for (int i = 0; i < n; i++) out[i] = SPI.transfer(0x00);
    digitalWrite(PIN_FRAM_CS, HIGH);
    SPI.endTransaction();
    interrupts();
}

void hal_fram_read(uint32_t addr, uint8_t *buf, size_t len) {
    noInterrupts();
    SPI.beginTransaction(FRAM_SPI_SETTINGS);
    digitalWrite(PIN_FRAM_CS, LOW);
    SPI.transfer(FRAM_OP_READ);
    fram_send_addr(addr);
    for (size_t i = 0; i < len; i++) {
        buf[i] = SPI.transfer(0x00);
    }
    digitalWrite(PIN_FRAM_CS, HIGH);
    SPI.endTransaction();
    interrupts();
}

void hal_fram_write(uint32_t addr, const uint8_t *buf, size_t len) {
    noInterrupts();
    SPI.beginTransaction(FRAM_SPI_SETTINGS);

    digitalWrite(PIN_FRAM_CS, LOW);  // WREN needs its own CS frame
    SPI.transfer(FRAM_OP_WREN);
    digitalWrite(PIN_FRAM_CS, HIGH);

    digitalWrite(PIN_FRAM_CS, LOW);
    SPI.transfer(FRAM_OP_WRITE);
    fram_send_addr(addr);
    for (size_t i = 0; i < len; i++) {
        SPI.transfer(buf[i]);
    }
    digitalWrite(PIN_FRAM_CS, HIGH);

    SPI.endTransaction();
    interrupts();
}

// -- Motor driver (TB6612FNG) ----------------------------------------------------
// Pins from spec section 5.4. Both PWM pins are FlexPWM-capable on the
// Teensy 4.0, so 20 kHz / 10-bit costs nothing.

static constexpr uint8_t PIN_PWMA = 9;
static constexpr uint8_t PIN_AIN1 = 15;
static constexpr uint8_t PIN_AIN2 = 14;
static constexpr uint8_t PIN_PWMB = 19;
static constexpr uint8_t PIN_BIN1 = 17;
static constexpr uint8_t PIN_BIN2 = 18;
static constexpr uint8_t PIN_STBY = 16;

static constexpr float MOTOR_PWM_HZ = 20000.0f;
static constexpr int MOTOR_PWM_MAX = 1023;  // 10-bit

void hal_motors_init() {
    const uint8_t pins[] = {PIN_PWMA, PIN_AIN1, PIN_AIN2,
                            PIN_PWMB, PIN_BIN1, PIN_BIN2, PIN_STBY};
    for (uint8_t p : pins) {
        pinMode(p, OUTPUT);
        digitalWrite(p, LOW);  // coast, standby asserted (STBY low = off)
    }
    analogWriteResolution(10);  // global for all analogWrite pins
    analogWriteFrequency(PIN_PWMA, MOTOR_PWM_HZ);
    analogWriteFrequency(PIN_PWMB, MOTOR_PWM_HZ);
}

void hal_motor_standby(bool standby) {
    digitalWrite(PIN_STBY, standby ? LOW : HIGH);  // STBY is active-low
}

void hal_motor_set(uint8_t channel, float duty) {
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;

    uint8_t in1 = (channel == 0) ? PIN_AIN1 : PIN_BIN1;
    uint8_t in2 = (channel == 0) ? PIN_AIN2 : PIN_BIN2;
    uint8_t pwm = (channel == 0) ? PIN_PWMA : PIN_PWMB;

    if (duty > 0.0f) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (duty < 0.0f) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);  // both low = coast
        digitalWrite(in2, LOW);
    }
    float mag = duty < 0.0f ? -duty : duty;
    analogWrite(pwm, static_cast<int>(mag * MOTOR_PWM_MAX + 0.5f));
}

// -- ESP32 radio link (Serial5) --------------------------------------------------

static constexpr uint32_t RADIO_BAUD = 460800;  // spec section 7

void hal_uart_radio_init() {
#if HAL_RADIO_ENABLED
    // Pull RX5 up BEFORE begin(): an unpowered or absent module leaves the
    // line floating, and a floating UART input decodes noise into
    // characters that the console would happily execute. Idle-high is the
    // UART's own resting state, so a pull-up simply asserts "no traffic".
    pinMode(21, INPUT_PULLUP);
    Serial5.begin(RADIO_BAUD);
#endif
}

// RADIO TX RING. Serial5.write() blocks once the UART's own TX buffer fills,
// and at 460800 baud a chatty moment would hold the main loop for
// milliseconds while a control loop is flying the mouse -- so this layer must
// never block. The first version simply DISCARDED whatever did not fit, which
// is fine for a telemetry line nobody reads and quietly destructive for
// anything else: the Teensy TX buffer is a few dozen bytes, so a console line
// longer than that lost its tail. The scale test lost the value it exists to
// print ("believed L 906.7  R 90" -- and the whole correction line after it).
//
// A ring in front of the UART keeps the non-blocking property and stops
// truncating: writes land in the ring, hal_radio_poll() drains whatever the
// UART has room for each main-loop pass. Only a genuine overrun -- more
// output queued than 1 KB, faster than 46 kB/s can leave -- drops anything,
// and then it drops the NEW bytes rather than corrupting a line already
// half-sent. Bulk dumps still belong on USB.
static uint8_t radio_tx[1024];
static volatile uint16_t radio_head = 0, radio_tail = 0;
static uint32_t radio_dropped = 0;

static inline uint16_t radio_tx_used() {
    return uint16_t((radio_head - radio_tail) & (sizeof(radio_tx) - 1));
}

size_t hal_radio_write(const uint8_t *buf, size_t len) {
#if HAL_RADIO_ENABLED
    size_t took = 0;
    for (size_t i = 0; i < len; i++) {
        uint16_t nxt = uint16_t((radio_head + 1) & (sizeof(radio_tx) - 1));
        if (nxt == radio_tail) {       // full
            radio_dropped++;
            break;
        }
        radio_tx[radio_head] = buf[i];
        radio_head = nxt;
        took++;
    }
    hal_radio_poll();
    return took;
#else
    (void)buf;
    return len;  // pretend success: the console must not care
#endif
}

void hal_radio_poll() {
#if HAL_RADIO_ENABLED
    int room = Serial5.availableForWrite();
    while (room > 0 && radio_tail != radio_head) {
        Serial5.write(radio_tx[radio_tail]);
        radio_tail = uint16_t((radio_tail + 1) & (sizeof(radio_tx) - 1));
        room--;
    }
#endif
}

uint32_t hal_radio_dropped() { return radio_dropped; }

void hal_radio_wait_room(uint16_t bytes, uint32_t timeout_ms) {
#if HAL_RADIO_ENABLED
    if (bytes >= sizeof(radio_tx) - 1) bytes = sizeof(radio_tx) - 2;
    uint32_t t0 = millis();
    while (sizeof(radio_tx) - 1 - radio_tx_used() < bytes) {
        hal_radio_poll();
        if (millis() - t0 > timeout_ms) return;
        delayMicroseconds(200);
    }
#else
    (void)bytes;
    (void)timeout_ms;
#endif
}

int hal_radio_read() {
#if HAL_RADIO_ENABLED
    return Serial5.available() ? Serial5.read() : -1;
#else
    return -1;
#endif
}

// -- Sensor sub-processor link (Serial2) -----------------------------------------
// Serial2 (pins 7=RX2, 8=TX2) to the sensor sub-MCU's Serial5. The sub-MCU
// firmware (TW3_sensor_sub_mcu.ino) runs this link at 460800 -- NOT the 115200
// in spec section 5.2; the sketch is the ground truth. Byte-pipe only; command
// bytes and the 5-byte wall packets are the Drivers layer's business (Sensors.h).

static constexpr uint32_t SENSOR_BAUD = 460800;  // matches the sub-MCU sketch

void hal_uart_sensor_init() {
    Serial2.begin(SENSOR_BAUD);
}

size_t hal_sensor_write(const uint8_t *buf, size_t len) {
    return Serial2.write(buf, len);
}

int hal_sensor_read() {
    return Serial2.available() ? Serial2.read() : -1;
}

// -- Quadrature encoders -------------------------------------------------------
// Hardware ENC modules on the i.MX RT1062 via the Teensyduino QuadEncoder
// library: x4 quadrature decoding in silicon, zero interrupts. Pins from
// spec section 5.5; all four are XBAR-routable (required by the ENC blocks).

#include "QuadEncoder.h"

static QuadEncoder enc_left(1, 0, 1);   // ENC channel 1: A = pin 0, B = pin 1
static QuadEncoder enc_right(2, 2, 3);  // ENC channel 2: A = pin 2, B = pin 3

void hal_encoders_init() {
    enc_left.setInitConfig();
    enc_left.init();
    enc_right.setInitConfig();
    enc_right.init();
}

int32_t hal_encoder_left() { return enc_left.read(); }
int32_t hal_encoder_right() { return enc_right.read(); }

void hal_encoders_zero() {
    enc_left.write(0);
    enc_right.write(0);
}

// -- Control timer -------------------------------------------------------------

static IntervalTimer control_timer;

bool hal_control_timer_begin(void (*isr)(), float hz) {
    return control_timer.begin(isr, 1000000.0f / hz);  // period in µs
}
