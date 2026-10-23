// Mouse-side ESP32-C3 Super Mini -- transparent UART <-> ESP-NOW bridge.
//
// Host side: UART1 wired to the Teensy Main MCU's Serial5 (spec section 5.3).
//   ESP32 GPIO21 = TX  -> Teensy pin 21 (RX5)
//   ESP32 GPIO20 = RX  <- Teensy pin 20 (TX5)
//   460800 baud, 8N1.
// Radio side: ESP-NOW unicast to the laptop-side radio (LAPTOP_MAC).
//
// Everything is a raw byte pipe -- no framing or interpretation. USB Serial is
// used only for a one-time bring-up banner (prints our own MAC so MOUSE_MAC on
// the laptop side can be verified).

#include <Arduino.h>

#include "EspNowLink.h"

// Peer: the laptop-side ESP32's STA MAC.
uint8_t LAPTOP_MAC[6] = { 0x14, 0x63, 0x93, 0xB7, 0xA3, 0xDC };

// UART1 <-> Teensy Serial5.
static constexpr int      UART_TX_PIN = 21;
static constexpr int      UART_RX_PIN = 20;
static constexpr uint32_t UART_BAUD   = 460800;
HardwareSerial UartLink(1);

static void printMac(const char *label, const uint8_t mac[6]) {
    Serial.printf("%s %02X:%02X:%02X:%02X:%02X:%02X\n", label,
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void setup() {
    Serial.begin(115200);  // USB CDC: bring-up banner only
    UartLink.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    bool ok = EspNowLink::begin(LAPTOP_MAC);

    uint8_t self[6];
    EspNowLink::ownMac(self);
    printMac("[mouse radio] own MAC :", self);
    printMac("[mouse radio] peer MAC:", LAPTOP_MAC);
    Serial.println(ok ? "[mouse radio] ESP-NOW ready" : "[mouse radio] ESP-NOW init FAILED");
}

void loop() {
    // Host UART -> ESP-NOW, batched into ESP-NOW-sized chunks for efficiency.
    static uint8_t buf[250];
    size_t n = 0;
    while (n < sizeof(buf) && UartLink.available()) {
        buf[n++] = static_cast<uint8_t>(UartLink.read());
    }
    if (n) EspNowLink::send(buf, n);

    // ESP-NOW -> host UART.
    while (EspNowLink::available()) {
        int b = EspNowLink::read();
        if (b < 0) break;
        UartLink.write(static_cast<uint8_t>(b));
    }
}
