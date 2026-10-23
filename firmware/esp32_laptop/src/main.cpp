// Laptop-side ESP32-C3 Super Mini -- transparent USB-serial <-> ESP-NOW bridge.
//
// Host side: native USB CDC (Serial) to the laptop's serial monitor.
// Radio side: ESP-NOW unicast to the mouse-side radio (MOUSE_MAC).
//
// This is the off-robot half of the byte pipe. Plug it into the laptop, open a
// serial monitor at 460800 baud, and you are talking to the Teensy Main MCU's
// dual console exactly as if wired directly (spec sections 5.3 and 7).
//
// A one-line bring-up banner is printed to the USB console at boot (before any
// link traffic) so the mouse-side MOUSE_MAC constant can be verified.

#include <Arduino.h>

#include "EspNowLink.h"

// Peer: the mouse-side ESP32's STA MAC.
uint8_t MOUSE_MAC[6] = { 0x14, 0x63, 0x93, 0xB7, 0xD7, 0x18 };

static void printMac(const char *label, const uint8_t mac[6]) {
    Serial.printf("%s %02X:%02X:%02X:%02X:%02X:%02X\n", label,
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void setup() {
    Serial.begin(460800);  // native USB CDC; baud nominal but matches the chain

    bool ok = EspNowLink::begin(MOUSE_MAC);

    uint8_t self[6];
    EspNowLink::ownMac(self);
    printMac("[laptop radio] own MAC :", self);
    printMac("[laptop radio] peer MAC:", MOUSE_MAC);
    Serial.println(ok ? "[laptop radio] ESP-NOW ready -- pass-through live"
                      : "[laptop radio] ESP-NOW init FAILED");
}

void loop() {
    // Host USB -> ESP-NOW, batched into ESP-NOW-sized chunks.
    static uint8_t buf[250];
    size_t n = 0;
    while (n < sizeof(buf) && Serial.available()) {
        buf[n++] = static_cast<uint8_t>(Serial.read());
    }
    if (n) EspNowLink::send(buf, n);

    // ESP-NOW -> host USB.
    while (EspNowLink::available()) {
        int b = EspNowLink::read();
        if (b < 0) break;
        Serial.write(static_cast<uint8_t>(b));
    }
}
