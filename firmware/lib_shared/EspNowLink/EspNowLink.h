// EspNowLink -- transparent byte pipe over ESP-NOW for the ESP32-C3 Super Mini.
//
// Both the mouse-side and laptop-side radios run identical logic: whatever
// bytes arrive on the "host" side (UART for the mouse, USB serial for the
// laptop) get relayed to the single ESP-NOW peer, and whatever arrives over
// ESP-NOW is queued for the host side to read back out. No framing, no
// interpretation -- the framing/meaning is the main MCU's business (see
// firmware/main_mcu, spec sections 5.3 and 7).
//
// Wiring is symmetric: mouse peers with LAPTOP_MAC, laptop peers with
// MOUSE_MAC. Both must sit on the same Wi-Fi channel (ESPNOW_CHANNEL).

#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// Shared Wi-Fi channel for the ESP-NOW peers. Both radios MUST match.
#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 1
#endif

namespace EspNowLink {

// Bring up Wi-Fi (STA, fixed channel, power-save off) and ESP-NOW, then add
// `peer` as the sole unicast peer. Returns true on success. Safe to print the
// local MAC afterwards via ownMac() to verify the hard-coded addresses.
bool begin(const uint8_t peer[6]);

// Relay `len` bytes to the peer, transparently split into <=250-byte ESP-NOW
// packets. Returns the number of bytes accepted (== len unless a send failed).
size_t send(const uint8_t *buf, size_t len);

// Bytes received over ESP-NOW and waiting to be drained to the host side.
size_t available();

// Next received byte, or -1 if none pending.
int read();

// Local STA MAC (6 bytes), for verifying the peer constants during bring-up.
void ownMac(uint8_t out[6]);

}  // namespace EspNowLink
