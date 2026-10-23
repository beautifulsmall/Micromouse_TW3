#include "EspNowLink.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

namespace {

constexpr size_t ESPNOW_MAX_PAYLOAD = 250;  // ESP-NOW hard limit per packet

uint8_t s_peer[6];

// Single-producer (ESP-NOW recv callback, Wi-Fi task) / single-consumer
// (loop) lock-free ring buffer. Power-of-two size so wraps are a mask.
constexpr size_t RX_RING = 4096;
static_assert((RX_RING & (RX_RING - 1)) == 0, "RX_RING must be a power of two");
volatile uint8_t s_ring[RX_RING];
volatile size_t s_head = 0;  // written by callback
volatile size_t s_tail = 0;  // read by loop

inline void ring_push(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        size_t next = (s_head + 1) & (RX_RING - 1);
        if (next == s_tail) return;  // full: drop the overflow, never block
        s_ring[s_head] = data[i];
        s_head = next;
    }
}

// Arduino-ESP32 core 3.x passes a recv-info struct; 2.x passed the raw MAC.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t * /*info*/, const uint8_t *data, int len) {
#else
void onRecv(const uint8_t * /*mac*/, const uint8_t *data, int len) {
#endif
    if (len > 0) ring_push(data, static_cast<size_t>(len));
}

}  // namespace

namespace EspNowLink {

bool begin(const uint8_t peer[6]) {
    memcpy(s_peer, peer, 6);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();  // no AP association; ESP-NOW only

    // Pin the channel and kill power-save so the link stays low-latency and
    // both radios are guaranteed to meet on the same channel.
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Cap TX power to 12 dBm (~half the C3's ~20 dBm max). Argument is in
    // 0.25 dBm units, so 12 dBm = 48. Must run after Wi-Fi has started.
    esp_wifi_set_max_tx_power(48);

    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_recv_cb(onRecv);

    esp_now_peer_info_t info = {};
    memcpy(info.peer_addr, s_peer, 6);
    info.channel = ESPNOW_CHANNEL;
    info.encrypt = false;
    if (esp_now_add_peer(&info) != ESP_OK) return false;

    return true;
}

size_t send(const uint8_t *buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        size_t chunk = len - sent;
        if (chunk > ESPNOW_MAX_PAYLOAD) chunk = ESPNOW_MAX_PAYLOAD;
        if (esp_now_send(s_peer, buf + sent, chunk) != ESP_OK) break;
        sent += chunk;
    }
    return sent;
}

size_t available() {
    return (s_head - s_tail) & (RX_RING - 1);
}

int read() {
    if (s_tail == s_head) return -1;
    uint8_t b = s_ring[s_tail];
    s_tail = (s_tail + 1) & (RX_RING - 1);
    return b;
}

void ownMac(uint8_t out[6]) {
    WiFi.macAddress(out);
}

}  // namespace EspNowLink
