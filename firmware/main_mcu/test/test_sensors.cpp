// Host-side test: wall-sensor driver packet parser (lib/Drivers/Sensors.*).
// Mocks the HAL sensor UART (a byte queue in / a TX log out) and millis(), so
// the 5-byte framing, resync-after-drop, 255 handling, and trigger emission
// can be exercised without hardware.
//
// Build:
//   g++ -std=c++17 -Istubs -I../lib/Drivers -I../lib/HAL ../lib/Drivers/Sensors.cpp test_sensors.cpp -o test_sensors && ./test_sensors

#include "Sensors.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>

// -- Arduino stub definition --------------------------------------------------
static unsigned long g_ms = 0;
unsigned long millis() { return g_ms; }

// -- Mock HAL sensor UART -----------------------------------------------------
static std::vector<uint8_t> rx_queue;
static size_t rx_pos = 0;
static std::vector<uint8_t> tx_log;

void hal_uart_sensor_init() {}
size_t hal_sensor_write(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) tx_log.push_back(buf[i]);
    return len;
}
int hal_sensor_read() {
    return rx_pos < rx_queue.size() ? rx_queue[rx_pos++] : -1;
}

static void feed(std::initializer_list<int> bytes) {
    for (int b : bytes) rx_queue.push_back((uint8_t)b);
}

static int failures = 0;
static void expect(const char *what, bool cond) {
    if (!cond) { std::printf("FAIL: %s\n", what); failures++; }
}

int main() {
    sensors_init();

    // -- Clean packet parses; poll sends nothing -------------------------------
    tx_log.clear();
    feed({30, 255, 40, 50, 0x0D});
    sensors_poll();
    WallRanges w = sensors_latest();
    expect("left parsed", w.left_mm == 30);
    // Front fields are swapped vs the raw packet (beam crossover): packet[1]=255
    // -> front_left, packet[2]=40 -> front_right.
    expect("front_left parsed (255 OOR)", w.front_left_mm == 255);
    expect("front_right parsed", w.front_right_mm == 40);
    expect("right parsed", w.right_mm == 50);
    expect("seq incremented", w.seq == 1);
    expect("poll sends nothing", tx_log.empty());
    expect("recent packet -> ready", sensors_ready());

    // -- Command senders emit the right bytes ----------------------------------
    tx_log.clear();
    sensors_free_run();
    expect("free_run emits 0x56", tx_log.size() == 1 && tx_log[0] == SENSOR_CMD_FREE_RUN);
    tx_log.clear();
    sensors_stop();
    expect("stop emits 0x57", tx_log.size() == 1 && tx_log[0] == SENSOR_CMD_STOP);
    tx_log.clear();
    sensors_trigger();
    expect("trigger emits 0x55", tx_log.size() == 1 && tx_log[0] == SENSOR_CMD_TRIGGER);

    // -- Two packets in one drain: latest = second, seq += 2 -------------------
    feed({10, 11, 12, 13, 0x0D, 20, 21, 22, 23, 0x0D});
    sensors_poll();
    w = sensors_latest();
    expect("latest is second packet", w.left_mm == 20 && w.right_mm == 23);
    expect("seq += 2", w.seq == 3);

    // -- Resync: a stray leading byte, then a good packet ----------------------
    feed({0x99, 60, 61, 62, 63, 0x0D});
    sensors_poll();
    w = sensors_latest();
    expect("resynced after stray byte", w.left_mm == 60 && w.right_mm == 63);
    expect("seq after resync", w.seq == 4);

    // -- Partial packet holds until completed across calls ---------------------
    uint32_t seq_before = sensors_latest().seq;
    feed({70, 71, 72});      // only 3 bytes so far
    sensors_poll();
    expect("partial packet not committed", sensors_latest().seq == seq_before);
    feed({73, 0x0D});        // completes it
    sensors_poll();
    w = sensors_latest();
    expect("completed across calls", w.left_mm == 70 && w.right_mm == 73);
    expect("seq after completion", w.seq == seq_before + 1);

    // -- ms_since_packet tracks millis since last commit -----------------------
    g_ms = 1000;
    feed({1, 2, 3, 4, 0x0D});
    sensors_poll();          // commits at t=1000
    g_ms = 1075;
    expect("ms_since_packet", sensors_ms_since_packet() == 75);

    if (failures == 0) { std::printf("ALL SENSOR TESTS PASSED\n"); return 0; }
    std::printf("%d SENSOR TEST(S) FAILED\n", failures);
    return 1;
}
