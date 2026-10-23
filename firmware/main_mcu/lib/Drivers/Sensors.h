// Wall-sensor sub-processor driver.
//
// Talks to the second Teensy (TW3_sensor_sub_mcu.ino) over the HAL sensor UART
// (Serial2 @ 460800). That MCU drives four S15366-256 laser triangulation
// sensors and, on a trigger byte, replies with one 5-byte packet:
//
//   [0] left_mm  [1] front_right_mm  [2] front_left_mm  [3] right_mm  [4] 0x0D
//
// Each range is a uint8 in mm, 255 = out of range. Side beams CROSS -- the
// right-mounted sensor reads the LEFT wall and vice-versa -- so the packet is
// already named by the wall measured.
//
// IMPORTANT: these are the DIRECT range ALONG each beam (calibrated on the
// sub-MCU), NOT the perpendicular distance to the wall. Converting to wall
// distance / lateral offset / heading (using the fixed laser mount geometry
// fused with IMU yaw) is a later Control-layer job; this driver only moves
// packets.
//
// Usage is non-blocking and free-run based: sensors_free_run() tells the
// sub-MCU to stream packets continuously (~440 Hz), and sensors_poll() -- called
// often -- drains the UART FIFO and keeps the most recent packet. The mouse
// never waits on the sub-MCU; it just reads the latest data available.
//
// (Free-run is used because single-shot triggering leaves the S15366 output
// registers cold -- reads come back invalid. Free-run keeps them warm. Bench
// testing (2026-07-19) disproved the old <= 5 s thermal cap: free-run may
// run indefinitely. Low-rate triggering during long search phases is now
// just laser-duty hygiene, not a requirement.)

#pragma once

#include <Arduino.h>

// Command bytes to the sub-MCU (match TW3_sensor_sub_mcu.ino).
constexpr uint8_t SENSOR_CMD_TRIGGER = 0x55;  // one measurement -> one packet
constexpr uint8_t SENSOR_CMD_FREE_RUN = 0x56;  // continuous (no time limit)
constexpr uint8_t SENSOR_CMD_STOP = 0x57;      // stop free-run
// Interlaced free-run (test mode, 2026-07-19): fire L, FR, R, FL in a loop
// and read each IC one slot late, inside the next slot's integration -- no
// primer double-fires. ~580 Hz at 400 us integration, ~4x lower laser duty
// than classic free-run. Same packet format.
constexpr uint8_t SENSOR_CMD_FREE_RUN_IL = 0x58;
// Raw-px free-run (auto-cal, 2026-08-06): classic free-run but each packet
// is 13 bytes -- the 4 mm bytes plus raw px + signal per channel -- so the
// main MCU can pair pixels with odometry during a calibration sweep.
constexpr uint8_t SENSOR_CMD_FREE_RUN_RAW = 0x59;
// LUT upload (auto-cal, 2026-08-08): main -> sub, followed by [ch][256 mm
// bytes][sum8]. The sub writes the channel's LUT and replies with an ACK
// (SENSOR_LUT_ACK ok / SENSOR_LUT_NAK bad checksum). Sent at boot from the
// main-MCU FRAM store; the sub keeps compiled defaults if none arrive.
constexpr uint8_t SENSOR_CMD_LOAD_LUT = 0x5A;
constexpr uint8_t SENSOR_CMD_PING = 0x5B;   // sub replies with the same byte
constexpr uint8_t SENSOR_CMD_DUMP_LUT = 0x5C;  // pull a live LUT channel back
constexpr uint8_t SENSOR_LUT_ACK = 0xA5;
constexpr uint8_t SENSOR_LUT_NAK = 0xE5;

constexpr uint8_t SENSOR_OUT_OF_RANGE = 255;

// Most recent beam ranges (mm along the beam; 255 = out of range). The two
// front fields are SWAPPED relative to the raw packet: the front beams cross
// ~100 mm out, so beyond the crossover (wall distances) packet[1] reads the
// left-front and packet[2] the right-front -- the driver assigns them to the
// correct side here (see commit_packet).
struct WallRanges {
    uint8_t left_mm;         // packet[0] -- right-mounted sensor, LEFT wall
    uint8_t front_right_mm;  // packet[2] (front beams cross ~100 mm; swapped)
    uint8_t front_left_mm;   // packet[1] (front beams cross ~100 mm; swapped)
    uint8_t right_mm;        // packet[3] -- left-mounted sensor, RIGHT wall
    uint32_t seq;            // ++ per fresh packet (freshness / drop detection)
};

// Raw per-channel sensor data (auto-cal only, populated in raw-px mode).
// WIRE order: 0=left, 1=front_right, 2=front_left, 3=right (NOT the
// fr/fl-swapped WallRanges order) -- px[i] is the pixel of the channel
// that produced it. px = spot position (Maxpos_X_int, the LUT index),
// sig = peak signal (Maxval_X). 255/0 as usual.
struct WallRaw {
    uint8_t px[4];
    uint8_t sig[4];
    uint32_t seq;
};

void sensors_init();

// Tell the sub-MCU to stream continuously (CMD_FREE_RUN). Call once to start.
void sensors_free_run();

// Interlaced continuous streaming (CMD_FREE_RUN_IL) -- see the constant.
void sensors_free_run_interlaced();

// Raw-px calibration streaming (CMD_FREE_RUN_RAW): 13-byte packets with
// raw px + signal per channel. Pair with sensors_latest_raw().
void sensors_free_run_raw();

// Stop free-run and turn the lasers off (CMD_STOP). Call when done streaming.
void sensors_stop();

// Request a single measurement (CMD_TRIGGER) -- for a low-duty search phase.
void sensors_trigger();

// Non-blocking: drain the UART FIFO and parse any complete packets, keeping the
// most recent. Call frequently (e.g. every main-loop iteration) so the FIFO
// doesn't overflow between reads. Sends nothing.
void sensors_poll();

// Snapshot of the latest parsed ranges.
WallRanges sensors_latest();

// Snapshot of the latest raw px/signal (valid only in raw-px mode).
WallRaw sensors_latest_raw();

// Probe the sub-MCU: send CMD_PING and wait up to timeout_ms for the echo.
// True if the sub-MCU answered (it is up and draining Serial5). Blocking;
// boot-time use only.
bool sensors_ping(uint32_t timeout_ms);

// Upload one 256-entry LUT channel to the sub-MCU (CMD_LOAD_LUT + ch + 256
// mm bytes + sum8), then wait for the ACK. True on ACK. Blocking; call
// only at boot / when not streaming. ch: 0=left 1=front_r 2=front_l 3=right.
bool sensors_upload_lut(uint8_t ch, const uint8_t lut[256]);

// Pull one channel's LIVE LUT back from the sub-MCU (CMD_DUMP_LUT + ch ->
// 256 mm bytes + sum8) into out[256]. Lets the main persist a GUI-calibrated
// LUT into its FRAM. True on a valid checksummed reply. Blocking; boot/bench
// use only (the sub must be in triggered mode -- stop any stream first).
bool sensors_download_lut(uint8_t ch, uint8_t out[256]);

// ---- Boot threshold check (bert, 2026-08-15) -------------------------------
// The sub-MCU dark-reads every channel at power-on -- lasers off, so it is
// measuring the ROOM -- and checks the two SIDE detection thresholds against
// what it found. It REPORTS; it does not set anything. (A version that set the
// thresholds to the measured floor + 20% was built and reverted the same day;
// the side thresholds are hand values, 22 as of 2026-08-15.)
//
// This pulls the result back, because the sub-MCU's own USB console is
// invisible on a real run: the mouse is always powered off before its flight
// log is read. Anything worth knowing has to come to this MCU.
//
// Reading the numbers: `dark` is the noise floor per channel and `thresh` is
// what the sensor rejects below. The margin between them is the whole point. A
// GOOD side return is only about 24 counts with the current optics, so there is
// not much room between noise and signal to begin with -- a floor anywhere near
// 20 means the room is too bright for these lenses.
//
// The two warnings are not symmetric, because the failures are not:
//   THIN  the threshold is under 20% clear of the floor. Marginal returns are
//         being thrown away; readings go to 255 and the horizon stands down.
//         Loud, and self-announcing.
//   UNDER the threshold is AT OR BELOW the floor. The peak detector latches on
//         noise and the LUT converts a noise pixel into a confident distance.
//         Nothing downstream catches it -- walls_beam_hit trusts any value
//         under 255 -- and with the horizon active the mouse steers at it.
//         Silent, and therefore the one that gets the louder warning.
//
// `vmin` is the firmware's second, separate amplitude gate (a spot is trusted
// only if Maxval_X >= vmin). It sits below the sensor threshold, so it is a
// backstop rather than the binding constraint.
struct SensorThresholds {
    bool    valid;         // a checksummed reply arrived
    uint8_t dark[4];       // measured noise floor, per channel
    uint8_t thresh[4];     // sensor MAX_THRESHOLD in force, per channel
    uint8_t vmin[4];       // firmware validity gate, per channel
    uint8_t flags;         // SENSOR_THRESH_F_*
};
constexpr uint8_t SENSOR_CMD_THRESH_REP = 0x5D;
constexpr int SENSOR_THRESH_REP_BYTES = 14;
constexpr uint8_t SENSOR_THRESH_F_RAN = 0x01;    // the check completed
constexpr uint8_t SENSOR_THRESH_F_THIN = 0x02;   // side margin under 20%
constexpr uint8_t SENSOR_THRESH_F_UNDER = 0x04;  // side threshold <= the floor

// Blocking; the sub must be in triggered mode (stop any stream first).
// rerun = true re-measures before replying, which is what to use when the
// lighting has changed since power-on.
bool sensors_threshold_report(SensorThresholds *out, bool rerun,
                              uint32_t timeout_ms);

// Milliseconds since the last complete packet (large => sub-MCU silent).
uint32_t sensors_ms_since_packet();

// True when a complete packet arrived recently (sub-MCU alive and in frame).
// False if the link has gone silent -- maze logic should not trust stale ranges.
bool sensors_ready();
