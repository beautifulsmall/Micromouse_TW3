// Sensor LUT store + boot uploader (auto-cal, bert 2026-08-08).
//
// The four wall-sensor pixel->mm lookup tables live as compiled arrays on
// the sub-MCU. The self-cal front sweep ('C') generates fresh tables from
// odometry ground truth; rather than reflash the sub-MCU, we keep the
// generated tables in the MAIN-MCU FRAM (the source of truth) and stream
// them to the sub-MCU over Serial5 at every boot, BEFORE hand-start / IMU
// cal. If no valid block is stored, the sub-MCU keeps its compiled
// defaults (upload is a no-op) -- so the mechanism is inert until 'W'
// deliberately commits a swept table.
//
// FRAM layout (SENSOR_LUT_FRAM_ADDR, see Fram.h map):
//   +0x00  magic  u32  'SLUT'
//   +0x04  version u8
//   +0x05  valid   u8   bitmask: bit0 left, bit1 front_r, bit2 front_l, bit3 right
//   +0x06  crc16   u16  over the 4x256 data region
//   +0x08  lut[4][256]  channel order matches the sub-MCU (0=left, 1=front_r,
//                       2=front_l, 3=right)
//
// NOTE (convention): the 'C' front tables are CENTRE-referenced (45 mm at
// contact). Once uploaded, front_mm becomes centre->wall everywhere it is
// consumed -- the front-stop threshold and the beam observer must expect
// that reference. Adopt deliberately.

#pragma once

#include <Arduino.h>

constexpr uint32_t SENSOR_LUT_FRAM_ADDR = 0x00200;
constexpr uint32_t SENSOR_LUT_MAGIC = 0x53554C54UL;  // 'SLUT'
constexpr uint8_t  SENSOR_LUT_VERSION = 1;
constexpr int      SENSOR_LUT_CHANNELS = 4;

// Store one channel's 256-entry table to FRAM and mark it valid (creates
// the block if absent). ch: 0=left 1=front_r 2=front_l 3=right.
void sensor_lut_store(uint8_t ch, const uint8_t lut[256]);

// Fetch one channel; returns false if the block is absent or ch not valid.
bool sensor_lut_get(uint8_t ch, uint8_t lut[256]);

// True if a valid block with at least one valid channel is present.
bool sensor_lut_present();

// Bitmask of valid channels (0 if none / block absent).
uint8_t sensor_lut_valid_mask();

// Invalidate the stored block -> next boot uses the sub-MCU's compiled
// defaults (does NOT restore the sub-MCU RAM until a power cycle).
void sensor_lut_clear();

// Pull every channel's LIVE LUT from the sub-MCU (e.g. GUI-calibrated in RAM)
// and store it into FRAM, so it persists and re-uploads at boot. Call from
// the bench with the sub-MCU idle (streaming stopped). Returns channels
// pulled, or -1 if the sub-MCU didn't answer.
int sensor_lut_pull_all();

// Ping the sub-MCU, then stream every valid channel to it over Serial5.
// Call once at boot after sensors_init(), before arming. Returns the
// number of channels uploaded, or -1 if the sub-MCU did not answer the
// ping (upload skipped; compiled defaults remain in force).
int sensor_lut_upload_all();
