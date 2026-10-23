// =============================================================================
// TW3_sensor_sub_mcu.ino — S15366-256 wall sensor firmware, Teensy 4.0
// Based on TW3_S15366_sensors_base/TW3_S15366_sensors_V5_good.ino
//
// Serial5 (TX pin 20, RX pin 21) baud: 460800 — must match SENSOR_BAUD in main MCU config.h
//
// Two operating modes, selected by command byte from the main MCU:
//
//   0x55  CMD_TRIGGER  — single measurement cycle → one 5-byte packet.
//                        For search phase. Main MCU sends at a reduced rate
//                        (e.g. every 4th loop = 100 Hz) to keep per-laser duty
//                        low (~5%) during the longer search phase.
//
//   0x56  CMD_FREE_RUN — continuous measurement: cycle all four lasers as fast
//                        as possible (~440 Hz update rate, ~87% per-laser duty).
//                        For fast run. (A ≤5 s thermal cap was assumed
//                        here for a long time; bench testing 2026-07-19
//                        showed it is NOT needed -- free-run may run
//                        indefinitely.)
//
//   0x57  CMD_STOP     — exit free-run, return to triggered mode.
//
//   0x5D  CMD_THRESH_REP — report the boot threshold check (+1 byte: non-
//                        zero re-measures first). Reply is 14 bytes:
//                        [4 dark floor][4 threshold][4 valid_min][flags][sum8].
//                        Triggered mode only. Exists because this MCU's USB
//                        console is invisible on a real run -- the mouse is
//                        always powered off before its flight log is read.
//
// Packet format (5 bytes):
// Side beams cross — each side sensor reads the OPPOSITE wall, so byte [0]
// (IC0, right-mounted) is the LEFT wall and byte [3] (IC2, left-mounted) is
// the RIGHT wall. Byte order is unchanged (measurement order); only the
// wall each byte represents is named by wall measured.
//   [0] left_mm      uint8   255 = out of range   (IC0, right-mounted)
//   [1] front_right  uint8   255 = out of range
//   [2] front_left   uint8   255 = out of range
//   [3] right_mm     uint8   255 = out of range   (IC2, left-mounted)
//   [4] 0x0D         terminator
// =============================================================================

#include <SPI.h>

// --- Pin definitions ---
#define laser_1     22  //22
#define laser_2      3    //3
#define laser_3      4     //4
#define laser_4     19   //19

#define SPI_CS_1     2
#define SPI_CS_2     5
#define SPI_CS_3     9
#define MST_1        1
#define MST_2        7
#define MST_3        8
#define TG_RESET    14   // dedicated GPIO — serial is on pins 20/21
#define SPI_RESET   15   // dedicated GPIO — serial is on pins 20/21
#define MISO_MUX_1   0
#define MISO_MUX_2   6
#define MISO_MUX_3  10

// --- S15366 register addresses (pre-shifted left by 1 in hardware) ---
#define REG_ADDR_MODE                        0
#define REG_ADDR_INTEGRATION_TIME_HIGH_BYTE  2
#define REG_ADDR_INTEGRATION_TIME_LOW_BYTE   4
#define REG_ADDR_BLANKING                    6
#define REG_ADDR_PIXEL_READOUT_START_Y       8
#define REG_ADDR_PIXEL_READOUT_START_X      10
#define REG_ADDR_PIXEL_READOUT_COUNT        12
#define REG_ADDR_ADC                        14
#define REG_ADDR_GAIN                       16
#define REG_ADDR_CALC_AREA                  20
#define REG_ADDR_MAX_THRESHOLD              22
#define REG_ADDR_FRAME_COUNT                24
#define REG_ADDR_MAXPOS_X_INT               26
#define REG_ADDR_MAXPOS_X_DEC              28
#define REG_ADDR_MAXPOS_Y_INT               32
#define REG_ADDR_MAXPOS_Y_DEC              34
#define REG_ADDR_OUT_XY_SUM_U               38
#define REG_ADDR_OUT_XY_SUM_L               40
#define REG_ADDR_MAX_VAL_X                  42
#define REG_ADDR_MAX_VAL_Y                  44

// --- Sensor configuration ---
uint8_t  mode_val             = 6;           // external start
// 400 us (was 500, 2026-07-19): shortens the measurement cycle for a
// higher free-run frame rate during the speed run (6 MST pulses per cycle
// -- the front pair fires twice for the double-buffer flush -- so -100 us
// each saves 600 us/cycle). Spot POSITION (the distance measurement) is
// integration-independent; only signal margin drops (~20%), so if long-
// range readings start dying early, re-tune thresholds or revert via the
// USB 'is'/'if' commands before touching the LUTs.
// PER-CHANNEL integration time (bert 2026-07-29). Was one value for the side
// pair and one for the front pair; the close-range saturation test needs them
// set individually. Indexed by CHANNEL in wire/packet order:
//   0 = left, 1 = front_right, 2 = front_left, 3 = right
// 'is'/'if' still set the side pair / front pair together; 'i0'..'i3' set one.
//
// Close-range saturation (the reason this is per-channel): the spot centre is
// a centroid over several pixels, so once those pixels saturate the centre
// degrades and near targets read badly. The default LUTs bottom out at 50-53 mm
// (lut_left px 28..226 -> 115..53 mm, lut_right px 50..246 -> 50..110 mm) with
// ~30-50 pixels of array still unused beyond, so the close limit is calibration
// coverage, not the sensor edge. Drop one channel's integration time with the
// target close in and watch Maxval_X come out of saturation and Maxpos_X settle
// -- if it does, the auto-cal sweep can be extended below 50 mm at that
// exposure. Note the LUT is calibrated AT an exposure: if Maxpos_X shifts with
// integration time, a re-cal is needed per exposure, not just a new floor.
// ch0/ch3 (left,right) 400 -> 300 us, bert 2026-08-02. Paired with the
// narrow-band filters + binning 0 + threshold 20 on those channels: the
// filter cuts the noise floor, so less integration still clears the lower
// threshold, and a shorter slot also lifts the interlaced frame rate.
// ch1/ch2 (the front pair) stay at 400 -- unfiltered and already the
// weak-signal channels (78-90% of their samples read 255).
uint16_t integration_time_ch[4] = {300, 400, 400, 300};  // us, MST LOW
static const char *const INTEG_CH_NAME[4] = {"left", "front_right",
                                             "front_left", "right"};
uint8_t  blanking_val         = 0;
uint8_t  Ypixel_start_val[3]  = {0, 0, 0};
uint8_t  Xpixel_start_val     = 0;
uint8_t  readout_pixels       = 255;
uint8_t  adc_val[3]           = {0xF5, 0xF5, 0xF5};
// gain bits: [1:0] binning, [5:2] readout gain, [7:6] charge amp.
// IC0 (left) and IC2 (right) binning 3 -> 0 (bert, 2026-08-02, with the wider
// -angle lenses fitted): binning sums neighbouring pixels, which trades spot
// POSITION resolution for signal. The wider lens already spreads more mm
// across each pixel, so paying more resolution for gain is the wrong way
// round on the side channels -- and the L+R corridor-width check reads that
// resolution directly (was sd 0.7-1.3 mm at binning 3). IC1 (the front pair)
// keeps binning 3: it is the weak-signal channel, 78-90% of its samples are
// already 255, and its job is a distance to a wall ahead, not fine lateral
// position.
uint8_t  gain_val[3]          = {0b00000000, 0b00000011, 0b00000000};
uint8_t  calc_area_val[3]     = {0, 1, 0};  // 0=5px, 1=9px, 2=19px
// IC0/IC2 threshold 60 -> 20. Lower floor = a spot is accepted with less
// signal, which is what recovers the grazing/off-axis returns that were
// reading 255 (76% dropout at 25-46 deg off-axis). Costs noise immunity, so
// watch for phantom close readings; the L+R sum check catches those too.
// IC1 (front) stays at 60.
//
// SIDES 20 -> 22 (bert, 2026-08-15). A HAND value again, and deliberately so:
// the auto-set version of this is REVERTED. threshold_selftest() still measures
// the noise floor at boot, but it only reports -- it no longer writes anything.
// See the note on it below for what the auto-set did and why it is gone.
uint8_t  threshold_val[3]     = {22, 60, 22};

// ---- BOOT THRESHOLD SELF-TEST (bert, 2026-08-15) ---------------------------
// "a threshold test at boot would help set that floor." The 'N' dark read has
// measured the noise floor since 2026-08-02, but the number it produced was
// then typed back in by hand, which means the mouse runs on whatever the
// lighting was on the day someone last did it. Ambient is the one input to this
// that changes without anybody changing anything -- so it is worth measuring at
// every power-on, and this does.
//
// IT MEASURES AND REPORTS. IT DOES NOT SET (bert, 2026-08-15). A version that
// set the side thresholds to floor + 20% was built and then reverted the same
// day; the thresholds are hand values again. That is the conservative
// arrangement, and the asymmetry below is the argument for it -- an auto-set
// threshold that comes out too LOW fails silently, and this loop has no
// downstream check that would catch it.
//
// What the check is FOR, then: the same +20% margin, used as a TEST rather than
// a setter. If the measured floor is within 20% of the configured threshold,
// the threshold is too close to the noise to mean anything, and the report says
// so. If the floor is at or above it, the threshold is UNDER the noise, and
// that is the dangerous one:
//   * TOO LOW IS SILENT. The peak detector latches on noise and the LUT
//     converts a noise pixel into a confident millimetre reading. Nothing
//     downstream catches that -- walls_beam_hit trusts any value under 255,
//     and with the horizon active the mouse now steers at what it is told.
//   * TOO HIGH IS LOUD. Readings just go to 255 and the horizon stands down.
// Only one of those two announces itself, which is why the quiet one gets the
// louder warning.
//
// For scale: a GOOD side return reads Maxval ~24 with the current optics (see
// the valid_min note below), so there is not much room between "noise" and
// "signal" to begin with. A floor anywhere near 20 means the room is too bright
// for these lenses, not that the mouse is fine.
//
// SIDE CHANNELS ONLY for the margin check. IC1 carries BOTH front channels, so
// one register serves two very different signal levels there, and the front
// pair is the weak-signal channel already (78-90% of its samples read 255). The
// front floors are still measured and reported, just not judged.
static const uint16_t THRESH_MARGIN_PCT = 120;   // want threshold >= floor+20%

// What the self-test found, kept for the report packet and the USB dump. The
// mouse is ALWAYS powered off before its flight log is read, so a number that
// only ever reaches this MCU's USB console is a number bert never sees on a
// real run -- CMD_THRESH_REP exists to get it to the main MCU.
uint8_t  boot_dark[4]    = {0, 0, 0, 0};   // measured floor per channel
uint8_t  boot_thresh[4]  = {0, 0, 0, 0};   // threshold in force per channel
uint8_t  boot_flags      = 0;              // see THRESH_F_* below
#define THRESH_F_RAN      0x01   // the self-test completed
#define THRESH_F_THIN     0x02   // a side floor is within 20% of its threshold
#define THRESH_F_UNDER    0x04   // a side floor is AT OR ABOVE its threshold

// SOFTWARE VALIDITY GATE -- a SECOND, entirely separate threshold (bert
// 2026-08-02). threshold_val above is the S15366's own REG_ADDR_MAX_THRESHOLD,
// applied inside the sensor. THIS one is the firmware's own "do I trust this
// spot" test: below it the channel reports 255 regardless of what the LUT says.
// It was hardcoded (170 / 120 / 170 / 170) in eight places.
//
// It has to be per-channel and tunable because Maxval scales with the optical
// and gain settings, and the side channels just lost about 13x of it:
//     binning 3 -> 0        /8
//     integration 400->300  x0.75
//     narrow-band filter    x0.85
// so a side channel that used to read Maxval 300 now reads ~24 -- far under the
// old 170 gate. Every reading became 255 while the LUT stayed perfectly good,
// because calibration captures Maxpos (spot POSITION), which none of those
// three changes affect. Good LUT plot, OUT OF RANGE live.
//
// Indexed by CHANNEL (0 left, 1 front_right, 2 front_left, 3 right). Set the
// side values from the 'N' dark read: a few times the measured noise floor.
// 'M<ch><val>' changes one at runtime; 'M' reports.
uint8_t  valid_min[4]         = {20, 120, 170, 20};

// --- Pin arrays ---
uint8_t SPI_CS[3]   = {SPI_CS_1,   SPI_CS_2,   SPI_CS_3};
uint8_t MST[3]      = {MST_1,      MST_2,      MST_3};
uint8_t laser[4]    = {laser_1,    laser_3,    laser_2,    laser_4};
// Sensor-to-laser mapping (laser_2/laser_3 swapped to correct hardware wiring):
//   Right  IC0 MST[0] → laser[3] (laser_4, pin 19)
//   Front  IC1 MST[1] → laser[1] (laser_3, pin  4)  front-right
//                      → laser[2] (laser_2, pin  3)  front-left
//   Left   IC2 MST[2] → laser[0] (laser_1, pin 22)
uint8_t miso_mux[3] = {MISO_MUX_1, MISO_MUX_2, MISO_MUX_3};

// --- Last readout ---
uint32_t Frame_cnt    = 0;
uint32_t Maxpos_X_int = 0;
uint32_t Maxpos_X_dec = 0;
uint32_t Maxpos_Y_int = 0;
uint32_t Maxpos_Y_dec = 0;
uint32_t Out_XY_sum_U = 0;
uint32_t Out_XY_sum_L = 0;
uint32_t Maxval_X     = 0;
uint32_t Maxval_Y     = 0;

// --- Distance results ---
// NOTE: these are the DIRECT range along each laser beam (the calibrated LUT
// value), not the perpendicular distance to the wall. Because the lasers are
// mounted at an angle and offset from the mouse centre, conversion to a wall
// distance is done off-board (GUI now; main MCU later, fused with IMU yaw to
// resolve the offset-vs-yaw ambiguity a single side beam cannot).
// Named by WALL measured (side beams cross): left_mm = IC0 (right-mounted),
// right_mm = IC2 (left-mounted). Packet order stays {left, fr, fl, right}.
float left_mm        = 255;   // packet [0] — IC0, right-mounted
float front_right_mm = 255;   // packet [1] — IC1
float front_left_mm  = 255;   // packet [2] — IC1
float right_mm       = 255;   // packet [3] — IC2, left-mounted

// --- Laser mounting geometry (reference; body frame) ---
// x = forward (mm, +ahead), y = left (mm, +to the mouse's left).
// angle = beam direction in degrees CCW from forward (+x):
//   0 = ahead, +90 = straight left, -90 = straight right.
// Angles are the MEASURED, fixed per-laser values. Positions: front lasers
// 28 mm forward, 18 mm apart (+/-9 mm); side lasers 40 mm forward, 30 mm apart
// (+/-15 mm). Front lasers angle inward (beams cross ahead). Side beams CROSS:
// each side sensor reads the OPPOSITE wall.
// Channel/packet order {ch0, ch1, ch2, ch3}: ch0 is right-mounted (y=-15) but
// angled +72.8 deg so it reads the LEFT wall; ch3 is left-mounted (y=+15) at
// -71 deg and reads the RIGHT wall. Positions are physical mount; unchanged.
// Recorded here as the physical reference; not yet used by this firmware.
const float laser_x_fwd_mm[4]  = {  40.0f,  28.0f,  28.0f,  40.0f };
const float laser_y_left_mm[4] = { -15.0f,  -9.0f,   9.0f,  15.0f };
const float laser_angle_deg[4] = {  72.8f,   8.4f,  -9.6f, -71.0f };

// =============================================================================
// Distance lookup tables — Maxpos_X_int (0-255) → distance_mm (uint8_t)
// 255 = out of range.  Populate entries from real measurements.
// Tables are named by the WALL each sensor measures (side beams cross):
//   lut_left  = IC0, right-mounted, reads LEFT wall  (0→255 direct)
//   lut_right = IC2, left-mounted,  reads RIGHT wall (255→0 inverted)
//   lut_front_r, lut_front_l = front (255→0 inverted)
// =============================================================================

uint8_t lut_left[256] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 115, 114, 114, 113,
    113, 112, 112, 111, 111, 110, 110, 109, 109, 108, 108, 107, 107, 106, 106, 105,
    105, 104, 104, 102, 102, 101, 101, 100, 100,  99,  99,  98,  98,  97,  97,  96,
     96,  95,  95,  94,  94,  94,  93,  93,  92,  92,  92,  91,  91,  90,  90,  90,
     89,  89,  88,  88,  88,  87,  87,  86,  86,  86,  85,  85,  84,  84,  84,  83,
     83,  83,  82,  82,  82,  82,  81,  81,  81,  80,  80,  80,  79,  79,  79,  78,
     78,  78,  77,  77,  77,  76,  76,  76,  76,  75,  75,  75,  74,  74,  74,  74,
     73,  73,  73,  73,  72,  72,  72,  72,  72,  71,  71,  71,  71,  70,  70,  70,
     70,  70,  69,  69,  69,  69,  68,  68,  68,  68,  68,  67,  67,  67,  67,  66,
     66,  66,  66,  66,  65,  65,  65,  65,  64,  64,  64,  64,  64,  62,  62,  62,
     62,  62,  62,  61,  61,  61,  61,  61,  61,  60,  60,  60,  60,  60,  59,  59,
     59,  59,  59,  59,  58,  58,  58,  58,  58,  57,  57,  57,  57,  57,  57,  56,
     56,  56,  56,  56,  55,  55,  55,  55,  55,  55,  54,  54,  54,  54,  54,  54,
     53,  53,  53, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
};

uint8_t lut_front_r[256] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 235, 225, 220, 215, 210, 205, 203, 199, 196, 191, 186, 183,
    179, 176, 173, 169, 166, 164, 161, 158, 156, 154, 152, 150, 148, 146, 144, 141,
    138, 136, 134, 134, 132, 130, 129, 127, 126, 124, 123, 121, 120, 118, 117, 116,
    115, 114, 113, 111, 110, 109, 108, 107, 106, 105, 105, 104, 103, 102, 101, 100,
     99,  99,  98,  97,  96,  95,  95,  94,  93,  92,  92,  91,  90,  89,  89,  88,
     87,  86,  86,  85,  85,  84,  84,  83,  83,  82,  81,  81,  80,  80,  79,  79,
     78,  78,  77,  77,  76,  76,  75,  75,  75,  74,  74,  74,  73,  73,  72,  72,
     72,  71,  71,  70,  70,  70,  69,  69,  69,  68,  68,  67,  67,  67,  66,  66,
     66,  66,  65,  65,  65,  65,  64,  64,  64,  64,  63,  63,  63,  63,  63,  63,
     63,  63,  62,  62,  62,  62,  61,  61,  61,  61,  60,  60,  60,  60,  59,  59,
     59,  59,  58,  58,  57,  57,  56,  55,  54,  54,  53,  52,  52,  51,  50,  49,
     49,  48, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
};

uint8_t lut_front_l[256] = {
    255, 255, 255, 255, 255, 255, 255,  48,  48,  48,  49,  49,  49,  49,  50,  50,
     50,  50,  51,  51,  51,  51,  51,  52,  52,  52,  52,  53,  53,  53,  53,  54,
     54,  54,  54,  55,  55,  55,  55,  55,  56,  56,  56,  56,  57,  57,  57,  57,
     58,  58,  58,  58,  58,  59,  59,  59,  59,  60,  60,  60,  60,  60,  61,  61,
     61,  61,  62,  62,  62,  62,  63,  63,  63,  63,  63,  64,  64,  64,  64,  65,
     65,  65,  65,  66,  66,  66,  66,  66,  67,  67,  67,  67,  68,  68,  68,  68,
     69,  69,  69,  70,  70,  70,  70,  71,  71,  71,  72,  72,  72,  73,  73,  73,
     74,  74,  74,  74,  74,  74,  75,  75,  75,  75,  76,  76,  76,  77,  77,  77,
     78,  79,  79,  79,  80,  81,  81,  81,  82,  83,  83,  83,  84,  85,  85,  85,
     86,  87,  87,  88,  88,  89,  90,  90,  91,  92,  92,  93,  94,  94,  95,  96,
     96,  97,  98,  99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
    112, 112, 113, 114, 115, 116, 117, 118, 120, 121, 123, 124, 126, 127, 129, 130,
    132, 134, 135, 137, 139, 140, 142, 144, 145, 147, 149, 152, 155, 157, 159, 162,
    165, 166, 171, 176, 179, 183, 186, 188, 191, 194, 196, 201, 206, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
};

uint8_t lut_right[256] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255,  50,  50,  50,  50,  50,  50,  50,  50,  50,  52,  52,  52,  52,  52,
     52,  52,  52,  52,  52,  52,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,
     54,  54,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  58,  58,  58,
     58,  58,  58,  58,  58,  58,  58,  58,  58,  60,  60,  60,  60,  60,  60,  60,
     60,  60,  62,  62,  62,  62,  62,  62,  62,  62,  64,  64,  64,  64,  64,  64,
     64,  64,  66,  66,  66,  66,  66,  66,  66,  66,  68,  68,  68,  68,  68,  68,
     68,  68,  70,  70,  70,  70,  70,  70,  70,  70,  72,  72,  72,  72,  72,  72,
     74,  74,  74,  74,  74,  74,  74,  76,  76,  76,  76,  76,  76,  76,  78,  78,
     78,  78,  78,  78,  78,  80,  80,  80,  80,  80,  80,  82,  82,  82,  82,  82,
     84,  84,  84,  84,  84,  86,  86,  86,  86,  86,  86,  88,  88,  88,  88,  88,
     90,  90,  90,  90,  90,  92,  92,  92,  92,  94,  94,  94,  94,  94,  96,  96,
     96,  96,  98,  98,  98,  98, 100, 100, 100, 100, 102, 102, 102, 104, 104, 104,
    106, 106, 106, 108, 108, 108, 110, 255, 255, 255, 255, 255, 255, 255, 255, 255,
};


// --- Per-channel raw values — all S15366 output registers, 4 channels ---
// Index: [0]=right, [1]=front-right, [2]=front-left, [3]=left
static uint8_t ch_frame[4] = {};   // FRAME_COUNT
static uint8_t ch_x_int[4] = {};   // MAXPOS_X_INT  (spot position, used for calibration)
static uint8_t ch_x_dec[4] = {};   // MAXPOS_X_DEC  (sub-pixel)
static uint8_t ch_y_int[4] = {};   // MAXPOS_Y_INT
static uint8_t ch_y_dec[4] = {};   // MAXPOS_Y_DEC
static uint8_t ch_sum_u[4] = {};   // OUT_XY_SUM upper byte  (total energy in calc area)
static uint8_t ch_sum_l[4] = {};   // OUT_XY_SUM lower byte
static uint8_t ch_sig_x[4] = {};   // MAX_VAL_X  (peak signal — used for threshold)
static uint8_t ch_sig_y[4] = {};   // MAX_VAL_Y
// Convenience aliases kept for legacy callers
#define px  ch_x_int
#define sig ch_sig_x

// --- Laser enable flag: set by 'r'/'d', cleared by 'q' ---
static bool lasers_enabled = false;

// --- Free-run flavour: classic (primer double-fire) vs interlaced ---
// Declared here (above all uses): set by CMD_FREE_RUN_IL / USB 'I',
// cleared by CMD_STOP / 'q' / 'd' / 'r'.
static bool interlaced = false;

// --- Per-channel enable: toggled by 'L{ch}{0/1}' command ---
// Index = TRANSMIT / dist-stream order (verified 2026-08-06; the prior
// comment had 0 and 3 reversed): 0=Left, 1=Front-R, 2=Front-L, 3=Right.
// save_channel(ch), laser_ch_enabled[ch], and the 'W' LUT map all use
// this same order (ch0->lut_left, 1->lut_front_r, 2->lut_front_l,
// 3->lut_right), matching the four Serial5.write bytes below.
static bool laser_ch_enabled[4] = {true, true, true, true};


// =============================================================================
// SPI helpers
// =============================================================================

// SPI slowed to 1 MHz (from the fast default) to rule out marginal config
// writes / bus noise causing the intermittent all-255. MODE0, MSB first -- the
// same mode as before; only the clock changes.
static const SPISettings S15366_SPI(1000000, MSBFIRST, SPI_MODE0);

void S15366_SPI_Write(uint8_t address, uint8_t value, uint8_t port) {
  SPI.beginTransaction(S15366_SPI);
  digitalWriteFast(miso_mux[port], HIGH);   // assert OE before CS
  digitalWriteFast(SPI_CS[port], LOW);
  SPI.transfer(address | 1);   // D0 high = write
  SPI.transfer(value);
  digitalWriteFast(SPI_CS[port], HIGH);
  digitalWriteFast(miso_mux[port], LOW);    // release OE after CS
  SPI.endTransaction();
}

uint8_t S15366_SPI_Read(uint8_t address, uint8_t port) {
  SPI.beginTransaction(S15366_SPI);
  digitalWriteFast(miso_mux[port], HIGH);
  digitalWriteFast(SPI_CS[port], LOW);
  SPI.transfer(address);       // D0 low = read
  uint8_t value = SPI.transfer(0);
  digitalWriteFast(SPI_CS[port], HIGH);
  digitalWriteFast(miso_mux[port], LOW);
  SPI.endTransaction();
  return value;
}

// Batched register read: ONE SPI transaction + ONE MISO-OE assert for a
// whole burst (the S15366 doesn't tri-state MISO, so the OE gates the
// shared bus -- it only needs toggling per PORT, not per register). CS is
// still pulsed per register (the part latches the address on the CS
// edge). Replaces 9 separate begin/OE/end cycles in read_all -- the SPI
// overhead the free-run frame rate was paying (bert, 2026-08-06).
static void S15366_read_regs(uint8_t port, const uint8_t *addrs,
                             uint8_t *out, int n) {
  SPI.beginTransaction(S15366_SPI);
  digitalWriteFast(miso_mux[port], HIGH);     // OE: this IC drives MISO
  for (int i = 0; i < n; i++) {
    digitalWriteFast(SPI_CS[port], LOW);
    SPI.transfer(addrs[i]);                    // D0 low = read
    out[i] = SPI.transfer(0);
    digitalWriteFast(SPI_CS[port], HIGH);
  }
  digitalWriteFast(miso_mux[port], LOW);       // release OE
  SPI.endTransaction();
}

// FAST PATH: only the two registers the distance needs (spot position +
// peak signal). Used by the measurement loop; ~4.5x fewer SPI reads than
// read_all, straight to the free-run frame rate.
void read_min(uint8_t port) {
  static const uint8_t A[2] = {REG_ADDR_MAXPOS_X_INT, REG_ADDR_MAX_VAL_X};
  uint8_t v[2];
  S15366_read_regs(port, A, v, 2);
  Maxpos_X_int = v[0];
  Maxval_X     = v[1];
}

// FULL readout (all 9 output registers) -- for the raw 'r' cal stream.
void read_all(uint8_t port) {
  static const uint8_t A[9] = {
    REG_ADDR_FRAME_COUNT,  REG_ADDR_MAXPOS_X_INT, REG_ADDR_MAXPOS_X_DEC,
    REG_ADDR_MAXPOS_Y_INT, REG_ADDR_MAXPOS_Y_DEC, REG_ADDR_OUT_XY_SUM_U,
    REG_ADDR_OUT_XY_SUM_L, REG_ADDR_MAX_VAL_X,    REG_ADDR_MAX_VAL_Y};
  uint8_t v[9];
  S15366_read_regs(port, A, v, 9);
  Frame_cnt    = v[0]; Maxpos_X_int = v[1]; Maxpos_X_dec = v[2];
  Maxpos_Y_int = v[3]; Maxpos_Y_dec = v[4]; Out_XY_sum_U = v[5];
  Out_XY_sum_L = v[6]; Maxval_X     = v[7]; Maxval_Y     = v[8];
}

// USB calibration-stream mode (declared here, above the measure
// functions, so read_meas can pick full vs min readout). The full USB
// interface that SETS this lives further down.
#define USB_CAL_OFF   0   // no USB output
#define USB_CAL_RAW   1   // px + sig + dist per channel
#define USB_CAL_DIST  2   // dist values only
static uint8_t usb_cal_mode = USB_CAL_OFF;

// The measurement loop reads MIN normally, ALL only when the raw cal
// stream is active (it needs every register). Set once per cycle.
static bool g_raw_readout = false;

// RAW-PX packet mode (Serial5): append px+sig per channel to the mm
// packet for the auto-cal LUT sweep. Set by CMD_FREE_RUN_RAW, cleared by
// CMD_STOP. read_min already fetches both px (Maxpos_X_int) and signal
// (Maxval_X), so no extra SPI cost.
static bool g_send_raw = false;
static inline void read_meas(uint8_t port) {
  if (g_raw_readout) read_all(port); else read_min(port);
}

// =============================================================================
// Sensor init
// =============================================================================

void S15366_init() {
  for (uint8_t x = 0; x < 3; x++) {
    S15366_SPI_Write(REG_ADDR_MODE,                      mode_val,              x);
    // Integration time is controlled by MST pulse width, not the register (external trigger mode)
    S15366_SPI_Write(REG_ADDR_BLANKING,                  blanking_val,           x);
    S15366_SPI_Write(REG_ADDR_PIXEL_READOUT_START_Y,     Ypixel_start_val[x],   x);
    S15366_SPI_Write(REG_ADDR_PIXEL_READOUT_START_X,     Xpixel_start_val,      x);
    S15366_SPI_Write(REG_ADDR_PIXEL_READOUT_COUNT,       readout_pixels,         x);
    S15366_SPI_Write(REG_ADDR_ADC,                       adc_val[x],             x);
    S15366_SPI_Write(REG_ADDR_GAIN,                      gain_val[x],            x);
    S15366_SPI_Write(REG_ADDR_CALC_AREA,                 calc_area_val[x],       x);
    S15366_SPI_Write(REG_ADDR_MAX_THRESHOLD,             threshold_val[x],       x);
  }
}

// =============================================================================
// Sensor detection
// =============================================================================
//
// Detects whether each S15366 IC is responding over SPI.
// Method: pulse MST twice without a laser; check if FRAME_COUNT increments.
// A working IC increments FRAME_COUNT on every measurement trigger.
// A missing/dead IC returns a stuck value (0x00 or 0xFF) for all reads.
//
// sensor_present[0] = right wall sensor (IC 0)
// sensor_present[1] = front sensor     (IC 1, drives both front channels)
// sensor_present[2] = left wall sensor (IC 2)
//
// Packet encoding per channel:
//   0       — sensor IC hardware error (not detected / SPI fault)
//   1–254   — distance in mm
//   255     — out of range (valid measurement, nothing detected)
// =============================================================================

static bool sensor_present[3] = {false, false, false};

static bool sensor_detect(uint8_t port) {
  // Write a known value to the MODE register then read it back.
  // Reliable even when FRAME_COUNT is stuck (e.g. MISO path issue).
  // mode_val = 6; if readback != 6 the IC is absent or SPI is broken.
  S15366_SPI_Write(REG_ADDR_MODE, mode_val, port);
  delayMicroseconds(50);
  uint8_t rb = S15366_SPI_Read(REG_ADDR_MODE, port);
  return (rb == mode_val);
}

static void sensors_detect_all() {
  for (uint8_t i = 0; i < 3; i++) {
    S15366_SPI_Write(REG_ADDR_MODE, mode_val, i);
    delayMicroseconds(50);
    uint8_t rb = S15366_SPI_Read(REG_ADDR_MODE, i);
    sensor_present[i] = (rb == mode_val);
    Serial.printf("  Sensor IC %d (%s): %s  (MODE reg readback 0x%02X, expect 0x%02X)\n",
                  i,
                  (i == 0) ? "right" : (i == 1) ? "front" : "left",
                  sensor_present[i] ? "OK" : "NOT DETECTED",
                  rb, mode_val);
  }
}

// =============================================================================
// Laser helpers
// =============================================================================

void lasers_all_off() {
  digitalWrite(laser_1, 0);
  digitalWrite(laser_2, 0);
  digitalWrite(laser_3, 0);
  digitalWrite(laser_4, 0);
}

// Save all S15366 output-register globals into a channel slot.
static inline void save_channel(uint8_t ch) {
  ch_frame[ch] = Frame_cnt;
  ch_x_int[ch] = Maxpos_X_int;
  ch_x_dec[ch] = Maxpos_X_dec;
  ch_y_int[ch] = Maxpos_Y_int;
  ch_y_dec[ch] = Maxpos_Y_dec;
  ch_sum_u[ch] = Out_XY_sum_U;
  ch_sum_l[ch] = Out_XY_sum_L;
  ch_sig_x[ch] = Maxval_X;
  ch_sig_y[ch] = Maxval_Y;
}

static inline void clear_channel(uint8_t ch) {
  ch_frame[ch] = ch_x_int[ch] = ch_x_dec[ch] = ch_y_int[ch] = ch_y_dec[ch] = 0;
  ch_sum_u[ch] = ch_sum_l[ch] = ch_sig_x[ch] = ch_sig_y[ch] = 0;
}

// Write all current config values to one IC (or all ICs if ic == 3).
static void apply_params(uint8_t ic) {
  uint8_t lo = ic, hi = ic + 1;
  if (ic >= 3) { lo = 0; hi = 3; }
  for (uint8_t x = lo; x < hi; x++) {
    S15366_SPI_Write(REG_ADDR_MODE,                       mode_val,              x);
    // Integration time is controlled by MST pulse width, not the register (external trigger mode)
    S15366_SPI_Write(REG_ADDR_BLANKING,                   blanking_val,          x);
    S15366_SPI_Write(REG_ADDR_PIXEL_READOUT_START_Y,      Ypixel_start_val[x],  x);
    S15366_SPI_Write(REG_ADDR_PIXEL_READOUT_START_X,      Xpixel_start_val,     x);
    S15366_SPI_Write(REG_ADDR_PIXEL_READOUT_COUNT,        readout_pixels,        x);
    S15366_SPI_Write(REG_ADDR_ADC,                        adc_val[x],            x);
    S15366_SPI_Write(REG_ADDR_GAIN,                       gain_val[x],           x);
    S15366_SPI_Write(REG_ADDR_CALC_AREA,                  calc_area_val[x],      x);
    S15366_SPI_Write(REG_ADDR_MAX_THRESHOLD,              threshold_val[x],      x);
  }
}

// =============================================================================
// One complete measurement cycle — called only when trigger received
// =============================================================================

// Settle time after MST HIGH (end of integration) before the SPI readout.
// The original free-run firmware used 50 us and read valid data, so 50 is the
// value here. (100 us was tried for single-shot/triggered mode -- since
// abandoned in favour of free-run -- and did not help.) Tune here.
static const uint16_t SENSOR_SETTLE_US = 50;

void measure_and_send() {
  g_raw_readout = (usb_cal_mode == USB_CAL_RAW);
  lasers_all_off();

  // If lasers are disabled (USB 'q' received), send an all-255 packet
  // to keep the main MCU happy but do not fire any laser.
  if (!lasers_enabled) {
    Serial5.write(sensor_present[0] ? (uint8_t)255 : (uint8_t)0);
    Serial5.write(sensor_present[1] ? (uint8_t)255 : (uint8_t)0);
    Serial5.write(sensor_present[1] ? (uint8_t)255 : (uint8_t)0);
    Serial5.write(sensor_present[2] ? (uint8_t)255 : (uint8_t)0);
    Serial5.write(0x0D);
    return;
  }

  // Packet byte encoding:
  //   0       = sensor IC hardware error
  //   1–254   = distance mm
  //   255     = out of range

  // --- Left wall (sensor IC 0, right-mounted, laser[3]; beam crosses) ---
  if (!sensor_present[0]) {
    clear_channel(0);
    Serial5.write((uint8_t)0);
  } else if (!laser_ch_enabled[0]) {
    clear_channel(0);
    Serial5.write((uint8_t)255);
  } else {
    digitalWriteFast(laser[3], 1);
    digitalWriteFast(MST[0], LOW);
    delayMicroseconds(integration_time_ch[0]);
    digitalWriteFast(MST[0], HIGH);
    digitalWriteFast(laser[3], 0);
    delayMicroseconds(SENSOR_SETTLE_US);
    read_meas(0);
    save_channel(0);
    left_mm = (Maxval_X < valid_min[0]) ? 255 : lut_left[Maxpos_X_int];
    Serial5.write((uint8_t)left_mm);
  }

  // --- Front-right and front-left (sensor IC 1, laser[1] and laser[2]) ---
  //
  // IC1 (S15366-256) is double-buffered: the SPI readout returns the result
  // of the PREVIOUS MST integration, not the one that just completed.
  // When both lasers fire back-to-back, each read returns the other laser's
  // data.  Fix: fire each laser TWICE.  The first (primer) integration flushes
  // the stale value out of the output buffer; the second gives valid data.
  //
  //   laser ON → MST LOW → integration_time µs → MST HIGH → laser OFF → 50 µs → read_all
  //
  if (!sensor_present[1]) {
    clear_channel(1);  clear_channel(2);
    front_right_mm = 0;
    front_left_mm  = 0;
    Serial5.write((uint8_t)0);
    Serial5.write((uint8_t)0);
  } else {
    // ── Front-right (laser[1]) ───────────────────────────────────────────
    if (!laser_ch_enabled[1]) {
      clear_channel(1);
      front_right_mm = 255;
    } else {
      // Primer — flushes stale data out of IC1 output buffer
      digitalWriteFast(laser[1], 1);
      digitalWriteFast(MST[1], LOW);
      delayMicroseconds(integration_time_ch[1]);
      digitalWriteFast(MST[1], HIGH);
      digitalWriteFast(laser[1], 0);
      delayMicroseconds(SENSOR_SETTLE_US);
      read_meas(1);  // primer discard
      // Actual measurement — now reads the primer's valid result
      digitalWriteFast(laser[1], 1);
      digitalWriteFast(MST[1], LOW);
      delayMicroseconds(integration_time_ch[1]);
      digitalWriteFast(MST[1], HIGH);
      digitalWriteFast(laser[1], 0);
      delayMicroseconds(SENSOR_SETTLE_US);
      read_meas(1);
      save_channel(1);
      front_right_mm = (Maxval_X < valid_min[1]) ? 255 : lut_front_r[Maxpos_X_int];
    }

    // ── Front-left (laser[2]) ────────────────────────────────────────────
    if (!laser_ch_enabled[2]) {
      clear_channel(2);
      front_left_mm = 255;
    } else {
      // Primer
      digitalWriteFast(laser[2], 1);
      digitalWriteFast(MST[1], LOW);
      delayMicroseconds(integration_time_ch[2]);
      digitalWriteFast(MST[1], HIGH);
      digitalWriteFast(laser[2], 0);
      delayMicroseconds(SENSOR_SETTLE_US);
      read_meas(1);                          // discard
      // Actual measurement
      digitalWriteFast(laser[2], 1);
      digitalWriteFast(MST[1], LOW);
      delayMicroseconds(integration_time_ch[2]);
      digitalWriteFast(MST[1], HIGH);
      digitalWriteFast(laser[2], 0);
      delayMicroseconds(SENSOR_SETTLE_US);
      read_meas(1);
      save_channel(2);
      front_left_mm = (Maxval_X < valid_min[2]) ? 255 : lut_front_l[Maxpos_X_int];
    }

    Serial5.write((uint8_t)front_right_mm);
    Serial5.write((uint8_t)front_left_mm);
  }

  // --- Right wall (sensor IC 2, left-mounted, laser[0]; beam crosses) ---
  if (!sensor_present[2]) {
    clear_channel(3);
    Serial5.write((uint8_t)0);
  } else if (!laser_ch_enabled[3]) {
    clear_channel(3);
    Serial5.write((uint8_t)255);
  } else {
    digitalWriteFast(laser[0], 1);
    digitalWriteFast(MST[2], LOW);
    delayMicroseconds(integration_time_ch[3]);
    digitalWriteFast(MST[2], HIGH);
    digitalWriteFast(laser[0], 0);
    delayMicroseconds(SENSOR_SETTLE_US);
    read_meas(2);
    save_channel(3);
    right_mm = (Maxval_X < valid_min[3]) ? 255 : lut_right[Maxpos_X_int];
    Serial5.write((uint8_t)right_mm);
  }

  // RAW-px calibration append (bert, 2026-08-06): in raw mode the 4 mm
  // bytes are followed by raw px + signal per channel (spot position and
  // peak strength -- the auto-cal ground truth), making a 13-byte packet
  // [L,FR,FL,R, px0,sig0,px1,sig1,px2,sig2,px3,sig3, 0x0D]. ch_x_int /
  // ch_sig_x were saved by save_channel above. Normal mode is unchanged
  // (5 bytes). Order matches the mm bytes: 0=left,1=fr,2=fl,3=right.
  if (g_send_raw) {
    Serial5.write((uint8_t)ch_x_int[0]); Serial5.write((uint8_t)ch_sig_x[0]);
    Serial5.write((uint8_t)ch_x_int[1]); Serial5.write((uint8_t)ch_sig_x[1]);
    Serial5.write((uint8_t)ch_x_int[2]); Serial5.write((uint8_t)ch_sig_x[2]);
    Serial5.write((uint8_t)ch_x_int[3]); Serial5.write((uint8_t)ch_sig_x[3]);
  }
  // --- Packet terminator ---
  Serial5.write(0x0D);

  // Ensure all lasers are off after the measurement cycle.
  // Each laser is already turned off immediately after its own MST pulse above,
  // but this guarantees no laser stays on if the function exits mid-cycle.
  lasers_all_off();

}

// =============================================================================
// INTERLACED measurement cycle (test mode, 2026-07-19, bert's design)
// =============================================================================
//
// Fire continuously: left, front-right, right, front-left. Read each IC
// one firing slot LATE, during the next slot's integration -- the SPI
// readout hides inside the 400 us MST window, so it costs nothing, and
// no IC is ever read while integrating (IC1's two firings are separated
// by another slot on both sides).
//
// Double-buffer attribution (empirical, from the primer experiments: a
// read returns the IC's PREVIOUS integration): on the front IC this
// shifts which CHANNEL emerges --
//   during slot 0 (fire L)  read IC1 -> front-RIGHT (previous cycle)
//   during slot 1 (fire FR) read IC0 -> left
//   during slot 2 (fire R)  read IC1 -> front-LEFT  (previous cycle)
//   during slot 3 (fire FL) read IC2 -> right
// Every channel updates once per cycle; worst staleness one cycle
// (~1.7 ms). Expected rate ~580 Hz at 400 us integration; per-laser duty
// ~23% (vs ~87% classic free-run), so the thermal cap relaxes too.
//
// One packet per cycle, same 5-byte format, sent after slot 3's read.
// First cycle after entry transmits one stale/garbage set -- harmless
// (main MCU debounces everything).
// =============================================================================

// Firing slots: {laser index, IC}. Order: L, FR, R, FL.
static const uint8_t il_laser[4] = {3, 1, 0, 2};
static const uint8_t il_ic[4]    = {0, 1, 2, 1};
// Slot -> CHANNEL (wire/packet order 0=left,1=front_right,2=front_left,3=right).
// Derive it from the READ, not from the fire order: each slot's result is read
// one slot LATER, so slot s fires whatever slot (s+1)%4 assigns.
//   slot 0 laser[3] IC0 -> read at slot 1 as left        -> ch0
//   slot 1 laser[1] IC1 -> read at slot 2 as front_left  -> ch2
//   slot 2 laser[0] IC2 -> read at slot 3 as right       -> ch3
//   slot 3 laser[2] IC1 -> read at slot 0 as front_right -> ch1
// (The front pair CROSS, which is why laser[1] ends up as front_LEFT here.)
static const uint8_t il_ch[4]    = {0, 2, 3, 1};

// ---- DARK READ (noise floor), bert 2026-08-02 -------------------------------
// Integrate and read WITHOUT firing the laser: Maxval_X is then pure noise
// floor -- sensor dark current plus whatever ambient light gets through the
// optics. That is the number the detection threshold should sit above, so with
// narrow-band filters now on L&R it can be measured per channel instead of
// guessed.
//
// The existing 'L<ch>0' laser toggle CANNOT be used for this: that branch calls
// clear_channel() and never reads the IC at all, so it reports zeros rather
// than the floor.
//
// THREE integrations, last one kept (two until 2026-09-22). The S15366
// double-buffers: the readout after MST pulse N returns integration N-1.
// An IC that has been IDLE -- since power-on at boot, since the stream
// stopped for a 'T' re-run -- has been integrating the whole time, so its
// first pulse here ends an integration milliseconds long. With two passes
// the kept read returned exactly that: pass 0's pulse ended the idle
// integration and read garbage, pass 1's pulse ended the first real 400 us
// frame and read the IDLE one. The front IC is unfiltered, so milliseconds
// of room light on it came out as "ch1 front_r dark 88" at every boot for
// six weeks, while ch2 -- the same IC, the same settings, measured second,
// after the IC was already cycling -- read 0, and the filtered sides read 0
// either way. It was never the room and never the threshold: it was the
// first channel on an idle IC. Pass 2 reads the first genuine dark frame.
static const uint8_t ch_laser[4] = {3, 1, 2, 0};   // ch -> laser index
static const uint8_t ch_ic[4]    = {0, 1, 1, 2};   // ch -> sensor IC
static const int DARK_PASSES = 3;

static void measure_dark(uint8_t ch, uint32_t *val_out, uint32_t *pos_out) {
  uint8_t ic = ch_ic[ch];
  if (!sensor_present[ic]) { *val_out = 0; *pos_out = 0; return; }
  for (int pass = 0; pass < DARK_PASSES; pass++) {
    digitalWriteFast(laser[ch_laser[ch]], 0);   // explicitly dark
    digitalWriteFast(MST[ic], LOW);
    delayMicroseconds(integration_time_ch[ch]);
    digitalWriteFast(MST[ic], HIGH);
    delayMicroseconds(SENSOR_SETTLE_US);
    read_meas(ic);                              // all but the last discarded
  }
  *val_out = Maxval_X;
  *pos_out = Maxpos_X_int;
}

// ---- BOOT THRESHOLD SELF-TEST ----------------------------------------------
// MEASURES AND REPORTS. It does not write a threshold -- see the long note by
// threshold_val for what the auto-setting version did and why it was reverted.
//
// Dark-read every channel, then judge the two SIDE thresholds against what was
// measured. The test is the margin bert asked the setter for: a threshold
// should sit at least THRESH_MARGIN_PCT of the noise floor, and a threshold at
// or below the floor is the silent failure -- the peak detector latches noise
// and the LUT turns it into a confident millimetre.
//
// Lasers are off for the whole test, so what is in front of the mouse does not
// matter -- only the light in the room. That is worth knowing when reading the
// numbers: a high floor means the venue, not the target.
void threshold_selftest(bool verbose) {
  boot_flags = 0;
  for (uint8_t ch = 0; ch < 4; ch++) {
    uint32_t v = 0, pos = 0;
    measure_dark(ch, &v, &pos);
    boot_dark[ch] = (v > 255) ? 255 : (uint8_t)v;
  }
  for (uint8_t ch = 0; ch < 4; ch++) boot_thresh[ch] = threshold_val[ch_ic[ch]];

  const uint8_t side_ch[2] = {0, 3};              // left, right
  for (uint8_t k = 0; k < 2; k++) {
    uint8_t ch = side_ch[k];
    if (!sensor_present[ch_ic[ch]]) continue;
    uint32_t want = ((uint32_t)boot_dark[ch] * THRESH_MARGIN_PCT + 99u) / 100u;
    if (boot_dark[ch] >= boot_thresh[ch]) boot_flags |= THRESH_F_UNDER;
    else if (want > boot_thresh[ch])      boot_flags |= THRESH_F_THIN;
  }
  boot_flags |= THRESH_F_RAN;

  if (!verbose) return;
  Serial.println("# threshold check (lasers off -- this measures the ROOM)");
  for (uint8_t ch = 0; ch < 4; ch++) {
    uint8_t ic = ch_ic[ch];
    Serial.printf("#   ch%d %-11s IC%d  dark %3d  threshold %3d  margin %+4d"
                  "  valid_min %3d\n",
                  ch, INTEG_CH_NAME[ch], ic, boot_dark[ch], threshold_val[ic],
                  (int)threshold_val[ic] - (int)boot_dark[ch], valid_min[ch]);
  }
  if (boot_flags & THRESH_F_UNDER)
    Serial.println("#   WARNING: a side threshold is AT OR BELOW the measured"
                   " noise floor. The peak detector will\n"
                   "#            latch on noise and the LUT will report it as a"
                   " confident distance. Raise it ('t<ic> <val>').");
  else if (boot_flags & THRESH_F_THIN)
    Serial.printf("#   NOTE: a side threshold is less than %d%% of its noise"
                  " floor -- thin margin. A good side return\n"
                  "#         is only ~24 counts with these optics, so there is"
                  " not much room to give away.\n",
                  (int)THRESH_MARGIN_PCT);
}

void measure_cycle_interlaced() {
  g_raw_readout = (usb_cal_mode == USB_CAL_RAW);
  if (!lasers_enabled) {
    measure_and_send();  // classic path already handles the disabled case
    return;
  }

  for (uint8_t s = 0; s < 4; s++) {
    uint8_t ic = il_ic[s];
    bool fire = sensor_present[ic];
    uint16_t integ = integration_time_ch[il_ch[s]];

    uint32_t t0 = micros();
    if (fire) {
      digitalWriteFast(laser[il_laser[s]], 1);
      digitalWriteFast(MST[ic], LOW);
    }

    // Hidden work: read the PREVIOUS slot's IC (its integration ended a
    // full slot ago -- settled long past SENSOR_SETTLE_US).
    switch (s) {
      case 0:  // IC1 -> front-right (previous cycle)
        if (sensor_present[1] && laser_ch_enabled[1]) {
          read_meas(1);
          save_channel(1);
          front_right_mm = (Maxval_X < valid_min[1]) ? 255 : lut_front_r[Maxpos_X_int];
        } else {
          front_right_mm = sensor_present[1] ? 255 : 0;
        }
        break;
      case 1:  // IC0 -> left
        if (sensor_present[0] && laser_ch_enabled[0]) {
          read_meas(0);
          save_channel(0);
          left_mm = (Maxval_X < valid_min[0]) ? 255 : lut_left[Maxpos_X_int];
        } else {
          left_mm = sensor_present[0] ? 255 : 0;
        }
        break;
      case 2:  // IC1 -> front-left (previous cycle)
        if (sensor_present[1] && laser_ch_enabled[2]) {
          read_meas(1);
          save_channel(2);
          front_left_mm = (Maxval_X < valid_min[2]) ? 255 : lut_front_l[Maxpos_X_int];
        } else {
          front_left_mm = sensor_present[1] ? 255 : 0;
        }
        break;
      case 3:  // IC2 -> right
        if (sensor_present[2] && laser_ch_enabled[3]) {
          read_meas(2);
          save_channel(3);
          right_mm = (Maxval_X < valid_min[3]) ? 255 : lut_right[Maxpos_X_int];
        } else {
          right_mm = sensor_present[2] ? 255 : 0;
        }
        break;
    }

    if (fire) {
      // Pad out the remainder of the integration window.
      while (micros() - t0 < integ) {}
      digitalWriteFast(MST[ic], HIGH);
      digitalWriteFast(laser[il_laser[s]], 0);
      delayMicroseconds(5);  // MST high hold before any neighbour work
    }
  }

  Serial5.write((uint8_t)left_mm);
  Serial5.write((uint8_t)front_right_mm);
  Serial5.write((uint8_t)front_left_mm);
  Serial5.write((uint8_t)right_mm);
  Serial5.write(0x0D);
  lasers_all_off();
}

// =============================================================================
// USB serial calibration interface
// =============================================================================
//
// Independent of the main MCU — uses the Teensy's own USB serial port.
// Connect a terminal (115200 baud) or run the sensor calibration GUI.
//
// Commands (send as ASCII):
//   r  — raw stream: CSV  left_px,left_sig,left_mm, fr_px,fr_sig,fr_mm, fl_px,fl_sig,fl_mm, right_px,right_sig,right_mm
//   d  — dist stream: CSV left_mm,fr_mm,fl_mm,right_mm   (ch0=left wall, ch3=right wall; beams cross)
//   s  — single shot (raw CSV, one line)
//   q  — quiet (stop streaming)
//   R  — re-detect sensor ICs (useful after power cycling a sensor board)
//   h  — help
//
// Byte encoding (same as packet): 0=IC error, 1–254=mm, 255=out-of-range
//
// In stream mode (r or d) the sub-MCU self-triggers at 20 Hz when no Serial5
// triggers are present — run standalone, no main MCU needed.
// =============================================================================

// (USB_CAL_* and usb_cal_mode are declared up near the SPI helpers so the
// measurement functions can read the mode -- see there.)
static uint32_t usb_log_last_us = 0;
#define USB_LOG_INTERVAL_US  50000UL   // 20 Hz GUI update cap
static uint32_t loop_us = 0;           // last measurement cycle duration

// Raw USB output: 10 values per channel × 4 channels = 40 fields
// frame, x_int, x_dec, y_int, y_dec, sum_u, sum_l, sig_x, sig_y, mm
static void usb_print_header() {
  if (usb_cal_mode == USB_CAL_RAW) {
    Serial.println("# 41 fields: [frame,x_int,x_dec,y_int,y_dec,sum_u,sum_l,sig_x,sig_y,mm] x4 channels (left,fr,fl,right), loop_us");
  } else {
    Serial.println("# 5 fields: left_mm,fr_mm,fl_mm,right_mm,loop_us");
  }
  Serial.println("# 0=IC error  255=out-of-range");
}

// usb_log: send one line to the GUI, throttled to USB_LOG_INTERVAL_US.
// Measurement timing is unaffected — this is only called after measure_and_send().
static void usb_log() {
  if (usb_cal_mode == USB_CAL_OFF) return;
  uint32_t now = micros();
  if (now - usb_log_last_us < USB_LOG_INTERVAL_US) return;
  usb_log_last_us = now;
  if (usb_cal_mode == USB_CAL_RAW) {
    // 41 fields: 40 sensor fields + loop_us (measurement cycle time in µs)
    Serial.printf(
      "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
      "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
      "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
      "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
      "%lu\n",
      ch_frame[0],ch_x_int[0],ch_x_dec[0],ch_y_int[0],ch_y_dec[0],ch_sum_u[0],ch_sum_l[0],ch_sig_x[0],ch_sig_y[0],(uint8_t)left_mm,
      ch_frame[1],ch_x_int[1],ch_x_dec[1],ch_y_int[1],ch_y_dec[1],ch_sum_u[1],ch_sum_l[1],ch_sig_x[1],ch_sig_y[1],(uint8_t)front_right_mm,
      ch_frame[2],ch_x_int[2],ch_x_dec[2],ch_y_int[2],ch_y_dec[2],ch_sum_u[2],ch_sum_l[2],ch_sig_x[2],ch_sig_y[2],(uint8_t)front_left_mm,
      ch_frame[3],ch_x_int[3],ch_x_dec[3],ch_y_int[3],ch_y_dec[3],ch_sum_u[3],ch_sum_l[3],ch_sig_x[3],ch_sig_y[3],(uint8_t)right_mm,
      loop_us);
  } else {
    Serial.printf("%d,%d,%d,%d,%lu\n",
      (uint8_t)left_mm,(uint8_t)front_right_mm,(uint8_t)front_left_mm,(uint8_t)right_mm,
      loop_us);
  }
}

// =============================================================================
// USB command handler
// Commands are newline-terminated; single-char commands also work without '\n'.
// Parameter commands:
//   is<val>      integration time 1-2047 µs for the SIDE pair  (ch0 + ch3)
//   if<val>      integration time 1-2047 µs for the FRONT pair (ch1 + ch2)
//   i<ch><val>   integration time for ONE channel, ch 0..3 =
//                  0 left, 1 front_right, 2 front_left, 3 right
//   i            report all four (GUI readback)
//   b <val>      blanking 0-255 (global)
//   g<ic> <val>  gain for IC 0-2, e.g. "g0 3"
//   a<ic> <val>  ADC  for IC 0-2, e.g. "a0 245"
//   c<ic> <val>  calc_area for IC 0-2 (0=5px 1=9px 2=19px)
//   t<ic> <val>  threshold for IC 0-2, 0-255
//   T            re-run the boot threshold check (dark read vs the configured
//                side thresholds; reports, never sets)
//   x <val>      Xpixel_start 0-255 (global)
//   n <val>      readout_pixels 1-255 (global)
// =============================================================================

static void process_usb_cmd(const char *line) {
  if (!line || !line[0]) return;
  switch (line[0]) {
    case 'r':
      lasers_enabled  = true;
      interlaced      = false;
      usb_cal_mode    = USB_CAL_RAW;
      usb_log_last_us = 0;   // send first line immediately
      Serial.println("# Raw stream (41-field) — q to stop");
      usb_print_header();
      break;
    case 'd':
      lasers_enabled  = true;
      interlaced      = false;
      usb_cal_mode    = USB_CAL_DIST;
      usb_log_last_us = 0;
      Serial.println("# Dist stream (4-field) — q to stop");
      usb_print_header();
      break;
    case 'I':
      // Interlaced test stream: same dist output, interlaced cycle. Use
      // 'r' (raw, 41st field = loop_us) to compare cycle times: classic
      // then "I" then compare. 'q' stops and clears the flavour.
      lasers_enabled  = true;
      interlaced      = true;
      usb_cal_mode    = USB_CAL_DIST;
      usb_log_last_us = 0;
      Serial.println("# INTERLACED dist stream — q to stop, d for classic");
      usb_print_header();
      break;
    case 's': {
      uint8_t saved  = usb_cal_mode;
      bool    l_save = lasers_enabled;
      usb_cal_mode   = USB_CAL_RAW;
      lasers_enabled = true;
      if (saved == USB_CAL_OFF) usb_print_header();
      uint32_t t0 = micros();
      measure_and_send();
      loop_us = micros() - t0;
      usb_log_last_us = 0;   // force immediate output for single-shot
      usb_log();
      usb_cal_mode   = saved;
      lasers_enabled = l_save;
      break;
    }
    case 'q':
      lasers_enabled = false;
      usb_cal_mode   = USB_CAL_OFF;
      interlaced     = false;
      lasers_all_off();
      Serial.println("# Stream off — lasers disabled");
      break;
    case 'R':
      Serial.println("# Re-detecting sensor ICs...");
      sensors_detect_all();
      break;
    case 'F':
      sensor_present[0] = sensor_present[1] = sensor_present[2] = true;
      Serial.println("# All ICs forced present");
      break;
    case 'L': {
      uint8_t ch = (uint8_t)(line[1] - '0');
      if (ch < 4) {
        laser_ch_enabled[ch] = (line[2] == '1');
        Serial.printf("# laser ch %d %s\n", ch, laser_ch_enabled[ch] ? "enabled" : "disabled");
      }
      break;
    }
    case 'W': {
      // W<ch>,<px>,<mm>  — set single LUT entry at runtime
      uint8_t ch;
      int px, mm_val;
      if (sscanf(line + 1, "%hhu,%d,%d", &ch, &px, &mm_val) == 3 && ch < 4 && px < 256) {
        uint8_t *lut = (ch == 0) ? lut_left :
                       (ch == 1) ? lut_front_r :
                       (ch == 2) ? lut_front_l : lut_right;
        lut[(uint8_t)px] = (uint8_t)constrain(mm_val, 0, 255);
      }
      break;
    }
    // 'V<ch>' -- dump the LIVE LUT for one channel back over USB.
    //
    // The GUI could WRITE a LUT (256 'W' commands) but never READ one:
    // CMD_DUMP_LUT exists only on the Serial5 link to the main MCU. So the GUI
    // plotted its own locally-built table while the live mm came from whatever
    // actually landed in the sub-MCU, and a partial/dropped send looked exactly
    // like a good LUT that reads wrong. This closes that loop -- diff this
    // against the plot before trusting a calibration.
    case 'V': {
      if (line[1] < '0' || line[1] > '3') {
        Serial.println("# usage: V<ch>  (0 left, 1 front_right, 2 front_left, 3 right)");
        break;
      }
      uint8_t ch = (uint8_t)(line[1] - '0');
      const uint8_t *lut = (ch == 0) ? lut_left :
                           (ch == 1) ? lut_front_r :
                           (ch == 2) ? lut_front_l : lut_right;
      uint8_t sum = 0;
      int lo = -1, hi = -1, mn = 255, mx = 0;
      for (int i = 0; i < 256; i++) {
        sum += lut[i];
        if (lut[i] != 255) {
          if (lo < 0) lo = i;
          hi = i;
          if (lut[i] < mn) mn = lut[i];
          if (lut[i] > mx) mx = lut[i];
        }
      }
      Serial.printf("# lut%d (%s) begin\n", ch, INTEG_CH_NAME[ch]);
      for (int base = 0; base < 256; base += 16) {
        Serial.printf("# lut%d[%3d]:", ch, base);
        for (int k = 0; k < 16; k++) Serial.printf(" %3d", lut[base + k]);
        Serial.println();
      }
      if (lo < 0) Serial.printf("# lut%d end sum=%u  EMPTY (all 255 -- send failed?)\n",
                                ch, sum);
      else Serial.printf("# lut%d end sum=%u  px %d..%d -> %d..%d mm\n",
                         ch, sum, lo, hi, mn, mx);
      break;
    }
    case 'h': case '?':
      Serial.println("# r=raw40  d=dist4  I=interlaced-dist  s=single  q=stop  R=redetect  F=force  N=dark/noise-floor  T=threshold check  V<ch>=dump LUT  h=help");
      Serial.println("# M<ch><val>=validity gate (why a good LUT reads OUT OF RANGE), M=report\n"
      "# params: is<integ_side>  if<integ_front>  i<ch><integ> (ch 0..3, one channel)  i=report  b<blank>  g<ic><gain>  a<ic><adc>  c<ic><area>  t<ic><thr>  x<xstart>  n<npix>");
      Serial.println("# L<ch><0/1>=laser enable  W<ch>,<px>,<mm>=set LUT entry  V<ch>=read LUT back");
      Serial.printf ("# current: integ ch0=%d ch1=%d ch2=%d ch3=%d (left,front_r,front_l,right) blank=%d xstart=%d npix=%d\n",
                     integration_time_ch[0], integration_time_ch[1],
                     integration_time_ch[2], integration_time_ch[3],
                     blanking_val, Xpixel_start_val, readout_pixels);
      for (uint8_t k = 0; k < 3; k++)
        Serial.printf("# IC%d: gain=0x%02X adc=0x%02X calc_area=%d threshold=%d\n",
                      k, gain_val[k], adc_val[k], calc_area_val[k], threshold_val[k]);
      break;
    // ── Parameter update commands ────────────────────────────────────────────
    // is<v> / if<v> set the side pair / front pair together (unchanged).
    // i<ch><v> sets ONE channel, ch 0..3 = left, front_right, front_left,
    // right -- the close-range saturation sweep needs one beam moved while the
    // other three hold, so the interlaced cadence and the comparison stay
    // intact. 'i' alone reports all four.
    case 'i': {
      if (line[1] == 's') {
        uint16_t v = (uint16_t)constrain(atoi(line + 2), 1, 2047);
        integration_time_ch[0] = integration_time_ch[3] = v;
        Serial.printf("# integration_time_side=%d (ch0,ch3)\n", v);
      } else if (line[1] == 'f') {
        uint16_t v = (uint16_t)constrain(atoi(line + 2), 1, 2047);
        integration_time_ch[1] = integration_time_ch[2] = v;
        Serial.printf("# integration_time_front=%d (ch1,ch2)\n", v);
      } else if (line[1] >= '0' && line[1] <= '3') {
        uint8_t ch = (uint8_t)(line[1] - '0');
        integration_time_ch[ch] = (uint16_t)constrain(atoi(line + 2), 1, 2047);
        Serial.printf("# integ_ch%d(%s)=%d\n", ch, INTEG_CH_NAME[ch],
                      integration_time_ch[ch]);
      } else {
        // bare 'i' -- report, so the GUI can read back what is live
        Serial.printf("# integ: ch0(left)=%d ch1(front_right)=%d "
                      "ch2(front_left)=%d ch3(right)=%d\n",
                      integration_time_ch[0], integration_time_ch[1],
                      integration_time_ch[2], integration_time_ch[3]);
      }
      break;
    }
    // 'N' -- dark read on all four channels: the NOISE FLOOR each threshold
    // must clear. Run it with the mouse where it will actually drive, under
    // the lighting it will actually see; a filtered channel should read far
    // lower than an unfiltered one.
    case 'N': {
      Serial.println("# dark read (lasers off) -- threshold should sit well above these");
      for (uint8_t ch = 0; ch < 4; ch++) {
        uint32_t v = 0, pos = 0;
        measure_dark(ch, &v, &pos);
        uint8_t ic = ch_ic[ch];
        Serial.printf("#   ch%d %-11s IC%d  integ %4d us  Maxval %4lu  Maxpos %3lu"
                      "  gate %3d  margin %+d\n",
                      ch, INTEG_CH_NAME[ch], ic, integration_time_ch[ch],
                      (unsigned long)v, (unsigned long)pos,
                      valid_min[ch], (int)valid_min[ch] - (int)v);
      }
      break;
    }
    // 'T' -- re-run the boot threshold check. Same thing setup() does:
    // dark-read every channel and judge the side thresholds against the floor.
    // Reports only; it does not change a threshold. Worth running again
    // whenever the lighting changes -- the lasers stay off, so this measures
    // the room and not the target. 't<ic> <val>' is what actually sets one.
    case 'T':
      threshold_selftest(true);
      break;
    // 'M<ch><val>' -- SOFTWARE validity gate for one channel; 'M' reports all.
    // This is the "do I trust this spot" amplitude test, NOT the sensor's own
    // threshold_val ('t'). If a channel reads OUT OF RANGE while its LUT plot
    // looks correct, this is almost always the reason.
    case 'M': {
      if (line[1] >= '0' && line[1] <= '3') {
        uint8_t ch = (uint8_t)(line[1] - '0');
        valid_min[ch] = (uint8_t)constrain(atoi(line + 2), 0, 255);
        Serial.printf("# valid_min ch%d(%s) = %d\n", ch, INTEG_CH_NAME[ch],
                      valid_min[ch]);
      } else {
        Serial.printf("# valid_min: ch0(left)=%d ch1(front_right)=%d "
                      "ch2(front_left)=%d ch3(right)=%d\n",
                      valid_min[0], valid_min[1], valid_min[2], valid_min[3]);
      }
      break;
    }
    case 'b': {
      blanking_val = (uint8_t)constrain(atoi(line + 1), 0, 255);
      apply_params(3);
      Serial.printf("# blanking=%d\n", blanking_val);
      break;
    }
    case 'g': {
      uint8_t ic = (uint8_t)(line[1] - '0');
      if (ic < 3) {
        gain_val[ic] = (uint8_t)constrain(atoi(line + 2), 0, 255);
        apply_params(ic);
        Serial.printf("# gain[%d]=0x%02X\n", ic, gain_val[ic]);
      }
      break;
    }
    case 'a': {
      uint8_t ic = (uint8_t)(line[1] - '0');
      if (ic < 3) {
        adc_val[ic] = (uint8_t)constrain(atoi(line + 2), 0, 255);
        apply_params(ic);
        Serial.printf("# adc[%d]=0x%02X\n", ic, adc_val[ic]);
      }
      break;
    }
    case 'c': {
      uint8_t ic = (uint8_t)(line[1] - '0');
      if (ic < 3) {
        calc_area_val[ic] = (uint8_t)constrain(atoi(line + 2), 0, 2);
        apply_params(ic);
        Serial.printf("# calc_area[%d]=%d\n", ic, calc_area_val[ic]);
      }
      break;
    }
    case 't': {
      uint8_t ic = (uint8_t)(line[1] - '0');
      if (ic < 3) {
        threshold_val[ic] = (uint8_t)constrain(atoi(line + 2), 0, 255);
        apply_params(ic);
        Serial.printf("# threshold[%d]=%d\n", ic, threshold_val[ic]);
      }
      break;
    }
    case 'x': {
      Xpixel_start_val = (uint8_t)constrain(atoi(line + 1), 0, 254);
      apply_params(3);
      Serial.printf("# Xpixel_start=%d\n", Xpixel_start_val);
      break;
    }
    case 'n': {
      readout_pixels = (uint8_t)constrain(atoi(line + 1), 1, 255);
      apply_params(3);
      Serial.printf("# readout_pixels=%d\n", readout_pixels);
      break;
    }
    default:
      break;
  }
}

// Single-char commands that don't need a newline terminator.
static const char SINGLE_CMDS[] = "rqdsRFh?I";

static void handle_usb_cal() {
  static char    line_buf[64];
  static uint8_t line_len = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      line_buf[line_len] = '\0';
      if (line_len > 0) process_usb_cmd(line_buf);
      line_len = 0;
      return;
    }

    if (line_len < 63) line_buf[line_len++] = c;

    // Dispatch single-char commands immediately (no newline needed).
    if (line_len == 1) {
      bool is_single = false;
      for (uint8_t k = 0; SINGLE_CMDS[k]; k++)
        if (c == SINGLE_CMDS[k]) { is_single = true; break; }
      if (is_single) {
        line_buf[1] = '\0';
        process_usb_cmd(line_buf);
        line_len = 0;
      }
    }
  }
}

// =============================================================================
// setup
// =============================================================================

// Extra Serial5 RX buffer so a 259-byte CMD_LOAD_LUT burst can never
// overflow the default 64-byte FIFO in the window before readBytes drains
// it (auto-cal LUT upload, 2026-08-08).
static uint8_t serial5_rx_extra[512];

void setup() {
  Serial.begin(115200);
  Serial5.begin(460800);   // must match SENSOR_BAUD in main MCU config.h
  Serial5.addMemoryForRead(serial5_rx_extra, sizeof(serial5_rx_extra));

  
  
  pinMode(SPI_CS_1,   OUTPUT);
  pinMode(SPI_CS_2,   OUTPUT);
  pinMode(SPI_CS_3,   OUTPUT);
  pinMode(MST_1,      OUTPUT);
  pinMode(MST_2,      OUTPUT);
  pinMode(MST_3,      OUTPUT);
  pinMode(TG_RESET,   OUTPUT);
  pinMode(SPI_RESET,  OUTPUT);
  pinMode(MISO_MUX_1, OUTPUT);
  pinMode(MISO_MUX_2, OUTPUT);
  pinMode(MISO_MUX_3, OUTPUT);
  pinMode(laser_1,    OUTPUT);
  pinMode(laser_2,    OUTPUT);
  pinMode(laser_3,    OUTPUT);
  pinMode(laser_4,    OUTPUT);

  digitalWrite(SPI_CS_1,   HIGH);
  digitalWrite(SPI_CS_2,   HIGH);
  digitalWrite(SPI_CS_3,   HIGH);
  digitalWrite(MST_1,      HIGH);
  digitalWrite(MST_2,      HIGH);
  digitalWrite(MST_3,      HIGH);
  digitalWrite(MISO_MUX_1, LOW);
  digitalWrite(MISO_MUX_2, LOW);
  digitalWrite(MISO_MUX_3, LOW);
  digitalWrite(TG_RESET,   LOW);
  digitalWrite(SPI_RESET,  LOW);
  lasers_all_off();

  // Reset (datasheet Fig 1-1): SPI_RSTB low pulse, then TG_Reset high >=12 MCLK
  // cycles; then configure + detect. A clean power-on reset (adequate off-time
  // between power cycles, or a supply bleed) is what the parts need -- the long
  // "warm-up" saga was a mis-diagnosis of a hardware power-on-reset issue.
  digitalWrite(SPI_RESET, LOW);   delayMicroseconds(2);
  digitalWrite(SPI_RESET, HIGH);  delayMicroseconds(2);
  digitalWrite(TG_RESET,  HIGH);  delayMicroseconds(5);
  digitalWrite(TG_RESET,  LOW);   delayMicroseconds(2);
  SPI.begin();
  delay(1);
  S15366_init();
  Serial.println("Sensor sub-MCU -- detecting ICs...");
  sensors_detect_all();
  // Measure the noise floor as it is RIGHT NOW, in this room, and check the
  // compiled side thresholds against it. Reports only -- it does not set a
  // threshold (that auto-set version was reverted the same day it was built).
  // 'T' re-runs it; the main MCU pulls the result with CMD_THRESH_REP.
  threshold_selftest(true);
  Serial.println("Ready. Serial5=main MCU triggers. USB=calibration (h for help).");
}

// =============================================================================
// Command bytes
// =============================================================================

#define CMD_TRIGGER   0x55   // single measurement (triggered / search mode)
#define CMD_FREE_RUN  0x56   // start continuous measurement (fast-run mode)
#define CMD_STOP      0x57   // stop free-run, return to triggered mode
#define CMD_FREE_RUN_IL 0x58 // interlaced free-run (test mode, ~580 Hz)
#define CMD_FREE_RUN_RAW 0x59 // free-run + append raw px/sig (auto-cal)
#define CMD_LOAD_LUT   0x5A   // main uploads a LUT channel: [ch][256][sum8]
#define CMD_PING       0x5B   // readiness probe: echo the byte back
#define CMD_DUMP_LUT   0x5C   // main requests a channel: [ch] -> [256][sum8]
// Boot threshold self-test result -> main MCU. Exists because the mouse is
// ALWAYS powered off before its flight log is read, so anything that only
// reaches this MCU's USB console is invisible on a real run. Optional payload
// byte 1 re-runs the test first (the room may have changed since power-on).
//   reply: [4 dark][4 threshold][4 valid_min][flags][sum8]  = 14 bytes
#define CMD_THRESH_REP 0x5D
#define THRESH_REP_BYTES 14
#define LUT_ACK        0xA5   // CMD_LOAD_LUT accepted
#define LUT_NAK        0xE5   // CMD_LOAD_LUT bad checksum

static bool free_running = false;


// =============================================================================
// loop
// =============================================================================

void loop() {
  handle_usb_cal();   // always check for USB calibration commands

  if (free_running) {
    // Continuous mode: fire immediately, check for CMD_STOP between cycles.
    // No duration limit: the old ≤5 s thermal cap was disproven by bench
    // testing (2026-07-19); interlaced duty is ~4x lower still.
    uint32_t t0 = micros();
    if (interlaced) measure_cycle_interlaced();
    else            measure_and_send();
    loop_us = micros() - t0;
    usb_log();
    while (Serial5.available()) {
      uint8_t c = Serial5.read();
      if (c == CMD_STOP) {
        free_running   = false;
        interlaced     = false;
        g_send_raw     = false;
        lasers_enabled = false;   // disarm + ensure lasers off on fast-run exit
        lasers_all_off();
        Serial.println("# Sensor: free-run stopped");
        return;
      }
    }
    return;
  }

  // Triggered mode: handle Serial5 command if present
  if (Serial5.available()) {
    uint8_t cmd = Serial5.read();
    switch (cmd) {
      case CMD_TRIGGER: {
        lasers_enabled = true;   // main MCU asked for a measurement → arm lasers
        uint32_t t0 = micros();
        measure_and_send();
        loop_us = micros() - t0;
        usb_log();
        break;
      }
      case CMD_FREE_RUN:
        // No config re-apply here: it's redundant (boot already configures the
        // ICs) and, if 'n' starts early, it's an extra cold-period SPI write that
        // can stall the sensor. Just arm + stream; measure_and_send() gates the
        // actual measurement on warm-up.
        lasers_enabled = true;   // arm lasers for continuous fast-run measurement
        free_running   = true;
        interlaced     = false;
        Serial.println("# Sensor: free-run started");
        break;
      case CMD_FREE_RUN_IL:
        lasers_enabled = true;
        free_running   = true;
        interlaced     = true;
        Serial.println("# Sensor: interlaced free-run started");
        break;
      case CMD_FREE_RUN_RAW:
        // Auto-cal: classic (non-interlaced) free-run with raw px/sig
        // appended to each packet (13-byte packet). save_channel needs
        // the classic path's per-channel reads.
        lasers_enabled = true;
        free_running   = true;
        interlaced     = false;
        g_send_raw     = true;
        Serial.println("# Sensor: raw-px free-run started");
        break;
      case CMD_STOP:
        lasers_enabled = false;  // disarm + ensure all lasers off
        g_send_raw     = false;
        lasers_all_off();
        break;
      case CMD_PING:
        Serial5.write((uint8_t)CMD_PING);   // readiness echo
        break;
      case CMD_THRESH_REP: {
        // Followed by one byte: non-zero re-runs the self-test first. Quiet on
        // the USB console when it does, because the main MCU may ask for this
        // mid-session and the GUI is parsing that stream.
        uint8_t again = 0;
        Serial5.setTimeout(50);
        if (Serial5.readBytes(&again, 1) == 1 && again) threshold_selftest(false);
        uint8_t rep[THRESH_REP_BYTES];
        uint8_t sum = 0;
        for (uint8_t i = 0; i < 4; i++) rep[i] = boot_dark[i];
        for (uint8_t i = 0; i < 4; i++) rep[4 + i] = boot_thresh[i];
        for (uint8_t i = 0; i < 4; i++) rep[8 + i] = valid_min[i];
        rep[12] = boot_flags;
        for (uint8_t i = 0; i < THRESH_REP_BYTES - 1; i++) sum += rep[i];
        rep[13] = sum;
        Serial5.write(rep, THRESH_REP_BYTES);
        break;
      }
      case CMD_DUMP_LUT: {
        // Main pulls the LIVE LUT (e.g. GUI-calibrated) back to persist in
        // its FRAM. Followed by [ch]; reply [256 mm bytes][sum8].
        uint8_t ch = 255;
        Serial5.setTimeout(200);
        if (Serial5.readBytes(&ch, 1) == 1 && ch < 4) {
          const uint8_t *lut = (ch == 0) ? lut_left :
                               (ch == 1) ? lut_front_r :
                               (ch == 2) ? lut_front_l : lut_right;
          uint8_t sum = ch;
          for (int i = 0; i < 256; i++) sum += lut[i];
          Serial5.write(lut, 256);
          Serial5.write(sum);
        }
        break;
      }
      case CMD_LOAD_LUT: {
        // Followed by [ch][256 mm bytes][sum8]. Block-read the payload,
        // verify the checksum, overwrite the channel's LUT in RAM, ACK.
        uint8_t buf[258];
        Serial5.setTimeout(200);
        size_t got = Serial5.readBytes(buf, sizeof(buf));
        uint8_t ch = buf[0];
        uint8_t sum = ch;
        for (int i = 0; i < 256; i++) sum += buf[1 + i];
        if (got == sizeof(buf) && ch < 4 && sum == buf[257]) {
          uint8_t *lut = (ch == 0) ? lut_left :
                         (ch == 1) ? lut_front_r :
                         (ch == 2) ? lut_front_l : lut_right;
          memcpy(lut, buf + 1, 256);
          Serial5.write((uint8_t)LUT_ACK);
          Serial.printf("# Sensor: LUT ch %d loaded (main-MCU FRAM)\n", ch);
        } else {
          Serial5.write((uint8_t)LUT_NAK);
          Serial.println("# Sensor: LUT load rejected (checksum/length)");
        }
        break;
      }
      default:
        break;   // ignore unknown bytes
    }
    return;
  }

  // No Serial5 activity -- USB self-trigger for standalone calibration streaming.
  // Measurement runs as fast as possible; usb_log() throttles GUI output internally.
  if (usb_cal_mode != USB_CAL_OFF) {
    uint32_t t0 = micros();
    if (interlaced) measure_cycle_interlaced();
    else            measure_and_send();
    loop_us = micros() - t0;
    usb_log();
  }
}
