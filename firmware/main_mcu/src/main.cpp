// Main MCU firmware entry point (Teensy 4.0).
//
// Boot, the main loop, and nothing else. Everything with substance lives in
// a library under lib/, per the layering in
// docs/firmware_architecture_adr.md (HAL -> Drivers -> Control ->
// Application):
//
//   lib/HAL         pins, buses, timers
//   lib/Drivers     IMU, encoders, motors, FRAM, wall-sensor sub-MCU link
//   lib/Control     500 Hz ISR: fusion, odometry, wheel loops, maneuvers
//   lib/Maze        wall model + flood fill (pure logic, host-tested)
//   lib/Application search executor, FRAM params/LUT/flight log, console,
//                   bench harnesses
//
// This file was 3200 lines in TW3_V1 and had absorbed the console, the
// parameter UI, and seventeen calibration harnesses. Keeping it to boot +
// loop is a deliberate constraint of the V2 baseline -- see
// docs/V2_BASELINE.md.
//
// Runtime shape: a 500 Hz hardware IntervalTimer ISR (Control.h) drains the
// ICM-45686 FIFO and runs Madgwick fusion, wheel odometry and the wheel
// velocity loops. The main loop below is everything that is NOT time
// critical: console input, the bench harnesses, the search executor's poll,
// and a 10 Hz status line. The status LEDs alternate as a liveness check;
// LED2 solid means IMU or control-timer init failed.
//
// Build/upload: `pio run -t upload` (env:teensy40 in platformio.ini).

#include <Arduino.h>

#include "Bench.h"
#include "Console.h"
#include "Control.h"
#include "Drivers.h"
#include "EncoderTiming.h"
#include "FlightLog.h"
#include "GoffTable.h"
#include "Vbat.h"
#include "HAL.h"
#include "LoopSweep.h"
#include "RouteRun.h"
#include "Params.h"
#include "Search.h"
#include "Explore.h"
#include "RunLadder.h"
#include "SensorLut.h"

static bool imu_ok = false;

void setup() {
    hal_init();
    hal_spi_init();
    hal_imu_init();
    Serial.begin(115200);
    hal_uart_radio_init();  // ESP32 link: console mirror for ground tests
    sensors_init();         // Serial2 link to the wall-sensor sub-MCU

    encoders_init();
    // Edge-timed wheel velocity (EncoderTiming.h): XBAR outputs 0..3, GPT1,
    // four DMA channels. After the ENC modules are up (same XBAR inputs).
    enc_timing_init();
    control_set_wheel_velocity_hook([](int32_t dl, int32_t dr, float *vl, float *vr, bool *ok) {
        enc_timing_tick(dl, dr);
        EncTimingWheel L = enc_timing_left(), R = enc_timing_right();
        *vl = L.v_mm_s; *vr = R.v_mm_s;
        *ok = enc_timing_ready();
    });
    motors_init();  // stays in standby until the self-test enables it
    vbat_init();    // battery sense on pin 23 (A9) -- see Vbat.h

    // FRAM + persisted gains, before the control ISR starts (FRAM access
    // masks interrupts; cheaper to do while there's nothing to mask).
    bool fram_ok = fram_init();
    ControlGains gains;
    bool params_loaded = params_load(gains);
    control_set_gains(gains);  // FRAM block if valid, else defaults
    bench_set_boot_status(fram_ok, params_loaded);
    flog_init();  // previous run's log header ('y' works pre-run)
    run_ladder_init();   // MR2..MR5 by run number, remembered across power cycles

    imu_ok = imu_init();
    if (imu_ok) {
        imu_ok = control_init();
    }

    search_set_printer(con_println);  // search executor console output
    explore_set_printer(con_println);  // the rolling search (Explore.h)
    search_goal_load();  // persisted practice-rig goal, if any (needs FRAM
                         // init above + the printer for its boot line)
    // The searched map, if one survived the last power cycle. AFTER the goal
    // load, because whether a restored map holds a PROVEN route is decided
    // against the current goal, not the one in force when it was saved.
    search_map_load();
    // The per-corner learned gate offsets that ride alongside that map
    // (GoffTable.h) -- learned at moderate speed, spent at race speed.
    goff_table_load();

    // The loop sweep. Its whole configuration is compiled in (LoopSweep.h),
    // so the workflow at the maze is flash -> place -> wave, with nothing
    // typed. Defaults FIRST, then the stored header: the loader needs the
    // compiled grid in hand to decide whether a stored session was flown
    // with these numbers and may therefore be resumed.
    sweep_set_printer(con_println);
    route_run_set_printer(con_println);
    sweep_load_defaults();
    sweep_resume_load();

    // Push any FRAM-stored sensor LUTs to the sub-MCU before arming / IMU
    // cal (bert, 2026-08-08). No-op if none stored -> sub keeps compiled
    // defaults. Blocking; the sub-MCU has had all of boot to come up.
    int lut_n = sensor_lut_upload_all();
    if (lut_n > 0)
        con_printf("sensor LUT: uploaded %d channel(s) from FRAM\n", lut_n);
    else if (lut_n < 0)
        con_println("sensor LUT: sub-MCU silent, using compiled defaults");

    // The sub-MCU's boot threshold check (bert, 2026-08-15). It dark-read
    // every channel at its own power-on -- lasers off, so it measured the
    // ROOM -- and compared the two SIDE thresholds against what it found.
    // Reported here because the sub-MCU's USB console is invisible on a real
    // run, and because neither failure announces itself as an error: too
    // high and the side beams stop returning at range, too low and they
    // return noise as a confident distance.
    bench_print_threshold_report(false);

    // Arm ONE mission on the hand start. Both the search and the sweep watch
    // the same front beam, so arming both would make a wave ambiguous --
    // which of two multi-minute autonomous runs did you just start?
    // SWEEP_ARM_AT_BOOT (LoopSweep.h) picks, at compile time, which campaign
    // this firmware is for.
    if (SWEEP_ARM_AT_BOOT) {
        sweep_arm();
    } else if (route_run_available()) {
        // A PROVEN ROUTE IN FRAM CHANGES WHAT A WAVE MEANS. The map survives
        // a power cycle, so the workflow at the maze is: wave (it searches),
        // power cycle, wave (it flies what it found). "Proven" is the
        // PESSIMISTIC flood -- unknown edges count as walls -- so a map with
        // a hole in it anywhere on every path to the goal falls through to a
        // search rather than flying through an edge nobody has looked at.
        //
        // 'h' forces the search back onto the hand start when the route is
        // not what is wanted (a re-search, a moved wall).
        route_run_arm();
    } else {
        // The race-start protocol. Missions end IDLE, because the map
        // survives a power cycle and the thing to do between runs is switch
        // the mouse off and clean the wheels.
        search_arm_handstart();
    }
}

void loop() {
    static bool led_phase = false;
    static uint32_t last_print_ms = 0;

    // Console commands, line-based (end with Enter), from USB or radio.
    // See docs/COMMANDS.md for the full reference.
    static char line_buf[32];
    static size_t line_len = 0;
    auto feed = [&](char c) {
        if (c == '\r' || c == '\n') {
            line_buf[line_len] = '\0';
            line_len = 0;
            bench_handle_line(line_buf);
        } else if (line_len < sizeof(line_buf) - 1) {
            line_buf[line_len++] = c;
        }
    };
    while (Serial.available()) feed((char)Serial.read());
    for (int c = hal_radio_read(); c >= 0; c = hal_radio_read()) feed((char)c);

    hal_radio_poll();   // drain the console ring; see HAL.h
    vbat_poll();        // battery sense refresh (10 Hz, main-loop only)
    control_set_vbat_mv(vbat_mv());   // the duty compensation (Control.h, vref)
    bench_poll();
    search_poll();
    explore_poll();
    route_run_poll();
    // THE FRONT GOVERNOR (Control.h): whoever is driving, the latest front
    // beams bound the commanded speed so the mouse never closes inside
    // 60 mm of a wall in its face. The missions above have polled the
    // sensors; when none is running the packet ages out and the cap lifts.
    {
        WallRanges w = sensors_latest();
        control_front_guard_feed(w.front_left_mm, w.front_right_mm, sensors_ms_since_packet());
    }

    if (!imu_ok) {
        // IMU or timer init failed: LED2 solid, keep reporting.
        hal_set_led2(true);
        if (millis() - last_print_ms >= 1000) {
            last_print_ms = millis();
            con_println("IMU init FAILED (WHO_AM_I/SPI or timer) -- check wiring");
        }
        return;
    }

    // Harnesses and missions own the serial output while they run (the 'v'
    // test streams CSV for live plotting -- the status line would corrupt
    // it). The raw-px monitor ('R') likewise owns the console.
    if (bench_busy()) return;

    if (millis() - last_print_ms >= 100) {
        last_print_ms = millis();
        led_phase = !led_phase;
        hal_set_led1(led_phase);
        hal_set_led2(!led_phase);

        ControlImuStatus s = control_imu_status();
        ControlHeading h = control_heading();
        ControlOdometry o = control_odometry();
        if (h.ready) {
            if (!bench_tick_enabled()) return;   // 'tick' turns it on
            con_printf(
                "tick=%lu | yaw % 8.2f pitch % 6.2f roll % 6.2f | "
                "L % 8.1fmm R % 8.1fmm v % 7.1fmm/s | %.1fC | %.2fV\n",
                s.ticks, h.yaw_deg, h.pitch_deg, h.roll_deg,
                o.left_mm, o.right_mm, o.vel_mm_s,
                imu_temp_c(s.latest.temp_raw),
                (double)(vbat_mv() / 1000.0f));
        } else {
            // Gyro bias calibration in progress -- keep the mouse still.
            con_printf(
                "tick=%lu samples=%lu batch=%u | calibrating gyro bias "
                "(hold still) | accel[g] % .3f % .3f % .3f\n",
                s.ticks, s.total_samples, s.last_batch,
                imu_accel_g(s.latest.ax), imu_accel_g(s.latest.ay),
                imu_accel_g(s.latest.az));
        }
    }
}
