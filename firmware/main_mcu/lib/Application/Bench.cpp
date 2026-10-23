#include "Bench.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Console.h"
#include "Control.h"
#include "Drivers.h"
#include "EncoderTiming.h"
#include "RunLadder.h"
#include "FlightLog.h"
#include "GoffTable.h"
#include "HAL.h"
#include "Maze.h"
#include "Vbat.h"
#include "Params.h"
#include "LapReport.h"
#include "LoopSweep.h"
#include "Path.h"
#include "Profile.h"
#include "Route.h"
#include "RouteRun.h"
#include "Search.h"
#include "Explore.h"
#include "SensorLut.h"
#include "WallAlign.h"
#include "WallGate.h"
#include "Walls.h"

// Boot status, for the 'p' readout.
// The idle 10 Hz status line, off by default (see Bench.h, 'tick').
static bool tick_enabled = false;
static bool fram_ok = false;
static bool params_loaded = false;
void bench_set_boot_status(bool fram, bool loaded) {
    fram_ok = fram;
    params_loaded = loaded;
}
bool bench_params_from_fram() { return params_loaded; }

// -- Sub-MCU threshold check readout ------------------------------------------
// The sub-MCU dark-reads every channel at power-on and checks the two SIDE
// detection thresholds against the measured noise floor (bert, 2026-08-15). It
// reports; it does not set anything -- the thresholds are hand values.
//
// `rerun` re-measures first, which is what to use when the lighting has changed
// since the mouse was switched on. The lasers stay off for the measurement, so
// it reads the room and not the target.
//
// Printed at boot AND available as the 'thr' console command, because neither
// failure announces itself as an error: too high and the side beams just stop
// returning at range, too low and they return noise as a confident distance.
void bench_print_threshold_report(bool rerun) {
    SensorThresholds t;
    // A re-run costs four dark reads at up to 400 us, twice each, plus SPI --
    // under 10 ms of work, but the sub may be mid-packet, so be generous.
    if (!sensors_threshold_report(&t, rerun, rerun ? 600 : 300)) {
        con_println("threshold: sub-MCU did not answer (streaming? try 'x' first)");
        return;
    }
    static const char *NAME[4] = {"left", "front_r", "front_l", "right"};
    con_printf("threshold check%s:\n",
               (t.flags & SENSOR_THRESH_F_RAN) ? "" : " (DID NOT RUN)");
    for (int i = 0; i < 4; i++) {
        con_printf("  ch%d %-7s dark %3u  thresh %3u  margin %+4d  vmin %3u\n",
                   i, NAME[i], (unsigned)t.dark[i], (unsigned)t.thresh[i],
                   (int)t.thresh[i] - (int)t.dark[i], (unsigned)t.vmin[i]);
    }
    // UNDER first and on its own: it is the silent one. A threshold below the
    // noise floor does not produce errors, it produces confident wrong
    // distances -- and with the horizon active the mouse now steers at those.
    if (t.flags & SENSOR_THRESH_F_UNDER) {
        con_println("  WARNING: a side threshold is AT OR BELOW the measured "
                    "noise floor. The sensor will latch on");
        con_println("           noise and the LUT will report it as a real "
                    "distance. Raise it before flying.");
    } else if (t.flags & SENSOR_THRESH_F_THIN) {
        con_println("  note: a side threshold is under 20% clear of its noise "
                    "floor. A good side return is only");
        con_println("        ~24 counts with these optics, so expect marginal "
                    "returns to be gone.");
    }
}

// -- Motor direction self-test ------------------------------------------------
// Triggered by sending 'm' over serial (mouse on blocks, wheels free!).
// Pulses each wheel forward at low duty and checks the encoders, printing
// a verdict: flip MOTOR_*_SIGN if a wheel runs backwards, swap
// MOTOR_LEFT_CH/MOTOR_RIGHT_CH if the wrong wheel moves (see Motors.h).

static constexpr float MTEST_DUTY = 0.20f;       // ~1.5 V effective at 7.4 V
static constexpr uint32_t MTEST_DRIVE_MS = 1500;
static constexpr uint32_t MTEST_PAUSE_MS = 700;

static int mtest_phase = -1;  // -1 idle; 0 L drive, 1 L report, 2 R drive, 3 R report
static uint32_t mtest_t0 = 0;
static float mtest_l0 = 0, mtest_r0 = 0;

static void mtest_report(const char *wheel, float d_this, float d_other) {
    con_printf("%s FWD: this wheel %+.1f mm, other %+.1f mm -> ", wheel, d_this, d_other);
    if (d_this > 10.0f && fabsf(d_other) < 5.0f) {
        con_println("OK");
    } else if (d_this < -10.0f) {
        con_printf("runs BACKWARDS: flip MOTOR_%s_SIGN in Motors.h\n", wheel);
    } else if (fabsf(d_other) > 10.0f && fabsf(d_this) < 5.0f) {
        con_println("WRONG WHEEL moved: swap MOTOR_LEFT_CH/MOTOR_RIGHT_CH in Motors.h");
    } else {
        con_println("inconclusive -- wheels free? battery on?");
    }
}

static void motor_test_poll() {
    if (mtest_phase < 0) return;
    uint32_t elapsed = millis() - mtest_t0;
    ControlOdometry o = control_odometry();

    switch (mtest_phase) {
        case 0:  // left wheel driving
            if (elapsed >= MTEST_DRIVE_MS) {
                motors_stop();
                mtest_report("LEFT", o.left_mm - mtest_l0, o.right_mm - mtest_r0);
                mtest_phase = 1;
                mtest_t0 = millis();
            }
            break;
        case 1:  // pause, then start right wheel
            if (elapsed >= MTEST_PAUSE_MS) {
                mtest_l0 = o.left_mm;
                mtest_r0 = o.right_mm;
                motor_set_right(MTEST_DUTY);
                mtest_phase = 2;
                mtest_t0 = millis();
            }
            break;
        case 2:  // right wheel driving
            if (elapsed >= MTEST_DRIVE_MS) {
                motors_stop();
                mtest_report("RIGHT", o.right_mm - mtest_r0, o.left_mm - mtest_l0);
                mtest_phase = 3;
                mtest_t0 = millis();
            }
            break;
        case 3:  // done
            motors_enable(false);
            con_println("motor self-test complete");
            mtest_phase = -1;
            break;
    }
}

// -- Duty-sweep characterisation ---------------------------------------------
// Triggered by 's' over serial (mouse on blocks, wheels free!). Steps both
// wheels through increasing open-loop duty, measuring steady-state velocity
// per step from encoder distance. Prints a duty->velocity table, then fits
// v = m*duty + c per wheel (VMAX = m, FRICTION = -c/m) and applies the
// result to the runtime gains ('w' persists them to FRAM).
//
// NOTE: velocities are unloaded (on blocks), so VMAX is optimistic vs
// on-floor -- the PD trims the difference. 0.45 max duty = ~3.3 V on the
// 3 V motors, brief enough per step to be safe.

static constexpr float STEST_DUTY_STEP = 0.05f;
static constexpr float STEST_DUTY_MAX = 0.45f;
static constexpr int STEST_STEPS = 9;  // 0.05 .. 0.45
static constexpr uint32_t STEST_SETTLE_MS = 500;   // spin-up, discarded
static constexpr uint32_t STEST_MEASURE_MS = 500;  // averaging window
static constexpr float STEST_FIT_MIN_MM_S = 30.0f; // exclude stalled points

static int stest_step = -1;       // -1 idle; 0..STEST_STEPS-1 running
static bool stest_measuring = false;
static uint32_t stest_t0 = 0;
static float stest_l0 = 0, stest_r0 = 0;
static float stest_vl[STEST_STEPS], stest_vr[STEST_STEPS];

// Least-squares v = m*duty + c over the non-stalled points.
struct StestFit {
    bool ok;
    float vmax;      // = m
    float friction;  // = -c/m
};

static StestFit stest_fit(const float *v, const char *wheel) {
    float sd = 0, sv = 0, sdd = 0, sdv = 0;
    int n = 0;
    for (int i = 0; i < STEST_STEPS; i++) {
        if (v[i] < STEST_FIT_MIN_MM_S) continue;
        float d = (i + 1) * STEST_DUTY_STEP;
        sd += d; sv += v[i]; sdd += d * d; sdv += d * v[i];
        n++;
    }
    if (n < 2) {
        con_printf("%s: not enough moving points to fit -- wheels free?\n", wheel);
        return {false, 0, 0};
    }
    float m = (n * sdv - sd * sv) / (n * sdd - sd * sd);
    float c = (sv - m * sd) / n;
    con_printf("%s: v = %.0f*duty %+.0f  ->  VMAX %.0f mm/s, FRICTION %.4f\n",
                  wheel, m, c, m, -c / m);
    return {true, m, -c / m};
}

static void stest_poll() {
    if (stest_step < 0) return;
    uint32_t elapsed = millis() - stest_t0;
    ControlOdometry o = control_odometry();

    if (!stest_measuring) {
        if (elapsed < STEST_SETTLE_MS) return;
        stest_l0 = o.left_mm;   // settle done: open the measuring window
        stest_r0 = o.right_mm;
        stest_measuring = true;
        stest_t0 = millis();
        return;
    }

    if (elapsed < STEST_MEASURE_MS) return;
    float dt_s = elapsed * 0.001f;
    stest_vl[stest_step] = (o.left_mm - stest_l0) / dt_s;
    stest_vr[stest_step] = (o.right_mm - stest_r0) / dt_s;
    float duty = (stest_step + 1) * STEST_DUTY_STEP;
    con_printf("duty %.2f | L %7.1f mm/s | R %7.1f mm/s\n",
                  duty, stest_vl[stest_step], stest_vr[stest_step]);

    stest_step++;
    if (stest_step < STEST_STEPS) {
        duty = (stest_step + 1) * STEST_DUTY_STEP;
        motor_set_left(duty);
        motor_set_right(duty);
        stest_measuring = false;
        stest_t0 = millis();
        return;
    }

    motors_stop();
    motors_enable(false);
    stest_step = -1;
    con_println("-- fit --");
    StestFit fl = stest_fit(stest_vl, "LEFT ");
    StestFit fr = stest_fit(stest_vr, "RIGHT");
    if (fl.ok && fr.ok) {
        ControlGains g = control_get_gains();
        g.vmax_left_mm_s = fl.vmax;
        g.friction_left = fl.friction;
        g.vmax_right_mm_s = fr.vmax;
        g.friction_right = fr.friction;
        control_set_gains(g);
        con_println("applied to runtime gains -- 'w' to save to FRAM, 'p' to review");
    }
    con_println("duty sweep complete");
}

static void stest_start() {
    con_println("duty sweep: 0.05 -> 0.45, both wheels (mouse on blocks!)");
    motors_enable(true);
    motor_set_left(STEST_DUTY_STEP);
    motor_set_right(STEST_DUTY_STEP);
    stest_step = 0;
    stest_measuring = false;
    stest_t0 = millis();
}

// -- Closed-loop velocity tracking test ----------------------------------------
// Triggered by 'v' over serial (mouse on blocks, wheels free!). Steps the
// velocity controller: 0 -> 300 -> 600 -> 0 mm/s, both wheels, printing
// target vs measured vs duty at 20 Hz. Verifies feedforward + PD tracking
// before trusting it on the floor.

static constexpr uint32_t VTEST_PHASE_MS = 1500;
static const float VTEST_TARGETS[] = {300.0f, 600.0f, 0.0f};
static constexpr int VTEST_PHASES = 3;

static int vtest_phase = -1;
static uint32_t vtest_t0 = 0;
static uint32_t vtest_last_print = 0;

static void vtest_poll() {
    if (vtest_phase < 0) return;
    uint32_t now = millis();

    // CSV for live plotting: target,left,right (mm/s).
    if (now - vtest_last_print >= 50) {
        vtest_last_print = now;
        ControlOdometry o = control_odometry();
        ControlVelStatus v = control_velocity_status();
        con_printf("%.0f,%.1f,%.1f\n", v.target_left_mm_s,
                      o.vel_left_mm_s, o.vel_right_mm_s);
    }

    if (now - vtest_t0 < VTEST_PHASE_MS) return;
    vtest_phase++;
    if (vtest_phase < VTEST_PHASES) {
        control_set_wheel_velocity(VTEST_TARGETS[vtest_phase],
                                   VTEST_TARGETS[vtest_phase]);
        vtest_t0 = now;
    } else {
        control_velocity_enable(false);
        vtest_phase = -1;
        con_println("velocity test complete");
    }
}

static void vtest_start() {
    con_println("target,left,right");  // CSV header for live plotting
    control_velocity_enable(true);
    control_set_wheel_velocity(VTEST_TARGETS[0], VTEST_TARGETS[0]);
    vtest_phase = 0;
    vtest_t0 = millis();
    vtest_last_print = 0;
}

// -- IMU/encoder alignment run (profiled) --------------------------------------
// Triggered by 'a'. Mouse ON THE FLOOR, pointing straight, ~1 m of clear
// runway. Drives a trapezoidal motion profile (TrapProfile, see Profile.h) --
// accel well below the traction limit -- while streaming the fused-motion
// estimator at 50 Hz as CSV:
//
//   v_ref,a_ref,v_enc,v_est,v_imu,a_fwd,wheel_acc,slip
//
// v_ref/a_ref are the profiler's commanded velocity/acceleration; a_ref also
// drives the acceleration feedforward (ControlGains::ka) so this test doubles
// as the accel-FF tuning run.
//
// Purpose, in order of what to check:
//   1. Slip gate: nothing actually slips here, so slip should stay 0. If the
//      gate trips, raise MOTION_SLIP_ACC_MM_S2 (Control.h) above the observed
//      wheel_acc - a_fwd residual.
//   2. Accel alignment: v_imu should track v_enc through the ramps (sign +
//      scale of the tilt-compensated accel); the summary reports v_imu drift.
//   3. Accel FF: AUTO-FIT. The run accumulates (a_ref, v_ref - v_enc) samples
//      through both ramps and least-squares fits the ka the run actually
//      needed (KaTuner, Profile.h); the summary applies it to the runtime
//      gains like the 's' sweep does. Re-run 'a' to verify convergence
//      (suggestion should barely move), then 'w' to persist.

static constexpr float ATEST_DIST = 720.0f;  // mm total travel
static constexpr float ATEST_V    = 400.0f;  // mm/s cruise cap
static constexpr float ATEST_ACC  = 500.0f;  // mm/s^2 accel/decel

static KaTuner atest_tuner;
static int atest_state = -1;  // -1 idle, 0 profiling, 1 settle-then-report
static uint32_t atest_t0 = 0;
static uint32_t atest_last_print = 0;
static float atest_d0 = 0;

// Only fit against samples solidly inside a ramp (and never during slip --
// encoder velocity is meaningless then).
static constexpr float ATEST_FIT_MIN_ACC = 0.2f * ATEST_ACC;

static void atest_poll() {
    if (atest_state < 0) return;
    uint32_t now = millis();

    if (atest_state == 0) {
        // The move executor (control_move_start) runs the profile in the
        // 500 Hz tick with position P + IMU heading hold; here we only log
        // and feed the ka tuner.
        ControlMoveStatus st = control_move_status();

        if (now - atest_last_print >= 20) {
            atest_last_print = now;
            ControlOdometry o = control_odometry();
            ControlMotion m = control_motion();
            con_printf("%.0f,%.0f,%.1f,%.1f,%.1f,%.0f,%.0f,%d,%.2f\n",
                          st.v_ref, st.a_ref, o.vel_mm_s, m.v_est_mm_s,
                          m.v_imu_mm_s, m.a_fwd_mm_s2, m.wheel_acc_mm_s2,
                          m.slip ? 1 : 0, st.yaw_err_deg);

            // Fit against the post-position-correction command: that is
            // what the velocity P term acts on.
            if (!m.slip && fabsf(st.a_ref) > ATEST_FIT_MIN_ACC) {
                atest_tuner.add(st.a_ref, st.v_cmd - o.vel_mm_s);
            }
        }

        if (!st.active) {
            atest_state = 1;
            atest_t0 = now;
        }
        return;
    }

    // atest_state == 1: hold zero briefly, then report.
    if (now - atest_t0 >= 500) {
        control_velocity_enable(false);
        atest_state = -1;
        ControlOdometry o = control_odometry();
        ControlMotion m = control_motion();
        ControlMoveStatus st = control_move_status();
        con_printf(
            "-- alignment run summary --\n"
            "move %s | commanded %.1f mm | fused d %.1f mm | encoder dist "
            "%.1f mm | final yaw err %.2f deg\n"
            "v_imu residual %.1f mm/s (pure-IMU drift over run)\n"
            "slip events %lu (expect 0 -- if >0, raise "
            "MOTION_SLIP_ACC_MM_S2)\n",
            st.done ? "DONE" : "TIMED OUT (position not settled)",
            ATEST_DIST, st.d_mm, o.dist_mm - atest_d0, st.yaw_err_deg,
            m.v_imu_mm_s, m.slip_events);

        // ka auto-fit (see KaTuner in Profile.h). Negative suggestions are
        // noise around a true ka of ~0 -- clamp rather than apply.
        ControlGains g = control_get_gains();
        if (atest_tuner.valid()) {
            float ka_new = atest_tuner.suggest(g.ka, g.kp);
            if (ka_new < 0.0f) ka_new = 0.0f;
            con_printf("ka auto-fit: %.6f -> %.6f (n=%d) -- applied to RAM; "
                       "re-run 'a' to verify, 'w' to save\n",
                       g.ka, ka_new, atest_tuner.samples());
            g.ka = ka_new;
            control_set_gains(g);
        } else {
            con_println("ka auto-fit: not enough clean ramp samples (slip?)");
        }
    }
}

static void atest_start() {
    ControlHeading h = control_heading();
    if (!h.ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    con_println("alignment run: ON FLOOR, ~1 m runway. CSV:");
    con_println("v_ref,a_ref,v_enc,v_est,v_imu,a_fwd,wheel_acc,slip,yaw_err");
    atest_tuner.reset();
    control_motion_zero();
    atest_d0 = control_odometry().dist_mm;
    control_velocity_enable(true);
    if (!control_move_start(ATEST_DIST, ATEST_V, ATEST_ACC)) {
        control_velocity_enable(false);
        con_println("move start refused -- fusion not ready?");
        return;
    }
    atest_state = 0;
    atest_t0 = millis();
    atest_last_print = 0;
}

// -- Straight-line self-calibration --------------------------------------------
// Triggered by 'c'. Mouse ON THE FLOOR, pointing straight, ~0.5 m clear.
// Alternating +/-200 mm legs; after each leg the heading hold's average
// cruise effort is folded into the steering trim:
//
//   strim += avg(dv) / avg(v)
//
// Trim is calibrated from FORWARD legs only (hardware 2026-07-08: the
// directional asymmetry means fwd/rev disagree, and one strim can't serve
// both -- the maze drives forward, so calibrate forward). Reverse legs only
// transport the mouse back to x=0. MUST RUN CLEAR OF WALLS: wall contact
// pins the yaw, the contact force absorbs any commanded differential, and
// the trim integrates a constant into its clamp (observed 2026-07-08 when
// shuttling inside the corridor). Converged when two consecutive forward
// legs finish inside tolerance (|yaw| < 1 deg, |pos err| < 15 mm); then 'w'
// persists strim.

// Profile chosen so the leg has a LONG cruise: the trim only samples cruise
// ticks where the heading has settled (MOVE_TRIM_YAW_SETTLED_DEG), and the
// launch kick needs ~0.3 s of cruise to decay through the P-only hold before
// those samples exist. 200 mm/s at 500 mm/s^2 leaves ~120 mm / ~0.6 s of
// cruise per 200 mm leg: ~0.3 s of settled samples even on a kicked leg.
static constexpr float CTEST_DIST = 200.0f;  // mm: shuttle far endpoint
static constexpr float CTEST_V = 200.0f;     // mm/s cruise
static constexpr float CTEST_ACC = 500.0f;   // mm/s^2
static constexpr int CTEST_MAX_LEGS = 16;
static constexpr float CTEST_YAW_TOL_DEG = 1.0f;
// 15 mm: the drive stalls ~11 mm short of target on the maze floor (stiction
// vs the P-only position loop -- see MOVE_DONE_POS_MM). The trim doesn't
// care about endpoint accuracy; this only sanity-checks the leg ran.
static constexpr float CTEST_POS_TOL_MM = 15.0f;
static constexpr float CTEST_STRIM_LIMIT = 0.10f;  // sanity clamp (total)
static constexpr float CTEST_STRIM_STEP = 0.02f;   // per-leg update clamp
static constexpr uint32_t CTEST_PAUSE_MS = 400;
// Legs shuttle between encoder x = 0 and x = CTEST_DIST (see ctest_start_leg),
// so per-leg execution error cannot ACCUMULATE and walk the mouse out of the
// test area (hardware 2026-07-08: relative legs random-walked to +380 mm).
// Leg length is clamped so the trim measurement keeps a usable cruise phase,
// and the test aborts rather than start/continue a leg outside the area.
static constexpr float CTEST_LEG_MIN_MM = 80.0f;
static constexpr float CTEST_LEG_MAX_MM = 320.0f;
static constexpr float CTEST_AREA_MIN_MM = -150.0f;  // between-leg bounds
static constexpr float CTEST_AREA_MAX_MM = 350.0f;
static constexpr float CTEST_RUNAWAY_MM = 100.0f;    // extra mid-leg margin

static int ctest_leg = -1;   // -1 idle; else leg index (even fwd, odd rev)
static int ctest_state = 0;  // 0 = leg running, 1 = pause before next
static uint32_t ctest_t0 = 0;
static bool ctest_fwd_ok = false;  // previous forward leg inside tolerance
static float ctest_x0 = 0.0f;      // encoder dist at test start (area origin)
static uint32_t ctest_slip0 = 0;   // slip events at leg start
static uint32_t ctest_last_print = 0;

// Diagnostic stream, FORWARD legs only (2026-07-08: the heading hold carries
// a standing dv the strim feedforward doesn't null as modelled, in an open
// area -- need to see WHERE the commanded differential goes). Per 25 ms:
//   d, yaw_err, vl, vr, v_enc
// Host-side algebra: commanded dv = kyaw*yaw_err (clamped +/-120) + strim*v;
// measured dv = (vr - vl)/2. Commanded ~= measured but yaw static => yaw
// authority / track wrong or encoder scale asym; measured << commanded =>
// wheel loops not delivering (directional motor asym / stiction).

static float ctest_dir() { return (ctest_leg % 2 == 0) ? 1.0f : -1.0f; }

// Encoder position within the test area. Encoders are the ground truth the
// user asked the shuttle to hold: unlike the fused d (slip gate can distrust
// the encoders mid-leg), this is what the wheels actually rolled.
static float ctest_x() { return control_odometry().dist_mm - ctest_x0; }

static bool ctest_start_leg() {
    // Aim at the endpoint, not "another 200 mm": overshoot on the previous
    // leg shortens this one instead of stacking.
    float target = (ctest_dir() > 0.0f) ? CTEST_DIST : 0.0f;
    float leg = target - ctest_x();
    if (leg > CTEST_LEG_MAX_MM) leg = CTEST_LEG_MAX_MM;
    if (leg < -CTEST_LEG_MAX_MM) leg = -CTEST_LEG_MAX_MM;
    if (fabsf(leg) < CTEST_LEG_MIN_MM) {
        leg = (leg < 0.0f ? -1.0f : 1.0f) * CTEST_LEG_MIN_MM;
    }
    if (!control_move_start(leg, CTEST_V, CTEST_ACC)) {
        return false;
    }
    ctest_slip0 = control_motion().slip_events;
    ctest_state = 0;
    return true;
}

static void ctest_finish(const char *msg) {
    control_velocity_enable(false);
    ctest_leg = -1;
    ControlGains g = control_get_gains();
    con_printf("%s: strim %+.5f -- 'w' to save\n", msg, g.strim);
}

static void ctest_poll() {
    if (ctest_leg < 0) return;
    uint32_t now = millis();

    if (ctest_state == 0) {
        ControlMoveStatus st = control_move_status();
        if (st.active) {
            // Diagnostic CSV, forward legs only (see header above).
            if (ctest_dir() > 0.0f && now - ctest_last_print >= 25) {
                ctest_last_print = now;
                ControlOdometry o = control_odometry();
                con_printf("%.1f,%.2f,%.1f,%.1f,%.1f\n", st.d_mm,
                              st.yaw_err_deg, o.vel_left_mm_s,
                              o.vel_right_mm_s, o.vel_mm_s);
            }
            // Mid-leg runaway guard: if the wheels have carried the mouse
            // clear of the area (position loop chasing a bad fused d), cut
            // the move instead of leaving the test area.
            float x = ctest_x();
            if (x < CTEST_AREA_MIN_MM - CTEST_RUNAWAY_MM ||
                x > CTEST_AREA_MAX_MM + CTEST_RUNAWAY_MM) {
                control_move_abort();
                ctest_finish("calibration ABORTED (mid-leg runaway -- "
                             "check slip gate / floor)");
            }
            return;
        }

        // Leg finished. Trim folds from FORWARD legs only: the launch-kick
        // asymmetry is DIRECTIONAL (fwd/rev needs differ), a single strim
        // cannot satisfy both, and the maze drives straights forward --
        // calibrate for that. REV legs just transport the mouse back to
        // x=0. A leg must also be SLIP-FREE (slip poisons the cruise stats;
        // endpoint settle does not matter -- stiction stall, not cruise).
        uint32_t slips = control_motion().slip_events - ctest_slip0;
        float d_err = st.d_ref - st.d_mm;
        ControlMoveTrim tr = control_move_trim();
        ControlGains g = control_get_gains();
        bool fwd = ctest_dir() > 0.0f;
        if (fwd && slips == 0 && tr.valid && fabsf(tr.avg_v_mm_s) > 50.0f) {
            float step = tr.avg_dv_mm_s / tr.avg_v_mm_s;
            if (step > CTEST_STRIM_STEP) step = CTEST_STRIM_STEP;
            if (step < -CTEST_STRIM_STEP) step = -CTEST_STRIM_STEP;
            g.strim += step;
            if (g.strim > CTEST_STRIM_LIMIT) g.strim = CTEST_STRIM_LIMIT;
            if (g.strim < -CTEST_STRIM_LIMIT) g.strim = -CTEST_STRIM_LIMIT;
            control_set_gains(g);
        }

        bool leg_ok = slips == 0 && st.done &&
                      fabsf(st.yaw_err_deg) < CTEST_YAW_TOL_DEG &&
                      fabsf(d_err) < CTEST_POS_TOL_MM;
        con_printf("leg %2d %s: yaw %+.2f deg, pos err %+.1f mm, x %+.1f mm, "
                   "slip %lu, strim -> %+.5f %s%s\n",
                   ctest_leg, fwd ? "FWD" : "REV(transport)",
                   st.yaw_err_deg, d_err, ctest_x(), (unsigned long)slips,
                   g.strim, tr.valid ? "" : "no-trim-samples ",
                   leg_ok ? "ok" : "");

        // Converged: two CONSECUTIVE forward legs inside tolerance.
        if (fwd) {
            if (leg_ok && ctest_fwd_ok) {
                ctest_finish("calibration CONVERGED");
                return;
            }
            ctest_fwd_ok = leg_ok;
        }

        if (++ctest_leg >= CTEST_MAX_LEGS) {
            ctest_finish("calibration stopped (max legs, not converged -- "
                         "check wheels/floor)");
            return;
        }
        ctest_state = 1;
        ctest_t0 = now;
        return;
    }

    // Pause between legs (mechanical settle), then launch the next one.
    if (now - ctest_t0 >= CTEST_PAUSE_MS) {
        float x = ctest_x();
        if (x < CTEST_AREA_MIN_MM || x > CTEST_AREA_MAX_MM) {
            ctest_finish("calibration ABORTED (outside test area)");
            return;
        }
        if (!ctest_start_leg()) ctest_finish("calibration aborted (move refused)");
    }
}

static void ctest_start() {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    con_println("straight-line calibration: ON FLOOR, ~0.5 m CLEAR OF WALLS "
                "(wall contact poisons the trim). Shuttle x=0 <-> x=200 mm");
    con_println("FWD-leg CSV: d,yaw_err,vl,vr,v_enc");
    ctest_last_print = 0;
    control_motion_zero();
    control_velocity_enable(true);
    ctest_x0 = control_odometry().dist_mm;
    ctest_leg = 0;
    ctest_fwd_ok = false;
    if (!ctest_start_leg()) ctest_finish("calibration aborted (move refused)");
}

// -- On-the-spot turn test (auto-tuning, closed-loop, track auto-cal) -----------
// Triggered by 't'. Mouse ON THE FLOOR, clear of walls. Runs profiled in-place
// turns through the maneuver executor (control_turn_start) and AUTO-RAISES the
// velocity-loop drive gain until the turn actually reaches the commanded rate
// and lands on the target angle. Symptom this fixes: at low turn speed the
// commanded wheel speed is small, so a soft kp can't build enough duty to
// overcome in-place tyre scrub -- the mouse creeps a few degrees and stalls.
//
// Each attempt turns TTEST_CAL_ANGLE (direction alternates to stay put) and is
// judged on two criteria:
//   1. rate: mean measured yaw rate during cruise >= TTEST_RATE_FRAC * omega
//   2. angle: the turn SETTLED (done) within TTEST_ANGLE_TOL of the target
// If either falls short the run is undershooting. WHICH WAY to move kpt is
// decided by the SLIP flag (2026-09-16, bert's session on the D9 floor):
// with kpt 0.0055 -> 0.0088 -> 0.0141 the cruise rate went 376 -> 291 ->
// 330 of 450 with slip flagged through the plateau and the mouse audibly
// BUZZING; with kpt 0.0040 the same turn ran 426/450, silent, no slip.
// Raising the wheel gain on a turn that is already slipping only chatters
// the tyres loose harder. So: undershoot WITH slip -> LOWER kpt (/ 1.3);
// undershoot without slip -> raise it (x 1.25, not the old x 1.6 that
// leapt straight past the good band); converge when the rate is reached
// with the plateau slip-free. On success the effective track is fitted
// (TrackTuner: for pure rotation 0.5*(v_right - v_left) = yaw_rate *
// track/2) and applied -- from slip-free samples only, and only when the
// attempt itself was clean (the slipping attempts fitted 75-78 mm, the
// clean ones 67-69, against the stored 72.3).
// Streams per tick as CSV: yaw_ref,yaw,yaw_err,omega_ref,yaw_rate,wheel_diff,slip
//
// NOTE: raises kp_turn (kpt), the in-place-turn velocity gain -- straights
// keep their own kp and are unaffected (split after the shared kp buzzed
// the 'c' legs, 2026-07-07). 'w' persists kpt + track.

// The rate is the REGISTRY's (`tomg` / `talp`, 2026-09-16): kpt must be
// tuned at the rate the missions actually use, and that rate is now a
// tuning variable itself (the recorded goal 180 of 2026-09-15 reached 298
// of 450 deg/s with ~30 % tyre slip). So: set tomg, run 't', read the
// cruise rate and the slip column, 'w'. The 450/4000 that lived here is
// the registry default.
static inline float TTEST_OMEGA_NOW() { return control_get_gains().turn_omega_dps; }
static inline float TTEST_ALPHA_NOW() { return control_get_gains().turn_alpha_dps2; }
static constexpr float TTEST_CAL_ANGLE = 90.0f; // per-attempt turn (no 180 wrap)
static constexpr float TTEST_RATE_FRAC = 0.82f; // cruise rate must reach 82% omega
static constexpr float TTEST_SLIP_FRAC = 0.15f; // plateau samples flagged slip -> "slipping"
static constexpr float TTEST_KP_DOWN = 1.3f;    // kp divisor when slipping
static constexpr float TTEST_KP_MIN = 0.002f;
static constexpr float TTEST_ANGLE_TOL = 5.0f;  // must settle within 5 deg
static constexpr float TTEST_KP_STEP = 1.25f;   // kp multiplier per clean undershoot
// 0.02, was 0.012: kpt converged to 0.01074 at 300 dps, right under the
// old cap -- tuning at 450 dps needs headroom above it. Wheel duty stays
// bounded by VCTRL_DUTY_LIMIT regardless.
static constexpr float TTEST_KP_MAX = 0.012f;   // safety cap on kp (0.0164 buzzed and slipped)
static constexpr int   TTEST_MAX_ATTEMPTS = 8;
static constexpr uint32_t TTEST_PAUSE_MS = 600;
// ramp+cruise: 0.3 x the rate (evaluated where used)
static inline float TTEST_FIT_MIN_DPS() { return 0.3f * TTEST_OMEGA_NOW(); }
static constexpr float TTEST_CRUISE_FRAC = 0.85f;               // omega -> cruise

static TrackTuner ttest_tuner;
static int ttest_phase = -1;   // -1 idle, 0 turn running, 1 pause before retry
static int ttest_attempt = 0;
static uint32_t ttest_t0 = 0;
static uint32_t ttest_last_print = 0;
static float ttest_dir = 1.0f;      // alternate each attempt to stay in place
static float ttest_rate_sum = 0.0f; // |yaw_rate| accumulated during cruise
static int ttest_rate_n = 0;
static int ttest_slip_n = 0;        // cruise samples with the slip gate tripped
static float ttest_track_fit = 0.0f;   // the last clean fit, reported
static bool ttest_apply_track = false; // 't.track': apply the fit this run
static uint32_t ttest_last_fit = 0;

static bool ttest_start_turn() {
    ttest_tuner.reset();
    ttest_rate_sum = 0.0f;
    ttest_rate_n = 0;
    ttest_slip_n = 0;
    if (!control_turn_start(ttest_dir * TTEST_CAL_ANGLE, TTEST_OMEGA_NOW(),
                            TTEST_ALPHA_NOW())) {
        return false;
    }
    ttest_phase = 0;
    ttest_t0 = millis();
    ttest_last_print = 0;
    ttest_last_fit = 0;
    return true;
}

// Converged: fit the effective track from this (full-rate) turn and stop.
static void ttest_apply_and_finish() {
    control_velocity_enable(false);
    ttest_phase = -1;
    ControlGains g = control_get_gains();
    con_printf("turn auto-tune CONVERGED at kpt=%.5f\n", g.kp_turn);
    if (ttest_tuner.valid()) {
        float tr = ttest_tuner.suggest_track_mm();
        if (tr > 40.0f && tr < 100.0f) {
            // REPORTED, NOT APPLIED (2026-09-16). The clean turns fit 67-69
            // against the stored 72.3, and `track` is the arc feedforward
            // too (dv_ff = omega x track/2): every corner's atrim / us /
            // kparc was tuned with 72.3 in place, so a 7 % change here
            // re-tunes the corners by stealth. 't.track' applies it, on a
            // day set aside for that.
            con_printf("track fit: %.2f mm from this turn (stored %.2f, n=%d) "
                       "-- NOT applied; 't.track' applies the next fit\n",
                          tr, g.track_mm, ttest_tuner.samples());
            ttest_track_fit = tr;
            if (ttest_apply_track) {
                g.track_mm = tr;
                control_set_gains(g);
                con_printf("track %.2f applied to RAM -- 'w' to persist\n", tr);
            }
        } else {
            con_printf("track auto-fit: %.2f mm out of band (n=%d) -- ignored\n",
                          tr, ttest_tuner.samples());
        }
    } else {
        con_println("track auto-fit: not enough clean samples");
    }
    con_println("kpt is turn-only (straights keep kp) -- 'w' to persist kpt");
    ttest_apply_track = false;
}

static void ttest_poll() {
    if (ttest_phase < 0) return;
    uint32_t now = millis();

    if (ttest_phase == 0) {
        ControlMoveStatus st = control_move_status();

        if (now - ttest_last_print >= 20) {
            ttest_last_print = now;
            ControlOdometry o = control_odometry();
            ControlMotion m = control_motion();
            float yaw = control_heading().yaw_deg;
            float wheel_diff = 0.5f * (o.vel_right_mm_s - o.vel_left_mm_s);
            con_printf("%.1f,%.1f,%.2f,%.0f,%.1f,%.1f,%d\n",
                          st.yaw_ref_deg, yaw, st.yaw_err_deg, st.omega_ref_dps,
                          st.yaw_rate_dps, wheel_diff, m.slip ? 1 : 0);

            // Measure the achieved rate only where the profile is at cruise.
            if (fabsf(st.omega_ref_dps) >= TTEST_CRUISE_FRAC * TTEST_OMEGA_NOW()) {
                ttest_rate_sum += fabsf(st.yaw_rate_dps);
                ttest_rate_n++;
                if (m.slip) ttest_slip_n++;
            }
        }

        // Track fit: through the body of the turn, never during slip. On its
        // own 5 ms gate -- tied to the 20 ms print gate a 90-deg turn yields
        // only ~18 samples above TTEST_FIT_MIN_DPS, under TrackTuner's n>=20
        // (seen on hardware 2026-07-09). Control runs at 500 Hz, so 5 ms
        // still means fresh odometry each sample (~70 per turn).
        if (now - ttest_last_fit >= 5) {
            ttest_last_fit = now;
            ControlOdometry o = control_odometry();
            ControlMotion m = control_motion();
            float wheel_diff = 0.5f * (o.vel_right_mm_s - o.vel_left_mm_s);
            if (!m.slip && fabsf(st.omega_ref_dps) > TTEST_FIT_MIN_DPS()) {
                ttest_tuner.add(st.yaw_rate_dps * DEG2RAD, wheel_diff);
            }
        }

        if (!st.active) {
            float mean_rate = ttest_rate_n ? ttest_rate_sum / ttest_rate_n : 0.0f;
            float slip_frac = ttest_rate_n ? (float)ttest_slip_n / ttest_rate_n : 0.0f;
            bool slipping = slip_frac >= TTEST_SLIP_FRAC;
            bool rate_ok = mean_rate >= TTEST_RATE_FRAC * TTEST_OMEGA_NOW();
            bool angle_ok = st.done && fabsf(st.yaw_err_deg) < TTEST_ANGLE_TOL;
            ControlGains g = control_get_gains();
            con_printf("attempt %d: kpt=%.5f -> cruise rate %.0f/%.0f dps, slip "
                       "%.0f%% of the plateau, final err %.2f deg (%s)\n",
                          ttest_attempt + 1, g.kp_turn, mean_rate, TTEST_OMEGA_NOW(),
                          slip_frac * 100.0f,
                          st.yaw_err_deg, st.done ? "settled" : "TIMEOUT");

            if (rate_ok && angle_ok && !slipping) {
                ttest_apply_and_finish();
                return;
            }
            // Which way? Slipping -> the tyres are already past what the
            // floor gives at this kpt: LOWER it. Clean undershoot -> raise.
            float kp_next = slipping ? g.kp_turn / TTEST_KP_DOWN
                                     : g.kp_turn * TTEST_KP_STEP;
            bool out = kp_next > TTEST_KP_MAX || kp_next < TTEST_KP_MIN;
            if (ttest_attempt + 1 >= TTEST_MAX_ATTEMPTS || out) {
                control_velocity_enable(false);
                ttest_phase = -1;
                con_printf("turn auto-tune STOPPED at kpt=%.5f (%s) -- %s; kpt "
                           "left in RAM\n", g.kp_turn,
                           out ? "kpt range" : "attempts",
                           slipping ? "still slipping at the lowest gain: the "
                                      "floor cannot give this rate, lower tomg"
                                    : "check battery/friction");
                return;
            }
            g.kp_turn = kp_next;
            control_set_gains(g);
            con_printf("%s -> %s kpt to %.5f, retrying\n",
                       slipping ? "slipping" : "clean undershoot",
                       slipping ? "LOWERING" : "raising", g.kp_turn);
            ttest_attempt++;
            ttest_dir = -ttest_dir;  // untwist: alternate direction
            ttest_phase = 1;
            ttest_t0 = now;
        }
        return;
    }

    // ttest_phase == 1: pause, then the next attempt.
    if (now - ttest_t0 >= TTEST_PAUSE_MS) {
        if (!ttest_start_turn()) {
            control_velocity_enable(false);
            ttest_phase = -1;
            con_println("turn test aborted (turn refused)");
        }
    }
}

static void ttest_start() {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    con_println("turn auto-tune: ON FLOOR, clear of walls. Raises kpt (turn-only "
                "gain) until turns hit rate + angle. CSV:");
    con_println("yaw_ref,yaw,yaw_err,omega_ref,yaw_rate,wheel_diff,slip");
    control_motion_zero();
    control_velocity_enable(true);
    ttest_attempt = 0;
    ttest_dir = 1.0f;
    if (!ttest_start_turn()) {
        control_velocity_enable(false);
        con_println("turn test aborted (turn refused)");
    }
}

// -- Smooth corner (arc) test --------------------------------------------------
// Triggered by 'r'. Mouse ON THE FLOOR with room for a circle of radius
// ARC_RADIUS. Runs ARC_SEGMENTS arcs of ARC_ANGLE each via control_arc_start;
// 4x90 = a full circle, so a clean arc returns to the START heading AND
// position. Position is dead-reckoned (integrated from odometry) since there
// are no wall sensors yet -- heading is closed-loop on the IMU -- so the loop
// CLOSURE error is the headline number for arc quality. CSV per tick:
//   seg,d_ref,d,yaw_ref,yaw,yaw_err,v_ref,omega_ref

static constexpr float ARC_RADIUS = 90.0f;    // mm (about half a cell)
static constexpr float ARC_OMEGA  = 180.0f;   // deg/s angular cruise
static constexpr float ARC_ALPHA  = 1000.0f;  // deg/s^2 angular accel
static constexpr float ARC_ANGLE  = 90.0f;    // deg per segment
static constexpr int   ARC_SEGMENTS = 4;      // 4x90 -> full-circle closure
static constexpr uint32_t ARC_PAUSE_MS = 400;

static int rtest_seg = -1;     // -1 idle; else segment index
static int rtest_state = 0;    // 0 = arc running, 1 = pause before next
static uint32_t rtest_t0 = 0;
static uint32_t rtest_last_print = 0;
static float rtest_x = 0.0f, rtest_y = 0.0f;  // dead-reckoned path
static float rtest_yaw0 = 0.0f, rtest_prevd = 0.0f;

static bool rtest_start_seg() {
    if (!control_arc_start(ARC_RADIUS, ARC_ANGLE, ARC_OMEGA, ARC_ALPHA)) {
        return false;
    }
    rtest_state = 0;
    rtest_t0 = millis();
    return true;
}

static void rtest_poll() {
    if (rtest_seg < 0) return;
    uint32_t now = millis();

    if (rtest_state == 0) {
        ControlMoveStatus st = control_move_status();
        if (now - rtest_last_print >= 20) {
            rtest_last_print = now;
            ControlOdometry o = control_odometry();
            float yaw = control_heading().yaw_deg;
            // Dead-reckon the centre path: project forward travel by heading.
            float dd = o.dist_mm - rtest_prevd;
            rtest_prevd = o.dist_mm;
            rtest_x += dd * cosf(yaw * DEG2RAD);
            rtest_y += dd * sinf(yaw * DEG2RAD);
            con_printf("%d,%.1f,%.1f,%.1f,%.1f,%.2f,%.0f,%.0f\n",
                          rtest_seg, st.d_ref, st.d_mm, st.yaw_ref_deg, yaw,
                          st.yaw_err_deg, st.v_ref, st.omega_ref_dps);
        }
        if (!st.active) {
            con_printf("arc %d done: final yaw err %.2f deg (%s)\n",
                          rtest_seg, st.yaw_err_deg, st.done ? "settled" : "TIMEOUT");
            rtest_state = 1;
            rtest_t0 = now;
        }
        return;
    }

    // rtest_state == 1: pause, then next segment (or finish).
    if (now - rtest_t0 >= ARC_PAUSE_MS) {
        if (++rtest_seg >= ARC_SEGMENTS) {
            control_velocity_enable(false);
            rtest_seg = -1;
            float closure = sqrtf(rtest_x * rtest_x + rtest_y * rtest_y);
            con_printf("-- arc loop done -- closure %.1f mm (expect ~0); "
                       "heading %.1f -> %.1f deg (expect return to start)\n",
                          closure, rtest_yaw0, control_heading().yaw_deg);
            return;
        }
        if (!rtest_start_seg()) {
            control_velocity_enable(false);
            rtest_seg = -1;
            con_println("arc test aborted (arc refused)");
        }
    }
}

static void rtest_start() {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    con_printf("arc test: ON FLOOR, need ~%.0f mm of circle. CSV:\n",
                  2.0f * ARC_RADIUS);
    con_println("seg,d_ref,d,yaw_ref,yaw,yaw_err,v_ref,omega_ref");
    control_motion_zero();
    control_velocity_enable(true);
    rtest_x = rtest_y = 0.0f;
    rtest_prevd = control_odometry().dist_mm;
    rtest_yaw0 = control_heading().yaw_deg;
    rtest_last_print = 0;
    rtest_seg = 0;
    if (!rtest_start_seg()) {
        control_velocity_enable(false);
        rtest_seg = -1;
        con_println("arc test aborted (arc refused)");
    }
}

// -- Wall-sensor live readout --------------------------------------------------
// Triggered by 'n' (toggle: 'n' again stops). Puts the sub-MCU in FREE-RUN
// (it streams ~440 Hz) and just reads the latest packet, printing the four raw
// beam ranges at ~20 Hz as CSV. sensors_poll() runs every loop iteration to
// keep the UART FIFO drained; the print is throttled. Values are direct BEAM
// RANGE in mm, 255 = out of range -- not perpendicular wall distance (that
// conversion comes later). ms_since flags a silent sub-MCU. NOTE: free-run
// runs the lasers at high duty -- stop it ('n') when you're done looking.
//
// YAW AND TEMPERATURE ARE IN THIS LINE (2026-08-13) and they are not
// decoration. 'n' counts as a running test, so it SUPPRESSES the 100 ms
// status line -- which is where yaw and IMU temperature used to live. That
// made the one measurement we most need impossible to take: laser and gyro
// readings of the SAME instant. The whole point of the gyro-vs-walls bench
// test is that the mouse does not move, so the lasers pin "nothing changed"
// while yaw is watched for drift; two streams that cannot be interleaved
// cannot do that. Leading `ms` so a drift RATE comes straight out of the
// capture without assuming the print cadence held.

static bool ntest_active = false;
static uint32_t ntest_last = 0;

static void ntest_start() {
    con_println("sensor readout (free-run). 'n' again to stop. CSV:");
    con_println("ms,left,front_r,front_l,right,seq,ms_since,L,R,F,front_mm,"
                "lat_mm,rdy,yaw,temp_c");
    con_println("** rdy column: 1 = fresh packet from sub-MCU, 0 = link silent **");
    sensors_free_run();
    ntest_active = true;
    ntest_last = 0;
}

static void ntest_stop() {
    ntest_active = false;
    sensors_stop();  // exit free-run, lasers off
    con_println("sensor readout stopped");
}

static void ntest_poll() {
    if (!ntest_active) return;
    sensors_poll();  // drain FIFO every iteration, keep the latest packet
    uint32_t now = millis();
    if (now - ntest_last < 50) return;
    ntest_last = now;
    WallRanges w = sensors_latest();
    uint32_t ms = sensors_ms_since_packet();
    WallView wv = walls_evaluate(w, ms);
    ControlHeading h = control_heading();
    ControlImuStatus st = control_imu_status();
    con_printf("%lu,%d,%d,%d,%d,%lu,%lu,%d,%d,%d,%.0f,%.1f,%d,%.3f,%.1f\n",
                  (unsigned long)now, w.left_mm,
                  w.front_right_mm, w.front_left_mm, w.right_mm,
                  (unsigned long)w.seq, (unsigned long)ms, wv.left, wv.right,
                  wv.front, wv.front_mm, wv.lateral_valid ? wv.lateral_mm : 0.0f,
                  sensors_ready() ? 1 : 0,
                  h.ready ? h.yaw_deg : 0.0f,
                  imu_temp_c(st.latest.temp_raw));
}

// -- Wall-sensor geometry capture ----------------------------------------------
// Triggered by 'g'. Averaged snapshot for the sensor-geometry calibration:
// free-runs the sub-MCU for GTEST_MS with the mouse held STILL at a known
// pose (e.g. centred + parallel in a corridor), accumulates per-beam stats,
// then prints mean/std/min/max/sample count per beam. Out-of-range samples
// (255) are excluded from the stats and counted separately (oor). Repeat at
// several known lateral offsets / yaws to fit the side-beam mount geometry.

static constexpr uint32_t GTEST_MS = 1500;

struct GStat {
    float sum, sum2;
    uint16_t n, oor;
    uint8_t mn, mx;
};

static bool gtest_active = false;
static uint32_t gtest_t0 = 0;
static uint32_t gtest_last_seq = 0;
static GStat gstat[4];

static void gstat_add(GStat &s, uint8_t v) {
    if (v == SENSOR_OUT_OF_RANGE) {
        s.oor++;
        return;
    }
    s.sum += v;
    s.sum2 += (float)v * (float)v;
    s.n++;
    if (v < s.mn) s.mn = v;
    if (v > s.mx) s.mx = v;
}

static void gtest_start() {
    con_printf("geometry capture: hold pose, %lu ms...\n",
                  (unsigned long)GTEST_MS);
    for (int i = 0; i < 4; i++) gstat[i] = {0.0f, 0.0f, 0, 0, 255, 0};
    sensors_free_run();
    gtest_active = true;
    gtest_t0 = millis();
    gtest_last_seq = sensors_latest().seq;
}

static void gtest_poll() {
    if (!gtest_active) return;
    sensors_poll();  // drain FIFO every loop iteration
    WallRanges w = sensors_latest();
    if (w.seq != gtest_last_seq) {
        gtest_last_seq = w.seq;
        gstat_add(gstat[0], w.left_mm);
        gstat_add(gstat[1], w.front_left_mm);
        gstat_add(gstat[2], w.front_right_mm);
        gstat_add(gstat[3], w.right_mm);
    }
    if (millis() - gtest_t0 < GTEST_MS) return;

    sensors_stop();  // lasers off
    gtest_active = false;
    static const char *names[4] = {"left", "front_l", "front_r", "right"};
    con_println("beam,mean_mm,std_mm,min,max,n,oor");
    for (int i = 0; i < 4; i++) {
        const GStat &s = gstat[i];
        if (s.n == 0) {
            con_printf("%s,-,-,-,-,0,%u\n", names[i], s.oor);
            continue;
        }
        float mean = s.sum / s.n;
        float var = s.sum2 / s.n - mean * mean;
        if (var < 0.0f) var = 0.0f;
        con_printf("%s,%.2f,%.2f,%u,%u,%u,%u\n", names[i], mean, sqrtf(var),
                      s.mn, s.mx, s.n, s.oor);
    }
    if (gstat[0].n + gstat[0].oor == 0) {
        con_println("** no packets -- sub-MCU silent? check Serial2 link **");
    }
}

// -- Wall-sensor placement test ('wcal') --------------------------------------
// bert, 2026-09-06: "a corridor wall sensor test: mouse centrally, 10 mm
// left, 10 mm right, streaming to the console to log. A console command to
// place the mouse in a displayed wall setup." The question it answers: do
// the two side beams read the SAME lateral the same way? D8's diagonal
// posts said the mouse was 16 mm left by the left beam and 2 mm left by the
// right beam, every lap, and the V's inside clearance rode on which was
// right.
//
// CORRIDOR: three static placements (centre / 10 mm left / 10 mm right),
// each captured WCAL_MS with a paced CSV stream, and 'wcal fit' for the
// slopes; or THE SWEEP ('wcal s', bert 2026-09-06 evening): start centred
// and parallel, slide sideways until the body TOUCHES the left wall, across
// to touch the right wall, back to centre, straight throughout, one stream.
// The two touches are each beam's reading with the bodywork on the wall --
// that sensor's contact zero (the "38" of FLINCH.md, per side), and L+R
// along the way is the corridor width in beam units. First result
// 2026-09-06: L 40 / R 36 at contact, 82.0 / 82.1 at centre, L+R 165.
//
// POSTS: in a real diagonal the posts ALTERNATE sides (bert: "the posts are
// offset in a real maze") -- 63.6 mm off the centre line, one side every
// 254.6 mm, the other side half-way between -- so a beam ever sees ONE post,
// and the far side at that station is the opposite sawtooth's valley,
// 190.9 mm away. Four tests, one post at a time: 'wcal lp' / 'wcal rp'
// static, centred on the line with the LEFT / RIGHT post at the beam
// (nudge fore/aft for the lowest reading: that is the geometry DGATE uses),
// and 'wcal lps' / 'wcal rps', the sweep with that post: centred, hold, slide
// TOWARD the post 20 mm (a tape mark), back to centre, AWAY 20 mm, back.
// Reports the post-side beam's centre, nearest (min) and furthest (max)
// readings and the slope each way. (No touch on a post: THE SIDE BEAMS
// CROSS -- the LEFT beam fires from the emitter on the RIGHT of the body,
// 15 mm off centre, across the nose, 17.2 deg forward of square; the RIGHT
// beam from the left emitter at 19 deg -- so the near corner of a post at
// 55.1 off the line is ~62 mm ahead of the mouse centre when the beam
// reads lowest, clear ahead of the body, and sliding sideways touches
// nothing.) After every capture the sensors stay on in WATCH mode, one
// line per 250 ms, so the next placement can be seen; 'wcal w' starts it
// by itself, 'wcal' stops it.
static bool any_test_running();
static constexpr uint32_t WCAL_MS = 2000;
static constexpr uint32_t WCAL_PRINT_MS = 50;
static constexpr uint32_t WCAL_SWEEP_MS = 30000;
static constexpr uint32_t WCAL_CENTRE_MS = 700;   // the opening stretch = centre
static constexpr float WCAL_POST_STEP_MM = 20.0f;  // the tape marks toward / away from the post
static constexpr uint32_t WCAL_WATCH_MS = 250;
static bool wcal_watch = false;
static uint32_t wcal_watch_last = 0;
static bool wcal_active = false;
static uint32_t wcal_t0 = 0, wcal_last_print = 0, wcal_last_seq = 0;
static int wcal_set = 0;             // 0 corridor, 1 LEFT post, 2 RIGHT post
static int wcal_slot = -1;           // corridor: 0 centre, 1 left 10, 2 right 10; posts: 0
static bool wcal_sweep = false;
static GStat wcal_stat[4];
static float wcal_mean[3][3][4];     // [set][slot][beam]
static bool wcal_have[3][3];
static const char *const WCAL_SLOT_NAME[3] = {"CENTRE", "10 mm LEFT", "10 mm RIGHT"};
static const char *const WCAL_SET_NAME[3] = {"CORRIDOR", "LEFT POST", "RIGHT POST"};
static uint8_t wcal_min[4], wcal_max[4];
static uint32_t wcal_min_at[4], wcal_max_at[4];
static float wcal_c_sum[4]; static uint16_t wcal_c_n[4];   // centre means
static double wcal_sum_s = 0, wcal_sum_s2 = 0; static uint32_t wcal_sum_n = 0;
static const float WCAL_TRUE_LAT[3] = {0.0f, 10.0f, -10.0f};   // + = left

static void wcal_help() {
    con_println("WALL SENSOR TEST -- the GUI's 'Wall sensor test' draws these to scale.");
    con_println("  CORRIDOR: a straight with BOTH walls (168 face to face), mouse parallel.");
    con_println("    wcal s          SWEEP: centred, hold ~1 s, slide to TOUCH the left wall,");
    con_println("                    across to touch the right, back to centre; 'wcal' stops it.");
    con_println("                    -> each beam's reading at contact (its zero), L+R.");
    con_println("    wcal c|l|r      statics, 2 s: centred / 10 mm LEFT / 10 mm RIGHT.");
    con_println("  POSTS: mouse on the diagonal centre line (tape it), ONE post at the beam,");
    con_println("    63.6 off the line; fore/aft where that beam reads LOWEST.");
    con_println("    wcal lp | rp    static, 2 s, centred, LEFT / RIGHT post at the beam.");
    con_println("    wcal lps | rps  SWEEP with that post: centred, hold ~1 s, slide TOWARD the");
    con_println("                    post 20 mm (tape mark), back, AWAY 20 mm, back.");
    con_println("  wcal w            WATCH: beams on, a line per 250 ms, to place the mouse.");
    con_println("                    Every capture ends in watch; only 'wcal off' turns the beams off.");
    con_println("  wcal fit          the captures against each other and the truth");
    con_println("  wcal x            forget the captures");
    con_println("  LEFT = the mouse moved toward its own left; keep it parallel throughout.");
}

static void wcal_watch_start() {
    wcal_watch = true;
    wcal_watch_last = 0;
    sensors_free_run();
    con_println("wcal watch: L,FL,FR,R every 250 ms -- place the mouse for the next test ('wcal off' = beams off)");
}

static void wcal_start(int set, int slot) {
    wcal_watch = false;
    wcal_set = set;
    wcal_slot = slot;
    wcal_sweep = slot < 0;
    for (int i = 0; i < 4; i++) {
        wcal_min[i] = 255; wcal_max[i] = 0; wcal_min_at[i] = 0; wcal_max_at[i] = 0;
        wcal_c_sum[i] = 0; wcal_c_n[i] = 0;
    }
    wcal_sum_s = wcal_sum_s2 = 0; wcal_sum_n = 0;
    if (wcal_sweep) {
        if (set == 0)
            con_printf("wcal: CORRIDOR SWEEP -- start CENTRED and parallel, hold ~1 s, then slide\n"
                       "  sideways until the body TOUCHES on the LEFT, across to touch on the\n"
                       "  RIGHT, back to centre. Keep it straight. Up to %lu s; 'wcal' stops it. CSV:\n",
                       (unsigned long)(WCAL_SWEEP_MS / 1000));
        else
            con_printf("wcal: %s SWEEP -- start CENTRED on the line, post at the beam, hold ~1 s,\n"
                       "  slide TOWARD the post 20 mm (tape mark), back to centre, AWAY 20 mm,\n"
                       "  back. Keep it straight. Up to %lu s; 'wcal' stops it. CSV:\n",
                       WCAL_SET_NAME[set], (unsigned long)(WCAL_SWEEP_MS / 1000));
        con_println("ms,L,FL,FR,R,lat");
        sensors_free_run();
        wcal_active = true;
        wcal_t0 = millis();
        wcal_last_print = 0;
        wcal_last_seq = sensors_latest().seq;
        return;
    }
    for (int i = 0; i < 4; i++) wcal_stat[i] = {0.0f, 0.0f, 0, 0, 255, 0};
    con_printf("wcal: %s %s -- hold still, %lu ms. CSV:\n",
               WCAL_SET_NAME[set], set == 0 ? WCAL_SLOT_NAME[slot] : "centred, post at the beam",
               (unsigned long)WCAL_MS);
    con_println("ms,L,FL,FR,R,lat");
    sensors_free_run();
    wcal_active = true;
    wcal_t0 = millis();
    wcal_last_print = 0;
    wcal_last_seq = sensors_latest().seq;
}

static void wcal_sweep_finish() {
    wcal_active = false;
    wcal_sweep = false;
    static const char *names[4] = {"L", "FL", "FR", "R"};
    con_printf("wcal %s sweep, %lu ms:\n", WCAL_SET_NAME[wcal_set],
               (unsigned long)(millis() - wcal_t0));
    con_println("beam,centre_mean,min_reading,min_at_ms,max_reading,max_at_ms");
    for (int i = 0; i < 4; i++) {
        hal_radio_wait_room(64, 10);
        if (wcal_c_n[i])
            con_printf("%s,%.1f,%u,%lu,%u,%lu\n", names[i], (double)(wcal_c_sum[i] / wcal_c_n[i]),
                       wcal_min[i], (unsigned long)wcal_min_at[i], wcal_max[i],
                       (unsigned long)wcal_max_at[i]);
        else
            con_printf("%s,-,%u,%lu,%u,%lu\n", names[i], wcal_min[i], (unsigned long)wcal_min_at[i],
                       wcal_max[i], (unsigned long)wcal_max_at[i]);
    }
    float Lc = wcal_c_n[0] ? wcal_c_sum[0] / wcal_c_n[0] : 0, Rc = wcal_c_n[3] ? wcal_c_sum[3] / wcal_c_n[3] : 0;
    hal_radio_wait_room(200, 20);
    if (wcal_set == 0) {
        con_printf("  touch readings: LEFT beam %u with the body on the left wall, RIGHT beam %u on the right\n",
                   wcal_min[0], wcal_min[3]);
        con_printf("  -> these are the per-beam contact zeros (SIDE_BEAM_CONTACT_MM, 38 compiled)\n");
        con_printf("  centre: L %.1f R %.1f -> (L - R)/2 = %+.1f mm of estimator zero bias; "
                   "centre-to-touch slope L %.3f R %.3f (49 mm of travel; 1.0 = calibrated lateral)\n",
                   (double)Lc, (double)Rc, (double)((Lc - Rc) * 0.5f),
                   (double)((Lc - wcal_min[0]) / 49.0f), (double)((Rc - wcal_min[3]) / 49.0f));
        if (wcal_sum_n > 10) {
            double mean = wcal_sum_s / wcal_sum_n;
            double var = wcal_sum_s2 / wcal_sum_n - mean * mean;
            if (var < 0) var = 0;
            con_printf("  L+R while both live: %.1f mean, sd %.1f over %lu packets (168 face to face "
                       "-> the corridor constant SWEEP_CORRIDOR_MM)\n", mean, sqrt(var),
                       (unsigned long)wcal_sum_n);
        }
        wcal_watch_start();
        return;
    }
    int i = wcal_set == 1 ? 0 : 3;
    float c = wcal_c_n[i] ? wcal_c_sum[i] / wcal_c_n[i] : 0;
    con_printf("  %s beam on the %s: centre %.1f (the DGATE per-side post constant, 57.6 compiled),\n"
               "  nearest %u (%.0f mm toward the post), furthest %u (%.0f mm away).\n",
               names[i], WCAL_SET_NAME[wcal_set], (double)c, wcal_min[i], (double)WCAL_POST_STEP_MM,
               wcal_max[i], (double)WCAL_POST_STEP_MM);
    con_printf("  slope toward %.3f mm/mm, away %.3f (1.0 = the reading is the lateral)\n",
               (double)((c - wcal_min[i]) / WCAL_POST_STEP_MM), (double)((wcal_max[i] - c) / WCAL_POST_STEP_MM));
    wcal_watch_start();
}

static void wcal_poll() {
    if (wcal_watch && !wcal_active) {
        if (any_test_running()) { wcal_watch = false; return; }   // something else owns the sensors
        sensors_poll();
        uint32_t now = millis();
        if (now - wcal_watch_last >= WCAL_WATCH_MS) {
            wcal_watch_last = now;
            WallRanges w = sensors_latest();
            hal_radio_wait_room(40, 5);
            con_printf("w,%u,%u,%u,%u\n", w.left_mm, w.front_left_mm, w.front_right_mm, w.right_mm);
        }
        return;
    }
    if (!wcal_active) return;
    sensors_poll();
    uint32_t now = millis();
    WallRanges w = sensors_latest();
    if (w.seq != wcal_last_seq) {
        wcal_last_seq = w.seq;
        gstat_add(wcal_stat[0], w.left_mm);
        gstat_add(wcal_stat[1], w.front_left_mm);
        gstat_add(wcal_stat[2], w.front_right_mm);
        gstat_add(wcal_stat[3], w.right_mm);
        if (wcal_sweep) {
            uint8_t b[4] = {w.left_mm, w.front_left_mm, w.front_right_mm, w.right_mm};
            for (int i = 0; i < 4; i++) {
                if (b[i] == 0 || b[i] == 255) continue;
                if (b[i] < wcal_min[i]) { wcal_min[i] = b[i]; wcal_min_at[i] = now - wcal_t0; }
                if (b[i] > wcal_max[i]) { wcal_max[i] = b[i]; wcal_max_at[i] = now - wcal_t0; }
                if (now - wcal_t0 < WCAL_CENTRE_MS) { wcal_c_sum[i] += b[i]; wcal_c_n[i]++; }
            }
            bool lv = w.left_mm != 0 && w.left_mm != 255;
            bool rv = w.right_mm != 0 && w.right_mm != 255;
            if (wcal_set == 0 && lv && rv) {
                double sum = (double)w.left_mm + w.right_mm;
                wcal_sum_s += sum; wcal_sum_s2 += sum * sum; wcal_sum_n++;
            }
        }
    }
    if (now - wcal_last_print >= WCAL_PRINT_MS) {
        wcal_last_print = now;
        WallView wv = walls_evaluate(w, sensors_ms_since_packet());
        hal_radio_wait_room(48, 10);
        con_printf("%lu,%u,%u,%u,%u,%.1f\n", (unsigned long)(now - wcal_t0),
                   w.left_mm, w.front_left_mm, w.front_right_mm, w.right_mm,
                   wv.lateral_valid ? wv.lateral_mm : 0.0f);
    }
    if (wcal_sweep) {
        if (now - wcal_t0 < WCAL_SWEEP_MS) return;
        wcal_sweep_finish();
        return;
    }
    if (now - wcal_t0 < WCAL_MS) return;
    wcal_active = false;
    static const char *names[4] = {"L", "FL", "FR", "R"};
    int set = wcal_set;
    con_println("beam,mean_mm,std_mm,min,max,n,oor");
    for (int i = 0; i < 4; i++) {
        const GStat &st = wcal_stat[i];
        hal_radio_wait_room(64, 10);
        if (st.n == 0) {
            con_printf("%s,-,-,-,-,0,%u\n", names[i], st.oor);
            wcal_mean[set][wcal_slot][i] = 0.0f;
            continue;
        }
        float mean = st.sum / st.n;
        float var = st.sum2 / st.n - mean * mean;
        if (var < 0.0f) var = 0.0f;
        con_printf("%s,%.2f,%.2f,%u,%u,%u,%u\n", names[i], mean, sqrtf(var),
                   st.mn, st.mx, st.n, st.oor);
        wcal_mean[set][wcal_slot][i] = mean;
    }
    int need = set == 0 ? -1 : (set == 1 ? 0 : 3);   // the beam that must see something
    wcal_have[set][wcal_slot] = need < 0 ? (wcal_stat[0].n > 0 && wcal_stat[3].n > 0)
                                         : wcal_stat[need].n > 0;
    if (!wcal_have[set][wcal_slot])
        con_println("** the side beam saw nothing -- not stored; check the setup **");
    else if (set == 0)
        con_printf("stored corridor %s: L %.1f  R %.1f  (%d of 3 placements)\n",
                   WCAL_SLOT_NAME[wcal_slot],
                   (double)wcal_mean[set][wcal_slot][0],
                   (double)wcal_mean[set][wcal_slot][3],
                   (int)wcal_have[set][0] + (int)wcal_have[set][1] +
                       (int)wcal_have[set][2]);
    else
        con_printf("stored %s: %s beam %.1f centred (post corner 55.1 from the line)\n",
                   WCAL_SET_NAME[set], names[need], (double)wcal_mean[set][0][need]);
    wcal_watch_start();
}

static void wcal_fit_corridor() {
    if (!(wcal_have[0][0] && wcal_have[0][1] && wcal_have[0][2])) {
        con_printf("wcal fit: CORRIDOR statics need all three placements (%c%c%c)\n",
                   wcal_have[0][0] ? 'c' : '-', wcal_have[0][1] ? 'l' : '-',
                   wcal_have[0][2] ? 'r' : '-');
        return;
    }
    float Lc = wcal_mean[0][0][0], Rc = wcal_mean[0][0][3];
    float Ll = wcal_mean[0][1][0], Rl = wcal_mean[0][1][3];
    float Lr = wcal_mean[0][2][0], Rr = wcal_mean[0][2][3];
    // Moving 10 mm LEFT: the left beam shortens, the right lengthens.
    float kL = (Lr - Ll) / 20.0f;   // mm of reading per mm of lateral
    float kR = (Rl - Rr) / 20.0f;
    con_println("wcal fit, CORRIDOR:");
    con_printf("  left beam : centre %.1f  slope %.3f mm/mm  (1.0 = calibrated lateral)\n",
               (double)Lc, (double)kL);
    con_printf("  right beam: centre %.1f  slope %.3f mm/mm\n", (double)Rc, (double)kR);
    con_printf("  centre offset (L - R)/2 = %+.1f mm -- the estimator's zero bias\n",
               (double)((Lc - Rc) * 0.5f));
    con_printf("  sum L+R: centre %.1f  left %.1f  right %.1f  (corridor: 168 walls)\n",
               (double)(Lc + Rc), (double)(Ll + Rl), (double)(Lr + Rr));
    con_println("  placement, truth, (R-L)/2, by L alone, by R alone   [mm, + = left]");
    for (int k = 0; k < 3; k++) {
        float L = wcal_mean[0][k][0], R = wcal_mean[0][k][3];
        float byL = (Lc - L) / (kL > 0.2f ? kL : 1.0f);
        float byR = (R - Rc) / (kR > 0.2f ? kR : 1.0f);
        con_printf("  %-11s %+5.1f  %+6.1f  %+6.1f  %+6.1f\n", WCAL_SLOT_NAME[k],
                   (double)WCAL_TRUE_LAT[k], (double)((R - L) * 0.5f),
                   (double)byL, (double)byR);
    }
}

static void wcal_fit_posts() {
    bool l = wcal_have[1][0], r = wcal_have[2][0];
    if (!l && !r) { con_println("wcal fit: no post statics yet (wcal lp / wcal rp)"); return; }
    con_println("wcal fit, POSTS (DGATE uses 57.6 - L for a left post, R - 57.6 for a right):");
    if (l) con_printf("  LEFT post : L beam %.1f centred -> per-side constant; D8 read 41 at 16 mm left\n",
                      (double)wcal_mean[1][0][0]);
    if (r) con_printf("  RIGHT post: R beam %.1f centred -> per-side constant; D8 read 57-60 at 2 mm left\n",
                      (double)wcal_mean[2][0][3]);
    if (l && r) con_printf("  L - R = %+.1f: the beams' disagreement on a centred post\n",
                           (double)(wcal_mean[1][0][0] - wcal_mean[2][0][3]));
}

static void wcal_command(const char *arg) {
    if (wcal_active && wcal_sweep) { wcal_sweep_finish(); return; }
    // A bare 'wcal' NEVER turns the beams off (bert: "the wcal (stop) is still
    // turning the beams off so I can't line up for next tests"): it stops a
    // sweep (into watch) and otherwise leaves the watch running. 'wcal off'
    // is the only thing that stops the beams.
    if (strcmp(arg, "off") == 0) {
        if (wcal_active) { con_println("wcal: a capture is running -- 'wcal' stops a sweep first"); return; }
        wcal_watch = false;
        sensors_stop();
        con_println("wcal: beams off");
        return;
    }
    if (arg[0] == '\0' && wcal_watch) {
        con_println("wcal: nothing to stop; the watch stays on ('wcal off' turns the beams off)");
        return;
    }
    if (arg[0] == '\0' || strcmp(arg, "help") == 0) { wcal_help(); return; }
    if (strcmp(arg, "w") == 0) {
        if (any_test_running()) { con_println("wcal: something is running -- stop it first"); return; }
        wcal_watch_start();
        return;
    }
    if (strcmp(arg, "x") == 0) {
        for (int s2 = 0; s2 < 3; s2++)
            for (int k = 0; k < 3; k++) wcal_have[s2][k] = false;
        con_println("wcal: captures cleared");
        return;
    }
    if (strcmp(arg, "fit") == 0) { wcal_fit_corridor(); wcal_fit_posts(); return; }
    int set = -1, slot = -2;
    if (strcmp(arg, "c") == 0)        { set = 0; slot = 0; }
    else if (strcmp(arg, "l") == 0)   { set = 0; slot = 1; }
    else if (strcmp(arg, "r") == 0)   { set = 0; slot = 2; }
    else if (strcmp(arg, "s") == 0)   { set = 0; slot = -1; }
    else if (strcmp(arg, "lp") == 0)  { set = 1; slot = 0; }
    else if (strcmp(arg, "rp") == 0)  { set = 2; slot = 0; }
    else if (strcmp(arg, "lps") == 0) { set = 1; slot = -1; }
    else if (strcmp(arg, "rps") == 0) { set = 2; slot = -1; }
    if (set < 0) { wcal_help(); return; }
    if (any_test_running()) { con_println("wcal: something is running -- stop it first"); return; }
    wcal_start(set, slot);
}

// -- Wall-referenced IMU alignment run -------------------------------------------
// Triggered by 'f'. The mouse steers on the IMU; the walls are ground truth.
// Runs a straight beside the wall(s) while feeding (fused distance, lateral)
// samples into WallAlign; at the end the fitted slope gives the heading error
// to the wall and the IMU is re-referenced (control_set_heading_deg) so the
// corridor direction reads as the heading the move STARTED at, corrected --
// i.e. after one 'f' in the boot corridor the maze axis is IMU zero. Boot use:
// the start->cell-2 move is 180 mm centre-to-centre heading north, with the
// left wall continuous across both cells; maze logic will run exactly this
// fit there. Correction is applied AFTER the move (never mid-move). Re-run
// 'f': the fitted error should be ~0.
//
// CSV per 20 ms: d,lat,lat_valid,yaw,rej

static constexpr float FTEST_DIST = 180.0f;  // mm: one cell, centre to centre
                                             // (the boot start->cell-2 move)
static constexpr float FTEST_V = 300.0f;     // mm/s cruise
static constexpr float FTEST_ACC = 500.0f;   // mm/s^2
static constexpr float FTEST_STEP_MM = 4.0f; // min distance between samples
static constexpr float FTEST_PRETURN_DEG = 1.0f;   // face the corridor first if
                                                   // more than this off 0
static constexpr float FTEST_YAW_GATE_DEG = 1.5f;  // only fit samples taken
                                                   // while the heading loop is
                                                   // on-reference (transients
                                                   // would contaminate the fit)

static WallAlign ftest_align;
static int ftest_state = -1;  // -1 idle, 0 pre-turn, 1 pause, 2 straight,
                              // 3 settle-then-report
static uint32_t ftest_t0 = 0;
static uint32_t ftest_last_print = 0;
static uint32_t ftest_last_seq = 0;
static float ftest_yaw0 = 0.0f;
static float ftest_last_d = 0.0f;

static void ftest_start_straight() {
    con_println("d,lat,lat_valid,yaw,rej");
    ftest_align.reset();
    control_motion_zero();
    sensors_free_run();
    ftest_yaw0 = control_heading().yaw_deg;
    ftest_last_d = -1e9f;
    ftest_last_seq = sensors_latest().seq;
    if (!control_move_start(FTEST_DIST, FTEST_V, FTEST_ACC)) {
        control_velocity_enable(false);
        sensors_stop();
        ftest_state = -1;
        con_println("move start refused -- fusion not ready?");
        return;
    }
    ftest_state = 2;
    ftest_t0 = millis();
    ftest_last_print = 0;
}

static void ftest_start() {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    con_println("wall-align run: ON FLOOR beside wall(s); corridor axis = "
                "heading 0. CSV:");
    control_velocity_enable(true);
    // The move must run down the corridor axis (heading 0), not wherever the
    // mouse happens to point -- after a previous re-reference the body can be
    // several degrees off. Same as real maze ops: turn to the cardinal first.
    float yaw_now = control_heading().yaw_deg;
    if (fabsf(yaw_now) > FTEST_PRETURN_DEG) {
        con_printf("facing corridor: in-place turn %+.2f deg first\n", -yaw_now);
        if (!control_turn_start(-yaw_now, 180.0f, 1200.0f)) {
            control_velocity_enable(false);
            con_println("pre-turn refused -- fusion not ready?");
            return;
        }
        ftest_state = 0;
        ftest_t0 = millis();
        return;
    }
    ftest_start_straight();
}

static void ftest_poll() {
    if (ftest_state < 0) return;
    uint32_t now = millis();

    if (ftest_state == 0) {  // pre-turn running
        if (!control_move_status().active) {
            ftest_state = 1;
            ftest_t0 = now;
        }
        return;
    }
    if (ftest_state == 1) {  // pause between turn and straight
        if (now - ftest_t0 >= 400) ftest_start_straight();
        return;
    }

    sensors_poll();  // drain the sub-MCU FIFO every loop iteration

    if (ftest_state == 2) {
        ControlMoveStatus st = control_move_status();
        WallRanges w = sensors_latest();
        WallView wv = walls_evaluate(w, sensors_ms_since_packet());

        // One sample per fresh packet, spaced at least FTEST_STEP_MM apart,
        // and only while the heading loop is on-reference: a yaw transient
        // means the travel direction is not yaw0, which would corrupt the
        // slope->heading-error fit.
        if (w.seq != ftest_last_seq && wv.lateral_valid &&
            fabsf(st.yaw_err_deg) < FTEST_YAW_GATE_DEG &&
            st.d_mm - ftest_last_d >= FTEST_STEP_MM) {
            ftest_last_seq = w.seq;
            ftest_last_d = st.d_mm;
            ftest_align.add(st.d_mm, wv.lateral_mm);
        }

        if (now - ftest_last_print >= 20) {
            ftest_last_print = now;
            con_printf("%.1f,%.1f,%d,%.2f,%d\n", st.d_mm, wv.lateral_mm,
                          wv.lateral_valid, control_heading().yaw_deg,
                          ftest_align.rejected());
        }
        if (!st.active) {
            ftest_state = 3;
            ftest_t0 = now;
        }
        return;
    }

    // ftest_state == 3: settle briefly, then fit + apply.
    if (now - ftest_t0 < 300) return;
    control_velocity_enable(false);
    sensors_stop();
    ftest_state = -1;

    con_printf("-- wall-align summary -- n=%d rejected=%d span=%.0f mm\n",
                  ftest_align.samples(), ftest_align.rejected(),
                  ftest_align.span_mm());
    if (!ftest_align.valid()) {
        con_println("fit NOT valid (need n>=20 over >=100 mm of wall; samples "
                    "only count with walls seen AND |yaw err| < 1.5 deg) -- "
                    "no correction applied");
        return;
    }
    float eps = ftest_align.yaw_err_deg();
    float off_end = ftest_align.offset_mm(ftest_last_d);
    con_printf("heading err to wall %+.2f deg | lateral offset at end "
               "%+.1f mm\n", eps, off_end);
    if (fabsf(eps) > WALL_ALIGN_MAX_CORR_DEG) {
        con_printf("correction > %.0f deg is implausible -- NOT applied\n",
                      WALL_ALIGN_MAX_CORR_DEG);
        return;
    }
    // The wall direction in the current IMU frame is (yaw0 - eps); subtract
    // it so the corridor axis reads 0. Applied between moves only.
    float yaw_now = control_heading().yaw_deg;
    float yaw_new = yaw_now - (ftest_yaw0 - eps);
    control_set_heading_deg(yaw_new);
    con_printf("IMU re-referenced: yaw %+.2f -> %+.2f deg (corridor = 0) -- "
               "re-run 'f' to verify (expect err ~0)\n", yaw_now, yaw_new);
}

// -- Front-wall stop test ---------------------------------------------------------
// Triggered by 'b'. Mouse ON FLOOR facing a front wall, close enough for the
// front beams to see it (present threshold 200 mm on the beam => front_mm up
// to ~220). Sequence: average the front distance stationary over
// BTEST_AVG_PACKETS, command a straight of exactly (front - BTEST_TARGET_MM),
// settle, re-average stationary, report:
//
//   stop error       final front_mm - target (what maze stops will feel)
//   travel check     odometry distance vs (front0 - front1): separates a
//                    front-GEOMETRY error (WALL_FRONT_X_MM / beam angles are
//                    nominal, unverified) from a stop-dynamics error --
//                    odometry is trusted ground truth over ~100 mm.
//
// Nothing is auto-applied: adjust WALL_FRONT_X_MM / BTEST_TARGET_MM from the
// report and re-run until the stop error centres on zero. The search-phase
// stop at a cell centre before a wall is exactly this maneuver.
//
// CSV per 20 ms while moving: d,front,v  (front -1 = wall not seen)

static constexpr float BTEST_TARGET_MM = 84.0f;  // front_mm at the stop point:
                                                 // cell centre to wall face =
                                                 // 90 - 6 inset (== SEARCH_FRONT_TARGET_MM)
static constexpr float BTEST_V = 300.0f;         // mm/s cruise (search speed)
static constexpr float BTEST_ACC = 500.0f;       // mm/s^2
static constexpr int   BTEST_AVG_PACKETS = 30;   // stationary average window
static constexpr float BTEST_MIN_MOVE_MM = 20.0f;   // refuse silly moves
static constexpr float BTEST_MAX_MOVE_MM = 400.0f;
static constexpr uint32_t BTEST_MEASURE_TIMEOUT_MS = 1500;

static int btest_state = -1;  // -1 idle, 0 measure start, 1 moving, 2 settle,
                              // 3 measure end
static uint32_t btest_t0 = 0;
static uint32_t btest_last_print = 0;
static uint32_t btest_last_seq = 0;
static float btest_front_sum = 0.0f;
static int btest_front_n = 0;
static float btest_front0 = 0.0f;   // stationary front average before the move
static float btest_move_mm = 0.0f;
static float btest_end_d = 0.0f;    // fused odometry distance at move end

// ---- 'R' RAW-PX MONITOR (auto-cal, 2026-08-06): stream the raw pixel +
// signal per channel to the console (verifies the raw-px Serial5 channel
// before the LUT sweep). Toggle on/off. Prints ~10 Hz.
static bool rawmon_active = false;
static uint32_t rawmon_last = 0;
static void rawmon_toggle() {
    if (rawmon_active) {
        rawmon_active = false;
        sensors_stop();
        con_println("raw-px monitor off");
    } else {
        rawmon_active = true;
        sensors_free_run_raw();
        con_println("raw-px monitor: px,sig per channel (L,FR,FL,R). "
                    "'R' again to stop");
    }
}
static void rawmon_poll() {
    if (!rawmon_active) return;
    sensors_poll();
    uint32_t now = millis();
    if (now - rawmon_last < 100) return;
    rawmon_last = now;
    WallRaw r = sensors_latest_raw();
    con_printf("px L%3u FR%3u FL%3u R%3u | sig %3u %3u %3u %3u\n",
               r.px[0], r.px[1], r.px[2], r.px[3],
               r.sig[0], r.sig[1], r.sig[2], r.sig[3]);
}

// ---- 'enc' WHEEL VELOCITY SOURCE MONITOR (edge timing, 2026-09-25): the
// count velocity and the edge-timed velocity side by side at 20 Hz while
// you spin a wheel by hand (or during any harness). The proof the XBAR/DMA
// capture is wired: a slow hand spin reads a steady number from the edge
// column and a 0/103/206 staircase from the count column; both agree on
// the sign; edges climb; overrun stays 0. Toggle; 'enc' again to stop.
static bool encmon_active = false;
static uint32_t encmon_last = 0;
static void encmon_toggle() {
    encmon_active = !encmon_active;
    if (encmon_active) {
        // Counts start from zero each time the monitor is switched on, so
        // "spin the wheel exactly ten turns" reads 820 per phase: a phase
        // that reads 810 is missing one slot per rev -- dust on the disc,
        // or a damaged slot (bert, 2026-09-25: "a clear spike in the
        // velocity every 67 mm").
        enc_timing_reset_counts();
        con_printf("enc monitor: count v | edge v | loop v | dir count/edge "
                   "(vsrc=%.0f, edge %s) -- edges L a+b / R a+b (82 per phase "
                   "per rev), since-edge us, overrun, dir mismatches. "
                   "'enc' again to stop\n",
                   (double)control_get_gains().vel_source,
                   enc_timing_ready() ? "ready" : "NOT READY");
    } else {
        con_println("enc monitor off");
    }
}
static void encmon_poll() {
    if (!encmon_active) return;
    uint32_t now = millis();
    if (now - encmon_last < 50) return;
    encmon_last = now;
    ControlWheelSources s = control_wheel_sources();
    EncTimingWheel L = enc_timing_left(), R = enc_timing_right();
    con_printf("L cnt %6.0f edge %6.0f loop %6.0f d%+d/%+d | "
               "R cnt %6.0f edge %6.0f loop %6.0f d%+d/%+d"
               " | e %lu+%lu/%lu+%lu since %.0f/%.0f ovr %lu/%lu dmm %lu/%lu%s\n",
               (double)s.count_l, (double)s.edge_l, (double)s.filt_l,
               L.dir_count, L.dir_edge,
               (double)s.count_r, (double)s.edge_r, (double)s.filt_r,
               R.dir_count, R.dir_edge,
               (unsigned long)L.total_a, (unsigned long)L.total_b,
               (unsigned long)R.total_a, (unsigned long)R.total_b,
               (double)L.since_edge_us, (double)R.since_edge_us,
               (unsigned long)L.overrun, (unsigned long)R.overrun,
               (unsigned long)L.dir_mismatch, (unsigned long)R.dir_mismatch,
               s.edge_in_use ? " *edge in use*" : "");
}

// ---- 'C' FRONT LUT SWEEP (self-cal rung 2, 2026-08-06): creep TOWARD a
// wall logging (raw front px -> odometry), stall-contact for the zero,
// then retroactively assign each px its TRUE forward distance = contact
// offset + (odo_at_contact - odo_at_sample). Builds lut_front_r / _l from
// physical odometry ground truth and prints them for comparison against
// the hand-tuned tables. Approach-and-log (no reverse); reuses the 'u'
// contact detector and the raw-px channel.
//
// Place the mouse facing a wall, ~250-300 mm back, in the clear.
//
// CENTRE-referenced (bert, 2026-08-08): at nose contact the mouse CENTRE
// is 45 mm from the wall surface (measured). So the LUT stores distance
// from the mouse CENTRE to the wall -- the pose reference the beam
// observer uses -- with NO guessed sensor offset. front_mm consumers
// (front-stop, observer) must expect centre-referenced distance.
static constexpr float CAL_FRONT_CONTACT_MM = 45.0f;  // centre->wall at contact (measured)
static constexpr float CAL_APPROACH_V = 50.0f;
static constexpr float CAL_SWEEP_V    = 25.0f;   // slow log creep
static constexpr float CAL_A          = 300.0f;
static constexpr float CAL_INRANGE_MM = 190.0f;  // front seen -> start logging
static constexpr int   CAL_SIG_MIN    = 110;     // accept sample if sig >=
static int   caltest_state = -1;   // -1 idle, 0 approach, 1 sweep+log
static float caltest_d0 = 0.0f;
static bool  caltest_moving = false;
static uint32_t caltest_ck_t = 0;
static float caltest_ck_d = 0.0f;
static uint32_t caltest_last_seq = 0;
// Accumulators: beam 0 = FR (raw px[1]), 1 = FL (raw px[2]).
static float    cal_odo_sum[2][256];
static uint16_t cal_cnt[2][256];
// Freshly-swept tables held in RAM until 'W' commits them to FRAM + uploads,
// indexed by SUB-MCU channel (0 left, 1 front_r, 2 front_l, 3 right). 'C'
// fills 1,2 (front); 'S' fills 0,3 (sides). cal_pending_mask bit<ch> = ready.
static uint8_t  cal_pending[4][256];
static int      cal_pending_lo[4], cal_pending_hi[4];
static uint8_t  cal_pending_mask = 0;

// Build one beam's 256-entry table into out[]; returns observed px lo/hi.
static void caltest_build_lut(int beam, uint8_t out[256], int &lo, int &hi) {
    float odo_contact = control_odometry().dist_mm - caltest_d0;
    lo = -1; hi = -1;
    float mm_at[256];
    for (int px = 0; px < 256; px++) {
        if (cal_cnt[beam][px] > 0) {
            float meanodo = cal_odo_sum[beam][px] / cal_cnt[beam][px];
            float mm = CAL_FRONT_CONTACT_MM + (odo_contact - meanodo);
            if (mm < 0) mm = 0;
            if (mm > 254) mm = 254;
            mm_at[px] = mm;
            if (lo < 0) lo = px;
            hi = px;
        } else {
            mm_at[px] = -1.0f;
        }
    }
    for (int px = 0; px < 256; px++) out[px] = 255;
    if (lo >= 0) {
        int prev = lo;                 // linear-interpolate gaps lo..hi
        for (int px = lo; px <= hi; px++) {
            if (mm_at[px] >= 0) {
                if (px > prev + 1 && mm_at[prev] >= 0) {
                    for (int g = prev + 1; g < px; g++) {
                        float f = (float)(g - prev) / (px - prev);
                        mm_at[g] = mm_at[prev] + f * (mm_at[px] - mm_at[prev]);
                    }
                }
                prev = px;
            }
        }
        for (int px = lo; px <= hi; px++)
            if (mm_at[px] >= 0) out[px] = (uint8_t)(mm_at[px] + 0.5f);
    }
}

static void caltest_print_lut(const char *name, const uint8_t lut[256],
                              int lo, int hi) {
    con_printf("%s  observed px %d..%d:\n", name, lo, hi);
    char line[128];
    for (int r = 0; r < 256; r += 16) {
        int n = 0;
        for (int px = r; px < r + 16; px++)
            n += snprintf(line + n, sizeof(line) - n, "%d,", lut[px]);
        con_println(line);
        delay(6);  // pace the radio
    }
}

static void caltest_build() {
    control_move_abort();
    control_velocity_enable(false);
    con_println("-- front LUT sweep done --");
    caltest_build_lut(0, cal_pending[1], cal_pending_lo[1], cal_pending_hi[1]);
    caltest_build_lut(1, cal_pending[2], cal_pending_lo[2], cal_pending_hi[2]);
    cal_pending_mask |= (1 << 1) | (1 << 2);   // front_r=ch1, front_l=ch2
    caltest_print_lut("lut_front_r (ch1)", cal_pending[1],
                      cal_pending_lo[1], cal_pending_hi[1]);
    caltest_print_lut("lut_front_l (ch2)", cal_pending[2],
                      cal_pending_lo[2], cal_pending_hi[2]);
    con_println("pending: press 'W' to commit to FRAM + upload to sub-MCU");
    sensors_stop();
    caltest_state = -1;
}

// 'W' -- commit the pending swept tables to the main-MCU FRAM and push
// them live to the sub-MCU (front_r = ch1, front_l = ch2). Boot re-uploads
// from FRAM. front_mm becomes CENTRE-referenced once this lands.
static const char *cal_ch_name(int ch) {
    return ch == 0 ? "left" : ch == 1 ? "front_r" : ch == 2 ? "front_l" : "right";
}

static void cal_commit_pending() {
    if (!cal_pending_mask) {
        con_println("no pending LUT -- run 'C' (front) or 'S' (sides) first");
        return;
    }
    sensors_stop();                        // ensure the sub is in triggered
    delay(20);                             // mode before the block upload
    con_println("committing pending channel(s) to FRAM + sub-MCU...");
    for (int ch = 0; ch < 4; ch++) {
        if (!(cal_pending_mask & (1 << ch))) continue;
        sensor_lut_store((uint8_t)ch, cal_pending[ch]);
        bool ok = sensors_upload_lut((uint8_t)ch, cal_pending[ch]);
        con_printf("  ch%d %-7s %s\n", ch, cal_ch_name(ch), ok ? "ACK" : "FAIL");
    }
    con_println("front_mm centre-ref; side left/right_mm are perp-from-centre");
}

static void caltest_start() {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    con_println("front LUT sweep (centre-referenced, 45 mm at contact): "
                "face a wall ~250-300 mm back, in clear");
    for (int b = 0; b < 2; b++)
        for (int i = 0; i < 256; i++) { cal_odo_sum[b][i] = 0; cal_cnt[b][i] = 0; }
    sensors_free_run_raw();
    control_velocity_enable(true);
    caltest_d0 = control_odometry().dist_mm;
    caltest_moving = false;
    caltest_ck_t = millis();
    caltest_ck_d = 0.0f;
    caltest_last_seq = sensors_latest_raw().seq;
    if (!control_move_start(400.0f, CAL_APPROACH_V, CAL_A)) {
        control_velocity_enable(false); sensors_stop();
        con_println("move refused -- fusion not ready?");
        return;
    }
    caltest_state = 0;
}

static void caltest_poll() {
    if (caltest_state < 0) return;
    sensors_poll();
    uint32_t now = millis();
    float d = control_odometry().dist_mm - caltest_d0;
    if (d > 2.0f) caltest_moving = true;

    // windowed stall = contact (same as the 'u' probe)
    if (now - caltest_ck_t >= 60) {
        float adv = d - caltest_ck_d;
        caltest_ck_t = now; caltest_ck_d = d;
        if (caltest_moving && adv < 0.15f) { caltest_build(); return; }
    }
    if (!control_move_status().active) { caltest_build(); return; }

    WallView wv = walls_evaluate(sensors_latest(), sensors_ms_since_packet());
    if (caltest_state == 0) {          // approach until the wall is in range
        if (wv.front && wv.front_mm < CAL_INRANGE_MM) {
            control_move_abort();
            control_move_start(400.0f, CAL_SWEEP_V, CAL_A);
            caltest_state = 1;
            con_println("  logging sweep (25 mm/s)");
        }
        return;
    }
    // state 1: log each fresh raw packet
    WallRaw r = sensors_latest_raw();
    if (r.seq != caltest_last_seq) {
        caltest_last_seq = r.seq;
        // beam 0 = FR (px[1]), beam 1 = FL (px[2])
        const int idx[2] = {1, 2};
        for (int b = 0; b < 2; b++) {
            uint8_t px = r.px[idx[b]], sig = r.sig[idx[b]];
            if (px != 0 && px != 255 && sig >= CAL_SIG_MIN) {
                cal_odo_sum[b][px] += d;
                cal_cnt[b][px] += 1;
            }
        }
    }
}

// ---- 'S' SIDE LUT SWEEP (auto-cal, 2026-08-08): calibrate the crossing
// side beams (ch0 reads the LEFT wall, ch3 the RIGHT) against odometry+IMU
// ground truth in a 2x4 special rig (outer walls only, no posts).
//
// Frame: origin on the south-wall surface, on the rig centreline; x=north
// (forward), y=left. The mouse starts rear-against-south-wall, laterally
// centred, facing north -> pose (SCAL_X0, 0, heading 0). Pose is integrated
// from wheel distance + IMU yaw through the WHOLE run (forward, turns), so
// move-settle slop never matters -- we log where the mouse ACTUALLY is.
//
// Two straight diagonals: up-left (sweeps the left wall near->far) then
// down-right (right wall). Per raw packet we raycast each side beam from its
// mount, at the live pose, to the known outer walls -> true along-beam
// range, then bake it to the yaw-0 perpendicular distance from centre
// d_perp = range*sin|angle| - 15 (front-consistent: the LUT stores perp,
// walls_evaluate passes it through). Bin pixel -> d_perp for ch0 / ch3.
static constexpr float SCAL_LEFT_Y  =  174.0f;   // 2 cells: (180-6) each side
static constexpr float SCAL_RIGHT_Y = -174.0f;
static constexpr float SCAL_NORTH_X =  708.0f;   // 4 cells: 4*180 - 2*6
static constexpr float SCAL_X0 = 45.0f;          // start centre N of south wall
static constexpr float SCAL_V = 40.0f, SCAL_A = 300.0f;
// A steep "V", BOTH legs pointing north (never face south, or the crossed
// beams swap which wall they read): up-left then up-right. Leg 1 sweeps the
// left wall (ch0), leg 2 the right wall (ch3). The vertex turn (80 deg) is
// near the left wall -- stop at NEAR=70 so the ~56 mm body half-diagonal
// clears it.
static constexpr float SCAL_HEAD1 =  40.0f;      // up-left (deg, world +CCW)
static constexpr float SCAL_HEAD2 = -40.0f;      // up-right (still facing N)
static constexpr float SCAL_NEAR_MM = 65.0f;     // stop: wall this close (perp)
                                                 // (~8 mm vertex-turn clearance)
static constexpr float SCAL_END_MARGIN = 80.0f;  // stop: this close to an end
static constexpr float SCAL_CAP_MM = 850.0f;     // per-run distance cap
static constexpr int   SCAL_SIG_MIN = 100;
static constexpr float SCAL_MAXRANGE = 200.0f;   // beam range validity
// mount geometry (mirror the sub-MCU laser arrays): ch0 left-wall reader,
// ch3 right-wall reader. x fwd, y left, angle = beam dir (deg, +CCW from +x).
static constexpr float SCAL_CH0_X = 40.0f, SCAL_CH0_Y = -15.0f, SCAL_CH0_A = 72.8f;
static constexpr float SCAL_CH3_X = 40.0f, SCAL_CH3_Y =  15.0f, SCAL_CH3_A = -71.0f;
static constexpr float DEG2RAD_S = 0.017453293f;

enum { SS_TURN1 = 0, SS_RUN1, SS_TURN2, SS_RUN2 };
static int   sctest_state = -1;
static float sctest_X, sctest_Y;          // live pose (rig frame, mm)
static float sctest_yaw_ref;             // fusion yaw at start (world north = 0)
static float sctest_prev_d;              // odometry dist at last integration
static uint32_t sctest_move_t, sctest_last_seq;
static double scal_sum[2][256];         // 0->ch0(left), 1->ch3(right)
static uint32_t scal_cnt[2][256];
static uint32_t sctest_dbg_t = 0;       // last live-debug print
static uint32_t sctest_ck_t = 0;        // stall window
static float sctest_ck_d = 0.0f;
static bool sctest_moving = false;
static uint32_t scal_run_n[2];          // samples logged this run (ch0/ch3)

static float sctest_world_yaw_deg() {
    return control_heading().yaw_deg - sctest_yaw_ref;   // 0 = north (+x)
}

// Raycast one beam from the live pose to WHICHEVER wall it actually hits
// (the crossed beams swap walls as the mouse yaws, so we don't assume one).
// Returns along-beam range to the nearest SIDE wall, or -1 if it misses / an
// end wall occludes it first / it's out of range.
static float sctest_raycast(float mx, float my, float ma_deg) {
    float wy = sctest_world_yaw_deg() * DEG2RAD_S;
    float c = cosf(wy), s = sinf(wy);
    float sx = sctest_X + mx * c - my * s;      // sensor world position
    float sy = sctest_Y + mx * s + my * c;
    float phi = wy + ma_deg * DEG2RAD_S;        // beam world direction
    float dx = cosf(phi), dy = sinf(phi);
    float best = 1e9f; bool best_side = false;
    // side walls (y = +/-174): valid hit if it lands within the rig length
    const float ywall[2] = {SCAL_LEFT_Y, SCAL_RIGHT_Y};
    for (int i = 0; i < 2; i++) {
        if (fabsf(dy) < 1e-3f) break;
        float t = (ywall[i] - sy) / dy;
        if (t <= 0.0f || t >= best) continue;
        float hx = sx + t * dx;
        if (hx >= 0.0f && hx <= SCAL_NORTH_X) { best = t; best_side = true; }
    }
    // end walls (x = 0 / NORTH): occluders -- if nearer than the side hit,
    // the beam is blocked and the sample is invalid.
    const float xwall[2] = {0.0f, SCAL_NORTH_X};
    for (int i = 0; i < 2; i++) {
        if (fabsf(dx) < 1e-3f) break;
        float t = (xwall[i] - sx) / dx;
        if (t <= 0.0f || t >= best) continue;
        float hy = sy + t * dy;
        if (hy >= SCAL_RIGHT_Y && hy <= SCAL_LEFT_Y) { best = t; best_side = false; }
    }
    if (!best_side || best > SCAL_MAXRANGE) return -1.0f;
    return best;
}

static void sctest_integrate() {
    float d = control_odometry().dist_mm;
    float ds = d - sctest_prev_d;
    sctest_prev_d = d;
    float wy = sctest_world_yaw_deg() * DEG2RAD_S;
    sctest_X += ds * cosf(wy);
    sctest_Y += ds * sinf(wy);
}

static void sctest_log_packet() {
    WallRaw r = sensors_latest_raw();
    if (r.seq == sctest_last_seq) return;
    sctest_last_seq = r.seq;
    // ch0 = raw px[0] -> bin 0 (lut_left); ch3 = raw px[3] -> bin 1 (lut_right).
    // Each channel's own beam, whichever wall it hits (perp is wall-agnostic).
    struct { int idx; float mx, my, a; int bin; } B[2] = {
        {0, SCAL_CH0_X, SCAL_CH0_Y, SCAL_CH0_A, 0},
        {3, SCAL_CH3_X, SCAL_CH3_Y, SCAL_CH3_A, 1}};
    for (int k = 0; k < 2; k++) {
        uint8_t px = r.px[B[k].idx], sig = r.sig[B[k].idx];
        if (px == 0 || px == 255 || sig < SCAL_SIG_MIN) continue;
        float range = sctest_raycast(B[k].mx, B[k].my, B[k].a);
        if (range < 0.0f) continue;
        float dperp = range * sinf(fabsf(B[k].a) * DEG2RAD_S) - 15.0f;
        if (dperp < 1.0f || dperp > 200.0f) continue;
        scal_sum[B[k].bin][px] += dperp;
        scal_cnt[B[k].bin][px] += 1;
        scal_run_n[B[k].bin] += 1;
    }
}

static void sctest_start_run(float head_deg) {
    float rel = (sctest_yaw_ref + head_deg) - control_heading().yaw_deg;
    while (rel > 180.0f) rel -= 360.0f;
    while (rel < -180.0f) rel += 360.0f;
    control_turn_start(rel, 120.0f, 800.0f);
    sctest_move_t = millis();
}

static void sctest_build_channel(int bin, int ch, const char *name) {
    uint8_t lut[256];
    for (int px = 0; px < 256; px++) lut[px] = 255;
    int lo = -1, hi = -1;
    float mm_at[256];
    for (int px = 0; px < 256; px++) {
        if (scal_cnt[bin][px] > 0) {
            float mm = (float)(scal_sum[bin][px] / scal_cnt[bin][px]);
            mm_at[px] = mm; if (lo < 0) lo = px; hi = px;
        } else mm_at[px] = -1.0f;
    }
    if (lo >= 0) {
        int prev = lo;
        for (int px = lo; px <= hi; px++) {
            if (mm_at[px] >= 0) {
                if (px > prev + 1 && mm_at[prev] >= 0)
                    for (int g = prev + 1; g < px; g++)
                        mm_at[g] = mm_at[prev] + (mm_at[px] - mm_at[prev]) *
                                   (float)(g - prev) / (px - prev);
                prev = px;
            }
        }
        for (int px = lo; px <= hi; px++)
            if (mm_at[px] >= 0) {
                int v = (int)(mm_at[px] + 0.5f);
                lut[px] = (uint8_t)(v < 1 ? 255 : v > 254 ? 254 : v);
            }
    }
    for (int px = 0; px < 256; px++) cal_pending[ch][px] = lut[px];
    cal_pending_lo[ch] = lo; cal_pending_hi[ch] = hi;
    cal_pending_mask |= (uint8_t)(1 << ch);
    caltest_print_lut(name, cal_pending[ch], lo, hi);
}

static void sctest_build() {
    control_move_abort();
    control_velocity_enable(false);
    sensors_stop();
    con_println("-- side LUT sweep done (perp-from-centre) --");
    // pixel-cell coverage per channel (how many pixels got >=1 sample)
    int covL = 0, covR = 0;
    for (int px = 0; px < 256; px++) {
        if (scal_cnt[0][px]) covL++;
        if (scal_cnt[1][px]) covR++;
    }
    con_printf("coverage: left %d px, right %d px\n", covL, covR);
    if (covL + covR == 0) {
        con_println("NO SAMPLES logged -- check placement/pose (run with debug)");
        sctest_state = -1;
        return;
    }
    sctest_build_channel(0, 0, "lut_left (ch0)");
    sctest_build_channel(1, 3, "lut_right (ch3)");
    con_println("pending: press 'W' to commit to FRAM + upload to sub-MCU");
    sctest_state = -1;
}

static void sctest_begin_run(int run_state) {
    control_move_start(SCAL_CAP_MM, SCAL_V, SCAL_A);
    sctest_move_t = millis();
    sctest_ck_t = millis(); sctest_ck_d = control_odometry().dist_mm;
    sctest_moving = false;
    scal_run_n[0] = scal_run_n[1] = 0;
    sctest_state = run_state;
}

static void sctest_poll() {
    if (sctest_state < 0) return;
    sensors_poll();          // keep the raw stream drained in every state
    sctest_integrate();
    uint32_t now = millis();
    bool moving = control_move_status().active;

    if (sctest_state == SS_TURN1) {
        if (now - sctest_move_t > 80 && !moving) sctest_begin_run(SS_RUN1);
        return;
    }
    if (sctest_state == SS_TURN2) {
        if (now - sctest_move_t > 80 && !moving) sctest_begin_run(SS_RUN2);
        return;
    }
    // SS_RUN1 / SS_RUN2
    sctest_log_packet();
    float dpl = SCAL_LEFT_Y - sctest_Y;     // perp to left wall
    float dpr = sctest_Y - SCAL_RIGHT_Y;    // perp to right wall
    float wy = sctest_world_yaw_deg();

    if (now - sctest_dbg_t >= 200) {        // live telemetry
        sctest_dbg_t = now;
        con_printf("S%d X%.0f Y%.0f yaw%.1f dL%.0f dR%.0f nL%lu nR%lu\n",
                   sctest_state == SS_RUN1 ? 1 : 2, sctest_X, sctest_Y, wy,
                   dpl, dpr, scal_run_n[0], scal_run_n[1]);
    }

    // stall guard (windowed): wheels commanded but not advancing = wall/jam
    bool stalled = false;
    float d = control_odometry().dist_mm;
    if (d - sctest_ck_d > 2.0f) sctest_moving = true;
    if (now - sctest_ck_t >= 100) {
        if (sctest_moving && (d - sctest_ck_d) < 0.3f) stalled = true;
        sctest_ck_t = now; sctest_ck_d = d;
    }

    bool near = (sctest_state == SS_RUN1) ? (dpl < SCAL_NEAR_MM)
                                          : (dpr < SCAL_NEAR_MM);
    bool endwall = (sctest_state == SS_RUN1) ? (sctest_X > SCAL_NORTH_X - SCAL_END_MARGIN)
                                             : (sctest_X < SCAL_END_MARGIN);
    bool ended = (now - sctest_move_t > 80 && !moving);
    if (near || endwall || ended || stalled) {
        const char *why = near ? "near-wall" : endwall ? "end" :
                          stalled ? "STALL" : "move-end";
        con_printf("S run %d done (%s) nL%lu nR%lu\n",
                   sctest_state == SS_RUN1 ? 1 : 2, why,
                   scal_run_n[0], scal_run_n[1]);
        control_move_abort();
        if (sctest_state == SS_RUN1) {
            sctest_start_run(SCAL_HEAD2); sctest_state = SS_TURN2;
        } else {
            sctest_build();
        }
    }
}

static void sctest_start() {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    con_println("side sweep: 2x4 rig, mouse rear-against-south-wall, centred, "
                "facing north. Two diagonals (up-left, down-right).");
    for (int b = 0; b < 2; b++)
        for (int i = 0; i < 256; i++) { scal_sum[b][i] = 0; scal_cnt[b][i] = 0; }
    cal_pending_mask &= (uint8_t)~((1 << 0) | (1 << 3));  // clear side pends
    sensors_free_run_raw();
    control_velocity_enable(true);
    sctest_X = SCAL_X0; sctest_Y = 0.0f;
    sctest_yaw_ref = control_heading().yaw_deg;   // define world-north = now
    sctest_prev_d = control_odometry().dist_mm;
    sctest_last_seq = sensors_latest_raw().seq;
    sctest_start_run(SCAL_HEAD1);
    sctest_state = SS_TURN1;
}

// ---- 'u' CONTACT PROBE (self-cal rung 1, 2026-08-06): gentle two-phase
// creep to a wall, detect contact by WHEEL STALL (encoders stop while the
// motors are still commanded forward -- the wheels can't turn against the
// wall), stop dead, and report the odometry distance travelled + the front
// beam at contact. Run it repeatedly against one wall: the spread of the
// reported distance IS the contact-repeatability = the ground-truth
// resolution the whole self-cal rests on. First buildable piece of the
// auto-cal (see sensor_autocal_design.md); accel-spike refinement + the
// survey/sweeps come after this proves out.
static int utest_state = -1;   // -1 idle, 0 approach(60), 1 creep(10)
static float utest_d0 = 0.0f;
static bool utest_moving = false;
static uint32_t utest_check_t = 0;   // last stall-window sample time
static float utest_check_d = 0.0f;   // distance at that sample
static constexpr float UTEST_APPROACH_V = 60.0f;   // brisk crawl to close in
static constexpr float UTEST_CREEP_V    = 12.0f;   // gentle final approach
static constexpr float UTEST_A          = 400.0f;  // mm/s^2, soft
static constexpr float UTEST_SLOW_AT_MM = 40.0f;   // front dist -> slow phase
// STALL = distance-windowed, NOT instantaneous velocity: at 12 mm/s the
// encoder ticks < once per 500 Hz loop (0.024 mm/tick vs 0.206 mm/count),
// so velocity reads in zero-gap bursts and a v-threshold false-fires
// instantly (2026-08-06). Over a 60 ms window the mouse should advance
// ~0.7 mm at creep; < 0.15 mm means the wheels aren't turning = contact.
static constexpr uint32_t UTEST_WIN_MS   = 60;
static constexpr float    UTEST_STALL_MM = 0.15f;  // advance/window -> stalled
static constexpr float    UTEST_ARM_MM   = 2.0f;   // clearly moving first
static constexpr float UTEST_MAX_MM     = 400.0f;  // creep target; abort on hit

static void utest_finish(const char *why) {
    control_move_abort();
    float d = control_odometry().dist_mm - utest_d0;
    WallView wv = walls_evaluate(sensors_latest(), sensors_ms_since_packet());
    con_printf("%s: odo %.1f mm | front %.0f mm\n", why, d,
               wv.front ? wv.front_mm : 255.0f);
    control_velocity_enable(false);
    sensors_stop();
    utest_state = -1;
}

static void utest_start() {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    con_println("contact probe: two-phase creep, stall-stop at the wall");
    sensors_free_run();
    control_velocity_enable(true);
    utest_d0 = control_odometry().dist_mm;
    utest_moving = false;
    utest_check_t = millis();
    utest_check_d = 0.0f;
    if (!control_move_start(UTEST_MAX_MM, UTEST_APPROACH_V, UTEST_A)) {
        control_velocity_enable(false);
        sensors_stop();
        con_println("move refused -- fusion not ready?");
        return;
    }
    utest_state = 0;
}

static void utest_poll() {
    if (utest_state < 0) return;
    sensors_poll();
    uint32_t now = millis();
    float d = control_odometry().dist_mm - utest_d0;
    if (d > UTEST_ARM_MM) utest_moving = true;       // clearly moving first

    // Distance-windowed stall check every UTEST_WIN_MS.
    if (now - utest_check_t >= UTEST_WIN_MS) {
        float adv = d - utest_check_d;
        utest_check_t = now;
        utest_check_d = d;
        if (utest_moving && adv < UTEST_STALL_MM) {
            utest_finish("CONTACT (stall)");
            return;
        }
    }
    if (!control_move_status().active) { utest_finish("END (no contact)"); return; }

    if (utest_state == 0) {  // approach: slow when the front wall is near
        WallView wv = walls_evaluate(sensors_latest(),
                                     sensors_ms_since_packet());
        if (wv.front && wv.front_mm < UTEST_SLOW_AT_MM) {
            control_move_abort();
            control_move_start(UTEST_MAX_MM, UTEST_CREEP_V, UTEST_A);
            utest_state = 1;
            con_println("  wall near -- gentle creep (12 mm/s)");
        }
    }
}

// ---- 'j' SCALE TEST (bert, 2026-08-03): drive N cells straight for the
// circ calibration. Command a known distance at a slip-free speed; the
// operator measures the PHYSICAL travel with a ruler; circ_new = circ *
// physical / commanded. Wifi-triggered = it's just a radio console cmd.
static int jtest_state = -1;
static float jtest_cmd_mm = 900.0f;   // 5 cells
static float jtest_d0 = 0.0f, jtest_l0 = 0.0f, jtest_r0 = 0.0f;
static constexpr float JTEST_V = 300.0f;   // slow: zero wheel slip
static constexpr float JTEST_A = 600.0f;
static void jtest_start(const char *args) {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    jtest_cmd_mm = 900.0f;
    if (args && args[0]) {
        float v = atof(args);
        if (v >= 90.0f && v <= 2880.0f) jtest_cmd_mm = v;
    }
    con_printf("scale test: driving %.0f mm (%.1f cells) straight -- "
               "measure physical travel, then circ_new = circ * phys / %.0f\n",
               jtest_cmd_mm, jtest_cmd_mm / 180.0f, jtest_cmd_mm);
    control_velocity_enable(true);
    { ControlOdometry o = control_odometry();
      jtest_d0 = o.dist_mm; jtest_l0 = o.left_mm; jtest_r0 = o.right_mm; }
    if (!control_move_start(jtest_cmd_mm, JTEST_V, JTEST_A)) {
        control_velocity_enable(false);
        con_println("move refused -- fusion not ready?");
        return;
    }
    jtest_state = 0;
}
static void jtest_poll() {
    if (jtest_state < 0) return;
    if (control_move_status().active) return;
    ControlOdometry o = control_odometry();
    ControlGains g = control_get_gains();
    float bl = o.left_mm - jtest_l0, br = o.right_mm - jtest_r0;
    con_printf("scale test done: believed L %.1f  R %.1f mm (cmd %.0f)\n",
               bl, br, jtest_cmd_mm);
    con_printf("  measure PHYSICAL travel, then: "
               "circl = %.2f * phys / %.1f ; circr = %.2f * phys / %.1f\n",
               g.circ_left_mm, bl, g.circ_right_mm, br);
    control_velocity_enable(false);
    jtest_state = -1;
}

// ---- 'slip' LAUNCH TEST (bert, 2026-09-23: "the mouse acceleration is
// still below what is expected. what steps can be taken to find the slip
// limits"). A triangular profile on a straight -- accelerate at A to the
// peak, brake at A to rest -- with the tyre read THREE ways at once:
//   1. wheel vs body: the encoder velocity gained over the pull against
//      the accelerometer's integral over the same time (v_imu, the
//      tilt-compensated forward accel integrated since zero). A wheel that
//      spins gains velocity the body does not: slip% = (dv_wheel -
//      dv_body) / dv_wheel. Same for the brake (a sliding wheel loses
//      velocity the body has not).
//   2. duty: the peak commanded duty in each phase. 0.85 reached before the
//      tyre lets go means the MOTOR is the limit at that rung, not grip --
//      and the two answers call for different fixes.
//   3. the ruler: believed distance against the world. Place the mouse a
//      measured distance from a front wall (the wall must be inside the
//      beam's range at the END, so start from about mm + 100 away); the
//      front beam at rest afterwards is the truth, and the difference from
//      the commanded distance is the slip in millimetres.
// Ladder A up from the flown 1200 until slip% leaves zero (grip) or duty
// hits the wall (motor). Pure longitudinal, from rest: the launch case.
// The mid-run limit -- torque on a tyre still carrying cornering load --
// is the ralon ladder of SLIP_LOOP.md, a different number.
static int slip_state = -1;   // -2 measuring the IMU bias (stationary), 0 accelerating, 1 braking
// THE FIRST LADDER (2026-09-23, 1200..16000 on D9, 15 logs) taught the
// test three things, all built in below:
//  (a) the IMU integral has a ~150 mm/s2 standing bias (tilt), so at low A
//      with long ramps the summary read 5-15 % of "slip" that the ruler
//      (906-917 believed, end wall +/-5 mm at every rung) flatly denied.
//      Now the bias is measured stationary for SLIP_BIAS_MS before the
//      launch and taken out of both phases.
//  (b) the peak was taken from a dip in the noisy wheel velocity, not the
//      profile; now it is the profile's turnover (a_ref < 0).
//  (c) above ~4000 the WHEELS did not follow the profile: the P-only
//      velocity loop (kp 0.0003, ka 0) needs a 1000 mm/s error to command
//      0.3 duty, so a 16000 ramp produced ~6500 wheel mm/s2 on average
//      with the duty at 0.6 of 0.85 -- and the body followed the wheels
//      within 5 %. The limit that showed was the CONTROLLER, not the tyre
//      (no tyre limit seen up to ~1 g of body acceleration late in the
//      pull). So the report now says what acceleration was ACHIEVED, and
//      names the controller when the wheels fell short with duty to spare.
static float slip_bias = 0.0f, slip_bias_sum = 0.0f;
static uint32_t slip_tbk = 0, slip_tvr0 = 0, slip_tbody0 = 0;
static float slip_v_wheel_bk = 0.0f, slip_v_imu_bk = 0.0f;
static float slip_wheel_min = 0.0f;      // most negative wheel v in the brake
static float slip_body_at_lock = 0.0f;   // body v when the wheels first stopped
static bool slip_timed_out = false;
// SECOND LADDER, 2026-09-24 (ka 5e-5): the brake phase was mis-reported
// -- its duration ran to the END OF THE MOVE, which after a hard stop is
// a second of the position loop crawling the last 30 mm, so 8000 printed
// as "achieved 1879 (23 %) << CONTROLLER LIMIT" while the time series
// showed the body shedding 2500 mm/s in 300 ms (7970). Now the brake is
// timed to v_ref reaching zero and the body's own stop, and the one
// thing that matters in a brake is reported by name: THE LOCK. At 10000
// and 12000 the loop drove the wheels to -500 while the body was still
// doing 2000 -- the tyre let go, the odometry counted nothing, and the
// mouse slid 250 mm into the end wall (believed 744-751 of 900).
static int slip_bias_n = 0;
static constexpr uint32_t SLIP_BIAS_MS = 300;
static float slip_a = 1200.0f, slip_mm = 900.0f, slip_vpk = 0.0f;
static float slip_d0 = 0.0f;
static float slip_v_wheel_pk = 0.0f, slip_v_imu_pk = 0.0f, slip_duty_acc = 0.0f;
static float slip_v_wheel_max = 0.0f;
static float slip_front0 = 0.0f;
static uint32_t slip_t0 = 0, slip_tpk = 0;
static constexpr float SLIP_V_CAP = 3500.0f;   // 2500 until 2026-09-25: the big floor
static uint32_t slip_log_seq = 0;
// THE TIME SERIES (bert, 2026-09-23: on an earlier mouse a hard standing
// start spun the wheels a full turn, which WIPED THE DUST off them; the
// clean rubber then bit, traction jumped, and the wheel velocity DROPPED
// as the body finally moved). A per-phase slip % averages that transient
// away, so the test also records the flight log at the packet rate, with
// the columns re-purposed for the launch:
//   s = wheel odometry (mm)     v = WHEEL velocity      ax = body accel
//   obs_along = IMU velocity (v_imu)   obs_cross = commanded v_ref
//   wf = wheel - fused distance        slide = duty x 100 (left wheel)
// 'y' dumps it; the first tens of ms are where the dust story shows.
static void slip_log_packet(uint32_t now) {
    WallRanges w = sensors_latest();
    if (w.seq == slip_log_seq) return;
    slip_log_seq = w.seq;
    ControlMotion mo = control_motion();
    ControlOdometry od = control_odometry();
    ControlHeading hd = control_heading();
    ControlMoveStatus ms = control_move_status();
    ControlVelStatus vs = control_velocity_status();
    FlogRec r;
    memset(&r, 0, sizeof r);
    r.t_ms = (uint16_t)(now - slip_t0);
    r.s_mm = (int16_t)(od.dist_mm - slip_d0);
    r.v_mm_s = (int16_t)od.vel_mm_s;
    r.yaw_cdeg = (int16_t)(hd.yaw_deg * 10.0f);
    r.yaw_err_cdeg = (int16_t)(ms.yaw_err_deg * 10.0f);
    r.ay_mm_s2 = (int16_t)mo.a_lat_mm_s2;
    r.ax_mm_s2 = (int16_t)mo.a_fwd_mm_s2;
    float d100 = vs.duty_left * 100.0f;
    r.slide_mm = (int8_t)(d100 > 127.0f ? 127.0f : (d100 < -127.0f ? -127.0f : d100));
    r.flags = (uint8_t)((slip_state == 2 ? 0x40 : (slip_state == 1 ? 0x80 : 0)) |
                        (mo.abs_active ? 0x20 : 0));   // bit6 braking, bit7 cruise, bit5 ABS acting
    r.left_mm = w.left_mm;
    r.right_mm = w.right_mm;
    r.front_left_mm = w.front_left_mm;
    r.front_right_mm = w.front_right_mm;
    r.wheel_fused_mm = (int16_t)(od.dist_mm - slip_d0 - mo.d_est_mm);
    r.omega_dps = (int16_t)mo.v_abs_mm_s;   // the ABS's body velocity, in the omega column
    r.obs_along_mm = (int16_t)mo.v_imu_mm_s;
    r.obs_cross_mm = (int16_t)ms.v_ref;
    // WHICH WHEEL (2026-09-25): log 18-02-00 showed the wheel spinning up
    // once per revolution on the launch (s = 31, 99, 164, 240 -- +527,
    // +354, +205, +55 over v_ref, duty pulled down, ax halved, the ABS
    // catching the later ones) and v is the mean of both wheels, so the
    // log could not say which tyre. The left-minus-right wheel velocity
    // goes in the obs_head column (unused on a straight).
    r.obs_head_cdeg = (int16_t)(od.vel_left_mm_s - od.vel_right_mm_s);
    flog_record(r);
}
static void slip_start(const char *args) {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    slip_a = 1200.0f; slip_mm = 900.0f;
    if (args && args[0]) {
        slip_a = atof(args);
        const char *c = strchr(args, ',');
        if (c) slip_mm = atof(c + 1);
    }
    if (slip_a < 300.0f) slip_a = 300.0f;
    if (slip_a > 20000.0f) slip_a = 20000.0f;
    if (slip_mm < 180.0f) slip_mm = 180.0f;
    if (slip_mm > 2880.0f) slip_mm = 2880.0f;
    // Triangular: v_pk = sqrt(a * d) (accelerate for half the distance,
    // brake for the other half), capped so a long run at a soft A does
    // not become a speed run.
    slip_vpk = sqrtf(slip_a * slip_mm);
    if (slip_vpk > SLIP_V_CAP) slip_vpk = SLIP_V_CAP;
    {
        WallView wv = walls_evaluate(sensors_latest(), sensors_ms_since_packet());
        slip_front0 = (wv.valid && wv.front) ? wv.front_mm : -1.0f;
    }
    con_printf("slip test: %.0f mm straight, accelerate at %.0f mm/s2 to "
               "%.0f mm/s, brake at %.0f to rest%s\n", slip_mm, slip_a,
               slip_vpk, slip_a,
               slip_front0 >= 0 ? "" : " (no front wall in range at the "
                                      "start -- the ruler is the END reading "
                                      "only)");
    sensors_free_run_interlaced();
    control_motion_zero();
    // Stationary first: the accelerometer's forward reading with the mouse
    // still IS the bias the integral will carry (tilt of the board, the
    // part's own offset). Averaged over SLIP_BIAS_MS, then the launch.
    slip_bias_sum = 0.0f; slip_bias_n = 0;
    slip_t0 = millis();
    slip_state = -2;
}
static void slip_launch() {
    slip_bias = slip_bias_n ? slip_bias_sum / slip_bias_n : 0.0f;
    con_printf("  IMU forward bias %+.0f mm/s2 (stationary, %d samples) -- "
               "taken out of the body velocities below\n", (double)slip_bias,
               slip_bias_n);
    control_velocity_enable(true);
    control_motion_zero();
    control_duty_peak_reset();
    { ControlOdometry o = control_odometry(); slip_d0 = o.dist_mm; }
    slip_v_wheel_max = 0.0f;
    slip_timed_out = false;
    slip_t0 = millis();
    slip_log_seq = sensors_latest().seq;
    flog_arm();
    {
        char b[160];
        snprintf(b, sizeof b, "slip:launch A%.0f mm%.0f vpk%.0f bias%.0f cols:s=wheel "
                 "v=wheel ax=body along=v_imu cross=v_ref wf=wheel-fused "
                 "slide=duty%% omega=v_abs head=vl-vr", (double)slip_a, (double)slip_mm, (double)slip_vpk,
                 (double)slip_bias);
        flog_set_banner(b);
    }
    if (!control_move_start(slip_mm, slip_vpk, slip_a)) {
        flog_stop();
        control_velocity_enable(false);
        con_println("move refused -- fusion not ready?");
        slip_state = -1;
        return;
    }
    slip_state = 0;
}
static void slip_poll() {
    if (slip_state == -1) return;
    sensors_poll();
    if (slip_state == -2) {
        ControlMotion mb = control_motion();
        slip_bias_sum += mb.a_fwd_mm_s2; slip_bias_n++;
        if (millis() - slip_t0 >= SLIP_BIAS_MS) slip_launch();
        return;
    }
    slip_log_packet(millis());
    ControlOdometry o = control_odometry();
    ControlMotion m = control_motion();
    if (slip_state == 0) {
        if (o.vel_mm_s > slip_v_wheel_max) slip_v_wheel_max = o.vel_mm_s;
        // The peak is the PROFILE's turnover (a_ref goes negative), not a
        // dip in the wheel velocity: read both velocities at that instant
        // and reset the duty peak for the brake.
        ControlMoveStatus ms = control_move_status();
        // The ramp ends when a_ref drops to 0 (a cruise at the 2500 cap)
        // or below (straight into the brake).
        bool over = !ms.active || (ms.a_ref < 1.0f && millis() - slip_t0 > 20);
        if (!over) return;
        slip_v_wheel_pk = o.vel_mm_s;
        slip_v_imu_pk = m.v_imu_mm_s;
        slip_duty_acc = control_velocity_status().duty_peak;
        control_duty_peak_reset();
        slip_tpk = millis();
        slip_tbk = slip_tpk;
        slip_v_wheel_bk = slip_v_wheel_pk;
        slip_v_imu_bk = slip_v_imu_pk;
        slip_state = 1;
        if (ms.active) return;
    }
    if (slip_state == 1) {
        // Cruise (if any) until the brake begins: the brake phase is
        // measured from ITS start, not from the top of the ramp.
        ControlMoveStatus ms = control_move_status();
        if (ms.active && ms.a_ref > -1.0f) return;
        slip_tbk = millis();
        slip_tvr0 = 0; slip_tbody0 = 0;
        slip_v_wheel_bk = o.vel_mm_s;
        slip_v_imu_bk = m.v_imu_mm_s;
        slip_wheel_min = o.vel_mm_s;
        slip_body_at_lock = 0.0f;
        control_duty_peak_reset();
        slip_state = 2;
        if (ms.active) return;
    }
    if (slip_state == 2) {
        ControlMoveStatus ms = control_move_status();
        if (!slip_tvr0 && ms.v_ref <= 0.5f) slip_tvr0 = millis();
        if (!slip_tbody0 && m.v_imu_mm_s < 100.0f) slip_tbody0 = millis();
        if (o.vel_mm_s < slip_wheel_min) {
            slip_wheel_min = o.vel_mm_s;
            if (o.vel_mm_s < 20.0f && slip_body_at_lock == 0.0f)
                slip_body_at_lock = m.v_imu_mm_s;   // the body when the wheels stopped
        }
        // A move that never finishes is a collision (the wall took the
        // last of the distance): abort it, report what there is.
        if (ms.active && millis() - slip_tbk > 3000) {
            control_move_abort();
            slip_timed_out = true;
        }
    }
    if (control_move_status().active) return;
    // Settle a moment so the front beam reads a stationary mouse.
    static uint32_t done_t = 0;
    if (done_t == 0) { done_t = millis(); return; }
    if (millis() - done_t < 300) return;
    done_t = 0;
    float duty_brk = control_velocity_status().duty_peak;
    float t_up_s = (slip_tpk - slip_t0) / 1000.0f;
    // The brake lasts until the PROFILE reaches zero -- not until the move
    // ends -- and the body's own stop (IMU velocity under 100) is timed too.
    uint32_t t_end = slip_tvr0 ? slip_tvr0 : millis() - 300;
    float t_dn_s = (t_end - slip_tbk) / 1000.0f;
    float t_body_s = slip_tbody0 ? (slip_tbody0 - slip_tbk) / 1000.0f : 0.0f;
    // Bias-corrected body velocities: the integral of a constant bias b
    // over each phase is b x t, and it is the same sign both ways (the
    // integral keeps climbing through the brake), which is why the raw
    // numbers read +slip accelerating and -slip braking at every rung.
    float dv_w_acc = slip_v_wheel_pk;
    float dv_b_acc = slip_v_imu_pk - slip_bias * t_up_s;
    float dv_w_brk = slip_v_wheel_bk - o.vel_mm_s;
    float dv_b_brk = (slip_v_imu_bk - m.v_imu_mm_s) + slip_bias * t_dn_s;
    float sl_acc = dv_w_acc > 1.0f ? (dv_w_acc - dv_b_acc) / dv_w_acc * 100.0f : 0.0f;
    float sl_brk = dv_w_brk > 1.0f ? (dv_w_brk - dv_b_brk) / dv_w_brk * 100.0f : 0.0f;
    float a_ach = t_up_s > 0.01f ? dv_w_acc / t_up_s : 0.0f;   // wheel mm/s2 achieved
    float b_ach = t_dn_s > 0.01f ? dv_w_brk / t_dn_s : 0.0f;
    float lim = control_get_gains().duty_limit;
    bool motor_acc = slip_duty_acc >= lim - 0.01f;
    bool motor_brk = duty_brk >= lim - 0.01f;
    bool ctrl_acc = !motor_acc && a_ach < 0.8f * slip_a;
    bool ctrl_brk = !motor_brk && b_ach < 0.8f * slip_a;
    float believed = o.dist_mm - slip_d0;
    WallView wv = walls_evaluate(sensors_latest(), sensors_ms_since_packet());
    float front1 = (wv.valid && wv.front) ? wv.front_mm : -1.0f;
    con_printf("slip test done: A %.0f asked, peak %.0f mm/s, %.0f ms up / %.0f ms down\n",
               slip_a, slip_v_wheel_pk, t_up_s * 1000.0f, t_dn_s * 1000.0f);
    con_printf("  ACCEL: achieved %.0f mm/s2 (%.0f %% of asked); wheel +%.0f, "
               "body +%.0f -> slip %+.1f %%; duty peak %.2f of %.2f%s%s\n",
               a_ach, slip_a > 0 ? a_ach / slip_a * 100.0f : 0.0f, dv_w_acc,
               dv_b_acc, sl_acc, slip_duty_acc, lim,
               motor_acc ? "  << MOTOR LIMIT" : "",
               ctrl_acc ? "  << CONTROLLER LIMIT (wheels could not follow the "
                          "ramp with duty to spare: kp/ka)" : "");
    float body_dec = t_body_s > 0.01f ? slip_v_imu_bk / t_body_s : 0.0f;
    con_printf("  BRAKE: profile %.0f mm/s2 over %.0f ms; body stopped in %s "
               "(%.0f mm/s2); wheel -%.0f, body -%.0f -> slip %+.1f %%; duty "
               "peak %.2f%s%s\n",
               b_ach, t_dn_s * 1000.0f,
               slip_tbody0 ? (String(t_body_s * 1000.0f, 0) + " ms").c_str()
                           : "NEVER (still moving when the move ended)",
               body_dec, dv_w_brk, dv_b_brk, sl_brk, duty_brk,
               motor_brk ? "  << MOTOR LIMIT" : "",
               ctrl_brk ? "  << CONTROLLER LIMIT" : "");
    if (slip_wheel_min < -50.0f)
        con_printf("  LOCK: the wheels were driven to %.0f mm/s while the body "
                   "was still doing %.0f -- a locked-wheel slide, the odometry "
                   "counting nothing%s\n", slip_wheel_min, slip_body_at_lock,
                   slip_body_at_lock > 500.0f
                       ? "  << BRAKE TYRE LIMIT (or the wall)" : " (at the very end: harmless)");
    if (slip_timed_out)
        con_println("  MOVE DID NOT FINISH -- aborted after 3 s of braking: "
                    "the wall took the rest of the distance");
    con_printf("  ABS: %s -- %lu episode(s), %lu ticks (%.0f ms) with a wheel "
               "target clamped to the band (flag bit5 in the log)\n",
               control_get_gains().abs_band > 0.0f
                   ? (String("band ") + String(control_get_gains().abs_band * 100.0f, 0) + " %").c_str()
                   : "OFF (absb=0)",
               (unsigned long)m.abs_events, (unsigned long)m.abs_ticks,
               (double)m.abs_ticks * 2.0);
    con_printf("  RULER: believed %.1f mm of %.0f commanded; front wall %s "
               "before, %s after%s; slip gate tripped %lu time(s)\n",
               believed, slip_mm,
               slip_front0 >= 0 ? String(slip_front0, 0).c_str() : "out of range",
               front1 >= 0 ? String(front1, 0).c_str() : "out of range",
               (slip_front0 >= 0 && front1 >= 0)
                   ? (String(" -> world ") + String(slip_front0 - front1, 0) +
                      " mm, wheels over-read by " +
                      String(believed - (slip_front0 - front1), 1)).c_str()
                   : "",
               (unsigned long)m.slip_events);
    con_println("  (+slip = the wheels gained/lost more speed than the body, "
                "bias-corrected; the ruler is the final word)");
    flog_set_outcome("slip test done");
    flog_stop();
    sensors_stop();
    con_printf("  flight log: %lu records at the packet rate -- 'y' for the "
               "time series (wheel v vs v_imu vs v_ref, duty in 'slide')\n",
               (unsigned long)flog_count());
    control_velocity_enable(false);
    slip_state = -1;
}

// ---- 'lat' LATERAL LIMIT TEST (2026-09-25 evening, for the big floor).
// The slip ladder found the tyre's LONGITUDINAL knee (~7000 mm/s^2). The
// corners run on ralat, a number chosen (4000) and never measured, and
// the D9 laps say the corner speed is where the time is. This is the
// lateral ladder in one run: a circle of radius R whose angular rate
// ramps at alpha to the rate that gives lateral A -- so the lateral load
// v^2/R climbs from 0 to A through the sweep -- while the log records
// what the mouse actually did: the IMU's lateral (ay) against the
// commanded v^2/R, and the gyro's rate against the commanded omega. Where
// the tyre holds, ay tracks v^2/R and omega tracks the command; past the
// knee the mouse runs wide -- ay flattens while the command keeps
// climbing, the measured omega falls under the commanded (the real radius
// grows), yaw_err opens. The report bins the sweep by commanded lateral
// and names the first bin that fell away. Needs a clear circle of ~2R +
// the body: no walls, so the big floor. Direction: +deg = left (CCW).
//
// Flight log columns (banner "lat:"): s = wheel distance, v = COMMANDED
// v (omega_ref x R), yaw, yaw_err, ay = measured lateral, ax = measured
// forward, omega = MEASURED yaw rate, obs_along = commanded lateral
// v^2/R, obs_cross = commanded omega (deg/s), wf = wheel - fused, flags
// bit0 arc, bit5 ABS, bit7 slip gate.
static int lat_state = -1;
static float lat_a = 6000.0f, lat_r = 84.0f, lat_deg = 720.0f, lat_omega = 0.0f, lat_alpha = 0.0f;
static uint32_t lat_t0 = 0, lat_log_seq = 0;
static float lat_d0 = 0.0f;
static constexpr int LAT_BINS = 24;            // 500 mm/s^2 each, to 12000
static constexpr float LAT_BIN = 500.0f;
static float lat_ay_sum[LAT_BINS], lat_om_sum[LAT_BINS], lat_omc_sum[LAT_BINS], lat_ye_sum[LAT_BINS];
static int lat_n[LAT_BINS];

static void lat_log_packet(uint32_t now) {
    WallRanges w = sensors_latest();
    if (w.seq == lat_log_seq) return;
    lat_log_seq = w.seq;
    ControlMotion mo = control_motion();
    ControlOdometry od = control_odometry();
    ControlHeading hd = control_heading();
    ControlMoveStatus ms = control_move_status();
    float v_cmd = ms.omega_ref_dps * DEG2RAD * lat_r;       // commanded v along the arc
    float a_lat_cmd = v_cmd * v_cmd / lat_r;                  // commanded lateral
    FlogRec r;
    memset(&r, 0, sizeof r);
    r.t_ms = (uint16_t)(now - lat_t0);
    r.s_mm = (int16_t)(od.dist_mm - lat_d0);
    r.v_mm_s = (int16_t)v_cmd;
    r.yaw_cdeg = (int16_t)(hd.yaw_deg * 10.0f);
    r.yaw_err_cdeg = (int16_t)(ms.yaw_err_deg * 10.0f);
    r.ay_mm_s2 = (int16_t)mo.a_lat_mm_s2;
    r.ax_mm_s2 = (int16_t)mo.a_fwd_mm_s2;
    r.flags = (uint8_t)((ms.active ? 1 : 0) | (mo.abs_active ? 0x20 : 0) | (mo.slip ? 0x80 : 0));
    r.left_mm = w.left_mm; r.right_mm = w.right_mm;
    r.front_left_mm = w.front_left_mm; r.front_right_mm = w.front_right_mm;
    r.wheel_fused_mm = (int16_t)(od.dist_mm - lat_d0 - mo.d_est_mm);
    r.omega_dps = (int16_t)ms.yaw_rate_dps;
    r.obs_along_mm = (int16_t)a_lat_cmd;
    r.obs_cross_mm = (int16_t)ms.omega_ref_dps;
    flog_record(r);
    // The bins: only while the command is above a floor (the ramp's first
    // instants are noise) and the arc is still being commanded.
    if (ms.active && a_lat_cmd > 200.0f) {
        int b = (int)(a_lat_cmd / LAT_BIN);
        if (b >= 0 && b < LAT_BINS) {
            lat_ay_sum[b] += fabsf(mo.a_lat_mm_s2);
            lat_om_sum[b] += fabsf(ms.yaw_rate_dps);
            lat_omc_sum[b] += fabsf(ms.omega_ref_dps);
            lat_ye_sum[b] += fabsf(ms.yaw_err_deg);
            lat_n[b]++;
        }
    }
}
static void lat_start(const char *args) {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    lat_a = 6000.0f; lat_r = 84.0f; lat_deg = 720.0f;
    if (args && args[0]) {
        lat_a = atof(args);
        const char *c = strchr(args, ',');
        if (c) { lat_r = atof(c + 1); const char *d = strchr(c + 1, ','); if (d) lat_deg = atof(d + 1); }
    }
    if (lat_a < 500.0f) lat_a = 500.0f;
    if (lat_a > 15000.0f) lat_a = 15000.0f;
    if (lat_r < 40.0f) lat_r = 40.0f;
    if (lat_r > 300.0f) lat_r = 300.0f;
    if (fabsf(lat_deg) < 180.0f) lat_deg = lat_deg < 0 ? -180.0f : 180.0f;
    if (fabsf(lat_deg) > 1800.0f) lat_deg = lat_deg < 0 ? -1800.0f : 1800.0f;
    // v at the target lateral, the rate that gives it, and a ramp that
    // spends ~35 % of the sweep climbing (angle under a rate ramp =
    // omega^2 / 2 alpha), holds A for ~30 %, and ramps down over the
    // last 35 % -- the executor's profile brakes the rate to zero at the
    // end. (The first floor session had 70 %: the profile was a
    // triangle that peaked at mid-sweep and the top bin got 40 samples.)
    float v = sqrtf(lat_a * lat_r);
    lat_omega = v / lat_r / DEG2RAD;
    lat_alpha = lat_omega * lat_omega / (2.0f * 0.35f * fabsf(lat_deg));
    if (lat_alpha < 100.0f) lat_alpha = 100.0f;
    con_printf("lateral test: R%.0f circle, %+.0f deg, rate ramping at %.0f deg/s2 "
               "to %.0f deg/s (v %.0f mm/s, lateral %.0f mm/s2). Needs ~%.0f mm of "
               "clear floor around the start. CLEAR OF WALLS.\n",
               lat_r, lat_deg, lat_alpha, lat_omega, v, lat_a, 2.0f * lat_r + 120.0f);
    for (int i = 0; i < LAT_BINS; i++) { lat_ay_sum[i] = lat_om_sum[i] = lat_omc_sum[i] = lat_ye_sum[i] = 0.0f; lat_n[i] = 0; }
    sensors_free_run_interlaced();
    control_velocity_enable(true);
    control_motion_zero();
    control_duty_peak_reset();
    lat_d0 = control_odometry().dist_mm;
    lat_t0 = millis();
    lat_log_seq = sensors_latest().seq;
    flog_arm();
    {
        char b[160];
        snprintf(b, sizeof b, "lat:circle A%.0f R%.0f deg%.0f om%.0f al%.0f cols:s=wheel "
                 "v=v_cmd ay=lat_meas ax=fwd_meas omega=omega_meas along=lat_cmd "
                 "cross=omega_cmd wf=wheel-fused", (double)lat_a, (double)lat_r,
                 (double)lat_deg, (double)lat_omega, (double)lat_alpha);
        flog_set_banner(b);
    }
    if (!control_arc_start(lat_r, lat_deg, lat_omega, lat_alpha)) {
        flog_stop();
        control_velocity_enable(false);
        sensors_stop();
        con_println("arc refused -- fusion not ready?");
        return;
    }
    lat_state = 0;
}
static void lat_report() {
    con_printf("lateral test done: R%.0f, %.0f deg commanded, duty peak %.2f of %.2f\n",
               lat_r, fabsf(lat_deg), control_velocity_status().duty_peak,
               control_get_gains().duty_limit);
    con_println("  lat_cmd   n   ay_meas  ay/cmd  om_meas/om_cmd  |yaw_err|");
    int knee = -1;
    for (int b = 0; b < LAT_BINS; b++) {
        if (lat_n[b] < 5) continue;
        float ac = (b + 0.5f) * LAT_BIN;
        float ay = lat_ay_sum[b] / lat_n[b];
        float omr = lat_omc_sum[b] > 0 ? lat_om_sum[b] / lat_omc_sum[b] : 0.0f;
        float ye = lat_ye_sum[b] / lat_n[b];
        con_printf("  %6.0f  %3d  %7.0f  %5.2f       %5.2f       %5.1f\n",
                   ac, lat_n[b], ay, ay / ac, omr, ye);
        if (knee < 0 && ac >= 1500.0f && (ay / ac < 0.85f || omr < 0.9f)) knee = b;
    }
    if (knee >= 0)
        con_printf("  KNEE: the tyre stopped following at ~%.0f mm/s2 lateral (measured ay "
                   "or yaw rate under 85-90 %% of the command) -- a race ralat is ~0.8 of "
                   "that: %.0f\n", knee * LAT_BIN, 0.8f * knee * LAT_BIN);
    else
        con_println("  no knee inside the sweep: the tyre held every bin -- go higher (A) "
                    "or the loop is the limit (duty peak)");
    ControlMotion m = control_motion();
    con_printf("  ABS acted %lu time(s); slip gate tripped %lu time(s)\n",
               (unsigned long)m.abs_events, (unsigned long)m.slip_events);
    flog_set_outcome("lateral test done");
    flog_stop();
    sensors_stop();
    con_printf("  flight log: %lu records -- 'y' for the time series (ay vs obs_along, "
               "omega vs obs_cross)\n", (unsigned long)flog_count());
    control_velocity_enable(false);
    lat_state = -1;
}
static void lat_poll() {
    if (lat_state < 0) return;
    sensors_poll();
    lat_log_packet(millis());
    ControlMoveStatus ms = control_move_status();
    if (lat_state == 0) {
        if (ms.active) {
            // A circle that stops turning is a collision or a spin-out:
            // the yaw error past 45 deg says the mouse is no longer on it.
            if (fabsf(ms.yaw_err_deg) > 45.0f) {
                control_move_abort();
                con_println("  lateral test ABORTED -- yaw error past 45 deg (spun out or hit something)");
                lat_state = 1;
            }
            return;
        }
        lat_state = 1;
    }
    if (lat_state == 1) lat_report();
}

static void btest_measure_reset() {
    btest_front_sum = 0.0f;
    btest_front_n = 0;
    btest_last_seq = sensors_latest().seq;
}

static void btest_start() {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    con_println("front-wall stop: ON FLOOR facing a wall <=220 mm ahead");
    sensors_free_run();
    btest_measure_reset();
    btest_state = 0;
    btest_t0 = millis();
}

static void btest_poll() {
    if (btest_state < 0) return;
    uint32_t now = millis();
    sensors_poll();  // drain the sub-MCU FIFO every loop iteration

    if (btest_state == 0 || btest_state == 3) {  // stationary averaging
        WallRanges w = sensors_latest();
        if (w.seq != btest_last_seq) {
            btest_last_seq = w.seq;
            WallView wv = walls_evaluate(w, sensors_ms_since_packet());
            if (wv.front) {
                btest_front_sum += wv.front_mm;
                btest_front_n++;
            }
        }
        if (btest_front_n >= BTEST_AVG_PACKETS) {
            float front = btest_front_sum / btest_front_n;
            if (btest_state == 0) {
                btest_front0 = front;
                btest_move_mm = front - BTEST_TARGET_MM;
                if (btest_move_mm < BTEST_MIN_MOVE_MM ||
                    btest_move_mm > BTEST_MAX_MOVE_MM) {
                    con_printf("front %.1f mm -> move %.1f mm out of range "
                               "(%.0f..%.0f) -- aborted\n", front,
                                  btest_move_mm, BTEST_MIN_MOVE_MM,
                                  BTEST_MAX_MOVE_MM);
                    sensors_stop();
                    btest_state = -1;
                    return;
                }
                con_printf("front %.1f mm -> straight %.1f mm (target %.1f)\n",
                              btest_front0, btest_move_mm, BTEST_TARGET_MM);
                con_println("d,front,v,v_enc,dutyL,dutyR,slip");
                control_velocity_enable(true);
                control_motion_zero();
                if (!control_move_start(btest_move_mm, BTEST_V, BTEST_ACC)) {
                    control_velocity_enable(false);
                    sensors_stop();
                    btest_state = -1;
                    con_println("move start refused -- fusion not ready?");
                    return;
                }
                btest_state = 1;
                btest_last_print = 0;
            } else {  // state 3: final stationary average -> report
                float err = front - BTEST_TARGET_MM;
                float sensor_travel = btest_front0 - front;
                con_printf("-- front-stop summary --\n");
                con_printf("stop error %+.1f mm (front %.1f vs target %.1f)\n",
                              err, front, BTEST_TARGET_MM);
                con_printf("travel: odometry %.1f mm vs front-sensor %.1f mm "
                           "(diff %+.1f -> front geometry, not dynamics)\n",
                              btest_end_d, sensor_travel,
                              btest_end_d - sensor_travel);
                sensors_stop();
                btest_state = -1;
            }
            return;
        }
        if (now - btest_t0 > BTEST_MEASURE_TIMEOUT_MS) {
            con_printf("front wall not in view (n=%d of %d) -- aborted\n",
                          btest_front_n, BTEST_AVG_PACKETS);
            control_velocity_enable(false);
            sensors_stop();
            btest_state = -1;
        }
        return;
    }

    if (btest_state == 1) {  // moving toward the wall
        ControlMoveStatus st = control_move_status();
        if (now - btest_last_print >= 20) {
            btest_last_print = now;
            WallView wv = walls_evaluate(sensors_latest(),
                                         sensors_ms_since_packet());
            ControlOdometry o = control_odometry();
            ControlMotion m = control_motion();
            ControlVelStatus vs = control_velocity_status();
            con_printf("%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%d\n", st.d_mm,
                          wv.front ? wv.front_mm : -1.0f, st.v_cmd,
                          o.vel_mm_s, vs.duty_left, vs.duty_right,
                          m.slip ? 1 : 0);
        }
        if (!st.active) {
            btest_end_d = st.d_mm;
            if (!st.done) {
                con_println("move TIMEOUT (not settled) -- reporting anyway");
            }
            btest_state = 2;
            btest_t0 = now;
        }
        return;
    }

    // btest_state == 2: settle, then the final stationary measure.
    if (now - btest_t0 < 300) return;
    control_velocity_enable(false);
    btest_measure_reset();
    btest_state = 3;
    btest_t0 = now;
}

// -- Lateral servo test -----------------------------------------------------------
// Triggered by 'l'. Mouse ON FLOOR in a corridor with at least one side wall,
// placed DELIBERATELY off-centre (~10-20 mm), roughly along the corridor axis
// (run 'f' first so corridor = heading 0). Runs a 2-cell straight feeding the
// wall lateral offset into the executor's lateral servo
// (control_set_lateral_bias_deg) exactly as maze search legs will: fresh
// packet -> walls_evaluate -> bias = -LAT_SERVO_DEG_PER_MM * lateral, clamped.
// Expect the offset to converge toward 0 with a ~140 mm first-order tail
// (see Control.h gain intuition) and the bias to decay with it. The end
// offset is measured STATIONARY (averaged like 'b') after a settle.
//
// CSV per 20 ms while moving: d,lat,bias,yaw   (lat -999 = no side wall seen)

static constexpr float LTEST_DIST = 360.0f;  // mm: two cells
static constexpr float LTEST_V = 300.0f;     // mm/s (search speed)
static constexpr float LTEST_ACC = 500.0f;   // mm/s^2
static constexpr float LTEST_PRETURN_DEG = 1.0f;
static constexpr int   LTEST_AVG_PACKETS = 30;   // stationary end measurement
static constexpr uint32_t LTEST_MEASURE_TIMEOUT_MS = 1500;

static int ltest_state = -1;  // -1 idle, 0 pre-turn, 1 pause, 2 straight,
                              // 3 settle, 4 measure end
static uint32_t ltest_t0 = 0;
static uint32_t ltest_last_print = 0;
static uint32_t ltest_last_seq = 0;
static float ltest_start_sum = 0.0f;  // first samples -> start offset
static int ltest_start_n = 0;
static float ltest_end_sum = 0.0f;    // stationary end average
static int ltest_end_n = 0;

static void ltest_start_straight() {
    con_println("d,lat,bias,yaw");
    control_motion_zero();
    sensors_free_run();
    ltest_start_sum = 0.0f;
    ltest_start_n = 0;
    ltest_last_seq = sensors_latest().seq;
    if (!control_move_start(LTEST_DIST, LTEST_V, LTEST_ACC)) {
        control_velocity_enable(false);
        sensors_stop();
        ltest_state = -1;
        con_println("move start refused -- fusion not ready?");
        return;
    }
    ltest_state = 2;
    ltest_t0 = millis();
    ltest_last_print = 0;
}

static void ltest_start() {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    con_println("lateral servo run: ON FLOOR in corridor, OFF-CENTRE ~10-20 mm; "
                "corridor axis = heading 0. CSV:");
    control_velocity_enable(true);
    float yaw_now = control_heading().yaw_deg;
    if (fabsf(yaw_now) > LTEST_PRETURN_DEG) {
        con_printf("facing corridor: in-place turn %+.2f deg first\n", -yaw_now);
        if (!control_turn_start(-yaw_now, 180.0f, 1200.0f)) {
            control_velocity_enable(false);
            con_println("pre-turn refused -- fusion not ready?");
            return;
        }
        ltest_state = 0;
        ltest_t0 = millis();
        return;
    }
    ltest_start_straight();
}

static void ltest_poll() {
    if (ltest_state < 0) return;
    uint32_t now = millis();

    if (ltest_state == 0) {  // pre-turn running
        if (!control_move_status().active) {
            ltest_state = 1;
            ltest_t0 = now;
        }
        return;
    }
    if (ltest_state == 1) {  // pause between turn and straight
        if (now - ltest_t0 >= 400) ltest_start_straight();
        return;
    }

    sensors_poll();  // drain the sub-MCU FIFO every loop iteration

    if (ltest_state == 2) {  // straight with the servo live
        ControlMoveStatus st = control_move_status();
        WallRanges w = sensors_latest();
        WallView wv = walls_evaluate(w, sensors_ms_since_packet());

        // Feed the servo one update per fresh packet -- the maze search
        // loop will do exactly this.
        if (w.seq != ltest_last_seq) {
            ltest_last_seq = w.seq;
            if (wv.lateral_valid) {
                if (ltest_start_n < 5) {  // first samples = start offset
                    ltest_start_sum += wv.lateral_mm;
                    ltest_start_n++;
                }
                control_set_lateral_bias_deg(-LAT_SERVO_DEG_PER_MM *
                                             wv.lateral_mm);
            } else {
                control_set_lateral_bias_deg(0.0f);  // no wall: hold cardinal
            }
        }

        if (now - ltest_last_print >= 20) {
            ltest_last_print = now;
            con_printf("%.1f,%.1f,%.2f,%.2f\n", st.d_mm,
                          wv.lateral_valid ? wv.lateral_mm : -999.0f,
                          st.lat_bias_deg, control_heading().yaw_deg);
        }
        if (!st.active) {
            ltest_state = 3;
            ltest_t0 = now;
        }
        return;
    }

    if (ltest_state == 3) {  // settle before the stationary end measurement
        if (now - ltest_t0 < 300) return;
        control_velocity_enable(false);
        ltest_end_sum = 0.0f;
        ltest_end_n = 0;
        ltest_last_seq = sensors_latest().seq;
        ltest_state = 4;
        ltest_t0 = now;
        return;
    }

    // ltest_state == 4: stationary end average, then report.
    {
        WallRanges w = sensors_latest();
        if (w.seq != ltest_last_seq) {
            ltest_last_seq = w.seq;
            WallView wv = walls_evaluate(w, sensors_ms_since_packet());
            if (wv.lateral_valid) {
                ltest_end_sum += wv.lateral_mm;
                ltest_end_n++;
            }
        }
        if (ltest_end_n >= LTEST_AVG_PACKETS) {
            sensors_stop();
            ltest_state = -1;
            float start = ltest_start_n ? ltest_start_sum / ltest_start_n
                                        : -999.0f;
            con_printf("-- lateral servo summary -- start offset %+.1f mm -> "
                       "end offset %+.1f mm (n=%d)\n",
                          start, ltest_end_sum / ltest_end_n, ltest_end_n);
            return;
        }
        if (now - ltest_t0 > LTEST_MEASURE_TIMEOUT_MS) {
            sensors_stop();
            ltest_state = -1;
            con_printf("end measurement starved (n=%d of %d) -- no side wall "
                       "at the stop?\n", ltest_end_n, LTEST_AVG_PACKETS);
        }
    }
}

// ============================================================================
// 'k' -- CORNER HARNESS (V2, 2026-08-18). ONE gated corner, on the bench.
//
// This is the bring-up rig for the fast run. It flies a single
// straight -> constant-radius arc -> straight path (lib/Control/Path) at
// sprint limits through the real path executor, with the inside-wall edge
// gate (lib/Control/WallGate) live, and reports what the corner cost.
//
// It exists because a whole route measures only the SUM of its corners.
// V1 spent two weeks reading four corners' accumulated error as if it were
// one number; the harness is what finally separated them, and it is what
// found the CW/CCW asymmetry that `tasym` now corrects.
//
// PHYSICAL SETUP (L-corridor, mouse ON THE FLOOR, gyro calibrated):
//
//        start                    corner cell
//     +--------+--------+--------+
//     |  mouse ->       ->       |          two cells of corridor in,
//     +--------+--------+        |          a 90-degree corner,
//                       |        |          two cells out,
//                       |        |          and a WALL closing the far end.
//                       +--------+
//
// Place the mouse at the first cell's CENTRE, facing along the corridor.
// The path stops at the last cell's centre, so at rest the front beam
// should read the cell-centre distance (SEARCH's 84 mm). What it actually
// reads, minus 84, is the ACCUMULATED PATH-LENGTH ERROR of that one corner.
//
// WHAT IT REPORTS
//   exit yaw err   how far off the commanded 90 the mouse actually turned.
//                  Systematic and direction-dependent -> `tasym`.
//                  Systematic and direction-INDEPENDENT -> `track`.
//   exit lateral   where it sits across the outgoing corridor. Wide exits
//                  that scale with omega are effective-track scrub.
//   front error    path-length error, as above.
//   gate           what the edge gate saw, and what it corrected.
//
// USAGE
//   k        right turn, R60 (the default)
//   kl       left turn,  R60
//   kr80     right turn, R80        (any radius 45..125)
//   kl100    left turn,  R100
//
// THE SWEEP THAT MATTERS (Path.h, "when to replace this"): run both
// directions at R60/80/100, then repeat the set at a higher `kv=`. A bias
// that stays put as speed rises is trim -- fix it with tasym/track and keep
// this geometry. A bias or scatter that GROWS with speed is the curvature
// step at the tangent points, and that is the signal to make the reference
// curvature-continuous rather than to keep tuning.
// ============================================================================

static constexpr float KTEST_CELL_MM = 180.0f;
static constexpr float KTEST_CELLS_IN = 2.0f;   // corridor cells before the corner
static constexpr float KTEST_CELLS_OUT = 2.0f;  // and after it
// Deliberately below Path.h's ceiling: the harness is where a speed rung is
// TRIED, so it starts at a value the plant is known to handle and is walked
// up with 'kv='.
static float ktest_v = 350.0f;      // mm/s
static float ktest_a_lon = 700.0f;  // mm/s^2
static float ktest_a_lat = 1000.0f; // mm/s^2
static constexpr float KTEST_PRETURN_DEG = 1.0f;
static constexpr int KTEST_AVG_PACKETS = 30;    // stationary end measurement
static constexpr uint32_t KTEST_MEASURE_TIMEOUT_MS = 1500;
// Front distance at a cell centre: half a cell minus the wall inset,
// 90 - 6 = 84. Same number the search stops on; keep them equal.
static constexpr float KTEST_FRONT_TARGET_MM = 84.0f;
// WHERE THE MOUSE IS PUT DOWN, mm from the start cell's REAR boundary to the
// mouse reference point. 90 = cell centre (the compiled default); 51 = back
// against the rear wall (6 mm wall face + 45 mm of half body).
//
// This is a knob because a cell centre cannot be eyeballed and the rear wall
// can be felt. It matters more than it looks: the two placements differ by
// 39 mm, and 39 mm of unmodelled setback is larger than the gate's own +/-35
// window -- so getting it wrong does not degrade the measurement, it makes
// the gate skip every single run and look broken. Set it with 'ky='.
static float ktest_start_y = 90.0f;
// Unexpected wall dead ahead during the run. The harness has no impact
// latch (V2 dropped V1's accelerometer one along with the speed run), so
// this beam guard is the only thing between a mis-set rig and a collision.
static constexpr float KTEST_FRONT_PANIC_MM = 55.0f;

PATH_STORAGE static SpeedPath ktest_path;
static WallGate ktest_gate;
static int ktest_state = -1;  // -1 idle, 0 pre-turn, 1 pause, 2 path,
                              // 3 settle, 4 measure
static uint32_t ktest_t0 = 0;
static uint32_t ktest_last_seq = 0;
static uint32_t ktest_last_print = 0;
static float ktest_R = PATH_RADIUS_DEFAULT_MM;
static float ktest_angle = -90.0f;   // -90 = right, +90 = left
static float ktest_yaw0 = 0.0f;      // world yaw at launch
static float ktest_lat_sum = 0.0f;   // exit-straight lateral average
static int ktest_lat_n = 0;
static float ktest_front_sum = 0.0f; // stationary front average at the end
static int ktest_front_n = 0;
static float ktest_odo0 = 0.0f;      // wheel odometer at launch
static float ktest_s_end = 0.0f;     // path s when the run completed
static const char *ktest_gate_note = "no lock";
static float ktest_gate_corr = 0.0f;

// Where the edge gate expects to SEE the inside wall end.
//
// The wall the corner turns around ends at the corner cell's NEAR boundary,
// which is half a cell short of the corner point:
//     spatial distance from the start pose = CELLS_IN*CELL - CELL/2
// The beam leads the mouse by WALL_GATE_BEAM_AHEAD_MM, so the drop is seen
// that much earlier.
//
// This lands on the APPROACH STRAIGHT, where path s and spatial distance
// are the same number. That is the only reason the arithmetic can be this
// plain -- past the tangent point they diverge by ~0.2R and anything mixing
// them picks that up as a systematic error (see Path.h, the anchor trap).
static float ktest_setback_mm() { return 0.5f * KTEST_CELL_MM - ktest_start_y; }

static float ktest_gate_expect_s() {
    return KTEST_CELLS_IN * KTEST_CELL_MM - 0.5f * KTEST_CELL_MM +
           ktest_setback_mm() - WALL_GATE_BEAM_AHEAD_MM;
}

static void ktest_report_and_stop(const char *why) {
    control_move_abort();
    control_velocity_enable(false);
    sensors_stop();
    ktest_state = -1;

    float yaw_err = control_heading().yaw_deg - (ktest_yaw0 + ktest_angle);
    while (yaw_err > 180.0f) yaw_err -= 360.0f;
    while (yaw_err < -180.0f) yaw_err += 360.0f;
    float lat = ktest_lat_n ? ktest_lat_sum / ktest_lat_n : -999.0f;
    float front = ktest_front_n ? ktest_front_sum / ktest_front_n : -1.0f;
    float wheel = control_odometry().dist_mm - ktest_odo0;

    con_printf("== corner %s R%.0f v%.0f: %s ==\n",
               ktest_angle > 0.0f ? "LEFT" : "RIGHT", ktest_R, ktest_v, why);
    con_printf("  exit yaw err %+.2f deg | exit lateral %s%.1f mm\n", yaw_err,
               lat <= -900.0f ? "n/a " : "", lat <= -900.0f ? 0.0f : lat);
    if (front >= 0.0f) {
        con_printf("  front %.1f mm vs %.0f expected -> path error %+.1f mm\n",
                   front, KTEST_FRONT_TARGET_MM,
                   KTEST_FRONT_TARGET_MM - front);
    } else {
        con_println("  front: no wall seen at the stop (rig missing the end "
                    "wall? path error not measured)");
    }
    con_printf("  gate: %s (%+.1f mm) | path s %.0f of %.0f | wheels %.0f\n",
               ktest_gate_note, ktest_gate_corr, ktest_s_end,
               ktest_path.length_mm(), wheel);
    con_printf("  s - wheels %+.1f mm (gate corrections + slip)\n",
               ktest_s_end - wheel);
}

static void ktest_launch() {
    // Straights are measured BETWEEN tangent points, so each corridor run
    // gives up R to the corner (Path.h: for a 90-degree arc the tangent
    // length is exactly R).
    float lead = KTEST_CELLS_IN * KTEST_CELL_MM + ktest_setback_mm() - ktest_R;
    float exit_ = KTEST_CELLS_OUT * KTEST_CELL_MM - ktest_R;
    if (lead < PATH_CORNER_MARGIN_MM || exit_ < PATH_CORNER_MARGIN_MM) {
        con_printf("R%.0f does not fit %0.f cells of corridor\n", ktest_R,
                   KTEST_CELLS_IN);
        control_velocity_enable(false);
        ktest_state = -1;
        return;
    }
    if (!ktest_path.begin_corner(lead, ktest_R, ktest_angle, exit_)) {
        con_println("path build failed");
        control_velocity_enable(false);
        ktest_state = -1;
        return;
    }
    ktest_path.set_limits(ktest_v, ktest_a_lon, ktest_a_lat);

    // Arm the edge gate on the INSIDE wall of the turn.
    WallGateConfig gc;
    gc.expect_s_mm = ktest_gate_expect_s();
    ktest_gate.arm(gc);
    ktest_gate_note = "no lock";
    ktest_gate_corr = 0.0f;

    control_motion_zero();
    sensors_free_run_interlaced();
    ktest_last_seq = sensors_latest().seq;
    ktest_lat_sum = 0.0f;
    ktest_lat_n = 0;
    ktest_front_sum = 0.0f;
    ktest_front_n = 0;
    ktest_odo0 = control_odometry().dist_mm;
    ktest_yaw0 = control_heading().yaw_deg;
    ktest_s_end = 0.0f;

    if (!control_path_start(&ktest_path)) {
        con_println("path start refused -- fusion not ready, or a move is "
                    "still active");
        control_velocity_enable(false);
        sensors_stop();
        ktest_state = -1;
        return;
    }
    con_printf("corner %s R%.0f: %.0f mm path, arc %.0f..%.0f, gate at "
               "s %.0f. CSV:\n",
               ktest_angle > 0.0f ? "LEFT" : "RIGHT", ktest_R,
               ktest_path.length_mm(), ktest_path.corner_entry_s_mm(0),
               ktest_path.corner_exit_s_mm(0), ktest_gate_expect_s());
    con_println("s,v,omega,yaw,yaw_err,arc,lat,inside,front");
    ktest_state = 2;
    ktest_t0 = millis();
    ktest_last_print = 0;
}

static void ktest_start(const char *args) {
    if (!control_heading().ready) {
        con_println("attitude not ready -- wait for gyro calibration");
        return;
    }
    // 'k' | 'kl' | 'kr' | 'kl80' | 'kr100'
    ktest_angle = -90.0f;
    if (args[0] == 'l' || args[0] == 'L') {
        ktest_angle = 90.0f;
        args++;
    } else if (args[0] == 'r' || args[0] == 'R') {
        args++;
    }
    if (*args) {
        float r = atof(args);
        if (r >= 45.0f && r <= 125.0f) {
            ktest_R = r;
        } else {
            con_println("radius must be 45..125 mm");
            return;
        }
    }
    con_printf("corner harness: L-corridor, %.0f cells in, %s R%.0f, %.0f "
               "cells out, wall at the far end. Mouse at the first cell "
               "CENTRE facing the corridor.\n",
               KTEST_CELLS_IN, ktest_angle > 0.0f ? "LEFT" : "RIGHT",
               ktest_R, KTEST_CELLS_OUT);

    control_velocity_enable(true);
    float yaw_now = control_heading().yaw_deg;
    if (fabsf(yaw_now) > KTEST_PRETURN_DEG) {
        con_printf("squaring up: in-place turn %+.2f deg first\n", -yaw_now);
        if (!control_turn_start(-yaw_now, 300.0f, 2000.0f)) {
            control_velocity_enable(false);
            con_println("pre-turn refused");
            return;
        }
        ktest_state = 0;
        ktest_t0 = millis();
        return;
    }
    ktest_launch();
}

static void ktest_poll() {
    if (ktest_state < 0) return;
    uint32_t now = millis();
    sensors_poll();

    if (ktest_state == 0) {  // pre-turn
        ControlMoveStatus st = control_move_status();
        if (st.active) return;
        ktest_state = 1;
        ktest_t0 = now;
        return;
    }
    if (ktest_state == 1) {  // let the tyres relax
        if (now - ktest_t0 < 300) return;
        ktest_launch();
        return;
    }

    if (ktest_state == 2) {  // flying the path
        ControlPathStatus ps = control_path_status();
        WallRanges w = sensors_latest();
        if (w.seq != ktest_last_seq) {
            ktest_last_seq = w.seq;
            WallView wv = walls_evaluate(w, sensors_ms_since_packet());

            // Edge gate on the inside wall of the turn.
            if (ktest_gate.armed() && !ktest_gate.fired()) {
                uint8_t inside = (ktest_angle > 0.0f) ? w.left_mm : w.right_mm;
                float corr = 0.0f;
                WallGate::Result r =
                    ktest_gate.update(ps.s_mm, inside, wv.lateral_valid,
                                      wv.lateral_mm, &corr);
                if (r == WallGate::CORRECTION) {
                    control_path_correct(corr);
                    ktest_gate_corr = corr;
                    ktest_gate_note = "locked";
                    con_printf("edge gate: s %+.1f mm (drop at %.0f, expected "
                               "%.0f)\n",
                               corr, ktest_gate.drop_s_mm(),
                               ktest_gate_expect_s());
                } else if (r == WallGate::SKIPPED_LAT) {
                    ktest_gate_note = "skipped: off centre";
                    con_printf("edge gate: skipped, lateral %+.0f mm\n",
                               ktest_gate.last_lat_mm());
                } else if (r == WallGate::SKIPPED_WINDOW) {
                    ktest_gate_note = "skipped: out of window";
                    con_printf("edge gate: skipped, drop at %.0f vs expected "
                               "%.0f\n",
                               ktest_gate.drop_s_mm(), ktest_gate_expect_s());
                }
            }

            // Exit-straight lateral: measured only past the arc, and only
            // where a side wall is actually in view.
            if (!ps.in_arc && ps.s_mm > ktest_path.corner_exit_s_mm(0) &&
                wv.lateral_valid) {
                ktest_lat_sum += wv.lateral_mm;
                ktest_lat_n++;
            }

            if (wv.front && wv.front_mm < KTEST_FRONT_PANIC_MM) {
                ktest_report_and_stop("FRONT PANIC -- unexpected wall");
                return;
            }

            if (now - ktest_last_print >= 20) {
                ktest_last_print = now;
                uint8_t inside = (ktest_angle > 0.0f) ? w.left_mm : w.right_mm;
                con_printf("%.0f,%.0f,%.0f,%.2f,%+.2f,%d,%.1f,%u,%.1f\n",
                           ps.s_mm, ps.v_mm_s, ps.omega_dps,
                           control_heading().yaw_deg, ps.yaw_err_deg,
                           ps.in_arc ? 1 : 0,
                           wv.lateral_valid ? wv.lateral_mm : -99.9f,
                           (unsigned)inside,
                           wv.front ? wv.front_mm : -1.0f);
            }
        }

        if (ps.active) return;
        ktest_s_end = ps.s_mm;
        if (!ps.done) {
            ktest_report_and_stop("path did not finish (timeout)");
            return;
        }
        ktest_state = 3;
        ktest_t0 = now;
        return;
    }

    if (ktest_state == 3) {  // settle before the stationary measurement
        if (now - ktest_t0 < 400) return;
        ktest_front_sum = 0.0f;
        ktest_front_n = 0;
        ktest_last_seq = sensors_latest().seq;
        ktest_state = 4;
        ktest_t0 = now;
        return;
    }

    if (ktest_state == 4) {  // stationary front average -> path error
        WallRanges w = sensors_latest();
        if (w.seq != ktest_last_seq) {
            ktest_last_seq = w.seq;
            WallView wv = walls_evaluate(w, sensors_ms_since_packet());
            if (wv.valid && wv.front) {
                ktest_front_sum += wv.front_mm;
                ktest_front_n++;
            }
        }
        if (ktest_front_n >= KTEST_AVG_PACKETS ||
            now - ktest_t0 > KTEST_MEASURE_TIMEOUT_MS) {
            ktest_report_and_stop("done");
        }
        return;
    }
}

// -- 'sweep.probe' : what the corridor measures while standing still -----------
//
// The whole ring experiment is calibrated on (right - left) / 2, and run 118
// showed that number is not being taken against a fixed ruler: the two side
// beams summed to 165 mm in one place and 182 mm in another, in the same
// corridor, on the same lap. A corridor is 168 mm wide everywhere or the
// beams are wrong somewhere, and no amount of flying will say which.
//
// So: set the mouse down by hand, centred as well as the eye can manage,
// and type 'sweep.probe'. It stands still, averages both side beams over
// PROBE_N packets, and prints what it thinks. Do it mid-straight, in the
// entry window before a corner, and in the exit window after one. If a
// hand-centred mouse reads 84/84 in all three, the beams are honest and the
// mouse really was 28 mm off; if it reads 60/117 in the exit window, the
// beams lie there and the last three runs measured an artefact.
static constexpr int PROBE_N = 40;
static int probe_state = -1;
static uint32_t probe_seq = 0;
static int probe_n = 0, probe_lat_n = 0;
static float probe_lsum = 0.0f, probe_rsum = 0.0f, probe_latsum = 0.0f;
static uint8_t probe_lmin = 255, probe_lmax = 0;
static uint8_t probe_rmin = 255, probe_rmax = 0;

static void probe_start() {
    sensors_free_run_interlaced();
    probe_seq = sensors_latest().seq;
    probe_n = probe_lat_n = 0;
    probe_lsum = probe_rsum = probe_latsum = 0.0f;
    probe_lmin = probe_rmin = 255;
    probe_lmax = probe_rmax = 0;
    probe_state = 0;
    con_println("probe: hold still...");
}

static void probe_poll() {
    if (probe_state < 0) return;
    sensors_poll();
    WallRanges w = sensors_latest();
    if (w.seq == probe_seq) return;
    probe_seq = w.seq;

    // RAW beams, not the wall model: the point is to check the model, so it
    // does not get a vote on its own evidence. 0 and 255 are INVALID, not a
    // distance -- a corridor reading of 255 means "no return", which in a
    // corridor is itself the finding.
    bool lok = w.left_mm != 0 && w.left_mm != 255;
    bool rok = w.right_mm != 0 && w.right_mm != 255;
    if (lok) {
        probe_lsum += w.left_mm;
        if (w.left_mm < probe_lmin) probe_lmin = w.left_mm;
        if (w.left_mm > probe_lmax) probe_lmax = w.left_mm;
    }
    if (rok) {
        probe_rsum += w.right_mm;
        if (w.right_mm < probe_rmin) probe_rmin = w.right_mm;
        if (w.right_mm > probe_rmax) probe_rmax = w.right_mm;
    }
    WallView wv = walls_evaluate(w, sensors_ms_since_packet());
    if (wv.lateral_valid) {
        probe_latsum += wv.lateral_mm;
        probe_lat_n++;
    }
    if (++probe_n < PROBE_N) return;

    sensors_stop();
    probe_state = -1;
    float l = probe_lmax ? probe_lsum / probe_n : -1.0f;
    float r = probe_rmax ? probe_rsum / probe_n : -1.0f;
    con_printf("probe: L %.1f [%u..%u]  R %.1f [%u..%u]  sum %.1f  "
               "raw_lat %+.1f  model_lat %+.1f  yaw %+.1f  (%d packets)\n",
               (double)l, probe_lmin, probe_lmax, (double)r, probe_rmin,
               probe_rmax, (double)(l + r), (double)((r - l) * 0.5f),
               probe_lat_n ? (double)(probe_latsum / probe_lat_n) : -999.0,
               (double)control_heading().yaw_deg, probe_n);
    con_println("  a centred mouse in a 168 mm corridor reads 84 / 84 / 168.");
}

static bool any_test_running() {
    return probe_state >= 0 ||
           mtest_phase >= 0 || stest_step >= 0 || vtest_phase >= 0 ||
           atest_state >= 0 || ctest_leg >= 0 || ttest_phase >= 0 ||
           rtest_seg >= 0 || ntest_active || gtest_active || wcal_active ||
           ftest_state >= 0 ||
           btest_state >= 0 || ltest_state >= 0 || ktest_state >= 0 ||
           sweep_running() ||
           jtest_state >= 0 || slip_state >= 0 || lat_state >= 0 || utest_state >= 0 || caltest_state >= 0 ||
           sctest_state >= 0 || search_running() || explore_running() ||
           route_run_running();
}

static void motor_test_start();  // defined below

// -- Parameter UI ---------------------------------------------------------------
// Runtime gains live in Control (ControlGains); FRAM makes them survive
// power cycles. Everything here WALKS THE REGISTRY (Params.h) -- there is no
// hand-maintained name list left in this file, which is the point: the
// if-chain this replaced had already drifted (arc_understeer was tunable
// nowhere despite being a gain, and the printed list was missing it too).
//
//   p            print every param, its value and range; '*' marks values
//                that differ from the compiled default
//   name=value   set (RAM only, clamped to the registry range; says so when
//                the clamp bites)
//   w            persist the lot to FRAM
//   param.dump   machine-readable table: PARAMDEF,<name>,<value>,<min>,<max>
//                one per line, then PARAMDEF,end. This is the GUI's
//                discovery call -- the GUI learns what exists by asking, so
//                appending a registry row never changes the GUI.
//   param.diff   only the params that differ from compiled defaults -- the
//                shadowing detector. Run it before believing ANY analysis
//                built on values read out of the source code.

static void print_params() {
    ControlGains g = control_get_gains();
    const ControlGains defs{};
    char line[96];
    for (int i = 0; i < PARAM_COUNT; i++) {
        const ParamDef &d = PARAM_REGISTRY[i];
        float v = g.*(d.field);
        bool changed = v != defs.*(d.field);
        snprintf(line, sizeof line, "  %-6s %11.6g   [%g .. %g]%s\n", d.name,
                 (double)v, (double)d.min, (double)d.max,
                 changed ? "   * (differs from compiled default)" : "");
        con_write(line);
    }
    con_printf("FRAM %s (%s) | boot: %s\n",
               fram_ok ? "ok" : "MISSING", fram_status_text(),
               params_loaded ? "loaded from FRAM" : "compiled defaults");
    uint8_t lm = sensor_lut_valid_mask();
    if (lm) con_printf("sensor LUT (FRAM): channels%s%s%s%s (centre-ref)\n",
                       (lm & 1) ? " left" : "", (lm & 2) ? " front_r" : "",
                       (lm & 4) ? " front_l" : "", (lm & 8) ? " right" : "");
    else con_println("sensor LUT (FRAM): none -- sub-MCU compiled defaults");
}

static void print_param_diff() {
    ControlGains g = control_get_gains();
    const ControlGains defs{};
    int n = 0;
    for (int i = 0; i < PARAM_COUNT; i++) {
        const ParamDef &d = PARAM_REGISTRY[i];
        if (g.*(d.field) == defs.*(d.field)) continue;
        con_printf("  %-6s %11.6g   (compiled default %g)\n", d.name,
                   (double)(g.*(d.field)), (double)(defs.*(d.field)));
        n++;
    }
    if (n == 0)
        con_println("  none -- the mouse is running pure compiled defaults");
    else
        con_printf("%d param(s) shadow their compiled defaults. Any analysis "
                   "quoting a source constant for these is wrong.\n", n);
}

static void print_param_dump() {
    ControlGains g = control_get_gains();
    for (int i = 0; i < PARAM_COUNT; i++) {
        const ParamDef &d = PARAM_REGISTRY[i];
        hal_radio_wait_room(64, 20);   // 43 lines is more than the ring
        con_printf("PARAMDEF,%s,%.6g,%g,%g\n", d.name, (double)(g.*(d.field)),
                   (double)d.min, (double)d.max);
    }
    hal_radio_wait_room(64, 20);
    con_println("PARAMDEF,end");
}

static void set_param(const char *name, float val) {
    // 'circ' fans out to both wheels -- the one deliberate alias, kept
    // because the 'j' test's operator instructions use it.
    if (strcmp(name, "circ") == 0) {
        set_param("circl", val);
        set_param("circr", val);
        return;
    }
    const ParamDef *d = param_find(name);
    if (!d) {
        con_printf("unknown param '%s' -- 'p' lists them all\n", name);
        return;
    }
    ControlGains g = control_get_gains();
    g.*(d->field) = val;
    int clamped = params_clamp(g);
    control_set_gains(g);
    if (clamped)
        con_printf("%s CLAMPED to %.6g (range %g .. %g) -- set (RAM only, "
                   "'w' to persist)\n", name, (double)(g.*(d->field)),
                   (double)d->min, (double)d->max);
    else
        con_printf("%s = %.6g set (RAM only -- 'w' to persist)\n", name,
                   (double)val);
}

void bench_handle_line(char *line) {
    if (line[0] == '\0') return;

    // Any serial activity means a bench session: stand the hand start
    // down so a stray hand can't launch a mission mid-work. Exceptions:
    // 'h' (re-arm -- don't cancel yourself) and the pure observability
    // toggles 'D'/'M', which must be usable while armed (disarming on 'D'
    // stopped the lasers and cost a telemetry run, 2026-07-10).
    // Settings assignments ('race=', 'obs=', 'clr=', 'v=', 'dclr=', ...) are
    // the same class as 'D'/'M': they configure, they never launch. Disarming
    // on them stopped the lasers and cost bert a run on 2026-08-12 -- exactly
    // the failure the 'D' exception above was added for in July, just via a
    // different door. Anything containing '=' is a config write, so it keeps
    // the hand start armed.
    const bool is_assignment = (strchr(line, '=') != nullptr);
    // READS NEVER DISARM (2026-09-21). The between-runs routine of a test
    // session is: lap ends armed, 'y' fetches the log, 'lr' the report,
    // 'mr' says which rung is next, wave. Until today 'y' stood the route
    // down like any other keystroke -- and its "route disarmed" line went
    // into the captured log file where nobody reads it -- so bert saw the
    // lasers go dark, typed 'h' to get them back, and searched the rig.
    // A dump reads FRAM; it launches nothing and moves nothing. Same for
    // 'D'/'M' (the July exception), the lap-report reads and 'p'.
    const bool is_read =
        (line[1] == '\0' && (line[0] == 'y' || line[0] == 'D' ||
                              line[0] == 'M' || line[0] == 'p')) ||
        strcmp(line, "mr") == 0 || strcmp(line, "lr") == 0 ||
        strncmp(line, "lr,", 3) == 0 || strcmp(line, "lr.all") == 0 ||
        strcmp(line, "enc") == 0;
    // The sweep arms itself at power-on (LoopSweep.h). If you are typing,
    // you are stood at a terminal and not about to wave -- so stand it down,
    // exactly as the search's hand start is stood down below. 'sweep.arm'
    // re-arms on the very next line, so the only cost is a printed line.
    if (sweep_armed() && !is_assignment && !is_read) sweep_disarm();
    // Same argument for the route run, which arms itself at power-on whenever
    // FRAM holds a proven map. 'route.arm' (or 'h') re-arms on the next line.
    if (route_run_armed() && !is_assignment && !is_read) route_run_disarm();
    if (search_armed() && !is_assignment && !is_read) {
        if (!(line[0] == 'h' && line[1] == '\0')) search_disarm_handstart();
    }

    // 'thr' reports the sub-MCU threshold self-test; 'thr!' re-measures the
    // room first. Both need the sub-MCU in triggered mode -- stop any stream
    // ('x') before asking, or the reply arrives interleaved with packets.
    // ---- Loop sweep (autonomous ring parameter experiment) ---------------
    // Verbs are words, not single chars: this thing drives the mouse round a
    // maze unattended for minutes, and it should not be one keystroke away
    // from a mistyped calibration command.
    // ---- Parameter registry verbs (see the Parameter UI note above) ------
    if (strcmp(line, "enc") == 0) {
        // Wheel velocity source monitor: prints while anything runs, too
        // (it only reads), so it can watch a harness drive the wheels.
        encmon_toggle();
        return;
    }
    if (strcmp(line, "search.old") == 0) {
        // The stop-and-sense search (Search.cpp), kept for comparison.
        if (!any_test_running()) search_start();
        return;
    }
    if (strcmp(line, "route.go") == 0) {
        // route_run_go() carries its own state guard. any_test_running()
        // would be wrong here: it counts an ARMED route as "running", and
        // armed is exactly the state go launches from.
        route_run_go();
        return;
    }
    if (strcmp(line, "lr") == 0 || strncmp(line, "lr,", 3) == 0) {
        // Reprint a stored lap report (LapReport.h): the latest, or 'lr,N'
        // the one N laps back -- the ring keeps eight, so a competition's
        // MR2..MR5 all survive without a laptop between runs.
        lap_report_print(line[2] ? atoi(line + 3) : 0);
        return;
    }
    if (strcmp(line, "tick") == 0 || strncmp(line, "tick=", 5) == 0) {
        // The idle status line: off by default, it drowns a tuning
        // session's output ten times a second.
        tick_enabled = line[4] ? atoi(line + 5) != 0 : !tick_enabled;
        con_printf("tick stream %s\n", tick_enabled ? "ON (10 Hz)" : "OFF");
        return;
    }
    if (strcmp(line, "mr") == 0 || strncmp(line, "mr=", 3) == 0) {
        // THE RUN LADDER (RunLadder.h): which rung the next wave flies.
        if (line[2]) {
            run_ladder_set((uint8_t)atoi(line + 3));
        }
        ControlGains g = control_get_gains();
        uint8_t n = run_ladder_next();
        if (n)
            con_printf("run ladder: next wave is MR%u -- out peak %.0f, home %.0f "
                       "(rv2..rv5 / rvh; 'mr=0' switches the ladder off, a map "
                       "save resets it to MR2)\n", (unsigned)n,
                       (double)run_ladder_out_v(n, g), (double)run_ladder_home_v(n, g));
        else
            con_printf("run ladder: OFF -- every leg flies rv %.0f ('mr=2' "
                       "starts the ladder; a map save does too)\n",
                       (double)g.route_v);
        return;
    }
    if (strcmp(line, "lr.all") == 0) {
        // Every stored report, oldest first -- the GUI scoreboard ingests
        // a whole event's LAPREP blocks from this after the fact.
        int n = lap_report_print_all();
        con_printf("lr.all: %d report(s)\n", n);
        return;
    }
    if (strncmp(line, "lstep", 5) == 0 && (line[5] == '\0' || line[5] == ' ' || line[5] == '=')) {
        // The lane-change step test (RouteRun): 'lstep 10' arms a +10 mm
        // (left) step of the corridor set point on the next lap's first
        // long two-wall straight; 'lstep -10' to the right; 'lstep 0' off.
        float mm = line[5] ? (float)atof(line + 6) : 0.0f;   // 'lstep 10' or 'lstep=10'
        if (mm > 25.0f) mm = 25.0f;
        if (mm < -25.0f) mm = -25.0f;
        route_run_set_lstep(mm);
        if (mm == 0.0f) con_println("lstep: off");
        else con_printf("lstep: %+.0f mm step armed for the next lap(s) -- first two-wall straight\n"
                        "  with 450 mm to its arc, held 250 mm; LSTEP/LS lines in the lap report\n",
                        (double)mm);
        return;
    }
    if (strncmp(line, "wcal", 4) == 0 && (line[4] == '\0' || line[4] == ' ')) {
        const char *a = line + 4;
        while (*a == ' ') a++;
        wcal_command(a);
        return;
    }
    if (strcmp(line, "vbat") == 0) {
        // Battery sense (Vbat.h). Calibration: meter on the pack, then set
        // VBAT_DIVIDER = meter_mV / the printed pin mV.
        vbat_print();
        {
            ControlGains g = control_get_gains();
            if (g.vbat_ref > 0.0f)
                con_printf("duty compensation: vref %.2f / vbat %.2f = x%.3f (Control.h)\n",
                           (double)g.vbat_ref, (double)(vbat_mv() / 1000.0f), (double)control_vbat_comp());
            else
                con_println("duty compensation: off (vref=0) -- 'vref=<this reading>' pins the duty law to today's pack");
        }
        return;
    }
    if (strcmp(line, "goffc") == 0) {
        // The per-corner learned gate offset table (GoffTable.h).
        goff_table_print();
        return;
    }
    if (strcmp(line, "goffc.clear all") == 0) {
        goff_table_clear_all();
        return;
    }
    if (strncmp(line, "goffc.trim ", 11) == 0) {
        // goffc.trim <cx> <cy> <n|e|s|w> <mm> -- the per-corner arc trim
        // (GoffTable.h). Same sign as atrim: + = arc later = exit further
        // OUTSIDE. Survives goffc.clear and the learner.
        int cx, cy;
        char hc;
        float mm;
        if (sscanf(line + 11, "%d %d %c %f", &cx, &cy, &hc, &mm) == 4) {
            int h = hc == 'n' || hc == 'N' ? 0 : hc == 'e' || hc == 'E' ? 1
                  : hc == 's' || hc == 'S' ? 2 : hc == 'w' || hc == 'W' ? 3 : -1;
            if (h >= 0 && cx >= 0 && cy >= 0 && cx < 16 && cy < 16) {
                if (goff_table_set_trim((uint8_t)cx, (uint8_t)cy, (uint8_t)h,
                                        mm)) {
                    goff_table_save();
                    goff_table_print();
                }
                return;
            }
        }
        con_println("usage: goffc.trim <cx> <cy> <n|e|s|w> <mm>   (+ = arc "
                    "later = exit further outside; same sense as atrim)");
        return;
    }
    if (strcmp(line, "goffc.clear") == 0) {
        goff_table_clear();
        return;
    }
    if (strncmp(line, "goffc.set ", 10) == 0) {
        // goffc.set <cx> <cy> <n|e|s|w> <mm> -- hand-write one entry, for a
        // corner that cannot learn because it never drops (a NO_EDGE case
        // may finally gate once its window covers the real edge).
        int cx, cy;
        char hc;
        float mm;
        if (sscanf(line + 10, "%d %d %c %f", &cx, &cy, &hc, &mm) == 4 &&
            cx >= 0 && cx < 16 && cy >= 0 && cy < 16) {
            int h = hc == 'n' || hc == 'N'   ? 0
                    : hc == 'e' || hc == 'E' ? 1
                    : hc == 's' || hc == 'S' ? 2
                    : hc == 'w' || hc == 'W' ? 3
                                             : -1;
            if (h >= 0) {
                if (goff_table_set((uint8_t)cx, (uint8_t)cy, (uint8_t)h,
                                   mm)) {
                    goff_table_save();
                    goff_table_print();
                }
                return;
            }
        }
        con_println("usage: goffc.set <cx> <cy> <n|e|s|w> <mm>");
        return;
    }
    if (strncmp(line, "route.dir", 9) == 0) {
        // route.dir=n|e|a chooses which way round a closed-circuit rig the
        // next lap runs (RouteRun.h). Bare "route.dir" reports.
        if (line[9] == '=' && line[10] != '\0' && line[11] == '\0')
            route_run_set_dir(line[10]);
        else if (line[9] == '\0')
            con_printf("route.dir=%c\n", route_run_get_dir());
        else
            con_println("route.dir: use route.dir=n | e | a");
        return;
    }
    if (strcmp(line, "param.dump") == 0) {
        // The GUI's discovery call: if the FRAM is not live, say so HERE,
        // where the session log will show it -- the boot line scrolls by
        // before the GUI connects. Flying on compiled defaults with no
        // sensor LUTs is what "FRAM missing" means (2026-09-15).
        if (!fram_ready())
            con_printf("WARNING: FRAM NOT LIVE -- %s. Params, LUTs, map and "
                       "goff are COMPILED DEFAULTS; nothing is saved. Wrong "
                       "build for the chip? (FRAM_MBIT %d)\n",
                       fram_status_text(), (int)FRAM_MBIT);
        print_param_dump();
        return;
    }
    if (strcmp(line, "param.diff") == 0) {
        print_param_diff();
        return;
    }
    if (strcmp(line, "sweep") == 0) {
        sweep_report();
        return;
    }
    if (strcmp(line, "sweep.arm") == 0) {
        if (!any_test_running()) sweep_arm();
        return;
    }
    if (strcmp(line, "sweep.probe") == 0) {
        // The sweep auto-arms at boot, so it owns the mouse the moment you
        // plug in. Stand it down -- same as 'sweep.dump' does -- rather than
        // printing "busy" at someone holding a mouse in a corridor.
        if (sweep_running() && !sweep_armed()) {
            con_println("sweep is flying -- 'sweep.stop' first");
            return;
        }
        sweep_disarm();
        if (!any_test_running()) probe_start();
        return;
    }
    if (strcmp(line, "sweep.stop") == 0) {
        if (sweep_running()) sweep_abort("user stop");
        else sweep_disarm();
        return;
    }
    if (strcmp(line, "sweep.restart") == 0 ||
        strcmp(line, "sweep.restart!") == 0) {
        if (any_test_running()) { con_println("busy -- 'sweep.stop' first"); return; }
        // THE SWEEP AND THE FLIGHT RECORDER SHARE THE BULK REGION, and
        // sweep_restart() stamps a sweep header over it. Run after a search,
        // it silently destroys that search's flight log -- which is exactly
        // what happened on 2026-08-22: a clean first search, then
        // 'sweep.dump' (which correctly found no SWEEP log), then
        // 'sweep.restart' looking for one, and the flight log was gone.
        //
        // sweep.dump already declines politely when the region belongs to the
        // other log. Restart is the destructive direction and had no such
        // guard at all. The map is NOT at risk -- it lives at 0x610..0x708,
        // below SWEEP_BASE -- but the flight log is the whole record of a
        // search and it is not recoverable.
        if (flog_owns_region() && strcmp(line, "sweep.restart") == 0) {
            con_println("REFUSED: the bulk FRAM region holds a FLIGHT LOG "
                        "(a search), not a sweep log.");
            con_println("  'y' dumps it, 'M' shows the map. Starting a sweep "
                        "session here ERASES the flight log.");
            con_println("  'sweep.restart!' to do it anyway.");
            return;
        }
        sweep_restart();
        return;
    }
    if (strcmp(line, "sweep.dump") == 0) {
        // Deliberately NOT gated on any_test_running() beyond a run in
        // flight: the sweep is armed at power-on, so the obvious thing --
        // plug in, type sweep.dump -- has to work. The disarm above has
        // already stood the hand start down by the time we get here.
        if (sweep_running()) con_println("sweep is flying -- 'sweep.stop' first");
        else sweep_dump(con_println);
        return;
    }
    if (strcmp(line, "route.arm") == 0) {
        if (!any_test_running()) route_run_arm();
        return;
    }
    if (strcmp(line, "h.search") == 0) {
        // The search on the hand start EVEN THOUGH the map has a proven
        // route (what plain 'h' did before 2026-09-21). A completed search
        // re-saves the map, clears the goff table and resets the ladder.
        route_run_disarm();
        if (!any_test_running()) search_arm_handstart();
        return;
    }
    if (strcmp(line, "route.stop") == 0) {
        if (route_run_running() && !route_run_armed()) route_run_abort("user stop");
        else route_run_disarm();
        return;
    }
    if (strcmp(line, "route") == 0) {
        // WOULD THE STORED MAP FLY, AND AT WHAT RADIUS? Compiles the route
        // without flying anything, so the 2T question is answered on the
        // bench rather than by a mouse discovering mid-corner that two arcs
        // wanted the same 20 mm of corridor.
        //
        // A COPY of the search's maze: the flood mutates, and a console
        // query has no business changing what the mission believes.
        if (route_run_running() && !route_run_armed()) {
            con_println("route is flying -- 'route.stop' first");
            return;
        }
        route_run_report();
        Maze m = search_maze();
        m.flood_to_goal(false);   // pessimistic -- only fully-known edges
        PATH_STORAGE static SpeedPath p;   // ~36 KB: never on the stack (Path.h)
        RouteStats st;
        RouteTurn t[16];
        float v = SWEEP_DEF_V[0], slew = SWEEP_DEF_SLEW[0];
        // Same corner-geometry context the mission compiles with, so this
        // report shows the route that would actually fly.
        {
            ControlGains rg = control_get_gains();
            route_set_corner_context(rg.route_alat, rg.route_uturn_r);
        }
        RouteResult r = route_build(m, 0, 0, HEADING_N, p, v, slew, &st, t, 16);
        con_printf("route from (0,0) N at v%.0f slew %.0f: %s\n",
                   (double)v, (double)slew, route_result_name(r));
        if (r != ROUTE_OK) {
            con_println("  (search first, or the map has unknown edges on "
                        "every path to the goal)");
            return;
        }
        con_printf("  %u cells, %u turns, %u below R%u, %u U-turn(s), "
                   "%.0f mm, min straight %.0f mm\n",
                   st.cells, st.turns, st.dropped, ROUTE_RADII[0], st.uturns,
                   (double)st.length_mm, (double)st.min_straight);
        for (int j = 0; j < st.turns && j < 16; j++) {
            bool u = t[j].sweep_deg == 180 || t[j].sweep_deg == -180;
            con_printf("  turn %d (%u,%u) R%u %s%s%s\n", j, t[j].cx, t[j].cy,
                       t[j].r_mm, t[j].sweep_deg > 0 ? "LEFT " : "RIGHT",
                       u ? "  180 U-TURN" : "",
                       (!u && t[j].r_mm < ROUTE_RADII[0]) ? "  << dropped" : "");
        }
        return;
    }
    if (strcmp(line, "sweep.recover") == 0) {
        // For the case where the header says the log is empty and it is not.
        // A wipe rewrites 256 header bytes and leaves every record in place,
        // so this reads the records straight out of the summary region and
        // prints whatever is plausibly still there. Read-only.
        if (sweep_running()) { con_println("sweep is flying -- 'sweep.stop' first"); return; }
        sweep_recover(con_println);
        return;
    }
    if (strncmp(line, "sweep.dir=", 10) == 0) {
        sweep_set_direction(line[10] == 'l' || line[10] == 'L');
        con_printf("sweep direction: %s turns (progress reset)\n",
                   sweep_left_turns() ? "LEFT" : "RIGHT");
        return;
    }
    if (strncmp(line, "sweep.trace=", 12) == 0) {
        sweep_set_trace_step((float)atof(line + 12));
        con_printf("sweep trace step %.1f mm%s\n", sweep_trace_step(),
                   sweep_trace_step() <= 0.0f ? " (tracing OFF)" : "");
        sweep_report();
        return;
    }
    if (strncmp(line, "sweep.r=", 8) == 0 || strncmp(line, "sweep.v=", 8) == 0 ||
        strncmp(line, "sweep.slew=", 11) == 0) {
        const char *arg = strchr(line, '=') + 1;
        uint16_t vals[8];
        int n = 0;
        while (*arg && n < 8) {
            vals[n++] = (uint16_t)atoi(arg);
            const char *comma = strchr(arg, ',');
            if (!comma) break;
            arg = comma + 1;
        }
        bool ok;
        if (line[6] == 'r') {
            uint8_t r8[8];
            for (int i = 0; i < n; i++) r8[i] = (uint8_t)vals[i];
            ok = sweep_set_radii(r8, n);
        } else if (line[6] == 'v') {
            ok = sweep_set_speeds(vals, n);
        } else {
            ok = sweep_set_slews(vals, n);
        }
        if (!ok) con_println("rejected: check count (1..8) and ranges "
                             "(R 45..125, v 80..900, slew 500..20000)");
        else sweep_report();
        return;
    }

    if (strcmp(line, "thr") == 0 || strcmp(line, "thr!") == 0) {
        bench_print_threshold_report(line[3] == '!');
        return;
    }
    // Persist the sub-MCU's LIVE LUTs (e.g. GUI-calibrated) into main FRAM,
    // so they survive + re-upload at boot instead of being overwritten by the
    // old FRAM. Stop any sub-MCU stream (GUI) first. Then reboot is safe.
    if (strcmp(line, "lutpull") == 0) {
        sensors_stop();
        delay(30);
        int n = sensor_lut_pull_all();
        if (n > 0) con_printf("lutpull: %d channel(s) sub-MCU -> FRAM "
                              "(persisted; re-uploads at boot)\n", n);
        else con_println("lutpull: FAILED (sub-MCU silent / streaming?)");
        return;
    }

    // Revert the sensor LUTs to the sub-MCU's compiled defaults: invalidate
    // the FRAM block so the next boot uploads nothing. (The live sub-MCU
    // RAM still holds an uploaded table until a power cycle.)
    if (strcmp(line, "lutclear") == 0) {
        sensor_lut_clear();
        con_println("sensor LUT store cleared -- compiled defaults next boot");
        return;
    }

    // Goal override for practice rigs: 'goal=x,y' (single cell) or
    // 'goal=x0,y0,x1,y1' (block); 'goal=std' restores the competition
    // 2x2 block. Parsed before the generic 'name=value' gain parser.
    // Corner-harness speed rung. Runtime only: every boot returns to the
    // compiled values, so a rung that is being TRIED cannot quietly become
    // the value everything else is judged against.
    if (strncmp(line, "ky=", 3) == 0) {
        float y = atof(line + 3);
        if (y >= 20.0f && y <= 160.0f) ktest_start_y = y;
        con_printf("corner harness start y = %.0f mm from the cell's rear "
                   "boundary (%s); lead-in %+.0f mm, gate at s %.0f\n",
                   ktest_start_y,
                   ktest_start_y > 85.0f ? "cell centre" : "back to the wall",
                   ktest_setback_mm(), ktest_gate_expect_s());
        return;
    }
    if (strncmp(line, "kv=", 3) == 0) {
        float v = atof(line + 3);
        if (v >= 50.0f && v <= 900.0f) ktest_v = v;
        con_printf("corner harness v = %.0f mm/s (RAM only)\n", ktest_v);
        return;
    }
    if (strncmp(line, "kalon=", 6) == 0) {
        float a = atof(line + 6);
        if (a >= 50.0f) ktest_a_lon = a;
        con_printf("corner harness a_lon = %.0f mm/s2 (%.2f g)\n",
                   ktest_a_lon, ktest_a_lon / 9810.0f);
        return;
    }
    if (strncmp(line, "kalat=", 6) == 0) {
        float a = atof(line + 6);
        if (a >= 50.0f) ktest_a_lat = a;
        con_printf("corner harness a_lat = %.0f mm/s2 (%.2f g) -> R60 arc "
                   "%.0f mm/s\n",
                   ktest_a_lat, ktest_a_lat / 9810.0f,
                   sqrtf(ktest_a_lat * 60.0f));
        return;
    }
    // ---- Map push (the GUI's "Push rig to mouse") ------------------------
    //
    // 34 short lines stage the full 16x16 wall bitmaps, then one commit
    // adopts them as if a search had produced the map -- so a KNOWN rig
    // (CAL-6 and friends) can be loaded without driving a search first.
    //
    //   mz.v=<i>,<hex4>   vertical wall plane i (0..16): bit y = wall
    //                     between (i-1,y) and (i,y)
    //   mz.h=<i>,<hex4>   horizontal plane i: bit x = wall between
    //                     (x,i-1) and (x,i)
    //   mz.commit         all 34 rows present -> adopt (every edge KNOWN,
    //                     nothing locked, nothing visited), persist to FRAM,
    //                     report whether the goal is reachable and whether
    //                     the route mission considers the map proven
    //   mz.clear          drop a half-staged push
    //
    // Staged rather than applied line by line, so a dropped radio packet
    // yields a refused commit instead of a half-pushed maze. The commit path
    // reuses Maze::load_state -- the same door search_map_load walks -- and
    // the pushed map is marked known-everywhere but LOCKED nowhere, so a
    // later real search may still overrule a wall the rig builder got wrong.
    // ---- FRAM MIGRATION (bert, 2026-09-13: "a FRAM dump and FRAM-upload
    // ... so the new blank chip gets the stored variables"). The region is
    // the small-block window 0x000-0x7FF (Fram.h FRAM_CFG_BYTES): params,
    // goal, sensor LUTs, searched map, goff table -- everything that is
    // calibration or tuning rather than a log. 64 lines of 32 bytes.
    //   fram.info                  chip ID, size measured, build size
    //   fram.dump                  FRAM,cfg,<addr>,<64 hex> x 64, then
    //                              FRAM,end,cfg,<bytes>,<crc32>
    //   fram.clear                 drop a half-staged upload
    //   fram.stage,<addr>,<64 hex> one line into RAM (any order)
    //   fram.commit,<crc32>        all 64 present and the CRC right ->
    //                              written, read back, verified; then
    //                              POWER CYCLE so every module reloads
    // Staged, like mz.*, so a dropped radio line is a refused commit and
    // never a half-written block.
    if (strncmp(line, "fram.", 5) == 0) {
        static uint8_t stage[FRAM_CFG_BYTES];
        static uint64_t have = 0;                    // 64 lines of 32 B
        const uint32_t LINE = 32, NL = FRAM_CFG_BYTES / LINE;
        const char *a = line + 5;
        if (strcmp(a, "info") == 0) {
            con_printf("FRAM: %s | RDID %s | build FRAM_MBIT %d (%lu KB), "
                       "flight log %lu records\n",
                       fram_status_text(), fram_id_text(), (int)FRAM_MBIT,
                       (unsigned long)(FRAM_SIZE_BYTES / 1024),
                       (unsigned long)FLOG_CAPACITY);
            return;
        }
        if (strcmp(a, "dump") == 0) {
            if (!fram_ready()) { con_println("fram.dump: no FRAM"); return; }
            uint8_t buf[LINE];
            char hex[2 * LINE + 1];
            uint32_t crc = 0;
            for (uint32_t i = 0; i < NL; i++) {
                fram_read(i * LINE, buf, LINE);
                for (uint32_t k = 0; k < LINE; k++)
                    snprintf(hex + 2 * k, 3, "%02X", buf[k]);
                hal_radio_wait_room(96, 20);
                con_printf("FRAM,cfg,%04lX,%s\n", (unsigned long)(i * LINE), hex);
            }
            {
                static uint8_t all[FRAM_CFG_BYTES];
                fram_read(0, all, FRAM_CFG_BYTES);
                crc = params_crc32(all, FRAM_CFG_BYTES);
            }
            hal_radio_wait_room(64, 20);
            con_printf("FRAM,end,cfg,%lu,%08lX\n", (unsigned long)FRAM_CFG_BYTES,
                       (unsigned long)crc);
            return;
        }
        if (strcmp(a, "clear") == 0) {
            have = 0;
            con_println("fram: staging cleared");
            return;
        }
        if (strncmp(a, "stage,", 6) == 0) {
            unsigned long addr = strtoul(a + 6, nullptr, 16);
            const char *h = strchr(a + 6, ',');
            if (!h || addr % LINE != 0 || addr >= FRAM_CFG_BYTES ||
                strlen(h + 1) < 2 * LINE) {
                con_println("fram.stage: bad line");
                return;
            }
            h++;
            for (uint32_t k = 0; k < LINE; k++) {
                char b[3] = {h[2 * k], h[2 * k + 1], 0};
                stage[addr + k] = (uint8_t)strtoul(b, nullptr, 16);
            }
            have |= (1ULL << (addr / LINE));
            return;   // silent: 64 of these arrive in a burst
        }
        if (strncmp(a, "commit", 6) == 0) {
            unsigned long want = 0;
            bool has_crc = a[6] == ',';
            if (has_crc) want = strtoul(a + 7, nullptr, 16);
            if (!fram_ready()) { con_println("fram.commit: no FRAM"); return; }
            int missing = 0;
            for (uint32_t i = 0; i < NL; i++) if (!(have >> i & 1)) missing++;
            if (missing) {
                con_printf("fram.commit: REFUSED, %d of %lu lines missing -- "
                           "fram.clear and send again\n", missing, (unsigned long)NL);
                return;
            }
            uint32_t crc = params_crc32(stage, FRAM_CFG_BYTES);
            if (has_crc && crc != want) {
                con_printf("fram.commit: REFUSED, CRC %08lX != %08lX\n",
                           (unsigned long)crc, (unsigned long)want);
                return;
            }
            fram_write(0, stage, FRAM_CFG_BYTES);
            static uint8_t back[FRAM_CFG_BYTES];
            fram_read(0, back, FRAM_CFG_BYTES);
            int bad = 0;
            for (uint32_t i = 0; i < FRAM_CFG_BYTES; i++) if (back[i] != stage[i]) bad++;
            have = 0;
            if (bad) {
                con_printf("fram.commit: written but %d bytes read back wrong "
                           "-- chip fault?\n", bad);
                return;
            }
            con_printf("fram.commit: %lu bytes written and verified, CRC %08lX. "
                       "POWER CYCLE NOW: params, goal, sensor LUTs, map and the "
                       "goff table all reload at boot.\n",
                       (unsigned long)FRAM_CFG_BYTES, (unsigned long)crc);
            return;
        }
        con_println("usage: fram.info | fram.dump | fram.clear | "
                    "fram.stage,<addr>,<hex> | fram.commit,<crc32>");
        return;
    }
    if (strncmp(line, "mz.", 3) == 0) {
        static uint16_t mzv[MAZE_SIZE + 1], mzh[MAZE_SIZE + 1];
        static uint32_t mz_have_v = 0, mz_have_h = 0;  // row bitmasks
        const uint32_t all = (1UL << (MAZE_SIZE + 1)) - 1;
        if (strcmp(line, "mz.clear") == 0) {
            mz_have_v = mz_have_h = 0;
            con_println("mz: staging cleared");
            return;
        }
        if (strcmp(line, "mz.commit") == 0) {
            if (mz_have_v != all || mz_have_h != all) {
                con_printf("mz: incomplete -- have %d/17 v, %d/17 h rows; "
                           "'mz.clear' and resend\n",
                           __builtin_popcount(mz_have_v),
                           __builtin_popcount(mz_have_h));
                return;
            }
            Maze::State st;
            for (int i = 0; i <= MAZE_SIZE; i++) {
                st.vwall[i] = mzv[i];
                st.hwall[i] = mzh[i];
                st.vknown[i] = 0xFFFF;
                st.hknown[i] = 0xFFFF;
                st.vlock[i] = 0;
                st.hlock[i] = 0;
            }
            for (int i = 0; i < MAZE_SIZE; i++) st.visited[i] = 0;
            Maze m = search_maze();   // keeps the current goal
            m.load_state(st);
            search_maze_adopt(m);     // replace + persist
            mz_have_v = mz_have_h = 0;
            Maze q = search_maze();
            q.flood_to_goal(false);   // pessimistic -- the route's own test
            uint16_t d = q.distance(0, 0);
            if (d == MAZE_DIST_UNREACHABLE)
                con_println("mz: ADOPTED and saved, but the goal is NOT "
                            "reachable from (0,0) on this map -- check the "
                            "walls or the goal ('goal=x,y')");
            else
                con_printf("mz: adopted and saved -- goal %u cells from "
                           "start, route %savailable\n", d,
                           route_run_available() ? "" : "NOT ");
            return;
        }
        unsigned ix, val;
        if (sscanf(line, "mz.v=%u,%x", &ix, &val) == 2 && ix <= MAZE_SIZE) {
            mzv[ix] = (uint16_t)val;
            mz_have_v |= 1UL << ix;
            return;   // silent ack: 34 rows would drown the log
        }
        if (sscanf(line, "mz.h=%u,%x", &ix, &val) == 2 && ix <= MAZE_SIZE) {
            mzh[ix] = (uint16_t)val;
            mz_have_h |= 1UL << ix;
            return;
        }
        con_println("usage: mz.v=<i>,<hex4> | mz.h=<i>,<hex4> | mz.commit | "
                    "mz.clear");
        return;
    }

    if (strncmp(line, "goal=", 5) == 0) {
        const char *arg = line + 5;
        int x0, y0, x1, y1;
        if (strcmp(arg, "std") == 0) {
            search_set_goal(MAZE_GOAL_MIN, MAZE_GOAL_MIN, MAZE_GOAL_MAX,
                            MAZE_GOAL_MAX);
        } else if (sscanf(arg, "%d,%d,%d,%d", &x0, &y0, &x1, &y1) == 4) {
            search_set_goal(x0, y0, x1, y1);
        } else if (sscanf(arg, "%d,%d", &x0, &y0) == 2) {
            search_set_goal(x0, y0, x0, y0);
        } else {
            con_println("usage: goal=x,y | goal=x0,y0,x1,y1 | goal=std");
        }
        return;
    }

    // Searched map: 'map=save' persists it now, 'map=wipe' forgets it (the
    // console equivalent of the long hand-start gesture), 'map=load' pulls
    // the stored one back over the RAM model. A completed search saves by
    // itself -- 'map=save' is for the bench, when you have driven a search
    // and want the map kept without waiting for the mission to end.
    if (strncmp(line, "map=", 4) == 0) {
        const char *arg = line + 4;
        if (strcmp(arg, "save") == 0) {
            search_map_save();
        } else if (strcmp(arg, "wipe") == 0) {
            search_map_wipe();
        } else if (strcmp(arg, "load") == 0) {
            if (!search_map_load()) con_println("no valid map in FRAM");
        } else {
            con_println("usage: map=save | map=wipe | map=load");
        }
        return;
    }

    char *eq = strchr(line, '=');
    if (eq != nullptr) {
        *eq = '\0';
        set_param(line, atof(eq + 1));
        return;
    }

    // Corner harness takes suffixed forms (k / kl / kr / kl80 / kr100), so
    // it dispatches before the single-char gate below.
    if (line[0] == 'k') {
        if (!any_test_running()) ktest_start(line + 1);
        return;
    }
    // Launch slip test: 'slip 3000' (A mm/s2, 900 mm) or 'slip 3000,720'.
    if (strncmp(line, "slip", 4) == 0 && (line[4] == '\0' || line[4] == ' ' || line[4] == '=')) {
        if (any_test_running()) con_println("slip: refused -- a test or mission is running ('route.stop' / 'e')");
        else slip_start(line[4] ? line + 5 : "");
        return;
    }
    if (strncmp(line, "lat", 3) == 0 && (line[3] == '\0' || line[3] == ' ' || line[3] == '=')) {
        if (any_test_running()) con_println("lat: refused -- a test or mission is running ('route.stop' / 'e')");
        else lat_start(line[3] ? line + 4 : "");
        return;
    }
    // Scale test: 'j' (5 cells) or 'j720' (custom mm).
    if (line[0] == 'j') {
        if (!any_test_running()) jtest_start(line + 1);
        return;
    }

    // 't.track': the turn auto-tune, applying its track fit (see
    // ttest_apply_and_finish).
    if (strcmp(line, "t.track") == 0) {
        if (!any_test_running()) { ttest_apply_track = true; ttest_start(); }
        return;
    }
    if (line[1] != '\0') {
        con_printf("unknown command '%s'\n", line);
        return;
    }
    switch (line[0]) {
        case 'm': if (!any_test_running()) motor_test_start(); break;
        case 's': if (!any_test_running()) stest_start(); break;
        case 'v': if (!any_test_running()) vtest_start(); break;
        case 'a': if (!any_test_running()) atest_start(); break;
        case 'c': if (!any_test_running()) ctest_start(); break;
        case 't':
            if (!any_test_running()) { ttest_apply_track = false; ttest_start(); }
            break;
        case 'r': if (!any_test_running()) rtest_start(); break;
        case 'n':
            if (ntest_active) ntest_stop();
            else if (!any_test_running()) ntest_start();
            break;
        case 'g': if (!any_test_running()) gtest_start(); break;
        case 'f': if (!any_test_running()) ftest_start(); break;
        case 'b': if (!any_test_running()) btest_start(); break;
        case 'l': if (!any_test_running()) ltest_start(); break;
        case 'u': if (!any_test_running()) utest_start(); break;
        case 'y':
            // Flight-recorder dump (CSV; blocking -- bench only). An ARMED
            // wave is not a running mission: since reads stopped disarming
            // (2026-09-21) the route sits armed between runs, and a 'y'
            // refused on that lost four ladder logs in one session
            // (2026-09-22) -- silently, which is the worse half. The dump
            // blocks ~15 s; the armed hand start simply resumes after it.
            if (any_test_running() && !route_run_armed() && !search_armed())
                con_println("y: refused -- a mission is running; stop it first");
            else
                flog_dump(con_println);
            break;
        case 'e':
            // THE ROLLING SEARCH (Explore.h, 2026-09-12). 'e' starts it;
            // the stop-and-sense search stays reachable as 'search.old'.
            if (explore_running()) explore_abort("user abort");
            else if (search_running()) search_abort("user abort");
            else if (!any_test_running()) explore_start();
            break;
        case 'M': search_dump_map(); break;  // model map, any time
        case 'E':  // explore-all toggle: complete the map, or stop at a route
            if (search_running()) {
                con_println("explore-all: not while a mission is running");
            } else {
                search_set_explore_all(!search_explore_all());
                con_printf("explore-all %s -- 'e' will %s\n",
                           search_explore_all() ? "ON" : "OFF",
                           search_explore_all()
                               ? "map every reachable wall, then come home"
                               : "stop once a route to the goal is proven");
            }
            break;
        case 'D': search_set_debug(!search_debug()); break;  // 50 Hz stream
        case 'R':  // raw-px monitor toggle (auto-cal channel check)
            if (rawmon_active || !any_test_running()) rawmon_toggle();
            break;
        case 'C':  // front LUT sweep (auto-cal; distinct from lowercase 'c')
            if (!any_test_running() && caltest_state < 0) caltest_start();
            break;
        case 'S':  // side LUT sweep (auto-cal, 2x4 rig)
            if (!any_test_running() && sctest_state < 0) sctest_start();
            break;
        case 'W':  // commit the pending swept LUT -> FRAM + upload to sub
            if (!any_test_running()) cal_commit_pending();
            break;
        case 'h':
            // ONE MISSION OWNS THE WAVE, and 'h' arms it THE WAY BOOT DOES
            // (2026-09-21): the route when FRAM holds a proven map, the
            // search otherwise. Until today 'h' always armed the SEARCH,
            // while every disarm message said "'h' re-arms" -- so after MR3
            // bert typed mr=4, then h, waved, and the mouse searched the
            // rig again: map re-saved, goff table cleared, ladder back to
            // MR2, the learning of three runs gone. A forced re-search
            // (a moved wall) is 'h.search', or the 5 s hold on the wave.
            if (any_test_running()) break;
            if (route_run_available()) {
                search_disarm_handstart();
                route_run_arm();
            } else {
                route_run_disarm();
                search_arm_handstart();
            }
            break;
        case 'z': encoders_zero(); break;
        case 'p': print_params(); break;
        case 'w':
            if (params_save(control_get_gains())) {
                con_println("params saved to FRAM");
                params_loaded = true;   // what is in RAM is now what is in FRAM
            } else {
                con_println("save FAILED -- FRAM missing?");
            }
            break;
        default:
            con_printf("unknown command '%c'\n", line[0]);
            break;
    }
}

static void motor_test_start() {
    ControlOdometry o = control_odometry();
    mtest_l0 = o.left_mm;
    mtest_r0 = o.right_mm;
    con_println("motor self-test: LEFT forward...");
    motors_enable(true);
    motor_set_left(MTEST_DUTY);
    mtest_phase = 0;
    mtest_t0 = millis();
}


// Poll every harness. One pass per main-loop iteration; each is a no-op
// unless its own harness is running.
void bench_poll() {
    // THE FRONT GOVERNOR (Control.h) is lifted only for the two harnesses
    // whose job is to touch the wall: the front LUT sweep ('C', 45 mm at
    // contact) and the contact probe ('u'). Everything else -- the 'b' stop
    // at 84, the slip runs toward the floor's end wall -- keeps it.
    control_front_guard_enable(caltest_state < 0 && utest_state < 0);
    motor_test_poll();
    stest_poll();
    vtest_poll();
    atest_poll();
    ctest_poll();
    ttest_poll();
    rtest_poll();
    ntest_poll();
    gtest_poll();
    wcal_poll();
    ftest_poll();
    btest_poll();
    ltest_poll();
    ktest_poll();
    sweep_poll();
    jtest_poll();
    slip_poll();
    lat_poll();
    utest_poll();
    caltest_poll();
    sctest_poll();
    probe_poll();
    rawmon_poll();
    encmon_poll();
}

// A harness, a mission, or the raw-px monitor owns the console right now.
bool bench_busy() { return any_test_running() || rawmon_active || encmon_active; }
bool bench_tick_enabled() { return tick_enabled; }
