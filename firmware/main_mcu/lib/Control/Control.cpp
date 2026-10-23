#include "Control.h"
#include "DutyMix.h"

#include <math.h>

#include "HAL.h"
#include "Path.h"

// Shared ISR <-> main-loop state. Written only by the ISR; read by the main
// loop via control_imu_status() with interrupts briefly disabled.
static volatile bool state_valid = false;
static ImuSample latest_sample;
static volatile uint32_t total_samples = 0;
static volatile uint32_t tick_count = 0;
static volatile uint16_t last_batch = 0;

// 500 Hz control tick. Steady state is ~13 FIFO packets per tick (6.4 kHz
// ODR / 500 Hz drain); 32 gives headroom to catch up after a delayed tick
// without unbounded ISR time.
static constexpr size_t MAX_PACKETS_PER_TICK = 32;

// Odometry state, written only by the ISR.
static int32_t odom_left_counts = 0;
static int32_t odom_right_counts = 0;
static float odom_vel_left = 0.0f;   // mm/s, filtered
static float odom_vel_right = 0.0f;  // mm/s, filtered

// PER-WHEEL runtime odometry scale (bert, 2026-08-03): the two wheels
// differ ~0.2% in circumference, so each odometer needs its own
// count->mm scale -- believed distance drifts otherwise, and the size
// mismatch drifts the nose at equal PWM. Set from ControlGains
// circ_left/right_mm; applied at every count->mm source so mot_d_est,
// v_enc, s, and the reported odometry all inherit it. Default 1.0.
static float g_odo_scale_l = 1.0f;
static float g_odo_scale_r = 1.0f;

// Runtime gains (see ControlGains in Control.h). Replaced atomically by
// control_set_gains() with the ISR masked; read freely inside the tick.
static ControlGains vctrl_gains;

static ControlWheelVelHook wheel_vel_hook = nullptr;
void control_set_wheel_velocity_hook(ControlWheelVelHook hook) { wheel_vel_hook = hook; }
// Both sources as they stood at the last tick, for the bench 'enc' readout
// (ISR-written, main-loop read through control_wheel_sources()).
static ControlWheelSources wheel_src;

static void odometry_tick() {
    int32_t left = encoder_left_counts();
    int32_t right = encoder_right_counts();

    // Per-tick deltas -> instantaneous velocity, then low-pass (see
    // ODOM_VEL_FILTER_ALPHA in Control.h for why).
    float kbase = ENC_MM_PER_COUNT * CONTROL_LOOP_HZ;
    int32_t dl = left - odom_left_counts, dr = right - odom_right_counts;
    float vl = dl * kbase * g_odo_scale_l;
    float vr = dr * kbase * g_odo_scale_r;
    // The edge-timed source (vsrc = 1): the hook always runs so its rings
    // never fall behind, and its velocity replaces the count one when
    // selected and fresh. The same circumference scale applies -- it
    // measures nominal millimetres too.
    wheel_src.count_l = vl;
    wheel_src.count_r = vr;
    wheel_src.edge_ok = false;
    if (wheel_vel_hook) {
        float el = 0.0f, er = 0.0f; bool ok = false;
        wheel_vel_hook(dl, dr, &el, &er, &ok);
        wheel_src.edge_l = el * g_odo_scale_l;
        wheel_src.edge_r = er * g_odo_scale_r;
        wheel_src.edge_ok = ok;
        if (ok && vctrl_gains.vel_source > 0.5f) {
            vl = wheel_src.edge_l;
            vr = wheel_src.edge_r;
        }
    }
    odom_vel_left += vctrl_gains.vfilt * (vl - odom_vel_left);
    odom_vel_right += vctrl_gains.vfilt * (vr - odom_vel_right);
    wheel_src.filt_l = odom_vel_left;
    wheel_src.filt_r = odom_vel_right;
    wheel_src.edge_in_use = wheel_src.edge_ok && vctrl_gains.vel_source > 0.5f;

    odom_left_counts = left;
    odom_right_counts = right;
}

// Wheel velocity controller state. Targets/flags written by the main loop
// with the ISR masked; the rest is ISR-owned.
static volatile bool vctrl_enabled = false;
static float vctrl_target_left = 0.0f;   // mm/s
static float vctrl_target_right = 0.0f;
static float vctrl_acc_left = 0.0f;      // mm/s^2, LINEAR accel feedforward ref (x ka)
static float vctrl_acc_right = 0.0f;
static float vctrl_ang_left = 0.0f;      // mm/s^2, DIFFERENTIAL accel ref (x kaa) -- see ControlGains::kaa
static float vctrl_ang_right = 0.0f;
static float vctrl_prev_err_left = 0.0f;
static float vctrl_prev_err_right = 0.0f;
static float vctrl_duty_left = 0.0f;
static float vctrl_duty_right = 0.0f;
// The duty ceiling is now a constant (VCTRL_DUTY_LIMIT). The thermal budget
// that used to move it -- an I^2t proxy folding PEAK back to CONT -- was
// removed at bert's instruction on 2026-08-27; see Control.h. What is kept is
// the observation the budget was standing in front of: how close the clamp
// actually came to binding. ISR-written, main-loop read.
static float vctrl_duty_peak = 0.0f;
static float vctrl_cm_peak = 0.0f;

// Set by control_maneuver_start: true while the active maneuver is a PURE
// rotation (dist == 0). Selects the stiff kp_turn in the velocity loop --
// see VCTRL_KP_TURN in Control.h for why turns and straights need
// different stiffness.
static bool move_pure_turn = false;
// True once the turn planner enters its finish band (move_tick): the
// wheel loops then drop from the stiff kp_turn back to the soft kp. The
// stiff gain exists to break scrub friction DURING rotation; at zero
// command it just limit-cycles against encoder noise ("buzz"), and when
// the cycle didn't decay it held |v_enc| over the settle gate for the
// whole timeout (hardware 2026-07-10: speed-run turn at (2,5), yaw and
// rate inside tolerance, settle never passed).
static bool move_turn_finish = false;

// Feedforward + PD (see Control.h for the model; vmax/friction are the
// per-wheel sweep-fitted gains). prev_err carries the D-term state.
// a_ff is the acceleration feedforward as DUTY (ka x linear + kaa x
// differential), summed by the caller so the ABS can zero it as one.
// BATTERY COMPENSATION (ControlGains::vbat_ref): the factor the whole duty
// is multiplied by, main-loop written, tick read. 1 when off.
static volatile float vctrl_vbat_comp = 1.0f;
static volatile uint16_t vctrl_vbat_mv = 0;
void control_set_vbat_mv(uint16_t mv) {
    vctrl_vbat_mv = mv;
    float ref = vctrl_gains.vbat_ref;
    float c = 1.0f;
    if (ref > 0.0f && mv > 1000) {
        c = ref * 1000.0f / (float)mv;
        if (c < VCTRL_VBAT_COMP_MIN) c = VCTRL_VBAT_COMP_MIN;
        if (c > VCTRL_VBAT_COMP_MAX) c = VCTRL_VBAT_COMP_MAX;
    }
    vctrl_vbat_comp = c;
}
float control_vbat_comp() { return vctrl_vbat_comp; }

static float velocity_duty(float v_ref, float a_ff, float v_meas,
                           float *prev_err, float vmax, float friction,
                           float kp, float ceiling) {
    float err = v_ref - v_meas;
    float derr = (err - *prev_err) * CONTROL_LOOP_HZ;
    *prev_err = err;

    float duty = v_ref / vmax + a_ff +
                 kp * err + vctrl_gains.kd * derr;
    if (v_ref > 0.0f) duty += friction;
    else if (v_ref < 0.0f) duty -= friction;
    // Torque is V_batt x duty: scale the lot so the pack's sag is invisible
    // to everything above this line (Control.h, vbat_ref).
    duty *= vctrl_vbat_comp;

    if (duty > ceiling) duty = ceiling;
    if (duty < -ceiling) duty = -ceiling;
    return duty;
}

// move_active is declared with the move executor below; velocity_tick needs
// it for the kp selection. Ditto the path executor's arc state: arcs need
// the STIFF gain too -- the differential fights the same scrub as an
// in-place turn plus forward-rolling dynamics, and on the soft gain the
// first hardware arc developed rotation at a third of the commanded rate,
// sweeping only ~27 of 90 deg inside the arc window (2026-07-10).
static bool move_active_flag();
static bool path_arc_stiff();
static float motion_body_v();   // the fused body velocity (estimator below)

// ---- THE ABS, v2 (ControlGains::abs_band; the story is in Control.h) ----
// v1 (2026-09-24 afternoon) was a duty OVERRIDE -- hold the wheel at body
// speed when it left the band -- and it fought the P loop into a 10 ms
// limit cycle: cut, release into a huge accumulated error, 0.85 duty, spin,
// cut... the launches got SLOWER (6241 for 8000 at the motor limit). And
// its "spinning" test fired through the whole brake, because at the top of
// a motor-limited ramp the profile is still far above the body, so the
// wheel was held at free roll while v_ref fell past it and the mouse
// coasted into the wall at 1300 mm/s. Six runs, one good stop, no worse
// result than the day before it -- bert: "was it luck".
//
// v2 is a TARGET CLAMP, continuous, no bang-bang: the wheel's velocity
// target may never be more than the band above or below the body's speed
// (v_est +/- half the commanded differential). The loop then tracks a
// target the tyre can deliver -- a bounded slip ratio, which is where a
// tyre makes its most force -- and the accumulated error never builds. A
// brake at 10000 becomes a wheel held at body - 12 %, still rolling, still
// counting. While the target is clamped the acceleration feedforward is
// the BODY's measured acceleration, not the profile's (the profile is
// asking for what the ground is not giving), and the estimator stops
// fusing the encoders (they are known to be off the ground's speed) for up
// to ABS_COAST_TICKS before its hard resync.
static bool abs_on_left = false, abs_on_right = false;
static uint32_t abs_ticks = 0, abs_events = 0;
static float abs_v = 0.0f;   // the ABS's body velocity (motion_tick)
static constexpr int ABS_VMAX_TICKS = 200;   // 400 ms of wheel history
// Half the commanded wheel differential above which the loop is in an arc
// and the ABS judges the wheel mean (v7): 40 mm/s is ~65 deg/s at the
// track, well above any straight's heading correction, well below any
// route corner (R84 at 580 mm/s is 400 deg/s, half_diff 250).
static constexpr float ABS_ARC_HALFDIFF_MM_S = 40.0f;
static float abs_venc_hist[ABS_VMAX_TICKS] = {0};
static int abs_venc_idx = 0;
static constexpr float ABS_BAND_FLOOR_MM_S = 250.0f;   // a launch's normal lead is under this
static constexpr float ABS_GRIP_ACC_MM_S2 = 1500.0f;   // wheel vs body accel: "gripping"
static void motion_abs_resync();
static int abs_clamp_ticks = 0;
static bool abs_timed_out = false;
static constexpr int ABS_CLAMP_MAX_TICKS = 250;   // 500 ms: longer than any brake
// The band edge on the demand's side, for the wheel that select-low drags in.
static float abs_edge(float v_ref, float v_body_wheel, float v_wheel) {
    float b = fabsf(v_body_wheel) * vctrl_gains.abs_band;
    if (b < ABS_BAND_FLOOR_MM_S) b = ABS_BAND_FLOOR_MM_S;
    float t = v_ref < v_body_wheel ? v_body_wheel - b : v_body_wheel + b;
    if (v_ref >= v_body_wheel - b && v_ref <= v_body_wheel + b) t = v_ref;
    if (v_body_wheel > 0.5f * ABS_BAND_FLOOR_MM_S) { if (t < 0.0f) t = 0.0f; }
    else if (v_body_wheel < -0.5f * ABS_BAND_FLOOR_MM_S) { if (t > 0.0f) t = 0.0f; }
    (void)v_wheel;
    return t;
}
static float abs_target(float v_ref, float v_body_wheel, float v_wheel,
                        float *a_ff, bool *on) {
    float band = vctrl_gains.abs_band;
    // No attitude, no body reference.
    if (band <= 0.0f || !fusion_ready()) { *on = false; return v_ref; }
    float b = fabsf(v_body_wheel) * band;
    if (b < ABS_BAND_FLOOR_MM_S) b = ABS_BAND_FLOOR_MM_S;
    float hi = v_body_wheel + b, lo = v_body_wheel - b;
    // TWO WAYS IN (v4). The DEMAND outside the band (the profile asking
    // for more slip than the tyre gives), or the WHEEL itself outside it in
    // the direction of the demand -- a wheel the feedforward has already
    // driven past the tyre (the 11000 brakes: target inside the band, the
    // wheel 1000 mm/s under the body, ka x a_ref pulling it on down, and a
    // v3 that only looked at the target saw nothing). Either way the
    // target becomes the band edge on the demand's side, and the wheel is
    // held there by the stiff gain (velocity_tick).
    float dev = v_wheel - v_body_wheel;
    bool locking = dev < -b && v_ref < v_body_wheel;
    bool spinning = dev > b && v_ref > v_body_wheel;
    // Hysteresis on the way out: a clamp opened by the wheel stays until
    // the wheel is back inside half the band.
    bool hold = *on && fabsf(dev) > 0.5f * b;
    float t = v_ref;
    if (t > hi || spinning || (hold && v_ref > v_body_wheel)) t = hi;
    else if (t < lo || locking || (hold && v_ref < v_body_wheel)) t = lo;
    // THE SIGN RULE (v3, sharpened in v5): a wheel is never driven AGAINST
    // a moving body. While the body is moving forward the target is at
    // least zero whatever the demand -- at the end of an 11000 brake the
    // profile has finished, the body is still doing 700, and the move
    // executor's position term asks for reverse to trim the overshoot;
    // on the stiff clamp gain that reverse became -0.85 of duty and a
    // locked wheel at the worst moment (18:14:00, believed 973, wall 59).
    // With the body stopped the demand's own sign rules, so a body
    // estimate that drifts negative after a stop still cannot become a
    // reverse command (v2's 600 mm).
    if (v_body_wheel > 0.5f * ABS_BAND_FLOOR_MM_S) { if (t < 0.0f) t = 0.0f; }
    else if (v_body_wheel < -0.5f * ABS_BAND_FLOOR_MM_S) { if (t > 0.0f) t = 0.0f; }
    else {
        if (v_ref >= 0.0f && t < 0.0f) t = 0.0f;
        if (v_ref <= 0.0f && t > 0.0f) t = 0.0f;
    }
    bool was = *on;
    *on = (t != v_ref) || locking || spinning;
    if (*on) { *a_ff = 0.0f; abs_ticks++; if (!was) abs_events++; }
    return t;
}

static void velocity_tick() {
    if (!vctrl_enabled) return;
    // THREE stiffness regimes, each with its own gain (see kp_arc in
    // Control.h for the measured A/B that split arcs from in-place turns):
    // in-place turns get kp_turn (scrub breakaway), rolling arcs get kp_arc
    // (differential authority without duty saturation), straights get the
    // soft kp (anything stiffer buzzes).
    float kp = vctrl_gains.kp;
    if (move_active_flag() && move_pure_turn && !move_turn_finish)
        kp = vctrl_gains.kp_turn;
    else if (path_arc_stiff())
        kp = vctrl_gains.kp_arc;

    // THE ABS: clamp each wheel's target to the band around its share of
    // the body speed BEFORE the loop sees it. Not during an in-place turn:
    // there the body velocity is zero by design and the wheels are MEANT
    // to run opposite to each other.
    float tgt_l = vctrl_target_left, tgt_r = vctrl_target_right;
    // Feedforward as duty: ka on the linear part, kaa on the differential.
    const float ff_full_l = vctrl_gains.ka * vctrl_acc_left + vctrl_gains.kaa * vctrl_ang_left;
    const float ff_full_r = vctrl_gains.ka * vctrl_acc_right + vctrl_gains.kaa * vctrl_ang_right;
    float acc_l = ff_full_l, acc_r = ff_full_r;
    if (!(move_active_flag() && move_pure_turn) && !abs_timed_out) {
        float v_body = motion_body_v();
        float half_diff = 0.5f * (vctrl_target_right - vctrl_target_left);
        if (fabsf(half_diff) > ABS_ARC_HALFDIFF_MM_S) {
            // IN AN ARC THE DIFFERENTIAL IS THE TURN, NOT A SLIP (v7, the
            // big floor, 2026-09-26). The lateral test's circles at R84
            // clamped 5-9 times per run on the fast ramps and CCW -- each
            // one a yaw transient (the measured rate 20 % over the command
            // for 30 ms, the skid catching and letting go) that put ONE
            // wheel outside its share of the body speed, and each clamp
            // then held that wheel and pushed the mouse wide: false knees
            // at 4000-5000 where the slow, clamp-free runs held 5500-7000.
            // Per-wheel judgement needs the body's share per wheel, and in
            // a turn that share depends on the tyres' slip angles, which
            // nobody knows. So in an arc the ABS judges the WHEEL MEAN
            // against the body -- a longitudinal slip, the brake into the
            // corner, still shows as both wheels leaving the body together
            // -- and shifts both targets by the same amount, so the turn's
            // differential is never touched.
            float v_mean = 0.5f * (odom_vel_left + odom_vel_right);
            float t_mean = 0.5f * (tgt_l + tgt_r);
            float ff_mean = 0.5f * (acc_l + acc_r);
            bool on = abs_on_left || abs_on_right;
            float t_new = abs_target(t_mean, v_body, v_mean, &ff_mean, &on);
            abs_on_left = abs_on_right = on;
            if (on) {
                float shift = t_new - t_mean;
                tgt_l += shift; tgt_r += shift;
                acc_l = acc_r = 0.0f;
            }
        } else {
        tgt_l = abs_target(tgt_l, v_body - half_diff, odom_vel_left, &acc_l, &abs_on_left);
        tgt_r = abs_target(tgt_r, v_body + half_diff, odom_vel_right, &acc_r, &abs_on_right);
        }
        // SELECT-LOW (v6): a two-wheeled mouse braking at the limit with one
        // wheel held and the other locked yaws -- the 19:24:58 run spun a
        // full turn in 100 ms (ax -23000, yaw -15 -> +170) the moment one
        // side let go. When either wheel is clamped, both are held in
        // their bands, so the braking stays symmetric.
        if (abs_on_left != abs_on_right) {
            bool was_l = abs_on_left, was_r = abs_on_right;
            abs_on_left = abs_on_right = true;
            if (!was_l) tgt_l = abs_edge(tgt_l, v_body - half_diff, odom_vel_left);
            if (!was_r) tgt_r = abs_edge(tgt_r, v_body + half_diff, odom_vel_right);
            acc_l = acc_r = 0.0f;
        }
        // A clamp that outlasts any brake this mouse flies is not a slip
        // event, it is the estimator gone wrong (the pirouette: centripetal
        // acceleration read as forward, abs_v climbing to 6900 while the
        // wheels were driven at 3000 in a spin). Release, resync, and stay
        // released until the wheels are back in the band on their own.
        if (abs_on_left || abs_on_right) {
            if (++abs_clamp_ticks > ABS_CLAMP_MAX_TICKS) {
                abs_timed_out = true;
                abs_on_left = abs_on_right = false;
                tgt_l = vctrl_target_left; tgt_r = vctrl_target_right;
                acc_l = ff_full_l; acc_r = ff_full_r;
                motion_abs_resync();
            }
        } else {
            abs_clamp_ticks = 0;
        }
    } else {
        abs_on_left = abs_on_right = false;
        if (abs_timed_out) {
            // Stand down once the wheels agree with the body again.
            float v_body = motion_body_v();
            if (fabsf(0.5f * (odom_vel_left + odom_vel_right) - v_body) < ABS_BAND_FLOOR_MM_S)
                { abs_timed_out = false; abs_clamp_ticks = 0; }
            else motion_abs_resync();
        }
    }
    // A CLAMPED WHEEL GETS THE STIFF GAIN AND NO ACCELERATION FEEDFORWARD
    // (v4, 2026-09-24 late: the 11000 brakes). With ka x a_ref = -0.55 of
    // duty feeding forward and the soft kp needing 1800 mm/s of error to
    // cancel it, a wheel whose tyre had let go was driven BACKWARDS to
    // -1431 mm/s while the body slid on at 1000 -- the clamp had set the
    // target to body - 12 % and the loop could not hold it. Held by
    // kp_turn (0.004: 250 mm/s of error is the whole duty range) with the
    // feedforward off, the wheel sits at the target, which is where the
    // tyre makes its most force.
    float kp_l = abs_on_left ? vctrl_gains.kp_turn : kp;
    float kp_r = abs_on_right ? vctrl_gains.kp_turn : kp;
    if (abs_on_left) acc_l = 0.0f;
    if (abs_on_right) acc_r = 0.0f;
    // Each wheel's REQUEST, unclipped (2.0 is only a sanity bound), then the
    // mixer (DutyMix.h): common mode capped at dmax, each wheel at dwheel,
    // steering first. dwheel = 0 is the old per-wheel clip at dmax.
    float req_l =
        velocity_duty(tgt_l, acc_l, odom_vel_left,
                      &vctrl_prev_err_left, vctrl_gains.vmax_left_mm_s,
                      vctrl_gains.friction_left, kp_l, 2.0f);
    float req_r =
        velocity_duty(tgt_r, acc_r, odom_vel_right,
                      &vctrl_prev_err_right, vctrl_gains.vmax_right_mm_s,
                      vctrl_gains.friction_right, kp_r, 2.0f);
    DutyMixOut mix = duty_mix(req_l, req_r, vctrl_gains.duty_limit,
                              vctrl_gains.wheel_duty_limit);
    vctrl_duty_left = mix.left;
    vctrl_duty_right = mix.right;
    {
        float wl = fabsf(vctrl_duty_left), wr = fabsf(vctrl_duty_right);
        float worst = wl > wr ? wl : wr;
        if (worst > vctrl_duty_peak) vctrl_duty_peak = worst;
        if (mix.common > vctrl_cm_peak) vctrl_cm_peak = mix.common;
    }
    motor_set_left(vctrl_duty_left);
    motor_set_right(vctrl_duty_right);
}

// Fused forward-motion estimator state (see Control.h for the model).
static float mot_v_est = 0.0f;      // mm/s
static float mot_d_est = 0.0f;      // mm
static float mot_d_enc = 0.0f;      // mm, pure encoder distance integral
static float mot_d_off = 0.0f;      // mm, slip-accrued correction to d_enc
static float mot_v_imu = 0.0f;      // mm/s, pure accel integral
static float mot_a_fwd = 0.0f;      // mm/s^2, filtered
static float mot_a_lat = 0.0f;      // mm/s^2, filtered, body +Y
static float mot_wheel_acc = 0.0f;  // mm/s^2, windowed difference
static float mot_v_hist[MOTION_WHEEL_ACC_TICKS] = {0};  // v_enc ring buffer
static int mot_v_hist_idx = 0;
static bool mot_slip = false;
static int mot_slip_ctr = 0;
static int mot_recover_ctr = 0;
static int mot_slip_hold = 0;  // ticks the gate has been open (timeout)
static uint32_t mot_slip_events = 0;

// ax_avg_g / ay_avg_g: batch-averaged body X and Y accel readings (g); valid
// if have_accel. Y takes no part in the forward estimator or the slip gate --
// it is measured and filtered alongside X purely so something on this mouse
// can see a broadside slide.
static void motion_tick(float ax_avg_g, float ay_avg_g, bool have_accel) {
    float v_enc = 0.5f * (odom_vel_left + odom_vel_right);

    if (!fusion_ready()) {
        // No attitude yet -> no gravity removal. Track encoders directly so
        // the estimate is sane the moment calibration completes.
        mot_v_est = v_enc;
        abs_v = v_enc;
        mot_v_hist[mot_v_hist_idx] = v_enc;
        mot_v_hist_idx = (mot_v_hist_idx + 1) % MOTION_WHEEL_ACC_TICKS;
        return;
    }

    if (have_accel) {
        float g_b[3];
        fusion_gravity_body(g_b);
        float a_raw = (ax_avg_g - g_b[0]) * GRAVITY_MM_S2;
        mot_a_fwd += MOTION_ACC_FILTER_ALPHA * (a_raw - mot_a_fwd);
        float a_raw_y = (ay_avg_g - g_b[1]) * GRAVITY_MM_S2;
        mot_a_lat += MOTION_ACC_FILTER_ALPHA * (a_raw_y - mot_a_lat);
    }

    // Windowed wheel acceleration (see MOTION_WHEEL_ACC_TICKS): difference
    // against the velocity MOTION_WHEEL_ACC_TICKS ticks ago.
    mot_wheel_acc = (v_enc - mot_v_hist[mot_v_hist_idx]) *
                    (CONTROL_LOOP_HZ / MOTION_WHEEL_ACC_TICKS);
    mot_v_hist[mot_v_hist_idx] = v_enc;
    mot_v_hist_idx = (mot_v_hist_idx + 1) % MOTION_WHEEL_ACC_TICKS;

    // Slip gate: wheels and body must accelerate together.
    if (!mot_slip) {
        if (fabsf(mot_wheel_acc - mot_a_fwd) > MOTION_SLIP_ACC_MM_S2) {
            if (++mot_slip_ctr >= MOTION_SLIP_ON_TICKS) {
                mot_slip = true;
                mot_slip_events++;
                mot_recover_ctr = 0;
            }
        } else {
            mot_slip_ctr = 0;
        }
    } else {
        // Re-engage once the encoders agree with the coasted estimate.
        if (fabsf(v_enc - mot_v_est) < MOTION_RECOVER_MM_S) {
            if (++mot_recover_ctr >= MOTION_RECOVER_TICKS) {
                mot_slip = false;
                mot_slip_ctr = 0;
            }
        } else {
            mot_recover_ctr = 0;
        }
        // LATCH-UP TIMEOUT (2026-07-31, flight-recorder root cause of
        // WEEKS of corner-2 failures): a coast is guaranteed-divergent
        // -- the IMU velocity walks away, |v_enc - v_est| never
        // re-enters the recovery band, the gate latches open and fused
        // d inflates ~36% for the REST OF THE FLIGHT (wf column: -117
        // mm by impact, breaking exactly at corner 2's brake+turn
        // entry). The wheels are the only absolute velocity reference:
        // after 60 ticks (120 ms, bounding the coast error to ~30 mm)
        // HARD-RESYNC the estimate to the encoders and close the gate.
        // ...unless the ABS is acting on a wheel: then the encoders are
        // KNOWN to be off the ground's speed (that is why it is acting),
        // resyncing to them would tell the ABS the body had stopped, and
        // the lock would stand. The ABS itself ends the episode by
        // bringing the wheel back into the recovery band.
        if (++mot_slip_hold >= 60) {
            mot_v_est = v_enc;
            mot_slip = false;
            mot_slip_ctr = 0;
        }
    }
    if (!mot_slip) mot_slip_hold = 0;

    constexpr float dt = 1.0f / CONTROL_LOOP_HZ;
    mot_v_imu += mot_a_fwd * dt;
    mot_v_est += mot_a_fwd * dt;
    if (!mot_slip) {
        mot_v_est += MOTION_K * (v_enc - mot_v_est);
    }

    // Distance: the encoders are ground truth whenever the wheels grip, so
    // d rides the raw encoder integral -- NOT the v_est integral, whose
    // K-lag through every ramp left d ~10 mm short of the encoders at leg
    // end (hardware 2026-07-08: position settle deadlocked on the phantom
    // shortfall). During slip the encoders over/under-count; the divergence
    // between the coasted v_est and v_enc accrues into a standing offset,
    // exactly the correction a slip episode should leave behind.
    mot_d_enc += v_enc * dt;
    if (mot_slip) {
        mot_d_off += (mot_v_est - v_enc) * dt;
    }
    mot_d_est = mot_d_enc + mot_d_off;

    // THE ABS's OWN BODY VELOCITY (v3, 2026-09-24 late). The fused v_est
    // cannot serve the ABS: its slip gate needs a 2500 mm/s2 acceleration
    // mismatch to open, a brake at 10000 against a body at 8000 is under
    // that, so v_est followed the wheels down at K = 0.05/tick, the band
    // followed v_est, and the lock ratcheted in anyway (run 17:43:49, body
    // 2192 with the wheels at -61). And holding the gate open by fiat (v2)
    // let v_est coast on the IMU past the stop and go NEGATIVE, the clamp
    // chased it, and the mouse REVERSED 600 mm (17:45:44, wheels -1935 --
    // bert: "seemed to reverse at the end"). So: an integral of the body
    // acceleration that is ANCHORED to the encoders only when the wheels
    // are provably gripping -- no ramp mismatch, no clamp acting -- and
    // free-runs on the IMU through every ramp; hard-zeroed when the wheels
    // are stopped and the profile asks nothing (a stopped mouse is stopped).
    // Drift over the longest ramp this mouse flies (~300 ms) is ~45 mm/s.
    abs_v += mot_a_fwd * dt;
    const bool gripping = !abs_on_left && !abs_on_right &&
                          fabsf(mot_wheel_acc - mot_a_fwd) < ABS_GRIP_ACC_MM_S2;
    if (gripping) abs_v += MOTION_K * (v_enc - abs_v);
    if (fabsf(v_enc) < 20.0f && vctrl_target_left == 0.0f &&
        vctrl_target_right == 0.0f)
        abs_v += 0.2f * (0.0f - abs_v);
    // PHYSICS BOUND (v6): a body cannot be going faster than its wheels
    // have driven it. abs_v is held under the fastest the wheels have run
    // in the last ABS_VMAX_TICKS (plus the band floor) -- the pirouette's
    // abs_v of 6900 against wheels that had never passed 2800 was the
    // integral of centripetal acceleration read as forward, and nothing
    // was there to say so.
    abs_venc_hist[abs_venc_idx] = v_enc;
    abs_venc_idx = (abs_venc_idx + 1) % ABS_VMAX_TICKS;
    float venc_max = 0.0f, venc_min = 0.0f;
    for (int i = 0; i < ABS_VMAX_TICKS; i++) {
        if (abs_venc_hist[i] > venc_max) venc_max = abs_venc_hist[i];
        if (abs_venc_hist[i] < venc_min) venc_min = abs_venc_hist[i];
    }
    if (abs_v > venc_max + ABS_BAND_FLOOR_MM_S) abs_v = venc_max + ABS_BAND_FLOOR_MM_S;
    if (abs_v < venc_min - ABS_BAND_FLOOR_MM_S) abs_v = venc_min - ABS_BAND_FLOOR_MM_S;
}
static void motion_abs_resync() { abs_v = 0.5f * (odom_vel_left + odom_vel_right); }

static float motion_body_v() { return abs_v; }

// Move executor state (see Control.h). ISR-owned; start/abort/status mask
// the ISR around their access.
static bool move_active = false;
static bool move_done = false;
static TrapProfile move_lin_prof;   // linear profile (mm)
static TrapProfile move_ang_prof;   // angular profile (deg)
static uint32_t move_start_tick = 0;
static float move_d0 = 0.0f;        // mot_d_est at move start
static float move_yaw0 = 0.0f;      // heading captured at move start
static float move_prev_yaw = 0.0f;  // previous-tick yaw, for the rate estimate
static float move_v_ref = 0.0f, move_a_ref = 0.0f, move_d_ref = 0.0f;
static float move_v_cmd = 0.0f, move_yaw_err = 0.0f;
static float move_omega_ref = 0.0f;  // deg/s, angular profile rate
static float move_yaw_ref = 0.0f;    // deg, yaw0 + angular d_ref
static float move_yaw_rate = 0.0f;   // deg/s, measured gyro yaw rate
static float move_dv_sum = 0.0f;     // heading-hold effort, straight cruise only
static float move_vcmd_sum = 0.0f;   // centre command, straight cruise only
static uint32_t move_cruise_n = 0;

// Heading-hold INTEGRAL: the P-only hold parks at err = disturbance/kyaw,
// and the 2026-07-15 mission telemetry showed exactly that -- yaw_err
// +1..+8 deg on EVERY straight leg and arc entry, all four cardinal
// headings, i.e. a constant body-frame disturbance strim didn't fully
// absorb. This trims it out. Integrates on rolling straights only:
// frozen in arcs and pure turns (large transient errors would wind it
// up) and below crawl speed (stiction eats differentials -- integrating
// against it winds up, then kicks on breakaway). Persists across legs
// within a mission (the disturbance does too); cleared on abort/recal.
// Tuning (2026-07-16, second flight): ki 8 was fast enough to wind on the
// TRANSIENT errors of high-speed arc blends (+10 deg for ~0.2 s added
// ~12 mm/s per blend), then the zigzag offered no straights to unwind on
// and the frozen overcorrection swung yaw_err -18..+30 -- the speed run
// oscillated into a panic. The error INPUT is therefore clamped (a
// transient integrates no faster than a standing error) and ki halved:
// worst-case windup 12 mm/s per SECOND, so a 0.2 s blend adds ~2 mm/s
// (noise) while the search phase's ~10 s of straights still learns the
// full trim before the speed run needs it.
// VELOCITY-SCHEDULED (2026-07-17, third flight): the residual disturbance
// scales with speed like the strim term it hides behind -- yaw_err ran +3
// deg on 300 mm/s search straights, +5.5 at 412 on the launch straight,
// +13..+15 in the 400 mm/s arcs (where the applied trim was a FIXED dv
// learned at 300, i.e. 40% light, and the resulting heading error
// displaced the arc line ~21 mm inside -> the (2,1) south-wall rub). The
// integral therefore learns a COEFFICIENT (trim per unit v, strim's
// units): dv_hold = c * v. Learned on rolling straights as before, and
// self-scales when the arcs fly faster than the trim was learned.
static constexpr float YAW_INT_KC = 0.0133f;  // c per (deg*s); = old ki 4 @300
static constexpr float YAW_INT_ERR_CLAMP_DEG = 3.0f;
static constexpr float YAW_INT_C_MAX = 0.10f;  // 30 mm/s @300, 45 @450
static constexpr float YAW_INT_OMEGA_GATE_DPS = 20.0f;
static constexpr float YAW_INT_V_GATE_MM_S = 50.0f;
static float yaw_int_c = 0.0f;
// Master enable (V2). The integral learns a trim coefficient and applies it as
// a heading-hold differential, so with it running the mouse holds heading even
// at kyaw = 0 -- which is exactly what it is for, and exactly what makes a
// "no heading authority" contrast case impossible to write. Calibration runs
// that need to MEASURE the raw asymmetry (and test_move's contrast) turn it
// off; nothing in a mission does.
static bool yaw_int_enabled = true;
void control_yaw_integral_enable(bool enable) {
    noInterrupts();
    yaw_int_enabled = enable;
    if (!enable) yaw_int_c = 0.0f;
    interrupts();
}
bool control_yaw_integral() { return yaw_int_enabled; }
static void yaw_int_update(float yaw_err, float omega_dps, float v_cmd) {
    if (!yaw_int_enabled) return;
    if (fabsf(omega_dps) > YAW_INT_OMEGA_GATE_DPS) return;
    if (fabsf(v_cmd) < YAW_INT_V_GATE_MM_S) return;
    float e = yaw_err;
    if (e > YAW_INT_ERR_CLAMP_DEG) e = YAW_INT_ERR_CLAMP_DEG;
    if (e < -YAW_INT_ERR_CLAMP_DEG) e = -YAW_INT_ERR_CLAMP_DEG;
    yaw_int_c += vctrl_gains.kyawi * e * (1.0f / CONTROL_LOOP_HZ);
    if (yaw_int_c > vctrl_gains.kyawimax) yaw_int_c = vctrl_gains.kyawimax;
    if (yaw_int_c < -vctrl_gains.kyawimax) yaw_int_c = -vctrl_gains.kyawimax;
}
static float move_lat_bias = 0.0f;   // lateral-servo yaw-ref bias, deg (see
                                     // Control.h); straights only
static float move_reflex_bias = 0.0f; // the flinch's channel: arcs too
static float path_bias_live = 0.0f;   // the slewed sum actually applied
static float path_lat_live = 0.0f, path_rfx_live = 0.0f;
// Pure-turn closed-loop planner state (see move_tick): absolute target and
// the rate/accel limits captured from control_turn_start's arguments.
static float move_turn_target_deg = 0.0f;
static float move_turn_omega_dps = 0.0f;
static float move_turn_alpha_dps2 = 0.0f;

static bool move_active_flag() { return move_active; }

// -- Speed-path executor state (see Control.h) ---------------------------------
// ISR-owned. start / correct / status mask the ISR around their access.
static const SpeedPath *path_ptr = nullptr;
static bool path_active = false;
static bool path_done = false;
static float path_d0 = 0.0f;       // mot_d_est at launch
static float path_yaw0 = 0.0f;     // fusion yaw at launch: the frame datum
static float path_frame_adj = 0.0f; // wall-frame alignment applied (deg)
static float path_bias = 0.0f;     // gate corrections already drained into s
static volatile float path_pend = 0.0f;  // queued, not yet drained
static float path_slip = 0.0f;     // slip model since the last anchor (Control.h)
static bool path_slip_on = false;  // armed at the first anchor, see Control.h
static float path_v_prev = 0.0f;
static float path_omega_prev = 0.0f;
static float path_slew_dps2 = PATH_OMEGA_SLEW_DPS2;
// mm of correction retired per mm travelled; set per call by
// control_path_correct(ds, over_mm).
static float path_drain_rate = PATH_CORR_DRAIN_FRAC;
// Status copies.
static float path_s = 0.0f;
static float path_v_cmd = 0.0f;
static float path_omega = 0.0f;
static float path_yaw_ref = 0.0f;
static float path_yaw_err = 0.0f;
static bool path_in_arc = false;
static uint32_t path_start_tick = 0;
static uint32_t path_timeout_ticks = 0;

// -- The front governor (see Control.h) ----------------------------------------
// fg_d_mm is written by the main loop (control_front_guard_poll) and read
// in the tick: a single float store is atomic on this core, so no masking.
static volatile bool fg_enabled = true;
static volatile bool fg_wall = false;
static volatile float fg_d_mm = 0.0f;
static bool fg_capping = false;       // tick-owned
static uint32_t fg_cap_ticks = 0;     // tick-owned, zeroed at each start
// The cap on the commanded forward speed, or a huge number when no wall.
static inline float front_gov_cap() {
    if (!fg_enabled || !fg_wall) return 1.0e9f;
    float room = fg_d_mm - FRONT_GOV_FLOOR_MM;
    if (room <= 0.0f) return 0.0f;
    return sqrtf(2.0f * FRONT_GOV_DECEL_MM_S2 * room);
}
// Apply it to a forward speed command; reverse is never capped.
static inline float front_gov_apply(float v) {
    float cap = front_gov_cap();
    if (v > cap) { fg_capping = true; fg_cap_ticks++; return cap; }
    fg_capping = false;
    return v;
}

// True while the path's angular command is live -- selects the stiff wheel
// gain in velocity_tick, the same way an in-place turn does. 20 deg/s is
// well above the heading hold's ordinary effort on a straight and well
// below any real arc rate (R60 at 245 mm/s is 234 deg/s).
static bool path_arc_stiff() {
    return path_active && fabsf(path_omega_prev) > 20.0f;
}

static float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static void move_tick() {
    if (!move_active) return;

    float t = (tick_count - move_start_tick) * (1.0f / CONTROL_LOOP_HZ);
    ProfileSample pl = move_lin_prof.sample(t);  // linear, mm
    ProfileSample pa = move_ang_prof.sample(t);  // angular, deg
    float d = mot_d_est - move_d0;
    float v_enc = 0.5f * (odom_vel_left + odom_vel_right);

    // Measured yaw. The RATE is computed once per tick in the caller now --
    // recomputing it here would difference the yaw against itself and read
    // zero, because the caller has already advanced move_prev_yaw.
    float yaw = fusion_yaw_deg();

    // Position P on the fused distance (see Control.h). PURE TURNS park
    // the linear channel entirely: fused d is IMU-coasted during the
    // slip-gated spin and integrates centripetal accel into PHANTOM
    // travel (the documented -63 mm goal-180 case) -- the settle gate
    // already ignores it, but kx acting on it COMMANDED a real ~60 mm/s
    // reverse crawl after an otherwise perfect 90 (yaw_err -0.55, rate
    // -0.5, yet "did not settle": the wheels were obeying the phantom,
    // 2026-07-18).
    float v_cmd = 0.0f;
    if (!move_pure_turn) {
        float corr = vctrl_gains.kx * (pl.d_ref - d);
        if (corr > MOVE_VCORR_MAX_MM_S) corr = MOVE_VCORR_MAX_MM_S;
        if (corr < -MOVE_VCORR_MAX_MM_S) corr = -MOVE_VCORR_MAX_MM_S;
        // Coast through a SMALL overshoot instead of reversing into it
        // (the search backup-before-spot-turn, bert 2026-08-04). Only
        // suppress the reverse when the mouse is ALREADY within the
        // settle position tolerance -- so a large overshoot still gets
        // corrected and the move can always settle (the wedge risk from
        // the first version, guarded 2026-08-05; the goal-stall that day
        // was an unreachable 16x16 goal in a 6x3 maze, not this).
        if (pl.v_ref == 0.0f && corr < 0.0f &&
            fabsf(pl.d_ref - d) < MOVE_DONE_POS_MM) {
            corr = 0.0f;
        }
        v_cmd = pl.v_ref + corr;
        // THE FRONT GOVERNOR (Control.h). The profile is time-based, so a
        // capped move falls behind its d_ref, kx pushes (clamped) and the
        // move ends "not settled" 60 mm from the wall: that is the report.
        v_cmd = front_gov_apply(v_cmd);
    }

    // Angular references. Two regimes:
    //
    // PURE TURNS run a closed-loop deceleration planner (bert's technique,
    // 2026-07-10): rate command = min(omega_max, accel ramp,
    // sqrt(2*alpha*remaining)) with `remaining` MEASURED from the fusion
    // yaw each tick. The time-based TrapProfile reference marched on
    // regardless of what the scrubbing tyres delivered, dumping all the
    // error into a weak P tail that the deadband ate (3.5-4.5 deg terminal
    // error, variance between 90s and 180s, settle flapping). The sqrt
    // curve re-plans the decel every tick from where the mouse ACTUALLY
    // is, and keeps meaningful command authority until degrees from the
    // target instead of a dying linear tail.
    //
    // Everything else (straights, arcs) keeps the time-based profile.
    float ang_v_ref, ang_a_ref, yaw_ref, yaw_err;
    bool ang_done;
    if (move_pure_turn) {
        float remaining = wrap180(move_turn_target_deg - yaw);
        float w_lim = sqrtf(2.0f * move_turn_alpha_dps2 * fabsf(remaining));
        float w_mag = w_lim;
        if (w_mag > move_turn_omega_dps) w_mag = move_turn_omega_dps;
        float w_ramp =
            fabsf(move_omega_ref) + move_turn_alpha_dps2 / CONTROL_LOOP_HZ;
        if (w_mag > w_ramp) w_mag = w_ramp;  // accel-limited ramp-up
        // Finish band: the sqrt curve steepens toward infinity at zero
        // remaining -- combined with the stiff kp_turn it would chatter
        // against stiction. Inside the band, command zero and settle
        // (and drop to the soft wheel gain -- see move_turn_finish).
        move_turn_finish = fabsf(remaining) < MOVE_TURN_FINISH_DEG;
        if (move_turn_finish) w_mag = 0.0f;
        ang_v_ref = (remaining >= 0.0f) ? w_mag : -w_mag;
        ang_a_ref = (ang_v_ref - move_omega_ref) * CONTROL_LOOP_HZ;
        yaw_ref = move_turn_target_deg;
        yaw_err = remaining;
        ang_done = fabsf(remaining) < MOVE_DONE_YAW_DEG;
    } else {
        // Heading reference follows the angular profile (fixed for a
        // straight). On straights (angular profile idle) the lateral wall
        // servo adds a small clamped bias steering toward the corridor
        // centre (Control.h) -- but only while the linear profile still
        // RUNS: once it is done the mouse is stopping, a heading bias can
        // no longer produce lateral motion, and holding it just pins
        // yaw_err past MOVE_DONE_YAW_DEG so the move can never settle
        // (hardware 2026-07-10).
        ang_v_ref = pa.v_ref;
        ang_a_ref = pa.a_ref;
        yaw_ref = move_yaw0 + pa.d_ref;
        if (!pl.done && pa.d_ref == 0.0f && pa.v_ref == 0.0f) {
            yaw_ref += move_lat_bias;
        }
        yaw_err = wrap180(yaw_ref - yaw);
        ang_done = pa.done;
    }

    // Rotation-rate feedforward + closed-loop heading hold. Positive dv
    // speeds the right wheel, rotating CCW (+yaw) -- matching a positive
    // omega_ref / positive yaw_err. PURE TURNS zero the heading-hold term:
    // the planner's rate command is already a function of measured yaw
    // (position feedback lives THERE) -- stacking kyaw * remaining on top
    // would double-count position and overdrive the spin far past omega.
    float half_track = 0.5f * vctrl_gains.track_mm;
    float dv_ff = (ang_v_ref * DEG2RAD) * half_track;
    float dv_fb = move_pure_turn ? 0.0f : vctrl_gains.kyaw * yaw_err;
    if (dv_fb > vctrl_gains.dvmax) dv_fb = vctrl_gains.dvmax;
    if (dv_fb < -vctrl_gains.dvmax) dv_fb = -vctrl_gains.dvmax;
    // Integral trim (see yaw_int_update): its own gates keep it frozen
    // during rotation; pure turns exclude it entirely (position feedback
    // lives in the turn planner, and a travel-friction trim has no
    // business inside a spin).
    if (!move_pure_turn) yaw_int_update(yaw_err, ang_v_ref, v_cmd);
    float dv_hold = move_pure_turn ? 0.0f : yaw_int_c * v_cmd;

    // During a STRAIGHT cruise (no rotation commanded), measure the hold's
    // standing effort: the 'c' calibration folds avg(dv)/avg(v) into
    // gains.strim (see ControlMoveTrim). Skipped whenever a turn is active.
    // NOTE: with a P-only hold the yaw settles AT its equilibrium error, so
    // do NOT gate these samples on small yaw error (tried 2026-07-08,
    // starved the trim of samples); the plain cruise average is the right
    // measurement, and the update law strim += avg_dv/avg_v lands on the
    // equilibrium in one step -- PROVIDED the mouse is free. Against a wall
    // the contact absorbs any differential and the average is meaningless:
    // run 'c' clear of walls.
    if (ang_v_ref == 0.0f && pl.a_ref == 0.0f && pl.v_ref != 0.0f) {
        move_dv_sum += dv_fb + dv_hold;  // 'c' must see the TOTAL effort
        move_vcmd_sum += v_cmd;
        move_cruise_n++;
    }

    float dv_total = dv_ff + dv_fb + dv_hold + vctrl_gains.strim * v_cmd;

    vctrl_target_left = v_cmd - dv_total;
    vctrl_target_right = v_cmd + dv_total;
    // Linear accel FF on both wheels; angular accel FF as a differential.
    float a_ang = (ang_a_ref * DEG2RAD) * half_track;
    float a_lin_ff = (!move_pure_turn && fg_capping) ? 0.0f : pl.a_ref;  // no push into the cap
    vctrl_acc_left = a_lin_ff;
    vctrl_acc_right = a_lin_ff;
    vctrl_ang_left = -a_ang;
    vctrl_ang_right = a_ang;

    move_v_ref = pl.v_ref;
    move_a_ref = pl.a_ref;
    move_d_ref = pl.d_ref;
    move_v_cmd = v_cmd;
    move_omega_ref = ang_v_ref;
    move_yaw_ref = yaw_ref;
    move_yaw_err = yaw_err;

    if (pl.done && ang_done) {
        // Pure turns skip the linear position gate: the slip gate correctly
        // cuts the encoders out while the wheels counter-rotate, so fused d
        // coasts on the IMU -- which integrates centripetal accel into
        // PHANTOM travel (hardware 2026-07-10: the goal-cell 180 "drifted"
        // -63 mm of fused d while yaw and rate were inside tolerance, and
        // the turn could never settle). A pure rotation has no linear
        // target worth gating; wheels-stopped + yaw gates own the settle.
        bool settled = (move_pure_turn ||
                        fabsf(pl.d_ref - d) < MOVE_DONE_POS_MM) &&
                       fabsf(v_enc) < MOVE_DONE_VEL_MM_S &&
                       fabsf(yaw_err) < MOVE_DONE_YAW_DEG &&
                       fabsf(move_yaw_rate) < MOVE_DONE_YAWRATE_DPS;
        bool timeout =
            t > move_lin_prof.duration_s() + MOVE_TIMEOUT_EXTRA_S &&
            t > move_ang_prof.duration_s() + MOVE_TIMEOUT_EXTRA_S;
        if (settled || timeout) {
            move_done = settled;
            move_active = false;
            move_lat_bias = 0.0f;  // never carries across moves
            move_reflex_bias = 0.0f;
            vctrl_target_left = vctrl_target_right = 0.0f;
            vctrl_acc_left = vctrl_acc_right = 0.0f; vctrl_ang_left = vctrl_ang_right = 0.0f;
        }
    }
}

// One tick of the speed-path executor (see Control.h). Mutually exclusive
// with move_tick by their start guards, so only one of them ever commands.
static void path_tick() {
    if (!path_active) return;

    // Drain a queued gate correction into the live s. Rate-limited on
    // purpose: s feeds the braking curves, so a step here is a step in the
    // commanded speed and rate (Control.h, PATH_CORR_DRAIN_MM).
    if (path_pend != 0.0f) {
        // Retire a fraction of the distance travelled this tick, not a fixed
        // number of mm -- see PATH_CORR_DRAIN_FRAC. v_prev is last tick's
        // commanded speed, which is what the mouse has just covered.
        float cap = path_drain_rate * fabsf(path_v_prev) *
                    (1.0f / CONTROL_LOOP_HZ);
        if (cap < PATH_CORR_DRAIN_FLOOR_MM) cap = PATH_CORR_DRAIN_FLOOR_MM;
        float step = path_pend;
        if (step > cap) step = cap;
        if (step < -cap) step = -cap;
        path_bias += step;
        path_pend -= step;
    }

    float s = (mot_d_est - path_d0) + path_bias + path_slip;
    float v_prev = path_v_prev;
    SpeedPath::Sample sm =
        path_ptr->sample(s, v_prev, 1.0f / CONTROL_LOOP_HZ);
    // THE FRONT GOVERNOR (Control.h): the wall in the beams bounds the
    // speed whatever the path says. The capped v is what v_prev carries,
    // so the planner re-accelerates from it when the cap lifts, and a_lin
    // below reads the cap as braking -- the feedforward brakes too.
    float v = front_gov_apply(sm.v_mm_s);
    path_v_prev = v;

    // Angular rate reference, through the slew rail. Constant-radius
    // corners step omega at every tangent point; the rail is what turns
    // that step into a bounded transient (see PATH_OMEGA_SLEW_DPS2).
    float w = sm.omega_dps;
    float dw = path_slew_dps2 / CONTROL_LOOP_HZ;
    if (w > path_omega_prev + dw) w = path_omega_prev + dw;
    if (w < path_omega_prev - dw) w = path_omega_prev - dw;
    float a_ang_dps2 = (w - path_omega_prev) * CONTROL_LOOP_HZ;
    path_omega_prev = w;

    float yaw = fusion_yaw_deg();
    // Heading reference is the PATH's yaw at s, relative to the yaw the
    // mouse was standing at when the run launched. The lateral wall servo's
    // bias applies on STRAIGHTS only: inside an arc the yaw reference is
    // doing real work and a bias on top of it steers the arc itself. The
    // reflex channel applies everywhere -- opening an arc away from a
    // post is its whole point (control_set_reflex_bias_deg).
    //
    // THE BIASES ARE SLEWED, WITH FEEDFORWARD (2026-09-06). A bias used to
    // be a STEP in the reference closed by the heading P term alone
    // (kyaw 10 -> 50 mm/s of wheel differential for a 5 deg error): D8
    // lap 83 held a -5 deg lane-change bias for 180 mm of diagonal and the
    // heading moved 2.4 deg. The rate feedforward is what turns the plant
    // in an arc; a reference change gets the same treatment here -- the
    // applied bias ramps at PATH_BIAS_SLEW_DPS and its rate goes into
    // dv_ff, so the heading follows the reference instead of trailing it,
    // and an arc entry drops the bias smoothly instead of as a step.
    // Two channels, two rates (Control.h PATH_LAT_BIAS_SLEW_DPS /
    // PATH_REFLEX_SLEW_DPS): the D8 laps of 2026-09-06 14:42 showed a
    // 150 deg/s ramp reaching only ~50 deg/s of measured yaw rate -- the
    // wheel-speed loop cannot follow a 33 ms pulse -- so the lane change
    // ramps slower and holds its feedforward longer; the reflex keeps the
    // fast ramp because it is a twitch and its P-term residue is harmless.
    float lat_target = sm.in_arc ? 0.0f : move_lat_bias;
    float dl = lat_target - path_lat_live;
    float dlmax = PATH_LAT_BIAS_SLEW_DPS / CONTROL_LOOP_HZ;
    if (dl > dlmax) dl = dlmax;
    if (dl < -dlmax) dl = -dlmax;
    path_lat_live += dl;
    float dr = move_reflex_bias - path_rfx_live;
    float drmax = PATH_REFLEX_SLEW_DPS / CONTROL_LOOP_HZ;
    if (dr > drmax) dr = drmax;
    if (dr < -drmax) dr = -drmax;
    path_rfx_live += dr;
    path_bias_live = path_lat_live + path_rfx_live;
    float w_bias = (dl + dr) * CONTROL_LOOP_HZ * PATH_BIAS_FF_GAIN;   // deg/s
    float yaw_ref = path_yaw0 + sm.yaw_deg + path_bias_live;
    float yaw_err = wrap180(yaw_ref - yaw);

    float half_track = 0.5f * vctrl_gains.track_mm;
    // Rate feedforward plus the measured CW/CCW asymmetry (VCTRL_TASYM):
    // a sign-independent CCW boost proportional to |omega|, half-weighted
    // on right turns because that is how the 'k' harness measured it.
    // ARC UNDERSTEER (see VCTRL_ARC_UNDERSTEER): a sign-INDEPENDENT fractional
    // boost, because runs 180-182 put both the left and the right U-turn
    // outside the compiled arc by the same 10 mm. It multiplies the rate
    // feedforward rather than adding to it, so it vanishes on a straight and
    // scales with how hard the corner is.
    float dv_ff = ((w + w_bias) * DEG2RAD) * half_track *
                  (1.0f + vctrl_gains.arc_understeer);
    // Then the SIGNED CW/CCW asymmetry, which is a different measurement and
    // keeps its own constant.
    float asym = vctrl_gains.tasym * fabsf(w * DEG2RAD) * half_track;
    dv_ff += (w >= 0.0f) ? asym : 0.5f * asym;

    float dv_fb = vctrl_gains.kyaw * yaw_err;
    if (dv_fb > vctrl_gains.dvmax) dv_fb = vctrl_gains.dvmax;
    if (dv_fb < -vctrl_gains.dvmax) dv_fb = -vctrl_gains.dvmax;

    // The integral's own gates (yaw_int_update) freeze it during rotation,
    // so it learns on the straights and self-scales into the arcs.
    yaw_int_update(yaw_err, w, v);
    float dv_hold = yaw_int_c * v;

    float dv_total = dv_ff + dv_fb + dv_hold + vctrl_gains.strim * v;
    vctrl_target_left = v - dv_total;
    vctrl_target_right = v + dv_total;

    float a_lin = (v - v_prev) * CONTROL_LOOP_HZ;
    // The slip model rides on the commanded acceleration -- see ControlGains.
    if (path_slip_on)
        path_slip += path_slip_step(a_lin, 1.0f / CONTROL_LOOP_HZ,
                                    vctrl_gains.slip_acc, vctrl_gains.slip_brk);
    if (path_slip > PATH_SLIP_MAX_MM) path_slip = PATH_SLIP_MAX_MM;
    if (path_slip < -PATH_SLIP_MAX_MM) path_slip = -PATH_SLIP_MAX_MM;
    float a_ang = (a_ang_dps2 * DEG2RAD) * half_track;
    vctrl_acc_left = a_lin;
    vctrl_acc_right = a_lin;
    vctrl_ang_left = -a_ang;
    vctrl_ang_right = a_ang;

    path_s = s;
    path_v_cmd = v;
    path_omega = w;
    path_yaw_ref = yaw_ref;
    path_yaw_err = yaw_err;
    path_in_arc = sm.in_arc;

    float v_enc = 0.5f * (odom_vel_left + odom_vel_right);
    if (sm.at_end && fabsf(v_enc) < MOVE_DONE_VEL_MM_S) {
        path_done = true;
        path_active = false;
        vctrl_target_left = vctrl_target_right = 0.0f;
        vctrl_acc_left = vctrl_acc_right = 0.0f; vctrl_ang_left = vctrl_ang_right = 0.0f;
        move_lat_bias = 0.0f;
        move_reflex_bias = 0.0f;
    } else if (tick_count - path_start_tick > path_timeout_ticks) {
        // done stays false: something stalled, and the caller must say so
        // rather than the run quietly reporting success.
        path_active = false;
        vctrl_target_left = vctrl_target_right = 0.0f;
        vctrl_acc_left = vctrl_acc_right = 0.0f; vctrl_ang_left = vctrl_ang_right = 0.0f;
        move_lat_bias = 0.0f;
        move_reflex_bias = 0.0f;
    }
}

static void control_tick() {
    ImuSample samples[MAX_PACKETS_PER_TICK];
    size_t n = imu_read_fifo(samples, MAX_PACKETS_PER_TICK);

    // Feed every packet to the orientation filter at the IMU sample rate
    // (FUSION_SAMPLE_DT per packet, ~13 updates per tick in steady state),
    // and average the batch's X accel for the motion estimator (averaging
    // rejects the 20 kHz-adjacent motor/gear vibration the single latest
    // sample would alias in).
    int32_t ax_sum = 0, ay_sum = 0;
    for (size_t i = 0; i < n; i++) {
        const ImuSample &s = samples[i];
        fusion_update(imu_gyro_dps(s.gx), imu_gyro_dps(s.gy),
                      imu_gyro_dps(s.gz), imu_accel_g(s.ax),
                      imu_accel_g(s.ay), imu_accel_g(s.az));
        ax_sum += s.ax;
        ay_sum += s.ay;
    }

    if (n > 0) {
        latest_sample = samples[n - 1];
        state_valid = true;
        total_samples += n;
    }
    last_batch = n;
    tick_count++;

    odometry_tick();
    motion_tick(n > 0 ? imu_accel_g(ax_sum / (int32_t)n) : 0.0f,
                n > 0 ? imu_accel_g(ay_sum / (int32_t)n) : 0.0f, n > 0);
    // MEASURED YAW RATE, every tick, whoever is driving.
    //
    // It used to be computed inside move_tick(), which returns immediately
    // when no MOVE is active -- so through an entire path run it stayed at
    // whatever the last move left, and the flight log recorded a column of
    // zeros. omega/v is the curvature the mouse ACTUALLY flew, which is the
    // one number that separates "the arc is wrong" from "the arc is right and
    // the mouse did not follow it". Runs 176-179 needed exactly that and did
    // not have it.
    {
        float yaw_now = fusion_yaw_deg();
        move_yaw_rate = wrap180(yaw_now - move_prev_yaw) * CONTROL_LOOP_HZ;
        move_prev_yaw = yaw_now;
    }
    move_tick();
    path_tick();
    velocity_tick();
}

bool control_init() {
    // Starts the settle delay + gyro bias calibration: the mouse must be
    // stationary for ~3 s after this call (2 s settle + 1 s bias window;
    // fusion_ready() / control_heading().ready then flips true). As more
    // subsystems gain init steps, keep this call last so their disturbance
    // is over before the bias window runs.
    fusion_reset();
    return hal_control_timer_begin(control_tick, CONTROL_LOOP_HZ);
}

ControlHeading control_heading() {
    ControlHeading h;
    noInterrupts();
    h.ready = fusion_ready();
    h.yaw_deg = fusion_yaw_deg();
    h.pitch_deg = fusion_pitch_deg();
    h.roll_deg = fusion_roll_deg();
    interrupts();
    return h;
}

void control_set_heading_deg(float yaw_deg) {
    noInterrupts();
    fusion_set_yaw_deg(yaw_deg);
    interrupts();
}

void control_recalibrate() {
    noInterrupts();
    fusion_reset();
    yaw_int_c = 0.0f;  // fresh mission: relearn the standing trim
    interrupts();
}

ControlOdometry control_odometry() {
    int32_t l, r;
    float vl, vr;
    noInterrupts();
    l = odom_left_counts;
    r = odom_right_counts;
    vl = odom_vel_left;
    vr = odom_vel_right;
    interrupts();

    ControlOdometry o;
    o.left_mm = encoder_mm(l) * g_odo_scale_l;
    o.right_mm = encoder_mm(r) * g_odo_scale_r;
    o.dist_mm = 0.5f * (o.left_mm + o.right_mm);
    o.vel_left_mm_s = vl;
    o.vel_right_mm_s = vr;
    o.vel_mm_s = 0.5f * (vl + vr);
    return o;
}

void control_velocity_enable(bool enable) {
    if (enable) motors_enable(true);  // release STBY before the ISR drives

    noInterrupts();
    vctrl_target_left = vctrl_target_right = 0.0f;
    vctrl_acc_left = vctrl_acc_right = 0.0f; vctrl_ang_left = vctrl_ang_right = 0.0f;
    vctrl_prev_err_left = vctrl_prev_err_right = 0.0f;
    vctrl_duty_left = vctrl_duty_right = 0.0f;
    // The budget does NOT reset on enable/disable: the motors do not cool
    // because the mission ended, and a session that aborts and restarts
    // twice a minute would otherwise get an unlimited burst every time.
    vctrl_enabled = enable;
    interrupts();

    if (!enable) {
        motors_stop();  // after the flag drops: the ISR won't re-drive
        motors_enable(false);
    }
}

float control_brake_to_stop(float v_stop_mm_s, uint32_t timeout_ms) {
    if (!vctrl_enabled) return 0.0f;
    // Zero the TARGETS but leave the loop running: velocity_duty() then sees
    // a large negative error and commands reverse duty by itself. Zero the
    // accel feedforward too -- a stale a_ref would fight the brake.
    //
    // AND TAKE THE EXECUTORS OFF THE TARGETS FIRST (2026-09-26, the search
    // that ended "path stalled" against a wall). path_tick and move_tick
    // rewrite vctrl_target_* every tick while they are active, so the zero
    // written here lasted one tick and the path drove on: the log shows the
    // hard front guard fire at 70 mm, this call block the main loop for
    // 393 ms while the mouse covered 62 mm at 234 mm/s, and return only
    // when the WALL brought the wheels under 60 mm/s. Every abort since run
    // 151 that "braked" from inside a path or a move did the same -- the
    // callers all stop the executor AFTER this returns. So: retire the
    // executors here, before the zero (aborted, not done; the callers'
    // control_move_abort afterwards is then a no-op), and the wheel loop
    // owns the wheels for the whole brake.
    noInterrupts();
    path_active = false;
    path_done = false;
    move_active = false;
    move_done = false;
    path_pend = 0.0f;
    vctrl_target_left = vctrl_target_right = 0.0f;
    vctrl_acc_left = vctrl_acc_right = 0.0f; vctrl_ang_left = vctrl_ang_right = 0.0f;
    interrupts();
    uint32_t t0 = millis();
    float v = 0.0f;
    for (;;) {
        noInterrupts();
        float vl = odom_vel_left, vr = odom_vel_right;
        interrupts();
        v = 0.5f * (vl + vr);
        if (fabsf(v) <= v_stop_mm_s) break;
        if (millis() - t0 >= timeout_ms) break;
        // Spin. The control ISR is what does the braking; this only waits for
        // it. No yield(): on Teensy that runs the event responder, and this
        // path is an abort -- the shortest, dumbest wait is the right one.
        // The noInterrupts()/interrupts() pair above is a compiler barrier,
        // so the velocity reads cannot be hoisted out of the loop.
    }
    return v;
}

ControlGains control_get_gains() {
    ControlGains g;
    noInterrupts();
    g = vctrl_gains;
    interrupts();
    return g;
}

void control_set_gains(const ControlGains &g) {
    noInterrupts();
    vctrl_gains = g;
    g_odo_scale_l = (g.circ_left_mm > 1.0f)
                        ? g.circ_left_mm / ENC_WHEEL_CIRCUMFERENCE_MM : 1.0f;
    g_odo_scale_r = (g.circ_right_mm > 1.0f)
                        ? g.circ_right_mm / ENC_WHEEL_CIRCUMFERENCE_MM : 1.0f;
    interrupts();
}

void control_set_wheel_velocity(float left_mm_s, float right_mm_s) {
    noInterrupts();
    vctrl_target_left = left_mm_s;
    vctrl_target_right = right_mm_s;
    vctrl_acc_left = vctrl_acc_right = 0.0f; vctrl_ang_left = vctrl_ang_right = 0.0f;
    interrupts();
}

void control_set_wheel_motion(float left_mm_s, float right_mm_s,
                              float left_acc_mm_s2, float right_acc_mm_s2) {
    noInterrupts();
    vctrl_target_left = left_mm_s;
    vctrl_target_right = right_mm_s;
    vctrl_acc_left = left_acc_mm_s2;
    vctrl_acc_right = right_acc_mm_s2;
    vctrl_ang_left = vctrl_ang_right = 0.0f;
    interrupts();
}

ControlVelStatus control_velocity_status() {
    ControlVelStatus s;
    noInterrupts();
    s.enabled = vctrl_enabled;
    s.target_left_mm_s = vctrl_target_left;
    s.target_right_mm_s = vctrl_target_right;
    s.duty_left = vctrl_duty_left;
    s.duty_right = vctrl_duty_right;
    s.duty_peak = vctrl_duty_peak;
    s.duty_cm_peak = vctrl_cm_peak;
    interrupts();
    return s;
}

void control_duty_peak_reset() {
    noInterrupts();
    vctrl_duty_peak = 0.0f;
    vctrl_cm_peak = 0.0f;
    interrupts();
}

bool control_maneuver_start(float dist_mm, float v_mm_s, float a_mm_s2,
                            float angle_deg, float omega_dps, float alpha_dps2) {
    if (!vctrl_enabled || !fusion_ready()) return false;
    if (path_active) return false;  // the two executors never overlap

    TrapProfile lin, ang;  // plan outside the masked section (sqrt etc.)
    lin.begin(dist_mm, v_mm_s, a_mm_s2);
    ang.begin(angle_deg, omega_dps, alpha_dps2);

    noInterrupts();
    move_pure_turn = (dist_mm == 0.0f && angle_deg != 0.0f);
    move_lin_prof = lin;
    move_ang_prof = ang;
    move_start_tick = tick_count;
    move_d0 = mot_d_est;
    move_yaw0 = fusion_yaw_deg();
    // Pure-turn planner setup (closed-loop decel; see move_tick).
    move_turn_target_deg = wrap180(move_yaw0 + angle_deg);
    move_turn_omega_dps = fabsf(omega_dps);
    move_turn_alpha_dps2 = fabsf(alpha_dps2);
    move_turn_finish = false;
    move_prev_yaw = move_yaw0;
    move_v_ref = move_a_ref = move_d_ref = 0.0f;
    move_v_cmd = move_yaw_err = 0.0f;
    move_omega_ref = move_yaw_rate = 0.0f;
    move_yaw_ref = move_yaw0;
    move_dv_sum = move_vcmd_sum = 0.0f;
    move_cruise_n = 0;
    move_lat_bias = 0.0f;  // each move starts unbiased (see Control.h)
    move_reflex_bias = 0.0f;
    move_done = false;
    fg_capping = false; fg_cap_ticks = 0;
    move_active = true;
    interrupts();
    return true;
}

void control_set_lateral_bias_deg(float bias_deg) {
    if (bias_deg > LAT_SERVO_BIAS_MAX_DEG) bias_deg = LAT_SERVO_BIAS_MAX_DEG;
    if (bias_deg < -LAT_SERVO_BIAS_MAX_DEG) bias_deg = -LAT_SERVO_BIAS_MAX_DEG;
    noInterrupts();
    move_lat_bias = bias_deg;
    interrupts();
}

void control_path_frame_adjust(float d_deg) {
    noInterrupts();
    if (path_active) {
        path_yaw0 += d_deg;
        path_frame_adj += d_deg;
    }
    interrupts();
}

void control_set_reflex_bias_deg(float bias_deg) {
    if (bias_deg > FLINCH_MAX_DEG) bias_deg = FLINCH_MAX_DEG;
    if (bias_deg < -FLINCH_MAX_DEG) bias_deg = -FLINCH_MAX_DEG;
    noInterrupts();
    move_reflex_bias = bias_deg;
    interrupts();
}

bool control_move_start(float dist_mm, float v_mm_s, float a_mm_s2) {
    return control_maneuver_start(dist_mm, v_mm_s, a_mm_s2, 0.0f, 0.0f, 0.0f);
}

bool control_move_start_heading(float dist_mm, float v_mm_s, float a_mm_s2,
                                float yaw_ref_deg) {
    if (!vctrl_enabled || !fusion_ready()) return false;
    if (path_active) return false;  // the two executors never overlap

    TrapProfile lin, ang;  // plan outside the masked section
    lin.begin(dist_mm, v_mm_s, a_mm_s2);
    ang.begin(0.0f, 0.0f, 0.0f);

    noInterrupts();
    move_pure_turn = false;
    move_lin_prof = lin;
    move_ang_prof = ang;
    move_start_tick = tick_count;
    move_d0 = mot_d_est;
    move_yaw0 = wrap180(yaw_ref_deg);  // ABSOLUTE reference, not captured
    move_prev_yaw = fusion_yaw_deg();
    move_v_ref = move_a_ref = move_d_ref = 0.0f;
    move_v_cmd = move_yaw_err = 0.0f;
    move_omega_ref = move_yaw_rate = 0.0f;
    move_yaw_ref = move_yaw0;
    move_dv_sum = move_vcmd_sum = 0.0f;
    move_cruise_n = 0;
    move_lat_bias = 0.0f;  // each move starts unbiased (see Control.h)
    move_reflex_bias = 0.0f;
    move_done = false;
    fg_capping = false; fg_cap_ticks = 0;
    move_active = true;
    interrupts();
    return true;
}

bool control_turn_start(float angle_deg, float omega_dps, float alpha_dps2) {
    return control_maneuver_start(0.0f, 0.0f, 0.0f, angle_deg, omega_dps,
                                  alpha_dps2);
}

bool control_arc_start(float radius_mm, float angle_deg, float omega_dps,
                       float alpha_dps2) {
    // Couple linear <-> angular by the radius so the two profiles share the
    // same time structure -> a true circular arc. Forward distance is the arc
    // length R*|theta|; the angular profile carries the (signed) direction.
    float omega_rad = omega_dps * DEG2RAD;
    float alpha_rad = alpha_dps2 * DEG2RAD;
    float dist = radius_mm * fabsf(angle_deg) * DEG2RAD;
    return control_maneuver_start(dist, radius_mm * omega_rad,
                                  radius_mm * alpha_rad, angle_deg, omega_dps,
                                  alpha_dps2);
}

ControlMoveTrim control_move_trim() {
    ControlMoveTrim t;
    noInterrupts();
    uint32_t n = move_cruise_n;
    float dv = move_dv_sum;
    float v = move_vcmd_sum;
    interrupts();
    t.valid = n >= 50;  // >= 100 ms of cruise
    t.avg_dv_mm_s = t.valid ? dv / n : 0.0f;
    t.avg_v_mm_s = t.valid ? v / n : 0.0f;
    return t;
}

void control_move_abort() {
    noInterrupts();
    move_active = false;
    move_done = false;
    path_active = false;
    path_done = false;
    path_pend = 0.0f;
    yaw_int_c = 0.0f;
    vctrl_target_left = vctrl_target_right = 0.0f;
    vctrl_acc_left = vctrl_acc_right = 0.0f; vctrl_ang_left = vctrl_ang_right = 0.0f;
    interrupts();
}

bool control_move_extend(float extra_mm) {
    bool ok = false;
    noInterrupts();
    // Pure straight only: an angular profile (turn/arc) must run to its
    // planned end -- extending the linear leg under it would break the
    // radius coupling.
    if (move_active && !move_pure_turn &&
        move_ang_prof.duration_s() == 0.0f) {
        float t = (tick_count - move_start_tick) * (1.0f / CONTROL_LOOP_HZ);
        ok = move_lin_prof.extend(extra_mm, t);
    }
    interrupts();
    return ok;
}

bool control_path_start(const SpeedPath *path, float yaw0_offset_deg) {
    if (!vctrl_enabled || !fusion_ready()) return false;
    if (path == nullptr || path->length_mm() <= 0.0f) return false;
    bool ok = false;
    noInterrupts();
    if (!move_active && !path_active) {
        path_ptr = path;
        path_d0 = mot_d_est;
        path_yaw0 = fusion_yaw_deg() + yaw0_offset_deg;
        path_frame_adj = 0.0f;
        path_bias_live = path_lat_live = path_rfx_live = 0.0f;
        path_bias = 0.0f;
        path_pend = 0.0f;
        path_slip = 0.0f;
        path_slip_on = false;   // launch straight exempt, see Control.h
        path_v_prev = 0.0f;
        path_omega_prev = 0.0f;
        path_drain_rate = PATH_CORR_DRAIN_FRAC;
        path_s = 0.0f;
        path_v_cmd = 0.0f;
        path_omega = 0.0f;
        path_yaw_ref = path_yaw0;
        path_yaw_err = 0.0f;
        path_in_arc = false;
        path_done = false;
        move_lat_bias = 0.0f;  // never carries in from a previous move
        move_reflex_bias = 0.0f;
        yaw_int_c = 0.0f;      // fresh run: relearn the standing trim
        fg_capping = false; fg_cap_ticks = 0;
        path_start_tick = tick_count;
        // Timeout: twice the time the path would take at HALF v_max, plus
        // two seconds. Generous on purpose -- this is a stall detector, not
        // a performance gate, and a run that trips it has already failed in
        // a way worth reporting rather than a way worth tuning.
        float t_nom = path->length_mm() / (0.5f * path->v_max());
        path_timeout_ticks =
            (uint32_t)((2.0f * t_nom + 2.0f) * CONTROL_LOOP_HZ);
        path_active = true;
        ok = true;
    }
    interrupts();
    return ok;
}

void control_set_path_slew(float dps2) {
    if (dps2 < 500.0f) dps2 = 500.0f;      // sane floor
    if (dps2 > 20000.0f) dps2 = 20000.0f;  // sane ceiling
    noInterrupts();
    path_slew_dps2 = dps2;
    interrupts();
}

float control_path_slew() { return path_slew_dps2; }

void control_path_slip_enable(bool on) {
    noInterrupts();
    path_slip_on = on;
    interrupts();
}

float control_path_slip_take() {
    noInterrupts();
    float took = path_slip;
    path_bias += path_slip;   // s = d + bias + slip is unchanged by this
    path_slip = 0.0f;
    interrupts();
    return took;
}

void control_path_extended() {
    noInterrupts();
    if (path_active && path_ptr) {
        float t_nom = path_ptr->length_mm() / (0.5f * path_ptr->v_max());
        path_timeout_ticks =
            (uint32_t)((2.0f * t_nom + 2.0f) * CONTROL_LOOP_HZ);
    }
    interrupts();
}

bool control_path_swap(const SpeedPath *next, float s_in_next_mm) {
    if (next == nullptr || next->length_mm() <= 0.0f) return false;
    bool ok = false;
    noInterrupts();
    if (path_active && path_ptr) {
        // s = (d - d0) + bias + slip must read s_in_next on the new path:
        // move the datum, keep the corrections (they are physical).
        float s_now = (mot_d_est - path_d0) + path_bias + path_slip;
        path_d0 += s_now - s_in_next_mm;
        path_ptr = next;
        path_s = s_in_next_mm;
        path_start_tick = tick_count;
        float t_nom = next->length_mm() / (0.5f * next->v_max());
        path_timeout_ticks =
            (uint32_t)((2.0f * t_nom + 2.0f) * CONTROL_LOOP_HZ);
        ok = true;
    }
    interrupts();
    return ok;
}

void control_path_correct(float ds_mm, float over_mm) {
    if (ds_mm > PATH_CORR_MAX_MM) ds_mm = PATH_CORR_MAX_MM;
    if (ds_mm < -PATH_CORR_MAX_MM) ds_mm = -PATH_CORR_MAX_MM;
    float rate = PATH_CORR_DRAIN_FRAC;
    if (over_mm > 1.0f) {
        rate = fabsf(ds_mm) / over_mm;
        if (rate < PATH_CORR_DRAIN_FRAC) rate = PATH_CORR_DRAIN_FRAC;
        if (rate > 1.0f) rate = 1.0f;  // never more than 1 mm of s per mm run
    }
    noInterrupts();
    if (path_active) {
        path_pend += ds_mm;
        path_drain_rate = rate;
    }
    interrupts();
}

ControlPathStatus control_path_status() {
    ControlPathStatus s;
    noInterrupts();
    s.active = path_active;
    s.done = path_done;
    s.s_mm = path_s;
    s.v_mm_s = path_v_cmd;
    s.omega_dps = path_omega;
    s.frame_deg = path_frame_adj;
    s.yaw_ref_deg = path_yaw_ref;
    s.yaw_err_deg = path_yaw_err;
    s.in_arc = path_in_arc;
    s.corr_mm = path_bias;
    s.slip_mm = path_slip;
    s.corr_pending_mm = path_pend;
    s.front_gov = fg_capping;
    s.bias_deg = path_bias_live;
    interrupts();
    return s;
}

// -- The front governor: feed and status (Control.h) --------------------------
void control_front_guard_feed(uint8_t front_left_mm, uint8_t front_right_mm, uint32_t age_ms) {
    bool wall = false;
    float d = 0.0f;
    if (age_ms < FRONT_GOV_STALE_MS && front_left_mm != 255 && front_right_mm != 255) {
        float fl = (float)front_left_mm, fr = (float)front_right_mm;
        if (fl <= FRONT_GOV_RANGE_MM && fr <= FRONT_GOV_RANGE_MM &&
            fabsf(fl - fr) <= FRONT_GOV_AGREE_MM) {
            wall = true;
            d = (fl < fr) ? fl : fr;
        }
    }
    fg_d_mm = d;      // distance first, then the flag that makes it live
    fg_wall = wall;
}
void control_front_guard_enable(bool on) { fg_enabled = on; }
ControlFrontGuard control_front_guard() {
    ControlFrontGuard g;
    noInterrupts();
    g.enabled = fg_enabled;
    g.wall = fg_wall;
    g.d_mm = fg_wall ? fg_d_mm : 0.0f;
    g.capping = fg_capping;
    g.cap_ticks = fg_cap_ticks;
    interrupts();
    return g;
}

ControlMoveStatus control_move_status() {
    ControlMoveStatus s;
    noInterrupts();
    s.active = move_active;
    s.done = move_done;
    s.v_ref = move_v_ref;
    s.a_ref = move_a_ref;
    s.d_ref = move_d_ref;
    s.v_cmd = move_v_cmd;
    s.d_mm = mot_d_est - move_d0;
    s.omega_ref_dps = move_omega_ref;
    s.yaw_ref_deg = move_yaw_ref;
    s.yaw_err_deg = move_yaw_err;
    s.yaw_rate_dps = move_yaw_rate;
    s.lat_bias_deg = move_lat_bias;
    interrupts();
    return s;
}

ControlWheelSources control_wheel_sources() {
    noInterrupts();
    ControlWheelSources s = wheel_src;
    interrupts();
    return s;
}

ControlMotion control_motion() {
    ControlMotion m;
    noInterrupts();
    m.v_est_mm_s = mot_v_est;
    m.d_est_mm = mot_d_est;
    m.v_imu_mm_s = mot_v_imu;
    m.a_fwd_mm_s2 = mot_a_fwd;
    m.a_lat_mm_s2 = mot_a_lat;
    m.wheel_acc_mm_s2 = mot_wheel_acc;
    m.slip = mot_slip;
    m.slip_events = mot_slip_events;
    m.abs_ticks = abs_ticks;
    m.abs_events = abs_events;
    m.abs_active = abs_on_left || abs_on_right;
    m.v_abs_mm_s = abs_v;
    interrupts();
    return m;
}

void control_motion_zero() {
    noInterrupts();
    mot_v_est = 0.0f;
    mot_d_est = 0.0f;
    mot_d_enc = 0.0f;
    mot_d_off = 0.0f;
    mot_v_imu = 0.0f;
    mot_slip = false;
    mot_slip_ctr = 0;
    mot_recover_ctr = 0;
    abs_ticks = 0; abs_events = 0; abs_v = 0.0f;
    abs_clamp_ticks = 0; abs_timed_out = false;
    for (int i = 0; i < ABS_VMAX_TICKS; i++) abs_venc_hist[i] = 0.0f;
    abs_on_left = abs_on_right = false;
    mot_slip_events = 0;
    interrupts();
}

ControlImuStatus control_imu_status() {
    ControlImuStatus s;
    noInterrupts();
    s.latest = latest_sample;
    s.total_samples = total_samples;
    s.ticks = tick_count;
    s.last_batch = last_batch;
    interrupts();
    if (!state_valid) {
        s.latest = ImuSample{};
    }
    return s;
}
