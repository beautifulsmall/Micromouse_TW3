// Control layer.
//
// Time-critical control code running from a fixed-rate hardware timer ISR,
// per the ADR's decision to use a single fixed-rate ISR instead of a full
// RTOS. Currently: drains the IMU FIFO every tick. Wheel velocity control,
// heading correction, and sensor fusion will layer in here.
//
// This layer depends only on the Drivers layer (never touches raw pins/
// buses directly). Keep ISR code deterministic and free of blocking calls
// (no delay(), no unbounded loops).
//
// NOTE: the ISR owns the SPI bus while it runs. Once the FRAM driver
// exists, main-loop SPI use must be coordinated with this ISR (e.g.
// NVIC priority / brief timer mask around main-loop transactions).

#pragma once

#include <Arduino.h>

#include "Drivers.h"
#include "Fusion.h"
#include "Profile.h"

class SpeedPath;  // Path.h -- only ever held by pointer here

// Control loop rate. 500 Hz to start with; IMU ODR (6.4 kHz) is set
// independently in the IMU driver -- the FIFO absorbs the difference.
constexpr float CONTROL_LOOP_HZ = 500.0f;

// Starts the fixed-rate control ISR. Call after imu_init().
// Returns false if no hardware timer slot was available.
bool control_init();

// Snapshot of the most recent IMU sample + counters, safe to call from the
// main loop (copies shared state with the ISR briefly masked).
struct ControlImuStatus {
    ImuSample latest;          // most recent decoded FIFO packet
    uint32_t total_samples;    // packets decoded since boot
    uint32_t ticks;            // ISR invocations since boot
    uint16_t last_batch;       // packets drained on the most recent tick
};
ControlImuStatus control_imu_status();

// Orientation estimate from the Madgwick fusion running in the ISR (see
// Fusion.h for frames/conventions). Safe to call from the main loop.
// `ready` is false until the settle delay + gyro bias calibration complete
// (~3 s stationary after control_init(): 2 s settle + 1 s bias window).
struct ControlHeading {
    float yaw_deg;    // heading, +CCW from above, (-180, +180]
    float pitch_deg;  // +nose up
    float roll_deg;   // +right side down
    bool ready;
};
ControlHeading control_heading();

// Re-reference heading to maze-wall ground truth (wraps
// fusion_set_yaw_deg with the ISR masked). Main-loop safe.
void control_set_heading_deg(float yaw_deg);

// Restart the IMU settle + gyro-bias calibration (fusion_reset, ISR
// masked). control_heading().ready reads false until it completes (~3 s,
// mouse stationary; the motion guard extends it if disturbed). Used by
// the hand-start sequence so the bias window runs AFTER the mouse was
// last touched -- the power-on calibration happens while the mouse is
// still being handled and is superseded by this one. Main-loop safe.
void control_recalibrate();

// Wheel odometry sampled each tick from the hardware quadrature decoders
// (forward = positive; see Encoders.h). Velocities are per-tick count
// deltas low-pass filtered with ODOM_VEL_FILTER_ALPHA (a raw 500 Hz delta
// is only ~10 counts at 1 m/s, so unfiltered velocity is quantisation-
// noisy). Main-loop safe.
struct ControlOdometry {
    float left_mm;      // distance travelled by left wheel since zero
    float right_mm;     // distance travelled by right wheel since zero
    float dist_mm;      // (left + right) / 2 -- mouse centre distance
    float vel_left_mm_s;
    float vel_right_mm_s;
    float vel_mm_s;     // (left + right) / 2
};
ControlOdometry control_odometry();

// An alternative wheel-velocity source (EncoderTiming): called every tick
// with this tick's count deltas; fills the velocities in NOMINAL mm/s and
// says whether it has a fresh measurement. Used only when the registry's
// vsrc selects it. Unset (the host tests): counts per tick as always.
typedef void (*ControlWheelVelHook)(int32_t d_left, int32_t d_right,
                                    float *v_left, float *v_right, bool *valid);
void control_set_wheel_velocity_hook(ControlWheelVelHook hook);
// Both sources as the last tick saw them (scaled mm/s), for the bench
// 'enc' readout: the raw count velocity, the edge velocity, and the
// filtered value the loop is actually using.
struct ControlWheelSources {
    float count_l, count_r;   // counts per tick x mm per count x Hz
    float edge_l, edge_r;     // EncoderTiming M/T velocity (0 when no hook)
    bool edge_ok;             // the hook reported a valid measurement
    bool edge_in_use;         // vsrc selected it AND it was valid
    float filt_l, filt_r;     // after vfilt: what the wheel loop sees
};
ControlWheelSources control_wheel_sources();

// Single-pole IIR coefficient for the velocity estimates: y += a*(x - y)
// per 500 Hz tick; 0.2 gives a ~10 ms time constant.
constexpr float ODOM_VEL_FILTER_ALPHA = 0.2f;

// -- Wheel velocity control --------------------------------------------------
// Per-wheel feedforward + PD, evaluated every control tick:
//
//   duty = v_ref / VCTRL_VMAX_*_MM_S            (feedforward: bulk of the duty)
//        + VCTRL_FRICTION_* * sign(v_ref)       (static-friction offset)
//        + VCTRL_KP * err + VCTRL_KD * d(err)/dt  (PD trims the residual)
//
// Gains are RUNTIME values (tunable over serial, persisted in FRAM via
// Params.h). The constants below are the compiled-in defaults, used when
// no valid parameter block exists in FRAM; the current values came from
// the 's' duty-sweep on hardware 2026-07-02 (free-air, on blocks).
//
// VMAX and FRICTION are per wheel: motors/gear trains never match exactly.
// The 's' sweep fits them (v = m*duty + c => VMAX = m, FRICTION = -c/m)
// and applies the result to the runtime gains. KP/KD: start soft --
// feedforward does most of the work; raise KP until tracking is tight,
// add KD if it oscillates.
constexpr float VCTRL_VMAX_LEFT_MM_S = 7515.0f;   // 's' sweep 2026-07-02
constexpr float VCTRL_VMAX_RIGHT_MM_S = 7591.0f;  // 's' sweep 2026-07-02
constexpr float VCTRL_FRICTION_LEFT = 0.002f;     // 's' sweep 2026-07-02
constexpr float VCTRL_FRICTION_RIGHT = 0.002f;    // 's' sweep 2026-07-02
constexpr float VCTRL_KP = 0.0004f;  // duty per mm/s
constexpr float VCTRL_KD = 0.0f;     // duty per mm/s^2

// SPLIT velocity P gain for IN-PLACE turns. Turning in place scrubs the tyres
// sideways at low wheel speed, and breaking that scrub friction needs a much
// stiffer velocity loop than rolling does -- but that stiff kp is past the
// straight-line stability margin (hardware 2026-07-07: 'c' legs buzzing /
// limit-cycling after 't' raised the then-shared kp). So: pure rotations
// (control_turn_start, dist == 0) run with KP_TURN; straights, arcs (wheels
// roll), and plain velocity commands run with KP. The 't' auto-tune raises
// KP_TURN only. Default matches KP (soft) -- run 't' to tune it up.
constexpr float VCTRL_KP_TURN = 0.0004f;  // duty per mm/s, in-place turns only

// Acceleration feedforward: duty added per mm/s^2 of COMMANDED accel (from the
// motion profiler, see Profile.h), so a ramp gets the inertia duty up front
// instead of leaning on the PD to accumulate error. Default 0 -- inert until
// tuned on the floor: run the 'a' profile with ka=0 first, then raise ka until
// v_enc tracks v_ref through the ramps with minimal lag.
// WARNING (2026-07-09): a corrupted ka (~1.5e-4) in FRAM caused accel surge
// (+65% over profile), braking wheel-lock, and ~12 mm decel-tail rollback on
// EVERY profiled move -- misdiagnosed for two days as stiction stall. If
// moves surge then roll back at the stop, check ka ('p') before anything else.
constexpr float VCTRL_KA = 0.0f;     // duty per mm/s^2

// Move-executor gains (see the move section below). KX closes the position
// loop so velocity-tracking shortfall cannot shorten a move; KYAW is the
// IMU heading hold that keeps straights straight.
constexpr float VCTRL_KX = 3.0f;    // (mm/s) of correction per mm of d error
constexpr float VCTRL_KYAW = 2.0f;  // (mm/s) differential per deg of yaw error

// Steering trim: FEEDFORWARD differential fraction of the commanded speed,
// compensating systematic left/right asymmetry (wheel diameter / motor
// gain) so the heading hold works around zero instead of fighting a
// standing disturbance. Positive trim speeds the RIGHT wheel (steers CCW,
// same sign convention as the heading-hold dv). Found automatically by the
// 'c' straight-line calibration (main.cpp): a wheel-gain error needs the
// same dv/v in forward AND reverse, while gyro-bias effects flip sign --
// alternating +/-200 mm legs isolates the true trim.
constexpr float VCTRL_STRIM = 0.0f;

// SLIP-ANGLE FEEDFORWARD: degrees of yaw offset INTO a turn per mm/s^2 of
// commanded lateral acceleration (a = v*omega). A tyre only makes lateral
// force at a slip angle, so following a curve needs the body rotated into
// it relative to the path tangent; command the bare tangent and the mouse
// holds the heading and still runs wide. Applied to the COMMANDED omega so
// it ramps in with the curvature -- pre-adjusted entering the turn, with no
// time lookahead (this is NOT the retired PATH_ANG_LEAD_S: that advanced
// path POSITION and cut corners; this offsets HEADING at the same s).
// Default 0 = inert until proven on the floor, same policy as VCTRL_KA.
// Order of magnitude: a few tenths of a degree per 1000 mm/s^2, so ~3e-4.
constexpr float VCTRL_KSLIP = 0.0f;      // deg per mm/s^2
constexpr float VCTRL_KSLIP_MAX_DEG = 12.0f;  // clamp on the ff term

// Effective wheel track (contact-point separation) used to feed a commanded
// body rotation rate forward into a wheel-speed differential during turns:
// wheel_dv = omega * (track / 2). The geometric value is 59 mm (spec 6.4);
// in-place turns scrub the tyres so the EFFECTIVE track runs a little larger.
// Runtime-tunable and auto-fit by the 't' turn test (TrackTuner, Profile.h).
constexpr float VCTRL_TRACK_MM = 59.0f;

// Turn-asymmetry feedforward: the 'k' corner harness (2026-07-21)
// measured a consistent CW rotational bias under sustained differential
// -- right turns over-rotated (+5 deg standing, corner cut 25-40 mm
// short), left turns under-rotated (+8..10 deg, wide into the outer
// wall) at every radius tested. Sign-independent |omega|-proportional
// CCW boost on the path executor's differential:
//   dv_ff = omega*(track/2) + tasym*|omega|*(track/2)
// Tune on the harness: raise until 'kl' exit yaw_err ~ 0, then check
// 'kr' stays near 0 (split any residual). 0 disables.
constexpr float VCTRL_TASYM = 0.10f;

// -- ARC UNDERSTEER: the mouse always runs OUTSIDE its arc ---------------------
//
// Runs 180-182, cross-track reconstructed from the flown yaw against the
// compiled clothoid-arc-clothoid, measured at the U-turn:
//
//     run 180  +12.4 mm outside      run 181  +9.7      run 182  +19.2
//
// and on the SAME run's left-hand U-turn (181 arc 6) +10.0 against the
// right-hand's +9.7. That symmetry is the whole finding. A signed bias --
// yaw_int's learned trim c*v, or VCTRL_TASYM -- pushes one hand out and the
// other IN. Both hands running out by the same amount is not a bias; it is a
// TRACKING LAG, and the only sign-independent way to lag a commanded rotation
// is to under-command the differential that produces it.
//
// The wheel loop is not the lag. Its feedforward is v/vmax (exact by
// construction) and its proportional gain is 0.0004 duty per mm/s against a
// 7515 mm/s-per-duty plant -- a loop gain of 3 per tick at 500 Hz, settling in
// one or two. The wheels deliver what they are asked for. What is asked for is
// wrong: omega = dv / track holds for rolling wheels, and a mouse in a
// sustained arc is not rolling cleanly -- the tyres carry a slip angle, so the
// body rotates less than the wheel-speed difference says it should.
//
// Solving the steady state gives the size of it. With kyaw as the only
// heading feedback inside an arc (the yaw integral is gated off above 20 dps),
//
//     yaw_err_ss = omega * (track_eff - track) / (2 * kyaw)
//
// At the U-turn's 5.576 rad/s and kyaw 2.0, the observed 10 deg of steady lag
// needs track_eff - track = 7.2 mm, i.e. an effective track near 66 against
// the 59 the geometry gives. Same coefficient predicts 10.7 deg at the R84
// nineties' 5.95 rad/s, of which they reach ~58 % before the arc ends (0.26 s
// against the loop's 0.30 s time constant) = 6.2 deg. Measured on run 181's
// nineties: 5.4, 7.0, 8.7, 10.9. It holds.
//
// SO WHY DOES THE MOUSE END UP OUTSIDE RATHER THAN JUST LATE? Because nothing
// in an arc measures position. kyaw closes on HEADING, and heading does
// recover -- every arc exits within a few degrees, and total sweep is 98-99 %
// of commanded. Cross-track is the INTEGRAL of the heading error, and a P loop
// that drives the error to zero leaves the integral wherever it got to. Ten
// degrees of lag held for 150 mm is 26 mm of permanent offset, and the mouse
// carries it to the end of the arc.
//
// That is also why the U-turn is worse than a ninety and not merely different:
// 308 mm of arc against 160, so twice the distance for the same lag to
// accumulate over. It is a lever arm, not a rate.
//
// The correction is a fractional boost on the rate feedforward, applied by the
// PATH executor only. In-place turns keep VCTRL_TRACK_MM alone -- that value
// is auto-fit by the 't' test against a genuinely scrubbing spin, which is a
// different regime with its own number.
//
// 0.12 = 7.2/59. PROVISIONAL, in two ways worth stating:
//
//  - It is calibrated against the COMPILED track of 59. Params in FRAM
//    override compiled defaults, so if the 't' test has already fitted a
//    larger track into FRAM, part of this is being counted twice. The route
//    run's settings banner now records the live track so the next log says
//    which. Check it before tuning this further.
//  - Linear in omega is the simplest model that fits, not the only one. Tyre
//    understeer is properly proportional to lateral acceleration, i.e. to
//    omega*v, and the U-turn (319 dps) and the nineties (341 dps) are too
//    close in rate to tell the two apart. One arc flown at half speed
//    separates them: linear-in-omega halves the lag, understeer quarters it.
//
// Erring high is the safer direction. Over-boosting makes the mouse cut the
// turn, which moves the apex AWAY from the end wall and leaves the exit
// laterally short -- and the corridor's lateral servo pulls that back on the
// next straight. Under-boosting leaves it heading for the wall.
constexpr float VCTRL_ARC_UNDERSTEER = 0.0f;

// -- DUTY: ONE CEILING, AND IT NEVER MOVES ------------------------------------
//
// Motors are 3 V rated on a 7.4 V supply, so full duty is 2.5x rated. This
// used to be two numbers and a thermal budget: a 0.45 continuous ceiling, a
// 0.85 burst ceiling, and an I^2t-style integrator that folded the mouse back
// to 0.45 after ~1.9 s of burst and let it recover over 30 s.
//
// bert, 2026-08-27: "the battery budget is up to me, earlier limits should be
// removed, the mouse can run for 10 mins off a single battery, there should be
// no J budget or limits that change related to energy budget."
//
// So the budget is gone -- not disabled, deleted. The reason to actually
// remove it rather than set J_MAX to zero is that a ceiling which MOVES is
// unfalsifiable from a log: every wide corner, every soft launch, every
// under-rotation had "maybe the budget folded" as a live explanation that cost
// real time to rule out. A fixed ceiling can be checked against the logged
// duty in one line.
//
// For the record, so nobody re-derives it: at these speeds the budget was
// never binding anyway. vmax is ~7500 mm/s at duty 1.0, so a 500 mm/s cruise
// asks for 0.067 feedforward and a 319 deg/s arc adds 0.044 of differential.
// Measured route-run duty sits around 0.15-0.25. The 0.45 fold could not have
// been reached outside a launch or a hard brake.
//
// What remains is a single hard clamp, applied per wheel in velocity_duty().
// It is a MOTOR limit -- protection against sustained stall, which is the one
// thing that actually kills these motors -- not an energy budget, and it does
// not vary with time, temperature, battery state or anything else.
constexpr float VCTRL_DUTY_LIMIT = 0.85f;

// Runtime controller gains; defaults match the VCTRL_* constants above.
struct ControlGains {
    float vmax_left_mm_s = VCTRL_VMAX_LEFT_MM_S;
    float vmax_right_mm_s = VCTRL_VMAX_RIGHT_MM_S;
    float friction_left = VCTRL_FRICTION_LEFT;
    float friction_right = VCTRL_FRICTION_RIGHT;
    float kp = VCTRL_KP;
    float kp_turn = VCTRL_KP_TURN;  // in-place turns only (scrub breakaway)
    float kd = VCTRL_KD;
    float ka = VCTRL_KA;
    float kx = VCTRL_KX;
    float kyaw = VCTRL_KYAW;
    float strim = VCTRL_STRIM;
    float track_mm = VCTRL_TRACK_MM;
    float tasym = VCTRL_TASYM;  // turn-asymmetry ff (see VCTRL_TASYM)
    // Fractional boost on the path executor's rate feedforward; arcs only.
    // See VCTRL_ARC_UNDERSTEER. 0 restores the pre-2026-08-27 behaviour.
    float arc_understeer = VCTRL_ARC_UNDERSTEER;
    float kslip = VCTRL_KSLIP;  // slip-angle ff (see VCTRL_KSLIP)
    // PER-WHEEL measured circumference: the two wheels differ, which at equal
    // PWM drifts the nose, so each odometer gets its own count->mm scale.
    // 'j' (the scale test) measures both.
    //
    // MEASURED 2026-08-21, three straight runs, yaw < 0.3 deg throughout:
    //
    //     believed L  907.10  (sd 0.00)
    //     believed R  904.50  (sd 0.14)
    //     actual      906.67  (sd 0.47, tape)
    //
    // The number that matters is L - R = 2.60 mm over 907, and it has NO
    // ruler in it: on a straight run both wheels cover the same ground, so a
    // 2.6 mm disagreement is purely the two constants disagreeing. It repeats
    // to 0.14 mm, where the tape only resolves ~1 mm -- which is why the
    // differential, not the absolute, is the useful output of 'j'.
    //
    //     circl = 67.82 * 906.67 / 907.10 = 67.788   (-0.048 %, below the
    //                                                 tape's own resolution)
    //     circr = 67.94 * 906.67 / 904.50 = 68.103   (+0.240 %, real)
    //
    // So the RIGHT wheel is bigger than the firmware believed -- it had been
    // set 0.18 % BELOW left when it should sit 0.46 % ABOVE.
    //
    // Worth knowing what that was costing: 2.60 mm of false differential over
    // one 900 mm straight, divided by the 72.31 mm track, is 2.06 deg of
    // rotation the wheels reported and the mouse never made. The ring's
    // measured CW/CCW asymmetry at tasym=0 is -1.83 deg. Same order, same
    // sign, within 12 % -- suggestive, NOT established, because the heading
    // loop closes on the gyro and should absorb a wheel-scale error. Re-fly
    // the ring after this and see whether the asymmetry moves; that is the
    // test, not the arithmetic.
    //
    // NOTE THE ORDER OF OPERATIONS. `strim` was fitted with the OLD constants
    // and has been quietly absorbing part of this 0.24 %. Changing these
    // makes strim wrong, so: set the circumferences, re-run 'c' to re-fit
    // strim, THEN re-fly the ring. Doing the ring first measures the two
    // errors fighting each other.
    float circ_left_mm = 67.788f;
    float circ_right_mm = 68.103f;
    // -- APPENDED 2026-08-27 (bert: "all motor, feed forward and pid
    // parameters" belong in the registry). Each of these was a compiled
    // constant that someone had already wanted to move; the constants they
    // shadow remain in this header as the DEFAULTS ONLY.
    //
    // Velocity-estimate IIR alpha (was ODOM_VEL_FILTER_ALPHA). At 0.2 this is
    // ~10 ms of lag INSIDE the wheel feedback loop -- a real slice of the
    // measured ~15 ms transport lag that runs every arc wide. Tunable so the
    // lag/noise trade can be walked on the floor instead of recompiled.
    float vfilt = ODOM_VEL_FILTER_ALPHA;
    // Heading-correction differential clamp, mm/s (was MOVE_DVCORR_MAX_MM_S).
    float dvmax = 240.0f;
    // Yaw-trim integral: learn rate (c per deg*s) and coefficient cap (was
    // YAW_INT_KC / YAW_INT_C_MAX).
    float kyawi = 0.0133f;
    float kyawimax = 0.10f;
    // Route mission: cruise speed, longitudinal accel, lateral cap, and the
    // rate rail its geometry is compiled against (were the ROUTE_RUN_*
    // constants). THE RAIL AND THE GEOMETRY MOVE TOGETHER: RouteRun compiles
    // the path AND programs the executor from this one value, so they cannot
    // disagree -- which is the bug class the 2026-08-27 rail fix closed.
    // Changing any of these takes effect at the next arm (the route is
    // compiled per leg).
    float route_v = 500.0f;      // rv
    float route_alon = 700.0f;   // ralon
    float route_alat = 4900.0f;  // ralat
    float route_slew = 6000.0f;  // rslew
    // THE ARC WHEEL GAIN, its own number at last (2026-08-28). Path arcs
    // borrowed kp_turn since 2026-07-10, when the soft kp delivered a third
    // of the commanded rotation. The CAL-6 A/B (runs 188/189) measured both
    // ends of that borrowing: at bert's scrub-tuned kpt=0.0107 a 79 mm/s
    // wheel error rails the duty at 0.85 -- the "gritty" turn noise was the
    // duty chattering against the rail, and it still tracked tolerably
    // (bulge +6..+18) because bang-bang torque averages out; at the soft
    // 0.0004 the turns went silent and UNDER-delivered -- the tooth U-turn
    // swept 168.5 of 180 and bulged +27. Arcs need a stiffness between the
    // two: enough to drive ~330 mm/s of differential against turn
    // resistance, low enough to stay off the rail. 0.003 is the geometric
    // middle of the A/B; ladder it on the rig (watch duty peak < ~0.6 and
    // sweep completion -> 180) and 'w' what wins. In-place turns keep
    // kp_turn untouched -- search spins still need the scrub breakaway.
    float kp_arc = 0.003f;   // kparc
    // -- appended 2026-08-29 (high-speed corner work) ------------------------
    // goff: bias on WHERE the gate expects each corner's post-drop, mm along
    // track. Positive = expect it later. Shifts every gated corner's anchor
    // the same way, so a systematic sensor-geometry offset (the beam sees
    // the post edge early/late) is one knob instead of eight trims.
    float gate_off_mm = 0.0f;    // goff
    // rur: U-turn radius, mm. >0 forces the radius and solves the clothoid
    // around it (71.74 = the forced -20% experiment that has flown since run
    // 184); 0 = solve the radius from the corner rail instead (the original
    // geometry, ~89.7 at v500). Compiled per leg at arm, like rv.
    float route_uturn_r = 71.74f;  // rur
    // -- appended 2026-08-29 (bert): PER-CORNER-TYPE gate offsets -----------
    // The corner-type database's first column, living in the registry so it
    // gets every existing door (GUI, 'w', clamps, LPARAM) for free. Each is
    // ADDED to the global goff for corners of its type. Positive = expect
    // the post later = the mouse believes it is behind = the turn starts
    // LATER in the world ("the right turns exit tight, they should be
    // started later" -> goffr positive). Gated corners only: a NO_EDGE
    // corner takes no correction, so no offset can steer it -- that class
    // needs the compiler-level per-type trim (L3), not a gate bias.
    float goff_r90 = 0.0f;   // goffr  -- right 90s
    float goff_l90 = 0.0f;   // goffl  -- left 90s
    float goff_ur = 0.0f;    // goffur -- right 180s
    float goff_ul = 0.0f;    // gofful -- left 180s
    // gklat / gklatr: the gate's lateral compensation, mm of along-track
    // drop correction per mm of lateral toward the inside wall, PER BEAM.
    // From the SPEC laser geometry (tools/sensor_cal_config.json): the
    // side beams CROSS -- the LEFT beam (ch0) fires from the emitter on the
    // right of the body, 17.2 deg forward of square -> slope tan(17.2) =
    // 0.31; the RIGHT beam (ch3) from the left emitter at 19.0 -> 0.34.
    // (This comment had them the other way round until 2026-09-06.) (A 20 mm bench-offset estimate of 0.18/0.68 was
    // yaw-contaminated and is retired; the spec values flew clean in laps
    // 101-106, 2026-08-29.) gklat is the LEFT beam (left turns watch it);
    // gklatr the RIGHT (right turns). 0 disables. RouteRun picks the
    // coefficient per corner by hand.
    float gate_klat = 0.0f;    // gklat  -- LEFT beam
    float gate_klat_r = 0.0f;  // gklatr -- RIGHT beam
    // glearn: gain of the per-corner goff table's learning update
    // (GoffTable.h): at each leg end every LOCKED corner folds in
    // g += glearn * (-corr). 0 freezes the table (race day); 1 jumps to
    // the last measurement. 0.5 converges in 2-3 laps on the ~1.5 mm
    // lap-to-lap sigma measured in laps 101-106 (2026-08-29).
    float gate_learn = 0.5f;   // glearn
    // r90 / l90: per-direction override of the 90-degree radius ladder's
    // top rung, mm. 0 = compiled ROUTE_RADII[0] (84, the max-clearance
    // radius: arc centred on the inside post); >0 forces that radius for
    // the direction, demotions walk the compiled ladder below, U-turns
    // keep rur. The lever is corner speed (sqrt(ralat x R): R84 -> 641
    // mm/s at ralat 4900, R100 -> 700); the cost is clearance margin the
    // compiler does not model -- the parameter carries the risk. Compiled
    // per leg at arm, like rv.
    float r90_right = 0.0f;    // r90
    float r90_left = 0.0f;     // l90
    // atrim / utrim: ARC PLACEMENT trim, mm -- the ring rig's proven exit
    // lever (runs 120-122: 21 mm of exit per 20 mm of trim), reborn for the
    // maze. Subtracted from the APPLIED gate correction (never from the
    // expectation): positive = the arc starts LATER in the world = the exit
    // moves toward the OUTSIDE wall. This is the only placement knob the
    // glearn loop cannot nullify -- the goff family shifts the expectation,
    // and the learner drives corr to zero against the physical edge, so
    // those knobs cancel within two laps ("changing goffr didn't have much
    // effect", bert 2026-08-30, laps 120-150: exits +12..20 mm inside at
    // EVERY radius 45..84). Changing a trim makes the table re-converge
    // once (bounded by GOFF_TABLE_STEP_MM); gated corners only.
    float arc_trim_90 = 0.0f;   // atrim -- both 90 directions
    float arc_trim_u = 0.0f;    // utrim -- both 180 directions
    // fgate: enable the FRONT-WALL gate (>0 = on). Corners whose turn cell
    // has a wall dead ahead anchor on that wall's range -- orthogonal to
    // the path, so nearly immune to lateral error, seen earlier than the
    // side wall-end, and sampled as a ramp (median of 5) rather than one
    // edge event. Corners without a front wall keep the side-edge gate.
    // 0 = off; 1 = OBSERVE (measure + record fcorr, never steer);
    // 2 = FALLBACK (steer only where the side gate found no edge -- the
    //     side edge stays the primary key, see the FGATE notes);
    // 3 = PRIMARY (front leads, side refines: the aggressive mode).
    float front_gate = 0.0f;    // fgate
    // ftrim: FRONT-gate bias trim, mm. The front beams read the wall face
    // at 216-254 mm -- far beyond the front LUT's calibration (contact,
    // 45 mm) -- so a standing long-range offset is expected. Set ftrim to
    // the standing FRONT corr and it goes to zero. Deliberately NOT
    // learnable: the goff table is shared with the side gate, and a front
    // bias fed into it would walk the side gate off its post.
    float front_trim = 0.0f;    // ftrim
    // dgate: the DIAGONAL gate (Route.h, the diagonal note; RouteRun's
    // DGATE block). On a diagonal the posts flick past on alternate sides
    // every 127.3 mm, and each one is seen by the side beam as a ramp then
    // a sharp drop-out. Laps 291-296 (2026-09-01): six posts, every
    // drop-out within 2-9 mm of the geometry (mean +6.2, sd 2.6), and the
    // closest reading gives the lateral (face at 57.6 from the line).
    // 0 = OBSERVE (record every post against prediction, never steer);
    // 1 = ALONG-TRACK (correct s off each drop-out edge; each post is a
    //     fresh anchor with the rest of the diagonal to drain into);
    // 2 = ALONG-TRACK + LATERAL (also hold a lateral bias from the post
    //     minima, refreshed at every post -- the posts steer).
    float diag_gate = 0.0f;     // dgate
    // dtrim: diagonal edge trim, mm, added to the predicted drop-out of
    // every post. The geometry says drop = post foot - beam lead + post
    // half-width; the first flight measured the real edge +6.2 mm later
    // than that (sd 2.6). TUNING RULE: dtrim = mean(obs - pred) from a
    // dgate=0 session; the DGATE lines print both.
    float diag_trim = 0.0f;     // dtrim
    // dlat: where the diagonal's centre line IS, mm, + = left of where the
    // post minima say it is. At dgate=2 the posts steer the mouse toward
    // dlat instead of toward zero. Exists because the lateral MEASUREMENT
    // has a small standing bias -- the two side beams disagree by ~3 mm on
    // the same geometry (L 82 / R 85 in a corridor; right-side posts read
    // ~3 mm lower than left on the diagonal) -- so a servo to "measured
    // zero" parks a couple of mm off the true line. TUNING RULE: fly
    // dgate=2 with dlat=0, read the DGATE lat column once it settles, and
    // set dlat to where the mouse should sit instead. Does nothing below
    // dgate=2.
    float diag_lat = 0.0f;      // dlat
    // Duty ceiling, per wheel (`dmax`). The motor-protection clamp that
    // VCTRL_DUTY_LIMIT documents (3 V motors on 7.4 V; sustained STALL is
    // what kills them, and a crash pinned against a post is a stall), now
    // in the registry so it can be A/B'd between laps -- fixed for the
    // whole lap and printed in LPARAM, so it stays falsifiable from a log
    // (the 08-27 rule: a ceiling that MOVES mid-run is not). Raising it
    // buys mid-corner yaw authority (laps 400-402: dutypk pegged at 0.85
    // while the (5,5) arc understeered at ralat 7000) but NOT grip --
    // past the tyres' slide line extra duty only spins the wheels. Raise
    // on a fresh battery first: sag costs the same headroom for free.
    float duty_limit = VCTRL_DUTY_LIMIT;  // dmax
    // THE SLIP MODEL (`sacc`, `sbrk`), mm of odometry error per m/s of
    // commanded speed change. A driven wheel slips in the direction of its
    // torque: under acceleration it turns further than the body moves
    // (odometry OVER-reads, the mouse thinks it is ahead of where it is),
    // under braking it turns less (UNDER-reads). Both are proportional to
    // the speed change that produced them, so the per-tick model is
    //     s_true - s_wheel  -=  sacc * dv          while accelerating
    //     s_true - s_wheel  +=  sbrk * |dv|        while braking
    // integrated on the COMMANDED acceleration (what makes the torque),
    // added to path s between anchors, and folded into the gate bias at
    // each anchor (control_path_slip_take) so it never grows unbounded.
    // Default 0 = inert (the VCTRL_KA policy). Fit from the lap report's
    // ds / dva / dvb / corr columns: with the right numbers the gate corr
    // shrinks to its noise floor across a ralon ladder instead of walking,
    // and -- the point -- the goff table stops absorbing speed-dependent
    // slip as if it were corner geometry, so ralon becomes a free axis
    // like rv (RACE_PLAN.md, 2026-09-04: "positive drift = wheels ahead of
    // the world" was this term, unmodelled). Units chosen so 10 means 10 mm
    // per 1000 mm/s gained; the 2026-09-04 ralon-1800 laps put the launch
    // straight's over-read at ~40 mm for ~+800 mm/s, i.e. sacc ~ 50.
    float slip_acc = 0.0f;      // sacc
    float slip_brk = 0.0f;      // sbrk
    // THE FLINCH (`flinch`, mm, 0 = off). bert, 2026-09-05: "as the wall
    // visibly comes within a limit the mouse should react... early mice
    // relied purely on wall data... it should be almost a basic instinct."
    // A raw side beam closer than this on a STRAIGHT adds a steering bias
    // away from it, FLINCH_DEG_PER_MM per mm inside the limit, on top of
    // the corridor servo (which is 0.125 deg/mm and needs both walls).
    // Straights only, on purpose: through a normal 90 the OUTER beam reads
    // 36-45 at the apex and diagonal posts pass at 41-48, so inside an arc
    // a distance alone cannot tell the wall the geometry puts there from
    // the post that is about to be hit -- and flinching from the outer
    // wall of a right-hander steers INTO its inside post. The arc cases
    // get their own fixes (the 45-in entry-lateral shift, DIAGONAL.md).
    // Start at 40: the CCW laps of 2026-09-05 rode (2,5) at lat -21 with
    // the beam at ~63 and the servo too weak to care.
    float flinch_mm = 0.0f;     // flinch
    // THE VECTOR REFLEX (`ftau`, ms, 0 = off) -- the flinch's arc case.
    // bert, 2026-09-05: "the mouse should twitch away from any very close
    // contact... the problem is how fast, and that needs vector data."
    // Lap 51 (D7 CW) clipped the 45-in's inside post with a rear corner
    // from the SAME place lap 50 passed clean; the only difference in the
    // logs was that the inside beam HELD a target through the arc and was
    // closing at ~0.55 mm per mm of travel (380 mm/s) from 64 down to 40,
    // where lap 50's beam had let go at 64. Range alone cannot see that
    // (62 mm is a normal number 11 degrees into a turn); range over
    // closing rate -- time to contact -- can, and it saw it 75 mm and
    // ~105 ms before the hit. So in an ARC the reflex watches the INSIDE
    // beam only (the outer one is meant to close to 36-45 at the apex),
    // computes tau = range / closing speed, and when the range is inside
    // REFLEX_ARC_RANGE_MM and tau is under this many ms it biases the
    // heading AWAY from the inside post -- opening the arc -- by up to
    // FLINCH_MAX_DEG, the full step once tau is under half of ftau. The
    // diagonal's posts sweep past with a steep ramp (130 -> 45 in 60 mm,
    // tau ~50 ms on a clean pass), so tau is NOT used there: the
    // diagonal reflex is a plain range limit (REFLEX_DIAG_RANGE_MM).
    // Simulated on the 2026-09-05 logs: 200 fires on lap 51 from s2521
    // and nowhere on lap 50. Start at 200.
    float flinch_tau_ms = 0.0f; // ftau
    // WALL-FRAME ALIGNMENT (`walign`, gain 0..1, 0 = off). bert, 2026-09-05:
    // "the mouse when moving has the walls as the absolute, the gyro comes
    // into use to measure against odometry for wheel slip, for turns. Is
    // the gyro updated with wall alignment?" It was not: the path's yaw
    // datum was the fused yaw at launch -- the BODY's frame -- and nothing
    // ever moved it. A mouse that settled 2 deg clockwise on its datum
    // press (laps 56/57/60) flew the whole lap in a frame 2 deg off: on
    // every straight the lateral servo traded angle for offset (lap 60's
    // launch straight: heading +1.7 in the frame, still drifting right
    // 8 mm over 700 mm), every arc was entered with that bias dropped,
    // (2,5) and the U missed their gates, (3,5) needed -43/-63 with a
    // 9-12 deg yaw error, and both laps hit the diagonal's first left
    // post. The three hand-aligned laps of the same session were clean.
    // With this on, on every straight with BOTH walls the route runner
    // fits the corridor's direction from the walls -- lateral drift rate
    // vs measured heading, over FRAME_FIT_MM of travel -- and rotates the
    // path frame toward it by this fraction of the estimate per window,
    // at most FRAME_STEP_MAX_DEG per window and FRAME_LAP_MAX_DEG per lap
    // (control_path_frame_adjust). The gyro keeps the short term; the
    // walls own the frame. Start at 0.8.
    float wall_align = 0.0f;    // walign
    // vout: THE V USES THE ROOM OUTSIDE IT, mm (0 = off). bert, 2026-09-07:
    // "coming into that bend offset gives no margin for error, it's worth
    // the vout addition to use the space available even at a time cost."
    // Around a V the outside is empty: the diagonals' outer posts sit one
    // post pitch (127 mm) before and after the V station, and between them
    // the outer side is only the sawtooth valley at 191 mm. So the diagonal
    // lateral set point (dlat) is pushed OUTWARD by this much, ramped in
    // over the last 127 mm before the V's arc and back out over the first
    // 127 mm after it, and the V's radius is 64 + vout. Offset lines meet
    // an arc centred on the same inside post at the same stations, so the
    // arc timing does not move; the inside clearance grows by vout, the
    // outer posts lose nothing (the set point is back to dlat by them).
    // Bounded by how fast the lane change can happen (DGATE_SERVO_SCALE,
    // 5 deg clamp: ~9 mm in 100 mm) -- start at 8.
    float v_out_mm = 0.0f;      // vout
    // latk: THE CORRIDOR LATERAL SERVO'S GAIN, deg of heading bias per mm
    // of lateral error (the TUNABLE form of LAT_SERVO_DEG_PER_MM, whose
    // history is below that constant; the compiled value is this one's
    // default and what the search/sweep modes still use). The lane-change
    // step test (`lstep`, RouteRun) measured the loop at 0.15 on
    // 2026-09-07: +10 mm commanded at 1016 mm/s -> +8 in 200 mm then a
    // plateau at 60-80 %, +20 -> +13 in 250 mm, yaw error never over 1
    // deg, no overshoot -- a first-order lane change with a 382 mm length
    // constant (1/k in radians), i.e. the GAIN is the limit, not the
    // mouse. The two instabilities that drove 0.6 -> 0.3 -> 0.15 were the
    // inner heading loop's lag; since the biases are slewed with rate
    // feedforward (PATH_BIAS_FF_GAIN, 2026-09-06) the heading follows the
    // reference within a degree, so the loop should now take more. Raise
    // with the step test: 0.3 (191 mm), then 0.45; back off at the first
    // overshoot. The diagonal servo has its own gain (DGATE_SERVO_DEG_PER_MM).
    float lat_servo_k = 0.15f;  // latk
    // starty: WHERE THE AXLE IS WHEN THE BACK PANEL IS ON THE REAR WALL, mm
    // from the start cell's rear wall face... measured from the maze's
    // rear edge line (the cell's south wall face is at 6): 51 assumes the
    // axle sits 45 mm ahead of the back panel (Search.cpp
    // SEARCH_START_Y_DEFAULT_MM, and its note that the run data disagrees
    // with 51 "in the direction that puts the mouse FURTHER ALONG than the
    // firmware believes"). D8, 2026-09-10: diagonal A's post 1 -- the
    // first landmark after an ungated 45-in -- read 18-24 mm EARLY against
    // prediction on every lap of every set, with sacc 45 and with sacc 20
    // alike, i.e. the mouse is ~20 mm further along than s from the first
    // packet, which is a launch datum error, not slip. The path is
    // compiled with lead-in = 90 - starty, and the datum sequence drives
    // 90 - starty off the cross wall to the centreline, so one number
    // fixes both. Measure it: back panel to axle centre + 6.
    float start_y_mm = 51.0f;   // starty
    // r135 (2026-09-11): 1 = the route compiler may fly a diagonal into a
    // corridor (or a corridor into a diagonal) as ONE 135-degree arc
    // (Route.h, THE 135); 0 = the pre-135 parse -- the pair is a U-turn or
    // its two 90s, and the joins fly slow on the speed rung. Compiled per
    // leg like rv. A switch, not a tune: it exists so the same maze can be
    // flown both ways in one session while the 135 earns its gates.
    float route_135 = 1.0f;     // r135
    // sv: the rolling search's speed, mm/s (Explore.h). 100 is what bert
    // asked for; the geometry (R60 inside the cell, decisions 20 mm past
    // the boundary) holds to ~300, above which the side evidence gets thin.
    float search_v_mm_s = 100.0f;  // sv
    // THE IN-PLACE 180 (bert, 2026-09-15: "maybe we need to tune the 180s
    // anyway"). The goal turn and the search's dead-end turns share these;
    // the first recorded goal turn (D9, 450/4000) reached a mean 298 deg/s
    // against the 450 asked, sd 99, while the wheels ran the odometric
    // equivalent of 428 -- ~30 % tyre slip in place, judder, and a ~26 mm
    // sideways + ~30 mm along translation of the body. The rate the floor
    // can deliver is a tuning question, so it is a registry one.
    float turn_omega_dps = 450.0f;   // tomg: in-place turn cruise rate
    float turn_alpha_dps2 = 4000.0f; // talp: in-place turn ramp
    // THE RUN LADDER (RunLadder.h, bert 2026-09-18): peak velocity of the
    // OUT leg by run number, and of the HOME leg on every run. rv stays
    // the hand-set value the route flies when the ladder is off (mr=0).
    float rv_mr2 = 800.0f;    // rv2
    float rv_mr3 = 1000.0f;   // rv3
    float rv_mr4 = 1500.0f;   // rv4
    float rv_mr5 = 2000.0f;   // rv5
    float rv_home = 500.0f;   // rvh: the unscored return leg
    // THE ABS (bert, 2026-09-24: "lets do the abs now"). The second slip
    // ladder's collisions were LOCK-UPS: braking at 10000-12000 the wheel
    // loop drove the wheels to -500 mm/s while the body was still doing
    // 2000, the tyre let go, the odometry counted nothing and the mouse
    // slid 250 mm into the end wall. The same loop spun the wheels 30-45 %
    // ahead of the body accelerating at 10000+. Either way a wheel far from
    // body speed is transmitting nothing, so the loop's torque is wasted
    // and its odometry is a lie. abs_band is the fraction of body speed
    // (v_est, the fused/IMU estimate, +/- the commanded differential) the
    // wheel's velocity TARGET may sit above or below the ground's speed:
    // the target is clamped to that band before the loop sees it, so the
    // loop only ever asks for a slip ratio the tyre can deliver -- a
    // braking wheel is held at body - band, still rolling, still counting.
    // Continuous, not a duty override (v1 was, and it limit-cycled against
    // the loop -- Control.cpp). 0 = off. Floor 250 mm/s so a normal
    // launch's wheel lead never trips it. Measured healthy slip: 2-7 %
    // below the knee (Bench.cpp 'slip'), lock-ups 100 %+, so 0.12 sits
    // between; the tyre's peak force is near 10-15 %.
    float abs_band = 0.12f;   // absb
    // WHEEL VELOCITY SOURCE (EncoderTiming.h, 2026-09-25): 0 = counts per
    // tick (the 103 mm/s LSB the loop has always had), 1 = edge timing
    // through the XBAR/DMA capture -- the timer's resolution, one edge of
    // lag. A registry switch so the two can be A/B'd on the floor with
    // nothing else changed; the count path is untouched either way.
    float vel_source = 0.0f;   // vsrc
    // ANGULAR ACCELERATION FEEDFORWARD (2026-09-25). `ka` used to multiply
    // the whole per-wheel acceleration reference, linear AND the
    // differential the omega rail commands at every arc entry and exit
    // (rslew 7000 deg/s^2 x half-track = 4400 mm/s^2 per wheel, 0.22 of
    // duty at ka 5e-5 -- reversed at the exit). ka was fitted on the slip
    // ladder's straights, where that term is zero, and the first D9 laps
    // with ka 5e-5 (15:59, 16:01 -- vsrc 0, absb 0, ralon 1200, i.e. the
    // 09-22 registry plus ka) under-rotated EVERY corner against its sweep
    // (+3.8 / -5.4 / +10.2 deg of exit error; 09-22: +0.8 / +0.1 / +2.8),
    // sat 3-4 deg short on every diagonal, and hit the post at the 45-out
    // twice. So the angular part has its own gain, default 0: the arcs
    // fly as they did on 09-22, the launch keeps ka.
    // SWEPT the same evening (17:56-18:02, vsrc 1, absb 0.12): 1e-5 flew
    // a complete lap (exit errors within 2 deg, like 09-22); 2e-5, 3e-5,
    // 8e-5 and 1e-4 under-rotated more at every step (+2.5 / +3.1 / +5.4
    // / +6.4 at corner 0, -2.4 / -3.8 / -9.0 / -10.7 at corner 1) and hit
    // the 45-out post every time. Monotonic, and the wrong way for a
    // term meant to help: the pulse on the unloaded inner wheel costs
    // tyre grip (the ABS catches the lock), the yaw response drops. The
    // clamp is not the limit; the tyre is. Stays 0.
    float kaa = 0.0f;          // kaa, duty per mm/s^2 of DIFFERENTIAL accel
    // ROUTE BRAKE DECELERATION (2026-09-25, see SpeedPath::set_limits): the
    // planner's braking curves into corners and to the path end. 0 = ralon,
    // as it always was. The launch and the straights accelerate at ralon;
    // the corner-entry brake, which runs into the clothoid where the
    // lateral load is building, needs its own, smaller number: ralon 4000
    // flew MR2-MR5 clean, 5000 hit the V from a 45-in it had entered
    // braking at -8400 with the tyre gone.
    float route_brk = 0.0f;    // rbrk
    // The run ladder's acceleration scale at MR2 (RunLadder.h): ralon,
    // ralat and rbrk are multiplied by mra at MR2, rising to x1.0 at MR5.
    // 1.0 = every rung at the registry's accelerations, as before.
    float ladder_accel_mr2 = 1.0f;   // mra
    // BATTERY COMPENSATION (`vref`, 2026-09-26; bert: "keeping the watts
    // delivered to the motor in line with requested torque"). A DC motor's
    // torque is set by the voltage across it: V_batt x duty. The duty law
    // above -- v/vmax, the friction offset, ka x a -- was fitted at ONE pack
    // voltage, so at any other the same duty makes a different torque:
    // the pack this month swung 1.90..2.20 on the (uncalibrated) vbat
    // instrument, +/-7 % about 2.05, i.e. 15 % of plant gain across a
    // session, which the soft kp (0.0003) does not recover (a 15 % vmax
    // loss at 1000 mm/s is 66 mm/s of standing error) and the goff table
    // then learns as if it were corner geometry. With vref > 0 every duty
    // -- feedforward, friction, acceleration and the PD trim together,
    // since the plant gain scales as a whole -- is multiplied by
    // vref / vbat before the ceiling, so V_batt x duty, the torque, is
    // what was asked for at the calibration voltage. vref is in the
    // instrument's own units (what 'vbat' / LAPREP print), so the divider
    // calibration is irrelevant to it: only the ratio matters. Set it to
    // the reading the current tuning was made at (2.00-2.15 for the
    // September ladders) and the fresh-pack launch stops hitting dmax
    // early and the last lap of the session stops being the slow one.
    // 0 = off (default: the compensation changes the plant; one variable
    // per session). Clamped to VCTRL_VBAT_COMP_MIN..MAX; off below 1 V
    // (no sample yet).
    float vbat_ref = 0.0f;           // vref
    // THE CLOTHOID HOLD (`chold`, 2026-09-26; SpeedPath::set_clothoid_hold
    // tells the story: the MR5 V at (2,2) entered braking at 5000 while the
    // lateral came on to 4000). 1 = the planner brakes to the arc speed by
    // the clothoid START and holds it until the exit clothoid is done, so
    // the tyre never carries brake or launch and cornering load together.
    // 0 = the planner as it was (brake into the ramp, launch out of it).
    // Costs ~15 ms a corner. Default 1: the crash was at the registry the
    // race would fly.
    float clothoid_hold = 1.0f;      // chold (0/1)
    // rdiag (2026-09-26, the evening the new maze's home leg hit posts on
    // MR2 twice: a same-hand 45-out -> 90 mm -> 135-in -> V, entered 20 mm
    // inside off the diagonal's exit offset with no straight to recentre
    // in; bert: "crashes is not acceptable" on MR2). 1 = the route compiler
    // may fly diagonals (45s, 135s, Vs) as it always has; 0 = ORTHOGONAL
    // ONLY -- every staircase compiles as the 90s it was, on the R84 ring
    // geometry that has 43 mm of inside clearance and eight sessions of
    // proof. The banker's switch: MR2-MR4 bank a time on 90s, the diagonal
    // form is asked for at MR5 only, or not at all on a maze whose
    // diagonals pack same-hand turns. Compiled per leg at arm, like r135.
    float route_diag = 1.0f;         // rdiag (0/1)
    // rturn (2026-09-26, the MR3 that hit the (2,2) wall on the fourth of
    // eleven zigzag turns: a wall had moved, the cell flood found an 18-cell
    // route with 11 R45/R60 turns and preferred it to the 21-cell route with
    // 6 that had flown all evening). Cells of cost per 90 turn when the
    // route is chosen (Route.cpp walk_weighted; a 180 counts twice): at
    // 2000 mm/s a turn is worth three or four cells of time, and every
    // turn is a post to miss. 0 = the plain cell flood, as before. 3 =
    // the race default.
    float route_turn_cost = 3.0f;    // rturn
    // dwheel (2026-10-01): the PER-WHEEL duty ceiling for steering, above the
    // straight-line cap dmax (DutyMix.h). The common mode (speed) stays at
    // dmax; a wheel may go to dwheel to keep the differential (steering),
    // and if it would pass that the speed gives way, not the steering.
    // 0 = the old per-wheel clip at dmax.
    float wheel_duty_limit = 1.0f;   // dwheel
    // rturnh (2026-10-01; bert: "for the return leg could the mouse choose a
    // longer route but one with the fewest turns, which would be even
    // safer"). The turn cost the HOME leg's route choice uses, cells per
    // turn, in place of rturn: the home leg is not timed, so a turn there
    // is worth far more cells than on the way out. 0 = use rturn.
    float route_turn_cost_home = 8.0f;   // rturnh
    // rdv (2026-10-03): the out leg flies diagonals only when its peak is at
    // or under this (mm/s); above it the leg is compiled orthogonal, as the
    // home leg is. The two MR4 (1500) out legs at Hazlemere both hit a post
    // leaving a V; MR2 (800) and MR3 (1000) flew the same route's diagonals
    // to the goal. 0 = no limit.
    float diag_v_max = 1000.0f;   // rdv
    // rvv / rvu (2026-10-04, the Hazlemere test-day analysis; bert: "the
    // diagonals were clear so they can stay but the v and u were the
    // problem so reduce them to safer options", and "MR2 must be the
    // safest fast run"). Speed caps, mm/s, 0 = none. rvv: a V and the turn
    // it hands into within ROUTE_V_CAP_AFTER_MM (the 135 that took six of
    // the day's crashes) never fly faster than this on any rung -- 320 is
    // the V's own speed at MR2 (R64 at ralat x0.4), where every V flown
    // that day was clean. rvu: the same for a U-turn; ~44 Us flew clean at
    // 338-476 that day, so it ships at 0.
    float v_cap_v = 320.0f;   // rvv
    float v_cap_u = 0.0f;     // rvu
    // vlearn (2026-10-04, race morning): the V timing learner (DiagLane.h)
    // on (1) or off (0). On the small maze's chain of three Vs it read
    // "post -18, dead reckoning +5.8" at every V -- the dead reckoning,
    // seeded vout outside, was 24 mm wrong -- and shifted all three Vs
    // +12 mm late after MR2 and +19/18/20 (the clamp) after MR3; at
    // Hazlemere it shifted (6,11) +7 late from a post 165 mm out and MR4
    // left on that. A V timing error of a few mm is absorbed by the post
    // servo; a learned 20 mm is not. Off for the race.
    float v_learn = 0.0f;     // vlearn
    // ghome (2026-10-04, race morning; bert: "it would be better for the
    // mouse to do a loop of the goal, just like it does in the search,
    // because the goal may have another entrance opposite the one we come
    // in so there will be no wall to back up to. Make the back up or loop
    // an option I can set when I see the maze"). How the home leg of a
    // two-leg run leaves the goal: 0 = stop, turn in place, press back onto
    // the first known wall behind (RouteRun, the goal datum), fly home;
    // 1 = stop, then fly a LOOP through the 2x2 goal block -- ahead,
    // across, back -- and out (Route.h, THE GOAL LOOP), no turn in place,
    // no press. 1 falls back to 0 when the block around the arrival cell
    // is not a 2x2 of known-open interior.
    float goal_home = 0.0f;   // ghome
};
constexpr float VCTRL_VBAT_COMP_MIN = 0.75f;
constexpr float VCTRL_VBAT_COMP_MAX = 1.35f;
// Main loop: the latest pack reading (mV, Vbat.h) for the compensation.
void control_set_vbat_mv(uint16_t mv);
float control_vbat_comp();            // the factor in force (1 when off)
constexpr float FRAME_FIT_MM = 150.0f;      // straight per frame estimate
constexpr float FRAME_STEP_MAX_DEG = 1.0f;  // per window
constexpr float FRAME_LAP_MAX_DEG = 6.0f;   // per lap, either sign
constexpr float FLINCH_DEG_PER_MM = 0.5f;   // 10 mm inside the limit = 5 deg
constexpr float FLINCH_MAX_DEG = 8.0f;

// The reflex's arc law, pure so it can be tested. range_mm / closing_mm_s
// is the time to contact; returns the yaw bias magnitude (deg, >= 0):
// 0 outside REFLEX_ARC_RANGE_MM, not closing, or tau >= ftau; ramping to
// FLINCH_MAX_DEG as tau falls to ftau/2. Sign (away from the beam) is the
// caller's.
// 60 on the first build; a V at R60 legitimately reads its inside post at
// ~55 near the apex (D8 laps 87-90, clean), and lap 86's reflex kicked 13
// deg into a clean V and sent it 13 deg off the next diagonal. 50 still
// fires 25 mm before the 40 that clipped the D7 45-in.
constexpr float REFLEX_ARC_RANGE_MM = 50.0f;
// WHERE THE BODY IS, in beam units. A side beam reads from the sensor,
// not from the bodywork, and the 2026-09-05 D7 laps calibrated the
// difference on posts: lap 48 passed diagonal post 2 at a reading of 39
// and was heard to touch; lap 52 read 37 at the same post and hit it hard
// (ay +26000, heading knocked 24 deg) 49 mm later, when the body beside
// the axle reached it; laps 47/49/50 read 41-44 and were clean. So a post
// at a reading of ~38 IS the bodywork, and "posts pass at 41-48 on a
// centred run" means a centred diagonal has 3-10 mm in hand. Thresholds
// that mean "too close" are set from this number, not from 0.
// MEASURED per beam on the bench 2026-09-06 (Bench 'wcal s', three sweeps,
// FLINCH.md): body against the wall reads LEFT 40, RIGHT 36 -- the 38 was
// their mean. Neither beam is linear over its range (L +5 at contact, -2.5
// centred, +1 at 133; R +1, -1, -8), so a threshold set "from contact"
// on the left has 4 mm less in hand than the same number on the right.
constexpr float SIDE_BEAM_CONTACT_L_MM = 40.0f;
constexpr float SIDE_BEAM_CONTACT_R_MM = 36.0f;
constexpr float SIDE_BEAM_CONTACT_MM = 38.0f;   // the mean; kept for the thresholds below
// The diagonal reflex limit was 35 on the first build -- BELOW contact,
// which is why lap 52 hit post 2 with the reflex armed and silent.
constexpr float REFLEX_DIAG_RANGE_MM = 45.0f;
inline float reflex_arc_bias_deg(float range_mm, float closing_mm_s,
                                 float ftau_ms) {
    if (ftau_ms <= 0.0f || closing_mm_s <= 0.0f) return 0.0f;
    if (range_mm >= REFLEX_ARC_RANGE_MM) return 0.0f;
    float tau_ms = range_mm / closing_mm_s * 1000.0f;
    if (tau_ms >= ftau_ms) return 0.0f;
    float k = 2.0f * (1.0f - tau_ms / ftau_ms);   // 0 at ftau, 1 at ftau/2
    if (k > 1.0f) k = 1.0f;
    return FLINCH_MAX_DEG * k;
}

// One tick of the slip model above, pure so it can be tested: returns the
// change in the s correction for a commanded linear acceleration a
// (mm/s^2) over dt (s). Negative while accelerating (wheels ahead of the
// world), positive while braking.
inline float path_slip_step(float a_mm_s2, float dt_s, float sacc, float sbrk) {
    float dv_m_s = a_mm_s2 * dt_s * 1e-3f;   // speed change this tick, m/s
    return dv_m_s > 0.0f ? -sacc * dv_m_s : -sbrk * dv_m_s;
}

// Get/replace the gains the ISR uses (main-loop safe; the setter masks
// the ISR briefly). Takes effect on the next control tick.
ControlGains control_get_gains();
void control_set_gains(const ControlGains &g);

// Enable/disable closed-loop control (main-loop only, not ISR-safe).
// Enabling releases the H-bridge standby and resets targets + PD state to
// zero; disabling coasts the motors and re-enters standby.
void control_velocity_enable(bool enable);

// BRAKE, THEN STOP. Blocking, bounded, main-loop only.
//
// motors_stop() sets duty to zero, and zero duty is COAST (Motors.h says so:
// "0 = coast"). Every abort path went straight there, so a guard that tripped
// at speed handed the mouse to rolling resistance -- which at 1.2 m/s is
// most of a maze. Run 151 hit a wall doing exactly that (bert: "the braking
// seemed to fail... reverse pwm will slow a forward moving mouse").
//
// The fix needs no new actuator model. The wheel PI already produces reverse
// duty on its own when the target is zero and the measurement is not:
// err = 0 - v is large and negative, kp*err drives the duty negative, and the
// H-bridge brakes. What was missing is that control_velocity_enable(false)
// zeroed the targets and killed the loop in the SAME call, so the braking it
// had just asked for was never executed.
//
// So: hold the loop up with zero targets until the mouse is actually slow,
// then cut. Bounded by timeout as well as by speed, because a wheel off the
// ground never reaches the threshold and an abort must always terminate.
// Returns the speed it gave up at, for the caller to record.
float control_brake_to_stop(float v_stop_mm_s = 60.0f,
                            uint32_t timeout_ms = 700);

// Per-wheel velocity setpoints, mm/s, forward positive. Main-loop safe.
// Clears the accel feedforward references to zero (plain velocity command).
void control_set_wheel_velocity(float left_mm_s, float right_mm_s);

// Per-wheel velocity + acceleration references (mm/s, mm/s^2; forward
// positive), for profiled moves. The accel refs feed ControlGains::ka. Use
// this instead of control_set_wheel_velocity() when driving from a TrapProfile
// (see Profile.h). Main-loop safe.
void control_set_wheel_motion(float left_mm_s, float right_mm_s,
                              float left_acc_mm_s2, float right_acc_mm_s2);

// Controller state snapshot for bench logging. Main-loop safe.
struct ControlVelStatus {
    bool enabled;
    float target_left_mm_s;
    float target_right_mm_s;
    float duty_left;   // last commanded duty, post-clamp
    float duty_right;
    // Largest |duty| either wheel has been commanded since the last
    // control_duty_peak_reset(). This replaced the moving ceiling and the
    // thermal proxy J (removed 2026-08-27). The ceiling is now the constant
    // VCTRL_DUTY_LIMIT, so the only question worth asking a run afterwards is
    // whether it ever got NEAR it -- one number, and it cannot be fooled by
    // sampling at the wrong moment the way a live ceiling could.
    float duty_peak;
    // The COMMON MODE's peak (the mean of the two wheels: speed) -- what the
    // straight-line cap dmax limits since the duty mixer (DutyMix.h).
    // duty_peak above stays the worst single wheel (up to dwheel).
    float duty_cm_peak;
};
ControlVelStatus control_velocity_status();

// Zero the duty peak. Call at the start of anything whose saturation you want
// to know about (a route leg, a sweep lap, an acceleration step).
void control_duty_peak_reset();

// -- Profiled maneuver executor (position + heading closed loop) ---------------
// The executor runs TWO TrapProfiles on a shared clock in the 500 Hz tick: a
// LINEAR profile (mm) for forward travel and an ANGULAR profile (deg) for
// rotation. A pure straight has a zero angular profile; a pure on-the-spot
// turn has a zero linear profile; both together give a moving-and-turning
// maneuver (arcs). Closed-loop corrections handle what a bare time profile
// leaves open (velocity shortfall -> distance undershoot; wheel asymmetry /
// FF error -> heading error):
//
//   v_cmd  = v_ref + kx * (d_ref - d)        position P on the FUSED distance
//   yaw_ref = yaw0 + ang_d_ref               heading reference FOLLOWS the
//                                            angular profile (fixed for a
//                                            straight, ramps for a turn)
//   dv_ff  = omega_ref * (track / 2)         rotation-rate feedforward
//   dv_fb  = kyaw * wrap(yaw_ref - yaw)      IMU heading hold / tracking P
//   dv     = dv_ff + dv_fb + strim * v_cmd
//
//   left  = v_cmd - dv,  right = v_cmd + dv
//   accel refs: left = a_ref - a_ang,  right = a_ref + a_ang
//               (a_ang = alpha_ref * track / 2, the angular accel FF for ka)
//
// The IMU owns steering (fusion design): a turn is driven open-loop by the
// rate FF and trimmed closed-loop against gyro yaw, so it lands on the target
// angle even if track is slightly off. The maneuver finishes when BOTH
// profiles are done AND position/velocity/heading are inside the DONE
// tolerances, or on safety timeout 1 s after the later profile end
// (status.done = false -- something is stuck).
constexpr float MOVE_VCORR_MAX_MM_S = 100.0f;   // position-correction clamp
// 240 (was 120, 2026-07-23): at kyaw 15 the old clamp saturated at just
// 8 deg of error -- the fast heading loop went bang-bang exactly on the
// corner transients it was raised to kill. 240 keeps proportional
// authority to 16 deg; wheel budget holds (450 cruise + 240 dv + ff
// still inside the ~750 mm/s wheel envelope at sprint speeds).
constexpr float MOVE_DVCORR_MAX_MM_S = 240.0f;  // heading-correction clamp
// Position settle tolerance. History: widened 3 -> 12 mm (2026-07-08) when
// every move ended ~11 mm short on the maze floor, then diagnosed properly
// (2026-07-09, 'b' front-wall stop with duty logging): the "stall" was
// decel-tail ROLLBACK from a corrupted ka (~1.5e-4 in FRAM) -- the accel FF
// kept commanding reverse duty into a stationary mouse for the rest of the
// profile tail. With ka=0 moves settle within ~1 mm. 5 mm then proved too
// tight on the maze floor (2026-07-10 search run: a post-turn straight
// slid 5.8 mm through the stop -- v=0, yaw fine, position the only failing
// gate -- and kx*5.8 = ~12 mm/s of correction is under the stiction
// deadband, so it stood there to timeout). 8 mm then collided with the
// search's DELIBERATE stop trim (profiles aim 6 mm short so braking
// overshoot lands on the cell centre): every good stop reads -5..-8.5 mm
// against the trimmed reference, and 8.2 aborted a run one cell from the
// goal. 12 mm tolerates trim + spread; this gate's job is detecting STUCK
// moves -- physical stop accuracy is wall-verified (shuffle) regardless.
constexpr float MOVE_DONE_POS_MM = 12.0f;
constexpr float MOVE_DONE_VEL_MM_S = 20.0f;
// Pure-turn planner finish band (deg): inside this remaining angle the
// closed-loop turn commands zero rate and settles -- the sqrt(2*a*rem)
// curve steepens toward infinity at zero and would chatter against
// stiction through the stiff kp_turn. This band IS the turn's terminal
// accuracy (the absolute-cardinal straights absorb it).
constexpr float MOVE_TURN_FINISH_DEG = 1.0f;
// Heading settle tolerance. History: 2.0 aborted search straights (the
// servo legitimately bends the path); 4.0 was then grazed repeatedly
// (+3.16, +3.63, +4.93 on 2026-07-10) -- the P-only heading hold parks at
// its EQUILIBRIUM error under a standing disturbance, and the stop
// freezes whatever the rolling loop carried. A standing yaw error at a
// stop is harmless by architecture: every next straight opens with an
// ABSOLUTE turn to the cardinal, and wall sensing tolerates the tilt. 6.0
// sits above the observed equilibrium band; the real fix (an integral
// term in the heading hold) remains on the follow-up list.
constexpr float MOVE_DONE_YAW_DEG = 6.0f;        // heading settle tolerance
constexpr float MOVE_DONE_YAWRATE_DPS = 30.0f;   // rotation settle tolerance

// -- Lateral wall servo (search straights) -------------------------------------
// During forward STRAIGHTS the main loop feeds the corridor lateral offset
// (walls_evaluate, per packet) into a small clamped bias on the executor's
// yaw REFERENCE, steering the mouse back toward the corridor centreline.
// This is a smooth reference input, not an IMU step, so it does not violate
// the corrections-between-moves rule (which protects the heading loop from
// steps in the measured yaw). The bias:
//   - applies only while the angular profile is idle (straights -- never
//     turns or arcs, including their settle phase)
//   - is cleared automatically at every move start (each move begins
//     unbiased until wall data says otherwise)
//   - is clamped to LAT_SERVO_BIAS_MAX_DEG both here and in the setter
// Sign: lateral_mm is +ve toward the LEFT wall; returning to centre needs a
// CW (-yaw) steer, so  bias = -LAT_SERVO_DEG_PER_MM * lateral_mm.
// Gain intuition: 0.4 deg/mm nulls a small offset over ~150 mm of travel
// (atan(y/150) = 0.38 deg per mm); at 300 mm/s that is a ~0.5 s / ~140 mm
// first-order convergence -- gentle enough not to fight the heading hold.
// IMPORTANT: the 'f' wall-align fit assumes travel parallel to yaw0 -- the
// servo bends the path, so 'f' must run WITHOUT feeding the servo (it does:
// only callers that set the bias get it).
// Gain history: 0.4 / 3.0 measured ~4x slower than the naive intuition on
// hardware (2026-07-10 'l': +9.9 -> +5.3 mm over 360 mm) -- the P-only
// heading hold lags the bias (kyaw authority + low-duty deadband), and the
// 3 deg clamp pinned through the first stretch. 0.6 / 5.0 compensates;
// re-measure with 'l' if kyaw or the wheel loops change.
// 0.3 (was 0.6, 2026-07-24): the outer lateral loop's damping was
// secretly the INNER heading loop's sluggishness -- at kyaw 3 (tau
// ~205 ms) 0.6 deg/mm was stable; at kyaw 15 (tau ~40 ms) the inner
// loop obeys the bias almost immediately and the outer loop went
// underdamped at sprint speed (lat swinging +15 -> -28 out of the first
// corners, two wall strikes 2026-07-24). Halving the gain restores the
// margin; convergence stays FASTER than the old 0.6 because the inner
// loop no longer dawdles.
// 0.3 -> 0.15 (2026-08-12). The same instability came back, one speed step
// later. Log 080: the mouse started ~11 mm LEFT of centre and the servo drove
// it to ~47 mm RIGHT -- a four-fold overshoot, commanded heading swinging
// -7.9 to +8.0 deg, slide winding to +17, and the clearance guard cutting
// 650 -> 233 mm/s to save it. Identical in shape to the July event above, just
// at 650 mm/s instead of whatever the July runs were doing.
//
// Why speed brings it back: heading -> lateral position is a double
// integrator, so this loop has no damping of its own and lives entirely on
// phase margin. The gain sets a LENGTH constant (1/k in radians = 191 mm at
// k=0.3), which is fixed, but the TIME available to develop the correction
// shrinks as speed rises. At 650 mm/s that 191 mm is about one cell, against
// an observed loop period of ~700 mm of travel -- roughly a quarter wave, i.e.
// no margin at all.
//
// 0.15 puts the length constant at 382 mm and the quarter wave at ~600 mm,
// comparable to the loop period rather than a quarter of it. Convergence is
// slower in distance but the loop stops ringing, and the corner entry stops
// being taken mid-swing -- which is the actual cause of the "too wide" corners
// across 070/078/079/080, not the path generator.
constexpr float LAT_SERVO_DEG_PER_MM = 0.15f;   // the default of `latk` (ControlGains)
// The bias clamp, both servos: 10 (was 5 until 2026-09-07 -- at 5 a gain
// of 0.3 saturated at 17 mm of error, which is exactly the error the step
// test is meant to measure the response to; 10 deg at 1 m/s is a 170 mm/s
// lateral rate, well inside the wall clearance guard's reach).
constexpr float LAT_SERVO_BIAS_MAX_DEG = 10.0f;

// Set the lateral-servo yaw-reference bias (deg, +CCW), clamped. Main-loop
// safe. 0 clears; cleared automatically at move start and move end.
void control_set_lateral_bias_deg(float bias_deg);

// THE REFLEX CHANNEL: a second yaw-reference bias (deg, +CCW), clamped to
// FLINCH_MAX_DEG, that -- unlike the lateral servo's -- applies INSIDE
// ARCS as well as on straights. It is how the flinch opens an arc away
// from a post it is about to clip: the rate feedforward keeps flying the
// compiled arc, the heading loop tracks (path yaw + bias), and the mouse
// runs wider for exactly as long as the bias is held. The lateral servo's
// bias stays straights-only because it is a model (corridor centre) and
// the model is wrong in an arc; this one is a raw beam and a rate, valid
// anywhere. Main-loop safe; cleared at path/move start and end.
void control_set_reflex_bias_deg(float bias_deg);

// Rotate the path frame's yaw datum by d_deg (+CCW). The wall-frame
// alignment's lever (ControlGains::wall_align): the yaw reference for the
// rest of the path shifts by d_deg and the heading loop follows. No-op
// when no path is active. Main-loop safe.
void control_path_frame_adjust(float d_deg);

// Heading-hold INTEGRAL master switch (default on; see YAW_INT_* in
// Control.cpp). The integral learns a trim coefficient c and applies
// dv_hold = c * v, so heading is held even with kyaw = 0. Turn it off for
// anything that must MEASURE the raw left/right asymmetry rather than have
// it silently corrected -- the 'c' straight-line calibration, a bench
// contrast run, or test_move's no-heading-authority case. Disabling also
// zeroes the learned coefficient. Main-loop safe.
void control_yaw_integral_enable(bool enable);
bool control_yaw_integral();

// (A settled-yaw gate on the trim samples was tried and removed 2026-07-08:
// with a P-only heading hold the yaw settles AT its equilibrium error, never
// near zero, so the gate starved the measurement. See move_tick.)
constexpr float MOVE_TIMEOUT_EXTRA_S = 1.0f;

constexpr float DEG2RAD = 0.017453292519943295f;

// General maneuver: dist_mm forward (signed) and angle_deg rotation (signed,
// +CCW) run together. v/a are the linear cruise/accel (mm/s, mm/s^2);
// omega/alpha are the angular cruise/accel (deg/s, deg/s^2). Requires the
// velocity controller enabled and fusion ready; returns false otherwise.
// Captures the current yaw as the heading reference. Main-loop safe.
bool control_maneuver_start(float dist_mm, float v_mm_s, float a_mm_s2,
                            float angle_deg, float omega_dps, float alpha_dps2);

// Straight move (thin wrapper: zero angular profile). Same semantics as
// before this became a maneuver executor.
bool control_move_start(float dist_mm, float v_mm_s, float a_mm_s2);

// Straight move with an ABSOLUTE heading reference (deg, fusion frame)
// instead of capturing the current yaw. The heading loop then pulls onto
// the given cardinal WHILE ROLLING -- where it has full authority --
// instead of freezing whatever residual the previous in-place turn left.
// (Hardware 2026-07-10: ~4.5 deg of turn undershoot captured as yaw0
// consumed the lateral servo's entire clamp; the leg ran 17-19 mm
// off-centre with the servo pegged, unable to converge.) Maze straights
// use this with the cell cardinal; calibration moves keep the capturing
// variant.
bool control_move_start_heading(float dist_mm, float v_mm_s, float a_mm_s2,
                                float yaw_ref_deg);

// On-the-spot turn (thin wrapper: zero linear profile). angle_deg is signed,
// +CCW. Main-loop safe.
bool control_turn_start(float angle_deg, float omega_dps, float alpha_dps2);

// Smooth corner: a circular arc of radius_mm (>0) sweeping angle_deg (signed,
// +CCW = curve left). Couples the linear and angular profiles by the radius
// (v = R*omega, a = R*alpha, forward distance = R*|angle|), which makes the two
// TrapProfiles time-identical so the mouse centre traces a true circle. omega/
// alpha are the angular cruise/accel (deg/s, deg/s^2). Main-loop safe.
bool control_arc_start(float radius_mm, float angle_deg, float omega_dps,
                       float alpha_dps2);

// Stop commanding the maneuver (targets to zero; motors stay enabled).
// Also aborts an active speed path.
void control_move_abort();

// -- Speed-path executor (fast run) --------------------------------------------
// Executes a compiled SpeedPath (Path.h) as ONE continuous trajectory. Each
// tick the fused forward distance -- plus whatever the wall gates have
// corrected -- IS the path position s; the planner's references are
// recomputed from that s (Path.h: every brake is feedback, never a clock);
// and the wheel targets mix
//
//   v                             from the planner at s
//   dv = omega_ff*(track/2)       rate feedforward, SLEWED (see below)
//      + tasym*|omega|*(track/2)  turn asymmetry (Control.h VCTRL_TASYM)
//      + kyaw*yaw_err             heading hold against the path's yaw(s)
//      + yaw_int_c*v              learned trim
//      + strim*v                  feedforward trim
//
// Mutually exclusive with the move executor: whichever starts first refuses
// the other until it finishes. The SpeedPath must outlive the run (the
// caller owns it).
//
// ANGULAR SLEW RAIL. Constant-radius corners step the curvature at every
// tangent point, so omega_ff steps too -- ask the plant for 234 deg/s
// instantly and it will not deliver, it will enter late and run wide. The
// rail bounds how fast the COMMANDED rate may change, which converts an
// untrackable step into a bounded, repeatable entry transient. It is a
// mitigation, not a fix; the fix is a curvature-continuous reference
// (clothoids), and Path.h records the measurement that says when to go
// there. 4000 deg/s^2 reaches the R60 arc rate in ~59 ms / ~24 mm at
// 400 mm/s.
constexpr float PATH_OMEGA_SLEW_DPS2 = 4000.0f;
// How fast a heading-reference BIAS may move, deg/s; its rate is fed
// forward like the path's own omega (path_tick). The lateral servo's lane
// change ramped at 60 deg/s until 2026-09-10 -- a 5 deg step over 83 ms /
// ~70 mm at 850 mm/s -- because the wheel-speed loop needs the feedforward
// SUSTAINED to turn the body (a 150 deg/s ramp produced ~50 deg/s of yaw
// rate, D8 laps 87-90, at feedforward gain 1). With PATH_BIAS_FF_GAIN 2.5
// the 60 deg/s ramp is followed at 65-80 % (laps 96-99), and the lane-
// change step test (LANE_STEP.md, 2026-09-07) then showed the ramp, not
// the gain, setting the speed of a 20 mm lane change at 1016 mm/s: latk
// 0.3 and 0.45 gave the same 90 % point (218 / 228 mm), because a 9 deg
// command took 150 ms = 150 mm to arrive. 120 deg/s halves that; the
// step test is the check (expect the 20 mm done in ~150 mm, still no
// overshoot). The reflex keeps 150 deg/s, it is a twitch.
constexpr float PATH_LAT_BIAS_SLEW_DPS = 120.0f;
constexpr float PATH_REFLEX_SLEW_DPS = 150.0f;
// The wheel-speed loop delivers ~40 % of a short differential ramp (D8
// laps 87-95: 150 deg/s asked, 50-55 measured; 60 asked, 21-27 measured;
// the heading landed at 2.3 of 5 deg either way). Sustained arc
// feedforward catches up; a bias ramp does not last long enough to. So
// the bias ramp's feedforward is over-driven by this factor -- the
// reference still moves at the slew rate, the wheels are asked for the
// rate the plant will actually turn it into.
constexpr float PATH_BIAS_FF_GAIN = 2.5f;

// The rail is RUNTIME-SETTABLE because it is an experimental variable, not
// just a safety number. Exit error from a constant-radius corner has two
// causes -- grip (understeer, scales with v^2/R) and the curvature step
// (entry transient, scales with v^2/(R*alpha)) -- and they are COLLINEAR in
// the (R, v) plane, so no amount of sweeping radius and speed can separate
// them. Only alpha can: if exit error moves when the rail moves, the
// reference is the problem and clothoids will fix it; if it does not, grip
// is the problem and clothoids will not. That is the open question in
// ADR-002, and this setter is the instrument for it.
// Runtime only -- every boot returns to the compiled value.
void control_set_path_slew(float dps2);
float control_path_slew();

// Start the path. Requires the velocity controller enabled and fusion
// ready. Captures the current yaw as the path frame's datum, so the path's
// yaw(s) is followed RELATIVE to however the mouse is standing -- the same
// choice control_maneuver_start makes, and it means a launch heading error
// is a heading error rather than a whole rotated trajectory.
//
// yaw0_offset_deg SHIFTS that datum. The default 0 is the behaviour above:
// the frame is the body. A caller that knows where the CORRIDOR is -- from
// the walls, not from the gyro -- passes (corridor_yaw - current_yaw) and
// the path is then built on the corridor, with the body's residual
// misalignment appearing as a heading error the loop drains while the mouse
// accelerates.
//
// The distinction is not cosmetic. Launching on the body means a mouse
// standing 3 degrees off the corridor flies the whole first leg 3 degrees
// off it, and the lateral servo has to hold a permanent bias just to run
// parallel -- it never converges, it only trades angle for offset. Run 140's
// last lap did exactly that: yaw_err pinned at +1.4 to +2.3 for the entire
// leg while the beams diverged 15 mm to 24 mm. Launching on the corridor
// makes the same 3 degrees a transient that the heading loop removes in the
// first few centimetres, with the wheels rolling.
bool control_path_start(const SpeedPath *path, float yaw0_offset_deg = 0.0f);

// Inject an along-track position correction from a wall gate (mm, signed;
// positive = the mouse is FARTHER along the path than odometry believed).
// Clamped to +/-PATH_CORR_MAX_MM per call -- anything larger is a
// misdetection, not a measurement -- and then DRAINED into the live s at
// PATH_CORR_DRAIN_MM per tick rather than applied as a step.
//
// Why drained: s feeds the braking curves, so a step in s is a step in the
// commanded speed and (through omega = v*kappa) in the commanded rate. V1
// learned this the expensive way -- a raw -53 mm edge "correction" jolted an
// approach into a +12 mm entry transient and a crash (2026-07-26).
//
// THE DRAIN IS A DISTANCE, NOT A TIME. Each tick it retires
// PATH_CORR_DRAIN_FRAC of the distance just travelled, so a correction is
// spread over a fixed LENGTH of run (25 mm over 100 mm at 0.25) no matter
// how fast the mouse is going. A fixed mm-per-tick rate is a fixed
// mm-per-SECOND rate, and the run out of a gate to the corner it anchors is
// a fixed DISTANCE -- so a time-based drain quietly stops fitting as the
// speed ladder is walked. At a gate 102 mm before the arc, 0.25 mm/tick
// covers a 25 mm correction at 450 mm/s and spills into the arc at 550.
// This is the same lesson the lateral servo taught (V2_BASELINE.md,
// finding 6): the loop lives on a length constant, and speed is what eats
// the margin.
//
// The floor keeps a correction finishing when the mouse is barely moving,
// where a purely proportional drain would stall.
constexpr float PATH_CORR_MAX_MM = 40.0f;
// Bound on the slip model's contribution between two anchors. Sized for a
// launch straight at sacc ~50 (roughly 40 mm) with room to spare; a value
// pinned here means the coefficients are wrong, not the mouse. The
// IMU-based slip GATE in the motion estimator (mot_d_off, the flight log's
// `slide` column) is a different instrument: it catches gross wheel-spin
// events, not the few-percent systematic over-read this term models.
constexpr float PATH_SLIP_MAX_MM = 80.0f;
constexpr float PATH_CORR_DRAIN_FRAC = 0.25f;      // of distance travelled
constexpr float PATH_CORR_DRAIN_FLOOR_MM = 0.02f;  // per tick, minimum
//
// over_mm: spread this correction over exactly this much travel instead of
// the default fraction. The caller knows something this module cannot --
// how much run is left before the correction stops being useful. The edge
// gate's room shrinks as the radius grows (the wall end is fixed at the
// cell boundary while the tangent point moves back with R: 117 mm of
// approach at R45, only 37 mm at R125), so a fixed fraction would quietly
// drain a large-radius correction INTO the arc it was meant to set up.
// Pass the room available and it always lands in time. <= 0 keeps the
// default.
void control_path_correct(float ds_mm, float over_mm = -1.0f);

// THE ROLLING PATH (Explore.h, 2026-09-12). A search extends its SpeedPath
// while the executor is flying it -- SpeedPath::add_straight / add_arc
// complete a segment before the count that exposes it moves, and sample()
// re-solves from s every tick, so the extension simply lifts the v = 0
// constraint the old end imposed. Two things the executor must be told:
//
// control_path_extended(): recompute the stall timeout from the path's new
// length (it was sized once at launch, for the length at launch).
//
// control_path_swap(next, s_in_next): continue on ANOTHER SpeedPath object
// that coincides with the current one from the mouse's position onward --
// the search rebuilds its path from the current cell once the old one holds
// too many segments (PATH_MAX_SEGS / PATH_MAX_CORNERS are finite). s_in_next
// is where the mouse is on the new path right now; corrections, slip, the
// frame and the biases carry over untouched. Main-loop safe; false if no
// path is active.
void control_path_extended();
bool control_path_swap(const SpeedPath *next, float s_in_next_mm);

struct ControlPathStatus {
    bool active;
    bool done;         // valid once !active: reached the end and settled
    float s_mm;        // path position (fused distance + gate corrections)
    float v_mm_s;      // commanded forward velocity
    float omega_dps;   // commanded angular rate, post-slew
    float yaw_ref_deg; // path yaw(s) + launch datum
    float yaw_err_deg;
    bool in_arc;
    float corr_mm;     // gate corrections applied so far (drained)
    float corr_pending_mm;  // queued, not yet drained
    float slip_mm;     // slip-model contribution since the last anchor
    float frame_deg;   // wall-frame alignment applied so far this path
    bool front_gov;    // the front governor capped v this tick (see below)
    // The APPLIED heading bias (lateral + reflex, after the slew), deg, +
    // left: yaw - (path line) = bias_deg - yaw_err_deg. The V exit's dead
    // reckoning integrates exactly that (DiagLane.h).
    float bias_deg;
};
ControlPathStatus control_path_status();  // main-loop safe

// THE FRONT GOVERNOR (bert, 2026-09-26, after the search drove into the
// goal's front wall at 300 mm/s with the wall in plain view of both front
// beams from 180 mm out: "the mouse should have a low level override to
// stop it actually running into a wall directly in front of it, it should
// never get to within 60 mm from mouse centre even in a search alcove").
//
// A reflex UNDER every mission, in the executor itself: whatever the path
// or the move asks, the commanded forward speed is capped at
//     v <= sqrt(2 x FRONT_GOV_DECEL x (d - FRONT_GOV_FLOOR_MM))
// where d is the front distance both beams agree on. The cap is a braking
// curve that lands at rest FLOOR mm from the wall (contact is 45), from
// whatever speed the wall was first seen at within the beams' 200 mm --
// 1500 mm/s at 200, 1060 at 130, 800 at 100, 620 at 84 (a search alcove:
// the dead-end stop at the cell centre reads 84 and is never touched),
// 380 at 69, 0 at 60. Nothing a correct path does comes near it: a route
// corner starts with the wall ~168 mm ahead at <= 900 mm/s (cap 1300); the
// search's own guards (Explore.h) act at 130 and 70 before it does.
// A wrong path -- the map's front wall 31 mm out of place, a cell
// miscounted, a lost mouse -- meets the cap and stops 15 mm from the nose
// instead of hitting; the mission then times out ("path stalled") and
// reports the governor held it.
//
// WHAT COUNTS AS A WALL: both front beams inside FRONT_GOV_RANGE_MM and
// within FRONT_GOV_AGREE_MM of each other, packet fresher than
// FRONT_GOV_STALE_MS; d is the nearer. One beam alone is a post clipped
// (the diagonal lane's posts alternate sides and the beams cross at
// ~100 mm, so a post is never in both at once); two beams that disagree
// by more are an oblique edge, not a face. The nearer of an agreeing pair
// is the distance that matters. Reverse motion is not capped.
//
// FRONT_GOV_DECEL is the curve's assumption, not a promise the tyre makes:
// the longitudinal knee measured ~7000 on the big floor; at 8000 the cap
// sits above every legitimate speed, and if the tyre only gives 6000 the
// mouse stops at ~50 -- still short of contact. The governor is fed from
// the main loop (main.cpp hands it the latest packet's two front beams and
// its age, after the missions have polled the sensors) and applied in the
// 1 kHz tick. It knows nothing of missions: it is under all of them.
constexpr float FRONT_GOV_FLOOR_MM = 60.0f;
constexpr float FRONT_GOV_DECEL_MM_S2 = 8000.0f;
constexpr float FRONT_GOV_RANGE_MM = 200.0f;
constexpr float FRONT_GOV_AGREE_MM = 40.0f;
constexpr uint32_t FRONT_GOV_STALE_MS = 50;
// Main loop, every iteration: the latest packet's front beams (mm forward
// from the centre, 255 = nothing) and how old the packet is.
void control_front_guard_feed(uint8_t front_left_mm, uint8_t front_right_mm, uint32_t age_ms);
void control_front_guard_enable(bool on); // default on; bench harnesses may lift it
struct ControlFrontGuard {
    bool enabled;
    bool wall;          // both beams agree on a face inside range
    float d_mm;         // that distance (nearer beam), or 0 when !wall
    bool capping;       // the cap bound the commanded speed this tick
    uint32_t cap_ticks; // ticks capped since the last path/move start
};
ControlFrontGuard control_front_guard();  // main-loop safe

// Fold the slip model's running contribution into the drained bias (s is
// unchanged by this) and return what was folded. Called by the route
// runner at every anchor: the segment's slip becomes permanent and the
// next segment's model starts from zero, so the term stays bounded and the
// lap report can record it per corner.
float control_path_slip_take();
// Arm / disarm the slip model. A path starts DISARMED: the launch straight
// is a pure-longitudinal pull from rest and the 2026-09-05 ralon ladder
// showed it does not slip (corr ~0 at 1192..1432 mm/s gained) while the
// mid-run segments after a U or a 90 do (47 mm per m/s) -- the tyre is
// still carrying cornering load when the torque comes on. The route
// runner arms it at the first anchor of the lap.
void control_path_slip_enable(bool on);

// Extend the ACTIVE pure straight by extra_mm (same direction) without
// stopping: the linear profile's cruise is stretched (TrapProfile::extend)
// so the mouse rolls on. False -- and no change -- if no straight is
// active, a rotation is involved, or the decel has already begun (the stop
// is then committed; plan the next leg after it). Main-loop safe.
bool control_move_extend(float extra_mm);

struct ControlMoveStatus {
    bool active;
    bool done;          // finished inside tolerance (valid once !active)
    float v_ref;        // linear profile references at the last tick
    float a_ref;
    float d_ref;
    float v_cmd;        // v_ref + position correction (centre-line command)
    float d_mm;         // fused distance travelled since move start
    float omega_ref_dps; // angular profile rate reference
    float yaw_ref_deg;   // yaw0 + angular d_ref (+ lateral bias on straights)
    float yaw_err_deg;   // wrap(yaw_ref - yaw)
    float yaw_rate_dps;  // measured gyro yaw rate (for turn done + TrackTuner)
    float lat_bias_deg;  // lateral-servo yaw-reference bias in effect
};
ControlMoveStatus control_move_status();  // main-loop safe

// Heading-hold effort measured during the cruise phase of the last move
// (accumulators reset by control_move_start). avg_dv is the CLOSED-LOOP
// differential only (the strim feedforward is excluded), so the trim
// update law is simply:  strim += avg_dv / avg_v.  valid is false if the
// move had no cruise samples (too short / never reached cruise).
struct ControlMoveTrim {
    bool valid;
    float avg_dv_mm_s;   // mean heading-hold differential during cruise
    float avg_v_mm_s;    // mean centre-line command during cruise (signed)
};
ControlMoveTrim control_move_trim();  // main-loop safe

// -- Fused forward motion (encoder + IMU, slip-gated) --------------------------
// Complementary filter on forward velocity, run every control tick:
//
//   predict:  v_est += a_fwd * dt        a_fwd = tilt-compensated body-X
//                                        accel (batch-averaged FIFO samples,
//                                        gravity removed via the Madgwick
//                                        attitude), low-pass filtered
//   correct:  v_est += K * (v_enc - v_est)   encoders: drift-free when
//                                             gripping; K small
//
// Slip gate: wheel acceleration (d/dt of encoder velocity) is compared to
// a_fwd. Wheels accelerating while the body doesn't (or vice versa) means
// slip -- the encoder correction is disabled (K = 0, coast on the IMU,
// where it is trustworthy for hundreds of ms) until encoder velocity
// re-converges with the estimate.
//
// Thresholds below are provisional: run the 'a' alignment test (main.cpp,
// mouse ON THE FLOOR with ~1 m of clear runway) to measure the real
// residual noise floor and set MOTION_SLIP_ACC_MM_S2 with margin.
constexpr float MOTION_K = 0.05f;                 // encoder gain per tick
constexpr float MOTION_ACC_FILTER_ALPHA = 0.3f;   // LP on a_fwd
// Wheel acceleration is differenced over this many ticks (32 ms), not
// tick-to-tick: one encoder count per tick is ~100 mm/s of instantaneous
// velocity quantisation, which differentiated at 500 Hz would swamp the
// slip gate. The window divides that noise by 16 for 16 ms of extra lag.
constexpr int MOTION_WHEEL_ACC_TICKS = 16;
// Gate history: 1500 false-tripped exactly 4x per 'c' leg on the maze floor
// (2026-07-08, 800 mm/s^2 ramps): the wheel-acc window (32 ms) vs a_fwd
// filter (~6 ms) lag mismatch spikes the residual at each profile corner,
// and each trip coasts fused d ~10 mm short -> position settle deadlock.
// Real wheelspin is motor-rate (>>2500); corner transients are not.
// 2026-07-09 postscript: those corner trips happened while a corrupted ka
// (~1.5e-4, FRAM) was slamming duty at every profile corner -- they were
// plausibly REAL micro-skids, not filter-lag artifacts. With ka=0 the
// corners are gentle; the gate could likely come back down (better slip
// sensitivity) -- revisit with a duty-logged run before relying on it.
constexpr float MOTION_SLIP_ACC_MM_S2 = 2500.0f;  // residual gate
constexpr int MOTION_SLIP_ON_TICKS = 3;           // gate must persist (6 ms)
constexpr float MOTION_RECOVER_MM_S = 30.0f;      // re-engage band
constexpr int MOTION_RECOVER_TICKS = 25;          // 50 ms inside the band
constexpr float GRAVITY_MM_S2 = 9806.65f;

// Snapshot of the estimator (main-loop safe). Values are zero until
// fusion_ready() (the attitude is needed for gravity removal).
struct ControlMotion {
    float v_est_mm_s;       // fused forward velocity
    float d_est_mm;         // integral of v_est since zero
    float v_imu_mm_s;       // PURE accel integral since zero -- drifts;
                            // exposed for the 'a' test / drift measurement
    float a_fwd_mm_s2;      // forward accel, tilt-compensated, filtered
    // LATERAL specific force, body +Y, same tilt compensation and filter as
    // a_fwd. In a turn the tyres must supply a = v * omega to hold the arc;
    // the accelerometer reads what they ACTUALLY supplied. The difference is
    // the one direct measurement of a broadside slide there is on this
    // mouse -- encoders and gyro are both blind to it, because a pure
    // lateral skid changes neither wheel angle nor heading (bert saw the
    // mouse move out on turns, 2026-08-20; runs 116-118 could not see it).
    float a_lat_mm_s2;
    float wheel_acc_mm_s2;  // d/dt of encoder centre velocity, filtered
    bool slip;              // encoder correction currently gated off
    uint32_t slip_events;   // times the gate has tripped since zero
    // THE ABS (ControlGains::abs_band): interventions since zero -- a
    // count of loop ticks on which a wheel was overridden to body speed
    // (left + right), and the number of distinct episodes.
    uint32_t abs_ticks;
    uint32_t abs_events;
    bool abs_active;        // either wheel clamped this tick
    float v_abs_mm_s;       // the ABS's own body velocity (IMU-integrated,
                            // encoder-anchored only while gripping)
};
ControlMotion control_motion();

// Zero v_est/d_est/v_imu/slip_events (start of a run or test).
void control_motion_zero();
