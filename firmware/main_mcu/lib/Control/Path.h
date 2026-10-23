// Speed-run trajectory: straights and CONSTANT-RADIUS arcs on one
// arclength coordinate s, with a per-tick velocity planner.
//
// V2, 2026-08-18. This is the geometry the mouse flew for its 5/5 clean
// runs (TW3_V1, 2026-08-01), rebuilt on its own without the clothoid /
// dense-diagonal / elastic-band / reshape work that came after. Read
// docs/adr/ADR-002-fast-run.md before changing the shape of this file.
//
// WHAT IS AND IS NOT HERE
// -----------------------
// A path is a list of SEGMENTS, each with constant curvature: kappa == 0 is
// a straight, kappa != 0 is a circular arc of radius 1/|kappa|, signed +CCW
// (a left turn). Nothing else. No clothoids, no diagonals, no splicing.
// Curvature is therefore a STEP at every tangent point, and the executor
// rounds that step with an angular-rate slew rail (Control.h,
// PATH_OMEGA_SLEW_DPS2) rather than the reference being trackable to begin
// with. That is a known, measured limitation and the reason V1 eventually
// moved to clothoids -- see the "WHEN TO REPLACE THIS" note at the bottom.
//
// EVERY BRAKE IS FEEDBACK, NEVER A CLOCK
// --------------------------------------
// sample() takes the MEASURED path position s and recomputes the whole
// velocity reference from it every tick. It does not integrate a plan.
// That is what makes the wall gates work at all: a gate corrects s, and the
// next tick's braking curves simply re-solve against the corrected value --
// there is no schedule to re-synchronise and no discontinuity to smooth.
// It is also why a slipped wheel or a bad corner costs distance rather than
// desynchronising the run.
//
// SPEED CONSTRAINTS, all evaluated per tick against measured s:
//   - pointwise:     v <= sqrt(a_lat / |kappa(s)|)
//   - every upcoming arc is a braking target at its exact entry point:
//                    v <= sqrt(v_arc^2 + 2*a_lon*d)   (d = distance to it)
//   - the path end is a v = 0 constraint
//   - accel ramp:    v <= v_prev + a_lon*dt
// The minimum wins. Short straights therefore self-limit -- v_max is a
// CEILING that is only reached where there is room for it.
//
// PATH-s VERSUS SPATIAL DISTANCE -- THE ANCHOR TRAP
// -------------------------------------------------
// Arclength through a corner is SHORTER than the corridor room the corner
// consumes: a 90-degree arc of radius R has path length pi*R/2 (94 mm at
// R60) but eats R (60 mm) off each of the two corridors it joins, i.e.
// 120 mm of spatial run. Anything that mixes the two acquires a systematic
// error of about 0.2*R -- in V1 the finish gate printed +0 while the corner
// gates printed +25, for exactly this reason.
//
// So: corner_entry_s_mm(k) is a PATH-s value, and corner_tangent_mm(k) is
// the SPATIAL room the corner takes out of each corridor. To anchor a
// landmark that sits X mm (spatial) before the corner point, with X > T:
//
//     path s = corner_entry_s_mm(k) - (X - T)
//
// On the approach straight ahead of a corner the two agree, which is why
// the inside-wall edge gate (Search/SpeedRun) can use plain distances --
// but only there. Do not extend that habit past the tangent point.
//
// Pure math -- no HAL, no Arduino, no floats-in-ISR concerns beyond cost.
// Host-tested in test/test_path.cpp.

#pragma once

#include <stdint.h>

// A CLOTHOID corner is 3 segments (ramp in, arc, ramp out) plus the
// straights between corners, and straights merge: an n-corner route is at
// most 3n + n + 1 segments.
// THE PLANNER CANNOT REFUSE A MAZE (2026-10-02, the eve of the full-size
// test day; bert: "the planner can't refuse a maze"). 24 corners was "far
// longer than a 16x16 needs" for a diagonal-merged out leg, but the home
// leg is orthogonal -- every staircase step a corner -- and 300 random
// 16x16 mazes refused 26 % of out legs and 19 % of home legs ("route:
// cannot compile this leg": no flight, or a mouse stranded at the goal).
// Now the limits are the maze's own: a walk of at most 256 cells has at
// most 255 grid turns, merged corners are never more than the grid turns,
// so 256 corners and 4 x 256 + 1 segments hold ANY route a 16x16 can
// have. ~36 KB per SpeedPath -- so every SpeedPath instance lives in RAM2
// (PATH_STORAGE: OCRAM, 511 KB free; cached, constructors still run), and
// the per-tick walks over the segments are bounded (seg_at is a binary
// search; sample() stops at the first arc too far ahead to bind).
constexpr int PATH_MAX_CORNERS = 256;
constexpr int PATH_MAX_SEGS = 4 * PATH_MAX_CORNERS + 1;
#if defined(__IMXRT1062__)
#define PATH_STORAGE __attribute__((section(".dmabuffers"), used))
#else
#define PATH_STORAGE
#endif

// Radius the bench harness and the first route runs use. 60 mm was V1's
// proven corner-harness default: it fits a 180 mm cell with 30 mm of
// straight either side of the tangent points, and at a_lat 1000 it carries
// sqrt(1000*60) = 245 mm/s through the arc.
constexpr float PATH_RADIUS_DEFAULT_MM = 60.0f;
// A corner needs R of corridor either side of the corner point (the tangent
// length of a 90-degree arc), plus a little straight for the executor to
// settle the entry transient in. Below this the corner does not fit and the
// caller must fall back to a stop-and-turn.
constexpr float PATH_CORNER_MARGIN_MM = 20.0f;

// -- Clothoid transitions -----------------------------------------------------
// OFF BY DEFAULT (length 0 = pure arcs), so a path built without asking for
// them is bit-identical to the constant-radius geometry that flew runs
// 116-135. Turn them on per path with set_clothoid_len().
//
// WHY THEY EXIST, measured rather than assumed. A constant-radius corner
// hands the executor a curvature STEP at each tangent point. The rail
// (Control.h) bounds how fast the commanded rate may change, so the mouse
// keeps rotating past the exit tangent by roughly omega^2/(2*alpha). Runs
// 130-135, 250+ corners, measured that overshoot straight-to-straight:
//
//     cell                    measured   omega^2/2alpha
//     R125 v500 slew2500        5.33          10.50
//     R100 v500 slew2500        7.61          16.41
//     R80  v500 slew6000        6.18          10.69
//     R60  v500 slew6000       13.69          19.00
//
// consistently ~60 % of the open-loop form (the yaw loop eats the rest) and
// the same shape everywhere. ADR-002 wrote the decision rule in advance --
// error moves with the rail -> clothoids; error does not -> grip -- and the
// rail A/B was decisive: halving the ramp roughly halved the overshoot
// (R80/v500: 11.86 at slew 2500 against 6.18 at 6000).
//
// Two other candidates were tested and rejected on data, which is why this
// is the remaining explanation rather than the first guess:
//   - grip: |ay|/a_dem never saturates, and reads 1.085 at the very cell
//     that crashed. It also differs by TURN DIRECTION at identical demand,
//     which grip cannot do.
//   - feedforward magnitude: track_mm x0.90 (runs 134 and 135, a clean
//     repeat) moved the overshoot by +0.31 deg -- i.e. slightly the WRONG
//     way. kyaw simply took up the slack; the split between feedforward and
//     feedback does not set the trajectory, the REFERENCE does.
//
// SIZING. The useful clothoid is the SHORTEST one the rail can already
// track: L = v^2 / (R * alpha). Below that the ramp is still rate-limited
// and nothing is gained; above it the corner eats corridor it does not need
// to. Note this does not ask the executor for anything new -- the rail was
// always going to take that long. What changes is that the GEOMETRY now
// accounts for the ramp, so the mouse's position follows the reference
// instead of drifting off it.
//
// THE BUDGET, and it bites hardest where clothoids are least needed. A
// corner consumes T of corridor either side (below), and the inside-wall
// landmark sits only 90 + beam_lead mm before the corner point -- 135 mm
// with the measured 45 mm lead. The gate must fire BEFORE the tangent
// point, so T <= ~135. A pure arc already has T = R, which leaves
// 135 - R of room: 10 mm at R125, 55 at R80, 75 at R60. So the clothoid
// budget GROWS as the radius tightens, which is exactly where the overshoot
// is worst. At R125 there is no room -- and no need, the overshoot there is
// 0.84 deg at slew 6000.
constexpr float PATH_CLOTHOID_MAX_MM = 90.0f;
// A U-TURN GETS ITS OWN CAP, and a much larger one, because the budget
// argument above does not apply to it. That 90 comes from a 90-degree
// corner's corridor budget: T <= 135 mm, and T grows with L. A U-turn has no
// corner point and no tangent -- its "T" is a SETBACK the route compiler
// chooses, and its clothoid runs across the cell rather than along the
// approach. What bounds it instead is the geometry itself: the pair may not
// sweep more than 180 degrees or no arc survives, i.e. L < pi*R.
//
// It has to be larger, not merely may be. A tighter U-turn buys its lost
// offset back from the clothoid's own lateral shift, and at R 71.7 that needs
// L around 185 mm. Leaving it on the 90 mm cap is what made every solve below
// ~5000 dps^2 fail.
constexpr float PATH_UTURN_CLOTHOID_MAX_MM = 220.0f;
// The fastest angular-rate ramp a solved U-turn geometry may ASSUME. A
// clothoid is a promise that the commanded rate rises at v^2/(R*L); if the
// executor's rail is slower it clips that ramp and the mouse flies a shape
// nobody solved.
//
// This is NOT bounded by Control.h's PATH_OMEGA_SLEW_DPS2. That constant is
// the executor's DEFAULT rail, not a limit -- control_set_path_slew accepts up
// to 20000, and a mission is expected to install the rail its path was
// compiled with. RouteRun does exactly that (and static_asserts that its own
// the rslew param is at least this). Until 2026-08-27 it did not, which is
// how every route corner came to be flown with its clothoid ramp clipped.
constexpr float PATH_UTURN_MAX_RAIL_DPS2 = 6000.0f;
// A clothoid pair cannot sweep more than the corner does. Cap the pair at
// this fraction of the total turn so a central arc always survives.
constexpr float PATH_CLOTHOID_MAX_SWEEP_FRAC = 0.80f;

// Compiled-in speed limits. Deliberately BELOW what V1 finished at
// (650 / 1000 / 2000): this geometry is being re-proven from scratch on the
// floor, and the speed ladder is walked by hand from a value that is known
// to have flown, not from the last value V1 reached with a different
// trajectory generator underneath it.
constexpr float PATH_V_MAX_DEFAULT = 400.0f;    // mm/s
constexpr float PATH_A_LON_DEFAULT = 700.0f;    // mm/s^2 (the proven search accel)
constexpr float PATH_A_LAT_DEFAULT = 1000.0f;   // mm/s^2

struct PathPose {
    float x_mm;
    float y_mm;
    float yaw_deg;  // +CCW, path frame (start pose is the datum)
};

struct PathSeg {
    float s0_mm;     // arclength at the segment start
    float len_mm;
    // Curvature at the segment START and END, 1/mm, signed +CCW. Curvature
    // varies LINEARLY between them, which makes one struct cover all three
    // primitives this path needs:
    //     kappa == kappa1 == 0     straight
    //     kappa == kappa1 != 0     circular arc
    //     kappa != kappa1          clothoid (linear curvature ramp)
    // A clothoid is what removes the curvature STEP at a tangent point --
    // see the clothoid note further down.
    float kappa;
    float kappa1;
    float x0_mm;     // pose at the segment start, precomputed so pose(s)
    float y0_mm;     // is O(1) after the segment lookup
    float yaw0_rad;
    // Speed ceiling on this segment, mm/s; 0 = none beyond the lateral cap.
    // Set by the route compiler on a corner it had to SLOW to fit (Route.h,
    // the speed rung): the planner brakes to it at the corner's entry the
    // same way it brakes to sqrt(a_lat / kappa), so a slowed corner is just
    // a tighter braking target, not a new mechanism.
    float vcap_mm_s;
};

class SpeedPath {
   public:
    SpeedPath() { clear(0.0f, 0.0f, 0.0f); }

    // Drop everything and set the start pose (path frame). yaw is degrees,
    // +CCW; the executor adds the world yaw offset at launch.
    void clear(float x0_mm, float y0_mm, float yaw0_deg);

    // Append a straight. Consecutive straights are merged -- a route
    // compiler can emit one per cell without inflating the segment count.
    bool add_straight(float len_mm);

    // Append a circular arc. radius_mm > 0; angle_deg is signed, +CCW =
    // curve LEFT. Registers a corner for the gate metadata below.
    bool add_arc(float radius_mm, float angle_deg);

    // Bench harness: lead straight, ONE corner, exit straight. Lengths are
    // the straight runs BETWEEN tangent points, so the spatial corridor run
    // either side of the corner point is lead + R and R + exit.
    bool begin_corner(float lead_mm, float radius_mm, float angle_deg,
                      float exit_mm);

    // CLOSED RING: four 90-degree corners of the same radius joined by four
    // sides of side_mm (corner point to corner point). The mouse starts on a
    // side, first_leg_mm before the first corner point, and the path ends
    // back at the start pose -- so a loop runner can fly it repeatedly.
    //
    // left_turns selects the sense: false = all right turns = clockwise
    // round a ring whose inside is on the mouse's right.
    //
    // The path CLOSES exactly (4*side - 8R of straight, plus four quarter
    // arcs); test_path asserts it, because a ring that does not close would
    // walk the start pose a little further round the maze every loop and
    // look exactly like drift.
    bool begin_ring(float first_leg_mm, float side_mm, float radius_mm,
                    bool left_turns);

    // ---- U-TURN ------------------------------------------------------------
    //
    // A 180 in one move, for the hairpin: two same-direction 90s in adjacent
    // cells are a U-turn that a corner-point builder has to chop in half.
    //
    // WHY IT NEEDS ITS OWN PRIMITIVE. Everything else here is placed by a
    // CORNER POINT with a tangent T either side, and T = R*tan(phi/2) --
    // which is infinite at phi = 180. A U-turn has no corner point. What it
    // has instead is a LATERAL OFFSET, and in a maze that offset is not a
    // choice: the exit corridor is one cell from the entry corridor, so the
    // offset is exactly 180 mm and the radius follows from it.
    //
    //     offset = 2 * (R + p),   p = the clothoid shift
    //
    // At v500 on the 6000 deg/s^2 rail that solves to R = 89.7 mm with 26.6 mm
    // clothoids sweeping 8.5 degrees each, leaving 163 degrees of constant
    // radius. Peak rate 319 deg/s -- LOWER than an R84 corner (341) -- against
    // 598 for the R60+R45 pair this replaces, and 392 for R73+R73.
    //
    // Measured on hardware before this existed: runs 156-163 flew the pair at
    // R60+R45 and the hairpin reached 23.9 deg of heading error against a 25
    // deg abort; runs 164-167 flew R73+R73 and it fell to 10.1, within 0.3 of
    // the predicted 9.8. The mechanism is rate-tracking lag, so the radius is
    // the whole lever -- which is what makes the 180 worth its own primitive.
    struct UTurn {
        float radius_mm;
        float clothoid_mm;
        // How far along the ENTRY corridor the path reaches before it comes
        // back. This is what decides whether the turn fits the end wall, and
        // it is the caller's job to place the entry accordingly.
        float extent_mm;
        float length_mm;    // arclength of the whole primitive
    };
    // Solve the radius whose 180 offsets exactly offset_mm at this (v, rail).
    // False if no radius in a sane range produces that offset.
    static bool uturn_solve(float offset_mm, float v_mm_s, float slew_dps2,
                            UTurn *out);

    // THE OTHER WAY ROUND: fix the RADIUS and solve the clothoid.
    //
    // uturn_solve above takes a rail and finds the radius. That is the right
    // shape when the rail is the thing you care about, but it cannot be asked
    // for a specific radius -- and it cannot reach a small one at all. The
    // offset a U-turn achieves is 2*(R + p) with p the clothoid's own lateral
    // shift; p grows as the clothoid lengthens, so a smaller R needs a longer
    // clothoid, which needs a gentler rail. Chasing that by lowering the rail
    // runs into a wall: offset(R) is U-SHAPED at a fixed rail, uturn_solve
    // bisects it as if it were monotonic, and below about 5000 dps^2 the
    // bracket straddles the minimum and the solve simply fails.
    //
    // Fixing R instead makes the problem one-dimensional AND monotonic: p
    // increases with L, so there is exactly one L that hits the offset, and
    // bisection cannot be fooled. The rail is then an OUTPUT -- v^2/(R*L),
    // reported rather than chosen -- and the caller's job is to check it is
    // inside PATH_OMEGA_SLEW_DPS2.
    //
    // The cost is that a tighter U-turn is nearly all clothoid: at R 71.7 the
    // arc that survives is around 40 mm of a 400 mm primitive. That is not a
    // defect. A long clothoid ramps the rate gently where the old geometry
    // stepped it, which is the opposite of the thing that has been hurting.
    //
    // False if no clothoid shorter than PATH_CLOTHOID_MAX_MM reaches the
    // offset, or if the required one would sweep more than 90 degrees per end
    // (the arc would have to be negative).
    static bool uturn_solve_radius(float offset_mm, float radius_mm,
                                   float v_mm_s, UTurn *out,
                                   float *slew_out = nullptr);
    // Append the solved U-turn. sign selects the sense as elsewhere: +1 turns
    // LEFT (+CCW), -1 turns RIGHT.
    //
    // entry_setback_mm is what corner_tangent_mm() will report for this
    // corner. It is NOT a geometric tangent -- there is no corner point --
    // it is the distance from the primitive's entry back to the cell centre
    // the turn is named by, and it is passed in because only the route
    // compiler knows where that centre is. Reporting it here means the wall
    // gate's arithmetic (entry_s + tangent - half a cell - beam lead) lands
    // on the same post for a U-turn as for a 90, with no special case.
    bool add_uturn(const UTurn &u, float sign, float entry_setback_mm);

    // Clothoid transition length, mm, applied to every arc added AFTER this
    // call. 0 (the default) = pure constant-radius arcs, bit-identical to
    // the pre-clothoid geometry. Clamped to PATH_CLOTHOID_MAX_MM and to
    // whatever the corner's own sweep can accommodate.
    void set_clothoid_len(float mm) {
        clothoid_len_ = (mm > 0.0f) ? mm : 0.0f;
        if (clothoid_len_ > PATH_CLOTHOID_MAX_MM)
            clothoid_len_ = PATH_CLOTHOID_MAX_MM;
    }
    float clothoid_len() const { return clothoid_len_; }

    // Corridor room a clothoid-arc-clothoid corner consumes EITHER SIDE of
    // the corner point, without building anything. The caller needs this to
    // size L against the landmark budget before committing to a path --
    // see the clothoid note above. L = 0 returns the pure-arc R*tan(phi/2).
    static float tangent_for(float radius_mm, float clothoid_mm,
                             float sweep_deg);

    // Shortest clothoid the rail can already track at this (R, v): below it
    // the ramp stays rate-limited and the geometry gains nothing.
    //   L = v^2 / (R * alpha),  alpha in deg/s^2 as the rail is specified.
    static float clothoid_for_rail(float radius_mm, float v_mm_s,
                                   float slew_dps2);

    // Longest clothoid whose corner still fits a corridor budget: the
    // largest L with tangent_for(R, L, sweep) <= t_max_mm. Returns 0 when
    // even the pure arc does not fit, which is the honest answer -- a
    // corner that already eats more corridor than the landmark allows
    // cannot be given a transition as well.
    static float clothoid_for_room(float radius_mm, float t_max_mm,
                                   float sweep_deg);

    // Speed ceiling stamped on every segment pushed AFTER this call until it
    // is cleared (0). A corner builder brackets its add_arc / add_uturn with
    // it; straights never carry one -- the planner's braking curve into the
    // capped corner is what slows the approach.
    void set_next_vcap(float v_mm_s) { next_vcap_ = (v_mm_s > 0.0f) ? v_mm_s : 0.0f; }

    // a_brk: the deceleration the braking curves (into corners and the path
    // end) are solved with. 0 = same as a_lon, as it always was. Separate
    // since 2026-09-25: a launch has no lateral load and takes 5000+, but
    // the brake INTO a corner overlaps the clothoid where the lateral is
    // building -- at ralon 5000 the 45-in read ax -8400 with the arc
    // already on, the tyre let go (slip gate open), the yaw lagged 6.6 deg
    // then overshot, and the V that followed was entered 13 deg behind
    // and hit the post. ralon 4000 flew the whole ladder clean.
    void set_limits(float v_max_mm_s, float a_lon_mm_s2, float a_lat_mm_s2,
                    float a_brk_mm_s2 = 0.0f);
    // THE CLOTHOID HOLD (2026-09-26, the MR5 crash on the new maze: the V at
    // (2,2), diagonal to diagonal, R72, entered off a 509 mm diagonal at
    // 1215 mm/s). The planner brakes to each arc's speed AT THE ARC -- the
    // ramp-in clothoid's binding point is its end -- so the brake runs
    // through the clothoid while the lateral load climbs from 0 to a_lat:
    // the log shows v commanded 905 -> 624 over the 43 mm of clothoid
    // (5000 mm/s^2 of brake, the pointwise cap shaping it) with the lateral
    // arriving at 4000, a resultant of 6400 on a tyre whose knee is
    // 5500-7000, the yaw 15 deg behind the reference at the arc, the post.
    // MR2-MR4 flew the same corner at 339/416/480 mm/s and the heading
    // error grew with each rung (-6, -12, -9). With the hold ON the entry
    // clothoid's cap is the ARC's speed and the braking target is the
    // clothoid START, so the brake is over before the lateral begins; the
    // exit clothoid likewise holds the arc speed until the curvature is
    // gone, so the launch out of the corner does not overlap the unloading
    // tyre either. Costs the clothoid lengths at arc speed instead of a
    // little above it: ~15 ms a corner. OFF is the planner as it was.
    void set_clothoid_hold(bool on) { clothoid_hold_ = on; }
    bool clothoid_hold() const { return clothoid_hold_; }
    float v_max() const { return v_max_; }
    float a_lon() const { return a_lon_; }
    float a_brk() const { return a_brk_ > 0.0f ? a_brk_ : a_lon_; }
    float a_lat() const { return a_lat_; }

    int seg_count() const { return n_; }
    const PathSeg &seg(int i) const { return seg_[i]; }
    float length_mm() const { return len_; }

    // Segment index containing s (clamped to the path).
    int seg_at(float s_mm) const;
    float kappa_at(float s_mm) const;
    // d(kappa)/ds, 1/mm^2. Zero on straights and arcs, constant along a
    // clothoid. Angular acceleration inside a transition is v^2 times this.
    float dkappa_at(float s_mm) const;
    bool in_arc(float s_mm) const { return kappa_at(s_mm) != 0.0f; }

    // Pose at arclength s (clamped). Exact, not sampled.
    PathPose pose(float s_mm) const;

    // -- Corner metadata, for the wall gates ---------------------------------
    int corner_count() const { return nc_; }
    // PATH-s of the arc's entry and exit tangent points.
    float corner_entry_s_mm(int k) const;
    float corner_exit_s_mm(int k) const;
    float corner_radius_mm(int k) const;
    // +1 = left / CCW, -1 = right / CW. The INSIDE of the turn is the left
    // wall for +1 and the right wall for -1 -- which is the wall whose END
    // the edge gate watches for.
    int corner_dir(int k) const;
    // SPATIAL corridor length the corner consumes either side of the corner
    // point. For a 90-degree arc this is exactly R; the general form is
    // R*tan(|angle|/2). See the anchor-trap note in the header comment.
    float corner_tangent_mm(int k) const;

    // -- Per-tick reference --------------------------------------------------
    struct Sample {
        float v_mm_s;     // commanded forward speed at s
        float omega_dps;  // v * kappa, the rate feedforward wants this
        float yaw_deg;    // reference heading at s (path frame)
        float kappa;      // curvature at s
        bool in_arc;
        bool at_end;      // s has reached the path end
    };
    // v_prev is the previous tick's commanded v (for the accel ramp) and
    // dt_s the tick period. Everything else comes from s.
    Sample sample(float s_mm, float v_prev_mm_s, float dt_s) const;

    // Pointwise speed cap at s, ignoring lookahead and the accel ramp.
    // Exposed for tests and for the 'lim' style diagnostics.
    float v_cap_at(float s_mm) const;

   private:
    bool push(float len_mm, float kappa0, float kappa1);
    bool add_arc_plain(float radius_mm, float angle_deg);

    float clothoid_len_ = 0.0f;
    float next_vcap_ = 0.0f;

    PathSeg seg_[PATH_MAX_SEGS];
    int n_ = 0;
    float len_ = 0.0f;

    int corner_seg_[PATH_MAX_CORNERS];    // first segment of the primitive
    int corner_nseg_[PATH_MAX_CORNERS];   // 1 (pure arc) or 3 (clothoid)
    float corner_t_mm_[PATH_MAX_CORNERS]; // corridor room, each side
    int nc_ = 0;

    float x0_ = 0.0f, y0_ = 0.0f, yaw0_rad_ = 0.0f;
    float v_max_ = PATH_V_MAX_DEFAULT;
    float a_lon_ = PATH_A_LON_DEFAULT;
    float a_brk_ = 0.0f;   // 0 = a_lon_
    float a_lat_ = PATH_A_LAT_DEFAULT;
    bool clothoid_hold_ = true;
};

// WHEN TO REPLACE THIS WITH CLOTHOIDS
// -----------------------------------
// Constant-radius corners hand the executor a curvature STEP at each
// tangent point: the reference asks for the full arc rate instantly, the
// plant cannot deliver it, and the mouse enters late and runs wide. The
// slew rail bounds the resulting jerk but does not remove the error -- it
// converts it into a bounded, REPEATABLE entry transient.
//
// Repeatable is the point. Measure it on the 'k' harness (exit yaw error
// and exit lateral, one corner at a time, both directions, at 2-3 radii):
//
//   * a bias that is the same every run is trim -- tasym, track, or a
//     constant entry lead. Fix it there and keep this geometry.
//   * a bias that GROWS with speed, or scatter that grows with speed, is
//     the curvature step itself. That is the signal to make the reference
//     trackable, i.e. clothoid entry/exit ramps, and V1's implementation is
//     the reference for how (TW3_V1 lib/Control/Path.cpp, corner_geom()).
//
// Do not go to clothoids because the first corner looks untidy. Go when the
// harness says the error scales with speed rather than sitting still.
