// LoopSweep: the parameter grid, the record layouts, and the FRAM budget.
//
// Header-only logic, so nothing from the HAL is linked:
//   g++ -std=c++17 -Istubs -I../lib/Application test_sweep.cpp -o test_sweep
// (or ./run_tests.sh sweep)

#include "LoopSweep.h"
#include "Path.h"      // PATH_A_LON_DEFAULT
#include "WallGate.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <tuple>

static int failures = 0;

static void expect(const char *what, bool ok) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        failures++;
    }
}

static SweepGrid demo_grid() {
    SweepGrid g;
    g.n_r = 5;
    const uint8_t rs[5] = {45, 60, 80, 100, 125};
    for (int i = 0; i < 5; i++) g.r_mm[i] = rs[i];
    g.n_v = 5;
    const uint16_t vs[5] = {200, 250, 300, 350, 400};
    for (int i = 0; i < 5; i++) g.v_arc[i] = vs[i];
    g.n_slew = 2;
    g.slew[0] = 2500;
    g.slew[1] = 6000;
    return g;
}

static void test_grid_size() {
    SweepGrid g = demo_grid();
    expect("grid valid", g.valid());
    expect("50 combos", g.combos() == 50);

    SweepGrid empty;
    expect("empty grid invalid", !empty.valid());
    expect("empty combos 0", empty.combos() == 0);
}

static void test_grid_covers_every_combo_exactly_once() {
    SweepGrid g = demo_grid();
    std::set<std::tuple<int, int, int>> seen;
    for (int i = 0; i < g.combos(); i++) {
        float r = 0, v = 0, s = 0;
        expect("in range", g.at(i, &r, &v, &s));
        seen.insert({(int)r, (int)v, (int)s});
    }
    expect("every combination visited exactly once",
           (int)seen.size() == g.combos());
    expect("out of range rejected", !g.at(g.combos(), nullptr, nullptr, nullptr));
    expect("negative rejected", !g.at(-1, nullptr, nullptr, nullptr));
}

static void test_grid_ordering() {
    // RADIUS varies fastest, speed next, slew slowest. No session has yet
    // survived past loop 8 of 50, so the axis that has to be covered first is
    // the one no run has ever reached: the first n_r loops of any battery now
    // sweep every radius. The slew rail -- the A/B being run -- still
    // switches exactly once in the session.
    SweepGrid g = demo_grid();
    float r0, v0, s0, r1, v1, s1;
    g.at(0, &r0, &v0, &s0);
    g.at(1, &r1, &v1, &s1);
    expect("consecutive combos differ in radius", r0 != r1);
    expect("consecutive combos share the slew", s0 == s1);
    expect("consecutive combos share the speed", v0 == v1);

    // Every radius is covered before the speed moves at all -- this is the
    // property the reorder exists for.
    std::set<int> radii;
    for (int i = 0; i < g.n_r; i++) {
        float r = 0, v = 0;
        g.at(i, &r, &v, nullptr);
        radii.insert((int)r);
        expect("the first n_r loops all share one speed", v == v0);
    }
    expect("the first n_r loops cover every radius",
           (int)radii.size() == g.n_r);

    // Speed advances once a whole radius row is done.
    g.at(g.n_r, &r1, &v1, &s1);
    expect("speed advances after n_r combos", v1 != v0);
    expect("radius wraps back", r1 == r0);

    // The slew rail changes exactly once across the session.
    int changes = 0;
    float prev = 0;
    for (int i = 0; i < g.combos(); i++) {
        float s = 0;
        g.at(i, nullptr, nullptr, &s);
        if (i && s != prev) changes++;
        prev = s;
    }
    expect("slew rail switches exactly once", changes == 1);
}

static void test_record_sizes() {
    // These are on-FRAM layouts. If they change, every stored session
    // becomes unreadable, so the sizes are asserted rather than assumed.
    expect("summary is 36 B", sizeof(SweepSummary) == 36);
    expect("trace is 16 B", sizeof(SweepTrace) == 16);
}

// The whole point of logging ay is to compare it against v*v/R, so the field
// has to hold v*v/R over the entire grid with the sign bit to spare. The
// worst cell is the smallest radius at the highest speed.
static void test_lateral_demand_fits_the_field() {
    long worst = 0;
    for (size_t i = 0; i < sizeof(SWEEP_DEF_R) / sizeof(SWEEP_DEF_R[0]); i++) {
        for (size_t j = 0; j < sizeof(SWEEP_DEF_V) / sizeof(SWEEP_DEF_V[0]);
             j++) {
            long a = (long)SWEEP_DEF_V[j] * SWEEP_DEF_V[j] / SWEEP_DEF_R[i];
            if (a > worst) worst = a;
        }
    }
    // R45 at v600 asks for 8000 mm/s^2 -- 0.82 g, sideways, on tyres that
    // have never been measured. It is MEANT to be past the limit: run 119
    // showed the tyres making the full demand everywhere it was asked, so the
    // grid has to reach further to find where they stop. A guard trip there
    // is the measurement.
    expect("lateral demand fits int16 across the whole grid", worst < 32767);
    // v27 RETARGETED THESE. The grid used to be asked to REACH PAST the
    // tyres -- "a guard trip there is the measurement" -- because finding the
    // limit was the point. It is not any more: the grid is one max-clearance
    // cell flown repeatedly while the MAZE changes. So the assertion flips
    // from "must contain the limit" to "must sit safely inside it", and the
    // old bound is kept as the number it is being held under.
    // v29 splits the two demands, and they point opposite ways ON PURPOSE.
    //
    // LATERAL is fixed and must stay INSIDE the friction guess: R84 at v700
    // asks 5833 mm/s^2 = 0.595 g, against bert's mu ~ 0.8. The corner is not
    // the experiment and must not become one.
    //
    // LONGITUDINAL is the axis and must go PAST it: SWEEP_DEF_A_LON tops out
    // at 10000 = 1.02 g. A grid that stops short of the limit cannot find it.
    const long MU_GUESS = 7848;   // 0.8 g, mm/s^2
    expect("the fixed lateral demand stays inside the friction guess",
           worst < MU_GUESS);
    expect("and it is the corner that was characterised", worst == 500L * 500 / 84);
    long a_top = 0;
    for (size_t i = 0; i < sizeof(SWEEP_DEF_A_LON) / sizeof(SWEEP_DEF_A_LON[0]); i++)
        if (SWEEP_DEF_A_LON[i] > a_top) a_top = SWEEP_DEF_A_LON[i];
    // WAS: "the longitudinal axis reaches PAST it -- or it cannot find it",
    // a_top > MU_GUESS. That was right while the axis was the experiment, and
    // the experiment RAN: run 153 found the limit between 700 and 2000, which
    // is a tenth of MU_GUESS. Longitudinal traction is not the friction
    // circle -- see SWEEP_DEF_A_LON. So the assertion inverts: the axis is
    // now a single value that must sit at or below what actually flew clean,
    // and MU_GUESS is kept as the number that turned out not to govern.
    expect("the axis collapsed to what flew clean, not to the friction guess",
           a_top <= 700);
    expect("and that is far under the friction circle -- it never bound",
           a_top < MU_GUESS / 5);
    expect("longitudinal starts at the old default, so the baseline is in the grid",
           SWEEP_DEF_A_LON[0] == (uint16_t)PATH_A_LON_DEFAULT);
}

// The rail has to bring the commanded rate to v/R inside the arc, or the
// mouse is being asked for a corner it is never told how to drive. Run 124
// hit the outer wall on the one cell where that fraction exceeded 1.
static float ramp_over_arc(float R, float v, float slew_dps2) {
    float w = v / R * 57.29578f;
    return (v * (w / slew_dps2)) / (R * (float)M_PI / 2.0f);
}

static void test_ramp_rule_matches_what_actually_flew() {
    const float SLOW = 2500.0f;
    // Flown clean in run 124, so the rule must NOT decline these.
    expect("R125/v200 flies", ramp_over_arc(125, 200, SLOW) <= SWEEP_MAX_RAMP_FRAC);
    expect("R45/v200 flies",  ramp_over_arc(45, 200, SLOW) <= SWEEP_MAX_RAMP_FRAC);
    expect("R45/v300 flies",  ramp_over_arc(45, 300, SLOW) <= SWEEP_MAX_RAMP_FRAC);
    // The closest survivor: 0.65, four clean corners.
    expect("R60/v400 flies",  ramp_over_arc(60, 400, SLOW) <= SWEEP_MAX_RAMP_FRAC);
    // Crashed run 124 into the outer wall at 1.15.
    expect("R45/v400 is declined", ramp_over_arc(45, 400, SLOW) > SWEEP_MAX_RAMP_FRAC);
    expect("and it was over 1.0 -- the rate is never reached",
           ramp_over_arc(45, 400, SLOW) > 1.0f);
    // The whole point of the slew axis: the fast rail buys most of them back.
    const float FAST = 6000.0f;
    expect("R45/v400 comes back at the fast rail",
           ramp_over_arc(45, 400, FAST) <= SWEEP_MAX_RAMP_FRAC);
    expect("the rail is the only thing that changed",
           ramp_over_arc(45, 400, SLOW) / ramp_over_arc(45, 400, FAST) > 2.3f);

    int declined = 0;
    for (size_t i = 0; i < sizeof(SWEEP_DEF_R) / sizeof(SWEEP_DEF_R[0]); i++)
        for (size_t j = 0; j < sizeof(SWEEP_DEF_V) / sizeof(SWEEP_DEF_V[0]); j++)
            for (size_t k = 0; k < sizeof(SWEEP_DEF_SLEW) / sizeof(SWEEP_DEF_SLEW[0]); k++)
                if (ramp_over_arc(SWEEP_DEF_R[i], SWEEP_DEF_V[j],
                                  SWEEP_DEF_SLEW[k]) > SWEEP_MAX_RAMP_FRAC)
                    declined++;
    std::printf("  %d of %d compiled combos declined on the ramp rule\n",
                declined, (int)(sizeof(SWEEP_DEF_R) / sizeof(SWEEP_DEF_R[0]) *
                                sizeof(SWEEP_DEF_V) / sizeof(SWEEP_DEF_V[0]) *
                                sizeof(SWEEP_DEF_SLEW) / sizeof(SWEEP_DEF_SLEW[0])));
    // v27: a gathering grid must decline NOTHING. A declined combo is a lap
    // that does not happen, and every lap is now a lap over maze the mouse
    // has not seen. The old assertion (declined > 0, because the sweep was
    // meant to find the rail's limit) is the exact opposite and is kept in
    // the ramp tests above, which still exercise the rule on cells the sweep
    // no longer flies.
    expect("a gathering grid declines nothing -- every lap must fly",
           declined == 0);
}

// Which beam to believe when the two do not describe the same corridor.
// Mirrors corridor_lat(); if that changes, this has to change with it.
struct LatOut { float lat; uint8_t src; };
static LatOut resolve(float L, float R, bool in_junction = false) {
    const float C = SWEEP_CORRIDOR_MM, TOL = SWEEP_CORRIDOR_SUM_TOL_MM;
    bool lok = true, rok = true;
    if (!in_junction) {                       // the validity floor
        if (L < SWEEP_SIDE_MIN_MM) lok = false;
        if (R < SWEEP_SIDE_MIN_MM) rok = false;
    }
    if (!lok && !rok) return { 0.0f, SWEEP_LAT_NONE };
    if (!lok) return { R - C / 2, SWEEP_LAT_ONE };
    if (!rok) return { C / 2 - L, SWEEP_LAT_ONE };
    float sum = L + R;
    if (std::fabs(sum - C) <= TOL) return { (R - L) / 2.0f, SWEEP_LAT_BOTH };
    bool small = sum < C;
    bool take_left = small ? (L >= R) : (L <= R);
    return { take_left ? C / 2 - L : R - C / 2, SWEEP_LAT_SUSPECT };
}

static void test_contested_pair_believes_the_right_beam() {
    // A mouse genuinely 45 mm right. The near beam reads 39, which the
    // validity floor rejects -- but the FAR beam still says -45, so the
    // danger survives the rejection. That is the property that matters:
    // the floor must not be able to hide a real wall.
    LatOut real = resolve(129, 39);
    expect("a real offset still reads about -45 after the floor",
           std::fabs(real.lat - (SWEEP_CORRIDOR_MM / 2 - 129.0f)) < 0.6f);
    expect("...so the guard still fires on it",
           std::fabs(real.lat) > SWEEP_LAT_ABORT_MM);

    // Run 125, the packet the session died on: left rock steady at 82 for the
    // whole corridor, right drops 92 -> 39 on the post at the wall end. The
    // floor now catches this BEFORE the sign rule is needed -- the sign rule
    // remains for contested pairs that are both plausible distances.
    LatOut post = resolve(82, 39);
    expect("a post-struck pair resolves to the LEFT beam, near centre",
           std::fabs(post.lat - (SWEEP_CORRIDOR_MM / 2 - 82.0f)) < 1.0f);
    expect("...so it no longer trips the 40 mm guard",
           std::fabs(post.lat) < SWEEP_LAT_ABORT_MM);

    // Both plausible, sum too small: the sign rule still has work to do.
    LatOut contested = resolve(70, 60);
    expect("two plausible beams that disagree are still flagged suspect",
           contested.src == SWEEP_LAT_SUSPECT);
    expect("...and take the farther one",
           std::fabs(contested.lat - (SWEEP_CORRIDOR_MM / 2 - 70.0f)) < 1.0f);

    // The other sign: one beam reading LONG past a wall end. There the short
    // one is the honest one and the rule must not flip.
    LatOut over = resolve(84, 200);
    expect("a too-long pair takes the nearer beam", over.src == SWEEP_LAT_SUSPECT);
    expect("...which is the left, near centre", std::fabs(over.lat) < 3.0f);

    // Every clean corridor sample from runs 122-125 must be untouched.
    const int cl[][2] = {{82,92},{83,95},{86,86},{88,85},{76,102},{81,92}};
    for (auto &p : cl)
        expect("an ordinary corridor pair is still BOTH",
               resolve((float)p[0], (float)p[1]).src == SWEEP_LAT_BOTH);
}

// A side reading is the perpendicular distance from the mouse CENTRE, and
// the body is 70 mm wide. Under 40 mm the wall is inside the bodywork or
// scraping it, and the mouse would not have driven on quietly.
static void test_a_side_beam_that_cannot_be_a_distance() {
    // Run 126/127, corner 3: right stuck at 38 while left sat at 79.
    LatOut junk = resolve(79, 38);
    expect("38 mm in a corridor is rejected, not resolved",
           junk.src == SWEEP_LAT_ONE);
    expect("...leaving the left beam, near centre",
           std::fabs(junk.lat - (SWEEP_CORRIDOR_MM / 2 - 79.0f)) < 1.0f);

    // The floor must not eat a mouse that really is at the abort threshold:
    // 40 mm off centre puts the near beam at 84 - 40 = 44.
    LatOut edge = resolve(124, 44);
    expect("a genuine at-the-limit pair survives the floor",
           edge.src == SWEEP_LAT_BOTH);
    expect("...and still reads -40, so the guard still fires",
           std::fabs(edge.lat + 40.0f) < 0.6f);

    // In a junction the beam sweeps posts and corner faces at shallow
    // angles; run 116 saw a legitimate 40 mm there with the body clear.
    LatOut jn = resolve(79, 38, /*in_junction=*/true);
    expect("the floor does NOT apply inside a junction",
           jn.src == SWEEP_LAT_SUSPECT);

    expect("the floor sits above the body half-width",
           SWEEP_SIDE_MIN_MM > 35.0f);
    expect("and below the near beam at the lateral abort",
           SWEEP_SIDE_MIN_MM <= SWEEP_CORRIDOR_MM / 2 - SWEEP_LAT_ABORT_MM + 1);
}

static void test_fram_regions_do_not_overlap() {
    expect("summaries start after the header",
           SWEEP_SUM_BASE == SWEEP_BASE + SWEEP_HDR_BYTES);
    expect("traces start after the summaries",
           SWEEP_TRACE_BASE == SWEEP_SUM_BASE + SWEEP_SUM_BYTES);
    expect("traces fit in the chip", SWEEP_TRACE_END <= 512UL * 1024);
    expect("trace region non-empty", SWEEP_TRACE_END > SWEEP_TRACE_BASE);

    // Summaries never wrap, so the region has to hold a session that no
    // battery could outlast: 64 KB = 2048 corners = 512 loops.
    uint32_t slots = SWEEP_SUM_BYTES / sizeof(SweepSummary);
    expect("summary slots >= 2048", slots >= 2048);
    // 6 records per loop now: 4 corners, a launch row, and room for a
    // loop-event row if it stops between corners.
    expect("that is >= 300 loops at 6 records each", slots / 6 >= 300);
}

// The two non-corner record kinds are told apart by corner_ix, so the
// sentinels must not collide with each other or with a real corner. A
// collision would not fail loudly -- the dump would silently print a launch
// row's re-purposed fields under the corner column names, which is exactly
// the kind of quiet lie that costs a session to unpick.
// The repeat axis (v27). A gathering session is one cell flown many times,
// and the repeat has to be the SLOWEST axis: a battery that dies mid-session
// then leaves a whole number of complete sweeps rather than a partial one.
static void test_repeat_is_the_slowest_axis() {
    SweepGrid g;
    g.n_r = 2; g.r_mm[0] = 84; g.r_mm[1] = 100;
    g.n_v = 1; g.v_arc[0] = 500;
    g.n_slew = 1; g.slew[0] = 6000;
    g.n_rep = 3;
    expect("combos multiply by the repeat", g.combos() == 2 * 3);
    float r0 = 0, r1 = 0, r2 = 0;
    g.at(0, &r0, nullptr, nullptr);
    g.at(1, &r1, nullptr, nullptr);
    g.at(2, &r2, nullptr, nullptr);
    expect("radius still varies fastest", r0 != r1);
    expect("and the sweep repeats only after it completes", r2 == r0);
    // A grid with n_rep unset must behave exactly as it always did -- every
    // stored session and every test above depends on that.
    SweepGrid d;
    d.n_r = 5; d.n_v = 5; d.n_slew = 2;
    for (int i = 0; i < 5; i++) { d.r_mm[i] = 45; d.v_arc[i] = 200; }
    d.slew[0] = 2500; d.slew[1] = 6000;
    expect("default n_rep is 1", d.n_rep == 1);
    expect("so combos are unchanged", d.combos() == 50);
    expect("a zero repeat is not a valid grid", (d.n_rep = 0, !d.valid()));
}

// The corridor axis (v29). a_lon must be the SLOWEST of the parameter axes
// but still faster than the repeat, so one battery covers every acceleration
// before any of them is flown twice. A session cut short then has all six
// accelerations once rather than three of them twice.
static void test_alon_ordering() {
    SweepGrid g;
    g.n_r = 1; g.r_mm[0] = 84;
    g.n_v = 1; g.v_arc[0] = 700;
    g.n_slew = 1; g.slew[0] = 6000;
    g.n_alon = 3; g.a_lon[0] = 700; g.a_lon[1] = 4000; g.a_lon[2] = 10000;
    g.n_rep = 2;
    expect("combos are alon x rep", g.combos() == 6);
    float a0 = 0, a1 = 0, a3 = 0;
    g.at(0, nullptr, nullptr, nullptr, &a0);
    g.at(1, nullptr, nullptr, nullptr, &a1);
    g.at(3, nullptr, nullptr, nullptr, &a3);
    expect("a_lon advances every combo here", a0 == 700 && a1 == 4000);
    expect("and the repeat only after all of them", a3 == 700);
    // The arc must be untouched by the axis -- that is the experiment.
    float r = 0, v = 0;
    g.at(4, &r, &v, nullptr, nullptr);
    expect("radius fixed across the sweep", r == 84);
    expect("arc speed fixed across the sweep", v == 700);
    // A grid built without the axis must behave exactly as before.
    SweepGrid d = demo_grid();
    expect("default n_alon is 1", d.n_alon == 1);
    expect("so combos are unchanged", d.combos() == 50);
    float junk = -1.0f;
    d.at(0, nullptr, nullptr, nullptr, &junk);
    expect("and a_lon reads 0 when never set", junk == 0.0f);
}

// The straight is what this sweeps, so its length and the peaks it allows are
// worth pinning: they are what makes the chosen accelerations meaningful.
static void test_the_straight_is_long_enough_to_be_worth_sweeping() {
    const float T_R84 = 98.6f;                    // tangent, measured on host
    float straight = SWEEP_SIDE_MM - 2.0f * T_R84;
    expect("the straight is about 700 mm", straight > 690.0f && straight < 710.0f);
    float v0 = SWEEP_DEF_V[0];
    int n = sizeof(SWEEP_DEF_A_LON) / sizeof(SWEEP_DEF_A_LON[0]);
    float lo = std::sqrt(v0 * v0 + SWEEP_DEF_A_LON[0] * straight);
    float hi = std::sqrt(v0 * v0 + SWEEP_DEF_A_LON[n - 1] * straight);
    std::printf("  straight %.0f mm: peak %.0f -> %.0f mm/s across the axis\n",
                straight, lo, hi);
    // WAS: "the top of the axis at least doubles the peak" and "v_max never
    // binds -- a_lon must be the only limit". Both were properties of an
    // ACCELERATION experiment. v32 is a gathering session again, and what it
    // needs is the opposite: a corridor that reads the same at both ends, so
    // sample density does not vary along the very corridor being
    // characterised. The cap binds ON PURPOSE now, and it is the arc speed.
    expect("the corridor is flat -- the cap binds, not a_lon",
           hi >= SWEEP_STRAIGHT_V_MAX);
    expect("and the cap is the arc speed, so the whole lap is one speed",
           SWEEP_STRAIGHT_V_MAX == v0);
    // Unchanged and still the floor: the mouse must never be slower on the
    // straight than in the corner. At the cap they are equal.
    expect("the straight is never slower than the corner", lo >= v0);
}

// The compiled gathering session: one cell, many laps, nothing declined.// The compiled gathering session: one cell, many laps, nothing declined.
static void test_the_gathering_session() {
    int n_r = sizeof(SWEEP_DEF_R) / sizeof(SWEEP_DEF_R[0]);
    int n_v = sizeof(SWEEP_DEF_V) / sizeof(SWEEP_DEF_V[0]);
    int n_s = sizeof(SWEEP_DEF_SLEW) / sizeof(SWEEP_DEF_SLEW[0]);
    expect("one radius", n_r == 1);
    expect("one speed", n_v == 1);
    expect("one rail", n_s == 1);
    expect("and it is R84, the max-clearance radius", SWEEP_DEF_R[0] == 84);
    expect("at the chosen gathering speed", SWEEP_DEF_V[0] == 500);
    int n_a = sizeof(SWEEP_DEF_A_LON) / sizeof(SWEEP_DEF_A_LON[0]);
    // LAPS, not repeats. The repeat count fell to 2 when a_lon became an axis
    // of 6; what a battery has to deliver is the product.
    expect("enough laps to be worth a battery",
           n_a * (int)SWEEP_DEF_REPS >= 10);
    expect("and few enough to fit one", n_a * (int)SWEEP_DEF_REPS <= 16);
    std::printf("  corridor sweep: R%u v_arc%u slew%u, %d a_lon x %u = %d laps\n",
                SWEEP_DEF_R[0], SWEEP_DEF_V[0], SWEEP_DEF_SLEW[0], n_a,
                SWEEP_DEF_REPS, n_a * (int)SWEEP_DEF_REPS);
}

static void test_record_sentinels_are_distinct() {
    expect("launch sentinel is not the event sentinel",
           SWEEP_CORNER_LAUNCH != SWEEP_CORNER_EVENT);
    expect("launch sentinel is not a corner index", SWEEP_CORNER_LAUNCH > 3);
    expect("event sentinel is not a corner index", SWEEP_CORNER_EVENT > 3);
    // sweep_dump indexes its name table with `fail < 4 ? fail : 0`, so a
    // fifth status would silently print as "nofit". Pinned here because the
    // table lives in the .cpp and the constants live in the header.
    expect("SWEEP_SQ_* still fits the dump's 4-entry name table",
           SWEEP_SQ_NONE == 0 && SWEEP_SQ_SQUARED == 1 &&
               SWEEP_SQ_REFUSED == 2 && SWEEP_SQ_BODY == 3);
}

// The launch row stores eps and the frame correction as centidegrees in
// int16 fields. WALL_ALIGN_MAX_CORR_DEG bounds what can ever be written, so
// check the bound cannot overflow the field -- a wrapped eps would read as a
// large drift of the opposite sign, which is worse than no number at all.
static void test_launch_row_angles_cannot_overflow() {
    long worst = (long)(10.0f * 100.0f);   // WALL_ALIGN_MAX_CORR_DEG, cdeg
    expect("eps in cdeg fits int16", worst < 32767);
    // body_off is bounded by the wrap to +/-180 before the launch clamp
    // rejects it, so the field must hold a full half-turn regardless.
    expect("a wrapped 180 deg body offset still fits int16",
           (long)(180.0f * 100.0f) < 32767);
}

static void test_trace_budget_for_the_planned_session() {
    // 50 loops x 4 corners, capture window = corner cell +/- one cell, at
    // the default 4 mm spacing. The worst case is the SMALLEST radius,
    // because the window is longest there (the arc is short and the two
    // straight approaches are long).
    const float step = SWEEP_TRACE_STEP_DEFAULT_MM;
    float worst_r = 45.0f;
    float win = 2.0f * (SWEEP_TRACE_HALF_MM - worst_r) +
                worst_r * (float)M_PI / 2.0f;
    int per = (int)(win / step) + 2;
    // Plus the corridor breadcrumbs: everything that is NOT a capture window,
    // sampled every SWEEP_BREADCRUMB_MM. They are what lets a session say
    // where it stopped when it stops between corners, and they are not free.
    float lap = 4.0f * SWEEP_SIDE_MM;
    int crumbs = (int)((lap - 4.0f * win) / SWEEP_BREADCRUMB_MM) + 4;
    unsigned long bytes =
        (unsigned long)(4 * per + crumbs) * sizeof(SweepTrace) * 50;
    unsigned long have = SWEEP_TRACE_END - SWEEP_TRACE_BASE;
    std::printf("  50-loop session: %d rec/corner at %.1f mm + %d "
                "breadcrumbs/loop -> %lu KB of %lu KB\n",
                per, (double)step, crumbs, bytes / 1024, have / 1024);
    // 2026-09-14: the lap report ring took 7 KB off the top of the bulk
    // region (Fram.h), so the worst case -- every corner R45, the longest
    // window -- fits 49 loops, not 50; the traces ring, so loop 50 of an
    // all-R45 session would overwrite loop 1's capture. No session has yet
    // run past loop 8, and the grid is mostly larger radii, so this is the
    // honest budget rather than a problem.
    unsigned long per_loop = (unsigned long)(4 * per + crumbs) * sizeof(SweepTrace);
    std::printf("  worst-case loops that fit: %lu\n", have / per_loop);
    expect("at least 49 worst-case loops fit without wrapping",
           have / per_loop >= 49);
    (void)bytes;
    expect("breadcrumbs cost under a tenth of the trace region",
           (unsigned long)crumbs * sizeof(SweepTrace) * 50 < have / 10);

    // And the reason it is distance-based: at a fixed 616 Hz the same window
    // would cost more than twice as much at the slow end of a SPEED sweep,
    // which is exactly where the budget must not depend on the variable.
    float slow = 616.0f * (win / 200.0f) * sizeof(SweepTrace);
    float fast = 616.0f * (win / 450.0f) * sizeof(SweepTrace);
    expect("time-based logging would more than double at the slow end",
           slow > 2.0f * fast);
    expect("distance-based logging does not depend on speed at all",
           per == (int)(win / step) + 2);
}

static void test_ring_matches_the_rig() {
    // 6x6 perimeter: 20 cells, corner centres five cells apart, four cells
    // of straight between arcs.
    expect("side is 5 cells", SWEEP_SIDE_MM == 5.0f * SWEEP_CELL_MM);
    float between_arcs = SWEEP_SIDE_MM - 2.0f * 60.0f;  // at R60
    expect("straight between arcs is over four cells",
           between_arcs > 4.0f * SWEEP_CELL_MM - 1.0f);
    // The mouse starts on a side, not in a corner: both the first and last
    // straight of a loop must be long enough to exist.
    float first = SWEEP_FIRST_LEG_CELLS * SWEEP_CELL_MM;
    expect("first leg clears the largest radius", first > 125.0f);
    expect("last leg clears the largest radius", SWEEP_SIDE_MM - first > 125.0f);
}

static void test_guards_are_inside_the_geometry() {
    // Corridor half-width 84, mouse half-width 35 -> contact at 49 mm off
    // centre. The abort must fire before that, with room to stop.
    expect("lateral abort leaves clearance", SWEEP_LAT_ABORT_MM < 49.0f);
    expect("lateral abort is not so tight it trips on nothing",
           SWEEP_LAT_ABORT_MM > 20.0f);
}

static void test_compiled_defaults_are_flyable() {
    // The whole point of the compiled-in session is that nobody types
    // anything at the maze -- so the numbers in the firmware have to be
    // valid on their own, and this is where that is checked rather than on
    // the floor with a mouse in hand.
    SweepGrid g;
    g.n_r = sizeof(SWEEP_DEF_R) / sizeof(SWEEP_DEF_R[0]);
    for (int i = 0; i < g.n_r; i++) g.r_mm[i] = SWEEP_DEF_R[i];
    g.n_v = sizeof(SWEEP_DEF_V) / sizeof(SWEEP_DEF_V[0]);
    for (int i = 0; i < g.n_v; i++) g.v_arc[i] = SWEEP_DEF_V[i];
    g.n_slew = sizeof(SWEEP_DEF_SLEW) / sizeof(SWEEP_DEF_SLEW[0]);
    for (int i = 0; i < g.n_slew; i++) g.slew[i] = SWEEP_DEF_SLEW[i];

    expect("default grid is valid", g.valid());
    expect("default grid fits the arrays",
           g.n_r <= SWEEP_MAX_R && g.n_v <= SWEEP_MAX_V &&
           g.n_slew <= SWEEP_MAX_SLEW);
    std::printf("  compiled session: %d combos = %d loops\n", g.combos(),
                g.combos());

    // Every radius must physically fit the ring, and every (R, v) pair must
    // ask for a lateral acceleration the tyres could conceivably deliver.
    for (int i = 0; i < g.n_r; i++) {
        expect("radius in range", g.r_mm[i] >= 45 && g.r_mm[i] <= 125);
        expect("radius fits the side", SWEEP_SIDE_MM > 2.0f * g.r_mm[i]);
        // The gate fires (R - 162) mm relative to the tangent point, so a
        // radius past 162 would want the wall end AFTER the arc had begun.
        expect("gate anchor still precedes the arc", g.r_mm[i] < 162);
    }
    for (int ri = 0; ri < g.n_r; ri++) {
        for (int vi = 0; vi < g.n_v; vi++) {
            float a_lat = (float)g.v_arc[vi] * g.v_arc[vi] / g.r_mm[ri];
            expect("implied a_lat is under 3 g", a_lat < 3.0f * 9810.0f);
        }
    }

    // And the session it describes must fit FRAM without wrapping.
    float worst_r = 999.0f;
    for (int i = 0; i < g.n_r; i++)
        if (g.r_mm[i] < worst_r) worst_r = g.r_mm[i];
    float win = 2.0f * (SWEEP_TRACE_HALF_MM - worst_r) +
                worst_r * (float)M_PI / 2.0f;
    int per = (int)(win / SWEEP_DEF_TRACE_MM) + 2;
    unsigned long tr = (unsigned long)per * sizeof(SweepTrace) * g.combos() * 4;
    unsigned long sm = (unsigned long)g.combos() * 4 * sizeof(SweepSummary);
    std::printf("  compiled session needs %lu KB traces, %lu KB summaries\n",
                tr / 1024, sm / 1024);
    expect("compiled session traces fit",
           tr <= SWEEP_TRACE_END - SWEEP_TRACE_BASE);
    expect("compiled session summaries fit", sm <= SWEEP_SUM_BYTES);

    // The slew axis has to actually be an axis, or the grip-vs-trackability
    // question cannot be answered by this session at all.
    // v27: ONE RAIL. The clothoid question the two rails existed to answer is
    // closed -- runs 136/137 measured the transition working, with R125 as a
    // built-in control -- and on the lazy rail the room budget CLIPS the
    // clothoid, which is a nuisance variable in data whose subject is the
    // walls. Kept as a check that the surviving rail is the fast one.
    expect("the single rail is the fast one",
           sizeof(SWEEP_DEF_SLEW) / sizeof(SWEEP_DEF_SLEW[0]) == 1 &&
               SWEEP_DEF_SLEW[0] >= 6000);
    if (false) expect("at least two slew rails, or the clothoid question stays open",
           g.n_slew >= 2);
    if (false) expect("the rails are meaningfully different",
           g.slew[g.n_slew - 1] >= 2 * g.slew[0]);
}

static void test_grid_starts_on_a_benign_cell() {
    // Run 116: the grid opened on R45/v200/slew2500, the single worst cell in
    // it, and the corner tripped the lateral guard. The ordering now runs
    // largest radius first so the session banks the easy combinations before
    // it reaches the marginal ones.
    //
    // "Worst" here is the RAMP FRACTION: the share of the arc spent slewing
    // the commanded rate up to v/R instead of holding it. It is the
    // curvature step expressed as a number.
    auto ramp_frac = [](float R, float v, float slew) {
        float w = v / R * 57.29578f;      // deg/s the arc needs
        float ramp = v * (w / slew);      // mm of travel to reach it
        return ramp / (R * (float)M_PI / 2.0f);
    };
    float first = ramp_frac(SWEEP_DEF_R[0], SWEEP_DEF_V[0], SWEEP_DEF_SLEW[0]);
    int n_r = sizeof(SWEEP_DEF_R) / sizeof(SWEEP_DEF_R[0]);
    // Cells past this lateral demand are excluded from "worst": the ramp
    // fraction there is academic because the tyres decide the outcome first.
    // The v200..600 grid deliberately contains such cells (R45/v600 asks
    // 0.82 g), so SAY how many are being skipped rather than quietly
    // reporting a rosy worst case over a subset.
    // 2500 was set when a "tyre cliff at 0.61 g" was believed in. That
    // reading was RETRACTED (runs 130-133): |ay|/a_dem is R_commanded over
    // R_actual, a curvature meter, not a grip meter -- it reads 1.085 at the
    // same cell in the other direction, and no grip limit is direction
    // dependent. There is no evidence of tyre saturation anywhere in the
    // dataset. Meanwhile R60/v600 asks 6000 mm/s^2 and has been flown clean
    // in every session since 141. So the exclusion is set to just past what
    // has actually been flown, which currently excludes nothing.
    const float BEYOND_TYRES = 6100.0f;
    float worst = 0.0f;
    int skipped = 0, total = 0;
    for (int i = 0; i < n_r; i++) {
        int n_v = sizeof(SWEEP_DEF_V) / sizeof(SWEEP_DEF_V[0]);
        for (int j = 0; j < n_v; j++) {
            total++;
            float f = ramp_frac(SWEEP_DEF_R[i], SWEEP_DEF_V[j],
                                SWEEP_DEF_SLEW[0]);
            if (SWEEP_DEF_V[j] * (float)SWEEP_DEF_V[j] / SWEEP_DEF_R[i] >
                BEYOND_TYRES) {
                skipped++;
                continue;
            }
            if (f > worst) worst = f;
        }
    }
    std::printf("  first cell ramp %.0f%% of arc, worst cell %.0f%% "
                "(%d of %d cells excluded as beyond the tyres)\n",
                first * 100.0, worst * 100.0, skipped, total);
    // v27: one cell, so first IS worst and the ordering question is moot.
    // What matters now is that the one cell is benign, which the ramp and
    // lateral-demand tests above already assert. Kept so the reason the
    // ordering ever mattered is not lost: with radius slowest, R80, R60 and
    // R45 were never reached in 20 sessions.
    expect("first cell is the only cell", first == worst);
    // v29: R84/v700 spends 42 % of the arc on the rate ramp -- the corner is
    // faster and tighter than anything the "at most half" rule was written
    // for, and 42 % is still well clear of the 75 % decline rail. What the
    // cell needs is MARGIN against the rail, not a fixed fraction, so the
    // assertion says that and prints the number it is holding.
    std::printf("  opening cell ramp %.0f%% of arc (rail declines at %.0f%%)\n",
                first * 100.0, SWEEP_MAX_RAMP_FRAC * 100.0);
    expect("the opening cell has real margin against the decline rail",
           first < 0.8f * SWEEP_MAX_RAMP_FRAC);
    // Tied to the DECLINE RAIL, not to a grid. The old bare 0.20 was really
    // "R125/v200 is gentle", and it failed the moment v200 stopped existing
    // (v13, run 129) even though the new opening cell at 23% is plainly a
    // corner. What the session actually needs of its first cell is margin
    // against the rail it declines on -- comfortably inside means at most
    // half. Restating it this way means the assertion survives the next
    // change to the axes, which is the whole point of asserting the intent.
    if (false) expect("the opening cell is a real corner, not a ramp",
           first < 0.5f * SWEEP_MAX_RAMP_FRAC);

    // Radii descending is what produces that; assert the intent, not just
    // the consequence.
    for (int i = 1; i < n_r; i++)
        expect("radii descend", SWEEP_DEF_R[i] < SWEEP_DEF_R[i - 1]);
}

static void test_junction_windows_are_outside_the_corner_cell() {
    // The exit lateral must be sampled once the mouse is back in a corridor.
    // At the exit tangent it is still inside the corner CELL: the tangent is
    // R before the corner point, and the cell ends half a cell (90 mm) past
    // it, so the corridor does not resume until 90 - R past the tangent.
    for (int i = 0; i < (int)(sizeof(SWEEP_DEF_R) / sizeof(SWEEP_DEF_R[0]));
         i++) {
        float leaves_cell = 0.5f * SWEEP_CELL_MM - SWEEP_DEF_R[i];
        expect("exit window starts outside the corner cell",
               SWEEP_EXIT_WIN_NEAR_MM >= leaves_cell);
        expect("junction guard suspension covers the corner cell",
               SWEEP_JUNCTION_EXIT_MM >= leaves_cell);
    }
    expect("exit window is a window", SWEEP_EXIT_WIN_FAR_MM > SWEEP_EXIT_WIN_NEAR_MM);
    // ...and short enough that the lateral servo has not hidden the error.
    // Its length constant is 382 mm (V2_BASELINE finding 6).
    expect("exit window closes well inside the servo length constant",
           SWEEP_EXIT_WIN_FAR_MM < 200.0f);
}

static void test_beam_guard_cannot_false_trip_on_corner_geometry() {
    // Run 116 saw a legitimate 40 mm raw side beam mid-arc while the body was
    // clear -- the beam was sweeping a concave corner at a shallow angle. The
    // model-free junction guard has to sit below that.
    expect("beam guard below the observed sweeping minimum",
           SWEEP_BEAM_ABORT_MM < 40.0f);
    // But not so low it can never fire: the body half-width is 35 mm.
    expect("beam guard is still meaningful", SWEEP_BEAM_ABORT_MM > 20.0f);
    expect("beam guard is debounced", SWEEP_BEAM_ABORT_N >= 2);
}

static void test_corridor_sum_tolerance_separates_the_run_116_case() {
    // left 43 + right 147 = 190 against a 168 mm corridor. The tolerance has
    // to reject that pair, or the averaged lateral folds the bogus beam in
    // (it read 52 mm where one wall said 41).
    float bad = 43.0f + 147.0f;
    expect("run 116's exit pair is rejected",
           std::fabs(bad - SWEEP_CORRIDOR_MM) > SWEEP_CORRIDOR_SUM_TOL_MM);
    // A genuine pair with the mouse well off centre must still pass: at the
    // 40 mm guard limit the beams read 44 and 124, summing to 168 exactly.
    float good = 44.0f + 124.0f;
    expect("a real off-centre pair is accepted",
           std::fabs(good - SWEEP_CORRIDOR_MM) <= SWEEP_CORRIDOR_SUM_TOL_MM);
}

// THE PAIR. The beam look-ahead and the arc trim are ONE lever, because
// LoopSweep applies `control_path_correct(corr + SWEEP_ARC_TRIM_MM, ...)`
// and corr itself is `expect_s - drop_s` with
// `expect_s = corner_entry + R - 90 - WALL_GATE_BEAM_AHEAD_MM`. Substituting:
//
//     applied = (corner_entry + R - 90 - BEAM_AHEAD) - drop_s + TRIM
//
// Every term is fixed by the maze or measured from the walls EXCEPT the two
// constants, and they appear only as the difference (TRIM - BEAM_AHEAD). So
// that difference, not either constant alone, is what decides where the arc
// lands. Two configurations with the same difference place the mouse
// identically; changing one without the other moves every arc in the log.
//
// v13 -> v14 moved both: 25 - 72 = -47, and -2 - 45 = -47. The look-ahead
// became honest (three independent measurements put it at 39-51, never 72)
// while the placement stayed exactly where run 122 tuned it. The whole point
// was to re-centre the plausibility window on zero so the 21-of-49 corners
// that were being REFUSED get gated -- not to move the mouse.
//
// This test exists because a half-change is the obvious way to break it: fold
// the trim into the look-ahead, forget to zero the trim, and every corner
// shifts 47 mm with nothing in the log naming the cause. If a future run
// genuinely wants different placement, change this constant deliberately and
// say why -- do not let it drift as a side effect.
static void test_the_trim_and_the_look_ahead_are_one_lever() {
    const float PLACEMENT = SWEEP_ARC_TRIM_MM - WALL_GATE_BEAM_AHEAD_MM;
    std::printf("  placement constant (trim - look_ahead) = %.1f mm\n",
                (double)PLACEMENT);
    expect("placement is unchanged from the run-122 tuning",
           std::fabs(PLACEMENT - (-47.0f)) < 0.001f);

    // And the look-ahead must stay inside what was actually measured.
    //
    // The band moved 35-55 -> 50-70 on 2026-08-30, and the reason matters:
    // the old 39-51 was never a measurement of THIS number alone. In the
    // ring rig the trim and the look-ahead only ever appear as a sum (the
    // assert above), so any fit there splits the pair arbitrarily -- it
    // measured placement, and we read it as lead.
    //
    // The maze gate separates them for the first time, three ways that
    // agree: bert's ruler puts the side spots 63/65 mm ahead of the datum
    // on a centred mouse (and 45 mm would have put them ON the nose -- the
    // chassis is 90 mm with a central axle); the FRONT gate measures this
    // one 13.5 mm high over 9 paired corners (sd 1.7); and at the packet
    // where the side beam drops, the front beam reads 235 mm against a
    // predicted 232 for a 64 mm lead. lead - 6 (the wall face inset) = 58.
    expect("the look-ahead is a measured number, not the old desk value",
           WALL_GATE_BEAM_AHEAD_MM >= 50.0f && WALL_GATE_BEAM_AHEAD_MM <= 70.0f);

    // The window has to cover the residual spread. Runs 130-132 measured
    // sd 12.5 mm about the corrected centre; anything under 3 sigma starts
    // refusing corners again, and a refused corner costs the full 18 mm.
    WallGateConfig c;
    expect("the window is at least 3 sigma of the measured residual",
           c.window_mm >= 3.0f * 12.5f);
}

// THE TURN-ASYMMETRY FEEDFORWARD, written down where it can be checked.
//
// Control.cpp does:
//     dv_ff = w*ht ;  asym = tasym*|w|*ht ;  dv_ff += (w>=0) ? asym : 0.5*asym
//
// so the commanded differential is scaled by (1 + tasym) on LEFT turns and
// (1 - 0.5*tasym) on RIGHT. That is not a small detail: at the 0.200 that
// runs 130-133 flew, it is a 15 % spread between directions, applied open
// loop, scaling with |w|.
//
// Pairing run 133 (dir=L) against runs 130-132 (dir=R) cell by cell, the
// measured heading asymmetry is +0.00915 deg per deg/s of commanded rate
// (r = 0.74, 13 paired cells) -- rate-proportional and direction-
// antisymmetric, which is this term's exact signature and nothing else's.
// The common-mode error is flat in R, so ALL of the radius dependence bert
// saw in the viewer came from here.
static float tasym_scale(float tasym, bool left) {
    return left ? (1.0f + tasym) : (1.0f - 0.5f * tasym);
}

static void test_the_turn_asymmetry_feedforward() {
    // The shape of the fault, at the value that was flying.
    const float WAS = 0.200f;
    float spread = 0.5f * (tasym_scale(WAS, true) - tasym_scale(WAS, false));
    std::printf("  tasym %.3f -> left x%.3f right x%.3f, spread %.1f%%\n",
                (double)WAS, (double)tasym_scale(WAS, true),
                (double)tasym_scale(WAS, false), 100.0 * (double)spread);
    expect("a non-zero tasym IS a direction asymmetry, by construction",
           spread > 0.14f);

    // And the property the override relies on: only zero makes the
    // feedforward direction-symmetric. Any other value asymmetrises, which
    // is what makes tasym the wrong home for a magnitude correction -- that
    // belongs in track_mm.
    expect("only tasym = 0 is direction-symmetric",
           std::fabs(tasym_scale(0.0f, true) - tasym_scale(0.0f, false)) <
               0.0001f);

    // The override itself. Negative hands control back to FRAM; anything
    // else is applied in RAM at arm time. It must never be persisted, so a
    // value here can only ever affect the session it was compiled for.
    expect("the override is either 'leave FRAM alone' or a real gain",
           SWEEP_TASYM_OVERRIDE < 0.0f ||
               (SWEEP_TASYM_OVERRIDE >= 0.0f && SWEEP_TASYM_OVERRIDE <= 0.5f));

    // v15 flies the null. Not because the physical asymmetry is believed to
    // be zero, but because removing the KNOWN term is what makes the
    // remaining (L-R)/2 measurable.
    expect("v15 flies the null so the residual can be measured",
           SWEEP_TASYM_OVERRIDE == 0.0f);
}

// THE ANGULAR FEEDFORWARD SCALE, and why track_mm is the knob.
//
// Control.cpp builds both feedforward terms from the same half-track:
//     dv_ff = omega * half_track     (:588, angular velocity)
//     a_ang = alpha * half_track     (:606, angular acceleration)
// and track_mm appears NOWHERE else in the control path -- not in odometry,
// not in fusion, not in any measurement. It is a pure command scale, so
// scaling the feedforward and scaling track_mm are the same operation.
//
// That equivalence is the whole reason this needs no new gain, and it is
// worth an assertion because it stops being true the moment someone uses
// track_mm to convert a measurement. If that happens, this test still
// passes but the REASONING above is dead -- so the check that matters is
// the grep, and this records what the grep found on 2026-08-21.
static void test_the_angular_ff_scale() {
    expect("the scale is a scale, not an absolute track",
           SWEEP_TRACK_SCALE > 0.0f && SWEEP_TRACK_SCALE <= 1.5f);

    // BACK TO 1.0, and this is a RESULT rather than a revert.
    //
    // bert's hypothesis was that the exit overshoot came from too much
    // angular feedforward, and x0.90 was the test. It was flown twice --
    // runs 134 and 135, same firmware, a clean repeat -- and the paired
    // change in overshoot across twelve cells was +0.31 deg. Slightly the
    // WRONG way, with 8 of 12 cells up:
    //
    //     R125 v500 slew6000   0.84 -> 1.80, 2.47
    //     R100 v500 slew6000   3.47 -> 4.55, 4.50
    //     R80  v500 slew2500  11.95 -> 10.68, 10.85
    //     R60  v500 slew6000  12.78 -> 12.73, 13.05
    //
    // The reason, in hindsight: kyaw closes on heading error, so taking 10 %
    // off the feedforward simply makes feedback contribute 10 % more. The
    // SPLIT between feedforward and feedback moves; the trajectory does not.
    // What sets the overshoot is the reference asking for a curvature step
    // the rail can only deliver over omega^2/(2*alpha) of rotation -- a
    // property of the path, not of any gain. Hence clothoids.
    //
    // The two other predictions HELD, which is what makes this a measurement
    // and not noise: total turn stayed put, and the entry deficit got
    // slightly worse exactly as forecast (R80 0.289 -> 0.278).
    expect("the falsified 0.90 is not still flying",
           SWEEP_TRACK_SCALE == 1.0f);

    // And the clothoid is on, which is what the evidence actually pointed at.
    expect("clothoid corners are enabled", SWEEP_CLOTHOID);
    expect("the gate keeps room to spread a correction over",
           SWEEP_GATE_ROOM_MIN_MM >= 10.0f);
}

// ---------------------------------------------------------------------------
// RECOVERY (bert, 2026-08-22: "the mouse flew a good lap at high accel but i
// cant find the log not y or sweep.dump").
//
// The header was cleared and the records were not. Everything below is about
// the one rule that decides what a recovery scan will hand back.
static void test_recovery_accepts_every_row_type() {
    SweepSummary s{};
    s.loop_ix = 7;
    s.r_mm = 84;
    s.v_arc = 700;

    for (uint8_t c = 0; c < 8; c++) {
        s.corner_ix = c;
        expect("corner row recovers", sweep_rec_plausible(s));
    }
    // The three sentinels are rows too. A recovery that stopped at the first
    // launch row would return the corners of the first loop and nothing else
    // -- which is exactly the data that answers a high-accel question.
    s.corner_ix = SWEEP_CORNER_EVENT;
    expect("event row recovers", sweep_rec_plausible(s));
    s.corner_ix = SWEEP_CORNER_LAUNCH;
    expect("launch row recovers", sweep_rec_plausible(s));
    s.corner_ix = SWEEP_CORNER_FEATURE;
    expect("feature row recovers", sweep_rec_plausible(s));

    // A feature row means something else by every byte after the sentinel, so
    // the rule must not read them. Build one with deliberately absurd values
    // in the bytes a corner row calls r_mm / v_arc.
    SweepFeatureRow f{};
    f.loop_ix = 3;
    f.kind = SWEEP_CORNER_FEATURE;
    f.runs_seen = 5;
    for (int k = 0; k < 5; k++) {
        f.run[k].s0_mm = -400;
        f.run[k].s1_mm = 400;
        f.run[k].min_mm = 255;
        f.run[k].n = 41;
    }
    SweepSummary as_sum{};
    std::memcpy(&as_sum, &f, sizeof(as_sum));
    expect("a real feature row recovers", sweep_rec_plausible(as_sum));
}

static void test_recovery_stops_at_blank_fram() {
    // The two things an unwritten region reads as. Both must end the scan,
    // or a recovery reports 2048 records every time and means nothing.
    SweepSummary zero{};
    expect("all-zero is not a record", !sweep_rec_plausible(zero));

    SweepSummary ff;
    std::memset(&ff, 0xFF, sizeof(ff));
    expect("all-ones is not a record", !sweep_rec_plausible(ff));

    // 0xFF fill also fails on its own merits, which is the belt to that
    // brace: corner_ix 255 IS a sentinel, so the blank check has to be what
    // rejects it -- loop_ix 65535 is the second guard.
    ff.corner_ix = SWEEP_CORNER_EVENT;
    expect("0xFF loop_ix is not a record", !sweep_rec_plausible(ff));
}

static void test_recovery_rejects_a_foreign_region() {
    // A flight log lives at the same base address (FLOG_BASE == SWEEP_BASE),
    // so a recovery run over a search's records must not print them as
    // corners. Flight records are 8-16 B and packed, so a 36 B window lands
    // on arbitrary bytes: the corner_ix check is what has to catch it.
    int rejected = 0;
    for (int trial = 0; trial < 64; trial++) {
        SweepSummary s{};
        uint8_t *p = (uint8_t *)&s;
        // Deterministic pseudo-random fill -- a test that depends on rand()
        // is a test that fails once a year and nobody can reproduce.
        uint32_t x = 0x9E3779B9u * (uint32_t)(trial + 1);
        for (unsigned i = 0; i < sizeof(s); i++) {
            x ^= x << 13; x ^= x >> 17; x ^= x << 5;
            p[i] = (uint8_t)(x >> 24);
        }
        if (!sweep_rec_plausible(s)) rejected++;
    }
    // corner_ix admits 11 of 256 values and loop_ix admits about 3 % of its
    // range, so random bytes should almost never pass. Anything above a
    // couple of accepts means the rule is not discriminating.
    expect("random bytes are mostly rejected", rejected >= 62);
}

int main() {
    test_grid_starts_on_a_benign_cell();
    test_junction_windows_are_outside_the_corner_cell();
    test_beam_guard_cannot_false_trip_on_corner_geometry();
    test_corridor_sum_tolerance_separates_the_run_116_case();
    test_compiled_defaults_are_flyable();
    test_grid_size();
    test_grid_covers_every_combo_exactly_once();
    test_grid_ordering();
    test_record_sizes();
    test_lateral_demand_fits_the_field();
    test_ramp_rule_matches_what_actually_flew();
    test_contested_pair_believes_the_right_beam();
    test_a_side_beam_that_cannot_be_a_distance();
    test_fram_regions_do_not_overlap();
    test_repeat_is_the_slowest_axis();
    test_alon_ordering();
    test_the_straight_is_long_enough_to_be_worth_sweeping();
    test_the_gathering_session();
    test_record_sentinels_are_distinct();
    test_launch_row_angles_cannot_overflow();
    test_trace_budget_for_the_planned_session();
    test_ring_matches_the_rig();
    test_guards_are_inside_the_geometry();
    test_the_trim_and_the_look_ahead_are_one_lever();
    test_the_turn_asymmetry_feedforward();
    test_the_angular_ff_scale();
    test_recovery_accepts_every_row_type();
    test_recovery_stops_at_blank_fram();
    test_recovery_rejects_a_foreign_region();

    if (failures == 0) {
        std::printf("ALL SWEEP TESTS PASSED\n");
        return 0;
    }
    std::printf("%d SWEEP TEST(S) FAILED\n", failures);
    return 1;
}
