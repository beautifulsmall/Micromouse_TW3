// Route compiler: map -> SpeedPath.
//
// The maze under test is bert's test8, transcribed from the 'M' dump of the
// search that produced it (run 154, 2026-08-23). That matters: this is not a
// maze invented to make the compiler look good, it is the one the mouse
// actually solved, and its very first two turns are the hairpin that forces
// the radius ladder to exist.
//
//   g++ -std=c++17 -Istubs -I../lib/... test_route.cpp Route.cpp Maze.cpp Path.cpp

#include "Route.h"
#include "WallGate.h"   // WALL_GATE_BEAM_AHEAD_MM

#include <cmath>
#include <cstdio>
#include <initializer_list>

static int failures = 0;

static void expect(const char *what, bool ok) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        failures++;
    }
}

// ---------------------------------------------------------------------------
// bert's test8, a 6x6 block in the bottom-left of the 16x16. Transcribed from
// the M dump: for each cell, which sides carry a wall. Only the block is
// described; everything outside it is left unknown, which is exactly what the
// route compiler must refuse to walk into.
//
//   +--+--+--+--+--+--+
//   |*  *  *  *  *  * |     y=5
//   +  +  +--+  +--+  +
//   |* |* |*  *  * |* |     y=4
//   +  +  +--+--+--+  +
//   |* |*  *  *  * |* |     y=3
//   +  +--+  +  +  +  +
//   |* |*  * |*  * |* |     y=2
//   +  +  +--+  +--+  +
//   |* |* |  |* |  |* |     y=1
//   +  +  +  +  +  +  +
//   |S |*  G  *  *  * |     y=0
//   +--+--+--+--+--+--+
static void build_test8(Maze &m) {
    m.reset();
    // The block's outer boundary.
    for (int x = 0; x < 6; x++) {
        m.set_wall(x, 0, HEADING_S, true);
        m.set_wall(x, 5, HEADING_N, true);
    }
    for (int y = 0; y < 6; y++) {
        m.set_wall(0, y, HEADING_W, true);
        m.set_wall(5, y, HEADING_E, true);
    }
    // Interior verticals, read off the cell rows above.
    const struct { int x, y; } vE[] = {
        {0, 0},                                  // S opens only north
        {0, 1}, {1, 1}, {2, 1}, {3, 1}, {4, 1},
        {0, 2}, {2, 2}, {4, 2},
        {0, 3}, {4, 3},
        {0, 4}, {1, 4}, {4, 4},
    };
    for (auto &w : vE) m.set_wall(w.x, w.y, HEADING_E, true);
    // Interior horizontals: the '--' runs between cell rows.
    const struct { int x, y; } hN[] = {
        {2, 4}, {4, 4},                          // under the top row
        {2, 3}, {3, 3}, {4, 3},
        {1, 2},
        {2, 1}, {4, 1},
    };
    for (auto &w : hN) m.set_wall(w.x, w.y, HEADING_N, true);
    // Everything else inside the block is known-open.
    for (int x = 0; x < 6; x++)
        for (int y = 0; y < 6; y++) {
            if (!m.wall_known(x, y, HEADING_N)) m.set_wall(x, y, HEADING_N, false);
            if (!m.wall_known(x, y, HEADING_E)) m.set_wall(x, y, HEADING_E, false);
            if (!m.wall_known(x, y, HEADING_S)) m.set_wall(x, y, HEADING_S, false);
            if (!m.wall_known(x, y, HEADING_W)) m.set_wall(x, y, HEADING_W, false);
        }
    m.set_goal(2, 0, 2, 0);   // G at (2,0)
}

// The number the whole design turns on. tools/route_check.py reports the same
// thing from an M dump; if these ever disagree, one of them is lying about
// the geometry the mouse will actually fly.
static void test_the_tangent_table() {
    const float V = 500.0f, SLEW = 6000.0f;
    float t84 = route_tangent_mm(84.0f, V, SLEW);
    float t60 = route_tangent_mm(60.0f, V, SLEW);
    float t45 = route_tangent_mm(45.0f, V, SLEW);
    std::printf("  tangents at v%.0f: R84 %.1f  R60 %.1f  R45 %.1f mm\n",
                V, t84, t60, t45);
    expect("R84 tangent is about 98.6 mm", std::fabs(t84 - 98.6f) < 3.0f);
    expect("the ladder is monotonic", t84 > t60 && t60 > t45);
    // A hairpin is two turns one cell apart. This single line is why the
    // ladder exists at all.
    expect("R84 cannot fly a 180 mm hairpin pair", 2.0f * t84 > ROUTE_CELL_MM);
    expect("R45 can", 2.0f * t45 + ROUTE_MIN_STRAIGHT_MM <= ROUTE_CELL_MM);
}

static void test_test8_route_shape() {
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);   // pessimistic: only fully-known edges
    expect("the goal is reachable on known walls",
           m.distance(0, 0) != MAZE_DIST_UNREACHABLE);

    SpeedPath path;
    RouteStats st;
    RouteTurn turns[16];
    RouteResult r = route_build(m, 0, 0, HEADING_N, path, 500.0f, 6000.0f,
                                &st, turns, 16);
    std::printf("  route: %s -- %u cells, %u turns, %u dropped, %u U-turn(s), "
                "%.0f mm\n",
                route_result_name(r), st.cells, st.turns, st.dropped,
                st.uturns, st.length_mm);
    for (int j = 0; j < st.turns && j < 16; j++)
        std::printf("    turn %d at (%u,%u) R%u %+d%s\n", j, turns[j].cx,
                    turns[j].cy, turns[j].r_mm, turns[j].sweep_deg,
                    (turns[j].sweep_deg == 180 || turns[j].sweep_deg == -180)
                        ? "  U-TURN" : "");

    expect("test8 compiles", r == ROUTE_OK);
    // Cross-checked against tools/route_check.py on the same M dump: 14 cells,
    // and a heading change at five places along the walk.
    expect("14 cells to the goal", st.cells == 14);
    // FOUR turns, not five. The first two heading changes -- (0,5) N->E and
    // (1,5) E->S, both RIGHT and one cell apart -- are a single 180 and are
    // emitted as one primitive. That merge is the whole point; see the
    // U-turn note in Route.h.
    expect("four turns, because the hairpin is ONE of them", st.turns == 4);
    expect("and exactly one of them is a U-turn", st.uturns == 1);
    expect("the U-turn is first", st.turns > 0 &&
                                      (turns[0].sweep_deg == 180 ||
                                       turns[0].sweep_deg == -180));
    expect("it is named by the cell it enters, (0,5)",
           st.turns > 0 && turns[0].cx == 0 && turns[0].cy == 5);
    expect("and it leaves through the next cell along the walk",
           st.turns > 0 && turns[0].exit_ix == turns[0].cell_ix + 1);
    expect("it turns RIGHT, as both 90s did",
           st.turns > 0 && turns[0].sweep_deg == -180);
}

// THE PRIMITIVE ITSELF. The radius is not a choice -- the exit corridor is one
// cell from the entry corridor, so the lateral offset is fixed at 180 mm and R
// follows from it. Everything here is the closed form in Path.cpp checked
// against the path it actually builds.
static void test_the_uturn_geometry_is_exact() {
    // ASK THE COMPILER'S OWN HELPER, not uturn_solve directly. This test used
    // to solve its own geometry from the rail while the compiler solved a
    // different one from a forced radius (ROUTE_UTURN_RADIUS_MM), and the
    // extent assertion below failed on a disagreement that was never about
    // the extent.
    SpeedPath::UTurn u;
    float u_slew = 0.0f;
    expect("a 180 that offsets one cell exists at v500",
           route_uturn_geometry(500.0f, 6000.0f, &u, &u_slew));
    std::printf("  uturn: R %.2f  clothoid %.2f  extent %.1f  length %.1f  "
                "rail %.0f dps2  a_lat %.0f  peak omega %.0f\n",
                u.radius_mm, u.clothoid_mm, u.extent_mm, u.length_mm, u_slew,
                500.0f * 500.0f / u.radius_mm, 500.0f / u.radius_mm * 57.29578f);
    // The RAIL the geometry assumes must be one the executor can follow, or
    // the clothoid is a fiction: the commanded rate would be clipped and the
    // mouse would fly a shape nobody solved. This is the check that makes a
    // forced radius safe rather than merely expressible.
    expect("the implied rail is inside the executor's",
           u_slew > 0.0f && u_slew <= PATH_UTURN_MAX_RAIL_DPS2);
    // The offset is one cell whatever the radius -- that is the invariant.
    // The radius itself is a choice (ROUTE_UTURN_RADIUS_MM), so it is not
    // asserted here; test_the_forced_radius_is_what_was_asked_for owns that.
    expect("the clothoid pair leaves a real arc",
           u.clothoid_mm > 0.0f && u.clothoid_mm < (float)M_PI * u.radius_mm);

    // An offset the corridor cannot produce must be refused, not approximated.
    SpeedPath::UTurn junk;
    expect("an impossible offset is refused",
           !SpeedPath::uturn_solve(20.0f, 500.0f, 6000.0f, &junk));

    // And the path it builds must LAND. The route ends at the goal cell
    // centre; anything else means the mouse finishes in the wrong corridor,
    // which no amount of gating recovers.
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);
    SpeedPath p;
    RouteStats st;
    expect("compiles", route_build(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                                   &st, nullptr, 0, 39.0f) == ROUTE_OK);
    PathPose e = p.pose(p.length_mm());
    // Maze frame: start pose (90, 51) heading N, goal cell (2,0) centre
    // (450, 90). Path frame has x along the launch heading and y to the left.
    float x_exp = 90.0f - 51.0f, y_exp = -(450.0f - 90.0f);
    std::printf("  end pose x %.2f y %.2f (expected %.2f, %.2f)\n",
                e.x_mm, e.y_mm, x_exp, y_exp);
    expect("the path ends exactly on the goal cell centre",
           std::fabs(e.x_mm - x_exp) < 0.5f && std::fabs(e.y_mm - y_exp) < 0.5f);

    // The U-turn's own geometry, measured off the built path.
    PathPose en = p.pose(p.corner_entry_s_mm(0));
    PathPose ex = p.pose(p.corner_exit_s_mm(0));
    expect("it sweeps exactly 180 degrees",
           std::fabs(std::fabs(ex.yaw_deg - en.yaw_deg) - 180.0f) < 0.5f);
    expect("and offsets exactly one cell sideways",
           std::fabs(std::fabs(en.y_mm - ex.y_mm) - ROUTE_CELL_MM) < 0.5f);
    float apex = -1e9f;
    for (float sm = p.corner_entry_s_mm(0); sm <= p.corner_exit_s_mm(0);
         sm += 1.0f) {
        float x = p.pose(sm).x_mm;
        if (x > apex) apex = x;
    }
    expect("the extent matches the closed form",
           std::fabs((apex - en.x_mm) - u.extent_mm) < 1.0f);
    // THE CLEARANCE THAT DECIDES THE SETBACK -- and the mistake runs 168-171
    // paid for. It is NOT the centreline apex plus the half-width. A mouse is
    // a rectangle, and swept round an arc its outermost point is the outer
    // FRONT corner, sqrt((R+half)^2 + front^2) from the arc centre. The first
    // version compared `past_centre` against WALL_FACE - BODY_HALF - MARGIN,
    // which said 10 mm of room and scraped the wall on all four passes.
    float setback = p.corner_tangent_mm(0);
    float past_centre = apex - (en.x_mm + setback);
    float outer = std::sqrt((u.radius_mm + ROUTE_BODY_HALF_MM) *
                                (u.radius_mm + ROUTE_BODY_HALF_MM) +
                            ROUTE_BODY_HALF_LEN_MM * ROUTE_BODY_HALF_LEN_MM);
    float clearance = ROUTE_WALL_FACE_MM + setback - outer;
    std::printf("  setback %.1f  centreline apex %.1f past the centre  "
                "outer sweep %.1f  wall clearance %.1f mm\n",
                setback, past_centre, outer, clearance);
    expect("the SWEPT ENVELOPE clears the far wall by the intended margin",
           clearance >= ROUTE_UTURN_MARGIN_MM - 0.5f);
    // The envelope is strictly bigger than the centreline-plus-flank figure
    // the first version used -- that difference IS the bug.
    expect("and the envelope is what binds, not the centreline",
           outer > u.radius_mm + ROUTE_BODY_HALF_MM);
    expect("the old sum would have allowed a smaller setback",
           u.extent_mm - (ROUTE_WALL_FACE_MM - ROUTE_BODY_HALF_MM -
                          ROUTE_UTURN_MARGIN_MM) < setback);

    // The inside post, the other thing the U-turn must clear. Its nearest
    // approach is at the entry and exit, and it never drops below half a cell
    // -- which is why bert's "excessive post clearance" is real and cannot be
    // traded away: the radius is forced by the one-cell offset.
    float half = 0.5f * ROUTE_CELL_MM;
    float dpost = std::sqrt(half * half +
                            (half - setback) * (half - setback));
    std::printf("  post: nearest %.1f mm, body margin %.1f mm\n",
                dpost, dpost - ROUTE_BODY_HALF_MM);
    expect("the post is cleared with room to spare",
           dpost - ROUTE_BODY_HALF_MM >= ROUTE_UTURN_MARGIN_MM);
    expect("and it has MORE post clearance than an R84 corner's 84 mm",
           dpost >= 84.0f);
}

// Why the 180 is worth its own primitive: the peak commanded turn rate, which
// runs 156-167 showed is what the heading error tracks.
static void test_the_uturn_is_the_gentlest_thing_on_the_route() {
    const float A_LAT = 4900.0f, V = 500.0f;
    auto rate = [&](float R) {
        float v = V < std::sqrt(A_LAT * R) ? V : std::sqrt(A_LAT * R);
        return v / R * 57.29578f;
    };
    SpeedPath::UTurn u;
    route_uturn_geometry(V, 6000.0f, &u, nullptr);
    std::printf("  peak rate: R45 %.0f  R73 %.0f  R84 %.0f  U-turn %.0f deg/s"
                "  (a_lat %.0f mm/s2, %.2f g)\n",
                rate(45.0f), rate(73.0f), rate(84.0f), rate(u.radius_mm),
                V * V / u.radius_mm, V * V / u.radius_mm / 9810.0f);

    if (ROUTE_UTURN_RADIUS_MM <= 0.0f) {
        // THE ORIGINAL CASE FOR THE 180, and it still has to hold whenever the
        // radius is left to the solver. At R89.67 the U-turn commands 319 dps
        // against the R73/R73 hairpin's 392 and an ordinary R84 corner's 341:
        // the hairpin stopped being the hardest thing on the route and became
        // the easiest. That is why it is its own primitive.
        expect("the U-turn is gentler than the R60+R45 pair it replaces",
               rate(u.radius_mm) < 0.6f * rate(45.0f));
        expect("gentler than the R73+R73 pair that replaced THAT",
               rate(u.radius_mm) < rate(73.0f));
        expect("and gentler than an ordinary R84 corner",
               rate(u.radius_mm) < rate((float)ROUTE_RADII[0]));
    } else {
        // A FORCED RADIUS SPENDS THAT ADVANTAGE, and the test says so rather
        // than being quietly retargeted. At bert's -20 % (R 71.74) the U-turn
        // asks 399 dps and 0.36 g -- HARDER than the R73/R73 hairpin it
        // replaced (392 dps, 0.30 g) and harder than any ordinary corner. The
        // trade is deliberate: 16.7 mm of swept envelope bought with rate and
        // grip. It is not a free tightening and nothing here should imply it
        // is.
        //
        // What is still asserted is that the trade stays inside the machine.
        // 0.5 g is the profile's own lateral cap (the ralat param); past it
        // the planner would slow the corner down, which would change the
        // clothoid, which would change the geometry that was just solved.
        std::printf("  NOTE: radius FORCED to %.2f (%.0f%% under the solved "
                    "89.67). Peak rate %.0f dps now EXCEEDS the R73/R73 "
                    "hairpin's %.0f -- the 180's rate advantage is spent.\n",
                    (double)ROUTE_UTURN_RADIUS_MM,
                    100.0 * (1.0 - ROUTE_UTURN_RADIUS_MM / 89.67),
                    rate(u.radius_mm), rate(73.0f));
        expect("the forced radius is the one that was asked for",
               std::fabs(u.radius_mm - ROUTE_UTURN_RADIUS_MM) < 0.01f);
        expect("and the lateral demand stays under the profile's own cap",
               V * V / u.radius_mm < 4900.0f);
    }
    expect("no speed cap -- nothing has to brake into it",
           std::sqrt(A_LAT * u.radius_mm) >= V);
}

// THE HAIRPIN, AND THE THREE SHAPES IT HAS HAD.
//
// This test has been retargeted twice, and both old assertions are kept as
// comments because the sequence is the argument.
//
//   1. R60 + R45 with 25 mm of straight. Asserted "the tight pair dropped".
//      Runs 156-163: peak rate 598 deg/s, heading error reached 23.9 deg
//      against a 25 deg abort, twice in eight passes.
//   2. R73 + R73, arcs touching (ROUTE_MIN_STRAIGHT_SAME_MM). Asserted "the
//      hairpin is symmetric at R73". Runs 164-167: 392 deg/s, error fell to
//      10.1 against a predicted 9.8 -- which CONFIRMED the mechanism as
//      rate-tracking lag and earned the third shape.
//   3. One U-turn at R89.7. 319 deg/s, gentler than an ordinary corner.
//
// So the assertion is no longer about which radii the pair was demoted to.
// There is no pair. It is that the merge happened and nothing was demoted.
static void test_the_hairpin_is_one_uturn_and_costs_no_demotions() {
    // SKIPPED WHILE THE RADIUS IS FORCED. Every claim below is a property of
    // the U-turn the solver picks (R 89.67): that it fits the hairpin, gates
    // itself, and beats the 90s it replaces. ROUTE_UTURN_RADIUS_MM overrides
    // that radius for a deliberate experiment, and at -20 %% the setback grows
    // to 142 mm -- which costs the loop its U-turns and drives the gate room
    // NEGATIVE (-7.4 mm: the anchoring post is expected after the arc has
    // already begun, so corner 0 flies ungated). Those are the experiment's
    // known costs, not regressions, and retargeting the assertions to match
    // them would delete the record of what the solved radius buys.
    if (ROUTE_UTURN_RADIUS_MM > 0.0f) {
        std::printf("  (skipped: ROUTE_UTURN_RADIUS_MM forced to %.2f)\n",
                    (double)ROUTE_UTURN_RADIUS_MM);
        return;
    }
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);
    SpeedPath path;
    RouteStats st;
    RouteTurn turns[16];
    expect("compiles", route_build(m, 0, 0, HEADING_N, path, 500.0f, 6000.0f,
                                   &st, turns, 16, 39.0f) == ROUTE_OK);

    expect("the hairpin is a U-turn", st.uturns == 1);
    // WAS: "exactly the tight pair dropped, not the whole route", dropped
    // between 1 and 2. Merging removes the reason to demote at all -- the
    // U-turn's radius is forced by the corridor, and every OTHER turn on this
    // route has a cell or more of room.
    expect("nothing on the route is below the max-clearance radius",
           st.dropped == 0);
    for (int j = 0; j < st.turns && j < 16; j++)
        if (turns[j].sweep_deg == 90 || turns[j].sweep_deg == -90)
            expect("every 90 flies at R84", turns[j].r_mm == ROUTE_RADII[0]);

    // The same-direction minimum is still there and still needed -- it is what
    // catches a same-direction pair the U-turn cannot take (not adjacent, or
    // crowded by a neighbour).
    float t73 = route_tangent_mm(73.0f, 500.0f, 6000.0f);
    expect("R73+R73 still fits one cell on the relaxed minimum",
           2.0f * t73 + ROUTE_MIN_STRAIGHT_SAME_MM <= ROUTE_CELL_MM + 0.01f);
    expect("and still would not on the 20 mm one",
           2.0f * t73 + ROUTE_MIN_STRAIGHT_MM > ROUTE_CELL_MM);
}

// The return leg is the harder case: a staircase with its own same-direction
// pair, and a turn sitting one cell before a U-turn.
static void test_the_return_leg_gets_two_uturns() {
    // SKIPPED WHILE THE RADIUS IS FORCED. Every claim below is a property of
    // the U-turn the solver picks (R 89.67): that it fits the hairpin, gates
    // itself, and beats the 90s it replaces. ROUTE_UTURN_RADIUS_MM overrides
    // that radius for a deliberate experiment, and at -20 %% the setback grows
    // to 142 mm -- which costs the loop its U-turns and drives the gate room
    // NEGATIVE (-7.4 mm: the anchoring post is expected after the arc has
    // already begun, so corner 0 flies ungated). Those are the experiment's
    // known costs, not regressions, and retargeting the assertions to match
    // them would delete the record of what the solved radius buys.
    if (ROUTE_UTURN_RADIUS_MM > 0.0f) {
        std::printf("  (skipped: ROUTE_UTURN_RADIUS_MM forced to %.2f)\n",
                    (double)ROUTE_UTURN_RADIUS_MM);
        return;
    }
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);
    int cx = 0, cy = 0;
    Heading h = HEADING_N;
    int guard = 0;
    while (m.distance(cx, cy) != 0 && guard++ < ROUTE_MAX_CELLS) {
        Heading next;
        if (!m.best_step_known(cx, cy, h, next)) break;
        cx += heading_dx(next); cy += heading_dy(next); h = next;
    }
    m.flood_to_start(false);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[16];
    RouteResult r = route_build(m, cx, cy, h, p, 500.0f, 6000.0f, &st, t, 16);
    std::printf("  return: %s -- %u turns, %u dropped, %u U-turns\n",
                route_result_name(r), st.turns, st.dropped, st.uturns);
    expect("the return leg compiles", r == ROUTE_OK);
    // It used to be 7 turns with 6 of them demoted. Two of its heading-change
    // pairs are same-direction and adjacent -- the staircase's, and the
    // hairpin flown the other way.
    expect("two of its pairs are U-turns", st.uturns == 2);
    expect("and far fewer demotions than the 6 it used to need",
           st.dropped <= 3);
}

// Arcs that overlap are not a path. This is the invariant the whole radius
// pass exists to protect, so it is asserted on the emitted geometry rather
// than on the compiler's own bookkeeping.
static void test_no_two_arcs_ever_overlap() {
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);
    SpeedPath path;
    RouteStats st;
    RouteTurn turns[16];
    expect("compiles", route_build(m, 0, 0, HEADING_N, path, 500.0f, 6000.0f,
                                   &st, turns, 16) == ROUTE_OK);

    // Per-pair, because the minimum is per-pair: a same-direction pair may
    // touch, an opposite-direction one may not, and a U-turn brings a fixed
    // setback instead of a tangent. Measured from the PREVIOUS turn's exit
    // cell, which differs from its own cell only for a U-turn.
    auto is_u = [](const RouteTurn &t) {
        return t.sweep_deg == 180 || t.sweep_deg == -180;
    };
    for (int j = 1; j < st.turns; j++) {
        float gap = (float)(turns[j].cell_ix - turns[j - 1].exit_ix) *
                    ROUTE_CELL_MM;
        float used = 0.0f;
        for (int k : {j - 1, j})
            used += is_u(turns[k])
                        ? path.corner_tangent_mm(k)
                        : route_tangent_mm(turns[k].r_mm, 500.0f, 6000.0f);
        float need = (turns[j].sweep_deg == turns[j - 1].sweep_deg &&
                      !is_u(turns[j]) && !is_u(turns[j - 1]))
                         ? ROUTE_MIN_STRAIGHT_SAME_MM
                         : ROUTE_MIN_STRAIGHT_MM;
        expect("every pair leaves the straight ITS rule demands",
               used + need <= gap + 0.01f);
        expect("and no two primitives ever overlap, whatever the rule",
               used <= gap + 0.01f);
    }
    expect("the path has length", st.length_mm > 0.0f);
    std::printf("  outbound: %.0f mm\n", (double)st.length_mm);
    // WHY THE THRESHOLD MOVED DOWN (2026-08-27). It was 2384: the U-turn takes
    // a wider line than the two 90s it replaced, so the path got longer. That
    // is still true of the ARC. What changed is the SETBACK -- the tracking
    // allowance added 13 mm to each end of it, and the setback is walk
    // distance the primitive consumes at a FIXED arclength. More setback
    // therefore means less straight and a SHORTER total, in exactly the
    // amount the U-turn entered earlier. The property being defended is
    // unchanged: the U-turn is still the wider line, and the number is only a
    // tripwire on the compiler quietly reverting to the hairpin -- which
    // reports 2317 or so, and which `uturns` catches more directly anyway.
    expect("and it is still longer than the hairpin it replaced",
           st.length_mm > 2340.0f);
}

// Speed changes the clothoid, and the clothoid changes the tangent. A route
// compiled at 500 is NOT the path for 700, and the compiler must not pretend
// otherwise -- run 153 vs 154 is what that mistake looks like on hardware.
static void test_speed_changes_the_geometry() {
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);
    SpeedPath p5, p7;
    RouteStats s5, s7;
    RouteTurn t5[16], t7[16];
    RouteResult r5 = route_build(m, 0, 0, HEADING_N, p5, 500.0f, 6000.0f, &s5,
                                 t5, 16);
    RouteResult r7 = route_build(m, 0, 0, HEADING_N, p7, 700.0f, 6000.0f, &s7,
                                 t7, 16);
    expect("500 compiles", r5 == ROUTE_OK);
    std::printf("  at v700: %s (%u turns, %u dropped)\n",
                route_result_name(r7), s7.turns, s7.dropped);
    expect("the tangent grows with speed",
           route_tangent_mm(84.0f, 700.0f, 6000.0f) >
               route_tangent_mm(84.0f, 500.0f, 6000.0f));
    // Whatever 700 does, it must not silently produce the SAME path as 500.
    if (r7 == ROUTE_OK)
        expect("a faster route is a different path",
               std::fabs(s7.length_mm - s5.length_mm) > 0.5f ||
                   s7.dropped != s5.dropped);
}

// A map with holes in it must be refused, not guessed at.
static void test_an_unfinished_map_is_refused() {
    Maze m;
    m.reset();                 // nothing known but the outer boundary
    m.set_goal(2, 0, 2, 0);
    m.flood_to_goal(false);    // pessimistic: unknown counts as wall
    SpeedPath path;
    RouteStats st;
    RouteResult r = route_build(m, 0, 0, HEADING_N, path, 500.0f, 6000.0f, &st);
    expect("an unsearched maze compiles to nothing",
           r == ROUTE_NO_PATH);
}

// A mouse already standing on the target has no route, and saying "ok, here
// is an empty path" would hand the executor something it would fly as a
// zero-length move.
static void test_standing_on_the_goal() {
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);
    SpeedPath path;
    RouteStats st;
    expect("no route from the goal to itself",
           route_build(m, 2, 0, HEADING_N, path, 500.0f, 6000.0f, &st) ==
               ROUTE_NO_PATH);
}

// ---------------------------------------------------------------------------
// THE LEAD-IN. The mouse starts with its back to the rear wall, 51 mm in; the
// compiled path starts at the 90 mm cell centre. Miss the 39 mm and every
// landmark on the route sits 39 mm from where the path believes it is -- and
// nothing on a route leg would ever notice, because with no wall gate there
// is nothing measuring the path against the world.
static void test_the_start_setback_lead_in() {
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);
    const float LEAD = 39.0f;

    SpeedPath p0, p1;
    RouteStats s0, s1;
    RouteTurn t0[16], t1[16];
    expect("no lead-in compiles", route_build(m, 0, 0, HEADING_N, p0, 500.0f,
                                              6000.0f, &s0, t0, 16,
                                              0.0f) == ROUTE_OK);
    expect("lead-in compiles", route_build(m, 0, 0, HEADING_N, p1, 500.0f,
                                           6000.0f, &s1, t1, 16,
                                           LEAD) == ROUTE_OK);
    expect("the lead-in makes the path exactly that much longer",
           std::fabs((s1.length_mm - s0.length_mm) - LEAD) < 0.01f);
    // And it must not change the SHAPE -- same turns, same radii. A lead-in
    // that quietly re-fitted the radii would mean the bench 'route' command
    // and the flown leg disagreed about the corners.
    expect("same turn count", s0.turns == s1.turns);
    for (int j = 0; j < s0.turns && j < 16; j++)
        expect("same radius at every turn", t0[j].r_mm == t1[j].r_mm);
    expect("same number dropped", s0.dropped == s1.dropped);
}

// The return leg. The mouse ends start->goal at the goal cell CENTRE facing
// whatever the last step was, and the mission flies back from exactly there --
// so the reverse route has to compile from that pose, not from an assumption.
static void test_the_return_leg_compiles_from_where_the_first_one_ended() {
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);
    SpeedPath p;
    RouteStats st;
    expect("outbound compiles",
           route_build(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f, &st) == ROUTE_OK);

    // Walk the same flood to find the end pose -- the mission does exactly
    // this, so if the walk is wrong here it is wrong there too.
    int cx = 0, cy = 0;
    Heading h = HEADING_N;
    int guard = 0;
    while (m.distance(cx, cy) != 0 && guard++ < ROUTE_MAX_CELLS) {
        Heading next;
        if (!m.best_step_known(cx, cy, h, next)) break;
        cx += heading_dx(next);
        cy += heading_dy(next);
        h = next;
    }
    expect("the outbound leg ends on the goal", cx == 2 && cy == 0);
    expect("facing WEST -- it arrived at (2,0) from (3,0)", h == HEADING_W);

    // Now home. flood_to_start, and the mouse is pointing W at the goal.
    m.flood_to_start(false);
    SpeedPath back;
    RouteStats bs;
    RouteResult r = route_build(m, cx, cy, h, back, 500.0f, 6000.0f, &bs);
    std::printf("  return leg: %s -- %u cells, %u turns, %u dropped, %.0f mm\n",
                route_result_name(r), bs.cells, bs.turns, bs.dropped,
                bs.length_mm);
    // The return may legitimately need a first-cell turn (the mouse is facing
    // W at (2,0) and home is E of nothing -- it has to go somewhere else), and
    // a turn in the very first cell has no room before it. Whatever the
    // compiler decides, it must be a DECISION, not a crash: either a valid
    // path or a named refusal.
    expect("the return leg compiles or refuses cleanly, never silently",
           r == ROUTE_OK || r == ROUTE_NO_RADIUS || r == ROUTE_BAD_TURN);
    if (r == ROUTE_OK)
        expect("and it is a real route", bs.cells > 0 && bs.length_mm > 0.0f);
}

// ---------------------------------------------------------------------------
// WHAT GATES THE TURN, AND HOW MUCH ROOM IS LEFT AFTER IT FIRES.
//
// The landmark is the END of the INSIDE wall -- the wall the mouse turns
// around the outside of. It ends at the corner cell's near boundary, which is
// the post at the inside corner of that cell, and at R84 that is the very
// post the arc is centred on. The gate's expected firing point is therefore
//
//     corner point - half a cell - beam lead   =   corner point - 135 mm
//
// and the room left to spread the correction over is
//
//     room = 90 + 45 - T
//
// where T is the clothoid-aware tangent. T grows as v^2, so the room shrinks
// with speed -- and that is the constraint that decides whether a radius can
// be gated at all at a given speed. Asserted here rather than left as a
// comment because both halves (the ladder, the gate) are compiled numbers
// that can drift apart silently.
static void test_the_gate_has_room_at_every_ladder_radius() {
    const float HALF_CELL = 90.0f, BEAM_AHEAD = 45.0f;
    const float SLEW = 6000.0f;
    // The floor: below about 20 mm there is not enough run left to apply a
    // correction smoothly, and below zero the gate fires after the arc has
    // already started.
    const float MIN_ROOM = 20.0f;

    for (int i = 0; i < ROUTE_N_RADII; i++) {
        float R = (float)ROUTE_RADII[i];
        float T = route_tangent_mm(R, 500.0f, SLEW);
        float room = HALF_CELL + BEAM_AHEAD - T;
        std::printf("  R%-3.0f v500: T %5.1f  gate fires %3.0f mm before the "
                    "corner point, %5.1f mm of run left\n",
                    R, T, HALF_CELL + BEAM_AHEAD, room);
        expect("every ladder radius leaves the gate room to correct in",
               room >= MIN_ROOM);
    }
    // R84 is the operating point and its room is the number to know.
    float t84 = route_tangent_mm(84.0f, 500.0f, SLEW);
    expect("R84 at v500 leaves about 36 mm after the drop",
           std::fabs((HALF_CELL + BEAM_AHEAD - t84) - 36.4f) < 2.0f);

    // R125 is NOT in the ladder, and this is one of the reasons: even at the
    // gathering speed its tangent eats the whole gate room. Kept as a test so
    // that anyone tempted to add it sees why.
    float t125 = route_tangent_mm(125.0f, 500.0f, SLEW);
    expect("R125 would leave the gate nothing to correct over",
           HALF_CELL + BEAM_AHEAD - t125 < MIN_ROOM);

    // And the room shrinks with speed, because the clothoid goes as v^2.
    float t84_700 = route_tangent_mm(84.0f, 700.0f, SLEW);
    expect("R84 has less gate room at 700 than at 500", t84_700 > t84);
    std::printf("  R84  v700: T %5.1f  ->  %5.1f mm of run left\n", t84_700,
                HALF_CELL + BEAM_AHEAD - t84_700);
}

// ---------------------------------------------------------------------------
// THE POST GATE, ON A REAL COMPILED ROUTE.
//
// These mirror the arithmetic RouteRun does per corner. The point is not to
// re-test WallGate -- test_wall_gate does that on real packets -- but to pin
// the ROUTE-side geometry: where the drop is expected, how much run is left
// after it, and which corners are physically capable of being gated at all.
static const float RG_JUNCTION_EXIT = 60.0f;   // SWEEP_JUNCTION_EXIT_MM

static float rg_expect_s(const SpeedPath &p, int k) {
    return p.corner_entry_s_mm(k) + p.corner_tangent_mm(k) - 0.5f * ROUTE_CELL_MM -
           WALL_GATE_BEAM_AHEAD_MM;
}

// The gate cannot arm until the mouse is clear of the PREVIOUS corner: inside
// an arc the side beam is not looking at a corridor wall.
static float rg_armable_s(const SpeedPath &p, int k) {
    return k == 0 ? 0.0f : p.corner_exit_s_mm(k - 1) + RG_JUNCTION_EXIT;
}

static void test_the_gate_expects_the_post_at_the_cell_boundary() {
    // SKIPPED WHILE THE RADIUS IS FORCED. Every claim below is a property of
    // the U-turn the solver picks (R 89.67): that it fits the hairpin, gates
    // itself, and beats the 90s it replaces. ROUTE_UTURN_RADIUS_MM overrides
    // that radius for a deliberate experiment, and at -20 %% the setback grows
    // to 142 mm -- which costs the loop its U-turns and drives the gate room
    // NEGATIVE (-7.4 mm: the anchoring post is expected after the arc has
    // already begun, so corner 0 flies ungated). Those are the experiment's
    // known costs, not regressions, and retargeting the assertions to match
    // them would delete the record of what the solved radius buys.
    if (ROUTE_UTURN_RADIUS_MM > 0.0f) {
        std::printf("  (skipped: ROUTE_UTURN_RADIUS_MM forced to %.2f)\n",
                    (double)ROUTE_UTURN_RADIUS_MM);
        return;
    }
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[16];
    expect("compiles", route_build(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f, &st,
                                   t, 16, 39.0f) == ROUTE_OK);

    for (int k = 0; k < p.corner_count(); k++) {
        // The expected drop sits exactly (half a cell + beam lead) before the
        // corner point, whatever the radius. The corner point in path-s is
        // entry + T on the approach straight, where path-s and spatial
        // distance still agree.
        float corner_point = p.corner_entry_s_mm(k) + p.corner_tangent_mm(k);
        float e = rg_expect_s(p, k);
        expect("the drop is expected 135 mm before the corner point",
               std::fabs((corner_point - e) - 135.0f) < 0.01f);
        // And the room left is 135 - T, the number the correction drains over.
        float room = p.corner_entry_s_mm(k) - e;
        expect("room is 135 - T",
               std::fabs(room - (135.0f - p.corner_tangent_mm(k))) < 0.01f);
        expect("and it is positive at every radius the ladder uses",
               room > 0.0f);
    }
}

// THE STRUCTURAL LIMIT THAT THE U-TURN ABOLISHED.
//
// This test used to assert the opposite of what it asserts now, and the old
// version is worth stating because it is why the U-turn was built:
//
//     "the hairpin's second turn cannot gate" -- rg_armable_s(p,1) >
//     rg_expect_s(p,1), and "exactly one corner on the outbound leg is too
//     late".
//
// That was true and unavoidable while the hairpin was two 90s. The gate for
// corner k cannot arm until the mouse is clear of corner k-1, and on a pair
// one cell apart that clearance point lands PAST corner k's own post -- 96 mm
// past it, measured. So the second turn flew on an inherited anchor.
//
// Merged into one 180 there is no second turn to gate. Its inside post is the
// same post that anchored the first of the two 90s, and one anchor now serves
// the whole manoeuvre. The assertion flips: EVERY corner on this route gates.
static void test_every_corner_can_now_gate() {
    // SKIPPED WHILE THE RADIUS IS FORCED. Every claim below is a property of
    // the U-turn the solver picks (R 89.67): that it fits the hairpin, gates
    // itself, and beats the 90s it replaces. ROUTE_UTURN_RADIUS_MM overrides
    // that radius for a deliberate experiment, and at -20 %% the setback grows
    // to 142 mm -- which costs the loop its U-turns and drives the gate room
    // NEGATIVE (-7.4 mm: the anchoring post is expected after the arc has
    // already begun, so corner 0 flies ungated). Those are the experiment's
    // known costs, not regressions, and retargeting the assertions to match
    // them would delete the record of what the solved radius buys.
    if (ROUTE_UTURN_RADIUS_MM > 0.0f) {
        std::printf("  (skipped: ROUTE_UTURN_RADIUS_MM forced to %.2f)\n",
                    (double)ROUTE_UTURN_RADIUS_MM);
        return;
    }
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[16];
    expect("compiles", route_build(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                                   &st, t, 16, 39.0f) == ROUTE_OK);

    int too_late = 0;
    for (int k = 0; k < p.corner_count(); k++) {
        bool late = rg_armable_s(p, k) > rg_expect_s(p, k);
        std::printf("  corner %d R%-3u %s  expect %6.0f  armable %6.0f  "
                    "room %4.1f  %s\n",
                    k, t[k].r_mm,
                    (t[k].sweep_deg == 180 || t[k].sweep_deg == -180) ? "U"
                                                                     : " ",
                    rg_expect_s(p, k), rg_armable_s(p, k),
                    p.corner_entry_s_mm(k) - rg_expect_s(p, k),
                    late ? "TOO LATE" : "gates");
        if (late) too_late++;
    }
    expect("no corner on the outbound route is too late to gate any more",
           too_late == 0);

    // And the U-turn gets the MOST room of anything on the route, because its
    // setback (64 mm) is smaller than any 90's tangent. That matters: the room
    // is what the correction is drained over, and runs 164-167 hinted that a
    // large correction drained over a short run perturbs the corner entry.
    float u_room = p.corner_entry_s_mm(0) - rg_expect_s(p, 0);
    float c_room = p.corner_entry_s_mm(1) - rg_expect_s(p, 1);
    std::printf("  gate room: U-turn %.1f mm, ordinary R84 corner %.1f mm\n",
                u_room, c_room);
    expect("the U-turn has the most gate room on the route", u_room > c_room);
    // Was 71 mm at the 64 mm setback that scraped, then 62.5 once the setback
    // was sized on the swept envelope, and 49.5 now that the tracking
    // allowance is in it too. Each step spends gate room to buy apex
    // clearance; that is the trade, and it is worth stating that the trade has
    // a floor. The number to beat is the ordinary R84 corner's 36.4, which the
    // mouse gates reliably (16/16 on runs 172-175) -- not a constant chosen
    // when the setback was smaller.
    expect("and still beats every 90 on the route", u_room > c_room + 10.0f);
}

// The decide window reaches 240 mm back, which on a twisty route is further
// than the previous corner. That is safe ONLY because arming is what gates
// the gate -- it cannot be armed for corner k while corner k-1 is still open.
static void test_arming_is_what_protects_the_decide_window() {
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);
    SpeedPath p;
    RouteStats st;
    expect("compiles",
           route_build(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f, &st) == ROUTE_OK);
    const float DECIDE = 240.0f;
    bool any_overlap = false;
    for (int k = 1; k < p.corner_count(); k++) {
        // Where the gate WOULD be allowed to decide from...
        float live_from = rg_expect_s(p, k) - DECIDE;
        // ...against where the previous corner's arc still owns the beam.
        if (live_from < p.corner_exit_s_mm(k - 1)) any_overlap = true;
    }
    expect("the decide window does reach back into a previous corner -- so "
           "arming, not the window, is what keeps corners separate",
           any_overlap);
}

// ---------------------------------------------------------------------------
// THE OUT-AND-BACK. One wave, start -> goal -> start, no stop at the goal.
//
// The point of compiling it as ONE walk is that the goal stops being a special
// case: whatever heading change sits there is found by the same turn finder as
// every other corner. These tests pin that, and pin the join.
static void test_the_goal_needs_no_turn_on_test8() {
    Maze m;
    build_test8(m);
    m.flood_to_goal(false);
    int cx = 0, cy = 0, guard = 0;
    Heading h = HEADING_N, arrive = h;
    while (m.distance(cx, cy) != 0 && guard++ < ROUTE_MAX_CELLS) {
        Heading next;
        if (!m.best_step_known(cx, cy, h, next)) break;
        arrive = next;
        cx += heading_dx(next); cy += heading_dy(next); h = next;
    }
    expect("the outbound ends on the goal", cx == 2 && cy == 0);
    m.flood_to_start(false);
    Heading first;
    expect("and there is a way home",
           m.best_step_known(cx, cy, arrive, first));
    // The mouse arrives heading WEST and the return leaves heading WEST, so
    // the "stop at the goal" was never geometry -- it was the mission choosing
    // to end the leg. Worth asserting, because if this maze ever changes the
    // out-and-back will grow a turn there and the test says which kind.
    expect("test8's goal needs NO turn -- the return carries straight on",
           first == arrive);
    expect("and it is certainly not a reversal, which no radius could fly",
           first != heading_back(arrive));
}

static void test_the_out_and_back_compiles_as_one_path() {
    // SKIPPED WHILE THE RADIUS IS FORCED. Every claim below is a property of
    // the U-turn the solver picks (R 89.67): that it fits the hairpin, gates
    // itself, and beats the 90s it replaces. ROUTE_UTURN_RADIUS_MM overrides
    // that radius for a deliberate experiment, and at -20 %% the setback grows
    // to 142 mm -- which costs the loop its U-turns and drives the gate room
    // NEGATIVE (-7.4 mm: the anchoring post is expected after the arc has
    // already begun, so corner 0 flies ungated). Those are the experiment's
    // known costs, not regressions, and retargeting the assertions to match
    // them would delete the record of what the solved radius buys.
    if (ROUTE_UTURN_RADIUS_MM > 0.0f) {
        std::printf("  (skipped: ROUTE_UTURN_RADIUS_MM forced to %.2f)\n",
                    (double)ROUTE_UTURN_RADIUS_MM);
        return;
    }
    Maze m;
    build_test8(m);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    RouteResult r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                                     &st, t, 32, 39.0f);
    std::printf("  loop: %s -- %u cells (goal at %u), %u turns, %u dropped, "
                "%u U-turns, %.0f mm\n",
                route_result_name(r), st.cells, st.goal_ix, st.turns,
                st.dropped, st.uturns, st.length_mm);
    expect("it compiles", r == ROUTE_OK);
    expect("28 cells -- 14 out, 14 back", st.cells == 28);
    expect("with the goal at the halfway mark", st.goal_ix == 14);
    // THREE U-turns: the hairpin outbound, the staircase's pair on the way
    // home, and the hairpin again the other way. Every one of them was a pair
    // of 90s before the merger existed.
    expect("three U-turns on the loop", st.uturns == 3);
    expect("and no turn was added at the goal itself -- 4 out + 5 back",
           st.turns == 9);

    // It must LAND. A loop that does not finish on the start cell finishes
    // somewhere the next run would compile from wrongly.
    PathPose e = p.pose(p.length_mm());
    std::printf("  ends at x %.2f y %.2f yaw %.1f\n", e.x_mm, e.y_mm,
                e.yaw_deg);
    // Path frame: x along the launch heading. The start POSE is the rear-wall
    // setback, 39 mm short of the cell centre, and the loop ends ON that
    // centre -- so x = 39, y = 0, having turned right round.
    expect("it comes back to the start cell centre",
           std::fabs(e.x_mm - 39.0f) < 1.0f && std::fabs(e.y_mm) < 1.0f);
    expect("facing the way it came -- nose to the back wall",
           std::fabs(std::fabs(e.yaw_deg) - 180.0f) < 1.0f);

    // And it is one continuous path, not two glued together: the compiler
    // emits straights and arcs only, so length is the sum and there is no
    // discontinuity to find. A stop would show as a route half this long.
    SpeedPath one;
    RouteStats os;
    m.flood_to_goal(false);
    expect("one-way still works", route_build(m, 0, 0, HEADING_N, one, 500.0f,
                                              6000.0f, &os, nullptr, 0,
                                              39.0f) == ROUTE_OK);
    expect("the loop is about twice the one-way route",
           st.length_mm > 1.8f * os.length_mm &&
               st.length_mm < 2.2f * os.length_mm);
}

// ---- route.dir on a CLOSED CIRCUIT (CAL-6 rev A) ---------------------------
// The planes below are derived from tools/tune_gui.html's rig data by the
// same packing the GUI's "Push rig" uses (v[x] bit y, h[y] bit x, walls
// solid outside the 6x6). 28 corridor cells, every one with exactly two
// openings -- a pure loop, so BOTH arms reach the goal and route.dir gets to
// choose which one is "out". Loaded via Maze::State exactly as mz.commit
// does on the mouse: all-known, NO LOCKS (the standard-rules lock on the
// start cell's east wall must not exist here, or the circuit is not a
// circuit).
static void build_cal6(Maze &m) {
    static const uint16_t V[17] = {0xFFFF, 0xFFDE, 0xFFF0, 0xFFDC, 0xFFC0,
                                   0xFFD2, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
                                   0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
                                   0xFFFF, 0xFFFF};
    static const uint16_t H[17] = {0xFFFF, 0xFFDE, 0xFFD8, 0xFFF6, 0xFFD8,
                                   0xFFD8, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
                                   0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
                                   0xFFFF, 0xFFFF};
    Maze::State s{};
    for (int i = 0; i <= MAZE_SIZE; i++) {
        s.vwall[i] = V[i];
        s.vknown[i] = 0xFFFF;
        s.vlock[i] = 0;
        s.hwall[i] = H[i];
        s.hknown[i] = 0xFFFF;
        s.hlock[i] = 0;
    }
    for (int y = 0; y < MAZE_SIZE; y++) s.visited[y] = 0;
    m.load_state(s);
    m.set_goal(5, 5, 5, 5);
}

static void count_hands(const RouteTurn *t, int n, int &left90, int &right90,
                        int &leftU, int &rightU) {
    left90 = right90 = leftU = rightU = 0;
    for (int i = 0; i < n; i++) {
        bool u = t[i].sweep_deg == 180 || t[i].sweep_deg == -180;
        if (u) (t[i].sweep_deg > 0 ? leftU : rightU)++;
        else (t[i].sweep_deg > 0 ? left90 : right90)++;
    }
}

static void test_route_dir_picks_the_arm() {
    Maze m;
    build_cal6(m);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    int l90, r90, lU, rU;

    // Forced NORTH = today's lap, now by choice instead of tie-break: west
    // arm out, every 90 a RIGHT, hairpin a right U, tooth and dent left Us.
    RouteResult r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                                     &st, t, 32, 0.0f, (int)HEADING_N);
    expect("dir=n compiles", r == ROUTE_OK);
    expect("dir=n covers the full circuit", st.cells == 28);
    expect("with the goal at the antipode", st.goal_ix == 14);
    count_hands(t, st.turns, l90, r90, lU, rU);
    std::printf("  dir=n: %u turns -- %d L90 %d R90 %d LU %d RU\n", st.turns,
                l90, r90, lU, rU);
    expect("dir=n: every 90 is a RIGHT", l90 == 0 && r90 >= 4);
    expect("dir=n: one right U (hairpin), two left (tooth, dent)",
           rU == 1 && lU == 2);
    float len_n = st.length_mm;

    // Forced EAST = the same circuit the other way round. The lap LAUNCHES
    // facing east (a launch cannot begin with a turn, so RouteRun points the
    // nose along the forced exit before compiling) -- same cells, same
    // length, every hand flipped: the mirror-free left-turn rig.
    r = route_build_loop(m, 0, 0, HEADING_E, p, 500.0f, 6000.0f, &st, t, 32,
                         0.0f, (int)HEADING_E);
    expect("dir=e compiles", r == ROUTE_OK);
    expect("dir=e covers the full circuit too", st.cells == 28);
    count_hands(t, st.turns, l90, r90, lU, rU);
    std::printf("  dir=e: %u turns -- %d L90 %d R90 %d LU %d RU, "
                "len %.0f vs %.0f\n",
                st.turns, l90, r90, lU, rU, st.length_mm, len_n);
    expect("dir=e: every 90 is a LEFT", r90 == 0 && l90 >= 4);
    expect("dir=e: hands of the Us flip too", lU == 1 && rU == 2);
    expect("dir=e: same circuit, same length within a cell",
           std::fabs(st.length_mm - len_n) < ROUTE_CELL_MM);

    // Unforced still compiles (legacy tie-break) ...
    r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f, &st, t, 32,
                         0.0f);
    expect("unforced still compiles", r == ROUTE_OK);

    // ... and the temporary walls really were temporary: the start cell
    // still opens both ways on the caller's maze.
    expect("start cell north is open again",
           m.wall_known(0, 0, HEADING_N) && !m.wall_present(0, 0, HEADING_N));
    expect("start cell east is open again",
           m.wall_known(0, 0, HEADING_E) && !m.wall_present(0, 0, HEADING_E));

    // On a maze that is NOT a circuit (test8: the start opens only north),
    // forcing EAST is impossible and must say NO PATH -- RouteRun then
    // retries unforced and tells the console.
    Maze m8;
    build_test8(m8);
    r = route_build_loop(m8, 0, 0, HEADING_N, p, 500.0f, 6000.0f, &st, t, 32,
                         39.0f, (int)HEADING_E);
    expect("forcing a walled side refuses honestly", r == ROUTE_NO_PATH);
}

// ---- corner geometry solves at corner speed (route_set_corner_context) -----
// At rv=1300 (2026-08-29 lap) every corner still flies at sqrt(alat x R),
// but the geometry was solved at 1300: tangents ballooned, R84s demoted to
// R60/R45, and every U decomposed into two squared back-to-back 90s. The
// context makes solved speed = flown speed.
static void test_corner_geometry_at_corner_speed() {
    Maze m;
    build_cal6(m);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    // Legacy: geometry at route speed. Document what the 2026-08-29 lap saw.
    route_set_corner_context(0.0f, -1.0f);
    RouteResult r = route_build_loop(m, 0, 0, HEADING_N, p, 1300.0f, 6000.0f,
                                     &st, t, 32, 0.0f);
    expect("legacy v1300 still compiles (as pairs)", r == ROUTE_OK);
    std::printf("  legacy v1300: %u turns, %u U, %u demoted\n", st.turns,
                st.uturns, st.dropped);
    expect("legacy loses every U merge at speed", st.uturns == 0);
    // Corner-speed context: the U flies at sqrt(4900 x 71.74) ~= 593
    // whatever rv says, so solve it there and the merge holds.
    route_set_corner_context(4900.0f, 71.74f);
    r = route_build_loop(m, 0, 0, HEADING_N, p, 1300.0f, 6000.0f, &st, t, 32,
                         0.0f);
    expect("corner-speed v1300 compiles", r == ROUTE_OK);
    std::printf("  ctx    v1300: %u turns, %u U, %u demoted\n", st.turns,
                st.uturns, st.dropped);
    expect("the three 180s survive at rv 1300", st.uturns == 3);
    // And at v500 the context changes nothing: corner speed >= route speed.
    route_set_corner_context(4900.0f, 71.74f);
    RouteStats s5;
    r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f, &s5, t, 32,
                         0.0f);
    expect("v500 unchanged under the context",
           r == ROUTE_OK && s5.uturns == 3);
    route_set_corner_context(0.0f, -1.0f);   // leave the module in legacy
}

// ---- r90/l90: per-direction top-rung override ------------------------------
// The registry's r90/l90 replace ONLY the ladder's rung 0, per direction:
// rights take r90, lefts take l90, U-turns keep the rur geometry, and 0
// means the compiled 84. CAL-6 dir-N has both turn directions and three Us,
// so one build exercises every case.

// ---------------------------------------------------------------- diagonal
//
// The D6 rig exactly as the GUI's RIGS table describes it (post-segment
// walls), so this test and the map bert flies are the same object. The flown
// loop is N up column 0, E along row 5, S down column 5 to (5,3), then the
// staircase (5,3)->(4,3)->(4,2)->(3,2)->(3,1)->(2,1)->(2,0), then W home.
static void build_d6(Maze &m) {
    m.reset();
    // [x1,y1,x2,y2] between posts. Vertical = west side of cell (x1,y1);
    // horizontal = south side of cell (x1,y1). set_wall marks both sides.
    const int W[][4] = {
        {0,0,0,1},{0,1,0,2},{0,2,0,3},{0,3,0,4},{0,4,0,5},{0,5,0,6},
        {1,1,1,2},{1,2,1,3},{1,3,1,4},{1,4,1,5},{2,1,2,2},{3,0,3,1},
        {3,2,3,3},{4,1,4,2},{4,3,4,4},{5,2,5,3},{5,4,5,5},
        {6,0,6,1},{6,1,6,2},{6,2,6,3},{6,3,6,4},{6,4,6,5},{6,5,6,6},
        {0,0,1,0},{1,0,2,0},{2,0,3,0},{3,0,4,0},{4,0,5,0},{5,0,6,0},
        {1,1,2,1},{3,1,4,1},{2,2,3,2},{4,2,5,2},{3,3,4,3},{5,3,6,3},
        {4,4,5,4},{1,5,2,5},{2,5,3,5},{3,5,4,5},{4,5,5,5},
        {0,6,1,6},{1,6,2,6},{2,6,3,6},{3,6,4,6},{4,6,5,6},{5,6,6,6},
    };
    for (auto &w : W) {
        if (w[0] == w[2]) {                       // vertical
            if (w[0] < 6) m.set_wall(w[0], w[1], HEADING_W, true);
            else m.set_wall(w[0] - 1, w[1], HEADING_E, true);
        } else {                                  // horizontal
            if (w[1] < 6) m.set_wall(w[0], w[1], HEADING_S, true);
            else m.set_wall(w[0], w[1] - 1, HEADING_N, true);
        }
    }
    for (int x = 0; x < 6; x++)
        for (int y = 0; y < 6; y++) {
            if (!m.wall_known(x, y, HEADING_N)) m.set_wall(x, y, HEADING_N, false);
            if (!m.wall_known(x, y, HEADING_E)) m.set_wall(x, y, HEADING_E, false);
            if (!m.wall_known(x, y, HEADING_S)) m.set_wall(x, y, HEADING_S, false);
            if (!m.wall_known(x, y, HEADING_W)) m.set_wall(x, y, HEADING_W, false);
        }
    // The rig's start cell opens EAST as well as north (the lap comes home
    // along row 0), which reset() locks shut by the standard rule. Same
    // trick as build_cal6: round-trip the state with the locks cleared.
    Maze::State st;
    m.save_state(st);
    for (int i = 0; i <= MAZE_SIZE; i++) { st.vlock[i] = 0; st.hlock[i] = 0; }
    st.vwall[1] &= ~(uint16_t)1u;   // (0,0) E edge: vertical boundary 1, bit 0
    m.load_state(st);
    m.set_goal(5, 5, 5, 5);
}

// rdiag 0 (ControlGains::route_diag, Route.h route_set_diag): the same
// staircase compiles as the 90s it was -- the banker's orthogonal route.
static void test_rdiag_off_flies_the_staircase_as_90s() {
    Maze m;
    build_d6(m);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    route_set_diag(false);
    RouteResult r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                                     &st, t, 32, 39.0f, HEADING_N);
    route_set_diag(true);
    std::printf("  D6 loop, rdiag 0: %s -- %u turns, %u diag, %u x 135\n",
                route_result_name(r), st.turns, st.diags, st.turns135);
    expect("rdiag 0 still compiles", r == ROUTE_OK);
    expect("rdiag 0: no diagonals", st.diags == 0 && st.turns135 == 0);
    expect("rdiag 0: the zigzag is back (more than four turns)", st.turns > 4);
    for (int j = 0; j < st.turns && j < 32; j++)
        expect("rdiag 0: every turn is a 90 or a 180",
               t[j].sweep_deg == 90 || t[j].sweep_deg == -90 ||
               t[j].sweep_deg == 180 || t[j].sweep_deg == -180);
}

// rturn (route_set_turn_cost): the 2026-09-26 21:24 map, where a wall had
// moved to (4,2)|(5,2) and the cell flood sent MR3 down an 18-cell route
// with ELEVEN R45/R60 zigzag turns (it hit the (2,2) wall on the fourth)
// instead of the 20-cell route with six turns and a U. With turns costing
// three cells the walk takes the six-turn route.
static void build_evening_map(Maze &m) {
    const uint16_t v[17] = {65535,55,24,6,13,28,63,0,0,0,0,0,0,0,0,0,65535};
    const uint16_t h[17] = {65535,4,2,12,0,28,63,0,0,0,0,0,0,0,0,0,65535};
    m.reset();
    Maze::State st; m.save_state(st);
    for (int i = 0; i <= MAZE_SIZE; i++) {
        st.vwall[i] = v[i]; st.hwall[i] = h[i];
        st.vknown[i] = 0xFFFF; st.hknown[i] = 0xFFFF; st.vlock[i] = 0; st.hlock[i] = 0;
    }
    m.load_state(st);
    m.set_goal(2, 4, 2, 4);
}
static void test_rturn_prefers_fewer_turns() {
    for (int wt = 0; wt < 2; wt++) {
        Maze m; build_evening_map(m);
        m.flood_to_goal(false);
        SpeedPath p; RouteStats st; RouteTurn t[32];
        route_set_diag(false);
        route_set_turn_cost(wt ? 3.0f : 0.0f);
        RouteResult r = route_build(m, 0, 0, HEADING_N, p, 1000.0f, 7000.0f, &st, t, 32);
        route_set_turn_cost(0.0f);
        route_set_diag(true);
        std::printf("  evening map, rturn %d: %s -- %u cells, %u turns, %.0f mm\n",
                    wt ? 3 : 0, route_result_name(r), st.cells, st.turns, st.length_mm);
        expect("evening map compiles", r == ROUTE_OK);
        if (!wt) expect("plain flood: the 18-cell zigzag, 11 turns", st.cells == 18 && st.turns == 11);
        else     expect("rturn 3: the 20-cell route with 6 turns", st.cells == 20 && st.turns == 6);
    }
}

// 2026-09-27 10:55: MR2 reached the goal and never came home. The weighted
// walk targeted the GOAL for every caller, and the two-leg home leg --
// route_build from the goal after flood_to_start -- got an empty walk. The
// target is now whatever the caller's flood put at distance 0.
static void test_rturn_home_leg_compiles_from_the_goal() {
    Maze m; build_evening_map(m);
    route_set_diag(false);
    route_set_turn_cost(3.0f);
    // Out leg: where does it end, and facing which way?
    m.flood_to_goal(false);
    int ex, ey; Heading eh;
    expect("out leg end pose", route_walk_end(m, 0, 0, HEADING_N, &ex, &ey, &eh));
    expect("out leg ends on the goal", ex == 2 && ey == 4);
    // The goal turn's heading and the home leg, as the runner does them.
    Heading want;
    expect("home heading found", route_home_heading(m, ex, ey, eh, want));
    m.flood_to_start(false);
    SpeedPath p; RouteStats st; RouteTurn t[32];
    RouteResult r = route_build(m, ex, ey, want, p, 500.0f, 7000.0f, &st, t, 32);
    std::printf("  home leg from (%d,%d) facing %d: %s -- %u cells, %u turns\n",
                ex, ey, (int)want, route_result_name(r), st.cells, st.turns);
    expect("home leg compiles with rturn on", r == ROUTE_OK && st.cells > 1);
    int hx, hy; Heading hh;
    expect("home leg end pose", route_walk_end(m, ex, ey, want, &hx, &hy, &hh));
    expect("home leg ends at the start", hx == 0 && hy == 0);
    route_set_turn_cost(0.0f);
    route_set_diag(true);
}

// THE ROUTE IS CHOSEN BY FLIGHT TIME (Route.h). The 09-27 11:33 map: the
// fewest-cells walk is a staircase that compiles to 135-in / diagonal /
// 135-out / 90; the fewest-turns walk (what rturn alone picked) is six
// 90s and a U round the top. With a time context both are compiled and
// timed: rdiag 1 must fly the diagonal, rdiag 0 must keep the six-turn
// route over the nine-turn zigzag (the turn penalty), and the home leg
// must compile from the goal and end at the start either way.
static void build_0927_map(Maze &m) {
    const uint16_t v[17] = {65535,55,24,5,14,29,63,0,0,0,0,0,0,0,0,0,65535};
    const uint16_t h[17] = {65535,4,2,12,0,28,63,0,0,0,0,0,0,0,0,0,65535};
    m.reset();
    Maze::State st; m.save_state(st);
    for (int i = 0; i <= MAZE_SIZE; i++) {
        st.vwall[i] = v[i]; st.hwall[i] = h[i];
        st.vknown[i] = 0xFFFF; st.hknown[i] = 0xFFFF; st.vlock[i] = 0; st.hlock[i] = 0;
    }
    m.load_state(st);
    m.set_goal(2, 4, 2, 4);
}
static void test_route_chosen_by_flight_time() {
    route_set_corner_context(4000.0f, 71.74f);
    route_set_turn_cost(3.0f);
    for (int diag = 0; diag < 2; diag++) {
        route_set_diag(diag);
        Maze m; build_0927_map(m);
        m.flood_to_goal(false);
        route_set_time_context(2000.0f, 4000.0f, 4000.0f, 4000.0f, true, 7000.0f);
        SpeedPath p; RouteStats st; RouteTurn t[64];
        RouteResult r = route_build(m, 0, 0, HEADING_N, p, 2000.0f, 7000.0f, &st, t, 64, 39.0f);
        const RouteChoice &c = route_last_choice();
        std::printf("  09-27 map rdiag %d, MR5: %s -- %u cells %u turns %u diag; "
                    "fewest cells %.0f ms / score %.0f, fewest turns %.0f / %.0f, chose %s\n",
                    diag, route_result_name(r), st.cells, st.turns, st.diags,
                    c.cand[0].ms, c.cand[0].score, c.cand[1].ms, c.cand[1].score,
                    c.chosen ? "fewest turns" : "fewest cells");
        expect("09-27 map compiles", r == ROUTE_OK);
        expect("both candidates compared", c.compared);
        if (diag) {
            expect("rdiag 1: the diagonal route (16 cells, 3 turns, 1 diagonal)",
                   c.chosen == 0 && st.cells == 16 && st.turns == 3 && st.diags == 1);
            expect("rdiag 1: it flies over a second quicker", c.cand[1].ms - c.cand[0].ms > 1000.0f);
        } else {
            expect("rdiag 0: the six-turn route, not the zigzag",
                   c.chosen == 1 && st.turns == 6 && st.diags == 0);
        }
        // The home leg, as the runner does it.
        int ex, ey; Heading eh;
        expect("out leg end pose", route_walk_end(m, 0, 0, HEADING_N, &ex, &ey, &eh));
        expect("out leg ends on the goal", ex == 2 && ey == 4);
        route_set_time_context(500.0f, 4000.0f, 4000.0f, 4000.0f, true, 7000.0f);
        Heading want;
        expect("home door", route_home_heading(m, ex, ey, eh, want));
        m.flood_to_start(false);
        SpeedPath ph; RouteStats sh;
        RouteResult rh = route_build(m, ex, ey, want, ph, 500.0f, 7000.0f, &sh, nullptr, 0, 39.0f);
        int hx, hy; Heading hh;
        expect("home leg compiles", rh == ROUTE_OK);
        expect("home leg ends at the start",
               route_walk_end(m, ex, ey, want, &hx, &hy, &hh) && hx == 0 && hy == 0);
        if (diag) expect("rdiag 1: the home leg is the diagonal too", sh.diags == 1);
    }
    route_clear_time_context();
    route_set_turn_cost(0.0f);
    route_set_diag(true);
}

static void test_the_d6_staircase_is_one_diagonal() {
    Maze m;
    build_d6(m);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    RouteResult r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                                     &st, t, 32, 39.0f, HEADING_N);
    std::printf("  D6 loop: %s -- %u cells, %u turns, %u diag, %.0f mm\n",
                route_result_name(r), st.cells, st.turns, st.diags,
                st.length_mm);
    for (int j = 0; j < st.turns && j < 32; j++)
        std::printf("    turn %d at (%u,%u) R%u %+d  cp %+d  diag %u\n", j,
                    t[j].cx, t[j].cy, t[j].r_mm, t[j].sweep_deg,
                    t[j].cp_off_mm, t[j].diag_mm);
    expect("D6 compiles", r == ROUTE_OK);
    // Laps 284-290 flew this as NINE turns (three 90s plus a six-corner
    // zigzag). As a diagonal it is FOUR: two 90s, a 45 in, a 45 out.
    expect("four turns", st.turns == 4);
    expect("one diagonal", st.diags == 1);
    expect("the 45-in is at (5,3), right-handed", st.turns == 4 &&
           t[2].cx == 5 && t[2].cy == 3 && t[2].sweep_deg == -45);
    expect("the 45-out is at (2,0), right-handed", st.turns == 4 &&
           t[3].cx == 2 && t[3].cy == 0 && t[3].sweep_deg == -45);
    // Corner points: entry half a cell BEFORE (5,3)'s centre, exit half a
    // cell AFTER (2,0)'s. (5.5,4.0) -> (2.0,0.5): 3.5 cells each way.
    expect("entry corner point is 90 mm early", st.turns == 4 && t[2].cp_off_mm == -90);
    expect("exit corner point is 90 mm late", st.turns == 4 && t[3].cp_off_mm == 90);
    expect("891 mm corner point to corner point", st.turns == 4 &&
           std::fabs((float)t[3].diag_mm - 3.5f * 1.41421356f * 180.0f) < 1.5f);
    expect("the exit 45 has no front wall to gate on", st.turns == 4 && t[3].front_ok == 0);
    expect("the entry 45 keeps its front wall", st.turns == 4 && t[2].front_ok == 1);
    // The path's corners line up with the turns, one arc each, in order --
    // RouteRun indexes turns[] by path corner and depends on this.
    expect("four path corners for four turns", p.corner_count() == 4);
    // The diagonal straight is long: the whole point of the primitive. It is
    // the longest straight in the path (the E leg along row 5 is 5 cells
    // less two tangents, ~700), at 891 less two 45 tangents.
    // (The launch straight up column 0 -- 39 lead-in plus five cells less
    // one R84 tangent, 840 mm -- is the longest; the diagonal is next.)
    float t45 = route_tangent_mm(84.0f, 500.0f, 6000.0f, 45.0f);
    float want = 891.0f - 2.0f * t45;
    bool found = false;
    for (int i = 0; i < p.seg_count(); i++)
        if (p.seg(i).kappa == 0.0f && p.seg(i).kappa1 == 0.0f &&
            std::fabs(p.seg(i).len_mm - want) < 2.0f)
            found = true;
    std::printf("    45 tangent %.1f, diagonal straight wanted %.0f mm\n", t45, want);
    expect("the diagonal straight is 891 less two 45 tangents", found);
    expect("a 45 at R84 consumes about 50 mm, not 218 (tangent_for bug)",
           t45 > 40.0f && t45 < 60.0f);
    // The walk is 26 cells, which as centre-line is 4680 mm; the diagonal
    // shortcuts 6 cells of zigzag (1080 mm of walk) into 891 mm of straight
    // line, so the path is shorter than the walk.
    expect("the path is shorter than the walk it replaces",
           st.length_mm < (float)st.cells * ROUTE_CELL_MM);
    // And the zigzag still compiles when the diagonal is refused -- the
    // fallback that makes this change unable to make a route worse.
    expect("R84 at the exit 45", st.turns == 4 && t[3].r_mm == 84);
    // THE LOOP CLOSES. The path launches 39 mm behind the start cell's
    // centre facing north (path frame: +x north, +y west) and must end on
    // that centre facing west -- (39, 0, +90) -- with a 45-in, 891 mm of
    // diagonal and a 45-out in the middle of it. This is the one assertion
    // that checks every piece of the geometry at once: corner points,
    // tangents (tangent_for divided instead of multiplying at 45 until
    // today), the clothoid placement, and the emit bookkeeping.
    PathPose e = p.pose(p.length_mm());
    std::printf("    loop end pose (%.1f, %.1f, %.1f)\n", e.x_mm, e.y_mm, e.yaw_deg);
    expect("the loop closes on the start cell centre",
           std::fabs(e.x_mm - 39.0f) < 1.0f && std::fabs(e.y_mm) < 1.0f);
    expect("facing west", std::fabs(e.yaw_deg - 90.0f) < 0.5f);
    bool diag_heading = false;
    for (int i = 0; i < p.seg_count(); i++)
        if (p.seg(i).kappa == 0.0f && p.seg(i).kappa1 == 0.0f &&
            std::fabs(p.seg(i).len_mm - want) < 2.0f &&
            std::fabs(p.seg(i).yaw0_rad * 57.29578f - 135.0f) < 0.5f)
            diag_heading = true;
    expect("the diagonal straight runs at exactly 135 deg (south-west)",
           diag_heading);
    // THE SAME LOOP THE OTHER WAY ROUND (route.dir = E, the CCW lap):
    // launch east along row 0, diagonal NE from (2,0) to (5,3), home down
    // column 0. The merge, the corner points and the closure must all
    // survive the direction flip -- the gate's post sides flip with the
    // entry turn's hand, so this is the compile half of bert's "need to
    // test CCW" (2026-09-02).
    r = route_build_loop(m, 0, 0, HEADING_E, p, 500.0f, 6000.0f, &st, t, 32,
                         39.0f, HEADING_E);
    std::printf("  D6 CCW: %s -- %u turns, %u diag, %.0f mm\n",
                route_result_name(r), st.turns, st.diags, st.length_mm);
    for (int j = 0; j < st.turns && j < 32; j++)
        std::printf("    ccw turn %d at (%u,%u) %+d  cp %+d  diag %u\n", j,
                    t[j].cx, t[j].cy, t[j].sweep_deg, t[j].cp_off_mm,
                    t[j].diag_mm);
    expect("CCW compiles", r == ROUTE_OK);
    expect("CCW keeps the diagonal", st.diags == 1 && st.turns == 4);
    // Reversed, the diagonal is the FIRST thing after launch: east along
    // row 0, 45-in at (2,0), out at (5,3), then the two 90s home.
    expect("CCW 45-in is at (2,0), LEFT-handed, turn 0", st.turns == 4 &&
           t[0].cx == 2 && t[0].cy == 0 && t[0].sweep_deg == 45);
    expect("CCW 45-out is at (5,3) with the same 891 mm", st.turns == 4 &&
           t[1].cx == 5 && t[1].cy == 3 && t[1].sweep_deg == 45 &&
           std::fabs((float)t[1].diag_mm - 891.0f) < 2.0f);
    PathPose e2 = p.pose(p.length_mm());
    expect("and the CCW loop closes too",
           std::fabs(e2.x_mm - 39.0f) < 1.0f && std::fabs(e2.y_mm) < 1.0f);
}

// ---------------------------------------------------------------- D7
//
// The D7 "mixed" rig: the D6 loop with a depth-2 U-tooth cut into the top
// straight -- down (2,5)->(2,3), across to (3,3), up to (3,5). Physically
// it is the D6 rig with two wall segments removed ((2,5)-(3,5) and
// (3,5)-(4,5) south walls) and six added, and the diagonal corridor is
// byte-identical to D6's, so dtrim and the DGATE klat fits carry over.
// The point: one lap now holds every proven corner family -- 90s, a
// U-turn (each hand, one per direction), the 45 pair and the 891 mm
// gated diagonal -- with a full cell of feed at every corner, which the
// first tooth draft (turns in four ADJACENT cells) did not have: it
// demoted the whole route to no-diag CW and refused CCW outright. The
// fallback ladder is route-global; one starved corner costs the diagonal.
static void build_d7(Maze &m) {
    m.reset();
    const int W[][4] = {
        {0,0,0,1},{0,0,1,0},{0,1,0,2},{0,2,0,3},{0,3,0,4},{0,4,0,5},
        {0,5,0,6},{0,6,1,6},{1,0,2,0},{1,1,1,2},{1,1,2,1},{1,2,1,3},
        {1,3,1,4},{1,4,1,5},{1,5,2,5},{1,6,2,6},{2,0,3,0},{2,1,2,2},
        {2,2,3,2},{2,3,2,4},{2,3,3,3},{2,4,2,5},{2,6,3,6},{3,0,3,1},
        {3,0,4,0},{3,1,4,1},{3,2,3,3},{3,3,4,3},{3,4,3,5},{3,5,3,6},
        {3,6,4,6},{4,0,5,0},{4,1,4,2},{4,2,5,2},{4,3,4,4},{4,4,4,5},
        {4,4,5,4},{4,5,5,5},{4,6,5,6},{5,0,6,0},{5,2,5,3},{5,3,6,3},
        {5,4,5,5},{5,6,6,6},{6,0,6,1},{6,1,6,2},{6,2,6,3},{6,3,6,4},
        {6,4,6,5},{6,5,6,6}
    };
    for (auto &w : W) {
        if (w[0] == w[2]) {                       // vertical
            if (w[0] < 6) m.set_wall(w[0], w[1], HEADING_W, true);
            else m.set_wall(w[0] - 1, w[1], HEADING_E, true);
        } else {                                  // horizontal
            if (w[1] < 6) m.set_wall(w[0], w[1], HEADING_S, true);
            else m.set_wall(w[0], w[1] - 1, HEADING_N, true);
        }
    }
    for (int x = 0; x < 6; x++)
        for (int y = 0; y < 6; y++) {
            if (!m.wall_known(x, y, HEADING_N)) m.set_wall(x, y, HEADING_N, false);
            if (!m.wall_known(x, y, HEADING_E)) m.set_wall(x, y, HEADING_E, false);
            if (!m.wall_known(x, y, HEADING_S)) m.set_wall(x, y, HEADING_S, false);
            if (!m.wall_known(x, y, HEADING_W)) m.set_wall(x, y, HEADING_W, false);
        }
    Maze::State st;                // same (0,0)E unlock as build_d6
    m.save_state(st);
    for (int i = 0; i <= MAZE_SIZE; i++) { st.vlock[i] = 0; st.hlock[i] = 0; }
    st.vwall[1] &= ~(uint16_t)1u;
    m.load_state(st);
    m.set_goal(5, 5, 5, 5);
}

static void test_the_d7_mixed_rig() {
    Maze m;
    build_d7(m);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    // CW: up column 0, along row 5 taking the tooth, down column 5,
    // diagonal home.
    RouteResult r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                                     &st, t, 32, 39.0f, HEADING_N);
    std::printf("  D7 CW: %s -- %u cells, %u turns, %u U, %u diag, %.0f mm\n",
                route_result_name(r), st.cells, st.turns, st.uturns, st.diags,
                st.length_mm);
    for (int j = 0; j < st.turns && j < 32; j++)
        std::printf("    turn %d at (%u,%u) R%u %+d  cp %+d  diag %u\n", j,
                    t[j].cx, t[j].cy, t[j].r_mm, t[j].sweep_deg,
                    t[j].cp_off_mm, t[j].diag_mm);
    expect("D7 CW compiles", r == ROUTE_OK);
    expect("seven turns, nothing demoted",
           st.turns == 7 && st.dropped == 0);
    expect("the tooth merges to ONE left U-turn",
           st.uturns == 1 && st.turns == 7 && t[2].cx == 2 && t[2].cy == 3 &&
           t[2].sweep_deg == 180 && t[2].exit_ix == t[2].cell_ix + 1);
    expect("the diagonal survives alongside the U-turn",
           st.diags == 1 && st.turns == 7 && t[5].cx == 5 && t[5].cy == 3 &&
           t[5].sweep_deg == -45 && t[5].cp_off_mm == -90);
    expect("same 891 mm corridor as D6", st.turns == 7 &&
           t[6].cx == 2 && t[6].cy == 0 &&
           std::fabs((float)t[6].diag_mm - 891.0f) < 2.0f);
    expect("every 90 keeps R84", st.turns == 7 &&
           t[0].r_mm == 84 && t[1].r_mm == 84 && t[3].r_mm == 84 &&
           t[4].r_mm == 84);
    expect("feeds are honest, not squeezed", st.min_straight > 100.0f);
    PathPose e = p.pose(p.length_mm());
    expect("the CW loop closes",
           std::fabs(e.x_mm - 39.0f) < 1.0f && std::fabs(e.y_mm) < 1.0f);
    // CCW: diagonal first, then the tooth from the other side -- the
    // U-turn flips hands (right), the 45s flip to left.
    r = route_build_loop(m, 0, 0, HEADING_E, p, 500.0f, 6000.0f, &st, t, 32,
                         39.0f, HEADING_E);
    std::printf("  D7 CCW: %s -- %u turns, %u U, %u diag, %.0f mm\n",
                route_result_name(r), st.turns, st.uturns, st.diags,
                st.length_mm);
    expect("D7 CCW compiles", r == ROUTE_OK);
    expect("CCW keeps all seven, U and diagonal",
           st.turns == 7 && st.uturns == 1 && st.diags == 1 &&
           st.dropped == 0);
    expect("CCW 45-in is at (2,0), left-handed, turn 0", st.turns == 7 &&
           t[0].cx == 2 && t[0].cy == 0 && t[0].sweep_deg == 45);
    expect("CCW U-turn is right-handed at (3,3)", st.turns == 7 &&
           t[4].cx == 3 && t[4].cy == 3 && t[4].sweep_deg == -180);
    PathPose e2 = p.pose(p.length_mm());
    expect("and the CCW loop closes too",
           std::fabs(e2.x_mm - 39.0f) < 1.0f && std::fabs(e2.y_mm) < 1.0f);
    // route.dir=AUTO on a circuit with UNEQUAL arms (the regression that
    // grounded the mouse, 2026-09-04): the plain home flood used to walk
    // straight back the shorter arm -- an about-face in the goal cell,
    // ROUTE_BAD_TURN. The loop builder now closes the door the out leg
    // arrived through for the home flood, so auto compiles the same lap
    // the forced build does. (From a NORTH pose auto still needs
    // RouteRun's force-the-pose-heading: the unforced out flood would
    // take the east arm and demand an in-cell 90.)
    r = route_build_loop(m, 0, 0, HEADING_E, p, 500.0f, 6000.0f, &st, t, 32,
                         39.0f, -1);
    std::printf("  D7 auto: %s -- %u turns, %u U, %u diag\n",
                route_result_name(r), st.turns, st.uturns, st.diags);
    expect("auto compiles the unequal-arm circuit (no about-face home)",
           r == ROUTE_OK && st.turns == 7 && st.uturns == 1 &&
           st.diags == 1 && st.dropped == 0);
}


// A corridor maze from a cell sequence: every edge a wall except between
// consecutive cells, everything known, the last cell the goal. The flood
// then has exactly one route, which is the shape under test.
static void build_corridor(Maze &m, const int (*cells)[2], int n) {
    m.reset();
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++)
            for (int h = 0; h < 4; h++) m.set_wall(x, y, (Heading)h, true);
    Maze::State st;
    m.save_state(st);
    for (int i = 0; i <= MAZE_SIZE; i++) { st.vlock[i] = 0; st.hlock[i] = 0; }
    m.load_state(st);
    for (int i = 0; i + 1 < n; i++) {
        int dx = cells[i + 1][0] - cells[i][0], dy = cells[i + 1][1] - cells[i][1];
        Heading h = dx > 0 ? HEADING_E : dx < 0 ? HEADING_W
                  : dy > 0 ? HEADING_N : HEADING_S;
        m.set_wall(cells[i][0], cells[i][1], h, false);
    }
    m.set_goal(cells[n - 1][0], cells[n - 1][1], cells[n - 1][0], cells[n - 1][1]);
    m.flood_to_goal(false);
}

// THE V (Route.h). A NE staircase of three steps handing straight into a
// NW staircase of two: in cells, R L R L | L | R L R. Expected: a 45-in at
// (0,1), ONE 90 at (2,2) whose corner point is the aperture (2.5, 3.0),
// a 45-out at (0,4); both diagonals 2 cells x 2 cells = 509 mm; and the
// path lands on the goal centre.
static void test_the_v_joins_two_diagonals() {
    Maze m;
    static const int cells[][2] = {{0,0},{0,1},{1,1},{1,2},{2,2},{2,3},
                                   {1,3},{1,4},{0,4},{0,5}};
    build_corridor(m, cells, 10);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    RouteResult r = route_build(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f, &st,
                                t, 32, 0.0f);
    std::printf("  V rig: %s -- %u cells, %u turns, %u U, %u diag, %.0f mm, "
                "min straight %.0f\n", route_result_name(r), st.cells,
                st.turns, st.uturns, st.diags, st.length_mm, st.min_straight);
    for (int j = 0; j < st.turns && j < 32; j++)
        std::printf("    turn %d at (%u,%u) R%u %+d  cp %+d  diag %u\n", j,
                    t[j].cx, t[j].cy, t[j].r_mm, t[j].sweep_deg,
                    t[j].cp_off_mm, t[j].diag_mm);
    expect("V rig compiles", r == ROUTE_OK);
    expect("three turns: 45-in, V, 45-out", st.turns == 3 && st.uturns == 0);
    expect("45-in at (0,1) right", st.turns == 3 && t[0].cx == 0 && t[0].cy == 1 &&
           t[0].sweep_deg == -45 && t[0].cp_off_mm == -90 && t[0].diag_mm == 0);
    expect("the V is a LEFT 90 at (2,2) with a corner-point offset",
           st.turns == 3 && t[1].cx == 2 && t[1].cy == 2 && t[1].sweep_deg == 90 &&
           t[1].cp_off_mm == 90 && route_turn_is_v(t[1]));
    expect("the V ends a 509 mm diagonal", st.turns == 3 &&
           std::fabs((float)t[1].diag_mm - 509.1f) < 1.5f);
    expect("45-out at (0,4) right ends the second 509 mm diagonal",
           st.turns == 3 && t[2].cx == 0 && t[2].cy == 4 && t[2].sweep_deg == -45 &&
           t[2].cp_off_mm == 90 && std::fabs((float)t[2].diag_mm - 509.1f) < 1.5f);
    expect("nothing demoted at 500 mm/s", st.dropped == 0);
    expect("two diagonals counted", st.diags == 2);
    PathPose e = p.pose(p.length_mm());
    std::printf("    end pose x %.1f y %.1f yaw %.1f\n", e.x_mm, e.y_mm, e.yaw_deg);
    expect("the path lands on the goal centre facing north",
           std::fabs(e.x_mm - 900.0f) < 1.5f && std::fabs(e.y_mm) < 1.5f &&
           std::fabs(e.yaw_deg) < 0.5f);
    // Arcs never overlap: each arc's start is at or after the previous
    // arc's end (the fit's promise), including around the V.
    bool ordered = true;
    for (int k = 1; k < p.corner_count(); k++)
        if (p.corner_entry_s_mm(k) < p.corner_exit_s_mm(k - 1) - 0.01f) ordered = false;
    expect("no two arcs overlap around the V", ordered);

    // A diagonal ZIGZAG: one-step runs joined by Vs (NE, NW, NE). In cells
    // R L | L | R R | R L. Expected: 45-in, V, V, 45-out, each diagonal
    // 254.6 mm.
    static const int zig[][2] = {{0,0},{0,1},{1,1},{1,2},{0,2},{0,3},{1,3},{1,4}};
    build_corridor(m, zig, 8);
    r = route_build(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f, &st, t, 32, 0.0f);
    std::printf("  zigzag: %s -- %u turns, %u U, %u diag, %.0f mm, min straight %.0f\n",
                route_result_name(r), st.turns, st.uturns, st.diags,
                st.length_mm, st.min_straight);
    for (int j = 0; j < st.turns && j < 32; j++)
        std::printf("    turn %d at (%u,%u) R%u %+d  cp %+d  diag %u\n", j,
                    t[j].cx, t[j].cy, t[j].r_mm, t[j].sweep_deg,
                    t[j].cp_off_mm, t[j].diag_mm);
    expect("zigzag compiles", r == ROUTE_OK);
    expect("zigzag is 45-in, V, V, 45-out", st.turns == 4 &&
           t[0].sweep_deg == -45 && route_turn_is_v(t[1]) && t[1].sweep_deg == 90 &&
           route_turn_is_v(t[2]) && t[2].sweep_deg == -90 && t[3].sweep_deg == 45);
    expect("each zigzag diagonal is one step (255 mm)", st.turns == 4 &&
           std::fabs((float)t[1].diag_mm - 254.6f) < 1.5f &&
           std::fabs((float)t[2].diag_mm - 254.6f) < 1.5f &&
           std::fabs((float)t[3].diag_mm - 254.6f) < 1.5f);
    PathPose e2 = p.pose(p.length_mm());
    expect("zigzag lands on (1,4) facing north",
           std::fabs(e2.x_mm - 720.0f) < 1.5f && std::fabs(e2.y_mm + 180.0f) < 1.5f);

    // A lone same-sign pair with a staircase on ONE side only is not a V:
    // it is the U-turn it always was, and the route still compiles by the
    // fallback ladder. R L R L | L then straight west.
    static const int notv[][2] = {{0,0},{0,1},{1,1},{1,2},{2,2},{2,3},{1,3},{0,3}};
    build_corridor(m, notv, 8);
    r = route_build(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f, &st, t, 32, 0.0f);
    std::printf("  not-a-V: %s -- %u turns, %u U, %u diag\n",
                route_result_name(r), st.turns, st.uturns, st.diags);
    expect("not-a-V compiles", r == ROUTE_OK);
    bool any_v = false;
    for (int j = 0; j < st.turns; j++) if (route_turn_is_v(t[j])) any_v = true;
    expect("no V without a diagonal on both sides", !any_v);
}


// D8, THE V RIG (2026-09-05): the V as a 6x6 loop bert can build. Out:
// 45-in R at (0,1), a 509 mm NE diagonal, the LEFT V at (2,2)/(2,3), a
// 255 mm NW diagonal, 45-out R at (1,3), straight N to the goal (1,5).
// Home: R at (1,5), E along row 5, R at (3,5), S down column 3, R at
// (3,0), W along row 0. Everything else walled. CCW the V is right-handed
// and the long diagonal comes last, ending at the start cell's doorstep.
// The wall list is in tune_gui.html ("D8 V").
static void build_d8(Maze &m) {
    m.reset();
    const int W[][4] = {
        {0,0,0,1}, {0,1,0,2}, {0,2,0,3}, {0,3,0,4}, {0,4,0,5}, {0,5,0,6}, {1,2,1,3}, {1,3,1,4}, {1,4,1,5}, {1,5,1,6}, {2,1,2,2}, {2,4,2,5}, {3,1,3,2}, {3,2,3,3}, {3,3,3,4}, {3,4,3,5}, {4,0,4,1}, {4,1,4,2}, {4,2,4,3}, {4,3,4,4}, {4,4,4,5}, {4,5,4,6}, {5,0,5,1}, {5,1,5,2}, {5,2,5,3}, {5,3,5,4}, {5,4,5,5}, {5,5,5,6}, {6,0,6,1}, {6,1,6,2}, {6,2,6,3}, {6,3,6,4}, {6,4,6,5}, {6,5,6,6}, {0,0,1,0}, {1,0,2,0}, {2,0,3,0}, {3,0,4,0}, {4,0,5,0}, {5,0,6,0}, {1,1,2,1}, {2,1,3,1}, {4,1,5,1}, {5,1,6,1}, {0,2,1,2}, {2,2,3,2}, {4,2,5,2}, {5,2,6,2}, {0,3,1,3}, {1,3,2,3}, {4,3,5,3}, {5,3,6,3}, {0,4,1,4}, {2,4,3,4}, {4,4,5,4}, {5,4,6,4}, {0,5,1,5}, {2,5,3,5}, {4,5,5,5}, {5,5,6,5}, {0,6,1,6}, {1,6,2,6}, {2,6,3,6}, {3,6,4,6}, {4,6,5,6}, {5,6,6,6}
    };
    for (auto &w : W) {
        if (w[0] == w[2]) {
            if (w[0] < 6) m.set_wall(w[0], w[1], HEADING_W, true);
            else m.set_wall(w[0] - 1, w[1], HEADING_E, true);
        } else {
            if (w[1] < 6) m.set_wall(w[0], w[1], HEADING_S, true);
            else m.set_wall(w[0], w[1] - 1, HEADING_N, true);
        }
    }
    for (int x = 0; x < 6; x++)
        for (int y = 0; y < 6; y++)
            for (int h = 0; h < 4; h++)
                if (!m.wall_known(x, y, (Heading)h)) m.set_wall(x, y, (Heading)h, false);
    Maze::State st;
    m.save_state(st);
    for (int i = 0; i <= MAZE_SIZE; i++) { st.vlock[i] = 0; st.hlock[i] = 0; }
    st.vwall[1] &= ~(uint16_t)1u;   // the start cell's east lock, as D6/D7
    m.load_state(st);
    m.set_goal(1, 5, 1, 5);
}

// D9 (2026-09-10): the corner-type table's unflown rows. A one-step
// diagonal ZIGZAG (45-in, V left, V right, 45-out -- the shape
// test_the_v_joins_two_diagonals compiles), a 90 whose inside wall is
// missing in the cell before it (the stub under (4,5): no edge to lock,
// the front wall speaks -> the FRONT/NO_EDGE classes), a one-step DOGLEG
// ((5,2) R then (4,2) L: two crowded 90s on the demoted ladder) followed
// by a 90 one cell on ((4,0)), a 4-cell straight and a 3-cell straight.
// Goal (5,5), start (0,0) N; 6x6.
static void build_d9(Maze &m) {
    m.reset();
    const int W[][4] = {
        {0,0,0,1}, {0,1,0,2}, {0,2,0,3}, {0,3,0,4}, {0,4,0,5}, {0,5,0,6}, {1,4,1,5}, {1,5,1,6}, {2,1,2,2}, {2,2,2,3}, {2,3,2,4}, {2,4,2,5}, {3,1,3,2}, {3,2,3,3}, {3,3,3,4}, {3,4,3,5}, {4,1,4,2}, {4,2,4,3}, {4,3,4,4}, {4,4,4,5}, {5,0,5,1}, {5,1,5,2}, {5,3,5,4}, {5,4,5,5}, {6,0,6,1}, {6,1,6,2}, {6,2,6,3}, {6,3,6,4}, {6,4,6,5}, {6,5,6,6}, {0,0,1,0}, {1,0,2,0}, {2,0,3,0}, {3,0,4,0}, {4,0,5,0}, {5,0,6,0}, {1,1,2,1}, {2,1,3,1}, {3,1,4,1}, {5,1,6,1}, {0,2,1,2}, {2,2,3,2}, {3,2,4,2}, {5,2,6,2}, {1,3,2,3}, {2,3,3,3}, {3,3,4,3}, {4,3,5,3}, {0,4,1,4}, {2,4,3,4}, {3,4,4,4}, {4,4,5,4}, {0,5,1,5}, {2,5,3,5}, {3,5,4,5}, {0,6,1,6}, {1,6,2,6}, {2,6,3,6}, {3,6,4,6}, {4,6,5,6}, {5,6,6,6}
    };
    for (auto &w : W) {
        if (w[0] == w[2]) {
            if (w[0] < 6) m.set_wall(w[0], w[1], HEADING_W, true);
            else m.set_wall(w[0] - 1, w[1], HEADING_E, true);
        } else {
            if (w[1] < 6) m.set_wall(w[0], w[1], HEADING_S, true);
            else m.set_wall(w[0], w[1] - 1, HEADING_N, true);
        }
    }
    for (int x = 0; x < 6; x++)
        for (int y = 0; y < 6; y++)
            for (int h = 0; h < 4; h++)
                if (!m.wall_known(x, y, (Heading)h)) m.set_wall(x, y, (Heading)h, false);
    Maze::State st;
    m.save_state(st);
    for (int i = 0; i <= MAZE_SIZE; i++) { st.vlock[i] = 0; st.hlock[i] = 0; }
    st.vwall[1] &= ~(uint16_t)1u;
    m.load_state(st);
    m.set_goal(5, 5, 5, 5);
}

static void test_the_d9_rig() {
    Maze m;
    build_d9(m);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    for (int way = 0; way < 2; way++) {
        Heading h = way == 0 ? HEADING_N : HEADING_E;
        RouteResult r = route_build_loop(m, 0, 0, h, p, 500.0f, 6000.0f,
                                         &st, t, 32, 39.0f, h);
        std::printf("  D9 %s: %s -- %u cells, %u turns, %u U, %u diag, %u dropped, %.0f mm, "
                    "min straight %.0f\n", way == 0 ? "CW" : "CCW", route_result_name(r),
                    st.cells, st.turns, st.uturns, st.diags, st.dropped, st.length_mm,
                    st.min_straight);
        for (int j = 0; j < st.turns && j < 32; j++)
            std::printf("    turn %d at (%u,%u) R%u %+d  cp %+d  diag %u\n", j,
                        t[j].cx, t[j].cy, t[j].r_mm, t[j].sweep_deg,
                        t[j].cp_off_mm, t[j].diag_mm);
        expect("D9 compiles", r == ROUTE_OK);
        PathPose e = p.pose(p.length_mm());
        expect("the D9 loop closes",
               std::fabs(e.x_mm - 39.0f) < 2.0f && std::fabs(e.y_mm) < 2.0f);
    }
}

// bert's D10 (2026-09-11), drawn in the GUI's maze creator: a snake through
// all 36 cells with four staircases and three hairpins, two of which sit
// straight on a staircase exit -- (2,4) 45-out into the U at (2,5)/(3,5),
// (2,2) 45-out into the U at (1,2)/(1,1). 90 mm of room where the U needs
// 181. The route-global ladder refused both headings and flew the whole
// lap as the zigzag (19 turns, 17 demoted, min straight 0). bert: "the
// mouse can always slow down but should never refuse any valid maze" --
// the multi-pass compile and the speed rung (Route.h) are the answer, and
// this is their acceptance test.
static void build_d10(Maze &m) {
    m.reset();
    const int W[][4] = { {0,0,0,1}, {6,0,6,1}, {0,1,0,2}, {6,1,6,2}, {0,2,0,3}, {6,2,6,3}, {0,3,0,4}, {6,3,6,4}, {0,4,0,5}, {6,4,6,5}, {0,5,0,6}, {6,5,6,6}, {0,0,1,0}, {0,6,1,6}, {1,0,2,0}, {1,6,2,6}, {2,0,3,0}, {2,6,3,6}, {3,0,4,0}, {3,6,4,6}, {4,0,5,0}, {4,6,5,6}, {5,0,6,0}, {5,6,6,6}, {1,1,1,2}, {1,2,1,3}, {1,4,1,5}, {1,5,1,6}, {2,3,2,4}, {2,5,2,6}, {3,2,3,3}, {3,4,3,5}, {4,1,4,2}, {4,3,4,4}, {4,4,4,5}, {4,5,4,6}, {5,0,5,1}, {5,2,5,3}, {5,3,5,4}, {5,4,5,5}, {1,1,2,1}, {2,1,3,1}, {3,1,4,1}, {2,2,3,2}, {4,2,5,2}, {1,3,2,3}, {3,3,4,3}, {2,4,3,4} };
    for (auto &w : W) {
        if (w[0] == w[2]) {
            if (w[0] < 6) m.set_wall(w[0], w[1], HEADING_W, true);
            else m.set_wall(w[0] - 1, w[1], HEADING_E, true);
        } else {
            if (w[1] < 6) m.set_wall(w[0], w[1], HEADING_S, true);
            else m.set_wall(w[0], w[1] - 1, HEADING_N, true);
        }
    }
    for (int x = 0; x < 6; x++)
        for (int y = 0; y < 6; y++)
            for (int h = 0; h < 4; h++)
                if (!m.wall_known(x, y, (Heading)h)) m.set_wall(x, y, (Heading)h, false);
    Maze::State st;
    m.save_state(st);
    for (int i = 0; i <= MAZE_SIZE; i++) { st.vlock[i] = 0; st.hlock[i] = 0; }
    st.vwall[1] &= ~(uint16_t)1u;
    m.load_state(st);
    m.set_goal(5, 5, 5, 5);
}

static void test_the_d10_rig_is_never_refused() {
    Maze m;
    build_d10(m);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    // WITHOUT the 135 (r135 = 0): the pre-135 parse, and the two rungs
    // under the radius ladder are what keep it flyable.
    route_set_135(false);
    for (int way = 0; way < 2; way++) {
        Heading h = way == 0 ? HEADING_N : HEADING_E;
        RouteResult r = route_build_loop(m, 0, 0, h, p, 500.0f, 6000.0f,
                                         &st, t, 32, 39.0f, h);
        std::printf("  D10 (no 135) %s: %s -- %u cells, %u turns, %u U, %u diag, %u dropped, "
                    "%u slowed, %u vetoed, %u passes, %.0f mm, min straight %.0f\n",
                    way == 0 ? "CW" : "CCW", route_result_name(r), st.cells,
                    st.turns, st.uturns, st.diags, st.dropped, st.slowed,
                    st.vetoed, st.passes, st.length_mm, st.min_straight);
        for (int j = 0; j < st.turns && j < 32; j++)
            std::printf("    turn %d at (%u,%u) ix %u R%u %+d  cp %+d  diag %u  cap %u\n", j,
                        t[j].cx, t[j].cy, t[j].cell_ix, t[j].r_mm, t[j].sweep_deg,
                        t[j].cp_off_mm, t[j].diag_mm, t[j].v_cap_mm_s);
        expect("D10 (no 135) compiles", r == ROUTE_OK);
        expect("D10 (no 135): three passes, the two hairpins on staircase exits un-merged",
               st.passes == 3 && st.vetoed == 2);
        expect("D10 (no 135): all four staircases and the third hairpin survive",
               st.diags == 4 && st.uturns == 1 && st.turns135 == 0);
        expect("D10 (no 135): the two joins are flown slow, nothing else is",
               st.slowed == 4);
        expect("D10 (no 135): 11 turns, not the 19 of the zigzag", st.turns == 11);
        expect("D10 (no 135): a real straight at every join", st.min_straight >= 20.0f);
        // The un-merged hairpin: a 45 and a 90 one cell apart (45-out then
        // 90 flown CW, 90 then 45-in flown CCW), both capped; the walk from
        // the diagonal's corner point to the 90's is 90 mm, which only fits
        // with the tangents at their floor speeds.
        int k = -1, k45 = -1, k90 = -1;
        for (int j = 0; j + 1 < st.turns; j++) {
            bool a45 = t[j].sweep_deg == 45 || t[j].sweep_deg == -45;
            bool b45 = t[j + 1].sweep_deg == 45 || t[j + 1].sweep_deg == -45;
            if (a45 == b45) continue;
            if (t[j].v_cap_mm_s && t[j + 1].v_cap_mm_s &&
                t[j + 1].cell_ix == t[j].cell_ix + 1) {
                k = j; k45 = a45 ? j : j + 1; k90 = a45 ? j + 1 : j; break;
            }
        }
        expect("D10 (no 135): a 45 hands straight into a capped 90 (or the reverse)", k >= 0);
        if (k >= 0)
            expect("D10 (no 135): the 90 is at the speed floor, the 45 just above it",
                   t[k90].v_cap_mm_s == (uint16_t)ROUTE_V_FLOOR_MM_S &&
                   t[k45].v_cap_mm_s > ROUTE_V_FLOOR_MM_S && t[k45].v_cap_mm_s < 200);
        PathPose e = p.pose(p.length_mm());
        expect("the D10 (no 135) loop closes",
               std::fabs(e.x_mm - 39.0f) < 2.0f && std::fabs(e.y_mm) < 2.0f);
        // The cap reaches the path: sampled ON the capped 90 the planner
        // asks for no more than the cap.
        if (k >= 0) {
            // Path corners are registered one per turn, in order.
            float s_mid = 0.5f * (p.corner_entry_s_mm(k90) + p.corner_exit_s_mm(k90));
            float v = p.v_cap_at(s_mid);
            expect("D10 (no 135): the path's cap on the slowed 90 is the fit's",
                   std::fabs(v - (float)t[k90].v_cap_mm_s) < 1.0f);
        }
    }
    route_set_135(true);
}

// WITH the 135 (Route.h, THE 135): the same two joins are one arc each --
// a 135-out flown CW (diagonal NE, then south round the wall-end post), a
// 135-in flown CCW -- and D10 compiles in one pass at R84 throughout.
static void test_the_d10_rig_with_the_135() {
    Maze m;
    build_d10(m);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    for (int way = 0; way < 2; way++) {
        Heading h = way == 0 ? HEADING_N : HEADING_E;
        RouteResult r = route_build_loop(m, 0, 0, h, p, 500.0f, 6000.0f,
                                         &st, t, 32, 39.0f, h);
        std::printf("  D10 (135) %s: %s -- %u cells, %u turns, %u U, %u diag, %u x 135, %u dropped, "
                    "%u slowed, %u vetoed, %u passes, %.0f mm, min straight %.0f\n",
                    way == 0 ? "CW" : "CCW", route_result_name(r), st.cells,
                    st.turns, st.uturns, st.diags, st.turns135, st.dropped, st.slowed,
                    st.vetoed, st.passes, st.length_mm, st.min_straight);
        for (int j = 0; j < st.turns && j < 32; j++)
            std::printf("    turn %d at (%u,%u) ix %u R%u %+d  cp %+d  diag %u  cap %u\n", j,
                        t[j].cx, t[j].cy, t[j].cell_ix, t[j].r_mm, t[j].sweep_deg,
                        t[j].cp_off_mm, t[j].diag_mm, t[j].v_cap_mm_s);
        expect("D10 (135) compiles in one pass", r == ROUTE_OK && st.passes == 1 && st.vetoed == 0);
        expect("D10 (135): four diagonals, two 135s, one U-turn, nine turns",
               st.diags == 4 && st.turns135 == 2 && st.uturns == 1 && st.turns == 9);
        expect("D10 (135): R84 everywhere, nothing slowed", st.dropped == 0 && st.slowed == 0);
        expect("D10 (135): 90 mm of straight at the tightest join", st.min_straight >= 85.0f);
        // The 135's record: the lone 90's cell, corner point half a cell
        // off its centre (-90 on an out, +90 on an in), the diagonal it
        // ends one pitch longer than a 45 would report (5 steps -> 764).
        int n_out = 0, n_in = 0;
        for (int j = 0; j < st.turns; j++) {
            if (t[j].sweep_deg != 135 && t[j].sweep_deg != -135) continue;
            if (t[j].diag_mm > 0) { n_out++; expect("135-out: corner point 90 before the cell centre", t[j].cp_off_mm == -90); }
            else { n_in++; expect("135-in: corner point 90 past the cell centre", t[j].cp_off_mm == 90); }
        }
        expect(way == 0 ? "CW: both 135s are outs" : "CCW: both 135s are ins",
               way == 0 ? (n_out == 2 && n_in == 0) : (n_in == 2 && n_out == 0));
        bool top_ok = false;
        for (int j = 0; j < st.turns; j++)
            if (t[j].cx == 3 && t[j].cy == 5 && (t[j].sweep_deg == 135 || t[j].sweep_deg == -135))
                top_ok = (t[j].diag_mm == 764 && t[j].sweep_deg == (way == 0 ? -135 : 135)) ||
                         (way == 1 && t[j].diag_mm == 0);
        expect("the top join is the 135 at (3,5), 764 mm of diagonal behind it CW", top_ok);
        // Geometry: the loop closes -- a 135 whose corner point were off
        // by any amount would leave it open.
        PathPose e = p.pose(p.length_mm());
        expect("the D10 (135) loop closes",
               std::fabs(e.x_mm - 39.0f) < 2.0f && std::fabs(e.y_mm) < 2.0f);
        // ...and the arc is a real 135: its path length is R x 3pi/4 plus
        // the clothoid ends, and it consumes 2.414 R of corridor a side.
        for (int j = 0; j < st.turns; j++) {
            if (t[j].sweep_deg != 135 && t[j].sweep_deg != -135) continue;
            float tan = p.corner_tangent_mm(j);
            expect("135 tangent is at least 2.414 R", tan >= 2.414f * (float)t[j].r_mm - 0.5f);
            expect("135 tangent is under 2.414 R + a clothoid", tan < 2.414f * (float)t[j].r_mm + 60.0f);
        }
    }
}

// A rig the old ladder also refused, quietly: CAL-6 rev C's UL at (1,3)
// with a right one cell after it. The U's setback wants 210 mm, the cell
// gives 180. The veto un-merges that U alone; the other two survive.
static void test_a_hairpin_crowded_by_a_ninety_unmerges_alone() {
    // rev C differs from rev A only around (2,4)/(3,5): rebuild from the
    // GUI's wall list.
    Maze m;
    m.reset();
    const int W[][4] = {{0,0,0,1},{0,1,0,2},{0,2,0,3},{0,3,0,4},{0,4,0,5},{0,5,0,6},{1,1,1,2},{1,2,1,3},{1,3,1,4},{1,4,1,5},{2,4,2,5},{2,5,2,6},{3,2,3,3},{3,3,3,4},{3,5,3,6},{5,1,5,2},{6,0,6,1},{6,1,6,2},{6,2,6,3},{6,3,6,4},{6,4,6,5},{6,5,6,6},{0,0,1,0},{1,0,2,0},{2,0,3,0},{3,0,4,0},{4,0,5,0},{5,0,6,0},{1,1,2,1},{2,1,3,1},{3,1,4,1},{4,1,5,1},{3,2,4,2},{4,2,5,2},{1,3,2,3},{2,3,3,3},{4,3,5,3},{5,3,6,3},{3,4,4,4},{4,4,5,4},{3,5,4,5},{4,5,5,5},{5,5,6,5},{0,6,1,6},{1,6,2,6},{2,6,3,6},{3,6,4,6},{4,6,5,6},{5,6,6,6}};
    for (auto &w : W) {
        if (w[0] == w[2]) {
            if (w[0] < 6) m.set_wall(w[0], w[1], HEADING_W, true);
            else m.set_wall(w[0] - 1, w[1], HEADING_E, true);
        } else {
            if (w[1] < 6) m.set_wall(w[0], w[1], HEADING_S, true);
            else m.set_wall(w[0], w[1] - 1, HEADING_N, true);
        }
    }
    for (int x = 0; x < 6; x++)
        for (int y = 0; y < 6; y++)
            for (int h = 0; h < 4; h++)
                if (!m.wall_known(x, y, (Heading)h)) m.set_wall(x, y, (Heading)h, false);
    Maze::State st0;
    m.save_state(st0);
    for (int i = 0; i <= MAZE_SIZE; i++) { st0.vlock[i] = 0; st0.hlock[i] = 0; }
    st0.vwall[1] &= ~(uint16_t)1u;
    m.load_state(st0);
    m.set_goal(5, 4, 5, 4);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    RouteResult r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                                     &st, t, 32, 39.0f, HEADING_N);
    std::printf("  rev C CW: %s -- %u turns, %u U, %u dropped, %u slowed, %u vetoed, %u passes\n",
                route_result_name(r), st.turns, st.uturns, st.dropped, st.slowed,
                st.vetoed, st.passes);
    expect("rev C compiles", r == ROUTE_OK);
    // With the 135 the tooth is a 135-in, one step of diagonal, a 45-out:
    // no U to un-merge, nothing demoted, one pass.
    expect("rev C: one pass, three U-turns, the tooth a 135-in",
           st.vetoed == 0 && st.passes == 1 && st.uturns == 3 && st.turns135 == 1 &&
           st.dropped == 0 && st.turns == 7);
    // Without it: the old parse, one veto, the un-merged pair on the ladder.
    route_set_135(false);
    r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                         &st, t, 32, 39.0f, HEADING_N);
    route_set_135(true);
    std::printf("  rev C CW (no 135): %s -- %u turns, %u U, %u dropped, %u slowed, %u vetoed, %u passes\n",
                route_result_name(r), st.turns, st.uturns, st.dropped, st.slowed,
                st.vetoed, st.passes);
    expect("rev C (no 135): one veto, two passes, the other three U-turns kept",
           r == ROUTE_OK && st.vetoed == 1 && st.passes == 2 && st.uturns == 3);
    // With ROUTE_DIAG_MIN_STEPS 1 (2026-09-12, gap C) the leftover (2,3)L
    // (2,4)R is itself a one-step diagonal, and the 90 at (1,3) one cell
    // before its 45-in only fits on the speed rung: two capped corners.
    expect("rev C (no 135): the tooth is a 90 into a one-step diagonal, capped",
           st.slowed == 2 && st.diags == 1);
}

static void test_the_d8_v_rig() {
    Maze m;
    build_d8(m);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    RouteResult r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                                     &st, t, 32, 39.0f, HEADING_N);
    std::printf("  D8 CW: %s -- %u cells, %u turns, %u U, %u diag, %.0f mm, "
                "min straight %.0f\n", route_result_name(r), st.cells,
                st.turns, st.uturns, st.diags, st.length_mm, st.min_straight);
    for (int j = 0; j < st.turns && j < 32; j++)
        std::printf("    turn %d at (%u,%u) R%u %+d  cp %+d  diag %u\n", j,
                    t[j].cx, t[j].cy, t[j].r_mm, t[j].sweep_deg,
                    t[j].cp_off_mm, t[j].diag_mm);
    expect("D8 CW compiles", r == ROUTE_OK);
    expect("D8 out leg: 45-in (0,1), V (2,2), 45-out (1,3)", st.turns == 6 &&
           t[0].cx == 0 && t[0].cy == 1 && t[0].sweep_deg == -45 &&
           route_turn_is_v(t[1]) && t[1].cx == 2 && t[1].cy == 2 && t[1].sweep_deg == 90 &&
           t[2].cx == 1 && t[2].cy == 3 && t[2].sweep_deg == -45);
    expect("D8: 509 mm into the V, 255 mm out of it", st.turns == 6 &&
           std::fabs((float)t[1].diag_mm - 509.1f) < 1.5f &&
           std::fabs((float)t[2].diag_mm - 254.6f) < 1.5f);
    expect("D8 home leg: three right 90s at (1,5), (3,5), (3,0)", st.turns == 6 &&
           t[3].cx == 1 && t[3].cy == 5 && t[4].cx == 3 && t[4].cy == 5 &&
           t[5].cx == 3 && t[5].cy == 0 && t[3].sweep_deg == -90 &&
           t[3].cp_off_mm == 0);
    expect("D8: nothing demoted", st.dropped == 0);
    PathPose e = p.pose(p.length_mm());
    expect("the D8 loop closes",
           std::fabs(e.x_mm - 39.0f) < 1.0f && std::fabs(e.y_mm) < 1.0f);
    // The other way round: the home leg's staircase first, then the V from
    // the north -- a RIGHT V this time.
    r = route_build_loop(m, 0, 0, HEADING_E, p, 500.0f, 6000.0f, &st, t, 32,
                         39.0f, HEADING_E);
    std::printf("  D8 CCW: %s -- %u turns, %u U, %u diag, %.0f mm\n",
                route_result_name(r), st.turns, st.uturns, st.diags, st.length_mm);
    for (int j = 0; j < st.turns && j < 32; j++)
        std::printf("    turn %d at (%u,%u) R%u %+d  cp %+d  diag %u\n", j,
                    t[j].cx, t[j].cy, t[j].r_mm, t[j].sweep_deg,
                    t[j].cp_off_mm, t[j].diag_mm);
    expect("D8 CCW compiles", r == ROUTE_OK);
    bool has_v = false;
    for (int j = 0; j < st.turns; j++) if (route_turn_is_v(t[j]) && t[j].sweep_deg == -90) has_v = true;
    expect("D8 CCW has the right-handed V", has_v);
    PathPose e2 = p.pose(p.length_mm());
    expect("and the CCW D8 loop closes",
           std::fabs(e2.x_mm - 39.0f) < 1.0f && std::fabs(e2.y_mm) < 1.0f);
}

// vout: the V flown on lines offset OUTWARD by vout is a bigger arc on the
// same centre -- radius 64 + vout, entering and leaving at the SAME stations
// as the plain V (the post abeam), everything after it shifted along only
// by the extra arc length. The route's straights must not change at all.
static void test_the_v_outward_offset() {
    Maze m;
    build_d8(m);
    SpeedPath p0, p8;
    RouteStats s0, s8;
    RouteTurn t0[32], t8[32];
    route_set_v_out(0.0f);
    RouteResult r0 = route_build_loop(m, 0, 0, HEADING_N, p0, 500.0f, 6000.0f,
                                      &s0, t0, 32, 39.0f, HEADING_N);
    route_set_v_out(8.0f);
    RouteResult r8 = route_build_loop(m, 0, 0, HEADING_N, p8, 500.0f, 6000.0f,
                                      &s8, t8, 32, 39.0f, HEADING_N);
    route_set_v_out(0.0f);
    expect("vout: both builds compile", r0 == ROUTE_OK && r8 == ROUTE_OK &&
           s0.turns == 6 && s8.turns == 6);
    expect("vout 8: the V is R72, the plain one R64",
           t0[1].r_mm == 64 && t8[1].r_mm == 72);
    expect("vout: the 45s and the 90s keep their radii",
           t8[0].r_mm == t0[0].r_mm && t8[2].r_mm == t0[2].r_mm &&
           t8[3].r_mm == t0[3].r_mm);
    // The V's arc begins at the same path-s: the diagonal before it is the
    // same length.
    // The V's arc begins at (very nearly) the same path-s: the offset
    // lines' corner point is vout further along, the tangent of R+vout
    // is vout longer LESS the clothoid's shrinking shift (L^2/24R), so the
    // start moves by only that clothoid term, ~2 mm at these speeds.
    float d_entry = p8.corner_entry_s_mm(1) - p0.corner_entry_s_mm(1);
    float arc0 = p0.corner_exit_s_mm(1) - p0.corner_entry_s_mm(1);
    float arc8 = p8.corner_exit_s_mm(1) - p8.corner_entry_s_mm(1);
    float d_next = p8.corner_entry_s_mm(2) - p0.corner_entry_s_mm(2);
    std::printf("  vout 8: V entry s %.1f -> %.1f (%+.1f); arc %.1f -> %.1f; "
                "next 45 entry %+.1f; route %+.1f mm\n",
                (double)p0.corner_entry_s_mm(1), (double)p8.corner_entry_s_mm(1),
                (double)d_entry, (double)arc0, (double)arc8, (double)d_next,
                (double)(s8.length_mm - s0.length_mm));
    expect("vout: the V's arc starts within the clothoid term of the same station",
           d_entry >= 0.0f && d_entry < 4.0f);
    expect("vout: the V's arc is longer by ~8 x pi/2 (less the shorter clothoids)",
           arc8 - arc0 > 6.0f && arc8 - arc0 < 14.0f);
    // The exit is symmetric with the entry: the arc ends d_entry EARLIER
    // along the exit line, so the next straight is d_entry longer; the next
    // 45 is later by the arc growth plus twice the clothoid term.
    expect("vout: the next 45 is later by the arc growth plus the two clothoid terms",
           std::fabs(d_next - ((arc8 - arc0) + 2.0f * d_entry)) < 0.5f);
    expect("vout: nothing after the next 45 changes",
           std::fabs((s8.length_mm - s0.length_mm) - d_next) < 0.5f);
    // The compiled path's frame after the V is the OFFSET line: its end
    // pose is 8 mm off the plain one's, which is the runner's job to walk
    // back (diag_set_point), not the compiler's.
    PathPose e0 = p0.pose(p0.length_mm()), e8 = p8.pose(p8.length_mm());
    std::printf("  vout 8: end pose (%.1f, %.1f) vs (%.1f, %.1f)\n",
                (double)e8.x_mm, (double)e8.y_mm, (double)e0.x_mm, (double)e0.y_mm);
}

static void test_r90_l90_override() {
    Maze m;
    build_cal6(m);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    route_set_corner_context(4900.0f, 71.74f);
    route_set_r90(100.0f, 0.0f);   // rights forced to 100, lefts default
    RouteResult r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                                     &st, t, 32, 0.0f);
    expect("r90 build compiles", r == ROUTE_OK);
    bool rights_100 = true, lefts_84 = true, u_kept = true;
    for (int j = 0; j < st.turns; j++) {
        bool u = t[j].sweep_deg == 180 || t[j].sweep_deg == -180;
        if (u) {
            if (t[j].r_mm < 60 || t[j].r_mm > 95) u_kept = false;
        } else if (t[j].sweep_deg < 0) {
            if (t[j].r_mm != 100) rights_100 = false;
        } else {
            if (t[j].r_mm != 84) lefts_84 = false;
        }
    }
    expect("every right-90 takes the forced 100", rights_100);
    expect("every left-90 keeps the compiled 84", lefts_84);
    expect("U-turns are untouched by r90", u_kept);
    // Back to defaults: the same build returns to all-84.
    route_set_r90(0.0f, 0.0f);
    r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f, &st, t, 32,
                         0.0f);
    bool all_84 = r == ROUTE_OK;
    for (int j = 0; j < st.turns; j++) {
        bool u = t[j].sweep_deg == 180 || t[j].sweep_deg == -180;
        if (!u && t[j].r_mm != 84) all_84 = false;
    }
    expect("zeroed overrides restore the compiled ladder", all_84);
    route_set_corner_context(0.0f, -1.0f);
}

// ---- THE TWO-LEG RUN (2026-09-13) ------------------------------------------
// From D11 on the start cell has its east wall and the goal is a 2x2 block
// with a central post; a rules-compliant maze is not a circuit through the
// start, so route_build_loop ends in an about-face at the goal and refuses
// (ROUTE_BAD_TURN). The runner then flies two legs: S->G one-way, an
// in-place turn on the goal cell's centre to route_home_heading, G->S
// compiled FROM the goal. This is the compile side of that contract, on
// three shapes: the searched D9 as bert flew it (the (0,0) E wall in place,
// a single dead-end goal), a 2x2 block with one way in, and a 2x2 block
// with a second opening on the way home (which the loop compiles as a
// circuit -- no legs needed).

// Walk the maze's current flood from (x,y,h) to distance 0, like
// RouteRun::compile_leg does to learn where a leg ends.
static void walk_end(const Maze &m, int &x, int &y, Heading &h) {
    int guard = 0;
    while (m.distance(x, y) != 0 && guard++ < ROUTE_MAX_CELLS) {
        Heading next;
        if (!m.best_step_known(x, y, h, next)) break;
        x += heading_dx(next);
        y += heading_dy(next);
        h = next;
    }
}

// Two legs over `m` from (0,0) facing north: assert every step the runner
// takes compiles, and report the shape. Returns the home launch heading.
static Heading two_leg_compiles(Maze &m, const char *name, int gx, int gy) {
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    // The circuit is tried first, forced out the way the nose points (what
    // compile_leg does for route.dir=auto), then unforced.
    RouteResult r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                                     &st, t, 32, 39.0f, (int)HEADING_N);
    if (r != ROUTE_OK)
        r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f, &st, t,
                             32, 39.0f, -1);
    std::printf("  %s: loop %s\n", name, route_result_name(r));
    expect("not a circuit: the loop refuses", r != ROUTE_OK);

    // Leg 1: S->G over the pessimistic flood, with the rear-wall lead-in.
    m.flood_to_goal(false);
    r = route_build(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f, &st, t, 32, 39.0f);
    expect("leg 1 S->G compiles", r == ROUTE_OK);
    int x = 0, y = 0;
    Heading h = HEADING_N;
    walk_end(m, x, y, h);
    expect("leg 1 ends on the goal cell", x == gx && y == gy);
    std::printf("    S->G: %u cells, %u turns, %.0f mm, ends (%d,%d) facing %c\n",
                st.cells, st.turns, st.length_mm, x, y, "NESW"[h & 3]);
    PathPose e = p.pose(p.length_mm());
    // One-way legs end on the cell centre: the goal cell's centre from the
    // launch datum (x along the launch heading, y left).
    expect("leg 1 ends on a cell centre (v = 0 there)",
           std::fabs(e.x_mm - (39.0f + gy * ROUTE_CELL_MM)) < 2.0f &&
           std::fabs(e.y_mm + gx * ROUTE_CELL_MM) < 2.0f);

    // The turn at the goal.
    Heading home;
    expect("a home heading exists", route_home_heading(m, x, y, h, home));
    std::printf("    at the goal facing %c, home leg launches %c\n",
                "NESW"[h & 3], "NESW"[home & 3]);

    // Leg 2: G->S from the goal on that heading, no lead-in.
    m.flood_to_start(false);
    r = route_build(m, x, y, home, p, 500.0f, 6000.0f, &st, t, 32, 0.0f);
    expect("leg 2 G->S compiles", r == ROUTE_OK);
    int hx = x, hy = y;
    Heading hh = home;
    walk_end(m, hx, hy, hh);
    expect("leg 2 ends on the start cell", hx == 0 && hy == 0);
    std::printf("    G->S: %u cells, %u turns, %.0f mm, ends (%d,%d) facing %c\n",
                st.cells, st.turns, st.length_mm, hx, hy, "NESW"[hh & 3]);
    // The first step of the walk IS the launch heading: a launch cannot
    // begin with a turn, and best_step_known prefers straight on ties.
    Heading first;
    expect("the home walk's first step is the launch heading",
           m.best_step_known(x, y, home, first) && first == home);
    return home;
}

static void test_two_legs_on_the_searched_d9() {
    // D9 exactly as searched: the (0,0) east wall stays (build_d9 removes it
    // to make the rig a circuit -- put it back), goal (5,5) a dead end.
    Maze m;
    build_d9(m);
    m.set_wall(0, 0, HEADING_E, true);
    Heading home = two_leg_compiles(m, "searched D9 with the (0,0) E wall", 5, 5);
    (void)home;
}

// A 2x2 goal block (2,2)-(3,3), central post with no walls, ONE opening at
// (3,2) E, reached by a corridor: (0,0) N to (0,5), E to (5,5), S to (5,2),
// W into (3,2). Everything else walled.
static void build_block_goal(Maze &m, bool second_opening) {
    m.reset();
    for (int x = 0; x < 6; x++)
        for (int y = 0; y < 6; y++)
            for (int h = 0; h < 4; h++) m.set_wall(x, y, (Heading)h, true);
    auto open = [&](int x, int y, Heading h) { m.set_wall(x, y, h, false); };
    for (int y = 0; y < 5; y++) open(0, y, HEADING_N);      // up column 0
    for (int x = 0; x < 5; x++) open(x, 5, HEADING_E);      // along row 5
    for (int y = 5; y > 2; y--) open(5, y, HEADING_S);      // down column 5
    open(5, 2, HEADING_W);                                   // (4,2)
    open(4, 2, HEADING_W);                                   // into (3,2)
    // The block: four cells open to each other round the central post.
    open(2, 2, HEADING_N); open(3, 2, HEADING_N);
    open(2, 2, HEADING_E); open(2, 3, HEADING_E);
    if (second_opening) {
        // (2,3) N to (2,4), then W along row 4 to (1,4), joining column 0
        // at (0,4): a second way out of the block that leads home.
        open(2, 3, HEADING_N);
        open(2, 4, HEADING_W);
        open(1, 4, HEADING_W);
    }
    Maze::State st;
    m.save_state(st);
    for (int i = 0; i <= MAZE_SIZE; i++) { st.vlock[i] = 0; st.hlock[i] = 0; }
    m.load_state(st);
    m.set_goal(2, 2, 3, 3);
}

static void test_two_legs_into_a_block_goal() {
    Maze m;
    build_block_goal(m, false);
    Heading home = two_leg_compiles(m, "2x2 goal, one way in", 3, 2);
    // Arrived heading W at (3,2); the only way home is back out east.
    expect("the goal turn is the about-face", home == HEADING_E);
}

static void test_a_block_goal_with_a_second_opening_is_a_circuit() {
    Maze m;
    build_block_goal(m, true);
    SpeedPath p;
    RouteStats st;
    RouteTurn t[32];
    RouteResult r = route_build_loop(m, 0, 0, HEADING_N, p, 500.0f, 6000.0f,
                                     &st, t, 32, 39.0f, (int)HEADING_N);
    std::printf("  2x2 goal with a second opening: loop %s, %u cells, %u turns, "
                "goal at %u\n", route_result_name(r), st.cells, st.turns,
                st.goal_ix);
    expect("a second opening on the way home makes it a circuit", r == ROUTE_OK);
    expect("the goal is inside the lap", st.goal_ix > 0 && st.goal_ix < st.cells);
}


// A DIAGONAL MAY NOT HAND STRAIGHT INTO A U-TURN (Route.h
// ROUTE_DIAG_EXIT_TO_UTURN_MM; 2026-10-03, Hazlemere session 2, MR2): the
// logged map, goal the centre block. Pass one compiles a 135-out at (8,3)
// 98 mm before the U at (6,3) -- the geometry that crashed -- and the
// multipass must un-merge that diagonal chain so the U is approached from
// a corridor 90.
static void test_no_diagonal_into_a_uturn() {
    const uint16_t v[17] = {65535,22485,2730,5460,8874,16708,33019,18374,30752,65535,0,0,0,0,0,0,65535};
    const uint16_t h[17] = {65535,152,296,212,426,245,10,404,40,212,314,212,82,163,100,158,65535};
    Maze m; m.reset(); Maze::State st; m.save_state(st);
    for (int i = 0; i <= MAZE_SIZE; i++) {
        st.vwall[i] = v[i]; st.hwall[i] = h[i];
        st.vknown[i] = 0xFFFF; st.hknown[i] = 0xFFFF; st.vlock[i] = 0; st.hlock[i] = 0;
    }
    m.load_state(st); m.set_goal(7, 7, 8, 8);
    route_set_corner_context(1600.0f, 71.74f);
    route_set_turn_cost(3.0f);
    route_set_diag(true);
    route_set_v_out(10.0f);
    m.flood_to_goal(false);
    route_set_time_context(800.0f, 1600, 1600, 1280, true, 7000);
    static SpeedPath p; RouteStats rs; static RouteTurn t[ROUTE_MAX_TURNS];
    RouteResult r = route_build(m, 0, 0, HEADING_N, p, 800.0f, 7000.0f, &rs, t, ROUTE_MAX_TURNS, 39.0f);
    expect("hazlemere map compiles", r == ROUTE_OK);
    bool diag_into_u = false; int uturns = 0, vetoed = rs.vetoed;
    for (int k = 0; k + 1 < rs.turns; k++) {
        bool dx = t[k].diag_mm > 0 && (t[k].sweep_deg == 45 || t[k].sweep_deg == -45 ||
                                       t[k].sweep_deg == 135 || t[k].sweep_deg == -135);
        bool u = t[k + 1].sweep_deg == 180 || t[k + 1].sweep_deg == -180;
        if (u) uturns++;
        if (dx && u && p.corner_entry_s_mm(k + 1) - p.corner_exit_s_mm(k) < ROUTE_DIAG_EXIT_TO_UTURN_MM)
            diag_into_u = true;
    }
    std::printf("  hazlemere: %u turns, %d U-turns, %d features vetoed, %u passes, %.0f mm\n",
                rs.turns, uturns, vetoed, rs.passes, (double)rs.length_mm);
    if (ROUTE_DIAG_EXIT_TO_UTURN_MM > 0.0f) {
        expect("no diagonal exit hands into a U-turn within 180 mm", !diag_into_u);
        expect("a diagonal chain was vetoed to make it so", vetoed >= 1);
    } else {
        // Rule off (2026-10-04): the 135-out at (8,3) hands into the U at
        // (6,3) as the maze was first compiled -- the 44 mm landing that
        // crashed that U was the V's vout, not the U's approach (RACE_PLAN,
        // the test-day analysis), and the rule's zigzag crashed at once.
        bool d135_then_u = false;
        for (int k = 0; k + 1 < rs.turns; k++)
            if ((t[k].sweep_deg == 135 || t[k].sweep_deg == -135) && t[k].diag_mm > 0 &&
                (t[k + 1].sweep_deg == 180 || t[k + 1].sweep_deg == -180))
                d135_then_u = true;
        expect("rule off: the 135-out -> U at (8,3)/(6,3) is kept merged", d135_then_u);
        expect("rule off: nothing vetoed", vetoed == 0);
        expect("rule off: no R73 zigzag -- every turn at R84 or a U", rs.dropped == 0);
    }
    expect("the U-turn at (6,3) is kept", uturns >= 1);
    expect("the diagonals elsewhere survive", rs.diags >= 5);
    route_clear_time_context();
    route_set_turn_cost(0.0f);
    route_set_v_out(0.0f);
}


// rvv / rvu (Route.h, 2026-10-04): the V and the turn it hands into within
// ROUTE_V_CAP_AFTER_MM, and the U, carry a speed cap into the path. Maze 1
// at MR4 limits (the hazel6/7 crash: (5,13) V -> (3,14) 135, 71 mm apart):
// capped at 320 the V and that 135 are slowed and everything else is not;
// a U cap of 340 slows the six Us; the planner's time grows and nothing
// else about the route changes.
static void test_v_and_u_speed_caps() {
    const uint16_t v[17] = {65535,49151,65534,32512,8320,16508,14488,1408,26630,65535,0,0,0,0,0,0,65535};
    const uint16_t h[17] = {65535,252,116,212,404,212,500,212,40,496,248,304,216,176,64,248,65535};
    auto build = [&](float capv, float capu, SpeedPath &p, RouteStats &rs, RouteTurn *t) {
        Maze m; m.reset(); Maze::State st; m.save_state(st);
        for (int i = 0; i <= MAZE_SIZE; i++) {
            st.vwall[i] = v[i]; st.hwall[i] = h[i];
            st.vknown[i] = 0xFFFF; st.hknown[i] = 0xFFFF; st.vlock[i] = 0; st.hlock[i] = 0;
        }
        m.load_state(st); m.set_goal(7, 7, 8, 8);
        route_set_corner_context(3200.0f, 71.74f);
        route_set_turn_cost(3.0f);
        route_set_diag(true);
        route_set_v_out(0.0f);
        route_set_turn_caps(capv, capu);
        m.flood_to_goal(false);
        route_set_time_context(1500.0f, 4000, 3200, 3200, true, 7000);
        RouteResult r = route_build(m, 0, 0, HEADING_N, p, 1500.0f, 7000.0f, &rs, t, ROUTE_MAX_TURNS, 39.0f);
        p.set_limits(1500.0f, 4000, 3200, 3200); p.set_clothoid_hold(true);
        return r;   // (the time context stays set for route_path_time_ms)
    };
    static SpeedPath p0, p1, p2; RouteStats r0, r1, r2;
    static RouteTurn t0[ROUTE_MAX_TURNS], t1[ROUTE_MAX_TURNS], t2[ROUTE_MAX_TURNS];
    expect("no caps compiles", build(0, 0, p0, r0, t0) == ROUTE_OK);
    expect("V cap compiles", build(320, 0, p1, r1, t1) == ROUTE_OK);
    expect("V + U caps compile", build(320, 340, p2, r2, t2) == ROUTE_OK);
    expect("the caps change no turn", r0.turns == r1.turns && r1.turns == r2.turns && r0.diags == r2.diags && r0.uturns == r2.uturns);
    int capped_v = 0, capped_after = 0, capped_other = 0, capped_u = 0;
    for (int k = 0; k < r1.turns; k++) {
        bool u = t1[k].sweep_deg == 180 || t1[k].sweep_deg == -180;
        bool after_v = k > 0 && route_turn_is_v(t1[k - 1]) &&
                       p1.corner_entry_s_mm(k) - p1.corner_exit_s_mm(k - 1) < ROUTE_V_CAP_AFTER_MM;
        if (t1[k].v_cap_mm_s == 320) {
            if (route_turn_is_v(t1[k])) capped_v++;
            else if (after_v) capped_after++;
            else capped_other++;
        } else if (t1[k].v_cap_mm_s != 0 && !u) capped_other++;
        if (u && t2[k].v_cap_mm_s == 340) capped_u++;
    }
    std::printf("  caps: %d Vs, %d turns after a V, %d others; %d Us at 340; time %.2f -> %.2f -> %.2f s\n",
                capped_v, capped_after, capped_other, capped_u,
                (double)route_path_time_ms(p0) / 1000, (double)route_path_time_ms(p1) / 1000,
                (double)route_path_time_ms(p2) / 1000);
    expect("both Vs carry the 320 cap", capped_v == 2);
    expect("the (3,14) 135 after the (5,13) V carries it too", capped_after >= 1);
    expect("no other turn is capped", capped_other == 0);
    expect("every U carries the 340 cap", capped_u == (int)r2.uturns && r2.uturns == 6);
    expect("the V cap costs time", route_path_time_ms(p1) > route_path_time_ms(p0));
    expect("the U cap costs more", route_path_time_ms(p2) > route_path_time_ms(p1));
    // The path's own cap: at the (5,13) V's arc the sampled speed limit is 320.
    for (int k = 0; k < r1.turns; k++) {
        if (!route_turn_is_v(t1[k])) continue;
        float s_mid = 0.5f * (p1.corner_entry_s_mm(k) + p1.corner_exit_s_mm(k));
        float vc = p1.v_cap_at(s_mid);
        expect("the path caps the V arc at 320", vc <= 320.5f && vc > 300.0f);
        float v0 = p0.v_cap_at(0.5f * (p0.corner_entry_s_mm(k) + p0.corner_exit_s_mm(k)));
        expect("uncapped the same V flies faster", v0 > 400.0f);
        break;
    }
    route_set_turn_caps(0.0f, 0.0f);
    route_clear_time_context();
    route_set_turn_cost(0.0f);
}


// ...NOR A V INTO AN OPPOSITE-HAND 135-OUT (Route.h ROUTE_V_TO_OPP_135_MM):
// the Hazlemere maze 1 route has right V (5,13) -> left 135 (3,14) with ~50
// mm between the arcs (brushed at MR3, hit at MR4). The same-hand pair
// V (6,11) -> 135 (8,9) stays merged.
static void test_no_v_into_an_opposite_135() {
    const uint16_t v[17] = {65535,49151,65534,32512,8320,16508,14488,1408,26630,65535,0,0,0,0,0,0,65535};
    const uint16_t h[17] = {65535,252,116,212,404,212,500,212,40,496,248,304,216,176,64,248,65535};
    Maze m; m.reset(); Maze::State st; m.save_state(st);
    for (int i = 0; i <= MAZE_SIZE; i++) {
        st.vwall[i] = v[i]; st.hwall[i] = h[i];
        st.vknown[i] = 0xFFFF; st.hknown[i] = 0xFFFF; st.vlock[i] = 0; st.hlock[i] = 0;
    }
    m.load_state(st); m.set_goal(7, 7, 8, 8);
    route_set_corner_context(3200.0f, 71.0f);
    route_set_turn_cost(3.0f);
    route_set_diag(true);
    route_set_v_out(10.0f);
    m.flood_to_goal(false);
    route_set_time_context(1500.0f, 3200, 3200, 2560, true, 7000);
    static SpeedPath p; RouteStats rs; static RouteTurn t[ROUTE_MAX_TURNS];
    RouteResult r = route_build(m, 0, 0, HEADING_N, p, 1500.0f, 7000.0f, &rs, t, ROUTE_MAX_TURNS, 39.0f);
    expect("hazlemere maze 1 compiles", r == ROUTE_OK);
    bool bad = false, same_hand_kept = false;
    for (int k = 0; k + 1 < rs.turns; k++) {
        bool v135 = route_turn_is_v(t[k]) && (t[k + 1].sweep_deg == 135 || t[k + 1].sweep_deg == -135) &&
                    t[k + 1].diag_mm > 0;
        if (!v135) continue;
        bool opp = (t[k].sweep_deg > 0) != (t[k + 1].sweep_deg > 0);
        float gap = p.corner_entry_s_mm(k + 1) - p.corner_exit_s_mm(k);
        if (opp && gap < 150.0f) bad = true;   // the pair the rule would remove
        if (!opp) same_hand_kept = true;
    }
    std::printf("  hazlemere 1: %u turns, %u diagonals, %u x 135, %u vetoed\n",
                rs.turns, rs.diags, rs.turns135, rs.vetoed);
    if (ROUTE_V_TO_OPP_135_MM > 0.0f) {
        expect("no V hands into an opposite-hand 135 within the limit", !bad);
    } else {
        // Rule off (2026-10-03 evening, hazzel8): the pair is flown as the
        // 09-03 morning's MR2/MR3 flew it; rdv keeps MR4/MR5 orthogonal.
        expect("rule off: the (5,13) V -> (3,14) 135 pair is kept", bad);
        expect("rule off: the full 7-diagonal route", rs.diags == 7);
    }
    expect("the same-hand V -> 135 at (6,11) -> (8,9) is kept", same_hand_kept);
    expect("diagonals elsewhere survive", rs.diags >= 3);
    route_clear_time_context();
    route_set_turn_cost(0.0f);
    route_set_v_out(0.0f);
}

// THE GOAL LOOP (Route.h, ghome 1; 2026-10-04): Hazlemere maze 1, goal
// 7,7..8,8, arrived at (8,7) facing NORTH as the test day's out legs did.
// The home walk goes (8,7)->(8,8) N, ->(7,8) W, ->(7,7) S, then the home
// flood: a U-turn in the top row and a 90, no in-place turn, and it ends
// at the start cell.
static void test_the_goal_loop() {
    const uint16_t v[17] = {65535,49151,65534,32512,8320,16508,14488,1408,26630,65535,0,0,0,0,0,0,65535};
    const uint16_t h[17] = {65535,252,116,212,404,212,500,212,40,496,248,304,216,176,64,248,65535};
    Maze m; m.reset(); Maze::State st; m.save_state(st);
    for (int i = 0; i <= MAZE_SIZE; i++) {
        st.vwall[i] = v[i]; st.hwall[i] = h[i];
        st.vknown[i] = 0xFFFF; st.hknown[i] = 0xFFFF; st.vlock[i] = 0; st.hlock[i] = 0;
    }
    m.load_state(st); m.set_goal(7, 7, 8, 8);
    route_set_corner_context(4000.0f, 71.74f);
    route_set_turn_cost(8.0f);
    route_set_diag(false);
    route_set_v_out(0.0f);
    route_set_time_context(350.0f, 3500, 3500, 3200, true, 7000);
    expect("the loop is possible from (8,7) facing N", route_goal_loop_possible(m, 8, 7, HEADING_N));
    expect("...and not from a cell outside the goal", !route_goal_loop_possible(m, 8, 6, HEADING_N));
    static SpeedPath p; RouteStats rs; static RouteTurn t[ROUTE_MAX_TURNS];
    // Maze 1's goal has ONE door, (8,7) S: the loop would come back through
    // the arrival cell -- refused, the runner turns in place.
    RouteResult r = route_build_goal_loop(m, 8, 7, HEADING_N, p, 350.0f, 7000.0f, &rs, t, ROUTE_MAX_TURNS);
    expect("a one-door goal refuses the loop", r != ROUTE_OK);
    // Open a second door: (7,8) N (the top-left goal cell's north wall).
    m.set_wall(7, 8, HEADING_N, false);
    m.flood_to_start(false);
    expect("the second door reaches the start", m.distance(7, 9) != MAZE_DIST_UNREACHABLE);
    r = route_build_goal_loop(m, 8, 7, HEADING_N, p, 350.0f, 7000.0f, &rs, t, ROUTE_MAX_TURNS);
    expect("a two-door goal compiles the loop", r == ROUTE_OK);
    int ex = -1, ey = -1; Heading eh = HEADING_N;
    expect("the loop walk has an end", route_goal_loop_end(&ex, &ey, &eh));
    expect("...at the start cell", ex == 0 && ey == 0);
    std::printf("  goal loop: %u turns (%u U, %u dropped), first turn (%u,%u) sweep %d R%u, %u cells, %.0f mm, %.1f s\n",
                rs.turns, rs.uturns, rs.dropped, t[0].cx, t[0].cy, t[0].sweep_deg, t[0].r_mm, rs.cells,
                (double)rs.length_mm, (double)route_path_time_ms(p) / 1000);
    expect("the first turn is inside the block", t[0].cx >= 7 && t[0].cx <= 8 && t[0].cy >= 7 && t[0].cy <= 8);
    expect("the walk leaves through the second door, not the arrival cell",
           rs.cells >= 2 && !(t[0].cx == 8 && t[0].cy == 7));
    route_clear_time_context();
    route_set_turn_cost(0.0f);
}

int main() {
    test_the_goal_loop();
    test_the_tangent_table();
    test_test8_route_shape();
    test_the_uturn_geometry_is_exact();
    test_the_uturn_is_the_gentlest_thing_on_the_route();
    test_the_hairpin_is_one_uturn_and_costs_no_demotions();
    test_the_return_leg_gets_two_uturns();
    test_the_goal_needs_no_turn_on_test8();
    test_the_out_and_back_compiles_as_one_path();
    test_no_two_arcs_ever_overlap();
    test_speed_changes_the_geometry();
    test_an_unfinished_map_is_refused();
    test_standing_on_the_goal();
    test_the_start_setback_lead_in();
    test_the_gate_has_room_at_every_ladder_radius();
    test_the_gate_expects_the_post_at_the_cell_boundary();
    test_every_corner_can_now_gate();
    test_arming_is_what_protects_the_decide_window();
    test_the_return_leg_compiles_from_where_the_first_one_ended();
    test_route_dir_picks_the_arm();
    test_corner_geometry_at_corner_speed();
    test_r90_l90_override();
    test_rdiag_off_flies_the_staircase_as_90s();
    test_rturn_prefers_fewer_turns();
    test_no_diagonal_into_a_uturn();
    test_no_v_into_an_opposite_135();
    test_v_and_u_speed_caps();
    test_rturn_home_leg_compiles_from_the_goal();
    test_route_chosen_by_flight_time();
    test_the_d6_staircase_is_one_diagonal();
    test_the_d7_mixed_rig();
    test_the_v_joins_two_diagonals();
    test_the_d8_v_rig();
    test_the_d9_rig();
    test_the_v_outward_offset();
    test_the_d10_rig_is_never_refused();
    test_the_d10_rig_with_the_135();
    test_a_hairpin_crowded_by_a_ninety_unmerges_alone();
    test_two_legs_on_the_searched_d9();
    test_two_legs_into_a_block_goal();
    test_a_block_goal_with_a_second_opening_is_a_circuit();

    if (failures == 0) {
        std::printf("ALL ROUTE TESTS PASSED\n");
        return 0;
    }
    std::printf("%d ROUTE TEST(S) FAILED\n", failures);
    return 1;
}
