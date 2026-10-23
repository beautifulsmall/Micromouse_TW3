#include "Route.h"

#include <string.h>

#include <math.h>

const char *route_result_name(RouteResult r) {
    switch (r) {
        case ROUTE_OK: return "ok";
        case ROUTE_NO_PATH: return "no known route";
        case ROUTE_TOO_LONG: return "route too long";
        case ROUTE_BAD_TURN: return "not a 90 degree turn";
        case ROUTE_NO_RADIUS: return "no radius fits a turn pair";
        case ROUTE_PATH_FULL: return "path out of segments";
    }
    return "?";
}

float route_tangent_mm(float radius_mm, float v_mm_s, float slew_dps2,
                       float sweep_deg) {
    float L = SpeedPath::clothoid_for_rail(radius_mm, v_mm_s, slew_dps2);
    return SpeedPath::tangent_for(radius_mm, L, sweep_deg);
}

// The setback a U-turn needs before the cell centre it is named by.
//
// Measured from the SWEPT ENVELOPE, not the centreline: the outermost point
// of the mouse on an arc is its outer front corner, at
// sqrt((R + half_width)^2 + front^2) from the arc centre. The arc centre sits
// `setback` south of the turn cell centre and the far wall face is
// ROUTE_WALL_FACE_MM north of it, so
//
//     setback + WALL_FACE - outer_sweep >= MARGIN
//
// Runs 168-171 used the centreline apex plus the half-width and scraped the
// wall on every pass. See the note in Route.h.
static float uturn_outer_sweep(float radius_mm) {
    float a = radius_mm + ROUTE_BODY_HALF_MM;
    return sqrtf(a * a + ROUTE_BODY_HALF_LEN_MM * ROUTE_BODY_HALF_LEN_MM);
}

// HOW FAR FORWARD THE BODY ACTUALLY REACHES, measured from the primitive's
// ENTRY. This is the number the setback needs, and it is not the one above.
//
// uturn_outer_sweep is a radius about the ARC CENTRE. The setback is measured
// from the entry. Those two points are not the same: the entry clothoid pushes
// the path forward before the constant-radius arc begins, so the arc centre
// sits (extent - R) ahead of the entry. At the operating point that is
//
//     extent 103.0 - R 89.67 = 13.3 mm
//
// and the old setback dropped it. Every U-turn this mouse has ever flown was
// therefore placed 13.3 mm later than the clearance calculation believed --
// which is bert's "it's still not turning soon enough", sitting in the
// arithmetic the whole time and not in the gate, the beam lead or the maze.
//
// It matters more than 13 mm suggests, because it grows with the clothoid. A
// tighter U-turn buys its lost offset back from clothoid shift, and at R 71.74
// the term is 86.6 mm -- which is why the tighter geometry needs a 155 mm
// setback rather than a smaller one, and why tightening the radius is the
// wrong direction. Numerically, over the solved family:
//
//     R 89.67 (rail 6000)  reach 145.9  setback 98.9   <- the best there is
//     R 88.64 (rail 3000)  reach 158.5  setback 111.5
//     R 77.84 (rail 1200)  reach 195.8  setback 148.8
//     R 72.32 (rail 1100)  reach 202.0  setback 155.0
//
// Closed form rather than a sweep of the built path: the maximum of
// x(s) + half_len*|cos th| + half_width*|sin th| over the turn is reached
// while the body is on the constant-radius arc, where it is exactly the
// circumscribed radius about the arc centre. Checked against a numerical
// sweep of the same geometry -- 145.9 both ways at R89.67, 202.3 against
// 202.0 at R72.3 (the residual is the clothoid, where the closed form is a
// hair optimistic and the margin covers it).
static float uturn_body_reach(const SpeedPath::UTurn &u) {
    return (u.extent_mm - u.radius_mm) + uturn_outer_sweep(u.radius_mm);
}

// ONE place decides which U-turn the mouse flies. See Route.h.
// -- Corner-geometry context (route_set_corner_context) ----------------------
//
// SOLVE CORNER GEOMETRY AT CORNER SPEED, NOT ROUTE SPEED. Tangents and the
// U-turn solve took v_mm_s -- the ROUTE speed -- while the speed planner
// (SpeedPath::set_limits) caps every arc at sqrt(alat x R). At rv=500 the
// two are close and nobody noticed. At rv=1300 (2026-08-29 lap) the corner
// still flies at ~640 mm/s, but the geometry was being solved for 1300:
// tangents ballooned, the radius fit demoted R84s to R60/R45, and the
// U-turn geometry could not clear at all -- every 180 decomposed into two
// squared-off back-to-back 90s. Flown speed and solved speed should be the
// same number; alat here is what makes them agree. 0 = legacy (solve at
// route speed).
//
// uturn_r makes the compiled ROUTE_UTURN_RADIUS_MM force tunable (`rur`):
// -1 = use the compiled constant; 0 = solve the radius from the rail; >0 =
// force that radius, mm. Set-before-build state like control_set_path_slew:
// RouteRun sets it from the registry at every compile, so a stale value
// cannot outlive a leg.
static float ctx_alat = 0.0f;
static float ctx_uturn_r = -1.0f;
void route_set_corner_context(float alat_mm_s2, float uturn_r_mm) {
    ctx_alat = alat_mm_s2;
    ctx_uturn_r = uturn_r_mm;
}

// r90/l90 (bert, 2026-08-30): per-direction override of the 90-degree
// ladder's TOP rung. 0 = compiled ROUTE_RADII[0] (the measured-optimal 84).
// >0 replaces rung 0 for that direction only -- demotions still walk the
// compiled ladder below, and U-turns are untouched (theirs is rur). The
// lever is corner SPEED: the planner allows sqrt(alat x R) through an arc,
// so R100 vs R84 at alat 4900 is 700 vs 641 mm/s -- bought with clearance
// margin the compiler does not model. The parameter carries the risk.
static float ctx_r90_right = 0.0f;
static float ctx_r90_left = 0.0f;
void route_set_r90(float right_mm, float left_mm) {
    ctx_r90_right = right_mm;
    ctx_r90_left = left_mm;
}
static float ctx_v_out = 0.0f;   // vout: the V's outward offset (Route.h)
void route_set_v_out(float mm) { ctx_v_out = mm > 0.0f ? mm : 0.0f; }
static float ctx_cap_v = 0.0f, ctx_cap_u = 0.0f;   // rvv / rvu (Route.h)
void route_set_turn_caps(float v_cap_mm_s, float u_cap_mm_s) {
    ctx_cap_v = v_cap_mm_s > 0.0f ? v_cap_mm_s : 0.0f;
    ctx_cap_u = u_cap_mm_s > 0.0f ? u_cap_mm_s : 0.0f;
}
float route_v_cap() { return ctx_cap_v; }
float route_u_cap() { return ctx_cap_u; }
static bool ctx_135 = true;      // r135 (Route.h, THE 135)
void route_set_135(bool on) { ctx_135 = on; }
static bool ctx_diag = true;     // rdiag: 0 = orthogonal only (ControlGains::route_diag)
void route_set_diag(bool on) { ctx_diag = on; }
bool route_diag_enabled() { return ROUTE_DIAG_ENABLED && ctx_diag; }
bool route_135_enabled() { return ROUTE_135_ENABLED && ctx_135; }
static float corner_speed(float v_mm_s, float r_mm) {
    if (ctx_alat <= 0.0f || r_mm <= 0.0f) return v_mm_s;
    float vc = sqrtf(ctx_alat * r_mm);
    return vc < v_mm_s ? vc : v_mm_s;
}

bool route_uturn_geometry(float v_mm_s, float slew_dps2,
                          SpeedPath::UTurn *out, float *slew_out) {
    float force_r = ctx_uturn_r >= 0.0f ? ctx_uturn_r : ROUTE_UTURN_RADIUS_MM;
    if (force_r > 0.0f) {
        return SpeedPath::uturn_solve_radius(ROUTE_CELL_MM, force_r,
                                             corner_speed(v_mm_s, force_r),
                                             out, slew_out);
    }
    if (slew_out) *slew_out = slew_dps2;
    if (!SpeedPath::uturn_solve(ROUTE_CELL_MM, v_mm_s, slew_dps2, out))
        return false;
    // The radius is now known, so refine once at the speed that radius will
    // actually be flown -- the first solve only needed to discover R.
    float vc = corner_speed(v_mm_s, out->radius_mm);
    if (vc < v_mm_s)
        return SpeedPath::uturn_solve(ROUTE_CELL_MM, vc, slew_dps2, out);
    return true;
}

// Three terms, three different measurements, and they add because they are
// independent: the build margin, how late the mouse STARTS the turn (the
// whole shape shifts), and how far outside the arc it then FLIES it (the shape
// bulges). See Route.h.
static float uturn_setback(const SpeedPath::UTurn &u) {
    return uturn_body_reach(u) - ROUTE_WALL_FACE_MM + ROUTE_UTURN_MARGIN_MM +
           ROUTE_UTURN_LATE_MM + ROUTE_UTURN_TRACK_MM;
}

float route_uturn_setback_mm(const SpeedPath::UTurn &u) {
    return uturn_setback(u);
}

// Does this U-turn actually clear, at that setback? Asserted rather than
// assumed -- a turn that cannot prove it fits is refused, and route_build
// then falls back to two 90s. The extent term is what stops a setback so
// large that the mouse would still be turning when it should be gone.
static bool uturn_clears(const SpeedPath::UTurn &u, float setback_mm) {
    if (setback_mm <= 0.0f) return false;
    float outer = uturn_body_reach(u);
    // Far wall of the turn row, WITH the late start AND the outward bulge
    // spent: the check has to hold at the position the mouse actually reaches,
    // not the one the path asked for.
    //
    // BE HONEST ABOUT WHAT THIS TESTS. Given the setback uturn_setback()
    // returns, this expression is algebraically MARGIN exactly -- the two are
    // the same equation rearranged, so it can never fail on a setback this
    // function computed. It is a TRIPWIRE, not a clearance test: it fires if a
    // caller ever passes a hand-chosen setback, or if the two formulas drift
    // apart in a future edit. Adding ROUTE_UTURN_TRACK_MM to one and not the
    // other is exactly the drift it is for, and the 1e-3 is because the
    // tautology lands on the boundary and float rounding decides which side.
    if (ROUTE_WALL_FACE_MM + setback_mm - outer - ROUTE_UTURN_LATE_MM -
            ROUTE_UTURN_TRACK_MM <
        ROUTE_UTURN_MARGIN_MM - 1e-3f)
        return false;
    // Inside post: the path's nearest approach is at the entry and exit,
    // sqrt(half_cell^2 + (half_cell - setback)^2) from the post centre.
    float half = 0.5f * ROUTE_CELL_MM;
    float d = sqrtf(half * half + (half - setback_mm) * (half - setback_mm));
    return d - ROUTE_BODY_HALF_MM >= ROUTE_UTURN_MARGIN_MM;
}

// THE TURN-WEIGHTED WALK (`rturn`, 2026-09-26). The cell flood counts cells
// and nothing else, so when a wall moved on the new maze it sent MR3 down
// an 18-cell route with ELEVEN R45/R60 turns through the bottom zigzag --
// the crowded-90s geometry the diagonals were invented to escape, forced
// orthogonal by rdiag 0 -- instead of the 21-cell route with six turns it
// had flown all evening. The mouse hit the (2,2) wall on the fourth turn.
// A turn is not free: at 2000 mm/s a straight cell is 90 ms and an R84 90
// with its brake and launch is ~350 ms more than the straight it replaces,
// so a turn is worth three or four cells of time, and every turn is a
// place to hit a post. With rturn > 0 the walk is a Dijkstra over
// (cell, heading) with cost = cells + rturn x turns (a 180 counts twice),
// over KNOWN-open edges only, to the nearest target by that cost. 0 = the
// plain flood walk, as it always was.
static float ctx_turn_cost = 0.0f;
void route_set_turn_cost(float cells_per_turn) { ctx_turn_cost = cells_per_turn > 0.0f ? cells_per_turn : 0.0f; }
float route_turn_cost() { return ctx_turn_cost; }

// The target is whatever the caller's flood put at distance 0 -- the goal
// cells after flood_to_goal, the start after flood_to_start -- so every
// caller (the out leg, the loop's home walk, the two-leg home leg through
// route_build) targets the same thing the plain walk would. (The first cut
// took a to_start flag and the two-leg home leg, which reaches this through
// route_build, targeted the goal it was standing on: an empty walk, no home
// leg, 2026-09-27 10:55.)
static RouteResult walk_weighted(const Maze &maze, int *x, int *y, Heading *h,
                                 uint8_t *rx, uint8_t *ry, Heading *rh, int *n) {
    constexpr int NS = MAZE_SIZE * MAZE_SIZE * 4;
    static uint16_t cost[NS];
    static int16_t pred[NS];
    static uint8_t done[NS];
    const uint16_t INF = 0xFFFF;
    const int step_c = 10;
    const int turn_c = (int)(10.0f * ctx_turn_cost + 0.5f);
    auto idx = [](int cx, int cy, Heading ch) { return (cy * MAZE_SIZE + cx) * 4 + (int)ch; };
    auto is_target = [&](int cx, int cy) { return maze.distance(cx, cy) == 0; };
    for (int i = 0; i < NS; i++) { cost[i] = INF; pred[i] = -1; done[i] = 0; }
    int s0 = idx(*x, *y, *h);
    cost[s0] = 0;
    int best_t = -1;
    for (;;) {
        // The cheapest open state (1024 states: a linear scan is fine, once a leg).
        int u = -1; uint16_t uc = INF;
        for (int i = 0; i < NS; i++) if (!done[i] && cost[i] < uc) { uc = cost[i]; u = i; }
        if (u < 0) break;
        done[u] = 1;
        int ux = (u / 4) % MAZE_SIZE, uy = (u / 4) / MAZE_SIZE;
        Heading uh = (Heading)(u & 3);
        if (is_target(ux, uy)) { best_t = u; break; }   // first target popped is the cheapest
        const Heading cand[4] = {uh, heading_left(uh), heading_right(uh), heading_back(uh)};
        const int tc[4] = {0, turn_c, turn_c, 2 * turn_c};
        for (int k = 0; k < 4; k++) {
            // KNOWN-open only, like best_step_known.
            if (!maze.wall_known(ux, uy, cand[k]) || maze.wall_present(ux, uy, cand[k])) continue;
            int nx = ux + heading_dx(cand[k]), ny = uy + heading_dy(cand[k]);
            if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) continue;
            int v = idx(nx, ny, cand[k]);
            uint32_t c = (uint32_t)uc + step_c + tc[k];
            if (c < cost[v]) { cost[v] = (uint16_t)c; pred[v] = (int16_t)u; }
        }
    }
    if (best_t < 0) return ROUTE_NO_PATH;
    // Reconstruct: the chain of states from the start to the target; each
    // state after the first is "arrived at (x,y) heading h", so the cell
    // stepped THROUGH is the predecessor's cell and the heading taken is h.
    static int16_t chain[NS];
    int len = 0;
    for (int u = best_t; u >= 0; u = pred[u]) chain[len++] = (int16_t)u;
    if (len < 1) return ROUTE_NO_PATH;
    for (int i = len - 1; i > 0; i--) {
        if (*n >= ROUTE_MAX_CELLS) return ROUTE_TOO_LONG;
        int u = chain[i], v = chain[i - 1];
        rx[*n] = (uint8_t)((u / 4) % MAZE_SIZE);
        ry[*n] = (uint8_t)((u / 4) / MAZE_SIZE);
        rh[*n] = (Heading)(v & 3);
        (*n)++;
    }
    *x = (best_t / 4) % MAZE_SIZE;
    *y = (best_t / 4) / MAZE_SIZE;
    *h = (Heading)(best_t & 3);
    return ROUTE_OK;
}

// Walk the CURRENT flood downhill from (x, y, h), appending the cells stepped
// through and the heading taken leaving each. Returns ROUTE_OK and updates the
// pose to where the walk ended.
//
// best_step_known, not best_step: a route compiled over OPTIMISTIC distances
// would happily aim the mouse through an edge nobody has ever looked at. The
// whole premise of a fast run is that the map is finished.
// With rturn > 0 the turn-weighted walk above replaces the downhill walk;
// the flood the caller ran still gates reachability.
// mode: -1 = the default (weighted when rturn > 0, else plain), 0 = plain
// (fewest cells), 1 = weighted (cells + rturn x turns).
static RouteResult walk_flood(Maze &maze, int *x, int *y, Heading *h,
                              uint8_t *rx, uint8_t *ry, Heading *rh, int *n,
                              int mode = -1) {
    if (maze.distance(*x, *y) == MAZE_DIST_UNREACHABLE) return ROUTE_NO_PATH;
    bool weighted = mode < 0 ? ctx_turn_cost > 0.0f : mode == 1;
    if (weighted) return walk_weighted(maze, x, y, h, rx, ry, rh, n);
    while (maze.distance(*x, *y) != 0) {
        if (*n >= ROUTE_MAX_CELLS) return ROUTE_TOO_LONG;
        Heading next;
        if (!maze.best_step_known(*x, *y, *h, next)) return ROUTE_NO_PATH;
        rx[*n] = (uint8_t)*x;
        ry[*n] = (uint8_t)*y;
        rh[*n] = next;
        (*n)++;
        *x += heading_dx(next);
        *y += heading_dy(next);
        *h = next;
    }
    return ROUTE_OK;
}

// Compile a walk that has already been made. Split out from route_build_pass
// so an out-and-back can hand it TWO walks joined end to end -- see
// route_build_loop. Everything downstream (turn finding, the U-turn merge,
// the radius fit, the emit) is then identical, which is the point: the goal
// stops being a special case and becomes whatever heading change happens to
// be there.
static RouteResult compile_walk(const Maze &maze, const uint8_t *rx,
                                const uint8_t *ry,
                                const Heading *rh, int n, Heading h0,
                                SpeedPath &path, float v_mm_s, float slew_dps2,
                                RouteStats *stats, RouteTurn *turns_out,
                                int max_turns_out, float lead_in_mm,
                                bool allow_uturn, bool allow_diag,
                                const RouteVeto *veto = nullptr, int nveto = 0,
                                RouteRefusal *refusal = nullptr) {
    if (stats) *stats = RouteStats{};
    if (n == 0) return ROUTE_NO_PATH;   // already at the target: nothing to fly
    auto vetoed = [&](int cell_ix, uint8_t kind) {
        for (int k = 0; k < nveto; k++)
            if (veto[k].cell_ix == cell_ix && veto[k].kind == kind) return true;
        return false;
    };

    // ---- 2. Find the heading changes -------------------------------------
    //
    // A turn happens at cell i when the heading LEAVING i differs from the
    // heading that ARRIVED at it. The first step's arrival heading is the
    // pose the mouse starts in, so a mouse placed facing the wrong way turns
    // in its own start cell -- and that turn has no room before it, which the
    // radius pass below will catch rather than silently clip.
    RouteTurn turn[ROUTE_MAX_TURNS];
    int nt = 0;
    Heading arrive = h0;
    for (int i = 0; i < n; i++) {
        if (rh[i] != arrive) {
            if (rh[i] == heading_back(arrive)) return ROUTE_BAD_TURN;
            if (nt >= ROUTE_MAX_TURNS) return ROUTE_TOO_LONG;
            turn[nt].cx = rx[i];
            turn[nt].cy = ry[i];
            turn[nt].entry_h = (uint8_t)arrive;   // maze frame, pre-turn
            // Front-gateable? A wall dead ahead of the turn cell on the
            // approach heading (face at datum + 84). Survives the U-merge
            // with the pair's first record, whose straight-ahead wall is
            // the one the front beams see on the approach.
            turn[nt].front_ok =
                maze.wall_present(rx[i], ry[i], arrive) ? 1 : 0;
            turn[nt].cell_ix = (uint8_t)i;
            // +CCW. N->W, W->S, S->E, E->N are lefts; the rest are rights.
            turn[nt].sweep_deg = (rh[i] == heading_left(arrive)) ? 90 : -90;
            turn[nt].r_mm = ROUTE_RADII[0];
            turn[nt].cp_off_mm = 0;
            turn[nt].diag_mm = 0;
            turn[nt].v_cap_mm_s = 0;
            nt++;
        }
        arrive = rh[i];
    }
    for (int j = 0; j < nt; j++) turn[j].exit_ix = turn[j].cell_ix;

    // ---- 2a. CANDIDATES: THE V AND THE 135 ----------------------------------
    //
    // A same-sign adjacent pair is one of four things, decided by what is
    // on each side of it (Route.h, THE 135):
    //     staircase | pair | staircase   a V       (diagonal into diagonal)
    //     staircase | pair | straight    a 135-out (diagonal into corridor)
    //     straight  | pair | staircase   a 135-in  (corridor into diagonal)
    //     straight  | pair | straight    a U-turn
    // The first three are left alone by the U-merge; 2c decides whether the
    // staircase(s) actually merge, and 2d makes a U of whatever did not.
    auto adj_alt = [&](int a, int b) {
        return a >= 0 && b < nt && turn[b].cell_ix == turn[a].cell_ix + 1 &&
               turn[b].sweep_deg == -turn[a].sweep_deg;
    };
    auto adj_same = [&](int a, int b) {
        return a >= 0 && b < nt && turn[b].cell_ix == turn[a].cell_ix + 1 &&
               turn[b].sweep_deg == turn[a].sweep_deg;
    };
    auto adj_any = [&](int a, int b) { return adj_alt(a, b) || adj_same(a, b); };
    auto v_candidate = [&](int j) {
        return allow_diag && ROUTE_DIAG_ENABLED && adj_same(j, j + 1) &&
               adj_alt(j - 1, j) && adj_alt(j + 1, j + 2);
    };
    auto out135_candidate = [&](int j) {
        return allow_diag && ROUTE_DIAG_ENABLED && route_135_enabled() &&
               adj_same(j, j + 1) && adj_alt(j - 1, j) && !adj_any(j + 1, j + 2);
    };
    auto in135_candidate = [&](int j) {
        return allow_diag && ROUTE_DIAG_ENABLED && route_135_enabled() &&
               adj_same(j, j + 1) && adj_alt(j + 1, j + 2) && !adj_any(j - 1, j);
    };

    // ---- 2b. MERGE ADJACENT SAME-DIRECTION PAIRS INTO U-TURNS -------------
    //
    // Left-then-left (or right-then-right) one cell apart is a 180. Merging
    // is done BEFORE the radius fit because a U-turn's radius is forced by
    // the maze and cannot be traded -- so the fit has to see it as a fixed
    // obstacle, not as something it may demote.
    //
    // Scanned left to right and non-overlapping: three same-direction turns
    // in a row (a spiral) merge the FIRST pair and leave the third as a 90,
    // which is correct -- a spiral is not two U-turns. A V or 135 candidate
    // is skipped here; if its staircase fails to merge in 2c, 2d makes the
    // U of it that it also is.
    SpeedPath::UTurn uturn{};
    // A forced radius (ROUTE_UTURN_RADIUS_MM) solves the CLOTHOID instead of
    // the radius; zero keeps the original solve, which finds the radius that
    // the corner rail happens to produce.
    bool have_uturn_geom =
        allow_uturn && route_uturn_geometry(v_mm_s, slew_dps2, &uturn);
    float u_set = have_uturn_geom ? uturn_setback(uturn) : 0.0f;
    if (have_uturn_geom && !uturn_clears(uturn, u_set)) have_uturn_geom = false;
    auto merge_u_at = [&](int j) {
        turn[j].sweep_deg = (int16_t)(turn[j].sweep_deg > 0 ? 180 : -180);
        turn[j].exit_ix = turn[j + 1].cell_ix;
        turn[j].r_mm = (uint8_t)(uturn.radius_mm + 0.5f);
        for (int m = j + 1; m + 1 < nt; m++) turn[m] = turn[m + 1];
        nt--;
    };
    if (have_uturn_geom && u_set > 0.0f) {
        for (int j = 0; j + 1 < nt; j++) {
            if (!adj_same(j, j + 1)) continue;
            if (v_candidate(j) || out135_candidate(j) || in135_candidate(j)) continue;
            if (vetoed(turn[j].cell_ix, ROUTE_VETO_UTURN)) continue;   // local fallback: stays two 90s
            merge_u_at(j);
        }
    }

    // ---- 2c. MERGE STAIRCASES INTO DIAGONALS, AND Vs / 135s AT THEIR ENDS --
    //
    // See the diagonal note in Route.h. A maximal run of alternating-sign
    // 90s, each exactly one cell after the last, becomes a 45 at its first
    // cell and a 45 at its last with one straight between. Runs shorter than
    // ROUTE_DIAG_MIN_STEPS stay as they are -- unless a V or a 135 bounds
    // them, when one step is enough. Two runs joined by a V share a corner
    // point: the first's exit and the second's entry collapse into ONE 90
    // there (Route.h, THE V). A run that hands into a lone same-sign 90 and
    // then a straight ends in a 135 instead of a 45: the corner point moves
    // half a cell past the lone 90's centre, against its departure, and
    // the 90 is absorbed (Route.h, THE 135); the mirror, a lone 90 handing
    // into a run, begins it with a 135. Done AFTER the U-merge (a U-turn is
    // never part of a staircase) and BEFORE the fit, so the fit sees each
    // pair's real room: the diagonal length, not a cell count.
    int n_diag = 0, n_135 = 0;
    if (allow_diag && ROUTE_DIAG_ENABLED) {
        auto is_u = [&](int j) { return turn[j].sweep_deg == 180 ||
                                        turn[j].sweep_deg == -180; };
        auto ux = [](Heading h) { return h == HEADING_E ? 1.0f : h == HEADING_W ? -1.0f : 0.0f; };
        auto uy = [](Heading h) { return h == HEADING_N ? 1.0f : h == HEADING_S ? -1.0f : 0.0f; };
        // Runs: maximal alternating chains.
        int rs[ROUTE_MAX_TURNS], re[ROUTE_MAX_TURNS];
        bool v_after[ROUTE_MAX_TURNS], o135[ROUTE_MAX_TURNS], i135[ROUTE_MAX_TURNS];
        bool exact[ROUTE_MAX_TURNS], ok[ROUTE_MAX_TURNS], banned[ROUTE_MAX_TURNS];
        bool absorbed[ROUTE_MAX_TURNS];
        float dlen[ROUTE_MAX_TURNS];
        int nr = 0;
        for (int j = 0; j < nt;) {
            int e = j;
            while (e + 1 < nt && !is_u(e) && !is_u(e + 1) && adj_alt(e, e + 1)) e++;
            rs[nr] = j; re[nr] = e;
            nr++;
            j = e + 1;
        }
        auto chain = [&](int r) { return re[r] > rs[r]; };          // >= 1 step
        auto lone = [&](int r) { return re[r] == rs[r] && !is_u(rs[r]); };
        for (int r = 0; r < nr; r++) {
            int j = rs[r], e = re[r];
            // What joins run r to run r+1: a V (two chains), or a 135-out
            // (a chain into a lone 90 with nothing adjacent after it); and
            // what precedes it: a 135-in (a lone 90 with nothing adjacent
            // before it, into a chain).
            bool same_next = r + 1 < nr && !is_u(e) && !is_u(rs[r + 1]) && adj_same(e, rs[r + 1]);
            v_after[r] = same_next && chain(r) && chain(r + 1);
            o135[r] = route_135_enabled() && same_next && chain(r) && lone(r + 1) &&
                      !adj_any(re[r + 1], re[r + 1] + 1);
            bool same_prev = r > 0 && !is_u(j) && !is_u(re[r - 1]) && adj_same(re[r - 1], j);
            i135[r] = route_135_enabled() && same_prev && chain(r) && lone(r - 1) &&
                      !adj_any(rs[r - 1] - 1, rs[r - 1]);
            // Corner points, in cell units. Entry: half a cell back along
            // the arrival heading from the first turn cell's centre. Exit:
            // half a cell on along the DEPARTURE heading from the last. For
            // a run entered through a V the entry point is the V's own
            // corner point -- the same aperture -- so the formula holds.
            // Through a 135 the point is where the diagonal's centre line
            // meets the corridor's: half a cell PAST the lone 90's centre
            // along its arrival (135-in), or half a cell BEFORE it against
            // its departure (135-out) -- one post pitch further along the
            // diagonal than the 45's point would be.
            float pix, piy, pox, poy;
            if (i135[r]) {
                int l = rs[r - 1];
                Heading h_in = (Heading)turn[l].entry_h;
                pix = (float)turn[l].cx + 0.5f + 0.5f * ux(h_in);
                piy = (float)turn[l].cy + 0.5f + 0.5f * uy(h_in);
            } else {
                Heading h_in = (Heading)turn[j].entry_h;
                pix = (float)turn[j].cx + 0.5f - 0.5f * ux(h_in);
                piy = (float)turn[j].cy + 0.5f - 0.5f * uy(h_in);
            }
            if (o135[r]) {
                int l = rs[r + 1];
                Heading h_out = rh[turn[l].cell_ix];
                pox = (float)turn[l].cx + 0.5f - 0.5f * ux(h_out);
                poy = (float)turn[l].cy + 0.5f - 0.5f * uy(h_out);
            } else {
                Heading h_out = rh[turn[e].cell_ix];
                pox = (float)turn[e].cx + 0.5f + 0.5f * ux(h_out);
                poy = (float)turn[e].cy + 0.5f + 0.5f * uy(h_out);
            }
            float dx = pox - pix, dy = poy - piy;
            // Exactly diagonal, or it is not the shape this primitive flies.
            // (Cell units are multiples of 0.5, so this compare is exact.)
            exact[r] = e > j && fabsf(fabsf(dx) - fabsf(dy)) <= 1e-3f &&
                       fabsf(dx) >= 0.5f;
            dlen[r] = sqrtf(dx * dx + dy * dy) * ROUTE_CELL_MM;
        }
        // Which runs merge. A V needs BOTH its runs to merge; a one-step run
        // merges only on the strength of a V or a 135. Iterate to a fixed
        // point -- withdrawing a V can un-merge a one-step run, which can
        // withdraw another V. A vetoed run (any member cell, the absorbed
        // 135 cell included) never merges.
        for (int r = 0; r < nr; r++) {
            banned[r] = false;
            for (int m = rs[r]; m <= re[r]; m++)
                if (vetoed(turn[m].cell_ix, ROUTE_VETO_DIAG)) banned[r] = true;
            if (o135[r] && vetoed(turn[rs[r + 1]].cell_ix, ROUTE_VETO_DIAG)) banned[r] = true;
            if (i135[r] && vetoed(turn[rs[r - 1]].cell_ix, ROUTE_VETO_DIAG)) banned[r] = true;
            ok[r] = !banned[r] && exact[r] && (re[r] - rs[r]) >= ROUTE_DIAG_MIN_STEPS;
            absorbed[r] = false;
        }
        for (int pass = 0; pass < nr + 1; pass++) {
            bool moved = false;
            for (int r = 0; r < nr; r++) {
                bool vb = r > 0 && v_after[r - 1];
                bool va = v_after[r];
                bool want = !banned[r] && exact[r] &&
                            ((re[r] - rs[r]) >= ROUTE_DIAG_MIN_STEPS ||
                             ((vb || va || o135[r] || i135[r]) &&
                              (re[r] - rs[r]) >= ROUTE_DIAG_MIN_STEPS_AT_V));
                if (want != ok[r]) { ok[r] = want; moved = true; }
            }
            for (int r = 0; r + 1 < nr; r++) {
                if (v_after[r] && !(ok[r] && ok[r + 1])) { v_after[r] = false; moved = true; }
            }
            for (int r = 0; r < nr; r++) {
                if (o135[r] && !ok[r]) { o135[r] = false; moved = true; }
                if (i135[r] && !ok[r]) { i135[r] = false; moved = true; }
            }
            if (!moved) break;
        }
        // A run that lost its 135 keeps a 45 at that end but its corner
        // point was computed for the 135: recompute the plain geometry.
        for (int r = 0; r < nr; r++) {
            if (!ok[r]) continue;
            int j = rs[r], e = re[r];
            Heading h_in = (Heading)turn[j].entry_h, h_out = rh[turn[e].cell_ix];
            float pix = (float)turn[j].cx + 0.5f - 0.5f * ux(h_in);
            float piy = (float)turn[j].cy + 0.5f - 0.5f * uy(h_in);
            float pox = (float)turn[e].cx + 0.5f + 0.5f * ux(h_out);
            float poy = (float)turn[e].cy + 0.5f + 0.5f * uy(h_out);
            if (i135[r]) { int l = rs[r - 1]; Heading hl = (Heading)turn[l].entry_h;
                           pix = (float)turn[l].cx + 0.5f + 0.5f * ux(hl); piy = (float)turn[l].cy + 0.5f + 0.5f * uy(hl); }
            if (o135[r]) { int l = rs[r + 1]; Heading hl = rh[turn[l].cell_ix];
                           pox = (float)turn[l].cx + 0.5f - 0.5f * ux(hl); poy = (float)turn[l].cy + 0.5f - 0.5f * uy(hl); }
            float dx = pox - pix, dy = poy - piy;
            exact[r] = fabsf(fabsf(dx) - fabsf(dy)) <= 1e-3f && fabsf(dx) >= 0.5f;
            dlen[r] = sqrtf(dx * dx + dy * dy) * ROUTE_CELL_MM;
            if (!exact[r]) ok[r] = false;   // (cannot happen: the plain points are always exact for a chain)
            if (o135[r]) absorbed[r + 1] = true;
            if (i135[r]) absorbed[r - 1] = true;
        }
        // Emit into a fresh list: (45-in | 135-in), (V | 45-out | 135-out)
        // per merged run, the interior steps dropped; unmerged runs copied
        // through untouched; a lone 90 absorbed by a 135 not at all.
        RouteTurn out[ROUTE_MAX_TURNS];
        int no = 0;
        bool entered_by_v = false;
        for (int r = 0; r < nr; r++) {
            int j = rs[r], e = re[r];
            if (absorbed[r]) continue;
            if (!ok[r]) {
                for (int m = j; m <= e; m++) out[no++] = turn[m];
                entered_by_v = false;
                continue;
            }
            if (!entered_by_v) {
                if (i135[r]) {
                    // The lone 90's record carries the 135: its cell, its
                    // arrival heading, its front wall (it arrives along the
                    // corridor, so a face ahead is real), and the corner
                    // point half a cell PAST its centre.
                    RouteTurn t = turn[rs[r - 1]];
                    t.sweep_deg = (int16_t)(t.sweep_deg > 0 ? 135 : -135);
                    t.cp_off_mm = (int8_t)(+ROUTE_DIAG_CP_OFF_MM);
                    t.diag_mm = 0;
                    out[no++] = t;
                    n_135++;
                } else {
                    RouteTurn t = turn[j];
                    t.sweep_deg = (int16_t)(t.sweep_deg > 0 ? 45 : -45);
                    t.cp_off_mm = (int8_t)(-ROUTE_DIAG_CP_OFF_MM);
                    t.diag_mm = 0;
                    out[no++] = t;
                }
            }
            bool v = v_after[r];
            RouteTurn x = o135[r] ? turn[rs[r + 1]] : turn[e];
            // A V keeps the 90 sweep (its sign; the pair's second turn has
            // the same) and is told apart by its corner-point offset. A
            // 135-out is the lone 90's record with the corner point half a
            // cell BEFORE its centre, against its departure.
            x.sweep_deg = (int16_t)(v ? (x.sweep_deg > 0 ? 90 : -90)
                                      : o135[r] ? (x.sweep_deg > 0 ? 135 : -135)
                                                : (x.sweep_deg > 0 ? 45 : -45));
            x.cp_off_mm = (int8_t)(o135[r] ? -ROUTE_DIAG_CP_OFF_MM : +ROUTE_DIAG_CP_OFF_MM);
            x.diag_mm = (uint16_t)(dlen[r] + 0.5f);
            x.front_ok = 0;   // arrives on the diagonal: no face ahead
            out[no++] = x;
            n_diag++;
            if (o135[r]) n_135++;
            if (v) {
                // The next run's first turn is the V's second half: absorbed.
                rs[r + 1]++;
            }
            entered_by_v = v;
        }
        for (int m = 0; m < no; m++) turn[m] = out[m];
        nt = no;
    }
    (void)n_diag;

    // ---- 2d. THE U-TURNS 2b LEFT FOR 2c ------------------------------------
    //
    // A V or 135 candidate whose staircase did not merge is still a
    // same-sign pair one cell apart: the U it also is. Same merge, same
    // veto, after the fact.
    if (have_uturn_geom && u_set > 0.0f) {
        for (int j = 0; j + 1 < nt; j++) {
            if (!adj_same(j, j + 1)) continue;
            if (turn[j].cp_off_mm != 0 || turn[j + 1].cp_off_mm != 0) continue;   // plain 90s only
            if (turn[j].sweep_deg != 90 && turn[j].sweep_deg != -90) continue;
            if (vetoed(turn[j].cell_ix, ROUTE_VETO_UTURN)) continue;
            merge_u_at(j);
        }
    }

    // ---- 3. Fit the radii --------------------------------------------------
    //
    // Every turn starts at the preferred radius and is demoted only where the
    // geometry says it must be. The constraint is pairwise: between turns j-1
    // and j there are gap_j millimetres of cell centres, and the two tangents
    // plus the minimum straight have to live inside it. The run before the
    // first turn and after the last are the same constraint with one tangent.
    //
    // Demote the turn with the LARGER tangent, because that is the one buying
    // the most room per step down the ladder, and re-check -- a single drop
    // can fix a pair, and a pair fixed here can be broken again by a
    // neighbour's drop, so it iterates until nothing moves. Bounded: every
    // pass either demotes something or stops, and there are only
    // nt * ROUTE_N_RADII demotions available.
    int rix[ROUTE_MAX_TURNS];
    int six[ROUTE_MAX_TURNS];   // speed rung (Route.h): 0 = the normal corner speed
    for (int j = 0; j < nt; j++) { rix[j] = 0; six[j] = 0; }

    auto is_uturn = [&](int j) { return turn[j].sweep_deg == 180 ||
                                        turn[j].sweep_deg == -180; };
    auto is_diag = [&](int j) { return turn[j].sweep_deg == 45 ||
                                       turn[j].sweep_deg == -45; };
    auto is_135 = [&](int j) { return turn[j].sweep_deg == 135 ||
                                      turn[j].sweep_deg == -135; };
    auto is_v = [&](int j) { return (turn[j].sweep_deg == 90 || turn[j].sweep_deg == -90) &&
                                    turn[j].cp_off_mm != 0; };
    auto is_diag_exit = [&](int j) { return turn[j].diag_mm > 0; };
    // Walk coordinate of a turn's corner point relative to its cell centre,
    // and of the point its primitive hands the walk back at.
    auto cp_in = [&](int j) {
        return (float)turn[j].cell_ix * ROUTE_CELL_MM + (float)turn[j].cp_off_mm;
    };
    auto cp_out = [&](int j) {
        return (float)turn[j].exit_ix * ROUTE_CELL_MM + (float)turn[j].cp_off_mm;
    };
    // For a U-turn this is the SETBACK, not a tangent -- and it is fixed,
    // because the radius is forced by the corridor. The fit may not demote
    // it, so a pair that will not fit is resolved by the caller re-running
    // without U-turns rather than by shrinking this.
    // The radius a turn takes at a given rung: rung 0 honours the r90/l90
    // per-direction override (route_set_r90), everything below walks the
    // compiled ladder unchanged.
    auto rung_radius = [&](int j, int rung) -> float {
        // r90/l90 are the NINETY-degree overrides; a 45 walks the ladder.
        if (rung == 0 && !is_diag(j) && turn[j].cp_off_mm == 0) {
            // (a V is a 90 with a corner-point offset: not a corridor 90,
            // so the corridor override does not apply)
            float o = turn[j].sweep_deg > 0 ? ctx_r90_left : ctx_r90_right;
            if (o > 0.0f) return o;
        }
        float r = (float)ROUTE_RADII[rung];
        if (is_v(j) && r > ROUTE_V_RADIUS_MAX_MM)
            r = ROUTE_V_RADIUS_MAX_MM + ctx_v_out;   // a V: inside clearance (Route.h)
        return r;
    };
    // The speed this turn is fitted (and, once slowed, emitted) at: the
    // corner speed for its radius, times the speed rung's factor, never
    // below the floor. Rung 0 is exactly what the fit did before the rung
    // existed.
    auto speed_of = [&](int j) {
        float r = rung_radius(j, rix[j]);
        float v = corner_speed(v_mm_s, r);
        for (int k = 0; k < six[j]; k++) v *= ROUTE_V_RUNG;
        if (six[j] > 0 && v < ROUTE_V_FLOOR_MM_S) v = ROUTE_V_FLOOR_MM_S;
        return v;
    };
    auto tangent_of = [&](int j) {
        // Geometry at the speed the corner is actually flown (see
        // route_set_corner_context) -- at high rv this is what keeps R84
        // from demoting itself over tangent room it will never need.
        float r = rung_radius(j, rix[j]);
        float t = is_uturn(j)
                      ? u_set
                      : route_tangent_mm(r, speed_of(j), slew_dps2,
                                         is_diag(j) ? 45.0f : is_135(j) ? 135.0f : 90.0f);
        // A V flown vout outside its lines: the offset lines' corner point
        // is vout further along each leg than the centreline one every
        // straight is measured from, so report the tangent from the
        // centreline corner point -- the arc still starts and ends at the
        // post-abeam stations (Route.h, ROUTE_V_RADIUS_MAX_MM).
        if (is_v(j)) t -= ctx_v_out;
        return t;
    };
    // The straight this pair must leave. Two turns the SAME way are a 180
    // chopped in half -- the curvature never changes sign, so a straight
    // between them only forces the rate to unwind to zero and back up. See
    // ROUTE_MIN_STRAIGHT_SAME_MM for the eight passes that measured it.
    auto min_straight_for = [&](int j) {
        return (turn[j].sweep_deg == turn[j - 1].sweep_deg)
                   ? ROUTE_MIN_STRAIGHT_SAME_MM
                   : ROUTE_MIN_STRAIGHT_MM;
    };
    // Report what could not be placed, for the veto loop.
    auto refuse = [&](int a, int b) {
        if (refusal) {
            refusal->cell_a = turn[a].cell_ix;
            refusal->cell_b = turn[b].cell_ix;
            refusal->uturn_a = is_uturn(a) ? 1 : 0;
            refusal->uturn_b = is_uturn(b) ? 1 : 0;
            refusal->diag_a = (is_diag(a) || is_v(a) || is_135(a)) ? 1 : 0;
            refusal->diag_b = (is_diag(b) || is_v(b) || is_135(b)) ? 1 : 0;
        }
        return ROUTE_NO_RADIUS;
    };
    // One step down for a turn. The radius ladder is walked for BOTH turns
    // of a pair before either is slowed: a rung of radius costs less time
    // than a rung of speed. A U-turn has nothing to give on either.
    auto demote_radius = [&](int j) {
        if (j < 0 || is_uturn(j) || rix[j] + 1 >= ROUTE_N_RADII) return false;
        rix[j]++;
        return true;
    };
    auto demote_speed = [&](int j) {
        if (j < 0 || is_uturn(j) || six[j] >= ROUTE_N_V_RUNGS) return false;
        six[j]++;
        return true;
    };
    auto demote = [&](int victim, int other) {
        return demote_radius(victim) || demote_radius(other) ||
               demote_speed(victim) || demote_speed(other);
    };

    for (int guard = 0; guard <= nt * (ROUTE_N_RADII + ROUTE_N_V_RUNGS) + 1; guard++) {
        bool changed = false;
        for (int j = 0; j < nt && !changed; j++) {
            float room, need;
            if (j == 0) {
                // Start of the route to the first turn: half a cell of
                // lead-in is all a mouse standing on a cell centre has.
                room = cp_in(0) + lead_in_mm;
                need = tangent_of(0);
            } else if (is_diag_exit(j)) {
                // The diagonal itself: corner point to corner point.
                room = (float)turn[j].diag_mm;
                need = tangent_of(j - 1) + tangent_of(j) + min_straight_for(j);
            } else {
                // From the PREVIOUS turn's exit, which is its own cell for a
                // 90 and the far cell for a U-turn -- corner point to corner
                // point, so a diagonal's shifted points count.
                room = cp_in(j) - cp_out(j - 1);
                need = tangent_of(j - 1) + tangent_of(j) + min_straight_for(j);
            }
            if (need <= room) continue;
            // Pick the victim: the larger tangent of the pair, or the only
            // one there is. Try the other before giving up -- the larger
            // tangent may already have nothing left.
            int victim = j;
            if (j > 0 && tangent_of(j - 1) > tangent_of(j)) victim = j - 1;
            int other = (j == 0) ? -1 : (victim == j ? j - 1 : j);
            if (!demote(victim, other)) return refuse(j > 0 ? j - 1 : 0, j);
            changed = true;
        }
        if (!changed) break;
    }
    // The tail: after the last turn there must still be room for its tangent.
    if (nt > 0) {
        float room = (float)n * ROUTE_CELL_MM - cp_out(nt - 1);
        while (tangent_of(nt - 1) > room) {
            if (!demote(nt - 1, -1)) return refuse(nt - 1, nt - 1);
        }
    }
    for (int j = 0; j < nt; j++) {
        if (!is_uturn(j)) {
            turn[j].r_mm = (uint8_t)(rung_radius(j, rix[j]) + 0.5f);
            turn[j].v_cap_mm_s = six[j] > 0 ? (uint16_t)(speed_of(j) + 0.5f) : 0;
        }
    }

    // ---- 4. Emit ----------------------------------------------------------
    //
    // Cell centres, not wall faces: the path starts at the centre of
    // (x0, y0) and ends at the centre of the target cell. A caller that
    // launches from the standard rear-wall setback adds its own lead-in
    // straight before this; keeping the compiler in cell-centre coordinates
    // is what makes it testable without a start convention baked in.
    path.clear(0.0f, 0.0f, 0.0f);
    float min_straight = 1e9f;
    float travelled = 0.0f;   // cell centres consumed, mm
    // Bookkeeping runs in CENTRE-LINE coordinates -- distance along the walk
    // through cell centres -- not in path arclength, because that is the frame
    // the turns are placed in. The two differ (a corner is shorter than the
    // two tangents it replaces) and stats.length_mm reports the real one.
    // The lead-in shifts the origin: with it, centre-line 0 is the mouse's
    // start pose and the first cell centre sits at lead_in_mm.
    float emitted = -lead_in_mm;
    float diag_t_in = 0.0f;   // the entry 45's tangent, for its exit's straight

    for (int j = 0; j < nt; j++) {
        float centre = (float)turn[j].cell_ix * ROUTE_CELL_MM;
        if (is_uturn(j)) {
            // ONE primitive, entering u_set before the first cell centre and
            // leaving u_set after the second. The walk distance it consumes
            // (one cell plus two setbacks) is not its arclength -- which is
            // already true of every corner here, see the note above.
            float straight = centre - u_set - emitted;
            if (straight < 0.0f) straight = 0.0f;
            if (j > 0 && straight < min_straight) min_straight = straight;
            if (straight > 0.0f && !path.add_straight(straight))
                return ROUTE_PATH_FULL;
            // rvu: the U flown under a cap. Its clothoid was solved at its
            // corner speed; a slower mouse follows that rail with room to
            // spare, so the geometry stands and only the speed changes.
            path.set_next_vcap(ctx_cap_u);
            if (ctx_cap_u > 0.0f && (turn[j].v_cap_mm_s == 0 || (float)turn[j].v_cap_mm_s > ctx_cap_u))
                turn[j].v_cap_mm_s = (uint16_t)(ctx_cap_u + 0.5f);
            if (!path.add_uturn(uturn, turn[j].sweep_deg > 0 ? 1.0f : -1.0f,
                                u_set))
                return ROUTE_PATH_FULL;
            emitted = (float)turn[j].exit_ix * ROUTE_CELL_MM + u_set;
            continue;
        }
        float R = (float)turn[j].r_mm;
        // A slowed corner is built at the speed the fit placed it at, so the
        // tangent the walk reserves and the arc the path builds agree; so is
        // a 135, which has no flown history to keep. Every other corner
        // keeps the rail clothoid it always had -- sized at rv, not at its
        // corner speed, so its built tangent is LONGER than the fit's (R84
        // 90: 133 mm against 101; a 45: 62 against 51) and the straights
        // between come out shorter than the fit reserved, clamped at zero.
        // That is the 9-40 mm by which every loop compiled at rv 1500 fails
        // to close today (COMP_GAPS.md, 2026-09-11) and it is left alone
        // here on purpose: reconciling it moves every corner the learners
        // were taught on, which is its own session. A 135 built at rv
        // would overshoot by ~40 mm a side, so it does not join the queue.
        float vc = turn[j].v_cap_mm_s > 0 ? (float)turn[j].v_cap_mm_s
                 : is_135(j) ? speed_of(j) : v_mm_s;
        float cap = turn[j].v_cap_mm_s > 0 ? vc : 0.0f;
        // rvv (Route.h): a V, and the turn it hands into within
        // ROUTE_V_CAP_AFTER_MM of straight, are capped. The straight is
        // measured as the fit measured it: tangent to tangent.
        if (ctx_cap_v > 0.0f) {
            bool after_v = j > 0 && is_v(j - 1) && !is_uturn(j - 1) &&
                           (cp_in(j) - tangent_of(j)) - (cp_out(j - 1) + tangent_of(j - 1)) <
                               ROUTE_V_CAP_AFTER_MM;
            if (is_v(j) || after_v) {
                if (cap <= 0.0f || ctx_cap_v < cap) cap = ctx_cap_v;
                if (vc > cap) vc = cap;
                turn[j].v_cap_mm_s = (uint16_t)(cap + 0.5f);
            }
        }
        float L = SpeedPath::clothoid_for_rail(R, vc, slew_dps2);
        float sweep = (float)turn[j].sweep_deg;
        float T = SpeedPath::tangent_for(R, L, fabsf(sweep));
        path.set_next_vcap(cap);
        // A V at vout: the arc is R = 64 + vout, but the walk measures its
        // tangents from the CENTRELINE corner point, vout short of the
        // offset lines' own (see tangent_of above, and Route.h).
        if (is_v(j)) T -= ctx_v_out;
        if (is_diag_exit(j)) {
            // The diagonal straight: from the entry 45's exit tangent to the
            // exit 45's entry tangent, corner point to corner point. The
            // walk resumes at the exit corner point, half a cell past the
            // exit cell's centre.
            float straight = (float)turn[j].diag_mm - diag_t_in - T;
            if (straight < 0.0f) straight = 0.0f;   // the fit guarantees this
            if (straight < min_straight) min_straight = straight;
            if (straight > 0.0f && !path.add_straight(straight))
                return ROUTE_PATH_FULL;
            path.set_clothoid_len(L);
            if (!path.add_arc(R, sweep)) return ROUTE_PATH_FULL;
            emitted = centre + (float)turn[j].cp_off_mm + T;
            // A V is also the NEXT diagonal's entry: its tangent is what
            // that diagonal's exit subtracts.
            diag_t_in = T;
            continue;
        }
        float cp = centre + (float)turn[j].cp_off_mm;   // corner point
        float straight = cp - T - emitted;
        if (straight < 0.0f) straight = 0.0f;   // the fit pass guarantees this
        if (j > 0 && straight < min_straight) min_straight = straight;
        if (straight > 0.0f && !path.add_straight(straight))
            return ROUTE_PATH_FULL;
        path.set_clothoid_len(L);
        if (!path.add_arc(R, sweep)) return ROUTE_PATH_FULL;
        emitted = cp + T;
        diag_t_in = T;   // only read by a diagonal exit, whose entry is j
        travelled = centre;
        (void)travelled;
    }
    float end = (float)n * ROUTE_CELL_MM;   // centre-line, lead-in excluded
    if (end > emitted && !path.add_straight(end - emitted))
        return ROUTE_PATH_FULL;

    if (turns_out) {
        for (int j = 0; j < nt && j < max_turns_out; j++) turns_out[j] = turn[j];
    }
    if (stats) {
        stats->cells = (uint8_t)n;
        stats->turns = (uint8_t)nt;
        stats->dropped = 0;
        stats->uturns = 0;
        stats->diags = (uint8_t)n_diag;
        stats->turns135 = (uint8_t)n_135;
        stats->slowed = 0;
        for (int j = 0; j < nt; j++) {
            if (is_uturn(j)) stats->uturns++;
            else if (rix[j] != 0) stats->dropped++;
            if (turn[j].v_cap_mm_s) stats->slowed++;
        }
        stats->length_mm = path.length_mm();
        stats->min_straight = (nt > 1) ? min_straight : 0.0f;
        stats->same_pairs = 0;
        for (int j = 1; j < nt; j++)
            if (turn[j].sweep_deg == turn[j - 1].sweep_deg &&
                !is_uturn(j) && !is_uturn(j - 1))
                stats->same_pairs++;
    }
    return ROUTE_OK;
}

// THE MULTI-PASS COMPILE (Route.h, NEVER REFUSE A VALID MAZE).
//
// Pass 1 compiles the walk with every feature it can merge. If the fit
// refuses a pair, the refusal names the pair; the feature responsible --
// a U-turn first (its setback is the one thing the fit cannot shrink),
// else the staircase one of the pair belongs to -- is vetoed by walk cell
// and the walk is compiled again from scratch, feature list and all, so
// no half-built turn list is ever edited. Everything the refusal did not
// touch is kept: that is the whole difference from the old route-global
// ladder, which follows here as a last resort and should be unreachable
// now that the speed rung fits any pair of plain 90s.
static RouteResult compile_walk_multipass(const Maze &maze, const uint8_t *rx,
                                          const uint8_t *ry, const Heading *rh,
                                          int n, Heading h0, SpeedPath &path,
                                          float v_mm_s, float slew_dps2,
                                          RouteStats *stats, RouteTurn *turns_out,
                                          int max_turns_out, float lead_in_mm) {
    RouteVeto veto[ROUTE_MAX_VETO];
    int nveto = 0;
    RouteResult r = ROUTE_NO_RADIUS;
    int passes = 0;
    for (;;) {
        RouteRefusal ref{};
        passes++;
        r = compile_walk(maze, rx, ry, rh, n, h0, path, v_mm_s, slew_dps2,
                         stats, turns_out, max_turns_out, lead_in_mm,
                         ROUTE_UTURN_ENABLED, route_diag_enabled(),
                         veto, nveto, &ref);
        if (r == ROUTE_OK && turns_out && stats && nveto < ROUTE_MAX_VETO) {
            // A diagonal exit handing straight into a U-turn (Route.h
            // ROUTE_DIAG_EXIT_TO_UTURN_MM): un-merge the diagonal, compile
            // again. The U keeps its merge; its approach becomes a corridor.
            int bad = -1;
            for (int k = 0; k + 1 < (int)stats->turns && k + 1 < max_turns_out; k++) {
                const RouteTurn &a = turns_out[k], &b = turns_out[k + 1];
                bool diag_exit = a.diag_mm > 0 &&
                                 (a.sweep_deg == 45 || a.sweep_deg == -45 ||
                                  a.sweep_deg == 135 || a.sweep_deg == -135);
                bool u = b.sweep_deg == 180 || b.sweep_deg == -180;
                float gap = path.corner_entry_s_mm(k + 1) - path.corner_exit_s_mm(k);
                if (ROUTE_DIAG_EXIT_TO_UTURN_MM > 0.0f && diag_exit && u &&
                    gap < ROUTE_DIAG_EXIT_TO_UTURN_MM) { bad = k; break; }
                // A V into an opposite-hand 135-out (ROUTE_V_TO_OPP_135_MM).
                bool v_then_135 = route_turn_is_v(a) &&
                                  (b.sweep_deg == 135 || b.sweep_deg == -135) && b.diag_mm > 0 &&
                                  ((a.sweep_deg > 0) != (b.sweep_deg > 0));
                if (ROUTE_V_TO_OPP_135_MM > 0.0f && v_then_135 && gap < ROUTE_V_TO_OPP_135_MM) { bad = k + 1; break; }
            }
            if (bad >= 0) {
                // The whole diagonal chain behind the exit -- back through
                // any Vs and 45/135-ins to the last corridor turn -- is
                // vetoed together: un-merging only the last run would leave
                // its V as a 45-out at the dropped radius, 20 mm before a
                // 90 (seen on the host: R45 + R45).
                int added = 0;
                for (int k = bad; k >= 0; k--) {
                    const RouteTurn &t = turns_out[k];
                    bool fam = t.sweep_deg == 45 || t.sweep_deg == -45 ||
                               t.sweep_deg == 135 || t.sweep_deg == -135 ||
                               route_turn_is_v(t);
                    if (!fam) break;
                    RouteVeto v = {t.cell_ix, ROUTE_VETO_DIAG};
                    bool dup = false;
                    for (int j = 0; j < nveto; j++)
                        if (veto[j].cell_ix == v.cell_ix && veto[j].kind == v.kind) dup = true;
                    if (!dup && nveto < ROUTE_MAX_VETO) { veto[nveto++] = v; added++; }
                    // ...and no U-merge on those cells either: an un-merged
                    // 135 leaves two same-hand 90s one cell apart, which IS
                    // the U shape, and two of those back to back refuse the
                    // fit and veto the route's real U-turns (seen on maze 2:
                    // 74 turns, 0 Us). They fly as the plain same-hand pair.
                    RouteVeto vu = {t.cell_ix, ROUTE_VETO_UTURN};
                    bool dupu = false;
                    for (int j = 0; j < nveto; j++)
                        if (veto[j].cell_ix == vu.cell_ix && veto[j].kind == vu.kind) dupu = true;
                    if (!dupu && nveto < ROUTE_MAX_VETO) { veto[nveto++] = vu; }
                }
                if (added) continue;
            }
        }
        if (r != ROUTE_NO_RADIUS) break;
        if (nveto >= ROUTE_MAX_VETO) break;
        RouteVeto v{};
        if (ref.uturn_b)      v = {ref.cell_b, ROUTE_VETO_UTURN};
        else if (ref.uturn_a) v = {ref.cell_a, ROUTE_VETO_UTURN};
        else if (ref.diag_b)  v = {ref.cell_b, ROUTE_VETO_DIAG};
        else if (ref.diag_a)  v = {ref.cell_a, ROUTE_VETO_DIAG};
        else break;                       // two plain 90s: nothing to un-merge
        bool dup = false;
        for (int k = 0; k < nveto; k++)
            if (veto[k].cell_ix == v.cell_ix && veto[k].kind == v.kind) dup = true;
        if (dup) break;                   // the same refusal twice: stop
        veto[nveto++] = v;
    }
    if (r == ROUTE_NO_RADIUS) {
        // The old global ladder, kept as the floor under the floor.
        const bool tries[2][2] = {{true, false}, {false, false}};
        for (int t = 0; t < 2 && r == ROUTE_NO_RADIUS; t++) {
            if (tries[t][0] && !ROUTE_UTURN_ENABLED) continue;
            passes++;
            r = compile_walk(maze, rx, ry, rh, n, h0, path, v_mm_s, slew_dps2,
                             stats, turns_out, max_turns_out, lead_in_mm,
                             tries[t][0], tries[t][1]);
        }
    }
    if (stats) {
        stats->vetoed = (uint8_t)nveto;
        stats->passes = (uint8_t)passes;
    }
    return r;
}

// ---------------------------------------------------------------- compare
// THE ROUTE IS CHOSEN BY FLIGHT TIME (bert, 2026-09-27: "on this maze the
// diagonal should be the shortest and quickest path but the mouse chose the
// straights -- does the mouse compare routes or pick the first solved
// one?"). It picked one: the walk (fewest cells, or with rturn fewest
// cells + turns) was chosen on the GRID, and the diagonals were only found
// afterwards, when the compiler merged the staircase it had been handed.
// So rturn, which counts every cell-level heading change, charged the
// staircase that becomes ONE diagonal eight turns -- and steered away from
// the quickest route on the maze. Measured on the 09-27 map (rdiag 1, MR5
// limits): the fewest-cells walk compiles to 135-in / diagonal / 135-out /
// 90, 2375 mm, ~3040 ms; the fewest-turns walk to 6 turns, 3284 mm, ~4650.
//
// Now, with rturn > 0 and a time context from the runner, BOTH walks are
// compiled -- the diagonals merged, the radii fitted, the speed caps on --
// and each is flown through the planner (SpeedPath::sample at the leg's own
// v and accelerations, clothoid hold as set) to a commanded time. The score
// is that time plus rturn cells' worth of time (at the leg's peak) per
// COMPILED turn, a U counting two: the planner does not model placement,
// and every arc is a post to miss -- the reason rturn exists. The lower
// score flies. A walk whose first step is not the launch heading is not a
// candidate (a leg launches straight). Without a time context (the host
// tests of the walk itself) the default walk is used, as before.
static bool tctx_set = false;
static float tctx_v = 0.0f, tctx_alon = 0.0f, tctx_alat = 0.0f,
             tctx_abrk = 0.0f, tctx_slew = 0.0f;
static bool tctx_chold = true;
void route_set_time_context(float v_peak, float a_lon, float a_lat, float a_brk,
                            bool clothoid_hold, float slew_dps2) {
    tctx_set = v_peak > 0.0f && a_lon > 0.0f && a_lat > 0.0f;
    tctx_v = v_peak; tctx_alon = a_lon; tctx_alat = a_lat; tctx_abrk = a_brk;
    tctx_chold = clothoid_hold; tctx_slew = slew_dps2;
}
void route_clear_time_context() { tctx_set = false; }

float route_path_time_ms(SpeedPath &p) {
    if (!tctx_set || p.length_mm() <= 0.0f) return 0.0f;
    p.set_limits(tctx_v, tctx_alon, tctx_alat, tctx_abrk);
    p.set_clothoid_hold(tctx_chold);
    const float dt = 0.002f;
    float v = 0.0f, s = 0.0f;
    int ticks = 0;
    while (s < p.length_mm() - 0.5f && ticks < 30000) {
        SpeedPath::Sample sm = p.sample(s, v, dt);
        v = sm.v_mm_s;
        s += (v > 5.0f ? v : 5.0f) * dt;   // never stall the walk at rest
        ticks++;
    }
    return ticks * dt * 1000.0f;
}

static RouteChoice last_choice;
static int last_mode = -1;   // the walk mode the last route_build flew
const RouteChoice &route_last_choice() { return last_choice; }

static float choice_score(float ms, const RouteStats &st) {
    float turns = (float)st.turns + (float)st.uturns;   // a U is a 180: two
    float cell_ms = 1000.0f * ROUTE_CELL_MM / (tctx_v > 1.0f ? tctx_v : 1.0f);
    return ms + ctx_turn_cost * cell_ms * turns;
}

PATH_STORAGE static SpeedPath cmp_path;                 // the second candidate's compile
static RouteTurn cmp_turns[ROUTE_MAX_TURNS];

// Compile the best of the candidate walks from (x0, y0, h0) into `path`.
// *score_out (if given) receives the winner's score (0 when not compared).
static RouteResult choose_and_compile(Maze &maze, int x0, int y0, Heading h0,
                                      SpeedPath &path, float v_mm_s,
                                      float slew_dps2, RouteStats *stats,
                                      RouteTurn *turns_out, int max_turns_out,
                                      float lead_in_mm, float *score_out) {
    uint8_t rx[ROUTE_MAX_CELLS], ry[ROUTE_MAX_CELLS];
    Heading rh[ROUTE_MAX_CELLS];
    last_choice = RouteChoice{};
    last_mode = -1;
    if (score_out) *score_out = 0.0f;
    if (!(ctx_turn_cost > 0.0f && tctx_set)) {
        int n = 0, cx = x0, cy = y0;
        Heading h = h0;
        RouteResult w = walk_flood(maze, &cx, &cy, &h, rx, ry, rh, &n);
        if (w != ROUTE_OK) return w;
        return compile_walk_multipass(maze, rx, ry, rh, n, h0, path, v_mm_s,
                                      slew_dps2, stats, turns_out, max_turns_out,
                                      lead_in_mm);
    }
    // Candidate 0 (fewest cells) straight into the caller's path; candidate
    // 1 (fewest turns) into the scratch; keep the better.
    uint8_t rx0[ROUTE_MAX_CELLS], ry0[ROUTE_MAX_CELLS];
    int n0 = -1;
    RouteStats st[2] = {};
    float ms[2] = {0, 0}, sc[2] = {0, 0};
    bool ok[2] = {false, false};
    RouteResult res[2] = {ROUTE_NO_PATH, ROUTE_NO_PATH};
    for (int m = 0; m < 2; m++) {
        int n = 0, cx = x0, cy = y0;
        Heading h = h0;
        if (walk_flood(maze, &cx, &cy, &h, rx, ry, rh, &n, m) != ROUTE_OK) continue;
        if (n == 0 || rh[0] != h0) continue;          // a leg launches straight
        if (m == 0) { n0 = n; memcpy(rx0, rx, n); memcpy(ry0, ry, n); }
        else if (n == n0 && !memcmp(rx, rx0, n) && !memcmp(ry, ry0, n)) {
            last_choice.same_walk = true;             // the same cells: nothing to compare
            continue;
        }
        SpeedPath &cp = (m == 0) ? path : cmp_path;
        RouteTurn *ct = (m == 0) ? turns_out : cmp_turns;
        int cmax = (m == 0) ? max_turns_out : ROUTE_MAX_TURNS;
        res[m] = compile_walk_multipass(maze, rx, ry, rh, n, h0, cp, v_mm_s,
                                        slew_dps2, &st[m], ct, cmax, lead_in_mm);
        if (res[m] != ROUTE_OK) continue;
        ok[m] = true;
        ms[m] = route_path_time_ms(cp);
        sc[m] = choice_score(ms[m], st[m]);
        last_choice.cand[m].valid = true;
        last_choice.cand[m].cells = st[m].cells;
        last_choice.cand[m].turns = st[m].turns;
        last_choice.cand[m].uturns = st[m].uturns;
        last_choice.cand[m].diags = st[m].diags;
        last_choice.cand[m].length_mm = st[m].length_mm;
        last_choice.cand[m].ms = ms[m];
        last_choice.cand[m].score = sc[m];
    }
    int win = -1;
    if (ok[0] && ok[1]) win = (sc[1] < sc[0]) ? 1 : 0;
    else if (ok[0]) win = 0;
    else if (ok[1]) win = 1;
    if (win < 0) return ok[0] ? res[0] : (res[0] != ROUTE_NO_PATH ? res[0] : res[1]);
    last_choice.compared = ok[0] && ok[1];
    last_choice.chosen = (uint8_t)win;
    last_mode = win;
    if (win == 1) {
        path = cmp_path;
        if (turns_out && max_turns_out > 0) {
            int k = st[1].turns < max_turns_out ? st[1].turns : max_turns_out;
            memcpy(turns_out, cmp_turns, sizeof(RouteTurn) * k);
        }
    }
    if (stats) *stats = st[win];
    if (score_out) *score_out = sc[win];
    return ROUTE_OK;
}

static RouteResult route_build_pass(Maze &maze, int x0, int y0, Heading h0,
                                    SpeedPath &path, float v_mm_s,
                                    float slew_dps2, RouteStats *stats,
                                    RouteTurn *turns_out, int max_turns_out,
                                    float lead_in_mm) {
    return choose_and_compile(maze, x0, y0, h0, path, v_mm_s, slew_dps2, stats,
                              turns_out, max_turns_out, lead_in_mm, nullptr);
}

bool route_walk_end(Maze &maze, int x, int y, Heading h,
                    int *ex, int *ey, Heading *eh) {
    uint8_t rx[ROUTE_MAX_CELLS], ry[ROUTE_MAX_CELLS];
    Heading rh[ROUTE_MAX_CELLS];
    int n = 0;
    if (walk_flood(maze, &x, &y, &h, rx, ry, rh, &n, last_mode) != ROUTE_OK) return false;
    *ex = x; *ey = y; *eh = h;
    return true;
}

RouteResult route_build(Maze &maze, int x0, int y0, Heading h0,
                        SpeedPath &path, float v_mm_s, float slew_dps2,
                        RouteStats *stats, RouteTurn *turns_out,
                        int max_turns_out, float lead_in_mm) {
    return route_build_pass(maze, x0, y0, h0, path, v_mm_s, slew_dps2, stats,
                            turns_out, max_turns_out, lead_in_mm);
}

// ---------------------------------------------------------------- loop
RouteResult route_build_loop(Maze &maze, int x0, int y0, Heading h0,
                             SpeedPath &path, float v_mm_s, float slew_dps2,
                             RouteStats *stats, RouteTurn *turns_out,
                             int max_turns_out, float lead_in_mm,
                             int force_exit) {
    uint8_t rx[ROUTE_MAX_CELLS], ry[ROUTE_MAX_CELLS];
    Heading rh[ROUTE_MAX_CELLS];
    int n = 0, cx = x0, cy = y0;
    Heading h = h0;

    // route.dir: choose which way round a closed circuit the lap runs, by
    // temporarily walling the start cell on this COPY of the maze. OUT leg:
    // every KNOWN-OPEN exit except the forced one is closed, so the walk can
    // only leave through it. HOME leg: only the forced side is closed, so
    // the walk must come home through a DIFFERENT side -- the other arm.
    // Each closure is undone before the other flood; nothing here survives
    // the call.
    const Heading sides[4] = {HEADING_N, HEADING_E, HEADING_S, HEADING_W};
    bool closed_out[4] = {false, false, false, false};
    if (force_exit >= 0) {
        // The forced side must actually be a KNOWN-OPEN exit, or the out
        // walk is impossible by construction.
        if (!maze.wall_known(x0, y0, (Heading)force_exit) ||
            maze.wall_present(x0, y0, (Heading)force_exit))
            return ROUTE_NO_PATH;
        for (int i = 0; i < 4; i++) {
            if (sides[i] == (Heading)force_exit) continue;
            if (maze.wall_known(x0, y0, sides[i]) &&
                !maze.wall_present(x0, y0, sides[i])) {
                maze.set_wall(x0, y0, sides[i], true);
                closed_out[i] = true;
            }
        }
    }
    maze.flood_to_goal(false);
    RouteResult w = walk_flood(maze, &cx, &cy, &h, rx, ry, rh, &n);
    if (force_exit >= 0) {
        for (int i = 0; i < 4; i++)   // reopen before anything else
            if (closed_out[i]) maze.set_wall(x0, y0, sides[i], false);
    }
    if (w != ROUTE_OK) return w;
    if (n == 0) return ROUTE_NO_PATH;
    int goal_ix = n;              // the goal cell is the one the walk ENDS on

    // Re-flood and keep walking from exactly where the first walk stopped --
    // same cell, same heading. The join is invisible to the compiler: cell
    // goal_ix simply has an arrival heading from the outbound walk and a
    // departure heading from the return one, and the turn finder does the
    // rest.
    // A start cell with ONE known exit (the rule from D11 on: the start has
    // its east wall) cannot be entered from another side, so the "not this
    // way home" closure would only refuse every lap; the circuit, if there
    // is one, runs through the goal block instead and the door closed at
    // the goal (below) is what chooses its direction.
    int open_exits = 0;
    for (int i = 0; i < 4; i++)
        if (maze.wall_known(x0, y0, sides[i]) &&
            !maze.wall_present(x0, y0, sides[i]))
            open_exits++;
    const bool close_home = force_exit >= 0 && open_exits > 1;
    if (close_home)
        maze.set_wall(x0, y0, (Heading)force_exit, true);  // home: not this way
    // A loop must not come home the way it went out. On a circuit with
    // UNEQUAL arms the plain home flood walks straight back the shorter
    // arm -- an about-face in the goal cell, which the turn finder rightly
    // refuses (ROUTE_BAD_TURN). D6's equal arms hid this: the tie-break's
    // back-penalty broke the tie away from the reversal, but on D7
    // (staircase arm 10, tooth arm 14; bert, 2026-09-04, "mouse wouldn't
    // run") shorter genuinely wins and route.dir=auto could never compile.
    // So close the door the out leg arrived through for the home flood,
    // exactly the trick force_exit plays at the start cell. If that leaves
    // no way home (a dead-end goal -- not a circuit), reopen and take the
    // plain flood, which behaves as before.
    const Heading arrived_door = heading_back(h);
    const bool close_door = !maze.wall_present(cx, cy, arrived_door);
    const int hx = cx, hy = cy, hn = n;
    const Heading hh = h;
    if (close_door) maze.set_wall(cx, cy, arrived_door, true);
    maze.flood_to_start(false);
    w = walk_flood(maze, &cx, &cy, &h, rx, ry, rh, &n);
    if (close_door) {
        maze.set_wall(hx, hy, arrived_door, false);
        if (w != ROUTE_OK) {                   // not a circuit: plain flood
            cx = hx; cy = hy; h = hh; n = hn;
            maze.flood_to_start(false);
            w = walk_flood(maze, &cx, &cy, &h, rx, ry, rh, &n);
        }
    }
    if (close_home)
        maze.set_wall(x0, y0, (Heading)force_exit, false);
    if (w != ROUTE_OK) return w;
    if (n <= goal_ix) return ROUTE_NO_PATH;   // no way home

    RouteResult r = compile_walk_multipass(maze, rx, ry, rh, n, h0, path, v_mm_s,
                                           slew_dps2, stats, turns_out,
                                           max_turns_out, lead_in_mm);
    if (r == ROUTE_OK && stats) stats->goal_ix = (uint8_t)goal_ix;
    return r;
}

// ---------------------------------------------------------------- goal loop
// The block's geometry around the arrival cell: the cell ahead must be a
// goal cell, and exactly one side must have a goal cell beside the arrival
// cell (a 2x2 block has one); the four interior walls must be KNOWN OPEN.
static bool goal_loop_cells(const Maze &maze, int gx, int gy, Heading h_in,
                            Heading *side_out, int *ax, int *ay, int *bx, int *by,
                            int *cx, int *cy) {
    if (!maze.in_goal(gx, gy)) return false;
    int a_x = gx + heading_dx(h_in), a_y = gy + heading_dy(h_in);
    if (a_x < 0 || a_y < 0 || a_x >= MAZE_SIZE || a_y >= MAZE_SIZE) return false;
    if (!maze.in_goal(a_x, a_y)) return false;
    Heading side = h_in;
    bool found = false;
    const Heading cand[2] = {heading_right(h_in), heading_left(h_in)};
    for (int i = 0; i < 2; i++) {
        int c_x = gx + heading_dx(cand[i]), c_y = gy + heading_dy(cand[i]);
        if (c_x < 0 || c_y < 0 || c_x >= MAZE_SIZE || c_y >= MAZE_SIZE) continue;
        if (!maze.in_goal(c_x, c_y)) continue;
        int b_x = a_x + heading_dx(cand[i]), b_y = a_y + heading_dy(cand[i]);
        if (!maze.in_goal(b_x, b_y)) continue;
        side = cand[i]; found = true;
        *ax = a_x; *ay = a_y; *bx = b_x; *by = b_y; *cx = c_x; *cy = c_y;
        break;
    }
    if (!found) return false;
    auto open_known = [&](int x, int y, Heading h) {
        return maze.wall_known(x, y, h) && !maze.wall_present(x, y, h);
    };
    if (!open_known(gx, gy, h_in)) return false;            // arrival -> ahead
    if (!open_known(*ax, *ay, side)) return false;          // ahead -> across
    if (!open_known(*bx, *by, heading_back(h_in))) return false;   // across -> back
    if (!open_known(*cx, *cy, heading_back(side))) return false;   // back -> arrival
    *side_out = side;
    return true;
}

bool route_goal_loop_possible(const Maze &maze, int gx, int gy, Heading h_in) {
    Heading side; int ax, ay, bx, by, cx, cy;
    return goal_loop_cells(maze, gx, gy, h_in, &side, &ax, &ay, &bx, &by, &cx, &cy);
}

static int gl_end_x = -1, gl_end_y = -1;
static Heading gl_end_h = HEADING_N;
bool route_goal_loop_end(int *ex, int *ey, Heading *eh) {
    if (gl_end_x < 0) return false;
    *ex = gl_end_x; *ey = gl_end_y; *eh = gl_end_h;
    return true;
}

RouteResult route_build_goal_loop(Maze &maze, int gx, int gy, Heading h_in,
                                  SpeedPath &path, float v_mm_s, float slew_dps2,
                                  RouteStats *stats, RouteTurn *turns_out,
                                  int max_turns_out) {
    gl_end_x = -1;
    Heading side; int ax, ay, bx, by, cx, cy;
    if (!goal_loop_cells(maze, gx, gy, h_in, &side, &ax, &ay, &bx, &by, &cx, &cy))
        return ROUTE_NO_PATH;
    uint8_t rx[ROUTE_MAX_CELLS], ry[ROUTE_MAX_CELLS];
    Heading rh[ROUTE_MAX_CELLS];
    int n = 0;
    // One forced step: AHEAD into the block (a leg launches straight). From
    // there the home flood decides -- out through a door of the far row or
    // the far column if the block has one, or back round to the arrival
    // door. The latter is a 270 in two cells of room (three same-direction
    // 90s one cell apart and a fourth the other way): the fit cannot hold a
    // U-turn and two 90s in it and un-merges to four tight arcs, the route
    // type with the worst record. So a walk that comes back through the
    // arrival cell is refused here and the runner turns in place instead;
    // the loop is for the goal with a second door (bert's case).
    rx[n] = (uint8_t)gx; ry[n] = (uint8_t)gy; rh[n] = h_in; n++;
    maze.flood_to_start(false);
    int x = ax, y = ay;
    Heading h = h_in;
    RouteResult w = walk_flood(maze, &x, &y, &h, rx, ry, rh, &n);
    if (w != ROUTE_OK) return w;
    if (n <= 1) return ROUTE_NO_PATH;
    if (rh[1] == heading_back(h_in)) return ROUTE_BAD_TURN;   // an about-face in the block
    for (int i = 1; i < n; i++)
        if (rx[i] == gx && ry[i] == gy) return ROUTE_NO_PATH;  // back through the arrival cell
    gl_end_x = x; gl_end_y = y; gl_end_h = h;
    return compile_walk_multipass(maze, rx, ry, rh, n, h_in, path, v_mm_s,
                                  slew_dps2, stats, turns_out, max_turns_out, 0.0f);
}

// A dry run of the loop build into a scratch path: the runner asks this at
// the goal, with the home leg's context set, before choosing loop or turn.
PATH_STORAGE static SpeedPath gl_probe;
bool route_goal_loop_viable(Maze &maze, int gx, int gy, Heading h_in,
                            float v_mm_s, float slew_dps2) {
    RouteStats st{};
    return route_build_goal_loop(maze, gx, gy, h_in, gl_probe, v_mm_s, slew_dps2,
                                 &st, nullptr, 0) == ROUTE_OK;
}

// ---------------------------------------------------------------- two legs
bool route_home_heading(Maze &maze, int gx, int gy, Heading arrived,
                        Heading &out) {
    maze.flood_to_start(false);
    if (maze.distance(gx, gy) == MAZE_DIST_UNREACHABLE) return false;
    if (maze.distance(gx, gy) == 0) return false;   // already home
    // The first step of the walk the home leg will actually be compiled
    // from (rturn's weighted walk may leave the goal by a different door
    // than the plain flood's best step). With a time context, each
    // candidate door is compiled and scored the way the home leg will be,
    // and the better door is the one the goal turn faces.
    uint8_t rx[ROUTE_MAX_CELLS], ry[ROUTE_MAX_CELLS];
    Heading rh[ROUTE_MAX_CELLS];
    Heading door[2];
    bool have[2] = {false, false};
    for (int m = 0; m < 2; m++) {
        int n = 0, x = gx, y = gy;
        Heading h = arrived;
        if (walk_flood(maze, &x, &y, &h, rx, ry, rh, &n, m) == ROUTE_OK && n > 0) {
            door[m] = rh[0];
            have[m] = true;
        }
    }
    if (!have[0] && !have[1]) return maze.best_step_known(gx, gy, arrived, out);
    if (!(ctx_turn_cost > 0.0f && tctx_set) || !have[0] || !have[1] || door[0] == door[1]) {
        out = have[ctx_turn_cost > 0.0f ? 1 : 0] ? door[ctx_turn_cost > 0.0f ? 1 : 0]
                                                 : door[have[0] ? 0 : 1];
        return true;
    }
    PATH_STORAGE static SpeedPath probe;
    float best = 0.0f;
    int pick = -1;
    for (int m = 0; m < 2; m++) {
        float sc = 0.0f;
        RouteStats st{};
        if (choose_and_compile(maze, gx, gy, door[m], probe, tctx_v, tctx_slew, &st,
                               nullptr, 0, 0.0f, &sc) != ROUTE_OK) continue;
        if (pick < 0 || sc < best) { best = sc; pick = m; }
    }
    out = door[pick >= 0 ? pick : 1];
    return true;
}
