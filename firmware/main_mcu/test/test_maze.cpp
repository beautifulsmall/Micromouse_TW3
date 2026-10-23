// Host-side test: maze model (lib/Maze/Maze.*) -- edge-shared wall storage,
// optimistic/pessimistic flood fill, and the best-step policy. Ends with a
// full flood-fill SEARCH SIMULATION against a "truth" maze: sense the current
// cell, update the model, re-flood, step -- the exact loop the search
// executor will run on hardware. Pure logic, no stub needed.
//
// Build:
//   g++ -std=c++17 -I../lib/Maze ../lib/Maze/Maze.cpp test_maze.cpp -o test_maze && ./test_maze

#include "Maze.h"

#include <cstdio>

static int failures = 0;
static void expect(const char *what, bool cond) {
    if (!cond) { std::printf("FAIL: %s\n", what); failures++; }
}

// Sense one cell of `truth` into `model`: what the mouse does on arriving in
// a cell (side sensors + front). Truth mazes only set PRESENT walls, so
// unknown truth edges read open -- exactly a wall-less edge.
static void sense_cell(Maze &model, const Maze &truth, int x, int y) {
    for (int h = 0; h < 4; h++) {
        model.set_wall(x, y, Heading(h), truth.wall_present(x, y, Heading(h)));
    }
    model.set_visited(x, y);
}

// Run the flood-fill search loop on `truth` from (0,0,N) until in_goal or
// step budget exhausted. Returns steps taken, -1 if stuck/budget blown.
static int simulate_search(Maze &model, const Maze &truth) {
    int x = 0, y = 0;
    Heading hd = HEADING_N;
    for (int step = 0; step < 400; step++) {
        if (model.in_goal(x, y)) return step;
        sense_cell(model, truth, x, y);
        model.flood_to_goal(true);
        Heading next;
        if (!model.best_step(x, y, hd, next)) return -1;
        hd = next;
        x += heading_dx(hd);
        y += heading_dy(hd);
    }
    return -1;
}

// ------------------------------------------------------------ full exploration
//
// A goal search stops as soon as a route is proven and routinely leaves a
// third of the maze unseen. These cover the frontier flood that sweeps up
// the rest, and in particular the ENCLOSED SUB-MAZE case -- a 6x6 rig inside
// a 16x16 model, where 220 cells are unknown forever and a naive "all walls
// known" completion test would hang the mission.

// Wall in a 6x6 box at the origin, everything outside untouched.
static void build_enclosed_6x6(Maze &m) {
    m.reset();
    for (int i = 0; i < 6; i++) {
        m.set_wall(i, 0, HEADING_S, true);
        m.set_wall(i, 5, HEADING_N, true);
        m.set_wall(0, i, HEADING_W, true);
        m.set_wall(5, i, HEADING_E, true);
    }
}

static void test_unknown_wall_accounting() {
    Maze m;
    m.reset();
    expect("a fresh cell has unknown walls", m.has_unknown_wall(2, 2));
    for (int h = 0; h < 4; h++) m.set_wall(2, 2, Heading(h), false);
    expect("all four set -> nothing unknown", !m.has_unknown_wall(2, 2));
    // Edges are SHARED, so resolving them from (2,2) also resolves the
    // touching edge of each neighbour. That is what lets the search skip a
    // cell it never entered, instead of driving to it for nothing.
    expect("the neighbour's shared edge came with it",
           m.wall_known(2, 3, HEADING_S));
    expect("but the neighbour is not fully known yet",
           m.has_unknown_wall(2, 3));
    expect("out of range is not unknown", !m.has_unknown_wall(-1, 0));
}

static void test_frontier_flood_points_at_the_nearest_unknown() {
    Maze m;
    m.reset();
    // Resolve a corridor of cells so the frontier is a known distance away.
    for (int y = 0; y <= 3; y++)
        for (int h = 0; h < 4; h++) m.set_wall(0, y, Heading(h), h == HEADING_E ? true : false);
    m.flood_to_unknown(true);
    expect("a resolved cell is not itself a seed", m.distance(0, 0) > 0);
    expect("distance falls toward the frontier",
           m.distance(0, 0) > m.distance(0, 3));
    Heading step;
    expect("there is a step to take", m.best_step(0, 0, HEADING_N, step));
    expect("and it heads up the corridor", step == HEADING_N);
}

static void test_enclosed_6x6_terminates() {
    Maze m;
    build_enclosed_6x6(m);
    // Nothing outside the box can ever be resolved, so a completion test
    // based on "no unknown wall anywhere" would never fire.
    int outside = 0;
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            if ((x >= 6 || y >= 6) && m.has_unknown_wall(x, y)) outside++;
    std::printf("  enclosed 6x6: %d cells outside the box, unknown forever\n",
                outside);
    expect("the outside really is permanently unknown", outside > 200);

    // Resolve every edge INSIDE the box, as a completed search would.
    for (int y = 0; y < 6; y++)
        for (int x = 0; x < 6; x++)
            for (int h = 0; h < 4; h++)
                if (!m.wall_known(x, y, Heading(h)))
                    m.set_wall(x, y, Heading(h), false);

    expect("the mission can now declare itself done",
           !m.unknown_reachable(0, 0, true));
    expect("even though most of the model is still unknown",
           m.has_unknown_wall(10, 10));
}

static void test_exploration_actually_finishes() {
    // Drive the frontier flood to exhaustion the way the mission does:
    // walk to the nearest unknown, resolve that cell, repeat. It must
    // terminate, and it must leave the whole box known.
    Maze m;
    build_enclosed_6x6(m);
    int x = 0, y = 0, guard = 0;
    while (m.unknown_reachable(x, y, true) && guard++ < 500) {
        m.flood_to_unknown(true);
        if (m.distance(x, y) != 0) {
            Heading step;
            if (!m.best_step(x, y, HEADING_N, step)) break;
            x += heading_dx(step);
            y += heading_dy(step);
            continue;
        }
        for (int h = 0; h < 4; h++)
            if (!m.wall_known(x, y, Heading(h)))
                m.set_wall(x, y, Heading(h), false);
    }
    std::printf("  exploration converged in %d iterations\n", guard);
    expect("it terminated rather than spinning", guard < 500);
    int left = 0;
    for (int yy = 0; yy < 6; yy++)
        for (int xx = 0; xx < 6; xx++)
            if (m.has_unknown_wall(xx, yy)) left++;
    expect("every cell in the box ended up known", left == 0);
}


// ROUTE-AWARE EXPLORE (Maze::flood_to_useful_unknown, 2026-10-02): only cells
// that could lie on a route within `slack` of the best known one are seeds.
static void test_useful_unknown_skips_off_route() {
    Maze m;   // everything unknown but the boundary; goal the centre 2x2
    for (int y = 0; y < 7; y++) m.set_wall(0, y, HEADING_N, false);   // column 0 up
    for (int x = 0; x < 7; x++) m.set_wall(x, 7, HEADING_E, false);   // row 7 east
    int known = -1;
    int n = m.flood_to_useful_unknown(4, &known);
    expect("useful: the known route is 14 cells", known == 14);
    expect("useful: some cells are worth a look", n > 0);
    m.flood_to_useful_unknown(4);
    expect("useful: (3,3) could be on a 14-cell route -- a seed", m.distance(3, 3) == 0);
    expect("useful: (15,0) is 29 cells round -- not a seed", m.distance(15, 0) != 0);
    expect("useful: (12,12) is not a seed either", m.distance(12, 12) != 0);
    // A known dead-end pocket: wall (3,3) in on three sides and open to (3,2);
    // its last edge known -> no unknown edge left, never a seed.
    m.set_wall(3, 3, HEADING_N, true); m.set_wall(3, 3, HEADING_E, true);
    m.set_wall(3, 3, HEADING_W, true); m.set_wall(3, 3, HEADING_S, false);
    m.flood_to_useful_unknown(4);
    expect("useful: a fully known dead end is not a seed", m.distance(3, 3) != 0);
}

// The whole search on random 16x16 truth mazes: to the goal, then the
// route-aware explore, then check the route the map PROVES is the truth's
// shortest -- and that it cost fewer steps than mapping every wall.
static void gen_truth(Maze &t, unsigned seed) {
    static const int DX[4] = {0, 1, 0, -1}, DY[4] = {1, 0, -1, 0};
    unsigned r = seed * 2654435761u + 12345u;
    auto rnd = [&]() { r = r * 1103515245u + 12345u; return (r >> 8) & 0xFFFF; };
    t.reset();
    for (int x = 0; x < 16; x++)
        for (int y = 0; y < 16; y++)
            for (int h = 0; h < 4; h++) t.set_wall(x, y, Heading(h), true);
    static bool vis[16][16];
    static int st[256];
    for (int x = 0; x < 16; x++) for (int y = 0; y < 16; y++) vis[x][y] = false;
    int sp = 0; st[sp++] = 0; vis[0][0] = true;
    while (sp) {
        int c = st[sp - 1], x = c % 16, y = c / 16, o[4], n = 0;
        for (int h = 0; h < 4; h++) {
            int nx = x + DX[h], ny = y + DY[h];
            if (nx >= 0 && ny >= 0 && nx < 16 && ny < 16 && !vis[nx][ny]) o[n++] = h;
        }
        if (!n) { sp--; continue; }
        int h = o[rnd() % n], nx = x + DX[h], ny = y + DY[h];
        t.set_wall(x, y, Heading(h), false);
        vis[nx][ny] = true;
        st[sp++] = ny * 16 + nx;
    }
    for (int i = 0; i < 30; i++) t.set_wall(rnd() % 15, rnd() % 15, Heading(rnd() % 2), false);
    t.set_wall(7, 7, HEADING_E, false); t.set_wall(7, 8, HEADING_E, false);
    t.set_wall(7, 7, HEADING_N, false); t.set_wall(8, 7, HEADING_N, false);
    t.set_wall(0, 0, HEADING_E, true);   // the rules' start cell: open north only
    t.set_wall(0, 0, HEADING_N, false);
}
static int run_search(Maze &m, const Maze &t, bool useful) {
    int x = 0, y = 0, steps = 0;
    Heading hd = HEADING_N;
    int phase = 0;   // 0 to goal, 1 explore, 2 done
    while (steps < 3000 && phase < 2) {
        sense_cell(m, t, x, y);
        if (phase == 0 && m.in_goal(x, y)) phase = 1;
        if (phase == 1) {
            bool more = useful ? m.useful_unknown_reachable(x, y, 4) : m.unknown_reachable(x, y, true);
            if (!more) { phase = 2; break; }
            if (useful) m.flood_to_useful_unknown(4); else m.flood_to_unknown(true);
        } else {
            m.flood_to_goal(true);
        }
        Heading next;
        if (!m.best_step(x, y, hd, next)) return -1;
        hd = next; x += heading_dx(hd); y += heading_dy(hd); steps++;
    }
    return steps;
}
static void test_route_aware_explore_finds_the_shortest_route() {
    int worse = 0, total_u = 0, total_a = 0, n = 0;
    for (unsigned seed = 1; seed <= 60; seed++) {
        Maze t; gen_truth(t, seed);
        t.flood_to_goal(false);
        int truth = t.distance(0, 0);
        Maze a, u;
        int sa = run_search(a, t, false), su = run_search(u, t, true);
        if (sa < 0 || su < 0) { worse++; continue; }
        u.flood_to_goal(false);
        if (u.distance(0, 0) != truth) worse++;
        total_u += su; total_a += sa; n++;
    }
    std::printf("  route-aware explore: %d mazes, %d steps on average (map-everything %d), "
                "%d without the true shortest route\n", n, total_u / (n ? n : 1),
                total_a / (n ? n : 1), worse);
    expect("route-aware explore always proves the true shortest route", worse == 0);
    expect("...in fewer steps than mapping everything", total_u < total_a);
}

int main() {
    test_unknown_wall_accounting();
    test_frontier_flood_points_at_the_nearest_unknown();
    test_enclosed_6x6_terminates();
    test_exploration_actually_finishes();
    test_useful_unknown_skips_off_route();
    test_route_aware_explore_finds_the_shortest_route();
    // -- Heading helpers ---------------------------------------------------------
    expect("left of N is W", heading_left(HEADING_N) == HEADING_W);
    expect("right of N is E", heading_right(HEADING_N) == HEADING_E);
    expect("back of E is W", heading_back(HEADING_E) == HEADING_W);
    expect("dx of E", heading_dx(HEADING_E) == 1 && heading_dy(HEADING_E) == 0);
    expect("dy of S", heading_dx(HEADING_S) == 0 && heading_dy(HEADING_S) == -1);

    // -- Reset presets -------------------------------------------------------------
    Maze m;
    expect("outer S boundary known+present", m.wall_present(4, 0, HEADING_S));
    expect("outer W boundary known+present", m.wall_present(0, 9, HEADING_W));
    expect("outer N boundary known+present", m.wall_present(9, 15, HEADING_N));
    expect("outer E boundary known+present", m.wall_present(15, 3, HEADING_E));
    expect("start cell E wall preset", m.wall_present(0, 0, HEADING_E));
    expect("start E seen from (1,0) W", m.wall_present(1, 0, HEADING_W));
    expect("interior unknown", !m.wall_known(5, 5, HEADING_N));
    expect("unknown reads not-present", !m.wall_present(5, 5, HEADING_N));

    // -- Edge sharing ----------------------------------------------------------------
    m.set_wall(5, 5, HEADING_N, true);
    expect("edge shared with (5,6) S", m.wall_present(5, 6, HEADING_S));
    expect("edge known both sides", m.wall_known(5, 6, HEADING_S));
    m.set_wall(5, 6, HEADING_S, false);  // retract from the other side
    expect("edge cleared from (5,5) N", !m.wall_present(5, 5, HEADING_N));
    expect("cleared edge still known", m.wall_known(5, 5, HEADING_N));
    m.set_wall(9, 9, HEADING_E, true);
    expect("E edge shared with (10,9) W", m.wall_present(10, 9, HEADING_W));

    // -- Optimistic flood, empty maze ---------------------------------------------
    m.reset();
    m.flood_to_goal(true);
    expect("goal cell dist 0", m.distance(7, 7) == 0);
    expect("all four goal cells dist 0", m.distance(8, 8) == 0);
    expect("empty-maze start dist = Manhattan 14", m.distance(0, 0) == 14);
    expect("corner (15,15) dist 14", m.distance(15, 15) == 14);

    // -- Pessimistic flood, nothing known -------------------------------------------
    m.flood_to_goal(false);
    expect("goal still 0 pessimistic", m.distance(7, 7) == 0);
    expect("start unreachable pessimistic", m.distance(0, 0) == MAZE_DIST_UNREACHABLE);

    // -- Walls reshape the field ----------------------------------------------------
    m.reset();
    // Known wall across the y=6|7 boundary except a gap at x=15. This sits
    // BELOW the whole goal block (y >= 7), so every route from the south
    // must funnel through the east gap. (A wall at y=7|8 would prove
    // nothing -- the goal block straddles that boundary.)
    for (int x = 0; x < 15; x++) m.set_wall(x, 6, HEADING_N, true);
    m.flood_to_goal(true);
    // From (0,0): east to (15,0) = 15, north to (15,7) = 7, west to the
    // nearest goal column x=8 = 7 -> 29.
    expect("detour distance 29", m.distance(0, 0) == 29);
    expect("cell above gap cheap", m.distance(15, 7) < 10);

    // -- best_step preference -------------------------------------------------------
    m.reset();
    m.flood_to_goal(true);
    Heading out;
    expect("step exists from start", m.best_step(0, 0, HEADING_N, out));
    expect("facing N prefers straight N", out == HEADING_N);
    expect("step exists facing E", m.best_step(3, 3, HEADING_E, out));
    expect("facing E prefers straight E (tie)", out == HEADING_E);
    expect("step exists facing S at (3,3)", m.best_step(3, 3, HEADING_S, out));
    expect("facing S turns, never back (tie l/r)",
           out == HEADING_E || out == HEADING_N);
    // Boxed-in cell: no passable improving neighbour.
    m.set_wall(0, 0, HEADING_N, true);  // E preset + N,S,W boundary/walls
    m.flood_to_goal(true);
    expect("boxed-in start has no step", !m.best_step(0, 0, HEADING_N, out));

    // -- Traversed-edge locking (wheels outrank beams) --------------------------
    m.reset();
    m.set_wall_traversed(3, 3, HEADING_N);
    expect("traversed edge known-open",
           m.wall_known(3, 3, HEADING_N) && !m.wall_present(3, 3, HEADING_N));
    m.set_wall(3, 3, HEADING_N, true);  // sensor lies -- must be ignored
    expect("locked edge immune to wall write", !m.wall_present(3, 3, HEADING_N));
    m.set_wall(3, 4, HEADING_S, true);  // same edge from the other cell
    expect("locked from both sides", !m.wall_present(3, 4, HEADING_S));
    m.set_wall(0, 15, HEADING_N, false);  // sensor claims boundary opening
    expect("boundary locked against opening", m.wall_present(0, 15, HEADING_N));
    m.set_wall(0, 0, HEADING_E, false);  // rules wall may not be retracted
    expect("start E wall locked", m.wall_present(0, 0, HEADING_E));
    m.reset();
    expect("interior lock cleared by reset", !m.wall_known(3, 3, HEADING_N));
    expect("boundary re-locked after reset", m.wall_present(0, 15, HEADING_N));

    // -- Runtime goal override (practice rigs) ---------------------------------------
    m.reset();
    m.set_goal(2, 0, 2, 0);  // bert's test maze: goal at (2,0)
    expect("custom goal cell", m.in_goal(2, 0));
    expect("default goal no longer goal", !m.in_goal(7, 7));
    m.flood_to_goal(true);
    // Direct east route is blocked by the PRESET start east wall, so the
    // optimistic route arcs over: (0,0)N (0,1)E (1,1)E (2,1)S (2,0) = 4.
    expect("goal(2,0) dist from start 4", m.distance(0, 0) == 4);
    // The rules wall is LOCKED -- sensing may not retract it.
    m.set_wall(0, 0, HEADING_E, false);
    m.flood_to_goal(true);
    expect("goal(2,0) dist still 4 (E wall locked)", m.distance(0, 0) == 4);
    // Goal survives reset(); 'std' restore is the caller's job.
    m.reset();
    expect("goal survives reset", m.in_goal(2, 0) && !m.in_goal(7, 7));
    m.set_goal(MAZE_GOAL_MIN, MAZE_GOAL_MIN, MAZE_GOAL_MAX, MAZE_GOAL_MAX);
    expect("std goal restored", m.in_goal(7, 7) && m.in_goal(8, 8));
    m.set_goal(9, 3, 2, 12);  // reversed corners normalise
    expect("normalised rect", m.in_goal(2, 3) && m.in_goal(9, 12) &&
                              !m.in_goal(1, 3));
    m.set_goal(MAZE_GOAL_MIN, MAZE_GOAL_MIN, MAZE_GOAL_MAX, MAZE_GOAL_MAX);

    // -- Search simulation: empty maze ---------------------------------------------
    Maze truth;  // no interior walls
    Maze model;
    int steps = simulate_search(model, truth);
    expect("empty maze solved", steps >= 0);
    expect("empty maze solved in exactly 14 steps", steps == 14);

    // -- Search simulation: walled maze with detour + dead end ----------------------
    truth.reset();
    // The y=6|7 wall with a single gap at x=15 (below the goal block).
    for (int x = 0; x < 15; x++) truth.set_wall(x, 6, HEADING_N, true);
    // A dead-end pocket on the approach: corridor into (12,3..5) closed on
    // three sides -- flood must route around/out of it.
    for (int y = 3; y <= 5; y++) {
        truth.set_wall(12, y, HEADING_E, true);
        truth.set_wall(12, y, HEADING_W, true);
    }
    truth.set_wall(12, 5, HEADING_N, true);
    // Funnel below the gap so the mouse gets steered near the pocket.
    for (int x = 2; x < 12; x++) truth.set_wall(x, 3, HEADING_N, true);
    model.reset();
    steps = simulate_search(model, truth);
    expect("walled maze solved", steps >= 0);
    expect("walled maze within step budget", steps >= 0 && steps < 200);

    // After search, the discovered map must contain a known route: the
    // PESSIMISTIC flood from the goal must reach the start.
    model.flood_to_goal(false);
    expect("pessimistic route exists after search",
           model.distance(0, 0) != MAZE_DIST_UNREACHABLE);
    // And it must never be shorter than the optimistic bound on truth.
    truth.flood_to_goal(true);
    expect("known route >= truth optimum",
           model.distance(0, 0) >= truth.distance(0, 0));

    // -- Return leg: flood to start on the discovered map ----------------------------
    model.flood_to_start(true);
    expect("start dist 0 in return flood", model.distance(0, 0) == 0);
    expect("goal has finite return dist",
           model.distance(7, 7) != MAZE_DIST_UNREACHABLE);

    // -- Practice-rig simulation: goal (2,0), rules E wall present ------------
    // The rig always has the start E wall (and it is locked besides), so
    // the route arcs over the top: N, E, E, S = 4 steps.
    Maze rig_truth;  // default: E wall preset, no interior walls
    Maze rig_model;
    rig_model.set_goal(2, 0, 2, 0);
    steps = simulate_search(rig_model, rig_truth);
    expect("rig goal(2,0) reached", steps >= 0);
    expect("rig solved in 4 steps (around the rules wall)", steps == 4);

    // -- Speed-run walk: proven route via best_step_known ----------------------
    // After the walled-maze search, the model holds a proven route: the
    // pessimistic flood walked with best_step_known must reach the goal in
    // EXACTLY the pessimistic distance -- never through an unknown edge.
    {
        model.flood_to_goal(false);
        uint8_t d0 = model.distance(0, 0);
        expect("speed: start reachable pessimistically",
               d0 != MAZE_DIST_UNREACHABLE);
        int x = 0, y = 0, walked = 0;
        Heading h = HEADING_N;
        while (!model.in_goal(x, y) && walked < 300) {
            Heading nx;
            if (!model.best_step_known(x, y, h, nx)) break;
            h = nx;
            x += heading_dx(nx);
            y += heading_dy(nx);
            walked++;
        }
        expect("speed: walk reaches the goal", model.in_goal(x, y));
        expect("speed: walk length equals pessimistic distance",
               walked == (int)d0);
    }

    // -- FRAM persistence round trip -------------------------------------------
    // The saved map is about to have a SPEED RUN flown on it after a power
    // cycle, so "restored" has to mean identical in every bit that carries
    // trust -- not just where the walls are, but which edges are KNOWN, which
    // are LOCKED, and which cells were visited.
    {
        // simulate_search above only senses and steps -- marking an edge
        // TRAVERSED is the executor's job on hardware, so nothing in this
        // file has set a lock yet. Set one deliberately: locks are the bits
        // most easily lost in a serialisation and the most expensive to lose,
        // since they are what stops a later sensor read walling off an edge
        // the mouse has driven through.
        model.set_wall_traversed(0, 0, HEADING_N);

        Maze::State st;
        model.save_state(st);

        Maze restored;
        restored.set_goal(model.goal_x0(), model.goal_y0(), model.goal_x1(),
                          model.goal_y1());
        restored.load_state(st);

        bool same = true, same_known = true, same_visited = true;
        for (int y = 0; y < MAZE_SIZE; y++) {
            for (int x = 0; x < MAZE_SIZE; x++) {
                for (int h = 0; h < 4; h++) {
                    Heading hh = Heading(h);
                    if (restored.wall_present(x, y, hh) !=
                        model.wall_present(x, y, hh)) same = false;
                    if (restored.wall_known(x, y, hh) !=
                        model.wall_known(x, y, hh)) same_known = false;
                }
                if (restored.visited(x, y) != model.visited(x, y))
                    same_visited = false;
            }
        }
        expect("persist: walls identical after restore", same);
        expect("persist: KNOWN bits identical after restore", same_known);
        expect("persist: visited set identical after restore", same_visited);

        // The proven route must survive too -- that is the whole point.
        model.flood_to_goal(false);
        uint8_t d_before = model.distance(0, 0);
        restored.flood_to_goal(false);
        expect("persist: same pessimistic route length",
               restored.distance(0, 0) == d_before);
        expect("persist: route still proven",
               restored.distance(0, 0) != MAZE_DIST_UNREACHABLE);

        // LOCK bits: a traversed edge stays un-retractable after a restore.
        // (0,0)->(0,1) was driven in every simulation above, so sensing a
        // wall onto it must be ignored on the restored map exactly as it is
        // on the original.
        restored.set_wall(0, 0, HEADING_N, true);
        expect("persist: locked traversed edge still refuses a wall",
               !restored.wall_present(0, 0, HEADING_N));

        // The outer boundary is re-asserted on load, not merely copied, so
        // even an all-zero block cannot open it.
        Maze::State blank{};
        Maze zeroed;
        zeroed.load_state(blank);
        expect("persist: boundary re-asserted from an all-zero block",
               zeroed.wall_present(0, 0, HEADING_S) &&
               zeroed.wall_present(0, 0, HEADING_W) &&
               zeroed.wall_present(15, 15, HEADING_N));
        zeroed.flood_to_goal(false);
        expect("persist: all-zero block leaves no proven route",
               zeroed.distance(0, 0) == MAZE_DIST_UNREACHABLE);

        // A restore must not smuggle in a stale flood: distances are cleared,
        // so a caller that forgets to flood gets UNREACHABLE rather than a
        // route computed against some other goal.
        Maze fresh;
        fresh.load_state(st);
        expect("persist: distance field cleared by load_state",
               fresh.distance(0, 0) == MAZE_DIST_UNREACHABLE);
    }

    if (failures == 0) std::printf("test_maze: all tests passed\n");
    return failures ? 1 : 0;
}
