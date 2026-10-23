// THE PLANNER CANNOT REFUSE A MAZE (2026-10-02; bert: "the planner can't
// refuse a maze"). 300 random 16x16 mazes (perfect DFS mazes -- far more
// twisted than a competition maze -- two in three with 25 extra openings),
// centre 2x2 goal: the out leg compiles with diagonals at 1600, the home
// leg orthogonal at 400 with rturnh 8, from wherever the out leg ends.
// Before the limits were raised: 26 % of out legs and 19 % of home legs
// were "path out of segments", 8 % of out legs "route too long".
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "Maze.h"
#include "Path.h"
#include "Route.h"

static int fails = 0;
static void expect(const char *what, bool ok) {
    std::printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) fails++;
}

static const int N = 16;
static const int DX[4] = {0, 1, 0, -1}, DY[4] = {1, 0, -1, 0};

static void gen(Maze &m, unsigned seed, int loops) {
    srand(seed);
    m.reset();
    for (int x = 0; x < N; x++)
        for (int y = 0; y < N; y++)
            for (int h = 0; h < 4; h++) m.set_wall(x, y, (Heading)h, true);
    std::vector<int> st;
    static bool vis[N][N];
    for (int x = 0; x < N; x++) for (int y = 0; y < N; y++) vis[x][y] = false;
    st.push_back(0);
    vis[0][0] = true;
    while (!st.empty()) {
        int c = st.back(), x = c % N, y = c / N, opts[4], n = 0;
        for (int h = 0; h < 4; h++) {
            int nx = x + DX[h], ny = y + DY[h];
            if (nx >= 0 && ny >= 0 && nx < N && ny < N && !vis[nx][ny]) opts[n++] = h;
        }
        if (!n) { st.pop_back(); continue; }
        int h = opts[rand() % n], nx = x + DX[h], ny = y + DY[h];
        m.set_wall(x, y, (Heading)h, false);
        vis[nx][ny] = true;
        st.push_back(ny * N + nx);
    }
    m.set_wall(7, 7, HEADING_E, false); m.set_wall(7, 8, HEADING_E, false);
    m.set_wall(7, 7, HEADING_N, false); m.set_wall(8, 7, HEADING_N, false);
    for (int i = 0; i < loops; i++)
        m.set_wall(rand() % (N - 1), rand() % (N - 1), (Heading)(rand() % 2), false);
    m.set_wall(0, 0, HEADING_E, true);
    m.set_wall(0, 0, HEADING_N, false);
    Maze::State s;
    m.save_state(s);
    for (int i = 0; i <= MAZE_SIZE; i++) { s.vknown[i] = 0xFFFF; s.hknown[i] = 0xFFFF; }
    m.load_state(s);
    m.set_goal(7, 7, 8, 8);
}

int main() {
    std::printf("[300 random 16x16 mazes, both legs]\n");
    static SpeedPath p, ph;
    static RouteTurn t[ROUTE_MAX_TURNS];
    int out_ok = 0, home_ok = 0, n = 0, max_turns_h = 0, max_segs_h = 0;
    for (unsigned seed = 1; seed <= 300; seed++) {
        Maze m;
        gen(m, seed, seed % 3 == 0 ? 0 : 25);
        route_set_corner_context(4000.0f, 71.74f);
        route_set_turn_cost(3.0f);
        route_set_diag(true);
        route_set_v_out(10.0f);
        m.flood_to_goal(false);
        route_set_time_context(1600.0f, 4000, 4000, 4000, true, 7000);
        RouteStats st;
        RouteResult r = route_build(m, 0, 0, HEADING_N, p, 1600.0f, 7000.0f, &st, t,
                                    ROUTE_MAX_TURNS, 39.0f);
        n++;
        if (r != ROUTE_OK) { std::printf("    seed %u OUT %s\n", seed, route_result_name(r)); continue; }
        out_ok++;
        int ex, ey; Heading eh;
        if (!route_walk_end(m, 0, 0, HEADING_N, &ex, &ey, &eh)) continue;
        route_set_diag(false);
        route_set_turn_cost(8.0f);
        route_set_time_context(400.0f, 4000, 4000, 4000, true, 7000);
        Heading want;
        if (!route_home_heading(m, ex, ey, eh, want)) continue;
        m.flood_to_start(false);
        RouteStats sh;
        RouteResult rh = route_build(m, ex, ey, want, ph, 400.0f, 7000.0f, &sh, nullptr, 0, 39.0f);
        if (rh != ROUTE_OK) { std::printf("    seed %u HOME %s\n", seed, route_result_name(rh)); continue; }
        home_ok++;
        if (sh.turns > max_turns_h) max_turns_h = sh.turns;
        if (ph.seg_count() > max_segs_h) max_segs_h = ph.seg_count();
    }
    std::printf("    out %d / %d, home %d / %d; worst home leg %d turns, %d of %d segments\n",
                out_ok, n, home_ok, n, max_turns_h, max_segs_h, PATH_MAX_SEGS);
    expect("every out leg compiles", out_ok == n);
    expect("every home leg compiles, orthogonal", home_ok == n);
    route_set_turn_cost(0.0f);
    route_set_diag(true);
    route_set_v_out(0.0f);
    route_clear_time_context();
    std::printf(fails ? "big_maze: %d FAILED\n" : "big_maze: all passed\n", fails);
    return fails ? 1 : 0;
}
