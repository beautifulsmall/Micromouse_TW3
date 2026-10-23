#include "Maze.h"
#include <string.h>

// Map (cell, side) -> edge array slot. N/S sides live in the horizontal
// arrays (boundary index y+1 / y, bit x); E/W in the vertical (boundary
// x+1 / x, bit y). Callers guarantee 0 <= x, y < MAZE_SIZE.
static inline void edge_slot(int x, int y, Heading h, bool &vertical,
                             int &index, int &bit) {
    switch (h) {
        case HEADING_N: vertical = false; index = y + 1; bit = x; break;
        case HEADING_S: vertical = false; index = y;     bit = x; break;
        case HEADING_E: vertical = true;  index = x + 1; bit = y; break;
        default:        vertical = true;  index = x;     bit = y; break;  // W
    }
}

static inline bool in_range(int x, int y) {
    return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

void Maze::reset() {
    for (int i = 0; i <= MAZE_SIZE; i++) {
        vwall_[i] = vknown_[i] = vlock_[i] = 0;
        hwall_[i] = hknown_[i] = hlock_[i] = 0;
    }
    for (int y = 0; y < MAZE_SIZE; y++) visited_[y] = 0;
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++) dist_[y][x] = MAZE_DIST_UNREACHABLE;

    // Outer boundary: known walls all round, LOCKED (no sensor read may
    // ever claim an opening in the outer wall).
    vwall_[0] = vwall_[MAZE_SIZE] = 0xFFFF;
    vknown_[0] = vknown_[MAZE_SIZE] = 0xFFFF;
    vlock_[0] = vlock_[MAZE_SIZE] = 0xFFFF;
    hwall_[0] = hwall_[MAZE_SIZE] = 0xFFFF;
    hknown_[0] = hknown_[MAZE_SIZE] = 0xFFFF;
    hlock_[0] = hlock_[MAZE_SIZE] = 0xFFFF;

    // Standard rules: the start cell's only opening is straight ahead
    // (north) -- its EAST wall is guaranteed present. Locked: rules (and
    // this rig) guarantee it, so sensing may not retract it.
    set_wall(0, 0, HEADING_E, true);
    vlock_[1] |= 1u << 0;  // (0,0) E edge = vertical boundary 1, bit y=0
}

void Maze::save_state(State &out) const {
    for (int i = 0; i <= MAZE_SIZE; i++) {
        out.vwall[i] = vwall_[i];
        out.vknown[i] = vknown_[i];
        out.vlock[i] = vlock_[i];
        out.hwall[i] = hwall_[i];
        out.hknown[i] = hknown_[i];
        out.hlock[i] = hlock_[i];
    }
    for (int y = 0; y < MAZE_SIZE; y++) out.visited[y] = visited_[y];
}

void Maze::load_state(const State &in) {
    for (int i = 0; i <= MAZE_SIZE; i++) {
        vwall_[i] = in.vwall[i];
        vknown_[i] = in.vknown[i];
        vlock_[i] = in.vlock[i];
        hwall_[i] = in.hwall[i];
        hknown_[i] = in.hknown[i];
        hlock_[i] = in.hlock[i];
    }
    for (int y = 0; y < MAZE_SIZE; y++) visited_[y] = in.visited[y];
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            dist_[y][x] = MAZE_DIST_UNREACHABLE;

    // Re-assert the invariants reset() establishes, rather than trusting the
    // stored bytes to carry them. A block written by an older firmware, or
    // corrupted in a way the CRC happens to miss, must not be able to open
    // the outer boundary -- that is the one edge set a sensor is never
    // allowed to argue with, and it costs six ORs to make certain.
    vwall_[0] = vwall_[MAZE_SIZE] = 0xFFFF;
    vknown_[0] = vknown_[MAZE_SIZE] = 0xFFFF;
    vlock_[0] = vlock_[MAZE_SIZE] = 0xFFFF;
    hwall_[0] = hwall_[MAZE_SIZE] = 0xFFFF;
    hknown_[0] = hknown_[MAZE_SIZE] = 0xFFFF;
    hlock_[0] = hlock_[MAZE_SIZE] = 0xFFFF;
}

void Maze::set_wall(int x, int y, Heading h, bool present) {
    if (!in_range(x, y)) return;
    bool vertical;
    int index, bit;
    edge_slot(x, y, h, vertical, index, bit);
    uint16_t mask = uint16_t(1u << bit);
    if ((vertical ? vlock_[index] : hlock_[index]) & mask) return;  // locked
    uint16_t *wall = vertical ? &vwall_[index] : &hwall_[index];
    uint16_t *known = vertical ? &vknown_[index] : &hknown_[index];
    if (present) *wall |= mask;
    else *wall &= ~mask;
    *known |= mask;
}

void Maze::set_wall_traversed(int x, int y, Heading h) {
    if (!in_range(x, y)) return;
    bool vertical;
    int index, bit;
    edge_slot(x, y, h, vertical, index, bit);
    uint16_t mask = uint16_t(1u << bit);
    if ((vertical ? vlock_[index] : hlock_[index]) & mask) return;  // already
    uint16_t *wall = vertical ? &vwall_[index] : &hwall_[index];
    uint16_t *known = vertical ? &vknown_[index] : &hknown_[index];
    uint16_t *lock = vertical ? &vlock_[index] : &hlock_[index];
    *wall &= ~mask;  // open
    *known |= mask;
    *lock |= mask;
}

bool Maze::wall_known(int x, int y, Heading h) const {
    if (!in_range(x, y)) return true;  // outside = as good as walled
    bool vertical;
    int index, bit;
    edge_slot(x, y, h, vertical, index, bit);
    return ((vertical ? vknown_[index] : hknown_[index]) >> bit) & 1u;
}

bool Maze::wall_present(int x, int y, Heading h) const {
    if (!in_range(x, y)) return true;
    bool vertical;
    int index, bit;
    edge_slot(x, y, h, vertical, index, bit);
    bool known = ((vertical ? vknown_[index] : hknown_[index]) >> bit) & 1u;
    bool wall = ((vertical ? vwall_[index] : hwall_[index]) >> bit) & 1u;
    return known && wall;
}

void Maze::walls_pack(uint16_t v[MAZE_SIZE + 1],
                      uint16_t h[MAZE_SIZE + 1]) const {
    // Known-and-present only (an unknown edge is not a wall for the map).
    for (int i = 0; i <= MAZE_SIZE; i++) {
        v[i] = vwall_[i] & vknown_[i];
        h[i] = hwall_[i] & hknown_[i];
    }
}

// Walls are 12 mm thick, centred on the 180 mm grid line; the sensor
// reads the SURFACE, 6 mm nearer than the line on the approaching side
// (bert, 2026-08-04). The effective plane is line - HALF*sign(approach),
// so a wall crossed from the west presents its face at line-6 and from
// the east at line+6 -- 12 mm apart, the two surfaces of one wall.
static constexpr float WALL_HALF_MM = 6.0f;

float Maze::raycast(float x0, float y0, float dx, float dy, float max_mm,
                    int &axis_out) const {
    const float C = 180.0f;
    float best = max_mm;
    axis_out = -1;
    if (dx > 1e-4f || dx < -1e-4f) {
        float ofs = (dx > 0.0f) ? -WALL_HALF_MM : WALL_HALF_MM;
        for (int gx = 0; gx <= MAZE_SIZE; gx++) {
            float t = (gx * C + ofs - x0) / dx;   // to the near surface
            if (t <= 0.0f || t >= best) continue;
            int row = (int)((y0 + t * dy) / C);
            if (row < 0 || row >= MAZE_SIZE) continue;
            if (((vwall_[gx] & vknown_[gx]) >> row) & 1u) {
                best = t; axis_out = 0;
            }
        }
    }
    if (dy > 1e-4f || dy < -1e-4f) {
        float ofs = (dy > 0.0f) ? -WALL_HALF_MM : WALL_HALF_MM;
        for (int gy = 0; gy <= MAZE_SIZE; gy++) {
            float t = (gy * C + ofs - y0) / dy;
            if (t <= 0.0f || t >= best) continue;
            int col = (int)((x0 + t * dx) / C);
            if (col < 0 || col >= MAZE_SIZE) continue;
            if (((hwall_[gy] & hknown_[gy]) >> col) & 1u) {
                best = t; axis_out = 1;
            }
        }
    }
    return axis_out < 0 ? -1.0f : best;
}

void Maze::set_visited(int x, int y) {
    if (in_range(x, y)) visited_[y] |= uint16_t(1u << x);
}

bool Maze::visited(int x, int y) const {
    return in_range(x, y) && ((visited_[y] >> x) & 1u);
}

bool Maze::passable(int x, int y, Heading h, bool optimistic) const {
    bool vertical;
    int index, bit;
    edge_slot(x, y, h, vertical, index, bit);
    bool known = ((vertical ? vknown_[index] : hknown_[index]) >> bit) & 1u;
    bool wall = ((vertical ? vwall_[index] : hwall_[index]) >> bit) & 1u;
    if (known) return !wall;
    return optimistic;  // unknown: open when optimistic, wall when not
}

void Maze::set_goal(int x0, int y0, int x1, int y1) {
    if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= MAZE_SIZE) x1 = MAZE_SIZE - 1;
    if (y1 >= MAZE_SIZE) y1 = MAZE_SIZE - 1;
    goal_x0_ = x0;
    goal_y0_ = y0;
    goal_x1_ = x1;
    goal_y1_ = y1;
}

// BFS outward from every cell already marked 0 in dist_. Split out of
// flood() so the goal/start seeding and the frontier seeding cannot drift
// apart -- they are the same search, differing only in what is a seed.
void Maze::flood_marked(bool optimistic) {
    // BFS ring buffer: 256 cells max, each enqueued at most once.
    uint8_t qx[MAZE_SIZE * MAZE_SIZE];
    uint8_t qy[MAZE_SIZE * MAZE_SIZE];
    int head = 0, tail = 0;

    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            if (dist_[y][x] == 0) {
                qx[tail] = uint8_t(x);
                qy[tail] = uint8_t(y);
                tail++;
            }

    while (head < tail) {
        int x = qx[head], y = qy[head];
        head++;
        uint8_t next = uint8_t(dist_[y][x] + 1);
        for (int h = 0; h < 4; h++) {
            if (!passable(x, y, Heading(h), optimistic)) continue;
            int nx = x + heading_dx(Heading(h));
            int ny = y + heading_dy(Heading(h));
            // Boundary edges are always known walls, so nx/ny stay in range.
            if (dist_[ny][nx] != MAZE_DIST_UNREACHABLE) continue;
            dist_[ny][nx] = next;
            qx[tail] = uint8_t(nx);
            qy[tail] = uint8_t(ny);
            tail++;
        }
    }
}

void Maze::flood(int x0, int y0, int x1, int y1, bool optimistic) {
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++) dist_[y][x] = MAZE_DIST_UNREACHABLE;
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) dist_[y][x] = 0;
    flood_marked(optimistic);
}

bool Maze::has_unknown_wall(int x, int y) const {
    if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE) return false;
    for (int h = 0; h < 4; h++)
        if (!wall_known(x, y, Heading(h))) return true;
    return false;
}

void Maze::flood_to_unknown(bool optimistic) {
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            dist_[y][x] = has_unknown_wall(x, y) ? 0 : MAZE_DIST_UNREACHABLE;
    flood_marked(optimistic);
}

bool Maze::unknown_reachable(int x, int y, bool optimistic) {
    if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE) return false;
    flood_to_unknown(optimistic);
    return dist_[y][x] != MAZE_DIST_UNREACHABLE;
}

int Maze::flood_to_useful_unknown(int slack, int *known_out) {
    // The best route the map PROVES (known-open edges only).
    flood(goal_x0_, goal_y0_, goal_x1_, goal_y1_, false);
    const uint8_t known = dist_[0][0];
    if (known_out) *known_out = known;
    // Optimistic distances to the goal and from the start.
    static uint8_t dg[MAZE_SIZE][MAZE_SIZE], ds[MAZE_SIZE][MAZE_SIZE];
    flood(goal_x0_, goal_y0_, goal_x1_, goal_y1_, true);
    memcpy(dg, dist_, sizeof dg);
    flood(0, 0, 0, 0, true);
    memcpy(ds, dist_, sizeof ds);
    const int bound = known == MAZE_DIST_UNREACHABLE ? 100000 : (int)known + slack;
    int seeds = 0;
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++) {
            bool seed = has_unknown_wall(x, y) && ds[y][x] != MAZE_DIST_UNREACHABLE &&
                        dg[y][x] != MAZE_DIST_UNREACHABLE &&
                        (int)ds[y][x] + (int)dg[y][x] <= bound;
            dist_[y][x] = seed ? 0 : MAZE_DIST_UNREACHABLE;
            if (seed) seeds++;
        }
    flood_marked(true);
    return seeds;
}

bool Maze::useful_unknown_reachable(int x, int y, int slack) {
    if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE) return false;
    if (flood_to_useful_unknown(slack) == 0) return false;
    return dist_[y][x] != MAZE_DIST_UNREACHABLE;
}

void Maze::flood_to_goal(bool optimistic) {
    flood(goal_x0_, goal_y0_, goal_x1_, goal_y1_, optimistic);
}

void Maze::flood_to_start(bool optimistic) {
    flood(0, 0, 0, 0, optimistic);
}

uint8_t Maze::distance(int x, int y) const {
    if (!in_range(x, y)) return MAZE_DIST_UNREACHABLE;
    return dist_[y][x];
}

bool Maze::best_step(int x, int y, Heading h, Heading &out) const {
    if (!in_range(x, y)) return false;
    // Tie-break order: straight, left, right, back.
    const Heading order[4] = {h, heading_left(h), heading_right(h),
                              heading_back(h)};
    uint8_t best = dist_[y][x];  // must IMPROVE on the current cell
    bool found = false;
    for (int i = 0; i < 4; i++) {
        Heading cand = order[i];
        // The mouse can only drive through edges not known-walled; unknown
        // edges are fine during search (optimistic) -- by the time a step
        // is executed the current cell's walls have been sensed anyway.
        if (!passable(x, y, cand, true)) continue;
        int nx = x + heading_dx(cand);
        int ny = y + heading_dy(cand);
        uint8_t d = dist_[ny][nx];
        if (d < best) {  // strictly better; order[] already encodes ties
            best = d;
            out = cand;
            found = true;
        }
    }
    return found;
}

bool Maze::best_step_known(int x, int y, Heading h, Heading &out) const {
    if (!in_range(x, y)) return false;
    const Heading order[4] = {h, heading_left(h), heading_right(h),
                              heading_back(h)};
    uint8_t best = dist_[y][x];
    bool found = false;
    for (int i = 0; i < 4; i++) {
        Heading cand = order[i];
        if (!passable(x, y, cand, false)) continue;  // KNOWN-open only
        int nx = x + heading_dx(cand);
        int ny = y + heading_dy(cand);
        uint8_t d = dist_[ny][nx];
        if (d < best) {
            best = d;
            out = cand;
            found = true;
        }
    }
    return found;
}
