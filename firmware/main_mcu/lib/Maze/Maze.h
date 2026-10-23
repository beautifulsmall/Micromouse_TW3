// Maze model: wall map, flood fill, and next-step policy.
//
// Pure logic (no HAL, no floats, no heap) -- host-tested in test_maze.cpp.
// The search executor (main.cpp / Application) owns WHEN to sense, update,
// and move; this module owns WHAT is known and WHERE to go next.
//
// Frame (spec 3.2): 16x16 cells, (0,0) at the bottom-left START, x east,
// y north. The mouse starts at (0,0) facing NORTH; the goal is the 2x2
// block (7..8, 7..8). Headings are absolute compass directions in this
// frame, NOT body-relative.
//
// Wall storage is by EDGE, not by cell, so the two cells sharing a wall can
// never disagree: vertical boundaries vwall[x] (x = 0..16) hold bit y;
// horizontal boundaries hwall[y] (y = 0..16) hold bit x. Each edge also has
// a KNOWN bit -- three states per edge: unknown / known-open / known-wall.
// The outer boundary and the start cell's EAST wall (standard rules: the
// start cell opens only straight ahead) are preset known at reset().
//
// Flood fill is a BFS over passable edges producing per-cell distance to a
// target set (goal block or start cell):
//   OPTIMISTIC  -- unknown edges count as OPEN. Used during search: the
//                  distance is a lower bound, and re-flooding as walls are
//                  discovered is the classic flood-fill search.
//   PESSIMISTIC -- unknown edges count as WALLS. Only fully-known paths
//                  remain: run this to_goal after search; if the distance at
//                  the start is finite, a guaranteed route exists for the
//                  speed run (and search can stop -- see dist_start()).
//
// best_step() picks the neighbouring cell that minimises the flood distance,
// preferring (on ties) straight > left > right > back -- in-place turns cost
// time and the back-turn costs most.

#pragma once

#include <stdint.h>

constexpr int MAZE_SIZE = 16;
constexpr uint8_t MAZE_DIST_UNREACHABLE = 255;

// Default goal block (spec 3.2): cells (7..8, 7..8) inclusive. The active
// goal is runtime-configurable (set_goal) so practice rigs with the goal
// elsewhere work without a reflash; it survives reset() -- a new search
// clears walls, not the mission definition.
constexpr int MAZE_GOAL_MIN = 7;
constexpr int MAZE_GOAL_MAX = 8;

enum Heading : uint8_t { HEADING_N = 0, HEADING_E = 1, HEADING_S = 2, HEADING_W = 3 };

inline Heading heading_left(Heading h)  { return Heading((h + 3) & 3); }
inline Heading heading_right(Heading h) { return Heading((h + 1) & 3); }
inline Heading heading_back(Heading h)  { return Heading((h + 2) & 3); }

// Unit step for a heading (dx, dy in the maze frame).
inline int heading_dx(Heading h) { return h == HEADING_E ? 1 : (h == HEADING_W ? -1 : 0); }
inline int heading_dy(Heading h) { return h == HEADING_N ? 1 : (h == HEADING_S ? -1 : 0); }

class Maze {
public:
    Maze() { reset(); }

    // Clears every edge to unknown, then presets the always-true walls as
    // KNOWN: the outer boundary, and the start cell's EAST wall.
    void reset();

    // Record one sensed wall (or confirmed opening) on side h of cell
    // (x, y). Marks the edge known; the neighbouring cell sees it too.
    // Out-of-range cells are ignored. LOCKED edges (traversed / boundary /
    // rules-preset) ignore the call entirely: the angled side beams can
    // misread near posts, but wheels and rules do not (hardware
    // 2026-07-10: a stationary re-sense wrote a wall onto an edge the
    // mouse had driven through twice, flood-blocking the mission).
    void set_wall(int x, int y, Heading h, bool present);

    // The mouse physically drove through this edge: known-open + LOCKED.
    void set_wall_traversed(int x, int y, Heading h);

    bool wall_known(int x, int y, Heading h) const;
    // True only if the edge is KNOWN and a wall is present (unknown reads
    // false -- pair with wall_known() when the distinction matters).
    bool wall_present(int x, int y, Heading h) const;

    // Full model state, for FRAM persistence across a power cycle.
    //
    // This is NOT walls_pack(): that flattens to known-and-present for an
    // offline reader, which throws away the difference between "known open"
    // and "not looked at yet", and throws away the LOCK bits entirely. A map
    // restored from that would let a sensor read retract an edge the mouse
    // had physically driven through -- the 2026-07-10 failure, where a
    // stationary re-sense wrote a wall onto a traversed edge and
    // flood-blocked the mission. A restored map must be exactly as trusted
    // as the one that was saved, because a SPEED RUN is about to be flown on
    // it, so every bit that carries trust is saved.
    //
    // The goal rectangle is deliberately NOT in here: it has its own FRAM
    // block already and survives reset(), so the mission definition and the
    // wall map stay independently replaceable.
    struct State {
        uint16_t vwall[MAZE_SIZE + 1], vknown[MAZE_SIZE + 1],
                 vlock[MAZE_SIZE + 1];
        uint16_t hwall[MAZE_SIZE + 1], hknown[MAZE_SIZE + 1],
                 hlock[MAZE_SIZE + 1];
        uint16_t visited[MAZE_SIZE];
    };
    void save_state(State &out) const;
    // Restores walls/known/lock/visited. The distance field is NOT restored:
    // it is derived, and leaving a stale flood in place would be a quiet way
    // to walk a route the current goal never asked for. It is cleared, so
    // the caller must flood before stepping.
    void load_state(const State &in);

    // Serialize KNOWN-AND-PRESENT walls (for the flight recorder's map
    // preamble -- offline tools then have the exact map, no inference).
    // v[x] bit y = wall on the vertical line x=x* (west face of cell x);
    // h[y] bit x = wall on the horizontal line y=y* (south face of cell y).
    void walls_pack(uint16_t v[MAZE_SIZE + 1], uint16_t h[MAZE_SIZE + 1]) const;

    // Raycast for the beam observer: from world origin (x0,y0) mm along
    // unit direction (dx,dy), return the distance to the first KNOWN-
    // PRESENT wall within max_mm, or -1 if none. axis_out = 0 if the hit
    // wall is VERTICAL (x-normal), 1 if HORIZONTAL (y-normal). World frame:
    // +x = east, +y = north, walls on the 180 mm grid.
    float raycast(float x0, float y0, float dx, float dy, float max_mm,
                  int &axis_out) const;

    // Visited bookkeeping (cells whose walls have all been sensed).
    void set_visited(int x, int y);
    bool visited(int x, int y) const;

    // Set the goal rectangle (inclusive, normalised, clamped to the grid).
    // A single-cell goal is x0==x1, y0==y1. NOT cleared by reset().
    void set_goal(int x0, int y0, int x1, int y1);

    bool in_goal(int x, int y) const {
        return x >= goal_x0_ && x <= goal_x1_ && y >= goal_y0_ &&
               y <= goal_y1_;
    }
    int goal_x0() const { return goal_x0_; }
    int goal_y0() const { return goal_y0_; }
    int goal_x1() const { return goal_x1_; }
    int goal_y1() const { return goal_y1_; }

    // Flood fill (see header comment). Recomputes the distance field.
    void flood_to_goal(bool optimistic);
    void flood_to_start(bool optimistic);

    // -- Full exploration ----------------------------------------------------
    // A goal search stops the moment a route is proven, which typically
    // leaves a third of the maze never looked at. A fast run over a route the
    // mouse discovered itself wants the whole picture, and so does any
    // offline analysis that compares predicted sensor data against live.

    // True if any of this cell's four edges is still unknown. Note an edge is
    // SHARED: sensing it from either side resolves it for both, so a cell all
    // of whose neighbours have been visited can be fully known without ever
    // being entered. Asking about walls rather than about visits is what lets
    // the search skip those instead of driving to them for nothing.
    bool has_unknown_wall(int x, int y) const;

    // Flood seeded from EVERY cell that still has an unknown edge, so
    // distance() falls toward the nearest unexplored place and best_step()
    // walks there with no extra machinery. Optimistic, necessarily: an
    // unknown edge has to be assumed passable or the frontier can never be
    // reached through one.
    void flood_to_unknown(bool optimistic);

    // Is there anything left to explore that can actually be got to from
    // (x,y)? Walls sealed behind a boundary -- or outside an enclosed
    // sub-maze, which is the 6x6 rig -- stay unknown forever, and a search
    // that waited for them would never terminate. This asks the question
    // that CAN be answered: unknown AND reachable.
    bool unknown_reachable(int x, int y, bool optimistic);

    // EXPLORE WHAT COULD STILL MATTER (2026-10-02; bert: "explore all should
    // be default on, there are often multiple routes to the goal. Known dead
    // ends do not need to be searched"). Flood seeded only from cells that
    // have an unknown edge AND could still lie on a start->goal route no
    // longer than the best KNOWN route plus `slack` cells: optimistic
    // start->cell + cell->goal <= known + slack. A dead-end pocket, a
    // sealed region, or anything too far off every candidate route is not a
    // seed, so the search never drives into it. With no known route yet the
    // bound is open (everything reachable counts). Returns the number of
    // seed cells (0 = every route worth having is proven); *known_out gets
    // the best known route's length in cells (MAZE_DIST_UNREACHABLE if none).
    int flood_to_useful_unknown(int slack, int *known_out = nullptr);
    bool useful_unknown_reachable(int x, int y, int slack);

    // Distance of a cell from the last flood's target set (0 inside it).
    // MAZE_DIST_UNREACHABLE if no passable path existed.
    uint8_t distance(int x, int y) const;

    // Absolute heading of the best next step from (x, y) arriving with
    // heading h: the passable neighbour with the lowest flood distance,
    // ties broken straight > left > right > back. False if no passable
    // neighbour improves on the current cell (boxed in / unreachable).
    bool best_step(int x, int y, Heading h, Heading &out) const;

    // Same, but steps only through KNOWN-OPEN edges (pessimistic): pair
    // with a pessimistic flood to walk a PROVEN route -- the speed run
    // must never gamble on an unknown edge.
    bool best_step_known(int x, int y, Heading h, Heading &out) const;

private:
    // Edge passability under a flood policy.
    bool passable(int x, int y, Heading h, bool optimistic) const;
    // BFS from every cell of an inclusive seed rectangle.
    void flood(int x0, int y0, int x1, int y1, bool optimistic);
    // BFS from every cell already marked distance 0.
    void flood_marked(bool optimistic);

    int goal_x0_ = MAZE_GOAL_MIN, goal_y0_ = MAZE_GOAL_MIN;
    int goal_x1_ = MAZE_GOAL_MAX, goal_y1_ = MAZE_GOAL_MAX;

    // Edge arrays (see header comment). vwall_[x] bit y; hwall_[y] bit x.
    uint16_t vwall_[MAZE_SIZE + 1];
    uint16_t vknown_[MAZE_SIZE + 1];
    uint16_t vlock_[MAZE_SIZE + 1];
    uint16_t hwall_[MAZE_SIZE + 1];
    uint16_t hknown_[MAZE_SIZE + 1];
    uint16_t hlock_[MAZE_SIZE + 1];
    uint16_t visited_[MAZE_SIZE];  // [y] bit x

    uint8_t dist_[MAZE_SIZE][MAZE_SIZE];  // [y][x]
};
