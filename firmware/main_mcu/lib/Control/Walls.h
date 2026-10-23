// Wall interpretation: raw beam ranges -> wall presence, front distance,
// perpendicular side distances, and lateral offset in the corridor.
//
// Sensor geometry layer (maze-mapping oriented): decide whether a left /
// right / front wall is present, how far ahead the front wall is (for
// stopping at a cell boundary), and where the mouse sits laterally between
// the side walls (for wall-following correction). Wall-referenced heading
// (fused with IMU yaw) comes later.
//
// Inputs are the driver's WallRanges (Sensors.h): direct beam ranges in mm,
// 255 = out of range, with the front pair already un-crossed. The sub-MCU only
// emits a range when its own spot-strength threshold is met, so a value < 255
// already means "something detected"; presence here is a range threshold on top
// of that (tunable to reject far/adjacent-cell returns).
//
// Body frame: x = forward (mm), origin at the mouse reference (centre). The
// front LUTs are FORWARD-FROM-CENTRE as of the auto-cal (bert 2026-08-08):
// the sub-MCU LUT maps pixel -> x from the mouse centre directly (45 mm at
// contact), baking in the sensor offset + beam angle, so walls_evaluate
// takes the beam value as x with no projection.
//
// Pure logic (thresholds), no HAL -- host-tested in test_walls.cpp.

#pragma once

#include "Sensors.h"

// Presence thresholds: a beam range (mm) at or below this counts as a wall.
// 255 (out of range) is always "no wall". Tune on hardware from the 'n'
// readout -- a centred corridor wall reads well under these.
// SIDE thresholds 200 -> 165 (bert 2026-08-02, wider lenses + re-cal). These
// only started doing real work with the new optics. The old side LUTs topped
// out around 110-115 mm, so the beam physically could not see past the adjacent
// wall and a 200 mm threshold rejected nothing -- any valid reading WAS the
// adjacent wall. The re-calibrated range is ~40..200 mm, which is far enough to
// see THROUGH a gap to the next wall over, and at 200 the threshold would
// accept that as "wall present" and invent walls that are a cell away.
//
// Geometry fixes the band (corridor half-width 84, mouse half-width 35, so the
// mouse can sit at most 49 mm off centre before it touches):
//     an ADJACENT wall reads      35 .. 133 mm
//     the NEXT wall over reads   215 .. 313 mm
// so anything from ~135 to ~215 separates them. 165 sits mid-band: comfortably
// above a real adjacent wall seen while hugging the other side, comfortably
// below the nearest thing visible through a gap.
constexpr uint8_t WALL_LEFT_PRESENT_MM = 165;
constexpr uint8_t WALL_RIGHT_PRESENT_MM = 165;
// FRONT unchanged: that pair was not re-calibrated and its LUT still spans
// ~48..235 mm, where 200 remains a sensible "is there a wall in this cell"
// cut (a wall one cell ahead sits at ~264 mm).
constexpr uint8_t WALL_FRONT_PRESENT_MM = 200;

// Front sensor geometry (body frame). DEPRECATED for the forward-distance
// path -- the front LUTs now bake this in (see above). Kept for reference /
// the LUT-conversion of the compiled hand defaults (28 + range*cos(angle)).
constexpr float WALL_FRONT_X_MM = 28.0f;         // front lasers' forward offset
constexpr float WALL_FRONT_LEFT_ANGLE_DEG = 8.4f;   // front_left_mm beam (|angle|)
constexpr float WALL_FRONT_RIGHT_ANGLE_DEG = 9.6f;  // front_right_mm beam (|angle|)

// Side-beam calibration. DEPRECATED (2026-08-08): the side LUTs now store
// PERPENDICULAR distance from centre directly (auto-cal 'S', yaw-0 ref), so
// walls_evaluate passes them through -- no K/C. Kept only to convert the
// compiled hand side defaults into the perp convention (d = K*range + C).
constexpr float WALL_SIDE_LEFT_K = 1.0256f;
constexpr float WALL_SIDE_LEFT_C = -28.8f;
constexpr float WALL_SIDE_RIGHT_K = 1.0f;
constexpr float WALL_SIDE_RIGHT_C = -20.5f;

// Corridor half-width, centreline to wall face = half a cell minus the wall
// inset: 90 - 6 = 84 (face-to-face 168 = 180 cell - 2*6). Used for the
// single-wall lateral estimate.
constexpr float WALL_CORRIDOR_HALF_MM = 84.0f;

// Sensor data older than this is treated as stale (sub-MCU silent / link down).
constexpr uint32_t WALL_STALE_MS = 100;

// Returned when no front wall is seen (effectively "far").
constexpr float WALL_FRONT_NONE_MM = 9999.0f;

struct WallView {
    bool left;        // left wall present
    bool right;       // right wall present
    bool front;       // front wall present
    float front_mm;   // forward distance to the front wall, or WALL_FRONT_NONE_MM
    float left_mm;    // perpendicular distance to the left wall face (if left)
    float right_mm;   // perpendicular distance to the right wall face (if right)
    float lateral_mm; // mouse offset from corridor centre, +ve toward the LEFT
                      // wall (body +y). From both walls when available (average,
                      // immune to corridor-width error), else the seen wall +
                      // WALL_CORRIDOR_HALF_MM. 0 when lateral_valid is false.
    bool lateral_valid;  // at least one side wall seen (and data fresh)
    bool valid;       // sensor data fresh (see WALL_STALE_MS)
};

// Evaluate wall presence + front distance from the latest ranges. ms_since is
// sensors_ms_since_packet(); when stale, valid = false and flags read false.
WallView walls_evaluate(const WallRanges &r, uint32_t ms_since_packet);

// ---- YAW-AWARE HIT GEOMETRY (bert 2026-07-30) -------------------------------
// walls_evaluate's side outputs are only meaningful with the mouse SQUARE in
// the corridor: the side LUTs store perpendicular-from-centre measured at a
// YAW-0 reference, and nothing de-rotated them (the "wall-referenced heading"
// pass at the top of this header was never built). Off-axis -- exactly where
// diagonals and zig-zags live -- that number is neither a perpendicular nor a
// range, and measured ~36 mm out against the map.
//
// The fix is exact, not an approximation. The sensor pixel only ever measures
// the ALONG-BEAM RANGE r; the LUT stores f(r), where f is the yaw-0
// perpendicular conversion. So f can be inverted with no loss and no yaw
// assumption at all:
//
//     r = (d + |oy|) / |sin a|          <- undoes the calibration exactly
//
// and from there the hit point follows from the IMU yaw. Everything below is
// built on that one inversion.
//
// Body frame: x forward, y LEFT, origin at the mouse reference point.
struct BeamHit {
    bool  valid;      // beam returned a usable range
    float x_mm;       // hit point, body frame, +x ahead
    float y_mm;       // hit point, body frame, +y left
    float range_mm;   // true along-beam range from the sensor face
};

// Convert one channel's reading to a body-frame hit point. `ch` is the packet/
// wire index: 0 left, 1 front_right, 2 front_left, 3 right. Needs no pose and
// no map -- purely the mouse's own geometry -- so it is valid at any yaw and
// on any path. 255/0 give valid=false.
BeamHit walls_beam_hit(const WallRanges &r, int ch);

// ---- CLEARANCE (the collision guard's input) --------------------------------
// Half-width of the mouse body (90 x 70 mm footprint, spec 3.1 / table 255).
constexpr float WALL_MOUSE_HALF_W_MM = 35.0f;
// Half-length, for deciding whether a hit is still ahead of the mouse.
constexpr float WALL_MOUSE_HALF_L_MM = 45.0f;

// Worst predicted BODY-EDGE clearance over all four beams: for each hit that is
// still ahead of the mouse, how much lateral room the body edge would have when
// it reaches that point (|y| - half width). Negative means the body sweeps
// through the detected point -- a strike, unless something changes.
//
// This is deliberately geometric and pose-free: no map, no path, no odometry,
// so it cannot be fooled by an accumulated position error. It only answers
// "is something in the way of where I am pointing right now".
//
// Returns WALL_CLEAR_NONE_MM when nothing relevant is in view.
constexpr float WALL_CLEAR_NONE_MM = 9999.0f;
float walls_clearance_mm(const WallRanges &r, uint32_t ms_since_packet);
