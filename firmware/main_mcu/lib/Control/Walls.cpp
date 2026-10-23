#include "Walls.h"

#include <math.h>

static constexpr float DEG2RAD_W = 0.017453292519943295f;

// ---- YAW-AWARE HIT GEOMETRY (see Walls.h) -----------------------------------
// Laser mounting, body frame (x fwd, y left), indexed by WIRE/packet channel:
//   0 left, 1 front_right, 2 front_left, 3 right.
// Side beams cross: the right-mounted sensor (oy < 0) reads the LEFT wall.
struct BeamGeom { float ox, oy, adeg; bool side; };
static const BeamGeom BEAM_GEOM[4] = {
    {40.0f, -15.0f,  72.8f, true },   // 0 left  (right-mounted, crosses)
    {28.0f,  -9.0f,   8.4f, false},   // 1 front_right
    {28.0f,   9.0f,  -9.6f, false},   // 2 front_left
    {40.0f,  15.0f, -71.0f, true },   // 3 right (left-mounted, crosses)
};

static uint8_t beam_raw(const WallRanges &r, int ch) {
    switch (ch) {
        case 0: return r.left_mm;
        case 1: return r.front_right_mm;
        case 2: return r.front_left_mm;
        default: return r.right_mm;
    }
}

BeamHit walls_beam_hit(const WallRanges &r, int ch) {
    BeamHit h = {false, 0.0f, 0.0f, 0.0f};
    if (ch < 0 || ch > 3) return h;
    uint8_t d = beam_raw(r, ch);
    if (d == SENSOR_OUT_OF_RANGE || d == 0) return h;
    const BeamGeom &b = BEAM_GEOM[ch];
    float a = b.adeg * DEG2RAD_W;

    // Undo the calibration to recover the true along-beam range. The pixel only
    // ever measured range; the side LUT stores the yaw-0 perpendicular of it,
    // so this inversion is exact and carries no yaw assumption. The FRONT LUTs
    // are already an along-beam range from the sensor face (measured on logs
    // 055/056/059: residual -2.5 mm, and the "forward-from-centre" reading this
    // header once claimed is rejected by ~27 mm), so they pass straight through.
    float rng;
    if (b.side) {
        float sn = fabsf(sinf(a));
        rng = (sn > 1e-3f) ? ((float)d + fabsf(b.oy)) / sn : (float)d;
    } else {
        rng = (float)d;
    }

    h.valid = true;
    h.range_mm = rng;
    h.x_mm = b.ox + rng * cosf(a);
    h.y_mm = b.oy + rng * sinf(a);
    return h;
}

float walls_clearance_mm(const WallRanges &r, uint32_t ms_since_packet) {
    if (ms_since_packet >= WALL_STALE_MS) return WALL_CLEAR_NONE_MM;
    float worst = WALL_CLEAR_NONE_MM;
    // SIDE CHANNELS ONLY, deliberately. The front pair points within ~10 deg of
    // straight ahead, so a front wall at any normal approach distance lands at
    // |y| well inside the half-width and would read as a permanent strike --
    // a guaranteed false positive every time one comes into view. A wall AHEAD
    // is a longitudinal problem (stop/turn in time) owned by the speed profile
    // and the front gate; walls_evaluate().front_mm already reports it. What
    // this function answers is the LATERAL question the side beams are actually
    // positioned to see: will my flank clear that post.
    for (int ch = 0; ch < 4; ch++) {
        if (ch != 0 && ch != 3) continue;          // 0 = left, 3 = right
        BeamHit h = walls_beam_hit(r, ch);
        if (!h.valid) continue;
        // Only points the body has not already drawn level with can still be
        // struck by the flank.
        if (h.x_mm <= WALL_MOUSE_HALF_L_MM) continue;
        float clear = fabsf(h.y_mm) - WALL_MOUSE_HALF_W_MM;
        if (clear < worst) worst = clear;
    }
    return worst;
}

WallView walls_evaluate(const WallRanges &r, uint32_t ms_since_packet) {
    WallView v;
    v.valid = ms_since_packet < WALL_STALE_MS;
    v.left_mm = 0.0f;
    v.right_mm = 0.0f;
    v.lateral_mm = 0.0f;
    v.lateral_valid = false;

    if (!v.valid) {
        v.left = v.right = v.front = false;
        v.front_mm = WALL_FRONT_NONE_MM;
        return v;
    }

    v.left = r.left_mm <= WALL_LEFT_PRESENT_MM;
    v.right = r.right_mm <= WALL_RIGHT_PRESENT_MM;

    bool fl = r.front_left_mm <= WALL_FRONT_PRESENT_MM;
    bool fr = r.front_right_mm <= WALL_FRONT_PRESENT_MM;
    v.front = fl || fr;

    // Forward distance = the closer valid front beam. The front LUTs are now
    // FORWARD-FROM-CENTRE (auto-cal 'C', 45 mm at contact): the sub-MCU LUT
    // bakes in the sensor forward offset and beam angle, so there is no
    // projection to apply here -- the beam value IS x from the mouse centre.
    // The compiled hand defaults are stored in the same convention (see the
    // sub-MCU lut_front_*), so this holds whether FRAM or defaults are live.
    float best = WALL_FRONT_NONE_MM;
    if (fl && (float)r.front_left_mm < best) best = (float)r.front_left_mm;
    if (fr && (float)r.front_right_mm < best) best = (float)r.front_right_mm;
    v.front_mm = best;

    // Side beams -> perpendicular wall distances (calibration in Walls.h) ->
    // lateral offset from the corridor centreline (+ve toward the LEFT wall,
    // body +y). Moving toward the left wall shrinks d_left and grows d_right,
    // so  y = HALF - d_left = d_right - HALF; with both walls the average
    // (d_right - d_left) / 2 needs no corridor-width constant at all.
    // Side LUTs are now PERPENDICULAR-from-centre (auto-cal 'S', 2026-08-08):
    // the sub-MCU LUT maps pixel -> d_perp directly (yaw-0 reference, sensor
    // offset + beam angle baked in), so no K/C conversion here. Compiled hand
    // defaults are converted to the same convention.
    if (v.left) v.left_mm = (float)r.left_mm;
    if (v.right) v.right_mm = (float)r.right_mm;
    if (v.left && v.right) {
        v.lateral_mm = 0.5f * (v.right_mm - v.left_mm);
        v.lateral_valid = true;
    } else if (v.left) {
        v.lateral_mm = WALL_CORRIDOR_HALF_MM - v.left_mm;
        v.lateral_valid = true;
    } else if (v.right) {
        v.lateral_mm = v.right_mm - WALL_CORRIDOR_HALF_MM;
        v.lateral_valid = true;
    }

    return v;
}
