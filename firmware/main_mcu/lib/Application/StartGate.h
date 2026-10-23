// THE START GATE (bert, 2026-09-30): "the power on problem with the laser
// sensors needs better identification: if the side sensors don't both
// provide multiple readings under 255 then toggle the LEDs one then the
// other until both readings are ~ walls, so it is very quickly known if the
// sensors have powered up correctly. Also don't allow a hand start until
// both side sensors are stable on walls -- this way a mouse can be picked up
// after a run to clean wheels before the next start."
//
// The start cell has a wall on BOTH sides, always (the maze rules make it
// so). A mouse sitting in it, sensors alive, reads two walls at ~82-86 mm,
// steady. A sensor that came up wrong after a quick power cycle reads 255
// (the known fault); a mouse in the hand reads 255 on both. Either way the
// wave must not count, and the LEDs must say so at a glance.
//
// READY = every packet for START_GATE_STABLE_MS has both side beams under
// the wall threshold and neither has jumped more than START_GATE_JUMP_MM
// since the packet before, with the stream fresh. One bad packet restarts
// the clock. The armed missions (the route's wave, the search's wave) run
// their packets through this and refuse the hand while it is not ready,
// and show START_GATE_LED_MS alternating LEDs instead of their own pattern.
//
// Header-only and free of the HAL so the host tests can drive it.

#pragma once

#include <stdint.h>

#include "Walls.h"

constexpr uint32_t START_GATE_STABLE_MS = 300;   // both walls, steady, this long
constexpr uint8_t START_GATE_JUMP_MM = 12;       // packet-to-packet, per beam
constexpr uint32_t START_GATE_STALE_MS = 100;    // no packet this long = not ready
constexpr uint32_t START_GATE_LED_MS = 150;      // the alternating LEDs' half period
constexpr uint32_t START_GATE_SAY_MS = 2000;     // not ready this long: say why, once

struct StartGate {
    bool ready = false;
    bool good_run = false;      // the current run of good packets has started
    uint32_t good_t0 = 0;       // ...at this time
    uint8_t prev_l = 255, prev_r = 255;
    uint32_t bad_t0 = 0;        // not ready since
    bool said_bad = false;      // the "why" line, once per not-ready spell
    uint8_t last_l = 255, last_r = 255;

    // Set by the route just before it hands a held wave to the search (the
    // 5 s wipe): the mouse has not moved, so the next reset keeps the state.
    bool carry_next_reset = false;
    // Which sides the start cell has a wall on. Both, by the rules; a
    // practice rig whose start cell opens sideways (D7's row-0 return) says
    // otherwise in its map, and the route sets these from it at arm. A side
    // not needed is not judged; with neither, the gate still needs a live,
    // steady stream.
    bool need_l = true, need_r = true;

    void reset(uint32_t now) {
        if (carry_next_reset) { carry_next_reset = false; return; }
        ready = false;
        good_run = false;
        good_t0 = now;
        prev_l = prev_r = 255;
        bad_t0 = now;
        said_bad = false;
        last_l = last_r = 255;
    }

    // One packet. Returns true when this packet made the gate READY (the
    // not-ready -> ready transition), for the caller to announce.
    bool packet(uint8_t l, uint8_t r, uint32_t now) {
        last_l = l;
        last_r = r;
        bool ok_l = !need_l || (l != 255 && l <= WALL_LEFT_PRESENT_MM);
        bool ok_r = !need_r || (r != 255 && r <= WALL_RIGHT_PRESENT_MM);
        bool walls = ok_l && ok_r;
        bool st_l = !need_l || (prev_l != 255 &&
                                (l > prev_l ? l - prev_l : prev_l - l) <= START_GATE_JUMP_MM);
        bool st_r = !need_r || (prev_r != 255 &&
                                (r > prev_r ? r - prev_r : prev_r - r) <= START_GATE_JUMP_MM);
        bool steady = st_l && st_r;
        prev_l = l;
        prev_r = r;
        if (!(walls && steady)) {
            if (ready) { bad_t0 = now; said_bad = false; }
            ready = false;
            good_run = false;
            return false;
        }
        if (!good_run) { good_run = true; good_t0 = now; }
        if (!ready && now - good_t0 >= START_GATE_STABLE_MS) {
            ready = true;
            return true;
        }
        return false;
    }

    // Every poll: a silent stream is not ready either.
    void tick(uint32_t ms_since_packet, uint32_t now) {
        if (ms_since_packet > START_GATE_STALE_MS) {
            if (ready) { bad_t0 = now; said_bad = false; }
            ready = false;
            good_run = false;
            prev_l = prev_r = 255;
        }
    }

    // True once per not-ready spell, after START_GATE_SAY_MS of it: the
    // caller prints why (the beams' values).
    bool time_to_say(uint32_t now) {
        if (ready || said_bad || now - bad_t0 < START_GATE_SAY_MS) return false;
        said_bad = true;
        return true;
    }

    // The alternating pattern while not ready: LED1 on the even half
    // periods, LED2 on the odd ones.
    static bool led1_phase(uint32_t now) { return ((now / START_GATE_LED_MS) & 1u) == 0; }
};

// One gate for the one start cell: the route's wave and the search's wave
// read the same beams, and the route hands a held wave to the search.
inline StartGate &start_gate_shared() {
    static StartGate g;
    return g;
}
