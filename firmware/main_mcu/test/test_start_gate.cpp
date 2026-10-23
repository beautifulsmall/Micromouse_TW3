// The start gate (StartGate.h): both side walls steady for 300 ms before a
// wave counts; LEDs alternate until then.
//
//   ./run_tests.sh start_gate

#include "StartGate.h"

#include <cstdio>

static int failures = 0;
static void expect(const char *what, bool ok) {
    if (!ok) { std::printf("FAIL: %s\n", what); failures++; }
}

// Feed packets every 2 ms from t0 to t1 with fixed readings.
static uint32_t feed(StartGate &g, uint8_t l, uint8_t r, uint32_t t0, uint32_t t1, bool *became = nullptr) {
    for (uint32_t t = t0; t < t1; t += 2) {
        bool b = g.packet(l, r, t);
        g.tick(1, t);
        if (b && became) *became = true;
    }
    return t1;
}

int main() {
    StartGate g;
    g.reset(0);
    // On the floor in the start cell: 82 / 85, steady.
    bool became = false;
    feed(g, 82, 85, 0, 290, &became);
    expect("not ready before 300 ms", !g.ready && !became);
    feed(g, 82, 85, 290, 320, &became);
    expect("ready after 300 ms of both walls", g.ready && became);

    // Picked up: both 255.
    feed(g, 255, 255, 320, 330);
    expect("picked up = not ready", !g.ready);
    expect("the why line waits 2 s", !g.time_to_say(1000));
    expect("the why line after 2 s", g.time_to_say(2400));
    expect("...once", !g.time_to_say(3000));

    // The power-on fault: one side stuck at 255.
    g.reset(4000);
    feed(g, 82, 255, 4000, 5000);
    expect("one side 255 = never ready", !g.ready);

    // A jump restarts the clock (a hand swept past a side beam).
    g.reset(6000);
    feed(g, 82, 85, 6000, 6200);
    g.packet(60, 85, 6200);
    feed(g, 82, 85, 6202, 6400);
    expect("a 22 mm jump restarts the 300 ms", !g.ready);
    feed(g, 82, 85, 6400, 6600);
    expect("...and 300 ms after it, ready", g.ready);

    // A silent stream is not ready.
    g.tick(500, 7000);
    expect("stale stream = not ready", !g.ready);

    // A start cell open on the right (a practice rig): only the left judged.
    g.reset(8000);
    g.need_r = false;
    feed(g, 82, 255, 8000, 8400);
    expect("need_r false: right 255 is fine", g.ready);
    g.need_r = true;

    // The route hands a held wave to the search: state carries.
    g.reset(9000);
    feed(g, 82, 85, 9000, 9400);
    expect("ready before the handoff", g.ready);
    g.carry_next_reset = true;
    g.reset(9400);
    expect("carry keeps it ready", g.ready && !g.carry_next_reset);

    // The LEDs alternate every 150 ms.
    expect("LED phase alternates", StartGate::led1_phase(0) && !StartGate::led1_phase(150) &&
                                   StartGate::led1_phase(300));

    if (failures) { std::printf("%d failure(s)\n", failures); return 1; }
    std::printf("test_start_gate: all passed\n");
    return 0;
}
