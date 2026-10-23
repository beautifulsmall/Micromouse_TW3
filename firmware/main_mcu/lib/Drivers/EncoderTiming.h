// EDGE-TIMED WHEEL VELOCITY (bert's XBAR + DMA capture, 2026-09-25).
//
// The wheel loop's velocity has been counts-per-tick: 0.2058 mm per count
// at 500 Hz is a 103 mm/s LSB, softened by the vfilt IIR into ~35 mm/s of
// noise and 10 ms of lag. That number is under this week's findings: the
// P gain has to be soft or the loop buzzes on the staircase (the kpt
// saga), the ABS needed a 250 mm/s band floor, the datum press creeps
// blind at 12 mm/s (one count every 17 ms), and an in-place turn at 300
// deg/s is under two counts a tick.
//
// This measures the TIME BETWEEN ENCODER EDGES instead. The four encoder
// signals already pass through the XBAR on their way to the ENC modules;
// the same XBAR inputs are also connected to its four DMA-capable outputs,
// each rising edge raises a DMA request, and the DMA copies the free-
// running GPT1 counter (24 MHz / 3 = 8 MHz, 0.125 us) into a ring buffer
// per phase -- no interrupts, no CPU in the path. Each control tick reads
// what arrived: velocity = edges x mm_per_edge / (time from the last edge
// of the previous window to the last edge of this one), the M/T method,
// with the timer's resolution rather than the loop's -- 0.04 % at 2500
// mm/s -- and one edge of lag instead of a filter's. With no new edge the
// estimate decays as mm_per_edge / (time since the last edge), so a stopped
// wheel reads zero within about one edge-time instead of a filter tail.
//
// Rising edges only (XBAR EDGE = 1): a slot and its gap are not the same
// width, so alternating rising/falling intervals would ripple at the slot
// rate; rising-to-rising is one slot pitch, 82 per rev per phase, and the
// two phases are a quarter pitch apart. What is left is the disc's own
// slot-to-slot pitch error, a percent or two at the slot rate -- the DMA
// record is what a disc map will be fitted from ('enc.map', later).
//
// Direction comes from the ENC count delta over the tick; the sign is
// latched while the count is still. The timestamps CAN give direction on
// their own -- the A->B and B->A gaps are T/4 and 3T/4 and swap with the
// direction -- and dir_edge reads it that way as a cross-check (A leading
// B is forward on the right wheel and backward on the left; EDGE_DIR_SIGN).
// PROVEN 2026-09-25 (bert's hand test, ~44 turns each wheel, forward and
// back): edge velocity mean within 1 % of the count velocity mean on both
// wheels, a steady 10-60 mm/s where the count column read 0/103; phase
// totals L 3595/3595, R 3817/3819 -- no slot missing; overrun 0. But that
// does not retire the ENC modules. Rising edges only see HALF the
// transitions: a wheel dithering back and forth across one edge position
// (the datum press, the end of a stop, a stall against a wall) raises the
// same phase's rising edge on every forward crossing and a falling edge,
// unseen, on every return -- the record reads A, A, A with no B, which is
// a wheel creeping forward, not a wheel standing still. The hardware
// quadrature decoder walks its state machine through both edges of both
// phases and counts +1, -1, +1, -1; it also carries a glitch filter this
// path has not got, and its count is the absolute slot index a disc map
// will be keyed on. So: the ENC count stays the POSITION and the anchor;
// the timestamps are the VELOCITY. Both are free -- the XBAR fans the
// same four signals out to both.
//
// Bench: 'enc' prints the count velocity and the edge velocity side by
// side while you spin a wheel. Registry 'vsrc' (#64) selects which one the
// wheel loop uses: 0 = counts per tick (as always), 1 = edge timing.
#pragma once

#include <Arduino.h>

// XBAR outputs 0..3, GPT1, four DMA channels. Call after hal_encoders_init()
// (the QuadEncoder library has set the pins and XBAR inputs by then).
void enc_timing_init();

// One control tick: fold in the edges captured since the last call.
// d_counts_* are this tick's ENC count deltas (for direction). Velocities
// are in NOMINAL mm/s (67.50 mm circumference); the caller applies the
// registry's per-wheel circumference scale as it does for counts.
void enc_timing_tick(int32_t d_counts_left, int32_t d_counts_right);

struct EncTimingWheel {
    float v_mm_s;         // signed, nominal circumference
    bool fresh;           // at least one edge arrived this tick
    uint32_t edges;       // edges seen this tick (both phases)
    float since_edge_us;  // time since the newest edge, at the tick
    uint32_t total_edges; // since init / enc_timing_reset_counts (both phases)
    uint32_t total_a, total_b;   // the same, per phase: a slot one phase
                                 // cannot see (dust, damage) reads as one
                                 // short per rev on that phase alone
    uint32_t overrun;     // ticks where a ring wrapped past the reader
    int8_t dir_count;     // direction latched from the ENC count delta (used)
    int8_t dir_edge;      // direction read from the A/B edge order (0 = none yet)
    uint32_t dir_mismatch;// ticks the two disagreed while the count moved
};
EncTimingWheel enc_timing_left();
EncTimingWheel enc_timing_right();
bool enc_timing_ready();   // init ran and the timer is counting
void enc_timing_reset_counts();   // zero total_edges/total_a/total_b/overrun/dir_mismatch
