// Edge-timed wheel velocity -- see EncoderTiming.h. From bert's
// Encoder_XBAR_DMA_V5_GPT example (firmware/example_code), made a driver:
// same XBAR inputs, outputs and DMA sources, same GPT1 at 8 MHz; the
// per-tick readout is the M/T window plus the no-edge decay, and the DMA
// rings are read by CITER exactly as the example does.
#include "EncoderTiming.h"

#include <DMAChannel.h>

#include "Encoders.h"

// ---- geometry -----------------------------------------------------------
// 82 slots per rev; rising edges only, one stream per phase: 82 edges per
// rev per phase, 0.8232 mm each at the nominal circumference.
static constexpr float EDGES_PER_REV_PHASE = ENC_COUNTS_PER_REV / 4.0f;   // 82
static constexpr float MM_PER_EDGE = ENC_WHEEL_CIRCUMFERENCE_MM / EDGES_PER_REV_PHASE;
static constexpr float GPT_HZ = 8.0e6f;                 // 24 MHz OSC / 3
static constexpr float US_PER_TICK = 1.0e6f / GPT_HZ;   // 0.125
static constexpr float V_FLOOR_MM_S = 3.0f;             // below this: stopped

// ---- the rings ----------------------------------------------------------
// 256 timestamps per phase: at 2500 mm/s a phase sees ~6 edges per 2 ms
// tick, so the ring holds ~85 ms -- forty ticks of slack. DTCM (plain
// globals on the Teensy 4) is not cached, so the DMA writes are visible
// to the reader without any cache maintenance.
static constexpr uint16_t RING_N = 256;
struct Phase {
    DMAChannel dma;
    uint32_t ring[RING_N];
    uint16_t citer_last;   // CITER at the previous tick
    uint32_t t_last;       // newest timestamp seen
    uint32_t pitch;        // last rising-to-rising interval, ticks (0 = none)
    bool seen;             // any edge ever
};
struct Wheel {
    Phase a, b;
    float v;               // nominal mm/s, signed
    int8_t dir;            // latched from the count delta
    EncTimingWheel out;
};
static Wheel W[2];
static bool ready_ = false;
// A-leads-B is forward on the right wheel and BACKWARD on the left (the
// left encoder is the right one mirrored): bert's hand test, 2026-09-25
// 14:43, 397 of 415 forward ticks read -1 on the left and +1 on the right.
static constexpr int8_t EDGE_DIR_SIGN[2] = {-1, +1};

// XBAR input numbers for the encoder pins (the same ones the QuadEncoder
// library routes to the ENC modules): pin 0 = AD_B0_03 -> IN17, pin 1 =
// AD_B0_02 -> IN16, pin 2 = EMC_04 -> IN06, pin 3 = EMC_05 -> IN07.
// XBAR outputs 0..3 are the four with DMA requests (DMAMUX sources 30, 31,
// 94, 95). Left A/B -> out 0/1, right A/B -> out 2/3.
static constexpr uint8_t XIN_L_A = 17, XIN_L_B = 16, XIN_R_A = 6, XIN_R_B = 7;
static const uint8_t DMA_SRC[4] = {DMAMUX_SOURCE_XBAR1_0, DMAMUX_SOURCE_XBAR1_1,
                                   DMAMUX_SOURCE_XBAR1_2, DMAMUX_SOURCE_XBAR1_3};

static void xbar_connect(unsigned input, unsigned output) {
    volatile uint16_t *sel = &XBARA1_SEL0 + (output / 2);
    uint16_t v = *sel;
    v = (output & 1) ? (uint16_t)((v & 0x00FF) | (input << 8))
                     : (uint16_t)((v & 0xFF00) | input);
    *sel = v;
}

static void dma_setup(Phase &p, uint8_t src) {
    // Every request copies GPT1_CNT (32-bit) into the next ring slot; the
    // ring wraps by DLASTSGA. CITER counts DOWN from RING_N and reloads.
    p.dma.TCD->SADDR = &GPT1_CNT;
    p.dma.TCD->SOFF = 0;
    p.dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);   // 32-bit
    p.dma.TCD->NBYTES_MLNO = 4;
    p.dma.TCD->SLAST = 0;
    p.dma.TCD->DADDR = p.ring;
    p.dma.TCD->DOFF = 4;
    p.dma.TCD->CITER_ELINKNO = RING_N;
    p.dma.TCD->BITER_ELINKNO = RING_N;
    p.dma.TCD->DLASTSGA = -(int32_t)(RING_N * 4);
    p.dma.TCD->CSR = 0;
    p.dma.triggerAtHardwareEvent(src);
    p.dma.enable();
    p.citer_last = RING_N;
    p.t_last = 0;
    p.pitch = 0;
    p.seen = false;
}

void enc_timing_init() {
    CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
    CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
    // GPT1: free-running 32-bit at 24 MHz / 3 = 8 MHz. Nothing else in the
    // firmware uses GPT1 (the control tick is a PIT via IntervalTimer).
    GPT1_CR = 0;
    GPT1_SR = 0x3F;
    GPT1_PR = 3 - 1;
    GPT1_CR = GPT_CR_EN | GPT_CR_CLKSRC(1) | GPT_CR_FRR;
    // The encoder pins are inputs to the XBAR (DIR_SEL = 0); the
    // QuadEncoder library has already muxed them and routed IN16/17/6/7 to
    // the ENC modules. Connect the same inputs to outputs 0..3 as well.
    IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_17 |
                         IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_16 |
                         IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_6 |
                         IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_7);
    xbar_connect(XIN_L_A, 0);
    xbar_connect(XIN_L_B, 1);
    xbar_connect(XIN_R_A, 2);
    xbar_connect(XIN_R_B, 3);
    // RISING edges only (EDGE = 1), DMA request enabled, status cleared.
    XBARA1_CTRL0 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(1) | XBARA_CTRL_DEN0 |
                   XBARA_CTRL_STS1 | XBARA_CTRL_EDGE1(1) | XBARA_CTRL_DEN1;
    XBARA1_CTRL1 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(1) | XBARA_CTRL_DEN0 |
                   XBARA_CTRL_STS1 | XBARA_CTRL_EDGE1(1) | XBARA_CTRL_DEN1;
    dma_setup(W[0].a, DMA_SRC[0]);
    dma_setup(W[0].b, DMA_SRC[1]);
    dma_setup(W[1].a, DMA_SRC[2]);
    dma_setup(W[1].b, DMA_SRC[3]);
    for (auto &w : W) { w.v = 0.0f; w.dir = 1; w.out = EncTimingWheel{}; }
    ready_ = true;
}

bool enc_timing_ready() { return ready_; }
void enc_timing_reset_counts() {
    for (auto &w : W) {
        w.out.total_edges = 0; w.out.total_a = 0; w.out.total_b = 0;
        w.out.overrun = 0; w.out.dir_mismatch = 0;
    }
}

// Read one phase's ring: how many edges arrived since the last tick, and
// the M/T interval they span (from the previous window's last edge to this
// window's last edge). Returns the edge count; *span_ticks the time.
static uint32_t phase_read(Phase &p, uint32_t *span_ticks, uint32_t *newest,
                           uint32_t *overrun) {
    uint16_t citer = p.dma.TCD->CITER_ELINKNO;   // next slot to write, counting down
    // Slots written since last time: indices (RING_N - citer_last) ..
    // (RING_N - citer) - 1, modulo the ring.
    uint32_t start = RING_N - p.citer_last, end = RING_N - citer;
    uint32_t n = (end + RING_N - start) % RING_N;
    p.citer_last = citer;
    if (n == 0) { *span_ticks = 0; return 0; }
    if (n >= RING_N - 8) { (*overrun)++; }       // reader nearly lapped
    uint32_t last = p.ring[(end + RING_N - 1) % RING_N];
    if (!p.seen) {
        // First edge ever: no previous window to span from. Use the first
        // and last of this batch if there are two, else nothing yet.
        if (n >= 2) *span_ticks = last - p.ring[start % RING_N];
        else *span_ticks = 0;
        p.seen = true;
        p.t_last = last;
        *newest = last;
        return n >= 2 ? n - 1 : 0;
    }
    *span_ticks = last - p.t_last;   // uint32 wrap-safe
    p.t_last = last;
    p.pitch = *span_ticks / n;       // mean pitch over this batch
    *newest = last;
    return n;
}

static void wheel_tick(Wheel &w, int32_t d_counts, uint32_t now) {
    if (d_counts > 0) w.dir = 1; else if (d_counts < 0) w.dir = -1;
    uint32_t sa, sb, na_t = 0, nb_t = 0, ovr = 0;
    uint32_t na = phase_read(w.a, &sa, &na_t, &ovr);
    uint32_t nb = phase_read(w.b, &sb, &nb_t, &ovr);
    float v = -1.0f;
    // M/T per phase, then the mean of whichever phases delivered.
    float va = (na && sa) ? na * MM_PER_EDGE * GPT_HZ / (float)sa : -1.0f;
    float vb = (nb && sb) ? nb * MM_PER_EDGE * GPT_HZ / (float)sb : -1.0f;
    if (va >= 0 && vb >= 0) v = 0.5f * (va + vb);
    else if (va >= 0) v = va;
    else if (vb >= 0) v = vb;
    w.out.edges = na + nb;
    w.out.fresh = v >= 0;
    w.out.total_edges += na + nb;
    w.out.total_a += na;
    w.out.total_b += nb;
    w.out.overrun += ovr;
    // Time since the newest edge of either phase.
    uint32_t t_new = 0;
    if (w.a.seen) t_new = w.a.t_last;
    if (w.b.seen && (!w.a.seen || (int32_t)(w.b.t_last - t_new) > 0)) t_new = w.b.t_last;
    float since_us = (w.a.seen || w.b.seen) ? (now - t_new) * US_PER_TICK : 1e9f;
    w.out.since_edge_us = since_us;
    if (v < 0) {
        // No edge this tick: the speed can be no more than one edge over
        // the time since the last one. Decay toward that bound.
        float bound = since_us > 1.0f ? MM_PER_EDGE * 1e6f / since_us : w.v;
        v = fabsf(w.v);
        if (bound < v) v = bound;
        if (v < V_FLOOR_MM_S) v = 0.0f;
    }
    w.v = v;
    w.out.v_mm_s = v * w.dir;
    // DIRECTION FROM THE TIMESTAMPS ALONE (bert, 2026-09-25: "if the edge
    // counters are per channel the relative timings give direction").
    // The two phases are a quarter pitch apart, so of the two gaps
    // between consecutive rising edges, A->B and B->A, one is T/4 and the
    // other 3T/4 -- and which is the short one flips with the direction
    // of travel, whatever the slot's mark/space ratio. Judged from the
    // newest edge of each phase whenever both fell within one pitch. The
    // sign convention (A leading B = +1) may be mirrored per wheel; the
    // 'enc' readout prints it beside the count direction so the wiring
    // says which. See the header for why this does NOT retire the ENC
    // count: a wheel dithering on one edge position produces rising
    // edges on one phase only, which no gap rule can read.
    if (w.a.seen && w.b.seen && w.a.pitch && w.b.pitch) {
        int32_t dt = (int32_t)(w.b.t_last - w.a.t_last);   // >0: B newer
        uint32_t gap = dt > 0 ? (uint32_t)dt : (uint32_t)(-dt);
        uint32_t T = (w.a.pitch + w.b.pitch) / 2;
        // Judge only when the gap is clearly a quarter or three quarters of
        // a pitch: within 0.4..0.6 T (a hand reversing, a pitch changing
        // fast) the two are not told apart and the last verdict stands. The
        // same hand test read 5 % mismatches without this band.
        if (gap < T * 2 / 5 || (gap > T * 3 / 5 && gap < T)) {
            bool short_gap = gap < T / 2;
            int8_t d = dt > 0 ? (short_gap ? 1 : -1) : (short_gap ? -1 : 1);
            d = (int8_t)(d * EDGE_DIR_SIGN[&w - W]);
            w.out.dir_edge = d;
            if (d_counts != 0 && d != w.dir) w.out.dir_mismatch++;
        }
    }
    w.out.dir_count = w.dir;
}

void enc_timing_tick(int32_t d_counts_left, int32_t d_counts_right) {
    if (!ready_) return;
    uint32_t now = GPT1_CNT;
    wheel_tick(W[0], d_counts_left, now);
    wheel_tick(W[1], d_counts_right, now);
}

EncTimingWheel enc_timing_left() { return W[0].out; }
EncTimingWheel enc_timing_right() { return W[1].out; }
