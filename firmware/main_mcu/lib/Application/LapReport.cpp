#include "LapReport.h"

#include <math.h>
#include <stdio.h>   // snprintf, for the LPARAM lines
#include <string.h>

#include "Console.h"
#include "Drivers.h"  // fram_read / fram_write / fram_ready
#include "HAL.h"      // hal_radio_poll -- the report paces its lines
#include "Params.h"   // params_crc32

// Working state for the lap in progress. Fixed-size, no allocation.
static LapHeader hdr;
static LapCorner cor[LAPREP_MAX_CORNERS];
static float entry_yaw[LAPREP_MAX_CORNERS];
static int sweep_cmd[LAPREP_MAX_CORNERS];
static bool entered[LAPREP_MAX_CORNERS];
static bool closed_[LAPREP_MAX_CORNERS];
static bool active = false;

static int8_t clamp8(float v) {
    if (v > 127.0f) return 127;
    if (v < -127.0f) return -127;
    return (int8_t)lroundf(v);
}
static uint8_t clampu8(float v) {
    if (v > 255.0f) return 255;
    if (v < 0.0f) return 0;
    return (uint8_t)lroundf(v);
}
static float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// ---- the ring ----------------------------------------------------------------
static uint32_t slot_addr(int k) {
    return LAPREP_FRAM_ADDR + (uint32_t)k * LAPREP_FRAM_BYTES;
}
// Read slot k's header; true if it holds a report of this version.
// Header size on FRAM by version (the corners follow it directly).
static size_t hdr_bytes(uint8_t version) {
    return version == 4 ? sizeof(LapHeaderV4) : sizeof(LapHeader);
}
static bool slot_header(int k, LapHeader &h) {
    fram_read(slot_addr(k), (uint8_t *)&h, sizeof h);
    if (h.magic != LAPREP_MAGIC) return false;
    if (h.version == 4) {
        // A pre-ladder block: same fields up to vbat, then the CRC.
        LapHeaderV4 o;
        fram_read(slot_addr(k), (uint8_t *)&o, sizeof o);
        h.mr = 0; h.v_peak_x10 = 0; h.spare[0] = h.spare[1] = 0;
        h.crc = o.crc;
        return o.n <= LAPREP_MAX_CORNERS;
    }
    return h.version == LAPREP_VERSION && h.n <= LAPREP_MAX_CORNERS;
}
// The slot holding the newest report (highest seq), -1 if none.
static int newest_slot(uint16_t *seq_out = nullptr) {
    int best = -1;
    uint16_t best_seq = 0;
    for (int k = 0; k < LAPREP_RING_N; k++) {
        LapHeader h;
        if (!slot_header(k, h)) continue;
        // seq is a lap counter that only ever grows (uint16, wraps after
        // 65535 laps -- not this decade); the highest is the newest.
        if (best < 0 || h.seq > best_seq) { best = k; best_seq = h.seq; }
    }
    if (seq_out) *seq_out = best_seq;
    return best;
}
// Read and verify slot k; false on anything but a clean block.
static bool slot_load(int k, LapHeader &h, LapCorner *c, const char **why) {
    if (!slot_header(k, h)) { *why = "none stored"; return false; }
    const size_t hb = hdr_bytes(h.version);
    fram_read(slot_addr(k) + hb, (uint8_t *)c, h.n * sizeof(LapCorner));
    uint32_t crc = h.crc;
    // The CRC spans the header AS STORED (its own layout, sans crc).
    uint8_t buf[sizeof(LapHeader)];
    fram_read(slot_addr(k), buf, hb);
    uint32_t want = params_crc32(buf, hb - 4) ^
                    params_crc32((uint8_t *)c, h.n * sizeof(LapCorner));
    h.crc = crc;
    if (want != crc) { *why = "stored block failed CRC"; return false; }
    return true;
}

void lap_report_begin(char dir) {
    memset(&hdr, 0, sizeof hdr);
    memset(cor, 0, sizeof cor);
    // The FRONT gate fires BEFORE the arc entry, so its measurement lands
    // before corner_enter() opens the window -- clearing the column there
    // wiped every reading (laps 227-234: not one fcorr survived). The
    // sentinel is therefore set once per LAP, not per corner.
    for (int i = 0; i < LAPREP_MAX_CORNERS; i++) cor[i].front_corr_x2 = 127;
    memset(entered, 0, sizeof entered);
    memset(closed_, 0, sizeof closed_);
    hdr.magic = LAPREP_MAGIC;
    hdr.version = LAPREP_VERSION;
    hdr.dir = (uint8_t)dir;
    // Continue the lap counter from the stored block, so seq survives both
    // power cycles and re-flashes -- it is the x-axis of every trend the
    // scoreboard will ever plot.
    uint16_t newest = 0;
    hdr.seq = newest_slot(&newest) >= 0 ? (uint16_t)(newest + 1) : 1;
    active = true;
}

void lap_report_corner_enter(int k, uint8_t cx, uint8_t cy, uint8_t r_mm,
                             int sweep_deg, float yaw_now) {
    if (!active || k < 0 || k >= LAPREP_MAX_CORNERS || entered[k]) return;
    entered[k] = true;
    entry_yaw[k] = yaw_now;
    sweep_cmd[k] = sweep_deg;
    cor[k].cx = cx;
    cor[k].cy = cy;
    cor[k].r_mm = r_mm;
    cor[k].sweep_d3 = (int8_t)(sweep_deg / 3);
    cor[k].apex_min_mm = 255;
    cor[k].exit_lat_mm = 127;
    cor[k].gate_lat_mm = 127;
    if (k >= hdr.n) hdr.n = (uint8_t)(k + 1);
}

void lap_report_corner_sample(int k, float yaw_err_deg, uint8_t outer_beam) {
    if (!active || k < 0 || k >= LAPREP_MAX_CORNERS || !entered[k] ||
        closed_[k])
        return;
    uint8_t ye = clampu8(fabsf(yaw_err_deg) * 10.0f);
    if (ye > cor[k].max_ye_x10) cor[k].max_ye_x10 = ye;
    if (outer_beam != 0 && outer_beam != 255 &&
        outer_beam < cor[k].apex_min_mm)
        cor[k].apex_min_mm = outer_beam;
}

void lap_report_gate(int k, LapGate g, float corr_mm, float glat_mm) {
    if (!active || k < 0 || k >= LAPREP_MAX_CORNERS) return;
    cor[k].gate = (uint8_t)g;
    cor[k].gate_corr_x2 = clamp8(corr_mm * 2.0f);
    cor[k].gate_lat_mm = clamp8(glat_mm);
}

void lap_report_front(int k, float corr_mm) {
    if (!active || k < 0 || k >= LAPREP_MAX_CORNERS) return;
    cor[k].front_corr_x2 = clamp8(corr_mm * 2.0f);
}

static int16_t clamp16(float v) {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
}

void lap_report_segment(int k, float ds_mm, float dv_acc_mm_s,
                        float dv_brk_mm_s, float slip_mm) {
    if (!active || k < 0 || k >= LAPREP_MAX_CORNERS) return;
    cor[k].ds_mm = clamp16(ds_mm);
    cor[k].dv_acc_8 = clampu8(dv_acc_mm_s / 8.0f);
    cor[k].dv_brk_8 = clampu8(dv_brk_mm_s / 8.0f);
    cor[k].slip_x2 = clamp16(slip_mm * 2.0f);
}

void lap_report_corner_close(int k, float yaw_now, float exit_ye_deg,
                             bool lat_valid, float lat_mm) {
    if (!active || k < 0 || k >= LAPREP_MAX_CORNERS || !entered[k] ||
        closed_[k])
        return;
    closed_[k] = true;
    // The commanded sweep is the net tangent change; a multiple-of-360 wrap
    // cannot happen inside one corner window, so wrap the difference to the
    // sweep's neighbourhood by subtracting the command first.
    float swept = wrap180(yaw_now - entry_yaw[k] - (float)sweep_cmd[k]);
    float sw = swept * 10.0f;
    if (sw > 32767.0f) sw = 32767.0f;
    if (sw < -32767.0f) sw = -32767.0f;
    cor[k].sweep_err_x10 = (int16_t)sw;
    cor[k].exit_ye_x10 = clamp8(exit_ye_deg * 10.0f);
    cor[k].exit_lat_mm = lat_valid ? clamp8(lat_mm) : 127;
}

static const char *gate_name(uint8_t g) {
    switch (g) {
        case LR_GATE_LOCK: return "LOCK";
        case LR_GATE_NO_EDGE: return "NO_EDGE";
        case LR_GATE_SKIP_LAT: return "SKIP_LAT";
        case LR_GATE_INHERITED: return "INHERIT";
        case LR_GATE_DIAG: return "DIAG";
        case LR_GATE_FRONT: return "FRONT";
        case LR_GATE_FUSED: return "FUSED";
        default: return "NONE";
    }
}

// Print one stored/working block. Small, stationary-only output: a LAPREP
// header line, one LC line per corner, a terminator the GUI keys on.
// Field syntax is strictly key=value with NO commas inside values
// (cell=x:y, not x,y) so the GUI can split on ',' then '='.
//
// The pause between lines drains the radio TX ring: 13 back-to-back lines
// are ~1.1 KB against a 1 KB ring that only drains when hal_radio_poll
// runs -- the exact mechanism that clipped the flight-log dump (FlightLog
// 2026-08-28). 3 ms/line at 460800 baud is ~140 B of headroom per ~90 B
// line, and the mouse is stationary, so the 40 ms total costs nothing.
// Wait for ROOM, not for time: at 115200 baud a 100-byte line takes ~9 ms
// to leave and 3 ms of polling drains a third of it, so the six LPARAM
// lines plus the header could out-run the 1 KB ring when a lap end had
// other traffic queued ahead of it (2026-09-06: four laps flown, three
// LAPREP blocks filed). The LC lines already wait for room; this makes
// every line of the block do the same.
static void line_pause() {
    hal_radio_wait_room(128, 20);
}
static void print_block(const LapHeader &h, const LapCorner *c) {
    con_printf("LAPREP,seq=%u,dir=%c,ms=%u,mm=%u,corners=%u,dutypk=%.2f,"
               "endye=%+.1f",
               h.seq, (char)(h.dir & 0x7F), (unsigned)h.lap_ms_x2 * 2,
               h.len_mm, h.n,
               (double)h.dutypk_x100 / 100.0,
               (double)h.end_ye_x10 / 10.0);
    if (h.dir & 0x80) con_write(",abort=1");
    // Battery rides the header so duty peaks read against the pack that
    // produced them (dutypk 0.65 vs 0.49 fresh, run 210). Omitted when
    // never sampled, so stored pre-vbat blocks reprint unchanged.
    if (h.vbat_x50 > 0)
        con_printf(",vbat=%.2f", (double)h.vbat_x50 * 50.0 / 1000.0);
    // The run ladder (v5): which rung, and the peak the leg was compiled
    // for. Absent on v4 blocks and with the ladder off.
    if (h.mr > 0) con_printf(",mr=%u", (unsigned)h.mr);
    if (h.v_peak_x10 > 0) con_printf(",v=%u", (unsigned)h.v_peak_x10 * 10);
    con_write("\n");
    line_pause();
    for (int i = 0; i < h.n && i < LAPREP_MAX_CORNERS; i++) {
        hal_radio_wait_room(160, 20);   // one LC line, see HAL.h
        con_printf("LC,i=%d,cell=%u:%u,r=%u,sweep=%d,err=%+.1f,maxye=%.1f,"
                   "apex=%u,exitye=%+.1f,lat=%d,gate=%s,corr=%+.1f,glat=%d",
                   i, c[i].cx, c[i].cy, c[i].r_mm,
                   (int)c[i].sweep_d3 * 3,
                   (double)c[i].sweep_err_x10 / 10.0,
                   (double)c[i].max_ye_x10 / 10.0, c[i].apex_min_mm,
                   (double)c[i].exit_ye_x10 / 10.0, (int)c[i].exit_lat_mm,
                   gate_name(c[i].gate),
                   (double)c[i].gate_corr_x2 / 2.0, (int)c[i].gate_lat_mm);
        // The front column, only where the front wall actually spoke.
        if (c[i].front_corr_x2 != 127)
            con_printf(",fcorr=%+.1f", (double)c[i].front_corr_x2 / 2.0);
        // The slip instrument, only where this corner's gate anchored.
        if (c[i].ds_mm != 0)
            con_printf(",ds=%d,dva=%u,dvb=%u,slip=%+.1f", (int)c[i].ds_mm,
                       (unsigned)c[i].dv_acc_8 * 8u,
                       (unsigned)c[i].dv_brk_8 * 8u,
                       (double)c[i].slip_x2 / 2.0);
        con_write("\n");
        line_pause();
    }
    con_println("LAPREP,end");
    line_pause();
}

// The registry, as flown, on LPARAM lines ahead of the report -- so the
// GUI's lap file carries the settings WITH each lap's numbers (bert,
// 2026-08-29). Live gains are what flew: they cannot change mid-leg. Only
// the lap-end path prints this ('lr' reprints a stored block, possibly
// from before a power cycle, and must not stamp it with today's gains).
static void print_params_live() {
    ControlGains g = control_get_gains();
    char line[100];
    int o = snprintf(line, sizeof line, "LPARAM");
    for (int i = 0; i < PARAM_COUNT; i++) {
        char item[48];
        int n = snprintf(item, sizeof item, ",%s=%g", PARAM_REGISTRY[i].name,
                         (double)(g.*(PARAM_REGISTRY[i].field)));
        if (o + n >= (int)sizeof line - 1) {
            con_println(line);
            line_pause();
            o = snprintf(line, sizeof line, "LPARAM");
        }
        o += snprintf(line + o, sizeof line - o, "%s", item);
    }
    con_println(line);
    line_pause();
}

void lap_report_set_vbat(uint16_t mv) {
    if (!active) return;
    uint32_t v = (mv + 25) / 50;
    hdr.vbat_x50 = (uint8_t)(v > 255 ? 255 : v);
}

void lap_report_set_run(uint8_t mr, float v_peak_mm_s) {
    if (!active) return;
    hdr.mr = mr;
    float v10 = v_peak_mm_s / 10.0f;
    hdr.v_peak_x10 = (uint8_t)(v10 < 0 ? 0 : v10 > 255 ? 255 : v10);
}

void lap_report_abort(uint32_t lap_ms, float len_mm, float duty_peak) {
    if (!active) return;
    // Everything finish() does -- LPARAM lines, FRAM, the printed block --
    // is exactly what a crash investigation wants, so an abort IS a finish
    // with the mark set. endye is meaningless mid-crash; 0 it.
    hdr.dir |= 0x80;
    lap_report_finish(lap_ms, len_mm, duty_peak, 0.0f);
}

void lap_report_finish(uint32_t lap_ms, float len_mm, float duty_peak,
                       float end_ye_deg) {
    if (!active) return;
    active = false;
    print_params_live();
    hdr.lap_ms_x2 = (uint16_t)(lap_ms / 2 > 0xFFFF ? 0xFFFF : lap_ms / 2);
    hdr.len_mm = (uint16_t)(len_mm < 0 ? 0
                            : len_mm > 65535.0f ? 65535
                                                : lroundf(len_mm));
    hdr.dutypk_x100 = clampu8(duty_peak * 100.0f);
    hdr.end_ye_x10 = clamp8(end_ye_deg * 10.0f);
    // CRC over header-sans-crc + the recorded corners, same discipline as
    // the params block: a torn write must read as "no report", never as a
    // plausible one.
    uint8_t buf[sizeof(LapHeader) + LAPREP_MAX_CORNERS * sizeof(LapCorner)];
    hdr.crc = 0;
    memcpy(buf, &hdr, sizeof hdr);
    memcpy(buf + sizeof hdr, cor, hdr.n * sizeof(LapCorner));
    hdr.crc = params_crc32(buf, sizeof(LapHeader) - 4);  // header fields
    hdr.crc = params_crc32(buf + sizeof(LapHeader),
                           hdr.n * sizeof(LapCorner)) ^ hdr.crc;
    if (fram_ready()) {
        // Slot = seq % 8: a run's reports land in consecutive slots and the
        // oldest is what gets overwritten.
        const uint32_t a = slot_addr(hdr.seq % LAPREP_RING_N);
        fram_write(a, (const uint8_t *)&hdr, sizeof hdr);
        fram_write(a + sizeof hdr, (const uint8_t *)cor,
                   hdr.n * sizeof(LapCorner));
    }
    print_block(hdr, cor);   // the mouse is stationary: transmit at stop
}

void lap_report_print(int back) {
    uint16_t newest = 0;
    int k = newest_slot(&newest);
    if (k < 0) {
        con_println("lap report: none stored");
        return;
    }
    if (back < 0) back = 0;
    if (back >= LAPREP_RING_N) {
        con_printf("lap report: the ring keeps %d (newest seq %u)\n",
                   LAPREP_RING_N, newest);
        return;
    }
    // The report `back` laps ago sits in the slot `back` below the newest
    // -- if it is still the report with that seq (a slot older than the
    // ring's depth has been overwritten by a newer lap, which then has the
    // wrong seq for the ask).
    int want_seq = (int)newest - back;
    int j = ((k - back) % LAPREP_RING_N + LAPREP_RING_N) % LAPREP_RING_N;
    LapHeader h;
    LapCorner c[LAPREP_MAX_CORNERS];
    const char *why = "";
    if (want_seq < 1 || !slot_load(j, h, c, &why) || h.seq != want_seq) {
        con_printf("lap report: no report %d back (newest seq %u; the "
                   "ring keeps %d)\n", back, newest, LAPREP_RING_N);
        return;
    }
    print_block(h, c);
}

int lap_report_print_all() {
    // Oldest first, by seq, so a scoreboard ingesting the whole ring sees
    // laps in the order they were flown.
    int printed = 0;
    LapHeader h;
    LapCorner c[LAPREP_MAX_CORNERS];
    const char *why = "";
    int order[LAPREP_RING_N];
    uint16_t seqs[LAPREP_RING_N];
    int n = 0;
    for (int k = 0; k < LAPREP_RING_N; k++) {
        if (!slot_header(k, h)) continue;
        order[n] = k;
        seqs[n] = h.seq;
        n++;
    }
    for (int i = 1; i < n; i++)          // insertion sort by seq, n <= 8
        for (int j = i; j > 0 && seqs[j - 1] > seqs[j]; j--) {
            uint16_t ts = seqs[j]; seqs[j] = seqs[j - 1]; seqs[j - 1] = ts;
            int to = order[j]; order[j] = order[j - 1]; order[j - 1] = to;
        }
    for (int i = 0; i < n; i++) {
        if (!slot_load(order[i], h, c, &why)) continue;
        print_block(h, c);
        printed++;
    }
    if (printed == 0) con_println("lap report: none stored");
    return printed;
}

uint32_t lap_report_latest_addr() {
    int k = newest_slot();
    return k < 0 ? 0 : slot_addr(k);
}
