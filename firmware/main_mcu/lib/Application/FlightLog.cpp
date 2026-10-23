#include "FlightLog.h"

#include <stdio.h>
#include <string.h>   // strncpy, for the settings banner

#include "Control.h"  // control_get_gains -- the snapshot reads the LIVE gains
#include "Drivers.h"
#include "HAL.h"     // hal_radio_poll -- paced() drains the TX ring while it waits
#include "Params.h"  // params_crc32 + the registry serialization (snapshot)

// The reserved field must hold the registry serialization, with room noted
// in FlightLog.h for a few appended params. Appending past that point is
// allowed -- it just costs another record-base shift, and this assert is
// what turns that from a silent overlap into a build error.
static_assert(PARAMS_BLOCK_BYTES <= FLOG_PARAMS_BYTES,
              "params snapshot outgrew its flight-log field -- grow "
              "FLOG_PARAMS_BYTES (shifts FLOG_REC_BASE: old stored logs "
              "will not dump on the new firmware)");

// Per-line pace for the dump (bert, 2026-08-04): the radio bridge dropped
// bytes with the old delay(2)/32-lines burst (~1800 B before a pause).
// Throttle EVERY line so throughput stays inside the link's buffer. A
// record line is ~55 B; at 6 ms/line that is ~9 kB/s -- ~12 s for a full
// 2000-record dump, comfortably paced. Raise if corruption persists.
//
// DRAIN WHILE WAITING (2026-08-28, runs 195/196): the wait must be a
// hal_radio_poll() loop, NOT delay(). The radio TX path is a 1 KB ring
// (HAL.cpp) that only moves bytes into the UART when hal_radio_poll()
// runs, and this dump blocks the main loop -- so with delay() the ONLY
// drain per line was the single poll inside hal_radio_write, capped at
// Serial5.availableForWrite() ~= 40 bytes. Every ~55 B record line over
// the radio therefore got ~44 bytes through and lost its tail -- the
// exact 44-char clipping seen whenever a log was fetched over WiFi
// ("bulk dumps still belong on USB" stopped being an option the day the
// GUI's hands-free fetch made every fetch a radio fetch). Polling for
// the same 6 ms drains ~276 B at 460800 baud: the ring never backs up
// and the average pace (~9 kB/s) the bridge is known to handle is
// unchanged.
static constexpr uint32_t FLOG_LINE_PACE_MS = 6;
static void paced(void (*fn)(const char *), const char *s) {
    fn(s);
    uint32_t t0 = millis();
    while (millis() - t0 < FLOG_LINE_PACE_MS) hal_radio_poll();
    hal_radio_poll();  // at least one drain even if millis just rolled
}

// Persistent header at FLOG_BASE (see FlightLog.h). Do not reorder.
struct FlogHdr {
    uint32_t magic;   // 'TW3L'
    uint32_t count;   // total records written this run
    uint32_t spare;
    uint32_t crc;     // over the three fields above
};
static_assert(sizeof(FlogHdr) == FLOG_HDR_BYTES, "header must stay 16 B");

constexpr uint32_t FLOG_MAGIC = 0x5457334C;  // "TW3L"

static bool active_ = false;
static uint32_t n_ = 0;        // total records written (ring index = n % CAP)
static uint32_t t0_ms_ = 0;

static void write_hdr() {
    FlogHdr h;
    h.magic = FLOG_MAGIC;
    h.count = n_;
    h.spare = 0;
    h.crc = params_crc32(&h, sizeof(FlogHdr) - sizeof(uint32_t));
    fram_write(FLOG_BASE, &h, sizeof h);
}

// REGION OWNERSHIP (V2, 2026-08-18). The flight recorder shares the bulk
// FRAM region with the loop sweep (LoopSweep.h): they instrument different
// things, are used months apart, and splitting 510 KB between them would
// halve both. Sharing is only safe if neither writes into a region the other
// owns -- and this file writes fixed-address blocks (the banner, the outcome
// string) that land inside the sweep's summary area.
//
// So every write here is gated on own_. It is true when boot found a flight
// log in the region, and it becomes true when flog_arm() deliberately takes
// the region over. After a sweep session a search abort therefore fails to
// record its outcome string rather than scribbling over the experiment --
// which is the right way round: the outcome is also printed, the sweep data
// is not recoverable.
static bool own_ = false;

void flog_init() {
    FlogHdr h;
    fram_read(FLOG_BASE, &h, sizeof h);
    if (h.magic == FLOG_MAGIC &&
        h.crc == params_crc32(&h, sizeof(FlogHdr) - sizeof(uint32_t))) {
        n_ = h.count;  // previous run readable via 'y'
        own_ = true;
    } else {
        n_ = 0;
        own_ = false;  // the region belongs to something else
    }
    active_ = false;
}

bool flog_owns_region() { return own_; }

void flog_set_walls(const uint16_t *v, const uint16_t *h, int n) {
    if (!own_) return;  // the sweep owns the region -- see flog_init
    if (!fram_ready()) return;
    uint8_t buf[FLOG_WALL_BYTES] = {0};
    // [0]=magic 0xW, [1]=n; then n v-uint16 LE, then n h-uint16 LE.
    buf[0] = 0x57; buf[1] = (uint8_t)n;
    int o = 2;
    for (int i = 0; i < n && o + 2 <= (int)FLOG_WALL_BYTES; i++, o += 2)
        { buf[o] = v[i] & 0xFF; buf[o+1] = v[i] >> 8; }
    for (int i = 0; i < n && o + 2 <= (int)FLOG_WALL_BYTES; i++, o += 2)
        { buf[o] = h[i] & 0xFF; buf[o+1] = h[i] >> 8; }
    fram_write(FLOG_WALL_ADDR, buf, FLOG_WALL_BYTES);
}

void flog_arm() {
    own_ = true;  // deliberately taking the region over
    active_ = true;
    n_ = 0;
    t0_ms_ = millis();
    write_hdr();
    // Stamp the LIVE gains into the log, every arm, unconditionally. This is
    // the record of what actually flew -- FRAM overrides compiled defaults,
    // so the source code is not evidence and the banner only ever carried
    // seven values. Reading them back is the dump's job (PARAM,... lines).
    params_save_at(FLOG_PARAMS_ADDR, control_get_gains());
}

void flog_stop() {
    if (active_) {
        active_ = false;
        write_hdr();
    }
}

void flog_resume() {
    if (!own_ || active_) return;
    active_ = true;
    write_hdr();
}

bool flog_active() { return active_; }

uint32_t flog_count() { return n_; }

void flog_record(const FlogRec &r) {
    if (!own_) return;  // the sweep owns the region -- see flog_init
    if (!active_ || !fram_ready()) return;
    FlogRec rec = r;
    rec.t_ms = (uint16_t)(millis() - t0_ms_);
    fram_write(FLOG_REC_BASE + (n_ % FLOG_CAPACITY) * FLOG_REC_BYTES, &rec,
               FLOG_REC_BYTES);
    n_++;
    // Keep the persistent count near-current so a mid-run power cut
    // still leaves the log retrievable (FeRAM: writes are free).
    if ((n_ & 15) == 0) write_hdr();
}

void flog_set_banner(const char *text) {
    if (!own_) return;  // the sweep owns the region -- see flog_init
    if (!fram_ready()) return;
    char b[FLOG_BANNER_BYTES];
    memset(b, 0, sizeof b);
    if (text) strncpy(b, text, sizeof(b) - 1);
    fram_write(FLOG_BANNER_ADDR, b, sizeof b);
    // Clear the outcome AND the horizon field. memset the WHOLE buffer first:
    // b still holds the banner past FLOG_OUTCOME_BYTES, and writing that tail
    // into the horizon field left it starting with 96 zeros followed by the
    // back half of the settings line -- which reads as "empty" to the dump and
    // so silently suppressed the HORIZON line on runs 090 and 092. Introduced
    // and found in the same day, on bert's first flight of it.
    memset(b, 0, sizeof b);
    fram_write(FLOG_OUTCOME_ADDR, b, FLOG_OUTCOME_BYTES);
    fram_write(FLOG_HZ_ADDR, b, FLOG_HZ_BYTES);
}

void flog_set_outcome(const char *why) {
    if (!own_) return;  // the sweep owns the region -- see flog_init
    if (!fram_ready() || !why) return;
    char b[FLOG_OUTCOME_BYTES];
    fram_read(FLOG_OUTCOME_ADDR, b, sizeof b);
    b[sizeof(b) - 1] = 0;
    // Truncate at the first byte that is not printable ASCII, rather than only
    // testing b[0]. Log 091 dumped an OUTCOME with eight bytes of binary
    // sitting where its terminator should have been, and the appended text ran
    // on past it -- strlen had happily walked straight through. A field that
    // has demonstrably held garbage is not a field to trust the first byte of.
    {
        size_t i = 0;
        while (i < sizeof(b) && b[i] >= 32 && b[i] <= 126) i++;
        b[i < sizeof(b) ? i : sizeof(b) - 1] = 0;
    }
    size_t n = strlen(b);
    if (n + 4 >= sizeof(b)) return;             // full: keep the FIRST reason,
                                                // which is the one that fired
    snprintf(b + n, sizeof(b) - n, n ? " | %s" : "%s", why);
    fram_write(FLOG_OUTCOME_ADDR, b, sizeof b);
}

void flog_set_horizon(const char *text) {
    if (!own_) return;  // the sweep owns the region -- see flog_init
    if (!fram_ready()) return;
    char b[FLOG_HZ_BYTES];
    memset(b, 0, sizeof b);
    if (text) strncpy(b, text, sizeof(b) - 1);
    fram_write(FLOG_HZ_ADDR, b, sizeof b);
}

void flog_dump(void (*println_fn)(const char *)) {
    if (!own_ && println_fn) {
        println_fn("no flight log: the region holds a loop-sweep log "
                   "('sweep.dump')");
        return;
    }
    if (!println_fn) return;
    // 192, not 128: the row grew to 25 columns with the predicted wall vector
    // and a realistic worst case is ~140 chars. snprintf would have TRUNCATED
    // silently, losing the last columns of every record -- which is exactly
    // the sort of thing that looks like a parser bug for a week.
    char line[192];
    uint32_t count = n_ < FLOG_CAPACITY ? n_ : FLOG_CAPACITY;
    uint32_t first = n_ < FLOG_CAPACITY ? 0 : n_ - FLOG_CAPACITY;
    // Wall-map preamble: exact searched map (see flog_set_walls). Lines
    // "MAZE_V,i,hex" and "MAZE_H,i,hex"; analyzer converts i,bit -> mm.
    {
        uint8_t wb[FLOG_WALL_BYTES];
        fram_read(FLOG_WALL_ADDR, wb, FLOG_WALL_BYTES);
        if (wb[0] == 0x57) {
            int nw = wb[1], o = 2;
            for (int i = 0; i < nw; i++, o += 2) {
                snprintf(line, sizeof line, "MAZE_V,%d,%u", i,
                         (unsigned)(wb[o] | (wb[o+1] << 8)));
                paced(println_fn, line);
            }
            for (int i = 0; i < nw; i++, o += 2) {
                snprintf(line, sizeof line, "MAZE_H,%d,%u", i,
                         (unsigned)(wb[o] | (wb[o+1] << 8)));
                paced(println_fn, line);
            }
        }
    }
    {
        char b[FLOG_BANNER_BYTES > FLOG_OUTCOME_BYTES ? FLOG_BANNER_BYTES
                                                      : FLOG_OUTCOME_BYTES];
        fram_read(FLOG_BANNER_ADDR, b, FLOG_BANNER_BYTES);
        b[sizeof(b) - 1] = 0;
        // Only printable ASCII: virgin FeRAM reads back 0xFF, and a garbage
        // banner is worse than none because it looks like a real setting.
        bool ok = b[0] >= 32 && b[0] < 127;
        for (unsigned i = 0; ok && i < sizeof(b) && b[i]; i++)
            if (b[i] < 32 || b[i] > 126) ok = false;
        if (ok) { snprintf(line, sizeof line, "SETTINGS,%s", b); println_fn(line); }
        fram_read(FLOG_OUTCOME_ADDR, b, FLOG_OUTCOME_BYTES);
        b[FLOG_OUTCOME_BYTES - 1] = 0;
        ok = b[0] >= 32 && b[0] < 127;
        for (unsigned i = 0; ok && i < FLOG_OUTCOME_BYTES && b[i]; i++)
            if (b[i] < 32 || b[i] > 126) ok = false;
        if (ok) { snprintf(line, sizeof line, "OUTCOME,%s", b); println_fn(line); }
    }
    {
        char hb[FLOG_HZ_BYTES];
        fram_read(FLOG_HZ_ADDR, hb, FLOG_HZ_BYTES);
        hb[FLOG_HZ_BYTES - 1] = 0;
        bool ok = hb[0] >= 32 && hb[0] < 127;
        for (unsigned i = 0; ok && i < FLOG_HZ_BYTES && hb[i]; i++)
            if (hb[i] < 32 || hb[i] > 126) ok = false;
        if (ok) { snprintf(line, sizeof line, "HORIZON,%s", hb);
                  println_fn(line); }
    }
    // The gains that flew this log, from the snapshot flog_arm stamped --
    // NOT the live gains, which may have been retuned since the run. A log
    // armed by pre-snapshot firmware fails the CRC and prints nothing, which
    // is honest: those runs genuinely did not record their gains.
    {
        ControlGains snap;
        if (params_load_at(FLOG_PARAMS_ADDR, snap)) {
            size_t o = (size_t)snprintf(line, sizeof line, "PARAM");
            for (int i = 0; i < PARAM_COUNT; i++) {
                int w = snprintf(line + o, sizeof line - o, ",%s=%.6g",
                                 PARAM_REGISTRY[i].name,
                                 (double)(snap.*(PARAM_REGISTRY[i].field)));
                if (w < 0 || (size_t)w >= sizeof line - o) {
                    // Full -- flush and continue on a fresh PARAM line rather
                    // than truncating silently (the log 078 lesson).
                    println_fn(line);
                    o = (size_t)snprintf(line, sizeof line, "PARAM");
                    w = snprintf(line + o, sizeof line - o, ",%s=%.6g",
                                 PARAM_REGISTRY[i].name,
                                 (double)(snap.*(PARAM_REGISTRY[i].field)));
                }
                o += (size_t)w;
            }
            println_fn(line);
        }
    }
    snprintf(line, sizeof line, "flight log: %lu records",
             (unsigned long)count);
    println_fn(line);
    // -- DEAD COLUMNS ARE NOT EMITTED (bert, 2026-08-27: "remove data from the
    // route run that prints zero's, it just bloats the logs").
    //
    // A route run fills none of the pose-observer or wall-prediction fields --
    // the observer runs on the search, not on a path -- so twelve of the
    // twenty-six columns are the character '0' on every one of ~10000 rows.
    // That is over half the bytes of the dump, sent down a radio link at
    // FLOG_LINE_PACE_MS per line.
    //
    // The test is MEASURED, not declared. Scanning the records and dropping
    // any optional column that is zero in all of them costs one extra FRAM
    // pass -- a fraction of a second against a dump that takes minutes -- and
    // has two properties a hard-coded "route runs don't have these" list would
    // not: it cannot go stale when the observer is switched back on, and a
    // column that IS present is thereby proof that something wrote to it.
    //
    // Only the observer/prediction block is optional. The core columns stay
    // unconditionally even when one happens to be all-zero, because there a
    // zero is a measurement: `omega` reading zero through a corner is a fault
    // report, and silently dropping the column would hide it. (It read zero on
    // every path run ever flown until 2026-08-27 -- exactly the bug this rule
    // is written to keep visible.)
    enum {
        C_SLIDE, C_OBS_ALONG, C_OBS_CROSS, C_OBS_N, C_CLEAR,
        C_PAX_L, C_PCO_L, C_PRN_L, C_PAX_R, C_PCO_R, C_PRN_R, C_OBS_HEAD,
        C_OPTIONAL_N
    };
    static const char *const route_name[C_OPTIONAL_N] = {
        "slide", "obs_along", "obs_cross", "obs_n", "clear",
        "pax_l", "pco_l", "prn_l", "pax_r", "pco_r", "prn_r", "obs_head"};
    // THE SAME BYTES MEAN OTHER THINGS ON A SEARCH (Explore.h, the flight
    // log section): the rolling search fills the observer/prediction block
    // with its cell, walls, votes and events. The banner it stamps begins
    // "search:", and that -- persisted with the log -- picks the names.
    static const char *const search_name[C_OPTIONAL_N] = {
        "slide", "corr", "lat", "ev", "clear",
        "cx", "hw", "wl", "cy", "dist", "wr", "fexp"};
    const char *const *opt_name = route_name;
    {
        char b[8];
        fram_read(FLOG_BANNER_ADDR, b, sizeof b);
        if (strncmp(b, "search:", 7) == 0) opt_name = search_name;
    }
    bool live[C_OPTIONAL_N];
    for (int c = 0; c < C_OPTIONAL_N; c++) live[c] = false;
    for (uint32_t i = 0; i < count; i++) {
        FlogRec r;
        fram_read(
            FLOG_REC_BASE + ((first + i) % FLOG_CAPACITY) * FLOG_REC_BYTES,
            &r, FLOG_REC_BYTES);
        if (r.slide_mm)      live[C_SLIDE] = true;
        if (r.obs_along_mm)  live[C_OBS_ALONG] = true;
        if (r.obs_cross_mm)  live[C_OBS_CROSS] = true;
        if (r.obs_n)         live[C_OBS_N] = true;
        if (r.clear_mm)      live[C_CLEAR] = true;
        if (r.pred_axis_l)   live[C_PAX_L] = true;
        if (r.pred_coord_l)  live[C_PCO_L] = true;
        if (r.pred_rng_l)    live[C_PRN_L] = true;
        if (r.pred_axis_r)   live[C_PAX_R] = true;
        if (r.pred_coord_r)  live[C_PCO_R] = true;
        if (r.pred_rng_r)    live[C_PRN_R] = true;
        if (r.obs_head_cdeg) live[C_OBS_HEAD] = true;
    }
    {
        int dropped = 0;
        for (int c = 0; c < C_OPTIONAL_N; c++) if (!live[c]) dropped++;
        if (dropped) {
            // Name them, so a reader who expects a column and does not find it
            // knows it was absent rather than lost. One line, whatever the
            // count -- the names are short and the buffer is 192.
            char nm[160];
            size_t o = 0;
            nm[0] = 0;
            for (int c = 0; c < C_OPTIONAL_N; c++) {
                if (live[c]) continue;
                int w = snprintf(nm + o, sizeof nm - o, "%s%s",
                                 o ? " " : "", opt_name[c]);
                if (w < 0 || (size_t)w >= sizeof nm - o) break;
                o += (size_t)w;
            }
            snprintf(line, sizeof line,
                     "OMITTED,%d all-zero columns,%s", dropped, nm);
            println_fn(line);
        }
    }

    // Header, built from the same `live` flags and in the same order as the
    // rows below -- `slide` sits between ax and flags in the record; the rest
    // follow omega.
    {
        size_t o = (size_t)snprintf(line, sizeof line,
                                    "t_ms,s,v,yaw,yaw_err,ay,ax");
        if (live[C_SLIDE])
            o += (size_t)snprintf(line + o, sizeof line - o, ",slide");
        o += (size_t)snprintf(line + o, sizeof line - o,
                              ",flags,L,FL,FR,R,wf,omega");
        for (int c = C_OBS_ALONG; c < C_OPTIONAL_N; c++)
            if (live[c])
                o += (size_t)snprintf(line + o, sizeof line - o, ",%s",
                                      opt_name[c]);
        println_fn(line);
    }

    for (uint32_t i = 0; i < count; i++) {
        FlogRec r;
        fram_read(
            FLOG_REC_BASE + ((first + i) % FLOG_CAPACITY) * FLOG_REC_BYTES,
            &r, FLOG_REC_BYTES);
        size_t o = (size_t)snprintf(line, sizeof line,
                                    "%u,%d,%d,%.1f,%.1f,%d,%d",
                                    r.t_ms, r.s_mm, r.v_mm_s,
                                    r.yaw_cdeg / 10.0f, r.yaw_err_cdeg / 10.0f,
                                    r.ay_mm_s2, r.ax_mm_s2);
        if (live[C_SLIDE])
            o += (size_t)snprintf(line + o, sizeof line - o, ",%d", r.slide_mm);
        o += (size_t)snprintf(line + o, sizeof line - o,
                              ",%u,%u,%u,%u,%u,%d,%d",
                              r.flags, r.left_mm, r.front_left_mm,
                              r.front_right_mm, r.right_mm, r.wheel_fused_mm,
                              r.omega_dps);
        if (live[C_OBS_ALONG])
            o += (size_t)snprintf(line + o, sizeof line - o, ",%d",
                                  r.obs_along_mm);
        if (live[C_OBS_CROSS])
            o += (size_t)snprintf(line + o, sizeof line - o, ",%d",
                                  r.obs_cross_mm);
        if (live[C_OBS_N])
            o += (size_t)snprintf(line + o, sizeof line - o, ",%u", r.obs_n);
        if (live[C_CLEAR])
            o += (size_t)snprintf(line + o, sizeof line - o, ",%d", r.clear_mm);
        if (live[C_PAX_L])
            o += (size_t)snprintf(line + o, sizeof line - o, ",%u",
                                  r.pred_axis_l);
        if (live[C_PCO_L])
            o += (size_t)snprintf(line + o, sizeof line - o, ",%d",
                                  r.pred_coord_l);
        if (live[C_PRN_L])
            o += (size_t)snprintf(line + o, sizeof line - o, ",%u",
                                  r.pred_rng_l);
        if (live[C_PAX_R])
            o += (size_t)snprintf(line + o, sizeof line - o, ",%u",
                                  r.pred_axis_r);
        if (live[C_PCO_R])
            o += (size_t)snprintf(line + o, sizeof line - o, ",%d",
                                  r.pred_coord_r);
        if (live[C_PRN_R])
            o += (size_t)snprintf(line + o, sizeof line - o, ",%u",
                                  r.pred_rng_r);
        if (live[C_OBS_HEAD])
            snprintf(line + o, sizeof line - o, ",%.2f",
                     r.obs_head_cdeg / 100.0f);
        paced(println_fn, line);  // per-line pace (see FLOG_LINE_PACE_MS)
    }
    println_fn("flight log: end");
}
