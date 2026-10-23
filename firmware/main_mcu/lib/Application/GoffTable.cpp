#include "GoffTable.h"
#include "HAL.h"

#include <math.h>
#include <string.h>

#include "Console.h"
#include "Drivers.h"  // fram_read / fram_write / fram_ready
#include "Params.h"   // params_crc32

// RAM copy. Sparse, append-only within a session; order is stable so the
// stored block is deterministic.
struct GoffEntry {
    uint8_t cx, cy, h;
    int8_t off_mm;    // learned (the glearn loop's layer)
    int8_t trim_mm;   // hand-set arc trim (GoffTable.h)
};
struct GoffHeader {
    uint32_t magic;
    uint8_t version;
    uint8_t n;
    uint16_t geom_rev;   // GOFF_TABLE_GEOM_REV that measured these entries
};
static_assert(sizeof(GoffEntry) == 5, "GoffEntry must stay 5 B");
static_assert(sizeof(GoffHeader) == 8, "GoffHeader must stay 8 B");
static_assert(sizeof(GoffHeader) + GOFF_TABLE_MAX * sizeof(GoffEntry) + 4 <=
                  GOFF_TABLE_FRAM_END - GOFF_TABLE_FRAM_ADDR,
              "goff table outgrew its FRAM window -- move it");

static GoffEntry tab[GOFF_TABLE_MAX];
static int n_entries = 0;

static int find(uint8_t cx, uint8_t cy, uint8_t h) {
    for (int i = 0; i < n_entries; i++)
        if (tab[i].cx == cx && tab[i].cy == cy && tab[i].h == h) return i;
    return -1;
}

static int8_t clamp_off(float v) {
    if (v > GOFF_TABLE_MAX_MM) return (int8_t)GOFF_TABLE_MAX_MM;
    if (v < -GOFF_TABLE_MAX_MM) return (int8_t)-GOFF_TABLE_MAX_MM;
    return (int8_t)lroundf(v);
}

static uint32_t table_crc(const GoffHeader &hd) {
    uint32_t c = params_crc32((const uint8_t *)&hd, sizeof hd);
    if (hd.n > 0)
        c ^= params_crc32((const uint8_t *)tab, hd.n * sizeof(GoffEntry));
    return c;
}

void goff_table_load() {
    n_entries = 0;
    GoffHeader hd;
    fram_read(GOFF_TABLE_FRAM_ADDR, (uint8_t *)&hd, sizeof hd);
    if (hd.magic == GOFF_TABLE_MAGIC && hd.version != GOFF_TABLE_VERSION) {
        con_printf("goff table: stored format v%u, firmware v%u -- starting "
                   "empty\n", hd.version, GOFF_TABLE_VERSION);
        return;
    }
    if (hd.magic != GOFF_TABLE_MAGIC || hd.n > GOFF_TABLE_MAX)
        return;  // virgin FRAM: start empty, silently
    if (hd.geom_rev != GOFF_TABLE_GEOM_REV) {
        // The firmware's gate geometry changed since these entries were
        // measured -- they describe a formula that no longer exists.
        con_printf("goff table: stored entries are geometry rev %u, firmware "
                   "is rev %u -- starting empty (relearns in 2-3 laps)\n",
                   hd.geom_rev, GOFF_TABLE_GEOM_REV);
        return;
    }
    fram_read(GOFF_TABLE_FRAM_ADDR + sizeof hd, (uint8_t *)tab,
              hd.n * sizeof(GoffEntry));
    uint32_t crc;
    fram_read(GOFF_TABLE_FRAM_ADDR + sizeof hd +
                  (uint32_t)hd.n * sizeof(GoffEntry),
              (uint8_t *)&crc, sizeof crc);
    // Torn write must read as "no table", never as plausible offsets --
    // same discipline as the params and lap-report blocks.
    if (crc != table_crc(hd)) {
        con_println("goff table: stored block failed CRC -- starting empty");
        return;
    }
    n_entries = hd.n;
    if (n_entries > 0)
        con_printf("goff table: %d learned corner(s) restored\n", n_entries);
}

float goff_table_mm(uint8_t cx, uint8_t cy, uint8_t h) {
    int i = find(cx, cy, h);
    return i < 0 ? 0.0f : (float)tab[i].off_mm;
}

static bool upsert(uint8_t cx, uint8_t cy, uint8_t h, float off) {
    int i = find(cx, cy, h);
    if (i < 0) {
        if (n_entries >= GOFF_TABLE_MAX) {
            con_printf("goff table FULL (%d) -- corner (%u,%u) h%u not "
                       "recorded\n", GOFF_TABLE_MAX, cx, cy, h);
            return false;
        }
        i = n_entries++;
        tab[i].cx = cx;
        tab[i].cy = cy;
        tab[i].h = h;
        tab[i].off_mm = 0;
        tab[i].trim_mm = 0;
    }
    tab[i].off_mm = clamp_off(off);
    return true;
}

float goff_table_trim_mm(uint8_t cx, uint8_t cy, uint8_t h) {
    int i = find(cx, cy, h);
    return i < 0 ? 0.0f : (float)tab[i].trim_mm;
}

bool goff_table_set_trim(uint8_t cx, uint8_t cy, uint8_t h, float trim_mm) {
    int i = find(cx, cy, h);
    if (i < 0) {
        if (!upsert(cx, cy, h, 0.0f)) return false;
        i = find(cx, cy, h);
    }
    tab[i].trim_mm = clamp_off(trim_mm);
    return true;
}

bool goff_table_learn(uint8_t cx, uint8_t cy, uint8_t h, float corr_mm,
                      float alpha) {
    if (alpha <= 0.0f) return true;  // learning off: a no-op, not an error
    // corr = expect - drop. Edge later than expected (corr < 0) means the
    // expectation must GROW, so the update is the sign-flipped residual.
    float d = alpha * -corr_mm;
    // An entry that already exists is presumed near-converged: slew it, so
    // a single bad lock cannot yank it (see GOFF_TABLE_STEP_MM). A new
    // corner takes the full step -- there is nothing yet to protect.
    if (find(cx, cy, h) >= 0) {
        if (d > GOFF_TABLE_STEP_MM) d = GOFF_TABLE_STEP_MM;
        if (d < -GOFF_TABLE_STEP_MM) d = -GOFF_TABLE_STEP_MM;
    }
    return upsert(cx, cy, h, goff_table_mm(cx, cy, h) + d);
}

bool goff_table_set(uint8_t cx, uint8_t cy, uint8_t h, float off_mm) {
    return upsert(cx, cy, h, off_mm);
}

void goff_table_save() {
    if (!fram_ready()) return;
    GoffHeader hd;
    memset(&hd, 0, sizeof hd);
    hd.magic = GOFF_TABLE_MAGIC;
    hd.version = GOFF_TABLE_VERSION;
    hd.geom_rev = GOFF_TABLE_GEOM_REV;
    hd.n = (uint8_t)n_entries;
    uint32_t crc = table_crc(hd);
    fram_write(GOFF_TABLE_FRAM_ADDR, (const uint8_t *)&hd, sizeof hd);
    if (n_entries > 0)
        fram_write(GOFF_TABLE_FRAM_ADDR + sizeof hd, (const uint8_t *)tab,
                   n_entries * sizeof(GoffEntry));
    fram_write(GOFF_TABLE_FRAM_ADDR + sizeof hd +
                   (uint32_t)n_entries * sizeof(GoffEntry),
               (const uint8_t *)&crc, sizeof crc);
}

void goff_table_clear() {
    // Keep every entry that carries a trim, with its offset zeroed.
    int kept = 0;
    for (int i = 0; i < n_entries; i++) {
        if (tab[i].trim_mm == 0) continue;
        tab[kept] = tab[i];
        tab[kept].off_mm = 0;
        kept++;
    }
    n_entries = kept;
    goff_table_save();
    if (kept)
        con_printf("goff table cleared (learned offsets); %d hand trim%s "
                   "kept -- 'goffc.clear all' drops those too\n",
                   kept, kept == 1 ? "" : "s");
    else
        con_println("goff table cleared");
}

void goff_table_clear_all() {
    n_entries = 0;
    goff_table_save();
    con_println("goff table cleared, trims included");
}

void goff_table_print() {
    if (n_entries == 0) {
        con_println("goff table: empty");
        return;
    }
    static const char HN[4] = {'N', 'E', 'S', 'W'};
    for (int i = 0; i < n_entries; i++) {
        hal_radio_wait_room(96, 20);
        // h with bit 2 set is a V's learned TIMING (DiagLane.h GOFF_KEY_V),
        // not a gate offset: printed with a trailing 'v'.
        con_printf("GOFFC,cell=%u:%u,h=%c%s,off=%+d,trim=%+d\n", tab[i].cx,
                   tab[i].cy, HN[tab[i].h & 3], (tab[i].h & 4) ? "v" : "",
                   (int)tab[i].off_mm, (int)tab[i].trim_mm);
    }
    con_printf("goff table: %d entr%s of %d\n", n_entries,
               n_entries == 1 ? "y" : "ies", GOFF_TABLE_MAX);
}

int goff_table_count() { return n_entries; }
