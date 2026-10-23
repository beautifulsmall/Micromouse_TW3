#include "Vbat.h"

#include "Console.h"

static float filt_mv = 0;
static int last_raw = 0;
static uint32_t last_ms = 0;
static bool primed = false;

void vbat_init() {
    pinMode(VBAT_PIN, INPUT);
    // Prime the filter so the first LAPREP after boot carries a real value
    // rather than a ramp from zero.
    last_raw = analogRead(VBAT_PIN);
    filt_mv = (float)last_raw / VBAT_ADC_MAX * VBAT_ADC_REF_MV * VBAT_DIVIDER;
    primed = true;
    last_ms = millis();
}

void vbat_poll() {
    if (!primed || millis() - last_ms < VBAT_PERIOD_MS) return;
    last_ms = millis();
    last_raw = analogRead(VBAT_PIN);
    float mv =
        (float)last_raw / VBAT_ADC_MAX * VBAT_ADC_REF_MV * VBAT_DIVIDER;
    filt_mv += VBAT_EMA * (mv - filt_mv);
}

uint16_t vbat_mv() { return (uint16_t)(filt_mv + 0.5f); }

void vbat_print() {
    con_printf("vbat: raw=%d  pin=%.0f mV  pack=%.2f V  (divider %.3f -- "
               "calibrate: meter_mV / pin_mV)\n",
               last_raw,
               (double)((float)last_raw / VBAT_ADC_MAX * VBAT_ADC_REF_MV),
               (double)(filt_mv / 1000.0f), (double)VBAT_DIVIDER);
}
