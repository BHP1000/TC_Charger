#include "ntc_array.h"
#ifdef TEMP_NODE

#include <Arduino.h>
#include <math.h>

// GPIO0-4 = ADC channels on ESP32-C3 Super Mini; 255 = no pin (ch5 unused)
static const uint8_t  s_adc_pins[NTC_COUNT]    = {0, 1, 2, 3, 4, 255};

// Measured series resistor values for each channel (Ohms)
// ch0=T1: 98.3K  ch1=T2: 98.5K  ch2=T3: 98.9K  ch3=T4: 98.9K  ch4=T5: 98.9K  ch5: unused
static const uint32_t s_series_ohms[NTC_COUNT] = {98300, 98500, 98900, 98900, 98900, 98900};

static float s_cal_offset[NTC_COUNT] = {0};
static float s_smoothed[NTC_COUNT];
static NTCArray_t s_ntc;
static bool s_initialized = false;

static float adc_raw_to_ohms(uint32_t raw_sum, uint8_t samples, uint8_t ch) {
    float adc_max = (float)((1 << NTC_ADC_BITS) - 1);
    float ratio   = (float)raw_sum / ((float)samples * adc_max);
    if (ratio <= 0.0f || ratio >= 1.0f) return -1.0f;
    return (float)s_series_ohms[ch] * ratio / (1.0f - ratio);
}

static float ohms_to_celsius(float r) {
    if (r <= 0.0f) return NTC_FAULT_LOW_C - 1.0f;
    float inv = (1.0f / (NTC_NOMINAL_TEMP_C + 273.15f)) +
                (1.0f / (float)NTC_BETA) *
                logf(r / (float)NTC_NOMINAL_OHMS);
    if (inv == 0.0f) return NTC_FAULT_LOW_C - 1.0f;
    return (1.0f / inv) - 273.15f;
}

void ntc_array_init(void) {
    analogReadResolution(NTC_ADC_BITS);
    for (uint8_t i = 0; i < NTC_COUNT; i++) {
        if (s_adc_pins[i] == 255) {
            s_smoothed[i]     = 25.0f;
            s_ntc.temp_c[i]   = 0.0f;
            s_ntc.raw_ohms[i] = -1.0f;
            s_ntc.valid[i]    = false;
            continue;
        }
        pinMode(s_adc_pins[i], INPUT);
        uint32_t sum = 0;
        for (uint8_t s = 0; s < NTC_OVERSAMPLE; s++) sum += analogRead(s_adc_pins[i]);
        float r = adc_raw_to_ohms(sum, NTC_OVERSAMPLE, i);
        float t = ohms_to_celsius(r) + s_cal_offset[i];
        s_smoothed[i]     = (t > NTC_FAULT_LOW_C && t < NTC_FAULT_HIGH_C) ? t : 25.0f;
        s_ntc.temp_c[i]   = s_smoothed[i];
        s_ntc.raw_ohms[i] = r;
        s_ntc.valid[i]    = false;
    }
    s_initialized = true;
    Serial.println("[NTC] init OK — ch0-4 active, ch5 not wired (Super Mini has 5 ADC pins)");
}

void ntc_array_update(void) {
    if (!s_initialized) return;
    s_ntc.max_temp_c  = NTC_FAULT_LOW_C;
    s_ntc.min_temp_c  = NTC_FAULT_HIGH_C;
    s_ntc.valid_count = 0;

    for (uint8_t i = 0; i < NTC_COUNT; i++) {
        if (s_adc_pins[i] == 255) {
            s_ntc.raw_ohms[i] = -1.0f;
            s_ntc.valid[i]    = false;
            s_ntc.temp_c[i]   = 0.0f;
            continue;
        }
        uint32_t sum = 0;
        for (uint8_t s = 0; s < NTC_OVERSAMPLE; s++) {
            sum += analogRead(s_adc_pins[i]);
            delayMicroseconds(50);
        }
        float r = adc_raw_to_ohms(sum, NTC_OVERSAMPLE, i);
        float t = ohms_to_celsius(r) + s_cal_offset[i];

        s_ntc.raw_ohms[i] = r;
        s_ntc.valid[i]    = (t > NTC_FAULT_LOW_C && t < NTC_FAULT_HIGH_C);

        if (s_ntc.valid[i]) {
            s_smoothed[i] = s_smoothed[i] +
                            NTC_SMOOTH_ALPHA * (t - s_smoothed[i]);
            s_ntc.temp_c[i] = s_smoothed[i];
            s_ntc.valid_count++;
            if (s_smoothed[i] > s_ntc.max_temp_c) s_ntc.max_temp_c = s_smoothed[i];
            if (s_smoothed[i] < s_ntc.min_temp_c) s_ntc.min_temp_c = s_smoothed[i];
        } else {
            s_ntc.temp_c[i] = 0.0f;
        }
    }
    if (s_ntc.valid_count == 0) {
        s_ntc.max_temp_c = 0.0f;
        s_ntc.min_temp_c = 0.0f;
    }
}

bool ntc_array_get(NTCArray_t *out) {
    if (!out || !s_initialized) return false;
    *out = s_ntc;
    return true;
}

void ntc_array_set_cal_offset(uint8_t ch, float offset_c) {
    if (ch < NTC_COUNT) s_cal_offset[ch] = offset_c;
}

#endif // TEMP_NODE
