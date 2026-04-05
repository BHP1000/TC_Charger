#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NTC_COUNT            6
#define NTC_NOMINAL_OHMS     100000
#define NTC_NOMINAL_TEMP_C   25.0f
#define NTC_BETA             3950
// Note: NTC series resistance is per-channel (see ntc_array.cpp)
#define NTC_ADC_BITS         12
#define NTC_OVERSAMPLE       8
#define NTC_SMOOTH_ALPHA     0.15f
#define NTC_FAULT_LOW_C     -50.0f
#define NTC_FAULT_HIGH_C     150.0f

typedef struct {
    float    temp_c[NTC_COUNT];
    float    raw_ohms[NTC_COUNT];
    bool     valid[NTC_COUNT];
    float    max_temp_c;
    float    min_temp_c;
    uint8_t  valid_count;
} NTCArray_t;

void ntc_array_init(void);
void ntc_array_update(void);
bool ntc_array_get(NTCArray_t *out);
void ntc_array_set_cal_offset(uint8_t ch, float offset_c);

#ifdef __cplusplus
}
#endif
