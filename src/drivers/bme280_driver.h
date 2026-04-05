#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float temperature_c;
    float humidity_pct;
    float pressure_hpa;
    bool  valid;
} BME280Data_t;

#if defined(MAIN_BRAIN) || defined(TEMP_NODE)

void bme280_init(void);
void bme280_update(void);
bool bme280_get(BME280Data_t *out);

#endif

#ifdef __cplusplus
}
#endif
