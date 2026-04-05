#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "rs485_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define THERMAL_LUT_SIZE    16

typedef struct {
    float    temp_c;
    uint16_t max_current_da;
} ThermalLUTEntry_t;

void               thermal_logic_init(void);
void               thermal_logic_update(void);
uint16_t           thermal_get_max_current_da(float max_temp_c);
bool               thermal_is_overtemp(float temp_c);
bool               thermal_is_undertemp(float temp_c);
float              thermal_get_max_ntc_temp(void);
const TempNodeData_t *thermal_get_last_data(void);

#ifdef __cplusplus
}
#endif
