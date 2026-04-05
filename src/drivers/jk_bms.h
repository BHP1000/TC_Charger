#pragma once
#ifdef TEMP_NODE

#include <stdint.h>
#include <stdbool.h>

#define JK_BMS_CELLS       20
#define JK_BMS_DESIGN_MAH  73000UL

typedef struct {
    uint16_t cell_mv[JK_BMS_CELLS];
    uint16_t cell_avg_mv;
    uint16_t cell_diff_mv;
    int16_t  mos_temp_dc;
    uint32_t pack_mv;
    int32_t  current_ma;
    int16_t  bat_temp1_dc;
    int16_t  bat_temp2_dc;
    uint32_t alarm_flags;
    int16_t  balance_ma;
    uint8_t  balance_status;
    uint8_t  soc_pct;
    int32_t  remaining_mah;
    uint32_t full_charge_mah;
    uint32_t cycle_count;
    uint8_t  soh_pct;
    uint32_t run_time_s;
    uint8_t  charge_en;
    uint8_t  discharge_en;
    uint8_t  charger_plugged;
    uint32_t design_cap_mah;
    bool     valid;
} JKBMSData_t;

void jk_bms_init(void);
void jk_bms_update(void);
bool jk_bms_get(JKBMSData_t *out);

#endif // TEMP_NODE
