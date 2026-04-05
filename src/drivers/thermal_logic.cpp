#include "thermal_logic.h"
#ifdef MAIN_BRAIN

#include <Arduino.h>
#include "rs485_driver.h"

#define OVERTEMP_CUTOFF_C    55.0f
#define UNDERTEMP_INHIBIT_C   0.0f
#define THERMAL_POLL_MS      500UL

static const ThermalLUTEntry_t s_lut[THERMAL_LUT_SIZE] = {
    {  0.0f,   0  },
    {  5.0f,  50  },
    { 10.0f, 100  },
    { 15.0f, 150  },
    { 20.0f, 200  },
    { 25.0f, 200  },
    { 30.0f, 180  },
    { 35.0f, 160  },
    { 40.0f, 130  },
    { 42.0f, 100  },
    { 44.0f,  80  },
    { 46.0f,  60  },
    { 48.0f,  40  },
    { 50.0f,  20  },
    { 52.0f,  10  },
    { 55.0f,   0  },
};

static TempNodeData_t s_last_data;
static bool           s_data_valid    = false;
static uint32_t       s_last_poll_ms  = 0;

void thermal_logic_init(void) {
    s_data_valid = false;
    Serial.println("[THERMAL] init OK");
}

void thermal_logic_update(void) {
    uint32_t now = millis();
    if (now - s_last_poll_ms < THERMAL_POLL_MS) return;
    s_last_poll_ms = now;

    TempNodeData_t d;
    if (rs485_request_temp_node(&d)) {
        s_last_data  = d;
        s_data_valid = true;
    } else {
        s_data_valid = false;
    }
}

uint16_t thermal_get_max_current_da(float max_temp_c) {
    if (max_temp_c >= OVERTEMP_CUTOFF_C)  return 0;
    if (max_temp_c <= UNDERTEMP_INHIBIT_C) return 0;
    for (int i = THERMAL_LUT_SIZE - 1; i >= 0; i--) {
        if (max_temp_c >= s_lut[i].temp_c) return s_lut[i].max_current_da;
    }
    return 0;
}

bool thermal_is_overtemp(float temp_c)  { return temp_c >= OVERTEMP_CUTOFF_C; }
bool thermal_is_undertemp(float temp_c) { return temp_c <= UNDERTEMP_INHIBIT_C; }

float thermal_get_max_ntc_temp(void) {
    if (!s_data_valid) return -999.0f;
    float mx = -999.0f;
    for (int i = 0; i < 6; i++) {
        if (s_last_data.valid_mask & (1 << i)) {
            if (s_last_data.temp_c[i] > mx) mx = s_last_data.temp_c[i];
        }
    }
    return mx;
}

const TempNodeData_t *thermal_get_last_data(void) {
    return s_data_valid ? &s_last_data : nullptr;
}

#endif // MAIN_BRAIN
