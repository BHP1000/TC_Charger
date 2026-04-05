#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RGB_STATE_IDLE = 0,
    RGB_STATE_CHARGING,
    RGB_STATE_OVERTEMP,
    RGB_STATE_UNDERTEMP,
    RGB_STATE_BMS_FAULT,
    RGB_STATE_COMM_TIMEOUT,
    RGB_STATE_FAULT,
    RGB_STATE_BOOT,
} RGBState_t;

void rgb_led_init(void);
void rgb_led_set_state(RGBState_t state);
void rgb_led_update(void);

#ifdef __cplusplus
}
#endif
