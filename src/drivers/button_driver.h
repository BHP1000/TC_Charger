#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef MAIN_BRAIN

#define BTN_NEXT_PIN    6       // S3 GPIO6 — cycle to next page
#define BTN_HOLD_PIN    7       // S3 GPIO7 — toggle auto-scroll hold
#define BTN_DEBOUNCE_MS 50UL

#ifdef __cplusplus
extern "C" {
#endif

void button_driver_init(void);
void button_driver_update(void);
bool button_next_pressed(void);
bool button_hold_active(void);

#ifdef __cplusplus
}
#endif

#endif // MAIN_BRAIN
