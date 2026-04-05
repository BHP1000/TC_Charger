#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef MAIN_BRAIN

// SH1106 1.3" 128x64 OLED — I2C
// ESP32-S3: GPIO8 = SDA, GPIO9 = SCL
#define OLED_SDA_PIN    8
#define OLED_SCL_PIN    9
#define OLED_I2C_ADDR   0x3C    // alt: 0x3D if OLED SA0 pin is HIGH

#ifdef __cplusplus
extern "C" {
#endif

void oled_display_init(void);
void oled_display_update(void);
void oled_display_next_page(void);

#ifdef __cplusplus
}
#endif

#endif // MAIN_BRAIN
