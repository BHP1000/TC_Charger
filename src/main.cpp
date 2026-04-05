#include <Arduino.h>
#include <esp_task_wdt.h>

#define WDT_TIMEOUT_S 5

#if defined(MAIN_BRAIN)
#include "drivers/can_driver.h"
#include "drivers/rs485_driver.h"
#include "drivers/thermal_logic.h"
#include "drivers/safety_state_machine.h"
#include "drivers/rgb_led.h"
#include "drivers/ble_server.h"
#include "drivers/oled_display.h"
#include "drivers/bme280_driver.h"
#include "drivers/gps_driver.h"
#include "drivers/button_driver.h"

void setup() {
    Serial.begin(115200);
    Serial.println("[MAIN_BRAIN] Boot");
    esp_task_wdt_config_t wdt_cfg = { .timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true };
    esp_task_wdt_reconfigure(&wdt_cfg);
    esp_task_wdt_add(NULL);
    rgb_led_init();
    esp_task_wdt_reset();
    button_driver_init();
    esp_task_wdt_reset();
    ble_server_init();
    esp_task_wdt_reset();
    rs485_init();
    esp_task_wdt_reset();
    can_driver_init();
    esp_task_wdt_reset();
    thermal_logic_init();
    esp_task_wdt_reset();
    safety_sm_init();
    esp_task_wdt_reset();
    oled_display_init();
    esp_task_wdt_reset();
    bme280_init();
    esp_task_wdt_reset();
    gps_driver_init();
    esp_task_wdt_reset();
}

void loop() {
    esp_task_wdt_reset();
    button_driver_update();
    safety_sm_update();
    rs485_poll();
    thermal_logic_update();
    rgb_led_update();
    ble_server_update();
    can_driver_update();
    oled_display_update();
    bme280_update();
    gps_driver_update();
}

#elif defined(TEMP_NODE)
#include "drivers/bme280_driver.h"
#include "drivers/ntc_array.h"
#include "drivers/rs485_driver.h"
#include "drivers/jk_bms.h"

void setup() {
    Serial.begin(115200);
    Serial.println("[TEMP_NODE] Boot");
    esp_task_wdt_config_t wdt_cfg = { .timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true };
    esp_task_wdt_reconfigure(&wdt_cfg);
    esp_task_wdt_add(NULL);
    bme280_init();
    ntc_array_init();
    jk_bms_init();
    rs485_init();
}

void loop() {
    esp_task_wdt_reset();
    ntc_array_update();
    bme280_update();
    jk_bms_update();
    rs485_respond();
}

#else
#error "No build target defined. Set -DMAIN_BRAIN or -DTEMP_NODE in build_flags."
#endif
