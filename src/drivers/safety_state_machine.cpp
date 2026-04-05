#include "safety_state_machine.h"
#ifdef MAIN_BRAIN

#include <Arduino.h>
#include "can_driver.h"
#include "thermal_logic.h"
#include "rgb_led.h"
#include "ble_server.h"

#define COMM_TIMEOUT_MS    3000UL

static SafetyState_t s_state       = SAFETY_STATE_INIT;
static bool          s_bms_fault   = false;
static bool          s_comm_to     = false;
static uint32_t      s_last_rs485_ok_ms = 0;

const char *safety_state_name(SafetyState_t s) {
    switch (s) {
        case SAFETY_STATE_INIT:         return "INIT";
        case SAFETY_STATE_IDLE:         return "IDLE";
        case SAFETY_STATE_CHARGING:     return "CHARGING";
        case SAFETY_STATE_OVERTEMP:     return "OVERTEMP";
        case SAFETY_STATE_UNDERTEMP:    return "UNDERTEMP";
        case SAFETY_STATE_BMS_FAULT:    return "BMS_FAULT";
        case SAFETY_STATE_COMM_TIMEOUT: return "COMM_TIMEOUT";
        case SAFETY_STATE_FAULT:        return "FAULT";
        default:                        return "UNKNOWN";
    }
}

static RGBState_t safety_to_rgb(SafetyState_t s) {
    switch (s) {
        case SAFETY_STATE_CHARGING:     return RGB_STATE_CHARGING;
        case SAFETY_STATE_OVERTEMP:     return RGB_STATE_OVERTEMP;
        case SAFETY_STATE_UNDERTEMP:    return RGB_STATE_UNDERTEMP;
        case SAFETY_STATE_BMS_FAULT:    return RGB_STATE_BMS_FAULT;
        case SAFETY_STATE_COMM_TIMEOUT: return RGB_STATE_COMM_TIMEOUT;
        case SAFETY_STATE_FAULT:        return RGB_STATE_FAULT;
        default:                        return RGB_STATE_IDLE;
    }
}

void safety_sm_init(void) {
    s_state            = SAFETY_STATE_IDLE;
    s_last_rs485_ok_ms = millis();
    can_send_disable();
    rgb_led_set_state(RGB_STATE_IDLE);
    Serial.println("[SAFETY] init -> IDLE, charger disabled");
}

void safety_sm_update(void) {
    SafetyState_t prev = s_state;

    // Auto comm-timeout: if thermal_get_last_data returns non-null, record last ok
    const TempNodeData_t *td = thermal_get_last_data();
    if (td != nullptr) {
        s_last_rs485_ok_ms = millis();
        s_comm_to = false;
    } else if ((millis() - s_last_rs485_ok_ms) > COMM_TIMEOUT_MS) {
        s_comm_to = true;
    }

    if (s_bms_fault) {
        s_state = SAFETY_STATE_BMS_FAULT;
        can_send_disable();
    } else if (s_comm_to) {
        s_state = SAFETY_STATE_COMM_TIMEOUT;
        can_send_disable();
    } else if (td != nullptr) {
        float max_t = thermal_get_max_ntc_temp();

        if (thermal_is_overtemp(max_t)) {
            s_state = SAFETY_STATE_OVERTEMP;
            can_send_disable();
        } else if (thermal_is_undertemp(max_t)) {
            s_state = SAFETY_STATE_UNDERTEMP;
            can_send_disable();
        } else {
            uint16_t cur_da = thermal_get_max_current_da(max_t);
            if (cur_da > 0) {
                ChargerCmd_t cmd = {
                    .voltage_dv = ble_server_get_pack_voltage_dv(),
                    .current_da = cur_da,
                    .control    = CAN_CTRL_CHARGE,
                    .mode       = CAN_MODE_CHARGE,
                };
                can_send_command(&cmd);
                s_state = SAFETY_STATE_CHARGING;
            } else {
                can_send_disable();
                s_state = SAFETY_STATE_IDLE;
            }
        }
    }

    if (s_state != prev) {
        Serial.printf("[SAFETY] %s -> %s\n", safety_state_name(prev), safety_state_name(s_state));
        rgb_led_set_state(safety_to_rgb(s_state));
    }
}

SafetyState_t safety_sm_get_state(void) { return s_state; }

void safety_sm_report_bms_fault(bool fault)      { s_bms_fault = fault; }
void safety_sm_report_comm_timeout(bool timeout) { s_comm_to   = timeout; }

#endif // MAIN_BRAIN
