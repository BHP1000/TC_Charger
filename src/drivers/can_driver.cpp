#include "can_driver.h"
#ifdef MAIN_BRAIN

#include <Arduino.h>
#include "driver/twai.h"

// ---- Module state ----
static bool            s_initialized      = false;
static ChargerCmd_t    s_last_cmd         = {0, 0, CAN_CTRL_DISABLE, CAN_MODE_CHARGE};
static ChargerStatus_t s_charger_status   = {};
static uint32_t        s_last_tx_ms       = 0;
static uint32_t        s_last_rx_ms       = 0;

// ---- Build and transmit a TX frame ----
static bool twai_send_frame(const ChargerCmd_t *cmd) {
    twai_message_t msg = {};
    msg.extd       = 1;
    msg.identifier = CAN_CHARGER_TX_ID;
    msg.data_length_code = 8;

    // Motorola (big-endian): MSB first
    msg.data[0] = (cmd->voltage_dv >> 8) & 0xFF;
    msg.data[1] =  cmd->voltage_dv        & 0xFF;
    msg.data[2] = (cmd->current_da >> 8)  & 0xFF;
    msg.data[3] =  cmd->current_da        & 0xFF;
    msg.data[4] =  cmd->control;
    msg.data[5] =  cmd->mode;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        Serial.printf("[CAN] TX error: %d\n", err);
        return false;
    }
    s_last_tx_ms = millis();
    return true;
}

// ---- Parse RX frame from charger ----
static void parse_rx_frame(const twai_message_t *msg) {
    if (msg->data_length_code < 8) return;

    uint16_t v_raw = ((uint16_t)msg->data[0] << 8) | msg->data[1];
    uint16_t i_raw = ((uint16_t)msg->data[2] << 8) | msg->data[3];
    uint8_t  fault = msg->data[4];
    uint8_t  stat  = msg->data[5];
    uint8_t  temp  = msg->data[7];

    s_charger_status.output_voltage_dv = v_raw;
    s_charger_status.output_current_da = i_raw;
    s_charger_status.temperature_c     = (int8_t)((int16_t)temp - 40);

    s_charger_status.fault.hardware_fault         = (fault >> 0) & 0x01;
    s_charger_status.fault.temperature_fault      = (fault >> 1) & 0x01;
    s_charger_status.fault.input_voltage_status   = (fault >> 2) & 0x03;
    s_charger_status.fault.output_undervoltage    = (fault >> 4) & 0x01;
    s_charger_status.fault.output_overvoltage     = (fault >> 5) & 0x01;
    s_charger_status.fault.output_overcurrent     = (fault >> 6) & 0x01;
    s_charger_status.fault.output_short_circuit   = (fault >> 7) & 0x01;

    s_charger_status.status.comm_timeout      = (stat >> 0) & 0x01;
    s_charger_status.status.working_status    = (stat >> 1) & 0x03;
    s_charger_status.status.init_complete     = (stat >> 3) & 0x01;
    s_charger_status.status.fan_on            = (stat >> 4) & 0x01;
    s_charger_status.status.cooling_pump_on   = (stat >> 5) & 0x01;

    s_charger_status.valid = true;
    s_last_rx_ms = millis();

    Serial.printf("[CAN] RX: %.1fV %.1fA fault=0x%02X stat=0x%02X temp=%d°C\n",
                  v_raw * 0.1f, i_raw * 0.1f, fault, stat, s_charger_status.temperature_c);
}

// ---- Public API ----

void can_driver_init(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_GPIO, (gpio_num_t)CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_250KBITS();
    // Accept only our charger's RX frame ID
    twai_filter_config_t  f_config = {
        .acceptance_code = (CAN_CHARGER_RX_ID << 3),
        .acceptance_mask = ~((CAN_CHARGER_RX_ID << 3)),
        .single_filter   = true,
    };

    esp_err_t err;
    err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("[CAN] install error %d, using accept-all filter\n", err);
        twai_filter_config_t f_all = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        err = twai_driver_install(&g_config, &t_config, &f_all);
    }
    if (err != ESP_OK) { Serial.printf("[CAN] driver install failed: %d\n", err); return; }

    err = twai_start();
    if (err != ESP_OK) { Serial.printf("[CAN] start failed: %d\n", err); return; }

    s_initialized = true;
    s_last_tx_ms  = millis();
    s_last_rx_ms  = millis();
    Serial.printf("[CAN] init OK — TX GPIO%d RX GPIO%d @ 250kbps\n", CAN_TX_GPIO, CAN_RX_GPIO);
}

bool can_send_disable(void) {
    s_last_cmd = {0, 0, CAN_CTRL_DISABLE, CAN_MODE_CHARGE};
    if (!s_initialized) return false;
    return twai_send_frame(&s_last_cmd);
}

bool can_send_sleep(void) {
    s_last_cmd = {0, 0, CAN_CTRL_SLEEP, CAN_MODE_CHARGE};
    if (!s_initialized) return false;
    return twai_send_frame(&s_last_cmd);
}

bool can_send_command(const ChargerCmd_t *cmd) {
    if (!cmd) return false;
    s_last_cmd = *cmd;
    if (!s_initialized) return false;
    return twai_send_frame(cmd);
}

bool can_read_status(ChargerStatus_t *out) {
    if (!out) return false;
    // Mark stale if no RX for >5s
    if ((millis() - s_last_rx_ms) > CAN_CHARGER_TIMEOUT_MS) {
        s_charger_status.valid = false;
    }
    *out = s_charger_status;
    return s_charger_status.valid;
}

void can_driver_update(void) {
    if (!s_initialized) return;

    // TX heartbeat — must send at least every CAN_CHARGER_TIMEOUT_MS or charger disables
    if ((millis() - s_last_tx_ms) >= CAN_TX_INTERVAL_MS) {
        twai_send_frame(&s_last_cmd);
    }

    // Drain RX queue
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        if (msg.extd && msg.identifier == CAN_CHARGER_RX_ID) {
            parse_rx_frame(&msg);
        }
    }
}

#endif // MAIN_BRAIN
