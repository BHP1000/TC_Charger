#ifdef MAIN_BRAIN
#include <Arduino.h>
#include "driver/twai.h"

// Phase 1.5 — TC OBC CAN Bench Test
// PURPOSE: verify TWAI init, send Disable frames, decode charger responses.
// SAFE: control byte = 0x01 (CAN_CTRL_DISABLE). Charger output stays OFF.
//
// TC OBC Protocol:
//   TX  ID = 0x1806E5F4  (BMS → Charger), 8 bytes, Motorola/big-endian
//     [0-1] = max voltage  × 0.1 V
//     [2-3] = max current  × 0.1 A
//     [4]   = control  0x00=charge  0x01=disable  0x02=sleep
//     [5]   = mode     0x00=charge  0x01=heat
//     [6-7] = reserved 0x00
//   RX  ID = 0x18FF50E5  (Charger → BMS), 8 bytes
//     [0-1] = output voltage × 0.1 V
//     [2-3] = output current × 0.1 A
//     [4]   = fault flags (bit0=HW, bit1=OTP, bit2=OVP, bit3=OCP)
//     [5]   = status flags (bit0=charging, bit1=full, bit2=idle)
//     [6]   = CC/CP/lock
//     [7]   = temperature (raw − 40 = °C)

// Match GPIO pins used in can_driver.cpp
#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5

#define CHARGER_TX_ID   0x1806E5F4UL
#define CHARGER_RX_ID   0x18FF50E5UL

static bool twai_ok = false;

static void send_disable_frame(void) {
    twai_message_t msg = {};
    msg.identifier       = CHARGER_TX_ID;
    msg.extd             = 1;
    msg.data_length_code = 8;
    // Bytes 0-3: voltage = 0, current = 0 (safe bench — charger already disabled)
    msg.data[0] = 0x00;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    msg.data[4] = 0x01;   // CAN_CTRL_DISABLE
    msg.data[5] = 0x00;   // CAN_MODE_CHARGE
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (err == ESP_OK)
        Serial.println("[CAN_PING] TX disable: 00 00 00 00 01 00 00 00");
    else
        Serial.printf("[CAN_PING] TX error: 0x%X\n", err);
}

static void decode_rx_frame(const twai_message_t *msg) {
    if (msg->data_length_code < 8) {
        Serial.printf("[CAN_RX] short frame dlc=%d\n", msg->data_length_code);
        return;
    }
    uint16_t v_raw  = ((uint16_t)msg->data[0] << 8) | msg->data[1];
    uint16_t i_raw  = ((uint16_t)msg->data[2] << 8) | msg->data[3];
    uint8_t  faults = msg->data[4];
    uint8_t  status = msg->data[5];
    uint8_t  cccp   = msg->data[6];
    int      temp_c = (int)msg->data[7] - 40;

    Serial.printf("[CAN_RX] V=%.1fV I=%.1fA fault=0x%02X status=0x%02X cccp=0x%02X temp=%d°C\n",
                  v_raw * 0.1f, i_raw * 0.1f, faults, status, cccp, temp_c);

    if (faults & 0x01) Serial.println("         [!] HW fault");
    if (faults & 0x02) Serial.println("         [!] Over-temperature");
    if (faults & 0x04) Serial.println("         [!] Over-voltage");
    if (faults & 0x08) Serial.println("         [!] Over-current");
    if (status & 0x01) Serial.println("         [i] Charging");
    if (status & 0x02) Serial.println("         [i] Charge complete");
    if (status & 0x04) Serial.println("         [i] Idle/standby");
}

static void poll_rx(void) {
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        if (msg.identifier == CHARGER_RX_ID)
            decode_rx_frame(&msg);
        else
            Serial.printf("[CAN_RX] unknown id=0x%08lX dlc=%d\n",
                          msg.identifier, msg.data_length_code);
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Phase 1.5 TC OBC CAN Bench Test ===");
    Serial.printf("TX ID=0x%08lX  RX ID=0x%08lX  250kbps\n", CHARGER_TX_ID, CHARGER_RX_ID);

    twai_general_config_t g = {
        .mode           = TWAI_MODE_NORMAL,
        .tx_io          = CAN_TX_PIN,
        .rx_io          = CAN_RX_PIN,
        .clkout_io      = TWAI_IO_UNUSED,
        .bus_off_io     = TWAI_IO_UNUSED,
        .tx_queue_len   = 5,
        .rx_queue_len   = 10,
        .alerts_enabled = TWAI_ALERT_ALL,
        .clkout_divider = 0,
    };
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g, &t, &f);
    if (err != ESP_OK) {
        Serial.printf("[CAN_PING] driver install failed: 0x%X\n", err);
        return;
    }
    err = twai_start();
    if (err != ESP_OK) {
        Serial.printf("[CAN_PING] start failed: 0x%X\n", err);
        return;
    }
    twai_ok = true;
    Serial.printf("[CAN_PING] TWAI ready  TX=GPIO%d  RX=GPIO%d\n", CAN_TX_PIN, CAN_RX_PIN);
    send_disable_frame();
}

void loop() {
    if (!twai_ok) return;
    poll_rx();

    static uint32_t last_tx = 0;
    if (millis() - last_tx >= 1000) {
        last_tx = millis();
        send_disable_frame();

        uint32_t alerts = 0;
        twai_read_alerts(&alerts, 0);
        if (alerts & TWAI_ALERT_TX_FAILED)    Serial.println("[CAN_PING] alert: TX failed");
        if (alerts & TWAI_ALERT_BUS_OFF)      Serial.println("[CAN_PING] alert: Bus-off");
        if (alerts & TWAI_ALERT_BUS_RECOVERED)Serial.println("[CAN_PING] alert: Bus recovered");
    }
}

#endif // MAIN_BRAIN
