// ============================================================
//  Charger CAN Functional Test
//  Board  : ESP32S3 Dev Module
//  CAN TX : GPIO 8  (CTX on transceiver)
//  CAN RX : GPIO 9  (CRX on transceiver)
//  Baud   : 250 kbps
// ============================================================
//
//  Serial Monitor commands (115200 baud, No line ending):
//    m   toggle monitor/listen-only mode (no TX, shows all frames)
//    +   increase current 1A
//    -   decrease current 1A
//    v   increase voltage 0.5V
//    V   decrease voltage 0.5V
//    s   stop  (set current to 0A)
//    r   resume (set current back to 2A)
//    w   send immediate wake-up frame
//    e   end charge (charger sleeps)
//    p   print current setpoints
//
//  Safety limits enforced:
//    - Max 400 W (current command refused if it would exceed this)
//    - Voltage setpoint clamped to 75.6 V - 92.4 V (84 V +/-10%)
// ============================================================

#include "driver/twai.h"

#define CAN_TX_PIN      GPIO_NUM_8
#define CAN_RX_PIN      GPIO_NUM_9

#define ID_BMS_TO_OBC   0x1806E5F4UL
#define ID_OBC_TO_BMS   0x18FF50E5UL

#define MAX_WATTS       400.0f
#define VOLT_MIN_DV     756
#define VOLT_MAX_DV     924

static uint16_t s_volt_dv    = 840;
static uint16_t s_cur_da     = 20;
static bool     s_stopped    = false;
static float    s_meas_v     = 0.0f;
static uint32_t s_last_tx_ms = 0;
static bool     s_can_ok     = false;
static bool     s_monitor    = false;

// ── helpers ─────────────────────────────────────────────────

void can_start(bool listen_only) {
    twai_stop();
    twai_driver_uninstall();
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN,
                              listen_only ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g, &t, &f);
    twai_start();
    Serial.printf("[MODE] %s\n", listen_only ? "MONITOR (listen-only, no TX)" : "NORMAL (TX enabled)");
}

void print_setpoints() {
    Serial.printf("[SET] Voltage=%.1fV  Current=%.1fA  MaxW=400W\n",
                  s_volt_dv * 0.1f, s_cur_da * 0.1f);
}

void send_bms_frame(uint8_t control) {
    twai_message_t msg = {};
    msg.extd               = 1;
    msg.identifier         = ID_BMS_TO_OBC;
    msg.data_length_code   = 8;

    // BYTE1-2: max charging voltage  (Motorola / big-endian)
    msg.data[0] = (s_volt_dv >> 8) & 0xFF;
    msg.data[1] =  s_volt_dv       & 0xFF;

    // BYTE3-4: max charging current
    uint16_t cur = s_stopped ? 0 : s_cur_da;
    msg.data[2] = (cur >> 8) & 0xFF;
    msg.data[3] =  cur       & 0xFF;

    // BYTE5: control  0x00=charge  0x01=stop output  0x02=end+sleep
    msg.data[4] = control;

    // BYTE6: mode 0x00=charging
    msg.data[5] = 0x00;

    // BYTE7-8: reserved
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        Serial.printf("[TX ERR] 0x%X\n", err);
    }
}

void decode_obc_frame(const twai_message_t &msg) {
    if (msg.data_length_code < 8) return;

    // BYTE1-2: output voltage
    uint16_t v_raw = ((uint16_t)msg.data[0] << 8) | msg.data[1];
    // BYTE3-4: output current
    uint16_t i_raw = ((uint16_t)msg.data[2] << 8) | msg.data[3];

    s_meas_v = v_raw * 0.1f;
    float meas_a = i_raw * 0.1f;
    float meas_w = s_meas_v * meas_a;

    // BYTE5: fault flags
    uint8_t faults = msg.data[4];
    // BYTE6: status flags
    uint8_t status = msg.data[5];
    // BYTE7: connector/lock flags
    uint8_t connector = msg.data[6];
    // BYTE8: temperature  (offset -40)
    int16_t temp_c = (int16_t)msg.data[7] - 40;

    // Working status: BYTE6 bits 1-2
    uint8_t work = (status >> 1) & 0x03;
    const char *work_str[] = { "Undefined", "Working", "Stopped", "Standby" };

    Serial.printf("[OBC] %.1fV  %.1fA  %.0fW  %d°C  [%s]\n",
                  s_meas_v, meas_a, meas_w, temp_c, work_str[work]);

    // BYTE5 fault bits
    if (faults & 0x01) Serial.println("       [!] Hardware protection active");
    if (faults & 0x02) Serial.println("       [!] Internal temperature protection");
    uint8_t input_st = (faults >> 2) & 0x03;
    if (input_st == 1) Serial.println("       [!] Input under-voltage");
    if (input_st == 2) Serial.println("       [!] Input over-voltage");
    if (input_st == 3) Serial.println("       [!] No input voltage");
    if (faults & 0x10) Serial.println("       [!] Output under-voltage");
    if (faults & 0x20) Serial.println("       [!] Output over-voltage");
    if (faults & 0x40) Serial.println("       [!] Output over-current");
    if (faults & 0x80) Serial.println("       [!] Output short circuit");

    // BYTE6 status bits
    if (status & 0x01) Serial.println("       [!] Comms timeout (charger side)");
    if (status & 0x08) Serial.println("       [i] Initialization complete");
    if (status & 0x10) Serial.println("       [i] Fan running");
    if (status & 0x20) Serial.println("       [i] Cooling pump running");

    // BYTE7 connector bits
    uint8_t cc_st = connector & 0x03;
    const char *cc_str[] = { "Not connected", "Half connected", "Connected", "CC error" };
    Serial.printf("       [CC] %s\n", cc_str[cc_st]);
    if (connector & 0x04) Serial.println("       [i] CP signal normal");
    if (connector & 0x08) Serial.println("       [!] Charging socket over-temp");
    if (connector & 0x80) Serial.println("       [i] S2 switch closed");
}

// ── setup ────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("========================================");
    Serial.println("  Charger CAN Functional Test");
    Serial.println("  GPIO8=CTX  GPIO9=CRX  250kbps");
    Serial.println("========================================");
    Serial.println("Commands: m monitor  + - current  v V voltage  s stop  r resume  w wake  e end  p print");
    print_setpoints();

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("[ERR] TWAI driver install failed — check wiring");
        return;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[ERR] TWAI start failed");
        return;
    }

    s_can_ok = true;
    Serial.println("[OK] CAN started in NORMAL mode — type m to switch to monitor/listen-only");
}

// ── loop ─────────────────────────────────────────────────────

void loop() {
    if (!s_can_ok) return;

    uint32_t now = millis();

    // Keep-alive every 3 s — skipped in monitor mode
    if (!s_monitor && (now - s_last_tx_ms >= 3000)) {
        s_last_tx_ms = now;
        uint8_t ctrl = s_stopped ? 0x01 : 0x00;
        send_bms_frame(ctrl);
        Serial.printf("[TX] %.1fV / %.1fA  ctrl=0x%02X\n",
                      s_volt_dv * 0.1f,
                      s_stopped ? 0.0f : s_cur_da * 0.1f,
                      ctrl);
    }

    // Receive — show all frames
    twai_message_t msg;
    if (twai_receive(&msg, 0) == ESP_OK) {
        if (msg.identifier == ID_OBC_TO_BMS) {
            decode_obc_frame(msg);
        } else {
            Serial.printf("[RX] ID=0x%08X  ", msg.identifier);
            for (uint8_t i = 0; i < msg.data_length_code; i++)
                Serial.printf("%02X ", msg.data[i]);
            Serial.println();
        }
    }

    // Serial commands
    if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case '+': {
                float max_a = (s_meas_v > 10.0f) ? (MAX_WATTS / s_meas_v) : 5.0f;
                uint16_t max_da = (uint16_t)(max_a * 10.0f);
                if (s_cur_da + 10 <= max_da) {
                    s_cur_da += 10;
                    Serial.printf("[SET] Current -> %.1fA\n", s_cur_da * 0.1f);
                } else {
                    Serial.printf("[LIM] 400W cap: max %.1fA at %.1fV\n", max_a, s_meas_v);
                }
                break;
            }
            case '-':
                if (s_cur_da >= 10) {
                    s_cur_da -= 10;
                    Serial.printf("[SET] Current -> %.1fA\n", s_cur_da * 0.1f);
                }
                break;
            case 'v':
                if (s_volt_dv + 5 <= VOLT_MAX_DV) {
                    s_volt_dv += 5;
                    Serial.printf("[SET] Voltage -> %.1fV\n", s_volt_dv * 0.1f);
                } else {
                    Serial.println("[LIM] Already at voltage ceiling (92.4V)");
                }
                break;
            case 'V':
                if (s_volt_dv - 5 >= VOLT_MIN_DV) {
                    s_volt_dv -= 5;
                    Serial.printf("[SET] Voltage -> %.1fV\n", s_volt_dv * 0.1f);
                } else {
                    Serial.println("[LIM] Already at voltage floor (75.6V)");
                }
                break;
            case 's':
                s_stopped = true;
                Serial.println("[SET] STOPPED — output current zeroed");
                break;
            case 'r':
                s_stopped = false;
                if (s_cur_da == 0) s_cur_da = 20;
                Serial.printf("[SET] RESUMED — current %.1fA\n", s_cur_da * 0.1f);
                break;
            case 'w':
                send_bms_frame(0x00);
                Serial.println("[TX] Wake-up frame sent");
                break;
            case 'e':
                send_bms_frame(0x02);
                Serial.println("[TX] End charge — charger will sleep");
                break;
            case 'm':
                s_monitor = !s_monitor;
                can_start(s_monitor);
                break;
            case 'p':
                print_setpoints();
                break;
        }
    }
}
