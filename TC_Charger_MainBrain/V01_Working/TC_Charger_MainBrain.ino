// =============================================================================
// TC_Charger -- Main Brain (ESP32-S3) Consolidated Production Firmware
// =============================================================================
// Board:  ESP32S3 Dev Module
// Flash:  16 MB
// PSRAM:  OPI
// Upload: COM3 @ 921600
// Serial: 115200
//
// Required Libraries (install via Arduino Library Manager):
//   - FastLED
//   - U8g2
//   - Adafruit AHTX0
//   - Adafruit BMP280
//   - TinyGPSPlus
//   (BLE + TWAI are built-in with the ESP32 board package)
// =============================================================================

#include <esp_task_wdt.h>
#include <driver/twai.h>
#include <FastLED.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>

// =============================================================================
//  CONFIGURATION & CONSTANTS
// =============================================================================

#define WDT_TIMEOUT_S       5

// ---- CAN ----
#define CAN_CHARGER_TX_ID       0x1806E5F4UL
#define CAN_CHARGER_RX_ID       0x18FF50E5UL
#define CAN_SPEED_KBPS          250
#define CAN_TX_GPIO             4
#define CAN_RX_GPIO             5
#define CAN_CTRL_CHARGE         0x00
#define CAN_CTRL_DISABLE        0x01
#define CAN_CTRL_SLEEP          0x02
#define CAN_MODE_CHARGE         0x00
#define CAN_MODE_HEAT           0x01
#define CAN_TX_INTERVAL_MS      1000UL
#define CAN_CHARGER_TIMEOUT_MS  5000UL

// ---- RS485 ----
#define RS485_BAUD           115200
#define RS485_TIMEOUT_MS     50
#define RS485_RETRY_COUNT    3
#define RS485_NODE_ID_BRAIN  0x01
#define RS485_NODE_ID_TEMP   0x02
#define RS485_NODE_ID_BMS    0x03
#define RS485_CMD_GET_TEMPS  0x10
#define RS485_CMD_RESP_TEMPS 0x11
#define RS485_CMD_GET_BMS    0x20
#define RS485_CMD_RESP_BMS   0x21
#define RS485_CMD_ACK        0x7E
#define RS485_CMD_NACK       0x7F
#define RS485_PKT_SYNC       0xAA
#define RS485_PKT_MAX_DATA   128
#define RS485_PKT_OVERHEAD   5
#define RS485_TEMPS_PAYLOAD_LEN 38
#define RS485_BMS_PAYLOAD_LEN   91
#define RS485_TX_PIN  17
#define RS485_RX_PIN  18
#define RS485_DE_PIN  0xFF    // auto-direction module, no DE pin
#define RS485_SERIAL  Serial1

// ---- Thermal ----
#define THERMAL_LUT_SIZE        16
#define OVERTEMP_CUTOFF_C       55.0f
#define UNDERTEMP_INHIBIT_C     0.0f
#define THERMAL_POLL_MS         500UL

// ---- RGB LED ----
#define RGB_PIN        48
#define RGB_NUM_LEDS   1
#define RGB_BRIGHTNESS 64

// ---- Buttons ----
#define BTN_NEXT_PIN    6
#define BTN_HOLD_PIN    7
#define BTN_DEBOUNCE_MS 50UL

// ---- OLED ----
#define OLED_SDA_PIN    8
#define OLED_SCL_PIN    9
#define OLED_I2C_ADDR   0x3C
#define OLED_UPDATE_MS  500
#define OLED_PAGE_MS    8000
#define OLED_NUM_PAGES  7

// ---- BME280 / AHT20 ----
#define ENV_SDA_PIN     8
#define ENV_SCL_PIN     9
#define BMP280_ADDR_A   0x76
#define BMP280_ADDR_B   0x77
#define ENV_UPDATE_MS   2000

// ---- GPS ----
#define GPS_TX_PIN              16
#define GPS_RX_PIN              15
#define GPS_BAUD                115200
#define GPS_COMPASS_ADDR        0x0D
#define GPS_CONSUMPTION_WH_PER_KM  20.0f

// ---- BLE ----
#define BLE_DEVICE_NAME         "ScooterCharger"
#define SVC_UUID                "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_SAFETY_STATE_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_MAX_TEMP_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define CHAR_CHARGE_CUR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define CHAR_PACK_VOLTAGE_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26ab"
#define CHAR_BMS_PACK_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26ac"
#define CHAR_BMS_HEALTH_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ad"
#define CHAR_BMS_CELLS_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26ae"
#define CHAR_GPS_UUID           "beb5483e-36e1-4688-b7f5-ea07361b26af"
#define BLE_NOTIFY_INTERVAL_MS  1000UL

// ---- Safety FSM ----
#define COMM_TIMEOUT_MS    3000UL

// =============================================================================
//  TYPE DEFINITIONS
// =============================================================================

// ---- CAN ----
typedef struct {
    uint16_t voltage_dv;
    uint16_t current_da;
    uint8_t  control;
    uint8_t  mode;
} ChargerCmd_t;

typedef struct {
    bool hardware_fault;
    bool temperature_fault;
    uint8_t input_voltage_status;
    bool output_undervoltage;
    bool output_overvoltage;
    bool output_overcurrent;
    bool output_short_circuit;
} ChargerFaultFlags_t;

typedef struct {
    bool     comm_timeout;
    uint8_t  working_status;
    bool     init_complete;
    bool     fan_on;
    bool     cooling_pump_on;
} ChargerStatusFlags_t;

typedef struct {
    uint16_t           output_voltage_dv;
    uint16_t           output_current_da;
    ChargerFaultFlags_t fault;
    ChargerStatusFlags_t status;
    int8_t             temperature_c;
    bool               valid;
} ChargerStatus_t;

// ---- RS485 ----
typedef struct {
    uint8_t  node_id;
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  data[RS485_PKT_MAX_DATA];
    uint16_t crc;
} RS485Packet_t;

typedef struct {
    float   temp_c[6];
    float   ambient_c;
    float   humidity_pct;
    float   pressure_hpa;
    uint8_t valid_mask;
    bool    bme_valid;
} TempNodeData_t;

typedef struct {
    uint16_t cell_mv[20];
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
} BMSData_t;

// ---- Thermal ----
typedef struct {
    float    temp_c;
    uint16_t max_current_da;
} ThermalLUTEntry_t;

// ---- RGB LED ----
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

// ---- Safety FSM ----
typedef enum {
    SAFETY_STATE_INIT = 0,
    SAFETY_STATE_IDLE,
    SAFETY_STATE_CHARGING,
    SAFETY_STATE_OVERTEMP,
    SAFETY_STATE_UNDERTEMP,
    SAFETY_STATE_BMS_FAULT,
    SAFETY_STATE_COMM_TIMEOUT,
    SAFETY_STATE_FAULT,
} SafetyState_t;

// ---- BME280 ----
typedef struct {
    float temperature_c;
    float humidity_pct;
    float pressure_hpa;
    bool  valid;
} BME280Data_t;

// ---- GPS ----
typedef struct {
    float    lat;
    float    lon;
    float    speed_kmh;
    float    course_deg;
    float    heading_deg;
    float    range_km;
    uint8_t  fix;
    uint8_t  satellites;
    bool     gps_valid;
    bool     compass_valid;
} GPSData_t;

// ---- Buttons ----
typedef struct {
    uint8_t  pin;
    bool     last_raw;
    bool     state;
    uint32_t last_change_ms;
    bool     event;
} Button_t;

// =============================================================================
//  CHARGER COMMAND STATE (used by OLED display + serial command handler)
// =============================================================================

#define HOME_VOLTAGE_DV      840
#define HOME_CURRENT_DA       20
#define CHARGER_VOLTAGE_MAX_DV  1080
#define CHARGER_VOLTAGE_MIN_DV   200
#define CHARGER_CURRENT_MAX_DA   600

static uint16_t cmd_voltage_dv   = HOME_VOLTAGE_DV;
static uint16_t cmd_current_da   = HOME_CURRENT_DA;
static bool     cmd_charging     = false;
static bool     cmd_buddy_mode   = false;

// =============================================================================
//  FORWARD DECLARATIONS
// =============================================================================

// CAN
void     can_driver_init(void);
bool     can_send_disable(void);
bool     can_send_sleep(void);
bool     can_send_command(const ChargerCmd_t *cmd);
bool     can_read_status(ChargerStatus_t *out);
void     can_driver_update(void);

// RS485
void rs485_init(void);
bool rs485_send_packet(const RS485Packet_t *pkt);
bool rs485_recv_packet(RS485Packet_t *pkt, uint32_t timeout_ms);
void rs485_poll(void);
bool rs485_request_temp_node(TempNodeData_t *out);
bool rs485_get_last_temp_node(TempNodeData_t *out);
bool rs485_request_bms(BMSData_t *out);
bool rs485_get_last_bms(BMSData_t *out);

// Thermal
void               thermal_logic_init(void);
void               thermal_logic_update(void);
uint16_t           thermal_get_max_current_da(float max_temp_c);
bool               thermal_is_overtemp(float temp_c);
bool               thermal_is_undertemp(float temp_c);
float              thermal_get_max_ntc_temp(void);
const TempNodeData_t *thermal_get_last_data(void);

// RGB LED
void rgb_led_init(void);
void rgb_led_set_state(RGBState_t state);
void rgb_led_update(void);

// Safety FSM
const char *safety_state_name(SafetyState_t s);
void        safety_sm_init(void);
void        safety_sm_update(void);
SafetyState_t safety_sm_get_state(void);
void        safety_sm_report_bms_fault(bool fault);
void        safety_sm_report_comm_timeout(bool timeout);

// BLE
void     ble_server_init(void);
void     ble_server_update(void);
bool     ble_server_is_connected(void);
uint16_t ble_server_get_pack_voltage_dv(void);

// OLED
void oled_display_init(void);
void oled_display_update(void);
void oled_display_next_page(void);

// BME280
void bme280_init(void);
void bme280_update(void);
bool bme280_get(BME280Data_t *out);

// GPS
void gps_driver_init(void);
void gps_driver_update(void);
bool gps_driver_get(GPSData_t *out);

// Buttons
void button_driver_init(void);
void button_driver_update(void);
bool button_next_pressed(void);
bool button_hold_active(void);

// =============================================================================
//  CAN DRIVER
// =============================================================================

static bool            can_initialized      = false;
static ChargerCmd_t    can_last_cmd         = {0, 0, CAN_CTRL_DISABLE, CAN_MODE_CHARGE};
static ChargerStatus_t can_charger_status   = {};
static uint32_t        can_last_tx_ms       = 0;
static uint32_t        can_last_rx_ms       = 0;

// Smoothed values for display (EMA alpha=0.1, smooths pulse charging spikes)
#define CAN_EMA_ALPHA  0.1f
static float    can_smooth_voltage  = 0.0f;
static float    can_smooth_current  = 0.0f;
static bool     can_smooth_primed   = false;

// Energy tracking (accumulated from smoothed averages)
static float    can_energy_wh       = 0.0f;   // watt-hours into battery
static float    can_energy_ah       = 0.0f;   // amp-hours into battery
static uint32_t can_energy_last_ms  = 0;       // last integration timestamp
static uint32_t can_charge_start_ms = 0;       // when charging started
static bool     can_energy_active   = false;   // only accumulate while charging

static bool twai_send_frame(const ChargerCmd_t *cmd) {
    twai_message_t msg = {};
    msg.extd       = 1;
    msg.identifier = CAN_CHARGER_TX_ID;
    msg.data_length_code = 8;

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
    can_last_tx_ms = millis();
    return true;
}

static void parse_rx_frame(const twai_message_t *msg) {
    if (msg->data_length_code < 8) return;

    uint16_t v_raw = ((uint16_t)msg->data[0] << 8) | msg->data[1];
    uint16_t i_raw = ((uint16_t)msg->data[2] << 8) | msg->data[3];
    uint8_t  fault = msg->data[4];
    uint8_t  stat  = msg->data[5];
    uint8_t  temp  = msg->data[7];

    can_charger_status.output_voltage_dv = v_raw;
    can_charger_status.output_current_da = i_raw;
    can_charger_status.temperature_c     = (int8_t)((int16_t)temp - 40);

    can_charger_status.fault.hardware_fault         = (fault >> 0) & 0x01;
    can_charger_status.fault.temperature_fault      = (fault >> 1) & 0x01;
    can_charger_status.fault.input_voltage_status   = (fault >> 2) & 0x03;
    can_charger_status.fault.output_undervoltage    = (fault >> 4) & 0x01;
    can_charger_status.fault.output_overvoltage     = (fault >> 5) & 0x01;
    can_charger_status.fault.output_overcurrent     = (fault >> 6) & 0x01;
    can_charger_status.fault.output_short_circuit   = (fault >> 7) & 0x01;

    can_charger_status.status.comm_timeout      = (stat >> 0) & 0x01;
    can_charger_status.status.working_status    = (stat >> 1) & 0x03;
    can_charger_status.status.init_complete     = (stat >> 3) & 0x01;
    can_charger_status.status.fan_on            = (stat >> 4) & 0x01;
    can_charger_status.status.cooling_pump_on   = (stat >> 5) & 0x01;

    can_charger_status.valid = true;
    can_last_rx_ms = millis();

    // Update EMA smoothing (handles pulse charging spikes)
    float raw_v = v_raw * 0.1f;
    float raw_a = i_raw * 0.1f;
    if (!can_smooth_primed) {
        can_smooth_voltage = raw_v;
        can_smooth_current = raw_a;
        can_smooth_primed  = true;
        can_energy_last_ms = millis();
    } else {
        can_smooth_voltage += CAN_EMA_ALPHA * (raw_v - can_smooth_voltage);
        can_smooth_current += CAN_EMA_ALPHA * (raw_a - can_smooth_current);
    }

    // Energy integration using smoothed values (accurate for pulse charging)
    if (can_energy_active && can_smooth_current > 0.05f) {
        uint32_t now = millis();
        float dt_hours = (float)(now - can_energy_last_ms) / 3600000.0f;
        can_energy_wh += can_smooth_voltage * can_smooth_current * dt_hours;
        can_energy_ah += can_smooth_current * dt_hours;
        can_energy_last_ms = now;
    } else {
        can_energy_last_ms = millis();
    }
}

void can_driver_init(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_GPIO, (gpio_num_t)CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t  f_config;
    f_config.acceptance_code = (CAN_CHARGER_RX_ID << 3);
    f_config.acceptance_mask = ~((CAN_CHARGER_RX_ID << 3));
    f_config.single_filter   = true;

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

    can_initialized = true;
    can_last_tx_ms  = millis();
    can_last_rx_ms  = millis();
    Serial.printf("[CAN] init OK - TX GPIO%d RX GPIO%d @ 250kbps\n", CAN_TX_GPIO, CAN_RX_GPIO);
}

bool can_send_disable(void) {
    can_last_cmd = {0, 0, CAN_CTRL_DISABLE, CAN_MODE_CHARGE};
    if (!can_initialized) return false;
    return twai_send_frame(&can_last_cmd);
}

bool can_send_sleep(void) {
    can_last_cmd = {0, 0, CAN_CTRL_SLEEP, CAN_MODE_CHARGE};
    if (!can_initialized) return false;
    return twai_send_frame(&can_last_cmd);
}

bool can_send_command(const ChargerCmd_t *cmd) {
    if (!cmd) return false;
    can_last_cmd = *cmd;
    if (!can_initialized) return false;
    return twai_send_frame(cmd);
}

bool can_read_status(ChargerStatus_t *out) {
    if (!out) return false;
    if ((millis() - can_last_rx_ms) > CAN_CHARGER_TIMEOUT_MS) {
        can_charger_status.valid = false;
    }
    *out = can_charger_status;
    return can_charger_status.valid;
}

void can_driver_update(void) {
    if (!can_initialized) return;

    if ((millis() - can_last_tx_ms) >= CAN_TX_INTERVAL_MS) {
        twai_send_frame(&can_last_cmd);
    }

    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        if (msg.extd && msg.identifier == CAN_CHARGER_RX_ID) {
            parse_rx_frame(&msg);
        }
    }
}

// =============================================================================
//  RS485 DRIVER (Master side)
// =============================================================================

static bool rs485_initialized = false;

static uint16_t crc16(const uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

void rs485_init(void) {
    RS485_SERIAL.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    rs485_initialized = true;
    Serial.printf("[RS485] init OK - baud=%d TX=GPIO%d RX=GPIO%d DE=none\n",
                  RS485_BAUD, RS485_TX_PIN, RS485_RX_PIN);
}

bool rs485_send_packet(const RS485Packet_t *pkt) {
    if (!pkt || !rs485_initialized) return false;
    if (pkt->len > RS485_PKT_MAX_DATA) return false;

    uint8_t buf[RS485_PKT_OVERHEAD + RS485_PKT_MAX_DATA];
    buf[0] = RS485_PKT_SYNC;
    buf[1] = pkt->node_id;
    buf[2] = pkt->cmd;
    buf[3] = pkt->len;
    for (uint8_t i = 0; i < pkt->len; i++) buf[4 + i] = pkt->data[i];

    uint16_t crc_val = crc16(buf, 4 + pkt->len);
    buf[4 + pkt->len]     = (uint8_t)(crc_val & 0xFF);
    buf[4 + pkt->len + 1] = (uint8_t)(crc_val >> 8);

    uint8_t total = 4 + pkt->len + 2;
    RS485_SERIAL.write(buf, total);
    RS485_SERIAL.flush();
    return true;
}

bool rs485_recv_packet(RS485Packet_t *pkt, uint32_t timeout_ms) {
    if (!pkt || !rs485_initialized) return false;

    uint8_t  buf[RS485_PKT_OVERHEAD + RS485_PKT_MAX_DATA];
    uint8_t  idx   = 0;
    bool     synced = false;
    uint32_t start  = millis();

    while ((millis() - start) < timeout_ms) {
        if (!RS485_SERIAL.available()) continue;
        uint8_t b = (uint8_t)RS485_SERIAL.read();

        if (!synced) {
            if (b == RS485_PKT_SYNC) { buf[idx++] = b; synced = true; }
            continue;
        }
        buf[idx++] = b;
        if (idx >= (uint8_t)(sizeof(buf))) return false;

        if (idx >= 4) {
            uint8_t expected = 4 + buf[3] + 2;
            if (idx == expected) {
                uint16_t rx_crc  = (uint16_t)buf[idx - 2] | ((uint16_t)buf[idx - 1] << 8);
                uint16_t calc    = crc16(buf, idx - 2);
                if (rx_crc != calc) return false;
                pkt->node_id = buf[1];
                pkt->cmd     = buf[2];
                pkt->len     = buf[3];
                for (uint8_t i = 0; i < pkt->len; i++) pkt->data[i] = buf[4 + i];
                pkt->crc     = rx_crc;
                return true;
            }
        }
    }
    return false;
}

static TempNodeData_t rs485_last_temp_data;
static bool           rs485_temp_data_valid = false;
static BMSData_t      rs485_last_bms_data;
static bool           rs485_bms_data_valid  = false;
static uint32_t       rs485_poll_ms         = 0;
static uint32_t       rs485_bms_poll_ms     = 0;
#define POLL_INTERVAL_MS     1000
#define BMS_POLL_INTERVAL_MS 5000

static float unpack_float(const uint8_t *buf, uint8_t &idx) {
    float v;
    uint8_t *p = (uint8_t *)&v;
    p[0] = buf[idx++]; p[1] = buf[idx++];
    p[2] = buf[idx++]; p[3] = buf[idx++];
    return v;
}

static inline uint16_t rd_u16(const uint8_t *b, uint8_t &i) {
    uint16_t v = (uint16_t)b[i] | ((uint16_t)b[i+1] << 8); i += 2; return v;
}
static inline int16_t rd_i16(const uint8_t *b, uint8_t &i) {
    return (int16_t)rd_u16(b, i);
}
static inline uint32_t rd_u32(const uint8_t *b, uint8_t &i) {
    uint32_t v = (uint32_t)b[i] | ((uint32_t)b[i+1]<<8) |
                 ((uint32_t)b[i+2]<<16) | ((uint32_t)b[i+3]<<24);
    i += 4; return v;
}
static inline int32_t rd_i32(const uint8_t *b, uint8_t &i) {
    return (int32_t)rd_u32(b, i);
}

bool rs485_request_temp_node(TempNodeData_t *out) {
    RS485Packet_t req = {};
    req.node_id = RS485_NODE_ID_TEMP;
    req.cmd     = RS485_CMD_GET_TEMPS;
    req.len     = 0;

    for (uint8_t attempt = 0; attempt < RS485_RETRY_COUNT; attempt++) {
        if (!rs485_send_packet(&req)) continue;

        RS485Packet_t resp = {};
        if (!rs485_recv_packet(&resp, RS485_TIMEOUT_MS)) continue;
        if (resp.node_id != RS485_NODE_ID_TEMP)     continue;
        if (resp.cmd     != RS485_CMD_RESP_TEMPS)   continue;
        if (resp.len     != RS485_TEMPS_PAYLOAD_LEN) continue;

        uint8_t i = 0;
        for (uint8_t ch = 0; ch < 6; ch++) out->temp_c[ch] = unpack_float(resp.data, i);
        out->ambient_c    = unpack_float(resp.data, i);
        out->humidity_pct = unpack_float(resp.data, i);
        out->pressure_hpa = unpack_float(resp.data, i);
        out->valid_mask   = resp.data[i++];
        out->bme_valid    = (bool)resp.data[i];

        rs485_last_temp_data  = *out;
        rs485_temp_data_valid = true;
        return true;
    }
    rs485_temp_data_valid = false;
    return false;
}

bool rs485_request_bms(BMSData_t *out) {
    RS485Packet_t req = {};
    req.node_id = RS485_NODE_ID_TEMP;
    req.cmd     = RS485_CMD_GET_BMS;
    req.len     = 0;

    for (uint8_t attempt = 0; attempt < RS485_RETRY_COUNT; attempt++) {
        if (!rs485_send_packet(&req)) continue;

        RS485Packet_t resp = {};
        if (!rs485_recv_packet(&resp, RS485_TIMEOUT_MS)) continue;
        if (resp.node_id != RS485_NODE_ID_TEMP)   continue;
        if (resp.cmd     != RS485_CMD_RESP_BMS)   continue;
        if (resp.len     != RS485_BMS_PAYLOAD_LEN) continue;

        uint8_t i = 0;
        for (uint8_t c = 0; c < 20; c++) out->cell_mv[c] = rd_u16(resp.data, i);
        out->cell_avg_mv    = rd_u16(resp.data, i);
        out->cell_diff_mv   = rd_u16(resp.data, i);
        out->mos_temp_dc    = rd_i16(resp.data, i);
        out->pack_mv        = rd_u32(resp.data, i);
        out->current_ma     = rd_i32(resp.data, i);
        out->bat_temp1_dc   = rd_i16(resp.data, i);
        out->bat_temp2_dc   = rd_i16(resp.data, i);
        out->alarm_flags    = rd_u32(resp.data, i);
        out->balance_ma     = rd_i16(resp.data, i);
        out->balance_status = resp.data[i++];
        out->soc_pct        = resp.data[i++];
        out->remaining_mah  = rd_i32(resp.data, i);
        out->full_charge_mah= rd_u32(resp.data, i);
        out->cycle_count    = rd_u32(resp.data, i);
        out->soh_pct        = resp.data[i++];
        out->run_time_s     = rd_u32(resp.data, i);
        out->charge_en      = resp.data[i++];
        out->discharge_en   = resp.data[i++];
        out->charger_plugged= resp.data[i++];
        out->design_cap_mah = rd_u32(resp.data, i);
        out->valid          = (bool)resp.data[i];

        rs485_last_bms_data  = *out;
        rs485_bms_data_valid = true;
        return true;
    }
    rs485_bms_data_valid = false;
    return false;
}

static bool rs485_temp_was_ok = false;
static bool rs485_bms_was_ok = false;

void rs485_poll(void) {
    uint32_t now = millis();

    if ((now - rs485_poll_ms) >= POLL_INTERVAL_MS) {
        rs485_poll_ms = now;
        TempNodeData_t td;
        if (rs485_request_temp_node(&td)) {
            if (!rs485_temp_was_ok) {
                Serial.println("[RS485] TempNode connected");
                rs485_temp_was_ok = true;
            }
        } else {
            if (rs485_temp_was_ok) {
                Serial.println("[RS485] TempNode lost");
                rs485_temp_was_ok = false;
            }
        }
    }

    if ((now - rs485_bms_poll_ms) >= BMS_POLL_INTERVAL_MS) {
        rs485_bms_poll_ms = now;
        BMSData_t bd;
        if (rs485_request_bms(&bd)) {
            if (!rs485_bms_was_ok) {
                Serial.println("[RS485] BMS connected");
                rs485_bms_was_ok = true;
            }
        } else {
            if (rs485_bms_was_ok) {
                Serial.println("[RS485] BMS lost");
                rs485_bms_was_ok = false;
            }
        }
    }
}

bool rs485_get_last_temp_node(TempNodeData_t *out) {
    if (!out || !rs485_temp_data_valid) return false;
    *out = rs485_last_temp_data;
    return true;
}

bool rs485_get_last_bms(BMSData_t *out) {
    if (!out || !rs485_bms_data_valid) return false;
    *out = rs485_last_bms_data;
    return true;
}

// =============================================================================
//  THERMAL LOGIC
// =============================================================================

static const ThermalLUTEntry_t thermal_lut[THERMAL_LUT_SIZE] = {
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

static TempNodeData_t thermal_last_data;
static bool           thermal_data_valid    = false;
static uint32_t       thermal_last_poll_ms  = 0;

void thermal_logic_init(void) {
    thermal_data_valid = false;
    Serial.println("[THERMAL] init OK");
}

void thermal_logic_update(void) {
    uint32_t now = millis();
    if (now - thermal_last_poll_ms < THERMAL_POLL_MS) return;
    thermal_last_poll_ms = now;

    TempNodeData_t d;
    if (rs485_request_temp_node(&d)) {
        thermal_last_data  = d;
        thermal_data_valid = true;
    } else {
        thermal_data_valid = false;
    }
}

uint16_t thermal_get_max_current_da(float max_temp_c) {
    if (max_temp_c >= OVERTEMP_CUTOFF_C)  return 0;
    if (max_temp_c <= UNDERTEMP_INHIBIT_C) return 0;
    for (int i = THERMAL_LUT_SIZE - 1; i >= 0; i--) {
        if (max_temp_c >= thermal_lut[i].temp_c) return thermal_lut[i].max_current_da;
    }
    return 0;
}

bool thermal_is_overtemp(float temp_c)  { return temp_c >= OVERTEMP_CUTOFF_C; }
bool thermal_is_undertemp(float temp_c) { return temp_c <= UNDERTEMP_INHIBIT_C; }

float thermal_get_max_ntc_temp(void) {
    if (!thermal_data_valid) return -999.0f;
    float mx = -999.0f;
    for (int i = 0; i < 6; i++) {
        if (thermal_last_data.valid_mask & (1 << i)) {
            if (thermal_last_data.temp_c[i] > mx) mx = thermal_last_data.temp_c[i];
        }
    }
    return mx;
}

const TempNodeData_t *thermal_get_last_data(void) {
    return thermal_data_valid ? &thermal_last_data : nullptr;
}

// =============================================================================
//  RGB LED
// =============================================================================

static CRGB rgb_leds[RGB_NUM_LEDS];
static RGBState_t rgb_state    = RGB_STATE_BOOT;
static uint32_t   rgb_last_ms  = 0;
static bool       rgb_phase    = false;

void rgb_led_init(void) {
    FastLED.addLeds<WS2812B, RGB_PIN, GRB>(rgb_leds, RGB_NUM_LEDS);
    FastLED.setBrightness(RGB_BRIGHTNESS);
    rgb_leds[0] = CRGB::White;
    FastLED.show();
    Serial.printf("[RGB] init OK - GPIO%d, %d LED(s)\n", RGB_PIN, RGB_NUM_LEDS);
}

void rgb_led_set_state(RGBState_t state) {
    rgb_state   = state;
    rgb_phase   = false;
    rgb_last_ms = 0;
}

void rgb_led_update(void) {
    uint32_t now      = millis();
    uint32_t interval = 500;
    CRGB     color_on = CRGB::Black;
    bool     blink    = false;

    switch (rgb_state) {
        case RGB_STATE_BOOT:
            color_on = CRGB::White;   interval = 200; blink = true;  break;
        case RGB_STATE_IDLE:
            color_on = CRGB::Blue;    interval = 1000; blink = true; break;
        case RGB_STATE_CHARGING:
            color_on = CRGB::Green;   interval = 0;   blink = false; break;
        case RGB_STATE_OVERTEMP:
            color_on = CRGB::Red;     interval = 150; blink = true;  break;
        case RGB_STATE_UNDERTEMP:
            color_on = CRGB::Cyan;    interval = 300; blink = true;  break;
        case RGB_STATE_BMS_FAULT:
            color_on = CRGB::Orange;  interval = 200; blink = true;  break;
        case RGB_STATE_COMM_TIMEOUT:
            color_on = CRGB::Yellow;  interval = 300; blink = true;  break;
        case RGB_STATE_FAULT:
            color_on = CRGB::Red;     interval = 100; blink = true;  break;
        default:
            color_on = CRGB::Purple;  interval = 500; blink = true;  break;
    }

    if (!blink) {
        rgb_leds[0] = color_on;
        FastLED.show();
        return;
    }

    if ((now - rgb_last_ms) >= interval) {
        rgb_last_ms = now;
        rgb_phase   = !rgb_phase;
        rgb_leds[0] = rgb_phase ? color_on : CRGB::Black;
        FastLED.show();
    }
}

// =============================================================================
//  BUTTON DRIVER
// =============================================================================

static Button_t btn_next      = { BTN_NEXT_PIN, true, true, 0, false };
static Button_t btn_hold_btn  = { BTN_HOLD_PIN, true, true, 0, false };
static bool     btn_hold_active_flag = false;

static void update_btn(Button_t *b) {
    bool raw = (bool)digitalRead(b->pin);
    if (raw == b->last_raw) return;
    b->last_raw = raw;
    if ((millis() - b->last_change_ms) < BTN_DEBOUNCE_MS) return;
    b->last_change_ms = millis();
    bool pressed = !raw;
    if (pressed && !b->state) b->event = true;
    b->state = pressed;
}

void button_driver_init(void) {
    pinMode(BTN_NEXT_PIN, INPUT_PULLUP);
    pinMode(BTN_HOLD_PIN, INPUT_PULLUP);
    Serial.printf("[BTN] init OK - NEXT=GPIO%d HOLD=GPIO%d\n",
                  BTN_NEXT_PIN, BTN_HOLD_PIN);
}

void button_driver_update(void) {
    update_btn(&btn_next);
    update_btn(&btn_hold_btn);
    if (btn_hold_btn.event) {
        btn_hold_btn.event = false;
        btn_hold_active_flag = !btn_hold_active_flag;
        Serial.printf("[BTN] hold %s\n", btn_hold_active_flag ? "ON" : "OFF");
    }
}

bool button_next_pressed(void) {
    if (btn_next.event) { btn_next.event = false; return true; }
    return false;
}

bool button_hold_active(void) { return btn_hold_active_flag; }

// =============================================================================
//  BME280 / AHT20 DRIVER
// =============================================================================

static Adafruit_AHTX0  env_aht;
static Adafruit_BMP280 env_bmp(&Wire);
static BME280Data_t    env_data;
static bool            env_aht_ok  = false;
static bool            env_bmp_ok  = false;
static uint32_t        env_last_ms = 0;

void bme280_init(void) {
    // Wire.begin() is handled by OLED U8G2 init -- do NOT call it again here

    env_aht_ok = env_aht.begin(&Wire);
    if (!env_aht_ok)
        Serial.println("[ENV] AHT20 not found - check SDA/SCL wiring");
    else
        Serial.println("[ENV] AHT20 init OK");

    env_bmp_ok = env_bmp.begin(BMP280_ADDR_A);
    if (!env_bmp_ok) env_bmp_ok = env_bmp.begin(BMP280_ADDR_B);
    if (!env_bmp_ok) {
        Serial.println("[ENV] BMP280 not found - check SDA/SCL wiring and SDO pin");
    } else {
        env_bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                            Adafruit_BMP280::SAMPLING_X2,
                            Adafruit_BMP280::SAMPLING_X16,
                            Adafruit_BMP280::FILTER_X4,
                            Adafruit_BMP280::STANDBY_MS_500);
        Serial.println("[ENV] BMP280 init OK");
    }

    env_data.valid = env_aht_ok || env_bmp_ok;
    env_last_ms    = millis();

    if (env_data.valid) {
        sensors_event_t hum, temp;
        if (env_aht_ok && env_aht.getEvent(&hum, &temp)) {
            env_data.temperature_c = temp.temperature;
            env_data.humidity_pct  = hum.relative_humidity;
        }
        if (env_bmp_ok) {
            float p = env_bmp.readPressure();
            env_data.pressure_hpa = (p > 0.0f) ? p / 100.0f : 0.0f;
            if (!env_aht_ok) env_data.temperature_c = env_bmp.readTemperature();
        }
        Serial.printf("[ENV] T=%.1fF H=%.1f%% P=%.3finHg\n",
                      env_data.temperature_c * 9.0f/5.0f + 32.0f,
                      env_data.humidity_pct,
                      env_data.pressure_hpa * 0.02953f);
    }
}

void bme280_update(void) {
    if (!env_aht_ok && !env_bmp_ok) return;
    uint32_t now = millis();
    if ((now - env_last_ms) < ENV_UPDATE_MS) return;
    env_last_ms = now;

    bool ok = false;

    if (env_aht_ok) {
        sensors_event_t hum, temp;
        if (env_aht.getEvent(&hum, &temp)) {
            env_data.temperature_c = temp.temperature;
            env_data.humidity_pct  = hum.relative_humidity;
            ok = true;
        }
    }

    if (env_bmp_ok) {
        float p = env_bmp.readPressure();
        float t = env_bmp.readTemperature();
        if (!isnan(p) && p > 30000.0f) {
            env_data.pressure_hpa = p / 100.0f;
            ok = true;
        }
        if (!env_aht_ok && !isnan(t)) env_data.temperature_c = t;
    }

    env_data.valid = ok;
    if (!ok) Serial.println("[ENV] read failed - check wiring");
}

bool bme280_get(BME280Data_t *out) {
    if (!out) return false;
    *out = env_data;
    return env_data.valid;
}

// =============================================================================
//  GPS DRIVER
// =============================================================================

static TinyGPSPlus gps_parser;
static GPSData_t   gps_data;
static bool        gps_compass_ok = false;

// ---- QMC5883L registers ----
#define QMC_REG_DATA    0x00
#define QMC_REG_STATUS  0x06
#define QMC_REG_CTRL1   0x09
#define QMC_REG_PERIOD  0x0B
#define QMC_STATUS_DRDY 0x01

static void compass_init(void) {
    Wire.beginTransmission(GPS_COMPASS_ADDR);
    Wire.write(QMC_REG_PERIOD);
    Wire.write(0x01);
    gps_compass_ok = (Wire.endTransmission() == 0);
    if (!gps_compass_ok) {
        Serial.println("[GPS] QMC5883L not found - compass disabled");
        return;
    }
    Wire.beginTransmission(GPS_COMPASS_ADDR);
    Wire.write(QMC_REG_CTRL1);
    Wire.write(0x1D);
    Wire.endTransmission();
    Serial.println("[GPS] QMC5883L init OK");
}

static bool compass_read(float *heading_deg) {
    Wire.beginTransmission(GPS_COMPASS_ADDR);
    Wire.write(QMC_REG_STATUS);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)GPS_COMPASS_ADDR, (uint8_t)1);
    if (!Wire.available()) return false;
    if (!(Wire.read() & QMC_STATUS_DRDY)) return false;

    Wire.beginTransmission(GPS_COMPASS_ADDR);
    Wire.write(QMC_REG_DATA);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)GPS_COMPASS_ADDR, (uint8_t)6);
    if (Wire.available() < 6) return false;

    int16_t x = (int16_t)((uint16_t)Wire.read() | ((uint16_t)Wire.read() << 8));
    int16_t y = (int16_t)((uint16_t)Wire.read() | ((uint16_t)Wire.read() << 8));
    Wire.read(); Wire.read();   // discard Z

    float h = atan2f((float)y, (float)x) * 180.0f / (float)M_PI;
    if (h < 0.0f) h += 360.0f;
    *heading_deg = h;
    return true;
}

static float estimate_range_km(void) {
    BMSData_t bms = {};
    if (!rs485_get_last_bms(&bms) || !bms.valid) return -1.0f;
    if (bms.remaining_mah <= 0) return 0.0f;
    float remaining_wh = ((float)bms.remaining_mah / 1000.0f) *
                         ((float)bms.pack_mv        / 1000000.0f) * 1000.0f;
    return remaining_wh / GPS_CONSUMPTION_WH_PER_KM;
}

void gps_driver_init(void) {
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    compass_init();
    memset(&gps_data, 0, sizeof(gps_data));
    gps_data.range_km = -1.0f;
    Serial.printf("[GPS] init OK - RX=GPIO%d TX=GPIO%d baud=%d\n",
                  GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
}

void gps_driver_update(void) {
    while (Serial2.available())
        gps_parser.encode((char)Serial2.read());

    if (gps_parser.location.isValid()) {
        gps_data.lat       = (float)gps_parser.location.lat();
        gps_data.lon       = (float)gps_parser.location.lng();
        gps_data.gps_valid = true;
    }
    if (gps_parser.speed.isValid())
        gps_data.speed_kmh  = (float)gps_parser.speed.kmph();
    if (gps_parser.course.isValid())
        gps_data.course_deg = (float)gps_parser.course.deg();

    gps_data.fix        = gps_parser.location.isValid() ? 1 : 0;
    gps_data.satellites = gps_parser.satellites.isValid()
                        ? (uint8_t)gps_parser.satellites.value() : 0;

    float hdg = 0.0f;
    gps_data.compass_valid = compass_read(&hdg);
    if (gps_data.compass_valid) gps_data.heading_deg = hdg;

    gps_data.range_km = estimate_range_km();
}

bool gps_driver_get(GPSData_t *out) {
    if (!out) return false;
    *out = gps_data;
    return gps_data.gps_valid;
}

// =============================================================================
//  STATE TRACKER (display-only, never sends CAN -- serial commands own the bus)
// =============================================================================

static SafetyState_t safety_state = SAFETY_STATE_INIT;

const char *safety_state_name(SafetyState_t s) {
    switch (s) {
        case SAFETY_STATE_INIT:         return "INIT";
        case SAFETY_STATE_IDLE:         return "IDLE";
        case SAFETY_STATE_CHARGING:     return "CHARGING";
        case SAFETY_STATE_OVERTEMP:     return "OVER TEMP";
        case SAFETY_STATE_UNDERTEMP:    return "UNDER TEMP";
        case SAFETY_STATE_BMS_FAULT:    return "BMS FAULT";
        case SAFETY_STATE_COMM_TIMEOUT: return "COMM TMO";
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
    safety_state = SAFETY_STATE_IDLE;
    rgb_led_set_state(RGB_STATE_IDLE);
}

// Display-only: updates state for OLED/RGB based on what serial commands are doing.
// Declared later because it references cmd_charging (defined in serial command section).
void safety_sm_update(void);

SafetyState_t safety_sm_get_state(void) { return safety_state; }

void safety_sm_report_bms_fault(bool fault)      { (void)fault; }
void safety_sm_report_comm_timeout(bool timeout)  { (void)timeout; }

// =============================================================================
//  BLE SERVER
// =============================================================================

static BLEServer           *ble_server_ptr      = nullptr;
static BLECharacteristic   *ble_ch_state    = nullptr;
static BLECharacteristic   *ble_ch_temp     = nullptr;
static BLECharacteristic   *ble_ch_cur      = nullptr;
static BLECharacteristic   *ble_ch_voltage  = nullptr;
static BLECharacteristic   *ble_ch_bms_pack    = nullptr;
static BLECharacteristic   *ble_ch_bms_health  = nullptr;
static BLECharacteristic   *ble_ch_bms_cells   = nullptr;
static BLECharacteristic   *ble_ch_gps         = nullptr;
static bool                 ble_connected  = false;
static uint16_t             ble_pack_voltage_dv = 840;
static uint32_t             ble_last_notify_ms  = 0;

class ServerCB : public BLEServerCallbacks {
    void onConnect(BLEServer *)    override { ble_connected = true;  Serial.println("[BLE] client connected"); }
    void onDisconnect(BLEServer *srv) override {
        ble_connected = false;
        Serial.println("[BLE] client disconnected, restarting advertising");
        srv->startAdvertising();
    }
};

class VoltageCB : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *ch) override {
        if (ch->getLength() >= 2) {
            const uint8_t *d = ch->getData();
            ble_pack_voltage_dv = (uint16_t)(d[0] | (d[1] << 8));
            Serial.printf("[BLE] pack voltage set -> %u dV (%.1f V)\n",
                          ble_pack_voltage_dv, ble_pack_voltage_dv * 0.1f);
        }
    }
};

void ble_server_init(void) {
    BLEDevice::init(BLE_DEVICE_NAME);
    ble_server_ptr = BLEDevice::createServer();
    ble_server_ptr->setCallbacks(new ServerCB());

    BLEService *svc = ble_server_ptr->createService(SVC_UUID);

    ble_ch_state = svc->createCharacteristic(CHAR_SAFETY_STATE_UUID,
                     BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    ble_ch_temp = svc->createCharacteristic(CHAR_MAX_TEMP_UUID,
                    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    ble_ch_cur = svc->createCharacteristic(CHAR_CHARGE_CUR_UUID,
                   BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    ble_ch_voltage = svc->createCharacteristic(CHAR_PACK_VOLTAGE_UUID,
                       BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    ble_ch_voltage->setCallbacks(new VoltageCB());
    uint8_t vbuf[2] = { (uint8_t)(ble_pack_voltage_dv & 0xFF), (uint8_t)(ble_pack_voltage_dv >> 8) };
    ble_ch_voltage->setValue(vbuf, 2);

    ble_ch_bms_pack = svc->createCharacteristic(CHAR_BMS_PACK_UUID,
                        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    ble_ch_bms_health = svc->createCharacteristic(CHAR_BMS_HEALTH_UUID,
                          BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    ble_ch_bms_cells = svc->createCharacteristic(CHAR_BMS_CELLS_UUID,
                         BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    ble_ch_gps = svc->createCharacteristic(CHAR_GPS_UUID,
                   BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    svc->start();
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(SVC_UUID);
    adv->setScanResponse(true);
    adv->start();
    Serial.println("[BLE] advertising as " BLE_DEVICE_NAME);
}

void ble_server_update(void) {
    uint32_t now = millis();
    if (now - ble_last_notify_ms < BLE_NOTIFY_INTERVAL_MS) return;
    ble_last_notify_ms = now;

    uint8_t state_byte = (uint8_t)safety_sm_get_state();
    ble_ch_state->setValue(&state_byte, 1);
    if (ble_connected) ble_ch_state->notify();

    float max_t = thermal_get_max_ntc_temp();
    ble_ch_temp->setValue((uint8_t *)&max_t, sizeof(float));
    if (ble_connected) ble_ch_temp->notify();

    uint16_t cur_da = thermal_get_max_current_da(max_t);
    uint8_t cbuf[2] = { (uint8_t)(cur_da & 0xFF), (uint8_t)(cur_da >> 8) };
    ble_ch_cur->setValue(cbuf, 2);
    if (ble_connected) ble_ch_cur->notify();

    BMSData_t bms = {};
    rs485_get_last_bms(&bms);

    // BMS pack characteristic (12 bytes)
    {
        uint8_t b[12];
        b[0]  = (bms.pack_mv >> 24) & 0xFF; b[1]  = (bms.pack_mv >> 16) & 0xFF;
        b[2]  = (bms.pack_mv >>  8) & 0xFF; b[3]  =  bms.pack_mv        & 0xFF;
        b[4]  = ((uint32_t)bms.current_ma >> 24) & 0xFF;
        b[5]  = ((uint32_t)bms.current_ma >> 16) & 0xFF;
        b[6]  = ((uint32_t)bms.current_ma >>  8) & 0xFF;
        b[7]  =  (uint32_t)bms.current_ma        & 0xFF;
        b[8]  = bms.soc_pct;
        b[9]  = bms.charge_en;
        b[10] = bms.discharge_en;
        b[11] = bms.charger_plugged;
        ble_ch_bms_pack->setValue(b, sizeof(b));
        if (ble_connected) ble_ch_bms_pack->notify();
    }

    // BMS health characteristic (23 bytes)
    {
        uint8_t b[23];
        b[0]  = ((uint16_t)bms.mos_temp_dc  >> 8) & 0xFF; b[1]  = (uint16_t)bms.mos_temp_dc  & 0xFF;
        b[2]  = ((uint16_t)bms.bat_temp1_dc >> 8) & 0xFF; b[3]  = (uint16_t)bms.bat_temp1_dc & 0xFF;
        b[4]  = ((uint16_t)bms.bat_temp2_dc >> 8) & 0xFF; b[5]  = (uint16_t)bms.bat_temp2_dc & 0xFF;
        b[6]  = bms.soh_pct;
        b[7]  = (bms.cycle_count    >> 24) & 0xFF; b[8]  = (bms.cycle_count    >> 16) & 0xFF;
        b[9]  = (bms.cycle_count    >>  8) & 0xFF; b[10] =  bms.cycle_count           & 0xFF;
        b[11] = (bms.full_charge_mah >> 24) & 0xFF; b[12] = (bms.full_charge_mah >> 16) & 0xFF;
        b[13] = (bms.full_charge_mah >>  8) & 0xFF; b[14] =  bms.full_charge_mah        & 0xFF;
        b[15] = (bms.design_cap_mah  >> 24) & 0xFF; b[16] = (bms.design_cap_mah  >> 16) & 0xFF;
        b[17] = (bms.design_cap_mah  >>  8) & 0xFF; b[18] =  bms.design_cap_mah         & 0xFF;
        b[19] = (bms.run_time_s >> 24) & 0xFF; b[20] = (bms.run_time_s >> 16) & 0xFF;
        b[21] = (bms.run_time_s >>  8) & 0xFF; b[22] =  bms.run_time_s        & 0xFF;
        ble_ch_bms_health->setValue(b, sizeof(b));
        if (ble_connected) ble_ch_bms_health->notify();
    }

    // BMS cells characteristic (44 bytes)
    {
        uint8_t b[44];
        uint8_t i = 0;
        for (uint8_t c = 0; c < 20; c++) {
            b[i++] = (bms.cell_mv[c] >> 8) & 0xFF;
            b[i++] =  bms.cell_mv[c]       & 0xFF;
        }
        b[i++] = (bms.cell_avg_mv  >> 8) & 0xFF; b[i++] = bms.cell_avg_mv  & 0xFF;
        b[i++] = (bms.cell_diff_mv >> 8) & 0xFF; b[i]   = bms.cell_diff_mv & 0xFF;
        ble_ch_bms_cells->setValue(b, sizeof(b));
        if (ble_connected) ble_ch_bms_cells->notify();
    }

    // GPS characteristic (26 bytes)
    {
        GPSData_t gps = {};
        gps_driver_get(&gps);
        uint8_t b[26];
        memcpy(&b[0],  &gps.lat,        4);
        memcpy(&b[4],  &gps.lon,        4);
        memcpy(&b[8],  &gps.speed_kmh,  4);
        memcpy(&b[12], &gps.course_deg, 4);
        memcpy(&b[16], &gps.heading_deg,4);
        memcpy(&b[20], &gps.range_km,   4);
        b[24] = gps.fix;
        b[25] = gps.satellites;
        ble_ch_gps->setValue(b, sizeof(b));
        if (ble_connected) ble_ch_gps->notify();
    }
}

bool     ble_server_is_connected(void)        { return ble_connected; }
uint16_t ble_server_get_pack_voltage_dv(void) { return ble_pack_voltage_dv; }

// =============================================================================
//  OLED DISPLAY
// =============================================================================

static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0,
    U8X8_PIN_NONE,
    OLED_SCL_PIN,
    OLED_SDA_PIN
);

static uint32_t oled_last_update  = 0;
static uint32_t oled_last_page_ms = 0;
static uint8_t  oled_page         = 0;
static bool     oled_held         = false;

// ---- helpers ----------------------------------------------------------------

static const char *oled_state_label(SafetyState_t s) {
    switch (s) {
        case SAFETY_STATE_INIT:         return "INIT";
        case SAFETY_STATE_IDLE:         return "IDLE";
        case SAFETY_STATE_CHARGING:     return "CHARGING";
        case SAFETY_STATE_OVERTEMP:     return "OVER TEMP";
        case SAFETY_STATE_UNDERTEMP:    return "UNDER TEMP";
        case SAFETY_STATE_BMS_FAULT:    return "BMS FAULT";
        case SAFETY_STATE_COMM_TIMEOUT: return "COMM TMO";
        case SAFETY_STATE_FAULT:        return "FAULT";
        default:                        return "UNKNOWN";
    }
}

static void draw_header(SafetyState_t state, const char *page_title) {
    const char *label = oled_state_label(state);

    u8g2.setFont(u8g2_font_7x13B_tr);
    int16_t  lw    = u8g2.getStrWidth(label);
    int16_t  x     = (128 - lw) / 2;

    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 15);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x, 12, label);
    u8g2.setDrawColor(1);

    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 23, page_title);
    uint8_t ind_x = 128;
    if (state == SAFETY_STATE_CHARGING) {
        ind_x -= 30;
        u8g2.drawStr(ind_x, 23, "[CHG]");
    }
    if (oled_held) {
        ind_x -= 30;
        u8g2.drawStr(ind_x, 23, "[HLD]");
    }
    u8g2.drawHLine(0, 24, 128);
}

// ---- Content Y layout -------------------------------------------------------
// Header:  y=0-14 inverted bar, y=15-23 subtitle, y=24 hline
// Content: y=34..63 for 6x10 font (4 rows at y=34,43,52,61)
//          y=32..63 for 5x7 font  (5 rows at y=32,39,46,53,60)

// Page 0 - Charger Output
static void draw_data(void) {
    u8g2.setFont(u8g2_font_6x10_tf);
    char buf[32];

    // Row 1: Smoothed pack voltage + Set target voltage
    ChargerStatus_t cs;
    bool has_chgr = can_read_status(&cs);
    if (has_chgr) {
        int sv = (int)(can_smooth_voltage * 10.0f);
        snprintf(buf, sizeof(buf), "%3d.%dV  Set:%3u.%uV",
                 sv / 10, sv % 10,
                 cmd_voltage_dv / 10, cmd_voltage_dv % 10);
    } else {
        snprintf(buf, sizeof(buf), "--.-V   Set:%3u.%uV",
                 cmd_voltage_dv / 10, cmd_voltage_dv % 10);
    }
    u8g2.drawStr(0, 34, buf);

    // Row 2: Smoothed output current + Set target current
    if (has_chgr) {
        int sa = (int)(can_smooth_current * 10.0f);
        snprintf(buf, sizeof(buf), "%3d.%dA  Set:%3u.%uA",
                 sa / 10, sa % 10,
                 cmd_current_da / 10, cmd_current_da % 10);
    } else {
        snprintf(buf, sizeof(buf), "--.-A   Set:%3u.%uA",
                 cmd_current_da / 10, cmd_current_da % 10);
    }
    u8g2.drawStr(0, 43, buf);

    // Row 3: Charger temp + smoothed power
    if (has_chgr) {
        int temp_f = (int)(cs.temperature_c * 9 / 5 + 32);
        int watts = (int)(can_smooth_voltage * can_smooth_current);
        snprintf(buf, sizeof(buf), "Chgr:%3dF  %4dW", temp_f, watts);
    } else {
        snprintf(buf, sizeof(buf), "Chgr:---F  ----W");
    }
    u8g2.drawStr(0, 52, buf);

    // Row 4: Charging state + fault indicator
    const char *state_str = cmd_charging ? "CHARGING" : "DISABLED";
    const char *fault_str = "";
    if (has_chgr) {
        if      (cs.fault.hardware_fault)      fault_str = " !HW";
        else if (cs.fault.temperature_fault)   fault_str = " !OTP";
        else if (cs.fault.output_overvoltage)  fault_str = " !OVP";
        else if (cs.fault.output_overcurrent)  fault_str = " !OCP";
        else if (cs.fault.output_short_circuit) fault_str = " !SHT";
    }
    snprintf(buf, sizeof(buf), "%s%s", state_str, fault_str);
    u8g2.drawStr(0, 61, buf);
}

// Page 1 - Ambient Sensors
static void draw_env(void) {
    u8g2.setFont(u8g2_font_6x10_tf);

    BME280Data_t mb;
    bool mb_ok = bme280_get(&mb);

    TempNodeData_t dn;
    bool dn_ok = rs485_get_last_temp_node(&dn);

    char buf[32];

    if (mb_ok && mb.valid) {
        float tf = mb.temperature_c * 9.0f / 5.0f + 32.0f;
        snprintf(buf, sizeof(buf), "MB:%5.1fF %4.1f%%", tf, mb.humidity_pct);
    } else {
        snprintf(buf, sizeof(buf), "MB: ---.-F  --.-%%");
    }
    u8g2.drawStr(0, 34, buf);

    if (mb_ok && mb.valid && mb.pressure_hpa > 0.0f)
        snprintf(buf, sizeof(buf), "   %6.3f inHg", mb.pressure_hpa * 0.02953f);
    else
        snprintf(buf, sizeof(buf), "   --.--- inHg");
    u8g2.drawStr(0, 43, buf);

    if (dn_ok && dn.bme_valid) {
        float tf = dn.ambient_c * 9.0f / 5.0f + 32.0f;
        snprintf(buf, sizeof(buf), "DN:%5.1fF %4.1f%%", tf, dn.humidity_pct);
    } else {
        snprintf(buf, sizeof(buf), "DN: ---.-F  --.-%%");
    }
    u8g2.drawStr(0, 52, buf);

    if (dn_ok && dn.bme_valid && dn.pressure_hpa > 0.0f)
        snprintf(buf, sizeof(buf), "   %6.3f inHg", dn.pressure_hpa * 0.02953f);
    else
        snprintf(buf, sizeof(buf), "   --.--- inHg");
    u8g2.drawStr(0, 61, buf);
}

// Page 2 - Battery NTC Temps (T1-T5)
static void draw_ntc(void) {
    u8g2.setFont(u8g2_font_6x10_tf);

    TempNodeData_t dn;
    bool ok = rs485_get_last_temp_node(&dn);

    char buf[24];

    for (uint8_t i = 1; i <= 5; i++) {
        bool valid = ok && (dn.valid_mask & (1 << i));
        if (valid) {
            float tf = dn.temp_c[i] * 9.0f / 5.0f + 32.0f;
            snprintf(buf, sizeof(buf), "T%u: %6.1f F", i, tf);
        } else {
            snprintf(buf, sizeof(buf), "T%u:  ---.- F", i);
        }
        u8g2.drawStr(0, 32 + (i - 1) * 8, buf);
    }
}

static float dc_to_f(int16_t dc) {
    return (dc / 10.0f) * 9.0f / 5.0f + 32.0f;
}

// Page 3 - Pack Status (V, A, Ah, flags)
static void draw_bms_pack(void) {
    u8g2.setFont(u8g2_font_6x10_tf);

    BMSData_t bms = {};
    bool ok = rs485_get_last_bms(&bms);

    char buf[28];

    if (ok && bms.valid) {
        uint32_t v_dv = bms.pack_mv / 100;
        snprintf(buf, sizeof(buf), "%3lu.%luV  SOC:%3u%%",
                 (unsigned long)(v_dv / 10), (unsigned long)(v_dv % 10),
                 bms.soc_pct);
    } else {
        snprintf(buf, sizeof(buf), "---.--V  SOC:--%%");
    }
    u8g2.drawStr(0, 34, buf);

    if (ok && bms.valid) {
        int32_t a_da = bms.current_ma / 100;
        char sign = (a_da >= 0) ? '+' : '-';
        if (a_da < 0) a_da = -a_da;
        snprintf(buf, sizeof(buf), "I:%c%3ld.%ldA",
                 sign, (long)(a_da / 10), (long)(a_da % 10));
    } else {
        snprintf(buf, sizeof(buf), "I:  --.-A");
    }
    u8g2.drawStr(0, 43, buf);

    if (ok && bms.valid) {
        uint32_t rem_dah  = (uint32_t)((bms.remaining_mah  < 0 ? 0 : bms.remaining_mah)  / 100);
        uint32_t full_dah = bms.full_charge_mah / 100;
        snprintf(buf, sizeof(buf), "%2lu.%luAh / %2lu.%luAh",
                 (unsigned long)(rem_dah / 10), (unsigned long)(rem_dah % 10),
                 (unsigned long)(full_dah / 10), (unsigned long)(full_dah % 10));
    } else {
        snprintf(buf, sizeof(buf), "--.-Ah / --.-Ah");
    }
    u8g2.drawStr(0, 52, buf);

    if (ok && bms.valid) {
        snprintf(buf, sizeof(buf), "C:%s D:%s %s",
                 bms.charge_en    ? "ON " : "OFF",
                 bms.discharge_en ? "ON " : "OFF",
                 bms.charger_plugged ? "PLUG" : "    ");
    } else {
        snprintf(buf, sizeof(buf), "C:--- D:---");
    }
    u8g2.drawStr(0, 61, buf);
}

// Page 4 - Pack Health (temps, SOH, cycles)
static void draw_bms_health(void) {
    u8g2.setFont(u8g2_font_6x10_tf);

    BMSData_t bms = {};
    bool ok = rs485_get_last_bms(&bms);

    char buf[28];

    if (ok && bms.valid) {
        snprintf(buf, sizeof(buf), "MOS:%5.1fF T1:%5.1fF",
                 dc_to_f(bms.mos_temp_dc), dc_to_f(bms.bat_temp1_dc));
    } else {
        snprintf(buf, sizeof(buf), "MOS:--.-F T1:--.-F");
    }
    u8g2.drawStr(0, 34, buf);

    if (ok && bms.valid) {
        snprintf(buf, sizeof(buf), "T2:%5.1fF  SOH:%3u%%",
                 dc_to_f(bms.bat_temp2_dc), bms.soh_pct);
    } else {
        snprintf(buf, sizeof(buf), "T2:--.-F  SOH:--%%");
    }
    u8g2.drawStr(0, 43, buf);

    if (ok && bms.valid) {
        uint32_t cap_dah = bms.design_cap_mah / 100;
        snprintf(buf, sizeof(buf), "Cyc:%4lu Cap:%2lu.%luAh",
                 (unsigned long)bms.cycle_count,
                 (unsigned long)(cap_dah / 10), (unsigned long)(cap_dah % 10));
    } else {
        snprintf(buf, sizeof(buf), "Cyc:---- Cap:--.-Ah");
    }
    u8g2.drawStr(0, 52, buf);

    if (ok && bms.valid) {
        uint32_t h = bms.run_time_s / 3600;
        uint32_t m = (bms.run_time_s % 3600) / 60;
        snprintf(buf, sizeof(buf), "Run:%4luh %02lum",
                 (unsigned long)h, (unsigned long)m);
    } else {
        snprintf(buf, sizeof(buf), "Run: ----h --m");
    }
    u8g2.drawStr(0, 61, buf);
}

// Page 5 - 20S Cell Voltages (5 cols x 4 rows = 20 cells)
static void draw_bms_cells(void) {
    BMSData_t bms = {};
    bool ok = rs485_get_last_bms(&bms);

    char buf[28];
    u8g2.setFont(u8g2_font_5x7_tf);

    // Summary row: min / max / avg / delta
    if (ok && bms.valid) {
        uint16_t mn = 0xFFFF, mx = 0;
        for (uint8_t c = 0; c < 20; c++) {
            if (bms.cell_mv[c] < mn) mn = bms.cell_mv[c];
            if (bms.cell_mv[c] > mx) mx = bms.cell_mv[c];
        }
        snprintf(buf, sizeof(buf), "Lo:%4u Hi:%4u Df:%3u",
                 mn, mx, bms.cell_diff_mv);
    } else {
        snprintf(buf, sizeof(buf), "Lo:---- Hi:---- Df:---");
    }
    u8g2.drawStr(0, 31, buf);

    // 5 columns x 4 rows = 20 cells
    // Format "X.XX" (2 decimal places) -- 4 chars x 6px = 24px per cell
    // Columns at x = 0, 26, 52, 78, 104
    for (uint8_t row = 0; row < 4; row++) {
        for (uint8_t col = 0; col < 5; col++) {
            uint8_t idx = row * 5 + col;
            uint8_t x   = col * 26;
            uint8_t y   = 40 + row * 8;
            if (ok && bms.valid) {
                uint16_t mv = bms.cell_mv[idx];
                snprintf(buf, sizeof(buf), "%u.%02u", mv / 1000, (mv % 1000) / 10);
            } else {
                snprintf(buf, sizeof(buf), "-.--");
            }
            u8g2.drawStr(x, y, buf);
        }
    }
}

// Page 6 - GPS & Range
static void draw_gps(void) {
    u8g2.setFont(u8g2_font_5x7_tf);

    GPSData_t gps = {};
    bool ok = gps_driver_get(&gps);

    char buf[32];

    // Row 1: Lat / Lon
    if (ok) {
        snprintf(buf, sizeof(buf), "Lat:%9.5f", gps.lat);
    } else {
        snprintf(buf, sizeof(buf), "Lat: ---.-----");
    }
    u8g2.drawStr(0, 32, buf);

    if (ok) {
        snprintf(buf, sizeof(buf), "Lon:%9.5f", gps.lon);
    } else {
        snprintf(buf, sizeof(buf), "Lon: ---.-----");
    }
    u8g2.drawStr(0, 40, buf);

    // Row 3: Speed + Course
    if (ok) {
        float mph = gps.speed_kmh * 0.621371f;
        snprintf(buf, sizeof(buf), "Spd:%5.1fmph Crs:%3.0f",
                 mph, gps.course_deg);
    } else {
        snprintf(buf, sizeof(buf), "Spd:--.-mph Crs:---");
    }
    u8g2.drawStr(0, 48, buf);

    // Row 4: Range + Heading
    if (ok && gps.range_km >= 0.0f) {
        float miles = gps.range_km * 0.621371f;
        snprintf(buf, sizeof(buf), "Rng:%5.1fmi", miles);
    } else {
        snprintf(buf, sizeof(buf), "Rng: --.-mi");
    }
    if (gps.compass_valid) {
        snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
                 " Hdg:%3.0f", gps.heading_deg);
    } else {
        snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
                 " Hdg:---");
    }
    u8g2.drawStr(0, 56, buf);

    // Row 5: Fix status + Satellites
    snprintf(buf, sizeof(buf), "%s  Sats:%2u",
             ok ? "FIX:YES" : "FIX:NO ", gps.satellites);
    u8g2.drawStr(0, 63, buf);
}

// ---- OLED public ----

void oled_display_init(void) {
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(14, 36, "ScooterCharger");
    u8g2.sendBuffer();
    Serial.printf("[OLED] init OK - SH1106 SDA=GPIO%d SCL=GPIO%d addr=0x%02X\n",
                  OLED_SDA_PIN, OLED_SCL_PIN, OLED_I2C_ADDR);
}

void oled_display_update(void) {
    uint32_t now = millis();
    if (now - oled_last_update < OLED_UPDATE_MS) return;
    oled_last_update = now;

    oled_held = button_hold_active();
    if (button_next_pressed()) {
        oled_page         = (oled_page + 1) % OLED_NUM_PAGES;
        oled_last_page_ms = now;
    }

    if (!oled_held && (now - oled_last_page_ms) >= OLED_PAGE_MS) {
        oled_last_page_ms = now;
        oled_page = (oled_page + 1) % OLED_NUM_PAGES;
    }

    SafetyState_t state = safety_sm_get_state();

    static const char *page_titles[OLED_NUM_PAGES] = {
        "1/7 Charger Output",
        "2/7 Ambient Sensors",
        "3/7 Battery NTC Temps",
        "4/7 Pack Status",
        "5/7 Pack Health",
        "6/7 20S Cell Volts",
        "7/7 GPS & Range",
    };

    u8g2.clearBuffer();
    draw_header(state, page_titles[oled_page]);
    if      (oled_page == 0) draw_data();
    else if (oled_page == 1) draw_env();
    else if (oled_page == 2) draw_ntc();
    else if (oled_page == 3) draw_bms_pack();
    else if (oled_page == 4) draw_bms_health();
    else if (oled_page == 5) draw_bms_cells();
    else                     draw_gps();
    u8g2.sendBuffer();
}

void oled_display_next_page(void) {
    oled_page = (oled_page + 1) % OLED_NUM_PAGES;
    oled_last_page_ms = millis();
}

// =============================================================================
//  SERIAL COMMAND INTERFACE (Putty / Serial Monitor)
// =============================================================================

// (cmd_voltage_dv, cmd_current_da, cmd_charging, cmd_buddy_mode
//  and HOME/CHARGER defines are declared near top of file)
static uint32_t cmd_status_ms    = 0;
#define CMD_STATUS_INTERVAL_MS  5000UL

// safety_sm_update() -- display-only, defined here because it uses cmd_charging
void safety_sm_update(void) {
    SafetyState_t prev = safety_state;

    if (cmd_charging) {
        safety_state = SAFETY_STATE_CHARGING;
    } else {
        safety_state = SAFETY_STATE_IDLE;
    }

    if (safety_state != prev) {
        rgb_led_set_state(safety_to_rgb(safety_state));
    }
}

static void cmd_print_help(void) {
    Serial.println();
    Serial.println("========= TC_Charger Commands =========");
    Serial.println("  r  - Resume charging at current setpoints");
    Serial.println("  s  - Stop (disable output, charger stays awake)");
    Serial.println("  e  - End charge (charger sleeps, needs AC power cycle)");
    Serial.println("  w  - Wake (send immediate CAN frame to charger)");
    Serial.println("  +  - Current +1A");
    Serial.println("  -  - Current -1A");
    Serial.println("  v  - Voltage +0.5V");
    Serial.println("  V  - Voltage -0.5V");
    Serial.println("  b  - Enter BUDDY mode (charge another battery)");
    Serial.println("  n  - Exit buddy mode, restore home settings");
    Serial.println("  0  - Reset energy counters (Wh/Ah) to zero");
    Serial.println("  p  - Print full status report");
    Serial.println("  ?  - Show this help");
    Serial.println();
    Serial.printf( "  Home preset: %.1fV / %.1fA (your 20S pack)\n",
                   HOME_VOLTAGE_DV * 0.1f, HOME_CURRENT_DA * 0.1f);
    Serial.printf( "  Charger range: %.1fV - %.1fV, max %.0fA\n",
                   CHARGER_VOLTAGE_MIN_DV * 0.1f,
                   CHARGER_VOLTAGE_MAX_DV * 0.1f,
                   CHARGER_CURRENT_MAX_DA * 0.1f);
    Serial.println("========================================");
    Serial.println();
}

static void cmd_print_status(void) {
    Serial.println();
    Serial.println("------------ Status Report ------------");

    // Mode
    Serial.printf("  Mode:      %s\n", cmd_buddy_mode ? "BUDDY (external battery)" : "NORMAL (home 20S pack)");

    // Setpoints
    float set_v = cmd_voltage_dv * 0.1f;
    float set_a = cmd_current_da * 0.1f;
    float set_w = set_v * set_a;
    Serial.printf("  Setpoint:  %.1fV  %.1fA  (%.0fW)  [%s]\n",
                  set_v, set_a, set_w,
                  cmd_charging ? "CHARGING" : "DISABLED");

    // CAN bus
    Serial.printf("  CAN bus:   TX=GPIO%d RX=GPIO%d @ 250kbps\n",
                  CAN_TX_GPIO, CAN_RX_GPIO);

    // Charger response (CAN RX, smoothed for pulse charging)
    ChargerStatus_t cs;
    if (can_read_status(&cs)) {
        int   temp_f = (int)(cs.temperature_c * 9 / 5 + 32);
        int   watts  = (int)(can_smooth_voltage * can_smooth_current);
        Serial.printf("  Charger:   %.1fV  %.1fA  %dW  %dF  (smoothed avg)\n",
                      can_smooth_voltage, can_smooth_current, watts, temp_f);

        // Fault flags
        uint8_t any_fault = cs.fault.hardware_fault | cs.fault.temperature_fault |
                            cs.fault.output_undervoltage | cs.fault.output_overvoltage |
                            cs.fault.output_overcurrent | cs.fault.output_short_circuit |
                            (cs.fault.input_voltage_status ? 1 : 0);
        if (any_fault) {
            Serial.print("  Faults:   ");
            if (cs.fault.hardware_fault)       Serial.print(" HW_FAULT");
            if (cs.fault.temperature_fault)    Serial.print(" OVER_TEMP");
            if (cs.fault.input_voltage_status == 1) Serial.print(" AC_UNDER");
            if (cs.fault.input_voltage_status == 2) Serial.print(" AC_OVER");
            if (cs.fault.input_voltage_status == 3) Serial.print(" NO_AC_INPUT");
            if (cs.fault.output_undervoltage)  Serial.print(" OUT_UV");
            if (cs.fault.output_overvoltage)   Serial.print(" OUT_OV");
            if (cs.fault.output_overcurrent)   Serial.print(" OUT_OC");
            if (cs.fault.output_short_circuit) Serial.print(" SHORT");
            Serial.println();
        } else {
            Serial.println("  Faults:    None");
        }

        // Status flags
        const char *work_str = "??";
        switch (cs.status.working_status) {
            case 0: work_str = "Initializing"; break;
            case 1: work_str = "Working";      break;
            case 2: work_str = "Stopped";      break;
            case 3: work_str = "Standby";      break;
        }
        Serial.printf("  Work:      %s  Init:%s  Fan:%s  Pump:%s\n",
                      work_str,
                      cs.status.init_complete   ? "YES" : "NO",
                      cs.status.fan_on          ? "ON"  : "OFF",
                      cs.status.cooling_pump_on ? "ON"  : "OFF");
    } else {
        Serial.println("  Charger:   NO RESPONSE");
        Serial.println("             - Is AC power applied to charger?");
        Serial.println("             - Check CANH/CANL wiring");
        Serial.println("             - Verify 120 ohm termination");
        Serial.println("             - SN65HVD230 S pin must be tied to GND");
    }

    // Energy totals
    if (can_energy_wh > 0.1f) {
        uint32_t elapsed_s = can_charge_start_ms ? (millis() - can_charge_start_ms) / 1000 : 0;
        Serial.printf("  Energy:    %.1fWh  %.2fAh\n", can_energy_wh, can_energy_ah);
        Serial.printf("  Duration:  %lu min %lu sec\n",
                      (unsigned long)(elapsed_s / 60), (unsigned long)(elapsed_s % 60));
        if (elapsed_s > 0) {
            float avg_w = can_energy_wh / ((float)elapsed_s / 3600.0f);
            Serial.printf("  Avg power: %.0fW\n", avg_w);
        }
    } else {
        Serial.println("  Energy:    0.0Wh (type '0' to reset counters)");
    }

    Serial.println("---------------------------------------");
    Serial.println();
}

static void cmd_send_now(void) {
    // Send current setpoints to charger immediately
    if (cmd_charging) {
        ChargerCmd_t cmd;
        cmd.voltage_dv = cmd_voltage_dv;
        cmd.current_da = cmd_current_da;
        cmd.control    = CAN_CTRL_CHARGE;
        cmd.mode       = CAN_MODE_CHARGE;
        can_send_command(&cmd);
    } else {
        can_send_disable();
    }
}

static void serial_command_update(void) {
    // ---- Periodic status line (every 5s) ----
    uint32_t now = millis();
    if ((now - cmd_status_ms) >= CMD_STATUS_INTERVAL_MS) {
        cmd_status_ms = now;

        ChargerStatus_t cs;
        bool has_rx = can_read_status(&cs);

        Serial.printf("[STATUS] %s | Set:%.1fV/%.1fA %s | ",
                      cmd_buddy_mode ? "BUDDY" : "HOME",
                      cmd_voltage_dv * 0.1f,
                      cmd_current_da * 0.1f,
                      cmd_charging ? "ON" : "OFF");

        if (has_rx) {
            int watts = (int)(can_smooth_voltage * can_smooth_current);
            Serial.printf("Chgr:%.1fV/%.1fA %dW %dF %s",
                          can_smooth_voltage,
                          can_smooth_current,
                          watts,
                          (int)(cs.temperature_c * 9 / 5 + 32),
                          cs.status.working_status == 1 ? "WORKING" :
                          cs.status.working_status == 2 ? "STOPPED" :
                          cs.status.working_status == 3 ? "STANDBY" : "INIT");

            if (cs.fault.hardware_fault)       Serial.print(" !HW");
            if (cs.fault.temperature_fault)    Serial.print(" !TEMP");
            if (cs.fault.output_overvoltage)   Serial.print(" !OVP");
            if (cs.fault.output_overcurrent)   Serial.print(" !OCP");
            if (cs.fault.output_short_circuit) Serial.print(" !SHORT");
            if (cs.fault.input_voltage_status == 1) Serial.print(" !AC_LOW");
            if (cs.fault.input_voltage_status == 2) Serial.print(" !AC_HIGH");
            if (cs.fault.input_voltage_status == 3) Serial.print(" !NO_AC");
        } else {
            Serial.print("Chgr: NO RESPONSE");
        }

        // Energy totals
        if (can_energy_wh > 0.1f) {
            Serial.printf(" | %.1fWh %.2fAh", can_energy_wh, can_energy_ah);
        }
        Serial.println();
    }

    // ---- Heartbeat: re-send last command every CAN interval ----
    // (can_driver_update handles this via s_last_cmd, so we just
    //  ensure cmd_send_now() was called after any setpoint change)

    // ---- Process serial input ----
    while (Serial.available()) {
        char c = (char)Serial.read();

        switch (c) {

            // ---- Charge control ----
            case 'r':
                cmd_charging = true;
                can_energy_active = true;
                if (can_charge_start_ms == 0) can_charge_start_ms = millis();
                can_energy_last_ms = millis();
                cmd_send_now();
                Serial.printf("[CMD] RESUME charging: %.1fV / %.1fA (%.0fW)\n",
                              cmd_voltage_dv * 0.1f, cmd_current_da * 0.1f,
                              cmd_voltage_dv * 0.1f * cmd_current_da * 0.1f);
                break;

            case 's':
                cmd_charging = false;
                can_energy_active = false;
                can_send_disable();
                if (can_energy_wh > 0.1f) {
                    uint32_t elapsed_s = (millis() - can_charge_start_ms) / 1000;
                    Serial.printf("[CMD] STOP -- Charged %.1fWh / %.2fAh in %lu min %lu sec\n",
                                  can_energy_wh, can_energy_ah,
                                  (unsigned long)(elapsed_s / 60),
                                  (unsigned long)(elapsed_s % 60));
                } else {
                    Serial.println("[CMD] STOP -- output disabled (charger stays awake on CAN)");
                }
                break;

            case 'e':
                cmd_charging = false;
                can_energy_active = false;
                can_send_sleep();
                if (can_energy_wh > 0.1f) {
                    uint32_t elapsed_s = (millis() - can_charge_start_ms) / 1000;
                    Serial.printf("[CMD] END -- Charged %.1fWh / %.2fAh in %lu min %lu sec\n",
                                  can_energy_wh, can_energy_ah,
                                  (unsigned long)(elapsed_s / 60),
                                  (unsigned long)(elapsed_s % 60));
                }
                Serial.println("       Charger entering sleep. Needs AC power cycle to wake.");
                break;

            case 'w': {
                // Wake: send an immediate CAN frame (useful after AC power-on)
                ChargerCmd_t wake;
                wake.voltage_dv = cmd_voltage_dv;
                wake.current_da = 0;            // zero current, just wake the bus
                wake.control    = CAN_CTRL_DISABLE;  // don't charge yet, just handshake
                wake.mode       = CAN_MODE_CHARGE;
                can_send_command(&wake);
                Serial.println("[CMD] WAKE -- sent CAN frame to charger");
                Serial.println("       If charger was put to sleep (e), it needs AC power cycle first.");
                Serial.println("       If charger just powered on, this establishes CAN link.");
                break;
            }

            // ---- Current adjustment ----
            case '+':
                cmd_current_da += 10;  // +1.0A
                if (cmd_current_da > CHARGER_CURRENT_MAX_DA) {
                    cmd_current_da = CHARGER_CURRENT_MAX_DA;
                    Serial.printf("  ** Max current %.0fA\n", CHARGER_CURRENT_MAX_DA * 0.1f);
                }
                cmd_send_now();
                Serial.printf("[CMD] Current -> %.1fA (%.0fW at %.1fV)\n",
                              cmd_current_da * 0.1f,
                              cmd_voltage_dv * 0.1f * cmd_current_da * 0.1f,
                              cmd_voltage_dv * 0.1f);
                break;

            case '-':
                if (cmd_current_da >= 10) cmd_current_da -= 10;
                else cmd_current_da = 0;
                cmd_send_now();
                Serial.printf("[CMD] Current -> %.1fA (%.0fW at %.1fV)\n",
                              cmd_current_da * 0.1f,
                              cmd_voltage_dv * 0.1f * cmd_current_da * 0.1f,
                              cmd_voltage_dv * 0.1f);
                break;

            // ---- Voltage adjustment ----
            case 'v':
                cmd_voltage_dv += 5;  // +0.5V
                if (cmd_voltage_dv > CHARGER_VOLTAGE_MAX_DV) {
                    cmd_voltage_dv = CHARGER_VOLTAGE_MAX_DV;
                    Serial.printf("  ** Max voltage %.1fV\n", CHARGER_VOLTAGE_MAX_DV * 0.1f);
                }
                cmd_send_now();
                Serial.printf("[CMD] Voltage -> %.1fV\n", cmd_voltage_dv * 0.1f);
                break;

            case 'V':
                if (cmd_voltage_dv > 5) cmd_voltage_dv -= 5;
                else cmd_voltage_dv = 0;
                if (cmd_voltage_dv < CHARGER_VOLTAGE_MIN_DV) {
                    cmd_voltage_dv = CHARGER_VOLTAGE_MIN_DV;
                    Serial.printf("  ** Min voltage %.1fV\n", CHARGER_VOLTAGE_MIN_DV * 0.1f);
                }
                cmd_send_now();
                Serial.printf("[CMD] Voltage -> %.1fV\n", cmd_voltage_dv * 0.1f);
                break;

            // ---- Buddy mode ----
            case 'b':
                if (!cmd_buddy_mode) {
                    cmd_buddy_mode = true;
                    cmd_charging   = false;
                    can_send_disable();
                    Serial.println();
                    Serial.println("=== BUDDY MODE ACTIVATED ===");
                    Serial.println("  Charger output DISABLED for safety.");
                    Serial.println("  1. Disconnect from your pack");
                    Serial.println("  2. Connect to buddy's battery");
                    Serial.printf( "  3. Adjust voltage with v/V (currently %.1fV)\n", cmd_voltage_dv * 0.1f);
                    Serial.printf( "  4. Adjust current with +/- (currently %.1fA)\n", cmd_current_da * 0.1f);
                    Serial.println("  5. Type 'r' to start charging");
                    Serial.println("  6. Type 'n' when done to restore home settings");
                    Serial.printf( "  Voltage range: %.1fV - %.1fV\n",
                                   CHARGER_VOLTAGE_MIN_DV * 0.1f, CHARGER_VOLTAGE_MAX_DV * 0.1f);
                    Serial.println("============================");
                    Serial.println();
                } else {
                    Serial.println("[CMD] Already in buddy mode. Type 'n' to exit.");
                }
                break;

            case 'n':
                if (cmd_buddy_mode) {
                    cmd_charging   = false;
                    can_energy_active = false;
                    cmd_buddy_mode = false;
                    cmd_voltage_dv = HOME_VOLTAGE_DV;
                    cmd_current_da = HOME_CURRENT_DA;
                    can_send_disable();
                    Serial.println();
                    Serial.println("=== NORMAL MODE RESTORED ===");
                    if (can_energy_wh > 0.1f) {
                        uint32_t elapsed_s = can_charge_start_ms ? (millis() - can_charge_start_ms) / 1000 : 0;
                        Serial.printf( "  Buddy session: %.1fWh / %.2fAh in %lu min %lu sec\n",
                                       can_energy_wh, can_energy_ah,
                                       (unsigned long)(elapsed_s / 60),
                                       (unsigned long)(elapsed_s % 60));
                    }
                    // Reset energy for next session
                    can_energy_wh = 0.0f;
                    can_energy_ah = 0.0f;
                    can_charge_start_ms = 0;
                    Serial.printf( "  Settings: %.1fV / %.1fA (home 20S pack)\n",
                                   cmd_voltage_dv * 0.1f, cmd_current_da * 0.1f);
                    Serial.println("  Energy counters reset. Reconnect your pack, type 'r' to charge.");
                    Serial.println("============================");
                    Serial.println();
                } else {
                    Serial.println("[CMD] Already in normal mode.");
                }
                break;

            // ---- Energy reset ----
            case '0':
                can_energy_wh = 0.0f;
                can_energy_ah = 0.0f;
                can_charge_start_ms = 0;
                Serial.println("[CMD] Energy counters reset to zero");
                break;

            // ---- Info ----
            case 'p':
                cmd_print_status();
                break;

            case '?':
                cmd_print_help();
                break;

            case '\r':
            case '\n':
                break;

            default:
                Serial.printf("[CMD] Unknown '%c' -- type '?' for help\n", c);
                break;
        }
    }
}

// =============================================================================
//  SETUP & LOOP
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  TC_Charger Main Brain");
    Serial.println("  HK-LF-115-58 6.6kW OBC Controller");
    Serial.println("========================================");
    Serial.println();
    Serial.printf("  CAN bus:   GPIO%d TX / GPIO%d RX @ 250kbps\n", CAN_TX_GPIO, CAN_RX_GPIO);
    Serial.printf("  Home pack: %.1fV / %.1fA (20S x 4.20V)\n",
                  HOME_VOLTAGE_DV * 0.1f, HOME_CURRENT_DA * 0.1f);
    Serial.printf("  Charger:   %.1fV - %.1fV, max %.0fA\n",
                  CHARGER_VOLTAGE_MIN_DV * 0.1f,
                  CHARGER_VOLTAGE_MAX_DV * 0.1f,
                  CHARGER_CURRENT_MAX_DA * 0.1f);
    Serial.println();
    Serial.println("  Charger is DISABLED on boot.");
    Serial.println("  Type 'r' to start charging, '?' for all commands");
    Serial.println("  Type 'w' after plugging in AC to wake charger");
    Serial.println("  Type 'b' for buddy mode (charge another battery)");
    Serial.println();

    esp_task_wdt_config_t wdt_cfg;
    wdt_cfg.timeout_ms     = WDT_TIMEOUT_S * 1000;
    wdt_cfg.idle_core_mask = 0;
    wdt_cfg.trigger_panic  = true;
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

    // Start with charger disabled
    can_send_disable();
    Serial.println("[BOOT] All drivers initialized. Charger DISABLED.");
    Serial.println();
}

void loop() {
    esp_task_wdt_reset();
    button_driver_update();
    safety_sm_update();
    serial_command_update();
    rs485_poll();
    thermal_logic_update();
    rgb_led_update();
    ble_server_update();
    can_driver_update();
    oled_display_update();
    bme280_update();
    gps_driver_update();
}
