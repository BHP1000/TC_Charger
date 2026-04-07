// =============================================================================
// TC_Charger -- Main Brain (ESP32-S3) Consolidated Production Firmware
// Version:   2.0.0  (2026-04-06)
// =============================================================================
//
// Board:  ESP32S3 Dev Module
// Flash:  16 MB
// PSRAM:  OPI
// Upload: COM3 @ 921600
// Serial: 115200
//
// Required Libraries (install via Arduino Library Manager):
//   - NimBLE-Arduino  (BLE server + TPMS scanner)
//   - FastLED
//   - U8g2
//   - Adafruit AHTX0
//   - Adafruit BMP280 Library
//   - TinyGPSPlus
//   - PubSubClient  (MQTT for Home Assistant)
//
// ============================================================
//  CHANGELOG
// ============================================================
//
//  v2.0.0  (2026-04-06)
//    - Migrated BLE from stock ESP32 library to NimBLE-Arduino 2.5.0
//    - Added TPMS BLE scanner (passive scan, dual protocol decoder)
//    - TPMS discovery mode (TPMSD), MAC assignment (TPMSF/TPMSR), NVS persist
//    - Added WiFi STA + MQTT for Home Assistant (PubSubClient)
//    - HA auto-discovery (8 sensor entities published on first connect)
//    - WiFi credential commands (WS/WP/WH/WM/WI/WC/WD)
//    - Added direct value set commands (SV/SA/SM/SR/SP/SO)
//    - Fixed BLE write command handler (getValue -> NimBLEAttValue)
//    - Fixed CAN bus-off recovery (TWAI state machine auto-reinit)
//    - BLE MTU 256 negotiation (NimBLEDevice::setMTU)
//    - COMM_INTERFACE_BLUEPRINT.md created (full integration reference)
//
//  v1.0.0  (2026-04-04)
//    - Initial consolidated firmware (stock ESP32 BLE library)
//    - CAN charger control (HK-LF-115-58 6.6kW OBC)
//    - Serial command interface (single + multi-char)
//    - Charge profiles (Manual/Storage/80/85/90/100%)
//    - Buddy mode, maintain mode, scheduled charging
//    - Outlet presets with power clamping
//    - Thermal zone protection with linear derating
//    - Energy tracking (session/lifetime/buddy) with cost
//    - WiFi AP with JSON API (/status, /cmd, /events)
//    - BLE server with 11 characteristics at 1Hz
//    - OLED display (7 pages), RGB LED, GPS, compass
//    - RS485 to Temp Node daughter board
//    - CAN inter-module commands (emergency stop, start/stop, set V/A)
//    - NVS persistent settings
//
// =============================================================================

#include <esp_task_wdt.h>
#include <driver/twai.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FastLED.h>
#include <NimBLEDevice.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include <PubSubClient.h>

// =============================================================================
//  FIRMWARE VERSION
// =============================================================================

#define FW_VERSION "2.0.0"

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

// CAN inter-module frame IDs
#define CAN_EMERGENCY_STOP_ID    0x00010000UL  // highest priority on bus
#define CAN_INTERMOD_CMD_ID      0x1A000001UL  // module -> Main Brain command
#define CAN_INTERMOD_STATUS_ID   0x1A000002UL  // Main Brain -> modules broadcast

// Inter-module command bytes (Byte 0 of CAN_INTERMOD_CMD_ID frame)
#define INTERMOD_CMD_ESTOP       0x00
#define INTERMOD_CMD_START       0x01
#define INTERMOD_CMD_STOP        0x02
#define INTERMOD_CMD_SET_AMPS    0x03  // Byte 1-2: current_da big-endian
#define INTERMOD_CMD_SET_VOLTS   0x04  // Byte 1-2: voltage_dv big-endian
#define INTERMOD_CMD_SET_PROFILE 0x05  // Byte 1: profile enum

// TPMS data from VESC module (CAN broadcast)
#define CAN_TPMS_DATA_ID         0x1C000010UL  // VESC -> all: tire pressure/temp

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
#define CHAR_CMD_UUID           "beb5483e-36e1-4688-b7f5-ea07361b26b0"   // command input (write)
#define CHAR_CHARGER_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26b1"   // charger telemetry (notify)

// ---- WiFi AP ----
#define WIFI_AP_SSID       "ScooterCharger"
#define WIFI_AP_PASS       "charge84v"
#define WIFI_AP_PORT       80

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
    SAFETY_STATE_UNPLUGGED,     // no CAN response from charger
    SAFETY_STATE_IDLE,          // charger connected, output disabled
    SAFETY_STATE_CHARGING,
    SAFETY_STATE_COMPLETE,      // auto-stopped at profile target
    SAFETY_STATE_MAINTAINING,   // battery tender mode, watching voltage
    SAFETY_STATE_SCHEDULED,     // waiting for scheduled charge start time
    SAFETY_STATE_OVERTEMP,
    SAFETY_STATE_UNDERTEMP,
    SAFETY_STATE_BMS_FAULT,
    SAFETY_STATE_COMM_TIMEOUT,
    SAFETY_STATE_FAULT,
} SafetyState_t;

// ---- Charge profiles ----
// Voltage-based stop points for 20S pack (adjustable per-cell voltage)
// SOC approximations for typical Li-ion NMC:
//   Storage ~3.80V/cell, 80% ~3.92V, 85% ~3.96V, 90% ~4.00V, 100% 4.20V
typedef enum {
    PROFILE_MANUAL = 0,    // no auto-stop, user controls everything
    PROFILE_STORAGE,       // stop at storage voltage
    PROFILE_80,            // stop at ~80% SOC
    PROFILE_85,            // stop at ~85% SOC
    PROFILE_90,            // stop at ~90% SOC
    PROFILE_100,           // stop at 100% (full charge)
    PROFILE_CUSTOM,        // user-set stop voltage
    PROFILE_COUNT
} ChargeProfile_t;

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
//  FORWARD DECLARATIONS (must be before command state -- functions used early)
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

// Serial/BLE/WiFi command processor
static void cmd_process_char(char c);
static void cmd_execute_multi(const char *buf, uint8_t len);
static void cmd_send_now(void);

// CAN inter-module
static void can_process_intermod(const twai_message_t *msg);

// Safety state (declared early -- used by CAN driver emergency stop)
static SafetyState_t safety_state = SAFETY_STATE_INIT;

// =============================================================================
//  CHARGER COMMAND STATE (used by OLED display + serial command handler)
// =============================================================================

#define HOME_VOLTAGE_DV      840
#define HOME_CURRENT_DA       20
#define HOME_CELLS            20
#define CHARGER_VOLTAGE_MAX_DV  1080
#define CHARGER_VOLTAGE_MIN_DV   200
#define CHARGER_CURRENT_MAX_DA   600

static uint16_t cmd_voltage_dv   = HOME_VOLTAGE_DV;
static uint16_t cmd_current_da   = HOME_CURRENT_DA;
static bool     cmd_charging     = false;
static bool     cmd_buddy_mode   = false;
static bool     cmd_charger_present = false;  // true when CAN RX is active

// ---- Charge profile state ----
static ChargeProfile_t cmd_profile = PROFILE_MANUAL;
static uint16_t cmd_custom_stop_dv = HOME_VOLTAGE_DV;  // custom stop voltage

// Per-cell voltages for each profile (multiply by HOME_CELLS for pack voltage)
// These are adjustable -- stored in NVS
static uint16_t profile_cell_mv[PROFILE_COUNT] = {
    0,       // MANUAL -- no auto-stop
    3800,    // STORAGE -- 3.80V/cell
    3920,    // 80% -- 3.92V/cell
    3960,    // 85% -- 3.96V/cell
    4000,    // 90% -- 4.00V/cell
    4200,    // 100% -- 4.20V/cell
    0,       // CUSTOM -- uses cmd_custom_stop_dv directly
};

static const char *profile_name(ChargeProfile_t p) {
    switch (p) {
        case PROFILE_MANUAL:  return "MANUAL";
        case PROFILE_STORAGE: return "STORAGE";
        case PROFILE_80:      return "80%";
        case PROFILE_85:      return "85%";
        case PROFILE_90:      return "90%";
        case PROFILE_100:     return "100%";
        case PROFILE_CUSTOM:  return "CUSTOM";
        default:              return "??";
    }
}

static uint16_t profile_stop_voltage_dv(void) {
    if (cmd_profile == PROFILE_MANUAL) return 0;  // no auto-stop
    if (cmd_profile == PROFILE_CUSTOM) return cmd_custom_stop_dv;
    return (uint16_t)((uint32_t)profile_cell_mv[cmd_profile] * HOME_CELLS / 100);
}

// ---- Maintenance mode (battery tender) ----
// After reaching profile target, keep charger plugged in.
// When voltage drops by maintain_drop_pct (default 5%), top back up.
static bool     maintain_enabled     = false;
static float    maintain_drop_pct    = 5.0f;   // start top-up after this % SOC drop
static bool     maintain_waiting     = false;  // true = watching voltage, false = topping up
static uint32_t maintain_check_ms    = 0;
#define MAINTAIN_CHECK_INTERVAL_MS  30000UL    // check voltage every 30 seconds

// Approximate SOC from voltage (20S NMC, linear approximation)
// Full = 84.0V (4.20V/cell), Empty = 60.0V (3.00V/cell)
static float voltage_to_soc_pct(float pack_v) {
    float cell_v = pack_v / (float)HOME_CELLS;
    if (cell_v >= 4.20f) return 100.0f;
    if (cell_v <= 3.00f) return 0.0f;
    return (cell_v - 3.00f) / (4.20f - 3.00f) * 100.0f;
}

// ---- Scheduled charging ----
// "Be at profile X by time Y" -- firmware calculates when to start
static bool     schedule_enabled     = false;
static uint8_t  schedule_target_hour = 8;      // target completion hour (0-23)
static uint8_t  schedule_target_min  = 0;      // target completion minute (0-59)
static ChargeProfile_t schedule_profile = PROFILE_100;
static bool     schedule_triggered   = false;  // true once auto-start fires
static float    schedule_avg_watts   = 0.0f;   // learned average charge power

// ---- Outlet presets ----
typedef enum {
    OUTLET_110_15A = 0,   // A: Standard US outlet
    OUTLET_110_20A,       // B: Kitchen/garage
    OUTLET_110_30A,       // C: Max charger on 110V
    OUTLET_220_15A,       // D: Standard 220V
    OUTLET_220_30A,       // E: Dryer/welder circuit
    OUTLET_EV_STATION,    // G: EV station (user sets amps, CP bypassed w/ dummy resistor)
    OUTLET_USER_DEFINED,  // U: User sets voltage + amps
    OUTLET_TEST_DEV,      // F: TEST/DEV -- no limits
    OUTLET_COUNT
} OutletPreset_t;

static OutletPreset_t outlet_preset = OUTLET_110_15A;  // default, loaded from NVS
static uint16_t outlet_voltage_ac   = 110;   // AC voltage for user-defined
static uint16_t outlet_amps_ac      = 15;    // AC amps for user-defined / EV station

static uint16_t outlet_max_watts(void) {
    switch (outlet_preset) {
        case OUTLET_110_15A:     return 1650;
        case OUTLET_110_20A:     return 2200;
        case OUTLET_110_30A:     return 3300;
        case OUTLET_220_15A:     return 3300;
        case OUTLET_220_30A:     return 6600;
        case OUTLET_EV_STATION:  return (uint16_t)((uint32_t)220 * outlet_amps_ac);
        case OUTLET_USER_DEFINED: return (uint16_t)((uint32_t)outlet_voltage_ac * outlet_amps_ac);
        case OUTLET_TEST_DEV:    return 0;  // 0 = no limit
        default:                 return 1650;
    }
}

static const char *outlet_name(void) {
    switch (outlet_preset) {
        case OUTLET_110_15A:      return "110V/15A";
        case OUTLET_110_20A:      return "110V/20A";
        case OUTLET_110_30A:      return "110V/30A";
        case OUTLET_220_15A:      return "220V/15A";
        case OUTLET_220_30A:      return "220V/30A";
        case OUTLET_EV_STATION:   return "EV Station";
        case OUTLET_USER_DEFINED: return "User Def";
        case OUTLET_TEST_DEV:     return "TEST/DEV";
        default:                  return "??";
    }
}

// Clamp charge current to outlet power limit
static uint16_t outlet_clamp_current_da(uint16_t requested_da) {
    uint16_t max_w = outlet_max_watts();
    if (max_w == 0) return requested_da;  // TEST/DEV, no limit
    float max_a = (float)max_w / (cmd_voltage_dv * 0.1f);
    uint16_t max_da = (uint16_t)(max_a * 10.0f);
    return (requested_da < max_da) ? requested_da : max_da;
}

// ---- TEST/DEV mode ----
static bool test_dev_mode = false;  // follows OUTLET_TEST_DEV selection

// ---- Thermal zones (battery temps only, charger temp excluded) ----
// Temps in Fahrenheit, adjustable, saved to NVS
static float tz_normal_max_f   = 140.0f;  // below = full rate (20-60C)
static float tz_caution_max_f  = 194.0f;  // 60-90C: degradation onset, puffing
static float tz_danger_max_f   = 248.0f;  // 90-120C: cell damage, heavy derate
                                           // above danger = CRITICAL: immediate shutdown

// Derating: linear reduction through CAUTION and DANGER zones
// Returns multiplier 0.0 to 1.0
static float thermal_zone_multiplier(float max_temp_f) {
    if (test_dev_mode) return 1.0f;  // TEST/DEV disables thermal
    if (max_temp_f < 32.0f) return 0.0f;  // below freezing, no charge
    if (max_temp_f <= tz_normal_max_f) return 1.0f;  // full rate
    if (max_temp_f >= tz_danger_max_f) return 0.0f;  // CRITICAL shutdown
    if (max_temp_f <= tz_caution_max_f) {
        // CAUTION: linear derate from 100% to 50%
        float pct = (max_temp_f - tz_normal_max_f) / (tz_caution_max_f - tz_normal_max_f);
        return 1.0f - pct * 0.5f;
    }
    // DANGER: linear derate from 50% to 0%
    float pct = (max_temp_f - tz_caution_max_f) / (tz_danger_max_f - tz_caution_max_f);
    return 0.5f - pct * 0.5f;
}

static const char *thermal_zone_name(float max_temp_f) {
    if (max_temp_f < 32.0f) return "FREEZE";
    if (max_temp_f <= tz_normal_max_f) return "NORMAL";
    if (max_temp_f <= tz_caution_max_f) return "CAUTION";
    if (max_temp_f <= tz_danger_max_f) return "DANGER";
    return "CRITICAL";
}

// ---- Per-sensor enable/disable + calibration ----
// NTC T1-T5 (indices 1-5, index 0 unused on C3 Super Mini)
static bool  sensor_ntc_enabled[6]  = {false, true, true, true, true, true};
static float sensor_ntc_offset_f[6] = {0, 0, 0, 0, 0, 0};  // calibration offset in F

// BMS temps
static bool  sensor_bms1_enabled = true;
static bool  sensor_bms2_enabled = true;
static float sensor_bms1_offset_f = 0.0f;
static float sensor_bms2_offset_f = 0.0f;

// Get highest enabled battery temperature (F). Returns -999 if none available.
static float get_max_battery_temp_f(void) {
    float mx = -999.0f;

    // NTC sensors from Temp Node
    TempNodeData_t td;
    if (rs485_get_last_temp_node(&td)) {
        for (int i = 1; i <= 5; i++) {
            if (sensor_ntc_enabled[i] && (td.valid_mask & (1 << i))) {
                float tf = td.temp_c[i] * 9.0f / 5.0f + 32.0f + sensor_ntc_offset_f[i];
                if (tf > mx) mx = tf;
            }
        }
    }

    // BMS temps
    BMSData_t bms;
    if (rs485_get_last_bms(&bms) && bms.valid) {
        if (sensor_bms1_enabled) {
            float tf = (bms.bat_temp1_dc / 10.0f) * 9.0f / 5.0f + 32.0f + sensor_bms1_offset_f;
            if (tf > mx) mx = tf;
        }
        if (sensor_bms2_enabled) {
            float tf = (bms.bat_temp2_dc / 10.0f) * 9.0f / 5.0f + 32.0f + sensor_bms2_offset_f;
            if (tf > mx) mx = tf;
        }
    }

    return mx;
}

// ---- Input power tracking ----
static float    input_peak_watts     = 0.0f;

// ---- Time source ----
static bool     time_valid           = false;
static uint8_t  time_hour            = 0;
static uint8_t  time_minute          = 0;
static uint32_t time_update_ms       = 0;

// ---- Electricity cost tracking ----
static float cost_per_kwh       = 0.12f;   // default $0.12/kWh (home)
static float cost_session       = 0.0f;    // current session cost
static float cost_lifetime      = 0.0f;    // lifetime cost (NVS)
static float cost_free_lifetime = 0.0f;    // energy from free chargers (NVS)

// Per-outlet cost rates ($/kWh). Free = 0, home default, station rates.
static float outlet_cost_rate(void) {
    switch (outlet_preset) {
        case OUTLET_EV_STATION: return 0.0f;  // assume free public station by default
        case OUTLET_TEST_DEV:   return 0.0f;
        default:                return cost_per_kwh;
    }
}

// ---- Charge completion notification ----
// BLE notification + RGB LED flash (no buzzer)
static bool notify_sent       = false;  // prevent repeat notifications

// ---- Thermal prediction ----
// Track rate of temperature rise to predict when derating will start
static float thermal_rise_rate_f_per_min = 0.0f;   // learned F/minute at current amps
static float thermal_last_temp_f         = -999.0f;
static uint32_t thermal_predict_ms       = 0;
#define THERMAL_PREDICT_INTERVAL_MS  60000UL  // sample temp every 60 seconds

static float thermal_time_to_caution_min(void) {
    // Estimate minutes until CAUTION zone at current rise rate
    float max_f = get_max_battery_temp_f();
    if (max_f < -900.0f || thermal_rise_rate_f_per_min <= 0.01f) return -1.0f;
    float degrees_left = tz_normal_max_f - max_f;
    if (degrees_left <= 0.0f) return 0.0f;
    return degrees_left / thermal_rise_rate_f_per_min;
}

// ---- GPS source selection ----
typedef enum {
    GPS_SOURCE_ONBOARD = 0,   // HGLRC M100 on UART2
    GPS_SOURCE_PHONE,         // received via BLE/WiFi from app
    GPS_SOURCE_BOTH,          // fused: phone primary, onboard fallback
} GPSSource_t;

static GPSSource_t gps_source = GPS_SOURCE_BOTH;  // default: use both

// Phone GPS data (received from app)
static float phone_gps_lat    = 0.0f;
static float phone_gps_lon    = 0.0f;
static float phone_gps_speed  = 0.0f;  // mph
static bool  phone_gps_valid  = false;
static uint32_t phone_gps_ms  = 0;
#define PHONE_GPS_TIMEOUT_MS  5000UL

// ---- TPMS (received from VESC module over CAN) ----
static bool     tpms_enabled      = false;   // toggleable
static float    tpms_front_psi    = 0.0f;
static float    tpms_front_temp_f = 0.0f;
static float    tpms_rear_psi     = 0.0f;
static float    tpms_rear_temp_f  = 0.0f;
static bool     tpms_front_valid  = false;
static bool     tpms_rear_valid   = false;
static uint32_t tpms_last_ms      = 0;
#define TPMS_TIMEOUT_MS  600000UL  // sensor lost if no data for 10 min (sensors sleep ~8 min)

// ---- WiFi Station mode (home network for Home Assistant MQTT) ----
#define WIFI_STA_RETRY_MS       30000UL   // retry connection every 30s
#define MQTT_PUBLISH_MS         5000UL    // publish telemetry every 5s
#define MQTT_TOPIC_STATE        "scooter_charger/state"
#define MQTT_TOPIC_CMD          "scooter_charger/cmd"
#define MQTT_TOPIC_DISCOVERY    "homeassistant/sensor/scooter_charger"
#define MQTT_CLIENT_ID          "ScooterCharger"

static char     wifi_sta_ssid[33]     = "";   // NVS
static char     wifi_sta_pass[65]     = "";   // NVS
static char     mqtt_host[64]         = "";   // NVS
static uint16_t mqtt_port             = 1883; // NVS
static bool     wifi_sta_connected    = false;
static uint32_t wifi_sta_retry_ms     = 0;
static uint32_t mqtt_last_publish_ms  = 0;
static bool     mqtt_discovery_sent   = false;

static WiFiClient mqtt_wifi_client;
static PubSubClient mqtt_client(mqtt_wifi_client);

// Forward declarations for WiFi STA + MQTT
static void wifi_sta_connect(void);
static void mqtt_callback(char *topic, byte *payload, unsigned int length);
static void mqtt_publish_state(void);
static void mqtt_send_discovery(void);
static void wifi_sta_update(void);

// ---- Session summary for app logging (BLE characteristic) ----
// Packed into existing charger telemetry + new session end characteristic
#define CHAR_SESSION_END_UUID "beb5483e-36e1-4688-b7f5-ea07361b26b2"

// ---- Multi-char command buffer ----
#define CMD_BUF_SIZE 32
static char  cmd_buf[CMD_BUF_SIZE];
static uint8_t cmd_buf_len = 0;
static bool    cmd_buf_active = false;  // true when collecting multi-char command

// =============================================================================
//  CAN DRIVER
// =============================================================================

static bool            can_initialized      = false;
static ChargerCmd_t    can_last_cmd         = {0, 0, CAN_CTRL_DISABLE, CAN_MODE_CHARGE};
static ChargerStatus_t can_charger_status   = {};
static uint32_t        can_last_tx_ms       = 0;
static bool            can_bus_recovering   = false;   // TWAI bus-off recovery in progress
static uint32_t        can_recovery_ms      = 0;       // when recovery started
static uint32_t        can_last_rx_ms       = 0;

// Smoothed values for display (EMA alpha=0.1, smooths pulse charging spikes)
#define CAN_EMA_ALPHA  0.1f
static float    can_smooth_voltage  = 0.0f;
static float    can_smooth_current  = 0.0f;
static bool     can_smooth_primed   = false;

// ---- Energy tracking ----
// Current session (resets each time 'r' is pressed after stop)
static float    nrg_session_wh      = 0.0f;
static float    nrg_session_ah      = 0.0f;
static uint32_t nrg_session_start   = 0;

// Previous session (overwritten when new session starts)
static float    nrg_prev_wh         = 0.0f;
static float    nrg_prev_ah         = 0.0f;
static uint32_t nrg_prev_duration_s = 0;

// Lifetime (persisted to NVS flash across reboots)
static float    nrg_lifetime_wh     = 0.0f;
static float    nrg_lifetime_ah     = 0.0f;
static Preferences nrg_prefs;
static uint32_t nrg_save_ms         = 0;
#define NRG_SAVE_INTERVAL_MS  60000UL  // save to NVS every 60 seconds

// Buddy session (completely separate, never counts toward mine)
static float    nrg_buddy_wh        = 0.0f;
static float    nrg_buddy_ah        = 0.0f;
static uint32_t nrg_buddy_start     = 0;

// Integration state
static uint32_t nrg_last_ms         = 0;
static bool     nrg_active          = false;

// ---- Range estimation ----
#define WH_PER_MILE_DEFAULT  32.0f   // ~20 Wh/km = 32 Wh/mile
static float    range_wh_per_mile   = WH_PER_MILE_DEFAULT;

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
        nrg_last_ms = millis();
    } else {
        can_smooth_voltage += CAN_EMA_ALPHA * (raw_v - can_smooth_voltage);
        can_smooth_current += CAN_EMA_ALPHA * (raw_a - can_smooth_current);
    }

    // Energy integration using smoothed values (accurate for pulse charging)
    if (nrg_active && can_smooth_current > 0.05f) {
        uint32_t now = millis();
        float dt_hours = (float)(now - nrg_last_ms) / 3600000.0f;
        float dWh = can_smooth_voltage * can_smooth_current * dt_hours;
        float dAh = can_smooth_current * dt_hours;

        if (cmd_buddy_mode) {
            nrg_buddy_wh += dWh;
            nrg_buddy_ah += dAh;
        } else {
            nrg_session_wh  += dWh;
            nrg_session_ah  += dAh;
            nrg_lifetime_wh += dWh;
            nrg_lifetime_ah += dAh;
            // Cost tracking
            float dCost = dWh * outlet_cost_rate() / 1000.0f;  // $/Wh to $/kWh
            cost_session += dCost;
            if (outlet_cost_rate() > 0.001f)
                cost_lifetime += dCost;
            else
                cost_free_lifetime += dWh / 1000.0f;  // track free kWh
        }
        nrg_last_ms = now;
    } else {
        nrg_last_ms = millis();
    }
}

void can_driver_init(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_GPIO, (gpio_num_t)CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_250KBITS();
    // Accept all frames -- we process charger, emergency, and inter-module IDs
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err;
    err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) { Serial.printf("[CAN] driver install failed: %d\n", err); return; }

    err = twai_start();
    if (err != ESP_OK) { Serial.printf("[CAN] start failed: %d\n", err); return; }

    can_initialized = true;
    can_last_tx_ms  = millis();
    can_last_rx_ms  = millis();
    Serial.printf("[CAN] init OK - TX GPIO%d RX GPIO%d @ 250kbps\n", CAN_TX_GPIO, CAN_RX_GPIO);
}

bool can_send_disable(void) {
    ChargerCmd_t c = {0, 0, CAN_CTRL_DISABLE, CAN_MODE_CHARGE};
    can_last_cmd = c;
    if (!can_initialized) return false;
    return twai_send_frame(&can_last_cmd);
}

bool can_send_sleep(void) {
    ChargerCmd_t c = {0, 0, CAN_CTRL_SLEEP, CAN_MODE_CHARGE};
    can_last_cmd = c;
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

    // --- TWAI bus-off recovery ---
    // If the TWAI peripheral enters bus-off (e.g., noise during charger plug-in),
    // it stops all TX/RX. We must detect this and recover automatically so the
    // charger can be detected after hot-plug without rebooting the ESP32.
    twai_status_info_t twai_info;
    if (twai_get_status_info(&twai_info) == ESP_OK) {
        if (twai_info.state == TWAI_STATE_BUS_OFF) {
            if (!can_bus_recovering) {
                can_bus_recovering = true;
                can_recovery_ms = millis();
                twai_initiate_recovery();
                Serial.println("[CAN] Bus-off detected -- initiating recovery");
            } else if ((millis() - can_recovery_ms) > 2000) {
                // Recovery taking too long -- full reinit
                can_bus_recovering = false;
                twai_stop();
                twai_driver_uninstall();
                can_initialized = false;
                Serial.println("[CAN] Recovery timeout -- reinitializing TWAI");
                can_driver_init();
            }
            return;  // skip TX/RX while in bus-off
        } else if (twai_info.state == TWAI_STATE_RECOVERING) {
            // Still recovering -- check timeout but don't call twai_start yet
            if (can_bus_recovering && (millis() - can_recovery_ms) > 2000) {
                can_bus_recovering = false;
                twai_stop();
                twai_driver_uninstall();
                can_initialized = false;
                Serial.println("[CAN] Recovery timeout -- reinitializing TWAI");
                can_driver_init();
            }
            return;  // skip TX/RX while recovering
        } else if (can_bus_recovering) {
            // Recovery complete (state is STOPPED) -- restart TWAI
            can_bus_recovering = false;
            twai_start();
            can_last_tx_ms = millis();
            can_last_rx_ms = millis();
            Serial.println("[CAN] Bus recovery complete -- resuming");
        }
    }

    if ((millis() - can_last_tx_ms) >= CAN_TX_INTERVAL_MS) {
        twai_send_frame(&can_last_cmd);
    }

    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        if (!msg.extd) continue;
        if (msg.identifier == CAN_CHARGER_RX_ID) {
            parse_rx_frame(&msg);
        } else if (msg.identifier == CAN_EMERGENCY_STOP_ID) {
            cmd_charging = false;
            nrg_active = false;
            can_send_disable();
            safety_state = SAFETY_STATE_FAULT;
            rgb_led_set_state(RGB_STATE_FAULT);
            Serial.println("[CAN] *** EMERGENCY STOP ***");
        } else if (msg.identifier == CAN_INTERMOD_CMD_ID) {
            can_process_intermod(&msg);
        } else if (msg.identifier == CAN_TPMS_DATA_ID && tpms_enabled) {
            // TPMS frame from VESC: byte0=wheel(0=front,1=rear), 1-2=psi*100, 3-4=temp_f*10
            if (msg.data_length_code >= 5) {
                uint8_t wheel = msg.data[0];
                float psi = ((uint16_t)msg.data[1] << 8 | msg.data[2]) / 100.0f;
                float temp = ((uint16_t)msg.data[3] << 8 | msg.data[4]) / 10.0f;
                if (wheel == 0) {
                    tpms_front_psi = psi; tpms_front_temp_f = temp; tpms_front_valid = true;
                } else {
                    tpms_rear_psi = psi; tpms_rear_temp_f = temp; tpms_rear_valid = true;
                }
                tpms_last_ms = millis();
            }
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
    if ((millis() - b->last_change_ms) < BTN_DEBOUNCE_MS) return;
    b->last_raw = raw;
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

const char *safety_state_name(SafetyState_t s) {
    switch (s) {
        case SAFETY_STATE_INIT:         return "INIT";
        case SAFETY_STATE_UNPLUGGED:    return "UNPLUGGED";
        case SAFETY_STATE_IDLE:         return "READY";
        case SAFETY_STATE_CHARGING:     return "CHARGING";
        case SAFETY_STATE_COMPLETE:     return "COMPLETE";
        case SAFETY_STATE_MAINTAINING:  return "MAINTAIN";
        case SAFETY_STATE_SCHEDULED:    return "SCHEDULED";
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
        case SAFETY_STATE_UNPLUGGED:    return RGB_STATE_BOOT;        // white slow blink
        case SAFETY_STATE_CHARGING:     return RGB_STATE_CHARGING;    // green solid
        case SAFETY_STATE_COMPLETE:     return RGB_STATE_IDLE;        // blue slow blink
        case SAFETY_STATE_MAINTAINING:  return RGB_STATE_IDLE;        // blue slow blink
        case SAFETY_STATE_SCHEDULED:    return RGB_STATE_COMM_TIMEOUT; // yellow blink
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

static NimBLEServer           *ble_server_ptr      = nullptr;
static NimBLECharacteristic   *ble_ch_state    = nullptr;
static NimBLECharacteristic   *ble_ch_temp     = nullptr;
static NimBLECharacteristic   *ble_ch_cur      = nullptr;
static NimBLECharacteristic   *ble_ch_voltage  = nullptr;
static NimBLECharacteristic   *ble_ch_bms_pack    = nullptr;
static NimBLECharacteristic   *ble_ch_bms_health  = nullptr;
static NimBLECharacteristic   *ble_ch_bms_cells   = nullptr;
static NimBLECharacteristic   *ble_ch_gps         = nullptr;
static NimBLECharacteristic   *ble_ch_cmd         = nullptr;
static NimBLECharacteristic   *ble_ch_charger     = nullptr;
static NimBLECharacteristic   *ble_ch_session_end = nullptr;
static bool                 ble_connected  = false;
static uint16_t             ble_pack_voltage_dv = 840;
static uint32_t             ble_last_notify_ms  = 0;

class CmdWriteCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *ch, NimBLEConnInfo &connInfo) override {
        // NimBLE getValue() returns NimBLEAttValue with .data(), .size(), .c_str()
        NimBLEAttValue val = ch->getValue();
        if (val.size() == 0) return;

        // Debug echo to serial so Putty shows BLE-received commands
        Serial.printf("[BLE] cmd(%u): ", (unsigned)val.size());
        for (unsigned i = 0; i < val.size(); i++) {
            char c = (char)val.data()[i];
            if (c >= 0x20 && c <= 0x7E) Serial.print(c);
            else Serial.printf("\\x%02X", (uint8_t)c);
        }
        Serial.println();

        // Multi-char commands (2+ chars): send directly to cmd_execute_multi
        if (val.size() >= 2 && val.data()[0] != '\r' && val.data()[0] != '\n') {
            char buf[CMD_BUF_SIZE];
            size_t blen = val.size();
            if (blen > CMD_BUF_SIZE - 1) blen = CMD_BUF_SIZE - 1;
            memcpy(buf, val.data(), blen);
            buf[blen] = '\0';
            // Strip trailing CR/LF
            while (blen > 0 && (buf[blen-1] == '\r' || buf[blen-1] == '\n')) {
                buf[--blen] = '\0';
            }
            if (blen >= 2) {
                cmd_execute_multi(buf, (uint8_t)blen);
                return;
            }
        }

        // Single-char commands: process each byte
        for (unsigned i = 0; i < val.size(); i++) {
            char c = (char)val.data()[i];
            if (c == '\r' || c == '\n' || c == '\0') continue;
            cmd_process_char(c);
        }
    }
};

class ServerCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *srv, NimBLEConnInfo &connInfo) override {
        ble_connected = true;
        Serial.println("[BLE] client connected");
    }
    void onDisconnect(NimBLEServer *srv, NimBLEConnInfo &connInfo, int reason) override {
        ble_connected = false;
        Serial.printf("[BLE] client disconnected (reason=%d), restarting advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

class VoltageCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *ch, NimBLEConnInfo &connInfo) override {
        NimBLEAttValue val = ch->getValue();
        if (val.size() >= 2) {
            const uint8_t *d = val.data();
            ble_pack_voltage_dv = (uint16_t)(d[0] | (d[1] << 8));
            Serial.printf("[BLE] pack voltage set -> %u dV (%.1f V)\n",
                          ble_pack_voltage_dv, ble_pack_voltage_dv * 0.1f);
        }
    }
};

void ble_server_init(void) {
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(256);  // Accept MTU negotiation up to 256 bytes (app requests this)
    ble_server_ptr = NimBLEDevice::createServer();
    ble_server_ptr->setCallbacks(new ServerCB());

    NimBLEService *svc = ble_server_ptr->createService(SVC_UUID);

    ble_ch_state = svc->createCharacteristic(CHAR_SAFETY_STATE_UUID,
                     NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    ble_ch_temp = svc->createCharacteristic(CHAR_MAX_TEMP_UUID,
                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    ble_ch_cur = svc->createCharacteristic(CHAR_CHARGE_CUR_UUID,
                   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    ble_ch_voltage = svc->createCharacteristic(CHAR_PACK_VOLTAGE_UUID,
                       NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    ble_ch_voltage->setCallbacks(new VoltageCB());
    uint8_t vbuf[2] = { (uint8_t)(ble_pack_voltage_dv & 0xFF), (uint8_t)(ble_pack_voltage_dv >> 8) };
    ble_ch_voltage->setValue(vbuf, 2);

    ble_ch_bms_pack = svc->createCharacteristic(CHAR_BMS_PACK_UUID,
                        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    ble_ch_bms_health = svc->createCharacteristic(CHAR_BMS_HEALTH_UUID,
                          NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    ble_ch_bms_cells = svc->createCharacteristic(CHAR_BMS_CELLS_UUID,
                         NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    ble_ch_gps = svc->createCharacteristic(CHAR_GPS_UUID,
                   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Command input -- phone/app writes single-char commands here
    ble_ch_cmd = svc->createCharacteristic(CHAR_CMD_UUID,
                     NIMBLE_PROPERTY::WRITE);
    ble_ch_cmd->setCallbacks(new CmdWriteCB());

    // Charger telemetry -- packed binary, notify at 1Hz
    // [0-1] smooth_voltage dV  [2-3] smooth_current dA  [4-7] session_wh (float)
    // [8-11] lifetime_wh (float)  [12] charging  [13] buddy_mode
    // [14-17] power_w (float)  [18-21] miles_added (float)
    ble_ch_charger = svc->createCharacteristic(CHAR_CHARGER_UUID,
                         NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Session end summary -- notified once when charge stops
    // App reads this to log session: start_v(2) end_v(2) wh(4) ah(4) duration_s(4) profile(1) outlet(1) cost(4) = 22 bytes
    ble_ch_session_end = svc->createCharacteristic(CHAR_SESSION_END_UUID,
                             NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // NimBLE 2.5.0: services start automatically when server starts (svc->start() deprecated)
    ble_server_ptr->start();
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SVC_UUID);
    adv->start();
    Serial.println("[BLE] NimBLE advertising as " BLE_DEVICE_NAME);
}

void ble_server_update(void) {
    if (!ble_ch_state || !ble_ch_temp || !ble_ch_cur) return;  // BLE init incomplete
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

    // Charger telemetry characteristic (22 bytes)
    {
        uint8_t b[22];
        uint16_t sv = (uint16_t)(can_smooth_voltage * 10.0f);
        uint16_t sa = (uint16_t)(can_smooth_current * 10.0f);
        b[0] = sv & 0xFF; b[1] = (sv >> 8) & 0xFF;
        b[2] = sa & 0xFF; b[3] = (sa >> 8) & 0xFF;
        float sess = cmd_buddy_mode ? nrg_buddy_wh : nrg_session_wh;
        memcpy(&b[4], &sess, 4);
        memcpy(&b[8], &nrg_lifetime_wh, 4);
        b[12] = cmd_charging ? 1 : 0;
        b[13] = cmd_buddy_mode ? 1 : 0;
        float pw = can_smooth_voltage * can_smooth_current;
        memcpy(&b[14], &pw, 4);
        float mi = sess / range_wh_per_mile;
        memcpy(&b[18], &mi, 4);
        ble_ch_charger->setValue(b, sizeof(b));
        if (ble_connected) ble_ch_charger->notify();
    }
}

bool     ble_server_is_connected(void)        { return ble_connected; }
uint16_t ble_server_get_pack_voltage_dv(void) { return ble_pack_voltage_dv; }

// Send session end summary via BLE + buzzer alert
static void notify_charge_complete(float start_v, float end_v, float wh, float ah,
                                    uint32_t duration_s, uint8_t profile, uint8_t outlet, float cost) {
    // BLE session end packet (22 bytes)
    if (ble_ch_session_end) {
        uint8_t b[22];
        uint16_t sv = (uint16_t)(start_v * 10.0f);
        uint16_t ev = (uint16_t)(end_v * 10.0f);
        b[0] = sv & 0xFF; b[1] = (sv >> 8) & 0xFF;
        b[2] = ev & 0xFF; b[3] = (ev >> 8) & 0xFF;
        memcpy(&b[4], &wh, 4);
        memcpy(&b[8], &ah, 4);
        memcpy(&b[12], &duration_s, 4);
        b[16] = profile;
        b[17] = outlet;
        memcpy(&b[18], &cost, 4);
        ble_ch_session_end->setValue(b, sizeof(b));
        if (ble_connected) ble_ch_session_end->notify();
    }

    // Flash RGB LED green 3 times to indicate completion
    for (int i = 0; i < 3; i++) {
        rgb_led_set_state(RGB_STATE_CHARGING);  // green
        FastLED.show();
        delay(300);
        rgb_led_set_state(RGB_STATE_IDLE);      // off
        FastLED.show();
        delay(300);
    }
}

// =============================================================================
//  TPMS BLE SCANNER (reads broadcast-only tire pressure sensors)
// =============================================================================
// Chinese BLE TPMS valve cap sensors broadcast advertisements continuously.
// No pairing or connection required -- we passively scan and decode.
// Two common protocols: "BR/SYTPMS" (service UUID 0x27A5, 7-byte mfr data)
// and "ZEEPIN/TP630" (18-byte mfr data). Discovery mode detects both.
// MAC addresses stored in NVS to filter only our sensors.

#define TPMS_SCAN_INTERVAL_MS   10000UL  // scan window: run scan every 10 seconds
#define TPMS_SCAN_DURATION_SEC  5        // each scan lasts 5 seconds
#define TPMS_DISCOVER_SEC       30       // discovery mode scan duration
#define TPMS_ALERT_REPEAT_MS    300000UL // repeat alerts every 5 minutes (not every scan)

// Adjustable alert thresholds (saved to NVS)
static float    tpms_alert_low_psi    = 30.0f;   // low pressure warning
static float    tpms_alert_crit_psi   = 20.0f;   // critical low -- immediate danger
static float    tpms_alert_high_psi   = 48.0f;   // over-pressure warning
static float    tpms_alert_high_temp_f = 158.0f;  // high temp warning (70C)
static float    tpms_alert_crit_temp_f = 185.0f;  // critical temp (85C)

// Alert rate limiting -- only repeat after TPMS_ALERT_REPEAT_MS
static uint32_t tpms_front_alert_ms = 0;
static uint32_t tpms_rear_alert_ms  = 0;

// Broadcast interval measurement
static uint32_t tpms_front_last_rx_ms = 0;  // timestamp of last front advertisement
static uint32_t tpms_rear_last_rx_ms  = 0;  // timestamp of last rear advertisement
static float    tpms_front_interval_s = 0;  // measured interval (seconds, EMA smoothed)
static float    tpms_rear_interval_s  = 0;
static bool     tpms_debug_interval   = false;  // print every rx with timing

// EMA smoothing for display (eliminates 0.1 PSI jitter at ADC boundary)
#define TPMS_EMA_ALPHA  0.3f
static float    tpms_front_psi_raw    = 0;  // raw from sensor (before smoothing)
static float    tpms_rear_psi_raw     = 0;
static float    tpms_front_temp_raw   = 0;
static float    tpms_rear_temp_raw    = 0;
static bool     tpms_front_primed     = false;  // first reading seeds EMA
static bool     tpms_rear_primed      = false;

// Slow leak detection -- alert if PSI drops > threshold over time
#define TPMS_LEAK_DROP_PSI    3.0f     // alert if pressure drops this much
#define TPMS_LEAK_TIME_MS     1800000UL  // over 30 minutes minimum
static float    tpms_front_baseline_psi = 0;  // baseline PSI for leak detection
static float    tpms_rear_baseline_psi  = 0;
static uint32_t tpms_front_baseline_ms  = 0;  // when baseline was set
static uint32_t tpms_rear_baseline_ms   = 0;

static bool     tpms_scan_active    = false;
static bool     tpms_discovering    = false;  // discovery mode: log all TPMS devices
static uint32_t tpms_last_scan_ms   = 0;
static char     tpms_front_mac[18]  = "";     // "AA:BB:CC:DD:EE:FF" NVS
static char     tpms_rear_mac[18]   = "";     // NVS
static float    tpms_front_battery  = 0.0f;   // volts or percent
static float    tpms_rear_battery   = 0.0f;

// Forward declaration
static void tpms_process_advertisement(const NimBLEAdvertisedDevice *dev);

class TPMSScanCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override {
        tpms_process_advertisement(dev);
    }
    void onScanEnd(const NimBLEScanResults &results, int reason) override {
        tpms_scan_active = false;
        if (tpms_discovering) {
            tpms_discovering = false;
            Serial.println("[TPMS] Discovery scan complete -- switching to normal mode");
        }
    }
};

static TPMSScanCB tpms_scan_cb;

static void tpms_start_scan(uint32_t duration_sec) {
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&tpms_scan_cb, true);  // true = want duplicates (repeated ads)
    scan->setActiveScan(false);    // passive scan -- don't send scan requests
    scan->setInterval(100);        // scan interval (ms)
    scan->setWindow(99);           // scan window (ms, nearly continuous)
    scan->setMaxResults(0);        // 0 = don't store results, just use callbacks
    // NimBLE 2.5.0: start() duration is in MILLISECONDS (passed to ble_gap_disc)
    bool ok = scan->start(duration_sec * 1000, false);
    tpms_scan_active = ok;
    if (!ok) {
        Serial.println("[TPMS] Scan failed to start");
    }
}

static void tpms_process_advertisement(const NimBLEAdvertisedDevice *dev) {
    // Check if this device has manufacturer data (AD type 0xFF)
    if (!dev->haveManufacturerData()) return;

    std::string mfr = dev->getManufacturerData();
    std::string addr = dev->getAddress().toString();
    int8_t rssi = dev->getRSSI();

    // Discovery mode: log everything that looks like TPMS
    if (tpms_discovering) {
        // Log any device with manufacturer data in TPMS-like size range
        if (mfr.size() >= 5 && mfr.size() <= 20) {
            Serial.printf("[TPMS-DISCOVER] MAC=%s  RSSI=%d  Name='%s'  MfrData(%u)=",
                          addr.c_str(), rssi,
                          dev->haveName() ? dev->getName().c_str() : "",
                          (unsigned)mfr.size());
            for (unsigned i = 0; i < mfr.size(); i++)
                Serial.printf("%02X ", (uint8_t)mfr[i]);
            Serial.println();

            // Also check for service UUID 0x27A5 (BR/SYTPMS type)
            if (dev->haveServiceUUID() && dev->isAdvertisingService(NimBLEUUID((uint16_t)0x27A5))) {
                Serial.printf("  -> BR/SYTPMS type detected (UUID 0x27A5)\n");
            }
        }
        return;
    }

    // Normal mode: only process our configured sensors
    if (!tpms_enabled) return;
    if (tpms_front_mac[0] == '\0' && tpms_rear_mac[0] == '\0') return;

    bool is_front = (tpms_front_mac[0] && addr == tpms_front_mac);
    bool is_rear  = (tpms_rear_mac[0]  && addr == tpms_rear_mac);
    if (!is_front && !is_rear) return;

    // Decode based on manufacturer data size
    float psi = 0.0f;
    float temp_f = 0.0f;
    float batt = 0.0f;

    if (mfr.size() >= 18 && (uint8_t)mfr[0] == 0x00 && (uint8_t)mfr[1] == 0x01) {
        // ZEEPIN/CozyLife format (confirmed working with dohome sensors):
        //   Bytes 0-1:   Company ID 0x0001
        //   Bytes 2-7:   Sensor MAC address
        //   Bytes 8-11:  Pressure in Pascals (uint32 LE)
        //   Bytes 12-15: Temperature x100 in Celsius (int32 LE)
        //   Byte 16:     Battery percentage
        //   Byte 17:     Alarm flag (0x00 = normal)
        uint32_t press_pa = (uint8_t)mfr[8] | ((uint8_t)mfr[9] << 8) |
                            ((uint8_t)mfr[10] << 16) | ((uint8_t)mfr[11] << 24);
        int32_t  temp_100 = (uint8_t)mfr[12] | ((uint8_t)mfr[13] << 8) |
                            ((uint8_t)mfr[14] << 16) | ((uint8_t)mfr[15] << 24);
        uint8_t  batt_pct = (uint8_t)mfr[16];

        psi = press_pa / 6894.76f;  // Pa to PSI
        temp_f = (temp_100 / 100.0f) * 9.0f / 5.0f + 32.0f;
        batt = batt_pct;
    } else if (mfr.size() >= 6 && mfr.size() <= 9) {
        // BR/SYTPMS format (fallback for other sensor brands):
        //   Byte 0: status flags
        //   Byte 1: battery voltage x10
        //   Byte 2: temperature in Celsius
        //   Bytes 3-4: pressure (encoding varies by clone)
        uint8_t bat_raw  = (uint8_t)mfr[1];
        uint8_t temp_c   = (uint8_t)mfr[2];
        uint16_t pres_raw = ((uint8_t)mfr[3] << 8) | (uint8_t)mfr[4];

        batt = bat_raw / 10.0f;
        temp_f = temp_c * 9.0f / 5.0f + 32.0f;

        if (pres_raw > 0 && pres_raw < 1000) {
            psi = pres_raw / 10.0f;
        } else if (pres_raw >= 1000) {
            psi = pres_raw * 0.145038f / 10.0f;
        }
    }

    if (psi < 0.1f) return;  // no valid pressure decoded

    // Measure broadcast interval
    uint32_t rx_now = millis();
    if (is_front) {
        if (tpms_front_last_rx_ms > 0) {
            float dt = (rx_now - tpms_front_last_rx_ms) / 1000.0f;
            if (dt > 0.5f && dt < 300.0f) {  // filter out duplicates and wraparound
                if (tpms_front_interval_s < 0.1f) tpms_front_interval_s = dt;
                else tpms_front_interval_s += 0.3f * (dt - tpms_front_interval_s);  // EMA
            }
            if (tpms_debug_interval)
                Serial.printf("[TPMS] FRONT rx  dt=%.1fs  avg=%.1fs  %.1fPSI %.0fF\n",
                              dt, tpms_front_interval_s, psi, temp_f);
        }
        tpms_front_last_rx_ms = rx_now;
    } else {
        if (tpms_rear_last_rx_ms > 0) {
            float dt = (rx_now - tpms_rear_last_rx_ms) / 1000.0f;
            if (dt > 0.5f && dt < 300.0f) {
                if (tpms_rear_interval_s < 0.1f) tpms_rear_interval_s = dt;
                else tpms_rear_interval_s += 0.3f * (dt - tpms_rear_interval_s);
            }
            if (tpms_debug_interval)
                Serial.printf("[TPMS] REAR  rx  dt=%.1fs  avg=%.1fs  %.1fPSI %.0fF\n",
                              dt, tpms_rear_interval_s, psi, temp_f);
        }
        tpms_rear_last_rx_ms = rx_now;
    }

    // Store data with EMA smoothing (eliminates 0.1 PSI jitter)
    if (is_front) {
        tpms_front_psi_raw  = psi;
        tpms_front_temp_raw = temp_f;
        if (!tpms_front_primed) {
            tpms_front_psi    = psi;
            tpms_front_temp_f = temp_f;
            tpms_front_baseline_psi = psi;
            tpms_front_baseline_ms  = rx_now;
            tpms_front_primed = true;
        } else {
            tpms_front_psi    += TPMS_EMA_ALPHA * (psi - tpms_front_psi);
            tpms_front_temp_f += TPMS_EMA_ALPHA * (temp_f - tpms_front_temp_f);
        }
        tpms_front_battery = batt;
        tpms_front_valid   = true;
        tpms_last_ms       = rx_now;

        // Slow leak detection
        if (tpms_front_baseline_ms > 0 &&
            (rx_now - tpms_front_baseline_ms) >= TPMS_LEAK_TIME_MS) {
            float drop = tpms_front_baseline_psi - tpms_front_psi;
            if (drop >= TPMS_LEAK_DROP_PSI) {
                Serial.printf("[TPMS] *** FRONT SLOW LEAK: dropped %.1f PSI in %lu min ***\n",
                              drop, (unsigned long)((rx_now - tpms_front_baseline_ms) / 60000));
            }
            tpms_front_baseline_psi = tpms_front_psi;  // reset baseline
            tpms_front_baseline_ms  = rx_now;
        }
    } else {
        tpms_rear_psi_raw  = psi;
        tpms_rear_temp_raw = temp_f;
        if (!tpms_rear_primed) {
            tpms_rear_psi    = psi;
            tpms_rear_temp_f = temp_f;
            tpms_rear_baseline_psi = psi;
            tpms_rear_baseline_ms  = rx_now;
            tpms_rear_primed = true;
        } else {
            tpms_rear_psi    += TPMS_EMA_ALPHA * (psi - tpms_rear_psi);
            tpms_rear_temp_f += TPMS_EMA_ALPHA * (temp_f - tpms_rear_temp_f);
        }
        tpms_rear_battery = batt;
        tpms_rear_valid   = true;
        tpms_last_ms      = rx_now;

        // Slow leak detection
        if (tpms_rear_baseline_ms > 0 &&
            (rx_now - tpms_rear_baseline_ms) >= TPMS_LEAK_TIME_MS) {
            float drop = tpms_rear_baseline_psi - tpms_rear_psi;
            if (drop >= TPMS_LEAK_DROP_PSI) {
                Serial.printf("[TPMS] *** REAR SLOW LEAK: dropped %.1f PSI in %lu min ***\n",
                              drop, (unsigned long)((rx_now - tpms_rear_baseline_ms) / 60000));
            }
            tpms_rear_baseline_psi = tpms_rear_psi;
            tpms_rear_baseline_ms  = rx_now;
        }
    }

    // Alerts (rate-limited: only repeat after TPMS_ALERT_REPEAT_MS)
    uint32_t *last_alert = is_front ? &tpms_front_alert_ms : &tpms_rear_alert_ms;
    uint32_t alert_now = millis();
    bool alert_due = (alert_now - *last_alert) >= TPMS_ALERT_REPEAT_MS || *last_alert == 0;

    if (alert_due) {
        const char *wheel = is_front ? "FRONT" : "REAR";
        bool alerted = false;

        if (psi < tpms_alert_crit_psi) {
            Serial.printf("[TPMS] *** %s CRITICAL LOW: %.1f PSI ***\n", wheel, psi);
            alerted = true;
        } else if (psi < tpms_alert_low_psi) {
            Serial.printf("[TPMS] %s low pressure: %.1f PSI\n", wheel, psi);
            alerted = true;
        }
        if (psi > tpms_alert_high_psi) {
            Serial.printf("[TPMS] %s high pressure: %.1f PSI\n", wheel, psi);
            alerted = true;
        }
        if (temp_f > tpms_alert_crit_temp_f) {
            Serial.printf("[TPMS] *** %s CRITICAL TEMP: %.0fF ***\n", wheel, temp_f);
            alerted = true;
        } else if (temp_f > tpms_alert_high_temp_f) {
            Serial.printf("[TPMS] %s high temp: %.0fF\n", wheel, temp_f);
            alerted = true;
        }

        if (alerted) *last_alert = alert_now;
    }
}

static void tpms_scan_update(void) {
    if (!tpms_enabled && !tpms_discovering) return;
    if (tpms_scan_active) return;  // scan in progress

    uint32_t now = millis();
    if ((now - tpms_last_scan_ms) >= TPMS_SCAN_INTERVAL_MS) {
        tpms_last_scan_ms = now;
        tpms_start_scan(tpms_discovering ? TPMS_DISCOVER_SEC : TPMS_SCAN_DURATION_SEC);
    }

    // Stale data check
    if (tpms_enabled && tpms_last_ms > 0 && (now - tpms_last_ms) > TPMS_TIMEOUT_MS) {
        if (tpms_front_valid || tpms_rear_valid) {
            tpms_front_valid = false;
            tpms_rear_valid  = false;
            Serial.println("[TPMS] Sensor data stale -- no signal for 60s");
        }
    }
}

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

static void draw_header(SafetyState_t state, const char *page_title) {
    const char *label = safety_state_name(state);

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
    uint32_t now = millis();

    // Detect charger presence from CAN RX
    ChargerStatus_t cs;
    cmd_charger_present = can_read_status(&cs);

    // Track peak power for input voltage detection
    if (cmd_charger_present && cmd_charging) {
        float pw = can_smooth_voltage * can_smooth_current;
        if (pw > input_peak_watts) input_peak_watts = pw;
        // Update average watts for schedule calculation
        if (schedule_avg_watts < 1.0f) schedule_avg_watts = pw;
        else schedule_avg_watts += 0.05f * (pw - schedule_avg_watts);
    }

    // Update time from GPS
    if (now - time_update_ms > 10000UL) {
        time_update_ms = now;
        GPSData_t gps;
        if (gps_driver_get(&gps) && gps.gps_valid) {
            // GPS time comes from TinyGPSPlus -- check if time is valid
            if (gps_parser.time.isValid()) {
                time_hour   = gps_parser.time.hour();
                time_minute = gps_parser.time.minute();
                time_valid  = true;
            }
        }
    }

    if (!cmd_charger_present) {
        safety_state = SAFETY_STATE_UNPLUGGED;

    } else if (cmd_charging) {
        // Check auto-stop: has pack reached profile target voltage?
        uint16_t stop_dv = profile_stop_voltage_dv();
        if (!test_dev_mode && stop_dv > 0 && can_smooth_voltage >= (stop_dv * 0.1f)) {
            cmd_charging = false;
            nrg_active = false;
            can_send_disable();
            safety_state = SAFETY_STATE_COMPLETE;
            float miles = (cmd_buddy_mode ? nrg_buddy_wh : nrg_session_wh) / range_wh_per_mile;
            Serial.printf("[AUTO] Charge complete at %.1fV (profile: %s, target: %.1fV)\n",
                          can_smooth_voltage, profile_name(cmd_profile), stop_dv * 0.1f);
            Serial.printf("[AUTO] Session: %.1fWh / %.2fAh (+%.1f mi)\n",
                          cmd_buddy_mode ? nrg_buddy_wh : nrg_session_wh,
                          cmd_buddy_mode ? nrg_buddy_ah : nrg_session_ah, miles);
            if (maintain_enabled) {
                maintain_waiting = true;
                maintain_check_ms = now;
                Serial.println("[MAINTAIN] Entering maintenance mode -- monitoring voltage");
            }

            // Charge completion notification
            if (!notify_sent) {
                notify_sent = true;
                float sess_wh = cmd_buddy_mode ? nrg_buddy_wh : nrg_session_wh;
                float sess_ah = cmd_buddy_mode ? nrg_buddy_ah : nrg_session_ah;
                uint32_t dur = nrg_session_start ? (now - nrg_session_start) / 1000 : 0;
                float sess_cost = cost_session;
                notify_charge_complete(0, can_smooth_voltage, sess_wh, sess_ah,
                                        dur, (uint8_t)cmd_profile, (uint8_t)outlet_preset, sess_cost);
                Serial.printf("[NOTIFY] Charge complete -- $%.2f this session\n", sess_cost);
            }
        } else {
            safety_state = SAFETY_STATE_CHARGING;
            notify_sent = false;  // reset for next completion

            // Thermal zone enforcement (skipped in TEST/DEV)
            float max_batt_f = get_max_battery_temp_f();
            if (!test_dev_mode && max_batt_f > -900.0f) {
                float mult = thermal_zone_multiplier(max_batt_f);
                if (mult <= 0.0f) {
                    // CRITICAL or FREEZE -- immediate shutdown
                    cmd_charging = false;
                    nrg_active = false;
                    can_send_disable();
                    safety_state = SAFETY_STATE_FAULT;
                    Serial.printf("[THERMAL] %s at %.0fF -- SHUTDOWN\n",
                                  thermal_zone_name(max_batt_f), max_batt_f);
                } else if (mult < 1.0f) {
                    // CAUTION or DANGER -- derate current
                    uint16_t derated_da = (uint16_t)(cmd_current_da * mult);
                    uint16_t clamped_da = outlet_clamp_current_da(derated_da);
                    ChargerCmd_t cmd;
                    cmd.voltage_dv = cmd_voltage_dv;
                    cmd.current_da = clamped_da;
                    cmd.control    = CAN_CTRL_CHARGE;
                    cmd.mode       = CAN_MODE_CHARGE;
                    can_send_command(&cmd);
                } else {
                    // NORMAL -- full rate, clamp to outlet
                    uint16_t clamped_da = outlet_clamp_current_da(cmd_current_da);
                    ChargerCmd_t cmd;
                    cmd.voltage_dv = cmd_voltage_dv;
                    cmd.current_da = clamped_da;
                    cmd.control    = CAN_CTRL_CHARGE;
                    cmd.mode       = CAN_MODE_CHARGE;
                    can_send_command(&cmd);
                }

                // Thermal prediction: track rate of temperature rise
                if ((now - thermal_predict_ms) >= THERMAL_PREDICT_INTERVAL_MS) {
                    thermal_predict_ms = now;
                    if (max_batt_f > -900.0f) {
                        if (thermal_last_temp_f > -900.0f) {
                            float delta_f = max_batt_f - thermal_last_temp_f;
                            if (delta_f > 0.0f) {
                                if (thermal_rise_rate_f_per_min < 0.01f)
                                    thermal_rise_rate_f_per_min = delta_f;
                                else
                                    thermal_rise_rate_f_per_min += 0.2f * (delta_f - thermal_rise_rate_f_per_min);
                            }
                        }
                        thermal_last_temp_f = max_batt_f;
                    }
                }
            }
        }

    } else if (maintain_enabled && maintain_waiting &&
               (safety_state == SAFETY_STATE_COMPLETE || safety_state == SAFETY_STATE_MAINTAINING)) {
        // Maintenance mode: check voltage periodically
        safety_state = SAFETY_STATE_MAINTAINING;
        if ((now - maintain_check_ms) >= MAINTAIN_CHECK_INTERVAL_MS) {
            maintain_check_ms = now;
            // Check if voltage has dropped enough to warrant top-up
            uint16_t stop_dv = profile_stop_voltage_dv();
            if (stop_dv > 0) {
                float target_v = stop_dv * 0.1f;
                float current_soc = voltage_to_soc_pct(can_smooth_voltage);
                float target_soc  = voltage_to_soc_pct(target_v);
                float drop = target_soc - current_soc;

                if (drop >= maintain_drop_pct) {
                    // SOC has dropped enough -- start top-up
                    Serial.printf("[MAINTAIN] SOC dropped %.1f%% (%.1fV -> need %.1fV) -- topping up\n",
                                  drop, can_smooth_voltage, target_v);
                    cmd_charging = true;
                    nrg_active = true;
                    nrg_last_ms = now;
                    cmd_send_now();
                    safety_state = SAFETY_STATE_CHARGING;
                    maintain_waiting = false;
                }
            }
        }

    } else if (schedule_enabled && !schedule_triggered && time_valid &&
               safety_state != SAFETY_STATE_CHARGING) {
        // Scheduled charge: calculate if it's time to start
        safety_state = SAFETY_STATE_SCHEDULED;
        uint16_t stop_dv = profile_stop_voltage_dv();
        if (stop_dv > 0 && schedule_avg_watts > 100.0f) {
            float target_v = stop_dv * 0.1f;
            float current_v = can_smooth_voltage;
            // Estimate Wh needed: rough approximation
            float wh_needed = (target_v - current_v) * cmd_current_da * 0.1f * 0.5f;
            if (wh_needed < 0) wh_needed = 0;
            float hours_needed = wh_needed / schedule_avg_watts;
            if (hours_needed < 0.1f) hours_needed = 0.1f;

            // Calculate start time
            int target_mins = schedule_target_hour * 60 + schedule_target_min;
            int start_mins = target_mins - (int)(hours_needed * 60.0f);
            if (start_mins < 0) start_mins += 1440;  // wrap past midnight

            int now_mins = time_hour * 60 + time_minute;

            // Check if it's time to start (within 2-minute window)
            int diff = now_mins - start_mins;
            if (diff < 0) diff += 1440;
            if (diff >= 0 && diff <= 2) {
                schedule_triggered = true;
                cmd_charging = true;
                nrg_active = true;
                nrg_last_ms = now;
                cmd_send_now();
                Serial.printf("[SCHEDULE] Starting charge now -- target %02d:%02d, est %.1f hrs needed\n",
                              schedule_target_hour, schedule_target_min, hours_needed);
                safety_state = SAFETY_STATE_CHARGING;
            }
        }

    } else if (safety_state == SAFETY_STATE_COMPLETE || safety_state == SAFETY_STATE_MAINTAINING) {
        // Stay in current state until user acts
    } else {
        safety_state = SAFETY_STATE_IDLE;
    }

    if (safety_state != prev) {
        if (prev == SAFETY_STATE_UNPLUGGED && safety_state != SAFETY_STATE_UNPLUGGED) {
            Serial.println("[CHARGER] Connected -- CAN communication established");
        } else if (safety_state == SAFETY_STATE_UNPLUGGED && prev != SAFETY_STATE_UNPLUGGED) {
            Serial.println("[CHARGER] Unplugged -- no CAN response");
        }
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
    Serial.println("  m  - Toggle MAINTAIN mode (battery tender)");
    Serial.println("  t  - Toggle SCHEDULE charge (be ready by time)");
    Serial.println("  b  - Enter BUDDY mode (charge another battery)");
    Serial.println("  n  - Exit buddy mode, restore home settings");
    Serial.println("  --- Profiles (auto-stop at target voltage) ---");
    Serial.println("  1  - MANUAL (no auto-stop)");
    Serial.printf( "  2  - STORAGE (%.1fV, %.3fV/cell)\n",
                   (float)(profile_cell_mv[PROFILE_STORAGE] * HOME_CELLS) / 1000.0f,
                   profile_cell_mv[PROFILE_STORAGE] / 1000.0f);
    Serial.printf( "  3  - 80%%  (%.1fV, %.3fV/cell)\n",
                   (float)(profile_cell_mv[PROFILE_80] * HOME_CELLS) / 1000.0f,
                   profile_cell_mv[PROFILE_80] / 1000.0f);
    Serial.printf( "  4  - 85%%  (%.1fV, %.3fV/cell)\n",
                   (float)(profile_cell_mv[PROFILE_85] * HOME_CELLS) / 1000.0f,
                   profile_cell_mv[PROFILE_85] / 1000.0f);
    Serial.printf( "  5  - 90%%  (%.1fV, %.3fV/cell)\n",
                   (float)(profile_cell_mv[PROFILE_90] * HOME_CELLS) / 1000.0f,
                   profile_cell_mv[PROFILE_90] / 1000.0f);
    Serial.printf( "  6  - 100%% (%.1fV, %.3fV/cell)\n",
                   (float)(profile_cell_mv[PROFILE_100] * HOME_CELLS) / 1000.0f,
                   profile_cell_mv[PROFILE_100] / 1000.0f);
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
    Serial.println("  --- Multi-char cmds (type then Enter) ---");
    Serial.println("  OA-OE - Outlet presets (110/15,20,30 220/15,30)");
    Serial.println("  OG24  - EV Station @ 24A (set your station amps)");
    Serial.println("  OU110,20 - User defined: voltage,amps");
    Serial.println("  OF    - TEST/DEV mode (all safety off)");
    Serial.println("  TZ1,140 - Set CAUTION temp threshold (F)");
    Serial.println("  TZ2,194 - Set DANGER temp threshold (F)");
    Serial.println("  TZ3,248 - Set CRITICAL temp threshold (F)");
    Serial.println("  TS1,0   - Disable NTC T1 (TS1-TS5)");
    Serial.println("  TSB1,0  - Disable BMS Temp1 (TSB1,TSB2)");
    Serial.println("  TC1,-2.0 - Calibrate NTC T1 offset (F)");
    Serial.println("  TCB1,1.5 - Calibrate BMS Temp1 offset (F)");
    Serial.println("  NLW,500  - Set lifetime Wh (after flash erase)");
    Serial.println("  NLA,6.5  - Set lifetime Ah");
    Serial.println("  NE,32.0  - Set efficiency (Wh/mile)");
    Serial.println("  NC,0.12  - Set electricity cost ($/kWh)");
    Serial.println("  NF,42.30 - Set lifetime cost ($)");
    Serial.println("  ND       - Dump all NVS values (backup)");
    Serial.println("  GA/GP/GB - GPS: onboard/phone/both");
    Serial.println("  TPMS1    - Enable TPMS BLE scanning");
    Serial.println("  TPMS0    - Disable TPMS");
    Serial.println("  TPMSD    - Discover sensors (scan 30s, log all MACs)");
    Serial.println("  TPMSF,MAC - Set front sensor MAC (e.g. TPMSF,aa:bb:cc:dd:ee:ff)");
    Serial.println("  TPMSR,MAC - Set rear sensor MAC");
    Serial.println("  TPMSI    - Show TPMS status and sensor info");
    Serial.println("  --- Direct set (for app control) ---");
    Serial.println("  SV,84.0  - Set exact voltage");
    Serial.println("  SA,12.0  - Set exact current (amps)");
    Serial.println("  SM,84.0,12.0 - Set voltage + current together");
    Serial.println("  SP,3     - Set profile (1=Manual..6=100%)");
    Serial.println("  SO,A     - Set outlet by letter (A-F,G,U)");
    Serial.println("  SR       - Request full status dump");
    Serial.println("  S1430    - Set clock to 14:30 (HHMM)");
    Serial.println("  --- WiFi / MQTT (Home Assistant) ---");
    Serial.println("  WS,SSID  - Set home WiFi SSID");
    Serial.println("  WP,PASS  - Set home WiFi password");
    Serial.println("  WH,IP    - Set MQTT broker address");
    Serial.println("  WM,1883  - Set MQTT port");
    Serial.println("  WI       - Show WiFi/MQTT status");
    Serial.println("  WC       - Force WiFi connect now");
    Serial.println("  WD       - Disconnect WiFi STA");
    Serial.println("========================================");
    Serial.println();
}

static void cmd_print_status(void) {
    Serial.println();
    Serial.println("-------- Status Report v" FW_VERSION " --------");

    // Mode
    Serial.printf("  State:     %s\n", safety_state_name(safety_state));
    Serial.printf("  Mode:      %s\n", cmd_buddy_mode ? "BUDDY (external battery)" : "NORMAL (home 20S pack)");

    // Profile
    uint16_t stop_dv = profile_stop_voltage_dv();
    if (stop_dv > 0) {
        Serial.printf("  Profile:   %s -- auto-stop at %.1fV\n",
                      profile_name(cmd_profile), stop_dv * 0.1f);
    } else {
        Serial.printf("  Profile:   %s -- no auto-stop\n", profile_name(cmd_profile));
    }

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

    // Energy & Range
    Serial.println("  ---- Energy ----");
    if (cmd_buddy_mode) {
        uint32_t el = nrg_buddy_start ? (millis() - nrg_buddy_start) / 1000 : 0;
        Serial.printf("  Buddy:     %.1fWh  %.2fAh  %lu:%02lu\n",
                      nrg_buddy_wh, nrg_buddy_ah,
                      (unsigned long)(el / 60), (unsigned long)(el % 60));
    } else {
        // Current session
        if (nrg_session_start) {
            uint32_t el = (millis() - nrg_session_start) / 1000;
            float miles = nrg_session_wh / range_wh_per_mile;
            Serial.printf("  Session:   %.1fWh  %.2fAh  %lu:%02lu  (+%.1f mi)\n",
                          nrg_session_wh, nrg_session_ah,
                          (unsigned long)(el / 60), (unsigned long)(el % 60), miles);
            if (el > 0) {
                float avg_w = nrg_session_wh / ((float)el / 3600.0f);
                Serial.printf("  Avg power: %.0fW\n", avg_w);
            }
        } else {
            Serial.println("  Session:   No active session");
        }

        // Previous session
        if (nrg_prev_wh > 0.1f) {
            float prev_mi = nrg_prev_wh / range_wh_per_mile;
            Serial.printf("  Previous:  %.1fWh  %.2fAh  %lu:%02lu  (+%.1f mi)\n",
                          nrg_prev_wh, nrg_prev_ah,
                          (unsigned long)(nrg_prev_duration_s / 60),
                          (unsigned long)(nrg_prev_duration_s % 60), prev_mi);
        }

        // Lifetime
        Serial.printf("  Lifetime:  %.1fWh  %.1fAh\n", nrg_lifetime_wh, nrg_lifetime_ah);
    }

    // Range calculator
    Serial.println("  ---- Range ----");
    Serial.printf("  Efficiency: %.1f Wh/mi\n", range_wh_per_mile);
    if (nrg_session_start && can_smooth_current > 0.5f) {
        float avg_power_w = can_smooth_voltage * can_smooth_current;
        // How many miles per hour of charging at current rate
        float mi_per_hour = avg_power_w / range_wh_per_mile;
        Serial.printf("  Charge rate: +%.1f mi/hr at current power\n", mi_per_hour);
        // Time to add 5, 10, 20 miles
        Serial.printf("  +5 mi:  %d min | +10 mi: %d min | +20 mi: %d min\n",
                      (int)(5.0f / mi_per_hour * 60.0f),
                      (int)(10.0f / mi_per_hour * 60.0f),
                      (int)(20.0f / mi_per_hour * 60.0f));
    } else {
        Serial.println("  Start charging to see time-to-range estimates");
    }

    // Time to target profiles
    if (can_smooth_current > 0.5f) {
        float avg_power_w = can_smooth_voltage * can_smooth_current;
        Serial.println("  ---- Time to Target ----");
        for (int pr = PROFILE_STORAGE; pr <= PROFILE_100; pr++) {
            float target_v = (float)((uint32_t)profile_cell_mv[pr] * HOME_CELLS) / 1000.0f;
            if (can_smooth_voltage < target_v) {
                float wh_est = (target_v - can_smooth_voltage) * can_smooth_current * 0.5f;
                int mins = (int)(wh_est / avg_power_w * 60.0f);
                Serial.printf("  %s (%.1fV): ~%d min\n", profile_name((ChargeProfile_t)pr), target_v, mins);
            } else {
                Serial.printf("  %s (%.1fV): already reached\n", profile_name((ChargeProfile_t)pr), target_v);
            }
        }
    }

    // Cost
    Serial.println("  ---- Cost ----");
    Serial.printf("  Rate:     $%.3f/kWh (%s)\n", outlet_cost_rate(), outlet_name());
    if (cost_session > 0.001f)
        Serial.printf("  Session:  $%.2f\n", cost_session);
    Serial.printf("  Lifetime: $%.2f (paid) + %.1f kWh free\n", cost_lifetime, cost_free_lifetime);

    // Thermal prediction
    float ttc = thermal_time_to_caution_min();
    if (ttc > 0.0f && cmd_charging) {
        Serial.printf("  Thermal:  ~%.0f min to CAUTION at current rate (%.1fF/min rise)\n",
                      ttc, thermal_rise_rate_f_per_min);
    }

    // TPMS
    if (tpms_enabled) {
        Serial.println("  ---- TPMS ----");
        bool stale = (millis() - tpms_last_ms) > TPMS_TIMEOUT_MS && tpms_last_ms > 0;
        if (tpms_front_valid)
            Serial.printf("  Front: %.1f PSI  %.0fF%s\n", tpms_front_psi, tpms_front_temp_f,
                          stale ? " STALE" : "");
        else
            Serial.println("  Front: no data");
        if (tpms_rear_valid)
            Serial.printf("  Rear:  %.1f PSI  %.0fF%s\n", tpms_rear_psi, tpms_rear_temp_f,
                          stale ? " STALE" : "");
        else
            Serial.println("  Rear:  no data");
    }

    // Maintain & Schedule
    Serial.println("  ---- Modes ----");
    Serial.printf("  Maintain:  %s", maintain_enabled ? "ON" : "OFF");
    if (maintain_enabled)
        Serial.printf(" (top up after %.0f%% drop, %s)\n",
                      maintain_drop_pct,
                      maintain_waiting ? "WATCHING" : "inactive");
    else Serial.println();

    Serial.printf("  Schedule:  %s", schedule_enabled ? "ON" : "OFF");
    if (schedule_enabled)
        Serial.printf(" (%s by %02d:%02d, %s)\n",
                      profile_name(cmd_profile),
                      schedule_target_hour, schedule_target_min,
                      schedule_triggered ? "TRIGGERED" : "waiting");
    else Serial.println();

    Serial.printf("  Outlet:    %s (max %uW)\n", outlet_name(), outlet_max_watts());
    Serial.printf("  Peak seen: %.0fW\n", input_peak_watts);

    // Battery temperature
    float max_batt_f = get_max_battery_temp_f();
    if (max_batt_f > -900.0f) {
        Serial.printf("  Batt temp: %.0fF [%s] derate:%.0f%%\n",
                      max_batt_f, thermal_zone_name(max_batt_f),
                      thermal_zone_multiplier(max_batt_f) * 100.0f);
    } else {
        Serial.println("  Batt temp: No sensors connected");
    }
    Serial.printf("  Zones:     NORMAL<%.0fF CAUTION<%.0fF DANGER<%.0fF\n",
                  tz_normal_max_f, tz_caution_max_f, tz_danger_max_f);

    if (time_valid)
        Serial.printf("  Time:      %02d:%02d (GPS)\n", time_hour, time_minute);
    else
        Serial.println("  Time:      No fix (waiting for GPS or app sync)");

    Serial.println("---------------------------------------");
    Serial.println();
}

static void cmd_send_now(void) {
    if (cmd_charging) {
        uint16_t clamped_da = outlet_clamp_current_da(cmd_current_da);
        ChargerCmd_t cmd;
        cmd.voltage_dv = cmd_voltage_dv;
        cmd.current_da = clamped_da;
        cmd.control    = CAN_CTRL_CHARGE;
        cmd.mode       = CAN_MODE_CHARGE;
        can_send_command(&cmd);
        if (clamped_da < cmd_current_da) {
            Serial.printf("  ** Clamped to %.1fA by %s outlet (%uW max)\n",
                          clamped_da * 0.1f, outlet_name(), outlet_max_watts());
        }
    } else {
        can_send_disable();
    }
}

static void serial_command_update(void) {
    uint32_t now = millis();

    // ---- Periodic NVS save (every 60s while charging) ----
    if (nrg_active && (now - nrg_save_ms) >= NRG_SAVE_INTERVAL_MS) {
        nrg_save_ms = now;
        nrg_prefs.putFloat("lifetime_wh", nrg_lifetime_wh);
        nrg_prefs.putFloat("lifetime_ah", nrg_lifetime_ah);
        nrg_prefs.putFloat("cost_life", cost_lifetime);
        nrg_prefs.putFloat("cost_free", cost_free_lifetime);
    }

    // ---- Periodic status line (every 5s) ----
    if ((now - cmd_status_ms) >= CMD_STATUS_INTERVAL_MS) {
        cmd_status_ms = now;

        ChargerStatus_t cs;
        bool has_rx = can_read_status(&cs);

        const char *st_name = safety_state_name(safety_state);
        // Timestamp: HH:MM if clock valid, otherwise uptime MM:SS
        if (time_valid)
            Serial.printf("[%02d:%02d] ", time_hour, time_minute);
        else {
            uint32_t up = now / 1000;
            Serial.printf("[%02lu:%02lu] ", (unsigned long)(up / 60) % 100, (unsigned long)(up % 60));
        }
        Serial.printf("%s%s %s | Set:%.1fV/%.1fA %s | ",
                      test_dev_mode ? "[DEV] " : "",
                      cmd_buddy_mode ? "BUDDY" : "HOME",
                      st_name,
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

        // Energy + range + cost
        if (cmd_buddy_mode && nrg_buddy_wh > 0.1f) {
            Serial.printf(" | Buddy:%.1fWh", nrg_buddy_wh);
        } else if (nrg_session_wh > 0.1f) {
            float miles = nrg_session_wh / range_wh_per_mile;
            Serial.printf(" | %.1fWh +%.1fmi", nrg_session_wh, miles);
            if (cost_session > 0.001f) Serial.printf(" $%.2f", cost_session);
        }
        Serial.println();
    }

    // ---- Heartbeat: re-send last command every CAN interval ----
    // (can_driver_update handles this via s_last_cmd, so we just
    //  ensure cmd_send_now() was called after any setpoint change)

    // ---- Process serial input ----
    while (Serial.available()) {
        cmd_process_char((char)Serial.read());
    }
}

// ---- Multi-char command executor ----
static void cmd_execute_multi(const char *buf, uint8_t len) {
    // Outlet presets: OA, OB, OC, OD, OE, OF, OG24, OU110,20
    if (buf[0] == 'O' && len >= 2) {
        switch (buf[1]) {
            case 'A': outlet_preset = OUTLET_110_15A; test_dev_mode = false; break;
            case 'B': outlet_preset = OUTLET_110_20A; test_dev_mode = false; break;
            case 'C': outlet_preset = OUTLET_110_30A; test_dev_mode = false; break;
            case 'D': outlet_preset = OUTLET_220_15A; test_dev_mode = false; break;
            case 'E': outlet_preset = OUTLET_220_30A; test_dev_mode = false; break;
            case 'F':
                outlet_preset = OUTLET_TEST_DEV;
                test_dev_mode = true;
                Serial.println("[DEV] TEST/DEV MODE -- ALL SAFETY DISABLED");
                break;
            case 'G':
                outlet_preset = OUTLET_EV_STATION;
                test_dev_mode = false;
                if (len > 2) outlet_amps_ac = (uint16_t)atoi(&buf[2]);
                Serial.printf("[OUTLET] EV Station @ %uA (max %uW)\n",
                              outlet_amps_ac, outlet_max_watts());
                return;
            case 'U':
                outlet_preset = OUTLET_USER_DEFINED;
                test_dev_mode = false;
                // Parse: OU110,20 or OU220,25
                if (len > 2) {
                    const char *comma = strchr(&buf[2], ',');
                    if (comma) {
                        outlet_voltage_ac = (uint16_t)atoi(&buf[2]);
                        outlet_amps_ac = (uint16_t)atoi(comma + 1);
                    }
                }
                Serial.printf("[OUTLET] User: %uV / %uA (max %uW)\n",
                              outlet_voltage_ac, outlet_amps_ac, outlet_max_watts());
                return;
            default:
                Serial.printf("[CMD] Unknown outlet: O%c\n", buf[1]);
                return;
        }
        Serial.printf("[OUTLET] %s (max %uW)\n", outlet_name(), outlet_max_watts());
        nrg_prefs.putUChar("outlet", (uint8_t)outlet_preset);
        nrg_prefs.putUShort("outlet_v", outlet_voltage_ac);
        nrg_prefs.putUShort("outlet_a", outlet_amps_ac);
        return;
    }

    // Thermal zone thresholds: TZ1,140  TZ2,194  TZ3,248
    if (buf[0] == 'T' && buf[1] == 'Z' && len >= 4) {
        const char *comma = strchr(&buf[2], ',');
        if (comma) {
            int zone = buf[2] - '0';
            float val = atof(comma + 1);
            switch (zone) {
                case 1: tz_normal_max_f = val;
                    Serial.printf("[THERMAL] CAUTION starts at %.0fF\n", val); break;
                case 2: tz_caution_max_f = val;
                    Serial.printf("[THERMAL] DANGER starts at %.0fF\n", val); break;
                case 3: tz_danger_max_f = val;
                    Serial.printf("[THERMAL] CRITICAL starts at %.0fF\n", val); break;
                default: Serial.println("[CMD] Zone must be 1, 2, or 3"); return;
            }
            nrg_prefs.putFloat("tz1", tz_normal_max_f);
            nrg_prefs.putFloat("tz2", tz_caution_max_f);
            nrg_prefs.putFloat("tz3", tz_danger_max_f);
        }
        return;
    }

    // Sensor enable/disable: TS1,0  TS3,1  TSB1,0  TSB2,1
    if (buf[0] == 'T' && buf[1] == 'S' && len >= 4) {
        if (buf[2] == 'B' && len >= 5) {
            // BMS temp: TSB1,0 or TSB2,1
            const char *comma = strchr(&buf[3], ',');
            if (comma) {
                int idx = buf[3] - '0';
                bool en = (atoi(comma + 1) != 0);
                if (idx == 1) { sensor_bms1_enabled = en;
                    Serial.printf("[SENSOR] BMS Temp1 %s\n", en ? "ENABLED" : "DISABLED"); }
                else if (idx == 2) { sensor_bms2_enabled = en;
                    Serial.printf("[SENSOR] BMS Temp2 %s\n", en ? "ENABLED" : "DISABLED"); }
            }
        } else {
            // NTC: TS1,0 or TS5,1
            const char *comma = strchr(&buf[2], ',');
            if (comma) {
                int idx = buf[2] - '0';
                bool en = (atoi(comma + 1) != 0);
                if (idx >= 1 && idx <= 5) {
                    sensor_ntc_enabled[idx] = en;
                    Serial.printf("[SENSOR] NTC T%d %s\n", idx, en ? "ENABLED" : "DISABLED");
                }
            }
        }
        return;
    }

    // Calibration offset: TC1,-2.0  TC3,1.5  TCB1,-3.0  TCB2,0
    if (buf[0] == 'T' && buf[1] == 'C' && len >= 4) {
        if (buf[2] == 'B' && len >= 5) {
            const char *comma = strchr(&buf[3], ',');
            if (comma) {
                int idx = buf[3] - '0';
                float offset = atof(comma + 1);
                if (idx == 1) { sensor_bms1_offset_f = offset;
                    Serial.printf("[CAL] BMS Temp1 offset: %+.1fF\n", offset); }
                else if (idx == 2) { sensor_bms2_offset_f = offset;
                    Serial.printf("[CAL] BMS Temp2 offset: %+.1fF\n", offset); }
                nrg_prefs.putFloat("calb1", sensor_bms1_offset_f);
                nrg_prefs.putFloat("calb2", sensor_bms2_offset_f);
            }
        } else {
            const char *comma = strchr(&buf[2], ',');
            if (comma) {
                int idx = buf[2] - '0';
                float offset = atof(comma + 1);
                if (idx >= 1 && idx <= 5) {
                    sensor_ntc_offset_f[idx] = offset;
                    Serial.printf("[CAL] NTC T%d offset: %+.1fF\n", idx, offset);
                    char key[8];
                    snprintf(key, sizeof(key), "cal%d", idx);
                    nrg_prefs.putFloat(key, offset);
                }
            }
        }
        return;
    }

    // TPMS commands: TPMS1/0, TPMSD (discover), TPMSF,MAC (front), TPMSR,MAC (rear), TPMSI (info)
    if (len >= 4 && buf[0] == 'T' && buf[1] == 'P' && buf[2] == 'M' && buf[3] == 'S') {
        if (len >= 5 && buf[4] == 'D') {
            // TPMSD -- start discovery scan (30 seconds, logs all TPMS devices)
            tpms_discovering = true;
            tpms_last_scan_ms = 0;  // trigger immediate scan
            Serial.println("[TPMS] Discovery mode ON -- scanning for 30 seconds...");
            Serial.println("       All TPMS sensors in range will be logged with MAC addresses.");
            Serial.println("       Then use TPMSF,MAC and TPMSR,MAC to assign front/rear.");
            return;
        }
        if (len >= 6 && buf[4] == 'F' && buf[5] == ',') {
            // TPMSF,AA:BB:CC:DD:EE:FF -- set front sensor MAC
            strncpy(tpms_front_mac, &buf[6], sizeof(tpms_front_mac) - 1);
            tpms_front_mac[sizeof(tpms_front_mac) - 1] = '\0';
            nrg_prefs.putString("tpms_f_mac", tpms_front_mac);
            Serial.printf("[TPMS] Front sensor MAC: %s\n", tpms_front_mac);
            return;
        }
        if (len >= 6 && buf[4] == 'R' && buf[5] == ',') {
            // TPMSR,AA:BB:CC:DD:EE:FF -- set rear sensor MAC
            strncpy(tpms_rear_mac, &buf[6], sizeof(tpms_rear_mac) - 1);
            tpms_rear_mac[sizeof(tpms_rear_mac) - 1] = '\0';
            nrg_prefs.putString("tpms_r_mac", tpms_rear_mac);
            Serial.printf("[TPMS] Rear sensor MAC: %s\n", tpms_rear_mac);
            return;
        }
        if (len >= 5 && buf[4] == 'I') {
            // TPMSI -- show TPMS info
            Serial.println();
            Serial.println("-------- TPMS Info --------");
            Serial.printf("  Enabled:    %s\n", tpms_enabled ? "YES" : "NO");
            Serial.printf("  Front MAC:  %s\n", tpms_front_mac[0] ? tpms_front_mac : "(not set)");
            if (tpms_front_valid)
                Serial.printf("  Front:      %.1f PSI  %.0fF  batt=%.1f\n",
                              tpms_front_psi, tpms_front_temp_f, tpms_front_battery);
            Serial.printf("  Rear MAC:   %s\n", tpms_rear_mac[0] ? tpms_rear_mac : "(not set)");
            if (tpms_rear_valid)
                Serial.printf("  Rear:       %.1f PSI  %.0fF  batt=%.1f\n",
                              tpms_rear_psi, tpms_rear_temp_f, tpms_rear_battery);
            Serial.printf("  Alerts:     LOW<%.0f  CRIT<%.0f  HIGH>%.0f PSI\n",
                          tpms_alert_low_psi, tpms_alert_crit_psi, tpms_alert_high_psi);
            Serial.printf("  Temp:       HIGH>%.0fF  CRIT>%.0fF\n",
                          tpms_alert_high_temp_f, tpms_alert_crit_temp_f);
            Serial.printf("  Repeat:     every %lu min\n",
                          (unsigned long)(TPMS_ALERT_REPEAT_MS / 60000));
            if (tpms_front_interval_s > 0.1f)
                Serial.printf("  Front rate: every %.1fs\n", tpms_front_interval_s);
            if (tpms_rear_interval_s > 0.1f)
                Serial.printf("  Rear rate:  every %.1fs\n", tpms_rear_interval_s);
            Serial.printf("  Debug:      %s\n", tpms_debug_interval ? "ON" : "OFF");
            Serial.printf("  Discover:   %s\n", tpms_discovering ? "ACTIVE" : "OFF");
            Serial.println("---------------------------");
            Serial.println();
            return;
        }
        if (len >= 5 && buf[4] == 'X') {
            // TPMSX -- toggle broadcast interval debug (prints every rx with timing)
            tpms_debug_interval = !tpms_debug_interval;
            tpms_front_interval_s = 0;
            tpms_rear_interval_s  = 0;
            tpms_front_last_rx_ms = 0;
            tpms_rear_last_rx_ms  = 0;
            Serial.printf("[TPMS] Broadcast debug %s -- monitoring rx intervals\n",
                          tpms_debug_interval ? "ON" : "OFF");
            return;
        }
        if (len >= 6 && buf[4] == 'A' && buf[5] == ',') {
            // TPMSA,low,crit,high -- set pressure alert thresholds (PSI)
            // Example: TPMSA,30,20,48
            float vals[3] = {0};
            int vi = 0;
            const char *p = &buf[6];
            vals[0] = atof(p);
            const char *c1 = strchr(p, ',');
            if (c1) { vals[1] = atof(c1 + 1); vi = 1;
                const char *c2 = strchr(c1 + 1, ',');
                if (c2) { vals[2] = atof(c2 + 1); vi = 2; }
            }
            if (vi >= 2) {
                tpms_alert_low_psi  = vals[0];
                tpms_alert_crit_psi = vals[1];
                tpms_alert_high_psi = vals[2];
                nrg_prefs.putFloat("tpms_low", tpms_alert_low_psi);
                nrg_prefs.putFloat("tpms_crit", tpms_alert_crit_psi);
                nrg_prefs.putFloat("tpms_high", tpms_alert_high_psi);
                Serial.printf("[TPMS] Alerts: LOW<%.0f  CRIT<%.0f  HIGH>%.0f PSI\n",
                              tpms_alert_low_psi, tpms_alert_crit_psi, tpms_alert_high_psi);
            } else {
                Serial.println("[TPMS] Usage: TPMSA,low,crit,high (e.g. TPMSA,30,20,48)");
            }
            return;
        }
        if (len >= 6 && buf[4] == 'T' && buf[5] == ',') {
            // TPMST,high_f,crit_f -- set temperature alert thresholds (F)
            // Example: TPMST,158,185
            float high_f = atof(&buf[6]);
            const char *c1 = strchr(&buf[6], ',');
            if (c1) {
                float crit_f = atof(c1 + 1);
                tpms_alert_high_temp_f = high_f;
                tpms_alert_crit_temp_f = crit_f;
                nrg_prefs.putFloat("tpms_ht", tpms_alert_high_temp_f);
                nrg_prefs.putFloat("tpms_ct", tpms_alert_crit_temp_f);
                Serial.printf("[TPMS] Temp alerts: HIGH>%.0fF  CRIT>%.0fF\n",
                              tpms_alert_high_temp_f, tpms_alert_crit_temp_f);
            } else {
                Serial.println("[TPMS] Usage: TPMST,high_f,crit_f (e.g. TPMST,158,185)");
            }
            return;
        }
        // TPMS1 / TPMS0 -- enable/disable
        if (len >= 5) {
            tpms_enabled = (buf[4] == '1');
        } else {
            tpms_enabled = !tpms_enabled;
        }
        nrg_prefs.putBool("tpms_on", tpms_enabled);
        Serial.printf("[TPMS] %s\n", tpms_enabled ? "ENABLED" : "DISABLED");
        if (tpms_enabled) {
            Serial.println("       Scanning BLE for TPMS sensors");
            if (tpms_front_mac[0] == '\0' && tpms_rear_mac[0] == '\0')
                Serial.println("       WARNING: No sensor MACs configured. Run TPMSD to discover.");
        }
        return;
    }

    // Schedule target time: T0800 (set schedule complete-by time to 08:00)
    if (buf[0] == 'T' && len == 5 && buf[1] >= '0' && buf[1] <= '2') {
        schedule_target_hour = (buf[1] - '0') * 10 + (buf[2] - '0');
        schedule_target_min  = (buf[3] - '0') * 10 + (buf[4] - '0');
        if (schedule_target_hour > 23) schedule_target_hour = 23;
        if (schedule_target_min > 59)  schedule_target_min = 59;
        Serial.printf("[SCHEDULE] Target time set to %02d:%02d\n",
                      schedule_target_hour, schedule_target_min);
        return;
    }

    // GPS source: GA (onboard), GP (phone), GB (both)
    if (buf[0] == 'G' && len >= 2) {
        switch (buf[1]) {
            case 'A': gps_source = GPS_SOURCE_ONBOARD;
                Serial.println("[GPS] Source: ONBOARD only"); break;
            case 'P': gps_source = GPS_SOURCE_PHONE;
                Serial.println("[GPS] Source: PHONE only"); break;
            case 'B': gps_source = GPS_SOURCE_BOTH;
                Serial.println("[GPS] Source: BOTH (phone primary, onboard fallback)"); break;
            default: Serial.println("[CMD] GPS: GA=onboard, GP=phone, GB=both"); break;
        }
        return;
    }

    // Phone GPS data: PG,lat,lon,speed_mph (from app)
    if (buf[0] == 'P' && buf[1] == 'G' && len >= 5) {
        // Parse: PG,26.12345,-80.54321,42.5
        const char *p = &buf[3];
        const char *c1 = strchr(p, ',');
        if (c1) {
            phone_gps_lat = atof(p);
            const char *c2 = strchr(c1 + 1, ',');
            if (c2) {
                phone_gps_lon = atof(c1 + 1);
                phone_gps_speed = atof(c2 + 1);
            } else {
                phone_gps_lon = atof(c1 + 1);
            }
            phone_gps_valid = true;
            phone_gps_ms = millis();
        }
        return;
    }

    // NVS commands: NLW,1234.5 (lifetime Wh), NLA,56.7 (lifetime Ah),
    //   NE,32.0 (efficiency Wh/mi), NC,0.12 (cost/kWh), ND (dump all)
    if (buf[0] == 'N' && len >= 2) {
        if (buf[1] == 'D') {
            // Dump all NVS values
            Serial.println();
            Serial.println("========= NVS Stored Values =========");
            Serial.printf("  Lifetime Wh:  %.1f  (restore: NLW,%.1f)\n", nrg_lifetime_wh, nrg_lifetime_wh);
            Serial.printf("  Lifetime Ah:  %.1f  (restore: NLA,%.1f)\n", nrg_lifetime_ah, nrg_lifetime_ah);
            Serial.printf("  Wh/mile:      %.1f  (restore: NE,%.1f)\n", range_wh_per_mile, range_wh_per_mile);
            Serial.printf("  Outlet:       %s   (restore: O%c)\n", outlet_name(),
                          outlet_preset <= OUTLET_220_30A ? ('A' + outlet_preset) : 'A');
            if (outlet_preset == OUTLET_USER_DEFINED)
                Serial.printf("  User outlet:  %uV/%uA  (restore: OU%u,%u)\n",
                              outlet_voltage_ac, outlet_amps_ac, outlet_voltage_ac, outlet_amps_ac);
            if (outlet_preset == OUTLET_EV_STATION)
                Serial.printf("  EV Station:   %uA  (restore: OG%u)\n", outlet_amps_ac, outlet_amps_ac);
            Serial.printf("  Thermal Z1:   %.0fF  (restore: TZ1,%.0f)\n", tz_normal_max_f, tz_normal_max_f);
            Serial.printf("  Thermal Z2:   %.0fF  (restore: TZ2,%.0f)\n", tz_caution_max_f, tz_caution_max_f);
            Serial.printf("  Thermal Z3:   %.0fF  (restore: TZ3,%.0f)\n", tz_danger_max_f, tz_danger_max_f);
            for (int i = 1; i <= 5; i++) {
                if (sensor_ntc_offset_f[i] != 0.0f)
                    Serial.printf("  NTC T%d cal:   %+.1fF  (restore: TC%d,%.1f)\n",
                                  i, sensor_ntc_offset_f[i], i, sensor_ntc_offset_f[i]);
            }
            if (sensor_bms1_offset_f != 0.0f)
                Serial.printf("  BMS T1 cal:   %+.1fF  (restore: TCB1,%.1f)\n",
                              sensor_bms1_offset_f, sensor_bms1_offset_f);
            if (sensor_bms2_offset_f != 0.0f)
                Serial.printf("  BMS T2 cal:   %+.1fF  (restore: TCB2,%.1f)\n",
                              sensor_bms2_offset_f, sensor_bms2_offset_f);
            Serial.printf("  Cost/kWh:     $%.3f  (restore: NC,%.3f)\n", cost_per_kwh, cost_per_kwh);
            Serial.printf("  Cost lifetime: $%.2f  (restore: NF,%.2f)\n", cost_lifetime, cost_lifetime);
            Serial.printf("  Free energy:  %.1f kWh\n", cost_free_lifetime);
            Serial.printf("  GPS source:   %s  (restore: G%c)\n",
                          gps_source == GPS_SOURCE_ONBOARD ? "ONBOARD" :
                          gps_source == GPS_SOURCE_PHONE ? "PHONE" : "BOTH",
                          gps_source == GPS_SOURCE_ONBOARD ? 'A' :
                          gps_source == GPS_SOURCE_PHONE ? 'P' : 'B');
            Serial.printf("  WiFi SSID:    %s  (restore: WS,%s)\n",
                          wifi_sta_ssid[0] ? wifi_sta_ssid : "(none)",
                          wifi_sta_ssid[0] ? wifi_sta_ssid : "YourSSID");
            Serial.printf("  WiFi pass:    %s\n", wifi_sta_pass[0] ? "****" : "(none)");
            Serial.printf("  MQTT host:    %s  (restore: WH,%s)\n",
                          mqtt_host[0] ? mqtt_host : "(none)",
                          mqtt_host[0] ? mqtt_host : "192.168.1.x");
            Serial.printf("  MQTT port:    %u  (restore: WM,%u)\n", mqtt_port, mqtt_port);
            Serial.println("=====================================");
            Serial.println();
            return;
        }

        const char *comma = strchr(&buf[1], ',');
        if (comma) {
            float val = atof(comma + 1);

            if (buf[1] == 'L' && buf[2] == 'W') {
                nrg_lifetime_wh = val;
                nrg_prefs.putFloat("lifetime_wh", nrg_lifetime_wh);
                Serial.printf("[NVS] Lifetime Wh set to %.1f\n", nrg_lifetime_wh);
                return;
            }
            if (buf[1] == 'L' && buf[2] == 'A') {
                nrg_lifetime_ah = val;
                nrg_prefs.putFloat("lifetime_ah", nrg_lifetime_ah);
                Serial.printf("[NVS] Lifetime Ah set to %.1f\n", nrg_lifetime_ah);
                return;
            }
            if (buf[1] == 'E') {
                range_wh_per_mile = val;
                nrg_prefs.putFloat("wh_per_mile", range_wh_per_mile);
                Serial.printf("[NVS] Efficiency set to %.1f Wh/mile\n", range_wh_per_mile);
                return;
            }
            if (buf[1] == 'C') {
                cost_per_kwh = val;
                nrg_prefs.putFloat("cost_kwh", cost_per_kwh);
                Serial.printf("[NVS] Cost set to $%.3f/kWh\n", cost_per_kwh);
                return;
            }
            if (buf[1] == 'F') {
                cost_lifetime = val;
                nrg_prefs.putFloat("cost_life", cost_lifetime);
                Serial.printf("[NVS] Lifetime cost set to $%.2f\n", cost_lifetime);
                return;
            }
        }
    }

    // Direct value set commands: SV, SA, SM, SR, SP, SO
    if (buf[0] == 'S' && len >= 2) {
        if (buf[1] == 'V' && len >= 3) {
            // SV,84.0 -- set exact voltage
            const char *comma = strchr(&buf[2], ',');
            float v = atof(comma ? comma + 1 : &buf[2]);
            uint16_t dv = (uint16_t)(v * 10.0f);
            if (dv > CHARGER_VOLTAGE_MAX_DV) dv = CHARGER_VOLTAGE_MAX_DV;
            if (dv < CHARGER_VOLTAGE_MIN_DV) dv = CHARGER_VOLTAGE_MIN_DV;
            cmd_voltage_dv = dv;
            cmd_send_now();
            Serial.printf("[CMD] Voltage set to %.1fV\n", cmd_voltage_dv * 0.1f);
            return;
        }
        if (buf[1] == 'A' && len >= 3) {
            // SA,12.0 -- set exact current
            const char *comma = strchr(&buf[2], ',');
            float a = atof(comma ? comma + 1 : &buf[2]);
            uint16_t da = (uint16_t)(a * 10.0f);
            if (da > CHARGER_CURRENT_MAX_DA) da = CHARGER_CURRENT_MAX_DA;
            cmd_current_da = da;
            cmd_send_now();
            Serial.printf("[CMD] Current set to %.1fA\n", cmd_current_da * 0.1f);
            return;
        }
        if (buf[1] == 'M' && len >= 3) {
            // SM,84.0,12.0 -- set both voltage and current atomically
            const char *comma1 = strchr(&buf[2], ',');
            if (comma1) {
                float v = atof(comma1 + 1);
                const char *comma2 = strchr(comma1 + 1, ',');
                if (comma2) {
                    float a = atof(comma2 + 1);
                    uint16_t dv = (uint16_t)(v * 10.0f);
                    uint16_t da = (uint16_t)(a * 10.0f);
                    if (dv > CHARGER_VOLTAGE_MAX_DV) dv = CHARGER_VOLTAGE_MAX_DV;
                    if (dv < CHARGER_VOLTAGE_MIN_DV) dv = CHARGER_VOLTAGE_MIN_DV;
                    if (da > CHARGER_CURRENT_MAX_DA) da = CHARGER_CURRENT_MAX_DA;
                    cmd_voltage_dv = dv;
                    cmd_current_da = da;
                    cmd_send_now();
                    Serial.printf("[CMD] Set %.1fV / %.1fA\n",
                                  cmd_voltage_dv * 0.1f, cmd_current_da * 0.1f);
                }
            }
            return;
        }
        if (buf[1] == 'R') {
            // SR -- request full status dump (for app sync on connect)
            cmd_print_status();
            return;
        }
        if (buf[1] == 'P' && len >= 3) {
            // SP,3 -- set profile by number (1-6)
            const char *comma = strchr(&buf[2], ',');
            int pn = atoi(comma ? comma + 1 : &buf[2]);
            if (pn >= 1 && pn <= 6) {
                cmd_profile = (ChargeProfile_t)(pn - 1);  // MANUAL=0, STORAGE=1, etc.
                uint16_t sdv = profile_stop_voltage_dv();
                if (sdv > 0)
                    Serial.printf("[PROFILE] %s -- auto-stop at %.1fV\n",
                                  profile_name(cmd_profile), sdv * 0.1f);
                else
                    Serial.printf("[PROFILE] %s -- no auto-stop\n", profile_name(cmd_profile));
            } else {
                Serial.println("[CMD] SP: profile 1-6 (1=Manual,2=Storage,3=80%,4=85%,5=90%,6=100%)");
            }
            return;
        }
        if (buf[1] == 'O' && len >= 3) {
            // SO,A -- set outlet by letter (same as OA-OF/OG/OU)
            const char *comma = strchr(&buf[2], ',');
            // Reuse outlet handler by constructing an O-command
            char obuf[CMD_BUF_SIZE];
            obuf[0] = 'O';
            uint8_t olen = 1;
            // Copy everything after SO or SO,
            const char *src = comma ? comma + 1 : &buf[2];
            while (*src && olen < CMD_BUF_SIZE - 1) obuf[olen++] = *src++;
            obuf[olen] = '\0';
            cmd_execute_multi(obuf, olen);
            return;
        }
        // Clock sync: S1430 (4 digits after S)
        if (len == 5 && buf[1] >= '0' && buf[1] <= '2') {
            time_hour   = (buf[1] - '0') * 10 + (buf[2] - '0');
            time_minute = (buf[3] - '0') * 10 + (buf[4] - '0');
            if (time_hour > 23) time_hour = 23;
            if (time_minute > 59) time_minute = 59;
            time_valid = true;
            Serial.printf("[TIME] Set to %02d:%02d\n", time_hour, time_minute);
            return;
        }
    }

    // WiFi credential commands: WS, WP, WH, WM, WI
    if (buf[0] == 'W' && len >= 2) {
        if (buf[1] == 'S' && len >= 3) {
            // WS,MySSID -- set home WiFi SSID
            const char *comma = strchr(&buf[2], ',');
            const char *val = comma ? comma + 1 : &buf[2];
            strncpy(wifi_sta_ssid, val, sizeof(wifi_sta_ssid) - 1);
            wifi_sta_ssid[sizeof(wifi_sta_ssid) - 1] = '\0';
            nrg_prefs.putString("wifi_ssid", wifi_sta_ssid);
            Serial.printf("[WIFI] SSID set: %s\n", wifi_sta_ssid);
            return;
        }
        if (buf[1] == 'P' && len >= 3) {
            // WP,MyPassword -- set home WiFi password
            const char *comma = strchr(&buf[2], ',');
            const char *val = comma ? comma + 1 : &buf[2];
            strncpy(wifi_sta_pass, val, sizeof(wifi_sta_pass) - 1);
            wifi_sta_pass[sizeof(wifi_sta_pass) - 1] = '\0';
            nrg_prefs.putString("wifi_pass", wifi_sta_pass);
            Serial.printf("[WIFI] Password set (%u chars)\n", (unsigned)strlen(wifi_sta_pass));
            return;
        }
        if (buf[1] == 'H' && len >= 3) {
            // WH,192.168.1.100 -- set MQTT broker host
            const char *comma = strchr(&buf[2], ',');
            const char *val = comma ? comma + 1 : &buf[2];
            strncpy(mqtt_host, val, sizeof(mqtt_host) - 1);
            mqtt_host[sizeof(mqtt_host) - 1] = '\0';
            nrg_prefs.putString("mqtt_host", mqtt_host);
            Serial.printf("[MQTT] Broker set: %s\n", mqtt_host);
            return;
        }
        if (buf[1] == 'M' && len >= 3) {
            // WM,1883 -- set MQTT port
            const char *comma = strchr(&buf[2], ',');
            mqtt_port = (uint16_t)atoi(comma ? comma + 1 : &buf[2]);
            nrg_prefs.putUShort("mqtt_port", mqtt_port);
            Serial.printf("[MQTT] Port set: %u\n", mqtt_port);
            return;
        }
        if (buf[1] == 'I') {
            // WI -- show WiFi/MQTT info
            Serial.println();
            Serial.println("-------- WiFi / MQTT Info --------");
            Serial.printf("  SSID:      %s\n", wifi_sta_ssid[0] ? wifi_sta_ssid : "(not set)");
            Serial.printf("  Password:  %s\n", wifi_sta_pass[0] ? "****" : "(not set)");
            Serial.printf("  STA state: %s\n", wifi_sta_connected ? "CONNECTED" : "DISCONNECTED");
            if (wifi_sta_connected)
                Serial.printf("  STA IP:    %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("  MQTT host: %s\n", mqtt_host[0] ? mqtt_host : "(not set)");
            Serial.printf("  MQTT port: %u\n", mqtt_port);
            Serial.printf("  MQTT:      %s\n", mqtt_client.connected() ? "CONNECTED" : "DISCONNECTED");
            Serial.printf("  AP SSID:   %s\n", WIFI_AP_SSID);
            Serial.printf("  AP IP:     %s\n", WiFi.softAPIP().toString().c_str());
            Serial.println("----------------------------------");
            Serial.println();
            return;
        }
        if (buf[1] == 'C') {
            // WC -- force WiFi STA + MQTT connect now
            wifi_sta_connect();
            return;
        }
        if (buf[1] == 'D') {
            // WD -- disconnect WiFi STA
            WiFi.disconnect(true);
            wifi_sta_connected = false;
            mqtt_client.disconnect();
            Serial.println("[WIFI] STA disconnected");
            return;
        }
    }

    Serial.printf("[CMD] Unknown multi-char: %s\n", buf);
}

// Shared command processor -- called from Serial, BLE, and WiFi
static void cmd_process_char(char c) {
        // Multi-char command buffering: starts with O, T (when followed by Z/S/C)
        // Enter key or semicolon executes the buffer
        if (cmd_buf_active) {
            if (c == '\r' || c == '\n' || c == ';') {
                cmd_buf[cmd_buf_len] = '\0';
                if (cmd_buf_len == 1) {
                    // Single char was buffered (e.g. just 'T' + Enter)
                    // Process as single-char command via the switch below
                    cmd_buf_active = false;
                    c = cmd_buf[0];
                    cmd_buf_len = 0;
                    // fall through to switch statement
                } else if (cmd_buf_len > 0) {
                    cmd_execute_multi(cmd_buf, cmd_buf_len);
                    cmd_buf_len = 0;
                    cmd_buf_active = false;
                    return;
                } else {
                    cmd_buf_active = false;
                    return;
                }
            } else {
                if (cmd_buf_len < CMD_BUF_SIZE - 1) {
                    cmd_buf[cmd_buf_len++] = c;
                }
                return;
            }
        }

        // Check if this char starts a multi-char command
        // O = outlet presets, T = thermal/sensor/cal (TZ, TS, TC, TPMS)
        // S = direct set (SV, SA, SM, SR, SP, SO) or clock sync (S1430)
        // W = WiFi credential commands (WS, WP, WH, WM, WI)
        // Note: single 't' for schedule toggle is handled in the switch below
        // Note: single 's' (lowercase stop) handled in switch, uppercase 'S' buffers
        if (c == 'O' || c == 'N' || c == 'G' || c == 'P' || c == 'S' || c == 'W') {
            cmd_buf_active = true;
            cmd_buf_len = 0;
            cmd_buf[cmd_buf_len++] = c;
            return;
        }
        if (c == 'T') {
            // Could be single 't' for schedule or multi-char TZ/TS/TC
            // Buffer it -- if next char is Z/S/C, continue buffering
            // If next char is Enter/other, process 't' as schedule toggle
            cmd_buf_active = true;
            cmd_buf_len = 0;
            cmd_buf[cmd_buf_len++] = c;
            return;
        }

        switch (c) {

            // ---- Charge control ----
            case 'r':
                cmd_charging = true;
                nrg_active = true;
                nrg_last_ms = millis();
                if (!cmd_buddy_mode) {
                    // New session if not already running
                    if (nrg_session_start == 0) {
                        nrg_session_wh = 0.0f;
                        nrg_session_ah = 0.0f;
                        nrg_session_start = millis();
                    }
                } else {
                    if (nrg_buddy_start == 0) nrg_buddy_start = millis();
                }
                cmd_send_now();
                Serial.printf("[CMD] RESUME charging: %.1fV / %.1fA (%.0fW)\n",
                              cmd_voltage_dv * 0.1f, cmd_current_da * 0.1f,
                              cmd_voltage_dv * 0.1f * cmd_current_da * 0.1f);
                break;

            case 's':
                cmd_charging = false;
                nrg_active = false;
                can_send_disable();
                if (cmd_buddy_mode && nrg_buddy_wh > 0.1f) {
                    uint32_t el = nrg_buddy_start ? (millis() - nrg_buddy_start) / 1000 : 0;
                    float miles = nrg_buddy_wh / range_wh_per_mile;
                    Serial.printf("[CMD] STOP -- Buddy: %.1fWh / %.2fAh in %lu:%02lu (%.1f mi added)\n",
                                  nrg_buddy_wh, nrg_buddy_ah,
                                  (unsigned long)(el / 60), (unsigned long)(el % 60), miles);
                } else if (nrg_session_wh > 0.1f) {
                    uint32_t el = nrg_session_start ? (millis() - nrg_session_start) / 1000 : 0;
                    float miles = nrg_session_wh / range_wh_per_mile;
                    Serial.printf("[CMD] STOP -- Session: %.1fWh / %.2fAh in %lu:%02lu (%.1f mi added)\n",
                                  nrg_session_wh, nrg_session_ah,
                                  (unsigned long)(el / 60), (unsigned long)(el % 60), miles);
                    Serial.printf("       Lifetime: %.1fWh / %.1fAh\n", nrg_lifetime_wh, nrg_lifetime_ah);
                } else {
                    Serial.println("[CMD] STOP -- output disabled (charger stays awake)");
                }
                break;

            case 'e':
                cmd_charging = false;
                nrg_active = false;
                if (!cmd_buddy_mode && nrg_session_wh > 0.1f) {
                    nrg_prev_wh = nrg_session_wh;
                    nrg_prev_ah = nrg_session_ah;
                    nrg_prev_duration_s = nrg_session_start ? (millis() - nrg_session_start) / 1000 : 0;
                    float miles = nrg_session_wh / range_wh_per_mile;
                    Serial.printf("[CMD] END -- Session: %.1fWh / %.2fAh in %lu:%02lu (%.1f mi)\n",
                                  nrg_session_wh, nrg_session_ah,
                                  (unsigned long)(nrg_prev_duration_s / 60),
                                  (unsigned long)(nrg_prev_duration_s % 60), miles);
                    Serial.printf("       Lifetime: %.1fWh / %.1fAh\n", nrg_lifetime_wh, nrg_lifetime_ah);
                    nrg_session_wh = 0.0f;
                    nrg_session_ah = 0.0f;
                    nrg_session_start = 0;
                }
                can_send_sleep();
                nrg_prefs.putFloat("lifetime_wh", nrg_lifetime_wh);
                nrg_prefs.putFloat("lifetime_ah", nrg_lifetime_ah);
                Serial.println("[CMD] Charger entering sleep. Type 'w' then 'r' to restart.");
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
                Serial.println("       Type 'r' to start charging after charger responds.");
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
                    nrg_active     = false;
                    cmd_buddy_mode = false;
                    cmd_voltage_dv = HOME_VOLTAGE_DV;
                    cmd_current_da = HOME_CURRENT_DA;
                    can_send_disable();
                    Serial.println();
                    Serial.println("=== NORMAL MODE RESTORED ===");
                    if (nrg_buddy_wh > 0.1f) {
                        uint32_t el = nrg_buddy_start ? (millis() - nrg_buddy_start) / 1000 : 0;
                        Serial.printf( "  Buddy total: %.1fWh / %.2fAh in %lu:%02lu\n",
                                       nrg_buddy_wh, nrg_buddy_ah,
                                       (unsigned long)(el / 60), (unsigned long)(el % 60));
                    }
                    nrg_buddy_wh = 0.0f;
                    nrg_buddy_ah = 0.0f;
                    nrg_buddy_start = 0;
                    Serial.printf( "  Home settings: %.1fV / %.1fA (20S pack)\n",
                                   cmd_voltage_dv * 0.1f, cmd_current_da * 0.1f);
                    Serial.println("  Reconnect your pack, type 'r' to charge.");
                    Serial.println("============================");
                    Serial.println();
                } else {
                    Serial.println("[CMD] Already in normal mode.");
                }
                break;

            // ---- Maintain mode ----
            case 'm':
                maintain_enabled = !maintain_enabled;
                if (maintain_enabled) {
                    maintain_waiting = false;
                    Serial.printf("[CMD] MAINTAIN mode ON -- will top up after %.0f%% SOC drop\n",
                                  maintain_drop_pct);
                    Serial.println("       Charge to profile target first, then leave plugged in.");
                } else {
                    maintain_waiting = false;
                    Serial.println("[CMD] MAINTAIN mode OFF");
                }
                break;

            // ---- Schedule charge ----
            case 't':
                if (!schedule_enabled) {
                    schedule_enabled = true;
                    schedule_triggered = false;
                    uint16_t stop_dv = profile_stop_voltage_dv();
                    Serial.printf("[CMD] SCHEDULE ON -- target %s by %02d:%02d\n",
                                  profile_name(cmd_profile == PROFILE_MANUAL ? PROFILE_100 : cmd_profile),
                                  schedule_target_hour, schedule_target_min);
                    if (cmd_profile == PROFILE_MANUAL) {
                        cmd_profile = PROFILE_100;
                        Serial.println("       Profile set to 100% (was MANUAL)");
                    }
                    if (stop_dv > 0)
                        Serial.printf("       Stop voltage: %.1fV\n", stop_dv * 0.1f);
                    if (time_valid)
                        Serial.printf("       Current time: %02d:%02d (from GPS)\n", time_hour, time_minute);
                    else
                        Serial.println("       WARNING: No time source yet (waiting for GPS fix)");
                    Serial.println("       Set time with WiFi: /cmd?c=T0800 (for 08:00)");
                } else {
                    schedule_enabled = false;
                    schedule_triggered = false;
                    Serial.println("[CMD] SCHEDULE OFF");
                }
                break;

            // ---- Charge profiles (1-6) ----
            case '1':
                cmd_profile = PROFILE_MANUAL;
                Serial.println("[PROFILE] MANUAL -- no auto-stop, you control everything");
                break;
            case '2':
                cmd_profile = PROFILE_STORAGE;
                Serial.printf("[PROFILE] STORAGE -- auto-stop at %.1fV (%.3fV/cell)\n",
                              profile_stop_voltage_dv() * 0.1f, profile_cell_mv[PROFILE_STORAGE] / 1000.0f);
                break;
            case '3':
                cmd_profile = PROFILE_80;
                Serial.printf("[PROFILE] 80%% -- auto-stop at %.1fV (%.3fV/cell)\n",
                              profile_stop_voltage_dv() * 0.1f, profile_cell_mv[PROFILE_80] / 1000.0f);
                break;
            case '4':
                cmd_profile = PROFILE_85;
                Serial.printf("[PROFILE] 85%% -- auto-stop at %.1fV (%.3fV/cell)\n",
                              profile_stop_voltage_dv() * 0.1f, profile_cell_mv[PROFILE_85] / 1000.0f);
                break;
            case '5':
                cmd_profile = PROFILE_90;
                Serial.printf("[PROFILE] 90%% -- auto-stop at %.1fV (%.3fV/cell)\n",
                              profile_stop_voltage_dv() * 0.1f, profile_cell_mv[PROFILE_90] / 1000.0f);
                break;
            case '6':
                cmd_profile = PROFILE_100;
                Serial.printf("[PROFILE] 100%% -- auto-stop at %.1fV (%.3fV/cell)\n",
                              profile_stop_voltage_dv() * 0.1f, profile_cell_mv[PROFILE_100] / 1000.0f);
                break;

            // ---- Energy reset ----
            case '0':
                nrg_session_wh = 0.0f;
                nrg_session_ah = 0.0f;
                nrg_session_start = 0;
                nrg_prev_wh = 0.0f;
                nrg_prev_ah = 0.0f;
                nrg_prev_duration_s = 0;
                Serial.println("[CMD] Session & previous counters reset (lifetime preserved)");
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


// =============================================================================
// =============================================================================
//  WIFI SERVER (fallback when BLE signal is weak)
// =============================================================================

static WebServer wifi_server(WIFI_AP_PORT);
static bool wifi_active = false;

static String wifi_build_status_json(void) {
    ChargerStatus_t cs;
    bool has_rx = can_read_status(&cs);

    String json = "{";
    json += "\"mode\":\"" + String(cmd_buddy_mode ? "BUDDY" : "HOME") + "\",";
    json += "\"charging\":" + String(cmd_charging ? "true" : "false") + ",";
    json += "\"set_voltage\":" + String(cmd_voltage_dv * 0.1f, 1) + ",";
    json += "\"set_current\":" + String(cmd_current_da * 0.1f, 1) + ",";
    json += "\"smooth_voltage\":" + String(can_smooth_voltage, 1) + ",";
    json += "\"smooth_current\":" + String(can_smooth_current, 1) + ",";
    json += "\"power_w\":" + String((int)(can_smooth_voltage * can_smooth_current)) + ",";
    json += "\"charger_responding\":" + String(has_rx ? "true" : "false") + ",";
    if (has_rx) {
        json += "\"charger_temp_f\":" + String((int)(cs.temperature_c * 9 / 5 + 32)) + ",";
        json += "\"working_status\":" + String(cs.status.working_status) + ",";
        json += "\"faults\":" + String(
            cs.fault.hardware_fault | cs.fault.temperature_fault |
            cs.fault.output_undervoltage | cs.fault.output_overvoltage |
            cs.fault.output_overcurrent | cs.fault.output_short_circuit) + ",";
    }
    json += "\"session_wh\":" + String(cmd_buddy_mode ? nrg_buddy_wh : nrg_session_wh, 1) + ",";
    json += "\"session_ah\":" + String(cmd_buddy_mode ? nrg_buddy_ah : nrg_session_ah, 2) + ",";
    json += "\"lifetime_wh\":" + String(nrg_lifetime_wh, 1) + ",";
    json += "\"lifetime_ah\":" + String(nrg_lifetime_ah, 1) + ",";
    json += "\"prev_wh\":" + String(nrg_prev_wh, 1) + ",";
    json += "\"miles_added\":" + String((cmd_buddy_mode ? nrg_buddy_wh : nrg_session_wh) / range_wh_per_mile, 1) + ",";
    json += "\"wh_per_mile\":" + String(range_wh_per_mile, 1) + ",";
    json += "\"profile\":\"" + String(profile_name(cmd_profile)) + "\",";
    uint16_t sdv = profile_stop_voltage_dv();
    json += "\"profile_stop_v\":" + String(sdv > 0 ? sdv * 0.1f : 0.0f, 1) + ",";
    json += "\"maintain\":" + String(maintain_enabled ? "true" : "false") + ",";
    json += "\"maintain_waiting\":" + String(maintain_waiting ? "true" : "false") + ",";
    json += "\"schedule\":" + String(schedule_enabled ? "true" : "false") + ",";
    json += "\"schedule_time\":\"" + String(schedule_target_hour) + ":" +
            (schedule_target_min < 10 ? "0" : "") + String(schedule_target_min) + "\",";
    json += "\"outlet\":\"" + String(outlet_name()) + "\",";
    json += "\"outlet_max_w\":" + String(outlet_max_watts()) + ",";
    json += "\"test_dev\":" + String(test_dev_mode ? "true" : "false") + ",";
    json += "\"peak_watts\":" + String((int)input_peak_watts) + ",";
    float mbf = get_max_battery_temp_f();
    json += "\"batt_temp_f\":" + String(mbf > -900.0f ? mbf : 0.0f, 1) + ",";
    json += "\"thermal_zone\":\"" + String(mbf > -900.0f ? thermal_zone_name(mbf) : "NONE") + "\",";
    json += "\"thermal_derate\":" + String(thermal_zone_multiplier(mbf) * 100.0f, 0) + ",";
    json += "\"time_valid\":" + String(time_valid ? "true" : "false") + ",";
    json += "\"time\":\"" + String(time_hour) + ":" +
            (time_minute < 10 ? "0" : "") + String(time_minute) + "\",";
    json += "\"state\":\"" + String(safety_state_name(safety_state)) + "\",";
    json += "\"cost_kwh\":" + String(cost_per_kwh, 3) + ",";
    json += "\"cost_session\":" + String(cost_session, 2) + ",";
    json += "\"cost_lifetime\":" + String(cost_lifetime, 2) + ",";
    json += "\"cost_free_kwh\":" + String(cost_free_lifetime, 1) + ",";
    json += "\"gps_source\":\"" + String(gps_source == GPS_SOURCE_ONBOARD ? "ONBOARD" :
                                         gps_source == GPS_SOURCE_PHONE ? "PHONE" : "BOTH") + "\",";
    float ttc = thermal_time_to_caution_min();
    json += "\"thermal_mins_to_caution\":" + String(ttc > 0 ? ttc : -1.0f, 0) + ",";
    json += "\"tpms_enabled\":" + String(tpms_enabled ? "true" : "false");
    if (tpms_enabled) {
        json += ",\"tpms_front_psi\":" + String(tpms_front_psi, 1);
        json += ",\"tpms_front_temp_f\":" + String(tpms_front_temp_f, 0);
        json += ",\"tpms_front_batt\":" + String(tpms_front_battery, 1);
        json += ",\"tpms_front_valid\":" + String(tpms_front_valid ? "true" : "false");
        json += ",\"tpms_rear_psi\":" + String(tpms_rear_psi, 1);
        json += ",\"tpms_rear_temp_f\":" + String(tpms_rear_temp_f, 0);
        json += ",\"tpms_rear_batt\":" + String(tpms_rear_battery, 1);
        json += ",\"tpms_rear_valid\":" + String(tpms_rear_valid ? "true" : "false");
    }
    json += ",\"wifi_sta\":" + String(wifi_sta_connected ? "true" : "false");
    json += ",\"mqtt\":" + String(mqtt_client.connected() ? "true" : "false");
    json += ",\"fw\":\"" + String(FW_VERSION) + "\"";
    json += "}";
    return json;
}

// WebSocket-style endpoint: /ws returns JSON continuously (long-poll)
// True WebSocket requires AsyncWebServer library -- this is a simple SSE alternative
// The app can use EventSource (Server-Sent Events) for real-time updates
static void wifi_handle_events(void) {
    WiFiClient client = wifi_server.client();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/event-stream");
    client.println("Cache-Control: no-cache");
    client.println("Connection: keep-alive");
    client.println();

    // Send one event (client reconnects for next)
    client.print("data: ");
    client.println(wifi_build_status_json());
    client.println();
}

static void wifi_handle_root(void) {
    wifi_server.send(200, "text/plain",
        "TC_Charger WiFi API\n"
        "GET  /status  - JSON telemetry\n"
        "GET  /cmd?c=X - Send command\n"
        "  Single: r,s,e,w,+,-,v,V,b,n,m,t,1-6,0,p,?\n"
        "  Direct: SV,84.0  SA,12.0  SM,84.0,12.0  SR  SP,3  SO,A\n"
        "  Outlet: OA-OF  OG24  OU110,20\n"
        "  WiFi:   WS,SSID  WP,PASS  WH,IP  WM,1883  WI  WC  WD\n"
        "GET  /events  - Server-Sent Events stream\n");
}

static void wifi_handle_status(void) {
    wifi_server.send(200, "application/json", wifi_build_status_json());
}

static void wifi_handle_cmd(void) {
    if (wifi_server.hasArg("c")) {
        String cmd = wifi_server.arg("c");

        // Multi-char commands (O*, TZ*, TS*, TC*, T0800, S1430)
        if (cmd.length() >= 2) {
            char first = cmd.charAt(0);

            // Schedule time: T0800
            if (first == 'T' && cmd.length() == 5 && cmd.charAt(1) >= '0' && cmd.charAt(1) <= '2') {
                schedule_target_hour = (cmd.charAt(1) - '0') * 10 + (cmd.charAt(2) - '0');
                schedule_target_min  = (cmd.charAt(3) - '0') * 10 + (cmd.charAt(4) - '0');
                if (schedule_target_hour > 23) schedule_target_hour = 23;
                if (schedule_target_min > 59)  schedule_target_min = 59;
                Serial.printf("[SCHEDULE] Target time set to %02d:%02d\n",
                              schedule_target_hour, schedule_target_min);
                wifi_server.send(200, "application/json",
                    "{\"ok\":true,\"schedule_time\":\"" +
                    String(schedule_target_hour) + ":" + String(schedule_target_min) + "\"}");
                return;
            }

            // Clock sync: S1430
            if (first == 'S' && cmd.length() == 5 && cmd.charAt(1) >= '0' && cmd.charAt(1) <= '2') {
                time_hour   = (cmd.charAt(1) - '0') * 10 + (cmd.charAt(2) - '0');
                time_minute = (cmd.charAt(3) - '0') * 10 + (cmd.charAt(4) - '0');
                time_valid  = true;
                Serial.printf("[TIME] Set to %02d:%02d (from app)\n", time_hour, time_minute);
                wifi_server.send(200, "application/json",
                    "{\"ok\":true,\"time\":\"" +
                    String(time_hour) + ":" + String(time_minute) + "\"}");
                return;
            }

            // Outlet, ThermalZone, TempSensor, TempCal, DirectSet, WiFi creds
            if (first == 'O' || first == 'N' || first == 'G' || first == 'P' ||
                first == 'S' || first == 'W' ||
                (first == 'T' && cmd.length() >= 3 &&
                 (cmd.charAt(1) == 'Z' || cmd.charAt(1) == 'S' || cmd.charAt(1) == 'C'))) {
                char buf[CMD_BUF_SIZE];
                cmd.toCharArray(buf, CMD_BUF_SIZE);
                cmd_execute_multi(buf, cmd.length());
                wifi_server.send(200, "application/json",
                    "{\"ok\":true,\"cmd\":\"" + cmd + "\"}");
                return;
            }
        }

        // Single-char commands
        for (unsigned int i = 0; i < cmd.length(); i++) {
            cmd_process_char(cmd.charAt(i));
        }
        wifi_server.send(200, "application/json",
            "{\"ok\":true,\"cmd\":\"" + cmd + "\"}");
    } else {
        wifi_server.send(400, "application/json",
            "{\"ok\":false,\"error\":\"missing ?c= parameter\"}");
    }
}

static void wifi_init(void) {
    // Set WiFi mode upfront: AP+STA if home credentials exist, AP-only otherwise
    if (wifi_sta_ssid[0] != '\0') {
        WiFi.mode(WIFI_AP_STA);
    } else {
        WiFi.mode(WIFI_AP);
    }

    // Start AP (always available for direct connection)
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    wifi_server.on("/", wifi_handle_root);
    wifi_server.on("/status", wifi_handle_status);
    wifi_server.on("/events", wifi_handle_events);
    wifi_server.on("/cmd", wifi_handle_cmd);
    wifi_server.begin();
    wifi_active = true;
    Serial.printf("[WIFI] AP started: %s / %s  http://%s\n",
                  WIFI_AP_SSID, WIFI_AP_PASS,
                  WiFi.softAPIP().toString().c_str());

    // If home WiFi credentials are stored, start STA connection
    if (wifi_sta_ssid[0] != '\0') {
        Serial.printf("[WIFI] Connecting to '%s'...\n", wifi_sta_ssid);
        WiFi.begin(wifi_sta_ssid, wifi_sta_pass);
        wifi_sta_retry_ms = millis();
    }
}

static void wifi_update(void) {
    if (wifi_active) wifi_server.handleClient();
}

// =============================================================================
//  WIFI STATION MODE + MQTT (Home Assistant)
// =============================================================================
// Connects to home WiFi in STA mode (concurrent with AP mode) and publishes
// charger telemetry to an MQTT broker for Home Assistant auto-discovery.
// Credentials stored in NVS, configurable via serial (WS/WP/WH/WM), BLE, or WiFi API.

static void wifi_sta_connect(void) {
    if (wifi_sta_ssid[0] == '\0') {
        Serial.println("[WIFI] No SSID configured. Use WS,YourSSID then WP,YourPassword");
        return;
    }
    Serial.printf("[WIFI] Connecting to '%s'...\n", wifi_sta_ssid);
    // Switch to AP+STA dual mode (preserves running AP)
    if (WiFi.getMode() != WIFI_AP_STA) {
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);  // re-init AP after mode change
    }
    WiFi.begin(wifi_sta_ssid, wifi_sta_pass);
    wifi_sta_retry_ms = millis();
}

static void mqtt_callback(char *topic, byte *payload, unsigned int length) {
    // Commands from Home Assistant -> charger
    char buf[CMD_BUF_SIZE];
    unsigned int blen = length;
    if (blen > CMD_BUF_SIZE - 1) blen = CMD_BUF_SIZE - 1;
    memcpy(buf, payload, blen);
    buf[blen] = '\0';

    Serial.printf("[MQTT] cmd: %s\n", buf);

    // Process as multi-char or single-char command
    if (blen >= 2) {
        cmd_execute_multi(buf, (uint8_t)blen);
    } else if (blen == 1) {
        cmd_process_char(buf[0]);
    }
}

static void mqtt_send_discovery(void) {
    // Send Home Assistant MQTT auto-discovery config for key entities
    // Each entity: topic homeassistant/sensor/scooter_charger/{id}/config
    struct {
        const char *id;
        const char *name;
        const char *unit;
        const char *dev_class;
        const char *val_tpl;
    } entities[] = {
        {"voltage",     "Charger Voltage",    "V",    "voltage",      "{{ value_json.smooth_voltage }}"},
        {"current",     "Charger Current",    "A",    "current",      "{{ value_json.smooth_current }}"},
        {"power",       "Charger Power",      "W",    "power",        "{{ value_json.power_w }}"},
        {"session_wh",  "Session Energy",     "Wh",   "energy",       "{{ value_json.session_wh }}"},
        {"lifetime_wh", "Lifetime Energy",    "Wh",   "energy",       "{{ value_json.lifetime_wh }}"},
        {"batt_temp",   "Battery Temp",       "F",    "temperature",  "{{ value_json.batt_temp_f }}"},
        {"cost_session","Session Cost",       "$",    "monetary",     "{{ value_json.cost_session }}"},
        {"state",       "Charger State",      "",     "",             "{{ value_json.state }}"},
    };

    for (unsigned i = 0; i < sizeof(entities)/sizeof(entities[0]); i++) {
        String topic = String(MQTT_TOPIC_DISCOVERY) + "/" + entities[i].id + "/config";
        String payload = "{";
        payload += "\"name\":\"" + String(entities[i].name) + "\",";
        payload += "\"stat_t\":\"" + String(MQTT_TOPIC_STATE) + "\",";
        payload += "\"val_tpl\":\"" + String(entities[i].val_tpl) + "\",";
        payload += "\"uniq_id\":\"scooter_charger_" + String(entities[i].id) + "\",";
        if (entities[i].unit[0])
            payload += "\"unit_of_meas\":\"" + String(entities[i].unit) + "\",";
        if (entities[i].dev_class[0])
            payload += "\"dev_cla\":\"" + String(entities[i].dev_class) + "\",";
        payload += "\"dev\":{";
        payload += "\"ids\":[\"scooter_charger_esp32\"],";
        payload += "\"name\":\"Scooter Charger\",";
        payload += "\"mf\":\"TC_Charger\",";
        payload += "\"mdl\":\"HK-LF-115-58 6.6kW\"";
        payload += "}}";

        mqtt_client.publish(topic.c_str(), payload.c_str(), true);  // retained
    }
    Serial.println("[MQTT] Home Assistant auto-discovery sent");
}

static void mqtt_publish_state(void) {
    // Reuse the same JSON builder as the WiFi /status endpoint
    String json = wifi_build_status_json();
    mqtt_client.publish(MQTT_TOPIC_STATE, json.c_str());
}

static void wifi_sta_update(void) {
    uint32_t now = millis();

    // Skip if no credentials configured
    if (wifi_sta_ssid[0] == '\0') return;

    // Check STA connection state
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifi_sta_connected) {
            wifi_sta_connected = true;
            Serial.printf("[WIFI] Connected to '%s' -- IP: %s\n",
                          wifi_sta_ssid, WiFi.localIP().toString().c_str());

            // Set up MQTT if broker is configured
            if (mqtt_host[0] != '\0') {
                mqtt_client.setServer(mqtt_host, mqtt_port);
                mqtt_client.setCallback(mqtt_callback);
                mqtt_client.setBufferSize(512);  // HA discovery payloads can be large
            }
        }

        // MQTT connection management
        if (mqtt_host[0] != '\0') {
            if (!mqtt_client.connected()) {
                if ((now - wifi_sta_retry_ms) >= WIFI_STA_RETRY_MS) {
                    wifi_sta_retry_ms = now;
                    Serial.printf("[MQTT] Connecting to %s:%u...\n", mqtt_host, mqtt_port);
                    if (mqtt_client.connect(MQTT_CLIENT_ID)) {
                        Serial.println("[MQTT] Connected");
                        mqtt_client.subscribe(MQTT_TOPIC_CMD);
                        if (!mqtt_discovery_sent) {
                            mqtt_send_discovery();
                            mqtt_discovery_sent = true;
                        }
                    } else {
                        Serial.printf("[MQTT] Failed, rc=%d\n", mqtt_client.state());
                    }
                }
            } else {
                mqtt_client.loop();

                // Publish telemetry periodically
                if ((now - mqtt_last_publish_ms) >= MQTT_PUBLISH_MS) {
                    mqtt_last_publish_ms = now;
                    mqtt_publish_state();
                }
            }
        }
    } else {
        if (wifi_sta_connected) {
            wifi_sta_connected = false;
            mqtt_discovery_sent = false;
            Serial.println("[WIFI] STA disconnected");
        }
        // Auto-retry connection
        if ((now - wifi_sta_retry_ms) >= WIFI_STA_RETRY_MS) {
            wifi_sta_retry_ms = now;
            WiFi.begin(wifi_sta_ssid, wifi_sta_pass);
        }
    }
}

// =============================================================================
//  CAN INTER-MODULE FRAMES
// =============================================================================
// Other modules on the shared CAN bus can send commands to the charger Main Brain.
// IDs and command bytes defined at top of file with other CAN constants.

static void can_process_intermod(const twai_message_t *msg) {
    if (msg->data_length_code < 1) return;
    uint8_t cmd_type = msg->data[0];

    switch (cmd_type) {
        case INTERMOD_CMD_ESTOP:
            cmd_charging = false;
            nrg_active = false;
            can_send_disable();
            safety_state = SAFETY_STATE_FAULT;
            rgb_led_set_state(RGB_STATE_FAULT);
            Serial.println("[CAN] EMERGENCY STOP from external module");
            break;

        case INTERMOD_CMD_START:
            cmd_charging = true;
            nrg_active = true;
            nrg_last_ms = millis();
            cmd_send_now();
            Serial.println("[CAN] Charge START from external module");
            break;

        case INTERMOD_CMD_STOP:
            cmd_charging = false;
            nrg_active = false;
            can_send_disable();
            Serial.println("[CAN] Charge STOP from external module");
            break;

        case INTERMOD_CMD_SET_AMPS:
            if (msg->data_length_code >= 3) {
                uint16_t req_da = ((uint16_t)msg->data[1] << 8) | msg->data[2];
                if (req_da > CHARGER_CURRENT_MAX_DA) req_da = CHARGER_CURRENT_MAX_DA;
                cmd_current_da = req_da;
                cmd_send_now();
                Serial.printf("[CAN] Current set to %.1fA from external module\n",
                              cmd_current_da * 0.1f);
            }
            break;

        case INTERMOD_CMD_SET_VOLTS:
            if (msg->data_length_code >= 3) {
                uint16_t req_dv = ((uint16_t)msg->data[1] << 8) | msg->data[2];
                if (req_dv > CHARGER_VOLTAGE_MAX_DV) req_dv = CHARGER_VOLTAGE_MAX_DV;
                if (req_dv < CHARGER_VOLTAGE_MIN_DV) req_dv = CHARGER_VOLTAGE_MIN_DV;
                cmd_voltage_dv = req_dv;
                cmd_send_now();
                Serial.printf("[CAN] Voltage set to %.1fV from external module\n",
                              cmd_voltage_dv * 0.1f);
            }
            break;

        case INTERMOD_CMD_SET_PROFILE:
            if (msg->data_length_code >= 2 && msg->data[1] < PROFILE_COUNT) {
                cmd_profile = (ChargeProfile_t)msg->data[1];
                Serial.printf("[CAN] Profile set to %s from external module\n",
                              profile_name(cmd_profile));
            }
            break;
    }
}

// =============================================================================
//  SETUP & LOOP
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  TC_Charger Main Brain  v" FW_VERSION);
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

    // Load settings from NVS flash (must be before wifi_init for STA credentials)
    nrg_prefs.begin("charger_nrg", false);
    nrg_lifetime_wh = nrg_prefs.getFloat("lifetime_wh", 0.0f);
    nrg_lifetime_ah = nrg_prefs.getFloat("lifetime_ah", 0.0f);
    range_wh_per_mile = nrg_prefs.getFloat("wh_per_mile", WH_PER_MILE_DEFAULT);

    // Load outlet preset
    outlet_preset = (OutletPreset_t)nrg_prefs.getUChar("outlet", (uint8_t)OUTLET_110_15A);
    outlet_voltage_ac = nrg_prefs.getUShort("outlet_v", 110);
    outlet_amps_ac = nrg_prefs.getUShort("outlet_a", 15);
    test_dev_mode = (outlet_preset == OUTLET_TEST_DEV);

    // Load thermal zones
    tz_normal_max_f  = nrg_prefs.getFloat("tz1", 140.0f);
    tz_caution_max_f = nrg_prefs.getFloat("tz2", 194.0f);
    tz_danger_max_f  = nrg_prefs.getFloat("tz3", 248.0f);

    // Load calibration offsets
    for (int i = 1; i <= 5; i++) {
        char key[8];
        snprintf(key, sizeof(key), "cal%d", i);
        sensor_ntc_offset_f[i] = nrg_prefs.getFloat(key, 0.0f);
    }
    sensor_bms1_offset_f = nrg_prefs.getFloat("calb1", 0.0f);
    sensor_bms2_offset_f = nrg_prefs.getFloat("calb2", 0.0f);

    // Load cost tracking
    cost_per_kwh = nrg_prefs.getFloat("cost_kwh", 0.12f);
    cost_lifetime = nrg_prefs.getFloat("cost_life", 0.0f);
    cost_free_lifetime = nrg_prefs.getFloat("cost_free", 0.0f);

    // Load WiFi STA + MQTT credentials
    {
        String s = nrg_prefs.getString("wifi_ssid", "");
        s.toCharArray(wifi_sta_ssid, sizeof(wifi_sta_ssid));
        s = nrg_prefs.getString("wifi_pass", "");
        s.toCharArray(wifi_sta_pass, sizeof(wifi_sta_pass));
        s = nrg_prefs.getString("mqtt_host", "");
        s.toCharArray(mqtt_host, sizeof(mqtt_host));
        mqtt_port = nrg_prefs.getUShort("mqtt_port", 1883);
    }

    // Load TPMS settings
    tpms_enabled = nrg_prefs.getBool("tpms_on", false);
    {
        String s = nrg_prefs.getString("tpms_f_mac", "");
        s.toCharArray(tpms_front_mac, sizeof(tpms_front_mac));
        s = nrg_prefs.getString("tpms_r_mac", "");
        s.toCharArray(tpms_rear_mac, sizeof(tpms_rear_mac));
    }
    tpms_alert_low_psi     = nrg_prefs.getFloat("tpms_low",  30.0f);
    tpms_alert_crit_psi    = nrg_prefs.getFloat("tpms_crit", 20.0f);
    tpms_alert_high_psi    = nrg_prefs.getFloat("tpms_high", 48.0f);
    tpms_alert_high_temp_f = nrg_prefs.getFloat("tpms_ht",   158.0f);
    tpms_alert_crit_temp_f = nrg_prefs.getFloat("tpms_ct",   185.0f);

    Serial.printf("[NVS] Lifetime: %.1fWh / %.1fAh  Efficiency: %.1f Wh/mi\n",
                  nrg_lifetime_wh, nrg_lifetime_ah, range_wh_per_mile);
    Serial.printf("[NVS] Outlet: %s (max %uW)%s\n",
                  outlet_name(), outlet_max_watts(),
                  test_dev_mode ? " [DEV MODE]" : "");
    Serial.printf("[NVS] Thermal zones: NORMAL<%.0fF CAUTION<%.0fF DANGER<%.0fF\n",
                  tz_normal_max_f, tz_caution_max_f, tz_danger_max_f);
    if (wifi_sta_ssid[0])
        Serial.printf("[NVS] Home WiFi: %s  MQTT: %s:%u\n",
                      wifi_sta_ssid,
                      mqtt_host[0] ? mqtt_host : "(none)",
                      mqtt_port);
    if (tpms_enabled)
        Serial.printf("[NVS] TPMS: ON  Front=%s  Rear=%s\n",
                      tpms_front_mac[0] ? tpms_front_mac : "(none)",
                      tpms_rear_mac[0] ? tpms_rear_mac : "(none)");

    // WiFi AP (+ STA if credentials exist) -- after NVS loaded
    wifi_init();
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
    wifi_update();
    wifi_sta_update();
    tpms_scan_update();
}
