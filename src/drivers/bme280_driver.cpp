#include "bme280_driver.h"
#if defined(MAIN_BRAIN) || defined(TEMP_NODE)

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// ---- I2C pin assignments ----
#if defined(MAIN_BRAIN)
  // ESP32-S3 DevKitC-1: shared bus with OLED (Wire already started by oled_display_init)
  // GPIO8 = SDA, GPIO9 = SCL
  #define ENV_SDA_PIN     8
  #define ENV_SCL_PIN     9
  #define ENV_WIRE_INIT   0   // OLED init already called Wire.begin()
#elif defined(TEMP_NODE)
  // ESP32-C3 Super Mini: GPIO8 = SDA, GPIO9 = SCL
  #define ENV_SDA_PIN     8
  #define ENV_SCL_PIN     9
  #define ENV_WIRE_INIT   1
#endif

// AHT20: fixed I2C address 0x38
// BMP280: 0x76 (SDO=GND) or 0x77 (SDO=3V3) — we probe both
#define BMP280_ADDR_A   0x76
#define BMP280_ADDR_B   0x77

#define ENV_UPDATE_MS   2000

static Adafruit_AHTX0  s_aht;
static Adafruit_BMP280 s_bmp(&Wire);
static BME280Data_t    s_data;
static bool            s_aht_ok  = false;
static bool            s_bmp_ok  = false;
static uint32_t        s_last_ms = 0;

void bme280_init(void) {
#if ENV_WIRE_INIT
    Wire.begin(ENV_SDA_PIN, ENV_SCL_PIN);
#endif

    s_aht_ok = s_aht.begin(&Wire);
    if (!s_aht_ok)
        Serial.println("[ENV] AHT20 not found — check SDA/SCL wiring");
    else
        Serial.println("[ENV] AHT20 init OK");

    s_bmp_ok = s_bmp.begin(BMP280_ADDR_A);
    if (!s_bmp_ok) s_bmp_ok = s_bmp.begin(BMP280_ADDR_B);
    if (!s_bmp_ok) {
        Serial.println("[ENV] BMP280 not found — check SDA/SCL wiring and SDO pin");
    } else {
        s_bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                          Adafruit_BMP280::SAMPLING_X2,
                          Adafruit_BMP280::SAMPLING_X16,
                          Adafruit_BMP280::FILTER_X4,
                          Adafruit_BMP280::STANDBY_MS_500);
        Serial.println("[ENV] BMP280 init OK");
    }

    s_data.valid = s_aht_ok || s_bmp_ok;
    s_last_ms    = millis();

    if (s_data.valid) {
        sensors_event_t hum, temp;
        if (s_aht_ok && s_aht.getEvent(&hum, &temp)) {
            s_data.temperature_c = temp.temperature;
            s_data.humidity_pct  = hum.relative_humidity;
        }
        if (s_bmp_ok) {
            float p = s_bmp.readPressure();
            s_data.pressure_hpa = (p > 0.0f) ? p / 100.0f : 0.0f;
            if (!s_aht_ok) s_data.temperature_c = s_bmp.readTemperature();
        }
        Serial.printf("[ENV] T=%.1fC H=%.1f%% P=%.1fhPa\n",
                      s_data.temperature_c, s_data.humidity_pct, s_data.pressure_hpa);
    }
}

void bme280_update(void) {
    if (!s_aht_ok && !s_bmp_ok) return;
    uint32_t now = millis();
    if ((now - s_last_ms) < ENV_UPDATE_MS) return;
    s_last_ms = now;

    bool ok = false;

    if (s_aht_ok) {
        sensors_event_t hum, temp;
        if (s_aht.getEvent(&hum, &temp)) {
            s_data.temperature_c = temp.temperature;
            s_data.humidity_pct  = hum.relative_humidity;
            ok = true;
        }
    }

    if (s_bmp_ok) {
        float p = s_bmp.readPressure();
        float t = s_bmp.readTemperature();
        if (!isnan(p) && p > 30000.0f) {
            s_data.pressure_hpa = p / 100.0f;
            ok = true;
        }
        if (!s_aht_ok && !isnan(t)) s_data.temperature_c = t;
    }

    s_data.valid = ok;
    if (!ok) Serial.println("[ENV] read failed — check wiring");
}

bool bme280_get(BME280Data_t *out) {
    if (!out) return false;
    *out = s_data;
    return s_data.valid;
}

#endif // MAIN_BRAIN || TEMP_NODE
