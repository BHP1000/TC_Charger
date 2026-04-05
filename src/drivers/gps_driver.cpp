#ifdef MAIN_BRAIN

#include "gps_driver.h"
#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include "rs485_driver.h"

static TinyGPSPlus s_gps;
static GPSData_t   s_data;
static bool        s_compass_ok = false;

// ---- QMC5883L registers ------------------------------------------------
#define QMC_REG_DATA    0x00
#define QMC_REG_STATUS  0x06
#define QMC_REG_CTRL1   0x09
#define QMC_REG_PERIOD  0x0B
#define QMC_STATUS_DRDY 0x01

static void compass_init(void) {
    Wire.beginTransmission(GPS_COMPASS_ADDR);
    Wire.write(QMC_REG_PERIOD);
    Wire.write(0x01);
    s_compass_ok = (Wire.endTransmission() == 0);
    if (!s_compass_ok) {
        Serial.println("[GPS] QMC5883L not found — compass disabled");
        return;
    }
    Wire.beginTransmission(GPS_COMPASS_ADDR);
    Wire.write(QMC_REG_CTRL1);
    Wire.write(0x1D);   // continuous mode, 200 Hz ODR, 8 G range, OSR 512
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

// ---- range estimation --------------------------------------------------

static float estimate_range_km(void) {
    BMSData_t bms = {};
    if (!rs485_get_last_bms(&bms) || !bms.valid) return -1.0f;
    if (bms.remaining_mah <= 0) return 0.0f;
    // remaining_Wh = remaining_mAh * pack_V / 1000
    float remaining_wh = ((float)bms.remaining_mah / 1000.0f) *
                         ((float)bms.pack_mv        / 1000000.0f) * 1000.0f;
    return remaining_wh / GPS_CONSUMPTION_WH_PER_KM;
}

// ---- public ------------------------------------------------------------

void gps_driver_init(void) {
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    compass_init();
    memset(&s_data, 0, sizeof(s_data));
    s_data.range_km = -1.0f;
    Serial.printf("[GPS] init OK — RX=GPIO%d TX=GPIO%d baud=%d\n",
                  GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
}

void gps_driver_update(void) {
    while (Serial2.available())
        s_gps.encode((char)Serial2.read());

    if (s_gps.location.isValid()) {
        s_data.lat       = (float)s_gps.location.lat();
        s_data.lon       = (float)s_gps.location.lng();
        s_data.gps_valid = true;
    }
    if (s_gps.speed.isValid())
        s_data.speed_kmh  = (float)s_gps.speed.kmph();
    if (s_gps.course.isValid())
        s_data.course_deg = (float)s_gps.course.deg();

    s_data.fix        = s_gps.location.isValid() ? 1 : 0;
    s_data.satellites = s_gps.satellites.isValid()
                        ? (uint8_t)s_gps.satellites.value() : 0;

    float hdg = 0.0f;
    s_data.compass_valid = compass_read(&hdg);
    if (s_data.compass_valid) s_data.heading_deg = hdg;

    s_data.range_km = estimate_range_km();
}

bool gps_driver_get(GPSData_t *out) {
    if (!out) return false;
    *out = s_data;
    return s_data.gps_valid;
}

#endif // MAIN_BRAIN
