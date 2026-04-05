#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef MAIN_BRAIN

#define GPS_TX_PIN              16      // S3 GPIO16 → GPS module RX
#define GPS_RX_PIN              15      // S3 GPIO15 ← GPS module TX
#define GPS_BAUD                115200
#define GPS_COMPASS_ADDR        0x0D    // QMC5883L fixed I2C address
#define GPS_CONSUMPTION_WH_PER_KM  20.0f  // default Wh/km — adjust for your scooter

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

#ifdef __cplusplus
extern "C" {
#endif

void gps_driver_init(void);
void gps_driver_update(void);
bool gps_driver_get(GPSData_t *out);

#ifdef __cplusplus
}
#endif

#endif // MAIN_BRAIN
