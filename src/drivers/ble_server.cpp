#include "ble_server.h"
#ifdef MAIN_BRAIN

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include "thermal_logic.h"
#include "can_driver.h"
#include "rs485_driver.h"
#include "gps_driver.h"

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

static BLEServer           *s_server      = nullptr;
static BLECharacteristic   *s_ch_state    = nullptr;
static BLECharacteristic   *s_ch_temp     = nullptr;
static BLECharacteristic   *s_ch_cur      = nullptr;
static BLECharacteristic   *s_ch_voltage  = nullptr;
static BLECharacteristic   *s_ch_bms_pack    = nullptr;
static BLECharacteristic   *s_ch_bms_health  = nullptr;
static BLECharacteristic   *s_ch_bms_cells   = nullptr;
static BLECharacteristic   *s_ch_gps         = nullptr;
static bool                 s_connected  = false;
static uint16_t             s_pack_voltage_dv = 840;
static uint32_t             s_last_notify_ms  = 0;

class ServerCB : public BLEServerCallbacks {
    void onConnect(BLEServer *)    override { s_connected = true;  Serial.println("[BLE] client connected"); }
    void onDisconnect(BLEServer *srv) override {
        s_connected = false;
        Serial.println("[BLE] client disconnected, restarting advertising");
        srv->startAdvertising();
    }
};

class VoltageCB : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *ch) override {
        if (ch->getLength() >= 2) {
            const uint8_t *d = ch->getData();
            s_pack_voltage_dv = (uint16_t)(d[0] | (d[1] << 8));
            Serial.printf("[BLE] pack voltage set -> %u dV (%.1f V)\n",
                          s_pack_voltage_dv, s_pack_voltage_dv * 0.1f);
        }
    }
};

void ble_server_init(void) {
    BLEDevice::init(BLE_DEVICE_NAME);
    s_server = BLEDevice::createServer();
    s_server->setCallbacks(new ServerCB());

    BLEService *svc = s_server->createService(SVC_UUID);

    s_ch_state = svc->createCharacteristic(CHAR_SAFETY_STATE_UUID,
                     BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    s_ch_temp = svc->createCharacteristic(CHAR_MAX_TEMP_UUID,
                    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    s_ch_cur = svc->createCharacteristic(CHAR_CHARGE_CUR_UUID,
                   BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    s_ch_voltage = svc->createCharacteristic(CHAR_PACK_VOLTAGE_UUID,
                       BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    s_ch_voltage->setCallbacks(new VoltageCB());
    uint8_t vbuf[2] = { (uint8_t)(s_pack_voltage_dv & 0xFF), (uint8_t)(s_pack_voltage_dv >> 8) };
    s_ch_voltage->setValue(vbuf, 2);

    // BMS pack: pack_mv(4) current_ma(4) soc_pct(1) charge_en(1) discharge_en(1) charger_plugged(1) = 12 bytes
    s_ch_bms_pack = svc->createCharacteristic(CHAR_BMS_PACK_UUID,
                        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    // BMS health: mos_temp_dc(2) bat_t1(2) bat_t2(2) soh_pct(1) cycle_count(4) full_mah(4) design_mah(4) run_s(4) = 23 bytes
    s_ch_bms_health = svc->createCharacteristic(CHAR_BMS_HEALTH_UUID,
                          BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    // BMS cells: 20x cell_mv(2 each) + avg_mv(2) + diff_mv(2) = 44 bytes
    s_ch_bms_cells = svc->createCharacteristic(CHAR_BMS_CELLS_UUID,
                         BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    // GPS: lat(4) lon(4) speed_kmh(4) course_deg(4) heading_deg(4) range_km(4) fix(1) sats(1) = 26 bytes
    s_ch_gps = svc->createCharacteristic(CHAR_GPS_UUID,
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
    if (now - s_last_notify_ms < BLE_NOTIFY_INTERVAL_MS) return;
    s_last_notify_ms = now;

    uint8_t state_byte = (uint8_t)safety_sm_get_state();
    s_ch_state->setValue(&state_byte, 1);
    if (s_connected) s_ch_state->notify();

    float max_t = thermal_get_max_ntc_temp();
    s_ch_temp->setValue((uint8_t *)&max_t, sizeof(float));
    if (s_connected) s_ch_temp->notify();

    uint16_t cur_da = thermal_get_max_current_da(max_t);
    uint8_t cbuf[2] = { (uint8_t)(cur_da & 0xFF), (uint8_t)(cur_da >> 8) };
    s_ch_cur->setValue(cbuf, 2);
    if (s_connected) s_ch_cur->notify();

    BMSData_t bms = {};
    rs485_get_last_bms(&bms);

    // BMS pack characteristic (12 bytes, big-endian)
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
        s_ch_bms_pack->setValue(b, sizeof(b));
        if (s_connected) s_ch_bms_pack->notify();
    }

    // BMS health characteristic (23 bytes, big-endian)
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
        s_ch_bms_health->setValue(b, sizeof(b));
        if (s_connected) s_ch_bms_health->notify();
    }

    // BMS cells characteristic (44 bytes, big-endian: 20 cells + avg + diff)
    {
        uint8_t b[44];
        uint8_t i = 0;
        for (uint8_t c = 0; c < 20; c++) {
            b[i++] = (bms.cell_mv[c] >> 8) & 0xFF;
            b[i++] =  bms.cell_mv[c]       & 0xFF;
        }
        b[i++] = (bms.cell_avg_mv  >> 8) & 0xFF; b[i++] = bms.cell_avg_mv  & 0xFF;
        b[i++] = (bms.cell_diff_mv >> 8) & 0xFF; b[i]   = bms.cell_diff_mv & 0xFF;
        s_ch_bms_cells->setValue(b, sizeof(b));
        if (s_connected) s_ch_bms_cells->notify();
    }

    // GPS characteristic (26 bytes, little-endian floats + 2 uint8)
    // Layout: lat(4) lon(4) speed_kmh(4) course_deg(4) heading_deg(4) range_km(4) fix(1) sats(1)
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
        s_ch_gps->setValue(b, sizeof(b));
        if (s_connected) s_ch_gps->notify();
    }
}

bool     ble_server_is_connected(void)       { return s_connected; }
uint16_t ble_server_get_pack_voltage_dv(void) { return s_pack_voltage_dv; }

#endif // MAIN_BRAIN
