#ifdef TEMP_NODE

#include "jk_bms.h"
#include <Arduino.h>
#include <string.h>

#define JK_SERIAL       Serial1
#define JK_DE_PIN       6
#define JK_ADDR         0x01
#define JK_POLL_MS      5000
#define JK_TIMEOUT_MS   120

static JKBMSData_t s_data;
static bool        s_valid     = false;
static uint32_t    s_last_poll = 0;

static uint16_t modbus_crc(const uint8_t *buf, uint8_t len) {
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

static uint8_t modbus_read_regs(uint16_t reg, uint16_t count, uint8_t *out) {
    uint8_t req[8];
    req[0] = JK_ADDR;
    req[1] = 0x03;
    req[2] = (uint8_t)(reg >> 8);
    req[3] = (uint8_t)(reg & 0xFF);
    req[4] = (uint8_t)(count >> 8);
    req[5] = (uint8_t)(count & 0xFF);
    uint16_t crc = modbus_crc(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    while (JK_SERIAL.available()) JK_SERIAL.read();

    digitalWrite(JK_DE_PIN, HIGH);
    delayMicroseconds(200);
    JK_SERIAL.write(req, 8);
    JK_SERIAL.flush();
    delayMicroseconds(200);
    digitalWrite(JK_DE_PIN, LOW);

    uint8_t expect_data = (uint8_t)(count * 2);
    uint8_t total       = 3 + expect_data + 2;
    uint8_t rbuf[260];
    if (total > sizeof(rbuf)) return 0;

    uint8_t  idx   = 0;
    uint32_t start = millis();
    while (idx < total) {
        if ((millis() - start) >= JK_TIMEOUT_MS) return 0;
        if (JK_SERIAL.available()) rbuf[idx++] = (uint8_t)JK_SERIAL.read();
    }

    uint16_t rx_crc = (uint16_t)rbuf[total - 2] | ((uint16_t)rbuf[total - 1] << 8);
    if (modbus_crc(rbuf, total - 2) != rx_crc)           return 0;
    if (rbuf[0] != JK_ADDR || rbuf[1] != 0x03)           return 0;
    if (rbuf[2] != expect_data)                           return 0;

    memcpy(out, &rbuf[3], expect_data);
    return expect_data;
}

static inline uint16_t u16be(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}
static inline int16_t i16be(const uint8_t *p) {
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static inline uint32_t u32be(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) |  (uint32_t)p[3];
}
static inline int32_t i32be(const uint8_t *p) {
    return (int32_t)u32be(p);
}

void jk_bms_init(void) {
    memset(&s_data, 0, sizeof(s_data));
    s_data.design_cap_mah = JK_BMS_DESIGN_MAH;
    s_last_poll = 0 - JK_POLL_MS;
    Serial.println("[JK-BMS] init OK — addr=0x01 cells=20 poll=5s");
}

void jk_bms_update(void) {
    uint32_t now = millis();
    if ((now - s_last_poll) < JK_POLL_MS) return;
    s_last_poll = now;

    uint8_t    buf[260];
    JKBMSData_t d = {};
    d.design_cap_mah = JK_BMS_DESIGN_MAH;

    // --- Read 1: cell voltages 0x1200, count=20 ---
    if (!modbus_read_regs(0x1200, 20, buf)) {
        Serial.println("[JK-BMS] cell voltage read FAILED");
        s_valid = false;
        return;
    }
    for (uint8_t i = 0; i < JK_BMS_CELLS; i++)
        d.cell_mv[i] = u16be(buf + i * 2);

    // --- Read 2: avg + diff at 0x1222, count=2 ---
    if (!modbus_read_regs(0x1222, 2, buf)) {
        Serial.println("[JK-BMS] avg/diff read FAILED");
        s_valid = false;
        return;
    }
    d.cell_avg_mv  = u16be(buf + 0);
    d.cell_diff_mv = u16be(buf + 2);

    // --- Read 3: status block 0x1245, count=28 ---
    // Reg offsets in response buffer (each reg = 2 bytes):
    // 0x1245 [0..1]  TempMos       INT16  0.1°C
    // 0x1246 [2..5]  WireResSta    UINT32 (skip)
    // 0x1248 [6..9]  BatVol        UINT32 mV
    // 0x124A [10..13] BatWatt      UINT32 mW (skip)
    // 0x124C [14..17] BatCurrent   INT32  mA
    // 0x124E [18..19] TempBat1     INT16  0.1°C
    // 0x124F [20..21] TempBat2     INT16  0.1°C
    // 0x1250 [22..25] AlarmFlags   UINT32
    // 0x1252 [26..27] BalanCurrent INT16  mA
    // 0x1253 [28..29] BalanSta|SOC UINT8|UINT8
    // 0x1254 [30..33] CapRemain    INT32  mAh
    // 0x1256 [34..37] FullChargeCap UINT32 mAh
    // 0x1258 [38..41] CycleCount   UINT32
    // 0x125A [42..45] CycleCap     UINT32 (skip)
    // 0x125C [46..47] SOH|Precharge UINT8|UINT8
    // 0x125D [48..49] UserAlarm    UINT16 (skip)
    // 0x125E [50..53] RunTime      UINT32 s
    // 0x1260 [54..55] Charge|Discharge UINT8|UINT8
    if (!modbus_read_regs(0x1245, 28, buf)) {
        Serial.println("[JK-BMS] status block read FAILED");
        s_valid = false;
        return;
    }
    d.mos_temp_dc     = i16be(buf +  0);
    d.pack_mv         = u32be(buf +  6);
    d.current_ma      = i32be(buf + 14);
    d.bat_temp1_dc    = i16be(buf + 18);
    d.bat_temp2_dc    = i16be(buf + 20);
    d.alarm_flags     = u32be(buf + 22);
    d.balance_ma      = i16be(buf + 26);
    d.balance_status  = buf[28];
    d.soc_pct         = buf[29];
    d.remaining_mah   = i32be(buf + 30);
    d.full_charge_mah = u32be(buf + 34);
    d.cycle_count     = u32be(buf + 38);
    d.soh_pct         = buf[46];
    d.run_time_s      = u32be(buf + 50);
    d.charge_en       = buf[54];
    d.discharge_en    = buf[55];

    // --- Read 4: charger plugged at 0x1277, count=1 ---
    if (modbus_read_regs(0x1277, 1, buf))
        d.charger_plugged = buf[1];

    d.valid = true;
    s_data  = d;
    s_valid = true;

    Serial.printf("[JK-BMS] SOC=%u%% Pack=%.3fV I=%.3fA SOH=%u%% Cyc=%lu Alm=0x%08lX\n",
                  d.soc_pct,
                  d.pack_mv   * 0.001f,
                  d.current_ma * 0.001f,
                  d.soh_pct,
                  (unsigned long)d.cycle_count,
                  (unsigned long)d.alarm_flags);
}

bool jk_bms_get(JKBMSData_t *out) {
    if (!out || !s_valid) return false;
    *out = s_data;
    return true;
}

#endif // TEMP_NODE
