#include "rs485_driver.h"
#include <Arduino.h>
#include "ntc_array.h"
#include "bme280_driver.h"
#ifdef TEMP_NODE
#include "jk_bms.h"
#endif

// ---- UART pin assignments ----
#define RS485_SERIAL    Serial1
#if defined(MAIN_BRAIN)
  #define RS485_TX_PIN  17    // ESP32-S3 GPIO17 → RS485 DI
  #define RS485_RX_PIN  18    // ESP32-S3 GPIO18 → RS485 RO
  #define RS485_DE_PIN  0xFF  // Auto-direction module — no DE pin
#elif defined(TEMP_NODE)
  #define RS485_TX_PIN   5    // ESP32-C3 Super Mini GPIO5  → RS485 DI
  #define RS485_RX_PIN  10    // ESP32-C3 Super Mini GPIO10 → RS485 RO
  #define RS485_DE_PIN   6    // ESP32-C3 Super Mini GPIO6  → RS485 DE+RE
#endif

static bool s_initialized = false;

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
    if (RS485_DE_PIN != 0xFF) {
        pinMode(RS485_DE_PIN, OUTPUT);
        digitalWrite(RS485_DE_PIN, LOW);
    }
    s_initialized = true;
    if (RS485_DE_PIN != 0xFF)
        Serial.printf("[RS485] init OK — baud=%d TX=GPIO%d RX=GPIO%d DE=GPIO%d\n",
                      RS485_BAUD, RS485_TX_PIN, RS485_RX_PIN, RS485_DE_PIN);
    else
        Serial.printf("[RS485] init OK — baud=%d TX=GPIO%d RX=GPIO%d DE=none\n",
                      RS485_BAUD, RS485_TX_PIN, RS485_RX_PIN);
}

bool rs485_send_packet(const RS485Packet_t *pkt) {
    if (!pkt || !s_initialized) return false;
    if (pkt->len > RS485_PKT_MAX_DATA) return false;

    uint8_t buf[RS485_PKT_OVERHEAD + RS485_PKT_MAX_DATA];
    buf[0] = RS485_PKT_SYNC;
    buf[1] = pkt->node_id;
    buf[2] = pkt->cmd;
    buf[3] = pkt->len;
    for (uint8_t i = 0; i < pkt->len; i++) buf[4 + i] = pkt->data[i];

    uint16_t crc = crc16(buf, 4 + pkt->len);
    buf[4 + pkt->len]     = (uint8_t)(crc & 0xFF);
    buf[4 + pkt->len + 1] = (uint8_t)(crc >> 8);

    uint8_t total = 4 + pkt->len + 2;
    if (RS485_DE_PIN != 0xFF) { digitalWrite(RS485_DE_PIN, HIGH); delayMicroseconds(100); }
    RS485_SERIAL.write(buf, total);
    RS485_SERIAL.flush();
    if (RS485_DE_PIN != 0xFF) { delayMicroseconds(100); digitalWrite(RS485_DE_PIN, LOW); }
    return true;
}

bool rs485_recv_packet(RS485Packet_t *pkt, uint32_t timeout_ms) {
    if (!pkt || !s_initialized) return false;

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

// ─── Main Brain (RS485 master) ────────────────────────────────────────────────
#ifdef MAIN_BRAIN

static TempNodeData_t s_last_temp_data;
static bool           s_temp_data_valid = false;
static BMSData_t      s_last_bms_data;
static bool           s_bms_data_valid  = false;
static uint32_t       s_poll_ms         = 0;
static uint32_t       s_bms_poll_ms     = 0;
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

        s_last_temp_data  = *out;
        s_temp_data_valid = true;
        return true;
    }
    s_temp_data_valid = false;
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

        s_last_bms_data  = *out;
        s_bms_data_valid = true;
        return true;
    }
    s_bms_data_valid = false;
    return false;
}

void rs485_poll(void) {
    uint32_t now = millis();

    if ((now - s_poll_ms) >= POLL_INTERVAL_MS) {
        s_poll_ms = now;
        TempNodeData_t td;
        if (rs485_request_temp_node(&td)) {
            Serial.printf("[RS485] TempNode T[0]=%.1fF T[1]=%.1fF T[2]=%.1fF "
                          "T[3]=%.1fF T[4]=%.1fF T[5]=%.1fF "
                          "Amb=%.1fF H=%.1f%% P=%.3finHg valid=0x%02X\n",
                          td.temp_c[0]*9.0f/5.0f+32.0f, td.temp_c[1]*9.0f/5.0f+32.0f,
                          td.temp_c[2]*9.0f/5.0f+32.0f, td.temp_c[3]*9.0f/5.0f+32.0f,
                          td.temp_c[4]*9.0f/5.0f+32.0f, td.temp_c[5]*9.0f/5.0f+32.0f,
                          td.ambient_c*9.0f/5.0f+32.0f, td.humidity_pct,
                          td.pressure_hpa*0.02953f, td.valid_mask);
        } else {
            Serial.println("[RS485] TempNode poll FAILED");
        }
    }

    if ((now - s_bms_poll_ms) >= BMS_POLL_INTERVAL_MS) {
        s_bms_poll_ms = now;
        BMSData_t bd;
        if (rs485_request_bms(&bd)) {
            Serial.printf("[RS485] BMS SOC=%u%% Pack=%.3fV I=%.3fA SOH=%u%% "
                          "Cyc=%lu MOS=%.1fF T1=%.1fF T2=%.1fF Alm=0x%08lX\n",
                          bd.soc_pct, bd.pack_mv * 0.001f, bd.current_ma * 0.001f,
                          bd.soh_pct, (unsigned long)bd.cycle_count,
                          bd.mos_temp_dc * 0.1f * 9.0f/5.0f + 32.0f,
                          bd.bat_temp1_dc * 0.1f * 9.0f/5.0f + 32.0f,
                          bd.bat_temp2_dc * 0.1f * 9.0f/5.0f + 32.0f,
                          (unsigned long)bd.alarm_flags);
        } else {
            Serial.println("[RS485] BMS poll FAILED");
        }
    }
}

bool rs485_get_last_temp_node(TempNodeData_t *out) {
    if (!out || !s_temp_data_valid) return false;
    *out = s_last_temp_data;
    return true;
}

bool rs485_get_last_bms(BMSData_t *out) {
    if (!out || !s_bms_data_valid) return false;
    *out = s_last_bms_data;
    return true;
}

#endif // MAIN_BRAIN

// ─── Temp Node (RS485 slave) ──────────────────────────────────────────────────
#ifdef TEMP_NODE
#include "jk_bms.h"

static void respond_temps(void) {
    NTCArray_t   ntc = {};
    BME280Data_t bme = {};
    ntc_array_get(&ntc);
    bme280_get(&bme);

    RS485Packet_t resp = {};
    resp.node_id = RS485_NODE_ID_TEMP;
    resp.cmd     = RS485_CMD_RESP_TEMPS;
    resp.len     = RS485_TEMPS_PAYLOAD_LEN;

    uint8_t i = 0;
    for (uint8_t ch = 0; ch < 6; ch++) {
        uint8_t *p = (uint8_t *)&ntc.temp_c[ch];
        resp.data[i++] = p[0]; resp.data[i++] = p[1];
        resp.data[i++] = p[2]; resp.data[i++] = p[3];
    }
    uint8_t *pa = (uint8_t *)&bme.temperature_c;
    resp.data[i++] = pa[0]; resp.data[i++] = pa[1];
    resp.data[i++] = pa[2]; resp.data[i++] = pa[3];

    uint8_t *ph = (uint8_t *)&bme.humidity_pct;
    resp.data[i++] = ph[0]; resp.data[i++] = ph[1];
    resp.data[i++] = ph[2]; resp.data[i++] = ph[3];

    uint8_t *pp = (uint8_t *)&bme.pressure_hpa;
    resp.data[i++] = pp[0]; resp.data[i++] = pp[1];
    resp.data[i++] = pp[2]; resp.data[i++] = pp[3];

    uint8_t mask = 0;
    for (uint8_t ch = 0; ch < 6; ch++) if (ntc.valid[ch]) mask |= (1 << ch);
    resp.data[i++] = mask;
    resp.data[i]   = bme.valid ? 1 : 0;

    rs485_send_packet(&resp);
}

static void respond_bms(void) {
    JKBMSData_t bms = {};
    jk_bms_get(&bms);

    RS485Packet_t resp = {};
    resp.node_id = RS485_NODE_ID_TEMP;
    resp.cmd     = RS485_CMD_RESP_BMS;
    resp.len     = RS485_BMS_PAYLOAD_LEN;

    uint8_t *d = resp.data;
    uint8_t i = 0;

    for (uint8_t c = 0; c < 20; c++) {
        d[i++] = (bms.cell_mv[c] >> 8) & 0xFF;
        d[i++] =  bms.cell_mv[c]       & 0xFF;
    }
    d[i++] = (bms.cell_avg_mv >> 8) & 0xFF;
    d[i++] =  bms.cell_avg_mv       & 0xFF;
    d[i++] = (bms.cell_diff_mv >> 8) & 0xFF;
    d[i++] =  bms.cell_diff_mv       & 0xFF;
    d[i++] = ((uint16_t)bms.mos_temp_dc >> 8) & 0xFF;
    d[i++] =  (uint16_t)bms.mos_temp_dc       & 0xFF;
    d[i++] = (bms.pack_mv >> 24) & 0xFF;
    d[i++] = (bms.pack_mv >> 16) & 0xFF;
    d[i++] = (bms.pack_mv >>  8) & 0xFF;
    d[i++] =  bms.pack_mv        & 0xFF;
    d[i++] = ((uint32_t)bms.current_ma >> 24) & 0xFF;
    d[i++] = ((uint32_t)bms.current_ma >> 16) & 0xFF;
    d[i++] = ((uint32_t)bms.current_ma >>  8) & 0xFF;
    d[i++] =  (uint32_t)bms.current_ma        & 0xFF;
    d[i++] = ((uint16_t)bms.bat_temp1_dc >> 8) & 0xFF;
    d[i++] =  (uint16_t)bms.bat_temp1_dc       & 0xFF;
    d[i++] = ((uint16_t)bms.bat_temp2_dc >> 8) & 0xFF;
    d[i++] =  (uint16_t)bms.bat_temp2_dc       & 0xFF;
    d[i++] = (bms.alarm_flags >> 24) & 0xFF;
    d[i++] = (bms.alarm_flags >> 16) & 0xFF;
    d[i++] = (bms.alarm_flags >>  8) & 0xFF;
    d[i++] =  bms.alarm_flags        & 0xFF;
    d[i++] = ((uint16_t)bms.balance_ma >> 8) & 0xFF;
    d[i++] =  (uint16_t)bms.balance_ma       & 0xFF;
    d[i++] = bms.balance_status;
    d[i++] = bms.soc_pct;
    d[i++] = ((uint32_t)bms.remaining_mah >> 24) & 0xFF;
    d[i++] = ((uint32_t)bms.remaining_mah >> 16) & 0xFF;
    d[i++] = ((uint32_t)bms.remaining_mah >>  8) & 0xFF;
    d[i++] =  (uint32_t)bms.remaining_mah        & 0xFF;
    d[i++] = (bms.full_charge_mah >> 24) & 0xFF;
    d[i++] = (bms.full_charge_mah >> 16) & 0xFF;
    d[i++] = (bms.full_charge_mah >>  8) & 0xFF;
    d[i++] =  bms.full_charge_mah        & 0xFF;
    d[i++] = (bms.cycle_count >> 24) & 0xFF;
    d[i++] = (bms.cycle_count >> 16) & 0xFF;
    d[i++] = (bms.cycle_count >>  8) & 0xFF;
    d[i++] =  bms.cycle_count        & 0xFF;
    d[i++] = bms.soh_pct;
    d[i++] = (bms.run_time_s >> 24) & 0xFF;
    d[i++] = (bms.run_time_s >> 16) & 0xFF;
    d[i++] = (bms.run_time_s >>  8) & 0xFF;
    d[i++] =  bms.run_time_s        & 0xFF;
    d[i++] = bms.charge_en;
    d[i++] = bms.discharge_en;
    d[i++] = bms.charger_plugged;
    d[i++] = (bms.design_cap_mah >> 24) & 0xFF;
    d[i++] = (bms.design_cap_mah >> 16) & 0xFF;
    d[i++] = (bms.design_cap_mah >>  8) & 0xFF;
    d[i++] =  bms.design_cap_mah        & 0xFF;
    d[i]   = bms.valid ? 1 : 0;

    rs485_send_packet(&resp);
}

void rs485_respond(void) {
    if (!s_initialized) return;
    if (!RS485_SERIAL.available()) return;

    RS485Packet_t req = {};
    if (!rs485_recv_packet(&req, 20)) return;
    if (req.node_id != RS485_NODE_ID_TEMP) return;

    if (req.cmd == RS485_CMD_GET_TEMPS) {
        respond_temps();
    } else if (req.cmd == RS485_CMD_GET_BMS) {
        respond_bms();
    }
}

#endif // TEMP_NODE
