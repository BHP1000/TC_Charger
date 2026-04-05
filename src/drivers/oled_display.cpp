#ifdef MAIN_BRAIN

#include "oled_display.h"
#include "safety_state_machine.h"
#include "ble_server.h"
#include "can_driver.h"
#include "thermal_logic.h"
#include "bme280_driver.h"
#include "rs485_driver.h"
#include "gps_driver.h"
#include "button_driver.h"

#include <U8g2lib.h>
#include <Wire.h>

// Full-framebuffer SH1106 128x64, hardware I2C
// Pass actual SCL/SDA pins so U8G2 calls Wire.begin() itself — avoids double-init
// that causes ESP_ERR_INVALID_STATE on the new ESP32-S3 I2C driver (ESP-IDF v5).
static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0,
    U8X8_PIN_NONE,
    OLED_SCL_PIN,
    OLED_SDA_PIN
);

static uint32_t s_last_update  = 0;
static uint32_t s_last_page_ms = 0;
static uint8_t  s_page         = 0;
static bool     s_held         = false;

#define OLED_UPDATE_MS   500
#define OLED_PAGE_MS    8000
#define OLED_NUM_PAGES   7

// ---- helpers ----------------------------------------------------------------

static const char *state_label(SafetyState_t s) {
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

// Draw the top status bar (inverted rectangle + state label centred inside)
// and a subtitle bar with page title on left and status indicators on right
static void draw_header(SafetyState_t state, const char *page_title) {
    const char *label = state_label(state);

    u8g2.setFont(u8g2_font_7x13B_tr);
    int16_t  lw    = u8g2.getStrWidth(label);
    int16_t  x     = (128 - lw) / 2;

    // filled bar 0..14
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 15);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x, 12, label);
    u8g2.setDrawColor(1);

    // subtitle bar 15..23: page title left, indicators right
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 23, page_title);
    uint8_t ind_x = 128;
    if (state == SAFETY_STATE_CHARGING) {
        ind_x -= 30;
        u8g2.drawStr(ind_x, 23, "[CHG]");
    }
    if (s_held) {
        ind_x -= 30;
        u8g2.drawStr(ind_x, 23, "[HLD]");
    }
    u8g2.drawHLine(0, 24, 128);
}

// Page 0 — charger data
static void draw_data(void) {
    u8g2.setFont(u8g2_font_6x10_tf);

    // --- row 1: pack voltage (setpoint from BLE) ---
    char buf[32];
    uint16_t vdv = ble_server_get_pack_voltage_dv();
    snprintf(buf, sizeof(buf), "Pack: %3u.%uV", vdv / 10, vdv % 10);
    u8g2.drawStr(0, 27, buf);

    // --- row 2: charger output current (actual, from CAN RX) ---
    ChargerStatus_t cs;
    if (can_read_status(&cs)) {
        snprintf(buf, sizeof(buf), "Out:  %3u.%uA",
                 cs.output_current_da / 10, cs.output_current_da % 10);
    } else {
        snprintf(buf, sizeof(buf), "Out:  --.-A");
    }
    u8g2.drawStr(0, 39, buf);

    // --- row 3: max NTC temperature ---
    float max_t = thermal_get_max_ntc_temp();
    if (max_t > -100.0f) {
        float max_f = max_t * 9.0f / 5.0f + 32.0f;
        int16_t t_int  = (int16_t)max_f;
        uint8_t t_frac = (uint8_t)((max_f - t_int < 0 ? -(max_f - t_int) : (max_f - t_int)) * 10);
        snprintf(buf, sizeof(buf), "Tmax: %3d.%uF", t_int, t_frac);
    } else {
        snprintf(buf, sizeof(buf), "Tmax: ---.-F");
    }
    u8g2.drawStr(0, 51, buf);

    // --- row 4: BLE status + charger fault indicator ---
    const char *ble_str = ble_server_is_connected() ? "BLE:ON " : "BLE:-- ";
    const char *chg_str = "CHG:OK";
    ChargerStatus_t cs2;
    if (can_read_status(&cs2)) {
        if      (cs2.fault.hardware_fault)      chg_str = "CHG:HW";
        else if (cs2.fault.temperature_fault)   chg_str = "CHG:OTP";
        else if (cs2.fault.output_overvoltage)  chg_str = "CHG:OVP";
        else if (cs2.fault.output_overcurrent)  chg_str = "CHG:OCP";
    }
    snprintf(buf, sizeof(buf), "%s %s", ble_str, chg_str);
    u8g2.drawStr(0, 63, buf);
}

// Page 1 — env sensors: main board (MB) + daughter board (DN)
static void draw_env(void) {
    u8g2.setFont(u8g2_font_6x10_tf);

    BME280Data_t mb;
    bool mb_ok = bme280_get(&mb);

    TempNodeData_t dn;
    bool dn_ok = rs485_get_last_temp_node(&dn);

    char buf[32];

    // --- MB row 1: temp + humidity ---
    if (mb_ok && mb.valid) {
        float tf = mb.temperature_c * 9.0f / 5.0f + 32.0f;
        snprintf(buf, sizeof(buf), "MB:%5.1fF %4.1f%%", tf, mb.humidity_pct);
    } else {
        snprintf(buf, sizeof(buf), "MB: ---.-F  --.-%%");
    }
    u8g2.drawStr(0, 25, buf);

    // --- MB row 2: pressure inHg ---
    if (mb_ok && mb.valid && mb.pressure_hpa > 0.0f)
        snprintf(buf, sizeof(buf), "   %6.3f inHg", mb.pressure_hpa * 0.02953f);
    else
        snprintf(buf, sizeof(buf), "   --.--- inHg");
    u8g2.drawStr(0, 34, buf);

    // --- DN row 1: temp + humidity ---
    if (dn_ok && dn.bme_valid) {
        float tf = dn.ambient_c * 9.0f / 5.0f + 32.0f;
        snprintf(buf, sizeof(buf), "DN:%5.1fF %4.1f%%", tf, dn.humidity_pct);
    } else {
        snprintf(buf, sizeof(buf), "DN: ---.-F  --.-%%");
    }
    u8g2.drawStr(0, 46, buf);

    // --- DN row 2: pressure inHg ---
    if (dn_ok && dn.bme_valid && dn.pressure_hpa > 0.0f)
        snprintf(buf, sizeof(buf), "   %6.3f inHg", dn.pressure_hpa * 0.02953f);
    else
        snprintf(buf, sizeof(buf), "   --.--- inHg");
    u8g2.drawStr(0, 55, buf);

    u8g2.drawStr(0, 63, "MB=MainBrd DN=Daughter");
}

// Page 2 — NTC sensor temps T1–T5
static void draw_ntc(void) {
    u8g2.setFont(u8g2_font_6x10_tf);

    TempNodeData_t dn;
    bool ok = rs485_get_last_temp_node(&dn);

    char buf[24];
    const uint8_t y_start = 25;
    const uint8_t y_step  = 9;

    for (uint8_t i = 1; i <= 5; i++) {
        bool valid = ok && (dn.valid_mask & (1 << i));
        if (valid) {
            float tf = dn.temp_c[i] * 9.0f / 5.0f + 32.0f;
            snprintf(buf, sizeof(buf), "T%u: %6.1f F", i, tf);
        } else {
            snprintf(buf, sizeof(buf), "T%u:  ---.- F", i);
        }
        u8g2.drawStr(0, y_start + (i - 1) * y_step, buf);
    }
}

// ---- helpers for BMS --------------------------------------------------------

static float dc_to_f(int16_t dc) {
    return (dc / 10.0f) * 9.0f / 5.0f + 32.0f;
}

// Page 3 — BMS pack overview
static void draw_bms_pack(void) {
    u8g2.setFont(u8g2_font_6x10_tf);

    BMSData_t bms = {};
    bool ok = rs485_get_last_bms(&bms);

    char buf[28];

    // row 1: pack voltage + SOC
    if (ok && bms.valid) {
        uint32_t v_dv = bms.pack_mv / 100;
        snprintf(buf, sizeof(buf), "%3lu.%luV  SOC:%3u%%",
                 (unsigned long)(v_dv / 10), (unsigned long)(v_dv % 10),
                 bms.soc_pct);
    } else {
        snprintf(buf, sizeof(buf), "---.--V  SOC:--%%");
    }
    u8g2.drawStr(0, 25, buf);

    // row 2: current  (pos=charging, neg=discharging)
    if (ok && bms.valid) {
        int32_t a_da = bms.current_ma / 100;
        char sign = (a_da >= 0) ? '+' : '-';
        if (a_da < 0) a_da = -a_da;
        snprintf(buf, sizeof(buf), "I:%c%3ld.%ldA",
                 sign, (long)(a_da / 10), (long)(a_da % 10));
    } else {
        snprintf(buf, sizeof(buf), "I:  --.-A");
    }
    u8g2.drawStr(0, 35, buf);

    // row 3: remaining Ah / full Ah
    if (ok && bms.valid) {
        uint32_t rem_dah  = (uint32_t)((bms.remaining_mah  < 0 ? 0 : bms.remaining_mah)  / 100);
        uint32_t full_dah = bms.full_charge_mah / 100;
        snprintf(buf, sizeof(buf), "%2lu.%luAh / %2lu.%luAh",
                 (unsigned long)(rem_dah / 10), (unsigned long)(rem_dah % 10),
                 (unsigned long)(full_dah / 10), (unsigned long)(full_dah % 10));
    } else {
        snprintf(buf, sizeof(buf), "--.-Ah / --.-Ah");
    }
    u8g2.drawStr(0, 45, buf);

    // row 4: CHG/DCHG enabled + charger plugged
    if (ok && bms.valid) {
        snprintf(buf, sizeof(buf), "C:%s D:%s %s",
                 bms.charge_en    ? "ON " : "OFF",
                 bms.discharge_en ? "ON " : "OFF",
                 bms.charger_plugged ? "PLUG" : "    ");
    } else {
        snprintf(buf, sizeof(buf), "C:--- D:---");
    }
    u8g2.drawStr(0, 55, buf);

    u8g2.drawStr(0, 63, "BMS Pack");
}

// Page 4 — BMS temps + health
static void draw_bms_health(void) {
    u8g2.setFont(u8g2_font_6x10_tf);

    BMSData_t bms = {};
    bool ok = rs485_get_last_bms(&bms);

    char buf[28];

    // row 1: MOS temp + bat T1
    if (ok && bms.valid) {
        snprintf(buf, sizeof(buf), "MOS:%5.1fF T1:%5.1fF",
                 dc_to_f(bms.mos_temp_dc), dc_to_f(bms.bat_temp1_dc));
    } else {
        snprintf(buf, sizeof(buf), "MOS:--.-F T1:--.-F");
    }
    u8g2.drawStr(0, 25, buf);

    // row 2: bat T2 + SOH
    if (ok && bms.valid) {
        snprintf(buf, sizeof(buf), "T2:%5.1fF  SOH:%3u%%",
                 dc_to_f(bms.bat_temp2_dc), bms.soh_pct);
    } else {
        snprintf(buf, sizeof(buf), "T2:--.-F  SOH:--%%");
    }
    u8g2.drawStr(0, 35, buf);

    // row 3: cycle count + design cap
    if (ok && bms.valid) {
        uint32_t cap_dah = bms.design_cap_mah / 100;
        snprintf(buf, sizeof(buf), "Cyc:%4lu Cap:%2lu.%luAh",
                 (unsigned long)bms.cycle_count,
                 (unsigned long)(cap_dah / 10), (unsigned long)(cap_dah % 10));
    } else {
        snprintf(buf, sizeof(buf), "Cyc:---- Cap:--.-Ah");
    }
    u8g2.drawStr(0, 45, buf);

    // row 4: run time h:m
    if (ok && bms.valid) {
        uint32_t h = bms.run_time_s / 3600;
        uint32_t m = (bms.run_time_s % 3600) / 60;
        snprintf(buf, sizeof(buf), "Run:%4luh %02lum",
                 (unsigned long)h, (unsigned long)m);
    } else {
        snprintf(buf, sizeof(buf), "Run: ----h --m");
    }
    u8g2.drawStr(0, 55, buf);

    u8g2.drawStr(0, 63, "BMS Health");
}

// Page 5 — Cell voltages (min / max / avg + compact 4x5 grid)
static void draw_bms_cells(void) {
    BMSData_t bms = {};
    bool ok = rs485_get_last_bms(&bms);

    char buf[28];

    // row 1 (small font): min / max / avg / delta
    u8g2.setFont(u8g2_font_5x7_tf);
    if (ok && bms.valid) {
        uint16_t mn = 0xFFFF, mx = 0;
        for (uint8_t c = 0; c < 20; c++) {
            if (bms.cell_mv[c] < mn) mn = bms.cell_mv[c];
            if (bms.cell_mv[c] > mx) mx = bms.cell_mv[c];
        }
        snprintf(buf, sizeof(buf), "Mn:%4umV Mx:%4umV", mn, mx);
        u8g2.drawStr(0, 22, buf);
        snprintf(buf, sizeof(buf), "Av:%4umV Df:%4umV",
                 bms.cell_avg_mv, bms.cell_diff_mv);
        u8g2.drawStr(0, 29, buf);
    } else {
        u8g2.drawStr(0, 22, "Mn:----  Mx:----");
        u8g2.drawStr(0, 29, "Av:----  Df:----");
    }

    // compact 4-column × 5-row grid (cells 1-20)
    // each cell: "XX.XX" → 5 chars × 6px = 30px; 4 cols = 120px, fits in 128
    u8g2.setFont(u8g2_font_5x7_tf);
    for (uint8_t row = 0; row < 5; row++) {
        for (uint8_t col = 0; col < 4; col++) {
            uint8_t idx = row * 4 + col;
            uint8_t x   = col * 32;
            uint8_t y   = 37 + row * 7;
            if (ok && bms.valid) {
                uint16_t mv = bms.cell_mv[idx];
                snprintf(buf, sizeof(buf), "%u.%03u", mv / 1000, mv % 1000);
            } else {
                snprintf(buf, sizeof(buf), "-.---");
            }
            u8g2.drawStr(x, y, buf);
        }
    }
}

// Page 6 — GPS location, speed, range, compass heading
static void draw_gps(void) {
    u8g2.setFont(u8g2_font_5x7_tf);

    GPSData_t gps = {};
    bool ok = gps_driver_get(&gps);

    char buf[32];

    // row 1: lat / lon
    if (ok) {
        snprintf(buf, sizeof(buf), "%.5f", gps.lat);
        u8g2.drawStr(0, 27, buf);
        snprintf(buf, sizeof(buf), "%.5f", gps.lon);
        u8g2.drawStr(64, 27, buf);
    } else {
        u8g2.drawStr(0, 27, "Lat: ---.-----");
        u8g2.drawStr(0, 34, "Lon: ---.-----");
    }

    // row 2: speed + GPS course
    if (ok) {
        snprintf(buf, sizeof(buf), "Spd:%5.1fkm %3.0fdeg",
                 gps.speed_kmh, gps.course_deg);
    } else {
        snprintf(buf, sizeof(buf), "Spd: --.-km ---deg");
    }
    u8g2.drawStr(0, 37, buf);

    // row 3: estimated range
    if (ok && gps.range_km >= 0.0f) {
        snprintf(buf, sizeof(buf), "Rng:%5.1f km", gps.range_km);
    } else {
        snprintf(buf, sizeof(buf), "Rng:  --.- km");
    }
    u8g2.drawStr(0, 47, buf);

    // row 4: compass heading + sats
    if (gps.compass_valid) {
        snprintf(buf, sizeof(buf), "Hdg:%5.1fdeg", gps.heading_deg);
    } else {
        snprintf(buf, sizeof(buf), "Hdg:  --.-deg");
    }
    snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
             " Sat:%2u", gps.satellites);
    u8g2.drawStr(0, 57, buf);

    // fix indicator
    u8g2.drawStr(0, 63, ok ? "GPS:FIX" : "GPS:NO FIX");
}

// ---- public -----------------------------------------------------------------

void oled_display_init(void) {
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(14, 36, "ScooterCharger");
    u8g2.sendBuffer();
    Serial.printf("[OLED] init OK — SH1106 SDA=GPIO%d SCL=GPIO%d addr=0x%02X\n",
                  OLED_SDA_PIN, OLED_SCL_PIN, OLED_I2C_ADDR);
}

void oled_display_update(void) {
    uint32_t now = millis();
    if (now - s_last_update < OLED_UPDATE_MS) return;
    s_last_update = now;

    // handle buttons
    s_held = button_hold_active();
    if (button_next_pressed()) {
        s_page         = (s_page + 1) % OLED_NUM_PAGES;
        s_last_page_ms = now;
    }

    // auto-advance only when not held
    if (!s_held && (now - s_last_page_ms) >= OLED_PAGE_MS) {
        s_last_page_ms = now;
        s_page = (s_page + 1) % OLED_NUM_PAGES;
    }

    SafetyState_t state = safety_sm_get_state();

    static const char *page_titles[OLED_NUM_PAGES] = {
        "Charger",
        "Environment",
        "NTC Temps",
        "BMS Pack",
        "BMS Health",
        "Cell Voltages",
        "GPS / Range",
    };

    u8g2.clearBuffer();
    draw_header(state, page_titles[s_page]);
    if      (s_page == 0) draw_data();
    else if (s_page == 1) draw_env();
    else if (s_page == 2) draw_ntc();
    else if (s_page == 3) draw_bms_pack();
    else if (s_page == 4) draw_bms_health();
    else if (s_page == 5) draw_bms_cells();
    else                  draw_gps();
    u8g2.sendBuffer();
}

void oled_display_next_page(void) {
    s_page = (s_page + 1) % OLED_NUM_PAGES;
    s_last_page_ms = millis();
}

#endif // MAIN_BRAIN
