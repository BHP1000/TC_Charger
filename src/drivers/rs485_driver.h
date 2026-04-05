#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ----- Bus config -----
#define RS485_BAUD           115200
#define RS485_TIMEOUT_MS     50
#define RS485_RETRY_COUNT    3

// ----- Node IDs -----
#define RS485_NODE_ID_BRAIN  0x01
#define RS485_NODE_ID_TEMP   0x02
#define RS485_NODE_ID_BMS    0x03

// ----- Commands -----
#define RS485_CMD_GET_TEMPS  0x10
#define RS485_CMD_RESP_TEMPS 0x11
#define RS485_CMD_GET_BMS    0x20
#define RS485_CMD_RESP_BMS   0x21
#define RS485_CMD_ACK        0x7E
#define RS485_CMD_NACK       0x7F

// ----- Packet -----
// Wire format: [0xAA][node_id][cmd][len][data...][crc_lo][crc_hi]
#define RS485_PKT_SYNC       0xAA
#define RS485_PKT_MAX_DATA   128
#define RS485_PKT_OVERHEAD   5   // sync + node + cmd + len + 2 crc

typedef struct {
    uint8_t  node_id;
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  data[RS485_PKT_MAX_DATA];
    uint16_t crc;
} RS485Packet_t;

// ----- Temp node payload (cmd 0x11) -----
// [0..23]  6x float temp_c       (4 bytes each)
// [24..27] float ambient_c
// [28..31] float humidity_pct
// [32..35] float pressure_hpa
// [36]     uint8 valid_mask
// [37]     uint8 bme_valid
#define RS485_TEMPS_PAYLOAD_LEN 38

typedef struct {
    float   temp_c[6];
    float   ambient_c;
    float   humidity_pct;
    float   pressure_hpa;
    uint8_t valid_mask;
    bool    bme_valid;
} TempNodeData_t;

// ----- BMS payload (cmd 0x21) -----
// [0..39]  20x uint16 cell_mv        big-endian
// [40..41] uint16 cell_avg_mv
// [42..43] uint16 cell_diff_mv
// [44..45] int16  mos_temp_dc        0.1°C
// [46..49] uint32 pack_mv            mV
// [50..53] int32  current_ma         mA  pos=charge neg=discharge
// [54..55] int16  bat_temp1_dc       0.1°C
// [56..57] int16  bat_temp2_dc       0.1°C
// [58..61] uint32 alarm_flags
// [62..63] int16  balance_ma
// [64]     uint8  balance_status     0=off 1=chg 2=dchg
// [65]     uint8  soc_pct
// [66..69] int32  remaining_mah
// [70..73] uint32 full_charge_mah
// [74..77] uint32 cycle_count
// [78]     uint8  soh_pct
// [79..82] uint32 run_time_s
// [83]     uint8  charge_en
// [84]     uint8  discharge_en
// [85]     uint8  charger_plugged
// [86..89] uint32 design_cap_mah
// [90]     uint8  valid
#define RS485_BMS_PAYLOAD_LEN 91

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

void rs485_init(void);
bool rs485_send_packet(const RS485Packet_t *pkt);
bool rs485_recv_packet(RS485Packet_t *pkt, uint32_t timeout_ms);

void rs485_poll(void);
bool rs485_request_temp_node(TempNodeData_t *out);
bool rs485_get_last_temp_node(TempNodeData_t *out);
bool rs485_request_bms(BMSData_t *out);
bool rs485_get_last_bms(BMSData_t *out);

void rs485_respond(void);

#ifdef __cplusplus
}
#endif
