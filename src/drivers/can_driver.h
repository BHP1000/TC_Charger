#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- CAN IDs (extended frame) ----
#define CAN_CHARGER_TX_ID       0x1806E5F4UL   // BMS → Charger
#define CAN_CHARGER_RX_ID       0x18FF50E5UL   // Charger → BMS
#define CAN_SPEED_KBPS          250

// ---- TX GPIO (adjust to match schematic) ----
#define CAN_TX_GPIO             4
#define CAN_RX_GPIO             5

// ---- TX frame constants (Byte 4 control field) ----
#define CAN_CTRL_CHARGE         0x00    // start / continue charging
#define CAN_CTRL_DISABLE        0x01    // close output immediately
#define CAN_CTRL_SLEEP          0x02    // charge end → charger sleeps

// ---- TX frame constants (Byte 5 mode field) ----
#define CAN_MODE_CHARGE         0x00
#define CAN_MODE_HEAT           0x01

// ---- TX send interval / timeout ----
#define CAN_TX_INTERVAL_MS      1000UL  // send heartbeat every 1 s
#define CAN_CHARGER_TIMEOUT_MS  5000UL  // charger disables if no TX for 5 s

// ---- Command frame (BMS → Charger) ----
typedef struct {
    uint16_t voltage_dv;    // target voltage  × 10  (0.1 V/bit)
    uint16_t current_da;    // target current  × 10  (0.1 A/bit)
    uint8_t  control;       // CAN_CTRL_* values
    uint8_t  mode;          // CAN_MODE_* values
} ChargerCmd_t;

// ---- Fault flags (Byte 4 of RX frame) ----
typedef struct {
    bool hardware_fault;        // BIT0
    bool temperature_fault;     // BIT1
    uint8_t input_voltage_status; // BIT2-3: 0=OK,1=under,2=over,3=none
    bool output_undervoltage;   // BIT4
    bool output_overvoltage;    // BIT5
    bool output_overcurrent;    // BIT6
    bool output_short_circuit;  // BIT7
} ChargerFaultFlags_t;

// ---- Status flags (Byte 5 of RX frame) ----
typedef struct {
    bool     comm_timeout;          // BIT0
    uint8_t  working_status;        // BIT1-2: 0=undef,1=work,2=stop,3=standby
    bool     init_complete;         // BIT3
    bool     fan_on;                // BIT4
    bool     cooling_pump_on;       // BIT5
} ChargerStatusFlags_t;

// ---- Full status from charger (Charger → BMS) ----
typedef struct {
    uint16_t           output_voltage_dv;   // 0.1 V/bit
    uint16_t           output_current_da;   // 0.1 A/bit
    ChargerFaultFlags_t fault;
    ChargerStatusFlags_t status;
    int8_t             temperature_c;       // Byte7 raw − 40
    bool               valid;               // frame received recently
} ChargerStatus_t;

void     can_driver_init(void);
bool     can_send_disable(void);
bool     can_send_sleep(void);
bool     can_send_command(const ChargerCmd_t *cmd);
bool     can_read_status(ChargerStatus_t *out);
void     can_driver_update(void);           // call every loop; handles TX heartbeat + RX polling

#ifdef __cplusplus
}
#endif
