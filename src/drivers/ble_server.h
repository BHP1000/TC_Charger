#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "safety_state_machine.h"

#ifdef __cplusplus
extern "C" {
#endif

void     ble_server_init(void);
void     ble_server_update(void);
bool     ble_server_is_connected(void);
uint16_t ble_server_get_pack_voltage_dv(void);

#ifdef __cplusplus
}
#endif
