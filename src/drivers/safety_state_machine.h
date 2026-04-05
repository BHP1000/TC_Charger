#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SAFETY_STATE_INIT = 0,
    SAFETY_STATE_IDLE,
    SAFETY_STATE_CHARGING,
    SAFETY_STATE_OVERTEMP,
    SAFETY_STATE_UNDERTEMP,
    SAFETY_STATE_BMS_FAULT,
    SAFETY_STATE_COMM_TIMEOUT,
    SAFETY_STATE_FAULT,
} SafetyState_t;

const char *safety_state_name(SafetyState_t s);
void        safety_sm_init(void);
void        safety_sm_update(void);
SafetyState_t safety_sm_get_state(void);
void        safety_sm_report_bms_fault(bool fault);
void        safety_sm_report_comm_timeout(bool timeout);

#ifdef __cplusplus
}
#endif
