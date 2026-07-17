#ifndef FAULT_H
#define FAULT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "measure.h"

typedef enum
{
    MCAPP_FAULT_NONE = 0,
    MCAPP_OVERCURRENT_FAULT_PHASE = 1,
    MCAPP_OVERCURRENT_FAULT_DCBUS = 2,
} MCAPP_FAULT_STATE_T;

typedef struct
{
    float overCurrentFaultLimit;
    MCAPP_FAULT_STATE_T faultState;
} MCAPP_FAULT_T;

bool MCAPP_OverCurrentFault_Detect(MCAPP_MEASURE_T *, MCAPP_FAULT_T *);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_H */
