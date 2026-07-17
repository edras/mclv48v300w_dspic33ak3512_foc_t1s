#ifndef __FDWEAK_H
#define __FDWEAK_H

#include <stdint.h>
#include <stdbool.h>

#include "foc_control_types.h"
#include "../motor_types.h"
#include "clarke_park.h"
#include "pi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float
        IdRef,
        IdRefFilt,
        IdRefFiltConst,
        IdRefMin,
        voltageLimitFW,
        fwNum,
        fwDen,
        fwEnableSpeed;

    const MCAPP_CONTROL_T *pCtrlParam;
    const MCAPP_MOTOR_T *pMotor;
} MCAPP_FLUX_WEAKENING_FF_T;

typedef struct
{
    float
        IdRef,
        IdRefFilt,
        IdRefFiltConst,
        IdRefMin,
        voltageMag,
        voltageMagRef,
        fwEnableSpeed;

    MC_PI_T FWeakPI;

    MCAPP_CONTROL_T *pCtrlParam;
    const MC_DQ_T *pVdq;
    const MCAPP_MOTOR_T *pMotor;
} MCAPP_FLUX_WEAKENING_VOLT_FB_T;

typedef struct
{
    float idRef,
          ImaxSquare,
          iqMax;

    uint8_t variant;

    MCAPP_FLUX_WEAKENING_VOLT_FB_T feedBackFW;
    MCAPP_FLUX_WEAKENING_FF_T feedForwardFW;
} MCAPP_ID_REFERENCE_T;

#undef ID_REFERNCE_FILTER_ENABLE

void MCAPP_FluxWeakeningControlInit(MCAPP_ID_REFERENCE_T *);
void MCAPP_FluxWeakeningControl(MCAPP_ID_REFERENCE_T *);

#ifdef __cplusplus
}
#endif
#endif /* end of __FDWEAK_H */
