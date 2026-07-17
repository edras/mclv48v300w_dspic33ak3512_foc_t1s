#ifndef __FOC_CONTROL_TYPES_H
#define __FOC_CONTROL_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "motor_control_types.h"

typedef struct
{
    float
        speedRef,
        targetSpeed,
        CLSpeedRampRate,
        idRef,
        iqRef;

    float
        MaxVoltageSquare;

    float
        idRefOffset,
        iqRefOffset,
        currentRamp;

    float
        lockVoltage;

    uint16_t
        openLoop,
        fluxWeakEnable,
        lockTime,
        lockTimeLimit;

} MCAPP_CONTROL_T;

typedef struct
{
    float   OLSpeedRampRate;
    MC_ANGLE_T OLtheta;
    int16_t qDeltaT;
    int32_t OLThetaSum;
    float
        lockCurrent,
        OLCurrent,
        OLCurrentMax,
        OLCurrentRampRate;
} MCAPP_OPENLOOPSTARTUP_T;

#ifdef __cplusplus
}
#endif

#endif /* end of __FOC_CONTROL_TYPES_H */
