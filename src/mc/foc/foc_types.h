#ifndef __FOC_TYPES_H
#define __FOC_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "foc_control_types.h"
#include "clarke_park.h"
#include "../motor_types.h"
#include "svm.h"
#include "pi.h"
#include "estim_interface.h"
#include "estim_pll.h"
#include "flux_control.h"

typedef enum
{
    FOC_INIT = 0,
    FOC_RTR_LOCK = 1,
    FOC_OPEN_LOOP = 2,
    FOC_CLOSE_LOOP = 3,
    FOC_FAULT = 4,
} FOC_CONTROL_STATE_T;

typedef struct
{
    float
        *pIa,
        *pIb,
        *pIc,
        *pVdc;

    float
        vdc,
        commutationAngle;

    uint8_t
        faultStatus,
        focState;

    MC_PI_T
        piIq,
        piId,
        piSpeed;

    MC_SINCOS_T
        sincosTheta;

    MC_DQ_T
        vdq,
        idq;

    MC_ABC_T
        iabc,
        vabc,
        vabcModulation;

    MC_ALPHABETA_T
        ialphabeta,
        valphabeta;

    MCAPP_ESTIMATOR_T
        estimator;

    MCAPP_ESTIM_INTERFACE_T
        estimatorInterface;

    MCAPP_ID_REFERENCE_T
        idRefGen;

    MCAPP_CONTROL_T
        ctrlParam;

    MCAPP_OPENLOOPSTARTUP_T
        startup;

    MCAPP_MOTOR_T
        *pMotor;

    MC_DUTYCYCLEOUT_T
        *pPWMDuty;

    float pwmPeriod;

    uint8_t
        sectorSVM,
        OLrampFinished;

    uint8_t
        *directionCmd;

} MCAPP_FOC_T;

#ifdef __cplusplus
}
#endif

#endif /* end of __FOC_TYPES_H */
