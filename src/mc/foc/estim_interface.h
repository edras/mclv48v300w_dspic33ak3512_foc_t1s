#ifndef ESTIM_INTERFACE_H
#define ESTIM_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "motor_control_types.h"
#include "estim_pll.h"

#define MCAPP_ESTIMATOR_T   MCAPP_ESTIMATOR_PLL_T

typedef struct
{
    MC_ANGLE_T thetaEle;
    MC_SPEED_T speedMech;
    int16_t qThetaOffset;
    MCAPP_ESTIMATOR_PLL_T *pEstimPLL;
} MCAPP_ESTIM_INTERFACE_T;

void MCAPP_EstimatorInit(MCAPP_ESTIM_INTERFACE_T *);
void MCAPP_EstimatorStep(MCAPP_ESTIM_INTERFACE_T *);

#ifdef __cplusplus
}
#endif

#endif /* ESTIM_INTERFACE_H */
