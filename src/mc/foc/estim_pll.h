#ifndef __ESTIM_PLL_H
#define __ESTIM_PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "foc_control_types.h"
#include "../motor_types.h"
#include "clarke_park.h"

typedef struct
{
    float q30DeltaTs;
    int16_t qTheta;
    int32_t qThetaStateVar;

    float lastIalpha;
    float lastIbeta;
    float dIalpha;
    float dIbeta;
    float Valpha;
    float Vbeta;

    int16_t qDiCounter;

    float inverseDt;
    float vIndalpha;
    float vIndbeta;
    float Esdf;
    float Esqf;

    float qKfilterEsdq;
    float qOmegaFiltConst;
    float qOmegaStateVar;
    float DIlimitLS;
    float DIlimitHS;
    float LastIalphaHS[8];
    float LastIbetaHS[8];

    float qEmagFiltConst;

    float invKfiConst,
          omega,
          omegaFilt;

    float thresholdSpeedDerivative;
    float thresholdSpeedBEMF;

    MC_ALPHABETA_T BEMFAlphaBeta;
    MC_DQ_T BEMFdq;

    const MCAPP_CONTROL_T *pCtrlParam;
    const MC_ALPHABETA_T *pIAlphaBeta;
    const MC_ALPHABETA_T *pVAlphaBeta;
    const MCAPP_MOTOR_T *pMotor;

} MCAPP_ESTIMATOR_PLL_T;

void MCAPP_EstimatorPLLInit(MCAPP_ESTIMATOR_PLL_T *);
void MCAPP_EstimatorPLL(MCAPP_ESTIMATOR_PLL_T *);

#ifdef __cplusplus
}
#endif

#endif /* end of __ESTIM_PLL_H */
