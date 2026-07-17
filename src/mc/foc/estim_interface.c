#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "estim_interface.h"
#include "../mc_calc_params.h"

void MCAPP_EstimatorInit(MCAPP_ESTIM_INTERFACE_T *pEstimInt)
{
    MCAPP_EstimatorPLLInit(pEstimInt->pEstimPLL);
}

void MCAPP_EstimatorStep(MCAPP_ESTIM_INTERFACE_T *pEstimInt)
{
    MCAPP_EstimatorPLL(pEstimInt->pEstimPLL);

    pEstimInt->speedMech.RPM = pEstimInt->pEstimPLL->omegaFilt;
    pEstimInt->thetaEle.counts = pEstimInt->pEstimPLL->qTheta + pEstimInt->qThetaOffset;
    pEstimInt->thetaEle.radian = ((float)pEstimInt->thetaEle.counts) * Q15_TO_RADIAN;
}
