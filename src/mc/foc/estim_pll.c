#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "estim_pll.h"
#include "clarke_park.h"
#include "../util.h"

void MCAPP_EstimatorPLLInit(MCAPP_ESTIMATOR_PLL_T *pEstim)
{
    pEstim->qDiCounter = 0;
    pEstim->qThetaStateVar = 0;
    pEstim->qTheta = 0;
    pEstim->omegaFilt = 0;
    pEstim->qOmegaStateVar = 0;
}

void MCAPP_EstimatorPLL(MCAPP_ESTIMATOR_PLL_T *pEstim)
{
    const MCAPP_MOTOR_T *pMotor = pEstim->pMotor;
    const MC_ALPHABETA_T *pIAlphaBeta = pEstim->pIAlphaBeta;

    MC_SINCOS_T estimSinCos;

    pEstim->Valpha = pEstim->pVAlphaBeta->alpha;
    pEstim->Vbeta  = pEstim->pVAlphaBeta->beta;

    uint16_t index = (pEstim->qDiCounter - 3) & 0x0003;

    if (fabsf(pEstim->omegaFilt) < pEstim->thresholdSpeedDerivative)
    {
        pEstim->dIalpha = (pIAlphaBeta->alpha - pEstim->LastIalphaHS[index]);
        pEstim->dIbeta = (pIAlphaBeta->beta - pEstim->LastIbetaHS[index]);

        if (pEstim->dIalpha > pEstim->DIlimitLS)
            pEstim->dIalpha = pEstim->DIlimitLS;
        if (pEstim->dIalpha < -pEstim->DIlimitLS)
            pEstim->dIalpha = -pEstim->DIlimitLS;
        if (pEstim->dIbeta > pEstim->DIlimitLS)
            pEstim->dIbeta = pEstim->DIlimitLS;
        if (pEstim->dIbeta < -pEstim->DIlimitLS)
            pEstim->dIbeta = -pEstim->DIlimitLS;

        pEstim->vIndalpha = (pMotor->Ls * pEstim->dIalpha * pEstim->inverseDt / 4);
        pEstim->vIndbeta = (pMotor->Ls * pEstim->dIbeta * pEstim->inverseDt / 4);
    }
    else
    {
        pEstim->dIalpha = (pIAlphaBeta->alpha - pEstim->LastIalphaHS[pEstim->qDiCounter]);
        pEstim->dIbeta = (pIAlphaBeta->beta - pEstim->LastIbetaHS[pEstim->qDiCounter]);

        if (pEstim->dIalpha > pEstim->DIlimitHS)
            pEstim->dIalpha = pEstim->DIlimitHS;
        if (pEstim->dIalpha < -pEstim->DIlimitHS)
            pEstim->dIalpha = -pEstim->DIlimitHS;
        if (pEstim->dIbeta > pEstim->DIlimitHS)
            pEstim->dIbeta = pEstim->DIlimitHS;
        if (pEstim->dIbeta < -pEstim->DIlimitHS)
            pEstim->dIbeta = -pEstim->DIlimitHS;

        pEstim->vIndalpha = (pMotor->Ls * pEstim->dIalpha * pEstim->inverseDt);
        pEstim->vIndbeta = (pMotor->Ls * pEstim->dIbeta * pEstim->inverseDt);
    }

    pEstim->qDiCounter = (pEstim->qDiCounter + 1) & 0x0003;
    pEstim->LastIalphaHS[pEstim->qDiCounter] = pIAlphaBeta->alpha;
    pEstim->LastIbetaHS[pEstim->qDiCounter] = pIAlphaBeta->beta;

    pEstim->BEMFAlphaBeta.alpha = (pEstim->Valpha - (pMotor->Rs * pIAlphaBeta->alpha) - pEstim->vIndalpha);
    pEstim->BEMFAlphaBeta.beta  = (pEstim->Vbeta - (pMotor->Rs * pIAlphaBeta->beta) - pEstim->vIndbeta);

    estimSinCos.sin = sinf(pEstim->qTheta * Q15_TO_RADIAN);
    estimSinCos.cos = cosf(pEstim->qTheta * Q15_TO_RADIAN);

    MC_TransformPark(&pEstim->BEMFAlphaBeta, &estimSinCos, &pEstim->BEMFdq);

    LowPassFilter(pEstim->BEMFdq.d, pEstim->qKfilterEsdq, &pEstim->Esdf);
    LowPassFilter(pEstim->BEMFdq.q, pEstim->qKfilterEsdq, &pEstim->Esqf);

    if (fabsf(pEstim->omegaFilt) > pEstim->thresholdSpeedBEMF)
    {
        if (pEstim->Esqf > 0)
            pEstim->omega = (pEstim->invKfiConst * (pEstim->Esqf - pEstim->Esdf));
        else
            pEstim->omega = (pEstim->invKfiConst * (pEstim->Esqf + pEstim->Esdf));
    }
    else
    {
        if (pEstim->omegaFilt > 0)
            pEstim->omega = (pEstim->invKfiConst * (pEstim->Esqf - pEstim->Esdf));
        else
            pEstim->omega = (pEstim->invKfiConst * (pEstim->Esqf + pEstim->Esdf));
    }

    pEstim->qThetaStateVar += (int32_t)(pEstim->omega * pEstim->q30DeltaTs);
    pEstim->qTheta = (int16_t)(pEstim->qThetaStateVar >> 15);

    LowPassFilter(pEstim->omega, pEstim->qOmegaFiltConst, &pEstim->omegaFilt);
}
