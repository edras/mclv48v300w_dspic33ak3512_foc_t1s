#include <stdint.h>
#include <math.h>

#include "../util.h"
#include "flux_control.h"
#include "../mc_calc_params.h"
#include "foc_control_types.h"

static void MCAPP_FeedForwardFluxWeakeningInit(MCAPP_FLUX_WEAKENING_FF_T *);
static void MCAPP_FeedBackFluxWeakeningInit(MCAPP_FLUX_WEAKENING_VOLT_FB_T *);
static float MCAPP_FeedForwardFluxWeakening(MCAPP_FLUX_WEAKENING_FF_T *);
static float MCAPP_FeedBackFluxWeakening(MCAPP_FLUX_WEAKENING_VOLT_FB_T *);

void MCAPP_FluxWeakeningControlInit(MCAPP_ID_REFERENCE_T *pIdRefGen)
{
    MCAPP_FeedBackFluxWeakeningInit(&pIdRefGen->feedBackFW);
    MCAPP_FeedForwardFluxWeakeningInit(&pIdRefGen->feedForwardFW);
}

void MCAPP_FluxWeakeningControl(MCAPP_ID_REFERENCE_T *pIdRefGen)
{
    float idRefSquared, iqSquaredLimit;

    if (pIdRefGen->variant == 1)
    {
        pIdRefGen->idRef = MCAPP_FeedBackFluxWeakening(&pIdRefGen->feedBackFW);
    }
    else if (pIdRefGen->variant == 2)
    {
        pIdRefGen->idRef = MCAPP_FeedForwardFluxWeakening(&pIdRefGen->feedForwardFW);
    }
    else
    {
        pIdRefGen->idRef = 0.0f;
    }

    idRefSquared = pIdRefGen->idRef * pIdRefGen->idRef;
    iqSquaredLimit = pIdRefGen->ImaxSquare - idRefSquared;
    pIdRefGen->iqMax = sqrtf(iqSquaredLimit);
}

static void MCAPP_FeedForwardFluxWeakeningInit(MCAPP_FLUX_WEAKENING_FF_T *pFdweak)
{
    pFdweak->IdRef = 0;
}

static void MCAPP_FeedBackFluxWeakeningInit(MCAPP_FLUX_WEAKENING_VOLT_FB_T *pFdWeak)
{
    MC_ControllerPIReset(&pFdWeak->FWeakPI, 0);
}

static float MCAPP_FeedForwardFluxWeakening(MCAPP_FLUX_WEAKENING_FF_T *pFdWeak)
{
    float IdRefOut;

    const MCAPP_CONTROL_T *pCtrlParam = pFdWeak->pCtrlParam;
    const MCAPP_MOTOR_T *pMotor = pFdWeak->pMotor;

    float omega_ref = (float)(pCtrlParam->speedRef * pMotor->polePairs * RPM_TO_ELEC_RAD_PER_S);

    if (fabsf(pCtrlParam->speedRef) > pFdWeak->fwEnableSpeed)
    {
        pFdWeak->fwNum = sqrtf(SquareFloat(pFdWeak->voltageLimitFW / omega_ref) -
                               SquareFloat(pMotor->Ls * pCtrlParam->iqRef));

        IdRefOut = ((-pMotor->Ke + pFdWeak->fwNum) / pMotor->Ls);
        SaturateFloat(&IdRefOut, pFdWeak->IdRefMin, 0.0f);
    }
    else
    {
        IdRefOut = 0.0f;
    }

    pFdWeak->IdRef = IdRefOut;
    return pFdWeak->IdRef;
}

static float MCAPP_FeedBackFluxWeakening(MCAPP_FLUX_WEAKENING_VOLT_FB_T *pFdWeak)
{
    const MC_DQ_T *pVdq = pFdWeak->pVdq;
    MCAPP_CONTROL_T *pCtrlParam = pFdWeak->pCtrlParam;

    float vdSqr, vqSqr, IdRefOut;

    vdSqr = (pVdq->d * pVdq->d);
    vqSqr = (pVdq->q * pVdq->q);
    pFdWeak->voltageMag = sqrtf(vdSqr + vqSqr);

    if (fabsf(pCtrlParam->speedRef) > pFdWeak->fwEnableSpeed)
    {
        pFdWeak->FWeakPI.inReference = pFdWeak->voltageMagRef;
        pFdWeak->FWeakPI.inMeasure = pFdWeak->voltageMag;
        MC_ControllerPIUpdate(&pFdWeak->FWeakPI);
        IdRefOut = pFdWeak->FWeakPI.output;
    }
    else
    {
        IdRefOut = 0;
        MC_ControllerPIReset(&pFdWeak->FWeakPI, 0);
    }

    pFdWeak->IdRef = IdRefOut;
    return pFdWeak->IdRef;
}
