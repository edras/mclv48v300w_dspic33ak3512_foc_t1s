#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "foc.h"
#include "foc_types.h"
#include "clarke_park.h"
#include "svm.h"
#include "estim_interface.h"
#include "estim_pll.h"
#include "../mc_calc_params.h"
#include "../mc_user_params.h"

static void MCAPP_FOCFeedbackPath(MCAPP_FOC_T *);
static void MCAPP_FOCForwardPath(MCAPP_FOC_T *);
static void MCAPP_ComputeOpenloopReference(MCAPP_OPENLOOPSTARTUP_T *, float);
static void MCAPP_CalculateModulationSignal(MC_ABC_T *, MC_ABC_T *, float);
static void MCAPP_TransformRRFvariables(MCAPP_FOC_T *, int16_t, int16_t);
static void MCAPP_FOCRotorLock(MCAPP_FOC_T *);
inline static uint8_t ReadyforClosedLoopTransition(MCAPP_FOC_T *);

void MCAPP_FOCInit(MCAPP_FOC_T *pFOC)
{
    MCAPP_CONTROL_T *pCtrlParam = &pFOC->ctrlParam;

    MC_ControllerPIReset(&pFOC->piId, 0);
    MC_ControllerPIReset(&pFOC->piIq, 0);
    MC_ControllerPIReset(&pFOC->piSpeed, 0);

    MCAPP_EstimatorInit(&pFOC->estimatorInterface);
    MCAPP_FluxWeakeningControlInit(&pFOC->idRefGen);

    pCtrlParam->lockTime = 0;
    pCtrlParam->speedRef = 0;
    pCtrlParam->idRef = 0;
    pCtrlParam->iqRef = 0;

    pFOC->faultStatus = 0;

    pFOC->pPWMDuty->dutycycle3 = 0;
    pFOC->pPWMDuty->dutycycle2 = 0;
    pFOC->pPWMDuty->dutycycle1 = 0;
}

void MCAPP_FOCStateMachine(MCAPP_FOC_T *pFOC)
{
    MCAPP_CONTROL_T *pCtrlParam = &pFOC->ctrlParam;
    uint8_t directionCmd = *pFOC->directionCmd;

    switch (pFOC->focState)
    {
        case FOC_INIT:
            MCAPP_FOCInit(pFOC);
            pFOC->focState = FOC_RTR_LOCK;
            break;

        case FOC_RTR_LOCK:
            MCAPP_FOCFeedbackPath(pFOC);
            MCAPP_FOCRotorLock(pFOC);

            if (pCtrlParam->lockTime < pCtrlParam->lockTimeLimit)
            {
                pCtrlParam->lockTime++;
            }
            else
            {
                pCtrlParam->lockTime = 0;
                pFOC->focState = FOC_OPEN_LOOP;
                pFOC->startup.OLThetaSum = 0;
                MC_ControllerPIReset(&pFOC->piId, pFOC->vdq.d);
                MC_ControllerPIReset(&pFOC->piIq, pFOC->vdq.q);
                if (directionCmd)
                {
                    pFOC->startup.OLCurrent = -(pFOC->startup.OLCurrentMax);
                }
                else
                {
                    pFOC->startup.OLCurrent = pFOC->startup.OLCurrentMax;
                }
            }
            break;

        case FOC_OPEN_LOOP:
            MCAPP_FOCFeedbackPath(pFOC);

            if (fabsf(pCtrlParam->speedRef) < pFOC->pMotor->MaxOLSpeed)
            {
                if (directionCmd)
                    pCtrlParam->speedRef -= pFOC->startup.OLSpeedRampRate;
                else
                    pCtrlParam->speedRef += pFOC->startup.OLSpeedRampRate;
                pFOC->OLrampFinished = 0;
            }
            else
            {
                pFOC->OLrampFinished = 1;
            }

            MCAPP_ComputeOpenloopReference(&pFOC->startup, pCtrlParam->speedRef);
            pFOC->commutationAngle = pFOC->startup.OLtheta.radian;

            pCtrlParam->iqRef = pFOC->startup.OLCurrent;
            pCtrlParam->idRef = 0;

            MCAPP_EstimatorStep(&pFOC->estimatorInterface);
            MCAPP_FOCForwardPath(pFOC);

            if ((pCtrlParam->openLoop == 0))
            {
                if (ReadyforClosedLoopTransition(pFOC))
                {
                    pFOC->focState = FOC_CLOSE_LOOP;
                    MCAPP_TransformRRFvariables(pFOC, pFOC->estimator.qTheta, pFOC->startup.OLtheta.counts);
                    MC_ControllerPIReset(&pFOC->piSpeed,
                        (pFOC->ctrlParam.iqRef + pFOC->ctrlParam.iqRefOffset));
                }
            }
            break;

        case FOC_CLOSE_LOOP:
            MCAPP_FOCFeedbackPath(pFOC);
            MCAPP_EstimatorStep(&pFOC->estimatorInterface);

            pFOC->commutationAngle = pFOC->estimatorInterface.thetaEle.radian;

            {
                float deltaSpeed = pCtrlParam->targetSpeed - pCtrlParam->speedRef;
                if (deltaSpeed > pCtrlParam->CLSpeedRampRate)
                    pCtrlParam->speedRef = pCtrlParam->speedRef + pCtrlParam->CLSpeedRampRate;
                else if (deltaSpeed < -pCtrlParam->CLSpeedRampRate)
                    pCtrlParam->speedRef = pCtrlParam->speedRef - pCtrlParam->CLSpeedRampRate;
                else
                    pCtrlParam->speedRef = pCtrlParam->targetSpeed;
            }

            MCAPP_FluxWeakeningControl(&pFOC->idRefGen);

            pFOC->piSpeed.inMeasure = pFOC->estimatorInterface.speedMech.RPM;
            pFOC->piSpeed.inReference = pCtrlParam->speedRef;
            pFOC->piSpeed.param.outMax = pFOC->idRefGen.iqMax;
            pFOC->piSpeed.param.outMin = -(pFOC->idRefGen.iqMax);
            MC_ControllerPIUpdate(&pFOC->piSpeed);
            pCtrlParam->iqRef = pFOC->piSpeed.output;

            if (pCtrlParam->idRefOffset > pCtrlParam->currentRamp)
                pCtrlParam->idRefOffset -= pCtrlParam->currentRamp;
            else if (pCtrlParam->idRefOffset < -pCtrlParam->currentRamp)
                pCtrlParam->idRefOffset += pCtrlParam->currentRamp;
            else
                pCtrlParam->idRefOffset = 0;

            pCtrlParam->idRef = pFOC->idRefGen.idRef + pCtrlParam->idRefOffset;
            MCAPP_FOCForwardPath(pFOC);
            break;

        case FOC_FAULT:
            break;

        default:
            pFOC->focState = FOC_FAULT;
            break;
    }
}

static void MCAPP_FOCFeedbackPath(MCAPP_FOC_T *pFOC)
{
    pFOC->iabc.a = *(pFOC->pIa);
    pFOC->iabc.b = *(pFOC->pIb);
    pFOC->iabc.c = *(pFOC->pIc);
    pFOC->vdc    = *(pFOC->pVdc);

    MC_TransformClarke(&pFOC->iabc, &pFOC->ialphabeta);
    MC_TransformPark(&pFOC->ialphabeta, &pFOC->sincosTheta, &pFOC->idq);
}

static void MCAPP_FOCForwardPath(MCAPP_FOC_T *pFOC)
{
    float vqSquaredLimit, vdSquared;

    pFOC->piId.inMeasure = pFOC->idq.d;
    pFOC->piId.inReference = pFOC->ctrlParam.idRef;
    MC_ControllerPIUpdate(&pFOC->piId);
    pFOC->vdq.d = pFOC->piId.output;

    vdSquared = pFOC->vdq.d * pFOC->vdq.d;
    vqSquaredLimit = pFOC->ctrlParam.MaxVoltageSquare - vdSquared;

    pFOC->piIq.param.outMax = sqrtf(vqSquaredLimit);
    pFOC->piIq.param.outMin = -(pFOC->piIq.param.outMax);

    pFOC->piIq.inMeasure = pFOC->idq.q;
    pFOC->piIq.inReference = pFOC->ctrlParam.iqRef;
    MC_ControllerPIUpdate(&pFOC->piIq);
    pFOC->vdq.q = pFOC->piIq.output;

    pFOC->sincosTheta.sin = sinf(pFOC->commutationAngle);
    pFOC->sincosTheta.cos = cosf(pFOC->commutationAngle);

    MC_TransformParkInverse(&pFOC->vdq, &pFOC->sincosTheta, &pFOC->valphabeta);
    MC_TransformClarkeInverseSwappedInput(&pFOC->valphabeta, &pFOC->vabc);

    MCAPP_CalculateModulationSignal(&pFOC->vabc, &pFOC->vabcModulation, pFOC->vdc);

    pFOC->sectorSVM = MC_CalculateSpaceVectorPhaseShifted(&pFOC->vabcModulation, pFOC->pwmPeriod, pFOC->pPWMDuty);
}

static void MCAPP_CalculateModulationSignal(MC_ABC_T *pabcIn, MC_ABC_T *pvabcOut, float vdc)
{
    pvabcOut->a = pabcIn->a * SQRT_3 / vdc;
    pvabcOut->b = pabcIn->b * SQRT_3 / vdc;
    pvabcOut->c = pabcIn->c * SQRT_3 / vdc;
}

static void MCAPP_ComputeOpenloopReference(MCAPP_OPENLOOPSTARTUP_T *pOL, float speedRef)
{
    pOL->OLThetaSum += (int32_t)(speedRef * pOL->qDeltaT);
    pOL->OLtheta.counts = (int16_t)(pOL->OLThetaSum >> 15);
    pOL->OLtheta.radian = ((float)pOL->OLtheta.counts) * Q15_TO_RADIAN;
}

static void MCAPP_TransformRRFvariables(MCAPP_FOC_T *pFOC,
                                         int16_t newRRFangleCount, int16_t prevRRFangleCount)
{
    float newRRFangle, id_new, iq_new, vd_new, vq_new;

    newRRFangle = ((float)(newRRFangleCount) * (float)M_PI / 32767.0f);

    pFOC->sincosTheta.sin = sinf(newRRFangle);
    pFOC->sincosTheta.cos = cosf(newRRFangle);

    int16_t deltaTheta = prevRRFangleCount - newRRFangleCount;
    float thetaOffset = ((float)deltaTheta) * (float)M_PI / 32767.0f;

    id_new = pFOC->idq.d * cosf(thetaOffset) - pFOC->idq.q * sinf(thetaOffset);
    iq_new = pFOC->idq.d * sinf(thetaOffset) + pFOC->idq.q * cosf(thetaOffset);
    vd_new = pFOC->vdq.d * cosf(thetaOffset) - pFOC->vdq.q * sinf(thetaOffset);
    vq_new = pFOC->vdq.d * sinf(thetaOffset) + pFOC->vdq.q * cosf(thetaOffset);

    MC_ControllerPIReset(&pFOC->piId, vd_new);
    MC_ControllerPIReset(&pFOC->piIq, vq_new);

    pFOC->ctrlParam.iqRefOffset = iq_new - pFOC->ctrlParam.iqRef;
    pFOC->ctrlParam.idRefOffset = id_new - pFOC->ctrlParam.idRef;
}

static void MCAPP_FOCRotorLock(MCAPP_FOC_T *pFOC)
{
    MCAPP_CONTROL_T *pCtrlParam = &pFOC->ctrlParam;

    pFOC->commutationAngle = 0;

    pFOC->sincosTheta.sin = sinf(pFOC->commutationAngle);
    pFOC->sincosTheta.cos = cosf(pFOC->commutationAngle);
    pFOC->vdq.d = pCtrlParam->lockVoltage;
    pFOC->vdq.q = 0;

    MC_TransformParkInverse(&pFOC->vdq, &pFOC->sincosTheta, &pFOC->valphabeta);
    MC_TransformClarkeInverseSwappedInput(&pFOC->valphabeta, &pFOC->vabc);
    MCAPP_CalculateModulationSignal(&pFOC->vabc, &pFOC->vabcModulation, pFOC->vdc);

    pFOC->sectorSVM = MC_CalculateSpaceVectorPhaseShifted(&pFOC->vabcModulation, pFOC->pwmPeriod, pFOC->pPWMDuty);
}

inline static uint8_t ReadyforClosedLoopTransition(MCAPP_FOC_T *pFOC)
{
    uint8_t readyStatus;

    if (pFOC->OLrampFinished == 1)
    {
        if (fabsf(pFOC->estimator.omegaFilt) > pFOC->pMotor->MaxOLSpeed)
            readyStatus = 1;
        else
            readyStatus = 0;
    }
    else
    {
        readyStatus = 0;
    }

    return readyStatus;
}
