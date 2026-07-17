#include <stdint.h>
#include "pi.h"

void MC_ControllerPIUpdate(MC_PI_T *pPI)
{
    float error;
    float outUnsat;

    MC_PIPARAMS_T *pParam = &pPI->param;
    MC_PISTATE_T *pstateVar = &pPI->stateVar;

    error = pPI->inReference - pPI->inMeasure;
    outUnsat = pstateVar->integrator + pParam->kp * error;

    if (outUnsat > pParam->outMax)
    {
        pPI->output = pParam->outMax;
    }
    else if (outUnsat < pParam->outMin)
    {
        pPI->output = pParam->outMin;
    }
    else
    {
        pstateVar->integrator = pstateVar->integrator + pParam->ki * error;
        pPI->output = outUnsat;
    }
}

void MC_ControllerPIReset(MC_PI_T *pPI, float resetValue)
{
    pPI->stateVar.integrator = resetValue;
}
