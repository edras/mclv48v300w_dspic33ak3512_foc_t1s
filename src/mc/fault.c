#include <stdint.h>
#include <stdbool.h>

#include "fault.h"

bool MCAPP_OverCurrentFault_Detect(MCAPP_MEASURE_T *pMotorInputs, MCAPP_FAULT_T *pFault)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent = &pMotorInputs->measureCurrent;

    bool faultStatus;
    float Imax, Ic;

    Ic = -pCurrent->Ia_actual - pCurrent->Ib_actual;

    if ((pCurrent->Ia_actual > pCurrent->Ib_actual) && (pCurrent->Ia_actual > Ic))
    {
        Imax = pCurrent->Ia_actual;
    }
    else if (pCurrent->Ib_actual > Ic)
    {
        Imax = pCurrent->Ib_actual;
    }
    else
    {
        Imax = Ic;
    }

    if (Imax > pFault->overCurrentFaultLimit)
    {
        pFault->faultState = MCAPP_OVERCURRENT_FAULT_PHASE;
        faultStatus = true;
    }
    else
    {
        pFault->faultState = MCAPP_FAULT_NONE;
        faultStatus = false;
    }

    return faultStatus;
}
