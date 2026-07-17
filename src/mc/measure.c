#include <stdint.h>
#include <stdbool.h>

#include "measure.h"

void MCAPP_MeasureCurrentInit(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent = &pMotorInputs->measureCurrent;

    pCurrent->counter = 0;
    pCurrent->sumIa = 0;
    pCurrent->sumIb = 0;
    pCurrent->sumIbus = 0;
    pCurrent->status = 0;
}

void MCAPP_MeasureCurrentOffset(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent = &pMotorInputs->measureCurrent;

    pCurrent->sumIa += pCurrent->Ia;
    pCurrent->sumIb += pCurrent->Ib;
    pCurrent->sumIbus += pCurrent->Ibus;
    pCurrent->counter++;

    if (pCurrent->counter >= OFFSET_COUNT_MAX)
    {
        pCurrent->offsetIa = (int16_t)(pCurrent->sumIa >> OFFSET_COUNT_BITS);
        pCurrent->offsetIb = (int16_t)(pCurrent->sumIb >> OFFSET_COUNT_BITS);
        pCurrent->offsetIbus = (int16_t)(pCurrent->sumIbus >> OFFSET_COUNT_BITS);

        pCurrent->counter = 0;
        pCurrent->sumIa = 0;
        pCurrent->sumIb = 0;
        pCurrent->sumIbus = 0;
        pCurrent->status = 1;
    }
}

void MCAPP_MeasureCurrentCalibrate(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent = &pMotorInputs->measureCurrent;

    /* Offset correction */
    pCurrent->Ia = pCurrent->Ia - pCurrent->offsetIa;
    pCurrent->Ib = pCurrent->Ib - pCurrent->offsetIb;
    pCurrent->Ibus = pCurrent->Ibus - pCurrent->offsetIbus;

    /* Convert ADC counts to actual values */
    pCurrent->Ia_actual = (float)(pCurrent->Ia * ADC_CURRENT_SCALE);
    pCurrent->Ib_actual = (float)(pCurrent->Ib * ADC_CURRENT_SCALE);
    /* Ia + Ib + Ic = 0 in a balanced 3-phase system */
    pCurrent->Ic_actual = -pCurrent->Ia_actual - pCurrent->Ib_actual;
}

int16_t MCAPP_MeasureCurrentOffsetStatus(MCAPP_MEASURE_T *pMotorInputs)
{
    return pMotorInputs->measureCurrent.status;
}
