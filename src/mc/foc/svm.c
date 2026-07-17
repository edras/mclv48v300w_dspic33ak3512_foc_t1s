#include <stdint.h>
#include "svm.h"

static void CalcTimes(float);

float T1, T2, Ta, Tb, Tc;

static void CalcTimes(float period)
{
    T1 = period * T1;
    T2 = period * T2;
    Tc = (period - T1 - T2) / 2;
    Tb = Tc + T1;
    Ta = Tb + T2;
}

uint8_t MC_CalculateSpaceVectorPhaseShifted(const MC_ABC_T *pABC, float period,
                                             MC_DUTYCYCLEOUT_T *pDutyCycleOut)
{
    uint8_t sector = 0;

    if (pABC->a > 0.0f)
    {
        sector = sector + 1;
    }
    if (pABC->b > 0.0f)
    {
        sector = sector + 2;
    }
    if (pABC->c > 0.0f)
    {
        sector = sector + 4;
    }

    switch (sector)
    {
        case 1:
            T2 = -pABC->b;
            T1 = -pABC->c;
            CalcTimes(period);
            pDutyCycleOut->dutycycle1 = Tb;
            pDutyCycleOut->dutycycle2 = Ta;
            pDutyCycleOut->dutycycle3 = Tc;
            break;
        case 2:
            T2 = -pABC->c;
            T1 = -pABC->a;
            CalcTimes(period);
            pDutyCycleOut->dutycycle1 = Ta;
            pDutyCycleOut->dutycycle2 = Tc;
            pDutyCycleOut->dutycycle3 = Tb;
            break;
        case 3:
            T2 = pABC->b;
            T1 = pABC->a;
            CalcTimes(period);
            pDutyCycleOut->dutycycle1 = Ta;
            pDutyCycleOut->dutycycle2 = Tb;
            pDutyCycleOut->dutycycle3 = Tc;
            break;
        case 4:
            T2 = -pABC->a;
            T1 = -pABC->b;
            CalcTimes(period);
            pDutyCycleOut->dutycycle1 = Tc;
            pDutyCycleOut->dutycycle2 = Tb;
            pDutyCycleOut->dutycycle3 = Ta;
            break;
        case 5:
            T2 = pABC->a;
            T1 = pABC->c;
            CalcTimes(period);
            pDutyCycleOut->dutycycle1 = Tc;
            pDutyCycleOut->dutycycle2 = Ta;
            pDutyCycleOut->dutycycle3 = Tb;
            break;
        case 6:
            T2 = pABC->c;
            T1 = pABC->b;
            CalcTimes(period);
            pDutyCycleOut->dutycycle1 = Tb;
            pDutyCycleOut->dutycycle2 = Tc;
            pDutyCycleOut->dutycycle3 = Ta;
            break;
        default:
            pDutyCycleOut->dutycycle1 = 0;
            pDutyCycleOut->dutycycle2 = 0;
            pDutyCycleOut->dutycycle3 = 0;
            break;
    }

    return sector;
}
