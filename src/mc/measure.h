#ifndef __MEASURE_H
#define __MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "mc_calc_params.h"

#define OFFSET_COUNT_BITS   (int16_t)10
#define OFFSET_COUNT_MAX    (int16_t)(1 << OFFSET_COUNT_BITS)

typedef struct
{
    float Ia_actual, Ib_actual, Ic_actual;
    int16_t offsetIa, offsetIb, offsetIbus;
    int16_t Ia, Ib, Ibus;
    int16_t counter;
    int16_t status;
    int32_t sumIa, sumIb, sumIbus;
} MCAPP_MEASURE_CURRENT_T;

typedef struct
{
    int16_t count;
    float value;
} MCAPP_MEASURE_VDC_T;

typedef struct
{
    int16_t measurePot;
    MCAPP_MEASURE_CURRENT_T measureCurrent;
    MCAPP_MEASURE_VDC_T measureVdc;
} MCAPP_MEASURE_T;

void MCAPP_MeasureCurrentOffset(MCAPP_MEASURE_T *);
void MCAPP_MeasureCurrentCalibrate(MCAPP_MEASURE_T *);
void MCAPP_MeasureCurrentInit(MCAPP_MEASURE_T *);
int16_t MCAPP_MeasureCurrentOffsetStatus(MCAPP_MEASURE_T *);

#ifdef __cplusplus
}
#endif

#endif /* end of __MEASURE_H */
