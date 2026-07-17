#ifndef __SVM_H
#define __SVM_H

#include <stdint.h>
#include <stdbool.h>

#include "clarke_park.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float dutycycle1;
    float dutycycle2;
    float dutycycle3;
} MC_DUTYCYCLEOUT_T;

uint8_t MC_CalculateSpaceVectorPhaseShifted(const MC_ABC_T *, float, MC_DUTYCYCLEOUT_T *);

#ifdef __cplusplus
}
#endif
#endif /* end of __SVM_H */
