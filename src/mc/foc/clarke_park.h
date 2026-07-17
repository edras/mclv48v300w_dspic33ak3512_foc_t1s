#ifndef __CLARKE_PARK_H
#define __CLARKE_PARK_H

#include <stdint.h>
#include <stdbool.h>

#include "motor_control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float d;
    float q;
} MC_DQ_T;

typedef struct
{
    float alpha;
    float beta;
} MC_ALPHABETA_T;

void MC_TransformClarkeInverseSwappedInput(const MC_ALPHABETA_T *, MC_ABC_T *);
void MC_TransformParkInverse(const MC_DQ_T *, const MC_SINCOS_T *, MC_ALPHABETA_T *);
void MC_TransformPark(const MC_ALPHABETA_T *, const MC_SINCOS_T *, MC_DQ_T *);
void MC_TransformClarke(const MC_ABC_T *, MC_ALPHABETA_T *);

#ifdef __cplusplus
}
#endif
#endif /* end of __CLARKE_PARK_H */
