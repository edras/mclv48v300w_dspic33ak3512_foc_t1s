#ifndef __PI_H
#define __PI_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float outMax;
    float outMin;
} MC_PIPARAMS_T;

typedef struct
{
    float integrator;
} MC_PISTATE_T;

typedef struct
{
    MC_PIPARAMS_T param;
    MC_PISTATE_T stateVar;
    float inReference;
    float inMeasure;
    float output;
} MC_PI_T;

void MC_ControllerPIUpdate(MC_PI_T *);
void MC_ControllerPIReset(MC_PI_T *, float resetValue);

#ifdef __cplusplus
}
#endif
#endif /* end of __PI_H */
