#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint16_t polePairs;
    float Rs;
    float Ls;
    float Ke;
    float NominalSpeed;
    float MaxSpeed;
    float MinSpeed;
    float MaxOLSpeed;
    float RatedCurrent;
} MCAPP_MOTOR_T;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TYPES_H */
