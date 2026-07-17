#ifndef __MOTOR_CONTROL_TYPES_H
#define __MOTOR_CONTROL_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float   a;
    float   b;
    float   c;
} MC_ABC_T;

typedef struct
{
    float cos;
    float sin;
} MC_SINCOS_T;

typedef struct
{
    int16_t counts;
    float radian;
} MC_ANGLE_T;

typedef struct
{
    float RPM;
    float radpersec;
} MC_SPEED_T;

#ifdef __cplusplus
}
#endif

#endif /* end of __MOTOR_CONTROL_TYPES_H */
