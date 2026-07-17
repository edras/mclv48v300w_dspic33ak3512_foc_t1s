#ifndef __MC_USER_PARAMS_H
#define __MC_USER_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>
#include "util.h"

/* Operational Modes */
#undef OPEN_LOOP_FUNCTIONING

/* Flux Weakening Variant:
 * 0 = Disable
 * 1 = PI Voltage feedback
 * 2 = Feed-forward */
#define FLUX_WEAKENING_VARIANT  2

/* Motor Selection:
 * 1 = Hurst DMA0204024B101
 * 4 = ACT 24V BLDC 57BLF02 */
#define MOTOR  4

#if MOTOR == 4
    #include "motor/act57blf02.h"
#else
    #include "motor/act57blf02.h"
#endif

/* Board Parameters */
#define MC1_PEAK_VOLTAGE        75.9f
#define MC1_PEAK_CURRENT        22.0f
#define DC_LINK_VOLTAGE         24.0f

/* DC Bus Utilization */
#define DCBUS_UTILIZATION_FACTOR    0.90f
#define VMAX_CLOSEDLOOP_CONTROL     ((float)DCBUS_UTILIZATION_FACTOR * DC_LINK_VOLTAGE / SQRT_3)

#ifdef __cplusplus
}
#endif

#endif /* end of __MC_USER_PARAMS_H */
