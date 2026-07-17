#ifndef __MC_CALC_PARAMS_H
#define __MC_CALC_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

#include "mc_user_params.h"

/* PWM Parameters - must match MCC Melody PWM configuration */
#define PWMFREQUENCY_HZ             20000
#define PWM_CLOCK_MHZ               400
#define DEADTIME_MICROSEC           0.75f
#define MC1_LOOPTIME_SEC            (float)(1.0f / PWMFREQUENCY_HZ)
#define LOOPTIME_MICROSEC           (MC1_LOOPTIME_SEC * 1000000.0f)
#define DEADTIME                    (uint32_t)(DEADTIME_MICROSEC * 16 * PWM_CLOCK_MHZ)
#define LOOPTIME_TCY                (uint32_t)((LOOPTIME_MICROSEC * 8 * PWM_CLOCK_MHZ) - 16)

/* Duty Cycle Limits */
#define MIN_DUTY                    (uint32_t)(DEADTIME + DEADTIME)
#define MAX_DUTY                    (LOOPTIME_TCY - (uint32_t)(DEADTIME + DEADTIME))

/* ADC Scaling */
#define MAX_ADC_COUNT               4096.0f
#define HALF_ADC_COUNT              2048
#define ADC_CURRENT_SCALE           (float)(MC1_PEAK_CURRENT / 32768.0f)

/* Motor BEMF Constant (electrical) */
#define MOTOR_BEMF_CONSTANT_ELEC    (float)(((((MOTOR_BEMF_CONSTANT_MECH / SQRT_3) * 60.0f) / 1000.0f) / (2.0f * M_PI)) / POLE_PAIRS)

/* Delta Ts factor for angle accumulation */
#define DELTA_T_Q30                 (float)(MC1_LOOPTIME_SEC * (2.0f / 60.0f) * POLE_PAIRS * Q30_MAX)

/* Back EMF PLL Estimator Parameters */
#define D_ILIMIT_HS                 (float)(NOMINAL_CURRENT_PEAK * MC1_LOOPTIME_SEC * MAXIMUM_SPEED_RPM * (2.0f * M_PI / 60.0f) * POLE_PAIRS)
#define D_ILIMIT_LS                 (float)(4.0f * D_ILIMIT_HS)
#define ESTIM_INVERSE_BEMF_CONSTANT (float)(1.0f / (MOTOR_BEMF_CONSTANT_MECH / (SQRT_3 * 1000.0f)))
#define THRESHOLD_SPEED_DERIVATIVE  (float)MAXIMUM_SPEED_RPM

/* BEMF Estimator filter parameters */
#define TAU_EDQ                     (float)(1.0f / (2.0f * M_PI * BEMF_FILTER_CUTOFF_FREQUENCY))
#define KFILTER_ESDQ                (float)(MC1_LOOPTIME_SEC / (MC1_LOOPTIME_SEC + TAU_EDQ))
/* Velocity filter parameters */
#define TAU_VELESTIM                (float)(1.0f / (2.0f * M_PI * VELOCITY_FILTER_CUTOFF_FREQUENCY))
#define KFILTER_VELESTIM            (float)(MC1_LOOPTIME_SEC / (MC1_LOOPTIME_SEC + TAU_VELESTIM))

/* Rotor Lock parameters */
#define LOCK_TIME_COUNTS            (uint16_t)(LOCK_TIME_SEC / MC1_LOOPTIME_SEC)
#define LOCKING_VOLTAGE             (float)((LOCK_CURRENT * MOTOR_PER_PHASE_RESISTANCE) * 2.0f)

/* Flux Weakening Parameters */
#define EFFECTIVE_VOLATGE_FW        (float)(VMAX_CLOSEDLOOP_CONTROL * FW_VOLATGE_MARGIN_FACTOR)
#define FLUX_WEAKENING_ENABLE_SPEED (float)(NOMINAL_SPEED_RPM / 2.0f)
#define TAU_FW_IDREF                (float)(1.0f / (2.0f * M_PI * FW_ID_FILTER_CUTOFF_FREQUENCY))
#define KFILTER_FW_IDREF            (float)(MC1_LOOPTIME_SEC / (MC1_LOOPTIME_SEC + TAU_FW_IDREF))
#define IMAX_SQUARE_FW              (float)(SPEEDCNTR_OUTMAX * SPEEDCNTR_OUTMAX)

/* Ramp Rates */
#define OL_SPEED_REF_RAMP_VALUE     (float)(OPEN_LOOP_SPEED_REF_RAMP_RATE * MC1_LOOPTIME_SEC)
#define CL_SPEED_REF_RAMP_VALUE     (float)(CLOSED_LOOP_SPEED_REF_RAMP_RATE * MC1_LOOPTIME_SEC)

/* Square of Maximum Voltage */
#define MAX_VOLTAGE_SQUARE          ((float)VMAX_CLOSEDLOOP_CONTROL * VMAX_CLOSEDLOOP_CONTROL)

/* Overcurrent fault comparator reference */
#define CMP_REF_DCBUS_FAULT         (uint16_t)(((OC_FAULT_LIMIT_DCBUS * HALF_ADC_COUNT) / MC1_PEAK_CURRENT) + HALF_ADC_COUNT)

#ifdef __cplusplus
}
#endif

#endif /* end of __MC_CALC_PARAMS_H */
