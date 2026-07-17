#ifndef __ACT57BLF02_H
#define __ACT57BLF02_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdint.h>

#include "../mc_user_params.h"

/* Parameters of ACT 24V BLDC 57BLF02 */
#define POLE_PAIRS                      4
#define MOTOR_PER_PHASE_RESISTANCE      0.2672f
#define MOTOR_PER_PHASE_INDUCTANCE      0.000135f
#define MOTOR_BEMF_CONSTANT_MECH        6.57f
#define MINIMUM_SPEED_RPM               500.0f
#define NOMINAL_SPEED_RPM               3000.0f
#define MAXIMUM_SPEED_RPM               3500.0f
#define NOMINAL_CURRENT_PHASE_RMS       7.0f

#define NOMINAL_CURRENT_PEAK            (float)(NOMINAL_CURRENT_PHASE_RMS * M_SQRT2)

/* D-axis Current Control Loop - PI Coefficients */
#define D_CURRCNTR_PTERM                0.450f
#define D_CURRCNTR_ITERM                0.044533f
#define D_CURRCNTR_OUTMAX               VMAX_CLOSEDLOOP_CONTROL

/* Q-axis Current Control Loop - PI Coefficients */
#define Q_CURRCNTR_PTERM                0.450f
#define Q_CURRCNTR_ITERM                0.044533f
#define Q_CURRCNTR_OUTMAX               VMAX_CLOSEDLOOP_CONTROL

/* Speed Control Loop - PI Coefficients */
#define SPEEDCNTR_PTERM                 0.00552f
#define SPEEDCNTR_ITERM                 0.0000124422f
#define SPEEDCNTR_OUTMAX                NOMINAL_CURRENT_PEAK

/* Filter constants */
#define BEMF_FILTER_CUTOFF_FREQUENCY    250.0f
#define VELOCITY_FILTER_CUTOFF_FREQUENCY 75.0f

/* Control parameters */
#define OPEN_LOOP_CURRENT               1.00f
#define OPEN_LOOP_SPEED_REF_RAMP_RATE   2000.0f
#define MAX_OPENLOOP_SPEED_RPM          MINIMUM_SPEED_RPM
#define CLOSED_LOOP_SPEED_REF_RAMP_RATE 2000.0f
#define CURRENT_RAMP_VALUE              0.05f

/* Fault parameters */
#define OC_FAULT_LIMIT_PHASE            20.0f
#define OC_FAULT_LIMIT_DCBUS            10.0f

/* Rotor locking parameters */
#define LOCK_TIME_SEC                   0.75f
#define LOCK_CURRENT                    0.75f

/* Flux Weakening Parameters */
#define FW_VOLATGE_MARGIN_FACTOR        0.90f
#define MAX_FW_NEGATIVE_ID_REF          (-(0.90f * NOMINAL_CURRENT_PEAK))
#define FW_PTERM                        (SPEEDCNTR_PTERM * 30.0f)
#define FW_ITERM                        (SPEEDCNTR_ITERM * 30.0f)
#define FW_ID_FILTER_CUTOFF_FREQUENCY   100.0f

#ifdef __cplusplus
}
#endif

#endif /* end of __ACT57BLF02_H */
