#ifndef MC_APP_H
#define MC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "measure.h"
#include "motor_types.h"
#include "foc/svm.h"
#include "foc/foc.h"
#include "foc/foc_types.h"
#include "fault.h"

#define MCAPP_CONTROL_SCHEME_T  MCAPP_FOC_T

typedef enum
{
    MCAPP_INIT = 0,
    MCAPP_CMD_WAIT = 1,
    MCAPP_OFFSET = 2,
    MCAPP_LOAD_START_READY_CHECK = 3,
    MCAPP_RUN = 4,
    MCAPP_STOPPING = 5,
    MCAPP_STOP = 6,
    MCAPP_FAULT = 7,
} MCAPP_STATE_T;

typedef struct
{
    uint8_t appState;
    uint8_t runCmd;
    uint8_t runCmdBuffer;
    uint8_t directionCmd;
    uint8_t directionCmdBuffer;

    float targetSpeed;

    MCAPP_MEASURE_T motorInputs;
    MCAPP_MOTOR_T motor;
    MCAPP_CONTROL_SCHEME_T controlScheme;
    MC_DUTYCYCLEOUT_T PWMDuty;
    MCAPP_FAULT_T fault;

    MCAPP_MEASURE_T *pMotorInputs;
    MCAPP_MOTOR_T *pMotor;
    MCAPP_CONTROL_SCHEME_T *pControlScheme;
    MC_DUTYCYCLEOUT_T *pPWMDuty;
} MC_APP_DATA_T;

/* Public interface */
void MC_APP_Init(void);
void MC_APP_ISR(void);
void MC_APP_SetRunCmd(uint8_t runCmd, uint8_t directionCmd);

/* Access to internal data for X2Cscope */
extern MC_APP_DATA_T mcApp;

#ifdef __cplusplus
}
#endif

#endif /* MC_APP_H */
