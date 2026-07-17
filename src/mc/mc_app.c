#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <xc.h>

#include "mc_app.h"
#include "mc_user_params.h"
#include "mc_calc_params.h"
#include "foc/foc.h"

/* MCC Melody generated ADC headers */
#include "mcc_generated_files/adc/adc1.h"
#include "mcc_generated_files/adc/adc2.h"
#include "mcc_generated_files/adc/adc3.h"

/* Application data instance */
MC_APP_DATA_T mcApp;
static MC_APP_DATA_T *pMCApp = &mcApp;

/* Static function prototypes */
static void MC_APP_StateMachine(MC_APP_DATA_T *pMCData);
static void MC_APP_ParamsInit(MC_APP_DATA_T *pMCData);
static void MC_APP_ControlSchemeConfig(MC_APP_DATA_T *pMCData);
static void MC_APP_ReceivedDataProcess(MC_APP_DATA_T *pMCData);
static void MC_APP_MotorInputsRead(MCAPP_MEASURE_T *pMotorInputs);
static void MC_APP_PWMEnable(void);
static void MC_APP_PWMDisable(void);
static void MC_APP_PWMDutySet(MC_DUTYCYCLEOUT_T *pPdc);

/**
 * @brief Initialize the motor control application
 */
void MC_APP_Init(void)
{
    MC_APP_ParamsInit(pMCApp);
    MC_APP_PWMDisable();
}

/**
 * @brief ISR entry point - called from ADC callback at 20kHz
 */
void MC_APP_ISR(void)
{
    MC_APP_MotorInputsRead(pMCApp->pMotorInputs);
    MC_APP_StateMachine(pMCApp);
    MC_APP_PWMDutySet(pMCApp->pPWMDuty);
}

/**
 * @brief Set run and direction commands from buttons
 */
void MC_APP_SetRunCmd(uint8_t runCmd, uint8_t directionCmd)
{
    pMCApp->runCmdBuffer = runCmd;
    pMCApp->directionCmdBuffer = directionCmd;
}

/**
 * @brief Application state machine
 */
static void MC_APP_StateMachine(MC_APP_DATA_T *pMCData)
{
    MCAPP_MEASURE_T *pMotorInputs = pMCData->pMotorInputs;
    MCAPP_CONTROL_SCHEME_T *pControlScheme = pMCData->pControlScheme;

    MC_APP_ReceivedDataProcess(pMCData);

    switch (pMCData->appState)
    {
        case MCAPP_INIT:
            MC_APP_PWMDisable();
            pMCData->runCmd = 0;
            pMCData->directionCmd = 0;
            MCAPP_FOCInit(pControlScheme);
            MCAPP_MeasureCurrentInit(pMotorInputs);
            pMCData->appState = MCAPP_CMD_WAIT;
            break;

        case MCAPP_CMD_WAIT:
            if (pMCData->runCmd == 1)
            {
                pMCData->appState = MCAPP_OFFSET;
            }
            break;

        case MCAPP_OFFSET:
            MCAPP_MeasureCurrentOffset(pMotorInputs);
            if (MCAPP_MeasureCurrentOffsetStatus(pMotorInputs))
            {
                pMCData->appState = MCAPP_LOAD_START_READY_CHECK;
            }
            break;

        case MCAPP_LOAD_START_READY_CHECK:
            MCAPP_MeasureCurrentCalibrate(pMotorInputs);
            /* Enable PWM outputs and start FOC */
            MC_APP_PWMEnable();
            pControlScheme->focState = FOC_INIT;
            pMCData->appState = MCAPP_RUN;
            break;

        case MCAPP_RUN:
            MCAPP_MeasureCurrentCalibrate(pMotorInputs);

            /* Check for over current fault */
            if (MCAPP_OverCurrentFault_Detect(pMotorInputs, &pMCData->fault))
            {
                pMCData->appState = MCAPP_FAULT;
                break;
            }

            MCAPP_FOCStateMachine(pControlScheme);

            if (pControlScheme->faultStatus == 1)
            {
                pMCData->appState = MCAPP_FAULT;
                break;
            }

            if (pMCData->runCmd == 0)
            {
                pMCData->appState = MCAPP_STOPPING;
            }
            break;

        case MCAPP_STOPPING:
            /* Keep FOC running with targetSpeed = 0, decelerate smoothly */
            MCAPP_MeasureCurrentCalibrate(pMotorInputs);
            MCAPP_FOCStateMachine(pControlScheme);

            /* Wait until motor speed drops below minimum speed */
            if (fabsf(pControlScheme->estimatorInterface.speedMech.RPM) < pMCData->pMotor->MinSpeed)
            {
                pMCData->appState = MCAPP_STOP;
            }
            break;

        case MCAPP_STOP:
            MC_APP_PWMDisable();
            pMCData->appState = MCAPP_INIT;
            break;

        case MCAPP_FAULT:
            MC_APP_PWMDisable();
            break;

        default:
            MC_APP_PWMDisable();
            break;
    }

    if ((pControlScheme->faultStatus == 1) || (pMCData->appState == MCAPP_FAULT))
    {
        MC_APP_PWMDisable();
    }
}

/**
 * @brief Initialize all motor control parameters
 */
static void MC_APP_ParamsInit(MC_APP_DATA_T *pMCData)
{
    memset(pMCData, 0, sizeof(MC_APP_DATA_T));

    pMCData->pControlScheme = &pMCData->controlScheme;
    pMCData->pMotorInputs = &pMCData->motorInputs;
    pMCData->pMotor = &pMCData->motor;
    pMCData->pPWMDuty = &pMCData->PWMDuty;

    MC_APP_ControlSchemeConfig(pMCData);
    pMCData->appState = MCAPP_INIT;
}

/**
 * @brief Configure the FOC control scheme parameters
 */
static void MC_APP_ControlSchemeConfig(MC_APP_DATA_T *pMCData)
{
    MCAPP_CONTROL_SCHEME_T *pCS = pMCData->pControlScheme;
    MCAPP_MEASURE_T *pMotorInputs = pMCData->pMotorInputs;
    MCAPP_MOTOR_T *pMotor = pMCData->pMotor;

    /* Configure current inputs */
    pCS->pIa = &pMotorInputs->measureCurrent.Ia_actual;
    pCS->pIb = &pMotorInputs->measureCurrent.Ib_actual;
    pCS->pIc = &pMotorInputs->measureCurrent.Ic_actual;
    pCS->pVdc = &pMotorInputs->measureVdc.value;
    pCS->pMotor = pMCData->pMotor;
    pCS->directionCmd = &pMCData->directionCmd;

    /* Motor parameters */
    pMotor->polePairs    = POLE_PAIRS;
    pMotor->Rs           = MOTOR_PER_PHASE_RESISTANCE;
    pMotor->Ls           = MOTOR_PER_PHASE_INDUCTANCE;
    pMotor->Ke           = MOTOR_BEMF_CONSTANT_ELEC;
    pMotor->NominalSpeed = NOMINAL_SPEED_RPM;
    pMotor->MaxSpeed     = MAXIMUM_SPEED_RPM;
    pMotor->MaxOLSpeed   = MAX_OPENLOOP_SPEED_RPM;
    pMotor->MinSpeed     = MINIMUM_SPEED_RPM;
    pMotor->RatedCurrent = NOMINAL_CURRENT_PEAK;

    /* Control parameters */
#ifdef OPEN_LOOP_FUNCTIONING
    pCS->ctrlParam.openLoop = 1;
#else
    pCS->ctrlParam.openLoop = 0;
#endif
    pCS->ctrlParam.lockTimeLimit = LOCK_TIME_COUNTS;
    pCS->ctrlParam.lockVoltage = LOCKING_VOLTAGE;
    pCS->ctrlParam.CLSpeedRampRate = CL_SPEED_REF_RAMP_VALUE;
    pCS->ctrlParam.currentRamp = CURRENT_RAMP_VALUE;
    pCS->ctrlParam.MaxVoltageSquare = MAX_VOLTAGE_SQUARE;

    /* Fault */
    pMCData->fault.overCurrentFaultLimit = OC_FAULT_LIMIT_PHASE;

    /* Startup */
    pCS->startup.lockCurrent = LOCK_CURRENT;
    pCS->startup.OLCurrent = OPEN_LOOP_CURRENT;
    pCS->startup.OLCurrentMax = OPEN_LOOP_CURRENT;
    pCS->startup.OLSpeedRampRate = OL_SPEED_REF_RAMP_VALUE;
    pCS->startup.qDeltaT = DELTA_T_Q30;

    /* PI controllers - Id */
    pCS->piId.param.kp = D_CURRCNTR_PTERM;
    pCS->piId.param.ki = D_CURRCNTR_ITERM;
    pCS->piId.param.outMax = D_CURRCNTR_OUTMAX;
    pCS->piId.param.outMin = -D_CURRCNTR_OUTMAX;

    /* PI controllers - Iq */
    pCS->piIq.param.kp = Q_CURRCNTR_PTERM;
    pCS->piIq.param.ki = Q_CURRCNTR_ITERM;
    pCS->piIq.param.outMax = Q_CURRCNTR_OUTMAX;
    pCS->piIq.param.outMin = -Q_CURRCNTR_OUTMAX;

    /* PI controllers - Speed */
    pCS->piSpeed.param.kp = SPEEDCNTR_PTERM;
    pCS->piSpeed.param.ki = SPEEDCNTR_ITERM;
    pCS->piSpeed.param.outMax = SPEEDCNTR_OUTMAX;
    pCS->piSpeed.param.outMin = -SPEEDCNTR_OUTMAX;

    /* PLL Estimator */
    pCS->estimator.pCtrlParam = &pCS->ctrlParam;
    pCS->estimator.pIAlphaBeta = &pCS->ialphabeta;
    pCS->estimator.pVAlphaBeta = &pCS->valphabeta;
    pCS->estimator.pMotor = pMCData->pMotor;
    pCS->estimator.invKfiConst = ESTIM_INVERSE_BEMF_CONSTANT;
    pCS->estimator.qKfilterEsdq = KFILTER_ESDQ;
    pCS->estimator.q30DeltaTs = DELTA_T_Q30;
    pCS->estimator.qOmegaFiltConst = KFILTER_VELESTIM;
    pCS->estimator.thresholdSpeedBEMF = 100.0f;
    pCS->estimator.thresholdSpeedDerivative = THRESHOLD_SPEED_DERIVATIVE;
    pCS->estimator.inverseDt = (float)(1.0f / MC1_LOOPTIME_SEC);
    pCS->estimator.DIlimitHS = D_ILIMIT_HS;
    pCS->estimator.DIlimitLS = D_ILIMIT_LS;

    /* Estimator Interface */
    pCS->estimatorInterface.pEstimPLL = &pCS->estimator;

    /* Flux Weakening */
    pCS->idRefGen.variant = FLUX_WEAKENING_VARIANT;
    pCS->idRefGen.feedBackFW.pCtrlParam = &pCS->ctrlParam;
    pCS->idRefGen.feedBackFW.pVdq = &pCS->vdq;
    pCS->idRefGen.feedBackFW.pMotor = pMCData->pMotor;
    pCS->idRefGen.feedBackFW.voltageMagRef = EFFECTIVE_VOLATGE_FW;
    pCS->idRefGen.feedBackFW.FWeakPI.param.outMax = 0;
    pCS->idRefGen.feedBackFW.FWeakPI.param.outMin = MAX_FW_NEGATIVE_ID_REF;
    pCS->idRefGen.feedBackFW.FWeakPI.param.kp = FW_PTERM;
    pCS->idRefGen.feedBackFW.FWeakPI.param.ki = FW_ITERM;
    pCS->idRefGen.feedBackFW.IdRefFiltConst = KFILTER_FW_IDREF;
    pCS->idRefGen.feedBackFW.fwEnableSpeed = FLUX_WEAKENING_ENABLE_SPEED;

    pCS->idRefGen.feedForwardFW.pCtrlParam = &pCS->ctrlParam;
    pCS->idRefGen.feedForwardFW.pMotor = pMCData->pMotor;
    pCS->idRefGen.feedForwardFW.IdRefMin = MAX_FW_NEGATIVE_ID_REF;
    pCS->idRefGen.feedForwardFW.voltageLimitFW = EFFECTIVE_VOLATGE_FW;
    pCS->idRefGen.feedForwardFW.IdRefFiltConst = KFILTER_FW_IDREF;
    pCS->idRefGen.feedForwardFW.fwEnableSpeed = FLUX_WEAKENING_ENABLE_SPEED;

    pCS->idRefGen.ImaxSquare = IMAX_SQUARE_FW;

    /* PWM period for SVM */
    pCS->pwmPeriod = (float)LOOPTIME_TCY;
    pCS->pPWMDuty = pMCData->pPWMDuty;
}

/**
 * @brief Process input data (potentiometer -> speed reference)
 */
static void MC_APP_ReceivedDataProcess(MC_APP_DATA_T *pMCData)
{
    MCAPP_CONTROL_SCHEME_T *pControlScheme = pMCData->pControlScheme;
    MCAPP_MOTOR_T *pMotor = pMCData->pMotor;
    MCAPP_MEASURE_T *pMotorInputs = pMCData->pMotorInputs;
    float potValueNormalized;

    pMCData->runCmd = pMCData->runCmdBuffer;
    pMCData->directionCmd = pMCData->directionCmdBuffer;

    potValueNormalized = ((float)pMotorInputs->measurePot / (float)MAX_ADC_COUNT);

    pMCData->targetSpeed = pMotor->MinSpeed +
                           ((pMotor->MaxSpeed - pMotor->MinSpeed) * potValueNormalized);

    if (pMCData->directionCmd)
    {
        pMCData->targetSpeed = -pMCData->targetSpeed;
    }

    if (pMCData->runCmd == 1)
    {
        pControlScheme->ctrlParam.targetSpeed = pMCData->targetSpeed;
    }
    else
    {
        pControlScheme->ctrlParam.targetSpeed = 0.0f;
    }

    pMotorInputs->measureVdc.value = pMotorInputs->measureVdc.count * MC1_PEAK_VOLTAGE / (float)MAX_ADC_COUNT;
}

/**
 * @brief Read motor inputs from ADC (MCC Melody API)
 */
static void MC_APP_MotorInputsRead(MCAPP_MEASURE_T *pMotorInputs)
{
    /* Read phase currents - centered around mid-scale (2048), scaled to Q15 */
    pMotorInputs->measureCurrent.Ia = (int16_t)((HALF_ADC_COUNT - (int16_t)ADC1_ConversionResultGet(ADC1_IA)) << 4);
    pMotorInputs->measureCurrent.Ib = (int16_t)((HALF_ADC_COUNT - (int16_t)ADC2_ConversionResultGet(ADC2_IB)) << 4);
    pMotorInputs->measureCurrent.Ibus = (int16_t)(((int16_t)ADC3_ConversionResultGet(ADC3_IBUS) - HALF_ADC_COUNT) << 4);

    /* Read potentiometer */
    pMotorInputs->measurePot = (int16_t)ADC2_ConversionResultGet(ADC2_POT);

    /* Read DC bus voltage */
    pMotorInputs->measureVdc.count = (int16_t)ADC3_ConversionResultGet(ADC3_VBUS);
}

/**
 * @brief Enable PWM outputs (remove override)
 */
static void MC_APP_PWMEnable(void)
{
    PG1DCbits.DC = 0;
    PG2DCbits.DC = 0;
    PG3DCbits.DC = 0;

    PG3IOCON2bits.OVRENH = 0;
    PG3IOCON2bits.OVRENL = 0;
    PG2IOCON2bits.OVRENH = 0;
    PG2IOCON2bits.OVRENL = 0;
    PG1IOCON2bits.OVRENH = 0;
    PG1IOCON2bits.OVRENL = 0;
}

/**
 * @brief Disable PWM outputs (activate override to LOW)
 */
static void MC_APP_PWMDisable(void)
{
    PG1DCbits.DC = 0;
    PG2DCbits.DC = 0;
    PG3DCbits.DC = 0;

    PG3IOCON2bits.OVRDAT = 0;
    PG2IOCON2bits.OVRDAT = 0;
    PG1IOCON2bits.OVRDAT = 0;

    PG3IOCON2bits.OVRENH = 1;
    PG3IOCON2bits.OVRENL = 1;
    PG2IOCON2bits.OVRENH = 1;
    PG2IOCON2bits.OVRENL = 1;
    PG1IOCON2bits.OVRENH = 1;
    PG1IOCON2bits.OVRENL = 1;
}

/**
 * @brief Set PWM duty cycles from FOC output (MCC Melody registers)
 */
static void MC_APP_PWMDutySet(MC_DUTYCYCLEOUT_T *pPdc)
{
    /* Limit duty cycles */
    if (pPdc->dutycycle1 < MIN_DUTY) pPdc->dutycycle1 = MIN_DUTY;
    if (pPdc->dutycycle2 < MIN_DUTY) pPdc->dutycycle2 = MIN_DUTY;
    if (pPdc->dutycycle3 < MIN_DUTY) pPdc->dutycycle3 = MIN_DUTY;
    if (pPdc->dutycycle1 > MAX_DUTY) pPdc->dutycycle1 = MAX_DUTY;
    if (pPdc->dutycycle2 > MAX_DUTY) pPdc->dutycycle2 = MAX_DUTY;
    if (pPdc->dutycycle3 > MAX_DUTY) pPdc->dutycycle3 = MAX_DUTY;

    /* Write to PWM duty cycle registers */
    PG1DCbits.DC = (uint32_t)(pPdc->dutycycle1);
    PG2DCbits.DC = (uint32_t)(pPdc->dutycycle2);
    PG3DCbits.DC = (uint32_t)(pPdc->dutycycle3);
}
