/**
 * @file mc_hal.c
 *
 * @brief Hardware Abstraction Layer for Motor Control - MCC Melody implementation.
 * 
 * This file maps the generic MC HAL interface to the dsPIC33AK512MC510 
 * peripherals configured by MCC Melody (ADC1/2/3, PWM PG1/PG2/PG3).
 */

#include <xc.h>
#include <stdint.h>

#include "mc_hal.h"

/* MCC Melody generated peripheral headers */
#include "mcc_generated_files/adc/adc1.h"
#include "mcc_generated_files/adc/adc2.h"
#include "mcc_generated_files/adc/adc3.h"

#include "mc/mc_calc_params.h"

void MC_HAL_MotorInputsRead(MCAPP_MEASURE_T *pMotorInputs)
{
    /* Phase currents: centered around mid-scale (2048), scaled to Q15 format */
    pMotorInputs->measureCurrent.Ia = (int16_t)((HALF_ADC_COUNT - (int16_t)ADC1_ConversionResultGet(ADC1_IA)) << 4);
    pMotorInputs->measureCurrent.Ib = (int16_t)((HALF_ADC_COUNT - (int16_t)ADC2_ConversionResultGet(ADC2_IB)) << 4);
    pMotorInputs->measureCurrent.Ibus = (int16_t)(((int16_t)ADC3_ConversionResultGet(ADC3_IBUS) - HALF_ADC_COUNT) << 4);

    /* Potentiometer */
    pMotorInputs->measurePot = (int16_t)ADC2_ConversionResultGet(ADC2_POT);

    /* DC bus voltage */
    pMotorInputs->measureVdc.count = (int16_t)ADC3_ConversionResultGet(ADC3_VBUS);
}

void MC_HAL_PWMEnableOutputs(void)
{
    /* Set duty cycles to zero before enabling */
    PG1DCbits.DC = 0;
    PG2DCbits.DC = 0;
    PG3DCbits.DC = 0;

    /* Remove override - PWM generators control the output pins */
    PG3IOCON2bits.OVRENH = 0;
    PG3IOCON2bits.OVRENL = 0;
    PG2IOCON2bits.OVRENH = 0;
    PG2IOCON2bits.OVRENL = 0;
    PG1IOCON2bits.OVRENH = 0;
    PG1IOCON2bits.OVRENL = 0;
}

void MC_HAL_PWMDisableOutputs(void)
{
    /* Set duty cycles to zero */
    PG1DCbits.DC = 0;
    PG2DCbits.DC = 0;
    PG3DCbits.DC = 0;

    /* Override data = LOW on all outputs */
    PG3IOCON2bits.OVRDAT = 0;
    PG2IOCON2bits.OVRDAT = 0;
    PG1IOCON2bits.OVRDAT = 0;

    /* Activate override - OVRDAT controls the output pins */
    PG3IOCON2bits.OVRENH = 1;
    PG3IOCON2bits.OVRENL = 1;
    PG2IOCON2bits.OVRENH = 1;
    PG2IOCON2bits.OVRENL = 1;
    PG1IOCON2bits.OVRENH = 1;
    PG1IOCON2bits.OVRENL = 1;
}

void MC_HAL_PWMDutySet(MC_DUTYCYCLEOUT_T *pPdc)
{
    /* Clamp duty cycles */
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

void MC_HAL_OverCurrentProtectionInit(void)
{
    /*=========================================================================
     * CMP3 / DAC3 Configuration
     * Comparator 3 monitors DC bus current (CMP3D input) against a DAC 
     * reference threshold. When bus current exceeds the limit, CMP3 output
     * triggers the PWM Fault PCI to disable outputs for the rest of the 
     * PWM cycle (cycle-by-cycle protection).
     *=========================================================================*/

    /* --- DAC Calibration --- */
    /* Read factory calibration data from Flash at 0x7F20E0 */
    uint32_t *FPDMDACaddress = (uint32_t *)(0x7F20E0);
    uint32_t FPDMDACdata = *FPDMDACaddress;
    uint32_t POSINLADJ = (FPDMDACdata & 0x00FF0000) >> 16;
    uint32_t NEGINLADJ = (FPDMDACdata & 0x0000FF00) >> 8;
    uint32_t DNLADJ    = (FPDMDACdata & 0x000000FF);

    /* --- DACCTRL1: Common DAC control --- */
    DACCTRL1 = 0;
    DACCTRL1bits.NEGINLADJ = NEGINLADJ;
    DACCTRL1bits.DNLADJ = DNLADJ;
    DACCTRL1bits.POSINLADJ = POSINLADJ;
    DACCTRL1bits.SIDL = 0;
    DACCTRL1bits.FCLKDIV = 0b111;   /* Filter clock = Divide by 8 */

    /* --- DACCTRL2: Timing --- */
    DACCTRL2 = 0;
    DACCTRL2bits.TMODTIME = 0;
    DACCTRL2bits.SSTIME = 0;

    /* --- DAC3CON: Individual DAC3 control --- */
    DAC3CON = 0;
    DAC3CONbits.DACEN = 0;   /* Will enable at the end */
    DAC3CONbits.IRQM = 0;   /* Interrupts disabled */
    DAC3CONbits.DACOEN = 0;  /* DAC output pin not connected */
    DAC3CONbits.TMCB = 0;    /* No leading-edge blanking */

    /* --- DAC3CMP: Comparator configuration --- */
    DAC3CMP = 0;
    DAC3CMPbits.HYSPOL = 0;  /* Hysteresis on rising edge */
    DAC3CMPbits.HYSSEL = 3;  /* 45 mV hysteresis */
    DAC3CMPbits.CBE = 0;     /* Blanking disabled */
    DAC3CMPbits.FLTREN = 0;  /* Digital filter disabled */
    DAC3CMPbits.CMPPOL = 0;  /* Output non-inverted: HIGH when IN+ > IN- */
    DAC3CMPbits.INPSEL = 0;  /* Positive input = CMP3A / RA5 (OPA3 bus current output) */
    DAC3CMPbits.INNSEL = 0;  /* Negative input = DACx reference */

    /* --- DAC3DAT: Set overcurrent threshold --- */
    DAC3DATbits.DACDAT = CMP_REF_DCBUS_FAULT;

    /* --- DAC3SLPCON/DAC3SLPDAT: Slope disabled --- */
    DAC3SLPCON = 0;
    DAC3SLPDAT = 0;

    /* --- Enable DAC3 and common DAC module --- */
    DAC3CONbits.DACEN = 1;
    DACCTRL1bits.ON = 1;

    /*=========================================================================
     * PWM Fault PCI Configuration (all 3 generators)
     * 
     * Mode: Cycle-by-cycle (auto-terminate at EOC when fault clears)
     * Source: CMP3 output (bit 30 in PGxF1PCI2)
     * 
     * PGxF1PCI1 = 0x03001000:
     *   - TERM = 1 (auto-terminate: cycle-by-cycle)  [bits 14:12]
     *   - ACP  = 3 (latched acceptance)              [bits 26:24]
     *   - PPS  = 0 (not inverted: active when HIGH)  [bit 28]
     *
     * PGxF1PCI2 = 0x40000000:
     *   - PSS bit 30 = Comparator 3 output
     *
     * NOTE: PCI registers must be written with generators OFF.
     * MCC enables them in PWM_Initialize(), so we disable briefly here.
     *=========================================================================*/

    /* Disable PWM generators to allow PCI register writes */
    PG1CONbits.ON = 0;
    PG2CONbits.ON = 0;
    PG3CONbits.ON = 0;

    PG1F1PCI1 = 0x03001000;
    PG1F1PCI2 = 0x40000000;

    PG2F1PCI1 = 0x03001000;
    PG2F1PCI2 = 0x40000000;

    PG3F1PCI1 = 0x03001000;
    PG3F1PCI2 = 0x40000000;

    /* Re-enable PWM generators */
    PG1CONbits.ON = 1;
    PG2CONbits.ON = 1;
    PG3CONbits.ON = 1;
}
