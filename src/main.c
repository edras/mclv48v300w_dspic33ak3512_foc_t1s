/*
 (c) [2026] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS "AS IS." 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#include "hal.h"
#include "systick/systick.h"
#include "T1S/t1s_lwip.h"
#include "T1S/udp_perf_client.h"
#include "X2Cscope/X2Cscope.h"
#include "mc/mc_app.h"
#include "app.h"

#include <stdio.h>

/* Heartbeat LED */
uint16_t heartBeatCount = 0;
#define HEART_BEAT_LED_COUNT_mS 250

/* Flag to prevent ISR execution before initialization completes */
static volatile uint8_t mcInitDone = 0;

/* Direction change state: stop motor, toggle direction, then auto-restart */
static uint8_t directionChangePending = 0;

/* variable to store FOC elapsed time */
static volatile uint32_t timerFOC = 0;


int main(void)
{
    HAL_Init();
    SysTick_Initialize();
    APP_Buttons_Init();

    printf("dsPIC33AK512MC510 + FOC + T1S\r\n");

    /* Initialize motor control (FOC) */
    MC_APP_Init();

    /* Initialize T1S network and X2Cscope over TCP */
    T1S_init();
    X2Cscope_Init();

    /* Signal that initialization is complete - ISR can now run FOC */
    mcInitDone = 1;

    SYSTICK_TIMEOUT led_timeout;
    SysTick_StartTimeOut(&led_timeout, HEART_BEAT_LED_COUNT_mS);

    while (1)
    {
        /* Service X2Cscope and T1S communication */
        X2Cscope_Communicate();
        T1S_execute();
        APP_Buttons_Task();

        /* SW1: Start/Stop Motor */
        if (APP_Button_WasPressed(APP_BTN_SW1))
        {
            if (mcApp.runCmdBuffer == 0)
            {
                mcApp.runCmdBuffer = 1;
            }
            else
            {
                mcApp.runCmdBuffer = 0;
            }
        }

        /* SW2: Toggle Direction - stop, reverse, restart */
        if (APP_Button_WasPressed(APP_BTN_SW2))
        {
            /* Toggle direction */
            if (mcApp.directionCmdBuffer == 0)
                mcApp.directionCmdBuffer = 1;
            else
                mcApp.directionCmdBuffer = 0;

            /* If motor is running, stop it and request auto-restart */
            if (mcApp.runCmdBuffer == 1)
            {
                mcApp.runCmdBuffer = 0;
                directionChangePending = 1;
            }
        }

        /* Auto-restart after direction change once motor has stopped */
        if (directionChangePending && (mcApp.appState == MCAPP_CMD_WAIT))
        {
            mcApp.runCmdBuffer = 1;
            directionChangePending = 0;
        }

        /* LED2 indicates motor running */
        mcApp.runCmd == 1 ? LED2_SetHigh() : LED2_SetLow();

        /* LED1 heartbeat */
        if (SysTick_IsTimeoutReached(&led_timeout))
        {
            SysTick_ResetTimeOut(&led_timeout);
            LED1_Toggle();
        }
    }

    return 0;
}

/**
 * @brief ADC2 Channel Callback - fires at 20kHz (PWM-triggered)
 * 
 * This is the control loop ISR. It calls the FOC state machine
 * and updates X2Cscope.
 */
void ADC2_ChannelCallback(enum ADC2_CHANNEL channel, uint16_t adcVal)
{
    if (channel != ADC2_POT) return;
    if (!mcInitDone) return;

    /* Start FOC timer */
    SCCP1_Timer_Start();

    /* Execute FOC control loop */
    MC_APP_ISR();

    /* Update X2Cscope sample buffer */
    X2Cscope_Update();

    /* Measure elapsed time for FOC loop */
    SCCP1_Timer_Stop();
    timerFOC = SCCP1_Timer_CounterGet(); /* Store elapsed time for FOC loop */
    timerFOC *= 40; /* Convert to nanoseconds (40ns per tick @ 200MHz) */
    CCP1TMR = 0; /* Reset timer for next FOC loop */  
}
