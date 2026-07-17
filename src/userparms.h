/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* (c) [2024] Microchip Technology Inc. and its subsidiaries
* 
* Subject to your compliance with these terms, you may use this Microchip 
* software and any derivatives exclusively with Microchip products. 
* You are responsible for complying with third party license terms applicable to
* your use of third party software (including open source software) that may 
* accompany this Microchip software.
* 
* Redistribution of this Microchip software in source or binary form is allowed 
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.
* 
* SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL 
* MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
* CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY
* LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL
* NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS
* SOFTWARE
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
#ifndef _USER_HEADER
#define _USER_HEADER

#include <math.h>


#define MAX_ADC_COUNT               (uint16_t)4095
#define MAX_CURRENT                 (float)(22.0)
#define ADC_CURRENT_SCALE           (float)(MAX_CURRENT/(float)2048)
#define ADC_CURRENT_SCALE_INVERSE   (float)((float)2048/ MAX_CURRENT)
#define MAX_VOLTAGE                 (float)(76.0)
#define ADC_VOLTAGE_SCALE           (float)(MAX_VOLTAGE/(float)4096)
#define ADC_VOLTAGE_SCALE_INVERSE   (float)((float)4096/ MAX_VOLTAGE)
#define DC_BUS_VOLTAGE              (float)24

/* Specify PWM Frequency in Hertz */
#define PWMFREQUENCY_HZ         20000/* Specify dead time in micro seconds */
#define DEADTIME_MICROSEC       0.75

#define FCY                     200000000UL
/*Specify PWM Module Clock in Mega Hertz*/
#define PWM_CLOCK_MHZ           200 

/* Specify PWM Period in micro seconds */
#define LOOPTIME_MICROSEC       50
        
// Specify bootstrap charging time in Seconds (mention at least 10mSecs)
#define BOOTSTRAP_CHARGING_TIME_SECS 0.01
  
// Calculate Bootstrap charging time in number of PWM Half Cycles
#define BOOTSTRAP_CHARGING_COUNTS (uint16_t)((BOOTSTRAP_CHARGING_TIME_SECS/LOOPTIME_SEC )* 2)
        
// Definition to enable or disable PWM Fault
#undef ENABLE_PWM_FAULT
/*Dead time in terms of PWM clock period*/  
#define DEADTIME                            (uint32_t)(DEADTIME_MICROSEC*16*PWM_CLOCK_MHZ)
/*Loop time in terms of PWM clock period*/
#define LOOPTIME_TCY                        (uint32_t)((LOOPTIME_MICROSEC*8*PWM_CLOCK_MHZ)-16)

#define MIN_DUTY                    (uint32_t)(2 * DEADTIME)
#define MAX_DUTY                    LOOPTIME_TCY - (uint32_t)(5 * DEADTIME)

#define LOOPTIME_SEC                (float)(1.0/PWMFREQUENCY_HZ)  

/* Define to run Hurst DMA0204024B101(AC300022: Hurst300 or Long Hurst)*/
#undef HURST300

/* Define to run ACT 24V BLDC 57BLF02 */
#define ACT57BLF02

/* Define to run ACT 24V BLDC 57BLF02-EC1024-500-T */
#undef ACT57BLF02_EC1024_500_T

#ifdef HURST300
    #define POLE_PAIRS                  (float)(5.0)
#endif

#ifdef ACT57BLF02
    #define POLE_PAIRS                  (float)(4.0)
#endif

#ifdef ACT57BLF02_EC1024_500_T
    #define POLE_PAIRS                  (float)(4.0)
#endif

#endif