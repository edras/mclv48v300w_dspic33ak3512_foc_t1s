#ifndef _HAL_H
#define _HAL_H  

#include <stdbool.h>                    // Defines true

#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/adc/adc2.h"
#include "mcc_generated_files/timer/sccp1.h"

#define MIKROE_T1S

void HAL_Init(void);

#endif