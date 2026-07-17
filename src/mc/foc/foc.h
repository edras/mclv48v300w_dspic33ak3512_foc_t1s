#ifndef __FOC_H
#define __FOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"

void MCAPP_FOCInit(MCAPP_FOC_T *);
void MCAPP_FOCStateMachine(MCAPP_FOC_T *);

#ifdef __cplusplus
}
#endif

#endif /* end of __FOC_H */
