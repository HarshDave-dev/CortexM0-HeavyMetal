#ifndef __CORTEXM0SYSTICK_H
#define __CORTEXM0SYSTICK_H

#include <stdint.h>

typedef enum
{
  CM0_OK       = 0x00U,
  CM0_ERROR    = 0x01U,
  CM0_BUSY     = 0x02U,
  CM0_TIMEOUT  = 0x03U
} CM0_StatusTypeDef;

CM0_StatusTypeDef HAL_InitTick (uint32_t TickPriority);
void HAL_IncTick(void);

extern uint32_t SystemCoreClock;


#endif /* __CORTEXM0SYSTICK_H */
