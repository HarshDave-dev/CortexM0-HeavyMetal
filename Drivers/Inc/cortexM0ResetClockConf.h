#ifndef __STM32F0xx_HAL_RCC_H
#define __STM32F0xx_HAL_RCC_H

#include <stdint.h>

#include "cortexM0BaseRegs.h"

#define RCC_HSI48_OFF               ((uint8_t)0x00U)
#define RCC_HSI48_ON                ((uint8_t)0x01U)

#define RCC_HCLK_DIV16                   RCC_CFGR_PPRE_DIV16 /*!< HCLK divided by 16 */

/* Initialization and de-initialization functions  ******************************/
void HAL_RCC_OscConfig(void );
void HAL_RCC_ClockConfig(uint32_t FLatency);

#endif /* __STM32F0xx_HAL_RCC_H */
