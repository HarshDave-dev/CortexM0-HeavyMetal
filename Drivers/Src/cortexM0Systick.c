#include "../Inc/cortexM0Systick.h"
#include "cortexM0BaseRegs.h"

__IO uint32_t uwTick;

CM0_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
	if (SysTick_Config(SystemCoreClock / (1000U)) > 0U)
	{
		return CM0_ERROR;
	}

	if (TickPriority < (1UL << __NVIC_PRIO_BITS))
	{
		NVIC_SetPriority(SysTick_IRQn, TickPriority);
	}
	else
	{
		return CM0_ERROR;
	}

	/* Return function status */
	return CM0_OK;
}

void HAL_IncTick(void)
{
	uwTick ++;
}



