#include "../Inc/interrupt_handler.h"
#include "../../App/Inc/main.h"
#include "cortexM0Systick.h"

void HardFault_Handler(void)
{
	while (1)
	{
		;
	}
}




void SysTick_Handler(void)
{
	HAL_IncTick();
	static int count = 0;
	if( (count % 200) == 0)
	{
		HAL_GPIO_TogglePin(GPIOC, LD3_Pin);
		HAL_GPIO_TogglePin(GPIOC, LD4_Pin);
		HAL_GPIO_TogglePin(GPIOC, LD5_Pin);
	}
	count++;
}
