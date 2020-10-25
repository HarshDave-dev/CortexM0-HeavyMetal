#include "../../App/Inc/main.h"
#include "cortexM0ResetClockConf.h"
#include "../../Drivers/Inc/cortexM0Utils.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);


int main(void)
{
	SystemClock_Config();
	MX_GPIO_Init();

	while (1)
	{
		;
	}
}


void SystemClock_Config(void)
{
	HAL_RCC_OscConfig();
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	HAL_RCC_ClockConfig(1);
}


static void MX_GPIO_Init(void)
{
	/* GPIO Ports Clock Enable */
	//Make an wrapper function in RCC.c file ~harsh
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LD3_Pin|LD6_Pin|LD4_Pin|LD5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_Init(GPIOC);
}

void SystemInit(void)
{
	;
}

void Error_Handler(void)
{
	;
}
