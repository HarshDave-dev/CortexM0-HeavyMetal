#include "../Inc/cortexM0ResetClockConf.h"
#include "../Inc/cortexM0Utils.h"

uint32_t SystemCoreClock = 8000000;

void HAL_RCC_OscConfig(void)
{
	SET_BIT(RCC->CR2, RCC_CR2_HSI48ON);

    /* Wait till HSI48 is ready */
	while(RCC->CR2 & (1 << 17) == 0)
    {
		;
    }
}


void HAL_RCC_ClockConfig(uint32_t FLatency)
{
	if(FLatency > READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))
	{
		FLASH->ACR = (FLASH->ACR&(~FLASH_ACR_LATENCY)) | (FLatency);
	}

	/*-------------------------- HCLK Configuration --------------------------*/
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, RCC_HCLK_DIV16);

    /* Set the new HCLK clock divider */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, /*RCC_ClkInitStruct->AHBCLKDivider*/0);

    /*------------------------- SYSCLK Configuration ---------------------------*/
    /* Check the HSI48 ready flag */
    if(RCC->CR2 & (1 << 17) == 0)
    {
    	return;
    }

    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, 3);
    
    while ((uint32_t)(RCC->CFGR & RCC_CFGR_SWS) != (3 << RCC_CFGR_SWS_Pos))
    {
    	;
    }

    if(FLatency > READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))
    {
    	FLASH->ACR = (FLASH->ACR&(~FLASH_ACR_LATENCY)) | (FLatency);
    }

    /*-------------------------- PCLK1 Configuration ---------------------------*/
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, 0);
  
    //Setting up the system clock prescaler 1ms system tick.
    SystemCoreClock = 48000000  >> 1;

    HAL_InitTick (0/*TICK_INT_PRIORITY*/);
}

