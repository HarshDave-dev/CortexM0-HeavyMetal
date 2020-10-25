#include "../Inc/cortexM0Gpio.h"

#define GPIO_NUMBER           (16U)

void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx)
{
	GPIOx->MODER |= (1 << 12);
	GPIOx->MODER |= (1 << 14);
	GPIOx->MODER |= (1 << 16);
	GPIOx->MODER |= (1 << 18);
}

void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  if (PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin;
  }
  else
  {
    GPIOx->BRR = (uint32_t)GPIO_Pin;
  }
}

void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  uint32_t odr;
  odr = GPIOx->ODR;
  GPIOx->BSRR = ((odr & GPIO_Pin) << GPIO_NUMBER) | (~odr & GPIO_Pin);
}
