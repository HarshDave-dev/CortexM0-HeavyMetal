#ifndef __MAIN_H
#define __MAIN_H
#include "../../Drivers/Inc/cortexM0Gpio.h"
#include "../../Drivers/Inc/cortexM0Systick.h"

void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define LD3_Pin GPIO_PIN_6
#define LD3_GPIO_Port GPIOC
#define LD6_Pin GPIO_PIN_7
#define LD6_GPIO_Port GPIOC
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD5_Pin GPIO_PIN_9
#define LD5_GPIO_Port GPIOC


#endif /* __MAIN_H */
