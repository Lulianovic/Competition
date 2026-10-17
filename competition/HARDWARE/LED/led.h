#ifndef _LED_H_
#define _LED_H_

#include <stdint.h>
#include "stm32f4xx.h"

// RGB LED 控制宏 (PE2-绿, PE3-蓝, PE4-红)
#define LED_GREEN_ON()		GPIO_SetBits(GPIOE, GPIO_Pin_2)
#define LED_GREEN_OFF()	GPIO_ResetBits(GPIOE, GPIO_Pin_2)
#define LED_BLUE_ON()		GPIO_SetBits(GPIOE, GPIO_Pin_3)
#define LED_BLUE_OFF()	GPIO_ResetBits(GPIOE, GPIO_Pin_3)
#define LED_RED_ON()		GPIO_SetBits(GPIOE, GPIO_Pin_4)
#define LED_RED_OFF()		GPIO_ResetBits(GPIOE, GPIO_Pin_4)

void LED_Init(void);	// LED初始化

#endif
