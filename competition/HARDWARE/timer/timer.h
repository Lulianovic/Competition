#ifndef _TIMER_H
#define _TIMER_H


#include "stm32f4xx.h"
#include <stdint.h>


// 基本定时器7中断初始化（用于生成系统所需的中断心跳）
void TIM7_Init(uint16_t arr, uint16_t psc);

// 蜂鸣器中断处理函数声明（由用户实现）
void BEEP_IRQHandler(void);
// RGB灯中断处理函数声明（由用户实现）
void LED_IRQHandler(void);

#endif
