#ifndef _BEEP_H_
#define _BEEP_H_

#include <stdint.h>
#include "stm32f4xx.h"


// 打开蜂鸣器
#define BEEP_ON()	GPIO_SetBits(GPIOE, GPIO_Pin_1)


// 关闭蜂鸣器
#define BEEP_OFF()	GPIO_ResetBits(GPIOE, GPIO_Pin_1)

void BEEP_Init(void);	// 蜂鸣器初始化
void BEEP_test(); // 蜂鸣器响指定时间（单位：毫秒）
#endif
