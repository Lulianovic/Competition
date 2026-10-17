#ifndef __BATTERY_H
#define __BATTERY_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"
#include<stdio.h>
// 电池电量阈值
#define  VBAT_40P    1065	// 电池40%电压
#define  VBAT_20P    1012	// 电池20%电压
#define  VBAT_10P    984	// 电池10%电压

void ADC2_Init(void);  // 输入电压检测初始化
uint16_t GetVoltage_X100(void);  // 获得输入电压
void GetVoltage_test(void);
#endif
