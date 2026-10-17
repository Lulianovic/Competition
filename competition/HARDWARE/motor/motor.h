#ifndef __MOTOR_H
#define __MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

// 电机控制接口函数
void MOTOR_Init(void); // 电机PWM控制初始化
void MOTOR_Left_SetPWM(int16_t pwm);	// 左电机控制
void MOTOR_Right_SetPWM(int16_t pwm);   // 右电机控制

#endif
