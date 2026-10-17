#ifndef _SPEED_H_
#define _SPEED_H_

#include <stdint.h>

// 电机PID控制函数
void MOTOR_SetSpeed(int16_t left, int16_t right);
// 左电机PID控制函数
int16_t MOTOR_Left_SetSpeed(int16_t target, int16_t current);
// 右电机PID控制函数
int16_t MOTOR_Right_SetSpeed(int16_t target, int16_t current);
// PID控制器复位（启动/急停/模式切换时调用）
void MOTOR_PID_Reset(void);

// 方向判断函数
uint8_t Is_Same_Direction(int16_t target, int16_t current);      // 是否同向
uint8_t Is_Opposite_Direction(int16_t target, int16_t current);  // 是否反向
uint8_t Is_Crossing_Zero(int16_t target, int16_t current);       // 是否过零

#endif
