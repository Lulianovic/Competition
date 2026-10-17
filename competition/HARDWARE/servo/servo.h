#ifndef _SERVO_H_
#define _SERVO_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "delay.h"


#define GRIPPER_OPEN    (-900)   // 机械爪完全打开对应的角度值 (单位0.1度)
#define GRIPPER_CLOSE   (900)    // 机械爪完全关闭对应的角度值 (单位0.1度)
#define GRIPPER_HALF    (-450)   // 机械爪半开对应的角度值 (单位0.1度)



void SERVO_S1_Init(void);                 // 舵机S1初始化
void SERVO_S1_SetAngle(int16_t angle);    // 设置舵机S1角度
void SERVO_S2_Init(void);                 // 舵机S2初始化
void SERVO_S2_SetAngle(int16_t angle);    // 设置舵机S2角度
void SERVO_S3456_Init(void);              // 舵机S3-S6初始化
void SERVO_S3_SetAngle(int16_t angle);    // 设置舵机S3角度
void SERVO_S4_SetAngle(int16_t angle);    // 设置舵机S4角度
void SERVO_S5_SetAngle(int16_t angle);    // 设置舵机S5角度
void SERVO_S6_SetAngle(int16_t angle);    // 设置舵机S6角度


void set_gripper_servo(int16_t angle);             // 设置机械爪舵机S6角度
void set_arm_servos(const int16_t angles[5]);      // 设置机械臂舵机S1-S5角度（不包含机械爪S6）
/* 平滑运动：从当前角度线性插值到目标角度 */
void move_smooth(const int16_t target[6], int steps);

extern int16_t current_angles[6];           // 当前所有舵机角度（单位0.1度）
#endif
