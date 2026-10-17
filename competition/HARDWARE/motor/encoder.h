#ifndef __ENCODER_H
#define __ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"



#define  FWD_PI                3.1416f      // 圆周率PI
#define  SQRT3                 1.732f       // 3开平方
#define  PID_RATE              100           // 速度采样周期，单位Hz，100Hz即10ms一次


// 二轮差速机器人参数
// 60减速比电机，空转190rpm
#define  FWD_WHEEL_BASE         0.0972f     // 左右轮间距，单位：m
#define  FWD_WB_SCALE           1.0f        // 轮距修正系数
#define  FWD_WHEEL_DIAMETER     0.0800f     // 车轮直径，单位：m

// 编码器参数
#define  FWD_WHEEL_LINE         1024        // 编码器线数
#define  FWD_GEAR_RATIO         60          // 减速比
#define  FWD_ENCODER_RESOLUTION (FWD_WHEEL_LINE * FWD_GEAR_RATIO * 4)  // 编码器分辨率(1秒编码器的计数值) = 1024 * 60 * 4 = 245760

// 轮子速度m/s与编码器计数值的转换系数
#define  FWD_WHEEL_SCALE        (FWD_PI * FWD_WHEEL_DIAMETER * PID_RATE / FWD_ENCODER_RESOLUTION)

// 电机空载最大转速（190rpm）对应的编码器计数值（cnt/20ms）
#define  FWD_MAX_RPM            190.0f      // 电机空载最大转速（单位：rpm）
#define  FWD_MAX_ENCODER_CNT    ((FWD_MAX_RPM * FWD_WHEEL_LINE * 4) / PID_RATE)  // = 190 * 1024 * 4 / 50 = 15564.8 cnt/20ms


// 测速接口函数
void ENCODER_Left_Init(void);          		 // 编码器初始化
uint16_t ENCODER_Left_GetCounter(void);		 // 编码器获取计数器数值
void ENCODER_Left_SetCounter(uint16_t count);// 编码器设置计数器数值

void ENCODER_Right_Init(void);          		// 编码器初始化
uint16_t ENCODER_Right_GetCounter(void);     	// 编码器获取计数器数值
void ENCODER_Right_SetCounter(uint16_t count);	// 编码器设置计数器数值

#endif
