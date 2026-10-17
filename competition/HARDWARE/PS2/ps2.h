#ifndef _PS2_H_
#define _PS2_H_

#include <stdint.h>
#include "stm32f4xx.h"

// PS2按键定义
#define  PS2_BT1_SELECT	0x01	// 选择按键
#define  PS2_BT1_JOY_L	0x02	// 左摇杆按键
#define  PS2_BT1_JOY_R	0x04	// 右摇杆按键 
#define  PS2_BT1_START	0x08	// 启动按键
#define  PS2_BT1_UP		0x10	// 方向 上按键
#define  PS2_BT1_RIGHT	0x20	// 方向 右按键
#define  PS2_BT1_DOWN	0x40	// 方向 下按键 
#define  PS2_BT1_LEFT	0x80	// 方向 左按键

#define  PS2_BT2_L2		  0x01	// 肩键 L2
#define  PS2_BT2_R2		  0x02	// 肩键 R2
#define  PS2_BT2_L1		  0x04	// 肩键 L1
#define  PS2_BT2_R1		  0x08	// 肩键 R1
#define  PS2_BT2_Triangle 0x10	// 功能 Y按键（三角）
#define  PS2_BT2_Square	  0x20	// 功能 B按键（圆形）
#define  PS2_BT2_Circle   0x40	// 功能 A按键（叉号）
#define  PS2_BT2_Cross	  0x80	// 功能 X按键（方形）

// PS2手柄键值数据结构体	 
typedef struct			 				
{
	uint8_t update;	/* 键值是否更新的标志 0~未接收  1~正在接收  2~接收成功 */
	uint8_t btn1;   /* B0:SLCT B1:JR  B0:JL B3:STRT B4:UP B5:R B6:DOWN  B7:L   */
	uint8_t btn2;   /* B0:L2   B1:R2  B2:L1 B3:R1   B4:Y  B5:B B6:A     B7:X */
	uint8_t PSS_RX; /* 右边摇杆  0x00 = 左    0xff = 右   */
	uint8_t PSS_RY; /* 右边摇杆  0x00 = 上    0xff = 下   */
	uint8_t PSS_LX; /* 左边摇杆  0x00 = 左    0xff = 右   */
	uint8_t PSS_LY; /* 左边摇杆  0x00 = 上    0xff = 下   */
	uint8_t SELECT;	// 选择按键
	uint8_t JOY_L;	// 左摇杆按键
	uint8_t JOY_R;	// 右摇杆按键
	uint8_t START;	// 启动按键
	uint8_t UP;		// 方向 上按键
	uint8_t RIGHT;	// 方向 右按键
	uint8_t DOWN;	// 方向 下按键
	uint8_t LEFT;	// 方向 左按键
	uint8_t L2;			// 肩键 L2
	uint8_t R2;			// 肩键 R2
	uint8_t L1;			// 肩键 L1
	uint8_t R1;			// 肩键 R1
	uint8_t Triangle;	// 功能 Y按键（三角）
	uint8_t Square;		// 功能 B按键（圆形）
	uint8_t Circle;  	// 功能 A按键（叉号）
	uint8_t Cross;		// 功能 X按键（方形）
}JOYSTICK_TypeDef;

// PS2手柄键值结构体
extern JOYSTICK_TypeDef PS2;  

void ps2_init(uint32_t baud);	// USART1 初始化（用于手柄）
void ps2_Send_Byte(uint8_t src);	// 串口发送单字节数据
void PS2_Shock(uint8_t left, uint8_t right); // 控制手柄震动
void ps2_test(); // PS2测试函数
#endif
