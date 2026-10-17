#include "linetracker.h"

// 光电循迹传感器（从左到右）：PA4、PA5、PA6、PA7、PC2、PC3、PC4、PC5

/**
  * @简  述  光电循迹功能初始化
  * @参  数  无
  * @返回值  无
  */
void LineTracker_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 启动GPIO时钟 
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE);
	
	// 将 PA4、PA5、PA6、PA7 初始化为输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 将 PC2、PC3、PC4、PC5 初始化为输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @简  述  获取光电传感器数据
  * @参  数  无
  * @返回值  无
  */
/* 使用直接寄存器读取（IDR），避免位带别名区不被仿真器支持的问题。 */
#define LINERD_PA(n)  ((GPIOA->IDR >> (n)) & 1)
#define LINERD_PC(n)  ((GPIOC->IDR >> (n)) & 1)

uint8_t GetLineTracker(void)
{	// 深色返回0	浅色返回1
	uint8_t value = 0;
	if( LINERD_PA(4) ) value |= (1<<0);
	if( LINERD_PA(5) ) value |= (1<<1);
	if( LINERD_PA(6) ) value |= (1<<2);
	if( LINERD_PA(7) ) value |= (1<<3);
	if( LINERD_PC(2) ) value |= (1<<4);
	if( LINERD_PC(3) ) value |= (1<<5);
	if( LINERD_PC(4) ) value |= (1<<6);
	if( LINERD_PC(5) ) value |= (1<<7);
	return value;
}

/**
  * @简  述  获取光电传感器状态
  * @参  数  无
  * @返回值  当前情况
  * 		  0x00 路线丢失
  *			  0xFF 十字路口
  */
uint8_t GetLineStatus(void)
{	// 深色返回0	浅色返回1
	uint8_t value = 0;
	if( LINERD_PA(4) ) value |= (1<<0);
	if( LINERD_PA(5) ) value |= (1<<1);
	if( LINERD_PA(6) ) value |= (1<<2);
	if( LINERD_PA(7) ) value |= (1<<3);
	if( LINERD_PC(2) ) value |= (1<<4);
	if( LINERD_PC(3) ) value |= (1<<5);
	if( LINERD_PC(4) ) value |= (1<<6);
	if( LINERD_PC(5) ) value |= (1<<7);
	return value;
}

/**
  * @简  述  根据光电传感器的状态获取转向值
  * @参  数  无
  * @返回值  转向值（负值左转，正值右转）
  */
int16_t GetLineTurning(void)
{	// 深色返回0	浅色返回1
	int16_t value = 0;
	if( LINERD_PA(4) ) value -= 200;
	if( LINERD_PA(5) ) value -= 100;
	if( LINERD_PA(6) ) value -= 50;
	if( LINERD_PA(7) ) value -= 25;
	if( LINERD_PC(2) ) value += 25;
	if( LINERD_PC(3) ) value += 50;
	if( LINERD_PC(4) ) value += 100;
	if( LINERD_PC(5) ) value += 200;
	return value;
}
