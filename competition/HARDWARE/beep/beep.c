#include "beep.h"
#include "delay.h"
/**
  * @简  述  蜂鸣器初始化
  * @参  数  无
  * @返回值  无
  */
void BEEP_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// GPIO配置 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 
	
	// 配置GPIO  推挽输出模式 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	// 关闭蜂鸣器
	GPIO_ResetBits(GPIOE,  GPIO_Pin_1);	
}	
void BEEP_test()
{
	BEEP_Init();
	BEEP_ON();  // 打开蜂鸣器
	delay_ms(1000);  // 蜂鸣器响1秒	
	BEEP_OFF(); // 关闭蜂鸣器
}