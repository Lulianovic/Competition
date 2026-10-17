#include "led.h"

/**
  * @简  述  LED初始化
  * @参  数  无
  * @返回值  无
  */
void LED_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// GPIO 配置 - RGB LED 使用 PE2(绿)、PE3(蓝)、PE4(红)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 
	
	// 配置GPIO  推挽输出模式 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	// 关闭LED
	GPIO_ResetBits(GPIOE, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);	
}	
