#include "battery.h"

#define ADC_REVISE  99.6

/**
  * @简  述  输入电压检测初始化
  * @参  数  无
  * @返回值  无
  */
void ADC2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

    // 使能ADC2和GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); 

    // 配置PB1为模拟输入(ADC Channel10) 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,ENABLE);	 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,DISABLE);
	
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; // ADC独立模式
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; // 采样时间延迟
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; // 禁止DMA访问
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; // ADC时钟预分频4分频
    ADC_CommonInit(&ADC_CommonInitStructure);
	
    // 配置ADC2
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // 12位分辨率
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;	// 禁止扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // 禁止连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // 禁止触发模式
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	// 右对齐
    ADC_InitStructure.ADC_NbrOfConversion = 1; // 转换1个通道
    ADC_Init(ADC2, &ADC_InitStructure);

    
	
	// ADC2 调整通道配置 
    ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);	//

    // 使能ADC2
    ADC_Cmd(ADC2, ENABLE);
}

/**
  * @简  述  VIN 获得输入电压
  * @参  数  无	  
  * @返回值  电压值，扩大100倍。例如7.2V输出为720。
  */
uint16_t GetVoltage_X100(void)
{
    uint16_t Input_Vol,temp;
	
	ADC_SoftwareStartConv(ADC2);   //使能指定的ADC2的, 软件转换启动功能 
	
    while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));//等待转换结束 
	
	temp = ADC_GetConversionValue(ADC2); // 12位ADC，右对齐，掩码0x0FFF获取有效数据
    // printf("[ADC] 原始ADC值: %u\n", temp);
	
	Input_Vol = (uint16_t)((3.3*11.0*ADC_REVISE * temp)/4095);	//分压比例1/11
	// printf("Input_Vol: %u\n", Input_Vol );
	return Input_Vol;
}

void GetVoltage_test(void)
{
    uint16_t Input_Vol;
    
    // 输入电压检测初始化
	// ADC2_Init();

    Input_Vol = GetVoltage_X100();
    
    printf("Input Voltage: %.2f V\n", Input_Vol / 100.0);
}
