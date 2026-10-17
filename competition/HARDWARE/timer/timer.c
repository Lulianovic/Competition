#include "timer.h"

/*
功  能： 基本定时器7中断初始化（用于生成系统所需的中断心跳）
参  数： arr    自动重装值
         psc    时钟预分频数
返回值： 无
说  明： 定时器溢出时间计算方法：Tout=((arr+1)*(psc+1))/Ft us
         Ft：定时器工作频率（单位：Mhz）
*/
void TIM7_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				// 定义定时器时基单元初始化结构体变量
    NVIC_InitTypeDef NVIC_InitStructure;							// 定义NVIC初始化结构体变量

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);			// 使能定时器时钟

	// 先清除可能存在的标志位
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	
    TIM_TimeBaseInitStructure.TIM_Period = arr;						// 设置自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;					// 设置预分频器值
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	// 设置计数器模式为向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// 设置时钟分频

    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStructure);				// 初始化定时器

	// 在使能中断前清除中断标志位
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	// 设置计数器的初始值，避免立即溢出
    TIM_SetCounter(TIM7, 0);
	
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);						// 启用定时器的更新中断
    TIM_Cmd(TIM7, ENABLE);											// 使能定时器

    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;					// 选用定时器中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	// 抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;			// 子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					// 启动中断通道
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  TIM7 中断服务函数 
 * @retval None
 */

extern  uint32_t g_sysTick;            // 系统运行 tick 计数
void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        
        g_sysTick++;
        
        // 蜂鸣器处理 (在中断中翻转实现频率控制)
        BEEP_IRQHandler();
        
        // RGB灯处理
        LED_IRQHandler();
    }
}
