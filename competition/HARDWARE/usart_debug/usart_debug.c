#include "sys.h"
#include "usart_debug.h"
#include <stdio.h>

//重定义_write函数，使能printf函数
int _write (int fd, char *pBuffer, int size)  
{  
    int i = 0;
	for (i = 0; i < size; i++)  
    {	//等待发送数据寄存器为空
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
		//发送一个字节数据到USART3
        USART_SendData(USART3, (uint8_t) *pBuffer++);
    }  
    return size;  
}

void USART3_Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	// NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); // 使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);// 使能USART1时钟
 
	// 串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          
    GPIO_Init(GPIOB, &GPIO_InitStructure);                  


	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART3, &USART_InitStructure); 
	
	USART_Cmd(USART3, ENABLE);  //使能串口1 
	// USART_ClearFlag(USART1, USART_FLAG_TC);
}

// 单字符发送
void USART3_SendChar(char c)
{
    // 等待发送数据寄存器为空
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    USART_SendData(USART3, c);
}

// 字符串发
void USART3_SendString(char *str)
{
    while (*str) {
        USART3_SendChar(*str++);
    }
}
