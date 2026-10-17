#include "ps2.h"
#include<stdio.h>
// PS2手柄键值结构体
JOYSTICK_TypeDef PS2;  

/**
  * @简  述  USART1 初始化（用于手柄）
  * @参  数  baud： 波特率设置
  * @返回值	 无
  */
void ps2_init(uint32_t baud)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); // 使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);// 使能USART1时钟
 
	// 串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);	// GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);// GPIOA10复用为USART1

	// USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // GPIOA9 与 GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      // 速度0MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            // 上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  // 初始化 GPIOA

	// USART1 初始化设置
	USART_InitStructure.USART_BaudRate = baud;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	// Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
	USART_Cmd(USART1, ENABLE);  //使能串口1 
	// USART_ClearFlag(USART1, USART_FLAG_TC);
}

/*
功  能： 串口发送单字节数据
参  数： src 发送的单字节数据
返回值： 无
*/
void ps2_Send_Byte(uint8_t src)
{
    USART_SendData(USART1, src);                                    //发送数据
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);   //等待串口发送完成
}

/*
功  能： 控制手柄震动
参  数： left 左电机强度	right 右电机强度
返回值： 无
*/
void PS2_Shock(uint8_t left, uint8_t right)
{
	USART_SendData(USART1, 0x57);
	USART_SendData(USART1, left);
	USART_SendData(USART1, right);
	USART_SendData(USART1, 0x03);
}

/*
功  能： 扫描手柄键值（推荐周期50ms扫一次）
参  数： src 发送的单字节数据
返回值： 无
*/
static void PS2_Scan(void)
{
	
	printf("PS2 recv: 0x75 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x07\n",PS2.btn1,PS2.btn2,PS2.PSS_RX,PS2.PSS_RY,PS2.PSS_LX,PS2.PSS_LY);
	if( 2 == PS2.update )
	{
		// joystick->update = 0;
		PS2.SELECT  =PS2.btn1 & PS2_BT1_SELECT;	 // 选择按键
		PS2.JOY_L   =PS2.btn1 & PS2_BT1_JOY_L;	 // 左摇杆按键
		PS2.JOY_R   =PS2.btn1 & PS2_BT1_JOY_R;	 // 右摇杆按键 
		PS2.START   =PS2.btn1 & PS2_BT1_START;	 // 启动按键
		PS2.UP      =PS2.btn1 & PS2_BT1_UP;		 // 方向 上按键
		PS2.RIGHT   =PS2.btn1 & PS2_BT1_RIGHT;	 // 方向 右按键
		PS2.DOWN    =PS2.btn1 & PS2_BT1_DOWN;	 // 方向 下按键 
		PS2.LEFT    =PS2.btn1 & PS2_BT1_LEFT;	 // 方向 左按键
		PS2.L2      =PS2.btn2 & PS2_BT2_L2;		 // 肩键 L2
		PS2.R2      =PS2.btn2 & PS2_BT2_R2;		 // 肩键 R2
		PS2.L1      =PS2.btn2 & PS2_BT2_L1;		 // 肩键 L1
		PS2.R1      =PS2.btn2 & PS2_BT2_R1;		 // 肩键 R1
		PS2.Triangle=PS2.btn2 & PS2_BT2_Triangle;// 功能 Y按键（三角）
		PS2.Square  =PS2.btn2 & PS2_BT2_Square;	 // 功能 B按键（圆形）
		PS2.Circle  =PS2.btn2 & PS2_BT2_Circle;	 // 功能 A按键（叉号）
		PS2.Cross   =PS2.btn2 & PS2_BT2_Cross;	 // 功能 X按键（方形）
	}
	else if( 0 == PS2.update )
	{
		PS2.SELECT  = 0;
		PS2.JOY_L   = 0;
		PS2.JOY_R   = 0;
		PS2.START   = 0;
		PS2.UP      = 0;
		PS2.RIGHT   = 0;
		PS2.DOWN    = 0;
		PS2.LEFT    = 0;
		PS2.L2      = 0;
		PS2.R2      = 0;
		PS2.L1      = 0;
		PS2.R1      = 0;
		PS2.Triangle= 0;
		PS2.Square  = 0;
		PS2.Circle  = 0;
		PS2.Cross   = 0;
	}
}

/**
  * @简  述  UART1 中断服务函数（处理PS2键值）
  * @参  数  无 
  * @返回值  无
  */
static uint16_t uart1_rx_counter = 0;
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		uint8_t Res = USART_ReceiveData(USART1);	
		switch( uart1_rx_counter )
		{
			case 0:
				if( 0x75 == Res ) 
				{
					uart1_rx_counter++;
					PS2.update = 1;
				}
			break;
			case 1:
				PS2.btn1 = Res;
				uart1_rx_counter++;
			break;
			case 2:
				PS2.btn2 = Res;
				uart1_rx_counter++;
			break;
			case 3:
				PS2.PSS_RX = Res;
				uart1_rx_counter++;
			break;
			case 4:
				PS2.PSS_RY = Res;
				uart1_rx_counter++;
			break;
			case 5:
				PS2.PSS_LX = Res;
				uart1_rx_counter++;
			break;
			case 6:
				PS2.PSS_LY = Res;
				uart1_rx_counter++;
			break;
			case 7:
				uart1_rx_counter = 0;
				if( (0x07 == Res) && (1 == PS2.update) )	// 说明数据是正确的
				{
					PS2.update = 2;
					PS2_Scan();
					// TIM6_Init(8400-1, 500-1);	// 启动定时器，50ms中断一次
				}
				else	// 数据不正确，清空所有值
					PS2.update = 0;
			break;
			default:
				PS2.update = 0;
				uart1_rx_counter = 0;
			break;
		}
	}
}

/*
功  能： 定时器中断服务函数
参  数： 无
返回值： 无
*/
// void TIM6_DAC_IRQHandler(void)
// {
//     if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)// 溢出中断
//     {
//         TIM_ClearITPendingBit(TIM6, TIM_IT_Update);	// 清除中断标志位
// 		TIM_Cmd(TIM6, DISABLE);						// 关闭定时器
// 		PS2.update = 0;
// 		PS2_Scan();
//     }
// }



void ps2_test()
{
	ps2_init(115200);	// 初始化USART1，波特率为115200
	while(1)
	{
		;
	}
}