#include "speaker.h"
#include <stdio.h>
#include "delay.h"
#define UART5_DMA_RX_BUFFER_MAX_LENGTH		(16)   // 回传数据很短，减小缓冲区
#define UART5_DMA_TX_BUFFER_MAX_LENGTH		(4096) // 最大4091字节文本+帧头

uint8_t UART5_DMA_RX_Buffer[UART5_DMA_RX_BUFFER_MAX_LENGTH];
uint8_t UART5_DMA_TX_Buffer[UART5_DMA_TX_BUFFER_MAX_LENGTH];

volatile uint8_t Speaker_DMA_Busy = 0;      // DMA发送忙标志
volatile uint8_t Speaker_Playing = 0;       // 语音播放忙标志（根据回传0x4E/0x4F）
volatile Speaker_StateTypeDef Speaker_State = SPEAKER_STATE_IDLE;

// 内部函数声明
static void UART5_DMA_Tx_Configuration(void);
static void UART5_SendFrame(uint8_t cmd, uint8_t param, uint8_t *data, uint16_t data_len);

// UART5 端口初始化
void UART5_Configuration(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    USART_DeInit(UART5);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
    // PC12为UART5_TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    
    // PD2为UART5_RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
    
    // 配置UART5
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);
    
    // 使能接收中断（用于接收状态回传）
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    
    // 配置中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 清除标志并使能
    USART_ClearFlag(UART5, USART_FLAG_TC | USART_FLAG_RXNE);
    USART_Cmd(UART5, ENABLE);
    
    // 使能DMA发送请求
    USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);
}

// DMA发送配置
static void UART5_DMA_Tx_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Stream7);
    while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE);
    
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART5->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UART5_DMA_TX_Buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = UART5_DMA_TX_BUFFER_MAX_LENGTH;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream7, &DMA_InitStructure);
    
    // 使能DMA发送完成中断
    DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


// DMA发送完成中断
void DMA1_Stream7_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream7, DMA_IT_TCIF7))
    {
        DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);
        DMA_Cmd(DMA1_Stream7, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Stream7, 0);
        
        Speaker_DMA_Busy = 0;  // DMA发送完成，但语音可能还在播放
        
#ifdef __DEBUG_stm32f407__
        __DEBUG_UART5_DMA_TX_COMPLETE++;
#endif
    }
}

// UART5中断处理（接收芯片回传状态）
void UART5_IRQHandler(void)
{
    uint8_t rx_data;
    
    // 接收数据中断
    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        rx_data = USART_ReceiveData(UART5);
        // printf("rx_data = 0x%x\n",rx_data);
        // 解析回传状态
        switch (rx_data)
        {
            case 0x4A:  // 初始化成功/唤醒成功
                Speaker_State = SPEAKER_STATE_IDLE;
                break;
                
            case 0x41:  // 接收成功（命令帧正确）
                // 收到此回传表示命令已接收，如果是合成命令，芯片开始播音
                Speaker_Playing = 1;
                Speaker_State = SPEAKER_STATE_BUSY;
                break;
                
            case 0x45:  // 接收失败（命令帧错误）
                Speaker_State = SPEAKER_STATE_ERROR;
                break;
                
            case 0x4E:  // 正在播音
                Speaker_Playing = 1;
                Speaker_State = SPEAKER_STATE_BUSY;
				
                break;
                
            case 0x4F:  // 芯片空闲（播放完成或处于空闲）
                Speaker_Playing = 0;
                Speaker_State = SPEAKER_STATE_IDLE;
                break;
                
            default:
                break;
        }
    }
}

// 发送原始帧数据（内部函数）
static void UART5_SendFrame(uint8_t cmd, uint8_t param, uint8_t *data, uint16_t data_len)
{
    uint16_t frame_len = 1 ; // 命令字+命令参数+文本长度
	if(cmd == 0x01)
	{
		frame_len = 2+ data_len;
	}
    
    // 等待语音模块空闲在发数据过去
    while (Speaker_Playing);
    // 构建帧
    UART5_DMA_TX_Buffer[0] = 0xFD;              // 帧头
    UART5_DMA_TX_Buffer[1] = (frame_len >> 8) & 0xFF;  // 长度高字节
    UART5_DMA_TX_Buffer[2] = frame_len & 0xFF;         // 长度低字节
    UART5_DMA_TX_Buffer[3] = cmd;               // 命令字
    UART5_DMA_TX_Buffer[4] = param;             // 命令参数
    
    // 复制文本数据
    if (data != NULL && data_len > 0)
    {
        memcpy(&UART5_DMA_TX_Buffer[5], data, data_len);
    }
    
    // 配置DMA并发送
    DMA_Cmd(DMA1_Stream7, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE);
    
    DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7 | 
                  DMA_FLAG_TEIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_FEIF7);
    
    DMA_SetCurrDataCounter(DMA1_Stream7, frame_len + 3);
    Speaker_DMA_Busy = 1;
    DMA_Cmd(DMA1_Stream7, ENABLE);
    
    // 清除TC标志
    USART_ClearFlag(UART5, USART_FLAG_TC);
}

// 初始化语音模块
void UART_Speaker_Init(void)
{
    UART5_Configuration(9600);          // 默认9600，可根据硬件配置修改
    UART5_DMA_Tx_Configuration();
    
    Speaker_DMA_Busy = 0;
    Speaker_Playing = 0;
    Speaker_State = SPEAKER_STATE_IDLE;
    
    // 等待芯片初始化完成（上电后会发送0x4A）
    // 如果芯片已上电，可能需要发送唤醒命令
    delay_ms(200);  // 等待芯片启动
    

    Speaker_SetVoice(SPEAKER_VOICE_DONALDDUCK); // 默认使用唐老鸭效果器
    Speaker_SetVolume(2);// 默认音量5，调小一些避免过大声
    Speaker_SetSpeed(20);// 默认语速5，调慢一些更清晰
    Speaker_SetTone(5);// 默认语调5，保持不变
}

// 基础语音合成（默认GB2312编码）
void UART_Speaker(char *speech_data)
{
    UART_Speaker_WithEncode(speech_data, SPEAKER_ENCODE_UTF8);
}

// 指定编码格式进行语音合成
void UART_Speaker_WithEncode(char *speech_data, Speaker_EncodeTypeDef encode)
{
    uint16_t len = strlen((char *)speech_data);
    
    // 长度检查：GB2312/GBK/Unicode最大4091字节，UTF8最大2045字节
    if (encode == SPEAKER_ENCODE_UTF8 && len > 2045) len = 2045;
    else if (len > 4091) len = 4091;
    
    UART5_SendFrame(0x01, encode, (uint8_t *)speech_data, len);
}

// 停止合成
void Speaker_Stop(void)
{
    UART5_SendFrame(0x02, 0x00, NULL, 0);
    Speaker_Playing = 0;
    Speaker_State = SPEAKER_STATE_IDLE;
}

// 暂停合成
void Speaker_Pause(void)
{
    UART5_SendFrame(0x03, 0x00, NULL, 0);
}

// 恢复合成
void Speaker_Resume(void)
{
    UART5_SendFrame(0x04, 0x00, NULL, 0);
}

// 进入Deep Sleep模式（低功耗）
void Speaker_Sleep(void)
{
    UART5_SendFrame(0x22, 0x00, NULL, 0);  // 或0x88
    Speaker_State = SPEAKER_STATE_SLEEP;
    Speaker_Playing = 0;
}

// 唤醒芯片
void Speaker_WakeUp(void)
{
    // 发送唤醒命令0xFF
    UART5_SendFrame(0xFF, 0x00, NULL, 0);
    delay_ms(175);  // 手册要求：唤醒到收到0x4A回传时间为175ms
}

// 查询芯片状态
void Speaker_QueryState(void)
{
    UART5_SendFrame(0x21, 0x00, NULL, 0);
}

// 设置音量（0-10，默认5）
void Speaker_SetVolume(uint8_t level)
{
    char buf[16];
    if (level > 10) level = 10;
    sprintf(buf, "[v%d]", level);
    UART_Speaker(buf);
    delay_ms(100);  // 等待设置完成
}

// 设置语速（0-30，默认5，值越小语速越慢）
void Speaker_SetSpeed(uint8_t level)
{
    char buf[16];
    if (level > 30) level = 30;
    sprintf(buf, "[s%d]", level);
    UART_Speaker(buf);
    delay_ms(100);
}

// 设置语调（0-10，默认5）
void Speaker_SetTone(uint8_t level)
{
    char buf[16];
    if (level > 10) level = 10;
    sprintf(buf, "[t%d]", level);
    UART_Speaker(buf);
    delay_ms(100);
}

// 切换发音人
void Speaker_SetVoice(Speaker_VoiceTypeDef voice)
{
    char buf[16];
    sprintf(buf, "[m%d]", voice);
    UART_Speaker(buf);
    delay_ms(100);
}

// 等待播放完成（阻塞式，超时约10秒）
uint8_t Speaker_WaitForIdle(void)
{
    uint32_t timeout = 0;
    while (Speaker_Playing && timeout < 2000)  // 1秒超时
    {
        delay_ms(1);
        timeout++;
        
        // 定期查询状态（可选）
        if (timeout % 500 == 0)  // 每500ms查询一次
        {
            Speaker_QueryState();
        }
    }

    if(timeout >= 2000) {
        printf("[警告] 等待播放完成超时！\n");
        return 0; // 超时
    }
    return 1; // 成功
}

// 获取忙状态（DMA发送忙或语音播放忙）
uint8_t Speaker_IsBusy(void)
{
    return (Speaker_DMA_Busy || Speaker_Playing);
}

// 获取详细状态
Speaker_StateTypeDef Speaker_GetState(void)
{
    return Speaker_State;
}

// 发送原始帧（高级用户直接操作）
void Speaker_SendRawFrame(uint8_t *data, uint16_t len)
{
	while (Speaker_DMA_Busy);
    
    if (len > UART5_DMA_TX_BUFFER_MAX_LENGTH)
        len = UART5_DMA_TX_BUFFER_MAX_LENGTH;
    
    memcpy(UART5_DMA_TX_Buffer, data, len);
    
    DMA_Cmd(DMA1_Stream7, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE);
    DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7 | 
                  DMA_FLAG_TEIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_FEIF7);
    DMA_SetCurrDataCounter(DMA1_Stream7, len);
    Speaker_DMA_Busy = 1;
    DMA_Cmd(DMA1_Stream7, ENABLE);
    USART_ClearFlag(UART5, USART_FLAG_TC);
}