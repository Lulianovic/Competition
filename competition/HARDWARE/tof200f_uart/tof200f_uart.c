#include "tof200f_uart.h"
#include <stdio.h>
#include "delay.h"

/* ==================== 私有变量 ==================== */

/* 8字节滑动窗口缓冲区 */
static uint8_t slide_buffer[TOF_FRAME_SIZE];
static volatile uint8_t slide_count = 0;


/* 处理后的有效帧数据 */
 TOF_Frame_t current_frame[TOF_PHY_SIZE];
 volatile bool frame_ready = false;

/* 系统滴答计数 */
static volatile uint32_t sys_tick_ms = 0;

/* ==================== 私有函数 ==================== */

/**
 * @brief 检查8字节缓冲区是否是合法帧
 * @param buf: 8字节缓冲区
 * @return: true=合法, false=不合法
 * @note: 合法标准：前6字节CRC == 后2字节（小端序）
 */
static bool IsValidFrame(uint8_t *buf)
{
    uint16_t calc_crc = TOF_Modbus_CRC16(buf, 5);
    uint16_t rx_crc = (buf[6] << 8) | buf[5];
    return (calc_crc == rx_crc);
}

/**
 * @brief 查找帧头位置
 * @param buf: 缓冲区
 * @param len: 缓冲区长度
 * @return: 帧头位置（0-based），-1=未找到
 * @note: 帧头标准：从机地址(1-6) + 0x03 + 0x02
 */
static int16_t FindFrameHeader(uint8_t *buf, uint8_t len)
{
    for (uint8_t i = 0; i <= len - 3; i++)
    {
        if (buf[i] >= 1 && buf[i] <= 6 && buf[i + 1] == 0x03 &&
            buf[i + 2] == 0x02)
        {
            return i;
        }
    }
    return -1;
}

/**
 * @brief 解析8字节缓冲区到帧结构
 * @param buf: 8字节缓冲区
 * @param frame: 输出帧结构
 */
static void ParseFrame(uint8_t *buf)
{

    if(buf[0] >= 1 && buf[0] <= 6  && buf[1] == 0x03 && buf[2] == 0x02) 
    {
        current_frame[buf[0]-1].slave_addr = buf[0];
        current_frame[buf[0]-1].distance   = (buf[3] << 8) | buf[4];
        current_frame[buf[0]-1].valid      = true;
        frame_ready = true;
    }              
    
    /* 清空缓冲区，准备接收下一帧 */
    slide_count = 0;
}

/**
 * @brief 滑动窗口处理 - 核心算法
 * @param new_data: 新接收到的字节
 * @note: 
 *   1. 新数据加入滑动缓冲区
 *   2. 缓冲区满8字节时检查
 *   3. CRC合法则保存帧数据
 *   4. 不合法则找帧头，移动数据，继续等待
 */
static void SlideWindow_Process(uint8_t new_data)
{
    /* 新数据加入缓冲区尾部 */
    if (slide_count < TOF_FRAME_SIZE) {
        slide_buffer[slide_count++] = new_data;
    }
    
    /* 缓冲区未满，继续接收 */
    if (slide_count < TOF_FRAME_SIZE) {
        return;
    }
    /* 缓冲区已满8字节，开始检查 */
    while (slide_count >= TOF_FRAME_SIZE) {
        /* 检查是否是合法帧（CRC校验） */
        if (IsValidFrame(slide_buffer)) {
            /* 合法帧，解析并保存 */
            ParseFrame(slide_buffer);
            return;
        }
        
        /* 不合法，寻找帧头 */
        int16_t header_pos = FindFrameHeader(slide_buffer, slide_count);
        
        if (header_pos > 0) {
            /* 找到帧头，丢弃帧头前的数据 */
            uint8_t move_count = slide_count - header_pos;
            for (uint8_t i = 0; i < move_count; i++) {
                slide_buffer[i] = slide_buffer[header_pos + i];
            }
            slide_count = move_count;
            
            /* 如果移动后仍满8字节，继续循环检查 */
            if (slide_count >= TOF_FRAME_SIZE) {
                continue;
            }
        } else  {
            /* 帧头在位置0，但CRC不合法，可能是数据损坏 ；也可能没有找到针头*/
            /* 丢弃第一个字节，继续接收 */
            for (uint8_t i = 0; i < slide_count - 1; i++) {
                slide_buffer[i] = slide_buffer[i + 1];
            }
            slide_count--;
        } 
        
        /* 退出循环，继续接收新数据 */
        break;
    }
}

/* ==================== 公共函数 ==================== */

/**
 * @brief 初始化 UART4 及 GPIO
 */
void TOF_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    /* 1. 使能时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    
    /* 2. 配置 GPIO 复用功能 */
    GPIO_PinAFConfig(TOF_UART_TX_PORT, TOF_UART_TX_PINSOURCE, TOF_UART_TX_AF);
    GPIO_PinAFConfig(TOF_UART_RX_PORT, TOF_UART_RX_PINSOURCE, TOF_UART_RX_AF);
    
    /* 3. 初始化 TX (PG14) */
    GPIO_InitStruct.GPIO_Pin = TOF_UART_TX_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(TOF_UART_TX_PORT, &GPIO_InitStruct);
    
    /* 4. 初始化 RX (PG9) */
    GPIO_InitStruct.GPIO_Pin = TOF_UART_RX_PIN;
    GPIO_Init(TOF_UART_RX_PORT, &GPIO_InitStruct);
    
    /* 5. 配置 UART6 参数 */
    USART_InitStruct.USART_BaudRate = TOF_UART_BAUDRATE;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(TOF_UART_INSTANCE, &USART_InitStruct);
    
    /* 6. 配置 NVIC */
    NVIC_InitStruct.NVIC_IRQChannel = TOF_UART_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = TOF_UART_PRE_PRIO;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = TOF_UART_SUB_PRIO;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    /* 7. 使能 UART6 */
    USART_Cmd(TOF_UART_INSTANCE, ENABLE);
    
    /* 8. 初始化变量 */
    slide_count = 0;
    frame_ready = false;
    memset(&current_frame, 0, sizeof(current_frame));
    
    /* 9. 启动接收中断 */
    TOF_UART_StartReceive_IT();
}


/**
 * @brief 启动接收中断 (RXNE)
 */
void TOF_UART_StartReceive_IT(void)
{
    USART_ITConfig(TOF_UART_INSTANCE, USART_IT_RXNE, ENABLE);
}


/**
 * @brief 发送单字节 (阻塞式)
 */
void TOF_UART_SendByte(uint8_t data)
{
    while (USART_GetFlagStatus(TOF_UART_INSTANCE, USART_FLAG_TXE) == RESET);
    USART_SendData(TOF_UART_INSTANCE, data);
    while (USART_GetFlagStatus(TOF_UART_INSTANCE, USART_FLAG_TC) == RESET);
}

/**
 * @brief 发送数据块
 */
void TOF_UART_SendData(uint8_t *pData, uint16_t Size)
{
    for (uint16_t i = 0; i < Size; i++) {
        TOF_UART_SendByte(pData[i]);
    }
}

/**
 * @brief 发送字符串
 */
void TOF_UART_SendString(char *str)
{
    while (*str) {
        TOF_UART_SendByte((uint8_t)*str++);
    }
}

/**
 * @brief 接收中断处理 - 滑动窗口核心
 */
void TOF_UART_RxHandler(void)
{
    /* 处理RXNE中断 - 读取数据会自动清除RXNE标志 */
    if (USART_GetITStatus(TOF_UART_INSTANCE, USART_IT_RXNE) != RESET) {
        uint8_t data = (uint8_t)USART_ReceiveData(TOF_UART_INSTANCE);
        

        // printf("0x%x \n",data);
        /* 滑动窗口处理新数据 */
         SlideWindow_Process(data);
        // 清除中断标志
        USART_ClearITPendingBit(USART6, USART_IT_RXNE);
    }
    
    /* 处理其他错误标志（只读清除） */
    if (USART_GetFlagStatus(TOF_UART_INSTANCE, USART_FLAG_ORE) == SET) {
        (void)USART_ReceiveData(TOF_UART_INSTANCE);  /* 读DR清除ORE */
    }
    if (USART_GetFlagStatus(TOF_UART_INSTANCE, USART_FLAG_FE) == SET ||
        USART_GetFlagStatus(TOF_UART_INSTANCE, USART_FLAG_PE) == SET) {
        (void)USART_ReceiveData(TOF_UART_INSTANCE);  /* 读DR清除错误 */
    }
}

/**
 * @brief 检查是否有新的有效帧
 */
bool TOF_IsFrameReady(void)
{
    return frame_ready;
}

void TOF_SetFrameReady(bool value)
{
    frame_ready = value;
}

/**
 * @brief 获取处理后的帧数据
 * @param frame: 输出帧结构
 * @return: true=成功获取, false=无有效帧
 */



/* ==================== Modbus RTU 协议实现 ==================== */

/**
 * @brief 计算 Modbus CRC16
 */
uint16_t TOF_Modbus_CRC16(uint8_t *pData, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)pData[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/**
 * @brief 读取距离值 (功能码 0x03)
 * @note: 使用新的滑动窗口机制，发送命令后等待有效帧
 */
void TOF_ReadDistance(uint8_t slave_addr)
{
    uint8_t tx_buf[8];
    
    /* 构造读取命令: 01 03 00 10 00 01 85 CF */
    tx_buf[0] = slave_addr;
    tx_buf[1] = 0x03;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x10;
    tx_buf[4] = 0x00;
    tx_buf[5] = 0x01;
    
    uint16_t crc = TOF_Modbus_CRC16(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;
    
    
    /* 发送命令 */
    TOF_UART_SendData(tx_buf, 8);
    
    
    
    return ;
}

/**
 * @brief 设置测距模式
 */
void TOF_SetMode(uint8_t slave_addr, uint8_t mode)
{
    uint8_t tx_buf[8];
    
    tx_buf[0] = slave_addr;
    tx_buf[1] = 0x06;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x04;
    tx_buf[4] = 0x00;
    tx_buf[5] = mode;
    
    uint16_t crc = TOF_Modbus_CRC16(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;
    
    TOF_UART_SendData(tx_buf, 8);
    
    return ;
}

/**
 * @brief 设置自动输出周期
 */
void TOF_SetAutoOutput(uint8_t slave_addr, uint16_t period_ms)
{
    uint8_t tx_buf[8];
    
    tx_buf[0] = slave_addr;
    tx_buf[1] = 0x06;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x05;
    tx_buf[4] = (period_ms >> 8) & 0xFF;
    tx_buf[5] = period_ms & 0xFF;
    
    uint16_t crc = TOF_Modbus_CRC16(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;
    
    TOF_UART_SendData(tx_buf, 8);
    
    
    return ;
}



/**
 * @brief 设置系统滴答值（需在SysTick中断中调用）
 */
void TOF_SetSysTick(uint32_t tick)
{
    sys_tick_ms = tick;
}

/**
 * @brief UART6中断服务函数（在stm32f4xx_it.c中调用）
 */
void TOF_UART_IRQHandler(void)
{
    TOF_UART_RxHandler();
}

/**
 * @brief 测试函数
 */
void tof200f_test(void)
{
    TOF_UART_Init();
    TOF_SetAutoOutput(0x1,100);
    printf("TOF200F Test Start\r\n");
    
    /* 示例：配置为长距离模式 */
    // TOF_SetMode(TOF_DEFAULT_ADDR, 2);

    delay_ms(100);
    
    /* 循环读取距离 */
    while (1) {
        if (TOF_IsFrameReady()) {

            for (size_t i = 0; i < TOF_PHY_SIZE; i++)
            {
                if(current_frame[i].valid)
                {
                    printf("从机%d 距离: %d mm\r\n", current_frame[i].slave_addr, current_frame[i].distance);
                    current_frame[i].valid  = 0;
                }
            }
            frame_ready = 0;
        } 
        
        delay_ms(100);
    }
}
