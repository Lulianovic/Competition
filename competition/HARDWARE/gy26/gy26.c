/**
 * GY-26电子指南针STM32F407标准库UART4驱动
 * 文件名：gy26_compass_std.c
 */

#include "gy26.h"
#include "misc.h"
#include <string.h>
#include <math.h>

// 全局变量
GY26_CompassTypeDef gy26_compass_instance;

// 私有函数声明
static uint8_t GY26_CalculateChecksum(uint8_t *data, uint8_t len);
static float GY26_ParseAngle(uint8_t *data);
static uint8_t GY26_ParseCalibrationLevel(uint8_t *data);
static void GY26_UART4_Init(void);
static void GY26_UART4_SendByte(uint8_t byte);
static void GY26_UART4_SendBytes(uint8_t *data, uint16_t len);

/**
 * @brief  初始化GY-26指南针模块
 * @param  compass: 指南针结构体指针
 * @retval GY26状态
 */
GY26_StatusTypeDef GY26_Init(void)
{
    // 初始化结构体
    gy26_compass_instance.angle = 0.0f;
    gy26_compass_instance.declination = 0.0f;
    gy26_compass_instance.calibration_level = 0;
    gy26_compass_instance.is_calibrating = false;
    gy26_compass_instance.rx_index = 0;
    gy26_compass_instance.last_receive_time = 0;
    // 清空接收缓冲区
    memset(gy26_compass_instance.rx_buffer, 0, GY26_FRAME_SIZE);
    
    // 初始化UART4
    GY26_UART4_Init();
	
    
    return GY26_OK;
}

/**
 * @brief  测量角度（发送测量命令）
 * @retval GY26状态
 */
GY26_StatusTypeDef GY26_MeasureAngle(void)
{
    // 发送测量角度命令
    GY26_UART4_SendByte(GY26_CMD_MEASURE_ANGLE);
    return GY26_OK;
}

/**
 * @brief  开始校准
 * @retval GY26状态
 */
GY26_StatusTypeDef GY26_StartCalibration(void)
{
    // 发送开始校准命令
    GY26_UART4_SendByte(GY26_CMD_CALIBRATE_START);
    
    gy26_compass_instance.is_calibrating = true;
    
    return GY26_OK;
}

/**
 * @brief  停止校准
 * @retval GY26状态
 */
GY26_StatusTypeDef GY26_StopCalibration(void)
{
    // 发送停止校准命令
    GY26_UART4_SendByte(GY26_CMD_CALIBRATE_STOP);
    
    gy26_compass_instance.is_calibrating = false;
    
    return GY26_OK;
}

/**
 * @brief  设置磁偏角
 * @param  declination: 磁偏角值（度）
 * @retval GY26状态
 */
GY26_StatusTypeDef GY26_SetDeclination(float declination)
{
    // 确保角度在0-359.9范围内
    if (declination < 0) declination += 360.0f;
    if (declination >= 360.0f) declination -= 360.0f;
    
    // 转换为0.1度单位的整数值（0-3599）
    uint16_t declination_10x = (uint16_t)(declination * 10.0f);
    
    // 高8位和低8位
    uint8_t high_byte = (declination_10x >> 8) & 0xFF;
    uint8_t low_byte = declination_10x & 0xFF;
    
    // 根据手册，磁偏角设置命令格式：0x03 + 高8位，0x04 + 低8位
    uint8_t cmd_high[2] = {0x03, high_byte};
    uint8_t cmd_low[2] = {0x04, low_byte};
    
    // 发送高字节命令
    GY26_UART4_SendBytes(cmd_high, 2);
    
    // 延时等待模块响应
    for(volatile int i = 0; i < 1000; i++);  // 简单延时
    
    // 发送低字节命令
    GY26_UART4_SendBytes(cmd_low, 2);
    
    // 更新本地存储的磁偏角值
    gy26_compass_instance.declination = declination;
    
    return GY26_OK;
}

/**
 * @brief  恢复出厂设置
 * @retval GY26状态
 */
GY26_StatusTypeDef GY26_FactoryReset(void)
{
    // 恢复出厂设置命令序列
    uint8_t cmd_sequence[4] = {0xA0, 0xAA, 0xA5, 0xC5};
    
    GY26_UART4_SendBytes(cmd_sequence, 4);
    
    return GY26_OK;
}

/**
 * @brief  UART4中断服务函数
 */
void GY26_UART_IRQHandler(void)
{
    // 检查是否接收到数据
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        // 读取接收到的字节
        uint8_t byte = USART_ReceiveData(UART4);
        
        // 检查是否正在接收一帧数据
        if (gy26_compass_instance.rx_index == 0)
        {
            // 寻找帧头
            if (byte == GY26_HEADER1)
            {
                gy26_compass_instance.rx_buffer[gy26_compass_instance.rx_index++] = byte;
            }
        }
        else if (gy26_compass_instance.rx_index == 1)
        {
            // 检查第二个字节是否为0x0A
            if (byte == GY26_HEADER2)
            {
                gy26_compass_instance.rx_buffer[gy26_compass_instance.rx_index++] = byte;
            }
            else
            {
                // 如果不是，重新开始寻找帧头
                gy26_compass_instance.rx_index = 0;
            }
        }
        else
        {
            // 接收数据字节
            gy26_compass_instance.rx_buffer[gy26_compass_instance.rx_index++] = byte;
            
            // 检查是否接收完一帧
            if (gy26_compass_instance.rx_index >= GY26_FRAME_SIZE)
            {
                // 处理完整的数据帧
                GY26_ProcessUartData();
                
                // 重置接收索引
                gy26_compass_instance.rx_index = 0;
            }
        }
        
        // 更新最后接收时间
        gy26_compass_instance.last_receive_time = 0; // 可以设置为系统tick，需要SysTick配置
        
        // 清除中断标志
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
}

/**
 * @brief  处理接收到的UART数据
 * @param  compass: 指南针结构体指针
 * @retval GY26状态
 */
GY26_StatusTypeDef GY26_ProcessUartData(void)
{
    
    // 验证帧头
    if (gy26_compass_instance.rx_buffer[0] != GY26_HEADER1 || 
        gy26_compass_instance.rx_buffer[1] != GY26_HEADER2)
    {
        return GY26_ERROR_FRAME;
    }
    
    // 验证校验和
    uint8_t calculated_checksum = GY26_CalculateChecksum(gy26_compass_instance.rx_buffer, GY26_FRAME_SIZE - 1);
    if (calculated_checksum != gy26_compass_instance.rx_buffer[GY26_FRAME_SIZE - 1])
    {
        return GY26_ERROR_CHECKSUM;
    }
    
    // 检查是否是校准等级数据
    if (gy26_compass_instance.rx_buffer[2] == 0x30 &&  // '0'
        gy26_compass_instance.rx_buffer[3] == 0x30 &&  // '0'
        gy26_compass_instance.rx_buffer[4] == 0x30 &&  // '0'
       gy26_compass_instance.rx_buffer[5] == 0x2E)    // '.'
    {
        // 解析校准等级
        gy26_compass_instance.calibration_level = GY26_ParseCalibrationLevel(gy26_compass_instance.rx_buffer);
    }
    else
    {
        // 解析角度值
        gy26_compass_instance.angle = GY26_ParseAngle(gy26_compass_instance.rx_buffer);
    }
    
    return GY26_OK;
}

/**
 * @brief  计算校验和
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval 校验和（低8位）
 */
static uint8_t GY26_CalculateChecksum(uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;
    
    for (uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
    
    return (uint8_t)(sum & 0xFF);  // 仅取低8位
}

/**
 * @brief  解析角度值
 * @param  data: 数据帧指针
 * @retval 角度值（度）
 */
static float GY26_ParseAngle(uint8_t *data)
{
    // 数据格式：百位(0-3) 十位(0-9) 个位(0-9) 小数点(0x2E) 小数位(0-9)
    
    // 将ASCII字符转换为数字
    uint8_t hundreds = data[2] - '0';  // 百位 (0-3)
    uint8_t tens = data[3] - '0';      // 十位 (0-9)
    uint8_t units = data[4] - '0';     // 个位 (0-9)
    uint8_t decimal = data[6] - '0';   // 小数位 (0-9)
    
    // 计算角度值
    float angle = hundreds * 100.0f + tens * 10.0f + units + decimal * 0.1f;
    
    return angle;
}

/**
 * @brief  解析校准等级
 * @param  data: 数据帧指针
 * @retval 校准等级 (0-9)
 */
static uint8_t GY26_ParseCalibrationLevel(uint8_t *data)
{
    // 校准等级在第6字节（索引6）
    uint8_t level = data[6] - '0';
    
    // 确保等级在0-9范围内
    if (level > 9) level = 0;
    
    return level;
}

/**
 * @brief  获取当前角度
 * @retval 当前角度
 */
float GY26_GetAngle(void)
{ 
	return gy26_compass_instance.angle;
}

/**
 * @brief  获取校准等级
 * @retval 校准等级
 */
uint8_t GY26_GetCalibrationLevel(void)
{
    return gy26_compass_instance.calibration_level;
}


/**
 * @brief  初始化UART4 (标准库)
 */
static void GY26_UART4_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 1. 使能外设时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    
    // 2. 配置UART4引脚
    // PC10 -> UART4_TX
    // PC11 -> UART4_RX
    
    // 配置TX引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // 配置RX引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // 连接GPIO到UART4
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
    
    // 3. 配置UART4参数
    USART_InitStructure.USART_BaudRate = 9600;  // 波特率9600
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 8位数据位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  // 1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;  // 无奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // 收发模式
    
    USART_Init(UART4, &USART_InitStructure);
    
    // 4. 配置中断
    // 使能接收中断
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    
    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 5. 使能UART4
    USART_Cmd(UART4, ENABLE);
}

/**
 * @brief  UART4发送单个字节
 * @param  byte: 要发送的字节
 */
static void GY26_UART4_SendByte(uint8_t byte)
{
    // 等待发送缓冲区空
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
    
    // 发送数据
    USART_SendData(UART4, byte);
    
    // 等待发送完成
    while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
}

/**
 * @brief  UART4发送多个字节
 * @param  data: 数据指针
 * @param  len: 数据长度
 */
static void GY26_UART4_SendBytes(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        GY26_UART4_SendByte(data[i]);
    }
}


/**
 * UART4中断服务函数（在stm32f4xx_it.c中）
 */
void UART4_IRQHandler(void)
{
    // 调用GY-26的中断处理函数
    GY26_UART_IRQHandler();
}


/**
 * 校准流程示例函数
 */
void PerformCalibration(void)
{
    // 1. 开始校准
    GY26_StartCalibration();
    
    // 2. 提示用户缓慢旋转模块一周（大约1分钟）
    // 这里可以点亮LED或通过串口输出提示信息
    
    // 3. 等待60秒（用户旋转模块）
    //Delay_ms(60000);
    
    // 4. 停止校准
    GY26_StopCalibration();
    
    // 5. 检查校准等级
    uint8_t cal_level = GY26_GetCalibrationLevel();
    
    // 6. 根据校准等级判断校准结果
    if (cal_level >= 5)
    {
        // 校准成功，等级较好
        // 可以保存校准参数（模块内部已自动保存）
    }
    else
    {
        // 校准等级较低，可能需要重新校准
    }
}

