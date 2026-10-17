/*****************************************************************************
* | 文件       	:   TCS34725.c
* | 功能    :   TCS34725 驱动程序 (STM32F407 标准库 I2C 版本)
* | 说明        :
*                Tcs34725 初始化、读取数据、写入数据 
                 以及数据处理

* | 日期        :   2026-03-13
* | 说明        :   适配 STM32F407 标准库 I2C

******************************************************************************/
#include "tcs34725.h"
#include <stdio.h>

/* 全局变量 */
static TCS34725IntegrationTime_t g_IntegrationTime = TCS34725_INTEGRATIONTIME_154MS;
static TCS34725Gain_t g_Gain = TCS34725_GAIN_60X;

/******************************************************************************
功能:   I2C 引脚和时钟初始化
参数	:
        无
说明    :   配置 I2C GPIO 和 I2C 外设，默认 100KHz 标准模式
******************************************************************************/
void TCS34725_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
    
    /* 使能 I2C 和 GPIO 时钟 */
    RCC_APB1PeriphClockCmd(TCS34725_I2C_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(TCS34725_I2C_GPIO_CLK, ENABLE);
    
    /* 配置 I2C SCL 和 SDA 引脚为复用开漏输出 */
    GPIO_InitStructure.GPIO_Pin = TCS34725_I2C_SCL_PIN | TCS34725_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(TCS34725_I2C_GPIO_PORT, &GPIO_InitStructure);
    
    /* 连接引脚到 I2C 复用功能 */
    GPIO_PinAFConfig(TCS34725_I2C_GPIO_PORT, TCS34725_I2C_SCL_SOURCE, TCS34725_I2C_AF);
    GPIO_PinAFConfig(TCS34725_I2C_GPIO_PORT, TCS34725_I2C_SDA_SOURCE, TCS34725_I2C_AF);
    
    /* I2C 配置 */
    I2C_InitStructure.I2C_ClockSpeed = 100000;              /* 100KHz 标准模式 */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;              /* I2C 模式 */
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;      /* 占空比 */
    I2C_InitStructure.I2C_OwnAddress1 = 0x01;               /* 本机地址 */
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;             /* 使能应答 */
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(TCS34725_I2C_PORT, &I2C_InitStructure);
    
    /* 使能 I2C */
    I2C_Cmd(TCS34725_I2C_PORT, ENABLE);
}

/******************************************************************************
功能:   I2C 等待事件
参数	:
        event : 等待的事件
返回值  :   0-成功, 1-超时
******************************************************************************/
static uint8_t TCS34725_I2C_WaitEvent(uint32_t event)
{
    uint32_t timeout = 10000;
    while (!I2C_CheckEvent(TCS34725_I2C_PORT, event))
    {
        if (--timeout == 0)
            return 1;
    }
    return 0;
}

/* 总线忙最多等待20ms；所有事件等待也有轮询上限，避免采摘任务永久阻塞。 */
static uint8_t TCS34725_I2C_WaitIdle(void)
{
    for (uint32_t waited = 0; waited < 20; waited++) {
        if (!I2C_GetFlagStatus(TCS34725_I2C_PORT, I2C_FLAG_BUSY)) return 0;
        SMART_DELAY_MS(1);
    }
    return 1;
}

/* 超时后释放事务并重新初始化本机I2C，下一轮采样可自动重试。 */
static void TCS34725_I2C_Abort(void)
{
    I2C_GenerateSTOP(TCS34725_I2C_PORT, ENABLE);
    I2C_AcknowledgeConfig(TCS34725_I2C_PORT, ENABLE);
    I2C_SoftwareResetCmd(TCS34725_I2C_PORT, ENABLE);
    I2C_SoftwareResetCmd(TCS34725_I2C_PORT, DISABLE);
    TCS34725_I2C_Init();
}

/******************************************************************************
功能:   向 TCS34725 寄存器写入一个字节
参数	:
        reg  : 寄存器地址
        data : 要写入的数据
返回值  :   0-成功, 1-失败
******************************************************************************/
uint8_t TCS34725_I2C_WriteByte(uint8_t reg, uint8_t data)
{
    /* 添加命令位 */
    reg = reg | TCS34725_CMD_BIT;

    if (TCS34725_I2C_WaitIdle()) goto failed;

    /* 发送起始条件 */
    I2C_GenerateSTART(TCS34725_I2C_PORT, ENABLE);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto failed;
    
    /* 发送设备地址 (写) */
    I2C_Send7bitAddress(TCS34725_I2C_PORT, TCS34725_ADDRESS, I2C_Direction_Transmitter);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto failed;
    
    /* 发送寄存器地址 */
    I2C_SendData(TCS34725_I2C_PORT, reg);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto failed;
    
    /* 发送数据 */
    I2C_SendData(TCS34725_I2C_PORT, data);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto failed;
    
    /* 发送停止条件 */
    I2C_GenerateSTOP(TCS34725_I2C_PORT, ENABLE);
    
    return 0;

failed:
    TCS34725_I2C_Abort();
    return 1;
}

/******************************************************************************
功能:   从 TCS34725 寄存器读取一个字节
参数	:
        reg  : 寄存器地址
        data : 读取到的数据指针
返回值  :   0-成功, 1-失败
******************************************************************************/
uint8_t TCS34725_I2C_ReadByte(uint8_t reg, uint8_t *data)
{
    /* 添加命令位 */
    reg = reg | TCS34725_CMD_BIT;

    if (TCS34725_I2C_WaitIdle()) goto failed;
    /* 使能应答 */
    I2C_AcknowledgeConfig(TCS34725_I2C_PORT, ENABLE);
    /* 发送起始条件 */
    I2C_GenerateSTART(TCS34725_I2C_PORT, ENABLE);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto failed;
    /* 发送设备地址 (写) */
    I2C_Send7bitAddress(TCS34725_I2C_PORT, TCS34725_ADDRESS, I2C_Direction_Transmitter);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto failed;

      I2C_Cmd(TCS34725_I2C_PORT, ENABLE);

    /* 发送寄存器地址 */
    I2C_SendData(TCS34725_I2C_PORT, reg);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto failed;
    
    /* 重新发送起始条件 */
    I2C_GenerateSTART(TCS34725_I2C_PORT, ENABLE);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto failed;
    
    /* 发送设备地址 (读) */
    I2C_Send7bitAddress(TCS34725_I2C_PORT, TCS34725_ADDRESS, I2C_Direction_Receiver);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto failed;
    
    /* 禁止应答并发送停止条件 */
    I2C_AcknowledgeConfig(TCS34725_I2C_PORT, DISABLE);
    I2C_GenerateSTOP(TCS34725_I2C_PORT, ENABLE);

    /* 等待数据接收 */
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto failed;
    *data = I2C_ReceiveData(TCS34725_I2C_PORT);
    
    return 0;

failed:
    TCS34725_I2C_Abort();
    return 1;
}

/******************************************************************************
功能:   从 TCS34725 寄存器读取一个字(2字节)
参数	:
        reg  : 寄存器地址
        data : 读取到的数据指针
返回值  :   0-成功, 1-失败
说明    :   使用 TCS34725_CMD_Read_Word 命令位自动读取两个字节
******************************************************************************/
uint8_t TCS34725_I2C_ReadWord(uint8_t reg, uint16_t *data)
{
    uint8_t high_byte, low_byte;
    
    /* 添加命令位 (字读取模式) */
    reg = reg | TCS34725_CMD_BIT | TCS34725_CMD_Read_Word;

    if (TCS34725_I2C_WaitIdle()) goto failed;

    /* 重新使能应答 */
    I2C_AcknowledgeConfig(TCS34725_I2C_PORT, ENABLE);

    /* 发送起始条件 */
    I2C_GenerateSTART(TCS34725_I2C_PORT, ENABLE);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto failed;
    
    /* 发送设备地址 (写) */
    I2C_Send7bitAddress(TCS34725_I2C_PORT, TCS34725_ADDRESS, I2C_Direction_Transmitter);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto failed;
    
    /* 发送寄存器地址 */
    I2C_SendData(TCS34725_I2C_PORT, reg);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto failed;
    
    /* 重新发送起始条件 */
    I2C_GenerateSTART(TCS34725_I2C_PORT, ENABLE);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto failed;
    
    /* 使能应答 */
    // I2C_AcknowledgeConfig(TCS34725_I2C_PORT, ENABLE);

    /* 发送设备地址 (读) */
    I2C_Send7bitAddress(TCS34725_I2C_PORT, TCS34725_ADDRESS, I2C_Direction_Receiver);
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto failed;
    
    
    /* 等待接收第一个字节 (低字节) */
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto failed;
    low_byte = I2C_ReceiveData(TCS34725_I2C_PORT);
    
    
    delay_us2(20); // 确保数据稳定，避免过早发送停止条件导致读取错误
    /* 禁止应答并发送停止条件 */
    I2C_AcknowledgeConfig(TCS34725_I2C_PORT, DISABLE);

    I2C_GenerateSTOP(TCS34725_I2C_PORT, ENABLE);
    
    /* 等待接收第二个字节 (高字节) */
    if (TCS34725_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto failed;
    high_byte = I2C_ReceiveData(TCS34725_I2C_PORT);
    
    
    /* 组合成16位数据 (小端模式) */
    *data = (uint16_t)(high_byte << 8) | low_byte;
    
    return 0;

failed:
    TCS34725_I2C_Abort();
    return 1;
}

/******************************************************************************
功能:   TCS34725 唤醒
说明    :   先使能振荡器，再使能 RGBC 转换
******************************************************************************/
void TCS34725_Enable(void)
{
    TCS34725_I2C_WriteByte(TCS34725_ENABLE, TCS34725_ENABLE_PON);
    SMART_DELAY_MS(3);
}


void TCS34725_StartADC(void)
{
    TCS34725_I2C_WriteByte(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    SMART_DELAY_MS(3);
}

/******************************************************************************
功能:   TCS34725 休眠
说明    :   关闭设备以节省功耗
******************************************************************************/
void TCS34725_Disable(void)
{
    uint8_t reg = 0;
    TCS34725_I2C_ReadByte(TCS34725_ENABLE, &reg);
    TCS34725_I2C_WriteByte(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

/******************************************************************************
功能:   设置积分时间
参数	:
        time : 积分时间，参考 TCS34725IntegrationTime_t 枚举
******************************************************************************/
void TCS34725_Set_IntegrationTime(TCS34725IntegrationTime_t time)
{
    TCS34725_I2C_WriteByte(TCS34725_ATIME, time);
    g_IntegrationTime = time;
}

/******************************************************************************
功能:   设置增益
参数	:
        gain : 增益，参考 TCS34725Gain_t 枚举
******************************************************************************/
void TCS34725_Set_Gain(TCS34725Gain_t gain)
{
    TCS34725_I2C_WriteByte(TCS34725_CONTROL, gain);
    g_Gain = gain;
}

/******************************************************************************
功能:   中断使能
******************************************************************************/
void TCS34725_Interrupt_Enable(void)
{
    uint8_t data = 0;
    TCS34725_I2C_ReadByte(TCS34725_ENABLE, &data);
    TCS34725_I2C_WriteByte(TCS34725_ENABLE, data | TCS34725_ENABLE_AIEN);
}

/******************************************************************************
功能:   中断禁止
******************************************************************************/
void TCS34725_Interrupt_Disable(void)
{
    uint8_t data = 0;
    TCS34725_I2C_ReadByte(TCS34725_ENABLE, &data);
    TCS34725_I2C_WriteByte(TCS34725_ENABLE, data & (~TCS34725_ENABLE_AIEN));
}

/******************************************************************************
功能:   设置中断持久寄存器
参数	:
        pers : 持久值，参考 TCS34725.h 中的定义
说明    :   中断需要保持多个周期才会触发
******************************************************************************/
void TCS34725_Set_Interrupt_Persistence_Reg(uint8_t pers)
{
    if (pers < 0x10)
        TCS34725_I2C_WriteByte(TCS34725_PERS, pers);
    else
        TCS34725_I2C_WriteByte(TCS34725_PERS, TCS34725_PERS_60_CYCLE);
}

/******************************************************************************
功能:   设置中断阈值
参数	:
        Threshold_H : 高中断阈值
        Threshold_L : 低中断阈值
说明    :   两个16位中断阈值寄存器允许用户在期望光照水平上下设置限制。
          当清除数据(CDATA)小于清除中断低阈值(AILTx)或大于清除中断高
          阈值(AIHTx)时可以产生中断
******************************************************************************/
void TCS34725_Set_Interrupt_Threshold(uint16_t Threshold_H, uint16_t Threshold_L)
{
    TCS34725_I2C_WriteByte(TCS34725_AILTL, Threshold_L & 0xFF);
    TCS34725_I2C_WriteByte(TCS34725_AILTH, Threshold_L >> 8);
    TCS34725_I2C_WriteByte(TCS34725_AIHTL, Threshold_H & 0xFF);
    TCS34725_I2C_WriteByte(TCS34725_AIHTH, Threshold_H >> 8);
}

/******************************************************************************
功能:   清除中断标志
******************************************************************************/
void TCS34725_Clear_Interrupt_Flag(void)
{
    TCS34725_I2C_WriteByte(TCS34725_CMD_Clear_INT, 0x00);
}

/******************************************************************************
功能:   TCS34725 初始化
参数	:
        无
返回值  :   0-成功, 1-失败
说明    :   初始化 I2C，检测设备ID，配置默认参数
******************************************************************************/
uint8_t TCS34725_Init(void)
{
    uint8_t ID = 0;
    
    /* 初始化 I2C */
    TCS34725_I2C_Init();
    
    /* 延时等待设备稳定 */
    SMART_DELAY_MS(1000);
    uint8_t ret = TCS34725_I2C_ReadByte(TCS34725_ID, &ID);
    /* 读取设备ID */
    if (ret != 0)
        return ret;
    
    /* 验证设备ID */
    if (ID != 0x44 && ID != 0x4D)
        return 1;
    
    printf("[颜色传感器000] TCS34725 设备ID: 0x%02X\n", ID);

    /* 使能设备 */
    TCS34725_Enable();
    /* 设置积分时间和增益 */
    TCS34725_Set_IntegrationTime(TCS34725_INTEGRATIONTIME_154MS);
    TCS34725_Set_Gain(TCS34725_GAIN_60X);
    
    g_IntegrationTime = TCS34725_INTEGRATIONTIME_154MS;
    g_Gain = TCS34725_GAIN_60X;
    
    /* 设置中断阈值 */
    TCS34725_Set_Interrupt_Threshold(0xFF00, 0x00FF);
    TCS34725_Set_Interrupt_Persistence_Reg(TCS34725_PERS_2_CYCLE);
    
    TCS34725_StartADC();
    
    /* 使能中断 */
    TCS34725_Interrupt_Enable();
    
    /* 设置板载LED亮度 */
    TCS34725_SetLight(40);
    
    return 0;
}

/******************************************************************************
功能:   读取 RGBC 数据
参数	:
        无
返回值  :   RGB 结构体，包含 R,G,B,C 四个通道的值
******************************************************************************/
/* 只有四个通道全部读取成功才提交数据，禁止将失败/半帧误判为无红椒。 */
uint8_t TCS34725_TryReadLatest_RGBData(RGB *out)
{
    RGB temp = {0};
    if (TCS34725_I2C_ReadWord(TCS34725_CDATAL | TCS34725_CMD_Read_Word, &temp.C) ||
        TCS34725_I2C_ReadWord(TCS34725_RDATAL | TCS34725_CMD_Read_Word, &temp.R) ||
        TCS34725_I2C_ReadWord(TCS34725_GDATAL | TCS34725_CMD_Read_Word, &temp.G) ||
        TCS34725_I2C_ReadWord(TCS34725_BDATAL | TCS34725_CMD_Read_Word, &temp.B)) return 1;
    *out = temp;
    return 0;
}

RGB TCS34725_ReadLatest_RGBData(void)
{
    RGB temp = {0};
    TCS34725_TryReadLatest_RGBData(&temp);
    return temp;
}

/* 保留原有调用者的等待行为；采摘任务使用上面的即时读取接口。 */
RGB TCS34725_Get_RGBData(void)
{
    RGB temp = TCS34725_ReadLatest_RGBData();
    /* 根据积分时间延时等待数据稳定 */
    switch (g_IntegrationTime)
    {
        case TCS34725_INTEGRATIONTIME_2_4MS:
            SMART_DELAY_MS(3);
            break;
        case TCS34725_INTEGRATIONTIME_24MS:
            SMART_DELAY_MS(24);
            break;
        case TCS34725_INTEGRATIONTIME_50MS:
            SMART_DELAY_MS(50);
            break;
        case TCS34725_INTEGRATIONTIME_101MS:
            SMART_DELAY_MS(101);
            break;
        case TCS34725_INTEGRATIONTIME_154MS:
            SMART_DELAY_MS(154);
            break;
        case TCS34725_INTEGRATIONTIME_700MS:
            SMART_DELAY_MS(700);
            break;
        default:
            break;
    }
    
    return temp;
}

/******************************************************************************
功能:   检测光照中断
参数	:
        Threshold_H : 高中断阈值
        Threshold_L : 低中断阈值
返回值  :   1-触发中断, 0-未触发
******************************************************************************/
uint8_t TCS34725_GetLux_Interrupt(uint16_t Threshold_H, uint16_t Threshold_L)
{
    TCS34725_Set_Interrupt_Threshold(Threshold_H, Threshold_L);
    
    /* 读取中断引脚状态 (需要配置外部中断) */
    /* 这里简单读取状态寄存器 */
    uint8_t status = 0;
    TCS34725_I2C_ReadByte(TCS34725_STATUS, &status);
    
    if (status & TCS34725_STATUS_AINT)
    {
        TCS34725_Clear_Interrupt_Flag();
        TCS34725_Set_Interrupt_Persistence_Reg(TCS34725_PERS_2_CYCLE);
        return 1;
    }
    return 0;
}

/******************************************************************************
功能:   计算色温
参数	:
        rgb : RGBC 数据结构
返回值  :   色温值 (单位: 开尔文)
******************************************************************************/
uint16_t TCS34725_Get_ColorTemp(RGB rgb)
{
    float cct;
    uint16_t r_comp, b_comp, ir;
    
    ir = (rgb.R + rgb.G + rgb.B > rgb.C) ? (rgb.R + rgb.G + rgb.B - rgb.C) / 2 : 0;
    r_comp = rgb.R - ir;
    b_comp = rgb.B - ir;
    
    if (r_comp == 0)
        r_comp = 1;
    
    cct = TCS34725_CT_Coef * (float)(b_comp) / (float)(r_comp) + TCS34725_CT_Offset;
    
    if (cct < 0)
        cct = 0;
    if (cct > 65535)
        cct = 65535;
    
    return (uint16_t)cct;
}

/******************************************************************************
功能:   计算光照强度
参数	:
        rgb : RGBC 数据结构
返回值  :   光照强度值 (单位: Lux)
******************************************************************************/
uint16_t TCS34725_Get_Lux(RGB rgb)
{
    float lux, cpl, atime_ms, gain_temp = 1.0f;
    uint16_t ir = 0;
    uint16_t r_comp, g_comp, b_comp;
    
    atime_ms = ((256 - g_IntegrationTime) * 2.4f);
    
    ir = (rgb.R + rgb.G + rgb.B > rgb.C) ? (rgb.R + rgb.G + rgb.B - rgb.C) / 2 : 0;
    r_comp = rgb.R - ir;
    g_comp = rgb.G - ir;
    b_comp = rgb.B - ir;
    
    switch (g_Gain)
    {
        case TCS34725_GAIN_1X:
            gain_temp = 1.0f;
            break;
        case TCS34725_GAIN_4X:
            gain_temp = 4.0f;
            break;
        case TCS34725_GAIN_16X:
            gain_temp = 16.0f;
            break;
        case TCS34725_GAIN_60X:
            gain_temp = 60.0f;
            break;
        default:
            gain_temp = 1.0f;
            break;
    }
    
    cpl = (atime_ms * gain_temp) / (TCS34725_GA * TCS34725_DF);
    
    if (cpl == 0)
        cpl = 1.0f;
    
    lux = (TCS34725_R_Coef * (float)(r_comp) + 
           TCS34725_G_Coef * (float)(g_comp) + 
           TCS34725_B_Coef * (float)(b_comp)) / cpl;
    
    if (lux < 0)
        lux = 0;
    if (lux > 65535)
        lux = 65535;
    
    return (uint16_t)lux;
}

/******************************************************************************
功能:   将原始RGB值转换为RGB888格式
参数	:
        rgb : RGBC 数据结构
返回值  :   32位RGB888颜色值 (0x00RRGGBB)
******************************************************************************/
uint32_t TCS34725_GetRGB888(RGB rgb)
{
    float i = 1.0f;
    
    /* 限制数据范围 */
    if (rgb.R >= rgb.G && rgb.R >= rgb.B)
        i = rgb.R / 255.0f + 1.0f;
    else if (rgb.G >= rgb.R && rgb.G >= rgb.B)
        i = rgb.G / 255.0f + 1.0f;
    else if (rgb.B >= rgb.G && rgb.B >= rgb.R)
        i = rgb.B / 255.0f + 1.0f;
    
    if (i != 0)
    {
        rgb.R = (uint16_t)(rgb.R / i);
        rgb.G = (uint16_t)(rgb.G / i);
        rgb.B = (uint16_t)(rgb.B / i);
    }
    
    /* 放大数据差异 */
    if (rgb.R > 30)
        rgb.R = rgb.R - 30;
    if (rgb.G > 30)
        rgb.G = rgb.G - 30;
    if (rgb.B > 30)
        rgb.B = rgb.B - 30;
    
    rgb.R = rgb.R * 255 / 225;
    rgb.G = rgb.G * 255 / 225;
    rgb.B = rgb.B * 255 / 225;
    
    if (rgb.R > 255)
        rgb.R = 255;
    if (rgb.G > 255)
        rgb.G = 255;
    if (rgb.B > 255)
        rgb.B = 255;
    
    printf("RGB888: R=%d G=%d B=%d\r\n", rgb.R, rgb.G, rgb.B);

    return ((uint32_t)rgb.R << 16) | ((uint32_t)rgb.G << 8) | rgb.B;
}

/******************************************************************************
功能:   将原始RGB值转换为RGB565格式
参数	:
        rgb : RGBC 数据结构
返回值  :   16位RGB565颜色值
******************************************************************************/
uint16_t TCS34725_GetRGB565(RGB rgb)
{
    float i = 1.0f;
    
    /* 限制数据范围 */
    if (rgb.R >= rgb.G && rgb.R >= rgb.B)
        i = rgb.R / 255.0f + 1.0f;
    else if (rgb.G >= rgb.R && rgb.G >= rgb.B)
        i = rgb.G / 255.0f + 1.0f;
    else if (rgb.B >= rgb.G && rgb.B >= rgb.R)
        i = rgb.B / 255.0f + 1.0f;
    
    if (i != 0)
    {
        rgb.R = (uint16_t)(rgb.R / i);
        rgb.G = (uint16_t)(rgb.G / i);
        rgb.B = (uint16_t)(rgb.B / i);
    }
    
    if (rgb.R > 30)
        rgb.R = rgb.R - 30;
    if (rgb.G > 30)
        rgb.G = rgb.G - 30;
    if (rgb.B > 30)
        rgb.B = rgb.B - 30;
    
    rgb.R = rgb.R * 255 / 225;
    rgb.G = rgb.G * 255 / 225;
    rgb.B = rgb.B * 255 / 225;
    
    if (rgb.R > 255)
        rgb.R = 255;
    if (rgb.G > 255)
        rgb.G = 255;
    if (rgb.B > 255)
        rgb.B = 255;
    
    /* 转换为RGB565: R(5bit) G(6bit) B(5bit) */
    return ((rgb.R & 0xF8) << 8) | ((rgb.G & 0xFC) << 3) | ((rgb.B & 0xF8) >> 3);
}

/******************************************************************************
功能:   设置板载LED亮度
参数	:
        value : 亮度值 0-100
说明    :   此函数为占位函数，需要根据实际硬件连接实现PWM控制
******************************************************************************/
void TCS34725_SetLight(uint16_t value)
{
    /* 用户需要根据实际硬件实现PWM控制 */
    /* 例如: 使用 TIM PWM 控制 LED */
    if (value > 100)
        value = 100;
    
    /* 占位: 实际使用时需要实现PWM输出 */
    (void)value;
}

/******************************************************************************
功能:   TCS34725 测试函数
说明    :   循环读取RGB数据并通过串口打印
******************************************************************************/
void TCS34725_Test(void)
{
    RGB rgb;
    uint32_t rgb888;
    uint16_t rgb565;
    uint16_t lux;
    uint16_t cct;
    
    if (TCS34725_Init() != 0)
    {
        printf("TCS34725 初始化错误!!\r\n");
        return;
    }
    printf("TCS34725 初始化成功!!\r\n");
    
    while (1)
    {
        rgb = TCS34725_Get_RGBData();
        rgb888 = TCS34725_GetRGB888(rgb);
        rgb565 = TCS34725_GetRGB565(rgb);
        lux = TCS34725_Get_Lux(rgb);
        cct = TCS34725_Get_ColorTemp(rgb);
        
        printf("R=%d G=%d B=%d C=%d\r\n", rgb.R, rgb.G, rgb.B, rgb.C);
        printf("RGB888=0x%06X RGB565=0x%04X\r\n", rgb888, rgb565);
        printf("Lux=%d CCT=%dK\r\n", lux, cct);
        
        if (TCS34725_GetLux_Interrupt(0xFF00, 0x0CFF) == 1)
            printf("Lux_Interrupt = 1\r\n");
        else
            printf("Lux_Interrupt = 0\r\n");
        
        printf("\r\n");
        
        SMART_DELAY_MS(500);
    }
}
