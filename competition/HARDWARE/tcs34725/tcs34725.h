/*****************************************************************************
* | 文件       	:   TCS34725.h
* | 功能    :   TCS34725 驱动程序 (STM32F407 标准库 I2C 版本)
* | 说明        :
*                Tcs34725 初始化、读取数据、写入数据 
                 以及数据处理
* | 日期        :   2026-03-13
* | 说明        :   适配 STM32F407 标准库 I2C

******************************************************************************/
#ifndef __TCS34725_H
#define __TCS34725_H

#include "stm32f4xx.h"
#include <stdint.h>
#include "delay.h"

#define TCS34725_I2C_SDA_SOURCE_Read GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_0)
/*============================ I2C 配置 ============================*/
/* 选择使用的 I2C 端口: I2C1 或 I2C2 或 I2C3 */
#define TCS34725_I2C_PORT       I2C2
#define TCS34725_I2C_CLK        RCC_APB1Periph_I2C2

/* I2C 引脚配置 (根据实际硬件修改) */
/* I2C2 使用 PF1(SCL), PF0(SDA) */
#define TCS34725_I2C_GPIO_CLK   RCC_AHB1Periph_GPIOF
#define TCS34725_I2C_GPIO_PORT  GPIOF
#define TCS34725_I2C_SCL_PIN    GPIO_Pin_1
#define TCS34725_I2C_SDA_PIN    GPIO_Pin_0
#define TCS34725_I2C_SCL_SOURCE GPIO_PinSource1
#define TCS34725_I2C_SDA_SOURCE GPIO_PinSource0
#define TCS34725_I2C_AF         GPIO_AF_I2C2

/* 中断引脚配置 (可选) */
#define TCS34725_INT_GPIO_CLK   RCC_AHB1Periph_GPIOB
#define TCS34725_INT_GPIO_PORT  GPIOB
#define TCS34725_INT_PIN        GPIO_Pin_0
#define TCS34725_INT_PIN_SOURCE EXTI_PinSource0
#define TCS34725_INT_EXTI_LINE  EXTI_Line0
#define TCS34725_INT_IRQ        EXTI0_IRQn

/*============================ 设备地址 ============================*/
/* I2C 7位地址 0x29, 8位地址 0x52 */
#define TCS34725_ADDRESS          (0x29 << 1)

/*============================ 寄存器定义 ============================*/
#define TCS34725_CMD_BIT          0x80
#define TCS34725_CMD_Read_Byte    0x00
#define TCS34725_CMD_Read_Word    0x20
#define TCS34725_CMD_Clear_INT    0x66     /* RGBC 中断标志清除 */

#define TCS34725_ENABLE           0x00     
#define TCS34725_ENABLE_AIEN      0x10     /* RGBC 中断使能 */
#define TCS34725_ENABLE_WEN       0x08     /* 等待使能 - 写入1激活等待定时器 */
#define TCS34725_ENABLE_AEN       0x02     /* RGBC 使能 - 写入1激活ADC, 0禁止 */
#define TCS34725_ENABLE_PON       0x01     /* 上电 - 写入1激活内部振荡器, 0禁止 */

#define TCS34725_ATIME            0x01     /* 积分时间 */
#define TCS34725_WTIME            0x03     /* 等待时间 (如果 TCS34725_ENABLE_WEN 被置位) */
#define TCS34725_WTIME_2_4MS      0xFF     /* WLONG0 = 2.4ms   WLONG1 = 0.029s */
#define TCS34725_WTIME_204MS      0xAB     /* WLONG0 = 204ms   WLONG1 = 2.45s  */
#define TCS34725_WTIME_614MS      0x00     /* WLONG0 = 614ms   WLONG1 = 7.4s   */

#define TCS34725_AILTL            0x04     /* 清除通道低中断阈值 */
#define TCS34725_AILTH            0x05
#define TCS34725_AIHTL            0x06     /* 清除通道高中断阈值 */
#define TCS34725_AIHTH            0x07

#define TCS34725_PERS             0x0C     /* 持久寄存器 - 中断的基本软件滤波机制 */
#define TCS34725_PERS_NONE        0x00     /* 每个RGBC周期产生一个中断 */
#define TCS34725_PERS_1_CYCLE     0x01     /* 1个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_2_CYCLE     0x02     /* 2个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_3_CYCLE     0x03     /* 3个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_5_CYCLE     0x04     /* 5个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_10_CYCLE    0x05     /* 10个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_15_CYCLE    0x06     /* 15个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_20_CYCLE    0x07     /* 20个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_25_CYCLE    0x08     /* 25个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_30_CYCLE    0x09     /* 30个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_35_CYCLE    0x0A     /* 35个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_40_CYCLE    0x0B     /* 40个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_45_CYCLE    0x0C     /* 45个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_50_CYCLE    0x0D     /* 50个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_55_CYCLE    0x0E     /* 55个清除通道值超出阈值范围产生中断 */
#define TCS34725_PERS_60_CYCLE    0x0F     /* 60个清除通道值超出阈值范围产生中断 */

#define TCS34725_CONFIG           0x0D
#define TCS34725_CONFIG_WLONG     0x02     /* 通过 TCS34725_WTIME 选择短或长(12倍)等待时间 */

#define TCS34725_CONTROL          0x0F     /* 设置传感器的增益级别 */
#define TCS34725_ID               0x12     /* 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */

#define TCS34725_STATUS           0x13
#define TCS34725_STATUS_AINT      0x10     /* RGBC 清除通道中断 */
#define TCS34725_STATUS_AVALID    0x01     /* 表示RGBC通道已完成一个积分周期 */

#define TCS34725_CDATAL           0x14     /* 清除通道数据 */
#define TCS34725_CDATAH           0x15
#define TCS34725_RDATAL           0x16     /* 红色通道数据 */
#define TCS34725_RDATAH           0x17
#define TCS34725_GDATAL           0x18     /* 绿色通道数据 */
#define TCS34725_GDATAH           0x19
#define TCS34725_BDATAL           0x1A     /* 蓝色通道数据 */
#define TCS34725_BDATAH           0x1B

/*============================ 系数定义 ============================*/
#define TCS34725_R_Coef           0.136 
#define TCS34725_G_Coef           1.000
#define TCS34725_B_Coef          -0.444
#define TCS34725_GA               1.0
#define TCS34725_DF               310.0
#define TCS34725_CT_Coef          3810.0
#define TCS34725_CT_Offset        1391.0

/*============================ 积分时间枚举 ============================*/
typedef enum
{
    TCS34725_INTEGRATIONTIME_2_4MS  = 0xFF,   /**<  2.4ms - 1 周期    - 最大计数: 1024  */
    TCS34725_INTEGRATIONTIME_24MS   = 0xF6,   /**<  24ms  - 10 周期   - 最大计数: 10240 */
    TCS34725_INTEGRATIONTIME_50MS   = 0xEB,   /**<  50ms  - 20 周期   - 最大计数: 20480 */
    TCS34725_INTEGRATIONTIME_101MS  = 0xD5,   /**<  101ms - 42 周期   - 最大计数: 43008 */
    TCS34725_INTEGRATIONTIME_154MS  = 0xC0,   /**<  154ms - 64 周期   - 最大计数: 65535 */
    TCS34725_INTEGRATIONTIME_700MS  = 0x00    /**<  700ms - 256 周期  - 最大计数: 65535 */
} TCS34725IntegrationTime_t;

/*============================ 增益枚举 ============================*/
typedef enum
{
    TCS34725_GAIN_1X                = 0x00,   /**<  无增益   */
    TCS34725_GAIN_4X                = 0x01,   /**<  4倍增益  */
    TCS34725_GAIN_16X               = 0x02,   /**<  16倍增益 */
    TCS34725_GAIN_60X               = 0x03    /**<  60倍增益 */
} TCS34725Gain_t;

/*============================ RGB 数据结构 ============================*/
typedef struct
{
    uint16_t R;
    uint16_t G;
    uint16_t B;
    uint16_t C;
} RGB;

/*============================ 标准RGB数据结构 (0-255范围) ============================*/
typedef struct
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_Standard;

/*============================ 函数声明 ============================*/

/* I2C 底层接口 */
void TCS34725_I2C_Init(void);
uint8_t TCS34725_I2C_WriteByte(uint8_t reg, uint8_t data);
uint8_t TCS34725_I2C_ReadByte(uint8_t reg, uint8_t *data);
uint8_t TCS34725_I2C_ReadWord(uint8_t reg, uint16_t *data);

/* 延时函数 */
void TCS34725_Delay_ms(uint32_t ms);
void TCS34725_Delay_us(uint32_t us);

/* 设备控制 */
uint8_t TCS34725_Init(void);
void TCS34725_Enable(void);
void TCS34725_StartADC(void);
void TCS34725_Disable(void);
void TCS34725_Set_Gain(TCS34725Gain_t gain);
void TCS34725_Set_IntegrationTime(TCS34725IntegrationTime_t it);

/* 数据读取 */
RGB TCS34725_Get_RGBData(void);
/* 读取当前RGBC寄存器，不在读取后等待积分；采样任务自行调度。
 * 重复轮询可能返回同一积分周期的数据，不表示新的传感器帧。 */
RGB TCS34725_ReadLatest_RGBData(void);
/* 返回0表示四通道读取成功；失败返回1且不修改输出。 */
uint8_t TCS34725_TryReadLatest_RGBData(RGB *out);
uint16_t TCS34725_Get_ColorTemp(RGB rgb);
uint16_t TCS34725_GetRGB565(RGB rgb);
uint32_t TCS34725_GetRGB888(RGB rgb);
uint16_t TCS34725_Get_Lux(RGB rgb);


/* 中断控制 */
void TCS34725_Interrupt_Enable(void);
void TCS34725_Interrupt_Disable(void);
void TCS34725_Set_Interrupt_Threshold(uint16_t Threshold_H, uint16_t Threshold_L);
void TCS34725_Set_Interrupt_Persistence_Reg(uint8_t TCS34725_PER);
void TCS34725_Clear_Interrupt_Flag(void);
uint8_t TCS34725_GetLux_Interrupt(uint16_t Threshold_H, uint16_t Threshold_L);

/* 板载LED控制 */
void TCS34725_SetLight(uint16_t value);

/* 测试函数 */
void TCS34725_Test(void);

#endif /* __TCS34725_H */
