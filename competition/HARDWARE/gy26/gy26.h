/**
 * GY-26电子指南针STM32F407标准库UART4驱动
 * 文件名：gy26_compass_std.h
 */

#ifndef GY26_COMPASS_STD_H
#define GY26_COMPASS_STD_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

// 定义模块命令
#define GY26_CMD_MEASURE_ANGLE     0x31    // 进行一次角度测量
#define GY26_CMD_CALIBRATE_START   0xC0    // 开始校准
#define GY26_CMD_CALIBRATE_STOP    0xC1    // 结束校准

// 数据帧结构
#define GY26_FRAME_SIZE            8       // 每帧8字节
#define GY26_HEADER1               0x0D    // 回车
#define GY26_HEADER2               0x0A    // 换行

// 错误代码
typedef enum {
    GY26_OK = 0,
    GY26_ERROR_UART,
    GY26_ERROR_CHECKSUM,
    GY26_ERROR_TIMEOUT,
    GY26_ERROR_FRAME,
    GY26_ERROR_CALIBRATION
} GY26_StatusTypeDef;

// 指南针主结构体
typedef struct {
    float angle;                    // 当前角度 (0.0-359.9度)
    float declination;              // 磁偏角 (0.0-359.9度)
    uint8_t calibration_level;      // 校准等级
    bool is_calibrating;           // 是否正在校准
    uint8_t rx_buffer[GY26_FRAME_SIZE];  // 接收缓冲区
    uint8_t rx_index;               // 接收索引
    uint32_t last_receive_time;     // 最后接收时间
} GY26_CompassTypeDef;

// 函数声明
GY26_StatusTypeDef GY26_Init(void);
GY26_StatusTypeDef GY26_MeasureAngle(void);
GY26_StatusTypeDef GY26_StartCalibration(void);
GY26_StatusTypeDef GY26_StopCalibration(void);
GY26_StatusTypeDef GY26_SetDeclination(float declination);
GY26_StatusTypeDef GY26_FactoryReset(void);
GY26_StatusTypeDef GY26_ProcessUartData(void);
void GY26_UART_IRQHandler(void);
float GY26_GetAngle(void);
uint8_t GY26_GetCalibrationLevel(void);


extern GY26_CompassTypeDef gy26_compass_instance;

#endif /* GY26_COMPASS_STD_H */

