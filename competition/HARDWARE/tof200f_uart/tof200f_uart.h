#ifndef TOF_UART_H
#define TOF_UART_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* UART6 接口配置 */
#define TOF_UART_INSTANCE       USART6
#define TOF_UART_BAUDRATE       115200

/* GPIO 引脚定义 (UART6 - PG14/PG9) */
#define TOF_UART_TX_PIN         GPIO_Pin_14
#define TOF_UART_TX_PORT        GPIOG
#define TOF_UART_TX_PINSOURCE   GPIO_PinSource14
#define TOF_UART_TX_AF          GPIO_AF_USART6

#define TOF_UART_RX_PIN         GPIO_Pin_9
#define TOF_UART_RX_PORT        GPIOG
#define TOF_UART_RX_PINSOURCE   GPIO_PinSource9
#define TOF_UART_RX_AF          GPIO_AF_USART6

/* 中断配置 */
#define TOF_UART_IRQn           USART6_IRQn
#define TOF_UART_IRQHandler     USART6_IRQHandler
#define TOF_UART_PRE_PRIO       0
#define TOF_UART_SUB_PRIO       0

/* 8字节滑动窗口缓冲区配置 */
#define TOF_FRAME_SIZE          7
#define TOF_RX_TIMEOUT_MS       100

/* 设备默认地址 */
#define TOF_DEFAULT_ADDR        0x01


#define TOF_PHY_SIZE        6
/* 处理后的帧数据结构 */
typedef struct {
    uint8_t  slave_addr;        /* 从机地址 */
    uint16_t distance;          /* 距离值 */
    bool     valid;             /* 数据是否有效 */
} TOF_Frame_t;

/* 函数声明 */
void TOF_UART_Init(void);
void TOF_UART_SendByte(uint8_t data);
void TOF_UART_SendData(uint8_t *pData, uint16_t Size);
void TOF_UART_SendString(char *str);

void TOF_UART_StartReceive_IT(void);
void TOF_UART_ClearRxBuffer(void);

/* 获取处理后的帧数据 */
bool TOF_IsFrameReady(void);
void TOF_SetFrameReady(bool value);
/* Modbus RTU 接口 */
uint16_t TOF_Modbus_CRC16(uint8_t *pData, uint16_t length);
void TOF_ReadDistance(uint8_t slave_addr);
void TOF_SetMode(uint8_t slave_addr, uint8_t mode);
void TOF_SetAutoOutput(uint8_t slave_addr, uint16_t period_ms);

/* 中断回调 (内部使用) */
void TOF_UART_RxHandler(void);
void tof200f_test(void);


extern TOF_Frame_t current_frame[TOF_PHY_SIZE];
#endif /* TOF_UART_H */
