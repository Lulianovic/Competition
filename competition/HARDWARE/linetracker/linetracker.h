#ifndef __LINETRACKER_H_
#define __LINETRACKER_H_

#include "stm32f4xx.h"
#include "sys.h"

#define LINE_S1 PAin(4)
#define LINE_S2 PAin(5)
#define LINE_S3 PAin(6)
#define LINE_S4 PAin(7)
#define LINE_S5 PCin(2)
#define LINE_S6 PCin(3)
#define LINE_S7 PCin(4)
#define LINE_S8 PCin(5)

void LineTracker_Init(void);
/**
  * @简  述  获取光电传感器状态
  * @参  数  无
  * @返回值  当前情况
  * 		  0x00 路线丢失
  *			  0xFF 十字路口
  */
uint8_t GetLineStatus(void);
/**
  * @简  述  根据光电传感器的状态获取转向值
  * @参  数  无
  * @返回值  转向值（负值左转，正值右转）
  */
int16_t GetLineTurning(void);

#endif
