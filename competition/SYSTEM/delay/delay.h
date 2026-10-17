#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  
#include "los_task.h"



void delay(uint32_t nus);
void delay_ms(uint32_t nms);
void delay_us(uint32_t nus);
void delay_us2(uint32_t nus);

#define SMART_DELAY_MS(ms)  \
        if (LOS_CurTaskIDGet() != LOS_ERRNO_TSK_ID_INVALID) { \
            LOS_Msleep(ms); \
        } else { \
            delay_ms(ms); \
        } \

#endif




























