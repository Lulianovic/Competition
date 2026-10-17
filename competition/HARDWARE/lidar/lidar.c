#include "lidar.h"
#include "motor.h"
#include "delay.h"
#include<stdio.h>
#include "speaker.h"
#include "speed.h"
/* ============== 全局变量定义 ============== */
LidarPacket Lidar_Pack_Data;
ProcessedPoint PointBuffer[BUFFER_SIZE];
volatile uint8_t Lidar_data_recv = 0;
static uint8_t current_valid = 0;
volatile uint32_t Lidar_data_cnt = 0;
uint8_t write_index = 0;
uint8_t last_write_cnt = 0;
RunPhase Current_Phase = PHASE_MAZE;
MazeState Maze_Status = MAZE_DECIDING;
HuntState Hunt_Status = HUNTING;

uint16_t Treasure_Count = 0;
uint16_t Front_Distance = 0;
uint8_t Escape_Flag = 0;
extern uint32_t counter_1MS;    // 1MS 定时器中断提供的系统时钟基准

/* 状态机变量 */
static uint32_t action_timer = 0;           // 动作计时器（转弯/撞击/掉头用）
static uint16_t last_treasure_angle = 0xFFFF; // 上次宝藏角度，避免重复检测
static uint16_t turn_start_opposite_dist = 0; // 转弯开始时对侧距离（用于判断转弯完成）

/* DMA接收缓冲区 */
#define RECEIVE_BUF_MAX_SIZE    108
uint8_t RX_BUF[RECEIVE_BUF_MAX_SIZE] = {0};

/* 雷达数据处理 */
void N10P1_DataHandle(uint8_t* temp_data);

/* ============== 串口和DMA初始化 ============== */
void UART_LiDAR_Init(void)
{           
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    DMA_InitTypeDef DMA_ReceiveInitStruct;
    
    /* 使能时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* GPIO复用配置 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);   
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);    
     
    /* PA2 - USART2_TX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* PA3 - USART2_RX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* USART2 参数配置 */
    USART_InitStructure.USART_BaudRate = 460800;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    
    /* DMA接收配置 */
    DMA_ReceiveInitStruct.DMA_Channel = DMA_Channel_4;
    DMA_ReceiveInitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_ReceiveInitStruct.DMA_Memory0BaseAddr = (uint32_t)RX_BUF;
    DMA_ReceiveInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_ReceiveInitStruct.DMA_BufferSize = RECEIVE_BUF_MAX_SIZE;
    DMA_ReceiveInitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_ReceiveInitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_ReceiveInitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_ReceiveInitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_ReceiveInitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_ReceiveInitStruct.DMA_Priority = DMA_Priority_Medium;
    DMA_ReceiveInitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_ReceiveInitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_ReceiveInitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_ReceiveInitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5, &DMA_ReceiveInitStruct);
    
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA1_Stream5, ENABLE);
    
    /* NVIC配置 */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

/* ============== 串口中断处理 ============== */
void USART2_IRQHandler(void)
{
    uint16_t recv_len;
    uint8_t temp_data[RECEIVE_BUF_MAX_SIZE];
    
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        (void)USART2->SR;
        (void)USART2->DR;
        
        DMA_Cmd(DMA1_Stream5, DISABLE);
        recv_len = RECEIVE_BUF_MAX_SIZE - DMA_GetCurrDataCounter(DMA1_Stream5);
        
        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | 
                     DMA_FLAG_TEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5);
        
        DMA1_Stream5->M0AR = (uint32_t)RX_BUF;
        DMA_SetCurrDataCounter(DMA1_Stream5, RECEIVE_BUF_MAX_SIZE);
        DMA_Cmd(DMA1_Stream5, ENABLE);
                
        if(recv_len == RECEIVE_BUF_MAX_SIZE) 
        {
            memcpy(temp_data, RX_BUF, RECEIVE_BUF_MAX_SIZE);
            N10P1_DataHandle(temp_data);
        }
    }
}

/* ============== 雷达数据协议解析 ============== */
void N10P1_DataHandle(uint8_t* temp_data)
{   
    uint8_t index;
    uint8_t crc = 0;
    uint8_t point_idx = 0;
    
    for(index = 0; index < RECEIVE_BUF_MAX_SIZE; index++)
    {  
        // printf("temp_data[%d] = 0x%x\n",index,temp_data[index]);   
        if(index < RECEIVE_BUF_MAX_SIZE - 1) crc += temp_data[index];
            
        switch(index)
        {
            case 0: if(temp_data[index] != 0xA5) return; break;
            case 1: if(temp_data[index] != 0x5A) return; break;
            case 2: break;
            case 3: Lidar_Pack_Data.speed_h = temp_data[index]; break;
            case 4: Lidar_Pack_Data.speed_l = temp_data[index]; break;
            case 5: Lidar_Pack_Data.start_angle_h = temp_data[index]; break;
            case 6: 
                Lidar_Pack_Data.start_angle_l = temp_data[index];
                point_idx = 0;
                break;
            
            case 7: case 10: case 13: case 16: case 19: case 22: case 25: case 28:
            case 31: case 34: case 37: case 40: case 43: case 46: case 49: case 52:
            case 55: case 58: case 61: case 64: case 67: case 70: case 73: case 76:
            case 79: case 82: case 85: case 88: case 91: case 94: case 97: case 100:
                Lidar_Pack_Data.point[point_idx].distance_h = temp_data[index] & 0x7F;
                break;
                
            case 8: case 11: case 14: case 17: case 20: case 23: case 26: case 29:
            case 32: case 35: case 38: case 41: case 44: case 47: case 50: case 53:
            case 56: case 59: case 62: case 65: case 68: case 71: case 74: case 77:
            case 80: case 83: case 86: case 89: case 92: case 95: case 98: case 101:
                Lidar_Pack_Data.point[point_idx].distance_l = temp_data[index];
                break;
                
            case 9: case 12: case 15: case 18: case 21: case 24: case 27: case 30:
            case 33: case 36: case 39: case 42: case 45: case 48: case 51: case 54:
            case 57: case 60: case 63: case 66: case 69: case 72: case 75: case 78:
            case 81: case 84: case 87: case 90: case 93: case 96: case 99: case 102:
                Lidar_Pack_Data.point[point_idx].strength = temp_data[index];
                point_idx++;
                break;
            
            case 103: case 104: break;
            case 105: Lidar_Pack_Data.end_angle_h = temp_data[index]; break;
            case 106: Lidar_Pack_Data.end_angle_l = temp_data[index]; break;
            case 107:
                if(crc == temp_data[index]) Process_Lidar_Data_Optimized();
                return;
            default: break;
        }
    }
}


/* ============== 角度工具函数 ============== */
uint8_t Is_Angle_Valid(uint16_t angle)
{
    /* 有效范围：270度-360度 和 0度-90度 */
    return ((angle >= VALID_ANGLE_START) || (angle <= VALID_ANGLE_END));
}

uint8_t Is_In_Detect_Area(uint16_t angle, uint16_t start, uint16_t end)
{
    /* 支持跨越0度的角度范围判断 */
    if(start <= end)
        return (angle >= start && angle <= end);
    else
        return (angle >= start || angle <= end);
}

static uint8_t Is_Frame_Angle_Valid(uint16_t start_angle, uint16_t end_angle)
{
    uint16_t temp_end = end_angle;
    if(start_angle > temp_end) temp_end += ANGLE_CIRCLE;
    uint16_t center = start_angle + (temp_end - start_angle) / 2;
    if(center >= ANGLE_CIRCLE) center -= ANGLE_CIRCLE;
    return Is_Angle_Valid(center);
}

/* ============== 雷达数据处理（优化版） ============== */
void Process_Lidar_Data_Optimized(void)
{
    static uint16_t last_avg_angle = VALID_ANGLE_START;
    
    uint16_t start_angle, end_angle;
    uint16_t temp_end_angle;
    uint16_t angle_step;
    
    uint16_t distances[16];
    uint16_t angles[16];
    uint8_t valid_count = 0;
    
    uint8_t min_idx = 0, max_idx = 0;
    uint32_t distance_sum = 0;
    uint32_t angle_sum = 0;
    uint8_t filtered_count = 0;
    
    uint16_t avg_distance;
    uint16_t avg_angle;
    
    int i;
    uint8_t j;
    
    /* 解析帧头角度 */
    start_angle = ((uint16_t)Lidar_Pack_Data.start_angle_h << 8) | Lidar_Pack_Data.start_angle_l;
    end_angle = ((uint16_t)Lidar_Pack_Data.end_angle_h << 8) | Lidar_Pack_Data.end_angle_l;
    
    /* 帧有效性检查 */
    if(!Is_Frame_Angle_Valid(start_angle, end_angle))
    {
        if(current_valid)
        {
            Lidar_data_recv = 0;
        }
        return;
    }
    
    /* 计算角度步长 */
    temp_end_angle = end_angle;
    if(start_angle > temp_end_angle) {
        temp_end_angle += ANGLE_CIRCLE;
    }
    angle_step = (temp_end_angle - start_angle) / 15;
    
    /* 提取双采样数据：每2点取有效点一个点 */
    for(i = 0; i < 32; i += 2)
    {
        uint16_t temp_angle = start_angle + angle_step * (i / 2);
        if(temp_angle >= ANGLE_CIRCLE) {
            temp_angle -= ANGLE_CIRCLE;
        }
        uint16_t current_angle = (uint16_t)temp_angle;
        
        if(!Is_Angle_Valid(current_angle)) {
            continue;
        }
        
        uint16_t distance1 = (Lidar_Pack_Data.point[i].distance_h << 8) | Lidar_Pack_Data.point[i].distance_l;
        uint16_t distance2 = (Lidar_Pack_Data.point[i+1].distance_h << 8) | Lidar_Pack_Data.point[i+1].distance_l;
        
        uint16_t final_distance = 0;
        
        if(distance1 >= MIN_VALID_DISTANCE && distance1 <= MAX_VALID_DISTANCE) {
            final_distance = distance1;
        } 
        else if(distance2 >= MIN_VALID_DISTANCE && distance2 <= MAX_VALID_DISTANCE) {
            final_distance = distance2;
        }
        
        if(final_distance > 0)
        {
            distances[valid_count] = final_distance;
            angles[valid_count] = current_angle;
            valid_count++;
            
            if(valid_count >= 16) {
                break;
            }
        }
    }
    
    if(valid_count < 3)
    {
        return;
    }
    
    /* 异常值剔除：去掉最大最小值 */
    min_idx = 0;
    max_idx = 0;
    
    for(j = 1; j < valid_count; j++) {
        if(distances[j] < distances[min_idx]) min_idx = j;
        if(distances[j] > distances[max_idx]) max_idx = j;
    }
    
    /* 计算平均值 */
    distance_sum = 0;
    angle_sum = 0;
    filtered_count = 0;

    for (j = 0; j < valid_count; j++)
    {
        if (j != min_idx && j != max_idx)
        {
            distance_sum += distances[j];
            // printf("angles[%d] = %d\n",j,angles[j]);
            if(angles[j] < start_angle)
            {
                angle_sum += angles[j] + ANGLE_CIRCLE;
            }else
            {
                angle_sum += angles[j];
            }
            
            filtered_count++;
        }
    }

    avg_distance = distance_sum / filtered_count;
    avg_angle = angle_sum / filtered_count;
    
    if(avg_angle >= ANGLE_CIRCLE) {
        avg_angle -= ANGLE_CIRCLE;
    }
    
    if(!Is_Angle_Valid(avg_angle))
    {
        return;
    }
    
    /* 检测新的一圈开始（角度从大到小跨越） */
    if(avg_angle >= VALID_ANGLE_START && last_avg_angle <= VALID_ANGLE_END)
    {
        // printf("-------------------------------------\n");
        if(current_valid)
        {
            Lidar_data_cnt++;
        }
        else
        {
            Lidar_data_cnt = 0;
        }

        current_valid = 1;
        Lidar_data_recv = 1;
        last_write_cnt = write_index;
        write_index = 0;
    }
    
    // printf("last_avg_angle = %d,avg_angle = %d\n",last_avg_angle,avg_angle);

    last_avg_angle = avg_angle;
    
    /* 存储滤波后的点数据 */
    if(current_valid)
    {
        PointBuffer[write_index].angle = avg_angle;
        PointBuffer[write_index].distance = avg_distance;
        PointBuffer[write_index].valid = 1;
        PointBuffer[write_index].quality = filtered_count;
        
        write_index = (write_index + 1) % BUFFER_SIZE;
    }
}

/* ============== 方向距离计算（增强版） ============== */
void Get_Direction_Distances(uint16_t* right, uint16_t* front, uint16_t* left)
{
    uint32_t i;
    *right = 0xFFFF; 
    *front = 0xFFFF; 
    *left = 0xFFFF;
    
    for(i = 0; i < last_write_cnt; i++)
    {
        if(!PointBuffer[i].valid) continue;
        
        uint16_t ang = PointBuffer[i].angle;
        uint16_t dist = PointBuffer[i].distance;
        
        /* 右侧 85度-95度 */
        if(ang >= RIGHT_DETECT_START && ang <= RIGHT_DETECT_END)
        {
            if(dist < *right) *right = dist;
        }
        /* 前方 330度-30度（跨越0度） */
        else if(Is_In_Detect_Area(ang, FRONT_DETECT_START, FRONT_DETECT_END))
        {
            if(dist < *front) *front = dist;
        }
        /* 左侧 265度-275度 */
        else if(ang >= LEFT_DETECT_START && ang <= LEFT_DETECT_END)
        {
            if(dist < *left) *left = dist;
        }
    }
}

/* 获取右上距离（50度-60度） */
static uint16_t Get_Right_Up_Distance(void)
{
    uint32_t i;
    uint16_t min_dist = 0xFFFF;
    
    for(i = 0; i < last_write_cnt; i++)
    {
        if(!PointBuffer[i].valid) continue;
        
        uint16_t ang = PointBuffer[i].angle;
        if(ang >= RIGHT_UP_DETECT_START && ang <= RIGHT_UP_DETECT_END)
        {
            if(PointBuffer[i].distance < min_dist) 
                min_dist = PointBuffer[i].distance;
        }
    }
    return min_dist;
}

/* 获取右下距离（120度-130度） */
static uint16_t Get_Right_Down_Distance(void)
{
    uint32_t i;
    uint16_t min_dist = 0xFFFF;
    
    for(i = 0; i < last_write_cnt; i++)
    {
        if(!PointBuffer[i].valid) continue;
        
        uint16_t ang = PointBuffer[i].angle;
        if(ang >= RIGHT_DOWN_DETECT_START && ang <= RIGHT_DOWN_DETECT_END)
        {
            if(PointBuffer[i].distance < min_dist) 
                min_dist = PointBuffer[i].distance;
        }
    }
    return min_dist;
}

/* 获取左前方距离（310度-320度，用于左转判断） */
static uint16_t Get_Left_Front_Distance(void)
{
    uint32_t i;
    uint16_t min_dist = 0xFFFF;
    
    for(i = 0; i < last_write_cnt; i++)
    {
        if(!PointBuffer[i].valid) continue;
        
        uint16_t ang = PointBuffer[i].angle;
        if(ang >= LEFT_FRONT_DETECT_START && ang <= LEFT_FRONT_DETECT_END)
        {
            if(PointBuffer[i].distance < min_dist) 
                min_dist = PointBuffer[i].distance;
        }
    }
    return min_dist;
}

/* ============== 贴墙走速度计算（窄走廊优化版） ============== */
/* 适用于理想距离200mm，有效范围150-250mm的窄走廊环境 */
static void Calculate_Wall_Following_Speed(uint16_t right_up, uint16_t right, uint16_t right_down, 
                                          int16_t* left_speed, int16_t* right_speed)
{
    int16_t error_dist;      // 距离偏差：正=太远(需右转靠近)，负=太近(需左转远离)
    int16_t error_angle;     // 角度偏差：正=墙向左斜(需右转)，负=墙向右斜(需左转)
    int16_t control;         // 综合控制量，正=右转(左快右慢)，负=左转
    
    /* 1. 计算偏差 */
    error_dist = (int16_t)right - WALL_IDEAL;  // 与200mm的偏差
    
    // 角度偏差：右上>右下表示墙面向左倾斜（机器人需要右转跟随）
    // 缩放角度影响（除以2），因为角度偏差通常比距离偏差大
    error_angle = ((int16_t)right_up - (int16_t)right_down) >> 1;
    
    /* 2. 紧急情况处理（硬保护） */
    if (right < 100) {  // 极近距离，可能撞墙，紧急左转
        *left_speed = 0;
        *right_speed = FORWARD_SPEED;
        printf("WALL: EMERGENCY! Too close (%d), hard left!\n", right);
        return;
    }
    if (right > 350 || right == 0xFFFF) {  // 完全丢失右墙（>350或无效）
        *left_speed = FORWARD_SPEED;
        *right_speed = 30;  // 缓慢右转寻找墙壁
        printf("WALL: Lost wall (%d), seeking right...\n", right);
        return;
    }
    
    /* 3. 边界软化处理（在150-250范围边缘提前减速调整） */
    if (right < WALL_MIN_LENG) {  // 150-200之间，偏近
        // 越靠近150，左转越急
        control = -((WALL_MIN_LENG - right) << 1);  // 线性增加左转力度，最大-100
    }
    else if (right > WALL_MAX_LENG) {  // 200-250之间，偏远
        // 越靠近250，右转越急  
        control = ((right - WALL_MAX_LENG) << 1);   // 线性增加右转力度，最大+100
    }
    else {
        /* 4. 正常范围内（150-250）使用比例控制 */
        // 距离控制：Kp_dist = 0.8（接近1，因为范围窄需要更敏感）
        // 角度控制：Kp_angle = 0.5（除以2）
        int16_t dist_ctrl = (error_dist * 4) >> 2;      // ×0.8 = ×4/5，但简化为×4>>2(实际×1)
        // 实际上直接用error_dist，因为范围小，偏差本身就不大
        
        control = error_dist + error_angle;
        
        /* 5. 死区处理（在190-210mm内且角度平缓时不调整） */
        if (error_dist > -10 && error_dist < 10 && error_angle > -20 && error_angle < 20) {
            control = 0;
        }
    }
    
    /* 6. 控制量平滑限幅（窄走廊转弯要柔和，避免震荡） */
    #define MAX_CONTROL  50  // 最大速度差限制在50（太大了容易撞墙）
    if (control > MAX_CONTROL) control = MAX_CONTROL;
    if (control < -MAX_CONTROL) control = -MAX_CONTROL;
    
    /* 7. 计算最终速度（基础速度降低以提高稳定性） */
    #define WALL_FOLLOW_BASE_SPEED  120  // 窄走廊降低基础速度，提高控制精度
    *left_speed = WALL_FOLLOW_BASE_SPEED + control;
    *right_speed = WALL_FOLLOW_BASE_SPEED - control;
    
    /* 8. 速度硬限幅 */
    #define MIN_SPEED 30   // 最低速，防止堵转
    #define MAX_SPEED 180  // 最高速，窄走廊不宜太快
    if (*left_speed > MAX_SPEED) *left_speed = MAX_SPEED;
    if (*left_speed < MIN_SPEED) *left_speed = MIN_SPEED;
    if (*right_speed > MAX_SPEED) *right_speed = MAX_SPEED;
    if (*right_speed < MIN_SPEED) *right_speed = MIN_SPEED;
    
    printf("WALL: R=%d, E_dist=%d, E_ang=%d, ctrl=%d, L=%d, R=%d\n", 
           right, error_dist, error_angle, control, *left_speed, *right_speed);
}

/* ============== 右手法则迷宫算法（贴墙走只在直行时检测） ============== */
void Right_Hand_Rule_Task(void)
{
    uint16_t right_dist, front_dist, left_dist;
    uint16_t right_up_dist, right_down_dist, left_front_dist;
    int16_t left_speed, right_speed;
    
    /* 获取所有方向距离 */
    Get_Direction_Distances(&right_dist, &front_dist, &left_dist);
    left_front_dist = Get_Left_Front_Distance();  /* 左前 310-320度 */
    right_up_dist = Get_Right_Up_Distance();      /* 右上 50-60度 */
    right_down_dist = Get_Right_Down_Distance();  /* 右下 120-130度 */

    printf("F:%d L:%d LF:%d | RU:%d R:%d RD:%d | State:%d\n", 
               front_dist, left_dist, left_front_dist, 
               right_up_dist, right_dist, right_down_dist, 
               Maze_Status);
   
    switch(Maze_Status)
    {
        case MAZE_DECIDING:
            printf("DECIDING\n");
            MOTOR_SetSpeed(0, 0);
            
            /* ========== 规则1：右转判断 ========== */
            /* 右上距离大于阈值 并且 右侧距离大于阈值 并且 右侧距离大于左侧距离 并且 右侧距离大于左前距离 */
            if(right_dist > WALL_THRESHOLD && 
               right_dist > left_dist && right_dist > left_front_dist )
            {
                UART_Speaker("右转");
                printf("DECISION: START TURN RIGHT\n");
                Maze_Status = MAZE_TURNING_RIGHT;
                MOTOR_SetSpeed(TURN_SPEED, -TURN_SPEED);
            }
            /* ========== 规则2：前进判断 ========== */
            else if(front_dist > FRONT_CLEARANCE)
            {
                UART_Speaker("直行");
                printf("DECISION: MOVE FORWARD\n");
                Maze_Status = MAZE_MOVING;
                /* 使用三个右侧距离计算贴墙走速度 */
                Calculate_Wall_Following_Speed(right_up_dist, right_dist, right_down_dist, 
                                               &left_speed, &right_speed);
                MOTOR_SetSpeed(left_speed, right_speed);
            }
            /* ========== 规则3：左转判断 ========== */
            else if(left_dist > WALL_THRESHOLD)
            {
                UART_Speaker("左转");
                printf("DECISION: START TURN LEFT\n");
                Maze_Status = MAZE_TURNING_LEFT;
                MOTOR_SetSpeed(-TURN_SPEED, TURN_SPEED);
            }
            /* ========== 规则4：掉头前检查侧前方 ========== */
            else
            {
                printf("CHECK SIDE FRONT BEFORE ROTATE\n");
                /* 左前方有空间，左转 */
                if(left_front_dist > WALL_THRESHOLD)
                {
                    printf("DECISION: TURN LEFT (left front clear)\n");
                    Maze_Status = MAZE_TURNING_LEFT;
                    MOTOR_SetSpeed(-TURN_SPEED, TURN_SPEED);
                }
                /* 右上有空间，右转 */
                else if(right_up_dist > WALL_THRESHOLD)
                {
                    printf("DECISION: TURN RIGHT (right up clear)\n");
                    Maze_Status = MAZE_TURNING_RIGHT;
                    MOTOR_SetSpeed(TURN_SPEED, -TURN_SPEED);
                }
                /* 所有方向都被阻挡，掉头 */
                else
                {
                    printf("DECISION: ROTATE (all blocked)\n");
                    Maze_Status = MAZE_ROTATING;
                    MOTOR_SetSpeed(ROTATE_SPEED, -ROTATE_SPEED);
                }
            }
            break;
            
        /* ========== 右转状态：只关注转弯完成，不检测贴墙走 ========== */
        case MAZE_TURNING_RIGHT:
            printf("TURNING_RIGHT\n");
            /* 转了一点后，检查前方是否通畅 */
            if(front_dist > FRONT_CLEARANCE)
            {
                /* 可以前进了，进入前进状态 */
                printf("RIGHT: front clear, switch to MOVING\n");
                Maze_Status = MAZE_MOVING;
                /* 切换到直行时再获取贴墙走数据并计算速度 */
                Calculate_Wall_Following_Speed(right_up_dist, right_dist, right_down_dist, 
                                               &left_speed, &right_speed);
                MOTOR_SetSpeed(left_speed, right_speed);
            }
            /* 右上还有空间，继续右转一点（不检测贴墙走） */
            else
            {
                printf("RIGHT: continue turning\n");
                MOTOR_SetSpeed(TURN_SPEED, -TURN_SPEED);
            }
            break;
            
        /* ========== 左转状态：只关注转弯完成，不检测贴墙走 ========== */
        case MAZE_TURNING_LEFT:
            printf("TURNING_LEFT\n");
            /* 转了一点后，检查前方是否通畅 */
            if(front_dist > FRONT_CLEARANCE)
            {
                /* 可以前进了，进入前进状态 */
                printf("LEFT: front clear, switch to MOVING\n");
                Maze_Status = MAZE_MOVING;
                /* 切换到直行时再获取贴墙走数据并计算速度 */
                right_up_dist = Get_Right_Up_Distance();
                right_down_dist = Get_Right_Down_Distance();
                Calculate_Wall_Following_Speed(right_up_dist, right_dist, right_down_dist, 
                                               &left_speed, &right_speed);
                MOTOR_SetSpeed(left_speed, right_speed);
            }
            /* 左前方还有空间，继续左转一点（不检测贴墙走） */
            else
            {
                printf("LEFT: continue turning\n");
                MOTOR_SetSpeed(-TURN_SPEED, TURN_SPEED);
            }
            break;
            
        /* ========== 掉头状态 ========== */
        case MAZE_ROTATING:
            printf("ROTATING\n");
            UART_Speaker("旋转");
            
            /* 掉头过程中也检查前方，如果前方通畅就停止掉头 */
            if(front_dist > FRONT_CLEARANCE)
            {
                printf("ROTATE: front clear, stop rotating\n");
                Maze_Status = MAZE_DECIDING;
                MOTOR_SetSpeed(0, 0);
            }
            else
            {
                MOTOR_SetSpeed(ROTATE_SPEED, -ROTATE_SPEED);
            }
            break;
            
        /* ========== 前进状态（使用三个右侧距离精细贴墙走） ========== */
        case MAZE_MOVING:
            /* 前方有阻挡，需要重新决策 */
            if(front_dist < FRONT_CLEARANCE)
            {
                printf("MOVING: front blocked, go to DECIDING\n");
                Maze_Status = MAZE_DECIDING;
                MOTOR_SetSpeed(0, 0);
            }
            /* 右上和右侧都出现通路，优先右转（右手法则） */
            else if(right_dist > WALL_THRESHOLD && 
                    right_dist > left_dist && right_dist > left_front_dist)
            {
                printf("MOVING: right clear, go to DECIDING for right turn\n");
                Maze_Status = MAZE_DECIDING;
                MOTOR_SetSpeed(0, 0);
            }
            else
            {
                /* 继续前进，使用三个右侧距离精细调整方向（贴墙走） */
                Calculate_Wall_Following_Speed(right_up_dist, right_dist, right_down_dist, 
                                               &left_speed, &right_speed);
                MOTOR_SetSpeed(left_speed, right_speed);
            }
            break;
    }
}

/* ============== 迷宫逃脱检测 ============== */
uint8_t Check_Maze_Escaped(void)
{
    uint16_t right, front, left;
    Get_Direction_Distances(&right, &front, &left);
    
    Front_Distance = front;
    
    /* 逃脱条件：前方>2米 且 右侧>1米 且 左侧>1米 */
    if(front > ESCAPE_DISTANCE && right > 1000 && left > 1000)
    {
        return 1;
    }
    return 0;
}

/* ============== 边界墙判断（探宝用） ============== */
uint8_t Is_Boundary_Wall(uint16_t angle, uint16_t distance)
{
    uint16_t right = 0xFFFF, left = 0xFFFF;
    uint8_t r_valid = 0, l_valid = 0;
    uint32_t i;
    
    for(i = 0; i < BUFFER_SIZE; i++)
    {
        if(!PointBuffer[i].valid) continue;
        
        if(PointBuffer[i].angle >= RIGHT_DETECT_START && PointBuffer[i].angle <= RIGHT_DETECT_END)
        {
            if(PointBuffer[i].distance < right) { right = PointBuffer[i].distance; r_valid = 1; }
        }
        else if(PointBuffer[i].angle >= LEFT_DETECT_START && PointBuffer[i].angle <= LEFT_DETECT_END)
        {
            if(PointBuffer[i].distance < left) { left = PointBuffer[i].distance; l_valid = 1; }
        }
    }
    
    if(r_valid && l_valid)
    {
        if(abs((int16_t)distance - (int16_t)right) < 500 ||
           abs((int16_t)distance - (int16_t)left) < 500)
            return 1;
    }
    return 0;
}

/* ============== 宝藏检测 ============== */
uint8_t Check_Treasure_Detected(void)
{
    uint16_t min_d = 0xFFFF, max_d = 0, treasure_ang = 0;
    uint8_t pts = 0;
    uint32_t i;
    
    for(i = 0; i < BUFFER_SIZE; i++)
    {
        if(!PointBuffer[i].valid) continue;
        if(!Is_In_Detect_Area(PointBuffer[i].angle, TREASURE_AREA_START, TREASURE_AREA_END)) continue;
        
        if(PointBuffer[i].distance < min_d) 
        {
            min_d = PointBuffer[i].distance;
            treasure_ang = PointBuffer[i].angle;
        }
        if(PointBuffer[i].distance > max_d) max_d = PointBuffer[i].distance;
        pts++;
    }
    
    if(pts >= 3 && (max_d - min_d) > TREASURE_THRESHOLD)
    {
        if(min_d < 150 || min_d > ESCAPE_DISTANCE) return 0;
        if(Is_Boundary_Wall(treasure_ang, min_d)) return 0;
        
        /* 角度去重 */
        if(last_treasure_angle != 0xFFFF)
        {
            int16_t diff = (int16_t)treasure_ang - (int16_t)last_treasure_angle;
            if(diff < 0) diff = -diff;
            uint16_t adiff = (uint16_t)diff;
            if(adiff > 18000) adiff = 36000 - adiff;
            if(adiff < TREASURE_DEAD_ZONE) return 0;
        }
        
        last_treasure_angle = treasure_ang;
        return 1;
    }
    return 0;
}

/* ============== 探宝任务 ============== */
void Treasure_Hunt_Task(void)
{
    uint32_t now;
    
    switch(Hunt_Status)
    {
        case HUNTING:
            MOTOR_SetSpeed(FORWARD_SPEED, FORWARD_SPEED);
            
            if(Check_Treasure_Detected())
            {
                Hunt_Status = CRASHING;
                action_timer = counter_1MS;
            }
            break;
            
        case CRASHING:
            MOTOR_SetSpeed(CRASH_SPEED, CRASH_SPEED);
            now = counter_1MS;
            if((now - action_timer) >= CRASH_TIME_MS)
            {
                Treasure_Count++;
                Hunt_Status = CONTINUE;
            }
            break;
            
        case CONTINUE:
            MOTOR_SetSpeed(FORWARD_SPEED, FORWARD_SPEED);
            Hunt_Status = HUNTING;
            break;
    }
}

/* ============== 迷宫主任务 ============== */
void Maze_Main_Task(void)
{
    switch(Current_Phase)
    {
        case PHASE_MAZE:
            Right_Hand_Rule_Task();
            
            // if(Check_Maze_Escaped())
            // {
            //     Current_Phase = PHASE_FINISHED;
            // }
            break;
            
        // case PHASE_TREASURE:
        //     printf("PHASE_TREASURE\n");
        //     Treasure_Hunt_Task();
            
        //     /* 检查是否逃脱 */
        //     if(Front_Distance > ESCAPE_DISTANCE)
        //     {
        //         Current_Phase = PHASE_FINISHED;
        //         MOTOR_SetSpeed(0, 0);
        //         Escape_Flag = 1;
        //     }
        //     break;
            
        case PHASE_FINISHED:
            printf("PHASE_FINISHED\n");
            MOTOR_SetSpeed(0, 0);
            break;
    }
}

/* ============== 系统初始化 ============== */
void Maze_Init(void)
{
    Current_Phase = PHASE_MAZE;
    Maze_Status = MAZE_DECIDING;
    Hunt_Status = HUNTING;
    Treasure_Count = 0;
    Front_Distance = 0;
    Escape_Flag = 0;
    action_timer = 0;
    last_treasure_angle = 0xFFFF;
    turn_start_opposite_dist = 0;
}