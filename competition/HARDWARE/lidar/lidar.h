#ifndef __LIDAR_H
#define __LIDAR_H

#include "stm32f4xx.h"
#include <stdlib.h>
#include <string.h>

/* ============== 缓冲大小 ============== */
#define BUFFER_SIZE         50      /* 滤波点缓冲区大小，存储50个滤波后的点 */

/* ============== 基础速度定义 ============== */
#define FORWARD_SPEED       150     /* 前进基准速度 */
#define TURN_SPEED          100     /* 转弯速度（90度转弯） */
#define ROTATE_SPEED        100     /* 原地旋转速度（180度掉头） */
#define CRASH_SPEED         200     /* 撞击 treasure 速度 */

/* ============== 角度定义（单位：百分之一度，即0.01度） ============== */
#define ANGLE_CIRCLE        36000   /* 360度 = 36000个百分之一度 */

/* 雷达有效扫描范围：270度-360度 和 0度-90度（右手法则需要和右侧、前方、左侧） */
#define VALID_ANGLE_START   23000   /* 230.00度 */
#define VALID_ANGLE_END     13000    /* 130.00度 */

/* ============== 方向检测角度范围（右手法则用） ============== */
/* 前方检测范围：330度-30度（跨越0度） */
#define FRONT_DETECT_START  33000   /* 330.00度 */
#define FRONT_DETECT_END    3000    /* 30.00度 */

/* 右侧检测范围（右手法则主检测）：85度-95度 */
#define RIGHT_DETECT_START      8500    /* 85.00度 */
#define RIGHT_DETECT_END        9500    /* 95.00度 */

/* 右上检测范围：50度-60度（用于贴墙走和右转判断） */
#define RIGHT_UP_DETECT_START   5000    /* 50.00度 */
#define RIGHT_UP_DETECT_END     6000    /* 60.00度 */

/* 右下检测范围：120度-130度（用于贴墙走） */
#define RIGHT_DOWN_DETECT_START 12000   /* 120.00度 */
#define RIGHT_DOWN_DETECT_END   13000   /* 130.00度 */

/* 左侧检测范围：265度-275度 */
#define LEFT_DETECT_START       26500   /* 265.00度 */
#define LEFT_DETECT_END         27500   /* 275.00度 */

/* 左上检测范围：300度-310度（对应右上对称，用于左贴墙） */
#define LEFT_UP_DETECT_START    30000   /* 300.00度 */
#define LEFT_UP_DETECT_END      31000   /* 310.00度 */

/* 左下检测范围：230度-240度（对应右下对称，用于左贴墙） */
#define LEFT_DOWN_DETECT_START  23000   /* 230.00度 */
#define LEFT_DOWN_DETECT_END    24000   /* 240.00度 */

/* ============== 侧前方检测角度范围（用于转弯完成判断） ============== */
/* 左转后检测左前方：310度-320度（与左上略有重叠，但用途不同） */
#define LEFT_FRONT_DETECT_START 31000   /* 310.00度 */
#define LEFT_FRONT_DETECT_END   32000   /* 320.00度 */

/* ============== 探宝检测角度范围 ============== */
/* 宝藏检测范围：前方330度-30度（跨越0度，与前方检测范围相同，但用途不同） */
#define TREASURE_AREA_START 33000   /* 330.00度 */
#define TREASURE_AREA_END   3000    /* 30.00度 */

/* ============== 距离阈值定义（单位：毫米） ============== */
#define WALL_THRESHOLD          600     /* 墙壁阈值：检测到墙壁的最大距离（mm） */
#define WALL_MAX_LENG           250     /* 右侧距离上限，>250mm需要向右靠近 */
#define WALL_MIN_LENG           150     /* 右侧距离下限，<150mm需要向左远离 */
#define WALL_IDEAL              200     /* 理想贴墙距离（mm） */
#define FRONT_CLEARANCE         300     /* 前方安全距离：<300mm需要转弯（mm） */
#define TREASURE_THRESHOLD      150     /* 宝藏距离突变阈值：30度范围内距离突变（最大-最小>150mm）认为是宝藏 */
#define ESCAPE_DISTANCE         2000    /* 逃脱距离：前方>2米认为已走出迷宫（mm） */
#define SIDE_FRONT_THRESHOLD    400     /* 侧前方距离阈值，大于此值认为可以继续转弯（mm） */

/* ============== 时间参数定义（单位：毫秒） ============== */
#define CRASH_TIME_MS           400     /* 撞击 treasure 持续时间 */
#define TURN_90_TIME_MS         200     /* 90度转弯参考时间 */
#define ROTATE_180_TIME_MS      400     /* 180度掉头参考时间 */

/* ============== 雷达数据有效范围 ============== */
#define MIN_VALID_DISTANCE      15      /* 最小有效距离：15mm（过滤近距离噪声） */
#define MAX_VALID_DISTANCE      15000   /* 最大有效距离：15m（过滤无效数据） */

/* ============== 宝藏检测参数 ============== */
#define TREASURE_DEAD_ZONE      2500    /* 宝藏角度死区：25度内不重复检测（百分之一度） */

/* ============== 运行阶段枚举 ============== */
typedef enum {
    PHASE_MAZE = 0,      /* 迷宫阶段：使用右手法则导航 */
    PHASE_TREASURE,      /* 探宝阶段：搜索并撞击宝藏 */
    PHASE_FINISHED       /* 完成阶段：已走出迷宫 */
} RunPhase;

/* ============== 迷宫状态枚举 ============== */
typedef enum {
    MAZE_DECIDING = 0,   /* 决策中：根据传感器数据决定下一步动作 */
    MAZE_TURNING_RIGHT,  /* 右转中：执行右转（一次转一点） */
    MAZE_TURNING_LEFT,   /* 左转中：执行左转（一次转一点） */
    MAZE_ROTATING,       /* 掉头中：执行180度原地旋转 */
    MAZE_MOVING          /* 直行中：沿当前方向直走 */
} MazeState;

/* ============== 探宝状态枚举 ============== */
typedef enum {
    HUNTING = 0,         /* 搜寻中：前方扫描宝藏 */
    CRASHING,            /* 撞击中：检测到宝藏，前进撞击 */
    CONTINUE             /* 继续：撞击动作完成，恢复搜寻状态 */
} HuntState;

/* ============== 雷达数据结构 ============== */
/* 单点数据：N10P1雷达每个角度的距离值和信号强度 */
typedef struct {
    uint8_t distance_h;  /* 距离高字节（bit6-0有效，bit7为强度标志） */
    uint8_t distance_l;  /* 距离低字节 */
    uint8_t strength;    /* 信号强度（可信度） */
} LidarPoint;

/* 雷达数据包：N10P1一帧数据包含32个点（双采样模式） */
typedef struct {
    uint8_t speed_h;         /* 转速高字节 */
    uint8_t speed_l;         /* 转速低字节（单位：0.01度/ms） */
    uint8_t start_angle_h;   /* 起始角度高字节 */
    uint8_t start_angle_l;   /* 起始角度低字节（单位：0.01度） */
    LidarPoint point[32];    /* 32个点的数据（双采样模式：16个角度、2次采样） */
    uint8_t end_angle_h;     /* 结束角度高字节 */
    uint8_t end_angle_l;     /* 结束角度低字节（单位：0.01度） */
    uint8_t crc;             /* 校验和：字节2-106累加和 */
} LidarPacket;

/* 滤波后的点数据：经过滤波和角度计算后的点信息 */
typedef struct {
    uint16_t angle;      /* 角度（百分之一度，0-36000） */
    uint16_t distance;   /* 距离（毫米） */
    uint8_t valid;       /* 有效标志：1表示有效，0表示无效 */
    uint8_t quality;     /* 数据质量：参与平均的点数（可信度） */
} ProcessedPoint;

/* ============== 函数声明 ============== */

/* 硬件初始化和中断处理 */
void UART_LiDAR_Init(void);             /* 初始化USART2+DMA接收N10P1雷达数据 */
void Process_Lidar_Data_Optimized(void); /* 处理一帧数据：双采样合并、角度计算、异常值剔除、平均值计算 */

/* 角度工具函数 */
uint8_t Is_Angle_Valid(uint16_t angle); /* 检查角度是否在有效扫描范围内（270度-360度和0度-90度） */
uint8_t Is_In_Detect_Area(uint16_t angle, uint16_t start, uint16_t end); /* 检查角度是否在指定检测范围内（支持跨0度） */

/* 迷宫控制函数 */
void Maze_Init(void);                   /* 初始化迷宫全局变量和状态，系统启动时调用 */
void Maze_Main_Task(void);              /* 迷宫主任务：根据Current_Phase分发不同任务 */

/* 右手法则迷宫算法 */
void Right_Hand_Rule_Task(void);        /* 右手法则核心逻辑：右转优先、直走、左转、掉头 */
void Get_Direction_Distances(uint16_t* right, uint16_t* front, uint16_t* left); /* 获取三个方向的最小距离 */
uint8_t Check_Maze_Escaped(void);       /* 检查是否已走出迷宫：前方>2m且右侧>1m且左侧>1m */

/* 探宝任务 */
void Treasure_Hunt_Task(void);          /* 探宝任务主函数：搜寻-撞击-继续循环 */
uint8_t Check_Treasure_Detected(void);  /* 宝藏检测算法：30度范围内距离突变且非边界墙 */
uint8_t Is_Boundary_Wall(uint16_t angle, uint16_t distance); /* 边界墙判断：与左右侧墙距离对比 */

/* ============== 外部变量声明 ============== */
extern LidarPacket Lidar_Pack_Data;         /* 雷达原始数据包（108字节，DMA接收缓冲区） */
extern ProcessedPoint PointBuffer[BUFFER_SIZE]; /* 滤波后的数据缓冲区（最多50点），用于后续计算 */
extern volatile uint8_t Lidar_data_recv;    /* 数据接收标志：1表示接收到完整一圈数据 */
extern volatile uint32_t Lidar_data_cnt;    /* 完整圈数计数器 */
extern uint8_t last_write_cnt;              /* 上一次写入点数 */
extern RunPhase Current_Phase;              /* 当前运行阶段：迷宫/探宝/完成 */
extern uint16_t Treasure_Count;             /* 已发现宝藏计数（撞击成功+1） */
extern uint16_t Front_Distance;             /* 当前前方最小距离（滤波值） */
extern uint8_t Escape_Flag;                 /* 逃脱标志：1表示已走出迷宫 */

#endif /* __LIDAR_H */