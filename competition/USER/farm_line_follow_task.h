#ifndef FARM_LINE_FOLLOW_TASK_H
#define FARM_LINE_FOLLOW_TASK_H

#include <stdint.h>
#include "los_task.h"
#include "linetracker.h"
#include "motor.h"
#include "speed.h"
#include "farm_line_follow.h"

/*
 * 循迹任务：沿黑线连续行驶（顺时针方向由初始车头朝向决定）。
 *
 * 周期 10ms（1 tick），与 PID 速率 PID_RATE=100Hz 一致。
 * 决策全部复用 farm_line_follow.h 的 FarmLineFollow_Step()：
 *   连续位置误差 → 比例转向 + 随误差降速 + 丢线三段式处置。
 * 本层只负责周期调度与执行，不重复实现任何决策逻辑。
 *
 * 注意：在线仿真平台不支持 LiteOS-M 任务调度，仿真时请使用 main.c 的裸机版本。
 *
 * 【符号契约】本层只使用车体物理语义：正值=向前，左/右轮对应车体左/右侧。
 * 极性/取反统一收敛在 motor.c 驱动层，此处绝不叠加取反或左右互换。
 */
static inline void *FarmLineFollow_Task(UINT32 arg)
{
    FarmLineFollowCtx ctx = {0U, 0, 0};

    (void)arg;
    while (1) {
        uint8_t status = GetLineStatus();
        int16_t left = 0, right = 0;

        FarmLineFollow_Step(&ctx, status, &left, &right);

        MOTOR_SetSpeed(left, right);
        LOS_TaskDelay(1);   /* 1 tick = 10ms @ LOSCFG_BASE_CORE_TICK_PER_SECOND=100 */
    }
    return NULL;
}

#endif /* FARM_LINE_FOLLOW_TASK_H */
