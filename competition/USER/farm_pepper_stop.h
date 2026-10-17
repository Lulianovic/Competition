#ifndef FARM_PEPPER_STOP_H
#define FARM_PEPPER_STOP_H
#include <stdint.h>
#include "tcs34725.h"   /* RGB */

/*
 * 辣椒采摘 AGV（方案02 第一步）：遇见红辣椒 → 停车 3 秒 → 到时继续前进
 *
 * 决策层纯逻辑（header-only，static inline），不触碰硬件：
 *   - 红色判定：C ≥ FARM_PEPPER_CMIN 且 R > 1.3×G 且 R > 1.3×B（整数比较，避免浮点）
 *   - 触发条件：处于"已武装"状态时，有效颜色帧判为红 → 触发一次停车
 *   - 停车计时：满 FARM_PEPPER_STOP_MS（3000ms）后放行
 *   - 重新武装：恢复后必须连续 FARM_PEPPER_CLEAR_MS（500ms）读到"有效且非红"的帧，
 *               才允许下一次触发；否则刚起步就会被同一个辣椒又叫停
 *
 * 通信失败的约定：I2C 四通道读取失败的一轮**不产生结论**——
 * 既不算"红"（不误停），也不算"非红"（不计入清除时间），与方案02 文档一致。
 */

/* —— 参数 —— */
#define FARM_PEPPER_CMIN          50U    /* 红色判定：C 下限 */
#define FARM_PEPPER_RATIO_NUM     13U    /* R > NUM/DEN × G、R > NUM/DEN × B */
#define FARM_PEPPER_RATIO_DEN     10U
#define FARM_PEPPER_SAMPLE_TICKS  10U    /* 颜色采样节流：每 10 个控制周期（100ms）读一次。
                                              传感器积分时间 154ms（≈6.5Hz），读得更快只会重复
                                              同一帧，却要让 10ms 控制周期承担一次 I2C 事务
                                              （约 2ms，失败时 I2C_CheckEvent 还会超时轮询更久），
                                              采样间隔太短会把循迹周期拖得忽长忽短，车就开始摆动 */
#define FARM_PEPPER_STOP_MS     3000U    /* 见红后的停车时长 */
#define FARM_PEPPER_CLEAR_MS     500U    /* 恢复后连续"无红"达到该时长才重新允许触发 */

/* —— 状态 —— */
typedef struct {
    uint32_t stop_ms;    /* 已停车时长 */
    uint32_t clear_ms;   /* 连续"有效且非红"的累计时长 */
    uint8_t  armed;      /* 1 = 允许触发下一次停车 */
} FarmPepperStopCtx;

/* 首次触发前已武装：赛道起点无需先等 500ms 无红 */
static inline void FarmPepperStop_Init(FarmPepperStopCtx *c)
{
    c->stop_ms  = 0U;
    c->clear_ms = 0U;
    c->armed    = 1U;
}

/**
  * @简  述  红色判定（整数比较；RGB 为 uint16_t，先提升到 uint32_t 防中间量溢出）
  * @参  数  rgb: 一次完整成功的四通道读数
  * @返回值  1 = 判为红，0 = 非红
  */
static inline uint8_t FarmPepper_IsRed(const RGB *rgb)
{
    uint32_t c = (uint32_t)rgb->C;
    uint32_t r = (uint32_t)rgb->R;
    uint32_t g = (uint32_t)rgb->G;
    uint32_t b = (uint32_t)rgb->B;

    if (c < (uint32_t)FARM_PEPPER_CMIN) {
        return 0U;
    }
    return (uint8_t)((r * (uint32_t)FARM_PEPPER_RATIO_DEN > g * (uint32_t)FARM_PEPPER_RATIO_NUM &&
                      r * (uint32_t)FARM_PEPPER_RATIO_DEN > b * (uint32_t)FARM_PEPPER_RATIO_NUM) ? 1U : 0U);
}

/**
  * @简  述  喂入一次颜色采样，判断是否应当开始停车
  * @参  数  c:          停车状态上下文
  * @参  数  frame_ok:   1 = 本轮四通道读取完整成功；0 = 读失败（本轮无结论）
  * @参  数  red:        frame_ok 为 1 时的判定结果（frame_ok 为 0 时忽略）
  * @参  数  elapsed_ms: 本次采样代表的实际时长（= 采样节流 × 控制周期 = 20ms）
  * @返回值  1 = 本次新触发停车，0 = 不触发
  */
static inline uint8_t FarmPepperStop_NewTrigger(FarmPepperStopCtx *c, uint8_t frame_ok,
                                                uint8_t red, uint32_t elapsed_ms)
{
    if (frame_ok == 0U) {
        return 0U;   /* 读失败：既不计红也不计清除，避免把通信故障当成"没有辣椒" */
    }
    if (c->armed == 0U) {
        if (red == 0U) {
            c->clear_ms += elapsed_ms;
            if (c->clear_ms >= FARM_PEPPER_CLEAR_MS) {
                c->armed = 1U;   /* 红椒已离开视野足够久，重新允许触发 */
            }
        }
        return 0U;
    }
    if (red == 0U) {
        return 0U;
    }
    /* 见红：立即制动，并解除武装，避免恢复后被同一目标连续叫停 */
    c->armed    = 0U;
    c->clear_ms = 0U;
    c->stop_ms  = 0U;
    return 1U;
}

/**
  * @简  述  停车计时推进
  * @参  数  c:          停车状态上下文
  * @参  数  elapsed_ms: 自上次调用经过的时间（= 控制周期 = 10ms）
  * @返回值  1 = 停车时长已满，可以继续前进；0 = 继续停车
  */
static inline uint8_t FarmPepperStop_HoldElapsed(FarmPepperStopCtx *c, uint32_t elapsed_ms)
{
    c->stop_ms += elapsed_ms;
    return (c->stop_ms >= FARM_PEPPER_STOP_MS) ? 1U : 0U;
}

#endif /* FARM_PEPPER_STOP_H */
