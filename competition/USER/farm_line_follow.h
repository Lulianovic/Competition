#ifndef FARM_LINE_FOLLOW_H
#define FARM_LINE_FOLLOW_H
#include <stdint.h>
#include "farm_speed_config.h"

/*
 * 八路巡线循迹决策层（连续比例控制）。
 *
 * 传感器从左到右为 S1～S8；GetLineStatus() 的约定是「压线为 0、浅色为 1」，
 * 因此本层取 on = ~status，on 的第 i 位为 1 表示第 i 路压在黑线上。
 *
 * 【居中判据（两道，先硬后软）】
 *   ① **中心锁定**（`FARM_LINE_CENTER_MASK`，首选）：正中两路 S4、S5 同时压线，
 *      说明线就在车体中心走廊内 → 直接判定居中并直行，不参与质心解算，**产生任何
 *      纠偏量都被禁止**。这是硬判据，因为它描述的是几何事实（线在正中间），
 *      不依赖"质心是否足够接近 0"这类阈值，因而不受邻路偶然压线/抖动影响。
 *   ② **死区**（`FARM_LINE_ERR_DEADBAND`，兜底）：中心两路未同时压线、但质心仍在
 *      中心附近时（如 S4 单独压线 err = -1、S5+S6 err = +2），同样按居中处理。
 *   两条合起来保证"居中时绝不纠偏"：这是实测故障（正对直线却左右摆动）的正面修正。
 *
 * 【位置误差】对压线的各路按权重 (2i-7)（S1=-7 … S8=+7）求平均，得到连续误差
 * err ∈ [-7, +7]：err < 0 线偏左，err > 0 线偏右，左右对称时 err = 0。
 * 相比"某一路命中即判档"的离散方案，误差连续、单调，不会因某一路偶然命中而跳变。
 *
 * 【转向】turn = TURN_GAIN × err（限幅 ±TURN_MAX），
 * 左轮 = forward + turn，右轮 = forward - turn。
 * err > 0（线偏右）→ 左轮加速、右轮减速 → 车头右转回线，物理语义自洽。
 *
 * 【降速】forward = BASE - (BASE - MIN_FORWARD) × |err| / 7。
 * 偏离越大前进速度越低，转弯半径越小、纠偏越快；一旦回到中心立即恢复常速。
 *
 * 【丢线】status == 0xFF 为八路全白，分四段处置：
 *   ① 间隙保持（lost_ticks ≤ HOLD_TICKS）
 *      线落在两传感器之间时同样表现为全白。此阶段沿用最后一次采样的误差
 *      继续修正（已按 |err| 降速），等线重新进入传感器即恢复闭环，
 *      避免把"间隙"误判成"离线"而直冲下线。
 *   ② 居中丢线保持直行（|last_err| ≤ 1 且 ≤ STRAIGHT_HOLD_TICKS）
 *      车体基本居中时线整体消失，典型情形是**白色起点/终点箭头把黑线断开**。
 *      此时必须继续直行等线回来；若照丢线方向旋转，就会在还没驶过箭头时原地打转
 *      （实测故障：起步未过白箭头就开始转向）。该窗口按时间封顶，超时转入 ③。
 *   ③ 双倍幅度旋转找回（≤ HOLD_TICKS + SEARCH_TICKS）
 *      偏离中丢线（如弯道来不及转、线整体滑出阵列）：按 ctx->last_dev 记录的
 *      "最近的偏离点"方向，以 2× 正常差速、低速旋转回去找线；内轮允许反转，
 *      形成原地转回，比正常纠偏更快把线扫回传感器下方。
 *      若从未发生过偏离（last_dev == 0），按顺时针赛道默认向右转回。
 *   ④ 停车保护（> HOLD_TICKS + SEARCH_TICKS）
 *      长时间未找回，停车，防止无控冲出场地。
 *
 * 【符号契约】本层只使用车体物理语义：正值 = 向前，左/右轮对应车体左/右侧。
 * 极性/左右映射统一收敛在 motor.c 驱动层，此处不叠加取反或左右互换。
 * 若实测转向与偏离方向相反（传感器左右装反），只改 farm_speed_config.h 中的
 * FARM_LINE_TURN_SIGN 一处，勿在其它层重复取反。
 */

#define FARM_LF_ALL_WHITE   0xFFU   /* 八路全白（GetLineStatus 约定） */
#define FARM_LF_ERR_SPAN    7       /* 位置误差归一化上限 */
#define FARM_LF_SENSOR_NUM  8U      /* 传感器路数 */

/* 循迹上下文：跨控制周期保存丢线计数与偏离记忆。 */
typedef struct {
    uint32_t lost_ticks;   /* 连续全白周期数 */
    int16_t  last_err;     /* 最近一次采样的位置误差（含 0）：用于间隙保持 */
    int16_t  last_dev;     /* 最近一次非零偏离（-7 ～ +7，偏离点记忆）：丢线后旋转找回 */
} FarmLineFollowCtx;

/**
  * @简  述  由巡线状态计算连续位置误差（含中心锁定与死区，居中时恒为 0）
  * @参  数  status: GetLineStatus() 返回值
  * @返回值  -7（线最靠左）～ +7（线最靠右）；全白无位置信息时返回 0；
  *          中心两路同时压线（中心锁定）或 |err| ≤ FARM_LINE_ERR_DEADBAND（死区）时返回 0
  */
static inline int16_t FarmLineFollow_Error(uint8_t status)
{
    uint8_t on = (uint8_t)(~status);
    int16_t sum = 0;
    uint8_t cnt = 0U;
    uint8_t i;
    int16_t err;

    /* 中心锁定（第一优先判据）：正中两路 S4、S5 同时压线 ⇒ 线就在车体中心走廊内
     * ⇒ 直接判定居中、直行，**不参与质心解算、不产生任何纠偏量**。
     * 必须是硬判据而不是"质心接近 0 就放过"：线宽与传感器间距会使压线路数在
     * 2~3 路之间抖（S4+S5 与 S3+S4 交替），单靠质心阈值仍会在某些周期放出 ±2 的误差
     * 并翻转转向 —— 那就是"中间两灯亮着车还在左右纠偏"的根源。 */
    if ((on & (uint8_t)FARM_LINE_CENTER_MASK) == (uint8_t)FARM_LINE_CENTER_MASK) {
        return 0;
    }

    for (i = 0U; i < FARM_LF_SENSOR_NUM; i++) {
        if (on & (uint8_t)(1U << i)) {
            sum = (int16_t)(sum + (int16_t)(2 * (int16_t)i - 7));
            cnt++;
        }
    }
    if (cnt == 0U) {
        return 0;   /* 全白无位置信息，由丢线状态机接管 */
    }
    /* 向零截断会把 sum/cnt 中的小数抹平（如 sum=-1、cnt=3 得 0），
     * 造成"偏离却不纠偏"的隐性死区；此处改用带符号的向上取整，
     * 保证只要加权质心偏离中心（sum != 0）就输出非零误差。 */
    if (sum >= 0) {
        err = (int16_t)((int16_t)(sum + (int16_t)cnt - 1) / (int16_t)cnt);
    } else {
        err = (int16_t)(-((int16_t)(-sum + (int16_t)cnt - 1) / (int16_t)cnt));
    }

    /* 死区（第二道）：|err| ≤ FARM_LINE_ERR_DEADBAND 时按居中处理（err = 0，左右等速直行）。
     * 覆盖"中心两路没同时压线、但质心仍在中心附近"的抖动形态（如 S4 单独压线 err=-1、
     * S5+S6 err=+2），没有它这些形态会被放大成左右翻转的差速。 */
    if (err <= (int16_t)FARM_LINE_ERR_DEADBAND && err >= -(int16_t)FARM_LINE_ERR_DEADBAND) {
        return 0;
    }
    return err;
}

static inline int16_t FarmLineFollow_Clamp(int32_t v, int16_t lo, int16_t hi)
{
    if (v < (int32_t)lo) {
        return lo;
    }
    if (v > (int32_t)hi) {
        return hi;
    }
    return (int16_t)v;
}

static inline int16_t FarmLineFollow_Abs(int16_t v)
{
    return (v < 0) ? (int16_t)(-v) : v;
}

/**
  * @简  述  由位置误差直接解算左右轮目标速度（降速 + 比例转向 + 限幅）
  * @参  数  err:   位置误差 -7 ～ +7
  * @参  数  left:  输出，左轮目标速度（正值向前）
  * @参  数  right: 输出，右轮目标速度（正值向前）
  * @返回值  无
  */
static inline void FarmLineFollow_Drive(int16_t err, int16_t *left, int16_t *right)
{
    int16_t mag = (err < 0) ? (int16_t)(-err) : err;
    int32_t forward = (int32_t)FARM_LINE_BASE_SPEED -
                      ((int32_t)FARM_LINE_BASE_SPEED - (int32_t)FARM_LINE_MIN_FORWARD_SPEED)
                          * (int32_t)mag / FARM_LF_ERR_SPAN;
    int32_t turn = (int32_t)FARM_LINE_TURN_GAIN * (int32_t)err * FARM_LINE_TURN_SIGN;

    if (turn > (int32_t)FARM_LINE_TURN_MAX) {
        turn = FARM_LINE_TURN_MAX;
    } else if (turn < -(int32_t)FARM_LINE_TURN_MAX) {
        turn = -FARM_LINE_TURN_MAX;
    }

    *left  = FarmLineFollow_Clamp(forward + turn, FARM_LINE_WHEEL_MIN, FARM_LINE_WHEEL_MAX);
    *right = FarmLineFollow_Clamp(forward - turn, FARM_LINE_WHEEL_MIN, FARM_LINE_WHEEL_MAX);
}

/**
  * @简  述  丢线后的双倍幅度旋转找回：沿最近的偏离点方向低速转回
  * @参  数  err:   偏离点记忆（-7 ～ +7），同时决定转回的方向与幅度
  * @参  数  left:  输出，左轮目标速度
  * @参  数  right: 输出，右轮目标速度
  * @返回值  无
  * @注  意  该模式允许内轮反转（原地转回），故不受 WHEEL_MIN 限幅约束，
  *          仅限幅在 ±WHEEL_MAX 内。偏离幅度越大，转回越猛（2× 正常差速）。
  */
static inline void FarmLineFollow_SpinSearch(int16_t err, int16_t *left, int16_t *right)
{
    int32_t turn = (int32_t)FARM_LINE_TURN_GAIN * FARM_LINE_LOST_TURN_SCALE
                   * (int32_t)err * FARM_LINE_TURN_SIGN;

    if (turn > (int32_t)FARM_LINE_LOST_TURN_MAX) {
        turn = FARM_LINE_LOST_TURN_MAX;
    } else if (turn < -(int32_t)FARM_LINE_LOST_TURN_MAX) {
        turn = -FARM_LINE_LOST_TURN_MAX;
    }

    *left  = FarmLineFollow_Clamp((int32_t)FARM_LINE_LOST_FORWARD_SPEED + turn,
                                  -FARM_LINE_WHEEL_MAX, FARM_LINE_WHEEL_MAX);
    *right = FarmLineFollow_Clamp((int32_t)FARM_LINE_LOST_FORWARD_SPEED - turn,
                                  -FARM_LINE_WHEEL_MAX, FARM_LINE_WHEEL_MAX);
}

/**
  * @简  述  单周期循迹决策：正常闭合控制 + 丢线三段式处置
  * @参  数  ctx:    循迹上下文（首次使用前须清零）
  * @参  数  status: GetLineStatus() 返回值
  * @参  数  left:   输出，左轮目标速度
  * @参  数  right:  输出，右轮目标速度
  * @返回值  无
  */
static inline void FarmLineFollow_Step(FarmLineFollowCtx *ctx, uint8_t status,
                                       int16_t *left, int16_t *right)
{
    if (status != FARM_LF_ALL_WHITE) {
        /* 有线：闭环连续修正，同时记录"最近的偏离点"供丢线后转回 */
        int16_t err = FarmLineFollow_Error(status);
        ctx->last_err   = err;
        ctx->lost_ticks = 0U;
        if (err != 0) {
            ctx->last_dev = err;
        }
        FarmLineFollow_Drive(err, left, right);
        return;
    }

    /* 全白：可能是线落在两传感器间隙，也可能是真离线，按时间分四段处置 */
    ctx->lost_ticks++;
    if (ctx->lost_ticks <= (uint32_t)FARM_LF_HOLD_TICKS) {
        /* ① 间隙保持：沿用最后一次采样的误差继续修正（已随误差降速） */
        FarmLineFollow_Drive(ctx->last_err, left, right);
    } else if (FarmLineFollow_Abs(ctx->last_err) <= 1 &&
               ctx->lost_ticks <= (uint32_t)FARM_LF_STRAIGHT_HOLD_TICKS) {
        /* ② 居中丢线（车体基本居中时线整体消失：白色起点/终点箭头把黑线断开）：
         *    保持直行等线回到传感器下方。此时若按偏离点旋转，车就会在还没过箭头时
         *    原地打转 —— 这就是"起步未过白箭头就开始转向"的直接原因。 */
        *left  = FARM_LINE_BASE_SPEED;
        *right = FARM_LINE_BASE_SPEED;
    } else if (ctx->lost_ticks <= (uint32_t)(FARM_LF_HOLD_TICKS + FARM_LF_SEARCH_TICKS)) {
        /* ③ 双倍幅度旋转找回：偏离中丢线（弯道来不及转、线整体滑出阵列）时，
         *    沿最近的偏离点方向转回；无偏离记忆时默认向右（顺时针赛道） */
        int16_t dev = (ctx->last_dev != 0) ? ctx->last_dev : (int16_t)FARM_LINE_LOST_DEFAULT_ERR;
        FarmLineFollow_SpinSearch(dev, left, right);
    } else {
        /* ④ 停车保护 */
        *left  = 0;
        *right = 0;
    }
}

#endif /* FARM_LINE_FOLLOW_H */
