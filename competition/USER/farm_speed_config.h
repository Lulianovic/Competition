#ifndef FARM_SPEED_CONFIG_H
#define FARM_SPEED_CONFIG_H

/*
 * 循迹速度与转向参数（连续比例控制，见 farm_line_follow.h）。
 *
 * 单位与 MOTOR_SetSpeed 一致：PWM 目标值，范围 ±4200，电机起转阈值约 400。
 * 命名含义：forward = 前进基准速度，turn = 左右轮差速（转向量）。
 * 调参顺序：先定 BASE_SPEED，再调 TURN_GAIN 至能压住线，最后用
 * MIN_FORWARD_SPEED 控制弯道减速幅度。
 */

/* —— 起步 —— */
#define FARM_START_SPEED              1000 /* 白色起点驶离速度 */

/* —— 前进速度：偏离越大越慢（"需要调整时先降速"）—— */
#define FARM_LINE_BASE_SPEED           650 /* 完全居中时的前进速度。
                                              抑制摆动的通用手段：摆动幅度与速度正相关，
                                              先降低巡航速度把幅度压小，机制查清后再提速 */
#define FARM_LINE_MIN_FORWARD_SPEED    450 /* 最大偏离（|err|=7）时的前进速度 */

/* —— 转向：turn = TURN_GAIN × err，误差范围 ±7 —— */
#define FARM_LINE_CENTER_MASK         0x18 /* 中心两路掩码：位 3 = S4、位 4 = S5。
                                              只要这两路同时压线 ⇒ 线在中心走廊内 ⇒
                                              强制直行、不产生任何纠偏量（硬判据，优先于质心） */
#define FARM_LINE_ERR_DEADBAND           1 /* 误差死区：|err| ≤ 1 时不纠偏，左右轮等速直行。
                                              原取 2：为覆盖线在中心附近时压线路数在 2~3 路
                                              之间抖动的形态（S4+S5 与 S3+S4 交替）。
                                              但 S3+S4 的质心恰为 -2，死区 2 会把它判成居中，
                                              车稳定偏一个灯位却完全不纠偏（实测 st 恒 0xF3、
                                              err 恒 0、左右轮等速 650）。改为 1：偏一格即
                                              产生 ±90 差速纠偏（约基准速度的 14%，力度温和）。
                                              代价：中心附近可能在"纠 90 / 锁 0"之间形成轻微
                                              蛇形（带宽小、幅度有限，可接受）；若实测摆动过大，
                                              优先下调 FARM_LINE_TURN_GAIN 而非把死区改回 2 */
#define FARM_LINE_TURN_GAIN             45 /* 比例增益：每单位误差对应的差速（过大→摆动） */
#define FARM_LINE_TURN_MAX             600 /* 差速上限，防极端情况转向过量 */
#define FARM_LINE_TURN_SIGN              1 /* 方向契约：+1 = S1 侧为车体左侧；
                                              实测转向相反时只改这一处为 -1 */

/* —— 轮速限幅：下限高于电机起转阈值，保证内轮仍在转动而非堵转 —— */
#define FARM_LINE_WHEEL_MIN            400
#define FARM_LINE_WHEEL_MAX           1600

/* —— 丢线处置（单位：控制周期，1 周期 = 10ms）—— */
#define FARM_LF_HOLD_TICKS              20 /* 200ms：线落传感器间隙时的保持窗口 */
#define FARM_LF_STRAIGHT_HOLD_TICKS    100 /* 1.0s：丢线时若车体居中（白色起点/终点箭头把
                                              黑线断开），继续直行等线回来，不做旋转找回 */
#define FARM_LF_SEARCH_TICKS           300 /* 3s：旋转找回时限，超时停车保护 */

/* —— 丢线后的"双倍幅度旋转找回"（沿最近一次偏离方向转回）—— */
#define FARM_LINE_LOST_TURN_SCALE        2 /* 转向幅度倍数：2× 正常差速 */
#define FARM_LINE_LOST_TURN_MAX        900 /* 旋转找回时的差速上限（比正常模式更大） */
#define FARM_LINE_LOST_FORWARD_SPEED   400 /* 旋转找回时的前进速度（低速，便于转回） */
#define FARM_LINE_LOST_DEFAULT_ERR       5 /* 无偏离记忆时的默认幅度与方向：+5 = 向右
                                              （顺时针赛道），仅在前向从未偏离过时使用 */

#endif /* FARM_SPEED_CONFIG_H */
