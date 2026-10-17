/**
 * 辣椒采摘 AGV 主程序（裸机循环版）。
 *
 * 采用裸机 while(1) 循环 + delay_ms 实现 10ms 周期控制，
 * 不依赖 LiteOS-M 任务调度，以兼容在线仿真环境。
 * 后续侦察辣椒等动作可在 while(1) 中以状态机方式追加。
 *
 * 循迹决策全部集中在 farm_line_follow.h 的 FarmLineFollow_Step()：
 * 连续位置误差 → 比例转向 + 随误差降速 + 丢线三段式处置。
 * 红辣椒处置集中在 farm_pepper_stop.h（方案02 第一步：见红停车 3 秒后继续前进）。
 * 夹取的固定角度序列在本文件；放筐走 servo_kinematics.c 的 release_gripper()，
 * 由逆运动解算到 FARM_BIN_DROP_* 定义的筐口上方再松爪。
 * 本文件只负责周期调度与状态编排。
 */
#include "sys.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "motor.h"
#include "encoder.h"
#include "speed.h"
#include "linetracker.h"
#include "tcs34725.h"
#include "usart_debug.h"
#include "delay.h"
#include "farm_line_follow.h"
#include "farm_speed_config.h"
#include "farm_pepper_stop.h"
#include "servo.h"
#include "servo_kinematics.h"   /* 逆运动学：move_to_position() / release_gripper() */

/* speed.c 里的实测轮速（m/s），仅遥测打印用；speed.h 未声明，故在此补 extern */
extern double Wheel_left_speed;
extern double Wheel_right_speed;

/* 循迹参数 */
#define FARM_LF_PERIOD_MS           10U   /* 控制周期：10ms */
#define FARM_PEPPER_SAMPLE_MS       (FARM_PEPPER_SAMPLE_TICKS * FARM_LF_PERIOD_MS) /* 颜色采样代表的时长 */
#define FARM_DEBUG_BAUD         115200U   /* 调试串口 USART3（串口助手） */
#define FARM_BOOT_DELAY_MS      2000U     /* 上电缓冲：主循环开始前纯等待 2 秒，不做任何动作 */

/*
 * 日志开关（只控制本文件的打印）。
 * 注意：printf 走 usart_debug.c 的 _write()，它会在 USART3 未使能时死等 TXE 而卡死，
 * 所以 init 里必须先做 USART3_Init()；TCS34725_Init() 内部也有一句 printf，
 * 同样依赖这一步，故本宏并不能免除 USART3 的初始化。
 */
#define FARM_LOG_ENABLE              1
#if FARM_LOG_ENABLE
#define FARM_LOG(...)  printf(__VA_ARGS__)
#else
#define FARM_LOG(...)  do { } while (0)
#endif

/* 夹具原始角度：S1=0°, S2=0°, S3=0°, S4=0°, S5=0°, S6=-90°（单位 0.1°） */
static const int16_t FARM_GRIPPER_HOME[6] = {0, 0, 0, 0, 0, -900};
/* 检测到红色后依次调整到的目标角度：S1=90°, S2=10°, S3=70°, S4=40°, S5=0°, S6=0° */
static const int16_t FARM_GRIPPER_TARGET[6] = {900, 100, 700, 400, 0, 0};

/**
 * @brief 将夹具设置到原始角度，作为上电/回位位置。
 */
static void FarmGripper_SetHome(void)
{
    int16_t arm[5] = {0, 0, 0, 0, 0};
    set_arm_servos(arm);
    set_gripper_servo(FARM_GRIPPER_HOME[5]);
}

/**
 * @brief 检测到红辣椒后，依次将夹具从原始角度调整到目标角度。
 *        动作顺序：S1 → S2 → S3 → S4 → S6；
 *        S5 目标值已是 0°，无需单独动作。
 */
static void FarmGripper_MoveToTargetSequence(void)
{
    int16_t arm[5] = {0, 0, 0, 0, 0};

    /* 从原始角度出发 */
    set_arm_servos(arm);
    set_gripper_servo(FARM_GRIPPER_HOME[5]);
    SMART_DELAY_MS(600);

    /* 依次调整各关节到目标角度 */
    arm[0] = FARM_GRIPPER_TARGET[0];   /* S1 = 90° */
    set_arm_servos(arm);
    SMART_DELAY_MS(600);

    arm[1] = FARM_GRIPPER_TARGET[1];   /* S2 = 10° */
    set_arm_servos(arm);
    SMART_DELAY_MS(600);

    arm[2] = FARM_GRIPPER_TARGET[2];   /* S3 = 70° */
    set_arm_servos(arm);
    SMART_DELAY_MS(600);

    arm[3] = FARM_GRIPPER_TARGET[3];   /* S4 = 40° */
    set_arm_servos(arm);
    SMART_DELAY_MS(600);

    /* S5 目标为 0°，与原始角度相同，不单独动作 */
    set_gripper_servo(FARM_GRIPPER_TARGET[5]);  /* S6 = 0° */
    SMART_DELAY_MS(600);
}

/**
 * @brief 夹具动作完成后恢复到原始角度，以便小车继续循迹。
 */
static void FarmGripper_ReturnHome(void)
{
    int16_t arm[5] = {0, 0, 0, 0, 0};
    set_arm_servos(arm);
    set_gripper_servo(FARM_GRIPPER_HOME[5]);
    SMART_DELAY_MS(600);
}

/* —— 起步阶段（穿越白色起点箭头）—— */
#define FARM_START_LINE_CONFIRM_MS   50U   /* 连续看到黑线该时长才算"已进入赛道" */
#define FARM_START_TIMEOUT_MS       3000U  /* 起步直行上限：超时仍未见线则停车告警 */

/* —— 循迹遥测（排查摆动用；不需要时置 0）—— */
#define FARM_TRACE_LINE              1
#define FARM_TRACE_TICKS            50U    /* 每 50 周期（500ms）打印一行 状态/误差/左右轮 */

/* 运行状态 */
#define FARM_STATE_STARTUP   0   /* 启动穿越白色箭头 */
#define FARM_STATE_FOLLOW    1   /* 正常循迹 */
#define FARM_STATE_STOP_RED  2   /* 遇见红辣椒：停车计时 */

/* 系统入口：硬件初始化 → 主循环（循迹 + 红辣椒停车）。 */
int main(void)
{
    uint8_t state = FARM_STATE_STARTUP;
    uint32_t startup_ms = 0;
    uint32_t black_ms = 0;                   /* 起步阶段连续看到黑线的时长 */
    uint16_t white_ticks = 0;                /* 循迹阶段连续全白的周期数（仅用于日志） */
    uint16_t trace_ticks = 0;                /* 循迹遥测打印节流 */
    uint16_t st_changes = 0;                 /* 遥测窗口内巡线状态变化次数 */
    uint8_t  trace_prev_st = FARM_LF_ALL_WHITE;
    uint8_t  trace_first = 1U;               /* 遥测窗口内是否还没有样本 */
    int16_t  err_min = 0, err_max = 0;       /* 窗口内误差范围 */
    int16_t  cmd_lmin = 0, cmd_lmax = 0;     /* 窗口内左轮命令范围 */
    int16_t  cmd_rmin = 0, cmd_rmax = 0;     /* 窗口内右轮命令范围 */
    int16_t  spd_lmin = 0, spd_lmax = 0;     /* 窗口内左轮实测速度范围（mm/s） */
    int16_t  spd_rmin = 0, spd_rmax = 0;     /* 窗口内右轮实测速度范围（mm/s） */
    uint8_t  color_ticks = 0U;               /* 颜色采样节流计数 */
    RGB      rgb = {0, 0, 0, 0};             /* 最近一次成功读到的四通道值 */
    FarmLineFollowCtx follow_ctx = {0U, 0, 0};
    FarmPepperStopCtx pepper_ctx;

    /* 1. 外设初始化 */
    USART3_Init(FARM_DEBUG_BAUD);    /* 必须先于任何 printf：否则 _write 会死等 TXE 卡死 */
    MOTOR_Init();
    ENCODER_Left_Init();
    ENCODER_Right_Init();
    LineTracker_Init();
    TCS34725_Init();          /* 颜色传感器（I2C2，内含约 1s 上电稳定延时，失败不阻塞） */
    SERVO_S1_Init();
    SERVO_S2_Init();
    SERVO_S3456_Init();       /* 机械臂 S3-S6 舵机 PWM 初始化 */
    MOTOR_PID_Reset();
    FarmPepperStop_Init(&pepper_ctx);
    FarmGripper_SetHome();    /* 上电后夹具回到原始角度 */
    FARM_LOG("[Farm] init done, boot delay %ums\r\n", (unsigned)FARM_BOOT_DELAY_MS);

    /* 2. 上电缓冲：什么都不做，只等 2 秒（等传感器/通信稳定）再进入主循环 */
    SMART_DELAY_MS(FARM_BOOT_DELAY_MS);

    /* 3. 主循环：10ms 周期 */
    while (1) {
        uint8_t status = GetLineStatus();
        uint8_t trigger = 0U;
        int16_t left = 0, right = 0;

        /* 颜色采样：仅在循迹状态、每 FARM_PEPPER_SAMPLE_TICKS 个周期读一次 */
        if (state == FARM_STATE_FOLLOW &&
            ++color_ticks >= FARM_PEPPER_SAMPLE_TICKS) {
            uint8_t frame_ok = 0U, red = 0U;

            color_ticks = 0U;
            if (TCS34725_TryReadLatest_RGBData(&rgb) == 0U) {
                frame_ok = 1U;                          /* 四通道完整成功才算一帧 */
                red = FarmPepper_IsRed(&rgb);
            }
            /* 读失败时 frame_ok = 0：本轮无结论，既不算红也不算"没有辣椒" */
            trigger = FarmPepperStop_NewTrigger(&pepper_ctx, frame_ok, red, FARM_PEPPER_SAMPLE_MS);
        }

        if (state == FARM_STATE_STARTUP) {
            /* 起步阶段：从白色起点箭头直行进入赛道，此阶段忽略巡线与颜色。
             * 八路全白是正常的（起点区没有黑线），因此**不做任何丢线处置**；
             * 必须等到连续看到黑线（FARM_START_LINE_CONFIRM_MS）才交棒给循迹层，
             * 否则循迹层会把"起点全白"当成离线，在还没驶过箭头时原地旋转找回。 */
            left  = FARM_START_SPEED;
            right = FARM_START_SPEED;
            startup_ms += FARM_LF_PERIOD_MS;
            if (status != FARM_LF_ALL_WHITE) {
                black_ms += FARM_LF_PERIOD_MS;
                if (black_ms >= FARM_START_LINE_CONFIRM_MS) {
                    state = FARM_STATE_FOLLOW;  /* 已进入赛道，此后不再回到起步 */
                    follow_ctx.lost_ticks = 0U; /* 交棒时清丢线计数 */
                    FARM_LOG("[Farm] line found in %ums, follow begin\r\n",
                             (unsigned)startup_ms);
                }
            } else {
                black_ms = 0U;
            }
            if (state == FARM_STATE_STARTUP && startup_ms >= FARM_START_TIMEOUT_MS) {
                left  = 0;      /* 起步直行 3s 仍未见黑线：停车告警，防止无控冲出场地 */
                right = 0;
                FARM_LOG("[Farm] startup timeout: no line in 3000ms, stop\r\n");
            }
        } else if (state == FARM_STATE_STOP_RED) {
            /* 停车阶段：目标速度置零，计时 3 秒后恢复前进 */
            left  = 0;
            right = 0;
            if (FarmPepperStop_HoldElapsed(&pepper_ctx, FARM_LF_PERIOD_MS)) {
                FarmGripper_ReturnHome();   /* 停车结束前夹具回到原始角度 */
                MOTOR_PID_Reset();          /* 清掉停车期间的输出再交回循迹 */
                follow_ctx.lost_ticks = 0U; /* 停车期间没读巡线，清掉旧的全白计数 */
                state = FARM_STATE_FOLLOW;
                FARM_LOG("[Farm] stop 3000ms done, go on\r\n");
            }
        } else if (trigger) {
            /* 见到红辣椒：立即制动 → 夹具动作夹取 → 放筐 → 进入 3 秒停车计时 */
            left  = 0;
            right = 0;
            MOTOR_SetSpeed(left, right);   /* 先立即制动，再执行夹具动作 */
            FARM_LOG("[Farm] red pepper: R/G/B/C = %u/%u/%u/%u, gripper sequence begin\r\n",
                     (unsigned)rgb.R, (unsigned)rgb.G, (unsigned)rgb.B, (unsigned)rgb.C);
            FarmGripper_MoveToTargetSequence();

            /* 放筐：保持夹紧抬臂 → 转向筐口 → 移到筐口上方 → 松爪落入 → 抬臂回正 */
            (void)release_gripper();
            state = FARM_STATE_STOP_RED;
            FARM_LOG("[Farm] gripper sequence done, stop 3000ms\r\n");
        } else {
            /* 循迹阶段：连续比例修正 + 丢线处置（间隙保持/居中直行/旋转找回/停车保护）。
             * 全白持续超过间隙保持窗口时打一条日志，便于确认丢线处置正在介入。 */
            if (status == FARM_LF_ALL_WHITE) {
                if (++white_ticks == (uint16_t)(FARM_LF_HOLD_TICKS + 1U)) {
                    FARM_LOG("[Farm] all-white over %ums, lost handling\r\n",
                             (unsigned)(FARM_LF_HOLD_TICKS * FARM_LF_PERIOD_MS));
                }
            } else {
                white_ticks = 0U;
            }
            FarmLineFollow_Step(&follow_ctx, status, &left, &right);
#if FARM_TRACE_LINE
            /* 遥测（每 500ms 一行）：把一窗口的数据压成"最值 + 变化次数"，
             * 用于区分三种"看着对齐却还在摆"的可能：
             *   ① chg 很大（几十次/500ms）且 err 范围大 → 巡线状态在抖（传感器/线宽），
             *      命令被左右反复翻 → 应对 err 做低通再进死区；
             *   ② chg 很小（2~4 次/500ms）且 err 范围约 ±2 → 缓慢的"漂移-纠正"极限环
             *      （两轮有固有速度差 + 死区/增益偏软）→ 需要积分修正或加大增益；
             *   ③ chg ≈ 0、err 恒为 0、L/R 命令恒定，但实测 vL/vR 仍在变 → 命令没问题，
             *      摆动来自轮速环或机械（左右电机特性不一致），改 speed.c 一侧。 */
            {
                int16_t e  = FarmLineFollow_Error(status);
                int16_t vl = (int16_t)(Wheel_left_speed * 1000.0);
                int16_t vr = (int16_t)(Wheel_right_speed * 1000.0);

                if (trace_first != 0U) {
                    trace_first = 0U;
                    err_min = err_max = e;
                    cmd_lmin = cmd_lmax = left;
                    cmd_rmin = cmd_rmax = right;
                    spd_lmin = spd_lmax = vl;
                    spd_rmin = spd_rmax = vr;
                } else {
                    if (e < err_min)  { err_min = e; }  else if (e > err_max)  { err_max = e; }
                    if (left < cmd_lmin)  { cmd_lmin = left; }  else if (left > cmd_lmax)  { cmd_lmax = left; }
                    if (right < cmd_rmin) { cmd_rmin = right; } else if (right > cmd_rmax) { cmd_rmax = right; }
                    if (vl < spd_lmin) { spd_lmin = vl; } else if (vl > spd_lmax) { spd_lmax = vl; }
                    if (vr < spd_rmin) { spd_rmin = vr; } else if (vr > spd_rmax) { spd_rmax = vr; }
                }
                if (status != trace_prev_st) {
                    trace_prev_st = status;
                    st_changes++;
                }
                if (++trace_ticks >= (uint16_t)FARM_TRACE_TICKS) {
                    trace_ticks = 0U;
                    FARM_LOG("[Farm] st=0x%02X chg=%u err[%d,%d] L[%d,%d] R[%d,%d] vL[%d,%d] vR[%d,%d]\r\n",
                             (unsigned)trace_prev_st,
                             (unsigned)st_changes,
                             (int)err_min, (int)err_max,
                             (int)cmd_lmin, (int)cmd_lmax,
                             (int)cmd_rmin, (int)cmd_rmax,
                             (int)spd_lmin, (int)spd_lmax,
                             (int)spd_rmin, (int)spd_rmax);
                    st_changes   = 0U;
                    trace_first  = 1U;
                }
            }
#else
            (void)trace_ticks;   /* 关闭遥测时避免"变量未使用"告警 */
#endif
        }

        MOTOR_SetSpeed(left, right);
        delay_ms(FARM_LF_PERIOD_MS);
    }
}
