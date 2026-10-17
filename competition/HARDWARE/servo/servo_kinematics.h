#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <stdint.h>

/*
 * 机械臂几何参数，单位：米（m）。
 *
 * 逆运动学使用的坐标系：
 *   +X：小车正前方
 *   +Y：小车右侧
 *   +Z：竖直向上
 *
 * S1（底座旋转）和 S5（末端自转）为旋转关节。
 * S6 为机械爪开合舵机，不参与逆运动学计算。
 */
/* 原点为机械臂底座 jointBase 转轴，几何参数单位为米。
 * 长度仅供显示；求解使用完整局部向量，不能将偏心连杆简单相加。
 * 参数来源及验证方法见 ARM_IK_CALIBRATION.md。 */
#define ARM_D1      0.009188179
#define ARM_A2      0.052441205192
#define ARM_A3      0.040861937074
#define ARM_A4      0.035717775642
#define ARM_D5      0.0049
#define ARM_L_TOOL  0.028637152442

/* 关节局部坐标 (右、上、前)，单位：米。 */
#define ARM_SHOULDER_OFFSET {0.014806454, 0.009188179, 0.003524305}
#define ARM_ELBOW_OFFSET    {0.0010479829, 0.052430683, 0.00007220794}
#define ARM_WRIST_OFFSET    {-0.000020282033, 0.0021581534, -0.0408049}
#define ARM_ROLL_OFFSET     {-0.0091482446, 0.034526348, -0.00002028221}
#define ARM_TOOL_OFFSET     {-0.001097347, 0.031545100, -0.001019223}
/* Open-jaw capture-volume centre, measured with S6=-90, not clampingPoint. */
#define ARM_IK_POSITION_TOL_M 0.00010 /* 求解误差 <= 0.1 mm */
#define ARM_COMMAND_POSITION_TOL_M 0.00025 /* 0.1 度量化后的误差 <= 0.25 mm */
#define ARM_IK_ORIENTATION_TOL 0.002 /* 旋转矩阵各元素容差 */

#define PI          3.14159265358979323846
#define DEG2RAD     (PI/180.0)  /* 角度转弧度 */
#define RAD2DEG     (180.0/PI)  /* 弧度转角度 */

/* 关节限位，单位：度。超过此范围视为不可达。 */
#define J1_MIN      (-90.0)
#define J1_MAX      90.0
#define J2_MIN      (-90.0)
#define J2_MAX      90.0
#define J3_MIN      (-90.0)
#define J3_MAX      90.0
#define J4_MIN      (-90.0)
#define J4_MAX      90.0
#define J5_MIN      (-90.0)
#define J5_MAX      90.0

/*
 * 舵机方向与零位校准。
 * 关节角定义与 RoboticArm.cs 一致，协议角度直接对应场景关节角。
 * 若实际舵机转动方向相反，将对应 SIGN 取反即可。
 * ZERO_OFFSET 为舵机 0 度与关节 0 度的偏差，单位：度。
 */
#define S1_SERVO_SIGN   (1.0)
#define S2_SERVO_SIGN   (1.0)
#define S3_SERVO_SIGN   (1.0)
#define S4_SERVO_SIGN   (1.0)
#define S5_SERVO_SIGN   (1.0)

#define S1_ZERO_OFFSET  (0.0)
#define S2_ZERO_OFFSET  (0.0)
#define S3_ZERO_OFFSET  (0.0)
#define S4_ZERO_OFFSET  (0.0)
#define S5_ZERO_OFFSET  (0.0)

#define STEP_NUM    20      /* 平滑运动插补步数 */
#define STEP_DELAY  25      /* 每步延时（ms），当前未使用 */

/* 6D 位姿结构体（可扩展用于上层任务） */
typedef struct {
    double x, y, z;
    double roll, pitch, yaw;
} Pose6D;

/* 5 关节角度结构体 */
typedef struct {
    double theta[5];
} JointAngles;

/* 正运动学：由 5 个舵机角度（弧度）计算 4x4 齐次变换矩阵 T */
void Kinematics_Forward(const double theta[5], double T[4][4]);

/* 完整逆运动学：不可实现的姿态返回 0，不投影/修改请求姿态。 */
int  Kinematics_Inverse(const double T_target[4][4], double solutions[4][5]);

/* 从多组解中选择与当前角度最接近的一组 */
void Kinematics_SelectOptimal(const double current[5], const double sols[4][5],
                              int num, double optimal[5]);

/* 4x4 矩阵乘法 */
void Matrix4x4_Mult(const double A[4][4], const double B[4][4], double C[4][4]);

/* 4x4 矩阵与 4 维向量相乘 */
void Matrix4x4_VecMult(const double T[4][4], const double v[4], double result[4]);

/* 仅位置逆运动学：由目标 (x, y, z) 与期望工具偏航角求解舵机角度 */
int Kinematics_InversePositionOnly(double x, double y, double z,
                                   double desired_tool_yaw,
                                   double solutions[4][5]);



/*
 * 运动到指定目标位置。
 * 原点为底座转轴，方向为车体 +X 前方，+Y 右侧，+Z 向上。
 * S5 保持当前角度，并在求解时计入其偏心夹爪对末端位置的影响。
 */
int move_to_position(double x, double y, double z);
int Kinematics_PositionFixedWrist(double x,double y,double z,
                                  const double current[5],double solutions[4][5]);

/* 先抬升并移动至筐口上方，再释放并回正；规划失败返回-1且不松爪。 */
int release_gripper(void);

/* 舵机测试函数：初始化并执行一个简单抓取动作 */
void servo_test(void);

#endif
