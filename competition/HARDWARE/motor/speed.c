#include "speed.h"
#include "motor.h"
#include "encoder.h"
#include <stdio.h>
// AGV 当前速度
int16_t CurVelocity_Left = 0, CurVelocity_Right = 0;


// PID 控制参数
// 仿真环境电机为理想模型，实物增益(kp=0.5,ki=0.5,kd=0.1)会导致
// 两轮独立PID因编码器噪声/相位滞后不同步、轮流过冲，形成居中摆头。
// 此处启用仿真增益；实物烧录时切回上方实物参数即可。
static float motor_kp = 0.1, motor_ki = 0.1, motor_kd = 0;

/**
  * @简  述 电机PID控制函数
  * @参  数 left:  左电机速度目标值 
  *      	 right: 右电机速度目标值 
  * @返回值 无
  */


// AGV 小车的 速度 变量 
double Wheel_left_speed = 0, Wheel_right_speed = 0;
double agv_x = 0, agv_w = 0;


void MOTOR_SetSpeed(int16_t left, int16_t right)
{
    // printf("MOTOR_SetSpeed : left = %d,right = %d\n",left,right);

    //1. 读取编码器计数值
    int16_t encoder_left = 0;
    int16_t encoder_right = 0;
    encoder_left = ENCODER_Left_GetCounter();
    encoder_right = -ENCODER_Right_GetCounter();
   
    ENCODER_Left_SetCounter(0);
    ENCODER_Right_SetCounter(0);

    if(encoder_left>30000)
    {
        encoder_left = 65535 - encoder_left;
    }

    if(encoder_left>30000)
    {
        encoder_left = 65535 - encoder_left;
    }

    // 2. 将编码器计数转换为轮子速度和AGV速度,如果有需要的话，可以在这里求取AGV的线速度和角速度，供其他模块使用
    Wheel_left_speed = encoder_left*FWD_WHEEL_SCALE;   // 左轮速度，单位为m/s
    Wheel_right_speed = encoder_right*FWD_WHEEL_SCALE; // 右轮速度，单位为m/s
    agv_x = (Wheel_left_speed + Wheel_right_speed) / 2.0; // AGV线速度，单位为m/s
    agv_w = (Wheel_right_speed - Wheel_left_speed) / (FWD_WHEEL_BASE*FWD_WB_SCALE); // AGV角速度，单位为rad/s

    /* 【直行对称性保证】当左右轮目标速度相等（居中直行、丢线直行等场景）时，
     * 直接把目标PWM等幅输出到两轮，**绕过两轮独立PID**。
     *
     * 原因：仿真环境电机虽为理想模型，但编码器反馈存在量化噪声与一拍相位滞后，
     * 左右轮各自的增量式PID会对"相同目标+不同瞬时反馈"算出不同PWM，
     * 导致车体在命令纯直行时仍左右轮流过冲——即"中间两灯亮着车还在摆"。
     * 等幅输出是几何对称的充分条件，从根上消除差速源。
     * 仍保留编码器读取/清零与速度计算，供遥测与下次闭环使用。 */
    if (left == right)
    {
        MOTOR_Left_SetPWM(left);
        MOTOR_Right_SetPWM(right);
        return;
    }

    // 3. 将编码器速度转换为PWM控制值，并调用电机控制函数
    if(encoder_left > FWD_MAX_ENCODER_CNT) encoder_left = FWD_MAX_ENCODER_CNT;   // 处理编码器计数器溢出
    if(encoder_left < -FWD_MAX_ENCODER_CNT) encoder_left = -FWD_MAX_ENCODER_CNT; // 处理编码器计数器溢出
    if(encoder_right > FWD_MAX_ENCODER_CNT) encoder_right = FWD_MAX_ENCODER_CNT;   // 处理编码器计数器溢出
    if(encoder_right < -FWD_MAX_ENCODER_CNT) encoder_right = -FWD_MAX_ENCODER_CNT; // 处理编码器计数器溢出

    //4. 将编码器速度转换为实时的PWM值，范围-4200~4200
    int16_t rt_left = encoder_left * 4200 /FWD_MAX_ENCODER_CNT ; // 将编码器速度转换为PWM实时值，范围-4200~4200
    int16_t rt_right = encoder_right * 4200 /FWD_MAX_ENCODER_CNT ; // 将编码器速度转换为PWM实时值，范围-4200~4200

    // printf("MOTOR_SetSpeed : rt_left = %d,rt_right = %d\n",rt_left,rt_right);

    int16_t letf_speed = MOTOR_Left_SetSpeed(left, rt_left);
    //5. 调用电机控制函数，进行PID控制 第一个参数是目标速度（用pwm表示），第二个参数是当前速度
	if( 0 == left )
		MOTOR_Left_SetPWM(0);
	else if(Is_Same_Direction(left,rt_left) == 0) //不是同向
    {
        MOTOR_Left_SetPWM(left);
    }else
    {
        MOTOR_Left_SetPWM(letf_speed);
    }
	
     int16_t right_speed = MOTOR_Right_SetSpeed(right, rt_right);

	if( 0 == right )
		MOTOR_Right_SetPWM(0);
    else if(Is_Same_Direction(right,rt_right) == 0) //不是同向
    {
        MOTOR_Right_SetPWM(right);
    }else
    {
        MOTOR_Right_SetPWM(right_speed);
    }


   // printf("letf_speed = %d , right_speed = %d\n", letf_speed,right_speed);
    // printf("MOTOR_Right_SetSpeed(right, rt_right) = %d\n", MOTOR_Right_SetSpeed(right, rt_right));
}



/**
  * @brief  判断目标值和当前值是否同向
  * @param  target:  目标值
  *         current: 当前值
  * @retval 1: 同向（都为正或都为负）
  *         0: 反向（一正一负）或至少有一个为0
  */
uint8_t Is_Same_Direction(int16_t target, int16_t current)
{
    return (target > 0 && current > 0) || (target < 0 && current < 0);
}





/**
  * @简  述 左电机PID控制函数
  * @参  数 target:  PWM速度目标值 
  *          current: PWM速度当前值
  * @返回值  电机PWM速度
  */
int16_t MOTOR_Left_SetSpeed(int16_t target, int16_t current)
{

    // 定义静态变量保存历史误差值
    static int16_t last_error = 0;      // 上一次的误差
    static int16_t last_error2 = 0;     // 上上次的误差
    static int16_t last_output = 0;   // 上一次的输出值

    int16_t motor_pwm_out = 0;
    
    // 计算当前误差
    int16_t error = target - current;
    // printf("error = %d\n",error);
    // 增量式PID计算公式：
    // Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k) + Kd * [e(k) - 2e(k-1) + e(k-2)]
    
    // 计算增量值
    int16_t delta_output = motor_kp * (error - last_error) +
                         motor_ki * error +
                         motor_kd * (error - 2 * last_error + last_error2);
    
    // 计算当前输出值 = 上一次输出 + 增量
    motor_pwm_out = last_output + delta_output;
    
    // 限制最大输出
    if(motor_pwm_out > 4200)
        motor_pwm_out = 4200;
    if(motor_pwm_out < -4200)
        motor_pwm_out = -4200;
    
    // 更新历史值
    last_error2 = last_error;     // 更新上上次误差
    last_error = error;           // 更新上次误差
    last_output = motor_pwm_out;  // 更新上次输出值
    

    if(Is_Same_Direction(target,current) == 0) //不是同向
    {
        last_error = 0;
        last_error2 = 0;
        last_output = target;
        motor_pwm_out = target;
        return motor_pwm_out;
    }

    // 返回PWM控制值
    return motor_pwm_out;
}

/**
  * @简  述 右电机PID控制函数
  * @参  数 target:  PWM速度目标值 
  *      	 current: PWM速度当前值
  * @返回值  电机PWM速度
  */
int16_t MOTOR_Right_SetSpeed(int16_t target, int16_t current)
{

    // 定义静态变量保存历史误差值
    static int16_t last_error = 0;      // 上一次的误差
    static int16_t last_error2 = 0;     // 上上次的误差
    static int16_t last_output = 0;   // 上一次的输出值

    int16_t motor_pwm_out = 0;
    
    // 计算当前误差
    int16_t error = target - current;
    // printf("error = %d\n",error);
    // 增量式PID计算公式：
    // Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k) + Kd * [e(k) - 2e(k-1) + e(k-2)]
    
    // 计算增量值
    int16_t delta_output = motor_kp * (error - last_error) +
                         motor_ki * error +
                         motor_kd * (error - 2 * last_error + last_error2);
    
    // 计算当前输出值 = 上一次输出 + 增量
    motor_pwm_out = last_output + delta_output;
    
    // 限制最大输出
    if(motor_pwm_out > 4200)
        motor_pwm_out = 4200;
    if(motor_pwm_out < -4200)
        motor_pwm_out = -4200;
    
    // 更新历史值
    last_error2 = last_error;     // 更新上上次误差
    last_error = error;           // 更新上次误差
    last_output = motor_pwm_out;  // 更新上次输出值
    

    if(Is_Same_Direction(target,current) == 0) //不是同向
    {
        last_error = 0;
        last_error2 = 0;
        last_output = target;
        motor_pwm_out = target;
        return motor_pwm_out;
    }

    // 返回PWM控制值
    return motor_pwm_out;
}

/**
  * @简  述 PID控制器复位（启动/急停/模式切换时调用）
  * @参  数 无
  * @返回值 无
  */
void MOTOR_PID_Reset(void)
{
    MOTOR_Left_SetPWM(0);
    MOTOR_Right_SetPWM(0);
}
