#include "servo.h" 
#include <string.h>

/**
  * @brief  舵机S1接口初始化
  * @param  无
  * @retval 无
  */
void SERVO_S1_Init(void)
{ 
    GPIO_InitTypeDef GPIO_InitStructure; 
    
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;   

    /* 使能GPIOB时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    /* 使能定时器10时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

    /* 配置GPIO复用 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);

    /* 配置GPIO为复用推挽输出 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 定时器时基配置
     * 系统时钟168MHz，预分频168得到1MHz
     * 周期20000对应20ms (50Hz PWM)
     */
    TIM_TimeBaseStructure.TIM_Prescaler = 168 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);
    
    /* PWM模式配置：通道1 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;      /* 初始占空比 */
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   
    
    TIM_OC1Init(TIM10, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);
    
    /* 使能自动重装载 */
    TIM_ARRPreloadConfig(TIM10, ENABLE);

    /* 启动定时器 */
    TIM_Cmd(TIM10, ENABLE);
}

/**
  * @brief  设置舵机S1角度
  * @param  angle 角度值，范围-900~900（单位为0.1度）
  *         例如：900表示90度，-900表示-90度
  * @retval 无
  */
void SERVO_S1_SetAngle(int16_t angle)
{
    if(angle > 900) angle = 900;
    if(angle < -900) angle = -900;
    
    /* 将角度转换为PWM占空比
     * 1.111f * angle + 1500
     * 1500对应0度（1.5ms脉冲宽度）
     */
    TIM_SetCompare1(TIM10, (1.111f * angle + 1500));
}

/**
  * @brief  释放舵机S1（停止输出PWM）
  * @param  无
  * @retval 无
  */
void SERVO_S1_Release(void)
{
    TIM_SetCompare1(TIM10, 0);
}

/**
  * @brief  舵机S2接口初始化
  * @param  无
  * @retval 无
  */
void SERVO_S2_Init(void)
{ 
    GPIO_InitTypeDef GPIO_InitStructure; 
    
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;   

    /* 使能GPIOB时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    /* 使能定时器11时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);

    /* 配置GPIO复用 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11);

    /* 配置GPIO为复用推挽输出 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 定时器时基配置 */
    TIM_TimeBaseStructure.TIM_Prescaler = 168 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);
    
    /* PWM模式配置：通道1 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   
    
    TIM_OC1Init(TIM11, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);
    
    /* 使能自动重装载 */
    TIM_ARRPreloadConfig(TIM11, ENABLE);

    /* 启动定时器 */
    TIM_Cmd(TIM11, ENABLE);
}

/**
  * @brief  设置舵机S2角度
  * @param  angle 角度值，范围-900~900（单位为0.1度）
  * @retval 无
  */
void SERVO_S2_SetAngle(int16_t angle)
{
    if(angle > 900) angle = 900;
    if(angle < -900) angle = -900;
    
    TIM_SetCompare1(TIM11, (1.111f * angle + 1500));
}

/**
  * @brief  释放舵机S2（停止输出PWM）
  * @param  无
  * @retval 无
  */
void SERVO_S2_Release(void)
{
    TIM_SetCompare1(TIM11, 0);
}

/**
  * @brief  舵机S3/S4/S5/S6接口初始化（使用TIM8）
  * @param  无
  * @retval 无
  */
void SERVO_S3456_Init(void)
{ 
    GPIO_InitTypeDef GPIO_InitStructure; 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;   
    
    /* 1. 使能时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    /* 2. 配置GPIO复用功能 - PC6, PC7, PC8, PC9 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    /* 3. 配置TIM8时基 - 50Hz PWM (20ms周期)
     * 系统时钟168MHz
     * 预分频168得到1MHz
     * 周期20000对应20ms
     * 中间值1500对应1.5ms (0度位置)
     */
    TIM_TimeBaseStructure.TIM_Prescaler = 168 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    /* 4. 配置PWM模式 - 4个通道 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 1500;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    
    /* 初始化并启用所有4个通道的预装载 */
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
    
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
    
    /* 5. 使能ARR预装载 */
    TIM_ARRPreloadConfig(TIM8, ENABLE);

    /* 6. 使能定时器 */
    TIM_Cmd(TIM8, ENABLE);
    
    /* 7. 【关键】使能高级定时器主输出MOE */
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
}

/**
  * @brief  设置舵机S3角度
  * @param  angle 角度值，范围-900~900（单位为0.1度）
  * @retval 无
  */
void SERVO_S3_SetAngle(int16_t angle)
{
    if(angle > 900) angle = 900;
    if(angle < -900) angle = -900;
    
    TIM_SetCompare1(TIM8, (1.111f * angle + 1500));
}

/**
  * @brief  释放舵机S3（停止输出PWM）
  * @param  无
  * @retval 无
  */
void SERVO_S3_Release(void)
{
    TIM_SetCompare1(TIM8, 0);
}

/**
  * @brief  设置舵机S4角度
  * @param  angle 角度值，范围-900~900（单位为0.1度）
  * @retval 无
  */
void SERVO_S4_SetAngle(int16_t angle)
{
    if(angle > 900) angle = 900;
    if(angle < -900) angle = -900;
    
    TIM_SetCompare2(TIM8, (1.111f * angle + 1500));
}

/**
  * @brief  释放舵机S4（停止输出PWM）
  * @param  无
  * @retval 无
  */
void SERVO_S4_Release(void)
{
    TIM_SetCompare2(TIM8, 0);
}

/**
  * @brief  设置舵机S5角度
  * @param  angle 角度值，范围-900~900（单位为0.1度）
  * @retval 无
  */
void SERVO_S5_SetAngle(int16_t angle)
{
    if(angle > 900) angle = 900;
    if(angle < -900) angle = -900;
    
    TIM_SetCompare3(TIM8, (1.111f * angle + 1500));
}

/**
  * @brief  释放舵机S5（停止输出PWM）
  * @param  无
  * @retval 无
  */
void SERVO_S5_Release(void)
{
    TIM_SetCompare3(TIM8, 0);
}

/**
  * @brief  设置舵机S6角度
  * @param  angle 角度值，范围-900~900（单位为0.1度）
  * @retval 无
  */
void SERVO_S6_SetAngle(int16_t angle)
{
    if(angle > 900) angle = 900;
    if(angle < -900) angle = -900;
    
    TIM_SetCompare4(TIM8, (1.111f * angle + 1500));
}

/**
  * @brief  释放舵机S6（停止输出PWM）
  * @param  无
  * @retval 无
  */
void SERVO_S6_Release(void)
{
    TIM_SetCompare4(TIM8, 0);
}



// 当前关节角度（0.1度单位）
int16_t current_angles[6] = {0, 0, 0, 0, 0, GRIPPER_OPEN};

/* 设置机械臂舵机 S1-S5（不包含机械爪 S6）
 * 用于单独控制机械臂运动，机械爪保持当前状态
 */
void set_arm_servos(const int16_t angles[5])
{
    int16_t applied[5];
    for (int i = 0; i < 5; i++) {
        applied[i] = angles[i] > 900 ? 900 : (angles[i] < -900 ? -900 : angles[i]);
    }
    SERVO_S1_SetAngle(applied[0]);
    SERVO_S2_SetAngle(applied[1]);
    SERVO_S3_SetAngle(applied[2]);
    SERVO_S4_SetAngle(applied[3]);
    SERVO_S5_SetAngle(applied[4]);
    /* 记录真正下发的有限位角度，供后续逆解选择使用。 */
    memcpy(current_angles, applied, sizeof(applied));
    SMART_DELAY_MS(50);  // 等待机械臂舵机到位
}

/* 设置机械爪舵机 S6（J6）
 * 单独控制机械爪的张合，应在机械臂到位后调用
 * angle: -900=张开, 0-900=闭合
 */
void set_gripper_servo(int16_t angle)
{
    if(angle > 900) angle = 900;
    if(angle < -900) angle = -900;
    SERVO_S6_SetAngle(angle);
    current_angles[5] = angle;
    SMART_DELAY_MS(20);  // 机械爪动作较快，等待时间短一些
}


/*
 * 平滑运动控制。
 * 在当前角度与目标角度之间进行 steps 步线性插值，依次调用 set_arm_servos。
 * 最后设置机械爪角度。
 */
void move_smooth(const int16_t target[6], int steps)
{
    int16_t start[5];
    int16_t gripper_target = target[5];

    if (steps <= 0) {
        steps = 1;
    }

    for (int i = 0; i < 5; i++) {
        start[i] = current_angles[i];
    }

    for (int s = 1; s <= steps; s++) {
        int16_t tmp[5];

        for (int i = 0; i < 5; i++) {
            int32_t delta = (int32_t)target[i] - (int32_t)start[i];
            tmp[i] = (int16_t)(start[i] + (delta * s) / steps);
        }

        set_arm_servos(tmp);
    }
    SMART_DELAY_MS(1000);  // 等待机械臂舵机到位
    set_gripper_servo(gripper_target);
}
