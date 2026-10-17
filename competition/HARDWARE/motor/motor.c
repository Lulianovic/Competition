#include "motor.h" 



/**
  * @简  述  电机PWM控制初始化，PWM频率为20KHZ	
  * @参  数  无
  * @返回值  无
  */
void MOTOR_Right_Init(void)
{ 
	
    GPIO_InitTypeDef GPIO_InitStructure; 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure; 
	
	//时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);	

	//复用功能配置
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9);
	
	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	//定时器配置
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_TimeBaseStructure.TIM_Period=4200-1;   //自动重装载值	
	TIM_TimeBaseStructure.TIM_Prescaler=1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

    //PWM1 Mode configuration: Channel1
	TIM_OCStructInit(&TIM_OCInitStructure); 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;	    //占空比初始化
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

    //PWM1 Mode configuration: Channel2
    TIM_OC2Init(TIM9, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM9, ENABLE);

    //使能定时器
    TIM_Cmd(TIM9, ENABLE);   
}


/**
  * @简  述  电机PWM控制初始化，PWM频率为20KHZ	
  * @参  数  无
  * @返回值  无
  */
void MOTOR_Left_Init(void)
{ 
	
GPIO_InitTypeDef GPIO_InitStructure; 

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure; 
	
	// 时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	

	// 复用功能配置
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); 
	
	// 配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	// 定时器配置
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period=4200-1;   //自动重装载值	
	TIM_TimeBaseStructure.TIM_Prescaler=1;     //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    //PWM1 Mode configuration: Channel1
	TIM_OCStructInit(&TIM_OCInitStructure); 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;	    //占空比初始化
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    //PWM1 Mode configuration: Channel2
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);


    TIM_ARRPreloadConfig(TIM1, ENABLE);

    //使能定时器
    TIM_Cmd(TIM1, ENABLE);   
	
	//使能MOE位
	TIM_CtrlPWMOutputs(TIM1,ENABLE);  
}

/**
  * @简  述  电机PWM控制初始化，PWM频率为20KHZ	
  * @参  数  无
  * @返回值  无
  */
void MOTOR_Init(void)
{ 	
    
	MOTOR_Left_Init();
	MOTOR_Right_Init();
}

/**
  * @简  述 电机PWM速度控制
  * @参  数 pwm 电机PWM控制量，范围-4200~4200
  * @返回值 无
  */

//经过实际测试，pwm的值 > 400时，电机才会转动
void MOTOR_Left_SetPWM(int16_t pwm)
{
	int16_t temp;
	
	temp = pwm;
	
	if(temp>4200)
		temp = 4200;
	if(temp<-4200)
		temp = -4200;
	
	if(temp > 0)
	{
		TIM_SetCompare1(TIM1, temp);
		TIM_SetCompare2(TIM1, 0);
	}
	else
	{
		TIM_SetCompare1(TIM1, 0);
		TIM_SetCompare2(TIM1, (-temp));
	}
}

/**
  * @简  述 电机PWM速度控制（新函数）
  * @参  数 speed 电机PWM控制量，范围-4200~4200
  * @返回值 无
  */
void MOTOR_Right_SetPWM(int16_t pwm)
{
	int16_t temp;
	
	temp = -pwm;
	
	if(temp>4200)
		temp = 4200;
	if(temp<-4200)
		temp = -4200;
	
	if(temp > 0)
	{
		TIM_SetCompare1(TIM9, 4200);
		TIM_SetCompare2(TIM9, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM9, 4200);
		TIM_SetCompare1(TIM9, (4200 + temp));
	}
}

