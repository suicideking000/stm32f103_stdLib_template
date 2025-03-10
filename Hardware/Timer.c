#include"Timer.h"


uint16_t counter_value1=0;
void Timer_Init(void)
{
    // Initialize timer here
    //RCC开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    //配置时钟
    TIM_InternalClockConfig(TIM2);
    //初始化定时器
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 10000-1;//ARR自动重装器
    TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;//PSC预分频器
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复计数器
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    //定时器中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    //NVIC中断
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //启动定时器
    TIM_Cmd(TIM2, ENABLE);
}

void External_Timer_Init(void)
{
     // Initialize timer here
    //RCC开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //配置时钟
    TIM_ETRClockMode1Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    //配置外部GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //初始化定时器
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 10-1;//ARR自动重装器
    TIM_TimeBaseStructure.TIM_Prescaler = 1-1;//PSC预分频器
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复计数器
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);//**TIM_IT_Update**: 定时器更新中断源。当定时器计数器溢出或更新事件发生时，会触发此中断。

    
    //定时器中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    //NVIC中断
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //启动定时器
    TIM_Cmd(TIM2, ENABLE);

}

void Timer_PWM_Init(void)
{
    // Initialize timer here
    //RCC开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    //配置时钟
    TIM_InternalClockConfig(TIM2);
    //初始化定时器
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 100-1;//ARR自动重装器
    TIM_TimeBaseStructure.TIM_Prescaler = 720-1;//PSC预分频器
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复计数器
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    //初始化输出比较定时器
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1;//CCR   占空比=CCR/ARR+1  频率=72M/(ARR+1)/(PSC+1)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_Cmd(TIM2, ENABLE);
    //初始化CH1输出引脚PA0
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//使用片上外设TIMER复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Timer_PWM_SetPulse(uint16_t pulse)//通过修改CCR修改占空比
{
    TIM_SetCompare1(TIM2, pulse);
}

void Timer_PWM_SetFrequency(uint16_t frequency)//通过修改PSC修改频率
{
    TIM_PrescalerConfig(TIM2, 72000000/(100*frequency)-1, TIM_PSCReloadMode_Update);
}

void Timer_PWM_IC_Init(void)//PWM输入捕获
{
    // Initialize timer here
    //RCC开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    //配置时钟
    TIM_InternalClockConfig(TIM3);
    //初始化定时器
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 65536-1;//ARR自动重装器
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;//PSC预分频器
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复计数器
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    //初始化输入捕获定时器通道1直连输入
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    //初始化输入捕获定时器通道2交叉输入
    TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
    //初始化CH1输入引脚PA0
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//输入上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //配置TRGI触发源为TI1FP1
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
    //配置从模式为reset
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);

    TIM_Cmd(TIM3, ENABLE);

}

uint32_t Timer_PWM_IC_GetFreq(void)
{
    return 1000000/(TIM_GetCapture1(TIM3)+1);
}

uint32_t Timer_PWM_IC_GetDuty(void)
{
    return (TIM_GetCapture2(TIM3)+1)*100/(TIM_GetCapture1(TIM3)+1);
}

uint16_t Timer_GetValue(void)
{
    return counter_value1;
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        counter_value1++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        //处理中断
        //LED_turn(GPIO_Pin_0);
        //counter_value++;
    }
}

