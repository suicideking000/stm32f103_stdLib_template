#include"Timer.h"

/*=============================================TIMER==================================================*/
/*stm32f103 有 tim1~4,时钟的开启和GPIO类似,也是定义一个时钟结构体,通用定时器使用的是APB1低速外设总线,但是也会获得36*2Mhz的时钟频率*/
/*假设使用TIM2时钟并开启中断*/
void __TIMER_Init(void)
{
    //RCC使能时钟总线
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    //使用内部时钟源
    TIM_InternalClockConfig(TIM2);
    //初始化定时器
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler=7200-1;//PSC预分频器
    TIM_TimeBaseStructure.TIM_Period=10000-1;//ARR自动重装器
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复计数器,多少次重装后更新事件,高级计时器
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//时钟分频 输入时钟为72Mhz
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式：从0计数到自动加载值(TIMx_ARR)，然后重新从0开始计数并产生溢出事件。
                                                             //向下计数模式：从自动装入的值(TIMx_ARR)开始向下计数到0，然后重新开始并产生溢出事件。
                                                             //中央对齐模式（向上/向下计数）：从0开始计数到自动装入的值-1，产生溢出事件，然后向下计数到1并再次产生溢出事件，接着从0重新开始。

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除中断标志位
    //定时器中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//使能了TIM2的更新中断，使得当TIM2计数器溢出或更新事件发生时，会触发中断。    
    //NVIC中断
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //启动定时器
    TIM_Cmd(TIM2, ENABLE);
}

void __ETR_TMIER_Init(void)//使用外部触发时钟,比如GPIO的上升沿
{
    //RCC使能时钟总线
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //配置外部时钟
    TIM_ETRClockMode1Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);//外部时钟源模式1:ETR外部时钟,其他定时器.GPIO捕获通道
                                                                                        //外部时钟源模式2:ETR外部时钟
    //配置时钟
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler=1;//PSC预分频器
    TIM_TimeBaseStructure.TIM_Period=10;//ARR自动重装器
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复计数器
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//时钟分频 输入时钟为外部GPIO
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除中断标志位
    //配置外部GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //定时器中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    //NVIC中断
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //启动定时器
    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        //处理中断
    }
}
/* =============================================PWM===============================================*/
/*输出PWM同样需要开启时钟,并且需要配置GPIO引脚,配置定时器,配置输出比较模式,配置输出比较通道*/
/*假设使用TIM2输出四路PWM*/
void __PWM_Init(void)
{
    //RCC使能时钟总线
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //配置时钟
    TIM_InternalClockConfig(TIM2);
    //初始化定时器
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    //输出pwm的频率为72Mhz/PSC/ARR
    TIM_TimeBaseStructure.TIM_Prescaler=7200-1;//PSC预分频器
    TIM_TimeBaseStructure.TIM_Period=100-1;//ARR自动重装器
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复计数器
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//时钟分频 输入时钟为72Mhz
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除中断标志位
    //初始化输出比较模式
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    //初始化输出比较通道1
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//PWM模式1
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//输出使能
    TIM_OCInitStructure.TIM_Pulse=50;//设置CCR寄存器,占空比=CCR/ARR+1
    TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//输出极性 有效电平为高
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);//使能CCR寄存器
    //初始化输出比较通道2
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//PWM模式1
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);//使能CCR寄存器
    //初始化输出比较通道3
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//PWM模式1
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);//使能CCR寄存器
    //初始化输出比较通道4
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//PWM模式1
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);//使能CCR寄存器
    //初始化输出引脚
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //启动定时器
    TIM_Cmd(TIM2, ENABLE);
}

/* =============================================IC===============================================*/
/*输入捕获（input capture），需要开启时钟,配置GPIO引脚,配置定时器,配置输入捕获模式,配置输入捕获通道*/
/*假设使用TIM3输入捕获*/
//测频法:在固定时间T内，计数器计数的次数为N，那么频率F= N/T
//测周法:在两个上升沿内,以标准频率fc计次,得到N,那么频率F= fc/N

//使用测周法,开始捕获时,计数器清零,当捕获到第一个上升沿时,计数器开始计数,当捕获到第二个上升沿时,计数器停止计数,计数器的值即为N
void __IC_Init(void)
{
    //使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //配置时钟
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler=72-1;//PSC预分频器 计数频率
    TIM_TimeBaseStructure.TIM_Period=65536-1;//ARR自动重装器 
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复计数器
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//时钟分频 输入时钟为72Mhz
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);//清除中断标志位
    //配置GPIO引脚 TIM3_CH1
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin= GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//输入上拉
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //初始化输入捕获模式
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;//通道1
    TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//上升沿捕获
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;//直连输入
    TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;//不分频
    TIM_ICInitStructure.TIM_ICFilter=0xF;//滤波器
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    //配置TRGI触发源为TI1FP1
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
    //配置从模式为reset
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);

    //启动定时器
    TIM_Cmd(TIM3, ENABLE); 
}

void __IC_PWMI_Init(void)
{
    //使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //配置时钟
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler=72-1;//PSC预分频器 计数频率
    TIM_TimeBaseStructure.TIM_Period=65536-1;//ARR自动重装器 
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复计数器
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//时钟分频 输入时钟为72Mhz
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);//清除中断标志位
    //配置GPIO引脚 TIM3_CH1
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin= GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//输入上拉
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //初始化输入捕获PWMI模式
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;//通道1
    TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//上升沿捕获
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;//直连输入
    TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;//不分频
    TIM_ICInitStructure.TIM_ICFilter=0xF;//滤波器
    TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);

    //配置TRGI触发源为TI1FP1
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
    //配置从模式为reset
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);

    //启动定时器
    TIM_Cmd(TIM3, ENABLE);
}

uint32_t __IC_GetFreq(void)//pwm频率
{
    return 1000000/(TIM_GetCapture1(TIM3)+1);
}

uint32_t __IC_GetDuty(void)//pwm占空比 高电平计数器在CCR2,整个周期计数器在CCR1
{
    return (TIM_GetCapture2(TIM3)+1)*100/(TIM_GetCapture1(TIM3)+1);
}

/*=============================================编码器==================================================*/
//编码器用于测量转速,需要开启时钟,配置GPIO引脚,配置定时器,配置编码器模式,配置编码器通道
//编码器捕获正交信号,需要两个输入接口 通道1,通道2
//假设使用TIM4编码器模式
void __TIMER_Encoder_Init()
{
    //使能RCC时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    //TIM_InternalClockConfig(TIM2); 编码器会接管时钟
    //配置时钟
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler=1-1;//PSC预分频器 计数频率 编码器时钟直接驱动计数器
    TIM_TimeBaseStructure.TIM_Period=65536-1;//ARR自动重装器
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复计数器
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//时钟分频 输入时钟为72Mhz
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除中断标志位
    //输入捕获单元
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;//通道1
    // TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//上升沿捕获
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;//直连输入
    // TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;//不分频
    TIM_ICInitStructure.TIM_ICFilter=0xF;//滤波器
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;//通道2
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    //配置编码器模式
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    //配置GPIO引脚 TIM4_CH1 TIM4_CH2
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin= GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//输入上拉
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    //启动定时器
    TIM_Cmd(TIM4, ENABLE);
}

uint16_t __TIMER_Encoder_GetValue(void)
{
    return TIM_GetCounter(TIM4);
}