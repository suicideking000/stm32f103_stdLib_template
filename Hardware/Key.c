#include "key.h"
#include "LED.h"

/*关于外部中断的开启, 第一步和GPIO一样 先开启APB外设总线并使能GPIO为输入模式 ,之后配置EXTI外部中断, 最后开启NVIC终端控制*/
//STM32 中的外部中断（EXTI）通过外部引脚（如 GPIO 引脚）触发，这些引脚的状态变化（上升沿、下降沿或电平）会激活外部中断。为了将 GPIO 引脚与 EXTI 系统连接起来，必须通过复用功能将引脚映射到 EXTI 线路。
//这是因为 STM32 的 GPIO 引脚并不直接连接到 EXTI 控制器，而是通过 复用功能来激活。

void Key_Init()//假设使用GPIOB的pin4
{
    /*设置NVIC分组,一般在main中设置*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /*使能复用外设*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    /*初始化GPIO*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*AFIO配置*/
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
    /*EXTI配置*/
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_StructInit(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    /*NVIC配置*/
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*中断回调函数,从Startup文件获取*/
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line5);
        //处理中断
        //LED_turn(GPIO_Pin_0);
        //counter_value++;
    }
}
