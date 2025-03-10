#include"LED.h"

/*关于GPIO的控制 :首先使能GPIO, 需要用RCC寄存器控制打开外设时钟总线APB*/
/*然后创建GPIO结构体并赋予初值*/

void __GPIO_pin_init(GPIO_TypeDef* GPIOx, uint16_t pin, GPIOSpeed_TypeDef speed, GPIOMode_TypeDef mode)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Speed = speed;
    GPIO_InitStructure.GPIO_Mode = mode;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void __GPIO_pin_set(GPIO_TypeDef* GPIOx, uint16_t pin)
{
    GPIO_SetBits(GPIOx, pin);
}

void __GPIO_pin_reset(GPIO_TypeDef* GPIOx, uint16_t pin)
{
    GPIO_ResetBits(GPIOx, pin);
}

uint8_t __GPIO_pin_read(GPIO_TypeDef* GPIOx, uint16_t pin)
{
    return GPIO_ReadInputDataBit(GPIOx, pin);
}