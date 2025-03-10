#ifndef  __LED_H
#define  __LED_H
#include "stm32f10x.h"
#include "Delay.h"

void LED_Init(void);
void LED_on(uint16_t pin);
void LED_off(uint16_t pin);
void LED_turn(uint16_t pin);

#endif
