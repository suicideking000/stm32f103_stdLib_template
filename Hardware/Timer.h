#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f10x.h"

void Timer_Init(void);
void External_Timer_Init(void);
void Timer_PWM_Init(void);
void Timer_PWM_SetPulse(uint16_t pulse);
void Timer_PWM_SetFrequency(uint16_t frequency);
void Timer_PWM_IC_Init(void);
uint32_t Timer_PWM_IC_GetFreq(void);
uint32_t Timer_PWM_IC_GetDuty(void);


uint16_t Timer_GetValue(void);
void TIM2_IRQHandler(void);

#endif // __TIMER_H
