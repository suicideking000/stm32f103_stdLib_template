#ifndef  __COUNTERSENSOR_H
#define  __COUNTERSENSOR_H
#include "stm32f10x.h"
#include "Delay.h"

void CounterSensor_Init(void);
uint16_t CounterSensor_GetValue(void);

#endif
