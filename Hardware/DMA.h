#ifndef __DMA_H
#define __DMA_H

#include "stm32f10x.h"

void MYDMA_Init(uint8_t *SRC_value,uint8_t *DST_value,uint32_t bufsize);

#endif
