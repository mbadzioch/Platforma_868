#ifndef DELAY_H
#define DELAY_H

#include <stdlib.h>
#include "stm32f0xx_conf.h"

void TimingDelay_Decrement(void);
void Delay_us(uint32_t nTime);
void Delay_ms(uint16_t);
void DelayInit(void);

static __IO uint32_t TimingDelay;

#endif
