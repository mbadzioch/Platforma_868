#include "delay.h"



void DelayInit(void){

	if (SysTick_Config(SystemCoreClock / 100000)){
		while (1);
	}
}

void TimingDelay_Decrement(void){
	if (TimingDelay != 0){
		TimingDelay--;
	}
}

void Delay_us(uint32_t nTime){
	TimingDelay = nTime/10;

	while(TimingDelay != 0);
}

void Delay_ms(uint16_t val){
	Delay_us(1000UL*(uint32_t)val);
}

void SysTick_Handler(void){
  TimingDelay_Decrement();
}
