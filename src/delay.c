#include "delay.h"



void DelayInit(void){

	if (SysTick_Config(SystemCoreClock / 1000)){
		while (1);
	}
}

void TimingDelay_Decrement(void){
	if (TimingDelay != 0){
		TimingDelay--;
	}
}

void Delay_ms(uint16_t val){
	TimingDelay = val;

	while(TimingDelay != 0);
}

void SysTick_Handler(void){
  TimingDelay_Decrement();
}
