#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx_conf.h"
#include "delay.h"
#include "timer.h"
#include "debugkom.h"

GPIO_InitTypeDef GPIO_InitStructure;
int main()
{
	DelayInit();
	Timer_Init();
	Debug_Init();

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


  while (1)
    {
	  GPIO_SetBits(GPIOB,GPIO_Pin_9);
	  Delay_ms(100);
	  GPIO_ResetBits(GPIOB,GPIO_Pin_9);
	  Delay_ms(100);
	  Debug_Main();
    }
}

