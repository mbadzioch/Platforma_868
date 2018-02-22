#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx_conf.h"
#include "delay.h"
#include "timer.h"
#include "debugkom.h"
#include "radio_driver.h"

GPIO_InitTypeDef GPIO_InitStructure;

uint8_t cosik;

char buf[64];


int main()
{
	DelayInit();
	Timer_Init();
	Debug_Init();
	RD_Init();

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
		Delay_ms(1000);
		cosik=RD_RadioSendReadTransaction(0x00);
		sprintf(buf,"Wartosc: %d\n\r",cosik);
		PC_Debug(buf);
	  //Debug_Main();
    }
}

