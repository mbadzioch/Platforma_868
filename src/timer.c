/*
 * timer.c
 *
 *  Created on: 17 paź 2016
 *      Author: Marcin
 */

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx_conf.h"
#include "timer.h"
#include "debugkom.h"


typedef enum{
	RUNNING = 0x01,
	STOPPED
}timerState_E;

typedef struct{
	uint8_t id;
	uint16_t period;
	uint16_t cntReg;
	timerOpt_E option;
	timerState_E state;
	void (*funkcja)(void);
}timer_S;

volatile timer_S timerTab[MAX_TIMERS_NUM];
volatile uint8_t timersNum=0;

/*
 * Rejestracja nowego timera z podaniem parametrów i wskaźnika na funkcję do wywołania
 *
 *	id - pole w które zostanie wpisany numer timera
 *	period - okres w ms
 *	option - opcja timera
 *
 *  Zwraca:
 *	 1 - sukces!
 *  -1 w przypadku braku miejsca
 *  -2 w przypadku zbyt długiego okresu
 *  -3 w przypadku konfliktu opcji
 *
 */
int8_t Timer_Register(uint8_t* id,uint16_t period, timerOpt_E option)
{
	if(timersNum == MAX_TIMERS_NUM)return -1;
	if(period > MAX_PERIOD) return -2;
	if((option&timerOpt_AUTORESET) && (option&timerOpt_AUTOSTOP))return -3;

	timerTab[timersNum].id=timersNum;
	timerTab[timersNum].period=period;
	timerTab[timersNum].cntReg=period;
	timerTab[timersNum].option=option;
	timerTab[timersNum].state=RUNNING;
	*id = timersNum;
	timersNum++;
	return 1;
}

void Timer_Reset(uint8_t* id)
{
	timerTab[*id].cntReg = timerTab[*id].period;
	timerTab[*id].state=RUNNING;
}
void Timer_Stop(uint8_t* id)
{
	timerTab[*id].cntReg = 0;
	timerTab[*id].state=STOPPED;
}
void Timer_Run(uint8_t* id)
{
	timerTab[*id].cntReg = timerTab[*id].period;
	timerTab[*id].state=RUNNING;
}
/*
 * Zwraca:
 * 	1 - jeśli minął zadany czas
 * 	0 - jeśli nie minął zadany czas
 * 	-1 - jeśli nie ma zarejestrowanego timera o podanym ID
 */
int8_t Timer_Check(uint8_t* id)
{
	if((*id > timersNum) || (timersNum==0)) return -1;
	if(timerTab[*id].cntReg==0){
		if(timerTab[*id].option&timerOpt_AUTORESET){
			if(timerTab[*id].state==STOPPED){
				return 0;
			}
			else{
				Timer_Reset(id);
				return 1;
			}
		}
		else if((timerTab[*id].option&timerOpt_AUTOSTOP) && (timerTab[*id].state==RUNNING)){
			timerTab[*id].state=STOPPED;
			return 1;
		}
		else return 0;
	}
	return 0;
}

void Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/1000000;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}
void Timer_Handler()
{
	for(uint8_t i=0;i<timersNum;i++){
		if(timerTab[i].state==RUNNING){
			if(timerTab[i].cntReg > 0)timerTab[i].cntReg--;
		}
	}
}
void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET){
		Timer_Handler();
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}

