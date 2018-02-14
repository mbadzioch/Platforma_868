/*
 * timer.h
 *
 *  Created on: 17 paź 2016
 *      Author: Marcin
 */

#ifndef TIMER_H_
#define TIMER_H_

#define MAX_TIMERS_NUM 8
#define MAX_PERIOD 65000

typedef enum{
	timerOpt_AUTORESET = 0x01,
	timerOpt_AUTOSTOP
}timerOpt_E;

void Timer_Init(void);
int8_t Timer_Check(uint8_t* id);
void Timer_Reset(uint8_t* id);
void Timer_Stop(uint8_t* id);
void Timer_Run(uint8_t* id);
int8_t Timer_Register(uint8_t* id,uint16_t period, timerOpt_E option);

void TIM1_UP_TIM16_IRQHandler();

#endif /* TIMER_H_ */
