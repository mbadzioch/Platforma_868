/*
 * komunikacja.c
 *
 *  Created on: 12 wrz 2016
 *      Author: Marcin
 */



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f0xx_conf.h"
#include "debugkom.h"
#include "timer.h"

#define UART_FTDI 1
#define UART_DEBUG 1

char *err[]={" OK "," Blad pozycji "," Blad zmiany stanu "," Blad main "," Timeout "};
char *state[]={" Ruch FWD "," Ruch REV "," HAM "," THAM "," STDBY "};
char *regerr[]={" OK "," ERRS1" ," ERRS2 "," ERRS3 "," ERRI1" ," ERRI2 "," ERRI3 "," ERRMS ","ERRCS","ERRCI1","ERRCI2","ERRCI3","ERRCMS","ERRCSL","ERRCSH","ERRCLOWI","ERRCHIGHI","ERRCRPM"};
char *regstate[]={"REGOBC","CALCMAG","SETMAG","WAITMAG","WAITSPEED","IDLE","OFF"};


static enum {DALL,DPWM,DMEAS}msgFormat=DALL;

volatile uint8_t debugBuf[DEBUG_BUF_LENGTH],debugOVF=0,debugBufRDY=0;
volatile uint16_t debugBufCnt=0;
uint8_t debugTimer;

char cbuf[36];

static void Debug_InputHandler(void);
static void Debug_Raport(void);
static void Uart_Init();
static void Uart_BufClr();
/*
 * 	Inicjalizacja modułu
 */
void Debug_Init(void)
{
	Uart_Init();
	Uart_BufClr();
	if(Timer_Register(&debugTimer,500,timerOpt_AUTORESET) != 1)PC_Debug("Blad rejestracji timera!\n\r");
	PC_Debug("\n\rPlatforma_868\n\r");
}
void Debug_Main(void)
{
	Debug_InputHandler();
	if(Timer_Check(&debugTimer)==1){
		Debug_Raport();
	}
}

//--------------=== LOCAL ===-------------------------

static void Debug_Raport(void)
{
	switch(msgFormat){
	case DALL:
		//sprintf(cbuf,"Podniesienie: %d.%1d [deg]   Odchylenie: %d.%1d [deg]\n\rNastawiony kat: %d Odchylka: %d\n\rCurrent: %d MaxCurrent: %d\n\rRegulator: %d Stan: %d Error: %d\n\r",podnosnikMain.posAngle/10,abs(podnosnikMain.posAngle%10),podnosnikMain.devAngle/10,abs(podnosnikMain.devAngle%10),podnosnikMain.setAngle,podnosnikMain.diff,podnosnikMain.current,podnosnikMain.peakCurrent,podnosnikMain.state,podnosnikMain.stateSter,podnosnikMain.errorReg);
		sprintf(cbuf,"Czesc!");
		PC_Debug(cbuf);
		break;
	case DMEAS:
//		sprintf(cbuf,"VI: %d, VL: %d, CL: %d, VR: %d, TB: %d TI: %d\n\r",meas.voltageInput,meas.voltageLed,meas.currentLed,meas.voltageRef,meas.temperatureBrd,meas.temperatureInt);
//		PC_Debug(cbuf);
		break;
	case DPWM:
//		sprintf(cbuf,"VI: %d, VL: %d, CL: %d, TB: %d, PWM: %d CURs: %d\n\r",meas.voltageInput,meas.voltageLed,meas.currentLed,meas.temperatureBrd,driver.pwmAct,driver.setCurrent);
//		PC_Debug(cbuf);
		break;
	default:
		break;
	}

	PC_Debug("\r\n\r\n");
}
static void Debug_InputHandler(void)
{
	uint8_t p;
	uint16_t temp=0;

	if(debugBufRDY==1){
		debugBufRDY=0;

		for(uint8_t i=0;i<debugBufCnt;i++){
			if((debugBuf[i] <= 'z') && (debugBuf[i]>='a')){
				debugBuf[i] = debugBuf[i]-('a'-'A');
			}
		}

		switch(debugBuf[0]){
		case 'S':
			if(debugBuf[1] == 'P'){
				if(debugBuf[2]==' '){
					p = 3;
					while(p<50){
						if((debugBuf[p]=='\r') || (debugBuf[p]=='\n'))break;
						if(temp!=0)temp*=10;
						temp=temp+(debugBuf[p]-48);
						p++;
					}
					sprintf(cbuf,"PWM: %d\n\r",temp);
					PC_Debug(cbuf);
				}
			}
			if(debugBuf[1] == 'L'){
				if(debugBuf[2]==' '){
					p = 3;
					while(p<50){
						if((debugBuf[p]=='\r') || (debugBuf[p]=='\n'))break;
						if(temp!=0)temp*=10;
						temp=temp+(debugBuf[p]-48);
						p++;
					}
					sprintf(cbuf,"Temperatura dolna: %d\n\r");
					PC_Debug(cbuf);
				}
			}
			break;
		case 'P':

			break;
		case 'D':
			if(debugBuf[1] == 'A'){
				msgFormat=DALL;
			}
			else if(debugBuf[1] == 'P'){
				msgFormat=DPWM;
			}
			else if(debugBuf[1] == 'M'){
				msgFormat=DMEAS;
			}
			else if(debugBuf[1] == 'T'){
				switch(debugBuf[2]){
				case 'F':
					Timer_Register(&debugTimer,500,timerOpt_AUTORESET);
					PC_Debug("Raport FAST\n\r");
					break;
				case 'M':
					Timer_Register(&debugTimer,1000,timerOpt_AUTORESET);
					PC_Debug("Raport MED\n\r");
					break;
				case 'S':
					Timer_Register(&debugTimer,5000,timerOpt_AUTORESET);
					PC_Debug("Raport SLOW\n\r");
					break;
				case 'X':
					Timer_Register(&debugTimer,10000,timerOpt_AUTORESET);
					PC_Debug("Raport XSLOW\n\r");
					break;
				case 'O':
					Timer_Stop(&debugTimer);
					PC_Debug("Raport OFF\n\r");
					break;
				default:
					PC_Debug("Blad!\n\r");
					break;
				}
			}
			else{
				PC_Debug("Blad!\n\r");
			}
			break;
		case 'R':
			switch(debugBuf[1]){
			case 'R':
				sprintf(cbuf,"Regulator wlaczony!\n\r");
				PC_Debug(cbuf);
				break;
			case 'O':
				sprintf(cbuf,"Regulator wylaczony!\n\r");
				PC_Debug(cbuf);
				break;
			default:
				PC_Debug("Blad!\n\r");
				break;
			}
			break;
		case 'K':
			break;
		default:
			PC_Debug("Blad!\n\r");
			break;
		}
		Uart_BufClr();

	}
	else if(debugOVF==1){
		Uart_BufClr();
	}
}

/*
 * 	Nadawanie przez UART
 */
void PC_Debug(volatile char *s)
{
	while(*s){
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		    USART_SendData(USART2, *s++);
	}
}
/*
 * Odbiór
 */
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET){
		if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE)){
			if(debugBufCnt < DEBUG_BUF_LENGTH){
				debugBuf[debugBufCnt] = USART_ReceiveData(USART2);
				if(debugBuf[debugBufCnt] == '\r'){
					debugBufRDY=1;
					USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
				}
				else{
					debugBufCnt++;
				}
			}
			else{
				USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
				debugOVF=1;
			}
		}
	}
}
/*
 * Przygotowanie bufora
 */
static void Uart_BufClr()
{
	for(uint16_t i=0;i<DEBUG_BUF_LENGTH;i++){
		debugBuf[i]=0;
	}
	debugBufCnt=0;
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}
/*
 * Konfiguracja UART:
 *
 * 	USART1: Komunikacja debug
 *
 * 	PB: 6,7
 *
 * 	115200,8,1,0,0
 *
 */
static void Uart_Init()
{
	USART_InitTypeDef UART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// GPIO Config
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	UART_InitStructure.USART_BaudRate = 115200;
	UART_InitStructure.USART_WordLength = USART_WordLength_8b;
	UART_InitStructure.USART_StopBits = USART_StopBits_1;
	UART_InitStructure.USART_Parity = USART_Parity_No;
	UART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

	// TX - PB3 RX - PB4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	USART_Init(USART2,&UART_InitStructure);
	USART_Cmd(USART2, ENABLE);

	// NVIC Config

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

 	NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}




