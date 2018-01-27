/*
 * bt.c
 *
 *  Created on: 2 gru 2015
 *      Author: Marcin
 */

#include "bt.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_conf.h"
#include "delay.h"

uint8_t epCmd[]={0x01, 0x27, 0xFC};
uint8_t epCfgData[]={0x06, 0x00, 0x07, 0x03, 0x08, 0x1F, 0x00, 0x14, 0x00, 0x0B, 0x11, 0x44, 0x52, 0x32, 0x30, 0x33, 0x2D, 0x32, 0x30, 0x31, 0x35, 0x30, 0x39, 0x37, 0x38, 0x00, 0x00, 0x10, 0x05, 0x00, 0x20, 0x02, 0x01, 0x00, 0x04, 0x00, 0x2B, 0x01, 0x02, 0x05, 0x00, 0x46, 0x02, 0x11, 0x44, 0x07, 0x00, 0x4B, 0x04, 0x31, 0x32, 0x33, 0x34, 0x05, 0x00, 0xB1, 0x02, 0x00, 0x60, 0x04, 0x00, 0xB4, 0x01, 0x08, 0x04, 0x00, 0xB6, 0x01, 0x01, 0x05, 0x00, 0xB8, 0x02, 0x03, 0x20, 0x0A, 0x00, 0xBC, 0x07, 0x7D, 0x01, 0x0C, 0xFF, 0xFF, 0x01, 0x00, 0x05, 0x00, 0xC4, 0x02, 0x01, 0x00, 0x10, 0x00, 0xC7, 0x0D, 0x00, 0x0C, 0x00, 0x00, 0x08, 0x00, 0x08, 0x00, 0x40, 0x09, 0x02, 0x1F, 0x40, 0x12, 0x00, 0xD8, 0x0F, 0xBA, 0xB0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x00, 0xEB, 0x0A, 0x18, 0x10, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0xF7, 0x06, 0x01, 0x01, 0x01, 0x00, 0x02, 0x01, 0x09, 0x00, 0xFF, 0x06, 0x03, 0x02, 0x03, 0x00, 0x04, 0x02, 0x09, 0x01, 0x07, 0x06, 0x01, 0x03, 0x02, 0x00, 0x03, 0x01, 0x06, 0x01, 0x0F, 0x03, 0x3C, 0x1E, 0x1E, 0x05, 0x01, 0x13, 0x02, 0x1E, 0x1E, 0x14, 0x01, 0x17, 0x11, 0x05, 0x44, 0x52, 0x32, 0x30, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x01, 0x52, 0x10, 0x42, 0x54, 0x35, 0x30, 0x35, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x01, 0x72, 0x08, 0x00, 0x39, 0x50, 0x50, 0x01, 0x20, 0x00, 0x01, 0x09, 0x01, 0x8A, 0x06, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x31, 0x01, 0xC0, 0x2E, 0x11, 0x01, 0x01, 0x00, 0x06, 0x01, 0x03, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01, 0xF0, 0x02, 0x03, 0x02, 0x53, 0x02, 0xE0, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x03, 0x30, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x03, 0xA4, 0x01, 0x5E, 0x04, 0x03, 0xA8, 0x01, 0x5E, 0x53, 0x03, 0xB0, 0x50, 0x49, 0x53, 0x53, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x4D, 0x37, 0x37, 0x10, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x4D, 0x37, 0x37, 0x53, 0x50, 0x50, 0x30, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t epAck[]={0x04 ,0x0E ,0x04 ,0x01 ,0x27 ,0xFC ,0x00};


volatile uint8_t u8ReceiveBuffer[RXBUFSIZE], u8SendBuffer[RXBUFSIZE], u8RxCounter=0, u8OVFFlag=0, u8FrameReadyFlag=0,u8StartPoint=0,u8EndPoint=0,u8ConfigFlag=0,u8InitFlag=0,u8RebootDoneFlag=0,u8RebootFlag=0,u8FirstRunFlag=0,u8EProg=0,u8ERespRecFlag=0;
char u8DataBuffer[RXBUFSIZE];


extern char cBT_NamePrefix[12]="SentisaBT-";
extern char cRecorderID[12]="Miarka";

//uint8_t BT_ProgEepromOverride=0;		// DEBUG

void BT_TurnOff(void);


//==========================================PHY Config======================================================
void BT_Conf_UART(uint8_t baudflag)
{
	USART_InitTypeDef UART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	if(baudflag == 0){
		UART_InitStructure.USART_BaudRate = 115200;
	}
	else{
		UART_InitStructure.USART_BaudRate = 921600;
	}
	UART_InitStructure.USART_WordLength = USART_WordLength_8b;
	UART_InitStructure.USART_StopBits = USART_StopBits_1;
	UART_InitStructure.USART_Parity = USART_Parity_No;
	UART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART6,&UART_InitStructure);
	USART_Cmd(USART6, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
}

void BT_Conf_Pins(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*
	 * PA8 - Reset
	 * PC1 - P20
	 * PA10- RTS
	 * PA15- SWBTN
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA,GPIO_Pin_8);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_SetBits(GPIOC,GPIO_Pin_1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void BT_Conf_Int(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

 	NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
}
//==========================================Obsługa buforów======================================================

void ClearBuffer()
{
	for(uint8_t i=0; i < RXBUFSIZE; i++){
		u8ReceiveBuffer[i]=0;
		u8DataBuffer[i]=0;
	}
	u8RxCounter=0;
	u8StartPoint=0;
	u8EndPoint=0;
}
void BT_NameUpdate(void)
{
	uint8_t i=0,k=0;
	for(i=11;i<17;i++){
		epCfgData[i]=cBT_NamePrefix[k];
		k++;
	}
	k=0;
	for(i=17;i<25;i++){
		epCfgData[i]=cRecorderID[k];
		k++;
	}
}
//==========================================Interface======================================================
uint8_t BT_ERespCheck(void)
{
	uint8_t i=0,fail=0;
	//IWDG_ReloadCounter();
	while(!u8ERespRecFlag){
		Delay_ms(100);
		i++;
		if(i>5)fail=1;
	}
	u8ERespRecFlag=0;
	ClearBuffer();
	if(fail)return 0;
	return 1;
}
uint8_t BT_WriteEEPROM(void)
{
	uint16_t i=0;
	uint8_t p=0,z=0,k=0;

	while(p<31){	// p - 31 paczek z danymi
		for(k=0; k<3; k++){
			while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
			USART_SendData(USART6, epCmd[k]); // Send Char
		}
		k=0;			// k - iterator paczki
		z=epCfgData[i]+1;	// z - liczba bajtów w paczce
		while(k<z){
			while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
			USART_SendData(USART6, epCfgData[i]); // Send Char
			i++;
			k++;
		}
		if(BT_ERespCheck() == 0)return 0;
		p++;
	}
	return 1;
}
void BT_Program()
{
	BT_Conf_UART(0);

	GPIO_ResetBits(GPIOC,GPIO_Pin_1);
	BT_Reset();
	Delay_ms(500);

	u8EProg=1;
	if(BT_WriteEEPROM()==0){
		BT_TurnOff();
		Delay_ms(100);
		BT_TurnOn();
		Delay_ms(100);
		BT_Reset();
		Delay_ms(500);
		ClearBuffer();
		BT_WriteEEPROM();
	}
	u8EProg=0;
	GPIO_SetBits(GPIOC,GPIO_Pin_1);
	BT_Reset();

	BT_Conf_UART(1);
}
void BT_CheckConfig()
{
	static uint16_t cnt=0;
	u8ConfigFlag=1;

	//IWDG_ReloadCounter();

	if(u8FirstRunFlag==0){
		u8FirstRunFlag=1;
		u8RebootDoneFlag=0;
		u8RebootFlag=1;
		ClearBuffer();
		BT_Reset();
	}
	else if(u8RebootDoneFlag==1){
		/*if(BT_ProgEepromOverride){			DEBUG
			BT_Program();
		}*/
		u8RebootFlag=0;
		u8ConfigFlag=0;
		u8InitFlag=0;
		ClearBuffer();
	}
	else if(cnt>50000){
		BT_Program();
		u8RebootFlag=0;
		u8ConfigFlag=0;
		u8InitFlag=0;
		ClearBuffer();
	}
	cnt++;
}
//==========================================Podstawowe Funkcje Obsługi======================================================
void BT_TurnOn(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
}
void BT_TurnOff(void)
{
	USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
}
void BT_Reset(void)
{
	USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	Delay_us(100);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
}
void BT_Init()
{
	BT_NameUpdate();
	BT_Conf_UART(1);
	BT_Conf_Pins();
	BT_Conf_Int();
	BT_TurnOn();
	u8InitFlag=1;
	ClearBuffer();
}
//==========================================UART======================================================
void BT_Send(volatile char *s)
{
	uint32_t timeout=0;
	static uint8_t err=0;

	IWDG_ReloadCounter();
	while(*s){
	 // wait until data register is empty
		timeout=0;
		while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET); // Wait for Empty
		if(err<=2){
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10)){ // Flow Control!!
				timeout++;
				Delay_us(1);
				if(timeout > 500000){
					err++;
					break;
				}
			}
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10)==0){
				err=0;
			}
		}
		if(err>2){
			if(*s++ == '$'){
				//BT_Reset();
				//Delay_ms(200);
				err=0;
				break;
			}
		}
		else{
			USART_SendData(USART6, *s++); // Send Char
		}
	}
}
void USART6_IRQHandler(void)
{
	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET){
		if (USART_GetFlagStatus(USART6, USART_FLAG_RXNE)){
			if(u8RxCounter < RXBUFSIZE){
				u8ReceiveBuffer[u8RxCounter] = USART_ReceiveData(USART6);

				if(u8ReceiveBuffer[u8RxCounter] == ':'){
					u8StartPoint = u8RxCounter;
				}
				else if(u8ReceiveBuffer[u8RxCounter] == '*'){
					u8EndPoint = u8RxCounter;
					u8FrameReadyFlag=1;
				}
				if(u8ConfigFlag==1){
					if(u8EProg==1){
						if((u8ReceiveBuffer[u8RxCounter] == 0x00) && (u8ReceiveBuffer[u8RxCounter-1] == 0xFC)){
							if((u8ReceiveBuffer[u8RxCounter-2] == 0x27) && (u8ReceiveBuffer[u8RxCounter-3] == 0x01)){
								u8ERespRecFlag=1;
							}
						}
					}
					else if(u8RebootFlag==1){
						if(u8RxCounter>1){
							if((u8ReceiveBuffer[u8RxCounter]=='^') && (u8ReceiveBuffer[u8RxCounter-1]=='T')){
								u8RebootDoneFlag=1;
							}
						}
					}
				}
				u8RxCounter++;
			}
			else{
				USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);
				u8OVFFlag=1;

			}
		}
	}
}

//==========================================/UART======================================================
//==========================================MAIN=======================================================
void BT_Main()
{
	static uint8_t u8BTState=1;

	if(u8InitFlag==1){
		BT_CheckConfig();
	}
	else
	{
		// Obsługa kom. przychodzącej
	}
	if(u8OVFFlag==1){
		u8OVFFlag=0;
		// Obsługa OVF!
		ClearBuffer();
		u8RxCounter=0;
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	}
}
