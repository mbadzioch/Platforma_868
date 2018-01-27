#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx_conf.h"
#include "delay.h"
#include "uart.h"
#include "tm_stm32f4_ds18b20.h"

static TM_OneWire_t OneWire;
uint8_t ROMval[8];
float sensTemp=0;
uint8_t cbuf[36];

GPIO_InitTypeDef GPIO_InitStructure;

void PerTempGet(int16_t *temp)
{
	static uint8_t errcnt=0;

	if((TM_DS18B20_AllDone(&OneWire))){
		if(!TM_DS18B20_Read(&OneWire, ROMval, &sensTemp)){
			errcnt++;
			if(TM_OneWire_First(&OneWire)){
				TM_OneWire_GetFullROM(&OneWire,ROMval);
				TM_DS18B20_SetResolution(&OneWire, ROMval, TM_DS18B20_Resolution_12bits);
				TM_DS18B20_StartAll(&OneWire);
				if(errcnt>100){
					*temp=3100;
				}
			}
			else{
				if(errcnt>100){
					*temp=3000;
				}
			}
		}
		else{
			errcnt=0;
			TM_DS18B20_StartAll(&OneWire);
			*temp = (int16_t)(1000*sensTemp);
		}
	}
}

int main()
{
	int16_t temperatura;
	SystemInit();
	DelayInit();
	UartInit();

	/*RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	OneWire.GPIOx = GPIOB;
	OneWire.GPIO_Pin = GPIO_Pin_6;
	if(TM_OneWire_First(&OneWire)){
		TM_OneWire_GetFullROM(&OneWire,ROMval);
		TM_DS18B20_SetResolution(&OneWire, ROMval, TM_DS18B20_Resolution_12bits);
		TM_DS18B20_StartAll(&OneWire);
		temperatura=1;
	}
	else{
		temperatura=77;
	}
*/
  while (1)
    {
	 //PerTempGet(&temperatura);
	 // sprintf(cbuf,"Temp: %d\n\r",temperatura);
	  PC_Send("Czesc");
	  Delay_ms(1000);
    }
}

