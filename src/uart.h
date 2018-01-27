/*
 * uart.h
 *
 *  Created on: 14 lis 2016
 *      Author: Marcin
 */

#ifndef UART_H_
#define UART_H_


void UartInit(void);
void Uart_Block(void);
void PC_Send(volatile char *s);

#endif /* UART_H_ */
