/*
 * bt.h
 *
 *  Created on: 2 gru 2015
 *      Author: Marcin
 */

#ifndef INCLUDE_BT_H_
#define INCLUDE_BT_H_



#define COMMEND 0x2a
#define SEPARATOR 0x26  //1e

#define RXBUFSIZE 128

void BT_Init(void);

void BT_Send(volatile char *s);
void ClearBuffer(void);
void BT_TurnOn(void);
void BT_Reset(void);
void BT_Main();

#endif /* INCLUDE_BT_H_ */
