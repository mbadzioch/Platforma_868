/*
 * komunikacja.h
 *
 *  Created on: 12 wrz 2016
 *      Author: Marcin
 */

#ifndef KOMUNIKACJA_H_
#define KOMUNIKACJA_H_


#define DEBUG_BUF_LENGTH 256

void Debug_Init(void);

void Debug_Main(void);
void PC_Debug(volatile char *s);


#endif /* KOMUNIKACJA_H_ */
