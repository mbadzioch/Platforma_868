/*=======================================================================================*
 * @file    radio.h
 * @author  Damian Pala
 * @date    20-02-2017
 * @brief   Header file for Radio module
 *
 *          This file contains API of Radio module
 *======================================================================================*/
/*----------------------- DEFINE TO PREVENT RECURSIVE INCLUSION ------------------------*/
#ifndef RADIO_H_
#define RADIO_H_

/**
 * @addtogroup Radio Description
 * @{
 * @brief Module for... .
 */

/*======================================================================================*/
/*                       ####### PREPROCESSOR DIRECTIVES #######                        */
/*======================================================================================*/
/*-------------------------------- INCLUDE DIRECTIVES ----------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "radio_reg.h"

/*----------------------------- LOCAL OBJECT-LIKE MACROS -------------------------------*/
//#define MODULATION_OOK_4800
//#define MODULATION_OOK_0600
#define MODULATION_GFSK_4800
//#define MODULATION_GFSK_1200

#define SENSOR_SCANNING_TIME_S                  62
#define SECOND_SENSOR_NOT_CONNECTED             0x3FF
#define ARCHIVAL_TRANSFER_TIMEOUT_MS            7000
#define RADIO_TX_TIMEOUT_MS                     100
#define RADIO_CHECK_CFG_TIMEOUT_MS              50000

/*---------------------------- LOCAL FUNCTION-LIKE MACROS ------------------------------*/

/*======================================================================================*/
/*                     ####### EXPORTED TYPE DECLARATIONS #######                       */
/*======================================================================================*/
/*-------------------------------- OTHER TYPEDEFS --------------------------------------*/

/*------------------------------------- ENUMS ------------------------------------------*/

/*------------------------------- STRUCT AND UNIONS ------------------------------------*/

/*======================================================================================*/
/*                    ####### EXPORTED OBJECT DECLARATIONS #######                      */
/*======================================================================================*/

/*======================================================================================*/
/*                   ####### EXPORTED FUNCTIONS PROTOTYPES #######                      */
/*======================================================================================*/
void RM_Init(void);
/**
 * @brief This is function that push data into global sensor received data structure
 */

uint8_t RM_GetCurrentRssi(void);
void RM_RadioTxTimeoutNotification(void);
void RM_Handler(void);
uint16_t RM_GetNetKeyFromRecorderSn(void);
/*======================================================================================*/
/*                          ####### INLINE FUNCTIONS #######                            */
/*======================================================================================*/

/**
 * @}
 */

#endif /* RADIO_H_ */
