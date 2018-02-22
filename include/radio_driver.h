/*=======================================================================================*
 * @file    radio_driver.h
 * @author  Damian Pala
 * @date    XX-XX-20XX
 * @brief   Header file for XXX module
 *
 *          This file contains API of XXX module
 *======================================================================================*/
/*----------------------- DEFINE TO PREVENT RECURSIVE INCLUSION ------------------------*/
#ifndef RADIO_DRIVER_H_
#define RADIO_DRIVER_H_

/**
 * @addtogroup XXX Description
 * @{
 * @brief Module for... .
 */

/*======================================================================================*/
/*                       ####### PREPROCESSOR DIRECTIVES #######                        */
/*======================================================================================*/
/*-------------------------------- INCLUDE DIRECTIVES ----------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*----------------------------- LOCAL OBJECT-LIKE MACROS -------------------------------*/
#define RADIO_DEFAULT_PACKET_LENGTH             6

/*---------------------------- LOCAL FUNCTION-LIKE MACROS ------------------------------*/
#define GET_MSB(word)                               ((word >> 8) & 0xFF)
#define GET_LSB(word)                               (word & 0xFF)
/*======================================================================================*/
/*                     ####### EXPORTED TYPE DECLARATIONS #######                       */
/*======================================================================================*/
/*-------------------------------- OTHER TYPEDEFS --------------------------------------*/

/*------------------------------------- ENUMS ------------------------------------------*/
typedef enum RD_RadioMode_Tag
{
    RADIO_MODE_STANDBY = 1,
    RADIO_MODE_SLEEP,
    RADIO_MODE_SENSOR,
    RADIO_MODE_READY,
    RADIO_MODE_TUNE,
} RD_RadioMode_T;

typedef enum RD_RadioState_Tag
{
    RADIO_STATE_IDLE = 1,
    RADIO_STATE_RX,
    RADIO_STATE_TX,
} RD_RadioState_T;

/*------------------------------- STRUCT AND UNIONS ------------------------------------*/

/*======================================================================================*/
/*                    ####### EXPORTED OBJECT DECLARATIONS #######                      */
/*======================================================================================*/

/*======================================================================================*/
/*                   ####### EXPORTED FUNCTIONS PROTOTYPES #######                      */
/*======================================================================================*/
void RD_Init(void);
void RD_RadioSendWriteTransaction(uint8_t address, uint8_t data);
uint8_t RD_RadioSendReadTransaction(uint8_t address);
void RD_RadioSendWriteBurst(uint8_t address, uint8_t *dataIn, uint16_t size);
void RD_RadioSendReadBurst(uint8_t address, uint8_t *dataOut, uint16_t size);
void RD_RadioTransmitData(uint8_t *dataIn, uint16_t size);
void RD_RadioReceiveData(uint8_t *dataOut, uint16_t size);
void RD_RadioMoveToState(RD_RadioState_T state);
void RD_RadioMoveToMode(RD_RadioMode_T mode);
void RD_RadioSleep(void);
void RD_ReconfigRadioIfNeeded(void);
void RD_RadioInterruptEnable(void);
void RD_RadioInterruptDisable(void);
void RD_RadioConfig(void);
void RD_SetTxHeader4B(uint8_t *byte);
void RD_SetRxCheckHeader4B(uint8_t *byte);
void RD_GetRxHeader4B(uint8_t *byte);

void RD_ClearInterruptsStatus(void);
void RD_RadioClearTxFifo(void);
void RD_RadioClearRxFifo(void);
bool RD_IsRadioDriverInitialized(void);
void EXTI4_15_IRQHandler(void);
/*======================================================================================*/
/*                          ####### INLINE FUNCTIONS #######                            */
/*======================================================================================*/

/**
 * @}
 */

#endif /* RADIO_DRIVER_H_ */
