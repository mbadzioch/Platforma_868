/*=======================================================================================*
 * @file    radio.c
 * @author  Damian Pala
 * @date    20-02-2017
 * @brief   This file contains all implementations for Radio module.
 *======================================================================================*/

/**
 * @addtogroup Radio Description
 * @{
 * @brief Module for... .
 */

/*======================================================================================*/
/*                       ####### PREPROCESSOR DIRECTIVES #######                        */
/*======================================================================================*/
/*-------------------------------- INCLUDE DIRECTIVES ----------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx_conf.h"
#include "radio.h"
#include "timer.h"
#include "radio_driver.h"

/*----------------------------- LOCAL OBJECT-LIKE MACROS -------------------------------*/
#define RADIO_FIFO_SIZE                                     64
#define RADIO_STD_FRAME_SIZE                                5
#define STD_FRAMES_FIFO_SIZE                                100
#define RECORDER_SENSORS_SLOTS                              4
#define LAST_FRAME_CNT_RESET_VALUE                          0
#define STATUS_BYTE_OFFSET                                  1
#define ARCHIVAL_FRAMES_FIFO_SIZE                           1000
#define ARCHIVAL_PACKET_STATUS_BYTE_OFFSET                  0
#define ARCHIVAL_PACKET_CNT_OFFSET                          1
#define ARCHIVAL_PACKET_CNT_RESET_VALUE                     0
#define ARCHIVAL_PACKET_FRAMES_IN_PACKET_CNT_OFFSET         2
#define ARCHIVAL_DATA_FRAME_SIZE                            3
#define ARCHIVAL_INFO_PACKET_CNT_OFFSET                     1
#define ARCHIVAL_PACKET_TO_SEND_BYTE_OFFSET                 5
#define ARCHIVAL_TID_BYTE_OFFSET                            6
#define ARCHIVAL_LOST_SIG_TIME_OFFSET                       7
#define ARCHIVAL_LOST_SIG_TIME_SIZE                         2
#define ARCHIVAL_TID_RESET_VALUE                            0
#define SENSOR_MODULE_ID_ERROR                              0xFFFF
#define UNPAIRED_SENSOR_MODULE_IDS_LIST_SIZE                32
#define PAIRED_SENSOR_MODULES_LIST_SIZE                     RECORDER_SENSORS_SLOTS
#define SENSOR_MODULE_LIST_CLEAR_SLOT                       0
#define SENSOR_MIN_FRAME_SIZE                               2

#define CREATE_STD_FIFO_ITEM_LOG                            1
#define CREATE_ARCH_FIFO_ITEM_LOG                           0

#define SENSOR_STATUS_SERVICE_PACKET                        0x80
#define SENSOR_STATUS_ADDED                                 0x40
#define SENSOR_STATUS_REC_ON                                0x20
#define SENSOR_STATUS_FIFO_LAST_PACKET                      0x10
#define SENSOR_STATUS_FIFO_INFO_FRAME                       0x08
#define SENSOR_STATUS_FIFO_PACKET                           0x04
#define SENSOR_STATUS_STD_FRAME                             0x02

#define RECORDER_STATUS_DATA_ACK                            0x80
#define RECORDER_STATUS_DATA_NACK                           0x40
#define RECORDER_STATUS_REC_ON                              0x20
#define RECORDER_STATUS_REC_OFF                             0x10
#define RECORDER_STATUS_SENS_ADDED                          0x08
#define RECORDER_STATUS_SENS_REMOVED                        0x04
#define RECORDER_STATUS_ARCHIVAL_TRANS_ALREADY_RXD          0x02

#define BT_RADIO_LOG_BUFFER_SIZE                            256

/*---------------------------- LOCAL FUNCTION-LIKE MACROS ------------------------------*/
#define IS_VALID_PACKED_RECEIVED_INTERRUPT(status)          (status & VALID_PACKED_RECEIVED_INTERRUPT)
#define IS_PACKAGE_SENT_INTERRUPT(status)                   (status & PACKET_SENT_INTERRUPT)
#define IS_CRC_ERROR_INTERRUPT(status)                      (status & CRC_ERROR)
#define IS_VALID_PREAMBLE_DETECTED(status)                  (status & VALID_PREAMBLE_DETECTED_IRQ_bm)
#define IS_VALID_SYNC_WORD_DETECTED(status)                 (status & SYNC_WORD_DETECTED_IRQ_bm)
#define IS_POWER_ON_RESET_DETECTED(status)                  (status & POR_IRQ_bm)

#define GET_SLAVE_SENSOR_ID(masterId)                       (masterId + 1)
#define GET_MASTER_SENSOR_ID(slaveId)                       (slaveId - 1)

#define IS_RECEIVED_STD_FRAME(status)                       (status & SENSOR_STATUS_STD_FRAME)
#define IS_SENSOR_PAIRED(status)                            (status & SENSOR_STATUS_ADDED)
#define IS_RECEIVED_ARCHIVAL_INFO_PACKET(status)            (status & SENSOR_STATUS_FIFO_INFO_FRAME)
#define IS_RECEIVED_ARCHIVAL_DATA_PACKET(status)            (status & SENSOR_STATUS_FIFO_PACKET)
#define IS_RECEIVED_ARCHIVAL_END_PACKET(status)             (status & SENSOR_STATUS_FIFO_LAST_PACKET)
#define IS_RECEIVED_SERVICE_PACKET(status)                  (status & SENSOR_STATUS_SERVICE_PACKET)

#ifndef __FILENAME__
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

#if defined(RADIO_LOG) && defined(DEBUG_TRACE)
  #define LOG_PUTS(text)                                    UTRACE_Printf("%s:%d " text, __FILENAME__, __LINE__);
  #define LOG_PRINTF(text, ...)                             UTRACE_Printf("%s:%d " text, __FILENAME__, __LINE__, __VA_ARGS__)
#endif

#ifndef LOG_PUTS
  #define LOG_PUTS(text)
#endif
#ifndef LOG_PRINTF
  #define LOG_PRINTF(text, ...)
#endif

#define BT_RADIO_LOG_PUTS(text, ...) \
  if (true == DebugConfiguration.showRadioBtLog) \
  { \
    sprintf((char*)BtRadioLogBuffer, text); \
    BT_Send((volatile char*)BtRadioLogBuffer); \
  }

#define BT_RADIO_LOG_PRINTF(text, ...) \
  if (true == DebugConfiguration.showRadioBtLog) \
  { \
    sprintf((char*)BtRadioLogBuffer, text, __VA_ARGS__); \
    BT_Send((volatile char*)BtRadioLogBuffer); \
  }

/*======================================================================================*/
/*                      ####### LOCAL TYPE DECLARATIONS #######                         */
/*======================================================================================*/
/*-------------------------------- OTHER TYPEDEFS --------------------------------------*/

/*------------------------------------- ENUMS ------------------------------------------*/
typedef enum SensorType_Tag
{
  SENSOR_TYPE_TEMPERATURE = 0,
  SENSOR_TYPE_HUMIDITY
} SensorType_T;

typedef enum SignalLevelRSSI_Tag
{
  SIGNAL_LEVEL_1_RSSI = 130,
  SIGNAL_LEVEL_2_RSSI = 150,
  SIGNAL_LEVEL_3_RSSI = 190
} SignalLevelRSSI_T;

typedef enum SignalLevel_Tag
{
  SIGNAL_LEVEL_0 = 0,
  SIGNAL_LEVEL_1,
  SIGNAL_LEVEL_2,
  SIGNAL_LEVEL_3,
} SignalLevel_T;

typedef enum ReceivedDataType_Tag
{
  RECEIVED_DATA_TYPE_STD_FRAME = 1,
  RECEIVED_DATA_TYPE_ARCHIVAL_PACKET,
  RECEIVED_DATA_TYPE_SERVICE_PACKET,
  RECEIVED_DATA_TYPE_UNKNOWN,
} ReceivedDataType_T;

typedef enum SensorDataType_Tag
{
  SENSOR_DATA_TYPE_STD_FRAME = 1,
  SENSOR_DATA_TYPE_ARCHIVAL,
} SensorDataType_T;

typedef enum ArchivalFifoStatus_Tag
{
  ARCHIVAL_FIFO_READY_TO_GET_DATA = 1,
  ARCHIVAL_FIFO_EMPTY,
} ArchivalFifoStatus_T;

typedef enum SensorModuleConnectionStatus_Tag
{
  SENSOR_MODULE_UNPAIRED_WITH_THIS_RECORDER = 1,
  SENSOR_MODULE_PAIRED_WITH_THIS_RECORDER
} SensorModuleConnectionStatus_T;

typedef enum DataRecordingStatus_Tag
{
  DATA_RECORDING_OFF = 1,
  DATA_RECORDING_ON
} DataRecordingStatus_T;

typedef enum SensorIdType_Tag
{
  SENSOR_ID_TYPE_MASTER = 1,
  SENSOR_ID_TYPE_SLAVE,
} SensorIdType_T;

typedef enum ArchivalPacketHandlerStatus_Tag
{
  ARCH_PACKET_HANDLER_STATUS_IDLE = 1,
  ARCH_PACKET_HANDLER_STATUS_BUSY,
} ArchivalPacketHandlerStatus_T;

typedef enum ArchivalPacketHandlingState_Tag
{
  ARCH_HANDLING_STATE_START = 1,
  ARCH_HANDLING_STATE_GET_DATA,
  ARCH_HANDLING_STATE_END,
  ARCH_HANDLING_STATE_ERROR
} ArchivalPacketHandlingState_T;

/*------------------------------- STRUCT AND UNIONS ------------------------------------*/
typedef struct SensorsData_Tag
{
  int16_t temperature; ///< real temp * 10
  int16_t temperatureOrHumidity;
  uint8_t secondSensorStatusTempOrHumidity;
  uint8_t batteryLevel;
  uint8_t doorState;
} SensorsData_T;

typedef struct StdFrameFifoItem_Tag
{
  uint16_t sensorModuleId;
  int16_t temperature; ///< real temp * 10
  int16_t temperatureOrHumidity;
  uint8_t secondSensorStatusTempOrHumidity;
  uint8_t batteryLevel;
  uint8_t doorState;
  uint8_t signalLevel;
} StdFrameFifoItem_T;

typedef struct ArchivalFrameFifoItem_Tag
{
  int16_t temperature;
  int16_t temperatureOrHumidity;
  uint8_t secondSensorStatusTempOrHumidity;
  uint8_t doorState;
} ArchivalFrameFifoItem_T;

typedef struct PairedSensorModuleListItem_Tag
{
  uint16_t sensorModuleId;
  uint16_t lastReceivedFrameCnt;
  uint16_t lastReceivedArchivalTid;
} PairedSensorModuleListItem_T;

typedef struct ArchivalTransferClientData_Tag
{
  uint16_t sensorModuleId;
  uint8_t archivalTID;
  uint8_t packetsToTransfer;
  uint16_t lostSignalTime;
  uint8_t lastArchivalDataPacketCnt;
  ArchivalPacketHandlerStatus_T archivalPacketHandlerStatus;
} ArchivalTransferClientData_T;

typedef struct ArchivalFifoData_Tag
{
  ArchivalFifoStatus_T archivalFifoStatus;
  uint16_t sensorModuleId;
  uint16_t lostSignalTime;
  uint16_t itemsLeftToGetByParent;
} ArchivalFifoData_T;

/*======================================================================================*/
/*                         ####### OBJECT DEFINITIONS #######                           */
/*======================================================================================*/
/*--------------------------------- EXPORTED OBJECTS -----------------------------------*/

/*---------------------------------- LOCAL OBJECTS -------------------------------------*/
static volatile uint8_t ReceivedDataBuffer[RADIO_FIFO_SIZE];
static volatile uint8_t ReceivedStatusByte = 0;
static volatile uint16_t ReceivedSensorModuleID = 0;
static volatile uint8_t RssiDuringReceiving = 0;
static bool RecorderStartedLookingForNewSensors;
static volatile bool IsPowerOnResetDetected = false;
static volatile bool IsDuringRadioReset = false;

static volatile uint8_t BtRadioLogBuffer[BT_RADIO_LOG_BUFFER_SIZE];
static bool IsRecorderInSensorCheckerMode = false;
static volatile SensorsData_T SensorsDataInSensorCheckerMode;
static volatile bool IsNewDataForSensorChecker = false;

/*======================================================================================*/
/*                    ####### LOCAL FUNCTIONS PROTOTYPES #######                        */
/*======================================================================================*/
void TxEndNotification(void);
void RxEndNotification(void);
void CrcErrorNotification(void);
void SyncWordDetectedNotification(void);

SignalLevel_T GetSignalLevel(uint8_t signalLevelRssi);

/**
 * @brief This function checks if sensor is paired with recorder based on its ID
 * @param [in] sensorModuleId is id of sensor module that data was received
 * @param [out] sensorModuleIndex is index in recorder list of sensor module that data was received
 * @return sensor module pairing status
 */
static ReceivedDataType_T RadioReceiveData(void);
static bool IsSensorFrame(uint8_t receivedDataLen);
static void RadioHandleReceivedData(void);
static void RadioSendResponse(uint8_t response);
void RadioMoveToRxStateWithFifosClear(void);
static bool IsSensorModuleIdValid(uint16_t id);




/*======================================================================================*/
/*                  ####### EXPORTED FUNCTIONS DEFINITIONS #######                      */
/*======================================================================================*/
void RM_Init(void)
{
  RD_RadioInterruptDisable();

  RD_Init();
  SetNetKeyInRadio();
  RadioMoveToRxStateWithFifosClear();

  RD_RadioInterruptEnable();
}

/*======================================================================================*/
/*                   ####### LOCAL FUNCTIONS DEFINITIONS #######                        */
/*======================================================================================*/
void TxEndNotification(void)
{
  RadioMoveToRxStateWithFifosClear();
 // TimerStop(&RadioTxTimeoutTimer);
}

void RxEndNotification(void)
{
  /* Move radio to IDLE state, IRQ from RX or TX will not occur */
  RD_RadioMoveToState(RADIO_STATE_IDLE);
  RadioHandleReceivedData();
}

void CrcErrorNotification(void)
{
  /* Do nothing because only recorder take data only if received valid package */
}

void SyncWordDetectedNotification(void)
{
  RssiDuringReceiving = RD_RadioSendReadTransaction(RSSI_LEVEL_26_REG);
}


SignalLevel_T GetSignalLevel(uint8_t signalLevelRssi)
{
  SignalLevel_T signalLevel;

  if (signalLevelRssi >= SIGNAL_LEVEL_3_RSSI)
  {
    signalLevel = SIGNAL_LEVEL_3;
  }
  else if ( (signalLevelRssi < SIGNAL_LEVEL_3_RSSI) && (signalLevelRssi >= SIGNAL_LEVEL_2_RSSI) )
  {
    signalLevel = SIGNAL_LEVEL_2;
  }
  else if ( (signalLevelRssi < SIGNAL_LEVEL_2_RSSI) && (signalLevelRssi >= SIGNAL_LEVEL_1_RSSI) )
  {
    signalLevel = SIGNAL_LEVEL_1;
  }
  else /* SigLevel < SIGNAL_LEVEL_1_RSSI */
  {
    signalLevel = SIGNAL_LEVEL_0;
  }

  return signalLevel;
}


static ReceivedDataType_T RadioReceiveData(void)
{
  ReceivedDataType_T ret;
  uint8_t receivedDataLen;

  receivedDataLen = RD_RadioSendReadTransaction(RECEIVED_PACKET_LENGTH_4B_REG);

  if (true == IsSensorFrame(receivedDataLen))
  {
    if (receivedDataLen <= RADIO_FIFO_SIZE)
    {
      RD_RadioReceiveData((uint8_t *)ReceivedDataBuffer, receivedDataLen);
      RD_RadioClearRxFifo();

      uint8_t receivedHeader[4];

      RD_GetRxHeader4B(receivedHeader);
      ReceivedSensorModuleID = BUILD_UINT16(receivedHeader[3], receivedHeader[2]);
      ReceivedStatusByte = ReceivedDataBuffer[0];
    }
    else
    {
      ReceivedSensorModuleID = SENSOR_MODULE_ID_ERROR;
      ReceivedStatusByte = 0;
    }

    if ( (IS_RECEIVED_STD_FRAME(ReceivedStatusByte)) && (true == IsSensorModuleIdValid(ReceivedSensorModuleID)) )
    {
      ret = RECEIVED_DATA_TYPE_STD_FRAME;
    }
    else if ( ( (IS_RECEIVED_ARCHIVAL_INFO_PACKET(ReceivedStatusByte)) || (IS_RECEIVED_ARCHIVAL_DATA_PACKET(ReceivedStatusByte)) ||
                (IS_RECEIVED_ARCHIVAL_END_PACKET(ReceivedStatusByte)) ) && (true == IsSensorModuleIdValid(ReceivedSensorModuleID)) )
    {
      ret = RECEIVED_DATA_TYPE_ARCHIVAL_PACKET;
    }
    else if (IS_RECEIVED_SERVICE_PACKET(ReceivedStatusByte))
    {
      ret = RECEIVED_DATA_TYPE_SERVICE_PACKET;
    }
    else
    {
      ret = RECEIVED_DATA_TYPE_UNKNOWN;
    }
  }
  else
  {
    ret = RECEIVED_DATA_TYPE_UNKNOWN;
//    LOG_PUTS("Received unknown data!\r\n");
  }

  return ret;
}

static bool IsSensorFrame(uint8_t receivedDataLen)
{
  if (receivedDataLen >= SENSOR_MIN_FRAME_SIZE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

static bool IsSensorModuleIdValid(uint16_t id)
{
  if ((id != 0) && (id != SENSOR_MODULE_ID_ERROR))
  {
    return true;
  }
  else
  {
    return false;
  }
}

static void RadioHandleReceivedData(void)
{
  ReceivedDataType_T ReceivedDataType;
  SensorModuleConnectionStatus_T sensorModuleConnectionStatus;

  ReceivedDataType = RadioReceiveData();

  sensorModuleConnectionStatus = IsSensorModulePaired(ReceivedSensorModuleID);

  if (SENSOR_MODULE_PAIRED_WITH_THIS_RECORDER == sensorModuleConnectionStatus)
  {
//    LOG_PRINTF("Received msg from paired sensor with recorder, status byte = %X\r\n", ReceivedStatusByte);
    switch(ReceivedDataType)
    {
      case RECEIVED_DATA_TYPE_STD_FRAME:
      {
        CreateStdFifoItemAndPush((uint8_t * const)ReceivedDataBuffer + STATUS_BYTE_OFFSET);

        break;
      }
      case RECEIVED_DATA_TYPE_ARCHIVAL_PACKET:
      {
        ArchivalPacketHandler();

        break;
      }
      case RECEIVED_DATA_TYPE_SERVICE_PACKET:
      {
        uint32_t packetCnt;

        packetCnt = BUILD_UINT16(ReceivedDataBuffer[1], ReceivedDataBuffer[2]);
        LOG_PRINTF("Service packet - Module ID: %x, packet cnt: %u\r\n", ReceivedSensorModuleID, packetCnt);
        (void)packetCnt;

        RadioSendResponse(RECORDER_STATUS_DATA_ACK);

        break;
      }
      case RECEIVED_DATA_TYPE_UNKNOWN:
      default:
      {
        RadioSendResponse(RECORDER_STATUS_DATA_NACK | RECORDER_STATUS_SENS_ADDED);

        break;
      }
    }

    if ( (true == RecorderStartedLookingForNewSensors) &&
         ( (RECEIVED_DATA_TYPE_STD_FRAME == ReceivedDataType) || (RECEIVED_DATA_TYPE_ARCHIVAL_PACKET == ReceivedDataType) ) )
    {
      AddSensorModuleToDiscoveredSensorsList((uint8_t * const)ReceivedDataBuffer + STATUS_BYTE_OFFSET);
    }
    else
    {
      /* Do nothing */
    }
  }
  else if ((SENSOR_MODULE_UNPAIRED_WITH_THIS_RECORDER == sensorModuleConnectionStatus) && (true == RecorderStartedLookingForNewSensors))
  {
    LOG_PRINTF("Received msg from sensor unpaired with recorder in sensor search mode, status byte = %X\r\n", ReceivedStatusByte);

    if ( (true == IsSensorModuleReadyToPair()) &&
         ( (RECEIVED_DATA_TYPE_STD_FRAME == ReceivedDataType) || (RECEIVED_DATA_TYPE_ARCHIVAL_PACKET == ReceivedDataType) ) )
    {
      AddSensorModuleToDiscoveredSensorsList((uint8_t * const)ReceivedDataBuffer + STATUS_BYTE_OFFSET);
    }
    else
    {
      /* Drop received data when sensor is paired with other recorder */
    }

    RadioMoveToRxStateWithFifosClear();
    if (true == UnpairedSensorModuleIDsList_IsItemOnList(ReceivedSensorModuleID))
    {
      RadioSendResponse(RECORDER_STATUS_DATA_ACK | RECORDER_STATUS_SENS_REMOVED);
      UnpairedSensorModuleIDsList_PopItem(ReceivedSensorModuleID);
    }
    else
    {
      /* Do nothing */
    }
  }
  else if ((SENSOR_MODULE_UNPAIRED_WITH_THIS_RECORDER == sensorModuleConnectionStatus) && IsRecorderInSensorCheckerMode)
  {
    uint8_t frameCnt;

    GetSensorsDataFromFrame((uint8_t * const)ReceivedDataBuffer + STATUS_BYTE_OFFSET, (SensorsData_T*)&SensorsDataInSensorCheckerMode, &frameCnt);
    IsNewDataForSensorChecker = true;

    RadioMoveToRxStateWithFifosClear();
    RadioSendResponse(RECORDER_STATUS_DATA_ACK);
  }
  else
  {
    /* Drop received data */
    LOG_PRINTF("Received msg from sensor: %x unpaired with recorder, status byte = %X\r\n", ReceivedSensorModuleID, ReceivedStatusByte);

    RadioMoveToRxStateWithFifosClear();
    if (true == UnpairedSensorModuleIDsList_IsItemOnList(ReceivedSensorModuleID))
    {
      RadioSendResponse(RECORDER_STATUS_DATA_ACK | RECORDER_STATUS_SENS_REMOVED);
      UnpairedSensorModuleIDsList_PopItem(ReceivedSensorModuleID);
    }
    else
    {
      /* Do nothing */
    }
  }
}


static void RadioSendResponse(uint8_t response)
{
  uint8_t newTxHeader[4];
  uint16_t netKey = RM_GetNetKeyFromRecorderSn();

  //TODO: ustawiać header poprawnie
  newTxHeader[3] = (uint8_t)GET_MSB(ReceivedSensorModuleID);
  newTxHeader[2] = (uint8_t)GET_LSB(ReceivedSensorModuleID);
  newTxHeader[1] = (uint8_t)GET_MSB(netKey);
  newTxHeader[0] = (uint8_t)GET_LSB(netKey);
  RD_SetTxHeader4B(newTxHeader);

  RD_RadioTransmitData(&response, 1);
}

void RadioMoveToRxStateWithFifosClear(void)
{
  RD_RadioClearRxFifo();
  RD_RadioClearTxFifo();
  RD_RadioMoveToState(RADIO_STATE_RX);
}


/*
 * Obsługa przerwania nIRQ z RFM22
 */
void EXTI4_15_IRQHandler(void)
{
	uint8_t interrupt1Status = 0;
	uint8_t interrupt2Status = 0;

	if (EXTI_GetITStatus(EXTI_Line8) != RESET){
		EXTI_ClearITPendingBit(EXTI_Line8);

	    interrupt1Status = RD_RadioSendReadTransaction(INTERRUPT_STATUS1_03_REG);
	    interrupt2Status = RD_RadioSendReadTransaction(INTERRUPT_STATUS2_04_REG);

	    if(IS_POWER_ON_RESET_DETECTED(interrupt2Status) && (false == IsDuringRadioReset) &&
	      (true == RD_IsRadioDriverInitialized()))
	    {
	      IsPowerOnResetDetected = true;
	    }

	    if(IS_VALID_PACKED_RECEIVED_INTERRUPT(interrupt1Status))
	    {
	      RxEndNotification();
	    }
	    else if(IS_PACKAGE_SENT_INTERRUPT(interrupt1Status))
	    {
	      TxEndNotification();
	    }
	    else if(IS_VALID_SYNC_WORD_DETECTED(interrupt2Status))
	    {
	      SyncWordDetectedNotification();
	    }
	    else if(IS_CRC_ERROR_INTERRUPT(interrupt1Status))
	    {
	      //CrcErrorNotification();
	    }
	    else
	    {
	      /* Do Nothing */
	    }

	}
}

/**
 * @}
 */
