/*=======================================================================================*
 * @file    radio_driver.c
 * @author  Damian Pala
 * @date    XX-XX-20XX
 * @brief   This file contains all implementations for XXX module.
 *======================================================================================*/

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
#include "delay.h"
#include "radio_driver.h"
#include "radio_reg.h"
#include "radio.h"

/*----------------------------- LOCAL OBJECT-LIKE MACROS -------------------------------*/
#define RADIO_SYNC_WORD                             0x46FA

//#define RADIO_SPI_PORT                              2
//#define RADIO_SPI_SCK_PIN                           10
//#define RADIO_SPI_MISO_PIN                          11
//#define RADIO_SPI_MOSI_PIN                          12
//#define RADIO_SPI_SS_PORT                           1
//#define RADIO_SPI_SS_PIN                            2
//#define RADIO_IRQ_PORT                              1
//#define RADIO_IRQ_PIN                               1

#define RD_RW_BM                                    (1 << 7)
#define RADIO_IDLE_STATE                            RADIO_STANDBY_MODE
#define RADIO_TX_STATE                              TX_ON_bm
#define RADIO_RX_STATE                              RX_ON_bm

#define RADIO_STANDBY_MODE                          0x00
#define RADIO_SLEEP_MODE                            EN_WT
#define RADIO_SENSOR_MODE                           EN_LBD
#define RADIO_READY_MODE                            XT_ON_bm
#define RADIO_TUNE_MODE                             PLL_ON_bm

//#define MODULATION_OOK_4800
//#define MODULATION_OOK_0600
#define MODULATION_GFSK_4800
//#define MODULATION_GFSK_1200
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
#define RD_ADRESS_AS_WRITE(address)                 ( RD_RW_BM | (address & 0x7F) )
#define RD_ADRESS_AS_READ(address)                  ( ~RD_RW_BM & (address & 0x7F) )

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

/*======================================================================================*/
/*                      ####### LOCAL TYPE DECLARATIONS #######                         */
/*======================================================================================*/
/*-------------------------------- OTHER TYPEDEFS --------------------------------------*/

/*------------------------------------- ENUMS ------------------------------------------*/

/*------------------------------- STRUCT AND UNIONS ------------------------------------*/

/*======================================================================================*/
/*                         ####### OBJECT DEFINITIONS #######                           */
/*======================================================================================*/
/*--------------------------------- EXPORTED OBJECTS -----------------------------------*/

/*---------------------------------- LOCAL OBJECTS -------------------------------------*/
static bool IsRadioDriverInitialized = false;
static bool u8RFIntEnaFlag = false;
static volatile bool IsPowerOnResetDetected = false;
static volatile bool IsDuringRadioReset = false;

static volatile uint8_t ReceivedDataBuffer[64];
static volatile uint8_t ReceivedStatusByte = 0;
static volatile uint16_t ReceivedSensorModuleID = 0;
static volatile uint8_t RssiDuringReceiving = 0;
/*======================================================================================*/
/*                    ####### LOCAL FUNCTIONS PROTOTYPES #######                        */
/*======================================================================================*/
static void GpioInit(void);
static void SpiInit(void);
static void RadioInterruptInit(void);
static inline void SpiSendData(uint8_t data);
inline uint8_t SpiReceiveData(void);
static inline void RadioClearFifos(void);
static inline void RadioSelectActive(void);
static inline void RadioSelectInactive(void);
static void RadioSwReset(void);

/*======================================================================================*/
/*                  ####### EXPORTED FUNCTIONS DEFINITIONS #######                      */
/*======================================================================================*/
void RD_Init(void)
{
  IsRadioDriverInitialized = false;

  GpioInit();
  SpiInit();
 // RadioInterruptInit(); TODO: ENABLE THIS
  RD_RadioConfig();

  IsRadioDriverInitialized = true;
}

void RD_RadioSendWriteTransaction(uint8_t address, uint8_t data)
{
  RadioSelectActive();
  SpiSendData(RD_ADRESS_AS_WRITE(address));
  SpiReceiveData();
  SpiSendData(data);
  SpiReceiveData();
  RadioSelectInactive();
}

uint8_t RD_RadioSendReadTransaction(uint8_t address)
{

  RadioSelectActive();
  SpiSendData(RD_ADRESS_AS_READ(address));
  SpiReceiveData();
  SpiSendData(0xFF);
  RadioSelectInactive();

  return SpiReceiveData();
}

void RD_RadioSendWriteBurst(uint8_t address, uint8_t *dataIn, uint16_t size)
{
  RadioSelectActive();
  SpiSendData(RD_ADRESS_AS_WRITE(address));
  SpiReceiveData();
  for (uint16_t i = 0; i < size; i++)
  {
    SpiSendData(*(dataIn++));
    SpiReceiveData();
  }
  RadioSelectInactive();
}

void RD_RadioSendReadBurst(uint8_t address, uint8_t *dataOut, uint16_t size)
{
  RadioSelectActive();
  SpiSendData(RD_ADRESS_AS_READ(address));
  SpiReceiveData();
  for (uint16_t i = 0; i < size; i++)
  {
    SpiSendData(0xFF);
    *(dataOut++) = SpiReceiveData();
  }
  RadioSelectInactive();
}

void RD_RadioTransmitData(uint8_t *dataIn, uint16_t size)
{
  RD_ClearInterruptsStatus();
  RD_RadioClearTxFifo();
  RD_RadioSendWriteTransaction(TRANSMIT_PACKET_LENGTH_3E_REG, size);
  RD_RadioSendWriteBurst(FIFO_ACCESS_7F_REG, dataIn, size);
  RD_RadioMoveToState(RADIO_STATE_TX);
}

void RD_RadioReceiveData(uint8_t *dataOut, uint16_t size)
{
  RD_RadioSendReadBurst(FIFO_ACCESS_7F_REG, dataOut, size);
}

void RD_ClearInterruptsStatus(void)
{
  (void)RD_RadioSendReadTransaction(INTERRUPT_STATUS1_03_REG);
  (void)RD_RadioSendReadTransaction(INTERRUPT_STATUS2_04_REG);
}

void RD_RadioClearTxFifo(void)
{
  RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL2_08_REG, FIFO_CLR_TX_bm);
  RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL2_08_REG, 0x00);
}

void RD_RadioClearRxFifo(void)
{
  RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL2_08_REG, FIFO_CLR_RX_bm);
  RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL2_08_REG, 0x00);
}

void RD_RadioSleep(void)
{
  RD_ClearInterruptsStatus();
  RadioClearFifos();
  RD_RadioMoveToMode(RADIO_MODE_STANDBY);
}

void RD_ReconfigRadioIfNeeded(void)
{
  if(RD_RadioSendReadTransaction(GPIO0_CONFIGURATION_0B_REG) != GPIO_TX_STATE)
  {
    RD_RadioConfig();
    RD_RadioClearRxFifo();
    RD_RadioClearTxFifo();
    RD_RadioMoveToState(RADIO_STATE_RX);
  }
  else
  {
    /* Do nothing */
  }
}

void RD_RadioMoveToMode(RD_RadioMode_T mode)
{
  switch(mode)
  {
    case RADIO_MODE_STANDBY:
    {
      RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL1_07_REG, RADIO_STANDBY_MODE);
      break;
    }
    case RADIO_MODE_SLEEP:
    {
      RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL1_07_REG, RADIO_SLEEP_MODE);
      break;
    }
    case RADIO_MODE_SENSOR:
    {
      RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL1_07_REG, RADIO_SENSOR_MODE);
      break;
    }
    case RADIO_MODE_READY:
    {
      RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL1_07_REG, RADIO_READY_MODE);
      break;
    }
    case RADIO_MODE_TUNE:
    {
      RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL1_07_REG, RADIO_TUNE_MODE);
      break;
    }
    default:
      break;
  }
}

void RD_RadioMoveToState(RD_RadioState_T state)
{
  switch(state)
  {
    case RADIO_STATE_IDLE:
    {
      RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL1_07_REG, RADIO_IDLE_STATE);
      break;
    }
    case RADIO_STATE_RX:
    {
      RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL1_07_REG, RADIO_RX_STATE);
      break;
    }
    case RADIO_STATE_TX:
    {
      RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL1_07_REG, RADIO_TX_STATE);
      break;
    }
    default:
      break;
  }
}

inline void RD_RadioInterruptEnable(void)
{
	u8RFIntEnaFlag = true;
}

inline void RD_RadioInterruptDisable(void)
{
	u8RFIntEnaFlag = false;
}

inline bool RD_IsRadioDriverInitialized(void)
{
  return IsRadioDriverInitialized;
}

void RD_RadioConfig(void)
{
  RadioSwReset();

  RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL1_07_REG, RADIO_READY_MODE);

  RD_RadioSendWriteTransaction(INTERRUPT_ENABLE1_05_REG, EN_CRC_ERROR_bm | EN_PK_RECEIVED_bm | EN_PK_SENT);
  RD_RadioSendWriteTransaction(INTERRUPT_ENABLE2_06_REG, EN_SYNC_WORD_DETECTED_bm | EN_POR_bm);
  RD_RadioSendWriteTransaction(CRISTAL_OSCILATOR_LOAD_CAP_09_REG, 0x7F);  // Set arbitrary to 12.5pF
  RD_RadioSendWriteTransaction(MICROCONTROLLER_OUTPUT_CLK_0A_REG, 0x00);
  RD_RadioSendWriteTransaction(GPIO0_CONFIGURATION_0B_REG, GPIO_TX_STATE);
  RD_RadioSendWriteTransaction(GPIO1_CONFIGURATION_0C_REG, GPIO_RX_STATE);
  RD_RadioSendWriteTransaction(GPIO2_CONFIGURATION_0D_REG, 0x00);
  RD_RadioSendWriteTransaction(IO_PORT_CONFIGURATION_0E_REG, 0x00);
  RD_RadioSendWriteTransaction(ADC_CONFIGURATION_0F_REG, 0x00);
  RD_RadioSendWriteTransaction(ADC_SENSOR_AMP_OFFSET_10_REG, 0x00);
  RD_RadioSendWriteTransaction(TEMP_SENSOR_CALIB_12_REG, 0x00);
  RD_RadioSendWriteTransaction(TEMP_VALUE_OFFSET_13_REG, 0x00);

  #ifdef MODULATION_OOK_4800
    /************* NOT REVIEWED *************/
  RD_RadioSendWriteTransaction(0x1C, 0xC9);   // IF filter bandwidth
  RD_RadioSendWriteTransaction(0x1D, 0x44);   // AFC Loop
  RD_RadioSendWriteTransaction(0x1E, 0x0A);     // AFC timing
  RD_RadioSendWriteTransaction(0x1F, 0x03);
  RD_RadioSendWriteTransaction(0x20, 0x9C);   // clock recovery
  RD_RadioSendWriteTransaction(0x21, 0x00);   // clock recovery
  RD_RadioSendWriteTransaction(0x22, 0xD1);   // clock recovery
  RD_RadioSendWriteTransaction(0x23, 0xB7);   // clock recovery
  RD_RadioSendWriteTransaction(0x24, 0x00);   // clock recovery timing
  RD_RadioSendWriteTransaction(0x25, 0xD4);   // clock recovery timing

  RD_RadioSendWriteTransaction(0x2A, 0x45);
  RD_RadioSendWriteTransaction(0x2C, 0x29);   // OOK ONLY
  RD_RadioSendWriteTransaction(0x2D, 0x04);
  RD_RadioSendWriteTransaction(0x2E, 0x29);

  RD_RadioSendWriteTransaction(0x6E, 0x27);   // TX data rate 1
  RD_RadioSendWriteTransaction(0x6F, 0x52);   // TX data rate 0

  RD_RadioSendWriteTransaction(0x71, 0x21);   // OOK
  RD_RadioSendWriteTransaction(0x72, 0x48);   // Frequency deviation setting to 45K=72*625
  #endif
  #ifdef MODULATION_OOK_0600
    /************* NOT REVIEWED *************/
  RD_RadioSendWriteTransaction(0x1C, 0x51);   // IF filter bandwidth
  RD_RadioSendWriteTransaction(0x1D, 0x44);   // AFC Loop
  RD_RadioSendWriteTransaction(0x1E, 0x0A);     // AFC timing
  RD_RadioSendWriteTransaction(0x1F, 0x03);
  RD_RadioSendWriteTransaction(0x20, 0xD0);   // clock recovery
  RD_RadioSendWriteTransaction(0x21, 0x00);   // clock recovery
  RD_RadioSendWriteTransaction(0x22, 0x9D);   // clock recovery
  RD_RadioSendWriteTransaction(0x23, 0x49);   // clock recovery
  RD_RadioSendWriteTransaction(0x24, 0x00);   // clock recovery timing
  RD_RadioSendWriteTransaction(0x25, 0xA0);   // clock recovery timing

  RD_RadioSendWriteTransaction(0x2A, 0x45);
  RD_RadioSendWriteTransaction(0x2C, 0x30);   // OOK ONLY
  RD_RadioSendWriteTransaction(0x2D, 0x23);
  RD_RadioSendWriteTransaction(0x2E, 0x29);

  RD_RadioSendWriteTransaction(0x6E, 0x04);   // TX data rate 1
  RD_RadioSendWriteTransaction(0x6F, 0xEA);   // TX data rate 0

  RD_RadioSendWriteTransaction(0x71, 0x21);   // OOK
  RD_RadioSendWriteTransaction(0x72, 0x48);   // Frequency deviation setting to 45K=72*625
  #endif
  #ifdef MODULATION_GFSK_4800

  #ifdef OTHER_RADIO_CFG
    /* ----- GFSK Modem settings -----
     * GFSK 4800kbps, 50ppm crystal, reg 1D to 0x44 - 0x40 in xls
     */
  RD_RadioSendWriteTransaction(IF_FILTER_BANDWIDTH_1C_REG, 0x99);
  RD_RadioSendWriteTransaction(AFC_LOOP_GEARSHIFT_OVERRIDE_1D_REG, 0x44);
  RD_RadioSendWriteTransaction(AFC_TIMING_CTRL_1E_REG, 0x0A);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_GEARSHIFT_OVERRIDE_1F_REG, 0x03);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_OVERSAMPLING_RATE_20_REG, 0xe2);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_OFFSET2_21_REG, 0x80);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_OFFSET1_22_REG, 0x1A);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_OFFSET0_23_REG, 0x37);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_TIMING_LOOP_GAIN1_24_REG, 0x00);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_TIMING_LOOP_GAIN0_25_REG, 0x08);
  RD_RadioSendWriteTransaction(RESERVED_2A_REG, 0x45);
  RD_RadioSendWriteTransaction(AGC_OVERRIDE_69_REG, 0x60); // Reserved but set in xls configurator


    /* ----- GFSK Modem settings -----
     * GFSK 4800kbps, 20ppm crystal from xls
     */
  RD_RadioSendWriteTransaction(IF_FILTER_BANDWIDTH_1C_REG, 0x1D);
  RD_RadioSendWriteTransaction(AFC_LOOP_GEARSHIFT_OVERRIDE_1D_REG, 0x40);
  RD_RadioSendWriteTransaction(AFC_TIMING_CTRL_1E_REG, 0x0A);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_GEARSHIFT_OVERRIDE_1F_REG, 0x03);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_OVERSAMPLING_RATE_20_REG, 0xA1);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_OFFSET2_21_REG, 0x20);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_OFFSET1_22_REG, 0x4E);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_OFFSET0_23_REG, 0xA5);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_TIMING_LOOP_GAIN1_24_REG, 0x00);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_TIMING_LOOP_GAIN0_25_REG, 0x13);
  RD_RadioSendWriteTransaction(RESERVED_2A_REG, 0x1E);
  RD_RadioSendWriteTransaction(AGC_OVERRIDE_69_REG, 0x60); // Reserved but set in xls configurator
  #endif
    /* ----- GFSK Modem settings -----
     * GFSK 4800kbps, 20ppm crystal from xls
     */
  RD_RadioSendWriteTransaction(IF_FILTER_BANDWIDTH_1C_REG, 0x99);
  RD_RadioSendWriteTransaction(AFC_LOOP_GEARSHIFT_OVERRIDE_1D_REG, 0x44);
  RD_RadioSendWriteTransaction(AFC_TIMING_CTRL_1E_REG, 0x0A);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_GEARSHIFT_OVERRIDE_1F_REG, 0x03);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_OVERSAMPLING_RATE_20_REG, 0xe2);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_OFFSET2_21_REG, 0x80);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_OFFSET1_22_REG, 0x1A);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_OFFSET0_23_REG, 0x37);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_TIMING_LOOP_GAIN1_24_REG, 0x00);
  RD_RadioSendWriteTransaction(CLK_RECOVERY_TIMING_LOOP_GAIN0_25_REG, 0x08);
  RD_RadioSendWriteTransaction(RESERVED_2A_REG, 0x45);
  RD_RadioSendWriteTransaction(AGC_OVERRIDE_69_REG, 0x60); // Reserved but set in xls configurator

    /* TX data rate 4800 kbps = 0x27 0x52*/
  RD_RadioSendWriteTransaction(TX_DATA_RATE1_6E_REG, 0x27);
  RD_RadioSendWriteTransaction(TX_DATA_RATE0_6F_REG, 0x52);

  RD_RadioSendWriteTransaction(MODULATION_MODE_CTRL2_71_REG, 0x23);   // GFSK without data inversion = 0x23
  RD_RadioSendWriteTransaction(FREQUENCY_DEVIATION_72_REG, 0x48);   // Frequency deviation setting 45kHz = 0x48; 30kHz = 0x30; def = 45K=72*625
  #endif
  #ifdef MODULATION_GFSK_1200
    /************* NOT REVIEWED *************/
  RD_RadioSendWriteTransaction(0x1C, 0x1B);   // IF filter bandwidth
  RD_RadioSendWriteTransaction(0x1D, 0x44);   // AFC Loop
  RD_RadioSendWriteTransaction(0x1E, 0x0A);     // AFC timing
  RD_RadioSendWriteTransaction(0x1F, 0x03);
  RD_RadioSendWriteTransaction(0x20, 0x83);   // clock recovery
  RD_RadioSendWriteTransaction(0x21, 0xC0);   // clock recovery
  RD_RadioSendWriteTransaction(0x22, 0x12);   // clock recovery
  RD_RadioSendWriteTransaction(0x23, 0xA9);   // clock recovery
  RD_RadioSendWriteTransaction(0x24, 0x00);   // clock recovery timing
  RD_RadioSendWriteTransaction(0x25, 0x04);   // clock recovery timing
  RD_RadioSendWriteTransaction(0x2A, 0x1C);

  RD_RadioSendWriteTransaction(0x2C, 0x2C);   // OOK ONLY
  RD_RadioSendWriteTransaction(0x2D, 0x11);
  RD_RadioSendWriteTransaction(0x2E, 0x2A);

  RD_RadioSendWriteTransaction(0x6E, 0x09);   // TX data rate 1
  RD_RadioSendWriteTransaction(0x6F, 0xD5);   // TX data rate 0

  RD_RadioSendWriteTransaction(0x71, 0x23);   // GFSK
  RD_RadioSendWriteTransaction(0x72, 0x30);   // Frequency deviation setting to 45K=72*625
  #endif

  RD_RadioSendWriteTransaction(RSSI_THRESHOLD_FOR_CLEAR_CH_INDICATOR_27_REG, 0x96); // RSSI threshold for CCA
  RD_RadioSendWriteTransaction(DATA_ACCESS_CTRL_30_REG, 0x8C); // Enabled packet rx, tx handling, enabled crc = 0x8C
  RD_RadioSendWriteTransaction(HEADER_CTRL1_32_REG, 0x33); // Expected broadcast and compare header 1 & 0
  RD_RadioSendWriteTransaction(HEADER_CTRL2_33_REG, 0x42); // Header 3, 2, 1, 0, Sync 3 & 2, packet length included in header = 0x22
  RD_RadioSendWriteTransaction(PREAMBLE_LEN_34_REG, 0x0A); // TODO: why 10? should be 16?
  RD_RadioSendWriteTransaction(PREAMBLE_DETECTION_CNTRL_35_REG, 0x22);    // 16 bits = 0x22
  RD_RadioSendWriteTransaction(SYNC_WORD3_36_REG, GET_MSB(RADIO_SYNC_WORD)); // Set arbitrarily
  RD_RadioSendWriteTransaction(SYNC_WORD2_37_REG, GET_LSB(RADIO_SYNC_WORD)); // Set arbitrarily
  RD_RadioSendWriteTransaction(SYNC_WORD1_38_REG, 0x00); // Set arbitrarily
  RD_RadioSendWriteTransaction(SYNC_WORD0_39_REG, 0x00); // Set arbitrarily
  RD_RadioSendWriteTransaction(HEADER3_3A_REG, 0x00);
  RD_RadioSendWriteTransaction(HEADER2_3B_REG, 0x00);
  RD_RadioSendWriteTransaction(HEADER1_3C_REG, 0x00);
  RD_RadioSendWriteTransaction(HEADER0_3D_REG, 0x00);
  RD_RadioSendWriteTransaction(TRANSMIT_PACKET_LENGTH_3E_REG, RADIO_DEFAULT_PACKET_LENGTH);
  RD_RadioSendWriteTransaction(CHECK_HEADER3_3F_REG, 0x00);
  RD_RadioSendWriteTransaction(CHECK_HEADER2_40_REG, 0x00);
  RD_RadioSendWriteTransaction(CHECK_HEADER1_41_REG, 0x00);
  RD_RadioSendWriteTransaction(CHECK_HEADER0_42_REG, 0x00);
  RD_RadioSendWriteTransaction(HEADER3_ENABLE_43_REG, 0x00);
  RD_RadioSendWriteTransaction(HEADER2_ENABLE_44_REG, 0x00);
  RD_RadioSendWriteTransaction(HEADER1_ENABLE_45_REG, 0xFF);
  RD_RadioSendWriteTransaction(HEADER0_ENABLE_46_REG, 0xFF);
  RD_RadioSendWriteTransaction(CHARGE_PUMP_CURRENT_TRIM_OVERR_58_REG, 0x80); // Reserved but set in xls configurator
  RD_RadioSendWriteTransaction(TX_POWER_6D_REG, TXPOW_20DBM);
  RD_RadioSendWriteTransaction(MODULATION_MODE_CTRL1_70_REG, 0x2C); // low data rates, manchester preamble polarity and data inv enable = 0x2C
  RD_RadioSendWriteTransaction(FREQ_OFFSET1_73_REG, 0x00);
  RD_RadioSendWriteTransaction(FREQ_OFFSET2_74_REG, 0x00);

  /* Frequency set 869MHz = 73 70 80 */
  RD_RadioSendWriteTransaction(FREQ_BAND_SEL_75_REG, 0x73);
  RD_RadioSendWriteTransaction(FREQ_NOMINAL_CARRIER_FREQ1_76_REG, 0x70);
  RD_RadioSendWriteTransaction(FREQ_NOMINAL_CARRIER_FREQ2_77_REG, 0x80);

  RD_RadioSendWriteTransaction(FREQ_HOPPING_CH_SEL_79_REG, 0x00); // no frequency hopping
  RD_RadioSendWriteTransaction(FREQ_HOPPING_STEP_SIZE_7A_REG, 0x00); // no frequency hopping

  RD_ClearInterruptsStatus();
}

void RD_SetTxHeader4B(uint8_t *byte)
{
  RD_RadioSendWriteTransaction(HEADER3_3A_REG, byte[3]);
  RD_RadioSendWriteTransaction(HEADER2_3B_REG, byte[2]);
  RD_RadioSendWriteTransaction(HEADER1_3C_REG, byte[1]);
  RD_RadioSendWriteTransaction(HEADER0_3D_REG, byte[0]);
}

void RD_SetRxCheckHeader4B(uint8_t *byte)
{
  RD_RadioSendWriteTransaction(CHECK_HEADER3_3F_REG, byte[3]);
  RD_RadioSendWriteTransaction(CHECK_HEADER2_40_REG, byte[2]);
  RD_RadioSendWriteTransaction(CHECK_HEADER1_41_REG, byte[1]);
  RD_RadioSendWriteTransaction(CHECK_HEADER0_42_REG, byte[0]);
}

void RD_GetRxHeader4B(uint8_t *byte)
{
  byte[3] = RD_RadioSendReadTransaction(RECEIVED_HEADER3_47_REG);
  byte[2] = RD_RadioSendReadTransaction(RECEIVED_HEADER2_48_REG);
  byte[1] = RD_RadioSendReadTransaction(RECEIVED_HEADER1_49_REG);
  byte[0] = RD_RadioSendReadTransaction(RECEIVED_HEADER0_4A_REG);
}

/*======================================================================================*/
/*                   ####### LOCAL FUNCTIONS DEFINITIONS #######                        */
/*======================================================================================*/
static void GpioInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;


  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* Initialize SPI pins */
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Mode                   = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType                  = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin                    = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStruct.GPIO_PuPd                   = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed                  = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_0);

  /* Initialize SPI SS pin */
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Mode                   = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType                  = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin                    = GPIO_Pin_12;
  GPIO_InitStruct.GPIO_PuPd                   = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed                  = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_SetBits(GPIOB, GPIO_Pin_12);

  /* Initialize radio IRQ pin */
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Mode                   = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_OType                  = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin                    = GPIO_Pin_8;
  GPIO_InitStruct.GPIO_PuPd                   = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed                  = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  // SDN PIN

  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Mode                   = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType                  = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin                    = GPIO_Pin_9;
  GPIO_InitStruct.GPIO_PuPd                   = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed                  = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_ResetBits(GPIOA, GPIO_Pin_9);
}

static void SpiInit(void)
{
  SPI_InitTypeDef SPI_InitStruct;

  SPI_StructInit(&SPI_InitStruct);
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // Configure SPI on 42 / 16 = 2,625 MHz
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  SPI_Init(SPI2, &SPI_InitStruct);

  SPI_Cmd(SPI2, ENABLE);
}

static void RadioInterruptInit(void)
{
  EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);

  EXTI_StructInit(&EXTI_InitStruct);
  EXTI_InitStruct.EXTI_Line = EXTI_Line8; // PB1 is connected to EXTI1
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_Init(&EXTI_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}

static inline void RadioSelectActive(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

static inline void RadioSelectInactive(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

static inline void SpiSendData(uint8_t data)
{

	SPI_SendData8(SPI2,data);
    /* Waiting until TX FIFO is empty */
    while (SPI_GetTransmissionFIFOStatus(SPI2) != SPI_TransmissionFIFOStatus_Empty);
    /* Wait busy flag */
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);

 /* SPI2->DR = data;
  while( !(SPI2->SR & SPI_I2S_FLAG_TXE) );
  while( !(SPI2->SR & SPI_I2S_FLAG_RXNE) );
  while( SPI2->SR & SPI_I2S_FLAG_BSY );
  data = SPI2->DR;*/
}

inline uint8_t SpiReceiveData(void)
{
	return SPI_ReceiveData8(SPI2);
 // return SPI2->DR;
}

static inline void RadioClearFifos(void)
{
  RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL2_08_REG, FIFO_CLR_TX_bm | FIFO_CLR_RX_bm);
  RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL2_08_REG, 0x00);
}

static void RadioSwReset(void)
{
  RD_RadioInterruptDisable();

  Delay_ms(1);
  RD_RadioSendWriteTransaction(OPERATING_AND_FNC_CTRL1_07_REG, SW_RES);
  Delay_ms(60);

  RD_RadioInterruptEnable();
}

/**
 * @}
 */
