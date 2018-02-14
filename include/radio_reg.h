/*=======================================================================================*
 * @file    header.h
 * @author  Damian Pala
 * @version 0.5
 * @date    XX-XX-20XX
 * @brief   Header file for XXX module
 *
 *          This file contains API of XXX module
 *======================================================================================*/
/*----------------------- DEFINE TO PREVENT RECURSIVE INCLUSION ------------------------*/
#ifndef RADIO_REG_H_
#define RADIO_REG_H_

/**
 * @addtogroup XXX Description
 * @{
 * @brief Module for... .
 */

/*======================================================================================*/
/*                       ####### PREPROCESSOR DIRECTIVES #######                        */
/*======================================================================================*/
/*-------------------------------- INCLUDE DIRECTIVES ----------------------------------*/

/*----------------------------- LOCAL OBJECT-LIKE MACROS -------------------------------*/
#define INTERRUPT_STATUS1_03_REG                        0x03
#define INTERRUPT_STATUS2_04_REG                        0x04
#define INTERRUPT_ENABLE1_05_REG                        0x05
#define INTERRUPT_ENABLE2_06_REG                        0x06
#define OPERATING_AND_FNC_CTRL1_07_REG                  0x07
#define OPERATING_AND_FNC_CTRL2_08_REG                  0x08
#define CRISTAL_OSCILATOR_LOAD_CAP_09_REG               0x09
#define MICROCONTROLLER_OUTPUT_CLK_0A_REG               0x0A
#define GPIO0_CONFIGURATION_0B_REG                      0x0B
#define GPIO1_CONFIGURATION_0C_REG                      0x0C
#define GPIO2_CONFIGURATION_0D_REG                      0x0D
#define IO_PORT_CONFIGURATION_0E_REG                    0x0E
#define ADC_CONFIGURATION_0F_REG                        0x0F
#define ADC_SENSOR_AMP_OFFSET_10_REG                    0x10
#define TEMP_SENSOR_CALIB_12_REG                        0x12
#define TEMP_VALUE_OFFSET_13_REG                        0x13
#define IF_FILTER_BANDWIDTH_1C_REG                      0x1C
#define AFC_LOOP_GEARSHIFT_OVERRIDE_1D_REG              0x1D
#define AFC_TIMING_CTRL_1E_REG                          0x1E
#define CLK_RECOVERY_GEARSHIFT_OVERRIDE_1F_REG          0x1F
#define CLK_RECOVERY_OVERSAMPLING_RATE_20_REG           0x20
#define CLK_RECOVERY_OFFSET2_21_REG                     0x21
#define CLK_RECOVERY_OFFSET1_22_REG                     0x22
#define CLK_RECOVERY_OFFSET0_23_REG                     0x23
#define CLK_RECOVERY_TIMING_LOOP_GAIN1_24_REG           0x24
#define CLK_RECOVERY_TIMING_LOOP_GAIN0_25_REG           0x25
#define RESERVED_2A_REG                                 0x2A
#define RSSI_LEVEL_26_REG                               0x26
#define RSSI_THRESHOLD_FOR_CLEAR_CH_INDICATOR_27_REG    0x27
#define DATA_ACCESS_CTRL_30_REG                         0x30
#define HEADER_CTRL1_32_REG                             0x32
#define HEADER_CTRL2_33_REG                             0x33
#define PREAMBLE_LEN_34_REG                             0x34
#define PREAMBLE_DETECTION_CNTRL_35_REG                 0x35
#define SYNC_WORD3_36_REG                               0x36
#define SYNC_WORD2_37_REG                               0x37
#define SYNC_WORD1_38_REG                               0x38
#define SYNC_WORD0_39_REG                               0x39
#define HEADER3_3A_REG                                  0x3A
#define HEADER2_3B_REG                                  0x3B
#define HEADER1_3C_REG                                  0x3C
#define HEADER0_3D_REG                                  0x3D
#define TRANSMIT_PACKET_LENGTH_3E_REG                   0x3E
#define CHECK_HEADER3_3F_REG                            0x3F
#define CHECK_HEADER2_40_REG                            0x40
#define CHECK_HEADER1_41_REG                            0x41
#define CHECK_HEADER0_42_REG                            0x42
#define HEADER3_ENABLE_43_REG                           0x43
#define HEADER2_ENABLE_44_REG                           0x44
#define HEADER1_ENABLE_45_REG                           0x45
#define HEADER0_ENABLE_46_REG                           0x46
#define RECEIVED_HEADER3_47_REG                         0x47
#define RECEIVED_HEADER2_48_REG                         0x48
#define RECEIVED_HEADER1_49_REG                         0x49
#define RECEIVED_HEADER0_4A_REG                         0x4A
#define RECEIVED_PACKET_LENGTH_4B_REG                   0x4B
#define CHARGE_PUMP_CURRENT_TRIM_OVERR_58_REG           0x58
#define AGC_OVERRIDE_69_REG                             0x69
#define TX_POWER_6D_REG                                 0x6D
#define TX_DATA_RATE1_6E_REG                            0x6E
#define TX_DATA_RATE0_6F_REG                            0x6F   
#define MODULATION_MODE_CTRL2_71_REG                    0x71
#define FREQUENCY_DEVIATION_72_REG                      0x72
#define MODULATION_MODE_CTRL1_70_REG                    0x70
#define FREQ_OFFSET1_73_REG                             0x73
#define FREQ_OFFSET2_74_REG                             0x74
#define FREQ_BAND_SEL_75_REG                            0x75
#define FREQ_NOMINAL_CARRIER_FREQ1_76_REG               0x76
#define FREQ_NOMINAL_CARRIER_FREQ2_77_REG               0x77
#define FREQ_HOPPING_CH_SEL_79_REG                      0x79
#define FREQ_HOPPING_STEP_SIZE_7A_REG                   0x7A
#define FIFO_ACCESS_7F_REG                              0x7F

/*---------------------------- LOCAL FUNCTION-LIKE MACROS ------------------------------*/

/*======================================================================================*/
/*                     ####### EXPORTED TYPE DECLARATIONS #######                       */
/*======================================================================================*/
/*-------------------------------- OTHER TYPEDEFS --------------------------------------*/

/*------------------------------------- ENUMS ------------------------------------------*/
typedef enum OperatingAndFncCtrl_1_Reg_Tag
{
    XT_ON_bm                        = (1 << 0),
    PLL_ON_bm                       = (1 << 1),
    RX_ON_bm                        = (1 << 2),
    TX_ON_bm                        = (1 << 3),
    X32_KSEL_bm                     = (1 << 4),
    EN_WT                           = (1 << 5),
    EN_LBD                          = (1 << 6),
    SW_RES                          = (1 << 7)
} OperatingAndFncCtrl_1_Reg_T;

typedef enum OperatingAndFncCtrl_2_Reg_Tag
{
    FIFO_CLR_TX_bm                  = (1 << 0),
    FIFO_CLR_RX_bm                  = (1 << 1),
} OperatingAndFncCtrl_2_Reg_T;

typedef enum TxPower_Tag
{
    TXPOW                           = 0x07,
    TXPOW_4X31                      = 0x08, // Not used in RFM22B
    TXPOW_1DBM                      = 0x00,
    TXPOW_2DBM                      = 0x01,
    TXPOW_5DBM                      = 0x02,
    TXPOW_8DBM                      = 0x03,
    TXPOW_11DBM                     = 0x04,
    TXPOW_14DBM                     = 0x05, // +28dBm on RF23bp
    TXPOW_17DBM                     = 0x06, // +29dBm on RF23bp
    TXPOW_20DBM                     = 0x07 // +30dBm on RF23bp 
} TxPower_T;

typedef enum InterruptEnable_1_Reg_Tag
{
    EN_CRC_ERROR_bm                 = (1 << 0),
    EN_PK_RECEIVED_bm               = (1 << 1),
    EN_PK_SENT                      = (1 << 2),
} InterruptEnable_1_Reg_T;

typedef enum InterruptEnable_2_Reg_Tag
{
    EN_SYNC_WORD_DETECTED_bm        = (1 << 7),
    EN_VALID_PREAMBLE_DETECTED_bm   = (1 << 6),
    EN_INVALID_PREAMBLE_DETECTED_bm = (1 << 5),
    EN_RSSI_bm                      = (1 << 4),
    EN_WAKE_UP_TIMER_bm             = (1 << 3),
    EN_LOW_BATTERY_DETECT_bm        = (1 << 2),
    EN_MODULE_READY_bm              = (1 << 1),
    EN_POR_bm                       = (1 << 0)
} InterruptEnable_2_Reg_T;

typedef enum GPIO_Configuration_Reg_Tag
{
    GPIO_TX_STATE                   = 0x12,
    GPIO_RX_STATE                   = 0x15,
} GPIO_Configuration_Reg_T;

typedef enum InterruptStatus1_03_Reg_Tag
{
  FIFO_UNDERFLOW_OVERFLOW_ERROR       = (1 << 7),
  TX_FIFO_ALMOST_FULL                 = (1 << 6),
  TX_FIFO_ALMOST_EMPTY                = (1 << 5),
  RX_FIFO_ALMOST_FULL                 = (1 << 4),
  EXT_INTERRUPT                       = (1 << 3),
  PACKET_SENT_INTERRUPT               = (1 << 2),
  VALID_PACKED_RECEIVED_INTERRUPT     = (1 << 1),
  CRC_ERROR                           = (1 << 0)
} InterruptStatus1_03_Reg_T;

typedef enum InterruptStatus2_04_Reg_Tag
{
  SYNC_WORD_DETECTED_IRQ_bm           = (1 << 7),
  VALID_PREAMBLE_DETECTED_IRQ_bm      = (1 << 6),
  INVALID_PREAMBLE_DETECTED_IRQ_bm    = (1 << 5),
  RSSI_IRQ_bm                         = (1 << 4),
  WAKE_UP_TIMER_IRQ_bm                = (1 << 3),
  LOW_BATTERY_DETECT_IRQ_bm           = (1 << 2),
  MODULE_READY_IRQ_bm                 = (1 << 1),
  POR_IRQ_bm                          = (1 << 0)
} InterruptStatus2_04_Reg_T;

/*------------------------------- STRUCT AND UNIONS ------------------------------------*/

/*======================================================================================*/
/*                    ####### EXPORTED OBJECT DECLARATIONS #######                      */
/*======================================================================================*/

/*======================================================================================*/
/*                   ####### EXPORTED FUNCTIONS PROTOTYPES #######                      */
/*======================================================================================*/

/*======================================================================================*/
/*                          ####### INLINE FUNCTIONS #######                            */
/*======================================================================================*/

/**
 * @}
 */

#endif /* RADIO_REG_H_ */
