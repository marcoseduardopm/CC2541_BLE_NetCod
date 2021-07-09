/**************************************************************************************************
  Filename:     hal_rf_broadcast.h
  Revised:      $Date: 2013-03-08 10:00:00 +0100 (Fri, 08 March 2013) $
  Revision:     $Revision: 1$

  Description:  

  Copyright 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef HAL_RF_BLE_BROADCAST_H
#define HAL_RF_BLE_BROADCAST_H

/******************************************************************************
* DEFINES
*/

// Function Return Values.
//#define SUCCESS                     0
#define FAIL                        1
#define FAIL_INVALID_PARAMETER      2
#define FAIL_LENGTH                 3
#define FAIL_RADIO_ACTIVE           4

// BLE broadcast channels.
#define BLE_BROADCAST_CHANNEL_37 23     // 2402 MHz
#define BLE_BROADCAST_CHANNEL_38 47     // 2426 MHz
#define BLE_BROADCAST_CHANNEL_39 101    // 2480 MHz

// Output Power Settings for CC2541.
#define TXPOWER_0_DBM           0xE1
#define TXPOWER_MINUS_2_DBM     0xD1
#define TXPOWER_MINUS_4_DBM     0xC1
#define TXPOWER_MINUS_6_DBM     0xB1
#define TXPOWER_MINUS_8_DBM     0xA1
#define TXPOWER_MINUS_10_DBM    0x91
#define TXPOWER_MINUS_12_DBM    0x81
#define TXPOWER_MINUS_14_DBM    0x71
#define TXPOWER_MINUS_16_DBM    0x61
#define TXPOWER_MINUS_18_DBM    0x51
#define TXPOWER_MINUS_20_DBM    0x41

// Bitmask for radio register RFIRQF1.
#define RFIRQF1_PINGRSP             0x80
#define RFIRQF1_TASKDONE            0x40
#define RFIRQF1_TXDONE              0x20
#define RFIRQF1_RXEMPTY             0x10
#define RFIRQF1_RXIGNORED           0x08
#define RFIRQF1_RXNOK               0x04
#define RFIRQF1_TXFLUSHED           0x02
#define RFIRQF1_RXOK                0x01

// Bitmask for radio register RFSTAT.
#define RFSTAT_MOD_UNDERFLOW        0x80
#define RFSTAT_DEM_STATUS           0x60
#define RFSTAT_SFD                  0x10
#define RFSTAT_CAL_RUNNING          0x08
#define RFSTAT_LOCK_STATUS          0x04
#define RFSTAT_TX_ACTIVE            0x02
#define RFSTAT_RX_ACTIVE            0x01

// Bitmask for radio register LLESTAT.
#define LLESTAT_AGC_LOWGAIN         0x10
#define LLESTAT_WAIT_T2E1           0x08
#define LLESTAT_LLE_IDLE            0x04
#define LLESTAT_SYNC_SEARCH         0x02
#define LLESTAT_VCO_ON              0x01

// Bitmask for radio register SW_CONF
#define SW_CONF_DUAL_RX     0x80
#define SW_CONF_SW_RX       0x20
#define SW_CONF_SW_LEN      0x1F

// Bitmask for radio register MDMCTRL1
#define MDMCTRL1_FOC_MODE   0xC0
#define MDMCTRL1_CORR_THR   0x1F

// Bitmask for radio register RFFSTATUS
#define TXAVAIL             0x80
#define TXFEMPTY            0x40
#define TXDTHEX             0x20
#define TXFFULL             0x10
#define RXAVAIL             0x08
#define RXFEMPTY            0x04
#define RXDTHEX             0x02
#define RXFFULL             0x01

// RFIRQM1 (0x6182) – RF Interrupt Masks
#define RFIRQM1_PINGRSP                 0x80
#define RFIRQM1_TASKDONE                0x40
#define RFIRQM1_TXDONE                  0x20
#define RFIRQM1_RXEMPTY                 0x10
#define RFIRQM1_RXIGNORED               0x08
#define RFIRQM1_RXNOK                   0x04
#define RFIRQM1_TXFLUSHED               0x02
#define RFIRQM1_RXOK                    0x01

// IP0 (0xA9) – Interrupt Priority 0
#define IP0_ST_P0INT_WDT                0x20
#define IP0_ENC_T4_P1INT                0x10
#define IP0_T3_I2C                      0x08
#define IP0_URX0_T2_UTX0                0x04
#define IP0_ADC_T1_P2INT_USB            0x02
#define IP0_RFERR_RF_DMA                0x01

// IP1 (0xB9) – Interrupt Priority 0
#define IP1_ST_P0INT_WDT                0x20
#define IP1_ENC_T4_P1INT                0x10
#define IP1_T3_I2C                      0x08
#define IP1_URX0_T2_UTX0                0x04
#define IP1_ADC_T1_P2INT_USB            0x02
#define IP1_RFERR_RF_DMA                0x01

// LLECTRL (0x61B1) – LLE Control
#define LLECTRL_LLE_MODE_SEL            0x06
#define LLECTRL_LLE_EN                  0x01


/***********************************************************************************
* GLOBAL FUNCTIONS
*/


void obtainSem0();
void releaseSem0();
void obtainSem1();
void releaseSem1();

void halRfBroadcastInit(void);
unsigned char halRfBroadcastSetChannel(unsigned char AdvChannel);
unsigned char halRfBroadcastLoadPacket(unsigned char *AdvData, unsigned char AdvDataLength, unsigned char *Address);
unsigned char halRfBroadcastChannelMap(unsigned char AdvChan37,unsigned char AdvChan38,unsigned char AdvChan39);
#endif