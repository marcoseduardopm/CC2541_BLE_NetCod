/***********************************************************************************

  Filename:       hal_rf.c

  Description:    CC254x proprietary mode radio interface. Adapted for use
                  with BasicRF.

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "hal_rf.h"
#include "hal_defs.h"
#include "hal_int.h"
#include "string.h"
#include "prop_regs.h"
#include "util.h"

/***********************************************************************************
* CONSTANTS AND DEFINES
*/

#define RSSI_OFFSET                 98

// Power indices
#define HAL_RF_TXPOWER_0_DBM        0
#define HAL_RF_TXPOWER_MIN_6_DBM    1
#define HAL_RF_TXPOWER_MIN_20_DBM   2

// TXPOWER values
#if (chip == 2541)

#define TXPOWER_0_DBM               0xE1
#define TXPOWER_MINUS_6_DBM         0xB1
#define TXPOWER_MINUS_20_DBM        0x41

#else

#define TXPOWER_0_DBM               0xA5
#define TXPOWER_MINUS_6_DBM         0x65
#define TXPOWER_MINUS_20_DBM        0x05

#endif


#if (chip == 2543) || (chip == 2545)

// Uncalibrated accuracy of 4 dB. ( Valid range: 0 to 64).
#define RSSI_OFFSET_LOW_GAIN        79

#elif (chip == 2544)

// Uncalibrated accuracy of +/- 6 dB. ( Valid range: 0 to 60).
#define RSSI_OFFSET_LOW_GAIN        77

#elif (chip == 2541)

#define RSSI_OFFSET_LOW_GAIN        98

#endif

// Bitmask for radio register RFIRQF1.
#define RFIRQF1_PINGRSP             0x80
#define RFIRQF1_TASKDONE            0x40
#define RFIRQF1_TXDONE              0x20
#define RFIRQF1_RXEMPTY             0x10
#define RFIRQF1_RXIGNORED           0x08
#define RFIRQF1_RXNOK               0x04
#define RFIRQF1_TXFLUSHED           0x02
#define RFIRQF1_RXOK                0x01

// Send a command to link layer
#define SEND_LLE_CMD(cmd)           st(while(RFST != 0); RFST = (cmd);)

/***********************************************************************************
* GLOBAL DATA
*/
static const menuItem_t pPowerSettings[] =
{
  "0dBm",      HAL_RF_TXPOWER_0_DBM,
  "-6dBm",     HAL_RF_TXPOWER_MIN_6_DBM,
  "-20dBm",    HAL_RF_TXPOWER_MIN_20_DBM
};

const menu_t powerMenu =
{
  pPowerSettings,
  N_ITEMS(pPowerSettings)
};

/***********************************************************************************
* LOCAL DATA
*/

static ISR_FUNC_PTR pfISR= NULL;
static uint8 pBuf[128];
static uint8 iRead;

/***********************************************************************************
* LOCAL FUNCTIONS
*/


/***********************************************************************************
* GLOBAL FUNCTIONS
*/

extern void halRfSetRadioFrequency(uint16 frequency);
extern void halRfInitRegs(void);


/***********************************************************************************
* @fn          	rfInit
*
* @brief       	Initialize radio.
*				
* @return      	none
*/
uint8 halRfInit(void)
{
  // Clear radio memory (The RAM registers don't have a default value set, must be set manually).
  memset((void*)RFCORE_RAM_PAGE, 0, RFCORE_RAM_PAGE_SZ);
  iRead = 0;

  // Write register settings
  halRfInitRegs();

  // Enable link layer engine
  LLECTRL = 0x01;

  // Make sure RX interrupts are enabled
  halRfEnableRxInterrupt();

  return SUCCESS;
}


/***********************************************************************************
* @fn      halRfGetChipId
*
* @brief   Get chip id
*
* @param   none
*
* @return  uint16 - result
*/
uint16 halRfGetChipId(void)
{
  return CHIPID;
}


/***********************************************************************************
* @fn      halRfGetChipVer
*
* @brief   Get chip version
*
* @param   none
*
* @return  uint8 - result
*/
uint8 halRfGetChipVer(void)
{
  return CHVER;
}


/***********************************************************************************
* @fn      halRfGetRandomByte
*
* @brief   Return random byte
*
* @param   none
*
* @return  uint8 - random byte
*/
uint8 halRfGetRandomByte(void)
{
  // Not yet implemented
  return 0x3F;
}


/***********************************************************************************
* @fn      halRfGetRssiOffset
*
* @brief   Return RSSI Offset
*
* @param   none
*
* @return  uint8 - RSSI offset
*/
uint8 halRfGetRssiOffset(void)
{
  return RSSI_OFFSET;
}


/***********************************************************************************
* @fn      halRfSetChannel
*
* @brief   Set RF channel in the 2.4GHz band. The Channel must be in the range 11-26,
*          11= 2405 MHz, channel spacing 5 MHz.
*
* @param   channel - logical channel number
*
* @return  none
*/
void halRfSetChannel(uint8 channel)
{
  uint16 freq;

  // Compatible with 802.15.4
  freq = 2405 + 5*(channel-11);
  halRfSetRadioFrequency(freq);
}


/***********************************************************************************
* @fn      halRfSetShortAddr
*
* @brief   Write short address to chip (stub: not applicable to this device)
*
* @param   none
*
* @return  none
*/
void halRfSetShortAddr(uint16 shortAddr)
{
  // Not implemented
}


/***********************************************************************************
* @fn      halRfSetPanId
*
* @brief   Write PAN Id to chip (stub: not applicable to this device)
*
* @param   none
*
* @return  none
*/
void halRfSetPanId(uint16 panId)
{
  // Not implemented
}


/***********************************************************************************
* @fn      halRfReadRxBuf
*
* @brief   Read RX buffer
*
* @param   uint8* pData - data buffer. This must be allocated by caller.
*          uint8 n - number of bytes
*
* @return  none
*/
void halRfReadRxBuf(uint8* pData, uint8 n)
{
  // Read data
  while (n-->0)
  {
    *pData++= pBuf[iRead++];
  }
}


/***********************************************************************************
* @fn      halRfSetPower
*
* @brief   Set TX output power
*
* @param   uint8 power - power level CC254X: 0, -6, -20 dBm
*
* @return  uint8 - SUCCESS or FAILED
*/
uint8 halRfSetTxPower(uint8 power)
{
  uint8 n;

  switch(power)
  {
  case HAL_RF_TXPOWER_MIN_20_DBM:
    n = TXPOWER_MINUS_20_DBM;
    break;
  case HAL_RF_TXPOWER_MIN_6_DBM:
    n = TXPOWER_MINUS_6_DBM;
    break;
  case HAL_RF_TXPOWER_0_DBM:
    n = TXPOWER_0_DBM;
    break;
  default:
    return FAILED;
  }

  // Set TX power
  TXPOWER = n;

  return SUCCESS;
}


/***********************************************************************************
* @fn      halRfWriteTxBuf
*
* @brief   Write data to the TX fifo
*
* @param   none
*
* @return  none
*/
void halRfWriteTxBuf(uint8 *data_ptr, uint8 len)
{
  uint8 i;

  data_ptr[0]= len  - 1;
  for (i = 0; i < len; i++)
  {
    RFD = data_ptr[i];
  }
}


/***********************************************************************************
* @fn      halRfTransmit
*
* @brief   Transmit frame.
*
* @param   none
*
* @return  uint8 - SUCCESS
*/
uint8 halRfTransmit(void)
{
  SEND_LLE_CMD(CMD_TX);

  return SUCCESS;
}


/***********************************************************************************
* @fn      halRfReceiveOn
*
* @brief   Turn receiver on
*
* @param   none
*
* @return  none
*/
void halRfReceiveOn(void)
{
  SEND_LLE_CMD(CMD_RX);
}


/***********************************************************************************
* @fn      halRfReceiveOff
*
* @brief   Turn receiver off
*
* @param   none
*
* @return  none
*/
void halRfReceiveOff(void)
{
  SEND_LLE_CMD(CMD_STOP);
}


/***********************************************************************************
* @fn      halRfDisableRxInterrupt
*
* @brief   Clear and disable RX interrupt.
*
* @param   none
*
* @return  none
*/
void halRfDisableRxInterrupt(void)
{
  // Disable RX interrupts
  RFIRQM1 &= ~RFIRQF1_RXOK;
  IEN2 &= ~1; // RFIE = 0
}


/***********************************************************************************
* @fn      halRfEnableRxInterrupt
*
* @brief   Enable RX interrupt.
*
* @param   none
*
* @return  none
*/
void halRfEnableRxInterrupt(void)
{
  // Enable RX interrupts
  RFIRQM1 |= RFIRQF1_RXOK;
  IEN2 |= 1; // RFIE = 1
}


/***********************************************************************************
* @fn      halRfRxInterruptConfig
*
* @brief   Configure RX interrupt.
*
* @param   none
*
* @return  none
*/
void halRfRxInterruptConfig(ISR_FUNC_PTR pf)
{
  uint8 x;

  HAL_INT_LOCK(x);
  pfISR= pf;
  HAL_INT_UNLOCK(x);
}


/***********************************************************************************
* @fn      halRfWaitTransceiverReady
*
* @brief   Wait until the transceiver is ready (SFD inactive).
*
* @param   none
*
* @return  none
*/
void halRfWaitTransceiverReady(void)
{
}


/***********************************************************************************
* @fn      rfIsr
*
* @brief   Interrupt service routine for radio
*
* @param   none
*
* @return  none
*/
HAL_ISR_FUNCTION(rfIsr,RF_VECTOR)
{
  uint8 rfirqf1;

  rfirqf1 = RFIRQF1;

  // Clear observed interrupt flags
  RFIRQF1 = ~rfirqf1;
  S1CON = 0;

  if (rfirqf1 & RFIRQF1_RXOK)
  {
    uint8 n, i;

    i = 0;
    n= RFD  + 2;  // Compatible with 802.15.4
    pBuf[i++] = n;
    while(i <= n )
    {
      pBuf[i++]= RFD;
    }

    // Compatible with 802.15.4
    pBuf[n] ^= 0x80;
    iRead = 0;

    if(pfISR)
    {
      (*pfISR)();         // Execute the custom packet handler
    }

    // Restart Rx
    SEND_LLE_CMD(CMD_RX);
  }
}

/***********************************************************************************
Copyright 2012 Texas Instruments Incorporated. All rights reserved.

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
***********************************************************************************/