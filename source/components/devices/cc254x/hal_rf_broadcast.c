/**************************************************************************************************
  Filename:     hal_rf_broadcast.c
  Revised:      $Date: 2013-03-08 10:00:00 +0100 (Fri, 08 March 2013) $
  Revision:     $Revision: 1$

  Description:  hal library for for all RF broadcast related functions.

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


/******************************************************************************
* INCLUDES
*/
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"
#include "string.h"
#include "ioCC2541.h"
#include "prop_regs.h"
#include "BLE_Broadcaster_cc254x.h"

void obtainSem0()
{
  uint8 sem;
  /*The registers SEMAPHORE0 and SEMAPHORE1 can be used to verify data integrity. These registers are
  changed to 0 when they are read. If a semaphore register is read and the value was 1, the semaphore
  has been successfully taken, and subsequent reads of the register return 0 until the semaphore is
  released. If a semaphore register is read as 0, the semaphore was not free.*/
  
  while((sem = SEMAPHORE0) != 1);
}
void releaseSem0()
{
  //A semaphore can be released by writing 1 to the semaphore register; this should only be done if the semaphore has previously been taken by the MCU.
  SEMAPHORE0 = 1;
}


void obtainSem1()
{
  uint8 sem;
  /*The registers SEMAPHORE0 and SEMAPHORE1 can be used to verify data integrity. These registers are
  changed to 0 when they are read. If a semaphore register is read and the value was 1, the semaphore
  has been successfully taken, and subsequent reads of the register return 0 until the semaphore is
  released. If a semaphore register is read as 0, the semaphore was not free.*/
  
  while((sem = SEMAPHORE1) != 1);
}
void releaseSem1()
{
  //A semaphore can be released by writing 1 to the semaphore register; this should only be done if the semaphore has previously been taken by the MCU.
  SEMAPHORE1 = 1;
}


/***********************************************************************************
* @fn          halRfBroadcastInit
*
* @brief       Initialize CC2541 radio in broadcast mode.
*
* @return      void
*/
void halRfBroadcastInit(void) 
{ 
    // Clear radio memory (The RAM registers don't have a default value set, must be set manually). 
    memset((void*)RFCORE_RAM_PAGE, 0, RFCORE_RAM_PAGE_SZ);
    halRfInit();
    
    // No timer 2 events sent to LLE.
    T2EVTCFG = 0x77;
    
    obtainSem0();
    //Sem0
    PRF.TASK_CONF.MODE              = 0;    // Basic mode, fixed length.
    PRF.TASK_CONF.REPEAT            = 0;    // no repeat.
    PRF.TASK_CONF.START_CONF        = 0;    // Start transmit immediately on command.
    PRF.TASK_CONF.STOP_CONF         = 0;    // Don't stop on timer 2 event 2.
        
    //Sem0
    PRF.PKT_CONF.ADDR_LEN           = 0;    // No address byte.
    PRF.PKT_CONF.AGC_EN             = 0;    // AGC disabled.
    PRF.PKT_CONF.START_TONE         = 0;    // No tone in front of packet.   
    releaseSem0();
    
    obtainSem1();
    //Sem1
    PRF.FIFO_CONF.TX_ADDR_CONF      = 0;    // Read address from PRF.ADDR_ENTRY[0].ADDRESS.
    PRF.FIFO_CONF.AUTOFLUSH_IGN     = 0;    // Flush duplicate packets
    PRF.FIFO_CONF.AUTOFLUSH_CRC     = 0;    // Do not Flush packets with CRC error // If the CRC is not correct and PRF_FIFO_CONF.AUTOFLUSH_CRC is 1, the LLE sends a discard RX FIFO command to remove the packet from the RX FIFO.
    PRF.FIFO_CONF.AUTOFLUSH_EMPTY   = 0;    // Flush packets with no payload
    PRF.FIFO_CONF.RX_STATUS_CONF    = 0;    // Don't append status information in FIFO
    PRF.FIFO_CONF.RX_ADDR_CONF      = 0;    // no Address byte in Rx FIFO
    
    //Sem1
    PRF.ADDR_ENTRY[0].CONF.REUSE    = 0;    // Reuse packet on same adv event (same payload on all three if active broadcast channels.)
    PRF.ADDR_ENTRY[0].CONF.ENA0 = 1;        // Necessary, according to TI to receive BLE packets.
    PRF.ADDR_ENTRY[0].CONF.VARLEN = 0;           // 0: Use fixed length given by RXLENGTH in receiver when receiving packets or ACKs
    PRF.ADDR_ENTRY[0].RXLENGTH = PAYLOAD_LENGTH + 13;        //16 //needs to be adjusted according to the packet we want to receive
    
    //Sem1
    // Set 3 byte CRC.
//    PRF_CRC_LEN = 0x03;
//    PRF_CRC_INIT[0] = 0x00;
//    PRF_CRC_INIT[1] = 0x55;
//    PRF_CRC_INIT[2] = 0x55;
//    PRF_CRC_INIT[3] = 0x55;
    //PRF_CRC_INIT[3] = 0x56; //test wrong crc
   
    PRF_CRC_LEN = 0x03;
    PRF_CRC_INIT[0] = 0x00;
    PRF_CRC_INIT[1] = 0x55;
    PRF_CRC_INIT[2] = 0x55;
    PRF_CRC_INIT[3] = 0x55;
    
    releaseSem1();
    
    
    // Update these
    TXCTRL    = 0x69;
    //https://community.silabs.com/s/article/kba-bt-0410-tx-power-limitations-for-regulatory-compliance-etsi-fcc-x?language=en_US
    //Considering regulations of FCC and ETSI, when AFH is applied and at least 15 channels are available, the maximum conducted output power, which is allowed by BLE stack, is 20 dBm on all channels except of on channel 37 and 38 (physical channels not logical channels) . The output power is limited to 18 dBm on channel 37 and 15.3 dBm on channel 38 in the case of all PHYs. There isn’t any limitation on channel 39, because the upper channel is only used for advertisements, so with the low duty cycle correction advertisements can be sent at full power.
    //0xE1 = 0dBm
    //0xE5 = 4dBm
    //0xF3 = 18dBm
    TXPOWER   = txPower;//0xF3;               // Set output power: 18 dBm.
    TXFILTCFG = 0x07;               // Set Tx filter bandwidth.
    IVCTRL    = 0x13;               // Set PA, mixer and DAC bias.
    ADCTEST0  = 0x10;               // Adjust ADC gain.
    
    FRMCTRL0  = 0x40;               // Data goes LSB over the air. 
    MDMCTRL0  = 0x04;               // Set 1 Mbps at 250 kHz deviation. 
    MDMCTRL1  = 0x48;               // Correlation threshold.
    MDMCTRL2  = 0x00;               // Syncword transmitted LSB to MSB, 1 leading preamble byte.
    
/*   MDMCTRL3
    0x23 -> 00: Correlation value above threshold is sufficient as sync criterion.
    0x63 -> 01: Correlation value above threshold and data decision on all symbols of
    sync word is used as sync criterion.
    0xA3 -> 10: Correlation value above threshold and data decision on all symbols of
    sync word is used as sync criterion. Accept one bit error in sync word
    11: Reserved*/
    //MDMCTRL3 = 0x23; 
    //MDMCTRL3  = 0x63;               // Set RSSI mode to peak detect after sync.
    MDMCTRL3  = 0xA3;
    
    MDMTEST0  = 0x01; 
     
    // Set 32 bit sync word:
    SW_CONF = 0x00; // for BLE: 4 byte of access address, which is always 0x8E89BED6 for advertizing packets.
    SW0 = 0xD6;
    SW1 = 0xBE;
    SW2 = 0x89;
    SW3 = 0x8E;
//    SW0 = 0x01;
//    SW1 = 0x23;
//    SW2 = 0x45;
//    SW3 = 0x67;
    
    
    // Enable PN7 whitener.
    BSP_P0 = 0x00;
    BSP_P1 = 0x5B;
    BSP_P2 = 0x06;
    BSP_P3 = 0x00;
    BSP_MODE = 0x01;
}


/***********************************************************************************
* @fn          	halRfBroadcastSetChannel
*
* @brief       	Change broadcast channel.
*	     
* @note         LLE must be idle while changing frequency.
*				
* @param        unsigned char AdvChannel: Advertising channel 37, 38 or 39.
*
* @return       SUCCESS:                Channel changed successfully.
*               FAIL_INVALID_PARAMETER: An illegal argument was supplied.
*               FAIL_RADIO_ACTIVE:      Radio is in TX/RX.
*/
unsigned char halRfBroadcastSetChannel(unsigned char AdvChannel) 
{
 
  obtainSem0();
  
    // Check if radio is idle or else return failure.
    if(RFSTAT & (RFSTAT_TX_ACTIVE | RFSTAT_RX_ACTIVE)) {
        // Radio is in TX/RX, exit with fail.
        return FAIL_RADIO_ACTIVE;
    }
    
  /* Initialization of PN7 whitener in accordance with advertising channel.
   * PRF_W_INIT should be set to 37, 38, or 39 (0x25, 0x26, or 0x27).
   */
    switch(AdvChannel) {
    case BLE_BROADCAST_CHANNEL_37 :
        PRF.CHAN.FREQ = 23;                 // Set frequency to 2402 MHz.
        PRF_W_INIT = 0x25;                  // Init according to BLE channel 37.
        break;
    case BLE_BROADCAST_CHANNEL_38 :
        PRF.CHAN.FREQ = 47;                 // Set frequency to 2426 MHz.
        PRF_W_INIT = 0x26;                  // Init according to BLE channel 37.
        break;
    case BLE_BROADCAST_CHANNEL_39 :
        PRF.CHAN.FREQ = 101;                // Set frequency to 2480 MHz.
        PRF_W_INIT = 0x27;                  // Init according to BLE channel 37.
        break;
    default :
        PRF.CHAN.FREQ = AdvChannel;                 // Set frequency to 2402 MHz.
        PRF_W_INIT = 0x25;                  // Init according to BLE channel 37.
        //releaseSem0();
        // Illegal channel in argument.
        //return FAIL_INVALID_PARAMETER;
        break;
    }
    releaseSem0();
    return 0; //0=SUCCESS;
}


/***********************************************************************************
* @fn          halRfLoadBleBroadcastPacket
*
* @brief       Load a ADV_NONCONN_IND BLE broadcast packet.
*              Hard coded public address: 0x112233445566
*
* @param       unsigned char *AdvData: Advertising data.
* @param       unsigned char AdvDataLength: Length of advertising data. 
*
* @return      SUCCESS: Packet loaded in to TXFIFO without errors.
*              FAIL_LENGTH: AdvData length is above maximum limit (26 bytes). 
*/
unsigned char halRfBroadcastLoadPacket(unsigned char *AdvData, unsigned char AdvDataLength, unsigned char *Address) {

    /* Local variables */
    unsigned char i = 0;  
  
    /* Check for maximum allowed payload size (AdvData allowed size: 0-31 bytes) */
    if(AdvDataLength > 26) {
      return FAIL_LENGTH;
    }
  
    /* BLE header and TXFIFO length paramter (required) */
    RFD = AdvDataLength + 13;       // FIFO entry length (Payload + BLE header + BLE length byte), (not transmitted).
    
    //Before transmitting the data below, the preamble and address (SYNC for the prop. protocol) are transmitted automatically. 
    //Then, the Header of BLE and the total length of the remaining data (minus the CRC, which is transmitted automatically) are transmitted
    //Finally, the CRC is transmitted automatically.
    RFD = 0x02;                     // BLE header ADV_NONCONN_IND PDU type (transmitted).
    RFD = AdvDataLength + 11;       // BLE length byte (transmitted).
  
    /* BLE: Advertiser’s public or random device address (required) */
    RFD = Address[0]; // Address (LSB)
    RFD = Address[1];
    RFD = Address[2];
    RFD = Address[3];
    RFD = Address[4];
    RFD = Address[5]; // Address (MSB)  
    
    /* AdvData/payload (required) */

    // BLE: Flags.
    RFD = 0x02;     // Length of next Data (3  Byte).
    RFD = 0x01;     // Type flag.
    RFD = 0x04;     // BR/EDR Not Supported.

    // BLE: Manufacturer Specific Data
    RFD = AdvDataLength + 1;    // Length of next Data.
    RFD = 0xFF;                 // Type "Manufacturer Specific Data"
    
    /* AdvData/payload (optional) */
    for (i = 0; i < AdvDataLength; i++) {
        RFD = AdvData[i];
    }
    
    /* Packet successfully loaded in to TXFIFO. */
    return 0;//0=SUCCESS;
}


/***********************************************************************************
* @fn           halRfChannelMap
*
* @brief        Active or suspend broadcast channels. The three
*               RF Channel  0,  2402 MHz, Advertising channel 37
*               RF Channel 12,  2426 MHz, Advertising channel 38
*               RF Channel 39,  2480 MHz, Advertising channel 39
*
* @param        unsigned char AdvChan37: Set value bigger than 0 to activate broadcast channel 37.
* @param        unsigned char AdvChan38: Set value bigger than 0 to activate broadcast channel 38.
* @param        unsigned char AdvChan39: Set value bigger than 0 to activate broadcast channel 39.
*
* @return       unsigned char:
*               Single byte returned where the three least significant bits (LSb) indicate active (1) or suspend (0) channel 37, 38, 39.
*/
unsigned char halRfBroadcastChannelMap(unsigned char AdvChan37,unsigned char AdvChan38,unsigned char AdvChan39) {
    unsigned char ActiveChannels = 0;
    if(AdvChan37) {
        ActiveChannels |= 0x01;
    }
    if(AdvChan38) {
        ActiveChannels |= 0x02;
    }
    if(AdvChan39) {
        ActiveChannels |= 0x04;
    }
    return ActiveChannels;
}