/*******************************************************************************
*  Filename:        hal_rf_proprietary.c
*  Revised:         $Date: 2013-04-25 14:23:41 +0200 (to, 25 apr 2013) $
*  Revision:        $Revision: 9917 $
*
*  Description:     CC254x proprietary mode radio interface.
*                   Compatible with CC2541, CC2543, CC2544 and CC2545.
*
*  Warning:         This hal interface does not cover all possible uses.
*
*
*  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/


/*******************************************************************************
* INCLUDES
*/
#include "hal_rf_proprietary.h"
#include "hal_timer2.h"
#include "ioCC254x_bitdef.h"


/*******************************************************************************
* CONSTANTS AND DEFINES
*/
// Various radio settings.
#define IS_2MBPS(mod) ((mod) == GFSK_2Mbps_320khz || (mod) == GFSK_2Mbps_500khz)
#define GET_MODULATION() ((MDMCTRL0 & 0x1E) >> 1)



/*******************************************************************************
* GLOBAL DATA
*/
volatile uint8 rfirqf1;


/*******************************************************************************
* LOCAL DATA
*/
static volatile enum {
    _radio_idle,
    _radio_rx,
    _radio_tx,
    _radio_rx_recalib
} _radio_mode;
static volatile uint8 _rx_avail;
static uint16 _turn_us;
static uint8 _gain;


/*******************************************************************************
* GLOBAL FUNCTIONS
*/
/*******************************************************************************
* @fn           RF_ISR
*
* @brief        Interrupt service routine for Radio. Reads the RFIRQF1 register 
*               and store the read value into the global variable rfirqf1. 
*               Clears the observed active RF interrupt flags.
*
* @param        void
*
* @return       void
*/
HAL_ISR_FUNCTION(RF_ISR,RF_VECTOR) {
    rfirqf1 = RFIRQF1;
    S1CON = 0;
    // Clear observed interrupt flags.
    RFIRQF1 = ~rfirqf1;
    return;
}


/*******************************************************************************
* @fn       halRfInit
*
* @brief    Reset Radio RAM-Based Registers, TXFIFO and RXFIFO. 
*           Also set the default output power setting.
*
* @param    void
*		
* @return   void
*/
void halRfInit(void) {
    /* Clear radio memory.
    *  The RAM registers don't have a default value set after power on */
    memset((void*)RFCORE_RAM_PAGE, 0, RFCORE_RAM_PAGE_SZ);

    // Reset RXFIFO and TXFIFO.
    halRfCommand(CMD_RXFIFO_RESET);
    halRfCommand(CMD_TXFIFO_RESET);
    
#if (chip == 2541)
    TXPOWER   = TXPOWER_0_DBM;     // Set default output power: 0dBm.
#else
    TXPOWER   = TXPOWER_4_DBM;     // Set default output power: 4dBm.
#endif
    return;
}


/*******************************************************************************
* @fn       halRfConfig
*
* @brief    Configures the radio for wanted operation.
*
* @param    uint8 modulation:
*			GFSK_250kbps_160khz 	0
*			GFSK_1Mbps_160khz		1
* 			GFSK_1Mbps_250khz		2
*			GFSK_2Mbps_320khz		3
*			GFSK_2Mbps_500khz		4
*			MSK_250kbps 			5
*			MSK_500kbps 			6
*
* @param    uint8 tx:
*           true: Configure radio for TX operations.
*           false: Configure radio for RX operations.
*
* @param    uint8 swLen:
*           Length of sync word in bits.
*           
* @param    uint8 pktConf:
*           Configuration to use.
*
* @return  void
*/
void halRfConfig(uint8 modulation, uint8 tx, uint8 swLen, uint8 pktConf) {
  
    /* RAM based registers. These are cleared in halRfInit(). */ 
    PRF.PKT_CONF.ADDR_LEN           = !!(pktConf & PKT_CONF_ADDR);
    PRF.TASK_CONF.MODE              = pktConf & PKT_CONF_MODE;
    
    PRF.TASK_CONF.REPEAT            = 0;        // No repeat.
    PRF.TASK_CONF.START_CONF        = 0;        // Start immediately after command.
    PRF.TASK_CONF.STOP_CONF         = 0;        // Don't stop on timer 2 event 2.
    PRF.FIFO_CONF.AUTOFLUSH_IGN     = 1;        // Flush duplicate packets.
    PRF.FIFO_CONF.AUTOFLUSH_CRC     = 1;        // Flush packets with CRC error.
    PRF.FIFO_CONF.AUTOFLUSH_EMPTY   = 1;        // Flush empty packets.
    PRF.FIFO_CONF.RX_STATUS_CONF    = 0;        // Don't append status information in FIFO.
    PRF.FIFO_CONF.RX_ADDR_CONF      = 0;        // No address byte in Rx FIFO.
    PRF.FIFO_CONF.TX_ADDR_CONF      = 0;        // Read address from PRF.ADDR_ENTRY[0].ADDRESS.
    
#if (chip == 2541)  
    TXCTRL    = 0x19;       // TX Settings.
    TXFILTCFG = 0x03;       // Set Tx filter bandwidth.
    IVCTRL    = 0x1B;       // Analog Control.
#else
    TXCTRL    = 0x79;       // Change DAC current step from '110' to '111'.
    ACOMPQS   = 0x16;       // Finetune Q phase relative I (i.e. skewness).
    TXFILTCFG = 0x07;       // Set Tx filter bandwidth.
#endif 
    
    // Amplitude weight in frequency offset compensation (assuming MSB first).
    FRMCTRL0 = 0x41 | (!!(pktConf & PKT_CONF_ADDR));
    // Set Modulation and data rate.
    MDMCTRL0 = (modulation << 1) | (!!(pktConf & PKT_CONF_PHASE_INV));
    // Correlation threshold.
    MDMCTRL1 = 0x40 | (swLen >> 2);
    // Set RSSI mode to peak detect after sync.
    MDMCTRL3 = 0x63;

    if (IS_2MBPS(modulation)) {
      	ADCTEST0 = 0x66;
        MDMCTRL2 = 0xCC;
        PRF.PKT_CONF.AGC_EN = 1;
        PRF.PKT_CONF.START_TONE = 1;
        if (tx) {
            MDMTEST0 = 0x01;
            PRF_RADIO_CONF = 0xD0;
        }
        else {
            MDMTEST0 = 0x0F;
            PRF_RADIO_CONF = 0x90;
        }
    }
    else {
        ADCTEST0 = 0x10;
        MDMCTRL2 = 0xC0;
        PRF.PKT_CONF.AGC_EN = 0;
        PRF.PKT_CONF.START_TONE = 0;
        MDMTEST0 = 0x01;
    }
    
    // Set gain  (will set correct gain even for 2 MBPS).
    halRfSetGain(HAL_RF_GAIN_HIGH);
    
    // Program sync word length and configure single sync word.
    SW_CONF = swLen;
    
    // Set default turnaround times ( used in auto-mode ).
    if (tx) {
        halRfSetTurnaroundTx(130);
    }
    else {
        halRfSetTurnaroundRx(130);
    }
    
    // Disable whitening by default.
    halRfSetWhitening(0);
    
    return;
}


/*******************************************************************************
* @fn           halRfPacketTxInterval
*
* @brief        Set up timer2 (radio timer) for periodic transmission of packets.
*               Uses the Overflow-Count Overflow feature on timer2.
*
* @param        uint32 us:
*               Number of us.
*
* @return       void
*/
void halRfPacketTxInterval(uint32 us) { 
    // Set up timer2.
    halTimer2Reset();
    halTimer2SetBasePeriod(TICK_VALUE_1US);
    halTimer2SetOverflowPeriod(us);

    // Enables the TIMER2_OVF_PER interrupt.
    halTimer2EnableInterrupt(T2IRQM_OVF_PERM);

    // Start each receive/transmit on Timer 2 event 1.
    PRF.TASK_CONF.START_CONF        = 1;          

    // Set timer 2 event 1 (T2_EVENT1 pulse) to overflow period and disable event 2.
    T2EVTCFG = 0x73;

    // Clear timer 2 IRQs
    T2IRQF = 0;

    // Start timer 2 (asynchronous).
    halTimer2Start(0);
    
    return;
}


/*******************************************************************************
* @fn       halRfSetRetrans
*
* @brief    Set the delay between retransmissions in micro seconds (us).
*
*           PRF.RETRANS_DELAY is given in units of 62.5 ns. The given 
*           retrans_delay_us argument is left arithmetic shifted by 4 (same as 
*           multiplying by 16) to translate into micro seconds.
*           
*           The time reference is set here:
*
*                                   Reference ->|<- Reference
* preamble byte (n) | preamble byte (n-1) | ... | last preamble byte | sync word | header | payload | CRC
* preamble byte (n) | preamble byte (n-1) | ... | last preamble byte | sync word | header | payload | CRC
*                                   Reference ->|<- Reference
*
*
* @param    uint8 num_retrans: 
*           Number of retransmissions (range: 0 to 255).
*
* @param    uint16 retrans_delay_us:
*           Delay between each retransmission given in micro seconds (us).
*		
* @return   void
*/
void halRfSetRetrans(uint8 num_retrans, uint16 retrans_delay_us) {
    uint8 modulation = GET_MODULATION();
    
    if ((modulation==GFSK_2Mbps_320khz) || (modulation==GFSK_2Mbps_500khz)) {
        PRF.RETRANS_DELAY = ((retrans_delay_us + _turn_us) << 4) - ((MDMCTRL2 & 0x0F) << 6) - 600;
    }
    else if((modulation==GFSK_1Mbps_160khz) || (modulation==GFSK_1Mbps_250khz)) {
        PRF.RETRANS_DELAY = ((retrans_delay_us + _turn_us) << 4) - ((MDMCTRL2 & 0x0F) << 7) - 612;
    }
    else if(modulation==MSK_500kbps) {
        PRF.RETRANS_DELAY = ((retrans_delay_us + _turn_us) << 4) - ((MDMCTRL2 & 0x0F) << 8) - 612;
    }
    else if((modulation==GFSK_250kbps_160khz) || (modulation==MSK_250kbps)) {
        PRF.RETRANS_DELAY = ((retrans_delay_us + _turn_us) << 4) - ((MDMCTRL2 & 0x0F) << 9) - 644;
    }
    PRF.RETRANS_CNT = num_retrans;
    return;
}  


/*******************************************************************************
* @fn       halRfSetTurnaroundRx
*
* @brief    Set the turnaround time from RX operation to TX operation (auto-mode)
*
* @param    uint16 turn_us:
*			Turnaround time in micro seconds.
*
* @return  void
*/
void halRfSetTurnaroundRx(uint16 turn_us) {
    uint8 modulation = GET_MODULATION();
    uint8 swLen = SW_CONF & SW_CONF_SW_LEN;
    if (swLen == 0) swLen = 32;
    _turn_us = turn_us;
 
    // Set turnaround times
    if((modulation==GFSK_2Mbps_320khz) || (modulation==GFSK_2Mbps_500khz)) {
        PRF.SEARCH_TIME   = 0;       // Search forever
        if (swLen >= 28) {
          PRF.RX_TX_TIME    = (_turn_us << 5) - ((MDMCTRL2 & 0x0F) << 7) + (swLen << 4) - 1882;
        }
        else if (swLen >= 20) {
          PRF.RX_TX_TIME    = (_turn_us << 5) - ((MDMCTRL2 & 0x0F) << 7) + (swLen << 4) - 1858;
        }
        else {
          PRF.RX_TX_TIME    = (_turn_us << 5) - ((MDMCTRL2 & 0x0F) << 7) + (swLen << 4) - 1831;
        }
    }
    else if((modulation==GFSK_1Mbps_160khz) || (modulation==GFSK_1Mbps_250khz)) { 
        PRF.SEARCH_TIME   = 0;       // Search forever
        if (swLen >= 28) {
          PRF.RX_TX_TIME    = (_turn_us << 5) - ((MDMCTRL2 & 0x0F) << 8) + (swLen << 5) - 2538;
        }
        else if (swLen >= 20) {
          PRF.RX_TX_TIME    = (_turn_us << 5) - ((MDMCTRL2 & 0x0F) << 8) + (swLen << 5) - 2474;
        }
        else {
          PRF.RX_TX_TIME    = (_turn_us << 5) - ((MDMCTRL2 & 0x0F) << 8) + (swLen << 5) - 2442;
        }
    }
    else if(modulation==MSK_500kbps) {
        PRF.SEARCH_TIME   = 0;       // Search forever
        if (swLen >= 28) {
          PRF.RX_TX_TIME    = (_turn_us << 5) - ((MDMCTRL2 & 0x0F) << 9) + (swLen << 6) - 3850;
        }
        else {
          PRF.RX_TX_TIME    = (_turn_us << 5) - ((MDMCTRL2 & 0x0F) << 9) + (swLen << 6) - 3626;
        }
    }
    else if((modulation==GFSK_250kbps_160khz) || (modulation==MSK_250kbps)) {
        PRF.SEARCH_TIME   = 0;       // Search forever
        if (swLen >= 20) {
          PRF.RX_TX_TIME    = (_turn_us << 5) - ((MDMCTRL2 & 0x0F) << 10) + (swLen << 6) - 4298;
        }
        else {
          PRF.RX_TX_TIME    = (_turn_us << 5) - ((MDMCTRL2 & 0x0F) << 10) + (swLen << 6) - 4778;
        }
    }
    return;
}


/*******************************************************************************
* @fn       halRfSetTurnaroundRx
*
* @brief    Set the turnaround time from TX operation to RX operation (auto-mode)
*
* @param    uint16 turn_us:
*			Turnaround time in micro seconds.
*
* @return  void
*/
void halRfSetTurnaroundTx(uint16 turn_us) {
    uint8 modulation = GET_MODULATION();
    uint8 swLen = SW_CONF & SW_CONF_SW_LEN;
    if (swLen == 0) swLen = 32;
   _turn_us = turn_us;
 
    // Set turnaround and search times
    if((modulation==GFSK_2Mbps_320khz) || (modulation==GFSK_2Mbps_500khz)) {
        PRF.SEARCH_TIME   = (swLen << 4) + 1300;
        PRF.TX_RX_TIME    = (_turn_us << 5) - 2432;    
    }
    else if((modulation==GFSK_1Mbps_160khz) || (modulation==GFSK_1Mbps_250khz)) { 
        PRF.SEARCH_TIME = (swLen << 5) + 1200;
        PRF.TX_RX_TIME    = (_turn_us << 5) - 1856;
    }
    else if(modulation==MSK_500kbps) {
        PRF.SEARCH_TIME = (swLen << 6) + 2000;
        PRF.TX_RX_TIME    = (_turn_us << 5) - 1600;
    }
    else if((modulation==GFSK_250kbps_160khz) || (modulation==MSK_250kbps)) {
        PRF.SEARCH_TIME = (swLen << 6) + 4700;
        PRF.TX_RX_TIME    = (_turn_us << 5) - 1800;
    }
    return;
}


/*******************************************************************************
* @fn          	halRfSetFrequency
*
* @brief       	Change frequency while MCU active. 
*	            LLE must be idle while changing frequency. 
*
* @return      	void
*/
void halRfSetFrequency(uint16 frequency) {
    PRF.CHAN.FREQ = frequency-2379;
    return;
}


/*******************************************************************************
* @fn       halRfSetGain
*
* @brief    Set receiver LNA gain.
*
* @param    uint8 - gain mode
*           HAL_RF_GAIN_LOW                     0
*           HAL_RF_GAIN_HIGH                    1
*
* @return   none
*/
void halRfSetGain(uint8 gainMode) {
    uint8 modulation = GET_MODULATION();
    _gain = gainMode;
    
    if (IS_2MBPS(modulation)) {
        // Always use this gain for 2 MBPS
        LNAGAIN   = 0x7F;     // LNA gain.
#if chip==2543 || chip==2545
        RXCTRL    = 0x2A;     // Receiver currents.
#elif chip==2544 || chip == 2541
        RXCTRL    = 0x29;     // Receiver currents.
#endif
        FSCTRL    = 0x5A;     // Prescaler and mixer currents.
        _gain = HAL_RF_GAIN_2MBPS;
    }
    else if (gainMode == HAL_RF_GAIN_HIGH) {
        LNAGAIN   = 0x7F;     // LNA gain.
        RXCTRL    = 0x3F;     // Receiver currents.
        FSCTRL    = 0x5A;     // Prescaler and mixer currents.
    }
    else if (gainMode == HAL_RF_GAIN_LOW) {
        LNAGAIN   = 0x3A;     // LNA gain.
        RXCTRL    = 0x33;     // Receiver currents.
        FSCTRL    = 0x55;     // Prescaler and mixer currents.
    }
    return;
}


/*******************************************************************************
* @fn          	halRfEnableRadio
*
* @brief       	Enable the link layer engine (LLE).
*	       
* @return      	void
*/
void halRfEnableRadio(void) {
    LLECTRL |= 0x01;
    return;
}


/*******************************************************************************
* @fn          	halRfDisableRadio
*
* @brief       	Hold LLE in reset.
*
* @note         The radio should be idle when setting the LLE in reset. If a 
*               Task is running one of the two semaphores might be taken. These 
*               need to be released before a new task can be started otherwise 
*               the device will fail to acquire the needed semaphore and the new 
*               task will fail to ecxecute and and with a TASKERR_SEM given in 
*               the PRF.ENDCAUSE register.
*
* @param        uint8 forceReset:
*               Force the LLE in reset
*
* @return       SUCCESS: LLE set in reset.
*               lleStat: Value read from LLESTAT, LLE not set in RESET.
*/
uint8 halRfDisableRadio(uint8 forceReset) {
    uint8 lleStat = LLESTAT;
  
    // Check if radio is idle.
    if((LLECTRL != 0) && !(lleStat & LLESTAT_LLE_IDLE)) {
        if(forceReset) {
            // End active task.
            SEND_LLE_CMD(CMD_STOP);
            
            /* 
            *  A CMD_SHUTDOWN will stop any active tasks immediately, 
            *  even aborting ongoing transmission or reception. But CMD_STOP
            *  is used here as it ends the operation more graceully, waiting 
            *  for ongoing TX/RX operations to complete. This might increase 
            *  the duration of the function to set the LLE in reset.
            */
            
            while(!(RFIRQF1 & RFIRQF1_TASKDONE));
        }
        else {
            // Radio is active, return the read LLE status (LLESTAT value might have changed).
            return lleStat;
        }
    }
  
    // LLE is idle or in reset -> Disable link layer engine.
    LLECTRL &= ~0x01;
    // Clear RF interrupts.
    RFIRQF1 = 0;
    S1CON = 0;
    // Set to a undefined value in enum type end cause.
    PRF.ENDCAUSE = TASK_UNDEF;
    return 0;//0=SUCCESS;
}


/*******************************************************************************
* @fn           halLoadAckPayload
*
* @brief        Load the ack payload into one of the two buffers for the Address 
*               Entry Number 0. An acknowledgement packet for each of the 8 
*               addressess can be placed in one of two dedicated buffer for that 
*               address. These two buffers constitute a FIFO capable of holding 
*               two packets. This functions only uses the buffer of the first 
*               address which are placed in the RAM page normally used for the 
*               hardware Tx FIFO.
*
* @param        uint8 ack_size: Size of payload in bytes.
*
* @param        uint8 entry_no:
*
* @param        uint8 *ack_payload_pointer:
*
* @return       0 on success, 9 on full buffers
*/
uint8 halRfLoadAckPayload(uint8 ack_size, uint8 entry_no, uint8 *ack_payload_pointer) {
    uint8 k, len[2];
    volatile uint8 *destptr;
  
    // Read ACK lengths
    len[0] = PRF.ADDR_ENTRY[entry_no].ACKLENGTH[0];
    len[1] = PRF.ADDR_ENTRY[entry_no].ACKLENGTH[1];
    k = PRF.ADDR_ENTRY[entry_no].SEQSTAT.NEXTACK;
    if (len[k] != 0) {
        // Buffer is not free, try the other one
        k = !k;
        if (len[k] != 0) {
            // Both FIFOs are full - exit function without entering payload. 
            return RUN_ERROR; 
        }  
    }
    if (entry_no < 2) {
        destptr = TXFIFO; // Use direct access
    }
    else {
        destptr = RFCORE_RAM_PAGE;
        RFRAMCFG = entry_no >> 1;
    }
    if (entry_no & 1) {
        destptr += 0x40;
    }
    if (k) {    
        destptr += 0x20;
    }
  
    for (int j = 0; j < ack_size; j++) {
        // Enter ACK payload
        *destptr++ = *ack_payload_pointer++;
    }
    // Set ram back to page 0
    if (entry_no >= 2) {
        RFRAMCFG = 0;
    }
    
    // Write length to address entry. 
    PRF.ADDR_ENTRY[0].ACKLENGTH[k] = ack_size; 

      // Ack. payload was loaded into one of the two buffers and will be sent on the next possible occation. 
    return RUN_SUCCESS;
}


/*******************************************************************************
* @fn          	halRfCommand
*
* @brief        The command register RFST can be used for sending commands to 
*               the LLE and the FIFOs. Commands in the range 0x80–0xFF are 
*               commands to the FIFOs. Other commands are commands to the LLE.	
*
*               If an unknown command is entered, the LLE responds by generating 
*               an LLEERR interrupt. If a task was running, it will stop.
*
* @param        uint8 command:
*               Command to be sent to the LLE
*	       
* @return      	void
*
*/
void halRfCommand(uint8 command) {
    // Make sure there is no command pending.
    while (RFST != 0);
    // Perform the command.
    RFST = command;
    return;
}


/*******************************************************************************
* @fn          	halGetLastRssi
*
* @brief       	Retrieve the RSSI value for the last received packet. The valid 
*               range is 0 to 64 where 0x80 indicate invalid/measurement not yet
*               complete.
*
* @param        void
*	       
* @return      	rssi value in dBm.
*               If RSSI value not valid (PRF_LAST_RSSI=0x80) then return 0. 
*/
int8 halRfGetLastRssi(void) {
    // Variable to store RSSI value.
    int8 rssi_val;
    // Read RSSI value for last received packet.
    rssi_val=PRF_LAST_RSSI;
    if(rssi_val == -128) {
        return -128;
    }

    if(_gain==HAL_RF_GAIN_2MBPS) {
        rssi_val -= RSSI_OFFSET_HIGH_GAIN;
    }
    else if (_gain==HAL_RF_GAIN_HIGH) {
        rssi_val -= RSSI_OFFSET_HIGH_GAIN;   
    }
    else {
       rssi_val -= RSSI_OFFSET_LOW_GAIN;
    }
    return rssi_val;
}


/*******************************************************************************
* @fn          	halRfGetRssi
*
* @brief       	Retrieve the current RSSI value on a  logarithmic scale.
*               Unit is 1 dB, offset depends on the absolute gain of the RX 
*               chain, including external components. The value given is in the
*               range 0 to approximately 64, with a change of 1 corresponding to
*               a 1 dB change. For high received signal levels, the reported 
*               RSSI saturates at one of the highest  possible reported values.
*               The highest RSSI value returned is (OFFSET + MAX RSSI) dBm 
*               (due to saturation).
*
* @Note         The receiver must be running before this function is called!
*               The RSSI is available some time after the receiver is started, 
*               regardless of whether sync is found.
*
* @param        uint8 rssi_accuracy:
*               RSSI accuracy setting (MDMTEST0_RSSI_ACC in register MDMTEST0).
*	       
* @return      	RSSI value in dBm for valid measurements. 
*               RUN_ERROR if the receiver is not running.              
*               -128 if RSSI value not valid. 
*/
int8 halRfGetRssi(uint8 rssi_accuracy) {
    // Check if receiver is running.
    if(!(RFSTAT&RFSTAT_RX_ACTIVE)) {
        // The receiver is NOT running as required. 
        return RUN_ERROR;
    }
  
    // Set RSSI accuracy.
    MDMTEST0 &= ~MDMTEST0_RSSI_ACC;
    MDMTEST0 |= (rssi_accuracy << 5);
  
    // Variable to store RSSI value.
    signed char rssi_val;

    // Read RSSI value (signed 2's complement)
    rssi_val = RSSI;
  
    // If measurement is unvalid, -128 (0x80) is returned.
    if(rssi_val == -128) {
        return -128;
    }
  
    // The reference level depends on the LNA gain setting. 
    // Subtract the correct RSSI OFFSET.
    if(_gain==HAL_RF_GAIN_2MBPS) {
        if((LLESTAT & 0x10) == 0x10) {
            // AGC algorithm has reduced the front-end gain
            rssi_val -= RSSI_OFFSET_LOW_GAIN;
        }
        else {
            rssi_val -= RSSI_OFFSET_HIGH_GAIN;
        }
    }
    else if (_gain==HAL_RF_GAIN_HIGH) {
        rssi_val -= RSSI_OFFSET_HIGH_GAIN;   
    }
    else {
        rssi_val -= RSSI_OFFSET_LOW_GAIN;
    }

    return rssi_val;
}


/*******************************************************************************
* @fn          	halRfSetRxAddress
*
* @brief       	Set Receiver address and max. packet length.
*
* @param        uint8 entry_no: entry number, total of 8 different entries.
* @param        uint8 confval: Configuration for given entry.
* @param        uint8 address: Address.
* @param        uint8 maxlen: Maximum allowed length of received packet.
*
* @return       void
*/
void halRfSetRxAddress(uint8 entry_no, uint8 confval, uint8 address, uint8 maxlen) {   
    PRF.ADDR_ENTRY[entry_no].RXLENGTH = maxlen;
    PRF.ADDR_ENTRY[entry_no].ADDRESS = address;
    if (PRF.TASK_CONF.MODE >= 2) {
        // Auto mode
        PRF.ADDR_ENTRY[entry_no].SEQSTAT.value = 0;
        PRF.ADDR_ENTRY[entry_no].ACKLENGTH[0] = 0;
        PRF.ADDR_ENTRY[entry_no].ACKLENGTH[1] = 0;
        PRF.ADDR_ENTRY[entry_no].N_TXDONE = 0;
        PRF.ADDR_ENTRY[entry_no].N_RXIGNORED = 0;
        PRF.ADDR_ENTRY[entry_no].N_RXOK = 0;
        PRF.ADDR_ENTRY[entry_no].N_RXNOK = 0;
    }
    else {
        PRF.ADDR_ENTRY[entry_no].N_RXOK = 0;
        PRF.ADDR_ENTRY[entry_no].N_RXNOK = 0;
    }
    PRF.ADDR_ENTRY[entry_no].CONF.value = confval;
    return;
}


/*******************************************************************************
* @fn          	halRfSetTxAddress
*
* @brief       	Set address used in the transmitted packet overhead.
*
* @param        uint8 address: Address.
* @param        uint8 confval: Configuration.
* @param        uint8 maxacklen: Maximum allowed length of received packet.
*	       
* @return       void
*/
void halRfSetTxAddress(uint8 address, uint8 confval, uint8 maxacklen) {   
    PRF.ADDR_ENTRY[0].ADDRESS = address;
    if (PRF.TASK_CONF.MODE >= 2) {
        // Auto mode
        PRF.ADDR_ENTRY[0].RXLENGTH = maxacklen;
        PRF.ADDR_ENTRY[0].N_TXDONE = 0;
        PRF.ADDR_ENTRY[0].N_RXIGNORED = 0;
        PRF.ADDR_ENTRY[0].N_RXOK = 0;
        PRF.ADDR_ENTRY[0].N_RXNOK = 0;
        PRF.ADDR_ENTRY[0].CONF.value = 0x18;
    }    
    else {
        PRF.ADDR_ENTRY[0].CONF.value = 0x00;
    }
    return;    
}


/*******************************************************************************
* @fn          	halRfSetCrc
*
* @brief       	Set CRC polynomial.
*
* @param        crc_conf_t crc_conf: Chosen CRC polynomial.
*	       
* @return       void
*/
void halRfSetCrc(crc_conf_t crc_conf) {
    switch (crc_conf) {
    case CRC_8_ATM:
        BSP_P0= 0x00;
        BSP_P1= 0x00;
        BSP_P2= 0x00;
        BSP_P3= 0x07;
        PRF.CRC_INIT[0]= 0x00;
        PRF.CRC_INIT[1]= 0x00;
        PRF.CRC_INIT[2]= 0x00;
        PRF.CRC_INIT[3]= 0xFF;
        PRF.CRC_LEN = 1;
        break;
    case CRC_16:
        BSP_P0= 0x00;
        BSP_P1= 0x00;
        BSP_P2= 0x05;
        BSP_P3= 0x80;
        PRF.CRC_INIT[0]= 0x00;
        PRF.CRC_INIT[1]= 0x00;
        PRF.CRC_INIT[2]= 0xFF;
        PRF.CRC_INIT[3]= 0xFF;
        PRF.CRC_LEN = 2;
        break;
    case CRC_16_CCITT:
        BSP_P0= 0x00;
        BSP_P1= 0x00;
        BSP_P2= 0x21;
        BSP_P3= 0x10;
        PRF.CRC_INIT[0]= 0x00;
        PRF.CRC_INIT[1]= 0x00;
        PRF.CRC_INIT[2]= 0xFF;
        PRF.CRC_INIT[3]= 0xFF;
        PRF.CRC_LEN = 2;
        break;
    case CRC_24:
        BSP_P0= 0x00;
        BSP_P1= 0xCB;
        BSP_P2= 0x6D;
        BSP_P3= 0x5D;
        PRF.CRC_INIT[0]= 0x00;
        PRF.CRC_INIT[1]= 0xFF;
        PRF.CRC_INIT[2]= 0xFF;
        PRF.CRC_INIT[3]= 0xFF;
        PRF.CRC_LEN = 3;
        break;
    case CRC_32:
        BSP_P0= 0xB7;
        BSP_P1= 0x1D;
        BSP_P2= 0xC1;
        BSP_P3= 0x04;
        PRF.CRC_INIT[0]= 0xFF;
        PRF.CRC_INIT[1]= 0xFF;
        PRF.CRC_INIT[2]= 0xFF;
        PRF.CRC_INIT[3]= 0xFF;
        PRF.CRC_LEN = 4;
        break;
    }
    return;
}        


/*******************************************************************************
* @fn          	halRfSetWhitening
*
* @brief       	Enable or disable whitening
*
* @param        uint8 whitening: Chosen whitening 
*
* @return       void
*/
void halRfSetWhitening(uint8 whitening) {
    if (whitening & BSP_MODE_W_PN9_EN) {
        BSP_MODE = 0x02;
        PRF.W_INIT = 0x80;
    }else if (whitening & BSP_MODE_W_PN7_EN) {
        BSP_MODE = 0x01;
        PRF.W_INIT = 0x80;
    }
    else {
        BSP_MODE = 0x00;
    }
    return;
}


/*******************************************************************************
* @fn          	halRfSetSyncWord
*
* @brief       	Set sync word.
*
* @param        uint32 sw: Primary sync word.
* @param        uint32 sw2: Secondary sync word.
*
* @return       void
*/
void halRfSetSyncWord(uint32 sw, uint32 sw2) {
    uint8 len;

    len = SW_CONF & SW_CONF_SW_LEN;
    if (len == 0) len = 32;

    // Shift sync word into most signincant bits.
    if (len < 32) {
        sw <<= (32 - len);
        sw2 <<= (32 - len);
    }
    
    // Enter primary sync word.
    SW3 = ((uint8 *) &sw)[3];
    SW2 = ((uint8 *) &sw)[2];
    if (len > 16) {
        SW1 = ((uint8 *) &sw)[1];
    }
    else {
        SW1 = 0;
    }
    if (len > 24) {
        SW0 = ((uint8 *) &sw)[0];
    }
    else {
        SW0 = 0;
    }

    // Enter secondary sync word if enabled.
    if (sw2 != 0) {
        SW7 = ((uint8 *) &sw2)[3];
        SW6 = ((uint8 *) &sw2)[2];
        if (len > 16) {
            SW5 = ((uint8 *) &sw2)[1];
        }
        else {
            SW5 = 0;
        }
        if (len > 24) {
            SW4 = ((uint8 *) &sw2)[0];
        }
        else {
            SW4 = 0;
        }
        // Configure dual sync word.
        SW_CONF |= SW_CONF_DUAL_RX;
    }   
    else {
        // Configure single sync word.
        SW_CONF &= ~SW_CONF_DUAL_RX;
    }
    return;
}


/*******************************************************************************
* @fn          	halRfStartRx
*
* @brief       	Enable/start the receiver.
*
* @param        void
*
* @return       void
*/
void halRfStartRx(void) {
    // Send Rx command
    SEND_LLE_CMD(CMD_RX);
    return;
}


/*******************************************************************************
* @fn          	halRfStopRx
*
* @brief       	disable/stop the receiver.
*
* @param        void
*
* @return       void
*/
void halRfStopRx(void) {
    // Send stop command
    SEND_LLE_CMD(CMD_STOP);
    return;
}


/*******************************************************************************
* @fn           halRfLoadPERPacketPayload
*
* @brief        Load the specific packet payload used in the CC254x packet error
*               rate into the buffer (TXFIFO).
*
* @param        uint8 packet_length
*
* @return       void
*/
void halRfLoadPERPacketPayload(uint8 packet_length, uint32 seq_packet) {
    uint8 i;
    // Enter length field in Tx FIFO (for next packet).
    RFD = packet_length;
    // Insert packet sequence number.
    RFD = (uint8) (seq_packet & 0xFF);
    RFD = (uint8) ((seq_packet >> 8) & 0xFF);
    RFD = (uint8) ((seq_packet >> 16) & 0xFF);
    RFD = (uint8) ((seq_packet >> 24) & 0xFF);

    // Enter pseudorandom payload.
    for (i=4; i<packet_length; i++) {
        // Clock the LFSR once (13× unrolling) to create pseudo-random byte.
        ADCCON1 = 0x07;
        // Read pseudo-random byte
        RFD = RNDL;
    }
    return;
}


/*******************************************************************************
* @fn           halRfPrepareTxPacket
*
* @brief        Enter payload into TXFIFO.
*
* @param        uint8 len: Packet length.
* @param        uint8 *data_ptr: Pointer to packet payload.
*
* @return       uint8: length of packet entered.
*/
uint8 halRfPrepareTxPacket(uint8 len, uint8 *data_ptr) {
    // Check that the packet can fit in the FIFO
    if (RFTXFLEN > 127 - len) {
        return 0;
    }
    // Enter length
    RFD = len;
    // Enter packet if pointer is provided. If pointer is NULL, data should be 
    // entered by writing to RFD
    if (data_ptr) {
        for (int i = 0; i < len; i++) {
            RFD = data_ptr[i];
        }
    }
    return len;
}


/*******************************************************************************
* @fn           halRfStartTx
*
* @brief        Enable/start the transmitter.
*
* @param        void
*
* @return       void
*/
void halRfStartTx(void) {
    // Send Tx command.
    SEND_LLE_CMD(CMD_TX);
    return;
}

    
/*******************************************************************************
* @fn           halRfDiscardTxPacket
*
* @brief        Discard the last entered packet currently in the TXFIFO.
*
* @param        void
*
* @return       void
*/
void halRfDiscardTxPacket(void) {
    // Avoid retransmission of last packet
    if (RFFSTATUS & TXAVAIL) {
        // Read length byte from TX FIFO
        uint8 length = RFTXFRD;
        // Remove that number of bytes from the FIFO
        RFTXFRP += length;
        // Deallocate FIFO to remove packet
        SEND_FIFO_CMD(CMD_TXFIFO_DEALLOC);
        // Increment sequence number
        PRF.ADDR_ENTRY[0].SEQSTAT.SEQ++;
    }
    return;
}


/*******************************************************************************
* @fn           halRfReadRxPacket
*
* @brief        Read the first available packet in the RXFIFO.
*
* @param        uint8 *pBuf: The byte array to store the packet in.
* @param        uint8 maxLen: The maximum packet size allowed in pBuf.
*
* @return       void
*/
uint8 halRfReadRxPacket(uint8 *pBuf, uint8 maxLen) {
    uint8 length;
    uint8 i;
    if (RFFSTATUS & RXAVAIL) {
        length = RFD;
        for (i = 0; i < length; i++) {
            uint8 byte = RFD;
            if (i < maxLen) {
                pBuf[i] = byte;
            }
        }
    }
    return length;
}


/*******************************************************************************
* @fn           halRfEnableInterrupt
*
* @brief        Enable RF interrupt source.
*
* @param        uint8 mask: The rf flag-mask to be enabled.
*
* @return       void
*/
void halRfEnableInterrupt(uint8 mask) {
    // Enable selected flags in mask.
    RFIRQM1 |= mask;
    // Enable RF interrupt
    IEN2 |= 1;
    return;
}


/*******************************************************************************
* @fn           halRfDisableInterrupt
*
* @brief        Disable RF interrupt source.
*
* @param        uint8 mask: The rf flag-mask to be disabled.
*
* @return       void
*/
void halRfDisableInterrupt(uint8 mask) {
    // Disable selected flags in mask.
    RFIRQM1 &= ~mask;
    // Disable RF interrupt.
    IEN2 &= 1;
    return;
}