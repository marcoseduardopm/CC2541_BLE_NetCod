/*******************************************************************************
*  Filename:        per_test_cc254x.c
*  Revised:         $Date: 2013-04-29 13:31:25 +0200 (ma, 29 apr 2013) $
*  Revision:        $Revision: 9926 $
*
*  Description:     Packet Error Rate (PER) test for the CC2541EM, CC2543EM, 
*                   CC2544Dongle and the CC2545EM.
*
*  note             This code is made for range evaluation as well as for use as 
*                   example code showing how to use the radio. The radio can be 
*                   utilized in more ways than what is used in this example code.
*
*  warning          This program is intended for use between one single master 
*                   device and one single slave. Make sure not to have multiple 
*                   Master devices powered up while performing test!
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
// Include device specific files
#if (chip==2541)
#include "ioCC2541.h"
#elif (chip==2543)
#include "ioCC2543.h"
#elif (chip==2544)
#include "ioCC2544.h"
#elif (chip==2545)
#include "ioCC2545.h"
#else
#error "Chip not supported!"
#endif
#include "per_test_cc254x.h"
#include "prop_regs.h"
#include "hal_timer2.h"
#include "hal_sleep.h"
#include "hal_rf_proprietary.h"
#include "hal_int.h"
#include "hal_board.h"
#include "hal_button.h"
#include "hal_led.h"

// Include files for EMs used on the SmartRF05EB.
#if (chip==2541 || chip==2543 || chip==2545)
#include "per_test_cc254x_strings.h"
#include "per_test_cc254x_query.h"
#include "hal_joystick.h"
#include "util_lcd.h"
#endif


/*******************************************************************************
* GLOBAL VARIABLES
*/
// Variables used for packet count.
unsigned long __xdata packets;                

// Char arrays used to store packet payload for acknowledgements.
unsigned char __xdata ackConfig[9];
unsigned char __xdata ack_command[2];

// Global flags.
extern volatile uint8 rfirqf1;
extern uint8 RadioTimeoutFlag;

#if(POWER_SAVING)
uint8 powerModeFlag;
#endif

/*******************************************************************************
* LOCAL FUNCTIONS
*/


/*******************************************************************************
* @fn           masterBeaconMode
*
* @brief        Mode where master device sends a beacon packet every 10 ms and 
*               wait for an acknowledgement packet from a REMOTE device which 
*               contains the test configuration.
*
* @param        void
*
* @return       unsigned char:
*               RUN_SUCCESS:    If all went well.
*               RUN_ERROR:      Exited with error.
*
*/
unsigned char masterBeaconMode(void) {
    // Variables for packet sequence, statistics, etc...
    uint32 packetSeq = 0, sentPackets = 0;
    uint8 ackLength;

#if chip==2541 || chip==2543 || chip==2545
    // Clear and update LCD display.
    halLcdClear();
    halLcdWriteLine(1, c2xd(text_Master_Mode));
    halLcdWriteLine(2, c2xd(text_beacon));
#endif
  
    // Enable radio in default beacon mode.
    halRfDisableRadio(FORCE);
    halRfInit();
    halRfConfig(GFSK_250kbps_160khz, TX, 32, PKT_CONF_NRF);
    halRfSetCrc(CRC_16_CCITT);
    halRfSetSyncWord(SYNCWORD, 0);
    halRfSetTxAddress(0xEF, ADDR_CONF_TX_AUTO, 16);
    PRF.ADDR_ENTRY[0].CONF.REUSE = 1;
    halRfSetFrequency(2402);
    halRfEnableRadio();
    
    // Seed pseudo-random generator.
    RNDL = 0x00;
    RNDL = 0xFF;

    // Enable RF interrupt.
    halRfEnableInterrupt(RFIRQF1_TASKDONE);

    // Enable global interrupt.
    halIntOn();

    // Load packet payload.
    halRfLoadPERPacketPayload(LENGTH, 0);
    
    // Set radio to transmit packets every 10 ms.
    halRfPacketTxInterval(VALUE_10MS_IN_1US_UNITS);

    while(1) {
        // Start transmitter.
        halRfStartTx();

        // Wait for TASKDONE and halt CPU (PM0) until task is completed.
        while (!(rfirqf1 & RFIRQF1_TASKDONE)) {
#if(POWER_SAVING)
            halSleepEnterPowerMode(CPU_HALT);
#endif
        }

        // If end cause was TASK_ENDOK then continue as master.
        if(PRF.ENDCAUSE == TASK_ENDOK) {
            // Check if ACK with config payload was received.
            if(rfirqf1 & RFIRQF1_RXOK) {
                if((ackLength = RFD) == 8) {
                    // Read out the test configuration from the RXFIFO.  
                    ackConfig[0] = ackLength;  // Read configuration byte array size. 
                    ackConfig[1] = RFD;        // Read channel setting (frequency).
                    ackConfig[2] = RFD;        // Read modulation setting.
                    ackConfig[3] = RFD;        // Read packet length (pktLen).
                    ackConfig[4] = RFD;        // Read byte 1 (LSB) for number of packets (packets).
                    ackConfig[5] = RFD;        // Read byte 2 for number of packets (packets). 
                    ackConfig[6] = RFD;        // Read byte 3 for number of packets (packets).
                    ackConfig[7] = RFD;        // Read byte 4 (MSB) for number of packets (packets).
                    ackConfig[8] = RFD;        // Read LNA gain (gain). 

                    // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
                    rfirqf1 = 0;

                    // Disables the TIMER2_OVF_PER interrupt.
                    halTimer2DisableInterrupt(T2IRQM_OVF_PERM);

                    // Enable global interrupt.
                    halIntOff();

                    return RUN_SUCCESS;
                }
                // Packet size mismatch, reset and continue in beacon mode.
                halRfCommand(CMD_RXFIFO_RESET);
                halRfLoadPERPacketPayload(LENGTH, ++packetSeq);
            }
        }
        // Check if acknowledge is not received.
        else if(PRF.ENDCAUSE == TASK_MAXRT) {
            sentPackets++;
        }
        else {
#if chip==2541 || chip==2543 || chip==2545
            // Write end cause error code to screen.
            printErrorScreen((unsigned char)PRF.ENDCAUSE);
#elif chip==2544
            HAL_LED_SET_1();
            HAL_LED_CLR_2();
#endif
            halMcuWaitMs(1000);
            return RUN_ERROR;
        }

        // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
        rfirqf1 = 0;
        // Set end cause to a undefined value.
        PRF.ENDCAUSE = TASK_UNDEF;

#if chip==2541 || chip==2543 || chip==2545
        // Toggle LED3 for every 64 sent packets.
        if(!(sentPackets % 64)) {
            halLedToggle(1);
            halLcdWriteLine(2, c2xd(text_beacon));
        }
        if(!(sentPackets % 128)) {
            halLcdWriteLine(2, c2xd(text_blank));
        }
#elif chip==2544
        // Toggle green LED2 for every 64 sent packets.
        if(!(sentPackets % 64)) {
            HAL_LED_TGL_2();
        }
#endif
    }
}


/*******************************************************************************
* @fn           masterTxMode
*
* @brief        Mode where Master device transmit all the packets for the Packet
*               Error Rate test at an interval of 10 ms.
*
* @param        void  
*
* @return       unsigned char:
*               RESTART_TEST:   If the user wants to restart the test.
*               START_NEW_TEST: If the user wants to define new test.
*               TEST_ERROR:     Unknown error. -> restart.
*/
unsigned char masterTxMode(void) {  
    // Variables for packet sequence, statistics, etc...
    uint32 packetSeq = 0, sentPackets = 0, tmp;
    uint8 pktLen, returnval = 0;

#if chip==2541 || chip==2543 || chip==2545
    // Clear and update LCD display.
    halLcdClear();  
    halLcdWriteLine(1, c2xd(text_Master_Mode));
#endif
  
    // Get test configuration from packet payload.
    pktLen = ackConfig[3];
    packets = ackConfig[4];
    packets |= (((long)ackConfig[5] << 8) & 0xFF00);
    packets |= (((long)ackConfig[6] << 16) & 0xFF0000);
    packets |= (((long)ackConfig[7] << 24) & 0xFF000000);

    // Set up radio in per test mode with the received configuration.
    halRfDisableRadio(FORCE);
    halRfInit();
    halRfConfig(ackConfig[2], TX, 32, PKT_CONF_NRF);
    halRfSetFrequency(ackConfig[1]);
    halRfSetCrc(CRC_16_CCITT);
    halRfSetSyncWord(SYNCWORD, 0);
    halRfSetTxAddress(0xEF, ADDR_CONF_TX_AUTO, 16);
    halRfCommand(CMD_TXFIFO_RESET);
    halRfEnableRadio();
  
    // 200 ms delay.
    halMcuWaitMs(200);

    // Set radio to transmit packets every 10 ms.
    halRfPacketTxInterval(VALUE_10MS_IN_1US_UNITS);

    // Enable RF interrupt.
    halRfEnableInterrupt(RFIRQF1_TASKDONE);

    // Enable global interrupt.
    halIntOn();

    while(sentPackets<packets) {      
        // Load payload.
        halRfLoadPERPacketPayload(pktLen, ++packetSeq);
        // Start transmitter
        halRfStartTx();          
    
        // Wait for TASKDONE and halt CPU (PM0) until task is completed.
        while (!(rfirqf1 & RFIRQF1_TASKDONE)) {
#if(POWER_SAVING)
            halSleepEnterPowerMode(CPU_HALT);
#endif
        }

        // Check if packet was received     
        if(PRF.ENDCAUSE == TASK_ENDOK) {
            if ((rfirqf1 & RFIRQF1_RXEMPTY)) {                
                sentPackets++;
#if chip==2541 || chip==2543 || chip==2545
                printTxtNuTxt(text_Pckts, sentPackets, text_blank,  2);
#endif
            }
            // Check if ACK with config payload was received.   
            if((rfirqf1 & RFIRQF1_RXOK) == 1) {
                // Check for commands from REMOTE device.
                tmp=RFD;
                if(tmp==1) {
                    tmp=RFD;
                    if(tmp==ACK_COMMAND_RESTART) {
                        returnval=ACK_COMMAND_RESTART;
                    }
                    else if(tmp==ACK_COMMAND_STOP) {
                        returnval=ACK_COMMAND_STOP;            
                    }
                    // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
                    rfirqf1 = 0;                              
                    // Set to a undefined value in enum type end cause.
                    PRF.ENDCAUSE = TASK_UNDEF;
                    // Reset LLE.
                    halRfDisableRadio(FORCE);
                    break;
                }
                halRfCommand(CMD_RXFIFO_RESET);
            }
#if chip==2541 || chip==2543 || chip==2545
            // Check if ACK without payload was received.    
            else if (rfirqf1 & RFIRQF1_RXEMPTY) {                
                // Remote is alive, update status on LCD. 
                halLcdWriteLine(3, c2xd(text_remote_online));
            }
#endif
        }
        // Received no ACK.     
        else if(PRF.ENDCAUSE == TASK_MAXRT) {    
            // Increase packet count. 
            sentPackets++;
#if chip==2541 || chip==2543 || chip==2545
            // Update LCD.
            printTxtNuTxt(text_Pckts, sentPackets, text_blank, 2);
            halLcdWriteLine(3, c2xd(text_remote_offline));

            // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
            rfirqf1 = 0;
#endif
        }
        else {
#if chip==2541 || chip==2543 || chip==2545
            // Write end cause error code to screen.
            printErrorScreen((unsigned char)PRF.ENDCAUSE);            
#elif chip==2544 
            HAL_LED_SET_1();
            HAL_LED_CLR_2();
#endif
            halMcuWaitMs(1000);
            return RUN_ERROR;
        }

        // Reset TXFIFO to clear any remaining data in buffer.
        halRfCommand(CMD_TXFIFO_RESET);
        // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
        rfirqf1 = 0;
        // Set to a undefined value in enum type end cause.
        PRF.ENDCAUSE = TASK_UNDEF;

        // Toggle LED3 for every other sent packets.
        if(sentPackets & 0x01) {
#if chip==2541 || chip==2543 || chip==2545
            halLedToggle(1);    
#elif chip==2544 
            HAL_LED_TGL_2();
#endif      
        }
    }
  
#if chip==2541 || chip==2543 || chip==2545
    halLedSet(1);
#elif chip==2544 
    HAL_LED_SET_2();
#endif

    return returnval;
}


#if chip==2541 || chip==2543 || chip==2545
/*******************************************************************************
* @fn           remoteSetupMode
*
* @brief        Starts the PER test mode for REMOTE. Will Send an ACK with test 
*               configuration to MASTER then exit the function if a single packet
*               is received and an acknowlegement is transmitted.
*
* @param        void
*
* @return       unsigned char:
*               ACK_COMMAND_REPEAT: 
*               ACK_COMMAND_STOP:   
*/
uint8 remoteSetupMode(void) {
    uint32 nuOfAckSent = 0;

    // Reset LLE (radio).
    halRfDisableRadio(FORCE);

    // Enable radio in default beacon mode.
    halRfInit();
    halRfConfig(GFSK_250kbps_160khz, RX, 32, PKT_CONF_NRF);
    halRfSetFrequency(2402);
    halRfSetCrc(CRC_16_CCITT);
    halRfSetSyncWord(SYNCWORD, 0);
    halRfSetRxAddress(0, ADDR_CONF_RX_AUTO_SW1, 0xEF, 32);
    halRfEnableRadio();

    // Load the configuration ack.
    halRfLoadAckPayload((ackConfig[0]-1), 0, (&ackConfig[1]));
    
    // Set 100 ms for radio timeout.
    halTimer2SetRadioTimeout(100, SINGLE);

    // Enable RF interrupt.
    halRfEnableInterrupt(RFIRQF1_TASKDONE);
      
    // Enable global interrupt.
    halIntOn();

    while(1) {
        // Start receiver.
        halRfStartRx();
    
        // Start timer 2 (asynchronous).
        halTimer2Start(0);
        
        // Wait for TASKDONE and halt CPU (PM0) until task is completed.
        while (!( rfirqf1 & RFIRQF1_TASKDONE)) {
#if(POWER_SAVING)
            halSleepEnterPowerMode(CPU_HALT);
#endif
        }

        // Task ended without error.
        if(PRF.ENDCAUSE == TASK_ENDOK) {
            // Check if packet was received.
            if((rfirqf1 & RFIRQF1_RXOK) == 1) {
                halRfCommand(CMD_RXFIFO_RESET);
            }
        }
        // If T2E2 then set up period of 100 ms is over.
        else if(PRF.ENDCAUSE == TASK_RXTIMEOUT) {

            // Stop Timer2.
            halTimer2Stop(0);

            // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
            rfirqf1 = 0;
            
            // Set to a undefined value in enum type end cause.
            PRF.ENDCAUSE = TASK_UNDEF;

            // Check if any ACK packets are sent.
            nuOfAckSent = PRF_N_TX;
            
            if(!nuOfAckSent) {
                ack_command[1] = queryUserCommand2();
                return ack_command[1];
            }
            break;
        }
        else {
            // Write end cause error code to screen.
            printErrorScreen((unsigned char)PRF.ENDCAUSE);
        }
        // Load configuration ACK again.
        halRfLoadAckPayload((ackConfig[0]-1), 0, (&ackConfig[1]));
        // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
        rfirqf1 = 0;
        // Set to a undefined value in enum type end cause.
        PRF.ENDCAUSE = TASK_UNDEF;
    }

    /* Disable interrupts enabled in this function and clear timer 2 */

    // Disable global interrupt.
    halIntOff();
    // Reset Timer2.
    halTimer2Reset();
    // Disable RF interrupt.
    halRfDisableInterrupt(RFIRQF1_TASKDONE);
    return 0;
}


/*******************************************************************************
* @fn           remoteRxMode
*
* @brief        Mode for receiving all the packets and updating statistics
*               while the Master device is transmitting packets. 
*
* @param        void
*
* @return       unsigned char:
*               SUCCESS             : The function ended successfully.
*               ACK_COMMAND_RESTART : The user wants to restart the test.
*               ACK_COMMAND_STOP    : The user wants to define new test.
*               RUN_ERROR           : Unknown error.
*/
unsigned char remoteRxMode(void) {
    // Variables for RSSI, packet sequence, statistics, etc...
    uint32 packetSeq = 0, packetSeqLast = 0, idx = 0;
    int8 rssi[8] = {0,0,0,0,0,0,0,0}, rssi_new = 0;
    int16 per = 0, rssi_sum = 0,rssi_avg = 0;
    uint32 okPackets = 0, crcPackets = 0, lostPackets = 0, nuOfAckSent = 0, tmp, testTime;
    uint8 counter = 0;

    // Setup the radio with the received configuration.
    halRfDisableRadio(FORCE);
    halRfInit();
    halRfConfig(ackConfig[2], RX, 32, PKT_CONF_NRF);
    halRfSetFrequency(2402);
    halRfSetCrc(CRC_16_CCITT);
    halRfSetSyncWord(SYNCWORD, 0);
    halRfSetRxAddress(0, ADDR_CONF_RX_AUTO_SW1, 0xEF, (1 + ackConfig[3]));
    halRfSetFrequency(ackConfig[1]); 
#if(POWER_SAVING)
    // Capture start of every Rx packet.
    PRF.RADIO_CONF.RXCAP = 1;
    uint8 tuneTimeoutFlag = 0, ovf10Bit1 = 0, ovf10Bit2 = 0;
    uint16 timeStampFine, fine;
    uint32 timeStampCoarse, coarse;
    uint32 sleepDuration;
#endif
    halRfEnableRadio();

    // Set timeout value (total test time + 1 second in margin) to end test when done.
    testTime = (10*(packets)) + 1000;
    halTimer2SetRadioTimeout(testTime, SINGLE);

    // Enable RF interrupt.
    halRfEnableInterrupt(RFIRQF1_TASKDONE);

    // Enable global interrupt.
    halIntOn();

    // Start timer 2 (asynchronous).
    halTimer2Start(0);

    // Set default value for the command expected to be received from REMOTE.
    ack_command[1]=0;
    
    while(1) {
        // Restart test if button1 is pushed.
        if(halButtonPushed() == HAL_BUTTON_1) {
            if(RadioTimeoutFlag) {
                // Button was held in longer than the remaining test time.
                ack_command[1]=ACK_COMMAND_REPEAT;
                break;
            }
            ack_command[1]=ACK_COMMAND_RESTART;
            halTimer2SetRadioTimeout(100, SINGLE);
            halTimer2Start(0);
            
            // Update number of transmitted ack packets.
            nuOfAckSent = PRF_N_TX;
        }
        else if(halJoystickPushed()) {
            ack_command[1] = ACK_COMMAND_STOP;
            halTimer2SetRadioTimeout(100, SINGLE);
            halTimer2Start(0);
        }

        // Start receiver.
        halRfStartRx();

        // Wait for TASKDONE and halt CPU (PM0) until task is completed.
        while (!( rfirqf1 & RFIRQF1_TASKDONE)) {
#if(POWER_SAVING)
            halSleepEnterPowerMode(CPU_HALT);
#endif
        }

        // Check if task ended correctly
        if(PRF.ENDCAUSE == TASK_ENDOK) {
            // Check if packet was received.
            if(rfirqf1 & RFIRQF1_RXOK) {
                // Check if packet length is as expected. 
                tmp=RFD;
                if(ackConfig[3]==tmp) {
                    okPackets++;
                    packetSeq = RFD;
                    tmp = RFD;
                    packetSeq |= ((( (long) tmp) << 8) & 0xFF00);
                    tmp = RFD;
                    packetSeq |= ((( (long) tmp) << 16) & 0xFF0000);
                    
                    // Calculate running average.
                    rssi_new=halRfGetLastRssi();
          
                    if(rssi_new) {
                        // Set index to equal to the 3 LSB bits from the packet sequence number.
                        idx = (packetSeq & 0x07);
                        // Update sum of all the 8 rssi values. 
                        rssi_sum += rssi_new - rssi[idx];
                        // Replace the old value with the new.
                        rssi[idx] = rssi_new;
                        /* Find the new average of the 8 RSSI samples. 
                        * The rssi value will always be negative. 
                        * Make sure that the sign bit is set. */
                        rssi_avg = (rssi_sum >> 3) | 0x8000;
                    }
          
                    // Sanity check of seq. number.
                    if(packetSeq < packetSeqLast) {
                        // Tell user to turn off all MASTER devices but one.
                        halLcdWriteLine(1, c2xd(text_Multiple_Masters));
                        halLcdWriteLine(2, c2xd(text_use_one_master));
                        halLcdWriteLine(3, c2xd(text_ERROR_RESTART2));
                        halLedSet(3);
                        halLedClear(1);
                        halMcuWaitMs(1000);
                        return RUN_ERROR;
                    }
                    else { 
                        // Check if any packets was lost.
                        while((packetSeq-1) != packetSeqLast++) {
                            lostPackets++;
                        }
                    }
                }

                halRfCommand(CMD_RXFIFO_RESET);

                // Reload selected ACK command.
                if(ack_command[1] != 0) {
                    halRfLoadAckPayload(1, 0, (&ack_command[1]));
                }
            }
            else if(rfirqf1 & RFIRQF1_RXNOK) {
                // Received packet with CRC error. 
                crcPackets++;
                /* Packets with CRC error is discarded from the FIFO and will 
                * then be counted as lost when the next valid packet with a new 
                * sequence number is received. */
                lostPackets--;
            }
            else {
                // Write end cause error code to screen.
                printErrorScreen((unsigned char)PRF.ENDCAUSE); 
                halMcuWaitMs(1000);
                return RUN_ERROR;
            }
        }
        // Check if test time has expired or last packet is received.
        if((PRF.ENDCAUSE == TASK_RXTIMEOUT) || (packetSeq == packets)) {
            // Disable global interrupt.
            halIntOff();
            // Set LED1 to indicate that test is complete.
            halLedSet(1);
            // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
            rfirqf1 = 0;
            // Set ENDCAUSE to a undefined value in enum type end cause.
            PRF.ENDCAUSE = TASK_UNDEF;
      
            if(ack_command[1]==ACK_COMMAND_RESTART) {
                if(nuOfAckSent == PRF_N_TX) {
                    /* If ACK_COMMAND_RESTART command was never sent, 
                    *  change to ACK_COMMAND_REPEAT */
                    ack_command[1]=ACK_COMMAND_REPEAT;
                    break;
                }
            }
            else if(ack_command[1]==0) {
                // Check if last sequence number matches the expected last packet.
                while(packetSeqLast++ != packets) {
                    lostPackets++;
                }

                // Update PER value. 
                per=((crcPackets+lostPackets)*1000)/(crcPackets+lostPackets+okPackets);
                printTxtNuTxt(text_PER1, per, text_PER2,  1);

                // Find index to place arrow sign at correct column in LCD display.
                tmp=okPackets;
                counter = 0;
                while(tmp /= 10) {
                    counter++;
                }
                // Update ok packets line with arrow to indicate to user the possibilty of pushing joystick down. 
                printTxtNuTxt(text_OK, okPackets, (text_arrow_down+counter),  3);
                // Present result screen on LCD.
                printResultScreen(okPackets, crcPackets, lostPackets, per, rssi_avg, counter, ackConfig[1], ackConfig[3], ackConfig[2]);
                // Query user to repeat the same test or start a new one.
                ack_command[1] = queryUserCommand();
            }
            // Exit infinite loop and end test.
            break;
        }

        // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
        rfirqf1 = 0;
        // Set to a undefined value in enum type end cause.
        PRF.ENDCAUSE = TASK_UNDEF;

        // Toggle green LED1 for every other received OK packet.
        if(okPackets & 0x01) {
            halLedToggle(1);
        }

        if(ack_command[1] == 0) {
            if(!(okPackets%8)) {
                // Update statistics on LCD (PER and RSSI running average).
                per=((crcPackets+lostPackets)*1000)/(crcPackets+lostPackets+okPackets);
                printTxtNuTxt(text_PER1, per, text_PER2,  1);
                printTxtNuTxt(text_RSSI, rssi_avg, text_dBm,  2);
            }
            // Update packet count on LCD line 3.
            printTxtNuTxt(text_OK, okPackets, text_blank,  3);
        }

#if(POWER_SAVING)
        /*  This part illustrate one way of implementing sleep functionality.
        *   This is not fully optimized and can be implemented in several ways.
        *   
        *   The main procedure here is:
        *       
        *       Get the captured timestamp from the start of the last received packet.
        *       Get the current time from timer2.
        *       Calculate the time delta and subtract that from the default sleep duration.
        *       Set the sleep time duration.
        *       Chose which power mode to enter (or abort if the sleep duration is to short).
        *       Enable Sleep Timer interrupt.
        *       Enable Global Interrupt (EA).
        *       Enter Power Mode.
        *       Wake up on sleep timer isr.
        *       Wait until 32 MHz XTAL is stable.
        *       Sync up the timer2 value witht he low speed 32 kHz sleep timer clock.
        */

        // Retrieve captured timestamp from start of received packet (SOP).
        halTimer2GetCapturedTime(&timeStampFine, &timeStampCoarse);

        // Retrieve current time from timer 2 (TIME).
        halTimer2GetFullCurrentTime(&fine, &coarse);

        /*  Find time delta between SOP and TIME (32 MHz ticks). This calcuation
        *   assumes that the timer2 base period is set to 1 ms (0x7D00). */
        coarse -= timeStampCoarse;
        if( fine < timeStampFine ) {
          fine += (0x7D00 - timeStampFine);
          coarse--;
        }
        else {
          fine -=  timeStampFine;
        }

        /*  Set sleep duration to default (approx. 9 ms) and subtract the 
        *   execution time since last received packet. */
        sleepDuration = 290;

        /*  Convert the 32 MHz ticks to 32 kHz ticks. The conversion has some 
        *   inaccuracy as it ignores the low speed clock setting. The CC2541 
        *   and CC2545 also has an optional low speed crystal oscillator which 
        *   has a slightly different frequency than the RC oscillator:
        *   LS-XOSC: 32.768 kHz
        *   LS-RCOSC: 32.753 kHz.*/
        sleepDuration -= (uint32) ((fine/977) + (32*coarse) + 32);

        if(sleepDuration < 32) {
            // If sleep duration is less than 1 ms, do not enter sleep.
            powerModeFlag = 0;
        }
        else if (sleepDuration < 100) {
            // If sleep duration is less than 3 ms, use PM1 instead of PM2.
            powerModeFlag = 1;
        }
        else {
            powerModeFlag = 2;
        }

        if(powerModeFlag) {
            // Set the sleep time compare value.
            halSleepSetSleepTimer(sleepDuration);

#if((chip == 2541) || (chip == 2545))
            // CC2541/45 has automatic sync between 32 kHz ST and timer2.
            halTimer2Stop(1);
#else 
            // Stop timer 2 and store current sleep timer value.
            halSleepSynchXOSCMsBase(STOP);
#endif
            // Disable the sleep timer interrupt.
            halSleepEnableInterrupt();

            // Make Sure Global Interrupt is enabled.
            halIntOn();
            
            // Set LED3 indicate entering power mode function.
            halLedSet(3);
            
            // No more pillow fights, time for bed.
            halSleepEnterPowerMode(powerModeFlag);
            
            // Clear LED3 indicate exit from power mode function.
            halLedClear(3);

            // Disable the sleep timer interrupt.
            halSleepDisableInterrupt();
            
#if((chip == 2541) || (chip == 2545))
          // CC2541/45 has automatic sync between 32 kHz ST and timer2.
          halTimer2Start(1);
#else
          halSleepSynchXOSCMsBase(START);
#endif
        }        
        /*  Due to the inaccuracy of the synchronizing between the sleep timer 
        *   and timer 2 done above, the code will trim the timer2 current value 
        *   for every 1024 transmitted packets from the  total packets as well as once after receiving one of the 10 last packets.
        */
        
        // Update timer2 value for every 1024 received packet.
        ovf10Bit1 = (packetSeq / 1024);
        if(ovf10Bit1 > ovf10Bit2) {
            halTimer2SetTimerCount(0, (packetSeq*10));
            ovf10Bit2++;
        }
        
        // Update timer2 count if one of the 10 last packets is received.
        if((packetSeq > (packets - 10)) && !tuneTimeoutFlag) {
            halTimer2SetTimerCount(0, (packetSeq*10));
            tuneTimeoutFlag = 1;
        }
#endif
    }

    /* Disable interrupts enabled in this function, clear timer 2 and return. */
    // Disable global interrupt.
    halIntOff();
    // Reset Timer2.
    halTimer2Reset();
    // Disable RF interrupt.
    halRfDisableInterrupt(RFIRQF1_TASKDONE);
    // Return command status.
    return ack_command[1];
}
#endif


/*******************************************************************************
* @fn          main
*
* @brief       Main program
*
* @param       void
*
* @return      int (does not return)
*/
int main(void) {
    // Main auto variables.
    unsigned char runCondition = 0;
#if(chip != 2544)
    unsigned char mode = 0;
#endif

    // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
    rfirqf1 = 0;

    /* Initialize Clock Source (32 Mhz Xtal),
    *  global interrupt (EA=1),  I/O ports and pheripherals(LCD). */
    halBoardInit();

#if (chip==2545)
    /* The CC2545EM use I/O pins instead of the ADC for the joystick. */
    // Enable pull down. on P0.x.
    PPULL = 0xC0;
#endif

    // Set size of arrays in first byte.
    ackConfig[0]=sizeof(ackConfig);
    ack_command[0]=sizeof(ack_command);

#if chip==2544  
    /* The CC2544Dongle only works in Master mode. */
    // Turn on LED1 ( green )
    HAL_LED_SET_2();

    /* CC2544 Dongle is always in master mode. */
    while(1) {
        // Start transmitting beacon packet every 10 ms.
        masterBeaconMode();
        do {
            // Start PER test in transmit mode.
            runCondition = masterTxMode();
        } while(runCondition == ACK_COMMAND_RESTART);
    }
#else
    
    // Turn on LED1 ( green ).
    halLedSet(1);
  
    // Set up welcome screen. 
    halLcdClear();
    halLcdWriteLine(1, c2xd(text_welcome1));
    halLcdWriteLine(2, c2xd(text_welcome2));
    halLcdWriteLine(3, c2xd(text_welcome3));
  
#if(POWER_SAVING)
    /* Set device into PM3 while waiting for user to push button 1 */
    // Interrupt enable on P0.1 (Button1).
    P0IEN |= BIT1;
    // Enable CPU Interrupt for Port 0 (IEN1.P0IE = 1).
    P0IE = 1;
#endif
    
    // Wait for user to press S1 to enter menu.
    while(halButtonPushed() != HAL_BUTTON_1) {
#if(POWER_SAVING)
        halSleepEnterPowerMode(CPU_HALT);
#endif
    }

#if(POWER_SAVING)
    // Interrupt disable on P0.1 (Button1).
    P0IEN &= ~BIT1;
    // Disable CPU Interrupt for Port 0 (IEN1.P0IE = 1).
    P0IE = 0;
#endif

    // Query user for operating mode.
    mode = queryMode();
  
    if(mode == MASTER) {
        while(1) {
            // Start transmitting beacon packet every 10 ms. 
            masterBeaconMode();
            do {
                // Start PER terst in transmit mode. 
                runCondition = masterTxMode();
            } while(runCondition == ACK_COMMAND_RESTART);
        }
    }
    else if(mode==REMOTE) {
        do {
            // Query user for PER test configuration.
            ackConfig[1] = queryFrequency();       // Set frequency.
            ackConfig[2] = queryModulation();      // Set modulation format.
            ackConfig[3] = queryPacketLength();    // Set packet length.
            packets = queryNuOfPackets();           // Set number of packets.

            // Update ackConfig array.
            ackConfig[4]=(packets & 0xFF);
            ackConfig[5]=((packets >> 8) & 0xFF);
            ackConfig[6]=((packets >> 16) & 0xFF);
            ackConfig[7]=((packets >> 24) & 0xFF);

            // If modulation setting != 2 Mbps
            if ((ackConfig[2] != GFSK_2Mbps_320khz) && (ackConfig[2] != GFSK_2Mbps_500khz)) {
                // Set LNA gain if AGC is disabled.
                ackConfig[8] = queryGain();
            }
            do {
                // Send configuration ACK to Master device. 
                runCondition=remoteSetupMode();
                if((runCondition != ACK_COMMAND_REPEAT) && (runCondition != ACK_COMMAND_STOP)) {
                    do {
                        // Start PER test in receive mode. 
                        runCondition=remoteRxMode();
                    } while(runCondition == ACK_COMMAND_RESTART);
                }
            } while(runCondition == ACK_COMMAND_REPEAT);
        } while(runCondition == ACK_COMMAND_STOP);
    }
#endif
}