/*******************************************************************************
*  Filename:        per_test_cc254x_query.c
*  Revised:         $Date: 2013-04-12 14:54:28 +0200 (Fri, 12 Apr 2013) $
*  Revision:        $Revision: 9731 $
*
*  Description:     Contains all user query functions for the per test 
*                   application.
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
#include "per_test_cc254x_query.h"
#include "per_test_cc254x.h"
#include "per_test_cc254x_strings.h"
#include "hal_rf_proprietary.h"
#include "hal_board.h"
#include "hal_button.h"
#include "hal_joystick.h"
#include "hal_lcd.h"
#include "util_lcd.h"
#include "hal_types.h"
#include "hal_led.h"


/*******************************************************************************
* GLOBAL VARIABLES
*/
// Buffer for use in conversion from __code constants to __xdata variables.
char __xdata txtstr16[18];


/*******************************************************************************
* LOCAL FUNCTIONS
*/
/*******************************************************************************
* @fn           port0_isr
*
* @brief        Port 0 Interrupt Service Routine, which executes when SRF05EB S1
*               (P0_1) is pressed.
*
* @Note         The order in which the following flags are cleared is important.
*
* @param        void
*
* @return       void
*/
#pragma vector = P0INT_VECTOR
__interrupt void port0_isr(void) {
    uint8 p0ifg;
    p0ifg = P0IFG;
    // Clear the active Port 0 Interrupt Flags.
    P0IFG &= ~p0ifg;
    // Clear CPU Interrupt Flag for P0 (IRCON.P0IF = 0).
    P0IF = 0;
}


/*******************************************************************************
* @fn           c2xd
*
* @brief        Convert code char constant pointer to xdata char constant pointer. 
*
* @param        const char __code * src : 
*               char __code pointer to "string" constant. 
*
* @Note:        Due to limited RAM size most of the constants are stored in 
*               CODE space (FLASH). This function convert code constants to 
*               xdata constants for use in writing text to LCD.
*
* @return       __xdata char pointer. 
*/
const char* c2xd( const char __code * src ) {
    const char* s;
    // Map the code pointer to the correct __xdata address.
    s = (const char*)(src+0x8000);
    return s;
}


/*******************************************************************************
* @fn          	printTxtNuTxt
*
* @brief       	Form char array (string) and print to selected row on LCD.
*
* @param        const char __code *text1  : First char array.
*               long number               : number to convert to string. 
*               const char __code *text2  : Second char array. 
*               uint8 lcd_line    : Select LCD line to write to (1 to 3)
*
* @return       void  
*/
void printTxtNuTxt(const char __code *text1, int32 number, const char __code *text2, uint8 lcdLine) {
    char *pValue;
    uint8 p1 = 0, p2 = 0;

    // Add text1 to string.
    while(text1[p2] != '\0') {   
        txtstr16[p1++]= text1[p2++];
    }
  
    // Clear counting variable.
    p2 = 0;

    // Add number to string. 
    pValue = convInt32ToText((int32) number);
     
    while(pValue[p2] != '\0') {
        txtstr16[p1++]= pValue[p2++];
    }

    // Clear counting variable
    p2 = 0;

    while(text2[p2] != '\0') {
        txtstr16[p1++]= text2[p2++];
    }

    // Set last character in string to NULL.
    txtstr16[p1]= '\0';
    halLcdWriteLine(lcdLine, txtstr16);
}      


/*******************************************************************************
* @fn          	printResultScreen
*
* @brief       	Present results from PER test on LCD.
*
* @param        results...
*
* @return       void  
*/
void printResultScreen(uint32 ok_packets, uint32 crc_packets, uint32 lost_packets, uint16 per, int16 rssi_avg, uint8 counter, uint8 channel, uint8 pktLen, uint8 modulation) {
    uint8 joyDir, level = 2, modulationIndexConversion[10]= {255, 255, 2, 4, 0, 255, 1, 3, 5, 6}, count = 2;
    if(channel >= 100) {
        count++;
    }
    while (1) {
        joyDir = halJoystickGetDir();
        if ((joyDir == HAL_JOYSTICK_EVT_DOWN) && (level < 3)) {
            level++;
        }
        else if ((joyDir == HAL_JOYSTICK_EVT_UP) && (level > 0)) {
            level--;
        }
    
        if ((level == 0)) {
            // Write Modulation used to LCD. 
            halLcdWriteLine(1, c2xd(header_line1));
            halLcdWriteLine(2, c2xd(header_line2));
            halLcdWriteLine(3, c2xd(mod_format_result[modulationIndexConversion[modulation]]));
        }
        else if ((level == 1)) {
            // Write Frequency and packet length to LCD.
            printTxtNuTxt(text_Packet, pktLen, text_bytes, 1); 
            printTxtNuTxt(text_FREQ, (channel + 2379), text_MHz, 2);
            printTxtNuTxt(text_Chan, channel, (text_arrow_middle+count),  3);
        }
        else if ((level == 2)) {
            // Update PER on LCD line 1
            printTxtNuTxt(text_PER1, per, text_PER2,  1);
            // Update running RSSI value on LCD line 2.
            printTxtNuTxt(text_RSSI, rssi_avg, text_dBm,  2);
            // Update packet count on LCD line 3.
            printTxtNuTxt(text_OK, ok_packets, (text_arrow_middle+counter),  3);
        }
        else if ((level == 3)) {
            printTxtNuTxt(text_OK, ok_packets, (text_arrow_up+counter),  1);
            // Update running RSSI value on LCD line 2.
            printTxtNuTxt(text_CRC, crc_packets, text_blank,  2);
            // Update packet count on LCD line 3.
            printTxtNuTxt(text_Lost, lost_packets, text_blank,  3);
        }
        if (halButtonPushed() == HAL_BUTTON_1) {
            break;
        }
        halMcuWaitMs(150);
    }
    return;
}


/*******************************************************************************
* @fn          	printResultScreen
*
* @brief       	Present ERROR screen on LCD.
*
* @param        uint8 endCause: end cause.
*
* @return       void  
*/
void printErrorScreen(uint8 endCause) {
    printTxtNuTxt(text_ERROR_CODE, endCause, text_blank, 1);
    halLcdWriteLine(2, c2xd(text_ERROR_RESTART1));
    halLcdWriteLine(3, c2xd(text_ERROR_RESTART2));
    halLedSet(3);    
    halLedClear(1);  
    return;
}


/*******************************************************************************
* @fn          queryMode
*
* @brief       Select mode.
*
* @param       void
*
* @return      Selected mode: MASTER or REMOTE. 
*/
uint8 queryMode(void) {  
    uint8 mode = REMOTE, joyDir ;
    
    halLcdClear();
    halLcdWriteLine(1, c2xd(text_ModeQ));
    halLcdWriteLine(2, c2xd(text_Remote));
    halLcdWriteLine(3, c2xd(header_select_bottom));
    mode = REMOTE;

    while (1) {
        joyDir = halJoystickGetDir();
  
        if ((joyDir == HAL_JOYSTICK_EVT_UP) && (mode == 0)) {
            mode = MASTER;
            halLcdWriteLine(2, c2xd(text_Master));
            halLcdWriteLine(3, c2xd(header_select_top));
        }
        else if ((joyDir == HAL_JOYSTICK_EVT_DOWN) && (mode == 1)) {
            mode = REMOTE;
            halLcdWriteLine(2, c2xd(text_Remote));
            halLcdWriteLine(3, c2xd(header_select_bottom));
        }

        if (halButtonPushed() == HAL_BUTTON_1) {
            break;
        }
        // 100 ms delay.
        halMcuWaitMs(100);
    }  
    // Return chosen mode. 
    return mode;
}


/*******************************************************************************
* @fn          queryFrequency
*
* @brief       Query user for a selected frequency. 
*
* @param       void
*
* @return      uint8
*/
uint8 queryFrequency(void) {  
    uint8 chan = 23, joyDir;
      
    halMcuWaitMs(100);
    halLcdClear();
    halLcdWriteLine(3, c2xd(header_select_bottom));
 
    while (1) {
    joyDir = halJoystickGetDir();
  
    if ((joyDir == HAL_JOYSTICK_EVT_UP) && (chan < 101)) {
        chan++;
    }
    else if ((joyDir == HAL_JOYSTICK_EVT_DOWN) && (chan > 23)) {
        chan--;
    }

    // Update last line ( selection info line ). 
    if(chan == 23) {
        halLcdWriteLine(3, c2xd(header_select_bottom));
    }
    else if(chan == 101) {
        halLcdWriteLine(3, c2xd(header_select_top));
    }
    else {
        halLcdWriteLine(3, c2xd(header_select_middle));
    }
    
    // Update LCD.
    printTxtNuTxt(text_Chan, chan, text_blank,  1);
    printTxtNuTxt(text_FREQ, (chan + 2379), text_MHz, 2);
    
    if (halButtonPushed() == HAL_BUTTON_1) {
        break;
    }
    halMcuWaitMs(150);
  }
  // return channel (frequency).
  return chan;
}


/*******************************************************************************
* @fn          queryModulation
*
* @brief       Query user for a selected modulation type and data rate. 
*
* @param       void
*
* @return      void
*/
uint8 queryModulation(void) {
    uint8 joyDir, modulation[] = {4, 6, 2, 7, 3, 8, 9}, modulationIndex = 0;
    
    halLcdClear();
    halLcdWriteLine(1, c2xd(header_line1));
    halLcdWriteLine(2, c2xd(header_line2));
    halLcdWriteLine(3, mod_format[modulationIndex]);

    while (1) {
        joyDir = halJoystickGetDir();

        if ((joyDir == HAL_JOYSTICK_EVT_UP) && (modulationIndex < 6)) {
            modulationIndex++;
        }
        else if ((joyDir == HAL_JOYSTICK_EVT_DOWN) && (modulationIndex > 0)) {  
            modulationIndex--;
        }

        // Update LCD.
        halLcdWriteLine(3, mod_format[modulationIndex]);

        if (halButtonPushed() == HAL_BUTTON_1) {
            break;
        }
        halMcuWaitMs(150);
    }
    // Return modulation scheme.
    return modulation[modulationIndex];
}


/*******************************************************************************
* @fn          queryPacketLength
*
* @brief       Query user for a selected packet length.  
*
* @param       void
*
* @return      uint8
*/
uint8 queryPacketLength(void) {
    uint8 pkt_len, pkt_len_indx = 0, joyDir;
    
    // Update LCD with inquiry for user.
    halLcdClear();
    halLcdWriteLine(1, c2xd(text_Packet_LengthQ));
    halLcdWriteLine(3, c2xd(header_select_bottom));
    pkt_len_indx = 0;
  
    while (1) {
        joyDir = halJoystickGetDir();
  
        if ((joyDir == HAL_JOYSTICK_EVT_UP) && (pkt_len_indx < sizeof(pkt_length) - 1)) {
            pkt_len_indx++;
        }
        else if ((joyDir == HAL_JOYSTICK_EVT_DOWN) && (pkt_len_indx > 0)) {
            pkt_len_indx--;
        }
    
        // Update LCD.
        pkt_len = pkt_length[pkt_len_indx];
        printTxtNuTxt(text_blank, pkt_len, text_bytes, 2);
       
        // Update last line ( selection info line ). 
        if(pkt_len_indx == 0) {
            halLcdWriteLine(3, c2xd(header_select_bottom));
        }
        else if(pkt_len_indx == 3) {
            halLcdWriteLine(3, c2xd(header_select_top));
        }
        else {
            halLcdWriteLine(3, c2xd(header_select_middle));
        }
    
        if (halButtonPushed() == HAL_BUTTON_1) {
            break;
        }
        halMcuWaitMs(150);
    }
    // Return packet length. 
    return pkt_len;
}


/*******************************************************************************
* @fn          queryNuOfPackets
*
* @brief       Query user for number of packets to be sent.  
*
* @param       void
*
* @return      uint8
*/
unsigned long queryNuOfPackets(void) {
    uint32 packets; 
    uint8 npkt_indx = 0, joyDir;
  
    // Update LCD with inquiry for user.
    halLcdClear();
    halLcdWriteLine(1, c2xd(text_Num_of_PacketsQ));
    halLcdWriteLine(3, c2xd(header_select_bottom));
    npkt_indx = 0;
  
    while (1) {
        joyDir = halJoystickGetDir();

        if ((joyDir == HAL_JOYSTICK_EVT_UP) && (npkt_indx < 4)) {
            npkt_indx++;
        }
        else if ((joyDir == HAL_JOYSTICK_EVT_DOWN) && (npkt_indx > 0)) {
            npkt_indx--;
        }

        // Update "number of packets" variable.
        packets = npackets[npkt_indx];

        // Update LCD.
        printTxtNuTxt(text_blank, packets, text_Packets, 2);
        // Update last line ( selection info line ).
        if(npkt_indx == 0) {
            halLcdWriteLine(3, c2xd(header_select_bottom));
        }
        else if(npkt_indx == 4) {
            halLcdWriteLine(3, c2xd(header_select_top));
        }
        else {
            halLcdWriteLine(3, c2xd(header_select_middle));
        }

        if (halButtonPushed() == HAL_BUTTON_1) {
            break;
        }
        halMcuWaitMs(150);
    }
    // Return number of packets to be sent/received.
    return packets;
} 


/*******************************************************************************
* @fn          queryGain
*
* @brief       Query user for HIGH or LOW gain.
*
* @param       void
*
* @return       uint8: Selected gain ( High or Low ).
*
*/
uint8 queryGain(void) { 
    uint8 gain = HAL_RF_GAIN_LOW, joyDir;

    // Update LCD with inquiry for user.
    halLcdClear();
    halLcdWriteLine(1, c2xd(text_LNA_gainQ));
    halLcdWriteLine(2, c2xd(text_LOW));
    halLcdWriteLine(3, c2xd(header_select_bottom));

    while (1) {
        joyDir = halJoystickGetDir();

        if ((joyDir == HAL_JOYSTICK_EVT_UP) && (gain == HAL_RF_GAIN_LOW)) {
            gain = HAL_RF_GAIN_HIGH;
            halLcdWriteLine(2, c2xd(text_HIGH));
            halLcdWriteLine(3, c2xd(header_select_top));
        }
        else if ((joyDir == HAL_JOYSTICK_EVT_DOWN) && (gain == HAL_RF_GAIN_HIGH)) {
            gain = HAL_RF_GAIN_LOW;
            halLcdWriteLine(2, c2xd(text_LOW));
            halLcdWriteLine(3, c2xd(header_select_bottom));
        }

        if (halButtonPushed() == HAL_BUTTON_1) {
            break;
        }
        halMcuWaitMs(150);
    }
    // return gain status.
    return gain;
}


/*******************************************************************************
* @fn           queryUserCommand
*
* @brief        Query user for setting up same test again or setting up a new.
*
* @param        void
*
* @return       uint8: User command.
*
*/
uint8 queryUserCommand(void) { 
    uint8 joyDir, command;

    // Query user for command (repeat test or set up new).
    halLcdClear();
    halLcdWriteLine(1, c2xd(text_Command));
    halLcdWriteLine(2, c2xd(text_Restart_TestQ));
    halLcdWriteLine(3, c2xd(header_select_bottom));
    
    command = ACK_COMMAND_REPEAT;
    
    while (1) {
        joyDir = halJoystickGetDir();
        if ((joyDir == HAL_JOYSTICK_EVT_UP)) {
            command = ACK_COMMAND_STOP;
            halLcdWriteLine(2, c2xd(text_New_TestQ));
            halLcdWriteLine(3, c2xd(header_select_top));
        }
        else if ((joyDir == HAL_JOYSTICK_EVT_DOWN)) {
            command = ACK_COMMAND_REPEAT;
            halLcdWriteLine(2, c2xd(text_Restart_TestQ));
            halLcdWriteLine(3, c2xd(header_select_bottom));
          }
          if (halButtonPushed() == HAL_BUTTON_1) {
            break;
          }
        halMcuWaitMs(150);
    }
    return command;
}


/*******************************************************************************
* @fn           queryUserCommand2
*
* @brief        Query user for setting up same test again or setting up a new.
*
* @param        void
*
* @return       uint8: User command.
*
*/
uint8 queryUserCommand2(void) {
    uint8 joyDir, command;

    halLcdClear();
    halLcdWriteLine(1, c2xd(text_no_beacon));
    halLcdWriteLine(2, c2xd(text_New_RetryQ));
    halLcdWriteLine(3, c2xd(header_select_bottom));
    command = ACK_COMMAND_REPEAT;

    while (1) {
        joyDir = halJoystickGetDir();
        if ((joyDir == HAL_JOYSTICK_EVT_UP)) {
            command = ACK_COMMAND_STOP;
            halLcdWriteLine(2, c2xd(text_New_TestQ));
            halLcdWriteLine(3, c2xd(header_select_top));
        }
        else if ((joyDir == HAL_JOYSTICK_EVT_DOWN)) {
            command = ACK_COMMAND_REPEAT;
            halLcdWriteLine(2, c2xd(text_New_RetryQ));
            halLcdWriteLine(3, c2xd(header_select_bottom));
        }
        if (halButtonPushed() == HAL_BUTTON_1) {
            break;
        }
        halMcuWaitMs(150);
    }
    return command;
}