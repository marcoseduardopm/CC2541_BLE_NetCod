/*******************************************************************************
*  Filename:        per_test_cc254x_strings.c
*  Revised:         $Date: 2013-04-12 14:54:28 +0200 (Fri, 12 Apr 2013) $
*  Revision:        $Revision: 9731 $
*
*  Description:     Contains all strings displayed on the SmartRF05EB LCD.
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
#include "hal_types.h"


/******************************************************************************
* STATIC VARIABLES
*/

/*  Most of the text written to LCD is stored in CODE memory space to save space
*   in RAM. The IAR compiler is set to write all constants by default to XDATA, 
*   but CC2543/45 only has 1 KB of RAM which is to small for all the string 
*   constants used in this program.
*/
// Welcome screen text.
#if chip==2541
const char __code text_welcome1[]           = "CC2541 PER Test ";
#elif chip==2543
const char __code text_welcome1[]           = "CC2543 PER Test ";
#elif chip==2545
const char __code text_welcome1[]           = "CC2545 PER Test ";
#endif
const char __code text_welcome2[]           = "Push 'Button 1' ";
const char __code text_welcome3[]           = "To Begin        ";

// Mode selection text.
const char __code text_ModeQ[]              = "Mode?";
const char __code text_Master[]             = "Master";
const char __code text_Remote[]             = "Remote";
      
// Frequency selection text.
const char __code text_ChannelQ[]           = "Channel: ";
const char __code text_Chan[]               = "Chan:   ";
const char __code text_FREQ[]               = "FREQ:   ";
const char __code text_MHz[]                = " MHz";
 
// Packet length selection text.
const char __code text_Packet_LengthQ[]     = "Packet Length?  ";
const char __code text_Packet[]             = "Packet: ";
const char __code text_bytes[]              = " bytes";
const char __code text_B[]                  = " B";

// Number of Packets selection text.
const char __code text_Num_of_PacketsQ[]    = "Num. of Packets?";
const char __code text_Packets[]            = " Packets";

// LNA Gain selection text.
const char __code text_LNA_gainQ[]          = "LNA gain?";
const char __code text_LOW[]                = "LOW";
const char __code text_HIGH[]               = "HIGH";

// PER test run text. 
const char __code text_Master_Mode[]        = "Master Mode (Tx)";
const char __code text_Remote_Mode[]        = "Remote Mode";
const char __code text_Pckts[]              = "Pckts: ";
const char __code text_beacon[]             = "Tx Beacon (10ms)";
const char __code text_no_beacon[]          = "No Beacon Signal";
const char __code text_retry[]              = "Retry?";
const char __code text_Command[]            = "Command:";
const char __code text_Restart_TestQ[]      = "Restart Test?";
const char __code text_New_TestQ[]          = "New Test?";
const char __code text_New_RetryQ[]         = "Retry?";
const char __code text_PER1[]               = "PER:   ";
const char __code text_PER2[]               = " %.";
const char __code text_RSSI[]               = "RSSI: ";
const char __code text_dBm[]                = " dBm";
const char __code text_dB[]                 = " dB";
const char __code text_OK[]                 = "N# OK: ";
const char __code text_CRC[]                = "CRC:   ";
const char __code text_Lost[]               = "Lost:  ";
const char __code text_ERROR_CODE[]         = "ERROR CODE: ";
const char __code text_Multiple_Masters[]   = "Multiple MASTERs";
const char __code text_use_one_master[]     = "Use one MASTER";
const char __code text_ERROR_RESTART1[]     = "Set up new test:";
const char __code text_ERROR_RESTART2[]     = "Press EM RESET";
const char __code text_remote_online[]      = "Remote: ONLINE";
const char __code text_remote_offline[]     = "Remote: OFFLINE";
const char __code text_arrow_down[9]        = {' ', ' ', ' ', ' ', ' ', ' ', ' ', 0x1F};
const char __code text_arrow_up[9]          = {' ', ' ', ' ', ' ', ' ', ' ', ' ', 0x1E};
const char __code text_arrow_middle[9]      = {' ', ' ', ' ', ' ', ' ', ' ', 0x1E, 0x1F};
const char __code text_blank[]              = "";
const char  __code header_select_middle[17] = {0x1E, 0x1F, ' ', ' ', ' ', ' ', ' ', '>', '>','B', 'u', 't', 't', 'o', 'n', '1'};
const char  __code header_select_top[17]    = {0x1F, ' ' , ' ', ' ', ' ', ' ', ' ', '>', '>','B', 'u', 't', 't', 'o', 'n', '1'};
const char  __code header_select_bottom[17] = {0x1E, ' ' , ' ', ' ', ' ', ' ', ' ', '>', '>','B', 'u', 't', 't', 'o', 'n', '1'};

// Modulation selection text.
const char __code header_line1[] =           "MOD  RATE DEV";
const char __code header_line2[] =           "     bps  kHz"; 
const char __xdata mod_format[][18]         = { {'G', 'F', 'S', 'K', ' ', '2', '5', '0', 'k', ' ', '1', '6', '0', ' ', ' ', 0x1E, '\0'},        // modulation = 0 - 250 kbps
                                                {'G', 'F', 'S', 'K', ' ', ' ', ' ', '1', 'M', ' ', '1', '6', '0', ' ', 0x1F, 0x1E, '\0'},       // modulation = 1 - 1 Mbps
                                                {'G', 'F', 'S', 'K', ' ', ' ', ' ', '1', 'M', ' ', '2', '5', '0', ' ', 0x1F, 0x1E, '\0'},       // modulation = 2 - 1 Mbps
                                                {'G', 'F', 'S', 'K', ' ', ' ', ' ', '2', 'M', ' ', '3', '2', '0', ' ', 0x1F, 0x1E, '\0'},       // modulation = 3 - 2 Mbps
                                                {'G', 'F', 'S', 'K', ' ', ' ', ' ', '2', 'M', ' ', '5', '0', '0', ' ', 0x1F, 0x1E, '\0'},       // modulation = 4 - 2 Mbps
                                                {'M', 'S', 'K', ' ', ' ', '2', '5', '0', 'k', ' ', '-', '-', '-', ' ', 0x1F, 0x1E, '\0'},       // modulation = 5 - 250 kbps
                                                {'M', 'S', 'K', ' ', ' ', '5', '0', '0', 'k', ' ', '-', '-', '-', ' ', ' ', 0x1F, '\0'} };      // modulation = 6 - 500 kbps

const char __code mod_format_result[7][17]   = { {'G', 'F', 'S', 'K', ' ', '2', '5', '0', 'k', ' ', '1', '6', '0', ' ', ' ', 0x1F, '\0'},        // modulation = 0 - 250 kbps
                                                {'G', 'F', 'S', 'K', ' ', ' ', ' ', '1', 'M', ' ', '1', '6', '0', ' ', ' ', 0x1F, '\0'},        // modulation = 1 - 1 Mbps
                                                {'G', 'F', 'S', 'K', ' ', ' ', ' ', '1', 'M', ' ', '2', '5', '0', ' ', ' ', 0x1F, '\0'},        // modulation = 2 - 1 Mbps
                                                {'G', 'F', 'S', 'K', ' ', ' ', ' ', '2', 'M', ' ', '3', '2', '0', ' ', ' ', 0x1F, '\0'},        // modulation = 3 - 2 Mbps
                                                {'G', 'F', 'S', 'K', ' ', ' ', ' ', '2', 'M', ' ', '5', '0', '0', ' ', ' ', 0x1F, '\0'},        // modulation = 4 - 2 Mbps
                                                {'M', 'S', 'K', ' ', ' ', '2', '5', '0', 'k', ' ', '-', '-', '-', ' ', ' ', 0x1F, '\0'},        // modulation = 5 - 250 kbps
                                                {'M', 'S', 'K', ' ', ' ', '5', '0', '0', 'k', ' ', '-', '-', '-', ' ', ' ', 0x1F, '\0'} };      // modulation = 6 - 500 kbps


// Constant arrays with parameters for PER test options.
const char __code mod_setting[]             = {4, 9, 6, 2, 7, 3};
const char __code pkt_length[4]             = {10, 16, 32, 63};
const unsigned long __code npackets[]       = {100, 1000, 10000, 100000, 1000000};