/*******************************************************************************
*  Filename:        per_test_cc254x_strings.h
*  Revised:         $Date: 2013-04-12 14:54:28 +0200 (Fri, 12 Apr 2013) $
*  Revision:        $Revision: 9731 $
*
*  Description:     Header file.
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


#ifndef _PER_TEST_CC254x_STRINGS_H
#define _PER_TEST_CC254x_STRINGS_H
/******************************************************************************
* STATIC VARIABLES
*/
// Welcome screen text.
extern const char __code text_welcome1[];
extern const char __code text_welcome2[];
extern const char __code text_welcome3[];

// Mode selection text.
extern const char __code text_ModeQ[];
extern const char __code text_Master[];
extern const char __code text_Remote[];
      
// Frequency selection text.
extern const char __code text_ChannelQ[];
extern const char __code text_Chan[];
extern const char __code text_FREQ[];
extern const char __code text_MHz[];
 
// Packet length selection text.
extern const char __code text_Packet_LengthQ[];
extern const char __code text_Packet[];
extern const char __code text_bytes[];
extern const char __code text_B[];

// Number of Packets selection text.
extern const char __code text_Num_of_PacketsQ[];
extern const char __code text_Packets[];

// LNA Gain selection text.
extern const char __code text_LNA_gainQ[];
extern const char __code text_LOW[];
extern const char __code text_HIGH[];

// PER test run text. 
extern const char __code text_Master_Mode[];
extern const char __code text_Remote_Mode[];
extern const char __code text_Pckts[];
extern const char __code text_beacon[];
extern const char __code text_no_beacon[];
extern const char __code text_retry[];
extern const char __code text_Command[];
extern const char __code text_Restart_TestQ[];
extern const char __code text_New_TestQ[];
extern const char __code text_New_RetryQ[];
extern const char __code text_PER1[];
extern const char __code text_PER2[];
extern const char __code text_RSSI[];
extern const char __code text_dBm[];
extern const char __code text_dB[];
extern const char __code text_OK[];
extern const char __code text_CRC[];
extern const char __code text_Lost[];
extern const char __code text_ERROR_CODE[];
extern const char __code text_Multiple_Masters[];
extern const char __code text_use_one_master[];
extern const char __code text_ERROR_RESTART1[];
extern const char __code text_ERROR_RESTART2[];
extern const char __code text_remote_online[];
extern const char __code text_remote_offline[];
extern const char __code text_arrow_down[9];
extern const char __code text_arrow_up[9];
extern const char __code text_arrow_middle[9];
extern const char __code text_blank[];
extern const char  __code header_select_middle[17];
extern const char  __code header_select_top[17];
extern const char  __code header_select_bottom[17];

// Modulation selection text.
extern const char __code header_line1[];
extern const char __code header_line2[]; 
extern const char __xdata mod_format[][18];
extern const char __code mod_format_result[7][17];

// Constant arrays with parameters for PER test options. 
extern const char __code mod_setting[];
extern const char __code pkt_length[4];
extern const unsigned long __code npackets[];

#endif