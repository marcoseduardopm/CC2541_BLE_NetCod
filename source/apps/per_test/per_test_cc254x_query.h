/*******************************************************************************
*  Filename:        per_test_cc254x_query.h
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

#ifndef _PER_TEST_CC254x_QUERY_H
#define _PER_TEST_CC254x_QUERY_H
#include "hal_types.h"

/******************************************************************************
* FUNCTION PROTOTYPES. 
*/
const char* c2xd( const char __code * src );
void printTxtNuTxt(const char __code *text1, int32 number, const char __code *text2, uint8 lcdLine);
void printResultScreen(uint32 ok_packets, uint32 crc_packets, uint32 lost_packets, uint16 per, int16 rssi_avg, uint8 counter, uint8 frequency, uint8 pktLen, uint8 modulation);
void printErrorScreen(uint8 endCause);
uint8 queryMode(void);
uint8 queryFrequency(void);
uint8 queryModulation(void);
uint8 queryPacketLength(void);
uint32 queryNuOfPackets(void);
uint8 queryGain(void);
uint8 queryUserCommand(void);
uint8 queryUserCommand2(void);

#endif