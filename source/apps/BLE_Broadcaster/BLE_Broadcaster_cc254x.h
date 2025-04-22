/*******************************************************************************
*  Filename:        BLE_Broadcaster_cc254x.h
*  Revised:         $Date: 2013-04-12 14:54:28 +0200 (Fri, 12 Apr 2013) $
*  Revision:        $Revision: 9731 $
*
*  Description:     Header file for CC254x per test.
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

#include "hal_types.h"

#ifndef _PER_TEST_CC254x_H
#define _PER_TEST_CC254x_H

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

#include "prop_regs.h"
#include "hal_timer2.h"
#include "hal_sleep.h"
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"
#include "hal_int.h"
#include "hal_board.h"
#include "hal_button.h"
#include "hal_led.h"
/*******************************************************************************
* DEFINES
*/
// Mode.
//#define REMOTE 0      // Remote Mode.
//#define MASTER 1      // Master Mode.

// Ack commands.
//#define ACK_COMMAND_RESTART   1
//#define ACK_COMMAND_REPEAT    2
//#define ACK_COMMAND_STOP      3

// Function return statements.
//#define RUN_SUCCESS           0
//#define RUN_ERROR             9

/*
* Use the same sync word that is used in SmartRFStudio.
*
* PS:
* Be aware that there does exist a few bad sync words which can have a negative
* impact on the link. For example the sync word should not be set up so that it
* is an extension of the preamble. If the preamble had the bit stream as follows:
* " 1010 1010 ...... " then try to avoid using a sync word with the same pattern
* because this can cause the receiver to sync on the preamble instead of the
* sync word.
*
* The sync word should also have some variation , i.e. avoid using 0xFFFFFFFF or
* 0x00000000.                                                                 */
#define SYNCWORD 0x29417671

//Define if its a Node or a Destiny
//#define MODETX

#ifdef MODETX
#define NODE_NUMBER 1
#endif

#define PAYLOAD_LENGTH 26
#define CHANNEL BLE_BROADCAST_CHANNEL_37
//#define CHANNEL 24
#define TOTAL_TIME 240

#define DAF 1
#define BNC 2
#define DNC 3
#define GDNC 4

#define OPERATION_MODE GDNC
#define TOTAL_NODES 2
#define TOTAL_TRANSMISSIONS 10

#if TOTAL_NODES == 2
#define ROWS 4
#define COLS 2
#define TIME_SLICES 4
#elif TOTAL_NODES == 3
#define ROWS 9
#define COLS 3
#define TIME_SLICES 9
#endif

struct deviceMap
{
  uint8 address[6];
  int number;
  uint16 receivedSequenceNumber;
  uint16 expectedSequenceNumber;
  uint32 totalPackages;
  uint32 packageLosses;
  uint32 numberOfTransmissions;
  float rssiSum;
  int8 rssi;
  uint8 receivedMessages[TOTAL_TRANSMISSIONS + 1];
};

typedef struct deviceMap deviceMap;

extern volatile uint8 rfirqf1;
extern deviceMap deviceList[COLS];
extern uint8 addressBytes[6];
extern uint8 messages[COLS][(PAYLOAD_LENGTH-2)/2];
extern uint8 actedThisPhase;
extern uint16 myNumber;
extern uint8 phase;
extern uint8 transmissionDone;
extern uint16 counter;
extern uint8 messagesFlags[9];
extern uint16 numberOfTransmissions;
extern uint8 receivedMask;
  
extern uint8 codingMatrix[ROWS][COLS];
extern uint16 resultMatrix[ROWS][(PAYLOAD_LENGTH-2)/2];
extern double inverseCodingMatrix[COLS][ROWS];

#endif