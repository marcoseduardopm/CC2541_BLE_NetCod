/*******************************************************************************
*  Filename:        BLE_Broadcaster_cc254x.c
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

#include "BLE_Broadcaster_cc254x.h"
#include "NodeFunctions.h"
#include "DestinyFunctions.h"
#include "CommonFunctions.h"
#include <stdlib.h>
#include <stdio.h>

volatile uint8 rfirqf1 = 0;
extern uint8 RadioTimeoutFlag;

/*******************************************************************************
* GLOBAL VARIABLES
*/
uint16 counter = 0;
uint8 ledStatus = 0;
uint8 transmissionDone = 0;
uint8 actedThisPhase = 0;

uint8 myNumber = 0;
uint8 phase = 0;

uint8 addressBytes[6];
uint8 messages[3][PAYLOAD_LENGTH-1];
uint8 messagesFlags[9];

deviceMap deviceList[3];

uint8 messageCounter = 0;
uint8 messageSent = 0;

uint8 codingMatrix[ROWS][COLS];
double codingMatrixDouble[ROWS][COLS];
double inverseCodingMatrix[COLS][ROWS];
uint8 resultMatrix[ROWS][PAYLOAD_LENGTH-1];
  
uint8 timeSlices;

uint8 powerModeFlag;

/*******************************************************************************
* LOCAL FUNCTIONS
*/

// Timer 1 Interrupt routine (every 1 ms)
HAL_ISR_FUNCTION(T1_ISR,T1_VECTOR) 
{
  counter++;
  if(counter >= TOTAL_TIME)
  {   
    ledStatus = !ledStatus;
    TurnLED(ledStatus);
    counter = 0;
    actedThisPhase = 0;
    phase = (phase+1);
    if(phase >= timeSlices)
    {
      phase = 0;
      transmissionDone = 1;
    }
    if(RFST != 0)
    {
      halRfCommand(CMD_SHUTDOWN);
      while (!(rfirqf1 & RFIRQF1_TASKDONE)){}
      rfirqf1 = 0;
    }
  }
  IRCON &= 0xFD; // Clean interrupt flag
  return;
}

void ConfigureTimer()
{
  CLKCONCMD &= 0xC7; //32MHz Timer tick
  T1CTL = 0x02;      //Count from 0 to T1CC0 (32000 - for 1 ms)
  T1CCTL0 = 0x44;    //Compare mode
  T1CC0L = 0x00;     //T1CC0 LSB
  T1CC0H = 0x7D;     //T1CC0 MSB
  IEN1 |= 0x2;       //Enable Timer 1 interrupt
}

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

    // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
    rfirqf1 = 0;
 
    //Ensure that the P12 is at input (high impedance)
    MCU_IO_INPUT(1, 2, MCU_IO_PULLDOWN);
    
    /* Initialize Clock Source (32 Mhz Xtal),
    *  global interrupt (EA=1),  I/O ports and pheripherals(LCD). */
    halBoardInit();

    //Ensure that the P12 is at input (high impedance)
    MCU_IO_INPUT(1, 2, MCU_IO_TRISTATE);
    
    printf("Init\n");
    
    halRfDisableRadio(FORCE);
    halRfBroadcastInit();
    halRfBroadcastSetChannel(CHANNEL);
    
    halRfEnableRadio();
    
    //all pins must be at high-impedance (input) if not used or output low
    for(int i = 0; i < 8; i++)
    {
      MCU_IO_INPUT(0, i, MCU_IO_PULLDOWN);
      MCU_IO_INPUT(1, i, MCU_IO_PULLDOWN);
    }    
    
    for(int i = 0; i < 5; i++)
    {
      MCU_IO_INPUT(2, i, MCU_IO_PULLDOWN);
    }

    sleepMode(1000, 0); //sleep 
    //Turn the LED ON
    TurnLED(1);
    sleepMode(200, 0); //sleep 
    //Ensure that the P12 is at input (high impedance)
    TurnLED(0);
    
    halRfEnableInterrupt(RFIRQF1_TASKDONE);
    
#if TOTAL_NODES == 2
    timeSlices = 4;
#elif TOTAL_NODES == 3
    timeSlices = 9;
#endif
    
#ifdef MODETX

#if NODE_NUMBER == 0
    uint32 waitTime = TOTAL_TIME/2;
#elif NODE_NUMBER == 1
    uint32 waitTime = TOTAL_TIME/3;
#elif NODE_NUMBER == 2
    uint32 waitTime = TOTAL_TIME/6;
#endif

    for(int i = 0; i < 6; i++)
      addressBytes[i] = deviceList[NODE_NUMBER].address[i];
    
    while(!ReceiveInitSignal()) {}
    
    //sleepMode(waitTime, 0);
    
    ClearMessages();
    
    //Set Timer 1 to interrupt every 1 ms
    ConfigureTimer();
    
    while(counter < waitTime){}
    counter = 0;
    
    while(1)  
    {
      
      NodeSetup();
      
      while(!transmissionDone)
      {
        NodeRun();
      }
      
      myNumber++;
      ClearMessages();
      transmissionDone = 0;
      messageSent = 0;
    }
    
#else
    
    IncludeDevices();
    
    uint8* startMessage = malloc(PAYLOAD_LENGTH-1*sizeof(uint8));
    Transmit(255,startMessage);
    free(startMessage);
    
    //Set Timer 1 to interrupt every 1 ms
    ConfigureTimer();
    
    while(1)  
    {
      
      DestinySetup();
      
      while(!transmissionDone)
      {
        DestinyRun();
      }
      
      for(int i = 0; i < ROWS; i++)
      {
        if(!messagesFlags[i])
          ZeroLine(codingMatrix[i],COLS);
        for(int j = 0; j < COLS; j++)
          codingMatrixDouble[i][j] = (double) codingMatrix[i][j];
      }
      
      InvertMatrix((double*)codingMatrixDouble,(double*)inverseCodingMatrix);
      GetResults(COLS,ROWS,ROWS,PAYLOAD_LENGTH-1);
      
      //if(messagesFlags[0]&&messagesFlags[1]&&messagesFlags[2]&&messagesFlags[3])
        //PrintMatrix((uint8*)messages,2,PAYLOAD_LENGTH-1);
      printf("%d %d\n", deviceList[0].sequenceNumber, deviceList[1].sequenceNumber);
      messageCounter = 0;
      ClearMessages();
      transmissionDone = 0;
    }
    
#endif
}