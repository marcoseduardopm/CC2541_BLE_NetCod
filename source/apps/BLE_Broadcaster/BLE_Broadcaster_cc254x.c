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
#include "BLE_Broadcaster_cc254x.h"
#include "prop_regs.h"
#include "hal_timer2.h"
#include "hal_sleep.h"
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"
#include "hal_int.h"
#include "hal_board.h"
#include "hal_button.h"
#include "hal_led.h"
#include <stdlib.h>

#define DEBUG 1
#define DELAYTIME 5000 //60000 //for TX

#if(DEBUG)
  #include <stdio.h>
#endif

// Global flags.
extern volatile uint8 rfirqf1;
extern uint8 RadioTimeoutFlag;

struct deviceMap
{
  uint8 address[6];
  int number;
  uint8 sequenceNumber;
  uint32 totalPackages;
  uint32 packageLosses;
};

typedef struct deviceMap deviceMap;

#if(DEBUG)
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) asm("nop");
#endif

#if(POWER_SAVING)
uint8 powerModeFlag;
#endif

/*******************************************************************************
* GLOBAL VARIABLES
*/

uint16 counter = 0;
uint8 ledStatus = 0;
uint8 isTimeToTransmit = 0;
uint16 randomTime;
uint32 myNumber = 0;

deviceMap* deviceList;
int numberOfDevices = 0;

/*******************************************************************************
* LOCAL FUNCTIONS
*/

void IncludeDevice(uint8* add, int num, uint32 seq)
{
  deviceMap newDevice;
  for(int i = 0; i < 6; i++)
    newDevice.address[i] = add[i];
  newDevice.number = num;
  newDevice.sequenceNumber = seq;
  newDevice.totalPackages = 1;
  newDevice.packageLosses = 0;
  
  numberOfDevices++;
  deviceList = realloc(deviceList, numberOfDevices*sizeof(deviceMap));
  deviceList[numberOfDevices-1] = newDevice;
}

deviceMap* GetDeviceByNumber(int num)
{
  for(int i = 0; i < numberOfDevices; i++)
  {
    if(deviceList[i].number == num)
      return &(deviceList[i]);
  }
  return NULL;
}

deviceMap* GetDeviceByAddress(uint8* add)
{
  for(int i = 0; i < numberOfDevices; i++)
  {
    uint8 isEqual = 1;
    for(int j = 0; j < 6; j++)
    {
      if(deviceList[i].address[j] != add[j])
        isEqual = 0;
    }
    if(isEqual)
      return &(deviceList[i]);
  }
  return NULL;
}

void GetMessagePayload(uint8* outputAddress, uint8* outputData)
{
  if(RFRXFLEN > PAYLOAD_LENGTH)
  {
    RFD;RFD;RFD;
    for(int i = 0; i < 6; i++){
      outputAddress[i] = RFD;
      //PRINTF("%d ", outputAddress[i]);
    }
    RFD;RFD;RFD;RFD;RFD;
    for(int i = 0; i < PAYLOAD_LENGTH; i++){
      outputData[i] = RFD;
      //PRINTF("%d ", outputData[i]);
    }
    //PRINTF("\n");
  }
  else
    PRINTF("EMPTY PACKAGE\n");
}
  
void halRfLoadBLEBroadcastPacketPayload(uint8 print)
{
  uint8 i;
  uint8 send_data_length = RFRXFLEN;
  if(print)
    PRINTF("%d: ", send_data_length);
  //Number of bytes effective data
  //send_data_length = (sizeof(data_to_send)/sizeof(int));

  uint8* received_data = malloc(send_data_length);
  //Read data from FIFO
  for(i=0;i<send_data_length;i++){
    //received_data[i] = RFD;
    //if(print)
      //PRINTF("%d ", received_data[i]);
  }
  PRINTF("\n");

  free(received_data);
  return;  
}

void sleepMode(uint32 sleepDurationMs, uint8 afterLastRecPackage)
{
  
#if(POWER_SAVING)
    //uint8 tuneTimeoutFlag = 0, ovf10Bit1 = 0, ovf10Bit2 = 0;
    uint16 timeStampFine, fine;
    uint32 timeStampCoarse, coarse;
    int32 sleepDuration;
        //  This part illustrate one way of implementing sleep functionality.
        //   This is not fully optimized and can be implemented in several ways.
        //
        //   The main procedure here is:
        //
        //       Get the captured timestamp from the start of the last received packet.
        //       Get the current time from timer2.
        //       Calculate the time delta and subtract that from the default sleep duration.
        //       Set the sleep time duration.
        //       Chose which power mode to enter (or abort if the sleep duration is to short).
        //       Enable Sleep Timer interrupt.
        //       Enable Global Interrupt (EA).
        //       Enter Power Mode.
        //       Wake up on sleep timer isr.
        //       Wait until 32 MHz XTAL is stable.
        //       Sync up the timer2 value witht he low speed 32 kHz sleep timer clock.
        //
        //  Set sleep duration to default (approx. 9 ms) and subtract the
        //  execution time since last received packet. 
        //sleepDuration = 290; //9ms
        sleepDuration = sleepDurationMs*32; //conversion to ms
        if(afterLastRecPackage)
        {
          // Retrieve captured timestamp from start of received packet (SOP).
          halTimer2GetCapturedTime(&timeStampFine, &timeStampCoarse);

          // Retrieve current time from timer 2 (TIME).
          halTimer2GetFullCurrentTime(&fine, &coarse);

          //  Find time delta between SOP and TIME (32 MHz ticks). This calcuation
          //  assumes that the timer2 base period is set to 1 ms (0x7D00). 
          coarse -= timeStampCoarse;
          if( fine < timeStampFine ) {
            fine += (0x7D00 - timeStampFine);
            coarse--;
          }
          else {
            fine -=  timeStampFine;
          }
          
          //  Convert the 32 MHz ticks to 32 kHz ticks. The conversion has some
          //  inaccuracy as it ignores the low speed clock setting. The CC2541
          //  and CC2545 also has an optional low speed crystal oscillator which
          //  has a slightly different frequency than the RC oscillator:
          //  LS-XOSC: 32.768 kHz
          //  LS-RCOSC: 32.753 kHz.
          
          //subtract the time difference between the last rec packet and now
          //from the sleep duration
          sleepDuration -= (uint32) ((fine/977) + (32*coarse) + 32);
        }
        
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

            // CC2541/45 has automatic sync between 32 kHz ST and timer2.
            halTimer2Stop(1);

            // Disable the sleep timer interrupt.
            halSleepEnableInterrupt();

            // Make Sure Global Interrupt is enabled.
            halIntOn();

            // Set LED3 indicate entering power mode function.
            //halLedSet(3);

            // No more pillow fights, time for bed.
            halSleepEnterPowerMode(powerModeFlag);

            // Clear LED3 indicate exit from power mode function.
            //halLedClear(3);

            // Disable the sleep timer interrupt.
            halSleepDisableInterrupt();

            // CC2541/45 has automatic sync between 32 kHz ST and timer2.
            halTimer2Start(1);//halTimer2Start(1);
        }
#endif
}

void Transmit(uint8 messageType, uint8 sequenceNumber, uint8 partnerNumber, uint8 partnerSequenceNumber, uint8* message)
{
  rfirqf1 = 0;
  
  uint8 payload[PAYLOAD_LENGTH];
  
  for(int i = 0;i< PAYLOAD_LENGTH; i++)    
    payload[i] = 0;
  
  payload[0] = messageType;
  payload[1] = sequenceNumber;
  payload[2] = partnerNumber;
  payload[3] = partnerSequenceNumber;
  
  for(int i = 4; i < PAYLOAD_LENGTH; i++)
  {
    payload[i] = message[i-4];
  }
  
  unsigned char addressBytes[6];
  addressBytes[0] = ADDRESS_LOW & 0xFF; // Address (LSB)
  addressBytes[1] = (ADDRESS_LOW >> 8) & 0xFF;
  addressBytes[2] = (ADDRESS_LOW >> 16) & 0xFF;
  addressBytes[3] = (ADDRESS_LOW >> 24) & 0xFF;
  addressBytes[4] = ADDRESS_HIGH & 0xFF;
  addressBytes[5] = (ADDRESS_HIGH >> 8) & 0xFF; // Address (MSB)  
  
  //obtainSem0();
  halRfBroadcastLoadPacket(payload, PAYLOAD_LENGTH, addressBytes);
  //releaseSem0();
  
  // Start transmitter.
  //while(RFST != 0);
  //RFST = CMD_TX;
  if(RFST != 0)
  {
    halRfCommand(CMD_SHUTDOWN);
    while (!(rfirqf1 & RFIRQF1_TASKDONE)){}
    rfirqf1 = 0;
  }
  halRfStartTx();
  
  // Wait for TASKDONE and halt CPU (PM0) until task is completed.
  while (!( rfirqf1 & RFIRQF1_TASKDONE)) {}     

  // If data received read FIFO   
  if(PRF.ENDCAUSE == TASK_ENDOK)    
  {    
    //Get packet data.
    //halRfLoadBLEBroadcastPacketPayload();
    //PRINTF("Tx Ok\n");
    //txNumber++;
    //PRINTF("TX: %d, RX: %d\n",txNumber,rxNumber);
  } else {
    PRINTF("ENDCAUSE: %d\n",PRF.ENDCAUSE);
    PRINTF("Tx NOk\n");
  }  
  halRfCommand(CMD_TXFIFO_RESET);

  rfirqf1 = 0;  
}

void Receive()
{
  // If data received read FIFO   
  if(PRF.ENDCAUSE == TASK_ENDOK)    
  {
    if(rfirqf1 & RFIRQF1_RXOK) {
      uint8 addressBytes[6];     
      uint8 payload[PAYLOAD_LENGTH];
      GetMessagePayload(addressBytes,payload);
    
      uint8 messageType = payload[0];
      uint8 sequenceNumber = payload[1];
      uint8 partnerNumber = payload[2];
      uint8 partnerSequenceNumber = payload[3];
      
      uint8 message[PAYLOAD_LENGTH-4];
      for(int i = 4; i < PAYLOAD_LENGTH; i++)
      {
        message[i-4] = payload[i];
      }
      
#if(MODETX)
      if(messageType == 0) // message from the sender itself
      {
        PRINTF("R\n");
        Transmit(1,0,OTHERNUMBER,sequenceNumber,message);
      }
#else
      deviceMap* device;
      if(messageType == 0)
      {
        device = GetDeviceByAddress(addressBytes);
        if(!device)
          IncludeDevice(addressBytes,numberOfDevices+1,sequenceNumber);
        else
        {
          if(sequenceNumber != device->sequenceNumber+1)
          {
            int diff = sequenceNumber - (device->sequenceNumber+1);
            if(diff > 0)
            {
              device->totalPackages = device->totalPackages + diff;
              device->packageLosses = device->packageLosses + diff;
              PRINTF("%x %lu %lu\n", device->address[0], (unsigned long)device->packageLosses, (unsigned long)device->totalPackages);
            }
            else if(diff < 0)
            {
              device->packageLosses = 0;
              device->totalPackages = 1;
              device->sequenceNumber = sequenceNumber;
              PRINTF("%x RENEW\n", device->address[0]);
            }
          }
          else
          {
            device->totalPackages = device->totalPackages + 1;
          }
          device->sequenceNumber = sequenceNumber;
          //PRINTF("OWN - %x - %d\n", addressBytes[0], sequenceNumber);
        }
      }
      else if(messageType == 1 || messageType == 2)
      {
        device = GetDeviceByNumber(partnerNumber);
        if(!device)
          IncludeDevice(addressBytes,numberOfDevices+1,partnerSequenceNumber);
        else
        {
          if(partnerSequenceNumber != device->sequenceNumber+1)
          {
            int diff = partnerSequenceNumber - (device->sequenceNumber+1);
            if(diff > 0)
            {
              device->totalPackages = device->totalPackages + diff;
              device->packageLosses = device->packageLosses + diff;
              PRINTF("%x %lu %lu\n", device->address[0], (unsigned long)device->packageLosses, (unsigned long)device->totalPackages);
            }
            else if(diff < 0)
            {
              device->packageLosses = 0;
              device->totalPackages = 1;
              device->sequenceNumber = partnerSequenceNumber;
              PRINTF("%x RENEW\n", device->address[0]);
            }
          }
          else
          {
            device->totalPackages = device->totalPackages + 1;
          }
          device->sequenceNumber = partnerSequenceNumber;
          //PRINTF("OTHER - %x - %d\n", addressBytes[0], partnerSequenceNumber);
        }
      }
      //halRfLoadBLEBroadcastPacketPayload(1);
#endif
    } else if(rfirqf1 & RFIRQF1_RXNOK) {
      //PRINTF("NOk ");
      //halRfLoadBLEBroadcastPacketPayload(1);          
    } else {
      PRINTF("*");
    }        
    halRfCommand(CMD_RXFIFO_RESET);
    rfirqf1 = 0;
  }
  else if((PRF.ENDCAUSE == TASK_RXTIMEOUT))
  {
    PRINTF("Timeout\n");
  }
  else 
  {
    PRINTF("- ");
    PRINTF("PRF.ENDCAUSE: %d\n", PRF.ENDCAUSE);
  }
  halRfCommand(CMD_RXFIFO_RESET);
}

void TurnLED(uint8 led)
{
  if(led)
    MCU_IO_OUTPUT(1, 2, 1);
  else
    MCU_IO_INPUT(1, 2, MCU_IO_PULLDOWN);
}

// Timer 1 Interrupt routine (every 1 ms)
HAL_ISR_FUNCTION(T1_ISR,T1_VECTOR) 
{
  if(counter == randomTime) // Set boolean to start transmit function
  {
    isTimeToTransmit = 1;
  }
  else if(counter > TOTAL_TIME)
  {
    ledStatus = !ledStatus;
    TurnLED(ledStatus);
    randomTime = rand() % TOTAL_TIME; // New random time to transmit
    counter = 0;
    isTimeToTransmit = 0;
  }
  IRCON &= 0xFD; // Clean interrupt flag
  counter++;
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
    
    PRINTF("Init\n");
    
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
#if(MODETX)
    //Set Timer 1 to interrupt every 1 ms
    ConfigureTimer();
    
    //Get Timer2 current time for seed initialization
    uint16 fine;
    uint32 coarse;
    halTimer2GetFullCurrentTime(&fine, &coarse);
    srand(coarse); //Random seed initialization
    
    randomTime = rand() % TOTAL_TIME; //Random initial time to transmit

    while(1)  
    {
      rfirqf1 = 0;
   
      if(isTimeToTransmit)
      {        
        char exampleMessage[22] = "MENSAGEM EXEMPLO";
        isTimeToTransmit = 0;
        Transmit(0,myNumber,0,0,(uint8*)exampleMessage);
        myNumber++;
      }
      else
      {
        // Start receiver.
        halRfStartRx();
        // Wait for TASKDONE and halt CPU (PM0) until task is completed.
        while ((!(rfirqf1 & RFIRQF1_TASKDONE)) && !(isTimeToTransmit)) {}  
       
        if(isTimeToTransmit)
        {
          rfirqf1 = 0;
          halRfCommand(CMD_SHUTDOWN);
          while (!(rfirqf1 & RFIRQF1_TASKDONE)){}
        }
        else
        {
          Receive();
        }
      }
      
      rfirqf1 = 0; // Clear RF interrupts.
      
      obtainSem0();
      PRF.ENDCAUSE = TASK_UNDEF;
      releaseSem0();
    }
#else
    uint8 address1[6] = {0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};
    uint8 address2[6] = {0xBB,0xBB,0xBB,0xBB,0xBB,0xBB};
    IncludeDevice(address1,1,0);
    IncludeDevice(address2,2,0);
    while(1)  
    {
      rfirqf1 = 0; 

      // Start receiver.
      halRfStartRx();
      // Wait for TASKDONE and halt CPU (PM0) until task is completed.
      while ((!(rfirqf1 & RFIRQF1_TASKDONE)) && !(isTimeToTransmit)) {}  
      
      Receive();
      
      rfirqf1 = 0;
      
      obtainSem0();
      PRF.ENDCAUSE = TASK_UNDEF;
      releaseSem0();
    }
#endif
}