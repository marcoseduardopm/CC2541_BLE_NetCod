#include "CommonFunctions.h"
#include "BLE_Broadcaster_cc254x.h"
#include <stdio.h>

void PrintMatrix(uint8* matrix, int lines, int columns)
{
  for(int i = 0; i < lines; i++)
  {
    for(int j = 0; j < columns; j++)
    {
      printf("%02X ",matrix[i * columns + j]);
    }
    printf("\n");
  }
  printf("\n\n\n");
}

void TurnLED(uint8 led)
{
  if(led)
    MCU_IO_OUTPUT(1, 2, 1);
  else
    MCU_IO_INPUT(1, 2, MCU_IO_PULLDOWN);
}

void IncludeDevices()
{
  deviceMap nodeA, nodeB, nodeC;
  
  for(int i = 0; i < 6; i++)
    nodeA.address[i] = 0xAA;
  nodeA.number = 0;
  nodeA.sequenceNumber = 0;
  nodeA.totalPackages = 0;
  nodeA.packageLosses = 0;
  
  for(int i = 0; i < 6; i++)
    nodeB.address[i] = 0xBB;
  nodeB.number = 1;
  nodeB.sequenceNumber = 0;
  nodeB.totalPackages = 0;
  nodeB.packageLosses = 0;
  
  for(int i = 0; i < 6; i++)
    nodeC.address[i] = 0xCC;
  nodeC.number = 2;
  nodeC.sequenceNumber = 0;
  nodeC.totalPackages = 0;
  nodeC.packageLosses = 0;

  deviceList[0] = nodeA;
  deviceList[1] = nodeB;
  deviceList[2] = nodeC;
}

void ClearMessages()
{
  for(int i = 0; i < 3; i ++)
  {
    for(int j = 0; j < PAYLOAD_LENGTH - 1; j++)
      messages[i][j] = 0;
  }
  for(int i = 0; i < ROWS; i++)
  {
    for (int j = 0; j < PAYLOAD_LENGTH-1; j++)
      resultMatrix[i][j] = 0;
  }
  for(int i = 0; i < 9; i ++)
    messagesFlags[i] = 0;
}

void GetMessagePayload(uint8* outputAddress, uint8* outputData)
{
  if(RFRXFLEN > PAYLOAD_LENGTH)
  {
    RFD;RFD;RFD;
    for(int i = 0; i < 6; i++){
      outputAddress[i] = RFD;
    }
    RFD;RFD;RFD;RFD;RFD;
    for(int i = 0; i < PAYLOAD_LENGTH; i++){
      outputData[i] = RFD;
    }
  }
}

void CodingMatrixConfig()
{
    
#if OPERATION_MODE == DAF  
#if TOTAL_NODES == 2
  uint8 partialMatrix[4][2] = {1,0,0,1,0,1,1,0};
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 2; j ++)
      codingMatrix[i][j] = partialMatrix[i][j];
  }
#elif TOTAL_NODES == 3
  
#endif
#elif OPERATION_MODE == BNC
#if TOTAL_NODES == 2
  uint8 partialMatrix[4][2] = {1,0,0,1,1,1,1,1};
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 2; j ++)
      codingMatrix[i][j] = partialMatrix[i][j];
  }
#elif TOTAL_NODES == 3
  
#endif
#elif OPERATION_MODE == DNC
#if TOTAL_NODES == 2
  
#elif TOTAL_NODES == 3
  
#endif
#elif OPERATION_MODE == GDNC
#if TOTAL_NODES == 2
  
#elif TOTAL_NODES == 3
  
#endif
#endif
}

void Transmit(uint8 messageType, uint8* message)
{
  rfirqf1 = 0;
  
  uint8 payload[PAYLOAD_LENGTH];
  
  for(int i = 0;i< PAYLOAD_LENGTH; i++)    
    payload[i] = 0;
  
  payload[0] = messageType;
  
  for(int i = 1; i < PAYLOAD_LENGTH; i++)
  {
    payload[i] = message[i-1];
  }
  
  halRfBroadcastLoadPacket(payload, PAYLOAD_LENGTH, addressBytes);
  
  // Start transmitter.  
  halRfStartTx();
  
  // Wait for TASKDONE and halt CPU (PM0) until task is completed.
  while (!( rfirqf1 & RFIRQF1_TASKDONE)) {}
  
  halRfCommand(CMD_TXFIFO_RESET);

  rfirqf1 = 0;  
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