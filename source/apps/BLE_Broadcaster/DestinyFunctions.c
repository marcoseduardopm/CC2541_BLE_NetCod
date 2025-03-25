#include "DestinyFunctions.h"
#include "CommonFunctions.h"
#include "BLE_Broadcaster_cc254x.h"
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"
#include "stdio.h"

#ifndef MODETX

void CopyMatrixLine(uint8* line, int matrixColumns, int lineNumber)
{
  for(int i = 0; i < matrixColumns; i++)
  {
      resultMatrix[lineNumber][i] = line[i];
  }
}

void Receive()
{
  rfirqf1 = 0;  
  halRfStartRx();
  
  while (!(rfirqf1 & RFIRQF1_TASKDONE)) {}  
  
  // If data received read FIFO   
  if(PRF.ENDCAUSE == TASK_ENDOK)    
  {
    if(rfirqf1 & RFIRQF1_RXOK) {
      
      uint8 addressBytes[6];     
      uint8 payload[PAYLOAD_LENGTH];
      
      GetMessagePayload(addressBytes,payload);
    
      uint8 messageType = payload[0];
      
      uint8 message[PAYLOAD_LENGTH-1];
      
      for(int i = 1; i < PAYLOAD_LENGTH; i++)
      {
        message[i-1] = payload[i];
      }
    
      switch(messageType)
      {
      case 0:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,0);
        messagesFlags[0] = 1;
        counter = TOTAL_TIME/2;
        phase = 0;
        messageCounter++;
        break;
      case 1:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,1);
        messagesFlags[1] = 1;
        counter = TOTAL_TIME/3;
        phase = 1;
        messageCounter++;
        break;
      case 2:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,2);
        messagesFlags[2] = 1;
        counter = TOTAL_TIME/6;
        phase = 2;
        break;
      case 3:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,0);
        messagesFlags[0] = 1;
        break;
      case 4:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,1);
        messagesFlags[1] = 1;
        break;
      case 5:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,2);
        messagesFlags[2] = 1;
        break;
      case 10:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,2);
        counter = TOTAL_TIME/2;
        messagesFlags[2] = 1;
        phase = 2;
        messageCounter++;
        break;
      case 11:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,3);
        counter = TOTAL_TIME/3;
        messagesFlags[3] = 1;
        phase = 3;
        messageCounter++;
        break;
      case 20:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,3);
        counter = TOTAL_TIME/2;
        messagesFlags[3] = 1;
        phase = 3;
        break;
      case 21:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,4);
        counter = TOTAL_TIME/2;
        messagesFlags[4] = 1;
        phase = 4;
        break;
      case 22:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,5);
        counter = TOTAL_TIME/3;
        messagesFlags[5] = 1;
        phase = 5;
        break;
      case 23:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,6);
        counter = TOTAL_TIME/3;
        messagesFlags[6] = 1;
        phase = 6;
        break;
      case 24:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,7);
        counter = TOTAL_TIME/6;
        messagesFlags[7] = 1;
        phase = 7;
        break;
      case 25:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,8);
        counter = TOTAL_TIME/6;
        messagesFlags[8] = 1;
        phase = 8;
        break;
      }

    }
  }
  
  halRfCommand(CMD_RXFIFO_RESET);
  rfirqf1 = 0;
}

void DAF2()
{
    Receive();
}

void DestinySetup()
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

void DestinyRun()
{
  if(!actedThisPhase)
  {
    actedThisPhase = 1;
#if OPERATION_MODE == DAF
#if TOTAL_NODES == 2
  DAF2();
#elif TOTAL_NODES == 3
  DAF3();
#endif
#elif OPERATION_MODE == BNC
#if TOTAL_NODES == 2
  BNC2();
#elif TOTAL_NODES == 3
  BNC3();
#endif
#elif OPERATION_MODE == DNC
#if TOTAL_NODES == 2
  DNC2();
#elif TOTAL_NODES == 3
  DNC3();
#endif
#elif OPERATION_MODE == GDNC
#if TOTAL_NODES == 2
  GDNC2();
#elif TOTAL_NODES == 3
  GDNC3();
#endif
#endif
  }
  else
    Receive();
}
#endif