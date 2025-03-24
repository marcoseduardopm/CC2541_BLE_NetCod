#include "DestinyFunctions.h"
#include "CommonFunctions.h"
#include "BLE_Broadcaster_cc254x.h"
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"
#include "stdio.h"

#ifndef MODETX

void PrintMatrix(int lines, int columns)
{
  for(int i = 0; i < lines; i++)
  {
    for(int j = 0; j < columns; j++)
    {
      printf("%02X ",resultMatrix[i][j]);
    }
    printf("\n");
  }
  printf("\n\n\n");
}

void CopyMatrixColumn(uint8* column, int matrixLines, int columnNumber)
{
  for(int i = 0; i < matrixLines; i++)
  {
      resultMatrix[i][columnNumber] = column[i];
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
        CopyMessage(messages[messageType],message);
        CopyMatrixColumn(message,PAYLOAD_LENGTH-1,0);
        messagesFlags[0] = 1;
        break;
      case 1:
        CopyMessage(messages[messageType],message);
        CopyMatrixColumn(message,PAYLOAD_LENGTH-1,1);
        messagesFlags[1] = 1;
        break;
      case 2:
        CopyMessage(messages[messageType],message);
        CopyMatrixColumn(message,PAYLOAD_LENGTH-1,2);
        messagesFlags[2] = 1;
        break;
      case 10:
        CopyMatrixColumn(message,PAYLOAD_LENGTH-1,2);
        break;
      case 11:
        CopyMatrixColumn(message,PAYLOAD_LENGTH-1,3);
        break;
      case 20:
        CopyMatrixColumn(message,PAYLOAD_LENGTH-1,3);
        break;
      case 21:
        CopyMatrixColumn(message,PAYLOAD_LENGTH-1,4);
        break;
      case 22:
        CopyMatrixColumn(message,PAYLOAD_LENGTH-1,5);
        break;
      case 23:
        CopyMatrixColumn(message,PAYLOAD_LENGTH-1,6);
        break;
      case 24:
        CopyMatrixColumn(message,PAYLOAD_LENGTH-1,7);
        break;
      case 25:
        CopyMatrixColumn(message,PAYLOAD_LENGTH-1,8);
        break;
      }

    }
  }
  
  halRfCommand(CMD_RXFIFO_RESET);
  rfirqf1 = 0;
}

void DAF2()
{
  switch(phase)
  {
  case 0:
    Receive();
    break;
  case 1:
    Receive();
    break;
  case 2:
    Receive();
    break;
  case 3:
    Receive();
    break;
  }
}

void DestinySetup()
{
#if OPERATION_MODE == DAF
#if TOTAL_NODES == 2
  uint8 partialMatrix[2][4] = {1,0,0,1,0,1,1,0};
  for(int i = 0; i < 2; i++)
  {
    for(int j = 0; j < 4; j ++)
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