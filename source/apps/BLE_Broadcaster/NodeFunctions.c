#include "NodeFunctions.h"
#include "CommonFunctions.h"
#include "BLE_Broadcaster_cc254x.h"
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"
#include <stdio.h>

#ifdef MODETX

uint8* PackMessage(uint32 seqNumber, char* message)
{
  uint8* newMessage = (uint8*)malloc(PAYLOAD_LENGTH-2*sizeof(uint8));
  newMessage[0] = (uint8) seqNumber;
  for(int i = 1; i < PAYLOAD_LENGTH-2; i++)
    newMessage[i] = (uint8) message[i-1];
  return newMessage;
}

void CopyMessage(uint8* destiny, uint8* source)
{
 for(int i = 0; i < PAYLOAD_LENGTH-2; i++)
   destiny[i] = source[i];
}


uint8 ReceiveInitSignal()
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
      
      if(messageType == 255)
      {
        phase = 0;
        myNumber = 0;
        halRfCommand(CMD_RXFIFO_RESET);
        rfirqf1 = 0;
        return 1;
      }
    }
  }
  
  halRfCommand(CMD_RXFIFO_RESET);
  rfirqf1 = 0;
  return 0;
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
      uint8 mask = payload[1];
      
      uint8 message[PAYLOAD_LENGTH-2];
      
      for(int i = 2; i < PAYLOAD_LENGTH; i++)
      {
        message[i-2] = payload[i];
      }
      if(messageType != NODE_NUMBER)
      {
        switch(messageType)
        {
        case 0:
          CopyMessage(messages[0],message);
          messagesFlags[0] = 1;
          if(NODE_NUMBER == 1)
          {
             //if(phase >= 2)
               //transmissionDone = 1;
             phase = 0;
             counter = (TOTAL_TIME/2 - TOTAL_TIME/3);
          }
          else if(NODE_NUMBER == 2)
          {
             phase = 0;
             counter = (TOTAL_TIME/2 - TOTAL_TIME/6);
          }
          break;
        case 1:
          CopyMessage(messages[1],message);
          messagesFlags[1] = 1;
          break;
        case 2:
          CopyMessage(messages[2],message);
          messagesFlags[2] = 1;
          break;
        case 3:
          CopyMessage(messages[0],message);
          messagesFlags[0] = 1;
          break;
        case 4:
          CopyMessage(messages[1],message);
          messagesFlags[1] = 1;
          break;
        case 5:
          CopyMessage(messages[2],message);
          messagesFlags[2] = 1;
          break;
        case 255:
          phase = 0;
          myNumber = 0;
          transmissionDone = 1;
          break;
        }
      }
    }
  }
  
  halRfCommand(CMD_RXFIFO_RESET);
  rfirqf1 = 0;
}

void MultiplyMatrix(int matrix1Lines, int matrix1Columns, int matrix2Lines, int matrix2Columns)
{
  if(matrix1Columns == matrix2Lines) 
  {
    for(int i = 0; i < matrix1Lines; i++)
    {
      for(int j = 0; j < matrix2Columns; j++)
      {
        uint8 sum = 0;
        for(int k = 0; k < matrix2Lines; k++)
        {
          sum += (codingMatrix[i][k] * messages[k][j]);
        }
        resultMatrix[i][j] = sum;
      }
    }
  }
}

void DAF2()
{
  uint8* line;
  uint8 mask = 0xFF;
#if NODE_NUMBER == 0
 if(phase == 0)
 {
  if(!messageSent)
  {
    Transmit(0,mask,messages[0]);
    messageSent = 1;
  }
 }
 else if(phase == 2)
 {
   if(messagesFlags[1])
    {
      MultiplyMatrix(4,2,2,PAYLOAD_LENGTH-2);
      line = resultMatrix[2];
      Transmit(10,mask,line);
    }
    else
    {
      Transmit(3,mask,messages[NODE_NUMBER]);
    }
 }
#elif NODE_NUMBER == 1
   if(phase == 1)
   {
     if(!messageSent)
    {
      Transmit(1,mask,messages[1]);
      messageSent = 1;
    }
   }
   else if(phase == 3)
   {
     if(messagesFlags[0])
      {
        MultiplyMatrix(4,2,2,PAYLOAD_LENGTH-2);
        line = resultMatrix[3];
        Transmit(11,mask,line);
      }
      else
      {
        Transmit(4,mask,messages[1]);
      }
   }
#endif
}


void BNC2()
{
  uint8* line;
  uint8 mask = 0xFF;
#if NODE_NUMBER == 0
 if(phase == 0)
 {
  if(!messageSent)
  {
    Transmit(0,mask,messages[0]);
    messageSent = 1;
  }
 }
 else if(phase == 2)
 {
   if(!messagesFlags[1])
      mask &= 0xFD;
   MultiplyMatrix(4,2,2,PAYLOAD_LENGTH-2);
   line = resultMatrix[2];
   Transmit(10,mask,line);
 }
#elif NODE_NUMBER == 1
   if(phase == 1)
   {
     if(!messageSent)
    {
      Transmit(1,mask,messages[1]);
      messageSent = 1;
    }
   }
   else if(phase == 3)
   {
     if(!messagesFlags[0])
        mask &= 0xFE;
     MultiplyMatrix(4,2,2,PAYLOAD_LENGTH-2);
     line = resultMatrix[3];
     Transmit(11,mask,line);
   }
#endif
}

void NodeSetup()
{
#if NODE_NUMBER == 0
  char exampleMessage[PAYLOAD_LENGTH-2] = "MENSAGEM EXEMPLO";
#elif NODE_NUMBER == 1
  char exampleMessage[PAYLOAD_LENGTH-2] = "OUTRA COISA";
#elif NODE_NUMBER == 2
  char exampleMessage[PAYLOAD_LENGTH-2] = "TERCEIRO NO";
#endif
  
  uint8* fullMessage = PackMessage(myNumber,exampleMessage);
  CopyMessage(messages[NODE_NUMBER],fullMessage);
  free(fullMessage);

  CodingMatrixConfig();
}

void NodeRun()
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