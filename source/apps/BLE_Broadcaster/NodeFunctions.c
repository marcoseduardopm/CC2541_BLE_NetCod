#include "NodeFunctions.h"
#include "CommonFunctions.h"
#include "BLE_Broadcaster_cc254x.h"
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"

#ifdef MODETX

uint8* PackMessage(uint32 seqNumber, char* message)
{
  uint8* newMessage = (uint8*)malloc(PAYLOAD_LENGTH-1*sizeof(uint8));
  newMessage[0] = (uint8) seqNumber;
  for(int i = 1; i < PAYLOAD_LENGTH-1; i++)
    newMessage[i] = (uint8) message[i-1];
  return newMessage;
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
      
      if(messageType <= 2 && messageType != NODE_NUMBER)
      {
        CopyMessage(messages[messageType],message);
        messagesFlags[messageType] = 1;
      }
      else if(messageType == 255)
      {
        phase = 0;
        myNumber = 0;
        transmissionDone = 1;
      }

    }
  }
  
  halRfCommand(CMD_RXFIFO_RESET);
  rfirqf1 = 0;
}

void MultiplyMatrix(int matrix1Lines, int matrix1Columns, int matrix2Lines, int matrix2Columns)
{
  if(matrix1Lines == matrix2Lines) //Matrix1 is transverse
  {
    for(int i = 0; i < matrix1Columns; i++)
    {
      for(int j = 0; j < matrix2Columns; j++)
      {
        uint8 sum = 0;
        for(int k = 0; k < matrix1Lines; k++)
        {
          sum += (messages[k][i] * codingMatrix[k][j]);
        }
        resultMatrix[i][j] = sum;
      }
    }
  }
}
  
uint8* GetMatrixColumn( int matrixLines, int columnNumber)
{
  uint8* column = malloc(matrixLines*sizeof(uint8));
  for(int i = 0; i < matrixLines; i++)
  {
      column[i] = resultMatrix[i][columnNumber];
  }
    return column;
}

void DAF2()
{
  uint8* column;
#if NODE_NUMBER == 0
 if(phase == 0)
 {
  Transmit(0,messages[0]);
 }
 else if(phase == 2)
 {
   if(messagesFlags[1])
    {
      MultiplyMatrix(2,PAYLOAD_LENGTH - 1,2,4);
      column = GetMatrixColumn(PAYLOAD_LENGTH - 1,2);
      Transmit(10,column);
      free(column);
    }
    else
    {
      Transmit(0,messages[NODE_NUMBER]);
    }
 }
#elif NODE_NUMBER == 1
   if(phase == 1)
   {
    Transmit(1,messages[1]);
   }
   else if(phase == 3)
   {
     if(messagesFlags[0])
      {
        MultiplyMatrix(2,PAYLOAD_LENGTH - 1,2,4);
        column = GetMatrixColumn(PAYLOAD_LENGTH - 1,3);
        Transmit(11,column);
        free(column);
      }
      else
      {
        Transmit(1,messages[1]);
      }
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