#include "NodeFunctions.h"
#include "BLE_Broadcaster_cc254x.h"
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"

uint8** codingMatrix;
uint8** resultMatrix;

void CopyMessage(uint8* destiny, uint8* source)
{
  for(int i = 0; i < PAYLOAD_LENGTH-1; i++)
    destiny[i] = source[i];
}

uint8* PackMessage(uint32 seqNumber, char* message)
{
  uint8* newMessage = (uint8*)malloc(PAYLOAD_LENGTH-1*sizeof(uint8));
  newMessage[0] = (uint8) seqNumber;
  for(int i = 1; i < PAYLOAD_LENGTH-1; i++)
    newMessage[i] = (uint8) message[i-1];
  return newMessage;
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
      
      if(messageType < 3 && messageType != NODE_NUMBER)
      {
        CopyMessage(messages[messageType],message);
      }

    }
  }
  
  halRfCommand(CMD_RXFIFO_RESET);
  rfirqf1 = 0;
}

uint8** MultiplyMatrix(uint8 matrix1[3][PAYLOAD_LENGTH-1], int matrix1Lines, int matrix1Columns, uint8** matrix2, int matrix2Lines, int matrix2Columns)
{
  uint8** resultMatrix;
  if(matrix1Lines == matrix2Lines) //Matrix1 is transverse
  {
    resultMatrix = (uint8**)malloc(matrix1Columns*matrix2Columns*sizeof(uint8));
    for(int i = 0; i < matrix1Columns; i++)
    {
      for(int j = 0; j < matrix2Columns; j++)
      {
        uint8 sum = 0;
        for(int k = 0; k < matrix1Lines; k++)
        {
          sum += (matrix1[k][i] *matrix2[k][j]);
        }
        resultMatrix[i][j] = sum;
      }
    }
  }
  return resultMatrix;
}
  
uint8* GetMatrixColumn(uint8** matrix, int matrixLines, int columnNumber)
{
  uint8* column = malloc(matrixLines*sizeof(uint8));
  for(int i = 0; i < matrixLines; i++)
  {
      column[i] = matrix[i][columnNumber];
  }
    return column;
}

void DAF2(uint8 phase, uint32 myNumber)
{
  uint8* column;
  switch(phase)
  {
  case 0:
#if NODE_NUMBER == 0
    Transmit(0,messages[NODE_NUMBER]);
    Receive();
#elif NODE_NUMBER == 1
    Receive();
#endif
    break;
  case 1:
#if NODE_NUMBER == 0
    Receive();
#elif NODE_NUMBER == 1
    Transmit(1,messages[NODE_NUMBER]);
    Receive();
#endif
    break;
  case 2:
#if NODE_NUMBER == 0
    resultMatrix = MultiplyMatrix(messages,2,PAYLOAD_LENGTH - 1,codingMatrix,2,4);
    column = GetMatrixColumn(resultMatrix,PAYLOAD_LENGTH - 1,2);
    Transmit(1,column);
    free(column);
    Receive();
#elif NODE_NUMBER == 1
    Receive();
#endif
    break;
  case 3:
#if NODE_NUMBER == 0
    Receive();
#elif NODE_NUMBER == 1
    resultMatrix = MultiplyMatrix(messages,2,PAYLOAD_LENGTH - 1,codingMatrix,2,4);
    column = GetMatrixColumn(resultMatrix,PAYLOAD_LENGTH - 1,3);
    Transmit(1,column);
    free(column);
    Receive();
#endif
    break;
  }
}

void NodeSetup(uint32 myNumber)
{
  char exampleMessage[PAYLOAD_LENGTH-2] = "MENSAGEM EXEMPLO";
  uint8* fullMessage = PackMessage(myNumber,exampleMessage);
  CopyMessage(messages[NODE_NUMBER],fullMessage);
  free(fullMessage);
#if OPERATION_MODE == DAF  
#if TOTAL_NODES == 2
  codingMatrix = (uint8**)malloc(2*4*sizeof(uint8));
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

void NodeClean()
{
  free(codingMatrix);
  free(resultMatrix);
}

void NodeRun(uint8 phase, uint32 myNumber)
{
#if OPERATION_MODE == DAF
#if TOTAL_NODES == 2
  DAF2(phase,myNumber);
#elif TOTAL_NODES == 3
  DAF3(phase);
#endif
#elif OPERATION_MODE == BNC
#if TOTAL_NODES == 2
  BNC2(phase);
#elif TOTAL_NODES == 3
  BNC3(phase);
#endif
#elif OPERATION_MODE == DNC
#if TOTAL_NODES == 2
  DNC2(phase);
#elif TOTAL_NODES == 3
  DNC3(phase);
#endif
#elif OPERATION_MODE == GDNC
#if TOTAL_NODES == 2
  GDNC2(phase);
#elif TOTAL_NODES == 3
  GDNC3(phase);
#endif
#endif
}