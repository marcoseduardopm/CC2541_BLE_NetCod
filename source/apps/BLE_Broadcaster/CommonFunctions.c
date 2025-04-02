#include "CommonFunctions.h"
#include "BLE_Broadcaster_cc254x.h"
#include <stdio.h>


/*void PrintMatrix(uint8* matrix, int lines, int columns)
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
}*/

void TurnLED(uint8 led)
{
  if(led)
    MCU_IO_OUTPUT(1, 2, 1);
  else
    MCU_IO_INPUT(1, 2, MCU_IO_PULLDOWN);
}

void IncludeDevices()
{   
  for(int i = 0; i < 6; i++)
    deviceList[0].address[i] = 0xAA;
  deviceList[0].number = 0;
  deviceList[0].receivedSequenceNumber = 0;
  deviceList[0].expectedSequenceNumber = 0;
  deviceList[0].totalPackages = 0;
  deviceList[0].packageLosses = 0; 
  
  for(int i = 0; i < 6; i++)
    deviceList[1].address[i] = 0xBB;
  deviceList[1].number = 1;
  deviceList[1].receivedSequenceNumber = 0;
  deviceList[1].expectedSequenceNumber = 0;
  deviceList[1].totalPackages = 0;
  deviceList[1].packageLosses = 0;
#if TOTAL_NODES == 3
  for(int i = 0; i < 6; i++)
    deviceList[2].address[i] = 0xCC;
  deviceList[2].number = 2;
  deviceList[2].receivedSequenceNumber = 0;
  deviceList[2].expectedSequenceNumber = 0;
  deviceList[2].totalPackages = 0;
  deviceList[2].packageLosses = 0;
#endif
      
  for(int i = 0; i < TOTAL_TRANSMISSIONS; i++)
  {
      deviceList[0].receivedMessages[i] = 0;
      deviceList[1].receivedMessages[i] = 0;
#if TOTAL_NODES == 3
      deviceList[2].receivedMessages[i] = 0;
#endif
  }
}

void ClearMessages()
{
  for(int i = 0; i < COLS; i ++)
  {
    for(int j = 0; j < (PAYLOAD_LENGTH-2)/2; j++)
      messages[i][j] = 0;
  }
  for(int i = 0; i < ROWS; i++)
  {
    for (int j = 0; j < (PAYLOAD_LENGTH-2)/2; j++)
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
  uint8 partialMatrix[9][3] = {1,0,0,0,1,0,0,0,1,0,1,0,0,0,1,1,0,0,0,0,1,1,0,0,0,1,0};
  for(int i = 0; i < 9; i++)
  {
    for(int j = 0; j < 3; j ++)
      codingMatrix[i][j] = partialMatrix[i][j];
  }
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
  uint8 partialMatrix[9][3] = {1,0,0,0,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
  for(int i = 0; i < 9; i++)
  {
    for(int j = 0; j < 3; j ++)
      codingMatrix[i][j] = partialMatrix[i][j];
  }
#endif
#elif OPERATION_MODE == DNC
#if TOTAL_NODES == 2
  uint8 partialMatrix[4][2] = {1,0,0,1,1,1,1,2};
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 2; j ++)
      codingMatrix[i][j] = partialMatrix[i][j];
  }
#elif TOTAL_NODES == 3
  uint8 partialMatrix[9][3] = {1,0,0,0,1,0,0,0,1,1,1,5,1,2,4,2,3,1,3,13,1,4,11,15,7,5,14};
  for(int i = 0; i < 9; i++)
  {
    for(int j = 0; j < 3; j ++)
      codingMatrix[i][j] = partialMatrix[i][j];
  }
#endif
#elif OPERATION_MODE == GDNC
#if TOTAL_NODES == 2
  uint8 partialMatrix[4][2] = {1,0,0,1,3,2,2,3};
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 2; j ++)
      codingMatrix[i][j] = partialMatrix[i][j];
  }
#elif TOTAL_NODES == 3
  uint8 partialMatrix[9][3] = {1,0,0,0,1,0,0,0,1,15,8,6,11,5,15,1,1,1,14,8,7,5,13,9,11,4,14};
  for(int i = 0; i < 9; i++)
  {
    for(int j = 0; j < 3; j ++)
      codingMatrix[i][j] = partialMatrix[i][j];
  }
#endif
#endif
}

void Transmit(uint8 messageType, uint8 mask, uint16* message)
{
  rfirqf1 = 0;
  
  uint8 payload[PAYLOAD_LENGTH];
  
  for(int i = 0;i< PAYLOAD_LENGTH; i++)    
    payload[i] = 0;
  
  payload[0] = messageType;
  payload[1] = mask;
  
  for(int i = 2; i < PAYLOAD_LENGTH; i+=2)
  {
    uint8 messageHigh = message[(i-2)/2] >> 8; 
    uint8 messageLow = message[(i-2)/2] & 0xFF;
    payload[i] = messageHigh;
    payload[i+1] = messageLow;
  }
  
  halRfBroadcastLoadPacket(payload, PAYLOAD_LENGTH, addressBytes);
  
  // Start transmitter.  
  halRfStartTx();
  
  // Wait for TASKDONE and halt CPU (PM0) until task is completed.
  while (!( rfirqf1 & RFIRQF1_TASKDONE)) {}
  
  halRfCommand(CMD_TXFIFO_RESET);

  rfirqf1 = 0;  
}
