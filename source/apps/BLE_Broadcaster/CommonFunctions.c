#include "CommonFunctions.h"
#include "BLE_Broadcaster_cc254x.h"
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"

void CopyMessage(uint8* destiny, uint8* source)
{
  for(int i = 0; i < PAYLOAD_LENGTH-1; i++)
    destiny[i] = source[i];
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