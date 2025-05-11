#include "DestinyFunctions.h"
#include "CommonFunctions.h"
#include "BLE_Broadcaster_cc254x.h"
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"
#include "stdio.h"
#include "ps_inv.h"
#include <math.h>

#ifndef MODETX

uint8 CheckCadastredAddress(uint8* address)
{
  for(int i = 0; i < cols; i++)
  {
    uint8 cadastred = 1;
    for(int j = 0; j < 6; j++)
    {
      if(address[j] != deviceList[i].address[j])
        cadastred = 0;
    }
    if(cadastred)
      return 1;
  }
  return 0;
}

void InvertMatrix(double *A, double *A_inv)
{
    double U[MAX_ROWS][MAX_COLS];                                                        
    double V[MAX_COLS][MAX_COLS];                                                        
    double singular_values[MAX_COLS];                                             
    double* dummy_array;      

    dummy_array = (double*) malloc(MAX_COLS * sizeof(double));                    
    if (dummy_array == NULL) { 
        //printf(" No memory available\n"); 
        exit(0); 
    } 

    double err = Singular_Value_Decomposition((double*) A, rows, cols, (double*) U, singular_values, (double*) V, dummy_array);   //

    free(dummy_array);                                                     
    //if (err < 0) 
        //printf("Failed to converge\n");     
                                         
    double tolerance = 0;
    
    Singular_Value_Decomposition_Inverse((double*) U, singular_values, (double*) V,
                    tolerance, rows, cols, (double*) A_inv); 
}

uint8 EmptyLine(uint8* line, int size)
{
  uint8 empty = 1;
  for(int i = 0; i < size; i++)
  {
    if(line[i] != 0)
      empty = 0;
  }
  return empty;
}

void GetResults(int matrix1Lines, int matrix1Columns, int matrix2Lines, int matrix2Columns)
{
  if(matrix1Columns == matrix2Lines) 
  {
    for(int i = 0; i < matrix1Lines; i++)
    {
      for(int j = 0; j < matrix2Columns; j++)
      {
        double sum = 0;
        for(int k = 0; k < matrix2Lines; k++)
        {
          if(totalNodes == 2)
            sum += (inverseCodingMatrix2Nodes[i][k] * (double)resultMatrix[k][j]);
          else if(totalNodes == 3)
            sum += (inverseCodingMatrix3Nodes[i][k] * (double)resultMatrix[k][j]);
          
          //sum += (inverseCodingMatrixDouble[i][k] * (double)resultMatrix[k][j]);
        }
        messages[i][j] = (uint8)(sum + 0.5);
      }
    }
  }
  for(int i = 0; i < cols; i++)
  {
    if(!EmptyLine(messages[i], (PAYLOAD_LENGTH-2)/2))
    {
      uint8 seqNumber = (uint8)messages[i][0];
      deviceList[i].receivedSequenceNumber = seqNumber;
      deviceList[i].receivedMessages[numberOfTransmissions] = seqNumber;
      
      if(seqNumber != deviceList[i].expectedSequenceNumber)
        deviceList[i].packageLosses = deviceList[i].packageLosses + 1;
      else
        deviceList[i].totalPackages++;
    }
    else
        deviceList[i].packageLosses = deviceList[i].packageLosses + 1;
  }
}

void ZeroLine(uint8* line, int size)
{
  for(int i = 0; i < size; i++)
    line[i] = 0;
}

void ZeroField(uint8* line, int size)
{
  for(int i = 0; i < size; i++)
  {
    uint8 bitMask = 1 << i;
    if((bitMask & receivedMask) == 0)
    {
      line[i] = 0;
    }
  }
}

void CopyMatrixLine(uint16* line, int matrixColumns, int lineNumber)
{
  for(int i = 0; i < matrixColumns; i++)
      resultMatrix[lineNumber][i] = line[i];
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
      
      if(GetMessagePayload(addressBytes,payload))
      {
        if(CheckCadastredAddress(addressBytes))
        {        
          uint8 messageType = payload[0];
          receivedMask = payload[1];
          
          uint16 message[(PAYLOAD_LENGTH-2)/2];
          
          for(int i = 2; i < PAYLOAD_LENGTH; i+=2)
          {
            message[(i-2)/2] = (((uint16)payload[i]) << 8) | payload[i+1];
          }
        
          switch(messageType)
          {
          case 0:
            CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,0);
            messagesFlags[0] = 1;
            deviceList[0].numberOfTransmissions = deviceList[0].numberOfTransmissions + 1;
            deviceList[0].rssiSum = deviceList[0].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
            //counter = TOTAL_TIME/2;
            //phase = 0;
            break;
          case 1:
            CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,1);
            messagesFlags[1] = 1;
            deviceList[1].numberOfTransmissions = deviceList[1].numberOfTransmissions + 1;
            deviceList[1].rssiSum = deviceList[1].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
            //counter = TOTAL_TIME/3;
            //phase = 1;
            break;

          case 2:
            if(totalNodes == 3)
            {
              CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,2);
              messagesFlags[2] = 1;
              deviceList[2].numberOfTransmissions = deviceList[2].numberOfTransmissions + 1;
              deviceList[2].rssiSum = deviceList[2].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
              //counter = TOTAL_TIME/6;
              //phase = 2;
              break;
            }
          case 3:
            CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,0);
            messagesFlags[0] = 1;
            deviceList[0].numberOfTransmissions = deviceList[0].numberOfTransmissions + 1;
            deviceList[0].rssiSum = deviceList[0].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
            break;
          case 4:
            CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,1);
            messagesFlags[1] = 1;
            deviceList[1].numberOfTransmissions = deviceList[1].numberOfTransmissions + 1;
            deviceList[1].rssiSum = deviceList[1].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
            break;
          case 5:
            if(totalNodes == 3)
            {
              CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,2);
              messagesFlags[2] = 1;
              deviceList[2].numberOfTransmissions = deviceList[2].numberOfTransmissions + 1;
              deviceList[2].rssiSum = deviceList[2].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
              break;
           }
          case 10:
            CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,2);
            messagesFlags[2] = 1;
            deviceList[0].numberOfTransmissions = deviceList[0].numberOfTransmissions + 1;
            deviceList[0].rssiSum = deviceList[0].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
            if(receivedMask != 0xFF)
              ZeroField(codingMatrix[2],2);
            break;
          case 11:
            CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,3);
            messagesFlags[3] = 1;
            deviceList[1].numberOfTransmissions = deviceList[1].numberOfTransmissions + 1;
            deviceList[1].rssiSum = deviceList[1].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
            if(receivedMask != 0xFF)
              ZeroField(codingMatrix[3],2);
            break;
          case 20:
            CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,3);
            messagesFlags[3] = 1;
            deviceList[0].numberOfTransmissions = deviceList[0].numberOfTransmissions + 1;
            deviceList[0].rssiSum = deviceList[0].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
            if(receivedMask != 0xFF)
              ZeroField(codingMatrix[3],3);
            break;
          case 21:
            CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,4);
            messagesFlags[4] = 1;
            deviceList[0].numberOfTransmissions = deviceList[0].numberOfTransmissions + 1;
            deviceList[0].rssiSum = deviceList[0].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
            if(receivedMask != 0xFF)
              ZeroField(codingMatrix[4],3);
            break;
          case 22:
            CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,5);
            messagesFlags[5] = 1;
            deviceList[1].numberOfTransmissions = deviceList[1].numberOfTransmissions + 1;
            deviceList[1].rssiSum = deviceList[1].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
            if(receivedMask != 0xFF)
              ZeroField(codingMatrix[5],3);
            break;
          case 23:
            CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,6);
            messagesFlags[6] = 1;
            deviceList[1].numberOfTransmissions = deviceList[1].numberOfTransmissions + 1;
            deviceList[1].rssiSum = deviceList[1].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
            if(receivedMask != 0xFF)
              ZeroField(codingMatrix[6],3);
            break;
          case 24:
            if(totalNodes == 3)
            {
              CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,7);
              messagesFlags[7] = 1;
              deviceList[2].numberOfTransmissions = deviceList[2].numberOfTransmissions + 1;
              deviceList[2].rssiSum = deviceList[2].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
              if(receivedMask != 0xFF)
                ZeroField(codingMatrix[7],3);
              break;
            }
          case 25:
            if(totalNodes == 3)
            {
              CopyMatrixLine(message,(PAYLOAD_LENGTH-2)/2,8);
              messagesFlags[8] = 1;
              deviceList[2].numberOfTransmissions = deviceList[2].numberOfTransmissions + 1;
              deviceList[2].rssiSum = deviceList[2].rssiSum + pow(10, (double)halRfGetLastRssi()/10.0);
              if(receivedMask != 0xFF)
                ZeroField(codingMatrix[8],3);
              break;
           }
          }
        }
      }
    }
  }
  
  halRfCommand(CMD_RXFIFO_RESET);
  rfirqf1 = 0;
}

void DestinySetup()
{
  CodingMatrixConfig();
}

void DestinyRun()
{
    receivedMask = 0xFF;
    Receive();
}
#endif