#include "DestinyFunctions.h"
#include "CommonFunctions.h"
#include "BLE_Broadcaster_cc254x.h"
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"
#include "stdio.h"
#include "pseudo_inverse.h"

#ifndef MODETX

void InvertMatrix(double *A, double *A_inv)
{
    double U[ROWS][COLS];                                                        
    double V[COLS][COLS];                                                        
    double singular_values[COLS];                                             
    double* dummy_array;      

    dummy_array = (double*) malloc(COLS * sizeof(double));                    
    if (dummy_array == NULL) { 
        //printf(" No memory available\n"); 
        exit(0); 
    } 

    double err = Singular_Value_Decomposition((double*) A, ROWS, COLS, (double*) U, singular_values, (double*) V, dummy_array);   //

    free(dummy_array);                                                     
    //if (err < 0) 
        //printf("Failed to converge\n");     
                                         
    double tolerance = 0;
    
    Singular_Value_Decomposition_Inverse((double*) U, singular_values, (double*) V,
                    tolerance, ROWS, COLS, (double*) A_inv); 
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
          sum += (inverseCodingMatrix[i][k] * (double)resultMatrix[k][j]);
        }
        messages[i][j] = (uint8)(sum + 0.5);
      }
    }
  }
  for(int i = 0; i < 3; i++)
  {
    if(!EmptyLine(messages[i], PAYLOAD_LENGTH - 1))
    {
      uint8 seqNumber = messages[i][0];
      int difference = seqNumber - deviceList[i].sequenceNumber - 1;
      if(difference < 0)
        difference = 0;
      deviceList[i].sequenceNumber = seqNumber;
      deviceList[i].packageLosses = deviceList[i].packageLosses + difference;
      deviceList[i].totalPackages++;
  
      //if(difference)
        //printf("%d: %d\n",i,deviceList[i].packageLosses);
    }
  }
}

void ZeroLine(uint8* line, int size)
{
  for(int i = 0; i < size; i++)
    line[i] = 0;
}

void CopyMatrixLine(uint8* line, int matrixColumns, int lineNumber)
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
        //printf("0, %d\n", counter);
        counter = TOTAL_TIME/2;
        phase = 0;
        messageCounter++;
        break;
      case 1:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,1);
        messagesFlags[1] = 1;
        //printf("1, %d\n", counter);
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
        //printf("10, %d\n", counter);
        counter = TOTAL_TIME/2;
        messagesFlags[2] = 1;
        phase = 2;
        messageCounter++;
        break;
      case 11:
        CopyMatrixLine(message,PAYLOAD_LENGTH-1,3);
        //printf("11, %d\n", counter);
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

void DestinySetup()
{
  CodingMatrixConfig();
}

void DestinyRun()
{
    Receive();
}
#endif