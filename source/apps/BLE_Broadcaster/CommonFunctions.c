#include "CommonFunctions.h"
#include "BLE_Broadcaster_cc254x.h"
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"
#include "pseudo_inverse.h"
#include <stdio.h>

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

void InvertMatrix(double *A, double *A_inv)
{
#if TOTAL_NODES == 2
#define ROWS 4
#define COLS 2
#elif TOTAL_NODES == 3
#define ROWS 9
#define COLS 3
#endif
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