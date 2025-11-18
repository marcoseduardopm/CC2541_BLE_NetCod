/*******************************************************************************
 *  Filename:        BLE_NC_cc254x.c
 *  Revised:         $Date: 2013-04-29 13:31:25 +0200 (ma, 29 apr 2013) $
 *  Revision:        $Revision: 9926 $
 *
 *  Description:     Packet Error Rate (PER) test for the CC2541EM, CC2543EM,
 *                   CC2544Dongle and the CC2545EM.
 *
 *  note             This code is made for range evaluation as well as for use as
 *                   example code showing how to use the radio. The radio can be
 *                   utilized in more ways than what is used in this example code.
 *
 *  warning          This program is intended for use between one single master
 *                   device and one single slave. Make sure not to have multiple
 *                   Master devices powered up while performing test!
 *
 *  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
// Include device specific files

#include "BLE_NC_cc254x.h"
#include "NodeFunctions.h"
#include "DestinyFunctions.h"
#include "CommonFunctions.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

volatile uint8 rfirqf1 = 0;
extern uint8 RadioTimeoutFlag;

/*******************************************************************************
 * GLOBAL VARIABLES
 */
volatile uint16 counter = 0;
uint8 ledStatus = 0;
uint8 transmissionDone = 0;
uint8 actedThisPhase = 0;

uint16 myNumber = 0;
uint8 phase = 0;

uint8 addressBytes[6];
uint8 messages[MAX_COLS][(PAYLOAD_LENGTH - 2) / 2];
uint8 messagesFlags[9];

uint8 receivedMask = 0;

deviceMap deviceList[MAX_COLS];

uint16 numberOfTransmissions = 0;

uint8 codingMatrix[MAX_ROWS][MAX_COLS];
uint16 resultMatrix[MAX_ROWS][(PAYLOAD_LENGTH - 2) / 2];

double inverseCodingMatrix2Nodes[COLS_2_NODES][ROWS_2_NODES];
double inverseCodingMatrix3Nodes[COLS_3_NODES][ROWS_3_NODES];

double *dataCodingMatrixDouble = NULL;
double **codingMatrixDouble = NULL;
//double *dataInverseMatrixDouble = NULL;
//double **inverseCodingMatrixDouble = NULL;

// double codingMatrixDouble[MAX_ROWS][MAX_COLS];
// double inverseCodingMatrixDouble[MAX_COLS][MAX_ROWS];

// operation_mode, totalNodes, totalTransmissions, and txPower sent by the server
uint8 operationMode;
uint16 totalNodes = 0;
uint16 totalTransmissions;
uint8 txPower = 0xF3; //18dBm
// cols, rows, time_slices  calculated after receiving the configuration from
uint16 rows;
uint16 cols;
uint8 timeSlices;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */
void resetGlobals()
{
    ledStatus = 0;
    transmissionDone = 0;
    actedThisPhase = 0;
    myNumber = 0;
    phase = 0;
    receivedMask = 0;
    numberOfTransmissions = 0;
}

// Timer 1 Interrupt routine (every 1 ms)
HAL_ISR_FUNCTION(T1_ISR, T1_VECTOR)
{
    counter++;
    if (counter >= TOTAL_TIME)
    {
        ledStatus = !ledStatus;
        // TurnLED(ledStatus);
        counter = 0;
        actedThisPhase = 0;
        phase = (phase + 1);
        if (phase >= timeSlices)
        {
            phase = 0;
            transmissionDone = 1;
        }
        // if(RFST != 0)
        //{
        halRfCommand(CMD_SHUTDOWN);
        // while (!(rfirqf1 & RFIRQF1_TASKDONE)){}
        // rfirqf1 = 0;
        //}
    }
    IRCON &= 0xFD; // Clean interrupt flag
    return;
}

void ConfigureTimer()
{
    // CLKCONCMD &= 0xC7; //32MHz Timer tick
    T1CTL = 0x02;   // Count from 0 to T1CC0 (32000 - for 1 ms)
    T1CCTL0 = 0x44; // Compare mode
    T1CC0L = 0x00;  // T1CC0 LSB
    T1CC0H = 0x7D;  // T1CC0 MSB
    IEN1 |= 0x2;    // Enable Timer 1 interrupt
}
void DisableTimer()
{
    IEN1 = IEN1 & ~0x2; // Enable Timer 1 interrupt
}
void calculateVars()
{
    if (totalNodes == 2)
    {
        rows = 4;
        cols = 2;
        timeSlices = 4;
    }
    else if (totalNodes == 3)
    {
        rows = 9;
        cols = 3;
        timeSlices = 9;
    }
}

void freeMatrices()
{
    if (dataCodingMatrixDouble != NULL)
    {
        free(dataCodingMatrixDouble);
        dataCodingMatrixDouble = NULL;
    }

    if (codingMatrixDouble != NULL)
    {
        free(codingMatrixDouble);
        codingMatrixDouble = NULL;
    }
/*
    if (dataInverseMatrixDouble != NULL)
    {
        free(dataInverseMatrixDouble);
        dataInverseMatrixDouble = NULL;
    }

    if (inverseCodingMatrixDouble != NULL)
    {
        free(inverseCodingMatrixDouble);
        inverseCodingMatrixDouble = NULL;
    }
    //*/
}

void allocateMatrices()
{
  //create and set the oldTotalNodes just once.
  static uint16 oldTotalNodes = 0;

  //if we did not create the matrix yet, or if the total nodes changed
  if(dataCodingMatrixDouble == NULL || oldTotalNodes != totalNodes)
  {
    //update the oldTotalNodes
    oldTotalNodes = totalNodes;

    //if we already have a created version, release it.
    if(dataCodingMatrixDouble != NULL)
      freeMatrices();

    // Allocate memory for codingMatrixDouble
    dataCodingMatrixDouble = (double *)malloc(rows * cols * sizeof(double));
    if (dataCodingMatrixDouble == NULL)
    {
        return;
    }

    codingMatrixDouble = (double **)malloc(rows * sizeof(double *));
    if (codingMatrixDouble == NULL)
    {
        free(dataCodingMatrixDouble);
        dataCodingMatrixDouble = NULL;
        return;
    }

    for (int i = 0; i < rows; i++)
    {
        codingMatrixDouble[i] = &dataCodingMatrixDouble[i * cols];
    }

/*
    // Allocate memory for inverseCodingMatrixDouble
    // OOM while trying to allocate. For now, lest just use globals.
    dataInverseMatrixDouble = (double *)malloc(cols * rows * sizeof(double));
    if (dataInverseMatrixDouble == NULL)
    {
        free(codingMatrixDouble);
        free(dataCodingMatrixDouble);
        codingMatrixDouble = NULL;
        dataCodingMatrixDouble = NULL;
        return;
    }
    inverseCodingMatrixDouble = (double **)malloc(cols * sizeof(double *));
    if (inverseCodingMatrixDouble == NULL)
    {
        free(dataInverseMatrixDouble);
        free(codingMatrixDouble);
        free(dataCodingMatrixDouble);
        inverseCodingMatrixDouble = NULL;
        codingMatrixDouble = NULL;
        dataCodingMatrixDouble = NULL;
        return;
    }

    for (int i = 0; i < cols; i++)
    {
        inverseCodingMatrixDouble[i] = &dataInverseMatrixDouble[i * rows];
    }
*/
  }
}


int getTxPowerdBm(unsigned char txPower)
{
    switch (txPower)
    {
    case 0xE1:
        return 0;
    case 0xE5:
        return 4;
    case 0xF3:
        return 18;
    default:
        return -99; // Unknown or unsupported value
    }
}
/*******************************************************************************
 * @fn          main
 *
 * @brief       Main program
 *
 * @param       void4
 *
 * @return      int (does not return)
 */
int main(void)
{

    // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
    rfirqf1 = 0;

    // Ensure that the P12 is at input (high impedance)
    MCU_IO_INPUT(1, 2, MCU_IO_PULLDOWN);

    /* Initialize Clock Source (32 Mhz Xtal),
     *  global interrupt (EA=1),  I/O ports and pheripherals(LCD). */
    halBoardInit();

    // Ensure that the P12 is at input (high impedance)
    MCU_IO_INPUT(1, 2, MCU_IO_TRISTATE);

    //printf("Init\n");

    halRfDisableRadio(FORCE);
    halRfBroadcastInit();
    halRfBroadcastSetChannel(CHANNEL);
    halRfEnableRadio();

    // all pins must be at high-impedance (input) if not used or output low
    for (int i = 0; i < 8; i++)
    {
        MCU_IO_INPUT(0, i, MCU_IO_PULLDOWN);
        MCU_IO_INPUT(1, i, MCU_IO_PULLDOWN);
    }

    for (int i = 0; i < 5; i++)
    {
        MCU_IO_INPUT(2, i, MCU_IO_PULLDOWN);
    }

    // sleepMode(1000, 0); //sleep
    // Turn the LED ON
    // TurnLED(1);
    // sleepMode(200, 0); //sleep
    // Ensure that the P12 is at input (high impedance)
    TurnLED(0);

    halRfEnableInterrupt(RFIRQF1_TASKDONE);

    uint16 *startMessage = malloc(PAYLOAD_LENGTH * sizeof(uint16));


#ifdef MODETX

#if NODE_NUMBER == 0
    uint32 waitTime = TOTAL_TIME / 2;
#elif NODE_NUMBER == 1
    uint32 waitTime = TOTAL_TIME / 3;
#elif NODE_NUMBER == 2
    uint32 waitTime = TOTAL_TIME / 6;
#endif
    // iteration loop, receiving a command and executing it continuously
    while (1)
    {
        resetGlobals();

        IncludeDevices();

        for (int i = 0; i < 6; i++)
            addressBytes[i] = deviceList[NODE_NUMBER].address[i];

        // wait for a new init signal
        while (!ReceiveInitSignal()){}
        //retransmit the received message just in case some peer dint listen to it
        startMessage[0] = operationMode;
        startMessage[1] = totalNodes;
        startMessage[2] = totalTransmissions;
        startMessage[3] = txPower;
        for (int i = 0; i < 2; i++) //2 times
        {
            Transmit(255, 0xFF, startMessage);
        }

        // Needed because the power might have changed
        halRfDisableRadio(FORCE);
        halRfBroadcastInit();
        halRfBroadcastSetChannel(CHANNEL);
        halRfEnableRadio();

        calculateVars();
        // create the matrices based on the obtained data if necessary
        allocateMatrices();

        // Needed because the power might have changed
        halRfBroadcastSetChannel(CHANNEL);

        // sleepMode(waitTime, 0);

        ClearMessages();

        counter = 0;
        // Set Timer 1 to interrupt every 1 ms
        ConfigureTimer();

        while (counter < waitTime)
        {
        }
        counter = 0;

        while (1)
        {
            NodeSetup();

            while (!transmissionDone)
            {
                NodeRun();
            }

            myNumber++;
            ClearMessages();
            transmissionDone = 0;

            // stop after sending all transmissions
            if (myNumber >= totalTransmissions)
                break;
        }


        TurnLED(1);
        // wait 200ms just to show that we are alive
        counter = 0;
        while (counter < 200)
        {
        }
        TurnLED(0);

        counter = 0;
        DisableTimer();
    }
#else
    // addd inside of the loop (while or for) the parameter you want to change.
    // Possiblity operationMode or txPower
    // operationMode = SERVER_DEF_OPERATION_MODE;
    totalNodes = SERVER_DEF_TOTAL_NODES;
    totalTransmissions = SERVER_DEF_TOTAL_TRANSMISSIONS_PER_TIME;
    // txPower = SERVER_DEF_TXPOWER;




    char opModes[] = {DAF, BNC, DNC, GDNC};
    //unsigned char txPowers[] = {0xE1, 0xE5, 0xF3};
    unsigned char txPowers[] = {0xE1, 0xE5}; //obtain only for 0 and 4dBm
    //char opModes[] = {GDNC, DNC};
    //unsigned char txPowers[] = { 0xF3};


    // Print header
    printf("Sch\tNod\tTxPW\tSucDec\tDecTot\tLost\tNumTrans\tTotTrans\tMeanRSSI\n");

    while (1)
    {

        //how many times to repeat per configuration?
        for (int iRepeat = 0; iRepeat < SERVER_DEF_TIMES_PER_CONFIG; iRepeat++)
        {
          for (int iTx = 0; iTx < sizeof(txPowers); iTx++)
          {
            txPower = txPowers[iTx];
            int dBm = getTxPowerdBm(txPower);

            for (int iOpMode = 0; iOpMode < sizeof(opModes); iOpMode++)
            {
                operationMode = opModes[iOpMode];
                resetGlobals();
                IncludeDevices();


                startMessage[0] = operationMode;
                startMessage[1] = totalNodes;
                startMessage[2] = totalTransmissions;
                startMessage[3] = txPower;

                for (int i = 0; i < 6; i++)
                {
                    Transmit(255, 0xFF, startMessage);
                }
                // calculate adicional variables based on the totalNodes
                calculateVars();
                // Create the matrices based on calculated vars
                // Note that it will create only if necessary
                allocateMatrices(); //


                // Set Timer 1 to interrupt every 1 ms
                ConfigureTimer();

                while (1)
                {

                    DestinySetup();

                    while (!transmissionDone)
                    {
                        DestinyRun();
                    }

                    numberOfTransmissions++;

                    for (int i = 0; i < rows; i++)
                    {
                        if (!messagesFlags[i])
                            ZeroLine(codingMatrix[i], cols);
                        for (int j = 0; j < cols; j++)
                            codingMatrixDouble[i][j] = (double)codingMatrix[i][j];
                    }

                    if(totalNodes == 2)
                      InvertMatrix((double *)dataCodingMatrixDouble, (double *)inverseCodingMatrix2Nodes);
                    else if(totalNodes == 3)
                      InvertMatrix((double *)dataCodingMatrixDouble, (double *)inverseCodingMatrix3Nodes);
                    //InvertMatrix((double *)dataCodingMatrixDouble, (double *)dataInverseMatrixDouble);
                    GetResults(cols, rows, rows, (PAYLOAD_LENGTH - 2) / 2);

                    ClearMessages();
                    transmissionDone = 0;

                    for (int i = 0; i < cols; i++)
                        deviceList[i].expectedSequenceNumber = deviceList[i].expectedSequenceNumber + 1;

                    if (numberOfTransmissions >= totalTransmissions)
                    {

                      // Print data for each node
                      for (int i = 0; i < cols; i++) {
                          deviceList[i].rssi = (int8)(log10(deviceList[i].rssiSum / deviceList[i].numberOfTransmissions) * 10);

                          const char* modeStr = "UNK";
                          if (operationMode == DAF)
                              modeStr = "DAF";
                          else if (operationMode == BNC)
                              modeStr = "BNC";
                          else if (operationMode == DNC)
                              modeStr = "DNC";
                          else if (operationMode == GDNC)
                              modeStr = "GDNC";

                          printf("%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
                              modeStr,
                              i,
                              dBm, // TX power in dBm
                              (int)deviceList[i].totalPackages,
                              totalTransmissions,
                              (int)deviceList[i].packageLosses,
                              (int)deviceList[i].numberOfTransmissions,
                              totalTransmissions * cols,
                              (int)deviceList[i].rssi
                          );
                      }

                        /*int dBm = getTxPowerdBm(txPower);

                        if (operationMode == DAF)
                            printf("DAF - TxPW %d dBm:\n", dBm);
                        else if (operationMode == BNC)
                            printf("BNC - TxPW %d dBm:\n", dBm);
                        else if (operationMode == DNC)
                            printf("DNC - TxPW %d dBm:\n", dBm);
                        else if (operationMode == GDNC)
                            printf("GDNC - TxPW %d dBm:\n", dBm);
                        else
                            printf("Unknown mode - TxPW %d dBm:\n", dBm);

                        for (int i = 0; i < cols; i++)
                        {
                            deviceList[i].rssi = (int8)(log10(deviceList[i].rssiSum / deviceList[i].numberOfTransmissions) * 10);

                            printf("Nod %d\n", i, dBm);
                            printf("%d/%d dec msg\n", (int)deviceList[i].totalPackages, totalTransmissions);
                            printf("%d msg lost\n", (int)deviceList[i].packageLosses);
                            printf("%d/%d pkt arrived\n", (int)deviceList[i].numberOfTransmissions, totalTransmissions * cols);
                            printf("mean RSSI: %d dBm\n\n", (int)deviceList[i].rssi);
                        }*/

                        break;
                    }
                }


                TurnLED(0);
                // wait 8s before restarting
                for (int kTime = 0; kTime < 40; kTime++)
                {
                    counter = 0;
                    while (counter < 200)
                    {
                    }
                }
                counter = 0;
                DisableTimer();
                TurnLED(0);
            }
          }
        }
    }


#endif
    //after completion, free the matrices (will never the reached)
    freeMatrices();
    // startmsg was created before the while
    free(startMessage);
}