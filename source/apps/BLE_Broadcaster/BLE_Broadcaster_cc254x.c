/*******************************************************************************
*  Filename:        BLE_Broadcaster_cc254x.c
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
#if (chip==2541)
#include "ioCC2541.h"
#elif (chip==2543)
#include "ioCC2543.h"
#elif (chip==2544)
#include "ioCC2544.h"
#elif (chip==2545)
#include "ioCC2545.h"
#else
#error "Chip not supported!"
#endif
#include "BLE_Broadcaster_cc254x.h"
#include "prop_regs.h"
#include "hal_timer2.h"
#include "hal_sleep.h"
#include "hal_rf_proprietary.h"
#include "hal_rf_broadcast.h"
#include "hal_int.h"
#include "hal_board.h"
#include "hal_button.h"
#include "hal_led.h"
#include "myData.h"

#define DEBUG 1 // imprime na tela
#define MODETX 1//0 receptor, 1 tx
#define DELAYTIME 60000/4 //for TX
#if(DEBUG)
#include <stdio.h>
#endif

#if defined (PLACA_A) && (PLACA_A == 1)
char endereco[] = {0,1,2,3,4,5};
char dados[] = {0x11,0x22,0x33};
char id = 0;
#elif defined (PLACA_B) && (PLACA_B == 1)
char endereco[] = {'A','B','C','D','E','F'};
char dados[] = {0x55,0x44,0x66};
char id = 'A';
#else//receptor
char endereco[] = {9,'B',6,'D','E',0};
char dados[] = {0x99,0x22,0x33};
char id = 0x99;
#endif


// Global flags.

extern volatile uint8 rfirqf1;
extern uint8 RadioTimeoutFlag;
#if(MODETX)
struct tagMyDevice myDevice;
struct tagReceiveData receiveData;
#else
struct tagMyDevice myDevice;
struct tagMyData myDatasA;
struct tagMyData myDatasB;

#endif

#if(DEBUG)
#define PRINTF(...) printf(__VA_ARGS__)
//#define eprintf(...) fPRINTF (stderr, __VA_ARGS__)
#else
#define PRINTF(...) asm("nop");
#endif

#if(POWER_SAVING)
uint8 powerModeFlag;
#endif

int received_data[20];

#if(MODETX)
void halRfLoadBLEBroadcastPacketPayload(uint8 print)
{
    
    uint8 i, addr[6], data[3], id;
    uint8 send_data_length = RFRXFLEN;//qtidade de bytes na fila
    if(print)
        PRINTF("%d: ", send_data_length);
    //Number of bytes effective data
    //send_data_length = (sizeof(data_to_send)/sizeof(int));
    
    //Read data from FIFO
    for(i=0;i<send_data_length;i++){
        received_data[i] = RFD;
        if(print)
            PRINTF("%d ", received_data[i]);
    }
    PRINTF("\n");
    id = received_data[3];
    if(myDevice.id != id){
         for(int i = 0; i<3;i++){
            receiveData.payload[i] = received_data[3+i];
         }
         myDevice.msgReceived =1;
    }
    
    return;  
}
#else
void halRfLoadBLEBroadcastPacketPayload(uint8 print)
{
    
    uint8 i, addr[6], data[3];
    uint8 send_data_length = RFRXFLEN;//qtidade de bytes na fila
    if(print)
        PRINTF("%d: ", send_data_length);
    //Number of bytes effective data
    //send_data_length = (sizeof(data_to_send)/sizeof(int));
    
    //Read data from FIFO
    for(i=0;i<send_data_length;i++){
        received_data[i] = RFD;
        if(print)
            PRINTF("%d ", received_data[i]);
    }
    PRINTF("\n");
    if(received_data[3] == 0  && received_data[8] == 5){
        for(int i = 0; i<6;i++){
            myDatasA.address[myDatasB.line][i] = received_data[3+i];
            if(i<3){
                myDatasA.payload[myDatasB.line][i] = received_data[14+i];
            }
        }
        myDatasA.line++;
    }
    else if(received_data[3] == 'A'  && received_data[8] == 'F'){
        for(int i = 0; i<6;i++){
            myDatasB.address[myDatasB.line][i] = received_data[3+i];
            if(i<3){
                myDatasB.payload[myDatasB.line][i] = received_data[14+i];
            }
        }
        myDatasB.line++;
    }
    if(myDatasA.line ==9){
        myDatasA.line = 0;
    }
    if(myDatasB.line==9){
        myDatasB.line = 0;
    }
    
    return;  
}
#endif
void sleepMode(uint32 sleepDurationMs, uint8 afterLastRecPackage)
{
    
#if(POWER_SAVING)
    uint8 tuneTimeoutFlag = 0, ovf10Bit1 = 0, ovf10Bit2 = 0;
    uint16 timeStampFine, fine;
    uint32 timeStampCoarse, coarse;
    int32 sleepDuration;
    sleepDuration = sleepDurationMs*32; //conversion to ms
    if(afterLastRecPackage)
    {
        // Retrieve captured timestamp from start of received packet (SOP).
        halTimer2GetCapturedTime(&timeStampFine, &timeStampCoarse);
        
        // Retrieve current time from timer 2 (TIME).
        halTimer2GetFullCurrentTime(&fine, &coarse);
        
        //  Find time delta between SOP and TIME (32 MHz ticks). This calcuation
        //  assumes that the timer2 base period is set to 1 ms (0x7D00). 
        coarse -= timeStampCoarse;
        if( fine < timeStampFine ) {
            fine += (0x7D00 - timeStampFine);
            coarse--;
        }
        else {
            fine -=  timeStampFine;
        }
        
        sleepDuration -= (uint32) ((fine/977) + (32*coarse) + 32);
    }
    
    if(sleepDuration < 32) {
        // If sleep duration is less than 1 ms, do not enter sleep.
        powerModeFlag = 0;
    }
    else if (sleepDuration < 100) {
        // If sleep duration is less than 3 ms, use PM1 instead of PM2.
        powerModeFlag = 1;
    }
    else {
        powerModeFlag = 2;
    }
    
    if(powerModeFlag) {
        // Set the sleep time compare value.
        halSleepSetSleepTimer(sleepDuration);
        
        // CC2541/45 has automatic sync between 32 kHz ST and timer2.
        halTimer2Stop(1);
        
        // Disable the sleep timer interrupt.
        halSleepEnableInterrupt();
        
        // Make Sure Global Interrupt is enabled.
        halIntOn();
        
        // Set LED3 indicate entering power mode function.
        //halLedSet(3);
        
        // No more pillow fights, time for bed.
        halSleepEnterPowerMode(powerModeFlag);
        
        // Clear LED3 indicate exit from power mode function.
        //halLedClear(3);
        
        // Disable the sleep timer interrupt.
        halSleepDisableInterrupt();
        
        // CC2541/45 has automatic sync between 32 kHz ST and timer2.
        halTimer2Start(1);
    }
    //  Due to the inaccuracy of the synchronizing between the sleep timer
    //  and timer 2 done above, the code will trim the timer2 current value
    //  for every 1024 transmitted packets from the  total packets as well as once after receiving one of the 10 last packets.
    //
#endif
}

void justSend(char *address_ptr, char *data_ptr)
{
    
    unsigned char address[6];
    for(int i=0;i<6;i++){
        address[i] = *(address_ptr+i) ; // Address (LSB)
    }
    //  Address[0] = 0; // Address (LSB)
    //  Address[1] = 1;
    //  Address[2] = 2;
    //  Address[3] = 3;
    //  Address[4] = 4;
    //  Address[5] = 5; // Address (MSB)  
    
    unsigned char data[3];
    for(int i=0;i<3;i++){
        data[i] = *(data_ptr+i) ; // Address (LSB)
    }
    //  data[0] = 0x1;
    //  data[1] = 0x2;
    //  data[2] = 0x3;
    
    
    //obtainSem0();
    halRfBroadcastLoadPacket(data, 3, address);
    //releaseSem0();
    
    unsigned char L= RFTXFLEN;
    PRINTF("La: %d\n", L);
    
    // Start transmitter.
    while(RFST != 0);
    RFST = CMD_TX;
    
    
    // Wait for TASKDONE and halt CPU (PM0) until task is completed.
    while (!( RFIRQF1 & RFIRQF1_TASKDONE)) {//passagem de ambas de 0 para 1, houve uma interrupção
        /*
#if(POWER_SAVING)
        halSleepEnterPowerMode(CPU_HALT);
#endif
        //*/
    }     
    
    // If data received read FIFO   
    if(PRF.ENDCAUSE == TASK_ENDOK)    
    {        
        //Get packet data.
        //halRfLoadBLEBroadcastPacketPayload();
        PRINTF("Tx Ok\n");
        
        //Turn the LED ON
        MCU_IO_OUTPUT(1, 2, 1);
        sleepMode(50, 0); //sleep 
        //Ensure that the P12 is at input (high impedance)
        MCU_IO_INPUT(1, 2, MCU_IO_TRISTATE);
        
    } else {
        PRINTF("Tx NOk\n");
        //Turn the LED ON
        MCU_IO_OUTPUT(1, 2, 1);
    }
    L= RFTXFLEN;
    PRINTF("Lb: %d\n", L);
    
    //Necessary to ensure that the semaphore is released after this function ends.
    //Looks like other tasks (like sleep modes) might disrupt the semaphore.
    //Futher investigation is necessary.
    //Sem. error otherwise
    obtainSem0();
    PRF.ENDCAUSE = TASK_UNDEF;
    releaseSem0();
    
}
void checkReceiveData(){
    // If data received read FIFO   
    if(PRF.ENDCAUSE == TASK_ENDOK){      
        if(RFIRQF1 & RFIRQF1_RXOK) {
            PRINTF("Ok ");
            halRfLoadBLEBroadcastPacketPayload(1);
        }
        else if(RFIRQF1 & RFIRQF1_RXNOK) {
            //PRINTF("NOk ");
            //halRfLoadBLEBroadcastPacketPayload(1);          
        }
        else {
            PRINTF("*");
        }     	   
        halRfCommand(CMD_RXFIFO_RESET);
    }
    else // Check if test time has expired or last packet is received.
        if((PRF.ENDCAUSE == TASK_RXTIMEOUT)) {
            PRINTF("Timeout\n");
            // Disable global interrupt.
            //halIntOff();
            // Set LED1 to indicate that test is complete.
            //halLedSet(1);
            // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
            //RFIRQF1 = 0;
            // Set ENDCAUSE to a undefined value in enum type end cause.
            PRF.ENDCAUSE = TASK_UNDEF;
            halRfCommand(CMD_RXFIFO_RESET);
        }
        else {
            PRINTF("- ");
            PRINTF("PRF.ENDCAUSE: %d\n", PRF.ENDCAUSE);
            //SEND_LLE_CMD(CMD_STOP);
            halRfCommand(CMD_RXFIFO_RESET);
        }
    // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
    RFIRQF1 = 0;
    //Necessary to ensure that the semaphore is released after this function ends.
    //Looks like other tasks (like sleep modes) might disrupt the semaphore.
    //Futher investigation is necessary.
    //Sem. error otherwise
    //obtainSem0();
    // Set to a undefined value in enum type end cause.
    PRF.ENDCAUSE = TASK_UNDEF;
}
void loadInformation(){
    myDevice.id = id;
    for(int i =0; i<3;i++){
        myDevice.payload[i] = dados[i];
    }
    for(int i =0; i<6;i++){
        myDevice.address[i] = endereco[i];
    }
}
void doBnc(){
    for(int i =0; i<3;i++){
        dados[i] ^= myDevice.payload[i];
    }
    for(int i =0; i<6;i++){
        endereco[i] ^= myDevice.address[i];
    }
    myDevice.msgCombined=1;
}
void recoveryMsgOriginal(){
    for(int i =0; i<3;i++){
        dados[i] = myDevice.payload[i];
    }
    for(int i =0; i<6;i++){
        endereco[i] = myDevice.address[i];
    }
    myDevice.flags = 0x01;
}


int main(void) {
    
    // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
    rfirqf1 = 0;
    
    //Ensure that the P12 is at input (high impedance)
    MCU_IO_INPUT(1, 2, MCU_IO_PULLDOWN);
    
    /* Initialize Clock Source (32 Mhz Xtal),
    *  global interrupt (EA=1),  I/O ports and pheripherals(LCD). */
    halBoardInit();
    //enable timer
    halTimer2EnableInterrupt(100);
    //Ensure that the P12 is at input (high impedance)
    MCU_IO_INPUT(1, 2, MCU_IO_TRISTATE);
    
    PRINTF("Init\n");
    
    halRfDisableRadio(FORCE);
    halRfBroadcastInit();
    halRfBroadcastSetChannel(BLE_BROADCAST_CHANNEL_30);
    loadInformation();
    
    
    ///
#if(POWER_SAVING)
    // Capture start of every Rx packet.
    //PRF.RADIO_CONF.RXCAP = 1;
#endif
    halRfEnableRadio();//= LLECTRL := 0x01;
    
    //all pins must be at high-impedance (input) if not used or output low
    for(int i = 0; i < 8; i++)
    {
        MCU_IO_INPUT(0, i, MCU_IO_PULLDOWN);
        MCU_IO_INPUT(1, i, MCU_IO_PULLDOWN);
    }    
    
    for(int i = 0; i < 5; i++)
    {
        MCU_IO_INPUT(2, i, MCU_IO_PULLDOWN);
    }
    
    sleepMode(3000, 0); //sleep 
    //Turn the LED ON
    MCU_IO_OUTPUT(1, 2, 1);
    sleepMode(200, 0); //sleep 
    //Ensure that the P12 is at input (high impedance)
    MCU_IO_INPUT(1, 2, MCU_IO_PULLDOWN);
    
    
    
    while(1)  
    {
#if(MODETX)
        //{
        if(!myDevice.msgSend || myDevice.msgCombinedSend){
            halRfDisableRadio(FORCE);
            
            //sleepMode(300000, 0); //sleep 60s before attempt to transmit
            //sleepMode(60000, 0); //sleep 60s before attempt to transmit
            sleepMode(DELAYTIME, 0); //sleep 60s before attempt to transmit
            
            //Put the LED at high impedance
            MCU_IO_INPUT(1, 2, MCU_IO_TRISTATE);
            
            halRfEnableRadio();//= LLECTRL := 0x01
            if(!myDevice.msgCombined){
                justSend( myDevice.address, myDevice.payload);
                myDevice.msgSend = 1;
                myDevice.timer = DELAYTIME;
            }
            else{
                justSend(endereco, dados);
                myDevice.msgCombinedSend =1;
            }
            if(myDevice.msgCombinedSend){
                recoveryMsgOriginal();
            }
            // Reset TXFIFO to clear any remaining data in buffer.
            halRfCommand(CMD_TXFIFO_RESET);
            // Clear the global RFIRQF1 shadow variable (RF Interrupt flags).
            rfirqf1 = 0;
            
            //obtainSem0();
            // Set to a undefined value in enum type end cause.
            PRF.ENDCAUSE = TASK_UNDEF;
        }
        else if(!myDevice.timer){
            myDevice.timer = DELAYTIME;
            myDevice.flags = 0;
        }
        else{
            while (!( RFIRQF1 & RFIRQF1_TASKDONE)) {
                checkReceiveData();
                if(myDevice.msgReceived){
                    doBnc();
                }
            }
        }
        //releaseSem0();
        
        //} else {
#else
        
        // Start receiver.
        halRfStartRx();
        
        // Wait for TASKDONE and halt CPU (PM0) until task is completed.
        while (!( RFIRQF1 & RFIRQF1_TASKDONE)) {
            /*
#if(POWER_SAVING)
            halSleepEnterPowerMode(CPU_HALT);
#endif
            //*/
        }       
        checkReceiveData();
#endif
        
    }
    
    // Disable interrupts enabled in this function, clear timer 2 and return.
    // Disable global interrupt.
    halIntOff();
    // Reset Timer2.
    halTimer2Reset();
    // Disable RF interrupt.
    halRfDisableInterrupt(RFIRQF1_TASKDONE);
    
}