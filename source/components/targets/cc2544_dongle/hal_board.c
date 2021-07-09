/*******************************************************************************
*  Filename:        hal_board.c
*  Revised:         $Date: 2013-04-19 17:01:24 +0200 (fr, 19 apr 2013) $
*  Revision:        $Revision: 9889 $
*
*  Description:     HAL board peripherals library for the cc2544Dongle board.
*
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
#include "hal_types.h"
#include "hal_defs.h"
#include "hal_int.h"
#include "hal_mcu.h"
#include "hal_board.h"
#include "clock.h"


/*******************************************************************************
* GLOBAL FUNCTIONS
*/

/*******************************************************************************
* @fn      halBoardInit
*
* @brief   Set up board. Initialize MCU, configure I/O pins and user interfaces
*
* @param   void
*
* @return  void
*/
void halBoardInit(void) {
    // Set clock (32 MHz XOSC default). 
    halMcuInit();
    
    // Make sure clock is stable ( 10 ms default wait in halMcuInit()). 
    while (CLKCONSTA != 0x80);

    // LEDs and Buttons:
    // All input GPIOs are set with pullup as default.
    // Set P0.0 and P0.1 as GPIO.
    P0SEL0 = 0;
    // Set P0.2 and P0.3 as GPIO.
    P0SEL1 = 0;
  
    // set P0.1 and P0.2 to output and the rest as input. 
    PDIR = (BIT1|BIT2); 
  
    // Clear LEDs.
    P0_1 = 0;
    P0_2 = 0;    
    
    halIntOn();
}