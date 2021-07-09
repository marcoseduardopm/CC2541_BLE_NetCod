/*******************************************************************************
*  Filename:        hal_led.c
*  Revised:         $Date: 2013-04-22 10:49:57 +0200 (ma, 22 apr 2013) $
*  Revision:        $Revision: 9900 $
*
*  Description:     HAL led library.
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
#include "hal_led.h"
#include "hal_board.h"

/*******************************************************************************
* GLOBAL VARIABLES
*/
// Stores state of led 4
volatile uint8 led4State=0;


/*******************************************************************************
* GLOBAL FUNCTIONS
*/

/*******************************************************************************
* @fn          halLedSet
*
* @brief       Turn LED on.
*
* @param       uint8 id - led to set
*
* @return      none
*/
void halLedSet(uint8 id) {
    switch (id)
    {
    case 1: HAL_LED_SET_1(); break;
    case 2: HAL_LED_SET_2(); break;
    case 3: HAL_LED_SET_3(); break;
    case 4: HAL_LED_SET_4(); led4State=1; break;

    default: break;
    }
}


/*******************************************************************************
* @fn          halLedClear
*
* @brief       Turn LED off.
*
* @param       uint8 id - led to clear
*
* @return      none
*/
void halLedClear(uint8 id) {
    switch (id) {
    case 1: HAL_LED_CLR_1(); break;
    case 2: HAL_LED_CLR_2(); break;
    case 3: HAL_LED_CLR_3(); break;
    case 4: HAL_LED_CLR_4(); led4State=0; break;
    default: break;
    }
}


/*******************************************************************************
* @fn          halLedToggle
*
* @brief       Change state of LED. If on, turn it off. Else turn on.
*
* @param       uint8 id - led to toggle
*
* @return      none
*/
void halLedToggle(uint8 id) {
    switch (id) {
    case 1: HAL_LED_TGL_1(); break;
    case 2: HAL_LED_TGL_2(); break;
    case 3: HAL_LED_TGL_3(); break;
    case 4: HAL_LED_TGL_4(); led4State ^= 1; break;
    default: break;
    }
}