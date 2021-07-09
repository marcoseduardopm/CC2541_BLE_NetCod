/*******************************************************************************
*  Filename:        hal_button.c
*  Revised:         $Date: 2013-04-29 13:31:25 +0200 (ma, 29 apr 2013) $
*  Revision:        $Revision: 9926 $
*
*  Description:     HAL button implementation for CC2544Dongle
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
#include "hal_board.h"
#include "hal_button.h"
#include "hal_cc8051.h"



/*******************************************************************************
* @fn           halButtonPushed
*
* @brief        This function detects if 'S1' or 'S2' is being pushed. The 
*               function implements software debounce. Return true only if 
*               previuosly called with button not pushed. Return true only once 
*               each time the button is pressed.
*
* @param        void
*
* @return       uint8: 
*          	    HAL_BUTTON_NONE: No Button is being pushed.
*          	    HAL_BUTTON_1: Button 1 is being pushed.
*          	    HAL_BUTTON_2: Button 2 is being pushed.
*          	    HAL_BUTTON_BOTH: Button 1 and 2 is being pushed.
*/
uint8 halButtonPushed(void) {
    uint8 v= HAL_BUTTON_NONE;
    uint8 button1 = HAL_BUTTON_1_PUSHED();
    uint8 button2 = HAL_BUTTON_2_PUSHED();

    if (button1 && button2) {
        HAL_DEBOUNCE(!HAL_BUTTON_1_PUSHED());
        v= HAL_BUTTON_BOTH;
    }
    else if (button1) {
        HAL_DEBOUNCE(!HAL_BUTTON_1_PUSHED());
        v= HAL_BUTTON_1;
    }
	else if (button2) {
        HAL_DEBOUNCE(!HAL_BUTTON_2_PUSHED());
        v= HAL_BUTTON_2;
    }
    return v;
}