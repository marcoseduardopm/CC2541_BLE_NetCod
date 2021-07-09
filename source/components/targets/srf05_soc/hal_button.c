/*******************************************************************************
*  Filename:        hal_button.c
*  Revised:         $Date: 2013-04-22 10:49:57 +0200 (ma, 22 apr 2013) $
*  Revision:        $Revision: 9900 $
*
*  Description:     HAL button implementation for CCSoC + SmartRF05EB
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
* @fn  halButtonInit
*
* @brief
*      Initializes the button functionality
*
* Parameters:
*
* @param  void
*
* @return void
*
*/
void halButtonInit(void) {
    // Button push input
    MCU_IO_INPUT(HAL_BOARD_IO_BTN_1_PORT, HAL_BOARD_IO_BTN_1_PIN, MCU_IO_TRISTATE);
}


/*******************************************************************************
* @fn  halButtonPushed
*
* @brief
*      This function detects if 'S1' is being pushed. The function
*      implements software debounce. Return true only if previuosly called
*      with button not pushed. Return true only once each time the button
*      is pressed.
*
* Parameters:
*
* @param  void
*
* @return uint8
*          1: Button is being pushed
*          0: Button is not being pushed
*
*/
uint8 halButtonPushed(void) {
    extern volatile uint8 led4State;
    uint8 v= HAL_BUTTON_NONE;

#ifdef SRF05EB_VERSION_1_3
    if (HAL_BUTTON_1_PUSHED()) {
        HAL_DEBOUNCE(!HAL_BUTTON_1_PUSHED());
        v= HAL_BUTTON_1;
    }

#else

    // Need to set direction because the button is shared with LED4 on SmartRF05EB rev 1.7+
    MCU_IO_INPUT(HAL_BOARD_IO_BTN_1_PORT, HAL_BOARD_IO_BTN_1_PIN, MCU_IO_TRISTATE);

    if (HAL_BUTTON_1_PUSHED()) {
        HAL_DEBOUNCE(!HAL_BUTTON_1_PUSHED());
        v= HAL_BUTTON_1;
    }

    // Restore for use with LED
    MCU_IO_OUTPUT(HAL_BOARD_IO_LED_4_PORT, HAL_BOARD_IO_LED_4_PIN, led4State);
#endif

    return v;
}
