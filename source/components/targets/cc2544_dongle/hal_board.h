/*******************************************************************************
*  Filename:        hal_board.h
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

#ifndef HAL_BOARD_H
#define HAL_BOARD_H

/*******************************************************************************
* INCLUDES
*/
#if (chip==2544)
#include "ioCC2544.h"
#else
#error "Chip not supported!"
#endif

#include "hal_cc8051.h"
#include "hal_defs.h"
#include "hal_mcu.h"


/*******************************************************************************
* CONSTANTS
*/
// Board properties
#define BOARD_NAME                     "CC2544Dongle"
#define NUM_LEDS                        2
#define NUM_BUTTONS                     2

// Clock
#define BSP_CONFIG_CLOCK_MHZ	        32

// LEDs
#define HAL_BOARD_IO_LED_1_PORT        	0   // Red
#define HAL_BOARD_IO_LED_1_PIN         	1
#define HAL_BOARD_IO_LED_2_PORT        	0   // Green
#define HAL_BOARD_IO_LED_2_PIN         	2

// Buttons
#define BUTTON_1        		0   // Button S1 ( by red LED )
#define BUTTON_2        		1   // Button S2 ( by green LED )
#define HAL_BOARD_IO_BTN_1_PORT        	0   // Button S1 ( by red LED )
#define HAL_BOARD_IO_BTN_1_PIN         	0
#define HAL_BOARD_IO_BTN_2_PORT        	0   // Button S2 ( by green LED )
#define HAL_BOARD_IO_BTN_2_PIN         	3


// Debounce
#define HAL_DEBOUNCE(expr)    { int i; for (i=0; i<500; i++) { if (!(expr)) i = 0; } }

/*******************************************************************************
 * MACROS
 */

// LED
#define HAL_LED_SET_1()                 MCU_IO_SET_HIGH(HAL_BOARD_IO_LED_1_PORT, HAL_BOARD_IO_LED_1_PIN)
#define HAL_LED_SET_2()                 MCU_IO_SET_HIGH(HAL_BOARD_IO_LED_2_PORT, HAL_BOARD_IO_LED_2_PIN)

#define HAL_LED_CLR_1()                 MCU_IO_SET_LOW(HAL_BOARD_IO_LED_1_PORT, HAL_BOARD_IO_LED_1_PIN)
#define HAL_LED_CLR_2()                 MCU_IO_SET_LOW(HAL_BOARD_IO_LED_2_PORT, HAL_BOARD_IO_LED_2_PIN)

#define HAL_LED_TGL_1()                 MCU_IO_TGL(HAL_BOARD_IO_LED_1_PORT, HAL_BOARD_IO_LED_1_PIN)
#define HAL_LED_TGL_2()                 MCU_IO_TGL(HAL_BOARD_IO_LED_2_PORT, HAL_BOARD_IO_LED_2_PIN)


// Buttons
#define HAL_BUTTON_1_PUSHED() (!MCU_IO_GET(HAL_BOARD_IO_BTN_1_PORT, \
    HAL_BOARD_IO_BTN_1_PIN))
#define HAL_BUTTON_2_PUSHED() (!MCU_IO_GET(HAL_BOARD_IO_BTN_2_PORT, \
    HAL_BOARD_IO_BTN_2_PIN))
	

// USB interface
#define HAL_USB_ENABLE()                st( USBCTRL= USBCTRL_PLL_EN | USBCTRL_USB_EN; \
                                        while (!(USBCTRL&USBCTRL_PLL_LOCKED)); )
#define HAL_USB_DISABLE()               st(USBCTRL = 0; while (USBCTRL & 0x80);)
#define HAL_USB_PLL_DISABLE()           st( USBCTRL&= ~USBCTRL_PLL_EN; \
                                        while (USBCTRL&USBCTRL_PLL_LOCKED); )
#define HAL_USB_PLL_ENABLE()            st( USBCTRL |= USBCTRL_PLL_EN; \
                                        while (!(USBCTRL&USBCTRL_PLL_LOCKED)); )
#define USB_PADS_ENABLE()               st(TR0 &= ~BIT2;)
#define USB_PADS_DISABLE()              st(TR0 |= BIT2;)
#define HAL_USB_PULLUP_ENABLE()         st( USBCTRL= USBCTRL | USBCTRL_USB_PUE;)
#define HAL_USB_PULLUP_DISABLE()        st( USBCTRL= USBCTRL & ~USBCTRL_USB_PUE;)
#define HAL_USB_INT_ENABLE()            st( P2IEN|= 0x20; IEN2|= 0x02; )
#define HAL_USB_INT_DISABLE()           st( P2IEN&= ~0x20; )
#define HAL_USB_INT_CLEAR()             st( P2IFG= 0; P2IF= 0; )
#define HAL_USB_RESUME_INT_ENABLE()     st ( USBCIE |= USBCIE_RESUMEIE; )
#define HAL_USB_RESUME_INT_DISABLE()    st ( USBCIE &= ~USBCIE_RESUMEIE; )


/*******************************************************************************
 * FUNCTION PROTOTYPES
 */
void halBoardInit(void);

#endif