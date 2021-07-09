/*******************************************************************************
*  Filename:        hal_board.c
*  Revised:         $Date: 2013-04-23 20:04:58 +0200 (ti, 23 apr 2013) $
*  Revision:        $Revision: 9906 $
*
*  Description:     HAL board peripherals library for the
*                   SmartRF05EB + CC2430EM/CC2531EM/CC2510EM/CC1110EM/CC2541EM/.
*			        .../CC2543EM/CC2545EM platforms
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
#include "hal_lcd.h"
#include "hal_joystick.h"
#include "clock.h"


// comment: should this be here?
#define SPI_CLOCK_POL_LO       0x00
#define SPI_CLOCK_POL_HI       0x80
#define SPI_CLOCK_PHA_0        0x00
#define SPI_CLOCK_PHA_1        0x40
#define SPI_TRANSFER_MSB_FIRST 0x20
#define SPI_TRANSFER_MSB_LAST  0x00

/*******************************************************************************
* LOCAL FUNCTIONS
*/

/*******************************************************************************
* GLOBAL VARIABLES
*/
#if defined MAKE_CRC_SHDW
#pragma location="CRC_SHDW"
const CODE uint16 _crcShdw = 0xFFFF;
#pragma required=_crcShdw

#endif

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
    
    // Set clock source.
    halMcuInit();
    
    /*
    // LEDs
#ifdef SRF05EB_VERSION_1_3
    // SmartRF05EB rev 1.3 has only one accessible LED
    MCU_IO_DIR_OUTPUT(HAL_BOARD_IO_LED_1_PORT, HAL_BOARD_IO_LED_1_PIN);
    HAL_LED_CLR_1();
#else
    MCU_IO_DIR_OUTPUT(HAL_BOARD_IO_LED_1_PORT, HAL_BOARD_IO_LED_1_PIN);
    HAL_LED_CLR_1();
    MCU_IO_DIR_OUTPUT(HAL_BOARD_IO_LED_2_PORT, HAL_BOARD_IO_LED_2_PIN);
    HAL_LED_CLR_2();
    MCU_IO_DIR_OUTPUT(HAL_BOARD_IO_LED_3_PORT, HAL_BOARD_IO_LED_3_PIN);
    HAL_LED_CLR_3();
    MCU_IO_DIR_OUTPUT(HAL_BOARD_IO_LED_4_PORT, HAL_BOARD_IO_LED_4_PIN);
    HAL_LED_CLR_4();
#endif
*/
    /*
    // Buttons
    MCU_IO_INPUT(HAL_BOARD_IO_BTN_1_PORT, HAL_BOARD_IO_BTN_1_PIN, MCU_IO_TRISTATE);

    // Joystick push input
    MCU_IO_INPUT(HAL_BOARD_IO_JOY_MOVE_PORT, HAL_BOARD_IO_JOY_MOVE_PIN, \
    MCU_IO_TRISTATE);

    // Analog input
    MCU_IO_PERIPHERAL(HAL_BOARD_IO_JOYSTICK_ADC_PORT, HAL_BOARD_IO_JOYSTICK_ADC_PIN);

#ifndef LCD_EXCLUDE
    halLcdSpiInit();
    halLcdInit();
#endif
*/
    halIntOn();
}


/*******************************************************************************
* @fn          halLcdSpiInit
*
* @brief       Initalise LCD SPI interface
*
* @param       none
*
* @return      none
*/
void halLcdSpiInit(void) {
    register uint8 baud_exponent;
    register uint8 baud_mantissa;
    
    // Configure peripheral
# if (chip==2543)
    // Set SPI on UART 0 alternative 2
    PERCFG &= ~0x02;  
    PERCFG |= 0x01; 
# elif (chip==2545)
    // Set SPI on UART 0 alternative 3 (PERCFG: ---- --10)
    PERCFG &= ~0x01;
    PERCFG |= 0x02;
# else
    // Set SPI on UART 1 alternative 2
    PERCFG |= 0x02;
#endif
    
    // Use SPI on USART
    halLcdSpiEna();
   
    // Set SPI speed to 1 MHz (3 MHz is max on CC2511)
# if (chip==2430 || chip==2431)
    baud_exponent = 15 + CC2430_GET_CLKSPD();
#elif (chip==2530 || chip==2531 || chip==2533)
    baud_exponent = 15 + CC2530_GET_CLKSPD();
#elif (chip==2510 || chip==1110)
    baud_exponent = 15 + CLKSPD_GET();
#elif (chip==2541 || chip==2543 || chip==2545)
    baud_exponent = 15;
#endif
    baud_mantissa = 83;

    // Configure peripheral
# if ((chip==2543) || (chip==2545))
    U0UCR  = 0x80;      // Flush and goto IDLE state. 8-N-1.
    U0CSR  = 0x00;      // SPI mode, master.
    U0GCR  = SPI_TRANSFER_MSB_FIRST | SPI_CLOCK_PHA_0 | SPI_CLOCK_POL_LO | baud_exponent;
    U0BAUD = baud_mantissa;
# else   
    U1UCR  = 0x80;      // Flush and goto IDLE state. 8-N-1.
    U1CSR  = 0x00;      // SPI mode, master.
    U1GCR  = SPI_TRANSFER_MSB_FIRST | SPI_CLOCK_PHA_0 | SPI_CLOCK_POL_LO | baud_exponent;
    U1BAUD = baud_mantissa;
# endif 
}

/*******************************************************************************
* @fn          halLcdSpiEn
*
* @brief       Enable LCD SPI interface
*
* @param       none
*
* @return      none
*/
void halLcdSpiEna(void) {
    // Use SPI on USART 1 alternative 2
    MCU_IO_PERIPHERAL(HAL_BOARD_IO_SPI_MISO_PORT, HAL_BOARD_IO_SPI_MISO_PIN);
    MCU_IO_PERIPHERAL(HAL_BOARD_IO_SPI_MOSI_PORT, HAL_BOARD_IO_SPI_MOSI_PIN);
    MCU_IO_PERIPHERAL(HAL_BOARD_IO_SPI_CLK_PORT,  HAL_BOARD_IO_SPI_CLK_PIN);
}

/*******************************************************************************
* @fn          halLcdSpiDis
*
* @brief       Disable LCD SPI interface (set pins to input, except CS)
*
* @param       none
*
* @return      none
*/
void halLcdSpiDis(void) {
    MCU_IO_INPUT(HAL_BOARD_IO_SPI_MISO_PORT, HAL_BOARD_IO_SPI_MISO_PIN, MCU_IO_TRISTATE);
    MCU_IO_INPUT(HAL_BOARD_IO_SPI_MOSI_PORT, HAL_BOARD_IO_SPI_MOSI_PIN, MCU_IO_TRISTATE);
    MCU_IO_INPUT(HAL_BOARD_IO_SPI_CLK_PORT, HAL_BOARD_IO_SPI_CLK_PIN, MCU_IO_TRISTATE);
}