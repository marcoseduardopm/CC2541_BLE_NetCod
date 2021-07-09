/*******************************************************************************
*  Filename:        util_lcd.c
*  Revised:         $Date: 2013-04-30 13:44:15 +0200 (ti, 30 apr 2013) $
*  Revision:        $Revision: 9937 $
*
*  Description:     Utility library for LCD control.
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
#include "hal_board.h"
#include "hal_lcd.h"
#include "hal_joystick.h"
#include "hal_button.h"
#include "hal_assert.h"
#include "hal_mcu.h"
#include "util_lcd.h"
#include "string.h"


#ifndef LCD_NOT_SUPPORTED
/*******************************************************************************
* LOCAL VARIABLES
*/
// Logo
#if !defined(SRF04EB) && !defined(ASSY_EXP4618_CC2420)

// For SRF05EB only
#define SRF05EB

const char XDATA ppBarGraphChar[8][8] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F },
    { 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F },
    { 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
    { 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
    { 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
    { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
};
#endif

static char pLcdLineBuffer[16];


/*******************************************************************************
* GLOBAL FUNCTIONS
*/

#ifdef ASSY_EXP4618_CC2420
extern void halLcd7SegWriteLine(const char *text);
#endif


/*******************************************************************************
* @fn          utilPrintText
*
* @brief       Prints a text string across all lines of the display. Newlines
*              cause continuation on the next line.
*
* @param       pTxt - text to display
*
* @param       n - number of characters to print
*
* @return      0
*/
void utilPrintText(uint8* pTxt, uint8 n) {
    uint8 li[3];
    uint8  i, iLine, nChars, nLines;

    // Display properties
    nLines= halLcdGetNumLines();
    nChars= nLines*halLcdGetLineLength();

    // Split string on newlines
    i= 0;
    iLine= 0;
    li[0]= 0;
    li[1]= 0xff;
    li[2]= 0xff;

    while(i<n && i<nChars && iLine<nLines) {
        if (pTxt[i]=='\n') {
            iLine++;
            li[iLine]= i+1;
            pTxt[i]= '\0';
        }
        i++;
    }

    // Display
	for (iLine=0; iLine<nLines; iLine++) {
		if (li[iLine]!=0xFF)
			halLcdWriteLine(HAL_LCD_LINE_1+iLine, (char XDATA*)pTxt + li[iLine] );
	}

}


#ifdef SRF05EB
/*******************************************************************************
* @fn         utilLoadBarGraph
*
* @brief      Load bar graph symbols on LCD. This function must be called before
*             utilLcdBarGraph can be used.
*
* @param      none
*
* @return     none
*/
void utilLoadBarGraph(void) {
	uint8 n;

    // Load the bar graph characters
    for (n = 0; n < 8; n++) {
        halLcdCreateSpecChar(n, ppBarGraphChar[n]);
    }
}


/*******************************************************************************
* @fn         utilDisplayBarGraph
*
* @brief      Display bar graph on LCD
*
* @param      uint8 line - line number
*             uint8 col - column number
*             uint8 min - minimum value
*             uint8 value - value to display
*
* @return     int8 - sampled RSSI value
*/
void utilDisplayBarGraph(uint8 line, uint8 col, uint8 min, uint8 value) {
    if (value <= min) {
        halLcdWriteChar(line, col, ' ');
    } else if (value >= min + 8) {
        halLcdWriteSpecChar(line, col, 7);
    } else {
        halLcdWriteSpecChar(line, col, value - min - 1);
    }
}
#endif


/*******************************************************************************
 * @fn          utilLcdDisplayValue
 *
 * @brief       Display value on display with optional text on right and left side
 *
 * @param       uint8 line
 *              char *pLeft
 *              int32 value
 *              char *pRight
 *
 * @return      none
 */
void utilLcdDisplayValue(uint8 line, char XDATA *pLeft, int32 value, char XDATA *pRight) {
    char *pValue;
    uint8 n;
    uint8 nChars;

    nChars= halLcdGetLineLength();
    pValue = convInt32ToText(value);

    for (n = 0; n < nChars; n++) {
        if (pLeft && *pLeft) {
            pLcdLineBuffer[n] = *(pLeft++);
        } else if (*pValue) {
            pLcdLineBuffer[n] = *(pValue++);
        } else if (pRight && *pRight) {
            pLcdLineBuffer[n] = *(pRight++);
        } else {
            pLcdLineBuffer[n] = ' ';
        }
    }
    halLcdWriteLine(line, (char XDATA *)pLcdLineBuffer);
}


/*******************************************************************************
 * @fn          utilLcdDisplayCounters
 *
 * @brief       Display counter values with text on display
 *
 * @param       uint8 line - display line
 *              int32 lValue - left value
 *              char lChar - left text
 *              int32 rValue - right value
 *              char rChar - right text
 *
 * @return      none
 */
void utilLcdDisplayCounters(uint8 line, int32 lValue, char lChar, int32 rValue, char rChar) {
    uint8 n;
    char *pCounter;
    uint8 nChars;

    nChars= halLcdGetLineLength();

    // Left part
    pLcdLineBuffer[0] = lChar;
    pLcdLineBuffer[1] = '=';
    pCounter = convInt32ToText(lValue);
    for (n = 2; n < 8; n++) {
        if (*pCounter) {
            pLcdLineBuffer[n] = *(pCounter++);
        } else {
            pLcdLineBuffer[n] = ' ';
        }
    }
    pLcdLineBuffer[8] = rChar;
    pLcdLineBuffer[9] = '=';
    pCounter = convInt32ToText(rValue);
    for (n = 10; n < nChars; n++) {
        if (*pCounter) {
            pLcdLineBuffer[n] = *(pCounter++);
        } else {
            pLcdLineBuffer[n] = ' ';
        }
    }
    halLcdWriteLine(line, (char XDATA *)pLcdLineBuffer);
}



static const char hex[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
/*******************************************************************************
 * @fn          utilLcdDisplayUint16
 *
 * @brief       Write a 16 bit value on the specified line on the LCD
 *
 * @param       uint8 line - line on display
 *              uint8 radix - HAL_LCD_RADIX_DEC or HAL_LCD_RADIX_HEX
 *              uint16 value - value to display
 *
 * @return      none
 */
void utilLcdDisplayUint16(uint8 line, uint8 radix, uint16 value) {
    if (radix == HAL_LCD_RADIX_DEC) {
        uint8 n = 0;
        uint8 c = 5;
        char *pValue = convInt32ToText((uint32)value);
        while (pValue[n]) n++;
        while (n-- && c)
            halLcdWriteChar(line, --c, pValue[n]);
        while (c)
            halLcdWriteChar(line, --c, ' ');
    }
    else if (radix == HAL_LCD_RADIX_HEX) {
        halLcdWriteChar(line, 0, '0');
        halLcdWriteChar(line, 1, 'x');
        halLcdWriteChar(line, 2, hex[(value & 0xF000) >> 12]);
        halLcdWriteChar(line, 3, hex[(value & 0x0F00) >>  8]);
        halLcdWriteChar(line, 4, hex[(value & 0x00F0) >>  4]);
        halLcdWriteChar(line, 5, hex[(value & 0x000F)]);
    }
}

#else

void utilPrintText(uint8* pTxt, uint8 n) { }

#endif