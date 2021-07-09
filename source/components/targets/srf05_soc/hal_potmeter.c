/*******************************************************************************
*  Filename:        hal_potmeter.c
*  Revised:         $Date: 2013-04-22 10:49:57 +0200 (ma, 22 apr 2013) $
*  Revision:        $Revision: 9900 $
*
*  Description:     HAL potmeter control.
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
#include "hal_potmeter.h"
#include "adc.h"
#include "hal_cc8051.h"
#include "hal_board.h"


/*******************************************************************************
* GLOBAL FUNCTIONS
*/
uint8 halPotmeterRead(void) {
    int16 adcValue;

    // Set potmeter ADC channel to input
    MCU_IO_INPUT(HAL_POTMETER_ADC_PORT, HAL_POTMETER_ADC_CH, MCU_IO_TRISTATE);

    adcValue = adcSampleSingle(ADC_REF_AVDD, ADC_9_BIT, HAL_POTMETER_ADC_CH);

    return (adcValue < 0 ? 0 : ((adcValue & 0x7FC0) >> 7));
}