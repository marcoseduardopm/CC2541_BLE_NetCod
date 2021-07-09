/***********************************************************************************

  Filename:       hal_rf_settings.c

  Description:    Register settings

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "hal_board.h"
#include "prop_regs.h"

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

void halRfInitRegs(void)
{
#if (chip == 2541)
  /* RF settings SoC: CC2541 */

  /* Setting: GFSK 2Mb, 500KHz deviation */

  PRF_CHAN           = 0x29; //
  PRF_TASK_CONF      = 0x01; // configuration of task control
  PRF_FIFO_CONF      = 0x09; // configure fifo use.
  PRF_PKT_CONF       = 0x06; // packet configuration
  PRF_CRC_LEN        = 0x02; // crc length.
  PRF_CRC_INIT[2]    = 0xFF; // initialization value for crc.
  PRF_CRC_INIT[3]    = 0xFF; // initialization value for crc.
  PRF_W_INIT         = 0xFF; // whitener initialization.
  PRF_ADDR_ENTRY0[0] = 0x01; // address entry 0, index 0. basic mode: conf. see user guide for details on auto mode.
  PRF_ADDR_ENTRY0[1] = 0x40; // address entry 0, index 1. basic mode: rxlength
  PRF_RADIO_CONF     = 0xD0; // configure radio hardware.
  FRMCTRL0           = 0x40; //
  TXPOWER            = 0xE1; // output power control
  TXCTRL             = 0x19; // tx settings
  MDMCTRL0           = 0x06; // modem configuration
  MDMCTRL1           = 0x48; // modem configuration
  MDMCTRL2           = 0x4C; // modem configuration
  MDMCTRL3           = 0x63; // modem configuration
  SW0                = 0x29; // primary sync word byte 0
  SW1                = 0x41; // primary sync word byte 1
  SW2                = 0x76; // primary sync word byte 2
  SW3                = 0x71; // primary sync word byte 3
  FSCTRL             = 0x5A; // frequency synthesizer tuning
  ADCTEST0           = 0x66; //
  MDMTEST0           = 0x01; // modem configuration
  TXFILTCFG          = 0x03; // tx filter configuration
  BSP_P1             = 0x00; // crc polynomial byte 1
  BSP_P2             = 0x21; // crc polynomial byte 2
  BSP_P3             = 0x10; // crc polynomial byte 3
  BSP_MODE           = 0x00; // bit stream processor configuration
  IVCTRL             = 0x1B; // analog control register
#endif
}


void halRfSetRadioFrequency(uint16 frequency)
{
  uint8 freqOffset;

#if (chip == 2541)
   freqOffset = frequency - 2379;
   FREQCTRL = freqOffset + 1;
   PRF_CHAN = freqOffset;
#endif

   return;
}