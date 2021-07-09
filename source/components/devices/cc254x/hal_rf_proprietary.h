/*******************************************************************************
*  Filename:        hal_rf_proprietary.h
*  Revised:         $Date: 2013-04-23 20:03:59 +0200 (ti, 23 apr 2013) $
*  Revision:        $Revision: 9905 $
*
*  Description:     HAL radio interface header file for the CC254x devices used 
*                   in proprietary mode.
*
*  Warning:         This hal interface does not cover all possible uses.
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


#ifndef HAL_RF_PROPRIETARY_H
#define HAL_RF_PROPRIETARY_H
/*******************************************************************************
* INCLUDES
*/
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

#include "hal_defs.h"
#include "hal_types.h"
#include "hal_mcu.h"
#include "prop_regs.h"
#include "string.h"

/*******************************************************************************
* TYPEDEFS
*/

/*******************************************************************************
* CONSTANTS AND DEFINES
*/
// Available modulation formats.
#define GFSK_250kbps_160khz 4
#define GFSK_1Mbps_160khz 6
#define GFSK_1Mbps_250khz 2
#define GFSK_2Mbps_320khz 7
#define GFSK_2Mbps_500khz 3
#define MSK_250kbps 8
#define MSK_500kbps 9

#define RUN_SUCCESS 0
#define RUN_ERROR   9

#define TX 1
#define RX 0

// Define force parameter.
#define FORCE   1
#define ATTEMPT 0

// Definitions for packet configuration
#define PKT_CONF_MODE               0x03 // Bit mask for mode bits
#define PKT_CONF_MODE_BASIC_FIXLEN  0x00
#define PKT_CONF_MODE_BASIC_VARLEN  0x01
#define PKT_CONF_MODE_AUTO_9BIT     0x02
#define PKT_CONF_MODE_AUTO_10BIT    0x03
#define PKT_CONF_ADDR               0x04 // Bit mask for address byte
#define PKT_CONF_PHASE_INV          0x08 // Bit mask for phase invert
#define PKT_CONF_INC_CRC            0x10 // Bit mask for including SW in CRC

// Compatibility modes
#define PKT_CONF_NRF (PKT_CONF_MODE_AUTO_9BIT | PKT_CONF_ADDR | PKT_CONF_INC_CRC)
#define PKT_CONF_NRF_COMPAT (PKT_CONF_MODE_BASIC_FIXLEN | PKT_CONF_ADDR | PKT_CONF_INC_CRC)
#define PKT_CONF_CC25_FIXLEN_NOADDR (PKT_CONF_MODE_BASIC_FIXLEN | PKT_CONF_PHASE_INV)
#define PKT_CONF_CC25_FIXLEN_ADDR (PKT_CONF_MODE_BASIC_FIXLEN | PKT_CONF_ADDR | PKT_CONF_PHASE_INV)
#define PKT_CONF_CC25_VARLEN_NOADDR (PKT_CONF_MODE_BASIC_VARLEN | PKT_CONF_PHASE_INV)
#define PKT_CONF_CC25_VARLEN_ADDR (PKT_CONF_MODE_BASIC_VARLEN | PKT_CONF_ADDR | PKT_CONF_PHASE_INV)

// Definitions for address configurations
#define ADDR_CONF_RX_AUTO_SW1       0x19
#define ADDR_CONF_RX_AUTO_SW2       0x1A
#define ADDR_CONF_RX_BASIC_SW1      0x01
#define ADDR_CONF_RX_BASIC_SW2      0x02
#define ADDR_CONF_TX_AUTO           0x18
#define ADDR_CONF_TX_BASIC          0x00

// TXPOWER values
#if (chip == 2541)
#define TXPOWER_0_DBM           0xE1
#define TXPOWER_MINUS_2_DBM     0xD1
#define TXPOWER_MINUS_4_DBM     0xC1
#define TXPOWER_MINUS_6_DBM     0xB1
#define TXPOWER_MINUS_8_DBM     0xA1
#define TXPOWER_MINUS_10_DBM    0x91
#define TXPOWER_MINUS_12_DBM    0x81
#define TXPOWER_MINUS_14_DBM    0x71
#define TXPOWER_MINUS_16_DBM    0x61
#define TXPOWER_MINUS_18_DBM    0x51
#define TXPOWER_MINUS_20_DBM    0x41
#else
#define TXPOWER_4_DBM           0xE5
#define TXPOWER_0_DBM           0xA5
#define TXPOWER_MINUS_6_DBM     0x65
#define TXPOWER_MINUS_12_DBM    0x45
#define TXPOWER_MINUS_15_DBM    0x25
#define TXPOWER_MINUS_20_DBM    0x05
#endif

#if (chip == 2543) || (chip == 2545)
// Uncalibrated accuracy of 4 dB. ( Valid range: 0 to 64). 
#define RSSI_OFFSET_HIGH_GAIN       99
#define RSSI_OFFSET_LOW_GAIN        79
#elif (chip == 2544)
// Uncalibrated accuracy of +/- 6 dB. ( Valid range: 0 to 60). 
#define RSSI_OFFSET_HIGH_GAIN       97
#define RSSI_OFFSET_LOW_GAIN        77
#elif (chip == 2541)
#define RSSI_OFFSET_HIGH_GAIN       107
#define RSSI_OFFSET_LOW_GAIN        98
#define RSSI_OFFSET_AGC_HIGH_GAIN   99
#define RSSI_OFFSET_AGC_LOW_GAIN    79
#endif

// Bitmask for radio register RFIRQF1.
#define RFIRQF1_PINGRSP             0x80
#define RFIRQF1_TASKDONE            0x40
#define RFIRQF1_TXDONE              0x20
#define RFIRQF1_RXEMPTY             0x10
#define RFIRQF1_RXIGNORED           0x08
#define RFIRQF1_RXNOK               0x04
#define RFIRQF1_TXFLUSHED           0x02
#define RFIRQF1_RXOK                0x01

// Bitmask for radio register RFSTAT.
#define RFSTAT_MOD_UNDERFLOW        0x80
#define RFSTAT_DEM_STATUS           0x60
#define RFSTAT_SFD                  0x10
#define RFSTAT_CAL_RUNNING          0x08
#define RFSTAT_LOCK_STATUS          0x04
#define RFSTAT_TX_ACTIVE            0x02
#define RFSTAT_RX_ACTIVE            0x01

// Bitmask for radio register LLESTAT.
#define LLESTAT_AGC_LOWGAIN         0x10
#define LLESTAT_WAIT_T2E1           0x08
#define LLESTAT_LLE_IDLE            0x04
#define LLESTAT_SYNC_SEARCH         0x02
#define LLESTAT_VCO_ON              0x01

// Bitmask for radio register SW_CONF
#define SW_CONF_DUAL_RX     0x80
#define SW_CONF_SW_RX       0x20
#define SW_CONF_SW_LEN      0x1F

// Bitmask for radio register MDMCTRL1
#define MDMCTRL1_FOC_MODE   0xC0
#define MDMCTRL1_CORR_THR   0x1F

// Bitmask for radio register RFFSTATUS
#define TXAVAIL             0x80
#define TXFEMPTY            0x40
#define TXDTHEX             0x20
#define TXFFULL             0x10
#define RXAVAIL             0x08
#define RXFEMPTY            0x04
#define RXDTHEX             0x02
#define RXFFULL             0x01

// Bitmask for radio register MDMTEST0
#define MDMTEST0_RSSI_ACC        0xE0
#define MDMTEST0_DC_BLOCK_LENGTH 0x0C
#define MDMTEST0_DC_BLOCK_MODE   0x03

// IP0 (0xA9) – Interrupt Priority 0
#define IP0_ST_P0INT_WDT                0x20
#define IP0_ENC_T4_P1INT                0x10
#define IP0_T3_I2C                      0x08
#define IP0_URX0_T2_UTX0                0x04
#define IP0_ADC_T1_P2INT_USB            0x02
#define IP0_RFERR_RF_DMA                0x01

// IP1 (0xB9) – Interrupt Priority 0
#define IP1_ST_P0INT_WDT                0x20
#define IP1_ENC_T4_P1INT                0x10
#define IP1_T3_I2C                      0x08
#define IP1_URX0_T2_UTX0                0x04
#define IP1_ADC_T1_P2INT_USB            0x02
#define IP1_RFERR_RF_DMA                0x01

// LLECTRL (0x61B1) – LLE Control
#define LLECTRL_LLE_MODE_SEL            0x06
#define LLECTRL_LLE_EN                  0x01

// RFC_OBS_CTRL0 (0x61AE) – RF Observation Mux Control 0
#define RFC_OBS_POL0            0x40
#define RFC_OBS_MUX0            0x3F
    #define RFC_SNIFF_DATA          0x07
    #define RFC_SNIFF_CLK           0x08
    #define TX_ACTIVE               0x09
    #define RX_ACTIVE               0x0A
    #define VCO_ON                  0x0B
    #define SYNCH_SEARCH            0x0C
    #define LLE_IDLE                0x0D
    #define WAIT_T2E1               0x0E
    #define AGC_LOWGAIN             0x0F
    #define FSC_LOCK                0x13
    #define PA_PD                   0x1B
    #define LNAMIX_PD               0x2C
    #define DEMO_SYNC_FOUND         0x30
    #define MOD_SYNC_SENT           0x31


// RSSI accuracy setting definitions
#define ONE_5_33_US_AVERAGE      0x00
#define TWO_5_33_US_AVERAGE      0x01
#define FOUR_5_33_US_AVERAGE     0x03
#define ONE_21_3_US_AVERAGE      0x04
#define TWO_21_3_US_AVERAGE      0x05
#define FOUR_21_3_US_AVERAGE     0x07

// Gain modes
#define HAL_RF_GAIN_LOW                     0
#define HAL_RF_GAIN_HIGH                    1
#define HAL_RF_GAIN_2MBPS                   2

// Default Packet format values. 
#define LENGTH      15
#define CRCLEN      2

#define SEND_LLE_CMD(cmd)      st(while(RFST != 0); RFST = (cmd);)
#define SEND_FIFO_CMD(cmd)     st(RFST = (cmd);)


typedef enum {
    CRC_8_ATM,      // 8-bit CRC from nRF
    CRC_16,         // 16-bit CRC from CC2500/CC251x
    CRC_16_CCITT,   // 16-bit CRC from nRF
    CRC_24,
    CRC_32
} crc_conf_t;


/*******************************************************************************
* GLOBAL FUNCTIONS
*/
void halRfConfig(uint8 modulation, uint8 tx, uint8 sw_len, uint8 pkt_conf);
void halRfDisableInterrupt(uint8 mask);
void halRfEnableInterrupt(uint8 mask);
void halRfEnableRadio(void);
uint8 halRfDisableRadio(uint8 forceReset);
void halRfInit(void);
void halRfStartRx(void);
void halRfStopRx(void);
void halRfStartTx(void);
void halRfSetTurnaroundRx(uint16 turn_us);
void halRfSetTurnaroundTx(uint16 turn_us);
void halRfSetRetrans(uint8 num_retrans, uint16 retrans_delay_us);
void halRfSetFrequency(uint16 frequency);
int8 halRfGetLastRssi(void);
int8 halRfGetRssi(uint8 accuracy);
void halRfSetGain(uint8 gainMode);
void halRfSetRxAddress(uint8 entry_no, uint8 confval, uint8 address, uint8 maxlen);
void halRfSetTxAddress(uint8 address, uint8 confval, uint8 maxacklen);
void halRfSetCrc(crc_conf_t crc_conf);
void halRfSetWhitening(uint8 whitening);
void halRfSetSyncWord(uint32 sw, uint32 sw2);
void halRfRecalibrateRx(void);
void halRfPacketTxInterval(uint32 us);
void halRfCommand(uint8 command);
void halRfDiscardTxPacket(void);
uint8 halRfReadRxPacket(uint8 *pBuf, uint8 maxLen);
uint8 halRfPrepareTxPacket(uint8 len, uint8 *data_ptr);
void  halRfLoadPERPacketPayload(uint8 packet_length, uint32 seq_packet);
uint8 halRfLoadAckPayload(uint8 ack_size, uint8 entry_no, uint8 *ack_payload_pointer);
void halRfEnableDebugSignalOnObssel1(uint8 obsselSignal);

#endif