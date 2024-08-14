/*!
 * @file      SX126X_radio_types.h
 *
 * @brief     Radio driver types for SX126x
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#define SX126X_XTAL_FREQ 32000000
#define FREQ_STEP 61.03515625 // TODO check and fix... this is a cut and past from sx1276

#define SX126X_POWER_MIN_LP_PA (-17) // Low Power PA
#define SX126X_POWER_MAX_LP_PA  (14)
#define SX126X_POWER_MIN_HP_PA  (-9) // High Power PA
#define SX126X_POWER_MAX_HP_PA  (22)
#define SX126X_POWER_MIN_HF_PA (-18) // High Frequency PA
#define SX126X_POWER_MAX_HF_PA  (13)

#define SX126X_IRQ_TX_DONE 0x00000004
#define SX126X_IRQ_RX_DONE 0x00000008
#define SX126X_IRQ_RADIO_NONE 0
// #define SX126X_IRQ_RADIO_ALL 0xFFFFFFFF

/*!
 * @brief Length in byte of the SX126X version blob
 */
#define SX126X_BL_VERSION_LENGTH ( 4 )

/*!
 * @brief Length in bytes of a PIN
 */
#define SX126X_BL_PIN_LENGTH ( 4 )

/*!
 * @brief Length in bytes of a chip EUI
 */
#define SX126X_BL_CHIP_EUI_LENGTH ( 8 )

/*!
 * @brief Length in bytes of a join EUI
 */
#define SX126X_BL_JOIN_EUI_LENGTH ( 8 )

typedef enum
{
    SX126X_LORA_PACKET_VARIABLE_LENGTH = 0x00, //!< The packet is on variable size, header included
    SX126X_LORA_PACKET_FIXED_LENGTH = 0x01,    //!< The packet is known on both sides, no header included in the packet
    SX126X_LORA_PACKET_EXPLICIT = SX126X_LORA_PACKET_VARIABLE_LENGTH,
    SX126X_LORA_PACKET_IMPLICIT = SX126X_LORA_PACKET_FIXED_LENGTH,
} SX126X_RadioLoRaPacketLengthsModes_t;

typedef enum
{
    SX126X_MODE_SLEEP = 0x00, //! The radio is in sleep mode
    SX126X_MODE_STDBY_RC,     //! The radio is in standby mode with RC oscillator
    SX126X_MODE_STDBY_XOSC,   //! The radio is in standby mode with XOSC oscillator
    SX126X_MODE_FS,           //! The radio is in frequency synthesis mode
    SX126X_MODE_RX,           //! The radio is in receive mode
    SX126X_MODE_RX_CONT,      //! The radio is in continuous receive mode
    SX126X_MODE_TX,           //! The radio is in transmit mode
    SX126X_MODE_CAD           //! The radio is in channel activity detection mode
} SX126X_RadioOperatingModes_t;

enum
{
    SX126X_RADIO_RESET_STATS_OC               = 0x0200,
    SX126X_RADIO_GET_STATS_OC                 = 0x0201,
    SX126X_RADIO_GET_PKT_TYPE_OC              = 0x0202,
    SX126X_RADIO_GET_RXBUFFER_STATUS_OC       = 0x0203,
    SX126X_RADIO_GET_PKT_STATUS_OC            = 0x0204,
    SX126X_RADIO_GET_RSSI_INST_OC             = 0x0205,
    SX126X_RADIO_SET_GFSK_SYNC_WORD_OC        = 0x0206,
    SX126X_RADIO_SET_LORA_PUBLIC_NETWORK_OC   = 0x0208,
    SX126X_RADIO_SET_RX_OC                    = 0x0209,
    SX126X_RADIO_SET_TX_OC                    = 0x020A,
    SX126X_RADIO_SET_RF_FREQUENCY_OC          = 0x020B,
    SX126X_RADIO_AUTOTXRX_OC                  = 0x020C,
    SX126X_RADIO_SET_CAD_PARAMS_OC            = 0x020D,
    SX126X_RADIO_SET_PKT_TYPE_OC              = 0x020E,
    SX126X_RADIO_SET_MODULATION_PARAM_OC      = 0x020F,
    SX126X_RADIO_SET_PKT_PARAM_OC             = 0x0210,
    SX126X_RADIO_SET_TX_PARAMS_OC             = 0x0211,
    SX126X_RADIO_SET_PKT_ADRS_OC              = 0x0212,
    SX126X_RADIO_SET_RX_TX_FALLBACK_MODE_OC   = 0x0213,
    SX126X_RADIO_SET_RX_DUTY_CYCLE_OC         = 0x0214,
    SX126X_RADIO_SET_PA_CFG_OC                = 0x0215,
    SX126X_RADIO_STOP_TIMEOUT_ON_PREAMBLE_OC  = 0x0217,
    SX126X_RADIO_SET_CAD_OC                   = 0x0218,
    SX126X_RADIO_SET_TX_CW_OC                 = 0x0219,
    SX126X_RADIO_SET_TX_INFINITE_PREAMBLE_OC  = 0x021A,
    SX126X_RADIO_SET_LORA_SYNC_TIMEOUT_OC     = 0x021B,
    SX126X_RADIO_SET_GFSK_CRC_PARAMS_OC       = 0x0224,
    SX126X_RADIO_SET_GFSK_WHITENING_PARAMS_OC = 0x0225,
    SX126X_RADIO_SET_RX_BOOSTED_OC            = 0x0227,
    SX126X_RADIO_SET_RSSI_CALIBRATION_OC      = 0x0229,
    SX126X_RADIO_SET_LORA_SYNC_WORD_OC        = 0x022B,
    SX126X_RADIO_SET_LR_FHSS_SYNC_WORD_OC     = 0x022D,
    SX126X_RADIO_CFG_BLE_BEACON_OC            = 0x022E,
    SX126X_RADIO_GET_LORA_RX_INFO_OC          = 0x0230,
    SX126X_RADIO_BLE_BEACON_SEND_OC           = 0x0231,
};

enum
{
    SX126X_SYSTEM_GET_STATUS_OC              = 0x0100,
    SX126X_SYSTEM_GET_VERSION_OC             = 0x0101,
    SX126X_SYSTEM_GET_ERRORS_OC              = 0x010D,
    SX126X_SYSTEM_CLEAR_ERRORS_OC            = 0x010E,
    SX126X_SYSTEM_CALIBRATE_OC               = 0x010F,
    SX126X_SYSTEM_SET_REGMODE_OC             = 0x0110,
    SX126X_SYSTEM_CALIBRATE_IMAGE_OC         = 0x0111,
    SX126X_SYSTEM_SET_DIO_AS_RF_SWITCH_OC    = 0x0112,
    SX126X_SYSTEM_SET_DIOIRQPARAMS_OC        = 0x0113,
    SX126X_SYSTEM_CLEAR_IRQ_OC               = 0x0114,
    SX126X_SYSTEM_CFG_LFCLK_OC               = 0x0116,
    SX126X_SYSTEM_SET_TCXO_MODE_OC           = 0x0117,
    SX126X_SYSTEM_REBOOT_OC                  = 0x0118,
    SX126X_SYSTEM_GET_VBAT_OC                = 0x0119,
    SX126X_SYSTEM_GET_TEMP_OC                = 0x011A,
    SX126X_SYSTEM_SET_SLEEP_OC               = 0x011B,
    SX126X_SYSTEM_SET_STANDBY_OC             = 0x011C,
    SX126X_SYSTEM_SET_FS_OC                  = 0x011D,
    SX126X_SYSTEM_GET_RANDOM_OC              = 0x0120,
    SX126X_SYSTEM_ERASE_INFOPAGE_OC          = 0x0121,
    SX126X_SYSTEM_WRITE_INFOPAGE_OC          = 0x0122,
    SX126X_SYSTEM_READ_INFOPAGE_OC           = 0x0123,
    SX126X_SYSTEM_READ_UID_OC                = 0x0125,
    SX126X_SYSTEM_READ_JOIN_EUI_OC           = 0x0126,
    SX126X_SYSTEM_READ_PIN_OC                = 0x0127,
    SX126X_SYSTEM_ENABLE_SPI_CRC_OC          = 0x0128,
    SX126X_SYSTEM_DRIVE_DIO_IN_SLEEP_MODE_OC = 0x012A,
};

enum
{
    SX126X_REGMEM_WRITE_REGMEM32_OC      = 0x0105,
    SX126X_REGMEM_READ_REGMEM32_OC       = 0x0106,
    SX126X_REGMEM_WRITE_MEM8_OC          = 0x0107,
    SX126X_REGMEM_READ_MEM8_OC           = 0x0108,
    SX126X_REGMEM_WRITE_BUFFER8_OC       = 0x0109,
    SX126X_REGMEM_READ_BUFFER8_OC        = 0x010A,
    SX126X_REGMEM_CLEAR_RXBUFFER_OC      = 0x010B,
    SX126X_REGMEM_WRITE_REGMEM32_MASK_OC = 0x010C,
};

enum
{
    SX126X_BL_GET_STATUS_OC            = 0x0100,
    SX126X_BL_GET_VERSION_OC           = 0x0101,
    SX126X_BL_ERASE_FLASH_OC           = 0x8000,
    SX126X_BL_WRITE_FLASH_ENCRYPTED_OC = 0x8003,
    SX126X_BL_REBOOT_OC                = 0x8005,
    SX126X_BL_GET_PIN_OC               = 0x800B,
    SX126X_BL_READ_CHIP_EUI_OC         = 0x800C,
    SX126X_BL_READ_JOIN_EUI_OC         = 0x800D,
};

typedef enum
{
    SX126X_RADIO_PA_SEL_LP = 0x00,  //!< Low-power Power Amplifier
    SX126X_RADIO_PA_SEL_HP = 0x01,  //!< High-power Power Amplifier
    SX126X_RADIO_PA_SEL_HF = 0x02,  //!< High-frequency Power Amplifier
} SX126X_radio_pa_selection_t;

typedef enum
{
    SX126X_RADIO_FALLBACK_STDBY_RC   = 0x01,  //!< Standby RC (Default)
    SX126X_RADIO_FALLBACK_STDBY_XOSC = 0x02,  //!< Standby XOSC
    SX126X_RADIO_FALLBACK_FS         = 0x03   //!< FS
} SX126X_radio_fallback_modes_t;

typedef enum
{
    SX126X_RADIO_RAMP_16_US  = 0x00,  //!< 16 us Ramp Time
    SX126X_RADIO_RAMP_32_US  = 0x01,  //!< 32 us Ramp Time
    SX126X_RADIO_RAMP_48_US  = 0x02,  //!< 48 us Ramp Time (Default)
    SX126X_RADIO_RAMP_64_US  = 0x03,  //!< 64 us Ramp Time
    SX126X_RADIO_RAMP_80_US  = 0x04,  //!< 80 us Ramp Time
    SX126X_RADIO_RAMP_96_US  = 0x05,  //!< 96 us Ramp Time
    SX126X_RADIO_RAMP_112_US = 0x06,  //!< 112 us Ramp Time
    SX126X_RADIO_RAMP_128_US = 0x07,  //!< 128 us Ramp Time
    SX126X_RADIO_RAMP_144_US = 0x08,  //!< 144 us Ramp Time
    SX126X_RADIO_RAMP_160_US = 0x09,  //!< 160 us Ramp Time
    SX126X_RADIO_RAMP_176_US = 0x0A,  //!< 176 us Ramp Time
    SX126X_RADIO_RAMP_192_US = 0x0B,  //!< 192 us Ramp Time
    SX126X_RADIO_RAMP_208_US = 0x0C,  //!< 208 us Ramp Time
    SX126X_RADIO_RAMP_240_US = 0x0D,  //!< 240 us Ramp Time
    SX126X_RADIO_RAMP_272_US = 0x0E,  //!< 272 us Ramp Time
    SX126X_RADIO_RAMP_304_US = 0x0F,  //!< 304 us Ramp Time
} SX126X_radio_ramp_time_t;

typedef enum
{
    SX126X_RADIO_LORA_NETWORK_PRIVATE = 0x00,  //!< LoRa private network
    SX126X_RADIO_LORA_NETWORK_PUBLIC  = 0x01,  //!< LoRa public network
} SX126X_radio_lora_network_type_t;

typedef enum
{
    SX126X_RADIO_LORA_SF5  = 0x05,  //!< Spreading Factor 5
    SX126X_RADIO_LORA_SF6  = 0x06,  //!< Spreading Factor 6
    SX126X_RADIO_LORA_SF7  = 0x07,  //!< Spreading Factor 7
    SX126X_RADIO_LORA_SF8  = 0x08,  //!< Spreading Factor 8
    SX126X_RADIO_LORA_SF9  = 0x09,  //!< Spreading Factor 9
    SX126X_RADIO_LORA_SF10 = 0x0A,  //!< Spreading Factor 10
    SX126X_RADIO_LORA_SF11 = 0x0B,  //!< Spreading Factor 11
    SX126X_RADIO_LORA_SF12 = 0x0C,  //!< Spreading Factor 12
} SX126X_radio_lora_sf_t;

typedef enum
{
    SX126X_RADIO_LORA_BW_10  = 0x08,  //!< Bandwidth 10.42 kHz
    SX126X_RADIO_LORA_BW_15  = 0x01,  //!< Bandwidth 15.63 kHz
    SX126X_RADIO_LORA_BW_20  = 0x09,  //!< Bandwidth 20.83 kHz
    SX126X_RADIO_LORA_BW_31  = 0x02,  //!< Bandwidth 31.25 kHz
    SX126X_RADIO_LORA_BW_41  = 0x0A,  //!< Bandwidth 41.67 kHz
    SX126X_RADIO_LORA_BW_62  = 0x03,  //!< Bandwidth 62.50 kHz
    SX126X_RADIO_LORA_BW_125 = 0x04,  //!< Bandwidth 125.00 kHz
    SX126X_RADIO_LORA_BW_250 = 0x05,  //!< Bandwidth 250.00 kHz
    SX126X_RADIO_LORA_BW_500 = 0x06,  //!< Bandwidth 500.00 kHz
    SX126X_RADIO_LORA_BW_200 = 0x0D,  //!< Bandwidth 203.00 kHz, 2G4 and compatible with LR112x chips only
    SX126X_RADIO_LORA_BW_400 = 0x0E,  //!< Bandwidth 406.00 kHz, 2G4 and compatible with LR112x chips only
    SX126X_RADIO_LORA_BW_800 = 0x0F,  //!< Bandwidth 812.00 kHz, 2G4 and compatible with LR112x chips only
} SX126X_radio_lora_bw_t;

typedef enum
{
    SX126X_RADIO_LORA_NO_CR     = 0x00,  //!< No Coding Rate
    SX126X_RADIO_LORA_CR_4_5    = 0x01,  //!< Coding Rate 4/5 Short Interleaver
    SX126X_RADIO_LORA_CR_4_6    = 0x02,  //!< Coding Rate 4/6 Short Interleaver
    SX126X_RADIO_LORA_CR_4_7    = 0x03,  //!< Coding Rate 4/7 Short Interleaver
    SX126X_RADIO_LORA_CR_4_8    = 0x04,  //!< Coding Rate 4/8 Short Interleaver
    SX126X_RADIO_LORA_CR_LI_4_5 = 0x05,  //!< Coding Rate 4/5 Long Interleaver
    SX126X_RADIO_LORA_CR_LI_4_6 = 0x06,  //!< Coding Rate 4/6 Long Interleaver
    SX126X_RADIO_LORA_CR_LI_4_8 = 0x07,  //!< Coding Rate 4/8 Long Interleaver
} SX126X_radio_lora_cr_t;

typedef enum
{
    SX126X_RADIO_MODE_SLEEP = 0x00,  //!< Sleep / Not recommended with LR1110 FW from 0x0303 to 0x0307 and LR1120 FW
                                     //!< 0x0101 in case of transition from Rx to Tx in LoRa
    SX126X_RADIO_MODE_STANDBY_RC   = 0x01,  //!< Standby RC
    SX126X_RADIO_MODE_STANDBY_XOSC = 0x02,  //!< Standby XOSC
    SX126X_RADIO_MODE_FS           = 0x03   //!< Frequency Synthesis
} SX126X_radio_intermediary_mode_t;

typedef enum
{
    SX126X_RADIO_LORA_CRC_OFF = 0x00,  //!< CRC deactivated
    SX126X_RADIO_LORA_CRC_ON  = 0x01,  //!< CRC activated
} SX126X_radio_lora_crc_t;

typedef enum
{
    SX126X_RADIO_LORA_PKT_EXPLICIT = 0x00,  //!< Explicit header: transmitted over the air
    SX126X_RADIO_LORA_PKT_IMPLICIT = 0x01,  //!< Implicit header: not transmitted over the air
} SX126X_radio_lora_pkt_len_modes_t;

typedef enum
{
    SX126X_RADIO_LORA_IQ_STANDARD = 0x00,  //!< IQ standard
    SX126X_RADIO_LORA_IQ_INVERTED = 0x01,  //!< IQ inverted
} SX126X_radio_lora_iq_t;

typedef enum
{
    SX126X_RADIO_PKT_NONE         = 0x00,  //!< State after cold start, Wi-Fi or GNSS capture
    SX126X_RADIO_PKT_TYPE_GFSK    = 0x01,  //!< GFSK modulation
    SX126X_RADIO_PKT_TYPE_LORA    = 0x02,  //!< LoRa modulation
    SX126X_RADIO_PKT_TYPE_BPSK    = 0x03,  //!< BPSK modulation
    SX126X_RADIO_PKT_TYPE_LR_FHSS = 0x04,  //!< LR-FHSS modulation
    SX126X_RADIO_PKT_TYPE_RANGING = 0x05,  //!< Ranging packet
} SX126X_radio_pkt_type_t;

typedef enum
{
    SX126X_RADIO_PA_REG_SUPPLY_VREG = 0x00,  //!< Power amplifier supplied by the main regulator
    SX126X_RADIO_PA_REG_SUPPLY_VBAT = 0x01   //!< Power amplifier supplied by the battery
} SX126X_radio_pa_reg_supply_t;

typedef enum
{
    SX126X_RADIO_RX_DUTY_CYCLE_MODE_RX  = 0x00,  //!< LoRa/GFSK: Uses Rx for listening to packets
    SX126X_RADIO_RX_DUTY_CYCLE_MODE_CAD = 0x01,  //!< Only in LoRa: Uses CAD to listen for over-the-air activity
} SX126X_radio_rx_duty_cycle_mode_t;

typedef enum
{
    SX126X_RADIO_GFSK_CRC_OFF         = 0x01,  //!< CRC check deactivated
    SX126X_RADIO_GFSK_CRC_1_BYTE      = 0x00,
    SX126X_RADIO_GFSK_CRC_2_BYTES     = 0x02,
    SX126X_RADIO_GFSK_CRC_1_BYTE_INV  = 0x04,
    SX126X_RADIO_GFSK_CRC_2_BYTES_INV = 0x06,
} SX126X_radio_gfsk_crc_type_t;

typedef enum
{
    SX126X_RADIO_GFSK_DC_FREE_OFF                   = 0x00,  //!< Whitening deactivated
    SX126X_RADIO_GFSK_DC_FREE_WHITENING             = 0x01,  //!< Whitening enabled
    SX126X_RADIO_GFSK_DC_FREE_WHITENING_SX128X_COMP = 0x03,  //!< Whitening enabled - SX128x compatibility
} SX126X_radio_gfsk_dc_free_t;

typedef enum
{
    SX126X_RADIO_GFSK_PKT_FIX_LEN = 0x00,  //!< Payload length is not sent/read over the air
    SX126X_RADIO_GFSK_PKT_VAR_LEN = 0x01,  //!< Payload length is sent/read over the air
    SX126X_RADIO_GFSK_PKT_VAR_LEN_SX128X_COMP =
        0x02,  //!< Payload length is sent/read over the air - SX128x compatibility
} SX126X_radio_gfsk_pkt_len_modes_t;

typedef enum
{
    SX126X_RADIO_GFSK_ADDRESS_FILTERING_DISABLE      = 0x00,  //!< Filter deactivated
    SX126X_RADIO_GFSK_ADDRESS_FILTERING_NODE_ADDRESS = 0x01,  //!< Filter on Node Address
    SX126X_RADIO_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES =
        0x02,  //!< Filtering on Node and Broadcast addresses
} SX126X_radio_gfsk_address_filtering_t;

typedef enum
{
    SX126X_RADIO_GFSK_PREAMBLE_DETECTOR_OFF        = 0x00,
    SX126X_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS  = 0x04,
    SX126X_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS = 0x05,
    SX126X_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_24BITS = 0x06,
    SX126X_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_32BITS = 0x07
} SX126X_radio_gfsk_preamble_detector_t;

typedef enum
{
    SX126X_RADIO_GFSK_BW_4800   = 0x1F,  //!< Bandwidth 4.8 kHz DSB
    SX126X_RADIO_GFSK_BW_5800   = 0x17,  //!< Bandwidth 5.8 kHz DSB
    SX126X_RADIO_GFSK_BW_7300   = 0x0F,  //!< Bandwidth 7.3 kHz DSB
    SX126X_RADIO_GFSK_BW_9700   = 0x1E,  //!< Bandwidth 9.7 kHz DSB
    SX126X_RADIO_GFSK_BW_11700  = 0x16,  //!< Bandwidth 11.7 kHz DSB
    SX126X_RADIO_GFSK_BW_14600  = 0x0E,  //!< Bandwidth 14.6 kHz DSB
    SX126X_RADIO_GFSK_BW_19500  = 0x1D,  //!< Bandwidth 19.5 kHz DSB
    SX126X_RADIO_GFSK_BW_23400  = 0x15,  //!< Bandwidth 23.4 kHz DSB
    SX126X_RADIO_GFSK_BW_29300  = 0x0D,  //!< Bandwidth 29.3 kHz DSB
    SX126X_RADIO_GFSK_BW_39000  = 0x1C,  //!< Bandwidth 39.0 kHz DSB
    SX126X_RADIO_GFSK_BW_46900  = 0x14,  //!< Bandwidth 46.9 kHz DSB
    SX126X_RADIO_GFSK_BW_58600  = 0x0C,  //!< Bandwidth 58.6 kHz DSB
    SX126X_RADIO_GFSK_BW_78200  = 0x1B,  //!< Bandwidth 78.2 kHz DSB
    SX126X_RADIO_GFSK_BW_93800  = 0x13,  //!< Bandwidth 93.8 kHz DSB
    SX126X_RADIO_GFSK_BW_117300 = 0x0B,  //!< Bandwidth 117.3 kHz DSB
    SX126X_RADIO_GFSK_BW_156200 = 0x1A,  //!< Bandwidth 156.2 kHz DSB
    SX126X_RADIO_GFSK_BW_187200 = 0x12,  //!< Bandwidth 187.2 kHz DSB
    SX126X_RADIO_GFSK_BW_234300 = 0x0A,  //!< Bandwidth 232.3 kHz DSB
    SX126X_RADIO_GFSK_BW_312000 = 0x19,  //!< Bandwidth 312.0 kHz DSB
    SX126X_RADIO_GFSK_BW_373600 = 0x11,  //!< Bandwidth 373.6 kHz DSB
    SX126X_RADIO_GFSK_BW_467000 = 0x09   //!< Bandwidth 467.0 kHz DSB
} SX126X_radio_gfsk_bw_t;

typedef enum
{
    SX126X_RADIO_GFSK_PULSE_SHAPE_OFF   = 0x00,  //!< No filter applied
    SX126X_RADIO_GFSK_PULSE_SHAPE_BT_03 = 0x08,  //!< Gaussian BT 0.3
    SX126X_RADIO_GFSK_PULSE_SHAPE_BT_05 = 0x09,  //!< Gaussian BT 0.5
    SX126X_RADIO_GFSK_PULSE_SHAPE_BT_07 = 0x0A,  //!< Gaussian BT 0.7
    SX126X_RADIO_GFSK_PULSE_SHAPE_BT_1  = 0x0B   //!< Gaussian BT 1.0
} SX126X_radio_gfsk_pulse_shape_t;

typedef enum
{
    SX126X_RADIO_GFSK_BITRATE_200k = 20,
    SX126X_RADIO_GFSK_BITRATE_300k = 30
} SX126X_radio_gfsk_bitrate_t;

typedef enum
{
    SX126X_RADIO_GFSK_FDEV_100k = 100
} SX126X_radio_gfsk_fdev_t;
