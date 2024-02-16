// SETTINGS FOR PHY BASED ON RADIO CONTROL LAYER (SOURCE FILE)
//
// Usage                  Generic
//
//
// CODE EXPORT INFORMATION
// This file is generated
//
// Tool name              SmartRF Studio 8
// Tool version           1.0.2.39
// Created                2024-01-26 15:07:45.964
// Computer               LT5CG2132LP5
// User
//
//
// WORKSPACE INFORMATION
//
// Workspace file         -
// Device                 CC2340R5
//     Package            QFN40 5x5 RKP
//     Revision(s)        B (2.0)
// SDK                    SimpleLink Low Power F3 SDK 7.40.00
// Board                  LP-EM-CC2340R5
// PHY                    2.4 GHz - Bluetooth 5, LE - 1 Mbps / 2 Mbps / Coded
// PHY abbreviation       ble
//
//
// PHY PROPERTIES
//
// Run-time properties:
//     Sub-PHY            1 Mbps
//     Channel            17 (2440 MHz)
//     TX output power    5.0 dBm
//
//
// VALIDATION WARNINGS
//
// No warnings

#include "rcl_settings_ble_generic.h"
#include DeviceFamily_constructPath(rf_patches/lrf_pbe_binary_generic.h)
#include DeviceFamily_constructPath(rf_patches/lrf_mce_binary_ble5.h)
#include DeviceFamily_constructPath(rf_patches/lrf_rfe_binary_ble5.h)


// Configuration: Common
static const uint32_t LRF_commonRegConfig[] =
{
    0x0000003F,                          // Segment length = 63
    0x0004A001,                          //     Data structure 32-bit region (start byte position = 4, count = 2)
    (uint32_t) &LRF_txPowerTableBle,     //         LRF_swParam : txPowerTable
    (uint32_t) &(fcfg->appTrims),        //         LRF_swParam : trimDef
    0x00003009,                          //     HW sparse region (address/value pairs, count = 10)
    0x10940000,                          //         LRFDPBE.MDMCMDPAR1
    0x20C00003,                          //         LRFDMDM.ADCDIGCONF
    0x20C8001F,                          //         LRFDMDM.MODSYMMAP0
    0x20E40001,                          //         LRFDMDM.DEMMISC1
    0x20F00000,                          //         LRFDMDM.DEMIQMC0
    0x20FC0000,                          //         LRFDMDM.DEMFIDC0
    0x30800000,                          //         LRFDRFE.MAGNTHRCFG
    0x30880000,                          //         LRFDRFE.RSSIOFFSET
    0x31201820,                          //         LRFDRFE.MISC0
    0x31300C07,                          //         LRFDRFE.PHEDISC
    0x14502004,                          //     HW 32-bit region (start address = 0x1450, count = 5)
    0x22000000,                          //         LRFDPBE.POLY0H                      LRFDPBE.POLY0L
    0x00065B00,                          //         LRFDPBE.POLY1H                      LRFDPBE.POLY1L
    0x00210000,                          //         LRFDPBE.FCFG0                       LRFDPBE.PHACFG
    0x008600C4,                          //         LRFDPBE.FCFG2                       LRFDPBE.FCFG1
    0x00440080,                          //         LRFDPBE.FCFG4                       LRFDPBE.FCFG3
    0x14682000,                          //     HW 32-bit region (start address = 0x1468, count = 1)
    0x00020004,                          //         LRFDPBE.TXFWBTHRS                   LRFDPBE.RXFRBTHRS
    0x10DC1001,                          //     HW 16-bit region (start address = 0x10DC, count = 2)
    0x0B0B0202,                          //         LRFDPBE.TIMPRE                      LRFDPBE.TIMCTL
    0x246C2000,                          //     HW 32-bit region (start address = 0x246C, count = 1)
    0x0000000C,                          //         LRFDMDM.MODMAIN                     LRFDMDM.BAUDPRE
    0x34542000,                          //     HW 32-bit region (start address = 0x3454, count = 1)
    0x7C000000,                          //         LRFDRFE.SPARE5                      LRFDRFE.SPARE4
    0x30B41001,                          //     HW 16-bit region (start address = 0x30B4, count = 2)
    0x00000006,                          //         LRFDRFE.PA0                         LRFDRFE.IFAMPRFLDO
    0x30C40005,                          //     HW zero region (start address = 0x30C4, count = 6)
    0x30E4100C,                          //     HW 16-bit region (start address = 0x30E4, count = 13)
    0x00000200,                          //         LRFDRFE.DCO                         LRFDRFE.ATSTREFH
    0x00000008,                          //         LRFDRFE.DIVLDO                      LRFDRFE.DIV
    0x00000000,                          //         LRFDRFE.DCOLDO0                     LRFDRFE.TDCLDO
    0x07060000,                          //         LRFDRFE.PRE0                        LRFDRFE.DCOLDO1
    0x06050000,                          //         LRFDRFE.PRE2                        LRFDRFE.PRE1
    0x40080603,                          //         LRFDRFE.CAL0                        LRFDRFE.PRE3
    0x00007F00,                          //         -                                   LRFDRFE.CAL1
    0x31381002,                          //     HW 16-bit region (start address = 0x3138, count = 3)
    0x047FDF7F,                          //         LRFDRFE.PLLMON1                     LRFDRFE.PLLMON0
    0x00001804,                          //         -                                   LRFDRFE.MOD0
    0x2022500A,                          //     RAM 16-bit region (start address = 0x2022, count = 11)
    0x009000B4,                          //         PBE_GENERIC_RAM.PKTCFG              PBE_GENERIC_RAM.SYNTHCALTIMEOUT
    0x03B00018,                          //         PBE_GENERIC_RAM.FIFOCFG             PBE_GENERIC_RAM.NUMCRCBITS
    0x00510000,                          //         PBE_GENERIC_RAM.WHITEINIT           PBE_GENERIC_RAM.EXTRABYTES
    0x55000000,                          //         PBE_GENERIC_RAM.CRCINITL            < GAP >
    0x01085555,                          //         PBE_GENERIC_RAM.LENCFG              PBE_GENERIC_RAM.CRCINITH
    0x00000000,                          //         -                                   PBE_GENERIC_RAM.LENOFFSET
    0x68046005,                          //     RAM 32-bit region (start address = 0x6804, count = 6)
    0x03000012,                          //         RFE_COMMON_RAM.TDCCAL0              RFE_COMMON_RAM.SYNTHCTL
    0x00100000,                          //         RFE_COMMON_RAM.TDCCAL2              RFE_COMMON_RAM.TDCCAL1
    0x569B0400,                          //         RFE_COMMON_RAM.K1LSB                RFE_COMMON_RAM.TDCPLL
    0x012D010A,                          //         RFE_COMMON_RAM.K2BL                 RFE_COMMON_RAM.K1MSB
    0x132C0034,                          //         RFE_COMMON_RAM.K3BL                 RFE_COMMON_RAM.K2AL
    0x916F07AB,                          //         RFE_COMMON_RAM.K5                   RFE_COMMON_RAM.K3AL
    0x68206005,                          //     RAM 32-bit region (start address = 0x6820, count = 6)
    0x00000000,                          //         RFE_COMMON_RAM.RTRIMMIN             RFE_COMMON_RAM.RTRIMOFF
    0x48080008,                          //         RFE_COMMON_RAM.DIVF                 RFE_COMMON_RAM.DIVI
    0x00000000,                          //         RFE_COMMON_RAM.DIVLDOF              RFE_COMMON_RAM.DIVLDOI
    0x00470014,                          //         RFE_COMMON_RAM.LDOSETTLE            RFE_COMMON_RAM.DIVLDOIOFF
    0x0005002E,                          //         RFE_COMMON_RAM.DCOSETTLE            RFE_COMMON_RAM.CHRGSETTLE
    0x0000FE00                           //         RFE_COMMON_RAM.IFAMPRFLDODEFAULT    RFE_COMMON_RAM.IFAMPRFLDOTX
};

// Configuration: Sub-PHY = 1 Mbps, 2 Mbps
static const uint32_t LRF_subPhy1Mbps2MbpsRegConfig[] =
{
    0x00024010,                          // Segment length = 16
    0x00003004,                          //     HW sparse region (address/value pairs, count = 5)
    0x1098AAAA,                          //         LRFDPBE.MDMCMDPAR2
    0x20B80015,                          //         LRFDMDM.SYSTIMEVTMUX0
    0x20F48087,                          //         LRFDMDM.DEMDSBU
    0x2134005F,                          //         LRFDMDM.DEMSWQU0
    0x30A41F40,                          //         LRFDRFE.SPARE3
    0x21001008,                          //     HW 16-bit region (start address = 0x2100, count = 9)
    0x00040224,                          //         LRFDMDM.DEMDSXB0                    LRFDMDM.DEMFEXB0
    0x04000005,                          //         LRFDMDM.DEMMAFI0                    LRFDMDM.DEMFIFE0
    0x00C17B20,                          //         LRFDMDM.DEMMAFI2                    LRFDMDM.DEMMAFI1
    0x7F27000F,                          //         LRFDMDM.DEMC1BE1                    LRFDMDM.DEMC1BE0
    0x0000017F,                          //         -                                   LRFDMDM.DEMC1BE2
    0x30B0B000,                          //     HW masked region (mask/value pairs, count = 1)
    0xFF0F000A,                          //         LRFDRFE.LNA
    0x68425001,                          //     RAM 16-bit region (start address = 0x6842, count = 2)
    0x00000045                           //         RFE_COMMON_RAM.AGCINFO              RFE_COMMON_RAM.SPARE1SHADOW
};

// Configuration: Sub-PHY = 1 Mbps, Coded
static const uint32_t LRF_subPhy1MbpsCodedRegConfig[] =
{
    0x00014005,                          // Segment length = 5
    0x0000A000,                          //     Data structure 32-bit region (start byte position = 0, count = 1)
    (uint32_t) &LRF_swConfig1MbpsBle,    //         LRF_swParam : swConfig
    0x00003001,                          //     HW sparse region (address/value pairs, count = 2)
    0x20D44000,                          //         LRFDMDM.BAUD
    0x21240A18                           //         LRFDMDM.SPARE0
};

// Configuration: Sub-PHY = 2 Mbps, Coded
static const uint32_t LRF_subPhy2MbpsCodedRegConfig[] =
{
    0x00030002,                          // Segment length = 2
    0x00007000,                          //     RAM sparse region (address/value pairs, count = 1)
    0x683E004C                           //         RFE_COMMON_RAM.PHYRSSIOFFSET
};

// Configuration: Sub-PHY = 1 Mbps
static const uint32_t LRF_subPhy1MbpsRegConfig[] =
{
    0x0003400A,                          // Segment length = 10
    0x00003002,                          //     HW sparse region (address/value pairs, count = 3)
    0x10908007,                          //         LRFDPBE.MDMCMDPAR0
    0x20F80F50,                          //         LRFDMDM.DEMCODC0
    0x30A0A246,                          //         LRFDRFE.SPARE2
    0x24742000,                          //     HW 32-bit region (start address = 0x2474, count = 1)
    0x118205F9,                          //         LRFDMDM.DEMMISC3                    LRFDMDM.DEMMISC2
    0x00007000,                          //     RAM sparse region (address/value pairs, count = 1)
    0x20200000,                          //         PBE_GENERIC_RAM.PHY
    0x683E5001,                          //     RAM 16-bit region (start address = 0x683E, count = 2)
    0x75F80050                           //         RFE_COMMON_RAM.SPARE0SHADOW         RFE_COMMON_RAM.PHYRSSIOFFSET
};

// Configuration: Sub-PHY = 2 Mbps
static const uint32_t LRF_subPhy2MbpsRegConfig[] =
{
    0x0001000D,                          // Segment length = 13
    0x0000A000,                          //     Data structure 32-bit region (start byte position = 0, count = 1)
    (uint32_t) &LRF_swConfig2MbpsBle,    //         LRF_swParam : swConfig
    0x00003004,                          //     HW sparse region (address/value pairs, count = 5)
    0x1090800F,                          //         LRFDPBE.MDMCMDPAR0
    0x20D48000,                          //         LRFDMDM.BAUD
    0x20F80F70,                          //         LRFDMDM.DEMCODC0
    0x21240A2C,                          //         LRFDMDM.SPARE0
    0x30A0A357,                          //         LRFDRFE.SPARE2
    0x24742000,                          //     HW 32-bit region (start address = 0x2474, count = 1)
    0x108100F9,                          //         LRFDMDM.DEMMISC3                    LRFDMDM.DEMMISC2
    0x00007001,                          //     RAM sparse region (address/value pairs, count = 2)
    0x20200009,                          //         PBE_GENERIC_RAM.PHY
    0x684055FA                           //         RFE_COMMON_RAM.SPARE0SHADOW
};

// Configuration: Sub-PHY = Coded
static const uint32_t LRF_subPhyCodedRegConfig[] =
{
    0x00020017,                          // Segment length = 23
    0x00003001,                          //     HW sparse region (address/value pairs, count = 2)
    0x10900097,                          //         LRFDPBE.MDMCMDPAR0
    0x10983C3C,                          //         LRFDPBE.MDMCMDPAR2
    0x24742000,                          //     HW 32-bit region (start address = 0x2474, count = 1)
    0x158210F9,                          //         LRFDMDM.DEMMISC3                    LRFDMDM.DEMMISC2
    0x20F41001,                          //     HW 16-bit region (start address = 0x20F4, count = 2)
    0x06F0808D,                          //         LRFDMDM.DEMCODC0                    LRFDMDM.DEMDSBU
    0x21001008,                          //     HW 16-bit region (start address = 0x2100, count = 9)
    0x00060A24,                          //         LRFDMDM.DEMDSXB0                    LRFDMDM.DEMFEXB0
    0x0000000C,                          //         LRFDMDM.DEMMAFI0                    LRFDMDM.DEMFIFE0
    0x00783C14,                          //         LRFDMDM.DEMMAFI2                    LRFDMDM.DEMMAFI1
    0x7F7F001F,                          //         LRFDMDM.DEMC1BE1                    LRFDMDM.DEMC1BE0
    0x0000014C,                          //         -                                   LRFDMDM.DEMC1BE2
    0x30941001,                          //     HW 16-bit region (start address = 0x3094, count = 2)
    0x34F21307,                          //         LRFDRFE.SPARE0                      LRFDRFE.MAGNCTL1
    0x34502000,                          //     HW 32-bit region (start address = 0x3450, count = 1)
    0x00B00013,                          //         LRFDRFE.SPARE3                      LRFDRFE.SPARE2
    0x30B0B000,                          //     HW masked region (mask/value pairs, count = 1)
    0xFF0F000E,                          //         LRFDRFE.LNA
    0x00007000,                          //     RAM sparse region (address/value pairs, count = 1)
    0x2020001B,                          //         PBE_GENERIC_RAM.PHY
    0x68425001,                          //     RAM 16-bit region (start address = 0x6842, count = 2)
    0x0001001C                           //         RFE_COMMON_RAM.AGCINFO              RFE_COMMON_RAM.SPARE1SHADOW
};

// Configuration: Coded TX rate = S8 (8 symbols per bit = 125 kbps)
static const uint32_t LRF_codedTxRateS8RegConfig[] =
{
    0x00044002,                          // Segment length = 2
    0x00003000,                          //     HW sparse region (address/value pairs, count = 1)
    0x21280000                           //         LRFDMDM.SPARE1
};

// Configuration: Coded TX rate = S2 (2 symbols per bit = 500 kbps)
static const uint32_t LRF_codedTxRateS2RegConfig[] =
{
    0x00040002,                          // Segment length = 2
    0x00003000,                          //     HW sparse region (address/value pairs, count = 1)
    0x21280001                           //         LRFDMDM.SPARE1
};


// LRF register configuration list
static const LRF_RegConfigList LRF_regConfigList = {
    .numEntries = 9,
    .entries = {
        (LRF_ConfigWord*) LRF_commonRegConfig,
        (LRF_ConfigWord*) LRF_subPhy1Mbps2MbpsRegConfig,
        (LRF_ConfigWord*) LRF_subPhy1MbpsCodedRegConfig,
        (LRF_ConfigWord*) LRF_subPhy2MbpsCodedRegConfig,
        (LRF_ConfigWord*) LRF_subPhy1MbpsRegConfig,
        (LRF_ConfigWord*) LRF_subPhy2MbpsRegConfig,
        (LRF_ConfigWord*) LRF_subPhyCodedRegConfig,
        (LRF_ConfigWord*) LRF_codedTxRateS8RegConfig,
        (LRF_ConfigWord*) LRF_codedTxRateS2RegConfig
    }
};

// LRF_TxShape data structure
const LRF_TxShape LRF_shapeBaseGfsk067Ble = {
    .scale                 = 0x0FDE2,
    .numCoeff              = 0x0011,
    .coeff                 = { 0x01, 0x02, 0x05, 0x0A, 0x14, 0x22, 0x37, 0x52, 0x71, 0x91, 0xB0, 0xCB, 0xE0, 0xEE, 0xF8, 0xFD, 0xFF }
};

// LRF_TxShape data structure
const LRF_TxShape LRF_shapeBaseGfsk05Ble = {
    .scale                 = 0x0F183,
    .numCoeff              = 0x0014,
    .coeff                 = { 0x01, 0x02, 0x03, 0x06, 0x0A, 0x11, 0x1A, 0x27, 0x37, 0x4B, 0x62, 0x7B, 0x94, 0xAD, 0xC4, 0xD8, 0xE7, 0xF3, 0xFB, 0xFF }
};

// LRF_SwConfig data structure
const LRF_SwConfig LRF_swConfig1MbpsBle = {
    .rxIntFrequency        = 1000000,
    .rxFrequencyOffset     = 0,
    .txFrequencyOffset     = 1000000,
    .modFrequencyDeviation = 0x0003D090,
    .txShape               = &LRF_shapeBaseGfsk067Ble,
    .bwIndex               = 0x00,
    .bwIndexDither         = 0x01
};

// LRF_SwConfig data structure
const LRF_SwConfig LRF_swConfig2MbpsBle = {
    .rxIntFrequency        = 850000,
    .rxFrequencyOffset     = 0,
    .txFrequencyOffset     = 850000,
    .modFrequencyDeviation = 0x0007A120,
    .txShape               = &LRF_shapeBaseGfsk05Ble,
    .bwIndex               = 0x01,
    .bwIndexDither         = 0x01
};

// LRF_TxPowerTable data structure
const LRF_TxPowerTable LRF_txPowerTableBle = {
    .numEntries            = 0x0000000E,
    .powerTable            = {
        { .power = { .fraction = 0, .dBm = -20 }, .tempCoeff = 0, .value = { .reserved = 0, .ib = 18, .gain = 0, .mode = 0, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = -16 }, .tempCoeff = 0, .value = { .reserved = 0, .ib = 20, .gain = 1, .mode = 0, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = -12 }, .tempCoeff = 5, .value = { .reserved = 0, .ib = 17, .gain = 3, .mode = 0, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = -8 }, .tempCoeff = 12, .value = { .reserved = 0, .ib = 17, .gain = 4, .mode = 0, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = -4 }, .tempCoeff = 25, .value = { .reserved = 0, .ib = 17, .gain = 5, .mode = 0, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = 0 }, .tempCoeff = 40, .value = { .reserved = 0, .ib = 19, .gain = 6, .mode = 0, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = 1 }, .tempCoeff = 65, .value = { .reserved = 0, .ib = 30, .gain = 6, .mode = 0, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = 2 }, .tempCoeff = 41, .value = { .reserved = 0, .ib = 39, .gain = 4, .mode = 1, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = 3 }, .tempCoeff = 43, .value = { .reserved = 0, .ib = 31, .gain = 5, .mode = 1, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = 4 }, .tempCoeff = 50, .value = { .reserved = 0, .ib = 37, .gain = 5, .mode = 1, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = 5 }, .tempCoeff = 55, .value = { .reserved = 0, .ib = 27, .gain = 6, .mode = 1, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = 6 }, .tempCoeff = 75, .value = { .reserved = 0, .ib = 38, .gain = 6, .mode = 1, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = 7 }, .tempCoeff = 80, .value = { .reserved = 0, .ib = 25, .gain = 7, .mode = 1, .noIfampRfLdoBypass = 0 } },
        { .power = { .fraction = 0, .dBm = 8 }, .tempCoeff = 180, .value = { .reserved = 0, .ib = 63, .gain = 7, .mode = 1, .noIfampRfLdoBypass = 0 } }
    }
};

// LRF_Config data structure
const LRF_Config LRF_configBle = {
    .pbeImage              = (const LRF_TOPsmImage*) LRF_PBE_binary_generic,
    .mceImage              = (const LRF_TOPsmImage*) LRF_MCE_binary_ble5,
    .rfeImage              = (const LRF_TOPsmImage*) LRF_RFE_binary_ble5,
    .regConfigList         = &LRF_regConfigList
};
