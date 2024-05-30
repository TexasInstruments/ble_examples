/*
 *  ======== ti_radio_config.h ========
 *  Configured RadioConfig module definitions
 *
 *  DO NOT EDIT - This file is generated for the CC2340R5RKP
 *  by the SysConfig tool.
 *
 *  Tool Name:           RadioConfig 8
 *  Tool Version:        7.40.00.07_eng
 *  RF Settings Version: 7.40.00.57
 */
#ifndef _TI_RADIO_CONFIG_H_
#define _TI_RADIO_CONFIG_H_

#include <ti/devices/DeviceFamily.h>
#include <ti/drivers/rcl/LRF.h>
#include DeviceFamily_constructPath(inc/hw_fcfg.h)

// RF settings version
#define RF_SETTINGS_VERSION "7.40.00.57"


// ****************************************************************
//  PHY name:            1 Mbps / 2 Mbps / Coded
//  PHY abbreviation/ID: ble
//  PHY definition file: ble_bt5_1m_2m_coded_cc23x0r5.json
// ****************************************************************

// PHY features
#define RCL_PHY_FEATURE_SUB_PHY_1_MBPS                0x0000
#define RCL_PHY_FEATURE_SUB_PHY_2_MBPS                0x0001
#define RCL_PHY_FEATURE_SUB_PHY_CODED                 0x0002
#define RCL_PHY_FEATURE_CODED_TX_RATE_S8              0x0000
#define RCL_PHY_FEATURE_CODED_TX_RATE_S2              0x0004



// LRF data structures
extern const LRF_TxShape LRF_shapeBaseGfsk067;
extern const LRF_TxShape LRF_shapeBaseGfsk05;
extern const LRF_SwConfig LRF_swConfig1Mbps;
extern const LRF_SwConfig LRF_swConfig2Mbps;
extern const LRF_TxPowerTable LRF_txPowerTable;
extern const LRF_Config LRF_config;


// Address    Module            Register             Bit(s)     Field              Value (1_mbps)    Value (2_mbps)    Value (coded)     
// -----------------------------------------------------------------------------------------------------------------------------------
// 0x1090     LRFDPBE           MDMCMDPAR0           [15:0]     VAL                0x8007            0x800F            0x0097            
// 0x1098     LRFDPBE           MDMCMDPAR2           [15:0]     VAL                0xAAAA            0xAAAA            0x3C3C            
// 0x10A0     LRFDPBE           POLY0L               [15:0]     VALLSB             0x0000            0x0000            0x0000            
// 0x10A4     LRFDPBE           POLY0H               [15:0]     VALMSB             0x2200            0x2200            0x2200            
// 0x10A8     LRFDPBE           POLY1L               [15:0]     VALLSB             0x5B00            0x5B00            0x5B00            
// 0x10AC     LRFDPBE           POLY1H               [15:0]     VALMSB             0x0006            0x0006            0x0006            
// 0x10B4     LRFDPBE           FCFG0                [5]        TXACOM             0x1               0x1               0x1               
// 0x10B4     LRFDPBE           FCFG0                [4]        TXADEAL            0x0               0x0               0x0               
// 0x10B4     LRFDPBE           FCFG0                [1]        RXACOM             0x0               0x0               0x0               
// 0x10B4     LRFDPBE           FCFG0                [0]        RXADEAL            0x1               0x1               0x1               
// 0x10B8     LRFDPBE           FCFG1                [8:0]      TXSTRT             0x0C4             0x0C4             0x0C4             
// 0x10BC     LRFDPBE           FCFG2                [7:0]      TXSIZE             0x86              0x86              0x86              
// 0x10C0     LRFDPBE           FCFG3                [8:0]      RXSTRT             0x080             0x080             0x080             
// 0x10C4     LRFDPBE           FCFG4                [7:0]      RXSIZE             0x44              0x44              0x44              
// 0x10D0     LRFDPBE           RXFRBTHRS            [9:0]      BYTES              0x004             0x004             0x004             
// 0x10D4     LRFDPBE           TXFWBTHRS            [9:0]      BYTES              0x002             0x002             0x002             
// 0x10DC     LRFDPBE           TIMCTL               [9]        SRC1               0x1               0x1               0x1               
// 0x10DC     LRFDPBE           TIMCTL               [1]        SRC0               0x1               0x1               0x1               
// 0x10E0     LRFDPBE           TIMPRE               [13:8]     PRE1               0x0B              0x0B              0x0B              
// 0x10E0     LRFDPBE           TIMPRE               [5:0]      PRE0               0x0B              0x0B              0x0B              
// 0x20B8     LRFDMDM           SYSTIMEVTMUX0        [11:6]     SEL1               0x00              0x00              -                 
// 0x20B8     LRFDMDM           SYSTIMEVTMUX0        [5:0]      SEL0               0x15              0x15              -                 
// 0x20C0     LRFDMDM           ADCDIGCONF           [1]        QBRANCHEN          0x1               0x1               0x1               
// 0x20C0     LRFDMDM           ADCDIGCONF           [0]        IBRANCHEN          0x1               0x1               0x1               
// 0x20C8     LRFDMDM           MODSYMMAP0           [7:4]      SYM1               0x1               0x1               0x1               
// 0x20C8     LRFDMDM           MODSYMMAP0           [3:0]      SYM0               0xF               0xF               0xF               
// 0x20D4     LRFDMDM           BAUD                 [15:0]     RATEWORD           0x4000            0x8000            0x4000            
// 0x20D8     LRFDMDM           BAUDPRE              [15:13]    ALIGNVALUE         0x0               0x0               0x0               
// 0x20D8     LRFDMDM           BAUDPRE              [12:8]     EXTRATEWORD        -                 -                 0x00              
// 0x20D8     LRFDMDM           BAUDPRE              [7:0]      PRESCALER          0x0C              0x0C              0x0C              
// 0x20DC     LRFDMDM           MODMAIN              [3:2]      FECSELECT          0x0               0x0               0x0               
// 0x20DC     LRFDMDM           MODMAIN              [1:0]      MODLEVELS          0x0               0x0               0x0               
// 0x20E4     LRFDMDM           DEMMISC1             [1:0]      CHFIBW             0x1               0x1               0x1               
// 0x20E8     LRFDMDM           DEMMISC2             [13:12]    MAFCGAIN           0x0               0x0               0x1               
// 0x20E8     LRFDMDM           DEMMISC2             [11]       STIMBYPASS         0x0               0x0               0x0               
// 0x20E8     LRFDMDM           DEMMISC2             [10]       STIMESTONLY        0x1               0x0               -                 
// 0x20E8     LRFDMDM           DEMMISC2             [9:7]      STIMTEAPERIOD      0x3               0x1               0x1               
// 0x20E8     LRFDMDM           DEMMISC2             [6:4]      STIMTEAGAIN        0x7               0x7               0x7               
// 0x20E8     LRFDMDM           DEMMISC2             [3]        PDIFLINPREDEN      0x1               0x1               0x1               
// 0x20E8     LRFDMDM           DEMMISC2             [2]        PDIFDESPECK        0x0               0x0               0x0               
// 0x20E8     LRFDMDM           DEMMISC2             [1]        PDIFIQCONJEN       0x0               0x0               0x0               
// 0x20E8     LRFDMDM           DEMMISC2             [0]        PDIFLIMITRANGE     0x1               0x1               0x1               
// 0x20EC     LRFDMDM           DEMMISC3             [14:13]    BDE2DVGA           0x0               -                 0x0               
// 0x20EC     LRFDMDM           DEMMISC3             [12]       BDE1FILTMODE       0x1               0x1               0x1               
// 0x20EC     LRFDMDM           DEMMISC3             [11:10]    LQIPERIOD          -                 -                 0x1               
// 0x20EC     LRFDMDM           DEMMISC3             [9:8]      BDE1DVGA           0x1               -                 0x1               
// 0x20EC     LRFDMDM           DEMMISC3             [7]        BDE1NUMSTAGES      0x1               0x1               0x1               
// 0x20EC     LRFDMDM           DEMMISC3             [6:5]      PDIFDECIM          -                 -                 0x0               
// 0x20EC     LRFDMDM           DEMMISC3             [4:0]      BDE2DECRATIO       0x02              0x01              0x02              
// 0x20F0     LRFDMDM           DEMIQMC0             [15:8]     GAINFACTOR         0x00              0x00              0x00              
// 0x20F0     LRFDMDM           DEMIQMC0             [7:0]      PHASEFACTOR        0x00              0x00              0x00              
// 0x20F4     LRFDMDM           DEMDSBU              [15:8]     DSBUAVGLENGTH      0x80              0x80              0x80              
// 0x20F4     LRFDMDM           DEMDSBU              [7:0]      DSBUDELAY          0x87              0x87              0x8D              
// 0x20F8     LRFDMDM           DEMCODC0             [11]       ESTSEL             0x1               0x1               0x0               
// 0x20F8     LRFDMDM           DEMCODC0             [10:9]     COMPSEL            0x3               0x3               0x3               
// 0x20F8     LRFDMDM           DEMCODC0             [8]        IIRUSEINITIAL      0x1               0x1               0x0               
// 0x20F8     LRFDMDM           DEMCODC0             [7:5]      IIRGAIN            0x2               0x3               0x7               
// 0x20F8     LRFDMDM           DEMCODC0             [4]        IIREN              0x1               0x1               0x1               
// 0x20FC     LRFDMDM           DEMFIDC0             [5:4]      COMPSEL            0x0               0x0               0x0               
// 0x2100     LRFDMDM           DEMFEXB0             [13]       OUT2PASSTHROUGH    -                 -                 0x0               
// 0x2100     LRFDMDM           DEMFEXB0             [12:11]    OUT2SRCSEL         -                 -                 0x1               
// 0x2100     LRFDMDM           DEMFEXB0             [10]       OUT1PASSTHROUGH    0x0               0x0               0x0               
// 0x2100     LRFDMDM           DEMFEXB0             [9:8]      OUT1SRCSEL         0x2               0x2               0x2               
// 0x2100     LRFDMDM           DEMFEXB0             [5:4]      B3SRCSEL           0x2               0x2               0x2               
// 0x2100     LRFDMDM           DEMFEXB0             [3:2]      B2SRCSEL           0x1               0x1               0x1               
// 0x2100     LRFDMDM           DEMFEXB0             [1:0]      B1SRCSEL           0x0               0x0               0x0               
// 0x2104     LRFDMDM           DEMDSXB0             [5]        OUT2PASSTHROUGH    0x0               -                 -                 
// 0x2104     LRFDMDM           DEMDSXB0             [3]        OUTSRCSEL2         0x0               0x0               0x0               
// 0x2104     LRFDMDM           DEMDSXB0             [2]        OUTSRCSEL1         0x1               0x1               0x1               
// 0x2104     LRFDMDM           DEMDSXB0             [1]        B2SRCSEL           0x0               0x0               0x1               
// 0x2104     LRFDMDM           DEMDSXB0             [0]        B1SRCSEL           0x0               0x0               0x0               
// 0x2108     LRFDMDM           DEMFIFE0             [11]       FINEFOESEL         0x0               0x0               0x0               
// 0x2108     LRFDMDM           DEMFIFE0             [10:9]     FOCFFSEL           0x0               0x0               0x0               
// 0x2108     LRFDMDM           DEMFIFE0             [4]        IIRUSEINITIAL      0x0               0x0               0x0               
// 0x2108     LRFDMDM           DEMFIFE0             [3:1]      IIRGAIN            0x2               0x2               0x6               
// 0x2108     LRFDMDM           DEMFIFE0             [0]        IIREN              0x1               0x1               0x0               
// 0x210C     LRFDMDM           DEMMAFI0             [15:8]     C1C7               0x04              0x04              0x00              
// 0x210C     LRFDMDM           DEMMAFI0             [7:0]      C0C8               0x00              0x00              0x00              
// 0x2110     LRFDMDM           DEMMAFI1             [15:8]     C3C5               0x7B              0x7B              0x3C              
// 0x2110     LRFDMDM           DEMMAFI1             [7:0]      C2C6               0x20              0x20              0x14              
// 0x2114     LRFDMDM           DEMMAFI2             [8:0]      C4                 0x0C1             0x0C1             0x078             
// 0x2118     LRFDMDM           DEMC1BE0             [15:11]    MASKB              0x00              0x00              0x00              
// 0x2118     LRFDMDM           DEMC1BE0             [10:6]     MASKA              0x00              0x00              0x00              
// 0x2118     LRFDMDM           DEMC1BE0             [5:4]      CASCCONF           0x0               0x0               0x1               
// 0x2118     LRFDMDM           DEMC1BE0             [3:0]      COPYCONF           0xF               0xF               0xF               
// 0x211C     LRFDMDM           DEMC1BE1             [15:8]     THRESHOLDB         0x7F              0x7F              0x7F              
// 0x211C     LRFDMDM           DEMC1BE1             [7:0]      THRESHOLDA         0x27              0x27              0x7F              
// 0x2120     LRFDMDM           DEMC1BE2             [9:8]      PEAKCONF           0x1               0x1               0x1               
// 0x2120     LRFDMDM           DEMC1BE2             [7:0]      THRESHOLDC         0x7F              0x7F              0x4C              
// 0x2124     LRFDMDM           SPARE0               [15:0]     VAL                0x0A18            0x0A2C            0x0A18            
// 0x2134     LRFDMDM           DEMSWQU0             [6]        AUTOMAFC           0x1               0x1               -                 
// 0x2134     LRFDMDM           DEMSWQU0             [4:0]      REFLEN             0x1F              0x1F              -                 
// 0x3080     LRFDRFE           MAGNTHRCFG           [1]        SEL                0x0               0x0               0x0               
// 0x3088     LRFDRFE           RSSIOFFSET           [7:0]      VAL                0x00              0x00              0x00              
// 0x3094     LRFDRFE           MAGNCTL1             [12]       PERMODE            -                 -                 0x1               
// 0x3094     LRFDRFE           MAGNCTL1             [11:8]     SCL                -                 -                 0x3               
// 0x3094     LRFDRFE           MAGNCTL1             [7:0]      PER                -                 -                 0x07              
// 0x3098     LRFDRFE           SPARE0               [15:0]     VAL                -                 -                 0x34F2            
// 0x30A0     LRFDRFE           SPARE2               [15:0]     VAL                0xA246            0xA357            0x0013            
// 0x30A4     LRFDRFE           SPARE3               [15:0]     VAL                0x1F40            0x1F40            0x00B0            
// 0x30A8     LRFDRFE           SPARE4               [15:0]     VAL                0x0000            0x0000            0x0000            
// 0x30AC     LRFDRFE           SPARE5               [15:0]     VAL                0x7C00            0x7C00            0x7C00            
// 0x30B0     LRFDRFE           LNA                  [7:4]      TRIM               0x0               0x0               0x0               
// 0x30B0     LRFDRFE           LNA                  [3]        BIAS               0x1               0x1               0x1               
// 0x30B0     LRFDRFE           LNA                  [2:1]      IB                 0x1               0x1               0x3               
// 0x30B4     LRFDRFE           IFAMPRFLDO           [15:9]     TRIM               0x00              0x00              0x00              
// 0x30B4     LRFDRFE           IFAMPRFLDO           [7:4]      AAFCAP             0x0               0x0               0x0               
// 0x30B4     LRFDRFE           IFAMPRFLDO           [3:1]      IFAMPIB            0x3               0x3               0x3               
// 0x30B8     LRFDRFE           PA0                  [4:0]      TRIM               0x00              0x00              0x00              
// 0x30C4     LRFDRFE           IFADC0               [14:12]    DITHERTRIM         0x0               0x0               0x0               
// 0x30C4     LRFDRFE           IFADC0               [11:10]    DITHEREN           0x0               0x0               0x0               
// 0x30C4     LRFDRFE           IFADC0               [7:4]      INT2ADJ            0x0               0x0               0x0               
// 0x30C4     LRFDRFE           IFADC0               [3:2]      AAFCAP             0x0               0x0               0x0               
// 0x30C8     LRFDRFE           IFADC1               [15]       NRZ                0x0               0x0               0x0               
// 0x30C8     LRFDRFE           IFADC1               [14:9]     TRIM               0x00              0x00              0x00              
// 0x30CC     LRFDRFE           IFADCLF              [15:12]    FF3                0x0               0x0               0x0               
// 0x30CC     LRFDRFE           IFADCLF              [11:8]     FF2                0x0               0x0               0x0               
// 0x30CC     LRFDRFE           IFADCLF              [7:4]      FF1                0x0               0x0               0x0               
// 0x30CC     LRFDRFE           IFADCLF              [3:0]      INT3               0x0               0x0               0x0               
// 0x30D0     LRFDRFE           IFADCQUANT           [2:0]      QUANTTHR           0x0               0x0               0x0               
// 0x30D4     LRFDRFE           IFADCALDO            [13:8]     TRIMOUT            0x00              0x00              0x00              
// 0x30D8     LRFDRFE           IFADCDLDO            [13:8]     TRIMOUT            0x00              0x00              0x00              
// 0x30E4     LRFDRFE           ATSTREFH             [14:10]    IREFTRIM           0x00              0x00              0x00              
// 0x30E4     LRFDRFE           ATSTREFH             [9]        BIAS               0x1               0x1               0x1               
// 0x30E8     LRFDRFE           DCO                  [6:3]      TAILRESTRIM        0x0               0x0               0x0               
// 0x30E8     LRFDRFE           DCO                  [0]        CRSCAPCM           0x0               0x0               0x0               
// 0x30EC     LRFDRFE           DIV                  [14:12]    NMIREFTRIM         0x0               0x0               0x0               
// 0x30EC     LRFDRFE           DIV                  [11:9]     PMIREFTRIM         0x0               0x0               0x0               
// 0x30EC     LRFDRFE           DIV                  [3]        OUT                0x1               0x1               0x1               
// 0x30EC     LRFDRFE           DIV                  [2:0]      RATIO              0x0               0x0               0x0               
// 0x30F0     LRFDRFE           DIVLDO               [14:8]     VOUTTRIM           0x00              0x00              0x00              
// 0x30F4     LRFDRFE           TDCLDO               [14:8]     VOUTTRIM           0x00              0x00              0x00              
// 0x30F8     LRFDRFE           DCOLDO0              [13:8]     SECONDTRIM         0x00              0x00              0x00              
// 0x30F8     LRFDRFE           DCOLDO0              [7:4]      FIRSTTRIM          0x0               0x0               0x0               
// 0x30FC     LRFDRFE           DCOLDO1              [10]       REFSRC             0x0               0x0               0x0               
// 0x30FC     LRFDRFE           DCOLDO1              [9:8]      DIVATST            0x0               0x0               0x0               
// 0x3100     LRFDRFE           PRE0                 [13:8]     PLLDIV1            0x07              0x07              0x07              
// 0x3100     LRFDRFE           PRE0                 [5:0]      PLLDIV0            0x06              0x06              0x06              
// 0x3104     LRFDRFE           PRE1                 [5:0]      HSDDC              0x00              0x00              0x00              
// 0x3108     LRFDRFE           PRE2                 [15:12]    MIDCALDIVLSB       0x0               0x0               0x0               
// 0x3108     LRFDRFE           PRE2                 [11:6]     CRSCALDIV          0x18              0x18              0x18              
// 0x3108     LRFDRFE           PRE2                 [5:0]      FSMDIV             0x05              0x05              0x05              
// 0x310C     LRFDRFE           PRE3                 [15:5]     FINECALDIV         0x030             0x030             0x030             
// 0x310C     LRFDRFE           PRE3                 [4:0]      MIDCALDIVMSB       0x03              0x03              0x03              
// 0x3110     LRFDRFE           CAL0                 [14:8]     FCSTART            0x40              0x40              0x40              
// 0x3110     LRFDRFE           CAL0                 [3:2]      TDCAVG             0x2               0x2               0x2               
// 0x3114     LRFDRFE           CAL1                 [14:8]     FCTOP              0x7F              0x7F              0x7F              
// 0x3114     LRFDRFE           CAL1                 [6:0]      FCBOT              0x00              0x00              0x00              
// 0x3120     LRFDRFE           MISC0                [13]       PHCPT              0x0               0x0               0x0               
// 0x3120     LRFDRFE           MISC0                [12]       TDCCALCORR         0x1               0x1               0x1               
// 0x3120     LRFDRFE           MISC0                [11]       TDCMSBCORR         0x1               0x1               0x1               
// 0x3120     LRFDRFE           MISC0                [5:4]      DLYCANCRS          0x2               0x2               0x2               
// 0x3130     LRFDRFE           PHEDISC              [13:10]    CNT                0x3               0x3               0x3               
// 0x3130     LRFDRFE           PHEDISC              [9:0]      THR                0x007             0x007             0x007             
// 0x3138     LRFDRFE           PLLMON0              [15:14]    PHELOLCNT          0x3               0x3               0x3               
// 0x3138     LRFDRFE           PLLMON0              [13:8]     PHELOLTHR          0x1F              0x1F              0x1F              
// 0x3138     LRFDRFE           PLLMON0              [6:0]      FCTHR              0x7F              0x7F              0x7F              
// 0x313C     LRFDRFE           PLLMON1              [12:8]     PHELOCKCNT         0x04              0x04              0x04              
// 0x313C     LRFDRFE           PLLMON1              [7:0]      PHELOCKTHR         0x7F              0x7F              0x7F              
// 0x3140     LRFDRFE           MOD0                 [12:11]    SCHEME             0x3               0x3               0x3               
// 0x3140     LRFDRFE           MOD0                 [10:8]     SYMSHP             0x0               0x0               0x0               
// 0x3140     LRFDRFE           MOD0                 [3:2]      INTPFACT           0x1               0x1               0x1               
// 0x2020     PBE_BLE5_RAM      PHY                  [1:0]      SEL                0x0               0x1               0x3               
// 0x2022     PBE_BLE5_RAM      PRETXIFS500K         [15:0]     VAL                -                 -                 0x0120            
// 0x2024     PBE_BLE5_RAM      PRETXIFS             [15:0]     VAL                0x018E            0x01B2            0x0102            
// 0x2026     PBE_BLE5_RAM      PRERXIFS             [15:0]     VAL                0x01AE            0x01AE            0x012C            
// 0x2028     PBE_BLE5_RAM      RXTIMEOUT            [15:0]     VAL                0x0226            0x01C2            0x07EA            
// 0x202A     PBE_BLE5_RAM      SYNTHCALTIMEOUT      [15:0]     VAL                0x00B4            0x00B4            0x00B4            
// 0x202C     PBE_BLE5_RAM      RECALTIMEOUT         [15:0]     VAL                0x00B4            0x00B4            0x00B4            
// 0x2032     PBE_BLE5_RAM      FIFOCFG              [8]        APPENDTIMESTAMP    0x1               0x1               0x1               
// 0x2032     PBE_BLE5_RAM      FIFOCFG              [7]        APPENDRSSI         0x1               0x1               0x1               
// 0x2032     PBE_BLE5_RAM      FIFOCFG              [6]        APPENDFREQEST      0x0               0x0               0x0               
// 0x2032     PBE_BLE5_RAM      FIFOCFG              [5]        APPENDLQI          0x0               0x0               0x0               
// 0x2032     PBE_BLE5_RAM      FIFOCFG              [4]        APPENDSTATUS       0x1               0x1               0x1               
// 0x2032     PBE_BLE5_RAM      FIFOCFG              [3]        APPENDCRC          0x0               0x0               0x0               
// 0x2032     PBE_BLE5_RAM      FIFOCFG              [2]        AUTOFLUSHEMPTY     0x1               0x1               0x1               
// 0x2032     PBE_BLE5_RAM      FIFOCFG              [1]        AUTOFLUSHIGN       0x1               0x1               0x1               
// 0x2032     PBE_BLE5_RAM      FIFOCFG              [0]        AUTOFLUSHCRC       0x1               0x1               0x1               
// 0x2036     PBE_BLE5_RAM      NAKHUB               [2:0]      MAXNAK             0x2               0x2               0x2               
// 0x6804     RFE_COMMON_RAM    SYNTHCTL             [7]        CHRGFILT           0x0               0x0               0x0               
// 0x6804     RFE_COMMON_RAM    SYNTHCTL             [6]        VREFBP             0x0               0x0               0x0               
// 0x6804     RFE_COMMON_RAM    SYNTHCTL             [5]        TXWAITMOD          0x0               0x0               0x0               
// 0x6804     RFE_COMMON_RAM    SYNTHCTL             [4]        PHEDISC            0x1               0x1               0x1               
// 0x6804     RFE_COMMON_RAM    SYNTHCTL             [3]        RTRIMTST           0x0               0x0               0x0               
// 0x6804     RFE_COMMON_RAM    SYNTHCTL             [2]        IIR                0x0               0x0               0x0               
// 0x6804     RFE_COMMON_RAM    SYNTHCTL             [1]        REFDTHR            0x1               0x1               0x1               
// 0x6804     RFE_COMMON_RAM    SYNTHCTL             [0]        FCDEM              0x0               0x0               0x0               
// 0x6806     RFE_COMMON_RAM    TDCCAL0              [10:8]     STOP               0x3               0x3               0x3               
// 0x6808     RFE_COMMON_RAM    TDCCAL1              [7:0]      SUB                0x00              0x00              0x00              
// 0x680A     RFE_COMMON_RAM    TDCCAL2              [5:0]      AVG                0x10              0x10              0x10              
// 0x680C     RFE_COMMON_RAM    TDCPLL               [10:8]     STOP               0x4               0x4               0x4               
// 0x680E     RFE_COMMON_RAM    K1LSB                [15:0]     VAL                0x569B            0x569B            0x569B            
// 0x6810     RFE_COMMON_RAM    K1MSB                [15:0]     VAL                0x010A            0x010A            0x010A            
// 0x6812     RFE_COMMON_RAM    K2BL                 [15]       HPM                0x0               0x0               0x0               
// 0x6812     RFE_COMMON_RAM    K2BL                 [14:0]     VAL                0x012D            0x012D            0x012D            
// 0x6814     RFE_COMMON_RAM    K2AL                 [15]       HPM                0x0               0x0               0x0               
// 0x6814     RFE_COMMON_RAM    K2AL                 [14:0]     VAL                0x0034            0x0034            0x0034            
// 0x6816     RFE_COMMON_RAM    K3BL                 [15:0]     VAL                0x132C            0x132C            0x132C            
// 0x6818     RFE_COMMON_RAM    K3AL                 [15:0]     VAL                0x07AB            0x07AB            0x07AB            
// 0x681A     RFE_COMMON_RAM    K5                   [15:0]     VAL                0x916F            0x916F            0x916F            
// 0x6820     RFE_COMMON_RAM    RTRIMOFF             [3:0]      VAL                0x0               0x0               0x0               
// 0x6822     RFE_COMMON_RAM    RTRIMMIN             [3:0]      VAL                0x0               0x0               0x0               
// 0x6824     RFE_COMMON_RAM    DIVI                 [14:12]    NMIREFTRIM         0x0               0x0               0x0               
// 0x6824     RFE_COMMON_RAM    DIVI                 [11:9]     PMIREFTRIM         0x0               0x0               0x0               
// 0x6824     RFE_COMMON_RAM    DIVI                 [3]        OUT                0x1               0x1               0x1               
// 0x6824     RFE_COMMON_RAM    DIVI                 [2:0]      RATIO              0x0               0x0               0x0               
// 0x6826     RFE_COMMON_RAM    DIVF                 [14:12]    NMIREFTRIM         0x4               0x4               0x4               
// 0x6826     RFE_COMMON_RAM    DIVF                 [11:9]     PMIREFTRIM         0x4               0x4               0x4               
// 0x6826     RFE_COMMON_RAM    DIVF                 [3]        OUT                0x1               0x1               0x1               
// 0x6826     RFE_COMMON_RAM    DIVF                 [2:0]      RATIO              0x0               0x0               0x0               
// 0x6828     RFE_COMMON_RAM    DIVLDOI              [14:8]     VOUTTRIM           0x00              0x00              0x00              
// 0x6828     RFE_COMMON_RAM    DIVLDOI              [0]        CTL                0x0               0x0               0x0               
// 0x682A     RFE_COMMON_RAM    DIVLDOF              [14:8]     VOUTTRIM           0x00              0x00              0x00              
// 0x682A     RFE_COMMON_RAM    DIVLDOF              [0]        CTL                0x0               0x0               0x0               
// 0x682C     RFE_COMMON_RAM    DIVLDOIOFF           [6:0]      VAL                0x14              0x14              0x14              
// 0x682E     RFE_COMMON_RAM    LDOSETTLE            [9:0]      VAL                0x047             0x047             0x047             
// 0x6830     RFE_COMMON_RAM    CHRGSETTLE           [9:0]      VAL                0x02E             0x02E             0x02E             
// 0x6832     RFE_COMMON_RAM    DCOSETTLE            [9:0]      VAL                0x005             0x005             0x005             
// 0x6834     RFE_COMMON_RAM    IFAMPRFLDOTX         [15:9]     TRIM               0x7F              0x7F              0x7F              
// 0x6836     RFE_COMMON_RAM    IFAMPRFLDODEFAULT    [15:9]     TRIM               0x00              0x00              0x00              
// 0x683E     RFE_COMMON_RAM    PHYRSSIOFFSET        [7:0]      VAL                0x50              0x4C              0x4C              
// 0x6840     RFE_COMMON_RAM    SPARE0SHADOW         [15:0]     VAL                0x75F8            0x55FA            -                 
// 0x6842     RFE_COMMON_RAM    SPARE1SHADOW         [15:0]     VAL                0x0045            0x0045            0x001C            
// 0x6844     RFE_COMMON_RAM    AGCINFO              [0]        MODE               0x0               0x0               0x1               
// Address    Module            Register             Bit(s)     Field              Value (s8)        Value (s2)        
// -----------------------------------------------------------------------------------------------------------------
// 0x2128     LRFDMDM           SPARE1               [15:0]     VAL                0x0000            0x0001            


#endif // _TI_RADIO_CONFIG_H_
