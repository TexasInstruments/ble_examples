/*
 *  ======== ti_radio_config.c ========
 *  Configured RadioConfig module definitions
 *
 *  DO NOT EDIT - This file is generated for the CC1354P10RSK
 *  by the SysConfig tool.
 *
 *  Radio Config module version : 1.17
 *  SmartRF Studio data version : 2.29.0
 */

#include "ti_radio_config.h"
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_multi_protocol.h)

// Custom overrides
#include <ti/ble5stack_flash/icall/inc/ble_overrides.h>


// *********************************************************************************
//   RF Frontend configuration
// *********************************************************************************
// RF design based on: LP_EM_CC1354P10_1

// TX Power tables
// The RF_TxPowerTable_DEFAULT_PA_ENTRY and RF_TxPowerTable_HIGH_PA_ENTRY macros are defined in RF.h.
// The following arguments are required:
// RF_TxPowerTable_DEFAULT_PA_ENTRY(bias, gain, boost, coefficient)
// RF_TxPowerTable_HIGH_PA_ENTRY(bias, ibboost, boost, coefficient, ldoTrim)
// See the Technical Reference Manual for further details about the "txPower" Command field.
// The PA settings require the CCFG_FORCE_VDDR_HH = 0 unless stated otherwise.

// 868 MHz, 13 dBm
RF_TxPowerTable_Entry txPowerTable_868_pa13[TXPOWERTABLE_868_PA13_SIZE] =
{
    {-20, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(0, 3, 0, 2, 0) }, // 0x0004C0
    {-15, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(1, 3, 0, 3, 0) }, // 0x0006C1
    {-10, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(2, 3, 0, 5, 0) }, // 0x000AC2
    {-5, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(4, 3, 0, 5, 0) }, // 0x000AC4
    {0, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(8, 3, 0, 8, 0) }, // 0x0010C8
    {1, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(9, 3, 0, 9, 0) }, // 0x0012C9
    {2, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(10, 3, 0, 9, 0) }, // 0x0012CA
    {3, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(11, 3, 0, 10, 0) }, // 0x0014CB
    {4, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(13, 3, 0, 11, 0) }, // 0x0016CD
    {5, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(14, 3, 0, 14, 0) }, // 0x001CCE
    {6, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(17, 3, 0, 16, 0) }, // 0x0020D1
    {7, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(20, 3, 0, 19, 0) }, // 0x0026D4
    {8, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(24, 3, 0, 22, 0) }, // 0x002CD8
    {9, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(28, 3, 0, 31, 0) }, // 0x003EDC
    {10, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(18, 2, 0, 31, 0) }, // 0x003E92
    {11, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(26, 2, 0, 51, 0) }, // 0x00669A
    {12, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(16, 0, 0, 82, 0) }, // 0x00A410
    // The original PA value (12.5 dBm) has been rounded to an integer value.
    {13, RF_TxPowerTable_CC13x4Sub1GHz_DEFAULT_PA_ENTRY(36, 0, 0, 89, 0) }, // 0x00B224
    RF_TxPowerTable_TERMINATION_ENTRY
};

// 868 MHz, 20 dBm
RF_TxPowerTable_Entry txPowerTable_868_pa20[TXPOWERTABLE_868_PA20_SIZE] =
{
    {14, RF_TxPowerTable_HIGH_PA_ENTRY(13, 0, 0, 28, 0) }, // 0x00380D
    {15, RF_TxPowerTable_HIGH_PA_ENTRY(18, 0, 0, 36, 0) }, // 0x004812
    {16, RF_TxPowerTable_HIGH_PA_ENTRY(24, 0, 0, 43, 0) }, // 0x005618
    {17, RF_TxPowerTable_HIGH_PA_ENTRY(28, 0, 0, 51, 2) }, // 0x02661C
    {18, RF_TxPowerTable_HIGH_PA_ENTRY(34, 0, 0, 64, 4) }, // 0x048022
    {19, RF_TxPowerTable_HIGH_PA_ENTRY(15, 3, 0, 36, 4) }, // 0x0448CF
    {20, RF_TxPowerTable_HIGH_PA_ENTRY(18, 3, 0, 71, 27) }, // 0x1B8ED2
    RF_TxPowerTable_TERMINATION_ENTRY
};


// 2400 MHz, 5 dBm
RF_TxPowerTable_Entry txPowerTable_2400_pa5[TXPOWERTABLE_2400_PA5_SIZE] =
{
    {-20, RF_TxPowerTable_DEFAULT_PA_ENTRY(8, 3, 0, 4) }, // 0x08C8
    {-18, RF_TxPowerTable_DEFAULT_PA_ENTRY(10, 3, 0, 4) }, // 0x08CA
    {-15, RF_TxPowerTable_DEFAULT_PA_ENTRY(13, 3, 0, 4) }, // 0x08CD
    {-12, RF_TxPowerTable_DEFAULT_PA_ENTRY(11, 2, 0, 7) }, // 0x0E8B
    {-10, RF_TxPowerTable_DEFAULT_PA_ENTRY(19, 3, 0, 7) }, // 0x0ED3
    {-9, RF_TxPowerTable_DEFAULT_PA_ENTRY(15, 2, 0, 12) }, // 0x188F
    {-6, RF_TxPowerTable_DEFAULT_PA_ENTRY(20, 2, 0, 12) }, // 0x1894
    {-5, RF_TxPowerTable_DEFAULT_PA_ENTRY(22, 2, 0, 12) }, // 0x1896
    {-3, RF_TxPowerTable_DEFAULT_PA_ENTRY(22, 1, 0, 19) }, // 0x2656
    {0, RF_TxPowerTable_DEFAULT_PA_ENTRY(33, 1, 0, 28) }, // 0x3861
    {1, RF_TxPowerTable_DEFAULT_PA_ENTRY(21, 0, 0, 39) }, // 0x4E15
    {2, RF_TxPowerTable_DEFAULT_PA_ENTRY(27, 0, 0, 48) }, // 0x601B
    {3, RF_TxPowerTable_DEFAULT_PA_ENTRY(36, 0, 0, 64) }, // 0x8024
    {4, RF_TxPowerTable_DEFAULT_PA_ENTRY(52, 0, 0, 60) }, // 0x7834
    {5, RF_TxPowerTable_DEFAULT_PA_ENTRY(63, 0, 0, 0) }, // 0x003F
    RF_TxPowerTable_TERMINATION_ENTRY
};



//*********************************************************************************
//  RF Setting:   BLE, 2 Mbps, LE 2M
//
//  PHY:          bt5le2m
//  Setting file: setting_bt5_le_2m.json
//*********************************************************************************

// PARAMETER SUMMARY
// NB! Setting RF parameters in this design has no effect as no RF commands are selected.

// TI-RTOS RF Mode Object
RF_Mode RF_modeBle =
{
    .rfMode = RF_MODE_AUTO,
    .cpePatchFxn = &rf_patch_cpe_multi_protocol,
    .mcePatchFxn = 0,
    .rfePatchFxn = 0
};

// Overrides for CMD_BLE5_RADIO_SETUP_PA
uint32_t pOverrides_bleCommon[] =
{
    // override_ble5_setup_override_common.json
    // Rx: Set DCDC settings drive strength and deadtime trim
    (uint32_t)0x00C188C3,
    // Synth: Increase mid code calibration time to 5 us
    (uint32_t)0x00058683,
    // Synth: Increase mid code calibration time to 5 us
    HW32_ARRAY_OVERRIDE(0x4004,0x0001),
    // Synth: Increase mid code calibration time to 5 us
    (uint32_t)0x38183C30,
    // Bluetooth 5: Default to no CTE. 
    HW_REG_OVERRIDE(0x5328,0x0000),
    // Synth: Set calibration fine point code to 60 (default: 64)
    HW_REG_OVERRIDE(0x4064,0x003C),
    // Bluetooth 5: Set DTX gain -5% for 1 Mbps
    (uint32_t)0x00E787E3,
    // Bluetooth 5: Set DTX threshold 1 Mbps
    (uint32_t)0x00950803,
    // Bluetooth 5: Set DTX gain -2.5% for 2 Mbps
    (uint32_t)0x00F487F3,
    // Bluetooth 5: Set DTX threshold 2 Mbps
    (uint32_t)0x012A0823,
    // Bluetooth 5: Set synth fine code calibration interval
    HW32_ARRAY_OVERRIDE(0x4020,0x0001),
    // Bluetooth 5: Set synth fine code calibration interval
    (uint32_t)0x41005F00,
    // Bluetooth 5: Adapt to synth fine code calibration interval
    (uint32_t)0xC0040141,
    // Bluetooth 5: Adapt to synth fine code calibration interval
    (uint32_t)0x0007DD44,
    // Bluetooth 5: Set enhanced TX shape
    (uint32_t)0x000D8C73,
    // Bluetooth 5: Set pilot tone length to 35 us
    HW_REG_OVERRIDE(0x6024,0x5B20),
    // Bluetooth 5: Compensate for 35 us pilot tone length
    (uint32_t)0x01640263,
    // ti/ble5stack_flash/icall/inc/ble_overrides.h
    BLE_STACK_OVERRIDES(),
    (uint32_t)0xFFFFFFFF
};

// Overrides for CMD_BLE5_RADIO_SETUP_PA
uint32_t pOverrides_ble1Mbps[] =
{
    // override_ble5_setup_override_1mbps.json
    // Bluetooth 5: Set pilot tone length to 35 us
    HW_REG_OVERRIDE(0x5334,0x0690),
    // Bluetooth 5: Compensate for modified pilot tone length
    (uint32_t)0x018F02A3,
    // override_ble5_symbol_error_tracking.json
    // Symbol tracking: timing correction
    HW_REG_OVERRIDE(0x50D4,0x00F9),
    // Symbol tracking: reduce sample delay
    HW_REG_OVERRIDE(0x50E0,0x0087),
    // Symbol tracking: demodulation order
    HW_REG_OVERRIDE(0x50F8,0x0014),
    (uint32_t)0xFFFFFFFF
};

// Overrides for CMD_BLE5_RADIO_SETUP_PA
uint32_t pOverrides_ble2Mbps[] =
{
    // override_ble5_setup_override_2mbps.json
    // Bluetooth 5: increase low gain AGC delay for 2 Mbps
    HW_REG_OVERRIDE(0x60A4,0x7D00),
    // Rx: increase AGC hysteresis (HIGH_GAIN=15, LOW_GAIN=11)
    HW_REG_OVERRIDE(0x6098,0x75FB),
    // Bluetooth 5: Set pilot tone length to 35 us
    HW_REG_OVERRIDE(0x5334,0x0690),
    // Bluetooth 5: Compensate for modified pilot tone length
    (uint32_t)0x012D02A3,
    // override_ble5_symbol_error_tracking.json
    // Symbol tracking: timing correction
    HW_REG_OVERRIDE(0x50D4,0x00F9),
    // Symbol tracking: reduce sample delay
    HW_REG_OVERRIDE(0x50E0,0x0087),
    // Symbol tracking: demodulation order
    HW_REG_OVERRIDE(0x50F8,0x0014),
    (uint32_t)0xFFFFFFFF
};

// Overrides for CMD_BLE5_RADIO_SETUP_PA
uint32_t pOverrides_bleCoded[] =
{
    // override_ble5_setup_override_coded.json
    // Bluetooth 5: Set pilot tone length to 35 us
    HW_REG_OVERRIDE(0x5334,0x0690),
    // Bluetooth 5: Compensate for modified pilot tone length
    (uint32_t)0x07E502A3,
    // Bluetooth 5: Set AGC mangnitude target to 0x21.
    HW_REG_OVERRIDE(0x609C,0x0021),
    (uint32_t)0xFFFFFFFF
};




