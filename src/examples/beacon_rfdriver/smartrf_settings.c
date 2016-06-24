/*
 * Filename: smartrf_settings.c
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
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
 *    documentation and/or other materials provided with the
 *    distribution.
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
 */

//*********************************************************************************
// These settings have been manually typed for use with ble non-connectable advertising on CC26xx with tirtos rf driver
//
//*********************************************************************************


//*********************************************************************************
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_ble_cmd.h>
#include <ti/drivers/rf/RF.h>
#include <rf_patches/rf_patch_cpe_ble.h>
#include <rf_patches/rf_patch_rfe_ble.h>
#include "smartrf_settings.h"


//Advertisment payload length in bytes
#define ADVLEN 31

// Advertisment data. Must be global for other files to access it.
char advData[ADVLEN] = {0};

static uint64_t devAddress = { 0xEEEEEEEEEEEE };

//#pragma data_alignment=4
//rfCoreHal_bleAdvOutput_t advOutput = {0};

// TI-RTOS RF Mode Object
RF_Mode RF_ble =
{
    .rfMode = RF_MODE_BLE,
    .cpePatchFxn = &rf_patch_cpe_ble,
    .mcePatchFxn = 0,
#ifdef ADV_NC_OPTIMIZED
    .rfePatchFxn = 0,
#else
    .rfePatchFxn = &rf_patch_rfe_ble,
#endif
};


// Overrides for CMD_RADIO_SETUP
uint32_t pOverrides[] =
{
#ifndef ADV_NC_OPTIMIZED
    0x00001007,  // Remove if RFE patch is not applied.
    0x00456088, // Adjust AGC reference level
#endif
    0x00354038, // Synth: Set RTRIM (POTAILRESTRIM) to 5
    0x4001402D, // Synth: Correct CKVD latency setting (address)
    0x00608402, // Synth: Correct CKVD latency setting (value)
    0x4001405D, // Synth: Set ANADIV DIV_BIAS_MODE to PG1 (address)
    0x1801F800, // Synth: Set ANADIV DIV_BIAS_MODE to PG1 (value)
    0x000784A3, // Synth: Set FREF = 3.43 MHz (24 MHz / 7)
    0xA47E0583, // Synth: Set loop bandwidth after lock to 80 kHz (K2)
    0xEAE00603, // Synth: Set loop bandwidth after lock to 80 kHz (K3, LSB)
    0x00010623, // Synth: Set loop bandwidth after lock to 80 kHz (K3, MSB)
    0x013800C3, // Use enhanced BLE shape
    /* Spur fix */
    0x02010403, // Synth: Use 24 MHz XOSC as synth clock, enable phase error discard feature
    0x40014035, // Synth: Set phase error error discard count to 1 (default 2) to get faster settling in TX (address)
    0x177F0408, // Synth: Set phase error error discard count to 1 (default 2) to get faster settling in TX (value)
    0x38000463, // Synth: Modify phase error discard threshold factor and offset
    /* Increase tone length due to spur fix */
    0x036052AC, // Add 6 us to tone in front of packet
    0x01AD02A3, // Compensate for 6 us added to tone in front of packet
    0x01680263, // Compensate for 6 us added to tone in front of packet
    0xFFFFFFFF,   // End of override list
};


// CMD_RADIO_SETUP
//#pragma data_alignment=4
rfc_CMD_RADIO_SETUP_t RF_cmdBleRadioSetup =
{
  .commandNo                = CMD_RADIO_SETUP,
  .pNextOp                  = NULL,
  .startTrigger.triggerType = TRIG_NOW,
  .condition.rule           = COND_ALWAYS,
  .pRegOverride             = pOverrides,
  .config.frontEndMode      = 0x0,  // Differential
  .config.biasMode          = 0x0,  // Internal bias
//  .txPower                  = 0x3161,  // 0 dbm
  .txPower                  = 0x9330,  // 5 dbm
  .mode                     = 0,    // BLE mode
};

/*
    { TX_POWER_0_DBM,        GEN_TX_POWER_VAL( 0x21, 1, 0x31 ) },
    { TX_POWER_5_DBM,        GEN_TX_POWER_VAL( 0x30, 0, 0x93 ) } };
	#define GEN_TX_POWER_VAL( ib, gc, tc )                                         \
  (uint16)(((ib) & 0x3F) | (((gc) & 0x03) << 6) | (((tc)&0xFF) << 8))

  0 dBm = 0x0021 | 0x0080 | 0x3100 = 0x3161
  5 dBm = 0x0030 | 0x0000 | 0x9300 = 0x9330
*/

rfc_bleAdvPar_t cmdAdvParam = {
  .advLen                     = ADVLEN,
  .pAdvData                   = (uint8_t*)advData,
  .pDeviceAddress             = (uint16_t*)&devAddress,
//  .endTrigger.triggerType     = TRIG_NEVER,
};


// CMD_ADV_NC (Beacon channel 37)
rfc_CMD_BLE_ADV_NC_t RF_cmdAdvNc0 =
{
  .commandNo                    = CMD_BLE_ADV_NC,
  .status                       = 0x0000,
  .pNextOp                      = (rfc_radioOp_t*)&RF_cmdAdvNc1,
  .startTime                    = 0x00000000,
  .startTrigger.triggerType     = TRIG_ABSTIME,
  .condition.rule               = COND_ALWAYS,
  .channel                      = 37,
  .whitening                    = 0,  // Use default
  .pParams                      = &cmdAdvParam,
  .pOutput                      = 0,
};


// CMD_ADV_NC (Beacon channel 38)
rfc_CMD_BLE_ADV_NC_t RF_cmdAdvNc1 =
{
  .commandNo                    = CMD_BLE_ADV_NC,
  .status                       = 0x0000,
  .pNextOp                      = (rfc_radioOp_t*)&RF_cmdAdvNc2,
  .startTime                    = 0x00000000,
  .startTrigger.triggerType     = TRIG_NOW,
  .condition.rule               = COND_ALWAYS,
  .channel                      = 38,
  .whitening                    = 0,  // Use default
  .pParams                      = &cmdAdvParam,
  .pOutput                      = 0,
};


// CMD_ADV_NC (Beacon channel 39)
rfc_CMD_BLE_ADV_NC_t RF_cmdAdvNc2 =
{
  .commandNo                    = CMD_BLE_ADV_NC,
  .status                       = 0x0000,
  .pNextOp                      = NULL,
  .startTime                    = 0x00000000,
  .startTrigger.triggerType     = TRIG_NOW,
  .condition.rule               = COND_NEVER,
  .channel                      = 39,
  .whitening                    = 0,  // Use default
  .pParams                      = &cmdAdvParam,
  .pOutput                      = 0,
};
