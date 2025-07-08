/******************************************************************************

@file  power_shutdown.c

@brief This file contains the application main functionality

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2024, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************

 Copyright (c) 2022-2025, Texas Instruments Incorporated
 All rights reserved.

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 ******************************************************************************
 *****************************************************************************/

#include "PowerShutdown/power_shutdown.h"
#include "KeyScan/key_scan.h"
#include "IR/IR.h"

#define RC_IDLE_CONN_TIMEOUT         900000  // 15 minutes
#define RC_IDLE_ADV_TIMEOUT          900000  // 15 minutes
#define CLOCK_UNITS_MS               1000  // convert to ms

#ifdef USE_ZIPIR

uint8_t pressed;
uint8_t ir_function;
IrInput_t ir_input;

extern const U8 gpio;
extern const U8 pwmChannel;

static const uint16_t numEncoding = 136;
static const uint16_t encoded_BlePairing[numEncoding]=
{
    0x80AA,    0x00AA,    0x8015,    0x003F,
    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,    0x8015,    0x0014,
    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x003F,
    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,    0x8015,    0x0014,
    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x003F,
    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x003F,
    0x8015,    0x0014,    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,
    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,
    0x8015,    0x003F,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x06D7,

    0x80AA,    0x00AA,    0x8015,    0x003F,
    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,    0x8015,    0x0014,
    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x003F,
    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,    0x8015,    0x0014,
    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x003F,
    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x003F,
    0x8015,    0x0014,    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,
    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,
    0x8015,    0x003F,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x06D7,
};

static const GENERIC_IR_TX_CLOCK_SETTING t =
{
 .clockSrcFreq = 65535,
 .invertOutput = 0,
 .modulateFreq = 38008,
 .clockSrc = 65535,
 .extendedSettings = 0,
 .pwmDutyCycleHighCount = 1,
 .pwmDutyCycleLowCount = 2,
};

#endif

ClockP_Handle rc_timeout_clkHandle_conn;
ClockP_Handle rc_timeout_clkHandle_adv;
#ifdef USE_ZIPIR
ClockP_Handle rc_ZIPIR_clkHandle_adv;
#endif

extern uint16_t connHandle;

void RestartShutdownClock(void)
{
    ClockP_stop(rc_timeout_clkHandle_conn);
    int32_t clockTicks = RC_IDLE_CONN_TIMEOUT * (CLOCK_UNITS_MS);
    ClockP_setTimeout(rc_timeout_clkHandle_conn, clockTicks);
    ClockP_start(rc_timeout_clkHandle_conn);
}

void TriggerShutdown(void)
{
   // Config GPIO to wakeup on interrupt
   GPIO_clearInt(CONFIG_GPIO_BUTTON_0_INPUT);
   GPIO_setConfig(CONFIG_GPIO_BUTTON_0_INPUT, GPIO_CFG_IN_NOPULL | GPIO_CFG_OUT_HIGH | GPIO_CFG_SHUTDOWN_WAKE_HIGH);

   GPIO_clearInt(CONFIG_GPIO_BUTTON_1_INPUT);
   GPIO_setConfig(CONFIG_GPIO_BUTTON_1_INPUT, GPIO_CFG_IN_NOPULL | GPIO_CFG_OUT_HIGH | GPIO_CFG_SHUTDOWN_WAKE_HIGH);

   GPIO_write(CONFIG_KEY_COL_1, 1);
   GPIO_write(CONFIG_KEY_COL_2, 1);
   GPIO_write(CONFIG_KEY_COL_3, 1);
   GPIO_write(CONFIG_KEY_COL_4, 1);
   GPIO_write(CONFIG_KEY_COL_5, 1);

   GPIO_clearInt(CONFIG_KEY_ROW_1);
   GPIO_setConfig(CONFIG_KEY_ROW_1, GPIO_CFG_IN_PD | GPIO_CFG_SHUTDOWN_WAKE_HIGH);

   GPIO_clearInt(CONFIG_KEY_ROW_2);
   GPIO_setConfig(CONFIG_KEY_ROW_2, GPIO_CFG_IN_PD | GPIO_CFG_SHUTDOWN_WAKE_HIGH);

   GPIO_clearInt(CONFIG_KEY_ROW_3);
   GPIO_setConfig(CONFIG_KEY_ROW_3, GPIO_CFG_IN_PD | GPIO_CFG_SHUTDOWN_WAKE_HIGH);

   GPIO_clearInt(CONFIG_KEY_ROW_4);
   GPIO_setConfig(CONFIG_KEY_ROW_4, GPIO_CFG_IN_PD | GPIO_CFG_SHUTDOWN_WAKE_HIGH);

   int_fast16_t  Status = PowerCC23X0_notify(PowerLPF3_ENTERING_SHUTDOWN);

   if(Status == Power_SOK)
   {
       Power_shutdown(0,0);
   }
}

void triggerDisconnect(void)
{
    GAP_TerminateLinkReq(connHandle, HCI_DISCONNECT_REMOTE_USER_TERM);
}

static void rc_timeout_clkHandleCB_conn(void)
{
    BLEAppUtil_invokeFunctionNoData(triggerDisconnect);
}

void ConfigPowerShutdownConn_start(void)
{
    ClockP_Params clockpParams;
    static ClockP_Struct rc_timeout_clkStructConn;
    ClockP_Params_init(&clockpParams);
    uint32_t clockTicks = RC_IDLE_CONN_TIMEOUT * (CLOCK_UNITS_MS); //set timeout
    clockpParams.period = clockTicks; //config for one-shot timer
    clockpParams.startFlag = true; //start now
    clockpParams.arg = (uintptr_t)rc_timeout_clkHandleCB_conn;
    rc_timeout_clkHandle_conn = ClockP_construct(&rc_timeout_clkStructConn, (void *)BLEAppUtil_invokeFunctionNoData, clockTicks, &clockpParams); //Initialize clock instance.
}

static void rc_timeout_clkHandleCB_adv(void)
{
    TriggerShutdown();
}

void ConfigPowerShutdownAdv_start(void)
{
    ClockP_Params clockpParams;
    static ClockP_Struct rc_timeout_clkStructAdv;
    ClockP_Params_init(&clockpParams);
    uint32_t clockTicks = RC_IDLE_ADV_TIMEOUT * (CLOCK_UNITS_MS); //set timeout
    clockpParams.period = clockTicks; //config for one-shot timer
    clockpParams.startFlag = true; // start now
    clockpParams.arg = (uintptr_t)rc_timeout_clkHandleCB_adv;
    rc_timeout_clkHandle_adv = ClockP_construct(&rc_timeout_clkStructAdv, (void *)BLEAppUtil_invokeFunctionNoData, clockTicks, &clockpParams); // Initialize clock instance.
}

#ifdef USE_ZIPIR
void StopZIPIRClock(void)
{
    ClockP_stop(rc_ZIPIR_clkHandle_adv);
    ClockP_destruct(rc_ZIPIR_clkHandle_adv);
    GAP_TerminateLinkReq(connHandle, HCI_DISCONNECT_REMOTE_USER_TERM);
}

static void rc_timeout_clkHandleCB_ZIPIR(void)
{
    ZipIR_Init(gpio, pwmChannel);
    //Power_setConstraint(PowerLPF3_DISALLOW_STANDBY);
    GENERIC_IR_BT_CLOCK_ENABLE(TRUE);
    GENERIC_IRTX_SEND_DATA(encoded_BlePairing, numEncoding, t);
    // at this point, safe to assume the full data is sent
    GENERIC_IRTX_ABORT_CURRENT_TRANSACTION();
    GENERIC_IR_BT_CLOCK_ENABLE(FALSE);
}

void ConfigZIPIRPairing_start(void)
{
    ClockP_Params clockpParams;
    static ClockP_Struct rc_timeout_clkStructZIPIR;
    ClockP_Params_init(&clockpParams);
    uint32_t clockTicks = 4000 * (CLOCK_UNITS_MS); //set timeout
    clockpParams.period = clockTicks; //config for one-shot timer
    clockpParams.startFlag = true; // start now
    clockpParams.arg = (uintptr_t)rc_timeout_clkHandleCB_ZIPIR;
    rc_ZIPIR_clkHandle_adv = ClockP_construct(&rc_timeout_clkStructZIPIR, (void *)BLEAppUtil_invokeFunctionNoData, clockTicks, &clockpParams); // Initialize clock instance.
}
#endif
