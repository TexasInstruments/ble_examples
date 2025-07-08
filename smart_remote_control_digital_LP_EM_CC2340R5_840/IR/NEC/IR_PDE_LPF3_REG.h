/******************************************************************************

@file  IR_PDE_LPF3_REG.h

@brief This file contains the application main functionality

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2025, Texas Instruments Incorporated
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

#ifndef USE_ZIPIR

#ifndef PDE_IR_GEN_IR_PDE_LPF3_REG_H_
#define PDE_IR_GEN_IR_PDE_LPF3_REG_H_

#include <stdint.h>
#include <ti/devices/cc23x0r5/inc/hw_memmap.h>
#include <ti/devices/cc23x0r5/inc/hw_evtsvt.h>
#include <ti/devices/cc23x0r5/inc/hw_lgpt.h>
#include <ti/devices/cc23x0r5/inc/hw_clkctl.h>
#include <ti/devices/cc23x0r5/inc/hw_ioc.h>
#include <ti/devices/cc23x0r5/driverlib/interrupt.h>
#include <ti/drivers/timer/LGPTimerLPF3.h>

#define PDEIRLPF3_CONFIG_0   (0)

#define PDEIRLPF3_CONFIG_QTY  (1)

/*
 * These are based on CC2340R5 TRM section "10.4.3 IR Generation"
 */
#define LGPTLPF3_CARRIER_BASE    (LGPT0_BASE)
#define LGPTLPF3_MODULATOR_BASE  (LGPT1_BASE)
#define LGPTLPF3_CHANNEL         (0) // always use channel 0 for given timer


typedef struct
{
    uint32_t    carrierFreq_kHz; // maybe assume this is constant for now
    uint32_t    frameInterval_usec;
    uint32_t    carrierIdleLevel;
} PDEIRLPF3_Params;

typedef struct
{
    uint8_t customCode;
    uint8_t dataCode;
} PDEIRGENLPF3_Transaction;

typedef enum
{
    PDEIRLPF3_STATE_IDLE,
    PDEIRLPF3_STATE_STARTING,
    PDEIRLPF3_STATE_LEAD_CODE,
    PDEIRLPF3_STATE_CUST_CODE_1,
    PDEIRLPF3_STATE_CUST_CODE_2_N,
    PDEIRLPF3_STATE_DATA_CODE_1,
    PDEIRLPF3_STATE_DATA_CODE_2,
    PDEIRLPF3_STATE_STOP_BIT,
    PDEIRLPF3_STATE_REPEAT_CODE_1,
    PDEIRLPF3_STATE_REPEAT_CODE_2
} PDEIRLPF3_State;

typedef struct
{
    uint8_t     isOpen;
    uint32_t    bitIndex;
    uint32_t    countPassed;
    uint32_t    frameInterval_tick;
    uint32_t    carrierIdleLevel;

    uint8_t LgptCarrier_index;
    uint8_t LgptModulator_index;

    LGPTimerLPF3_Handle LgptCarrierHandle;
    LGPTimerLPF3_Handle LgptModulatorHandle;

    PDEIRLPF3_State     state;
} PDEIRLPF3_Object;

typedef struct
{
    uint32_t LgptModulatorBaseAddr;
    uint32_t LgptCarrierBaseAddr;

    uint32_t LgptModulatorInts;
} PDEIRLPF3_HWAttrs;

typedef struct
{
    PDEIRLPF3_Object    *object;
    const PDEIRLPF3_HWAttrs   *hwAttrs;
} PDEIRLPF3_Config;

typedef enum
{
    PDEIRLPF3_STATUS_OK = 0,
    PDEIRLPF3_STATUS_BAD_STATE,
    PDEIRLPF3_STATUS_INVALID_PAR,
    PDEIRLPF3_STATUS_BUSY,
} PDEIRLPF3_Status;

typedef PDEIRLPF3_Config *PDEIRLPF3_Handle;

extern void PDEIRLPF3_init();
extern void PDEIRLPF3_Params_init(PDEIRLPF3_Params *pParams);
extern PDEIRLPF3_Handle PDEIRLPF3_open(uint8_t index, PDEIRLPF3_Params *pParams);
extern PDEIRLPF3_Status PDEIRLPF3_transfer(PDEIRLPF3_Handle handle, PDEIRGENLPF3_Transaction *transaction);
extern PDEIRLPF3_Status PDEIRLPF3_stop(PDEIRLPF3_Handle handle);
extern void PDEIRLPF3_close(PDEIRLPF3_Handle handle);


#endif /* PDE_IR_GEN_IR_PDE_LPF3_REG_H_ */

#endif
