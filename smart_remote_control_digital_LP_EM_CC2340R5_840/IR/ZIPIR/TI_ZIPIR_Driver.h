/******************************************************************************

 @file  TI_ZIPIR_Driver.h

 @brief This file contains the ZIPIR main functions.

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

#ifdef USE_ZIPIR

#ifndef ZIPIR_TI_ZIPIR_DRIVER_H_
#define ZIPIR_TI_ZIPIR_DRIVER_H_

#include "typedefs.h"

// TI stuff 2025-03-14
#include <stdint.h>
#include <ti/devices/cc23x0r5/inc/hw_memmap.h>
#include <ti/devices/cc23x0r5/inc/hw_evtsvt.h>
#include <ti/devices/cc23x0r5/inc/hw_lgpt.h>
#include <ti/devices/cc23x0r5/inc/hw_clkctl.h>
#include <ti/devices/cc23x0r5/inc/hw_ioc.h>
#include <ti/devices/cc23x0r5/driverlib/interrupt.h>
#include <ti/drivers/timer/LGPTimerLPF3.h>
#include <ti/drivers/dpl/SemaphoreP.h>

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
#ifdef USE_ZIPIR
    uint32_t    carrierFreq_Hz; // ZIPIR needs period resolution of 100ns
#else
    uint32_t    carrierFreq_kHz;
#endif

#ifdef USE_ZIPIR
    uint32_t    pwmDutyCycleHighCount;
    uint32_t    pwmDutyCycleLowCount;
#elif USE_IRLPGT
    uint32_t    frameInterval_usec;
#endif
    uint32_t    carrierIdleLevel;
} PDEIRLPF3_Params;


typedef struct
{
#ifdef USE_ZIPIR
// encoding will be done within UEI's driver
    uint16_t *pIrEncoding;
    uint16_t numEncoding;
    uint16_t index;
//    uint8_t blocking; // may need to block in some cases...?
#elif USE_IRLPGT
    uint8_t customCode;
    uint8_t dataCode;
#endif
} PDEIRGENLPF3_Transaction;


#ifdef USE_ZIPIR
// no clear way for ZIPIR UEI code to pass state to TI IR code
typedef enum
{
    PDEIRLPF3_STATE_IDLE,
    PDEIRLPF3_STATE_ACTIVE
} PDEIRLPF3_State;
#else
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
#endif

typedef struct
{
    SemaphoreP_Struct transferComplete;

    uint8_t     isOpen;
#ifdef USE_ZIPIR
    // UEI ZIPIR driver will handle timing details
#elif USE_IRLPGT
    uint32_t    bitIndex;
    uint32_t    countPassed;
    uint32_t    frameInterval_tick;
#endif
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

typedef const PDEIRLPF3_Config *PDEIRLPF3_Handle;

extern void PDEIRLPF3_init();
extern void PDEIRLPF3_Params_init(PDEIRLPF3_Params *pParams);
extern PDEIRLPF3_Handle PDEIRLPF3_open(uint8_t index, PDEIRLPF3_Params *pParams);
extern PDEIRLPF3_Status PDEIRLPF3_transfer(PDEIRLPF3_Handle handle, PDEIRGENLPF3_Transaction *transaction);
extern PDEIRLPF3_Status PDEIRLPF3_stop(PDEIRLPF3_Handle handle);
extern void PDEIRLPF3_close(PDEIRLPF3_Handle handle);
// end TI stuff 2025-03-14

// Platform types
typedef struct
{
    uint32_t    clockSrcFreq;
    uint8_t     invertOutput;
    uint32_t    modulateFreq;
    uint32_t    clockSrc;
    uint32_t    extendedSettings;
    uint32_t    pwmDutyCycleHighCount;
    uint32_t    pwmDutyCycleLowCount;
} GENERIC_IR_TX_CLOCK_SETTING; // TODO: confirm types are ok

// Platform defines
#define ACLK1       (0xFFFF) // TODO: specify the clock
#define GENERIC_CLK_FREQ_24_MHZ  (0xFFFF) // TODO: check specific value here to use 24MHz clock
#define GENERIC_CLK_SOURCE  (0xFFFF) // TODO: select clock source which can provide 24MHz

#define GENERIC_INVALID_APP_TIMER_ID (0xFFFF) // TODO: check if this value indicates invalid

#define GPIO_OUTPUT_ENABLE (0) // TODO: set this with actual val to enable output

// Platform functions
extern void GENERIC_GPIO_CONFIGURE_PIN(uint32_t port, uint32_t pin, uint32_t dir, uint32_t pinInitState);
extern void GENERIC_IRTX_SET_PORT_PIN(uint32_t port, uint32_t pin);

extern void GENERIC_IRTX_INIT();
extern void GENERIC_IRTX_ABORT_CURRENT_TRANSACTION();
extern bool GENERIC_IRTX_IS_AVAILABLE();
extern void GENERIC_IRTX_SEND_DATA(U16 *_irtx_data_ptr, U16 _irIndex, GENERIC_IR_TX_CLOCK_SETTING _ir_TxClkSetting);

extern void GENERIC_INIT_APP_TIMER();
extern int GENERIC_START_APP_TIMER(void (*_IrDurationCallback)(void), U32 _duration);
extern void GENERIC_STOP_APP_TIMER(int _irDurationTimerID);

// IR types
typedef enum
{
    IR_TX_IDLE = 0,
    IR_TX_PAYLOAD = 1,
    IR_TX_BUSY = 2,
    IR_TX_DONE = 3,
} IR_TX_STATE;

typedef struct
{
    IR_TX_STATE irtx_state;
} IR_TX_Driver_State;

extern IR_TX_Driver_State *irtxDrvState;

// IR defines
#define GENERIC_IRTX_REG_SETTINGS_MASK (0) // TODO: select the mask for target functionality
#define GENERIC_IRTX_REG_SETTINGS (0) // TODO: select the mask for target functionality

// IR functions
extern void (*GENERIC_IRTX_POST_HOOK)(IR_TX_STATE state);


#endif /* ZIPIR_TI_ZIPIR_DRIVER_H_ */

#endif
