/******************************************************************************

@file  IR_PDE_LPF3_REG.c

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

#include "IR/NEC/IR_PDE_LPF3_REG.h"

#include <ti/drivers/timer/LGPTimerLPF3.h>
#include "IR/LGPTTimerLPF3_ext.h"
#include <ti/drivers/Power.h>
#include <ti/drivers/GPIO.h>
#include <ti/devices/cc23x0r5/inc/hw_memmap.h>
#include <ti/devices/cc23x0r5/inc/hw_ints.h>
#include <ti/devices/cc23x0r5/inc/hw_types.h>
#include <ti/devices/cc23x0r5/inc/hw_lgpt.h>
#include <ti/devices/cc23x0r5/inc/hw_evtsvt.h>
#include "ti_drivers_config.h"

// for debug purposes
#define STATIC

#define CARRIER_FREQ_kHz    (38)
//#define TICK_FROM_USEC(usec) ((usec)*CARRIER_FREQ_kHz/1000)
#define TICK_FROM_USEC(usec) ((10*(usec)*CARRIER_FREQ_kHz+5000)/10000)

// this is for offset due to overhead from interrupt handling
#define CAL_OFFSET_USEC (31)
#define CAL_STATIC_OFFSET_USEC (31)
#define CAL_OFFSET_TICK (TICK_FROM_USEC(CAL_OFFSET_USEC))
//#define FRAME_CNT_CAL(frameCount) (frameCount - CAL_OFFSET_TICK)
#define FRAME_CNT_CAL(frameCount) (frameCount)

extern const LGPTimerLPF3_Config LGPTimerLPF3_config[];
extern const uint_least8_t LGPTimerLPF3_count;

static volatile uint8_t gInitDone = 0;
static PDEIRGENLPF3_Transaction gCurTrans;

typedef struct
{
    // how many cycles of carrier wave freq that it will be active
    uint16_t activeCount;

    // how many cycles of carrier wave freq total for the length
    uint16_t frameCount;
} PDEIR_Encoding;

static const uint16_t CAL_ACTIVE_PLUS_TICK_OFFSET = 1;
static const uint16_t CAL_FRAME_MINUS_TICK_OFFSET = 1;
static const uint16_t CAL_BIT_VAL_1_FRAME_MINUS_TICK_OFFSET = 2;
static const uint16_t CAL_BIT_VAL_0_FRAME_MINUS_TICK_OFFSET = 0;
static const uint16_t CAL_REPEAT2_PLUS_TICK_OFFSET = 17; //14; // 12;
static const uint16_t CAL_STOP_BIT_MINUS_TICK_OFFSET = 25; //49;

static inline uint16_t CNT_PRE_CAL_ACTIVE(uint16_t cntPostCal)
{
    return cntPostCal - CAL_ACTIVE_PLUS_TICK_OFFSET;
}
static inline uint16_t CNT_PRE_CAL_FRAME(uint16_t cntPostCal)
{
    return cntPostCal + CAL_FRAME_MINUS_TICK_OFFSET;
}
static inline uint16_t CNT_POST_CAL_FRAME(uint16_t cntPreCal)
{
    return cntPreCal - CAL_FRAME_MINUS_TICK_OFFSET;
}


static const PDEIR_Encoding NEC_lead =
{
     .activeCount = TICK_FROM_USEC(4500) + CAL_ACTIVE_PLUS_TICK_OFFSET,
     .frameCount = TICK_FROM_USEC(4500 + 4500) - CAL_FRAME_MINUS_TICK_OFFSET
};

static const PDEIR_Encoding NEC_bitval_0 =
{
     .activeCount = TICK_FROM_USEC(560) + CAL_ACTIVE_PLUS_TICK_OFFSET,
     .frameCount = TICK_FROM_USEC(1125) - CAL_FRAME_MINUS_TICK_OFFSET - CAL_BIT_VAL_0_FRAME_MINUS_TICK_OFFSET
};
static const PDEIR_Encoding NEC_bitval_1 =
{
     .activeCount = TICK_FROM_USEC(560) + CAL_ACTIVE_PLUS_TICK_OFFSET,
     .frameCount =  TICK_FROM_USEC(2250) + CAL_FRAME_MINUS_TICK_OFFSET - CAL_BIT_VAL_1_FRAME_MINUS_TICK_OFFSET
};
static PDEIR_Encoding NEC_stopBit =
{
     .activeCount = TICK_FROM_USEC(560) + CAL_ACTIVE_PLUS_TICK_OFFSET
     // frameCount is dynamic, and should last til end of the full frameInterval
};

// stop bit should be modulated til the end of the frame_interval
//static PDEIR_Encoding NEC_stop_bit;

static const PDEIR_Encoding NEC_repeat[2] =
{
     {
          .activeCount = TICK_FROM_USEC(4500) + CAL_ACTIVE_PLUS_TICK_OFFSET,
          .frameCount = TICK_FROM_USEC(4500 + 2250) - CAL_FRAME_MINUS_TICK_OFFSET
     },
     {
          .activeCount = TICK_FROM_USEC(560) + CAL_ACTIVE_PLUS_TICK_OFFSET,
          .frameCount = TICK_FROM_USEC(108000 - 4500 - 2250 - 560) - CAL_FRAME_MINUS_TICK_OFFSET + CAL_REPEAT2_PLUS_TICK_OFFSET
     }
};

static const PDEIRLPF3_Params defaultParams =
{
 // for NEC-like
 .carrierFreq_kHz = 38,
 .frameInterval_usec = 108000,
 .carrierIdleLevel  = LGPT_OUTCTL_CLROUT0
};

static PDEIRLPF3_Object object;
static const PDEIRLPF3_HWAttrs hwAttrs =
{
     .LgptCarrierBaseAddr = LGPTLPF3_CARRIER_BASE,
     .LgptModulatorBaseAddr = LGPTLPF3_MODULATOR_BASE,
     .LgptModulatorInts  =  LGPT_IMSET_ZERO_SET
};

STATIC const PDEIRLPF3_Config PDEIRLPF3_configs[PDEIRLPF3_CONFIG_QTY] =
{
    // PDEIRLPF3_CONFIG_0
    {
        .object   = &object,
        .hwAttrs  = &hwAttrs
    }
};

static uint8_t setLgptIndexes(PDEIRLPF3_Config *config);
static uint8_t isMatchedLgptIndexes(PDEIRLPF3_Config *config);
static PDEIRLPF3_Status initHw(PDEIRLPF3_Handle handle, PDEIRLPF3_Params *params);
static LGPTimerLPF3_ChannelNo getLgptChan(uint8_t lgptConfigIndex);
static void PDEIRLPF3_modulator_callback();
static void PDEIRLPF3_resetLgpt(PDEIRLPF3_Handle handle);


void PDEIRLPF3_init()
{
    // not sure if anything is needed to initialize yet...

    if (gInitDone)
    {
        return;
    }

    {
        object.isOpen = 0;
        object.state = PDEIRLPF3_STATE_IDLE;
        LGPTimerLPF3_init();
    }

    gInitDone = 1;
}
void PDEIRLPF3_Params_init(PDEIRLPF3_Params *pParams)
{
    if (pParams)
    {
        *pParams = defaultParams;
    }
}
PDEIRLPF3_Handle PDEIRLPF3_open(uint8_t index, PDEIRLPF3_Params *pParams)
{
    if (index > PDEIRLPF3_CONFIG_QTY) {
        return 0;
    }

    PDEIRLPF3_Handle handle             = &PDEIRLPF3_configs[index];

    PDEIRLPF3_Object *object            = handle->object;
    PDEIRLPF3_HWAttrs const *hwAttrs    = handle->hwAttrs;
    LGPTimerLPF3_Object *carrierObject   = object->LgptCarrierHandle->object;
    LGPTimerLPF3_Object *modulatorObject = object->LgptModulatorHandle->object;

    uint32_t key = HwiP_disable();
    if (object->isOpen || carrierObject->isOpen || modulatorObject->isOpen)
    {
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        HwiP_restore(key);
        return handle;
    }
    object->isOpen = 1;
    carrierObject->isOpen = 1;
    modulatorObject->isOpen = 1;
    HwiP_restore(key);

    Power_setConstraint(PowerLPF3_DISALLOW_STANDBY);
    Power_setDependency(PowerLPF3_PERIPH_LGPT0);
    Power_setDependency(PowerLPF3_PERIPH_LGPT1);

    PDEIRLPF3_Params params;
    if (pParams)
    {
        params = *pParams;
    }
    else
    {
        params = defaultParams;
    }

    object->bitIndex = 0;
    object->countPassed = 0; // how many ticks of params.carrierFreq_kHz
    object->state = PDEIRLPF3_STATE_IDLE;
    object->carrierIdleLevel = params.carrierIdleLevel;

    if (initHw(handle, &params) != PDEIRLPF3_STATUS_OK)
        while (1); // error

    object->frameInterval_tick = TICK_FROM_USEC(params.frameInterval_usec);

    return handle;

}
PDEIRLPF3_Status PDEIRLPF3_transfer(PDEIRLPF3_Handle handle, PDEIRGENLPF3_Transaction *transaction)
{
    if (!handle)
        return PDEIRLPF3_STATUS_INVALID_PAR;

    uint32_t key = HwiP_disable();
    if (!handle && handle->object->state != PDEIRLPF3_STATE_IDLE)
    {
        HwiP_restore(key);
        return PDEIRLPF3_STATUS_BAD_STATE;
    }
    handle->object->state = PDEIRLPF3_STATE_STARTING;
    HwiP_restore(key);

    // this is new transmission, so clear the object
    // alternatively the object may be cleared after the end of a transaction...
    handle->object->bitIndex = 0;
    handle->object->countPassed = 0;
    gCurTrans = *transaction;

    handle->object->state = PDEIRLPF3_STATE_LEAD_CODE;
    //handle->object->countPassed += CNT_PRE_CAL(NEC_lead.frameCount);
    handle->object->countPassed += CNT_PRE_CAL_FRAME(NEC_lead.frameCount);

    // Set TGT to the length of the symbol, assume lead frame for now
    HWREG(LGPT1_BASE + LGPT_O_TGT) &= ~LGPT_TGT_VAL_M;
    HWREG(LGPT1_BASE + LGPT_O_TGT) |= (NEC_lead.frameCount << LGPT_TGT_VAL_S);

    // Set C0CC to number of 38 kHz pulses in the symbol, assume lead frame for now
    HWREG(LGPT1_BASE + LGPT_O_C0CC) &= ~LGPT_C0CC_VAL_M;
    HWREG(LGPT1_BASE + LGPT_O_C0CC) |= (NEC_lead.activeCount << LGPT_C0CC_VAL_S);

    // clear and enable interrupt for modulator
    IntClearPend(INT_LGPT1_COMB);
    IntEnable(INT_LGPT1_COMB);

    // start the IRGEN system
    // Start carrier wave in up-periodic. CTL.MODE = 2
    HWREG(LGPT0_BASE + LGPT_O_CTL) &= ~LGPT_CTL_MODE_M;
    HWREG(LGPT0_BASE + LGPT_O_CTL) |= LGPT_CTL_MODE_UP_PER;

    // Start modulator in up-periodic. CTL.MODE = 2
    HWREG(LGPT1_BASE + LGPT_O_CTL) &= ~LGPT_CTL_MODE_M;
    HWREG(LGPT1_BASE + LGPT_O_CTL) |= LGPT_CTL_MODE_UP_PER;
}
PDEIRLPF3_Status PDEIRLPF3_stop(PDEIRLPF3_Handle handle)
{
    if (!handle)
        return PDEIRLPF3_STATUS_INVALID_PAR;

    HWREG(handle->hwAttrs->LgptCarrierBaseAddr + LGPT_O_CTL) &= ~LGPT_CTL_MODE_M;
    HWREG(handle->hwAttrs->LgptModulatorBaseAddr) &= ~LGPT_CTL_MODE_M;
    IntDisable(INT_LGPT1_COMB);

    // set carrier output level to idle
    HWREG(handle->hwAttrs->LgptCarrierBaseAddr + LGPT_O_OUTCTL) |= handle->object->carrierIdleLevel;
    GPIO_setConfigAndMux(handle->object->LgptCarrierHandle->hwAttrs->channelConfig[0].pin,
                         GPIO_CFG_OUT_LOW,
                         IOC_IOC3_PORTCFG_BASE);

    return PDEIRLPF3_STATUS_OK;
}
void PDEIRLPF3_close(PDEIRLPF3_Handle handle)
{
    if (!handle)
        return;

    uint32_t key = HwiP_disable();
    if (!handle->object->isOpen)
    {
        HwiP_restore(key);
        return;
    }
    handle->object->isOpen = 0;
    handle->object->LgptCarrierHandle->object->isOpen = 0;
    handle->object->LgptModulatorHandle->object->isOpen = 0;

    PDEIRLPF3_stop(handle);
    PDEIRLPF3_resetLgpt(handle);

    Power_releaseConstraint(PowerLPF3_DISALLOW_STANDBY);
    Power_releaseDependency(PowerLPF3_PERIPH_LGPT0);
    Power_releaseDependency(PowerLPF3_PERIPH_LGPT1);

    HwiP_restore(key);
}

static PDEIRLPF3_Status initHw(PDEIRLPF3_Handle handle, PDEIRLPF3_Params *params)
{
    // initialize LGPTs... LGPT0 for carrier wave and LGPT1 for modulator

    /** from TRM:
        LGPT0 (Carrier wave generation):
        • Set the PRECFG.TICKDIV and TGT to fulfill the following equation:
        (PRECFG.TICKDIV+1)*TGT*2 = 48 MHz / 38 kHz
        E.g. PRECFG.TICKDIV = 3 and TGT 421. This gives a carrier wave of approximately 38 kHz (Theoretically
        38.00475 kHz).
        • Set C0CC = 210 to give roughly 50% duty cycle.
        • Set C0CFG.CCACT = TGL_ON_CMP, and C0CFG.OUT0 = 1.
        • Set IRGEN.CTL = 1.
        • Start timer in up-periodic. CTL.MODE = 2
     */

    // Toby(2024-04-11): use these values instead
    const uint8_t div = 3;
    const uint16_t tgt = 315;
    const uint16_t dutyHigh = tgt/3;
    uint32_t srcClkFreq = params->carrierFreq_kHz * 1000;

    // setup LGPT0 for IR carrier
    {
        const uint8_t ccactConfig = LGPT_C0CFG_CCACT_SET_ON_0_TGL_ON_CMP; // dutyHigh based duty cycle

        // ? start clock for LGPT0 and wait for it to complete?
        HWREG(CLKCTL_BASE + CLKCTL_O_CLKENSET0) |= CLKCTL_CLKENSET0_LGPT0_CLK_SET;
        while ((HWREG(CLKCTL_BASE + CLKCTL_O_CLKCFG0) & CLKCTL_CLKCFG0_LGPT0_M)
                !=  CLKCTL_CLKCFG0_LGPT0_CLK_EN);

        // halt on debug
        HWREG(LGPT0_BASE + LGPT_O_EMU) |= LGPT_EMU_HALT_EN | LGPT_EMU_CTL_IMMEDIATE;

        // Set the PRECFG.TICKDIV and TGT
        HWREG(LGPT0_BASE + LGPT_O_PRECFG) &= ~LGPT_PRECFG_TICKDIV_M;
        HWREG(LGPT0_BASE + LGPT_O_PRECFG) |= (div << LGPT_PRECFG_TICKDIV_S);
        HWREG(LGPT0_BASE + LGPT_O_TGT) &= ~LGPT_TGT_VAL_M;
        HWREG(LGPT0_BASE + LGPT_O_TGT) |= (tgt << LGPT_TGT_VAL_S);

        // Set C0CC to give roughly 33% duty cycle.
        HWREG(LGPT0_BASE + LGPT_O_C0CC) &= ~LGPT_C0CC_VAL_M;
        HWREG(LGPT0_BASE + LGPT_O_C0CC) |= (dutyHigh << LGPT_C0CC_VAL_S);

        // Set C0CFG.CCACT = TGL_ON_CMP, and C0CFG.OUT0 = 1.
        HWREG(LGPT0_BASE + LGPT_O_C0CFG) &= ~LGPT_C0CFG_CCACT_M;
        HWREG(LGPT0_BASE + LGPT_O_C0CFG) |= ccactConfig;
        HWREG(LGPT0_BASE + LGPT_O_C0CFG) &= ~LGPT_C0CFG_OUT0_M;
        HWREG(LGPT0_BASE + LGPT_O_C0CFG) |= LGPT_C0CFG_OUT0_EN;

        // Set IRGEN.CTL = 1.
        HWREG(LGPT0_BASE + LGPT_O_IRGEN) &= ~LGPT_IRGEN_CTL_M;
        HWREG(LGPT0_BASE + LGPT_O_IRGEN) |= LGPT_IRGEN_CTL_EN;
    }

    // setup LGPT1 for IR modulator
    {
        // ? start clock for LGPT0 and wait for it to complete
        HWREG(CLKCTL_BASE + CLKCTL_O_CLKENSET0) |= CLKCTL_CLKENSET0_LGPT1_CLK_SET;
        while ((HWREG(CLKCTL_BASE + CLKCTL_O_CLKCFG0) & CLKCTL_CLKCFG0_LGPT1_M)
                !=  CLKCTL_CLKCFG0_LGPT1_CLK_EN);

        // halt on debug
        HWREG(LGPT1_BASE + LGPT_O_EMU) |= LGPT_EMU_HALT_EN | LGPT_EMU_CTL_IMMEDIATE;

        // enable interrupt for updating the IR symbol
        //IntRegister(INT_LGPT1_COMB, test_lgpt_hwreg_callback);
        IntRegister(INT_LGPT1_COMB, PDEIRLPF3_modulator_callback);
        IntClearPend(INT_LGPT1_COMB);
        IntSetPriority(INT_LGPT1_COMB, INT_PRI_LEVEL0);
        //IntEnable(INT_LGPT1_COMB);
        //HWREG(LGPT1_BASE + LGPT_O_IMSET) |= LGPT_IMSET_TGT_SET | LGPT_IMSET_ZERO_SET;
        HWREG(LGPT1_BASE + LGPT_O_IMSET) |= handle->hwAttrs->LgptModulatorInts;
        // add a phase delay, may help with rounding errors
    //    HWREG(LGPT1_BASE + LGPT_O_CTL) &= ~LGPT_CTL_INTP_M;
    //    HWREG(LGPT1_BASE + LGPT_O_CTL) |= LGPT_CTL_INTP_LATE;

        // Set PRECFG.TICKSRC = FALL_TICK
        HWREG(LGPT1_BASE + LGPT_O_PRECFG) &= ~LGPT_PRECFG_TICKSRC_M;
        HWREG(LGPT1_BASE + LGPT_O_PRECFG) |= LGPT_PRECFG_TICKSRC_FALL_TICK;

        // Set C0CFG.CCACT = SET_ON_0_TGL_ON_CMP, C0CFG.OUT0 = 1
        HWREG(LGPT1_BASE + LGPT_O_C0CFG) &= ~LGPT_C0CFG_CCACT_M;
        HWREG(LGPT1_BASE + LGPT_O_C0CFG) |= LGPT_C0CFG_CCACT_SET_ON_0_TGL_ON_CMP;
        HWREG(LGPT1_BASE + LGPT_O_C0CFG) &= ~LGPT_C0CFG_OUT0_M;
        HWREG(LGPT1_BASE + LGPT_O_C0CFG) |= LGPT_C0CFG_OUT0_EN;

        // Set EVTSVT.LGPT1TENSEL = LGPT0C0
        HWREG(EVTSVT_BASE + EVTSVT_O_LGPT1TENSEL) |= EVTSVT_LGPT1TENSEL_PUBID_LGPT0C0;

        // Set TGT to the length of the symbol, assume lead frame for now
        HWREG(LGPT1_BASE + LGPT_O_TGT) &= ~LGPT_TGT_VAL_M;
        HWREG(LGPT1_BASE + LGPT_O_TGT) |= (NEC_lead.frameCount << LGPT_TGT_VAL_S);

        // Set C0CC to number of 38 kHz pulses in the symbol, assume lead frame for now
        HWREG(LGPT1_BASE + LGPT_O_C0CC) &= ~LGPT_C0CC_VAL_M;
        HWREG(LGPT1_BASE + LGPT_O_C0CC) |= (NEC_lead.activeCount << LGPT_C0CC_VAL_S);
    }

    // configure IR pin
    {
        HWREG(IOC_BASE + IOC_O_IOC11) |= IOC_IOC11_PORTCFG_PFUNC3;
    }

    return PDEIRLPF3_STATUS_OK;
}

static void PDEIRLPF3_resetLgpt(PDEIRLPF3_Handle handle)
{
    // inspired from LGPTimerLPF3_resetHw

    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint32_t base                       = hwAttrs->baseAddr;
    const LGPTimerLPF3_IntMask intMask  = (LGPTimerLPF3_INT_TGT | LGPTimerLPF3_INT_ZERO |
                                           LGPTimerLPF3_INT_COUNTER_CHANGE |
                                           LGPTimerLPF3_INT_DIR_CHANGE |
                                           LGPTimerLPF3_INT_CH0_CC | LGPTimerLPF3_INT_CH1_CC |
                                           LGPTimerLPF3_INT_CH2_CC);

    /* Disable timer and reset channels */
    uint32_t regVal          = (LGPT_CTL_C2RST | LGPT_CTL_C1RST | LGPT_CTL_C0RST | LGPT_CTL_MODE_DIS);
    HWREG(handle->hwAttrs->LgptCarrierBaseAddr + LGPT_O_CTL) = regVal;
    HWREG(handle->hwAttrs->LgptModulatorBaseAddr + LGPT_O_CTL) = regVal;

    /* Reset interrupt mask */
    LGPTimerLPF3_disableInterrupt(handle, intMask);
    HWREG(LGPT1_BASE + LGPT_O_IMCLR) |= intMask;

    /* Clear interrupt statuses */
    HWREG(handle->hwAttrs->LgptCarrierBaseAddr + LGPT_O_ICLR) = intMask;
    HWREG(handle->hwAttrs->LgptModulatorBaseAddr + LGPT_O_ICLR) = intMask;

    /* Reset timer counter register */
    HWREG(handle->hwAttrs->LgptCarrierBaseAddr + LGPT_O_CNTR) = 0x0;
    HWREG(handle->hwAttrs->LgptModulatorBaseAddr + LGPT_O_CNTR) = 0x0;


    /* Reset timer counter target registers.
     * Supported counter widths are 16 bits and 24 bits.
     */
    const uint32_t resetVal = 0xFFFF; // COUNTER_MASK_16_BIT;
    HWREG(handle->hwAttrs->LgptCarrierBaseAddr + LGPT_O_TGT)    = resetVal;
    HWREG(handle->hwAttrs->LgptCarrierBaseAddr + LGPT_O_TGTNC)  = resetVal;
    HWREG(handle->hwAttrs->LgptCarrierBaseAddr + LGPT_O_PTGT)   = 0x00;
    HWREG(handle->hwAttrs->LgptCarrierBaseAddr + LGPT_O_PTGTNC) = 0x00;
    HWREG(handle->hwAttrs->LgptModulatorBaseAddr + LGPT_O_TGT)    = resetVal;
    HWREG(handle->hwAttrs->LgptModulatorBaseAddr + LGPT_O_TGTNC)  = resetVal;
    HWREG(handle->hwAttrs->LgptModulatorBaseAddr + LGPT_O_PTGT)   = 0x00;
    HWREG(handle->hwAttrs->LgptModulatorBaseAddr + LGPT_O_PTGTNC) = 0x00;

    /* Reset pre-scaler */
    HWREG(handle->hwAttrs->LgptCarrierBaseAddr + LGPT_O_PRECFG)   = 0;

    /* Reset debug configuration */
    HWREG(base + LGPT_O_EMU) = 0;
    HWREG(handle->hwAttrs->LgptCarrierBaseAddr + LGPT_O_EMU) = 0;
    HWREG(handle->hwAttrs->LgptModulatorBaseAddr + LGPT_O_EMU) = 0;

    HWREG(EVTSVT_BASE + EVTSVT_O_LGPT1TENSEL) &= ~EVTSVT_LGPT1TENSEL_PUBID_LGPT0C0;
}

static void PDEIRLPF3_modulator_callback()
{
    // only expect this to be the Modulator
    // if nothing is done here, then the same symbol should output on the IR
    // Update PTGT on RIS.ZERO interrupt to send a new symbol.

   // return;

    uint32_t lgptMis = HWREG(LGPT1_BASE + LGPT_O_MIS);

    if (lgptMis & LGPT_MIS_ZERO_SET)
    {
        uint32_t savedModeLgpt1 = HWREG(LGPT1_BASE + LGPT_O_CTL) & LGPT_CTL_MODE_M;
        HWREG(LGPT1_BASE + LGPT_O_CTL) &= ~LGPT_CTL_MODE_M;

        // TODO: consider setting the level of LGPT1 out?
        PDEIRLPF3_Handle pdeIrHandle = (PDEIRLPF3_Handle) &PDEIRLPF3_configs[0];
        PDEIRLPF3_Object *pdeIrObj = pdeIrHandle->object;

        uint16_t activeCount;
        uint16_t frameCount;

        switch (pdeIrObj->state)
        {

        case PDEIRLPF3_STATE_LEAD_CODE:
        {
            // finished sending lead code, ready to send next part
            pdeIrObj->state = PDEIRLPF3_STATE_CUST_CODE_1;

            if ((1 << pdeIrObj->bitIndex) & gCurTrans.customCode)
            {
                // bit is 1
                activeCount = NEC_bitval_1.activeCount;
                frameCount  = NEC_bitval_1.frameCount;
            }
            else
            {
                // bit is 0
                activeCount = NEC_bitval_0.activeCount;
                frameCount  = NEC_bitval_0.frameCount;
            }
            pdeIrObj->bitIndex += 1;
        } break;

        case PDEIRLPF3_STATE_CUST_CODE_1:
        {
            if ((1 << pdeIrObj->bitIndex) & gCurTrans.customCode)
            {
                // bit is 1
                activeCount = NEC_bitval_1.activeCount;
                frameCount  = NEC_bitval_1.frameCount;
            }
            else
            {
                // bit is 0
                activeCount = NEC_bitval_0.activeCount;
                frameCount  = NEC_bitval_0.frameCount;
            }
            pdeIrObj->bitIndex += 1;
            if (pdeIrObj->bitIndex >= 8)
            {
                pdeIrObj->state    = PDEIRLPF3_STATE_CUST_CODE_2_N;
                pdeIrObj->bitIndex = 0;
            }
            //pdeIrObj->countPassed += frameCount;
            pdeIrObj->countPassed += CNT_PRE_CAL_FRAME(frameCount);
        } break;

        case PDEIRLPF3_STATE_CUST_CODE_2_N:
        {
            if ((1 << pdeIrObj->bitIndex) & gCurTrans.customCode)
            {
                activeCount = NEC_bitval_1.activeCount;
                frameCount  = NEC_bitval_1.frameCount;
                // bit is 1, but we need the 1's complement for this state
/*                activeCount = NEC_bitval_0.activeCount;
                frameCount  = NEC_bitval_0.frameCount;*/
            }
            else
            {
                activeCount = NEC_bitval_0.activeCount;
                frameCount  = NEC_bitval_0.frameCount;
                // bit is 0, but we need the 1's complement for this state
/*                activeCount = NEC_bitval_1.activeCount;
                frameCount  = NEC_bitval_1.frameCount;*/
            }
            pdeIrObj->bitIndex += 1;
            if (pdeIrObj->bitIndex >= 8)
            {
                pdeIrObj->state    = PDEIRLPF3_STATE_DATA_CODE_1;
                pdeIrObj->bitIndex = 0;
            }
            //pdeIrObj->countPassed += frameCount;
            pdeIrObj->countPassed += CNT_PRE_CAL_FRAME(frameCount);
        } break;

        case PDEIRLPF3_STATE_DATA_CODE_1:
        {
            if ((1 << pdeIrObj->bitIndex) & gCurTrans.dataCode)
            {
                // bit is 1
                activeCount = NEC_bitval_1.activeCount;
                frameCount  = NEC_bitval_1.frameCount;
            }
            else
            {
                // bit is 0
                activeCount = NEC_bitval_0.activeCount;
                frameCount = NEC_bitval_0.frameCount;
            }
            pdeIrObj->bitIndex += 1;
            if (pdeIrObj->bitIndex >= 8)
            {
                pdeIrObj->state    = PDEIRLPF3_STATE_DATA_CODE_2;
                pdeIrObj->bitIndex = 0;
            }
            //pdeIrObj->countPassed += frameCount;
            pdeIrObj->countPassed += CNT_PRE_CAL_FRAME(frameCount);
        } break;

        case PDEIRLPF3_STATE_DATA_CODE_2:
        {
            if ((1 << pdeIrObj->bitIndex) & gCurTrans.dataCode)
            {
                // bit is 1, but we need complement
                activeCount = NEC_bitval_0.activeCount;
                frameCount  = NEC_bitval_0.frameCount;
            }
            else
            {
                // bit is 0, but we need complement
                activeCount = NEC_bitval_1.activeCount;
                frameCount  = NEC_bitval_1.frameCount;
            }
            pdeIrObj->bitIndex += 1;
            if (pdeIrObj->bitIndex >= 8)
            {
                pdeIrObj->state = PDEIRLPF3_STATE_STOP_BIT;
                pdeIrObj->bitIndex = 0;
            }
            //pdeIrObj->countPassed += frameCount;
            pdeIrObj->countPassed += CNT_PRE_CAL_FRAME(frameCount);
        } break;

        case PDEIRLPF3_STATE_STOP_BIT:
        {
            /*
             * Stop bit consists of active and idle portions.
             * Active portion is always 560usec.
             * Idle portion is remaining time until frame ends i.e. ticks(108msec) - countPassed
             */
            activeCount = NEC_stopBit.activeCount;
            // idle portion of stop bit is equal to ticks(108msec) - countPassed.
            // countPassed should be the pre-calibrated count passed.
            // for frameCount, need to add at least a single calibration.
            //frameCount = pdeIrObj->frameInterval_tick - pdeIrObj->countPassed;
            pdeIrObj->countPassed += CNT_PRE_CAL_FRAME(NEC_stopBit.activeCount);
            frameCount = pdeIrObj->frameInterval_tick - pdeIrObj->countPassed - CAL_STOP_BIT_MINUS_TICK_OFFSET;
            pdeIrObj->state = PDEIRLPF3_STATE_REPEAT_CODE_1;
        } break;

        case PDEIRLPF3_STATE_REPEAT_CODE_1:
        {
            activeCount = NEC_repeat[0].activeCount;
            frameCount = NEC_repeat[0].frameCount;
            pdeIrObj->state = PDEIRLPF3_STATE_REPEAT_CODE_2;
        } break;

        case PDEIRLPF3_STATE_REPEAT_CODE_2:
        {
            activeCount = NEC_repeat[1].activeCount;
            frameCount = NEC_repeat[1].frameCount;
            pdeIrObj->state = PDEIRLPF3_STATE_REPEAT_CODE_1;
        } break;

        default:
            while (1); // should never get here...
        }
        // Set TGT to the length of the symbol.
        HWREG(LGPT1_BASE + LGPT_O_TGT) &= ~LGPT_TGT_VAL_M;
        HWREG(LGPT1_BASE + LGPT_O_TGT) |= frameCount << LGPT_TGT_VAL_S;
        // Set C0CC to number of 38 kHz pulses in the symbol.
        HWREG(LGPT1_BASE + LGPT_O_C0CC) &= ~LGPT_C0CC_VAL_M;
        HWREG(LGPT1_BASE + LGPT_O_C0CC) |= activeCount << LGPT_C0CC_VAL_S;

        // restore timer
        HWREG(LGPT1_BASE + LGPT_O_CTL) |= savedModeLgpt1;
    }
}

#endif
