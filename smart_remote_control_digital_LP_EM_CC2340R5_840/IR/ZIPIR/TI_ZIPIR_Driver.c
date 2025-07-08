/******************************************************************************

 @file  TI_ZIPIR_Driver.c

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

#include "typedefs.h"
#include "TI_ZIPIR_Driver.h"
#include "ZipIR_API.h"
#include "ZipIRBlaster.h"
#include "ZipIRDecoder.h"
#include "ZipIRInterface.h"

// TI stuff 2025-03-14

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

extern const LGPTimerLPF3_Config LGPTimerLPF3_config[];
extern const uint_least8_t LGPTimerLPF3_count;

static volatile uint8_t gInitDone = 0;
static PDEIRGENLPF3_Transaction gCurTrans;

static const PDEIRLPF3_Params defaultParams =
{
#ifdef USE_ZIPIR
 // ZIPIR needs resolution of carrier period at 100nsec
 .carrierFreq_Hz = 38000,
 // ZIPIR sets timing
 .pwmDutyCycleHighCount = 1,
 .pwmDutyCycleLowCount = 1,
#else
 // for NEC-like
 .carrierFreq_kHz = 38,
 .frameInterval_usec = 108000,
#endif
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
static uint8_t setupNextZipirFrame();

static PDEIRLPF3_Handle gHandle = 0;

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
        //GPIO_write(CONFIG_GPIO_LED_GREEN,1); cleanup_todo: this was probably for debug
        HwiP_restore(key);
        return 0;
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
#ifdef USE_ZIPIR
    object->state = PDEIRLPF3_STATE_IDLE;
#elif USE_IRLPGT
    object->bitIndex = 0;
    object->countPassed = 0; // how many ticks of params.carrierFreq_kHz
    object->state = PDEIRLPF3_STATE_IDLE;
#endif

    object->carrierIdleLevel = params.carrierIdleLevel;

    if (initHw(handle, &params) != PDEIRLPF3_STATUS_OK)
        while (1); // error

#ifdef USE_ZIPIR

#elif USE_IRLPGT
    object->frameInterval_tick = TICK_FROM_USEC(params.frameInterval_usec);
#endif

    if (!SemaphoreP_constructBinary(&(object->transferComplete), 0))
        while (1); // error...

    return handle;

}
PDEIRLPF3_Status PDEIRLPF3_transfer(PDEIRLPF3_Handle handle, PDEIRGENLPF3_Transaction *transaction)
{
    if (!handle)
        return PDEIRLPF3_STATUS_INVALID_PAR;

    uint32_t key = HwiP_disable();

#ifdef USE_ZIPIR
    // no clear way for ZIPIR UEI code to pass state to TI IR code, so just handle it in TI code
    if (handle->object && handle->object->state != PDEIRLPF3_STATE_IDLE)
    {
        HwiP_restore(key);
        return PDEIRLPF3_STATUS_BAD_STATE;
    }
    handle->object->state = PDEIRLPF3_STATE_ACTIVE;
#elif USE_IRLPGT
    if (handle->object->state != PDEIRLPF3_STATE_IDLE)
    {
        HwiP_restore(key);
        return PDEIRLPF3_STATUS_BAD_STATE;
    }
    handle->object->state = PDEIRLPF3_STATE_STARTING;
#endif
    HwiP_restore(key);

    // this is new transmission, so clear the object
    // alternatively the object may be cleared after the end of a transaction...
#ifdef USE_ZIPIR
    // ZIPIR handles data and timing
    gCurTrans = *transaction;
#else
    handle->object->bitIndex = 0;
    handle->object->countPassed = 0;
    gCurTrans = *transaction;
#endif

    // setup modulator

#ifdef USE_ZIPIR

    setupNextZipirFrame();

#elif USE_IRLPGT
    handle->object->state = PDEIRLPF3_STATE_LEAD_CODE;
    //handle->object->countPassed += CNT_PRE_CAL(NEC_lead.frameCount);
    handle->object->countPassed += CNT_PRE_CAL_FRAME(NEC_lead.frameCount);

    // Set TGT to the length of the symbol, assume lead frame for now
    HWREG(LGPT1_BASE + LGPT_O_TGT) &= ~LGPT_TGT_VAL_M;
    HWREG(LGPT1_BASE + LGPT_O_TGT) |= (NEC_lead.frameCount << LGPT_TGT_VAL_S);


    // Set C0CC to number of 38 kHz pulses in the symbol, assume lead frame for now
    HWREG(LGPT1_BASE + LGPT_O_C0CC) &= ~LGPT_C0CC_VAL_M;
    HWREG(LGPT1_BASE + LGPT_O_C0CC) |= (NEC_lead.activeCount << LGPT_C0CC_VAL_S);
#endif

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
    //HWREG(handle->hwAttrs->LgptModulatorBaseAddr) &= ~LGPT_CTL_MODE_M; // original, toby commented
    HWREG(handle->hwAttrs->LgptModulatorBaseAddr + LGPT_O_CTL) &= ~LGPT_CTL_MODE_M; // toby added
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
    SemaphoreP_destruct(&handle->object->transferComplete);

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
            (48MHz)/((PRECFG.TICKDIV+1) × (TGT+1)) = Wanted Frequency (for example, 38kHz)
            For example, PRECFG.TICKDIV = 2 and TGT 420. This gives a carrier wave of approximately 38kHz
            (Theoretically 38.00475kHz).
        • Set C0CC = 210 to give roughly 50% duty cycle.
        • Set C0CFG.CCACT = TGL_ON_CMP, and C0CFG.OUT0 = 1.
        • Set IRGEN.CTL = 1.
        • Start timer in up-periodic. CTL.MODE = 2
     */

    // live way to calculate frequency...
    // could be replaced later by some LUT to approximate
    const uint8_t div = 0; // for most cases (IR carrier freq <=732 Hz (based on math with 48MHz, 16-bit max value), div=0 is enough)
    uint16_t tgt = (480000000/(params->carrierFreq_Hz*(div+1))-5)/10; // not the fastest math here...
    //const uint16_t dutyHigh = tgt/3;
    const uint16_t dutyHigh = tgt * params->pwmDutyCycleHighCount /
                              (params->pwmDutyCycleHighCount + params->pwmDutyCycleLowCount);
#ifdef USE_ZIPIR
    uint32_t srcClkFreq = params->carrierFreq_Hz;
#elif USE_IRLPGT
    uint32_t srcClkFreq = params->carrierFreq_kHz * 1000;
#endif

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

        // Set C0CC to desired duty cycle.
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

#ifdef USE_ZIPIR
        // The TGT and C0CC will be set in _transfer, due to possibly different frequencies.
        // Frequencies can change based on which ZIPIR codebase.
#elif USE_IRLPGT
        // Set TGT to the length of the symbol, assume lead frame for now
        HWREG(LGPT1_BASE + LGPT_O_TGT) &= ~LGPT_TGT_VAL_M;
        HWREG(LGPT1_BASE + LGPT_O_TGT) |= (NEC_lead.frameCount << LGPT_TGT_VAL_S);

        // Set C0CC to number of 38 kHz pulses in the symbol, assume lead frame for now
        HWREG(LGPT1_BASE + LGPT_O_C0CC) &= ~LGPT_C0CC_VAL_M;
        HWREG(LGPT1_BASE + LGPT_O_C0CC) |= (NEC_lead.activeCount << LGPT_C0CC_VAL_S);
#endif
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

#ifdef USE_ZIPIR
static uint8_t setupNextZipirFrame()
{
#define MASK_BIT_14_0  (0x7FFF)
    // frameLen is length of mark + space
    // currently, at least for power on/off, each mark seems guaranteed to be paired with a space next
    const uint16_t frameLen = ( gCurTrans.pIrEncoding[gCurTrans.index]   +
                                gCurTrans.pIrEncoding[gCurTrans.index + 1]
                                 ) & MASK_BIT_14_0;

    // Set C0CC to number of 38 kHz pulses in the symbol, assume lead frame for now
    HWREG(LGPT1_BASE + LGPT_O_C0CC) &= ~LGPT_C0CC_VAL_M;
    HWREG(LGPT1_BASE + LGPT_O_C0CC) |= (gCurTrans.pIrEncoding[gCurTrans.index] & MASK_BIT_14_0)
                                          << LGPT_C0CC_VAL_S;

    // previous: Set TGT to the length of the symbol, assume lead frame for now
    // new: Set TGT to the length of the first symbol from ZIPIR encoding
    HWREG(LGPT1_BASE + LGPT_O_TGT) &= ~LGPT_TGT_VAL_M;
    HWREG(LGPT1_BASE + LGPT_O_TGT) |= (frameLen << LGPT_TGT_VAL_S) & LGPT_TGT_VAL_M;

    // done extracting info from first two data, check if any more and prepare if yes
    if (gCurTrans.index + 2  >  gCurTrans.numEncoding)
    {
        return 0; // done
    }
    else
    {
        gCurTrans.index += 2;
        return 1;
    }
}
static void PDEIRLPF3_modulator_callback()
{
    uint32_t lgptMis = HWREG(LGPT1_BASE + LGPT_O_MIS);

    if (lgptMis & LGPT_MIS_ZERO_SET)
    {
        uint32_t savedModeLgpt1 = HWREG(LGPT1_BASE + LGPT_O_CTL) & LGPT_CTL_MODE_M;
        HWREG(LGPT1_BASE + LGPT_O_CTL) &= ~LGPT_CTL_MODE_M;
        // decode the ZIPIR data and setup the next interrupt
        if (!setupNextZipirFrame())
        {
            PDEIRLPF3_Handle pdeIrHandle = (PDEIRLPF3_Handle) &PDEIRLPF3_configs[0];
            SemaphoreP_post(&pdeIrHandle->object->transferComplete);
            pdeIrHandle->object->state = PDEIRLPF3_STATE_IDLE;
            PDEIRLPF3_stop(gHandle);
        }
        else
        {
            // next frame setup, now re-enable interrupts
            // restore timer
            HWREG(LGPT1_BASE + LGPT_O_CTL) |= savedModeLgpt1;
        }
    }
}
#elif USE_IRLPGT
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
// end TI stuff 2025-03-14

static IR_TX_Driver_State irtxDrvState_src =
{
     .irtx_state = IR_TX_IDLE
};
IR_TX_Driver_State *irtxDrvState = &irtxDrvState_src;

// TODO: check if this is actually needed
void (*GENERIC_IRTX_POST_HOOK)(IR_TX_STATE state) = NULL;

void GENERIC_GPIO_CONFIGURE_PIN(uint32_t port, uint32_t pin, uint32_t dir, uint32_t pinInitState)
{
    // TODO: implement
}

void GENERIC_IRTX_SET_PORT_PIN(uint32_t port, uint32_t pin)
{
    // TODO: implement
}

void GENERIC_IRTX_INIT()
{
    // TODO: call TI Driver _init() funcs for IR
    // for now, we can use the same as PDEIR?
    PDEIRLPF3_init();
    //GENERIC_IRTX_POST_HOOK = updateSaveIrParams();

}

void GENERIC_IRTX_ABORT_CURRENT_TRANSACTION()
{
    // TODO: stop the IR TX
    // parameter checking already done in both _stop and _close
    PDEIRLPF3_stop(gHandle);

    //gHandle = 0;
}
bool GENERIC_IRTX_IS_AVAILABLE()
{
    return (gHandle != 0);
}
void GENERIC_IRTX_SEND_DATA(U16 *_irtx_data_ptr, U16 _irIndex, GENERIC_IR_TX_CLOCK_SETTING _ir_TxClkSetting)
{
    // TODO: implement

    if (!gHandle) while (1); // error...

    // prepare IR settings clock settings
    PDEIRLPF3_Params params;
    PDEIRLPF3_Params_init(&params);
    params.carrierFreq_Hz = _ir_TxClkSetting.modulateFreq;
    params.pwmDutyCycleHighCount = _ir_TxClkSetting.pwmDutyCycleHighCount;
    params.pwmDutyCycleLowCount  = _ir_TxClkSetting.pwmDutyCycleLowCount;
    initHw(gHandle, &params);

    // prepare transaction
    PDEIRGENLPF3_Transaction t;
    t.pIrEncoding = _irtx_data_ptr;
    t.numEncoding = _irIndex;
    t.index    = 0;

    PDEIRLPF3_transfer(gHandle, &t);

    // pend on semaphore
    SemaphoreP_pend(&gHandle->object->transferComplete, SemaphoreP_WAIT_FOREVER);

    // normally we should wait for key release before stopping any IR, but just stop here for easy test
    ZipIR_Stop(STOP_IR);
    //IrTxTimerCallback();
}
void GENERIC_INIT_APP_TIMER()
{
    // future_TODO: call TI Driver _init() funcs for timer

    /*
     Make sure platform timer routines are initialized.
     The timer routine must support callback.
     Is latency from FreeRTOS timer acceptable to ZIPIR?

     If latency is important, consider using LGPT directly...
     For CC2340R5 IRGEN, timers used: LGPT0 and LGPT1.
     There are two remaining timers, LGPT2 and LGPT3, but these may be used by the
     BLE stack.

     Based on looking into ZIPIR code, seems that this timer is only
     for the IR Macro TX (sequence of IR codes + duration).
     So maybe timing is more relaxed.

     Since we are not doing IR Macro right now, let's just leave
     these _APP_TIMER functions blank!
     */
}
int GENERIC_START_APP_TIMER(void (*_IrDurationCallback)(void), U32 _duration)
{
    // future_TODO: leave blank for now. see comment in GENERIC_INIT_APP_TIMER
}
void GENERIC_STOP_APP_TIMER(int _irDurationTimerID)
{
    // future_TODO: leave blank for now. see comment in GENERIC_INIT_APP_TIMER
}
void GENERIC_IR_BT_CLOCK_ENABLE(BOOL enable)
{
    // TODO: enable/disable the IR clock
    // from the UEI code, this is only enabled prior to sending the TX, so it's likely we just need to disable by closing the PDEIR immediately,
    // and enable by doing the full _open
    if (enable && !gHandle)
    {
        // enable
        PDEIRLPF3_Params params;
        PDEIRLPF3_Params_init(&params); // default should be ok, carrier and duty will be reconfigured by GENERIC_IRTX_SEND_DATA
        gHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &params);
        if (!gHandle) while (1); // error...
    }
    else
    {
        //disable
        PDEIRLPF3_stop(gHandle);
        PDEIRLPF3_close(gHandle);
        gHandle = 0;
    }
}

#endif
