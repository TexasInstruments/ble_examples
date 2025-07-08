/******************************************************************************

@file  LGPTTimerLPF3.c

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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/timer/LGPTimerLPF3.h>
#include <ti/devices/DeviceFamily.h>

#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(inc/hw_lgpt.h)

/* Define for interrupt mask */
#define INT_MASK                                                                                                    \
    (LGPTimerLPF3_INT_TGT | LGPTimerLPF3_INT_ZERO | LGPTimerLPF3_INT_COUNTER_CHANGE | LGPTimerLPF3_INT_DIR_CHANGE | \
     LGPTimerLPF3_INT_CH0_CC | LGPTimerLPF3_INT_CH1_CC | LGPTimerLPF3_INT_CH2_CC)

/* Field mask defines */
#define COUNTER_MASK_16_BIT 0x00FFFF
#define COUNTER_MASK_24_BIT 0xFFFFFF

/* LGPTimer configuration */
extern const LGPTimerLPF3_Config LGPTimerLPF3_config[];
extern const uint_least8_t LGPTimerLPF3_count;

/* Static functions */
static void LGPTimerLPF3_initHw(LGPTimerLPF3_Handle handle);
static void LGPTimerLPF3_resetHw(LGPTimerLPF3_Handle handle);
static void LGPTimerLPF3_initIO(LGPTimerLPF3_Handle handle);
static bool LGPTimerLPF3_isChannelActionCaptureType(LGPTimerLPF3_ChannelAction channelAction);
static void LGPTimerLPF3_resetIO(LGPTimerLPF3_Handle handle);
static void LGPTimerLPF3_configureDebugStall(LGPTimerLPF3_Handle handle, LGPTimerLPF3_DebugMode mode);
static void LGPTimerLPF3_configurePrescaler(LGPTimerLPF3_Handle handle, uint8_t prescalerDiv);
static void LGPTimerLPF3_configureChannels(LGPTimerLPF3_Handle handle);
static void LGPTimerLPF3HwiFxn(uintptr_t a0);
static int LGPTimerLPF3_postNotifyFxn(unsigned int eventType, uintptr_t eventArg, uintptr_t clientArg);

/* Default LGPTimer parameters */
static const LGPTimerLPF3_Params LGPTimerLPF3_defaultParams = {
    .hwiCallbackFxn      = NULL,
    .intPhaseLate        = true,
    .prescalerDiv        = 0,
    .debugStallMode      = LGPTimerLPF3_DEBUG_STALL_OFF,
    .counterDirChCompare = LGPTimerLPF3_CH_COMPARE_COUNTER_DIR_BOTH,
    .channelProperty[0] =
        {
            .action    = LGPTimerLPF3_CH_DISABLE,
            .inputEdge = LGPTimerLPF3_CH_EDGE_NONE,
        },
    .channelProperty[1] =
        {
            .action    = LGPTimerLPF3_CH_DISABLE,
            .inputEdge = LGPTimerLPF3_CH_EDGE_NONE,
        },
    .channelProperty[2] =
        {
            .action    = LGPTimerLPF3_CH_DISABLE,
            .inputEdge = LGPTimerLPF3_CH_EDGE_NONE,
        },
};

/*
 *  ======== LGPTimerLPF3_close ========
 */
void LGPTimerLPF3_close(LGPTimerLPF3_Handle handle)
{
    /* Get the pointer to the object and hwAttrs */
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;
    LGPTimerLPF3_Object *object         = handle->object;

    /* Stop and reset timer */
    LGPTimerLPF3_resetHw(handle);

    /* Reset any configured GPIO pins */
    LGPTimerLPF3_resetIO(handle);

    HwiP_destruct(&(object->hwi));

    /* Unregister power notification objects */
    Power_unregisterNotify(&object->postNotify);

    /* Release dependency for timer */
    Power_releaseDependency(hwAttrs->powerID);

    uint32_t key = HwiP_disable();

    /* Close Timer */
    object->isOpen = false;

    HwiP_restore(key);
}

/*
 *  ======== LGPTimerLPF3_open ========
 */
LGPTimerLPF3_Handle LGPTimerLPF3_open(uint_least8_t index, const LGPTimerLPF3_Params *params)
{
    LGPTimerLPF3_Handle handle = NULL;
    uintptr_t key;
    LGPTimerLPF3_Object *object;
    LGPTimerLPF3_HWAttrs const *hwAttrs;

    /* Check that requested index is valid */
    if (index < LGPTimerLPF3_count)
    {
        handle  = (LGPTimerLPF3_Handle)&LGPTimerLPF3_config[index];
        hwAttrs = handle->hwAttrs;
        object  = handle->object;
    }
    else
    {
        return NULL;
    }

    key = HwiP_disable();

    if (object->isOpen)
    {
        HwiP_restore(key);
        return NULL;
    }
    object->isOpen = true;

    HwiP_restore(key);

    /* Save param values used by LGPTimerLPF3_initHw() */
    object->intPhaseLate        = params->intPhaseLate;
    object->prescalerDiv        = params->prescalerDiv;
    object->debugStallMode      = params->debugStallMode;
    object->counterDirChCompare = params->counterDirChCompare;
    object->channelProperty[0]  = params->channelProperty[0];
    object->channelProperty[1]  = params->channelProperty[1];
    object->channelProperty[2]  = params->channelProperty[2];

    /* Store callback function */
    object->hwiCallbackFxn = params->hwiCallbackFxn;

    /* Register power dependency */
    Power_setDependency(hwAttrs->powerID);

    /* Configure selected GPIO pins */
    LGPTimerLPF3_initIO(handle);

    /* Initialize the LGPT hardware module */
    LGPTimerLPF3_initHw(handle);

    /* Register notification function */
    Power_registerNotify(&object->postNotify, PowerLPF3_AWAKE_STANDBY, LGPTimerLPF3_postNotifyFxn, (uintptr_t)handle);

    /* Construct RTOS HWI for this LGPT peripheral */
    HwiP_Params hwiParams;
    HwiP_Params_init(&hwiParams);
    hwiParams.arg       = (uintptr_t)handle;
    hwiParams.enableInt = true;
    hwiParams.priority  = hwAttrs->intPriority;
    HwiP_construct(&object->hwi, hwAttrs->intNum, LGPTimerLPF3HwiFxn, &hwiParams);

    /* LGPTimer opened successfully */
    return handle;
}

/*
 *  ======== LGPTimerLPF3_Params_init ========
 */
void LGPTimerLPF3_Params_init(LGPTimerLPF3_Params *params)
{
    *params = LGPTimerLPF3_defaultParams;
}

/*
 *  ======== LGPTimerLPF3_start ========
 */
void LGPTimerLPF3_start(LGPTimerLPF3_Handle handle, LGPTimerLPF3_Mode mode)
{
    uint32_t regVal;
    uintptr_t key;

    /* Get the pointer to the hwAttrs */
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Start timer counter in specified mode */
    key    = HwiP_disable();
    regVal = HWREG(hwAttrs->baseAddr + LGPT_O_CTL) & ~LGPT_CTL_MODE_M;
    regVal |= (mode & LGPT_CTL_MODE_M);
    HWREG(hwAttrs->baseAddr + LGPT_O_CTL) = regVal;
    HwiP_restore(key);

    /* Set constraint to disallow standby while running */
    Power_setConstraint(PowerLPF3_DISALLOW_STANDBY);
}

/*
 *  ======== LGPTimerLPF3_stop ========
 */
void LGPTimerLPF3_stop(LGPTimerLPF3_Handle handle)
{
    uintptr_t key;

    /* Get the pointer to the hwAttrs */
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Disable timer counter */
    key = HwiP_disable();
    HWREG(hwAttrs->baseAddr + LGPT_O_CTL) &= ~LGPT_CTL_MODE_M;
    HwiP_restore(key);

    /* Clear constraint to allow standby again */
    Power_releaseConstraint(PowerLPF3_DISALLOW_STANDBY);
}

/*
 *  ======== LGPTimerLPF3_setInitialCounterTarget ========
 */
void LGPTimerLPF3_setInitialCounterTarget(LGPTimerLPF3_Handle handle, uint32_t value, bool intFlagClr)
{
    /* Get the pointer to the hwAttrs */
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Set timer counter target value. Timer width (16 or 24 bits) depends on
     * the selected peripheral instance.
     */
    uint32_t targetReg = LGPT_O_TGT;
    uint32_t fieldMask = COUNTER_MASK_16_BIT;
    if (LGPTimerLPF3_getCounterWidth(handle) == 24)
    {
        fieldMask = COUNTER_MASK_24_BIT;
    }
    if (!intFlagClr)
    {
        targetReg = LGPT_O_TGTNC;
    }
    HWREG(hwAttrs->baseAddr + targetReg) = value & fieldMask;
}

/*
 *  ======== LGPTimerLPF3_setNextCounterTarget ========
 */
void LGPTimerLPF3_setNextCounterTarget(LGPTimerLPF3_Handle handle, uint32_t value, bool intFlagClr)
{
    /* Get the pointer to the hwAttrs */
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Set timer counter target value for next period. Timer width (16 or 24 bits)
     * depends on the selected peripheral instance.
     */
    uint32_t targetReg = LGPT_O_PTGT;
    uint32_t fieldMask = COUNTER_MASK_16_BIT;
    if (LGPTimerLPF3_getCounterWidth(handle) == 24)
    {
        fieldMask = COUNTER_MASK_24_BIT;
    }
    if (!intFlagClr)
    {
        targetReg = LGPT_O_PTGTNC;
    }
    HWREG(hwAttrs->baseAddr + targetReg) = value & fieldMask;
}

/*
 *  ======== LGPTimerLPF3_setInitialChannelCompVal ========
 */
void LGPTimerLPF3_setInitialChannelCompVal(LGPTimerLPF3_Handle handle,
                                           LGPTimerLPF3_ChannelNo chNo,
                                           uint32_t value,
                                           bool intFlagClr)
{
    /* Set channel number dependent register offset */
    uint32_t regOffset = sizeof(uint32_t) * chNo;

    /* Get the pointer to the hwAttrs */
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Set channel compare value. Compare value width (16 or 24 bits) depends on
     * the selected peripheral instance.
     */
    uint32_t regAddr   = LGPT_O_C0CC + regOffset;
    uint32_t fieldMask = COUNTER_MASK_16_BIT;
    if (LGPTimerLPF3_getCounterWidth(handle) == 24)
    {
        fieldMask = COUNTER_MASK_24_BIT;
    }
    if (!intFlagClr)
    {
        /* Use No clear variant of the C0CC register */
        regAddr = LGPT_O_C0CCNC + regOffset;
    }
    HWREG(hwAttrs->baseAddr + regAddr) = value & fieldMask;
}

/*
 *  ======== LGPTimerLPF3_setNextChannelCompVal ========
 */
void LGPTimerLPF3_setNextChannelCompVal(LGPTimerLPF3_Handle handle,
                                        LGPTimerLPF3_ChannelNo chNo,
                                        uint32_t value,
                                        bool intFlagClr)
{
    /* Set channel number dependent register offset */
    uint32_t regOffset = sizeof(uint32_t) * chNo;

    /* Get the pointer to the hwAttrs */
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Set channel compare value for next period. Compare value width (16 or 24 bits)
     * depends on the selected peripheral instance.
     */
    uint32_t regAddr   = LGPT_O_PC0CC + regOffset;
    uint32_t fieldMask = COUNTER_MASK_16_BIT;
    if (LGPTimerLPF3_getCounterWidth(handle) == 24)
    {
        fieldMask = COUNTER_MASK_24_BIT;
    }
    if (!intFlagClr)
    {
        /* Use No clear variant of the PC0CC register */
        regAddr = LGPT_O_PC0CCNC + regOffset;
    }
    HWREG(hwAttrs->baseAddr + regAddr) = value & fieldMask;
}

/*
 *  ======== LGPTimerLPF3_setChannelOutputLevel ========
 */
void LGPTimerLPF3_setChannelOutputLevel(LGPTimerLPF3_Handle handle,
                                        LGPTimerLPF3_ChannelNo chNo,
                                        LGPTimerLPF3_ChannelLevel level)
{
    /* Get the pointer to the hwAttrs */
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Manually override the current channel output level on the selected peripheral instance */
    uint32_t regVal                          = level << (chNo * 2);
    HWREG(hwAttrs->baseAddr + LGPT_O_OUTCTL) = regVal;
}

/*
 *  ======== LGPTimerLPF3_getChCompareVal ========
 */
uint32_t LGPTimerLPF3_getChCompareVal(LGPTimerLPF3_Handle handle, LGPTimerLPF3_ChannelNo chNo)
{
    /* Get the pointer to the hwAttrs */
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Get channel compare value. Timer width (16 or 24 bits) depends on
     * the selected peripheral instance.
     */
    uint32_t targetReg = LGPT_O_C0CCNC + (sizeof(uint32_t) * chNo);

    return (HWREG(hwAttrs->baseAddr + targetReg));
}

/*
 *  ======== LGPTimerLPF3_getChCaptureVal ========
 */
uint32_t LGPTimerLPF3_getChCaptureVal(LGPTimerLPF3_Handle handle, LGPTimerLPF3_ChannelNo chNo)
{
    uint32_t captureVal;

    captureVal = LGPTimerLPF3_getChCompareVal(handle, chNo);

    return captureVal;
}

/*
 *  ======== LGPTimerLPF3_getNextChCompareVal ========
 */
uint32_t LGPTimerLPF3_getNextChCompareVal(LGPTimerLPF3_Handle handle, LGPTimerLPF3_ChannelNo chNo)
{
    /* Get the pointer to the hwAttrs */
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Get channel compare value. Timer width (16 or 24 bits) depends on
     * the selected peripheral instance.
     */
    uint32_t targetReg = LGPT_O_PC0CCNC + (sizeof(uint32_t) * chNo);

    return (HWREG(hwAttrs->baseAddr + targetReg));
}

/*
 *  ======== LGPTimerLPF3_getNextChCaptureVal ========
 */
uint32_t LGPTimerLPF3_getNextChCaptureVal(LGPTimerLPF3_Handle handle, LGPTimerLPF3_ChannelNo chNo)
{
    uint32_t captureVal;

    captureVal = LGPTimerLPF3_getNextChCompareVal(handle, chNo);

    return captureVal;
}

/*
 *  ======== LGPTimerLPF3_getCounter ========
 */
uint32_t LGPTimerLPF3_getCounter(LGPTimerLPF3_Handle handle)
{
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;
    return HWREG(hwAttrs->baseAddr + LGPT_O_CNTR);
}

/*
 *  ======== LGPTimerLPF3_enableInterrupt ========
 */
void LGPTimerLPF3_enableInterrupt(LGPTimerLPF3_Handle handle, LGPTimerLPF3_IntMask intMask)
{
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Enable interrupts in timer unit */
    HWREG(hwAttrs->baseAddr + LGPT_O_IMSET) = intMask;
}

/*
 *  ======== LGPTimerLPF3_disableInterrupt ========
 */
void LGPTimerLPF3_disableInterrupt(LGPTimerLPF3_Handle handle, LGPTimerLPF3_IntMask intMask)
{
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Disable the specified interrupts */
    HWREG(hwAttrs->baseAddr + LGPT_O_IMCLR) = intMask;
}

/*
 *  ======== LGPTimerLPF3_setArg ========
 */
void LGPTimerLPF3_setArg(LGPTimerLPF3_Handle handle, void *arg)
{
    LGPTimerLPF3_Object *object = handle->object;
    object->arg                 = (uint32_t)arg;
}

/*
 *  ======== LGPTimerLPF3_getCounterWidth ========
 */
uint32_t LGPTimerLPF3_getCounterWidth(LGPTimerLPF3_Handle handle)
{
    /* Get the pointer to the hwAttrs */
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    uint32_t counterWidth = 16;
    if ((HWREG(hwAttrs->baseAddr + LGPT_O_DESCEX) & LGPT_DESCEX_CNTRW_M) == LGPT_DESCEX_CNTRW_CNTR24)
    {
        counterWidth = 24;
    }

    return counterWidth;
}

/*
 *  ======== LGPTimerLPF3_getArg ========
 */
uint32_t LGPTimerLPF3_getArg(LGPTimerLPF3_Handle handle)
{
    LGPTimerLPF3_Object *object = handle->object;

    return ((uint32_t)object->arg);
}

/*!
 *  @brief  Function that controls timer debug stall mode.
 *          When enabled, the timer counter will stop either immediatly or when the
 *          timer counter equals 0 dependent on the selected mode.
 *
 *  @param[in]  handle   A LGPTimerLPF3 handle returned from LGPTimerLPF3_open()
 *  @param[in]  mode     One of the defined debug stall modes
 *
 */
static void LGPTimerLPF3_configureDebugStall(LGPTimerLPF3_Handle handle, LGPTimerLPF3_DebugMode mode)
{
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    HWREG(hwAttrs->baseAddr + LGPT_O_EMU) = mode;
}

/*!
 *  @brief  Function that sets the timer prescaler division factor on the specified LGPTimer
 *          and selects prescaler source tick to be the system clock.
 *          The default prescaler division factor after chip reset is 0 which forces
 *          a division by 1 (prescaler not used).
 *          The prescaler outputs the timer counter clock. This clock equals system clock
 *          divided by the division factor pluss 1.
 *          Timer counter clock = system_clock / (prescalerDiv + 1)
 *          This function must be called by LGPTimerLPF3_initHw().
 *
 *  @param[in]  handle         A LGPTimerLPF3 handle returned from LGPTimerLPF3_open()
 *  @param[in]  prescalerDiv   The prescaler division factor. Must be a value in the range
 *                             of 0-255.
 *
 */
static void LGPTimerLPF3_configurePrescaler(LGPTimerLPF3_Handle handle, uint8_t prescalerDiv)
{
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Configure prescaler division and set prescaler tick source to system clock */
    HWREG(hwAttrs->baseAddr + LGPT_O_PRECFG) = (prescalerDiv) << LGPT_PRECFG_TICKDIV_S;
}

/*!
 *  @brief  Function that initializes the HW when LGPTimerLPF3_open() is called.
 *          Timer counter is disabled and channels are reset.
 *          Then the following is configured as specified by params that are stored in
 *          object:
 *          - Interrupt phase
 *          - Prescaler
 *          - Debug stall mode
 *          - Counter compare direction
 *          - Capture/compare channels
 *
 *  @param[in]  handle   A LGPTimerLPF3 handle returned from LGPTimerLPF3_open()
 *
 */
static void LGPTimerLPF3_initHw(LGPTimerLPF3_Handle handle)
{
    LGPTimerLPF3_Object *object         = handle->object;
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Disable timer counter, reset channels and configure both interrupt phase and
     * counter compare direction.
     */
    uint32_t regVal = LGPT_CTL_C2RST | LGPT_CTL_C1RST | LGPT_CTL_C0RST | LGPT_CTL_MODE_DIS |
                      object->counterDirChCompare;
    if (object->intPhaseLate)
    {
        regVal |= LGPT_CTL_INTP_LATE;
    }
    HWREG(hwAttrs->baseAddr + LGPT_O_CTL) = regVal;

    /* Configure the prescaler */
    LGPTimerLPF3_configurePrescaler(handle, object->prescalerDiv);

    /* Configure debug stall mode */
    LGPTimerLPF3_configureDebugStall(handle, object->debugStallMode);

    /* Configure capture/compare channels */
    LGPTimerLPF3_configureChannels(handle);
}

/*!
 *  @brief  Restore LGPTimer unit registers back to reset values.
 *          Needed since module does not have reset functionality.
 *
 * @param[in]  handle   A LGPTimerLPF3 handle returned from LGPTimerLPF3_open()
 *
 */
static void LGPTimerLPF3_resetHw(LGPTimerLPF3_Handle handle)
{
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint32_t base                       = hwAttrs->baseAddr;
    LGPTimerLPF3_IntMask intMask        = INT_MASK;

    /* Disable timer and reset channels */
    uint32_t regVal          = (LGPT_CTL_C2RST | LGPT_CTL_C1RST | LGPT_CTL_C0RST | LGPT_CTL_MODE_DIS);
    HWREG(base + LGPT_O_CTL) = regVal;

    /* Reset interrupt mask */
    LGPTimerLPF3_disableInterrupt(handle, intMask);

    /* Clear interrupt statuses */
    HWREG(base + LGPT_O_ICLR) = intMask;

    /* Reset timer counter register */
    HWREG(base + LGPT_O_CNTR) = 0x0;

    /* Reset timer counter target registers.
     * Supported counter widths are 16 bits and 24 bits.
     */
    uint32_t resetVal = COUNTER_MASK_16_BIT;
    if (LGPTimerLPF3_getCounterWidth(handle) == 24)
    {
        resetVal = COUNTER_MASK_24_BIT;
    }
    HWREG(base + LGPT_O_TGT)    = resetVal;
    HWREG(base + LGPT_O_TGTNC)  = resetVal;
    HWREG(base + LGPT_O_PTGT)   = 0x00;
    HWREG(base + LGPT_O_PTGTNC) = 0x00;

    /* Reset pre-scaler */
    HWREG(base + LGPT_O_PRECFG)   = 0;
    HWREG(base + LGPT_O_PREEVENT) = 0;

    /* Reset debug configuration */
    HWREG(base + LGPT_O_EMU) = 0;
}

/*!
 *  @brief  Configures the three capture/compare channels of the LGPTimer.
 *
 */
static void LGPTimerLPF3_configureChannels(LGPTimerLPF3_Handle handle)
{
    uint32_t regAddr;
    uint32_t regVal;

    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;
    LGPTimerLPF3_Object *object         = handle->object;

    /* Configure channel 0 if selected to be used */
    if (object->channelProperty[0].action != LGPTimerLPF3_CH_DISABLE)
    {
        regAddr = LGPT_O_C0CFG;
        regVal  = object->channelProperty[0].inputEdge;
        regVal |= object->channelProperty[0].action;
        regVal |= (LGPT_C0CFG_OUT0_EN);
        if (LGPTimerLPF3_isChannelActionCaptureType(object->channelProperty[0].action))
        {
            regVal |= LGPT_C0CFG_INPUT_IO;
        }
        HWREG(hwAttrs->baseAddr + regAddr) = regVal;
    }

    /* Configure channel 1 if selected to be used */
    if (object->channelProperty[1].action != LGPTimerLPF3_CH_DISABLE)
    {
        regAddr = LGPT_O_C1CFG;
        regVal  = object->channelProperty[1].inputEdge;
        regVal |= object->channelProperty[1].action;
        regVal |= (LGPT_C1CFG_OUT1_EN);
        if (LGPTimerLPF3_isChannelActionCaptureType(object->channelProperty[1].action))
        {
            regVal |= LGPT_C1CFG_INPUT_IO;
        }
        HWREG(hwAttrs->baseAddr + regAddr) = regVal;
    }

    /* Configure channel 2 if selected to be used */
    if (object->channelProperty[2].action != LGPTimerLPF3_CH_DISABLE)
    {
        regAddr = LGPT_O_C2CFG;
        regVal  = object->channelProperty[2].inputEdge;
        regVal |= object->channelProperty[2].action;
        regVal |= (LGPT_C2CFG_OUT2_EN);
        if (LGPTimerLPF3_isChannelActionCaptureType(object->channelProperty[2].action))
        {
            regVal |= LGPT_C2CFG_INPUT_IO;
        }
        HWREG(hwAttrs->baseAddr + regAddr) = regVal;
    }
}

/*!
 *  @brief  Initializes selected GPIO pins for the three capture/compare
 *          channels of the LGPTimer including their complementary channels.
 *          The complementary channels are outputs only.
 *
 */
static void LGPTimerLPF3_initIO(LGPTimerLPF3_Handle handle)
{
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;
    LGPTimerLPF3_Object *object         = handle->object;

    /* Loop over all available channels on the LGPT peripheral */
    for (uint32_t i = 0; i < NO_OF_LGPT_CHANNELS; i++)
    {
        /* Check if channel is selected for GPIO usage */
        if (hwAttrs->channelConfig[i].pin != GPIO_INVALID_INDEX)
        {
            /* Channel selected for GPIO pin usage. Configure dependent on channel action */
            if (LGPTimerLPF3_isChannelActionCaptureType(object->channelProperty[i].action))
            {
                /* Configure as input */
                GPIO_setConfigAndMux(hwAttrs->channelConfig[i].pin, GPIO_CFG_INPUT, hwAttrs->channelConfig[i].pinMux);
            }
            else
            {
                /* Configure as output */
                GPIO_setConfigAndMux(hwAttrs->channelConfig[i].pin, GPIO_CFG_NO_DIR, hwAttrs->channelConfig[i].pinMux);
            }
        }

        /* Check if complementary channel is selected for GPIO usage */
        if (hwAttrs->channelConfig[i].nPin != GPIO_INVALID_INDEX)
        {
            /* Complementary channel output selected for GPIO pin usage. Configure as output */
            GPIO_setConfigAndMux(hwAttrs->channelConfig[i].nPin, GPIO_CFG_NO_DIR, hwAttrs->channelConfig[i].nPinMux);
        }
    }
}

/*!
 *  @brief  Checks if specified channel action is a capture type action.
 *          Capture type channel action requires the channel to be configured
 *          as input.
 *
 */
static bool LGPTimerLPF3_isChannelActionCaptureType(LGPTimerLPF3_ChannelAction channelAction)
{
    bool captureType = false;

    if ((channelAction == LGPTimerLPF3_CH_SET_ON_CAPTURE_PERIODIC) ||
        (channelAction == LGPTimerLPF3_CH_SET_ON_CAPTURE_ONCE) ||
        (channelAction == LGPTimerLPF3_CH_PULSE_WIDTH_MEASURE))
    {
        captureType = true;
    }

    return captureType;
}

/*!
 *  @brief  Resets configured GPIO pins of the LGPTimer.
 *
 */
static void LGPTimerLPF3_resetIO(LGPTimerLPF3_Handle handle)
{
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Loop over all available channels on the LGPT peripheral */
    for (uint32_t i = 0; i < NO_OF_LGPT_CHANNELS; i++)
    {
        GPIO_resetConfig(hwAttrs->channelConfig[i].pin);
        GPIO_resetConfig(hwAttrs->channelConfig[i].nPin);
    }
}

/*!
 *  @brief  LGPTimer interrupt handler - Clears corresponding interrupt(s)
 *          and calls callback function with handle and bitmask argument.
 *
 */
static void LGPTimerLPF3HwiFxn(uintptr_t a0)
{
    LGPTimerLPF3_Handle handle          = (LGPTimerLPF3_Handle)a0;
    LGPTimerLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;
    LGPTimerLPF3_Object *object         = handle->object;

    LGPTimerLPF3_HwiFxn callbackFxn = object->hwiCallbackFxn;

    /* Get masked interrupt status */
    uint32_t intMask = HWREG(hwAttrs->baseAddr + LGPT_O_MIS);

    /* Clear interrupt status */
    HWREG(hwAttrs->baseAddr + LGPT_O_ICLR) = intMask;

    if (callbackFxn != NULL)
    {
        callbackFxn(handle, intMask);
    }
}

/*
 *  ======== LGPTimerLPF3_postNotifyFxn ========
 *  Called by Power module when waking up from standby.
 */
static int LGPTimerLPF3_postNotifyFxn(unsigned int eventType, uintptr_t eventArg, uintptr_t clientArg)
{
    /* Reconfigure the hardware if returning from sleep */
    if (eventType == PowerLPF3_AWAKE_STANDBY)
    {
        LGPTimerLPF3_initHw((LGPTimerLPF3_Handle)clientArg);
    }

    return Power_NOTIFYDONE;
}
