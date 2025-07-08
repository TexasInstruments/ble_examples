/*****************************************************************************
 ** COPYRIGHT 2020 UNIVERSAL ELECTRONICS INC (“UEI”). ALL RIGHTS RESERVED.
 **
 ** These materials (“Materials”) are the intellectual property of UEI, and are
 ** provided by UEI on an “As-Is” basis without warranties or representations.
 **
 ** The Materials are provided to the direct recipient of the Materials
 ** ("Recipient") and such Recipient's authorized partners and customers for
 ** internal development, evaluation and implementation purposes only
 ** ("Authorized Purpose").
 **
 ** The user of the Materials may not:
 ** (i) modify or create any derivative works of the Materials;
 ** (ii) decompile, disassemble, reverse engineer, or otherwise attempt to
 ** derive or extract any part of the Materials; or
 ** (iii) redistribute, sell, lease, sublicense, or otherwise transfer ownership
 ** of the Materials without the prior written consent of UEI.
 **
 ** By accessing and using the Materials, the user agrees to use the Materials
 ** solely for the Authorized Purpose only.
 ********************************************************************************
 * File : ZipIRBlaster.c
 * Revision : $Revision: $
 * Date : $Date: $
 * Updated by : $Author: $
 * Description :
 *
 ******************************************************************************/

#ifdef USE_ZIPIR

/* INCLUDE FILES */
#include "ZipIR_API.h"
#include "ZipIRDecoder.h"
#include "ZipIRBlaster.h"
/* PUBLIC FUNCTION DECLARATIONS */
void GENERIC_IR_BT_CLOCK_ENABLE(BOOL enable);
/* LOCAL VARIABLE DEFINITIONS */
#if TOBYDEBUG_202503_ENUMNAME
static U8 appir_pwm, appir_dutyCycle, irErrCount;
static U8 irState;
static U8 irState2;
#else
static U8 appir_pwm, appir_dutyCycle, irErrCount, irState, irState2;
#endif
// Duty cycle lookup table
static U8 dcTable[16] = { 0, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0 };
static U16 ir_index = 0, frameCounter;
static U16 irtx_data[MAX_IRTX_SIZE], irtx_data2[MAX_IRTX_SIZE];
static U16 *irtx_data_ptr;
static int irDurationTimerID;
static U32 zipir_carrier_freq;
static IrInput_t irInput;
static GENERIC_IR_TX_CLOCK_SETTING ir_TxClkSetting;
/* LOCAL FUNCTION DEFINITIONS */
static void IrInitialize(U8 irPin);
static void InitIRsetting(void);
#if TOBYDEV_20250318

#else
static void IrTxTimerCallback(void);
#endif
static void IrDurationCallback(void);
static void IrStopTimer(void);
// Function indices start at 0 for ZipIrBlastSignal()
static void ZipIrBlastSignal(U16 *buf, U16 *index, U16 function, U8 sequence,
                             U8 frameType);
static void IrConvertToWaveForm(U8 repSeqInd, U16 *buf, U16 *irIndex);
static void Irtx_processTx_HookFunc_post(IR_TX_STATE state);
/**
 *******************************************************************************
 * @fn ZipIR_Init
 *
 * @brief Initializes the appIR driver.
 *
 * @param[in] gpio - IRTX pin
 * @param[in] pwmChannel - PWM channel (see chip data sheet for valid values)
 *
 ******************************************************************************/
void ZipIR_Init(U8 gpio, U8 pwmChannel)
{
    GENERIC_IRTX_INIT();
    appir_pwm = pwmChannel;
    GENERIC_INIT_APP_TIMER();
    IrInitialize(gpio);
}
/**
 *******************************************************************************
 * @fn IrInitialize
 *
 * @brief Initializes IR timers, registers, global variables, and put
 * the IRTX into the idle (non-active) state
 *
 * @param[in] irPin - IR pin to be used as IR transmission (IRTX)
 *
 ******************************************************************************/
static void IrInitialize(U8 irPin)
{
    U8 port, pin;
    // Init IR
    GENERIC_IRTX_ABORT_CURRENT_TRANSACTION();
    GENERIC_IR_BT_CLOCK_ENABLE(FALSE);
    switch (irPin)
    {
// Assign hardware-specific port and pin values here
        // TODO(Toby): configure pin
    }
    GENERIC_GPIO_CONFIGURE_PIN(port, pin, GPIO_OUTPUT_ENABLE, 0);
    GENERIC_IRTX_SET_PORT_PIN(port, pin);
    // Init duration timer
    irDurationTimerID = GENERIC_INVALID_APP_TIMER_ID;
    IrStopTimer();
    irState = UEIRTX_IDLE;
    irState2 = 0;
    // Initialize all IR related variables and IRTX here...
    irErrCount = 0;
    ir_index = 0;
}
/**
 *******************************************************************************
 * @fn InitIRsetting
 *
 * @brief Initializes IR carrier generator and carrier frequency.
 *
 ******************************************************************************/
static void InitIRsetting(void)
{
    // Assign post hook
    if (GENERIC_IRTX_POST_HOOK == NULL)
    {
        GENERIC_IRTX_POST_HOOK = Irtx_processTx_HookFunc_post;
    }
    if (ZIPZcsdata.carrierPeriod == 0)
    {
        zipir_carrier_freq = 0;
    }
    else
    {
        zipir_carrier_freq = (100000000 + (U32)(ZIPZcsdata.carrierPeriod / 2))
                / (U32) ZIPZcsdata.carrierPeriod;
    }
    appir_dutyCycle = dcTable[ZIPZcsdata.dutyCycle];
    // Need to use 24MHz clock source - 1MHz cannot make 37.9kHz
    ir_TxClkSetting.clockSrcFreq = GENERIC_CLK_FREQ_24_MHZ;
    ir_TxClkSetting.invertOutput = FALSE;
    ir_TxClkSetting.modulateFreq = zipir_carrier_freq;
    // Assign legacy as default
    ir_TxClkSetting.clockSrc = GENERIC_CLK_SOURCE;
    ir_TxClkSetting.extendedSettings = 0;
    ir_TxClkSetting.pwmDutyCycleHighCount = 1;
    ir_TxClkSetting.pwmDutyCycleLowCount = 1;
    if (zipir_carrier_freq != 0)
    {
        U32 pwmCount, df, cf;
        ir_TxClkSetting.clockSrc = ACLK1;
        ir_TxClkSetting.extendedSettings = (appir_pwm
                & GENERIC_IRTX_REG_SETTINGS_MASK) | GENERIC_IRTX_REG_SETTINGS;
        // Modify carrier period
        pwmCount = 2;
        ir_TxClkSetting.pwmDutyCycleLowCount = appir_dutyCycle;
        pwmCount = 1 + appir_dutyCycle;
        df = 24000000 / (zipir_carrier_freq * pwmCount);
        cf = (24000000 / df) / pwmCount;
        ZIPZcsdata.carrierPeriod = (U32) 100000000 / cf;
    }
    irErrCount = 0;
}
BOOL ZipIR_Active(void)
{
    if (irState == UEIRTX_IDLE)
    {
        return FALSE;
    }
    return TRUE;
}
/**
 *******************************************************************************
 * @fn IrTxTimerCallback
 *
 * @brief IR transmission handler for make, repeat, and break frame.
 *
 ******************************************************************************/
#if TOBYDEV_20250318
void IrTxTimerCallback(void)
#else
static void IrTxTimerCallback(void)
#endif
{
    irState2 &= ~IS2_IRTX_ACTIVE;
    // Check IR transmission ready flag
    if (irState2 & IS2_READY_TO_XMIT)
    {
        if ((irState == UEIRTX_STARTED) || (irState == UEIRTX_REPEATING)
                || (irState == UEIRTX_END))
        {
            if (irtxDrvState->irtx_state == IR_TX_IDLE)
            {
                // Transmit IR from buffer
                if (!(irState2 & IS2_STOP_COMMAND)
                        || ((irState == UEIRTX_REPEATING)
                                && !(irState2 & IS2_REPEAT_FLAG))
                        || ((irState == UEIRTX_END)
                                && !(irState2 & IS2_REPEAT_FLAG)))
                {
                    GENERIC_IRTX_SEND_DATA(irtx_data_ptr, ir_index,
                                           ir_TxClkSetting);
                    // Flag to indicate this IR frame has been sent at least
                    // once and GENERIC_IRTX_SEND_DATA() is executed
                    irState2 |= (IS2_REPEAT_FLAG | IS2_IRTX_ACTIVE);
                }
                // Ping-pong data buffer if not repeating frame
                if ((irState != UEIRTX_REPEATING)
                        || (irState2 & IS2_STOP_COMMAND))
                {
                    // Ping-pong to the next IR data buffer
                    frameCounter++;
                    if (frameCounter & 0x01)
                    {
                        irtx_data_ptr = irtx_data2;
                    }
                    else
                    {
                        irtx_data_ptr = irtx_data;
                    }
                }
                irErrCount = 0;
                // Clear IR transmission ready flag
                irState2 &= ~IS2_READY_TO_XMIT;
            }
        }
    }
    if (irState == UEIRTX_STARTED)
    {
        ir_index = 0;
        if (ZIPZcsdata.wsp & 0x02)
        {
            irState = UEIRTX_REPEATING;
            irState2 &= ~IS2_REPEAT_FLAG;
            ZipIrBlastSignal(irtx_data_ptr, &ir_index, irInput.function,
                             irInput.keyCounter, IR_REPEAT_FRAME);
        }
        else if (ZIPZcsdata.wsp & 0x01)
        {
            irState = UEIRTX_END;
            irState2 &= ~IS2_REPEAT_FLAG;
            ZipIrBlastSignal(irtx_data_ptr, &ir_index, irInput.function,
                             irInput.keyCounter, IR_BREAK_FRAME);
        }
        else
        {
            irState = UEIRTX_END;
        }
    }
    else if (irState == UEIRTX_END)
    {
        if (!(irState2 & IS2_IRTX_ACTIVE))
        {
            irState = UEIRTX_IDLE;
        }
    }
    else if (irState == UEIRTX_REPEATING)
    {
        // Check key state at app layer
        if (irState2 & IS2_STOP_COMMAND)
        {
            ir_index = 0;
            if (ZIPZcsdata.wsp & 0x01)
            {
                irState2 &= ~IS2_REPEAT_FLAG;
                ZipIrBlastSignal(irtx_data_ptr, &ir_index, irInput.function,
                                 irInput.keyCounter, IR_BREAK_FRAME);
                irState = UEIRTX_END;
                if (!(irState2 & IS2_IRTX_ACTIVE))
                {
                    GENERIC_IRTX_SEND_DATA(irtx_data_ptr, ir_index,
                                           ir_TxClkSetting);
                    // Flag to indicate this IR frame has been sent at least
                    // once and GENERIC_IRTX_SEND_DATA() is executed
                    irState2 |= (IS2_REPEAT_FLAG | IS2_IRTX_ACTIVE);
                }
            }
            else
            {
                // Check whether GENERIC_IRTX_SEND_DATA() is active
                if (irState2 & IS2_IRTX_ACTIVE)
                {
                    irState = UEIRTX_END;
                }
                else
                {
                    irState = UEIRTX_IDLE;
                }
            }
        }
        else
        {
            // Stays repeating, ir_data and ir_index initialized already
        }
    }
    if ((irState == UEIRTX_REPEATING) || (irState == UEIRTX_END))
    {
        // Check for data in the buffer
        if (ir_index)
        {
            // Set transmission ready and wait-for-idle flags
            irState2 |= (IS2_WAIT_FOR_IDLE | IS2_READY_TO_XMIT);
        }
    }
    else if (irState == UEIRTX_IDLE)
    {
        // Reset irState2 flags (Note: Do not change the order of this instruction
        // or else an extra pulse may be sent after key is released)
        irState2 &= ~(IS2_WAIT_FOR_IDLE | IS2_READY_TO_XMIT | IS2_STOP_COMMAND
                | IS2_REPEAT_FLAG | IS2_IRTX_ACTIVE);
        GENERIC_IRTX_ABORT_CURRENT_TRANSACTION();
        GENERIC_IR_BT_CLOCK_ENABLE(FALSE);
        // Stop IR duration timer
        IrStopTimer();
        // Call back if defined by caller
        if (irInput.irCallback != NULL)
        {
            irInput.irCallback();
        }
    }
}
/**
 *******************************************************************************
 * @fn ZipIR_Transmit
 *
 * @brief Transmits IR for the specified function and code set.
 *
 * @param[in] ir_input.keyFlag - IR transmission type
 * @param[in] ir_input.keyCounter - used for toggle functions
 * @param[in] ir_input.function - index from the function key map table
 * @param[in] ir_input.codesetId - code set ID Pointer
 * @param[in] ir_input.irCallback - call-back function when IR TX is done
 * @param[in] ir_input.duration - transmission duration in ms units
 *
 * @return 0x00 = Ready for next IR transmission
 * other = Busy transmitting IR
 *
 ******************************************************************************/
U8 ZipIR_Transmit(IrInput_t *ir_input)
{
    if (ir_input == NULL)
    {
        return irState;
    }
    // Added a safety check here to prevent hang up
    if (!GENERIC_IRTX_IS_AVAILABLE())
    {
        GENERIC_IRTX_ABORT_CURRENT_TRANSACTION();
        GENERIC_IR_BT_CLOCK_ENABLE(FALSE);
    }
    // Exit if IR is not ready
    if (irState != UEIRTX_IDLE)
    {
        return irState;
    }
// Check for valid function
    if (!ZipIR_CheckFunction(ir_input->codesetId, ir_input->function))
    {
// Function not found - exit driver
        return irState;
    }
    // Save input parameters to a static buffer
    memcpy(&irInput, ir_input, sizeof(IrInput_t));
    // Init bit buffer
    InitBitBuffer((U8*) irInput.codesetId, 0, 0);
    if (DecodeZSFData() == FALSE)
    {
        // Init bit buffer again because DecodeZSFData() adjusted internal variables
        InitBitBuffer((U8*) irInput.codesetId, 0, 0);
        if (DecodeZCLData(0, 0) == FALSE) // (0, 0) = do not parse codeset number
        {
            return irState;
        }
    }
    InitIRsetting();
    GetWSFunction(irInput.function, irInput.keyCounter);
    ir_index = 0;
    irtx_data_ptr = irtx_data;
    frameCounter = 0;
    if (ZIPZcsdata.wsp & 0x04)
    {
        ZipIrBlastSignal(irtx_data_ptr, &ir_index, irInput.function,
                         irInput.keyCounter, IR_MAKE_FRAME);
    }
    else if (ZIPZcsdata.wsp & 0x02)
    {
        ZipIrBlastSignal(irtx_data_ptr, &ir_index, irInput.function,
                         irInput.keyCounter, IR_REPEAT_FRAME);
    }
    else
    {
        return irState;
    }
    // Treat max limit as an error
    if (ir_index >= MAX_IRTX_SIZE - 1)
    {
        return irState;
    }
    if (ir_index)
    {
        GENERIC_IR_BT_CLOCK_ENABLE(TRUE);
        IrStopTimer();
        if (ZIPZcsdata.wsp & 0x04)
        {
            irState = UEIRTX_STARTED;
        }
        else
        {
            irState = UEIRTX_REPEATING;
        }
        // Flag to transmit IR right away
        irState2 = IS2_READY_TO_XMIT;
        // Check for transmission duration
        if ((irInput.keyFlag & KF_XMIT_TYPE) == MACRO_XMIT)
        {
            // Check for default duration
            if (irInput.duration == 0)
            {
                ZipIR_StartTimer(MACRO_IR_DURATION_INTERVAL);
            }
            else
            {
                // Allow maximum of 10 minutes IR transmission to prevent battery
                // drain
                if (irInput.duration > MAX_IR_DURATION_INTERVAL)
                {
                    irInput.duration = MAX_IR_DURATION_INTERVAL;
                }
                ZipIR_StartTimer(irInput.duration);
            }
        }
        // Start IR transmission
        IrTxTimerCallback();
    }
    return irState;
}
/**
 *******************************************************************************
 * @fn ZipIR_Stop
 *
 * @brief Terminates ongoing IR transmission.
 *
 * @param[in] stopCmd - stop command
 *
 ******************************************************************************/
void ZipIR_Stop(U8 stopCmd)
{
    if (stopCmd == SUSPEND_IR)
    {
        // Suspend IR transmission here
    }
    irState2 |= IS2_STOP_COMMAND;
}
/**
 *******************************************************************************
 * @fn IrDurationCallback
 *
 * @brief Callback routine for IR duration timer. This routine issues a
 * stop command to terminate IR when the IR timer duration has expired.
 *
 ******************************************************************************/
static void IrDurationCallback(void)
{
    ZipIR_Stop(STOP_IR);
    IrStopTimer();
}
/**
 *******************************************************************************
 * @fn ZipIR_StartTimer
 *
 * @brief Starts IR duration timer.
 *
 * @param[in] duration - IR duration in ms
 *
 ******************************************************************************/
void ZipIR_StartTimer(U32 duration)
{
    irDurationTimerID = GENERIC_START_APP_TIMER(IrDurationCallback, duration);
}
/**
 *******************************************************************************
 * @fn IrStopTimer
 *
 * @brief Stops IR duration timer.
 *
 ******************************************************************************/
static void IrStopTimer(void)
{
    GENERIC_STOP_APP_TIMER(irDurationTimerID);
}
/**
 *******************************************************************************
 * @fn ZipIrBlastSignal
 *
 * @brief Sets up the data timing (waveform) buffer for the specified
 * function.
 *
 * @param[in] buf - buffer where waveform will be stored
 * @param[in] index - pointer to buffer length variable
 * @param[in] function - index of function
 * @param[in] sequence - toggle counter
 * @param[in] frameType - type of frame (make, repeat, or break)
 *
 ******************************************************************************/
static void ZipIrBlastSignal(U16 *buf, U16 *index, U16 function, U8 sequence,
                             U8 frameType)
{
    switch (frameType)
    {
    case IR_MAKE_FRAME:
        if (ZIPZcsdata.wsp & 0x04)
        {
            IrConvertToWaveForm(ZIPZcsdata.wsIndex[0], buf, index);
        }
        break;
    case IR_REPEAT_FRAME:
        if (ZIPZcsdata.wsp & 0x02)
        {
            IrConvertToWaveForm(ZIPZcsdata.wsIndex[1], buf, index);
        }
        break;
    case IR_BREAK_FRAME:
        if (ZIPZcsdata.wsp & 0x01)
        {
            IrConvertToWaveForm(ZIPZcsdata.wsIndex[2], buf, index);
        }
        break;
    default:
        break;
    }
}
/**
 *******************************************************************************
 * @fn IrConvertToWaveForm
 *
 * @brief Gets the waveform data timings for the specified waveform
 * substitution.
 *
 * @param[in] repSeqInd - repeat sequence index (i.e., waveform substitution)
 * @param[in] buf - buffer where data will be stored
 * @param[in] irIndex - pointer to buffer length variable
 *
 ******************************************************************************/
static void IrConvertToWaveForm(U8 repSeqInd, U16 *buf, U16 *irIndex)
{
    U16 irSignal;
    int i, j, len, count, length, number, index = 1, timingIndex;
    count = ZIPZcsdata.bwWaveformSubs;
    LocateWS(repSeqInd);
    for (j = 0; j < ZIPZcsdata.waveformSubs[repSeqInd]; j++)
    {
        len = GetBits(count);
        length = ZIPZcsdata.repSeqSize[len];
        number = 0;
        while (number < length)
        {
            timingIndex = ZIPZcsdata.repSeq[len][number];
            if (index & 0x01)
            {
                irSignal = APPIR_HIGH;
            }
            else
            {
                irSignal = APPIR_LOW;
            }
            // Carrier mode
            if (ZIPZcsdata.carrierPeriod)
            {
                U32 nCarrierPulses = ZIPZcsdata.timTbl[timingIndex];
                if (timingIndex < ZIPZcsdata.longTime)
                {
                    nCarrierPulses += ZIPZcsdata.timLongTbl[timingIndex]
                            * 0x7FFF;
                }
                while (nCarrierPulses > 0x7FFF)
                {
                    buf[*irIndex] = irSignal + 0x7FFE;
                    (*irIndex)++;
                    nCarrierPulses -= 0x7FFF;
                }
                if (nCarrierPulses)
                {
                    buf[*irIndex] = irSignal + nCarrierPulses - 1;
                    (*irIndex)++;
                }
#ifndef SKIP_0_TIMINGS
                else
                {
                    // We need at least 2 pulses to prevent the code from
// hanging up, so we add one
                    buf[*irIndex] = irSignal + 1;
                    (*irIndex)++;
                }
#endif
                if (*irIndex >= MAX_IRTX_SIZE)
                {
                    *irIndex = MAX_IRTX_SIZE - 1;
                }
            }
            else // Timer mode
            {
                if (timingIndex < ZIPZcsdata.longTime)
                {
                    i = ZIPZcsdata.timLongTbl[timingIndex];
                    while (i--)
                    {
                        buf[*irIndex] = irSignal + 0x7FFE;
                        (*irIndex)++;
                    }
                }
                if (ZIPZcsdata.timTbl[timingIndex])
                {
                    buf[*irIndex] = irSignal + ZIPZcsdata.timTbl[timingIndex]
                            - 1;
                    (*irIndex)++;
                    if (*irIndex >= MAX_IRTX_SIZE)
                    {
                        *irIndex = MAX_IRTX_SIZE - 1;
                    }
                }
#ifndef SKIP_0_TIMINGS
                else
                {
                    // We need at least 2 pulses to prevent the code from
                    // hanging up, so we add one
                    buf[*irIndex] = irSignal + 1;
                }
#endif
            }
            index++;
            number++;
        } // while loop
    } // for loop
      // If the last burst is mark only, add a dummy space to prevent consecutive mark
      // hangup
    if (!(index & 0x01))
    {
        buf[*irIndex] = 10;
        (*irIndex)++;
        index++;
    }
#ifdef CALLBACK_DELAY
  // Do not adjust the timing if the last space burst does not exist
  if (index & 0x01)
  {
  // Adjust the last space burst timing to compensate for the time it takes to
  // call GENERIC_IRTX_SEND_DATA()
  if (ZIPZcsdata.carrierPeriod)
  {
  number = ( ((U32)ADJUST_TIME)*100 + ZIPZcsdata.carrierPeriod/2 ) /
  ZIPZcsdata.carrierPeriod;
  }
  else
  {
   number = ADJUST_TIME;
   }
   // Make the last pulse space shorter to improve the code gap error
   (*irIndex)--;
   if (buf[*irIndex] > number)
   {
   buf[*irIndex] -= number;
   }
   (*irIndex)++;
   }
  #endif
}
/**
 *******************************************************************************
 * @fn Irtx_processTx_HookFunc_post
 *
 * @brief
 *
 * @param[in] state - IR transmission state
 *
 ******************************************************************************/
static void Irtx_processTx_HookFunc_post(IR_TX_STATE state)
{
    switch (state)
    {
    case IR_TX_IDLE:
        IrTxTimerCallback();
        break;
    case IR_TX_PAYLOAD:
        break;
    case IR_TX_BUSY:
        break;
    case IR_TX_DONE:
    default:
        break;
    }
}

#endif
