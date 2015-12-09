/**
 *  @file  ICallCC2650.c
 *  @brief CC2650 specific ICall function implementation
 *
 *
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "ICallPlatform.h"
#include <ICallCC26xxDefs.h>

#include <stdint.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

#ifdef __IAR_SYSTEMS_ICC__
#include <intrinsics.h>
#endif

extern bool PowerCC26XX_isStableXOSC_HF(void);

/**
 * @internal Flag offset where dependencies starts.
 * Note that the value may change if ICallCC26xxDefs.h changes.
 * When the flags change, not only this module but the client code
 * also changes resulting into requirement to update both images.
 */
#define ICALLCC2650_PWR_CFG_D_OFFSET                   5

/**
 * @internal Invalid configuration flag mask.
 * Note that the value may change if ICallCC26xxDefs.h changes.
 * When the flags change, not only this module but the client code
 * also changes resulting into requirement to update both images.
 */
#define ICALLCC2650_PWR_CFG_NMASK                      0xff000000l

/** @internal power activity counter */
static uint_least8_t ICallPlatform_pwrActivityCount = 0;

/** @internal power constraint dependency bitmap */
typedef uint_least32_t ICallPlatform_pwrBitmap_t;

/** @internal power activity counter action configuration */
static ICallPlatform_pwrBitmap_t ICallPlatform_pwrCfgACAction =
  ( ICALL_PWR_C_SD_DISALLOW |
    ICALL_PWR_C_SB_DISALLOW );

/** @internal power notify handler */
int
ICallPlatform_pwrNotify(unsigned int eventType, uintptr_t eventArg, UArg clientArg)
{
  ICall_PwrNotifyData *data = (ICall_PwrNotifyData *) clientArg;
  ICall_PwrTransition transition;
  switch (eventType)
  {
  case PowerCC26XX_AWAKE_STANDBY:
    transition = ICALL_PWR_AWAKE_FROM_STANDBY;
    break;
  case PowerCC26XX_ENTERING_STANDBY:
    transition = ICALL_PWR_ENTER_STANDBY;
    break;
  case PowerCC26XX_ENTERING_SHUTDOWN:
    transition = ICALL_PWR_ENTER_SHUTDOWN;
    break;
  case PowerCC26XX_AWAKE_STANDBY_LATE:
    transition = ICALL_PWR_AWAKE_FROM_STANDBY_LATE;
    break;
  default:
    ICALL_HOOK_ABORT_FUNC();
    break;
  }
  data->_private(transition, data);
  return Power_NOTIFYDONE;
}

/**
 * @internal
 * Sets constraints and dependencies.
 *
 * @param bitmap  bitmap of constraint and dependency flags.
 */
static void ICallPlatform_pwrRequireImpl(uint_fast32_t bitmap)
{
  for (;;)
  {
#ifdef __IAR_SYSTEMS_ICC__
    char pos = __CLZ(bitmap);
#else
    int pos = __clz(bitmap);
#endif
    if (pos == 32)
      break;
    pos = 31 - pos;
    bitmap ^= 1ul << pos;
    if (pos < ICALLCC2650_PWR_CFG_D_OFFSET)
    {
      Power_setConstraint((unsigned int) (1<<pos));
    }
    else
    {
      Power_setDependency(pos - ICALLCC2650_PWR_CFG_D_OFFSET);
    }
  }
}

/**
 * @internal
 * Releases constraints and dependencies.
 *
 * @param bitmap  bitmap of constraint and dependency flags.
 */
static void ICallPlatform_pwrDispenseImpl(uint_fast32_t bitmap)
{
  for (;;)
  {
#ifdef __IAR_SYSTEMS_ICC__
    char pos = __CLZ(bitmap);
#else
    int pos = __clz(bitmap);
#endif
    if (pos == 32)
      break;
    pos = 31 - pos;
    bitmap ^= 1ul << pos;
    if (pos < ICALLCC2650_PWR_CFG_D_OFFSET)
    {
      Power_releaseConstraint((unsigned int) (1<<pos));
    }
    else
    {
      Power_releaseDependency(pos - ICALLCC2650_PWR_CFG_D_OFFSET);
    }
  }
}

/* See ICallPlatform.h for description */
ICall_Errno
ICallPlatform_pwrUpdActivityCounter(ICall_PwrUpdActivityCounterArgs *args)
{
  if (args->incFlag)
  {
    /* Increment counter */
    ICall_CSState state = ICall_enterCSImpl();
    if (ICallPlatform_pwrActivityCount == 255)
    {
      /* Counter value overflow */
      ICall_leaveCSImpl(state);
      return ICALL_ERRNO_OVERFLOW;
    }
    ICallPlatform_pwrActivityCount++;
    if (ICallPlatform_pwrActivityCount == 1)
    {
      /* Needs to stay in power active state.
       * Set constraints and dependencies as configured. */
      ICallPlatform_pwrRequireImpl(ICallPlatform_pwrCfgACAction);
    }
    ICall_leaveCSImpl(state);
  }
  else
  {
    /* Decrement counter */
    ICall_CSState state = ICall_enterCSImpl();
    if (ICallPlatform_pwrActivityCount == 0)
    {
      /* Counter value underflow */
      ICall_leaveCSImpl(state);
      return ICALL_ERRNO_UNDERFLOW;
    }
    ICallPlatform_pwrActivityCount--;
    if (ICallPlatform_pwrActivityCount == 0)
    {
      /* No activity.
       * Lift the constraints and dependencies as configured. */
      ICall_leaveCSImpl(state);
      ICallPlatform_pwrDispenseImpl(ICallPlatform_pwrCfgACAction);
    }
    else
    {
      ICall_leaveCSImpl(state);
    }
  }
  return ICALL_ERRNO_SUCCESS;
}

/* See ICallPlatform.h for description */
ICall_Errno
ICallPlatform_pwrConfigACAction(ICall_PwrBitmapArgs *args)
{
  ICall_CSState state;

  if (args->bitmap & ICALLCC2650_PWR_CFG_NMASK)
  {
    return ICALL_ERRNO_INVALID_PARAMETER;
  }

  state = ICall_enterCSImpl();

  if (ICallPlatform_pwrActivityCount > 0)
  {
    /* Release previous constraints */
    ICallPlatform_pwrDispenseImpl(ICallPlatform_pwrCfgACAction);
    /* Set new constraints */
    ICallPlatform_pwrRequireImpl(args->bitmap);
  }
  ICallPlatform_pwrCfgACAction = args->bitmap;

  ICall_leaveCSImpl(state);
  return ICALL_ERRNO_SUCCESS;
}

/* See ICallPlatform.h for description */
ICall_Errno
ICallPlatform_pwrRequire(ICall_PwrBitmapArgs *args)
{
  if (args->bitmap & ICALLCC2650_PWR_CFG_NMASK)
  {
    return ICALL_ERRNO_INVALID_PARAMETER;
  }

  ICallPlatform_pwrRequireImpl(args->bitmap);

  return ICALL_ERRNO_SUCCESS;
}

/* See ICallPlatform.h for description */
ICall_Errno
ICallPlatform_pwrDispense(ICall_PwrBitmapArgs *args)
{
  if (args->bitmap & ICALLCC2650_PWR_CFG_NMASK)
  {
    return ICALL_ERRNO_INVALID_PARAMETER;
  }

  ICallPlatform_pwrDispenseImpl(args->bitmap);

  return ICALL_ERRNO_SUCCESS;
}

/* See ICallPlatform.h for description */
ICall_Errno
ICallPlatform_pwrRegisterNotify(ICall_PwrRegisterNotifyArgs *args)
{
  Power_NotifyObj *obj[1];
  size_t i;

  static const unsigned int events[1] =
  {
    (unsigned int)((uint_least32_t) PowerCC26XX_ENTERING_STANDBY |
     (uint_least32_t) PowerCC26XX_ENTERING_SHUTDOWN |
     (uint_least32_t) PowerCC26XX_AWAKE_STANDBY |
     (uint_least32_t) PowerCC26XX_AWAKE_STANDBY_LATE)
  };

  args->obj->_private = args->fn;
  for (i = 0; i < sizeof(obj)/sizeof(obj[0]); i++)
  {
    obj[i] =  (Power_NotifyObj *) ICall_mallocImpl(sizeof(Power_NotifyObj));
    if (!obj[i])
    {
      size_t j;
      for (j = 0; j < i; j++)
      {
        ICall_freeImpl(obj[j]);
      }
      return ICALL_ERRNO_NO_RESOURCE;
    }
  }
  for (i = 0; i < sizeof(obj)/sizeof(obj[0]); i++)
  {
    Power_registerNotify(obj[i], events[i],
                         ICallPlatform_pwrNotify,
                         (UArg) args->obj);
  }
  return ICALL_ERRNO_SUCCESS;
}

/* See ICallPlatform.h for description */
ICall_Errno
ICallPlatform_pwrIsStableXOSCHF(ICall_GetBoolArgs* args)
{
  args->value = PowerCC26XX_isStableXOSC_HF();
  return ICALL_ERRNO_SUCCESS;
}

/* See ICallPlatform.h for description */
ICall_Errno
ICallPlatform_pwrSwitchXOSCHF(ICall_FuncArgsHdr* args)
{
  PowerCC26XX_switchXOSC_HF();
  return ICALL_ERRNO_SUCCESS;
}

/* See ICallPlatform.h for description */
ICall_Errno
ICallPlatform_pwrGetTransitionState(ICall_PwrGetTransitionStateArgs *args)
{
  args->state = Power_getTransitionState();
  return ICALL_ERRNO_SUCCESS;
}

/* See ICallPlatform.h for description */
ICall_Errno
ICallPlatform_pwrGetXOSCStartupTime(ICall_PwrGetXOSCStartupTimeArgs * args)
{
  args->value = PowerCC26XX_getXoscStartupTime(args->timeUntilWakeupInMs);
  return ICALL_ERRNO_SUCCESS;
}

#ifdef COVERAGE_TEST

static const struct _ICallPlatform_pwrConstraintMapEntry_t
{
  ICallPlatform_pwrBitmap_t icallid;
  Power_Constraint pwrid;
} ICallPlatform_pwrConstraintMap[] =
{
  { ICALL_PWR_C_SD_DISALLOW, Power_SD_DISALLOW },
  { ICALL_PWR_C_SB_DISALLOW, Power_SB_DISALLOW },
  { ICALL_PWR_C_IDLE_PD_DISALLOW, Power_IDLE_PD_DISALLOW },
  { ICALL_PWR_C_SB_VIMS_CACHE_RETAIN, Power_SB_VIMS_CACHE_RETAIN },
};

static const struct _ICallPlatform_pwrResourceMapEntry_t
{
  ICallPlatform_pwrBitmap_t icallid;
  Power_Resource pwrid;
} ICallPlatform_pwrResourceMap[] =
{
  { ICALL_PWR_D_PERIPH_GPT0, PERIPH_GPT0 },
  { ICALL_PWR_D_PERIPH_GPT1, PERIPH_GPT1 },
  { ICALL_PWR_D_PERIPH_GPT2, PERIPH_GPT2 },
  { ICALL_PWR_D_PERIPH_GPT3, PERIPH_GPT3 },
  { ICALL_PWR_D_PERIPH_SSI0, PERIPH_SSI0 },
  { ICALL_PWR_D_PERIPH_SSI1, PERIPH_SSI1 },
  { ICALL_PWR_D_PERIPH_UART0, PERIPH_UART0 },
  { ICALL_PWR_D_PERIPH_I2C0, PERIPH_I2C0 },
  { ICALL_PWR_D_PERIPH_TRNG, PERIPH_TRNG },
  { ICALL_PWR_D_PERIPH_GPIO, PERIPH_GPIO },
  { ICALL_PWR_D_PERIPH_UDMA, PERIPH_UDMA },
  { ICALL_PWR_D_PERIPH_CRYPTO, PERIPH_CRYPTO },
  { ICALL_PWR_D_PERIPH_I2S, PERIPH_I2S },
  { ICALL_PWR_D_PERIPH_RFCORE, PERIPH_RFCORE },
  { ICALL_PWR_D_XOSC_HF, XOSC_HF },
  { ICALL_PWR_D_DOMAIN_PERIPH, DOMAIN_PERIPH },
  { ICALL_PWR_D_DOMAIN_SERIAL, DOMAIN_SERIAL },
  { ICALL_PWR_D_DOMAIN_RFCORE, DOMAIN_RFCORE },
  { ICALL_PWR_D_DOMAIN_SYSBUS, DOMAIN_SYSBUS },
};

static const struct _ICallPlatform_pwrTransitionStateMapEntry_t
{
  uint_fast8_t icallid;
  Power_TransitionState pwrid;
} ICallPlatform_pwrTransitionStateMap[] =
{
  { ICALL_PWR_TRANSITION_STAY_IN_ACTIVE, Power_ACTIVE },
  { ICALL_PWR_TRANSITION_ENTERING_SLEEP, Power_ENTERING_SLEEP },
  { ICALL_PWR_TRANSITION_EXITING_SLEEP, Power_EXITING_SLEEP },
};

/**
 * @internal
 * Verification function for platform specific implementation.
 */
void ICallPlatform_verify(void)
{
  size_t i;
  for (i = 0; i < sizeof(ICallPlatform_pwrConstraintMap) /
                  sizeof(ICallPlatform_pwrConstraintMap[0]); i++)
  {
    if (ICallPlatform_pwrConstraintMap[i].icallid !=
        ICallPlatform_pwrConstraintMap[i].pwrid)
    {
      ICall_abort();
    }
    if (ICallPlatform_pwrConstraintMap[i].icallid &
        ICALLCC2650_PWR_CFG_NMASK)
    {
      ICall_abort();
    }
  }
  for (i = 0; i < sizeof(ICallPlatform_pwrResourceMap) /
                  sizeof(ICallPlatform_pwrResourceMap[0]); i++)
  {
    if (ICallPlatform_pwrResourceMap[i].icallid !=
        (1ull << (ICALLCC2650_PWR_CFG_D_OFFSET +
                 ICallPlatform_pwrResourceMap[i].pwrid)))
    {
      ICall_abort();
    }
    if (ICallPlatform_pwrResourceMap[i].icallid &
        ICALLCC2650_PWR_CFG_NMASK)
    {
      ICall_abort();
    }
  }
  for (i = 0; i < sizeof(ICallPlatform_pwrTransitionStateMap) /
                  sizeof(ICallPlatform_pwrTransitionStateMap[0]); i++)
  {
    if (ICallPlatform_pwrTransitionStateMap[i].icallid !=
        ICallPlatform_pwrTransitionStateMap[i].pwrid)
    {
      ICall_abort();
    }
  }
  if (ICall_pwrGetTransitionState() != ICALL_PWR_TRANSITION_STAY_IN_ACTIVE)
  {
    ICall_abort();
  }
}
#endif /* COVERAGE_TEST */
