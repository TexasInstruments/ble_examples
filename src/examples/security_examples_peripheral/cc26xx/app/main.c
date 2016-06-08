/**
  @file  main.c
  @brief main entry of the BLE stack sample application.

 Copyright (c) 2013-2016, Texas Instruments Incorporated
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
*/
#include <xdc/runtime/Error.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/BIOS.h>

#include "icall.h"
#include "bcomdef.h"
#include "peripheral.h"
#include "security_examples_peripheral.h"

/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

#ifndef USE_DEFAULT_USER_CFG

#include "ble_user_config.h"

// BLE user defined configuration
bleUserCfg_t user0Cfg = BLE_USER_CFG;

#endif // USE_DEFAULT_USER_CFG

#if defined( TI_DRIVERS_DISPLAY_INCLUDED )
#include "board_display.h"
#endif // TI_DRIVERS_DISPLAY_INCLUDED

/*******************************************************************************
 * EXTERNS
 */

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*
 *  ======== main ========
 */
int main()
{
  /* Register Application callback to trap asserts raised in the Stack */
  RegisterAssertCback(AssertHandler);
  
  PIN_init(BoardGpioInitTable);

#ifndef POWER_SAVING
    /* Set constraints for Standby, powerdown and idle mode */
    Power_setConstraint(Power_SB_DISALLOW);
    Power_setConstraint(Power_IDLE_PD_DISALLOW);
#endif // POWER_SAVING
    
    /* Initialize ICall module */
    ICall_init();

    /* Start tasks of external images - Priority 5 */
    ICall_createRemoteTasks();
    
    /* Kick off profile - Priority 3 */
    GAPRole_createTask();
    
    security_examples_peripheral_createTask();

    /* enable interrupts and start SYS/BIOS */
    BIOS_start();
    
    return 0;
}

/**
 * Error handled to be hooked into TI-RTOS
 */
Void smallErrorHook(Error_Block *eb)
{
  for (;;);
}

/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the Application's callback handler for asserts raised
 *              in the stack.
 *
 * input parameters
 *
 * @param       assertCause    - Assert cause as defined in hal_assert.h.
 * @param       assertSubcause - Optional assert subcause (see hal_assert.h).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void AssertHandler(uint8 assertCause, uint8 assertSubcause)
{
#ifdef TI_DRIVERS_DISPLAY_INCLUDED
  Board_openDisplay(BOARD_DISPLAY_TYPE_LCD);
  Board_writeString(">>>STACK ASSERT", 0);
#endif // TI_DRIVERS_DISPLAY_INCLUDED

  // check the assert cause
  switch (assertCause)
  {
    case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
#ifdef TI_DRIVERS_DISPLAY_INCLUDED
      Board_writeString("***ERROR***", 1);
      Board_writeString(">> OUT OF MEMORY!", 2);
#endif // TI_DRIVERS_DISPLAY_INCLUDED
      break;

    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
      // check the subcause
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
#ifdef TI_DRIVERS_DISPLAY_INCLUDED
        Board_writeString("***ERROR***", 1);
        Board_writeString(">> INTERNAL FW ERROR!", 2);
#endif // TI_DRIVERS_DISPLAY_INCLUDED
      }
      else
      {
#ifdef TI_DRIVERS_DISPLAY_INCLUDED
        Board_writeString("***ERROR***", 1);
        Board_writeString(">> INTERNAL ERROR!", 2);
#endif // TI_DRIVERS_DISPLAY_INCLUDED
      }
      break;

    case HAL_ASSERT_CAUSE_ICALL_ABORT:
#ifdef TI_DRIVERS_DISPLAY_INCLUDED
        Board_writeString("***ERROR***", 1);
        Board_writeString(">> ICALL ABORT!", 2);
#endif // TI_DRIVERS_DISPLAY_INCLUDED
      HAL_ASSERT_SPINLOCK;
      break;

    default:
#ifdef TI_DRIVERS_DISPLAY_INCLUDED
        Board_writeString("***ERROR***", 1);
        Board_writeString(">> DEFAULT SPINLOCK!", 2);
#endif // TI_DRIVERS_DISPLAY_INCLUDED
      HAL_ASSERT_SPINLOCK;
  }

  return;
}
