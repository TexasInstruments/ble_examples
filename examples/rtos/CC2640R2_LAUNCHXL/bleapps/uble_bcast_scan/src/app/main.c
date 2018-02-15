/******************************************************************************

 @file       main.c

 @brief main entry function for micro stack application

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************

 Copyright (c) 2013-2017, Texas Instruments Incorporated
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
 Release Name: simplelink_cc2640r2_sdk_01_50_00_58_s
 Release Date: 2017-10-17 18:50:56
 *****************************************************************************/

/******************************************************************************
*
 * INCLUDES
 */

#include <xdc/runtime/Error.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/display/Display.h>

#include <board.h>

#include <hal_assert.h>
#include <bcomdef.h>

#include "uble_bcast_scan.h"

/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * EXTERNS
 */
extern assertCback_t halAssertCback;

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

extern Display_Handle dispHandle;

/*******************************************************************************
 * @fn          Main
 *
 * @brief       Application Main
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
int main()
{
  /* Register Application callback to trap asserts raised in the Stack */
  halAssertCback = AssertHandler;

  PIN_init(BoardGpioInitTable);

  /* Enable iCache prefetching */
  VIMSConfigure(VIMS_BASE, TRUE, TRUE);

  /* Enable cache */
  VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);

#ifndef POWER_SAVING
  /* Set constraints for Standby, powerdown and idle mode */
  /* PowerCC26XX_SB_DISALLOW may be redundant */
  Power_setConstraint(PowerCC26XX_SB_DISALLOW);
  Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
#endif /* POWER_SAVING */

  /* Create Application task. */
  UBLEBcastScan_createTask();

  BIOS_start();     /* enable interrupts and start SYS/BIOS */

  return 0;
}

/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the callback handler for asserts raised in the stack or
 *              the application.
 *
 *              An application developer is encouraged to extend this function
 *              for use by their own application.  To do this, add hal_assert.c
 *              to your project workspace, the path to hal_assert.h (this can
 *              be found on the stack side). Asserts are raised by including
 *              hal_assert.h and using macro HAL_ASSERT(cause) to raise an
 *              assert with argument assertCause.  the assertSubcause may be
 *              optionally set by macro HAL_ASSERT_SET_SUBCAUSE(subCause) prior
 *              to asserting the cause it describes. More information is
 *              available in hal_assert.h.
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
#if !defined(Display_DISABLE_ALL)
  // Open the display if the app has not already done so
  if ( !dispHandle )
  {
#if !defined(Display_DISABLE_ALL)
  #if defined(BOARD_DISPLAY_USE_LCD)
    dispHandle = Display_open(Display_Type_LCD, NULL);
  #elif defined (BOARD_DISPLAY_USE_UART)
    dispHandle = Display_open(Display_Type_UART, NULL);
  #endif // BOARD_DISPLAY_USE_LCD, BOARD_DISPLAY_USE_UART
#endif // Display_DISABLE_ALL
  }

  Display_print0(dispHandle, 0, 0, ">>> ASSERT");
#endif // ! Display_DISABLE_ALL

  // check the assert cause
  switch (assertCause)
  {
    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
      // check the subcause
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
      #if !defined(Display_DISABLE_ALL)
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL FW ERROR!");
      #endif
      }
      else
      {
      #if !defined(Display_DISABLE_ALL)
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL ERROR!");
      #endif
      }

      break;

    default:
    #if !defined(Display_DISABLE_ALL)
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> DEFAULT SPINLOCK!");
    #endif
      HAL_ASSERT_SPINLOCK;
  }

  return;
}


/*******************************************************************************
 * @fn          smallErrorHook
 *
 * @brief       Error handler to be hooked into TI-RTOS.
 *
 * input parameters
 *
 * @param       eb - Pointer to Error Block.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void smallErrorHook(Error_Block *eb)
{
  for (;;);
}


/*******************************************************************************
 */
