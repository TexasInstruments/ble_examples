/**
  @file  main.c
  @brief main entry of the BLE stack sample application.

  <!--
  Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ``AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
  -->
*/

/*******************************************************************************
 * INCLUDES
 */

#include <xdc/runtime/Error.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/BIOS.h>

#include "icall.h"
#include "multi.h"
#include "multi_role.h"

/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

#ifndef USE_DEFAULT_USER_CFG

#include "ble_user_config.h"

// BLE user defined configuration
bleUserCfg_t user0Cfg = BLE_USER_CFG;

#endif // USE_DEFAULT_USER_CFG

#include <ti/mw/display/Display.h>

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
  RegisterAssertCback(AssertHandler);
  
  PIN_init(BoardGpioInitTable);

#ifndef POWER_SAVING
    /* Set constraints for Standby, powerdown and idle mode */
    Power_setConstraint(Power_SB_DISALLOW);
    Power_setConstraint(Power_IDLE_PD_DISALLOW);
#endif //POWER_SAVING

    /* Initialize ICall module */
    ICall_init();
  
    /* Start tasks of external images - Priority 5 */
    ICall_createRemoteTasks();
    
    /* Kick off profile - Priority 3 */
    GAPRole_createTask();
    
    /* Kick off application - Priority 1 */
    multi_role_createTask();
    
    /* enable interrupts and start SYS/BIOS */
    BIOS_start();
    
    return 0;
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
  // Open the display if the app has not already done so
  if ( !dispHandle )
  {
    dispHandle = Display_open(Display_Type_LCD, NULL);
  }

  Display_print0(dispHandle, 0, 0, ">>>STACK ASSERT");

  // check the assert cause
  switch (assertCause)
  {
    case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> OUT OF MEMORY!");
      break;

    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
      // check the subcause
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL FW ERROR!");
      }
      else
      {
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL ERROR!");
      }
      break;

    case HAL_ASSERT_CAUSE_ICALL_ABORT:
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> ICALL ABORT!");
      HAL_ASSERT_SPINLOCK;
      break;

    default:
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> DEFAULT SPINLOCK!");
      HAL_ASSERT_SPINLOCK;
  }

  return;
}


/*******************************************************************************
 * @fn          smallErrorHook
 *
 * @brief       Error handled to be hooked into TI-RTOS.
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
