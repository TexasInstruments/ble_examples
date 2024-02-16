/******************************************************************************

 @file  main.c

 @brief main entry of the BLE stack sample application.

 Group: WCS, BTS
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2013 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

/*******************************************************************************
 * INCLUDES
 */

/* RTOS header files */
#include <FreeRTOS.h>
#include <stdint.h>
#include <task.h>
#ifdef __ICCARM__
    #include <DLib_Threads.h>
#endif
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC23X0.h>
#include <ti/display/Display.h>
#include <ti/drivers/UART2.h>
#include <ti/common/cc26xx/uartlog/UartLog.h>
#include <ti/devices/DeviceFamily.h>

#include <icall.h>
#include "hal_assert.h"
#include "bcomdef.h"

#ifndef USE_DEFAULT_USER_CFG
#include "ble_user_config.h"
#include "ble_stack_api.h"
// BLE user defined configuration
icall_userCfg_t user0Cfg = BLE_USER_CFG;
#endif // USE_DEFAULT_USER_CFG


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
extern void appMain(void);
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

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
  RegisterAssertCback(AssertHandler);

  Board_init();

  /* Update User Configuration of the stack */
  user0Cfg.appServiceInfo->timerTickPeriod = ICall_getTickPeriod();
  user0Cfg.appServiceInfo->timerMaxMillisecond  = ICall_getMaxMSecs();

  /* Initialize all applications tasks */
  appMain();

  /* Start the FreeRTOS scheduler */
  vTaskStartScheduler();

  return 0;

}

//*****************************************************************************
//
//! \brief Application defined stack overflow hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //Handle FreeRTOS Stack Overflow
    while(1)
    {
    }
}

/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the Application's callback handler for asserts raised
 *              in the stack.  When EXT_HAL_ASSERT is defined in the Stack Wrapper
 *              project this function will be called when an assert is raised,
 *              and can be used to observe or trap a violation from expected
 *              behavior.
 *
 *              As an example, for Heap allocation failures the Stack will raise
 *              HAL_ASSERT_CAUSE_OUT_OF_MEMORY as the assertCause and
 *              HAL_ASSERT_SUBCAUSE_NONE as the assertSubcause.  An application
 *              developer could trap any malloc failure on the stack by calling
 *              HAL_ASSERT_SPINLOCK under the matching case.
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
void AssertHandler(uint8_t assertCause, uint8_t assertSubcause)
{
    Log_error2(">>>STACK ASSERT Cause 0x%02x subCause 0x%02x",
               assertCause, assertSubcause);

    // check the assert cause
    switch(assertCause)
    {
    case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
        Log_error0("***ERROR***");
        Log_error0(">> OUT OF MEMORY!");
        break;

    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
        // check the subcause
        if(assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
        {
            Log_error0("***ERROR***");
            Log_error0(">> INTERNAL FW ERROR!");
        }
        else
        {
            Log_error0("***ERROR***");
            Log_error0(">> INTERNAL ERROR!");
        }
        break;

    case HAL_ASSERT_CAUSE_ICALL_ABORT:
        Log_error0("***ERROR***");
        Log_error0(">> ICALL ABORT!");
        //HAL_ASSERT_SPINLOCK;
        break;

    case HAL_ASSERT_CAUSE_ICALL_TIMEOUT:
        Log_error0("***ERROR***");
        Log_error0(">> ICALL TIMEOUT!");
        //HAL_ASSERT_SPINLOCK;
        break;

    case HAL_ASSERT_CAUSE_WRONG_API_CALL:
        Log_error0("***ERROR***");
        Log_error0(">> WRONG API CALL!");
        //HAL_ASSERT_SPINLOCK;
        break;

    default:
        Log_error0("***ERROR***");
        Log_error0(">> DEFAULT SPINLOCK!");
        //HAL_ASSERT_SPINLOCK;
    }

    return;
}




/*******************************************************************************
 */
