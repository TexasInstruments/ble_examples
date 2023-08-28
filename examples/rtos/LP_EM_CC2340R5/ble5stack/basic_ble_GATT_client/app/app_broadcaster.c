/******************************************************************************

@file  app_broadcaster.c

@brief This example file demonstrates how to activate the broadcaster role with
the help of BLEAppUtil APIs.

BroadcasterAdvHandler structure is used for define advertising event handling
callback function and eventMask is used to specify the events that
will be received and handled.
In addition, fill the BLEAppUtil_AdvInit_t structure with variables generated
by the Sysconfig.

In the events handler functions, write what actions are done after each event.
In this example, A message will be printed after enabling/disabling
advertising.

In the Peripheral_start() function at the bottom of the file, registration,
initialization and activation are done using the BLEAppUtil API functions,
using the structures defined in the file.

More details on the functions and structures can be seen next to the usage.

Group: WCS, BTS
$Target Device: DEVICES $

******************************************************************************
$License: BSD3 2022 $
******************************************************************************
$Release Name: PACKAGE NAME $
$Release Date: PACKAGE RELEASE DATE $
*****************************************************************************/

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( BROADCASTER_CFG ) )

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti_ble_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>

//*****************************************************************************
//! Prototypes
//*****************************************************************************
void Broadcaster_AdvEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Globals
//*****************************************************************************


BLEAppUtil_EventHandler_t broadcasterAdvHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_ADV_TYPE,
    .pEventHandler  = Broadcaster_AdvEventHandler,
    .eventMask      = BLEAPPUTIL_ADV_START_AFTER_ENABLE |
                      BLEAPPUTIL_ADV_END_AFTER_DISABLE
};

//! Store handle needed for each advertise set
uint8 broadcasterAdvHandle_1;

const BLEAppUtil_AdvInit_t broadcasterInitAdvSet1 =
{
    /* Advertise data and length */
    .advDataLen        = sizeof(advData1),
    .advData           = advData1,

    /* Scan respond data and length */
    .scanRespDataLen   = 0,
    .scanRespData      = NULL,

    .advParam        = &advParams1
};

const BLEAppUtil_AdvStart_t broadcasterStartAdvSet1 =
{
  /* Use the maximum possible value. This is the spec-defined maximum for */
  /* directed advertising and infinite advertising for all other types */
  .enableOptions         = GAP_ADV_ENABLE_OPTIONS_USE_MAX,
  .durationOrMaxEvents   = 0
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Broadcaster_AdvEventHandler
 *
 * @brief   The purpose of this function is to handle advertise events
 *          that rise from the GAP and were registered in
 *          @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Broadcaster_AdvEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    BLEAppUtil_AdvEventData_t *advData = (BLEAppUtil_AdvEventData_t *)pMsgData;

    switch(event)
    {
        case BLEAPPUTIL_ADV_START_AFTER_ENABLE:
        {
            MenuModule_printf(APP_MENU_ADV_EVENT, 0, "Adv status: Started - handle: "
                              MENU_MODULE_COLOR_YELLOW "%d" MENU_MODULE_COLOR_RESET,
                              advData->pBuf->advHandle);
            break;
        }

        case BLEAPPUTIL_ADV_END_AFTER_DISABLE:
        {
            MenuModule_printf(APP_MENU_ADV_EVENT, 0, "Adv status: Ended - handle: "
                              MENU_MODULE_COLOR_YELLOW "%d" MENU_MODULE_COLOR_RESET,
                              advData->pBuf->advHandle);
            break;
        }

        default:
        {
            break;
        }
    }
}

/*********************************************************************
 * @fn      Broadcaster_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the broadcaster
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Broadcaster_start()
{
    bStatus_t status = SUCCESS;

    status = BLEAppUtil_registerEventHandler(&broadcasterAdvHandler);
    if(status != SUCCESS)
    {
        // Return status value
        return(status);
    }

    status = BLEAppUtil_initAdvSet(&broadcasterAdvHandle_1, &broadcasterInitAdvSet1);
    if(status != SUCCESS)
    {
        // Return status value
        return(status);
    }

    status = BLEAppUtil_advStart(broadcasterAdvHandle_1, &broadcasterStartAdvSet1);
    if(status != SUCCESS)
    {
        // Return status value
        return(status);
    }

    // Return status value
    return(status);
}

#endif // ( HOST_CONFIG & ( BROADCASTER_CFG ) )
