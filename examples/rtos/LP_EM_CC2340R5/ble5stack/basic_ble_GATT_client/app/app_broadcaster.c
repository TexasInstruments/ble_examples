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
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2023, Texas Instruments Incorporated
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
    switch(event)
    {
        case BLEAPPUTIL_ADV_START_AFTER_ENABLE:
        {
            MenuModule_printf(APP_MENU_ADV_EVENT, 0, "Adv status: Started - handle: "
                              MENU_MODULE_COLOR_YELLOW "%d" MENU_MODULE_COLOR_RESET,
                              ((BLEAppUtil_AdvEventData_t *)pMsgData)->pBuf->advHandle);
            break;
        }

        case BLEAPPUTIL_ADV_END_AFTER_DISABLE:
        {
            MenuModule_printf(APP_MENU_ADV_EVENT, 0, "Adv status: Ended - handle: "
                              MENU_MODULE_COLOR_YELLOW "%d" MENU_MODULE_COLOR_RESET,
                              ((BLEAppUtil_AdvEventData_t *)pMsgData)->pBuf->advHandle);
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
