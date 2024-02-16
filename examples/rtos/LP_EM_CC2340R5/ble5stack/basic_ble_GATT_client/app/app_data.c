/******************************************************************************

@file  app_data.c

@brief This file contains the application data functionality

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

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
//*****************************************************************************
extern uint8_t charVal;
static void GATT_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

// Events handlers struct, contains the handlers and event masks
// of the application data module
BLEAppUtil_EventHandler_t dataGATTHandler =
{
    .handlerType    = BLEAPPUTIL_GATT_TYPE,
    .pEventHandler  = GATT_EventHandler,
    .eventMask      = BLEAPPUTIL_ATT_FLOW_CTRL_VIOLATED_EVENT |
                      BLEAPPUTIL_ATT_MTU_UPDATED_EVENT |
                      BLEAPPUTIL_ATT_READ_RSP |
                      BLEAPPUTIL_ATT_WRITE_RSP |
                      BLEAPPUTIL_ATT_EXCHANGE_MTU_RSP|
                      BLEAPPUTIL_ATT_ERROR_RSP |
                      BLEAPPUTIL_ATT_HANDLE_VALUE_NOTI
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      GATT_EventHandler
 *
 * @brief   The purpose of this function is to handle GATT events
 *          that rise from the GATT and were registered in
 *          @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
static void GATT_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  gattMsgEvent_t *gattMsg = ( gattMsgEvent_t * )pMsgData;
  switch ( gattMsg->method )
  {
    case ATT_FLOW_CTRL_VIOLATED_EVENT:
      {
          MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "GATT status: ATT flow control is violated");
      }
      break;

    case ATT_MTU_UPDATED_EVENT:
      {
          MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "GATT status: ATT MTU update to %d",
                            gattMsg->msg.mtuEvt.MTU);
      }
      break;
    case ATT_READ_RSP:
        {

            MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Read rsp = 0x%02x",
                              gattMsg->msg.readRsp.pValue[0]);

             break;
        }
    case ATT_WRITE_RSP:
        {
            MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Write sent = 0x%02x",
                              charVal);
                              //gattMsg->msg.writeReq.pValue[0]);
             break;
        }
    case ATT_EXCHANGE_MTU_RSP:
        {
            MenuModule_printf(APP_MENU_CONN_EVENT, 0, "MTU max size client = %d MTU max size server = %d",
                              gattMsg->msg.exchangeMTUReq.clientRxMTU, gattMsg->msg.exchangeMTURsp.serverRxMTU);
            break;
        }

    case   ATT_HANDLE_VALUE_NOTI:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "Notification received = 0x%02x",
                                          gattMsg->msg.handleValueNoti.pValue[0]);
            break;
        }
    case ATT_ERROR_RSP:
        {
            attErrorRsp_t  *pReq = (attErrorRsp_t  *)pMsgData;

            MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Error %d",
                              pReq->errCode);
           break;

        }

    default:
      break;
  }
}

/*********************************************************************
 * @fn      Data_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the data
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Data_start( void )
{
  bStatus_t status = SUCCESS;

  // Register the handlers
  status = BLEAppUtil_registerEventHandler( &dataGATTHandler );

  // Return status value
  return( status );
}
