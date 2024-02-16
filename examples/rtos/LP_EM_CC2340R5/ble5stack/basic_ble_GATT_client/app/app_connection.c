/******************************************************************************

@file  app_connection.c

@brief This example file demonstrates how to activate the central role with
the help of BLEAppUtil APIs.

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
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <stdarg.h>

#include "ti_ble_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>

//*****************************************************************************
//! Prototypes
//*****************************************************************************
void Connection_ConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
void Connection_HciGAPEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

static uint8_t Connection_addConnInfo(uint16_t connHandle, uint8_t *pAddr);
static uint8_t Connection_removeConnInfo(uint16_t connHandle);

//*****************************************************************************
//! Globals
//*****************************************************************************

// Events handlers struct, contains the handlers and event masks
// of the application central role module
BLEAppUtil_EventHandler_t connectionConnHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_CONN_TYPE,
    .pEventHandler  = Connection_ConnEventHandler,
    .eventMask      = BLEAPPUTIL_LINK_ESTABLISHED_EVENT |
                      BLEAPPUTIL_LINK_TERMINATED_EVENT |
                      BLEAPPUTIL_LINK_PARAM_UPDATE_EVENT |
                      BLEAPPUTIL_LINK_PARAM_UPDATE_REQ_EVENT
};

BLEAppUtil_EventHandler_t connectionHciGAPHandler =
{
    .handlerType    = BLEAPPUTIL_HCI_GAP_TYPE,
    .pEventHandler  = Connection_HciGAPEventHandler,
    .eventMask      = BLEAPPUTIL_HCI_COMMAND_STATUS_EVENT_CODE |
                      BLEAPPUTIL_HCI_LE_EVENT_CODE
};

// Holds the connection handles
static App_connInfo connectionConnList[MAX_NUM_BLE_CONNS];

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Connection_ConnEventHandler
 *
 * @brief   The purpose of this function is to handle connection related
 *          events that rise from the GAP and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Connection_ConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch(event)
    {
        case BLEAPPUTIL_LINK_ESTABLISHED_EVENT:
        {
            gapEstLinkReqEvent_t *gapEstMsg = (gapEstLinkReqEvent_t *)pMsgData;

            // Add the connection to the connected device list
            Connection_addConnInfo(gapEstMsg->connectionHandle, gapEstMsg->devAddr);

            /*! Print the peer address and connection handle number */
            MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Conn status: Established - "
                              "Connected to " MENU_MODULE_COLOR_YELLOW "%s " MENU_MODULE_COLOR_RESET
                              "connectionHandle = " MENU_MODULE_COLOR_YELLOW "%d" MENU_MODULE_COLOR_RESET,
                              BLEAppUtil_convertBdAddr2Str(gapEstMsg->devAddr), gapEstMsg->connectionHandle);

            /*! Print the number of current connections */
            MenuModule_printf(APP_MENU_NUM_CONNS, 0, "Connections number: "
                              MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                              linkDB_NumActive());

            break;
        }

        case BLEAPPUTIL_LINK_TERMINATED_EVENT:
        {
            gapTerminateLinkEvent_t *gapTermMsg = (gapTerminateLinkEvent_t *)pMsgData;

            // Remove the connection from the conneted device list
            Connection_removeConnInfo(gapTermMsg->connectionHandle);

            /*! Print the peer address and connection handle number */
            MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Conn status: Terminated - "
                              "connectionHandle = " MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                              "reason = " MENU_MODULE_COLOR_YELLOW "%d" MENU_MODULE_COLOR_RESET,
                              gapTermMsg->connectionHandle, gapTermMsg->reason);

            /*! Print the number of current connections */
            MenuModule_printf(APP_MENU_NUM_CONNS, 0, "Connections number: "
                              MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                              linkDB_NumActive());

            break;
        }

        case BLEAPPUTIL_LINK_PARAM_UPDATE_REQ_EVENT:
        {
            gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsgData;

            // Only accept connection intervals with slave latency of 0
            // This is just an example of how the application can send a response
            if(pReq->req.connLatency == 0)
            {
                BLEAppUtil_paramUpdateRsp(pReq,TRUE);
            }
            else
            {
                BLEAppUtil_paramUpdateRsp(pReq,FALSE);
            }

            break;
        }

        case BLEAPPUTIL_LINK_PARAM_UPDATE_EVENT:
        {
            gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsgData;

            // Get the address from the connection handle
            linkDBInfo_t linkInfo;
            if (linkDB_GetInfo(pPkt->connectionHandle, &linkInfo) ==  SUCCESS)
            {
              // The status HCI_ERROR_CODE_PARAM_OUT_OF_MANDATORY_RANGE indicates that connection params did not change but the req and rsp still transpire
              if(pPkt->status == SUCCESS)
              {
                  MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Conn status: Params update - "
                                    "connectionHandle = " MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                                    pPkt->connectionHandle);
              }
              else
              {
                  MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Conn status: Params update failed - "
                                    MENU_MODULE_COLOR_YELLOW "0x%x " MENU_MODULE_COLOR_RESET
                                    "connectionHandle = " MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                                    pPkt->opcode, pPkt->connectionHandle);
              }
            }

            break;
        }

        default:
        {
            break;
        }
    }
}

/*********************************************************************
 * @fn      Connection_HciGAPEventHandler
 *
 * @brief   The purpose of this function is to handle HCI GAP events
 *          that rise from the HCI and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Connection_HciGAPEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{

    switch (event)
    {
        case BLEAPPUTIL_HCI_COMMAND_STATUS_EVENT_CODE:
        {
            hciEvt_CommandStatus_t *pHciMsg = (hciEvt_CommandStatus_t *)pMsgData;
            switch ( event )
            {
              case HCI_LE_SET_PHY:
              {
                  if (pHciMsg->cmdStatus ==
                      HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
                  {
                      MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Conn status: Phy update - failure, peer does not support this");
                  }
                  else
                  {
                      MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Conn status: Phy update - "
                                        MENU_MODULE_COLOR_YELLOW "0x%02x" MENU_MODULE_COLOR_RESET,
                                        pHciMsg->cmdStatus);
                  }
                  break;
              }


              default:
              {
                  break;
              }
              break;
            }
        }

        case BLEAPPUTIL_HCI_LE_EVENT_CODE:
        {
            hciEvt_BLEPhyUpdateComplete_t *pPUC = (hciEvt_BLEPhyUpdateComplete_t*) pMsgData;

            if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
            {
              if (pPUC->status != SUCCESS)
              {
                  MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Conn status: Phy update failure - connHandle = %d",
                                    pPUC->connHandle);
              }
              else
              {
#if !defined(Display_DISABLE_ALL)
                  char * currPhy =
                          (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1 Mbps" :
                          (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2 Mbps" :
                          (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "CODED" : "Unexpected PHY Value";
                  MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Conn status: Phy update - connHandle = %d PHY = %s",
                                    pPUC->connHandle, currPhy);
#endif // #if !defined(Display_DISABLE_ALL)
              }
            }

            break;
        }

        default:
        {
            break;
        }

    }
}

/*********************************************************************
 * @fn      Connection_addConnInfo
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          If there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t Connection_addConnInfo(uint16_t connHandle, uint8_t *pAddr)
{
  uint8_t i = 0;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connectionConnList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connectionConnList[i].connHandle = connHandle;
      memcpy(connectionConnList[i].peerAddress, pAddr, B_ADDR_LEN);

      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      Connection_removeConnInfo
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          If connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t Connection_removeConnInfo(uint16_t connHandle)
{
  uint8_t i = 0;
  uint8_t index = 0;
  uint8_t maxNumConn = MAX_NUM_BLE_CONNS;

  for (i = 0; i < maxNumConn; i++)
  {
    if (connectionConnList[i].connHandle == connHandle)
    {
      // Mark the entry as deleted
      connectionConnList[i].connHandle = LINKDB_CONNHANDLE_INVALID;

      break;
    }
  }

  // Save the index to return
  index = i;

  // Shift the items in the array
  for(i = 0; i < maxNumConn - 1; i++)
  {
      if (connectionConnList[i].connHandle == LINKDB_CONNHANDLE_INVALID &&
          connectionConnList[i + 1].connHandle == LINKDB_CONNHANDLE_INVALID)
      {
        break;
      }
      if (connectionConnList[i].connHandle == LINKDB_CONNHANDLE_INVALID &&
          connectionConnList[i + 1].connHandle != LINKDB_CONNHANDLE_INVALID)
      {
        memmove(&connectionConnList[i], &connectionConnList[i+1], sizeof(App_connInfo));
        connectionConnList[i + 1].connHandle = LINKDB_CONNHANDLE_INVALID;
      }
  }

  return index;
}

/*********************************************************************
 * @fn      Connection_getConnList
 *
 * @brief   Get the connection list
 *
 * @return  connection list
 */
App_connInfo *Connection_getConnList(void)
{
  return connectionConnList;
}

/*********************************************************************
 * @fn      Connection_getConnhandle
 *
 * @brief   Find connHandle in the connected device list by index
 *
 * @return  the connHandle found. If there is no match,
 *          MAX_NUM_BLE_CONNS will be returned.
 */
uint16_t Connection_getConnhandle(uint8_t index)
{

    if (index < MAX_NUM_BLE_CONNS)
    {
      return connectionConnList[index].connHandle;
    }

  return MAX_NUM_BLE_CONNS;
}

/*********************************************************************
 * @fn      Connection_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the connection
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Connection_start()
{
    bStatus_t status = SUCCESS;
    uint8 i;

    // Initialize the connList handles
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        connectionConnList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
    }

    status = BLEAppUtil_registerEventHandler(&connectionConnHandler);
    if(status != SUCCESS)
    {
        return(status);
    }

    status = BLEAppUtil_registerEventHandler(&connectionHciGAPHandler);
    if(status != SUCCESS)
    {
        return(status);
    }

    return status;
}

/*********************************************************************
 * @fn      Connection_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, LL_INACTIVE_CONNECTIONS will be returned.
 */
uint16_t Connection_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connectionConnList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return LL_INACTIVE_CONNECTIONS;
}

#endif // ( HOST_CONFIG & (CENTRAL_CFG | PERIPHERAL_CFG) )
