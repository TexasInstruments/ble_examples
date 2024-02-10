/******************************************************************************

@file  app_connection.c

@brief This example file demonstrates how to activate the central role with
the help of BLEAppUtil APIs.

Group: WCS, BTS
$Target Device: DEVICES $

******************************************************************************
$License: BSD3 2022 $
******************************************************************************
$Release Name: PACKAGE NAME $
$Release Date: PACKAGE RELEASE DATE $
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

            break;
        }

        case BLEAPPUTIL_LINK_TERMINATED_EVENT:
        {
            gapTerminateLinkEvent_t *gapTermMsg = (gapTerminateLinkEvent_t *)pMsgData;

            // Remove the connection from the conneted device list
            Connection_removeConnInfo(gapTermMsg->connectionHandle);

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

              if(pPkt->status == SUCCESS)
              {
            
              }
              else
              {
     
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
                    
                  }
                  else
                  {
                 
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
               
              }
              else
              {
#if !defined(Display_DISABLE_ALL)
                  char * currPhy =
                          (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1 Mbps" :
                          (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2 Mbps" :
                          (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "CODED" : "Unexpected PHY Value";
                 
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

#endif // ( HOST_CONFIG & (CENTRAL_CFG | PERIPHERAL_CFG) )
