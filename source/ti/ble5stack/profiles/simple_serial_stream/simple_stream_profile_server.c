/**********************************************************************************************
 * Filename:       simple_stream_profile_server.c
 *
 * Description:    This file contains the implementation of the service.
 *
 * Copyright (c) 2019, Texas Instruments Incorporated
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
 *
 *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <icall.h>
#include <simple_stream_profile_server.h>
#include "icall_ble_api.h"


/*********************************************************************
 * DEFINES
 */

// The size of the notification header is opcode + handle
#define SSS_NOTI_HDR_SIZE   (ATT_OPCODE_SIZE + 2)

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// SimpleStreamServer Service UUID
CONST uint8_t SimpleStreamServerUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SIMPLESTREAMSERVER_SERV_UUID)
};

// DataIn UUID
CONST uint8_t SimpleStreamServer_DataInUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SIMPLESTREAMSERVER_DATAIN_UUID)
};
// DataOut UUID
CONST uint8_t SimpleStreamServer_DataOutUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SIMPLESTREAMSERVER_DATAOUT_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static SimpleStreamServerCBs_t *pAppCBs = NULL;

// Data queue to hold the outgoing data
static List_List streamOutQueue;

static uint16_t heapHeadroom = 0;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t SimpleStreamServerDecl = { ATT_UUID_SIZE, SimpleStreamServerUUID };

// Characteristic "DataIn" Properties (for declaration)
static uint8_t SimpleStreamServer_DataInProps = GATT_PROP_WRITE;

// Characteristic "DataIn" Value variable
static uint8_t SimpleStreamServer_DataInVal[SIMPLESTREAMSERVER_DATAIN_LEN] = {0};
// Characteristic "DataOut" Properties (for declaration)
static uint8_t SimpleStreamServer_DataOutProps = GATT_PROP_NOTIFY;

// Characteristic "DataOut" Value variable
static uint8_t SimpleStreamServer_DataOutVal[SIMPLESTREAMSERVER_DATAOUT_LEN] = {0};

// Characteristic "DataOut" CCCD
static gattCharCfg_t *SimpleStreamServer_DataOutConfig;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t SimpleStreamServerAttrTbl[] =
{
  // SimpleStreamServer Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&SimpleStreamServerDecl
  },
    // DataIn Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &SimpleStreamServer_DataInProps
    },
      // DataIn Characteristic Value
      {
        { ATT_UUID_SIZE, SimpleStreamServer_DataInUUID },
        GATT_PERMIT_WRITE,
        0,
        SimpleStreamServer_DataInVal
      },
    // DataOut Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &SimpleStreamServer_DataOutProps
    },
      // DataOut Characteristic Value
      {
        { ATT_UUID_SIZE, SimpleStreamServer_DataOutUUID },
        0,
        0,
        SimpleStreamServer_DataOutVal
      },
      // DataOut CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&SimpleStreamServer_DataOutConfig
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t SimpleStreamServer_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t SimpleStreamServer_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );
static bStatus_t SimpleStreamServer_transmitNode( SimpleStreamNode_t *node );
static bStatus_t SimpleStreamServer_queueData( SimpleStreamNode_t *node );
static void      SimpleStreamServer_clearQueue();
/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t SimpleStreamServerCBs =
{
  SimpleStreamServer_ReadAttrCB,  // Read callback function pointer
  SimpleStreamServer_WriteAttrCB, // Write callback function pointer
  NULL                          // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * SimpleStreamServer_AddService- Initializes the SimpleStreamServer service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t SimpleStreamServer_AddService( uint32_t rspTaskId )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  SimpleStreamServer_DataOutConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( SimpleStreamServer_DataOutConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, SimpleStreamServer_DataOutConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( SimpleStreamServerAttrTbl,
                                        GATT_NUM_ATTRS( SimpleStreamServerAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &SimpleStreamServerCBs );

  if (status == SUCCESS)
  {
      // Initialize the outgoing stream queue
      List_clearList(&streamOutQueue);
  }

  return ( status );
}

/*
 * SimpleStreamServer_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t SimpleStreamServer_RegisterAppCBs( SimpleStreamServerCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn          SimpleStreamServer_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t SimpleStreamServer_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;

  // If we get here, that means you've forgotten to add an if clause for a
  // characteristic value attribute in the attribute table that has READ permissions.
  *pLen = 0;
  status = ATT_ERR_ATTR_NOT_FOUND;

  return status;
}

/*********************************************************************
 * @fn      SimpleStreamServer_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t SimpleStreamServer_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);

    if ( pAppCBs && pAppCBs->pfnCccUpdateCb )
    {
        uint16_t value = pValue[0];

        // Call app function from stack task context.
        pAppCBs->pfnCccUpdateCb(value);
    }
  }
  // See if request is regarding the DataIn Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, SimpleStreamServer_DataInUUID, pAttr->type.len) )
  {
      // Only notify application if there is any data in the payload
      if ( len > 0 )
      {
          if ( pAppCBs && pAppCBs->pfnIncomingDataCb )
          {
              // Call app function from stack task context.
              pAppCBs->pfnIncomingDataCb(connHandle, paramID, len, pValue);
          }
      }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}

/*********************************************************************
 * @fn      SimpleStreamServer_queueData
 *
 * @brief   Adds a new SimpleStreamNode_t node to the data queue
 *
 * @param   node  - data node to add to the queue
 *
 * @return  SUCCESS, FAILURE or INVALIDPARAMETER
 */
static bStatus_t SimpleStreamServer_queueData( SimpleStreamNode_t *node )
{
    bStatus_t ret = SUCCESS;
    gattCharCfg_t *pItem = NULL;

    if (node != NULL)
    {
        // Find the correct CCCD
        int i;
        for ( i = 0; i < linkDBNumConns; i++ )
        {
            if (SimpleStreamServer_DataOutConfig[i].connHandle == node->connHandle)
            {
                pItem = &(SimpleStreamServer_DataOutConfig[i]);
                break;
            }
        }

        // Only store the data if the connection is valid an notifications is allowed
        if ( ( pItem != NULL) &&
             ( pItem->connHandle != LINKDB_CONNHANDLE_INVALID ) &&
             ( pItem->value != GATT_CFG_NO_OPERATION ) &&
             ( pItem->value & GATT_CLIENT_CFG_NOTIFY ))
        {
            List_put(&streamOutQueue, (List_Elem *) node);
        }
        else
        {
            ret = FAILURE;
        }
    }
    else
    {
        ret = INVALIDPARAMETER;
    }

    return ret;
}

/*********************************************************************
 * @fn          SimpleStreamServer_transmitNode
 *
 * @brief       Transmits as much as possible of a SimpleStreamNode_t node
 *              over BLE notifications.
 *
 * @param       node - The node to send
 *
 * @return      SUCCESS, FAILURE, INVALIDPARAMETER, MSG_BUFFER_NOT_AVAIL,
 *              bleNotCOnnected, bleMemAllocError, blePending, bleInvaludMtuSize or
 *              bleTimeout
 */
static bStatus_t SimpleStreamServer_transmitNode( SimpleStreamNode_t* node)
{
    bStatus_t ret = SUCCESS;
    attHandleValueNoti_t noti;
    linkDBInfo_t connInfo;

    // Find out what the maximum MTU size is
    ret = linkDB_GetInfo(node->connHandle, &connInfo);

    // Queue up as many notification slots as possible
    if ( (ret == SUCCESS) && (node != NULL) ) {

        // Determine allocation size
        uint16_t allocLen = (node->len - node->offset);
        if ( allocLen > (connInfo.MTU - SSS_NOTI_HDR_SIZE) )
        {
            allocLen = connInfo.MTU - SSS_NOTI_HDR_SIZE;
        }

        noti.len = 0;
        noti.pValue = (uint8 *)GATT_bm_alloc( node->connHandle, ATT_HANDLE_VALUE_NOTI,
                                              allocLen, &noti.len );

        // If allocation was successful, copy out data out of the buffer and send it
        if (noti.pValue) {

            memcpy(noti.pValue, (void *) ((uint8_t *) node->payload + node->offset), noti.len);

            // The outgoing data attribute offset is 4
            noti.handle = SimpleStreamServerAttrTbl[4].handle;

            ret = GATT_Notification( node->connHandle, &noti, FALSE );

            // If unable to send the message, free allocated buffers and return
            if ( ret != SUCCESS )
            {
                GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
            }
            else
            {
                // Increment node data offset
                node->offset += noti.len;
            }
        }
        else
        {
            // Unable to allocate space for a notification, return failure
            ret = bleMemAllocError;
        }
    }

    return ret;
}

/*********************************************************************
 * @fn      SimpleStreamServer_clearQueue
 *
 * @brief   Clears and free the allocated outgoing stream queue
 *
 * @param   None
 *
 * @return  None
 */
void SimpleStreamServer_clearQueue()
{
    // Pop and free the whole queue
    while(!List_empty(&streamOutQueue))
    {
        SimpleStreamNode_t *node = (SimpleStreamNode_t *) List_get(&streamOutQueue);
        ICall_free(node);
    }
}

/*********************************************************************
 * @fn      SimpleStreamServer_processStream
 *
 * @brief   Sends out as much as possible from the outgoing stream
 *          queue using BLE notifications
 *
 * @param   connHandle  - connection message was received on
 * @param   *pValue     - pointer to data buffer
 * @param   len         - size of the data buffer
 *
 * @return  SUCCESS, FAILURE, INVALIDPARAMETER, MSG_BUFFER_NOT_AVAIL,
 *          bleNotCOnnected, bleMemAllocError, blePending, bleInvaludMtuSize or
 *          bleTimeout
 */
bStatus_t SimpleStreamServer_processStream()
{
    bStatus_t ret = SUCCESS;

    // Send data starting from the list head
    SimpleStreamNode_t *node = (SimpleStreamNode_t *) List_get(&streamOutQueue);

    while ((ret == SUCCESS) && (node != NULL))
    {
        ret = SimpleStreamServer_transmitNode(node);

        // Check that we really did send all data before freeing the node
        if ((node->len - node->offset) == 0)
        {
            ICall_free(node);
            // Move to next queue entry
            node = (SimpleStreamNode_t *) List_get(&streamOutQueue);
        }

        if (ret != SUCCESS)
        {
            // We could not send all the data contained in the node, add it back to the queue
            List_putHead(&streamOutQueue, (List_Elem *) node);
        }
    }

    return ret;
}

/*********************************************************************
 * @fn      SimpleStreamServer_disconnectStream
 *
 * @brief   Disconnect the steam.
 *          Clear and free up the existing outgoing stream queue.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleStreamServer_disconnectStream()
{
    // Clear the outgoing stream queue
    SimpleStreamServer_clearQueue();
}

/*********************************************************************
 * @fn      SimpleStreamServer_setHeadroomLimit
 *
 * @brief   Sets the limit on how much heap that needs to be available
 *          following a memory allocation.
 *
 * @param   minHeapHeadRoom - Smallest amount of free heap following
 *          an memory allocation.
 *
 * @return  none
 */
void SimpleStreamServer_setHeadroomLimit(uint16_t minHeapHeadroom)
{
    // Store the minimal heap headroom limit
    heapHeadroom = minHeapHeadroom;
}

/*********************************************************************
 * @fn      SimpleStreamServer_allocateWithHeadroom
 *
 * @brief   Checks if there will be enough free heap left following
 *          a memory allocation. If there is enough heap, it will allocate
 *          the memory.
 *
 * @param   allocSize - number of bytes to be allocated
 *
 * @return  none
 */
void* SimpleStreamServer_allocateWithHeadroom(uint16_t allocSize)
{
    void *allocatedBuffer = NULL;
    ICall_heapStats_t  stats;
    ICall_CSState key;

    // Perform this inside a critical section
    key = ICall_enterCriticalSection();

    // Get the current free heap
    ICall_getHeapStats(&stats);

    if (((uint16_t) allocSize) < ((int32_t)(stats.totalFreeSize - heapHeadroom)))
    {
        allocatedBuffer = ICall_malloc(allocSize);
    }

    // Leave the critical section
    ICall_leaveCriticalSection(key);

    return allocatedBuffer;
}

/*********************************************************************
 * @fn      SimpleStreamServer_sendData
 *
 * @brief   Put the data into the outgoing stream queue and sends as
 *          much as possible using BLE notifications.
 *
 * @param   connHandle  - connection message was received on
 * @param   *pValue     - pointer to data buffer
 * @param   len         - size of the data buffer
 *
 * @return  SUCCESS, FAILURE, INVALIDPARAMETER, MSG_BUFFER_NOT_AVAIL,
 *          bleNotCOnnected, bleMemAllocError, blePending, bleInvaludMtuSize or
 *          bleTimeout
 */
bStatus_t SimpleStreamServer_sendData(uint16_t connHandle, void *data, uint16_t len)
{
    bStatus_t ret = bleMemAllocError;
    SimpleStreamNode_t* newNode;

    // Store the data into the queue
    newNode = (SimpleStreamNode_t*) SimpleStreamServer_allocateWithHeadroom(sizeof(SimpleStreamNode_t) + len);
    if (newNode != NULL)
    {
        newNode->connHandle = connHandle;
        newNode->offset     = 0;
        newNode->len        = len;
        memcpy(newNode->payload, data, len);

        // Add the data to the stream queue
        ret = SimpleStreamServer_queueData(newNode);

        if (ret == SUCCESS)
        {
            ret = SimpleStreamServer_processStream();
        }
        else if (ret == FAILURE)
        {
            ICall_free(newNode);
        }
    }

    return ret;
}
