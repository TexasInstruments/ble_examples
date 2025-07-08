/******************************************************************************

@file  audio_stream_server.c

@brief This file contains the Data Stream service sample GATT service
        for use with the BLE sample application.

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2025, Texas Instruments Incorporated
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

 Copyright (c) 2022-2025, Texas Instruments Incorporated
 All rights reserved.

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 ******************************************************************************
 *****************************************************************************/

#ifdef AUDIO_TEST

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <icall.h>
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "audio_stream_server.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include "ble_stack_api.h"
#include <ti/drivers/GPIO.h>

/*********************************************************************
 * CONSTANTS
 */
// The size of the notification header is opcode + handle
#define AS_NOTI_HDR_SIZE   (ATT_OPCODE_SIZE + 2)

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Data Stream Server Service UUID: 0xC0C0
GATT_UUID( as_serv_UUID, AS_SERV_UUID );

// Data In Characteristic UUID: 0xC0C1
GATT_UUID( as_dataIn_UUID, AS_DATAIN_UUID );

// Data Out Characteristic UUID: 0xC0C2
GATT_UUID( as_dataOut_UUID, AS_DATAOUT_UUID );



static AS_cb_t *as_profileCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Data Stream Server Service declaration
static const gattAttrType_t as_service = { ATT_UUID_SIZE, as_serv_UUID };

// Characteristic "DataIn" Properties
static uint8 as_dataIn_props = GATT_PROP_WRITE;

// Characteristic "DataIn" Value variable
static uint8 as_dataIn_val[AS_DATAIN_LEN] = {0};

// Characteristic "DataIn" User Description
static uint8 as_dataIn_userDesp[] = "Write Data";

// Characteristic "DataOut" Properties
static uint8 as_dataOut_props = GATT_PROP_NOTIFY;

// Characteristic "DataOut" Value variable
uint8 as_dataOut_val[AS_DATAOUT_LEN] = {0};

// Characteristic "DataOut" Configuration each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *as_dataOut_config;

// Characteristic "DataOut" User Description
static uint8 as_dataOut_userDesp[] = "Server Data";

// Data queue to hold the outgoing data
static List_List streamOutQueue;

static uint16_t heapHeadroom = 0;

/*********************************************************************
 * Profile Attributes - Table
 */

//static gattAttribute_t as_attrTbl[] =
//{
// /*--------------------type-------------------*/ /*------------permissions-------------*/ /*------------------pValue--------------------*/
//   // Data Stream Service
//   GATT_BT_ATT( primaryServiceUUID,                 GATT_PERMIT_READ,                        (uint8 *) &as_service ),
//
//   // DataIn Characteristic Properties
//   GATT_BT_ATT( characterUUID,                      GATT_PERMIT_READ,                        &as_dataIn_props ),
//   // DataIn Characteristic Value
//   GATT_ATT( as_dataIn_UUID,                       GATT_PERMIT_WRITE,                       as_dataIn_val ),
//   // DataIn Characteristic User Description
//   GATT_BT_ATT( charUserDescUUID,                   GATT_PERMIT_READ,                        as_dataIn_userDesp ),
//
//   // DataOut Characteristic Properties
//   GATT_BT_ATT( characterUUID,                      GATT_PERMIT_READ,                        &as_dataOut_props ),
//   // DataOut Characteristic Value
//   GATT_ATT( as_dataOut_UUID,                      0,                                       as_dataOut_val ),
//   // DataOut Characteristic configuration
//   GATT_BT_ATT( clientCharCfgUUID,                  GATT_PERMIT_READ | GATT_PERMIT_WRITE,    (uint8 *) &as_dataOut_config ),
//   // DataOut Characteristic User Description
//   GATT_BT_ATT( charUserDescUUID,                   GATT_PERMIT_READ,                        as_dataOut_userDesp ),
//};


static gattAttribute_t as_attrTbl[] =
{
  // SimpleStreamServer Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&as_service
  },
    // DataIn Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &as_dataIn_props
    },
      // DataIn Characteristic Value
      {
        { ATT_UUID_SIZE, as_dataIn_UUID },
        GATT_PERMIT_WRITE,
        0,
        as_dataIn_val
      },
    // DataOut Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &as_dataOut_props
    },
      // DataOut Characteristic Value
      {
        { ATT_UUID_SIZE, as_dataOut_UUID },
        0,
        0,
        as_dataOut_val
      },
      // DataOut CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&as_dataOut_config
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t AS_writeAttrCB( uint16 connHandle,
                                  gattAttribute_t *pAttr,
                                  uint8 *pValue, uint16 len,
                                  uint16 offset, uint8 method );

static bStatus_t AS_sendNotification( uint8 *pValue, uint16 len );
static bStatus_t AS_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t AS_transmitNode( AudioStreamNode_t *node );
static bStatus_t AS_queueData( AudioStreamNode_t *node );
static void      AS_clearQueue();

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Data Stream Server Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
const gattServiceCBs_t as_servCBs =
{
 AS_ReadAttrCB,                           // Read callback function pointer
  AS_writeAttrCB,                // Write callback function pointer
  NULL                            // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AS_addService
 *
 * @brief   This function initializes the Data Stream Server service
 *          by registering GATT attributes with the GATT server.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t AS_addService( void )
{
  bStatus_t status = SUCCESS;

  // Allocate Client Characteristic Configuration table
  as_dataOut_config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * MAX_NUM_BLE_CONNS );
  if ( as_dataOut_config == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, as_dataOut_config );

  // Register GATT attribute list and CBs with GATT Server
  status = GATTServApp_RegisterService( as_attrTbl,
                                        GATT_NUM_ATTRS( as_attrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &as_servCBs );

  List_clearList(&streamOutQueue);

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      AS_registerProfileCBs
 *
 * @brief   Registers the profile callback function. Only call
 *          this function once.
 *
 * @param   profileCallback - pointer to profile callback functions.
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t AS_registerProfileCBs( AS_cb_t *profileCallback )
{
  if ( profileCallback )
  {
    as_profileCBs = profileCallback;

    return ( SUCCESS );
  }

  return ( INVALIDPARAMETER );
}

/*********************************************************************
 * @fn      AS_setParameter
 *
 * @brief   Set a Data Stream Service parameter.
 *
 * @param   param - Characteristic UUID
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 * @param   len - length of data to write
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t AS_setParameter(uint8 param, void *pValue, uint16 len)
{
  bStatus_t status = SUCCESS;

  // Verify input parameters
  if ( pValue == NULL )
  {
    return ( INVALIDPARAMETER );
  }

  switch ( param )
  {
    case AS_DATAOUT_ID:
      status = AS_sendNotification( (uint8 *)pValue, len );
      break;

    default:
      status = INVALIDPARAMETER;
      break;
  }

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      AS_writeAttrCB
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
 * @return  SUCCESS or stack call status
 */
static bStatus_t AS_writeAttrCB( uint16 connHandle,
                                        gattAttribute_t *pAttr,
                                        uint8 *pValue, uint16 len,
                                        uint16 offset, uint8 method )
{
  bStatus_t status = SUCCESS;

  // Verify input parameters
  if ( pAttr == NULL || pValue == NULL )
  {
    return ( INVALIDPARAMETER );
  }

  /******************************************************/
  /****** Client Characteristic Configuration ***********/
  /******************************************************/

  if ( ! memcmp( pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len ) )
  {
     AS_cccUpdate_t *cccUpdate;

    // Allow only notifications
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY );

    // Notify profile
    if ( status == SUCCESS )
    {
      // This allocation will be free by bleapp_util
      cccUpdate = (AS_cccUpdate_t *)ICall_malloc( sizeof( AS_cccUpdate_t ) );
      if ( cccUpdate == NULL )
      {
        // Return error status
        return ( bleMemAllocError );
      }

      // Copy the data and send it to the profile
      cccUpdate->connHandle = connHandle;
      cccUpdate->value = BUILD_UINT16( pValue[0], pValue[1] );

      // Callback function to notify profile of change on the client characteristic configuration
      BLEAppUtil_invokeFunction( as_profileCBs->pfnOnCccUpdateCB, (char *)cccUpdate );
    }
  }

  /******************************************************/
  /*********** Data In Characteristic  ******************/
  /******************************************************/
  else if ( ! memcmp( pAttr->type.uuid, as_dataIn_UUID, pAttr->type.len ) )
  {
    // Only notify profile if there is any data in the payload
    if ( len > 0  && as_profileCBs && as_profileCBs->pfnIncomingDataCB)
    {
      AS_dataIn_t *dataIn;


      // This allocation will be free by bleapp_util
      dataIn = (AS_dataIn_t *)ICall_malloc( sizeof( AS_dataIn_t ) + len);
      if ( dataIn == NULL )
      {
        // Return error status
        return ( bleMemAllocError );
      }

      // If allocation was successful,
      // Copy the data and send it to the profile
      if ( len > 0 )
      {
        memcpy( dataIn->pValue, pValue, len );
      }
      dataIn->connHandle = connHandle;
      dataIn->len = len;

      // Callback function to notify profile of change on the client characteristic configuration
      status = BLEAppUtil_invokeFunction( (void *)as_profileCBs->pfnIncomingDataCB, (char *)dataIn );
    }
  }

  // If we get here, that means you've forgotten to add an if clause for a
  // characteristic value attribute in the attribute table that has WRITE permissions.
  else
  {
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      AS_sendNotification
 *
 * @brief   Transmits data over BLE notifications.
 *
 * @param   pValue - pointer to data to be written
 * @param   len - length of data to be written
 *
 * @return  SUCCESS, or stack call status
 */

volatile uint16_t notiHandle = 0;

static bStatus_t AS_sendNotification(uint8 *pValue, uint16 len)
{
  bStatus_t status = SUCCESS;
  gattAttribute_t *pAttr = NULL;
  attHandleValueNoti_t noti = {0};
  linkDBInfo_t connInfo = {0};
  uint16 offset = 0;
  uint8 i = 0;

  // Verify input parameters
  if ( pValue == NULL )
  {
    return ( INVALIDPARAMETER );
  }

  // Find the characteristic value attribute
  pAttr = GATTServApp_FindAttr(as_attrTbl, GATT_NUM_ATTRS(as_attrTbl), as_dataOut_val);
  if ( pAttr != NULL )
  {

    // Check the ccc value for each BLE connection
    for ( i = 0; i < MAX_NUM_BLE_CONNS; i++ )
    {
      gattCharCfg_t *pItem = &( as_dataOut_config[i] );

      // If the connection has register for notifications
      if ( ( pItem->connHandle != LINKDB_CONNHANDLE_INVALID ) &&
           ( pItem->value == GATT_CLIENT_CFG_NOTIFY) )
      {
        // Find out what the maximum MTU size is for each connection
        status = linkDB_GetInfo(pItem->connHandle, &connInfo);
        offset = 0;

        while ( status == SUCCESS &&  len != offset )
        {
          // Determine allocation size
          uint16_t allocLen = (len - offset);
          if ( allocLen > ( connInfo.MTU - AS_NOTI_HDR_SIZE ) )
          {
            // If len > MTU split data to chunks of MTU size
            allocLen = connInfo.MTU - AS_NOTI_HDR_SIZE;
          }

          noti.len = allocLen;
          noti.pValue = (uint8 *)GATT_bm_alloc( pItem->connHandle, ATT_HANDLE_VALUE_NOTI, allocLen, 0 );
          if ( noti.pValue != NULL )
          {
            // If allocation was successful, copy out data and send it
            memcpy(noti.pValue, pValue + offset, noti.len);
            noti.handle = pAttr->handle;

            // Send the data over BLE notifications
            notiHandle = noti.handle;
            status = GATT_Notification( pItem->connHandle, &noti, FALSE );

            // If unable to send the data, free allocated buffers and return
            if ( status != SUCCESS )
            {
              GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
            }
            else
            {
              // Increment data offset
              offset += allocLen;
            }
          }
          else
          {
            status = bleNoResources;
          }
        } // End of while
      }
    } // End of for
  } // End of if

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      AS_queueData
 *
 * @brief   Adds a new AudioStreamNode_t node to the data queue
 *
 * @param   node  - data node to add to the queue
 *
 * @return  SUCCESS, FAILURE or INVALIDPARAMETER
 */
static bStatus_t AS_queueData( AudioStreamNode_t *node )
{
    bStatus_t ret = SUCCESS;
    gattCharCfg_t *pItem = NULL;

    if (node != NULL)
    {
        // Find the correct CCCD
        int i;
        for ( i = 0; i < linkDBNumConns; i++ )
        {
            if (as_dataOut_config[i].connHandle == node->connHandle)
            {
                pItem = &(as_dataOut_config[i]);
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
 * @fn          AS_transmitNode
 *
 * @brief       Transmits as much as possible of a AudioStreamNode_t node
 *              over BLE notifications.
 *
 * @param       node - The node to send
 *
 * @return      SUCCESS, FAILURE, INVALIDPARAMETER, MSG_BUFFER_NOT_AVAIL,
 *              bleNotCOnnected, bleMemAllocError, blePending, bleInvaludMtuSize or
 *              bleTimeout
 */
int totalNotificationsAttempted = 0;
int notificationsFailed = 0;
int notificationsSent = 0;
int lengthOfNotification = 0;
int notificationAllocationsAttempted = 0;
int notificationAllocationsFailed = 0;
int notificationAllocationsSuccess = 0;
uint8_t *payloadNotification;
uint8_t testPayload[] =
{
 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31,
 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d,
 0x6e, 0x6f
};
static bStatus_t AS_transmitNode( AudioStreamNode_t* node)
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
        if ( allocLen > (connInfo.MTU - AS_NOTI_HDR_SIZE) )
        {
            allocLen = connInfo.MTU - AS_NOTI_HDR_SIZE;
        }

        noti.len = 0;
        noti.pValue = (uint8 *)GATT_bm_alloc( node->connHandle, ATT_HANDLE_VALUE_NOTI,
                                              allocLen, &noti.len );
        notificationAllocationsAttempted++;

        // If allocation was successful, copy out data out of the buffer and send it
        if (noti.pValue) {
            notificationAllocationsSuccess++;

            // Normal use
            memcpy(noti.pValue, (void *) ((uint8_t *) node->payload + node->offset), noti.len);

            // Known data test
//            memcpy(noti.pValue, (void *) ((uint8_t *) node->payload + node->offset), noti.len);


            lengthOfNotification = noti.len;
            payloadNotification = noti.pValue;

            // The outgoing data attribute offset is 4
            noti.handle = as_attrTbl[4].handle;

            ret = GATT_Notification( node->connHandle, &noti, FALSE );
            totalNotificationsAttempted++;

            // If unable to send the message, free allocated buffers and return
            if ( ret != SUCCESS )
            {
                GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
                notificationsFailed++;
            }
            else
            {
                // Increment node data offset
                node->offset += noti.len;
                notificationsSent++;
            }
        }
        else
        {
            // Unable to allocate space for a notification, return failure
            ret = bleMemAllocError;
            notificationAllocationsFailed++;
        }
    }

    return ret;
}

/*********************************************************************
 * @fn      AS_clearQueue
 *
 * @brief   Clears and free the allocated outgoing stream queue
 *
 * @param   None
 *
 * @return  None
 */
void AS_clearQueue()
{
    // Pop and free the whole queue
    while(!List_empty(&streamOutQueue))
    {
        AudioStreamNode_t *node = (AudioStreamNode_t *) List_get(&streamOutQueue);
        ICall_free(node);
    }
}

/*********************************************************************
 * @fn      AS_processStream
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
bStatus_t AS_processStream()
{
    bStatus_t ret = SUCCESS;

    // Send data starting from the list head
    AudioStreamNode_t *node = (AudioStreamNode_t *) List_get(&streamOutQueue);

    while ((ret == SUCCESS) && (node != NULL))
    {
        ret = AS_transmitNode(node);

        // Check that we really did send all data before freeing the node
        if ((node->len - node->offset) == 0)
        {
            ICall_free(node);
            // Move to next queue entry
            node = (AudioStreamNode_t *) List_get(&streamOutQueue);
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
 * @fn      AS_disconnectStream
 *
 * @brief   Disconnect the steam.
 *          Clear and free up the existing outgoing stream queue.
 *
 * @param   none
 *
 * @return  none
 */
void AS_disconnectStream()
{
    // Clear the outgoing stream queue
    AS_clearQueue();
}

/*********************************************************************
 * @fn      AS_setHeadroomLimit
 *
 * @brief   Sets the limit on how much heap that needs to be available
 *          following a memory allocation.
 *
 * @param   minHeapHeadRoom - Smallest amount of free heap following
 *          an memory allocation.
 *
 * @return  none
 */
void AS_setHeadroomLimit(uint16_t minHeapHeadroom)
{
    // Store the minimal heap headroom limit
    heapHeadroom = minHeapHeadroom;
}

/*********************************************************************
 * @fn      AS_allocateWithHeadroom
 *
 * @brief   Checks if there will be enough free heap left following
 *          a memory allocation. If there is enough heap, it will allocate
 *          the memory.
 *
 * @param   allocSize - number of bytes to be allocated
 *
 * @return  none
 */
int32_t totalFreeSizeTest = 0;
uint16_t allocSizeTest= 0;
void* AS_allocateWithHeadroom(uint16_t allocSize)
{
    void *allocatedBuffer = NULL;
    ICall_heapStats_t  stats;
    ICall_CSState key;
    allocSizeTest = allocSize;

    // Perform this inside a critical section
    key = ICall_enterCriticalSection();

    // Get the current free heap
    ICall_getHeapStats(&stats);
    totalFreeSizeTest = stats.totalFreeSize;
//    if (((uint16_t) allocSize) < ((int32_t)(stats.totalFreeSize - heapHeadroom)))
//    {
        allocatedBuffer = ICall_malloc(allocSize);
//    }

    // Leave the critical section
    ICall_leaveCriticalSection(key);

    return allocatedBuffer;
}

/*********************************************************************
 * @fn      AS_sendData
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
uint8_t testNodePayload[] =
{
 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31,
 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d,
 0x6e, 0x6f
};
uint8_t nodeLength = 0;
uint8_t* nodePayloadQueued;
int queueSuccess = 0;
int queueFailure = 0;
bStatus_t AS_sendData(uint16_t connHandle, void *data, uint16_t len)
{

    bStatus_t ret = SUCCESS;
    attHandleValueNoti_t noti;
    linkDBInfo_t connInfo;

        uint16_t allocLen = len;

        noti.len = 0;
        noti.pValue = (uint8 *)GATT_bm_alloc( connHandle, ATT_HANDLE_VALUE_NOTI,
                                              allocLen, &noti.len );
        notificationAllocationsAttempted++;

        // If allocation was successful, copy out data out of the buffer and send it
        if (noti.pValue)
        {
            notificationAllocationsSuccess++;

            // Normal use
            memcpy(noti.pValue, (void *) ((uint8_t *) data), noti.len);

            // Known data test
//            memcpy(noti.pValue, (void *) ((uint8_t *) node->payload + node->offset), noti.len);


            lengthOfNotification = noti.len;
            payloadNotification = noti.pValue;

            noti.handle = (GATTServApp_FindAttr(as_attrTbl, GATT_NUM_ATTRS(as_attrTbl), as_dataOut_val))->handle;
            //noti.handle = as_attrTbl[4].handle;

            ret = GATT_Notification( connHandle, &noti, FALSE );
            totalNotificationsAttempted++;

            // If unable to send the message, free allocated buffers and return
            if ( ret != SUCCESS )
            {
                GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
                notificationsFailed++;
            }
            else
            {
                notificationsSent++;
            }
        }
        else
        {
            // Unable to allocate space for a notification, return failure
            ret = bleMemAllocError;
            notificationAllocationsFailed++;
        }

    return ret;

}

static bStatus_t AS_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
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
*********************************************************************/
#endif


#ifndef AUDIO_TEST
/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <icall.h>
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "audio_stream_server.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include "ble_stack_api.h"
#include <ti/drivers/GPIO.h>

/*********************************************************************
 * CONSTANTS
 */
// The size of the notification header is opcode + handle
#define AS_NOTI_HDR_SIZE   (ATT_OPCODE_SIZE + 2)

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/// @brief TI Base 128-bit UUID: F000XXXX-0451-4000-B000-000000000000
#define ANDROID_BASE_UUID_128( uuid )  0x64, 0xB6, 0x17, 0xF6, 0x01, 0xAF, 0x7D, 0xBC, \
                                  0x05, 0x4F, 0x21, 0x5A, LO_UINT16( uuid ), HI_UINT16( uuid ), 0x5E, 0xAB

static uint16_t heapHeadroom = 0;

// Data queue to hold the outgoing data
static List_List streamOutQueue;

// Audio GATT Profile Service UUID: 0x0001
static const uint8 audioProfileServUUID[ATT_UUID_SIZE] =
{
 ANDROID_BASE_UUID_128(AUDIO_GOOGLE_SERVICE_UUID)
};

// Write Characteristic UUID: 0x0002
static const uint8 audioProfileWriteUUID[ATT_UUID_SIZE] =
{
 ANDROID_BASE_UUID_128(AUDIO_GOOGL_TX_CHAR_UUID)
};

// Read Characteristic UUID: 0x0003
static const uint8 audioProfileReadUUID[ATT_UUID_SIZE] =
{
 ANDROID_BASE_UUID_128(AUDIO_GOOGL_RX_CHAR_UUID)
};

// Control Characteristic UUID: 0x0004
static const uint8 audioProfileControlUUID[ATT_UUID_SIZE] =
{
 ANDROID_BASE_UUID_128(AUDIO_GOOGL_CTL_CHAR_UUID)
};

//----------------------------------------------------------------------//

typedef struct {
    uint16_t version;
    uint16_t legacy_0x0003;
    uint8_t  supported_assist;
} rc_req_capabilities;

typedef struct {
    rc_req_capabilities get_caps;
    uint8_t mic_open;
    uint8_t mic_close;
    uint8_t mic_extend;
} atvv_char_tx;

typedef struct {
    uint8_t  reason;
    uint8_t  codec_used;
    uint8_t  stream_id;
} audio_start_struct;

typedef struct {
    uint8_t  codec_used;
    uint16_t frame_no;
    uint16_t pred_value;
    uint8_t  step_index;
} audio_sync_struct;

typedef struct {
    uint8_t command;
    uint16_t version;
    uint8_t codec_supported;
    uint8_t assistant_interaction;
    uint16_t audio_fram_size;
    uint8_t extra_config;
    uint8_t reserved;
} rc_rep_capabilities;

typedef struct {
    uint8_t audio_stop;
    audio_start_struct audio_start;
    uint8_t start_search;
    audio_sync_struct audio_sync;
    rc_rep_capabilities caps_resp;
    uint16_t mic_open_error;
} atvv_char_ctl;

#define ATVV_CHAR_TX_LEN 8
#define ATVV_CHAR_AUDIO_LEN 255
#define ATVV_CHAR_CTL_LEN 21

//AUDIO_GOOGLE_SERVICE_UUID
// Audio Profile Service
static const gattAttrType_t audioProfileService = {ATT_UUID_SIZE, audioProfileServUUID};

//AUDIO_GOOGL_TX_CHAR_UUID
//atv_char_tx_data
// WRITE Characteristic Properties
static uint8 audioProfileWriteProps = GATT_PROP_WRITE_NO_RSP;
// WRITE Characteristic Value
static uint8_t audioProfileWrite[ATVV_CHAR_TX_LEN];
// WRITE Characteristic Configuration Descriptor Value
static gattCharCfg_t *audioProfileWriteConfig;

// READ Characteristic Properties
static uint8 audioProfileReadProps = GATT_PROP_NOTIFY;
// READ Characteristic Value
static uint8_t audioProfileRead[ATVV_CHAR_AUDIO_LEN];
// READ Characteristic Configuration Descriptor Value
static gattCharCfg_t *audioProfileReadConfig;

// CONTROL Characteristic Properties
static uint8 audioProfileControlProps = GATT_PROP_NOTIFY;
// CONTROL Characteristic Value
static uint8_t audioProfileControl[ATVV_CHAR_CTL_LEN];
// CONTROL Characteristic Configuration Descriptor Value
static gattCharCfg_t *audioProfileControlConfig;

static AS_cb_t *as_profileCBs = NULL;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t audioProfileAttrTbl[] =
{
 /*--------------------type-------------------*/ /*------------permissions-------------*/ /*------------------pValue--------------------*/
   // GOOGLE Audio Stream Service
   GATT_BT_ATT( primaryServiceUUID,                 GATT_PERMIT_READ,                        (uint8 *)&audioProfileService ),

   // WRITE Characteristic Declaration
   GATT_BT_ATT( characterUUID,                      GATT_PERMIT_READ,                        &audioProfileWriteProps ),
   // WRITE Characteristic Value
   GATT_ATT( audioProfileWriteUUID,                 GATT_PERMIT_WRITE,                       audioProfileWrite ),
   // WRITE Characteristic configuration
   GATT_BT_ATT( clientCharCfgUUID,                  GATT_PERMIT_READ | GATT_PERMIT_WRITE,    (uint8 *)&audioProfileWriteConfig ),

   // READ Characteristic Declaration
   GATT_BT_ATT( characterUUID,                      GATT_PERMIT_READ,                        &audioProfileReadProps ),
   // READ Characteristic Value
   GATT_ATT( audioProfileReadUUID,                  GATT_PERMIT_READ,                        audioProfileRead ),
   // READ Characteristic configuration
   GATT_BT_ATT( clientCharCfgUUID,                  GATT_PERMIT_READ | GATT_PERMIT_WRITE,    (uint8 *)&audioProfileReadConfig ),

   // Control Characteristic Declaration
   GATT_BT_ATT( characterUUID,                      GATT_PERMIT_READ,                        &audioProfileControlProps ),
   // Control Characteristic Value
   GATT_ATT( audioProfileControlUUID,               GATT_PERMIT_READ,                        audioProfileControl ),
   // Control Characteristic configuration
   GATT_BT_ATT( clientCharCfgUUID,                  GATT_PERMIT_READ | GATT_PERMIT_WRITE,    (uint8 *)&audioProfileControlConfig ),
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t AS_writeAttrCB( uint16 connHandle, gattAttribute_t *pAttr, uint8 *pValue, uint16 len, uint16 offset, uint8 method );
static bStatus_t AS_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr, uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen, uint8_t method );
static bStatus_t AS_transmitNode( AudioStreamNode_t *node );
static bStatus_t AS_queueData( AudioStreamNode_t *node );
static void AS_clearQueue();

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Data Stream Server Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
const gattServiceCBs_t as_servCBs =
{
  AS_ReadAttrCB,                 // Read callback function pointer
  AS_writeAttrCB,                // Write callback function pointer
  NULL                           // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AS_addService
 *
 * @brief   This function initializes the Data Stream Server service
 *          by registering GATT attributes with the GATT server.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t AS_addService( void )
{
  bStatus_t status = SUCCESS;

  // Allocate Client Characteristic Configuration table
  audioProfileWriteConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * MAX_NUM_BLE_CONNS );
  if ( audioProfileWriteConfig == NULL )
  {
    return ( bleMemAllocError );
  }
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, audioProfileWriteConfig );

  // Allocate Audio Stream Client Characteristic Configuration table
  audioProfileReadConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) * MAX_NUM_BLE_CONNS);
  if ( audioProfileReadConfig == NULL )
  {
    return bleMemAllocError;
  }
  // Initialize Audio Stream Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, audioProfileReadConfig );

  // Allocate Audio Stream Client Characteristic Configuration table
  audioProfileControlConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) * MAX_NUM_BLE_CONNS);
  if ( audioProfileControlConfig == NULL )
  {
    return bleMemAllocError;
  }
  // Initialize Audio Stream Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, audioProfileControlConfig );


  // Register GATT attribute list and CBs with GATT Server
  status = GATTServApp_RegisterService( audioProfileAttrTbl,
                                        GATT_NUM_ATTRS( audioProfileAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &as_servCBs );

  List_clearList(&streamOutQueue);

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      AS_registerProfileCBs
 *
 * @brief   Registers the profile callback function. Only call
 *          this function once.
 *
 * @param   profileCallback - pointer to profile callback functions.
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t AS_registerProfileCBs( AS_cb_t *profileCallback )
{
  if ( profileCallback )
  {
    as_profileCBs = profileCallback;

    return ( SUCCESS );
  }

  return ( INVALIDPARAMETER );
}

/*********************************************************************
 * @fn      AS_setParameter
 *
 * @brief   Set a Data Stream Service parameter.
 *
 * @param   param - Characteristic UUID
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 * @param   len - length of data to write
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t AS_setParameter(uint8 param, void *pValue, uint16 len)
{
  bStatus_t status = SUCCESS;

  // Verify input parameters
  if ( pValue == NULL )
  {
    return ( INVALIDPARAMETER );
  }

/*
  switch ( param )
  {
    case AUDIOPROFILE_AUDIO:
      status = AS_sendNotification( (uint8 *)pValue, len );
      break;

    default:
      status = INVALIDPARAMETER;
      break;
  }
*/

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      AS_writeAttrCB
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
 * @return  SUCCESS or stack call status
 */

static bStatus_t AS_writeAttrCB( uint16 connHandle,
                                        gattAttribute_t *pAttr,
                                        uint8 *pValue, uint16 len,
                                        uint16 offset, uint8 method )
{

  bStatus_t status = SUCCESS;
  //__asm__("BKPT");
  // Verify input parameters
  if ( pAttr == NULL || pValue == NULL )
  {
    return ( INVALIDPARAMETER );
  }

  /******************************************************/
  /****** Client Characteristic Configuration ***********/
  /******************************************************/

  if ( ! memcmp( pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len ) )
  {
     AS_cccUpdate_t *cccUpdate;
    // Allow only notifications
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY );

    // Notify profile
    if ( status == SUCCESS )
    {
      // This allocation will be free by bleapp_util
      cccUpdate = (AS_cccUpdate_t *)ICall_malloc( sizeof( AS_cccUpdate_t ) );
      if ( cccUpdate == NULL )
      {
        // Return error status
        return ( bleMemAllocError );
      }

      // Copy the data and send it to the profile
      cccUpdate->connHandle = connHandle;
      cccUpdate->value = BUILD_UINT16( pValue[0], pValue[1] );

      // Callback function to notify profile of change on the client characteristic configuration
      BLEAppUtil_invokeFunction( as_profileCBs->pfnOnCccUpdateCB, (char *)cccUpdate );
    }
  }

  /******************************************************/
  /*********** Data In Characteristic  ******************/
  /******************************************************/
  else if ( ! memcmp( pAttr->type.uuid, audioProfileWriteUUID, pAttr->type.len ) )
  {

    // Only notify profile if there is any data in the payload
    if ( len > 0  && as_profileCBs && as_profileCBs->pfnIncomingDataCB)
    {
      AS_dataIn_t *dataIn;

      // This allocation will be free by bleapp_util
      dataIn = (AS_dataIn_t *)ICall_malloc( sizeof( AS_dataIn_t ) + len);
      if ( dataIn == NULL )
      {
        // Return error status
        return ( bleMemAllocError );
      }

      // If allocation was successful,
      // Copy the data and send it to the profile
      if ( len > 0 )
      {
        memcpy( dataIn->pValue, pValue, len );
      }
      dataIn->connHandle = connHandle;
      dataIn->len = len;

      // Callback function to notify profile of change on the client characteristic configuration
      status = BLEAppUtil_invokeFunction( (void *)as_profileCBs->pfnIncomingDataCB, (char *)dataIn );
    }
  }

  // If we get here, that means you've forgotten to add an if clause for a
  // characteristic value attribute in the attribute table that has WRITE permissions.
  else
  {
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      AS_sendNotification_Control
 *
 * @brief   Transmits data over BLE notifications.
 *
 * @param   pValue - pointer to data to be written
 * @param   len - length of data to be written
 *
 * @return  SUCCESS, or stack call status
 */
int totalNotificationsAttempted = 0;
int notificationsFailed = 0;
int notificationsSent = 0;
int lengthOfNotification = 0;
int notificationAllocationsAttempted = 0;
int notificationAllocationsFailed = 0;
int notificationAllocationsSuccess = 0;
uint8_t *payloadNotification;
volatile uint16_t notiHandle = 0;

/*********************************************************************
 * @fn      AS_queueData
 *
 * @brief   Adds a new AudioStreamNode_t node to the data queue
 *
 * @param   node  - data node to add to the queue
 *
 * @return  SUCCESS, FAILURE or INVALIDPARAMETER
 */
static bStatus_t AS_queueData( AudioStreamNode_t *node )
{
    bStatus_t ret = SUCCESS;
    gattCharCfg_t *pItem = NULL;

    if (node != NULL)
    {
        // Find the correct CCCD
        int i;
        for ( i = 0; i < linkDBNumConns; i++ )
        {
            if (audioProfileControlConfig[i].connHandle == node->connHandle)
            {
                pItem = &(audioProfileControlConfig[i]);
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
 * @fn          AS_transmitNode
 *
 * @brief       Transmits as much as possible of a AudioStreamNode_t node
 *              over BLE notifications.
 *
 * @param       node - The node to send
 *
 * @return      SUCCESS, FAILURE, INVALIDPARAMETER, MSG_BUFFER_NOT_AVAIL,
 *              bleNotCOnnected, bleMemAllocError, blePending, bleInvaludMtuSize or
 *              bleTimeout
 */

static bStatus_t AS_transmitNode( AudioStreamNode_t* node)
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
        if ( allocLen > (connInfo.MTU - AS_NOTI_HDR_SIZE) )
        {
            allocLen = connInfo.MTU - AS_NOTI_HDR_SIZE;
        }

        noti.len = 0;
        noti.pValue = (uint8 *)GATT_bm_alloc( node->connHandle, ATT_HANDLE_VALUE_NOTI,
                                              allocLen, &noti.len );
        notificationAllocationsAttempted++;

        // If allocation was successful, copy out data out of the buffer and send it
        if (noti.pValue) {
            notificationAllocationsSuccess++;

            // Normal use
            memcpy(noti.pValue, (void *) ((uint8_t *) node->payload + node->offset), noti.len);

            // Known data test
//            memcpy(noti.pValue, (void *) ((uint8_t *) node->payload + node->offset), noti.len);


            lengthOfNotification = noti.len;
            payloadNotification = noti.pValue;

            // The outgoing data attribute offset is 4
            noti.handle = audioProfileAttrTbl[4].handle;

            ret = GATT_Notification( node->connHandle, &noti, FALSE );
            totalNotificationsAttempted++;

            // If unable to send the message, free allocated buffers and return
            if ( ret != SUCCESS )
            {
                GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
                notificationsFailed++;
            }
            else
            {
                // Increment node data offset
                node->offset += noti.len;
                notificationsSent++;
            }
        }
        else
        {
            // Unable to allocate space for a notification, return failure
            ret = bleMemAllocError;
            notificationAllocationsFailed++;
        }
    }

    return ret;
}

/*********************************************************************
 * @fn      AS_clearQueue
 *
 * @brief   Clears and free the allocated outgoing stream queue
 *
 * @param   None
 *
 * @return  None
 */
void AS_clearQueue()
{
    // Pop and free the whole queue
    while(!List_empty(&streamOutQueue))
    {
        AudioStreamNode_t *node = (AudioStreamNode_t *) List_get(&streamOutQueue);
        ICall_free(node);
    }
}

/*********************************************************************
 * @fn      AS_processStream
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
bStatus_t AS_processStream()
{
    bStatus_t ret = SUCCESS;

    // Send data starting from the list head
    AudioStreamNode_t *node = (AudioStreamNode_t *) List_get(&streamOutQueue);

    while ((ret == SUCCESS) && (node != NULL))
    {
        ret = AS_transmitNode(node);

        // Check that we really did send all data before freeing the node
        if ((node->len - node->offset) == 0)
        {
            ICall_free(node);
            // Move to next queue entry
            node = (AudioStreamNode_t *) List_get(&streamOutQueue);
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
 * @fn      AS_disconnectStream
 *
 * @brief   Disconnect the steam.
 *          Clear and free up the existing outgoing stream queue.
 *
 * @param   none
 *
 * @return  none
 */
void AS_disconnectStream()
{
    // Clear the outgoing stream queue
    AS_clearQueue();
}

/*********************************************************************
 * @fn      AS_setHeadroomLimit
 *
 * @brief   Sets the limit on how much heap that needs to be available
 *          following a memory allocation.
 *
 * @param   minHeapHeadRoom - Smallest amount of free heap following
 *          an memory allocation.
 *
 * @return  none
 */
void AS_setHeadroomLimit(uint16_t minHeapHeadroom)
{
    // Store the minimal heap headroom limit
    heapHeadroom = minHeapHeadroom;
}

/*********************************************************************
 * @fn      AS_allocateWithHeadroom
 *
 * @brief   Checks if there will be enough free heap left following
 *          a memory allocation. If there is enough heap, it will allocate
 *          the memory.
 *
 * @param   allocSize - number of bytes to be allocated
 *
 * @return  none
 */
int32_t totalFreeSizeTest = 0;
uint16_t allocSizeTest= 0;
void* AS_allocateWithHeadroom(uint16_t allocSize)
{
    void *allocatedBuffer = NULL;
    ICall_heapStats_t  stats;
    ICall_CSState key;
    allocSizeTest = allocSize;

    // Perform this inside a critical section
    key = ICall_enterCriticalSection();

    // Get the current free heap
    ICall_getHeapStats(&stats);
    totalFreeSizeTest = stats.totalFreeSize;
//    if (((uint16_t) allocSize) < ((int32_t)(stats.totalFreeSize - heapHeadroom)))
//    {
        allocatedBuffer = ICall_malloc(allocSize);
//    }

    // Leave the critical section
    ICall_leaveCriticalSection(key);

    return allocatedBuffer;
}

/*********************************************************************
 * @fn      AS_sendData
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

uint8_t nodeLength = 0;
uint8_t* nodePayloadQueued;
int queueSuccess = 0;
int queueFailure = 0;

bStatus_t AS_sendData(uint16_t connHandle, void *data, uint16_t len , uint8_t char_type)
{

    bStatus_t ret = SUCCESS;
    attHandleValueNoti_t noti;
    linkDBInfo_t connInfo;

        uint16_t allocLen = len;

        noti.len = 0;
        noti.pValue = (uint8 *)GATT_bm_alloc( connHandle, ATT_HANDLE_VALUE_NOTI,
                                              allocLen, &noti.len );
        notificationAllocationsAttempted++;

        // If allocation was successful, copy out data out of the buffer and send it
        if (noti.pValue)
        {
            notificationAllocationsSuccess++;

            memcpy(noti.pValue, (void *) ((uint8_t *) data), noti.len);

            lengthOfNotification = noti.len;
            payloadNotification = noti.pValue;

            if (char_type == 1) {
                noti.handle = (GATTServApp_FindAttr(audioProfileAttrTbl, GATT_NUM_ATTRS(audioProfileAttrTbl), audioProfileRead))->handle;
            }
            else {
                noti.handle = (GATTServApp_FindAttr(audioProfileAttrTbl, GATT_NUM_ATTRS(audioProfileAttrTbl), audioProfileControl))->handle;
            }

            ret = GATT_Notification( connHandle, &noti, FALSE );
            totalNotificationsAttempted++;

            // If unable to send the message, free allocated buffers and return
            if ( ret != SUCCESS )
            {
                GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
                notificationsFailed++;
            }
            else
            {
                notificationsSent++;
            }
        }
        else
        {
            // Unable to allocate space for a notification, return failure
            ret = bleMemAllocError;
            notificationAllocationsFailed++;
        }

    return ret;

}

static bStatus_t AS_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
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
#endif
/*********************************************************************
*********************************************************************/
