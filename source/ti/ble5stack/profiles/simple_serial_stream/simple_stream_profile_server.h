/**********************************************************************************************
 * Filename:       simple_stream_profile_server.h
 *
 * Description:    This file contains the Simple Stream Server service definitions and
 *                 prototypes.
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


#ifndef _SIMPLESTREAMSERVER_H_
#define _SIMPLESTREAMSERVER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <ti/drivers/utils/List.h>
#include "icall_ble_api.h"

/*********************************************************************
* CONSTANTS
*/
// Service UUID
#define SIMPLESTREAMSERVER_SERV_UUID 0xC0C0

//  Characteristic defines
#define SIMPLESTREAMSERVER_DATAIN_ID   0
#define SIMPLESTREAMSERVER_DATAIN_UUID 0xC0C1
#define SIMPLESTREAMSERVER_DATAIN_LEN  1

//  Characteristic defines
#define SIMPLESTREAMSERVER_DATAOUT_ID   1
#define SIMPLESTREAMSERVER_DATAOUT_UUID 0xC0C2
#define SIMPLESTREAMSERVER_DATAOUT_LEN  1

// Profile UUIDs
extern const uint8_t SimpleStreamServerUUID[ATT_UUID_SIZE];
extern const uint8_t SimpleStreamServer_DataInUUID[ATT_UUID_SIZE];
extern const uint8_t SimpleStreamServer_DataOutUUID[ATT_UUID_SIZE];

/*********************************************************************
 * TYPEDEFS
 */

// Data structure used to store outgoing data
typedef struct
{
    List_Elem elem;
    uint16_t connHandle;
    uint16_t offset;
    uint16_t len;
    uint8_t payload[];
} SimpleStreamNode_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback to indicate client characteristic configuration has been updated
typedef void (*SimpleStreamServerCCCUpdate_t)(uint16_t value);

// Callback when new data is received
typedef void (*SimpleStreamServerIncomingData_t)(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);

typedef struct
{
    SimpleStreamServerCCCUpdate_t           pfnCccUpdateCb;
    SimpleStreamServerIncomingData_t        pfnIncomingDataCb;  // Called when receiving data
} SimpleStreamServerCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */

/*
 * SimpleStreamServer_AddService - Initializes the SimpleStreamServer service by registering
 *                                 GATT attributes with the GATT server.
 *
 */
extern bStatus_t SimpleStreamServer_AddService( uint32_t rspTaskId);

/*
 * SimpleStreamServer_RegisterAppCBs - Registers the application callback function.
 *                                     Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t SimpleStreamServer_RegisterAppCBs( SimpleStreamServerCBs_t *appCallbacks );

/*
 * SimpleStreamServer_sendData - Put the data into the outgoing stream queue and sends as
 *                               much as possible using BLE notifications.
 *
 *    *data - pointer to data buffer
 *    len     - size of the data buffer
 */
extern bStatus_t SimpleStreamServer_sendData(uint16_t connHandle, void *data, uint16_t len);

/*
 * SimpleStreamServer_processStream - Sends out as much as possible from the outgoing stream
 *                                    queue using BLE notifications.
 */
extern bStatus_t SimpleStreamServer_processStream();

/*
 * SimpleStreamServer_disconnectStream - Disconnect the steam.
 *                                       Clear and free up the existing outgoing stream queue.
 */
extern void      SimpleStreamServer_disconnectStream();

/*
 * SimpleStreamServer_setHeadroomLimit - Sets the limit on how much heap that needs to be available
 *                                       following a memory allocation.
 */
extern void     SimpleStreamServer_setHeadroomLimit(uint16_t minHeapHeadroom);

/*
 * SimpleStreamServer_allocateWithHeadroom - Checks if there will be enough free heap left
 *                                           following a memory allocation. If there is
 *                                           enough heap, it will allocate the memory.
 */
extern void*    SimpleStreamServer_allocateWithHeadroom(uint16_t allocSize);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _SIMPLESTREAMSERVER_H_ */
