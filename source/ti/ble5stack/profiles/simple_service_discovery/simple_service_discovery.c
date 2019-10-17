/**********************************************************************************************
 * Filename:       simple_service_discovery.c
 *
 * Description:    This file contains the implementation of the simple service discovery API.
 *
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
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

#include "simple_service_discovery.h"
#include "icall_ble_api.h"

/*********************************************************************
 * DEFINES
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * ENUMS
 */

// Discovery states
typedef enum
{
  BLE_DISC_STATE_IDLE,
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
} discStates_t;

// Parsing info response states
typedef enum
{
  BLE_INFO_RSP_IDLE,                 // Looking for characteristic UUID
  BLE_INFO_RSP_DESC,                 // Looking for char. descriptor UUID
  BLE_INFO_RSP_CCCD,                 // Looking for client char. configuration UUID
} parseState_t;

/*********************************************************************
* GLOBAL VARIABLES
*/

/*********************************************************************
 * LOCAL VARIABLES
 */

// State of the discovery process
static discStates_t discoveryState = BLE_DISC_STATE_IDLE;

// Parsing info response state
static parseState_t findInforRspState = BLE_INFO_RSP_IDLE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleServiceDiscovery_processFindInfoRsp(attFindInfoRsp_t rsp,
                                                      simpleService_t *service);

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
 * @fn      SimpleServiceDiscovery_discoverService
 *
 * @brief   Perform discovery of the given simpleService_t structure.
 *          This function is to be called inside the GATT discovery event
 *          callback to process the service discovery.
 *
 * @param   connHandle    - connection handle
 *          entity        - ICall entity of the calling task
 *          *service      - pointer to the service struct
 *          *pMsg         - pointer to the received gattMsgEvent_t
 *
 * @return  SIMPLE_DISCOVERY_SUCCESSFUL, SIMPLE_DISCOVERY_FINDING_SERVICE,
 *          SIMPLE_DISCOVERY_FINDING_CHAR or SIMPLE_DISCOVERY_UNSUCCESSFUL.
 */
uint32_t SimpleServiceDiscovery_discoverService(uint16_t connHandle, ICall_EntityID entity,
                                                simpleService_t *service, gattMsgEvent_t *pMsg)
{
    uint32_t retVal = 0;

    switch (discoveryState) {
    case BLE_DISC_STATE_IDLE:
    {
        discoveryState = BLE_DISC_STATE_SVC;

        // Discovery the service
        GATT_DiscPrimaryServiceByUUID(connHandle, service->uuid.uuid, service->uuid.len,
                                      entity);

        retVal = SIMPLE_DISCOVERY_FINDING_SERVICE;
        break;
    }
    case BLE_DISC_STATE_SVC:
    {
        // Service found, store handles
        if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
            pMsg->msg.findByTypeValueRsp.numInfo > 0)
        {
          service->startHandle = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
          service->endHandle = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        }

        // If procedure complete
        if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
             (pMsg->hdr.status == bleProcedureComplete))  ||
            (pMsg->method == ATT_ERROR_RSP))
        {
          if (service->startHandle != 0)
          {
            discoveryState = BLE_DISC_STATE_CHAR;
            GATT_DiscAllCharDescs(connHandle, service->startHandle, service->endHandle, entity);
            retVal = SIMPLE_DISCOVERY_FINDING_CHAR;
          }
          else
          {
            discoveryState = BLE_DISC_STATE_IDLE;
            retVal = SIMPLE_DISCOVERY_UNSUCCESSFUL;
          }
        }
        break;
    }
    case BLE_DISC_STATE_CHAR:
    {
        // Characteristic found, store handle
        if (pMsg->method == ATT_FIND_INFO_RSP)
        {

            if (pMsg->msg.findInfoRsp.numInfo > 0)
            {
              SimpleServiceDiscovery_processFindInfoRsp(pMsg->msg.findInfoRsp, service);
            }

            if (pMsg->hdr.status == bleProcedureComplete)
            {
              discoveryState = BLE_DISC_STATE_IDLE;
              retVal = SIMPLE_DISCOVERY_SUCCESSFUL;
            }
        }
        break;
    }
    default:
        break;
    }

    return retVal;
}

/*********************************************************************
 * @fn      SimpleSerialBridgeClient_processFindInfoRsp
 *
 * @brief   Process a ATT findInfoRsp and populates a simple service struct
 *
 * @param   rsp     - findInfoRsp msg.
 * @param   service - service struct to populate.
 *
 * @return  void
 */
static void SimpleServiceDiscovery_processFindInfoRsp(attFindInfoRsp_t rsp, simpleService_t *service)
{
    static uint8_t lastCharIndex;
    uint8_t i = 0;
    uint8_t uuidLength = (rsp.format == ATT_HANDLE_BT_UUID_TYPE) ? ATT_BT_UUID_SIZE : ATT_UUID_SIZE;
    uint8_t *pPair     = rsp.pInfo;
    uint8_t pairSize   = 2 + uuidLength;

    while(pPair != (rsp.pInfo + (pairSize * rsp.numInfo)))
    {
        switch(findInforRspState) {
        case BLE_INFO_RSP_IDLE:
        {
            // We are looking for a characteristic declaration
            if (!memcmp(characterUUID, &pPair[2], ATT_BT_UUID_SIZE))
            {
                // We found it, move to state 2
                findInforRspState = BLE_INFO_RSP_DESC;
            }

            break;
        }
        // We look for specific characteristics
        case BLE_INFO_RSP_DESC:
        {

            for(i = 0; i < service->numChars; i++)
            {
                // Is it this one?
                if ((service->chars[i].uuid.len == uuidLength) &&
                    (!memcmp(service->chars[i].uuid.uuid, &pPair[2], uuidLength)))
                {
                    // We found it, save the handle
                    service->chars[i].handle = BUILD_UINT16(pPair[0],
                                                            pPair[1]);
                    lastCharIndex = i;
                    // Look for a cccd
                    findInforRspState = BLE_INFO_RSP_CCCD;

                    break;
                }
            }

            break;
        }
        case BLE_INFO_RSP_CCCD:
        {
            // Is there a CCCD belonging to this characteristic?
            if (!memcmp(clientCharCfgUUID, &pPair[2], ATT_BT_UUID_SIZE))
            {
                // We found it, save the handle
                service->chars[lastCharIndex].cccdHandle = BUILD_UINT16(pPair[0],
                                                           pPair[1]);
                // Go back to looking for a new characteristic
                findInforRspState = BLE_INFO_RSP_IDLE;
            }
            // Found new characteristic!
            else if (!memcmp(characterUUID, &pPair[2], ATT_BT_UUID_SIZE))
            {
                findInforRspState = BLE_INFO_RSP_DESC;
            }

            break;
        }
        default:
            break;
        }

        // Move pointer to next pair
        pPair += pairSize;
    }
}
