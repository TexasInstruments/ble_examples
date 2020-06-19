/**********************************************************************************************
 * Filename:       simple_service_discovery.h
 *
 * Description:    This file contains the simple service discovery definitions and
 *                 prototypes.
 *
 * Copyright (c) 2018-2020, Texas Instruments Incorporated
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


#ifndef _SIMPLESERVICEDISCOVERY_H_
#define _SIMPLESERVICEDISCOVERY_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include <stdint.h>
#include "icall_ble_api.h"

/*********************************************************************
 * DEFINES
 */

#define SIMPLE_DISCOVERY_FINDING_SERVICE    (0)
#define SIMPLE_DISCOVERY_FINDING_CHAR       (1)
#define SIMPLE_DISCOVERY_UNSUCCESSFUL       (2)
#define SIMPLE_DISCOVERY_SUCCESSFUL         (3)

/*********************************************************************
* CONSTANTS
*/

/*********************************************************************
 * TYPEDEFS
 */

// Simple service and characteristic types
typedef struct
{
  // Value UUID
  gattAttrType_t uuid;
  uint16_t handle;
  uint16_t cccdHandle;
} simpleServiceChar_t;

typedef struct
{
  // Service UUID
  gattAttrType_t  uuid;
  uint16_t startHandle;
  uint16_t endHandle;

  uint8_t numChars;
  simpleServiceChar_t chars[];

} simpleService_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
 */

/*
 * SimpleServiceDiscovery_discoverService - Perform discovery of the given simpleService_t structure.
 *                                          This function is to be called inside the GATT discovery event
 *                                          callback to process the service discovery.
 *    connHandle    - connection handle
 *    entity        - ICall entity of the calling task
 *    *service      - pointer to the service struct
 *    *pMsg         - pointer to the received gattMsgEvent_t
 *
 *    returns       - SIMPLE_DISCOVERY_SUCCESSFUL, SIMPLE_DISCOVERY_FINDING_SERVICE,
 *                    SIMPLE_DISCOVERY_FINDING_CHAR or SIMPLE_DISCOVERY_UNSUCCESSFUL.
 */
extern uint32_t SimpleServiceDiscovery_discoverService(uint16_t connHandle, ICall_EntityID entity,
                                                       simpleService_t *service, gattMsgEvent_t *pMsg);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _SIMPLESERVICEDISCOVERY_H_ */
