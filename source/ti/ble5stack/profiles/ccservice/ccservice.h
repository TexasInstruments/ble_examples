/******************************************************************************

 @file  ccservice.h

 @brief This file contains the proprietary Connection Control Service
        Prototypes.

 Group: CMCU, LPRF
 Target Device: CC2652

 ******************************************************************************
 
 Copyright (c) 2010-2021, Texas Instruments Incorporated
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

 *****************************************************************************/

#ifndef CCSERVICE_H
#define CCSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Service Parameters
#define CCSERVICE_CHAR1                 0  // R  connection parameters (6 bytes)
#define CCSERVICE_CHAR2                 1  // W  requested connection parameters
#define CCSERVICE_CHAR3                 2  // W disconnect request

// Service UUID
#define CCSERVICE_SERV_UUID             0xCCC0
#define CCSERVICE_CHAR1_UUID            0xCCC1
#define CCSERVICE_CHAR2_UUID            0xCCC2
#define CCSERVICE_CHAR3_UUID            0xCCC3

// Length of Characteristics in bytes
#define CCSERVICE_CHAR1_LEN             6
#define CCSERVICE_CHAR2_LEN             8
#define CCSERVICE_CHAR3_LEN             1

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*ccChange_t)(uint8_t paramID);

typedef struct
{
  ccChange_t pfnCcChange;  // Called when characteristic value changes
} ccCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * CcService_addService- Initializes the Connection Control Profile service
 *          by registering GATT attributes with the GATT server.
 *
 */

extern bStatus_t CcService_addService(void);

/*
 * CcService_registerAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t CcService_registerAppCBs(ccCBs_t *appCallbacks);

/*
 * CcService_setParameter - Set a Connection Control Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t CcService_setParameter(uint8_t param, uint8_t len, void *value);

/*
 * CcService_getParameter - Get a Connection Control Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to read.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t CcService_getParameter(uint8_t param, void *value);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* CCSERVICE_H */
