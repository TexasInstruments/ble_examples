/**********************************************************************************************
 * Filename:       Throughput_Service.h
 *
 * Description:    This file contains the Throughput_Service service definitions and
 *                 prototypes.
 *
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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


#ifndef _THROUGHPUT_SERVICE_H_
#define _THROUGHPUT_SERVICE_H_

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

/*********************************************************************
* CONSTANTS
*/
// Service UUID
#define THROUGHPUT_SERVICE_SERV_UUID 0x1234

//  Characteristic defines
#define THROUGHPUT_SERVICE_UPDATE_PDU      0
#define THROUGHPUT_SERVICE_UPDATE_PDU_UUID 0x1235
#define THROUGHPUT_SERVICE_UPDATE_PDU_LEN  1

//  Characteristic defines
#define THROUGHPUT_SERVICE_UPDATE_PHY      1
#define THROUGHPUT_SERVICE_UPDATE_PHY_UUID 0x1236
#define THROUGHPUT_SERVICE_UPDATE_PHY_LEN  1

//  Characteristic defines
#define THROUGHPUT_SERVICE_TOGGLE_THROUGHPUT      2
#define THROUGHPUT_SERVICE_TOGGLE_THROUGHPUT_UUID 0x1237
#define THROUGHPUT_SERVICE_TOGGLE_THROUGHPUT_LEN  1

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
typedef void (*Throughput_ServiceChange_t)( uint8 paramID );

typedef struct
{
  Throughput_ServiceChange_t        pfnChangeCb;  // Called when characteristic value changes
} Throughput_ServiceCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * Throughput_Service_AddService- Initializes the Throughput_Service service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t Throughput_Service_AddService( void );

/*
 * Throughput_Service_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Throughput_Service_RegisterAppCBs( Throughput_ServiceCBs_t *appCallbacks );

/*
 * Throughput_Service_SetParameter - Set a Throughput_Service parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Throughput_Service_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Throughput_Service_GetParameter - Get a Throughput_Service parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Throughput_Service_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _THROUGHPUT_SERVICE_H_ */
