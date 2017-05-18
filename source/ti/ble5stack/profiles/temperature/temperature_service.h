/*
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 */
/*!
 *  @file        Temperature_Service.h
 *
 *  @brief       Custom TI GATT service to read temperature sensor data.
 *
 *  This service defines the following Characterisics:
 *
 *  Characteristic Name | UUID   | Length | Purpose
 *  ------------------- | ------ | ------ | -------
 *  Data                | 0xAA01 | 4      | Object[0:7], Object[8:15], Ambience[0:7], Ambience[8:15]
 *  Config              | 0xAA02 | 1      | Write 0x01 to enable data collection, 0x00 to disable.
 *  Period              | 0xAA03 | 1      | Resolution 10 ms. Range 300 ms (0x1E) to 2.55 sec (0xFF). Default 1 second (0x64)
 */

#ifndef _TEMPERATURE_SERVICE_H_
#define _TEMPERATURE_SERVICE_H_

#include "bcomdef.h"

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
 * TYPEDEFS
 */
typedef struct Temperature_Service_Data_t
{
  uint8_t objectLowByte;
  uint8_t objectHighByte;
  uint8_t ambienceLowByte;
  uint8_t ambienceHighByte;
} Temperature_Service_Data;

/*********************************************************************
* CONSTANTS
*/
/*! UUID for the GATT Primary Service Declaration
 */
#define TEMPERATURE_SERVICE_SERV_UUID   0xAA00

//  Characteristic defines
#define TEMPERATURE_SERVICE_DATA        0
#define TEMPERATURE_SERVICE_DATA_UUID   0xAA01
#define TEMPERATURE_SERVICE_DATA_LEN    sizeof(Temperature_Service_Data)

//  Characteristic defines
#define TEMPERATURE_SERVICE_CONFIG      1
#define TEMPERATURE_SERVICE_CONFIG_UUID 0xAA02
#define TEMPERATURE_SERVICE_CONFIG_LEN  1

//  Characteristic defines
#define TEMPERATURE_SERVICE_PERIOD      2
#define TEMPERATURE_SERVICE_PERIOD_UUID 0xAA03
#define TEMPERATURE_SERVICE_PERIOD_LEN  1

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

/*!
 *  Temperature_Service callback function prototype
 *
 *  Callback occur when characteristic values have changed
 *
 *  @param paramID Profile parameter ID
 */
typedef void (*Temperature_ServiceChange_t)( uint8 paramID );

/*!
 * Data structure of callback functions.
 */
typedef struct
{
    // Called when characteristic value changes
    Temperature_ServiceChange_t        pfnChangeCb;
} Temperature_ServiceCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*!
 *  @brief  Initialize the Temperature Service
 *
 *  Initializes the Temperature_Service service by registering GATT attributes
 *  with the GATT server.
 *
 *  @return Generic Status Return values defined by bcomdef.h
 */
extern bStatus_t Temperature_Service_AddService( void );

/*!
 *  @brief  Registers the application callback function.
 *
 *  @warn   Only call this function once.
 *
 *  @param  appCallbacks Pointer to application callbacks.
 *
 *  @return Generic Status Return values defined by bcomdef.h
 */
extern bStatus_t Temperature_Service_RegisterAppCBs(
    Temperature_ServiceCBs_t *appCallbacks );

/*!
 *  @brief  Set a Temperature_Service parameter.
 *
 *  @param param    Profile parameter ID
 *
 *  @param len      length of data to write
 *
 *  @param value    pointer to data to write.  This is dependent on the
 *                  parameter ID and WILL be cast to the appropriate data type
 *                  (example: data type of uint16 will be cast to uint16
 *                  pointer).
 *
 *  @return Generic Status Return values defined by bcomdef.h
 */
extern bStatus_t Temperature_Service_SetParameter( uint8 param, uint8 len,
    void *value );

/*
 *  @brief Get a Temperature_Service parameter.
 *
 *  @param param    Profile parameter ID
 *
 *  @param value    pointer to data to write.  This is dependent on the
 *                  parameter ID and WILL be cast to the appropriate data type
 *                  (example: data type of uint16 will be cast to uint16
 *                  pointer).
 *
 *  @return Generic Status Return values defined by bcomdef.h
 */
extern bStatus_t Temperature_Service_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _TEMPERATURE_SERVICE_H_ */
