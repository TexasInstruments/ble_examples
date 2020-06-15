/******************************************************************************

 @file       heartrateservice.h

 @brief This file contains the Heart Rate service definitions and prototypes
        prototypes.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2012-2017, Texas Instruments Incorporated
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
 Release Name: simplelink_cc2640r2_sdk_01_50_00_58
 Release Date: 2017-10-17 18:09:51
 *****************************************************************************/

#ifndef HEARTRATESERVICE_H
#define HEARTRATESERVICE_H

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


// Heart Rate Service Parameters
#define HEARTRATE_MEAS                      0
#define HEARTRATE_MEAS_CHAR_CFG             1
#define HEARTRATE_SENS_LOC                  2
#define HEARTRATE_COMMAND                   3

// Maximum length of heart rate measurement characteristic
#define HEARTRATE_MEAS_MAX                  (ATT_MTU_SIZE -5)

// Values for flags
#define HEARTRATE_FLAGS_FORMAT_UINT16       0x01
#define HEARTRATE_FLAGS_CONTACT_NOT_SUP     0x00
#define HEARTRATE_FLAGS_CONTACT_NOT_DET     0x04
#define HEARTRATE_FLAGS_CONTACT_DET         0x06
#define HEARTRATE_FLAGS_ENERGY_EXP          0x08
#define HEARTRATE_FLAGS_RR                  0x10

// Values for sensor location
#define HEARTRATE_SENS_LOC_OTHER            0x00
#define HEARTRATE_SENS_LOC_CHEST            0x01
#define HEARTRATE_SENS_LOC_WRIST            0x02
#define HEARTRATE_SENS_LOC_FINGER           0x03
#define HEARTRATE_SENS_LOC_HAND             0x04
#define HEARTRATE_SENS_LOC_EARLOBE          0x05
#define HEARTRATE_SENS_LOC_FOOT             0x06

// Value for command characteristic
#define HEARTRATE_COMMAND_ENERGY_EXP        0x01

// ATT Error code
// Control point value not supported
#define HEARTRATE_ERR_NOT_SUP               0x80

// Heart Rate Service bit fields
#define HEARTRATE_SERVICE                   0x00000001

// Callback events
#define HEARTRATE_MEAS_NOTI_ENABLED         1
#define HEARTRATE_MEAS_NOTI_DISABLED        2
#define HEARTRATE_COMMAND_SET               3

/*********************************************************************
 * TYPEDEFS
 */

// Heart Rate Service callback function
typedef void (*heartRateServiceCB_t)(uint8 event);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS
 */

/*
 * HeartRate_AddService- Initializes the Heart Rate service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t HeartRate_AddService(uint32 services);

/*
 * HeartRate_Register - Register a callback function with the
 *          Heart Rate Service
 *
 * @param   pfnServiceCB - Callback function.
 */

extern void HeartRate_Register(heartRateServiceCB_t pfnServiceCB);

/*
 * HeartRate_SetParameter - Set a Heart Rate parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t HeartRate_SetParameter(uint8 param, uint8 len, void *value);

/*
 * HeartRate_GetParameter - Get a Heart Rate parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t HeartRate_GetParameter(uint8 param, void *value);

/*********************************************************************
 * @fn          HeartRate_MeasNotify
 *
 * @brief       Send a notification containing a heart rate
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t HeartRate_MeasNotify(uint16 connHandle,
                                      attHandleValueNoti_t *pNoti);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HEARTRATESERVICE_H */
