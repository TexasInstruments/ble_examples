/******************************************************************************

 @file  bpservice.h

 @brief This file contains the BloodPressure service definitions and prototypes
        prototypes.

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2011-2020, Texas Instruments Incorporated
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

#ifndef BLOODPRESSURESERVICE_H
#define BLOODPRESSURESERVICE_H

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

// BloodPressure Service Parameters
#define BLOODPRESSURE_MEAS                      0
#define BLOODPRESSURE_MEAS_CHAR_CFG             1
#define BLOODPRESSURE_IMEAS_CHAR_CFG            2
#define BLOODPRESSURE_TIMESTAMP                 3
#define BLOODPRESSURE_PULSE                     4
#define BLOODPRESSURE_INTERVAL                  5

// Length of measurements
#define BLOODPRESSURE_TIMESTAMP_LEN             7 //length of timestamp
#define BLOODPRESSURE_TIME_LEN                  7 //length of timestamp
#define BLOODPRESSURE_INTERVAL_LEN              1

// Maximum length of blood pressure measurement characteristic
#define BLOODPRESSURE_MEAS_MAX                  (ATT_MTU_SIZE -5)

// Values for flags
#define BLOODPRESSURE_FLAGS_MMHG                0x00
#define BLOODPRESSURE_FLAGS_KPA                 0x01
#define BLOODPRESSURE_FLAGS_TIMESTAMP           0x02
#define BLOODPRESSURE_FLAGS_PULSE               0x04
#define BLOODPRESSURE_FLAGS_USER                0x08
#define BLOODPRESSURE_FLAGS_STATUS              0x10


// Values for sensor location
#define BLOODPRESSURE_SITE_ARMPIT               0x01
#define BLOODPRESSURE_SITE_BODY                 0x02
#define BLOODPRESSURE_SITE_EAR                  0x03
#define BLOODPRESSURE_SITE_FINGER               0x04
#define BLOODPRESSURE_SITE_GASTRO               0x05
#define BLOODPRESSURE_SITE_MOUTH                0x06
#define BLOODPRESSURE_SITE_RECTUM               0x07
#define BLOODPRESSURE_SITE_TOE                  0x08
#define BLOODPRESSURE_SITE_TYMPNUM              0x09

// BloodPressure Service bit fields
#define BLOODPRESSURE_SERVICE                   0x00000001

// Callback events
#define BLOODPRESSURE_MEAS_NOTI_ENABLED         1
#define BLOODPRESSURE_MEAS_NOTI_DISABLED        2
#define BLOODPRESSURE_IMEAS_NOTI_ENABLED        3
#define BLOODPRESSURE_IMEAS_NOTI_DISABLED       4
#define BLOODPRESSURE_TIME_SET                  5

/*********************************************************************
 * TYPEDEFS
 */

// BloodPressure Service callback function
typedef void (*bloodPressureServiceCB_t)(uint8 event);

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
 * @fn      BloodPressure_AddService
 *
 * @brief   Initializes the BloodPressure   service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
extern bStatus_t BloodPressure_AddService(uint32 services);

/*
 * @fn      BloodPressure_Register
 *
 * @brief   Register a callback function with the BloodPressure Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void BloodPressure_Register(bloodPressureServiceCB_t pfnServiceCB);

/*
 * BloodPressure_SetParameter - Set a BloodPressure parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len   - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t BloodPressure_SetParameter(uint8 param, uint8 len,
                                            void *value);

/*
 * BloodPressure_GetParameter - Get a BloodPressure parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t BloodPressure_GetParameter(uint8 param, void *value);

/*********************************************************************
 * @fn          BloodPressure_MeasIndicate
 *
 * @brief       Send a notification containing a blood pressure
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti      - pointer to notification structure
 * @param       taskId     - calling task's Id.
 *
 * @return      Success or Failure
 */
extern bStatus_t BloodPressure_MeasIndicate(uint16 connHandle,
                                            attHandleValueInd_t *pNoti,
                                            uint8 taskId);

/*********************************************************************
 * @fn          BloodPressure_IMeasNotify
 *
 * @brief       Send a notification containing a blood pressure
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti      - pointer to notification structure
 * @param       taskId     - calling task's Id.
 *
 * @return      Success or Failure
 */
extern bStatus_t BloodPressure_IMeasNotify(uint16 connHandle,
                                           attHandleValueNoti_t *pNoti,
                                           uint8 taskId);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BLOODPRESSURESERVICE_H */
