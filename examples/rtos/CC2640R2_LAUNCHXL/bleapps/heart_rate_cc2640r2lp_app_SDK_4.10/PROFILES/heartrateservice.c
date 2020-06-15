/******************************************************************************

 @file       heartrateservice.c

 @brief This file contains the Heart Rate sample service for use with the
        Heart Rate sample application.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2011-2017, Texas Instruments Incorporated
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

/*********************************************************************
 * INCLUDES
 */
#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "heartrateservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Position of heart rate measurement value in attribute array
#define HEARTRATE_MEAS_VALUE_POS            2

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Heart rate service
CONST uint8 heartRateServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HEARTRATE_SERV_UUID), HI_UINT16(HEARTRATE_SERV_UUID)
};

// Heart rate measurement characteristic
CONST uint8 heartRateMeasUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HEARTRATE_MEAS_UUID), HI_UINT16(HEARTRATE_MEAS_UUID)
};

// Sensor location characteristic
CONST uint8 heartRateSensLocUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BODY_SENSOR_LOC_UUID), HI_UINT16(BODY_SENSOR_LOC_UUID)
};

// Command characteristic
CONST uint8 heartRateCommandUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HEARTRATE_CTRL_PT_UUID), HI_UINT16(HEARTRATE_CTRL_PT_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static heartRateServiceCB_t heartRateServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// Heart Rate Service attribute
static CONST gattAttrType_t heartRateService = {ATT_BT_UUID_SIZE,
                                                heartRateServUUID};

// Heart Rate Measurement Characteristic
// Note characteristic value is not stored here
static uint8 heartRateMeasProps = GATT_PROP_NOTIFY;
static uint8 heartRateMeas = 0;
static gattCharCfg_t *heartRateMeasClientCharCfg;

// Sensor Location Characteristic
static uint8 heartRateSensLocProps = GATT_PROP_READ;
static uint8 heartRateSensLoc = 0;

// Command Characteristic
static uint8 heartRateCommandProps = GATT_PROP_WRITE;
static uint8 heartRateCommand = 0;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t heartRateAttrTbl[] =
{
  // Heart Rate Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&heartRateService                /* pValue */
  },

    // Heart Rate Measurement Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &heartRateMeasProps
    },

      // Heart Rate Measurement Value
      {
        { ATT_BT_UUID_SIZE, heartRateMeasUUID },
        0,
        0,
        &heartRateMeas
      },

      // Heart Rate Measurement Client Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) &heartRateMeasClientCharCfg
      },

    // Sensor Location Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &heartRateSensLocProps
    },

      // Sensor Location Value
      {
        { ATT_BT_UUID_SIZE, heartRateSensLocUUID },
        GATT_PERMIT_READ,
        0,
        &heartRateSensLoc
      },

    // Command Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &heartRateCommandProps
    },

      // Command Value
      {
        { ATT_BT_UUID_SIZE, heartRateCommandUUID },
        GATT_PERMIT_WRITE,
        0,
        &heartRateCommand
      }
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t heartRate_ReadAttrCB(uint16_t connHandle,
                                      gattAttribute_t *pAttr, uint8_t *pValue,
                                      uint16_t *pLen, uint16_t offset,
                                      uint16_t maxLen, uint8_t method);
static bStatus_t heartRate_WriteAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr, uint8_t *pValue,
                                       uint16_t len, uint16_t offset,
                                       uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Heart Rate Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t heartRateCBs =
{
  heartRate_ReadAttrCB,  // Read callback function pointer
  heartRate_WriteAttrCB, // Write callback function pointer
  NULL                   // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HeartRate_AddService
 *
 * @brief   Initializes the Heart Rate service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t HeartRate_AddService(uint32 services)
{
  uint8 status;

  // Allocate Client Characteristic Configuration table
  heartRateMeasClientCharCfg = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                              linkDBNumConns );
  if ( heartRateMeasClientCharCfg == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes.
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, heartRateMeasClientCharCfg);

  if (services & HEARTRATE_SERVICE)
  {
    // Register GATT attribute list and CBs with GATT Server App.
    status = GATTServApp_RegisterService(heartRateAttrTbl,
                                         GATT_NUM_ATTRS(heartRateAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &heartRateCBs);
  }
  else
  {
    status = SUCCESS;
  }

  return (status);
}

/*********************************************************************
 * @fn      HeartRate_Register
 *
 * @brief   Register a callback function with the Heart Rate Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void HeartRate_Register(heartRateServiceCB_t pfnServiceCB)
{
  heartRateServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      HeartRate_SetParameter
 *
 * @brief   Set a Heart Rate parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t HeartRate_SetParameter(uint8 param, uint8 len, void *value)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
     case HEARTRATE_MEAS_CHAR_CFG:
      // Need connection handle
      //heartRateMeasClientCharCfg.value = *((uint16*)value);
      break;

    case HEARTRATE_SENS_LOC:
      heartRateSensLoc = *((uint8*)value);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      HeartRate_GetParameter
 *
 * @brief   Get a Heart Rate parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t HeartRate_GetParameter(uint8 param, void *value)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
    case HEARTRATE_MEAS_CHAR_CFG:
      // Need connection handle
      //*((uint16*)value) = heartRateMeasClientCharCfg.value;
      break;

    case HEARTRATE_SENS_LOC:
      *((uint8*)value) = heartRateSensLoc;
      break;

    case HEARTRATE_COMMAND:
      *((uint8*)value) = heartRateCommand;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

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
bStatus_t HeartRate_MeasNotify(uint16 connHandle, attHandleValueNoti_t *pNoti)
{
  uint16 value = GATTServApp_ReadCharCfg(connHandle,
                                         heartRateMeasClientCharCfg);

  // If notifications enabled
  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    // Set the handle.
    pNoti->handle = heartRateAttrTbl[HEARTRATE_MEAS_VALUE_POS].handle;

    // Send the notification.
    return GATT_Notification(connHandle, pNoti, FALSE);
  }

  return bleIncorrectMode;
}

/*********************************************************************
 * @fn          heartRate_ReadAttrCB
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
static bStatus_t heartRate_ReadAttrCB(uint16_t connHandle,
                                      gattAttribute_t *pAttr, uint8_t *pValue,
                                      uint16_t *pLen, uint16_t offset,
                                      uint16_t maxLen, uint8_t method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

  if (uuid == BODY_SENSOR_LOC_UUID)
  {
    *pLen = 1;
    pValue[0] = *pAttr->pValue;
  }
  else
  {
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return (status);
}

/*********************************************************************
 * @fn      heartRate_WriteAttrCB
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
static bStatus_t heartRate_WriteAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr, uint8_t *pValue,
                                       uint16_t len, uint16_t offset,
                                       uint8_t method)
{
  bStatus_t status = SUCCESS;

  uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
  switch (uuid)
  {
    case HEARTRATE_CTRL_PT_UUID:
      if (offset > 0)
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      else if (len != 1)
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      else if (*pValue != HEARTRATE_COMMAND_ENERGY_EXP)
      {
        status = HEARTRATE_ERR_NOT_SUP;
      }
      else
      {
        *(pAttr->pValue) = pValue[0];

        (*heartRateServiceCB)(HEARTRATE_COMMAND_SET);

      }
      break;

    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY);
      if (status == SUCCESS)
      {
        uint16 charCfg = BUILD_UINT16(pValue[0], pValue[1]);

        (*heartRateServiceCB)((charCfg == GATT_CFG_NO_OPERATION) ?
                                HEARTRATE_MEAS_NOTI_DISABLED :
                                HEARTRATE_MEAS_NOTI_ENABLED);
      }
      break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return (status);
}


/*********************************************************************
*********************************************************************/
