/******************************************************************************

 @file  bpservice.c

 @brief This file contains the BloodPressure sample service for use with the
        BloodPressure sample application.

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "bpservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Position of bloodPressure measurement value in attribute array
#define BLOODPRESSURE_MEAS_VALUE_POS       2
#define BLOODPRESSURE_MEAS_CONFIG_POS      3
#define BLOODPRESSURE_IMEAS_VALUE_POS      5
#define BLOODPRESSURE_IMEAS_CONFIG_POS     6

#define BLOODPRESSURE_MULTI_BOND_BIT       5

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// BloodPressure service
CONST uint8 bloodPressureServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BLOODPRESSURE_SERV_UUID), HI_UINT16(BLOODPRESSURE_SERV_UUID)
};

// BloodPressure temperature characteristic
CONST uint8 bloodPressureTempUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BLOODPRESSURE_MEAS_UUID), HI_UINT16(BLOODPRESSURE_MEAS_UUID)
};

// BloodPressure Intermediate Cuff Pressure
CONST uint8 bloodPressureImeasUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(IMEDIATE_CUFF_PRESSURE_UUID), HI_UINT16(IMEDIATE_CUFF_PRESSURE_UUID)
};

// BloodPressure Feature
CONST uint8 bpFeatureUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BLOODPRESSURE_FEATURE_UUID), HI_UINT16(BLOODPRESSURE_FEATURE_UUID)
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

static bloodPressureServiceCB_t bloodPressureServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// BloodPressure Service attribute.
static CONST gattAttrType_t bloodPressureService = { ATT_BT_UUID_SIZE, bloodPressureServUUID };

// BloodPressure Characteristic.
static uint8 bloodPressureTempProps = GATT_PROP_INDICATE;
static gattCharCfg_t *bloodPressureMeasConfig;
static uint8 bloodPressureTemp = 0;

// Intermediate Measurement.
static uint8  bloodPressureImeasProps = GATT_PROP_NOTIFY;
static uint8  bloodPressureImeas=0;
static gattCharCfg_t *bloodPressureIMeasConfig;

// BP Feature
/*
 * bit 0 Body Movement Detection Support bit
 * bit 1 Cuff Fit Detection Support bit
 * bit 2 Irregular Pulse Detection Support bit
 * bit 3 Pulse Rate Range Detection Support bit
 * bit 4 Measurement Position Detection Support bit
 * bit 5 Multiple Bond Support bit
 * bit 6. Reserved for Future Use
 */
static uint8  bpFeatureProps = GATT_PROP_READ;
static uint16 bpFeature = 0x01 << BLOODPRESSURE_MULTI_BOND_BIT;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t bloodPressureAttrTbl[] =
{
  // BloodPressure Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&bloodPressureService            /* pValue */
  },

    //////////////////////////////////////////////
    // BLOOD PRESSURE MEASUREMENT
    //////////////////////////////////////////////

    // 1. Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &bloodPressureTempProps
    },

    // 2. Characteristic Value
    {
      { ATT_BT_UUID_SIZE, bloodPressureTempUUID },
      0, //return READ_NOT_PERMITTED
      0,
      &bloodPressureTemp
    },

    // 3.Characteristic Configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE,
      0,
      (uint8 *)&bloodPressureMeasConfig
    },

    //////////////////////////////////////////////
    // INTERMEDIATE CUFF PRESSURE
    //////////////////////////////////////////////

    // 4.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &bloodPressureImeasProps
    },

    // 5.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, bloodPressureImeasUUID },
      0, //return READ_NOT_PERMITTED
      0,
      &bloodPressureImeas
    },

    // 6.Characteristic Configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE,
      0,
      (uint8 *)&bloodPressureIMeasConfig
    },

    //////////////////////////////////////////////
    // FEATURE
    //////////////////////////////////////////////

    // 7.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &bpFeatureProps
    },

    // 8.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, bpFeatureUUID },
      GATT_PERMIT_READ,
      0,
      (uint8 *)&bpFeature
    },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 BloodPressure_ReadAttrCB(uint16_t connHandle,
                                      gattAttribute_t *pAttr, uint8_t *pValue,
                                      uint16_t *pLen, uint16_t offset,
                                      uint16_t maxLen, uint8_t method);
static bStatus_t BloodPressure_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Blood Pressure Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t bloodPressureCBs =
{
  BloodPressure_ReadAttrCB,  // Read callback function pointer.
  BloodPressure_WriteAttrCB, // Write callback function pointer.
  NULL                       // Authorization callback function pointer.
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
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
bStatus_t BloodPressure_AddService(uint32 services)
{
  uint8 status;

  // Allocate Client Characteristic Configuration table
  bloodPressureMeasConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                           linkDBNumConns );
  if ( bloodPressureMeasConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Allocate Client Characteristic Configuration table
  bloodPressureIMeasConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                            linkDBNumConns );
  if ( bloodPressureIMeasConfig == NULL )
  {
    // Free already allocated data
    ICall_free( bloodPressureMeasConfig );

    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes.
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, bloodPressureMeasConfig);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, bloodPressureIMeasConfig);

  if (services & BLOODPRESSURE_SERVICE)
  {
    // Register GATT attribute list and CBs with GATT Server App.
    status = GATTServApp_RegisterService(bloodPressureAttrTbl,
                                         GATT_NUM_ATTRS(bloodPressureAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &bloodPressureCBs);
  }
  else
  {
    status = SUCCESS;
  }

  return (status);
}

/*********************************************************************
 * @fn      BloodPressure_Register
 *
 * @brief   Register a callback function with the BloodPressure Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void BloodPressure_Register(bloodPressureServiceCB_t pfnServiceCB)
{
  bloodPressureServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      BloodPressure_SetParameter
 *
 * @brief   Set a parameter.
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
bStatus_t BloodPressure_SetParameter(uint8 param, uint8 len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
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
bStatus_t BloodPressure_GetParameter(uint8 param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

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
bStatus_t BloodPressure_MeasIndicate(uint16 connHandle,
                                     attHandleValueInd_t *pNoti,uint8 taskId)
{
  uint16 value = GATTServApp_ReadCharCfg(connHandle, bloodPressureMeasConfig);

  // If indications enabled
  if (value & GATT_CLIENT_CFG_INDICATE)
  {
    // Set the handle.
    pNoti->handle = bloodPressureAttrTbl[BLOODPRESSURE_MEAS_VALUE_POS].handle;

    // Send the Indication.
    return GATT_Indication(connHandle, pNoti, FALSE, taskId);
  }

  return bleIncorrectMode;
}


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
bStatus_t BloodPressure_IMeasNotify(uint16 connHandle,
                                   attHandleValueNoti_t *pNoti, uint8 taskId)
{
  uint16 value = GATTServApp_ReadCharCfg(connHandle,
                                           bloodPressureIMeasConfig);

  // If notifications enabled
  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    // Set the handle.
    pNoti->handle = bloodPressureAttrTbl[BLOODPRESSURE_IMEAS_VALUE_POS].handle;

    // Send the Indication.
    return GATT_Notification(connHandle, pNoti, FALSE);
  }

  return bleIncorrectMode;
}

/*********************************************************************
 * @fn          BloodPressure_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 BloodPressure_ReadAttrCB(uint16_t connHandle,
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

  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch (uuid)
    {
      case BLOODPRESSURE_FEATURE_UUID:
        {
          *pLen = 2;
          pValue[0] = LO_UINT16(bpFeature);
          pValue[1] = HI_UINT16(bpFeature);
        }
        break;

      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }

  return (status);
}

/*********************************************************************
 * @fn      BloodPressure_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t BloodPressure_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

  switch (uuid)
  {
    case GATT_CLIENT_CHAR_CFG_UUID:
      if (pAttr->handle ==
           bloodPressureAttrTbl[BLOODPRESSURE_MEAS_CONFIG_POS].handle)
      {
        // BloodPressure Indications.
        status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                offset,
                                                GATT_CLIENT_CFG_INDICATE);
        if (status == SUCCESS)
        {
          uint16 value = BUILD_UINT16(pValue[0], pValue[1]);

          (*bloodPressureServiceCB)((value == GATT_CFG_NO_OPERATION) ?
                                     BLOODPRESSURE_MEAS_NOTI_DISABLED :
                                     BLOODPRESSURE_MEAS_NOTI_ENABLED);
        }
      }
      else if (pAttr->handle ==
                bloodPressureAttrTbl[BLOODPRESSURE_IMEAS_CONFIG_POS].handle)
      {
        // BloodPressure Notifications.
        status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                offset, GATT_CLIENT_CFG_NOTIFY);
        if (status == SUCCESS)
        {
          uint16 value = BUILD_UINT16(pValue[0], pValue[1]);

          (*bloodPressureServiceCB)((value == GATT_CFG_NO_OPERATION)  ?
                                     BLOODPRESSURE_IMEAS_NOTI_DISABLED :
                                     BLOODPRESSURE_IMEAS_NOTI_ENABLED);
        }
      }
      else
      {
        status = ATT_ERR_INVALID_HANDLE;
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
