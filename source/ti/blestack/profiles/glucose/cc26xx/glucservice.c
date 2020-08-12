/******************************************************************************

 @file  glucservice.c

 @brief This file contains the Glucose sample service for use with the
        Glucose   sample application.

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

 ******************************************************************************
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "glucservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Position of glucose measurement value in attribute array.
#define GLUCOSE_MEAS_VALUE_POS         2
#define GLUCOSE_MEAS_CONFIG_POS        3
#define GLUCOSE_CONTEXT_VALUE_POS      5
#define GLUCOSE_CONTEXT_CONFIG_POS     6
#define GLUCOSE_CTL_PNT_VALUE_POS      10
#define GLUCOSE_CTL_PNT_CONFIG_POS     11

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Glucose service.
CONST uint8 glucoseServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GLUCOSE_SERV_UUID), HI_UINT16(GLUCOSE_SERV_UUID)
};

// Glucose characteristic.
CONST uint8 glucoseMeasUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GLUCOSE_MEAS_UUID), HI_UINT16(GLUCOSE_MEAS_UUID)
};

// Glucose Measurement Context.
CONST uint8 glucoseContextUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GLUCOSE_CONTEXT_UUID), HI_UINT16(GLUCOSE_CONTEXT_UUID)
};

// Glucose Feature.
CONST uint8 glucoseFeatureUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GLUCOSE_FEATURE_UUID), HI_UINT16(GLUCOSE_FEATURE_UUID)
};

// Record Control Point.
CONST uint8 recordControlPointUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RECORD_CTRL_PT_UUID), HI_UINT16(RECORD_CTRL_PT_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/* TRUE if record transfer in progress */
extern bool glucoseSendAllRecords;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static glucoseServiceCB_t glucoseServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// Glucose Service attribute.
static CONST gattAttrType_t glucoseService = {ATT_BT_UUID_SIZE, glucoseServUUID};

// Glucose Characteristic.
static uint8 glucoseProps = GATT_PROP_NOTIFY;
static gattCharCfg_t *glucoseMeasConfig;
static uint8 glucoseMeas = 0;

// Measurement Context.
static uint8  glucoseContextProps = GATT_PROP_NOTIFY;
static uint8  glucoseContext=0;
static gattCharCfg_t *glucoseContextConfig;

// Glucose Feature.
static uint8 glucoseFeatureProps = GATT_PROP_READ;
static uint16 glucoseFeature = GLUCOSE_FEAT_ALL;

// Glucose Control.
static uint8  glucoseControlProps = GATT_PROP_INDICATE | GATT_PROP_WRITE;
static uint8  glucoseControl=0;
static gattCharCfg_t *glucoseControlConfig;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t glucoseAttrTbl[] =
{
  // Glucose Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&glucoseService                /* pValue */
  },

    // 1. Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &glucoseProps
    },

    // 2. Characteristic Value
    {
      { ATT_BT_UUID_SIZE, glucoseMeasUUID },
      0, //return READ_NOT_PERMITTED
      0,
      &glucoseMeas
    },

    // 3.Characteristic Configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
      0,
      (uint8 *)&glucoseMeasConfig
    },

    //////////////////////////////////////////////
    // MEASUREMENT CONTEXT
    //////////////////////////////////////////////

    // 4.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &glucoseContextProps
    },

    // 5.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, glucoseContextUUID },
      0, //return READ_NOT_PERMITTED
      0,
      &glucoseContext
    },

    // 6.Characteristic Configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
      0,
      (uint8 *)&glucoseContextConfig
    },

    //////////////////////////////////////////////
    // GLUCOSE FEATURE
    //////////////////////////////////////////////

    // 7.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &glucoseFeatureProps
    },

    // 8.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, glucoseFeatureUUID },
      GATT_PERMIT_ENCRYPT_READ,
      0,
      (uint8 *) &glucoseFeature
    },

    //////////////////////////////////////////////
    // GLUCOSE CONTROL POINT
    //////////////////////////////////////////////

    // 9.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &glucoseControlProps
    },

    // 10.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, recordControlPointUUID },
      GATT_PERMIT_AUTHEN_WRITE,
      0,
      &glucoseControl
    },

    // 11.Characteristic Configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
      0,
      (uint8 *)&glucoseControlConfig
    }
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t glucose_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t *pLen,
                                    uint16_t offset, uint16_t maxLen,
                                    uint8_t method);
static bStatus_t glucose_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                     uint8_t *pValue, uint16_t len,
                                     uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Service Callbacks.
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t  glucoseCBs =
{
  glucose_ReadAttrCB,   // Read callback function pointer
  glucose_WriteAttrCB,  // Write callback function pointer
  NULL                  // Authorization callback function pointer
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Glucose_AddService
 *
 * @brief   Initializes the Glucose   service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Glucose_AddService(uint32 services)
{
  uint8 status;

  // Allocate Client Characteristic Configuration table
  glucoseMeasConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                     linkDBNumConns );
  if ( glucoseMeasConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Allocate Client Characteristic Configuration table
  glucoseContextConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                        linkDBNumConns );
  if ( glucoseContextConfig == NULL )
  {
    // Free already allocated data
    ICall_free( glucoseMeasConfig );

    return ( bleMemAllocError );
  }

  // Allocate Client Characteristic Configuration table
  glucoseControlConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                        linkDBNumConns );
  if ( glucoseControlConfig == NULL )
  {
    // Free already allocated data
    ICall_free( glucoseMeasConfig );
    ICall_free( glucoseContextConfig );

    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes.
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, glucoseMeasConfig);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, glucoseContextConfig);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, glucoseControlConfig);

  if (services & GLUCOSE_SERVICE)
  {
    // Register GATT attribute list and CBs with GATT Server App.
    status = GATTServApp_RegisterService(glucoseAttrTbl,
                                         GATT_NUM_ATTRS(glucoseAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &glucoseCBs);
  }
  else
  {
    status = SUCCESS;
  }

  return (status);
}

/*********************************************************************
 * @fn      Glucose_Register
 *
 * @brief   Register a callback function with the Glucose Service.
 *
 * @param   pfnServiceCB - Callback function.
 *          pfnCtlPntCB - Callback for control point
 *
 * @return  None.
 */
extern void Glucose_Register(glucoseServiceCB_t pfnServiceCB)
{
  glucoseServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      Glucose_SetParameter
 *
 * @brief   Set a glucose parameter.
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
bStatus_t Glucose_SetParameter(uint8 param, uint8 len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case GLUCOSE_FEATURE_PARAM:
      glucoseFeature = *((uint16*)value);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      Glucose_GetParameter
 *
 * @brief   Get a Glucose parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Glucose_GetParameter(uint8 param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case GLUCOSE_FEATURE_PARAM:
      *((uint16*)value) = glucoseFeature;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          Glucose_MeasSend
 *
 * @brief       Send a glucose measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti      - pointer to notification structure
 * @param       taskId     - caller's task Id.
 *
 * @return      Success or Failure
 */
bStatus_t Glucose_MeasSend(uint16 connHandle, attHandleValueNoti_t *pNoti,
                               uint8 taskId)
{
  uint16 value = GATTServApp_ReadCharCfg(connHandle, glucoseMeasConfig);

  // If notifications enabled
  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    // Set the handle.
    pNoti->handle = glucoseAttrTbl[GLUCOSE_MEAS_VALUE_POS].handle;

    // Send the Indication.
    return GATT_Notification(connHandle, pNoti, FALSE);
  }

  return bleNotReady;
}

/*********************************************************************
 * @fn          Glucose_ContextSend
 *
 * @brief       Send a glucose measurement context.
 *
 * @param       connHandle - connection handle
 * @param       pNoti      - pointer to notification structure
 * @param       taskId     - caller's task Id.
 *
 * @return      Success or Failure
 */
bStatus_t Glucose_ContextSend(uint16 connHandle,
                                  attHandleValueNoti_t *pNoti, uint8 taskId)
{
  uint16 value = GATTServApp_ReadCharCfg(connHandle, glucoseContextConfig);

  // If notifications enabled
  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    // Set the handle.
    pNoti->handle = glucoseAttrTbl[GLUCOSE_CONTEXT_VALUE_POS].handle;

    // Send the Indication.
    return GATT_Notification(connHandle, pNoti, FALSE);
  }

  return bleNotReady;
}

/*********************************************************************
 * @fn          Glucose_CtlPntIndicate
 *
 * @brief       Send an indication containing a control point
 *              message.
 *
 * @param       connHandle - connection handle
 * @param       pInd       - pointer to indication structure
 * @param       taskId     - caller's task Id.
 *
 * @return      Success or Failure
 */
bStatus_t Glucose_CtlPntIndicate(uint16 connHandle,
                                     attHandleValueInd_t *pInd, uint8 taskId)
{
  uint16 value = GATTServApp_ReadCharCfg(connHandle, glucoseControlConfig);

  // If indications enabled
  if (value & GATT_CLIENT_CFG_INDICATE)
  {
    // Set the handle.
    pInd->handle = glucoseAttrTbl[GLUCOSE_CTL_PNT_VALUE_POS].handle;

    // Send the Indication.
    return GATT_Indication(connHandle, pInd, FALSE, taskId);
  }

  return bleNotReady;
}

/*********************************************************************
 * @fn          glucose_ReadAttrCB
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
static bStatus_t glucose_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t *pLen,
                                    uint16_t offset, uint16_t maxLen,
                                    uint8_t method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation
  // (no attributes in the profile are long).
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID.
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch (uuid)
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those types for reads.

      case GLUCOSE_FEATURE_UUID:
        *pLen = 2;
        pValue[0] = LO_UINT16(glucoseFeature);
        pValue[1] = HI_UINT16(glucoseFeature);
        break;

      default:
        // Should never get here!
        // (characteristics 3 and 4 do not have read permissions).
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }

  return (status);
}

/*********************************************************************
 * @fn      glucose_WriteAttrCB
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
static bStatus_t glucose_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                     uint8_t *pValue, uint16_t len,
                                     uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation
  // (no attributes in the profile are long).
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

  switch (uuid)
  {

  case  GATT_CLIENT_CHAR_CFG_UUID:
      // Glucose Notifications.
      if ((pAttr->handle == glucoseAttrTbl[GLUCOSE_MEAS_CONFIG_POS].handle ||
           pAttr->handle == glucoseAttrTbl[GLUCOSE_CONTEXT_CONFIG_POS].handle))
      {
        status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY);
        if (status == SUCCESS)
        {
          uint16 charCfg = BUILD_UINT16(pValue[0], pValue[1]);

          if(pAttr->handle == glucoseAttrTbl[GLUCOSE_MEAS_CONFIG_POS].handle)
          {
            (*glucoseServiceCB)((charCfg == 0) ? GLUCOSE_MEAS_NTF_DISABLED :
                                                 GLUCOSE_MEAS_NTF_ENABLED, NULL, NULL);
          }
          else
          {
            (*glucoseServiceCB)((charCfg == 0) ? GLUCOSE_CONTEXT_NTF_DISABLED :
                                                 GLUCOSE_CONTEXT_NTF_ENABLED, NULL, NULL);
          }
        }
      }
      // Glucose Indications.
      else if (pAttr->handle == glucoseAttrTbl[GLUCOSE_CTL_PNT_CONFIG_POS].handle)
      {
        status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_INDICATE);
        if (status == SUCCESS)
        {
            uint16 charCfg = BUILD_UINT16(pValue[0], pValue[1]);

            (*glucoseServiceCB)((charCfg == 0) ? GLUCOSE_CTL_PNT_IND_DISABLED :
                                                 GLUCOSE_CTL_PNT_IND_ENABLED, NULL, NULL);
        }
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE;
      }
    break;

    case  RECORD_CTRL_PT_UUID:
      if(len >= GLUCOSE_CTL_PNT_MIN_SIZE  && len <= GLUCOSE_CTL_PNT_MAX_SIZE)
      {
        uint8 opcode = pValue[0];

        // If transfer in progress
        if (opcode != CTL_PNT_OP_ABORT && glucoseSendAllRecords)
        {
          status = GLUCOSE_ERR_IN_PROGRESS;
        }
        // If CCC not configured for glucose measurement
        else if (opcode == CTL_PNT_OP_REQ &&
                 !((GATTServApp_ReadCharCfg(connHandle, glucoseControlConfig) &
                     GATT_CLIENT_CFG_INDICATE)))
        {
          status = GLUCOSE_ERR_CCC_CONFIG;
        }
        else
        {
          (*glucoseServiceCB)(GLUCOSE_CTL_PNT_CMD, pValue, len);
        }
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
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
