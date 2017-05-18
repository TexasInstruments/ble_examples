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
/*
 *  ======== temperature_service.c ========
 */



/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "temperature_service.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

//! UUID for the GATT Primary Service Declaration
const uint8_t temperature_service_uuid[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(TEMPERATURE_SERVICE_SERV_UUID)
};

// Update_PDU UUID
const uint8_t temperature_service_data_uuid[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(TEMPERATURE_SERVICE_DATA_UUID)
};
// Update_PHY UUID
const uint8_t temperature_service_config_uuid[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(TEMPERATURE_SERVICE_CONFIG_UUID)
};
// Toggle_Temperature UUID
const uint8_t temperature_service_period_uuid[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(TEMPERATURE_SERVICE_PERIOD_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static Temperature_ServiceCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static const gattAttrType_t temperature_serviceDecl = { ATT_UUID_SIZE, temperature_service_uuid };

// Characteristic "Data" Properties (for declaration)
static uint8_t temperature_service_data_props = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8_t temperature_service_data_val[TEMPERATURE_SERVICE_DATA_LEN] = {0};
static gattCharCfg_t *temperature_service_data_cccd; //This characteristic is notifiable!

// Characteristic "Config" Properties (for declaration)
static uint8_t temperature_service_config_props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8_t temperature_service_config_val[TEMPERATURE_SERVICE_CONFIG_LEN] = {0};

// Characteristic "Period" Properties (for declaration)
static uint8_t temperature_service_period_props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8_t temperature_service_period_val[TEMPERATURE_SERVICE_PERIOD_LEN] = {0};

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t temperature_serviceAttrTbl[] =
{
  // Temperature_Service Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&temperature_serviceDecl
  },
    // Data Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &temperature_service_data_props
    },
      // Data Characteristic Value
      {
        { ATT_UUID_SIZE, temperature_service_data_uuid },
        GATT_PERMIT_READ,
        0,
        temperature_service_data_val
      },

      // Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&temperature_service_data_cccd
      },

    // Config Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &temperature_service_config_props
    },
      // Config Characteristic Value
      {
        { ATT_UUID_SIZE, temperature_service_config_uuid },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        temperature_service_config_val
      },
    // Period Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &temperature_service_period_props
    },
      // Period Characteristic Value
      {
        { ATT_UUID_SIZE, temperature_service_period_uuid },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        temperature_service_period_val
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Temperature_Service_ReadAttrCB( uint16 connHandle,
    gattAttribute_t *pAttr, uint8 *pValue, uint16 *pLen, uint16 offset,
    uint16 maxLen, uint8 method );
static bStatus_t Temperature_Service_WriteAttrCB( uint16 connHandle,
    gattAttribute_t *pAttr, uint8 *pValue, uint16 len, uint16 offset,
    uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
const gattServiceCBs_t Temperature_ServiceCBs =
{
  Temperature_Service_ReadAttrCB,  // Read callback function pointer
  Temperature_Service_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * Temperature_Service_AddService- Initializes the Temperature_Service service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t Temperature_Service_AddService( void )
{
    uint8_t status;

    temperature_service_data_cccd =
      (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );

    if ( temperature_service_data_cccd == NULL )
    {
      return ( bleMemAllocError );
    }

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, temperature_service_data_cccd );

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( temperature_serviceAttrTbl,
        GATT_NUM_ATTRS( temperature_serviceAttrTbl ),
        GATT_MAX_ENCRYPT_KEY_SIZE,
        &Temperature_ServiceCBs );

    return ( status );
}

/*
 * Temperature_Service_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t Temperature_Service_RegisterAppCBs(
    Temperature_ServiceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*
 * Temperature_Service_SetParameter - Set a Temperature_Service parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Temperature_Service_SetParameter( uint8 param, uint8 len,
    void *value )
{
    bStatus_t ret = SUCCESS;
    switch ( param )
    {
        case TEMPERATURE_SERVICE_DATA:
            if ( len == TEMPERATURE_SERVICE_DATA_LEN )
            {
                memcpy(temperature_service_data_val, value, len);

                // See if Notification has been enabled
                ret = GATTServApp_ProcessCharCfg( temperature_service_data_cccd,
                                                  temperature_service_data_val,
                                                  FALSE,
                                                  temperature_serviceAttrTbl,
                                                  GATT_NUM_ATTRS( temperature_serviceAttrTbl ),
                                                  INVALID_TASK_ID,
                                                  Temperature_Service_ReadAttrCB );
            }
            else
            {
                ret = bleInvalidRange;
            }
            break;

        case TEMPERATURE_SERVICE_CONFIG:
            if ( len == TEMPERATURE_SERVICE_CONFIG_LEN )
            {
                memcpy(temperature_service_config_val, value, len);
            }
            else
            {
                ret = bleInvalidRange;
            }
            break;

        case TEMPERATURE_SERVICE_PERIOD:
            if ( len == TEMPERATURE_SERVICE_PERIOD_LEN )
            {
                memcpy(temperature_service_period_val, value, len);
            }
            else
            {
                ret = bleInvalidRange;
            }
            break;

        default:
            ret = INVALIDPARAMETER;
            break;
    }
    return ret;
}


/*
 * Temperature_Service_GetParameter - Get a Temperature_Service parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Temperature_Service_GetParameter( uint8 param, void *value )
{
    bStatus_t ret = SUCCESS;
    switch ( param )
    {
        case TEMPERATURE_SERVICE_DATA:
            memcpy(value, temperature_service_data_val,
                TEMPERATURE_SERVICE_DATA_LEN);
            break;

        case TEMPERATURE_SERVICE_CONFIG:
            memcpy(value, temperature_service_config_val,
                TEMPERATURE_SERVICE_CONFIG_LEN);
            break;

        case TEMPERATURE_SERVICE_PERIOD:
            memcpy(value, temperature_service_period_val,
                TEMPERATURE_SERVICE_PERIOD_LEN);
            break;

        default:
            ret = INVALIDPARAMETER;
            break;
    }
    return ret;
}


/*********************************************************************
 * @fn          Temperature_Service_ReadAttrCB
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
static bStatus_t Temperature_Service_ReadAttrCB( uint16 connHandle,
    gattAttribute_t *pAttr, uint8 *pValue, uint16 *pLen, uint16 offset,
    uint16 maxLen, uint8 method )
{
    bStatus_t status = SUCCESS;

    // See if request is regarding the Update_PDU Characteristic Value
    if ( ! memcmp(pAttr->type.uuid, temperature_service_data_uuid, pAttr->type.len) )
    {
        // Prevent malicious ATT ReadBlob offsets.
        if ( offset > TEMPERATURE_SERVICE_DATA_LEN )
        {
            status = ATT_ERR_INVALID_OFFSET;
        }
        else
        {
            // Transmit as much as possible
            *pLen = MIN(maxLen, TEMPERATURE_SERVICE_DATA_LEN - offset);
            memcpy(pValue, pAttr->pValue + offset, *pLen);
        }
    }
    // See if request is regarding the Update_PHY Characteristic Value
    else if ( ! memcmp(pAttr->type.uuid, temperature_service_config_uuid, pAttr->type.len) )
    {
        // Prevent malicious ATT ReadBlob offsets.
        if ( offset > TEMPERATURE_SERVICE_CONFIG_LEN )
        {
            status = ATT_ERR_INVALID_OFFSET;
        }
        else
        {
            // Transmit as much as possible
            *pLen = MIN(maxLen, TEMPERATURE_SERVICE_CONFIG_LEN - offset);
            memcpy(pValue, pAttr->pValue + offset, *pLen);
        }
    }
    // See if request is regarding the Toggle_Temperature Characteristic Value
    else if ( ! memcmp(pAttr->type.uuid, temperature_service_period_uuid, pAttr->type.len) )
    {
        // Prevent malicious ATT ReadBlob offsets.
        if ( offset > TEMPERATURE_SERVICE_PERIOD_LEN )
        {
            status = ATT_ERR_INVALID_OFFSET;
        }
        else
        {
            // Transmit as much as possible
            *pLen = MIN(maxLen, TEMPERATURE_SERVICE_PERIOD_LEN - offset);
            memcpy(pValue, pAttr->pValue + offset, *pLen);
        }
    }
    else
    {
        /*
         * If we get here, that means you've forgotten to add an if clause for a
         * characteristic value attribute in the attribute table that has READ
         * permissions.
         */
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
    }

    return status;
}


/*********************************************************************
 * @fn      Temperature_Service_WriteAttrCB
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
static bStatus_t Temperature_Service_WriteAttrCB( uint16 connHandle,
    gattAttribute_t *pAttr, uint8 *pValue, uint16 len, uint16 offset,
    uint8 method )
{
    bStatus_t status  = SUCCESS;
    uint8_t   paramID = 0xFF;

    // See if request is regarding a Client Characterisic Configuration
    if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
    {
        // Allow only notifications.
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                    offset, GATT_CLIENT_CFG_NOTIFY);
    }
    // See if request is regarding the Update_PHY Characteristic Value
    else if ( ! memcmp(pAttr->type.uuid, temperature_service_config_uuid, pAttr->type.len) )
    {
        if ( offset + len > TEMPERATURE_SERVICE_CONFIG_LEN )
        {
            status = ATT_ERR_INVALID_OFFSET;
        }
        else
        {
            // Copy pValue into the variable we point to from the attribute table.
            memcpy(pAttr->pValue + offset, pValue, len);

            // Only notify application if entire expected value is written
            if ( offset + len == TEMPERATURE_SERVICE_CONFIG_LEN)
                paramID = TEMPERATURE_SERVICE_CONFIG;
        }
    }
    // See if request is regarding the Toggle_Temperature Characteristic Value
    else if ( ! memcmp(pAttr->type.uuid, temperature_service_period_uuid, pAttr->type.len) )
    {
        if ( offset + len > TEMPERATURE_SERVICE_PERIOD_LEN )
        {
            status = ATT_ERR_INVALID_OFFSET;
        }
        else
        {
            // Copy pValue into the variable we point to from the attribute table.
            memcpy(pAttr->pValue + offset, pValue, len);

            // Only notify application if entire expected value is written
            if ( offset + len == TEMPERATURE_SERVICE_PERIOD_LEN)
                paramID = TEMPERATURE_SERVICE_PERIOD;
        }
    }
    else
    {
        /*
         * If we get here, that means you've forgotten to add an if clause for a
         * characteristic value attribute in the attribute table that has WRITE
         * permissions.
         */
        status = ATT_ERR_ATTR_NOT_FOUND;
    }

    // Let the application know something changed (if it did) by using the
    // callback it registered earlier (if it did).
    if (paramID != 0xFF)
    {
        if ( pAppCBs && pAppCBs->pfnChangeCb )
        {
            // Call app function from stack task context.
            pAppCBs->pfnChangeCb( paramID );
        }
    }
    return status;
}
