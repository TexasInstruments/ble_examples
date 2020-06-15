/******************************************************************************

 @file       battservice.c

 @brief This file contains the Battery service.

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

/*********************************************************************
 * INCLUDES
 */

#include <string.h>
#include <xdc/std.h>
#include <stdbool.h>
#include <driverlib/aon_batmon.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "hiddev.h"
#include "battservice.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define BATT_LEVEL_VALUE_IDX        2 // Position of battery level in attribute array
#define BATT_LEVEL_VALUE_CCCD_IDX   3 // Position of battery level CCCD in attribute array

#define BATT_LEVEL_VALUE_LEN        1

/**
 * GATT Characteristic Descriptions
 */
#define GATT_DESC_LENGTH_UUID            0x3111 // Used with Unit percent

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Battery service
CONST uint8_t battServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BATT_SERV_UUID), HI_UINT16(BATT_SERV_UUID)
};

// Battery level characteristic
CONST uint8_t battLevelUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BATT_LEVEL_UUID), HI_UINT16(BATT_LEVEL_UUID)
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

// Application callback.
static battServiceCB_t battServiceCB;

// Measurement setup callback.
static battServiceSetupCB_t battServiceSetupCB = NULL;

// Critical battery level setting.
static uint8_t battCriticalLevel;

// Maximum battery level.
static uint16_t battMaxLevel = BATT_MAX_VOLTAGE;

// Measurement teardown callback.
static battServiceTeardownCB_t battServiceTeardownCB = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Battery Service attribute.
static CONST gattAttrType_t battService = { ATT_BT_UUID_SIZE, battServUUID };

// Battery level characteristic.
static uint8_t battLevelProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8_t battLevel = 100;

// Characteristic Presentation Format of the Battery Level Characteristic.
static gattCharFormat_t battLevelPresentation = {
  GATT_FORMAT_UINT8,           /* format */
  0,                           /* exponent */
  GATT_UNIT_PERCENTAGE_UUID,   /* unit */
  GATT_NS_BT_SIG,              /* name space */
  GATT_DESC_LENGTH_UUID        /* desc */
};

static gattCharCfg_t *battLevelClientCharCfg;

// HID Report Reference characteristic descriptor, battery level.
static uint8_t hidReportRefBattLevel[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_BATT_LEVEL_IN, HID_REPORT_TYPE_INPUT };

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t battAttrTbl[] =
{
  // Battery Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&battService                     /* pValue */
  },

    // Battery Level Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &battLevelProps
    },

      // Battery Level Value
      {
        { ATT_BT_UUID_SIZE, battLevelUUID },
        GATT_PERMIT_READ,
        0,
        &battLevel
      },

      // Battery Level Client Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *) &battLevelClientCharCfg
      },

      // HID Report Reference characteristic descriptor, batter level input
      {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefBattLevel
      },

      // Characteristic Presentation format
      {
        { ATT_BT_UUID_SIZE, charFormatUUID },
        GATT_PERMIT_READ,
        0,
        (uint8_t *)&battLevelPresentation
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t battReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                uint16_t maxLen, uint8 method );
static bStatus_t battWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                 uint8_t *pValue, uint16_t len, uint16_t offset,
                                 uint8 method );

static void battNotify(uint16_t connHandle);
static uint8_t battMeasure(void);
static void battNotifyLevel(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Battery Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t battCBs =
{
  battReadAttrCB,  // Read callback function pointer
  battWriteAttrCB, // Write callback function pointer
  NULL             // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Batt_AddService
 *
 * @brief   Initializes the Battery Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Batt_AddService(void)
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  battLevelClientCharCfg = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                          linkDBNumConns );
  if ( battLevelClientCharCfg == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, battLevelClientCharCfg);

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService(battAttrTbl,
                                       GATT_NUM_ATTRS(battAttrTbl),
                                       GATT_MAX_ENCRYPT_KEY_SIZE,
                                       &battCBs);

  // Enable the Battery Monitor.
  // The batterry monitor is enable and configure at startup by the boot code, it should not be change.

  return (status);
}

/*********************************************************************
 * @fn      Batt_Register
 *
 * @brief   Register a callback function with the Battery Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void Batt_Register(battServiceCB_t pfnServiceCB)
{
  battServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      Batt_SetParameter
 *
 * @brief   Set a Battery Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Batt_SetParameter(uint8_t param, uint8_t len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case BATT_PARAM_CRITICAL_LEVEL:
      battCriticalLevel = *((uint8*)value);

      // If below the critical level and critical state not set, notify it
      if (battLevel < battCriticalLevel)
      {
        battNotifyLevel();
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      Batt_GetParameter
 *
 * @brief   Get a Battery Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Batt_GetParameter(uint8_t param, void *value)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
    case BATT_PARAM_LEVEL:
      *((uint8*)value) = battLevel;
      break;

    case BATT_PARAM_CRITICAL_LEVEL:
      *((uint8*)value) = battCriticalLevel;
      break;

    case BATT_PARAM_SERVICE_HANDLE:
      *((uint16*)value) = GATT_SERVICE_HANDLE(battAttrTbl);
      break;

    case BATT_PARAM_BATT_LEVEL_IN_REPORT:
      {
        hidRptMap_t *pRpt = (hidRptMap_t *)value;

        pRpt->id = hidReportRefBattLevel[0];
        pRpt->type = hidReportRefBattLevel[1];
        pRpt->handle = battAttrTbl[BATT_LEVEL_VALUE_IDX].handle;
        pRpt->pCccdAttr = &battAttrTbl[BATT_LEVEL_VALUE_CCCD_IDX];
        pRpt->mode = HID_PROTOCOL_MODE_REPORT;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          Batt_MeasLevel
 *
 * @brief       Measure the battery level and update the battery
 *              level value in the service characteristics.  If
 *              the battery level-state characteristic is configured
 *              for notification and the battery level has changed
 *              since the last measurement, then a notification
 *              will be sent.
 *
 * @return      Success
 */
bStatus_t Batt_MeasLevel(void)
{
  uint16_t level;

  level = battMeasure();

  // If level has gone down
  if (level < battLevel)
  {
    // Update level
    battLevel = (uint8)(level & 0x00FF);

    // Send a notification
    battNotifyLevel();
  }

  return SUCCESS;
}

/*********************************************************************
 * @fn      Batt_Setup
 *
 * @brief   Set up which ADC source is to be used. Defaults to VDD/3.
 *
 * @param   adc_ch - ADC Channel, e.g. HAL_ADC_CHN_AIN6
 * @param   minVal - max battery level
 * @param   maxVal - min battery level
 * @param   sCB - HW setup callback
 * @param   tCB - HW tear down callback
 * @param   cCB - percentage calculation callback
 *
 * @return  none.
 */
void Batt_Setup(uint16_t maxVal, battServiceSetupCB_t sCB,
                battServiceTeardownCB_t tCB)
{
  battMaxLevel = maxVal;
  battServiceSetupCB = sCB;
  battServiceTeardownCB = tCB;
}

/*********************************************************************
 * @fn          battReadAttrCB
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
static bStatus_t battReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                uint16_t maxLen, uint8 method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

  // Measure battery level if reading level
  if (uuid == BATT_LEVEL_UUID)
  {
    uint8_t level;

    level = battMeasure();

    // If level has gone down
    if (level < battLevel)
    {
      // Update level
      battLevel = level;
    }

    *pLen = 1;
    pValue[0] = battLevel;
  }
  else if (uuid == GATT_REPORT_REF_UUID)
  {
    *pLen = HID_REPORT_REF_LEN;
    memcpy(pValue, pAttr->pValue, HID_REPORT_REF_LEN);
  }
  else
  {
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return (status);
}

/*********************************************************************
 * @fn      battWriteAttrCB
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
static bStatus_t battWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                 uint8_t *pValue, uint16_t len, uint16_t offset,
                                 uint8 method)
{
  bStatus_t status = SUCCESS;

  uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
  switch (uuid)
  {
    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY);
      if (status == SUCCESS)
      {
        uint16_t charCfg = BUILD_UINT16(pValue[0], pValue[1]);

        if (battServiceCB)
        {
          (*battServiceCB)((charCfg == GATT_CFG_NO_OPERATION) ?
                            BATT_LEVEL_NOTI_DISABLED :
                            BATT_LEVEL_NOTI_ENABLED);
        }
      }
      break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return (status);
}

/*********************************************************************
 * @fn          battNotify
 *
 * @brief       Send a notification of the level state characteristic.
 *
 * @param       connHandle - linkDB item
 *
 * @return      None.
 */
static void battNotify(uint16_t connHandle)
{
  uint16_t value = GATTServApp_ReadCharCfg(connHandle,
                                           battLevelClientCharCfg);
  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    attHandleValueNoti_t noti;

    noti.pValue = GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI,
                                BATT_LEVEL_VALUE_LEN, NULL);
    if (noti.pValue != NULL)
    {
      noti.handle = battAttrTbl[BATT_LEVEL_VALUE_IDX].handle;
      noti.len = BATT_LEVEL_VALUE_LEN;
      noti.pValue[0] = battLevel;

      if (GATT_Notification(connHandle, &noti, FALSE) != SUCCESS)
      {
        GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
      }
    }
  }
}

/*********************************************************************
 * @fn      battMeasure
 *
 * @brief   Measure the battery level with the ADC and return
 *          it as a percentage 0-100%.
 *
 * @return  Battery level.
 */
static uint8_t battMeasure(void)
{
  uint32_t percent;

  // Call measurement setup callback
  if (battServiceSetupCB != NULL)
  {
    battServiceSetupCB();
  }

  // Read the battery voltage (V), only the first 12 bits
  percent = AONBatMonBatteryVoltageGet();

  // Convert to from V to mV to avoid fractions.
  // Fractional part is in the lower 8 bits thus converting is done as follows:
  // (1/256)/(1/1000) = 1000/256 = 125/32
  // This is done most effectively by multiplying by 125 and then shifting
  // 5 bits to the right.
  percent = (percent * 125) >> 5;
  // Convert to percentage of maximum voltage.
  percent = ((percent* 100) / battMaxLevel);

  // Call measurement teardown callback
  if (battServiceTeardownCB != NULL)
  {
    battServiceTeardownCB();
  }

  return percent;
}

/*********************************************************************
 * @fn      battNotifyLevelState
 *
 * @brief   Send a notification of the battery level state
 *          characteristic if a connection is established.
 *
 * @return  None.
 */
static void battNotifyLevel(void)
{
  uint8_t i;
  for (i = 0; i < linkDBNumConns; i++)
  {
    uint16_t connHandle = battLevelClientCharCfg[i].connHandle;

    if (connHandle != INVALID_CONNHANDLE)
    {
      // Send notification to connected device.
      battNotify(connHandle);
    }
  }
}


/*********************************************************************
*********************************************************************/
