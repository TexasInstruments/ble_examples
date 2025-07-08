/******************************************************************************

@file  hid_service.c

@brief This file contains the device info application functionality

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2025, Texas Instruments Incorporated
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

 Copyright (c) 2022-2025, Texas Instruments Incorporated
 All rights reserved.

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 ******************************************************************************
 *****************************************************************************/

#include <string.h>
#include <icall.h>
#include "util.h"
#include "icall_ble_api.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_internal.h>
#include "hid_service.h"
#include "HID/hiddev.h"
#include "att.h"
#include "gatt.h"
#include "app_peripheral.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "battservice.h"

// HID service
GATT_BT_UUID(hidServUUID, HID_SERV_UUID);

// HID Information characteristic
GATT_BT_UUID(hidInfoUUID, HID_INFORMATION_UUID);

// HID Report Map characteristic
GATT_BT_UUID(hidReportMapUUID, REPORT_MAP_UUID);

// HID Boot Keyboard Input Report characteristic
GATT_BT_UUID(hidBootKeyInputUUID, BOOT_KEY_INPUT_UUID);

// HID Control Point characteristic
GATT_BT_UUID(hidControlPointUUID, HID_CTRL_PT_UUID);

// HID Report characteristic
GATT_BT_UUID(hidReportUUID, REPORT_UUID);

// HID Protocol Mode characteristic
GATT_BT_UUID(hidProtocolModeUUID, PROTOCOL_MODE_UUID);

// HID Boot Keyboard Output Report characteristic
GATT_BT_UUID(hidBootKeyOutputUUID, BOOT_KEY_OUTPUT_UUID);

extern uint8 linkDBNumConns;

// HID Information characteristic value
static const uint8 hidInfo[HID_INFORMATION_LEN] =
{
  LO_UINT16(0x0111), HI_UINT16(0x0111),           // bcdHID (USB HID version)
  0x00,                                           // bCountryCode
  HID_KBD_FLAGS                                   // Flags
};

// HID Report Map characteristic value
// Keyboard report descriptor
static const uint8 hidReportMap[] =
{
  0x05, 0x01,  // USAGE_PAGE (Generic Desktop)
  0x09, 0x06,  // USAGE (Keyboard)
  0xA1, 0x01,  // COLLECTION (Application)
  0x85, 0x01,  // REPORT_ID (1)
               //
  0x05, 0x07,  //   USAGE_PAGE (Key Codes)
  0x19, 0xE0,  //   USAGE_MIN (224)
  0x29, 0xE7,  //   USAGE_MAX (231)
  0x15, 0x00,  //   LOGICAL_MINIMUM (0)
  0x25, 0x01,  //   LOGICAL_MAXIMUM (1)
               //
  0x75, 0x01,  //     REPORT_SIZE (1)
  0x95, 0x08,  //     REPORT_COUNT (1)
  0x81, 0x02,  //     INPUT (Constant)
  0x95, 0x01,  //     REPORT_COUNT (5)
  0x75, 0x08,  //     REPORT_SIZE (1)
  0x81, 0x01,  //     INPUT (Constant)
  0x95, 0x05,  //     REPORT_COUNT (1)
  0x75, 0x01,  //     REPORT_SIZE (3)

  0x05, 0x08,  //     USAGE_PAGE (LEDs)
  0x19, 0x01,  //     USAGE_MIN (1)
  0x29, 0x05,  //     USAGE_MAX (5)
  0x91, 0x02,  //     OUTPUT (Data, Variable, Absolute)

               //   LED report padding
  0x95, 0x01,  //     REPORT_COUNT (1)
  0x75, 0x03,  //     REPORT_SIZE (3)
  0x91, 0x01,  //     OUTPUT (Constant)
               //
               //   Key arrays (6 bytes)
  0x95, 0x06,  //     REPORT_COUNT (6)
  0x75, 0x08,  //     REPORT_SIZE (8)
  0x15, 0x1E,  //     LOGICAL_MINIMUM (0)
  0x25, 0xF1,  //     LOGICAL_MAXIMUM (101)
  0x05, 0x07,  //     USAGE_PAGE (Key Codes)
  0x19, 0x1E,  //     USAGE_MIN (0)
  0x29, 0xF1,  //     USAGE_MAX (101)
  0x81, 0x00,  //     INPUT (Data, Array)
               //
  0xC0,        // END_COLLECTION
               //
                    // Consumer Report in
  0x05, 0x0C,       // Usage Page (Consumer)
  0x09, 0x01,       // Usage (Consumer Control)
  0xA1, 0x01,       // Collection (Application)
  0x85, 0x02,       // Report Id
  0x75, 0x10,       // global, report size 16 bits
  0x95, 0x02,       // global, report count 2
  0x15, 0x01,       // global, min  0x01
  0x26, 0x8c,0x02,  // global, max  0x28c
  0x19, 0x01,       // local, min   0x01
  0x2a, 0x8c,0x02,  // local, max   0x28c
  0x81, 0x00,       // main,  input data variable, absolute
  0xc0,             // main, end collection
};

// HID report map length
uint8 hidReportMapLen = sizeof(hidReportMap);

// HID report mapping table
static hidRptMap_t  hidRptMap[HID_NUM_REPORTS];

/*********************************************************************
 * Profile Attributes - variables
 */

// Device Information Service attribute
static const gattAttrType_t hidService = GATT_ATT_BT_UUID_TYPE(hidServUUID);

// Include attribute (Battery service)
static uint16 include = GATT_INVALID_HANDLE;

// HID Information characteristic
static uint8 hidInfoProps = GATT_PROP_READ;

// HID Control Point characteristic
static uint8 hidControlPointProps = GATT_PROP_WRITE_NO_RSP;
static uint8 hidControlPoint;

// HID Protocol Mode characteristic
static uint8 hidProtocolModeProps = GATT_PROP_READ | GATT_PROP_WRITE_NO_RSP;
uint8 hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

// HID Report Map characteristic
static uint8 hidReportMapProps = GATT_PROP_READ;

// HID Report characteristic, key input
static uint8 hidReportKeyInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportKeyIn;
gattCharCfg_t *hidReportKeyInClientCharCfg;

// HID Report Reference characteristic descriptor, key input
static uint8 hidReportRefKeyIn[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT };

// HID Report characteristic, LED output
static uint8 hidReportLedOutProps = GATT_PROP_READ  |
                                    GATT_PROP_WRITE |
                                    GATT_PROP_WRITE_NO_RSP;
static uint8 hidReportLedOut;

// HID Report Reference characteristic descriptor, LED output
static uint8 hidReportRefLedOut[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_OUTPUT };

// HID Boot Keyboard Input Report
static uint8 hidReportBootKeyInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportBootKeyIn;
static gattCharCfg_t *hidReportBootKeyInClientCharCfg;

// HID Boot Keyboard Output Report
static uint8 hidReportBootKeyOutProps = GATT_PROP_READ  |
                                        GATT_PROP_WRITE |
                                        GATT_PROP_WRITE_NO_RSP;
static uint8 hidReportBootKeyOut;

// HID Report characteristic, consumer control input
static uint8 hidReportCCInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportCCIn;
static gattCharCfg_t *hidReportCCInClientCharCfg;

// HID Report Reference characteristic descriptor, consumer control input
static uint8 hidReportRefCCIn[HID_REPORT_REF_LEN] = { HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT };


static uint8 hidExtReportRefDesc[2] = { 0x00, 0x00 };


/*********************************************************************
 * Profile Attributes - Table
 */
// Firmware Revision String
GATT_BT_UUID(hidDevInfoFirmwareRevUUID, FIRMWARE_REV_UUID);
gattAttribute_t hidKbdAttrTbl[] =
{
  /*---------------type----------------*/  /*--permissions--*/  /*-----------pValue-----------*/

  // HID Service
  GATT_BT_ATT( primaryServiceUUID,          GATT_PERMIT_READ,       (uint8 *)&hidService ),
  // Included service (battery)
  GATT_BT_ATT( includeUUID,                   GATT_PERMIT_READ,       (uint8 *)&include ),

  // HID Information characteristic declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &hidInfoProps ),
  // HID Information characteristic
  GATT_BT_ATT( hidInfoUUID,                 GATT_PERMIT_ENCRYPT_READ,       (uint8 *) hidInfo ),

  // HID Control Point characteristic declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &hidControlPointProps ),
  // HID Control Point characteristic
  GATT_BT_ATT( hidControlPointUUID,        GATT_PERMIT_ENCRYPT_WRITE,       &hidControlPoint ),

  // HID Protocol Mode characteristic declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &hidProtocolModeProps ),
  // HID Protocol Mode characteristic
  GATT_BT_ATT( hidProtocolModeUUID,     GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,       &hidProtocolMode ),

  // HID Report Map characteristic declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &hidReportMapProps ),
  // HID Report Map characteristic
  GATT_BT_ATT( hidReportMapUUID,      GATT_PERMIT_ENCRYPT_READ,       (uint8 *) hidReportMap ),

  // HID External Report Reference Descriptor
  GATT_BT_ATT( extReportRefUUID,               GATT_PERMIT_READ,        (uint8 *) hidExtReportRefDesc ),


  // HID Report characteristic declaration, key input
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &hidReportKeyInProps ),
  // HID Report characteristic, key input
  GATT_BT_ATT( hidReportUUID,               GATT_PERMIT_ENCRYPT_READ,       &hidReportKeyIn ),

  // HID Report characteristic client characteristic configuration, key input
  GATT_BT_ATT( clientCharCfgUUID,               GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,      (uint8 *) &hidReportKeyInClientCharCfg ),

  // HID Report Reference characteristic descriptor, key input
  GATT_BT_ATT( reportRefUUID,          GATT_PERMIT_READ,        hidReportRefKeyIn ),

  // HID Report characteristic, LED output declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &hidReportLedOutProps ),
  // HID Report characteristic, LED output
  GATT_BT_ATT( hidReportUUID,        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,       &hidReportLedOut ),

  // HID Report Reference characteristic descriptor, LED output
  GATT_BT_ATT( reportRefUUID,        GATT_PERMIT_READ,       hidReportRefLedOut ),

  // HID Boot Keyboard Input Report declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &hidReportBootKeyInProps),
  // HID Boot Keyboard Input Report
  GATT_BT_ATT( hidBootKeyInputUUID,         GATT_PERMIT_ENCRYPT_READ,       &hidReportBootKeyIn),
  // HID Boot Keyboard Input Report characteristic client characteristic configuration
  GATT_BT_ATT( clientCharCfgUUID,           GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,      (uint8 *)&hidReportBootKeyInClientCharCfg),

  // HID Boot Keyboard Output Report declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &hidReportBootKeyOutProps),
  // HID Boot Keyboard Output Report
  GATT_BT_ATT( hidBootKeyOutputUUID,        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,       &hidReportBootKeyOut),

  // HID Report characteristic declaration, consumer control
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &hidReportCCInProps),
  // HID Report characteristic, consumer control
  GATT_BT_ATT( hidReportUUID,               GATT_PERMIT_ENCRYPT_READ,       &hidReportCCIn),
  // HID Report characteristic client characteristic configuration, consumer control
  GATT_BT_ATT( clientCharCfgUUID,           GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,    (uint8 *) &hidReportCCInClientCharCfg),
  // HID Report Reference characteristic descriptor, consumer control
  GATT_BT_ATT( reportRefUUID,               GATT_PERMIT_READ,       hidReportRefCCIn),
};

// Attribute index enumeration-- these indexes match array elements above
enum
{
  HID_SERVICE_IDX,                // HID Service
  HID_INCLUDED_SERVICE_IDX,       // Included Service (battery)
  HID_INFO_DECL_IDX,              // HID Information characteristic declaration
  HID_INFO_IDX,                   // HID Information characteristic
  HID_CONTROL_POINT_DECL_IDX,     // HID Control Point characteristic declaration
  HID_CONTROL_POINT_IDX,          // HID Control Point characteristic
  HID_PROTOCOL_MODE_DECL_IDX,     // HID Protocol Mode characteristic declaration
  HID_PROTOCOL_MODE_IDX,          // HID Protocol Mode characteristic
  HID_REPORT_MAP_DECL_IDX,        // HID Report Map characteristic declaration
  HID_REPORT_MAP_IDX,             // HID Report Map characteristic
  HID_EXT_REPORT_REF_DESC_IDX,    // HID External Report Reference Descriptor
  HID_REPORT_KEY_IN_DECL_IDX,     // HID Report characteristic declaration, key input
  HID_REPORT_KEY_IN_IDX,          // HID Report characteristic, key input
  HID_REPORT_KEY_IN_CCCD_IDX,     // HID Report characteristic client characteristic configuration, key input
  HID_REPORT_REF_KEY_IN_IDX,      // HID Report Reference characteristic descriptor, key input
  HID_REPORT_LED_OUT_DECL_IDX,    // HID Report characteristic, LED output declaration
  HID_REPORT_LED_OUT_IDX,         // HID Report characteristic, LED output
  HID_REPORT_REF_LED_OUT_IDX,     // HID Report Reference characteristic descriptor, LED output
  HID_BOOT_KEY_IN_DECL_IDX,       // HID Boot Keyboard Input Report declaration
  HID_BOOT_KEY_IN_IDX,            // HID Boot Keyboard Input Report
  HID_BOOT_KEY_IN_CCCD_IDX,       // HID Boot Keyboard Input Report characteristic client characteristic configuration
  HID_BOOT_KEY_OUT_DECL_IDX,      // HID Boot Keyboard Output Report declaration
  HID_BOOT_KEY_OUT_IDX,           // HID Boot Keyboard Output Report
  HID_REPORT_CC_IN_DECL_IDX,      // HID Report characteristic declaration, consumer control
  HID_REPORT_CC_IN_IDX,           // HID Report characteristic, consumer control
  HID_REPORT_CC_IN_CCCD_IDX,      // HID Report characteristic client characteristic configuration, consumer control
  HID_REPORT_REF_CC_IN_IDX,       // HID Report Reference characteristic descriptor, consumer control
};

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Device Info Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
const gattServiceCBs_t hidKbdCBs =
{
  HidDev_ReadAttrCB, // Read callback function pointer
  HidDev_WriteAttrCB,               // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * @fn       HidKbdCC_AddService
 *
 * @brief   Initializes tHID service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t HidKbdCC_AddService( void )
{
    uint8 status = SUCCESS;

    // Allocate Client Charateristic Configuration tables.
    hidReportKeyInClientCharCfg = (gattCharCfg_t *)BLEAppUtil_malloc(sizeof(gattCharCfg_t) *
                                                                linkDBNumConns);
    if (hidReportKeyInClientCharCfg == NULL)
    {
      return bleMemAllocError;
    }

    hidReportBootKeyInClientCharCfg = (gattCharCfg_t *)BLEAppUtil_malloc(sizeof(gattCharCfg_t) *
                                                                    linkDBNumConns);
    if (hidReportBootKeyInClientCharCfg == NULL)
    {
       BLEAppUtil_free(hidReportKeyInClientCharCfg);

      return bleMemAllocError;
    }

    hidReportCCInClientCharCfg = (gattCharCfg_t *)BLEAppUtil_malloc(sizeof(gattCharCfg_t) *
                                                               linkDBNumConns);
    if (hidReportCCInClientCharCfg == NULL)
    {
        BLEAppUtil_free(hidReportKeyInClientCharCfg);
        BLEAppUtil_free(hidReportBootKeyInClientCharCfg);
      return bleMemAllocError;
    }

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, hidReportKeyInClientCharCfg);
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, hidReportBootKeyInClientCharCfg);
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, hidReportCCInClientCharCfg);

  // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(hidKbdAttrTbl,
                                      GATT_NUM_ATTRS(hidKbdAttrTbl),
                                      GATT_MAX_ENCRYPT_KEY_SIZE,
                                      &hidKbdCBs);

    // Set up included service
    Batt_GetParameter(BATT_PARAM_SERVICE_HANDLE,
                      &GATT_INCLUDED_HANDLE(hidKbdAttrTbl, HID_INCLUDED_SERVICE_IDX));

    // Construct map of reports to characteristic handles
    // Each report is uniquely identified via its ID and type

    // Key input report
    hidRptMap[0].id = hidReportRefKeyIn[0];
    hidRptMap[0].type = hidReportRefKeyIn[1];
    hidRptMap[0].handle = hidKbdAttrTbl[HID_REPORT_KEY_IN_IDX].handle;
    hidRptMap[0].pCccdAttr = &hidKbdAttrTbl[HID_REPORT_KEY_IN_CCCD_IDX];
    hidRptMap[0].mode = HID_PROTOCOL_MODE_REPORT;

    // LED output report
    hidRptMap[1].id = hidReportRefLedOut[0];
    hidRptMap[1].type = hidReportRefLedOut[1];
    hidRptMap[1].handle = hidKbdAttrTbl[HID_REPORT_LED_OUT_IDX].handle;
    hidRptMap[1].pCccdAttr = NULL;
    hidRptMap[1].mode = HID_PROTOCOL_MODE_REPORT;

    // Boot keyboard input report
    // Use same ID and type as key input report
    hidRptMap[2].id = hidReportRefKeyIn[0];
    hidRptMap[2].type = hidReportRefKeyIn[1];
    hidRptMap[2].handle = hidKbdAttrTbl[HID_BOOT_KEY_IN_IDX].handle;
    hidRptMap[2].pCccdAttr = &hidKbdAttrTbl[HID_BOOT_KEY_IN_CCCD_IDX];
    hidRptMap[2].mode = HID_PROTOCOL_MODE_BOOT;

    // Boot keyboard output report
    // Use same ID and type as LED output report
    hidRptMap[3].id = hidReportRefLedOut[0];
    hidRptMap[3].type = hidReportRefLedOut[1];
    hidRptMap[3].handle = hidKbdAttrTbl[HID_BOOT_KEY_OUT_IDX].handle;
    hidRptMap[3].pCccdAttr = NULL;
    hidRptMap[3].mode = HID_PROTOCOL_MODE_BOOT;

    // Consumer Control input report
    hidRptMap[4].id = hidReportRefCCIn[0];
    hidRptMap[4].type = hidReportRefCCIn[1];
    hidRptMap[4].handle = hidKbdAttrTbl[HID_REPORT_CC_IN_IDX].handle;
    hidRptMap[4].pCccdAttr = &hidKbdAttrTbl[HID_REPORT_CC_IN_CCCD_IDX];
    hidRptMap[4].mode = HID_PROTOCOL_MODE_REPORT;

    // Battery level input report
    VOID Batt_GetParameter(BATT_PARAM_BATT_LEVEL_IN_REPORT,
                           &(hidRptMap[2]));

    // Setup report ID map
    HidDev_RegisterReports(HID_NUM_REPORTS, hidRptMap);

    return status;
}

/*********************************************************************
 * @fn      HidKbd_SetParameter
 *
 * @brief   Set a HID Kbd parameter.
 *
 * @param   id     - HID report ID.
 * @param   type   - HID report type.
 * @param   uuid   - attribute uuid.
 * @param   len    - length of data to right.
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the input parameters and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  GATT status code.
 */
uint8 HidKbd_SetParameter(uint8 id, uint8 type, uint16 uuid, uint8 len,
                          void *pValue)
{
  bStatus_t ret = SUCCESS;

  switch (uuid)
  {
    case REPORT_UUID:
      if (type ==  HID_REPORT_TYPE_OUTPUT)
      {
        if (len == 1)
        {
          hidReportLedOut = *((uint8 *)pValue);
        }
        else
        {
          ret = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        ret = ATT_ERR_ATTR_NOT_FOUND;
      }
      break;

    case BOOT_KEY_OUTPUT_UUID:
      if (len == 1)
      {
        hidReportBootKeyOut = *((uint8 *)pValue);
      }
      else
      {
        ret = ATT_ERR_INVALID_VALUE_SIZE;
      }
      break;

    default:
      // Ignore the request
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      HidKbd_GetParameter
 *
 * @brief   Get a HID Kbd parameter.
 *
 * @param   id     - HID report ID.
 * @param   type   - HID report type.
 * @param   uuid   - attribute uuid.
 * @param   pLen   - length of data to be read
 * @param   pValue - pointer to data to get.  This is dependent on
 *          the input parameters and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  GATT status code.
 */
uint8 HidKbd_GetParameter(uint8 id, uint8 type, uint16 uuid, uint8 *pLen,
                          void *pValue)
{
  switch (uuid)
  {
    case REPORT_UUID:
      if (type ==  HID_REPORT_TYPE_OUTPUT)
      {
        *((uint8 *)pValue) = hidReportLedOut;
        *pLen = 1;
      }
      else
      {
        *pLen = 0;
      }
      break;

    case BOOT_KEY_OUTPUT_UUID:
      *((uint8 *)pValue) = hidReportBootKeyOut;
      *pLen = 1;
      break;

    default:
      *pLen = 0;
      break;
  }

  return (SUCCESS);
}

/*********************************************************************
*********************************************************************/
