/******************************************************************************

@file  hiddev.c

@brief This file contains the HID main functionalities.

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
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <ti/drivers/apps/Button.h>
#include "hiddev.h"
#include "battservice.h"
#include <app_main.h>
#include "hid_service.h"
#include "app_peripheral.h"
#include "util.h"
#include "audio_stream_server.h"
#include "audio.h"
#include "PowerShutdown/power_shutdown.h"
#include "NVSExt/nvs_ext.h"
#include "KeyScan/key_scan.h"
#include "IR/IR.h"

#define reportQEmpty()                        (firstQIdx == lastQIdx)

typedef enum
{
    HID_MODULE_FIRST_ROW,
    HID_MODULE_SECOND_ROW,
    HID_MODULE_THIRD_ROW,
    HID_MODULE_FORTH_ROW
}Hid_indexMapping;

/*********************************************************************
 * CONSTANTS
 */

#define HID_DEV_DATA_LEN                      9
#ifdef HID_DEV_RPT_QUEUE_LEN
  #define HID_DEV_REPORT_Q_SIZE               (HID_DEV_RPT_QUEUE_LEN+1)
#else
  #define HID_DEV_REPORT_Q_SIZE               (10+1)
#endif

#ifndef MIN
#define MIN(n,m)   (((n) < (m)) ? (n) : (m))
#endif

// HID Auto Sync White List configuration parameter. This parameter should be
// set to FALSE if the HID Host (i.e., the Master device) uses a Resolvable
// Private Address (RPA). It should be set to TRUE, otherwise.
#ifndef HID_AUTO_SYNC_WL
  #define HID_AUTO_SYNC_WL                    FALSE
#endif
//*****************************************************************************
//! Functions
//*****************************************************************************
static bStatus_t Hid_initButtons(void);
static bStatus_t Hid_initKeys(void);

/*********************************************************************
 * TYPEDEFS
 */

// Event passed from other profiles.
typedef struct
{
  appEvtHdr_t hdr; // Event header
  uint8_t *pData;  // Event data
} hidDevEvt_t;


typedef struct
{
 uint8_t id;
 uint8_t type;
 uint8_t len;
 uint8_t data[HID_DEV_DATA_LEN];
} hidDevReport_t;

void Hid_buttonPower();
void Hid_buttonInput();
void Hid_buttonReleased();
void Hid_keyOkay();
void Hid_keyPower();
void Hid_keyVolumnUp();
void Hid_keyVolumnDown();
void Hid_keyVolumnMute();
void Hid_keyYoutube();
void Hid_keyNetflix();
void Hid_keyHome();
void Hid_keyDPad_Center();
void Hid_key_Back();
void Hid_keyDPad_Guide();
void Hid_keyAudio();
void Hid_keyProgramDown();
void Hid_keyProgramUp();
void Hid_keyDashboard();
void Hid_keyApp04();
void Hid_keyApp03();
void Hid_ProfileSwitch();
void Hid_keyDPad_Right();
void Hid_keyDPad_Left();
void Hid_keyDPad_Up();
void Hid_keyDPad_Down();

//*****************************************************************************
//! Globals
//*****************************************************************************

/*********************************************************************
 * LOCAL VARIABLES
 */
static Clock_Struct idleTimeoutClock;

static hidRptMap_t *pHidDevRptTbl;

static uint8_t hidDevRptTblLen;

static hidDevCB_t *pHidDevCB;

static hidDevCfg_t *pHidDevCfg;
// Pending reports
static uint8_t firstQIdx = 0;
static uint8_t lastQIdx = 0;
static hidDevReport_t hidDevReportQ[HID_DEV_REPORT_Q_SIZE];

// Whether to change to the preferred connection parameters
static uint8_t updateConnParams = TRUE;

// Last report sent out
static hidDevReport_t lastReport = { 0 };

// State when HID reports are ready to be sent out
// Report ready delay clock
static Clock_Struct reportReadyClock;

// GAP State
static gaprole_States_t hidDevGapState = GAPROLE_INIT;

// TRUE if connection is secure
uint8_t hidDevConnSecure = FALSE;

extern uint16_t connHandle;

// GAP connection handle
static uint16_t gapConnHandle;

// TRUE if pairing in progress
uint8_t hidDevPairingStarted = FALSE;

// Status of last pairing
static uint8_t pairingStatus = SUCCESS;

// Pairing state
static uint8_t hidDevGapBondPairingState = HID_GAPBOND_PAIRING_STATE_NONE;

// State when HID reports are ready to be sent out
static volatile uint8_t hidDevReportReadyState = TRUE;

// Application GAP Role and GAP Bond states
static gaprole_States_t harGapRoleState = GAPROLE_INIT;
static uint8_t harGapBondState = HID_GAPBOND_PAIRING_STATE_NONE;

extern uint8_t conn_established;
static uint8_t audio_key_enabled = 0;

KeypadConfig keypadConfig;
uint8_t factory_reset=0;

#ifdef USE_ZIPIR
uint8_t pressed;
uint8_t ir_function;
IrInput_t ir_input;
#else
PDEIRGENLPF3_Transaction pdeIrTrans;
extern PDEIRLPF3_Handle pdeIrHandle;
extern PDEIRLPF3_Params IR_params;
#endif

// Buttons handles
Button_Handle handle_left;
Button_Handle handle_right;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

#ifdef USE_ZIPIR

extern const U8 gpio;
extern const U8 pwmChannel;

static const uint16_t numEncoding = 136;
static const uint16_t encoded_BlePairing[numEncoding]=
{
    0x80AA,    0x00AA,    0x8015,    0x003F,
    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,    0x8015,    0x0014,
    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x003F,
    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,    0x8015,    0x0014,
    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x003F,
    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x003F,
    0x8015,    0x0014,    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,
    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,
    0x8015,    0x003F,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x06D7,

    0x80AA,    0x00AA,    0x8015,    0x003F,
    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,    0x8015,    0x0014,
    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x003F,
    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,    0x8015,    0x0014,
    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x003F,
    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x003F,
    0x8015,    0x0014,    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,
    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x003F,    0x8015,    0x0014,
    0x8015,    0x003F,    0x8015,    0x0014,    0x8015,    0x0014,    0x8015,    0x06D7,
};

static const GENERIC_IR_TX_CLOCK_SETTING t =
{
 .clockSrcFreq = 65535,
 .invertOutput = 0,
 .modulateFreq = 38008,
 .clockSrc = 65535,
 .extendedSettings = 0,
 .pwmDutyCycleHighCount = 1,
 .pwmDutyCycleLowCount = 2,
};

#endif

// Task events and processing functions.
static void HidDev_clock(char *pData);
// Process reconnection delay
static void HidDev_reportReady(char *pData);
// HID reports.
static hidRptMap_t *HidDev_reportByHandle(uint16_t handle);
static hidRptMap_t *HidDev_reportById(uint8_t id, uint8_t type);
static hidRptMap_t *HidDev_reportByCccdHandle(uint16_t handle);
static void HidDev_enqueueReport(uint8_t id, uint8_t type, uint8_t len, uint8_t *pData);
static hidDevReport_t *HidDev_dequeueReport(void);
static void HidDev_sendReport(uint8_t id, uint8_t type, uint8_t len, uint8_t *pData);
static uint8_t HidDev_sendNoti(uint16_t handle, uint8_t len, uint8_t *pData);
static void Hid_Send_Report(char *pData);


static void HIDAdvRemote_handleEventCB(uint8_t evt)
{
  uint32_t hwiKey;

  switch(evt)
  {
    case(HID_DEV_GAPROLE_STATE_CHANGE_EVT):
      // Update application GAP Role state
      HidDev_GetParameter(HIDDEV_GAPROLE_STATE, &harGapRoleState);
      break;

    case(HID_DEV_GAPBOND_STATE_CHANGE_EVT):
      // Update application GAP Bond pairing state
      HidDev_GetParameter(HIDDEV_GAPBOND_STATE, &harGapBondState);

      break;

    default:
      // Do nothing
      break;
  }

  return;
}

// HID Dev callbacks
static hidDevCB_t hidAdvRemoteHidCBs =
{
  HIDAdvRemote_handleEventCB,
};

// HID Dev configuration
static hidDevCfg_t hidAdvRemoteCfg =
{
  DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
  HID_KBD_FLAGS               // HID feature flags
};

bStatus_t HidDev_start(void)
{
  bStatus_t status = SUCCESS;
  uint8_t  *devAddr = NULL;

  Batt_AddService();

  Util_constructClock(&reportReadyClock, (void *)BLEAppUtil_invokeFunctionNoData,
                      HID_REPORT_READY_TIME, 0, false, (uint32)HidDev_reportReady);

  // Add HID keyboard service
  status = HidKbdCC_AddService();
  // Register for HID Dev callback
  HidDev_Register(&hidAdvRemoteCfg, &hidAdvRemoteHidCBs);


  Hid_initButtons();
  Hid_initKeys();


  return status;
}

/*********************************************************************
 * @fn      HidDev_reportReadyClockCB
 *
 * @brief   Handles HID reports when delay has expired
 *
 * @param   None.
 *
 * @return  None.
 */
static void HidDev_reportReady( char *pData )
{
    // Allow reports to be sent
    hidDevReportReadyState = TRUE;
    // If there are reports in the queue
     if (!reportQEmpty())
     {
         // If connection is secure
         if (hidDevConnSecure && hidDevReportReadyState)
         {
           hidDevReport_t *pReport = HidDev_dequeueReport();

           if (pReport != NULL)
           {
             // Send report.
             HidDev_sendReport(pReport->id, pReport->type, pReport->len,
                               pReport->data);
           }

           // If there is another report in the queue
           if (!reportQEmpty())
           {
             Util_rescheduleClock(&reportReadyClock,1);

           }
         }

     }
}

/*********************************************************************
 * @fn      HidDev_Register
 *
 * @brief   Register a callback function with HID Dev.
 *
 * @param   pCfg         - Parameter configuration.
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
void HidDev_Register(hidDevCfg_t *pCfg, hidDevCB_t *pCBs)
{
  pHidDevCB = pCBs;
  pHidDevCfg = pCfg;

  // If configured and not zero, create the idle timeout clock.
  if ((pHidDevCfg != NULL) && (pHidDevCfg->idleTimeout != 0))
  {
    Util_constructClock(&idleTimeoutClock, (void *)BLEAppUtil_invokeFunctionNoData,
                        pHidDevCfg->idleTimeout, 0, false, (uint32)HidDev_clock);

  }
}

/*********************************************************************
 * @fn      HidDev_RegisterReports
 *
 * @brief   Register the report table with HID Dev.
 *
 * @param   numReports - Length of report table.
 * @param   pRpt       - Report table.
 *
 * @return  None.
 */
void HidDev_RegisterReports(uint8_t numReports, hidRptMap_t *pRpt)
{
  pHidDevRptTbl = pRpt;
  hidDevRptTblLen = numReports;
}


/*********************************************************************
 * @fn      HidDev_Report
 *
 * @brief   Send a HID report.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   len   - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
void HidDev_Report(uint8_t id, uint8_t type, uint8_t len, uint8_t *pData)
{
  // Validate length of report
  if ( len > HID_DEV_DATA_LEN )
  {
    return;
  }

/*  // If connected
  if (hidDevGapState == GAPROLE_CONNECTED)
  {*/
    // If connection is secure
    if (hidDevConnSecure)
    {
      // Make sure there're no pending reports.
      if (reportQEmpty())
      {
        // Send report.
        HidDev_sendReport(id, type, len, pData);
        return;
      }
    }
  //}
  // HidDev task will send report when secure connection is established.
  HidDev_enqueueReport(id, type, len, pData);
}

/*********************************************************************
 * @fn      HidDev_Close
 *
 * @brief   Close the connection or stop advertising.
 *
 * @param   None.
 *
 * @return  None.
 */
void HidDev_Close(void)
{
  uint8_t param;

  // If connected then disconnect.
  if (hidDevGapState == GAPROLE_CONNECTED)
  {
    BLEAppUtil_disconnect(gapConnHandle);
  }
}

/*********************************************************************
 * @fn      HidDev_GetParameter
 *
 * @brief   Get a HID Dev parameter.
 *
 * @param   param  - profile parameter ID
 * @param   pValue - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t HidDev_GetParameter(uint8_t param, void *pValue)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case HIDDEV_GAPROLE_STATE:
      *((uint8_t*)pValue) = hidDevGapState;
      break;

    case HIDDEV_GAPBOND_STATE:
      *((uint8_t*)pValue) = hidDevGapBondPairingState;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      HidDev_reportByCccdHandle
 *
 * @brief   Find the HID report structure for the given CCC handle.
 *
 * @param   handle - ATT handle
 *
 * @return  Pointer to HID report structure
 */
static hidRptMap_t *HidDev_reportByCccdHandle(uint16_t handle)
{
  uint8_t i;
  hidRptMap_t *p = pHidDevRptTbl;

  for (i = hidDevRptTblLen; i > 0; i--, p++)
  {
    if ((p->pCccdAttr != NULL) && (p->pCccdAttr->handle == handle))
    {
      return p;
    }
  }

  return NULL;
}


/*********************************************************************
 * @fn      HidDev_reportById
 *
 * @brief   Find the HID report structure for the Report ID and type.
 *
 * @param   id   - HID report ID
 * @param   type - HID report type
 *
 * @return  Pointer to HID report structure
 */
static hidRptMap_t *HidDev_reportById(uint8_t id, uint8_t type)
{
  uint8_t i;
  hidRptMap_t *p = pHidDevRptTbl;

  for (i = hidDevRptTblLen; i > 0; i--, p++)
  {
    if (p->id == id && p->type == type && p->mode == hidProtocolMode)
    {
      return p;
    }
  }

  return NULL;
}

/*********************************************************************
 * @fn      HidDev_sendReport
 *
 * @brief   Send a HID report.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   len   - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
static void HidDev_sendReport(uint8_t id, uint8_t type, uint8_t len, uint8_t *pData)
{
  hidRptMap_t *pRpt;
  // Get ATT handle for report.
  if ((pRpt = HidDev_reportById(id, type)) != NULL)
  {
    uint8_t value = GATTServApp_ReadCharCfg(gapConnHandle, GATT_CCC_TBL(pRpt->pCccdAttr->pValue));
    // If notifications are enabled
     //if (value & GATT_CLIENT_CFG_NOTIFY)
     //{
      // Send report notification
      if (HidDev_sendNoti(pRpt->handle, len, pData) == SUCCESS)
      {
        // Save the report just sent out
        lastReport.id = id;
        lastReport.type = type;
        lastReport.len = len;
        memcpy(lastReport.data, pData, len);
      }
     //}
  }
}


/*********************************************************************
 * @fn      hidDevSendNoti
 *
 * @brief   Send a HID notification.
 *
 * @param   handle - Attribute handle.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  Success or failure.
 */
static uint8_t HidDev_sendNoti(uint16_t handle, uint8_t len, uint8_t *pData)
{
  uint8_t status;
  attHandleValueNoti_t noti;


  noti.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_NOTI, len, NULL);
  if (noti.pValue != NULL)
  {
    noti.handle = handle;
    noti.len = len;
    memcpy(noti.pValue, pData, len);

    // Send notification
    status = GATT_Notification(gapConnHandle, &noti, FALSE);
    if (status != SUCCESS)
    {
      GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
    }
  }
  else
  {
    status = bleMemAllocError;
  }

  return status;
}

/*********************************************************************
 * @fn      HidDev_enqueueReport
 *
 * @brief   Enqueue a HID report to be sent later.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   len   - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
static void HidDev_enqueueReport(uint8_t id, uint8_t type, uint8_t len,
                                 uint8_t *pData)
{
  uint8_t bondCnt = 0;
  VOID GAPBondMgr_GetParameter(GAPBOND_BOND_COUNT, &bondCnt);
  // Enqueue only if bonded.
  if (bondCnt > 0)
  {
    // Update last index.
    lastQIdx = (lastQIdx + 1) % HID_DEV_REPORT_Q_SIZE;

    if (lastQIdx == firstQIdx)
    {
      // Queue overflow; discard oldest report.
      firstQIdx = (firstQIdx + 1) % HID_DEV_REPORT_Q_SIZE;
    }

    // Save report.
    hidDevReportQ[lastQIdx].id = id;
    hidDevReportQ[lastQIdx].type = type;
    hidDevReportQ[lastQIdx].len = len;
    memcpy(hidDevReportQ[lastQIdx].data, pData, len);

    if (hidDevConnSecure)
    {
      // Notify our task to send out pending reports.
        BLEAppUtil_invokeFunctionNoData(Hid_Send_Report);

    }
  }
}


/*********************************************************************
 * @fn      HidDev_dequeueReport
 *
 * @brief   Dequeue a HID report to be sent out.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   len   - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
static hidDevReport_t *HidDev_dequeueReport(void)
{
  if (reportQEmpty())
  {
    return NULL;
  }

  // Update first index.
  firstQIdx = (firstQIdx + 1) % HID_DEV_REPORT_Q_SIZE;

  return (&(hidDevReportQ[firstQIdx]));
}


/*********************************************************************
 * @fn      HidDev_clockHandler
 *
 * @brief   Clock handle for all clock events.  This function stores an event
 *          flag and wakes up the application's event processor.
 *
 * @param   arg - event flag.
 *
 * @return  None
 */
static void HidDev_clock(char *pData)
{
    uint16_t connHandle=0;
    uint8_t index=0;
    if (hidDevGapState == GAPROLE_CONNECTED)
    {
        // If pairing in progress then restart timer.
        if (hidDevPairingStarted)
        {
        }
        // Else disconnect and don't allow reports to be sent
        else
        {
          hidDevReportReadyState = FALSE;

          for(index =0 ; index< MAX_NUM_BLE_CONNS ; index++)
          {
              connHandle = Connection_getConnhandle(index);
              if(MAX_NUM_BLE_CONNS != connHandle)
              {
                  break;
              }
          }
          BLEAppUtil_disconnect(connHandle);

        }
    }

}

/*****************************************************
 * @fn          HidDev_ReadAttrCB
 *
 * @brief       HID Dev attribute read callback.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr      - pointer to attribute
 * @param       pValue     - pointer to data to be read
 * @param       pLen       - length of data to be read
 * @param       offset     - offset of the first octet to be read
 * @param       maxLen     - maximum length of data to be read
 * @param       method     - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
bStatus_t HidDev_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                            uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                            uint16_t maxLen, uint8_t method)
{
  bStatus_t   status = SUCCESS;
  hidRptMap_t *pRpt;

  uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

  // Only report map is long.
  if (offset > 0 && uuid != REPORT_MAP_UUID)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  if (uuid == REPORT_UUID ||
      uuid == BOOT_KEY_INPUT_UUID ||
      uuid == BOOT_KEY_OUTPUT_UUID ||
      uuid == BOOT_MOUSE_INPUT_UUID)
  {
    // Find report ID in table.
    if ((pRpt = HidDev_reportByHandle(pAttr->handle)) != NULL)
    {
      // Execute report callback.
      status  = (*pHidDevCB->reportCB)(pRpt->id, pRpt->type, uuid,
                                        HID_DEV_OPER_READ, pLen, pValue);
    }
    else
    {
      *pLen = 0;
    }
  }
  else if (uuid == REPORT_MAP_UUID)
  {
    // If the value offset of the Read Blob Request is greater than the
    // length of the attribute value, an Error Response shall be sent with
    // the error code Invalid Offset.
    if (offset > hidReportMapLen)
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Determine read length.
      *pLen = MIN(maxLen, (hidReportMapLen - offset));

      // Copy data.
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else if (uuid == HID_INFORMATION_UUID)
  {
    *pLen = HID_INFORMATION_LEN;
    memcpy(pValue, pAttr->pValue, HID_INFORMATION_LEN);
  }
  else if (uuid == GATT_REPORT_REF_UUID)
  {
    *pLen = HID_REPORT_REF_LEN;
    memcpy(pValue, pAttr->pValue, HID_REPORT_REF_LEN);
  }
  else if (uuid == PROTOCOL_MODE_UUID)
  {
    *pLen = HID_PROTOCOL_MODE_LEN;
    pValue[0] = pAttr->pValue[0];
  }
  else if (uuid == GATT_EXT_REPORT_REF_UUID)
  {
    *pLen = HID_EXT_REPORT_REF_LEN;
    memcpy(pValue, pAttr->pValue, HID_EXT_REPORT_REF_LEN);
  }
  // Restart idle timer.
  if (status == SUCCESS)
  {
#if IDLE_TIMER_EN
          HidDev_StartIdleTimer();
#endif
  }

  return (status);
}


/*********************************************************************
 * @fn      HidDev_WriteAttrCB
 *
 * @brief   HID Dev attribute write callback.
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr      - pointer to attribute
 * @param   pValue     - pointer to data to be written
 * @param   len        - length of data
 * @param   offset     - offset of the first octet to be written
 * @param   method     - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
bStatus_t HidDev_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                             uint8_t *pValue, uint16_t len, uint16_t offset,
                             uint8_t method)
{
  bStatus_t   status = SUCCESS;
  hidRptMap_t *pRpt;

  // Make sure it's not a blob operation (no attributes in the profile are long).
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

  if (uuid == REPORT_UUID || uuid == BOOT_KEY_OUTPUT_UUID)
  {
    // Find report ID in table.
    if ((pRpt = HidDev_reportByHandle(pAttr->handle)) != NULL)
    {
      // Execute report callback.
      status  = (*pHidDevCB->reportCB)(pRpt->id, pRpt->type, uuid,
                                        HID_DEV_OPER_WRITE, &len, pValue);
    }
  }
  else if (uuid == HID_CTRL_PT_UUID)
  {
    // Validate length and value range.
    if (len == 1)
    {
      if (pValue[0] == HID_CMD_SUSPEND ||  pValue[0] == HID_CMD_EXIT_SUSPEND)
      {
        // Execute HID app event callback.
        (*pHidDevCB->evtCB)((pValue[0] == HID_CMD_SUSPEND) ?
                             HID_DEV_SUSPEND_EVT : HID_DEV_EXIT_SUSPEND_EVT);
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE;
      }
    }
    else
    {
      status = ATT_ERR_INVALID_VALUE_SIZE;
    }
  }
  else if (uuid == GATT_CLIENT_CHAR_CFG_UUID)
  {
    status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                            offset, GATT_CLIENT_CFG_NOTIFY);
    if (status == SUCCESS)
    {
      uint16_t charCfg = BUILD_UINT16(pValue[0], pValue[1]);

      // Find report ID in table.
      if ((pRpt = HidDev_reportByCccdHandle(pAttr->handle)) != NULL)
      {
        // Execute report callback.
        (*pHidDevCB->reportCB)(pRpt->id, pRpt->type, uuid,
                               (charCfg == GATT_CLIENT_CFG_NOTIFY) ?
                               HID_DEV_OPER_ENABLE : HID_DEV_OPER_DISABLE,
                               &len, pValue);
      }
    }
  }
  else if (uuid == PROTOCOL_MODE_UUID)
  {
    if (len == HID_PROTOCOL_MODE_LEN)
    {
      if (pValue[0] == HID_PROTOCOL_MODE_BOOT ||
          pValue[0] == HID_PROTOCOL_MODE_REPORT)
      {
        pAttr->pValue[0] = pValue[0];

        // Execute HID app event callback.
        (*pHidDevCB->evtCB)((pValue[0] == HID_PROTOCOL_MODE_BOOT) ?
                            HID_DEV_SET_BOOT_EVT : HID_DEV_SET_REPORT_EVT);
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE;
      }
    }
    else
    {
      status = ATT_ERR_INVALID_VALUE_SIZE;
    }
  }

  // Restart idle timer.
  if (status == SUCCESS)
  {
#if IDLE_TIMER_EN
          HidDev_StartIdleTimer();
#endif
  }
  return (status);
}

/*********************************************************************
 * @fn      HidDev_StartIdleTimer
 *
 * @brief   Start the idle timer.
 *
 * @return  None.
 */
void HidDev_StartIdleTimer(void)
{
  if ((pHidDevCfg != NULL) && (pHidDevCfg->idleTimeout > 0))
  {
    Util_startClock(&idleTimeoutClock);
  }

}

/*********************************************************************
 * @fn      HidDev_StopIdleTimer
 *
 * @brief   Stop the idle timer.
 *
 * @param   None.
 *
 * @return  None.
 */
void HidDev_StopIdleTimer(void)
{
  if ((pHidDevCfg != NULL) && (pHidDevCfg->idleTimeout > 0))
  {
    Util_stopClock(&idleTimeoutClock);
  }
}

/*********************************************************************
 * @fn      HidDev_reportByHandle
 *
 * @brief   Find the HID report structure for the given handle.
 *
 * @param   handle - ATT handle
 *
 * @return  Pointer to HID report structure
 */
static hidRptMap_t *HidDev_reportByHandle(uint16_t handle)
{
  uint8_t i;
  hidRptMap_t *p = pHidDevRptTbl;

  for (i = hidDevRptTblLen; i > 0; i--, p++)
  {
    if (p->handle == handle && p->mode == hidProtocolMode)
    {
      return p;
    }
  }

  return NULL;
}


static void Hid_Send_Report(char *pData)
{
    // If connection is secure
    if (hidDevConnSecure && hidDevReportReadyState)
    {
      hidDevReport_t *pReport = HidDev_dequeueReport();
      if (pReport != NULL)
      {
        // Send report.
        HidDev_sendReport(pReport->id, pReport->type, pReport->len, pReport->data);
      }
      // If there is another report in the queue
      if (!reportQEmpty())
      {
        BLEAppUtil_invokeFunctionNoData(Hid_Send_Report);
      }
    }
}

/*********************************************************************
 * @fn      MenuModule_buttonsCallback
 *
 * @brief   Buttons callback, it will be called once one of the events
 *          provided to the button module will occur
 *
 * @param   buttonHandle - the button handle
 * @param   buttonEvents - the button event
 *
 * @return  None
 */

uint8_t stop_audio = 0;
uint8_t first = 0;
uint8_t keypressed = 0;

void HidModule_buttonsCallback(Button_Handle buttonHandle, Button_EventMask buttonEvents)
{
    InvokeFromBLEAppUtilContext_t callbackSelection[2][2] =
    {
     {Hid_buttonPower,      Hid_buttonInput},
     {Hid_buttonReleased,   NULL}
    };

    uint8 whichButton = (buttonHandle == handle_left) ? 0 : 1;
    uint8 whichPress;

    switch(buttonEvents)
    {
        case Button_EV_PRESSED:
        {
            whichPress = 0;
            break;
        }
        case Button_EV_RELEASED:
        {
            whichPress = 1;
            whichButton = 0;
            break;
        }
    }

    if (conn_established == 1) {
        RestartShutdownClock();
    }
    // Switch the context to the BLE App Util task context
    BLEAppUtil_invokeFunctionNoData(callbackSelection[whichPress][whichButton]);
}

/*********************************************************************
 * @fn      Hid_initButtons
 *
 * @brief   Initialize the buttons using the buttons module APIs
 *
 * @return  SUCCESS, FAILURE
 */

static bStatus_t Hid_initButtons(void)
{
    Button_Params params;
    Button_Params_init(&params);

    // Set the buttons parameters
    params.buttonEventMask = Button_EV_PRESSED | Button_EV_RELEASED;
    params.buttonCallback = HidModule_buttonsCallback;
    params.debounceDuration = HID_MODULE_DEBOUNCE_DURATION;
    // Open the buttons
    handle_left = Button_open(CONFIG_BUTTON_0, &params);
    handle_right = Button_open(CONFIG_BUTTON_1, &params);

    if(handle_left == NULL || handle_right == NULL)
    {
        return FAILURE;
    }
    return SUCCESS;
}

/*********************************************************************
 * @fn      Hid_initKeys
 *
 * @brief   Initialize the keys using the keys module API
 *
 * @return  SUCCESS, FAILURE
 */

static bStatus_t Hid_initKeys(void)
{
    // Initialize KeyPad configuration parameters
    keypadConfig.rowPins[0] = CONFIG_KEY_ROW_1;
    keypadConfig.rowPins[1] = CONFIG_KEY_ROW_2;
    keypadConfig.rowPins[2] = CONFIG_KEY_ROW_3;
    keypadConfig.rowPins[3] = CONFIG_KEY_ROW_4;

    keypadConfig.colPins[0] = CONFIG_KEY_COL_1;
    keypadConfig.colPins[1] = CONFIG_KEY_COL_2;
    keypadConfig.colPins[2] = CONFIG_KEY_COL_3;
    keypadConfig.colPins[3] = CONFIG_KEY_COL_4;
    keypadConfig.colPins[4] = CONFIG_KEY_COL_5;

    keypadConfig.debounceDelay_us = 1000000;
    keypadConfig.max_pressed_keys = 3;
    Keypad_init(&keypadConfig);
    return 0;
}

uint8 zipIRclock_started;

// Task for handling keypad events
void keypadTask(char *key_action_type) {

    PressedKeys keys;
    char key_type = key_action_type[0];

    if (key_type == 1){
        // Read the currently pressed key
        if (Keypad_read(&keys, &keypadConfig) == 1) {

            if (conn_established == 1) {
                RestartShutdownClock();
            }

           // How many keys pressed at the same time.
            if (keys.key_count == 1){
                uint8_t i = 0;
                if ((keys.pressedKeys[i].row == 0) && (keys.pressedKeys[i].col == 0)){
                    Hid_keyHome();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 0) && (keys.pressedKeys[i].col == 1)){
                    Hid_keyDPad_Guide();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 0) && (keys.pressedKeys[i].col == 2)){
                    Hid_ProfileSwitch();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 0) && (keys.pressedKeys[i].col == 3)){
                    Hid_key_Back();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 0) && (keys.pressedKeys[i].col == 4)){
                    Hid_keyAudio();
                }

                else if ((keys.pressedKeys[i].row == 1) && (keys.pressedKeys[i].col == 0)){
                    Hid_keyDPad_Up();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 1) && (keys.pressedKeys[i].col == 1)){
                    Hid_keyDPad_Down();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 1) && (keys.pressedKeys[i].col == 2)){
                    Hid_keyDPad_Right();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 1) && (keys.pressedKeys[i].col == 3)){
                    Hid_keyDPad_Left();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 1) && (keys.pressedKeys[i].col == 4)){
                    Hid_keyDPad_Center();
                    //Hid_buttonReleased();
                }

                else if ((keys.pressedKeys[i].row == 2) && (keys.pressedKeys[i].col == 0)){
                    Hid_keyProgramUp();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 2) && (keys.pressedKeys[i].col == 1)){
                    Hid_keyProgramDown();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 2) && (keys.pressedKeys[i].col == 2)){
                    Hid_keyVolumnMute();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 2) && (keys.pressedKeys[i].col == 3)){
                    Hid_keyVolumnUp();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 2) && (keys.pressedKeys[i].col == 4)){
                    Hid_keyVolumnDown();
                    //Hid_buttonReleased();
                }

                else if ((keys.pressedKeys[i].row == 3) && (keys.pressedKeys[i].col == 0)){
                    Hid_keyYoutube();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 3) && (keys.pressedKeys[i].col == 1)){
                    Hid_keyNetflix();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 3) && (keys.pressedKeys[i].col == 2)){
                    Hid_keyApp04();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 3) && (keys.pressedKeys[i].col == 3)){
                    Hid_keyApp03();
                    //Hid_buttonReleased();
                }
                else if ((keys.pressedKeys[i].row == 3) && (keys.pressedKeys[i].col == 4)){
                    Hid_keyDashboard();
                    //Hid_buttonReleased();
                }
            }
            // Multiple Keys pressed at same time.
            else if (keys.key_count == 2){
                // Pairing Mode & Re-Pairing: [HOME]+[BACK] for longer than 4s.
                if ((keys.pressedKeys[0].row == 0) && (keys.pressedKeys[0].col == 0) && (keys.pressedKeys[1].row == 0) && (keys.pressedKeys[1].col == 3)){
                    //Set bit to go back to Undirect Adv
/*                    uint8_t backtoUndirecAdv[B_ADDR_LEN+1] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
                    osal_snv_write(BLE_NVID_CUST_START, (B_ADDR_LEN + 1), (uint8 *)backtoUndirecAdv);
                    factory_reset = 1;
                    Hid_buttonReleased();
                    GAP_TerminateLinkReq(connHandle, HCI_DISCONNECT_REMOTE_USER_TERM);*/
                    GPIO_write(CONFIG_GPIO_LED_GREEN,1);
                    #ifdef USE_ZIPIR
                    ConfigZIPIRPairing_start();
                    zipIRclock_started = 1;
                    #endif
                }
                // Factory Reset Mode (Clear Bond Cache): [DPAD CENTER]+[MUTE] for longer than 4s.
                if ((keys.pressedKeys[0].row == 1) && (keys.pressedKeys[0].col == 4) && (keys.pressedKeys[1].row == 2) && (keys.pressedKeys[1].col == 2)){
                    //Set bit to go back to Undirected Adv
/*                    uint8_t backtoUndirecAdv[B_ADDR_LEN+1] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
                    osal_snv_write(BLE_NVID_CUST_START, (B_ADDR_LEN + 1), (uint8 *)backtoUndirecAdv);
                    factory_reset = 1;
                    Hid_buttonReleased();
                    GAP_TerminateLinkReq(connHandle, HCI_DISCONNECT_REMOTE_USER_TERM);*/
                    GPIO_write(CONFIG_GPIO_LED_RED,1);
                }
                // Google Bug Report Mode: [DPAD CENTER]+[BACK] for longer than 1s (In IR mode, the sent code data is 0x96.)
                if ((keys.pressedKeys[0].row == 1) && (keys.pressedKeys[0].col == 4) && (keys.pressedKeys[1].row == 0) && (keys.pressedKeys[1].col == 3)){
                    // TBD
                }
                // Google Accessibility shortcut: [DPAD DOWN]+[BACK] for longer than 1s.
                if ((keys.pressedKeys[0].row == 2) && (keys.pressedKeys[0].col == 1) && (keys.pressedKeys[1].row == 0) && (keys.pressedKeys[1].col == 3)){
                    // TBD
                }
            }

            else if (keys.key_count == 3){
                // Couch Mode: More than Max Keys pressed at same time, RC will disconnect and go to shutdown.
                //Set bit to go back to Undirected Adv
                uint8_t backtoUndirecAdv[B_ADDR_LEN+1] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
                osal_snv_write(BLE_NVID_CUST_START, (B_ADDR_LEN + 1), (uint8 *)backtoUndirecAdv);
                factory_reset = 1;
                Hid_buttonReleased();
                GAP_TerminateLinkReq(connHandle, HCI_DISCONNECT_REMOTE_USER_TERM);
            }
        }
    }
    else {
        //For voice press and hold speak mode
        if (zipIRclock_started == 1){
            StopZIPIRClock();
            zipIRclock_started = 0;
        }
        Hid_buttonReleased();
    }

    // Configure keypad rows as inputs with pull-down resistor and interrupts enabled for button release detection.
    GPIO_setConfig(CONFIG_KEY_ROW_1, GPIO_CFG_IN_PD | GPIO_CFG_HYSTERESIS_ON | GPIO_CFG_INT_BOTH_EDGES_INTERNAL | GPIO_CFG_INT_ENABLE);
    GPIO_setConfig(CONFIG_KEY_ROW_2, GPIO_CFG_IN_PD | GPIO_CFG_HYSTERESIS_ON | GPIO_CFG_INT_BOTH_EDGES_INTERNAL | GPIO_CFG_INT_ENABLE);
    GPIO_setConfig(CONFIG_KEY_ROW_3, GPIO_CFG_IN_PD | GPIO_CFG_HYSTERESIS_ON | GPIO_CFG_INT_BOTH_EDGES_INTERNAL | GPIO_CFG_INT_ENABLE);
    GPIO_setConfig(CONFIG_KEY_ROW_4, GPIO_CFG_IN_PD | GPIO_CFG_HYSTERESIS_ON | GPIO_CFG_INT_BOTH_EDGES_INTERNAL | GPIO_CFG_INT_ENABLE);
}

static void HIDAdvRemote_sendReport(uint16_t keycode)
{
  uint8_t consumer_key[4];

  consumer_key[0] = LO_UINT16(keycode);
  consumer_key[1] = HI_UINT16(keycode);
  consumer_key[2] = 0x00;
  consumer_key[3] = 0x00;
  HidDev_Report(HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT, 4, consumer_key);
}

// KEYS HID:

void Hid_ProfileSwitch()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_PROFILE_SWITCH);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_PROFILE_SWITCH;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyProgramUp()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_CHANNEL_UP);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x7;
        pdeIrTrans.dataCode = 0x12;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyProgramDown()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_CHANNEL_DOWN);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x7;
        pdeIrTrans.dataCode = 0x10;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyDashboard()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_SETTINGS);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_TV;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

uint8_t buf_audio_start[4] = {0x04, 0x03, 0x02, 0x00};

#ifndef AUDIO_TEST
void Hid_keyAudio()
{
    if (conn_established == 1 && audio_key_enabled == 0) {

        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }

        buf_audio_start[3]++; //increase stream_id
        if(buf_audio_start[3] >= 0x80) {
            buf_audio_start[3] = 1; //stream_id range 0x01~0x80
        }

        stop_audio = 0;
        Audio_sendBufferOverBLE(buf_audio_start, sizeof(buf_audio_start), 2);
        Audio_enable();
        audio_key_enabled = 1;
    }
}
#endif

#ifdef AUDIO_TEST
void Hid_keyAudio()
{
    //GPIO_write(CONFIG_GPIO_LED_GREEN,1);
    GPIO_write(CONFIG_GPIO_LED_RED,1);
    Audio_enable();
}
#endif

void Hid_keyDPad_Guide()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_GUIDE);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_GUIDE;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_key_Back()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_BACK);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_BACK;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyDPad_Center()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_DPAD_CENTER);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_DPAD_CENTER;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyDPad_Left()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_DPAD_LEFT);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_DPAD_LEFT;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyDPad_Right()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_DPAD_RIGHT);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_DPAD_RIGHT;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyDPad_Up()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_DPAD_UP);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_DPAD_UP;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyDPad_Down()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_DAPD_DOWN);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_DAPD_DOWN;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyHome()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_HOME);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_HOME;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyVolumnDown()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_VOLUME_DOWN);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR
        ZipIR_Init(gpio, pwmChannel);
        ir_input.keyFlag    = 0;
        ir_input.keyCounter = 4;
        ir_input.function   = 0;
        ir_input.codesetId  = ZIPIR_T2051_voldn;
        ir_input.irCallback = 0;
        ir_input.duration   = 0;
        ZipIR_Transmit(&ir_input);
        #else
        pdeIrTrans.customCode = 0x7;
        pdeIrTrans.dataCode = 0xB;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyVolumnUp()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_VOLUME_UP);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR
        ZipIR_Init(gpio, pwmChannel);
        ir_input.keyFlag    = 0;
        ir_input.keyCounter = 3;
        ir_input.function   = 0;
        ir_input.codesetId  = ZIPIR_T2051_volup;
        ir_input.irCallback = 0;
        ir_input.duration   = 0;
        ZipIR_Transmit(&ir_input);
        #else
        pdeIrTrans.customCode = 0x7;
        pdeIrTrans.dataCode = 0x7;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyVolumnMute()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_VOLUME_MUTE);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR
        ZipIR_Init(gpio, pwmChannel);
        ir_input.keyFlag    = 0;
        ir_input.keyCounter = 2;
        ir_input.function   = 0;
        ir_input.codesetId  = ZIPIR_T2051_mute;
        ir_input.irCallback = 0;
        ir_input.duration   = 0;
        ZipIR_Transmit(&ir_input);
        #else
        pdeIrTrans.customCode = 0x7;
        pdeIrTrans.dataCode = 0xF;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}


void Hid_keyApp04()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_F20);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_F20;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyApp03()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_F19);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_F19;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyYoutube()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_F17);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_F17;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_keyNetflix()
{
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_F18);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR

        #else
        pdeIrTrans.customCode = 0x40;
        pdeIrTrans.dataCode = IR_KEYCODE_F18;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

uint8_t buf_audio_stop[2] = {0x00, 0x02};

#ifndef AUDIO_TEST
void Hid_buttonReleased()
{
    GPIO_write(CONFIG_GPIO_LED_GREEN,0);
    GPIO_write(CONFIG_GPIO_LED_RED,0);
    keypressed = 0;

    if (conn_established == 1) {
        if (audio_key_enabled == 1) {
            stop_audio = 1;
            Audio_disable();
            Audio_sendBufferOverBLE(buf_audio_stop, sizeof(buf_audio_stop), 2);
            audio_key_enabled = 0;
        }
        else{
            HIDAdvRemote_sendReport(KEY_NONE);
        }
    }
    else {
        #ifdef USE_ZIPIR
        ZipIR_Stop(STOP_IR);
        #endif
    }
}
#endif

#ifdef AUDIO_TEST
void Hid_buttonReleased()
{
    //Do nothing.
}
#endif

//BUTTONS HID:

void Hid_buttonPower()
{
    //write_nvs_ext();
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_POWER);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR
        ZipIR_Init(gpio, pwmChannel);
        ir_input.keyFlag    = 0;
        ir_input.keyCounter = 1;
        ir_input.function   = 0;
        ir_input.codesetId  = ZIPIR_T2051_power;
        ir_input.irCallback = 0;
        ir_input.duration   = 0;
        ZipIR_Transmit(&ir_input);
        #else
        pdeIrTrans.customCode = 0x7; // from SM remote
        pdeIrTrans.dataCode = 0x2; // from SM remote
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        #endif
    }
}

void Hid_buttonInput()
{
    //read_nvs_ext();
    if (conn_established == 1) {
        HIDAdvRemote_sendReport(KEYCODE_TV_INPUT);
        if (hidDevConnSecure) {
            GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        }
        else {
            GPIO_write(CONFIG_GPIO_LED_RED,1);
        }
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);

        #ifdef USE_ZIPIR
        ZipIR_Init(gpio, pwmChannel);
        //Power_setConstraint(PowerLPF3_DISALLOW_STANDBY);
        GENERIC_IR_BT_CLOCK_ENABLE(TRUE);
        GENERIC_IRTX_SEND_DATA(encoded_BlePairing, numEncoding, t);
        // at this point, safe to assume the full data is sent
        GENERIC_IRTX_ABORT_CURRENT_TRANSACTION();
        GENERIC_IR_BT_CLOCK_ENABLE(FALSE);
        }

        #else
        pdeIrTrans.customCode = 0x4;
        pdeIrTrans.dataCode = 0xB;
        pdeIrHandle = PDEIRLPF3_open(PDEIRLPF3_CONFIG_0, &IR_params);
        PDEIRLPF3_transfer(pdeIrHandle, &pdeIrTrans);
        usleep(125 * 1000);
        PDEIRLPF3_stop(pdeIrHandle);
        PDEIRLPF3_close(pdeIrHandle);
        }
        #endif
}

