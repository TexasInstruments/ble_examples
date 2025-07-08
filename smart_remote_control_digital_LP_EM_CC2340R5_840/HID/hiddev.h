/******************************************************************************

@file  hiddev.h

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

#ifndef HIDDEV_H
#define HIDDEV_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>

/*********************************************************************
 * CONSTANTS
 */

// Number of HID reports defined in the service
//#define HID_NUM_REPORTS          6

// HID Report IDs for the service
#define HID_RPT_ID_KEY_IN        1  // Keyboard input report ID
#define HID_RPT_ID_CC_IN         2  // Consumer Control input report ID
#define HID_RPT_ID_LED_OUT       0  // LED output report ID

// HID feature flags
#define HID_KBD_FLAGS            HID_FLAGS_REMOTE_WAKE

// Initial GAP Bond pairing state
#define HID_GAPBOND_PAIRING_STATE_NONE    0xFF  //!< Not connected/paired

// HID Device Parameters
#define HIDDEV_ERASE_ALLBONDS       0x00  // Erase all of the bonded devices.
                                          // Write Only. No Size.
#define HIDDEV_GAPROLE_STATE        0x01  // Reading this parameter will return
                                          // the HID Dev GAP Peripheral Role
                                          // State. Read Only. Size is uint8_t.
#define HIDDEV_GAPBOND_STATE        0x02  // Reading this parameter will return
                                          // the HID Dev GAP Bond Manager
                                          // Pairing State. Read Only.
                                          // Size is uint8_t.
// HID read/write operation
#define HID_DEV_OPER_WRITE          0  // Write operation
#define HID_DEV_OPER_READ           1  // Read operation
#define HID_DEV_OPER_ENABLE         2  // Notification enabled for report ID
#define HID_DEV_OPER_DISABLE        3  // Notifications disabled for report ID

// HID callback events
#define HID_DEV_SUSPEND_EVT               0  // HID suspend
#define HID_DEV_EXIT_SUSPEND_EVT          1  // HID exit suspend
#define HID_DEV_SET_BOOT_EVT              2  // HID set boot mode
#define HID_DEV_SET_REPORT_EVT            3  // HID set report mode
#define HID_DEV_GAPROLE_STATE_CHANGE_EVT  4  // HID GAP Role state change
#define HID_DEV_GAPBOND_STATE_CHANGE_EVT  5  // HID GAP Bond state change

/* HID Report type */
#define HID_REPORT_TYPE_INPUT       1
#define HID_REPORT_TYPE_OUTPUT      2
#define HID_REPORT_TYPE_FEATURE     3

/* HID information flags */
#define HID_FLAGS_REMOTE_WAKE           0x01 // RemoteWake
#define HID_FLAGS_NORMALLY_CONNECTABLE  0x02 // NormallyConnectable

/* Control point commands */
#define HID_CMD_SUSPEND             0x00 // Suspend
#define HID_CMD_EXIT_SUSPEND        0x01 // Exit Suspend

/* HID protocol mode values */
#define HID_PROTOCOL_MODE_BOOT      0x00 // Boot Protocol Mode
#define HID_PROTOCOL_MODE_REPORT    0x01 // Report Protocol Mode

/* Attribute value lengths */
#define HID_PROTOCOL_MODE_LEN       1    // HID Protocol Mode
#define HID_INFORMATION_LEN         4    // HID Information
#define HID_REPORT_REF_LEN          2    // HID Report Reference Descriptor
#define HID_EXT_REPORT_REF_LEN      2    // External Report Reference Descriptor
#define HID_CTRL_PT_LEN             1    // HID Control point

//Android Remote Control HID Key Definition
#define KEYCODE_BOOKMARK                0x022A
#define KEYCODE_ALL_APPS                0x01A2
#define KEYCODE_PROFILE_SWITCH          0x019C
#define KEYCODE_ASSIST                  0x0221
#define KEYCODE_SETTINGS                0x009F
#define KEYCODE_DPAD_UP                 0x0042
#define KEYCODE_DAPD_DOWN               0x0043
#define KEYCODE_DPAD_LEFT               0x0044
#define KEYCODE_DPAD_RIGHT              0x0045
#define KEYCODE_DPAD_CENTER             0x0041
#define KEYCODE_BACK                    0x0224
#define KEYCODE_HOME                    0x0223
#define KEYCODE_GUIDE                   0x008d
#define KEYCODE_TV                      0x0089
#define KEYCODE_POWER                   0x0030
#define KEYCODE_TV_INPUT                0x01BB
#define KEYCODE_VOLUME_MUTE             0x00E2
#define KEYCODE_VOLUME_UP               0x00E9
#define KEYCODE_VOLUME_DOWN             0x00EA
#define KEYCODE_F17                     0x0077
#define KEYCODE_F18                     0x0078
#define KEYCODE_F19                     0x0079
#define KEYCODE_F20                     0x007A
#define KEYCODE_CHANNEL_UP              0x009C
#define KEYCODE_CHANNEL_DOWN            0x009D
#define KEYCODE_MEDIA_SKIP_BACKWARD     0x00B4
#define KEYCODE_MEDIA_RECORD            0x00CE
#define KEYCODE_MEDIA_PLAY_PAUSE        0x00CD
#define KEYCODE_MEDIA_SKIP_FORWARD      0x00B3
#define KEYCODE_PROG_RED                0x0069
#define KEYCODE_PROG_GREEN              0x006A
#define KEYCODE_PROG_YELLOW             0x006C
#define KEYCODE_PROG_BLUE               0x006B
#define KEYCODE_1                       0x001E
#define KEYCODE_2                       0x001F
#define KEYCODE_3                       0x0020
#define KEYCODE_4                       0x0021
#define KEYCODE_5                       0x0022
#define KEYCODE_6                       0x0023
#define KEYCODE_7                       0x0024
#define KEYCODE_8                       0x0025
#define KEYCODE_9                       0x0026
#define KEYCODE_0                       0x0027
#define KEYCODE_INFO                    0x01BD
#define KEYCODE_PERIOD                  0x01BC
#define KEYCODE_CAPTIONS                0x0061
#define KEYCODE_TV_TELETEXT             0x0185
#define KEY_NONE                        0x0

#define HID_MODULE_LONG_PRESS_DURATION              400    //msec
#define HID_MODULE_REPEAT_INTERVAL_DURATION         100    //msec
#define HID_MODULE_DEBOUNCE_DURATION                100    //msec
#define DEFAULT_HID_IDLE_TIMEOUT                    60000  //msec

// Battery measurement period in ms.
#define DEFAULT_BATT_PERIOD                   15000
// TRUE to run scan parameters refresh notify test.
#define DEFAULT_SCAN_PARAM_NOTIFY_TEST        TRUE
// Advertising intervals (units of 625us, 160=100ms).
#define HID_INITIAL_ADV_INT_MIN               48
#define HID_INITIAL_ADV_INT_MAX               80
#define HID_HIGH_ADV_INT_MIN                  32
#define HID_HIGH_ADV_INT_MAX                  48
#define HID_LOW_ADV_INT_MIN                   1600
#define HID_LOW_ADV_INT_MAX                   1600

// Advertising timeouts in sec.
#define HID_INITIAL_ADV_TIMEOUT               60
#define HID_HIGH_ADV_TIMEOUT                  5
#define HID_LOW_ADV_TIMEOUT                   0
// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN               8
#define HID_REPORT_READY_TIME                 1000

#define CONFIG_KEY_COL_1                      10
#define CONFIG_KEY_COL_2                      9
#define CONFIG_KEY_COL_3                      21
#define CONFIG_KEY_COL_4                      23
#define CONFIG_KEY_COL_5                      25

/*********************************************************************
 * TYPEDEFS
 */

// HID report mapping table
typedef struct
{
  uint16_t        handle;       // Handle of report characteristic
  gattAttribute_t *pCccdAttr;   // Pointer to CCCD attribute for report characteristic
  uint8_t         id;           // Report ID
  uint8_t         type;         // Report type
  uint8_t         mode;         // Protocol mode (report or boot)
} hidRptMap_t;

// HID dev configuration structure
typedef struct
{
  uint32_t    idleTimeout;      // Idle timeout in milliseconds
  uint8_t     hidFlags;         // HID feature flags

} hidDevCfg_t;

/*********************************************************************
 * Global Variables
 */

// These variables are defined in the service source file that uses HID Dev

// HID report map length

extern uint8_t hidReportMapLen;

// HID protocol mode
extern uint8_t hidProtocolMode;


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// HID Report callback
typedef uint8_t (*hidDevReportCB_t)(uint8_t id, uint8_t type, uint16_t uuid,
                                   uint8_t oper, uint16_t *pLen, uint8_t *pData);

// HID event callback
typedef void (*hidDevEvtCB_t)(uint8_t evt);

// HID passcode callback
typedef void (*hidDevPasscodeCB_t)(uint8_t  *deviceAddr,
                                   uint16_t connectionHandle,
                                   uint8_t uiInputs, uint8_t uiOutputs);

typedef struct
{
  hidDevReportCB_t    reportCB;
  hidDevEvtCB_t       evtCB;
  hidDevPasscodeCB_t  passcodeCB;
} hidDevCB_t;


/*********************************************************************
 * API FUNCTIONS
 */

extern bStatus_t HidDev_start(void);

/*********************************************************************
 * @fn      HidDev_createTask
 *
 * @brief   Task creation function for the HID service.
 *
 * @param   none
 *
 * @return  none
 */
extern void HidDev_createTask(void);

/*********************************************************************
 * @fn      HidDev_StartDevice
 *
 * @brief   Start the GAP Role and Register the Bond Manager.
 *          This function is intended to be called from the application
 *          task after setting up the GAP Role and Bond Manager.
 *
 * @param   None.
 *
 * @return  None.
 */
extern void HidDev_StartDevice(void);

/*********************************************************************
 * @fn      HidDev_Register
 *
 * @brief   Register a callback function with HID Dev.
 *
 * @param   pCfg - Parameter configuration.
 * @param   pCBs - Callback function.
 *
 * @return  None.
 */
extern void HidDev_Register(hidDevCfg_t *pCfg, hidDevCB_t *pCBs);

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
extern void HidDev_RegisterReports(uint8_t numReports, hidRptMap_t *pRpt);

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
extern void HidDev_Report(uint8_t id, uint8_t type, uint8_t len,
                          uint8_t *pData);

/*********************************************************************
 * @fn      HidDev_Close
 *
 * @brief   Close the connection or stop advertising.
 *
 * @return  None.
 */
extern void HidDev_Close(void);

/*********************************************************************
 * @fn      HidDev_SetParameter
 *
 * @brief   Set a HID Dev parameter.
 *
 * @param   param  - Profile parameter ID
 * @param   len    - length of data to right
 * @param   pValue - pointer to data to write.  This is dependent on
 *                   the parameter ID and WILL be cast to the appropriate
 *                   data type (example: data type of uint16_t will be cast to
 *                   uint16_t pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t HidDev_SetParameter(uint8_t param, uint8_t len, void *pValue);

/*********************************************************************
 * @fn      HidDev_GetParameter
 *
 * @brief   Get a HID Dev parameter.
 *
 * @param   param  - Profile parameter ID
 * @param   pValue - pointer to data to get.  This is dependent on
 *                   the parameter ID and WILL be cast to the appropriate
 *                   data type (example: data type of uint16_t will be cast to
 *                   uint16_t pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t HidDev_GetParameter(uint8_t param, void *pValue);

/*********************************************************************
 * @fn      HidDev_PasscodeRsp
 *
 * @brief   Respond to a passcode request.
 *
 * @param   status   - SUCCESS if passcode is available, otherwise
 *                     see @ref SMP_PAIRING_FAILED_DEFINES.
 * @param   passcode - integer value containing the passcode.
 *
 * @return  none
 */
extern void HidDev_PasscodeRsp(uint8_t status, uint32_t passcode);

/*********************************************************************
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
extern bStatus_t HidDev_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen,
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method);

/*********************************************************************
 * @fn      HidDev_WriteAttrCB
 *
 * @brief   HID Dev attribute read callback.
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
extern bStatus_t HidDev_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len,
                                    uint16_t offset, uint8_t method);

/*********************************************************************
 * @fn      HidDev_StartIdleTimer
 *
 * @brief   Start the idle timer.
 *
 * @return  None.
 */
extern void HidDev_StartIdleTimer(void);

/*********************************************************************
 * @fn      HidDev_StopIdleTimer
 *
 * @brief   Stop the idle timer.
 *
 * @param   None.
 *
 * @return  None.
 */
extern void HidDev_StopIdleTimer(void);

/*********************************************************************
 * @fn      HidDev_StartAdvertising
 *
 * @brief   Start advertising.
 *
 * @param   None.
 *
 * @return  None.
 */
extern void HidDev_StartAdvertising(void);

/*********************************************************************
 * @fn      HidDevState_ChangeHandler
 *
 * @brief   Start advertising.
 *
 * @param   None.
 *
 * @return  None.
 */
extern void HidDevState_ChangeHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HIDDEV_H */
