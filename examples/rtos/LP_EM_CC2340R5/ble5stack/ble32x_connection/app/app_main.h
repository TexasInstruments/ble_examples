/******************************************************************************

@file  app_main.h

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2023, Texas Instruments Incorporated
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

#ifndef APP_MAIN_H_
#define APP_MAIN_H_

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Typedefs
//*****************************************************************************
typedef enum
{
    APP_MENU_GENERAL_STATUS_LINE,
    APP_MENU_DEVICE_ADDRESS,
    APP_MENU_DEVICE_RP_ADDRESS,
    APP_MENU_ADV_EVENT,
    APP_MENU_SCAN_EVENT,
    APP_MENU_NUM_CONNS,
    APP_MENU_CONN_EVENT,
    APP_MENU_PAIRING_EVENT,
    APP_MENU_PROFILE_STATUS_LINE,
    APP_MENU_PROFILE_STATUS_LINE1,
    APP_MENU_PROFILE_STATUS_LINE2,
    APP_MENU_PROFILE_STATUS_LINE3,
    APP_MENU_PROFILE_STATUS_LINE4
}AppMenu_rows;

PACKED_ALIGNED_TYPEDEF_STRUCT
{
  /// Type of TargetA address in the directed advertising PDU
  uint8_t  addressType;
  /// TargetA address
  BLEAppUtil_BDaddr  address;
}App_scanResults;

// Connected device information
PACKED_ALIGNED_TYPEDEF_STRUCT
{
  uint16_t  connHandle;             // Connection Handle
  BLEAppUtil_BDaddr peerAddress;    // The address of the peer device
} App_connInfo;

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Peripheral_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the peripheral
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Peripheral_start(void);

/*********************************************************************
 * @fn      Broadcaster_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the broadcaster
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Broadcaster_start(void);

/*********************************************************************
 * @fn      Central_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the central
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Central_start(void);

/*********************************************************************
 * @fn      Observer_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the observer
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Observer_start(void);

/*********************************************************************
 * @fn      Connection_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the connection
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Connection_start(void);

/*********************************************************************
 * @fn      Pairing_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the pairing
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Pairing_start(void);

/*********************************************************************
 * @fn      Data_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the data
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Data_start(void);

/*********************************************************************
 * @fn      DevInfo_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Device Info service.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t DevInfo_start(void);

/*********************************************************************
 * @fn      SimpleGatt_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Simple GATT profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t SimpleGatt_start(void);

/*********************************************************************
 * @fn      OAD_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the OAD profile and service.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t OAD_start(void);

/*********************************************************************
 * @fn      DataStream_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Data Stream profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t DataStream_start(void);

#ifdef DeviceFamily_CC27XX
/*********************************************************************
 * @fn      L2CAPCOC_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the L2CAP COC.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t L2CAPCOC_start(void);
#endif
/*********************************************************************
 * @fn      CGM_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the CGM profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t CGM_start(void);

/*********************************************************************
 * @fn      GL_start

 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Glucose profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t GL_start(void);

/*********************************************************************
 * @fn      HT_start

 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Health Thermometer profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t HT_start(void);

/*********************************************************************
 * @fn      Menu_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize the
 *          menu
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Menu_start(void);

/*********************************************************************
 * @fn      Scan_getScanResList
 *
 * @brief   Get the scan result list
 *
 * @param   scanRes - a scan list pointer
 *
 * @return  The number of results in the list
 */
uint8 Scan_getScanResList(App_scanResults **scanRes);

/*********************************************************************
 * @fn      Connection_getConnList
 *
 * @brief   Get the connection list
 *
 * @return  connection list
 */
App_connInfo *Connection_getConnList(void);

/*********************************************************************
 * @fn      Connection_getConnhandle
 *
 * @brief   Find connHandle in the connected device list by index
 *
 * @return  the connHandle found. If there is no match,
 *          MAX_NUM_BLE_CONNS will be returned.
 */
uint16_t Connection_getConnhandle(uint8_t index);

/*********************************************************************
 * @fn      Connection_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, LL_INACTIVE_CONNECTIONS will be returned.
 */
uint16_t Connection_getConnIndex(uint16_t connHandle);

#endif /* APP_MAIN_H_ */
