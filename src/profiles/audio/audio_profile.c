/******************************************************************************

 @file  audio_profile.c

 @brief This file contains the audio profile sample service profile for use
        with the BLE sample application.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2015-2016, Texas Instruments Incorporated
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
 Release Name: ble_sdk_2_02_01_18
 Release Date: 2016-10-26 15:20:04
 *****************************************************************************/

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
#include "peripheral.h"

#include "audio_profile.h"
#include "ll.h"

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
// audio GATT Profile Service UUID: 0xB000
static CONST uint8 audioProfileServUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(AUDIO_SERV_UUID)
};

// Start/Stop Characteristic UUID: 0xB001
static CONST uint8 audioProfileStartUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(AUDIOPROFILE_START_UUID)
};

// Audio Stream Characteristic UUID: 0xB002
static CONST uint8 audioProfileAudioUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(AUDIOPROFILE_AUDIO_UUID)
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

/*********************************************************************
 * Profile Attributes - variables
 */
// Audio Profile Service attribute
static CONST gattAttrType_t audioProfileService = {ATT_UUID_SIZE,
                                                   audioProfileServUUID};

// Audio Profile Start/Stop Characteristic Properties
static uint8 audioProfileStartProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Start/Stop Characteristic Value
static uint8 audioProfileStart = 0;

// Start/Stop Characteristic Configuration Descriptor Value
static gattCharCfg_t *audioProfileStartConfig;

// Simple Profile Audio Stream Characteristic Properties
static uint8 audioProfileAudioProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Audio Stream Characteristic Value
static uint8_t audioProfileAudio[BLEAUDIO_NOTSIZE];

// Audio Stream Characteristic Configuration Descriptor Value
static gattCharCfg_t *audioProfileAudioConfig;


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t audioProfileAttrTbl[] =
{
  // Audio Profile Service
  {
    {ATT_BT_UUID_SIZE, primaryServiceUUID},   /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&audioProfileService             /* pValue */
  },

    // Start/Stop Characteristic Declaration
    {
      {ATT_BT_UUID_SIZE, characterUUID},
      GATT_PERMIT_READ,
      0,
      &audioProfileStartProps
    },

      // Start/Stop Characteristic Value
      {
        {ATT_UUID_SIZE, audioProfileStartUUID},
        GATT_PERMIT_READ,
        0,
        &audioProfileStart
      },

      // Start/Stop Characteristic configuration
      {
        {ATT_BT_UUID_SIZE, clientCharCfgUUID},
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&audioProfileStartConfig
      },

    // Audio Stream Characteristic Declaration
    {
      {ATT_BT_UUID_SIZE, characterUUID},
      GATT_PERMIT_READ,
      0,
      &audioProfileAudioProps
    },

      // Audio Stream Characteristic Value
      {
        {ATT_UUID_SIZE, audioProfileAudioUUID},
        GATT_PERMIT_READ,
        0,
        (uint8 *)audioProfileAudio
      },

      // Audio Stream Characteristic configuration
      {
        {ATT_BT_UUID_SIZE, clientCharCfgUUID},
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&audioProfileAudioConfig
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 audioProfile_ReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint16 *pLen, uint16 offset,
                                     uint16 maxLen, uint8 method);
static bStatus_t audioProfile_WriteAttrCB(uint16 connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8 *pValue,
                                          uint16 len,
                                          uint16 offset,
                                          uint8 method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Audio Profile Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
static CONST gattServiceCBs_t audioProfileCBs =
{
  audioProfile_ReadAttrCB,  // Read callback function pointer
  audioProfile_WriteAttrCB, // Write callback function pointer
  NULL                      // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Audio_AddService
 *
 * @brief   Initializes the Audio Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  SUCCESS, bleMemAllocError, or return value of
 *          GATTServApp_RegisterService
 */
bStatus_t Audio_AddService(void)
{
  uint8 status = SUCCESS;

  // Allocate Audio Cmd Client Characteristic Configuration table
  audioProfileStartConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t)*
                                                          linkDBNumConns );
  if (audioProfileStartConfig == NULL)
  {
    return bleMemAllocError;
  }

  // Initialize Audio Cmd Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, audioProfileStartConfig);

  // Allocate Audio Stream Client Characteristic Configuration table
  audioProfileAudioConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t)*
                                                          linkDBNumConns);
  if (audioProfileAudioConfig == NULL)
  {
    return bleMemAllocError;
  }

  // Initialize Audio Stream Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, audioProfileAudioConfig);

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService(audioProfileAttrTbl,
                                       GATT_NUM_ATTRS(audioProfileAttrTbl),
                                       GATT_MAX_ENCRYPT_KEY_SIZE,
                                       &audioProfileCBs);

  return status;
}

/*********************************************************************
 * @fn      Audio_SetParameter
 *
 * @brief   Set an Audio Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  SUCCESS, bleInvalidRange, INVALIDPARAMETER, or return
 *          value of GATTServApp_ProcessCharCfg
 */
bStatus_t Audio_SetParameter(uint8 param, uint8 len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case AUDIOPROFILE_START:
      if (len == sizeof (audioProfileStart))
      {
        audioProfileStart = *((uint8*)value);

        // See if Notifications have been enabled and send
        ret = GATTServApp_ProcessCharCfg(audioProfileStartConfig,
                                         &audioProfileStart,
                                         FALSE,
                                         audioProfileAttrTbl,
                                         GATT_NUM_ATTRS(audioProfileAttrTbl),
                                         INVALID_TASK_ID,
                                         audioProfile_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case AUDIOPROFILE_AUDIO:
      {
        VOID memcpy(audioProfileAudio, value, BLEAUDIO_NOTSIZE);

        // See if Notifications have been enabled and send
        ret = GATTServApp_ProcessCharCfg(audioProfileAudioConfig,
                                         (uint8_t *)audioProfileAudio,
                                         FALSE,
                                         audioProfileAttrTbl,
                                         GATT_NUM_ATTRS(audioProfileAttrTbl),
                                         INVALID_TASK_ID,
                                         audioProfile_ReadAttrCB);
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ret;
}

/*********************************************************************
 * @fn      Audio_GetParameter
 *
 * @brief   Get a Audio Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t Audio_GetParameter(uint8 param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case AUDIOPROFILE_START:
      *((uint8*)value) = audioProfileStart;
      break;

    case AUDIOPROFILE_AUDIO:
      VOID memcpy(value, audioProfileAudio, BLEAUDIO_NOTSIZE);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ret;
}

/*********************************************************************
 * @fn          audioProfile_ReadAttrCB
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
 * @return      SUCCESS, ATT_ERR_INSUFFICIENT_AUTHOR,
 *              ATT_ERR_ATTR_NOT_LONG, or ATT_ERR_INVALID_HANDLE
 */
static uint8 audioProfile_ReadAttrCB(uint16 connHandle,
                                     gattAttribute_t *pAttr,
                                     uint8 *pValue,
                                     uint16 *pLen,
                                     uint16 offset,
                                     uint16 maxLen,
                                     uint8 method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
  {
    return ATT_ERR_ATTR_NOT_LONG;
  }

  if (pAttr->type.len == ATT_UUID_SIZE)
  {
    // 128-bit UUID
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[12], pAttr->type.uuid[13]);
    switch (uuid)
    {
      case AUDIOPROFILE_START_UUID:
        // Let remote side know the current state of play
        *pLen = sizeof(audioProfileStart);
        pValue[0] = *pAttr->pValue;
        break;

      case AUDIOPROFILE_AUDIO_UUID:
        *pLen = BLEAUDIO_NOTSIZE;
        VOID memcpy(pValue, pAttr->pValue, BLEAUDIO_NOTSIZE);
        break;

      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 16-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return status;
}

/*********************************************************************
 * @fn      audioProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of read message
 *
 * @return  SUCCESS, ATT_ERR_INSUFFICIENT_AUTHOR,
 *          ATT_ERR_ATTR_NOT_LONG, or ATT_ERR_INVALID_HANDLE
 */
static bStatus_t audioProfile_WriteAttrCB(uint16 connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8 *pValue,
                                          uint16 len,
                                          uint16 offset,
                                          uint8 method)
{
  bStatus_t status = SUCCESS;

  if (offset != 0)
  {
    return ATT_ERR_ATTR_NOT_LONG;
  }

  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch (uuid)
    {
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                offset, GATT_CLIENT_CFG_NOTIFY);
        break;

      default:
        // Should never get here!
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else if (pAttr->type.len == ATT_UUID_SIZE)
  {
    // 128-bit UUID
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[12], pAttr->type.uuid[13]);
    switch (uuid)
    {
      case AUDIOPROFILE_START_UUID:
      case AUDIOPROFILE_AUDIO_UUID:
        // Write not permitted
        status = ATT_ERR_WRITE_NOT_PERMITTED;
        break;

      default:
        // Should never get here!
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    status = ATT_ERR_INVALID_HANDLE;
  }

  return status;
}

/*********************************************************************
*********************************************************************/
