/******************************************************************************

 @file  audio_profile.h

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

#ifndef  AUDIOPROFILE_H
#define  AUDIOPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
// Profile Parameters
#define AUDIOPROFILE_START              0
#define AUDIOPROFILE_AUDIO              1

// Audio Profile Service UUID
#define AUDIO_SERV_UUID                 0xB000

// Key Pressed UUID
#define AUDIOPROFILE_START_UUID         0xB001
#define AUDIOPROFILE_AUDIO_UUID         0xB002

#define AUDIOPROFILE_CMD_LEN            1

#define BLEAUDIO_BUFSIZE                96
#define BLEAUDIO_HDRSIZE                4

#ifdef DLE_ENABLED // Data Length Extension Enable
#define BLEAUDIO_NOTSIZE                100
#define BLEAUDIO_NUM_NOT_PER_FRAME      1
#else
#define BLEAUDIO_NOTSIZE                20
#define BLEAUDIO_NUM_NOT_PER_FRAME      5
#endif
/*********************************************************************
 * TYPEDEFS
 */
typedef struct audioServiceConfig
{
  uint8 mode;
  uint16 l2capCh;
} audioServiceConfig_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*audioProfileChange_t)(uint8 paramID);

typedef struct
{
  // Called when characteristic value changes
  audioProfileChange_t pfnAudioProfileChange;
} audioProfileCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn          Audio_AddService
 *
 * @brief       Initializes the Audio Profile service by registering
 *              GATT attributes with the GATT server.
 *
 * @param       None.
 *
 * @return      Generic BLE status return
 */
extern bStatus_t Audio_AddService(void);

/*********************************************************************
 * @fn          Audio_SetParameter
 *
 * @brief       Set an Audio Profile parameter.
 *
 * @param       param - Profile parameter ID
 * @param       len - length of data to right
 * @param       value - pointer to data to write. This is dependent on
 *                      the parameter ID and WILL be cast to the appropriate
 *                      data type (example: data type of uint16 will be cast to
 *                      uint16 pointer).
 *
 * @return      Generic BLE status return
 */
extern bStatus_t Audio_SetParameter(uint8 param, uint8 len, void *value);

/*********************************************************************
 * @fn          Audio_GetParameter
 *
 * @brief       Get an Audio Profile parameter.
 *
 * @param       param - Profile parameter ID
 * @param       value - pointer to data to read. This is dependent on
 *                      the parameter ID and WILL be cast to the appropriate
 *                      data type (example: data type of uint16 will be cast to
 *                      uint16 pointer).
 *
 * @return      Generic BLE status return
 */
extern bStatus_t Audio_GetParameter(uint8 param, void *value);

/*********************************************************************
 * @fn          Audio_ProcessEvent
 *
 * @brief       Audio Profile event handler routine.
 *
 * @param       taskID - The HCI Test Application OSAL task identifer.
 * @param       events - HCI Test Application OSAL task events.
 *
 * @return      Unprocessed events.
 */
extern uint16 Audio_ProcessEvent(uint8 task_id, uint16 events);

/*********************************************************************
 * @fn          Audio_StartTxStreaming
 *
 * @brief       This is the Audio Pofile start function that
 *              will start audio streaming.
 *
 * @param       None.
 *
 * @return      None.
 */
extern void Audio_StartTxStreaming(void);

/*********************************************************************
 * @fn          Audio_StopTxStreaming
 *
 * @brief       This is the Audio Pofile stop function that
 *              will stop audio streaming.
 *
 * @param       None.
 *
 * @return      None.
 */
extern void Audio_StopTxStreaming(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif // AUDIOPROFILE_H
