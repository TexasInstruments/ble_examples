/*
 * Filename: audio_client_disc.h
 *
 * Description: Implements GATT client for audio_profile_dle found in
 * this folder. This module is responsible for parsing GATT messages
 * and storing the handles based on them.
 *
 * Calls to this module run in the application task context. A callback
 * must be plugged for the module to handle higher priority CBs such as
 * Audio data and I2S callbacks
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef AUDIO_CLIENT_DISC_H
#define AUDIO_CLIENT_DISC_H

#ifdef __cplusplus
extern "C"
{
#endif


/*********************************************************************
 * INCLUDES
 */
#include "icall_ble_api.h"

/*********************************************************************
 * CONSTANTS
 */

#define ATT_READ_BY_TYPE_HVP_UUID_IDX(idx)      (idx + 5)
#define ATT_FIND_INFO_HVP_UUID_IDX(idx)         (idx + 2)

#define ATT_HVP_ATTR_HDL(idx, val)              BUILD_UINT16( val[idx],         \
                                                                val[idx + 1] )
#define ATT_HVP_ATTR_VAL_HDL(idx, val)          BUILD_UINT16( val[idx + 3],     \
                                                                val[idx + 4] )

#define ATT_HVP_SZ_BT_UUID                      7
#define ATT_HVP_SZ_UUID                         21


// Status Codes
#define AUDIO_CLIENT_DISC_INVALID_PARAM          -1
#define AUDIO_CLIENT_DISC_INVALID_STATE          -2
#define AUDIO_CLIENT_DISC_ATTR_NOT_FOUND         -3
#define AUDIO_CLIENT_DISC_SUCCESS                0
#define AUDIO_CLIENT_DISC_COMPLETE               1


/*********************************************************************
 * TYPEDEFS
 */

typedef int8_t AudioClientDisc_status_t;

typedef enum
{
    AudioClientDisc_state_idle,                 // Idle
    AudioClientDisc_state_MTU,                  // Exchange ATT MTU size
    AudioClientDisc_state_service,              // Service discovery
    AudioClientDisc_state_char,                 // Characteristic discovery
    AudioClientDisc_state_CCCD                  // CCCD discovery
}AudioClientDisc_State_e;


typedef struct
{
    /* Audio Service handle */
    uint16_t audioSvcStartHdl;
    uint16_t audioSvcEndHdl;
    /* Audio START characteristic */
    uint16_t audioStartEndHandle;
    uint16_t audioStartCharValueHandle;
    uint16_t audioStartCCCHandle;
    /* Audio "Data" characteristic */
    uint16_t audioDataEndHandle;
    uint16_t audioDataCharValueHandle;
    uint16_t audioDataCCCHandle;
}AudioClientDisc_handles_t;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AudioClientDisc_open
 *
 * @brief   Kick off state machine on connection established
 *
 * @param   audioSvcHandles - pointer to handle structure to
 *          to be filled out.
 *
 * @return  status - AUDIO_CLIENT_DISC_SUCCESS
 *                 - AUDIO_CLIENT_DISC_INVALID_PARAM
 */
AudioClientDisc_status_t AudioClientDisc_open(AudioClientDisc_handles_t
                                                *audioSvcHandles);

/*********************************************************************
 * @fn      AudioClientDisc_close
 *
 * @brief   Reset internal handle variables
 *
 * @param   None.
 *
 * @return  None.
 */
void AudioClientDisc_close(void);

/*********************************************************************
* @fn      AudioClientDisc_isComplete
*
* @brief
*
* @param   none
*
* @return  whether or not svc disc is complete
*/
bool AudioClientDisc_isComplete(void);

/*********************************************************************
* @fn      AudioClientDisc_processGATTDisc
*
* @brief   Process GATT discovery event
*
* @param   pMsg - pointer to discovery event stack message
*
* @return  status
*/
AudioClientDisc_status_t AudioClientDisc_processGATTDisc(gattMsgEvent_t *pMsg,
                                                        ICall_EntityID selfEntity);


#ifdef __cplusplus
}
#endif

#endif /* AUDIO_CLIENT_DISC_H */
