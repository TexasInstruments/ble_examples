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

/*********************************************************************
 * INCLUDES
 */
#include <profiles/audio_dle/audio_profile_dle.h>
#include <profiles/audio_dle/audio_client_disc.h>



/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static AudioClientDisc_status_t startServiceDiscovery( gattMsgEvent_t *pMsg,
                                                        ICall_EntityID selfEntity,
                                                        uint8_t connIdx);
static AudioClientDisc_status_t processServiceDiscovery( gattMsgEvent_t *pMsg,
                                                        ICall_EntityID selfEntity,
                                                        uint8_t connIdx);
static AudioClientDisc_status_t processCharDiscovery( gattMsgEvent_t *pMsg,
                                                        ICall_EntityID selfEntity,
                                                        uint8_t connIdx);
static AudioClientDisc_status_t processCharDescDiscovery( gattMsgEvent_t *pMsg,
                                                            ICall_EntityID selfEntity,
                                                            uint8_t connIdx);

/*********************************************************************
 * LOCAL VARIABLES
 */
// Reset state variable
static AudioClientDisc_State_e discState[MAX_NUM_BLE_CONNS];

static AudioClientDisc_handles_t *serviceHandles[MAX_NUM_BLE_CONNS];

static bool discoverStartCCCD[MAX_NUM_BLE_CONNS];

static bool discoveryComplete[MAX_NUM_BLE_CONNS];

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AudioClientDisc_init
 *
 * @brief   Initialize the audio client discovery module
 *
 * @return  None.
 */
void AudioClientDisc_init(void)
{
    for(uint8_t i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        discState[i] = AudioClientDisc_state_idle;
        serviceHandles[i] = NULL;
        discoverStartCCCD[i] = false;
        discoveryComplete[i] = false;
    }
}

/*********************************************************************
 * @fn      AudioClientDisc_close
 *
 * @brief   Reset internal handle variables
 *
 * @param   connIdx - connection index
 *
 * @return  None.
 */
void AudioClientDisc_close(uint8_t connIdx)
{
    // Reset handles to known invalid
    serviceHandles[connIdx]->audioSvcStartHdl            = GATT_INVALID_HANDLE;
    serviceHandles[connIdx]->audioSvcEndHdl              = GATT_INVALID_HANDLE;
    serviceHandles[connIdx]->audioStartEndHandle         = GATT_INVALID_HANDLE;
    serviceHandles[connIdx]->audioStartCharValueHandle   = GATT_INVALID_HANDLE;
    serviceHandles[connIdx]->audioStartCCCHandle         = GATT_INVALID_HANDLE;
    serviceHandles[connIdx]->audioDataEndHandle          = GATT_INVALID_HANDLE;
    serviceHandles[connIdx]->audioDataCharValueHandle    = GATT_INVALID_HANDLE;
    serviceHandles[connIdx]->audioDataCCCHandle          = GATT_INVALID_HANDLE;

    discState[connIdx] = AudioClientDisc_state_idle;

    serviceHandles[connIdx] = NULL;
    discoveryComplete[connIdx] = false;

    return;
}

/*********************************************************************
 * @fn      AudioClientDisc_open
 *
 * @brief   Kick off state machine on connection established
 *
 * @param   audioSvcHandles - pointer to handle structure to
 *          to be filled out.
 * @param   connIdx - connection index
 *
 * @return  status - AUDIO_CLIENT_DISC_SUCCESS
 *                 - AUDIO_CLIENT_DISC_INVALID_PARAM
 */
AudioClientDisc_status_t AudioClientDisc_open(AudioClientDisc_handles_t
                                                *audioSvcHandles,
                                                uint8_t connIdx)
{
    // Store pointer to handle structure
    AudioClientDisc_status_t status = AUDIO_CLIENT_DISC_SUCCESS;
    if(audioSvcHandles != NULL)
    {
         discState[connIdx] = AudioClientDisc_state_MTU;
         serviceHandles[connIdx] = audioSvcHandles;
         discoverStartCCCD[connIdx] = false;
         discoveryComplete[connIdx] = false;
    }
    else
    {
        status = AUDIO_CLIENT_DISC_INVALID_PARAM;
    }

    return (status);
}

/*********************************************************************
 * @fn      AudioClientDisc_processGATTDisc
 *
 * @brief   Process GATT discovery event
 *
 * @param   pMsg - pointer to discovery event stack message
 * @param   selfEntity - ICall taskID of application task
 * @param   connIdx - connection index
 *
 * @return  status
 */
AudioClientDisc_status_t AudioClientDisc_processGATTDisc(gattMsgEvent_t *pMsg,
                                                        ICall_EntityID selfEntity,
                                                        uint8_t connIdx)
{
    AudioClientDisc_status_t status = AUDIO_CLIENT_DISC_SUCCESS;

    // If we've updated the MTU size
    switch(discState[connIdx])
    {
        case AudioClientDisc_state_MTU:
            // Process results from MTU exchange
            startServiceDiscovery(pMsg, selfEntity, connIdx);
            break;

        case AudioClientDisc_state_service:
            // Process results from GATT_DiscPrimaryServiceByUUID
            status = processServiceDiscovery(pMsg, selfEntity, connIdx);
            break;

        case AudioClientDisc_state_char:
            // Process results from GATT_DiscAllChars
            status = processCharDiscovery(pMsg,selfEntity, connIdx);
            break;

        case AudioClientDisc_state_CCCD:
            // Process results from GATT_DiscAllCharDescs
            status = processCharDescDiscovery(pMsg,selfEntity, connIdx);

            if(status == AUDIO_CLIENT_DISC_COMPLETE)
            {
                discoveryComplete[connIdx] = true;
            }
            break;
    }

    return (status);
}

/*********************************************************************
 * @fn      AudioClientDisc_isComplete
 *
 * @brief   Checks if audio service discovery is complete for a given
 *          connection index
 *
 * @param   connIdx - connection index
 *
 * @return  true  - if the audio service has been discovered, false otherwise
 */
bool AudioClientDisc_isComplete(uint8_t connIdx)
{
    return (discoveryComplete[connIdx]);
}

/*********************************************************************
 * PRIVATE FUNCTIONS
 */

/*********************************************************************
 * @fn      startServiceDiscovery
 *
 * @brief   Discover Audio Primary service, fill in handles
 *
 * @param   pMsg - GATT message from stack
 * @param   selfEntity - ICall taskID of application task
 * @param   connIdx - connection index
 *
 * @return  status - AUDIO_CLIENT_DISC_SUCCESS if service handles stored
 *                   AUDIO_CLIENT_DISC_ATTR_NOT_FOUND if no handle found
 */
static AudioClientDisc_status_t startServiceDiscovery( gattMsgEvent_t *pMsg,
                                                        ICall_EntityID selfEntity,
                                                        uint8_t connIdx)
{
    // MTU size response received, discover audio service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP ||
        pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
        // Advance state
        discState[connIdx] = AudioClientDisc_state_service;

        // Discovery of audio service
        GATT_DiscPrimaryServiceByUUID(pMsg->connHandle, audioProfileServUUID,
                                      ATT_UUID_SIZE,
                                      selfEntity);
    }

    return (AUDIO_CLIENT_DISC_SUCCESS);
}

/*********************************************************************
 * @fn      processServiceDiscovery
 *
 * @brief   Discover Audio Primary service, fill in handles
 *
 * @param   pMsg - GATT message from stack
 * @param   selfEntity - ICall taskID of application task
 * @param   connIdx - connection index
 *
 * @return  status - AUDIO_CLIENT_DISC_SUCCESS if service handles stored
 *                   AUDIO_CLIENT_DISC_ATTR_NOT_FOUND if no handle found
 */
static AudioClientDisc_status_t processServiceDiscovery( gattMsgEvent_t *pMsg,
                                                            ICall_EntityID selfEntity,
                                                            uint8_t connIdx)
{
    AudioClientDisc_status_t status = AUDIO_CLIENT_DISC_SUCCESS;

    // Service discovery results are of type findByTypeValRsp, cast it here
    attFindByTypeValueRsp_t *findByTypeValRsp = (attFindByTypeValueRsp_t *) \
                                                &(pMsg->msg.findByTypeValueRsp);

    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        findByTypeValRsp->numInfo > 0)
    {
        serviceHandles[connIdx]->audioSvcStartHdl = ATT_ATTR_HANDLE(findByTypeValRsp->pHandlesInfo, 0);
        serviceHandles[connIdx]->audioSvcEndHdl = ATT_GRP_END_HANDLE(findByTypeValRsp->pHandlesInfo, 0);
    }

    // If procedure is complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      // If we've discovered the service
      if (serviceHandles[connIdx]->audioSvcStartHdl != GATT_INVALID_HANDLE)
      {
        discState[connIdx] = AudioClientDisc_state_char;

        // Send characteristic discovery request
         GATT_DiscAllChars(pMsg->connHandle, serviceHandles[connIdx]->audioSvcStartHdl,
                                serviceHandles[connIdx]->audioSvcEndHdl, selfEntity);
      }
      else
      {
        discState[connIdx] = AudioClientDisc_state_idle;
        status = AUDIO_CLIENT_DISC_ATTR_NOT_FOUND;
      }
    }

    return (status);
}

/*********************************************************************
 * @fn      processCharDiscovery
 *
 * @brief   Discover all audio chars, fill in handles
 *
 * @param   pMsg - GATT message from stack
 * @param   selfEntity - ICall taskID of application task
 * @param   connIdx - connection index
 *
 * @return  status - AUDIO_CLIENT_DISC_SUCCESS if service handles stored
 *                   AUDIO_CLIENT_DISC_ATTR_NOT_FOUND if no handle found
 */
static AudioClientDisc_status_t processCharDiscovery( gattMsgEvent_t *pMsg,
                                                        ICall_EntityID selfEntity,
                                                        uint8_t connIdx)
{
    AudioClientDisc_status_t status = AUDIO_CLIENT_DISC_SUCCESS;

    // Characteristics are discovery using readByType response, cast here
    attReadByTypeRsp_t *readByTypeRsp = (attReadByTypeRsp_t *) \
                                            &(pMsg->msg.readByTypeRsp);

    // Characteristic found
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (readByTypeRsp->numPairs > 0))
    {
        // Handle value pairs can be differntly sized based on the UUIDs
        uint8_t hvpSize = readByTypeRsp->len;

        // Iterate through each handle value pair
        uint8_t idx = 0;
        for (uint8_t i = 0; i < readByTypeRsp->numPairs; i++)
        {
            // Set the idex into the array of HVPs
            idx = i*hvpSize;

            // Process 128-bit UUID handle value pair
            if(!memcmp(&(readByTypeRsp->pDataList[ATT_READ_BY_TYPE_HVP_UUID_IDX(idx)]),
                        audioProfileStartUUID,  ATT_UUID_SIZE))
            {
                serviceHandles[connIdx]->audioStartCharValueHandle = ATT_HVP_ATTR_VAL_HDL(idx, readByTypeRsp->pDataList);
            }
            else if(!memcmp(&(readByTypeRsp->pDataList[ATT_READ_BY_TYPE_HVP_UUID_IDX(idx)]),
                          audioProfileAudioUUID,  ATT_UUID_SIZE))
            {
                // We know the data/stream char is the last in the service
                serviceHandles[connIdx]->audioDataEndHandle = serviceHandles[connIdx]->audioSvcEndHdl;
                /*
                 * We know the audio start data chars are sequential, we can determine
                 * the end start char's end handle based on the data char's start handle
                 */
                serviceHandles[connIdx]->audioStartEndHandle = ATT_HVP_ATTR_HDL(idx, readByTypeRsp->pDataList);
                serviceHandles[connIdx]->audioDataCharValueHandle = ATT_HVP_ATTR_VAL_HDL(idx, readByTypeRsp->pDataList);
            }

        }
    }

    if (((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
        if((serviceHandles[connIdx]->audioStartCharValueHandle != GATT_INVALID_HANDLE) &&
            (serviceHandles[connIdx]->audioDataCharValueHandle  != GATT_INVALID_HANDLE))
        {
            discState[connIdx] = AudioClientDisc_state_CCCD;

            // Discover all descriptors for the Start char
            GATT_DiscAllCharDescs(pMsg->connHandle,
                                 (serviceHandles[connIdx]->audioStartCharValueHandle + 1),
                                  serviceHandles[connIdx]->audioStartEndHandle, selfEntity );
            discoverStartCCCD[connIdx] = true;
        }
        else
        {
            discState[connIdx] = AudioClientDisc_state_idle;
            status = AUDIO_CLIENT_DISC_ATTR_NOT_FOUND;
        }
    }

    return (status);
}

/*********************************************************************
 * @fn      processCharDescDiscovery
 *
 * @brief   Discover all audio chars, fill in handles
 *
 * @param   pMsg - GATT message from stack
 * @param   selfEntity - ICall taskID of application task
 * @param   connIdx - connection index
 *
 * @return  status - AUDIO_CLIENT_DISC_SUCCESS if service handles stored
 *                   AUDIO_CLIENT_DISC_ATTR_NOT_FOUND if no handle found
 */
static AudioClientDisc_status_t processCharDescDiscovery( gattMsgEvent_t *pMsg,
                                                            ICall_EntityID selfEntity,
                                                            uint8_t connIdx)
{
    AudioClientDisc_status_t status = AUDIO_CLIENT_DISC_SUCCESS;
    attFindInfoRsp_t *findInfoRsp = (attFindInfoRsp_t *) \
                                            &(pMsg->msg.findInfoRsp);
    // Characteristic found
    if (pMsg->method == ATT_FIND_INFO_RSP)
    {
        uint8_t findInfoHVPSize = 0;
        uint8_t idx = 0;

        if(findInfoRsp->format == ATT_HANDLE_BT_UUID_TYPE)
        {
            findInfoHVPSize = ATT_BT_UUID_SIZE + sizeof(uint16_t);
        }
        else
        {
            findInfoHVPSize = ATT_UUID_SIZE + sizeof(uint16_t);
        }

        for(uint8_t curPair = 0; curPair < findInfoRsp->numInfo; curPair++)
        {
            if(!memcmp(&(findInfoRsp->pInfo[ATT_FIND_INFO_HVP_UUID_IDX(curPair)]),
                        clientCharCfgUUID, ATT_BT_UUID_SIZE))
            {

                idx = curPair*findInfoHVPSize;
                if(discoverStartCCCD[connIdx] == true)
                {
                    serviceHandles[connIdx]->audioStartCCCHandle = ATT_HVP_ATTR_HDL(idx,
                                                            findInfoRsp->pInfo);

                    discoverStartCCCD[connIdx] = false;
                    GATT_DiscAllCharDescs(pMsg->connHandle,
                                            (serviceHandles[connIdx]->audioDataCharValueHandle + 1),
                                            serviceHandles[connIdx]->audioDataEndHandle,
                                            selfEntity );
                }
                else
                {
                    serviceHandles[connIdx]->audioDataCCCHandle = ATT_HVP_ATTR_HDL(idx,
                                                            findInfoRsp->pInfo);
                }
            }
        }
    }
    if ((((pMsg->method == ATT_FIND_INFO_RSP) &&
        (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP)) &&
        ((serviceHandles[connIdx]->audioStartCCCHandle != GATT_INVALID_HANDLE) &&
        (serviceHandles[connIdx]->audioDataCCCHandle != GATT_INVALID_HANDLE)))
    {
        discState[connIdx] = AudioClientDisc_state_idle;
        status = AUDIO_CLIENT_DISC_COMPLETE;
    }
    else
    {
        status = AUDIO_CLIENT_DISC_ATTR_NOT_FOUND;
    }

    return (status);
}
