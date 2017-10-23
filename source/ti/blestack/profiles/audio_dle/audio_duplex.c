/*
 * Filename: audio_duplex.c
 *
 * Description: Implements bidirectional VoGP audio/voice communication
 * This module connects data from the user application task(buttons),
 * data from the I2S driver (local audio frames), and data from the
 * BLE-Stack Audio Profile (incoming audio frames).
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
#include <string.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <driverlib/vims.h>
#include <driverlib/flash.h>
#include <ti/display/Display.h>
#include <profiles/audio_dle/audio_profile_dle.h>
#include <ti/audiocodec/audiocodec.h>
#include <third_party/sbc/include/msbc_library.h>
#include <ti/drivers/i2s/I2SCC26XX.h>
#include <ti/drivers/pdm/Codec1.h>

#include "audio_duplex.h"
#include "icall_ble_api.h"
#include "hal_flash.h"

/*********************************************************************
 * CONSTANTS
 */

#define BLEAUDIO_NUM_NOT_PER_FRAME_ADPCM  1
#define BLEAUDIO_NUM_NOT_PER_FRAME_MSBC   1


#define AUDIO_DUPLEX_DISP_STAT1           6
#define AUDIO_DUPLEX_DISP_STAT2           7

/*
 * Required Memory for Bi-directional audio streaming:
 * The I2S driver requires two blocks of memory to be
 * passed in at its open call.
 *  1. Queue memory for TI-RTOS Queue elements
 *  2. Sample memory for sample buffers
 *
 * The amount of memory required while streaming is:
 * (uncompressed buffer)*2 (bidirectional)*sizeof(unit16_t)*I2SCC26XX_QUEUE_SIZE
 *
 * An MSBC frame is larger than ADPCM so in this case we use the worst case
 */
#define I2S_BUF     sizeof(int16_t) * (AUDIO_DUPLEX_MSBC_SAMPLES_PER_FRAME *   \
                                        I2SCC26XX_QUEUE_SIZE * 2)

#define I2S_MEM_BASE (GPRAM_BASE + FlashSectorSizeGet())

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void AudioDuplex_i2sCallbackFxn(I2SCC26XX_Handle handle,
                                        I2SCC26XX_StreamNotification *notification);
static void AudioDuplex_enableCache();
static void AudioDuplex_disableCache();
static bStatus_t AudioDuplex_transmitAudioFrame(uint8_t *buf);
static I2SCC26XX_Handle AudioDuplex_openI2S(void);
static void AudioDuplex_startI2Sstream(void);
static void AudioDuplex_stopI2Sstream(void);
static void AudioDuplex_sendStartCmd(void);
static void AudioDuplex_sendStopCmd(void);
static void AudioDuplex_shiftEncodedBuffer(uint8_t *encodedBuf, uint8_t len,
                                            uint8_t streamType);

/*********************************************************************
 * LOCAL VARIABLES
 */

// Audio Buffer variables
static int16_t *audio_decoded = NULL;
static uint8_t *i2sContMgtBuffer = NULL;
static sbc_t sbc = {0};
static size_t written = 0;

// I2S Variables
static I2SCC26XX_Handle i2sHandle = NULL;
static I2SCC26XX_StreamNotification i2sStream;
static bool i2sStreamInProgress = false;
static uint8_t volume = AUDIO_DUPLEX_MAX_VOLUME;
static uint8_t seqNum = 0;

// BLE Connection handle of stream
uint16_t audioConnectionHandle = NULL;
// Display Interface
static Display_Handle hDisp = NULL;
// A function pointer to call in the application to set an event
static pfnAudioDuplexCB_t appAudioCB = NULL;

static AudioDuplex_streamVars streamVariables =
{
    AudioDuplex_stream_idle,
    AudioDuplex_stream_idle,
    AUDIO_DUPLEX_STREAM_TYPE_NONE,
    AUDIO_DUPLEX_STREAM_TYPE_NONE,
    0,
    0,
    I2SCC26XX_QUEUE_SIZE,
    BLEAUDIO_MAX_NOTSIZE,
    0, // si
    0, // pv
    0, // si_rx
    0, // pv_rx
    AUDIO_DUPLEX_MAX_VOLUME, // maxVolume
};

static I2SCC26XX_Params i2sParams =
{
    .requestMode            = I2SCC26XX_CALLBACK_MODE,
    .ui32requestTimeout     = BIOS_WAIT_FOREVER,
    .callbackFxn            = AudioDuplex_i2sCallbackFxn,
    .blockSize              = AUDIO_DUPLEX_MSBC_SAMPLES_PER_FRAME,
    .pvContBuffer           = NULL,
    .ui32conBufTotalSize    = 0,
    .pvContMgtBuffer        = NULL,
    .ui32conMgtBufTotalSize = 0,
    .currentStream          = &i2sStream
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AudioDuplex_open
 *
 * @brief   Called during initialization, opens codec and I2S driver
 *          Initializes hardware and adds Audio Profile
 *
 * @param   None.
 *
 * @return  None.
 */
int8_t AudioDuplex_open(Display_Handle displayHandle, PIN_Handle pinHandle,
                            pfnAudioDuplexCB_t inputCB)
{
    uint8_t status = AUDIO_DUPLEX_SUCCESS;
    hDisp = displayHandle;

    // Store app callback if not null
    if(inputCB == NULL)
    {
        Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT1, 0,
                        "Fail: AudioCB is NULL");
        return (AUDIO_INVALID_PARAMS);
    }

    appAudioCB = inputCB;

    /* Initialize I2S driver */
    i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
    I2SCC26XX_init(i2sHandle);

    // Initialize TLV320AIC3254 Codec on Audio BP
    status = AudioCodecOpen();
    if( AUDIO_CODEC_STATUS_SUCCESS != status)
    {
        Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT1, 0,
                        "Fail: Can't open codec");
    }
    // Configure Codec
    status =  AudioCodecConfig(AUDIO_CODEC_TI_3254, AUDIO_CODEC_16_BIT,
                                AUDIO_DUPLEX_SAMPLE_RATE, AUDIO_DUPLEX_NUM_CHAN,
                                AUDIO_DUPLEX_OUTPUT_OPTION,
                                AUDIO_DUPLEX_INPUT_OPTION);
    if( AUDIO_CODEC_STATUS_SUCCESS != status)
    {
        Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT1, 0,
                        "Fail: Can't configure BP");
    }

    // Add the Audio service
    status = Audio_AddService();

    if(SUCCESS != status)
    {
        Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT1, 0,
                        "Fail: Can't add Audio Service");
    }

    return (status);
}

/*********************************************************************
 * @fn      AudioDuplex_close
 *
 * @brief   Closes hardware, stops streaming
 *
 * @param   None.
 *
 * @return  None.
 */
void AudioDuplex_close(void)
{
    // Close the interface to the hardware codec
    AudioCodecClose();

    // Reset stream vars
    streamVariables.streamState =  AudioDuplex_stream_idle;
    streamVariables.requestedStreamState =  AudioDuplex_stream_idle;
    streamVariables.streamType =  AUDIO_DUPLEX_STREAM_TYPE_NONE;
    streamVariables.requestedStreamType =  AUDIO_DUPLEX_STREAM_TYPE_NONE;
    streamVariables.samplesPerFrame =  0;
    streamVariables.notificationsPerFrame =  0;
    streamVariables.numOfFramesToBuffer =  I2SCC26XX_QUEUE_SIZE;
    streamVariables.rfFrameSize =  BLEAUDIO_MAX_NOTSIZE;
    streamVariables.si =  0; // si;
    streamVariables.pv =  0; // pv;
    streamVariables.si_rx =  0; // si_rx;
    streamVariables.pv_rx =  0; // pv_rx;
    streamVariables.maxVolume =  AUDIO_DUPLEX_MAX_VOLUME; // maxVolume;
}

/*********************************************************************
 * @fn      AudioDuplex_setConnectionHandle
 *
 * @brief   Set the connection handle of audio streaming
 *
 * @param   connectionHandle - Connection handle.
 *
 * @return  None
 */
void AudioDuplex_setConnectionHandle(uint16_t connectionHandle)
{
    audioConnectionHandle = connectionHandle;
}

/*********************************************************************
 * @fn      AudioDuplex_getConnectionHandle
 *
 * @brief   Get the connection handle of audio streaming
 *
 * @return  connectionHandle- the handle used to stream
 */
uint16_t AudioDuplex_getConnectionHandle(void)
{
    return (audioConnectionHandle);
}

/*********************************************************************
 * @fn      AudioDuplex_eventHandler
 *
 * @brief   Processes Events called from I2S or BLE-Stack callbacks
 *
 * @param   events - Events to process.
 *
 * @return  None
 */
void AudioDuplex_eventHandler(uint8_t events)
{
    // Read in samples from I2S driver and encode
    if (events & AUDIO_DUPLEX_I2S_FRAME_EVENT)
    {
        if (i2sStreamInProgress)
        {
            I2SCC26XX_BufferRequest bufferRequest;
            I2SCC26XX_BufferRelease bufferRelease;
            bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN;
            // Indicate that we request buffer from input stream
            bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
            while (gotBuffer)
            {
                uint8_t *inBuffer = (uint8_t *)bufferRequest.bufferIn;
                uint8_t *bufHead = inBuffer;
                uint8_t si_local = streamVariables.si;
                uint16_t pv_local = streamVariables.pv;

                if (streamVariables.streamType == AUDIO_DUPLEX_STREAM_TYPE_MSBC)
                {
                    sbc_encode(&sbc, (int16_t *)bufferRequest.bufferIn,
                                streamVariables.samplesPerFrame * sizeof(int16_t),
                                inBuffer,
                                AUDIO_DUPLEX_MSBC_ENCODED_SIZE, &written);

                    // Shift the buffer to acount for the seqNum hdr
                    AudioDuplex_shiftEncodedBuffer(inBuffer,
                                                   BLEAUDIO_MAX_NOTSIZE,
                                                   AUDIO_DUPLEX_STREAM_TYPE_MSBC);
                    inBuffer[1] = seqNum++;
                }
                else
                {
                    Codec1_encodeBuff(inBuffer,
                                    (int16_t *)bufferRequest.bufferIn,
                                    streamVariables.samplesPerFrame,
                                    &streamVariables.si, &streamVariables.pv);

                    // Reset the pointer
                    inBuffer = bufHead;
                    bufferRequest.bufferIn = bufHead;

                    // Shift the buffer to acount for the seqNum hdr
                    AudioDuplex_shiftEncodedBuffer(inBuffer,
                                                   BLEAUDIO_MAX_NOTSIZE,
                                                   AUDIO_DUPLEX_STREAM_TYPE_ADPCM);

                    inBuffer[0] = seqNum++;
                    // Send previous PV and SI
                    inBuffer[1] = si_local;
                    inBuffer[2] = LO_UINT16(pv_local);
                    inBuffer[3] = HI_UINT16(pv_local);
                }

                AudioDuplex_transmitAudioFrame(inBuffer);

                // Release the buffer back to the I2S driver
                bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
                bufferRelease.bufferHandleOut = NULL;
                I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);

                // Request the next buffer from I2S driver
                bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN;
                // Indicate that we request buffer from input stream
                gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
            }
        }
    }

    // Set the state vars to cleanup I2S streaming
    if (events & AUDIO_DUPLEX_STOP_I2S_EVENT)
    {
        AudioDuplex_stopI2Sstream();
    }

    // Send stop command, cleanup stream
    if (events & AUDIO_DUPLEX_SEND_STOP_CMD_EVENT)
    {
        AudioDuplex_sendStopCmd();
    }

    // Startup I2S sampling
    if (events &   AUDIO_DUPLEX_START_I2S_EVENT)
    {
        AudioDuplex_startI2Sstream();
    }

    // Startup BLE Streaming
    if (events & AUDIO_DUPLEX_SEND_START_CMD_EVENT)
    {
        AudioDuplex_sendStartCmd();
    }

    // Handle error events from I2S driver
    if (events & AUDIO_DUPLEX_I2S_ERROR_EVENT)
    {
        Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT1, 0, "I2S Error Event");

        // Move to stop state
        uint_least16_t hwiKey = Hwi_disable();
        streamVariables.streamType = AudioDuplex_send_stop_cmd;
        if(appAudioCB != NULL)
        {
            // Notify the Application of the event
            (*appAudioCB)(AUDIO_DUPLEX_SEND_STOP_CMD_EVENT);
        }
        Hwi_restore(hwiKey);
    }
}

/*********************************************************************
 * @fn      AudioDuplex_processData
 *
 * @brief   Process incoming audio data
 *
 * @return  None
 */
void AudioDuplex_processData(AudioDuplex_dataType data_type,
                              AudioDuplex_audioData *pMsg)
{
    static uint16_t prevSeqNum = 0xFFFF;
    static int numberOfPackets = 0, lostPackets = 0;
    static int msbcnumberOfPackets = 0, msbclostPackets = 0;

    // Check to see if notification is from audio data or control char
    if (data_type == AudioDuplex_data)
    {
        // If we have started the stream collect frames into a buffer
        if (streamVariables.streamType == AUDIO_DUPLEX_CMD_START)
        {
            // Check to see if a frame has been missed and report it
            if (((prevSeqNum + 1) & 0x00FF) != pMsg->pValue[0])
            {
                uint16_t curSeqNum = pMsg->pValue[0];
                uint8_t missedFrames = 0;
                if (pMsg->pValue[0] > prevSeqNum)
                {
                    missedFrames = curSeqNum - prevSeqNum;
                }
                else
                {
                    missedFrames = (curSeqNum + 256) - prevSeqNum;
                }

                numberOfPackets += missedFrames;
                lostPackets += missedFrames;
                Display_print2(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                                "Missing frame, PER %d/%d",
                                lostPackets, numberOfPackets);
            }
            else
            {
                numberOfPackets++;
            }
            prevSeqNum = pMsg->pValue[0];
        }
        else if (streamVariables.streamType == AUDIO_DUPLEX_CMD_START_MSBC)
        {
            // Check to see if a frame has been missed and report it
            if (((prevSeqNum + 1) & 0x00FF) != pMsg->pValue[1])
            {
                uint16_t curSeqNum = pMsg->pValue[1];
                uint8_t missedFrames = 0;

                if (pMsg->pValue[1] > prevSeqNum)
                {
                    missedFrames = curSeqNum - prevSeqNum;
                }
                else
                {
                    missedFrames = (curSeqNum + 256) - prevSeqNum;
                }

                msbcnumberOfPackets += missedFrames;
                msbclostPackets += missedFrames;
                Display_print4(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                                "Missing frame, PER %d/%d (%d vs %d)",
                                msbclostPackets, msbcnumberOfPackets,
                                pMsg->pValue[1],
                                prevSeqNum);
            }
            else
            {
                msbcnumberOfPackets++;
            }
            prevSeqNum = pMsg->pValue[1];
        }

        // If we have received a complete frame OTA decode it and send to I2S
        // for playback
        if (i2sStreamInProgress)
        {
            I2SCC26XX_BufferRequest bufferRequest;
            I2SCC26XX_BufferRelease bufferRelease;
            bufferRequest.buffersRequested = I2SCC26XX_BUFFER_OUT;
            // Indicate that we request buffer from output stream
            bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
            if (gotBuffer)
            {
                // Call codec specific decode fxns
                if (streamVariables.streamType == AUDIO_DUPLEX_CMD_START_MSBC)
                {
                    sbc_decode(&sbc, pMsg->pValue, AUDIO_DUPLEX_MSBC_ENCODED_SIZE,
                              (int16_t *)bufferRequest.bufferOut,
                              streamVariables.samplesPerFrame * sizeof(int16_t),
                              &written);
                }
                else
                {
                    streamVariables.pv_rx = BUILD_UINT16(pMsg->pValue[2],
                                                          pMsg->pValue[3]);
                    streamVariables.si_rx = pMsg->pValue[1];
                    Codec1_decodeBuff((int16_t *)bufferRequest.bufferOut,
                                      (uint8_t *)&pMsg->pValue[4],
                                      streamVariables.samplesPerFrame * sizeof(int16_t),
                                      &streamVariables.si_rx, &streamVariables.pv_rx);
                }

                // Send the buffer to the BoosterPack
                bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
                bufferRelease.bufferHandleIn = NULL;
                I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
            }
        }
    }
    else if (data_type == AudioDuplex_start_stop)
    {
        // Audio/Voice commands are 1B in length
        if(AUDIOPROFILE_CMD_LEN == pMsg->len)
        {
            // If we received a stop command reset the SI, PV
            if(AUDIO_DUPLEX_CMD_START == *(pMsg->pValue))
            {
                if (streamVariables.streamType != AUDIO_DUPLEX_CMD_STOP)
                {
                    Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                                    "Already started stream");
                }
                else
                {
                    // We received a start command for ADPCM, start the stream
                    AudioDuplex_startStreaming(AUDIO_DUPLEX_STREAM_TYPE_ADPCM);

                    Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                                    "ADPCM Stream");
                }
            }
            else if(AUDIO_DUPLEX_CMD_START_MSBC == *(pMsg->pValue))
            {
                if (streamVariables.streamType != AUDIO_DUPLEX_CMD_STOP)
                {
                    Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                                    "Already started stream");
                }
                else
                {
                    // We received a start command for MSBC, start the stream
                    AudioDuplex_startStreaming(AUDIO_DUPLEX_STREAM_TYPE_MSBC);
                    // Initialize encoder
                    sbc_init_msbc(&sbc, 0);
                    Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                                    "mSBC Stream");
                }
            }
            else if(AUDIO_DUPLEX_CMD_STOP == *(pMsg->pValue))
            {

                if(streamVariables.streamType != AUDIO_DUPLEX_STREAM_TYPE_NONE)
                {
                    Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                                    "Received Stop, sending stop");
                    prevSeqNum = 0xFFFF;
                    AudioDuplex_stopStreaming();
                }
                else
                {
                    Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                                    "Received Stop, re-starting");
                    prevSeqNum = 0xFFFF;
                    AudioDuplex_startStreaming(streamVariables.requestedStreamType);
                }
            }
        }
    }
}

/*********************************************************************
 * @fn      AudioDuplex_startStreaming
 *
 * @brief   Starts streaming audio to connected device
 *
 * @param   requestedStreamType - The type of stream to start:
 *                              - AUDIO_DUPLEX_STREAM_TYPE_ADPCM
 *                              - AUDIO_DUPLEX_STREAM_TYPE_MSBC
 *
 * @return  None.
 */
void AudioDuplex_startStreaming(uint8_t requestedStreamType)
{
    // Do stream state logic
    if (streamVariables.streamState == AudioDuplex_stream_idle)
    {
        // Start stream, from IDLE
        streamVariables.streamType = requestedStreamType;

        if(requestedStreamType == AUDIO_DUPLEX_STREAM_TYPE_MSBC)
        {
            streamVariables.samplesPerFrame = AUDIO_DUPLEX_MSBC_SAMPLES_PER_FRAME;
            streamVariables.notificationsPerFrame = BLEAUDIO_NUM_NOT_PER_FRAME_MSBC;
            streamVariables.requestedStreamType = AUDIO_DUPLEX_STREAM_TYPE_MSBC;
        }
        else
        {
            streamVariables.samplesPerFrame = AUDIO_DUPLEX_ADPCM_SAMPLES_PER_FRAME;
            streamVariables.notificationsPerFrame = BLEAUDIO_NUM_NOT_PER_FRAME_ADPCM;
            streamVariables.requestedStreamType = AUDIO_DUPLEX_STREAM_TYPE_ADPCM;

        }

        streamVariables.numOfFramesToBuffer = I2SCC26XX_QUEUE_SIZE;
        streamVariables.rfFrameSize = BLEAUDIO_MAX_NOTSIZE;
        Audio_SetAudioDataLen(streamVariables.rfFrameSize);

        streamVariables.requestedStreamState = AudioDuplex_stream_active;
        streamVariables.maxVolume = streamVariables.maxVolume;
    }
    else if(streamVariables.streamType != requestedStreamType &&
            requestedStreamType != AudioDuplex_stream_idle)
    {
        AudioDuplex_stopStreaming();

        // Start stream, from IDLE
        streamVariables.requestedStreamType = requestedStreamType;
        return;
    }
    else
    {
        // Stop stream
        streamVariables.requestedStreamType = AUDIO_DUPLEX_STREAM_TYPE_NONE;
        streamVariables.requestedStreamState = AudioDuplex_stream_idle;
        // Start chain of events to stop stream
        AudioDuplex_stopStreaming();
        return;
    }

    // Increase TX power during stream
    HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);

    i2sHandle = AudioDuplex_openI2S();

    if (i2sHandle != NULL)
    {
        Display_print1(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                        "Opened I2S: %d samples/frame",
                        streamVariables.samplesPerFrame);
        // Move to send start command
        streamVariables.streamState = AudioDuplex_send_start_cmd;
        if(appAudioCB != NULL)
        {
            // Notify the Application of the event
            (*appAudioCB)(AUDIO_DUPLEX_SEND_START_CMD_EVENT);
        }
    }
    else
    {
        Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                        "Failed to opened I2S");
        // Return, or move to IDLE state
        streamVariables.streamState = AudioDuplex_stream_idle;
    }
}

/*********************************************************************
 * @fn      AudioDuplex_stopStreaming
 *
 * @brief   Close and cleanup audio stream
 *
 * @param   None.
 *
 * @return  None.
 */
void AudioDuplex_stopStreaming(void)
{
    streamVariables.requestedStreamState = AudioDuplex_send_stop_cmd;
    streamVariables.streamState = AudioDuplex_send_stop_cmd;
    // Start chain of events to stop stream
    AudioDuplex_sendStopCmd();

    // Turn output volume back down
    volume = 0;
    AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, AUDIO_DUPLEX_OUTPUT_OPTION, volume);
    // Volume control
    AudioCodecMicVolCtrl(AUDIO_CODEC_TI_3254, AUDIO_DUPLEX_INPUT_OPTION, volume);

    if (streamVariables.streamType == AUDIO_DUPLEX_CMD_START_MSBC)
    {
        sbc_finish(&sbc);
    }

    if (i2sHandle != NULL)
    {
        I2SCC26XX_close(i2sHandle);
        Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                        "Closed I2S driver");

        if (audio_decoded)
        {
            audio_decoded = NULL;
            Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                            "Free'd memory for I2S driver");
        }
        else
        {
            Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                            "Failed to free memory for I2S driver");
        }
        if (i2sContMgtBuffer)
        {
            i2sContMgtBuffer = NULL;
        }

        // Re-enable the instruction cache
        AudioDuplex_enableCache();
    }

    streamVariables.streamType = AUDIO_DUPLEX_CMD_STOP;
    streamVariables.streamState =  AudioDuplex_stream_idle;
    Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0, "No Stream");
}

/*********************************************************************
 * @fn      AudioDuplex_sendStartCmd
 *
 * @brief   Sends a start command to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioDuplex_sendStartCmd(void)
{
    // Check that we're in the correct state
    if (streamVariables.streamState == AudioDuplex_send_start_cmd)
    {
        uint8_t startCmd = streamVariables.streamType;
        uint8_t retVal = Audio_SetParameter(AUDIOPROFILE_START,
                                            AUDIOPROFILE_CMD_LEN,
                                            &startCmd);
        if (retVal == SUCCESS)
        {
            Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                            "Sent Start Cmd, initializing encoder");
            if (streamVariables.streamType == AUDIO_DUPLEX_STREAM_TYPE_MSBC)
            {
                // Initialize encoder
                sbc_init_msbc(&sbc, 0);
            }
            else
            {
                // Initialize encoder
                streamVariables.pv = 0;
                streamVariables.si = 0;
            }
            // Try next state
            streamVariables.streamState = AudioDuplex_start_I2S;
            if(appAudioCB != NULL)
            {
                // Notify the Application of the event
                (*appAudioCB)(AUDIO_DUPLEX_START_I2S_EVENT);
            }
        }
        else
        {
            if(appAudioCB != NULL)
            {
                // Notify the Application of the event
                (*appAudioCB)(AUDIO_DUPLEX_SEND_START_CMD_EVENT);
            }
        }
    }
    else
    {
        if(appAudioCB != NULL)
        {
            // Notify the Application of the event
            (*appAudioCB)(AUDIO_DUPLEX_START_I2S_EVENT);
        }
    }

}

/*********************************************************************
 * @fn      AudioDuplex_sendStopCmd
 *
 * @brief   Sends a stop command to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioDuplex_sendStopCmd(void)
{
    // Check that we're in the correct state
    if (streamVariables.streamState == AudioDuplex_send_stop_cmd)
    {
        uint8_t retVal = SUCCESS;

        if (linkDB_Up(audioConnectionHandle))
        {
            uint8_t stopCmd = AUDIO_DUPLEX_CMD_STOP;
            retVal = Audio_SetParameter(AUDIOPROFILE_START, AUDIOPROFILE_CMD_LEN,
                                         &stopCmd);
            if (retVal == SUCCESS)
            {
                // Reset TX power
                // Move to stop I2S stream
                HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
                streamVariables.streamState = AudioDuplex_stop_I2S;
                AudioDuplex_stopI2Sstream();
            }
            else
            {
                Display_print1(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                                "Failed to send STOP: %d", retVal);
                // Try again
                if(appAudioCB != NULL)
                {
                    // Notify the Application of the event
                    (*appAudioCB)(AUDIO_DUPLEX_SEND_STOP_CMD_EVENT);
                }
            }
        }
        else
        {
            // Reset TX power
            HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
            // Move to stop I2S stream
            streamVariables.streamState = AudioDuplex_stop_I2S;
            if(appAudioCB != NULL)
            {
                // Notify the Application of the event
                (*appAudioCB)(AUDIO_DUPLEX_STOP_I2S_EVENT);
            }
        }
    }
    else
    {
        // Try next state
        if(appAudioCB != NULL)
        {
            // Notify the Application of the event
            (*appAudioCB)(AUDIO_DUPLEX_STOP_I2S_EVENT);
        }
    }
}

/*********************************************************************
 * @fn      AudioDuplex_startI2Sstream
 *
 * @brief   Start I2S stream
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioDuplex_startI2Sstream(void)
{
    // Check that we're in the correct state
    if (streamVariables.streamState == AudioDuplex_start_I2S)
    {
        if (streamVariables.requestedStreamState == AudioDuplex_stream_active)
        {
            // It's now safe to update audio data length
            Audio_SetAudioDataLen(streamVariables.rfFrameSize);
            // Try to start I2S stream
            i2sStreamInProgress = I2SCC26XX_startStream(i2sHandle);
            if (i2sStreamInProgress)
            {
                    // Move to ACTIVE as we have completed start sequence
                    streamVariables.streamState = AudioDuplex_stream_active;

                if (streamVariables.streamType == AUDIO_DUPLEX_STREAM_TYPE_MSBC)
                {
                    Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                                    "mSBC Stream Started");
                }
                else if (streamVariables.streamType == AUDIO_DUPLEX_STREAM_TYPE_ADPCM)
                {
                    Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                                    "ADPCM Stream Started");
                }
            }
            else
            {
                Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                                "Failed to start I2S stream");
            }
        }
        else
        {
            Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                            "Started stream when Active was not requested");
        }
    }
}

/*********************************************************************
 * @fn      AudioDuplex_stopI2Sstream
 *
 * @brief   Stop I2S stream
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioDuplex_stopI2Sstream(void)
{
    // Check that we're in the correct state
    if (streamVariables.streamState == AudioDuplex_stop_I2S)
    {
        // Try to stop I2S stream
        if (I2SCC26XX_stopStream(i2sHandle))
        {
            Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                            "Stopped I2S stream");
            i2sStreamInProgress = false;
        }
        else
        {
            Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                            "Failed to stop I2S stream");
        }
    }
    else
    {
        Display_print1(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                        "Tried to stop I2S stream in state %d",
                        streamVariables.streamState);
    }
}

static I2SCC26XX_Handle AudioDuplex_openI2S(void)
{
    I2SCC26XX_Handle i2sHandleTmp = NULL;
    // Allocate memory for decoded PCM data
    i2sParams.ui32conMgtBufTotalSize =  I2S_BLOCK_OVERHEAD_IN_BYTES *  \
                                        streamVariables.numOfFramesToBuffer\
                                        * 2;

    i2sParams.ui32conBufTotalSize    =  sizeof(int16_t) * (streamVariables.samplesPerFrame * \
                                        streamVariables.numOfFramesToBuffer \
                                        * 2);

    // Disable instruction cache to use for audio buffers
    AudioDuplex_disableCache();
    i2sContMgtBuffer = (uint8_t *)(I2S_MEM_BASE + I2S_BUF + 1);
    audio_decoded = (int16_t *)I2S_MEM_BASE;

    if (audio_decoded)
    {
        // Setup I2S Params
        i2sParams.blockSize              = streamVariables.samplesPerFrame;
        i2sParams.pvContBuffer           = (void *) audio_decoded;
        i2sParams.pvContMgtBuffer        = (void *) i2sContMgtBuffer;

        // Reset I2S handle and attempt to open
        i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
        i2sHandleTmp = I2SCC26XX_open(i2sHandle, &i2sParams);

        volume = streamVariables.maxVolume;
    }
    else
    {
        Display_print0(hDisp, AUDIO_DUPLEX_DISP_STAT2, 0,
                        "Failed to allocate mem for I2S");
        if (i2sContMgtBuffer)
        {
          i2sContMgtBuffer = NULL;
        }
    }

    AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, AUDIO_DUPLEX_OUTPUT_OPTION, volume);
    // Volume control
    AudioCodecMicVolCtrl(AUDIO_CODEC_TI_3254, AUDIO_DUPLEX_INPUT_OPTION, volume);

    return (i2sHandleTmp);
}

/*********************************************************************
 * @fn      AudioDuplex_transmitAudioFrame
 *
 * @brief   Transmits processed audio frame to connected device
 *
 * @param   buf - pointer to PDM buffer
 *
 * @return  None.
 */
static bStatus_t AudioDuplex_transmitAudioFrame(uint8_t *buf)
{
    bStatus_t notiStatus = SUCCESS;
    if(linkDB_Up(audioConnectionHandle))
    {
        // Send streamVariables.notificationsPerFrame GATT notifications for every audio frame
        for (int i = 0; i < streamVariables.notificationsPerFrame; )
        {
            notiStatus = Audio_SetParameter(AUDIOPROFILE_AUDIO, streamVariables.rfFrameSize, buf);
            if (notiStatus == SUCCESS)
            {
                // Move on to next section of audio frame
                buf += streamVariables.rfFrameSize;
                i++;
            }
            else
            {
                // Return the error immediately
                return(notiStatus);
            }
        }

    }
    else
    {
        notiStatus = FAILURE;
    }

    return(notiStatus);
}

/*********************************************************************
 * @fn      AudioDuplex_shiftEncodedBuffer
 *
 * @brief   Shifts an encoded buffer to include headers for RF
 *          This is used to prevent double copying
 *
 * @param   encodedBuf - The encoded buffer for copying
 * @param   len - length of input buffer
 * @param   streamType - Type of stream, determines how much to shift
 *
 * @return  None.
 */
static void AudioDuplex_shiftEncodedBuffer(uint8_t *encodedBuf, uint8_t len,
                                            uint8_t streamType)
{
    uint8_t hdrSize = 0;

    if(streamType == AUDIO_DUPLEX_STREAM_TYPE_ADPCM)
    {
        hdrSize = AUDIO_DUPLEX_HDRSIZE_ADPCM;
    }
    else if (streamType == AUDIO_DUPLEX_STREAM_TYPE_ADPCM)
    {
        hdrSize = AUDIO_DUPLEX_HDRSIZE_MSBC;
    }
    else
    {
        // Assume unencoded buffer, return
        return;
    }

    // Shift the buffer based on stream type
    for( int16_t index = ( len - 1 ) ; index >= hdrSize ; index-- )
    {
      encodedBuf[index] = encodedBuf[index - hdrSize];
    }
}

/*********************************************************************
 * @fn      AudioDuplex_disableCache
 *
 * @brief   Disables the instruction cache and sets power constaints
 *          This prevents the device from sleeping while streaming
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioDuplex_disableCache()
{
    uint_least16_t hwiKey = Hwi_disable();
    Power_setConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
    Power_setConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
    VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_DISABLED, true);
    Hwi_restore(hwiKey);
}

/*********************************************************************
 * @fn      AudioDuplex_enableCache
 *
 * @brief   Enables the instruction cache and releases power constaints
 *          Allows device to sleep again
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioDuplex_enableCache()
{
    uint_least16_t hwiKey = Hwi_disable();
    Power_releaseConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
    Power_releaseConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
    VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_ENABLED, true);
    Hwi_restore(hwiKey);
}

/*********************************************************************
 * @fn      AudioDuplex_i2sCallbackFxn
 *
 * @brief   Callback functtion from I2S driver, sets events to be
 *          processed in the task context
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioDuplex_i2sCallbackFxn(I2SCC26XX_Handle handle,
                                        I2SCC26XX_StreamNotification *notification)
{
    if (notification->status == I2SCC26XX_STREAM_ERROR)
    {
        if(appAudioCB != NULL)
        {
            // Notify the Application of the event
            (*appAudioCB)(AUDIO_DUPLEX_I2S_ERROR_EVENT);
        }
    }
    else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY)
    {
        if(appAudioCB != NULL)
        {
            // Notify the Application of the event
            (*appAudioCB)(AUDIO_DUPLEX_I2S_FRAME_EVENT);
        }
    }
    else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY_BUT_NO_AVAILABLE_BUFFERS)
    {
        if(appAudioCB != NULL)
        {
            // Notify the Application of the event
            (*appAudioCB)(AUDIO_DUPLEX_I2S_FRAME_EVENT);
        }
    }
}

/*********************************************************************
*********************************************************************/
