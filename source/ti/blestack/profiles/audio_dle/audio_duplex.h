/*
 * Filename: audio_duplex.h
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

#ifndef AUDIO_DUPLEX_H
#define AUDIO_DUPLEX_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * CONSTANTS
 */

#define AUDIO_DUPLEX_SUCCESS                        0
#define AUDIO_MEM_FAIL                              -1
#define AUDIO_HW_FAIL                               -2
#define AUDIO_INVALID_PARAMS                        -3

#define AUDIO_DUPLEX_I2S_FRAME_EVENT                0x01
#define AUDIO_DUPLEX_I2S_ERROR_EVENT                0x02
#define AUDIO_DUPLEX_SEND_STOP_CMD_EVENT            0x04
#define AUDIO_DUPLEX_SEND_START_CMD_EVENT           0x08
#define AUDIO_DUPLEX_STOP_I2S_EVENT                 0x10
#define AUDIO_DUPLEX_START_I2S_EVENT                0x20

#define AUDIO_DUPLEX_CMD_STOP                       0x00
#define AUDIO_DUPLEX_CMD_START                      0x04
#define AUDIO_DUPLEX_CMD_START_MSBC                 0x05
#define AUDIO_DUPLEX_CMD_NONE                       0xFF

#define AUDIO_DUPLEX_STREAM_TYPE_NONE               AUDIO_DUPLEX_CMD_STOP
#define AUDIO_DUPLEX_STREAM_TYPE_ADPCM              AUDIO_DUPLEX_CMD_START
#define AUDIO_DUPLEX_STREAM_TYPE_MSBC               AUDIO_DUPLEX_CMD_START_MSBC

#define AUDIO_DUPLEX_VOLUME_UP                      0x01
#define AUDIO_DUPLEX_VOLUME_DOWN                    0xFF

#define AUDIO_DUPLEX_INPUT_OPTION                   AUDIO_CODEC_MIC_ONBOARD
#define AUDIO_DUPLEX_OUTPUT_OPTION                  AUDIO_CODEC_SPEAKER_HP
#define AUDIO_DUPLEX_MAX_VOLUME                     75


#define AUDIO_DUPLEX_BUFSIZE_ADPCM                  96
#define AUDIO_DUPLEX_HDRSIZE_ADPCM                  4

#define AUDIO_DUPLEX_ADPCM_SAMPLES_PER_FRAME        AUDIO_DUPLEX_BUFSIZE_ADPCM

#define AUDIO_DUPLEX_MSBC_SAMPLES_PER_FRAME         120
#define AUDIO_DUPLEX_HDRSIZE_MSBC                   1
#define AUDIO_DUPLEX_MSBC_ENCODED_SIZE              57

#define AUDIO_DUPLEX_SAMPLE_RATE                    16000
#define AUDIO_DUPLEX_NUM_CHAN                       2

/*********************************************************************
 * TYPEDEFS
 */
typedef enum
{
  AudioDuplex_stream_idle,
  AudioDuplex_send_start_cmd,
  AudioDuplex_start_I2S,
  AudioDuplex_stream_active,
  AudioDuplex_send_stop_cmd,
  AudioDuplex_stop_I2S,
}AudioDuplex_streamState;

/**
 * This enum is used to identify the data passed on from _processGATTMsg
 */
typedef enum
{
  AudioDuplex_start_stop,
  AudioDuplex_data
} AudioDuplex_dataType;

/**
 * General data struct.
 */
typedef struct
{
  uint16 len;    //!< Length of data
  uint8 *pValue; //!< Data
} AudioDuplex_audioData;

typedef struct
{
  AudioDuplex_streamState    streamState;
  AudioDuplex_streamState    requestedStreamState;
  uint8_t           streamType;
  uint8_t           requestedStreamType;
  uint8_t           samplesPerFrame;
  uint8_t           notificationsPerFrame;
  uint8_t           numOfFramesToBuffer;
  uint8_t           rfFrameSize;
  int8_t            si; //Step Index
  int16_t           pv; //Predict Value
  int8_t            si_rx;
  int16_t           pv_rx;
  int8_t            maxVolume;
} AudioDuplex_streamVars;

/**
 * @brief Audio Duplex Callback Function
 *
 * This callback notifies the application of an event that occurred in a
 * lower layer audio callback such as the I2S module
 *
 * @param events Bitmask of events posted by audio duplex module
 */
typedef void (*pfnAudioDuplexCB_t)(uint8_t events);

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
extern int8_t AudioDuplex_open(Display_Handle displayHandle,
                                PIN_Handle pinHandle,
                                pfnAudioDuplexCB_t inputCB);

/*********************************************************************
 * @fn      AudioDuplex_close
 *
 * @brief   Closes hardware, stops streaming
 *
 * @param   None.
 *
 * @return  None.
 */
extern void AudioDuplex_close(void);

/*********************************************************************
 * @fn      AudioDuplex_setConnectionHandle
 *
 * @brief   Set the connection handle of audio streaming
 *
 * @param   cxnHdl - Connection handle.
 *
 * @return  None
 */
extern void AudioDuplex_setConnectionHandle(uint16_t connectionHandle);

/*********************************************************************
 * @fn      AudioDuplex_getConnectionHandle
 *
 * @brief   Get the connection handle of audio streaming
 *
 * @return  connectionHandle- the handle used to stream
 */
extern uint16_t AudioDuplex_getConnectionHandle(void);

/*********************************************************************
 * @fn      AudioDuplex_eventHandler
 *
 * @brief   Processes Events called from I2S or BLE-Stack callbacks
 *
 * @param   events - Events to process.
 *
 * @return  None
 */
extern void AudioDuplex_eventHandler(uint8_t events);

/*********************************************************************
 * @fn      AudioDuplex_processData
 *
 * @brief   Process incoming audio data
 *
 * @return  None
 */
extern void AudioDuplex_processData(AudioDuplex_dataType data_type,
                                    AudioDuplex_audioData *pMsg);

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
extern void AudioDuplex_startStreaming(uint8_t requestedStreamType);

/*********************************************************************
 * @fn      AudioDuplex_stopStreaming
 *
 * @brief   Close and cleanup audio stream
 *
 * @param   None.
 *
 * @return  None.
 */
extern void AudioDuplex_stopStreaming(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_DUPLEX_H */
