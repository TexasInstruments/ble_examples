/*
 * Filename: audio_central.c
 *
 * Description: This is the audio part of the simple_peripheral example
 * modified to send audio data over BLE.
 *
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

#include <ti/display/Display.h>
#include "board_key.h"

#include "board.h"

#include "hci_tl.h"

#include "simple_central_bidirectional_audio.h"

#include "audio_central.h"
#include <ti/blestack/profiles/audio_dle/audio_profile_dle.h>

#include <ti/drivers/pdm/Codec1.h>
#include <ti/blestack/audiocodec/audiocodec.h>
#include <third_party/sbc/include/msbc_library.h>
#include <ti/drivers/i2s/I2SCC26XX.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

/*********************************************************************
 * CONSTANTS
 */
const unsigned char msbc_data[] =
{
    0xad, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x01, 0x12,
    0xe1, 0xeb, 0x31, 0x60, 0x76, 0xcd, 0x61, 0xf3,
    0x40, 0xe5, 0x09, 0x38, 0xc4, 0xba, 0xa3, 0xa2,
    0x38, 0x7b, 0x09, 0xb8, 0x1d, 0xdf, 0x30, 0x7c,
    0xd1, 0xa2, 0x42, 0x4b, 0xe5, 0xae, 0xa9, 0x15,
    0x9e, 0x1e, 0xc1, 0x62, 0x07, 0x6e, 0xb5, 0x1f,
    0x33, 0x56, 0x90, 0x92, 0xf9, 0x7b, 0xaa, 0x35,
    0xe0
};

#define BLEAUDIO_NUM_NOT_PER_FRAME_ADPCM  1
#define BLEAUDIO_NUM_NOT_PER_FRAME_MSBC   1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void AudioCentral_transmitAudioFrame(uint8_t *buf);
static void AudioCentral_i2sCallbackFxn(I2SCC26XX_Handle handle,
                                        I2SCC26XX_StreamNotification *notification);

/*********************************************************************
 * LOCAL VARIABLES
 */
static int16_t *audio_decoded;
static uint8_t *i2sContMgtBuffer;
static uint8_t audio_encoded[100] = {0};
static uint8_t audio_encoded_to_send[100] = {0};
static sbc_t sbc = {0};
static size_t written = 0;

static streamVars_t streamVariables = {
  STREAM_STATE_IDLE,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Board_LED1
};

static I2SCC26XX_Handle i2sHandle = NULL;
static I2SCC26XX_StreamNotification i2sStream;
static I2SCC26XX_Params i2sParams = {
  .requestMode            = I2SCC26XX_CALLBACK_MODE,
  .ui32requestTimeout     = BIOS_WAIT_FOREVER,
  .callbackFxn            = AudioCentral_i2sCallbackFxn,
  .blockSize              = MSBC_SAMPLES_PER_FRAME,
  .pvContBuffer           = NULL,
  .ui32conBufTotalSize    = 0,
  .pvContMgtBuffer        = NULL,
  .ui32conMgtBufTotalSize = 0,
  .currentStream          = &i2sStream
};
static bool i2sStreamInProgress = false;

static uint8_t volume = 0, seqNum = 0;

// Display Interface
static Display_Handle hDisp = NULL;
// IO handle
static PIN_Handle hPins = NULL;


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AudioCentral_init
 *
 * @brief   Called during initialization, opens codec and I2S driver
 *
 * @param   None.
 *
 * @return  None.
 */
void AudioCentral_init(Display_Handle displayHandle, PIN_Handle pinHandle)
{
  // Share display handle with application, we still operate under the same task context.
  hDisp = displayHandle;
  // Share pin handle with application, we still operate under the same task context.
  hPins = pinHandle;

  /* Initialize I2S driver */
  i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
  I2SCC26XX_init(i2sHandle);

  // Initialize TLV320AIC3254 Codec on Audio BP
  AudioCodecOpen();
  // Configure Codec
  AudioCodecConfig(AUDIO_CODEC_TI_3254, AUDIO_CODEC_16_BIT, 16000, 2, OUTPUT_OPTION, INPUT_OPTION);
}

/*********************************************************************
 * @fn      AudioCentral_eventHandler
 *
 * @brief   Event handler called by the Simple BLE Peripheral task function.
 *
 * @param   events - Events to process
 *
 * @return  processedEvents: Bitmask of events actually processed.
 */
uint16_t AudioCentral_eventHandler(uint8_t events)
{
  uint16_t processedEvents = 0;

  if (events & AUDIO_I2S_FRAME_EVENT)
  {
    events &= ~AUDIO_I2S_FRAME_EVENT;
    if (i2sStreamInProgress) {
      I2SCC26XX_BufferRequest bufferRequest;
      I2SCC26XX_BufferRelease bufferRelease;
      bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN;
      // Indicate that we request buffer from input stream
      bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
      while (gotBuffer) {
        PIN_setOutputValue( hPins, Board_DIO26_ANALOG, 0);
        if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC) {
          sbc_encode(&sbc, (int16_t *)bufferRequest.bufferIn,
                      streamVariables.samplesPerFrame * sizeof(int16_t),
                      audio_encoded_to_send, MSBC_ENCODED_SIZE, &written);
          audio_encoded_to_send[1] = seqNum++;
        }
        else {
          audio_encoded_to_send[0] = seqNum++;
          // Send previous PV and SI
          audio_encoded_to_send[1] = streamVariables.si;
          audio_encoded_to_send[2] = LO_UINT16(streamVariables.pv);
          audio_encoded_to_send[3] = HI_UINT16(streamVariables.pv);
          Codec1_encodeBuff((uint8_t *)&audio_encoded_to_send[4],
                            (int16_t *)bufferRequest.bufferIn,
                            streamVariables.samplesPerFrame,
                            &streamVariables.si, &streamVariables.pv);
        }
        AudioCentral_transmitAudioFrame(audio_encoded_to_send);
        PIN_setOutputValue( hPins, Board_DIO26_ANALOG, 1);
        bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
        bufferRelease.bufferHandleOut = NULL;
        I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
        bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN;
        // Indicate that we request buffer from input stream
        gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
      }
    }
  }
  if (events & AUDIO_I2S_ERROR_EVENT)
  {
   events &= ~AUDIO_I2S_ERROR_EVENT;
   PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 0);

   // Move to stop state
   uint_least16_t hwiKey = Hwi_disable();
   streamVariables.streamType = STREAM_STATE_SEND_STOP_CMD;
   SimpleBLECentral_setEvent(AUDIO_SEND_STOP_CMD_EVENT);
   Hwi_restore(hwiKey);
   PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 1);
  }

  return processedEvents;
}

/*********************************************************************
 * @fn      AudioCentral_processData
 *
 * @brief   Process incoming audio data
 *
 * @return  None
 */
void AudioCentral_processData(AUDIO_DATA_TYPE_E data_type,
                              genericAudioData_t *pMsg)
{
  static uint8_t audio_pkt_counter = 0, frameReady = FALSE;
  static uint16_t prevSeqNum = 0;
  static int numberOfPackets = 0, lostPackets = 0;
  static int msbcnumberOfPackets = 0, msbclostPackets = 0;
#ifdef LOOP_BACK_AUDIO
  static uint8_t returnNot;
#endif //LOOP_BACK_AUDIO
  // Check to see if notification is from audio data or control char
  if (data_type == AUDIO_DATA)
  {
    PIN_setOutputValue( hPins, Board_DIO28_ANALOG, 0);

    if (streamVariables.streamType == BLE_AUDIO_CMD_START)
    {
      // Copy to encoded buffer
      memcpy(&audio_encoded[audio_pkt_counter * pMsg->len], pMsg->pValue,
              pMsg->len);

      // Increment the packet counter, handle wrap around logic
      audio_pkt_counter++;
      if (audio_pkt_counter == streamVariables.notificationsPerFrame)
      {
        audio_pkt_counter = 0;
        frameReady = TRUE;
      }
      numberOfPackets++;
      if (((prevSeqNum + 1) & 0x00FF) != pMsg->pValue[0]) {
        uint16_t curSeqNum = pMsg->pValue[0];
        uint8_t missedFrames = 0;
        if (pMsg->pValue[0] > prevSeqNum) {
          missedFrames = curSeqNum - prevSeqNum;
        } else {
          missedFrames = (curSeqNum + 256) - prevSeqNum;
        }
        numberOfPackets += missedFrames;
        lostPackets += missedFrames;
        Display_print2(hDisp, 5, 0, "Missing frame, PER %d/%d", lostPackets,
                        numberOfPackets);
      }
      prevSeqNum = pMsg->pValue[0];
    }
    else if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC)
    {
      if ((pMsg->pValue[0] == 0xAD) &&
          (pMsg->pValue[2] == 0x00))
      {
        audio_pkt_counter = 0;
      }
      msbcnumberOfPackets++;
      if (((prevSeqNum + 1) & 0x00FF) != pMsg->pValue[1]) {
          uint16_t curSeqNum = pMsg->pValue[1];
          uint8_t missedFrames = 0;
          if (pMsg->pValue[1] > prevSeqNum) {
              missedFrames = curSeqNum - prevSeqNum;
          } else {
              missedFrames = (curSeqNum + 256) - prevSeqNum;
          }
          msbcnumberOfPackets += missedFrames;
          msbclostPackets += missedFrames;
          Display_print4(hDisp, 5, 0, "Missing frame, PER %d/%d (%d vs %d)",
                          msbclostPackets, msbcnumberOfPackets, pMsg->pValue[1],
                          prevSeqNum);
      }
      prevSeqNum = pMsg->pValue[1];

      if (audio_pkt_counter == 2) {
        memcpy(&audio_encoded[audio_pkt_counter * pMsg->len], pMsg->pValue,
                pMsg->len - 3);
        frameReady = TRUE;
      }
      else
      {
        memcpy(&audio_encoded[audio_pkt_counter * pMsg->len], pMsg->pValue, pMsg->len);
        if (streamVariables.notificationsPerFrame == 1) {
          frameReady = TRUE;
        }
      }
      audio_pkt_counter++;
    }
    if (frameReady == TRUE) {
      if (i2sStreamInProgress) {
        I2SCC26XX_BufferRequest bufferRequest;
        I2SCC26XX_BufferRelease bufferRelease;
        bufferRequest.buffersRequested = I2SCC26XX_BUFFER_OUT;
        // Indicate that we request buffer from output stream
        bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
        if (gotBuffer) {
          if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC) {
            sbc_decode(&sbc, audio_encoded, MSBC_ENCODED_SIZE,
                      (int16_t *)bufferRequest.bufferOut,
                      streamVariables.samplesPerFrame * sizeof(int16_t),
                      &written);
          }
          else {
            streamVariables.pv_rx = BUILD_UINT16(audio_encoded[2],
                                                  audio_encoded[3]);
            streamVariables.si_rx = audio_encoded[1];
            Codec1_decodeBuff((int16_t *)bufferRequest.bufferOut,
                              (uint8_t *)&audio_encoded[4],
                              streamVariables.samplesPerFrame * sizeof(int16_t),
                              &streamVariables.si_rx, &streamVariables.pv_rx);
          }
          bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
          bufferRelease.bufferHandleIn = NULL;
          I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
        }
        if (gotBuffer) {
          if (volume < streamVariables.maxVolume) {
            volume++;
            if ((volume % 20) == 0) {
              // Volume control
              AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, OUTPUT_OPTION, volume);
#if INPUT_OPTION
              // Volume control
              AudioCodecMicVolCtrl(AUDIO_CODEC_TI_3254, INPUT_OPTION, volume);
#endif //INPUT_OPTION
            }
          } else if (volume > streamVariables.maxVolume) {
            volume--;
            // Turn down volume
            if ((volume % 20) == 0) {
              // Volume control
              AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, OUTPUT_OPTION, volume);
#if INPUT_OPTION
              // Volume control
              AudioCodecMicVolCtrl(AUDIO_CODEC_TI_3254, INPUT_OPTION, volume);
#endif //INPUT_OPTION
            }
          }
        }
      }
      else if (streamVariables.i2sOpened == TRUE) {
        PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 0);
        // Start I2S stream
        i2sStreamInProgress = I2SCC26XX_startStream(i2sHandle);
        if (!i2sStreamInProgress) {
          Display_print0(hDisp, 5, 0, "Failed to start I2S stream");
          I2SCC26XX_stopStream(i2sHandle);
        }
        else {
          Display_print0(hDisp, 5, 0, "Started I2S stream");
        }
        PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 1);
      }
      else {
        // Failed to open, take the opportunity to try again
        // Allocate memory for decoded PCM data
        i2sParams.ui32conMgtBufTotalSize = I2S_BLOCK_OVERHEAD_IN_BYTES * \
                                            streamVariables.numOfFramesToBuffer * 2;
        i2sParams.ui32conBufTotalSize    = sizeof(int16_t) * (streamVariables.samplesPerFrame * \
                                           streamVariables.numOfFramesToBuffer * 2);

        i2sContMgtBuffer = ICall_malloc(i2sParams.ui32conMgtBufTotalSize);
        audio_decoded = ICall_malloc(i2sParams.ui32conBufTotalSize);
        if (audio_decoded) {
          i2sParams.blockSize              = streamVariables.samplesPerFrame;
          i2sParams.pvContBuffer           = (void *) audio_decoded;
          i2sParams.pvContMgtBuffer        = (void *) i2sContMgtBuffer;
          I2SCC26XX_open(i2sHandle, &i2sParams);
          Display_print1(hDisp, 5, 0, "Opened I2S driver: %d", 3);
          volume = 40;
          streamVariables.i2sOpened = TRUE;
        }
        else {
          Display_print0(hDisp, 5, 0, "Failed to allocate mem for I2S driver on frame");
          if (i2sContMgtBuffer) {
            ICall_free(i2sContMgtBuffer);
            i2sContMgtBuffer = NULL;
          }

        }
      }
      frameReady = FALSE;
    }
    else if ((streamVariables.i2sOpened == FALSE) && (streamVariables.streamType != BLE_AUDIO_CMD_STOP)) {
      // Failed to open, take the opportunity to try again
      // Allocate memory for decoded PCM data
      i2sParams.ui32conMgtBufTotalSize = I2S_BLOCK_OVERHEAD_IN_BYTES * \
                                          streamVariables.numOfFramesToBuffer * 2;
      i2sParams.ui32conBufTotalSize    = sizeof(int16_t) * (streamVariables.samplesPerFrame * \
                                          streamVariables.numOfFramesToBuffer * 2);

      i2sContMgtBuffer = ICall_malloc(i2sParams.ui32conMgtBufTotalSize);
      audio_decoded = ICall_malloc(i2sParams.ui32conBufTotalSize);
      if (audio_decoded) {
        i2sParams.blockSize              = streamVariables.samplesPerFrame;
        i2sParams.pvContBuffer           = (void *) audio_decoded;
        i2sParams.pvContMgtBuffer        = (void *) i2sContMgtBuffer;
        I2SCC26XX_open(i2sHandle, &i2sParams);
        volume = 40;
        Display_print1(hDisp, 5, 0, "Opened I2S driver: %d", 4);
        streamVariables.i2sOpened = TRUE;
      }
      else {
        Display_print0(hDisp, 5, 0, "Failed to allocate mem for I2S driver on fragment");
        if (i2sContMgtBuffer) {
          ICall_free(i2sContMgtBuffer);
          i2sContMgtBuffer = NULL;
        }

      }
    }
#ifdef LOOP_BACK_AUDIO
    if (streamVariables.streamType == BLE_AUDIO_CMD_START)
    {
      returnNot = Audio_SetParameter(AUDIOPROFILE_AUDIO,
                                      streamVariables.rfFrameSize,
                                      pMsg->pValue);
      if (returnNot != SUCCESS)
      {
        Display_print0(hDisp, 5, 0, "Failed to send back the audio data msg");
      }
    }
#endif //LOOP_BACK_AUDIO
    PIN_setOutputValue( hPins, Board_DIO28_ANALOG, 1);
  }
  else if (data_type == AUDIO_DATA_START_STOP)
  {
    PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 0);
    // Audio/Voice commands are 1B in length
    if(AUDIOPROFILE_CMD_LEN == pMsg->len)
    {
      // If we received a stop command reset the audio_pkt_counter, SI, PV
      if(BLE_AUDIO_CMD_START == *(pMsg->pValue))
      {
        if (streamVariables.streamType != BLE_AUDIO_CMD_STOP) {
          Display_print0(hDisp, 5, 0, "Already started stream");
        }
        else {
          audio_pkt_counter = 0;
          PIN_setOutputValue(hPins, Board_RLED, Board_LED_ON);
          streamVariables.streamType = BLE_AUDIO_CMD_START;
          streamVariables.notificationsPerFrame = BLEAUDIO_NUM_NOT_PER_FRAME_ADPCM;
          streamVariables.numOfFramesToBuffer = I2SCC26XX_QUEUE_SIZE - 2;
          streamVariables.activeLED = Board_LED2;
          streamVariables.samplesPerFrame = ADPCM_SAMPLES_PER_FRAME;
          streamVariables.maxVolume = 75;
          streamVariables.rfFrameSize = BLEAUDIO_MAX_NOTSIZE;
          Audio_SetAudioDataLen(streamVariables.rfFrameSize);

          // Allocate memory for decoded PCM data
          i2sParams.ui32conMgtBufTotalSize = I2S_BLOCK_OVERHEAD_IN_BYTES * \
                                              streamVariables.numOfFramesToBuffer * 2;
          i2sParams.ui32conBufTotalSize    = sizeof(int16_t) * (streamVariables.samplesPerFrame * \
                                              streamVariables.numOfFramesToBuffer * 2);

          i2sContMgtBuffer = ICall_malloc(i2sParams.ui32conMgtBufTotalSize);
          audio_decoded = ICall_malloc(i2sParams.ui32conBufTotalSize);
          if (audio_decoded) {
            i2sParams.blockSize              = streamVariables.samplesPerFrame;
            i2sParams.pvContBuffer           = (void *) audio_decoded;
            i2sParams.pvContMgtBuffer        = (void *) i2sContMgtBuffer;
            I2SCC26XX_open(i2sHandle, &i2sParams);
            Display_print1(hDisp, 5, 0, "Opened I2S driver: %d", 1);
            streamVariables.i2sOpened = TRUE;
          }
          else {
            Display_print0(hDisp, 5, 0, "Failed to allocate mem for I2S driver on start");
            if (i2sContMgtBuffer) {
              ICall_free(i2sContMgtBuffer);
              i2sContMgtBuffer = NULL;
            }
          }
          Display_print0(hDisp, 5, 0, "ADPCM Stream");
        }
      }
      else if(BLE_AUDIO_CMD_START_MSBC == *(pMsg->pValue))
      {
        if (streamVariables.streamType != BLE_AUDIO_CMD_STOP) {
          Display_print0(hDisp, 5, 0, "Already started stream");
        }
        else {
          audio_pkt_counter = 0;
          PIN_setOutputValue(hPins, Board_RLED, Board_LED_ON);
          streamVariables.streamType = BLE_AUDIO_CMD_START_MSBC;
          streamVariables.notificationsPerFrame = BLEAUDIO_NUM_NOT_PER_FRAME_MSBC;
          streamVariables.numOfFramesToBuffer = I2SCC26XX_QUEUE_SIZE;
          streamVariables.activeLED = Board_LED1;
          streamVariables.samplesPerFrame = MSBC_SAMPLES_PER_FRAME;
          streamVariables.maxVolume = 75;
          streamVariables.rfFrameSize = MSBC_ENCODED_SIZE;
          Audio_SetAudioDataLen(streamVariables.rfFrameSize);

          // Allocate memory for decoded PCM data
          i2sParams.ui32conMgtBufTotalSize = I2S_BLOCK_OVERHEAD_IN_BYTES * \
                                              streamVariables.numOfFramesToBuffer * 2;
          i2sParams.ui32conBufTotalSize    = sizeof(int16_t) * (streamVariables.samplesPerFrame * \
                                              streamVariables.numOfFramesToBuffer * 2);

          i2sContMgtBuffer = ICall_malloc(i2sParams.ui32conMgtBufTotalSize);
          audio_decoded = ICall_malloc(i2sParams.ui32conBufTotalSize);
          if (audio_decoded) {
            i2sParams.blockSize              = streamVariables.samplesPerFrame;
            i2sParams.pvContBuffer           = (void *) audio_decoded;
            i2sParams.pvContMgtBuffer        = (void *) i2sContMgtBuffer;
            I2SCC26XX_open(i2sHandle, &i2sParams);
            Display_print1(hDisp, 5, 0, "Opened I2S driver: %d", 2);
            streamVariables.i2sOpened = TRUE;
          }
          else {
            Display_print0(hDisp, 5, 0, "Failed to allocate mem for I2S driver on start");
            if (i2sContMgtBuffer) {
              ICall_free(i2sContMgtBuffer);
              i2sContMgtBuffer = NULL;
            }
          }
          // Initialize encoder
          sbc_init_msbc(&sbc, 0);
          Display_print0(hDisp, 5, 0, "mSBC Stream");
        }
      }
      else if(BLE_AUDIO_CMD_STOP == *(pMsg->pValue))
      {
        audio_pkt_counter = 0;
        AudioCentral_stopStreaming();
      }
    }
    PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 1);
  }
}

/*********************************************************************
 * @fn      AudioCentral_stopStreaming
 *
 * @brief   Close and cleanup audio stream
 *
 * @param   None.
 *
 * @return  None.
 */
void AudioCentral_stopStreaming(void)
{
  PIN_setOutputValue(hPins, streamVariables.activeLED, 0);
  if (i2sStreamInProgress) {
    I2SCC26XX_stopStream(i2sHandle);
    i2sStreamInProgress = false;
    Display_print0(hDisp, 5, 0, "Stopped I2S stream");

    /* Turn output volume back down */ //TODO: Turn off codec
    volume = 0;
    AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, OUTPUT_OPTION, volume);
#if INPUT_OPTION
    // Volume control
    AudioCodecMicVolCtrl(AUDIO_CODEC_TI_3254, INPUT_OPTION, volume);
#endif //INPUT_OPTION



    if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC)
    {
      sbc_finish(&sbc);
    }
  }
  if (streamVariables.i2sOpened == TRUE)
  {
    I2SCC26XX_close(i2sHandle);
    streamVariables.i2sOpened = FALSE;
    Display_print0(hDisp, 5, 0, "Closed I2S driver");
    if (audio_decoded) {
      ICall_free(audio_decoded);
      audio_decoded = NULL;
      Display_print0(hDisp, 5, 0, "Free'd memory for I2S driver");
    }
    else {
      Display_print0(hDisp, 5, 0, "Failed to free memory for I2S driver");
    }
    if (i2sContMgtBuffer) {
      ICall_free(i2sContMgtBuffer);
      i2sContMgtBuffer = NULL;
    }
  }
  streamVariables.streamType = BLE_AUDIO_CMD_STOP;
  Display_print0(hDisp, 5, 0, "No Stream");
}


/*********************************************************************
 * @fn      AudioCentral_i2sCallbackFxn
 *
 * @brief   Callback functtion from I2S driver, sets events to be
 *          processed in the task context
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioCentral_i2sCallbackFxn(I2SCC26XX_Handle handle,
                                        I2SCC26XX_StreamNotification *notification)
{
    if (notification->status == I2SCC26XX_STREAM_ERROR) {
        SimpleBLECentral_setEvent(AUDIO_I2S_ERROR_EVENT);
    }
    else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY) {
      // Provide buffer
      SimpleBLECentral_setEvent(AUDIO_I2S_FRAME_EVENT);
      PIN_setOutputValue( hPins, Board_DIO25_ANALOG,
                          !(PIN_getOutputValue(Board_DIO25_ANALOG)));
      PIN_setOutputValue( hPins,
                          (streamVariables.activeLED == Board_LED0) ? Board_LED1 : Board_LED0, 0);
    }
    else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY_BUT_NO_AVAILABLE_BUFFERS) {
      // Provide buffer
      SimpleBLECentral_setEvent(AUDIO_I2S_FRAME_EVENT);
      PIN_setOutputValue( hPins, Board_DIO25_ANALOG,
                          !(PIN_getOutputValue(Board_DIO25_ANALOG)));
      PIN_setOutputValue( hPins,
                          (streamVariables.activeLED == Board_LED0) ? Board_LED1 : Board_LED0, 1);
    }
    else {
      PIN_setOutputValue( hPins,
                          (streamVariables.activeLED == Board_LED0) ? Board_LED1 : Board_LED0, 1);
    }
}

/*********************************************************************
 * @fn      AudioCentral_transmitAudioFrame
 *
 * @brief   Transmits processed audio frame to connected device
 *
 * @param   buf - pointer to PDM buffer
 *
 * @return  None.
 */
static void AudioCentral_transmitAudioFrame(uint8_t *buf)
{
  PIN_setOutputValue( hPins, Board_DIO28_ANALOG, 0);
  // Send streamVariables.notificationsPerFrame GATT notifications for every audio frame
  for (int i = 0; i < streamVariables.notificationsPerFrame; )
  {
    if (Audio_SetParameter(AUDIOPROFILE_AUDIO, streamVariables.rfFrameSize, buf) == SUCCESS)
    {
      PIN_setOutputValue( hPins, Board_DIO28_ANALOG,
                          !(PIN_getOutputValue(Board_DIO28_ANALOG)));
      // Move on to next section of audio frame
      buf += streamVariables.rfFrameSize;
      i++;
      PIN_setOutputValue(hPins, streamVariables.activeLED, 1);
    }
    else
    {
      PIN_setOutputValue(hPins, streamVariables.activeLED, 0);
    }
  }
  PIN_setOutputValue( hPins, Board_DIO28_ANALOG, 1);
}

/*********************************************************************
*********************************************************************/
