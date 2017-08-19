/*
 * Filename: audio_peripheral.c
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
#include <ti/sysbios/BIOS.h>

#include <ti/display/Display.h>
#include "board_key.h"

#include "board.h"

#include "hci_tl.h"

#include "simple_peripheral_bidirectional_audio.h"

#include "audio_peripheral.h"
#include <ti/blestack/profiles/audio_dle/audio_profile_dle.h>

#include <ti/drivers/pdm/Codec1.h>
#include <ti/blestack/audiocodec/audiocodec.h>
#include <third_party/sbc/include/msbc_library.h>
#include <ti/drivers/i2s/I2SCC26XX.h>

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

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8_t AudioPeripheral_transmitAudioStreamCmd(uint8_t cmd);
static void AudioPeripheral_startStreaming(void);
static void AudioPeripheral_transmitAudioFrame(uint8_t *buf);
//static void AudioPeripheral_stopStreaming(void);
static void AudioPeripheral_sendStopCmd(void);
static void AudioPeripheral_sendStartCmd(void);
static void AudioPeripheral_startI2Sstream(void);
static void AudioPeripheral_stopI2Sstream(void);
static void AudioPeripheral_finishStream(void);
static void AudioPeripheral_i2sCallbackFxn(I2SCC26XX_Handle handle,
                                            I2SCC26XX_StreamNotification *notification);

/*********************************************************************
 * LOCAL VARIABLES
 */
static int16_t *pcmSamples;
static uint8_t *i2sContMgtBuffer;
static uint8_t audio_encoded[100] = {0};
static sbc_t sbc = {0};
static size_t written = 0;

static streamVars_t streamVariables = {
  STREAM_STATE_IDLE,
  STREAM_STATE_IDLE,
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
  .callbackFxn            = AudioPeripheral_i2sCallbackFxn,
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
 * @fn      AudioPeripheral_init
 *
 * @brief   Called during initialization
 *
 * @param   None.
 *
 * @return  None.
 */
void AudioPeripheral_init(Display_Handle dispHandle, PIN_Handle pinHandle)
{
  // Share handles with application, run in the same task context.
  hDisp = dispHandle;
  hPins = pinHandle;

  // Then initialize I2S driver
  i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
  I2SCC26XX_init(i2sHandle);

  // Initialize TLV320AIC3254 Codec on Audio BP
  AudioCodecOpen();
  // Configure Codec
  AudioCodecConfig(AUDIO_CODEC_TI_3254, AUDIO_CODEC_16_BIT, 16000, 2,
                    OUTPUT_OPTION, INPUT_OPTION);
}

/*********************************************************************
 * @fn      AudioPeripheral_eventHandler
 *
 * @brief   Event handler called by the Simple BLE Peripheral task function.
 *
 * @param   events - Events to process.
 *
 * @return  processedEvents: Bitmask of events actually processed.
 */
uint16_t AudioPeripheral_eventHandler(uint8_t events)
{
  uint16_t processedEvents = 0;
  if (events & AUDIO_I2S_FRAME_EVENT)
  {
    processedEvents |= AUDIO_I2S_FRAME_EVENT;
    if (i2sStreamInProgress) {
      I2SCC26XX_BufferRequest bufferRequest;
      I2SCC26XX_BufferRelease bufferRelease;
      bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN;
      // Indicate that we request buffer from input stream
      bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
      while (gotBuffer) {
        PIN_setOutputValue( hPins, Board_DIO26_ANALOG, 0);

        if (streamVariables.streamType == AUDIO_STREAM_TYPE_MSBC) {
          sbc_encode(&sbc, (int16_t *)bufferRequest.bufferIn,
                      streamVariables.samplesPerFrame * sizeof(int16_t),
                      audio_encoded, MSBC_ENCODED_SIZE, &written);
          //            memcpy(audio_encoded, msbc_data, sizeof(msbc_data));
          audio_encoded[1] = seqNum++;
        }
        else {
          audio_encoded[0] = seqNum++;
          // Send previous PV and SI
          audio_encoded[1] = streamVariables.si;
          audio_encoded[2] = LO_UINT16(streamVariables.pv);
          audio_encoded[3] = HI_UINT16(streamVariables.pv);
          Codec1_encodeBuff((uint8_t *)&audio_encoded[4],
                            (int16_t *)bufferRequest.bufferIn,
                            streamVariables.samplesPerFrame,
                            &streamVariables.si, &streamVariables.pv);
        }
        AudioPeripheral_transmitAudioFrame(audio_encoded);
        PIN_setOutputValue( hPins, Board_DIO26_ANALOG, 1);
        bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
#if LOOP_BACK_AUDIO
        // Loop back audio to test simultaneous input and output
        if (bufferRequest.bufferOut) {
          memcpy(bufferRequest.bufferOut, bufferRequest.bufferIn,
                  streamVariables.samplesPerFrame * sizeof(int16_t));
          bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
        } else {
          bufferRelease.bufferHandleOut = NULL;
        }
#else
        bufferRelease.bufferHandleOut = NULL;
#endif
        I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
        if (volume <= 70) {
          if ((volume & 0x0F) == 0x04) {
            if (streamVariables.streamType == AUDIO_STREAM_TYPE_ADPCM)
            {
              // Volume control
              AudioCodecMicVolCtrl(AUDIO_CODEC_TI_3254, INPUT_OPTION, volume);
            }
#if OUTPUT_OPTION
            AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, OUTPUT_OPTION, volume);
#endif // OUTPUT_OPTION
          }
          volume++;
        }
        bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN;
        // Indicate that we request buffer from input stream
        gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
      }
    }
  }

  if (events & AUDIO_STOP_I2S_EVENT)
  {
    processedEvents |= AUDIO_STOP_I2S_EVENT;
    AudioPeripheral_stopI2Sstream();
  }

  if (events & AUDIO_SEND_STOP_CMD_EVENT)
  {
    processedEvents |= AUDIO_SEND_STOP_CMD_EVENT;
    AudioPeripheral_sendStopCmd();
  }

  if (events &   AUDIO_START_I2S_EVENT)
  {
    processedEvents |= AUDIO_START_I2S_EVENT;
    AudioPeripheral_startI2Sstream();
  }

  if (events & AUDIO_SEND_START_CMD_EVENT)
  {
    processedEvents |= AUDIO_SEND_START_CMD_EVENT;
    AudioPeripheral_sendStartCmd();
  }

  if (events & AUDIO_I2S_ERROR_EVENT)
  {
    processedEvents |= AUDIO_I2S_ERROR_EVENT;
    PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 0);

    Display_print0(hDisp, 4, 0, "I2S Error Event");

    // Move to stop state
    streamVariables.streamState = STREAM_STATE_SEND_STOP_CMD;
    SimpleBLEPeripheral_setEvent(AUDIO_SEND_STOP_CMD_EVENT);
    PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 1);
  }

  return processedEvents;
}

/*********************************************************************
 * @fn      AudioPeripheral_processData
 *
 * @brief   Process incoming audio data
 *
 * @return  None
 */
void AudioPeripheral_processData(AUDIO_DATA_TYPE_E data_type,
                                  genericAudioData_t *pMsg)
{
  static uint16_t prevSeqNum = 0;
  static int numberOfPackets = 0, lostPackets = 0;
  // Check to see if notification is from audio data or control char
  if (data_type == AUDIO_DATA)
  {
    PIN_setOutputValue( hPins, Board_DIO28_ANALOG, 0);
    if (streamVariables.streamState == STREAM_STATE_ACTIVE) {
      numberOfPackets++;
      uint16_t curSeqNum = 0; //pMsg->pValue[0];
      if (streamVariables.streamType == AUDIO_STREAM_TYPE_ADPCM) {
        curSeqNum = pMsg->pValue[0];
      } else {
        curSeqNum = pMsg->pValue[1];
      }
      if (((prevSeqNum + 1) & 0x00FF) != curSeqNum) {
        uint8_t missedFrames = 0;
        if (curSeqNum > prevSeqNum) {
          missedFrames = curSeqNum - prevSeqNum;
        } else {
          missedFrames = (curSeqNum + 256) - prevSeqNum;
        }
        numberOfPackets += missedFrames;
        lostPackets += missedFrames;
        Display_print2(hDisp, 5, 0, "Missing frame, PER %d/%d", lostPackets,
                        numberOfPackets);
      }
      prevSeqNum = curSeqNum;

      I2SCC26XX_BufferRequest bufferRequest;
      I2SCC26XX_BufferRelease bufferRelease;
      bufferRequest.buffersRequested = I2SCC26XX_BUFFER_OUT;
      // Indicate that we request buffer from output stream
      bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
      if (gotBuffer) {
        PIN_setOutputValue( hPins, Board_DIO26_ANALOG, 0);
        if (streamVariables.streamType == AUDIO_STREAM_TYPE_ADPCM)
        {
          // Decode and play back returned audio from Central
          streamVariables.pv_rx = BUILD_UINT16(pMsg->pValue[2], pMsg->pValue[3]);
          streamVariables.si_rx = pMsg->pValue[1];
          Codec1_decodeBuff((int16_t *)bufferRequest.bufferOut,
                            (uint8_t *)&pMsg->pValue[4],
                            streamVariables.samplesPerFrame * sizeof(int16_t),
                            &streamVariables.si_rx, &streamVariables.pv_rx);
        } else {
          sbc_decode(&sbc, pMsg->pValue, MSBC_ENCODED_SIZE,
                      (int16_t *)bufferRequest.bufferOut,
                      streamVariables.samplesPerFrame * sizeof(int16_t),
                      &written);
        }
        PIN_setOutputValue( hPins, Board_DIO26_ANALOG, 1);
        bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
        bufferRelease.bufferHandleIn = NULL;
        I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
      }
    } else {
      Display_print0(hDisp, 5, 0, "Received data outside of stream (ok)");
    }
    PIN_setOutputValue( hPins, Board_DIO28_ANALOG, 1);
  }
}

/*********************************************************************
 * @fn      AudioPeripheral_handleKeys
 *
 * @brief   Handles keys passed from application.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 *
 * @return  none
 */
void AudioPeripheral_handleKeys(uint8_t keys)
{
  static uint8_t previousKeys = 0;

  // Only process changes in keys pressed
  if (keys != previousKeys) {
    // Check for both keys first
    if (keys == (KEY_LEFT | KEY_RIGHT))
    {
      if (streamVariables.streamState != STREAM_STATE_IDLE)
      {
        // Start chain of events to stop stream
        AudioPeripheral_stopStreaming();
      }
    }
    else if (keys & KEY_LEFT)
    {
      if (streamVariables.streamState == STREAM_STATE_IDLE) {
        // Start MSBC stream, from IDLE
        streamVariables.streamType = AUDIO_STREAM_TYPE_MSBC;
        streamVariables.samplesPerFrame = MSBC_SAMPLES_PER_FRAME;
        streamVariables.numOfFramesToBuffer = I2SCC26XX_QUEUE_SIZE;
        streamVariables.rfFrameSize = MSBC_ENCODED_SIZE;
        Audio_SetAudioDataLen(streamVariables.rfFrameSize);
        streamVariables.activeLED = Board_LED2;
        streamVariables.requestedStreamType = AUDIO_STREAM_TYPE_NONE;
        streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
        AudioPeripheral_startStreaming();
      }
      else if (streamVariables.streamType == AUDIO_STREAM_TYPE_ADPCM) {
        // Start chain of events to stop stream
        AudioPeripheral_stopStreaming();
        // Override defaults, change stream to mSBC, from ADPCM
        streamVariables.requestedStreamType = AUDIO_STREAM_TYPE_MSBC;
        streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
      }
      else if (streamVariables.streamType == AUDIO_STREAM_TYPE_MSBC) {
        // Stop mSBC stream
        streamVariables.requestedStreamType = AUDIO_STREAM_TYPE_NONE;
        streamVariables.requestedStreamState = STREAM_STATE_IDLE;
        // Start chain of events to stop stream
        AudioPeripheral_stopStreaming();
      }
    }
    else if (keys & KEY_RIGHT)
    {
      if (streamVariables.streamState == STREAM_STATE_IDLE) {
        // Start ADPCM stream, from IDLE
        streamVariables.streamType = AUDIO_STREAM_TYPE_ADPCM;
        streamVariables.samplesPerFrame = ADPCM_SAMPLES_PER_FRAME;
        streamVariables.numOfFramesToBuffer = I2SCC26XX_QUEUE_SIZE - 2;
        streamVariables.rfFrameSize = BLEAUDIO_MAX_NOTSIZE;
        Audio_SetAudioDataLen(streamVariables.rfFrameSize);
        streamVariables.activeLED = Board_LED1;
        streamVariables.requestedStreamType = AUDIO_STREAM_TYPE_NONE;
        streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
        AudioPeripheral_startStreaming();
      }
      else if (streamVariables.streamType == AUDIO_STREAM_TYPE_MSBC) {
        // Start chain of events to stop stream
        AudioPeripheral_stopStreaming();
        // Override defaults, change stream to ADPCM, from mSBC
        streamVariables.requestedStreamType = AUDIO_STREAM_TYPE_ADPCM;
        streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
      }
      else if (streamVariables.streamType == AUDIO_STREAM_TYPE_ADPCM) {
        // Start chain of events to stop stream
        AudioPeripheral_stopStreaming();
      }
    }
  }
  previousKeys = keys;
}

/*********************************************************************
 * PRIVATE FUNCTIONS
 */

/*********************************************************************
 * @fn      AudioPeripheral_transmitAudioStreamCmd
 *
 * @brief   Transmits GATT Notification in order to start or stop stream
 *
 * @param   cmd - command to transmit
 *
 * @return  SUCCESS if successful, FAILURE if not
 */
static uint8_t AudioPeripheral_transmitAudioStreamCmd(uint8_t cmd)
{
  return Audio_SetParameter(AUDIOPROFILE_START, AUDIOPROFILE_CMD_LEN, &cmd);
}

/*********************************************************************
 * @fn      AudioPeripheral_transmitAudioFrame
 *
 * @brief   Transmits processed audio frame to connected device
 *
 * @param   buf - pointer to PDM buffer
 *
 * @return  None.
 */
static void AudioPeripheral_transmitAudioFrame(uint8_t *buf)
{
  PIN_setOutputValue( hPins, Board_DIO28_ANALOG, 0);
  if (Audio_SetParameter(AUDIOPROFILE_AUDIO, streamVariables.rfFrameSize, buf) == SUCCESS)
  {
    PIN_setOutputValue( hPins, Board_DIO28_ANALOG,
                        !(PIN_getOutputValue(Board_DIO28_ANALOG)));
    PIN_setOutputValue(hPins, streamVariables.activeLED, 1);
  }
  else
  {
    PIN_setOutputValue(hPins, streamVariables.activeLED, 0);
  }
  PIN_setOutputValue( hPins, Board_DIO28_ANALOG, 1);
}

/*********************************************************************
 * @fn      AudioPeripheral_startStreaming
 *
 * @brief   Starts streaming audio to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioPeripheral_startStreaming(void)
{
  // LED on while streaming
  PIN_setOutputValue(hPins, streamVariables.activeLED, 1);

  // Increase TX power during stream
  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);

  I2SCC26XX_Handle i2sHandleTmp = NULL;
  // Allocate memory for decoded PCM data
  i2sParams.ui32conMgtBufTotalSize = I2S_BLOCK_OVERHEAD_IN_BYTES * \
                                      streamVariables.numOfFramesToBuffer * 2;
  i2sParams.ui32conBufTotalSize    = sizeof(int16_t) * \
                                      (streamVariables.samplesPerFrame * \
                                       streamVariables.numOfFramesToBuffer * 2);
  i2sContMgtBuffer = ICall_malloc(i2sParams.ui32conMgtBufTotalSize);
  pcmSamples = ICall_malloc(i2sParams.ui32conBufTotalSize);
  if (pcmSamples) {
    i2sParams.blockSize              = streamVariables.samplesPerFrame;
    i2sParams.pvContBuffer           = (void *) pcmSamples;
    i2sParams.pvContMgtBuffer        = (void *) i2sContMgtBuffer;
    i2sHandleTmp = I2SCC26XX_open(i2sHandle, &i2sParams);
  }
  else {
    Display_print0(hDisp, 5, 0, "Failed to allocate for new stream");
    if (i2sContMgtBuffer) {
      ICall_free(i2sContMgtBuffer);
      i2sContMgtBuffer = NULL;
    }
  }

  if (i2sHandleTmp == i2sHandle) {
    Display_print1(hDisp, 5, 0, "Opened I2S: %d samples/frame",
                    streamVariables.samplesPerFrame);
    // Move to send start command
    streamVariables.streamState = STREAM_STATE_SEND_START_CMD;
    SimpleBLEPeripheral_setEvent(AUDIO_SEND_START_CMD_EVENT);
  }
  else {
    Display_print0(hDisp, 5, 0, "Failed to opened I2S");
    // Return, or move to IDLE state
    streamVariables.streamState = STREAM_STATE_IDLE;
    // Turn LED off again
    PIN_setOutputValue(hPins, streamVariables.activeLED, 0);
  }
}

/*********************************************************************
 * @fn      AudioPeripheral_stopStreaming
 *
 * @brief   Stops streaming audio to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
void AudioPeripheral_stopStreaming(void)
{
  streamVariables.requestedStreamState = STREAM_STATE_IDLE;
  streamVariables.requestedStreamType = AUDIO_STREAM_TYPE_NONE;
  PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 0);
  // Check if we're at the right state in the stopping process
  if (streamVariables.streamState == STREAM_STATE_ACTIVE) {
    // Start by sending STOP command
    streamVariables.streamState = STREAM_STATE_SEND_STOP_CMD;
    Display_print0(hDisp, 4, 0, "Send Stop Command");
    SimpleBLEPeripheral_setEvent(AUDIO_SEND_STOP_CMD_EVENT);
  }
  PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 1);
}

/*********************************************************************
 * @fn      AudioPeripheral_finishStream
 *
 * @brief   Finish stream
 *
 * @param   None.
 *
 * @return  None.
 */
void AudioPeripheral_finishStream(void) {
  // LED off
  PIN_setOutputValue(hPins, streamVariables.activeLED, 0);
  /* Turn output volume back down */
  volume = 0;
  AudioCodecMicVolCtrl(AUDIO_CODEC_TI_3254, INPUT_OPTION, volume);
#if OUTPUT_OPTION
  AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, OUTPUT_OPTION, volume);
#endif // OUTPUT_OPTION

  if (streamVariables.streamType == AUDIO_STREAM_TYPE_MSBC)
  {
    sbc_finish(&sbc);
  }
  /* Close I2S driver */
  I2SCC26XX_close(i2sHandle);
  /* Free memory */
  ICall_free(pcmSamples);
  pcmSamples = NULL;
  ICall_free(i2sContMgtBuffer);
  i2sContMgtBuffer = NULL;
  Display_print0(hDisp, 5, 0, "Closed I2S driver");
}

/*********************************************************************
 * @fn      AudioPeripheral_startI2Sstream
 *
 * @brief   Start I2S stream
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioPeripheral_startI2Sstream(void)
{
  PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 0);
  // Check that we're in the correct state
  if (streamVariables.streamState == STREAM_STATE_START_I2S) {
    if (streamVariables.requestedStreamState == STREAM_STATE_ACTIVE) {
      // It's now safe to update audio data length
      Audio_SetAudioDataLen(streamVariables.rfFrameSize);
      // Try to start I2S stream
      i2sStreamInProgress = I2SCC26XX_startStream(i2sHandle);
      if (i2sStreamInProgress) {
        // Move to ACTIVE as we have completed start sequence
        streamVariables.streamState = STREAM_STATE_ACTIVE;

        if (streamVariables.streamType == AUDIO_STREAM_TYPE_MSBC) {
          Display_print0(hDisp, 5, 0, "mSBC Stream Started");
        }
        else if (streamVariables.streamType == AUDIO_STREAM_TYPE_ADPCM) {
          Display_print0(hDisp, 5, 0, "ADPCM Stream Started");
        }
      }
      else {
        Display_print0(hDisp, 5, 0, "Failed to start I2S stream");
      }
    }
    else {
      Display_print0(hDisp, 5, 0, "Started stream when Active was not requested");
    }
  }
  PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 1);
}

/*********************************************************************
 * @fn      AudioPeripheral_stopI2Sstream
 *
 * @brief   Stop I2S stream
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioPeripheral_stopI2Sstream(void)
{
  // Check that we're in the correct state
  if (streamVariables.streamState == STREAM_STATE_STOP_I2S) {
    // Try to stop I2S stream
    if (I2SCC26XX_stopStream(i2sHandle)) {
      Display_print0(hDisp, 5, 0, "Stopped I2S stream");
      AudioPeripheral_finishStream();
      if (streamVariables.requestedStreamState == STREAM_STATE_IDLE) {
        // Simply move to IDLE as we have completed stop sequence
        streamVariables.streamState = STREAM_STATE_IDLE;
      }
      else if (streamVariables.requestedStreamType == AUDIO_STREAM_TYPE_MSBC) {
        // Start chain of events to start stream again
        streamVariables.streamType = AUDIO_STREAM_TYPE_MSBC;
        streamVariables.samplesPerFrame = MSBC_SAMPLES_PER_FRAME;
        streamVariables.numOfFramesToBuffer = I2SCC26XX_QUEUE_SIZE;
        streamVariables.rfFrameSize = MSBC_ENCODED_SIZE;
        streamVariables.activeLED = Board_LED2;
        streamVariables.requestedStreamType = AUDIO_STREAM_TYPE_NONE;
        streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
        AudioPeripheral_startStreaming();
      }
      else if (streamVariables.requestedStreamType == AUDIO_STREAM_TYPE_ADPCM) {
        streamVariables.streamType = AUDIO_STREAM_TYPE_ADPCM;
        streamVariables.samplesPerFrame = ADPCM_SAMPLES_PER_FRAME;
        streamVariables.numOfFramesToBuffer = I2SCC26XX_QUEUE_SIZE - 2;
        streamVariables.rfFrameSize = BLEAUDIO_MAX_NOTSIZE;
        streamVariables.activeLED = Board_LED1;
        streamVariables.requestedStreamType = AUDIO_STREAM_TYPE_NONE;
        streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
        AudioPeripheral_startStreaming();
      }
      else {
        Display_print2(hDisp, 5, 0, "Incorrect state %d|%d",
                       streamVariables.requestedStreamState,
                       streamVariables.requestedStreamType);
      }
    }
    else {
      Display_print0(hDisp, 5, 0, "Failed to stop I2S stream");
    }
  }
  else {
    Display_print1(hDisp, 5, 0, "Tried to stop I2S stream in state %d",
                    streamVariables.streamState);
  }
}

/*********************************************************************
 * @fn      AudioPeripheral_sendStartCmd
 *
 * @brief   Sends a start command to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioPeripheral_sendStartCmd(void)
{
  // Check that we're in the correct state
  if (streamVariables.streamState == STREAM_STATE_SEND_START_CMD) {
    if (AudioPeripheral_transmitAudioStreamCmd(streamVariables.streamType) == SUCCESS)
    {
      PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 0);
      Display_print0(hDisp, 5, 0, "Sent Start Cmd, initializing encoder");
      if (streamVariables.streamType == AUDIO_STREAM_TYPE_MSBC) {
        // Initialize encoder
        sbc_init_msbc(&sbc, 0);
      }
      else {
        // Initialize encoder
        streamVariables.pv = 0;
        streamVariables.si = 0;
      }
      // Try next state
      streamVariables.streamState = STREAM_STATE_START_I2S;
      SimpleBLEPeripheral_setEvent(AUDIO_START_I2S_EVENT);
      PIN_setOutputValue( hPins, Board_DIO27_ANALOG, 1);
    }
    else {
      // Try again
      SimpleBLEPeripheral_setEvent(AUDIO_SEND_START_CMD_EVENT);
    }
  }
  else {
    // Try next state
    SimpleBLEPeripheral_setEvent(AUDIO_START_I2S_EVENT);
  }

}

/*********************************************************************
 * @fn      AudioPeripheral_sendStopCmd
 *
 * @brief   Sends a stop command to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioPeripheral_sendStopCmd(void)
{
  // Check that we're in the correct state
  if (streamVariables.streamState == STREAM_STATE_SEND_STOP_CMD) {
    uint8_t retVal = SUCCESS;

    if (SimpleBLEPeripheral_isConnected())
    {
      retVal = AudioPeripheral_transmitAudioStreamCmd(BLE_AUDIO_CMD_STOP);
      if (retVal == SUCCESS)
      {
        // Reset TX power
        // Move to stop I2S stream
        HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
        streamVariables.streamState = STREAM_STATE_STOP_I2S;
        SimpleBLEPeripheral_setEvent(AUDIO_STOP_I2S_EVENT);
      }
      else
      {
        Display_print1(hDisp, 5, 0, "Failed to send STOP: %d", retVal);
        // Try again
        SimpleBLEPeripheral_setEvent(AUDIO_SEND_STOP_CMD_EVENT);
      }
    }
    else
    {
      // Reset TX power
      HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
      // Move to stop I2S stream
      streamVariables.streamState = STREAM_STATE_STOP_I2S;
      SimpleBLEPeripheral_setEvent(AUDIO_STOP_I2S_EVENT);
    }
  }
  else {
    // Try next state
    SimpleBLEPeripheral_setEvent(AUDIO_STOP_I2S_EVENT);
  }
}

static void AudioPeripheral_i2sCallbackFxn(I2SCC26XX_Handle handle,
                                           I2SCC26XX_StreamNotification *notification)
{
  if (notification->status == I2SCC26XX_STREAM_ERROR) {
    /* Let thread process PDM error */
    SimpleBLEPeripheral_setEvent(AUDIO_I2S_ERROR_EVENT);
  }
  else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY) {
    // Provide buffer
    SimpleBLEPeripheral_setEvent(AUDIO_I2S_FRAME_EVENT);
    PIN_setOutputValue( hPins, Board_DIO25_ANALOG,
                        !(PIN_getOutputValue(Board_DIO25_ANALOG)));
    PIN_setOutputValue( hPins,
                        (streamVariables.activeLED == Board_LED0) ? Board_LED1 : Board_LED0, 0);
  }
  else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY_BUT_NO_AVAILABLE_BUFFERS) {
    // Provide buffer
    SimpleBLEPeripheral_setEvent(AUDIO_I2S_FRAME_EVENT);
    PIN_setOutputValue( hPins, Board_DIO25_ANALOG,
                        !(PIN_getOutputValue(Board_DIO25_ANALOG)));
    PIN_setOutputValue( hPins, (streamVariables.activeLED == Board_LED0) ? Board_LED1 : Board_LED0, 1);
  }
  else {
    PIN_setOutputValue( hPins,
                        (streamVariables.activeLED == Board_LED0) ? Board_LED1 : Board_LED0, 1);
  }
}

/*********************************************************************
*********************************************************************/
