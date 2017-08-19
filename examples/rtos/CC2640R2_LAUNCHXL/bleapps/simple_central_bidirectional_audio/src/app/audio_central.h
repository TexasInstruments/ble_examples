/*
 * Filename: audio_central.h
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

#ifndef AUDIO_CENTRAL_H
#define AUDIO_CENTRAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <ti/display/Display.h>
#include "board_key.h"

#include "board.h"

#include <ti/blestack/profiles/audio_dle/audio_profile_dle.h>
#include <ti/drivers/pdm/Codec1.h>

#include <ti/blestack/audiocodec/audiocodec.h>
#include <third_party/sbc/include/msbc_library.h>
#include <ti/drivers/i2s/I2SCC26XX.h>

/*********************************************************************
 * CONSTANTS
 */
// Events used in SimplePeripheral
#define AUDIO_I2S_FRAME_EVENT                 0x01
#define AUDIO_I2S_ERROR_EVENT                 0x02
#define AUDIO_SEND_STOP_CMD_EVENT             0x04
#define AUDIO_SEND_START_CMD_EVENT            0x08
#define AUDIO_STOP_I2S_EVENT                  0x10
#define AUDIO_START_I2S_EVENT                 0x20

#define BLE_AUDIO_CMD_STOP                    0x00
#define BLE_AUDIO_CMD_START                   0x04
#define BLE_AUDIO_CMD_START_MSBC              0x05
#define BLE_AUDIO_CMD_NONE                    0xFF

#define AUDIO_STREAM_TYPE_NONE                BLE_AUDIO_CMD_STOP
#define AUDIO_STREAM_TYPE_ADPCM               BLE_AUDIO_CMD_START
#define AUDIO_STREAM_TYPE_MSBC                BLE_AUDIO_CMD_START_MSBC

#define BLE_AUDIO_CMD_VOLUME_UP               0x01
#define BLE_AUDIO_CMD_VOLUME_DOWN             0xFF

#define INPUT_OPTION                          AUDIO_CODEC_MIC_ONBOARD //AUDIO_CODEC_MIC_LINE_IN //
#define OUTPUT_OPTION                         AUDIO_CODEC_SPEAKER_HP
#define BLEAUDIO_BUFSIZE_ADPCM                96
#define BLEAUDIO_HDRSIZE_ADPCM                4

#define ADPCM_SAMPLES_PER_FRAME               (BLEAUDIO_BUFSIZE_ADPCM * 2)
#define MSBC_SAMPLES_PER_FRAME                120
#define MSBC_ENCODED_SIZE                     57

/*********************************************************************
 * TYPEDEFS
 */
typedef enum {
  STREAM_STATE_IDLE,
  STREAM_STATE_SEND_START_CMD,
  STREAM_STATE_START_I2S,
  STREAM_STATE_ACTIVE,
  STREAM_STATE_SEND_STOP_CMD,
  STREAM_STATE_STOP_I2S,
} STREAM_STATE_E;

/**
 * General data struct.
 */
typedef struct
{
  uint16 len;    //!< Length of data
  uint8 *pValue; //!< Data
} genericAudioData_t;

typedef struct {
  uint8_t streamType;
  uint8_t samplesPerFrame;
  uint8_t notificationsPerFrame;
  uint8_t numOfFramesToBuffer;
  uint8_t rfFrameSize;
  int8_t si; //Step Index
  int16_t pv; //Predict Value
  int8_t si_rx;
  int16_t pv_rx;
  int8_t maxVolume;
  uint8_t activeLED;
  uint8_t i2sOpened;
} streamVars_t;

/**
 * This enum is used to identify the data passed on from _processGATTMsg
 */
typedef enum {
  AUDIO_DATA_START_STOP,
  AUDIO_DATA
} AUDIO_DATA_TYPE_E;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
extern void AudioCentral_init(Display_Handle displayHandle, PIN_Handle pinHandle);
extern uint16_t AudioCentral_eventHandler(uint8_t events);
extern void AudioCentral_processData(AUDIO_DATA_TYPE_E data_type, genericAudioData_t *pMsg);
extern void AudioCentral_handleKeys(uint8_t keys);
extern void AudioCentral_stopStreaming(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_CENTRAL_H */
