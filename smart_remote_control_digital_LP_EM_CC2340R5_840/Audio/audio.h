/******************************************************************************

@file  audio.h

@brief This file contains both the AMIC and DMIC main functionalities.

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2025, Texas Instruments Incorporated
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

 Copyright (c) 2022-2025, Texas Instruments Incorporated
 All rights reserved.

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 ******************************************************************************
 *****************************************************************************/

#ifndef APPLICATION_AUDIO_H_
#define APPLICATION_AUDIO_H_

#include <stdint.h>

/* Event to be used for interfacing */
#define AUDIO_DATA_READY_EVENT 0x10

/* Error codes */
#define AUDIO_SUCCESS           1
#define AUDIO_FAIL_OPENING      255
#define AUDIO_FAIL_NOT_OPENED   254
#define AUDIO_FAIL_NO_DATA      253

/* Buffer sizes (depends on Codec used) */
#ifdef USE_ADPCM_CODEC
#define ADPCM_ENC_FRAME_SIZE               (16U)
#define ADPCM_PCM_FRAME_SIZE               (32U)
#define AUDIO_PCM_FRAME_SIZE          ADPCM_PCM_FRAME_SIZE
#define AUDIO_ENC_FRAME_SIZE          ADPCM_ENC_FRAME_SIZE
#else
#ifdef USE_MSBC_CODEC
    #ifdef USE_BW_24KHz
    //#define MSBC_ENC_FRAME_SIZE               (171U)
    //#define MSBC_PCM_FRAME_SIZE               (360U)
    #define MSBC_ENC_FRAME_SIZE               (57U)
    #define MSBC_PCM_FRAME_SIZE               (120U)
    #else
    #define MSBC_ENC_FRAME_SIZE               (57U)
    #define MSBC_PCM_FRAME_SIZE               (120U)
    #endif
#define AUDIO_PCM_FRAME_SIZE          MSBC_PCM_FRAME_SIZE
#define AUDIO_ENC_FRAME_SIZE          MSBC_ENC_FRAME_SIZE
#endif
#endif

extern uint8_t Audio_init(void);
extern uint8_t Audio_enable(void);
extern uint8_t Audio_disable(void);
extern uint8_t Audio_getCompressedBuffer(uint8_t* encBuf);

#endif /* APPLICATION_AUDIO_H_ */
