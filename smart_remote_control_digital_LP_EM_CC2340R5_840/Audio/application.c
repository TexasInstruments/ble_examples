/******************************************************************************

@file  application.c

@brief This file contains the audio application main functionality

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
#include <string.h>
#include <stdarg.h>
#include <FreeRTOS.h>
#include <task.h>
#include "util.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
#include <ti_drivers_config.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include "app_main.h"
#include "audio.h"

// The selection of the number of encoded buffers per Bluetooth packet
// should be selected considering the payload size, the lengh of
// each encoded buffer, the connection interval, etc.
// The sizes suggested here are for ATT MTU=251 and CI=7.5 ms
// Larger values increase RAM consumption and delay.
// Smaller values make the system more likely to drop data during RF transmission.
#ifdef USE_ADPCM_CODEC
#define NUM_OF_ENC_BUFFERS_PER_BLE_PACKET    5 // max is 15
#else
#define NUM_OF_ENC_BUFFERS_PER_BLE_PACKET    3 // max is 4
#endif

uint8_t audio_encBuf[NUM_OF_ENC_BUFFERS_PER_BLE_PACKET][AUDIO_ENC_FRAME_SIZE];

uint8_t bleStreamIsEnabled = 0;
uint8_t noDataInstances = 0;
uint8_t dataInstances = 0;

void Application_onSystemInitialized(void);
void Application_onBLEConnectionEstablished(void);
void Application_onBLEConnectionTerminated(void);
void Application_onBLEStreamEnabled(void);
void Application_onBLEStreamDisabled(void);
extern void Audio_sendBufferOverBLE();

/*********************************************************************
 * @fn      Application_onSystemInitialized
 *
 * @brief   Called when the Bluetooth stack is initialized.
 *
 * @param   None
 *
 * @return  None.
 */
void Application_onSystemInitialized(void)
{
    uint8_t audioStatus;

    audioStatus = Audio_init();
    if(AUDIO_SUCCESS != audioStatus)
    {
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "Error: Audio_init() failed");

        return;
    }
    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "Info: Audio_init() success");
    return;
}

/*********************************************************************
 * @fn      Application_onBLEConnectionEstablished
 *
 * @brief   Called when a Bluetooth connection has been established
 *          with a peer device.
 *
 * @param   None
 *
 * @return  None.
 */
void Application_onBLEConnectionEstablished(void)
{

}

/*********************************************************************
 * @fn      Application_onBLEConnectionTerminated
 *
 * @brief   Called when the Bluetooth connection has been terminated.
 *
 * @param   None
 *
 * @return  None.
 */

void Application_onBLEStreamEnabled(void)
{
    bleStreamIsEnabled = 1;
}

void Application_onBLEStreamDisabled(void)
{
    bleStreamIsEnabled = 0;
}

void Application_DoAudioEvent()
{

    static int encBufIndex = 0;

    uint16_t dataLength = Audio_getCompressedBuffer(audio_encBuf[encBufIndex]);

    if(AUDIO_FAIL_NO_DATA == dataLength)
    {
        noDataInstances++;
    }
    else
    {
        if(0 != dataLength)
        {
            encBufIndex++;
            dataInstances++;
        }
    }
    // When we have enough ADPCM data, send the packet
    if(encBufIndex == NUM_OF_ENC_BUFFERS_PER_BLE_PACKET)
    {
        Audio_sendBufferOverBLE(audio_encBuf, sizeof(audio_encBuf), 1);
        encBufIndex = 0;
    }

}
