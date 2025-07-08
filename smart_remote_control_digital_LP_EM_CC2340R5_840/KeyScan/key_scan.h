/******************************************************************************

@file  key_scan.h

@brief This file contains the application main functionality

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2024, Texas Instruments Incorporated
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

#include <stdint.h>
#include <stdbool.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include "ti_drivers_config.h"

// Define keypad configuration
#define NUM_ROWS 4
#define NUM_COLS 5

#define CONFIG_KEY_COL_1                      10
#define CONFIG_KEY_COL_2                      9
#define CONFIG_KEY_COL_3                      21
#define CONFIG_KEY_COL_4                      23
#define CONFIG_KEY_COL_5                      25

#define CONFIG_KEY_ROW_1                      0
#define CONFIG_KEY_ROW_2                      24
#define CONFIG_KEY_ROW_3                      7
#define CONFIG_KEY_ROW_4                      8


// Define keypad key structure
typedef struct {
    uint8_t row;
    uint8_t col;
} KeypadKey;

// Define key pressed structure
typedef struct {
    KeypadKey pressedKeys[NUM_ROWS * NUM_COLS];
    uint8_t key_count;
} PressedKeys;

// Define keypad configuration structure
typedef struct {
    uint8_t rowPins[NUM_ROWS];
    uint8_t colPins[NUM_COLS];
    uint32_t debounceDelay_us;
    uint8_t max_pressed_keys;
} KeypadConfig;

// Initialize keypad driver
void Keypad_init(const KeypadConfig *config);

// Read the currently pressed key, returns true if a key is pressed
uint8_t Keypad_read(PressedKeys *keys, const KeypadConfig *config);
