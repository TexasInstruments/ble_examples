/******************************************************************************

@file  key_scan.c

@brief This file contains the application main functionality

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

#include "KeyScan/key_scan.h"

// Key state matrix
static uint8_t keyState[NUM_ROWS][NUM_COLS] = {0};

// Function prototypes
extern void keypadTask(char *key_action_type);
static void keys_press_callback(uint_least8_t index, const KeypadConfig *config);
static uint8_t counter = 0;

void Keypad_init(const KeypadConfig *config) {

    // Initialize GPIO
    GPIO_init();

    // Configure keypad rows as inputs with pull-down resistor and interrupts enabled
    for (int i = 0; i < NUM_ROWS; i++) {
        GPIO_setConfig(config->rowPins[i], GPIO_CFG_IN_PD | GPIO_CFG_HYSTERESIS_ON | GPIO_CFG_INT_BOTH_EDGES_INTERNAL | GPIO_CFG_INT_ENABLE);
        // Set row pins to same callback
        GPIO_setCallback(config->rowPins[i], keys_press_callback);
    }

    // Configure keypad columns as outputs
    for (int i = 0; i < NUM_COLS; i++) {
        GPIO_setConfig(config->colPins[i], GPIO_CFG_OUT_STD);
        // Set columns to high
        GPIO_write(config->colPins[i], 1);
    }
}

uint8_t Keypad_read(PressedKeys *keys, const KeypadConfig *config) {

    // Read the currently pressed key
    uint8_t keyPressed = 0;
    // Count keys pressed at the same time.
    uint8_t numKeysPressed = 0;
    // Set all columns to low
    CPUDelay(1000000);
    for (int col = 0; col < NUM_COLS; col++) {
         GPIO_write(config->colPins[col], 0);
    }

    // Read state of each key and update keyState array
    for (int col = 0; col < NUM_COLS; col++) {
        // Drive one column high at a time
        GPIO_write(config->colPins[col], 1);
        // Read the row pins
        for (int row = 0; row < NUM_ROWS; row++) {
             uint8_t currState = GPIO_read(config->rowPins[row]);
             // Implement de-bouncing logic
             // Only update keyState if the state remains stable for a certain period
             if (currState == keyState[row][col])
             {
                 keyState[row][col] = currState;
             }
             else
             {
                 // Delay to filter out bouncing signals
                 //CPUDelay(200000);
                 if (currState == GPIO_read(config->rowPins[row])) {
                     keyState[row][col] = currState;
                  }
              }
         }
         // Set column back to low
         GPIO_write(config->colPins[col], 0);
    }

    // Set all columns to high
    for (int col = 0; col < NUM_COLS; col++) {
         GPIO_write(config->colPins[col], 1);
    }

    // Check for key presses
    for (int row = 0; row < NUM_ROWS; row++) {
        for (int col = 0; col < NUM_COLS; col++) {
            if (keyState[row][col] == 1) {
                // Key is pressed
                keys->pressedKeys[numKeysPressed].row = row;
                keys->pressedKeys[numKeysPressed].col = col;
                keyPressed = 1;
                numKeysPressed++;
                keyState[row][col] = 0; // Clear the key press
                if (numKeysPressed > config->max_pressed_keys)
                {
                    // How many keys are pressed at the same time.
                    keys->key_count = numKeysPressed;
                    keyPressed = 3;
                    return keyPressed;
                }
            }
        }
    }

    // How many keys pressed at the same time.
    keys->key_count = numKeysPressed;
    return keyPressed;
}

static void keys_press_callback(uint_least8_t index, const KeypadConfig *config) {

        char *key_action_type = ICall_malloc(sizeof(char));

        // Disable interrupts
        GPIO_disableInt(CONFIG_KEY_ROW_1);
        GPIO_disableInt(CONFIG_KEY_ROW_2);
        GPIO_disableInt(CONFIG_KEY_ROW_3);
        GPIO_disableInt(CONFIG_KEY_ROW_4);

        if (GPIO_read(CONFIG_KEY_ROW_1) || GPIO_read(CONFIG_KEY_ROW_2) || GPIO_read(CONFIG_KEY_ROW_3) || GPIO_read(CONFIG_KEY_ROW_4)){
            key_action_type[0] = 1;
            counter++;
        }
        else {
            key_action_type[0] = 0;
            counter = 0;
        }

        if (counter > 1){
            key_action_type[0] = 0;
        }

        // Signal the semaphore
        BLEAppUtil_invokeFunction(keypadTask, key_action_type);
}
