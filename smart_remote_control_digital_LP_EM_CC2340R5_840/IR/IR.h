/******************************************************************************

@file  IR.h

@brief This file contains the Infrared main functions.

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-20254, Texas Instruments Incorporated
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

#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>
#include "ti_drivers_config.h"
#include <ti/devices/cc23x0r5/inc/hw_evtsvt.h>
#include <ti/devices/cc23x0r5/inc/hw_lgpt.h>
#include <ti/devices/cc23x0r5/inc/hw_clkctl.h>
#include <ti/devices/cc23x0r5/inc/hw_ioc.h>
#include <ti/devices/cc23x0r5/driverlib/interrupt.h>

#ifdef USE_ZIPIR
#include "ZIPIR/ZipIR_API.h"
#include "ZIPIR/ZipIRDecoder.h"
#include "ZIPIR/dataset/zsf_samsung.h"
#else
#include "NEC/IR_PDE_LPF3_REG.h"
#endif

void IRNec_start();
#define IRTX_PIN    11

//*********************** Android Remote Control IR Key Definition **************************//

#define IR_KEYCODE_BOOKMARK                0x74
#define IR_KEYCODE_ALL_APPS                0x57
#define IR_KEYCODE_PROFILE_SWITCH          0x59
#define IR_KEYCODE_ASSIST                  0x46
#define IR_KEYCODE_SETTINGS                0x0F
#define IR_KEYCODE_NOTIFICATION            0x10
#define IR_KEYCODE_DPAD_UP                 0x15
#define IR_KEYCODE_DAPD_DOWN               0x16
#define IR_KEYCODE_DPAD_LEFT               0x17
#define IR_KEYCODE_DPAD_RIGHT              0x18
#define IR_KEYCODE_DPAD_CENTER             0x19
#define IR_KEYCODE_BACK                    0x48
#define IR_KEYCODE_HOME                    0x47
#define IR_KEYCODE_GUIDE                   0x32
#define IR_KEYCODE_TV                      0x61
#define IR_KEYCODE_POWER                   0x21
#define IR_KEYCODE_TV_INPUT                0x60
#define IR_KEYCODE_VOLUME_MUTE             0x25
#define IR_KEYCODE_VOLUME_UP               0x23
#define IR_KEYCODE_VOLUME_DOWN             0x24
#define IR_KEYCODE_F17                     0x63
#define IR_KEYCODE_F18                     0x64
#define IR_KEYCODE_F19                     0x67
#define IR_KEYCODE_F20                     0x68
#define IR_KEYCODE_CHANNEL_UP              0x33
#define IR_KEYCODE_CHANNEL_DOWN            0x34
#define IR_KEYCODE_MEDIA_SKIP_BACKWARD     0x51
#define IR_KEYCODE_MEDIA_RECORD            0x54
#define IR_KEYCODE_MEDIA_PLAY_PAUSE        0x52
#define IR_KEYCODE_MEDIA_SKIP_FORWARD      0x53
#define IR_KEYCODE_PROG_RED                0x4B
#define IR_KEYCODE_PROG_GREEN              0x4A
#define IR_KEYCODE_PROG_YELLOW             0x49
#define IR_KEYCODE_PROG_BLUE               0x4C
#define IR_KEYCODE_1                       0x01
#define IR_KEYCODE_2                       0x02
#define IR_KEYCODE_3                       0x03
#define IR_KEYCODE_4                       0x04
#define IR_KEYCODE_5                       0x05
#define IR_KEYCODE_6                       0x06
#define IR_KEYCODE_7                       0x07
#define IR_KEYCODE_8                       0x08
#define IR_KEYCODE_9                       0x09
#define IR_KEYCODE_0                       0x0A
#define IR_KEYCODE_INFO                    0x29
#define IR_KEYCODE_PERIOD                  0x2A
#define IR_KEYCODE_CAPTIONS                0x58
#define IR_KEYCODE_TV_TELETEXT             0x75
