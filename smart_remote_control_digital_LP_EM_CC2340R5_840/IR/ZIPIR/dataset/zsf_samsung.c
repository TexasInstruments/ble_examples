/******************************************************************************

 @file  zsf_samsung.c

 @brief This file contains the ZIPIR Dataset.

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

const unsigned char ZIPIR_T2051_voldn[] =
{
0x82 ,0x01 ,0x22 ,0x00 ,0xA4 ,0x73 ,0x10 ,0x45 ,0xA0 ,0x91 ,0x94 ,0x69 ,0xC2 ,0x44 ,0x23 ,0x46 ,0x47 ,0x04 ,0x04 ,0x05 ,0xA6 ,0x9C ,0x71 ,0xC7 ,0x1C ,0x6C ,0x47 ,0x11 ,0x20 ,0x02 ,0x96 ,0x00 ,0xD0 ,0x68 ,0x85 ,0x88 ,0x00 ,0x08 ,0x10 ,0x40
};

const unsigned char ZIPIR_T2051_volup[] =
{
0x82 ,0x01 ,0x1F ,0x00 ,0xa4 ,0x73 ,0x10 ,0x45 ,0xa0 ,0x91 ,0x94 ,0x69 ,0xc2 ,0x44 ,0x23 ,0x44 ,0x44 ,0x7c ,0x45 ,0xa7 ,0x1a ,0x69 ,0xa7 ,0x1c ,0x71 ,0xc7 ,0x18 ,0x24 ,0x00 ,0x39 ,0x12 ,0x44 ,0x90 ,0x00 ,0x0c ,0x10 ,0x40
};


const unsigned char ZIPIR_T2051_power[] =
{
0x82 ,0x01 ,0x2C ,0x00 ,0xA4 ,0x73 ,0x10 ,0x45 ,0xA0 ,0x91 ,0x94 ,0x69 ,0xC2 ,0x44 ,0x23 ,0x46 ,0x44 ,0x48 ,0xD3 ,0x00 ,0x96 ,0x13 ,0x4D ,0x35 ,0x1A ,0x71 ,0xA7 ,0x1C ,0x71 ,0xC7 ,0x1C ,0x00 ,0x52 ,0xEE ,0x1D ,0x57 ,0x58 ,0xDB ,0x21 ,0xD5 ,0x75 ,0x8D ,0xB2 ,0x1D ,0x57 ,0x58 ,0xDB ,0x20 ,0x10 ,0xD0
};

const unsigned char ZIPIR_T2051_mute[] =
{
0x82 ,0x01 ,0x1F ,0x00 ,0xA4 ,0x73 ,0x10 ,0x45 ,0xA0 ,0x91 ,0x94 ,0x69 ,0xC2 ,0x44 ,0x23 ,0x44 ,0x44 ,0x7C ,0x45 ,0xA7 ,0x1A ,0x69 ,0xA7 ,0x1C ,0x71 ,0xC7 ,0x18 ,0x24 ,0x00 ,0x39 ,0x12 ,0x08 ,0x92 ,0x00 ,0x0C ,0x10 ,0x40
};

