/******************************************************************************

 @file  zsf_dataset.h

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

#ifndef ZIPIR_ZSF_DATASET_H_
#define ZIPIR_ZSF_DATASET_H_

unsigned char zsf[] =
{
    /////////////////////////////////////////////////////////////////////////
    // ZSF header block
    /////////////////////////////////////////////////////////////////////////

    0x82, // ZSF file ID
    0x00, // Device ID
    0x1e, // Size of <ZCS> - 1



    /////////////////////////////////////////////////////////////////////////
    // ZCS Code Set data block
    /////////////////////////////////////////////////////////////////////////

    0x00, // 0b0000 (0000) : VER = 0
    // ...0000 : EM = 0
    0x9C, 0x43, // 0b10011100 0b0100 (0011): CP = 0x9C4 = 2500 ==> 0.02500 ms or 40 kHz
    // ...0011 : DC = 0x3 ==> 30%
    0x0C, // 0b000011 (00) : NCT = 3 ==> 4 CTs total
    0x43, // ...00 0b01 (000011) : NLT = 1 ==> 1 CTL and 3 CTNs
    0x4B, 0xC9, // ...000011 0b01001011 0b11 (001001) : CTL[0] = 0x0D2F = 3375 ==> 27000 us
    0x60, // ...001001 0b011000 (00) : CTN[0] = 0x258 = 600 ==> 2400 us
    0x4B, 0x02, // ...00 0b01001011 0b00 (000010) : CTN[1] = 0x12C = 300 ==> 1200 us
    0x58, // ...000010 0b010110 (00) : CTN[2] = 0x096 = 150 ==> 600 us
    0x60, // ...00 0b0110 (000: NRS = 6 ==> 7 RSs total
    // ...00 (00) : LRS = 0 ==> SRS size is 3 bits
    0x48, // ...00 0b0 (1001000) : SRS[0] = 0 ==> RS[0] has 1 element
    // ...100 (1000) : SRS[1] = 4 ==> RS[1] has 5 elements
    // ...100 (0) : SRS[2] = 4 ==> RS[2] has 5 elements
    0x80, // ...0 0b10 (000000) : SRS[3] = 2 ==> RS[3] has 3 elements
    // ...000 (000) : SRS[4] = 0 ==> RS[4] has 1 element
    // ...000 : SRS[5] = 0 ==> RS[5] has 1 element
    0x2F, // 0b001 (01111) : SRS[6] = 1 ==> RS[6] has 2 elements
    // 4 CTs total ==> 2 bits per RS[i] element
    // RS[0] : ...01 (111)
    // = {1} (1 element)
    0x7F, // RS[1] : ...11 1 0b0 11 11 11 (1)
    // = {3, 2, 3, 3, 3} (5 elements)
    0xFF, 0xFF, // RS[2] : ...1 0b1 11 11 11 1 0b1 (1111111)
    // = {3, 3, 3, 3, 3} (5 elements)
    // RS[3] : ...11 11 11 (1)
    // = {3, 3, 3} (3 elements)
    0x82, // RS[4] : ...1 0b1 (0000010)
    // = {3} (1 element)
    // RS[5] : ...00 (00010)
    // = {0} (1 element)
    // RS[6] : ...00 01 (0)
    // = {0, 1} (2 elements)
    0x00, 0x97, // ...0 0b00000000 1: NWS = 1 ==> 2 WSs total
    // ...001 (0111) : LWS = 1 ==> SWS size is 4 bits
    // ...0111 : SWS[0] = 7 ==> WS[0] has 8 elements
    0xE0, // 0b1110 (0000) : SWS[1] = 14 ==> WS[1] has 15 elements
    // 7 RSs total ==> 3 bits per WS[i] element
    0x53, 0x85, 0x50, // WS[0] : ...000 0 0b01 010 011 0b100 001 01 0b0 101 (0000)
    // = {0, 1, 2, 3, 4, 1, 2, 5} (8 elements)
    0x53, 0x85, 0x62, 0x9C, // WS[1] : ...000 0 0b01 010 011 0b100 001 01 0b0 110 001 0 0b10 011 100
    0x2A, 0x80, // 0b001 010 10 0b1 (0000000)
    // = {0, 1, 2, 3, 4, 1, 2, 6, 1, 2, 3, 4, 1, 2, 5} (15 elements)



    //////////////////////////////////////////////////////////////////////////////
    // ZFT Function Table data block
    //////////////////////////////////////////////////////////////////////////////

    0x10, // 0b0001 (0000) : BWS = 1 bits per WS
    // ...0000 : SQC = 0 ==> 1 Transmission Sequence
    0xD0 // 0b110 (10000) : WSP[0][0] = 0b110 ==> WSM and WSR are present
    // ...1 (0000) : WSM[0][0] ==> WS[1]
    // ...0 (000) : WSR[0][0] ==> WS[0]
};




#endif /* ZIPIR_ZSF_DATASET_H_ */
