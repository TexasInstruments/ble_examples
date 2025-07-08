/******************************************************************************

 @file  zcl_dataset.h

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

#ifndef ZIPIR_ZCL_DATASET_H_
#define ZIPIR_ZCL_DATASET_H_

unsigned char TestCodeSet[] =
{
    0x03, // VER
    0x00, 0x00, 0x00, 0xEC, // FSZ = 236 bytes total
    0x05, 0x45, 0x7E, 0x12, // FCD = 88,440,338 seconds since 01/01/12
    0x02, // TC = 2 device types (including final null entry)
    0x54, // DT[0] = 'T'
    0x00, 0x01, // DCC[0] = 1 code set of this type
    0x00, 0x00, // DCSI[0] = record index of this code set
    0x00, // DT[1] = final entry in Device Type Table
    0x00, 0x00, // DCC[0]
    0x00, 0x01, // DCSI[0]
    0x00, 0x02, // CC = 2 code sets (including final delimiter entry)
    0x06, 0xDE, // CID[0] = 1758
    0x00, 0x00, 0x00, 0x00, // CSS[0] = 0-byte offset
    0x00, 0xAB, // FTO[0] = 171-byte offset
    0xFF, 0xFF, // CID[1] = final entry in Code Set Lookup Table
    0x00, 0x00, 0x00, 0xC6, // CSS[1] = 198-byte offset (i.e., end of ZipIR data)
    0x00, 0x00, // FTO[1] (not used; set to 0)



    ////////////////////////////////////////////////////////////////////////////
    // ZCS Code Set data block
    ////////////////////////////////////////////////////////////////////////////

    0x00,// VER = 0, EM = 0
    0xA4, 0x73, // CP = 0xA47 = 2631 ==> 0.02631 ms or 38 kHz; DC = 0x3 ==> 30%
    0x1C, // 0b000111 (00) : NCT = 7 ==> 8 CTs total
    0x8B, // ...00 0b10 (001011) : NLT = 2 ==> 2 CTLs and 6 CTNs
    0xBD, 0x84, // ...001011 0b10111101 0b10 (000100) : CTL[0] = 0x2EF6 = 12022 ==> 96176 us
    0xCD, 0xA3, // ...000100 0b11001101 0b10 (100011) : CTL[1] = 0x1336 = 4918 ==> 39344 us
    0x29, // ...100011 0b001010 (01) : CTN[0] = 0x8CA = 2250 ==> 9000 us
    0x19, 0x48, // ...01 0b00011001 0b01 (001000) : CTN[1] = 0x465 = 1125 ==> 4500 us
    0xCC, // ...001000 0b110011 (00) : CTN[2] = 0x233 = 563 ==> 2252 us
    0x69, 0xC2, // ...00 0b01101001 0b11 (000010) : CTN[3] = 0x1A7 = 423 ==> 1692 us
    0x44, // ...000010 0b010001 (00) : CTN[4] = 0x091 = 145 ==> 580 us
    0x23, 0x4E, // ...00 0b00100011 0b01 (001110) : CTN[5] = 0x08D = 141 ==> 564 us
    // ...001110 : NRS = 14 ==> 15 RSs total
    0x44,// 0b01 (000100) : LRS = 1 ==> SRS size is 4 bits
    // ...0001 (00) : SRS[0] = 1 ==> RS[0] has 2 elements
    0x44,// ...00 0b01 (000100) : SRS[1] = 1 ==> RS[1] has 2 elements
    // ...0001 (00) : SRS[2] = 1 ==> RS[2] has 2 elements
    0x8C,// ...00 0b10 (001100) : SRS[3] = 2 ==> RS[3] has 3 elements
    // ...0011 (00) : SRS[4] = 3 ==> RS[4] has 4 elements
    0xD1,// ...00 0b11 (010001) : SRS[5] = 3 ==> RS[5] has 4 elements
    // ...0100 (01) : SRS[6] = 4 ==> RS[6] has 5 elements
    0xA4,// ...01 0b10 (100100) : SRS[7] = 6 ==> RS[7] has 7 elements
    // ...1001 (00) : SRS[8] = 9 ==> RS[8] has 10 elements
    0xC0,// ...00 0b11 (000000) : SRS[9] = 3 ==> RS[9] has 4 elements
    // ...0000 (00) : SRS[10] = 0 ==> RS[10] has 1 element
    0x00,// ...00 0b00 (000000) : SRS[11] = 0 ==> RS[11] has 1 element
    // ...0000 (00) : SRS[12] = 0 ==> RS[12] has 1 element
    0x05,// ...00 0b00 (000101) : SRS[13] = 0 ==> RS[13] has 1 element
    // ...0001 (01) : SRS[14] = 1 ==> RS[14] has 2 elements
    // 8 CTs total ==> 3 bits per RS[i] element
    0x3F,// RS[0] : ...01 0b0 011 (1111)
    // = {2, 3} (2 elements)
    0xAE,// RS[1] : ...111 1 0b10 (101110)
    // = {7, 6} (2 elements)
    // RS[2] : ...101 110
    // = {5, 6} (2 elements)
    0xFB, 0xFD, // RS[3] : 0b111 110 11 0b1 (1111101)
    // = {7, 6, 7} (3 elements)
    0x75,// RS[4] : ...111 110 1 0b01 110 (101)
    // = {7, 6, 5, 6} (4 elements)
    0xD7, 0x7D, // RS[5] : ...101 0b110 101 11 0b0 (1111101)
    // = {5, 6, 5, 6} (4 elements)
    0xF7,// RS[6] : ...111 110 1 0b11 110 111
    // = {7, 6, 7, 6, 7} (5 elements)
    0xDF, 0x7D, 0x75, // RS[7] : 0b110 111 11 0b0 111 110 1 0b01 110 (101)
    // = {6, 7, 6, 7, 6, 5, 6} (7 elements)
    0xDF, 0x5D, 0x75, 0xCA, // RS[8] : ...101 0b110 111 11 0b0 101 110 1 0b01 110 101 0b110 (01010)
    // = {5, 6, 7, 6, 5, 6, 5, 6, 5, 6} (10 elements)
    0x61,// RS[9] : ...010 10 0b0 110 000 (1)
    // = {2, 4, 6, 0} (4 elements)
    0xF5,// RS[10] : ...1 0b11 (110101)
    // = {3} (1 element)
    // RS[11] : ...110 (101)
    // = {6} (1 element)
    // RS[12] : ...101
    // = {5} (1 element)
    0x3B,// RS[13] : 0b001 (11011)
    // = {1} (1 element)
    0x82,// RS[14] : ...110 11 0b1 (0000010)
    // = {6, 7} (2 elements)
    0x07,// ...0000010 0b000 (00111) : NWS = 16 ==> 17 WSs
    // ...001 (11) : LWS = 1 ==> SWS size is 4 bits
    0xF3,// ...11 0b11 (110011) : SWS[0] = 15 ==> WS[0] has 16 elements
    // ...1100 (11) : SWS[1] = 12 ==> WS[1] has 13 elements
    0xB7,// ...11 0b10 (110111) : SWS[2] = 14 ==> WS[2] has 15 elements
    // ...1101 (11) : SWS[3] = 13 ==> WS[3] has 14 elements
    0xB3,// ...11 0b10 (110011) : SWS[4] = 14 ==> WS[4] has 15 elements
    // ...1100 (11) : SWS[5] = 12 ==> WS[5] has 13 elements
    0x7B,// ...11 0b01 (111011) : SWS[6] = 13 ==> WS[6] has 14 elements
    // ...1110 (11) : SWS[7] = 14 ==> WS[7] has 15 elements
    0xB7,// ...11 0b10 (110111) : SWS[8] = 14 ==> WS[8] has 15 elements
    // ...1101 (11) : SWS[9] = 13 ==> WS[9] has 14 elements
    0xBB,// ...11 0b10 (111011) : SWS[10] = 14 ==> WS[10] has 15 elements
    // ...1110 (11) : SWS[11] = 14 ==> WS[11] has 15 elements
    0xF7,// ...11 0b11 (110111) : SWS[12] = 15 ==> WS[12] has 16 elements
    // ...1101 (11) : SWS[13] = 13 ==> WS[13] has 14 elements
    0x38,// ...11 0b00 (111000) : SWS[14] = 12 ==> WS[14] has 13 elements
    // ...1110 (00) : SWS[15] = 14 ==> WS[15] has 15 elements
    0x01,// ...00 0b00 (000001) : SWS[16] = 0 ==> WS[16] has 1 element
    // 15 RSs total ==> 4 bits per WS[i] element
    0xD9,
    0xE1, 0x54, 0x91, // WS[0] : ...0000 01 0b11 0110 01 0b11 1000 01 0b01 0101 00 0b10 0100 (01)
    0xB9, 0xC5, 0x4B, 0x41, // ...01 0b10 1110 01 0b11 0001 01 0b01 0010 11 0b01 (000001)
    // = {0, 7, 6, 7, 8, 5, 5, 2, 4, 6, 14, 7, 1, 5, 2, 13} (16 elements)
    0xD9, 0xE1, 0x4B, 0x1D, // WS[1] : ...0000 01 0b11 0110 01 0b11 1000 01 0b01 0010 11 0b00 0111 (01)
    0x9E, 0x34, // ...01 0b10 0111 10 0b00 1101 (00)
    // = {0, 7, 6, 7, 8, 5, 2, 12, 7, 6, 7, 8, 13} (13 elements)
    0x1D, 0x9E, 0x15, 0x5A, // WS[2] : ...00 0b00 0111 01 0b10 0111 10 0b00 0101 01 0b01 0110 (10)
    0xD9, 0xD5, 0x4B, 0x41, // ...10 0b11 0110 01 0b11 0101 01 0b01 0010 11 0b01 (000001)
    // = {0, 7, 6, 7, 8, 5, 5, 6, 11, 6, 7, 5, 5, 2, 13} (15 elements)
    0xD9, 0xE1, 0x49, 0x10, // WS[3] : ...0000 01 0b11 0110 01 0b11 1000 01 0b01 0010 01 0b00 0100 (00)
    0xDC, 0x63, 0x41, // ...00 0b11 0111 00 0b01 1000 11 0b01 (000001)
    // = {0, 7, 6, 7, 8, 5, 2, 4, 4, 3, 7, 1, 8, 13} (14 elements)
    0xD9, 0xE1, 0x71, 0xC8, // WS[4] : ...0000 01 0b11 0110 01 0b11 1000 01 0b01 1100 01 0b11 0010 (00)
    0xDF, 0x1D, 0x74, // ...00 0b11 0111 11 0b00 0111 01 0b01 1101 (00)
    // = {0, 7, 6, 7, 8, 5, 12, 7, 2, 3, 7, 12, 7, 5, 13} (15 elements)
    0x1D, 0x9E, 0x17, 0x1D, // WS[5] : ...00 0b00 0111 01 0b10 0111 10 0b00 0101 11 0b00 0111 (01)
    0x9E, 0x0B, 0x41, // ...01 0b10 0111 10 0b00 0010 11 0b01 (000001)
    // = {0, 7, 6, 7, 8, 5, 12, 7, 6, 7, 8, 2, 13} (13 elements)
    0xD9, 0xE1, 0x4A, 0x9C, // WS[6] : ...0000 01 0b11 0110 01 0b11 1000 01 0b01 0010 10 0b10 0111 (00)
    0xDC, 0xA3, 0x41, // ...00 0b11 0111 00 0b10 1000 11 0b01 (000001)
    // = {0, 7, 6, 7, 8, 5, 2, 10, 7, 3, 7, 2, 8, 13} (14 elements)
    0xD9, 0xE1, 0x49, 0xAD, // WS[7] : ...0000 01 0b11 0110 01 0b11 1000 01 0b01 0010 01 0b10 1011 (01)
    0x9D, 0x55, 0x74, // ...01 0b10 0111 01 0b01 0101 01 0b01 1101 (00)
    // = {0, 7, 6, 7, 8, 5, 2, 6, 11, 6, 7, 5, 5, 5, 13} (15 elements)
    0x1D, 0x9E, 0x14, 0x55, // WS[8] : ...00 0b00 0111 01 0b10 0111 10 0b00 0101 00 0b01 0101 (01)
    0x29, 0xCE, 0xE3, 0x41, // ...01 0b00 1010 01 0b11 0011 10 0b11 1000 11 0b01 (000001)
    // = {0, 7, 6, 7, 8, 5, 1, 5, 4, 10, 7, 3, 11, 8, 13} (15 elements)
    0xD9, 0xE1, 0x47, 0x1E, // WS[9] : ...0000 01 0b11 0110 01 0b11 1000 01 0b01 0001 11 0b00 0111 (10)
    0x9D, 0x23, 0x41, // ...10 0b10 0111 01 0b00 1000 11 0b01 (000001)
    // = {0, 7, 6, 7, 8, 5, 1, 12, 7, 10, 7, 4, 8, 13} (14 elements)
    0xD9, 0xE1, 0x51, 0xB9, // WS[10] : ...0000 01 0b11 0110 01 0b11 1000 01 0b01 0100 01 0b10 1110 (01)
    0xC5, 0x55, 0x74, // ...01 0b11 0001 01 0b01 0101 01 0b01 1101 (00)
    // = {0, 7, 6, 7, 8, 5, 4, 6, 14, 7, 1, 5, 5, 5, 13} (15 elements)
    0x1D, 0x9E, 0x0B, 0x1D, // WS[11] : ...00 0b00 0111 01 0b10 0111 10 0b00 0010 11 0b00 0111 (01)
    0x29, 0xC8, 0x63, 0x41, // ...01 0b00 1010 01 0b11 0010 00 0b01 1000 11 0b01 (000001)
    // = {0, 7, 6, 7, 8, 2, 12, 7, 4, 10, 7, 2, 1, 8, 13} (15 elements)
    0xD9, 0xE1, 0x69, 0xCA, // WS[12] : ...0000 01 0b11 0110 01 0b11 1000 01 0b01 1010 01 0b11 0010 (10)
    0x9C, 0xB1, 0xD7, 0x41, // ...10 0b10 0111 00 0b10 1100 01 0b11 0101 11 0b01 (000001)
    // = {0, 7, 6, 7, 8, 5, 10, 7, 2, 10, 7, 2, 12, 7, 5, 13} (16 elements)
    0xD9, 0xE1, 0x69, 0xCD, // WS[13] : ...0000 01 0b11 0110 01 0b11 1000 01 0b01 1010 01 0b11 0011 (01)
    0xCA, 0x0B, 0x41, // ...01 0b11 0010 10 0b00 0010 11 0b01 (000001)
    // = {0, 7, 6, 7, 8, 5, 10, 7, 3, 7, 2, 8, 2, 13} (14 elements)
    0xD9, 0xE1, 0x4D, 0xE9, // WS[14] : ...0000 01 0b11 0110 01 0b11 1000 01 0b01 0011 01 0b11 1010 (01)
    0xD6, 0x34, // ...01 0b11 0101 10 0b00 1101 (00)
    // = {0, 7, 6, 7, 8, 5, 3, 7, 10, 7, 5, 8, 13} (13 elements)
    0x1D, 0x9E, 0x15, 0xAD, // WS[15] : ...00 0b00 0111 01 0b10 0111 10 0b00 0101 01 0b10 1011 (01)
    0x9D, 0x55, 0x4B, 0x64, // ...01 0b10 0111 01 0b01 0101 01 0b01 0010 11 0b01 (100100)
    // = {0, 7, 6, 7, 8, 5, 6, 11, 6, 7, 5, 5, 5, 2, 13} (15 elements)
    // WS[16] : ...1001 (00)
    // = {9} (1 element)


    ////////////////////////////////////////////////////////////////////////////
    // ZFT Function Table data block
    ////////////////////////////////////////////////////////////////////////////

    0x50,// BWS = 5 bits per WS; SQC = 0 ==> 1 Transmission Sequence
    0xCE, // 0b110 (01110) : WSP[0] = 0b110 ==> WSM and WSR are present
    // ...01110 : WSM[0] = 14
    0x86,// 0b10000 (110) : WSR[0] = 16
    // ...110 : WSP[1] = 0b110 ==> WSM and WSR are present
    0x34,// 0b00110 (100) : WSM[1] = 6
    0x32, // ...100 0b00 (110010) : WSR[1] = 16
    // ...110 (010) : WSP[2] = 0b110 ==> WSM and WSR are present
    0x61,// ...010 0b01 (100001) : WSM[2] = 9
    // ...10000 (1) : WSR[2] = 16
    0x83,// ...1 0b10 (000011) : WSP[3] = 0b110 ==> WSM and WSR are present
    // ...00001 (1) : WSM[3] = 1
    0x0C,// ...1 0b0000 (1100) : WSR[3] = 16
    // ...110 (0) : WSP[4] = 0b110 ==> WSM and WSR are present
    0xB8,// ...0 0b1011 (1000) : WSM[4] = 11
    0x61, // ...1000 0b0 (1100001) : WSR[4] = 16
    // ...110 (0001) : WSP[5] = 0b110 ==> WSM and WSR are present
    0xC3,// ...0001 0b1 (1000011) : WSM[5] = 3
    // ...10000 (11) : WSR[5] = 16
    0x22,// ...11 0b0 (0100010) : WSP[6] = 0b110 ==> WSM and WSR are present
    // ...01000 (10) : WSM[6] = 8
    0x18,// ...10 0b000 (11000) : WSR[6] = 16
    // ...110 (00) : WSP[7] = 0b110 ==> WSM and WSR are present
    0x10,// ...00 0b000 (10000) : WSM[7] = 0
    // ...10000 : WSR[7] = 16
    0xCC,// 0b110 (01100) : WSP[8] = 0b110 ==> WSM and WSR are present
    // ...01100 : WSM[8] = 12
    0x86,// 0b10000 (110) : WSR[8] = 16
    // ...110 : WSP[9] = 0b110 ==> WSM and WSR are present
    0x24,// 0b00100 (100) : WSM[9] = 4
    0x32, // ...100 0b00 (110010) : WSR[9] = 16
    // ...110 (010) : WSP[10] = 0b110 ==> WSM and WSR are present
    0xA1,// ...010 0b10 (100001) : WSM[10] = 10
    // ...10000 (1) : WSR[10] = 16
    0x85,// ...1 0b10 (000101) : WSP[11] = 0b110 ==> WSM and WSR are present
    // ...00010 (1) : WSM[11] = 2
    0x0C,// ...1 0b0000 (1100) : WSR[11] = 16
    // ...110 (0) : WSP[12] = 0b110 ==> WSM and WSR are present
    0x58,// ...0 0b0101 (1000) : WSM[12] = 5
    0x67, // ...1000 0b0 (1100111) : WSR[12] = 16
    // ...110 (0111) : WSP[13] = 0b110 ==> WSM and WSR are present
    0xC3,// ...0111 0b1 (1000011) : WSM[13] = 15
    // ...10000 (11) : WSR[13] = 16
    0x1E,// ...11 0b0 (0011110) : WSP[14] = 0b110 ==> WSM and WSR are present
    // ...00111 (10) : WSM[14] = 7
    0x19,// ...10 0b000 (11001) : WSR[14] = 16
    // ...110 (01) : WSP[15] = 0b110 ==> WSM and WSR are present
    0xB0 // ...01 0b101 (10000) : WSM[15] = 13
    // ...10000 : WSR[15] = 16
};

#endif /* ZIPIR_ZCL_DATASET_H_ */
