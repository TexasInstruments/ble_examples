/******************************************************************************

@file  NVS_ext.c

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

#include <NVSExt/nvs_ext.h>

NVS_Handle nvsRegion;
NVS_Params nvsParams;
NVS_Attrs regionAttrs;

char buf_read[16];
char but_write[] = "Hellou";

void NVSExt_start(void){

    NVS_init();
    // Initialize optional NVS parameters
    NVS_Params_init(&nvsParams);

}

void write_nvs_ext(){

    uint_fast16_t status;
    NVS_init();

    // Open NVS driver instance
    NVS_Params_init(&nvsParams);

    nvsRegion = NVS_open(CONFIG_NVS_EXT, &nvsParams);
//    __asm__("BKPT");
//     write "Hello" to the base address of region 0, verify after write
    status = NVS_write(nvsRegion, 0, (void *)but_write, sizeof(but_write), NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
    if (status != NVS_STATUS_SUCCESS) {
        // Error handling code
    }

//    // Close NVS region
    NVS_close(nvsRegion);

}

uint8_t succesfulRead = 0;

void read_nvs_ext(){
    succesfulRead = 0;
    uint_fast16_t status;
    // Open NVS driver instance
    nvsRegion = NVS_open(CONFIG_NVS_EXT, &nvsParams);

    // Copy "Hello" from nvsRegion into local 'buf'
    status = NVS_read(nvsRegion, 0, (void *)buf_read, sizeof(but_write));
    if (status != NVS_STATUS_SUCCESS) {
        // Error handling code
    }

//    __asm__("BKPT");
    if (!memcmp(but_write, buf_read, sizeof(but_write))){
        GPIO_write(CONFIG_GPIO_LED_RED,1);
        succesfulRead = 1;

    }

    // Close NVS region
    NVS_close(nvsRegion);

}

void erase_nvs_ext(void){

    uint_fast16_t status;
    nvsRegion = NVS_open(CONFIG_NVS_EXT, NULL);
    // Confirm that the NVS region opened properly
    if (nvsRegion == NULL) {
        // Error handling code
    }
    // Fetch the generic NVS region attributes for nvsRegion
    NVS_getAttrs(nvsRegion, &regionAttrs);
    // Erase the first sector of nvsRegion
    status = NVS_erase(nvsRegion, 0, regionAttrs.sectorSize);
    if (status != NVS_STATUS_SUCCESS) {
        // Error handling code
    }

    // Close NVS region
    NVS_close(nvsRegion);

}

