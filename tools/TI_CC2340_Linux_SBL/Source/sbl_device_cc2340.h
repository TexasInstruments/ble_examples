/*
 * Copyright (c) 2021-2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */ 

#ifndef SBL_DEVICE_CC2340_H_
#define SBL_DEVICE_CC2340_H_

#include "sbl_device.h"
#include "Linux_Serial.h"

#define CC23XX_FLASH_BASE                   0x00000000

#define SBL_CC2340_PAGE_ERASE_SIZE          2048	
#define SBL_CC2340_FLASH_START_ADDRESS      0x00000000
#define SBL_CC2340_RAM_START_ADDRESS        0x20000000

#define SBL_CC2340_ACCESS_WIDTH_32B         1
#define SBL_CC2340_ACCESS_WIDTH_8B          0
#define SBL_CC2340_PAGE_ERASE_TIME_MS       20
#define SBL_CC2340_MAX_BYTES_PER_TRANSFER   252
#define SBL_CC2340_MAX_MEMWRITE_BYTES       247
#define SBL_CC2340_MAX_MEMWRITE_WORDS       61
#define SBL_CC2340_MAX_MEMREAD_BYTES        253
#define SBL_CC2340_MAX_MEMREAD_WORDS        63
#define SBL_CC2340_FLASH_SIZE_CFG           0x40024004
#define SBL_CC2340_BL_CONFIG_PAGE_OFFSET    0xFDB
#define SBL_CC2340_BL_CONFIG_ENABLED_BM     0xC5
#define SBL_CC2340_BL_WORK_MEMORY_START     0x20000000
#define SBL_CC2340_BL_WORK_MEMORY_END       0x2000016F
#define SBL_CC2340_BL_STACK_MEMORY_START    0x20000FC0
#define SBL_CC2340_BL_STACK_MEMORY_END      0x20000FFF

#define SBL_CC2340_CCFG_START_ADDRESS       0x4E020000
#define SBL_CC2340_RAM_SIZE_CFG             0x40082250 

#define SBL_CC2340_CCFG_BASE      0x4E020000



extern tSblStatus ping();                                      
extern tSblStatus reset();                                      
extern void setDeviceFlashBase(uint32_t valFlashBase);          
extern uint32_t getDeviceFlashBase();                           
extern uint32_t getFlashSize();                                 
extern uint32_t getRamSize();                                   
extern tSblStatus writeFlashRange(uint32_t ui32StartAddress,
                           uint32_t ui32ByteCount, const char *pcData); 
extern tSblStatus eraseFlashRange(uint32_t ui32StartAddress,
                              uint32_t ui32ByteCount);                 
extern tSblStatus calculateCrc32(uint32_t ui32StartAddress,
                                 uint32_t ui32ByteCount, uint32_t *pui32Crc);   
extern tSblStatus detectAutoBaud(void);                                
extern tSblStatus readFlashSize(uint32_t *pui32FlashSize);                
extern tSblStatus readRamSize(uint32_t *pui32RamSize);                 

extern tSblStatus chipErase();                                          

#endif /* SBL_DEVICE_CC2340_H_ */
