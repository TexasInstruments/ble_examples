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
#ifndef SBL_DEVICE_H_
#define SBL_DEVICE_H_
#include <stdbool.h>
#include "Linux_Serial.h"

typedef enum {
    CMD_PING             = 0x20,
    CMD_GET_STATUS       = 0x21,
    CMD_GET_CHIP_ID      = 0x22,
    CMD_RESET            = 0x23,
    CMD_CHIP_ERASE       = 0x24,
    CMD_CRC32            = 0x25,
    CMD_DOWNLOAD         = 0x26,
    CMD_DOWNLOAD_CRC     = 0x27,
    CMD_SEND_DATA        = 0x28,
    CMD_SECTOR_ERASE     = 0x29,
    CMD_MEMORY_READ      = 0x2A,
    CMD_MEMORY_WRITE     = 0x2B,
    CMD_SET_CCFG         = 0x2D,
    CMD_BANK_ERASE       = 0x2C,
}cmd_t;


typedef enum {
    SBL_SUCCESS = 0,
    SBL_ERROR,
    SBL_ARGUMENT_ERROR,
    SBL_TIMEOUT_ERROR,
    SBL_PORT_ERROR,
    SBL_ENUM_ERROR,
    SBL_UNSUPPORTED_FUNCTION,
    SBL_MALLOC_ERROR
} tSblStatus;

/* Early samples had different command IDs */
typedef enum
{
    REV1_CMD_BANK_ERASE   = 0x2A,
    REV1_CMD_SET_CCFG     = 0x2B,
    REV1_CMD_MEMORY_READ  = 0x2C,
    REV1_CMD_MEMORY_WRITE = 0x2D,
}cmdRev1_t;

typedef enum {
    CMD_RET_SUCCESS      = 0x40,
    CMD_RET_UNKNOWN_CMD  = 0x41,
    CMD_RET_INVALID_CMD  = 0x42,
    CMD_RET_INVALID_ADR  = 0x43,
    CMD_RET_FLASH_FAIL   = 0x44,
    CMD_RET_RET_CRC_FAIL         = 0x45,
    CMD_RET_RET_NEEDS_CHIP_ERASE = 0x46,
}cmdRespStatus_t;

extern tSblStatus sendAutoBaud(bool *bBaudSetOk);
extern tSblStatus getCmdResponse(bool *bAck, uint8_t maxRetries);
extern tSblStatus sendCmdResponse(bool bAck);
extern tSblStatus getResponseData(uint8_t *pcData, uint32_t *ui32MaxLen,
                                  uint32_t ui32MaxRetries);
extern uint8_t generateCheckSum(cmd_t cmdType, const char *pcData,
                                      uint32_t ui32DataLen);
extern uint32_t calcCrcLikeChip(const uint8_t *pData, uint32_t ulByteCount);
extern tSblStatus setProgress(uint32_t ui32Progress);
extern tSblStatus sendCmd(cmd_t cmdType, const uint8_t *pcSendData/* = NULL*/,
                   uint32_t ui32SendLen/* = 0*/);
extern tSblStatus readStatus(uint32_t *pui32Status);
extern char *getCmdStatusString(cmdRespStatus_t ui32Status);
extern char *getCmdString(cmd_t ui32Cmd);
extern void byteSwap(uint8_t *pcArray);
extern void ulToCharArray(const uint32_t ui32Src, uint8_t *pcDst);
extern uint32_t charArrayToUL(const char *pcSrc);
extern void setupCallbacks(void);
/*
 Typedefs for callback functions to report status and progress to application
*/
typedef void (*tStatusFPTR)(char *pcText, bool bError);
typedef void (*tProgressFPTR)(uint32_t ui32Value);

#endif  SBL_DEVICE_H_ 
