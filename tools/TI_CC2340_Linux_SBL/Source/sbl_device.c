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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "sbl_device.h"


/* Status and progress variables */
static uint32_t    sm_progress;
static tProgressFPTR    sm_pProgressFunction;
static tStatusFPTR      sm_pStatusFunction;
static  void appStatus(char *pcText, bool bError);
static  void appProgress(uint32_t progress);

/* Application callback - 1 */
static void setCallBackStatusFunction(tStatusFPTR pSf)
{
    sm_pStatusFunction = pSf;
}

/* Application callback - 2 */
static void setCallBackProgressFunction(tProgressFPTR pPf)
{
    sm_pProgressFunction = pPf;
}

/****************************************************************
 * Function Name : getCmdResponse
 * Description   : Get ACK/NAK from the boot loader.
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        @bAck: True if response is ACK, false if response
 *                      is NAK.
 *               @maxRetries: How many times ComPort::
 *               readBytes() can time out before fail is issued.
 ****************************************************************/
tSblStatus getCmdResponse(bool *bAck, uint8_t maxRetries)
{
    uint8_t pIn[2];
    memset(pIn, 0, 2);
    *bAck = false;
    uint32_t bytesRecv = 0;

    if(get_filed() < 0)
        return (SBL_PORT_ERROR);

    /* Expect 2 bytes */
    bytesRecv = serialRead(pIn, 2);

    if(bytesRecv < 2)
        return (SBL_TIMEOUT_ERROR);
    else
    {
        if(pIn[0] == 0x00 && pIn[1] == 0xCC)
        {
            *bAck = true;
            /*printf("ACK received 0x%02X 0x%02X.\n", pIn[0], pIn[1]);*/
            return (SBL_SUCCESS);
        }
        else if(pIn[0] == 0x00 && pIn[1] == 0x33)
        {
            printf("NACK received 0x%02X 0x%02X.\n", pIn[0], pIn[1]);
            return (SBL_SUCCESS);
        }
        else
        {
            printf("ACK/NAK not received. Expected 0x00 0xCC or 0x00 0x33, received 0x%02X 0x%02X.\n", pIn[0], pIn[1]);
            return SBL_ERROR;
        }
    }
}

/****************************************************************
 * Function Name : sendAutoBaud
 * Description   : Send auto baud.
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        @bBaudSetOk: True if response is ACK, false otherwise
 ****************************************************************/
tSblStatus sendAutoBaud(bool *bBaudSetOk)
{
    *bBaudSetOk = false;

    /* Send 0x55 0x55 and expect ACK */
    uint8_t pData[2];
    memset(pData, 0x55, 2);
    if(serialWrite(pData, 2) != 2)
    {
        printf("Communication init failed. Failed to Auto baud data.\n");
        return (SBL_PORT_ERROR);
    }

    if(getCmdResponse(bBaudSetOk, 2) != SBL_SUCCESS)
    {
        /* No response received. Invalid baud rate?*/
        printf("No response from device. Device may not be in bootloader mode. Reset device and try again.\nIf problem persists, check connection and baud rate.\n");
        return (SBL_PORT_ERROR);
    }

    return (SBL_SUCCESS);
}

/****************************************************************
 * Function Name : sendCmdResponse
 * Description   : Send command response (ACK/NAK).
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        @bAck: True if response is ACK, false if response
 *                      is NAK.
 ****************************************************************/
tSblStatus sendCmdResponse(bool bAck)
{
    if(get_filed() < 0)
        return (SBL_PORT_ERROR);

    uint8_t pData[2];
    pData[0] = 0x00;
    pData[1] = (bAck) ? 0xCC : 0x33;
    if(serialWrite(pData, 2) != 2)
    {
        printf("Communication init failed. Failed to send ACK/NAK response.\n");
        return (SBL_PORT_ERROR);
    }
    return (SBL_SUCCESS);
}

/****************************************************************
 * Function Name : getResponseData
 * Description   : Get response data from device.
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        @pcData: Pointer to where received data will be
 *                        stored.
 *               @ui32MaxLen: Max number of bytes that can be received.
 *                            Is populated with the actual number
 *                            of bytes received.
 *               @ui32MaxRetries: Not used
 ****************************************************************/
tSblStatus getResponseData(uint8_t *pcData, uint32_t *ui32MaxLen,
                           uint32_t ui32MaxRetries)
{
    uint8_t pcHdr[2];
    uint32_t numPayloadBytes;
    uint8_t hdrChecksum, dataChecksum;
    uint32_t bytesRecv = 0;

    if(get_filed() < 0)
        return (SBL_PORT_ERROR);

    /* Read length and checksum */
    memset(pcHdr, 0, 2);
    bytesRecv = serialRead(&pcHdr[bytesRecv], 2-bytesRecv);
    if(bytesRecv < 2)
        return (SBL_TIMEOUT_ERROR);

    numPayloadBytes = pcHdr[0]-2;
    hdrChecksum = pcHdr[1];

    /* Check if length byte is too long. */
    if(numPayloadBytes > *ui32MaxLen)
    {
        printf("Error: Device sending more data than expected. \nMax expected was %d, sent was %d.\n", *ui32MaxLen, (numPayloadBytes+2));
        return SBL_ERROR;
    }

    /* Read the payload data */
    bytesRecv = 0;
    bytesRecv = serialRead(&pcData[bytesRecv], (numPayloadBytes-bytesRecv));

    /* Have we received what we expected */
    if(bytesRecv < numPayloadBytes)
    {
        *ui32MaxLen = bytesRecv;
        printf("\nTIMEOUT ERROR\n");
        printf("\nexpected %i Bytes\n",numPayloadBytes);
        printf("\nreceived %i Bytes\n",bytesRecv);
        return (SBL_TIMEOUT_ERROR);
    }

    /* Verify data checksum */
    dataChecksum = generateCheckSum(0, (const char*)pcData, numPayloadBytes);
    if(dataChecksum != hdrChecksum)
    {
        printf("Checksum verification error. Expected 0x%02X, got 0x%02X.\n", hdrChecksum, dataChecksum);
        return (SBL_ERROR);
    }

    *ui32MaxLen = bytesRecv;
    return SBL_SUCCESS;
}

/****************************************************************
 * Function Name : sendCmd
 * Description   : Send command.
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        @ui32Cmd: The command to send.
 *               @pcSendData: Pointer to the data to send with the
 *               command.
 *               @ui32SendLen: The number of bytes to send from
 *               \e pcSendData.
 ****************************************************************/
tSblStatus sendCmd(cmd_t cmdType, const uint8_t *pcSendData,
                   uint32_t ui32SendLen)
{

    if(get_filed() < 0)
        return (SBL_PORT_ERROR);

    uint8_t pktLen = ui32SendLen + 3; // +3 => <1b Length>, <1B cksum>, <1B cmd>
    uint8_t* cmdPkt = (uint8_t*)calloc(pktLen, sizeof(uint8_t));

    if(!cmdPkt)
        return (SBL_MALLOC_ERROR);

    uint8_t pktSum = generateCheckSum(cmdType, (const char*)pcSendData, ui32SendLen);
    cmdPkt[0] = pktLen;
    cmdPkt[1] = pktSum;
    cmdPkt[2] = cmdType;

    if(ui32SendLen)
        memcpy(&cmdPkt[3], pcSendData, ui32SendLen);

    /* Send the packet */
    if(serialWrite(cmdPkt, pktLen) != pktLen)
    {
        printf("Writing to device failed [CMD: 0x%2x]\n",(uint8_t)cmdType);
        free(cmdPkt);
        return (SBL_PORT_ERROR);
    }

    free(cmdPkt);
    return (SBL_SUCCESS);
}

/****************************************************************
 * Function Name : generateCheckSum
 * Description   : This function generates the bootloader protocol
 *                 checksum.
 * Returns       : Returns the generated checksum.
 * Params        @ui32Cmd: The bootloader command
 *               @pcData: Pointer to the command data.
 *               @ui32DataLen: Data length in bytes.
 ****************************************************************/
uint8_t generateCheckSum(cmd_t cmdType, const char *pcData,
                               uint32_t ui32DataLen)
{
    uint8_t ui8CheckSum = cmdType;
    for(uint32_t i = 0; i < ui32DataLen; i++)
    {
        ui8CheckSum += pcData[i];
    }
    return ui8CheckSum;
}

/****************************************************************
 * Function Name : calcCrcLikeChip
 * Description   : Calculate crc32 checksum the way CC2340 does it.
 * Returns       : Checksum
 * Params        :
 ****************************************************************/
uint32_t calcCrcLikeChip(const uint8_t *pData, uint32_t ulByteCount)
{
    uint32_t d, ind;
    uint32_t acc = 0xFFFFFFFF;
    const uint32_t ulCrcRand32Lut[] =
    {
     0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
     0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
     0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
     0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C
    };

    while (ulByteCount--)
    {
        d = *pData++;
        ind = (acc & 0x0F) ^ (d & 0x0F);
        acc = (acc >> 4) ^ ulCrcRand32Lut[ind];
        ind = (acc & 0x0F) ^ (d >> 4);
        acc = (acc >> 4) ^ ulCrcRand32Lut[ind];
    }

    return (acc ^ 0xFFFFFFFF);
}

/****************************************************************
 * Function Name : setProgress
 * Description   : This functions sets the SBL progress.
 * Returns       : Void
 * Params        @ui32Progress: The current progress, typically
 *                              in percent [0-100].
 ****************************************************************/
tSblStatus setProgress(uint32_t ui32Progress)
{
    if(sm_pProgressFunction)
    {
        sm_pProgressFunction(ui32Progress);
    }

    sm_progress = ui32Progress;

    return (SBL_SUCCESS);
}

/****************************************************************
 * Function Name : byteSwap
 * Description   : Utility function for swapping the byte order
 *                 of a 4B char array.
 * Returns       : Void
 * Params        @pcArray: The char array to byte swap.
 ****************************************************************/
void byteSwap(uint8_t *pcArray)
{
    uint8_t tmp[2] = {pcArray[0], pcArray[1]};
    pcArray[0] = pcArray[3];
    pcArray[1] = pcArray[2];
    pcArray[2] = tmp[1];
    pcArray[3] = tmp[0];
}

/****************************************************************
 * Function Name : ulToCharArray
 * Description   : Utility function for splitting 32 bit variable
 *                  into char array (4 elements). Data are converted
 *                  MSB, that is, \e pcDst[0] is the most significant
 *                  byte. of a 4B char array.
 * Returns       : Void
 * Params        @ui32Src: The 32 bit variable to convert.
 *               @pcDst: Pointer to the char array where the data
 *                       will be stored.
 ****************************************************************/
void ulToCharArray(const uint32_t ui32Src, uint8_t *pcDst)
{
    /* MSB first*/
    pcDst[0] =  (uint8_t)(ui32Src >> 24);
    pcDst[1] =  (uint8_t)(ui32Src >> 16);
    pcDst[2] =  (uint8_t)(ui32Src >> 8);
    pcDst[3] =  (uint8_t)(ui32Src >> 0);
}

/****************************************************************
 * Function Name : charArrayToUL
 * Description   : Utility function for converting 4 elements in
                   char array into 32 bit variable. Data are
                   converted MSB, that is. \e pcSrc[0] is the
                   most significant byte.
 * Returns       :  Returns the 32 bit variable.
 * Params        @pcSrc: A pointer to the source array.
 ****************************************************************/
uint32_t charArrayToUL(const char *pcSrc)
{
    uint32_t ui32Val = (uint8_t)pcSrc[3];
    ui32Val += (((uint32_t)pcSrc[2]) & 0xFF) << 8;
    ui32Val += (((uint32_t)pcSrc[1]) & 0xFF) << 16;
    ui32Val += (((uint32_t)pcSrc[0]) & 0xFF) << 24;
    return (ui32Val);
}

/****************************************************************
 * Function Name : readStatus
 * Description   : This function gets status from device.
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        @pui32Status: Pointer to where status is stored.
 ****************************************************************/
tSblStatus readStatus(uint32_t *pui32Status)
{
    tSblStatus retCode = SBL_SUCCESS;
    bool bSuccess = false;

    if(get_filed() < 0)
        return (SBL_PORT_ERROR);

    /* Send command */
    if((retCode = sendCmd(CMD_GET_STATUS, NULL, 0)) != SBL_SUCCESS)
        return retCode;

    /* Receive command response */
    if((retCode = getCmdResponse(&bSuccess, 0)) != SBL_SUCCESS)
        return retCode;

    if(!bSuccess)
        return SBL_ERROR;

    /* Receive command response data */
    uint8_t status = 0;
    uint32_t ui32NumBytes = 1;
    if((retCode = getResponseData(&status, &ui32NumBytes, 0)) != SBL_SUCCESS)
    {
        /* Respond with NAK */
        sendCmdResponse(false);
        sleep(1);
        return retCode;
    }

    /* Respond with ACK */
    sendCmdResponse(true);
    sleep(1);

    *pui32Status = status;
    return SBL_SUCCESS;
}

/****************************************************************
 * Function Name : getCmdStatusString
 * Description   : This function returns a string with the device
 *                  status name of \e ui32Status serial bootloader
 *                  status value.
 * Returns       :  Returns string with name of device status.
 * Params        @ui32Status: The serial bootloader status value.
 ****************************************************************/
char *getCmdStatusString(cmdRespStatus_t ui32Status)
{
    switch(ui32Status)
    {
    case CMD_RET_FLASH_FAIL:    return "FLASH_FAIL"; break;
    case CMD_RET_INVALID_ADR:   return "INVALID_ADR"; break;
    case CMD_RET_INVALID_CMD:   return "INVALID_CMD"; break;
    case CMD_RET_SUCCESS:       return "SUCCESS"; break;
    case CMD_RET_UNKNOWN_CMD:   return "UNKNOWN_CMD"; break;
    default:                    return "Unknown status"; break;
    }
}

/****************************************************************
 * Function Name : getCmdString
 * Description   : This function returns a string with the device
                     command name of \e ui32Cmd.
 * Returns       :  Returns string with name of device command.
 * Params        @ui32Cmd: The serial bootloader command.
 ****************************************************************/
char *getCmdString(cmd_t ui32Cmd)
{
    switch(ui32Cmd)
    {
    case CMD_PING:             return "CMD_PING"; break;
    case CMD_CRC32:            return "CMD_CRC32"; break;
    case CMD_DOWNLOAD:         return "CMD_DOWNLOAD"; break;
    case CMD_GET_CHIP_ID:      return "CMD_GET_CHIP_ID"; break;
    case CMD_GET_STATUS:       return "CMD_GET_STATUS"; break;
    case CMD_RESET:            return "CMD_RESET"; break;
    default: return "Unknown command"; break;
    }
}

/* Application status function
 * (used as SBL status callback) */
void appStatus(char *pcText, bool bError)
{
    if(bError)
        fprintf(stderr, "%s", pcText);
    else
        printf("appStatus: %s\r\n", pcText);
}

/* Application progress function
 * (used as SBL progress callback) */
void appProgress(uint32_t progress)
{
    fprintf(stdout, "\r%d%% ", progress);
    fflush(stdout);
}

/****************************************************************
 * Function Name : setupCallbacks
 * Description   : Register callbacks
 * Returns       : None
 * Params        @ui32Status: The serial bootloader status value.
 ****************************************************************/
void setupCallbacks(void)
{
    setCallBackStatusFunction(&appStatus);
    setCallBackProgressFunction(&appProgress);
}

