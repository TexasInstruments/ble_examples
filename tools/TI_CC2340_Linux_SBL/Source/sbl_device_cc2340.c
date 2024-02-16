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
#include <stdbool.h>
#include <stdint.h>
#include "sbl_device_cc2340.h"


/* Macros */
#define MIN(x, y) (((x) < (y)) ? (x) : (y))


/* Struct used when splitting long transfers */
typedef struct {
    uint32_t startAddr;
    uint32_t byteCount;
    uint32_t startOffset;
    bool     bExpectAck;
} tTransfer;

/* Static Var's */
static uint32_t m_flashSize;
static uint32_t m_ramSize;
static uint32_t m_deviceId;
static uint32_t devFlashBase;
static bool m_bCommInitialized;


/* Device revision. Used internally by SBL to handle
 * early samples with different command IDs.
 */
static uint32_t m_deviceRev;

/* Static functions */
static tSblStatus cmdDownload(uint32_t ui32Address, uint32_t ui32Size);
static uint32_t addressToPage(uint32_t ui32Address);

/* Some small functions. Lets save some file space */
uint32_t getFlashSize() { return (m_flashSize);}
uint32_t getRamSize() { return m_ramSize; }

void setDeviceFlashBase(uint32_t valFlashBase) { devFlashBase = valFlashBase;}
uint32_t getDeviceFlashBase() { return(devFlashBase);}

/****************************************************************
 * Function Name : eraseFlashRange
 * Description   : This function erases device flash pages.
 *                 Starting page is the page that includes the
 *                 address in \e startAddress. Ending page is the page
 *                 that includes the address <startAddress + byteCount>.
 *                 CC2340 erase size is 2KB.
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        @ui32StartAddress: The start address in flash.
 *               @ui32ByteCount: The number of bytes to erase.
 ****************************************************************/
tSblStatus eraseFlashRange(uint32_t ui32StartAddress,
                              uint32_t ui32ByteCount)
{
    tSblStatus retCode = SBL_SUCCESS;
    bool bSuccess = false;
    uint8_t pcPayload[4];
    uint32_t devStatus;

    if(get_filed() < 0)
        return (SBL_PORT_ERROR);

    /*Calculate retry count */
    uint32_t ui32PageCount = ui32ByteCount / SBL_CC2340_PAGE_ERASE_SIZE;
    if( ui32ByteCount % SBL_CC2340_PAGE_ERASE_SIZE) ui32PageCount ++;
    setProgress( 0 );

    for(uint32_t i = 0; i < ui32PageCount; i++)
    {
        /* Build payload - 4B address (MSB first) */
        ulToCharArray(ui32StartAddress + i*(4096), &pcPayload[0]);

        /* Send command */
        if((retCode = sendCmd(CMD_SECTOR_ERASE, pcPayload, 4)) != SBL_SUCCESS)
            return (retCode);

        /* Receive command response (ACK/NAK) */
        if((retCode = getCmdResponse(&bSuccess, 0)) != SBL_SUCCESS)
            return (retCode);

        if(!bSuccess)
            return (SBL_ERROR);

        /* Check device status (Flash failed if page(s) locked) */

        readStatus(&devStatus);
        if(devStatus != CMD_RET_SUCCESS)
        {
            printf("Flash erase failed. (Status 0x%02X = %s). Flash pages may be locked.\n", devStatus, getCmdStatusString(devStatus));
            return (SBL_ERROR);
        }

        setProgress( 100*(i+1)/ui32PageCount );
    }

    return (SBL_SUCCESS);
}


/****************************************************************
 * Function Name : ping
 * Description   : This function sends ping command to device.
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        : None
 ****************************************************************/
tSblStatus ping()
{
    tSblStatus retCode = SBL_SUCCESS;
    bool bResponse = false;

    if(get_filed() < 0)
        return (SBL_PORT_ERROR);

    /* Send command */
    if((retCode = sendCmd(CMD_PING, NULL, 0)) != SBL_SUCCESS)
        return retCode;

    /* Get response */
    if((retCode = getCmdResponse(&bResponse, 0)) != SBL_SUCCESS)
        return retCode;

    return (bResponse) ? SBL_SUCCESS : SBL_ERROR;
}

/****************************************************************
 * Function Name : getDeviceRev
 * Description   : This function sends ping command to device.
 * Returns       : None
 * Params        : None
 ****************************************************************/
uint32_t getDeviceRev(uint32_t deviceId)
{
    uint32_t tmp = deviceId >> 28;
    switch(tmp)
    {
    /* Early samples (Rev 1)*/
    case 0:
    case 1:
        return 1;
    default:
        return 2;
    }
}

/****************************************************************
 * Function Name : readDeviceId
 * Description   : This function reads device ID.
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        : @pui32DeviceId: Pointer to where device ID is
 *                                 stored.
 ****************************************************************/
tSblStatus readDeviceId(uint32_t *pui32DeviceId)
{
    int retCode = SBL_SUCCESS;
    bool bSuccess = false;

    if(get_filed() < 0)
        return (SBL_PORT_ERROR);

    /* Send command */
    if((retCode = sendCmd(CMD_GET_CHIP_ID, NULL, 0)) != SBL_SUCCESS)
        return retCode;

    /* Receive command response (ACK/NAK) */
    if((retCode = getCmdResponse(&bSuccess, 0)) != SBL_SUCCESS)
        return retCode;

    if(!bSuccess)
        return (SBL_ERROR);

    /* Receive response data */
    uint8_t pId[4];
    memset(pId, 0, 4);
    uint32_t numBytes = 4;
    if((retCode = getResponseData(pId, &numBytes, 0)) != SBL_SUCCESS)
    {
        /* Respond with NAK */
        sendCmdResponse(false);
        return retCode;
    }

    if(numBytes != 4)
    {
        /* Respond with NAK */
        sendCmdResponse(false);
        printf("Didn't receive 4 B.\n");
        return (SBL_ERROR);
    }

    /* Respond with ACK */
    sendCmdResponse(true);

    /* Store retrieved ID and report success */
    *pui32DeviceId = charArrayToUL((const char*)pId);
    m_deviceId = *pui32DeviceId;

    /* Store device revision (used internally, see sbl_device_cc2340.h) */
    m_deviceRev = getDeviceRev(m_deviceId);

    return (SBL_SUCCESS);
}

/****************************************************************
 * Function Name : readFlashSize
 * Description   : This function reads device FLASH size in bytes.
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        : @pui32FlashSize: Pointer to where FLASH size is
 *                  stored.
 ****************************************************************/
tSblStatus readFlashSize(uint32_t *pui32FlashSize)
{
    tSblStatus retCode = SBL_SUCCESS;

    *pui32FlashSize = 512*1024;

    m_flashSize = *pui32FlashSize;

    return (SBL_SUCCESS);
}

/****************************************************************
 * Function Name : readRamSize
 * Description   : This function reads device RAM size in bytes.
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        : @pui32RamSize: Pointer to where RAM size is
                     stored.
 ****************************************************************/
tSblStatus readRamSize(uint32_t *pui32RamSize)
{
    int retCode = SBL_SUCCESS;
    /* Save RAM size internally */
    *pui32RamSize=(36*1024);
    m_ramSize = (36*1024);

    return (retCode);
}

/****************************************************************
 * Function Name : reset
 * Description   : This function reset the device. Communication
                   to the device must be reinitialized after calling
                   this function.
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        : None.
 ****************************************************************/
tSblStatus reset()
{
    tSblStatus retCode = SBL_SUCCESS;
    bool bSuccess = false;

    if(get_filed() < 0)
        return (SBL_PORT_ERROR);

    /* Send CMD */
    if((retCode = sendCmd(CMD_RESET, NULL, 0)) != SBL_SUCCESS)
        return retCode;

    /* Receive command response (ACK/NAK) */
    if((retCode = getCmdResponse(&bSuccess, 0)) != SBL_SUCCESS)
        return retCode;

    if(!bSuccess)
    {
        printf("Reset command NAKed by device.\n");
        return (SBL_ERROR);
    }

    m_bCommInitialized = false;
    return (SBL_SUCCESS);
}

/****************************************************************
 * Function Name : addressInBLWorkMemory
 * Description   :  This function checks if the specified \e
                    ui32StartAddress (and range) overlaps the
                    bootloader's working memory or stack area.

    The bootloader does not protect against writing to these ranges,
    but doing so is almost guaranteed to crash the bootloader and
    requires a reboot. SRAM ranges used by the bootloader:
       Work memory @ 0x20000000-0x2000016F
       Stack area  @ 0x20000FC0-0x20000FFF

 * Returns       : Returns true if the address/range is within
                     the device RAM.
 * Params        @ui32StartAddress:The start address of the range
 *               @ui32ByteCount: (Optional) The number of bytes
 *               in the range.
 ****************************************************************/
bool addressInBLWorkMemory(uint32_t ui32StartAddress,
                           uint32_t ui32ByteCount/* = 1*/)
{
    uint32_t ui32EndAddr = ui32StartAddress + ui32ByteCount;

    if(ui32StartAddress <= SBL_CC2340_BL_WORK_MEMORY_END)
        return true;

    if((ui32StartAddress >= SBL_CC2340_BL_STACK_MEMORY_START) &&
            (ui32StartAddress <= SBL_CC2340_BL_STACK_MEMORY_END))
        return true;

    if((ui32EndAddr >= SBL_CC2340_BL_STACK_MEMORY_START) &&
            (ui32EndAddr <= SBL_CC2340_BL_STACK_MEMORY_END))
        return true;

    return false;
}

/****************************************************************
 * Function Name : cmdSendData
 * Description   : This function sends the CC2340 SendData command
                     and handles the device response.
 * Returns       :  Returns SBL_SUCCESS if command and response was
                     successful.
 * Params        : @pcData: Pointer to the data to send.
 *                 @ui32ByteCount: The number of bytes to send.
 ****************************************************************/
tSblStatus cmdSendData(const uint8_t *pcData, uint32_t ui32ByteCount)
{
    tSblStatus retCode = SBL_SUCCESS;
    bool bSuccess = false;

    /* Check input arg's */
    if(ui32ByteCount > SBL_CC2340_MAX_BYTES_PER_TRANSFER)
    {
        printf("Error: Byte count (%d) exceeds maximum transfer size %d.\n", ui32ByteCount, SBL_CC2340_MAX_BYTES_PER_TRANSFER);
        return (SBL_ERROR);
    }

    /* Send CMD */
    if((retCode = sendCmd(CMD_SEND_DATA, pcData, ui32ByteCount)) != SBL_SUCCESS)
        return (retCode);

    /* Receive command response (ACK/NAK) */
    if((retCode = getCmdResponse(&bSuccess, 3)) != SBL_SUCCESS)
        return (retCode);

    if(!bSuccess)
        return (SBL_ERROR);

    return (SBL_SUCCESS);
}

/****************************************************************
 * Function Name : writeFlashRange
 * Description   : Write \e unitCount words of data to device FLASH.
                   Source array is 8 bit wide. Parameters \e
                   startAddress and \e unitCount must be a a
                   multiple of 4. This function does not erase the
                   flash before writing data, this must be done
                   using e.g. eraseFlashRange().
 * Returns       : Returns SBL_SUCCESS, ...
 * Params        : @ui32StartAddress: Start address in device. Must
                     be a multiple of 4.
 *                 @ui32UnitCount: Must be a multiple of 4.
 *                 @pcData:Pointer to source data.
 ****************************************************************/
tSblStatus writeFlashRange(uint32_t ui32StartAddress,
                           uint32_t ui32ByteCount, const char *pcData)
{

    uint32_t devStatus = CMD_RET_UNKNOWN_CMD;
    tSblStatus retCode = SBL_SUCCESS;
    char u8tCmdStatus[1];
    uint32_t bytesLeft, dataIdx, bytesInTransfer,u32tStatusLength=1;
    uint32_t transferNumber = 1;
    bool bACK=true;
    bool bBlToBeDisabled = false;
    tTransfer pvTransfer[2];
    uint32_t ui32TotChunks = (ui32ByteCount / SBL_CC2340_MAX_BYTES_PER_TRANSFER);
    uint32_t ui32CurrChunk = 0;
    if(ui32ByteCount % SBL_CC2340_MAX_BYTES_PER_TRANSFER) ui32TotChunks++;


    /*Init transfer*/
    pvTransfer[0].bExpectAck  = true;           /*Check if operation successfull?*/
    pvTransfer[0].byteCount = ui32ByteCount;   
    pvTransfer[0].startAddr = ui32StartAddress;
    pvTransfer[0].startOffset = 0;
    

    /* For each transfer */
    for(uint32_t i = 0; i < 1; i++)
    {
        /* Sanity check */
        if(pvTransfer[i].byteCount == 0)
            continue;

        /* Set progress */
        setProgress(addressToPage(pvTransfer[i].startAddr));

        /* Send download command */
        if((retCode = cmdDownload(pvTransfer[i].startAddr,
                                  pvTransfer[i].byteCount)) != SBL_SUCCESS)
            return (retCode);

        /* Check status after download command */
        retCode = readStatus(&devStatus);
        if(retCode != SBL_SUCCESS)
        {
            printf("Error during download initialization. Failed to read device status after sending download command.\n");
            return (retCode);
        }

        /* Send data in chunks */
        bytesLeft = pvTransfer[i].byteCount;
        dataIdx   = pvTransfer[i].startOffset;
        while(bytesLeft)
        {
            /* Set progress */
            setProgress( ((100*(++ui32CurrChunk))/ui32TotChunks) );		
            /* Limit transfer count */
            bytesInTransfer = MIN(SBL_CC2340_MAX_BYTES_PER_TRANSFER, bytesLeft);

            /* Send Data command */
            if((retCode = cmdSendData((const uint8_t*)&pcData[dataIdx], bytesInTransfer)) != SBL_SUCCESS)
            {
                printf("Error during flash download. \n- Start address 0x%08X (page %d). \n- Tried to transfer %d bytes. \n- This was transfer %d.\n",
                       (ui32StartAddress+dataIdx),
                       addressToPage(ui32StartAddress+dataIdx),
                       bytesInTransfer,
                       (transferNumber));
                return (retCode);
            }
            if(pvTransfer[0].bExpectAck){
            	if((retCode=sendCmd(CMD_GET_STATUS,NULL,0))!=SBL_SUCCESS){
    			printf("Sending GET_STATUS_CMD failed\n");
    		return(SBL_ERROR);}

    		/* Receive command response (ACK/NAK) */
    		if((retCode = getCmdResponse(&bACK, 5)) != SBL_SUCCESS){
    		printf("Device NAKed Status CMD command\n");
    		return (retCode);}

    		if(retCode = getResponseData((uint8_t*)u8tCmdStatus,&u32tStatusLength,0) != SBL_SUCCESS){
    			sendCmdResponse(false);
        		return (retCode);}
                
    		sendCmdResponse(true);
    		if(u8tCmdStatus[0]!=0x40){
    		printf("Error during flash download. \n- Start address 0x%08X (page %d). \n- Tried to transfer %d bytes. \n- This was transfer %d.\n",
                       (ui32StartAddress+dataIdx),
                       addressToPage(ui32StartAddress+dataIdx),
                       bytesInTransfer,
                       (transferNumber));
                       return (SBL_ERROR);
    		}
            
            }
            /* Update index and bytesLeft */
            bytesLeft -= bytesInTransfer;
            dataIdx += bytesInTransfer;
            transferNumber++;
        }
    }
    return (SBL_SUCCESS);
}

/****************************************************************
 * Function Name : setCCFG
 * Description   : Writes the CC234x defined CCFG fields to the
 *                  flash CCFG area with the values received in
 *                  the data bytes of this command.
 * Returns       :  Returns SBL_SUCCESS, ...
 * Params        : @ui32Field: CCFG Field ID which identifies the
 *                  CCFG parameter to be written.
 *                 @ui32FieldValue:  Field value to be programmed.
 ****************************************************************/
tSblStatus setCCFG(uint32_t ui32Field, uint32_t ui32FieldValue)
{
    tSblStatus retCode = SBL_SUCCESS;
    bool bSuccess = false;

    if(get_filed() < 0)
        return (SBL_PORT_ERROR);

    /*
     Generate payload
     - 4B Field ID
     - 4B Field value
    */
    char pcPayload[8];
    ulToCharArray(ui32Field, (uint8_t*)&pcPayload[0]);
    ulToCharArray(ui32FieldValue, (uint8_t*)&pcPayload[4]);

    /* Send command */
    if((retCode = sendCmd(CMD_SET_CCFG, (const uint8_t*)pcPayload, 8)) != SBL_SUCCESS)
        return (retCode);

    /* Receive command response (ACK/NAK) */
    if((retCode = getCmdResponse(&bSuccess, 0)) != SBL_SUCCESS)
        return (retCode);

    if(!bSuccess)
    {
        printf("Set CCFG command NAKed by device.\n");
        return (SBL_ERROR);
    }

    return (SBL_SUCCESS);
}

/****************************************************************
 * Function Name : addressToPage
 * Description   : This function returns the page within which
 *                  address \e ui32Address is located.
 * Returns       :  Returns the flash page within which an address
 *                  is located.
 * Params        : @ui32Address: The address.
 ****************************************************************/
uint32_t addressToPage(uint32_t ui32Address)
{
    return ((ui32Address - SBL_CC2340_FLASH_START_ADDRESS) /                  \
            SBL_CC2340_PAGE_ERASE_SIZE);
}

/****************************************************************
 * Function Name : addressInRam
 * Description   : This function checks if the specified \e
                     ui32StartAddress (and range)is located within
                     the device RAM area.
 * Returns       :  Returns true if the address/range is within
 *                  the device RAM.
 * Params        : @ui32Address: The start address of the range.
 *                 @pui32Bytecount:The number of bytes in the range.
 ****************************************************************/
bool addressInRam(uint32_t ui32StartAddress,
                  uint32_t ui32ByteCount/* = 1*/)
{
    uint32_t ui32EndAddr = ui32StartAddress + ui32ByteCount;

    if(ui32StartAddress < SBL_CC2340_RAM_START_ADDRESS)
        return false;

    if(ui32EndAddr > (SBL_CC2340_RAM_START_ADDRESS + getRamSize()))
        return false;

    return true;
}

/****************************************************************
 * Function Name : addressInFlash
 * Description   : This function checks if the specified \e
 *                  ui32StartAddress (and range) is located within
 *                  the device FLASH area.
 * Returns       :  Returns true if the address/range is within the
 *                  device flash.
 * Params        : @ui32Address: The start address of the range
 *                 @pui32Bytecount:The number of bytes in the range.
 ****************************************************************/
bool addressInFlash(uint32_t ui32StartAddress,
                    uint32_t ui32ByteCount/* = 1*/)
{
    uint32_t ui32EndAddr = ui32StartAddress + ui32ByteCount;

    if((ui32StartAddress < SBL_CC2340_FLASH_START_ADDRESS) && (ui32StartAddress<SBL_CC2340_CCFG_START_ADDRESS))
        return false;

    if(ui32EndAddr > (SBL_CC2340_FLASH_START_ADDRESS + getFlashSize()))
    {
        if(ui32EndAddr>(SBL_CC2340_CCFG_START_ADDRESS+0x800))
        {
            return false;
        }
    }
    
    return true;
}

/****************************************************************
 * Function Name : cmdDownload
 * Description   : This function sends the CC2340 download command
 *                  and handles the device response.
 * Returns       :  Returns SBL_SUCCESS if command and response was
 *                  successful.
 * Params        : @ui32Address:  The start address in CC2340 flash.
 *                 @pui32Bytecount:The total number of bytes to
 *                 program on the device.
 ****************************************************************/
tSblStatus cmdDownload(uint32_t ui32Address, uint32_t ui32Size)
{
    int retCode = SBL_SUCCESS;
    bool bSuccess = false;

    /*Check input arguments*/ 
    if(!addressInFlash(ui32Address, ui32Size))
    {
        printf("Flash download: Address range (0x%08X + %d bytes) is not in device FLASH nor RAM.\n", ui32Address, ui32Size);
        return (SBL_ARGUMENT_ERROR);
    }
    if(ui32Size & 0x03)
    {
        printf("Flash download: Byte count must be a multiple of 4\n");
        return (SBL_ARGUMENT_ERROR);
    }

    /*
    Generate payload
    - 4B Program address
    - 4B Program size
    */    
    
    char pcPayload[8];
    ulToCharArray(ui32Address, (uint8_t*)&pcPayload[0]);
    ulToCharArray(ui32Size, (uint8_t*)&pcPayload[4]);

    /* Send command */
    if((retCode = sendCmd(CMD_DOWNLOAD, (const uint8_t*)pcPayload, 8)) != SBL_SUCCESS)
        return retCode;

    /* Receive command response (ACK/NAK) */
    if((retCode = getCmdResponse(&bSuccess, 0)) != SBL_SUCCESS)
        return retCode;

    /* Return command response */
    return (bSuccess) ? SBL_SUCCESS : SBL_ERROR;
}

/****************************************************************
 * Function Name : 
 
 * Description   : Calculate CRC over \e byteCount bytes, starting
 *                  at address  \e startAddress.
 * Returns       :  Returns SBL_SUCCESS, ...
 * Params        : @ui32StartAddress:  Start address in device.
 *                 @ui32ByteCount:Number of bytes to calculate CRC32
 *                 over.
 *                 @pui32Crc: Pointer to where checksum from device
 *                 is stored.
 ****************************************************************/
tSblStatus calculateCrc32(uint32_t ui32StartAddress,uint32_t ui32ByteCount, uint32_t *pui32Crc)
{
    printf("+-----------------------------------\n");
    printf("CRC32 Verification\n");
    tSblStatus retCode = SBL_SUCCESS;
    
    bool bSuccess = false;
    char pcPayload[12];
    uint32_t ui32RecvCount = 1;

    /* Check input arguments */
    if(!addressInFlash(ui32StartAddress, ui32ByteCount) &&
            !addressInRam(ui32StartAddress, ui32ByteCount))
    {
        printf("Specified address range (0x%08X + %d bytes) is not in device FLASH nor RAM.\n", ui32StartAddress, ui32ByteCount);
        return (SBL_ARGUMENT_ERROR);
    }
    if(get_filed() < 0)
        return (SBL_PORT_ERROR);
    /*
     Build payload
     - 4B address (MSB first)
     - 4B byte count(MSB first)
    */
    
    printf("StartAddress: 0x%x\n",ui32StartAddress); 
    printf("Byte Count: 0x%x\n",ui32ByteCount); 
    
    ulToCharArray(ui32StartAddress, (uint8_t*)&pcPayload[0]);
    ulToCharArray(ui32ByteCount, (uint8_t*)&pcPayload[4]);
    ulToCharArray(*pui32Crc, (uint8_t*)&pcPayload[8]);
    
    /* Send command */
    if((retCode = sendCmd(CMD_CRC32, (const uint8_t*)pcPayload, 12)) != SBL_SUCCESS){
        return (retCode);
        printf("Sending CRC32 CMD failed.\n");
        }

    /* Receive command response (ACK/NAK) */
    if((retCode = getCmdResponse(&bSuccess, 5)) != SBL_SUCCESS){
     	printf("Device NAKed CRC32 command.\n");
        return (retCode);
}
    if((retCode=sendCmd(CMD_GET_STATUS,NULL,0))!=SBL_SUCCESS)
    {
    printf("Sending GET_STATUS_CMD failed\n");
    return(SBL_ERROR);
    }
    /* Receive command response (ACK/NAK) */
    if((retCode = getCmdResponse(&bSuccess, 5)) != SBL_SUCCESS){
    	 printf("Device NAKed Status CMD command\n");
        return (retCode);}
    
     
    if(retCode = getResponseData((uint8_t*)pcPayload,&ui32RecvCount,0) != SBL_SUCCESS){
    	sendCmdResponse(false);
        return (retCode);}
    sendCmdResponse(true); 
    if(pcPayload[0]==0x40){
    	printf("CRC32-Matched \n");	
    	/* Set progress */
     	printf("+-----------------------------------\n");
     	return (SBL_SUCCESS);}
     else if (pcPayload[0]==0x43){
     	printf("Address Error\n");
    	printf("Please check if .bin file matches sector size (2048 Bytes)\n");
    	printf("+-----------------------------------\n");
    	return(SBL_ERROR);}
     else {
    	printf("CRC-Missmatch!!\n");
    	printf("+-----------------------------------\n");
    	return(SBL_ERROR);}

}

/****************************************************************
 * Function Name : detectAutoBaud
 * Description   : Detect the baud rate
 * Returns       : SBL_SUCCESS ...
 * Params        : None.
 ****************************************************************/
tSblStatus detectAutoBaud(void)
{
    uint8_t wrPkt[2] = {0x55, 0x55};
    uint8_t rdPkt[2] = {0, 0};
    if(serialWrite(wrPkt, 2) != 2)
        return (SBL_ERROR);

    if(serialRead(rdPkt, 2) != 2)
        return(SBL_ERROR);

    if(rdPkt[0] == 0x00 && rdPkt[1] == 0xCC)
      {
          printf("Auto baud detected 0x%02X 0x%02X.\n", rdPkt[0], rdPkt[1]);
          return (SBL_SUCCESS);
      }
      else if(rdPkt[0] == 0x00 && rdPkt[1] == 0x33)
      {
          printf("NACK received 0x%02X 0x%02X.\n", rdPkt[0], rdPkt[1]);
          return (SBL_SUCCESS);
      }
      else
      {
          printf("ACK/NAK not received. Expected 0x00 0xCC or 0x00 0x33, received 0x%02X 0x%02X.\n", rdPkt[0], rdPkt[1]);
          return (SBL_ERROR);
      }
}

tSblStatus chipErase()
{
    int retCode= SBL_SUCCESS;
    bool bSuccess=false;

    if(get_filed() < 0){
        return SBL_PORT_ERROR;
    }
       if((retCode = sendCmd(CMD_CHIP_ERASE, NULL, 0)) != SBL_SUCCESS)
    {
        return retCode;
    }
        
    /*
     Receive command response (ACK/NAK)
    */
     if((retCode = getCmdResponse(&bSuccess, 0)) != SBL_SUCCESS)
            return (retCode);
    /*send command*/
    if(!bSuccess){
          printf("Chip Erase command NAKed by device");
          return (SBL_ERROR);
    }
    return retCode;
}
