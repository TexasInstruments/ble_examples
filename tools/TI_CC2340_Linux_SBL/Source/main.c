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
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <string.h>

/* Custom Includes */
#include "Linux_Serial.h"
#include "sbl_device.h"
#include "sbl_device_cc2340.h"
#include "myFile.h"

/* read only variables */
const char *portName = NULL;
const char *filename = NULL; /*Path of .bin to be flashed*/
const char *ccfgFilename= NULL;
const char *command= NULL;
static FILE *fPtr = NULL;  
static FILE *CCFGfPtr= NULL; 

/* CMD line cases */
enum cmdArgs{
    ONE = 1,
    TWO = 2,
    THREE = 3,
    FOUR =4,
};


int main(int argc, char **argv)
{
    printf("\n+-----------------------------------\n");
    printf("Serial Bootloader Library for CC2340\n");
    printf("Platform: Linux                     \n");
    printf("Compiler: GCC                       \n");
    printf("ARG0 = %s\n",argv[0]);
    printf("ARG1 = %s\n",argv[1]);
    printf("ARG2 = %s\n",argv[2]);
    printf("ARG3 = %s\n",argv[3]);
    printf("+-----------------------------------\n");

    /* Do some initial command line checks */
    switch(argc)
    {
    case FOUR:
        portName = argv[1];
        filename = argv[2];
        ccfgFilename=argv[3];
        printf("SBL Port i/p: %s\r\n", portName);
        printf("Firmware i/p: %s\r\n\n", filename);
        printf("CCFG Filename i/p: %s\r\n\n", ccfgFilename);
        printf("All Good :)\r\n");
        break;
    case TWO:
        if (strcmp(argv[1],"-help")==0){
			printf("CC2340 -Linux Bootloader- HELP Overview\n");
			printf("The function SBL.out take a total number of 3 Arguments:\n");
			printf("To call the function use: \n");
			printf("SBL.out [Serial Port] [Firmware] [CCFG]\r\n");
			printf("[Serial Port]...name of serial port connected to CC2340\r\n");
			printf("[Firmware]......Path of Firmware binary file\r\n");
			printf("[CCFG]..........Path of CCFG binary file\r\n");
			printf("+-----------------------------------\n");
			exit(EXIT_SUCCESS);
			break;}
    default:
        printf("INVALID ARG'S...EXISTING :(\r\n");
        printf("Valid format of running the SBL is:\r\n");
        printf("SBL.out [Serial Port] [Firmware Image path] [CCFG filepath]\r\n");
        printf("Type: SBL.out -help to get more information\r\n");
        printf("+-----------------------------------\n");
        exit(EXIT_FAILURE);
        break;
    }

    /*Start Bootloader 
    Initialize system*/
    printf("\n+-----------------------------------\n");
    uint8_t *memPtr = NULL;                 /* Ptr to hold read data */
    uint8_t *ccfgmemPtr = NULL;                 /* Ptr to hold read data */
    long fileSz = 0;                        /* Holds the size of the .bin file, in bytes */
    uint32_t fileCrc,ccfgCrc;       /* Variables to save CRC checksum */
    uint32_t tmp = 0;
    bool ackChk = false;

    uint32_t devCcfgBase=SBL_CC2340_CCFG_START_ADDRESS;      /*CFG start addrass only used for CC2340*/ 
    uint32_t byteCount=2048;       /*file size in bytes*/
    uint32_t ccfgByteCount=0;   /*file size in bytes for CC2340 CCFG*/
    uint32_t devFlashBase;   /*device Flash base*/
    printf("DevCcfgBase: 0x%x\r\n", devCcfgBase);
    printf("DevFlashBase: 0x%x\r\n", devFlashBase);
    uint32_t u32tRuntime_start,u32tRuntime_stop;
    u32tRuntime_start=time(NULL);
    
    printf("\n+-----------------------------------\n");
    /* Open the port */
    openPort(portName);

    /* Configure port */
    configPort();
    printf("+-----------------------------------\n");

    /* Setup callbacks */
    setupCallbacks();

    /* Set flash base for cc2340 */
    setDeviceFlashBase(CC23XX_FLASH_BASE);

    /* Detect baud rate */
    if(detectAutoBaud() != SBL_SUCCESS)
    {
        printf("ERROR: baud detect  failed\n");
        closePort();
        exit(EXIT_FAILURE);
    }
    else
        printf("Baudrate detected !\n");

    /* Check if the host is reachable */
    if(ping() != SBL_SUCCESS)
    {
        printf("ERROR: Host unreachable\n");
        closePort();
        exit(EXIT_FAILURE);
    }
    else
        printf("PING: Host detected !\n");

    if(readFlashSize(&tmp) != SBL_SUCCESS)
    {
        printf("ERROR: Unable to read flash size\n");
        closePort();
        exit(EXIT_FAILURE);
    }
    else
        printf("Flash size: %u\n",getFlashSize());

    if(readRamSize(&tmp) != SBL_SUCCESS)
    {
        printf("ERROR: Unable to read RAM size\n");
        closePort();
        exit(EXIT_FAILURE);
    }
    else
        printf("RAM size: %u\n",getRamSize());

    /* Open the file */
    if((fPtr = openFile(filename)) == NULL)
    {
        printf("ERROR: opening file\n");
        closePort();
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("FILE open OK\n");
        if(!(fileSz = getFileSize(fPtr)))
        {
            printf("ERROR: getting file size\n");
            closePort();
            closeFile(fPtr);
            exit(EXIT_FAILURE);
        }
        else
        {
            printf("FILE size OK,\nFile Size: %lu Byte\n",fileSz);

            /* Allocate memory to read the file */
            memPtr = (uint8_t*)calloc(fileSz, sizeof(uint8_t));
            if(memPtr)
            {
                uint32_t fileRd = 0;

                printf(" CALLOC OK\n");
                if((fileRd = fread(memPtr,1 , fileSz, fPtr)) != fileSz)
                {
                    printf("ERROR: File read failed\n");
                    free(memPtr);
                    closePort();
                    closeFile(fPtr);
                    exit(EXIT_FAILURE);
                }
                else
                    printf("FILE read OK,\n file Read: %u Bytes\n", fileRd);
            }
            else
            {
                printf("ERROR: calloc failed\n");
                closePort();
                closeFile(fPtr);
                exit(EXIT_FAILURE);
            }
        }
    }/*End of all file operations*/ 
printf("+-----------------------------------\n");
/* Calculate file CRC checksum */
fileCrc = calcCrcLikeChip(memPtr, fileSz);

/*CCFG File Regions:
ccfg.bootCfg.crc32          0x0000-0x000B
ccfg.crc32                  0x0010-0x074B
ccfg.userRecord.crc32       0x0750-0x07FB*/
   
uint8_t Crc32Value[4];
uint16_t CcfgRegionStart[4] =   {0x0000, 0x0010,0x075,0x07D0};
uint16_t CcfgRegionEnd[4]=      {0x00B, 0x074, 0x0CB, 0x07FB};
uint16_t regionStartAddr;
uint16_t regionEndAddr;
uint8_t addrIndex;

if((CCFGfPtr = openFile(ccfgFilename)) == NULL)
    {
        printf("ERROR: opening file\n");
        closePort();
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("FILE open OK\n");
        printf("+-----------------------------------\n");
        printf("Start Updating CCFG....\n");

        /*Update CRC32 for 4 regions in CCFG*/
        for(addrIndex=0;addrIndex<4;addrIndex++){
            regionStartAddr=CcfgRegionStart[addrIndex];
            regionEndAddr=CcfgRegionEnd[addrIndex];

            /*set Read pointer to the beginning*/
            fseek(CCFGfPtr,regionStartAddr,SEEK_SET);

            /*read datra from region start to region end*/
            ccfgByteCount=regionEndAddr-regionStartAddr+1;
        
            /* Allocate memory to read the file */
            ccfgmemPtr = (uint8_t*)calloc(ccfgByteCount, sizeof(uint8_t));
            printf("memory allocated");
            if(ccfgmemPtr)
            {
                uint32_t fileRd = 0;
                printf("CALLOC OK\n");
                if((fileRd = fread(ccfgmemPtr,1 , ccfgByteCount, CCFGfPtr)) != ccfgByteCount)
                {
                    printf("ERROR: File read failed\n");
                    free(ccfgmemPtr);
                    closePort();
                    closeFile(CCFGfPtr);
                    exit(EXIT_FAILURE);
                }
                else
                    printf("FILE read OK, fileRd: %u \n", fileRd);
            }
            else
            {
                printf("ERROR: calloc failed\n");
                closePort();
                closeFile(CCFGfPtr);
                exit(EXIT_FAILURE);
            }
            /*calculate CRC checksum*/
            uint32_t ccfgfileCrc=calcCrcLikeChip(ccfgmemPtr,ccfgByteCount);
            printf("CRC32 for 0x%x to 0x%x is %x\n",regionStartAddr,regionEndAddr,ccfgfileCrc);
            Crc32Value[0] = (uint8_t)   (ccfgfileCrc&0x000000FF);
            Crc32Value[1] = (uint8_t)   ((ccfgfileCrc&0x0000FF00)>>8);
            Crc32Value[2] = (uint8_t)   ((ccfgfileCrc&0x00FF0000)>>16);
            Crc32Value[3] = (uint8_t)   ((ccfgfileCrc&0xFF000000)>>24);

            printf("CRC32Value= [0x%x,0x%x,0x%x,0x%x]\n",Crc32Value[0],Crc32Value[1],Crc32Value[2],Crc32Value[3]);

            /*Set write pointe to Byte of region end*/
            fseek(CCFGfPtr,regionEndAddr,SEEK_SET);
            /*Add data to region End*/
            fwrite(&Crc32Value[0],1,4,CCFGfPtr);
            printf("***CRC32 written to file address  %x to %x\n",regionStartAddr,regionEndAddr);
        } /*end of for loop*/
        
        /*Get Overall file size*/
        ccfgByteCount = getFileSize(CCFGfPtr); 
        uint32_t fileRd = 0;
        if((fileRd = fread(ccfgmemPtr,1 , ccfgByteCount, CCFGfPtr)) != ccfgByteCount)
                {
                printf("ERROR: Erase failed\n");
                free(memPtr);
                free(ccfgmemPtr);
                closePort();
                closeFile(CCFGfPtr);
                closeFile(fPtr);
                exit(EXIT_FAILURE);
                }
                else{
                    printf("FILE read OK, fileRd: %u \n", fileRd);
                    ccfgCrc=calcCrcLikeChip(ccfgmemPtr,ccfgByteCount);                    
                    }
    closeFile(CCFGfPtr);        
    }/* End of all file operations*/
    printf("+-----------------------------------\n");

    /* Erase flash */
    printf("Erasing flash ...\n");
    if(chipErase()!= SBL_SUCCESS)
    {
        printf("ERROR: Erase failed\n");
        free(memPtr);
        free(ccfgmemPtr);
        closePort();
        closeFile(CCFGfPtr);
        closeFile(fPtr);
        exit(EXIT_FAILURE);
    }
    else
        printf("ERASE OK\n");


    /* Write file to device flash memory */
    printf("+-----------------------------------\n");
    printf("Writing flash ...\n");
    if(writeFlashRange(getDeviceFlashBase(), fileSz, (char*)memPtr) != SBL_SUCCESS)
    {
        printf("ERROR: writing failed\n");
        free(memPtr);
        free(ccfgmemPtr);
        closePort();
        closeFile(CCFGfPtr);
        closeFile(fPtr);
        exit(EXIT_FAILURE);
    }
    else
        printf("\nWRITE OK\n");

     /* Write ccfg file to device flash memory */
    printf("+-----------------------------------\n");
    printf("Writing ccfg ...\n");
    if(writeFlashRange(devCcfgBase, ccfgByteCount, (char*)ccfgmemPtr) != SBL_SUCCESS)
    {
        printf("ERROR: Writing failed\n");
        free(memPtr);
        free(ccfgmemPtr);
        closePort();
        closeFile(CCFGfPtr);
        closeFile(fPtr);
        exit(EXIT_FAILURE);
    }
    else
        printf("WRITE OK\n");  

    /* Calculate CRC checksum of flashed content */
    printf("Calculating CRC of flashed content ...\n");
    if(calculateCrc32(getDeviceFlashBase(), fileSz, &fileCrc) != SBL_SUCCESS)
    {
        printf("ERROR: CRC failed\n");
        free(memPtr);
        closePort();
        closeFile(fPtr);
        exit(EXIT_FAILURE);
    }
    printf("Calculating CRC of CCFG content ...\n");
    if(calculateCrc32(devCcfgBase, ccfgByteCount, &ccfgCrc) != SBL_SUCCESS)
    {
        printf("ERROR: CRC failed\n");
        free(memPtr);
        closePort();
        closeFile(fPtr);
        exit(EXIT_FAILURE);
    }

    /* Reset the device */
    if(reset() != SBL_SUCCESS)
    {
        printf("ERROR: RST failed!\n");
        printf("ERROR: Erase failed\n");
        free(memPtr);
        free(ccfgmemPtr);
        closePort();
        closeFile(CCFGfPtr);
        closeFile(fPtr);
        exit(EXIT_FAILURE);
    }
    else
        printf("RST OK\n");
    u32tRuntime_stop=time(NULL);
    printf("Runtime: %i Seconds\n",u32tRuntime_stop-u32tRuntime_start);	
    /* If we got here, means all succeeded */
    printf("+-----------------------------------\n");
    printf("CC2340 FIRMWARE UPGRADE COMPLETED !-\n");
    printf("+-----------------------------------\n\n");

    /* De-allocate the memory */
    free(memPtr);

    /* Close all */
    closeFile(fPtr);
    free(ccfgmemPtr);
    closePort();

    /* exit on success */
    exit(EXIT_SUCCESS);
}
