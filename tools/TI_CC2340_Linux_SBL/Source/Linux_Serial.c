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

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

/* Custom Includes */
#include "Linux_Serial.h"

/* Static variables */
static int fd = 0;
static struct termios SerialPortSettings;

/* Static functions */
static void setBaudRate(bautSet_t baud);

/****************************************************************
 * Function Name : openPort
 * Description   : Opens the serial port
 * Returns       : 0 on success, -1 on failure
 * Params        @port: Path to the serial port
 ****************************************************************/
int openPort(const char *port)
{
    if((fd = open(port, O_RDWR | O_NOCTTY)) < 0)
        perror("USB: ERROR OPENING PORT |");
    else
        printf("USB: PORT OPEN SUCCESSFUL !\r\n");
    return(fd);
}

/****************************************************************
 * Function Name : closePort
 * Description   : Closes the serial port
 * Returns       : 0 on success, -1 on failure
 * Params        @fd: file descriptor
 ****************************************************************/
int closePort()
{
    int rc = 0;
    if((rc = close(fd)) < 0)
        perror("USB: ERROR CLOSING PORT |");
    else
        printf("USB: PORT CLOSED SUCCESSFUL !\r\n");
    return(rc);
}
/****************************************************************
 * Function Name : clearRxbuffer
 * Description   : Discards old data in the buffer
 * Returns       : None
 * Params        @None
 ****************************************************************/
void clearRxbuffer(void)
{
    /* This sleep is required for the flush to work
     * properly, seems like a bug in the kernel. For
     * more info refer to the link below:
     * https://stackoverflow.com/questions/13013387/clearing-the-serial-ports-buffer
     */
    sleep(1);
    tcflush(fd, TCIOFLUSH);
}

/****************************************************************
 * Function Name : setBaudRate
 * Description   : Set the baud rate
 * Returns       : None
 * Params        @baud: Baudrate
 ****************************************************************/
static void setBaudRate(bautSet_t baud)
{
    switch(baud)
    {
    case B_9600:
        /*Set Baudrate to 9600 Baud*/
        cfsetispeed(&SerialPortSettings,B9600);
        cfsetospeed(&SerialPortSettings,B9600);
        printf("\n  BaudRate = 9600\n");
        break;
    case B_115200:
        /*Set Baudrate to 115200 Baud*/
        cfsetispeed(&SerialPortSettings,B115200);
        cfsetospeed(&SerialPortSettings,B115200);
        printf("\n  BaudRate = 115200\n");
        break;
    case B_921600:
        /*Set Baudrate to 921600 Baud*/
        cfsetispeed(&SerialPortSettings,B921600);
        cfsetospeed(&SerialPortSettings,B921600);
        printf("\n  BaudRate = 921600\n");
        break;
    case B_1152000:
        /*Set Baudrate to 1152000*/
        cfsetispeed(&SerialPortSettings,B1152000);
        cfsetospeed(&SerialPortSettings,B1152000);
         printf("\n  BaudRate = 1152000\n");
        break;

    default:
        printf("ERROR: INVALID BAUD\n");
        break;
    }
}

/****************************************************************
 * Function Name : configPort
 * Description   : Populate the termios structure
 * Returns       : None
 * Params        @None
 ****************************************************************/
void configPort(void)
{
     /*Using thermios structure to set Serial Port structure*/
    memset(&SerialPortSettings, 0, sizeof(SerialPortSettings));
    setBaudRate(B_1152000);

    /*Configure serial port message interpretation */
    /* Enable receiver*/ 
    SerialPortSettings.c_cflag |= (CLOCAL | CREAD);

    /* Set Data Size= 8Bit        */
    SerialPortSettings.c_cflag &= ~CSIZE;                           
    SerialPortSettings.c_cflag |= CS8;

    /* No Parity   */
    SerialPortSettings.c_cflag &= ~PARENB;

    /* 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CRTSCTS;

    /* setup for non-canonical mode */
    SerialPortSettings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    SerialPortSettings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    SerialPortSettings.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    SerialPortSettings.c_cc[VMIN] = 0;
    SerialPortSettings.c_cc[VTIME] = 2;

    /* Flush out if there is any previously pending data* */
    clearRxbuffer();

    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0)
        perror("ERROR in Setting attributes |");
    else
        printf("  StopBits = 1 \n  Parity   = none\n");
}

/****************************************************************
 * Function Name : serialWrite
 * Description   : Write bytes on Tx
 * Returns       : Number of bytes written
 * Params        @dataPtr: Pointer to the buffer to be written
 *               @dataLen: Length of the data
 ****************************************************************/
int serialWrite(uint8_t *wrPtr, uint8_t wrDataLen)
{
    int wrbytes = write(fd, wrPtr, wrDataLen);
    /* Wait until entire buffer is written */
    tcdrain(fd);
    return(wrbytes);
}

/****************************************************************
 * Function Name : serialRead
 * Description   : Reads bytes on the RX
 * Returns       : Number of bytes read
 * Params        @dataPtr: Pointer to the buffer to be populated
 *               @dataLen: Length of the data
 ****************************************************************/
int serialRead(uint8_t *rdPtr, uint8_t rdDataLen)
{
    int rdbytes = read(fd, rdPtr, rdDataLen);
    return(rdbytes);
    /* SerialRead should always return valid value!!!*/
    /* Bootloader will runn into issues if this does not run properly*/
}

/****************************************************************
 * Function Name : get_filed
 * Description   : Returns fd
 * Returns       : None
 * Params        @None
 ****************************************************************/
int get_filed(void)
{
    return(fd);
}


