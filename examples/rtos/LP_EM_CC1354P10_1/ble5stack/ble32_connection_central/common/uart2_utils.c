/******************************************************************************

@file  uart2_utils.c

Group: WCS, BTS
Target Device: cc13xx cc26xx

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
*****************************************************************************/

// *****************************************************************************
// uart_utils.c
// *****************************************************************************
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#if defined ( USE_WIFI_RADIO )
#include "lwip/sockets.h"
#endif
#if defined ( USE_BLE_RADIO )
#include <ti/sysbios/knl/Mailbox.h>
#endif

#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/display/Display.h>
#include <ti/drivers/SDFatFS.h>
#include <third_party/fatfs/ffcio.h>

#include <semaphore.h>

#include "ti_drivers_config.h"

#include "define.h"
#include "structs.h"
#include "externs.h"
#include "proto.h"

UART2_Handle OpenUARTChannel( uint8_t UARTIndex )
{
    UART2_Handle hUART;
    UART2_Params uartParams;

// Create a UART with data processing off.
    UART2_Params_init( &uartParams );
    uartParams.readMode = UART2_Mode_BLOCKING; //UART_MODE_BLOCKING;
//    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.writeMode = UART2_Mode_BLOCKING; //UART_MODE_BLOCKING;
//    uartParams.readTimeout = BIOS_WAIT_FOREVER;
//    uartParams.writeTimeout = BIOS_WAIT_FOREVER;
    uartParams.readCallback = NULL;
//    uartParams.readCallback = transportReadCb;
    uartParams.writeCallback = NULL;
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL; //UART_RETURN_FULL;
//    uartParams.writeDataMode = UART_DATA_BINARY;
//    uartParams.readDataMode = UART_DATA_BINARY;
//    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
//    uartParams.baudRate = 230400;
//    uartParams.baudRate = 460800;
//    uartParams.baudRate = 921600;
    uartParams.dataLength = UART2_DataLen_8; //UART_LEN_8;
    uartParams.stopBits = UART2_StopBits_1; //UART_STOP_ONE;
    uartParams.parityType = UART2_Parity_NONE; //UART_PAR_NONE;
//    uartParams.parityType = UART_PAR_EVEN;

    hUART = UART2_open( UARTIndex, &uartParams );
//    UART_control(uartHandle, UART_CMD_RXDISABLE, NULL);
    return hUART;
}

UART2_Handle OpenUARTChannelAtBaudRate( uint8_t UARTIndex, uint32_t BaudRate )
{
    UART2_Handle hUART;
    UART2_Params uartParams;

// Create a UART with data processing off.
    UART2_Params_init( &uartParams );
    uartParams.readMode = UART2_Mode_BLOCKING; //UART_MODE_BLOCKING;
//    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.writeMode = UART2_Mode_BLOCKING; //UART_MODE_BLOCKING;
//    uartParams.readTimeout = BIOS_WAIT_FOREVER;
//    uartParams.writeTimeout = BIOS_WAIT_FOREVER;
    uartParams.readCallback = NULL;
//    uartParams.readCallback = transportReadCb;
    uartParams.writeCallback = NULL;
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL; //UART_RETURN_FULL;
//    uartParams.readReturnMode = UART2_ReadReturnMode_PARTIAL; //UART_RETURN_FULL;
//    uartParams.writeDataMode = UART_DATA_BINARY;
//    uartParams.readDataMode = UART_DATA_BINARY;
//    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = BaudRate;
//    uartParams.baudRate = 230400;
//    uartParams.baudRate = 460800;
//    uartParams.baudRate = 921600;
    uartParams.dataLength = UART2_DataLen_8; //UART_LEN_8;
    uartParams.stopBits = UART2_StopBits_1; //UART_STOP_ONE;
    uartParams.parityType = UART2_Parity_NONE; //UART_PAR_NONE;
//    uartParams.parityType = UART_PAR_EVEN;

    hUART = UART2_open( UARTIndex, &uartParams );
//    UART_control(uartHandle, UART_CMD_RXDISABLE, NULL);
    return hUART;
}

// *****************************************************************************
// end of file
// *****************************************************************************
