/******************************************************************************

@file  ThreadBLE_Message.c

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
// ThreadBLE_Message.c
// *****************************************************************************
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <file.h>

/* POSIX Header files */
#include <pthread.h>

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>
#include <ti/display/Display.h>
#include <ti/drivers/UART2.h>
#include <ti/display/DisplayUart2.h>
#define     DisplayUart_Handle      DisplayUart2_Handle
#include <ti/display/DisplayExt.h>
#include <ti/display/AnsiColor.h>

#include "ti_drivers_config.h"
#include "ble_user_config.h"

#include "define.h"
#include "structs.h"
#include "externs.h"
#include "proto.h"

extern uint8_t numConn;

#define SSSC_EVT_SEND_PACKET                    0x08
uint8_t SimpleSerialSocketClient_enqueueMsg(uint8_t event,
                                                   uint8_t status,
                                                   uint8_t *pData,
                                                   uint16_t arg0);

#define MAX_NUMBER_OF_SPI_MESSAGE_BUFFERS             4
uint8_t SPI_MessageBuffer[ MAX_NUMBER_OF_SPI_MESSAGE_BUFFERS ][ SPI_TRANSFER_SIZE ];

#define MAX_NUMBER_OF_MSG_BUFFERS   8
uint8_t MsgCmdBuffer[ MAX_NUMBER_OF_MSG_BUFFERS ][ SPI_TRANSFER_SIZE + 16 ];
uint32_t MsgCmdBufferIndex = 0;

uint8_t MsgDataBuffer[ SPI_TRANSFER_SIZE + 16 ];

int32_t CreateBLEMessageThread( void )
{
    int32_t Status;
    pthread_t thread;
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    Mailbox_Params mailboxParams;
    Error_Block eb;

// *****************************************************************************
// Create Message Mailbox
// *****************************************************************************
    Mailbox_Params_init( &mailboxParams );
    hMailboxMessage = ( Mailbox_Handle )Mailbox_create( sizeof( MsgObj ), 4, &mailboxParams, &eb );

// *****************************************************************************
// Create Message Thread
// *****************************************************************************
    pthread_attr_init( &pAttrs );
    priParam.sched_priority = 1;
    Status = pthread_attr_setdetachstate( &pAttrs, PTHREAD_CREATE_DETACHED );
    if( Status == 0 )
    {
        pthread_attr_setschedparam( &pAttrs, &priParam );
        Status = pthread_attr_setstacksize( &pAttrs, 2560 );
        if( Status == 0 )
        {
            Status = pthread_create( &thread, &pAttrs, BLEMessageThread, NULL );
        }
    }
    return Status;
}

void *BLEMessageThread(void *args)
{
    bool MailboxFlag, ConnectedFlag;
    int32_t ReturnValue;
    uint32_t Index, BytesToSend, CommandByteIndex, MessageBufferReadIndex, TotalBytesToSend, DataBytesToSend;
    uint16_t SendProtocol;
    size_t UARTBytesSent;
    uint8_t *CmdBufferPtr, *DataPtr;
    MsgObj message;
    struct PRINT_LIST *PrintListItemPtr;
    struct BLE_DEVICE_INFO *DeviceInfoPtr;
    uint8_t TempBuffer[ 48 ];
    struct CMD_PACKET_HEADER *CmdPacketHeaderPtr;
    struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr;
    struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr;

    MessageBufferReadIndex = 0;
    BLEInfo.PrintInfo.PrintQueue = Queue_create( NULL, NULL );
    while( hDisplay == NULL )
        Task_sleep( 100 );

//    MsgMessage[ MsgMessageIndex ].Message = CMD_MSG_ADD_CONNECTION;
//    Mailbox_post( hMailboxMessage, &MsgMessage[ MsgMessageIndex++ ], BIOS_NO_WAIT );      // post msg containing LED state into the MAILBOX
//    if( MsgMessageIndex >= ( sizeof( MsgMessage ) / sizeof( MsgObj ) ) )
//        MsgMessageIndex = 0;

    Display_printf( hDisplay, 0, 0, "BLEMessageThread: Running main loop" );
// ******************************************************************************
// Main BLE Message Loop
// ******************************************************************************
    while( TRUE )
    {
        MailboxFlag = Mailbox_pend( hMailboxMessage, &message, BIOS_WAIT_FOREVER );
        if( MailboxFlag > 0 )
        {
            switch( message.Message )
            {
                case CMD_MSG_ADD_CONNECTION:
                    Display_printf( hDisplay, 0, 0, "Added device/handle = #%d:0x%x", message.ShortWord1, message.ShortWord2 );
                    break;
                case CMD_MSG_REMOVE_CONNECTION:
                    Display_printf( hDisplay, 0, 0, "Removed device/handle = #%d:0x%x", message.ShortWord1, message.ShortWord2 );
                    break;
                case CMD_MSG_SEND_MESSAGE:
                    CmdBufferPtr = ( uint8_t * )message.FramePtr;
                    BytesToSend = message.FrameBytes;
#if defined( USE_UART_CMD )
                    ReturnValue = UART2_write( hUARTCommand, ( void * )CmdBufferPtr, BytesToSend, &UARTBytesSent );
                    UARTCommandInfo.PacketsSent++;
#elif defined( USE_HOST_OVER_SPI )
                    SendHostCommandData( ( struct CMD_PACKET_DATAGRAM * )CmdBufferPtr, BytesToSend, NET_PROTO_SERIAL_PORT );
#endif
                    break;
                case CMD_MSG_SEND_GATT_DATA:
// ******************************************************************************
// Send 32 channels of 64 bytes per channel = 2048 bytes
// ******************************************************************************
                    TotalBytesToSend = MAX_NUMBER_OF_BLE_DEVICES * 64; //sizeof( BLECentralInfo.DeviceArray );
                    DataPtr = ( uint8_t * )message.FramePtr; //BLECentralInfo.DeviceArray;
                    Index = 0;
                    while( TotalBytesToSend > 0 )
                    {
                        if( TotalBytesToSend > 256 ) //SPI_TRANSFER_SIZE )
                            DataBytesToSend = 256; //SPI_TRANSFER_SIZE;
                        else
                            DataBytesToSend = TotalBytesToSend;
                        CmdBufferPtr = ( uint8_t * )MsgCmdBuffer[ MsgCmdBufferIndex++ ];
                        MsgCmdBufferIndex %= MAX_NUMBER_OF_MSG_BUFFERS;

                        BytesToSend = CreateMessageWithBuffer( CMD_BLE_CONTROL, CMD_BLE_32X_GATT_DATA_BLOCK, 0, ( uint8_t * )CmdBufferPtr, ( uint8_t * )DataPtr, DataBytesToSend, NET_PROTO_UDP );
                        DataPtr += DataBytesToSend;
                        TotalBytesToSend -= DataBytesToSend;
                        Index++;

                        CmdPacketHeaderPtr = ( struct CMD_PACKET_HEADER * )CmdBufferPtr;
                        if( Index == 1 )
                            CmdPacketHeaderPtr->StartOfFrameFlag = TRUE;
                        if( TotalBytesToSend == 0 )
                            CmdPacketHeaderPtr->EndOfFrameFlag = TRUE;

#if defined( USE_HOST_OVER_SPI )
                        SendHostCommandData( ( struct CMD_PACKET_DATAGRAM * )CmdBufferPtr, BytesToSend, NET_PROTO_SERIAL_PORT );
#endif
                        Task_sleep( 400 );
                    }
// ******************************************************************************
// Send Latency & RSSI
// ******************************************************************************
                    DataPtr = MsgDataBuffer;
                    DataBytesToSend = sizeof( BLECentralInfo.Latency );
                    memcpy( DataPtr, &BLECentralInfo.Latency, DataBytesToSend );
                    DataPtr += DataBytesToSend;
                    for( Index = 0; Index < 32; Index++ )
                    {
                        DataPtr[ Index ] = ( int8_t )BLECentralInfo.DeviceInfo[ Index ].RSSI;
                    }
                    DataBytesToSend += 32;
                    DataPtr = MsgDataBuffer;
                    CmdBufferPtr = ( uint8_t * )MsgCmdBuffer[ MsgCmdBufferIndex++ ];
                    MsgCmdBufferIndex %= MAX_NUMBER_OF_MSG_BUFFERS;
                    BytesToSend = CreateMessageWithBuffer( CMD_BLE_CONTROL, CMD_BLE_32X_LATENCY_RSSI, 0, ( uint8_t * )CmdBufferPtr, ( uint8_t * )DataPtr, DataBytesToSend, NET_PROTO_UDP );
#if defined( USE_HOST_OVER_SPI )
                    SendHostCommandData( ( struct CMD_PACKET_DATAGRAM * )CmdBufferPtr, BytesToSend, NET_PROTO_SERIAL_PORT );
#endif
                    Task_sleep( 400 );

//                    CmdBufferPtr = ( uint8_t * )MsgCmdBuffer[ MsgCmdBufferIndex++ ];
//                    MsgCmdBufferIndex %= MAX_NUMBER_OF_MSG_BUFFERS;
//                    DataPtr = ( uint8_t * )BLECentralInfo.DeviceArray;
//                    DataBytesToSend = 64;
//                    BytesToSend = CreateMessageWithBuffer( CMD_BLE_CONTROL, CMD_BLE_32X_READ_DATA_BLOCK, 0, ( uint8_t * )CmdBufferPtr, ( uint8_t * )DataPtr, DataBytesToSend, NET_PROTO_UDP );
//                    SendHostCommandData( ( struct CMD_PACKET_DATAGRAM * )CmdBufferPtr, BytesToSend, NET_PROTO_SERIAL_PORT );
                    break;
                case CMD_MSG_CONNECT_UPDATE:
                    DeviceInfoPtr = ( struct BLE_DEVICE_INFO * )message.FramePtr;
                    CommandByteIndex = 0;
                    TempBuffer[ CommandByteIndex++ ] = BLECentralInfo.CurrentJoinedDevices;
                    TempBuffer[ CommandByteIndex++ ] = BLECentralInfo.TotalJoinedCtr;
                    TempBuffer[ CommandByteIndex++ ] = BLECentralInfo.TotalUnjoinedCtr;
                    TempBuffer[ CommandByteIndex++ ] = BLECentralInfo.TotalConnectedDevices;
                    TempBuffer[ CommandByteIndex++ ] = message.FrameBytes;
                    memcpy( &TempBuffer[ CommandByteIndex ], DeviceInfoPtr, sizeof( struct BLE_DEVICE_INFO) );
                    CommandByteIndex += sizeof( struct BLE_DEVICE_INFO );
                    CmdBufferPtr = ( uint8_t * )MsgCmdBuffer[ MsgCmdBufferIndex++ ];
                    MsgCmdBufferIndex %= MAX_NUMBER_OF_MSG_BUFFERS;
#if defined( USE_UART_CMD )
                    BytesToSend = CreateMessageWithBuffer( CMD_BLE_CONTROL, CMD_BLE_32X_CONNECT_RESULT, 0, ( uint8_t * )CmdBufferPtr, ( uint8_t * )TempBuffer, CommandByteIndex, NET_PROTO_SERIAL_PORT );
                    ReturnValue = UART2_write( hUARTCommand, ( void * )MsgCmdBuffer, BytesToSend, &UARTBytesSent );
                    UARTCommandInfo.PacketsSent++;
#elif defined( USE_HOST_OVER_SPI )
                    BytesToSend = CreateMessageWithBuffer( CMD_BLE_CONTROL, CMD_BLE_32X_CONNECT_RESULT, 0, ( uint8_t * )CmdBufferPtr, ( uint8_t * )TempBuffer, CommandByteIndex, NET_PROTO_TCP );
                    SendHostCommandData( ( struct CMD_PACKET_DATAGRAM * )CmdBufferPtr, BytesToSend, NET_PROTO_SERIAL_PORT );
#endif
                    break;
                case CMD_MSG_PRINT_MESSAGE:
                    BLEInfo.PrintInfo.CurrentlyPrintingFlag = TRUE;
                    while( !Queue_empty( BLEInfo.PrintInfo.PrintQueue ) )
                    {
                        PrintListItemPtr = ( struct PRINT_LIST * )Queue_dequeue( BLEInfo.PrintInfo.PrintQueue );
// ******************************************************************************
// Send BLE data to the network
// ******************************************************************************
                        CmdBufferPtr = ( uint8_t * )MsgCmdBuffer[ MsgCmdBufferIndex++ ];
                        MsgCmdBufferIndex %= MAX_NUMBER_OF_MSG_BUFFERS;
                        BytesToSend = CreateMessageWithBuffer( CMD_TEXT_CONTROL, 0, 0, ( uint8_t * )CmdBufferPtr,
                                                               ( uint8_t * )PrintListItemPtr->TextPtr, PrintListItemPtr->Size, NET_PROTO_SERIAL_PORT );
#if defined( USE_UART_CMD )
                        ReturnValue = UART2_write( hUARTCommand, ( void * )CmdBufferPtr, BytesToSend, &UARTBytesSent );
                        UARTCommandInfo.PacketsSent++;
#elif defined( USE_HOST_OVER_SPI )
                        SendHostCommandData( ( struct CMD_PACKET_DATAGRAM * )CmdBufferPtr, BytesToSend, NET_PROTO_SERIAL_PORT );
#endif
// ******************************************************************************
// Display text on console
// ******************************************************************************
                        UART2_write( hUARTConsole, PrintListItemPtr->TextPtr, PrintListItemPtr->Size, &UARTBytesSent );
                    }
                    BLEInfo.PrintInfo.CurrentlyPrintingFlag = FALSE;
                    break;
                case CMD_MSG_PROCESS_SPI_MESSAGE:
                    SrcPacketDatagramPtr = ( struct CMD_PACKET_DATAGRAM * )message.FramePtr;
                    DestPacketDatagramPtr = ( struct CMD_PACKET_DATAGRAM * )SPI_MessageBuffer[ MessageBufferReadIndex++ ];
                    MessageBufferReadIndex %= MAX_NUMBER_OF_SPI_MESSAGE_BUFFERS;
                    SendProtocol = message.ShortWord1;
                    ReturnValue = ProcessCommand( SrcPacketDatagramPtr, DestPacketDatagramPtr, SendProtocol );
                    break;
                default:
                    break;
            }
        }
    }
    return NULL;
}

// *****************************************************************************
// end of file
// *****************************************************************************