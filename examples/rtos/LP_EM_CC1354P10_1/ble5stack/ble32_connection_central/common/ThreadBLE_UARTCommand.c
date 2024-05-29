/******************************************************************************

@file  ThreadBLE_UARTCommand.c

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
// ThreadBLE_UARTCommand.c
// *****************************************************************************
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

// XDCtools Header files
#include <xdc/std.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/display/Display.h>

#include "ti_drivers_config.h"

#include "define.h"
#include "structs.h"
#include "externs.h"
#include "proto.h"

uint32_t CreateMessageWithBuffer( uint8_t Command, uint8_t SubCommand, uint8_t DestinationAddress, uint8_t *CmdPtr, uint8_t *SrcPtr, uint32_t BytesToTransfer, uint8_t SendProtocol )
{
    struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr;
    struct CMD_PACKET_HEADER *CmdPacketHeaderPtr;

    CmdPacketDatagramPtr = ( struct CMD_PACKET_DATAGRAM * )CmdPtr;
    CmdPacketHeaderPtr = (struct CMD_PACKET_HEADER *)&CmdPacketDatagramPtr->Header;
    memset( CmdPacketHeaderPtr, 0, sizeof( struct CMD_PACKET_HEADER ) );
    CmdPacketHeaderPtr->SyncByte = PACKET_SYNC_CODE;
    CmdPacketHeaderPtr->Command = Command;
    CmdPacketHeaderPtr->SubCommand = SubCommand;
    CmdPacketHeaderPtr->SourceAddress = 0; //RadioDeviceInfo.MyBoard.ID;
    CmdPacketHeaderPtr->DestinationAddress = DestinationAddress;
    CmdPacketHeaderPtr->SendProtocol = SendProtocol;
    if( ( SrcPtr != NULL ) && ( BytesToTransfer > 0 ) )
    {
        memcpy( ( uint8_t * )CmdPacketDatagramPtr->Payload, SrcPtr, BytesToTransfer );
        CmdPacketHeaderPtr->PayloadBytes = BytesToTransfer;
    }
    return sizeof( struct CMD_PACKET_HEADER ) + CmdPacketHeaderPtr->PayloadBytes;
}

uint32_t CreateMessageOnly( uint8_t Command, uint8_t SubCommand, uint8_t DestinationAddress, uint8_t *CmdPtr, uint8_t SendProtocol )
{
    struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr;
    struct CMD_PACKET_HEADER *CmdPacketHeaderPtr;

    CmdPacketDatagramPtr = ( struct CMD_PACKET_DATAGRAM * )CmdPtr;
    CmdPacketHeaderPtr = (struct CMD_PACKET_HEADER *)&CmdPacketDatagramPtr->Header;
    memset( CmdPacketHeaderPtr, 0, sizeof( struct CMD_PACKET_HEADER ) );
    CmdPacketHeaderPtr->SyncByte = PACKET_SYNC_CODE;
    CmdPacketHeaderPtr->Command = Command;
    CmdPacketHeaderPtr->SubCommand = SubCommand;
    CmdPacketHeaderPtr->SourceAddress = 0; //RadioDeviceInfo.MyBoard.ID;
    CmdPacketHeaderPtr->DestinationAddress = DestinationAddress;
    CmdPacketHeaderPtr->SendProtocol = SendProtocol;
    return sizeof( struct CMD_PACKET_HEADER );
}

uint32_t AppendMessageWithBuffer( uint8_t *CmdPtr, uint8_t *SrcPtr, uint32_t BytesToTransfer )
{
    struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr;
    struct CMD_PACKET_HEADER *CmdPacketHeaderPtr;

    if( ( SrcPtr != NULL ) && ( BytesToTransfer > 0 ) )
    {
        CmdPacketDatagramPtr = ( struct CMD_PACKET_DATAGRAM * )CmdPtr;
        CmdPacketHeaderPtr = (struct CMD_PACKET_HEADER *)&CmdPacketDatagramPtr->Header;
        memcpy( ( uint8_t * )&CmdPacketDatagramPtr->Payload[ CmdPacketHeaderPtr->PayloadBytes ], SrcPtr, BytesToTransfer );
        CmdPacketHeaderPtr->PayloadBytes += BytesToTransfer;
    }
    return sizeof( struct CMD_PACKET_HEADER ) + CmdPacketHeaderPtr->PayloadBytes;
}

#if defined ( USE_UART_CMD )
int32_t CreateUARTCommandThread( void )
{
    int32_t Status;
    pthread_t thread;
    pthread_attr_t pAttrs;
    struct sched_param priParam;

// *****************************************************************************
// Create Message Thread
// *****************************************************************************
    pthread_attr_init( &pAttrs );
    priParam.sched_priority = 1;
    Status = pthread_attr_setdetachstate( &pAttrs, PTHREAD_CREATE_DETACHED );
    if( Status == 0 )
    {
        pthread_attr_setschedparam( &pAttrs, &priParam );
        Status = pthread_attr_setstacksize( &pAttrs, 2048 );
        if( Status == 0 )
        {
            Status = pthread_create( &thread, &pAttrs, UARTCommandThread, NULL );
        }
    }
    return Status;
}

void *UARTCommandThread(void *args)
{
    uint16_t BufferSize, ProcessedBytes, PayloadBytes, Length;
    size_t BytesRead, BytesSent;
    int32_t Result;
    uint32_t Index, BytesToSend;
    struct BUFFER_INFO *CommandBufferInfoPtr;
    struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr;
    struct CMD_PACKET_HEADER *CmdPacketHeaderPtr;
    char TextBuffer[ 16 ];
    uint8_t CommandBuffer[ 32 ];

// *****************************************************************************
// Wait on valid UART handle
// *****************************************************************************
    while( hDisplay == NULL )
        Task_sleep( 100 );

    Display_printf( hDisplay, 0, 0, "UARTCommandThread: Starting main loop" );
// *****************************************************************************
// Open GPS UART for LS20031 GPS Module
// *****************************************************************************
    hUARTCommand = OpenUARTChannelAtBaudRate( CONFIG_UART1_AUX, UART_CMD_BAUD_RATE );
    if( hUARTCommand != 0 )
    {
        UART2_rxEnable( hUARTCommand );
        Display_printf( hDisplay, 0, 0, "UARTCommandThread: UART successfully opened" );
    }

    CommandBufferInfoPtr = ( struct BUFFER_INFO * )&UARTCommandInfo.CommandBufferInfo;
    memset( CommandBufferInfoPtr, 0, sizeof( struct BUFFER_INFO ) );
    UARTCommandInfo.State = COMMAND_STATE_SYNC;
    UARTCommandInfo.BytesToRead = 1;
    AllocateBufferInfo( CommandBufferInfoPtr, UART_BUFFER_SIZE, 32 );

    sprintf( TextBuffer, "Hey WiFi\n" );
    Length = strlen( TextBuffer );
    BytesToSend = CreateMessageWithBuffer( CMD_TEXT_CONTROL, 0, 0, ( uint8_t * )CommandBuffer, ( uint8_t * )TextBuffer, Length, NET_PROTO_SERIAL_PORT );
    Result = UART2_write( hUARTCommand, ( void * )CommandBuffer, BytesToSend, &BytesSent );
    UARTCommandInfo.PacketsSent++;

//    Task_sleep( 10000 );
//    Result = UART2_write( hUARTCommand, ( void * )CommandBuffer, BytesToSend, &BytesSent );
//    UARTCommandInfo.PacketsSent++;

// *****************************************************************************
// UART Command Main Loop
// *****************************************************************************
    UARTCommandInfo.WriteIndex = UARTCommandInfo.ReadIndex = UARTCommandInfo.BufferLevel = 0;
    Display_printf( hDisplay, 0, 0, "UARTCommandThread: Running main loop" );
    while( TRUE )
    {
        BytesRead = 0;
        Result = UART2_read( hUARTCommand, &UARTCommandInfo.Buffer[ 0 ], UARTCommandInfo.BytesToRead, &BytesRead );
        if( BytesRead > 0 )
        {
            for( Index = 0; Index < BytesRead; Index++ )
            {
                *CommandBufferInfoPtr->WritePtr++ = UARTCommandInfo.Buffer[ Index ];
                if( CommandBufferInfoPtr->WritePtr >= CommandBufferInfoPtr->TopPtr )
                {
                    CommandBufferInfoPtr->WritePtr = CommandBufferInfoPtr->BeginPtr;
                    CommandBufferInfoPtr->WrapValue = CommandBufferInfoPtr->WriteBufferSize;
                }
            }
            CommandBufferInfoPtr->TotalBytesWritten += BytesRead;
            CommandBufferInfoPtr->BufferLevel += BytesRead;
PROCESS_STATE:
            switch( UARTCommandInfo.State )
            {
                case COMMAND_STATE_SYNC:
                    if( CommandBufferInfoPtr->ReadPtr[ 0 ] == PACKET_SYNC_CODE )
                    {
                        UARTCommandInfo.State = COMMAND_STATE_HEADER;
                        UARTCommandInfo.BytesToRead = sizeof( struct CMD_PACKET_HEADER ) - 1;
                    }
                    else
                        AdvanceBufferReadPtr( ( struct BUFFER_INFO * )CommandBufferInfoPtr, 1 );
                    break;
                case COMMAND_STATE_HEADER:
                    if( CommandBufferInfoPtr->BufferLevel >= sizeof( struct CMD_PACKET_HEADER ) )
                     {
                        CmdPacketDatagramPtr = ( struct CMD_PACKET_DATAGRAM * )GetContiguousReadPtr(
                                ( struct BUFFER_INFO * )CommandBufferInfoPtr, sizeof( struct CMD_PACKET_HEADER ) );
                        CmdPacketHeaderPtr = ( struct CMD_PACKET_HEADER * )&CmdPacketDatagramPtr->Header;
                        PayloadBytes = CmdPacketHeaderPtr->PayloadBytes;
                        UARTCommandInfo.State = COMMAND_STATE_PAYLOAD;
                        UARTCommandInfo.BytesToRead = PayloadBytes;
                        if( PayloadBytes == 0 )
                        {
                            UARTCommandInfo.BytesToRead = 1;
                            goto PROCESS_STATE;
                        }
                     }
                    break;
                case COMMAND_STATE_PAYLOAD:
                    if( CommandBufferInfoPtr->BufferLevel >= sizeof( struct CMD_PACKET_HEADER ) + PayloadBytes )
                    {
                        BufferSize = sizeof( struct CMD_PACKET_HEADER ) + PayloadBytes;
                        CmdPacketDatagramPtr = ( struct CMD_PACKET_DATAGRAM * )GetContiguousAuxReadPtr(
                                ( struct BUFFER_INFO * )CommandBufferInfoPtr, UARTCommandInfo.Buffer, BufferSize );
                        CmdPacketHeaderPtr = ( struct CMD_PACKET_HEADER * )&CmdPacketDatagramPtr->Header;
                        ProcessedBytes = CommandBufferInfoPtr->BufferLevel;
                        switch( CmdPacketHeaderPtr->SyncByte )
                        {
                            case PACKET_SYNC_CODE:
                                    BufferSize = sizeof( struct CMD_PACKET_HEADER ) + CmdPacketHeaderPtr->PayloadBytes;
                                    ProcessedBytes = ProcessCommand( ( struct CMD_PACKET_DATAGRAM * )CmdPacketDatagramPtr,
                                                         NULL, NET_PROTO_SERIAL_PORT );
                                    AdvanceBufferReadPtr( ( struct BUFFER_INFO * )CommandBufferInfoPtr, ProcessedBytes );
                                break;
                            default:
                                AdvanceBufferReadPtr( ( struct BUFFER_INFO * )CommandBufferInfoPtr, ProcessedBytes );
                                break;
                        }
                        if( CommandBufferInfoPtr->BufferLevel > 0 )
                        {
                            Display_printf( hDisplay, 0, 0, "Should not happen!" );
                        }
                        UARTCommandInfo.State = COMMAND_STATE_SYNC;
                        UARTCommandInfo.BytesToRead = 1;
                    }
                    break;
                default:
                    Display_printf( hDisplay, 0, 0, "Invalid Command State" );
                    break;
            } // end of switch( UARTCommandInfo.State )
        } // end of if( BytesRead > 0 )
    } // end of while( TRUE )
    return NULL;
}
#endif

// *****************************************************************************
// end of file
// *****************************************************************************
