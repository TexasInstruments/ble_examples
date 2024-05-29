/******************************************************************************

@file  ThreadSlaveSPI.c

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
// ThreadSlaveSPI.c
// *****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include <pthread.h>

// BIOS Header files
#include "ti_rtos.h"

//#if defined ( BOARD_ID_MCU_CC1350 ) || defined ( BOARD_ID_MCU_CC1352 )
//typedef int     SOCKET;
//struct sockaddr_in {};
//struct sockaddr {};
//#endif

// Example/Board Header files
//#include "Board.h"
#include "ti_drivers_config.h"

#include <ti/display/Display.h>

#include "define.h"
#include "structs.h"
#include "externs.h"
#include "proto.h"

#if defined ( USE_HOST_OVER_SPI )

int SendHostCommandData( struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr, uint32_t BytesToSend, uint8_t WirelessProtocol )
{
//    uint16_t BytesToSend;
    size_t BytesRead;

//    BytesToSend = CmdPacketDatagramPtr->Header.PayloadBytes + sizeof( struct CMD_PACKET_HEADER );
    switch( WirelessProtocol )
    {
//        case  NET_PROTO_RADIO:
//            CopyToRadioBufferAndSendRadioPacket( ( uint8_t * )CmdPacketDatagramPtr, BytesToSend );
//            BytesToSend = 0;
//            break;
#if defined( USE_HOST_OVER_UART ) || defined( USE_HOST_OVER_SPI )
        case NET_PROTO_SERIAL_PORT:
    #if defined( USE_HOST_OVER_UART )
            UART2_write( hUARTSub1GHz, ( uint8_t * )CmdPacketDatagramPtr, BytesToSend, &BytesRead );
            BytesToSend = 0;
            UARTAuxCmdInfo.PacketsSent++;
    #elif defined( USE_HOST_OVER_SPI )
            MsgSPI[ MsgSPIIndex ].Message = CMD_SPI_SEND_CMD;
            MsgSPI[ MsgSPIIndex ].FramePtr = ( char * )CmdPacketDatagramPtr;
            MsgSPI[ MsgSPIIndex ].FrameBytes = BytesToSend;
            Mailbox_post( hMailboxSPI, &MsgSPI[ MsgSPIIndex++ ], BIOS_NO_WAIT );
            if( MsgSPIIndex >= ( sizeof( MsgSPI ) / sizeof( MsgObj ) ) )
                MsgSPIIndex = 0;
            if( SPITaskInfo.State == STATE_SPI_WAITING_ON_XFER )
            {
                ToggleSPIHostIRQRequestToRead();
            }
            BytesToSend = 0;
    #endif
#endif
            break;
        default:
            break;
    }
    return BytesToSend;
}

// Routine for generating strobe to SPI master to initiate a SPI read transaction
void ToggleSPIHostIRQRequestToRead( void )
{
// *****************************************************************************
// Issue SPI read transfer interrupt to host processor (active low strobe)
// *****************************************************************************
    GPIO_write( CONFIG_GPIO_BLE_IRQ, 0 );
    GPIO_write( CONFIG_GPIO_BLE_IRQ, 1 );
    return;
}

// ISR for master request
void GPIOMasterRequest( uint_least8_t index )
{
    GPIO_write( CONFIG_GPIO_BLE_SACK, 1 );
    Semaphore_post( hSemaHostMasterRequest );
    GPIO_write( CONFIG_GPIO_BLE_SACK, 0 );
    return;
}

// SPI callback routine
void transferCompleteFxn( SPI_Handle handle, SPI_Transaction *transaction )
{
    SPITaskInfo.XferCompleteFlag = TRUE;
    Semaphore_post( hSemaHostSPI );
}

// *****************************************************************************
// Create SPI slave thread
// *****************************************************************************
int32_t CreateSlaveSPIThread( void )
{
    int32_t Status;
    pthread_t thread;
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    Semaphore_Params semParams;
    Mailbox_Params mailboxParams;
    Error_Block eb;

    memset( &SPITaskInfo, 0, sizeof( struct SPI_TASK_INFO ) );
    Error_init( &eb );
// *****************************************************************************
// Initialize SPI semaphores
// *****************************************************************************
    Semaphore_Params_init( &semParams );
    semParams.mode = Semaphore_Mode_COUNTING;
    hSemaHostSPI = ( Semaphore_Handle )Semaphore_create( 0, &semParams, &eb );

    Semaphore_Params_init( &semParams );
    semParams.mode = Semaphore_Mode_BINARY;
    hSemaHostMasterRequest = ( Semaphore_Handle )Semaphore_create( 0, &semParams, &eb );

// *****************************************************************************
// Create SPI Mailbox
// *****************************************************************************
    Mailbox_Params_init( &mailboxParams );
    hMailboxSPI = ( Mailbox_Handle )Mailbox_create( sizeof( MsgObj ), 4, &mailboxParams, &eb );

// *****************************************************************************
// Create SPI slave thread
// *****************************************************************************
    pthread_attr_init( &pAttrs );
    priParam.sched_priority = 1;
    Status = pthread_attr_setdetachstate( &pAttrs, PTHREAD_CREATE_DETACHED );
    if( Status == 0 )
    {
        pthread_attr_setschedparam( &pAttrs, &priParam );
        Status = pthread_attr_setstacksize( &pAttrs, 1536 );
        if( Status == 0 )
        {
            Status = pthread_create( &thread, &pAttrs, SlaveSPIThread, NULL );
        }
    }
    return Status;
}

// *****************************************************************************
// SPI slave thread
// *****************************************************************************
void *SlaveSPIThread( void *arg )
{
    bool TransferFlag, MessageFlag;
    int ReturnBytes, BytesToTransfer;
    int32_t StartIndex;
    uint32_t Header, BytesToSend, StartTicks, DeltaTicks, PendingMessages;
    uint8_t *CaptureBufferReadPtr, *SPITxBufferPtr, *SPIRxBufferPtr;
    SPI_Params SlaveSpiParams;
    SPI_Transaction transaction;
    MsgObj message;

// *****************************************************************************
// Wait on console to be available
// *****************************************************************************
    while( hDisplay == NULL )
        Task_sleep( 1000 );

    GPIO_write( CONFIG_GPIO_BLE_IRQ, 1 );
    Semaphore_reset( hSemaHostSPI, 0 );
    Semaphore_reset( hSemaHostMasterRequest, 0 );

    SPITaskInfo.State = STATE_SPI_INIT;
    SPI_init();
    SPI_Params_init( &SlaveSpiParams );
    SlaveSpiParams.frameFormat = SPI_POL0_PHA0;
    SlaveSpiParams.mode = SPI_PERIPHERAL;
    SlaveSpiParams.transferCallbackFxn = transferCompleteFxn;
    SlaveSpiParams.transferMode = SPI_MODE_CALLBACK;
//    SlaveSpiParams.transferMode = SPI_MODE_BLOCKING;
//    SlaveSpiParams.bitRate = 2000000;
    SlaveSpiParams.transferTimeout = 100000;
    SlaveSpiParams.dataSize = 8;
    SPITaskInfo.hSlaveSPI = SPI_open( CONFIG_SPI_0, &SlaveSpiParams );
    if( SPITaskInfo.hSlaveSPI == NULL )
    {
        Display_printf( hDisplay, 0, 0, "Error initializing slave SPI" );
        while( TRUE );
    }
    else
    {
        Display_printf( hDisplay, 0, 0, "SlaveSPIThread: Slave SPI initialized" );
    }
    SPITaskInfo.BytesToTransfer = SPI_TRANSFER_SIZE;
//    memset( SPITaskInfo.TxtBuffer, 0, sizeof( SPITaskInfo.TxtBuffer ) );

// *****************************************************************************
// Enable host master request
// *****************************************************************************
    GPIO_setConfig( CONFIG_GPIO_BLE_MREQ, GPIO_CFG_IN_PD | GPIO_CFG_IN_INT_RISING );
    GPIO_setCallback( CONFIG_GPIO_BLE_MREQ, GPIOMasterRequest );
    GPIO_enableInt( CONFIG_GPIO_BLE_MREQ );
// *****************************************************************************
// Toggle Host IRQ line to tell the host to perform a SPI read operation on the 1st command
// *****************************************************************************
//    ToggleSPIHostIRQRequestToRead();
// *****************************************************************************
// Host SPI main loop
// *****************************************************************************
    transaction.txBuf = ( void * )SPITaskInfo.TxtBuffer[ SPITaskInfo.TxBufferIndex ];
    SPITxBufferPtr = ( uint8_t * )SPITaskInfo.TxtBuffer[ SPITaskInfo.TxBufferIndex ];
    SPITaskInfo.TxBufferIndex++;
    SPITaskInfo.TxBufferIndex %= SPI_MAX_NUMBER_OF_BUFFERS;
    *( uint32_t * )SPITxBufferPtr = 0;
    Display_printf( hDisplay, 0, 0, "SlaveSPIThread: Waiting on master request" );
    Semaphore_pend( hSemaHostMasterRequest, BIOS_WAIT_FOREVER );
    while( TRUE )
    {
// *****************************************************************************
// Setup SPI transfer parameters
// *****************************************************************************
        SPITaskInfo.State = STATE_SPI_INITIATING_XFER;
        transaction.count = SPITaskInfo.BytesToTransfer;
        transaction.rxBuf = ( void * )SPITaskInfo.RcvBuffer[ SPITaskInfo.RxBufferIndex ];
        SPIRxBufferPtr = ( uint8_t * )SPITaskInfo.RcvBuffer[ SPITaskInfo.RxBufferIndex ];
        SPITaskInfo.RxBufferIndex++;
        SPITaskInfo.RxBufferIndex %= SPI_MAX_NUMBER_OF_BUFFERS;
        SPITaskInfo.XferCompleteFlag = FALSE;
//        Display_printf( hDisplay, 0, 0, "TaskSPIMaster: Waiting for transfer" );
// *****************************************************************************
// Start SPI transfer
// *****************************************************************************
        GPIO_write( CONFIG_GPIO_LED_RED, 1 );
        GPIO_write( CONFIG_GPIO_LED_RED, 0 );
        TransferFlag = SPI_transfer( SPITaskInfo.hSlaveSPI, &transaction );
        GPIO_write( CONFIG_GPIO_LED_GREEN, 1 );
        GPIO_write( CONFIG_GPIO_LED_GREEN, 0 );
        if( TransferFlag == TRUE )
        {
            SPITaskInfo.State = STATE_SPI_WAITING_ON_XFER;
// *****************************************************************************
// Wait on semaphore from SPI transfer callback function
// *****************************************************************************
            Semaphore_pend( hSemaHostSPI, BIOS_WAIT_FOREVER );
            if( SPITaskInfo.XferCompleteFlag == TRUE )
            {
// *****************************************************************************
// Valid SPI transfer - process receive buffer
// *****************************************************************************
                SPITaskInfo.State = STATE_SPI_PROCESS_INCOMING_MESSAGE;
                SPITaskInfo.ValidSPITransfers++;
// *****************************************************************************
// Process receive buffer if valid
// *****************************************************************************
//                Display_printf( hDisplay, 0, 0, "SlaveSPIThread: transfer completed" );
                if( SPIRxBufferPtr[ 0 ] == PACKET_SYNC_CODE )
                {
                    SPITaskInfo.State = STATE_SPI_PROCESS_COMMAND;
                    SPITaskInfo.ValidSPICommands++;
//                    Display_printf( hDisplay, 0, 0, "SlaveSPIThread: valid command" );
// *****************************************************************************
// Valid header - process command
// *****************************************************************************
                    MsgMessage[ MsgMessageIndex ].Message = CMD_MSG_PROCESS_SPI_MESSAGE;
                    MsgMessage[ MsgMessageIndex ].FramePtr = ( char * )SPIRxBufferPtr;
                    MsgMessage[ MsgMessageIndex ].ShortWord1 = NET_PROTO_SERIAL_PORT;
                    Mailbox_post( hMailboxMessage, &MsgMessage[ MsgMessageIndex++ ], BIOS_NO_WAIT );      // post msg containing LED state into the MAILBOX
                    if( MsgMessageIndex >= ( sizeof( MsgMessage ) / sizeof( MsgObj ) ) )
                        MsgMessageIndex = 0;
                } // end of if( SPITaskInfo.RcvBuffer[ 0 ] == PACKET_SYNC_CODE )
                else
                {
//                  Display_printf( hDisplay, 0, 0, "TaskSPIMaster: invalid command" );
                    SPITaskInfo.InvalidSPICommands++;
                }
            } // end of if( SPITaskInfo.XferCompleteFlag == TRUE )
        } // end of if( TransferFlag == TRUE )
        else
        {
            SPITaskInfo.State = STATE_SPI_PROCESSING_ERROR;
            SPITaskInfo.InvalidSPITransfers++;
            Display_printf( hDisplay, 0, 0, "Unsuccessful slave SPI transfer" );
            continue;
        }
// *****************************************************************************
// Waiting on host (SPI master) to set MREQ pin to process message and initiate SPI transaction
// *****************************************************************************
        SPITaskInfo.State = STATE_SPI_WAITING_ON_XFER;
        Semaphore_pend( hSemaHostMasterRequest, BIOS_WAIT_FOREVER );

// *****************************************************************************
// Zero out transmit buffer for any receive only type of operation
// *****************************************************************************
//        transaction.txBuf = ( void * )SPITaskInfo.TxtBuffer[ SPITaskInfo.TxBufferIndex ];
//        SPITxBufferPtr = ( uint8_t * )SPITaskInfo.TxtBuffer[ SPITaskInfo.TxBufferIndex ];
//        SPITaskInfo.TxBufferIndex++;
//        SPITaskInfo.TxBufferIndex %= SPI_MAX_NUMBER_OF_BUFFERS;
//        *( uint32_t * )SPITxBufferPtr = 0;
// *****************************************************************************
// Process internal message from tasks
// *****************************************************************************
        SPITaskInfo.State = STATE_SPI_WAITING_ON_MESSAGE;
        MessageFlag = Mailbox_pend( hMailboxSPI, &message, BIOS_NO_WAIT );             // process message
        if( MessageFlag == TRUE )
        {
//            Display_printf( hDisplay, 0, 0, "TaskSPIMaster: received message" );
            SPITaskInfo.NumberOfMessages++;
            switch( message.Message )
            {
// *****************************************************************************
// Send a request for a SPI read operation to the host and setup the transmit buffer to point to the read buffer
// *****************************************************************************
                case CMD_SPI_SEND_CMD:
                    SPITaskInfo.State = STATE_SPI_REQUESTING_READ_TRANSFER;
//                    Display_printf( hDisplay, 0, 0, "TaskSPIMaster: Issuing send command" );
                    SPITaskInfo.MessagesSent++;
                    transaction.txBuf = ( void * )message.FramePtr;
//                    memcpy( ( uint8_t * )SPITaskInfo.TxtBuffer, ( uint8_t * )message.FramePtr, message.FrameBytes );
//                    transaction.txBuf = ( void * )SPITaskInfo.TxtBuffer;
//                    SPI_transferCancel( SPITaskInfo.hSlaveSPI );
// *****************************************************************************
// Issue SPI read transfer interrupt to host processor by toggling IRQ line
// *****************************************************************************
//                    ToggleSPIHostIRQRequestToRead();
                    break;
                default:
                    break;
            } // end of switch( message.Message )
        } // end of if( MessageFlag == TRUE )
        else
        {
            transaction.txBuf = ( void * )SPITaskInfo.TxtBuffer[ SPITaskInfo.TxBufferIndex ];
            SPITxBufferPtr = ( uint8_t * )SPITaskInfo.TxtBuffer[ SPITaskInfo.TxBufferIndex ];
            SPITaskInfo.TxBufferIndex++;
            SPITaskInfo.TxBufferIndex %= SPI_MAX_NUMBER_OF_BUFFERS;
            *( uint32_t * )SPITxBufferPtr = 0;
        }
    } // end of while( TRUE )
//    SPI_close(SPITaskInfo.hSlaveSPI);
}

#endif

// *****************************************************************************
// end of file
// *****************************************************************************

