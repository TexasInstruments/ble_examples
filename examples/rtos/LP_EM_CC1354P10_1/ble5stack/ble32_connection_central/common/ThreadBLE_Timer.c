// *****************************************************************************
// ThreadBLE_TimerShelby.c
// *****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <pthread.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

// XDCtools Header files
#include <xdc/std.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/display/Display.h>

#include "define.h"
#include "structs.h"
#include "externs.h"
#include "proto.h"

//#define SSSC_EVT_SEND_PACKET                    0x08

int32_t CreateBLETimerThread( void )
{
    int32_t Status;
    pthread_t thread;
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    Semaphore_Params semParams;
    Error_Block eb;

    Semaphore_Params_init( &semParams );
    semParams.mode = Semaphore_Mode_BINARY;
    hSemaTimer = ( Semaphore_Handle )Semaphore_create( 0, &semParams, &eb );

// *****************************************************************************
// Create Message Thread
// *****************************************************************************
    pthread_attr_init( &pAttrs );
    priParam.sched_priority = 1;
    Status = pthread_attr_setdetachstate( &pAttrs, PTHREAD_CREATE_DETACHED );
    if( Status == 0 )
    {
        pthread_attr_setschedparam( &pAttrs, &priParam );
        Status = pthread_attr_setstacksize( &pAttrs, 4096 );
        if( Status == 0 )
        {
            Status = pthread_create( &thread, &pAttrs, BLETimerThread, NULL );
        }
    }
    return Status;
}

void *BLETimerThread(void *args)
{
    bool ConnectedFlag;
    int32_t ReturnValue;
    uint32_t CommandByteIndex, SemaCtr, Index, Length, BytesToSend, LoopCtr;
    size_t UARTBytesSent;
    uint8_t CommandBuffer[ 128 ];
    char TextBuffer[ 16 ];
    char TempBuffer[ 512 ];

// *****************************************************************************
// Wait on valid UART handle
// *****************************************************************************
    while( hDisplay == NULL )
        Task_sleep( 100 );

// *****************************************************************************
// Start main loop
// *****************************************************************************
    SemaCtr = 0;
    LoopCtr = 0;
    Display_printf( hDisplay, 0, 0, "BLETimerThread: Starting main loop" );
    while( TRUE )
    {
        Semaphore_pend( hSemaTimer, 100000 );              // process loop @ 1 Hz
        SemaCtr++;
// *****************************************************************************
// Send test command out UART channel
// *****************************************************************************
#if defined ( USE_UART_CMD )
//        if( hUARTCommand != NULL )
//        {
//            sprintf( TextBuffer, "Hey WiFi: %d\n", ++LoopCtr );
//            Length = strlen( TextBuffer );
//            BytesToSend = CreateMessageWithBuffer( CMD_TEXT_CONTROL, 0, 0, ( uint8_t * )CommandBuffer, ( uint8_t * )TextBuffer, Length, NET_PROTO_SERIAL_PORT );
//            ReturnValue = UART2_write( hUARTCommand, ( void * )CommandBuffer, BytesToSend, &UARTBytesSent );
//            UARTCommandInfo.PacketsSent++;
//        }

//        CommandByteIndex = 0;
//        TempBuffer[ CommandByteIndex++ ] = BLECentralInfo.CurrentJoinedDevices;
//        TempBuffer[ CommandByteIndex++ ] = 0;
//        memcpy( &TempBuffer[ CommandByteIndex ], &BLECentralInfo.DeviceInfo[ 0 ], sizeof( struct BLE_DEVICE_INFO) );
//        CommandByteIndex += sizeof( struct BLE_DEVICE_INFO);
//        BytesToSend = CreateMessageWithBuffer( CMD_BLE_CONTROL, CMD_BLE_32X_SCAN32_RESULT, 0, ( uint8_t * )CommandBuffer, ( uint8_t * )TempBuffer, CommandByteIndex, NET_PROTO_SERIAL_PORT );
//        ReturnValue = UART2_write( hUARTCommand, ( void * )CommandBuffer, BytesToSend, &UARTBytesSent );
//        UARTCommandInfo.PacketsSent++;
//        BLECentralInfo.DeviceInfo[ 0 ].JoinedCtr++;
//
//        CommandByteIndex = 0;
//        TempBuffer[ CommandByteIndex++ ] = BLECentralInfo.CurrentJoinedDevices;
//        TempBuffer[ CommandByteIndex++ ] = 1;
//        memcpy( &TempBuffer[ CommandByteIndex ], &BLECentralInfo.DeviceInfo[ 1 ], sizeof( struct BLE_DEVICE_INFO) );
//        CommandByteIndex += sizeof( struct BLE_DEVICE_INFO);
//        BytesToSend = CreateMessageWithBuffer( CMD_BLE_CONTROL, CMD_BLE_32X_SCAN32_RESULT, 0, ( uint8_t * )CommandBuffer, ( uint8_t * )TempBuffer, CommandByteIndex, NET_PROTO_SERIAL_PORT );
//        ReturnValue = UART2_write( hUARTCommand, ( void * )CommandBuffer, BytesToSend, &UARTBytesSent );
//        UARTCommandInfo.PacketsSent++;
//        BLECentralInfo.DeviceInfo[ 1 ].JoinedCtr++;

#endif
    } // end of while( TRUE )
    return NULL;
}

// *****************************************************************************
// end of file
// *****************************************************************************
