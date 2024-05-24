// *****************************************************************************
// ThreadBLE_UARTConsole.c
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
#include <mqueue.h>
#include <semaphore.h>
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

#include "define.h"
#include "structs.h"
#include "externs.h"
#include "proto.h"

int32_t CreateUARTConsoleThread( void )
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
        Status = pthread_attr_setstacksize( &pAttrs, 2560 );
        if( Status == 0 )
        {
            Status = pthread_create( &thread, &pAttrs, UARTConsoleThread, NULL );
        }
    }
    return Status;
}

void *UARTConsoleThread(void *args)
{
    bool ReturnValue, KeysFlag, InvalidCommandFlag;
    char Key, Byte;
    int32_t Result, ByteIndex, Index, NumberOfParameters, Length;
    size_t BytesRead, BytesWritten;
    char *TokenPtr, *NextValuePtr;
//    char DelimiterString[ 4 ];
    char *ParametersPtr[ 4 ];
    char TextString[ 32 ];

#if defined( USE_BLE_RADIO )
    while( hDisplay == NULL )
        Task_sleep( 10000 );
#else
    hDisplay = Display_open( Display_Type_UART, NULL );
    if( hDisplay == NULL )
    {
        while( TRUE );
    }
#endif
    hUARTConsole =  ( UART2_Handle )( ( DisplayUart_Handle )( hDisplay->object ) )->hUart;
    Display_printf( hDisplay, 0, 0, "TaskUARTConsole: Got valid UART handle - starting application" );

    for( Index = 0; Index < 4; Index++ )
    {
        ParametersPtr[ Index ] = ( char * )malloc( 16 );
    }
// *******************************************************************************************************
// Create Message Thread
// *******************************************************************************************************
    Display_printf( hDisplay, 0, 0, "UART Console Thread: Starting main loop" );
    while( TRUE )
    {
       ByteIndex = 0;
       KeysFlag = TRUE;
//       strcpy( DelimiterString,  " " );
       while( KeysFlag )
       {
//           UART_read( hUARTConsole, &TextString[ ByteIndex++ ], 1 );
           UART2_read( hUARTConsole, &Byte, 1, &BytesRead );
           UART2_write( hUARTConsole, &Byte, 1, &BytesWritten );
           TextString[ ByteIndex++ ] = Byte;
           switch( Byte )
           {
               case '\b':
                   TextString[ --ByteIndex ] = '\0';
                   TextString[ --ByteIndex ] = '\0';
//                   ByteIndex -= 2;
                   continue;
               case '\r':
                   TextString[ --ByteIndex ] = '\0';
//                   ByteIndex -= 1;
                   break;
               case '\n':
                   TextString[ --ByteIndex ] = '\0';
                   KeysFlag = FALSE;
                   break;
               default:
                    break;
           }
       }  // end of while( KeysFlag )
//        }while( TextString[ ByteIndex - 1 ] != '\n' );
//       if( ( TextString[ 0 ] == '\0' ) || ( TextString[ 0 ] == '\r' ) || ( TextString[ 0 ] == '\n' ) )
//           continue;
        TokenPtr = strtok( TextString, " " );
        Index = 0;
        NumberOfParameters = 0;
        while( TokenPtr != NULL )
        {
//          UART_PRINT( "string = %s\n", token );
//          UART_PRINT( "size = %d\n", strlen( token ) );
            if( Index == 0 )
            {
                Key = TokenPtr[ 0 ];
//              UART_PRINT( "Key = %c\n", Key );
            }
            else if( Index > 0 )
            {
                NumberOfParameters++;
//              Parameters[ Index - 1 ] = atoi( token );
//               Parameters[ Index - 1 ] = TokenPtr[ 0 ]; //strtol( TokenPtr, &NextValuePtr, 16 );
                strcpy( ParametersPtr[ Index - 1 ], TokenPtr );
//              UART_PRINT( "param = %d\n", Param1 );
//              if( ( token[0] >= '0' ) && ( token[0] <= '9' ) )
//                  UART_PRINT( "Data = %d\n", token[0] - '0' );
            }
//           TokenPtr = strtok( NULL, DelimiterString );
            TokenPtr = strtok( NULL, " " );
            Index++;
        }
        InvalidCommandFlag = ProcessConsoleCommand( Key, NumberOfParameters, &ParametersPtr );
        if( InvalidCommandFlag == TRUE )
            Display_printf( hDisplay, 0, 0, "Invalid command" );
    } // end of while( TRUE )
    return NULL;
}

// *****************************************************************************
// end of file
// *****************************************************************************
