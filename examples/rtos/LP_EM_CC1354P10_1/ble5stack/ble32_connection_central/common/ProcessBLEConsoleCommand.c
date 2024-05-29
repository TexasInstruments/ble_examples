/******************************************************************************

@file  ProcessBLEConsoleCommand.c

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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#if defined ( USE_WIFI_RADIO )
#include "lwip/sockets.h"
#endif

#if defined ( USE_BLE_RADIO )
#include <ti/sysbios/knl/Mailbox.h>
#endif

#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <ti/drivers/UART2.h>
#include <ti/display/DisplayUart2.h>
#include <ti/display/DisplayExt.h>
#include <ti/display/AnsiColor.h>
#define DisplayUart_Handle      DisplayUart2_Handle
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDFatFS.h>
#include <third_party/fatfs/ffcio.h>

#include <mqueue.h>
#include <semaphore.h>

#include "ti_drivers_config.h"

#include "define.h"
#include "structs.h"
#include "externs.h"
#include "proto.h"

uint8_t TestCommandBuffer[ 2 ][ 64 ];
uint8_t TestCommandBufferIndex = 0;

char RemoteFileNameToRead[ 16 ];
char RemoteFileNameToWrite[ 16 ];

bool ProcessConsoleCommand( char Key, int32_t NumberOfParameters, char **ParametersPtr )
{
	bool InvalidCommandFlag, SuccessFlag, SendFlag, PrintFlag;
    uint8_t Byte, Flags;
    size_t BytesRead, BytesWritten;
    uint16_t ShortValue;
	int32_t ReturnValue, Result, BytesSent, Value, FileBytesRead, BytesToSend, Length, FileSize;
	uint32_t Index, Size;
    uint8_t *DataPtr;
    char *StringPtr;
    double fValue, fFrequency;
    struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr;
    struct CMD_PACKET_HEADER *CmdPacketHeaderPtr;
    char TextString[ 64 ], AuxTextString[ 16 ], FileName[ 32 ];
    uint8_t Buffer[ 32 ];
    MsgObj MessageObject;

	InvalidCommandFlag = FALSE;
	switch( Key )
	{
// ****************************************************************************
// BLE ( b )
// ****************************************************************************
        case 'b':
            if( NumberOfParameters == 0 )
            {
                Display_printf( hDisplay, 0, 0, "Unknown command" );
            }
            else
            {
                switch( ParametersPtr[ 0 ][ 0 ] )
                {
// ****************************************************************************
// display help menu ( b ? )
// ****************************************************************************
                    case '?':
//                        Display_printf( hDisplay, 0, 0, "%s", BLEHelpMenuText );
                        break;
// ****************************************************************************
// ble discover ( b d 'x' )
// ****************************************************************************
                    case 'd':
                        if( NumberOfParameters >= 2 )
                        {
                            switch( ParametersPtr[ 1 ][ 0 ] )
                            {
// ****************************************************************************
// ble discover ( b d 0 )
// ****************************************************************************
                                case '0':
                                    BLEHostCommand( CMD_BLE_32X_DISCOVER, 0 );
                                    break;
                                case '1':
                                    BLEHostCommand( CMD_BLE_32X_DISCOVER, 1 );
                                    break;
                            }
                        }
                        break;
// ****************************************************************************
// ble start/stop GATT control ( b g 'x' )
// ****************************************************************************
                    case 'g':
                        if( NumberOfParameters >= 2 )
                        {
                            switch( ParametersPtr[ 1 ][ 0 ] )
                            {
// ****************************************************************************
// ble stop GATT ( b g 0 )
// ****************************************************************************
                                case '0':
                                    if( BLECentralInfo.EnableGATTWriteFlag == FALSE )
                                    {
                                        Display_printf( hDisplay, 0, 0, "GATT writes are already disabled" );
                                    }
                                    else
                                        BLEHostCommand( CMD_BLE_32X_STOP_GATT, 0 );
                                    break;
// ****************************************************************************
// ble start GATT ( b g 1 )
// ****************************************************************************
                                case '1':
                                    if( BLECentralInfo.EnableGATTWriteFlag == TRUE )
                                    {
                                        Display_printf( hDisplay, 0, 0, "GATT writes are already enabled" );
                                    }
                                    else
                                        BLEHostCommand( CMD_BLE_32X_START_GATT, 1 );
                                    break;
                            }
                        }
                        break;
                    case 'l':
                        if( NumberOfParameters >= 2 )
                        {
                            switch( ParametersPtr[ 1 ][ 0 ] )
                            {
// ****************************************************************************
// turn off LED ( b l 0 )
// ****************************************************************************
                                case '0':
                                    GPIO_write( CONFIG_GPIO_LED_RED, 0 );
                                    break;
// ****************************************************************************
// turn on LED ( b l 1 )
// ****************************************************************************
                                case '1':
                                    GPIO_write( CONFIG_GPIO_LED_GREEN, 1 );
                                    break;
                            }
                        }
                        break;
// ****************************************************************************
// MTU Update
// ****************************************************************************
                    case 'm':
                        if( NumberOfParameters >= 1 )
                        {
                            if( BLECentralInfo.EnableMTUFlag == TRUE )
                            {
                                Display_printf( hDisplay, 0, 0, "MTU updates are already enabled" );
                            }
                            else
                                BLEHostCommand( CMD_BLE_32X_MTU_UPDATE, 0 );
                            break;
                        }
                        break;
// ****************************************************************************
// Notify
// ****************************************************************************
                    case 'n':
                        if( NumberOfParameters >= 1 )
                        {
                            if( BLECentralInfo.EnableNotificationFlag == TRUE )
                            {
                                Display_printf( hDisplay, 0, 0, "Notifications are already enabled" );
                            }
                            else
                                BLEHostCommand( CMD_BLE_32X_NOTIFY, 0 );
                            break;
                        }
                        break;
// ****************************************************************************
// print string
// ****************************************************************************
                    case 'p':
                        if( NumberOfParameters >= 2 )
                        {
                            BLEHostCommand( CMD_BLE_32X_PRINT_STRING, 0 );
                            break;
                        }
                        break;
                    default:
                        break;
                }
            }
            break;
// ****************************************************************************
// clear console screen ( c )
// ****************************************************************************
        case 'c':
            Display_printf( hDisplay, 0, 0, "%s%s", CURSOR_HOME, CLEAR_SCREEN );
            break;
// ****************************************************************************
// help menu ( h )
// ****************************************************************************
       case 'h':
           if( NumberOfParameters == 0 )
           {
               Display_printf( hDisplay, 0, 0, "%s", MainHelpMenuText );
           }
           else
           {
               switch( ParametersPtr[ 0 ][ 0 ] )
               {
                   case '?':
                       Display_printf( hDisplay, 0, 0, "%s", MainHelpMenuText );
                       break;
                   default:
                       break;
               }
           }
           break;
// ****************************************************************************
// enable/disable console printf flags
// ****************************************************************************
       case 'p':
             if( NumberOfParameters == 0 )
             {
#if defined ( USE_SUB1GHZ_RADIO )
                 RadioInfo.ConsolePrintFlag = FALSE;
#endif
#if defined( USE_GPS_UART )
                 SensorInfo.GPSInfo.Status.PrintFlag = FALSE;
#endif
#if defined ( USE_INA_23X )
                 INAInfo.ConsolePrintFlag = FALSE;
#endif
#if defined ( USE_IMU )
                 SensorInfo.IMUInfo.Status.PrintFlag = FALSE;
#endif
             }
             else
             {
                 switch( ParametersPtr[ 0 ][ 0 ] )
                 {
// ****************************************************************************
// get status of print flags ( p ? )
// ****************************************************************************
                     case '?':
#if defined ( USE_IMU )
                         Display_printf( hDisplay, 0, 0, "IMU print flag = %d", SensorInfo.IMUInfo.Status.PrintFlag );
#endif
//                         Display_printf( hDisplay, 0, 0, "%s", PrintHelpMenuText );
                         return InvalidCommandFlag;
                     default:
                         break;
                 }
                 if( NumberOfParameters >= 2 )
                 {
                     switch( ParametersPtr[ 0 ][ 0 ] )
                     {
#if defined ( USE_IMU )
                         case 'u':
// ****************************************************************************
// INA print control
// ****************************************************************************
                             switch( ParametersPtr[ 1 ][ 0 ] )
                             {
// ****************************************************************************
// print: INA status ( p i ? )
// ****************************************************************************
                                 case '?':
                                     Display_printf( hDisplay, 0, 0, "IMU print flag = %d", SensorInfo.IMUInfo.Status.PrintFlag );
                                     break;
// ****************************************************************************
// print: INA print disable ( p i 0 )
// ****************************************************************************
                                 case '0':
                                     SensorInfo.IMUInfo.Status.PrintFlag = FALSE;
                                     break;
// ****************************************************************************
// print: INA print enable ( p i 1 )
// ****************************************************************************
                                 case '1':
                                     SensorInfo.IMUInfo.Status.PrintFlag = TRUE;
                                     break;
                                 default:
                                     break;
                             }
                             break;
#endif
                         default:
                             break;
                     } // end of switch( ParametersPtr[ 0 ][ 0 ] )
                 } // end of if( NumberOfParameters >= 2 )
             }
             break;
#if defined ( USE_HOST_OVER_SPI )
        case 't':
            Size = CreateMessageWithBuffer( CMD_TEXT_CONTROL, 0, 0, TestCommandBuffer[ TestCommandBufferIndex ], "Hey WiFi", sizeof( "Hey WiFi" ), NET_PROTO_SERIAL_PORT );
            SendHostCommandData( ( struct CMD_PACKET_DATAGRAM * )TestCommandBuffer[ TestCommandBufferIndex ], Size, NET_PROTO_SERIAL_PORT );
            TestCommandBufferIndex++;
            if( TestCommandBufferIndex >= 2 )
                TestCommandBufferIndex = 0;
            break;
#endif
        default:
            InvalidCommandFlag = TRUE;
            break;
    } // end of switch( Key )
	return InvalidCommandFlag;
}

// *****************************************************************************
// end of file
// *****************************************************************************

