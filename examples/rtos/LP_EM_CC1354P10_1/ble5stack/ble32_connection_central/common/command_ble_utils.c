// *****************************************************************************
// command_utils.c
// *****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#if defined ( USE_WIFI_RADIO )
#include "lwip/sockets.h"
#endif

#if defined ( USE_BLE_RADIO )
#include <ti/sysbios/knl/Mailbox.h>
#endif

#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/display/Display.h>
#include <ti/drivers/SDFatFS.h>
#include <third_party/fatfs/ffcio.h>

#include "ti_drivers_config.h"

#include <mqueue.h>
#include <semaphore.h>

#include "define.h"
#include "structs.h"
#include "externs.h"
#include "proto.h"

#define MAX_NUMBER_SUB1GHZ_BUFFERS 4

uint8_t TextString[ 128 ];
char NetBuffer[ MAX_NUMBER_SUB1GHZ_BUFFERS ][ 512 ];
int NetPayloadIndex = 0;

int32_t ProcessCommand( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr, int SendProtocol )
{
    bool SuccessFlag, FoundFlag;
    uint8_t Byte, Byte1, Byte2, AckIndex, SubCommand, FilePhaseIndex, RSSIBinIndex, BytesPerPacketIndex;
    uint16_t Word, SwapWord, BytesPerPacket, TestNumber, Interval, BufferIndex, RadioAddress;
    uint32_t ReturnBytes, Value, PacketsToSend, TotalPackets, BitRate, DeltaTime, WaitTime;
    int32_t Result, ReturnValue, Index, CommandByteIndex, ByteIndex, BytesSent, Length, BytesToSend, TotalTicks, DeltaTicks;
    int32_t i32IntegerPart, i32FractionPart;
    size_t UARTBytesSent;
    double fSeconds, fValue, fDistance, fBearing;
    uint8_t *PayloadPtr, *TempPtr;
    uint32_t *HistogramPtr;
    uint8_t CommandBuffer[ 32 ];
    uint8_t Byte2Array[ 8 ];
    struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr;
    struct CMD_PACKET_HEADER *CmdPacketHeaderPtr;
    struct SENSOR_CMD_INFO *SensorCmdInfoPtr;
    struct HISTOGRAM_TIMING_INFO *HistogramTimingInfoPtr;
    struct HISTOGRAM_RSSI_INFO *HistogramRSSIInfoPtr;
    struct BQ27441_INFO *BatteryInfoPtr;
    struct GPS_INFO *GPSInfoPtr;
    struct COMP_DATA PERValue, RemoteLatitude, RemoteLongitude;

    CmdPacketDatagramPtr = ( struct CMD_PACKET_DATAGRAM * )SrcPacketDatagramPtr;
    CmdPacketHeaderPtr = ( struct CMD_PACKET_HEADER * )&CmdPacketDatagramPtr->Header;
    ReturnBytes = CmdPacketHeaderPtr->PayloadBytes + sizeof( struct CMD_PACKET_HEADER );
    if( CmdPacketHeaderPtr->SyncByte == PACKET_SYNC_CODE )
    {
        if( ( CmdPacketHeaderPtr->DestinationAddress == 0 ) || ( CmdPacketHeaderPtr->DestinationAddress == RadioDeviceInfo.MyBoard.ID ) )
        {
            CommandByteIndex = 0;
            switch( CmdPacketHeaderPtr->Command )
            {
                case CMD_TEXT_CONTROL:
                    strncpy( ( char * )TextString, (const char *)&SrcPacketDatagramPtr->Payload[ CommandByteIndex ], CmdPacketHeaderPtr->PayloadBytes );
                    TextString[ CmdPacketHeaderPtr->PayloadBytes ] = '\0';
                    Display_printf( hDisplay, 0, 0, "String = %s", ( char * )TextString );
                    break;
                case CMD_BOARD_CONTROL:
                    Display_printf( hDisplay, 0, 0, "Received board control command" );
                    break;
                case CMD_BLE_CONTROL:
                    switch( CmdPacketHeaderPtr->SubCommand )
                    {
                        case CMD_BLE_32X_DISCOVER:
                            if( SrcPacketDatagramPtr->Payload[ 0 ] == TRUE )
                            {
                                Display_printf( hDisplay, 0, 0, "Discover on" );
                                BLEHostCommand( CMD_BLE_32X_DISCOVER, 1 );
                            }
                            else
                            {
                                Display_printf( hDisplay, 0, 0, "Discover off" );
                                BLEHostCommand( CMD_BLE_32X_DISCOVER, 0 );
                            }
                            break;
                        case CMD_BLE_32X_CONNECT:
                            if( SrcPacketDatagramPtr->Payload[ 0 ] == TRUE )
                                Display_printf( hDisplay, 0, 0, "Connect index = %d", SrcPacketDatagramPtr->Payload[ 1 ] );
                            else
                                Display_printf( hDisplay, 0, 0, "DisConnect index = %d", SrcPacketDatagramPtr->Payload[ 1 ] );
                            break;
                        case CMD_BLE_32X_AUTO_GATT:
                            Display_printf( hDisplay, 0, 0, "Auto GATT" );
                            if( BLECentralInfo.EnableMTUFlag == TRUE )
                            {
                                Display_printf( hDisplay, 0, 0, "MTU updates are already enabled" );
                            }
                            else
                            {
                                BLEHostCommand( CMD_BLE_32X_MTU_UPDATE, 0 );
//                                BLEHostCommand( CMD_BLE_32X_NOTIFY, 0 );
//                                BLEHostCommand( CMD_BLE_32X_START_GATT, 1 );
                            }
                            break;
                        case CMD_BLE_32X_LED:
                            SubCommand = SrcPacketDatagramPtr->Payload[ 0 ];
                            Display_printf( hDisplay, 0, 0, "LED %d", SubCommand );
                            switch( SubCommand )
                            {
                                case 0:
                                    GPIO_write( CONFIG_GPIO_LED_RED, 0 );
                                    break;
                                case 1:
                                    GPIO_write( CONFIG_GPIO_LED_GREEN, 1 );
                                    break;
                            }
                            break;
                        case CMD_BLE_32X_PRINT_STRING:
                            Display_printf( hDisplay, 0, 0, "Print %d", SrcPacketDatagramPtr->Payload[ CommandByteIndex++ ] );
                            CmdPacketHeaderPtr->PayloadBytes--;
                            strncpy( ( char * )TextString, (const char *)&SrcPacketDatagramPtr->Payload[ CommandByteIndex ], CmdPacketHeaderPtr->PayloadBytes );
                            TextString[ CmdPacketHeaderPtr->PayloadBytes ] = '\0';
                            Display_printf( hDisplay, 0, 0, "String = %s", ( char * )TextString );
                            break;
//                        case CMD_BLE_32X_READ_DATA_BLOCK:
//                            Display_printf( hDisplay, 0, 0, "Read %d", SrcPacketDatagramPtr->Payload[ 0 ] );
//                            sprintf( TextString, "Hey WiFi\n" );
//                            Length = strlen( TextString );
//#if defined( USE_UART_CMD )
//                            BytesToSend = CreateMessageWithBuffer( CMD_TEXT_CONTROL, 0, 0, ( uint8_t * )CommandBuffer, ( uint8_t * )TextString, Length, NET_PROTO_SERIAL_PORT );
//                            Result = UART2_write( hUARTCommand, ( void * )CommandBuffer, BytesToSend, &UARTBytesSent );
//                            UARTCommandInfo.PacketsSent++;
//#endif
//                            break;
                        case CMD_BLE_32X_GET_STATUS:
                            Display_printf( hDisplay, 0, 0, "Status %d", SrcPacketDatagramPtr->Payload[ 0 ] );

                            sprintf( TextString, "WiFi Status\n" );
                            Length = strlen( TextString );
                            BytesToSend = CreateMessageWithBuffer( CMD_BLE_CONTROL, CMD_BLE_32X_STATUS_RESULT, 0, ( uint8_t * )CommandBuffer, ( uint8_t * )TextString, Length, NET_PROTO_SERIAL_PORT );
#if defined( USE_UART_CMD )
                            ReturnValue = UART2_write( hUARTCommand, ( void * )CommandBuffer, BytesToSend, &UARTBytesSent );
                            UARTCommandInfo.PacketsSent++;
#endif
                            break;
                        case CMD_BLE_32X_MTU_UPDATE:
                            if( BLECentralInfo.EnableMTUFlag == TRUE )
                            {
                                Display_printf( hDisplay, 0, 0, "MTU updates are already enabled" );
                            }
                            else
                                BLEHostCommand( CMD_BLE_32X_MTU_UPDATE, 0 );
                            break;
                        case CMD_BLE_32X_NOTIFY:
                            if( BLECentralInfo.EnableNotificationFlag == TRUE )
                            {
                                Display_printf( hDisplay, 0, 0, "Notifications are already enabled" );
                            }
                            else
                                BLEHostCommand( CMD_BLE_32X_NOTIFY, 0 );
                            break;
                        case CMD_BLE_32X_START_GATT:
                            if( BLECentralInfo.EnableGATTWriteFlag == TRUE )
                            {
                                Display_printf( hDisplay, 0, 0, "GATT writes are already enabled" );
                            }
                            else
                                BLEHostCommand( CMD_BLE_32X_START_GATT, 1 );
                            break;
                        case CMD_BLE_32X_STOP_GATT:
                            if( BLECentralInfo.EnableGATTWriteFlag == FALSE )
                            {
                                Display_printf( hDisplay, 0, 0, "GATT writes are already disabled" );
                            }
                            else
                                BLEHostCommand( CMD_BLE_32X_STOP_GATT, 0 );
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            } // end of switch( CmdPacketHeaderPtr->Command )
        } // end of if( ( CmdPacketHeaderPtr->DestinationAddress == 0 ) || ( CmdPacketHeaderPtr->DestinationAddress == RadioDeviceInfo.MyBoard.ID ) )
    } // end of if( CmdPacketHeaderPtr->SyncByte == PACKET_SYNC_CODE )
    return ReturnBytes;
}

//int32_t SendCommandDataToNetwork( uint16_t Message, struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr, uint8_t WirelessProtocol )
//{
//    struct CMD_PACKET_HEADER *CmdPacketHeaderPtr;
//
//    CmdPacketHeaderPtr = ( struct CMD_PACKET_HEADER * )&CmdPacketDatagramPtr->Header;
//    switch( WirelessProtocol )
//    {
//        case  NET_PROTO_TCP:
//            if( hMailboxTCPServerTx != NULL )
//            {
//                MsgTCPServerTx[ MsgTCPServerTxIndex ].FramePtr = ( char * )CmdPacketDatagramPtr;
//                MsgTCPServerTx[ MsgTCPServerTxIndex ].FrameBytes = CmdPacketHeaderPtr->PayloadBytes + sizeof( struct CMD_PACKET_HEADER );
//                MsgTCPServerTx[ MsgTCPServerTxIndex ].Message = Message;
//                Mailbox_post( hMailboxTCPServerTx, &MsgTCPServerTx[ MsgTCPServerTxIndex++ ], BIOS_WAIT_FOREVER );       // post msg containing LED state into the MAILBOX
//                if( MsgTCPServerTxIndex >= ( sizeof( MsgTCPServerTx ) / sizeof( MsgObj ) ) )
//                    MsgTCPServerTxIndex = 0;
//            }
//            break;
//        case  NET_PROTO_UDP:
//        case  NET_PROTO_MULTICAST:
//            MsgMessage[ MsgMessageIndex ].FramePtr = ( char * )CmdPacketDatagramPtr;
//            MsgMessage[ MsgMessageIndex ].FrameBytes = CmdPacketHeaderPtr->PayloadBytes + sizeof( struct CMD_PACKET_HEADER );
//            MsgMessage[ MsgMessageIndex ].ShortWord1 = WirelessProtocol;
//            MsgMessage[ MsgMessageIndex ].Message = Message;
//            Mailbox_post( hMailboxMessage, &MsgMessage[ MsgMessageIndex++ ], BIOS_WAIT_FOREVER );       // post msg containing LED state into the MAILBOX
//            if( MsgMessageIndex >= ( sizeof( MsgMessage ) / sizeof( MsgObj ) ) )
//                MsgMessageIndex = 0;
//            break;
//        default:
//            break;
//    }
//    return 0;
//}

// *****************************************************************************
// end of file
// *****************************************************************************
