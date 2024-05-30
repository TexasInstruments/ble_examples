/******************************************************************************

@file  externs.h

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
// externs.h
// *****************************************************************************
#if defined ( USE_WIFI_RADIO )
#include "tcpip_if.h"
#endif

//*****************************************************************************
// Board Parameters
//*****************************************************************************
extern struct RADIO_DEVICE_INFO RadioDeviceInfo;
extern struct BOARD_IP_SERVER_INFO IPServerInfo;
extern struct BOARD_QUEUE_INFO BoardQueueInfo;

// *****************************************************************************
// UART handles
// *****************************************************************************
extern UART2_Handle hUARTConsole;
extern UART2_Handle hUARTCommand;
extern UART2_Handle uartHandle;
#if defined( USE_GPS_UART )
    extern UART2_Handle hUART_GPS;
#endif

// *****************************************************************************
// SPI
// *****************************************************************************
extern struct SPI_TASK_INFO SPITaskInfo;

// *****************************************************************************
// Messages
// *****************************************************************************
#if defined ( USE_WIFI_RADIO )
    extern mqd_t TCPServerMsgQueue;
#endif

    extern mqd_t hMessageMsgQueue;
    extern MsgObj MsgRadio[ 4 ];
    extern uint32_t MsgRadioIndex;
    //Mailbox_Handle hMailboxRadio;
    extern mqd_t hRadioMsgQueue;

    extern MsgObj MsgRadioSinglePERTask[ 4 ];
    extern uint32_t MsgRadioSinglePERTaskIndex;
    //Mailbox_Handle hMailboxRadioSinglePERTask = NULL;
    extern mqd_t hRadioSinglePERTaskMsgQueue;

    extern MsgObj MsgRadioMultiPERTask[ 4 ];
    extern uint32_t MsgRadioMultiPERTaskIndex;
    //Mailbox_Handle hMailboxRadioMultiPERTask;
    extern mqd_t hRadioMultiPERTaskMsgQueue;

    extern MsgObj MsgMessage[ 4 ];
    extern uint32_t MsgMessageIndex;
    extern mqd_t hMessageMsgQueue;

    extern MsgObj MsgSPI[ 4 ];
    extern uint32_t MsgSPIIndex;
    extern mqd_t hSPIMsgQueue;

    extern MsgObj MsgFileThread[ 4 ];
    extern uint32_t MsgFileThreadIndex;
    extern mqd_t hFileThreadMsgQueue;

    extern MsgObj MsgSlaveSPI[ 4 ];
    extern uint32_t MsgSlaveSPIIndex;

#if !defined ( USE_BLE_RADIO )
// *****************************************************************************
// Semaphores
// *****************************************************************************
extern sem_t hSemaRadioReceive;
extern sem_t hSemaRadioAckReceived;
extern sem_t hSemaRadioAppPktSent;
extern sem_t hSemaRadioXmitComplete;
extern sem_t hSemaRadioWaitOnEOF;
extern sem_t hSemaINA23x;
extern sem_t hSemaSensors;
extern sem_t hSemaTimer;
#if defined ( USE_HOST_OVER_SPI )
    extern sem_t hSemaHostSPI;
    extern sem_t hSemaSlaveAck;
#endif

// *****************************************************************************
// I2C handles
// *****************************************************************************
#if defined ( USE_I2C )
I2C_Handle  hI2CDevice;
#endif

#endif

// *****************************************************************************
// UART
// *****************************************************************************
extern struct UART_CMD_INFO UARTCommandInfo;

// *****************************************************************************
// I2C Structures
// *****************************************************************************
extern struct I2C_INFO I2CInfo;

// *****************************************************************************
// SPI
// *****************************************************************************
#if defined ( USE_HOST_OVER_SPI )
    extern struct SPI_TASK_INFO SPITaskInfo;
    extern SPI_Handle hMasterSPI;
#endif

// *****************************************************************************
// SD Card Info
// *****************************************************************************
#if defined( USE_SD_CARD )
    extern SDFatFS_Handle sdfatfsHandle;
    extern struct SD_CARD_DIR_FILE_INFO SDCardDirFileInfo;
    extern struct AUDIO_FILE_INFO AudioFileInfo;
    extern struct FILE_STREAMING_INFO FileStreamingInfo;

    extern const char SDFilePreString[];
    extern char const SD_AudioCaptureFileName[];
    extern char const SD_AudioPlaybackFileName[];
#endif

// *****************************************************************************
// Peripherals
// *****************************************************************************
extern Display_Handle hDisplay;

// *****************************************************************************
// Mailbox handles and messages
// *****************************************************************************
//#if defined ( USE_BLE_RADIO )
//extern MsgObj MsgMessage[ 16 ];
//extern uint32_t MsgMessageIndex;
//extern Mailbox_Handle hMailboxMessage;
//#endif

// *****************************************************************************
// BLE
// *****************************************************************************
#if defined ( USE_BLE_RADIO )
extern struct BLE_INFO BLEInfo;
extern struct BLE_CENTRAL_INFO BLECentralInfo;
extern Semaphore_Handle hSemaTimer;
    #if defined ( USE_HOST_OVER_SPI )
        Semaphore_Handle hSemaHostSPI;
        Semaphore_Handle hSemaHostMasterRequest;
    #endif
extern Mailbox_Handle hMailboxMessage;
extern Mailbox_Handle hMailboxSPI;
#endif

// *****************************************************************************
// INA233 Variables
// *****************************************************************************
#if defined( USE_INA_23X )
extern struct INA_INFO INAInfo;
extern uint16_t INA_ADCConfigArray[ 5 ];
#endif

// *****************************************************************************
// Network
// *****************************************************************************
#if defined ( USE_WIFI_RADIO )
extern struct WIFI_INFO WiFiInfo;
extern int32_t GlobalTCPClientSocket;

extern struct NET_INFO *TCPClientInfoPtr;
extern struct NET_INFO *UDPClientInfoPtr;
extern struct NET_INFO *BroadcastServerInfoPtr;
#endif

// *****************************************************************************
// Radio
// *****************************************************************************
extern struct RADIO_INFO RadioInfo;
extern struct PACKET_INFO RadioPacketInfo;
extern struct PHY_INFO RadioPHYInfo[ RADIO_PHY_MAX ];
extern const uint16_t AcksArray[ MAX_INDEX_OF_ACKS ];
extern const uint32_t RetransmitTestArray[ 64 ];

// *****************************************************************************
// Text Utils
// *****************************************************************************
bool atohex( char *Buffer, int Count, int *DataPtr );

//*****************************************************************************
// Sensor Info
//*****************************************************************************
#if defined( USE_GPS_UART ) || defined ( USE_IMU ) || defined( USE_PIR ) || defined( USE_INA_23X )
extern struct SENSOR_INFO SensorInfo;
#endif

// *****************************************************************************
// Help Menus
// *****************************************************************************
extern const char BoardHelpMenuText[];
extern const char MainHelpMenuText[];
extern const char WiFiHelpScreenText[];
extern const char FileHelpMenuText[];
#if defined ( USE_PIR )
extern const char PIRHelpMenuText[];
#endif
#if defined ( USE_STEPPER_MOTOR )
extern const char MSPM0_MotorHelpMenuText[];
#endif
#if defined ( USE_SUB1GHZ_RADIO )
extern const char RadioHelpMenuText[];
extern const char RadioLocalHelpMenuText[];
extern const char RadioRemoteHelpMenuText[];
#endif
// *****************************************************************************
// end of file
// *****************************************************************************
