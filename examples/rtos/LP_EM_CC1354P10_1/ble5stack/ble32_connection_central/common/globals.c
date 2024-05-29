/******************************************************************************

@file  globals.c

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
// globals.c
// *****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#if defined ( USE_WIFI_RADIO )
#include "lwip/sockets.h"
//#include "wifi_if.h"
#endif

#if defined ( USE_BLE_RADIO )
#include <ti/sysbios/knl/Mailbox.h>
#endif

#include <mqueue.h>
#include <semaphore.h>

#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/display/Display.h>
#include <ti/drivers/SDFatFS.h>
#include <third_party/fatfs/ffcio.h>

#include "ti_drivers_config.h"

#include "define.h"
#include "structs.h"
#include "externs.h"
#include "proto.h"

//*****************************************************************************
// Board Parameters
//*****************************************************************************
struct RADIO_DEVICE_INFO RadioDeviceInfo;
struct BOARD_IP_SERVER_INFO IPServerInfo;
struct BOARD_QUEUE_INFO BoardQueueInfo;

// *****************************************************************************
// UART handles
// *****************************************************************************
UART2_Handle hUARTConsole = NULL;
UART2_Handle hUARTCommand = NULL;
#if defined( USE_GPS_UART )
    UART2_Handle hUART_GPS = NULL;
#endif
UART2_Handle uartHandle = NULL;
Display_Handle hDisplay = NULL;

// *****************************************************************************
// Mailbox handles and messages
// *****************************************************************************
#if defined ( USE_BLE_RADIO )
Mailbox_Handle hMailboxMessage;
Mailbox_Handle hMailboxSPI;
#endif

//*****************************************************************************
// Sensor Info
//*****************************************************************************
#if defined( USE_GPS_UART ) || defined ( USE_IMU ) || defined( USE_PIR ) || defined( USE_INA_23X )
struct SENSOR_INFO SensorInfo;
#endif

// *****************************************************************************
// INA233 Variables
// *****************************************************************************
#if defined( USE_INA_23X )
struct INA_INFO INAInfo;
uint16_t INA_ADCConfigArray[ 5 ] =
{
    0x47B7,     // 64 averages @ 4156 msec x 2 (current + voltage) = 0.532 seconds or 1.9 Hz
    0x476F,     // 64 averages @ 2116 usec x 2 (current + voltage) = 0.271seconds or 3.7 Hz
    0x4727,     // 64 averages @ 1100 usec x 2 (current + voltage) = 0.1408 seconds or 7.1 Hz
    0x46DF,     // 64 averages @ 588 usec x 2 (current + voltage) = 0.0753 seconds or 13.3 Hz
    0x4697      // 64 averages @ 332 usec x 2 (current + voltage) = 0.042 seconds or 23.5 Hz
//    0x4E07,     // 1024 averages @ 140 usec x 2 (current + voltage) = 0.28 seconds
//    0x4E4F,     // 1024 averages @ 204 usec x 2 (current + voltage) = 0.41 seconds
//    0x4EDF,     // 1024 averages @ 588 usec x 2 (current + voltage) = 1.2 seconds
//    0x4F27,     // 1024 averages @ 1.1 msec x 2 (current + voltage) = 2.25 seconds
//    0x4F6F      // 1024 averages @ 2.116 msec x 2 (current + voltage) = 4.3 seconds
};
#endif

// *****************************************************************************
// SPI
// *****************************************************************************
#if defined ( USE_HOST_OVER_SPI )
struct SPI_TASK_INFO SPITaskInfo;
SPI_Handle hMasterSPI = NULL;
#endif

// *****************************************************************************
// UART
// *****************************************************************************
#if defined ( USE_UART_CMD )
struct UART_CMD_INFO UARTCommandInfo;
#endif

// *****************************************************************************
// I2C handles
// *****************************************************************************
I2C_Handle  hI2CDevice;

// *****************************************************************************
// I2C Structures
// *****************************************************************************
#if defined( USE_I2C )
struct I2C_INFO I2CInfo;
#endif

//#endif

// *****************************************************************************
// SD Card Info
// *****************************************************************************
#if defined( USE_SD_CARD )
const char SDFilePreString[] = "fat:0:";
struct FILE_STREAMING_INFO FileStreamingInfo;

SDFatFS_Handle sdfatfsHandle = NULL;
struct SD_CARD_DIR_FILE_INFO SDCardDirFileInfo;
struct AUDIO_FILE_INFO AudioFileInfo;
#endif

// *****************************************************************************
// Messages
// *****************************************************************************

MsgObj MsgRadio[ 4 ];
uint32_t MsgRadioIndex = 0;
//Mailbox_Handle hMailboxRadio;
mqd_t hRadioMsgQueue = NULL;

MsgObj MsgRadioSinglePERTask[ 4 ];
uint32_t MsgRadioSinglePERTaskIndex;
//Mailbox_Handle hMailboxRadioSinglePERTask = NULL;
mqd_t hRadioSinglePERTaskMsgQueue = NULL;

MsgObj MsgRadioMultiPERTask[ 4 ];
uint32_t MsgRadioMultiPERTaskIndex;
//Mailbox_Handle hMailboxRadioMultiPERTask;
mqd_t hRadioMultiPERTaskMsgQueue = NULL;

MsgObj MsgMessage[ 4 ];
uint32_t MsgMessageIndex = 0;
mqd_t hMessageMsgQueue = NULL;

MsgObj MsgSPI[ 4 ];
uint32_t MsgSPIIndex = 0;
mqd_t hSPIMsgQueue = NULL;

MsgObj MsgFileThread[ 4 ];
uint32_t MsgFileThreadIndex = 0;
mqd_t hFileThreadMsgQueue = NULL;

MsgObj MsgSlaveSPI[ 4 ];
uint32_t MsgSlaveSPIIndex = 0;

// *****************************************************************************
// Semaphores
// *****************************************************************************
#if !defined ( USE_BLE_RADIO )
sem_t hSemaRadioReceive;
sem_t hSemaRadioAckReceived;
sem_t hSemaRadioAppPktSent;
sem_t hSemaRadioXmitComplete;
sem_t hSemaRadioWaitOnEOF;
//sem_t hSemaINA23x;
sem_t hSemaSensors;
sem_t hSemaTimer;
    #if defined ( USE_HOST_OVER_SPI )
        sem_t hSemaHostSPI;
        sem_t hSemaSlaveAck;
    #endif
#endif
// *****************************************************************************
// BLE
// *****************************************************************************
#if defined ( USE_BLE_RADIO )
struct BLE_INFO BLEInfo;
//struct BLE_CENTRAL_INFO BLECentralInfo = {
//    1,
//    2, 17, 55,
//    {
//        {
//             0, 0, -15, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 112, 24
//        },
//        {
//             0, 0, -25, 0x65, 0x45, 0x23, 0x67, 0x67, 0x88, 34, 12
//        },
//        {
//             0, 0, -19, 0x78, 0x45, 0x89, 0x23, 0x65, 0x98, 23, 766
//        },
//        {
//             0, 0, -45, 0x22, 0x14, 0x78, 0x54, 0x32, 0x66, 34, 67
//        }
//    }
//};
struct BLE_CENTRAL_INFO BLECentralInfo;
Semaphore_Handle hSemaTimer = NULL;
    #if defined ( USE_HOST_OVER_SPI )
        Semaphore_Handle hSemaHostSPI;
        Semaphore_Handle hSemaHostMasterRequest;
    #endif
#endif

// *****************************************************************************
// Network
// *****************************************************************************
#if defined ( USE_WIFI_RADIO )
struct WIFI_INFO WiFiInfo;

mqd_t TCPServerMsgQueue = NULL;
int32_t GlobalTCPClientSocket = -1;

//*****************************************************************************
// Network Variables
//*****************************************************************************
struct NET_INFO *TCPClientInfoPtr = NULL;
struct NET_INFO *UDPClientInfoPtr = NULL;
struct NET_INFO *BroadcastServerInfoPtr = NULL;
#endif

// *****************************************************************************
// Radio
// *****************************************************************************
#if defined ( USE_SUB1GHZ_RADIO ) || defined( USE_WIFI_RADIO )
struct RADIO_INFO RadioInfo;
struct PACKET_INFO RadioPacketInfo;
const uint16_t AcksArray[ MAX_INDEX_OF_ACKS ] =
{
     0, 3, 5, 10
};
const uint32_t RetransmitTestArray[ 64 ] =
{
    0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x10111213, 0x14151617, 0x18191A1B, 0x1C1D1E1F,
    0x20212223, 0x24252627, 0x28292A2B, 0x2C2D2E2F, 0x30313233, 0x34353637, 0x38393A3B, 0x3C3D3E3F,
    0x40414243, 0x44454647, 0x48494A4B, 0x4C4D4E4F, 0x50515253, 0x54555657, 0x58595A5B, 0x5C5D5E5F,
    0x60616263, 0x64656667, 0x68696A6B, 0x6C6D6E6F, 0x70717273, 0x74757677, 0x78797A7B, 0x7C7D7E7F,
    0x80818283, 0x84858687, 0x88898A8B, 0x8C8D8E8F, 0x90919293, 0x94959697, 0x98999A9B, 0x9C9D9E9F,
    0xA0A1A2A3, 0xA4A5A6A7, 0xA8A9AAAB, 0xACADAEAF, 0xB0B1B2B3, 0xB4B5B6B7, 0xB8B9BABB, 0xBCBDBEBF,
    0xC0C1C2C3, 0xC4C5C6C7, 0xC8C9CACB, 0xCCCDCECF, 0xD0D1D2D3, 0xD4D5D6D7, 0xD8D9DADB, 0xDCDDDEDF,
    0xE0E1E2E3, 0xE4E5E6E7, 0xE8E9EAEB, 0xECEDEEEF, 0xF0F1F2F3, 0xF4F5F6F7, 0xF8F9FAFB, 0xFCFDFEFF
};

struct PHY_INFO RadioPHYInfo[ RADIO_PHY_MAX ] =
{
// *****************************************************************************
// 5KHz LRM
// *****************************************************************************
    {
// Text String
        "5 KHz LRM",
// Max Bit Rate
        5000,
// Packets to Send
        50,
// Bytes per packet array
        { 50, 100 },
// Bit rates (No Ack vs Ack)
// 5 Kbps, w/o ACK
        {
             { 4600, 4800 },
// 5 Kbps, with ACK
             { 4100, 4500 }
        }
    },
// *****************************************************************************
// 50KHz
// *****************************************************************************
    {
// Text String
       "50 KHz",
// Max Bit Rate
       50000,
// Packets to Send
       500,
// Bytes per packet array
       { 100, 200 },
// Bit rates (No Ack vs Ack)
// 50 Kbps, w/oACK
       {
            { 44000, 47000 },
// 50 Kbps, with NACK
            { 37000, 43000 }
       }
   },
// *****************************************************************************
// 200KHz
// *****************************************************************************
   {
// Text String
        "200 KHz",
// Max Bit Rate
        200000,
// Packets to Send
        2000,
// Bytes per packet array
        { 100, 200 },
// Bit rates (No Ack vs Ack)
// 200 Kbps, w/oACK
        {
           { 160000, 178000 },
// 200 Kbps, with NACK
           { 130000, 158000 }
        }
   },
// *****************************************************************************
// 500KHz
// *****************************************************************************
   {
// Text String
       "500 KHz",
// Max Bit Rate
       500000,
// Packets to Send
       5000,
// Bytes per packet array
       { 100, 200 },
// Bit rates (No Ack vs Ack)
// 500 Kbps, w/oACK
       {
          { 334000, 401000 },
// 500 Kbps, with NACK
          { 265000, 346000 }
       }
    },
// *****************************************************************************
// 1 MHz
// *****************************************************************************
    {
// Text String
         "1 MHz",
// Max Bit Rate
         1000000,
// Packets to Send
         10000,
// Bytes per packet array
         { 100, 200 },
// Bit rates (No Ack vs Ack)
// 1 Mbps, w/oACK
         {
              { 552000, 711000 },
// 1 Mbps, with NACK
              { 409000, 579000 }
         }
    }
};
#endif
// *****************************************************************************
// Menus
// *****************************************************************************
const char BoardHelpMenuText[] = {
"\n*******************\n\
 Board Commands:\n\
 Usage:\n\
 ? : help menu\n\
 a : board address\n\
*******************\n" };

const char MainHelpMenuText[] = {
"\n*******************\n\
 Top Level Commands:\n\
 Usage:\n\
 ? : help \n\
 b : board\n\
 c : console\n\
 h : help\n\
*******************\n" };

const char WiFiHelpScreenText[] = {
"\n*******************\n\
 Usage:\n\
 ? : get status\n\
 r : router/AP \n\
   ? : status\n\
   s : SSID (text)\n\
   p : password (text)\n\
 w : write parameters to FLASH \n\
 x : exit & restart \n\
*******************\n" };

const char FileHelpMenuText[] = {
"\n*******************\n\
 File Commands:\n\
 Usage:\n\
 ? : help menu\n\
 l : list all files\n\
   a : list all files\n\
   m : list media files\n\
     f : file select\n\
       # : media index\n\
   p : list PCM files\n\
 r : remove file\n\
 p : print file contents\n\
*******************\n" };

#if defined ( USE_PIR )
const char PIRHelpMenuText[] = {
"\n*******************\n\
 PIR Commands:\n\
 Usage:\n\
 ? : help menu\n\
 d : channel mask\n\
   ? : status\n\
   # : value\n\
 e : events\n\
   ? : status\n\
   c : clear\n\
 g : get status\n\
 m : motion flag\n\
   ? : status\n\
   0/1 : off/on\n\
 p : print control\n\
   ? : status\n\
   d : raw data\n\
     0/1 : off/on\n\
   l : local\n\
     0/1 : off/on\n\
   m : motion\n\
     0/1 : off/on\n\
   s : status\n\
     0/1 : off/on\n\
 r : update rate\n\
   ? : status\n\
   # : value\n\
 s : send flag\n\
   ? : status\n\
   d : data\n\
     0/1 : off/on\n\
   m : motion\n\
     0/1 : off/on\n\
   t : text\n\
     # : string\n\
 t : threshold\n\
   ? : status\n\
   # : value\n\
*******************\n" };
#endif

#if defined ( USE_STEPPER_MOTOR )
const char MSPM0_MotorHelpMenuText[] = {
"\n*******************\n\
 Motor Commands:\n\
 Usage:\n\
 ? : help menu\n\
 a : run antenna test\n\
 s : write value to servo\n\
*******************\n" };
#endif

#if defined ( USE_SUB1GHZ_RADIO )
const char RadioHelpMenuText[] = {
"\n*******************\n\
 Radio Commands:\n\
 Usage:\n\
 ? : help menu\n\
 a : address\n\
   ? : display all\n\
   p : set PER destination address\n\
     # : destination address\n\
   s : select desired address\n\
     # : desired address\n\
 b : send broadcast packet\n\
 l : local radio\n\
   ? : help menu\n\
 m : modify PER parameters\n\
   ? : get PER parameters\n\
   b : set bit rate\n\
     # : bit rate\n\
   p : set bytes per packet\n\
     # : bytes per packet\n\
   s : set # of packets to send\n\
     # : packets to send\n\
 p : print control for Rx/Tx\n\
   ? : status\n\
   r : receiver\n\
     ? : status\n\
     0/1 : off/on\n\
   t : transmitter\n\
     ? : status\n\
     0/1 : off/on\n\
 r : remote radio\n\
   ? : help menu\n\
 t : test\n\
   ? : help menu\n\
*******************\n" };

const char RadioLocalHelpMenuText[] = {
"\n*******************\n\
 Radio Local Commands:\n\
 'l'\n\
 Usage:\n\
 ? : help menu\n\
 a : ack\n\
   ? : status\n\
   0/1 : off/on\n\
   n : max retransmits\n\
     # : # of retransmits\n\
   r : retransmit array\n\
   s : status regs\n\
 c : PER control/status\n\
   c : control\n\
   r : receive status\n\
   t : transmit status\n\
 f : radio frequency\n\
   ? : status\n\
   f : forced\n\
     # : frequency(float)\n\
   s : sync\n\
     # : frequency(float)\n\
 g : GPS\n\
   ? : status\n\
   a : set latitude(float)\n\
   b : broadcast\n\
     ? : status\n\
     0/1 : off/on\n\
   o : set longitude(float)\n\
 p : radio PHY\n\
   ? : current PHY\n\
   d : set default PHY\n\
     # : PHY index\n\
   f : forced\n\
     # : PHY index\n\
   r : auto PHY reset\n\
     ? : status\n\
     0/1 : off/on\n\
   s : sync\n\
     # : PHY index\n\
 s : sensors\n\
   b : battery info\n\
   g : gps info\n\
 t : test\n\
   a : audio regs\n\
     ? : status\n\
     c : clear\n\
   f : file regs\n\
     ? : status\n\
     c : clear\n\
   v : video regs\n\
     ? : status\n\
     c : clear\n\
*******************\n" };

const char RadioRemoteHelpMenuText[] = {
"\n*******************\n\
 Radio Remote Commands:\n\
 'r'\n\
 Usage:\n\
 ? : help menu\n\
 a : ack\n\
   0/1 : off/on\n\
   n : max retransmits\n\
     # : # of retransmits\n\
   r : retransmit array\n\
   s : status regs\n\
 c : PER status\n\
   r : receive status\n\
   t : transmit status\n\
 e : encoder\n\
   ? : status\n\
   0/1 : off/on\n\
 f : file transfer\n\
     ? : status\n\
     0/1 : off/on\n\
     b : bit rate\n\
       # : bit rate\n\
     d : directory\n\
     r : file read\n\
       # : file name\n\
     w : file write\n\
       # : file name\n\
 g : GPS\n\
   ? : status\n\
   b : broadcast\n\
     0/1 : off/on\n\
 p : PER\n\
     m : multi(0 to 4)\n\
     s : single\n\
     r : auto PHY reset\n\
       0/1 : off/on\n\
 s : sensors\n\
   b : battery info\n\
   g : gps info\n\
 t : test\n\
   a : antenna test\n\
*******************\n" };


#endif

// *****************************************************************************
// end of file
// *****************************************************************************
