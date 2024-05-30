/******************************************************************************

@file  define_gps.h

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
// define_gps.h
// *****************************************************************************
#ifndef DEFINE_GPS_H_
#define DEFINE_GPS_H_

    #define MAX_SIZE_OF_NMEA_STRING         128
//*****************************************************************************
// Sony GPS CXD5605
//*****************************************************************************
#if defined( USE_GPS_SONY )
    #define GPS_BUFFER0_BIT_MASK            0x01
    #define GPS_BUFFER1_BIT_MASK            0x02
    #define GPS_BUFFER2_BIT_MASK            0x04
    #define GPS_BUFFER3_BIT_MASK            0x08
    #define GPS_COMMAND_LOADH               0x02
    #define GPS_COMMAND_LOADC1              0x04
    #define GPS_COMMAND_LOADC2              0x05
    #define GPS_COMMAND_EXEC                0x06
    #define GPS_NORMAL_COMMAND_RESULT       0x0400
    #define GPS_SIZE_OF_SONY_PACKET_DATA    74
#endif

//*****************************************************************************
// GPS CC4000
//*****************************************************************************
// GPS Messages Status
#define OLD_GPS_MSG         0x00
#define NEW_GPS_MSG         0x01
// GPS Message Delimiters
#define START_OF_MSG        '$'
#define FIELD_DELIMITER     ','
#define CHECKSUM_DELIMITER  '*'
#define END_OF_MSG_CR       '\r'
#define END_OF_MSG_LF       '\n'
#define DEGREES_SYMBOL      0x7F
// GPS Message Properties

enum GPS_STATES { GPS_STATE_START, GPS_STATE_DATA, GPS_STATE_CHECKSUM, GPS_STATE_END };
enum GPS_ERRORS { GPS_ERROR_NONE, GPS_ERROR_BUFFER_OVERRUN, GPS_ERROR_BAD_CHECKSUM, GPS_ERROR_CHECKSUM_LENGTH, GPS_ERROR_CHECKSUM_OVERRUN, GPS_ERROR_BAD_ENDING };
enum PMTK_CMDS { CHANGE_BAUD_RATE, CHANGE_UPDATE_RATE, ENABLE_OUTPUT_FIELDS };
enum PMTK_FIELDS { GLL=1, RMC=2, VTG=4, GGA=8, GSA=0x10, GSV=0x20 };

#define GPS_BUFFERS         2

#define GPS_MSG_MAX_SIZE    82          // Section 5.3 NMEA Standard
#define GPS_HEADER_SIZE     6           // Section 5.2 NMEA Standard,
                                        // Header: $GPxxx,
#define GPS_CHECKSUM_SIZE   2           // 2 Bytes
#define GPS_GSV_MAX_MSGS    4           // Maximum number of GSV Messages
#define GPS_GSV_MSG_NUM     9           // GSV msg field that indicates
                                        // current msg  number
#define GPS_GSV_MSG_TOT     7       // GSV msg field that indicates total number of
                                        // GSV messages the receiver is sending.

#define GPS_MAX_SV_NUM      20
// GPS Message Headers
#define GPGLL_HEADER        "$GPGLL"
#define GPRMC_HEADER        "$GPRMC"
#define GPGGA_HEADER        "$GPGGA"
#define GPVTG_HEADER        "$GPVTG"
#define GPGSA_HEADER        "$GPGSA"
#define GPGSV_HEADER        "$GPGSV"
#define PTIGCD_HEADER       "$PTIGCD"

// GPS Message Errors (API return - MS) // Section 5.4 NMEA Standard
#define NO_ERROR            0x00
#define CHECKSUM_ERROR      0x01
#define INVALID_CHARACTER   0x02
#define INVALID_HEADER      0x04
#define MSG_SIZE_ERROR      0x08
//#define UART_ERROR            0x10
#define OTHER_ERROR         0x20
// GPS State Machine States
#define GPS_MSG_START       0
#define GPS_DATA            1
#define GPS_CHECKSUM        2
#define GPS_MSG_END         3

#define GPS_DATA_INVALID   'V'
#define GPS_DATA_VALID     'A'

//*****************************************************************************
// CC4000 CFWVER Message Definitions.
//*****************************************************************************
#define CFWVER_TOT_MSGS     1
#define CFWVER_MSG_NUM      2
#define CFWVER_DEVICE       5
#define CFWVER_VERSION      7
#define CFWVER_DATE         8
#define CFWVER_FW_DATE      8
#define CFWVER_FW_TIME      9

//*****************************************************************************
// CC4000 GPSVER Message Definitions.
//*****************************************************************************
#define GPSVER_TOT_MSGS     1
#define GPSVER_MSG_NUM      2
#define GPSVER_SPVERSION   11

//*****************************************************************************
//
// GPS Status Definitions
//
//*****************************************************************************
#define OFF 0
#define ON  1
#define NONE 4
#define GPS_OFF 0
#define GPS_STANDBY 1
#define GPS_SEARCHING 2
#define GPS_TRACKING 3

#define NO_FIRST_FIX 0
#define FIRST_FIX_OBTAINED 1

//*****************************************************************************
// CC4000 Status Definitions
//*****************************************************************************
#define CC4000_TRACKING     1
#define CC4000_ACQUIRING    2
#define CC4000_INITIALIZING 3
#define CC4000_STATUS_FAIL  0
#define CC4000_NO_STATUS  4

#define CC4000_LOCK_30MS  983    // About 60ms
#define CC4000_LOCK_60MS  1966   // About 60ms
#define CC4000_LOCK_75MS  2458   // About 75ms
#define CC4000_LOCK_150MS  4915  // About 150ms
#define CC4000_LOCK_450MS  14746 // About 450ms
#define CC4000_LOCK_550MS  18022 // About 550ms
#define CC4000_LOCK_650MS  21299 // About 650ms

//*****************************************************************************
// GLL Message Definitions.
//*****************************************************************************
#define GLL_LATITUDE        1
#define GLL_NS              2
#define GLL_LONGITUDE       3
#define GLL_EW              4
#define GLL_UTC_TIME        5
#define GLL_DATA_STATUS     6
#define GLL_MODE            7

#define GPS_DATA_INVALID   'V'
#define GPS_DATA_VALID     'A'

//*****************************************************************************
// RMC Message Definitions.
//*****************************************************************************
#define RMC_UTC_TIME        1
#define RMC_DATA_STATUS     2
#define RMC_LATITUDE        3
#define RMC_NS              4
#define RMC_LONGITUDE       5
#define RMC_EW              6
#define RMC_SPEED_GND_KNOTS 7
#define RMC_COURSE_GND_TRUE 8
#define RMC_DATE            9
#define RMC_MAGNETIC_VAR   10
#define RMC_MAGNETIC_DEG   11

//*****************************************************************************
// GGA Message Definitions.
//*****************************************************************************
#define GGA_UTC_TIME        1
#define GGA_LATITUDE        2
#define GGA_NS              3
#define GGA_LONGITUDE       4
#define GGA_EW              5
#define GGA_QUALITY_IND     6
#define GGA_NUM_USED_SVS    7
#define GGA_HOR_DIL_PREC    8
#define GGA_ALTITUDE        9
#define GGA_ALTITUDE_UNITS  10
#define GGA_GEOIDAL_SEP    11
#define GGA_GEOIDAL_SEP_UNITS 12
#define GGA_AGE_DIFF_DATA  13
#define GGA_DIFF_REF_ID    14

//*****************************************************************************
// VTG Message Definitions.
//*****************************************************************************
#define VTG_COURSE_GND_TRUE 1
#define VTG_COURSE_GND_T    2
#define VTG_COURSE_GND_MAGN 3
#define VTG_COURSE_GND_M    4
#define VTG_SPEED_GND_KNOTS 5
#define VTG_SPEED_GND_N     6
#define VTG_SPEED_GND_KMHRS 7
#define VTG_SPEED_GND_K     8
#define VTG_MODE        9

//*****************************************************************************
// GSA Message Definitions.
//*****************************************************************************
#define GSA_FIX_MODE        1
#define GSA_FIX_STATUS      2
#define GSA_SV_IDS_FIRST    3
#define GSA_SV_IDS_LAST    14
#define GSA_PDOP           15
#define GSA_HDOP           16
#define GSA_VDOP           17
#define GSA_SV_CHAR_LEN         3

//*****************************************************************************
// GSV Message Definitions.
//*****************************************************************************
#define GSV_TOT_MSGS        1
#define GSV_MSG_NUM     2
#define GSV_NUM_VIEW_SVS    3
#define GSV_SV_ID_START     4
#define GSV_SV_ELEV_START   5
#define GSV_SV_AZI_START    6
#define GSV_SV_SNR_START    7
#define GSV_SV_NEXT_OFFSET  4
#define GSV_SV_MAX_PER_MSG  4
#define GSV_SV_AZI_MAX_LEN  3
#define GSV_SV_ID_MAX_LEN   2
#define GSV_SV_ELEV_MAX_LEN 2
#define GSV_SV_SNR_MAX_LEN  2

#endif /* DEFINE_GPS_H_ */

// *****************************************************************************
// end of file
// *****************************************************************************
