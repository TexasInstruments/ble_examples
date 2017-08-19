/******************************************************************************

 @file       ancs.h

 @brief This file contains the ANCS Application sample application for use
        with the CC2640R2 Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************

 Copyright (c) 2017-2017, Texas Instruments Incorporated
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
 Release Name: simplelink_cc2640r2_sdk_1_40_00_45
 Release Date: 2017-07-20 17:16:59
 *****************************************************************************/

#ifndef ANCS_H
#define ANCS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
/*********************************************************************
 * CONSTANTS
 */

// ANCS discovery states.
enum
{
  ANCS_IDLE,                        // Initial idle state.
  ANCS_EXCHANGE_MTU,                // Wait for the MTU exchange to take place.
  ANCS_DISC_SERVICE,                // Discover the ANCS service by UUID.
  ANCS_STORE_SERVICE_HANDLES,       // Store the ANCS service handles in the cache.
  ANCS_DISC_CHARS,                  // Discover the three characteristics: Notification Source, Control Point, and Data Source.
  ANCS_STORE_CHARS_HANDLES,         // Store the handles of each characteristic in the handle cache.
  ANCS_DISC_NS_DESCS,               // Discover the descriptors of the Notification Source (Trying to locate the CCCD).
  ANCS_STORE_NS_DESCS_HANDLES,      // Store the descriptor's handles in the handle cache (ANCS_NOTIF_SCR_HDL_CCCD).
  ANCS_DISC_DS_DESCS,               // Discover the descriptors of the Data Source (Trying to locate the CCCD).
  ANCS_STORE_DS_DESCS_HANDLES,      // Store the descriptor's handles in the handle cache (ANCS_DATA_SRC_HDL_CCCD).
  ANCS_WRITE_DS_CCCD,               // Perform a write to the Notification Source's CCCD to subscribe for notifications.
  ANCS_WRITE_NS_CCCD,               // Perform a write to the Data Source's CCCD to subscribe for notifications.
  ANCS_DISC_FINISH,                 // Final state signifying the end of the discovery process.

  ANCS_DISC_FAILED = 0xFF           // A failure state reached only if an error occurs.
};

// ANCS handle cache indices.
enum
{
  ANCS_NOTIF_SCR_HDL_START,         // ANCS Notification Source characteristic start handle.
  ANCS_NOTIF_SCR_HDL_END,           // ANCS Notification Source characteristic end handle.
  ANCS_NOTIF_SCR_HDL_CCCD,          // ANCS Notification Source CCCD handle.
    
  ANCS_CTRL_POINT_HDL_START,        // ANCS Control Point characteristic start handle.
  ANCS_CTRL_POINT_HDL_END,          // ANCS Control Point characteristic end handle.
    
  ANCS_DATA_SRC_HDL_START,          // ANCS Data Source characteristic start handle.
  ANCS_DATA_SRC_HDL_END,            // ANCS Data Source characteristic end handle.
  ANCS_DATA_SRC_HDL_CCCD,           // ANCS Data Source CCCD handle.
};

// Cache array length.
#define HDL_CACHE_LEN            8

// States for the notification attribute retrieval state machine (Ancs_handleNotifAttrRsp()). 
enum
{
  NOTI_ATTR_ID_BEGIN,                 // ANCS notification attribute initial retrieval state.
      
  NOTI_ATTR_ID_APPID,                 // ANCS notification attribute AppID retrieval state.
  APP_ATTR_ID_DN,                      // ANCS application attribute display name retrieval state.  
  NOTI_ATTR_ID_TITLE,                 // ANCS notification attribute Title retrieval state.
  NOTI_ATTR_ID_SUBTITLE,              // ANCS notification attribute Subtitle retrieval state.  
  NOTI_ATTR_ID_MESSAGE,               // ANCS notification attribute Message retrieval state.
  NOTI_ATTR_ID_MESSAGE_SIZE,          // ANCS notification attribute Message Size retrieval state.      
  NOTI_ATTR_ID_DATE,                  // ANCS notification attribute Date retrieval state. 

  NOTI_ATTR_ID_END                    // ANCS notification attribute final retrieval state.
};

// States for processing Data Source packets.
enum
{
  NOTI_ATTR_FIRST_PKT,                // Initial retrieved Data Source packet processing state.
  NOTI_ATTR_CONTINUE_PKT,             // Post-Initial retrieved Data Source packet processing state.
};


/*********************************************************************
 * MACROS
 */

// Number of bytes required to store an ANCS notification UID
#define ANCS_NOTIF_UID_LENGTH                                 4

// CommandID Values
#define COMMAND_ID_GET_NOTIFICATION_ATTRIBUTES                0x00
#define COMMAND_ID_GET_APP_ATTRIBUTES                         0x01
#define COMMAND_ID_PERFORM_NOTIFICATION_ACTION                0x02

#define ACTION_ID_POSITIVE                                    0
#define ACTION_ID_NEGATIVE                                    1

// Notification AttributeID Values
#define NOTIFICATION_ATTRIBUTE_ID_APP_IDENTIFIER              0       
#define NOTIFICATION_ATTRIBUTE_ID_TITLE                       1       
#define NOTIFICATION_ATTRIBUTE_ID_SUBTITLE                    2       
#define NOTIFICATION_ATTRIBUTE_ID_MESSAGE                     3       
#define NOTIFICATION_ATTRIBUTE_ID_MESSAGE_SIZE                4 
#define NOTIFICATION_ATTRIBUTE_ID_DATE                        5 
#define NOTIFICATION_ATTRIBUTE_ID_POSITIVE_ACTION_LABEL       6 
#define NOTIFICATION_ATTRIBUTE_ID_NEGATIVE_ACTION_LABEL       7

// EventID Values
#define EVENT_ID_NOTIFICATION_ADDED                           0
#define EVENT_ID_NOTIFICATION_MODIFIED                        1
#define EVENT_ID_NOTIFICATION_REMOVED                         2

// EventFlags
#define EVENT_FLAG_SILENT                                     0x01    
#define EVENT_FLAG_IMPORTANT                                  0x02    
#define EVENT_FLAG_PREEXISTING                                0x04    
#define EVENT_FLAG_POSITIVE_ACTION                            0x08    
#define EVENT_FLAG_NEGATIVE_ACTION                            0x10    

// CategoryID Values
#define CATEGORY_ID_OTHER                                     0
#define CATEGORY_ID_INCOMING_CALL                             1
#define CATEGORY_ID_MISSED_CALL                               2
#define CATEGORY_ID_VOICEMAIL                                 3
#define CATEGORY_ID_SOCIAL                                    4
#define CATEGORY_ID_SCHEDULE                                  5
#define CATEGORY_ID_EMAIL                                     6
#define CATEGORY_ID_NEWS                                      7
#define CATEGORY_ID_HEALTH_AND_FITNESS                        8
#define CATEGORY_ID_BUSINESS_AND_FINANCE                      9
#define CATEGORY_ID_LOCATION                                  10
#define CATEGORY_ID_ENTERTAINMENT                             11


// Define ANCS Client Flags
#define CLIENT_NONE                                           0x00
#define CLIENT_IMPORTANT_ALERT                                0x01
#define CLIENT_POSITIVE_ACT                                   0x02
#define CLIENT_NEG_ACT                                        0x04

// Error Codes received from Control Point
#define UNKNOWN_COMMAND                                       0xA0
#define INVALID_COMMAND                                       0xA1
#define INVALID_PARAMETER                                     0xA2
#define ACTION_FAILED                                         0xA3

// AppAttributeID values
#define APP_ATTRIBUTE_ID_DISPLAY_NAME                         0x00

// ANCS Control Point action length.
#define PERFORM_NOTIFICATION_ACTION_LENGTH                    6
                 
/*********************************************************************
 * GLOBAL
 */
// The iPhones Connection handle.
extern uint16_t Ancs_connHandle;

// The ANCS handle cache.
extern uint16_t Ancs_handleCache[HDL_CACHE_LEN];

// Stores Data Service notification processing state.
extern uint8_t notifAttrPktProcessState;

// Stores Data Service app attribute processing state
extern uint8_t appAttrPktProcessState;
/*********************************************************************
 * FUNCTIONS
 */

// ANCS service discovery functions.
extern uint8_t Ancs_subsNotifSrc(void);
extern uint8_t Ancs_subsDataSrc(void);
 
// ANCS notification handling function.
extern void Ancs_processDataServiceNotif(gattMsgEvent_t *pMsg);
extern void Ancs_queueNewNotif(gattMsgEvent_t *pMsg);
extern void Ancs_popAllNotifsFromQueue(void);
extern void Ancs_acceptIncomingCall(void);
extern void Ancs_declineIncomingCall(void);


/*********************************************************************
 *********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ANCS_H */
