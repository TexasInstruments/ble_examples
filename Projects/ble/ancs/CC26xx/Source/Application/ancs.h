/*
 * ancs.h
 *
 * Shared ANCS demo structures
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef ANCS_H
#define ANCS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// ANCS discovery states
enum
{
  DISC_IDLE = 0x00,                  // Idle state
  
  DISC_ANCS_START = 0x10,            // ANCS service
  DISC_ANCS_SVC,                     // Discover service
  DISC_ANCS_CHAR,                    // Discover all characteristics
  DISC_ANCS_CCCD,                    // Discover ANCS CCCD
  
  DISC_FAILED = 0xFF                 // Discovery failed
};

// ANCS handle cache indexes
enum
{
  HDL_ANCS_NTF_NOTIF_START,             // ANCS notification characteristic start handle
  HDL_ANCS_NTF_NOTIF_END,               // ANCS notification characteristic end handle
  HDL_ANCS_NTF_CCCD,                    // ANCS notification CCCD
  
  HDL_ANCS_CTRL_PT_START,
  HDL_ANCS_CTRL_PT_END,
  
  HDL_ANCS_DATA_SRC_START,             // ANCS data source characteristic start handle
  HDL_ANCS_DATA_SRC_END,               // ANCS data source characteristic end handle
  HDL_ANCS_DATA_SRC_CCCD,              // ANCS data source CCCD
  
  
  HDL_CACHE_LEN
};




  
/*********************************************************************
 * TYPEDEFS
 */
typedef int32 notificationUID_t;

typedef struct
{
  uint8  attrID;            //
  uint16 maxLen;            // Some attributes need to have length
} ctrlPtCmdParamAttrWithLen_t;

typedef struct
{
  uint8  attrID;            //
} ctrlPtCmdParamAttr_t;

typedef struct _ancsCSKey_t
{
    uint_least16_t hwikey;
    uint_least16_t taskkey;  
} _ancsCSKey_t;
/*********************************************************************
 * MACROS
 */

// CommandID Values
#define CommandIDGetNotificationAttributes      0       // CommandIDGetNotificationAttributes
#define CommandIDGetAppAttributes               1       // CommandIDGetAppAttributes
#define CommandIDPerformNotificationAction      2       // CommandIDPerformNotificationAction

#define ActionIDPositive        0
#define ActionIDNegative        1

// Notification AttributeID Values
#define NotificationAttributeIDAppIdentifier            0       // 
#define NotificationAttributeIDTitle                    1       // (Needs to be followed by a 2-bytes max length parameter)
#define NotificationAttributeIDSubtitle                 2       // (Needs to be followed by a 2-bytes max length parameter)
#define NotificationAttributeIDMessage                  3       // (Needs to be followed by a 2-bytes max length parameter)
#define NotificationAttributeIDMessageSize              4 
#define NotificationAttributeIDDate                     5 
#define NotificationAttributeIDPositiveActionLabel      6 
#define NotificationAttributeIDNegativeActionLabel      7

// EventID Values
#define EventIDNotificationAdded        0
#define EventIDNotificationModified     1
#define EventIDNotificationRemoved      2

// EventFlags
#define EventFlagSilent             0x01    // (1 << 0)
#define EventFlagImportant          0x02    // (1 << 1)
#define EventFlagPreExisting        0x04    // (1 << 2)
#define EventFlagPositiveAction     0x08    // (1 << 3)
#define EventFlagNegativeAction     0x10    // (1 << 4)

// CategoryID Values
#define CategoryIDOther                 0
#define CategoryIDIncomingCall          1
#define CategoryIDMissedCall            2
#define CategoryIDVoicemail             3
#define CategoryIDSocial                4
#define CategoryIDSchedule              5
#define CategoryIDEmail                 6
#define CategoryIDNews                  7
#define CategoryIDHealthAndFitness      8
#define CategoryIDBusinessAndFinance    9
#define CategoryIDLocation              10
#define CategoryIDEntertainment         11

//Define ANCS Client Flags
#define CLIENT_NONE             0x00
#define CLIENT_IMPORTANT_ALERT  0x01
#define CLIENT_POSITIVE_ACT     0x02
#define CLIENT_NEG_ACT          0x04


/*********************************************************************
 * GLOBAL
 */
enum
{
  CCCD_CONFIG_NOTIF = 0x00,
  CCCD_CONFIG_DATA,
  CCCD_CONFIG_DONE
};


// Connection handle
extern uint16_t Ancs_connHandle;

// Handle cache
extern uint16_t Ancs_handleCache[HDL_CACHE_LEN];

/*********************************************************************
 * FUNCTIONS
 */

/* 
 * ANCS service discovery functions.
 */
extern uint8_t Ancs_discStart(void);
extern uint8_t Ancs_discGattMsg(uint8_t state, gattMsgEvent_t *pMsg);
extern uint8_t Ancs_subsNotifSrc(void);
extern uint8_t Ancs_unSubsNotifSrc(void);
extern uint8_t Ancs_subsDataSrc(void);
extern uint8_t Ancs_unSubsDataSrc(void);
extern void Ancs_performPositiveAction();
extern void Ancs_performNegativeAction();

/* 
 * ANCS notification handling function.
 */
extern void Ancs_handleNotification(gattMsgEvent_t *pMsg);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ANCS_H */
