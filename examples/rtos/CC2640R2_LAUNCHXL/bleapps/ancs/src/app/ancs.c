/******************************************************************************

 @file       ancs.c

 @brief This file contains the ANCS Application sample application for use
        with the CC2640R2 Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************

 Copyright (c) 2013-2017, Texas Instruments Incorporated
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


/*********************************************************************
 * INCLUDES
 */
#include "string.h"
#include "bcomdef.h"
#include <ti/display/Display.h>
#include "Board.h"
#include "ancs.h"
#include "ancsApp.h"
#include "icall_ble_api.h"


/*********************************************************************
 * CONSTANTS
 */

#define NOTIF_ATTR_REQUEST_METADATA_LENGTH        0x08
#define DATA_BUFFER_SIZE                          100

#define REQUESTED_ATTR_ID_APPID                   0x01
#define REQUESTED_ATTR_ID_DS                      0x02
#define REQUESTED_ATTR_ID_TITLE                   0x04  
#define REQUESTED_ATTR_ID_SUBTITLE                0x08
#define REQUESTED_ATTR_ID_MESSAGE                 0x10 
#define REQUESTED_ATTR_ID_MESSAGE_SIZE            0x20   
#define REQUESTED_ATTR_ID_DATE                    0x40

 
#define ATTR_APPID_REQUEST_SIZE                   0
#define ATTR_TITLE_REQUEST_SIZE                   20
#define ATTR_SUBTITLE_REQUEST_SIZE                30  
#define ATTR_MESSAGE_REQUEST_SIZE                 DATA_BUFFER_SIZE - 1  
#define ATTR_MESSAGE_SIZE_REQUEST_SIZE            0        
#define ATTR_DATE_REQUEST_SIZE                    0


/*********************************************************************
 * TYPEDEFS
 */

typedef struct
{
  uint8_t categoryID;
  uint8_t notificationUID[4];
  uint8_t currentState;
  uint8_t requestedAttrs;
} notifQueueData_t;

typedef struct notifQueueNode_t
{
  notifQueueData_t notifData;
  struct notifQueueNode_t *pNext;
} notifQueueNode_t;


/*********************************************************************
 * GLOBAL VARIABLES
 */
// Stores the connection handle to the iPhone.
uint16_t Ancs_connHandle;

// Stores the state of the Data Source GATT notification processing function (Ancs_processDataServiceNotif()).
uint8_t notifAttrPktProcessState;

// Stores the state of the Data Source GATT notification processing function (Ancs_processAppAttr()).
uint8_t appAttrPktProcessState;
/*********************************************************************
 * LOCAL VARIABLES
 */

// Used to stored the retrieved attribute data. 
static uint8_t  dataBuf[DATA_BUFFER_SIZE] = { '\0' };

// Stores the length of the retrieved attribute data.
static uint16_t dataLen = 0x0000;

// Stores the notification ID of the head of the queue.
static uint8_t currentNotifUID[ANCS_NOTIF_UID_LENGTH] = { 0x00 };
static uint8_t incomingCallUID[ANCS_NOTIF_UID_LENGTH] = { 0x00 };

// Holds the value for whether or not there is an incomingCall type notification in the queue.
static bool haveIncomingCall = FALSE;

// Points to the head of the queue.
static notifQueueNode_t *pNotifQueueFront = NULL;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

// Basic link-list structure queue functions.
static void Ancs_findAndRemoveFromQueue(uint8_t *pNotifUID);
static notifQueueNode_t* Ancs_findNotifInQueue(uint8_t *pNotifUID);
       void Ancs_popAllNotifsFromQueue(void);
static void Ancs_popNotifFromQueue(void);
static void Ancs_pushNotifToQueue(uint8_t categoryID, uint8_t *pNotifUID);
static bool Ancs_queueEmpty(void);
static uint8_t Ancs_queueSize(void);

// Functions used to process incoming GATT notifications from the 
// Notification Source and Data Source, and request additional data.
       void Ancs_acceptIncomingCall(void);
static uint8_t Ancs_CCCDConfig(uint16_t attrHdl, uint8_t isEnable);
       void Ancs_declineIncomingCall(void);
static uint8_t Ancs_getNotifAttr(uint8_t *pNotificationUID, uint8_t attributeID, uint16_t len);
static uint8_t Ancs_getAppAttr(uint8_t *appID, uint8_t attributeID);
static void Ancs_handleNotifAttrRsp(uint8_t *pNotificationUID);
static uint8_t Ancs_performNegativeAction(uint8_t *notifUID);
static uint8_t Ancs_performPositiveAction(uint8_t *notifUID);
static void Ancs_processAppAttr(gattMsgEvent_t *pMsg);
       void Ancs_processDataServiceNotif(gattMsgEvent_t *pMsg);
static void Ancs_processNotifications(void);
static void Ancs_processNotificationServiceNotif(notifQueueNode_t *pNotif);
static void Ancs_printNotifDate(uint8_t*  dataBuf);
       void Ancs_queueNewNotif(gattMsgEvent_t *pMsg);
       uint8_t Ancs_subsDataSrc(void);
       uint8_t Ancs_subsNotifSrc(void);

/*********************************************************************
 * QUEUE FUNCTIONS (In alphabetical order)
 */



/*********************************************************************
 * @fn      Ancs_findAndRemoveFromQueue
 *
 * @brief   Find a specific notification and remove it from the queue
 *
 * @param   pNotifUID  - notification UID
 *
 * @return  none
 */
static void Ancs_findAndRemoveFromQueue(uint8_t *pNotifUID)
{
  if (pNotifQueueFront == NULL)
    return;
  notifQueueNode_t *pSearch;
  notifQueueNode_t *pSearchLast;
  pSearch       = pNotifQueueFront;
  pSearchLast   = pNotifQueueFront;

  uint8_t notifUID[ANCS_NOTIF_UID_LENGTH];
  VOID memcpy(notifUID, pNotifUID, ANCS_NOTIF_UID_LENGTH);

  while ((memcmp(notifUID, pSearch->notifData.notificationUID, ANCS_NOTIF_UID_LENGTH) != 0) && pSearch != NULL)
  {
    pSearchLast = pSearch;
    pSearch     = pSearch->pNext;
  }

  if (pSearch != NULL)
  {
    pSearchLast->pNext = pSearch->pNext;
    ICall_free(pSearch);
    Ancs_queueSize();
  }

  return;
}

/*********************************************************************
 * @fn      Ancs_findNotifInQueue
 *
 * @brief   Find a Notification & Category in the existing list
 *
 * @param   pNotifUID  - notification UID
 *
 * @return  none
 */
static notifQueueNode_t* Ancs_findNotifInQueue(uint8_t *pNotifUID)
{
  notifQueueNode_t *pSearch;
  uint8_t notifUID[ANCS_NOTIF_UID_LENGTH];

  VOID memcpy(notifUID, pNotifUID, ANCS_NOTIF_UID_LENGTH);

  pSearch = pNotifQueueFront;
  while ((memcmp(notifUID, pSearch->notifData.notificationUID, ANCS_NOTIF_UID_LENGTH) != 0) && pSearch != NULL)
  {
    pSearch = pSearch->pNext;
  }

  if (pSearch == NULL) // Not in the list
    return NULL;

  else
    return pSearch;
}

/*********************************************************************
 * @fn      Ancs_popAllNotifsFromQueue
 *
 * @brief   Clear the queue of all notifications
 *
 * @param   none   
 *
 * @return  none
 */
void Ancs_popAllNotifsFromQueue(void)
{
  notifQueueNode_t *pSearch;
  notifQueueNode_t *pDelete;

  pSearch = pNotifQueueFront;

  while (pSearch != NULL)
  {
    pDelete = pSearch;
    pSearch = pSearch->pNext;
    ICall_free(pDelete);
    pDelete = NULL;
  }
  pNotifQueueFront = NULL;
  
  return;
}

/*********************************************************************
 * @fn      Ancs_popNotifFromQueue
 *
 * @brief   Move to the front of the queue to the next element and delete the old front
 *
 * @param   none   
 *
 * @return  none
 */
static void Ancs_popNotifFromQueue(void)
{   
  if ( pNotifQueueFront == NULL)
    return;

  notifQueueNode_t *pFrontOld;
  pFrontOld = pNotifQueueFront;

  pNotifQueueFront = pNotifQueueFront->pNext;
  ICall_free(pFrontOld);
  Ancs_queueSize();

  return;
}

/*********************************************************************
 * @fn      Ancs_pushNotifToQueue
 *
 * @brief   Add a Notification & Category to list
 *
 * @param   categoryID - category ID of the notification
 * @param   pNotifUID  - notification UID
 *
 * @return  none
 */
static void Ancs_pushNotifToQueue(uint8_t categoryID, uint8_t *pNotifUID)
{
  notifQueueNode_t *pNew;
  notifQueueNode_t *pSearch;

  pNew = ICall_malloc(sizeof(notifQueueNode_t));
  if (pNew == NULL)
    return;

  // Store categoryID and notification ID.
  pNew->notifData.categoryID = categoryID;
  VOID memcpy(pNew->notifData.notificationUID, pNotifUID, ANCS_NOTIF_UID_LENGTH);
  pNew->notifData.currentState = NOTI_ATTR_ID_BEGIN;
  pNew->notifData.requestedAttrs = 0;
  pNew->pNext = NULL;

  if (pNotifQueueFront == NULL) // New list
  {
    pNotifQueueFront = pNew;
  }
  else  // Add to the last position of the list
  {
    pSearch = pNotifQueueFront;
    while (pSearch->pNext != NULL)
    {
      pSearch = pSearch->pNext;
    }
    pSearch->pNext = pNew;
  }
  Ancs_queueSize();

  return;
}

/*********************************************************************
 * @fn      Ancs_queueEmpty
 *
 * @brief   Indicate if the notification queue is empty or not.
 *
 * @param   none   
 *
 * @return  bool - Return TRUE if pNotifQueueFront equals NULL, FALSE else.
 */
static bool Ancs_queueEmpty(void)
{
  return (pNotifQueueFront == NULL);
}

/*********************************************************************
 * @fn      Ancs_queueSize
 *
 * @brief   Print the current size of the notification queue.  
 *
 * @param   none
 *
 * @return  uint8_t - Number of notifications in the queue.
 */
static uint8_t Ancs_queueSize(void)
{
  uint8_t notifCount = 0;
  notifQueueNode_t *pCount;
  pCount = pNotifQueueFront;

  while(pCount != NULL)
  {
    notifCount++;
    pCount = pCount->pNext;
  }
  Display_print1(dispHandle, 4, 0, "Total Notifications:\t%d", notifCount);

  return notifCount;
}

/*********************************************************************
 * NOTIFICATION FUNCTIONS (In alphabetical order)
 */

/*********************************************************************
 * @fn      Ancs_acceptIncomingCall
 *
 * @brief   Accept an incoming phone call.
 *
 * @param   none
 *
 * @return  none
 */
void Ancs_acceptIncomingCall(void)
{
  if(haveIncomingCall == TRUE)
  {
    Ancs_performPositiveAction(incomingCallUID);
    haveIncomingCall = FALSE;
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery State:\tAccept Incoming Call");
#endif
    Display_print0(dispHandle, 16, 0, "Accepted Incoming Call");
  }
}

/*********************************************************************
 * @fn      Ancs_CCCDConfig
 *
 * @brief   subscribe Notification Source. 
 *
 * @param   none.
 *
 * @return  uint8_t SUCCESS/FAILURE
 */
static uint8_t Ancs_CCCDConfig(uint16_t attrHdl, uint8_t isEnable)
{
  // Declare return variable status.
  uint8_t status;

  // Stores the GATT write request parameters.
  attWriteReq_t req;

  // Allocate memory for the request.
  req.pValue = GATT_bm_alloc(Ancs_connHandle, ATT_WRITE_REQ, 2, NULL);

  // If the allocation failed, return FAILURE.
  if (req.pValue == NULL)
    status = FAILURE;

  // Else, prepare the request.
  else
  {
    // Set the data length to 2 ("01" = 2 bytes).
    req.len = 2;

    // If we are enabling notifications, set the write data to "01".
    if (isEnable == TRUE)
    {
      req.pValue[0] = LO_UINT16(GATT_CLIENT_CFG_NOTIFY);
      req.pValue[1] = HI_UINT16(GATT_CLIENT_CFG_NOTIFY);
    }

    // Else, disable notifications, thus set the write data to "00".
    else
    {
      req.pValue[0] = 0x00;
      req.pValue[1] = 0x00;
    }

    // Signature and command must be set to zero.
    req.sig = 0;
    req.cmd = 0;

    // Set the handle to the passed value (either the Notification Source's CCCD handle
    // or the Data Source's CCCD handle).
    req.handle = attrHdl;

    // Send write request. If it fails, free the memory allocated and 
    // return a failure.
    status = GATT_WriteCharValue(Ancs_connHandle, &req, ICall_getEntityId());
    if ( status != SUCCESS)
      GATT_bm_free((gattMsg_t *) &req, ATT_WRITE_REQ);
  }

  return status;
}

/*********************************************************************
 * @fn      Ancs_declineIncomingCall
 *
 * @brief   Reject an incoming phone call.
 *
 * @param   none
 *
 * @return  none
 */
void Ancs_declineIncomingCall(void)
{
  if(haveIncomingCall == TRUE)
  {
      Ancs_performNegativeAction(incomingCallUID);
      haveIncomingCall = FALSE;
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery State:\tDeclined Incoming Call");
#endif
    Display_print0(dispHandle, 16, 0, "Declined Incoming Call");
  }
}

/*********************************************************************
 * @fn      Ancs_getNotifAttr
 *
 * @brief   Get notification attributes. 
 *
 * @param   pNotificationUID - notification's ID.
 *
 * @param   attributeID - attribute's ID.
 *
 * @return  uint8_t SUCCESS/FAILURE
 */
static uint8_t Ancs_getNotifAttr(uint8_t *pNotificationUID, uint8_t attributeID, uint16_t len)
{
  uint8_t status;
  uint8_t cmdLen = 8;
  if (len == 0)
    cmdLen = 6;

  // Do a write
  attWriteReq_t req;

  req.pValue = GATT_bm_alloc(Ancs_connHandle, ATT_WRITE_REQ, cmdLen, NULL);
  uint8_t *requestPayload = req.pValue;
  if (req.pValue != NULL)
  {
    // Get the ANCS control point handle.
    req.handle = Ancs_handleCache[ANCS_CTRL_POINT_HDL_START];

    // Set command length.
    req.len = cmdLen;

    // Set Command ID.
    *requestPayload = COMMAND_ID_GET_NOTIFICATION_ATTRIBUTES;
    requestPayload++;

    // Set NotificationUID
    VOID memcpy(requestPayload, pNotificationUID, ANCS_NOTIF_UID_LENGTH);
    requestPayload += ANCS_NOTIF_UID_LENGTH;

    // Set attributeID
    *requestPayload = attributeID;
    requestPayload++;

    // Set length to desired max length to be retrieved.
    *requestPayload = LO_UINT16(len);       
    requestPayload++;
    *requestPayload = HI_UINT16(len);

    // Signature and command must be set to zero.
    req.sig = 0;
    req.cmd = 0;

    // Execute the write.
    status = GATT_WriteCharValue(Ancs_connHandle, &req, ICall_getEntityId());
    if (status != SUCCESS)
    {
      // If it fails free the message.
      Display_print1(dispHandle, 10, 0, "CP WRITE ERROR:\t%d",status);
      GATT_bm_free((gattMsg_t *) &req, ATT_WRITE_REQ);
    }
  }

  return status;
}

/*********************************************************************
 * @fn      Ancs_getAppAttr
 *
 * @brief   Get application attributes.
 *
 * @param   appID - applciation's ID
 *
 * @param   attributeID - attribute's ID.
 *
 * @return  uint8_t SUCCESS/FAILURE
 */
static uint8_t Ancs_getAppAttr(uint8_t *appID, uint8_t attributeID)
{
  uint8_t status;

  uint8_t *lenCheck = appID;
  uint8_t appIDLen = 0;

  while(*lenCheck != '\0')
  {
    lenCheck++;
    appIDLen++;
  }

  // Add 1 for the NULL terminator.
  appIDLen++;


  // 1 for Command ID, Length of the AppID, 1 for the AttrID
  uint8_t cmdLen = 1 + appIDLen + 1;

  // Do a write
  attWriteReq_t req;

  req.pValue = GATT_bm_alloc(Ancs_connHandle, ATT_WRITE_REQ, cmdLen, NULL);
  uint8_t *requestPayload = req.pValue;
  if (req.pValue != NULL)
  {
    // Get the ANCS control point handle.
    req.handle = Ancs_handleCache[ANCS_CTRL_POINT_HDL_START];

    // Set command length.
    req.len = cmdLen;

    // Set Command ID.
    *requestPayload = COMMAND_ID_GET_APP_ATTRIBUTES;
    requestPayload++;

    // Set AppID
    VOID memcpy(requestPayload, appID, appIDLen);
    requestPayload += appIDLen;

    // Set attributeID
    *requestPayload = attributeID;
    requestPayload++;

    // Signature and command must be set to zero.
    req.sig = 0;
    req.cmd = 0;

    // Execute the write.
    status = GATT_WriteCharValue(Ancs_connHandle, &req, ICall_getEntityId());
    if (status != SUCCESS)
    {
      // If it fails free the message.
      Display_print1(dispHandle, 10, 0, "CP WRITE ERROR:\t%d",status);
      GATT_bm_free((gattMsg_t *) &req, ATT_WRITE_REQ);
    }
  }

  return status;
}

/*********************************************************************
 * @fn      Ancs_handleNotifAttrRsp
 *
 * @brief   Handle response value of Notification Attributes from iOS device.
 *
 * @param   attrID - attributes ID.
 *
 * @return  uint8_t SUCCESS/FAILURE
 */
static void Ancs_handleNotifAttrRsp(uint8_t *pNotificationUID)
{
  // Will store the queue node with the passed notification UID.
  notifQueueNode_t *pNode;

  // Retrieve the notification with the passed UID.
  pNode = Ancs_findNotifInQueue(pNotificationUID);

  switch ( pNode->notifData.currentState )
  {
  // The initial state is used to kick-off the state machine and 
  // immediately proceed to the AppID state (hence the missing break).   
  case NOTI_ATTR_ID_BEGIN:
    pNode->notifData.currentState = NOTI_ATTR_ID_APPID;

  // If the AppID request flag hasn't been set, request the AppID attribute
  // data and set the AppID request flag.
  case NOTI_ATTR_ID_APPID:
    if( !(pNode->notifData.requestedAttrs & REQUESTED_ATTR_ID_APPID) )
    {
      Ancs_getNotifAttr(pNotificationUID, NOTIFICATION_ATTRIBUTE_ID_APP_IDENTIFIER, ATTR_APPID_REQUEST_SIZE);
      pNode->notifData.requestedAttrs |= REQUESTED_ATTR_ID_APPID;
    }
    break;

  case APP_ATTR_ID_DN:
    if( !(pNode->notifData.requestedAttrs & REQUESTED_ATTR_ID_DS) )
    {
      Ancs_getAppAttr(dataBuf, APP_ATTRIBUTE_ID_DISPLAY_NAME);
      pNode->notifData.requestedAttrs |= REQUESTED_ATTR_ID_DS;
    }
    break;

  // If the Title request flag hasn't been set, request the Title attribute
  // data and set the Title request flag.
  case NOTI_ATTR_ID_TITLE:
    if( !(pNode->notifData.requestedAttrs & REQUESTED_ATTR_ID_TITLE) )
    {
      Ancs_getNotifAttr(pNotificationUID, NOTIFICATION_ATTRIBUTE_ID_TITLE, ATTR_TITLE_REQUEST_SIZE);
      pNode->notifData.requestedAttrs |= REQUESTED_ATTR_ID_TITLE;
    }
    break;

  // If the Subtitle request flag hasn't been set, request the Subtitle attribute
  // data and set the Subtitle request flag.
  case NOTI_ATTR_ID_SUBTITLE:
    if( !(pNode->notifData.requestedAttrs & REQUESTED_ATTR_ID_SUBTITLE) )
    {
      Ancs_getNotifAttr(pNotificationUID, NOTIFICATION_ATTRIBUTE_ID_SUBTITLE, ATTR_SUBTITLE_REQUEST_SIZE);
      pNode->notifData.requestedAttrs |= REQUESTED_ATTR_ID_SUBTITLE;
    }
    break;

  // If the Message request flag hasn't been set, request the Message attribute
  // data and set the Message request flag.
  case NOTI_ATTR_ID_MESSAGE:
    if( !(pNode->notifData.requestedAttrs & REQUESTED_ATTR_ID_MESSAGE) )
    {
      Ancs_getNotifAttr(pNotificationUID, NOTIFICATION_ATTRIBUTE_ID_MESSAGE, ATTR_MESSAGE_REQUEST_SIZE);
      pNode->notifData.requestedAttrs |= REQUESTED_ATTR_ID_MESSAGE;
    }
    break;

  // If the Message Size request flag hasn't been set, request the Message Size attribute
  // data and set the Message Size request flag.
  case NOTI_ATTR_ID_MESSAGE_SIZE:
    if( !(pNode->notifData.requestedAttrs & REQUESTED_ATTR_ID_MESSAGE_SIZE) )
    {
      Ancs_getNotifAttr(pNotificationUID, NOTIFICATION_ATTRIBUTE_ID_MESSAGE_SIZE, ATTR_MESSAGE_SIZE_REQUEST_SIZE);
      pNode->notifData.requestedAttrs |= REQUESTED_ATTR_ID_MESSAGE_SIZE;
    }     
    break;

  // If the Date request flag hasn't been set, request the Date attribute
  // data and set the Date request flag.
  case NOTI_ATTR_ID_DATE:
    if( !(pNode->notifData.requestedAttrs & REQUESTED_ATTR_ID_DATE) )
    {
      Ancs_getNotifAttr(pNotificationUID, NOTIFICATION_ATTRIBUTE_ID_DATE, ATTR_DATE_REQUEST_SIZE);
      pNode->notifData.requestedAttrs |= REQUESTED_ATTR_ID_DATE;
    }
    break;

  // End state, do nothing but signify all requests have been made.
  case NOTI_ATTR_ID_END:
    break;

  default:
    break;

  }
  return;
}


/*********************************************************************
 * @fn      Ancs_processAppAttr
 *
 * @brief   Extract and reassemble the retrieved data from the Data Source notification (App attributes)
 *
 * @param   pMsg - GATT message.
 *
 * @return  none
 */
static void Ancs_processAppAttr(gattMsgEvent_t *pMsg)
{
  // Pointer to the GATT Msg data.
  uint8_t *packetData;

  // The variable that will keep track of the ANCS attribute
  // currently being processed.
  static uint8_t AttrID;

  // The variable will keep track of the current index the data
  // buffer will be written to if more than one packet is needed.
  static uint8_t currentDataBufWriteIndex;

  // Point to the GATT Msg data
  packetData = pMsg->msg.handleValueNoti.pValue;

  // Check if this is the first retrieved packet for potentially
  // a set of packets to be sent by the ANCS Data Service.
  switch(notifAttrPktProcessState)
  {
    case NOTI_ATTR_FIRST_PKT:
    {
      // Tracks the metadata length of the first packet.
      uint8_t responseLen = 0;

      // Initialize the data buffer write index to zero, as this is
      // the first packet.
      currentDataBufWriteIndex = 0;

      // Ensure Command ID is equal to zero as stated in the spec.
      if (*packetData != COMMAND_ID_GET_APP_ATTRIBUTES)
          return;
      packetData++;
      responseLen++;

      // Skip the appID.
      while(*packetData != '\0')
      {
        packetData++;
        responseLen++;
      }

      // Skip the NULL terminator.
      packetData++;
      responseLen++;

      // Store the ANCS attribute ID of the retrieved attribute.
      AttrID = *packetData;
      packetData++;
      responseLen++;

      // Store the 2-byte length of the data that is being retrieved.
      dataLen = BUILD_UINT16(*packetData, *(packetData + 1));

      // Check if the length is zero, if so the notification does not
      // have the specified attribute as stated in the ANCS spec.
      if ( dataLen == 0 )
      {
        if (AttrID == APP_ATTRIBUTE_ID_DISPLAY_NAME)
          Display_print0(dispHandle, 7, 0, "* App Name:\tNot available");
        // pNotifQueueFront->notifData.currentState++;
        pNotifQueueFront->notifData.currentState++;
        Ancs_processNotifications();

        return;
      }

      // Move the pointer to the data portion.
      packetData += 2;
      responseLen += 2;

      // Clear the data buffer in preparation for the new data.
      VOID memset(dataBuf, '\0', DATA_BUFFER_SIZE);

      // If the data length specified in the first ANCS Data Service notification
      // is greater than the number of bytes that was sent in the
      // first data packet(total GATT msg length - request metadata), the data will be split into multiple packets.
      if (dataLen > pMsg->msg.handleValueNoti.len - responseLen)
      {
        // Copy the number of bytes that were sent in the
        // first packet to the data buffer, then set the
        // data buffer write index, and set the state from first packet
        // to continued packet.
        VOID memcpy(dataBuf, packetData, (pMsg->msg.handleValueNoti.len - responseLen));
        currentDataBufWriteIndex = (pMsg->msg.handleValueNoti.len - responseLen);
        appAttrPktProcessState = NOTI_ATTR_CONTINUE_PKT;

        // Subtract the number of data bytes contained in the first packet
        // from the total number of expected data bytes.
        dataLen -= (pMsg->msg.handleValueNoti.len - responseLen);
        return;
      }
      else
      {
        // In this case all the ANCS attribute data was contained in
        // the first packet so the data is copied, and both the index and
        // length are reset.
        VOID memcpy(dataBuf, packetData, dataLen);
        currentDataBufWriteIndex = 0;
        dataLen = 0x0000;
      }
    }

    case NOTI_ATTR_CONTINUE_PKT:
    {
      if (dataLen > 0)
      {
        // Copy all the data from the notification packet to the data buffer
        // starting from the current data buffer write index.
        VOID memcpy(dataBuf + currentDataBufWriteIndex, pMsg->msg.handleValueNoti.pValue,
                                                   pMsg->msg.handleValueNoti.len);
        // Subtract the number of data bytes contained in the packet from
        // the total, and increase the data buffer write index by that amount.
        dataLen -= pMsg->msg.handleValueNoti.len;
        currentDataBufWriteIndex += pMsg->msg.handleValueNoti.len;

        // Checks if this is the last the continued packet.
        if (dataLen == 0x0000)
        {
          // If so reset the write index and the state.
          currentDataBufWriteIndex = 0;
          notifAttrPktProcessState = NOTI_ATTR_FIRST_PKT;
        }
      }
    }
    break;

    default:
    break;
  }

  // Now we have real data, to display it on LCD for demo now,
  // customer needs to change it from here to deal with data
  if (dataLen == 0)
  {
    pNotifQueueFront->notifData.currentState++;

    if (AttrID == APP_ATTRIBUTE_ID_DISPLAY_NAME)
      Display_print1(dispHandle, 7, 0, "* App Name:\t%s", (char* )dataBuf);
  }
  // Check if the dataLen variable was overflowed if packets got mismatched.
  // This may occur if rapid connecting and disconnecting to the device is performed.
  else if(dataLen > DATA_BUFFER_SIZE - 1)
  {
    Display_print0(dispHandle, 7,  0, "* App Name:\tDATA CURRUPTED");
    Ancs_findAndRemoveFromQueue(currentNotifUID);
    notifAttrPktProcessState = NOTI_ATTR_FIRST_PKT;
  }
  // Continue processing the current notification.
  Ancs_processNotifications();
}

/*********************************************************************
 * @fn      Ancs_processDataServiceNotif
 *
 * @brief   Extract and reassemble the retrieved data from the Data Source notifications
 *
 * @param   pMsg - GATT message.
 *
 * @return  none
 */
void Ancs_processDataServiceNotif(gattMsgEvent_t *pMsg)
{
  // Pointer to the GATT Msg data.
  uint8_t *packetData;

  // The variable that will keep track of the ANCS attribute
  // currently being processed.
  static uint8_t AttrID;

  // The variable will keep track of the current index the data
  // buffer will be written to if more than one packet is needed.
  static uint8_t  currentDataBufWriteIndex;

  // Point to the GATT Msg data
  packetData = pMsg->msg.handleValueNoti.pValue;

  // Check if this is the first retrieved packet for potentially
  // a set of packets to be sent by the ANCS Data Service.
  switch(notifAttrPktProcessState)
  {
    case NOTI_ATTR_FIRST_PKT:
    {
      // Initialize the data buffer write index to zero, as this is
      // the first packet.
      currentDataBufWriteIndex = 0;
  
      // Ensure Command ID is equal to zero as stated in the spec.
      if (*packetData != COMMAND_ID_GET_NOTIFICATION_ATTRIBUTES)
      {
        if(*packetData == COMMAND_ID_GET_APP_ATTRIBUTES)
        {
          Ancs_processAppAttr(pMsg);
        }
        return;
      }
      packetData++;
  
      // Copy the ANCS notification UID so it may be used by
      // to perform a positive or negative action is desired.
      VOID memcpy(currentNotifUID, packetData, ANCS_NOTIF_UID_LENGTH);

      packetData += ANCS_NOTIF_UID_LENGTH;
  
      // Store the ANCS attribute ID of the retrieved attribute.
      AttrID = *packetData;
      packetData++;
  
      // Store the 2-byte length of the data that is being retrieved.
      dataLen = BUILD_UINT16(*packetData, *(packetData + 1));
  
      // Check if the length is zero, if so the notification does not
      // have the specified attribute as stated in the ANCS spec.
      if ( dataLen == 0 )
      {
        if (AttrID == NOTIFICATION_ATTRIBUTE_ID_APP_IDENTIFIER)
          Display_print0(dispHandle, 7, 0, "* AppID:\tNot available");
        else if (AttrID == NOTIFICATION_ATTRIBUTE_ID_TITLE)
          Display_print0(dispHandle, 8, 0, "* Title:\tNot available");
        else if (AttrID == NOTIFICATION_ATTRIBUTE_ID_MESSAGE)
        {
          Display_print0(dispHandle, 9, 0, "* Message:\tNot available");
        }
        pNotifQueueFront->notifData.currentState++;

        Ancs_processNotifications();
        
        return;
      }
      // Move the pointer to the data portion.
      packetData += 2;
  
      // Clear the data buffer in preparation for the new data.
      VOID memset(dataBuf, '\0', DATA_BUFFER_SIZE);
  
      // If the data length specified in the first ANCS Data Service notification
      // is greater than the number of bytes that was sent in the 
      // first data packet(total GATT msg length - request metadata), the data will be split into multiple packets.
      if (dataLen > pMsg->msg.handleValueNoti.len - NOTIF_ATTR_REQUEST_METADATA_LENGTH)
      {
        // Copy the number of bytes that were sent in the 
        // first packet to the data buffer, then set the 
        // data buffer write index, and set the state from first packet
        // to continued packet.
        VOID memcpy(dataBuf, packetData, (pMsg->msg.handleValueNoti.len - NOTIF_ATTR_REQUEST_METADATA_LENGTH));
        currentDataBufWriteIndex = (pMsg->msg.handleValueNoti.len - NOTIF_ATTR_REQUEST_METADATA_LENGTH);
        notifAttrPktProcessState = NOTI_ATTR_CONTINUE_PKT;
  
        // Subtract the number of data bytes contained in the first packet
        // from the total number of expected data bytes.
        dataLen -= (pMsg->msg.handleValueNoti.len - NOTIF_ATTR_REQUEST_METADATA_LENGTH);
        return;
      }
      else
      {
        // In this case all the ANCS attribute data was contained in
        // the first packet so the data is copied, and both the index and 
        // length are reset. 
        VOID memcpy(dataBuf, packetData, dataLen);
        currentDataBufWriteIndex = 0;
        dataLen = 0x0000;
      }
    }
    break;

    // Check if the is a continued data packet.
    case NOTI_ATTR_CONTINUE_PKT:
    {  
      if (dataLen > 0)
      {
        // Copy all the data from the notification packet to the data buffer
        // starting from the current data buffer write index.
        VOID memcpy(dataBuf + currentDataBufWriteIndex, pMsg->msg.handleValueNoti.pValue,
                                                   pMsg->msg.handleValueNoti.len);
        // Subtract the number of data bytes contained in the packet from
        // the total, and increase the data buffer write index by that amount.
        dataLen -= pMsg->msg.handleValueNoti.len;
        currentDataBufWriteIndex += pMsg->msg.handleValueNoti.len;
  
        // Checks if this is the last the continued packet.
        if (dataLen == 0x0000)
        {
          // If so reset the write index and the state.
          currentDataBufWriteIndex = 0;
          notifAttrPktProcessState = NOTI_ATTR_FIRST_PKT;
        }
      }
    }
    break;

    default:
    break;
  }

  // Now we have real data, to display it on LCD for demo now,
  // customer needs to change it from here to deal with data
  if (dataLen == 0)
  {
    pNotifQueueFront->notifData.currentState++;

    if (AttrID == NOTIFICATION_ATTRIBUTE_ID_APP_IDENTIFIER)
      Display_print1(dispHandle, 7, 0, "* AppID:\t%s", (char* )dataBuf);
    else if (AttrID == NOTIFICATION_ATTRIBUTE_ID_TITLE)
      Display_print1(dispHandle, 8, 0, "* Title:\t%s", (char* )dataBuf);

    else if (AttrID == NOTIFICATION_ATTRIBUTE_ID_MESSAGE)
      Display_print1(dispHandle, 9, 0, "* Message:\t%s", (char* )dataBuf);

    else if (AttrID == NOTIFICATION_ATTRIBUTE_ID_DATE)
      Ancs_printNotifDate(dataBuf);
  }
  // Check if the dataLen variable was overflowed if packets got mismatched.
  // This may occur if rapid connecting and disconnecting to the device is performed.
  else if(dataLen > DATA_BUFFER_SIZE - 1)
  {
    Display_print0(dispHandle, 7,  0, "* AppID:\tDATA CURRUPTED");
    Display_print0(dispHandle, 7,  0, "* App Name:\tDATA CURRUPTED");
    Display_print0(dispHandle, 8,  0, "* Title:\tDATA CURRUPTED");
    Display_print0(dispHandle, 9,  0, "* Message:\tDATA CURRUPTED");
    Display_print0(dispHandle, 10, 0, "* Date:\t\tDATA CURRUPTED");
    Ancs_findAndRemoveFromQueue(currentNotifUID);
    notifAttrPktProcessState = NOTI_ATTR_FIRST_PKT;
  }
  // Continue processing the current notification.
  Ancs_processNotifications();

  return;
}

/*********************************************************************
 * @fn      Ancs_processNotifications
 *
 * @brief   Process the front notification in ANCS notification queue.
 *
 * @param   none
 *
 * @return  none
 */
static void Ancs_processNotifications(void)
{
  // Check if the notification queue is empty. If it is, return.
  if(!Ancs_queueEmpty())
  {
    // If not check if the current notification is in its finished state.
    // If it is, remove it from the queue.
    if( pNotifQueueFront->notifData.currentState == NOTI_ATTR_ID_END )
          Ancs_popNotifFromQueue();

    // Process the notification currently at the front of the queue.
    Ancs_processNotificationServiceNotif(pNotifQueueFront);
  }

  return;
}

/*********************************************************************
 * @fn      Ancs_processNotificationServiceNotif
 *
 * @brief   Process ANCS Notification Service notifications
 *
 * @param   pMsg - GATT message.
 *
 * @return  none
 */
static void Ancs_processNotificationServiceNotif(notifQueueNode_t *pNotif)
{

  // If the notification is in its initial state, display its category.
  if ( pNotifQueueFront->notifData.currentState == NOTI_ATTR_ID_BEGIN )
  {
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "New Notification");
#endif
    switch (pNotif->notifData.categoryID)
    {
    case CATEGORY_ID_OTHER:
      Display_print0(dispHandle, 6, 0, "* Category:\tOther");
      break;

    case CATEGORY_ID_INCOMING_CALL:
      Display_print0(dispHandle, 6, 0, "* Category:\tIncomingCall");
      haveIncomingCall = TRUE;
      memcpy(incomingCallUID, pNotif->notifData.notificationUID, ANCS_NOTIF_UID_LENGTH);
      break;

    case CATEGORY_ID_MISSED_CALL:
      Display_print0(dispHandle, 6, 0, "* Category:\tMissedCall");
      haveIncomingCall = FALSE;
      break;

    case CATEGORY_ID_VOICEMAIL:
      Display_print0(dispHandle, 6, 0, "* Category:\tVoicemail");
      break;

    case CATEGORY_ID_SOCIAL:
      Display_print0(dispHandle, 6, 0, "* Category:\tSocial");
      break;

    case CATEGORY_ID_SCHEDULE:
      Display_print0(dispHandle, 6, 0, "* Category:\tSchedule");
      break;

    case CATEGORY_ID_EMAIL:
      Display_print0(dispHandle, 6, 0, "* Category:\tEmail");
      break;

    case CATEGORY_ID_NEWS:
      Display_print0(dispHandle, 6, 0, "* Category:\tNews");
      break;

    case CATEGORY_ID_HEALTH_AND_FITNESS:
      Display_print0(dispHandle, 6, 0, "* Category:\tHealth And Fitness");
      break;

    case CATEGORY_ID_BUSINESS_AND_FINANCE:
      Display_print0(dispHandle, 6, 0, "* Category:\tBusiness And Finance");
      break;

    case CATEGORY_ID_LOCATION:
      Display_print0(dispHandle, 6, 0, "* Category:\tLocation");
      break;

    case CATEGORY_ID_ENTERTAINMENT:
      Display_print0(dispHandle, 6, 0, "* Category:\tEntertainment");
      break;

    default:
      break;

    }
  }

  // Move to the attribute retrieval state machine.
  Ancs_handleNotifAttrRsp(pNotif->notifData.notificationUID);

  return;
}

/*********************************************************************
 * @fn      Ancs_performNegativeAction
 *
 * @brief   Performs a negative action on the notification with the passed UID
 *
 * @param   notifUID - A pointer to a four byte array that contains a notification's UID
 *
 * @return  status - Returns the status of the GATT write
 */

static uint8_t Ancs_performNegativeAction(uint8_t* notifUID)
{
  // This will store the return value.
  uint8_t status;

  // Declare the GATT write request.
  attWriteReq_t req;

  // Allocate the memory for the request.
  req.pValue = GATT_bm_alloc(Ancs_connHandle, ATT_WRITE_REQ, PERFORM_NOTIFICATION_ACTION_LENGTH, NULL);

  // If the allocation was not successful, set status to FAILURE. 
  if (req.pValue == NULL)
    status = FAILURE;

  // If not, proceed with the GATT request.
  else
  {
    // Create a pointer to the request's data portion.
    uint8_t *requestPayload = req.pValue;

    // Set the handle to the Control Point's start handle stored in the handle cache.
    req.handle = Ancs_handleCache[ANCS_CTRL_POINT_HDL_START];

    // Set the write length of the GATT write.
    req.len = PERFORM_NOTIFICATION_ACTION_LENGTH;

    // Set the command ID to perform an action on the notification.
    *requestPayload = COMMAND_ID_PERFORM_NOTIFICATION_ACTION; 
    requestPayload++;

    // Copy the ANCS notification UID to the request.
    VOID memcpy(requestPayload, notifUID, ANCS_NOTIF_UID_LENGTH);
    requestPayload += ANCS_NOTIF_UID_LENGTH;

    // Set the action type to negative.
    *requestPayload = ACTION_ID_NEGATIVE;

    // Signature and command must be set to zero.
    req.sig = 0;
    req.cmd = 0;
    
    status = GATT_WriteCharValue(Ancs_connHandle, &req, ICall_getEntityId());
    // If the GATT write is unsuccessful, free the allocated memory and set the status to FAILURE.
    if (status != SUCCESS)
      GATT_bm_free((gattMsg_t *) &req, ATT_WRITE_REQ);
  }

  return status;
}

/*********************************************************************
 * @fn      Ancs_performPositiveAction
 *
 * @brief   Performs a positive action on the notification with the passed UID
 *
 * @param   notifUID - A pointer to a four byte array that contains a notification's UID
 *
 * @return  status - Returns the status of the GATT write
 */

static uint8_t Ancs_performPositiveAction(uint8_t *notifUID)
{
  // This will store the return value.
  uint8_t status;

  // Declare the GATT write request.
  attWriteReq_t req;

  // Allocate the memory for the request.
  req.pValue = GATT_bm_alloc(Ancs_connHandle, ATT_WRITE_REQ, PERFORM_NOTIFICATION_ACTION_LENGTH, NULL);

  // If the allocation was not successful, set status to FAILURE. 
  if (req.pValue == NULL)
    status = FAILURE;

  // If not, proceed with the GATT request.
  else
  {
    // Create a pointer to the request's data portion.
    uint8_t *requestPayload = req.pValue;

    // Set the handle to the Control Point's start handle stored in the handle cache.
    req.handle = Ancs_handleCache[ANCS_CTRL_POINT_HDL_START];

    // Set the write length of the GATT write.
    req.len = PERFORM_NOTIFICATION_ACTION_LENGTH;

    // Set the command ID to perform an action on the notification.
    *requestPayload = COMMAND_ID_PERFORM_NOTIFICATION_ACTION; 
    requestPayload++;

    // Copy the ANCS notification UID to the request.
    VOID memcpy(requestPayload, notifUID, ANCS_NOTIF_UID_LENGTH);
    requestPayload += ANCS_NOTIF_UID_LENGTH;
    
    // Set the action type to positive.
    *requestPayload = ACTION_ID_POSITIVE;

    // Signature and command must be set to zero.
    req.sig = 0;
    req.cmd = 0;
    
    status = GATT_WriteCharValue(Ancs_connHandle, &req, ICall_getEntityId());
    // If the GATT write is unsuccessful, free the allocated memory and set the status to FAILURE.
    if (status != SUCCESS)
      GATT_bm_free((gattMsg_t *) &req, ATT_WRITE_REQ);
  }

  return status;
}

/*********************************************************************
 * @fn      Ancs_printNotifDate
 *
 * @brief   Processes the date data and prints it in a more user friendly format.
 *
 * @param   dataBuf - Pointer to the Data buffer the data is stored in.
 *
 * @return  none
 */
static void Ancs_printNotifDate(uint8_t *dataBuf)
{
  if(dataBuf[12] == '\0')
    return;

  char year[5]   = {'\0'};
  char month[3]  = {'\0'};
  char day[3]    = {'\0'};
  char hour[3]   = {'\0'};
  char minute[3] = {'\0'};
  char second[3] = {'\0'};
  
  memcpy(year,   dataBuf,      4);
  memcpy(month,  dataBuf + 4,  2);
  memcpy(day,    dataBuf + 6,  2);
  memcpy(hour,   dataBuf + 9,  2);
  memcpy(minute, dataBuf + 11, 2);
  memcpy(second, dataBuf + 13, 2);
  
  uint8_t num;
  char time[14] = {'\0'};
  time[2] = ':';
  memcpy(time + 3, minute, 2);
  time[5] = ':';
  memcpy(time + 6, second, 2);
  num = 10 * (hour[0] - '0') + (hour[1] - '0');

  if (num > 12)
  {
   num -= 12;
   memcpy(time + 8, " PM", 3);
  }
  else
   memcpy(time + 8, " AM", 3);
  
  if (num < 10)
  {
    time[0] = '0';
    time[1] = (char) (num + '0');
  }
  else
  {
    time[0] = '1';
    time[1] = (char) ((num % 10) + '0');
  }

  if(memcmp(month,"01", 2) == 0)
    Display_print3(dispHandle, 11, 0, "* Date:\t\tJanuary %s, %s at %s", day, year, time);

  else if(memcmp(month,"02", 2) == 0)
    Display_print3(dispHandle, 11, 0, "* Date:\t\tFebruary %s, %s at %s", day, year, time);

  else if(memcmp(month,"03", 2) == 0)
    Display_print3(dispHandle, 11, 0, "* Date:\t\tMarch %s, %s at %s", day, year, time);

  else if(memcmp(month,"04", 2) == 0)
    Display_print3(dispHandle, 11, 0, "* Date:\t\tApril %s, %s at %s", day, year, time);

  else if(memcmp(month,"05", 2) == 0)
    Display_print3(dispHandle, 11, 0, "* Date:\t\tMay %s, %s at %s", day, year, time);

  else if(memcmp(month,"06", 2) == 0)
    Display_print3(dispHandle, 11, 0, "* Date:\t\tJune %s, %s at %s", day, year, time);

  else if(memcmp(month,"07", 2) == 0)
    Display_print3(dispHandle, 11, 0, "* Date:\t\tJuly %s, %s at %s", day, year, time);

  else if(memcmp(month,"08", 2) == 0)
    Display_print3(dispHandle, 11, 0, "* Date:\t\tAugust %s, %s at %s", day, year, time);

  else if(memcmp(month,"09", 2) == 0)
    Display_print3(dispHandle, 11, 0, "* Date:\t\tSeptember %s, %s at %s", day, year, time);

  else if(memcmp(month,"10", 2) == 0)
    Display_print3(dispHandle, 11, 0, "* Date:\t\tOctober %s, %s at %s", day, year, time);

  else if(memcmp(month,"11", 2) == 0)
    Display_print3(dispHandle, 11, 0, "* Date:\t\tNovember %s, %s at %s", day, year, time);

  else if(memcmp(month,"12", 2) == 0)
    Display_print3(dispHandle, 11, 0, "* Date:\t\tDecember %s, %s at %s", day, year, time);

  else
    Display_print4(dispHandle, 11, 0, "* Date:\t\t%s/%s/%s at %s",  month, day, year, time);

  return;
}

/*********************************************************************
 * @fn      Ancs_queueNewNotif
 *
 * @brief   Extract data from the GATT notification and push it to the queue.
 *
 * @param   pMsg - GATT message.
 *
 * @return  none
 */
void Ancs_queueNewNotif(gattMsgEvent_t *pMsg)
{

  uint8_t len = pMsg->msg.handleValueNoti.len;
  if (len != 8)
  {
    Display_print0(dispHandle, 11, 0, "");
    Display_print0(dispHandle, 11, 0, "Error evt len");
    return;
  }

  // Create pointer to GATT notification data.
  uint8_t *packetData = pMsg->msg.handleValueNoti.pValue;

  // Store the ANCS notification's eventID
  uint8_t eventID = packetData[0];

  // Store the ANCS notification's eventFlag
#ifdef IGNORE_PREEXISTING_NOTIFICATIONS
  uint8_t eventFlag = packetData[1];
#endif

  // Store the ANCS notification's categoryID
  uint8_t categoryID = packetData[2];

  // Notification UID from packetData[4] to packetData[7]
  uint8_t* pNotificationUID = packetData + ANCS_NOTIF_UID_LENGTH;


#ifdef IGNORE_PREEXISTING_NOTIFICATIONS
  if (eventFlag & EVENT_FLAG_PREEXISTING)
    return;
#endif

  if (eventID == EVENT_ID_NOTIFICATION_ADDED)
  {
    // If it is not in the search list, add it.
    Ancs_pushNotifToQueue(categoryID, pNotificationUID);
  }
  else if (eventID == EVENT_ID_NOTIFICATION_REMOVED)
  {
    if(memcmp(pNotificationUID, currentNotifUID, 4))
      Ancs_findAndRemoveFromQueue(pNotificationUID);
  }

  // Move to the attribute retrieval process.
  Ancs_processNotifications();

  return;
}

/*********************************************************************
 * @fn      Ancs_subsDataSrc
 *
 * @brief   Subscribe Data Source
 *
 * @param   none
 *
 * @return  uint8_t SUCCESS/FAILURE
 */
uint8_t Ancs_subsDataSrc(void)
{
  // Empty notification list first
  Ancs_popAllNotifsFromQueue();

  // Call the function to write "01" to the Data Source CCCD. 
  return Ancs_CCCDConfig(Ancs_handleCache[ANCS_DATA_SRC_HDL_CCCD], TRUE);
}

/*********************************************************************
 * @fn      Ancs_subsNotifSrc
 *
 * @brief   Subscribe Notification Source
 *
 * @param   none
 *
 * @return  uint8_t SUCCESS/FAILURE
 */
uint8_t Ancs_subsNotifSrc(void)
{
  // Empty notification list first
  Ancs_popAllNotifsFromQueue();

  // Call the function to write "01" to the Notification Source CCCD. 
  return Ancs_CCCDConfig(Ancs_handleCache[ANCS_NOTIF_SCR_HDL_CCCD], TRUE);
}

/*********************************************************************
 *********************************************************************/
