/*
 * ancs_client.c
 *
 * Implements ANCS client functionality
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

/*********************************************************************
 * INCLUDES
 */
#include "string.h"
#include "bcomdef.h"
#include "board_lcd.h"
#include "Board.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "ancs.h"

#include <ti/drivers/lcd/LCDDogm1286.h>
/*********************************************************************
 * MACROS
 */

// Maximum attribute parameter length
#define MAX_ATTR_PARAM_LEN            0x14      // 20
#define MAX_NOTIF_ATTR_DATA_LEN_PER_BLE_PKE         (20 - 1 - 4 - 1 - 2)
   

   

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

typedef struct
{
  uint8_t categoryID;           
  uint8_t notificationUID[4];   
} notifCatType_t;

typedef struct notifCatTypeList_t
{
  notifCatType_t notifCategory;           
  struct notifCatTypeList_t *pNext;   
} notifCatTypeList_t;

static notifCatTypeList_t *pNotifCatListStart = NULL;

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint16_t Ancs_connHandle;
uint8_t clientFlags = CLIENT_NONE;
uint8_t configCCCDState = CCCD_CONFIG_NOTIF;
/*********************************************************************
 * EXTERNAL VARIABLES
 */
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Data source state, for getting notification attribute ID values
enum
{
  NOTI_ATTR_ID_BEGIN = 0x00,
  
  NOTI_ATTR_ID_APPID,
  NOTI_ATTR_ID_TITLE,                
  NOTI_ATTR_ID_SUBTITLE,                    // 
  NOTI_ATTR_ID_MESSAGE,                     // Get the message content
  NOTI_ATTR_ID_MESSAGE_SIZE,
  NOTI_ATTR_ID_DATE,
  
  NOTI_ATTR_ID_END                       
};
static uint8_t notiAttrState = NOTI_ATTR_ID_BEGIN;
enum
{
  NOTI_ATTR_FIRST_PKT = 0x00,
  NOTI_ATTR_CONTINUE_PKT
};
static uint8_t nitiAttrPktRcvState = NOTI_ATTR_FIRST_PKT;
static uint8_t dataBuf[100] = {'\0'};
static uint16_t dataLen = 0x0000;
static uint8_t NotiUID[4] = {0x00}; 
static uint8_t lastUID[4] = {0x00}; 
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Ancs_notifCatAdd2List(uint8_t categoryID, uint8_t *pNotifUID);
static notifCatTypeList_t* Ancs_notifCatFindList(uint8_t *pNotifUID);
static void Ancs_notifCatDelListAll(void);
static uint8_t Ancs_CCCDConfig(uint16_t attrHdl, uint8_t isEnable);
static uint8_t Ancs_getNotifAttr(uint8_t *pNotificationUID, 
                                 uint8_t attributeID, uint16_t len);
static void Ancs_handleNotifAttrRsp(uint8_t *pNotificationUID);

/*********************************************************************
 * @fn      Ancs_notifCatAdd2List
 *
 * @brief   Add a Notification & Category to list. 
 *
 * @param   categoryID - category ID of the notification.
 *          pNotifUID  - notification UID.   
 *
 * @return  none
 */
static void Ancs_notifCatAdd2List(uint8_t categoryID, uint8_t *pNotifUID)
{
  notifCatTypeList_t *pNew;
  notifCatTypeList_t *pSearch;
  
  pNew = ICall_malloc(sizeof(notifCatTypeList_t));
  if (pNew == NULL)
    return;
  
  // Store categoryID and notification ID.
  pNew->notifCategory.categoryID = categoryID;
  VOID memcpy(pNew->notifCategory.notificationUID, pNotifUID, 4);
  pNew->pNext = NULL;
  
  if (pNotifCatListStart == NULL) // New list
  {
    pNotifCatListStart = pNew;
  }
  else  // Add to the last position of the list
  {
    pSearch = pNotifCatListStart;
    while (pSearch->pNext != NULL)
    {
      pSearch = pSearch->pNext;
    }
    pSearch->pNext = pNew;
  }
}

/*********************************************************************
 * @fn      Ancs_notifCatFindList
 *
 * @brief   Find a Notification & Category in the existing list. 
 *
 * @param   pNotifUID  - notification UID.   
 *
 * @return  none
 */
static notifCatTypeList_t* Ancs_notifCatFindList(uint8_t *pNotifUID)
{
  notifCatTypeList_t *pSearch;
  uint8_t notifUID[4];
  
  VOID memcpy(notifUID, pNotifUID, 4);
  
   pSearch = pNotifCatListStart;
   while ((memcmp(notifUID, pSearch->notifCategory.notificationUID, 4) != 0) 
          && pSearch != NULL)
   {
     pSearch = pSearch->pNext;
   }
   
   if (pSearch == NULL) // Not in the list
     return NULL;
   else
     return pSearch;
}

/*********************************************************************
 * @fn      Ancs_notifCatDelListAll
 *
 * @brief   Delete the whole list of Notification & Category. 
 *
 * @param   pNotifUID  - notification UID.   
 *
 * @return  none
 */
static void Ancs_notifCatDelListAll(void)
{
   notifCatTypeList_t *pSearch;
   notifCatTypeList_t *p;
   
   pSearch = pNotifCatListStart;

   while(pSearch != NULL)
   {
     p = pSearch;
     pSearch = pSearch->pNext;
     ICall_free(p);
     p = NULL;
   }
   pNotifCatListStart = NULL;
}


/*********************************************************************
 * @fn      Ancs_handleNotification
 *
 * @brief   Handle notifications. 
 *
 * @param   pMsg - GATT message.
 *
 * @return  none
 */
void Ancs_handleNotification(gattMsgEvent_t *pMsg)
{
  static uint8_t importantAlertCnt = 0;
    
  uint8_t i;
  // Look up the handle in the handle cache
  for (i = 0; i < HDL_CACHE_LEN; i++)
  {
    if (pMsg->msg.handleValueNoti.handle == Ancs_handleCache[i])
    {
      break;
    }
  }

  // Perform processing for this handle 
  switch (i)
  {
    case HDL_ANCS_NTF_NOTIF_START:
      // Display more detailed info later
      {

        uint8_t len = pMsg->msg.handleValueNoti.len;
        if (len != 8)
        {
          LCD_WRITE_STRING("Error evt len", LCD_PAGE6);
          return;
        }

        
        uint8_t *p = pMsg->msg.handleValueNoti.pValue;
        uint8_t eventID = p[0];

        uint8_t eventFlag = p[1]; 

        uint8_t categoryID = p[2];

        // notification UID from p[4] to p[7]
        uint8_t* pNotificationUID = p+4;
        //Copy notification to global flag
        if(eventID != EventIDNotificationRemoved)
          VOID memcpy(lastUID, pNotificationUID, 4);
        
        
        //If we added a new urgent notification
        if ((eventFlag & EventFlagImportant) && 
            (eventID == EventIDNotificationAdded))
        {
          clientFlags |= CLIENT_IMPORTANT_ALERT;
          importantAlertCnt++;
        }
        //If we added a new urgent notification
        if ((eventFlag & EventFlagImportant) && 
            (eventID == EventIDNotificationRemoved))
        {
          importantAlertCnt--;
          if(0 == importantAlertCnt)
            clientFlags &= ~CLIENT_IMPORTANT_ALERT;
        }
        

        
        // If not in service disvoery state, then move to Notification Attribute
        if (eventID != EventIDNotificationRemoved
            && (categoryID == CategoryIDIncomingCall
                || categoryID == CategoryIDMissedCall
                || categoryID == CategoryIDSocial)
              )
        {
          if (eventID == EventIDNotificationAdded)
          {
            // If it is not in the search list, add it
            if (Ancs_notifCatFindList(pNotificationUID) ==  NULL)
              Ancs_notifCatAdd2List(categoryID, pNotificationUID);
          }
          notiAttrState = NOTI_ATTR_ID_BEGIN;
          Ancs_handleNotifAttrRsp(pNotificationUID);
        }
      }
      break;

      case HDL_ANCS_DATA_SRC_START:
        {
          // Display incoming call ID or message now for now
          uint8_t *p;
          uint8_t lcdPage; 

          p = pMsg->msg.handleValueNoti.pValue;
          // this is the beginning of a new data packet
          if (nitiAttrPktRcvState == NOTI_ATTR_FIRST_PKT) 
          {

            // Skip Command ID, 1 byte
            if (*p != CommandIDGetNotificationAttributes)
              return;
            p++;        
            
            // Skip Notification UID, 4 bytes
            //uint8_t *pNotiUID = (pMsg->msg.handleValueNoti.pValue) + 1;
            VOID memcpy(NotiUID, p, 4);
            p += 4;     
            
            // Skip Attribute ID, 1 byte
            p++;        
            
            // Skip the data length
            dataLen = BUILD_UINT16(*p, *(p+1));
            p += 2;     
            
            VOID memset(dataBuf, '\0', 100);

            // This packet contains only part of full data packet, 
            //need to wait for more notifications from iOS device.
            if (dataLen > MAX_NOTIF_ATTR_DATA_LEN_PER_BLE_PKE)
            {
              VOID memcpy(dataBuf, p, MAX_NOTIF_ATTR_DATA_LEN_PER_BLE_PKE);
              nitiAttrPktRcvState = NOTI_ATTR_CONTINUE_PKT;
              dataLen -= MAX_NOTIF_ATTR_DATA_LEN_PER_BLE_PKE;
              return;
            }
            else
            {
              VOID memcpy(dataBuf, p, dataLen);
              dataLen = 0x0000;
            }
          }
          // Then all the pValue is data, without header
          else if (nitiAttrPktRcvState == NOTI_ATTR_CONTINUE_PKT)       
          {
            if (dataLen > 0)
            {
      
              // Copy all notification value
              VOID memcpy(dataBuf + MAX_NOTIF_ATTR_DATA_LEN_PER_BLE_PKE, 
                          pMsg->msg.handleValueNoti.pValue, 
                          pMsg->msg.handleValueNoti.len); 
              

              dataLen -= pMsg->msg.handleValueNoti.len;

              if (dataLen ==  0x0000)
              {
                // Reset the state
                nitiAttrPktRcvState = NOTI_ATTR_FIRST_PKT;              
              }
              else
              {
                return;
              }
            }
          }
          // Now we have real data, to display it on LCD for demo now, 
          //customer needs to change it from here to deal with data
          lcdPage = 5;
          if (notiAttrState == NOTI_ATTR_ID_MESSAGE)
            lcdPage = 6;
          else if (notiAttrState == NOTI_ATTR_ID_DATE)
          {
            lcdPage = 7;
          }
          if (notiAttrState == NOTI_ATTR_ID_TITLE
              || notiAttrState == NOTI_ATTR_ID_MESSAGE
              || notiAttrState == NOTI_ATTR_ID_DATE)
          {
            if (dataBuf[0] == '\0')
              LCD_WRITE_STRING(" ", lcdPage);
            else
              LCD_WRITE_STRING((char*)dataBuf, lcdPage);
          }
          // Fetch & handle & parse notification attributes
          Ancs_handleNotifAttrRsp(NotiUID);
        } // end case HDL_ANCS_DATA_SRC_START:
        break;
 
      default:
      break;
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
  uint8_t status = SUCCESS;
      
  attWriteReq_t req;
      
  req.pValue = GATT_bm_alloc(Ancs_connHandle, ATT_WRITE_REQ, 2, NULL);
  if (req.pValue != NULL)
  {
    req.len = 2;
    if (isEnable == TRUE)
    {
      req.pValue[0] = LO_UINT16(GATT_CLIENT_CFG_NOTIFY);
      req.pValue[1] = HI_UINT16(GATT_CLIENT_CFG_NOTIFY);
    }
    else
    {
      req.pValue[0] = 0x00;
      req.pValue[1] = 0x00;
    }
    req.sig = 0;
    req.cmd = 0;
    
    req.handle = attrHdl;
    
    // Send write request
    if (GATT_WriteCharValue(Ancs_connHandle, &req, 
                             ICall_getEntityId()) != SUCCESS)
    {
      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
      status = FAILURE;
    }
    
  }

  return status;
}

/*********************************************************************
 * @fn      Ancs_subsNotifSrc
 *
 * @brief   subscribe Notification Source. 
 *
 * @param   none.
 *
 * @return  uint8_t SUCCESS/FAILURE
 */
uint8_t Ancs_subsNotifSrc(void)
{
  uint8_t status = SUCCESS;
  // Empty notification list first
  Ancs_notifCatDelListAll();
  
  status = Ancs_CCCDConfig(Ancs_handleCache[HDL_ANCS_NTF_CCCD], TRUE);    
  
  return status;
}

/*********************************************************************
 * @fn      Ancs_unSubsNotifSrc
 *
 * @brief   subscribe Notification Source. 
 *
 * @param   none.
 *
 * @return  uint8_t SUCCESS/FAILURE
 */
uint8_t Ancs_unSubsNotifSrc(void)
{
  uint8_t status = SUCCESS;
      
  status = Ancs_CCCDConfig(Ancs_handleCache[HDL_ANCS_NTF_CCCD], FALSE);    
  
  return status;
}


/*********************************************************************
 * @fn      Ancs_subsDataSrc
 *
 * @brief   subscribe Data Source. 
 *
 * @param   none.
 *
 * @return  uint8_t SUCCESS/FAILURE
 */
uint8_t Ancs_subsDataSrc(void)
{
  uint8_t status = SUCCESS;
  
      
  status = Ancs_CCCDConfig(Ancs_handleCache[HDL_ANCS_DATA_SRC_CCCD], TRUE);    
  
  return status;
}

/*********************************************************************
 * @fn      Ancs_unSubsDataSrc
 *
 * @brief   subscribe Data Source. 
 *
 * @param   none.
 *
 * @return  uint8_t SUCCESS/FAILURE
 */
uint8_t Ancs_unSubsDataSrc(void)
{
  uint8_t status = SUCCESS;
      
  status = Ancs_CCCDConfig(Ancs_handleCache[HDL_ANCS_DATA_SRC_CCCD], FALSE);    
  
  return status;
}


/*********************************************************************
 * @fn      Ancs_getNotifAttr
 *
 * @brief   Get notification attributes. 
 *
 * @param   attrID - attributes ID.
 *
 * @return  uint8_t SUCCESS/FAILURE
 */
static uint8_t Ancs_getNotifAttr(uint8_t *pNotificationUID, uint8_t attributeID, uint16_t len)
{
  uint8_t status = SUCCESS;
  uint8_t cmdLen = 8;
  if (len == 0)
    cmdLen = 6;
  
  // Do a write
  attWriteReq_t req;

  req.pValue = GATT_bm_alloc(Ancs_connHandle, ATT_WRITE_REQ, cmdLen, NULL);
  uint8_t *pV = req.pValue;
  if ( req.pValue != NULL )
  {
    req.handle = Ancs_handleCache[HDL_ANCS_CTRL_PT_START];
    req.len = cmdLen;
    *pV = 0x00;                                 // Command ID
    pV++;
    VOID memcpy(pV, pNotificationUID, 4);       // NotificationUID
    pV += 4;
    *pV = attributeID;                          // attributeID
    pV++;
    if (len != 0)
    {
      *pV = LO_UINT16(len);            // Length of data that could be returned
      pV++;
      *pV = HI_UINT16(len);
    }
    
    req.sig = 0;
    req.cmd = 0;

    status = GATT_WriteCharValue(Ancs_connHandle, &req, ICall_getEntityId());
    if ( status != SUCCESS )
    {
      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
  }
   
  return status;
}

/*********************************************************************
 * @fn      Ancs_performPositiveAction
 *
 * @brief   Performs a positive action on the latest notification added to list 
 *
 * @return  
 */

void Ancs_performPositiveAction()
{
  uint8_t status = SUCCESS;
  uint8_t cmdLen = 6;
  
  // Do a write
  attWriteReq_t req;

  req.pValue = GATT_bm_alloc(Ancs_connHandle, ATT_WRITE_REQ, cmdLen, NULL);
  uint8_t *requestPayload = req.pValue;
  if ( req.pValue != NULL )
  {
    req.handle = Ancs_handleCache[HDL_ANCS_CTRL_PT_START];
    req.len = cmdLen;
    *requestPayload = CommandIDPerformNotificationAction;   // Command ID
    requestPayload++;
    VOID memcpy(requestPayload, lastUID, 4);                // NotificationUID
    requestPayload += 4;
    *requestPayload = ActionIDPositive;                     // actionID    
    req.sig = 0;
    req.cmd = 0;

    status = GATT_WriteCharValue(Ancs_connHandle, &req, ICall_getEntityId());
    if ( status != SUCCESS )
    {
      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
  }
   
  return;
}

/*********************************************************************
 * @fn      Ancs_performNegativeAction
 *
 * @brief   Performs a negative action on the latest notification added to list 
 *
 * @return  
 */

void Ancs_performNegativeAction()
{
  uint8_t status = SUCCESS;
  uint8_t cmdLen = 6;
  
  // Do a write
  attWriteReq_t req;

  req.pValue = GATT_bm_alloc(Ancs_connHandle, ATT_WRITE_REQ, cmdLen, NULL);
  uint8_t *requestPayload = req.pValue;
  if ( req.pValue != NULL )
  {
    req.handle = Ancs_handleCache[HDL_ANCS_CTRL_PT_START];
    req.len = cmdLen;
    *requestPayload = CommandIDPerformNotificationAction;   // Command ID
    requestPayload++;
    VOID memcpy(requestPayload, lastUID, 4);                // NotificationUID
    requestPayload += 4;
    *requestPayload = ActionIDNegative;                     // actionID    
    req.sig = 0;
    req.cmd = 0;

    status = GATT_WriteCharValue(Ancs_connHandle, &req, ICall_getEntityId());
    if ( status != SUCCESS )
    {
      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
  }
   
  return;
}

/*********************************************************************
 * @fn      Ancs_handleNotifAttrRsp
 *
 * @brief   Handle response value of Notification Attrbutes from iOS device. 
 *
 * @param   attrID - attributes ID.
 *
 * @return  uint8_t SUCCESS/FAILURE
 */
static void Ancs_handleNotifAttrRsp(uint8_t *pNotificationUID)
{
  notifCatTypeList_t *pNode;
  
  switch (notiAttrState)
  {
    case NOTI_ATTR_ID_BEGIN:
      {
        Ancs_getNotifAttr(pNotificationUID, NotificationAttributeIDAppIdentifier, 0);           // No length per spec
        notiAttrState = NOTI_ATTR_ID_APPID;
      }
      break;
  
    case NOTI_ATTR_ID_APPID:
      {
        // To get NotificationAttributeIDTitle, that is the calling people/number
        // To do this, write to Notification Control Point
        // Missed call info archive
        Ancs_getNotifAttr(pNotificationUID, NotificationAttributeIDTitle, 30);
        notiAttrState = NOTI_ATTR_ID_TITLE;
      }
      break;
      
    case NOTI_ATTR_ID_TITLE:  
      {
        pNode = Ancs_notifCatFindList(pNotificationUID);
        if (pNode->notifCategory.categoryID != CategoryIDSocial) // Not messages
        {
          Ancs_getNotifAttr(pNotificationUID, NotificationAttributeIDDate, 0);            // No length per spec
          notiAttrState = NOTI_ATTR_ID_DATE;
        }
        else
        {
          Ancs_getNotifAttr(pNotificationUID, NotificationAttributeIDSubtitle, 20);
          notiAttrState = NOTI_ATTR_ID_SUBTITLE;
        }
      }
      break;
      
    case NOTI_ATTR_ID_SUBTITLE:
      {
        Ancs_getNotifAttr(pNotificationUID, NotificationAttributeIDMessage, 100);
        notiAttrState = NOTI_ATTR_ID_MESSAGE;
      }
      break;
    
    case NOTI_ATTR_ID_MESSAGE:
      {
        Ancs_getNotifAttr(pNotificationUID, NotificationAttributeIDMessageSize, 0);     // No length per spec
        notiAttrState = NOTI_ATTR_ID_MESSAGE_SIZE;
        //notiAttrState = NOTI_ATTR_ID_END;
      }
      break;
    
    case NOTI_ATTR_ID_MESSAGE_SIZE:
      {
        Ancs_getNotifAttr(pNotificationUID, NotificationAttributeIDDate, 0);            // No length per spec
        notiAttrState = NOTI_ATTR_ID_DATE;
      }
      break;
    
    case NOTI_ATTR_ID_DATE:
      {
        notiAttrState = NOTI_ATTR_ID_END;
      }
      break;
      
    default:
      break;

  }
  
}

/*********************************************************************
*********************************************************************/
