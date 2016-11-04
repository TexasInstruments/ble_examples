/*
 * Filename: multi.c
 *
 * Description: Profile code for multi GAPRole
 *
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
#include <string.h>
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <driverlib/ioc.h>

#include "gap.h"
#include "gatt.h"
#include "hci_tl.h"
#include "linkdb.h"
#include "util.h"

#include "gattservapp.h"
#include "multi.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/
// Profile Events
#define START_ADVERTISING_EVT         0x0001  // Start Advertising
#define CONN_PARAM_TIMEOUT_EVT        0x0002  // parameter update timeout

#define DEFAULT_ADVERT_OFF_TIME       30000   // 30 seconds

// connection interval
#define DEFAULT_MIN_CONN_INTERVAL     0x0006  // 100 milliseconds
#define DEFAULT_MAX_CONN_INTERVAL     0x0C80  // 4 seconds
#define MIN_CONN_INTERVAL             0x0006  // minimum connection interval
#define MAX_CONN_INTERVAL             0x0C80  // max connection interval

#define MIN_SLAVE_LATENCY             0       // min slave latency
#define MAX_SLAVE_LATENCY             500     // max slave latency

// supervision timeout
#define MIN_TIMEOUT_MULTIPLIER        0x000a  // min supervision timeout
#define MAX_TIMEOUT_MULTIPLIER        0x0c80  // max supervision timeout
#define DEFAULT_TIMEOUT_MULTIPLIER    1000    // 10 seconds

// Task configuration
#define GAPROLE_TASK_PRIORITY         3
#ifndef GAPROLE_TASK_STACK_SIZE
#define GAPROLE_TASK_STACK_SIZE       440
#endif

/*********************************************************************
* TYPEDEFS
*/

// App event structure
typedef struct
{
  uint8_t  event;  // event type
  uint8_t  status; // event status
} gapRoleEvt_t;

// set/get param event structure
typedef struct
{
  uint16_t  connHandle;  
  uint16_t  value;       
} gapRoleInfoParam_t;

/*********************************************************************
* GLOBAL VARIABLES
*/

// Link DB maximum number of connections
uint8 linkDBNumConns;

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/
// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for app messages
static Queue_Struct appMsg;

// Clock object used to signal timeout
static Clock_Struct startAdvClock;
static Clock_Struct updateTimeoutClock;

// Task pending events
static uint16_t events = 0;

// Task setup
Task_Struct gapRoleTask;
Char gapRoleTaskStack[GAPROLE_TASK_STACK_SIZE];

/*********************************************************************
* Profile Parameters - reference GAPROLE_PROFILE_PARAMETERS for
* descriptions
*/
static uint8_t  gapRole_profileRole;
static uint8_t  gapRole_IRK[KEYLEN];
static uint8_t  gapRole_SRK[KEYLEN];
static uint32_t gapRole_signCounter;
static uint8_t  gapRole_bdAddr[B_ADDR_LEN];
static uint8_t  gapRole_AdvEnabled = TRUE;
static uint8_t  gapRole_AdvNonConnEnabled = FALSE;
static uint16_t gapRole_AdvertOffTime = DEFAULT_ADVERT_OFF_TIME;
static uint8_t  gapRole_AdvertDataLen = 3;

static uint8_t  gapRole_AdvertData[B_MAX_ADV_LEN] =
{
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,   // AD Type = Flags
  // Limited Discoverable & BR/EDR not supported
  (GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED),
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

// Connection parameter update parameters.
static gapRole_updateConnParams_t gapRole_updateConnParams =
{
  .connHandle = INVALID_CONNHANDLE,
  .minConnInterval = DEFAULT_MIN_CONN_INTERVAL,
  .maxConnInterval = DEFAULT_MAX_CONN_INTERVAL,
  .slaveLatency = MIN_SLAVE_LATENCY,
  .timeoutMultiplier = DEFAULT_TIMEOUT_MULTIPLIER
};
static uint8_t paramUpdateNoSuccessOption = GAPROLE_NO_ACTION;

static uint8_t  gapRole_ScanRspDataLen = 0;
static uint8_t  gapRole_ScanRspData[B_MAX_ADV_LEN] = {0};
static uint8_t  gapRole_AdvEventType;
static uint8_t  gapRole_AdvDirectType;
static uint8_t  gapRole_AdvDirectAddr[B_ADDR_LEN] = {0};
static uint8_t  gapRole_AdvChanMap;
static uint8_t  gapRole_AdvFilterPolicy;

static uint8_t  gapRoleMaxScanRes = 8;

// Application callbacks
static gapRolesCBs_t *pGapRoles_AppCGs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

/*********************************************************************
* Profile Attributes - Table
*/

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void gapRole_init(void);
static void gapRole_taskFxn(UArg a0, UArg a1);
static uint8_t gapRole_processStackMsg(ICall_Hdr *pMsg);
static uint8_t gapRole_processGAPMsg(gapEventHdr_t *pMsg);
static void gapRole_SetupGAP(uint8_t* numConns);
static void gapRole_setEvent(uint32_t event);
static void gapRole_HandleParamUpdateNoSuccess(void);

/*********************************************************************
* CALLBACKS
*/
void gapRole_clockHandler(UArg a0);

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @brief   Set a GAP Role parameter.
*
* Public function defined in peripheral.h.
*/
bStatus_t GAPRole_SetParameter(uint16_t param, uint8_t len, void *pValue, uint8 connHandle)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
  case GAPROLE_IRK:
    if (len == KEYLEN)
    {
      VOID memcpy(gapRole_IRK, pValue, KEYLEN) ;
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_SRK:
    if (len == KEYLEN)
    {
      VOID memcpy(gapRole_SRK, pValue, KEYLEN) ;
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_SIGNCOUNTER:
    if (len == sizeof (uint32_t))
    {
      gapRole_signCounter = *((uint32_t*)pValue);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_ADVERT_ENABLED:  //connectable advertising
    if (len == sizeof(uint8_t))
    {
      // Non-connectable advertising must be disabled.
      if (gapRole_AdvNonConnEnabled != TRUE)
      {
        uint8_t oldAdvEnabled = gapRole_AdvEnabled;
        gapRole_AdvEnabled = *((uint8_t*)pValue);
        
        //if ending advertising
        if ((oldAdvEnabled) && (gapRole_AdvEnabled == FALSE))
        {
          //cancel advertising
          return GAP_EndDiscoverable(selfEntity);
        }
        //if starting advertising
        else if ((oldAdvEnabled == FALSE) && (gapRole_AdvEnabled))
        {
          // Turn on advertising.
          if (linkDB_NumActive() >= linkDBNumConns) //don't do conn adv if we don't have any avilable links
          {
            gapRole_AdvEnabled = FALSE;
            return bleNoResources; // no more available links
          }
          else
          {
            gapRole_setEvent(START_ADVERTISING_EVT);
          }
        }
      }
      else
      {
        ret = bleIncorrectMode;
      }
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_ADV_NONCONN_ENABLED:
    if (len == sizeof(uint8_t))
    {
      // Connectable advertising must be disabled.
      if (gapRole_AdvEnabled != TRUE)
      {
        uint8_t oldAdvEnabled = gapRole_AdvNonConnEnabled;
        gapRole_AdvNonConnEnabled = *((uint8_t*)pValue);
        
        //if cancelling advertising  
        if ((oldAdvEnabled) && (gapRole_AdvNonConnEnabled == FALSE))
        {
          // end advertising
          VOID GAP_EndDiscoverable(selfEntity);
        }
        //if starting advertising
        else if ((oldAdvEnabled == FALSE) && (gapRole_AdvNonConnEnabled))
        {  
          //turnon advertising
          gapRole_setEvent(START_ADVERTISING_EVT);
        }
      }
      else
      {
        ret = bleIncorrectMode;
      }
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_ADVERT_OFF_TIME:
    if (len == sizeof (uint16_t))
    {
      gapRole_AdvertOffTime = *((uint16_t*)pValue);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_ADVERT_DATA:
    if (len <= B_MAX_ADV_LEN)
    {
      VOID memset(gapRole_AdvertData, 0, B_MAX_ADV_LEN);
      VOID memcpy(gapRole_AdvertData, pValue, len);
      gapRole_AdvertDataLen = len;
      
      // Update the advertising data
      ret = GAP_UpdateAdvertisingData(selfEntity,
                                      TRUE, gapRole_AdvertDataLen, gapRole_AdvertData);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_SCAN_RSP_DATA:
    if (len <= B_MAX_ADV_LEN)
    {
      VOID memset(gapRole_ScanRspData, 0, B_MAX_ADV_LEN);
      VOID memcpy(gapRole_ScanRspData, pValue, len);
      gapRole_ScanRspDataLen = len;
      
      // Update the Response Data
      ret = GAP_UpdateAdvertisingData(selfEntity,
                                      FALSE, gapRole_ScanRspDataLen, gapRole_ScanRspData);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_ADV_EVENT_TYPE:
    if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= GAP_ADTYPE_ADV_LDC_DIRECT_IND))
    {
      gapRole_AdvEventType = *((uint8_t*)pValue);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_ADV_DIRECT_TYPE:
    if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= ADDRMODE_PRIVATE_RESOLVE)) 
    {
      gapRole_AdvDirectType = *((uint8_t*)pValue);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_ADV_DIRECT_ADDR:
    if (len == B_ADDR_LEN)
    {
      VOID memcpy(gapRole_AdvDirectAddr, pValue, B_ADDR_LEN) ;
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_ADV_CHANNEL_MAP:
    if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= 0x07))
    {
      gapRole_AdvChanMap = *((uint8_t*)pValue);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_ADV_FILTER_POLICY:
    if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= GAP_FILTER_POLICY_WHITE))
    {
      gapRole_AdvFilterPolicy = *((uint8_t*)pValue);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case GAPROLE_MAX_SCAN_RES:
    if (len == sizeof (uint8_t))
    {
      gapRoleMaxScanRes = *((uint8_t*)pValue);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;        
    
  default:
    // The param value isn't part of this profile, try the GAP.
    if ((param < TGAP_PARAMID_MAX) && (len == sizeof (uint16_t)))
    {
      ret = GAP_SetParamValue(param, *((uint16_t*)pValue));
    }
    else
    {
      ret = INVALIDPARAMETER;
    }
    break;
  }
  
  return (ret);
}

/*********************************************************************
* @brief   Get a GAP Role parameter.
*
* Public function defined in peripheral.h.
*/
bStatus_t GAPRole_GetParameter(uint16_t param, void *pValue, uint8_t connHandle)
{
  bStatus_t ret = SUCCESS;
  
  switch (param)
  {
  case GAPROLE_PROFILEROLE:
    *((uint8_t*)pValue) = gapRole_profileRole;
    break;
    
  case GAPROLE_IRK:
    VOID memcpy(pValue, gapRole_IRK, KEYLEN) ;
    break;
    
  case GAPROLE_SRK:
    VOID memcpy(pValue, gapRole_SRK, KEYLEN) ;
    break;
    
  case GAPROLE_SIGNCOUNTER:
    *((uint32_t*)pValue) = gapRole_signCounter;
    break;
    
  case GAPROLE_BD_ADDR:
    VOID memcpy(pValue, gapRole_bdAddr, B_ADDR_LEN) ;
    break;
    
  case GAPROLE_ADVERT_ENABLED:
    *((uint8_t*)pValue) = gapRole_AdvEnabled;
    break;
    
  case GAPROLE_ADV_NONCONN_ENABLED:
    *((uint8_t*)pValue) = gapRole_AdvNonConnEnabled;
    break;
    
  case GAPROLE_ADVERT_OFF_TIME:
    *((uint16_t*)pValue) = gapRole_AdvertOffTime;
    break;
    
  case GAPROLE_ADVERT_DATA:
    VOID memcpy(pValue , gapRole_AdvertData, gapRole_AdvertDataLen);
    break;
    
  case GAPROLE_SCAN_RSP_DATA:
    VOID memcpy(pValue, gapRole_ScanRspData, gapRole_ScanRspDataLen) ;
    break;
    
  case GAPROLE_ADV_EVENT_TYPE:
    *((uint8_t*)pValue) = gapRole_AdvEventType;
    break;
    
  case GAPROLE_ADV_DIRECT_TYPE:
    *((uint8_t*)pValue) = gapRole_AdvDirectType;
    break;
    
  case GAPROLE_ADV_DIRECT_ADDR:
    VOID memcpy(pValue, gapRole_AdvDirectAddr, B_ADDR_LEN) ;
    break;
    
  case GAPROLE_ADV_CHANNEL_MAP:
    *((uint8_t*)pValue) = gapRole_AdvChanMap;
    break;
    
  case GAPROLE_ADV_FILTER_POLICY:
    *((uint8_t*)pValue) = gapRole_AdvFilterPolicy;
    break;
    
  case GAPROLE_MAX_SCAN_RES:
    *((uint8_t*)pValue) = gapRoleMaxScanRes;
    break;      
    
  default:
    // The param value isn't part of this profile, try the GAP.
    if (param < TGAP_PARAMID_MAX)
    {
      *((uint16_t*)pValue) = GAP_GetParamValue(param);
    }
    else
    {
      ret = INVALIDPARAMETER;
    }
    break;
  }
  
  return (ret);
}

/*********************************************************************
* @brief   Does the device initialization.
*
* Public function defined in peripheral.h.
*/
bStatus_t GAPRole_StartDevice(gapRolesCBs_t *pAppCallbacks, uint8_t* numConns)
{
  // Clear all of the Application callbacks
  if (pAppCallbacks)
  {
    pGapRoles_AppCGs = pAppCallbacks;
  }
  
  // Start the GAP
  gapRole_SetupGAP(numConns);
  
  return (SUCCESS);
}

/*********************************************************************
* @brief   Terminates the existing connection.
*
* Public function defined in peripheral.h.
*/
bStatus_t GAPRole_TerminateConnection(uint16_t connHandle)
{
  return (GAP_TerminateLinkReq(selfEntity, connHandle, 
                               HCI_DISCONNECT_REMOTE_USER_TERM));
}

/*********************************************************************
* @fn      GAPRole_createTask
*
* @brief   Task creation function for the GAP Peripheral Role.
*
* @param   none
*
* @return  none
*/
void GAPRole_createTask(void)
{
  Task_Params taskParams;
  
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = gapRoleTaskStack;
  taskParams.stackSize = GAPROLE_TASK_STACK_SIZE;
  taskParams.priority = GAPROLE_TASK_PRIORITY;
  
  Task_construct(&gapRoleTask, gapRole_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* LOCAL FUNCTION PROTOTYPES
*/

/*********************************************************************
* @fn      gapRole_init
*
* @brief   Initialization function for the GAP Role Task.
*
* @param   none
*
* @return  none
*/
static void gapRole_init(void)
{ 
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);
  
  // Setup timers as one-shot timers
  Util_constructClock(&startAdvClock, gapRole_clockHandler, 
                      0, 0, false, START_ADVERTISING_EVT);
  Util_constructClock(&updateTimeoutClock, gapRole_clockHandler,
                      0, 0, false, CONN_PARAM_TIMEOUT_EVT);
  
  // Initialize the Profile Advertising and Connection Parameters
  gapRole_profileRole = GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL;
  VOID memset(gapRole_IRK, 0, KEYLEN);
  VOID memset(gapRole_SRK, 0, KEYLEN);
  gapRole_signCounter = 0;
  gapRole_AdvEventType = GAP_ADTYPE_ADV_IND;
  gapRole_AdvDirectType = ADDRTYPE_PUBLIC;
  gapRole_AdvChanMap = GAP_ADVCHAN_ALL;
  gapRole_AdvFilterPolicy = GAP_FILTER_POLICY_ALL;
  
  // Get link DB maximum number of connections
  linkDBNumConns = linkDB_NumConns();
  
  // Restore Items from NV
  VOID osal_snv_read(BLE_NVID_IRK, KEYLEN, gapRole_IRK);
  VOID osal_snv_read(BLE_NVID_CSRK, KEYLEN, gapRole_SRK);
  VOID osal_snv_read(BLE_NVID_SIGNCOUNTER, sizeof(uint32_t), 
                     &gapRole_signCounter);
}

/**
* @brief   Establish a link to a peer device.
*
* Public function defined in central.h.
*/
bStatus_t GAPRole_EstablishLink(uint8_t highDutyCycle, uint8_t whiteList,
                                uint8_t addrTypePeer, uint8_t *peerAddr)
{
  gapEstLinkReq_t params;
  
  params.taskID = ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, 
                                            selfEntity);
  params.highDutyCycle = highDutyCycle;
  params.whiteList = whiteList;
  params.addrTypePeer = addrTypePeer;
  VOID memcpy(params.peerAddr, peerAddr, B_ADDR_LEN);
  
  return GAP_EstablishLinkReq(&params);
}

/**
* @brief   Start a device discovery scan.
*
* Public function defined in central.h.
*/
bStatus_t GAPRole_StartDiscovery(uint8_t mode, uint8_t activeScan, uint8_t whiteList)
{
  gapDevDiscReq_t params;
  
  params.taskID = ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, 
                                            selfEntity);
  params.mode = mode;
  params.activeScan = activeScan;
  params.whiteList = whiteList;
  
  return GAP_DeviceDiscoveryRequest(&params);
}

/**
* @brief   Cancel a device discovery scan.
*
* Public function defined in central.h.
*/
bStatus_t GAPRole_CancelDiscovery(void)
{
  return GAP_DeviceDiscoveryCancel(selfEntity);
}

/*********************************************************************
* @fn      gapRole_taskFxn
*
* @brief   Task entry point for the GAP Peripheral Role.
*
* @param   a0 - first argument
* @param   a1 - second argument
*
* @return  none
*/
static void gapRole_taskFxn(UArg a0, UArg a1)
{  
  // Initialize profile
  gapRole_init();
  
  // Profile main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);
    
    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;
      
      if (ICall_fetchServiceMsg(&src, &dest, 
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8_t safeToDealloc = TRUE;
        
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;
          
          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & GAP_EVENT_SIGN_COUNTER_CHANGED)
            {
              // Sign counter changed, save it to NV
              VOID osal_snv_write(BLE_NVID_SIGNCOUNTER, sizeof(uint32_t), 
                                  &gapRole_signCounter);
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = gapRole_processStackMsg((ICall_Hdr *)pMsg);
          }
        }
        
        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }
    
    if (events & START_ADVERTISING_EVT)
    { 
      events &= ~START_ADVERTISING_EVT;
      
      //if any type of advertising is enabled
      if (gapRole_AdvEnabled || gapRole_AdvNonConnEnabled)
      {
        gapAdvertisingParams_t params;
        
        // Setup advertisement parameters
        if (gapRole_AdvNonConnEnabled)
        {
          // Only advertise non-connectable undirected.
          params.eventType = GAP_ADTYPE_ADV_NONCONN_IND;
        }
        else
        {
          // connectable advertising
          params.eventType = gapRole_AdvEventType;
          params.initiatorAddrType = gapRole_AdvDirectType;
          VOID memcpy(params.initiatorAddr, gapRole_AdvDirectAddr, B_ADDR_LEN);
        }
        
        //set advertising channel map
        params.channelMap = gapRole_AdvChanMap;
        //set advertising filter policy
        params.filterPolicy = gapRole_AdvFilterPolicy;
        
        //start advertising
        GAP_MakeDiscoverable(selfEntity, &params);
      }
    }
    if (events & CONN_PARAM_TIMEOUT_EVT)
    {
      events &= ~CONN_PARAM_TIMEOUT_EVT;
      
      // Unsuccessful in updating connection parameters
      gapRole_HandleParamUpdateNoSuccess();
    }    
  }
}

/*********************************************************************
* @fn      gapRole_processStackMsg
*
* @brief   Process an incoming task message.
*
* @param   pMsg - message to process
*
* @return  none
*/
static uint8_t gapRole_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
  
  switch (pMsg->event)
  {
  case GAP_MSG_EVENT:
    safeToDealloc = gapRole_processGAPMsg((gapEventHdr_t *)pMsg);
    break;
    
  case L2CAP_SIGNAL_EVENT:
    {
      l2capSignalEvent_t *pPkt = (l2capSignalEvent_t *)pMsg;
      
      // Process the Parameter Update Response
      if (pPkt->opcode == L2CAP_PARAM_UPDATE_RSP)
      {
        l2capParamUpdateRsp_t *pRsp = (l2capParamUpdateRsp_t *)&(pPkt->cmd.updateRsp);
        
        if ((pRsp->result == L2CAP_CONN_PARAMS_REJECTED) &&
            (paramUpdateNoSuccessOption == GAPROLE_TERMINATE_LINK))
        {
          // Cancel connection param update timeout timer
          Util_stopClock(&updateTimeoutClock);
          
          // Terminate connection immediately
          GAPRole_TerminateConnection(pPkt->connHandle);
        }
        else
        {
          uint16_t timeout = GAP_GetParamValue(TGAP_CONN_PARAM_TIMEOUT);
          
          // Wait for Controller to update connection parameters if they're
          // accepted. Otherwise, decide what to do based on no success option.
          Util_restartClock(&updateTimeoutClock, timeout);
        }
      }
    }
    break;      
    
  default:
    break;
  }
  
  return (safeToDealloc);
}

/*********************************************************************
* @fn      gapRole_processGAPMsg
*
* @brief   Process an incoming task message.
*
* @param   pMsg - message to process
*
* @return  none
*/
static uint8_t gapRole_processGAPMsg(gapEventHdr_t *pMsg)
{
  uint8_t notify = FALSE;   // State changed so notify the app? (default no)
  
  switch (pMsg->opcode)
  {
  //device initialized
  case GAP_DEVICE_INIT_DONE_EVENT:
    {
      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;
      bStatus_t stat = pPkt->hdr.status;
      
      if (stat == SUCCESS)
      {
        // Save off the generated keys
        VOID osal_snv_write(BLE_NVID_IRK, KEYLEN, gapRole_IRK);
        VOID osal_snv_write(BLE_NVID_CSRK, KEYLEN, gapRole_SRK);
        
        // Save off the information
        VOID memcpy(gapRole_bdAddr, pPkt->devAddr, B_ADDR_LEN);
        
        // Update the advertising data
        stat = GAP_UpdateAdvertisingData(selfEntity,
                                         TRUE, gapRole_AdvertDataLen, gapRole_AdvertData);
      }
      
      notify = TRUE;
    }
    break;
    
  //update advertising done
  case GAP_ADV_DATA_UPDATE_DONE_EVENT:
    {
      gapAdvDataUpdateEvent_t *pPkt = (gapAdvDataUpdateEvent_t *)pMsg;
      
      if (pPkt->hdr.status == SUCCESS)
      {
        // if scan response
        if (pPkt->adType)
        {
          // Setup the Response Data
          pPkt->hdr.status = GAP_UpdateAdvertisingData(selfEntity,
               FALSE, gapRole_ScanRspDataLen, gapRole_ScanRspData);
        }
        // if advertisement data and not currently advertising
        else if (Util_isActive(&startAdvClock) == FALSE)
        {
          // Start advertising
          gapRole_setEvent(START_ADVERTISING_EVT);
        }
      }
      //notify application of failure
      if (pPkt->hdr.status != SUCCESS)
      {
        notify = TRUE;
      }
    }
    break;
  
  //advertising started or ended
  case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
  case GAP_END_DISCOVERABLE_DONE_EVENT:
    {
      gapMakeDiscoverableRspEvent_t *pPkt = (gapMakeDiscoverableRspEvent_t *)pMsg;
      
      if (pPkt->hdr.status == SUCCESS)
      {
        //if advertising started
        if (pMsg->opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
        {
          gapRole_AdvEnabled = TRUE;
        }
        //if advertising ended
        else
        {
          if (gapRole_AdvertOffTime != 0) //restart advertising if param is set
          {
            if ((gapRole_AdvEnabled) || (gapRole_AdvNonConnEnabled))
            {
              Util_restartClock(&startAdvClock, gapRole_AdvertOffTime);
            }
          }
          else
          {
            // Since gapRole_AdvertOffTime is set to 0, the device should not
            // automatically become discoverable again after a period of time.
            // Set enabler to FALSE; device will become discoverable again when
            // this value gets set to TRUE
            if (gapRole_AdvEnabled == TRUE)
            {
              gapRole_AdvEnabled = FALSE;
            }
          }
        }
        notify = TRUE;
      }
      //if we're already advertising
      else if (pPkt->hdr.status == LL_STATUS_ERROR_COMMAND_DISALLOWED)
      {
        notify = FALSE;
      }
    }
    break;
    
  //connection formed
  case GAP_LINK_ESTABLISHED_EVENT:
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;
      uint8_t advertEnable;
      
      //if formed sucessfully
      if (pPkt->hdr.status == SUCCESS)
      {
        // Notify the Bond Manager to the connection
        VOID GAPBondMgr_LinkEst(pPkt->devAddrType, pPkt->devAddr,
                                pPkt->connectionHandle, pPkt->connRole);    
        
        //advertising will stop after connection formed as slave
        if ((pPkt->connRole) == GAP_PROFILE_PERIPHERAL)
        {
          gapRole_AdvEnabled = FALSE;
          //reenable advertising
          advertEnable = TRUE;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                               &advertEnable, NULL);            
        }
      }
      //if not formed sucessfully
      else if (pPkt->hdr.status == bleGAPConnNotAcceptable)
      {
        // Set enabler to FALSE; device will become discoverable again when
        // this value gets set to TRUE
        gapRole_AdvEnabled = FALSE;
      }
      
      notify = TRUE;
    }
    break;
    
  //connection terminated
  case GAP_LINK_TERMINATED_EVENT:
    {
      gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;
      
      linkDBInfo_t pInfo;
      linkDB_GetInfo(pPkt->connectionHandle, &pInfo);
      
      // notify bond manager
      GAPBondMgr_LinkTerm(pPkt->connectionHandle);
      
      notify = TRUE;
    }
    break;
    
  //security request received from slave
  case GAP_SLAVE_REQUESTED_SECURITY_EVENT:
    {
      uint16_t connHandle = ((gapSlaveSecurityReqEvent_t *)pMsg)->connectionHandle;
      uint8_t authReq = ((gapSlaveSecurityReqEvent_t *)pMsg)->authReq;
      
      //start security initialization
      GAPBondMgr_SlaveReqSecurity(connHandle, authReq);
    }
    break;     
    
  //connection parameter update
  case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;
      
      // Cancel connection param update timeout timer (if active)
      Util_stopClock(&updateTimeoutClock);
      
      if (pPkt->hdr.status == SUCCESS)
      {
        notify = TRUE;
      }
    }
    break;
    
  default:
    notify = TRUE;
    break;
  }
  
  if (notify == TRUE) //app needs to take further action
  {
    if (pGapRoles_AppCGs && pGapRoles_AppCGs->pfnPassThrough)
    {
      //call application callback
      return (pGapRoles_AppCGs->pfnPassThrough((gapMultiRoleEvent_t *)pMsg));
    }
  }
  
  return TRUE;
}

/*********************************************************************
* @fn      gapRole_HandleParamUpdateNoSuccess
*
* @brief   Handle unsuccessful connection parameters update.
*
* @param   none
*
* @return  none
*/
static void gapRole_HandleParamUpdateNoSuccess(void)
{  
  // See which option was chosen for unsuccessful updates
  switch (paramUpdateNoSuccessOption)
  {
  //attempt to send the connection parameter update again
  case GAPROLE_RESEND_PARAM_UPDATE:           
    gapRole_connUpdate(GAPROLE_RESEND_PARAM_UPDATE, &gapRole_updateConnParams);
    break;
    
  //drop the connection
  case GAPROLE_TERMINATE_LINK:
    GAPRole_TerminateConnection(gapRole_updateConnParams.connHandle);
    break;
    
  //give up and proceed
  case GAPROLE_NO_ACTION:
    // fall through
  default:
    //do nothing
    break;
  }
}

/********************************************************************
* @fn          gapRole_connUpdate
*
* @brief       Start the connection update procedure
*
* @param       handleFailure - what to do if the update does not occur.
*              Method may choose to terminate connection, try again,
*              or take no action
* @param       pConnParams   - connection parameters to use
*
* @return      SUCCESS: operation was successful.
*              INVALIDPARAMETER: Data can not fit into one packet.
*              MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
*              bleInvalidRange: 
*              bleIncorrectMode: invalid profile role.
*              bleAlreadyInRequestedMode: already updating link parameters.
*              bleNotConnected: Connection is down
*              bleMemAllocError: Memory allocation error occurred.
*              bleNoResources: No available resource
*/
bStatus_t gapRole_connUpdate(uint8_t handleFailure, gapRole_updateConnParams_t *pConnParams)
{
  bStatus_t status;
  linkDBInfo_t pInfo;
  
  //ensure connection exists
  linkDB_GetInfo(pConnParams->connHandle, &pInfo);
  if (!(pInfo.stateFlags & LINK_CONNECTED))
  {
    return (bleNotConnected);
  }
  // Make sure we don't send an L2CAP Connection Parameter Update Request
  // command within TGAP(conn_param_timeout) of an L2CAP Connection Parameter
  // Update Response being received.
  if (Util_isActive(&updateTimeoutClock) == FALSE)
  {     
    uint16_t timeout = GAP_GetParamValue(TGAP_CONN_PARAM_TIMEOUT);
    gapUpdateLinkParamReq_t linkParams;
    
    linkParams.connectionHandle = pConnParams->connHandle;
    linkParams.intervalMin = pConnParams->minConnInterval;
    linkParams.intervalMax = pConnParams->maxConnInterval;
    linkParams.connLatency = pConnParams->slaveLatency;
    linkParams.connTimeout = pConnParams->timeoutMultiplier;
    
    //send parameter update
    status = GAP_UpdateLinkParamReq( &linkParams );
    
    if(status == SUCCESS)
    {
      //store update params for possible resending
      paramUpdateNoSuccessOption = handleFailure;
      VOID memcpy(&gapRole_updateConnParams, pConnParams, sizeof(gapRole_updateConnParams_t));
      
      // start timeout clock
      Util_restartClock(&updateTimeoutClock, timeout);
    }
  }
  else
  {
    return(blePending);
  }
  
  return status;
}

/*********************************************************************
* @fn      gapRole_SetupGAP
*
* @brief   Call the GAP Device Initialization function using the
*          Profile Parameters. Negotiate the maximum number
*          of simultaneous connections with the stack.
*
* @param   numConns - desired number of simultaneous connections
*
* @return  none
*/
static void gapRole_SetupGAP(uint8_t* numConns)
{
 
  //set number of possible simultaneous connections
  *numConns = linkDBNumConns;
  
  VOID GAP_DeviceInit(selfEntity, gapRole_profileRole, gapRoleMaxScanRes, 
                       gapRole_IRK, gapRole_SRK, 
                      (uint32*)&gapRole_signCounter);
}

/*********************************************************************
* @fn      gapRole_setEvent
*
* @brief   Set an event
*
* @param   event - event to be set
*
* @return  none
*/
static void gapRole_setEvent(uint32_t event)
{
  events |= event;
  
  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*********************************************************************
* @fn      gapRole_clockHandler
*
* @brief   Clock handler function
*
* @param   a0 - event
*
* @return  none
*/
void gapRole_clockHandler(UArg a0)
{
  gapRole_setEvent(a0);
}

