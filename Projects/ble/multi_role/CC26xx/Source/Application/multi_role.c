/******************************************************************************
 * @file  simple_ble_topology.c
 *
 * @description Application task for the Simple Topology example
 *
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
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
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#include "multi.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
#include "board_lcd.h"
#include "board_key.h"
#include "Board.h"

#include "linkdb.h"
#include "multi_role.h"

#include <ti/drivers/lcd/LCDDogm1286.h>

/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Connection parameters if automatic  parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800
#define DEFAULT_DESIRED_SLAVE_LATENCY         0
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

//connection parameters
#define DEFAULT_CONN_INT                      42
#define DEFAULT_CONN_TIMEOUT                  200
#define DEFAULT_CONN_LATENCY                  0

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// Scan parameters
#define DEFAULT_SCAN_DURATION                 4000
#define DEFAULT_SCAN_WIND                     80
#define DEFAULT_SCAN_INT                      80

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE
   
// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Task configuration
#define SBT_TASK_PRIORITY                     1

#ifndef SBT_TASK_STACK_SIZE
#define SBT_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define SBT_STATE_CHANGE_EVT                  0x0001
#define SBT_CHAR_CHANGE_EVT                   0x0002
#define SBT_CONN_EVT_END_EVT                  0x0004
#define SBT_START_DISCOVERY_EVT               0x0008
#define SBT_KEY_CHANGE_EVT                    0x0010

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

// LCD defines
#define MAIN_MENU 0
#define DEVICE_MENU 1

#define CONNECTED_DEVICES 0
#define DISCOVERED_DEVICES 1

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  uint16_t event;  // event type
  uint8_t status; // event status
  uint8_t *pData; // event data pointer
} sbtEvt_t;

/*********************************************************************
 * LOCAL VARIABLES
 */
// structure to store link attributes 
extern gapRoleInfo_t multiConnInfo[MAX_NUM_BLE_CONNS];

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock object used to signal discovery timeout
static Clock_Struct startDiscClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbmTask;
Char sbmTaskStack[SBT_TASK_STACK_SIZE];

// LCD menu variables
uint8_t LCDmenu = MAIN_MENU;
uint8_t selectKey = DISCOVERED_DEVICES;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  16,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S', 'i', 'm', 'p', 'l', 'e', ' ', 'T', 'o', 'p', 'o', 'l', 'o', 'g', 'y',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Topology";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Connection handle of current connection 
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charHdl[MAX_NUM_BLE_CONNS] = {0};

// Maximim PDU size (default = 27 octets)
static uint16 maxPduSize;  

// Scanning state
static bool scanningStarted = FALSE;

// Number of scan results and scan result index
static uint8_t scanRes;
static uint8_t scanIdx;

// Number of connected devices
static int8_t connIdx = -1;

// connecting state
static uint8_t connecting_state = 0;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Value to write
static uint8_t charVal = 0;

// Value read/write toggle
static bool doWrite = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleTopology_init( void );
static void simpleTopology_taskFxn(UArg a0, UArg a1);
static uint8_t simpleTopology_processStackMsg(ICall_Hdr *pMsg);
static uint8_t simpleTopology_processGATTMsg(gattMsgEvent_t *pMsg);
static void simpleTopology_processAppMsg(sbtEvt_t *pMsg);
static void simpleTopology_processCharValueChangeEvt(uint8_t paramID);
static void simpleTopology_processRoleEvent(gapMultiRoleEvent_t *pEvent);
static void simpleTopology_sendAttRsp(void);
static void simpleTopology_freeAttRsp(uint8_t status);
static void simpleTopology_charValueChangeCB(uint8_t paramID);
static uint8_t simpleTopology_enqueueMsg(uint16_t event, uint8_t status, uint8_t *pData);
static void simpleTopology_startDiscovery(void);
static void simpleTopology_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static bool simpleTopology_findSvcUuid(uint16_t uuid, uint8_t *pData, uint8_t dataLen);
static void simpleTopology_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void simpleTopology_startDiscovery(void);
static void simpleTopology_handleKeys(uint8_t shift, uint8_t keys);
static uint8_t simpleTopology_eventCB(gapMultiRoleEvent_t *pEvent);
static void simpleTopology_sendAttRsp(void);
static void simpleTopology_freeAttRsp(uint8_t status);

void simpleTopology_startDiscHandler(UArg a0);
void simpleTopology_keyChangeHandler(uint8 keysPressed);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleTopology_gapRoleCBs =
{
  simpleTopology_eventCB,        // events to be handled by the app are passed through the GAP Role here
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleTopology_simpleProfileCBs =
{
  simpleTopology_charValueChangeCB // Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      simpleTopology_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleTopology_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbmTaskStack;
  taskParams.stackSize = SBT_TASK_STACK_SIZE;
  taskParams.priority = SBT_TASK_PRIORITY;

  Task_construct(&sbmTask, simpleTopology_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      simpleTopology_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void simpleTopology_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, simpleTopology_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);
  
  //init keys and LCD
  Board_initKeys(simpleTopology_keyChangeHandler);
  Board_openLCD();
  
  // Setup the GAP
  {
    /*-------------------PERIPHERAL-------------------*/
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
    GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);    
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);    
    /*-------------------CENTRAL-------------------*/
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
    GAP_SetParamValue(TGAP_CONN_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, DEFAULT_CONN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, DEFAULT_CONN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_CONN_TIMEOUT);
    GAP_SetParamValue(TGAP_CONN_EST_LATENCY, DEFAULT_CONN_LATENCY);
  }
  
  // Setup the GAP Role Profile
  {
    /*--------PERIPHERAL-------------*/
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;
    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable, NULL);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime, NULL);
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData, NULL);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval, NULL);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval, NULL);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency, NULL);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout, NULL);
    /*--------------CENTRAL-----------------*/
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;
    GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t), 
                                &scanRes, NULL);        
    
    // Register with GAP for HCI/Host messages
    GAP_RegisterForMsgs(selfEntity);
  }
  
  //GATT
  {
    /*---------------------SERVER------------------------*/
    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
  
    // Initialize GATT Server Services
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    DevInfo_AddService();                        // Device Information Service
    SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
    
    // Setup Profile Characteristic Values
    {
      uint8_t charValue1 = 1;
      uint8_t charValue2 = 2;
      uint8_t charValue3 = 3;
      uint8_t charValue4 = 4;
      uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
      
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                                 &charValue1);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                                 &charValue2);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                                 &charValue3);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                                 &charValue4);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                                 charValue5);
    }
    
    // Register callback with SimpleGATTprofile
    SimpleProfile_RegisterAppCBs(&simpleTopology_simpleProfileCBs);
    
    /*-----------------CLIENT------------------*/
    // Initialize GATT Client
    VOID GATT_InitClient();
  
    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);
   
    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(selfEntity);    
    
  }

  // Start the Device
  VOID GAPRole_StartDevice(&simpleTopology_gapRoleCBs);
}

/*********************************************************************
 * @fn      simpleTopology_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Multi.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void simpleTopology_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  simpleTopology_init();

  // Application main loop
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
        uint8 safeToDealloc = TRUE;
        
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Event *pEvt = (ICall_Event *)pMsg;
          
          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBT_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              simpleTopology_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = simpleTopology_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbtEvt_t *pMsg = (sbtEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          simpleTopology_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }

    if (events & SBT_START_DISCOVERY_EVT)
    {      
      events &= ~SBT_START_DISCOVERY_EVT;
      
      simpleTopology_startDiscovery();
    }  
  }
}

/*********************************************************************
 * @fn      simpleTopology_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t simpleTopology_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
    
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = simpleTopology_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;
            
          default:
            break;
        }
      }
      break;
      
    case GAP_MSG_EVENT:
      simpleTopology_processRoleEvent((gapMultiRoleEvent_t *)pMsg);
      break;        
      
    default:
      // do nothing
      break;
  }
  
  return (safeToDealloc);
}

/*********************************************************************
 * @fn      simpleTopology_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t simpleTopology_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBT_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      simpleTopology_freeAttRsp(FAILURE);
      
      // Hold on to the response message for retransmission
      pAttRsp = pMsg;
      
      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.
    
    // Display the opcode of the message that caused the violation.
    LCD_WRITE_STRING_VALUE("FC Violated:", pMsg->msg.flowCtrlEvt.opcode,
                           10, LCD_PAGE6);
  }    
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE6);
  }
  
  //messages from GATT server
  if ((gapRoleNumLinks(GAPROLE_ACTIVE_LINKS) > 0))
  {
    if ((pMsg->method == ATT_READ_RSP)   ||
        ((pMsg->method == ATT_ERROR_RSP) &&
         (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {      
        LCD_WRITE_STRING_VALUE("Read Error", pMsg->msg.errorRsp.errCode, 10,
                               LCD_PAGE6);
      }
      else
      {
        // After a successful read, display the read value
        LCD_WRITE_STRING_VALUE("Read rsp:", pMsg->msg.readRsp.pValue[0], 10,
                               LCD_PAGE6);
      }
      
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      
      if (pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP)
      {     
        LCD_WRITE_STRING_VALUE("Write Error", pMsg->msg.errorRsp.errCode, 10,
                               LCD_PAGE6);
      }
      else
      {
        // After a succesful write, display the value that was written and
        // increment value
        LCD_WRITE_STRING_VALUE("Write sent:", charVal++, 10, LCD_PAGE6);
      }
    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      simpleTopology_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.  
  
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
  
  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      simpleTopology_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void simpleTopology_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;
    
    // Increment retransmission count
    rspTxRetry++;
    
    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);
      
      // We're done with the response message
      simpleTopology_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      LCD_WRITE_STRING_VALUE("Rsp send retry:", rspTxRetry, 10, LCD_PAGE6);
    }
  }
}

/*********************************************************************
 * @fn      simpleTopology_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void simpleTopology_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING_VALUE("Rsp sent, retry:", rspTxRetry, 10, LCD_PAGE6);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
      
      LCD_WRITE_STRING_VALUE("Rsp retry failed:", rspTxRetry, 10, LCD_PAGE6);
    }
    
    // Free response message
    ICall_freeMsg(pAttRsp);
    
    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      simpleTopology_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void simpleTopology_processAppMsg(sbtEvt_t *pMsg)
{
  switch (pMsg->event)
  {
    case SBT_STATE_CHANGE_EVT:
      simpleTopology_processStackMsg((ICall_Hdr *)pMsg->pData);
      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;

    case SBT_CHAR_CHANGE_EVT:
      simpleTopology_processCharValueChangeEvt(pMsg->status);
      break;

    case SBT_KEY_CHANGE_EVT:
      simpleTopology_handleKeys(0, pMsg->status);
      break;
      
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      simpleTopology_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t simpleTopology_eventCB(gapMultiRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (simpleTopology_enqueueMsg(SBT_STATE_CHANGE_EVT, SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }
  
  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      simpleTopology_processRoleEvent
 *
 * @brief   Multi role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleTopology_processRoleEvent(gapMultiRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        maxPduSize = pEvent->initDone.dataPktLen;
        
        LCD_WRITE_STRING("Connected to 0", LCD_PAGE0);
        LCD_WRITE_STRING(Util_convertBdAddr2Str(pEvent->initDone.devAddr),
                         LCD_PAGE1);
        LCD_WRITE_STRING("Initialized", LCD_PAGE2);

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, pEvent->initDone.devAddr);    
      }
      break;

      case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
        {
          if (gapRoleNumLinks(GAPROLE_ACTIVE_LINKS) > 0)
          {
            LCD_WRITE_STRING("Advertising", LCD_PAGE2);
          }
          else
          {
            LCD_WRITE_STRING("Advertising", LCD_PAGE2);
          }
        }
      break;

      case GAP_END_DISCOVERABLE_DONE_EVENT:
        {
          if (gapRoleNumLinks(GAPROLE_AVAILABLE_LINKS) > 0)
          {
            LCD_WRITE_STRING("Ready to Advertise", LCD_PAGE2);
          }
          else
          {
            LCD_WRITE_STRING("Can't Adv : No links", LCD_PAGE2);
          }
        }
      break;      
      
    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        {
          if (simpleTopology_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
                                           pEvent->deviceInfo.pEvtData,
                                           pEvent->deviceInfo.dataLen))
          {
            simpleTopology_addDeviceInfo(pEvent->deviceInfo.addr,
                                           pEvent->deviceInfo.addrType);
          }
        }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        scanningStarted = FALSE;

        // if not filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE)
        {
          // Copy results
          scanRes = pEvent->discCmpl.numDevs;
          memcpy(devList, pEvent->discCmpl.pDevList,
                 (sizeof(gapDevRec_t) * scanRes));
        }
        
        LCD_WRITE_STRING_VALUE("Devices Found", scanRes, 10, LCD_PAGE3);
        
        if (scanRes > 0)
        {
          LCD_WRITE_STRING("<- To Select", LCD_PAGE4);
        }

        // initialize scan index to last device
        scanIdx = scanRes;
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          LCD_WRITE_STRING("Connected!", LCD_PAGE3);
          LCD_WRITE_STRING_VALUE("Connected to ", gapRoleNumLinks(GAPROLE_ACTIVE_LINKS) ,10, LCD_PAGE0);
          
          //update state
          connecting_state = 0;
          //store connection handle
          connHandle = pEvent->linkCmpl.connectionHandle;

          //if we're not advertising, attempt to turn advertising back on
          uint8_t adv;
          GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &adv, NULL);
          if (adv == 1) //connected and advertising
          {
            if (gapRoleNumLinks(GAPROLE_AVAILABLE_LINKS) > 0)
            {
              LCD_WRITE_STRING("Advertising", LCD_PAGE2);
            }
            else //no available links
            {
              LCD_WRITE_STRING("Can't adv: no links", LCD_PAGE2);
            }
          }
          else //not currently advertising
          {
            LCD_WRITE_STRING("Ready to Advertise", LCD_PAGE2);
            //attempt to turn advertising back o
            uint8_t advertEnabled = TRUE;        
            uint8_t stat = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled, NULL);
            if (stat == bleNoResources) //no more links
            {
              LCD_WRITE_STRING("Can't adv: no links", LCD_PAGE2);
            }
            else if (stat == SUCCESS) //turning advertising back on
            {
              LCD_WRITE_STRING("Advertising", LCD_PAGE2);
            }
            else
            {
              while(1);
            }
          }
          
          // Print last connected device
          LCD_WRITE_STRING("", LCD_PAGE4);
          LCD_WRITE_STRING(Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr), LCD_PAGE5 );                         

          // initiate service discovery
          Util_startClock(&startDiscClock);
        }
        else
        {
          connHandle = GAP_CONNHANDLE_INIT;        
          discState = BLE_DISC_STATE_IDLE;
          
          LCD_WRITE_STRING("Connect Failed", LCD_PAGE4);
          LCD_WRITE_STRING_VALUE("Reason:", pEvent->gap.hdr.status, 10, 
                                 LCD_PAGE3);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        connHandle = GAP_CONNHANDLE_INIT;      
        discState = BLE_DISC_STATE_IDLE;
  
        uint8_t i;
        for (i=0; i < MAX_NUM_BLE_CONNS; i++)
        {
          if (multiConnInfo[i].gapRole_ConnectionHandle == GAPROLE_CONN_JUST_TERMINATED)
          {
            //clear screen, reset discovery info, and return to main menu
            multiConnInfo[i].gapRole_ConnectionHandle = INVALID_CONNHANDLE;
            charHdl[i] = 0;
            LCD_WRITE_STRING_VALUE("Connected to ", gapRoleNumLinks(GAPROLE_ACTIVE_LINKS) ,10, LCD_PAGE0);
            LCD_WRITE_STRING("Disconnected!", LCD_PAGE5);
            LCD_WRITE_STRING("Main Menu", LCD_PAGE3);
            LCDmenu = MAIN_MENU;
          }
          if ((gapRoleNumLinks(GAPROLE_ACTIVE_LINKS) == (MAX_NUM_BLE_CONNS-1))) //now we can advertise again
          {
            LCD_WRITE_STRING("Ready to Advertise", LCD_PAGE2);
          }
        }        
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        LCD_WRITE_STRING_VALUE("Param Update:", pEvent->linkUpdate.status,
                                10, LCD_PAGE2);
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      simpleTopology_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void simpleTopology_charValueChangeCB(uint8_t paramID)
{
  simpleTopology_enqueueMsg(SBT_CHAR_CHANGE_EVT, paramID, NULL);
}

/*********************************************************************
 * @fn      simpleTopology_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void simpleTopology_processCharValueChangeEvt(uint8_t paramID)
{
  uint8_t newValue;

  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

      LCD_WRITE_STRING_VALUE("Char 1:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);

      LCD_WRITE_STRING_VALUE("Char 3:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;

    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      simpleTopology_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void simpleTopology_startDiscHandler(UArg a0)
{
  events |= SBT_START_DISCOVERY_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      simpleTopology_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static uint8_t simpleTopology_enqueueMsg(uint16_t event, uint8_t status, uint8_t *pData)
{
  sbtEvt_t *pMsg = ICall_malloc(sizeof(sbtEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->event = event;
    pMsg->status = status;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
  
  return FALSE;
}

/*********************************************************************
 * @fn      simpleTopology_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void simpleTopology_keyChangeHandler(uint8 keys)
{
  simpleTopology_enqueueMsg(SBT_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      simpleTopology_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void simpleTopology_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if (LCDmenu == MAIN_MENU)
  {
    if (keys & KEY_LEFT)  //show discovery results
    {
      selectKey = DISCOVERED_DEVICES;
      
      // If discovery has occurred and a device was found
      if (!scanningStarted && scanRes > 0)
      {
        // Increment index of current result (with wraparound)
        scanIdx++;
        if (scanIdx >= scanRes)
        {
          scanIdx = 0;
        }
        
        LCD_WRITE_STRING_VALUE("Device", (scanIdx + 1), 10, LCD_PAGE3);
        LCD_WRITE_STRING(Util_convertBdAddr2Str(devList[scanIdx].addr), LCD_PAGE4);
      }
      return;
    }
    if (keys & KEY_UP)  //Scan for devices
    {
      // Start or stop discovery
      if (gapRoleNumLinks(GAPROLE_AVAILABLE_LINKS) > 0) //if we can connect to another device
      {
        if (!scanningStarted) //if we're not already scanning
        {
          scanningStarted = TRUE;
          scanRes = 0;
          
          LCD_WRITE_STRING("Discovering...", LCD_PAGE3);
          LCD_WRITE_STRING("", LCD_PAGE4);
          LCD_WRITE_STRING("", LCD_PAGE6);
          
          GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                 DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                 DEFAULT_DISCOVERY_WHITE_LIST);      
        }
        else //cancel scanning
        {
          LCD_WRITE_STRING("Discovery Cancelled", LCD_PAGE3);
          GAPRole_CancelDiscovery();
          scanningStarted = FALSE;
        }
      }
      else // can't add more links at this time
      {
        LCD_WRITE_STRING("Can't scan:no links ", LCD_PAGE3);
      }
      return;
    }
    if (keys & KEY_RIGHT)  // turn advertising on / off
    {
      uint8_t adv;
      uint8_t adv_status;
      GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &adv_status, NULL);
      if (adv_status) //turn off
      {
        adv = FALSE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv, NULL);
      }
      else //turn on
      {
        adv = TRUE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv, NULL);
      }
      return;
    }
    
    if (keys & KEY_SELECT) //connect to a discovered device
    {
      if (selectKey == DISCOVERED_DEVICES)    // connect to a device  
      {
        uint8_t addrType;
        uint8_t *peerAddr;
        
        if (connecting_state == 1) // if already attempting to connect, cancel connection
        {
        	GAPRole_TerminateConnection(0xFFFE);
        	LCD_WRITE_STRING("Connecting stopped.", LCD_PAGE3);
        	connecting_state = 0;
        }
        else //establish new connection
        {
        	// if there is a scan result
        	if (scanRes > 0)
        	{
        		// connect to current device in scan result
        		peerAddr = devList[scanIdx].addr;
        		addrType = devList[scanIdx].addrType;

        		GAPRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                	DEFAULT_LINK_WHITE_LIST,
									addrType, peerAddr);
        		connecting_state = 1;

        		LCD_WRITE_STRING("Connecting", LCD_PAGE3);
        		LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddr), LCD_PAGE4);
        	}
        }
      }
      else if (selectKey == CONNECTED_DEVICES) //enter the device menu
      {
        if (multiConnInfo[connIdx].gapRole_ConnectionHandle != INVALID_CONNHANDLE)
        {
          LCDmenu = DEVICE_MENU;
          LCD_WRITE_STRING("Device Menu", LCD_PAGE3);
          LCD_WRITE_STRING(Util_convertBdAddr2Str(multiConnInfo[connIdx].gapRole_devAddr), LCD_PAGE4);
          if (multiConnInfo[connIdx].gapRole_ConnRole == GAP_PROFILE_CENTRAL)
          {
             LCD_WRITE_STRING("Connected as Central", LCD_PAGE5);
             LCD_WRITE_STRING("", LCD_PAGE6);
             LCD_WRITE_STRING("", LCD_PAGE7);
          }
          else //PERIPHERAL
          {
            LCD_WRITE_STRING("Connected as Periph", LCD_PAGE5);
            LCD_WRITE_STRING("", LCD_PAGE6);
            LCD_WRITE_STRING("", LCD_PAGE7);            
          }
          //use this connection for all functionality
          connHandle = multiConnInfo[connIdx].gapRole_ConnectionHandle;
        }
        else // no active connection here
        LCD_WRITE_STRING("No Connection here.", LCD_PAGE3);
      }
      return;
    }
    
    if (keys & KEY_DOWN) //browse connected devices
    {
      LCD_WRITE_STRING("Connected Device:", LCD_PAGE3);
      if (++connIdx >= MAX_NUM_BLE_CONNS) //increment connIdx
      {
        connIdx = 0;
      }   
      if (multiConnInfo[connIdx].gapRole_ConnectionHandle != INVALID_CONNHANDLE) //if there is a connection at this index
      {
        LCD_WRITE_STRING(Util_convertBdAddr2Str(multiConnInfo[connIdx].gapRole_devAddr), LCD_PAGE4);
      }
      else
      {
        LCD_WRITE_STRING("N/A", LCD_PAGE4);
      }
      selectKey = CONNECTED_DEVICES;
    } 
    return;
  }
  
  else if (LCDmenu == DEVICE_MENU)
  {
    if (keys & KEY_UP) //read/whrite char
    {
      if (charHdl[connIdx] != 0)
      {
        uint8_t status;
        
        // Do a read or write as long as no other read or write is in progress
        if (doWrite)
        {
          // Do a write
          attWriteReq_t req;
          
          req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, 1, NULL);
          if ( req.pValue != NULL )
          {
            req.handle = charHdl[connIdx];
            req.len = 1;
            req.pValue[0] = charVal;
            req.sig = 0;
            req.cmd = 0;
            
            status = GATT_WriteCharValue(connHandle, &req, selfEntity);
            if ( status != SUCCESS )
            {
              GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
            }
          }
        }
        else
        {
          // Do a read
          attReadReq_t req;
          
          req.handle = charHdl[connIdx];
          status = GATT_ReadCharValue(connHandle, &req, selfEntity);
        }
        
        if (status == SUCCESS)
        {
          doWrite = !doWrite;
        }
      }
      return;
    }
    
    if (keys & KEY_RIGHT) //connection update...eventually
    {
      asm("NOP");
      return;
    }
    
    if (keys & KEY_SELECT)
    {
      GAPRole_TerminateConnection(connHandle);
      
      LCD_WRITE_STRING("Disconnecting", LCD_PAGE5);
      LCD_WRITE_STRING("", LCD_PAGE6);
      
      return;
    }

    if (keys & KEY_DOWN) //back to main menu
    {
      LCDmenu = MAIN_MENU;
      LCD_WRITE_STRING("Main Menu", LCD_PAGE3);
      //clear screen
      LCD_WRITE_STRING("", LCD_PAGE4);
      LCD_WRITE_STRING("", LCD_PAGE5);
      LCD_WRITE_STRING("", LCD_PAGE6);
      LCD_WRITE_STRING("", LCD_PAGE7);
      LCD_WRITE_STRING_VALUE("Connected to ", gapRoleNumLinks(GAPROLE_ACTIVE_LINKS) ,10, LCD_PAGE0);
      
      connIdx = 0;
      
      return;
    }
  }
}

/*********************************************************************
 * @fn      simpleTopology_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void simpleTopology_startDiscovery(void)
{
  attExchangeMTUReq_t req;
  
  // Initialize cached handles
  svcStartHdl = svcEndHdl = 0;
    
  discState = BLE_DISC_STATE_MTU;
  
  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;
  
  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      simpleTopology_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void simpleTopology_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{ 
  if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {   
    // MTU size updated
    LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE4);
  }
  else if (discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover simple BLE service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                         HI_UINT16(SIMPLEPROFILE_SERV_UUID) };        
      discState = BLE_DISC_STATE_SVC;

      // Discovery simple BLE service
      VOID GATT_DiscPrimaryServiceByUUID(connHandle, uuid, ATT_BT_UUID_SIZE,
                                         selfEntity);
    }
  }
  else if (discState == BLE_DISC_STATE_SVC)
  {
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }
    
    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) && 
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;
          
        // Discover characteristic
        discState = BLE_DISC_STATE_CHAR;
        
        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        VOID GATT_ReadUsingCharUUID(connHandle, &req, selfEntity);
      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) && 
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
      //find index to store handle
      uint8_t connIndex = gapRoleInfo_Find(connHandle);
      charHdl[connIndex] = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                             pMsg->msg.readByTypeRsp.pDataList[1]);
      
      LCD_WRITE_STRING("Simple Svc Found", LCD_PAGE6);
    }
    
    discState = BLE_DISC_STATE_IDLE;
  }    
}

/*********************************************************************
 * @fn      simpleTopology_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool simpleTopology_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while (pData < pEnd)
  {
    // Get length of next AD item
    adLen = *pData++;
    if (adLen > 0)
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ((adType == GAP_ADTYPE_16BIT_MORE) || 
          (adType == GAP_ADTYPE_16BIT_COMPLETE))
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while (adLen >= 2 && pData < pEnd)
        {
          // Check for match
          if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if (adLen == 1)
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      simpleTopology_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleTopology_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;
  
  // If result count not at max
  if (scanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < scanRes; i++)
    {
      if (memcmp(pAddr, devList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }
    
    // Add addr to scan result list
    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);
    devList[scanRes].addrType = addrType;
    
    // Increment scan result count
    scanRes++;
  }
}

/*********************************************************************
*********************************************************************/
