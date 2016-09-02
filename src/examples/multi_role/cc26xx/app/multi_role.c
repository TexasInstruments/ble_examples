/******************************************************************************
* @file  simple_ble_topology.c
*
* @description Application task for the Simple Topology example
*
*
 Copyright (c) 2013-2016, Texas Instruments Incorporated
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
#ifdef DEBUG
#include <driverlib/ioc.h>
#endif // DEBUG

#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simple_gatt_profile.h"

#include "multi.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include <ti/mw/display/Display.h>
#include "util.h"
#include "board_key.h"
#include "Board.h"

#include "linkdb.h"
#include "multi_role.h"

#include <ti/mw/lcd/LCDDogm1286.h>

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
#define DEFAULT_CONN_INT                      80
#define DEFAULT_CONN_TIMEOUT                  1000
#define DEFAULT_CONN_LATENCY                  0

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// Scan parameters
#define DEFAULT_SCAN_DURATION                 3000
#define DEFAULT_SCAN_WIND                     80
#define DEFAULT_SCAN_INT                      80

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          FALSE

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
#define MR_TASK_PRIORITY                     1

#ifndef MR_TASK_STACK_SIZE
#define MR_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define MR_STATE_CHANGE_EVT                  0x0001
#define MR_CHAR_CHANGE_EVT                   0x0002
#define MR_CONN_EVT_END_EVT                  0x0004
#define MR_START_DISCOVERY_EVT               0x0008
#define MR_KEY_CHANGE_EVT                    0x0010
#define MR_PAIRING_STATE_EVT                 0x0020
#define MR_PASSCODE_NEEDED_EVT               0x0040

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
  uint8_t *pData; // event data pointer
} mrEvt_t;

// pairing callback event
typedef struct
{
  uint16 connectionHandle; //!< connection Handle
  uint8 state;             //!< state returned from GAPBondMgr
  uint8 status;            //!< status of state
} gapPairStateEvent_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
* LOCAL VARIABLES
*/
// array to store index to connection handle map
uint16_t connHandleMap[MAX_NUM_BLE_CONNS];

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
Task_Struct mrTask;
Char mrTaskStack[MR_TASK_STACK_SIZE];

// LCD menu variables
uint8_t LCDmenu = MAIN_MENU;
uint8_t selectKey = DISCOVERED_DEVICES;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  13,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'M', 'u', 'l', 't', 'i', ' ', 'R', 'o', 'l', 'e', ':', ')',
  
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
static void multi_role_init( void );
static void multi_role_taskFxn(UArg a0, UArg a1);
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg);
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg);
static void multi_role_processAppMsg(mrEvt_t *pMsg);
static void multi_role_processCharValueChangeEvt(uint8_t paramID);
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent);
static void multi_role_sendAttRsp(void);
static void multi_role_freeAttRsp(uint8_t status);
static void multi_role_charValueChangeCB(uint8_t paramID);
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData);
static void multi_role_startDiscovery(void);
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static bool multi_role_findSvcUuid(uint16_t uuid, uint8_t *pData, uint8_t dataLen);
static void multi_role_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void multi_role_startDiscovery(void);
static void multi_role_handleKeys(uint8_t keys);
static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent);
static void multi_role_sendAttRsp(void);
static void multi_role_freeAttRsp(uint8_t status);
static uint16_t multi_role_mapConnHandleToIndex(uint16_t connHandle);
void multi_role_startDiscHandler(UArg a0);
void multi_role_keyChangeHandler(uint8 keysPressed);
static uint8_t multi_role_addMappingEntry(uint16_t connHandle);
static void multi_role_processPasscode(gapPasskeyNeededEvent_t *pData);
static void multi_role_processPairState(gapPairStateEvent_t* pairingEvent);
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status);

/*********************************************************************
* PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t multi_role_gapRoleCBs =
{
  multi_role_eventCB,        // events to be handled by the app are passed through the GAP Role here
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t multi_role_simpleProfileCBs =
{
  multi_role_charValueChangeCB // Characteristic value change callback
};

// GAP Bond Manager Callbacks
static gapBondCBs_t multi_role_BondMgrCBs =
{
  (pfnPasscodeCB_t)multi_role_passcodeCB, // Passcode callback
  multi_role_pairStateCB                  // Pairing state callback
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      multi_role_createTask
*
* @brief   Task creation function for the Simple BLE Peripheral.
*
* @param   None.
*
* @return  None.
*/
void multi_role_createTask(void)
{
  Task_Params taskParams;
  
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = mrTaskStack;
  taskParams.stackSize = MR_TASK_STACK_SIZE;
  taskParams.priority = MR_TASK_PRIORITY;
  
  Task_construct(&mrTask, multi_role_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      multi_role_init
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
static void multi_role_init(void)
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
  Util_constructClock(&startDiscClock, multi_role_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);
  
  //init keys and LCD
  Board_initKeys(multi_role_keyChangeHandler);
  dispHandle = Display_open(Display_Type_LCD, NULL);
  
  // Setup the GAP
  {
    /*-------------------PERIPHERAL-------------------*/
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
    GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);    
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);    
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MAX, advInt);    
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
    SimpleProfile_RegisterAppCBs(&multi_role_simpleProfileCBs);
    
    /*-----------------CLIENT------------------*/
    // Initialize GATT Client
    VOID GATT_InitClient();
    
    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);
    
    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(selfEntity);    
  }
  
  // Setup the GAP Bond Manager
  {
    uint8_t pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = FALSE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }  
  
  // Start the Device
  VOID GAPRole_StartDevice(&multi_role_gapRoleCBs);
  
  // Start Bond Manager
  VOID GAPBondMgr_Register(&multi_role_BondMgrCBs);  
  
  // init index to channel map
  uint8_t i;
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    connHandleMap[i] = INVALID_CONNHANDLE;
  }  
  
#ifdef DEBUG
  // Map RFC_GPO0 to DIO6
  IOCPortConfigureSet(IOID_6, IOC_PORT_RFC_GPO0,
                      IOC_IOMODE_NORMAL);
  // Map RFC_GPO1 to DIO7
  IOCPortConfigureSet(IOID_7, IOC_PORT_RFC_GPO1,
                      IOC_IOMODE_NORMAL);  
#endif //DEBUG  
}

/*********************************************************************
* @fn      multi_role_taskFxn
*
* @brief   Application task entry point for the Simple BLE Multi.
*
* @param   a0, a1 - not used.
*
* @return  None.
*/
static void multi_role_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  multi_role_init();
  
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
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;
          
          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & MR_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              multi_role_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = multi_role_processStackMsg((ICall_Hdr *)pMsg);
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
        mrEvt_t *pMsg = (mrEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          multi_role_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }
    
    if (events & MR_START_DISCOVERY_EVT)
    {      
      events &= ~MR_START_DISCOVERY_EVT;
      
      multi_role_startDiscovery();
    }  
  }
}

/*********************************************************************
* @fn      multi_role_processStackMsg
*
* @brief   Process an incoming stack message.
*
* @param   pMsg - message to process
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
  
  switch (pMsg->event)
  {
  case GATT_MSG_EVENT:
    // Process GATT message
    safeToDealloc = multi_role_processGATTMsg((gattMsgEvent_t *)pMsg);
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
    multi_role_processRoleEvent((gapMultiRoleEvent_t *)pMsg);
    break;        
    
  default:
    // do nothing
    break;
  }
  
  return (safeToDealloc);
}

/*********************************************************************
* @fn      multi_role_processGATTMsg
*
* @brief   Process GATT messages and events.
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   MR_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      multi_role_freeAttRsp(FAILURE);
      
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
    Display_print1(dispHandle, LCD_PAGE6, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }    
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_print1(dispHandle, LCD_PAGE6, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }
  
  //messages from GATT server
  if (linkDB_NumActive() > 0)
  {
    if ((pMsg->method == ATT_READ_RSP)   ||
        ((pMsg->method == ATT_ERROR_RSP) &&
         (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {      
        Display_print1(dispHandle, LCD_PAGE6, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Display_print1(dispHandle, LCD_PAGE6, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
      }
      
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      
      if (pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP)
      {     
        Display_print1(dispHandle, LCD_PAGE6, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a succesful write, display the value that was written and
        // increment value
        Display_print1(dispHandle, LCD_PAGE6, 0, "Write sent: %d", charVal++);
      }
    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      multi_role_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.  
  
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
  
  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
* @fn      multi_role_sendAttRsp
*
* @brief   Send a pending ATT response message.
*
* @param   none
*
* @return  none
*/
static void multi_role_sendAttRsp(void)
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
      multi_role_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, LCD_PAGE6, 0, "Rsp send retry:", rspTxRetry);
    }
  }
}

/*********************************************************************
* @fn      multi_role_freeAttRsp
*
* @brief   Free ATT response message.
*
* @param   status - response transmit status
*
* @return  none
*/
static void multi_role_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, LCD_PAGE6, 0, "Rsp sent, retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
      
      Display_print1(dispHandle, LCD_PAGE6, 0, "Rsp retry failed: %d", rspTxRetry);
    }
    
    // Free response message
    ICall_freeMsg(pAttRsp);
    
    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
* @fn      multi_role_processAppMsg
*
* @brief   Process an incoming callback from a profile.
*
* @param   pMsg - message to process
*
* @return  None.
*/
static void multi_role_processAppMsg(mrEvt_t *pMsg)
{
  switch (pMsg->event)
  {
  case MR_STATE_CHANGE_EVT:
    multi_role_processStackMsg((ICall_Hdr *)pMsg->pData);
    // Free the stack message
    ICall_freeMsg(pMsg->pData);
    break;
    
  case MR_CHAR_CHANGE_EVT:
    multi_role_processCharValueChangeEvt(*(pMsg->pData));
    // Free the app data
    ICall_free(pMsg->pData);
    break;
    
  case MR_KEY_CHANGE_EVT:
    multi_role_handleKeys(*(pMsg->pData));
    // Free the app data
    ICall_free(pMsg->pData);
    break;
    
  case MR_PAIRING_STATE_EVT:
    multi_role_processPairState((gapPairStateEvent_t*)pMsg->pData);
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  case MR_PASSCODE_NEEDED_EVT:
    multi_role_processPasscode((gapPasskeyNeededEvent_t*)pMsg->pData);
    // Free the app data
    ICall_free(pMsg->pData);
    break;    
    
  default:
    // Do nothing.
    break;
  }
}

/*********************************************************************
* @fn      multi_role_eventCB
*
* @brief   Central event callback function.
*
* @param   pEvent - pointer to event structure
*
* @return  TRUE if safe to deallocate event message, FALSE otherwise.
*/
static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (multi_role_enqueueMsg(MR_STATE_CHANGE_EVT, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }
  
  // Caller should free the event
  return TRUE;
}

/*********************************************************************
* @fn      multi_role_processRoleEvent
*
* @brief   Multi role event processing function.
*
* @param   pEvent - pointer to event structure
*
* @return  none
*/
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
  case GAP_DEVICE_INIT_DONE_EVENT:  
    {
      maxPduSize = pEvent->initDone.dataPktLen;
      
      Display_print0(dispHandle, LCD_PAGE0, 0, "Connected to 0");
      Display_print0(dispHandle, LCD_PAGE1, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
      Display_print0(dispHandle, LCD_PAGE2, 0, "Initialized");
      
      DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, pEvent->initDone.devAddr);    
    }
    break;
    
  case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
    {
      Display_print0(dispHandle, LCD_PAGE2, 0, "Advertising");
    }
    break;
    
  case GAP_END_DISCOVERABLE_DONE_EVENT:
    {
      if (linkDB_NumActive() < MAX_NUM_BLE_CONNS)
      {
        Display_print0(dispHandle, LCD_PAGE2, 0, "Ready to Advertise");
      }
      else
      {
        Display_print0(dispHandle, LCD_PAGE2, 0, "Can't Adv : No links");
      }
    }
    break;      
    
  case GAP_DEVICE_INFO_EVENT:
    {
      // if filtering device discovery results based on service UUID
      if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      {
        if (multi_role_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
                                   pEvent->deviceInfo.pEvtData,
                                   pEvent->deviceInfo.dataLen))
        {
          multi_role_addDeviceInfo(pEvent->deviceInfo.addr,
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
      
      Display_print1(dispHandle, LCD_PAGE3, 0, "Devices Found %d", scanRes);
      
      if (scanRes > 0)
      {
        Display_print0(dispHandle, LCD_PAGE4, 0, "<- To Select");
      }
      
      // initialize scan index to last device
      scanIdx = scanRes;
    }
    break;
    
  case GAP_LINK_ESTABLISHED_EVENT:
    {
      if (pEvent->gap.hdr.status == SUCCESS)
      {
        Display_print0(dispHandle, LCD_PAGE3, 0, "Connected!");
        Display_print1(dispHandle, LCD_PAGE0, 0, "Connected to %d", linkDB_NumActive());
        
        //update state
        connecting_state = 0;
        //store connection handle
        connHandle = pEvent->linkCmpl.connectionHandle;
        //add index-to-connHanlde mapping entry
        multi_role_addMappingEntry(connHandle);
        
        //turn off advertising if no available links
        if (linkDB_NumActive() >= MAX_NUM_BLE_CONNS)
        {
          uint8_t advertEnabled = FALSE;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled, NULL);
          Display_print0(dispHandle, LCD_PAGE2, 0, "Can't adv: no links");
        }
        
        // Print last connected device
        Display_print0(dispHandle, LCD_PAGE4, 0, "");
        Display_print0(dispHandle, LCD_PAGE5, 0, Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));
        
        // initiate service discovery
        Util_startClock(&startDiscClock);
      }
      else
      {
        connHandle = GAP_CONNHANDLE_INIT;        
        discState = BLE_DISC_STATE_IDLE;
        
        Display_print0(dispHandle, LCD_PAGE4, 0, "Connect Failed");
        Display_print1(dispHandle, LCD_PAGE3, 0, "Reason: %d", pEvent->gap.hdr.status);
      }
    }
    break;
    
  case GAP_LINK_TERMINATED_EVENT:
    {
      //find index
      uint8_t index = multi_role_mapConnHandleToIndex(pEvent->linkTerminate.connectionHandle);
      //clear screen, reset discovery info, and return to main menu
      connHandleMap[index] = INVALID_CONNHANDLE;
      charHdl[index] = 0;
      Display_print1(dispHandle, LCD_PAGE0, 0, "Connected to %d", linkDB_NumActive());
      Display_print0(dispHandle, LCD_PAGE5, 0, "Disconnected!");
      LCDmenu = MAIN_MENU;
      if (linkDB_NumActive() == (MAX_NUM_BLE_CONNS-1)) //now we can advertise again
      {
        Display_print0(dispHandle, LCD_PAGE2, 0, "Ready to Advertise");
        Display_print0(dispHandle, LCD_PAGE3, 0, "Ready to Scan");
      }      
    }
    break;
    
  case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      Display_print1(dispHandle, LCD_PAGE6, 0, "Param Update %d", pEvent->linkUpdate.status);
    }
    break;
    
  default:
    break;
  }
}

/*********************************************************************
* @fn      multi_role_charValueChangeCB
*
* @brief   Callback from Simple Profile indicating a characteristic
*          value change.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_charValueChangeCB(uint8_t paramID)
{
  uint8_t *pData;
  
  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {    
    *pData = paramID;  
  
    // Queue the event.
    multi_role_enqueueMsg(MR_CHAR_CHANGE_EVT, pData);
  }
}

/*********************************************************************
* @fn      multi_role_processCharValueChangeEvt
*
* @brief   Process a pending Simple Profile characteristic value change
*          event.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_processCharValueChangeEvt(uint8_t paramID)
{
  uint8_t newValue;
  
  switch(paramID)
  {
  case SIMPLEPROFILE_CHAR1:
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);
    
    Display_print1(dispHandle, LCD_PAGE4, 0, "Char 1: %d", (uint16_t)newValue);
    break;
    
  case SIMPLEPROFILE_CHAR3:
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);
    
    Display_print1(dispHandle, LCD_PAGE4, 0, "Char 3: %d", (uint16_t)newValue);
    break;
    
  default:
    // should not reach here!
    break;
  }
}

/*********************************************************************
* @fn      multi_role_startDiscHandler
*
* @brief   Clock handler function
*
* @param   a0 - ignored
*
* @return  none
*/
void multi_role_startDiscHandler(UArg a0)
{
  events |= MR_START_DISCOVERY_EVT;
  
  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*********************************************************************
* @fn      multi_role_enqueueMsg
*
* @brief   Creates a message and puts the message in RTOS queue.
*
* @param   event - message event.
* @param   pData - pointer to data to be queued
*
* @return  None.
*/
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData)
{
  mrEvt_t *pMsg = ICall_malloc(sizeof(mrEvt_t));
  
  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;
    
    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
  
  return FALSE;
}

/*********************************************************************
* @fn      multi_role_keyChangeHandler
*
* @brief   Key event handler function
*
* @param   a0 - ignored
*
* @return  none
*/
void multi_role_keyChangeHandler(uint8 keys)
{
  uint8_t *pData;
  
  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {    
    *pData = keys;  
  
    // Queue the event.
    multi_role_enqueueMsg(MR_KEY_CHANGE_EVT, pData);
  }
}

/*********************************************************************
* @fn      multi_role_handleKeys
*
* @brief   Handles all key events for this device.
*
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
static void multi_role_handleKeys(uint8_t keys)
{
  linkDBInfo_t pInfo;
  
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
        
        Display_print1(dispHandle, LCD_PAGE3, 0, "Device %d", (scanIdx + 1));
        Display_print0(dispHandle, LCD_PAGE4, 0, Util_convertBdAddr2Str(devList[scanIdx].addr));
      }
      return;
    }
    
    if (keys & KEY_UP)  //Scan for devices
    {
      // Start or stop discovery
      if (linkDB_NumActive() < MAX_NUM_BLE_CONNS) //if we can connect to another device
      {
        if (!scanningStarted) //if we're not already scanning
        {
          scanningStarted = TRUE;
          scanRes = 0;
          
          Display_print0(dispHandle, LCD_PAGE3, 0, "Discovering...");
          Display_print0(dispHandle, LCD_PAGE4, 0, "");
          Display_print0(dispHandle, LCD_PAGE6, 0, "");
          Display_print0(dispHandle, LCD_PAGE7, 0, "");
          
          GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                 DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                 DEFAULT_DISCOVERY_WHITE_LIST);      
        }
        else //cancel scanning
        {
          Display_print0(dispHandle, LCD_PAGE3, 0, "Discovery Cancelled");
          GAPRole_CancelDiscovery();
          scanningStarted = FALSE;
        }
      }
      else // can't add more links at this time
      {
        Display_print0(dispHandle, LCD_PAGE3, 0, "Can't scan:no links ");
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
          Display_print0(dispHandle, LCD_PAGE3, 0, "Connecting stopped.");
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
            
            Display_print0(dispHandle, LCD_PAGE3, 0, "Connecting");
            Display_print0(dispHandle, LCD_PAGE4, 0, Util_convertBdAddr2Str(peerAddr));
          }
        }
      }
      else if (selectKey == CONNECTED_DEVICES) //enter the device menu
      {
        if (connHandleMap[connIdx] != INVALID_CONNHANDLE)    
        {
          linkDB_GetInfo(connHandleMap[connIdx], &pInfo);
          LCDmenu = DEVICE_MENU;
          Display_print0(dispHandle, LCD_PAGE3, 0, "Device Menu");
          Display_print0(dispHandle, LCD_PAGE4, 0, Util_convertBdAddr2Str(pInfo.addr));
          if (pInfo.connRole == GAP_PROFILE_CENTRAL)
          {
            Display_print0(dispHandle, LCD_PAGE5, 0, "Connected as Central");
            Display_print0(dispHandle, LCD_PAGE6, 0, "");
            Display_print0(dispHandle, LCD_PAGE7, 0, "");
          }
          else //PERIPHERAL
          {
            Display_print0(dispHandle, LCD_PAGE5, 0, "Connected as Periph");
            Display_print0(dispHandle, LCD_PAGE6, 0, "");
            Display_print0(dispHandle, LCD_PAGE7, 0, "");
          }
          //use this connection for all functionality
          connHandle = connHandleMap[connIdx];
        }
        else // no active connection here
        {
          Display_print0(dispHandle, LCD_PAGE3, 0, "No Connection here.");
        }       
      }
      return;
    }
    
    if (keys & KEY_DOWN) //browse connected devices
    {
      Display_print0(dispHandle, LCD_PAGE3, 0, "Connected Device:");
      if (++connIdx >= MAX_NUM_BLE_CONNS) //increment connIdx
      {
        connIdx = 0;
      }   
      if (connHandleMap[connIdx] != INVALID_CONNHANDLE) //if there is a connection at this index
      {
        linkDB_GetInfo(connHandleMap[connIdx], &pInfo);
        Display_print0(dispHandle, LCD_PAGE4, 0, Util_convertBdAddr2Str(pInfo.addr));
      }
      else
      {
        Display_print0(dispHandle, LCD_PAGE4, 0, "N/A");
      }
      selectKey = CONNECTED_DEVICES;
      return;    
    } 
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
    
    if (keys & KEY_RIGHT) //connection update
    {
      gapRole_updateConnParams_t updateParams =
      {
        .connHandle = connHandle,
        .minConnInterval = 80,
        .maxConnInterval = 150,
        .slaveLatency = 0,
        .timeoutMultiplier = 200
      };
      bStatus_t status = gapRole_connUpdate( GAPROLE_NO_ACTION, &updateParams);
      if (status == SUCCESS)
      {
        Display_print0(dispHandle, LCD_PAGE6, 0, "Updating");
      }
      else if (status == blePending)
      {
        Display_print0(dispHandle, LCD_PAGE6, 0, "Already Updating");
      }
      return;
    }
    
    if (keys & KEY_SELECT)
    {
      GAPRole_TerminateConnection(connHandle);
      
      Display_print0(dispHandle, LCD_PAGE5, 0, "Disconnecting");
      Display_print0(dispHandle, LCD_PAGE6, 0, "");
      Display_print0(dispHandle, LCD_PAGE7, 0, "");
      
      return;
    }
    
    if (keys & KEY_DOWN) //back to main menu
    {
      LCDmenu = MAIN_MENU;
      Display_print0(dispHandle, LCD_PAGE3, 0, "Main Menu");
      //clear screen
      Display_print0(dispHandle, LCD_PAGE4, 0, "");
      Display_print0(dispHandle, LCD_PAGE5, 0, "");
      Display_print0(dispHandle, LCD_PAGE6, 0, "");
      Display_print0(dispHandle, LCD_PAGE7, 0, "");  
      
      connIdx = 0;
      return;
    }
  }
}

/*********************************************************************
* @fn      multi_role_startDiscovery
*
* @brief   Start service discovery.
*
* @return  none
*/
static void multi_role_startDiscovery(void)
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
* @fn      multi_role_processGATTDiscEvent
*
* @brief   Process GATT discovery event
*
* @return  none
*/
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{ 
  if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {   
    // MTU size updated
    Display_print1(dispHandle, LCD_PAGE4, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
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
      uint8_t connIndex = multi_role_mapConnHandleToIndex(connHandle);
      charHdl[connIndex] = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                                        pMsg->msg.readByTypeRsp.pDataList[1]);
      
      Display_print0(dispHandle, LCD_PAGE6, 0, "Simple Svc Found");
    }
    
    discState = BLE_DISC_STATE_IDLE;
  }    
}

/*********************************************************************
* @fn      multi_role_findSvcUuid
*
* @brief   Find a given UUID in an advertiser's service UUID list.
*
* @return  TRUE if service UUID found
*/
static bool multi_role_findSvcUuid(uint16_t uuid, uint8_t *pData,
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
* @fn      multi_role_addDeviceInfo
*
* @brief   Add a device to the device discovery result list
*
* @return  none
*/
static void multi_role_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
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
 * @fn      gapRoleFindIndex
 *
 * @brief   to translate connection handle to index
 *
 * @param   connHandle (connection handle)
 *
 * @return  none
 */
static uint16_t multi_role_mapConnHandleToIndex(uint16_t connHandle)
{
  uint16_t index;
  for (index = 0; index < MAX_NUM_BLE_CONNS; index ++)
  {
    if (connHandleMap[index] == connHandle)
    {
      return index;
    }
  }
  //not found if we got here
  return INVALID_CONNHANDLE;
}

/************************************************************************
 * @fn      multi_role_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  gapPairStateEvent_t *pData;
  
  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPairStateEvent_t))))
  {    
    pData->connectionHandle = connHandle;    
    pData->state = state;
    pData->status = status;
    
    // Enqueue the event.
    multi_role_enqueueMsg(MR_PAIRING_STATE_EVT, (uint8_t *) pData);
  }
}

/*********************************************************************
 * @fn      multi_role_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison)
{
  gapPasskeyNeededEvent_t *pData;
  
  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPasskeyNeededEvent_t))))
  {
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->connectionHandle = connHandle;    
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;
    
    // Enqueue the event.
    multi_role_enqueueMsg(MR_PASSCODE_NEEDED_EVT, (uint8_t *) pData);
  }
}

/*********************************************************************
 * @fn      multi_role_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void multi_role_processPairState(gapPairStateEvent_t* pairingEvent)
{
  if (pairingEvent->state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_print1(dispHandle, LCD_PAGE7, 0,"Cxn %d pairing started", pairingEvent->connectionHandle);
  }
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (pairingEvent->status == SUCCESS)
    {
      Display_print1(dispHandle, LCD_PAGE7, 0,"Cxn %d pairing success", pairingEvent->connectionHandle);
    }
    else
    {
      Display_print2(dispHandle, LCD_PAGE7, 0, "Cxn %d pairing fail: %d", pairingEvent->connectionHandle, pairingEvent->status);
    }
  }
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      Display_print1(dispHandle, LCD_PAGE7, 0, "Cxn %d bonding success", pairingEvent->connectionHandle);
    }
  }
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      Display_print1(dispHandle, LCD_PAGE7, 0, "Cxn %d bond save success", pairingEvent->connectionHandle);
    }
    else
    {
      Display_print2(dispHandle, LCD_PAGE7, 0, "Cxn %d bond save failed: %d", pairingEvent->connectionHandle, pairingEvent->status);
    }
  }
}

/*********************************************************************
 * @fn      multi_role_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void multi_role_processPasscode(gapPasskeyNeededEvent_t *pData)
{
  uint32_t passcode = 123456;
  Display_print1(dispHandle, LCD_PAGE7, 0, "Passcode: %d", passcode);
  GAPBondMgr_PasscodeRsp(pData->connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      gapRoleFindIndex
 *
 * @brief   to translate connection handle to index
 *
 * @param   connHandle (connection handle)
 *
 * @return  none
 */
static uint8_t multi_role_addMappingEntry(uint16_t connHandle)
{
  uint16_t index;
  for (index = 0; index < MAX_NUM_BLE_CONNS; index ++)
  {
    if (connHandleMap[index] == INVALID_CONNHANDLE)
    {
      connHandleMap[index] = connHandle;
      return index;
    }
  }
  //no room if we get here
  return bleNoResources;
}        
        
/*********************************************************************
*********************************************************************/
