/******************************************************************************

 @file  simple_central.c

 @brief This file contains the Simple BLE Central sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************

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

 ******************************************************************************
 Release Name: ble_sdk_2_02_00_31
 Release Date: 2016-06-16 18:57:29
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"

#include "hci_tl.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simple_gatt_profile.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include "board_key.h"
#include <ti/mw/display/Display.h>
#include "board.h"

#include "simple_central.h"

#include "ble_user_config.h"




/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Simple BLE Central Task Events
#define SBC_START_DISCOVERY_EVT               0x0001
#define SBC_PAIRING_STATE_EVT                 0x0002
#define SBC_PASSCODE_NEEDED_EVT               0x0004
#define SBC_RSSI_READ_EVT                     0x0008
#define SBC_KEY_CHANGE_EVT                    0x0010
#define SBC_STATE_CHANGE_EVT                  0x0020
#define SBC_CONNECTING_TIMEOUT_EVT	      	  0x0040

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Initial minimum connection interval (units of 1.25 ms.)
#define INITIAL_MIN_CONN_INTERVAL 	      	  160

// Initial minimum connection interval (units of 1.25 ms.)
#define INITIAL_MAX_CONN_INTERVAL             320

// Initial slave latency
#define INITIAL_SLAVE_LATENCY 		      	  0

// Initial supervision timeout (units of 1.25 ms)
#define INITIAL_CONN_TIMEOUT          	      700

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      80

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      160

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                   864
#endif

// Application states
typedef enum
{
  BLE_STATE_IDLE,
  BLE_STATE_BROWSING,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCOVERED,
  BLE_STATE_DISCONNECTING
} bleState_t;


// Discovery states
typedef enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
} bleDiscState_t;

// Connection parameter update
enum
{
	INITIAL_PARAMETERS,
	DEFAULT_UPDATE_PARAMETERS
};

// Screen row
enum
{
	ROW_ZERO = 0,
	ROW_ONE = 1,
	ROW_TWO = 2,
	ROW_THREE = 3,
	ROW_FOUR = 4,
	ROW_FIVE = 5,
	ROW_SIX = 6,
	ROW_SEVEN = 7
};

// Menu item state
typedef enum {
	MENU_ITEM_CONN_PARAM_UPDATE,
	MENU_ITEM_RSSI,
	MENU_ITEM_READ_WRITE,
	MENU_ITEM_DISCONNECT
  } MenuItem_t;

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} sbcEvt_t;

// RSSI read data structure
typedef struct
{
  Clock_Struct *pClock; // pointer to clock struct
  uint16_t period;      // how often to read RSSI
  uint16_t connHandle;  // connection handle
} readRssi_t;

/**
 * Type of device discovery (Scan) to perform.
 */
typedef struct
{
	char localName[20];	 		 //!< Device's Name
    uint8_t addrType;            //!< Address Type: @ref ADDRTYPE_DEFINES
    uint8_t addr[B_ADDR_LEN];    //!< Device's Address
    uint8_t nameLength; 	 	 //!< Device name length
} devRecInfo_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;

// Clock object used to timeout connection
static Clock_Struct connectingClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task pending events
static uint16_t events = 0;

// Task configuration
Task_Struct sbcTask;
Char sbcTaskStack[SBC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8_t scanRes;
static uint8_t scanIdx;

// Scan result list
static devRecInfo_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static bool scanningStarted = FALSE;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static bleState_t state = BLE_STATE_IDLE;

// Discovery state
static bleDiscState_t discState = BLE_DISC_STATE_IDLE;

// Connection paramters
static uint8_t currentConnectionParameter = INITIAL_PARAMETERS;

//Connection option state
static MenuItem_t selectedMenuItem = MENU_ITEM_CONN_PARAM_UPDATE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charHdl = 0;

// Value to write
static uint8_t charVal = 0;

// Value read/write toggle
static bool doWrite = FALSE;

// GATT read/write procedure state
static bool procedureInProgress = FALSE;

// Maximum PDU size (default = 27 octets)
static uint16_t maxPduSize;

// Array of RSSI read structures
static readRssi_t readRssi[MAX_NUM_BLE_CONNS];

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleBLECentral_init(void);
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1);

static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg);
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_startDiscovery(void);
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen);
static void SimpleBLECentral_discoverDevices(void);
void SimpleBLECentral_timeoutConnecting(UArg arg0);
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static bool SimpleBLECentral_findLocalName(uint8_t *pEvtData, uint8_t dataLen);
static void SimpleBLECentral_addDeviceName(uint8_t i, uint8_t *pEvtData,
										   uint8_t dataLen);
static void SimpleBLECentral_processPairState(uint8_t pairState, uint8_t status);
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);

static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static bStatus_t SimpleBLECentral_StartRssi(uint16_t connHandle, uint16_t period);
static bStatus_t SimpleBLECentral_CancelRssi(uint16_t connHandle);
static readRssi_t *SimpleBLECentral_RssiAlloc(uint16_t connHandle);
static readRssi_t *SimpleBLECentral_RssiFind(uint16_t connHandle);
static void SimpleBLECentral_RssiFree(uint16_t connHandle);

static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs);
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t pairState,
                                         uint8_t status);

void SimpleBLECentral_startDiscHandler(UArg a0);
void SimpleBLECentral_keyChangeHandler(uint8_t keys);
void SimpleBLECentral_readRssiHandler(UArg a0);

static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t status,
                                           uint8_t *pData);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t SimpleBLECentral_roleCB =
{
  SimpleBLECentral_eventCB    					// Event callback
};

// Bond Manager Callbacks
static gapBondCBs_t SimpleBLECentral_bondCB =
{
  (pfnPasscodeCB_t)SimpleBLECentral_passcodeCB, // Passcode callback
  SimpleBLECentral_pairStateCB                  // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLECentral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbcTaskStack;
  taskParams.stackSize = SBC_TASK_STACK_SIZE;
  taskParams.priority = SBC_TASK_PRIORITY;

  Task_construct(&sbcTask, SimpleBLECentral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_init(void)
{
  uint8_t i;

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, SimpleBLECentral_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);

  // Set initial connection parameter values
  GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, INITIAL_MIN_CONN_INTERVAL);
  GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, INITIAL_MAX_CONN_INTERVAL);
  GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, INITIAL_CONN_TIMEOUT);
  GAP_SetParamValue(TGAP_CONN_EST_LATENCY, INITIAL_SLAVE_LATENCY);

  // Construct clock for connecting timeout
  Util_constructClock(&connectingClock, SimpleBLECentral_timeoutConnecting,
		  	  	  	  DEFAULT_SCAN_DURATION, 0, false, 0);

  Board_initKeys(SimpleBLECentral_keyChangeHandler);

  //In the project predefines, UART is disabled by default. Thus LCD display will be used.
  //Please see the project documentation for instructions on choosing LDC or UART display.
  Display_Params dispParams;
  dispParams.lineClearMode = DISPLAY_CLEAR_BOTH;
  dispHandle = Display_open(Display_Type_LCD, &dispParams);
  if (dispHandle == NULL) dispHandle = Display_open(Display_Type_UART, &dispParams); // Try UART if no LCD found

  // Initialize internal data
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    readRssi[i].connHandle = GAP_CONNHANDLE_ALL;
    readRssi[i].pClock = NULL;
  }

  // Setup Central Profile
  {
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;

    GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8_t),
                                &scanRes);
  }

  // Setup GAP
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void *)attDeviceName);

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = DEFAULT_PASSCODE;
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    uint8_t mitm = DEFAULT_MITM_MODE;
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    uint8_t bonding = DEFAULT_BONDING_MODE;


    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Start the Device
  VOID GAPCentralRole_StartDevice(&SimpleBLECentral_roleCB);

  // Register with bond manager after starting device
  GAPBondMgr_Register(&SimpleBLECentral_bondCB);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  Display_print0(dispHandle, ROW_ZERO, 0, "BLE Central");
}

/*********************************************************************
 * @fn      SimpleBLECentral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLECentral_init();

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
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    // If RTOS queue is not empty, process app message
    while (!Queue_empty(appMsgQueue))
    {
      sbcEvt_t *pMsg = (sbcEvt_t *)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message
        SimpleBLECentral_processAppMsg(pMsg);

        // Free the space from the message
        ICall_free(pMsg);
      }
    }

    if (events & SBC_START_DISCOVERY_EVT)
    {
      events &= ~SBC_START_DISCOVERY_EVT;

      SimpleBLECentral_startDiscovery();
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleBLECentral_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;

    case GATT_MSG_EVENT:
      SimpleBLECentral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            SimpleBLECentral_processCmdCompleteEvt((hciEvt_CmdComplete_t *)pMsg);
            break;

          default:
            break;
        }
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBC_STATE_CHANGE_EVT:
      SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;

    case SBC_KEY_CHANGE_EVT:
      SimpleBLECentral_handleKeys(0, pMsg->hdr.state);
      break;

    case SBC_RSSI_READ_EVT:
      {
        readRssi_t *pRssi = (readRssi_t *)pMsg->pData;

        // If link is up and RSSI reads active
        if (pRssi->connHandle != GAP_CONNHANDLE_ALL &&
            linkDB_Up(pRssi->connHandle))
        {
          // Restart timer
          Util_restartClock(pRssi->pClock, pRssi->period);

          // Read RSSI
          VOID HCI_ReadRssiCmd(pRssi->connHandle);
        }
      }
      break;

    // Pairing event
    case SBC_PAIRING_STATE_EVT:
      {
        SimpleBLECentral_processPairState(pMsg->hdr.state, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    // Passcode event
    case SBC_PASSCODE_NEEDED_EVT:
      {
        SimpleBLECentral_processPasscode(connHandle, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    // Connecting to device timed out
    case SBC_CONNECTING_TIMEOUT_EVT:
      {
    	  GAPCentralRole_TerminateLink(connHandle);
      }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        maxPduSize = pEvent->initDone.dataPktLen;
        Display_print0(dispHandle, ROW_ZERO, 0, "BLE Central");
        Display_print0(dispHandle, ROW_ONE, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
        Display_print0(dispHandle, ROW_TWO, 0, "Initialized");
        Display_print0(dispHandle, ROW_SEVEN, 0, ">RIGHT to scan");
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
    	  //Find peer device address by UUID
		if ( (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE) ||
			SimpleBLECentral_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
                                         pEvent->deviceInfo.pEvtData,
                                         pEvent->deviceInfo.dataLen))
		{
			SimpleBLECentral_addDeviceInfo(pEvent->deviceInfo.addr,
                                           pEvent->deviceInfo.addrType);
		}

	    // Check if the discovered device is already in scan results
		uint8_t i;
	    for (i = 0; i < scanRes; i++)
	    {
	      if (memcmp(pEvent->deviceInfo.addr, devList[i].addr , B_ADDR_LEN) == 0)
	      {
	    	  //Check if pEventData contains a device name
	    	  if (SimpleBLECentral_findLocalName(pEvent->deviceInfo.pEvtData,
	    			  	  	     	 	 	 	 pEvent->deviceInfo.dataLen))
	    	  {
	    		  //Update deviceInfo entry with the name
	    		  SimpleBLECentral_addDeviceName(i, pEvent->deviceInfo.pEvtData,
	    				  	  	  	  	  	  	  	pEvent->deviceInfo.dataLen);
	    	  }
	      }
	    }
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        scanningStarted = FALSE;
        // initialize scan index to first
        scanIdx = 0;
        Display_clearLines(dispHandle, ROW_ONE, ROW_SEVEN);
        Display_print1(dispHandle, ROW_ONE, 0, "Devices found %d", scanRes);
        state = BLE_STATE_DISCOVERED;

        if (scanRes > 0)
        {
        	Display_print0(dispHandle, ROW_SIX, 0, "<LEFT to browse");
        }
        Display_print0(dispHandle, ROW_SEVEN, 0, ">RIGHT to scan");
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
        	//Connect to selected device
          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;
          procedureInProgress = TRUE;

          // If service discovery not performed initiate service discovery
          if (charHdl == 0)
          {
            Util_startClock(&startDiscClock);
          }

          //Find device name in devList struct
          uint8_t i;
          for (i = 0; i < scanRes; i++)
          {
        	  if (memcmp(pEvent->linkCmpl.devAddr, devList[i].addr, B_ADDR_LEN) == NULL)
        	  {
        		  break;
        	  }
          }
          Display_clearLines(dispHandle, ROW_ONE, ROW_SEVEN);
          Display_print1(dispHandle, ROW_ONE, 0, "%s", devList[i].localName);
          Display_print1(dispHandle, ROW_TWO, 0, "%s", Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));
          Display_print0(dispHandle, ROW_THREE, 0, "Connected");
          selectedMenuItem = MENU_ITEM_CONN_PARAM_UPDATE;
          Display_print0(dispHandle, ROW_SEVEN, 0, ">Param upd req");
        }
        else
        {
          state = BLE_STATE_IDLE;
          connHandle = GAP_CONNHANDLE_INIT;
          discState = BLE_DISC_STATE_IDLE;

          Display_clearLine(dispHandle, ROW_FOUR);
          Display_print0(dispHandle, ROW_TWO, 0, "Connect Failed");
          Display_print1(dispHandle, ROW_THREE, 0, "Reason: %d", pEvent->gap.hdr.status);
          Display_print0(dispHandle, ROW_SEVEN, 0, ">RIGHT to scan");
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;
        discState = BLE_DISC_STATE_IDLE;
        charHdl = 0;
        procedureInProgress = FALSE;

        // Cancel RSSI reads
        SimpleBLECentral_CancelRssi(pEvent->linkTerminate.connectionHandle);

        //Clear screen and display disconnect reason
        Display_clearLines(dispHandle, ROW_ONE, ROW_SEVEN);
        Display_print0(dispHandle, ROW_ONE, 0, "Disconnected");
        Display_print1(dispHandle, ROW_TWO, 0, "Reason: %d", pEvent->linkTerminate.reason);
        Display_print0(dispHandle, ROW_SEVEN, 0, ">RIGHT to scan");
        selectedMenuItem = MENU_ITEM_CONN_PARAM_UPDATE;
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
    	  if (state == BLE_STATE_CONNECTED)
    	  {
    		  if (pEvent->linkUpdate.status == SUCCESS)
    		  {
    			  Display_print1(dispHandle, ROW_FOUR, 0, "ParUpd: %d ms", pEvent->linkUpdate.connInterval * 1.25);
    		  }
    		  else
    		  {
    			  Display_print1(dispHandle, ROW_FOUR, 0, "Param error: %d", pEvent->linkUpdate.status);
    		  }
    	  }
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_handleKeys
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
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys)
{
	switch (state)
	{
		case BLE_STATE_IDLE:
			if (keys & KEY_RIGHT)
			{
				// Discover devices
				SimpleBLECentral_discoverDevices();
			}
			//If LEFT is pressed, nothing happens.
			break;

		case BLE_STATE_DISCOVERED:
			if (keys & KEY_LEFT)
			{
				//Display Discovery Results
				if (!scanningStarted && scanRes > 0)
				{
					if (scanIdx >= scanRes)
					{
						Display_clearLines(dispHandle, ROW_TWO, ROW_SEVEN);
						Display_print0(dispHandle, ROW_SIX, 0, "<LEFT to browse");
						Display_print0(dispHandle, ROW_SEVEN, 0, ">RIGHT to scan");

						state = BLE_STATE_BROWSING;
						scanIdx = 0;
					}
					else
					{
						Display_print1(dispHandle, ROW_ONE, 0, "Device %d", (scanIdx + 1));
						Display_print0(dispHandle, ROW_TWO, 0, Util_convertBdAddr2Str(devList[scanIdx].addr));
						Display_print1(dispHandle, ROW_THREE, 0, "%s", devList[scanIdx].localName);
						Display_print0(dispHandle, ROW_SEVEN, 0, ">RIGHT to connect");

						state = BLE_STATE_BROWSING;
						scanIdx++;
					}
				}
				return;
			}
			else if (keys & KEY_RIGHT)
			{
				//Start scanning
				SimpleBLECentral_discoverDevices();
			}
			break;

		case BLE_STATE_BROWSING:
			if (keys & KEY_LEFT)
			{
				//Navigate through discovery results
				if (!scanningStarted && scanRes > 0)
				{
					if (scanIdx >= scanRes)
					{
						//Display the scan option
						Display_clearLines(dispHandle, ROW_ONE, ROW_SEVEN);
						Display_print1(dispHandle, ROW_ONE, 0, "Devices found %d", scanRes);
						Display_print0(dispHandle, ROW_SIX, 0, "<LEFT to browse");
						Display_print0(dispHandle, ROW_SEVEN, 0, ">RIGHT to scan");

						state = BLE_STATE_BROWSING;
						scanIdx = 0;
					}
					else
					{
						//Display next device
						Display_print1(dispHandle, ROW_ONE, 0, "Device %d", (scanIdx + 1));
						Display_print0(dispHandle, ROW_TWO, 0, Util_convertBdAddr2Str(devList[scanIdx].addr));
						Display_print1(dispHandle, ROW_THREE, 0, "%s", devList[scanIdx].localName);
						Display_print0(dispHandle, ROW_SEVEN, 0, ">RIGHT to connect");

						state = BLE_STATE_BROWSING;
						scanIdx++;
					}
				}
			}
			else if (keys & KEY_RIGHT)
			{
				//Scan for devices if the scan option is displayed
				if (scanIdx == 0){
					SimpleBLECentral_discoverDevices();
				}

				//Connect to displayed device
				else
				{
					uint8_t addrType;
					uint8_t *peerAddr;
					if (scanRes > 0 && state == BLE_STATE_BROWSING)
					{
						// connect to current device in scan result
						peerAddr = devList[scanIdx-1].addr;
						addrType = devList[scanIdx-1].addrType;

						state = BLE_STATE_CONNECTING;

						Util_startClock(&connectingClock);

						GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
							                     	 DEFAULT_LINK_WHITE_LIST,
													 addrType, peerAddr);

						Display_clearLines(dispHandle, ROW_FOUR, ROW_SEVEN);
						Display_print0(dispHandle, ROW_TWO, 0, Util_convertBdAddr2Str(peerAddr));
						Display_print0(dispHandle, ROW_FOUR, 0, "Connecting");
					}
				}
			}
			break;

		case BLE_STATE_CONNECTING:
			//Nothing happens if buttons are pressed while the device is connecting.
			break;

		case BLE_STATE_CONNECTED:
			if (keys & KEY_LEFT) //Navigate though menu.
			{
				//Iterate through rows
				switch (selectedMenuItem)
				{
				    case MENU_ITEM_CONN_PARAM_UPDATE:
				    	selectedMenuItem = MENU_ITEM_RSSI;
				    	if (SimpleBLECentral_RssiFind(connHandle) == NULL)
				    	{
				    		Display_print0(dispHandle, ROW_SEVEN, 0, ">Start RSSI poll");
				    	}
				    	else
				    	{
				    		Display_print0(dispHandle, ROW_SEVEN, 0, ">Stop RSSI poll");
				    	}
				    	break;

				    case MENU_ITEM_RSSI:
				    	selectedMenuItem = MENU_ITEM_READ_WRITE;
				    	Display_print0(dispHandle, ROW_SEVEN, 0, ">Read/write req");
				    	break;

				    case MENU_ITEM_READ_WRITE:
				    	selectedMenuItem = MENU_ITEM_DISCONNECT;
				    	Display_print0(dispHandle, ROW_SEVEN, 0, ">Disconnect");
				    	break;

				    case MENU_ITEM_DISCONNECT:
				    	selectedMenuItem = MENU_ITEM_CONN_PARAM_UPDATE;
				    	Display_print0(dispHandle, ROW_SEVEN, 0, ">Param upd req");
				    	break;
				}
			}
			if (keys & KEY_RIGHT)
			{
				switch (selectedMenuItem)
				{
					case MENU_ITEM_CONN_PARAM_UPDATE:
						//Connection Parameter Update
						Display_print0(dispHandle, ROW_FOUR, 0, "Param upd req");
						switch (currentConnectionParameter)
						{
							case INITIAL_PARAMETERS:
								GAPCentralRole_UpdateLink(connHandle,
											  DEFAULT_UPDATE_MIN_CONN_INTERVAL,
											  DEFAULT_UPDATE_MAX_CONN_INTERVAL,
											  DEFAULT_UPDATE_SLAVE_LATENCY,
											  DEFAULT_UPDATE_CONN_TIMEOUT);
								currentConnectionParameter = DEFAULT_UPDATE_PARAMETERS;
								break;
							case DEFAULT_UPDATE_PARAMETERS:
								GAPCentralRole_UpdateLink(connHandle,
											  INITIAL_MIN_CONN_INTERVAL,
											  INITIAL_MAX_CONN_INTERVAL,
											  INITIAL_SLAVE_LATENCY,
											  INITIAL_CONN_TIMEOUT);
								currentConnectionParameter = INITIAL_PARAMETERS;
								break;
						}
						break;

					case MENU_ITEM_RSSI:
						// Start or cancel RSSI polling
						if (SimpleBLECentral_RssiFind(connHandle) == NULL)
						{
							Display_clearLine(dispHandle, ROW_FIVE);
							SimpleBLECentral_StartRssi(connHandle, DEFAULT_RSSI_PERIOD);
							Display_print0(dispHandle, ROW_SEVEN, 0, ">Stop RSSI poll");
						}
						else
						{
							SimpleBLECentral_CancelRssi(connHandle);
							Display_print0(dispHandle, ROW_FIVE, 0, "RSSI Cancelled");
							if (selectedMenuItem == MENU_ITEM_RSSI)
							{
								Display_print0(dispHandle, ROW_SEVEN, 0, ">Start RSSI poll");
							}
						}
						break;

				  	case MENU_ITEM_READ_WRITE:
				  		if (state == BLE_STATE_CONNECTED &&
				  		    charHdl != 0                 &&
						    procedureInProgress == FALSE)
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
				  					Display_print0(dispHandle, ROW_SIX, 0, "Write req sent");
				  					req.handle = charHdl;
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
				  				else
				  				{
				  					status = bleMemAllocError;
				  				}
				  			}
				  			else
				  			{
				  				// Do a read
				  				attReadReq_t req;
				  				req.handle = charHdl;
				  				status = GATT_ReadCharValue(connHandle, &req, selfEntity);
				  				Display_print0(dispHandle, ROW_SIX, 0, "Read req sent");
				  			}

				  			if (status == SUCCESS)
				  			{
				  				procedureInProgress = TRUE;
				  				doWrite = !doWrite;
				  			}
				  		}
				  		break;

				  	case MENU_ITEM_DISCONNECT:
				  		GAPCentralRole_TerminateLink(connHandle);
				  		state = BLE_STATE_DISCONNECTING;
				  		Display_clearLines(dispHandle, ROW_ONE, ROW_SEVEN);
				  		Display_print0(dispHandle, ROW_ONE, 0, "Disconnecting");
				  		break;
				}
			}
	}
	return;
}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg)
{
	if (state == BLE_STATE_CONNECTED)
    {
		// See if GATT server was unable to transmit an ATT response
		if (pMsg->hdr.status == blePending)
		{
			// No HCI buffer was available. App can try to retransmit the response
			// on the next connection event. Drop it for now.
			Display_print1(dispHandle, ROW_SIX, 0, "ATT Rsp drped %d", pMsg->method);
		}
		else if ((pMsg->method == ATT_READ_RSP)   ||
                ((pMsg->method == ATT_ERROR_RSP)  &&
                 (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
		{
			if (pMsg->method == ATT_ERROR_RSP)
			{
				Display_print1(dispHandle, ROW_SIX, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
			}
			else
			{
				// After a successful read, display the read value
				Display_print1(dispHandle, ROW_SIX, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
			}

			procedureInProgress = FALSE;
		}
		else if ((pMsg->method == ATT_WRITE_RSP)  ||
                ((pMsg->method == ATT_ERROR_RSP)  &&
                 (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
		{
			if (pMsg->method == ATT_ERROR_RSP)
			{
				Display_print1(dispHandle, ROW_SIX, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
			}
			else
			{
				// After a successful write, display the value that was written and
				// increment value
				Display_print1(dispHandle, ROW_SIX, 0, "Write sent: %d", charVal++);
			}

			procedureInProgress = FALSE;
		}
		else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
		{
			// ATT request-response or indication-confirmation flow control is
			// violated. All subsequent ATT requests or indications will be dropped.
			// The app is informed in case it wants to drop the connection.

			// Display the opcode of the message that caused the violation.
			Display_print1(dispHandle, ROW_THREE, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
		}
		else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
		{
			// MTU size updated
			Display_print1(dispHandle, ROW_THREE, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
		}
		else if (discState != BLE_DISC_STATE_IDLE)
		{
			SimpleBLECentral_processGATTDiscEvent(pMsg);
		}
    } // else - in case a GATT message came after a connection has dropped, ignore it.

	// Needed only for ATT Protocol messages
	GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
	switch (pMsg->cmdOpcode)
	{
		case HCI_READ_RSSI:
		{
			if (state == BLE_STATE_CONNECTED)
			{
				int8 rssi = (int8)pMsg->pReturnParam[3];
				Display_print1(dispHandle, ROW_FIVE, 0, "RSSI -dB: %d", (uint32_t)(-rssi));
			}
		}
			break;

		default:
			break;
	}
}

/*********************************************************************
 * @fn      SimpleBLECentral_StartRssi
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   period - RSSI read period in ms
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static bStatus_t SimpleBLECentral_StartRssi(uint16_t connHandle, uint16_t period)
{
	readRssi_t *pRssi;

	// Verify link is up
	if (!linkDB_Up(connHandle))
	{
		return bleIncorrectMode;
	}

	// If already allocated
	if ((pRssi = SimpleBLECentral_RssiFind(connHandle)) != NULL)
	{
		// Stop timer
		Util_stopClock(pRssi->pClock);
		pRssi->period = period;
	}

	// Allocate structure
	else if ((pRssi = SimpleBLECentral_RssiAlloc(connHandle)) != NULL)
	{
		pRssi->period = period;
	}
	// Allocate failed
	else
	{
		return bleNoResources;
	}
	// Start timer
	Util_restartClock(pRssi->pClock, period);
	return SUCCESS;
}

/*********************************************************************
 * @fn      SimpleBLECentral_CancelRssi
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static bStatus_t SimpleBLECentral_CancelRssi(uint16_t connHandle)
{
	readRssi_t *pRssi;
	if ((pRssi = SimpleBLECentral_RssiFind(connHandle)) != NULL)
	{
		// Stop timer
		Util_stopClock(pRssi->pClock);

		// Free RSSI structure
		SimpleBLECentral_RssiFree(connHandle);
		return SUCCESS;
	}
	// Not found
	return bleIncorrectMode;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiAlloc
 *
 * @brief   Allocate an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if allocation failed.
 */
static readRssi_t *SimpleBLECentral_RssiAlloc(uint16_t connHandle)
{
	uint8_t i;

	// Find free RSSI structure
	for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
	{
		if (readRssi[i].connHandle == GAP_CONNHANDLE_ALL)
		{
			readRssi_t *pRssi = &readRssi[i];
			pRssi->pClock = (Clock_Struct *)ICall_malloc(sizeof(Clock_Struct));
			if (pRssi->pClock)
			{
				Util_constructClock(pRssi->pClock, SimpleBLECentral_readRssiHandler,
                            		0, 0, false, i);
				pRssi->connHandle = connHandle;
				return pRssi;
			}
		}
	}
	// No free structure found
	return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFind
 *
 * @brief   Find an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if not found.
 */
static readRssi_t *SimpleBLECentral_RssiFind(uint16_t connHandle)
{
	uint8_t i;
	// Find free RSSI structure
	for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
	{
		if (readRssi[i].connHandle == connHandle)
		{
			return &readRssi[i];
		}
	}
	// Not found
	return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFree
 *
 * @brief   Free an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  none
 */
static void SimpleBLECentral_RssiFree(uint16_t connHandle)
{
	uint8_t i;

	// Find RSSI structure
	for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
	{
		if (readRssi[i].connHandle == connHandle)
		{
			readRssi_t *pRssi = &readRssi[i];
			if (pRssi->pClock)
			{
				Clock_destruct(pRssi->pClock);

				// Free clock struct
				ICall_free(pRssi->pClock);
				pRssi->pClock = NULL;
			}
			pRssi->connHandle = GAP_CONNHANDLE_ALL;
			break;
		}
	}
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleBLECentral_processPairState(uint8_t pairState, uint8_t status)
{
	if (pairState == GAPBOND_PAIRING_STATE_STARTED)
	{
		Display_print0(dispHandle, ROW_SIX, 0, "Pairing started");
	}
	else if (pairState == GAPBOND_PAIRING_STATE_COMPLETE)
	{
		if (status == SUCCESS)
		{
			Display_print0(dispHandle, ROW_SIX, 0, "Pairing success");
		}
		else
		{
			Display_print1(dispHandle, ROW_SIX, 0, "Pairing fail: %d", status);
		}
	}
	else if (pairState == GAPBOND_PAIRING_STATE_BONDED)
	{
		if (status == SUCCESS)
		{
			Display_print0(dispHandle, ROW_SIX, 0, "Bonding success");
		}
	}
	else if (pairState == GAPBOND_PAIRING_STATE_BOND_SAVED)
	{
		if (status == SUCCESS)
		{
			Display_print0(dispHandle, ROW_SIX, 0, "Bond save succ");
		}
		else
		{
			Display_print1(dispHandle, ROW_SIX, 0, "Bnd save fail: %d", status);
		}
	}
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs)
{
	uint32_t  passcode;

	// Create random passcode
	passcode = Util_GetTRNG();
	passcode %= 1000000;

	// Display passcode to user
	if (uiOutputs != 0)
    {
		Display_print0(dispHandle, ROW_FOUR, 0, "Passcode:");
		Display_print1(dispHandle, ROW_FIVE, 0, "%d", passcode);
    }
	// Send passcode response
	GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void SimpleBLECentral_startDiscovery(void)
{
	attExchangeMTUReq_t req;

	// Initialize cached handles
	svcStartHdl = svcEndHdl = charHdl = 0;

	discState = BLE_DISC_STATE_MTU;

	// Discover GATT Server's Rx MTU size
	req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

	// ATT MTU size should be set to the minimum of the Client Rx MTU
	// and Server Rx MTU values
	VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
	if (discState == BLE_DISC_STATE_MTU)
	{
		// MTU size response received, discover simple BLE service
		if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
		{
			uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                               HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

			// Just in case we're using the default MTU size (23 octets)
			Display_print1(dispHandle, ROW_THREE, 0, "MTU Size: %d", ATT_MTU_SIZE);
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
			charHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                                   pMsg->msg.readByTypeRsp.pDataList[1]);
			Display_print0(dispHandle, ROW_THREE, 0, "Simple Svc Found");
			procedureInProgress = FALSE;
		}
		discState = BLE_DISC_STATE_IDLE;
	}
}

/*********************************************************************
 * @fn      SimpleBLECentral_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
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
					if ((pData[0] == LO_UINT16(uuid)) &&
						(pData[1] == HI_UINT16(uuid)))
					{
						// Match found
						return TRUE;
					}

					// Go to next AD item
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
				// Go to next AD item
				pData += adLen;
			}
		}
	}
	// Match not found
	return FALSE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_discoverDevices
 *
 * @brief   Scan to discover devices.
 *
 * @return  none
 */
static void SimpleBLECentral_discoverDevices(void){
	if (!scanningStarted)
	{
		scanningStarted = TRUE;

		//Clear old scan results
		scanRes = 0;
		memset(devList, NULL, sizeof(devList[0])*DEFAULT_MAX_SCAN_RES);

		Display_clearLines(dispHandle, ROW_ONE, ROW_SEVEN);
		Display_print0(dispHandle, ROW_ONE, 0, "Discovering...");
		GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
									  DEFAULT_DISCOVERY_ACTIVE_SCAN,
									  DEFAULT_DISCOVERY_WHITE_LIST);
	}
	else
	{
		GAPCentralRole_CancelDiscovery();
	}
}

/**********************************************************************
 * @fn      SimpleBLECentral_timeoutConnecting
 *
 * @brief   Post event if connecting is timed out.
 *
 * @return  none
 */
Void SimpleBLECentral_timeoutConnecting(UArg arg0)
{
    if (state == BLE_STATE_CONNECTING)
    {
    	SimpleBLECentral_enqueueMsg(SBC_CONNECTING_TIMEOUT_EVT, 0, NULL);
    }
}

/*********************************************************************
 * @fn      SimpleBLECentral_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
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
 * @fn      SimpleBLECentral_findLocalName
 *
 * @brief   Check if pEvtData contains a device local name
 *
 * @return  TRUE if local name found
 */
static bool SimpleBLECentral_findLocalName(uint8_t *pEvtData, uint8_t dataLen)
{
	uint8_t adLen;
	uint8_t adType;
	uint8_t *pEnd;

	pEnd = pEvtData + dataLen - 1;

	// While end of data not reached
	while (pEvtData < pEnd)
	{
		// Get length of next data item
	    adLen = *pEvtData++;
	    if (adLen > 0)
	    {
	    	adType = *pEvtData;

	    	// If AD type is for local name
	    	if ((adType == GAP_ADTYPE_LOCAL_NAME_SHORT) ||
	            (adType == GAP_ADTYPE_LOCAL_NAME_COMPLETE))
	    	{
	    		pEvtData++;
	    		adLen--;
	    		// For each local name in list
	    		if (adLen >= 2 && pEvtData < pEnd)
	    		{
	    			return TRUE;
	    		}

	    		// Handle possible erroneous extra byte in advertisement data
	    		if (adLen == 1)
	    		{
	    			pEvtData++;
	    		}
	    	}
	    	else
	    	{
	    		// Go to next item
	    		pEvtData += adLen;
	    	}
	    }
	}
	// No name found
	return FALSE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_addDeviceName
 *
 * @brief   Add a name to an existing device in the scan result list
 *
 * @return  none
 */
static void SimpleBLECentral_addDeviceName(uint8_t i, uint8_t *pEvtData, uint8_t dataLen)
{
	uint8_t scanRspLen;
	uint8_t scanRspType;
	uint8_t *pEnd;

	pEnd = pEvtData + dataLen - 1;

	// While end of data not reached
	while (pEvtData < pEnd)
	{
		// Get length of next scan response item
		scanRspLen = *pEvtData++;
		if (scanRspLen > 0)
		{
			scanRspType = *pEvtData;

			// If scan response type is for local name
			if ((scanRspType == GAP_ADTYPE_LOCAL_NAME_SHORT) ||
		        (scanRspType == GAP_ADTYPE_LOCAL_NAME_COMPLETE))
			{
				//Set name length in the device struct.
				devList[i].nameLength = scanRspLen - 1;
				pEvtData++;
				uint8_t j = 0;

				//Copy device name from the scan response data
				while ((pEvtData < pEnd) &&
		    		   (j < scanRspLen - 1))
				{
					devList[i].localName[j] = *pEvtData;
					pEvtData++;
					j++;
				}
			}
		}
		else
		{
			// Go to next scan response item
			pEvtData += scanRspLen;
		}
	}
}


/*********************************************************************
 * @fn      SimpleBLECentral_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent)
{
	// Forward the role event to the application
	if (SimpleBLECentral_enqueueMsg(SBC_STATE_CHANGE_EVT,
                                    SUCCESS, (uint8_t *)pEvent))
	{
		// App will process and free the event
		return FALSE;
	}

	// Caller should free the event
	return TRUE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t pairState,
                                         uint8_t status)
{
	uint8_t *pData;

	// Allocate space for the event data.
	if ((pData = ICall_malloc(sizeof(uint8_t))))
	{
		*pData = status;

		// Queue the event.
		SimpleBLECentral_enqueueMsg(SBC_PAIRING_STATE_EVT, pairState, pData);
	}
}

/*********************************************************************
 * @fn      SimpleBLECentral_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    SimpleBLECentral_enqueueMsg(SBC_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_startDiscHandler(UArg a0)
{
	events |= SBC_START_DISCOVERY_EVT;

	// Wake up the application thread when it waits for clock event
	Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLECentral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_keyChangeHandler(uint8_t keys)
{
	SimpleBLECentral_enqueueMsg(SBC_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_readRssiHandler
 *
 * @brief   Read RSSI handler function
 *
 * @param   a0 - read RSSI index
 *
 * @return  none
 */
void SimpleBLECentral_readRssiHandler(UArg a0)
{
	SimpleBLECentral_enqueueMsg(SBC_RSSI_READ_EVT, SUCCESS,
                              (uint8_t *)&readRssi[a0]);
}

/*********************************************************************
 * @fn      SimpleBLECentral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t status,
                                           uint8_t *pData)
{
	sbcEvt_t *pMsg = ICall_malloc(sizeof(sbcEvt_t));

	// Create dynamic pointer to message.
	if (pMsg)
	{
		pMsg->hdr.event = event;
		pMsg->hdr.state = status;
		pMsg->pData = pData;

		// Enqueue the message.
		return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
	}
	return FALSE;
}
