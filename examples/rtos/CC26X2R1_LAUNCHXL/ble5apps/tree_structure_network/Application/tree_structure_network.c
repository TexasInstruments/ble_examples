/******************************************************************************

@file  tree_structure_network.c

@brief This file contains the tree_structure_network sample application for use
with the TI Bluetooth Low Energy Protocol Stack.

Group: CMCU, LPRF
Target Device: CC2652

******************************************************************************

 Copyright (c) 2013-2019, Texas Instruments Incorporated
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

*****************************************************************************/

/*********************************************************************
* INCLUDES
*/
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#if !(defined __TI_COMPILER_VERSION__)
#include <intrinsics.h>
#endif

#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include <tree_network_service.h>

#include <board.h>
#include <board_key.h>

#include "tree_structure_network.h"

/*********************************************************************
* CONSTANTS
*/

//Char value table length
#define string_index_max 8

//Send char value table
uint8_t charVals[string_index_max][TREENETWORKSERVICE_CHAR1_LEN] = { 
                            {'0', '1', '0', '0', '\0'}, //Father 1 led off
                            {'0', '1', '1', '0', '\0'}, //Father 1's child 1 led off
                            {'0', '2', '0', '0', '\0'}, //Father 2 led off
                            {'0', '2', '1', '0', '\0'}, //Father 2's child 1 led off
                            {'0', '1', '0', '1', '\0'}, //Father 1 led on
                            {'0', '1', '1', '1', '\0'}, //Father 1's child 1 led on
                            {'0', '2', '0', '1', '\0'}, //Father 2 led on
                            {'0', '2', '1', '1', '\0'}  //Father 2's child 1 led on
							// Reserved
                            	 // Father index
                                      // Father branch elements (child index)
                                           // Led ON/OFF
                            }; // Should be consistent with those in GattWrite

// defgroup GAP_Profile_Roles GAP Profile Roles
#define GAP_PROFILE_MULTIROLE       0x10

// Address mode of the local device
// Note: When using the DEFAULT_ADDRESS_MODE as ADDRMODE_RANDOM or 
// ADDRMODE_RP_WITH_RANDOM_ID, GAP_DeviceInit() should be called with 
// it's last parameter set to a static random address
#define DEFAULT_ADDRESS_MODE                  ADDRMODE_RP_WITH_PUBLIC_ID

// Application events
#define TSN_EVT_CHAR_CHANGE         1
#define TSN_EVT_KEY_CHANGE          2
#define TSN_EVT_ADV_REPORT          3
#define TSN_EVT_SCAN_ENABLED        4
#define TSN_EVT_SCAN_DISABLED       5
#define TSN_EVT_SVC_DISC            6
#define TSN_EVT_ADV                 7
#define TSN_EVT_PAIRING_STATE       8
#define TSN_EVT_PASSCODE_NEEDED     9
#define TSN_EVT_SEND_PARAM_UPDATE   10
#define TSN_EVT_PERIODIC            11
#define TSN_EVT_READ_RPA            12
#define TSN_EVT_INSUFFICIENT_MEM    13
#define TSN_EVT_EN_NOTI             14
#define TSN_EVT_WRITE_CHAR          15
#define TSN_EVT_CHAR1_NOTI          16

// Internal Events for RTOS application
#define TSN_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define TSN_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define TSN_ALL_EVENTS                        (TSN_ICALL_EVT           | \
                                               TSN_QUEUE_EVT)

// Default PHY for scanning
#define DEFAULT_SCAN_PHY                     SCAN_PRIM_PHY_1M

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Default scan duration in 10 ms
#define DEFAULT_SCAN_DURATION                 200 // 2 sec

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Minimum connection interval (units of 1.25ms, 80=100ms) for parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 104=130ms) for  parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     104

// Slave latency to use for parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 300=3s) for parameter update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          300

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_PASS_TO_APP

// Task configuration
#define TSN_TASK_PRIORITY                     1
#ifndef TSN_TASK_STACK_SIZE
#define TSN_TASK_STACK_SIZE                   1024
#endif

// Advertising report fields to keep in the list
// Interested in only peer address type and peer address
#define TSN_ADV_RPT_FIELDS   (SCAN_ADVRPT_FLD_ADDRTYPE | SCAN_ADVRPT_FLD_ADDRESS)

// Discovery states
typedef enum {
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR,                 // Characteristic discovery
  BLE_DISC_STATE_ENABLE_CHAR_NOTIFICATION,
  BLE_DISC_STATE_WRITE_CHAR
} discState_t;

// Row numbers for two-button menu
#define TBM_ROW_APP            0
#define TSN_ROW_NON_CONN      (TBM_ROW_APP + 0)
#define TSN_ROW_MYADDRSS      (TBM_ROW_APP + 1)
#define TSN_ROW_RPA           (TBM_ROW_APP + 2)
#define TSN_ROW_ADVERTIS      (TBM_ROW_APP + 3)
#define TSN_ROW_NUM_CONN      (TBM_ROW_APP + 4)
#define TSN_ROW_CUR_CONN      (TBM_ROW_APP + 5)
#define TSN_ROW_SECURITY      (TBM_ROW_APP + 6)
#define TSN_ROW_CHARSTAT      (TBM_ROW_APP + 7)
#define TSN_ROW_SEL_CONN      (TBM_ROW_APP + 8)
#define TSN_ROW_MYROLE        (TBM_ROW_APP + 9)
#define TSN_ROW_MYMSG         (TBM_ROW_APP + 10)
#define TSN_ROW_NOTI          (TBM_ROW_APP + 11)

// address string length is an ascii character for each digit +
#define TSN_ADDR_STR_SIZE    	 			  15

// How often to perform periodic event (in msec)
#define TSN_PERIODIC_EVT_PERIOD               1000

// How often to read current current RPA (in ms)
#define TSN_EVT_READ_RPA_PERIOD               3000

// Delay (in ms) after connection establishment before sending a parameter update requst
#define TSN_SEND_PARAM_UPDATE_DELAY           6000

#define CONNINDEX_INVALID  0xFF

// Spin if the expression is not true
#define TREESTRUCTURENETWORK_ASSERT(expr) if (!(expr)) tree_structure_network_spin();

/*********************************************************************
* TYPEDEFS
*/

// App event passed from profiles.
typedef struct
{
  uint8_t event;    // event type
  void *pData;   // event data pointer
} tsnEvt_t;

// Container to store paring state info when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} tsnPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} tsnPasscodeData_t;

// Scanned device information record
typedef struct
{
  uint8_t addrType;         // Peer Device's Address Type
  uint8_t addr[B_ADDR_LEN]; // Peer Device Address
} scanRec_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;
  uint8_t data[];
} tsnClockEventData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} tsnGapAdvEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
  List_Elem elem;
  uint16_t  connHandle;
} tsnConnHandleEntry_t;

// Connected device information
typedef struct
{
  uint16_t         connHandle;           // Connection Handle
  uint8_t          addr[B_ADDR_LEN];     // Peer Device Address
  uint8_t          charHandle;           // Characteristic Handle
  Clock_Struct*    pUpdateClock;         // pointer to clock struct
  uint8_t          discState;            // Per connection discovery state
} tsnConnRec_t;

// Container to store notify event data when passing from GATTMsg event. 
typedef struct
{
  uint16_t connHandle;
  uint8_t data[TREENETWORKSERVICE_CHAR1_LEN];
} tsnNotiEventData_t;

/*********************************************************************
* GLOBAL VARIABLES
*/

// Display Interface
Display_Handle dispHandle = NULL;

/* Pin driver handles */
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State ledPinState;

/*
 * Initial LED pin configuration table
 *   - LEDs Board_PIN_LED0 & Board_PIN_LED1 are off.
 */
PIN_Config ledPinTable[] = {
    Board_PIN_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL |
    PIN_DRVSTR_MAX,
    Board_PIN_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL |
    PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*********************************************************************
* LOCAL VARIABLES
*/

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct clkPeriodic;

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass periodic event to clock handler
tsnClockEventData_t periodicUpdateData =
{
  .event = TSN_EVT_PERIODIC
};

// Memory to pass RPA read event ID to clock handler
tsnClockEventData_t argRpaRead =
{
  .event = TSN_EVT_READ_RPA
};

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct tsnTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(tsnTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t tsnTaskStack[TSN_TASK_STACK_SIZE];

static uint8_t scanRspData[] =
{
  // complete name
  18,      // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'T', 'r', 'e', 'e', ' ', 'N', 'e', 't', 'w', 'o', 'r', 'k', ' ', 'N', 'o', 'd', 'e',

  // Tx power level
  2,      // length of this data
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
  LO_UINT16(TREENETWORKSERVICE_SERV_UUID),
  HI_UINT16(TREENETWORKSERVICE_SERV_UUID)
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Tree Network Device";

// Maximim PDU size (default = 27 octets)
static uint16 tsnMaxPduSize;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
// Number of scan results filtered by Service UUID
static uint8_t numScanRes = 0;

// Scan results filtered by Service UUID
static scanRec_t scanList[DEFAULT_MAX_SCAN_RES];
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Number of connected devices
static uint8_t numConn = 0;

// Connection handle of current connection
static uint16_t tsnConnHandle = LINKDB_CONNHANDLE_INVALID;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Per-handle connection info
static tsnConnRec_t connList[MAX_NUM_BLE_CONNS];

// Advertising handles
static uint8 advHandle;

static bool tsnIsAdvertising = false;
// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

// Initiating PHY
static uint8_t tsnInitPhy = INIT_PHY_1M;

// Periodic task enable flag
static uint8_t en_period_task = 0;
static volatile uint32_t bytesRecvd = 0;

// Tree structure role defination
volatile uint8_t my_role = 0;

// Grandfather index
volatile uint8_t grandfather_index = 0;

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void tree_structure_network_init(void);
static void tree_structure_network_scanInit(void);
static void tree_structure_network_advertInit(void);
static void tree_structure_network_taskFxn(UArg a0, UArg a1);
static uint8_t tree_structure_network_processStackMsg(ICall_Hdr *pMsg);
static uint8_t tree_structure_network_processGATTMsg(gattMsgEvent_t *pMsg);
static void tree_structure_network_processAppMsg(tsnEvt_t *pMsg);
static void tree_structure_network_processCharValueChangeEvt(uint8_t paramID);
static void tree_structure_network_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void tree_structure_network_processPasscode(tsnPasscodeData_t *pData);
static void tree_structure_network_processPairState(tsnPairStateData_t* pairingEvent);
static void tree_structure_network_processGapMsg(gapEventHdr_t *pMsg);
static void tree_structure_network_processParamUpdate(uint16_t connHandle);
static void tree_structure_network_processAdvEvent(tsnGapAdvEventData_t *pEventData);
static void tree_structure_network_charValueChangeCB(uint8_t paramID);
static status_t tree_structure_network_enqueueMsg(uint8_t event, void *pData);
static void tree_structure_network_handleKeys(uint8_t keys);
static uint16_t tree_structure_network_getConnIndex(uint16_t connHandle);
static void tree_structure_network_keyChangeHandler(uint8_t keys);
static uint8_t tree_structure_network_addConnInfo(uint16_t connHandle, uint8_t *pAddr, uint8_t role);
static void tree_structure_network_performPeriodicTask(void);
static void tree_structure_network_clockHandler(UArg arg);
static uint8_t tree_structure_network_clearConnListEntry(uint16_t connHandle);
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
static void tree_structure_network_addScanInfo(uint8_t *pAddr, uint8_t addrType);
static bool tree_structure_network_findSvcUuid(uint16_t uuid, uint8_t *pData, uint16_t dataLen);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
static uint8_t tree_structure_network_removeConnInfo(uint16_t connHandle);
static void tree_structure_network_startSvcDiscovery(void);
#ifndef Display_DISABLE_ALL
static char* tree_structure_network_getConnAddrStr(uint16_t connHandle);
#endif
static void tree_structure_network_advCB(uint32_t event, void *pBuf, uintptr_t arg);
static void tree_structure_network_scanCB(uint32_t evt, void* msg, uintptr_t arg);
static void tree_structure_network_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);
static void tree_structure_network_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status);
static void tree_structure_network_updateRPA(void);
bool tree_structure_network_doGattWriteString(uint8_t internal_tsnConnHandle, uint8_t *pAddr);
bool tree_structure_network_doGattWriteEnableNotification();
static void tree_structure_network_processChar1ValueNotiEvt(tsnNotiEventData_t *pMsg);

/*********************************************************************
 * EXTERN FUNCTIONS
*/
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
* PROFILE CALLBACKS
*/

// Simple GATT Profile Callbacks
static treeNetworkServiceCBs_t tree_structure_network_treeNetworkServiceCBs =
{
  tree_structure_network_charValueChangeCB // Characteristic value change callback
};

// GAP Bond Manager Callbacks
static gapBondCBs_t tree_structure_network_BondMgrCBs =
{
  tree_structure_network_passcodeCB, // Passcode callback
  tree_structure_network_pairStateCB                  // Pairing state callback
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
 * @fn      tree_structure_network_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void tree_structure_network_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
* @fn      tree_structure_network_createTask
*
* @brief   Task creation function for tree_structure_network.
*
* @param   None.
*
* @return  None.
*/
void tree_structure_network_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = tsnTaskStack;
  taskParams.stackSize = TSN_TASK_STACK_SIZE;
  taskParams.priority = TSN_TASK_PRIORITY;

  Task_construct(&tsnTask, tree_structure_network_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      tree_structure_network_init
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
static void tree_structure_network_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Open Display.
  //dispHandle = Display_open(Display_Type_ANY, NULL);
  dispHandle = Display_open(Display_Type_UART, NULL);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clock for internal periodic events.
  Util_constructClock(&clkPeriodic, tree_structure_network_clockHandler,
                      TSN_PERIODIC_EVT_PERIOD, 0, false,
                      (UArg)&periodicUpdateData);

  // Open LED pins
  ledPinHandle = PIN_open(&ledPinState, ledPinTable);
  if(!ledPinHandle)
  {
      Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Error initializing board LED pins");
      Task_exit();
  }
  
  // Init key debouncer
  Board_initKeys(tree_structure_network_keyChangeHandler);

  // Initialize Connection List
  tree_structure_network_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide.
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (void *)attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
  #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
  #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command.
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client, used by GAPBondMgr to look for RPAO characteristic for network privacy
  GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Setup the GAP Bond Manager
  {
    // Send a pairing request after connecting.
    uint8_t pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    // Use authenticated pairing: require passcode.
    uint8_t mitm = TRUE;
    // This device only has display capabilities. Therefore, it will display the
    // passcode during pairing. However, since the default passcode is being
    // used, there is no need to display anything.
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    // Request bonding (storing long-term keys for re-encryption upon subsequent
    // connections without repairing)
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);                // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);        // GATT attributes
  DevInfo_AddService();                             // Device Information Service
  TreeNetworkService_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

  // Setup the TreeNetworkService Characteristic Values
  // For more information, see the GATT and GATTServApp sections in the User's Guide.
    uint8_t charValue1[TREENETWORKSERVICE_CHAR1_LEN] = { 1, 2, 3, 4};

    TreeNetworkService_SetParameter(TREENETWORKSERVICE_CHAR1, TREENETWORKSERVICE_CHAR1_LEN,
                               charValue1);

  // Register callback with SimpleGATTprofile
  TreeNetworkService_RegisterAppCBs(&tree_structure_network_treeNetworkServiceCBs);

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&tree_structure_network_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide.
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  //Initialize GAP layer for Peripheral and Central role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL, selfEntity,
                 addrMode, NULL);
}

/*********************************************************************
* @fn      tree_structure_network_taskFxn
*
* @brief   Application task entry point for the tree_structure_network.
*
* @param   a0, a1 - not used.
*
* @return  None.
*/
static void tree_structure_network_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  tree_structure_network_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, TSN_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
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
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = tree_structure_network_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & TSN_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          tsnEvt_t *pMsg = (tsnEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            tree_structure_network_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
* @fn      tree_structure_network_processStackMsg
*
* @brief   Process an incoming stack message.
*
* @param   pMsg - message to process
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t tree_structure_network_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      tree_structure_network_processGapMsg((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = tree_structure_network_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch (pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
          break;

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

        // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
          {
            hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
            switch ( pMyMsg->cmdOpcode )
            {
              case HCI_LE_SET_PHY:
                {
                  if (pMyMsg->cmdStatus ==
                      HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
                  {
                    Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0,
                            "PHY Change failure, peer does not support this");
                  }
                  else
                  {
                    Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0,
                                   "PHY Update Status: 0x%02x",
                                   pMyMsg->cmdStatus);
                  }
                }
                break;
              case HCI_DISCONNECT:
                break;

              default:
                {
                  Display_printf(dispHandle, TSN_ROW_NON_CONN, 0,
                                 "Unknown Cmd Status: 0x%04x::0x%02x",
                                 pMyMsg->cmdOpcode, pMyMsg->cmdStatus);
                }
              break;
            }
          }
          break;

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC
            = (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
          {
            if (pPUC->status != SUCCESS)
            {

              Display_printf(dispHandle, TSN_ROW_SECURITY, 0,
                             "%s: PHY change failure",
                             tree_structure_network_getConnAddrStr(pPUC->connHandle));
            }
            else
            {
              Display_printf(dispHandle, TSN_ROW_SECURITY, 0,
                             "%s: PHY updated to %s",
                             tree_structure_network_getConnAddrStr(pPUC->connHandle),
              // Only symmetrical PHY is supported.
              // rxPhy should be equal to txPhy.
                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1 Mbps" :
                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2 Mbps" :
                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "Coded" : "Unexpected PHY Value");
            }
          }

          break;
        }

        default:
          break;
      }

      break;
    }

    case L2CAP_SIGNAL_EVENT:
      // place holder for L2CAP Connection Parameter Reply
      break;

    default:
      // Do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      tree_structure_network_processGapMsg
 *
 * @brief   GAP message processing function.
 *
 * @param   pMsg - pointer to event message structure
 *
 * @return  none
 */
static void tree_structure_network_processGapMsg(gapEventHdr_t *pMsg)
{
  switch (pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      if(pPkt->hdr.status == SUCCESS)
      {
        // Store the system ID
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = pPkt->devAddr[0];
        systemId[1] = pPkt->devAddr[1];
        systemId[2] = pPkt->devAddr[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = pPkt->devAddr[5];
        systemId[6] = pPkt->devAddr[4];
        systemId[5] = pPkt->devAddr[3];

        // Set Device Info Service Parameter
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        //Setup and start advertising
        tree_structure_network_advertInit();

        // Set device info characteristic
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, pPkt->devAddr);
      }

      //Setup scanning
      tree_structure_network_scanInit();

      tsnMaxPduSize = pPkt->dataPktLen;

      //Display initialized state status
      Display_printf(dispHandle, TSN_ROW_NUM_CONN, 0, "Num Conns: %d", numConn);
      Display_printf(dispHandle, TSN_ROW_NON_CONN, 0, "Initialized");
      Display_printf(dispHandle, TSN_ROW_MYADDRSS, 0, "Tree Device Address: %s",(char *)Util_convertBdAddr2Str(pPkt->devAddr));

      break;
    }

    case GAP_CONNECTING_CANCELLED_EVENT:
    {
      Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0,
                     "Conneting attempt cancelled");
      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      uint16_t connHandle = ((gapEstLinkReqEvent_t*) pMsg)->connectionHandle;
      // Update the role type when the lnk established
      uint8_t role = ((gapEstLinkReqEvent_t*) pMsg)->connRole;
      static uint8_t last_role;
      
      // Judge the role change
      if (  ( (role == GAP_PROFILE_CENTRAL) && (last_role == GAP_PROFILE_PERIPHERAL) ) 
              || ( (role == GAP_PROFILE_PERIPHERAL) && (last_role == GAP_PROFILE_CENTRAL) )  )
      {
        // New father node
        my_role = GAP_PROFILE_MULTIROLE;
      }
      else
      {
        // Child or grandpa node
        my_role = role;
      }
	  
      // Save the last role state
      last_role = role;
      
      uint8_t* pAddr      = ((gapEstLinkReqEvent_t*) pMsg)->devAddr;
      uint8_t  connIndex;
      uint8_t* pStrAddr;
      
      // Add this connection info to the list
      connIndex = tree_structure_network_addConnInfo(connHandle, pAddr, role);
      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      TREESTRUCTURENETWORK_ASSERT(connIndex < MAX_NUM_BLE_CONNS);
	   connList[connIndex].charHandle = 0;

      pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);
	  
      Display_printf(dispHandle, TSN_ROW_NON_CONN, 0, "Connected to %s", pStrAddr);
      Display_printf(dispHandle, TSN_ROW_NUM_CONN, 0, "Num Conns: %d", numConn);

      tsnConnHandle = connHandle;

      // Only central do the connection
      if (my_role == GAP_PROFILE_CENTRAL)
      {
        tree_structure_network_doSelectConn(connIndex);
      }
      else if (my_role == GAP_PROFILE_MULTIROLE)
      {
        tree_structure_network_doSelectConn(connIndex);
      }
      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      uint16_t connHandle = ((gapTerminateLinkEvent_t*) pMsg)->connectionHandle;
      uint8_t connIndex;
      uint8_t* pStrAddr;
      
      // Mark this connection deleted in the connected device list.
      connIndex = tree_structure_network_removeConnInfo(connHandle);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      TREESTRUCTURENETWORK_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

      Display_printf(dispHandle, TSN_ROW_NON_CONN, 0, "%s is disconnected",
                     pStrAddr);
      Display_printf(dispHandle, TSN_ROW_NUM_CONN, 0, "Num Conns: %d", numConn);
      Display_clearLines(dispHandle, TSN_ROW_CUR_CONN, TSN_ROW_NOTI);
      
      PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, 0x00);
      PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 0x00);

      // Start advertising since there is room for more connections
      GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

      // If no active connections
      if (numConn == 0)
      {
        // Stop periodic clock
        Util_stopClock(&clkPeriodic);
      }

      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;
      gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;

      rsp.connectionHandle = pReq->req.connectionHandle;
      rsp.signalIdentifier = pReq->req.signalIdentifier;

      // Only accept connection intervals with slave latency of 0
      // This is just an example of how the application can send a response
      if(pReq->req.connLatency == 0)
      {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

      break;
    }

     case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

        // Get the address from the connection handle
        linkDBInfo_t linkInfo;
        if (linkDB_GetInfo(pPkt->connectionHandle, &linkInfo) ==  SUCCESS)
        {

          if(pPkt->status == SUCCESS)
          {
            Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0,
                          "Link Param Updated: %s",
                           Util_convertBdAddr2Str(linkInfo.addr));
          }
          else
          {
            // Display the address of the connection update failure
            Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0,
                           "Link Param Update Failed 0x%h: %s", pPkt->opcode,
                           Util_convertBdAddr2Str(linkInfo.addr));
          }
        }
        // Check if there are any queued parameter updates
        tsnConnHandleEntry_t *connHandleEntry = (tsnConnHandleEntry_t *)List_get(&paramUpdateList);
        if (connHandleEntry != NULL)
        {
          // Attempt to send queued update now
          tree_structure_network_processParamUpdate(connHandleEntry->connHandle);

          // Free list element
          ICall_free(connHandleEntry);
        }
        break;
      }


    default:
      break;
  }
}

/*********************************************************************
* @fn      tree_structure_network_scanInit
*
* @brief   Setup initial device scan settings.
*
* @return  None.
*/
static void tree_structure_network_scanInit(void)
{
  uint8_t temp8;
  uint16_t temp16;

  // Setup scanning
  // For more information, see the GAP section in the User's Guide.

  // Register callback to process Scanner events
  GapScan_registerCb(tree_structure_network_scanCB, NULL);

  // Set Scanner Event Mask
  GapScan_setEventMask(GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED |
                       GAP_EVT_ADV_REPORT);

  // Set Scan PHY parameters
  GapScan_setPhyParams(DEFAULT_SCAN_PHY, SCAN_TYPE_ACTIVE,
                       SCAN_PARAM_DFLT_INTERVAL, SCAN_PARAM_DFLT_INTERVAL);

  // Set Advertising report fields to keep
  temp16 = TSN_ADV_RPT_FIELDS;
  GapScan_setParam(SCAN_PARAM_RPT_FIELDS, &temp16);
  // Set Scanning Primary PHY
  temp8 = DEFAULT_SCAN_PHY;
  GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);
  // Set LL Duplicate Filter
  temp8 = SCAN_FLT_DUP_ENABLE;
  GapScan_setParam(SCAN_PARAM_FLT_DUP, &temp8);

  // Set PDU type filter -
  // Only 'Connectable' and 'Complete' packets are desired.
  // It doesn't matter if received packets are
  // whether Scannable or Non-Scannable, whether Directed or Undirected,
  // whether Scan_Rsp's or Advertisements, and whether Legacy or Extended.
  temp16 = SCAN_FLT_PDU_CONNECTABLE_ONLY | SCAN_FLT_PDU_COMPLETE_ONLY;
  GapScan_setParam(SCAN_PARAM_FLT_PDU_TYPE, &temp16);
}

/*********************************************************************
* @fn      tree_structure_network_scanInit
*
* @brief   Setup initial advertisment and start advertising from device init.
*
* @return  None.
*/
static void tree_structure_network_advertInit(void)
{
  uint8_t status = FAILURE;
  // Setup and start Advertising
  // For more information, see the GAP section in the User's Guide.

  // Temporary memory for advertising parameters for set #1. These will be copied
  // by the GapAdv module
  GapAdv_params_t advParam = GAPADV_PARAMS_LEGACY_SCANN_CONN;

  // Create Advertisement set #1 and assign handle
  GapAdv_create(&tree_structure_network_advCB, &advParam,
                &advHandle);

  // Load advertising data for set #1 that is statically allocated by the app
  GapAdv_loadByHandle(advHandle, GAP_ADV_DATA_TYPE_ADV,
                      sizeof(advertData), advertData);

  // Load scan response data for set #1 that is statically allocated by the app
  GapAdv_loadByHandle(advHandle, GAP_ADV_DATA_TYPE_SCAN_RSP,
                      sizeof(scanRspData), scanRspData);

  // Set event mask for set #1
  GapAdv_setEventMask(advHandle,
                      GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                      GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                      GAP_ADV_EVT_MASK_SET_TERMINATED);

  // Enable legacy advertising for set #1
  status = GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

  if(status != SUCCESS)
  {
    tsnIsAdvertising = false;
    Display_printf(dispHandle, TSN_ROW_ADVERTIS, 0, "Error: Failed to Start Advertising!");
  }

  if (addrMode > ADDRMODE_RANDOM)
  {
    tree_structure_network_updateRPA();

    // Create one-shot clock for RPA check event.
    Util_constructClock(&clkRpaRead, tree_structure_network_clockHandler,
                        TSN_EVT_READ_RPA_PERIOD, 0, true,
                        (UArg) &argRpaRead);
  }
}

/*********************************************************************
 * @fn      tree_structure_network_advCB
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void tree_structure_network_advCB(uint32_t event, void *pBuf, uintptr_t arg)
{
  tsnGapAdvEventData_t *pData = ICall_malloc(sizeof(tsnGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(tree_structure_network_enqueueMsg(TSN_EVT_ADV, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}


/*********************************************************************
* @fn      tree_structure_network_processGATTMsg
*
* @brief   Process GATT messages and events.
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t tree_structure_network_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // Get connection index from handle
  uint8_t connIndex = tree_structure_network_getConnIndex(pMsg->connHandle);
  TREESTRUCTURENETWORK_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }


  // Messages from GATT server
  if (linkDB_Up(pMsg->connHandle))
  {
    if ((pMsg->method == ATT_READ_RSP)   ||
        ((pMsg->method == ATT_ERROR_RSP) &&
         (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
      }

    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {

      if (pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP)
      {
        Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a succesful write, display the value that was written and
        // increment value
        Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "write success\n\n", connIndex + 1);
      }
    }
    else if (pMsg->method == ATT_HANDLE_VALUE_NOTI)
    {
      {
        //Post the notification transmit task
        tsnNotiEventData_t *paramUpdateNotiData;
        paramUpdateNotiData = ICall_malloc( sizeof(tsnNotiEventData_t) );
        paramUpdateNotiData->connHandle = connIndex;
        memcpy(paramUpdateNotiData->data, pMsg->msg.handleValueNoti.pValue, TREENETWORKSERVICE_CHAR1_LEN);
        tree_structure_network_enqueueMsg(TSN_EVT_CHAR1_NOTI, paramUpdateNotiData);
      }
    }
    if (connList[connIndex].discState != BLE_DISC_STATE_IDLE)
    {
      tree_structure_network_processGATTDiscEvent(pMsg);
    }
  } // Else - in case a GATT message came after a connection has dropped, ignore it.

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      tree_structure_network_processParamUpdate
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static void tree_structure_network_processParamUpdate(uint16_t connHandle)
{
  gapUpdateLinkParamReq_t req;
  uint8_t connIndex;

  req.connectionHandle = connHandle;
  req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
  req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
  req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
  req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

  connIndex = tree_structure_network_getConnIndex(connHandle);
  TREESTRUCTURENETWORK_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Deconstruct the clock object
  Clock_destruct(connList[connIndex].pUpdateClock);
  // Free clock struct
  ICall_free(connList[connIndex].pUpdateClock);
  connList[connIndex].pUpdateClock = NULL;

  // Send parameter update
  bStatus_t status = GAP_UpdateLinkParamReq(&req);

  // If there is an ongoing update, queue this for when the udpate completes
  if (status == bleAlreadyInRequestedMode)
  {
    tsnConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(tsnConnHandleEntry_t));
    if (connHandleEntry)
    {
      connHandleEntry->connHandle = connHandle;

      List_put(&paramUpdateList, (List_Elem *)&connHandleEntry);
    }
  }
}

/*********************************************************************
* @fn      tree_structure_network_processAppMsg
*
* @brief   Process an incoming callback from a profile.
*
* @param   pMsg - message to process
*
* @return  None.
*/
static void tree_structure_network_processAppMsg(tsnEvt_t *pMsg)
{
  bool safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case TSN_EVT_CHAR_CHANGE:
    {
      tree_structure_network_processCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;
    }

    case TSN_EVT_KEY_CHANGE:
    {
      tree_structure_network_handleKeys(*(uint8_t *)(pMsg->pData));
      break;
    }

    case TSN_EVT_ADV_REPORT:
    {
      GapScan_Evt_AdvRpt_t* pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      if (tree_structure_network_findSvcUuid(TREENETWORKSERVICE_SERV_UUID,
                                 pAdvRpt->pData, pAdvRpt->dataLen))
      {
        tree_structure_network_addScanInfo(pAdvRpt->addr, pAdvRpt->addrType);
        Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Discovered: %s",
                       Util_convertBdAddr2Str(pAdvRpt->addr));
      }
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
      Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Discovered: %s",
                     Util_convertBdAddr2Str(pAdvRpt->addr));
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

      // Free scan payload data
      if (pAdvRpt->pData != NULL)
      {
        ICall_free(pAdvRpt->pData);
      }
      break;
    }

    case TSN_EVT_SCAN_ENABLED:
    {
      Display_printf(dispHandle, TSN_ROW_NUM_CONN, 0, "Discovering...");
      break;
    }

    case TSN_EVT_SCAN_DISABLED:
    {
      uint8_t numReport;
      uint8_t i;
      static uint8_t* pAddrs = NULL;
      uint8_t* pAddrTemp;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      numReport = numScanRes;
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
      GapScan_Evt_AdvRpt_t advRpt;

      numReport = ((GapScan_Evt_End_t*) (pMsg->pData))->numReport;
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

      Display_printf(dispHandle, TSN_ROW_ADVERTIS, 0,
                     "%d devices discovered", numReport);

      if (numReport > 0)
      {
        // Also enable "Connect to"
        tree_structure_network_doConnect(0);
      }

      // Allocate buffer to display addresses
      pAddrs = ICall_malloc(numReport * TSN_ADDR_STR_SIZE);
      if (pAddrs == NULL)
      {
        numReport = 0;
      }

      if (pAddrs != NULL)
      {
        pAddrTemp = pAddrs;
        for (i = 0; i < numReport; i++, pAddrTemp += TSN_ADDR_STR_SIZE)
        {
  #if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
          // Get the address from the list, convert it to string, and
          // copy the string to the address buffer
          memcpy(pAddrTemp, Util_convertBdAddr2Str(scanList[i].addr),
                 TSN_ADDR_STR_SIZE);
  #else // !DEFAULT_DEV_DISC_BY_SVC_UUID
          // Get the address from the report, convert it to string, and
          // copy the string to the address buffer
          GapScan_getAdvReport(i, &advRpt);
          memcpy(pAddrTemp, Util_convertBdAddr2Str(advRpt.addr),
                 TSN_ADDR_STR_SIZE);
  #endif // DEFAULT_DEV_DISC_BY_SVC_UUID
        }
      }
      break;
    }

    case TSN_EVT_SVC_DISC:
    {
      tree_structure_network_startSvcDiscovery();
      break;
    }

    case TSN_EVT_ADV:
    {
      tree_structure_network_processAdvEvent((tsnGapAdvEventData_t*)(pMsg->pData));
      break;
    }

    case TSN_EVT_PAIRING_STATE:
    {
      tree_structure_network_processPairState((tsnPairStateData_t*)(pMsg->pData));
      break;
    }

    case TSN_EVT_PASSCODE_NEEDED:
    {
      tree_structure_network_processPasscode((tsnPasscodeData_t*)(pMsg->pData));
      break;
    }

    case TSN_EVT_SEND_PARAM_UPDATE:
    {
      // Extract connection handle from data
      uint16_t locConnHandle = *(uint16_t *)(((tsnClockEventData_t *)pMsg->pData)->data);
      tree_structure_network_processParamUpdate(locConnHandle);
      safeToDealloc = FALSE;
      break;
    }

    case TSN_EVT_PERIODIC:
    {
      // If the current role is grandpa node:
      if(my_role == GAP_PROFILE_CENTRAL)
        tree_structure_network_performPeriodicTask();
      break;
    }

    case TSN_EVT_READ_RPA:
    {
      tree_structure_network_updateRPA();
      break;
    }

    case TSN_EVT_INSUFFICIENT_MEM:
    {
      // We are running out of memory.
      Display_printf(dispHandle, TSN_ROW_SECURITY, 0, "Insufficient Memory");

      // We might be in the middle of scanning, try stopping it.
      GapScan_disable();
      break;
    }
    
    case TSN_EVT_EN_NOTI:
    {
      tree_structure_network_doGattWriteEnableNotification();
      break;
    }

    case TSN_EVT_WRITE_CHAR:
    {
      tree_structure_network_doGattWriteString(tsnConnHandle, "LED2");
      break;
    }

    //Add notification event
    case TSN_EVT_CHAR1_NOTI:
    {
      tree_structure_network_processChar1ValueNotiEvt((tsnNotiEventData_t*)(pMsg->pData));
      break;
    }
    
    default:
      // Do nothing.
      break;
  }

  if ((safeToDealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      tree_structure_network_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void tree_structure_network_processAdvEvent(tsnGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      tsnIsAdvertising = true;
      Display_printf(dispHandle, TSN_ROW_ADVERTIS, 0, "Advertising Enabled");
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
      tsnIsAdvertising = false;
      Display_printf(dispHandle, TSN_ROW_ADVERTIS, 0, "Advertising Disabled");
      break;

    case GAP_EVT_ADV_START:
      Display_printf(dispHandle, TSN_ROW_ADVERTIS, 0, "Adv Started %d Enabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_END:
      Display_printf(dispHandle, TSN_ROW_ADVERTIS, 0, "Adv Ended %d Disabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
      tsnIsAdvertising = false;
#ifndef Display_DISABLE_ALL
      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);
#endif
      Display_printf(dispHandle, TSN_ROW_ADVERTIS, 0, "Advertising disabled after conn %d",
                     advSetTerm->handle, advSetTerm->connHandle );
    }
    break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
      break;

    default:
      break;
  }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
/*********************************************************************
 * @fn      tree_structure_network_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool tree_structure_network_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                      uint16_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  if (dataLen > 0)
  {
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
  }

  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      tree_structure_network_addScanInfo
 *
 * @brief   Add a device to the scanned device list
 *
 * @return  none
 */
static void tree_structure_network_addScanInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;

  // If result count not at max
  if (numScanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < numScanRes; i++)
    {
      if (memcmp(pAddr, scanList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(scanList[numScanRes].addr, pAddr, B_ADDR_LEN);
    scanList[numScanRes].addrType = addrType;

    // Increment scan result count
    numScanRes++;
  }
}
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

/*********************************************************************
 * @fn      tree_structure_network_scanCB
 *
 * @brief   Callback called by GapScan module
 *
 * @param   evt - event
 * @param   msg - message coming with the event
 * @param   arg - user argument
 *
 * @return  none
 */
void tree_structure_network_scanCB(uint32_t evt, void* pMsg, uintptr_t arg)
{
  uint8_t event;

  if (evt & GAP_EVT_ADV_REPORT)
  {
    event = TSN_EVT_ADV_REPORT;
  }
  else if (evt & GAP_EVT_SCAN_ENABLED)
  {
    event = TSN_EVT_SCAN_ENABLED;
  }
  else if (evt & GAP_EVT_SCAN_DISABLED)
  {
    event = TSN_EVT_SCAN_DISABLED;
  }
  else if (evt & GAP_EVT_INSUFFICIENT_MEMORY)
  {
    event = TSN_EVT_INSUFFICIENT_MEM;
  }
  else
  {
    return;
  }

  if(tree_structure_network_enqueueMsg(event, pMsg) != SUCCESS)
  {
    ICall_free(pMsg);
  }

}

/*********************************************************************
* @fn      tree_structure_network_charValueChangeCB
*
* @brief   Callback from Simple Profile indicating a characteristic
*          value change.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void tree_structure_network_charValueChangeCB(uint8_t paramID)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = paramID;

    // Queue the event.
    if(tree_structure_network_enqueueMsg(TSN_EVT_CHAR_CHANGE, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      tree_structure_network_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static status_t tree_structure_network_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  tsnEvt_t *pMsg = ICall_malloc(sizeof(tsnEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      tree_structure_network_processCharValueChangeEvt
 *
 * @brief   Process a pending Tree Network Service characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void tree_structure_network_processCharValueChangeEvt(uint8_t paramId)
{
  uint8_t newValue[TREENETWORKSERVICE_CHAR1_LEN]={0};
  uint8_t valueToCompare[TREENETWORKSERVICE_CHAR1_LEN] = {'0', '0', '0', '0', '\0'};
  switch(paramId)
  {
    case TREENETWORKSERVICE_CHAR1:
      TreeNetworkService_GetParameter(TREENETWORKSERVICE_CHAR1, &newValue);
      PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, newValue[3] & 0x02);
	  
      Display_printf(dispHandle, TSN_ROW_CHARSTAT, 0, "Char 1: %s", (uintptr_t)newValue);
      { 
        uint8_t i = 0;
        uint8_t state;
        uint8_t child_index = 0;
        uint8_t valid_table[MAX_NUM_BLE_CONNS] ={0};
        uint8_t valid_table_sum = 0;
        
        for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
        {
          if (connList[i].connHandle != LINKDB_CONNHANDLE_INVALID)
            valid_table[i] = 1;
          else
            valid_table[i] = 0;
        }
        
        //get sum
        for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
          valid_table_sum = valid_table_sum + valid_table[i];
        
        //Father node or Children node to do action
        if( (my_role == GAP_PROFILE_PERIPHERAL) || (my_role == GAP_PROFILE_MULTIROLE) )
        {
          if ( strncmp((char const*)newValue, (char const*)valueToCompare, TREENETWORKSERVICE_CHAR1_LEN) )
          {
            //Father type peripheral
            if (valid_table_sum > 1)
            {
              Display_printf(dispHandle, TSN_ROW_MYROLE, 0, "Father %c", newValue[1]);

			  //Is the message to father? (Check newValue[2]). If yes, no transmit required
              if (newValue[2] == '0')
              {
                Display_printf(dispHandle, TSN_ROW_MYMSG, 0, "Received message %s", newValue);
				// set/reset LED
                PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, newValue[3] & 0x01);
              }
			  // Message to child, transmit to the appointed child
              else if ( (newValue[2] - 48) <= (valid_table_sum - 1) )
              {
				// Transpose ASCII value to decimal value (48 =  ASCII 0)
                child_index = newValue[2] - 48;
                state = tree_structure_network_doSelectConn(child_index);
                if (state == false)
                  Display_printf(dispHandle, TSN_ROW_SEL_CONN, 0, "doSelectConn fail: %d", child_index);
                else
                {
				  // Transmit the data
                  tree_structure_network_doGattWriteString(connList[child_index].connHandle, newValue);
                }
              }
              else
              {
                Display_printf(dispHandle, TSN_ROW_SEL_CONN, 0, "doSelectConn fail: %c isn't in connected list", (newValue[2] - 1));
              }
            }
            //child type peripheral
            else if (valid_table_sum == 1)
            {
              if (newValue[0] == 'L')
              {
                // Connected child, no action. 
				Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Connected, GLED ON");
              }
              else
              {
                Display_printf(dispHandle, TSN_ROW_MYROLE, 0, "Father %c's child %c", newValue[1], newValue[2]);
                Display_printf(dispHandle, TSN_ROW_MYMSG, 0, "Get message %s", newValue);
              }
              // set/reset the led
              PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, newValue[3] & 0x01);
            }
          }//End of newValue != {'0000'}
        }//End of peripheral
      }
      break;

    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      tree_structure_network_processChar1ValueNotiEvt
 *
 * @brief   Process a pending Simple Profile characteristic value notification 
 *          event.
 *
 * @param   paramID - parameter ID of the value that was notified.
 */
static void tree_structure_network_processChar1ValueNotiEvt(tsnNotiEventData_t *pMsg)
{
  uint8_t i = 0;
  uint8_t valid_table[MAX_NUM_BLE_CONNS] ={0};
  uint8_t valid_table_sum = 0;
  
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle != LINKDB_CONNHANDLE_INVALID)
      valid_table[i] = 1;
    else
      valid_table[i] = 0;
  }
  
  //get sum
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
    valid_table_sum = valid_table_sum + valid_table[i];
  
  //Father type node
  if (my_role == GAP_PROFILE_MULTIROLE)
  {
    //Father type peripheral, send the notification up to grandfather
    if (valid_table_sum > 1)
    {
      attHandleValueNoti_t noti;
      status_t status;
  
      noti.pValue = (uint8 *)GATT_bm_alloc( grandfather_index, ATT_HANDLE_VALUE_NOTI, TREENETWORKSERVICE_CHAR1_LEN, NULL );
      if ( noti.pValue != NULL )
      {
        uint8_t connIndex = tree_structure_network_getConnIndex(grandfather_index);
        
        noti.handle = connList[connIndex].charHandle;
        noti.len = TREENETWORKSERVICE_CHAR1_LEN;
        pMsg->data[2] = '0' + pMsg->connHandle;     // order number in tree = connHandle
        memcpy(noti.pValue, pMsg->data, TREENETWORKSERVICE_CHAR1_LEN);
		// Transmit the notification
        status = GATT_Notification( grandfather_index, &noti, FALSE );
        Display_printf(dispHandle, TSN_ROW_CHARSTAT, 0, "Sent key press noti.");
      }
      if ( status != SUCCESS )
      {
        GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
      }
    }
  }
  //Grandfather type, display the key value
  else if (my_role == GAP_PROFILE_CENTRAL)
  {
  	// father index starts from 0, so add 1 here
    pMsg->data[1] = '0' + pMsg->connHandle + 1;
    if (pMsg->data[2] == '0')
      Display_printf(dispHandle, TSN_ROW_CHARSTAT, 0, "Key 1: from Father %c", pMsg->data[1]);
    else 
      Display_printf(dispHandle, TSN_ROW_CHARSTAT, 0, "Key 1: from Father %c 's child %c", pMsg->data[1], pMsg->data[2]);
  }
}

/*********************************************************************
 * @fn      tree_structure_network_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (TSN_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the first characteristic in the Tree Network
 *          Service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void tree_structure_network_performPeriodicTask(void)
{  
  uint8_t state;
  uint8_t i = 0;
  uint8_t father_index = 0;
  static uint8_t string_index = 0;
  uint8_t valid_table[MAX_NUM_BLE_CONNS] ={0};
  uint8_t valid_table_sum = 0;

  // Loop all connected device
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle != LINKDB_CONNHANDLE_INVALID)
      valid_table[i] = 1;
    else
      valid_table[i] = 0;
  }
  
  // Get the total number
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
    valid_table_sum = valid_table_sum + valid_table[i];
  
  // Loop the charValue table, which includes the tree structure network instruction
  if (string_index >= string_index_max - 1)
    string_index = 0;
  else
    string_index ++;
  
  // Grandfather type node 
  // According the charVals table to loop the defined sub-node
  if(my_role == GAP_PROFILE_CENTRAL)
  {
      if (valid_table_sum != 0) //Connected number should > 0
      {
        // Change the ASCII to the number, get the fatehr index
        father_index = charVals[string_index][1] - '0' - 1;
        if (father_index <= valid_table_sum - 1)
        {
          state = tree_structure_network_doSelectConn(father_index);
          if (state == false)
            Display_printf(dispHandle, TSN_ROW_SEL_CONN, 0, "doSelectConn fail: %d", father_index);
          else
          {
            // Send the data to the father
            state = tree_structure_network_doGattWriteString(connList[father_index].connHandle, charVals[string_index]);
          }
        }
        else
        {
          Display_printf(dispHandle, TSN_ROW_SEL_CONN, 0, "doSelectConn fail: %d is out of list", father_index);
        }
      }
  }
}

/*********************************************************************
 * @fn      tree_structure_network_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void tree_structure_network_updateRPA(void)
{
  uint8_t* pRpaNew;

  // Read the current RPA.
  pRpaNew = GAP_GetDevAddress(FALSE);

  if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
  {
    // If the RPA has changed, update the display
    Display_printf(dispHandle, TSN_ROW_RPA, 0, "RP Addr: %s",
                   Util_convertBdAddr2Str(pRpaNew));
    memcpy(rpa, pRpaNew, B_ADDR_LEN);
  }
}

/*********************************************************************
 * @fn      tree_structure_network_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void tree_structure_network_clockHandler(UArg arg)
{
  tsnClockEventData_t *pData = (tsnClockEventData_t *)arg;

  if (pData->event == TSN_EVT_PERIODIC)
  {
    // Start the next period
    Util_startClock(&clkPeriodic);

    // Send message to perform periodic task
    tree_structure_network_enqueueMsg(TSN_EVT_PERIODIC, NULL);
  }
  else if (pData->event == TSN_EVT_READ_RPA)
  {
    // Start the next period
    Util_startClock(&clkRpaRead);

    // Send message to read the current RPA
    tree_structure_network_enqueueMsg(TSN_EVT_READ_RPA, NULL);
  }
  else if (pData->event == TSN_EVT_SEND_PARAM_UPDATE)
  {
    // Send message to app
    tree_structure_network_enqueueMsg(TSN_EVT_SEND_PARAM_UPDATE, pData);
  }
}

/*********************************************************************
* @fn      tree_structure_network_keyChangeHandler
*
* @brief   Key event handler function
*
* @param   a0 - ignored
*
* @return  none
*/
static void tree_structure_network_keyChangeHandler(uint8_t keys)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = keys;

    tree_structure_network_enqueueMsg(TSN_EVT_KEY_CHANGE, pValue);
  }
}

/*********************************************************************
* @fn      tree_structure_network_handleKeys
*
* @brief   Handles all key events for this device.
*
* @param   keys - bit field for key events. Valid entries:
*                 KEY_LEFT
*                 KEY_RIGHT
*
* @return  none
*/
static void tree_structure_network_handleKeys(uint8_t keys)
{
  uint32_t rtnVal = 0;
  if (keys & KEY_LEFT)
  {
    // Check if the key is still pressed
    if (PIN_getInputValue(Board_PIN_BUTTON0) == 0)
    {
      if (my_role == GAP_PROFILE_CENTRAL)
      {
        if (en_period_task == 0)
        {
          // Grandfather type node enable the period task to loop the sub-nodes
          en_period_task = 1;
          Util_startClock(&clkPeriodic);
        }
        else
        {
          en_period_task = 0;
          Util_stopClock(&clkPeriodic);
        }
      }
      else
      {
        // Father/child node: send the key value
        uint8_t valueToCopy[TREENETWORKSERVICE_CHAR1_LEN] = {'0', '0', '0', 'A', '\0'};
        TreeNetworkService_SetParameter(TREENETWORKSERVICE_CHAR1, TREENETWORKSERVICE_CHAR1_LEN, &valueToCopy);
      }
    }
  }
  else if (keys & KEY_RIGHT)
  {
    // Check if the key is still pressed
    rtnVal = PIN_getInputValue(Board_PIN_BUTTON1);
    if (rtnVal == 0)
    {
      tree_structure_network_doDiscoverDevices(0);
    }
  }
}

/*********************************************************************
* @fn      tree_structure_network_processGATTDiscEvent
*
* @brief   Process GATT discovery event
*
* @param   pMsg - pointer to discovery event stack message
*
* @return  none
*/
static void tree_structure_network_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  uint8_t connIndex = tree_structure_network_getConnIndex(pMsg->connHandle);
  TREESTRUCTURENETWORK_ASSERT(connIndex < MAX_NUM_BLE_CONNS);
  uint8_t event;
  
  if (connList[connIndex].discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover simple service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(TREENETWORKSERVICE_SERV_UUID),
                                         HI_UINT16(TREENETWORKSERVICE_SERV_UUID) };

      connList[connIndex].discState = BLE_DISC_STATE_SVC;

      // Discovery simple service
      VOID GATT_DiscPrimaryServiceByUUID(pMsg->connHandle, uuid,
                                         ATT_BT_UUID_SIZE, selfEntity);
    }
  }
  else if (connList[connIndex].discState == BLE_DISC_STATE_SVC)
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
        connList[connIndex].discState = BLE_DISC_STATE_CHAR;

        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(TREENETWORKSERVICE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(TREENETWORKSERVICE_CHAR1_UUID);

        VOID GATT_DiscCharsByUUID(pMsg->connHandle, &req, selfEntity);
      }
    }
  }
  else if (connList[connIndex].discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
      uint8_t connIndex = tree_structure_network_getConnIndex(tsnConnHandle);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      TREESTRUCTURENETWORK_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      // Store the handle of the simpleprofile characteristic 1 value
      connList[connIndex].charHandle
        = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[3],
                       pMsg->msg.readByTypeRsp.pDataList[4]);

      Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Simple Svc Found");
    }
    
    // Next procedure to enable char1 notification
    connList[connIndex].discState = BLE_DISC_STATE_ENABLE_CHAR_NOTIFICATION;
  }
  else if (connList[connIndex].discState == BLE_DISC_STATE_ENABLE_CHAR_NOTIFICATION)
  {
      event = TSN_EVT_EN_NOTI;
      if(tree_structure_network_enqueueMsg(event, 0) != SUCCESS)
      {
          ICall_free(pMsg);
      }
      Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Enable notification");
      
      // Next procedure to write char1
      connList[connIndex].discState = BLE_DISC_STATE_WRITE_CHAR;
  }
  else if (connList[connIndex].discState == BLE_DISC_STATE_WRITE_CHAR)
  {
     event = TSN_EVT_WRITE_CHAR;
     if(tree_structure_network_enqueueMsg(event, 0) != SUCCESS)
     {
         ICall_free(pMsg);
     }
     Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Send data");
     connList[connIndex].discState = BLE_DISC_STATE_IDLE;
  }
  
}

/*********************************************************************
* @fn      tree_structure_network_getConnIndex
*
* @brief   Translates connection handle to index
*
* @param   connHandle - the connection handle
*
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
*/
static uint16_t tree_structure_network_getConnIndex(uint16_t connHandle)
{
  uint8_t i;
  // Loop through connection
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    // If matching connection handle found
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  // Not found if we got here
  return(MAX_NUM_BLE_CONNS);
}

#ifndef Display_DISABLE_ALL
/*********************************************************************
 * @fn      tree_structure_network_getConnAddrStr
 *
 * @brief   Return, in string form, the address of the peer associated with
 *          the connHandle.
 *
 * @return  A null-terminated string of the address.
 *          if there is no match, NULL will be returned.
 */
static char* tree_structure_network_getConnAddrStr(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return Util_convertBdAddr2Str(connList[i].addr);
    }
  }

  return NULL;
}
#endif

/*********************************************************************
 * @fn      tree_structure_network_clearConnListEntry
 *
 * @brief   clear device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t tree_structure_network_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != LINKDB_CONNHANDLE_ALL)
  {
    connIndex = tree_structure_network_getConnIndex(connHandle);
    // Get connection index from handle
    if(connIndex >= MAX_NUM_BLE_CONNS)
    {
      return bleInvalidRange;
    }
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
    {
      connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
      connList[i].charHandle = 0;
      connList[i].discState  =  0;
    }
  }

  return SUCCESS;
}


/************************************************************************
* @fn      tree_structure_network_pairStateCB
*
* @param   connHandle - the connection handle
*
* @param   state - pairing state
*
* @param   status - status of pairing state
*
* @return  none
*/
static void tree_structure_network_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status)
{
  tsnPairStateData_t *pData = ICall_malloc(sizeof(tsnPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if (tree_structure_network_enqueueMsg(TSN_EVT_PAIRING_STATE, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
* @fn      tree_structure_network_passcodeCB
*
* @brief   Passcode callback.
*
* @param   deviceAddr - pointer to device address
*
* @param   connHandle - the connection handle
*
* @param   uiInputs - pairing User Interface Inputs
*
* @param   uiOutputs - pairing User Interface Outputs
*
* @param   numComparison - numeric Comparison 20 bits
*
* @return  none
*/
static void tree_structure_network_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs,
                                  uint32_t numComparison)
{
  tsnPasscodeData_t *pData = ICall_malloc(sizeof(tsnPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData)
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if (tree_structure_network_enqueueMsg(TSN_EVT_PASSCODE_NEEDED, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
* @fn      tree_structure_network_processPairState
*
* @brief   Process the new paring state.
*
* @param   pairingEvent - pairing event received from the stack
*
* @return  none
*/
static void tree_structure_network_processPairState(tsnPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
      Display_printf(dispHandle, TSN_ROW_SECURITY, 0, "Pairing started");
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {
        linkDBInfo_t linkInfo;

        Display_printf(dispHandle, TSN_ROW_SECURITY, 0, "Pairing success");

        if (linkDB_GetInfo(pPairData->connHandle, &linkInfo) == SUCCESS)
        {
          // If the peer was using private address, update with ID address
          if ((linkInfo.addrType == ADDRTYPE_PUBLIC_ID ||
               linkInfo.addrType == ADDRTYPE_RANDOM_ID) &&
              !Util_isBufSet(linkInfo.addrPriv, 0, B_ADDR_LEN))

          {
            // Update the address of the peer to the ID address
            Display_printf(dispHandle, TSN_ROW_NON_CONN, 0, "Addr updated: %s",
                           Util_convertBdAddr2Str(linkInfo.addr));

            // Update the connection list with the ID address
            uint8_t i = tree_structure_network_getConnIndex(pPairData->connHandle);

            TREESTRUCTURENETWORK_ASSERT(i < MAX_NUM_BLE_CONNS);
            memcpy(connList[i].addr, linkInfo.addr, B_ADDR_LEN);
          }
        }
      }
      else
      {
        Display_printf(dispHandle, TSN_ROW_SECURITY, 0, "Pairing fail: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, TSN_ROW_SECURITY, 0, "Encryption success");
      }
      else
      {
        Display_printf(dispHandle, TSN_ROW_SECURITY, 0, "Encryption failed: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, TSN_ROW_SECURITY, 0, "Bond save success");
      }
      else
      {
        Display_printf(dispHandle, TSN_ROW_SECURITY, 0, "Bond save failed: %d", status);
      }

      break;

    default:
      break;
  }
}

/*********************************************************************
* @fn      tree_structure_network_processPasscode
*
* @brief   Process the Passcode request.
*
* @return  none
*/
static void tree_structure_network_processPasscode(tsnPasscodeData_t *pData)
{
  // Display passcode to user
  if (pData->uiOutputs != 0)
  {
    Display_printf(dispHandle, TSN_ROW_SECURITY, 0, "Passcode: %d",
                   B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pData->connHandle, SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      tree_structure_network_startSvcDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void tree_structure_network_startSvcDiscovery()
{
  uint8_t connIndex = tree_structure_network_getConnIndex(tsnConnHandle);

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  TREESTRUCTURENETWORK_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = 0;

  connList[connIndex].discState = BLE_DISC_STATE_MTU;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = tsnMaxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(tsnConnHandle, &req, selfEntity);
}

/*********************************************************************
* @fn      tree_structure_network_addConnInfo
*
* @brief   add a new connection to the index-to-connHandle map
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static uint8_t tree_structure_network_addConnInfo(uint16_t connHandle, uint8_t *pAddr,
                                      uint8_t role)
{
  uint8_t i;
  tsnClockEventData_t *paramUpdateEventData;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;
      memcpy(connList[i].addr, pAddr, B_ADDR_LEN);
      numConn++;

      // If a peripheral, start the clock to send a connection parameter update
      if(role == GAP_PROFILE_PERIPHERAL)
      {
        // Allocate data to send through clock handler
        paramUpdateEventData = ICall_malloc(sizeof(tsnClockEventData_t) +
                                            sizeof(uint16_t));
        if(paramUpdateEventData)
        {
          // Set clock data
          paramUpdateEventData->event = TSN_EVT_SEND_PARAM_UPDATE;
          *((uint16_t *)paramUpdateEventData->data) = connHandle;

          // Create a clock object and start
          connList[i].pUpdateClock
            = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

          if (connList[i].pUpdateClock)
          {
            Util_constructClock(connList[i].pUpdateClock,
                                tree_structure_network_clockHandler,
                                TSN_SEND_PARAM_UPDATE_DELAY, 0, true,
                                (UArg) paramUpdateEventData);
          }
        }
        else
        {
          // Memory allocation failed
          TREESTRUCTURENETWORK_ASSERT(false);
        }
      }

      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      tree_structure_network_removeConnInfo
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t tree_structure_network_removeConnInfo(uint16_t connHandle)
{
  uint8_t connIndex = tree_structure_network_getConnIndex(connHandle);

  if(connIndex < MAX_NUM_BLE_CONNS)
  {
    Clock_Struct* pUpdateClock = connList[connIndex].pUpdateClock;

    if (pUpdateClock != NULL)
    {
      // Stop and destruct the RTOS clock if it's still alive
      if (Util_isActive(pUpdateClock))
      {
        Util_stopClock(pUpdateClock);
      }

      // Destruct the clock object
      Clock_destruct(pUpdateClock);
      // Free clock struct
      ICall_free(pUpdateClock);
    }
    // Clear Connection List Entry
    tree_structure_network_clearConnListEntry(connHandle);
    numConn--;
  }

  return connIndex;
}

/*********************************************************************
* @fn      tree_structure_network_doDiscoverDevices
*
* @brief   Respond to user input to start scanning
*
* @param   index - not used
*
* @return  TRUE since there is no callback to use this value
*/
bool tree_structure_network_doDiscoverDevices(uint8_t index)
{
  (void) index;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // The stack does not need to record advertising reports
  // since the application will filter them by Service UUID and save.

  // Reset number of scan results to 0 before starting scan
  numScanRes = 0;
  GapScan_enable(0, DEFAULT_SCAN_DURATION, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // Let the stack record the advertising reports as many as up to DEFAULT_MAX_SCAN_RES.
  GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID


  return (true);
}

/*********************************************************************
 * @fn      tree_structure_network_doStopDiscovering
 *
 * @brief   Stop on-going scanning
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool tree_structure_network_doStopDiscovering(uint8_t index)
{
  (void) index;

  GapScan_disable();

  return (true);
}

/*********************************************************************
 * @fn      tree_structure_network_doCancelConnecting
 *
 * @brief   Cancel on-going connection attempt
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool tree_structure_network_doCancelConnecting(uint8_t index)
{
  (void) index;

  GapInit_cancelConnect();

  return (true);
}

/*********************************************************************
* @fn      tree_structure_network_doConnect
*
* @brief   Respond to user input to form a connection
*
* @param   index - index as selected from the tsnMenuConnect
*
* @return  TRUE since there is no callback to use this value
*/
bool tree_structure_network_doConnect(uint8_t index)
{
  // Set the phy parameters
  GapInit_setPhyParam(INIT_PHY_1M, INIT_PHYPARAM_CONN_INT_MIN, 160);
  GapInit_setPhyParam(INIT_PHY_1M, INIT_PHYPARAM_CONN_INT_MAX, 160);
  GapInit_setPhyParam(INIT_PHY_1M, INIT_PHYPARAM_CONN_LATENCY, 5);
  GapInit_setPhyParam(INIT_PHY_1M, INIT_PHYPARAM_SUP_TIMEOUT, 1000);
  
  // Temporarily disable advertising
  GapAdv_disable(advHandle);

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  GapInit_connect(scanList[index].addrType & MASK_ADDRTYPE_ID,
                  scanList[index].addr, tsnInitPhy, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  GapScan_Evt_AdvRpt_t advRpt;

  GapScan_getAdvReport(index, &advRpt);

  GapInit_connect(advRpt.addrType & MASK_ADDRTYPE_ID,
                  advRpt.addr, tsnInitPhy, 0);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

  Display_printf(dispHandle, TSN_ROW_NON_CONN, 0, "Connecting...");

  return (true);
}

/*********************************************************************
 * @fn      tree_structure_network_doSelectConn
 *
 * @brief   Select a connection to communicate with
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool tree_structure_network_doSelectConn(uint8_t index)
{
  // index cannot be equal to or greater than MAX_NUM_BLE_CONNS
  TREESTRUCTURENETWORK_ASSERT(index < MAX_NUM_BLE_CONNS);

  tsnConnHandle  = connList[index].connHandle;

  if (connList[index].charHandle == 0)
  {
    // Initiate service discovery
    tree_structure_network_enqueueMsg(TSN_EVT_SVC_DISC, NULL);
  }

  // Clear non-connection-related message
  Display_clearLine(dispHandle, TSN_ROW_NON_CONN);

  return (true);
}

/*********************************************************************
 * @fn      tree_structure_network_doGattRead
 *
 * @brief   GATT Read
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool tree_structure_network_doGattRead(uint8_t index)
{
  attReadReq_t req;
  uint8_t connIndex = tree_structure_network_getConnIndex(tsnConnHandle);

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  TREESTRUCTURENETWORK_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  req.handle = connList[connIndex].charHandle;
  GATT_ReadCharValue(tsnConnHandle, &req, selfEntity);

  return (true);
}

/*********************************************************************
 * @fn      tree_structure_network_doGattWriteString
 *
 * @brief   GATT Write
 *
 * @param   internal_tsnConnHandle - Connection Handle to Write
 *          pAddr - GATT Writing String address
 *
 * @return  always true
 */
bool tree_structure_network_doGattWriteString(uint8_t internal_tsnConnHandle, uint8_t *pAddr)
{
  status_t status;

  attWriteReq_t req;

  req.pValue = GATT_bm_alloc(internal_tsnConnHandle, ATT_WRITE_REQ, TREENETWORKSERVICE_CHAR1_LEN, NULL);

  if ( req.pValue != NULL )
  {
    uint8_t connIndex = tree_structure_network_getConnIndex(internal_tsnConnHandle);

    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    TREESTRUCTURENETWORK_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    req.handle = connList[connIndex].charHandle;
    req.len = TREENETWORKSERVICE_CHAR1_LEN;
    memcpy(req.pValue, pAddr, TREENETWORKSERVICE_CHAR1_LEN);
    req.sig = 0;
    req.cmd = 0;

    if (pAddr[0] == 'L'); // LED Command for link establishment (no action)
    else if (pAddr[2] == '0')
    {
	  // LED command for father
      if (pAddr[3] == '0')
      {
        Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Father %c 's RLED off", pAddr[1], pAddr[3]);
      }
      else
      {
        Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Father %c 's RLED on", pAddr[1], pAddr[3]);
      }
    }
    else
    {
	  // LED command for child
      if (pAddr[3] == '0')
        Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Father %c 's child %c 's RLED off", pAddr[1], pAddr[2], pAddr[3]);
      else
        Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Father %c 's child %c 's RLED on", pAddr[1], pAddr[2], pAddr[3]);
    }
    
    status = GATT_WriteCharValue(internal_tsnConnHandle, &req, selfEntity);
    if ( status != SUCCESS )
    {
      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
  }

  return (true);
}

/*********************************************************************
 * @fn      tree_structure_network_doGattWriteEnableNotification
 *
 * @brief   GATT Writing to enable notification
 *
 * @param   Null
 *
 * @return  always true
 */
bool tree_structure_network_doGattWriteEnableNotification()
{
  status_t status;

  attWriteReq_t req;

  req.pValue = GATT_bm_alloc(tsnConnHandle, ATT_WRITE_REQ, 2, NULL);

  if ( req.pValue != NULL )
  {
    uint8_t connIndex = tree_structure_network_getConnIndex(tsnConnHandle);

    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    TREESTRUCTURENETWORK_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    req.handle = connList[connIndex].charHandle + 1;
    req.len = 2;
    req.pValue[0] = 0x01;
    req.pValue[1] = 0x00;
    req.sig = 0;
    req.cmd = 0;

    Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Enable notification\n");

    // Enable the notification function
    status = GATT_WriteCharValue(tsnConnHandle, &req, selfEntity);
    if ( status != SUCCESS )
    {
      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
  }

  return (true);
}

/*********************************************************************
* @fn      tree_structure_network_doConnUpdate
*
* @brief   Respond to user input to do a connection update
*
* @param   index - index as selected from the tsnMenuConnUpdate
*
* @return  TRUE since there is no callback to use this value
*/
bool tree_structure_network_doConnUpdate(uint8_t index)
{
  gapUpdateLinkParamReq_t params;

  (void) index; //may need to get the real connHandle?

  params.connectionHandle = tsnConnHandle;
  params.intervalMin = DEFAULT_UPDATE_MIN_CONN_INTERVAL;
  params.intervalMax = DEFAULT_UPDATE_MAX_CONN_INTERVAL;
  params.connLatency = DEFAULT_UPDATE_SLAVE_LATENCY;

  linkDBInfo_t linkInfo;
  if (linkDB_GetInfo(tsnConnHandle, &linkInfo) == SUCCESS)
  {
    if (linkInfo.connTimeout == DEFAULT_UPDATE_CONN_TIMEOUT)
    {
      params.connTimeout = DEFAULT_UPDATE_CONN_TIMEOUT + 200;
    }
    else
    {
      params.connTimeout = DEFAULT_UPDATE_CONN_TIMEOUT;
    }
  }
  else
  {
    Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0,
                   "update :%s, Unable to find link information",
                   Util_convertBdAddr2Str(linkInfo.addr));
  }
  GAP_UpdateLinkParamReq(&params);

  Display_printf(dispHandle, TSN_ROW_CUR_CONN, 0, "Param update Request:connTimeout =%d",
                 params.connTimeout*CONN_TIMEOUT_MS_CONVERSION);

  return (true);
}

/*********************************************************************
* @fn      multi_role_doDisconnect
*
* @brief   Respond to user input to terminate a connection
*
* @param   index - index as selected from the tsnMenuConnUpdate
*
* @return  always true
*/
bool tree_structure_network_doDisconnect(uint8_t index)
{
  (void) index;

  // Disconnect
  GAP_TerminateLinkReq(tsnConnHandle, HCI_DISCONNECT_REMOTE_USER_TERM);

  return (true);
}

/*********************************************************************
* @fn      tree_structure_network_doAdvertise
*
* @brief   Respond to user input to terminate a connection
*
* @param   index - index as selected from the tsnMenuConnUpdate
*
* @return  always true
*/
bool tree_structure_network_doAdvertise(uint8_t index)
{
  (void) index;

  // If we're currently advertising
  if (tsnIsAdvertising)
  {
    // Turn off advertising
    GapAdv_disable(advHandle);
  }
  // If we're not currently advertising
  else
  {
    if (numConn < MAX_NUM_BLE_CONNS)
    {
      // Start advertising since there is room for more connections
      GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
    }
    else
    {
      Display_printf(dispHandle, TSN_ROW_ADVERTIS, 0,
                     "At Maximum Connection Limit, Cannot Enable Advertisment");
    }
  }

  return (true);
}

/*********************************************************************
*********************************************************************/
