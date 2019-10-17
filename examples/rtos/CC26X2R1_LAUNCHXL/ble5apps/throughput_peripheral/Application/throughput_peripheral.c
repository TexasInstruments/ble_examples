/******************************************************************************

 @file  throughput_peripheral.c

 @brief This file contains the Throughput Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: CMCU, LPRF
 Target Device: CC2652

 ******************************************************************************
 
 Copyright (c) 2013-2018, Texas Instruments Incorporated
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
#include <ti/sysbios/hal/Hwi.h>

#include <ti/display/Display.h>

#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include <profiles/ccservice.h>
#include <profiles/throughput_service.h>

#include <board.h>
#include <board_key.h>

#include <menu/two_btn_menu.h>

#include "throughput_peripheral_menu.h"
#include "throughput_peripheral.h"

/*********************************************************************
 * CONSTANTS
 */

// Address mode of the local device
#define DEFAULT_ADDRESS_MODE                 ADDRMODE_PUBLIC

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE            GAP_ADTYPE_FLAGS_GENERAL

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL         160

// Minimum connection interval for parameter update request
#define LAUNCHPAD_DEFAULT_DESIRED_MIN_CONN_INTERVAL      160    // 200ms
#define IPHONE_DEFAULT_DESIRED_MIN_CONN_INTERVAL         12     // 15ms
#define ANDROID_DEFAULT_DESIRED_MIN_CONN_INTERVAL        6      // 7.5ms

// Maximum connection interval for parameter update request
#define LAUNCHPAD_DEFAULT_DESIRED_MAX_CONN_INTERVAL      160    // 200ms
#define IPHONE_DEFAULT_DESIRED_MAX_CONN_INTERVAL         12     // 15ms
#define ANDROID_DEFAULT_DESIRED_MAX_CONN_INTERVAL        6      // 7.5ms

// Slave latency to use for parameter update request
#define LAUNCHPAD_DEFAULT_DESIRED_SLAVE_LATENCY           0
#define IPHONE_DEFAULT_DESIRED_SLAVE_LATENCY              0
#define ANDROID_DEFAULT_DESIRED_SLAVE_LATENCY             0

// Supervision timeout value for parameter update request
#define LAUNCHPAD_DEFAULT_DESIRED_CONN_TIMEOUT          300     // 3000ms
#define IPHONE_DEFAULT_DESIRED_CONN_TIMEOUT             300     // 3000ms
#define ANDROID_DEFAULT_DESIRED_CONN_TIMEOUT            300     // 3000ms

// Connection parameter request limits
#define ANDROID_MINIMUM_CONNINT                         6       // 7.5ms
#define IPHONE_MINIMUM_CONNINT                          12      // 15ms
#define IPHONE_MINIMUM_CONNTIMEOUT                      1600    // 2000ms
#define IPHONE_MAXIMUM_CONNTIMEOUT                      4800    // 6000ms

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION    GAP_UPDATE_REQ_PASS_TO_APP

// Default initial connection intervals for peers. Used for peer identification
#define DEFAULT_LAUNCHPAD_INITIAL_CONN_INT   INIT_PHYPARAM_DFLT_CONN_INT_MAX
#define DEFAULT_IPHONE_INITIAL_CONN_INT      24
#define DEFAULT_ANDROID_INITIAL_CONN_INT     6

// How often to read current current RPA (in ms)
#define TP_READ_RPA_EVT_PERIOD               3000

// Delay (in ms) after connection establishment before sending a parameter update request
#define TP_SEND_PARAM_UPDATE_DELAY           500

// Task configuration
#define TP_TASK_PRIORITY                     1

#ifndef TP_TASK_STACK_SIZE
#define TP_TASK_STACK_SIZE                   1024
#endif

// Application events
#define TP_TIMEOUT_EVT                       0
#define TP_CHAR_CHANGE_EVT                   1
#define TP_KEY_CHANGE_EVT                    2
#define TP_ADV_EVT                           3
#define TP_PAIR_STATE_EVT                    4
#define TP_PASSCODE_EVT                      5
#define TP_READ_RPA_EVT                      6
#define TP_SEND_PARAM_UPDATE_EVT             7
#define TP_CONN_EVT                          8


// Internal Events for RTOS application
#define TP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define TP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define TP_THROUGHPUT_EVT                    Event_Id_00
#define TP_PDU_CHANGE_EVT                    Event_Id_01
#define TP_PHY_CHANGE_EVT                    Event_Id_02
#define CCSERVICE_UPDATE_EVT                 Event_Id_03
#define CCSERVICE_KILL_CON_EVT               Event_Id_04

// Bitwise OR of all RTOS events to pend on
#define TP_ALL_EVENTS                        (TP_ICALL_EVT             | \
                                              TP_QUEUE_EVT             | \
                                              TP_THROUGHPUT_EVT        | \
                                              TP_PDU_CHANGE_EVT        | \
                                              TP_PHY_CHANGE_EVT        | \
                                              CCSERVICE_UPDATE_EVT     | \
											  CCSERVICE_KILL_CON_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define TP_ADDR_STR_SIZE     15

// Row numbers for two-button menu
#define TP_ROW_SEPARATOR_1      (TBM_ROW_APP + 0)
#define TP_ROW_TOGGLE           (TBM_ROW_APP + 1)
#define TP_ROW_STATUS_1         (TBM_ROW_APP + 2)
#define TP_ROW_STATUS_2         (TBM_ROW_APP + 3)
#define TP_ROW_CONNECTION       (TBM_ROW_APP + 4)
#define TP_ROW_ADVSTATE         (TBM_ROW_APP + 5)
#define TP_ROW_IDA              (TBM_ROW_APP + 6)
#define TP_ROW_RPA              (TBM_ROW_APP + 7)
#define TP_ROW_STATUS_CONNINT   (TBM_ROW_APP + 8)

// Default values for connection PHY and PDU size
#define DEFAULT_PHY         TS_PHY_1M
#define DEFAULT_PDU         27

// Default TX-time values used for DLE
#define DEFAULT_TX_TIME     328
#define DEFAULT_MAX_TX_TIME 2120
#define CODED_MAX_TX_TIME   17040

// The combined overhead for L2CAP and ATT notification headers
#define TOTAL_PACKET_OVERHEAD (L2CAP_HDR_SIZE + ATT_HANDLE_VALUE_IND_HDR_SIZE)

// The different peerTypes used for the connection info
#define peerType_NOTIDENTIFIED       0x00
#define peerType_LAUNCHPAD           0x01
#define peerType_IPHONE              0x02
#define peerType_ANDROID             0x04
#define peerType_CODEDPHY            0x08
// Spin if the expression is not true
#define THROUGHPUTPERIPHERAL_ASSERT(expr) if (!(expr)) Throughput_Peripheral_Spin();

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;             // event type
  void    *pData;            // pointer to message
} tpEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} tpPairStateData_t;

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
} tpPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} tpGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;
  uint8_t data[];
} tpClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
  List_Elem elem;
  uint16_t  connHandle;
} tpConnHandleEntry_t;

// Connected device information
typedef struct
{
  uint16_t              connHandle;                 // Connection Handle
  tpClockEventData_t*   pParamUpdateEventData;      // Pointer to the parameter request data
  Clock_Struct*         pUpdateClock;               // Pointer to parameter update clock
  uint16_t              connTimeout;                // Connection Timeout parameter
  bool                  establishingConn;           // If the connection is being established or not
  bool                  initialParamsSet;           // Whether the initial connection parameters have been set
  uint32                oldMsgCounter;              // Previous MsgCounter-value.
  uint8_t               oldPHY;                     // Previous PHY-setting. Used to ensure correct info in App
  uint8                 requestedPHY;               // The requested PHY. Used to ensure correct info in App
  uint8_t               oldPDU;                     // Previous PDU-value. Used to ensure correct info in App
  uint8_t               peerType;                   // What type of device the peer is and what features it has
  bool                  restartThroughput;          // Whether to restart throughput demo after parameter change
} tpConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

// Task configuration (Only Valid for CCS, if using IAR this code will not work.)
Task_Struct tpTask;
#pragma DATA_ALIGN(tpTaskStack, 8)
uint8_t tpTaskStack[TP_TASK_STACK_SIZE];

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;
// Clock instance to start throughput after connection interval has been exchanged
static Clock_Struct clkStartThroughput;
// Clock instance for connection timeout
static Clock_Struct clkTimeout;

// Memory to pass RPA read event ID to clock handler
tpClockEventData_t argRpaRead =
{ .event = TP_READ_RPA_EVT };

// Memory to pass Timeout event ID
tpClockEventData_t argTimeout =
{ .event = TP_TIMEOUT_EVT };

// Connection info
static tpConnRec_t connInfo;

// Current connection handle as chosen by menu
static uint16_t menuConnHandle = LINKDB_CONNHANDLE_INVALID;

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Throughput Periph";

// Advertisement data
static uint8_t advertData[] =
{
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(THROUGHPUT_SERVICE_SERV_UUID),
  HI_UINT16(THROUGHPUT_SERVICE_SERV_UUID)
};

// Scan Response Data
static uint8_t scanRspData[] =
{
  // complete name
  0x12,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'T',
  'h',
  'r',
  'o',
  'u',
  'g',
  'h',
  'p',
  'u',
  't',
  ' ',
  'P',
  'e',
  'r',
  'i',
  'p',
  'h',

  // connection interval range
  5,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(LAUNCHPAD_DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 200ms
  HI_UINT16(LAUNCHPAD_DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(LAUNCHPAD_DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(LAUNCHPAD_DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  2,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// Advertising handles
static uint8 advHandleLegacy;
static uint8 advHandleLongRange;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

// Flag to Toggle Throughput Demo
static bool throughputOn = false;

// MTU of the current connection
static uint16_t currentMTU = MAX_PDU_SIZE;

// Message counter for Throughput Demo
static uint32 msg_counter = 0;

// PHY Options
static uint16_t phyOptions = LL_PHY_OPT_NONE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void ThroughputPeripheral_init( void );
static void ThroughputPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t ThroughputPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t ThroughputPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void ThroughputPeripheral_processGapMessage(gapEventHdr_t *pMsg);
static void ThroughputPeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void ThroughputPeripheral_processAdvEvent(tpGapAdvEventData_t *pEventData);
static void ThroughputPeripheral_processAppMsg(tpEvt_t *pMsg);
static void ThroughputPeripheral_processCharValueChangeEvt(uint8_t paramId);
static void ThroughputPeripheral_updateRPA(void);
static void ThroughputPeripheral_clockHandler(UArg arg);
static void ThroughputPeripheral_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);
static void ThroughputPeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void ThroughputPeripheral_processPairState(tpPairStateData_t *pPairState);
static void ThroughputPeripheral_processPasscode(tpPasscodeData_t *pPasscodeData);
static void ThroughputPeripheral_charValueChangeCB(uint8_t paramID);
static status_t ThroughputPeripheral_enqueueMsg(uint8_t event, void *pData);
static void ThroughputPeripheral_keyChangeHandler(uint8 keys);
static void ThroughputPeripheral_handleKeys(uint8_t keys);
static void ThroughputPeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);

static void ThroughputPeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
static void ThroughputPeripheral_addConn(uint16_t connHandle);

static void ThroughputPeripheral_removeConn(void);
static void ThroughputPeripheral_processParamUpdate(uint16_t connHandle);
static status_t ThroughputPeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts);
static void ThroughputPeripheral_resetConnInfo(void);
static void ThroughputPeripheral_menuSwitchCb(tbmMenuObj_t* pMenuObjCurr,
                                          tbmMenuObj_t* pMenuObjNext);

static void ThroughputPeripheral_throughputOn(void);
static void ThroughputPeripheral_throughputOff(void);
static void ThroughputPeripheral_sendData(uint16_t connHandle);

static void ThroughputPeripheral_ConnectionControl_ValueChangeCB(uint8_t paramID);
static void ThroughputPeripheral_ConnectionControl_updateChar(uint16_t connInterval,
                                                              uint16_t connSlaveLatency, uint16_t connTimeout);
static void ThroughputPeripheral_ConnectionControl_processCharChangeEvt(uint8_t paramID);

static void ThroughputPeripheral_ConnectionControlChangeCB(uint8_t paramID);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t ThroughputPeripheral_BondMgrCBs =
{
  ThroughputPeripheral_passcodeCb,              // Passcode callback
  ThroughputPeripheral_pairStateCb             // Pairing/Bonding state Callback
};

// Throughput GATT Profile Callbacks
static Throughput_ServiceCBs_t ThroughputPeripheral_throughputProfileCBs =
{
  ThroughputPeripheral_charValueChangeCB // Characteristic value change callback
};
// Connection Control Profile Callbacks
static ccCBs_t ThroughputPeripheralTag_ccCBs =
{
 ThroughputPeripheral_ConnectionControlChangeCB,               // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Throughput_Peripheral_Spin
 *
 * @brief   Spin forever if a THROUGHPUTPERIPHERAL_ASSERT fails
 *
 * @param   none
 *
 * @return  none
 */
static void Throughput_Peripheral_Spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      ThroughputPeripheral_createTask
 *
 * @brief   Task creation function for the Throughput Peripheral.
 *
 * @param   none
 *
 * @return  none
 */
void ThroughputPeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = tpTaskStack;
  taskParams.stackSize = TP_TASK_STACK_SIZE;
  taskParams.priority = TP_TASK_PRIORITY;

  Task_construct(&tpTask, ThroughputPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      ThroughputPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   none
 *
 * @return  none
 */
static void ThroughputPeripheral_init(void)
{

  void Throughput_buildMenu(void); 
  
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the BLE-Stack 
  // User's Guide.
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide.
  {
    // Don't send a pairing request after connecting; the peer device must
    // initiate pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    // Use authenticated pairing: require passcode.
    uint8_t mitm = FALSE;
    // This device only has display capabilities. Therefore, it will display the
    // passcode during pairing. However, since the default passcode is being
    // used, there is no need to display anything.
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    // Request bonding (storing long-term keys for re-encryption upon subsequent
    // connections without repairing)
    uint8_t bonding = FALSE;
    // Usen non-secure mode
    uint8_t gapbondSecure = GAPBOND_SECURE_CONNECTION_NONE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &gapbondSecure);
  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           	 // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   	 // GATT Service
  DevInfo_AddService();                       	 // Device Information Service
  CcService_addService();                        // Connection Control Service

  // Setup the ThroughputProfile Characteristic Values
  // For more information, see the GATT and GATTServApp sections in the BLE-Stack 
  // User's Guide.
  {
    // Set Initial Values of Characteristics in GATT Table
    uint8_t default_pdu = DEFAULT_PDU;
    uint8_t default_phy = DEFAULT_PHY;

    Throughput_Service_SetParameter(THROUGHPUT_SERVICE_UPDATE_PDU, sizeof(uint8_t),
                               &default_pdu);
    Throughput_Service_SetParameter(THROUGHPUT_SERVICE_UPDATE_PHY, sizeof(uint8_t),
                               &default_phy);

  }

  // Initialize the GATT attributes
  Throughput_Service_AddService();      // Throughput Service

  // Register callbacks with Throughput Profile
  Throughput_Service_RegisterAppCBs(&ThroughputPeripheral_throughputProfileCBs);

  // Register callbacks with Connection Control Profile
  CcService_registerAppCBs(&ThroughputPeripheralTag_ccCBs);

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&ThroughputPeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the BLE-Stack User's 
  // Guide.
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Initialize GATT Client
  GATT_InitClient();

  // Init key debouncer
  Board_initKeys(ThroughputPeripheral_keyChangeHandler);

  // By Default Allow Central to support any and all PHYs
  HCI_LE_SetDefaultPhyCmd(LL_PHY_USE_ANY_PHY, LL_PHY_1_MBPS | LL_PHY_2_MBPS| HCI_PHY_CODED, LL_PHY_1_MBPS | LL_PHY_2_MBPS| HCI_PHY_CODED);

  // Set the Transmit Power of the Device to +5dBm
  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);

  // Set the RX Gain to be highest
  HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);

  // Initialize Connection Info
  ThroughputPeripheral_resetConnInfo();

  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, NULL);

  // The type of display is configured based on the BOARD_DISPLAY_USE...
  // preprocessor definitions
  dispHandle = Display_open(Display_Type_ANY, NULL);

  // Set the title of the main menu
  TBM_SET_TITLE(&tpMenuMain, "Texas Instruments Bluetooth 5 Throughput Peripheral");
  tbm_setItemStatus(&tpMenuMain, TBM_ITEM_NONE, TBM_ITEM_ALL);

  tbm_initTwoBtnMenu(dispHandle, &tpMenuMain, 2, ThroughputPeripheral_menuSwitchCb);
  Display_printf(dispHandle, TP_ROW_SEPARATOR_1, 0, "====================");
}

/*********************************************************************
 * @fn      ThroughputPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Throughput Peripheral
 *
 * @param   a0 - not used
 * @param   a1 - not used
 *
 * @return  none
 */
static void ThroughputPeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  ThroughputPeripheral_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, TP_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;
      uint16_t connHandle = 0;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = ThroughputPeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
          else
          {
              // Get connection handle
              connHandle = connInfo.connHandle;
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & TP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          tpEvt_t *pMsg = (tpEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            ThroughputPeripheral_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
      if (events & TP_THROUGHPUT_EVT)
      {
        // Set that the initial connection event is done
        connInfo.establishingConn = FALSE;

        // Get connection handle from index
        connHandle = connInfo.connHandle;

        // Begin Throughput Demo in App Task
        ThroughputPeripheral_sendData(connHandle);
      }

      if (events & TP_PHY_CHANGE_EVT)
      {
        // Variables needed
        uint8_t newPHY = 0;

        // Get Prereq data.
        Throughput_Service_GetParameter(THROUGHPUT_SERVICE_UPDATE_PHY, &newPHY);

        // Which PHY is picked?
        static uint8_t phy[] = {
          HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_CODED, HCI_PHY_CODED
        };

        // Swtich to determine PHY options (needed for coded S2 and S8 mode)
        switch(newPHY)
        {
        case 0:
        case 1:
          phyOptions = LL_PHY_OPT_NONE;
          break;
        case 2:
          phyOptions = LL_PHY_OPT_S2;
          break;
        case 3:
          phyOptions = LL_PHY_OPT_S8;
          break;
        }
        // Store the requested PHY to verify that the PHY actually has changed
        connInfo.requestedPHY = phy[newPHY];
        // Set this device's Phy Preference on the current connection.
        HCI_LE_SetPhyCmd(connHandle, LL_PHY_USE_PHY_PARAM, phy[newPHY], phy[newPHY], phyOptions);


      }

      if (events & TP_PDU_CHANGE_EVT)
      {
        // Variables needed
        uint16_t newPDU = 0;
        uint16_t txTime = DEFAULT_TX_TIME;

        // Get Prereq data.
        Throughput_Service_GetParameter(THROUGHPUT_SERVICE_UPDATE_PDU, &newPDU);

        // Decide which TxTime to use
        if(newPDU > DEFAULT_PDU)
        {
            if(connInfo.peerType & peerType_CODEDPHY)
            {
                txTime = CODED_MAX_TX_TIME;
            }
            else
            {
                txTime = DEFAULT_MAX_TX_TIME;
            }
        }
        // DLE HCI command to adjust PDU size
        HCI_LE_SetDataLenCmd(connHandle, newPDU, txTime);
	  }
      if (events & CCSERVICE_UPDATE_EVT) 
	  {
          Display_clearLine(dispHandle, TP_ROW_STATUS_CONNINT);
          // Send a parameter request using the new connection parameters of the connection parameter characteristic
          ThroughputPeripheral_ConnectionControl_processCharChangeEvt(CCSERVICE_CHAR2);

      }
      if (events & CCSERVICE_KILL_CON_EVT) 
	  {
          // Terminate the current connection
          ThroughputPeripheral_ConnectionControl_processCharChangeEvt(CCSERVICE_CHAR3);
      }
    }
  }
}

/*********************************************************************
 * @fn      ThroughputPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ThroughputPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      ThroughputPeripheral_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = ThroughputPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        {
          ThroughputPeripheral_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;
        }

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
              if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
              {
                Display_printf(dispHandle, TP_ROW_STATUS_1, 0,
                        "PHY Change failure, peer does not support this");
              }
              else
              {
                if(BOARD_DISPLAY_USE_LCD == 0)
                {
                  Display_printf(dispHandle, TP_ROW_STATUS_1, 0,
                                 "PHY Update Status Event: 0x%x",
                                 pMyMsg->cmdStatus);
                }
              }

              ThroughputPeripheral_updatePHYStat(HCI_LE_SET_PHY, (uint8_t *)pMsg);
              break;
            }
            default:
              break;
          }
          break;
        }

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC =
            (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

          bool ExtendingTxTime = FALSE;

          // A Phy Update Has Completed or Failed
          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
          {
            if (pPUC->status != SUCCESS)
            {
              if (pPUC->status == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
              {
                Display_printf(dispHandle, TP_ROW_STATUS_1, 0,
                      "PHY Change failure, peer does not support this");
              }
              else
              {
                Display_printf(dispHandle, TP_ROW_STATUS_1, 0,
                      "PHY Change failure, error %d", pPUC->status);
              }
              // Revert the value of the characteristic to before the attempted PHY change
              Throughput_Service_SetParameter(THROUGHPUT_SERVICE_UPDATE_PHY, sizeof(uint8_t), &connInfo.oldPHY);
            }
            else
            {
              // Only symmetrical PHY is supported.
              // rxPhy should be equal to txPhy.
              if(BOARD_DISPLAY_USE_LCD == 1)
              {
                Display_printf(dispHandle, TP_ROW_STATUS_2, 0,
                              "PHY: %s",
                              (pPUC->rxPhy == HCI_PHY_USE_PHY_PARAM) ? "1M" :  //Connection is established on 1M, so most likely 1M.
                              (pPUC->rxPhy == HCI_PHY_1_MBPS) ? "1M" :
                              (pPUC->rxPhy == HCI_PHY_2_MBPS) ? "2M" :
                              "CODED");
              }
              else
              {
                Display_printf(dispHandle, TP_ROW_STATUS_2, 0,
                              "PHY Updated to %s",
                              (pPUC->rxPhy == HCI_PHY_USE_PHY_PARAM) ? "1M" :  //Connection is established on 1M, so most likely 1M.
                              (pPUC->rxPhy == HCI_PHY_1_MBPS) ? "1M" :
                              (pPUC->rxPhy == HCI_PHY_2_MBPS) ? "2M" :
                              "CODED");
              }
              // Check if we actually updated the PHY
              if(connInfo.requestedPHY == pPUC->rxPhy)
              {
                // Update the stored previous PHY value for the current connection
                Throughput_Service_GetParameter(THROUGHPUT_SERVICE_UPDATE_PHY, &connInfo.oldPHY);

                // Update PDU to use extended txTime if this is the initial update to CODED
                if((pPUC->rxPhy == HCI_PHY_CODED) && !(connInfo.peerType & peerType_CODEDPHY))
                {
                  connInfo.peerType |= peerType_CODEDPHY;

                  Event_post(syncEvent, TP_PDU_CHANGE_EVT);
                  // Don't start throughput, let the Peripheral change TxTime first
                  ExtendingTxTime = TRUE;
                }
              }
              else if(connInfo.establishingConn)
              {
                // Devices which don't support data transfer on 1M PHY switch to 2M PHY upon connecting
                // so we have to update the characteristic to display the new PHY
                uint8_t initialPHY = pPUC->rxPhy == HCI_PHY_1_MBPS ? TS_PHY_1M :
                                     pPUC->rxPhy == HCI_PHY_2_MBPS ? TS_PHY_2M :
                                             TS_PHY_CODED_S2;

                Throughput_Service_SetParameter(THROUGHPUT_SERVICE_UPDATE_PHY, sizeof(uint8_t),
                                                               &initialPHY);
              }
              else
              {
                Throughput_Service_SetParameter(THROUGHPUT_SERVICE_UPDATE_PHY, sizeof(uint8_t),
                                             &connInfo.oldPHY);
              }
            }
            ThroughputPeripheral_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT, (uint8_t *)pMsg);
          }

          if (pPUC->BLEEventCode == HCI_BLE_DATA_LENGTH_CHANGE_EVENT)
          {
            // TX PDU Size Updated
            hciEvt_BLEDataLengthChange_t *dleEvt = (hciEvt_BLEDataLengthChange_t *)pMsg;
            if(BOARD_DISPLAY_USE_LCD == 1)
            {
              Display_print1(dispHandle, TP_ROW_STATUS_2 + 1, 0, "PDU: %dB", dleEvt->maxTxOctets);
            }
            else
            {
              Display_print1(dispHandle, TP_ROW_STATUS_2, 0, "PDU Size updated to: %dB", dleEvt->maxTxOctets);
            }
            connInfo.oldPDU = dleEvt->maxRxOctets;
          }
          // Restart throughput as it was turned off to change PHY and PDU size
          if ((pPUC->BLEEventCode == HCI_BLE_DATA_LENGTH_CHANGE_EVENT   ||
               pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT) &&
               !ExtendingTxTime && connInfo.restartThroughput)
          {
              ThroughputPeripheral_throughputOn();
              connInfo.restartThroughput = FALSE;
          }
          break;
        }

        default:
          break;
      }
      break;
    }
    default:
      // do nothing
      break;
  }
  return (safeToDealloc);
}

/*********************************************************************
 * @fn      ThroughputPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @param   *pMsg - pointer to the GATT event message
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ThroughputPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_printf(dispHandle, TP_ROW_STATUS_1, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    if(BOARD_DISPLAY_USE_LCD == 1)
    {
      Display_printf(dispHandle, TP_ROW_STATUS_1, 0, "MTU: %d", pMsg->msg.mtuEvt.MTU);
    }
    else
    {
      Display_printf(dispHandle, TP_ROW_STATUS_1, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }
    currentMTU = pMsg->msg.mtuEvt.MTU;

  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      ThroughputPeripheral_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void ThroughputPeripheral_processGapMessage(gapEventHdr_t *pMsg)
{
  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      bStatus_t status = FAILURE;

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

        Display_printf(dispHandle, TP_ROW_STATUS_1, 0, "Initialized");

        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide.

        // Temporary memory for advertising parameters for set #1. These will be copied
        // by the GapAdv module
        GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&ThroughputPeripheral_advCallback, &advParamLegacy,
                               &advHandleLegacy);
        THROUGHPUTPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advertData), advertData);
        THROUGHPUTPERIPHERAL_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanRspData), scanRspData);
        THROUGHPUTPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        THROUGHPUTPERIPHERAL_ASSERT(status == SUCCESS);

        // Use long range params to create long range set #2
        GapAdv_params_t advParamLongRange = GAPADV_PARAMS_AE_LONG_RANGE_CONN;

        // Create Advertisement set #2 and assign handle
        status = GapAdv_create(&ThroughputPeripheral_advCallback, &advParamLongRange,
                               &advHandleLongRange);
        THROUGHPUTPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #2 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLongRange, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advertData), advertData);
        THROUGHPUTPERIPHERAL_ASSERT(status == SUCCESS);

        // Load scan response data for set #2 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLongRange, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanRspData), scanRspData);
        THROUGHPUTPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #2
        status = GapAdv_setEventMask(advHandleLongRange,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable long range advertising for set #2
        status = GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        THROUGHPUTPERIPHERAL_ASSERT(status == SUCCESS);

        // Display device address
        if(BOARD_DISPLAY_USE_LCD == 1)
        {
          Display_printf(dispHandle, TP_ROW_IDA, 0, "%s",
                        Util_convertBdAddr2Str(pPkt->devAddr));
        }
        else
        {
          Display_printf(dispHandle, TP_ROW_IDA, 0, "%s Addr: %s",
                        (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
                        Util_convertBdAddr2Str(pPkt->devAddr));
        }
        if (addrMode > ADDRMODE_RANDOM)
        {
          ThroughputPeripheral_updateRPA();

          // Create one-shot clock for RPA check event.
          Util_constructClock(&clkRpaRead, ThroughputPeripheral_clockHandler,
                              TP_READ_RPA_EVT_PERIOD, 0, true,
                              (UArg) &argRpaRead);
        }
      }

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, TP_ROW_STATUS_2, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      if (pPkt->hdr.status == SUCCESS)
      {
        // Add connection info
        ThroughputPeripheral_addConn(pPkt->connectionHandle);
        // Calculate connection interval to ms * 100 for display purposes
        uint32_t connInt = (pPkt->connInterval) * 125;
        // Display connection interval
        Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Initial connection interval: %d.%d", connInt / 100, connInt % 100);

        // Discover what type of device the peer is based on the initial connection interval:
        if (connInfo.peerType == peerType_NOTIDENTIFIED)
        {
            switch(pPkt->connInterval)
            {
            case DEFAULT_LAUNCHPAD_INITIAL_CONN_INT:
                connInfo.peerType |= peerType_LAUNCHPAD;
                break;
            case DEFAULT_IPHONE_INITIAL_CONN_INT:
                connInfo.peerType |= peerType_IPHONE;
                break;
            default:
                connInfo.peerType |= peerType_ANDROID;
                break;
            }
        }
        connInfo.connTimeout = pPkt->connTimeout;

        /* Enable connection selection option if the peer is a phone. For the
        * Launchpad, we disable it until the Central writes to the characteristic
        * values of Throughput Service, signaling that we have chosen to "work with"
        * the Peripheral
        */
        if(!(connInfo.peerType & peerType_LAUNCHPAD))
        {
          tbm_setItemStatus(&tpMenuMain, TP_ITEM_SELECT_CONN, TBM_ITEM_NONE);
        }
        // Display the address of this connection
        Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Connected: %s, %s",
                       Util_convertBdAddr2Str(pPkt->devAddr),
                       (connInfo.peerType & peerType_LAUNCHPAD) ? "a Launchpad" :
                       (connInfo.peerType & peerType_IPHONE)    ? "an iPhone"   :
                                                                  "an Android");

      }
      // Stop advertising since there is no room for more connections
      GapAdv_disable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      GapAdv_disable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

      ThroughputPeripheral_ConnectionControl_updateChar(pPkt->connInterval, pPkt->connLatency, pPkt->connTimeout);
	  
      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Device Disconnected!");
      Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      // Remove the connection info
      ThroughputPeripheral_removeConn();

      // If no active connections
      if (numActive == 0)
      {
        // Disable Connection Selection option
        tbm_setItemStatus(&tpMenuMain, TBM_ITEM_NONE, TP_ITEM_SELECT_CONN);
      }

      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

      // Clear remaining lines
      Display_clearLines(dispHandle, TP_ROW_STATUS_1, TP_ROW_STATUS_2);
      ThroughputPeripheral_throughputOff();
      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;

      gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;

      rsp.connectionHandle = pReq->req.connectionHandle;

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
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);
      if(pPkt->status == SUCCESS)
      {
        // Multiply interval by 125 to get in ms, increased by 100
        uint32_t connInt = (linkInfo.connInterval) * 125;

        if(BOARD_DISPLAY_USE_LCD == 1)
        {
          Display_clearLine(dispHandle, TP_ROW_STATUS_2);
          Display_printf(dispHandle, TP_ROW_CONNECTION + 1, 0, "ConnInt: %d.%dms", connInt / 100, connInt % 100);
        }
        else
        {
          // Display the address of the connection update
          Display_printf(dispHandle, TP_ROW_STATUS_2, 0, "Link Param Updated: %s",
                         Util_convertBdAddr2Str(linkInfo.addr));
          Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Current connection interval: %d.%dms", connInt / 100, connInt % 100);
        }
        // If we don't get the 7.5ms connection interval on Android, send one more parameter request
        if((connInfo.peerType & peerType_ANDROID) && (linkInfo.connInterval != DEFAULT_ANDROID_INITIAL_CONN_INT)
                                                    && (!connInfo.initialParamsSet))
        {
          // Allocate data to send through clock handler
          connInfo.pParamUpdateEventData = ICall_malloc(sizeof(tpClockEventData_t) +
                                                           sizeof (uint16_t));
          if(connInfo.pParamUpdateEventData)
          {
            connInfo.pParamUpdateEventData->event = TP_SEND_PARAM_UPDATE_EVT;
            *((uint16_t *)connInfo.pParamUpdateEventData->data) = connInfo.connHandle;
            // Create a clock object and start
            connInfo.pUpdateClock = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));
            if (connInfo.pUpdateClock)
            {
              Util_constructClock(connInfo.pUpdateClock,
                                ThroughputPeripheral_clockHandler,
                                TP_SEND_PARAM_UPDATE_DELAY, 0, true,
                                (UArg) (connInfo.pParamUpdateEventData));
            }
            else
            {
              ICall_free(connInfo.pParamUpdateEventData);
            }
          }
        }

        // The initial connection parameters have been set, we only try once to get 7.5ms on Android
        // if the initial request fails
        connInfo.initialParamsSet = TRUE;

        // Write the current connection timeout value for use in the pseudo-timeout timer. We need this
        // because the GAP_LINK_TERMINATED event can't interrupt the while-loop in sendData.
        connInfo.connTimeout = linkInfo.connTimeout;

        // Update connection parameter characteristic
        ThroughputPeripheral_ConnectionControl_updateChar(linkInfo.connInterval, linkInfo.connLatency, linkInfo.connTimeout);

        // Restart throughput if it was on before the parameter request
        if(connInfo.restartThroughput)
        {
          ThroughputPeripheral_throughputOn();
          connInfo.restartThroughput = FALSE;
        }
      }
      else
      {
        // Display the address of the connection update failure
        if(BOARD_DISPLAY_USE_LCD == 1)
        {
          if(pPkt->status == HCI_ERROR_CODE_UNACCEPTABLE_CONN_PARAMETERS)
          {
            Display_printf(dispHandle, TP_ROW_STATUS_2, 0, "Params unacceptable");
          }
          // iPhones only accept connection interval updates some times. Re-send the parameter update until
          // we get a proper response
          else if((pPkt->status == HCI_ERROR_CODE_CONN_TERM_BY_LOCAL_HOST) ||
                  (pPkt->status == HCI_ERROR_CODE_DIFFERENT_TRANSACTION_COLLISION))
          {
            Display_printf(dispHandle, TP_ROW_STATUS_2, 0, "Requesting");
          }
          else
          {
            Display_printf(dispHandle, TP_ROW_STATUS_2, 0, "Fail: 0x%x", pPkt->status);
          }
        }
        else
        {
          if(pPkt->status == HCI_ERROR_CODE_UNACCEPTABLE_CONN_PARAMETERS)
          {
            Display_printf(dispHandle, TP_ROW_STATUS_2, 0, "Params unacceptable");
          }
          else if((pPkt->status == HCI_ERROR_CODE_CONN_TERM_BY_LOCAL_HOST) ||
                  (pPkt->status == HCI_ERROR_CODE_DIFFERENT_TRANSACTION_COLLISION))
          {
            Display_printf(dispHandle, TP_ROW_STATUS_2, 0, "Requesting");
          }
          else
          {
            Display_printf(dispHandle, TP_ROW_STATUS_2, 0, "Parameter update failed: 0x%x", pPkt->status);
          }
        }
        Display_clearLine(dispHandle, TP_ROW_STATUS_CONNINT);
        // iPhones only accept connection interval updates some times. Re-send the parameter update until
        // we get a proper response
        if((connInfo.peerType & peerType_IPHONE) &&
            ((pPkt->status == HCI_ERROR_CODE_CONN_TERM_BY_LOCAL_HOST) ||
             (pPkt->status == HCI_ERROR_CODE_DIFFERENT_TRANSACTION_COLLISION)))
        {
          Event_post(syncEvent, CCSERVICE_UPDATE_EVT);
        }
      }
      // Check if there are any queued parameter updates
      tpConnHandleEntry_t *connHandleEntry = (tpConnHandleEntry_t *)List_get(&paramUpdateList);
      if (connHandleEntry != NULL)
      {
        // Attempt to send queued update now
        ThroughputPeripheral_processParamUpdate(connHandleEntry->connHandle);

        // Free list element
        ICall_free(connHandleEntry);
      }
      break;
    }

    default:
      Display_clearLines(dispHandle, TP_ROW_STATUS_1, TP_ROW_STATUS_2);
      break;
  }
}

/*********************************************************************
 * @fn      ThroughputPeripheral_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void ThroughputPeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  tpGapAdvEventData_t *pData = ICall_malloc(sizeof(tpGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(ThroughputPeripheral_enqueueMsg(TP_ADV_EVT, pData) != SUCCESS)
    {
      ICall_freeMsg(pData);
    }
  }
}

/*********************************************************************
 * @fn      ThroughputPeripheral_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData - advertising event data
 *
 * @return  none
 */
static void ThroughputPeripheral_processAdvEvent(tpGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      if(BOARD_DISPLAY_USE_LCD == 0)
      {
        Display_printf(dispHandle, TP_ROW_ADVSTATE, 0, "Adv Set %d Enabled",
                       *(uint8_t *)(pEventData->pBuf));
      }
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
      if(BOARD_DISPLAY_USE_LCD == 0)
      {
        Display_printf(dispHandle, TP_ROW_ADVSTATE, 0, "Adv Set %d Disabled",
                       *(uint8_t *)(pEventData->pBuf));
      }
      break;

    case GAP_EVT_ADV_START:
      break;

    case GAP_EVT_ADV_END:
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);
      if(BOARD_DISPLAY_USE_LCD == 0)
      {
        Display_printf(dispHandle, TP_ROW_ADVSTATE, 0, "Adv Set %d disabled after conn %d",
                     advSetTerm->handle, advSetTerm->connHandle );
      }
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

/*********************************************************************
 * @fn      ThroughputPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void ThroughputPeripheral_processAppMsg(tpEvt_t *pMsg)
{
  bool dealloc = TRUE;
  switch (pMsg->event)
  {
    case TP_CHAR_CHANGE_EVT:
      ThroughputPeripheral_processCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;
    case TP_KEY_CHANGE_EVT:
      ThroughputPeripheral_handleKeys(*(uint8_t*)(pMsg->pData));
      break;

    case TP_ADV_EVT:
      ThroughputPeripheral_processAdvEvent((tpGapAdvEventData_t*)(pMsg->pData));
      break;

    case TP_PAIR_STATE_EVT:
      ThroughputPeripheral_processPairState((tpPairStateData_t*)(pMsg->pData));
      break;

    case TP_PASSCODE_EVT:
      ThroughputPeripheral_processPasscode((tpPasscodeData_t*)(pMsg->pData));
      break;

    case TP_READ_RPA_EVT:
      ThroughputPeripheral_updateRPA();
      break;

    case TP_SEND_PARAM_UPDATE_EVT:
    {
      // Extract connection handle from data
      uint16_t connHandle = *(uint16_t *)(((tpClockEventData_t *)pMsg->pData)->data);
      ThroughputPeripheral_processParamUpdate(connHandle);
      // This data is not dynamically allocated
      dealloc = FALSE;
      break;
    }
    case TP_TIMEOUT_EVT:
    {
        // Check if the message counter is the same value as last time the clock ran out
        if(msg_counter == connInfo.oldMsgCounter)
        {
            // Turn off throughput to handle the GAP_LINK_TERMINATED-event
            ThroughputPeripheral_throughputOff();
        }
        else
        {
            // Restart clock
            connInfo.oldMsgCounter = msg_counter;
            Util_restartClock(&clkTimeout, connInfo.connTimeout * 10);
        }
        break;
    }
    default:
      // Do nothing.
      break;
  }

  // Free message data if it exists
  if ((dealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      ThroughputPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Throughput Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void ThroughputPeripheral_processCharValueChangeEvt(uint8_t paramId)
{
  // Enable connection selection option
  tbm_setItemStatus(&tpMenuMain, TP_ITEM_SELECT_CONN, TBM_ITEM_NONE);
  switch(paramId)
  {
    case THROUGHPUT_SERVICE_UPDATE_PDU:
      // Check if we are to restart throughput after the parameter has changed
      if(throughputOn)
      {
        connInfo.restartThroughput = TRUE;
      }
      // Turn off Throughput - to allow application to process profile value change
      ThroughputPeripheral_throughputOff();

      // Inform Application to update PDU
      Event_post(syncEvent, TP_PDU_CHANGE_EVT);

      break;

    case THROUGHPUT_SERVICE_UPDATE_PHY:
      // Check if we are to restart throughput after the parameter has changed
      if(throughputOn)
      {
        connInfo.restartThroughput = TRUE;
      }
      // Turn off Throughput - to allow application to process profile value change
      ThroughputPeripheral_throughputOff();

      // Inform Application to update PHY
      Event_post(syncEvent, TP_PHY_CHANGE_EVT);

      break;

    case THROUGHPUT_SERVICE_TOGGLE_THROUGHPUT:
    {
        // Toggle throughput
        ThroughputPeripheral_doToggleThroughput(0);
	}
    break;

    default:
      break;
  }
}


/*********************************************************************
 * @fn      ThroughputPeripheral_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   none
 *
 * @return  none
 */
static void ThroughputPeripheral_updateRPA(void)
{
  // Read the current RPA.
  // The parameters for the call to HCI_LE_ReadLocalResolvableAddressCmd
  // are not needed to be accurate to retrieve the local resolvable address.
  // The 1st parameter can be any of ADDRMODE_PUBLIC and ADDRMODE_RANDOM.
  // The 2nd parameter only has to be not NULL.
  // The result will come with HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS
  // complete event.
 HCI_LE_ReadLocalResolvableAddressCmd(0, rpa);
}


/*********************************************************************
 * @fn      ThroughputPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  none
 */
static void ThroughputPeripheral_clockHandler(UArg arg)
{
  tpClockEventData_t *pData = (tpClockEventData_t *)arg;

  if (pData->event == TP_READ_RPA_EVT)
  {
    // Start the next period
    Util_startClock(&clkRpaRead);

    // Send message to read the current RPA
    ThroughputPeripheral_enqueueMsg(TP_READ_RPA_EVT, NULL);
  }

  if (pData->event == TP_SEND_PARAM_UPDATE_EVT)
  {
    // Send message to app
    ThroughputPeripheral_enqueueMsg(TP_SEND_PARAM_UPDATE_EVT, pData);
  }
  if (pData->event == TP_TIMEOUT_EVT)
  {
      ThroughputPeripheral_enqueueMsg(TP_TIMEOUT_EVT, NULL);
  }
}

/*********************************************************************
 * @fn      ThroughputPeripheral_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @param   *pDeviceAddr  - pointer to the peer address
 * @param   connHandle    - connection handle of the pairing peer
 * @param   uiInputs      - TRUE if the local device should accept a
 *                          passcode input
 * @param   uiOutputs     - TRUE if the local device should display
 *                          the passcode
 * @param   numComparison - Is zero until the passcode pairing is
 *                          complete. After that, the it is the code that
 *                          should be displayed for numeric comparison for
 *                          numeric comparison pairing
 *
 * @return  none
 */
static void ThroughputPeripheral_passcodeCb(uint8_t *pDeviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  tpPasscodeData_t *pData = ICall_malloc(sizeof(tpPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if(ThroughputPeripheral_enqueueMsg(TP_PASSCODE_EVT, pData) != SUCCESS)
    {
      ICall_freeMsg(pData);
    }
  }
}

/*********************************************************************
 * @fn      ThroughputPeripheral_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle of the peer
 * @param   state      - state of the pairing process
 * @param   status     - status of the pairing process
 *
 * @return  none
 */
static void ThroughputPeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  tpPairStateData_t *pData = ICall_malloc(sizeof(tpPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(ThroughputPeripheral_enqueueMsg(TP_PAIR_STATE_EVT, pData) != SUCCESS)
    {
      ICall_freeMsg(pData);
    }
  }
}

/*********************************************************************
 * @fn      ThroughputPeripheral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @param   *pPairData - pointer to the passcode data from the pairing
 *                       process
 *
 * @return  none
 */
static void ThroughputPeripheral_processPairState(tpPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
      Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Pairing started");
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Pairing success");
      }
      else
      {
        Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Pairing fail: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Encryption success");
      }
      else
      {
        Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Encryption failed: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Bond save success");
      }
      else
      {
        Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Bond save failed: %d", status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      ThroughputPeripheral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @param   *pPasscodeData - pointer to the passcode data from the
 *                           pairing process
 *
 * @return  none
 */
static void ThroughputPeripheral_processPasscode(tpPasscodeData_t *pPasscodeData)
{
  // Display passcode to user
  if (pPasscodeData->uiOutputs != 0)
  {
    Display_printf(dispHandle, TP_ROW_CONNECTION, 0, "Passcode: %d",
                   B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      ThroughputPeripheral_charValueChangeCB
 *
 * @brief   Callback from Throughput Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void ThroughputPeripheral_charValueChangeCB(uint8_t paramId)
{
  // If we wish to process the message in the Application Context
  // We would utilize inter process communications such as below

  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = paramId;

    if (ThroughputPeripheral_enqueueMsg(TP_CHAR_CHANGE_EVT, pValue) != SUCCESS)
      {
        ICall_freeMsg(pValue);
      }
  }
}

/**********************************************************************
 * @fn      ThroughputPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  SUCCESS - message was enqueued successfully
 * @return  FAILURE - message was not enqueued
 * @return  bleMemAllocError - could not allocate memory for the message
 */
static status_t ThroughputPeripheral_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  tpEvt_t *pMsg = ICall_malloc(sizeof(tpEvt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }
  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      ThroughputPeripheral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys - bitmap of pressed keys
 *
 * @return  none
 */
static void ThroughputPeripheral_keyChangeHandler(uint8_t keys)
{
  // Allocate memory for the message
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = keys;

    if(ThroughputPeripheral_enqueueMsg(TP_KEY_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_freeMsg(pValue);
    }
  }
}

/*********************************************************************
 * @fn      ThroughputPeripheral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 *
 * @return  none
 */
static void ThroughputPeripheral_handleKeys(uint8_t keys)
{
  if (keys & KEY_LEFT)
  {
    // Check if the key is still pressed. Workaround for possible bouncing.
    if (PIN_getInputValue(Board_PIN_BUTTON0) == 0)
    {
      tbm_buttonLeft();
    }
  }
  else if (keys & KEY_RIGHT)
  {
    // Check if the key is still pressed. Workaround for possible bouncing.
    if (PIN_getInputValue(Board_PIN_BUTTON1) == 0)
    {
      tbm_buttonRight();
    }
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void ThroughputPeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  uint8_t status = pMsg->pReturnParam[0];

  //Find which command this command complete is for
  switch (pMsg->cmdOpcode)
  {
    case HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS:
    {
      uint8_t* pRpaNew = &(pMsg->pReturnParam[1]);

      if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
      {
        // If the RPA has changed, update the display
        Display_printf(dispHandle, TP_ROW_RPA, 0, "RP Addr: %s",
                       Util_convertBdAddr2Str(pRpaNew));
        memcpy(rpa, pRpaNew, B_ADDR_LEN);
      }
      break;
    }
    // If the PDU size change is unsuccessful, restart throughput.
    case HCI_LE_SET_DATA_LENGTH:
    {
      if(status != SUCCESS)
      {
          // The peer doesn't support DLE, restart throughput if it was on
          if(connInfo.restartThroughput)
          {
            ThroughputPeripheral_throughputOn();
            connInfo.restartThroughput = FALSE;
          }
          /* Reset the value of the characteristic to ensure that the correct value
          *  is displayed in the app.
          */
          Throughput_Service_SetParameter(THROUGHPUT_SERVICE_UPDATE_PDU, sizeof(uint8_t),
                                          &connInfo.oldPDU);
      }
      break;
    }
    default:
      break;
  }
}

/*********************************************************************
* @fn      ThroughputPeripheral_updatePHYStat
*
* @brief   Update the auto phy update state machine
*
* @param   connHandle - the connection handle
*
* @return  none
*/
static void ThroughputPeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
{
  switch (eventCode)
  {
    case HCI_LE_SET_PHY:
    {
      // Get connection handle from list
      tpConnHandleEntry_t *connHandleEntry =
                           (tpConnHandleEntry_t *)List_get(&setPhyCommStatList);

      if (connHandleEntry)
      {
        ICall_free(connHandleEntry);
      }
      break;
    }

    // LE Event - a Phy update has completed or failed
    case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
    {
    // Do nothing
      break;
    }
    default:
      break;
  } // end of switch (eventCode)
}

/*********************************************************************
 * @fn      ThroughputPeripheral_addConn
 *
 * @brief   Add a device to the connected device info
 *
 * @param   connHandle - connection handle of the peer to add to connInfo
 *
 * @return  none
 */
static void ThroughputPeripheral_addConn(uint16_t connHandle)
{

  if (connInfo.connHandle == LINKDB_CONNHANDLE_INVALID)
  {
    // Found available entry to put a new connection info in
    connInfo.connHandle = connHandle;

    // Allocate data to send through clock handler
    connInfo.pParamUpdateEventData = ICall_malloc(sizeof(tpClockEventData_t) +
                                                     sizeof (uint16_t));
    if(connInfo.pParamUpdateEventData)
    {
      connInfo.pParamUpdateEventData->event = TP_SEND_PARAM_UPDATE_EVT;
      *((uint16_t *)connInfo.pParamUpdateEventData->data) = connHandle;
      // Create a clock object and start
      connInfo.pUpdateClock = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));
      if (connInfo.pUpdateClock)
      {
        Util_constructClock(connInfo.pUpdateClock,
                            ThroughputPeripheral_clockHandler,
                            TP_SEND_PARAM_UPDATE_DELAY,
                            0, true,
                            (UArg) (connInfo.pParamUpdateEventData));
      }
      else
      {
        ICall_free(connInfo.pParamUpdateEventData);
      }
    }
  }
}

/*********************************************************************
 * @fn      ThroughputPeripheral_removeConn
 *
 * @brief   Remove the device and free its clocks
 *
 * @param   none
 *
 * @return  none
 */
static void ThroughputPeripheral_removeConn(void)
{

  Clock_Struct* pUpdateClock = connInfo.pUpdateClock;

// Stop and destruct the connection parameter update clock
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
  // Reset Connection Info Entry
  ThroughputPeripheral_resetConnInfo();
}

/*********************************************************************
 * @fn      ThroughputPeripheral_processParamUpdate
 *
 * @brief   Process a parameter update request
 *
 * @param   connHandle - connection handle of the peer
 *
 * @return  none
 */
static void ThroughputPeripheral_processParamUpdate(uint16_t connHandle)
{
  // Request an initial connection parameter update based on what the
  // peer was recognized as when the initial connection was formed
  gapUpdateLinkParamReq_t req;
  req.connectionHandle = connHandle;
  if(connInfo.peerType & peerType_LAUNCHPAD)
  {
    req.connLatency = LAUNCHPAD_DEFAULT_DESIRED_SLAVE_LATENCY;
    req.connTimeout = LAUNCHPAD_DEFAULT_DESIRED_CONN_TIMEOUT;
    req.intervalMin = LAUNCHPAD_DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    req.intervalMax = LAUNCHPAD_DEFAULT_DESIRED_MAX_CONN_INTERVAL;
  }
  else if(connInfo.peerType & peerType_IPHONE)
  {
    req.connLatency = IPHONE_DEFAULT_DESIRED_SLAVE_LATENCY;
    req.connTimeout = IPHONE_DEFAULT_DESIRED_CONN_TIMEOUT;
    req.intervalMin = IPHONE_DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    req.intervalMax = IPHONE_DEFAULT_DESIRED_MAX_CONN_INTERVAL;
  }
  else
  {
    req.connLatency = ANDROID_DEFAULT_DESIRED_SLAVE_LATENCY;
    req.connTimeout = ANDROID_DEFAULT_DESIRED_CONN_TIMEOUT;
    req.intervalMin = ANDROID_DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    req.intervalMax = ANDROID_DEFAULT_DESIRED_MAX_CONN_INTERVAL;
  }

  // Send parameter update
  bStatus_t status = GAP_UpdateLinkParamReq(&req);

  // If there is an ongoing update, queue this for when the update completes
  if (status == bleAlreadyInRequestedMode)
  {
    tpConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(tpConnHandleEntry_t));
    if (connHandleEntry)
    {
      connHandleEntry->connHandle = connHandle;

      List_put(&paramUpdateList, (List_Elem *)&connHandleEntry);
    }
  }
  // Deconstruct the clock object
  Clock_destruct(connInfo.pUpdateClock);
  // Free clock struct
  ICall_free(connInfo.pUpdateClock);
  connInfo.pUpdateClock = NULL;
}

/*********************************************************************
* @fn      ThroughputPeripheral_setPhy
*
* @brief   Call the HCI set phy API and and add the handle to a
*          list to match it to an incoming command status event
*
* @param   connHandle - Connection handle of the peer
* @param   allPhys    - Host preference on how to handle txPhy and rxPhy
* @param   txPhy      - Bit field of Host preferred Tx PHY
* @param   rxPhy      - Bit field of Host preferred Rx PHY
* @param   phyOpts    - Bit field of Host preferred PHY options
*
* @return  SUCCESS
*/
static status_t ThroughputPeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts)
{
  // Allocate list entry to store handle for command status
  tpConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(tpConnHandleEntry_t));

  if (connHandleEntry)
  {
    connHandleEntry->connHandle = connHandle;

    // Add entry to the phy command status list
    List_put(&setPhyCommStatList, (List_Elem *)connHandleEntry);

    // Send PHY Update
    HCI_LE_SetPhyCmd(connHandle, allPhys, txPhy, rxPhy, phyOpts);
  }
  return SUCCESS;
}

/*********************************************************************
 * @fn      ThroughputPeripheral_resetConnInfo
 *
 * @brief   Reset the connection info of the current connection
 *
 * @param   none
 *
 * @return  none
 */
static void ThroughputPeripheral_resetConnInfo(void)
{
  // Clear specific handle or all handles
  connInfo.connHandle = LINKDB_CONNHANDLE_INVALID;
  connInfo.establishingConn = TRUE;
  connInfo.peerType = peerType_NOTIDENTIFIED;
  connInfo.oldPHY = LL_PHY_NONE;
  connInfo.requestedPHY = LL_PHY_NONE;
  connInfo.oldPDU = DEFAULT_PDU;
  connInfo.oldMsgCounter = 0;
  connInfo.initialParamsSet = FALSE;
  connInfo.restartThroughput = FALSE;

  // Set Initial Values of Characteristics in GATT Table
  uint8_t default_pdu = DEFAULT_PDU;
  uint8_t default_phy = DEFAULT_PHY;

  Throughput_Service_SetParameter(THROUGHPUT_SERVICE_UPDATE_PDU, sizeof(uint8_t),
                             &default_pdu);
  Throughput_Service_SetParameter(THROUGHPUT_SERVICE_UPDATE_PHY, sizeof(uint8_t),
                             &default_phy);

}

/*********************************************************************
 * @fn      ThroughputPeripheral_menuSwitchCb
 *
 * @brief   Detect menu context switching
 *
 * @param   pMenuObjCurr - the current menu object
 * @param   pMenuObjNext - the menu object the context is about to switch to
 *
 * @return  none
 */
static void ThroughputPeripheral_menuSwitchCb(tbmMenuObj_t* pMenuObjCurr,
                                       tbmMenuObj_t* pMenuObjNext)
{
  uint8_t NUMB_ACTIVE_CONNS = linkDB_NumActive();

  // We are only interested in the events of
  // entering scMenuConnect, tpMenuSelectConn, and scMenuMain for now
  if (pMenuObjNext == &tpMenuSelectConn)
  {
    static uint8_t* pAddrs;
    uint8_t* pAddrTemp;

    if (pAddrs != NULL)
    {
      ICall_free(pAddrs);
    }

    // Allocate buffer to display addresses
    pAddrs = ICall_malloc(NUMB_ACTIVE_CONNS * TP_ADDR_STR_SIZE);

    if (pAddrs == NULL)
    {
      TBM_SET_NUM_ITEM(&tpMenuSelectConn, 0);
    }
    else
    {

      TBM_SET_NUM_ITEM(&tpMenuSelectConn, 1);

      pAddrTemp = pAddrs;

      if (connInfo.connHandle != LINKDB_CONNHANDLE_INVALID)
      {
        // Get the address from the connection handle
        linkDBInfo_t linkInfo;
        linkDB_GetInfo(connInfo.connHandle, &linkInfo);
        // This connection is active. Set the corresponding menu item with
        // the address of this connection and enable the item.
        memcpy(pAddrTemp, Util_convertBdAddr2Str(linkInfo.addr),
               TP_ADDR_STR_SIZE);
        TBM_SET_ACTION_DESC(&tpMenuSelectConn, 0, pAddrTemp);
        tbm_setItemStatus(&tpMenuSelectConn, 1, TP_ITEM_NONE);
        pAddrTemp += TP_ADDR_STR_SIZE;
      }
      else
      {
        // This connection is not active. Disable the corresponding menu item.
        tbm_setItemStatus(&tpMenuSelectConn, TP_ITEM_NONE, 1);
      }
    }
  }
  else if (pMenuObjNext == &tpMenuMain)
  {
    // Now we are not in a specific connection's context

    // Clear connection-related message
    Display_clearLine(dispHandle, TP_ROW_CONNECTION);
  }
}
/*********************************************************************
 * @fn      ThroughputPeripheral_throughputOn
 *
 * @brief   Set throughputOn = TRUE and post an event to start
 *          throughput. Prints "Throughput ON"
 *
 * @param   none
 *
 * @return  none
 */
static void ThroughputPeripheral_throughputOn(void)
{
  // Turn on Throughput Demo & signal application
  // We don't turn on throughput when the initial PHY and PDU is being set
  if(!connInfo.establishingConn)
  {
      throughputOn = true;

      Display_print0(dispHandle, TP_ROW_TOGGLE, 0, "Throughput ON");

      Event_post(syncEvent, TP_THROUGHPUT_EVT);
  }
}
/*********************************************************************
 * @fn      ThroughputPeripheral_throughputOff
 *
 * @brief   Turns off throughput by setting throughputOn = FALSE.
 *          Also prints "Throughput OFF"
 *
 * @param   none
 *
 * @return  none
 */
static void ThroughputPeripheral_throughputOff(void)
{
  // Turn off Throughput Demo
  throughputOn = false;

  Display_print0(dispHandle, TP_ROW_TOGGLE, 0, "Throughput OFF");
}

/*********************************************************************
 * @fn      ThroughputPeripheral_sendData
 *
 * @brief   Sends ATT notifications in a tight while loop to demo
 *          throughput
 *
 * @param   uint16_t connHandle - Connection Handle
 *
 * @return  none
 */
static void ThroughputPeripheral_sendData(uint16_t connHandle)
{
  uint16_t len = 0;
  attHandleValueNoti_t noti;
  bStatus_t status;
  msg_counter = 0;

  // Subtract the total packet overhead of ATT and L2CAP layer from notification payload
  len = currentMTU - TOTAL_PACKET_OVERHEAD;

  // Get the handle of the throughput service value characteristic
  noti.handle = getThroughput_Service_Data_Handle();
  noti.len = len;
  Util_constructClock(&clkTimeout, ThroughputPeripheral_clockHandler,
                                (connInfo.connTimeout * 10), 0, true,
                                (UArg) &argTimeout);
  while(throughputOn)
  {
    noti.pValue = (uint8 *)GATT_bm_alloc( connHandle, ATT_HANDLE_VALUE_NOTI, GATT_MAX_MTU, &len );

    if ( noti.pValue != NULL ) //if allocated
    {
      // Fill the notification with data
      noti.pValue[0] = (msg_counter >> 24) & 0xFF;
      noti.pValue[1] = (msg_counter >> 16) & 0xFF;
      noti.pValue[2] = (msg_counter >> 8) & 0xFF;
      noti.pValue[3] = msg_counter & 0xFF;

      // Attempt to send the notification w/ no authentication
      status = GATT_Notification( connHandle, &noti, 0x00);
      if ( status != SUCCESS)
      {
        // If noti not sent, free the message. (If it is sent, the BLE-Stack will free the message.)
        GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
      }
      else
      {
        // Notification is successfully sent, increment counters
        msg_counter++;
      }
    }
    else
    {
      // bleNoResources was returned.
      // This is expected since we are continuosly trying to queue new packets.
      // Since we don't care about what happens to the data in the packet, we do 
      // nothing.
      asm(" NOP ");
    }
    // If RTOS queue is not empty, process app message.
    // We need to process the app message here in the case of a button press
     while (!Queue_empty(appMsgQueueHandle))
    {
      tpEvt_t *pMsg = (tpEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
      if (pMsg)
      {
        // Process message.
        ThroughputPeripheral_processAppMsg(pMsg);

        // Free the space from the message.
        ICall_free(pMsg);
      }
    }
  }
  if (Util_isActive(&clkTimeout))
  {
    Util_stopClock(&clkTimeout);
  }
  // Destruct the clock object
  Clock_destruct(&clkTimeout);
}

/*********************************************************************
 * @fn      ThroughputPeripheral_doSelectConn
 *
 * @brief   Select a connection to communicate with
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool ThroughputPeripheral_doSelectConn(uint8_t index)
{
  menuConnHandle = connInfo.connHandle;

  // Set the menu title and go to this connection's context
  TBM_SET_TITLE(&tpMenuPerConn, TBM_GET_ACTION_DESC(&tpMenuSelectConn, index));

  // Clear non-connection-related message
  Display_clearLine(dispHandle, TP_ROW_CONNECTION);

  tbm_goTo(&tpMenuPerConn);

  return (true);
}

/*********************************************************************
 * @fn      ThroughputPeripheral_doSetConnPhy
 *
 * @brief   Set PHY preference.
 *
 * @param   index - 0: 1M PHY
 *                  1: 2M PHY
 *                  2: CODED PHY: S2
 *                  3: CODED PHY: S8
 *
 * @return  always true
 */
bool ThroughputPeripheral_doSetConnPhy(uint8 index)
{
  bool status = TRUE;

  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_CODED,
  };

  static uint8_t options[] = {
    LL_PHY_OPT_NONE, LL_PHY_OPT_NONE,
    LL_PHY_OPT_S2, LL_PHY_OPT_S8,
  };

  ThroughputPeripheral_throughputOff();

  phyOptions = options[index];

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX.
  // Note PHYs are already enabled by default in build_config.opt in stack project.

  ThroughputPeripheral_setPhy(menuConnHandle, LL_PHY_USE_PHY_PARAM, phy[index],
                          phy[index], phyOptions);

  Display_printf(dispHandle, TP_ROW_STATUS_1, 0, "PHY preference: %s",
                 TBM_GET_ACTION_DESC(&tpMenuConnPhy, index));
  Throughput_Service_SetParameter(THROUGHPUT_SERVICE_UPDATE_PHY, sizeof(uint8_t),
                                 &index);
  return status;
}

/*********************************************************************
 * @fn      ThroughputPeripheral_doToggleThroughput
 *
 * @brief   Toggle throughput
 *
 * @param   index - 0 or 1
 *          0 - Make a controlled throughput toggle using the parameter
 *              from the Throughput Toggle characteristic
 *          1 - Toggle throughput locally with the throughputOn variable.
 *              Also used for toggling from phones, as they always write
 *              the value 2 to the characteristic.
 *
 * @return  always true
 */
bool ThroughputPeripheral_doToggleThroughput(uint8 index)
{

  // Variables needed
  uint8_t newToggleVal = 0;

  // Get Prereq data.
  Throughput_Service_GetParameter(THROUGHPUT_SERVICE_TOGGLE_THROUGHPUT, &newToggleVal);

  // If it is called from doToggleThroughput on the peripheral
  if(index == 1)
  {
    newToggleVal = !newToggleVal;
    Throughput_Service_SetParameter(THROUGHPUT_SERVICE_TOGGLE_THROUGHPUT, sizeof(uint8_t),
                                    &newToggleVal);
  }
  // If the peer is a phone, use throughputOn instead of the parameter because
  // the Throughput Toggle-characteristic is always 2
  if(!(connInfo.peerType & peerType_LAUNCHPAD))
  {
     throughputOn = !throughputOn;
     newToggleVal = throughputOn;
  }
  if(newToggleVal)
  {
    // Turn on Throughput Demo
    throughputOn = true;
    Display_print0(dispHandle, TP_ROW_TOGGLE, 0, "Throughput ON");
    Event_post(syncEvent, TP_THROUGHPUT_EVT);
  }
  else
  {
    // Turn off Throughput Demo
    throughputOn = false;

    Display_print0(dispHandle, TP_ROW_TOGGLE, 0, "Throughput OFF");
  }
  return true;
}
/*********************************************************************
 * @fn      ThroughputPeripheral_ConnControlValueChangeCB
 *
 * @brief   Callback function for Throughput_Peripheral_Conn_Ctrl to
 *          post an event to process either a Connection Interval
 *          characteristic change or disconnect from the peer
 *
 * @param   paramID - What service the callback was generated for
 *
 * @return  none
 */
static void ThroughputPeripheral_ConnectionControl_ValueChangeCB(uint8_t paramID) {
    if (paramID == CCSERVICE_CHAR2) {
        // Check if we are to restart throughput after the parameter has changed
        if(throughputOn)
        {
          connInfo.restartThroughput = TRUE;
        }
        // Turn off throughput to handle the request
        ThroughputPeripheral_throughputOff();
        // Inform Application to update CCService
        Event_post(syncEvent, CCSERVICE_UPDATE_EVT);
    }
    if (paramID == CCSERVICE_CHAR3) {

        // Turn off throughput to handle the request
        ThroughputPeripheral_throughputOff();
        // Inform Application to update CCService
        Event_post(syncEvent, CCSERVICE_KILL_CON_EVT);
    }
}

/*******************************************************************************
* @fn      ThroughputPeripheral_ConnectionControl_update
*
* @brief   Update the Connection Control service with the current connection
*          control settings
* @param   connInterval - the connection interval of the current connection
* @param   connSlaveLatency - the connection slave latency of the current connection
* @param   connTimeout - the connection timeout of the current connection
*
* @return  none
*/
static void ThroughputPeripheral_ConnectionControl_updateChar(uint16_t connInterval,
                                                 uint16_t connSlaveLatency, uint16_t connTimeout)
{
 uint8_t buf[CCSERVICE_CHAR1_LEN];

 buf[0] = LO_UINT16(connInterval);
 buf[1] = HI_UINT16(connInterval);
 buf[2] = LO_UINT16(connSlaveLatency);
 buf[3] = HI_UINT16(connSlaveLatency);
 buf[4] = LO_UINT16(connTimeout);
 buf[5] = HI_UINT16(connTimeout);

 CcService_setParameter(CCSERVICE_CHAR1,sizeof(buf),buf);
}
/*******************************************************************************
* @fn      ThroughputPeripheral_ConnControl_processCharChangeEvt
*
* @brief   Process a change in the connection control characteristics
*
* @param   paramID - what service to process the characteristic value for
* @param   connHandle - the connection handle of the current peer
* @param   peerType - what device the peer is
*
* @return  none
*/
static void ThroughputPeripheral_ConnectionControl_processCharChangeEvt(uint8_t paramID)
{
 // CCSERVICE_CHAR1: read & notify only

 // CCSERVICE_CHAR: requested connection parameters
 if (paramID == CCSERVICE_CHAR2)
 {
   uint8_t buf[CCSERVICE_CHAR2_LEN];

   // Get new connection parameters
   CcService_getParameter(CCSERVICE_CHAR2, buf);

   gapUpdateLinkParamReq_t req;
   req.connectionHandle = connInfo.connHandle;
   req.intervalMin = BUILD_UINT16(buf[0],buf[1]);
   req.intervalMax = BUILD_UINT16(buf[2],buf[3]);
   req.connLatency = BUILD_UINT16(buf[4],buf[5]);
   req.connTimeout = BUILD_UINT16(buf[6],buf[7]);

   /* iPhone has certain requirements for the parameters of a connection parameter request.
   *  For reference, see Apple Accessory Design Guidelines. The following criteria are based on
   *  version R5 of the guidelines.
   */
   if(connInfo.peerType & peerType_IPHONE)
   {
     // Interval Min % 15ms == 0
     req.intervalMin = (req.intervalMin / 12) * 12;

     // Interval Max >= Interval Min + 15ms
     if(req.intervalMin == IPHONE_MINIMUM_CONNINT)
     {
         req.intervalMax = IPHONE_MINIMUM_CONNINT;
     }
     else if(req.intervalMax < (req.intervalMin + 12))
     {
         req.intervalMax = req.intervalMin + 12;
     }

     // Timeout > Interval Max * 3
     if(req.connTimeout <= (3*req.intervalMax))
     {
       req.connTimeout = (3 * req.intervalMax) + 12;
     }

     // 2000ms <= Timeout <= 6000ms
     if(req.connTimeout > IPHONE_MAXIMUM_CONNTIMEOUT)
     {
         req.connTimeout = IPHONE_MAXIMUM_CONNTIMEOUT;
     }
     else if(req.connTimeout < IPHONE_MINIMUM_CONNTIMEOUT)
     {
       req.connTimeout = IPHONE_MINIMUM_CONNTIMEOUT  * 1.25;
     }

     // Timeout parameter is in units of 10ms, but the Interval is per 1.25ms, so we divide by 8
     req.connTimeout = req.connTimeout / 8;

     // The iPhone app sends out parameters that are 1/1.25 = 0.80 times smaller than the slider shows. We have to compensate for this
     req.intervalMin = (req.intervalMin * 125) / 100;
     req.intervalMax = (req.intervalMax * 125) / 100;
     req.connTimeout = (req.connTimeout * 125) / 100;
   }
   // Android has some quirks as well: The requested parameters are 
   // 1/1.25 = 0.80 times too small, and the displayed values on the phone are 
   // 1.25 too great. Also, Android doesn't care what the parameters are as long 
   // as the Interval is greater than or equal to 7.5ms.
   else if(connInfo.peerType & peerType_ANDROID)
   {
     // If the desired interval is less than 7.5ms, we choose the lowest possible interval, which is 7.5ms.
     if(req.intervalMin < ANDROID_DEFAULT_DESIRED_MIN_CONN_INTERVAL)
     {
       req.intervalMin = ANDROID_DEFAULT_DESIRED_MIN_CONN_INTERVAL;
     }
     // Otherwise, we multiply by 1.25 to compensate.
     else
     {
       req.intervalMin = (req.intervalMin * 125) / 100;
     }
     // Set Max desired interval to Min desired interval.
     req.intervalMax = req.intervalMin;
   }

   // Send parameter update
   bStatus_t status = GAP_UpdateLinkParamReq(&req);
   if(status != SUCCESS)
   {
     Display_printf(dispHandle, TP_ROW_STATUS_CONNINT, 0, "Status: 0x%x", status);
   }
 }
 // CCSERVICE_CHAR3: Disconnect request
 else if (paramID == CCSERVICE_CHAR3)
 {
   // Any change in the value will terminate the connection
   bStatus_t status = GAP_TerminateLinkReq(connInfo.connHandle, L2CAP_TERM_BY_PSM);
   if(status != SUCCESS)
   {
     Display_printf(dispHandle, TP_ROW_STATUS_CONNINT, 0, "Status: 0x%x", status);
   }
 }
}
/*******************************************************************************
* @fn      ThroughputPeripheral_ConnectionControlChangeCB
*
* @brief   Callback from Connection Control indicating a value change
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  none
*/
static void ThroughputPeripheral_ConnectionControlChangeCB(uint8_t paramID)
{
 // Wake up the application thread
   ThroughputPeripheral_ConnectionControl_ValueChangeCB(paramID);
}



/*********************************************************************
*********************************************************************/
