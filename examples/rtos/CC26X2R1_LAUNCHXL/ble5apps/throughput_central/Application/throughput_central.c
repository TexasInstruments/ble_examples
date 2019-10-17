/******************************************************************************

 @file  throughput_central.c

 @brief This file contains the Throughput Central sample application for use
        with the CC2652 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc13x2_26x2

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

 ******************************************************************************
 
 
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

//Included for heap debug
#include <xdc/cfg/global.h>

#include "bcomdef.h"

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"
#include "osal_list.h"
#include "board_key.h"
#include "Board.h"

#include "ble_user_config.h"

#include <profiles/throughput_service.h>

#include <menu/two_btn_menu.h>
#include "throughput_central.h"
#include "throughput_central_menu.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Application events
#define TC_EVT_KEY_CHANGE             0x01
#define TC_EVT_SCAN_ENABLED           0x02
#define TC_EVT_SCAN_DISABLED          0x03
#define TC_EVT_ADV_REPORT             0x04
#define TC_EVT_SVC_DISC               0x05
#define TC_EVT_PAIR_STATE             0x06
#define TC_EVT_PASSCODE_NEEDED        0x07
#define TC_EVT_READ_RPA               0x08
#define TC_EVT_INSUFFICIENT_MEM       0x09
#define TC_EVT_TOGGLE_THROUGHPUT      0x0A
#define TC_EVT_PDU_UPDATE             0x0B
#define TC_EVT_PHY_UPDATE             0x0C
#define TC_EVT_MEASURE_INST_SPEED_EVT 0x0D
#define TC_EVT_MEASURE_AVG_SPEED_EVT  0x0E
#define TC_EVT_READ_RSSI              0x0F

// Throughput Central Task Events
#define TC_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define TC_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define TC_ALL_EVENTS                        (TC_ICALL_EVT           | \
                                              TC_QUEUE_EVT)
// Address mode of the local device
#define DEFAULT_ADDRESS_MODE                 ADDRMODE_PUBLIC

// Default PHY for scanning and initiating
#define DEFAULT_SCAN_PHY                     SCAN_PRIM_PHY_1M
#define DEFAULT_INIT_PHY                     INIT_PHY_1M

// Default Duplicate Packet filter
#define DEFAULT_LL_FILTER                    SCAN_FLT_DUP_ENABLE

// Default PDU filter
#define DEFAULT_PDU_FILTER                   SCAN_FLT_PDU_CONNECTABLE_ONLY | SCAN_FLT_PDU_COMPLETE_ONLY

// Default scan duration in 10 ms
#define DEFAULT_SCAN_DURATION                100 // 1 s

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                  3000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID         TRUE

// How often to read current current RPA (in ms)
#define DEFAULT_READ_RPA_PERIOD              3000

// Delay when switching between Coded PHY modes
#define CODED_PHY_CHANGE_DELAY               500

// Task configuration
#define TC_TASK_PRIORITY                     1

#ifndef TC_TASK_STACK_SIZE
#define TC_TASK_STACK_SIZE                   1024
#endif

// Advertising report fields to keep in the list
// Interested in only peer address type and peer address
#define DEFAULT_ADV_RPT_FIELDS   (SCAN_ADVRPT_FLD_ADDRTYPE | SCAN_ADVRPT_FLD_ADDRESS)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define TC_ADDR_STR_SIZE     15

// Row numbers for two-button menu
#define TC_ROW_SEPARATOR        (TBM_ROW_APP + 0)
#define TC_ROW_CUR_CONN         (TBM_ROW_APP + 1)
#define TC_ROW_ANY_CONN         (TBM_ROW_APP + 2)
#define TC_ROW_NON_CONN         (TBM_ROW_APP + 3)
#define TC_ROW_NUM_CONN         (TBM_ROW_APP + 4)
#define TC_ROW_IDA              (TBM_ROW_APP + 5)
#define TC_ROW_RPA              (TBM_ROW_APP + 6)
#define TC_ROW_PEER_ADDR        (TBM_ROW_APP + 7)
#define TC_ROW_THROUGHPUT_INST  (TBM_ROW_APP + 8)
#define TC_ROW_THROUGHPUT_AVG   (TBM_ROW_APP + 9)
#define TC_ROW_NUM_PACKETS_LOST (TBM_ROW_APP + 10)

// Spin if the expression is not true
#define THROUGHPUTCENTRAL_ASSERT(expr) if (!(expr)) ThroughputCentral_spin();

// Circular buffer size for throughput
#define CB_SIZE                 8

// Default/Max PDU and Tx Time
#define DEFAULT_PDU_SIZE        27
#define DEFAULT_TX_TIME         328

#define DLE_MAX_PDU_SIZE        251
#define DLE_MAX_TX_TIME         17040

/*********************************************************************
 * TYPEDEFS
 */

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} tcEvt_t;

// Scanned device information record
typedef struct
{
  uint8_t addrType;         // Peer Device's Address Type
  uint8_t addr[B_ADDR_LEN]; // Peer Device Address
} scanRec_t;

typedef struct
{
  uint16_t charHandle;          //!< Characteristic Handle
  uint8 addr[ATT_UUID_SIZE];    //!< UUID of characteristic
} throughputProfileHdl_t;

typedef struct
{
  uint32_t bytesRecvd;              //  Total amount of bytes received
  uint32_t bytesRecvd_cb[CB_SIZE];  //  Circular buffer for the CB_SIZE latest bytes
  int      bytesRecvd_cb_index;     //  Index for the current position in the circular buffer
  bool     cbBufferFilled;          //  Whether the buffer is full
  uint32_t instantRate;             //  Total number of bytes received the previous second
  uint32_t averageRate;             //  Average of the total number of bytes received
} InstTime_t;

// Connected device information
typedef struct
{
  uint16_t connHandle;                                           // Connection Handle
  uint8_t  addr[B_ADDR_LEN];                                     // Peer Device Address
  throughputProfileHdl_t throughputObject[NUM_THROUGHPUT_CHARS]; // Number of Throughput chars in its service (see throughput.)
  Clock_Struct *pRssiClock;                                      // pointer to clock struct
  uint8_t throughputToggle;                                      // value to indicate if throughput is on or off
  uint8_t phyIndex;                                              // index for PHY RX and TX selection
  uint8_t phyOptions;                                            // PHY option selection mainly for Coded PHY
  uint8_t phyClock_phyIndex;                                     // Requested PHY index for Coded PHY transitions
  bool phyConfirm;                                               // PHY Confirm Status
  uint8_t pduValue;                                              // value to indicate the PDU of the connection
  InstTime_t metrics;                                              // Metric for throughput
  bool establishingConn;                                         // Boolean value to indicate if the connection is being formed
} connRec_t;

// Container to store paring state info when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint16_t connHandle;
  uint8_t  status;
} tcPairStateData_t;
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

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock object for calling speedHandler() every second to calculate throughput
static Clock_Struct speedClock;

// Clock object for delayed PHY/PDU characteristic writes
static Clock_Struct delayedTrigger;

// Clock object for switching between coded PHY's, changing to 1M PHY between
// codings for stability
static Clock_Struct startPHYClock;

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration (Only Valid for CCS, if using IAR this code will not work.)
Task_Struct tcTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(tcTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t tcTaskStack[TC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Throughput Central";

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
// Number of scan results filtered by Service UUID
static uint8_t numScanRes = 0;

// Scan results filtered by Service UUID
static scanRec_t scanList[DEFAULT_MAX_SCAN_RES];
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

// Number of connected devices
static uint8_t numConn = 0;

// List of connections
static connRec_t connList[MAX_NUM_BLE_CONNS];

// Connection handle of current connection
static uint16_t tcConnHandle = LINKDB_CONNHANDLE_INVALID;

// Accept or reject L2CAP connection parameter update request
static bool acceptParamUpdateReq = true;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Maximum PDU size (default = 27 octets)
static uint16_t tcMaxPduSize;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

// Strings for PHY
static uint8_t* phyName[] = {
  "1M", "2M",
  "Coded:S2", "Coded:S8",
  "Coded"
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void ThroughputCentral_init(void);
static void ThroughputCentral_taskFxn(uintptr_t a0, uintptr_t a1);

static void ThroughputCentral_handleKeys(uint8_t keys);
static uint8_t ThroughputCentral_processStackMsg(ICall_Hdr *pMsg);
static void ThroughputCentral_processGapMsg(gapEventHdr_t *pMsg);
static void ThroughputCentral_processGATTMsg(gattMsgEvent_t *pMsg);
static void ThroughputCentral_processAppMsg(tcEvt_t *pMsg);
static void ThroughputCentral_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void ThroughputCentral_startSvcDiscovery(void);
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
static bool ThroughputCentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                      uint16_t dataLen);
static void ThroughputCentral_addScanInfo(uint8_t *pAddr, uint8_t addrType);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
static uint8_t ThroughputCentral_addConnInfo(uint16_t connHandle, uint8_t *pAddr);
static uint8_t ThroughputCentral_removeConnInfo(uint16_t connHandle);
static uint8_t ThroughputCentral_getConnIndex(uint16_t connHandle);
static char* ThroughputCentral_getConnAddrStr(uint16_t connHandle);
static void ThroughputCentral_processPairState(uint8_t state, tcPairStateData_t* pPairStateData);
static void ThroughputCentral_processPasscode(uint16_t connHandle,
                                          uint8_t uiOutputs);
static void ThroughputCentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static status_t ThroughputCentral_StartRssi();
static status_t ThroughputCentral_StopRssi(uint16_t connHandle);
static void ThroughputCentral_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs,
                                     uint32_t numComparison);
static void ThroughputCentral_pairStateCb(uint16_t connHandle, uint8_t state,
                                      uint8_t status);

static void ThroughputCentral_keyChangeHandler(uint8 keys);
static void ThroughputCentral_clockHandler(UArg arg);
static void ThroughputCentral_speedHandler(UArg a0);
static void ThroughputCentral_PHYHandler(UArg a0);
static status_t ThroughputCentral_enqueueMsg(uint8_t event, uint8_t status,
                                        uint8_t *pData);
static void ThroughputCentral_scanCb(uint32_t evt, void* msg, uintptr_t arg);
static void ThroughputCentral_menuSwitchCb(tbmMenuObj_t* pMenuObjCurr,
                                       tbmMenuObj_t* pMenuObjNext);
static void ThroughputCentral_processInstantSpeed(connRec_t *connection);
static void ThroughputCentral_processAverageSpeed(connRec_t *connection);
static void ThroughputCentral_processMetric(connRec_t *connection);
static void ThroughputCentral_processThroughput(uint16_t connHandle);
static void ThroughputCentral_processPDUUpdate(uint16_t charValue);
static void ThroughputCentral_processPHYUpdate(uint16_t charValue);


/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Bond Manager Callbacks
static gapBondCBs_t bondMgrCBs =
{
  ThroughputCentral_passcodeCb, // Passcode callback
  ThroughputCentral_pairStateCb // Pairing/Bonding state Callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      ThroughputCentral_spin
 *
 * @brief   Spin forever if a THROUGHPUTCENTRAL_ASSERT fails
 *
 * @param   none
 *
 * @return  none
 */
static void ThroughputCentral_spin(void)
{
  volatile uint8_t x;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_createTask
 *
 * @brief   Task creation function for the Throughput Central.
 *
 * @param   none
 *
 * @return  none
 */
void ThroughputCentral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = tcTaskStack;
  taskParams.stackSize = TC_TASK_STACK_SIZE;
  taskParams.priority = TC_TASK_PRIORITY;

  Task_construct(&tcTask, ThroughputCentral_taskFxn, &taskParams, NULL);
}
/*********************************************************************
 * @fn      ThroughputCentral_placeholderClockFxn
 *
 * @brief   Placeholder function for the initialisation of the
 *          delayed trigger clock. Is replaced by one of the following 
 *          functions whan the clock is started:
 *          - ThroughputCentral_delayedPDUTriggerFxn
 *          - ThroughputCentral_delayedPHYTriggerFxn
 *          - ThroughputCentral_delayedThroughputTriggerFxn
 *
 * @param   arg = NULL
 *
 * @return  none
 */
void ThroughputCentral_placeholderClockFxn(UArg arg)
{
    while(1);
}
/*********************************************************************
 * @fn      ThroughputCentral_defaultPDUTriggerFxn
 *
 * @brief   Function used for delayed calls of
 *          ThroughputCentral_processPDUUpdate
 *
 * @param   arg - The desired PDU size (27 or 251)
 *
 * @return  none
 */
void ThroughputCentral_delayedPDUTriggerFxn(UArg arg)
{
    ThroughputCentral_enqueueMsg(TC_EVT_PDU_UPDATE, SUCCESS, (uint8_t *)arg);
}
/*********************************************************************
 * @fn      ThroughputCentral_defaultPHYTriggerFxn
 *
 * @brief   Function used for delayed calls to
 *          ThroughputCentral_processPHYUpdate
 *
 * @param   arg - The desired connection PHY
 *
 * @return  none
 */
void ThroughputCentral_delayedPHYTriggerFxn(UArg arg)
{
    ThroughputCentral_enqueueMsg(TC_EVT_PHY_UPDATE, SUCCESS, (uint8_t *)arg);
}
/*********************************************************************
 * @fn      ThroughputCentral_defaultToggleThroughputTriggerFxn
 *
 * @brief   Function used for delayed calls to
 *          ThroughputCentral_processThroughput
 *
 * @param   arg - Connhandle of the connection to toggle throughput to
 *
 * @return  none
 */
void ThroughputCentral_delayedThroughputTriggerFxn(UArg arg)
{
    ThroughputCentral_enqueueMsg(TC_EVT_TOGGLE_THROUGHPUT, SUCCESS, (uint8_t *)arg);
}

/*********************************************************************
 * @fn      ThroughputCentral_Init
 *
 * @brief   Initialization function for the Throughput Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void ThroughputCentral_init(void)
{
  uint8_t i;

  // Create the menu
  Throughput_buildMenu();

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // One-time clock function.
  Util_constructClock(&delayedTrigger, ThroughputCentral_placeholderClockFxn,
                      1000, 0, false, NULL);

  // Setup throughput clock to run every second
  Util_constructClock(&speedClock, ThroughputCentral_speedHandler,
                      1000, 1000, true, NULL);

  // Set up a PHY Clock for transitions between Coded PHYs
  Util_constructClock(&startPHYClock, ThroughputCentral_PHYHandler,
                      0, 0, false, 0);

  // Initialize internal data
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
    connList[i].pRssiClock = NULL;
    connList[i].throughputToggle = 0;
    connList[i].phyIndex = TS_PHY_1M;
    connList[i].phyOptions = LL_PHY_OPT_NONE;
    connList[i].pduValue = DEFAULT_PDU_SIZE;
    connList[i].establishingConn = TRUE;
    connList[i].metrics.cbBufferFilled = FALSE;
    memset(connList[i].metrics.bytesRecvd_cb, 0, sizeof(connList[i].metrics.bytesRecvd_cb));
  }

  Board_initKeys(ThroughputCentral_keyChangeHandler);

  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void *)attDeviceName);

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set Bond Manager parameters
  {
    // Don't send a pairing request after connecting; the device waits for the
    // application to start pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_NO_PAIRING;
    // Do not use authenticated pairing
    uint8_t mitm = FALSE;
    // This is a display only device
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    // Create a bond during the pairing process
    uint8_t bonding = FALSE;
    uint8_t gapbondSecure = GAPBOND_SECURE_CONNECTION_NONE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &gapbondSecure);
  }

  // Start Bond Manager and register callback
  // This must be done before initialing the GAP layer
  VOID GAPBondMgr_Register(&bondMgrCBs);

  // Accept all parameter update requests
  GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, GAP_UPDATE_REQ_ACCEPT_ALL);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Define the address mode
  GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

  // Initialize GAP layer for Central role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_CENTRAL, selfEntity, addrMode, NULL);

// Get Current Data Length
  HCI_LE_ReadMaxDataLenCmd();

  // By Default Allow Central to support any and all PHYs
  HCI_LE_SetDefaultPhyCmd(LL_PHY_USE_ANY_PHY, LL_PHY_1_MBPS | LL_PHY_2_MBPS| HCI_PHY_CODED, LL_PHY_1_MBPS | LL_PHY_2_MBPS| HCI_PHY_CODED);

  // Set the Transmit Power of the Device to +5dBm
  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);

  // Set the RX Gain to be highest
  HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);

  // The type of display is configured based on the BOARD_DISPLAY_USE...
  // preprocessor definitions
  dispHandle = Display_open(Display_Type_ANY, NULL);

  // Set the title of the main menu
  TBM_SET_TITLE(&tcMenuMain, "Texas Instruments Bluetooth 5 Throughput Central");

  // Disable all items in the main menu
  tbm_setItemStatus(&tcMenuMain, TC_ITEM_NONE, TC_ITEM_ALL);
  // Initialize Two-button Menu
  tbm_initTwoBtnMenu(dispHandle, &tcMenuMain, 4, ThroughputCentral_menuSwitchCb);
  Display_printf(dispHandle, TC_ROW_SEPARATOR, 0, "====================");
}


/*********************************************************************
 * @fn      ThroughputCentral_taskFxn
 *
 * @brief   Application task entry point for the Throughput Central.
 *
 * @param   a0 and a1 - uint pointers, not used
 *
 * @return  events not processed
 */
static void ThroughputCentral_taskFxn(uintptr_t a0, uintptr_t a1)
{
  // Initialize application
  ThroughputCentral_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, TC_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      while (ICall_fetchServiceMsg(&src, &dest,
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
           safeToDealloc = ThroughputCentral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message
      if (events & TC_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          tcEvt_t *pMsg = (tcEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
              // Process message
              ThroughputCentral_processAppMsg(pMsg);

              // Free the space from the message
              ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 *
 * @return  none
 */
static void ThroughputCentral_handleKeys(uint8_t keys)
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
 * @fn      ThroughputCentral_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ThroughputCentral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      ThroughputCentral_processGapMsg((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      ThroughputCentral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch (pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
          ThroughputCentral_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
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
                    Display_printf(dispHandle, TC_ROW_CUR_CONN, 0,
                            "PHY Change failure, peer does not support this");
                  }
                  else
                  {
                    Display_printf(dispHandle, TC_ROW_CUR_CONN, 0,
                                   "PHY Update Status: 0x%02x",
                                   pMyMsg->cmdStatus);
                  }
                }
                break;
              case HCI_DISCONNECT:
                break;

              default:
                {
                  Display_printf(dispHandle, TC_ROW_NON_CONN, 0,
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
              Display_printf(dispHandle, TC_ROW_ANY_CONN, 0,
                             "%s: PHY change failure",
                             ThroughputCentral_getConnAddrStr(pPUC->connHandle));
            }
            else
            {
              uint8_t connIndex = ThroughputCentral_getConnIndex(tcConnHandle);

              // Figure out which PHY is being used
              uint8_t temp = 0;
              switch(pPUC->txPhy)
              {
                case HCI_PHY_1_MBPS:
                  temp = 0;
                  break;

                case HCI_PHY_2_MBPS:
                  temp = 1;
                  break;

                case HCI_PHY_CODED:
                  temp = 4;
                  break;
              }

              // If PhyConfirm is false, that means we initated the change
              // if that is the case, then we can use detailed information
              // for coded PHY - use phyIndex instead
              if (connList[connIndex].phyConfirm == false)
              {
                // This means that the phyIndex was assigned by us.
                // Confirm the value

                // Critial Section so our Timer's SWI can't read the value while
                // we're writing to it.
                UInt key = Hwi_disable();
                {
                  // Confirm the change to the phyIndex
                  connList[connIndex].phyConfirm = true;
                }
                Hwi_restore(key);
              }
              else
              {
                // the peer device requested the change to PHY
                // update phyIndex accordingly and display the value
                connList[connIndex].phyIndex = temp;
              }

              // Inform User that the PHY was Updated, and which PHY is
              // the PHY being used for the connection
              if(BOARD_DISPLAY_USE_LCD == 1)
              {
                Display_printf(dispHandle, TC_ROW_ANY_CONN, 0, "PHY: %s", phyName[temp]);
              }
              else
              {
                Display_printf(dispHandle, TC_ROW_ANY_CONN, 0,
                               "%s: PHY updated to %s",
                               ThroughputCentral_getConnAddrStr(pPUC->connHandle),
                               phyName[temp]);
              }
            }
          }
          if (pPUC->BLEEventCode == HCI_BLE_DATA_LENGTH_CHANGE_EVENT)
          {
            // TX PDU Size Updated
            hciEvt_BLEDataLengthChange_t *dleEvt = (hciEvt_BLEDataLengthChange_t *)pMsg;
            if(BOARD_DISPLAY_USE_LCD == 1)
            {
              Display_printf(dispHandle, TC_ROW_ANY_CONN, 0, "PDU: %dB", dleEvt->maxRxOctets);
            }
            else
            {
              Display_printf(dispHandle, TC_ROW_ANY_CONN, 0, "%s: PDU updated to: %dB",
                             ThroughputCentral_getConnAddrStr(pPUC->connHandle),
                             dleEvt->maxRxOctets);
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
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      ThroughputCentral_processGapMsg
 *
 * @brief   GAP message processing function.
 *
 * @param   pMsg - pointer to event message structure
 *
 * @return  none
 */
static void ThroughputCentral_processGapMsg(gapEventHdr_t *pMsg)
{
  switch (pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      // Assign initial parameter variable values
      uint8_t scanPHY = DEFAULT_SCAN_PHY;
      uint8_t LLFilter = DEFAULT_LL_FILTER;
      uint16_t advFieldsToKeep = DEFAULT_ADV_RPT_FIELDS;
      uint16_t PDUFilter = DEFAULT_PDU_FILTER;
      GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      // Setup scanning
      // For more information, see the GAP section in the User's Guide.

      // Register callback to process Scanner events
      GapScan_registerCb(ThroughputCentral_scanCb, NULL);

      // Set Scanner Event Mask
      GapScan_setEventMask(GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED |
                           GAP_EVT_ADV_REPORT);

      // Set Scan PHY parameters
      GapScan_setPhyParams(DEFAULT_SCAN_PHY, SCAN_TYPE_ACTIVE,
                           SCAN_PARAM_DFLT_INTERVAL, SCAN_PARAM_DFLT_INTERVAL);

      // Set Advertising report fields to keep
      GapScan_setParam(SCAN_PARAM_RPT_FIELDS, &advFieldsToKeep);
      // Set Scanning Primary PHY
      GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &scanPHY);
      // Set LL Duplicate Filter
      GapScan_setParam(SCAN_PARAM_FLT_DUP, &LLFilter);

      // Set PDU type filter -
      // Only 'Connectable' and 'Complete' packets are desired.
      // It doesn't matter if received packets are
      // whether Scannable or Non-Scannable, whether Directed or Undirected,
      // whether Scan_Rsp's or Advertisements, and whether Legacy or Extended.

      GapScan_setParam(SCAN_PARAM_FLT_PDU_TYPE, &PDUFilter);
      if(pPkt->dataPktLen != NULL)
      {
          tcMaxPduSize = pPkt->dataPktLen;
      }

      // Enable "Discover Devices" and "Set Scanning PHY" in the main menu
      tbm_setItemStatus(&tcMenuMain,
                        TC_ITEM_STARTDISC | TC_ITEM_SCANPHY, TC_ITEM_NONE);

      Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "Initialized");


      // Display device address
      Display_printf(dispHandle, TC_ROW_IDA, 0, "%s Addr: %s",
                     (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
                     Util_convertBdAddr2Str(pPkt->devAddr));

      if (addrMode > ADDRMODE_RANDOM)
      {
        // If the local device is supposed to use privacy,
        // read the current RPA and display.
        // The parameters for the call to HCI_LE_ReadLocalResolvableAddressCmd
        // don't have to be accurate to retrieve the local resolvable address.
        // The 1st parameter can be any of ADDRMODE_PUBLIC and ADDRMODE_RANDOM.
        // The 2nd parameter only has to be not NULL.
        // The result will come with HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS
        // complete event.
        // Update the current RPA.
        HCI_LE_ReadLocalResolvableAddressCmd(0, rpa);

        // Create one-shot clock for RPA check event.
        Util_constructClock(&clkRpaRead, ThroughputCentral_clockHandler,
                            DEFAULT_READ_RPA_PERIOD, 0, true, TC_EVT_READ_RPA);
      }
      break;
    }

    case GAP_CONNECTING_CANCELLED_EVENT:
    {
      uint16_t itemsToEnable = TC_ITEM_SCANPHY | TC_ITEM_STARTDISC |
                               TC_ITEM_CONNECT;

      if (numConn > 0)
      {
        itemsToEnable |= TC_ITEM_SELECTCONN;
      }

      Display_printf(dispHandle, TC_ROW_NON_CONN, 0,
                     "Connecting attempt cancelled");

      // Enable "Discover Devices", "Connect To", and "Set Scanning PHY"
      // and disable everything else.
      tbm_setItemStatus(&tcMenuMain,
                        itemsToEnable, TC_ITEM_ALL & ~itemsToEnable);

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      uint16_t connHandle = ((gapEstLinkReqEvent_t*) pMsg)->connectionHandle;
      uint8_t* pAddr = ((gapEstLinkReqEvent_t*) pMsg)->devAddr;
      uint8_t  connIndex;
      uint32_t itemsToDisable = TC_ITEM_STOPDISC | TC_ITEM_CANCELCONN;
      uint8_t* pStrAddr;
      uint8_t i;
      uint8_t numConnectable = 0;

      // Add this connection info to the list
      connIndex = ThroughputCentral_addConnInfo(connHandle, pAddr);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      THROUGHPUTCENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      for (i = 0; i < NUM_THROUGHPUT_CHARS; i++)
      {
          memset(&connList[connIndex].throughputObject[i], '\0', sizeof(throughputProfileHdl_t));
      }

      pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

      if(BOARD_DISPLAY_USE_LCD == 1)
      {
        Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "To: %s", pStrAddr);
      }
      else
      {
        Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "Connected to %s", pStrAddr);
        Display_printf(dispHandle, TC_ROW_NUM_CONN, 0, "Num Conns: %d", numConn);
      }

      // If we already have maximum allowed number of connections,
      // disable device discovery and additional connection making.
      if (numConn >= MAX_NUM_BLE_CONNS)
      {
        itemsToDisable |= TC_ITEM_SCANPHY | TC_ITEM_STARTDISC | TC_ITEM_CONNECT;
      }

      for (i = 0; i < TBM_GET_NUM_ITEM(&tcMenuConnect); i++)
      {
        if (!memcmp(TBM_GET_ACTION_DESC(&tcMenuConnect, i), pStrAddr,
            TC_ADDR_STR_SIZE))
        {
          // Disable this device from the connection choices
          tbm_setItemStatus(&tcMenuConnect, TC_ITEM_NONE, 1 << i);
        }
        else if (TBM_IS_ITEM_ACTIVE(&tcMenuConnect, i))
        {
          numConnectable++;
        }
      }

      if (numConnectable == 0)
      {
        // Disable "Connect To" since there is no device possible to connect
        itemsToDisable |= TC_ITEM_CONNECT;
      }

      // Enable/disable Main menu items properly
      tbm_setItemStatus(&tcMenuMain,
                       TC_ITEM_ALL & ~(itemsToDisable), itemsToDisable);
      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      uint16_t connHandle = ((gapTerminateLinkEvent_t*) pMsg)->connectionHandle;
      uint8_t connIndex;
      uint32_t itemsToEnable = TC_ITEM_STARTDISC | TC_ITEM_SCANPHY;
      uint8_t* pStrAddr;
      uint8_t i;
      uint8_t numConnectable = 0;

      // Stop RSSI
      ThroughputCentral_StopRssi(connHandle);

      // Mark this connection deleted in the connected device list.
      connIndex = ThroughputCentral_removeConnInfo(connHandle);


      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      THROUGHPUTCENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

      Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "%s is disconnected",
                     pStrAddr);
      Display_printf(dispHandle, TC_ROW_NUM_CONN, 0, "Num Conns: %d", numConn);

      Display_clearLines(dispHandle, TC_ROW_PEER_ADDR, TC_ROW_NUM_PACKETS_LOST);

      for (i = 0; i < TBM_GET_NUM_ITEM(&tcMenuConnect); i++)
      {
        if (!memcmp(TBM_GET_ACTION_DESC(&tcMenuConnect, i), pStrAddr,
                     TC_ADDR_STR_SIZE))
        {
          // Enable this device in the connection choices
          tbm_setItemStatus(&tcMenuConnect, 1 << i, TC_ITEM_NONE);
        }

        if (TBM_IS_ITEM_ACTIVE(&tcMenuConnect, i))
        {
          numConnectable++;
        }
      }

      if (numConnectable > 0)
      {
        // Enable "Connect To" since there are device(s) possible to connect
        itemsToEnable |= TC_ITEM_CONNECT;
      }

      if (numConn > 0)
      {
        // There still is an active connection to select
        itemsToEnable |= TC_ITEM_SELECTCONN;
      }

      // Enable/disable items properly.
      tbm_setItemStatus(&tcMenuMain,
                        itemsToEnable, TC_ITEM_ALL & ~itemsToEnable);

      // If we are in the context which the terminated connection was associated
      // with, go to main menu.
      if (connHandle == tcConnHandle)
      {
        tbm_goTo(&tcMenuMain);
      }

      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
      if (acceptParamUpdateReq)
      {
        gapUpdateLinkParamReqReply_t rsp;
        gapUpdateLinkParamReq_t *pReq;

        pReq = &((gapUpdateLinkParamReqEvent_t *)pMsg)->req;

        rsp.connectionHandle = pReq->connectionHandle;
        rsp.intervalMin = pReq->intervalMin;
        rsp.intervalMax = pReq->intervalMax;
        rsp.connLatency = pReq->connLatency;
        rsp.connTimeout = pReq->connTimeout;
        rsp.accepted = TRUE;

        // Send application's requested parameters back.
        VOID GAP_UpdateLinkParamReqReply(&rsp);
      }
      else
      {
        gapUpdateLinkParamReqReply_t rsp;
        gapUpdateLinkParamReq_t *pReq;

        pReq = &((gapUpdateLinkParamReqEvent_t *)pMsg)->req;

        rsp.connectionHandle = pReq->connectionHandle;
        rsp.accepted = FALSE;

        // Reject the request.
        VOID GAP_UpdateLinkParamReqReply(&rsp);
      }

      break;

     case GAP_LINK_PARAM_UPDATE_EVENT:
     {
       gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

        // Get the address from the connection handle
        linkDBInfo_t linkInfo;
        if (linkDB_GetInfo(pPkt->connectionHandle, &linkInfo) ==  SUCCESS)
        {

          if(pPkt->status == SUCCESS)
          {
            Display_printf(dispHandle, TC_ROW_CUR_CONN, 0,
                          "Updated: %s, connInterval: %dms",
                           Util_convertBdAddr2Str(linkInfo.addr),
                           (linkInfo.connInterval * 125) / 100);

          }
          else
          {
            // Display the address of the connection update failure
            Display_printf(dispHandle, TC_ROW_CUR_CONN, 0,
                           "Update Failed 0x%h: %s", pPkt->opcode,
                           Util_convertBdAddr2Str(linkInfo.addr));
          }
        }
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @param   *pMsg - pointer to the GATT message
 *
 * @return  none
 */
static void ThroughputCentral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (linkDB_Up(pMsg->connHandle))
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->method == ATT_HANDLE_VALUE_NOTI)
    {
      uint8_t connIndex = ThroughputCentral_getConnIndex(pMsg->connHandle);

      // Critial Section so our Timer's SWI can't read the value while
      // we're writing to it.
      UInt key = Hwi_disable();
      {
        connList[connIndex].metrics.bytesRecvd += pMsg->msg.handleValueNoti.len;
      }
      Hwi_restore(key);
      GATT_bm_free(&pMsg->msg, pMsg->method);

    }
    else if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      Display_printf(dispHandle, TC_ROW_CUR_CONN, 0,
                     "ATT Rsp dropped %d", pMsg->method);
    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_printf(dispHandle, TC_ROW_CUR_CONN, 0,
                       "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Display_printf(dispHandle, TC_ROW_CUR_CONN, 0,
                       "Read rsp: 0x%02x", pMsg->msg.readRsp.pValue[0]);
      }
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_printf(dispHandle, TC_ROW_CUR_CONN, 0,
                       "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      // Ignore a successful write response
      tbm_goTo(&tcMenuPerConn);
    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.

      // Display the opcode of the message that caused the violation.
      Display_printf(dispHandle, TC_ROW_CUR_CONN, 0,
                     "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      // MTU size updated
      Display_printf(dispHandle, TC_ROW_ANY_CONN, 0,
                     "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      ThroughputCentral_processGATTDiscEvent(pMsg);
    }
  } 
  // else - in case a GATT message came after a connection has dropped, ignore it.

  // Needed only for ATT Protocol messages
  if(pMsg)
  {
      GATT_bm_free(&pMsg->msg, pMsg->method);
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_processAppMsg
 *
 * @brief   Scanner application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void ThroughputCentral_processAppMsg(tcEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case TC_EVT_KEY_CHANGE:
      ThroughputCentral_handleKeys(pMsg->hdr.state);
      break;

    case TC_EVT_ADV_REPORT:
    {
      GapScan_Evt_AdvRpt_t* pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      if (ThroughputCentral_findSvcUuid(THROUGHPUT_SERVICE_SERV_UUID, pAdvRpt->pData, pAdvRpt->dataLen))
      {
        ThroughputCentral_addScanInfo(pAdvRpt->addr, pAdvRpt->addrType);
        Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "Discovered: %s", Util_convertBdAddr2Str(pAdvRpt->addr));
      }
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
      Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "Discovered: %s", Util_convertBdAddr2Str(pAdvRpt->addr));
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
      if (pAdvRpt->pData != NULL)
      {
        ICall_free(pAdvRpt->pData);
      }
      break;
    }

    case TC_EVT_SCAN_ENABLED:
      // Disable everything but "Stop Discovering" on the menu
      tbm_setItemStatus(&tcMenuMain, TC_ITEM_STOPDISC,
                        (TC_ITEM_ALL & ~TC_ITEM_STOPDISC));
      Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "Discovering...");
      break;

    case TC_EVT_SCAN_DISABLED:
    {
      uint8_t numReport;
      uint8_t i;
      static uint8_t* pAddrs = NULL;
      uint8_t* pAddrTemp;
      uint16_t itemsToEnable = TC_ITEM_STARTDISC | TC_ITEM_SCANPHY;
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      numReport = numScanRes;
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
      GapScan_Evt_AdvRpt_t advRpt;

      numReport = ((GapScan_Evt_End_t*) (pMsg->pData))->numReport;
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
      ICall_free(pMsg->pData);
      Display_printf(dispHandle, TC_ROW_NON_CONN, 0,
                     "%d devices discovered", numReport);

      if (numReport > 0)
      {
        // Also enable "Connect to"
        itemsToEnable |= TC_ITEM_CONNECT;
      }

      if (numConn > 0)
      {
        // Also enable "Work with"
        itemsToEnable |= TC_ITEM_SELECTCONN;
      }

      // Enable "Discover Devices", "Set Scanning PHY", and possibly
      // "Connect to" and/or "Work with".
      // Disable "Stop Discovering".
      tbm_setItemStatus(&tcMenuMain, itemsToEnable, TC_ITEM_STOPDISC);

      if (pAddrs != NULL)
      {
        ICall_free(pAddrs);
      }

      // Allocate buffer to display addresses
      pAddrs = ICall_malloc(numReport * TC_ADDR_STR_SIZE);

      if (pAddrs == NULL)
      {
        numReport = 0;
      }

      TBM_SET_NUM_ITEM(&tcMenuConnect, numReport);

      if (pAddrs != NULL)
      {
      pAddrTemp = pAddrs;
      for (i = 0; i < numReport; i++, pAddrTemp += TC_ADDR_STR_SIZE)
      {
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        // Get the address from the list, convert it to string, and
        // copy the string to the address buffer
        memcpy(pAddrTemp, Util_convertBdAddr2Str(scanList[i].addr),
               TC_ADDR_STR_SIZE);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
        // Get the address from the report, convert it to string, and
        // copy the string to the address buffer
        GapScan_getAdvReport(i, &advRpt);
        memcpy(pAddrTemp, Util_convertBdAddr2Str(advRpt.addr),
               TC_ADDR_STR_SIZE);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

        // Assign the string to the corresponding action description of the menu
        TBM_SET_ACTION_DESC(&tcMenuConnect, i, pAddrTemp);
      }

        //ICall_free(pAddrs);
      }
      break;
    }

    case TC_EVT_SVC_DISC:
      ThroughputCentral_startSvcDiscovery();
      break;

    case TC_EVT_READ_RSSI:
    {
        uint8_t connIndex = pMsg->hdr.state;
        uint16_t connHandle = connList[connIndex].connHandle;
        // If link is still valid
        if (connHandle != LINKDB_CONNHANDLE_INVALID)
        {
          // Restart timer
          Util_startClock(connList[connIndex].pRssiClock);

          // Read RSSI
          VOID HCI_ReadRssiCmd(connHandle);
        }

        break;
    }
    // Pairing event
    case TC_EVT_PAIR_STATE:
    {
      ThroughputCentral_processPairState(pMsg->hdr.state, (tcPairStateData_t*) (pMsg->pData));

      ICall_free(pMsg->pData);
      break;
    }

    case TC_EVT_PASSCODE_NEEDED:
    {
      ThroughputCentral_processPasscode(tcConnHandle, *pMsg->pData);
      ICall_free(pMsg->pData);
      break;
    }

    case TC_EVT_TOGGLE_THROUGHPUT:
    {
      ThroughputCentral_processThroughput((uint16_t)*pMsg->pData);
      ICall_free(pMsg->pData);
      break;
    }

    case TC_EVT_PDU_UPDATE:
    {
      ThroughputCentral_processPDUUpdate((uint16_t)*pMsg->pData);
      ICall_free(pMsg->pData);
      break;
    }

    case TC_EVT_PHY_UPDATE:
    {
      ThroughputCentral_processPHYUpdate((uint16_t)*pMsg->pData);
      ICall_free(pMsg->pData);
      break;
    }

    case TC_EVT_MEASURE_INST_SPEED_EVT:
    {
      ThroughputCentral_processInstantSpeed((connRec_t *)(pMsg->pData));
      break;
    }
    case TC_EVT_MEASURE_AVG_SPEED_EVT:
    {
      ThroughputCentral_processAverageSpeed((connRec_t *)(pMsg->pData));
      break;
    }

    case TC_EVT_READ_RPA:
    {
      // Read the current RPA.
      // The parameters for the call to HCI_LE_ReadLocalResolvableAddressCmd
      // are not needed to be accurate to retrieve the local resolvable address.
      // The 1st parameter can be any of ADDRMODE_PUBLIC and ADDRMODE_RANDOM.
      // The 2nd parameter only has to be not NULL.
      // The result will come with HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS
      // complete event.
      //HCI_LE_ReadLocalResolvableAddressCmd(0, rpa);

      if (HCI_LE_ReadLocalResolvableAddressCmd(0, rpa) == SUCCESS)
      {
        Display_printf(dispHandle, TC_ROW_RPA, 0, "RP Addr: %s", Util_convertBdAddr2Str(rpa));
      }
      break;
    }


    // Insufficient memory
    case TC_EVT_INSUFFICIENT_MEM:
    {
      // We are running out of memory.
      Display_printf(dispHandle, TC_ROW_ANY_CONN, 0, "Insufficient Memory");

      // We might be in the middle of scanning, try stopping it.
      GapScan_disable();
      break;
    }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @param   Pointer to the GATT message
 *
 * @return  none
 */
static void ThroughputCentral_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  status_t ret;

  if (discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover throughput service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_UUID_SIZE] =     { TI_BASE_UUID_128(THROUGHPUT_SERVICE_SERV_UUID) };

      discState = BLE_DISC_STATE_SVC;

      // Discovery throughput service
      ret = GATT_DiscPrimaryServiceByUUID(pMsg->connHandle, uuid,
                                         ATT_UUID_SIZE, selfEntity);
      if (ret != SUCCESS)
      {
          while (1);
      }
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
        // The Throughput Service Exists!
        // At this point svcStartHdl and svcEndHdl are valid
        // A write can be performed if needed to a particular UUID
        // Defined in the profile's header file
        // Further discovery is needed to determine characteristic value handles

        // Set Statemachine to parse ATT_READ_BY_TYPE_RSP
        discState = BLE_DISC_STATE_CHAR;

        // Find all characteristics within our throughput service
        VOID GATT_DiscAllChars(pMsg->connHandle, svcStartHdl, svcEndHdl, selfEntity);
      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristics found, store handles
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
      uint8_t connIndex = ThroughputCentral_getConnIndex(pMsg->connHandle);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      THROUGHPUTCENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      // This will ONLY work if you have up to NUM_THROUGHPUT_CHARS chars in the service.
      int i = 0;
      for (i = 0; i < pMsg->msg.readByTypeRsp.numPairs; i++)
      {
          // Due to the difference between the structure and the data given, apply some logic to
          // extract out the correct information (i.e, ignore the 3 byte header)
          throughputProfileHdl_t* temp = (throughputProfileHdl_t*)((pMsg->msg.readByTypeRsp.pDataList + 3) + (pMsg->msg.readByTypeRsp.len * i));

          connList[connIndex].throughputObject[i] = *temp;
      }
      // Now verify that the UUIDs are in the order the indexes are
      // We'll skip this step, as the way we implemented the profile on
      // the peripheral always responds with PDU first then PHY characteristics

      // This means we can index throughputHandles like throughputHandles[THROUGHPUT_SERVICE_UPDATE_PDU]

      // Inform user that the Throughput Service is found, and ready to use
      Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Throughput Service Found, Setting PDU and PHY");

      // Inform Application to set PDU
      uint8_t *pData;

      // Allocate space for the PDU value
      if ((pData = ICall_malloc(sizeof(uint8_t))))
      {
        *pData = connList[connIndex].pduValue;

        // Enqueue the event.
        if(ThroughputCentral_enqueueMsg(TC_EVT_PDU_UPDATE, SUCCESS, pData) != SUCCESS)
        {
          ICall_freeMsg(pData);
        }
      }
      // Start RSSI, connection should be stable now
      ThroughputCentral_StartRssi();
    }

    discState = BLE_DISC_STATE_IDLE;
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_startSvcDiscovery
 *
 * @brief   Start service discovery.
 *
 * @param   none
 *
 * @return  none
 */
static void ThroughputCentral_startSvcDiscovery(void)
{
  status_t ret;

  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = 0;

  // Prep the State Machine for MTU Exchange
  discState = BLE_DISC_STATE_MTU;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = tcMaxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  ret = GATT_ExchangeMTU(tcConnHandle, &req, selfEntity);
  if (ret != SUCCESS)
  {
      Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Error exchanging MTU (ret 0x%x)", ret);
  }
}

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
/*********************************************************************
 * @fn      ThroughputCentral_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @param   uuid    - the UUID of the service to find.
 *          *pData  - pointer to the data to search in, usually and
 *          advertisement report.
 *          dataLen - length of the data to search in.
 *
 * @return  TRUE if service UUID found
 */
static bool ThroughputCentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
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
 * @fn      ThroughputCentral_addScanInfo
 *
 * @brief   Add a device to the scanned device list
 *
 * @param   *pAddr   - pointer to the address of the device
 *          addrType - what type of address pAddr is:
 *          Public, random, public ID, random ID, or anonymous
 *
 * @return  none
 */
static void ThroughputCentral_addScanInfo(uint8_t *pAddr, uint8_t addrType)
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
 * @fn      ThroughputCentral_addConnInfo
 *
 * @brief   Add a device to the connected device list
 *
 * @param   connHandle - connection handle of the connection to add
 *          to the connection list.
 *          *pAddr     - pointer to the device address
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t ThroughputCentral_addConnInfo(uint16_t connHandle, uint8_t *pAddr)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;
      memcpy(connList[i].addr, pAddr, B_ADDR_LEN);
      numConn++;

      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      ThroughputCentral_removeConnInfo
 *
 * @brief   Remove a device from the connected device list and reset
 *          the connection list parameters of the device
 *
 * @param   connHandle - connection handle of the connection to remove
 *          from the connection list.
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t ThroughputCentral_removeConnInfo(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      // Found the entry to mark as deleted
      connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
      connList[i].pRssiClock = NULL;
      connList[i].throughputToggle = 0;
      connList[i].phyIndex = TS_PHY_1M;
      connList[i].phyOptions = LL_PHY_OPT_NONE;
      connList[i].pduValue = DEFAULT_PDU_SIZE;
      connList[i].establishingConn = TRUE;
      connList[i].metrics.cbBufferFilled = FALSE;
      memset(connList[i].metrics.bytesRecvd_cb, 0, sizeof(connList[i].metrics.bytesRecvd_cb));
      numConn--;

      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      ThroughputCentral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @param   connHandle - connection handle of the peer to get the
 *                       index for
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t ThroughputCentral_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      ThroughputCentral_getConnAddrStr
 *
 * @brief   Return, in string form, the address of the peer associated with
 *          the connHandle.
 *
 * @param   connHandle - connection handle for the peer to get the address
 *                       for
 *
 * @return  A null-terminated string of the address.
 *          if there is no match, NULL will be returned.
 */
static char* ThroughputCentral_getConnAddrStr(uint16_t connHandle)
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

/*********************************************************************
 * @fn      ThroughputCentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @param   *pPairData - pointer to the passcode data from the pairing
 *                       process
 *
 * @return  none
 */
static void ThroughputCentral_processPairState(uint8_t state, tcPairStateData_t* pPairData)
{
  uint8_t status = pPairData->status;
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      linkDBInfo_t linkInfo;

      Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Pairing success");

      if (linkDB_GetInfo(pPairData->connHandle, &linkInfo) == SUCCESS)
      {
        // If the peer was using private address, update with ID address
        if ((linkInfo.addrType == ADDRTYPE_PUBLIC_ID ||
             linkInfo.addrType == ADDRTYPE_RANDOM_ID) &&
             !Util_isBufSet(linkInfo.addrPriv, 0, B_ADDR_LEN))
        {
          // Update the address of the peer to the ID address
          Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "Addr updated: %s",
                         Util_convertBdAddr2Str(linkInfo.addr));

          // Update the connection list with the ID address
          uint8_t i = ThroughputCentral_getConnIndex(pPairData->connHandle);

          THROUGHPUTCENTRAL_ASSERT(i < MAX_NUM_BLE_CONNS);
          memcpy(connList[i].addr, linkInfo.addr, B_ADDR_LEN);
        }
      }
    }
    else
    {
      Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_ENCRYPTED)
  {
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Encryption success");
    }
    else
    {
      Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Encryption failed: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Bond save success");
    }
    else
    {
      Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @param   connHandle - connection handle of the peer in the passcode
 *                       process
 * @param   uiOutputs  - if TRUE, display the passcode
 *
 * @return  none
 */
static void ThroughputCentral_processPasscode(uint16_t connHandle,
                                          uint8_t uiOutputs)
{
  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Passcode: %d",
                   B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connHandle , SUCCESS, B_APP_DEFAULT_PASSCODE);
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
static void ThroughputCentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
      uint16_t connHandle = BUILD_UINT16(pMsg->pReturnParam[1],
                                         pMsg->pReturnParam[2]);
      int8 rssi = (int8)pMsg->pReturnParam[3];

      if(BOARD_DISPLAY_USE_LCD == 1)
      {
        Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "RSSI: -%d dBm", -rssi);
      }
      else
      {
        Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "%s: RSSI -%d dBm",
                     ThroughputCentral_getConnAddrStr(connHandle), -rssi);
      }
      break;
    }

    case HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS:
    {
      uint8_t* pRpaNew = &(pMsg->pReturnParam[1]);

      if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
      {
        // If the RPA has changed, update the display
        Display_printf(dispHandle, TC_ROW_RPA, 0, "RP Addr: %s",
                       Util_convertBdAddr2Str(pRpaNew));
        memcpy(rpa, pRpaNew, B_ADDR_LEN);
      }
      break;
    }


    default:
      break;
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_StartRssi
 *
 * @brief   Start periodic RSSI reads on the current link.
 *
 * @param   none
 *
 * @return  SUCCESS: RSSI Read timer started
 *          bleIncorrectMode: Already started
 *          bleNoResources: No resources
 */
static status_t ThroughputCentral_StartRssi(void)
{
  uint8_t connIndex = ThroughputCentral_getConnIndex(tcConnHandle);

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  THROUGHPUTCENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // If already running
  if (connList[connIndex].pRssiClock != NULL)
  {
    return bleIncorrectMode;
  }

  // Create a clock object and start
  connList[connIndex].pRssiClock
    = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

  if (connList[connIndex].pRssiClock)
  {
    Util_constructClock(connList[connIndex].pRssiClock,
                        ThroughputCentral_clockHandler,
                        DEFAULT_RSSI_PERIOD, 0, true,
                        (connIndex << 8) | TC_EVT_READ_RSSI);
  }
  else
  {
    return bleNoResources;
  }

  return SUCCESS;
}

/*********************************************************************
 * @fn      ThroughputCentral_StopRssi
 *
 * @brief   Stop periodic RSSI reads on a link.
 *
 * @param   connection handle
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: Has not started
 */
static status_t ThroughputCentral_StopRssi(uint16_t connHandle)
{
  uint8_t connIndex = ThroughputCentral_getConnIndex(connHandle);

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  THROUGHPUTCENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // If already terminated
  if (connList[connIndex].pRssiClock == NULL)
  {
    return bleIncorrectMode;
  }

  // Stop timer
  Util_stopClock(connList[connIndex].pRssiClock);

  // Destroy the clock object
  Clock_destruct(connList[connIndex].pRssiClock);

  // Free clock struct
  ICall_free(connList[connIndex].pRssiClock);
  connList[connIndex].pRssiClock = NULL;

  Display_clearLine(dispHandle, TC_ROW_ANY_CONN);

  return SUCCESS;
}

/*********************************************************************
 * @fn      ThroughputCentral_passcodeCb
 *
 * @brief   Passcode callback, enqueues a TC_EVT_PASSCODE_NEEDED message
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
static void ThroughputCentral_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs,
                                     uint32_t numComparison)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    if(ThroughputCentral_enqueueMsg(TC_EVT_PASSCODE_NEEDED, 0, pData) != SUCCESS)
    {
      ICall_freeMsg(pData);
    }
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle of the peer
 * @param   state      - state of the pairing process
 * @param   status     - status of the pairing process
 *
 * @return  none
 */
static void ThroughputCentral_pairStateCb(uint16_t connHandle, uint8_t state,
                                      uint8_t status)
{
  tcPairStateData_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(tcPairStateData_t))))
  {
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(ThroughputCentral_enqueueMsg(TC_EVT_PAIR_STATE, state, (uint8_t*) pData) != SUCCESS)
    {
      ICall_freeMsg(pData);
    }
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 *
 * @return  none
 */
static void ThroughputCentral_keyChangeHandler(uint8 keys)
{
  ThroughputCentral_enqueueMsg(TC_EVT_KEY_CHANGE, keys, NULL);
}


/*********************************************************************
 * @fn      ThroughputCentral_clockHandler
 *
 * @brief   clock handler function
 *
 * @param   arg - argument from the clock initiator
 *
 * @return  none
 */
static void ThroughputCentral_clockHandler(UArg arg)
{
  uint8_t evtId = (uint8_t) (arg & 0xFF);

  switch (evtId)
  {
    case TC_EVT_READ_RPA:
      // Restart timer
      Util_startClock(&clkRpaRead);
      // Let the application handle the event
      ThroughputCentral_enqueueMsg(TC_EVT_READ_RPA, 0, NULL);
      break;

    case TC_EVT_READ_RSSI:
      ThroughputCentral_enqueueMsg(TC_EVT_READ_RSSI, (uint8_t) (arg >> 8) , NULL);
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_speedHandler
 *
 * @brief   RTOS clock handler that counts number of bytes received
 *
 * @param   a0 - RTOS clock arg0, not used.
 *
 * @return  void
 */
static void ThroughputCentral_speedHandler(UArg a0)
{
  for (int i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if ((connList[i].connHandle != 0xFFFF) && connList[i].throughputToggle)
    {
        ThroughputCentral_processMetric(&connList[i]);
    }
  }
}


/*********************************************************************
 * @fn      ThroughputCentral_PHYHandler
 *
 * @brief   RTOS clock handler for Coded PHY changes
 *
 * @param   a0 - RTOS clock argument, not used.
 *
 * @return  void
 */
static void ThroughputCentral_PHYHandler(UArg a0)
{
  uint8_t connIndex = ThroughputCentral_getConnIndex(tcConnHandle);
  THROUGHPUTCENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Check if we've changed to 1M before changing to the requested PHY
  if(connList[connIndex].phyIndex == TS_PHY_1M && connList[connIndex].phyConfirm)
  {
    // Because we are in a SWI, the UART Driver should not be used
    // Inform the Application task to send request
      uint8_t *pData;

      // Allocate space for the PHY change.
      if ((pData = ICall_malloc(sizeof(uint8_t))))
      {
        *pData = connList[connIndex].phyClock_phyIndex;

        // Enqueue the event.
        if(ThroughputCentral_enqueueMsg(TC_EVT_PHY_UPDATE, SUCCESS, pData) != SUCCESS)
        {
          ICall_freeMsg(pData);
        }
      }
  }
  else
  {
    // We're still trying to get to a coded PHY
    // Restart the timer
    Util_restartClock(&startPHYClock, CODED_PHY_CHANGE_DELAY);
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   *pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static status_t ThroughputCentral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  uint8_t success;
  tcEvt_t *pMsg = ICall_malloc(sizeof(tcEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
    return(success) ? SUCCESS : FAILURE;
  }

  return (bleMemAllocError);
}

/*********************************************************************
 * @fn      ThroughputCentral_scanCb
 *
 * @brief   Callback called by GapScan module
 *
 * @param   evt - event
 * @param   msg - message coming with the event
 * @param   arg - user argument
 *
 * @return  none
 */
static void ThroughputCentral_scanCb(uint32_t evt, void* pMsg, uintptr_t arg)
{
 uint8_t event;

  if (evt & GAP_EVT_ADV_REPORT)
  {
    event = TC_EVT_ADV_REPORT;
  }
  else if (evt & GAP_EVT_SCAN_ENABLED)
  {
    event = TC_EVT_SCAN_ENABLED;
  }
  else if (evt & GAP_EVT_SCAN_DISABLED)
  {
    event = TC_EVT_SCAN_DISABLED;
  }
  else if (evt & GAP_EVT_INSUFFICIENT_MEMORY)
  {
    event = TC_EVT_INSUFFICIENT_MEM;
  }
  else
  {
    return;
  }

  if(ThroughputCentral_enqueueMsg(event, SUCCESS, pMsg) != SUCCESS)
  {
    ICall_free(pMsg);
  }
}

/*********************************************************************
 * @fn      ThroughputCentral_menuSwitchCb
 *
 * @brief   Detect menu context switching
 *
 * @param   pMenuObjCurr - the current menu object
 * @param   pMenuObjNext - the menu object the context is about to switch to
 *
 * @return  none
 */
static void ThroughputCentral_menuSwitchCb(tbmMenuObj_t* pMenuObjCurr,
                                       tbmMenuObj_t* pMenuObjNext)
{
  // interested in only the events of
  // entering tcMenuConnect, tcMenuSelectConn, and tcMenuMain for now
  if (pMenuObjNext == &tcMenuConnect)
  {
    uint8_t i, j;
    uint32_t itemsToDisable = TC_ITEM_NONE;

    for (i = 0; i < TBM_GET_NUM_ITEM(&tcMenuConnect); i++)
    {
      for (j = 0; j < MAX_NUM_BLE_CONNS; j++)
      {
        if ((connList[j].connHandle != LINKDB_CONNHANDLE_INVALID) &&
            !memcmp(TBM_GET_ACTION_DESC(&tcMenuConnect, i),
                    Util_convertBdAddr2Str(connList[j].addr),
                    TC_ADDR_STR_SIZE))
        {
          // Already connected. Add to the set to be disabled.
          itemsToDisable |= (1 << i);
        }
      }
    }

    // Eventually only non-connected device addresses will be displayed.
    tbm_setItemStatus(&tcMenuConnect,
                      TC_ITEM_ALL & ~itemsToDisable, itemsToDisable);
  }
  else if (pMenuObjNext == &tcMenuSelectConn)
  {
    static uint8_t* pAddrs;
    uint8_t* pAddrTemp;

    if (pAddrs != NULL)
    {
      ICall_free(pAddrs);
    }

    // Allocate buffer to display addresses
    pAddrs = ICall_malloc(numConn * TC_ADDR_STR_SIZE);

    if (pAddrs == NULL)
    {
      TBM_SET_NUM_ITEM(&tcMenuSelectConn, 0);
    }
    else
    {
      uint8_t i;

      TBM_SET_NUM_ITEM(&tcMenuSelectConn, MAX_NUM_BLE_CONNS);

      pAddrTemp = pAddrs;

      // Add active connection info to the menu object
      for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
      {
        if (connList[i].connHandle != LINKDB_CONNHANDLE_INVALID)
        {
          // This connection is active. Set the corresponding menu item with
          // the address of this connection and enable the item.
          memcpy(pAddrTemp, Util_convertBdAddr2Str(connList[i].addr),
                 TC_ADDR_STR_SIZE);
          TBM_SET_ACTION_DESC(&tcMenuSelectConn, i, pAddrTemp);
          tbm_setItemStatus(&tcMenuSelectConn, (1 << i), TC_ITEM_NONE);
          pAddrTemp += TC_ADDR_STR_SIZE;
        }
        else
        {
          // This connection is not active. Disable the corresponding menu item.
          tbm_setItemStatus(&tcMenuSelectConn, TC_ITEM_NONE, (1 << i));
        }
      }
    }
  }
  else if (pMenuObjNext == &tcMenuMain)
  {
    // Now we are not in a specific connection's context
    tcConnHandle = LINKDB_CONNHANDLE_INVALID;

    // Clear connection-related message
    Display_clearLine(dispHandle, TC_ROW_CUR_CONN);
  }
}
/*********************************************************************
 * @fn      ThroughputCentral_processInstantSpeed
 *
 * @brief   Function called by a TC_EVT_MEASURE_INST_SPEED_EVT to calculate
 *          the instant throughput speed based on a bytesRecvd, which is
 *          updated with the length of the received ATT_HANDLE_VALUE_NOTI
 *          message.
 *
 * @param   *connection - Pointer to the connection to calculate
 *          the instant speed of.
 *
 * @return  none
 */
static void ThroughputCentral_processInstantSpeed(connRec_t *connection)
{
  uint8_t connIndex = ThroughputCentral_getConnIndex(connection->connHandle);
  uint8_t* pStrAddr;
  uint8_t prev_index;
  pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

  prev_index = connection->metrics.bytesRecvd_cb_index == 0 ? CB_SIZE - 1 : connection->metrics.bytesRecvd_cb_index - 1;

  // Note at this point, Bytes have been recorded and index has been incremented by one
  uint32_t bitsReceived = connection->metrics.bytesRecvd_cb[prev_index];

  // Convert Bytes to bits
  bitsReceived = 8*bitsReceived;

  // Display peer address
  Display_print2(dispHandle,
                 TC_ROW_PEER_ADDR,
                 0,
                 "Connection #%d - %s: ",
                connIndex + 1, pStrAddr);

  // Display Throughput
  if(BOARD_DISPLAY_USE_LCD == 1)
  {
    Display_print2(dispHandle,
                   TC_ROW_NUM_CONN,
                   0,
                   "Inst %d.%dkb/s",
                   (bitsReceived/1000),(bitsReceived % 1000));
  }
  else
  {
    Display_print2(dispHandle,
                   TC_ROW_THROUGHPUT_INST,
                   0,
                   "Instant Rate (kb/s): %d.%d",
                   (bitsReceived/1000),(bitsReceived % 1000));
  }
  connection->metrics.instantRate = (bitsReceived/1000);
}

/*********************************************************************
 * @fn      ThroughputCentral_processAverageSpeed
 *
 * @brief   Function called by a TC_EVT_MEASURE_AVG_SPEED_EVT to calculate
 *          the average throughput speed based on a circular data buffer
 *          for received notification data. The event is called by
 *          ThroughputCentral_processMetric.
 *
 * @param   *connection - Pointer to the connection to calculate
 *          the average speed of.
 *
 * @return  none
 */
static void ThroughputCentral_processAverageSpeed(connRec_t *connection)
{
    // local vars
    uint32_t bitsReceived = 0;
    int i;
    uint8_t connIndex = ThroughputCentral_getConnIndex(connection->connHandle);

    // Determine Average from Circular Buffer
    if (connection->metrics.cbBufferFilled)
    {
      // Average using Total Buffer Size
      for (i = 0; i < CB_SIZE; i++)
      {
        bitsReceived += connection->metrics.bytesRecvd_cb[i];
      }
      bitsReceived = bitsReceived/CB_SIZE;
    }
    else
    {
      // Average using Running Buffer Size (prior to buffer being filled)
      for (i = 0; i < connection->metrics.bytesRecvd_cb_index; i++)
      {
        bitsReceived += connection->metrics.bytesRecvd_cb[i];
      }
      bitsReceived = bitsReceived/(connection->metrics.bytesRecvd_cb_index);
    }
    // Convert to bits
    bitsReceived = 8*bitsReceived;

    // Display Throughput
    if(BOARD_DISPLAY_USE_LCD == 1)
    {
      Display_print2(dispHandle,
                   TC_ROW_IDA,
                   0,
                   "Avg  %d.%dkb/s",
                   (bitsReceived/1000),(bitsReceived % 1000));
    }
    else
    {
      Display_print3(dispHandle,
                   TC_ROW_THROUGHPUT_AVG,
                   0,
                   "Average Rate (kb/s): %d.%d over %d Samples",
                   (bitsReceived/1000),(bitsReceived % 1000), CB_SIZE);
    }
    connection->metrics.averageRate = (bitsReceived/1000);
}

/*********************************************************************
 * @fn      ThroughputCentral_processMetric
 *
 * @brief   Process instant and average throughput speeds. Resets the counter for
 *          the number of bytes received the previous second and increments the
 *          circular buffer index
 *
 * @param   *connection - the connection for which to calculate average and instant throughput
 *
 * @return  none
 */
static void ThroughputCentral_processMetric(connRec_t *connection)
{
  // Place Bytes Recieved into Circular Buffer
  connection->metrics.bytesRecvd_cb[connection->metrics.bytesRecvd_cb_index] = connection->metrics.bytesRecvd;

  // Update Instantanous Throughput
  ThroughputCentral_enqueueMsg(TC_EVT_MEASURE_INST_SPEED_EVT, SUCCESS, (uint8_t *)connection);

  // Update Average Throughput
  ThroughputCentral_enqueueMsg(TC_EVT_MEASURE_AVG_SPEED_EVT, SUCCESS, (uint8_t *)connection);

  // Calculate next Index + Update Rolling Average
  connection->metrics.bytesRecvd_cb_index++;
  // Reset the count
  connection->metrics.bytesRecvd = 0;
  if ( connection->metrics.bytesRecvd_cb_index >= CB_SIZE )
  {
    // Wrap the index back to the head
    connection->metrics.bytesRecvd_cb_index = 0;

    // Indicate that the buffer is now filled
    connection->metrics.cbBufferFilled = true;
  }

}

/*********************************************************************
 * @fn      ThroughputCentral_processThroughput
 *
 * @brief   Process throughpt update and generate a characteristic
 *          write request to send to the peripheral
 *
 * @param   connHandle - connection handle of the peer
 *
 * @return  none
 */
static void ThroughputCentral_processThroughput(uint16_t connHandle)
{
    // Variables Needed for Write
    attWriteReq_t writeReq;
    uint8_t connIndex = ThroughputCentral_getConnIndex(connHandle);
    uint8_t tempToggleVal = !(connList[connIndex].throughputToggle);
    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    THROUGHPUTCENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    // Populate the Request Structure
    writeReq.cmd = 0;
    writeReq.handle = connList[connIndex].throughputObject[THROUGHPUT_SERVICE_TOGGLE_THROUGHPUT].charHandle;
    writeReq.len = THROUGHPUT_SERVICE_TOGGLE_THROUGHPUT_LEN;
    writeReq.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, THROUGHPUT_SERVICE_TOGGLE_THROUGHPUT_LEN, NULL);
    memcpy(writeReq.pValue, &tempToggleVal, THROUGHPUT_SERVICE_TOGGLE_THROUGHPUT_LEN);
    writeReq.sig = 0;

    // Perform a GATT Write + Check Status
    uint8_t status;

    status = GATT_WriteCharValue(connHandle, &writeReq, selfEntity);

    if( status != SUCCESS )
    {
      // We didn't successfully send this command to the stack!
      // Let's attempt to retransmit again and free the pValue pointer

      GATT_bm_free((gattMsg_t *)&writeReq, ATT_WRITE_REQ);

      uint8_t *pData;
      if ((pData = ICall_malloc(sizeof(uint8_t))))
      {
        *pData = connHandle;
        // Start a clock to perform a delayed requeue of the event
        Clock_setFunc(Clock_handle(&delayedTrigger), ThroughputCentral_delayedThroughputTriggerFxn, (UArg)pData);
        Clock_start(Clock_handle(&delayedTrigger));
      }
    }
    else
    {
      // Transmitting to the stack was successful
      // The peripheral should being doing throughput soon
      connList[connIndex].throughputToggle = !connList[connIndex].throughputToggle;
      Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Requested Peer Toggle Throughput %s",
                     tempToggleVal ? "On"  : "Off");
    }
}

/*********************************************************************
 * @fn      ThroughputCentral_processPDUUpdate
 *
 * @brief   Process PDU update and generate a characteristic
 *          write request to send to the peripheral
 *
 * @param   charValue - the desired PDU value, either 27B or 251B
 *
 * @return  none
 */
static void ThroughputCentral_processPDUUpdate(uint16_t charValue)
{
    // When Changing PDU Size, throughput is momentarily stopped on the peripheral
    // side for the application to process the change.
    // During this time the throughput will not reflect the correct value

    // Attempt to send PDU update via GATT Write
    // Variables Needed for GATT Write

    attWriteReq_t writeReq;
    uint8_t connIndex = ThroughputCentral_getConnIndex(tcConnHandle);
    connList[connIndex].pduValue = charValue;

    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    THROUGHPUTCENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    // Populate the Request Structure
    writeReq.cmd = 0;
    writeReq.handle = connList[connIndex].throughputObject[THROUGHPUT_SERVICE_UPDATE_PDU].charHandle;
    writeReq.len = THROUGHPUT_SERVICE_UPDATE_PDU_LEN;
    writeReq.pValue = GATT_bm_alloc(tcConnHandle, ATT_WRITE_REQ, THROUGHPUT_SERVICE_UPDATE_PDU_LEN, NULL);
    memcpy(writeReq.pValue, &connList[connIndex].pduValue, THROUGHPUT_SERVICE_UPDATE_PDU_LEN);
    writeReq.sig = 0;

    // Perform a GATT Write + Check Status
    uint8_t status;

    status = GATT_WriteCharValue(tcConnHandle, &writeReq, selfEntity);

    if( status != SUCCESS )
    {
        // We didn't successfully send this command to the stack!
        // Let's attempt to retransmit again and free the pValue pointer

        GATT_bm_free((gattMsg_t *)&writeReq, ATT_WRITE_REQ);

        // Requeue the Message
        uint8_t *pData;
        if ((pData = ICall_malloc(sizeof(uint8_t))))
        {
          *pData = connList[connIndex].pduValue;
          Clock_setFunc(Clock_handle(&delayedTrigger), ThroughputCentral_delayedPDUTriggerFxn, (UArg)pData);
          Clock_start(Clock_handle(&delayedTrigger));
        }
    }
    else
    {
        // Transmitting to the stack was successful
        // Inform user that a Request was sent to update peer's PDU Size
        if(BOARD_DISPLAY_USE_LCD == 1)
        {
          Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "Req. PDU: %dB", connList[connIndex].pduValue);
        }
        else
        {
          Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Requested Peer Change TX PDU Size to %dB", connList[connIndex].pduValue);
        }
        // Start PHY change request here if this is the initial connection attempt.
        if(connList[connIndex].establishingConn)
        {
          uint8_t *pData;
          if ((pData = ICall_malloc(sizeof(uint8_t))))
          {
            *pData = connList[connIndex].phyIndex;

            Clock_setFunc(Clock_handle(&delayedTrigger), ThroughputCentral_delayedPHYTriggerFxn, (UArg) pData);
            Clock_start(Clock_handle(&delayedTrigger));
          }
        }
    }
}

/*********************************************************************
 * @fn      ThroughputCentral_processPHYUpdate
 *
 * @brief   Process PHY update and generate a characteristic
 *          write request to send to the peripheral
 *
 * @param   charValue - the desired PHY for the characteristic
 *          and connection.
 *          0: 1M PHY
 *          1: 2M PHY
 *          2: CODED PHY: S2
 *          3: CODED PHY: S8
 *
 * @return  none
 */
static void ThroughputCentral_processPHYUpdate(uint16_t charValue)
{
    attWriteReq_t writeReq;
    uint8_t connIndex = ThroughputCentral_getConnIndex(tcConnHandle);
    // Critical Section so our Timer's SWI can't read the value while
    // we're writing to it.
    UInt key = Hwi_disable();
    {
      // Assign the PHY index - so we can keep track of PHY,
      // more importantly, coded phy and which symbol rate is being used
      connList[connIndex].phyIndex = charValue;
      // Reset confirm, indicating that it's the PHY being used in the
      // connection yet.
      connList[connIndex].phyConfirm = false;
    }
    Hwi_restore(key);
    // Populate the Request Structure
    writeReq.cmd = 0;
    writeReq.handle = connList[connIndex].throughputObject[THROUGHPUT_SERVICE_UPDATE_PHY].charHandle;
    writeReq.len = THROUGHPUT_SERVICE_UPDATE_PHY_LEN;
    writeReq.pValue = GATT_bm_alloc(tcConnHandle, ATT_WRITE_REQ, THROUGHPUT_SERVICE_UPDATE_PHY_LEN, NULL);
    memcpy(writeReq.pValue, &connList[connIndex].phyIndex, THROUGHPUT_SERVICE_UPDATE_PHY_LEN);
    writeReq.sig = 0;

    // Perform a GATT Write + Check Status
    uint8_t status;

    status = GATT_WriteCharValue(tcConnHandle, &writeReq, selfEntity);

    if( status != SUCCESS )
    {
        // We didn't successfully send this command to the stack!
        // Let's attempt to retransmit again and free the pValue pointer

        GATT_bm_free((gattMsg_t *)&writeReq, ATT_WRITE_REQ);

        uint8_t *pData;
        // Allocate space for the delayed PHY request
        if ((pData = ICall_malloc(sizeof(uint8_t))))
        {
          *pData = charValue;

          Clock_setFunc(Clock_handle(&delayedTrigger), ThroughputCentral_delayedPHYTriggerFxn, (UArg)pData);
          Clock_start(Clock_handle(&delayedTrigger));
        }
    }
    else
    {
        // Transmitting to the stack was successful
        /* The initial connection has been formed on 1M PHY, so the Peripheral won't
        *  initiate a PHY change.
        */
        if(connList[connIndex].establishingConn)
        {
          Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "Throughput Can Now Be Toggled");
          connList[connIndex].establishingConn = FALSE;
        }
        else
        {
        // Inform user that a request was sent to update peer's PHY Size
        Display_print1(dispHandle, TC_ROW_CUR_CONN, 0,
                       "Requested Peer Change PHY to %s",
                       (connList[connIndex].phyIndex == TS_PHY_1M)       ? "1M" :
                       (connList[connIndex].phyIndex == TS_PHY_2M)       ? "2M" :
                       (connList[connIndex].phyIndex == TS_PHY_CODED_S2) ? "Coded:S2" :
                        "Coded:S8");
        }
    }
}


/*********************************************************************
 * @fn      ThroughputCentral_doDiscoverDevices
 *
 * @brief   Enables scanning
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool ThroughputCentral_doDiscoverDevices(uint8_t index)
{
  (void) index;
  status_t ret;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // The stack does not need to record advertising reports
  // since the application will filter them by Service UUID and save.

  // Reset number of scan results to 0 before starting scan
  numScanRes = 0;
  ret = GapScan_enable(0, DEFAULT_SCAN_DURATION, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // Let the stack record the advertising reports as many as up to DEFAULT_MAX_SCAN_RES.
  ret = GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

  if (ret != SUCCESS)
  {
      Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "Error in scanning ... (0x%x)", ret);
  }

  // Enable only "Stop Discovering" and disable all others in the main menu
  tbm_setItemStatus(&tcMenuMain, TC_ITEM_STOPDISC,
                    (TC_ITEM_ALL & ~TC_ITEM_STOPDISC));

  return ((ret) ? false : true);
}

/*********************************************************************
 * @fn      ThroughputCentral_doStopDiscovering
 *
 * @brief   Stop on-going scanning
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool ThroughputCentral_doStopDiscovering(uint8_t index)
{
  (void) index;

  GapScan_disable();

  return (true);
}

/*********************************************************************
 * @fn      ThroughputCentral_doEstablishLink
 *
 * @brief   Establish a link to a peer device
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool ThroughputCentral_doConnect(uint8_t index)
{
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  GapInit_connect(scanList[index].addrType, scanList[index].addr,
                  DEFAULT_INIT_PHY, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  GapScan_Evt_AdvRpt_t advRpt;

  GapScan_getAdvReport(index, &advRpt);

  GapInit_connect(advRpt.addrType, advRpt.addr, DEFAULT_INIT_PHY, 0);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

  // Enable only "Cancel Connecting" and disable all others in the main menu
  tbm_setItemStatus(&tcMenuMain, TC_ITEM_CANCELCONN,
                    (TC_ITEM_ALL & ~TC_ITEM_CANCELCONN));

  Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "Connecting...");

  tbm_goTo(&tcMenuMain);

  return (true);
}

/*********************************************************************
 * @fn      ThroughputCentral_doCancelConnecting
 *
 * @brief   Cancel on-going connection attempt
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool ThroughputCentral_doCancelConnecting(uint8_t index)
{
  (void) index;

  GapInit_cancelConnect();

  return (true);
}


/*********************************************************************
 * @fn      ThroughputCentral_doDisconnect
 *
 * @brief   Disconnect the specified link
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool ThroughputCentral_doDisconnect(uint8_t index)
{
  GAP_TerminateLinkReq(tcConnHandle, HCI_DISCONNECT_REMOTE_USER_TERM);

  return (true);
}

/*********************************************************************
 * @fn      ThroughputCentral_doSelectConn
 *
 * @brief   Select a connection to communicate with
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool ThroughputCentral_doSelectConn(uint8_t index)
{
  // index cannot be equal to or greater than MAX_NUM_BLE_CONNS
  THROUGHPUTCENTRAL_ASSERT(index < MAX_NUM_BLE_CONNS);

  tcConnHandle = connList[index].connHandle;

  if (connList[index].throughputObject[0].charHandle == 0)
  {
    // Initiate service discovery
    ThroughputCentral_enqueueMsg(TC_EVT_SVC_DISC, 0, NULL);
  }

  // Set the menu title and go to this connection's context
  TBM_SET_TITLE(&tcMenuPerConn, TBM_GET_ACTION_DESC(&tcMenuSelectConn, index));

  // Clear non-connection-related message
  Display_clearLine(dispHandle, TC_ROW_NON_CONN);

  tbm_goTo(&tcMenuPerConn);

  return (true);
}

bool ThroughputCentral_doConnectAll(uint8_t index)
{
  // index cannot be equal to or greater than MAX_NUM_BLE_CONNS
  THROUGHPUTCENTRAL_ASSERT(index < MAX_NUM_BLE_CONNS);

  return (true);
}

/*********************************************************************
 * @fn      ThroughputCentral_doSetConnPhy
 *
 * @brief   Set Connection PHY preference.
 *
 * @param   index - 0: 1M PHY
 *                  1: 2M PHY
 *                  2: CODED PHY: S2
 *                  3: CODED PHY: S8
 *
 * @return  always true
 */
bool ThroughputCentral_doSetConnPhy(uint8_t index)
{

  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_CODED,
  };

  static uint8_t options[] = {
    LL_PHY_OPT_NONE, LL_PHY_OPT_NONE,
    LL_PHY_OPT_S2, LL_PHY_OPT_S8,
  };

  uint8_t connIndex = ThroughputCentral_getConnIndex(tcConnHandle);
  connList[connIndex].phyOptions = options[index];

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  THROUGHPUTCENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

   // Generate index to send over to peripheral
  uint8_t data;
  switch(phy[index])
  {
    case HCI_PHY_1_MBPS:
      data = TS_PHY_1M;
      break;

    case HCI_PHY_2_MBPS:
      data = TS_PHY_2M;
      break;

    case HCI_PHY_CODED:
      {
        if(connList[connIndex].phyOptions == LL_PHY_OPT_S2)
          data = TS_PHY_CODED_S2;
        else if (connList[connIndex].phyOptions == LL_PHY_OPT_S8)
          data = TS_PHY_CODED_S8;
      }
      break;
  }

  if(connList[connIndex].connHandle != LINKDB_CONNHANDLE_INVALID)
  {
    // Check if we're already using coded PHY - switch over to 1M
    // between in order to keep stability
    if(connList[connIndex].phyIndex != data && data >= TS_PHY_CODED_S2 &&
       connList[connIndex].phyIndex >= TS_PHY_CODED_S2)
    {
      uint8_t *pData;
      if ((pData = ICall_malloc(sizeof(uint8_t))))
       {
         *pData = TS_PHY_1M;

         // Enqueue the event.
         if(ThroughputCentral_enqueueMsg(TC_EVT_PHY_UPDATE, SUCCESS, pData) != SUCCESS)
         {
           ICall_freeMsg(pData);
         }
       }

      // Start A Timer to trigger a Coded PHY change
      Util_restartClock(&startPHYClock, CODED_PHY_CHANGE_DELAY);

      // Assign the requested PHY to the payload of the PHY handler
      connList[connIndex].phyClock_phyIndex = data;
    }
    else
    {

      uint8_t *pData;
      if ((pData = ICall_malloc(sizeof(uint8_t))))
      {
        *pData = data;

        // Enqueue the event.
        if(ThroughputCentral_enqueueMsg(TC_EVT_PHY_UPDATE, SUCCESS, pData) != SUCCESS)
        {
          ICall_freeMsg(pData);
        }
      }
    }
  }
  else
  {
    // Set this device's Phy Preference on the current connection.
    HCI_LE_SetPhyCmd(tcConnHandle, LL_PHY_USE_PHY_PARAM, phy[index],
                     phy[index], connList[connIndex].phyOptions);

    // Set this device's PHY Perference on future connections by using:
    HCI_LE_SetDefaultPhyCmd(LL_PHY_USE_PHY_PARAM, phy[index], phy[index]);
  }

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX. For more information, see the LE 2M PHY section in 
  //the BLE-StackUser's Guide.
  // Note PHYs are already enabled by default in build_config.opt in stack project.

  Display_printf(dispHandle, TC_ROW_CUR_CONN, 0, "PHY preference: %s",
                 TBM_GET_ACTION_DESC(&tcMenuConnPhy, index));

  return (true);
}

/*********************************************************************
 * @fn      ThroughputCentral_doSetScanPhy
 *
 * @brief   Set PHYs for scanning.
 *
 * @param   index - 0: 1M PHY
 *                  1: CODED PHY (Long range)
 *
 * @return  always true
 */
bool ThroughputCentral_doSetScanPhy(uint8_t index)
{
  uint8_t newScanPhy;

  if (index == 0)
  {
    newScanPhy = SCAN_PRIM_PHY_1M;
  }
  else
  {
    newScanPhy = SCAN_PRIM_PHY_CODED;
  }

  // Set scanning primary PHY
  GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &newScanPhy);

  Display_printf(dispHandle, TC_ROW_NON_CONN, 0, "Primary Scan PHY: %s",
                 TBM_GET_ACTION_DESC(&tcMenuScanPhy, index));

  tbm_goTo(&tcMenuMain);

  return (true);
}

/*********************************************************************
 * @fn      ThroughputCentral_doSetDLEPDU
 *
 * @brief   Set PDU preference.
 *
 * @param   index - 0, 1
 *
 * @return  always true
 */
bool ThroughputCentral_doSetDLEPDU(uint8 index)
{
  uint8_t connIndex = ThroughputCentral_getConnIndex(tcConnHandle);
  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  THROUGHPUTCENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Vars to keep track of active packet length settings
  uint16_t* txOctets = ICall_malloc(sizeof(uint16_t));
  uint16_t txTime = 0;

  switch (index)
  {
    case 0:
      *txOctets = DEFAULT_PDU_SIZE;
      txTime =  DEFAULT_TX_TIME;
      break;
    case 1:
      *txOctets = DLE_MAX_PDU_SIZE;
      txTime = DLE_MAX_TX_TIME;
      break;
  }

  // ONLY RX PDU of Peripheral can be modified from central
  // In other words, using the commands below which adjust this devices TX PDU,
  // the peer device will adjust it's RX PDU size to allow reception.

  if(connList[connIndex].connHandle != LINKDB_CONNHANDLE_INVALID)
  {
    // Here we'll utilize the throughput profile to have the peer device
    // change it's  PDU size in order to send more data and increase throughput
    // or decrease TX PDU size to reduce throughput

    // Inform the Application to perform a GATT write with
    // the selected size
    ThroughputCentral_enqueueMsg(TC_EVT_PDU_UPDATE, SUCCESS, (uint8_t*) txOctets);
  }
  else
  {
    // DLE HCI command to adjust PDU size for current connection
    HCI_LE_SetDataLenCmd(connList[connIndex].connHandle, *txOctets, txTime);

    // Write suggested default for future connections
    HCI_LE_WriteSuggestedDefaultDataLenCmd(*txOctets, txTime);

    ICall_free(txOctets);
  }

  return (true);
}

/*********************************************************************
 * @fn      ThroughputCentral_doToggleThroughput
 *
 * @brief   Toggle throughput on/off
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool ThroughputCentral_doToggleThroughput(uint8 index)
{
    uint8_t *pData;

    // Allocate space for the PHY change.
    if ((pData = ICall_malloc(sizeof(uint8_t))))
    {
      *pData = tcConnHandle;

      // Enqueue the event.
      if(ThroughputCentral_enqueueMsg(TC_EVT_TOGGLE_THROUGHPUT, SUCCESS, pData) != SUCCESS)
      {
        ICall_freeMsg(pData);
      }
    }
    return(true);
}


/*********************************************************************
*********************************************************************/
