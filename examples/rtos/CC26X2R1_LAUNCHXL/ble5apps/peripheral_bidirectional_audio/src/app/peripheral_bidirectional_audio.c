/******************************************************************************

 @file  peripheral_bidirectional_audio.c

 @brief This file contains the Peripheral Bidirectional Audio sample application
        for use with the CC26x2 Bluetooth Low Energy Protocol Stack.

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

#include <board.h>
#include <board_key.h>

#include <menu/two_btn_menu.h>

#include "peripheral_bidirectional_audio_menu.h"
#include "peripheral_bidirectional_audio.h"

#include <profiles/audio_dle/audio_duplex.h>
#include <profiles/audio_dle/audio_profile_dle.h>
#include <profiles/audio_dle/audio_client_disc.h>


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Address mode of the local device
#define DEFAULT_ADDRESS_MODE                  ADDRMODE_PUBLIC

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Minimum connection interval (units of 1.25ms, 80=100ms) for parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 104=130ms) for  parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8

// Slave latency to use for parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 300=3s) for parameter update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          300

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_DENY_ALL

// How often to read current current RPA (in ms)
#define PA_READ_RPA_EVT_PERIOD               3000

// Task configuration
#define PA_TASK_PRIORITY                     1

#ifndef PA_TASK_STACK_SIZE
#define PA_TASK_STACK_SIZE                   1024
#endif

// Application events
#define PA_STATE_CHANGE_EVT                  0
#define PA_KEY_CHANGE_EVT                    1
#define PA_ADV_EVT                           2
#define PA_PAIR_STATE_EVT                    3
#define PA_PASSCODE_EVT                      4
#define PA_READ_RPA_EVT                      5
#define PA_SEND_PARAM_UPDATE_EVT             6
#define PA_AUDIO_EVT                         7

// Internal Events for RTOS application
#define PA_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define PA_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define PA_ALL_EVENTS                        (PA_ICALL_EVT             | \
                                              PA_QUEUE_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define PA_ADDR_STR_SIZE     15

// Row numbers for two-button menu
#define PA_ROW_SEPARATOR_1                    (TBM_ROW_APP + 0)
#define PA_ROW_STATUS_1                       (TBM_ROW_APP + 1)
#define PA_ROW_STATUS_2                       (TBM_ROW_APP + 2)
#define PA_ROW_CONNECTION                     (TBM_ROW_APP + 3)
#define PA_ROW_ADVSTATE                       (TBM_ROW_APP + 4)
#define PA_ROW_RSSI                           (TBM_ROW_APP + 5)
#define PA_ROW_IDA                            (TBM_ROW_APP + 6)
#define PA_ROW_RPA                            (TBM_ROW_APP + 7)
#define PA_ROW_DEBUG                          (TBM_ROW_APP + 8)
#define PA_DISP_ROW_GEN_STATUS1               (TBM_ROW_APP + 9)
#define PA_DISP_ROW_GEN_STATUS2               (TBM_ROW_APP + 10)

// Spin if the expression is not true
#define PERIPHERALAUDIO_ASSERT(expr) if (!(expr)) PeripheralAudio_spin();

#if MAX_NUM_BLE_CONNS > 1
  #error "Error: Only 1 connection supported for bidirectional audio streaming"
#endif //MAX_NUM_BLE_CONNS > 1

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} paEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} paPairStateData_t;

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
} paPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} paGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;
  uint8_t data[];
} spClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
  List_Elem elem;
  uint16_t  connHandle;
} spConnHandleEntry_t;

// Connected device information
typedef struct
{
  uint16_t         connHandle;                        // Connection Handle
  Clock_Struct*    pUpdateClock;                      // pointer to clock struct
  bool             phyCngRq;                          // Set to true if PHY change request is in progress
  uint8_t          currPhy;
  uint8_t          rqPhy;
  uint8_t          phyRqFailCnt;                      // PHY change request count
} paConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

// Task configuration
Task_Struct paTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(paTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t paTaskStack[PA_TASK_STACK_SIZE];

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

// Memory to pass RPA read event ID to clock handler
spClockEventData_t argRpaRead =
{ .event = PA_READ_RPA_EVT };

// Per-handle connection info
static paConnRec_t connList[MAX_NUM_BLE_CONNS];

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Bidirectional Audio";

// Advertisement data
static uint8_t advertData[] =
{
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  (ATT_UUID_SIZE + 0x01),   // length of this data, Audio service UUID + header
  GAP_ADTYPE_128BIT_MORE,   // some of the UUID's, but not all
  TI_BASE_UUID_128(AUDIO_SERV_UUID),
};

// Scan Response Data4e
static uint8_t scanRspData[] =
{
  // complete name
  20,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'B',
  'i',
  'd',
  'i',
  'r',
  'e',
  'c',
  't',
  'i',
  'o',
  'n',
  'a',
  'l',
  ' ',
  'A',
  'u',
  'd',
  'i',
  'o',

  // connection interval range
  5,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

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

// Structure to store all handles found by AudioClientDisc module
static AudioClientDisc_handles_t  audioSvcHandles;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void PeripheralAudio_init( void );
static void PeripheralAudio_taskFxn(UArg a0, UArg a1);

static uint8_t PeripheralAudio_processStackMsg(ICall_Hdr *pMsg);
static uint8_t PeripheralAudio_processGATTMsg(gattMsgEvent_t *pMsg);
static void PeripheralAudio_processGapMessage(gapEventHdr_t *pMsg);
static void PeripheralAudio_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void PeripheralAudio_processAdvEvent(paGapAdvEventData_t *pEventData);
static void PeripheralAudio_processAppMsg(paEvt_t *pMsg);
#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
static void PeripheralAudio_updateRPA(void);
#endif // PRIVACY_1_2_CFG
static void PeripheralAudio_clockHandler(UArg arg);
static void PeripheralAudio_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);
static void PeripheralAudio_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void PeripheralAudio_processPairState(paPairStateData_t *pPairState);
static void PeripheralAudio_processPasscode(paPasscodeData_t *pPasscodeData);
static uint8_t PeripheralAudio_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData);
static void PeripheralAudio_keyChangeHandler(uint8 keys);
static void PeripheralAudio_handleKeys(uint8_t keys);
static void PeripheralAudio_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void PeripheralAudio_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
static uint8_t PeripheralAudio_addConn(uint16_t connHandle);
static uint8_t PeripheralAudio_getConnIndex(uint16_t connHandle);
static uint8_t PeripheralAudio_removeConn(uint16_t connHandle);
static void PeripheralAudio_processParamUpdate(uint16_t connHandle);
static uint8_t PeripheralAudio_clearConnListEntry(uint16_t connHandle);
static void PeripheralAudio_setEvent(uint8_t newEvents);
static bStatus_t PeripheralAudio_EnableNotification( uint16_t connHandle,
                                                    uint16_t attrHandle );

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t PeripheralAudio_BondMgrCBs =
{
  PeripheralAudio_passcodeCb,       // Passcode callback
  PeripheralAudio_pairStateCb       // Pairing/Bonding state Callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      PeripheralAudio_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void PeripheralAudio_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 */
void PeripheralAudio_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = paTaskStack;
  taskParams.stackSize = PA_TASK_STACK_SIZE;
  taskParams.priority = PA_TASK_PRIORITY;

  Task_construct(&paTask, PeripheralAudio_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      PeripheralAudio_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void PeripheralAudio_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, &paramUpdateDecision);
  }

  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {
    // Don't send a pairing request after connecting; the peer device must
    // initiate pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
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
  GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&PeripheralAudio_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  // These values are set in the worst case for Coded PHY
  HCI_LE_WriteSuggestedDefaultDataLenCmd(LL_MAX_LINK_DATA_LEN,
                                          LL_MAX_LINK_DATA_TIME_CODED);

  // Initialize GATT Client
  GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Init key debouncer
  Board_initKeys(PeripheralAudio_keyChangeHandler);

  // Initialize Connection List
  PeripheralAudio_clearConnListEntry(CONNHANDLE_ALL);

  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, NULL);

  // The type of display is configured based on the BOARD_DISPLAY_USE...
  // preprocessor definitions
  dispHandle = Display_open(Display_Type_ANY, NULL);
  Display_printf(dispHandle, 0, 0, "\f");

  // Initialize Two-Button Menu module
  TBM_SET_TITLE(&menuMain, "Peripheral Bidirectional Audio");
  tbm_setItemStatus(&menuMain, PA_ITEM_CONFIG, PA_ITEM_NONE);

  tbm_initTwoBtnMenu(dispHandle, &menuMain, 4, NULL);
  Display_printf(dispHandle, PA_ROW_SEPARATOR_1, 0, "====================");

  AudioDuplex_open((pfnAudioDuplexCB_t)PeripheralAudio_setEvent);
}

/*********************************************************************
 * @fn      PeripheralAudio_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 */
static void PeripheralAudio_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  PeripheralAudio_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, PA_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

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
            safeToDealloc = PeripheralAudio_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & PA_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          paEvt_t *pMsg = (paEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            PeripheralAudio_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t PeripheralAudio_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      PeripheralAudio_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = PeripheralAudio_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        {
          PeripheralAudio_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;
        }

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
        {
          hciEvt_HardwareError_t *hwErr = (hciEvt_HardwareError_t *)pMsg;
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, hwErr->hardwareCode);
          break;
        }

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
                Display_printf(dispHandle, PA_ROW_STATUS_1, 0,
                        "PHY Change failure, peer does not support this");
              }
              else
              {
                Display_printf(dispHandle, PA_ROW_STATUS_1, 0,
                               "PHY Update Status Event: 0x%x",
                               pMyMsg->cmdStatus);
              }

              PeripheralAudio_updatePHYStat(HCI_LE_SET_PHY, (uint8_t *)pMsg);
              break;
            }

            case HCI_EXT_CONN_EVENT_NOTICE:
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

          // A Phy Update Has Completed or Failed
          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
          {
            static bool firstUpdate = true;
            if (pPUC->status != SUCCESS)
            {
              Display_printf(dispHandle, PA_ROW_STATUS_1, 0,
                             "PHY Change failure");
            }
            else
            {
              // Only symmetrical PHY is supported.
              // rxPhy should be equal to txPhy.
              Display_printf(dispHandle, PA_ROW_STATUS_2, 0,
                             "PHY Updated to %s",
                             (pPUC->rxPhy == HCI_PHY_1_MBPS) ? "1M" :
                             (pPUC->rxPhy == HCI_PHY_2_MBPS) ? "2M" :
                              "CODED");

              if(firstUpdate && (pPUC->rxPhy == HCI_PHY_CODED))
              {
                firstUpdate = false;
                HCI_LE_SetPhyCmd(pPUC->connHandle, 0, HCI_PHY_CODED,
                                  HCI_PHY_CODED, LL_PHY_OPT_S2);
              }
            }

            PeripheralAudio_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT,
                                          (uint8_t *)pMsg);
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
 * @fn      PeripheralAudio_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t PeripheralAudio_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_printf(dispHandle, PA_ROW_STATUS_1, 0,
                    "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_printf(dispHandle, PA_ROW_STATUS_1, 0,
                    "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  if(!AudioClientDisc_isComplete(0))
  {
    AudioClientDisc_status_t status = AudioClientDisc_processGATTDisc(pMsg,
                                                                      selfEntity,
                                                                      0);

    if (status == AUDIO_CLIENT_DISC_COMPLETE)
    {

      PeripheralAudio_EnableNotification(pMsg->connHandle,
                                          audioSvcHandles.audioStartCCCHandle);

      PeripheralAudio_EnableNotification(pMsg->connHandle,
                                          audioSvcHandles.audioDataCCCHandle);

      AudioDuplex_setConnectionHandle(pMsg->connHandle);

      Display_printf(dispHandle, PA_DISP_ROW_GEN_STATUS2, 0,
                      "Audio Service Discovery Complete");

      // Reject connection parameter requests when streaming audio
      GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION,
                        GAP_UPDATE_REQ_DENY_ALL);

      // Disable audio configuration once connected
      tbm_setItemStatus(&menuMain, PA_ITEM_NONE, PA_ITEM_CONFIG);
    }
  }

  switch ( pMsg->method )
  {
    case ATT_HANDLE_VALUE_NOTI:
    {
      // Check to see if notification is from audio data or control char
      if (pMsg->msg.handleValueNoti.handle == audioSvcHandles.audioDataCharValueHandle)
      {
        AudioDuplex_audioData pData;
        pData.len = pMsg->msg.handleValueNoti.len;
        pData.pValue = pMsg->msg.handleValueNoti.pValue;
        AudioDuplex_processData(AudioDuplex_data, &pData);
      }
      else if (pMsg->msg.handleValueNoti.handle == audioSvcHandles.audioStartCharValueHandle)
      {
        AudioDuplex_audioData pData;
        pData.len = pMsg->msg.handleValueNoti.len;
        pData.pValue = pMsg->msg.handleValueNoti.pValue;
        AudioDuplex_processData(AudioDuplex_start_stop, &pData);
      }
      break;
    }
    // Service Change indication
    case ATT_HANDLE_VALUE_IND:
    {
      // Note: this logic assumes that the only indications that will be sent
      //       will come from that GATT Service Changed Characteristic
      if ( pMsg->hdr.status == SUCCESS )
      {
        // Acknowledge receipt of indication
        ATT_HandleValueCfm( pMsg->connHandle );
      }
      break;
    }

    default:
      break;
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      PeripheralAudio_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void PeripheralAudio_processAppMsg(paEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case PA_KEY_CHANGE_EVT:
      PeripheralAudio_handleKeys(*(uint8_t*)(pMsg->pData));
      break;

    case PA_ADV_EVT:
      PeripheralAudio_processAdvEvent((paGapAdvEventData_t*)(pMsg->pData));
      break;

    case PA_PAIR_STATE_EVT:
      PeripheralAudio_processPairState((paPairStateData_t*)(pMsg->pData));
      break;

    case PA_PASSCODE_EVT:
      PeripheralAudio_processPasscode((paPasscodeData_t*)(pMsg->pData));
      break;

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
    case PA_READ_RPA_EVT:
      PeripheralAudio_updateRPA();
      break;
#endif // PRIVACY_1_2_CFG
    case PA_AUDIO_EVT:
      AudioDuplex_eventHandler(pMsg->hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }

  // Free message data if it exists
  if (pMsg->pData)
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void PeripheralAudio_processGapMessage(gapEventHdr_t *pMsg)
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

        Display_printf(dispHandle, PA_ROW_STATUS_1, 0, "Initialized");

        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        // Temporary memory for advertising parameters for set #1. These will be copied
        // by the GapAdv module
        GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&PeripheralAudio_advCallback, &advParamLegacy,
                               &advHandleLegacy);
        PERIPHERALAUDIO_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advertData), advertData);
        PERIPHERALAUDIO_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanRspData), scanRspData);
        PERIPHERALAUDIO_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        PERIPHERALAUDIO_ASSERT(status == SUCCESS);

        // Use long range params to create long range set #2
        GapAdv_params_t advParamLongRange = GAPADV_PARAMS_AE_LONG_RANGE_CONN;

        // Create Advertisement set #2 and assign handle
        status = GapAdv_create(&PeripheralAudio_advCallback, &advParamLongRange,
                               &advHandleLongRange);
        PERIPHERALAUDIO_ASSERT(status == SUCCESS);

        // Load advertising data for set #2 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLongRange, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advertData), advertData);
        PERIPHERALAUDIO_ASSERT(status == SUCCESS);

        // Load scan response data for set #2 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLongRange, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanRspData), scanRspData);
        PERIPHERALAUDIO_ASSERT(status == SUCCESS);

        // Set event mask for set #2
        status = GapAdv_setEventMask(advHandleLongRange,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable long range advertising for set #2
        status = GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        PERIPHERALAUDIO_ASSERT(status == SUCCESS);

        // Display device address
        Display_printf(dispHandle, PA_ROW_IDA, 0, "%s Addr: %s",
                       (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
                       Util_convertBdAddr2Str(pPkt->devAddr));

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
        if (addrMode > ADDRMODE_RANDOM)
        {
          PeripheralAudio_updateRPA();

          // Create one-shot clock for RPA check event.
          Util_constructClock(&clkRpaRead, PeripheralAudio_clockHandler,
                              PA_READ_RPA_EVT_PERIOD, 0, true,
                              (UArg) &argRpaRead);
        }
#endif // PRIVACY_1_2_CFG
      }

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, PA_ROW_STATUS_2, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      if (pPkt->hdr.status == SUCCESS)
      {
        // Add connection to list and start RSSI
        PeripheralAudio_addConn(pPkt->connectionHandle);

        // Display the address of this connection
        Display_printf(dispHandle, PA_ROW_STATUS_1, 0, "Connected to %s",
                       Util_convertBdAddr2Str(pPkt->devAddr));

        // Startup discovery processing on peer
        AudioClientDisc_open(&audioSvcHandles, 0);
      }

      if (numActive < MAX_NUM_BLE_CONNS)
      {
        // Start advertising since there is room for more connections
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }
      else
      {
        // Stop advertising since there is no room for more connections
        GapAdv_disable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_disable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

      AudioDuplex_stopStreaming();

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, PA_ROW_STATUS_1, 0, "Device Disconnected!");
      Display_printf(dispHandle, PA_ROW_STATUS_2, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      // Remove the connection from the list
      PeripheralAudio_removeConn(pPkt->connectionHandle);

      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

      // Clear remaining lines
      Display_clearLine(dispHandle, PA_ROW_CONNECTION);

      // Enable audio configuration when advertising
      tbm_setItemStatus(&menuMain, PA_ITEM_CONFIG, PA_ITEM_NONE);

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
        // Display the address of the connection update
        Display_printf(dispHandle, PA_ROW_STATUS_2, 0, "Link Param Updated: %s",
                       Util_convertBdAddr2Str(linkInfo.addr));
      }
      else
      {
        // Display the address of the connection update failure
        Display_printf(dispHandle, PA_ROW_STATUS_2, 0,
                       "Link Param Update Failed 0x%x: %s", pPkt->opcode,
                       Util_convertBdAddr2Str(linkInfo.addr));
      }

      // Check if there are any queued parameter updates
      spConnHandleEntry_t *connHandleEntry = (spConnHandleEntry_t *)List_get(&paramUpdateList);
      if (connHandleEntry != NULL)
      {
        // Attempt to send queued update now
        PeripheralAudio_processParamUpdate(connHandleEntry->connHandle);

        // Free list element
        ICall_free(connHandleEntry);
      }

      break;
    }

    default:
      Display_clearLines(dispHandle, PA_ROW_STATUS_1, PA_ROW_STATUS_2);
      break;
  }
}

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
/*********************************************************************
 * @fn      PeripheralAudio_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void PeripheralAudio_updateRPA(void)
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
#endif // PRIVACY_1_2_CFG

/*********************************************************************
 * @fn      PeripheralAudio_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void PeripheralAudio_clockHandler(UArg arg)
{
  spClockEventData_t *pData = (spClockEventData_t *)arg;

  if (pData->event == PA_READ_RPA_EVT)
  {
    // Start the next period
    Util_startClock(&clkRpaRead);

    // Post event to read the current RPA
    PeripheralAudio_enqueueMsg(PA_READ_RPA_EVT, 0, NULL);
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys - bitmap of pressed keys
 *
 * @return  none
 */
static void PeripheralAudio_keyChangeHandler(uint8_t keys)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = keys;

    PeripheralAudio_enqueueMsg(PA_KEY_CHANGE_EVT, 0, pValue);
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 */
static void PeripheralAudio_handleKeys(uint8_t keys)
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
 * @fn      PeripheralAudio_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void PeripheralAudio_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  paGapAdvEventData_t *pData = ICall_malloc(sizeof(paGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    PeripheralAudio_enqueueMsg(PA_ADV_EVT, 0, (uint8_t *)pData);
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void PeripheralAudio_processAdvEvent(paGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      Display_printf(dispHandle, PA_ROW_ADVSTATE, 0, "Adv Set %d Enabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
      Display_printf(dispHandle, PA_ROW_ADVSTATE, 0, "Adv Set %d Disabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_START:
      break;

    case GAP_EVT_ADV_END:
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);

      Display_printf(dispHandle, PA_ROW_ADVSTATE, 0, "Adv Set %d disabled after conn %d",
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


/*********************************************************************
 * @fn      PeripheralAudio_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void PeripheralAudio_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  paPairStateData_t *pData = ICall_malloc(sizeof(paPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    PeripheralAudio_enqueueMsg(PA_PAIR_STATE_EVT, 0, (uint8_t *)pData);
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void PeripheralAudio_passcodeCb(uint8_t *pDeviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  paPasscodeData_t *pData = ICall_malloc(sizeof(paPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    PeripheralAudio_enqueueMsg(PA_PASSCODE_EVT, 0, (uint8_t *)pData);
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void PeripheralAudio_processPairState(paPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
      Display_printf(dispHandle, PA_ROW_CONNECTION, 0, "Pairing started");
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, PA_ROW_CONNECTION, 0, "Pairing success");
      }
      else
      {
        Display_printf(dispHandle, PA_ROW_CONNECTION, 0, "Pairing fail: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, PA_ROW_CONNECTION, 0, "Encryption success");
      }
      else
      {
        Display_printf(dispHandle, PA_ROW_CONNECTION, 0, "Encryption failed: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, PA_ROW_CONNECTION, 0, "Bond save success");
      }
      else
      {
        Display_printf(dispHandle, PA_ROW_CONNECTION, 0, "Bond save failed: %d", status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void PeripheralAudio_processPasscode(paPasscodeData_t *pPasscodeData)
{
  // Display passcode to user
  if (pPasscodeData->uiOutputs != 0)
  {
    Display_printf(dispHandle, PA_ROW_CONNECTION, 0, "Passcode: %d",
                   B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      PeripheralAudio_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t PeripheralAudio_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  paEvt_t *pMsg = ICall_malloc(sizeof(paEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
  }

  return (false);
}


/*********************************************************************
 * @fn      PeripheralAudio_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t PeripheralAudio_addConn(uint16_t connHandle)
{
  uint8_t i;
  uint8_t status = bleNoResources;

  // Try to find an available entry
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;

      // Set default PHY to 1M
      connList[i].currPhy = HCI_PHY_1_MBPS;

      break;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      PeripheralAudio_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t PeripheralAudio_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      PeripheralAudio_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. CONNHANDLE_ALL will always succeed.
 */
static uint8_t PeripheralAudio_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != CONNHANDLE_ALL)
  {
    // Get connection index from handle
    connIndex = PeripheralAudio_getConnIndex(connHandle);
    if(connIndex >= MAX_NUM_BLE_CONNS)
    {
      return(bleInvalidRange);
    }
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == CONNHANDLE_ALL))
    {
      connList[i].connHandle = CONNHANDLE_INVALID;
      connList[i].currPhy = 0;
      connList[i].phyCngRq = 0;
      connList[i].phyRqFailCnt = 0;
      connList[i].rqPhy = 0;
    }
  }

  return(SUCCESS);
}

/*********************************************************************
 * @fn      PeripheralAudio_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t PeripheralAudio_removeConn(uint16_t connHandle)
{
  uint8_t connIndex = PeripheralAudio_getConnIndex(connHandle);

  if(connIndex != MAX_NUM_BLE_CONNS)
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
    PeripheralAudio_clearConnListEntry(connHandle);
  }

  return connIndex;
}

/*********************************************************************
 * @fn      PeripheralAudio_processParamUpdate
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static void PeripheralAudio_processParamUpdate(uint16_t connHandle)
{
  gapUpdateLinkParamReq_t req;
  uint8_t connIndex;

  req.connectionHandle = connHandle;
  req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
  req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
  req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
  req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

  connIndex = PeripheralAudio_getConnIndex(connHandle);
  PERIPHERALAUDIO_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

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
    spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));
    if (connHandleEntry)
    {
      connHandleEntry->connHandle = connHandle;

      List_put(&paramUpdateList, (List_Elem *)&connHandleEntry);
    }
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void PeripheralAudio_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  uint8_t status = pMsg->pReturnParam[0];

  //Find which command this command complete is for
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
      // Display RSSI value, if RSSI is higher than threshold, change to faster PHY
      if (status == SUCCESS)
      {

        Display_printf(dispHandle, PA_ROW_RSSI, 0,
                       "RSSI:-%d",
                       (uint32_t)(-(int8_t)pMsg->pReturnParam[3]));
      }
      break;
    }

    case HCI_LE_READ_PHY:
    {
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, PA_ROW_RSSI + 2, 0, "RXPh: %d, TXPh: %d",
                       pMsg->pReturnParam[3], pMsg->pReturnParam[4]);
      }
      break;
    }

    case HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS:
    {
      uint8_t* pRpaNew = &(pMsg->pReturnParam[1]);

      if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
      {
        // If the RPA has changed, update the display
        Display_printf(dispHandle, PA_ROW_RPA, 0, "RP Addr: %s",
                       Util_convertBdAddr2Str(pRpaNew));
        memcpy(rpa, pRpaNew, B_ADDR_LEN);
      }
      break;
    }

    default:
      break;
  } // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
* @fn      PeripheralAudio_updatePHYStat
*
* @brief   Update the auto phy update state machine
*
* @param   connHandle - the connection handle
*
* @return  None
*/
static void PeripheralAudio_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
{
  uint8_t connIndex;

  switch (eventCode)
  {
    case HCI_LE_SET_PHY:
    {
      // Get connection handle from list
      spConnHandleEntry_t *connHandleEntry =
                           (spConnHandleEntry_t *)List_get(&setPhyCommStatList);

      if (connHandleEntry)
      {
        // Get index from connection handle
        connIndex = PeripheralAudio_getConnIndex(connHandleEntry->connHandle);

        ICall_free(connHandleEntry);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;

          if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
          {
            // Update the phychange request status for active RSSI tracking connection
            connList[connIndex].phyCngRq = FALSE;
            connList[connIndex].phyRqFailCnt++;
          }
        }
      }
      break;
    }

    // LE Event - a Phy update has completed or failed
    case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
    {
      hciEvt_BLEPhyUpdateComplete_t *pPUC =
                                     (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

      if(pPUC)
      {
        // Get index from connection handle
        connIndex = PeripheralAudio_getConnIndex(pPUC->connHandle);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          // Update the phychange request status for active RSSI tracking connection
          connList[connIndex].phyCngRq = FALSE;

          if (pPUC->status == SUCCESS)
          {
            connList[connIndex].currPhy = pPUC->rxPhy;
          }
          if(pPUC->rxPhy != connList[connIndex].rqPhy)
          {
            connList[connIndex].phyRqFailCnt++;
          }
          else
          {
            // Reset the request phy counter and requested phy
            connList[connIndex].phyRqFailCnt = 0;
            connList[connIndex].rqPhy = 0;
          }
        }
      }

      break;
    }

    default:
      break;
  } // end of switch (eventCode)
}

/*********************************************************************
 * @fn      PeripheralAudio_setEvent
 *
 * @brief   Function to set event in the Central Bidirectional Audio task.
 *
 * @param   None.
 *
 * @return  None.
 */
static void PeripheralAudio_setEvent(uint8_t newEvents)
{
  PeripheralAudio_enqueueMsg(PA_AUDIO_EVT, newEvents, NULL);
}

/*********************************************************************
 * @fn      PeripheralAudio_EnableNotification
 *
 * @brief   Enable notification for a given attribute handle.
 *
 * @param   connHandle - connection handle to send notification on
 * @param   attrHandle - attribute handle to send notification for
 *
 * @return  status of gatt write
 */
static bStatus_t PeripheralAudio_EnableNotification( uint16_t connHandle,
                                                    uint16_t attrHandle )
{
  attWriteReq_t req;

  req.pValue = GATT_bm_alloc( connHandle, ATT_WRITE_REQ, 2, NULL );
  if ( req.pValue != NULL )
  {
    uint8_t notificationsOn[] = {0x01, 0x00};

    req.handle = attrHandle;

    req.len = 2;
    memcpy(req.pValue, notificationsOn, 2);

    req.sig = 0;
    req.cmd = TRUE;

    bStatus_t status = GATT_WriteNoRsp( connHandle, &req);

    if ( status != SUCCESS )
    {
      GATT_bm_free( (gattMsg_t *)&req, ATT_WRITE_REQ );
    }

    return (status);
  }
  else
  {
    return (bleNoResources);
  }
}

/*********************************************************************
*********************************************************************/
