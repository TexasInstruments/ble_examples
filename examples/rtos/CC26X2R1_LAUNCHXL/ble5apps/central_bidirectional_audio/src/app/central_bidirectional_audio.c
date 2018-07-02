/******************************************************************************

@file  central_bidirectional_audio.c

@brief This file contains the Central Bidirectional Audio sample application
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
#include <ti/drivers/PIN.h>

#include "bcomdef.h"

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "board_key.h"
#include "board.h"

#include "ble_user_config.h"

#include <profiles/audio_dle/audio_duplex.h>
#include <profiles/audio_dle/audio_client_disc.h>
#include <profiles/audio_dle/audio_profile_dle.h>

#include <menu/two_btn_menu.h>
#include "central_bidirectional_audio.h"
#include "central_bidirectional_audio_menu.h"

/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/

// Application events
#define CA_EVT_KEY_CHANGE          0x01
#define CA_EVT_SCAN_ENABLED        0x02
#define CA_EVT_SCAN_DISABLED       0x03
#define CA_EVT_ADV_REPORT          0x04
#define CA_EVT_SVC_DISC            0x05
#define CA_EVT_PAIR_STATE          0x07
#define CA_EVT_PASSCODE_NEEDED     0x08
#define CA_EVT_READ_RPA            0x09
#define CA_EVT_INSUFFICIENT_MEM    0x0A
#define CA_AUDIO_EVT               0x0B

// Simple Central Task Events
#define CA_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define CA_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define CA_ALL_EVENTS                        (CA_ICALL_EVT           | \
                                              CA_QUEUE_EVT)

// Address mode of the local device
#define DEFAULT_ADDRESS_MODE                  ADDRMODE_PUBLIC

// Default PHY for scanning and initiating
#define DEFAULT_SCAN_PHY                     SCAN_PRIM_PHY_1M
#define DEFAULT_INIT_PHY                     SCAN_PRIM_PHY_1M

// Default scan duration in 10 ms
#define DEFAULT_SCAN_DURATION                100 // 10s

// Default minimum connection interval (units of 1.25ms)
#define DEFAULT_MIN_CONN_INTERVAL             8

// Default maximum connection interval (units of 1.25ms)
#define DEFAULT_MAX_CONN_INTERVAL             8

// Default minimum connection interval (units of 1.25ms)
#define DEFAULT_CODED_MIN_CONN_INTERVAL             13

// Default maximum connection interval (units of 1.25ms)
#define DEFAULT_CODED_MAX_CONN_INTERVAL             13

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      8

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// How often to read current current RPA (in ms)
#define CA_READ_RPA_PERIOD                    3000

// Task configuration
#define CA_TASK_PRIORITY                     1

#ifndef CA_TASK_STACK_SIZE
#define CA_TASK_STACK_SIZE                   1024
#endif

// Advertising report fields to keep in the list
// Interested in only peer address type and peer address
#define CA_ADV_RPT_FIELDS   (SCAN_ADVRPT_FLD_ADDRTYPE | SCAN_ADVRPT_FLD_ADDRESS)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define CA_ADDR_STR_SIZE     15

// Row numbers for two-button menu
#define CA_ROW_SEPARATOR_1                    (TBM_ROW_APP + 0)
#define CA_ROW_OWN_ADDR                       (TBM_ROW_APP + 1)
#define CA_ROW_NUM_CXNS                       (TBM_ROW_APP + 2)
#define CA_ROW_PEER_ADDR                      (TBM_ROW_APP + 3)
#define CA_ROW_GATT_STATE                     (TBM_ROW_APP + 4)
#define CA_ROW_PHY_STATE                      (TBM_ROW_APP + 5)
#define CA_ROW_DISC_STATE                     (TBM_ROW_APP + 6)
#define CA_ROW_PAIRING_STATE                  (TBM_ROW_APP + 7)
#define CA_ROW_AUDIO_STATE                    (TBM_ROW_APP + 8)
#define CA_ROW_HCI_STATE                      (TBM_ROW_APP + 9)

// Spin if the expression is not true
#define CentralAudio_ASSERT(expr) if (!(expr)) CentralAudio_spin();

#if MAX_NUM_BLE_CONNS > 1
  #error "Error: Only 1 connection supported for bidirectional audio streaming"
#endif //MAX_NUM_BLE_CONNS > 1


/*********************************************************************
* TYPEDEFS
*/

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} caEvt_t;

typedef struct
{
  uint8_t uiOutputs;
  uint16_t connHandle;
}passcodeEvt_t;

// Connected device information
typedef struct
{
  uint16_t connHandle;        // Connection Handle
  uint8_t  addr[B_ADDR_LEN];  // Peer Device Address
  AudioClientDisc_handles_t  audioSvcHandles; // Audio Service handles
} connRec_t;

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

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct caTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(caTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t caTaskStack[CA_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Central Audio Tx/Rx";

// Number of connected devices
static uint8_t numConn = 0;
static bool connectionInProgress = false;

// List of connections
static connRec_t connList[MAX_NUM_BLE_CONNS];

// Accept or reject L2CAP connection parameter update request
static bool acceptParamUpdateReq = true;

// Maximum PDU size (default = 27 octets)
static uint16_t caMaxPduSize;

// Preferred PHY
static bool waitforPhyChange = false;
static uint8_t preferredInitPhy = INIT_PHY_1M;
static uint8_t preferredStreamPhy = HCI_PHY_1_MBPS;

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

/* Pin driver handles */
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State allPinState;

static PIN_Config ledPinTable[] =
{
  // LEDs initially off
  Board_PIN_RLED   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_PIN_GLED   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void CentralAudio_init(void);
static void CentralAudio_taskFxn(uintptr_t a0, uintptr_t a1);

static void CentralAudio_handleKeys(uint8_t keys);
static uint8_t CentralAudio_processStackMsg(ICall_Hdr *pMsg);
static void CentralAudio_processGapMsg(gapEventHdr_t *pMsg);
static void CentralAudio_processGATTMsg(gattMsgEvent_t *pMsg);
static bool CentralAudio_processAppMsg(caEvt_t *pMsg);
static bool CentralAudio_findSvcUuid(const uint8_t *uuid, uint8_t uuidLen,
                                    uint8_t *pData,
                                    uint16_t dataLen);
static uint8_t CentralAudio_addConnInfo(uint16_t connHandle, uint8_t *pAddr);
static uint8_t CentralAudio_removeConnInfo(uint16_t connHandle);
static uint8_t CentralAudio_getConnIndex(uint16_t connHandle);
static char* CentralAudio_getConnAddrStr(uint16_t connHandle);
static void CentralAudio_processPairState(uint8_t state, uint8_t status);
static void CentralAudio_processPasscode(uint16_t connHandle,
                                        uint8_t uiOutputs);
static void CentralAudio_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void CentralAudio_passcodeCb(uint8_t *deviceAddr, uint16_t connHandle,
                                   uint8_t uiInputs, uint8_t uiOutputs,
                                   uint32_t numComparison);
static void CentralAudio_pairStateCb(uint16_t connHandle, uint8_t state,
                                    uint8_t status);

static void CentralAudio_keyChangeHandler(uint8 keys);

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
static void CentralAudio_clockHandler(UArg arg);
#endif // PRIVACY_1_2_CFG

static uint8_t CentralAudio_enqueueMsg(uint8_t event, uint8_t status,
                                      uint8_t *pData);

static void CentralAudio_scanCb(uint32_t evt, void* msg, uintptr_t arg);
static bStatus_t CentralAudio_EnableNotification( uint16_t connHandle,
                                                uint16_t attrHandle );
static void CentralAudio_setEvent(uint8_t newEvents);
static bool CentralAudio_alreadyConnected(uint8_t  *addr);

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
CentralAudio_passcodeCb, // Passcode callback
CentralAudio_pairStateCb // Pairing/Bonding state Callback
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      CentralAudio_spin
*
* @brief   Spin forever
*
* @param   none
*/
static void CentralAudio_spin(void)
{
  volatile uint8_t x;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
* @fn      CentralAudio_createTask
*
* @brief   Task creation function for the Simple Central.
*
* @param   none
*
* @return  none
*/
void CentralAudio_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = caTaskStack;
  taskParams.stackSize = CA_TASK_STACK_SIZE;
  taskParams.priority = CA_TASK_PRIORITY;

  Task_construct(&caTask, CentralAudio_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      CentralAudio_Init
*
* @brief   Initialization function for the Simple Central App Task.
*          This is called during initialization and should contain
*          any application specific initialization (ie. hardware
*          initialization/setup, table initialization, power up
*          notification).
*
* @param   none
*
* @return  none
*/
static void CentralAudio_init(void)
{
  uint8_t i;

  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Initialize internal data
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
  }

  Board_initKeys(CentralAudio_keyChangeHandler);

  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void *)attDeviceName);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  // These values are set in the worst case for Coded PHY
  HCI_LE_WriteSuggestedDefaultDataLenCmd(LL_MAX_LINK_DATA_LEN,
                                          LL_MAX_LINK_DATA_TIME_CODED);

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
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    // Do not use authenticated pairing
    uint8_t mitm = FALSE;
    // This is a display only device
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    // Create a bond during the pairing process
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Start Bond Manager and register callback
  // This must be done before initialing the GAP layer
  VOID GAPBondMgr_Register(&bondMgrCBs);


  /* Audio streaming requires a specific connection interval in order to achieve
   * the necessary throughput
   */

  // Set phy params for 1 and 2 M PHY
  GapInit_setPhyParam((INIT_PHY_1M | INIT_PHY_2M),
                        INIT_PHYPARAM_CONN_INT_MAX, DEFAULT_MAX_CONN_INTERVAL);
  GapInit_setPhyParam((INIT_PHY_1M | INIT_PHY_2M),
                        INIT_PHYPARAM_CONN_INT_MIN, DEFAULT_MIN_CONN_INTERVAL);

  // Slave latency and supervision timeout should be the same for all PHYsS
  GapInit_setPhyParam((INIT_PHY_1M | INIT_PHY_2M | INIT_PHY_CODED),
                        INIT_PHYPARAM_CONN_LATENCY, 0);
  GapInit_setPhyParam((INIT_PHY_1M | INIT_PHY_2M | INIT_PHY_CODED),
                        INIT_PHYPARAM_SUP_TIMEOUT, 10);

  // Accept all parameter update requests, until audio device is found
  GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, GAP_UPDATE_REQ_ACCEPT_ALL);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Initialize GAP layer for Central role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_CENTRAL, selfEntity, addrMode, NULL);

  dispHandle = Display_open(Display_Type_ANY, NULL);
  Display_printf(dispHandle, 0, 0, "\f");

  // Initialize Two-Button Menu module
  TBM_SET_TITLE(&menuMain, "Central Bidirectional Audio");
  tbm_setItemStatus(&menuMain, CA_ITEM_CONFIG, TBM_ITEM_ALL);

  tbm_initTwoBtnMenu(dispHandle, &menuMain, 4, NULL);
  Display_printf(dispHandle, CA_ROW_SEPARATOR_1, 0, "====================");

  AudioDuplex_open((pfnAudioDuplexCB_t)CentralAudio_setEvent);

  // Open all pins
  ledPinHandle = PIN_open(&allPinState, ledPinTable);

  // Turn on Red led to indicate nothing has been connectedf
  PIN_setOutputValue( ledPinHandle, Board_PIN_RLED, 1);
  PIN_setOutputValue( ledPinHandle, Board_PIN_GLED, 0);
}

/*********************************************************************
* @fn      CentralAudio_taskFxn
*
* @brief   Application task entry point for the Simple Central.
*
* @param   none
*
* @return  events not processed
*/
static void CentralAudio_taskFxn(uintptr_t a0, uintptr_t a1)
{
  // Initialize application
  CentralAudio_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, CA_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
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
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = CentralAudio_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message
      if (events & CA_QUEUE_EVT)
      {
        caEvt_t *pMsg;
        while (pMsg = (caEvt_t *)Util_dequeueMsg(appMsgQueue))
        {
          // Process message
          bool freeNeeded = CentralAudio_processAppMsg(pMsg);

          // Free the space from the message, if not already freed by
          // CentralAudio_processAppMsg
          if(freeNeeded)
          {
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
* @fn      CentralAudio_processStackMsg
*
* @brief   Process an incoming task message.
*
* @param   pMsg - message to process
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t CentralAudio_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      CentralAudio_processGapMsg((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      CentralAudio_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch (pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
          CentralAudio_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;

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
                  if (pMyMsg->cmdStatus ==
                      HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
                  {
                    Display_printf(dispHandle, CA_ROW_PHY_STATE, 0,
                            "PHY Change failure, peer does not support this");
                  }
                  else
                  {
                    Display_printf(dispHandle, CA_ROW_PHY_STATE, 0,
                                   "PHY Update Status: 0x%02x",
                                   pMyMsg->cmdStatus);
                  }
                }
                break;
              case HCI_DISCONNECT:
                break;

              default:
                {
                  Display_printf(dispHandle, CA_ROW_HCI_STATE, 0,
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
              Display_printf(dispHandle, CA_ROW_PHY_STATE, 0,
                             "%s: PHY change failure",
                             CentralAudio_getConnAddrStr(pPUC->connHandle));
            }
            else
            {
              Display_printf(dispHandle, CA_ROW_PHY_STATE, 0,
                             "%s: PHY updated to %s",
                             CentralAudio_getConnAddrStr(pPUC->connHandle),
                             (pPUC->rxPhy == HCI_PHY_1_MBPS) ? "1 Mbps" :
                               ((pPUC->rxPhy == HCI_PHY_2_MBPS) ? "2 Mbps" :
                                   "Coded"));

              if((waitforPhyChange) && (pPUC->rxPhy == preferredStreamPhy) &&
                 (pPUC->txPhy == preferredStreamPhy))
              {
                uint8_t connIdx = CentralAudio_getConnIndex(pPUC->connHandle);
                waitforPhyChange = false;

                if(AudioClientDisc_isComplete(connIdx))
                {
                  tbm_setItemStatus(&menuMain, CA_ITEM_STREAM, CA_ITEM_NONE);
                  tbm_goTo(&menuMain);
                }
              }
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
* @fn      CentralAudio_processAppMsg
*
* @brief   Scanner application event processing function.
*
* @param   pMsg - pointer to event structure
*
* @return  none
*/
static bool CentralAudio_processAppMsg(caEvt_t *pMsg)
{
bool safeToDealloc = true;

  switch (pMsg->hdr.event)
  {
    case CA_EVT_KEY_CHANGE:
      CentralAudio_handleKeys(pMsg->hdr.state);
      break;

    case CA_EVT_ADV_REPORT:
    {
      GapScan_Evt_AdvRpt_t* pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);

      /* We have received an advertisement report
       * This code will connect based on the following criteria:
       *   1. We are not already in the process of forming a connection
       *   2. We are not already connected to this device
       *   3. This device contains an audio service UUID in its adv or scan rsp
       *      payload
       */
      if (!connectionInProgress &&
          !CentralAudio_alreadyConnected(pAdvRpt->addr) &&
           CentralAudio_findSvcUuid(audioProfileServUUID, ATT_UUID_SIZE,
                                    pAdvRpt->pData, pAdvRpt->dataLen))
      {
        status_t stat = GapInit_connect(pAdvRpt->addrType & MASK_ADDRTYPE_ID,
                        pAdvRpt->addr, preferredInitPhy, 0);

        if(stat == SUCCESS)
        {
          // We will only attempt to connect to one device at a time
          connectionInProgress = true;
        }

        Display_printf(dispHandle, CA_ROW_PEER_ADDR, 0,
                       "Connecting to: %s, with stat %d",
                       Util_convertBdAddr2Str(pAdvRpt->addr), stat);
      }

      if (pAdvRpt->pData != NULL)
      {
        ICall_free(pAdvRpt->pData);
      }
      ICall_free(pAdvRpt);
      safeToDealloc = false;

      break;
    }

    case CA_EVT_SCAN_ENABLED:
      Display_printf(dispHandle, CA_ROW_DISC_STATE, 0, "Discovering...");

      PIN_setOutputValue(ledPinHandle, Board_PIN_RLED,
                              !PIN_getOutputValue(Board_PIN_RLED));
      break;

    case CA_EVT_SCAN_DISABLED:
    {
      safeToDealloc = false;

      PIN_setOutputValue(ledPinHandle, Board_PIN_RLED,
                              !PIN_getOutputValue(Board_PIN_RLED));

      if(numConn > 0)
      {
        // Turn off LED if we are connected
        PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, 0 );
      }

      Display_clearLine(dispHandle, CA_ROW_DISC_STATE);

      ICall_free(pMsg);

      break;
    }

    // Pairing event
    case CA_EVT_PAIR_STATE:
    {
      CentralAudio_processPairState(pMsg->hdr.state, *pMsg->pData);

      ICall_free(pMsg->pData);
      break;
    }

    // Passcode event
    case CA_EVT_PASSCODE_NEEDED:
    {
      passcodeEvt_t *passcodeEvt = (passcodeEvt_t *)pMsg->pData;
      CentralAudio_processPasscode(passcodeEvt->connHandle,
                                   passcodeEvt->uiOutputs);

      ICall_free(pMsg->pData);
      break;
    }

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
    case CA_EVT_READ_RPA:
    {
      // Read the current RPA.
      // The parameters for the call to HCI_LE_ReadLocalResolvableAddressCmd
      // are not needed to be accurate to retrieve the local resolvable address.
      // The 1st parameter can be any of ADDRMODE_PUBLIC and ADDRMODE_RANDOM.
      // The 2nd parameter only has to be not NULL.
      // The result will come with HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS
      // complete event.
      HCI_LE_ReadLocalResolvableAddressCmd(0, rpa);
      break;
    }
#endif // PRIVACY_1_2_CFG

    // Insufficient memory
    case CA_EVT_INSUFFICIENT_MEM:
    {
      // We are running out of memory.
      Display_printf(dispHandle, CA_ROW_DISC_STATE, 0,
                      "Insufficient Memory");

      // We might be in the middle of scanning, try stopping it.
      GapScan_disable();
      break;
    }
    case CA_AUDIO_EVT:
    {
      AudioDuplex_eventHandler(pMsg->hdr.state);
      break;
    }

    default:
      // Do nothing.
      break;
  }
return (safeToDealloc);
}

/*********************************************************************
* @fn      CentralAudio_alreadyConnected
*
* @brief   Function to check if we are already connected to this device
*
* @param   addr - pointer to device address
*
* @return  none
*/
static bool CentralAudio_alreadyConnected(uint8_t  *addr)
{
  for(uint8_t idx = 0; idx < MAX_NUM_BLE_CONNS; idx++)
  {
    if(connList[idx].connHandle != LINKDB_CONNHANDLE_INVALID &&
       (!memcmp(addr, connList[idx].addr, B_ADDR_LEN)))
    {
      return true;
    }
  }

  return false;
}

/*********************************************************************
* @fn      CentralAudio_setEvent
*
* @brief   Function to set event in the Central Bidirectional Audio task.
*
* @param   None.
*
* @return  None.
*/
static void CentralAudio_setEvent(uint8_t newEvents)
{
  CentralAudio_enqueueMsg(CA_AUDIO_EVT, newEvents, NULL);
}

/*********************************************************************
* @fn      CentralAudio_processGapMsg
*
* @brief   GAP message processing function.
*
* @param   pMsg - pointer to event message structure
*
* @return  none
*/
static void CentralAudio_processGapMsg(gapEventHdr_t *pMsg)
{
  switch (pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      uint8_t temp8;
      uint16_t temp16;
      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      // Setup scanning
      // For more information, see the GAP section in the User's Guide:
      // http://software-dl.ti.com/lprf/ble5stack-latest/

      // Register callback to process Scanner events
      GapScan_registerCb(CentralAudio_scanCb, NULL);

      // Set Scanner Event Mask
      GapScan_setEventMask(GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED |
                           GAP_EVT_ADV_REPORT);

      // Set Scan PHY parameters
      GapScan_setPhyParams(DEFAULT_SCAN_PHY, SCAN_TYPE_ACTIVE,
                           SCAN_PARAM_DFLT_INTERVAL, SCAN_PARAM_DFLT_INTERVAL);

      // Set Advertising report fields to keep
      temp16 = CA_ADV_RPT_FIELDS;
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

      caMaxPduSize = pPkt->dataPktLen;

      Display_printf(dispHandle, CA_ROW_NUM_CXNS, 0, "Num Conns: %d",
                      numConn);

      // Display device address
      Display_printf(dispHandle, CA_ROW_OWN_ADDR, 0, "%s Addr: %s",
                     (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
                     Util_convertBdAddr2Str(pPkt->devAddr));

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
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
        HCI_LE_ReadLocalResolvableAddressCmd(0, rpa);

        // Create one-shot clock for RPA check event.
        Util_constructClock(&clkRpaRead, CentralAudio_clockHandler,
                            CA_READ_RPA_PERIOD, 0, true, CA_EVT_READ_RPA);
      }
#endif // PRIVACY_1_2_CFG

      // Enable scanning menu
      tbm_setItemStatus(&menuMain, (CA_ITEM_SCAN | CA_ITEM_CONFIG | CA_ITEM_PHY),
                          CA_ITEM_NONE);
      break;
    }

    case GAP_CONNECTING_CANCELLED_EVENT:
    {

      Display_printf(dispHandle, CA_ROW_DISC_STATE, 0,
                     "Conneting attempt cancelled");

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      uint16_t connHandle = ((gapEstLinkReqEvent_t*) pMsg)->connectionHandle;
      uint8_t* pAddr = ((gapEstLinkReqEvent_t*) pMsg)->devAddr;
      uint8_t  connIndex;
      uint8_t* pStrAddr;

      // Add this connection info to the list
      connIndex = CentralAudio_addConnInfo(connHandle, pAddr);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      CentralAudio_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

      Display_printf(dispHandle, CA_ROW_PEER_ADDR, 0,
                      "Connected to %s", pStrAddr);
      Display_printf(dispHandle, CA_ROW_NUM_CXNS, 0,
                      "Num Conns: %d", numConn);

      connectionInProgress = false;

      attExchangeMTUReq_t req;

      // Discover GATT Server's Rx MTU size
      req.clientRxMTU = caMaxPduSize - L2CAP_HDR_SIZE;

      // ATT MTU size should be set to the minimum of the Client Rx MTU
      // and Server Rx MTU values
      GATT_ExchangeMTU(connHandle, &req, selfEntity);

      // Startup discovery processing on peer
      AudioClientDisc_open(&connList[connIndex].audioSvcHandles,
                           connIndex);

      // Disable PHY and scan menu
      tbm_setItemStatus(&menuMain, CA_ITEM_NONE, (CA_ITEM_PHY | CA_ITEM_SCAN));

      // Clear discovery line since we are connected
      Display_clearLine(dispHandle, CA_ROW_DISC_STATE);

      // Turn on GLED to indicate connection, turn off RLED
      PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 1);
      PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, 0);

      if(waitforPhyChange)
      {
        uint8_t phyOpts = 0;
        if(preferredStreamPhy == HCI_PHY_CODED)
        {
          phyOpts = LL_PHY_OPT_S2;
        }

        // Change to desired stream PHY
        HCI_LE_SetPhyCmd(connHandle, 0, preferredStreamPhy, preferredStreamPhy,
                          phyOpts);
      }

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      uint16_t connHandle = ((gapTerminateLinkEvent_t*) pMsg)->connectionHandle;
      uint8_t connIndex;
      uint8_t* pStrAddr;

      AudioDuplex_stopStreaming();

      // Mark this connection deleted in the connected device list.
      connIndex = CentralAudio_removeConnInfo(connHandle);

      // Cleanup service discovery
      AudioClientDisc_close(connIndex);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      CentralAudio_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      Display_clearLines(dispHandle, CA_ROW_NUM_CXNS, CA_ROW_AUDIO_STATE+1);

      pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

      Display_printf(dispHandle, CA_ROW_PEER_ADDR, 0,
                      "%s is disconnected",
                      pStrAddr);
      Display_printf(dispHandle, CA_ROW_NUM_CXNS, 0,
                      "Num Conns: %d", numConn);

      // Disable the streaming menu, enable scanning and config
      tbm_setItemStatus(&menuMain, (CA_ITEM_SCAN | CA_ITEM_CONFIG | CA_ITEM_PHY),
                        CA_ITEM_STREAM);

      // Turn on GLED to indicate disconnect, turn on RLED
      PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 0);
      PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, 1);

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
            Display_printf(dispHandle, CA_ROW_PEER_ADDR, 0,
                          "Updated: %s, connTimeout:%d",
                           Util_convertBdAddr2Str(linkInfo.addr),
                           linkInfo.connTimeout);
          }
          else
          {
            // Display the address of the connection update failure
            Display_printf(dispHandle, CA_ROW_PEER_ADDR, 0,
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
* @fn      CentralAudio_handleKeys
*
* @brief   Handles all key events for this device.
*
* @param   keys - bit field for key events. Valid entries:
*                 KEY_LEFT
*                 KEY_RIGHT
*
* @return  none
*/
static void CentralAudio_handleKeys(uint8_t keys)
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
* @fn      CentralAudio_processGATTMsg
*
* @brief   Process GATT messages and events.
*
* @return  none
*/
static void CentralAudio_processGATTMsg(gattMsgEvent_t *pMsg)
{
  uint8_t connIndex = CentralAudio_getConnIndex(pMsg->connHandle);

  if (linkDB_Up(pMsg->connHandle))
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      Display_printf(dispHandle, CA_ROW_GATT_STATE, 0,
                     "ATT Rsp dropped %d", pMsg->method);
    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_printf(dispHandle, CA_ROW_GATT_STATE, 0,
                       "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Display_printf(dispHandle, CA_ROW_GATT_STATE, 0,
                       "Read rsp: 0x%02x", pMsg->msg.readRsp.pValue[0]);
      }
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_printf(dispHandle, CA_ROW_GATT_STATE, 0,
                       "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.

      // Display the opcode of the message that caused the violation.
      Display_printf(dispHandle, CA_ROW_GATT_STATE, 0,
                     "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      // MTU size updated
      Display_printf(dispHandle, CA_ROW_GATT_STATE, 0,
                     "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }

    if(!AudioClientDisc_isComplete(connIndex))
    {
      AudioClientDisc_status_t status = AudioClientDisc_processGATTDisc(pMsg,
                                                                        selfEntity,
                                                                        connIndex);

      if (status == AUDIO_CLIENT_DISC_COMPLETE)
      {

        CentralAudio_EnableNotification(pMsg->connHandle,
                                        connList[connIndex].audioSvcHandles.audioStartCCCHandle);

        CentralAudio_EnableNotification(pMsg->connHandle,
                                        connList[connIndex].audioSvcHandles.audioDataCCCHandle);

        AudioDuplex_setConnectionHandle(pMsg->connHandle);

        Display_printf(dispHandle, CA_ROW_GATT_STATE, 0,
                        "Audio Service Discovery Complete on connIdx %d",
                        connIndex);

        // Enable the streaming menu, disable scanning and codec config,
        // If we are not waiting for a PHY change.
        if(!waitforPhyChange)
        {
          tbm_setItemStatus(&menuMain, CA_ITEM_STREAM,
                            (CA_ITEM_SCAN | CA_ITEM_CONFIG | CA_ITEM_PHY));
        }

        // Reject connection parameter requests when streaming audio
        GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION,
                          GAP_UPDATE_REQ_DENY_ALL);
      }
    }

    switch ( pMsg->method )
    {
      case ATT_HANDLE_VALUE_NOTI:
      {
        // Check to see if notification is from audio data or control char
        if (pMsg->msg.handleValueNoti.handle == connList[connIndex].audioSvcHandles.audioDataCharValueHandle)
        {
          AudioDuplex_audioData pData;
          pData.len = pMsg->msg.handleValueNoti.len;
          pData.pValue = pMsg->msg.handleValueNoti.pValue;
          AudioDuplex_processData(AudioDuplex_data, &pData);
        }
        else if (pMsg->msg.handleValueNoti.handle == connList[connIndex].audioSvcHandles.audioStartCharValueHandle)
        {
          // Toggle GLED with every audio packet
          PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 0);
          AudioDuplex_audioData pData;
          pData.len = pMsg->msg.handleValueNoti.len;
          pData.pValue = pMsg->msg.handleValueNoti.pValue;
          AudioDuplex_processData(AudioDuplex_start_stop, &pData);
          PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 1);
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
  } // else - in case a GATT message came after a connection has dropped

  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
* @fn      CentralAudio_EnableNotification
*
* @brief   Enable notification for a given attribute handle.
*
* @param   connHandle - connection handle to send notification on
* @param   attrHandle - attribute handle to send notification for
*
* @return  status of gatt write
*/
static bStatus_t CentralAudio_EnableNotification( uint16_t connHandle,
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
* @fn      CentralAudio_processCmdCompleteEvt
*
* @brief   Process an incoming OSAL HCI Command Complete Event.
*
* @param   pMsg - message to process
*
* @return  none
*/
static void CentralAudio_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
      uint16_t connHandle = BUILD_UINT16(pMsg->pReturnParam[1],
                                         pMsg->pReturnParam[2]);
      int8 rssi = (int8)pMsg->pReturnParam[3];

      Display_printf(dispHandle, CA_ROW_HCI_STATE, 0, "%s: RSSI -%d dBm",
                     CentralAudio_getConnAddrStr(connHandle), -rssi);

      break;
    }

    case HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS:
    {
      uint8_t* pRpaNew = &(pMsg->pReturnParam[1]);

      if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
      {
        // If the RPA has changed, update the display
        Display_printf(dispHandle, CA_ROW_OWN_ADDR, 0, "RP Addr: %s",
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
* @fn      CentralAudio_processPairState
*
* @brief   Process the new paring state.
*
* @return  none
*/
static void CentralAudio_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_printf(dispHandle, CA_ROW_PAIRING_STATE, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, CA_ROW_PAIRING_STATE, 0, "Pairing success");
    }
    else
    {
      Display_printf(dispHandle, CA_ROW_PAIRING_STATE, 0,
                      "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_ENCRYPTED)
  {
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, CA_ROW_PAIRING_STATE, 0,
                      "Encryption success");
    }
    else
    {
      Display_printf(dispHandle, CA_ROW_PAIRING_STATE, 0,
                      "Encryption failed: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, CA_ROW_PAIRING_STATE, 0,
                      "Bond save success");
    }
    else
    {
      Display_printf(dispHandle, CA_ROW_PAIRING_STATE, 0,
                      "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
* @fn      CentralAudio_processPasscode
*
* @brief   Process the Passcode request.
*
* @return  none
*/
static void CentralAudio_processPasscode(uint16_t connHandle,
                                        uint8_t uiOutputs)
{
  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_printf(dispHandle, CA_ROW_PAIRING_STATE, 0, "Passcode: %d",
                   B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connHandle , SUCCESS, B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
* @fn      CentralAudio_findSvcUuid
*
* @brief   Find a given UUID in an advertiser's service UUID list.
*
* @return  TRUE if service UUID found
*/
static bool CentralAudio_findSvcUuid(const uint8_t *uuid, uint8_t uuidLen,
                                    uint8_t *pData,
                                    uint16_t dataLen)
{
  // Iterate over each byte in the advertisement data
  for(uint8_t advByte = 0; advByte < dataLen;)
  {
    uint8_t length = pData[advByte];
    // Advance to header byte
    advByte++;
    // First parse the length byte
    if(length == (uuidLen + 1))
    {
      // Compare current byte to the proper advertisement flag
      if( pData[advByte] == GAP_ADTYPE_16BIT_MORE     ||
          pData[advByte] == GAP_ADTYPE_16BIT_COMPLETE ||
          pData[advByte] == GAP_ADTYPE_128BIT_MORE    ||
          pData[advByte] == GAP_ADTYPE_128BIT_COMPLETE)
      {
        // Advance advByte past the header
        advByte++;
        if (!memcmp(&(pData[advByte]), uuid, uuidLen))
        {
          // We have found a match
          return true;
        }
        else
        {
          // If we found a UUID listing, but not the one we are looking for, skip
          // to next flag
          advByte += (length - 1);
        }
      }
      else
      {
        // If we have not found a UUID record just advance by one
        advByte += length;
      }
    }
    else
    {
      // Advance past this item
      advByte+= length;
    }

  }

  // We have searched the entire advertisement and found no match
  return false;
}

/*********************************************************************
* @fn      CentralAudio_addConnInfo
*
* @brief   Add a device to the connected device list
*
* @return  index of the connected device list entry where the new connection
*          info is put in.
*          if there is no room, MAX_NUM_BLE_CONNS will be returned.
*/
static uint8_t CentralAudio_addConnInfo(uint16_t connHandle, uint8_t *pAddr)
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
* @fn      CentralAudio_removeConnInfo
*
* @brief   Remove a device from the connected device list
*
* @return  index of the connected device list entry where the new connection
*          info is removed from.
*          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
*/
static uint8_t CentralAudio_removeConnInfo(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      // Found the entry to mark as deleted
      connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
      numConn--;

      break;
    }
  }

  return i;
}

/*********************************************************************
* @fn      CentralAudio_getConnIndex
*
* @brief   Find index in the connected device list by connHandle
*
* @return  the index of the entry that has the given connection handle.
*          if there is no match, MAX_NUM_BLE_CONNS will be returned.
*/
static uint8_t CentralAudio_getConnIndex(uint16_t connHandle)
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
* @fn      CentralAudio_getConnAddrStr
*
* @brief   Return, in string form, the address of the peer associated with
*          the connHandle.
*
* @return  A null-terminated string of the address.
*          if there is no match, NULL will be returned.
*/
static char* CentralAudio_getConnAddrStr(uint16_t connHandle)
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
* @fn      CentralAudio_pairStateCb
*
* @brief   Pairing state callback.
*
* @return  none
*/
static void CentralAudio_pairStateCb(uint16_t connHandle, uint8_t state,
                                    uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    CentralAudio_enqueueMsg(CA_EVT_PAIR_STATE, state, pData);
  }
}

/*********************************************************************
* @fn      CentralAudio_passcodeCb
*
* @brief   Passcode callback.
*
* @return  none
*/
static void CentralAudio_passcodeCb(uint8_t *deviceAddr, uint16_t connHandle,
                                   uint8_t uiInputs, uint8_t uiOutputs,
                                   uint32_t numComparison)
{
  passcodeEvt_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(passcodeEvt_t))))
  {
    pData->uiOutputs = uiOutputs;
    pData->connHandle = connHandle;

    // Enqueue the event.
    CentralAudio_enqueueMsg(CA_EVT_PASSCODE_NEEDED, 0, (uint8_t *)pData);
  }
}

/*********************************************************************
* @fn      CentralAudio_keyChangeHandler
*
* @brief   Key event handler function
*
* @param   a0 - ignored
*
* @return  none
*/
static void CentralAudio_keyChangeHandler(uint8 keys)
{
  CentralAudio_enqueueMsg(CA_EVT_KEY_CHANGE, keys, NULL);
}

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
/*********************************************************************
* @fn      CentralAudio_clockHandler
*
* @brief   clock handler function
*
* @param   arg - argument from the clock initiator
*
* @return  none
*/
void CentralAudio_clockHandler(UArg arg)
{
  uint8_t evtId = (uint8_t) (arg & 0xFF);

  switch (evtId)
  {
    case CA_EVT_READ_RPA:
      // Restart timer
      Util_startClock(&clkRpaRead);
      // Let the application handle the event
      CentralAudio_enqueueMsg(CA_EVT_READ_RPA, 0, NULL);
      break;

    default:
      break;
  }
}
#endif // PRIVACY_1_2_CFG

/*********************************************************************
* @fn      CentralAudio_enqueueMsg
*
* @brief   Creates a message and puts the message in RTOS queue.
*
* @param   event - message event.
* @param   state - message state.
* @param   pData - message data pointer.
*
* @return  TRUE or FALSE
*/
static uint8_t CentralAudio_enqueueMsg(uint8_t event, uint8_t state,
                                         uint8_t *pData)
{
  caEvt_t *pMsg = ICall_malloc(sizeof(caEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return (false);
}

/*********************************************************************
* @fn      CentralAudio_scanCb
*
* @brief   Callback called by GapScan module
*
* @param   evt - event
* @param   msg - message coming with the event
* @param   arg - user argument
*
* @return  none
*/
void CentralAudio_scanCb(uint32_t evt, void* pMsg, uintptr_t arg)
{
  if (evt & GAP_EVT_ADV_REPORT)
  {
    CentralAudio_enqueueMsg(CA_EVT_ADV_REPORT, SUCCESS, pMsg);
  }
  else if (evt & GAP_EVT_SCAN_ENABLED)
  {
    CentralAudio_enqueueMsg(CA_EVT_SCAN_ENABLED, SUCCESS, pMsg);
  }
  else if (evt & GAP_EVT_SCAN_DISABLED)
  {
    CentralAudio_enqueueMsg(CA_EVT_SCAN_DISABLED, SUCCESS, pMsg);
  }
  else if (evt & GAP_EVT_INSUFFICIENT_MEMORY)
  {
    CentralAudio_enqueueMsg(CA_EVT_INSUFFICIENT_MEM, SUCCESS, pMsg);
  }
}

/*********************************************************************
* @fn      CentralAudio_doScanAction
*
* @brief   Start or stop scanning menu action handler
*
* @param   index - Not used
*
* @return  always true
*/
bool CentralAudio_doScanAction(uint8_t index)
{
  // Start scanning immediately.
  status_t status = GapScan_enable(0, DEFAULT_SCAN_DURATION, 0);
  if(status != SUCCESS)
  {
      Display_printf(dispHandle, CA_ROW_DISC_STATE, 0,
                      "Start Scan failed: %d", status);
  }

  return (true);
}

/*********************************************************************
* @fn      CentralAudio_doStartStream
*
* @brief   Start Audio Streaming.
*
* @param   index - Not used
*
* @return  always true
*/
bool CentralAudio_doStartStream(uint8_t index)
{
  if(linkDB_Up(AudioDuplex_getConnectionHandle()))
  {
    // Deny cxn param requests when streaming audio
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION,
                        GAP_UPDATE_REQ_DENY_ALL);
    switch(index)
    {
      case 0:
      {
        AudioDuplex_startStreaming(AUDIO_DUPLEX_STREAM_TYPE_ADPCM);
        Display_printf(dispHandle, CA_ROW_AUDIO_STATE, 0,
                        "Stream Started: ADPCM");
        // Go to the main menu
        tbm_goTo(&menuMain);
        break;
      }
      case 1:
      {
        AudioDuplex_startStreaming(AUDIO_DUPLEX_STREAM_TYPE_MSBC);
        Display_printf(dispHandle, CA_ROW_AUDIO_STATE, 0,
                        "Stream Started: MSBC");

        // Go to the main menu
        tbm_goTo(&menuMain);
        break;
      }
      default:
        break;
    }
  }
  return (true);
}

/*********************************************************************
* @fn      CentralAudio_doStopStream
*
* @brief   Start Audio Streaming.
*
* @param   index - Not used
*
* @return  always true
*/
bool CentralAudio_doStopStream(uint8_t index)
{
  // Start chain of events to stop stream
  Display_printf(dispHandle, CA_ROW_AUDIO_STATE, 0,
                        "Stream stopped");

  AudioDuplex_stopStreaming();

  // Go to the main menu
  tbm_goTo(&menuMain);
  return (true);
}

/*********************************************************************
* @fn      CentralAudio_doSetPhy
*
* @brief   Set the phy preferences for scanning and connecting
*          Note: this can only be done before a connection
*
* @param   index - 0 = 1M PHY scan, init
*                  1 = 1M PHY scan, 1M init, 2M stream
*                  2 = Coded S=2 PHY scan, Coded S=2 init
*
* @return  always true
*/
bool CentralAudio_doSetPhy(uint8_t index)
{
  switch(index)
  {
    case 0:
    {
      Display_printf(dispHandle, CA_ROW_PHY_STATE, 0,
                      "Set Scan PHY 1M, Init PHY 1M");
      GapInit_setPhyParam(INIT_PHY_1M, INIT_PHYPARAM_CONN_INT_MAX,
                            DEFAULT_MAX_CONN_INTERVAL);
      GapInit_setPhyParam(INIT_PHY_1M, INIT_PHYPARAM_CONN_INT_MIN,
                            DEFAULT_MIN_CONN_INTERVAL);

      break;
    }
    case 1:
    {
      Display_printf(dispHandle, CA_ROW_PHY_STATE, 0,
                      "Set Scan PHY 1M, Init PHY 1M, Stream PHY 2M");

      /*
       * We cannot initiate on 2M PHY so we will establish on 1M and switch
       * to 2M PHY. In this case, streaming will be blocked until
       * audio discovery is complete and 2M phy is enabled
       */
      waitforPhyChange = true;
      preferredStreamPhy = HCI_PHY_2_MBPS;
      GapInit_setPhyParam(INIT_PHY_1M, INIT_PHYPARAM_CONN_INT_MAX,
                            DEFAULT_MAX_CONN_INTERVAL);
      GapInit_setPhyParam(INIT_PHY_1M, INIT_PHYPARAM_CONN_INT_MIN,
                            DEFAULT_MIN_CONN_INTERVAL);
      break;
    }
    case 2:
    {
      preferredInitPhy = HCI_PHY_CODED;
      Display_printf(dispHandle, CA_ROW_PHY_STATE, 0,
                      "Set Scan PHY Coded S=2, Init PHY 1M, Stream PHY Coded S=2");

      uint8_t temp8 = HCI_PHY_CODED;
      GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);

      GapInit_setPhyParam(HCI_PHY_CODED, INIT_PHYPARAM_CONN_INT_MAX,
                            DEFAULT_CODED_MAX_CONN_INTERVAL);
      GapInit_setPhyParam(HCI_PHY_CODED, INIT_PHYPARAM_CONN_INT_MIN,
                            DEFAULT_CODED_MIN_CONN_INTERVAL);
      break;
    }
    default:
      break;
  }

  // Go to the main menu
  tbm_goTo(&menuMain);
  return (true);
}

/*********************************************************************
*********************************************************************/
