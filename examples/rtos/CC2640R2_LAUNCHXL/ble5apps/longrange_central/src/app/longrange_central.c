/******************************************************************************

 @file  longrange_central.c

 @brief This file contains the Throughput Central sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************

 Copyright (c) 2013-2017, Texas Instruments Incorporated
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
 Release Name: simplelink_cc2640r2_sdk_1_35_00_07_eng
 Release Date: 2017-03-23 10:36:21
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

#include "bcomdef.h"

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "central.h"

#include "board_key.h"
#include <menu/two_btn_menu.h>

#include "longrange_central_menu.h"
#include "profiles/temperature/temperature_service.h"
#include "profiles/throughput/throughput_service.h"

#include "board.h"

#include "longrange_central.h"

#include "ble_user_config.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// These Constants are for Application Queue Events
// These Events contain information on the Queue and need to be processed
#define SBC_STATE_CHANGE_EVT                  0x0001
#define SBC_KEY_CHANGE_EVT                    0x0002
#define SBC_RSSI_READ_EVT                     0x0004
#define SBC_PDU_UPDATE_EVT                    0x0008
#define SBC_PHY_UPDATE_EVT                    0x0010
#define SBC_MEASURE_INST_SPEED_EVT            0x0020
#define SBC_ENABLE_TEMPERATURE_NOTI_EVT       0x0040

// Simple BLE Central Task Events - often containing no information to process
// other than the event itself
#define SBC_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBC_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBC_START_DISCOVERY_EVT               Event_Id_00
#define SBC_PERIODIC_LED_EVT                  Event_Id_01

#define SBC_ALL_EVENTS                        (SBC_ICALL_EVT                | \
                                               SBC_QUEUE_EVT                | \
                                               SBC_START_DISCOVERY_EVT      | \
                                               SBC_PERIODIC_LED_EVT)

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

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   4000

#define SBC_PERIODIC_LED_PERIOD               500

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPCENTRALROLE_PARAM_UPDATE_REQ_AUTO_ACCEPT

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
#define DEFAULT_UPDATE_CONN_TIMEOUT           100

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           3000

#define CODED_PHY_CHANGE_DELAY                500

// Type of Display to open
#if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
  #define SBC_DISPLAY_TYPE Display_Type_LCD
#elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
  #define SBC_DISPLAY_TYPE Display_Type_UART
#else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
  #define SBC_DISPLAY_TYPE 0 // Option not supported
#endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART

// Row numbers
#define SBC_ROW_RESULT          TBM_ROW_APP
#define SBC_ROW_STATUS_1        (TBM_ROW_APP + 1)
#define SBC_ROW_STATUS_2        (TBM_ROW_APP + 2)
#define SBC_ROW_PEER_DEVICE     (TBM_ROW_APP + 2)
#define SBC_ROW_STATUS_3        (TBM_ROW_APP + 3)
#define SBC_ROW_PHY             (TBM_ROW_APP + 3)
#define SBC_ROW_RSSI            (TBM_ROW_APP + 4)
#define SBC_ROW_TEMP            (TBM_ROW_APP + 5)

// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                   864
#endif
#define DLE_MAX_PDU_SIZE 251
#define DLE_MAX_TX_TIME 17040

#define DEFAULT_PDU_SIZE 27
#define DEFAULT_TX_TIME 328

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_THROUGHPUT_SERVICE,  // Throughput Service discovery
  BLE_DISC_STATE_THROUGHPUT_CHAR,     // Throughput Characteristic discovery
  BLE_DISC_STATE_TEMPERATURE_SERVICE, // Temperature Service discovery
  BLE_DISC_STATE_TEMPERATURE_CHAR,    // Temperature Characteristic discovery
};

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
  uint16_t period;      // how often to read RSSI
  uint16_t connHandle;  // connection handle
  Clock_Struct *pClock; // pointer to clock struct
} readRssi_t;

// A struct to contain scan response data we're interested in
// Nameless struct typedefed to scanResultDeviceRecord_t
typedef struct
{
  uint8 addrType;            //!< Address Type: @ref GAP_Addr_Types
  uint8 addr[B_ADDR_LEN];    //!< Device's Address
} scanResultDeviceRecord_t;

typedef struct
{
  uint16_t charHdl;          //!< Characteristic Handle
  uint8 addr[ATT_UUID_SIZE]; //!< UUID of characteristic
} CharProfileHdl_t;

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

// Clock object used to signal timeout
static Clock_Struct startDiscClock;
static Clock_Struct periodicLED;
static Clock_Struct startPHYClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbcTask;
Char sbcTaskStack[SBC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "BLE5 Central";

// Number of scan results and scan result index
static uint8_t scanRes = 0;
static int8_t scanIdx = -1;

// Scan result list
static scanResultDeviceRecord_t devList[DEFAULT_MAX_SCAN_RES];

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Maximum PDU size (default = 27 octets)
static uint16 maxPduSize;

// Array of RSSI read structures
static readRssi_t readRssi[MAX_NUM_BLE_CONNS];

//// Received byte counters + circular buffer for Throughput Data
//static volatile uint32_t bytesRecvd = 0;
//#define CB_SIZE 10
//static uint32_t bytesRecvd_cb[CB_SIZE];
//static int      bytesRecvd_cb_index = 0;
//static bool     cbBufferFilled = false;

// Strings for PHY
static uint8_t* phyName[] = {
  "1 Mbps", "2 Mbps",
  "Coded:S2", "Coded:S8",
  "Coded"
};

// PHY Index
static uint8_t phyIndex = 0;
static bool phyConfirm = true;

// Pointer to requested PHY index
static uint8_t* phyClock_phyIndex= 0;

// PHY Options
static uint16_t phyOptions = HCI_PHY_OPT_NONE;

static CharProfileHdl_t* throughputHandles = NULL;
static CharProfileHdl_t* temperatureHandles = NULL;

// PIN config

// Global pin resources
PIN_State  pin;
PIN_Handle appPins;
static PIN_Config pinTable[] =
{
    Board_RLED       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */
    Board_GLED       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */

    PIN_TERMINATE
};

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
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData, uint8_t dataLen);
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType, uint8_t dataType, uint8_t dataLen, uint8_t* data);
static void SimpleBLECentral_performLEDTask(void);
static void SimpleBLECentral_clockHandler(UArg arg);

static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static bStatus_t SimpleBLECentral_StartRssi(uint16_t connHandle, uint16_t period);
static bStatus_t SimpleBLECentral_CancelRssi(uint16_t connHandle);
static readRssi_t *SimpleBLECentral_RssiAlloc(uint16_t connHandle);
static readRssi_t *SimpleBLECentral_RssiFind(uint16_t connHandle);
static void SimpleBLECentral_RssiFree(uint16_t connHandle);

static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent);

void SimpleBLECentral_speedHandler(UArg a0);
void SimpleBLECentral_PHYHandler(UArg a0);
void SimpleBLECentral_startDiscHandler(UArg a0);
void SimpleBLECentral_keyChangeHandler(uint8 keys);
void SimpleBLECentral_readRssiHandler(UArg a0);

static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t status,
                                           void *pData);

static void SBC_ClearDeviceList();
static void SBC_NextDevice();
static void SBC_ConnectToDevice();

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t SimpleBLECentral_roleCB =
{
  SimpleBLECentral_eventCB     // Event callback
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
  ICall_registerApp(&selfEntity, &syncEvent);

  appPins = PIN_open(&pin, pinTable);
  PIN_setOutputValue(appPins, Board_RLED, Board_LED_ON);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, SimpleBLECentral_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, NULL);

  // Setup throughput clock to run every second
  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicLED, SimpleBLECentral_clockHandler,
                      SBC_PERIODIC_LED_PERIOD, 0, false, SBC_PERIODIC_LED_EVT);

  // Set up a PHY Clock for transitions between Coded PHYs
  Util_constructClock(&startPHYClock, SimpleBLECentral_PHYHandler,
                      0, 0, false, 0);

  Board_initKeys(SimpleBLECentral_keyChangeHandler);

  // Open Display.
  dispHandle = Display_open(SBC_DISPLAY_TYPE, NULL);

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

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Start the Device
  VOID GAPCentralRole_StartDevice(&SimpleBLECentral_roleCB);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  /*
   * TBM stuff
   */

  // Set the title of the main menu
  TBM_SET_TITLE(&sbcMenuMain, "Texas Instruments Bluetooth 5 Long Range Demo");

  // Initialize Two-Button Menu module
  tbm_setItemStatus(&sbcMenuMain, TBM_ITEM_0, TBM_ITEM_1 | TBM_ITEM_2 );
  tbm_setItemStatus(&sbcMenuScanandConnect, TBM_ITEM_ALL, TBM_ITEM_0 | TBM_ITEM_1);

  tbm_initTwoBtnMenu(dispHandle, &sbcMenuMain, 2, NULL);

  // Get Current Data Length
  HCI_LE_ReadMaxDataLenCmd();

  // By Default Allow Central to support any and all PHYs
  HCI_LE_SetDefaultPhyCmd(LL_PHY_USE_ANY_PHY,
                          LL_PHY_1_MBPS | LL_PHY_2_MBPS | HCI_PHY_CODED,
                          LL_PHY_1_MBPS | LL_PHY_2_MBPS | HCI_PHY_CODED);

  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
  HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);
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
    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, SBC_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
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

      // If RTOS queue is not empty, process app message
      if (events & SBC_QUEUE_EVT)
      {
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
      }

      // Instantanous Speed Event
      if (events & SBC_START_DISCOVERY_EVT)
      {
        SimpleBLECentral_startDiscovery();
      }

      if (events & SBC_PERIODIC_LED_EVT)
      {
        Util_startClock(&periodicLED);

        // Perform periodic application task
        SimpleBLECentral_performLEDTask();
      }
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

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          case HCI_LE_EVENT_CODE:
            {
              hciEvt_BLEPhyUpdateComplete_t *pPUC
                = (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

              if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
              {
                if (pPUC->status != SUCCESS)
                {
                  Display_print0(dispHandle, SBC_ROW_RESULT, 0, "PHY Change failure");
                }
                else
                {
                  // Inform User that the PHY was Updated, and which PHY is
                  // the PHY being used for the connection
                  Display_print0(dispHandle, SBC_ROW_RESULT, 0, "PHY Update Complete");

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
                  if( phyConfirm == false)
                  {
                    // This means that the phyIndex was assigned by us.
                    // Confirm the value

                    // Critial Section so our Timer's SWI can't read the value while
                    // we're writing to it.
                    UInt key = Hwi_disable();
                    {
                      // Confirm the change to the phyIndex
                      phyConfirm = true;
                    }
                    Hwi_restore(key);
                  }
                  else
                  {
                    // the peer device requested the change to PHY
                    // update phyIndex accordingly and display the value
                    phyIndex = temp;
                  }

                  // Tell the use which PHY we're now using
                  Display_print1(dispHandle, SBC_ROW_PHY, 0, "Current PHY: %s", phyName[phyIndex]);

                  Util_startClock(&periodicLED);
                }
              }
            }
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

  attWriteReq_t writeReq = {0};

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

    case SBC_ENABLE_TEMPERATURE_NOTI_EVT:
      {
        uint8_t data[2] = {0x01, 0x00};

        // Populate the Request Structure
        writeReq.cmd = 0;
        writeReq.handle = temperatureHandles[TEMPERATURE_SERVICE_DATA].charHdl + 1;
        writeReq.len = 2;
        writeReq.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, 2, NULL);
        memcpy(writeReq.pValue, data, 2);
        writeReq.sig = 0;

        // Perform a GATT Write + Check Status
        uint8_t status;

        status = GATT_WriteCharValue(connHandle, &writeReq, selfEntity);

        if( status != SUCCESS ) {
          // Inform user that a Request was sent to update peer's PDU Size
          Display_print0(dispHandle, SBC_ROW_RESULT, 0, "Can't enable Notifications");

          GATT_bm_free((gattMsg_t *)writeReq.pValue, ATT_WRITE_REQ);
        }
        else {
          SimpleBLECentral_doSetPhy(3);
        }
      }
      break;

    case SBC_PDU_UPDATE_EVT:
      {
        // When Changing PDU Size, throughput is momentaryly stopped on the peripehral
        // side for the application to process the change.
        // During this time the throughput not reflect the correct value

        // Attempt to send PDU update via GATT Write
        // Variables Needed for GATT Write
        uint8_t pduSize = (uint8_t) *(pMsg->pData); // Cast down to uint8_t

        // Populate the Request Structure
        writeReq.cmd = 0;
        writeReq.handle = throughputHandles[THROUGHPUT_SERVICE_UPDATE_PDU].charHdl;
        writeReq.len = THROUGHPUT_SERVICE_UPDATE_PHY_LEN;
        writeReq.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, THROUGHPUT_SERVICE_UPDATE_PHY_LEN, NULL);
        memcpy(writeReq.pValue, &pduSize, THROUGHPUT_SERVICE_UPDATE_PHY_LEN);
        writeReq.sig = 0;

        // Perform a GATT Write + Check Status
        uint8_t status;

        status = GATT_WriteCharValue(connHandle, &writeReq, selfEntity);

        if( status != SUCCESS )
        {
          // We didn't successfully send this command to the stack!
          // Let's attempt to retransmit again and free the pValue pointer

          GATT_bm_free((gattMsg_t *)writeReq.pValue, ATT_WRITE_REQ);

          // Requeue the Message - don't free the memory for PDU size yet
          SimpleBLECentral_enqueueMsg(SBC_PDU_UPDATE_EVT, SUCCESS, pMsg->pData);
        }
        else
        {
          // Transmitting to the stack was successful
          // The peripheral should being doing throughput soon

          // Inform user that a Request was sent to update peer's PDU Size
          Display_print1(dispHandle, SBC_ROW_RESULT, 0, "Requested Peer Change TX PDU Size to %dB", pduSize);

          // Free the Allocated Memory
          if(pMsg->pData)
          {
            ICall_free(pMsg->pData);
          }
        }
        break;
      }

    case SBC_PHY_UPDATE_EVT:
      {
        // When Changing PHY, throughput is stopped on the peripehral
        // side for the application to process the change.
        // During this time the throughput not reflect the correct value

        // Critial Section so our Timer's SWI can't read the value while
        // we're writing to it.
        UInt key = Hwi_disable();
        {
          // Assign the PHY index - so we can keep track of PHY,
          // more importantly, coded phy and which symbol rate is being used
          phyIndex = (uint8_t) *(pMsg->pData);
          // reset confirm, indicating that it's the PHY being used in the
          // connection yet.
          phyConfirm = false;
        }
        Hwi_restore(key);

        // Populate the Request Structure
        writeReq.cmd = 0;
        writeReq.handle = throughputHandles[THROUGHPUT_SERVICE_UPDATE_PHY].charHdl;
        writeReq.len = THROUGHPUT_SERVICE_UPDATE_PHY_LEN;
        writeReq.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, THROUGHPUT_SERVICE_UPDATE_PHY_LEN, NULL);
        memcpy(writeReq.pValue, &phyIndex, THROUGHPUT_SERVICE_UPDATE_PHY_LEN);
        writeReq.sig = 0;

        // Perform a GATT Write + Check Status
        uint8_t status;

        status = GATT_WriteCharValue(connHandle, &writeReq, selfEntity);

        if( status != SUCCESS )
        {
          // We didn't successfully send this command to the stack!
          // Let's attempt to retransmit again and free the pValue pointer

          Display_print1(dispHandle, SBC_ROW_RESULT, 0, "GATT_WriteCharValue status: %d", status);

          GATT_bm_free((gattMsg_t *)writeReq.pValue, ATT_WRITE_REQ);

          // Requeue the Message - don't free the memory for PHY change yet
          SimpleBLECentral_enqueueMsg(SBC_PHY_UPDATE_EVT, SUCCESS, pMsg->pData);
        }
        else
        {
          // Transmitting to the stack was successful
          // The peripheral should being doing throughput soon

          // Inform user that a Request was sent to update peer's PHY Size
          Display_print1(dispHandle, SBC_ROW_RESULT, 0, "Requested Peer Change PHY to %s", phyName[phyIndex]);

          // Note if we're already using coded PHY, switching between S2 and S8
          // won't produce a PHY change event.

          // Free the Allocated Memory
          if(pMsg->pData)
          {
            ICall_free(pMsg->pData);
          }
        }
        break;
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
        Display_print1(dispHandle, SBC_ROW_RESULT, 0, "This Device's BDADDR : %s", Util_convertBdAddr2Str(pEvent->initDone.devAddr));
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        /*
         *  Device Filtering can be done here for UUID or advertisement
         *  Data if desired.
         *
         *  We populate the devList with both the scan response data and
         *  advertisement data of each device discovered
         */
        SimpleBLECentral_addDeviceInfo(pEvent->deviceInfo.addr,
                                       pEvent->deviceInfo.addrType,
                                       pEvent->deviceInfo.eventType,
                                       pEvent->deviceInfo.dataLen,
                                       pEvent->deviceInfo.pEvtData);
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // Initialize scan index.
        scanIdx = -1;

        /*
         * Note that pEvent->discCmpl contains a list of device records (NOT scan response data)
         * Scan Response Data is contained in the GAP_DEVICE_INFO_EVENT during Scanning
         * We're verifying that we got the right number of device info responses
         * If you don't care about device response data, you could just use
         * the data from the GAP_DEVICE_DISCOVERY_EVENT as your scan results list
         * as shown in the commented code below
         *
         * If we're not filtering on UUID then we should have gotten ALL of the
         * possible devices scanned in our device list.
         */
        Display_print1(dispHandle, SBC_ROW_RESULT, 0, "%d Devices Found", scanRes);

        if (scanRes > 0)
        {
          // Re enable all Menu Functions
          tbm_setItemStatus(&sbcMenuScanandConnect, TBM_ITEM_ALL, TBM_ITEM_NONE);

          // Display the first scan Result
          SBC_NextDevice();
        }
        else
        {
          // No Results, reenable scanning only
          tbm_setItemStatus(&sbcMenuScanandConnect, TBM_ITEM_ALL, TBM_ITEM_0 | TBM_ITEM_1);
        }
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;

          //Util_startClock(&periodicLED);

          // Go to Main Menu
          tbm_goTo(&sbcMenuMain);

          // Disable Scan Connect Menu, enable everything else
          tbm_setItemStatus(&sbcMenuMain, TBM_ITEM_ALL, TBM_ITEM_0);

          // Forget about the Scan Results
          SBC_ClearDeviceList();

          // If service discovery not performed initiate service discovery
          Util_startClock(&startDiscClock);

          // Update Display
          Display_print1(dispHandle, SBC_ROW_PEER_DEVICE, 0, "Peer Device : %s", Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));
          Display_print0(dispHandle, SBC_ROW_RESULT, 0, "Connected, Exchanging MTU");
          Display_print0(dispHandle, SBC_ROW_PHY, 0, "PHY: 1 Mbps");

          // Start RSSI collection
          SimpleBLECentral_StartRssi(connHandle, DEFAULT_RSSI_PERIOD);
        }
        else
        {
          state = BLE_STATE_IDLE;
          connHandle = GAP_CONNHANDLE_INIT;
          discState = BLE_DISC_STATE_IDLE;

          // TODO: Remember scan Results and re enable menu

          Display_print1(dispHandle, SBC_ROW_RESULT, 0, "Reason: %d", pEvent->gap.hdr.status);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;
        discState = BLE_DISC_STATE_IDLE;
        scanIdx = -1;

        // Throughput as well, if enabled
        Util_stopClock(&periodicLED);
        SimpleBLECentral_performLEDTask();

        // Cancel RSSI reads
        SimpleBLECentral_CancelRssi(pEvent->linkTerminate.connectionHandle);

        Display_print1(dispHandle, SBC_ROW_RESULT, 0, "Reason: %d", pEvent->linkTerminate.reason);
        Display_clearLine(dispHandle, SBC_ROW_PEER_DEVICE);
        Display_clearLine(dispHandle, SBC_ROW_PHY);
        Display_clearLine(dispHandle, SBC_ROW_RSSI);
        Display_clearLine(dispHandle, SBC_ROW_TEMP);

        // Go to Main Menu
        tbm_goTo(&sbcMenuMain);

        // Enable Scan Connect Menu, Disable everything else
        tbm_setItemStatus(&sbcMenuMain, TBM_ITEM_0, TBM_ITEM_1 | TBM_ITEM_2 | TBM_ITEM_3 | TBM_ITEM_4);

        // No Results, reenable scanning only
        tbm_setItemStatus(&sbcMenuScanandConnect, TBM_ITEM_ALL, TBM_ITEM_0 | TBM_ITEM_1);
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        Display_print1(dispHandle, SBC_ROW_RESULT, 0, "Param Update: %d", pEvent->linkUpdate.status);
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
  if (keys & KEY_LEFT)
  {
    // Check if the key is still pressed. WA for possible bouncing.
    if (PIN_getInputValue(Board_PIN_BUTTON0) == 0)
    {
      tbm_buttonLeft();
    }
  }

  if (keys & KEY_RIGHT)
  {
    // Check if the key is still pressed. WA for possible bouncing.
    if (PIN_getInputValue(Board_PIN_BUTTON1) == 0)
    {
      tbm_buttonRight();
    }
  }
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
      // Display_print1(dispHandle, SBC_ROW_GATT_RESULT, 0, "ATT Rsp dropped %d", pMsg->method);
    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        // Display_print1(dispHandle, SBC_ROW_GATT_RESULT, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        // Display_print1(dispHandle, SBC_ROW_GATT_RESULT, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
      }
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, SBC_ROW_RESULT, 0, "GATT Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful write, display the value that was written and
        // increment value
        Display_print0(dispHandle, SBC_ROW_RESULT, 0, "GATT Write Sent to Peer");
      }
    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.

      // Display the opcode of the message that caused the violation.
      // Display_print1(dispHandle, SBC_ROW_GATT_RESULT, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_HANDLE_VALUE_NOTI)
    {
      // Critial Section so our Timer's SWI can't read the value while
      // we're writing to it.
//      UInt key = Hwi_disable();
//      {
//        bytesRecvd += pMsg->msg.handleValueNoti.len;
//      }
//      Hwi_restore(key);
      Temperature_Service_Data *data = (Temperature_Service_Data *)pMsg->msg.handleValueNoti.pValue;

      Display_print1(dispHandle, SBC_ROW_TEMP, 0, "Object Temperature: %02d (C)",
                     BUILD_UINT16(data->objectLowByte, data->objectHighByte));
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      // MTU size updated
      Display_print0(dispHandle, SBC_ROW_RESULT, 0, "MTU Exchanged");
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
        int8 rssi = (int8)pMsg->pReturnParam[3];

        Display_print1(dispHandle, SBC_ROW_RSSI, 0, "RSSI -dBm: %d", (uint32_t)(-rssi));
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
  svcStartHdl = svcEndHdl = 0;

  // Free up memory for Characteristic handles
  if ( throughputHandles )
  {
    ICall_free(throughputHandles);
    throughputHandles = NULL;
  }

  if ( temperatureHandles )
  {
    ICall_free(temperatureHandles);
    temperatureHandles = NULL;
  }

  // Prep the State Machine for MTU Exchange
  discState = BLE_DISC_STATE_MTU;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
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
 * @fn      SimpleBLECentral_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  int i = 0;

  switch(discState)
  {
    case BLE_DISC_STATE_MTU:
      {
        // MTU size response received, discover simple BLE service
        if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
        {
          // Use the Discovery State Machine to get service start stop handles
          discState = BLE_DISC_STATE_THROUGHPUT_SERVICE;

          // UUID of Service to be Discovered
          uint8_t uuid[ATT_UUID_SIZE] = { TI_BASE_UUID_128(THROUGHPUT_SERVICE_SERV_UUID) };

          // Discovery throughput service
          VOID GATT_DiscPrimaryServiceByUUID(connHandle, uuid, ATT_UUID_SIZE,
                                             selfEntity);
        }
        else {
          //Do something else if we didn't get a RSP
        }
      }
      break;

    case BLE_DISC_STATE_THROUGHPUT_SERVICE:
      {
        // Service found, store handles
        if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
            pMsg->msg.findByTypeValueRsp.numInfo > 0)
        {
          svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
          svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        }

        if (((pMsg->hdr.status == bleProcedureComplete)  ||
             (pMsg->method == ATT_ERROR_RSP)) && svcStartHdl)
        {
            // The Throughput Service Exists!
            // At this point save valid svcStartHdl and svcEndHdl
            // A write can be performed if needed to a particular UUID
            // Defined in the profile's header file
            // Further discovery is needed to determine characteristic value handles

            // Set Statemachine to parse ATT_READ_BY_TYPE_RSP
            discState = BLE_DISC_STATE_THROUGHPUT_CHAR;

            // Find all characteristics within our throughput service
            VOID GATT_DiscAllChars(connHandle, svcStartHdl, svcEndHdl, selfEntity);
        }
      }
      break;

    case BLE_DISC_STATE_THROUGHPUT_CHAR:
      {
        // Characteristics found, store handles
        if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
            (pMsg->msg.readByTypeRsp.numPairs > 0))
        {
          // Allocate space for the handle UUID pairs
          throughputHandles = ICall_malloc(sizeof(CharProfileHdl_t) * pMsg->msg.readByTypeRsp.numPairs);

          // Note there are 16 bytes in a 128bit UUID + 2 bytes for the Handle
          // 18 bytes of information need to be copied
          // the remaining 3 bytes indiated in the length field is due to
          // Header information the stack uses.
          for (i = 0; i < pMsg->msg.readByTypeRsp.numPairs; i++)
          {
            // Due to the difference between the structure and the data given, apply some logic to
            // extract out the correct information (ie, ignore the 3 byte header)
            CharProfileHdl_t* temp = (CharProfileHdl_t*)((pMsg->msg.readByTypeRsp.pDataList + 3) + (pMsg->msg.readByTypeRsp.len * i));

            throughputHandles[i].charHdl = temp->charHdl;
            memcpy(throughputHandles[i].addr, temp->addr, ATT_UUID_SIZE);
          }
          // Now verify that the UUIDs are in the order the indexes are
          // We'll skip this step, as the way we implemented the profile on
          // the peripheral always responds with PDU first then PHY characteristics

          // This means we can index throughputHandles like throughputHandles[THROUGHPUT_SERVICE_UPDATE_PDU]
        }

        if (((pMsg->hdr.status == bleProcedureComplete)  ||
             (pMsg->method == ATT_ERROR_RSP)) && throughputHandles)
        {
          // Inform user that the Throughput Service is found, and ready to use
          Display_print0(dispHandle, SBC_ROW_RESULT, 0, "Throughput Service Found, Starting Throughput");

          // Set Statemachine to parse ATT_READ_BY_TYPE_RSP
          discState = BLE_DISC_STATE_TEMPERATURE_SERVICE;
          svcStartHdl = svcEndHdl = 0;

          // UUID of Service to be Discovered
          uint8_t uuid[ATT_UUID_SIZE] = { TI_BASE_UUID_128(TEMPERATURE_SERVICE_SERV_UUID) };

          // Discovery throughput service
          VOID GATT_DiscPrimaryServiceByUUID(connHandle, uuid, ATT_UUID_SIZE,
                                             selfEntity);
        }
      }
      break;

    case BLE_DISC_STATE_TEMPERATURE_SERVICE:
      {
        // Service found, store handles
        if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
            pMsg->msg.findByTypeValueRsp.numInfo > 0)
        {
          svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
          svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        }

        if (((pMsg->hdr.status == bleProcedureComplete)  ||
             (pMsg->method == ATT_ERROR_RSP)) && svcStartHdl)
        {
            // The Throughput Service Exists!
            // At this point save valid svcStartHdl and svcEndHdl
            // A write can be performed if needed to a particular UUID
            // Defined in the profile's header file
            // Further discovery is needed to determine characteristic value handles

            // Set Statemachine to parse ATT_READ_BY_TYPE_RSP
            discState = BLE_DISC_STATE_TEMPERATURE_CHAR;

            // Find all characteristics within our throughput service
            VOID GATT_DiscAllChars(connHandle, svcStartHdl, svcEndHdl, selfEntity);
        }
      }
      break;

    case BLE_DISC_STATE_TEMPERATURE_CHAR:
      {
        // Characteristics found, store handles
        if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
            (pMsg->msg.readByTypeRsp.numPairs > 0))
        {
          // Allocate space for the handle UUID pairs
          temperatureHandles = ICall_malloc(sizeof(CharProfileHdl_t) * pMsg->msg.readByTypeRsp.numPairs);

          // Note there are 16 bytes in a 128bit UUID + 2 bytes for the Handle
          // 18 bytes of information need to be copied
          // the remaining 3 bytes indiated in the length field is due to
          // Header information the stack uses.
          for (i = 0; i < pMsg->msg.readByTypeRsp.numPairs; i++)
          {
            // Due to the difference between the structure and the data given, apply some logic to
            // extract out the correct information (ie, ignore the 3 byte header)
            CharProfileHdl_t* temp = (CharProfileHdl_t*)((pMsg->msg.readByTypeRsp.pDataList + 3) + (pMsg->msg.readByTypeRsp.len * i));

            temperatureHandles[i].charHdl = temp->charHdl;
            memcpy(temperatureHandles[i].addr, temp->addr, ATT_UUID_SIZE);
          }
          // Now verify that the UUIDs are in the order the indexes are
          // We'll skip this step, as the way we implemented the profile on
          // the peripheral always responds with PDU first then PHY characteristics

          // This means we can index temperatureHandles like temperatureHandles[THROUGHPUT_SERVICE_UPDATE_PDU]
        }

        if (((pMsg->hdr.status == bleProcedureComplete)  ||
           (pMsg->method == ATT_ERROR_RSP)) && temperatureHandles)
        {
          // Inform user that the Throughput Service is found, and ready to use
          Display_print0(dispHandle, SBC_ROW_RESULT, 0, "Throughput & Temperature Services Found!");

          discState = BLE_DISC_STATE_IDLE;

          SimpleBLECentral_enqueueMsg(SBC_ENABLE_TEMPERATURE_NOTI_EVT, SUCCESS, NULL);
        }
      }
      break;

    default:
      {
        // Do something meaningful here if we have an issue
        discState = BLE_DISC_STATE_IDLE;
        // Inform user that the Throughput Service is found, and ready to use
        Display_print0(dispHandle, SBC_ROW_RESULT, 0, "Error in discovering profiles and services!");
      }
      break;
  }
}

/*
 * Local Function to quickly check the device list
 * For a particualr Address
 */
bool checkDevList(uint8_t* addr, uint8_t* index)
{
    int i = 0;

    for(i = 0; i < scanRes; i++)
    {
      if(memcmp(devList[i].addr, addr, B_ADDR_LEN) == 0)
      {
        *index = i;
        return true;
      }
    }

    return false;
}

/*********************************************************************
 * @fn      SimpleBLECentral_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *           Info added is: BDAddr and Scan Response Data
 *          It's assumed all data passed in will get freed
 *
 * @return  none
 */
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType,
                                           uint8_t dataType, uint8_t dataLen,
                                           uint8_t* data)
{
  uint8_t index;

  // If result count not at max
  if (scanRes < DEFAULT_MAX_SCAN_RES)
  {
    switch(dataType)
    {
      case GAP_ADRPT_ADV_IND:
      case GAP_ADRPT_ADV_SCAN_IND:

        if (SimpleBLECentral_findSvcUuid(THROUGHPUT_SERVICE_SERV_UUID,
                                         data, dataLen) &&
            SimpleBLECentral_findSvcUuid(TEMPERATURE_SERVICE_SERV_UUID,
                                         data, dataLen))
        {
          if (checkDevList(pAddr, &index) == false)
          {
            devList[scanRes].addrType = addrType;
            memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);

            scanRes++;
          }
        }
        break;

      case GAP_ADRPT_SCAN_RSP:
      case GAP_ADRPT_ADV_DIRECT_IND:
      case GAP_ADRPT_ADV_NONCONN_IND:
      default:
        /* Ignore all others */
        break;
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
  Event_post(syncEvent, SBC_START_DISCOVERY_EVT);
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
void SimpleBLECentral_keyChangeHandler(uint8 keys)
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
static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t state,
                                           void *pData)
{
  sbcEvt_t *pMsg = ICall_malloc(sizeof(sbcEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_PHYHandler
 *
 * @brief   RTOS clock handler for Coded PHY changes
 *
 * @param   a0 - RTOS clock arg0.
 *
 * @return  void
 */
void SimpleBLECentral_PHYHandler(UArg a0)
{
  // Check if we've changed to 1M before changing to the requested PHY
  if(phyIndex == 0 && phyConfirm)
  {
    // Because we are in a SWI, the UART Driver should not be used
    // Inform the Application task to send request
    SimpleBLECentral_enqueueMsg(SBC_PHY_UPDATE_EVT, SUCCESS, phyClock_phyIndex);
  }
  else
  {
    // We're still tring to get to a coded PHY
    // Restart the timer
    Util_restartClock(&startPHYClock, CODED_PHY_CHANGE_DELAY);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_doSetPhy
 *
 * @brief   Set PHY preference.
 *
 * @param   index - 0, 1, 2, 3
 *
 * @return  always true
 */
bool SimpleBLECentral_doSetPhy(uint8 index)
{
  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_CODED, HCI_PHY_CODED
  };

  // Swtich to determine PHY options (needed for coded S2 and S8 mode)
  switch(index)
  {
  case 0:
  case 1:
    phyOptions = HCI_PHY_OPT_NONE;
    break;
  case 2:
    phyOptions = HCI_PHY_OPT_S2;
    break;
  case 3:
    phyOptions = HCI_PHY_OPT_S8;
    break;
  }

  // Generate index to send over to peripheral
  uint8_t* data = ICall_malloc(sizeof(uint8_t));
  switch(phy[index])
  {
    case HCI_PHY_1_MBPS:
      *data = 0;
      break;

    case HCI_PHY_2_MBPS:
      *data = 1;
      break;

    case HCI_PHY_CODED:
      {
        if(phyOptions == HCI_PHY_OPT_S2)
          *data = 2;
        else if (phyOptions == HCI_PHY_OPT_S8)
          *data = 3;
      }
      break;
  }

  if( throughputHandles )
  {

    // Check if we're already using coded PHY - switch over to 1M
    // between in order to keep stability
    if(phyIndex != *data && *data >= 2 && phyIndex >= 2)
    {
      uint8_t* phy1M = ICall_malloc(sizeof(uint8_t));
      *phy1M = 0;
      SimpleBLECentral_enqueueMsg(SBC_PHY_UPDATE_EVT, SUCCESS, phy1M);

      // Start A Timer to trigger a Coded PHY change
      Util_restartClock(&startPHYClock, CODED_PHY_CHANGE_DELAY);

      // Assign the requested PHY to the payload of the PHY handler
      phyClock_phyIndex = data;
    }
    else
    {
      // Inform the Application to perform a GATT write with
      // the selected size - this will tell the peripehral to change PHY
      SimpleBLECentral_enqueueMsg(SBC_PHY_UPDATE_EVT, SUCCESS, data);
    }

  }
  else
  {
    // Set this device's Phy Preference on the current connection.
    HCI_LE_SetPhyCmd(connHandle, LL_PHY_USE_PHY_PARAM, phy[index], phy[index], phyOptions);

    // Set this device's PHY Perference on future connections by using:
    HCI_LE_SetDefaultPhyCmd(LL_PHY_USE_PHY_PARAM, phy[index], phy[index]);

    ICall_free(data);
  }

  // Go to Main Menu
  tbm_goTo(&sbcMenuMain);

  return true;
}

/*********************************************************************
 * @fn      SimpleBLECentral_doScanAndConnect
 *
 * @brief   Set PHY preference.
 *
 * @param   index - 0, 1, 2(, 3, 4)
 *
 * @return  always true
 */
bool SimpleBLECentral_doScanAndConnect(uint8 index)
{
  switch (index)
  {
    case 0:
      // SELECT NEXT DEVICE ON SCAN LIST
      SBC_NextDevice();

      break;

    case 1:
      // CONNECT TO SELECTED DEVICE
      SBC_ConnectToDevice();

      break;

    case 2:
      // SCAN FOR DEVICES

      // Disable Scanning until completed
      tbm_setItemStatus(&sbcMenuScanandConnect, TBM_ITEM_NONE, TBM_ITEM_ALL);

      // Indicate to the user that Scanning Has Started
      Display_print0(dispHandle, SBC_ROW_RESULT, 0, "Scanning...");

      // Clear the Device List
      SBC_ClearDeviceList();

      // Command to tell GAPRole to start scanning
      GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                    DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                    DEFAULT_DISCOVERY_WHITE_LIST);
      break;
  }

  return true;
}

/*********************************************************************
 * @fn      SimpleBLECentral_doToggleRSSI
 *
 * @brief   Toggle RSSI Readings
 *
 * @param   index (ignored)
 *
 * @return  always true
 */
bool SimpleBLECentral_doToggleRSSI(uint8 index)
{
  // Ignored
  (void)index;

  if (SimpleBLECentral_RssiFind(connHandle) == NULL)
  {
    Display_print0(dispHandle, SBC_ROW_RSSI, 0, "RSSI Starting");
    SimpleBLECentral_StartRssi(connHandle, DEFAULT_RSSI_PERIOD);
  }
  else
  {
    SimpleBLECentral_CancelRssi(connHandle);
    Display_print0(dispHandle, SBC_ROW_RSSI, 0, "RSSI Cancelled");
  }

  return true;
}

/*********************************************************************
 * @fn      SimpleBLECentral_doDisconnect
 *
 * @brief   Disconnect from current Connection
 *
 * @param   index (ignored)
 *
 * @return  always true
 */
bool SimpleBLECentral_doDisconnect(uint8 index)
{
  // Ignored
  (void)index;

  GAPCentralRole_TerminateLink(connHandle);

  return true;
}

/*********************************************************************
 * @fn      SBC_ClearDeviceList
 *
 * @brief   Clear the Device List and Display.
 *
 * @return  void
 */
void SBC_ClearDeviceList()
{
  // Clear the Device Display
  Display_clearLine(dispHandle, SBC_ROW_STATUS_2);
  Display_clearLine(dispHandle, SBC_ROW_STATUS_3);

  // Reset Scan Res indicating No Valid Scan data on Device List
  scanRes = 0;
  scanIdx = -1;
}

/*********************************************************************
 * @fn      SBC_NextDevice
 *
 * @brief   Select the next device on the Device List and Display.
 *
 * @return  void
 */
void SBC_NextDevice(){
  // Increment scanIndex to the next valid entry
  ((scanIdx + 1) == scanRes) ? scanIdx = 0 : scanIdx++;

  // Print the Device pointed to by the Index
  Display_print1(dispHandle, SBC_ROW_STATUS_2, 0, "Scanned Device %d", (scanIdx+1));
  Display_print0(dispHandle, SBC_ROW_STATUS_3, 0, Util_convertBdAddr2Str(devList[scanIdx].addr));
}

/*********************************************************************
 * @fn      SBC_ConnectToDevice
 *
 * @brief   Connect to the selected Device
 *
 * @return  void
 */
void SBC_ConnectToDevice()
{
  // connect to current device selected by scanIdx
  uint8_t *peerAddr = devList[scanIdx].addr;
  uint8_t addrType = devList[scanIdx].addrType;

  // GAP Role to Connecting
  state = BLE_STATE_CONNECTING;

  /* Change default sup. timeout */
  GAP_SetParamValue( TGAP_CONN_EST_SUPERV_TIMEOUT, 500 );

  GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                               DEFAULT_LINK_WHITE_LIST,
                               addrType, peerAddr);
}

/*********************************************************************
 * @fn      SimpleBLECentral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLECentral_performLEDTask(void)
{
  static uint32_t ledPin;

  if (state == BLE_STATE_CONNECTED) {
    ledPin = (ledPin) ? 0 : 1;
    PIN_setOutputValue(appPins, Board_RLED, 0);
  }
  else {
    ledPin = 0;
    PIN_setOutputValue(appPins, Board_RLED, 1);
  }

  PIN_setOutputValue(appPins, Board_GLED, ledPin);
}

/*********************************************************************
 * @fn      SimpleBLECentral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLECentral_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}

/*********************************************************************
*********************************************************************/
