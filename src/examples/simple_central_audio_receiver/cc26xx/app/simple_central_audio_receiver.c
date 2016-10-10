/*
 * Filename: simple_central_audio_receiver.c
 *
 * Description: This is the simple_central example modified to receive
 * data over BLE at a high throughput.
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
#include "gatt_profile_uuid.h"
#include "gatt.h"
#include "gatt_uuid.h"

#include "OSAL.h"
#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include "board_key.h"
#include <ti/mw/display/Display.h>
#include "board.h"

#include "simple_central.h"

#include "ble_user_config.h"

#ifdef AUDIO_SERVICE
#include "audio_profile.h"
#endif

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
/*********************************************************************
 * MACROS
 */
uint8 audioConfigEnable = 0;

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
#define SBC_SCANNING_TOGGLE_EVT               0x0040


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
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

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

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Change it to initialize pairing from central since sensortag audio project does not initial pairing
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000
// Default service discovery timer delay in ms
#define DEFAULT_SCANNING_TOGGLECLOCK          200

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                   864
#endif

#define APP_MIN_CONN_INTERVAL                 8      // 10ms, need short connection interval in order to reach the needed throughput for audio streaming
#define APP_MAX_CONN_INTERVAL                 16     // 20ms

#define APP_SLAVE_LATENCY                     0      // Initially 0 for fast connection. 49 slave latency (500ms effective interval)
#define APP_CONN_TIMEOUT                      500    // 1.6s supervision timeout

// Gap Bond Manager States
#define UNPAIRED_STATE                        0x00
#define PAIRED_BONDED_STATE                   0x01

#define TI_COMPANY_ID                         0x000D  // To maintain connectivity with SensorTag audio project

// Service Change flags
#define NO_CHANGE                             0x00
#define CHANGE_OCCURED                        0x01

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
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
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

typedef struct
{
  // Service and Characteristic discovery variables.
  uint16 keyCharHandle;
  uint16 keyCCCHandle;

#ifdef AUDIO_SERVICE
  uint16 audioStartCharValueHandle;
  uint16 audioDataCharValueHandle;
#endif
  uint8  lastRemoteAddr[B_ADDR_LEN];
} SimpleBLECentral_HandleInfo_t;


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

bool resultFindRC;
bool resultFindST;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;
// Clock object used to signal start/end scanning
static Clock_Struct scanningToggleClock;

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

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static bool scanningStarted = FALSE;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charHdl = 0;

// Maximum PDU size (default = 27 octets)
static uint16 maxPduSize;

// Array of RSSI read structures
static readRssi_t readRssi[MAX_NUM_BLE_CONNS];

/* Pin driver handles */
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State allPinState;

static uint8 remoteAddr[B_ADDR_LEN] = {0,0,0,0,0,0};

// Handle info saved here after connection to skip service discovery.
static SimpleBLECentral_HandleInfo_t remoteHandles;

static uint16 serviceToDiscover = GATT_INVALID_HANDLE;

static uint8 enableCCCDs = TRUE;

// Service and Characteristic discovery variables.

static uint16 keyCharHandle          = GATT_INVALID_HANDLE;

/* Audio START characteristic */
static uint16 audioStartCharValueHandle   = GATT_INVALID_HANDLE;
static uint16 audioStartCCCHandle         = GATT_INVALID_HANDLE;
/* Audio "Data" characteristic */
static uint16 audioDataCharValueHandle    = GATT_INVALID_HANDLE;
static uint16 audioDataCCCHandle         = GATT_INVALID_HANDLE;

// CCC's of the notifications
static uint16 keyCCCHandle           = GATT_INVALID_HANDLE;
static uint8 serviceDiscComplete = FALSE;

//static uint8 keyReportFound = FALSE;

/* UART driver */
static UART_Handle uartHandle;
static UART_Params uartParams;
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
static void SimpleBLECentral_startDiscovery(void);
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen);
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
;static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status);
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);

static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);

static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs);
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status);

void SimpleBLECentral_startDiscHandler(UArg a0);
void SimpleBLECentral_keyChangeHandler(uint8 keys);
void SimpleBLECentral_readRssiHandler(UArg a0);

static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t status,
                                           uint8_t *pData);

static uint8 SimpleBLECentral_FindHIDRemote( uint8* pData, uint8 length );
static void SimpleBLECentral_SetIdle( void );
static uint8 SimpleBLECentral_BondCount( void );
static void SimpleBLECentral_EstablishLink( uint8 whiteList, uint8 addrType, uint8 *remoteAddr );
static void SimpleBLECentral_EnableNotification( uint16 connHandle, uint16 attrHandle );
static void SimpleBLECentral_DiscoverService( uint16 connHandle, uint16 svcUuid );
static void SimpleBLECentral_SaveHandles( void );

static void SimpleBLECentral_scanningToggleHandler(UArg a0);

PIN_Config ledPinTable[] = {
    Board_RLED   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,         /* LED initially off             */
    Board_GLED   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,         /* LED initially off             */
  PIN_TERMINATE
};


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t SimpleBLECentral_roleCB =
{
  SimpleBLECentral_eventCB     // Event callback
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

  // Open all pins
  ledPinHandle = PIN_open(&allPinState, ledPinTable);

  // Turn on Red led to indicate nothing has been connected
  PIN_setOutputValue( ledPinHandle, Board_RLED, 1);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, SimpleBLECentral_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);

  // Periodic event to toggle LED to indicate if scanning is in progress
  Util_constructClock(&scanningToggleClock, SimpleBLECentral_scanningToggleHandler,
                      DEFAULT_SCANNING_TOGGLECLOCK, 0, false, SBC_SCANNING_TOGGLE_EVT);

  // Init UART and specify non-default parameters
  UART_Params_init(&uartParams);
  uartParams.baudRate      = 400000;
  uartParams.writeDataMode = UART_DATA_BINARY;
  // Open the UART and do the write
  uartHandle = UART_open(Board_UART, &uartParams);


  Board_initKeys(SimpleBLECentral_keyChangeHandler);

  dispHandle = Display_open(Display_Type_GRLIB, NULL);

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
    uint8_t failAction = GAPBOND_FAIL_TERMINATE_ERASE_BONDS;
    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_BOND_FAIL_ACTION,sizeof(uint8_t), &failAction);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Set connection parameters:
  {
    uint16 minconnectionInterval = APP_MIN_CONN_INTERVAL;
    uint16 maxconnectionInterval = APP_MAX_CONN_INTERVAL;

    uint16 slaveLatency = APP_SLAVE_LATENCY;
    uint16 timeout = APP_CONN_TIMEOUT;

    VOID GAP_SetParamValue( TGAP_CONN_EST_INT_MIN, minconnectionInterval );
    VOID GAP_SetParamValue( TGAP_CONN_EST_INT_MAX, maxconnectionInterval );
    VOID GAP_SetParamValue( TGAP_CONN_EST_LATENCY, slaveLatency );
    VOID GAP_SetParamValue( TGAP_CONN_EST_SUPERV_TIMEOUT, timeout );

  }

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

  Display_print0(dispHandle, 0, 0, "Audio Central");
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

    if (events & SBC_SCANNING_TOGGLE_EVT)
    {
      events &= ~SBC_SCANNING_TOGGLE_EVT;

      Util_startClock(&scanningToggleClock);
      PIN_setOutputValue(ledPinHandle, Board_GLED, !PIN_getOutputValue(Board_GLED));
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
  static uint8 addrType;
  static uint8 peerDeviceFound = FALSE;

  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        maxPduSize = pEvent->initDone.dataPktLen;

        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
        Display_print0(dispHandle, 2, 0, "Initialized");
        if ( SimpleBLECentral_BondCount() > 0 )
        {
          // Initiate connection
          SimpleBLECentral_EstablishLink( TRUE, addrType, remoteAddr );
        }
        else
        {
          // Sit idle till ask to scan
          SimpleBLECentral_SetIdle();
        }
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        {
          if ((SimpleBLECentral_findSvcUuid( TI_COMPANY_ID,
                                            pEvent->deviceInfo.pEvtData,
                                            pEvent->deviceInfo.dataLen)) ||
              (SimpleBLECentral_findSvcUuid( HID_SERV_UUID,
                                            pEvent->deviceInfo.pEvtData,
                                            pEvent->deviceInfo.dataLen)) )
          {
            SimpleBLECentral_addDeviceInfo(pEvent->deviceInfo.addr,
                                           pEvent->deviceInfo.addrType);
            addrType = pEvent->deviceInfo.addrType;
            osal_memcpy( remoteAddr, pEvent->deviceInfo.addr, B_ADDR_LEN );
            peerDeviceFound = TRUE;
          }
        }
        if ( ( peerDeviceFound == TRUE ) &&
            ( pEvent->deviceInfo.eventType == GAP_ADRPT_SCAN_RSP ) &&
              SimpleBLECentral_FindHIDRemote( pEvent->deviceInfo.pEvtData,
                                  pEvent->deviceInfo.dataLen ) )
        {
          // End device discovery
          VOID GAPCentralRole_CancelDiscovery();
        }
      }
      break;

  case GAP_DEVICE_DISCOVERY_EVENT:
    {
      Util_stopClock(&scanningToggleClock);
      PIN_setOutputValue(ledPinHandle, Board_GLED, 0);
      // discovery complete
      scanningStarted = FALSE;
      // If we have found a connectable device, establish a connection
      if ( peerDeviceFound == TRUE )
      {

        SimpleBLECentral_EstablishLink( FALSE, addrType, remoteAddr );

        peerDeviceFound = FALSE;
        scanRes = 0;
      }
      else if ( scanRes > 0 )
      {
        scanRes--;

        // Scan again
        // Begin scanning
        GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST );
      }
      else
      {
        // Go idle
        SimpleBLECentral_SetIdle();
      }
    }
    break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;

          // If service discovery not performed initiate service discovery
          if (charHdl == 0)
          {
            Util_startClock(&startDiscClock);
          }

          Display_print0(dispHandle, 2, 0, "Connected");
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));
          PIN_setOutputValue(ledPinHandle, Board_GLED, 1);
          PIN_setOutputValue(ledPinHandle, Board_RLED, 0);
        }
        else if ( SimpleBLECentral_BondCount() > 0 )
        {
          // Re-initiate connection
          SimpleBLECentral_EstablishLink( TRUE, addrType, remoteAddr );
        }
        else
        {
          connHandle = GAP_CONNHANDLE_INIT;

          // Go idle
          SimpleBLECentral_SetIdle();
          Display_print0(dispHandle, 2, 0, "Connect Failed");
          Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->gap.hdr.status);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {

        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;
        charHdl = 0;

        Display_print0(dispHandle, 2, 0, "Disconnected");
        Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->linkTerminate.reason);
        Display_clearLine(dispHandle, 4);

        if ( serviceDiscComplete == TRUE )
        {
          // Remember the address of the last connected remote
          osal_memcpy( remoteHandles.lastRemoteAddr, remoteAddr, B_ADDR_LEN );

          // Save handle information
          SimpleBLECentral_SaveHandles();
        }


        // Invalidate service discovery variables.
        serviceDiscComplete    = FALSE;
        keyCharHandle          = GATT_INVALID_HANDLE;

        keyCCCHandle           = GATT_INVALID_HANDLE;
        serviceToDiscover      = GATT_INVALID_HANDLE;

#ifdef AUDIO_SERVICE
        audioStartCharValueHandle  = GATT_INVALID_HANDLE;
        audioStartCCCHandle        = GATT_INVALID_HANDLE;
        audioDataCharValueHandle   = GATT_INVALID_HANDLE;
        audioDataCCCHandle         = GATT_INVALID_HANDLE;
#endif
        PIN_setOutputValue(ledPinHandle, Board_GLED, 0);
        PIN_setOutputValue(ledPinHandle, Board_RLED, 1);
        enableCCCDs = TRUE;
        if ( SimpleBLECentral_BondCount() > 0 )
        {
          // Re-initiate connection
          SimpleBLECentral_EstablishLink( TRUE, addrType, remoteAddr );
        }
        else
        {
          // Go idle
          SimpleBLECentral_SetIdle();
        }
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        Display_print1(dispHandle, 2, 0, "Param Update: %d", pEvent->linkUpdate.status);
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
  (void)shift;  // Intentionally unreferenced parameter

  if (keys & KEY_LEFT)
  {
    // Start or stop discovery
    if  (state == BLE_STATE_IDLE )
    {
      if (!scanningStarted)
      {
        scanningStarted = TRUE;
        scanRes = 0;

        Display_print0(dispHandle, 2, 0, "Discovering...");
        Display_clearLines(dispHandle, 3, 5);
        PIN_setOutputValue( ledPinHandle, Board_RLED, 0);

        GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST);

        Util_startClock(&scanningToggleClock);
      }
    }
    else if (state == BLE_STATE_CONNECTED )
    {
      state = BLE_STATE_DISCONNECTING;
      VOID GAPCentralRole_TerminateLink( connHandle );
      PIN_setOutputValue( ledPinHandle, Board_GLED, 0);
    }

    return;
  }

  if (keys & KEY_RIGHT)
  {
    // If bonds exist, erase all of them
    if ( ( SimpleBLECentral_BondCount() > 0 ) && ( state != BLE_STATE_CONNECTED ) )
    {
      if ( state == BLE_STATE_CONNECTING )
      {
        state = BLE_STATE_DISCONNECTING;
        VOID GAPCentralRole_TerminateLink( GAP_CONNHANDLE_INIT );
      }

      VOID GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, NULL );
      Display_print0(dispHandle, 5, 0, "Erase Bond info ");
    }

    return;
  }

}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static uint8_t counter;
static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (state == BLE_STATE_CONNECTED)
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      Display_print1(dispHandle, 4, 0, "ATT Rsp dropped %d", pMsg->method);
    };

    switch ( pMsg->method )
    {
    case ATT_HANDLE_VALUE_NOTI:    
      if (pMsg->msg.handleValueNoti.handle == audioDataCharValueHandle) {
        // Output the receive packets through UART, and use audio_frame_serial_print.py to decode
        UART_write(uartHandle, (pMsg->msg.handleValueNoti.pValue) , 20);
        counter++;

        if (counter == 50){
          PIN_setOutputValue(ledPinHandle, Board_RLED, !PIN_getOutputValue(Board_RLED));
          counter = 0;
        }

      }
      break;

    case ATT_FIND_BY_TYPE_VALUE_RSP:
      // Response from GATT_DiscPrimaryServiceByUUID
      // Service found, store handles
      if ( pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        svcStartHdl =
          ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        svcEndHdl =
          ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      }
      // If procedure complete
      else if ( pMsg->hdr.status == bleProcedureComplete )
      {
        if ( svcStartHdl != 0 )
        {

#ifdef AUDIO_SERVICE
          if ( serviceToDiscover == AUDIO_SERV_UUID)
          {
            // Discover all characteristics
            GATT_DiscAllChars( connHandle, svcStartHdl, svcEndHdl, selfEntity );
          }
#endif

        }
      }
      break;

    case ATT_ERROR_RSP:

      if (serviceToDiscover ==  AUDIO_SERV_UUID
            && pMsg->msg.errorRsp.reqOpcode == ATT_FIND_BY_TYPE_VALUE_REQ
            && pMsg->msg.errorRsp.handle == 0x0001)
            //0x0001 is the start attribute handle of 0xfff0, AUDIO_SERV_UUID
      {

          if ( (enableCCCDs == TRUE) && (keyCharHandle != GATT_INVALID_HANDLE))
          {
            keyCCCHandle = keyCharHandle + 1;
            // Begin configuring the characteristics for notifications
            SimpleBLECentral_EnableNotification( connHandle, keyCCCHandle );
          }
      }
      break;

    case ATT_READ_BY_TYPE_RSP:
      // Response from Discover all Characteristics.
      // Success indicates packet with characteristic discoveries.
      if ( pMsg->hdr.status == SUCCESS )
      {
        attReadByTypeRsp_t *pRsp = &pMsg->msg.readByTypeRsp;

        if( serviceToDiscover ==  AUDIO_SERV_UUID )
        {
          uint16 charUUID = GATT_INVALID_HANDLE;
          uint16 *pHandle = &charUUID;
          /* Write into charUUID what Audio Profile char value we're dealing with */
          *pHandle = BUILD_UINT16( pRsp->pDataList[17] , pRsp->pDataList[18]);
          if      (charUUID == AUDIOPROFILE_START_UUID) {
            pHandle = &audioStartCharValueHandle;
            *pHandle = BUILD_UINT16( pRsp->pDataList[3] , pRsp->pDataList[4]);
          }
          else if (charUUID == AUDIOPROFILE_AUDIO_UUID ){
            pHandle = &audioDataCharValueHandle;
            *pHandle = BUILD_UINT16( pRsp->pDataList[3] , pRsp->pDataList[4]);                     
          }
        }
        break;
      }

      // This indicates that there is no more characteristic data
      // to be discovered within the given handle range.
      else if ( pMsg->hdr.status == bleProcedureComplete )
      {
        if ( serviceToDiscover == AUDIO_SERV_UUID )
        {
          counter = 0;
          /* This kicks off the enabling the 1st of notification enable event */
          if (audioStartCharValueHandle != GATT_INVALID_HANDLE) {
            audioStartCCCHandle = audioStartCharValueHandle + 1 ;
            SimpleBLECentral_EnableNotification( connHandle, audioStartCCCHandle );
          }
          break;
        }

      }
      break;

    case ATT_WRITE_RSP:
      if ( pMsg->hdr.status == SUCCESS && !serviceDiscComplete )
      {
          uint16 handle = GATT_INVALID_HANDLE;

          // Chain the CCCD enable writes so that a RSP for one triggers the next enable.
          if (audioStartCCCHandle == GATT_INVALID_HANDLE) {
            handle = audioStartCCCHandle = audioStartCharValueHandle + 1;
          }
          else if (audioDataCCCHandle == GATT_INVALID_HANDLE) {
            handle = audioDataCCCHandle = audioDataCharValueHandle + 1;
          }
          else if (keyCCCHandle == GATT_INVALID_HANDLE ) {
            handle = keyCCCHandle = keyCharHandle + 1;
          }
          else {
            serviceDiscComplete = TRUE;
            break;
          }

          SimpleBLECentral_EnableNotification( connHandle, handle );

          break;

      }

      break;



    // Service Change indication
    case ATT_HANDLE_VALUE_IND:
      // Note: this logic assumes that the only indications that will be sent
      //       will come from that GATT Service Changed Characteristic
      if ( pMsg->hdr.status == SUCCESS )
      {

        // Acknowledge receipt of indication
        ATT_HandleValueCfm( pMsg->connHandle );

      }
      break;

    default:
      // Unknown event
      break;
    } //switch
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

        Display_print1(dispHandle, 4, 0, "RSSI -dB: %d", (uint32_t)(-rssi));
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status)
{
  Display_clearLines(dispHandle, 5, 5);
  switch (state)
  {
  case GAPBOND_PAIRING_STATE_STARTED:
    Display_print0(dispHandle, 2, 0, "Pairing started");
    break;

  case GAPBOND_PAIRING_STATE_COMPLETE:
    if (status == SUCCESS)
    {
      // Enter a GAP Bond manager Paired state
      Display_print0(dispHandle, 2, 0, "Pairing success");

      // Begin Service Discovery of AUDIO Service to find out report handles
      serviceToDiscover = AUDIO_SERV_UUID;
      SimpleBLECentral_DiscoverService( connHandle, serviceToDiscover );
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
    }
    break;

  case GAPBOND_PAIRING_STATE_BOND_SAVED:
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bond Saved");
    }
    break;

  case GAPBOND_PAIRING_STATE_BONDED:
    if (status == SUCCESS)
    {

      if ( osal_memcmp( remoteHandles.lastRemoteAddr, remoteAddr, B_ADDR_LEN ) == TRUE  )
      {

        if (
#ifdef AUDIO_SERVICE
            ( remoteHandles.audioStartCharValueHandle == GATT_INVALID_HANDLE )         ||
              ( remoteHandles.audioDataCharValueHandle == GATT_INVALID_HANDLE )
#endif
                )
        {
          serviceToDiscover = AUDIO_SERV_UUID;

          // We must perform service discovery again, something might have changed.
          // Begin Service Discovery
          SimpleBLECentral_DiscoverService( connHandle, serviceToDiscover );

          serviceDiscComplete = FALSE;
          audioConfigEnable =0; //This will re-trig an audio configuration if needed
        }
        else
        {
          // No change, restore handle info.
          // bonding indicates that we probably already enabled all these characteristics. easy fix if not.
          serviceDiscComplete    = TRUE;

#ifdef AUDIO_SERVICE
          audioStartCharValueHandle = remoteHandles.audioStartCharValueHandle;
          audioDataCharValueHandle  = remoteHandles.audioDataCharValueHandle;

          //Still, Force update of Audio config for now...
          audioConfigEnable =1;
#endif
        }
      }
      else if ( osal_isbufset( remoteHandles.lastRemoteAddr, 0x00, B_ADDR_LEN ) == TRUE )
      {
        // lastRemoteAddr is all 0's, which means the device was bonded before
        // it was power-cycled, and that we probably already enabled all CCCDs.
        // So, we only need to find out attribute report handles.
        enableCCCDs = FALSE;

        // Begin Service Discovery of HID Service to find out report handles
        serviceToDiscover = AUDIO_SERV_UUID;
        SimpleBLECentral_DiscoverService( connHandle, serviceToDiscover );
      }

      Display_print0(dispHandle, 2, 0, "Bond save success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
    }
    break;

  default:
    break;
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
    Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
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
          (adType == GAP_ADTYPE_16BIT_COMPLETE) ||
          (adType == GAP_ADTYPE_MANUFACTURER_SPECIFIC))
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
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    SimpleBLECentral_enqueueMsg(SBC_PAIRING_STATE_EVT, state, pData);
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
                                           uint8_t *pData)
{
  sbcEvt_t *pMsg = ICall_malloc(sizeof(sbcEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_FindHIDRemote
 *
 * @brief   Search Scan Response data for a "HID AdvRemote"
 *
 * @param   pData - received advertising data
 * @param   dataLen - advertising data length
 *
 * @return  TRUE if found, false otherwise
 */
static uint8 SimpleBLECentral_FindHIDRemote( uint8* pData, uint8 length )
{
  resultFindRC = FALSE;
  resultFindST = FALSE;
  static uint8 remoteNameRC[] =
  {
    'C', 'C', '2', '6', '5', '0', ' ', 'R', 'C'
  };

  // move pointer to the start of the scan response data.
  pData += 2;

  // adjust length as well
  length -= 2;

  resultFindRC = osal_memcmp( remoteNameRC, pData, length );

  // did not find RC, then search for ST
  if (!resultFindRC){

    static uint8 remoteNameST[] =
    {
      'C', 'C', '2', '6', '5', '0', ' ',
      'S', 'e', 'n',  's',  'o',  'r',  'T',  'a',  'g'
    };

    //    // complete name
    //  0x11,   // length of this data
    //  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    //  'C', 'C', '2', '6', '5', '0', ' ',
    //  'S', 'e', 'n',  's',  'o',  'r',  'T',  'a',  'g',
    //
    //  // connection interval range
    //  0x05,   // length of this data
    //  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    //  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    //  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    //  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
    //  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
    //
    //  // Tx power level
    //  0x02,   // length of this data
    //  GAP_ADTYPE_POWER_LEVEL,
    //  0       // 0dBm

    // adjust length as well, totla length is 0x11 + 0xA = 0x1B = 27
    // the needed array size(for the name) is only 16 bytes
//    length -= 11; // already moved when search for RC
    length -= 9;
    resultFindST = osal_memcmp( remoteNameST, pData, length );
  }
  return (resultFindRC || resultFindST);
}

/*********************************************************************
 * @fn      SimpleBLECentral_BondCount
 *
 * @brief   Gets the total number of bonded devices.
 *
 * @param   none.
 *
 * @return  number of bonded devices.
 */
static uint8 SimpleBLECentral_BondCount( void )
{
  uint8 bondCnt = 0;

  VOID GAPBondMgr_GetParameter( GAPBOND_BOND_COUNT, &bondCnt );

  return ( bondCnt );
}

/*********************************************************************
 * @fn      SimpleBLECentral_SetIdle
 *
 * @brief   Set the device to idle.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_SetIdle( void )
{
  state = BLE_STATE_IDLE;
  Util_stopClock(&scanningToggleClock);
  PIN_setOutputValue( ledPinHandle, Board_GLED, 0);
  PIN_setOutputValue( ledPinHandle, Board_RLED, 1);
  Display_print0(dispHandle, 2, 0, "Idle...");
}

/*********************************************************************
 * @fn      SimpleBLECentral_EstablishLink
 *
 * @brief   Establish a link to a peer device.
 *
 * @param   whiteList - determines use of the white list
 * @param   addrType - address type of the peer devic
 * @param   remoteAddr - peer device address
 *
 * @return  none
 */
static void SimpleBLECentral_EstablishLink( uint8 whiteList, uint8 addrType, uint8 *remoteAddr )
{
  if ( state != BLE_STATE_CONNECTED )
  {
    state = BLE_STATE_CONNECTING;

    // Try to connect to remote device
    GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                 DEFAULT_LINK_WHITE_LIST,
                                 addrType, remoteAddr);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_DiscoverService
 *
 * @brief   Discover service using UUID.
 *
 * @param   connHandle - connection handle to do discovery on
 * @param   svcUuid - service UUID to discover
 *
 * @return  none
 */
static void SimpleBLECentral_DiscoverService( uint16 connHandle, uint16 svcUuid )
{
  if(svcUuid == AUDIO_SERV_UUID) // only take care of Audio Service in this project
  {
    uint8 uuid[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0,
                      0x00, 0x40, 0x51, 0x04, LO_UINT16( svcUuid ), HI_UINT16( svcUuid ), 0x00, 0xF0};

    VOID GATT_DiscPrimaryServiceByUUID( connHandle, uuid, ATT_UUID_SIZE, selfEntity );
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_EnableNotification
 *
 * @brief   Enable notification for a given attribute handle.
 *
 * @param   connHandle - connection handle to send notification on
 * @param   attrHandle - attribute handle to send notification for
 *
 * @return  none
 */
static void SimpleBLECentral_EnableNotification( uint16 connHandle, uint16 attrHandle )
{
  attWriteReq_t req;

  req.pValue = GATT_bm_alloc( connHandle, ATT_WRITE_REQ, 2, NULL );
  if ( req.pValue != NULL )
  {
    uint8 notificationsOn[] = {0x01, 0x00};

    req.handle = attrHandle;

    req.len = 2;
    osal_memcpy(req.pValue, notificationsOn, 2);

    req.sig = 0;
    req.cmd = 0;

    if ( GATT_WriteCharValue( connHandle, &req, selfEntity ) != SUCCESS )
    {
      GATT_bm_free( (gattMsg_t *)&req, ATT_WRITE_REQ );
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_SaveHandles
 *
 * @brief   save handle information in case next connection is to the
 *          same bonded device.
 *
 * @param   none.
 *
 * @return  none.
 */
static void SimpleBLECentral_SaveHandles( void )
{
  // Service and Characteristic discovery variables.
  remoteHandles.keyCharHandle          = keyCharHandle;

//  // CCC's of the notifications
//  remoteHandles.keyCCCHandle           = keyCCCHandle;

#ifdef AUDIO_SERVICE
  remoteHandles.audioStartCharValueHandle = audioStartCharValueHandle;
  remoteHandles.audioDataCharValueHandle  = audioDataCharValueHandle;
#endif
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLECentral_scanningToggleHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
*********************************************************************/
