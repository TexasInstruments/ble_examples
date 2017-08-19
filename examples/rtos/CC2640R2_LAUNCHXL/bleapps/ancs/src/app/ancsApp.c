/******************************************************************************

 @file       ancsApp.c

 @brief This file contains the ANCS Application sample application for use
        with the CC2640R2 Bluetooth Low Energy Protocol Stack.

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
 Release Name: simplelink_cc2640r2_sdk_1_40_00_45
 Release Date: 2017-07-20 17:16:59
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

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "devinfoservice.h"

#include "ll_common.h"

#include "peripheral.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include "board.h"

// ANCS App includes.
#include "board_key.h"
#include "ancsApp.h"
#include "ancs.h"

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL                  160

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE                     GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL             6

// Maximum connection interval (units of 1.25ms, 800=1000ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL             6

// Slave latency to use for automatic parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY                 0

// Supervision timeout value (units of 10ms, 1000=10s) for automatic parameter
// update request
#define DEFAULT_DESIRED_CONN_TIMEOUT                  1000

// After the connection is formed, the peripheral waits until the central
// device asks for its preferred connection parameters
#define DEFAULT_ENABLE_UPDATE_REQUEST                 GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL                 6

// How often to perform periodic event (in msec)
#define ANCSAPP_PERIODIC_EVT_PERIOD                   5000

// Application specific event ID for HCI Connection Event End Events
#define ANCSAPP_HCI_CONN_EVT_END_EVT                  0x0001

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
    #define ANCSAPP_DISPLAY_TYPE Display_Type_LCD
  #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
    #define ANCSAPP_DISPLAY_TYPE Display_Type_UART
  #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
    #define ANCSAPP_DISPLAY_TYPE 0 // Option not supported
  #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#else // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #define ANCSAPP_DISPLAY_TYPE 0 // No Display
#endif // !Display_DISABLE_ALL

// Task configuration
#define ANCSAPP_TASK_PRIORITY                         1

#ifndef ANCSAPP_TASK_STACK_SIZE
#define ANCSAPP_TASK_STACK_SIZE                       644
#endif

#define IO_BUTTON_LEFT                                8

#define IO_BUTTON_RIGHT                               16

#define IO_BUTTON_BOTH                                24

// ANCS: 7905F431-B5CE-4E99-A40F-4B1E122D00D0
#define ANCSAPP_ANCS_SVC_UUID 0xD0, 0x00, 0x2D, 0x12, 0x1E, 0x4B, 0x0F, 0xA4, 0x99, 0x4E, 0xCE, 0xB5, 0x31, 0xF4, 0x05, 0x79
// Notification Source: UUID 9FBF120D-6301-42D9-8C58-25E699A21DBD (notifiable)
#define ANCSAPP_NOTIF_SRC_CHAR_UUID                   0x1DBD 
// Control point: UUID 69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9 (writable with response)
#define ANCSAPP_CTRL_PT_CHAR_UUID                     0xD9D9
// Data Source: UUID 22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB (notifiable)
#define ANCSAPP_DATA_SRC_CHAR_UUID                    0x7BFB

#define CHAR_DESC_HDL_UUID128_LEN                     21 // (5 + 16) bytes = 21 bytes.

#define NUMBER_OF_ANCS_CHARS                          3

#define LAST_ANCS_CHAR                                1

#ifdef USE_WATCHDOG_TIMER
  #define WATCHDOG_TIMER_TIMEOUT_PERIOD                 1500000 * 5 // 1 second * 5 
  #define ANCSAPP_PERIODIC_EVT                          Event_Id_02
#endif

// Application events
#define ANCSAPP_STATE_CHANGE_EVT                      0x0001
#define ANCSAPP_CHAR_CHANGE_EVT                       0x0002
#define ANCSAPP_PAIRING_STATE_EVT                     0x0004
#define ANCSAPP_PASSCODE_NEEDED_EVT                   0x0008

#define ANCSAPP_START_DISC_EVT                        Event_Id_00
#define ANCSAPP_KEY_CHANGE_EVT                        Event_Id_01


// Internal Events for RTOS application.
#define ANCSAPP_ICALL_EVT                             ICALL_MSG_EVENT_ID // Event_Id_31
#define ANCSAPP_QUEUE_EVT                             UTIL_QUEUE_EVENT_ID // Event_Id_30


// Bitwise OR of all events to pend on.
#ifdef USE_WATCHDOG_TIMER
  #define ANCSAPP_ALL_EVENTS                           (ANCSAPP_ICALL_EVT        | \
                                                        ANCSAPP_QUEUE_EVT        | \
                                                        ANCSAPP_KEY_CHANGE_EVT   | \
                                                        ANCSAPP_START_DISC_EVT   | \
                                                        ANCSAPP_PERIODIC_EVT)
#else
  #define ANCSAPP_ALL_EVENTS                           (ANCSAPP_ICALL_EVT        | \
                                                        ANCSAPP_QUEUE_EVT        | \
                                                        ANCSAPP_KEY_CHANGE_EVT   | \
                                                        ANCSAPP_START_DISC_EVT)
#endif

// Application states
enum
{
  ANCS_STATE_IDLE = 0,
  ANCS_STATE_DISCOVERY,
  ANCS_STATE_READY,
};

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
  uint8_t *pData;  // event data
} ancsAppEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Store discovered handles
uint16_t Ancs_handleCache[HDL_CACHE_LEN];

// Display Interface
Display_Handle dispHandle = NULL;

// Watchdog handle
Watchdog_Handle watchdogHandle;
/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8_t discoveryState = ANCS_EXCHANGE_MTU;

// The handle Cache of the ANCS.
static uint8_t ancsAppState = ANCS_STATE_IDLE;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

#ifdef USE_WATCHDOG_TIMER
  // Clock instances for internal periodic events.
  static Clock_Struct periodicClock;
#endif

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct ancsAppTask;
Char ancsAppTaskStack[ANCSAPP_TASK_STACK_SIZE];

// Scan response data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x0A,// length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE, 'A', 'N', 'C', 'S', ' ', 'D', 'e', 'm', 'o',

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

// Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,// length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  // Service Solicitation: this peripheral (NC) is looking for the ANCS
  // on the iOS device. As per Apple Bluetooth Design Guidelines, soliciting
  // the ANCS will cause the device to show up in the iOS settings app.
  0x11, // length of this data
  GAP_ADTYPE_SERVICES_LIST_128BIT,
  // The ANCS's UUID.
  ANCSAPP_ANCS_SVC_UUID
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "ANCS Demo";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void AncsApp_init( void );
static void AncsApp_taskFxn(UArg a0, UArg a1);
static uint8_t AncsApp_processStackMsg(ICall_Hdr *pMsg);
static uint8_t AncsApp_processGATTMsg(gattMsgEvent_t *pMsg);
static void AncsApp_processAppMsg(ancsAppEvt_t *pMsg);
static void AncsApp_processStateChangeEvt(gaprole_States_t newState);
static void AncsApp_sendAttRsp(void);
static void AncsApp_freeAttRsp(uint8_t status);
static void AncsApp_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle, uint8_t uiInputs, uint8_t uiOutputs);
static void AncsApp_pairStateCB(uint16_t connHandle, uint8_t state, uint8_t status);
static void AncsApp_processPairState(uint8_t state, uint8_t status);
static void AncsApp_processPasscode(uint8_t uiOutputs);
static void AncsApp_stateChangeCB(gaprole_States_t newState); 
static uint8_t AncsApp_enqueueMsg(uint8_t event, uint8_t state, uint8_t *pData);
/********************ANCS APP FUNCTIONS********************/
// Board I/O
static void AncsApp_keyPressCB(uint8 keys);
static void AncsApp_handleKeysEvt(uint8_t keys);

// ANCS service discovery
static void AncsApp_discoverService(gattMsgEvent_t *pMsg);

#ifdef USE_WATCHDOG_TIMER
  /********************WATCHDOG FUNCTIONS********************/
  static void watchdogCallback(uintptr_t unused);
  static void AncsApp_performPeriodicTask(void);
  static void AncsApp_clockHandler(UArg arg);
#endif

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Peripheral GAPRole Callbacks
static gapRolesCBs_t ancsApp_gapRoleCBs =
{
  AncsApp_stateChangeCB     // GAPRole State Change Callbacks
};

// GAP Bond Manager Callbacks
// These are set to NULL since they are not needed. The application
// is set up to only perform justworks pairing.
static gapBondCBs_t ancsApp_BondMgrCBs =
{
  (pfnPasscodeCB_t) AncsApp_passcodeCB, // Passcode callback
  AncsApp_pairStateCB                   // Pairing / Bonding state Callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AncsApp_createTask
 *
 * @brief   Task creation function for the ANCS app.
 *
 * @param   None.
 *
 * @return  None.
 */
void AncsApp_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = ancsAppTaskStack;
  taskParams.stackSize = ANCSAPP_TASK_STACK_SIZE;
  taskParams.priority = ANCSAPP_TASK_PRIORITY;

  Task_construct(&ancsAppTask, AncsApp_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      AncsApp_init
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
static void AncsApp_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

       HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_21_DBM);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

#if defined( USE_FPGA )
  // configure RF Core SMI Data Link
  IOCPortConfigureSet(IOID_12, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_11, IOC_PORT_RFC_GPI0, IOC_STD_INPUT);

  // configure RF Core SMI Command Link
  IOCPortConfigureSet(IOID_10, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_OUT, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_9, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_IN, IOC_STD_INPUT);

  // configure RF Core tracer IO
  IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT);
#else // !USE_FPGA
  #if defined( DEBUG_SW_TRACE )
    // configure RF Core tracer IO
    IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);
  #endif // DEBUG_SW_TRACE
#endif // USE_FPGA

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Initialize the CC2640r2's I/O 
  Board_initKeys(AncsApp_keyPressCB);

  dispHandle = Display_open(ANCSAPP_DISPLAY_TYPE, NULL);

#ifdef USE_WATCHDOG_TIMER
  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, AncsApp_clockHandler,
                      ANCSAPP_PERIODIC_EVT_PERIOD, 0, false, ANCSAPP_PERIODIC_EVT);
  
  // Call watchdog init functions
  Watchdog_init();
  Watchdog_Params params;
  params.callbackFxn    = (Watchdog_Callback)watchdogCallback;
  params.resetMode      = Watchdog_RESET_ON;
  params.debugStallMode = Watchdog_DEBUG_STALL_ON;
  watchdogHandle        = Watchdog_open(Board_WATCHDOG0, &params);
  // 5 * 1 seconds. 
  Watchdog_setReload(watchdogHandle, WATCHDOG_TIMER_TIMEOUT_PERIOD);
#endif

  // Set GAP Parameters: After a connection was established, delay in seconds
  // before sending when GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE,...)
  // uses GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS or
  // GAPROLE_LINK_PARAM_UPDATE_INITIATE_APP_PARAMS
  // For current defaults, this has no effect.
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the Peripheral GAPRole Profile. For more information see the User's
  // Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
  {
    // Device starts advertising upon initialization of GAP
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until re-enabled by the application
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the Peripheral GAPRole Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set GAP Parameters to set the advertising interval
  // For more information, see the GAP section of the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html
  {
    // Use the same interval for general and limited advertising.
    // Note that only general advertising will occur based on the above configuration
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager. For more information see the section in the
  // User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
  {
    // Don't send a pairing request after connecting; the peer device must
    // initiate pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    // Use authenticated pairing: require passcode.
    uint8_t mitm = TRUE;
    // This device only has display capabilities. Therefore, it will display the
    // passcode during pairing. However, since the default passcode is being
    // used, there is no need to display anything.
    uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
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

  // For ANCS, the device must register an a GATT client, whereas the
  // iPhone acts as a GATT server.
  VOID GATT_InitClient();
  GATT_RegisterForInd(selfEntity);

  // Start the Device
  VOID GAPRole_StartDevice(&ancsApp_gapRoleCBs);

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&ancsApp_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  //Set default values for Data Length Extension
  {
    //Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    //This API is documented in hci.h
    //See the LE Data Length Extension section in the BLE-Stack User's Guide for information on using this command:
    //http://software-dl.ti.com/lprf/sdg-latest/html/cc2640/index.html
    //HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

#if !defined (USE_LL_CONN_PARAM_UPDATE)
  // Get the currently set local supported LE features
  // The HCI will generate an HCI event that will get received in the main
  // loop
  HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

  Display_print0(dispHandle, 0, 0, "ANCS Demo");
}

/*********************************************************************
 * @fn      AncsApp_taskFxn
 *
 * @brief   Application task entry point for the ANCS App.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void AncsApp_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  AncsApp_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, ANCSAPP_ALL_EVENTS,
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
          if (pEvt->signature == 0xffff)
          {
            // The GATT server might have returned a blePending as it was trying
            // to process an ATT Response. Now that we finished with this
            // connection event, let's try sending any remaining ATT Responses
            // on the next connection event.
            if (pEvt->event_flag & ANCSAPP_HCI_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              AncsApp_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = AncsApp_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & ANCSAPP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          ancsAppEvt_t *pMsg = (ancsAppEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            AncsApp_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
      // Service discovery event.
      if (events & ANCSAPP_START_DISC_EVT)
      {
        //This event will kick off service discovery.
        //The event is only called once, when the GAPROLE is connected
        // discoveryState = ANCS_DISC_SERVICE;
        AncsApp_discoverService(NULL);
      }
#ifdef USE_WATCHDOG_TIMER      
      // Periodic Event for the WDT
      if (events & ANCSAPP_PERIODIC_EVT)
      {
        Util_startClock(&periodicClock);

        // Perform periodic application task
        AncsApp_performPeriodicTask();
      }
#endif
    }
  }
}

/*********************************************************************
 * @fn      AncsApp_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t AncsApp_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = AncsApp_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {

        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            {

#if !defined (USE_LL_CONN_PARAM_UPDATE)
              // This code will disable the use of the LL_CONNECTION_PARAM_REQ
              // control procedure (for connection parameter updates, the
              // L2CAP Connection Parameter Update procedure will be used
              // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
              // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE
              // The L2CAP Connection Parameter Update procedure is used to
              // support a delta between the minimum and maximum connection
              // intervals required by some iOS devices.

              // Parse Command Complete Event for opcode and status
              hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;
              uint8_t   pktStatus = command_complete->pReturnParam[0];

              //find which command this command complete is for
              switch (command_complete->cmdOpcode)
              {
                case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                  {
                    if (pktStatus == SUCCESS)
                    {
                      uint8_t featSet[8];

                      // Get current feature set from received event (bits 1-9
                      // of the returned data
                      memcpy( featSet, &command_complete->pReturnParam[1], 8 );

                      // Clear bit 1 of byte 0 of feature set to disable LL
                      // Connection Parameter Updates
                      CLR_FEATURE_FLAG( featSet[0], LL_FEATURE_CONN_PARAMS_REQ );

                      // Update controller with modified features
                      HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
                    }
                  }
                  break;

                default:
                  //do nothing
                  break;
              }
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

            }
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          default:
            break;
        }
      }
      break;

      default:
        // do nothing
        break;

    }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      AncsApp_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t AncsApp_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // Assume that all initial GATT messages pertain to the discovery
  // process while the app is in the ANCS_STATE_DISCOVERY state.
  if(ancsAppState == ANCS_STATE_DISCOVERY)
  {
    AncsApp_discoverService(pMsg);
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_HANDLE_VALUE_NOTI || pMsg->method == ATT_HANDLE_VALUE_IND)
  {
    // If we receive a GATT notification, we can assume it pertains to ANCS
    // because we only subscribe to notifications from the Notification Source
    // ancs Data Source.

    // This variable is used just to make the code look clearer for the 
    // two conditionals below :)
    uint8_t notifHandle = pMsg->msg.handleValueNoti.handle;

    // Check if the handle matches the handle of the Notification Source
    // stored in the handle cache. If so, store the notification in the queue.
    if ( notifHandle == Ancs_handleCache[ANCS_NOTIF_SCR_HDL_START])
    {
      Ancs_queueNewNotif(pMsg);
    }

    // If it is not from the Notification Source we check to see if it is from 
    // the Data Source by checking if the handle is equal to the Data Source
    // Start handle stored in the cache. If so, process the data it holds,
    // and ask for more.
    else if ( notifHandle == Ancs_handleCache[ANCS_DATA_SRC_HDL_START])
      Ancs_processDataServiceNotif(pMsg);

  }
  //ANCS requires authentication, if the NP attempts to read/write chars on the
  //NP without proper authentication, the NP will respond with insufficent_athen
  //error to which we must respond with a slave security request
  else if  (pMsg->method == ATT_ERROR_RSP &&
            pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ &&
            pMsg->msg.errorRsp.errCode == ATT_ERR_INSUFFICIENT_AUTHEN)
  {
    uint16 conn_handle;
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &conn_handle);
    uint8_t mitm;
    uint8_t bonding;
    GAPBondMgr_GetParameter(GAPBOND_MITM_PROTECTION, &mitm);
    GAPBondMgr_GetParameter(GAPBOND_BONDING_ENABLED, &bonding);
    uint8_t authRequest = ((mitm & 0x01) << 2) | ((bonding & 0x01) << 1) | (bonding & 0x01);
  
    discoveryState = ANCS_DISC_FINISH;
    ancsAppState   = ANCS_STATE_READY;
    GAP_SendSlaveSecurityRequest(conn_handle, authRequest);
  }
  else if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
            ANCSAPP_HCI_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      AncsApp_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  //If we have received a read or write response, assume that it is related to 
  //CCCD configuration
  else if (pMsg->method == ATT_READ_RSP || pMsg->method == ATT_WRITE_RSP)
  {
  }
  //Otherwise subscribe to notification source if ANCS discovery is complete
  else
  {
  }
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      AncsApp_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void AncsApp_sendAttRsp(void)
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
      AncsApp_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, 5, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}

/*********************************************************************
 * @fn      AncsApp_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void AncsApp_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, 5, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, 5, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      AncsApp_discoverService
 *
 * @brief   Function to handle the discovery of the ANCS service
 *
 * @param   pMsg - GATT message to process, may be NULL in DISC_ANCS_START 
 *
 * @return  none
 */
static void AncsApp_discoverService(gattMsgEvent_t *pMsg)
{
  // These will hold the ANCS service start and end handles that are returned after 
  // GATT_DiscPrimaryServiceByUUID() function receives a response from the iPhone.
  static uint16_t Ancs_svcStartHdl;
  static uint16_t Ancs_svcEndHdl;

  // Stores the error code, should the discovery process fail at any state.
  static uint8_t errorcode = 0;

  // Enter the state machine.
  switch (discoveryState)
  {

    // Wait to start the discovery process till after the MTU exchange has occurred.
    case ANCS_EXCHANGE_MTU:
    {
      // If the MTU exchange has occurred, proceed to the discovery process.
      if (pMsg->method == ATT_MTU_UPDATED_EVENT)
      {
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t1");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tExchange MTUs");
#endif 
        discoveryState = ANCS_DISC_SERVICE;
        Event_post(syncEvent, ANCSAPP_START_DISC_EVT);
      }
    }
    break;

    // Perform a GATT Discover Primary Service By Service UUID to located the ANCS
    // handles.
    case ANCS_DISC_SERVICE:
    {
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t2");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tDiscover the ANCS");
#endif 
      // Initialize the ANCS handles to zero.
      Ancs_svcStartHdl = 0;
      Ancs_svcEndHdl   = 0;

      // Store the ANCS UUID for GATT request.
      uint8_t uuid[ATT_UUID_SIZE] = {ANCSAPP_ANCS_SVC_UUID};

      // Discover the ANCS by UUID.
      uint8_t discCheck = GATT_DiscPrimaryServiceByUUID(Ancs_connHandle, uuid, ATT_UUID_SIZE, ICall_getEntityId());

      // If successfully discovered proceed, throw error if not.      
      if(discCheck == SUCCESS)
        discoveryState = ANCS_STORE_SERVICE_HANDLES;
      else
      {
        Display_print1(dispHandle, 12, 0, "ANCS_DISC_SERVICE FAILURE, Error code:\t%d",discCheck);
        discoveryState = ANCS_DISC_FAILED;
        errorcode = 1;
      }
    }
    break;

    // Store the ANCS handles requested in the previous state.
    case ANCS_STORE_SERVICE_HANDLES:
    {
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t3");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tStore the ANCS handles");
#endif 
      // Did the application receive a response from the GATT Disc Primary Service? 
      if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP )
      {
        // Check if the ANCS was found.
        if (pMsg->msg.findByTypeValueRsp.numInfo > 0)
        {
          // Found the ANCS, so store the handles and proceed.
          Ancs_svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
          Ancs_svcEndHdl   = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
          discoveryState   = ANCS_DISC_CHARS;
        }
        else
        {
          // The ANCS was not found.
          Display_print0(dispHandle, 12, 0, "ANCS_STORE_SERVICE_HANDLES FAILURE");
          discoveryState = ANCS_DISC_FAILED;
          errorcode = 2;
        }

      }
    }
    break;

    // Use the ANCS handles to discovery the ANCS's characteristics' handles.
    case ANCS_DISC_CHARS:
    {
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t4");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tDiscover the ANCS characteristics");
#endif 
      // Check if service handle discovery event has completed.
      if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP )
      {
        if(pMsg->hdr.status == bleProcedureComplete)
        {
          // Sanity check to make sure the handle is valid before proceeding. 
          if (Ancs_svcStartHdl != 0)
          {
            // Discover all characteristics of the ANCS.
            uint8_t discCheck = GATT_DiscAllChars(Ancs_connHandle, Ancs_svcStartHdl, Ancs_svcEndHdl, ICall_getEntityId());
            
            // If the request was successfully sent, proceed with the discovery process.
            if (discCheck == SUCCESS)
            {
              discoveryState = ANCS_STORE_CHARS_HANDLES;
            }
            // If not, throw an error.
            else
            {
              Display_print1(dispHandle, 12, 0, "ANCS_DISC_CHARS FAILURE, Error code:\t%d",discCheck);
              discoveryState = ANCS_DISC_FAILED;
              errorcode = 3;
            }
          }
        }
      }
    }
    break;

    // Store the retrieved ANCS characteristic handles.
    case ANCS_STORE_CHARS_HANDLES:
    {
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t5");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tStore the ANCS characteristics' handles");
#endif 
      // Wait until GATT "Read by type response" is received, then confirm that the correct number of
      // pairs are present, and that their length is correct
      if (pMsg->method == ATT_READ_BY_TYPE_RSP )
      {
        if ( (pMsg->msg.readByTypeRsp.numPairs == NUMBER_OF_ANCS_CHARS) && (pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID128_LEN) )
        {
          // Pointer to the pair list data in the GATT response.
          uint8_t   *pCharPairList;
          // Will store the start and end handles of the current pair.
          uint16_t  charStartHandle;
          uint16_t  charEndHandle;
          // Stores to the UUID of the current pair.
          uint16_t  charUuid;
          // Stores what pair the loop is currently processing.
          uint8_t   currentCharIndex;
          
          // Set the pair pointer to the first pair.
          pCharPairList = pMsg->msg.readByTypeRsp.pDataList;

          // Iterate through all three pairs found.
          for(currentCharIndex = NUMBER_OF_ANCS_CHARS; currentCharIndex > 0 ; currentCharIndex--)
          {
            // Extract the starting handle, ending handle, and UUID of the current characteristic.
            charStartHandle = BUILD_UINT16(pCharPairList[3], pCharPairList[4]);
            // To extract the end handle for each characteristic, take the starting handle of the next characteristic
            // and subtract one from it.
            charEndHandle   = BUILD_UINT16(pCharPairList[CHAR_DESC_HDL_UUID128_LEN], pCharPairList[CHAR_DESC_HDL_UUID128_LEN + 1]) - 1;
            charUuid        = BUILD_UINT16(pCharPairList[5], pCharPairList[6]);

            // Store the start and end handles in the handle cache corresponding to
            // their UUID.
            switch (charUuid)
            {
              // If it's the Notification Source.
              case ANCSAPP_NOTIF_SRC_CHAR_UUID:
                Ancs_handleCache[ANCS_NOTIF_SCR_HDL_START] = charStartHandle;
                Ancs_handleCache[ANCS_NOTIF_SCR_HDL_END]   = charEndHandle;
                break;
              // If it's the Control Point.  
              case ANCSAPP_CTRL_PT_CHAR_UUID:
                Ancs_handleCache[ANCS_CTRL_POINT_HDL_START] = charStartHandle;
                Ancs_handleCache[ANCS_CTRL_POINT_HDL_END]   = charEndHandle;
                break;
              // If it's the Data Source.  
              case ANCSAPP_DATA_SRC_CHAR_UUID:
                Ancs_handleCache[ANCS_DATA_SRC_HDL_START] = charStartHandle;
                Ancs_handleCache[ANCS_DATA_SRC_HDL_END]   = charEndHandle;
                break;
                
              default:
                break;
            }

            // If this is the final characteristic found in the response,
            // reset its end handle to the ANCS's end handle. This is because
            // there is no next staring handle to use as a reference and subtract one
            // from, so instead the ending handle of the ANCS must be used.
            if(currentCharIndex == LAST_ANCS_CHAR)
              Ancs_handleCache[ANCS_DATA_SRC_HDL_END]  = Ancs_svcEndHdl;

            // Increment the pair pointer to the next pair.
            pCharPairList += CHAR_DESC_HDL_UUID128_LEN;
          }

          // Sanity check to ensure that each start handle is valid and
          // less than each respective end handle.
          if(Ancs_handleCache[ANCS_NOTIF_SCR_HDL_START]  != 0 &&
             Ancs_handleCache[ANCS_CTRL_POINT_HDL_START] != 0 &&
             Ancs_handleCache[ANCS_DATA_SRC_HDL_START]   != 0) 
          {
            if(Ancs_handleCache[ANCS_NOTIF_SCR_HDL_START]  < Ancs_handleCache[ANCS_NOTIF_SCR_HDL_END]  &&
               Ancs_handleCache[ANCS_CTRL_POINT_HDL_START] < Ancs_handleCache[ANCS_CTRL_POINT_HDL_END] &&
               Ancs_handleCache[ANCS_DATA_SRC_HDL_START]   < Ancs_handleCache[ANCS_DATA_SRC_HDL_END])
            { 
              discoveryState = ANCS_DISC_NS_DESCS;
            }
            else
            {
              Display_print0(dispHandle, 12, 0, "ANCS_STORE_CHARS_HANDLES FAILURE");
              discoveryState = ANCS_DISC_FAILED;
              errorcode = 4;
            } 
          }
          // Throw an error if the handles are invalid.
          else
          {
            Display_print0(dispHandle, 12, 0, "ANCS_STORE_CHARS_HANDLES FAILURE");
            discoveryState = ANCS_DISC_FAILED;
            errorcode = 5;
          }        
        }
        // Throw an error if the length or number of pairs is incorrect.
        else
        {
          Display_print0(dispHandle, 12, 0, "ANCS_STORE_CHARS_HANDLES FAILURE");
          discoveryState = ANCS_DISC_FAILED;
          errorcode = 6;
        }
      }
    }
    break;

    // Discover the Notification Source's descriptors (namely, the CCCD) using the start 
    // and end handle stored in the handle cache.
    case ANCS_DISC_NS_DESCS:
    {
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t6");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tDiscover the Notification Source's CCCD");
#endif 
      // Wait until the characteristic handle discovery has finished.
      if ( (pMsg->method == ATT_READ_BY_TYPE_RSP) && (pMsg->hdr.status == bleProcedureComplete) )
      {
        // Discover the ANCS Notification Source descriptors.
        uint8_t discCheck = GATT_DiscAllCharDescs(Ancs_connHandle,
                              Ancs_handleCache[ANCS_NOTIF_SCR_HDL_START],
                              Ancs_handleCache[ANCS_NOTIF_SCR_HDL_END],
                              ICall_getEntityId());
        // If the discovery was successful, proceed.
        if ( discCheck == SUCCESS )
          discoveryState = ANCS_STORE_NS_DESCS_HANDLES;
        // If not, throw an error and invalidate the CCCD handle in the handle cache.
        else
        {
          Display_print0(dispHandle, 6, 0, "ANCS_DISC_NS_DESCS FAILURE");
          Ancs_handleCache[ANCS_NOTIF_SCR_HDL_START] = 0;
          discoveryState = ANCS_DISC_FAILED;
          errorcode = 7;
        }
      }
    }
    break;

    // Store the retrieved Notification Source descriptors (namely, the CCCD).
    case ANCS_STORE_NS_DESCS_HANDLES:
    {
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t7");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tStore the Notification Source's CCCD handle");
#endif 
      // Wait for the discovery response.
      if (pMsg->method == ATT_FIND_INFO_RSP )
      {
        // Sanity check to validate that at least one descriptors pair was found,
        // and that the pair length is correct.
        if ( (pMsg->msg.findInfoRsp.numInfo > 0) && (pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE) )
        {
          // This will keep track of the current pair being processed.
          uint8_t currentPair;

          // Iterate through the pair list.
          for(currentPair = 0; currentPair < pMsg->msg.findInfoRsp.numInfo; currentPair++)
          {
            // Check if the pair is a CCCD.
            if (ATT_BT_PAIR_UUID(pMsg->msg.findInfoRsp.pInfo, currentPair) == GATT_CLIENT_CHAR_CFG_UUID)
            {
              // If so, store the handle in the handle cache, and proceed.
              Ancs_handleCache[ANCS_NOTIF_SCR_HDL_CCCD] = ATT_BT_PAIR_HANDLE(pMsg->msg.findInfoRsp.pInfo, currentPair);
              discoveryState = ANCS_DISC_DS_DESCS;
            }
          }
        }
      }
    }
    break;

    // Discover the Data Source's descriptors (namely, the CCCD) using the start 
    // and end handle stored in the handle cache.
    case ANCS_DISC_DS_DESCS:
    {
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t8");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tDiscover the Data Source's CCCD");
#endif 
      // Wait until the Notification Source descriptors discovery has finished.
      if ( (pMsg->method == ATT_FIND_INFO_RSP) && (pMsg->hdr.status == bleProcedureComplete) )
      {

        // Discover ANCS Notification Source CCCD 
        uint8_t discCheck = GATT_DiscAllCharDescs(Ancs_connHandle,
                              Ancs_handleCache[ANCS_DATA_SRC_HDL_START] + 1,
                              Ancs_handleCache[ANCS_DATA_SRC_HDL_END],
                              ICall_getEntityId());
        // If the discovery was successful, proceed.
        if (discCheck == SUCCESS ) 
          discoveryState = ANCS_STORE_DS_DESCS_HANDLES;
        // If not, throw an error and invalidate the CCCD handle in the handle cache.
        else
        {
          Display_print0(dispHandle, 6, 0, "ANCS_DISC_DS_DESCS FAILURE");
          Ancs_handleCache[ANCS_DATA_SRC_HDL_CCCD] = 0;
          discoveryState = ANCS_DISC_FAILED;
          errorcode = 8;
        }
      }
    }
    break;

    // Discover the Data Source's descriptors (namely, the CCCD) using the start 
    // and end handle stored in the handle cache.
    case ANCS_STORE_DS_DESCS_HANDLES:
    {
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t9");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tStore the Data Source's CCCD handle");
#endif 
      // Wait for the discovery response.
      if (pMsg->method == ATT_FIND_INFO_RSP )
      {
        // Sanity check to validate that at least one descriptors pair was found,
        // and that the pair length is correct.
        if ( (pMsg->msg.findInfoRsp.numInfo > 0) && (pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE) )
        {
          // This will keep track of the current pair being processed.
          uint8_t currentPair;

          // Iterate through the pair list.
          for(currentPair = 0; currentPair < pMsg->msg.findInfoRsp.numInfo; currentPair++)
          {
            // Check if the pair is a CCCD.
            if (ATT_BT_PAIR_UUID(pMsg->msg.findInfoRsp.pInfo, currentPair) == GATT_CLIENT_CHAR_CFG_UUID)
            {
              // If so, store the handle in the handle cache, and proceed to the subscription process.
              Ancs_handleCache[ANCS_DATA_SRC_HDL_CCCD] = ATT_BT_PAIR_HANDLE(pMsg->msg.findInfoRsp.pInfo, currentPair);
              discoveryState = ANCS_WRITE_DS_CCCD;

              // The next state may need to run multiple times, thus it relies on 
              // event posts as opposed to a singular event like a BLE complete response.
              Event_post(syncEvent, ANCSAPP_START_DISC_EVT);
            }
          }
        }
      }
    }
    break;

    // Subscribe to the Data Source. This is done first as when the Notification Source 
    // is subscribed too, it will immediately send GATT notifications for every ANCS notification  
    // present. This can hinder app functionality as some ANCS notifications maybe ignored 
    // due to the app receiving them before it is initialized to a state which can process them.
    // To avoid this, we subscribe to the Notification Source second.
    case ANCS_WRITE_DS_CCCD:
    { 
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t10");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tSubscribe to the Data Source");
#endif 
      // Call the function which writes to the Data Source's CCCD.
      uint8_t check = Ancs_subsDataSrc();

      // If it is successful, advance the state. If the check is not 
      // successful, a loop will be sustained until the subscription succeeds.
      if(check == SUCCESS)
        discoveryState = ANCS_WRITE_NS_CCCD;
      Event_post(syncEvent, ANCSAPP_START_DISC_EVT);
    }
    break;

    // Subscribe to the Notification source second for the reason listed in the last state.
    case ANCS_WRITE_NS_CCCD:
    {
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t11");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tSubscribe to the Notification Source");
#endif 
      /// Call the function which writes to the Notification Source's CCCD.
      uint8_t check = Ancs_subsNotifSrc();

      // If it is successful, set the discovery state to FINISH and the app state to READY.
      // If the check is not successful, a loop will be sustained until the subscription succeeds.
      if(check == SUCCESS)
      {
        discoveryState = ANCS_DISC_FINISH;
        ancsAppState   = ANCS_STATE_READY;
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t12");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tProcessing notification data");
#endif 
      }
      else
        Event_post(syncEvent, ANCSAPP_START_DISC_EVT);
    }
    break;
  }
  if(errorcode != 0)
  {
#ifdef USE_GUI_COMPOSER
    Display_print1(dispHandle, 16, 0, "Discovery State:\tDiscovery Error: %d",errorcode);
#else
    Display_print1(dispHandle, 16, 0, "Discovery Error: %d",errorcode);
#endif
  }
  return;
}

/*********************************************************************
 * @fn      AncsApp_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void AncsApp_processAppMsg(ancsAppEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    // State change event triggered.
    case ANCSAPP_STATE_CHANGE_EVT:
      {
        AncsApp_processStateChangeEvt((gaprole_States_t)pMsg->hdr.state);
      }
      break;

    // Pairing event triggered.
    case ANCSAPP_PAIRING_STATE_EVT:
      {
        AncsApp_processPairState(pMsg->hdr.state, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    // Passcode event triggered.
    case ANCSAPP_PASSCODE_NEEDED_EVT:
      {
        AncsApp_processPasscode(*pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    // CC2640R2 board button event triggered.
    case ANCSAPP_KEY_CHANGE_EVT:
      {
        AncsApp_handleKeysEvt(pMsg->hdr.state);
    
        break;
      }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      AncsApp_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void AncsApp_stateChangeCB(gaprole_States_t newState)
{
  AncsApp_enqueueMsg(ANCSAPP_STATE_CHANGE_EVT, newState, 0);
}

/*********************************************************************
 * @fn      AncsApp_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void AncsApp_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, 2, 0, "Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
#ifdef USE_WATCHDOG_TIMER
      Util_startClock(&periodicClock);
#endif
      Display_print0(dispHandle, 2, 0, "Advertising");
      break;

#ifdef PLUS_BROADCASTER
    // After a connection is dropped, a device in PLUS_BROADCASTER will continue
    // sending non-connectable advertisements and shall send this change of
    // state to the application.  These are then disabled here so that sending
    // connectable advertisements can resume.
    case GAPROLE_ADVERTISING_NONCONN:
      {
#ifdef USE_WATCHDOG_TIMER
        Util_startClock(&periodicClock);
#endif
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        AncsApp_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        uint8_t peerAddress[B_ADDR_LEN];
        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
        Display_print0(dispHandle, 2, 0, "Connected");
        Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));


#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t0");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tConnected");
#endif 

        // Initialize the app and discovery state to their respective beginning.
        notifAttrPktProcessState = NOTI_ATTR_FIRST_PKT;
        appAttrPktProcessState   = NOTI_ATTR_FIRST_PKT;
        discoveryState           = ANCS_EXCHANGE_MTU;
        ancsAppState             = ANCS_STATE_DISCOVERY;

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // Set to true for non-connectable advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Display_print0(dispHandle, 2, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:
      // Free the ATT response, clear the handle cache, and clear the ANCS notification queue.
#ifdef USE_WATCHDOG_TIMER
      Util_stopClock(&periodicClock);
#endif
      AncsApp_freeAttRsp(bleNotConnected);
      VOID memset(Ancs_handleCache, '\0', HDL_CACHE_LEN*2);
      Display_print0(dispHandle, 3, 0, "Disconnected");
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t0");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tDisconnected");
#endif 
      Ancs_popAllNotifsFromQueue();

      // Clear remaining lines
      Display_clearLines(dispHandle, 4, 30);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      // Free the ATT response, clear the handle cache, and clear the ANCS notification queue.
#ifdef USE_WATCHDOG_TIMER
      Util_stopClock(&periodicClock);
#endif
      AncsApp_freeAttRsp(bleNotConnected);
      VOID memset(Ancs_handleCache, '\0', HDL_CACHE_LEN*2);
      Display_print0(dispHandle, 3, 0, "Timed Out");
#ifdef USE_GUI_COMPOSER
    Display_print0(dispHandle, 16, 0, "Discovery Progress:\t0");
    Display_print0(dispHandle, 16, 0, "Discovery State:\tTimed out");
#endif 
      Ancs_popAllNotifsFromQueue();

      // Clear remaining lines
      Display_clearLines(dispHandle, 4, 30);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif // PLUS_BROADCASTER
      break;

    case GAPROLE_ERROR:
      Display_print0(dispHandle, 2, 0, "Error");
      break;

    default:
      Display_clearLine(dispHandle, 2);
      break;
  }

}

/*********************************************************************
 * @fn      AncsApp_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void AncsApp_pairStateCB(uint16_t connHandle, uint8_t state,
                                            uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    AncsApp_enqueueMsg(ANCSAPP_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      AncsApp_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void AncsApp_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_print0(dispHandle, 2, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Pairing Successful");

      // Now that the device has successfully paired to the iPhone,
      // the subscription will not fail due to insufficient authentication.
      discoveryState = ANCS_WRITE_DS_CCCD;
      Event_post(syncEvent, ANCSAPP_START_DISC_EVT);      
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bonding Successful");    
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bond save success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      AncsApp_handleKeysEvt
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events.
 *
 * @return  none
 */
static void AncsApp_handleKeysEvt(uint8_t keys)
{
  // If: Both keys are pressed.
  if (keys == IO_BUTTON_BOTH)
    Display_clearLines(dispHandle, 4, 30);

  // Else If: A single key is pressed.
  else
  { 
    // Calls the function to perform a negative action upon the current notification.
    if (keys == IO_BUTTON_LEFT)
        Ancs_acceptIncomingCall();

    // Calls the function to perform a positive action upon the current notification.
    else if (keys == IO_BUTTON_RIGHT)
      Ancs_declineIncomingCall();
  }

  return;
}

/*********************************************************************
 * @fn      AncsApp_keyPressCB
 *
 * @brief   Callback to the handle keys function (AncsApp_handleKeysEvt()).
 *
 * @param   keys - bit field for key events.
 *
 * @return  none
 */
static void AncsApp_keyPressCB(uint8 keys)
{
  // Enqueue the event.
  AncsApp_enqueueMsg(ANCSAPP_KEY_CHANGE_EVT, keys, 0);

  return;
}

/*********************************************************************
 * @fn      AncsApp_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void AncsApp_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                           uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    AncsApp_enqueueMsg(ANCSAPP_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      AncsApp_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void AncsApp_processPasscode(uint8_t uiOutputs)
{
  // This app uses a default passcode. A real-life scenario would handle all
  // pairing scenarios and likely generate this randomly.
  uint32_t passcode = B_APP_DEFAULT_PASSCODE;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  uint16_t connectionHandle;
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

#ifdef USE_WATCHDOG_TIMER
/*
 *  ======== watchdogCallback ========
 *  Watchdog interrupt callback function.
 */
static void watchdogCallback(uintptr_t unused)
{
    /* Clear watchdog interrupt flag */
    Watchdog_clear(watchdogHandle);
    Display_print0(dispHandle, 16, 0, "Watchdog kicked!");

    /* Insert timeout handling code here. */
}
/*********************************************************************
 * @fn      AncsApp_performPeriodicTask
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
static void AncsApp_performPeriodicTask(void)
{
  Watchdog_clear(watchdogHandle);
  Display_print0(dispHandle, 16, 0, "Watchdog cleared!");
}

/*********************************************************************
 * @fn      AncsApp_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void AncsApp_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}
#endif
/*********************************************************************
 * @fn      AncsApp_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t AncsApp_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  ancsAppEvt_t *pMsg = ICall_malloc(sizeof(ancsAppEvt_t));

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
*********************************************************************/
