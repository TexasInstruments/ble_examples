/******************************************************************************

 @file  blood_pressure.c

 @brief This file contains the Blood pressure sample application for use with
        the Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2011-2020, Texas Instruments Incorporated
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
#include <ti/drivers/GPIO.h>

#include <icall.h>
#include "util.h"

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "devinfoservice.h"
#include "bletime.h"
#include "bpservice.h"
#include "ll_common.h"

#include "peripheral.h"
#include "board_key.h"

#include "board.h"

#include "utc_clock.h"
#include "blood_pressure.h"

/*********************************************************************
 * CONSTANTS
 */
// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_WAIT_BOTH_PARAMS

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Some values used to simulate measurements
#define FLAGS_IDX_MAX                         7

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default passkey
#define DEFAULT_PASSKEY                       0

#define CUFF_MAX                              40      

// Max measurement storage count
#define BP_STORE_MAX                          4

// Delay to begin service discovery from start of connection in ms
#define DEFAULT_DISCOVERY_DELAY               1000

#define BP_DISCONNECT_PERIOD                  6000

#define TIMER_CUFF_PERIOD                     500

//Events for Blood Pressure measurements
#define BP_CUFF_MEASUREMENT_EVENT             1
#define BP_FINAL_MEASUREMENT_EVENT            2

#define BP_CUFF_MEAS_LEN                              19
#define BP_MEAS_LEN                                   19

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
    #define BP_DISPLAY_TYPE Display_Type_LCD
  #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
    #define BP_DISPLAY_TYPE Display_Type_UART
  #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
    #define BP_DISPLAY_TYPE 0 // Option not supported
  #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#else // Display_DISABLE_ALL
  #define BP_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

// Task configuration
#define BLOODPRESSURE_TASK_PRIORITY                   1
#define BLOODPRESSURE_TASK_STACK_SIZE                 644
// Blood Pressure Task Events
#define BP_SERVICE_CB_EVT                             0x0001
#define BP_KEY_CHANGE_EVT                             0x0002
#define BP_PAIRING_STATE_CB_EVT                       0x0004
#define BP_PASSCODE_CB_EVT                            0x0008
#define BP_PERIPHERAL_CB_EVT                          0x0010

// Internal Events for RTOS application
#define BP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define BP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define BP_START_DISC_EVT                    Event_Id_00
#define BP_TIMER_MEAS_EVT                    Event_Id_01
#define BP_DISCONNECT_EVT                    Event_Id_02

// Bitwise OR of all events to pend on
#define BP_ALL_EVENTS                        (BP_ICALL_EVT        | \
                                              BP_QUEUE_EVT        | \
                                              BP_START_DISC_EVT   | \
                                              BP_TIMER_MEAS_EVT   | \
                                              BP_DISCONNECT_EVT)
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // Event header.
  uint8_t *pData;  // Event data.
} bloodPressureEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct startDiscoveryClock;
static Clock_Struct bloodPressureMeasClock;
static Clock_Struct disconnectClock;

// Handle for the Message Queue.
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// GAP State
static gaprole_States_t gapProfileState = GAPROLE_INIT;

// Connection handle.
static uint16_t connHandle = 0;

// Task configuration.
Task_Struct BloodPressure_task;
Char BloodPressure_taskStack[BLOODPRESSURE_TASK_STACK_SIZE];

// Service discovery state.
static uint8_t servDiscState = DISC_IDLE;

// Service discovery complete.
static uint8_t servDiscComplete = FALSE;

// Characteristic configuration state.
static uint8_t Time_configState = TIME_CONFIG_START;

// TRUE if discovery postponed due to pairing.
static uint8_t servDiscPostponed = FALSE;

// GAP Profile - Name attribute for SCAN RSP data.
static uint8_t scanResponseData[] =
{
  0x12,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'B',
  'l',
  'o',
  'o',
  'd',
  'P',
  'r',
  'e',
  's',
  's',
  'u',
  'r',
  'e',
  ' ',
  'S',
  'e',
  'n',
  's',
  'o',
  'r',
    // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL, //TX Power Level
  0       // 0dBm
};

// Advertisement data.
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(BLOODPRESSURE_SERV_UUID),
  HI_UINT16(BLOODPRESSURE_SERV_UUID)
  /* Removed - No Target Address in AD or Scan Response - Single Bond
  0x07,
  0x17,
  0x15,0x53,0x00,0x57,0x60,0x00
  */
};

// Device name attribute value.
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "BloodPressure Sensor";

// Bonded peer address.
static uint8_t timeAppBondedAddr[B_ADDR_LEN];

// Last connection address.
static uint8_t lastConnAddr[B_ADDR_LEN] = {0xf,0xf,0xf,0xf,0xf,0xe};;

static bool connectedToLastAddress = false;

// GAP connected device address.
static uint8_t connDeviceAddr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static uint16_t bpSystolic = 120; //mmg
static uint16_t bpDiastolic = 80; //mmg
static uint16_t bpMAP = 90; //70-110mmg
static uint16_t bpPulseRate = 60; //pulseRate
static uint8_t  bpUserId = 1;
static uint16_t bpMeasStatus = 0;

// flags for simulated measurements.
static const uint8_t bloodPressureFlags[FLAGS_IDX_MAX] =
{
  BLOODPRESSURE_FLAGS_MMHG | BLOODPRESSURE_FLAGS_TIMESTAMP |
    BLOODPRESSURE_FLAGS_PULSE | BLOODPRESSURE_FLAGS_USER |
    BLOODPRESSURE_FLAGS_STATUS,
  BLOODPRESSURE_FLAGS_MMHG | BLOODPRESSURE_FLAGS_TIMESTAMP,
  BLOODPRESSURE_FLAGS_MMHG,
  BLOODPRESSURE_FLAGS_KPA,
  BLOODPRESSURE_FLAGS_KPA | BLOODPRESSURE_FLAGS_TIMESTAMP,
  BLOODPRESSURE_FLAGS_KPA | BLOODPRESSURE_FLAGS_TIMESTAMP |
    BLOODPRESSURE_FLAGS_PULSE,
  0x00
};

// Initial value of flags.
static uint8_t bloodPressureFlagsIdx = 0;

// Cutoff measurement count. When we reach max, send final BP meas.
static uint8_t cuffCount = 0;
static uint8_t measEvents = 0;

static attHandleValueInd_t bpStoreMeas[BP_STORE_MAX];
static uint8_t bpStoreStartIndex = 0;
static uint8_t bpStoreIndex = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

// Application tasks and event processing.
static void BloodPressure_init(void);
static void BloodPressure_taskFxn(UArg a0, UArg a1);
static void BloodPressure_processStackMsg(ICall_Hdr *pMsg);
static void BloodPressure_processGattMsg(gattMsgEvent_t *pMsg);
static void BloodPressure_processAppMsg(bloodPressureEvt_t *pMsg);
static void BloodPressure_clockHandler(UArg arg);
static uint8_t BloodPressure_enqueueMsg(uint16_t event, uint8_t state, uint8_t *pData);
static void BloodPressure_toggleAdvertising(void);

// Service discovery.
static void BloodPressure_startDiscEvt(void);

// Blood pressure service measurements.
static void BloodPressure_measureEvt(void);
static void BloodPressure_cuffMeas(void);
static void BloodPressure_finalMeas(void);
static void BloodPressure_processCuffMeas(void);
static void BloodPressure_storeIndications(attHandleValueInd_t* pInd);
static void BloodPressure_sendStoredMeas(void);
static void BloodPressure_simulateMeas(void);

// Peripheral role events.
static void BloodPressure_stateChangeCB(gaprole_States_t newState);
static void BloodPressure_stateChangeEvt(gaprole_States_t newState);
static void BloodPressure_disconnected(void);
static void BloodPressure_disconnectEvt(void);

// Blood pressure service.
static void BloodPressure_serviceCB(uint8_t event);
static void BloodPressure_CCCUpdateEvt(void);

// GAP Bond Manager.
static void BloodPressure_passcodeCB(uint8_t *deviceAddr,
                                     uint16_t connectionHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs,
                                     uint32_t numericComparison);
static void BloodPressure_passcodeEvt(uint8_t uiOutputs);
static void BloodPressure_pairStateCB(uint16_t connHandle, uint8_t state,
                                      uint8_t status);
static void BloodPressure_pairStateEvt(uint8_t state, uint8_t status);

// Keys.
void BloodPressure_keyChangeHandler(uint8_t keys);
static void BloodPressure_handleKeys(uint8_t shift, uint8_t keys);

// User interface.
static void BloodPressure_updateUI(void);
static void BloodPressure_printBPFlags(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t bloodPressure_PeripheralCBs =
{
  BloodPressure_stateChangeCB  // Profile State Change Callbacks
};

// Bond Manager Callbacks
static gapBondCBs_t timeAppBondCB =
{
  BloodPressure_passcodeCB,  // Passcode callback
  BloodPressure_pairStateCB  // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      BloodPressure_createTask
 *
 * @brief   Task creation function for the Blood Pressure application.
 *
 * @param   none
 *
 * @return  none
 */
void BloodPressure_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = BloodPressure_taskStack;
  taskParams.stackSize = BLOODPRESSURE_TASK_STACK_SIZE;
  taskParams.priority = BLOODPRESSURE_TASK_PRIORITY;

  Task_construct(&BloodPressure_task, BloodPressure_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      BloodPressure_Init
 *
 * @brief   Initialization function for the BloodPressure App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   None.
 *
 * @return  None.
 */
void BloodPressure_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Setup clocks.
  Util_constructClock(&startDiscoveryClock, BloodPressure_clockHandler,
                      DEFAULT_DISCOVERY_DELAY, 0, false, BP_START_DISC_EVT);
  Util_constructClock(&bloodPressureMeasClock, BloodPressure_clockHandler,
                      TIMER_CUFF_PERIOD, 0, false, BP_TIMER_MEAS_EVT);
  Util_constructClock(&disconnectClock, BloodPressure_clockHandler,
                      BP_DISCONNECT_PERIOD, 0, false, BP_DISCONNECT_EVT);

  Board_initKeys(BloodPressure_keyChangeHandler);
  /* Call driver init functions */
  dispHandle = Display_open(BP_DISPLAY_TYPE, NULL);
  GPIO_init();

  // Configure the LED pins
  GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
  GPIO_setConfig(Board_GPIO_LED1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

  // Setup the GAP Peripheral Role Profile.
  {
#if AUTO_ADV
    uint8_t initial_advertising_enable = TRUE;
#else
    // Press right key to initiate advertising and measurement.
    uint8_t initial_advertising_enable = FALSE;
#endif //AUTO_ADV

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE.
    uint16_t gapRole_AdvertOffTime = 0;

    uint8_t enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role parameters.
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                          &initial_advertising_enable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                          &gapRole_AdvertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof (scanResponseData),
                          scanResponseData);

    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData),
                          advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                          &enable_update_request);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                          &desired_min_interval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                          &desired_max_interval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desired_slave_latency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                          &desired_conn_timeout);
  }

  // Set the GAP characteristics.
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Setup the GAP Bond Manager.
  {
    uint32_t passkey = DEFAULT_PASSKEY; // passkey "000000"
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    uint8_t mitm = DEFAULT_MITM_MODE;
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    uint8_t bonding = DEFAULT_BONDING_MODE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof (uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof (uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof (uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof (uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof (uint8_t),
                            &bonding);
  }

  // Stop config reads when done.
  Time_configDone = FALSE;

  // Initialize GATT Client.
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications.
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes.
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Add blood pressure service.
  BloodPressure_AddService(GATT_ALL_SERVICES);

  // Add device info service.
  DevInfo_AddService();

  // Register for BloodPressure service callback.
  BloodPressure_Register(BloodPressure_serviceCB);

  // Initialize measurement storage table
  memset(bpStoreMeas, 0, (sizeof(attHandleValueInd_t) * BP_STORE_MAX));

  // Start the Device.
  VOID GAPRole_StartDevice(&bloodPressure_PeripheralCBs);

  // Register with bond manager after starting device.
  GAPBondMgr_Register((gapBondCBs_t *) &timeAppBondCB);

  // Initialize the UTC clock.
  UTC_init();

  // Update UI.
  BloodPressure_updateUI();
}

/*********************************************************************
 * @fn      BloodPressure_taskFxn
 *
 * @brief   BloodPressure Application Task entry point.  This function
 *          is called to initialize and then process all events for the task.
 *          Events include timers, messages and any other user defined events.
 *
 * @param   a0, a1 - not used.
 *
 * @return  none.
 */
void BloodPressure_taskFxn(UArg a0, UArg a1)
{
  // Initialize the application.
  BloodPressure_init();

  // Application main loop.
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, BP_ALL_EVENTS,
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
          // Process inter-task message.
          BloodPressure_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    // If RTOS queue is not empty, process app message.
    while (!Queue_empty(appMsgQueue))
    {
      bloodPressureEvt_t *pMsg =
        (bloodPressureEvt_t*)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message.
        BloodPressure_processAppMsg(pMsg);

        // Free the space from the message.
        ICall_free(pMsg);
      }
    }

    if (events)
    {
      // Service discovery.
      if (events & BP_START_DISC_EVT)
      {
        events &= ~BP_START_DISC_EVT;

        BloodPressure_startDiscEvt();
      }

      // Blood pressure measurement.
      if (events & BP_TIMER_MEAS_EVT)
      {
        events &= ~BP_TIMER_MEAS_EVT;

        BloodPressure_measureEvt();
      }

      // Disconnect timer expired
      if (events & BP_DISCONNECT_EVT)
      {
        events &= ~BP_DISCONNECT_EVT;

        BloodPressure_disconnectEvt();
      }
    }
  }
}

/*********************************************************************
 * @fn      BloodPressure_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void BloodPressure_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      BloodPressure_processGattMsg((gattMsgEvent_t *) pMsg);
      break;

    default:
      // Do nothing.
      break;
  }
}


/*********************************************************************
 * @fn      BloodPressure_processGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void BloodPressure_processGattMsg(gattMsgEvent_t *pMsg)
{
  // Measurement Indication Confirmation
  if(pMsg->method == ATT_HANDLE_VALUE_CFM)
  {
      BloodPressure_sendStoredMeas();
  }

  if (pMsg->method == ATT_HANDLE_VALUE_NOTI ||
       pMsg->method == ATT_HANDLE_VALUE_IND)
  {
    Time_indGattMsg(pMsg);
  }
  else if (pMsg->method == ATT_READ_RSP ||
            pMsg->method == ATT_WRITE_RSP)
  {
    Time_configState = Time_configGattMsg (Time_configState, pMsg);
    if (Time_configState == TIME_CONFIG_CMPL)
    {
      servDiscComplete = TRUE;
    }
  }
  else
  {
    servDiscState = Time_discGattMsg(servDiscState, pMsg);
    if (servDiscState == DISC_IDLE)
    {
      // Start characteristic configuration
      Time_configState = Time_configNext(TIME_CONFIG_START);
    }
  }

  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      BloodPressure_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void BloodPressure_keyChangeHandler(uint8_t keys)
{
  // Enqueue key event.
  BloodPressure_enqueueMsg(BP_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      BloodPressure_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 *
 * @return  none
 */
static void BloodPressure_handleKeys(uint8_t shift, uint8_t keys)
{
  // Up key.
  if (keys & KEY_LEFT)
  {
    // Set simulated measurement flag index.
    if (++bloodPressureFlagsIdx == FLAGS_IDX_MAX)
    {
      bloodPressureFlagsIdx = 0;
    }
    BloodPressure_printBPFlags();
  }
  // Right key.
  if (keys & KEY_RIGHT)
  {
    // If the device is not in a connection, toggle advertising on and off and
    // start a blood pressure measurement.
    if(gapProfileState != GAPROLE_CONNECTED)
    {
      // Toggle Advertising.
      BloodPressure_toggleAdvertising();

      // Reset cuff count.
      cuffCount = 0;

      // This is a cuff measurement.
      measEvents = BP_CUFF_MEASUREMENT_EVENT;

      // Start simulation timer (start --> cuff -->measurement ready).
      Util_startClock(&bloodPressureMeasClock);
    }
    // Connected mode, simulate measurements.
    else
    {
      BloodPressure_simulateMeas();
    }
  }
}

/*********************************************************************
 * @fn      BloodPressure_processAppMsg
 *
 * @brief   Process an incoming app message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void BloodPressure_processAppMsg(bloodPressureEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case BP_PERIPHERAL_CB_EVT:
      BloodPressure_stateChangeEvt((gaprole_States_t)pMsg->
	                                hdr.state);
      break;

    case BP_KEY_CHANGE_EVT:
      BloodPressure_handleKeys(0, pMsg->hdr.state);
      break;
	  
	// Pairing state CB event.
    case BP_PAIRING_STATE_CB_EVT:
      BloodPressure_pairStateEvt(pMsg->hdr.state, *pMsg->pData);

      ICall_free(pMsg->pData);
      break;

    // Passcode CB event.
    case BP_PASSCODE_CB_EVT:
      BloodPressure_passcodeEvt(*pMsg->pData);
	  ICall_free(pMsg->pData);
      break;

    // Blood pressure service CB event.
    case BP_SERVICE_CB_EVT:
      {
        // If notifications have been enabled
        if (pMsg->hdr.state == BLOODPRESSURE_MEAS_NOTI_ENABLED)
        {
          BloodPressure_CCCUpdateEvt();
        }
      }
      break;


    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      BloodPressure_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void BloodPressure_stateChangeCB(gaprole_States_t newState)
{
  // Enqueue the message.
  BloodPressure_enqueueMsg(BP_PERIPHERAL_CB_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      BloodPressure_toggleAdvertising
 *
 * @brief   Toggle advertising status.
 *
 * @param   none
 *
 * @return  none
 */
static void BloodPressure_toggleAdvertising(void)
{
  uint8_t advStatus;

  // Find the current GAP advertisement status.
  GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &advStatus);

  // Get the opposite state.
  advStatus = !advStatus;

  // Change the GAP advertisement status to opposite of current status.
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                       &advStatus);
}

/*********************************************************************
 * @fn      BloodPressure_simulateMeas
 *
 * @brief   Modify simulated measurement values.
 *
 * @param   none
 *
 * @return  none
 */
static void BloodPressure_simulateMeas(void)
{
  if (gapProfileState == GAPROLE_CONNECTED)
  {
      Display_print0(dispHandle, 6, 0, "Updating simulated measurements.");
    if (bpSystolic < 150)
      bpSystolic +=1;
    else
      bpSystolic = 80;

    if (bpDiastolic < 110)
      bpDiastolic +=1;
    else
      bpDiastolic = 90;

    if (bpMAP < 110)
      bpMAP +=1;
    else
      bpMAP =70;

    if (bpPulseRate < 140)
      bpPulseRate +=1;
    else
      bpPulseRate =40;

    if (bpUserId < 5)
      bpUserId +=1;
    else
      bpUserId =1;
  }
}

/*********************************************************************
 * @fn      BloodPressure_startDiscEvt
 *
 * @brief   Event to handle the start of service discovery.
 *
 * @return  none
 */
void BloodPressure_startDiscEvt(void)
{
  servDiscState = Time_discStart();
}

/*********************************************************************
 * @fn      BloodPressure_measureEvt
 *
 * @brief   process blood pressure measurement event.
 *
 * @param   arg - unused argument.
 *
 * @return  none
 */
static void BloodPressure_measureEvt(void)
{
  if (measEvents == BP_CUFF_MEASUREMENT_EVENT)
  {
    BloodPressure_cuffMeas();
  }
  else if (measEvents == BP_FINAL_MEASUREMENT_EVENT)
  {
    BloodPressure_finalMeas();
  }
}

/*********************************************************************
 * @fn      BloodPressure_cuffMeas
 *
 * @brief   process cutoff measurements
 *
 * @return  none
 */
static void BloodPressure_cuffMeas(void)
{
  // Perform a Cutoff Measurement
  BloodPressure_processCuffMeas();

  cuffCount++;

  // If cuff count not met, keep sending cuff measurements
  if(cuffCount < CUFF_MAX)
  {
    // Set cuff measurement event.
    measEvents = BP_CUFF_MEASUREMENT_EVENT;

    // Start interval timer to send BP, just for simulation.
    Util_startClock(&bloodPressureMeasClock);
  }
  // Get ready to send final measurement.
  else
  {
    // Set final measurement event.
    measEvents = BP_FINAL_MEASUREMENT_EVENT;

    // Start timer to send final BP meas.
    Util_startClock(&bloodPressureMeasClock);
  }
}

/*********************************************************************
 * @fn      BloodPressure_processCuffMeas
 *
 * @brief   Prepare and send a bloodPressure measurement notification
 *
 * @return  none
 */
static void BloodPressure_processCuffMeas(void)
{
  attHandleValueNoti_t bloodPressureNotiMeas;
  /* Turn on user LED */
  GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

  bloodPressureNotiMeas.pValue = GATT_bm_alloc(connHandle,
                                               ATT_HANDLE_VALUE_NOTI,
                                               BP_CUFF_MEAS_LEN, NULL);
  if (bloodPressureNotiMeas.pValue != NULL)
  {
    // ATT value notification structure.
    uint8_t *p = bloodPressureNotiMeas.pValue;

    // Flags.
    uint8_t flags = BLOODPRESSURE_FLAGS_MMHG  | BLOODPRESSURE_FLAGS_TIMESTAMP |
                    BLOODPRESSURE_FLAGS_PULSE | BLOODPRESSURE_FLAGS_USER      |
                    BLOODPRESSURE_FLAGS_STATUS;

    // Flags 1 byte long.
    *p++ = flags;

    // BloodPressure components
    // IEEE The 16-bit value contains a 4-bit exponent to base 10,
    // followed by a 12-bit mantissa. Each is in 2's complement form.
    *p++ = LO_UINT16(bpSystolic); //120 = 0x0078  SFloat little endian = 0x7800
    *p++ = HI_UINT16(bpSystolic);
    *p++ = 0xFF;   //not used in cuff  IEEE NaN =0x07FF
    *p++ = 0x07;
    *p++ = 0xFF;   //not used in cuff IEEE NaN =0x07FF
    *p++ = 0x07;

    // Timestamp.
    if (flags & BLOODPRESSURE_FLAGS_TIMESTAMP)
    {
      UTCTimeStruct time;

      // Get time structure from the UTC_clock module.
      UTC_convertUTCTime(&time, UTC_getClock());

      *p++ = 0x07;
      *p++ = 0xdb;
      *p++ = time.month;
      *p++ = time.day;
      *p++ = time.hour;
      *p++ = time.minutes;
      *p++ = time.seconds;
    }

    if(flags & BLOODPRESSURE_FLAGS_PULSE)
    {
      *p++ = LO_UINT16(bpPulseRate);
      *p++ = HI_UINT16(bpPulseRate);
    }

    if(flags & BLOODPRESSURE_FLAGS_USER)
    {
      *p++ = bpUserId;
    }

    if(flags & BLOODPRESSURE_FLAGS_STATUS)
    {
      *p++ = LO_UINT16(bpMeasStatus);
      *p++ = HI_UINT16(bpMeasStatus);
    }

    bloodPressureNotiMeas.len = (uint8_t) (p - bloodPressureNotiMeas.pValue);

    // Cuff measurements are not stored, only sent.
    if (BloodPressure_IMeasNotify(connHandle, &bloodPressureNotiMeas,
                                  selfEntity) != SUCCESS)
    {
      GATT_bm_free((gattMsg_t *)&bloodPressureNotiMeas, ATT_HANDLE_VALUE_NOTI);
    }
  }
}

/*********************************************************************
 * @fn      BloodPressure_finalMeas
 *
 * @brief   Prepare and send a bloodPressure measurement indication
 *
 * @return  none
 */
static void BloodPressure_finalMeas(void)
{
  // BloodPressure measurement value stored in this structure.
  attHandleValueInd_t  bloodPressureIndMeas;

  bloodPressureIndMeas.pValue = GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_IND,
                                              BP_MEAS_LEN, NULL);
  if (bloodPressureIndMeas.pValue != NULL)
  {
    // ATT value notification structure.
    uint8_t *p = bloodPressureIndMeas.pValue;

    // Get the flags.
    uint8_t flags = bloodPressureFlags[bloodPressureFlagsIdx];

    // flags 1 byte long.
    *p++ = flags;

    // Blood pressure components.
    *p++ = LO_UINT16(bpSystolic);
    *p++ = HI_UINT16(bpSystolic);
    *p++ = LO_UINT16(bpDiastolic);
    *p++ = HI_UINT16(bpDiastolic);
    *p++ = LO_UINT16(bpMAP);
    *p++ = HI_UINT16(bpMAP);

    // Timestamp.
    if (flags & BLOODPRESSURE_FLAGS_TIMESTAMP)
    {
      UTCTimeStruct time;

      // Get time structure from the UTC_clock module.
      UTC_convertUTCTime(&time, UTC_getClock());

      *p++ = 0x07;
      *p++ = 0xdb;
      *p++ = time.month;
      *p++ = time.day;
      *p++ = time.hour;
      *p++ = time.minutes;
      *p++ = time.seconds;
    }

    if(flags & BLOODPRESSURE_FLAGS_PULSE)
    {
      *p++ = LO_UINT16(bpPulseRate);
      *p++ = HI_UINT16(bpPulseRate);
    }

    if(flags & BLOODPRESSURE_FLAGS_USER)
    {
      *p++ = bpUserId;
    }

    if(flags & BLOODPRESSURE_FLAGS_STATUS)
    {
      *p++ = LO_UINT16(bpMeasStatus);
      *p++ = HI_UINT16(bpMeasStatus);
    }

    bloodPressureIndMeas.len = (uint8_t) (p - bloodPressureIndMeas.pValue);

    // Store measurement.
    BloodPressure_storeIndications(&bloodPressureIndMeas);

    // Send stored measurements.
    BloodPressure_sendStoredMeas();

    // Start disconnect timer.
    Util_startClock(&disconnectClock);
  }
}

/*********************************************************************
 * @fn      BloodPressure_storeIndications
 *
 * @brief   Queue indications
 *
 * @return  none
 */
static void BloodPressure_storeIndications(attHandleValueInd_t *pInd)
{
  attHandleValueInd_t *pStoreInd = &bpStoreMeas[bpStoreIndex];

  if (pStoreInd->pValue != NULL)
  {
    // Free old indication's payload.
    GATT_bm_free((gattMsg_t *)pStoreInd, ATT_HANDLE_VALUE_IND);
  }

  // Store measurement.
  memcpy(pStoreInd, pInd, sizeof(attHandleValueInd_t));

  // Store index.
  bpStoreIndex = bpStoreIndex + 1;
  if (bpStoreIndex > BP_STORE_MAX)
  {
    bpStoreIndex = 0;
  }

  if (bpStoreIndex == bpStoreStartIndex)
  {
    bpStoreStartIndex = bpStoreStartIndex + 1;
    if(bpStoreStartIndex > BP_STORE_MAX)
    {
      bpStoreStartIndex = 0;
    }
  }
}

/*********************************************************************
 * @fn      BloodPressure_sendStoredMeas
 *
 * @brief   Send a stored measurement indication. An incoming indication
 *          confirmation will trigger the next pending stored measurement.
 *
 * @return  none
 */
static void BloodPressure_sendStoredMeas(void)
{
  // We were previously connected to this peer before so send any stored
  // measurements.
  if (bpStoreStartIndex != bpStoreIndex)
  {
    attHandleValueInd_t *pStoreInd = &bpStoreMeas[bpStoreStartIndex];

    // Send measurements - can fail if not connected or CCC not enabled.
    bStatus_t status = BloodPressure_MeasIndicate(connHandle, pStoreInd,
                                                  selfEntity);
    // If successful, increment the counters and the indication confirmation
    // will trigger the next indication if there are more pending.
    if (status == SUCCESS)
    {
      bpStoreStartIndex = bpStoreStartIndex + 1;

      // Wrap around buffer.
      if (bpStoreStartIndex > BP_STORE_MAX)
      {
        bpStoreStartIndex = 0;
      }

      // Clear out this Meas indication.
      memset(pStoreInd, 0, sizeof(attHandleValueInd_t));
    }
  }
}

/*********************************************************************
 * @fn      bloodPressure_DisconnectEvt
 *
 * @brief   Event to handle disconnecting after sending measurement.
 *
 * @return  none
 */
static void BloodPressure_disconnectEvt(void)
{
    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_ON);

  // Terminate Connection.
  GAPRole_TerminateConnection();
}

/*********************************************************************
 * @fn      BloodPressure_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void BloodPressure_pairStateCB(uint16_t connHandle, uint8_t state,
                                            uint8_t status)
{
  uint8_t *pData;

  // Note: connHandle is not used, so the variable is not stored.

  // Allocate message data
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    BloodPressure_enqueueMsg(BP_PAIRING_STATE_CB_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      BloodPressure_pairStateEvt
 *
 * @brief   Pairing state event processor.
 *
 * @param   connHandle - the device with which this pairing event applies.
 *
 * @param   state - the new device state.
 *
 * @param   status - if the action succeeded or failed.
 *
 * @return  none
 */
static void BloodPressure_pairStateEvt(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_print0(dispHandle, 1, 0, "Pairing started");
    // Stop the clock.  Service discovery will continue after pairing.
    Util_stopClock(&startDiscoveryClock);

    // Set flag.
    servDiscPostponed = TRUE;

  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
	  Display_print0(dispHandle, 1, 0, "Pairing success");
      memcpy(timeAppBondedAddr, connDeviceAddr, B_ADDR_LEN);

      // If discovery was postponed start discovery.
      if (servDiscPostponed && (servDiscComplete == FALSE))
      {
        servDiscPostponed = FALSE;
		
		// Wake up the application.
  		Event_post(syncEvent, BP_START_DISC_EVT);
      }
    }
    else
    {
      Display_print1(dispHandle, 1, 0, "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 1, 0, "Bonding success");
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 1, 0, "Bond save success");
    }
    else
    {
      Display_print1(dispHandle, 1, 0, "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      BloodPressure_stateChangeEvt
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void BloodPressure_stateChangeEvt(gaprole_States_t newState)
{
  // If connected
  if (newState == GAPROLE_CONNECTED)
  {
    //Turn off LED
    GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_OFF);

    // Get connected device's address.
    GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, &connDeviceAddr);

    // Get connection handle.
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);

    // Set the Time module's copy of the connHandle
    Time_connHandle = connHandle;

    // If connected to device without bond do service discovery.
    if (memcmp(connDeviceAddr, timeAppBondedAddr, B_ADDR_LEN))
    {
      servDiscComplete = FALSE;
    }
    else
    {
      servDiscComplete = TRUE;
    }

    // If this is the most recently connected device don't do discovery.
    if(!memcmp(connDeviceAddr, lastConnAddr, B_ADDR_LEN))
    {
      servDiscComplete = TRUE;
      connectedToLastAddress = true;
    }
    else
    {
      // Save the last connected address.
      memcpy(lastConnAddr, connDeviceAddr, B_ADDR_LEN);
    }

    // Initiate service discovery if necessary.
    if (servDiscComplete == FALSE  &&
         DEFAULT_PAIRING_MODE != GAPBOND_PAIRING_MODE_INITIATE)
    {
      Util_startClock(&startDiscoveryClock);
    }
  }
  // If disconnected
  else if (gapProfileState == GAPROLE_CONNECTED &&
            newState != GAPROLE_CONNECTED)
  {
    // Stop disconnect timer (in case device disconnected for another reason).
    Util_stopClock(&disconnectClock);

    BloodPressure_disconnected();

    // Stop measurement timer.
    Util_stopClock(&bloodPressureMeasClock);

#if AUTO_ADV
    // Begin advertising.
    BloodPressure_toggleAdvertising();
#endif //AUTO_ADV
  }

  // If not in a connection but not advertising
  else if (newState == GAPROLE_WAITING_AFTER_TIMEOUT ||
           newState == GAPROLE_WAITING)
  {
      /* Turn off user LED */
      GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);
#if AUTO_ADV
    // Begin advertising.
    BloodPressure_toggleAdvertising();
#endif //AUTO_ADV
  }

  // If started
  else if (newState == GAPROLE_STARTED)
  {
    // Do nothing.
  }

  // Store new GAP Role state.
  gapProfileState = newState;

  // Update user interface.
  BloodPressure_updateUI();
}

/*********************************************************************
 * @fn      BloodPressure_disconnected
 *
 * @brief   Handle disconnect.
 *
 * @return  none
 */
static void BloodPressure_disconnected(void)
{
  uint8_t advEnable = FALSE;

  // Stop discovery clock in case it is running.
  Util_stopClock(&startDiscoveryClock);

  // Re-initialize state variables.
  servDiscState = DISC_IDLE;
  servDiscPostponed = FALSE;

  // Reset conn handle and Time module's copy
  Time_connHandle = connHandle = INVALID_CONNHANDLE;

  // Reset bloodPressure measurement client configuration.
  uint16_t param = 0;

  BloodPressure_SetParameter(BLOODPRESSURE_MEAS_CHAR_CFG, sizeof(uint16_t),
                             (uint8_t *) &param);

  // Reset bloodPressure intermediate measurement client configuration.
  BloodPressure_SetParameter(BLOODPRESSURE_IMEAS_CHAR_CFG, sizeof(uint16_t),
                             (uint8_t *) &param);

  // Disable advertising upon disconnect.
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advEnable);
}

/*********************************************************************
 * @fn      BloodPressure_serviceCB
 *
 * @brief   Callback function for bloodpressure service.
 *
 * @param   event - service event
 *
 *   Need to know if BP Meas enabled so we can send stored meas.

      #define BLOODPRESSURE_MEAS_NOTI_ENABLED         1
      #define BLOODPRESSURE_MEAS_NOTI_DISABLED        2

 * @return  none
 */
static void BloodPressure_serviceCB(uint8_t event)
{
  // Queue the event.
  BloodPressure_enqueueMsg(BP_SERVICE_CB_EVT, event, NULL);
}

/*********************************************************************
 * @fn      BloodPressure_CCCUpdateEvt
 *
 * @brief   Event to handle updates of the CCC for the blood pressure service.
 *
 * @return  none
 */
static void BloodPressure_CCCUpdateEvt(void)
{
  // If actively connected
  if (gapProfileState == GAPROLE_CONNECTED)
  {
    // If previously connected and measurements are active send stored.
    if(connectedToLastAddress == true)
    {
      // Send stored measurements.
      BloodPressure_sendStoredMeas();
    }
  }
}

/*********************************************************************
 * @fn      BloodPressure_passcodeEvt
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void BloodPressure_passcodeEvt(uint8_t uiOutputs)
{
  // This app uses a default passcode. A real-life scenario would handle all
  // pairing scenarios and likely generate this randomly.
  uint32_t passcode = DEFAULT_PASSKEY;
  
  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_print1(dispHandle, 7, 0, "Passcode: %d", passcode);
  }

  uint16_t connectionHandle;
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      BloodPressure_clockHandler
 *
 * @brief   Clock handle for all clock instances.  Argument is set
 *          during clock module initialization - this is how the
 *          handler knows which clock has expired.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void BloodPressure_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      BloodPressure_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void BloodPressure_passcodeCB(uint8_t *deviceAddr,
                                     uint16_t connectionHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs,
                                     uint32_t numericComparison)
{
  // Note:  the only parameter used here is connectionHandle, but this has
  // already been obtained by gapConnHandle so the argument is not stored.

  // Queue the event.
  BloodPressure_enqueueMsg(BP_PASSCODE_CB_EVT, 0, NULL);
}

/*********************************************************************
 * @fn      Update UI
 *
 * @brief   Update user interface LCD and LED.
 *
 * @param   none
 *
 * @return  none
 */
static void BloodPressure_updateUI(void)
{
  Display_print0(dispHandle, 0, 0, "Blood Pressure");

  static uint8_t ownAddress[B_ADDR_LEN];

  // Number of stored measurements.
  uint16_t count =0;

  // Store index.
  if(bpStoreIndex > bpStoreStartIndex)
  {
    count = bpStoreIndex - bpStoreStartIndex;
  }

  if(bpStoreStartIndex > bpStoreIndex)
  {
     count = (BP_STORE_MAX-bpStoreStartIndex) + bpStoreIndex+1;
  }

  // Update according to new state.
   switch (gapProfileState)
  {
    case GAPROLE_INIT:
          Display_print0(dispHandle, 1, 0, "Initialized");
          break;
    case GAPROLE_STARTED:
          Display_print0(dispHandle, 1, 0, "Started");
          GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
          Display_print0(dispHandle, 2, 0, Util_convertBdAddr2Str(ownAddress));
#ifndef AUTO_ADV
          Display_print0(dispHandle, 3, 0, "-> Begin advertising");
#endif
          BloodPressure_printBPFlags();
          break;
    case GAPROLE_ADVERTISING:
         Display_print0(dispHandle, 1, 0, "Advertising");
         Display_clearLine(dispHandle, 2);
         Display_clearLine(dispHandle, 3);
         Display_print1(dispHandle, 6, 0, "Stored %d",  count);
         break;
    case GAPROLE_WAITING:
    case GAPROLE_WAITING_AFTER_TIMEOUT:
         Display_clearLine(dispHandle, 6);
         Display_print0(dispHandle, 1, 0, "Waiting    ");
         Display_print1(dispHandle, 6, 0, "Stored %d",  count);
         Display_clearLine(dispHandle, 5);
#ifndef AUTO_ADV
         Display_print0(dispHandle, 3, 0, "-> Begin advertising");
#endif
        break;
    case GAPROLE_CONNECTED:
        Display_print0(dispHandle, 1, 0, "Connected  ");
        Display_print0(dispHandle, 5, 0, "-> Simulate BP");
        break;
    case GAPROLE_ERROR:
        Display_print0(dispHandle, 1, 0, "Error      ");
        break;
    default:
        break;
  }
}

/*********************************************************************
 * @fn      BloodPressure_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - new state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t BloodPressure_enqueueMsg(uint16_t event, uint8_t state,
                                        uint8_t *pData)
{
  bloodPressureEvt_t *pMsg;

  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(bloodPressureEvt_t)))
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
 * @fn      BloodPressure_printBPFlags
 *
 * @brief   Print what blood pressure flags are active
 *
 */
static void BloodPressure_printBPFlags(void)
{
    switch (bloodPressureFlagsIdx)
    {
    case 0:
        Display_print0(dispHandle, 4, 0, "<- Blood pressure flags: MMHG | TIMESTAMP | PULSE | USER | STATUS");
        break;
    case 1:
        Display_print0(dispHandle, 4, 0, "<- Blood pressure flags: MMHG | TIMESTAMP");
        break;
    case 2:
        Display_print0(dispHandle, 4, 0, "<- Blood pressure flags: MMHG");
        break;
    case 3:
        Display_print0(dispHandle, 4, 0, "<- Blood pressure flags: KPA");
        break;
    case 4:
        Display_print0(dispHandle, 4, 0, "<- Blood pressure flags: KPA | TIMESTAMP");
        break;
    case 5:
        Display_print0(dispHandle, 4, 0, "<- Blood pressure flags: KPA | TIMESTAMP | PULSE");
        break;
    case 6:
        Display_print0(dispHandle, 4, 0, "<- Blood pressure flags: NONE");
        break;
    }
}

/*********************************************************************
*********************************************************************/
