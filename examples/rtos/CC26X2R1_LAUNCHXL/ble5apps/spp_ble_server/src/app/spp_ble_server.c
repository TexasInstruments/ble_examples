/******************************************************************************

 @file  spp_ble_server.c

 @brief This file contains the SPP BLE Server sample application for use
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

#include <ti/display/Display.h>

#if !(defined __TI_COMPILER_VERSION__)
#include <intrinsics.h>
#endif

#include <ti/drivers/utils/List.h>

#include "icall.h"
#include "util.h"
#include "bcomdef.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "devinfoservice.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include "board.h"
#include "board_key.h"

#include "spp_ble_server.h"
#include "serial_port_service.h"
#include "inc/sdi_task.h"

/*********************************************************************
 * MACROS
 */

// Connection indices map to events by shifting 1 left by the index
#define CONN_INDEX_TO_EVENT(x)             (1 << x)

// Count the trailing zeros to map back from event to index
#if defined __TI_COMPILER_VERSION__
#define CONN_EVENT_TO_INDEX(x) (sizeof(unsigned int) * 8 - 1 - __clz((unsigned int) x))
#else
#define CONN_EVENT_TO_INDEX(x) (sizeof(unsigned int) * 8 - 1 - __CLZ((unsigned int) x))
#endif

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
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 104=130ms) for  parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     104

// Slave latency to use for parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 300=3s) for parameter update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          300

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_PASS_TO_APP

// How often to perform periodic event (in ms)
#define SBS_PERIODIC_EVT_PERIOD               5000

// How often to read current current RPA (in ms)
#define SBS_READ_RPA_EVT_PERIOD               3000

// Delay (in ms) after connection establishment before sending a parameter update requst
#define SBS_SEND_PARAM_UPDATE_DELAY           6000

// Task configuration
#define SBS_TASK_PRIORITY                     1

#ifndef SBS_TASK_STACK_SIZE
#define SBS_TASK_STACK_SIZE                   1024
#endif

// Application events
#define SBS_STATE_CHANGE_EVT                  0
#define SBS_CHAR_CHANGE_EVT                   1
#define SBS_KEY_CHANGE_EVT                    2
#define SBS_ADV_EVT                           3
#define SBS_PAIR_STATE_EVT                    4
#define SBS_PASSCODE_EVT                      5
#define SBS_PERIODIC_EVT                      6
#define SBS_READ_RPA_EVT                      7
#define SBS_SEND_PARAM_UPDATE_EVT             8
#define SBS_UART_QUEUE_EVT                    9

// Internal Events for RTOS application
#define SBS_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBS_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define SBS_ALL_EVENTS                        (SBS_ICALL_EVT        | \
                                               SBS_QUEUE_EVT        | \
                                               SBS_UART_QUEUE_EVT   | \
                                               SBS_PERIODIC_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define SBS_ADDR_STR_SIZE     15

// Row numbers for two-button menu
#define SBS_ROW_SEPARATOR_1   (TBM_ROW_APP + 0)
#define SBS_ROW_STATUS_1      (TBM_ROW_APP + 1)
#define SBS_ROW_STATUS_2      (TBM_ROW_APP + 2)
#define SBS_ROW_CONNECTION    (TBM_ROW_APP + 3)
#define SBS_ROW_ADVSTATE      (TBM_ROW_APP + 4)
#define SBS_ROW_RSSI          (TBM_ROW_APP + 5)
#define SBS_ROW_IDA           (TBM_ROW_APP + 6)
#define SBS_ROW_RPA           (TBM_ROW_APP + 7)
#define SBS_ROW_DEBUG         (TBM_ROW_APP + 8)

// For storing the active connections
#define SBS_RSSI_TRACK_CHNLS        1            // Max possible channels can be GAP_BONDINGS_MAX
#define SBS_MAX_RSSI_STORE_DEPTH    5
#define SBS_INVALID_HANDLE          0xFFFF
#define RSSI_2M_THRSHLD           -30           // -80 dB rssi
#define RSSI_1M_THRSHLD           -40           // -90 dB rssi
#define RSSI_S2_THRSHLD           -50           // -100 dB rssi
#define RSSI_S8_THRSHLD           -60           // -120 dB rssi
#define SBS_PHY_NONE                LL_PHY_NONE  // No PHY set
#define AUTO_PHY_UPDATE            0xFF

// Spin if the expression is not true
#define SIMPLEPERIPHERAL_ASSERT(expr) if (!(expr)) spp_ble_server_spin();

/*********************************************************************
 * TYPEDEFS
 */

// RTOS queue for profile/app messages.
typedef struct _queueRec_
{
  Queue_Elem _elem;          // queue element
  uint8_t *pData;            // pointer to app data
} queueRec_t;

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} spEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} spPairStateData_t;

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
} spPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} spGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;                //
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
  int8_t           rssiArr[SBS_MAX_RSSI_STORE_DEPTH];
  uint8_t          rssiCntr;
  int8_t           rssiAvg;
  bool             phyCngRq;                          // Set to true if PHY change request is in progress
  uint8_t          currPhy;
  uint8_t          rqPhy;
  uint8_t          phyRqFailCnt;                      // PHY change request count
  bool             isAutoPHYEnable;                   // Flag to indicate auto phy change
} spConnRec_t;

// App event passed from profiles.
typedef struct
{
  uint8_t event;  // Type of event
  uint8_t *pData;  // New data
  uint8_t length; // New status
} sbpUARTEvt_t;
/*********************************************************************
 * GLOBAL VARIABLES
 */

// Global pin resources
PIN_State pinGpioState;
PIN_Handle hGpioPin;

uint16 currentMTUSize;

// Display Interface
Display_Handle dispHandle = NULL;

// Task configuration
Task_Struct spTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(spTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t spTaskStack[SBS_TASK_STACK_SIZE];

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueueHandle;

// Queue object used for UART messages
static Queue_Struct appUARTMsg;
static Queue_Handle appUARTMsgQueue;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkPeriodic;
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass periodic event ID to clock handler
spClockEventData_t argPeriodic =
{ .event = SBS_PERIODIC_EVT };

// Memory to pass RPA read event ID to clock handler
spClockEventData_t argRpaRead =
{ .event = SBS_READ_RPA_EVT };

// Per-handle connection info
static spConnRec_t connList[MAX_NUM_BLE_CONNS];

// Current connection handle as chosen by menu
static uint16_t menuConnHandle = CONNHANDLE_INVALID;

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "SPP BLE Server";

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
#if !defined(FEATURE_OAD)
  0x03,   // length of this data
#else //OAD for external flash
  0x05,  // length of this data
#endif //FEATURE_OAD
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
#endif //FEATURE_OAD
  LO_UINT16(SERIALPORTSERVICE_SERV_UUID),
  HI_UINT16(SERIALPORTSERVICE_SERV_UUID)
};


// Scan Response Data
static uint8_t scanRspData[] =
{
  // complete name
  15,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',
  'P',
  'P',
  ' ',
  'B',
  'L',
  'E',
  ' ',
  'S',
  'E',
  'R',
  'V',
  'E',
  'R',

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

// Pins that are actively used by the application
static PIN_Config SPPBLEAppPinTable[] =
{
    Board_PIN_RLED       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */
    Board_PIN_GLED       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */

    PIN_TERMINATE
};

static uint16 currentConnHandle = CONNHANDLE_INVALID;

const char *strPHY[] = {"1 Mbps", "2 Mbps", "1 & 2 Mbps", "Coded", "1 & 2 Mbps, & Coded", "Auto PHY change"};

// Value to write
#define CHAR_LEN 42
static uint8_t charVal[CHAR_LEN] = {0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SPPBLEServer_init( void );
static void SPPBLEServer_taskFxn(UArg a0, UArg a1);

static uint8_t SPPBLEServer_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SPPBLEServer_processGATTMsg(gattMsgEvent_t *pMsg);
static void SPPBLEServer_processGapMessage(gapEventHdr_t *pMsg);
static void SPPBLEServer_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void SPPBLEServer_processAdvEvent(spGapAdvEventData_t *pEventData);
static void SPPBLEServer_processAppMsg(spEvt_t *pMsg);
static void SPPBLEServer_processCharValueChangeEvt(uint8_t paramId);
static void SPPBLEServer_performPeriodicTask(void);
static void SPPBLEServer_charValueChangeCB(uint8_t paramID);

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
static void SPPBLEServer_updateRPA(void);
#endif // PRIVACY_1_2_CFG
static void SPPBLEServer_clockHandler(UArg arg);
static void SPPBLEServer_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);
static void SPPBLEServer_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void SPPBLEServer_processPairState(spPairStateData_t *pPairState);
static void SPPBLEServer_processPasscode(spPasscodeData_t *pPasscodeData);
static void SPPBLEServer_charValueChangeCB(uint8_t paramId);
static void SPPBLEServer_enqueueMsg(uint8_t event, void *pData);
void SPPBLEServer_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len);

char* convInt32ToText(int32 value);
static void SPPBLEServer_keyChangeHandler(uint8 keys);
static void SPPBLEServer_handleKeys(uint8_t keys);
static void SPPBLEServer_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void SPPBLEServer_initPHYRSSIArray(void);
static void SPPBLEServer_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
static uint8_t SPPBLEServer_addConn(uint16_t connHandle);
static uint8_t SPPBLEServer_getConnIndex(uint16_t connHandle);
static uint8_t SPPBLEServer_removeConn(uint16_t connHandle);
static void SPPBLEServer_processParamUpdate(uint16_t connHandle);
static status_t SPPBLEServer_startAutoPhyChange(uint16_t connHandle);
static status_t SPPBLEServer_stopAutoPhyChange(uint16_t connHandle);
static status_t SPPBLEServer_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts);
static uint8_t SPPBLEServer_clearConnListEntry(uint16_t connHandle);
bool SPPBLEServer_doSetConnPhy(uint8 index);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t SPPBLEServer_BondMgrCBs =
{
  SPPBLEServer_passcodeCb,       // Passcode callback
  SPPBLEServer_pairStateCb       // Pairing/Bonding state Callback
};

// Serial Port Profile Callbacks

static SerialPortServiceCBs_t SPPBLEServer_SerialPortService_CBs =
{
  SPPBLEServer_charValueChangeCB // Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      spp_ble_server_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void spp_ble_server_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}


/*******************************************************************************
 * @fn      SPPBLEServer_blinkLed
 *
 * @brief   Blinks a led 'n' times, duty-cycle 50-50
 * @param   led - led identifier
 * @param   nBlinks - number of blinks
 *
 * @return  none
 */
void SPPBLEServer_blinkLed(uint8_t led, uint8_t nBlinks)
{
  uint8_t i;

  for (i=0; i<nBlinks; i++)
  {
    PIN_setOutputValue(hGpioPin, led, Board_LED_ON);
    delay_ms(BLINK_DURATION);
    PIN_setOutputValue(hGpioPin, led, Board_LED_OFF);
    delay_ms(BLINK_DURATION);
  }
}

/*******************************************************************************
 * @fn      SPPBLEServer_toggleLed
 *
 * @brief   Toggle an LED
 * @param   led - led identifier
 * @param   state - state to change the LED
 *
 * @return  none
 */
void SPPBLEServer_toggleLed(uint8_t led, uint8_t state)
{
    uint8_t nextLEDstate = 0;

    if(state == Board_LED_TOGGLE)
    {
      nextLEDstate = !(PIN_getOutputValue(led));
    }
    else
    {
      nextLEDstate = state;
    }

    PIN_setOutputValue(hGpioPin, led, nextLEDstate);
}

/*********************************************************************
 * @fn      SPPBLEServer_createTask
 *
 * @brief   Task creation function for the SPP BLE Server.
 */
void SPPBLEServer_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = spTaskStack;
  taskParams.stackSize = SBS_TASK_STACK_SIZE;
  taskParams.priority = SBS_TASK_PRIORITY;

  Task_construct(&spTask, SPPBLEServer_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SPPBLEServer_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void SPPBLEServer_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Hard code the BD Address till CC2650 board gets its own IEEE address
  uint8 bdAddress[B_ADDR_LEN] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05 };
  HCI_EXT_SetBDADDRCmd(bdAddress);

  // Handling of LED
  hGpioPin = PIN_open(&pinGpioState, SPPBLEAppPinTable);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsg);
  appUARTMsgQueue = Util_constructQueue(&appUARTMsg);

  // Create one-shot clock for internal periodic events.
  Util_constructClock(&clkPeriodic, SPPBLEServer_clockHandler,
                      SBS_PERIODIC_EVT_PERIOD, 0, false, (UArg)&argPeriodic);

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

  SerialPortService_AddService(GATT_ALL_SERVICES);  //SerialPortBLE service

  // Register callback with SerialPortService
  SerialPortService_RegisterAppCBs(&SPPBLEServer_SerialPortService_CBs);

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&SPPBLEServer_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

#ifdef SDI_USE_UART
    //Register to receive UART messages
  SDITask_registerIncomingRXEventAppCB(SPPBLEServer_enqueueUARTMsg);
#endif
  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
    #define APP_SUGGESTED_PDU_SIZE 27 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 328 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  GATT_InitClient();

  // Init key debouncer
  Board_initKeys(SPPBLEServer_keyChangeHandler);

  // Initialize Connection List
  SPPBLEServer_clearConnListEntry(CONNHANDLE_ALL);

  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, NULL);

  // Initialize array to store connection handle and RSSI values
  SPPBLEServer_initPHYRSSIArray();

  //Display project name and Bluetooth 5 support
  uint8_t initMsg[] = "Hello from SPP BLE Server! With Bluetooth 5 support!\n\r";
  DEBUG(initMsg);

  SPPBLEServer_blinkLed(Board_PIN_RLED, 1);
}

/*********************************************************************
 * @fn      SPPBLEServer_taskFxn
 *
 * @brief   Application task entry point for the SPP BLE Server.
 *
 * @param   a0, a1 - not used.
 */
static void SPPBLEServer_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SPPBLEServer_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, SBS_ALL_EVENTS,
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
            uint8_t i;

            // Loop through to check events for all connections
            for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
            {
              if (pEvt->event_flag & BV(i))
              {
                // Get connection index from event
                uint8_t connIndex = CONN_EVENT_TO_INDEX(pEvt->event_flag);

                // Get connection handle from index
                uint16_t connHandle = connList[connIndex].connHandle;

                // If auto phy change is enabled
                if (connList[connIndex].isAutoPHYEnable == TRUE)
                {
                  // Read the RSSI
                  HCI_ReadRssiCmd(connHandle);
                }
              }
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SPPBLEServer_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SBS_UART_QUEUE_EVT)
      {
        // If RTOS queue is not empty, process app message.
        if (!Queue_empty(appUARTMsgQueue))
        {
          // Get the message at the front of the queue but still keep it in the queue
          queueRec_t *pRec = Queue_head(appUARTMsgQueue);
          sbpUARTEvt_t *pMsg = (sbpUARTEvt_t *)pRec->pData;

          if (pMsg && (linkDB_Up(currentConnHandle)))
          {
            bStatus_t retVal = FAILURE;

            switch(pMsg->event)
            {
            case SBS_UART_DATA_EVT:
              {
                // Send UART data received from UART terminal using notifications over the air
                retVal = SerialPortService_SetParameter(SERIALPORTSERVICE_CHAR_DATA, pMsg->length, pMsg->pData);

                if(retVal != SUCCESS)
                {
                  DEBUG("Noti FAIL"); DEBUG((uint8_t*)convInt32ToText((int)retVal)); DEBUG_NEWLINE();
                }
                else
                {
                  // Increment TX status counter
                  SerialPortService_AddStatusTXBytes(pMsg->length);

                  // Remove from queue
                  Util_dequeueMsg(appUARTMsgQueue);

                  // Deallocate data payload being transmitted.
                  ICall_freeMsg(pMsg->pData);
                  // Free the space from the message.
                  ICall_free(pMsg);
                }

                if(!Queue_empty(appUARTMsgQueue))
                {
                  // Wake up the application to flush out any remaining UART data in the queue.
                  Event_post(syncEvent, SBS_UART_QUEUE_EVT);
                }
                break;
              }
            default:
              break;
            }
          }
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SBS_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          spEvt_t *pMsg = (spEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            SPPBLEServer_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }

    }
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SPPBLEServer_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SPPBLEServer_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SPPBLEServer_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        {
          SPPBLEServer_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
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
                Display_printf(dispHandle, SBS_ROW_STATUS_1, 0,
                        "PHY Change failure, peer does not support this");
              }
              else
              {
                Display_printf(dispHandle, SBS_ROW_STATUS_1, 0,
                               "PHY Update Status Event: 0x%x",
                               pMyMsg->cmdStatus);
              }

              SPPBLEServer_updatePHYStat(HCI_LE_SET_PHY, (uint8_t *)pMsg);
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
            if (pPUC->status != SUCCESS)
            {
              DEBUG("PHY Change failure");
            }
            else
            {
              // Only symmetrical PHY is supported.
              // rxPhy should be equal to txPhy.
              DEBUG("PHY Changed to: ");
              DEBUG((uint8_t*)((pPUC->rxPhy == HCI_PHY_1_MBPS) ? "1M" :
                                  (pPUC->rxPhy == HCI_PHY_2_MBPS) ? "2M" :
                                    "CODED"));
              DEBUG_NEWLINE();
            }

            SPPBLEServer_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT, (uint8_t *)pMsg);
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
 * @fn      SPPBLEServer_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SPPBLEServer_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_printf(dispHandle, SBS_ROW_STATUS_1, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    currentMTUSize = pMsg->msg.mtuEvt.MTU;
    SDITask_setAppDataSize(currentMTUSize);
    DEBUG("MTU Size: "); DEBUG((uint8_t*)convInt32ToText((int)currentMTUSize)); DEBUG_NEWLINE();
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SPPBLEServer_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SPPBLEServer_processAppMsg(spEvt_t *pMsg)
{
  switch (pMsg->event)
  {
    case SBS_CHAR_CHANGE_EVT:
      SPPBLEServer_processCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;

    case SBS_KEY_CHANGE_EVT:
      SPPBLEServer_handleKeys(*(uint8_t*)(pMsg->pData));
      break;

    case SBS_ADV_EVT:
      SPPBLEServer_processAdvEvent((spGapAdvEventData_t*)(pMsg->pData));
      break;

    case SBS_PAIR_STATE_EVT:
      SPPBLEServer_processPairState((spPairStateData_t*)(pMsg->pData));
      break;

    case SBS_PASSCODE_EVT:
      SPPBLEServer_processPasscode((spPasscodeData_t*)(pMsg->pData));
      break;

    case SBS_PERIODIC_EVT:
      SPPBLEServer_performPeriodicTask();
      break;

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
    case SBS_READ_RPA_EVT:
      SPPBLEServer_updateRPA();
      break;
#endif // PRIVACY_1_2_CFG

    case SBS_SEND_PARAM_UPDATE_EVT:
    {
      // Extract connection handle from data
      uint16_t connHandle = *(uint16_t *)(((spClockEventData_t *)pMsg->pData)->data);

      SPPBLEServer_processParamUpdate(connHandle);
      break;
    }

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
 * @fn      SPPBLEServer_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void SPPBLEServer_processGapMessage(gapEventHdr_t *pMsg)
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

        Display_printf(dispHandle, SBS_ROW_STATUS_1, 0, "Initialized");
        DEBUG("Initialized...");

        // Display the address of this device
        DEBUG("BD ADDR: "); DEBUG((uint8_t*)Util_convertBdAddr2Str(pPkt->devAddr));
        DEBUG_NEWLINE();

        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        // Temporary memory for advertising parameters for set #1. These will be copied
        // by the GapAdv module
        GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&SPPBLEServer_advCallback, &advParamLegacy,
                               &advHandleLegacy);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advertData), advertData);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanRspData), scanRspData);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Display device address
        Display_printf(dispHandle, SBS_ROW_IDA, 0, "%s Addr: %s",
                       (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
                       Util_convertBdAddr2Str(pPkt->devAddr));

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
        if (addrMode > ADDRMODE_RANDOM)
        {
          SPPBLEServer_updateRPA();

          // Create one-shot clock for RPA check event.
          Util_constructClock(&clkRpaRead, SPPBLEServer_clockHandler,
                              SBS_READ_RPA_EVT_PERIOD, 0, true,
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
      DEBUG("Num Conns: "); DEBUG((uint8_t*)convInt32ToText((uint32_t)numActive));
      DEBUG_NEWLINE();

      if (pPkt->hdr.status == SUCCESS)
      {
        // Add connection to list and start RSSI
        SPPBLEServer_addConn(pPkt->connectionHandle);
        currentConnHandle = pPkt->connectionHandle;

        // Display the address of this connection
        DEBUG("CONNECTED to: "); DEBUG((uint8_t*)Util_convertBdAddr2Str(pPkt->devAddr));
        DEBUG_NEWLINE();

        // Toggle LED to indicate connection status
        SPPBLEServer_toggleLed(Board_PIN_GLED, Board_LED_TOGGLE);

        // Start Periodic Clock.
        Util_startClock(&clkPeriodic);
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

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      DEBUG("DISCONNECTED...");
      DEBUG("Num Conns: "); DEBUG((uint8_t*)convInt32ToText((uint32_t)numActive));
      DEBUG_NEWLINE();

      // Toggle LED to indicate connection status
      SPPBLEServer_toggleLed(Board_PIN_GLED, Board_LED_TOGGLE);

      // Remove the connection from the list and disable RSSI if needed
      SPPBLEServer_removeConn(pPkt->connectionHandle);

      // If no active connections
      if (numActive == 0)
      {
        // Stop periodic clock
        Util_stopClock(&clkPeriodic);
      }

      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

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
        Display_printf(dispHandle, SBS_ROW_STATUS_2, 0, "Link Param Updated: %s",
                       Util_convertBdAddr2Str(linkInfo.addr));
      }
      else
      {
        // Display the address of the connection update failure
        Display_printf(dispHandle, SBS_ROW_STATUS_2, 0,
                       "Link Param Update Failed 0x%x: %s", pPkt->opcode,
                       Util_convertBdAddr2Str(linkInfo.addr));
      }

      // Check if there are any queued parameter updates
      spConnHandleEntry_t *connHandleEntry = (spConnHandleEntry_t *)List_get(&paramUpdateList);
      if (connHandleEntry != NULL)
      {
        // Attempt to send queued update now
        SPPBLEServer_processParamUpdate(connHandleEntry->connHandle);

        // Free list element
        ICall_free(connHandleEntry);
      }

      break;
    }

    default:
      Display_clearLines(dispHandle, SBS_ROW_STATUS_1, SBS_ROW_STATUS_2);
      break;
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_charValueChangeCB
 *
 * @brief   Callback from Serial Port Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SPPBLEServer_charValueChangeCB(uint8_t paramId)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = paramId;

    SPPBLEServer_enqueueMsg(SBS_CHAR_CHANGE_EVT, pValue);
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void SPPBLEServer_processCharValueChangeEvt(uint8_t paramId)
{
  switch(paramId)
  {
    case SERIALPORTSERVICE_CHAR_CONFIG:
      // TODO: Add application processing here
      break;
    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBS_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SPPBLEServer_performPeriodicTask(void)
{
  //TODO: Add periodic task for application here
}

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
/*********************************************************************
 * @fn      SPPBLEServer_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SPPBLEServer_updateRPA(void)
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
 * @fn      SPPBLEServer_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SPPBLEServer_clockHandler(UArg arg)
{
  spClockEventData_t *pData = (spClockEventData_t *)arg;

  if (pData->event == SBS_PERIODIC_EVT)
  {
    // Start the next period
    Util_startClock(&clkPeriodic);

    // Post event to wake up the application
    SPPBLEServer_enqueueMsg(SBS_PERIODIC_EVT, NULL);
  }
  else if (pData->event == SBS_READ_RPA_EVT)
  {
    // Start the next period
    Util_startClock(&clkRpaRead);

    // Post event to read the current RPA
    SPPBLEServer_enqueueMsg(SBS_READ_RPA_EVT, NULL);
  }
  else if (pData->event == SBS_SEND_PARAM_UPDATE_EVT)
  {
    // Send message to app
    SPPBLEServer_enqueueMsg(SBS_SEND_PARAM_UPDATE_EVT, pData);
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys - bitmap of pressed keys
 *
 * @return  none
 */
static void SPPBLEServer_keyChangeHandler(uint8_t keys)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = keys;

    SPPBLEServer_enqueueMsg(SBS_KEY_CHANGE_EVT, pValue);
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 */
static void SPPBLEServer_handleKeys(uint8_t keys)
{
  menuConnHandle = currentConnHandle;
  static uint8_t index = 0;

  if (keys & KEY_LEFT)
  {
    // Check if the key is still pressed. Workaround for possible bouncing.
    if (PIN_getInputValue(Board_PIN_BUTTON0) == 0)
    {
      // Send test data
      if (linkDB_Up(currentConnHandle))
      {
        uint8_t status;

        SPPBLEServer_toggleLed(Board_PIN_RLED, Board_LED_TOGGLE);

        for(int x =0; x< CHAR_LEN; x++)
          charVal[x] = ('A'+x);

        // Send the notification
        status = SerialPortService_SetParameter(SERIALPORTSERVICE_CHAR_DATA, CHAR_LEN, &charVal);

        if(status != SUCCESS){
          DEBUG("Noti fail");
        }
      }
    }
  }
  else if (keys & KEY_RIGHT)
  {

    // Check if the key is still pressed. Workaround for possible bouncing.
    if (PIN_getInputValue(Board_PIN_BUTTON1) == 0)
    {

      SPPBLEServer_toggleLed(Board_PIN_GLED, Board_LED_TOGGLE);

      if(linkDB_Up(currentConnHandle))
      {
        switch(index)
        {
        case 0:
          DEBUG("Changing PHY to 2M...");
          index = 1; // 2M
          SPPBLEServer_doSetConnPhy(index);
          break;
        case 1:
          DEBUG("Changing PHY to Coded...");
          index = 3; // Coded
          SPPBLEServer_doSetConnPhy(index);
          break;
        case 3:
          DEBUG("Changing PHY to ALL...");
          index = 4; // 1M,2M,Coded
          SPPBLEServer_doSetConnPhy(index);
          break;
        case 4:
          DEBUG("Changing PHY to Auto PHY...");
          index = 5; // Auto PHY
          SPPBLEServer_startAutoPhyChange(currentConnHandle);
          break;
        default:
          DEBUG("Changing PHY to 1M...");
          SPPBLEServer_stopAutoPhyChange(currentConnHandle);
          index = 0; // 1M
          SPPBLEServer_doSetConnPhy(index);
          break;
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_doSetConnPhy
 *
 * @brief   Set PHY preference.
 *
 * @param   index - 0: 1M PHY
 *                  1: 2M PHY
 *                  2: 1M + 2M PHY
 *                  3: CODED PHY (Long range)
 *                  4: 1M + 2M + CODED PHY
 *
 * @return  always true
 */
bool SPPBLEServer_doSetConnPhy(uint8 index)
{
  bool status = TRUE;

  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS | HCI_PHY_CODED,
    AUTO_PHY_UPDATE
  };

  uint8_t connIndex = SPPBLEServer_getConnIndex(menuConnHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX.
  // If auto PHY update is not selected and if auto PHY update is enabled, then
  // stop auto PHY update
  // Note PHYs are already enabled by default in build_config.opt in stack project.
  if(phy[index] != AUTO_PHY_UPDATE)
  {
    // Cancel RSSI reading  and auto phy changing
    SPPBLEServer_stopAutoPhyChange(connList[connIndex].connHandle);

    SPPBLEServer_setPhy(menuConnHandle, 0, phy[index], phy[index], 0);

    DEBUG("PHY preference: "); DEBUG((uint8_t*)strPHY[index]); DEBUG_NEWLINE();
  }
  else
  {
    // Start RSSI read for auto PHY update (if it is disabled)
    SPPBLEServer_startAutoPhyChange(menuConnHandle);
  }

  return status;
}
/*********************************************************************
 * @fn      SPPBLEServer_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void SPPBLEServer_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  spGapAdvEventData_t *pData = ICall_malloc(sizeof(spGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    SPPBLEServer_enqueueMsg(SBS_ADV_EVT, pData);
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void SPPBLEServer_processAdvEvent(spGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      DEBUG("Advertising..."); DEBUG_NEWLINE();
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
      DEBUG("Advertising Disabled"); DEBUG_NEWLINE();
      break;

    case GAP_EVT_ADV_START:
      break;

    case GAP_EVT_ADV_END:
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
      DEBUG("ADV set terminated"); DEBUG_NEWLINE();
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
 * @fn      SPPBLEServer_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SPPBLEServer_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  spPairStateData_t *pData = ICall_malloc(sizeof(spPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    SPPBLEServer_enqueueMsg(SBS_PAIR_STATE_EVT, pData);
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SPPBLEServer_passcodeCb(uint8_t *pDeviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  spPasscodeData_t *pData = ICall_malloc(sizeof(spPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    SPPBLEServer_enqueueMsg(SBS_PASSCODE_EVT, pData);
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SPPBLEServer_processPairState(spPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
      Display_printf(dispHandle, SBS_ROW_CONNECTION, 0, "Pairing started");
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, SBS_ROW_CONNECTION, 0, "Pairing success");
      }
      else
      {
        Display_printf(dispHandle, SBS_ROW_CONNECTION, 0, "Pairing fail: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, SBS_ROW_CONNECTION, 0, "Encryption success");
      }
      else
      {
        Display_printf(dispHandle, SBS_ROW_CONNECTION, 0, "Encryption failed: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, SBS_ROW_CONNECTION, 0, "Bond save success");
      }
      else
      {
        Display_printf(dispHandle, SBS_ROW_CONNECTION, 0, "Bond save failed: %d", status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SPPBLEServer_processPasscode(spPasscodeData_t *pPasscodeData)
{
  // Display passcode to user
  if (pPasscodeData->uiOutputs != 0)
  {
    Display_printf(dispHandle, SBS_ROW_CONNECTION, 0, "Passcode: %d",
                   B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      SPPBLEServer_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event  - message event.
 * @param   status - message status.
 *
 * @return  None.
 */
void SPPBLEServer_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len)
{
  sbpUARTEvt_t *pMsg;
  queueRec_t *pRec;

  // Enqueue message to be sent over the air
  {
    // Create dynamic pointer to message.
    if (pMsg = ICall_malloc(sizeof(sbpUARTEvt_t)))
    {
      pMsg->event = event;
      pMsg->pData = (uint8 *)ICall_allocMsg(len);
      if(pMsg->pData)
      {
        // payload
        memcpy(pMsg->pData , data, len);
      }
      pMsg->length = len;

      // Enqueue the message.
      if ((pRec = ICall_malloc(sizeof(queueRec_t))))
      {
        pRec->pData = (uint8*)pMsg;
        // This is an atomic operation
        Queue_put(appUARTMsgQueue, &pRec->_elem);

        Event_post(syncEvent, SBS_UART_QUEUE_EVT);
      }else
      {
        DEBUG("appUARTMsgQueue ERROR");
        ICall_free(pMsg);
      }
    }
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static void SPPBLEServer_enqueueMsg(uint8_t event, void *pData)
{
  spEvt_t *pMsg = ICall_malloc(sizeof(spEvt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
  }
}

/*********************************************************************
 * @fn      SPPBLEServer_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SPPBLEServer_addConn(uint16_t connHandle)
{
  uint8_t i;
  uint8_t status = bleNoResources;
  spClockEventData_t *paramUpdateEventData;

  // Try to find an available entry
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;

      // Allocate data to send through clock handler
      paramUpdateEventData = ICall_malloc(sizeof(spClockEventData_t) +
                                          sizeof (uint16_t));
      if(paramUpdateEventData)
      {
        paramUpdateEventData->event = SBS_SEND_PARAM_UPDATE_EVT;
        *((uint16_t *)paramUpdateEventData->data) = connHandle;

        // Create a clock object and start
        connList[i].pUpdateClock
          = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

        if (connList[i].pUpdateClock)
        {
          Util_constructClock(connList[i].pUpdateClock,
                              SPPBLEServer_clockHandler,
                              SBS_SEND_PARAM_UPDATE_DELAY, 0, false, //ZH
                              (UArg) paramUpdateEventData);
        }
      }
      else
      {
        status = bleMemAllocError;
      }

      // Set default PHY to 1M
      connList[i].currPhy = HCI_PHY_1_MBPS;

      break;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      SPPBLEServer_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SPPBLEServer_getConnIndex(uint16_t connHandle)
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
 * @fn      SPPBLEServer_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. CONNHANDLE_ALL will always succeed.
 */
static uint8_t SPPBLEServer_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != CONNHANDLE_ALL)
  {
    // Get connection index from handle
    connIndex = SPPBLEServer_getConnIndex(connHandle);
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
      memset(connList[i].rssiArr, 0, SBS_MAX_RSSI_STORE_DEPTH);
      connList[i].rssiAvg = 0;
      connList[i].rssiCntr = 0;
      connList[i].isAutoPHYEnable = FALSE;
    }
  }

  return(SUCCESS);
}

/*********************************************************************
 * @fn      SPPBLEServer_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SPPBLEServer_removeConn(uint16_t connHandle)
{
  uint8_t connIndex = SPPBLEServer_getConnIndex(connHandle);

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
    // Stop Auto PHY Change
    SPPBLEServer_stopAutoPhyChange(connHandle);
    // Clear Connection List Entry
    SPPBLEServer_clearConnListEntry(connHandle);
  }

  return connIndex;
}

/*********************************************************************
 * @fn      SPPBLEServer_processParamUpdate
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static void SPPBLEServer_processParamUpdate(uint16_t connHandle)
{
  gapUpdateLinkParamReq_t req;
  uint8_t connIndex;

  req.connectionHandle = connHandle;
  req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
  req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
  req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
  req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

  connIndex = SPPBLEServer_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

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
static void SPPBLEServer_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  uint8_t status = pMsg->pReturnParam[0];

  // Find which command this command complete is for
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
      // Display RSSI value, if RSSI is higher than threshold, change to faster PHY
      if (status == SUCCESS)
      {
        uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1], pMsg->pReturnParam[2]);

        uint8_t index = SPPBLEServer_getConnIndex(handle);
        SIMPLEPERIPHERAL_ASSERT(index < MAX_NUM_BLE_CONNS);

        connList[index].rssiArr[connList[index].rssiCntr++] =
                                                  (int8_t)pMsg->pReturnParam[3];
        connList[index].rssiCntr %= SBS_MAX_RSSI_STORE_DEPTH;

        int16_t sum_rssi = 0;
        for(uint8_t cnt=0; cnt<SBS_MAX_RSSI_STORE_DEPTH; cnt++)
        {
          sum_rssi += connList[index].rssiArr[cnt];
        }
        connList[index].rssiAvg = (uint32_t)(sum_rssi/SBS_MAX_RSSI_STORE_DEPTH);

        Display_printf(dispHandle, SBS_ROW_RSSI, 0,
                       "RSSI:-%d, AVG RSSI:-%d",
                       (uint32_t)(-(int8_t)pMsg->pReturnParam[3]),
                       (uint32_t)(-sum_rssi/SBS_MAX_RSSI_STORE_DEPTH));

        uint8_t phyRq = SBS_PHY_NONE;
        uint8_t phyRqS = SBS_PHY_NONE;
        uint8_t phyOpt = LL_PHY_OPT_NONE;

        if(connList[index].phyCngRq == FALSE)
        {
          if((connList[index].rssiAvg >= RSSI_2M_THRSHLD) &&
             (connList[index].currPhy != HCI_PHY_2_MBPS) &&
             (connList[index].currPhy != SBS_PHY_NONE))
          {
            // try to go to higher data rate
            phyRqS = phyRq = HCI_PHY_2_MBPS;
          }
          else if((connList[index].rssiAvg < RSSI_2M_THRSHLD) &&
                  (connList[index].rssiAvg >= RSSI_1M_THRSHLD) &&
                  (connList[index].currPhy != HCI_PHY_1_MBPS) &&
                  (connList[index].currPhy != SBS_PHY_NONE))
          {
            // try to go to legacy regular data rate
            phyRqS = phyRq = HCI_PHY_1_MBPS;
          }
          else if((connList[index].rssiAvg >= RSSI_S2_THRSHLD) &&
                  (connList[index].rssiAvg < RSSI_1M_THRSHLD) &&
                  (connList[index].currPhy != SBS_PHY_NONE))
          {
            // try to go to lower data rate S=2(500kb/s)
            phyRqS = HCI_PHY_CODED;
            phyOpt = LL_PHY_OPT_S2;
            phyRq = BLE5_CODED_S2_PHY;
          }
          else if(connList[index].rssiAvg < RSSI_S2_THRSHLD )
          {
            // try to go to lowest data rate S=8(125kb/s)
            phyRqS = HCI_PHY_CODED;
            phyOpt = LL_PHY_OPT_S8;
            phyRq = BLE5_CODED_S8_PHY;
          }
          if((phyRq != SBS_PHY_NONE) &&
             // First check if the request for this phy change is already not honored then don't request for change
             (((connList[index].rqPhy == phyRq) &&
               (connList[index].phyRqFailCnt < 2)) ||
              (connList[index].rqPhy != phyRq)))
          {
            //Initiate PHY change based on RSSI
            SPPBLEServer_setPhy(connList[index].connHandle, 0,
                                    phyRqS, phyRqS, phyOpt);
            connList[index].phyCngRq = TRUE;

            // If it a request for different phy than failed request, reset the count
            if(connList[index].rqPhy != phyRq)
            {
              // then reset the request phy counter and requested phy
              connList[index].phyRqFailCnt = 0;
            }

            if(phyOpt == LL_PHY_OPT_NONE)
            {
              connList[index].rqPhy = phyRq;
            }
            else if(phyOpt == LL_PHY_OPT_S2)
            {
              connList[index].rqPhy = BLE5_CODED_S2_PHY;
            }
            else
            {
              connList[index].rqPhy = BLE5_CODED_S8_PHY;
            }

          }
        } // end of if(connList[index].phyCngRq == FALSE)
      } // end of if (status == SUCCESS)
      break;
    }

    case HCI_LE_READ_PHY:
    {
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, SBS_ROW_RSSI + 2, 0, "RXPh: %d, TXPh: %d",
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
        Display_printf(dispHandle, SBS_ROW_RPA, 0, "RP Addr: %s",
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
* @fn      SPPBLEServer_initPHYRSSIArray
*
* @brief   Initializes the array of structure/s to store data related
*          RSSI based auto PHy change
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static void SPPBLEServer_initPHYRSSIArray(void)
{
  //Initialize array to store connection handle and RSSI values
  memset(connList, 0, sizeof(connList));
  for (uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++)
  {
    connList[index].connHandle = SBS_INVALID_HANDLE;
  }
}
/*********************************************************************
      // Set default PHY to 1M
 * @fn      SPPBLEServer_startAutoPhyChange
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   devAddr - device address
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static status_t SPPBLEServer_startAutoPhyChange(uint16_t connHandle)
{
  status_t status = FAILURE;

  // Get connection index from handle
  uint8_t connIndex = SPPBLEServer_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Start Connection Event notice for RSSI calculation
  status = HCI_EXT_ConnEventNoticeCmd(connHandle, selfEntity,
                                      CONN_INDEX_TO_EVENT(connIndex));

  // Flag in connection info if successful
  if (status == SUCCESS)
  {
    connList[connIndex].isAutoPHYEnable = TRUE;
  }

  return status;
}

/*********************************************************************
 * @fn      SPPBLEServer_stopAutoPhyChange
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static status_t SPPBLEServer_stopAutoPhyChange(uint16_t connHandle)
{
  // Get connection index from handle
  uint8_t connIndex = SPPBLEServer_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Stop connection event notice
  HCI_EXT_ConnEventNoticeCmd(connHandle, selfEntity, 0);

  // Also update the phychange request status for active RSSI tracking connection
  connList[connIndex].phyCngRq = FALSE;
  connList[connIndex].isAutoPHYEnable = FALSE;

  return SUCCESS;
}

/*********************************************************************
 * @fn      SPPBLEServer_setPhy
 *
 * @brief   Call the HCI set phy API and and add the handle to a
 *          list to match it to an incoming command status event
 */
static status_t SPPBLEServer_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts)
{
  // Allocate list entry to store handle for command status
  spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));

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
* @fn      SPPBLEServer_updatePHYStat
*
* @brief   Update the auto phy update state machine
*
* @param   connHandle - the connection handle
*
* @return  None
*/
static void SPPBLEServer_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
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
        connIndex = SPPBLEServer_getConnIndex(connHandleEntry->connHandle);

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
        connIndex = SPPBLEServer_getConnIndex(pPUC->connHandle);

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

/*******************************************************************************
* @fn          convInt32ToText
*
* @brief       Converts 32 bit int to text
*
* @param       int32 value
*
* @return      char* - pointer to text buffer which is a file scope allocated array
*/
char* convInt32ToText(int32 value) {
    static char pValueToTextBuffer[12];
    char *pLast;
    char *pFirst;
    char last;
    uint8 negative;

    pLast = pValueToTextBuffer;

    // Record the sign of the value
    negative = (value < 0);
    value = ABS(value);

    // Print the value in the reverse order
    do {
        *(pLast++) = '0' + (uint8)(value % 10);
        value /= 10;
    } while (value);

    // Add the '-' when the number is negative, and terminate the string
    if (negative) *(pLast++) = '-';
    *(pLast--) = 0x00;

    // Now reverse the string
    pFirst = pValueToTextBuffer;
    while (pLast > pFirst) {
        last = *pLast;
        *(pLast--) = *pFirst;
        *(pFirst++) = last;
    }

    return pValueToTextBuffer;
}
/*********************************************************************
*********************************************************************/
