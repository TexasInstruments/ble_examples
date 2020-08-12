/******************************************************************************

 @file       heart_rate.c

 @brief This file contains the Heart Rate sample application for use with the
 CC26xx Bluetooth Low Energy Protocol Stack.

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
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, HEARTRATEECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_ble_example_pack_01_50_00_62
 Release Date: 2017-11-01 10:38:41
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

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "heartrateservice.h"
#include "devinfoservice.h"
#include "battservice.h"

#include "board_key.h"

#include "heart_rate.h"

/*********************************************************************
 * MACROS
 */

// Convert BPM to RR-Interval for data simulation purposes.
#define HEARTRATE_BPM2RR(bpm)            ((uint16) 60 * 1024 / (uint16) (bpm))

/*********************************************************************
 * CONSTANTS
 */

// Address mode of the local device
// Note: When using the DEFAULT_ADDRESS_MODE as ADDRMODE_RANDOM or
// ADDRMODE_RP_WITH_RANDOM_ID, GAP_DeviceInit() should be called with
// it's last parameter set to a static random address
#define DEFAULT_ADDRESS_MODE                  ADDRMODE_PUBLIC

// How often to perform heart rate periodic event.
#define DEFAULT_HEARTRATE_PERIOD                        2000

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled.
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL               200

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled.
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL               1600

// Slave latency to use if automatic parameter update request is enabled.
#define DEFAULT_DESIRED_SLAVE_LATENCY                   1

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled.
#define DEFAULT_DESIRED_CONN_TIMEOUT                    1000

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_PASS_TO_APP

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL                     6

// Battery measurement period in ms
#define DEFAULT_BATT_PERIOD                             15000

// How often to read current current RPA (in ms)
#define HEARTRATE_READ_RPA_EVT_PERIOD               3000

// Delay (in ms) after connection establishment before sending a parameter update requst
#define HEARTRATE_SEND_PARAM_UPDATE_DELAY           6000

// Arbitrary values used to simulate measurements.
#define HEARTRATE_BPM_DEFAULT                           73
#define HEARTRATE_BPM_MAX                               80
#define HEARTRATE_ENERGY_INCREMENT                      10
#define HEARTRATE_FLAGS_IDX_MAX                         7

#define HEARTRATE_MEAS_LEN                              9

// Task configuration
#define HEARTRATE_TASK_PRIORITY                         1
#define HEARTRATE_TASK_STACK_SIZE                       644

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define HEARTRATE_ADDR_STR_SIZE     15

//Application Events
#define HEARTRATE_KEY_CHANGE_EVT                        0x0001
#define HEARTRATE_MEAS_EVT                              0x0002
#define HEARTRATE_BATT_EVT                              0x0004
#define HEARTRATE_PASSCODE_NEEDED_EVT                   0x0008
#define HEARTRATE_CONN_EVT                              0x0010
#define HEARTRATE_ADV_EVT                               0x0012
#define HEARTRATE_READ_RPA_EVT                          0x0014
#define HEARTRATE_SEND_PARAM_UPDATE_EVT                 0x0016
#define HEARTRATE_PERIODIC_EVT                          0x0018
#define HEARTRATE_INVALID_HANDLE                        0x0020

// Internal events for RTOS application.
#define HEARTRATE_ICALL_EVT                             ICALL_MSG_EVENT_ID  // Event_Id_31
#define HEARTRATE_QUEUE_EVT                             UTIL_QUEUE_EVENT_ID // Event_Id_30
#define HEARTRATE_MEAS_PERIODIC_EVT                     Event_Id_00
#define HEARTRATE_BATT_PERIODIC_EVT                     Event_Id_01

#define HEARTRATE_ALL_EVENTS                            (HEARTRATE_ICALL_EVT         | \
                                                         HEARTRATE_QUEUE_EVT         | \
                                                         HEARTRATE_MEAS_PERIODIC_EVT | \
                                                         HEARTRATE_BATT_PERIODIC_EVT)

#define HEARTRATE_MEAS_LEN                              9

#define AUTO_PHY_UPDATE            0xFF

// For storing the active connections
#define HEARTRATE_MAX_RSSI_STORE_DEPTH    5
// Spin if the expression is not true
#define HEARTRATE_ASSERT(expr) if (!(expr)) heart_rate_spin();

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
    appEvtHdr_t hdr;  // Event header
    uint8_t *pData;  // event data
} heartRateEvt_t;

typedef enum
{
    NOT_REGISTER = 0,
    FOR_AOA_SCAN = 1,
    FOR_ATT_RHEARTRATE = 2,
    FOR_AOA_SEND = 4,
    FOR_TOF_SEND = 8
} connectionEventRegisterCause_u;

typedef struct
{
    uint32_t event;
    void *pBuf;
} heartRateGapAdvEventData_t;

typedef struct
{
    uint8_t event;                //
    uint8_t data[];
} heartRateClockEventData_t;

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
} heartRatePasscodeData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
    List_Elem elem;
    uint16_t connHandle;
} heartRateConnHandleEntry_t;

// Connected device information
typedef struct
{
    uint16_t connHandle;                        // Connection Handle
    heartRateClockEventData_t *pParamUpdateEventData;
    Clock_Struct *pUpdateClock;                      // pointer to clock struct
    int8_t rssiArr[HEARTRATE_MAX_RSSI_STORE_DEPTH];
    uint8_t rssiCntr;
    int8_t rssiAvg;
    bool phyCngRq;           // Set to true if PHY change request is in progress
    uint8_t currPhy;
    uint8_t rqPhy;
    uint8_t phyRqFailCnt;                      // PHY change request count
    bool isAutoPHYEnable;                   // Flag to indicate auto phy change
} heartRateConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t connectionEventRegisterCauseBitMap = NOT_REGISTER; //see connectionEventRegisterCause_u

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages.
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct measPerClock;

static Clock_Struct battPerClock;

// Per-handle connection info
static heartRateConnRec_t connList[MAX_NUM_BLE_CONNS];

// Memory to pass RPA read event ID to clock handler
heartRateClockEventData_t argRpaRead = { .event = HEARTRATE_READ_RPA_EVT };

// Memory to pass periodic event ID to clock handler
heartRateClockEventData_t argPeriodic = { .event = HEARTRATE_PERIODIC_EVT };
// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Task configuration
Task_Struct HeartRate_task;
Char HeartRate_taskStack[HEARTRATE_TASK_STACK_SIZE];

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkPeriodic;

// Scan Response Data
static uint8_t scanRspData[] = {
        // complete name
        18,// length of this data
        GAP_ADTYPE_LOCAL_NAME_COMPLETE, 'H', 'e', 'a', 'r', 't', ' ', 'R', 'a',
        't', 'e', ' ', 'S', 'e', 'n', 's', 'o', 'r',

        // connection interval range
        5,// length of this data
        GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE, LO_UINT16(
                DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
        HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), LO_UINT16(
                DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
        HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

        // Tx power level
        2,// length of this data
        GAP_ADTYPE_POWER_LEVEL, 0       // 0dBm
        };

//GAP - Advertisement data (max size = 31 bytes)
static uint8_t advertData[] = {
        // Flags
        0x02,
        GAP_ADTYPE_FLAGS,
        GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
        // Service UUIDs
        0x05,
        GAP_ADTYPE_16BIT_MORE,
        LO_UINT16(HEARTRATE_SERV_UUID), HI_UINT16(HEARTRATE_SERV_UUID),
        LO_UINT16(BATT_SERV_UUID), HI_UINT16(BATT_SERV_UUID) };

// Device name attribute value.
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Heart Rate Sensor";

// GAP connection handle.
static uint16_t gapConnHandle;

// Advertising handles
static uint8 advHandleLegacy;
static uint8 advHandleLongRange;

// Components of heart rate measurement structure.
static uint8_t heartRateBpm = HEARTRATE_BPM_DEFAULT;
static uint16_t heartRateEnergyLvl = 0;
static uint16_t heartRateRrInterval = HEARTRATE_BPM2RR(HEARTRATE_BPM_DEFAULT);
static uint16_t heartRateRrInterval2 = HEARTRATE_BPM2RR(HEARTRATE_BPM_DEFAULT);

// Advertising state
static bool isAdvertising = FALSE;

// Connection state
static bool isConnected = FALSE;

// Index for array below.
static uint8_t flagsIdx = 0;

// Flags for simulated measurements.
static const uint8_t heartRateflags[HEARTRATE_FLAGS_IDX_MAX] = {
        HEARTRATE_FLAGS_CONTACT_NOT_SUP,
        HEARTRATE_FLAGS_CONTACT_NOT_DET,
        HEARTRATE_FLAGS_CONTACT_DET | HEARTRATE_FLAGS_ENERGY_EXP,
        HEARTRATE_FLAGS_CONTACT_DET | HEARTRATE_FLAGS_RR,
        HEARTRATE_FLAGS_CONTACT_DET | HEARTRATE_FLAGS_ENERGY_EXP
                | HEARTRATE_FLAGS_RR,
        (HEARTRATE_FLAGS_FORMAT_UINT16 | HEARTRATE_FLAGS_CONTACT_DET |
        HEARTRATE_FLAGS_ENERGY_EXP | HEARTRATE_FLAGS_RR),
        0x00 };

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = { 0 };
#endif // PRIVACY_1_2_CFG

/*********************************************************************
 * LOCAL FUNCTIONS
 */

// Task functions and message processing.
static void HeartRate_init(void);
static void HeartRate_taskFxn(UArg a0, UArg a1);
static void HeartRate_processStackMsg(ICall_Hdr *pMsg);
static void HeartRate_processGattMsg(gattMsgEvent_t *pMsg);
static void HeartRate_processGapMessage(gapEventHdr_t *pMsg);

static void HeartRate_processAppMsg(heartRateEvt_t *pMsg);
static uint8_t HeartRate_enqueueMsg(uint8_t event, uint8_t state,
                                    uint8_t *pData);

static void HeartRate_clockHandler(UArg arg);
static void HeartRate_measPerTask(void);
static void HeartRate_battPerTask(void);
static void HeartRate_measNotify(void);
static bool HeartRate_toggleAdvertising(void);
static uint8_t HeartRate_addConn(uint16_t connHandle);
static void HeartRate_processParamUpdate(uint16_t connHandle);
static void HeartRate_initPHYRSSIArray(void);

static uint8_t HeartRate_clearConnListEntry(uint16_t connHandle);

static status_t HeartRate_stopAutoPhyChange(uint16_t connHandle);

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
static void HeartRate_updateRPA(void);
#endif // PRIVACY_1_2_CFG
// Events and callbacks for profiles and keys.
static void HeartRate_battCB(uint8_t event);
static void HeartRate_battEvt(uint8_t event);
static void HeartRate_serviceCB(uint8_t event);
static void HeartRate_heartRateEvt(uint8_t event);
static void HeartRate_keyPressHandler(uint8_t keys);
static void HeartRate_handleKeys(uint8_t shift, uint8_t keys);

static void HeartRate_advCallback(uint32_t event, void *pBuf, uintptr_t arg);

static uint8_t HeartRate_removeConn(uint16_t connHandle);
static uint8_t HeartRate_getConnIndex(uint16_t connHandle);

static void HeartRate_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                 uint8_t uiInputs, uint8_t uiOutputs,
                                 uint32_t numComparison);
static void HeartRate_processPasscode(heartRatePasscodeData_t *pPasscodeData);

static void HeartRate_processConnEvt(Gap_ConnEventRpt_t *pReport);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Bond Manager Callbacks.
static const gapBondCBs_t heartRateBondCB = {
        (pfnPasscodeCB_t) HeartRate_passcodeCB,    // Passcode callback
        NULL                     // Pairing state callback.
        };

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      heart_rate_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void heart_rate_spin(void)
{
    volatile uint8_t x = 0;

    while (1)
    {
        x++;
    }
}

/*********************************************************************
 * @fn      HeartRate_createTask
 *
 * @brief   Task creation function for the Heart Rate.
 *
 * @param   none
 *
 * @return  none
 */
void HeartRate_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = HeartRate_taskStack;
    taskParams.stackSize = HEARTRATE_TASK_STACK_SIZE;
    taskParams.priority = HEARTRATE_TASK_PRIORITY;

    Task_construct(&HeartRate_task, HeartRate_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      HeartRate_init
 *
 * @brief   Initialization function for the Heart Rate application thread.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
void HeartRate_init(void)
{
    // ******************************************************************
    // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

    // Create an RTOS queue for message from profile to be sent to app.
    appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

    // Create one-shot clocks for internal periodic events.
    Util_constructClock(&measPerClock, HeartRate_clockHandler,
    DEFAULT_HEARTRATE_PERIOD,
                        0, false,
                        HEARTRATE_MEAS_PERIODIC_EVT);
    Util_constructClock(&battPerClock, HeartRate_clockHandler,
    DEFAULT_BATT_PERIOD,
                        0, false,
                        HEARTRATE_BATT_PERIODIC_EVT);

    // Initialize keys on SRF06.
    Board_initKeys(HeartRate_keyPressHandler);

    // Initialize Connection List
    HeartRate_clearConnListEntry(CONNHANDLE_ALL);

    // Set the Device Name characteristic in the GAP GATT Service
    // For more information, see the section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Configure GAP
    {
        uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

        // Pass all parameter update requests to the app for it to decide
        GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
    }

    // Setup the GAP Bond Manager.
    {
        uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8_t mitm = FALSE;
        uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        uint8_t bonding = TRUE;

        GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t),
                                &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t),
                                &mitm);
        GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t),
                                &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t),
                                &bonding);
    }

    // Initialize GATT attributes.
    GGS_AddService(GATT_ALL_SERVICES);// GAP
    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

    // Add heart rate service.
    HeartRate_AddService(GATT_ALL_SERVICES);
    // Add device info service.
    DevInfo_AddService();
    // Add battery service.
    Batt_AddService();

    // Setup the Heart Rate Characteristic Values.
    {
        uint8_t sensLoc = HEARTRATE_SENS_LOC_WRIST;
        HeartRate_SetParameter(HEARTRATE_SENS_LOC, sizeof(uint8_t), &sensLoc);
    }

    // Setup Battery Characteristic Values.
    {
        uint8_t critical = DEFAULT_BATT_CRITICAL_LEVEL;
        Batt_SetParameter(BATT_PARAM_CRITICAL_LEVEL, sizeof(uint8_t),
                          &critical);
    }

    // Register for Heart Rate service callback.
    HeartRate_Register(&HeartRate_serviceCB);

    // Register for Battery service callback.
    Batt_Register(&HeartRate_battCB);

    // Start the Bond Manager.
    GAPBondMgr_Register((gapBondCBs_t* )&heartRateBondCB);

    // Register with GAP for HCI/Host messages
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Set default values for Data Length Extension
    // Extended Data Length Feature is already enabled by default
    {
        // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
        // Some brand smartphone is essentially needing 251/2120, so we set them here.
#define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
#define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

        // This API is documented in hci.h
        // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
        // http://software-dl.ti.com/lprf/ble5stack-latest/
        HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE,
                                               APP_SUGGESTED_TX_TIME);
    }

    // Initialize GATT Client
    GATT_InitClient();
    // Initialize Connection List
    HeartRate_clearConnListEntry(CONNHANDLE_ALL);

    //Initialize GAP layer for Peripheral role and register to receive GAP events
    GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, NULL);

    // Initialize array to store connection handle and RSSI values
    HeartRate_initPHYRSSIArray();

}

/*********************************************************************
 * @fn      RunningSensor_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void HeartRate_processPasscode(heartRatePasscodeData_t *pPasscodeData)
{
#if defined(GAP_BOND_MGR)
    // Send passcode response
    GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle, SUCCESS,
                           B_APP_DEFAULT_PASSCODE);
#endif
}

/*********************************************************************
 * @fn      HeartRate_initPHYRSSIArray
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
static void HeartRate_initPHYRSSIArray(void)
{
    //Initialize array to store connection handle and RSSI values
    memset(connList, 0, sizeof(connList));
    for (uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++)
    {
        connList[index].connHandle = HEARTRATE_INVALID_HANDLE;
    }
}

/*********************************************************************
 * @fn      HeartRate_clearConnListEntry
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. CONNHANDLE_ALL will always succeed.
 */
static uint8_t HeartRate_clearConnListEntry(uint16_t connHandle)
{
    uint8_t i;
    // Set to invalid connection index initially
    uint8_t connIndex = MAX_NUM_BLE_CONNS;

    if (connHandle != CONNHANDLE_ALL)
    {
        // Get connection index from handle
        connIndex = HeartRate_getConnIndex(connHandle);
        if (connIndex >= MAX_NUM_BLE_CONNS)
        {
            return (bleInvalidRange);
        }
    }

    // Clear specific handle or all handles
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if ((connIndex == i) || (connHandle == CONNHANDLE_ALL))
        {
            connList[i].connHandle = CONNHANDLE_INVALID;
            connList[i].currPhy = 0;
            connList[i].phyCngRq = 0;
            connList[i].phyRqFailCnt = 0;
            connList[i].rqPhy = 0;
            memset(connList[i].rssiArr, 0, HEARTRATE_MAX_RSSI_STORE_DEPTH);
            connList[i].rssiAvg = 0;
            connList[i].rssiCntr = 0;
            connList[i].isAutoPHYEnable = FALSE;
        }
    }

    return (SUCCESS);
}

/*********************************************************************
 * @fn      HeartRate_taskFxn
 *
 * @brief   Application task entry point for the Heart Rate.
 *
 * @param   none
 *
 * @return  none
 */
static void HeartRate_taskFxn(UArg a0, UArg a1)
{
    // Initialize the application.
    HeartRate_init();

    // Application main loop.
    for (;;)
    {
        uint32_t events;

        events = Event_pend(syncEvent, Event_Id_NONE, HEARTRATE_ALL_EVENTS,
        ICALL_TIMEOUT_FOREVER);

        if (events)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            if (ICall_fetchServiceMsg(&src, &dest,
                                      (void**) &pMsg) == ICALL_ERRNO_SUCCESS)
            {
                if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    // Process inter-task message.
                    HeartRate_processStackMsg((ICall_Hdr*) pMsg);
                }

                if (pMsg)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // If RTOS queue is not empty, process app message.
            if (events & HEARTRATE_QUEUE_EVT)
            {
                while (!Queue_empty(appMsgQueueHandle))
                {
                    heartRateEvt_t *pMsg = (heartRateEvt_t*) Util_dequeueMsg(
                            appMsgQueueHandle);
                    if (pMsg)
                    {
                        // Process message.
                        HeartRate_processAppMsg(pMsg);

                        // Free the space from the message.
                        ICall_free(pMsg);
                    }
                }
            }

            // Heart rate service periodic task.
            if (events & HEARTRATE_MEAS_PERIODIC_EVT)
            {
                HeartRate_measPerTask();
            }

            // Battery service periodic task.
            if (events & HEARTRATE_BATT_PERIODIC_EVT)
            {
                HeartRate_battPerTask();
            }
        }
    }
}

/*********************************************************************
 * @fn      HeartRate_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HeartRate_processStackMsg(ICall_Hdr *pMsg)
{
    switch (pMsg->event)
    {
    case GAP_MSG_EVENT:
        HeartRate_processGapMessage((gapEventHdr_t*) pMsg);
        break;
    case GATT_MSG_EVENT:
        HeartRate_processGattMsg((gattMsgEvent_t*) pMsg);
        break;

    case HCI_GAP_EVENT_EVENT:
    {

        // Process HCI message
        switch (pMsg->status)
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

            // Parse Command Complete Event for opcode and status
            hciEvt_CmdComplete_t *command_complete =
                    (hciEvt_CmdComplete_t*) pMsg;
            uint8_t pktStatus = command_complete->pReturnParam[0];

            //find which command this command complete is for
            switch (command_complete->cmdOpcode)
            {
            case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
            {
                if (pktStatus == SUCCESS)
                {
                    uint8_t featSet[8];

                    // get current feature set from received event (bits 1-9 of
                    // the returned data
                    memcpy(featSet, &command_complete->pReturnParam[1], 8);

                    // Clear bit 1 of byte 0 of feature set to disable LL
                    // Connection Parameter Updates
                    CLR_FEATURE_FLAG(featSet[0], LL_FEATURE_CONN_PARAMS_REQ);

                    // Update controller with modified features
                    HCI_EXT_SetLocalSupportedFeaturesCmd(featSet);
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

        default:
            break;
        }
    }
        break;

    default:
        // Do nothing.
        break;
    }
}

/*********************************************************************
 * @fn      HeartRate_processGattMsg
 *
 * @brief   Process GATT messages.
 *
 * @param   pMsg - pointer the the GATT message.
 *
 * @return  none
 */
static void HeartRate_processGattMsg(gattMsgEvent_t *pMsg)
{
    GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      HeartRate_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t HeartRate_getConnIndex(uint16_t connHandle)
{
    uint8_t i;

    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if (connList[i].connHandle == connHandle)
        {
            return i;
        }
    }

    return (MAX_NUM_BLE_CONNS);
}
/*********************************************************************
 * @fn      HeartRate_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HeartRate_processAppMsg(heartRateEvt_t *pMsg)
{
    switch (pMsg->hdr.event)
    {

    case HEARTRATE_KEY_CHANGE_EVT:
        HeartRate_handleKeys(0, pMsg->hdr.state);
        break;

    case HEARTRATE_MEAS_EVT:
        HeartRate_heartRateEvt(pMsg->hdr.state);
        break;

    case HEARTRATE_BATT_EVT:
        HeartRate_battEvt(pMsg->hdr.state);
        break;

        // Passcode event
    case HEARTRATE_PASSCODE_NEEDED_EVT:
        HeartRate_processPasscode((heartRatePasscodeData_t*) (pMsg->pData));
        break;
    case HEARTRATE_CONN_EVT:
    {
        HeartRate_processConnEvt((Gap_ConnEventRpt_t*) (pMsg->pData));

        ICall_free(pMsg->pData);
        break;
    }

    default:
        // Do nothing.
        break;
    }
}

/*********************************************************************
 * @fn      HeartRate_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void HeartRate_processGapMessage(gapEventHdr_t *pMsg)
{
    switch (pMsg->opcode)
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        bStatus_t status = FAILURE;

        gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t*) pMsg;

        if (pPkt->hdr.status == SUCCESS)
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
            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
                                 systemId);

            // Setup and start Advertising
            // For more information, see the GAP section in the User's Guide:
            // http://software-dl.ti.com/lprf/ble5stack-latest/

            // Temporary memory for advertising parameters for set #1. These will be copied
            // by the GapAdv module
            GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;

            // Create Advertisement set #1 and assign handle
            status = GapAdv_create(&HeartRate_advCallback, &advParamLegacy,
                                   &advHandleLegacy);
            HEARTRATE_ASSERT(status == SUCCESS);

            // Load advertising data for set #1 that is statically allocated by the app
            status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                         sizeof(advertData), advertData);
            HEARTRATE_ASSERT(status == SUCCESS);

            // Load scan response data for set #1 that is statically allocated by the app
            status = GapAdv_loadByHandle(advHandleLegacy,
                                         GAP_ADV_DATA_TYPE_SCAN_RSP,
                                         sizeof(scanRspData), scanRspData);
            HEARTRATE_ASSERT(status == SUCCESS);

            // Set event mask for set #1
            status = GapAdv_setEventMask(
                    advHandleLegacy,
                    GAP_ADV_EVT_MASK_START_AFTER_ENABLE
                            | GAP_ADV_EVT_MASK_END_AFTER_DISABLE
                            | GAP_ADV_EVT_MASK_SET_TERMINATED);
#if AUTO_ADV
            isAdvertising = TRUE;
            GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
#endif //AUTO_ADV

            // Use long range params to create long range set #2
            GapAdv_params_t advParamLongRange = GAPADV_PARAMS_AE_LONG_RANGE_CONN;

            // Create Advertisement set #2 and assign handle
            status = GapAdv_create(&HeartRate_advCallback,
                                   &advParamLongRange, &advHandleLongRange);
            HEARTRATE_ASSERT(status == SUCCESS);

            // Load advertising data for set #2 that is statically allocated by the app
            status = GapAdv_loadByHandle(advHandleLongRange,
                                         GAP_ADV_DATA_TYPE_ADV,
                                         sizeof(advertData), advertData);
            HEARTRATE_ASSERT(status == SUCCESS);

            // Set event mask for set #2
            status = GapAdv_setEventMask(
                    advHandleLongRange,
                    GAP_ADV_EVT_MASK_START_AFTER_ENABLE
                            | GAP_ADV_EVT_MASK_END_AFTER_DISABLE
                            | GAP_ADV_EVT_MASK_SET_TERMINATED);
#if AUTO_ADV
            isAdvertising = TRUE;
            GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
#endif //AUTO_ADV

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
            if (addrMode > ADDRMODE_RANDOM)
            {
                HeartRate_updateRPA();

                // Create one-shot clock for RPA check event.
                Util_constructClock(&clkRpaRead, HeartRate_clockHandler,
                HEARTRATE_READ_RPA_EVT_PERIOD,
                                    0, true, (UArg) &argRpaRead);
            }
#endif // PRIVACY_1_2_CFG
        }

        break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t*) pMsg;

        uint8_t numActive = linkDB_NumActive();

        if (pPkt->hdr.status == SUCCESS)
        {
            isConnected = TRUE;
            // Add connection to list and start RSSI
            HeartRate_addConn(pPkt->connectionHandle);

            // Start Periodic Clock.
            Util_startClock(&clkPeriodic);
        }

        if (numActive >= MAX_NUM_BLE_CONNS)
        {
            // Stop advertising since there is no room for more connections
            GapAdv_disable(advHandleLegacy);
            GapAdv_disable(advHandleLongRange);
            isAdvertising = FALSE;
        }

        break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
        gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t*) pMsg;
        isConnected = FALSE;
        GapAdv_disable(advHandleLegacy);
        isAdvertising = FALSE;

        uint8_t numActive = linkDB_NumActive();

        // Remove the connection from the list and disable RSSI if needed
        HeartRate_removeConn(pPkt->connectionHandle);

        // If no active connections
        if (numActive == 0)
        {
            // Stop periodic clock
            Util_stopClock(&clkPeriodic);

        }

        break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
        gapUpdateLinkParamReqReply_t rsp;

        gapUpdateLinkParamReqEvent_t *pReq =
                (gapUpdateLinkParamReqEvent_t*) pMsg;

        rsp.connectionHandle = pReq->req.connectionHandle;

        // Only accept connection intervals with slave latency of 0
        // This is just an example of how the application can send a response
        if (pReq->req.connLatency == 0)
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
        gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t*) pMsg;

        // Get the address from the connection handle
        linkDBInfo_t linkInfo;
        linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

        // Check if there are any queued parameter updates
        heartRateConnHandleEntry_t *connHandleEntry =
                (heartRateConnHandleEntry_t*) List_get(&paramUpdateList);
        if (connHandleEntry != NULL)
        {
            // Attempt to send queued update now
            HeartRate_processParamUpdate(connHandleEntry->connHandle);

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
 * @fn      HeartRate_processParamUpdate
 *
 * @brief   Process a parameters update request
 *
 * @return  None
 */
static void HeartRate_processParamUpdate(uint16_t connHandle)
{
    gapUpdateLinkParamReq_t req;
    uint8_t connIndex;

    req.connectionHandle = connHandle;
    req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

    connIndex = HeartRate_getConnIndex(connHandle);
    if (connIndex >= MAX_NUM_BLE_CONNS)
    {
        return;
    }

    // Deconstruct the clock object
    Clock_destruct(connList[connIndex].pUpdateClock);
    // Free clock struct, only in case it is not NULL
    if (connList[connIndex].pUpdateClock != NULL)
    {
        ICall_free(connList[connIndex].pUpdateClock);
        connList[connIndex].pUpdateClock = NULL;
    }
    // Free ParamUpdateEventData, only in case it is not NULL
    if (connList[connIndex].pParamUpdateEventData != NULL)
        ICall_free(connList[connIndex].pParamUpdateEventData);

    // Send parameter update
    bStatus_t status = GAP_UpdateLinkParamReq(&req);

    // If there is an ongoing update, queue this for when the udpate completes
    if (status == bleAlreadyInRequestedMode)
    {
        heartRateConnHandleEntry_t *connHandleEntry = ICall_malloc(
                sizeof(heartRateConnHandleEntry_t));
        if (connHandleEntry)
        {
            connHandleEntry->connHandle = connHandle;

            List_put(&paramUpdateList, (List_Elem*) connHandleEntry);
        }
    }
}
/*********************************************************************
 * @fn      HeartRate_keyPressHandler
 *
 * @brief   Key event handler function.
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void HeartRate_keyPressHandler(uint8_t keys)
{
    // Enqueue the event.
    HeartRate_enqueueMsg(HEARTRATE_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      HeartRate_clearPendingParamUpdate
 *
 * @brief   clean pending param update request in the paramUpdateList list
 *
 * @param   connHandle - connection handle to clean
 *
 * @return  none
 */
void HeartRate_clearPendingParamUpdate(uint16_t connHandle)
{
    List_Elem *curr;

    for (curr = List_head(&paramUpdateList); curr != NULL;
            curr = List_next(curr))
    {
        if (((heartRateConnHandleEntry_t*) curr)->connHandle == connHandle)
        {
            List_remove(&paramUpdateList, curr);
        }
    }
}

/*********************************************************************
 * @fn      HeartRate_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t HeartRate_removeConn(uint16_t connHandle)
{
    uint8_t connIndex = HeartRate_getConnIndex(connHandle);

    if (connIndex != MAX_NUM_BLE_CONNS)
    {
        Clock_Struct *pUpdateClock = connList[connIndex].pUpdateClock;

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
            // Free ParamUpdateEventData
            ICall_free(connList[connIndex].pParamUpdateEventData);
        }
        // Clear pending update requests from paramUpdateList
        HeartRate_clearPendingParamUpdate(connHandle);
        // Stop Auto PHY Change
        HeartRate_stopAutoPhyChange(connHandle);
        // Clear Connection List Entry
        HeartRate_clearConnListEntry(connHandle);
    }

    return connIndex;
}

/*********************************************************************
 * @fn      HeartRate_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t HeartRate_addConn(uint16_t connHandle)
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

            // Allocate data to send through clock handler
            connList[i].pParamUpdateEventData = ICall_malloc(
                    sizeof(heartRateClockEventData_t) + sizeof(uint16_t));
            if (connList[i].pParamUpdateEventData)
            {
                connList[i].pParamUpdateEventData->event =
                HEARTRATE_SEND_PARAM_UPDATE_EVT;
                *((uint16_t*) connList[i].pParamUpdateEventData->data) =
                        connHandle;

                // Create a clock object and start
                connList[i].pUpdateClock = (Clock_Struct*) ICall_malloc(
                        sizeof(Clock_Struct));

                if (connList[i].pUpdateClock)
                {
                    Util_constructClock(
                            connList[i].pUpdateClock, HeartRate_clockHandler,
                            HEARTRATE_SEND_PARAM_UPDATE_DELAY,
                            0, true,
                            (UArg) (connList[i].pParamUpdateEventData));
                }
                else
                {
                    ICall_free(connList[i].pParamUpdateEventData);
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
 * @fn      HeartRate_handleKeys
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
static void HeartRate_handleKeys(uint8_t shift, uint8_t keys)
{
    // Left key.
    if (keys & KEY_LEFT)
    {
        // If not in a connection, toggle advertising on and off.
        if (isConnected == FALSE)
        {
            // Toggle GAP advertisement status.
            // Set flag if advertising was cancelled.
            HeartRate_toggleAdvertising();
        }
    }

    // Right key.
    if (keys & KEY_RIGHT)
    {
        // Set simulated measurement flag index.
        if (++flagsIdx == HEARTRATE_FLAGS_IDX_MAX)
        {
            flagsIdx = 0;
        }
    }
}

/*********************************************************************
 * @fn      HeartRate_toggleAdvertising
 *
 * @brief   Toggle advertising state.
 *
 * @param   none
 *
 * @return  status - TRUE if advertising, FALSE otherwise.
 */
static bool HeartRate_toggleAdvertising(void)
{

    if (isAdvertising)
    {
        GapAdv_disable(advHandleLegacy);
        GapAdv_disable(advHandleLongRange);

        isAdvertising = FALSE;
    }
    else
    {
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        isAdvertising = TRUE;
    }
    return isAdvertising;
}

/*********************************************************************
 * @fn      HeartRate_measNotify
 *
 * @brief   Prepare and send a heart rate measurement notification.
 *
 * @return  none
 */
static void HeartRate_measNotify(void)
{
    attHandleValueNoti_t heartRateMeas;

    heartRateMeas.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_NOTI,
    HEARTRATE_MEAS_LEN,
                                         NULL);
    if (heartRateMeas.pValue != NULL)
    {
        uint8_t *p = heartRateMeas.pValue;
        uint8_t flags = heartRateflags[flagsIdx];

        // Build heart rate measurement structure from simulated values.
        *p++ = flags;
        *p++ = heartRateBpm;

        if (flags & HEARTRATE_FLAGS_FORMAT_UINT16)
        {
            // Additional byte for 16 bit format.
            *p++ = 0;
        }

        if (flags & HEARTRATE_FLAGS_ENERGY_EXP)
        {
            *p++ = LO_UINT16(heartRateEnergyLvl);
            *p++ = HI_UINT16(heartRateEnergyLvl);
        }

        if (flags & HEARTRATE_FLAGS_RR)
        {
            *p++ = LO_UINT16(heartRateRrInterval);
            *p++ = HI_UINT16(heartRateRrInterval);
            *p++ = LO_UINT16(heartRateRrInterval2);
            *p++ = HI_UINT16(heartRateRrInterval2);
        }

        heartRateMeas.len = (uint8) (p - heartRateMeas.pValue);

        // Send notification.
        if (HeartRate_MeasNotify(gapConnHandle, &heartRateMeas) != SUCCESS)
        {
            GATT_bm_free((gattMsg_t*) &heartRateMeas, ATT_HANDLE_VALUE_NOTI);
        }

        // Update simulated values.
        heartRateEnergyLvl += HEARTRATE_ENERGY_INCREMENT;
        if (++heartRateBpm == HEARTRATE_BPM_MAX)
        {
            heartRateBpm = HEARTRATE_BPM_DEFAULT;
        }

        heartRateRrInterval = heartRateRrInterval2 = HEARTRATE_BPM2RR(
                heartRateBpm);
    }
}

/*********************************************************************
 * @fn      HeartRate_serviceCB
 *
 * @brief   Callback function for heart rate service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void HeartRate_serviceCB(uint8_t event)
{
    // Enqueue the message.
    HeartRate_enqueueMsg(HEARTRATE_MEAS_EVT, event, NULL);
}

/*********************************************************************
 * @fn      HeartRate_heartRateEvt
 *
 * @brief   event handler for heart rate service callbacks.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void HeartRate_heartRateEvt(uint8_t event)
{
    if (event == HEARTRATE_MEAS_NOTI_ENABLED)
    {
        // If connected start periodic measurement.
        if (isConnected)
        {
            Util_startClock(&measPerClock);
        }
    }
    else if (event == HEARTRATE_MEAS_NOTI_DISABLED)
    {
        // Stop periodic measurement.
        Util_stopClock(&measPerClock);
    }
    else if (event == HEARTRATE_COMMAND_SET)
    {
        // Reset energy expended.
        heartRateEnergyLvl = 0;
    }
}

/*********************************************************************
 * @fn      HeartRate_battCB
 *
 * @brief   Callback function for battery service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void HeartRate_battCB(uint8_t event)
{
    // Enqueue the message.
    HeartRate_enqueueMsg(HEARTRATE_BATT_EVT, event, NULL);
}

/*********************************************************************
 * @fn      HeartRate_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void HeartRate_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
    heartRateGapAdvEventData_t *pData = ICall_malloc(
            sizeof(heartRateGapAdvEventData_t));

    if (pData)
    {
        pData->event = event;
        pData->pBuf = pBuf;

        if (HeartRate_enqueueMsg(HEARTRATE_ADV_EVT, NULL,
                                 (uint8_t*) (pData)) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

/*********************************************************************
 * @fn      HeartRate_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void HeartRate_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                 uint8_t uiInputs, uint8_t uiOutputs,
                                 uint32_t numComparison)
{
    uint8_t *pData;

// Allocate space for the passcode event.
    if ((pData = ICall_malloc(sizeof(uint8_t))))
    {
        *pData = uiOutputs;

// Enqueue the event.
        HeartRate_enqueueMsg(HEARTRATE_PASSCODE_NEEDED_EVT, 0, pData);
    }
}

/*********************************************************************
 * @fn      HeartRate_battEvt
 *
 * @brief   Event handler for battery service callbacks.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void HeartRate_battEvt(uint8_t event)
{
    if (event == BATT_LEVEL_NOTI_ENABLED)
    {
        // If connected start periodic measurement.
        if (isConnected)
        {
            Util_startClock(&battPerClock);
        }
    }
    else if (event == BATT_LEVEL_NOTI_DISABLED)
    {
        // Stop periodic measurement.
        Util_stopClock(&battPerClock);
    }
}

/*********************************************************************
 * @fn      HeartRate_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   none
 *
 * @return  none
 */
static void HeartRate_clockHandler(UArg arg)
{
    Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      HeartRate_measPerTask
 *
 * @brief   Perform a periodic heart rate measurement.
 *
 * @param   none
 *
 * @return  none
 */
static void HeartRate_measPerTask(void)
{
    if (isConnected)
    {
        // Send heart rate measurement notification.
        HeartRate_measNotify();

        // Restart timer.
        Util_startClock(&measPerClock);
    }
}

/*********************************************************************
 * @fn      HeartRate_battPerTask
 *
 * @brief   Perform a periodic task for battery measurement.
 *
 * @param   none
 *
 * @return  none
 */
static void HeartRate_battPerTask(void)
{
    if (isConnected)
    {
        // Perform battery level check.
        Batt_MeasLevel();

        // Restart timer.
        Util_startClock(&battPerClock);
    }
}
#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
/*********************************************************************
 * @fn      HeartRate_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void HeartRate_updateRPA(void)
{
    uint8_t *pRpaNew;

    // Read the current RPA.
    pRpaNew = GAP_GetDevAddress(FALSE);

    if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
    {
        memcpy(rpa, pRpaNew, B_ADDR_LEN);
    }
}
#endif // PRIVACY_1_2_CFG

/*********************************************************************
 * @fn      HeartRate_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  TRUE or FALSE
 */
static uint8_t HeartRate_enqueueMsg(uint8_t event, uint8_t state,
                                    uint8_t *pData)
{
    heartRateEvt_t *pMsg = ICall_malloc(sizeof(heartRateEvt_t));

// Create dynamic pointer to message.
    if (pMsg)
    {
        pMsg->hdr.event = event;
        pMsg->hdr.state = state;
        pMsg->pData = pData;

// Enqueue the message.
        return Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t*) pMsg);
    }

    return FALSE;
}

static status_t HeartRate_stopAutoPhyChange(uint16_t connHandle)
{
    // Get connection index from handle
    uint8_t connIndex = HeartRate_getConnIndex(connHandle);
    HEARTRATE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    // Stop connection event notice
    Gap_RegisterConnEventCb(NULL, GAP_CB_UNREGISTER, connHandle);

    // Also update the phychange request status for active RSSI tracking connection
    connList[connIndex].phyCngRq = FALSE;
    connList[connIndex].isAutoPHYEnable = FALSE;

    return SUCCESS;
}

/*********************************************************************
 * @fn      HeartRate_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void HeartRate_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
    // Get index from handle
    uint8_t connIndex = HeartRate_getConnIndex(pReport->handle);

    if (connIndex >= MAX_NUM_BLE_CONNS)
    {

        return;
    }

    // If auto phy change is enabled
    if (connList[connIndex].isAutoPHYEnable == TRUE)
    {
        // Read the RSSI
        HCI_ReadRssiCmd(pReport->handle);
    }

}

/*********************************************************************
 *********************************************************************/
