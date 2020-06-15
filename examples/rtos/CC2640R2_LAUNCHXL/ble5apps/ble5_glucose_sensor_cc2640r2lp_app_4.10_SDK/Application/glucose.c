/******************************************************************************

 @file       glucose.c

 @brief This file contains the Glucose Sensor sample application for use with
 the CC2650 Bluetooth Low Energy Protocol Stack.

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

#include "devinfoservice.h"
#include "glucservice.h"

#include "board_key.h"
#include "utc_clock.h"

#include "glucose.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Address mode of the local device
// Note: When using the DEFAULT_ADDRESS_MODE as ADDRMODE_RANDOM or
// ADDRMODE_RP_WITH_RANDOM_ID, GAP_DeviceInit() should be called with
// it's last parameter set to a static random address
#define DEFAULT_ADDRESS_MODE                  ADDRMODE_PUBLIC

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_PASS_TO_APP

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     TRUE //FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY //GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Notification period in ms
#define DEFAULT_NOTI_PERIOD                   100

// Delay (in ms) after connection establishment before sending a parameter update requst
#define GLUCOSE_SEND_PARAM_UPDATE_DELAY           6000

// How often to read current current RPA (in ms)
#define GLUCOSE_READ_RPA_EVT_PERIOD               3000

// Meas state bit field
#define GLUCOSE_MEAS_STATE_VALID              0x01
#define GLUCOSE_MEAS_STATE_FILTER_PASS        0x02
#define GLUCOSE_MEAS_STATE_ALL                (GLUCOSE_MEAS_STATE_VALID | GLUCOSE_MEAS_STATE_FILTER_PASS)

// Some values used to simulate measurements
#define MEAS_IDX_MAX                          sizeof(glucoseMeasArray)/sizeof(glucoseMeas_t)

// Maximum number of dynamically allocated measurements
// (must be less than MEAS_IDX_MAX)
#define DYNAMIC_REC_MAX                       1

#define GLUCOSE_MEAS_LEN                      17
#define GLUCOSE_CONTEXT_LEN                   17
#define GLUCOSE_CTL_PNT_LEN                   4

// Task configuration
#define GLUCOSE_TASK_PRIORITY                 1
#define GLUCOSE_TASK_STACK_SIZE               644

#define GLUCOSE_KEY_CHANGE_EVT                0x0000
#define GLUCOSE_SERVICE_EVT                   0x0002
#define GLUCOSE_PASSCODE_EVT                  0x0004
#define GLUCOSE_PAIRING_EVT                   0x0006
#define GLUCOSE_CONN_EVT                      0x0008
#define GLUCOSE_ADV_EVT                       0x0010
#define GLUCOSE_READ_RPA_EVT                  0x0012
#define GLUCOSE_SEND_PARAM_UPDATE_EVT         0x0014
#define GLUCOSE_PERIODIC_EVT                  0x0016
#define GLUCOSE_INVALID_HANDLE                0x0018

// Glucose Task Events
#define GLUCOSE_ICALL_EVT                     ICALL_MSG_EVENT_ID // Event_Id_31
#define GLUCOSE_QUEUE_EVT                     UTIL_QUEUE_EVENT_ID // Event_Id_30
#define GLUCOSE_NOTI_TIMEOUT_EVT              Event_Id_00

#define GLUCOSE_ALL_EVENTS                    (GLUCOSE_ICALL_EVT        | \
                                               GLUCOSE_QUEUE_EVT        | \
                                               GLUCOSE_NOTI_TIMEOUT_EVT)

#define AUTO_PHY_UPDATE            0xFF

// For storing the active connections
#define GLUCOSE_MAX_RSSI_STORE_DEPTH    5
// Spin if the expression is not true
#define GLUCOSE_ASSERT(expr) if (!(expr)) glucose_spin();

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
    appEvtHdr_t hdr; // Event header
    uint8_t *pData;  // Event data
} glucoseEvt_t;

typedef struct
{
    uint8_t *pVal; // pointer to value.
    uint8_t len;   // value length.
} glucoseServEvt_t;

typedef struct
{
    uint32_t event;
    void *pBuf;
} glucoseGapAdvEventData_t;

// Contains the data of control point command.
typedef struct
{
    uint8_t len;
    uint8_t data[GLUCOSE_CTL_PNT_MAX_SIZE];
} glucoseCtlPntMsg_t;

typedef struct
{
    uint8_t event;
    uint8_t data[];
} glucoseClockEventData_t;

// Data in a glucose measurement as defined in the profile.
typedef struct
{
    uint8_t state;
    uint8_t flags;
    uint16_t seqNum;
    UTCTimeStruct baseTime;
    int16 timeOffset;
    uint16_t concentration;
    uint8_t typeSampleLocation;
    uint16_t sensorStatus;
} glucoseMeas_t;

// Context data as defined in profile.
typedef struct
{
    uint8_t flags;
    uint16_t seqNum;
    uint8_t extendedFlags;
    uint8_t carboId;
    uint16_t carboVal;
    uint8_t mealVal;
    uint8_t TesterHealthVal;
    uint16_t exerciseDuration;
    uint8_t exerciseIntensity;
    uint8_t medId;
    uint16_t medVal;
    uint16_t HbA1cVal;
} glucoseContext_t;

typedef enum
{
    NOT_REGISTER = 0,
    FOR_AOA_SCAN = 1,
    FOR_ATT_RSP = 2,
    FOR_AOA_SEND = 4,
    FOR_TOF_SEND = 8
} connectionEventRegisterCause_u;

// List element for parameter update and PHY command status lists
typedef struct
{
    List_Elem elem;
    uint16_t connHandle;
} glucoseConnHandleEntry_t;

// Connected device information
typedef struct
{
    uint16_t connHandle;                        // Connection Handle
    glucoseClockEventData_t *pParamUpdateEventData;
    Clock_Struct *pUpdateClock;                      // pointer to clock struct
    int8_t rssiArr[GLUCOSE_MAX_RSSI_STORE_DEPTH];
    uint8_t rssiCntr;
    int8_t rssiAvg;
    bool phyCngRq;           // Set to true if PHY change request is in progress
    uint8_t currPhy;
    uint8_t rqPhy;
    uint8_t phyRqFailCnt;                      // PHY change request count
    bool isAutoPHYEnable;                   // Flag to indicate auto phy change
} glucoseConnRec_t;

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
static Clock_Struct notiTimeoutClock;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Task configuration.
Task_Struct glucoseTask;
Char glucoseTaskStack[GLUCOSE_TASK_STACK_SIZE];

// Per-handle connection info
static glucoseConnRec_t connList[MAX_NUM_BLE_CONNS];

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Memory to pass RPA read event ID to clock handler
glucoseClockEventData_t argRpaRead = { .event = GLUCOSE_READ_RPA_EVT };

// Memory to pass periodic event ID to clock handler
glucoseClockEventData_t argPeriodic = { .event = GLUCOSE_PERIODIC_EVT };

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkPeriodic;

// Scan Response Data
static uint8_t scanRspData[] = {
// complete name
        8,// length of this data
        GAP_ADTYPE_LOCAL_NAME_COMPLETE, 'G', 'l', 'u', 'c', 'o', 's', 'e',

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

static uint8_t advertData[] = {
// flags
        0x02,
        GAP_ADTYPE_FLAGS,
        GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
        // service UUIDs
        0x05,
        GAP_ADTYPE_16BIT_MORE,
        LO_UINT16(GLUCOSE_SERV_UUID), HI_UINT16(GLUCOSE_SERV_UUID), LO_UINT16(
                DEVINFO_SERV_UUID),
        HI_UINT16(DEVINFO_SERV_UUID) };

// Device name attribute value.
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Glucose Sensor";

// Bonded peer address.
static uint8_t glucoseBondedAddr[B_ADDR_LEN];

// GAP connection handle.
static uint16_t gapConnHandle;

// Set to true if context should be sent with measurement data.
static bool glucoseSendContext = false;

// If true, then send all valid and selected glucose measurements.
bool glucoseSendAllRecords = false;

// Advertising handles
static uint8 advHandleLegacy;
static uint8 advHandleLongRange;
// Advertising state
static bool isAdvertising = FALSE;

// Connection state
static bool isConnected = FALSE;

// For the example application we have hard coded glucose measurements.
// Note dates are in UTC time; day and month start at 0.
static glucoseMeas_t glucoseMeasArray[] = {
//Meas 1
        {
        GLUCOSE_MEAS_STATE_ALL,
          GLUCOSE_MEAS_FLAG_ALL,   //mol/L
          1, { 14, 17, 23, 22, 1, 2013 }, 0, 0xC050, // 0.008 (8.0 mmol)
          (GLUCOSE_LOCATION_FINGER | GLUCOSE_TYPE_CAPILLARY_WHOLE), 0 },
        //Meas 2
        {
        GLUCOSE_MEAS_STATE_ALL,
          GLUCOSE_MEAS_FLAG_ALL,   //mol/L
          2, { 6, 0, 0, 2, 1, 2012 }, 0,                      // Time offset
          0xC03C,                 // 0.006
          (GLUCOSE_LOCATION_AST | GLUCOSE_TYPE_CAPILLARY_PLASMA), 0xFFFF, // Status
        },
        //Meas 3
        {
        GLUCOSE_MEAS_STATE_ALL,
          GLUCOSE_MEAS_FLAG_ALL,                              //mol/L
          3, { 11, 23, 0, 5, 2, 2012 }, 0, 0xC037,                // 5.5 mmol/L
          (GLUCOSE_LOCATION_EARLOBE | GLUCOSE_TYPE_VENOUS_WHOLE), 0x5555 },
        // Time offset +1 hr
        //Meas 4
        {
        GLUCOSE_MEAS_STATE_ALL,
          GLUCOSE_MEAS_FLAG_ALL,                              //mol/L
          4, { 12, 2, 0, 14, 1, 2011 }, 60, 0xB165,  // 3.57 mmol /L
          (GLUCOSE_LOCATION_CONTROL | GLUCOSE_TYPE_VENOUS_PLASMA), 0xAAAA },
        //Meas 5
        {
        GLUCOSE_MEAS_STATE_ALL,
          GLUCOSE_MEAS_FLAG_ALL,                              //mol/L
          5, { 13, 5, 12, 12, 1, 2011 }, 60, 0xB1F6, // 5.02 mmol /L
          (GLUCOSE_LOCATION_NOT_AVAIL | GLUCOSE_TYPE_ARTERIAL_WHOLE), 0xA5A5 },
        //Meas 6
        {
        GLUCOSE_MEAS_STATE_ALL,
          GLUCOSE_MEAS_FLAG_ALL & ~GLUCOSE_MEAS_FLAG_UNITS,   //kg/L
          6, { 7, 15, 0, 5, 2, 2011 }, 60, 0xB07E, // 126 mg/dl
          (GLUCOSE_LOCATION_FINGER | GLUCOSE_TYPE_ARTERIAL_PLASMA), 0x5A5A },
        // Time offset -2 hrs
        //Meas 7
        {
        GLUCOSE_MEAS_STATE_ALL,
          GLUCOSE_MEAS_FLAG_ALL & ~GLUCOSE_MEAS_FLAG_UNITS,   //kg/L
          7, { 9, 4, 0, 14, 4, 2011 }, -120, 0xB064,    // 100 mg/dl
          (GLUCOSE_LOCATION_FINGER | GLUCOSE_TYPE_UNDETER_WHOLE), 0x55AA },
        //Meas 8
        {
        GLUCOSE_MEAS_STATE_ALL,
          GLUCOSE_MEAS_FLAG_ALL & ~GLUCOSE_MEAS_FLAG_UNITS,    //kg/L
          8, { 0, 0, 0, 1, 1, 2013 }, -120, 0xB05A,  // 90 mg/dl
          (GLUCOSE_LOCATION_AST | GLUCOSE_TYPE_UNDETER_PLASMA), 0xAA55 },
        //Meas 9
        {
        GLUCOSE_MEAS_STATE_ALL,
          GLUCOSE_MEAS_FLAG_ALL & ~GLUCOSE_MEAS_FLAG_UNITS,    //kg/L
          9, { 12, 0, 0, 5, 2, 2013 }, -120, 0xB048, // 72 mg/dl
          (GLUCOSE_LOCATION_EARLOBE | GLUCOSE_TYPE_ISF), 0x1111 }, };

// Each measurement entry must have a corresponding context, it is only sent
// based on the flag in the measurement, but it must exist for this app.
static glucoseContext_t glucoseContextArray[] = {
//Context 1
        {
        GLUCOSE_CONTEXT_FLAG_ALL & ~GLUCOSE_CONTEXT_FLAG_MEDICATION_UNITS, //kg
          1, 0,
          GLUCOSE_CARBO_BREAKFAST,
          9,
          GLUCOSE_MEAL_PREPRANDIAL,
          GLUCOSE_HEALTH_NONE | GLUCOSE_TESTER_SELF, 1800, 70,
          GLUCOSE_MEDICATION_RAPID,
          100, 50 },
        //Context 2
        {
        GLUCOSE_CONTEXT_FLAG_ALL & ~GLUCOSE_CONTEXT_FLAG_MEDICATION_UNITS, //kg
          2, 0,
          GLUCOSE_CARBO_BREAKFAST,
          10,
          GLUCOSE_MEAL_PREPRANDIAL,
          GLUCOSE_HEALTH_NONE | GLUCOSE_TESTER_SELF, 1800, 70,
          GLUCOSE_MEDICATION_RAPID,
          100, 49 },
        //Context 3
        {
        GLUCOSE_CONTEXT_FLAG_ALL, //L
          3, 0,
          GLUCOSE_CARBO_BREAKFAST,
          10,
          GLUCOSE_MEAL_PREPRANDIAL,
          GLUCOSE_HEALTH_NONE | GLUCOSE_TESTER_SELF, 1800, 70,
          GLUCOSE_MEDICATION_RAPID,
          100, 49 },
        //Context 4
        {
        GLUCOSE_CONTEXT_FLAG_ALL & ~GLUCOSE_CONTEXT_FLAG_MEDICATION_UNITS, //kg
          4, 0,
          GLUCOSE_CARBO_BREAKFAST,
          10,
          GLUCOSE_MEAL_PREPRANDIAL,
          GLUCOSE_HEALTH_NONE | GLUCOSE_TESTER_SELF, 1800, 70,
          GLUCOSE_MEDICATION_RAPID,
          100, 49 },
        //Context 5
        {
        GLUCOSE_CONTEXT_FLAG_ALL & ~GLUCOSE_CONTEXT_FLAG_MEDICATION_UNITS, //kg
          5, 0,
          GLUCOSE_CARBO_BREAKFAST,
          10,
          GLUCOSE_MEAL_PREPRANDIAL,
          GLUCOSE_HEALTH_NONE | GLUCOSE_TESTER_SELF, 1800, 70,
          GLUCOSE_MEDICATION_RAPID,
          100, 49 },
        //Context 6
        {
        GLUCOSE_CONTEXT_FLAG_ALL, //L
          6, 0,
          GLUCOSE_CARBO_BREAKFAST,
          10,
          GLUCOSE_MEAL_PREPRANDIAL,
          GLUCOSE_HEALTH_NONE | GLUCOSE_TESTER_SELF, 1800, 70,
          GLUCOSE_MEDICATION_RAPID,
          100, 49 },
        //Context 7
        {
        GLUCOSE_CONTEXT_FLAG_ALL & ~GLUCOSE_CONTEXT_FLAG_MEDICATION_UNITS, //kg
          7, 0,
          GLUCOSE_CARBO_BREAKFAST,
          10,
          GLUCOSE_MEAL_PREPRANDIAL,
          GLUCOSE_HEALTH_NONE | GLUCOSE_TESTER_SELF, 1800, 70,
          GLUCOSE_MEDICATION_RAPID,
          100, 49 },
        //Context 8
        {
        GLUCOSE_CONTEXT_FLAG_ALL & ~GLUCOSE_CONTEXT_FLAG_MEDICATION_UNITS, //kg
          8, 0,
          GLUCOSE_CARBO_BREAKFAST,
          11,
          GLUCOSE_MEAL_PREPRANDIAL,
          GLUCOSE_HEALTH_NONE | GLUCOSE_TESTER_SELF, 1800, 70,
          GLUCOSE_MEDICATION_RAPID,
          100, 48 },
        //Context 9
        {
        GLUCOSE_CONTEXT_FLAG_ALL, //L
          9, 0,
          GLUCOSE_CARBO_BREAKFAST,
          12,
          GLUCOSE_MEAL_PREPRANDIAL,
          GLUCOSE_HEALTH_NONE | GLUCOSE_TESTER_SELF, 1800, 70,
          GLUCOSE_MEDICATION_RAPID,
          100, 47 } };

// GAP connected device address.
static uint8_t connDeviceAddr[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// initial index of measurement/context.
static uint8_t glucoseMeasIdx = 0;

static uint16_t seqNum = MEAS_IDX_MAX;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
// Application task and event processing.
static void GlucoseSensor_init(void);
static void GlucoseSensor_taskFxn(UArg a0, UArg a1);
static uint8_t GlucoseSensor_enqueueMsg(uint16_t event, uint8_t state,
                                        uint8_t *pData);
static void GlucoseSensor_processParamUpdate(uint16_t connHandle);
static void GlucoseSensor_processGapMessage(gapEventHdr_t *pMsg);
static void GlucoseSensor_clockHandler(UArg arg);
static void GlucoseSensor_processStackMsg(ICall_Hdr *pMsg);
static void GlucoseSensor_initPHYRSSIArray(void);
static void GlucoseSensor_processGattMsg(gattMsgEvent_t *pMsg);
static void GlucoseSensor_processAppMsg(glucoseEvt_t *pMsg);
static bool GlucoseSensor_toggleAdvertising(void);
static uint8_t GlucoseSensor_addConn(uint16_t connHandle);
static void GlucoseSensor_contextSend(void);
static uint8_t GlucoseSensor_clearConnListEntry(uint16_t connHandle);
static status_t GlucoseSensor_stopAutoPhyChange(uint16_t connHandle);

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
static void GlucoseSensor_updateRPA(void);
#endif // PRIVACY_1_2_CFG

static void GlucoseSensor_ctlPntHandleOpcode(uint8_t opcode, uint8_t oper,
                                             uint8_t filterType, void *param1,
                                             void *param2);
static void GlucoseSensor_ctlPntRsp(uint8_t rspCode, uint8_t opcode);
static void GlucoseSensor_ctlPntNumRecRsp(uint16_t numRecords);
static void GlucoseSensor_ctlPntIndicate(uint8_t opcode, uint8_t oper,
                                         uint8_t value0, uint8_t value1);
static uint8_t GlucoseSensor_filterRecords(uint8_t oper, uint8_t filterType,
                                           void *param1, void *param2, bool set,
                                           uint8_t mask, uint8_t opcode);
static uint8_t GlucoseSensor_findNumValidRecords(void);
static uint8_t GlucoseSensor_findFirstValidFilteredIdx(uint8_t startIndex);
static uint8_t GlucoseSensor_markFirstValidRec(bool setBits, uint8_t mask);
static uint8_t GlucoseSensor_markLastValidRec(bool setBits, uint8_t mask);
static uint8_t GlucoseSensor_markAllRecords(bool setBits, uint8_t mask);
static uint8_t GlucoseSensor_markAllRecordsEarlierThanSeqNum(uint16_t *pSeqNum,
                                                             bool set,
                                                             uint8_t mask);
static uint8_t GlucoseSensor_markAllRecordsLaterThanSeqNum(uint16_t *pSeqNum,
                                                           bool set,
                                                           uint8_t mask);
static uint8_t GlucoseSensor_markAllRecordsInRangeSeqNum(uint16_t *pSeqNum1,
                                                         uint16_t *pSeqNum2,
                                                         bool set,
                                                         uint8_t mask);

static uint8_t GlucoseSensor_removeConn(uint16_t connHandle);
static uint8_t GlucoseSensor_getConnIndex(uint16_t connHandle);
static uint8_t GlucoseSensor_markAllRecordsEarlierThanTime(UTCTimeStruct *pTime,
                                                           bool set,
                                                           uint8_t mask);
static uint8_t GlucoseSensor_markAllRecordsLaterThanTime(UTCTimeStruct *pTime,
                                                         bool set,
                                                         uint8_t mask);
static uint8_t GlucoseSensor_markAllRecordsInRangeTime(UTCTimeStruct *pTime1,
                                                       UTCTimeStruct *pTime2,
                                                       bool set, uint8_t mask);
static void GlucoseSensor_measSend(void);
static void GlucoseSensor_processCtlPntMsg(glucoseCtlPntMsg_t *pMsg);
static void GlucoseSensor_sendNext(void);
static uint8_t GlucoseSensor_verifyTime(UTCTimeStruct *pTime);
static void GlucoseSensor_processConnEvt(Gap_ConnEventRpt_t *pReport);

// Glucose service.
static void GlucoseSensor_serviceCB(uint8_t event, uint8_t *pValue,
                                    uint8_t len);
static void GlucoseSensor_processServiceEvt(uint8_t event, uint8_t *pValue,
                                            uint8_t len);

// Passcode.
static void GlucoseSensor_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs);
static void GlucoseSensor_processPasscodeEvt(uint16_t connHandle);

// Pairing state.
static void GlucoseSensor_pairStateCB(uint16_t connHandle, uint8_t state,
                                      uint8_t status);
static void GlucoseSensor_processPairStateEvt(uint8_t state, uint8_t status);

// Keys.
static void GlucoseSensor_keyPressHandler(uint8_t keys);
static void GlucoseSensor_handleKeys(uint8_t shift, uint8_t keys);

static void GlucoseSensor_advCallback(uint32_t event, void *pBuf,
                                      uintptr_t arg);

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = { 0 };
#endif // PRIVACY_1_2_CFG

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks.
//static gapRolesCBs_t glucose_PeripheralCBs = { GlucoseSensor_stateChangeCB // Profile State Change Callbacks
//        };
// Bond Manager Callbacks.
static const gapBondCBs_t glucoseBondCB = {
        (pfnPasscodeCB_t) GlucoseSensor_passcodeCB, GlucoseSensor_pairStateCB };

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      glucose_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void glucose_spin(void)
{
    volatile uint8_t x = 0;

    while (1)
    {
        x++;
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_createTask
 *
 * @brief   Task creation function for the Glucose Sensor application.
 *
 * @param   none
 *
 * @return  none
 */
void GlucoseSensor_createTask(void)
{
    Task_Params taskParams;

    // Configure task.
    Task_Params_init(&taskParams);
    taskParams.stack = glucoseTaskStack;
    taskParams.stackSize = GLUCOSE_TASK_STACK_SIZE;
    taskParams.priority = GLUCOSE_TASK_PRIORITY;

    Task_construct(&glucoseTask, GlucoseSensor_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      Glucose_init
 *
 * @brief   Initialization function for the Glucose App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
void GlucoseSensor_init(void)
{
    // ******************************************************************
    // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);

    // Hard code the DB Address till CC2650 board gets its own IEEE address.
    //uint8 bdAddress[B_ADDR_LEN] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x5A };
    //HCI_EXT_SetBDADDRCmd(bdAddress);

    // Set device's Sleep Clock Accuracy
    //HCI_EXT_SetSCACmd(40);

    // Create an RTOS queue for message from profile to be sent to app.
    appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

    // Create one-shot clocks for internal periodic events.
    Util_constructClock(&notiTimeoutClock, GlucoseSensor_clockHandler,
    DEFAULT_NOTI_PERIOD,
                        0, false, GLUCOSE_NOTI_TIMEOUT_EVT);

    Board_initKeys(GlucoseSensor_keyPressHandler);

    // Initialize Connection List
    GlucoseSensor_clearConnListEntry(CONNHANDLE_ALL);

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
        uint8_t pairMode = DEFAULT_PAIRING_MODE;
        uint8_t mitm = DEFAULT_MITM_MODE;
        uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
        uint8_t bonding = DEFAULT_BONDING_MODE;

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
    Glucose_AddService(GATT_ALL_SERVICES);
    DevInfo_AddService();

    // Register for Glucose service callback.
    Glucose_Register(GlucoseSensor_serviceCB);

    // Start the Bond Manager.
    GAPBondMgr_Register((gapBondCBs_t* )&glucoseBondCB);

    // Register with GAP for HCI/Host messages
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

#if !defined (USE_LL_CONN_PARAM_UPDATE)
    // Get the currently set local supported LE features
    // The HCI will generate an HCI event that will get received in the main
    // loop
    HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

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
    GlucoseSensor_clearConnListEntry(CONNHANDLE_ALL);

    //Initialize GAP layer for Peripheral role and register to receive GAP events
    GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, NULL);

    // Initialize array to store connection handle and RSSI values
    GlucoseSensor_initPHYRSSIArray();
}

/*********************************************************************
 * @fn      GlucoseSensor_initPHYRSSIArray
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
static void GlucoseSensor_initPHYRSSIArray(void)
{
    //Initialize array to store connection handle and RSSI values
    memset(connList, 0, sizeof(connList));
    for (uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++)
    {
        connList[index].connHandle = GLUCOSE_INVALID_HANDLE;
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. CONNHANDLE_ALL will always succeed.
 */
static uint8_t GlucoseSensor_clearConnListEntry(uint16_t connHandle)
{
    uint8_t i;
    // Set to invalid connection index initially
    uint8_t connIndex = MAX_NUM_BLE_CONNS;

    if (connHandle != CONNHANDLE_ALL)
    {
        // Get connection index from handle
        connIndex = GlucoseSensor_getConnIndex(connHandle);
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
            memset(connList[i].rssiArr, 0, GLUCOSE_MAX_RSSI_STORE_DEPTH);
            connList[i].rssiAvg = 0;
            connList[i].rssiCntr = 0;
            connList[i].isAutoPHYEnable = FALSE;
        }
    }

    return (SUCCESS);
}

/*********************************************************************
 * @fn      GlucoseSensor_taskFxn
 *
 * @brief   Glucose Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   not used.
 *
 * @return  events not processed
 */
void GlucoseSensor_taskFxn(UArg a0, UArg a1)
{
    // Initialize the application.
    GlucoseSensor_init();

    // Application main loop.
    for (;;)
    {
        uint32_t events;

        events = Event_pend(syncEvent, Event_Id_NONE, GLUCOSE_ALL_EVENTS,
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
                    GlucoseSensor_processStackMsg((ICall_Hdr*) pMsg);
                }

                if (pMsg)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // If RTOS queue is not empty, process app message.
            if (events & GLUCOSE_QUEUE_EVT)
            {
                while (!Queue_empty(appMsgQueueHandle))
                {
                    glucoseEvt_t *pMsg = (glucoseEvt_t*) Util_dequeueMsg(
                            appMsgQueueHandle);
                    if (pMsg)
                    {
                        // Process message.
                        GlucoseSensor_processAppMsg(pMsg);

                        // Free the space from the message.
                        ICall_free(pMsg);
                    }
                }
            }

            // Notification timeout.
            if (events & GLUCOSE_NOTI_TIMEOUT_EVT)
            {
                // Send next notification.
                GlucoseSensor_sendNext();
            }
        }
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void GlucoseSensor_processStackMsg(ICall_Hdr *pMsg)
{
    switch (pMsg->event)
    {

    case GAP_MSG_EVENT:
        GlucoseSensor_processGapMessage((gapEventHdr_t*) pMsg);
        break;

    case GATT_MSG_EVENT:
        GlucoseSensor_processGattMsg((gattMsgEvent_t*) pMsg);
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
 * @fn      GlucoseSensor_processGattMsg
 *
 * @brief   Process GATT messages.
 *
 * @param   pMsg - pointer the the GATT message.
 *
 * @return  none
 */
static void GlucoseSensor_processGattMsg(gattMsgEvent_t *pMsg)
{
    GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      GlucoseSensor_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t GlucoseSensor_getConnIndex(uint16_t connHandle)
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
 * @fn      GlucoseSensor_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void GlucoseSensor_processAppMsg(glucoseEvt_t *pMsg)
{
    switch (pMsg->hdr.event)
    {

    case GLUCOSE_KEY_CHANGE_EVT:
        GlucoseSensor_handleKeys(0, pMsg->hdr.state);
        break;

        // Glucose service event.
    case GLUCOSE_SERVICE_EVT:
    {
        glucoseServEvt_t *pEvt = (glucoseServEvt_t*) pMsg->pData;

        GlucoseSensor_processServiceEvt(pMsg->hdr.state, pEvt->pVal, pEvt->len);

        ICall_free(pEvt);
    }
        break;

        // Pairing event.
    case GLUCOSE_PAIRING_EVT:
        GlucoseSensor_processPairStateEvt(pMsg->hdr.state, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;

        // Passcode event.
    case GLUCOSE_PASSCODE_EVT:
        GlucoseSensor_processPasscodeEvt(*(uint16_t*) pMsg->pData);

        ICall_free(pMsg->pData);
        break;

    case GLUCOSE_CONN_EVT:
    {
        GlucoseSensor_processConnEvt((Gap_ConnEventRpt_t*) (pMsg->pData));

        ICall_free(pMsg->pData);
        break;
    }
    default:
        // Do nothing.
        break;
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void GlucoseSensor_processGapMessage(gapEventHdr_t *pMsg)
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
            status = GapAdv_create(&GlucoseSensor_advCallback, &advParamLegacy,
                                   &advHandleLegacy);
            GLUCOSE_ASSERT(status == SUCCESS);

            // Load advertising data for set #1 that is statically allocated by the app
            status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                         sizeof(advertData), advertData);
            GLUCOSE_ASSERT(status == SUCCESS);

            // Load scan response data for set #1 that is statically allocated by the app
            status = GapAdv_loadByHandle(advHandleLegacy,
                                         GAP_ADV_DATA_TYPE_SCAN_RSP,
                                         sizeof(scanRspData), scanRspData);
            GLUCOSE_ASSERT(status == SUCCESS);

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
            status = GapAdv_create(&GlucoseSensor_advCallback,
                                   &advParamLongRange, &advHandleLongRange);
            GLUCOSE_ASSERT(status == SUCCESS);

            // Load advertising data for set #2 that is statically allocated by the app
            status = GapAdv_loadByHandle(advHandleLongRange,
                                         GAP_ADV_DATA_TYPE_ADV,
                                         sizeof(advertData), advertData);
            GLUCOSE_ASSERT(status == SUCCESS);

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
                GlucoseSensor_updateRPA();

                // Create one-shot clock for RPA check event.
                Util_constructClock(&clkRpaRead, GlucoseSensor_clockHandler,
                GLUCOSE_READ_RPA_EVT_PERIOD,
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
            GlucoseSensor_addConn(pPkt->connectionHandle);

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
        GlucoseSensor_removeConn(pPkt->connectionHandle);

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
        glucoseConnHandleEntry_t *connHandleEntry =
                (glucoseConnHandleEntry_t*) List_get(&paramUpdateList);
        if (connHandleEntry != NULL)
        {
            // Attempt to send queued update now
            GlucoseSensor_processParamUpdate(connHandleEntry->connHandle);

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
 * @fn      GlucoseSensor_processParamUpdate
 *
 * @brief   Process a parameters update request
 *
 * @return  None
 */
static void GlucoseSensor_processParamUpdate(uint16_t connHandle)
{
    gapUpdateLinkParamReq_t req;
    uint8_t connIndex;

    req.connectionHandle = connHandle;
    req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

    connIndex = GlucoseSensor_getConnIndex(connHandle);
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
        glucoseConnHandleEntry_t *connHandleEntry = ICall_malloc(
                sizeof(glucoseConnHandleEntry_t));
        if (connHandleEntry)
        {
            connHandleEntry->connHandle = connHandle;

            List_put(&paramUpdateList, (List_Elem*) connHandleEntry);
        }
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_keyPressHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys - keys detected.
 *
 * @return  none
 */
void GlucoseSensor_keyPressHandler(uint8_t keys)
{
    // Enqueue the event.
    GlucoseSensor_enqueueMsg(GLUCOSE_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      GlucoseSensor_clearPendingParamUpdate
 *
 * @brief   clean pending param update request in the paramUpdateList list
 *
 * @param   connHandle - connection handle to clean
 *
 * @return  none
 */
void GlucoseSensor_clearPendingParamUpdate(uint16_t connHandle)
{
    List_Elem *curr;

    for (curr = List_head(&paramUpdateList); curr != NULL;
            curr = List_next(curr))
    {
        if (((glucoseConnHandleEntry_t*) curr)->connHandle == connHandle)
        {
            List_remove(&paramUpdateList, curr);
        }
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t GlucoseSensor_removeConn(uint16_t connHandle)
{
    uint8_t connIndex = GlucoseSensor_getConnIndex(connHandle);

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
        GlucoseSensor_clearPendingParamUpdate(connHandle);
        // Stop Auto PHY Change
        GlucoseSensor_stopAutoPhyChange(connHandle);
        // Clear Connection List Entry
        GlucoseSensor_clearConnListEntry(connHandle);
    }

    return connIndex;
}

/*********************************************************************
 * @fn      GlucoseSensor_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t GlucoseSensor_addConn(uint16_t connHandle)
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
                    sizeof(glucoseClockEventData_t) + sizeof(uint16_t));
            if (connList[i].pParamUpdateEventData)
            {
                connList[i].pParamUpdateEventData->event =
                GLUCOSE_SEND_PARAM_UPDATE_EVT;
                *((uint16_t*) connList[i].pParamUpdateEventData->data) =
                        connHandle;

                // Create a clock object and start
                connList[i].pUpdateClock = (Clock_Struct*) ICall_malloc(
                        sizeof(Clock_Struct));

                if (connList[i].pUpdateClock)
                {
                    Util_constructClock(
                            connList[i].pUpdateClock,
                            GlucoseSensor_clockHandler,
                            GLUCOSE_SEND_PARAM_UPDATE_DELAY,
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
 * @fn      GlucoseSensor_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_UP
 *                 KEY_RIGHT
 *
 * @return  none
 */
static void GlucoseSensor_handleKeys(uint8_t shift, uint8_t keys)
{
    if (keys & KEY_LEFT)
    {
        if (!glucoseSendAllRecords)
        {
            // Set simulated measurement flag index.
            if (GlucoseSensor_findNumValidRecords() > 0)
            {
                GlucoseSensor_markAllRecords(true,
                GLUCOSE_MEAS_STATE_FILTER_PASS);

                glucoseMeasIdx = GlucoseSensor_findFirstValidFilteredIdx(
                        glucoseMeasIdx + 1);
                if (glucoseMeasIdx == MEAS_IDX_MAX)
                {
                    glucoseMeasIdx = GlucoseSensor_findFirstValidFilteredIdx(0);
                }

                if (isConnected)
                {
                    GlucoseSensor_measSend();
                }
            }
            else
            {
                uint8_t i;
                // Populate dynamic measurement records.
                for (i = 0; i < DYNAMIC_REC_MAX; i++)
                {
                    glucoseMeasArray[i].state |= GLUCOSE_MEAS_STATE_VALID;
                    glucoseMeasArray[i].seqNum = ++seqNum;
                    glucoseContextArray[i].seqNum = seqNum;

                    // Set context info follows bit.
                    if (i % 2)
                    {
                        glucoseMeasArray[i].flags |=
                        GLUCOSE_MEAS_FLAG_CONTEXT_INFO;
                    }
                    else
                    {
                        glucoseMeasArray[i].flags &=
                                ~GLUCOSE_MEAS_FLAG_CONTEXT_INFO;
                    }
                }
            }
        }
    }

    if (keys & KEY_RIGHT)
    {

        // If not in a connection, toggle advertising on and off.
        if (isConnected == FALSE)
        {
            GlucoseSensor_toggleAdvertising();
        }
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_toggleAdvertising
 *
 * @brief   Toggle advertising state.
 *
 * @param   none
 *
 * @return  status - TRUE if advertising, FALSE otherwise.
 */
static bool GlucoseSensor_toggleAdvertising(void)
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
 * @fn      GlucoseSensor_sendNext
 *
 * @brief   Send next notification.
 *
 * @return  none
 */
static void GlucoseSensor_sendNext(void)
{
    if (glucoseSendContext)
    {
        glucoseSendContext = false;
        GlucoseSensor_contextSend();
    }
    else if (glucoseSendAllRecords)
    {
        glucoseMeasIdx = GlucoseSensor_findFirstValidFilteredIdx(
                glucoseMeasIdx + 1);

        if (glucoseMeasIdx == MEAS_IDX_MAX)
        {
            glucoseMeasIdx = 0;
            glucoseSendAllRecords = false;
            GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_SUCCESS, CTL_PNT_OP_REQ);
        }
        else
        {
            GlucoseSensor_measSend();
        }
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_processCtlPntMsg
 *
 * @brief   Process Control Point messages.
 *
 * @return  none
 */
static void GlucoseSensor_processCtlPntMsg(glucoseCtlPntMsg_t *pMsg)
{
    uint8_t opcode = pMsg->data[0];
    uint8_t oper = pMsg->data[1];
    UTCTimeStruct time1, time2;
    bool opcodeValid = true;
    uint16_t seqNum1, seqNum2;

    switch (opcode)
    {
    case CTL_PNT_OP_REQ:
    case CTL_PNT_OP_CLR:
    case CTL_PNT_OP_GET_NUM:
        if (oper == CTL_PNT_OPER_NULL)
        {
            GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_OPER_INVALID, opcode);
            opcodeValid = false;
        }
        break;

    case CTL_PNT_OP_ABORT:
        if (oper != CTL_PNT_OPER_NULL)
        {
            GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_OPER_INVALID, opcode);
            opcodeValid = false;
        }
        break;

    default:
        GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_OPCODE_NOT_SUPPORTED, opcode);
        opcodeValid = false;
        break;
    }

    if (opcodeValid)
    {
        switch (oper)
        {
        case CTL_PNT_OPER_NULL:
        case CTL_PNT_OPER_ALL:
        case CTL_PNT_OPER_FIRST:
        case CTL_PNT_OPER_LAST:
            if (pMsg->len == 2)
            {
                GlucoseSensor_ctlPntHandleOpcode(opcode, oper, 0, NULL, NULL);
            }
            else
            {
                // No operand should exist, but msg is longer than 2 bytes.
                GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_OPERAND_INVALID, opcode);
            }
            break;

        case CTL_PNT_OPER_RANGE:
            // Check filter type.
            if (pMsg->data[2] == CTL_PNT_FILTER_SEQNUM && pMsg->len == 7)
            {
                seqNum1 = BUILD_UINT16(pMsg->data[3], pMsg->data[4]);
                seqNum2 = BUILD_UINT16(pMsg->data[5], pMsg->data[6]);

                if (seqNum1 <= seqNum2)
                {
                    GlucoseSensor_ctlPntHandleOpcode(opcode, oper,
                                                     pMsg->data[2], &seqNum1,
                                                     &seqNum2);
                }
                else
                {
                    GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_OPERAND_INVALID,
                                            opcode);
                }
            }
            else if (pMsg->data[2] == CTL_PNT_FILTER_TIME && pMsg->len == 17)
            {
                time1.year = BUILD_UINT16(pMsg->data[3], pMsg->data[4]);
                time1.month = pMsg->data[5];
                time1.day = pMsg->data[6];
                time1.hour = pMsg->data[7];
                time1.minutes = pMsg->data[8];
                time1.seconds = pMsg->data[9];

                time2.year = BUILD_UINT16(pMsg->data[10], pMsg->data[11]);
                time2.month = pMsg->data[12];
                time2.day = pMsg->data[13];
                time2.hour = pMsg->data[14];
                time2.minutes = pMsg->data[15];
                time2.seconds = pMsg->data[16];

                GlucoseSensor_ctlPntHandleOpcode(opcode, oper, pMsg->data[2],
                                                 &time1, &time2);
            }
            else
            {
                GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_OPERAND_INVALID, opcode);
            }
            break;

        case CTL_PNT_OPER_LESS_EQUAL:
        case CTL_PNT_OPER_GREATER_EQUAL:
            // Check filter type.
            if (pMsg->data[2] == CTL_PNT_FILTER_SEQNUM && pMsg->len == 5)
            {
                seqNum1 = BUILD_UINT16(pMsg->data[3], pMsg->data[4]);

                GlucoseSensor_ctlPntHandleOpcode(opcode, oper, pMsg->data[2],
                                                 &seqNum1, NULL);
            }
            else if (pMsg->data[2] == CTL_PNT_FILTER_TIME && pMsg->len == 10)
            {
                time1.year = BUILD_UINT16(pMsg->data[3], pMsg->data[4]);
                time1.month = pMsg->data[5];
                time1.day = pMsg->data[6];
                time1.hour = pMsg->data[7];
                time1.minutes = pMsg->data[8];
                time1.seconds = pMsg->data[9];

                GlucoseSensor_ctlPntHandleOpcode(opcode, oper, pMsg->data[2],
                                                 &time1, NULL);
            }
            else
            {
                GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_FILTER_NOT_SUPPORTED,
                                        opcode);
            }
            break;

        default:
            GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_OPER_NOT_SUPPORTED, opcode);
            break;
        }
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void GlucoseSensor_pairStateCB(uint16_t connHandle, uint8_t state,
                                      uint8_t status)
{
    uint8_t *pData;

    if ((pData = ICall_malloc(sizeof(uint8_t))))
    {
        *pData = status;

        GlucoseSensor_enqueueMsg(GLUCOSE_PAIRING_EVT, state, pData);
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_processPairStateEvt
 *
 * @brief   Pairing state callback event processor.
 *
 * @return  none
 */
static void GlucoseSensor_processPairStateEvt(uint8_t state, uint8_t status)
{
    if (state == GAPBOND_PAIRING_STATE_COMPLETE)
    {
        if (status == SUCCESS)
        {
            // Store the address of the bonded address.
            memcpy(glucoseBondedAddr, connDeviceAddr, B_ADDR_LEN);
        }
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void GlucoseSensor_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs)
{
    uint16_t *pData;

    if ((pData = ICall_malloc(sizeof(uint16_t))))
    {
        *pData = connHandle;

        GlucoseSensor_enqueueMsg(GLUCOSE_PASSCODE_EVT, 0, (uint8_t*) pData);
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void GlucoseSensor_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
    glucoseGapAdvEventData_t *pData = ICall_malloc(
            sizeof(glucoseGapAdvEventData_t));

    if (pData)
    {
        pData->event = event;
        pData->pBuf = pBuf;

        if (GlucoseSensor_enqueueMsg(GLUCOSE_ADV_EVT, NULL,
                                     (uint8_t*) (pData)) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_processPasscodeEvt.
 *
 * @brief   Passcode callback event processor.
 *
 * @return  none
 */
static void GlucoseSensor_processPasscodeEvt(uint16_t connHandle)
{
    // This app uses a default passcode. A real-life scenario would handle all
    // pairing scenarios and likely generate this randomly.
    GAPBondMgr_PasscodeRsp(connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      GlucoseSensor_measSend
 *
 * @brief   Prepare and send a glucose measurement
 *
 * @return  none
 */
static void GlucoseSensor_measSend(void)
{
    if (glucoseMeasArray[glucoseMeasIdx].state == GLUCOSE_MEAS_STATE_ALL)
    {
        attHandleValueNoti_t glucoseMeas;

        glucoseMeas.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_NOTI,
        GLUCOSE_MEAS_LEN,
                                           NULL);
        if (glucoseMeas.pValue != NULL)
        {
            // ATT value notification structure.
            uint8_t *p = glucoseMeas.pValue;

            uint8_t flags = glucoseMeasArray[glucoseMeasIdx].flags;

            // Flags 1 byte long.
            *p++ = flags;

            // Sequence number.
            *p++ = LO_UINT16(glucoseMeasArray[glucoseMeasIdx].seqNum);
            *p++ = HI_UINT16(glucoseMeasArray[glucoseMeasIdx].seqNum);

            // Base time; convert day and month from utc time to characteristic format.
            *p++ = LO_UINT16(glucoseMeasArray[glucoseMeasIdx].baseTime.year);
            *p++ = HI_UINT16(glucoseMeasArray[glucoseMeasIdx].baseTime.year);
            *p++ = (glucoseMeasArray[glucoseMeasIdx].baseTime.month + 1);
            *p++ = (glucoseMeasArray[glucoseMeasIdx].baseTime.day + 1);
            *p++ = glucoseMeasArray[glucoseMeasIdx].baseTime.hour;
            *p++ = glucoseMeasArray[glucoseMeasIdx].baseTime.minutes;
            *p++ = glucoseMeasArray[glucoseMeasIdx].baseTime.seconds;

            // Time offset.
            if (flags & GLUCOSE_MEAS_FLAG_TIME_OFFSET)
            {
                *p++ = LO_UINT16(glucoseMeasArray[glucoseMeasIdx].timeOffset);
                *p++ = HI_UINT16(glucoseMeasArray[glucoseMeasIdx].timeOffset);
            }

            // Concentration.
            if (flags & GLUCOSE_MEAS_FLAG_CONCENTRATION)
            {
                *p++ = LO_UINT16(
                        glucoseMeasArray[glucoseMeasIdx].concentration);
                *p++ = HI_UINT16(
                        glucoseMeasArray[glucoseMeasIdx].concentration);
                *p++ = glucoseMeasArray[glucoseMeasIdx].typeSampleLocation;
            }

            if (flags & GLUCOSE_MEAS_FLAG_STATUS_ANNUNCIATION)
            {
                *p++ = LO_UINT16(glucoseMeasArray[glucoseMeasIdx].sensorStatus);
                *p++ = HI_UINT16(glucoseMeasArray[glucoseMeasIdx].sensorStatus);
            }

            glucoseMeas.len = (uint8_t) (p - glucoseMeas.pValue);

            if (flags & GLUCOSE_MEAS_FLAG_CONTEXT_INFO)
            {
                glucoseSendContext = true;
            }

            // Send Measurement.
            if (Glucose_MeasSend(gapConnHandle, &glucoseMeas,
                                 selfEntity) != SUCCESS)
            {
                GATT_bm_free((gattMsg_t*) &glucoseMeas, ATT_HANDLE_VALUE_NOTI);
            }

            // Start notification timer.
            Util_startClock(&notiTimeoutClock);
        }
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_contextSend
 *
 * @brief   Prepare and send a glucose measurement context.
 *
 * @return  none
 */
static void GlucoseSensor_contextSend(void)
{
    if (glucoseMeasArray[glucoseMeasIdx].state == GLUCOSE_MEAS_STATE_ALL)
    {
        attHandleValueNoti_t glucoseContext;

        glucoseContext.pValue = GATT_bm_alloc(gapConnHandle,
        ATT_HANDLE_VALUE_NOTI,
                                              GLUCOSE_CONTEXT_LEN,
                                              NULL);
        if (glucoseContext.pValue != NULL)
        {
            // ATT value notification structure.
            uint8_t *p = glucoseContext.pValue;

            uint8_t flags = glucoseContextArray[glucoseMeasIdx].flags;

            // Flags 1 byte long.
            *p++ = flags;

            // Sequence number.
            *p++ = LO_UINT16(glucoseContextArray[glucoseMeasIdx].seqNum);
            *p++ = HI_UINT16(glucoseContextArray[glucoseMeasIdx].seqNum);

            if (flags & GLUCOSE_CONTEXT_FLAG_EXTENDED)
            {
                *p++ = glucoseContextArray[glucoseMeasIdx].extendedFlags;
            }

            if (flags & GLUCOSE_CONTEXT_FLAG_CARBO)
            {
                *p++ = glucoseContextArray[glucoseMeasIdx].carboId;
                *p++ = LO_UINT16(glucoseContextArray[glucoseMeasIdx].carboVal);
                *p++ = HI_UINT16(glucoseContextArray[glucoseMeasIdx].carboVal);
            }

            if (flags & GLUCOSE_CONTEXT_FLAG_MEAL)
            {
                *p++ = glucoseContextArray[glucoseMeasIdx].mealVal;
            }

            if (flags & GLUCOSE_CONTEXT_FLAG_TESTER_HEALTH)
            {
                *p++ = glucoseContextArray[glucoseMeasIdx].TesterHealthVal;
            }

            if (flags & GLUCOSE_CONTEXT_FLAG_EXERCISE)
            {
                *p++ = LO_UINT16(
                        glucoseContextArray[glucoseMeasIdx].exerciseDuration);
                *p++ = HI_UINT16(
                        glucoseContextArray[glucoseMeasIdx].exerciseDuration);
                *p++ = glucoseContextArray[glucoseMeasIdx].exerciseIntensity;
            }

            if (flags & GLUCOSE_CONTEXT_FLAG_MEDICATION)
            {
                *p++ = glucoseContextArray[glucoseMeasIdx].medId;
                *p++ = LO_UINT16(glucoseContextArray[glucoseMeasIdx].medVal);
                *p++ = HI_UINT16(glucoseContextArray[glucoseMeasIdx].medVal);
            }

            if (flags & GLUCOSE_CONTEXT_FLAG_HbA1c)
            {
                *p++ = LO_UINT16(glucoseContextArray[glucoseMeasIdx].HbA1cVal);
                *p++ = HI_UINT16(glucoseContextArray[glucoseMeasIdx].HbA1cVal);
            }

            glucoseContext.len = (uint8_t) (p - glucoseContext.pValue);

            // Send Measurement.
            if (Glucose_ContextSend(gapConnHandle, &glucoseContext,
                                    selfEntity) != SUCCESS)
            {
                GATT_bm_free((gattMsg_t*) &glucoseContext,
                ATT_HANDLE_VALUE_NOTI);
            }

            // Start notification timer.
            Util_startClock(&notiTimeoutClock);
        }
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_ctlPntRsp
 *
 * @brief   Send a record control point response.
 *
 * @param   rspCode - the status code of the operation
 * @param   opcode - control point opcode
 *
 * @return  none
 */
static void GlucoseSensor_ctlPntRsp(uint8_t rspCode, uint8_t opcode)
{
    GlucoseSensor_ctlPntIndicate(CTL_PNT_OP_REQ_RSP, CTL_PNT_OPER_NULL, opcode,
                                 rspCode);
}

/*********************************************************************
 * @fn      GlucoseSensor_ctlPntNumRecRsp
 *
 * @brief   Send a record control point num records response.
 *
 * @param   numRecords - number of records found
 * @param   oper - operator used to filter the record list
 *
 * @return  none
 */
static void GlucoseSensor_ctlPntNumRecRsp(uint16_t numRecords)
{
    GlucoseSensor_ctlPntIndicate(CTL_PNT_OP_NUM_RSP, CTL_PNT_OPER_NULL,
                                 LO_UINT16(numRecords), HI_UINT16(numRecords));
}

/*********************************************************************
 * @fn      GlucoseSensor_ctlPntIndicate
 *
 * @brief   Send an indication containing a control point message.
 *
 * @param   opcode - opcode
 * @param   oper - operator
 * @param   value0 - first value
 * @param   value1 - second value
 *
 * @return  none
 */
static void GlucoseSensor_ctlPntIndicate(uint8_t opcode, uint8_t oper,
                                         uint8_t value0, uint8_t value1)
{
    attHandleValueInd_t glucoseCtlPntRsp;

    glucoseCtlPntRsp.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_IND,
    GLUCOSE_CTL_PNT_LEN,
                                            NULL);
    if (glucoseCtlPntRsp.pValue != NULL)
    {
        glucoseCtlPntRsp.len = GLUCOSE_CTL_PNT_LEN;
        glucoseCtlPntRsp.pValue[0] = opcode;
        glucoseCtlPntRsp.pValue[1] = oper;
        glucoseCtlPntRsp.pValue[2] = value0;
        glucoseCtlPntRsp.pValue[3] = value1;

        // Send indication.
        if (Glucose_CtlPntIndicate(gapConnHandle, &glucoseCtlPntRsp,
                                   selfEntity) != SUCCESS)
        {
            GATT_bm_free((gattMsg_t*) &glucoseCtlPntRsp, ATT_HANDLE_VALUE_IND);
        }
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_serviceCB
 *
 * @brief   Callback function for glucose service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void GlucoseSensor_serviceCB(uint8_t event, uint8_t *valueP, uint8_t len)
{
    glucoseServEvt_t *pData;

    if ((pData = (glucoseServEvt_t*) ICall_malloc(sizeof(glucoseServEvt_t))))
    {
        if ((pData->pVal = (uint8_t*) ICall_malloc(len)))

            memcpy(pData->pVal, valueP, len);

        pData->len = len;

        GlucoseSensor_enqueueMsg(GLUCOSE_SERVICE_EVT, event, (uint8_t*) pData);
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_processServiceEvt
 *
 * @brief   Event processor for the glucose service callback function.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void GlucoseSensor_processServiceEvt(uint8_t event, uint8_t *valueP,
                                            uint8_t len)
{
    switch (event)
    {
    // Control point command.
    case GLUCOSE_CTL_PNT_CMD:
    {
        glucoseCtlPntMsg_t msg;

        msg.len = len;

        memcpy(msg.data, valueP, len);

        // Free valueP
        ICall_free(valueP);

        // Process the control point command.
        GlucoseSensor_processCtlPntMsg(&msg);
    }
        break;

    default:
        // Do nothing.
        break;
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_markAllRecords
 *
 * @brief   Set the valid/filtered flags in the measurement array.
 *
 * @param   setBits - if true bits are set, otherwise cleared
 * @param   mask - bits to set or clear
 *
 * @return  number of valid records set or cleared.
 */
static uint8_t GlucoseSensor_markAllRecords(bool setBits, uint8_t mask)
{
    uint8_t i;
    uint8_t count = 0;

    for (i = 0; i < MEAS_IDX_MAX; i++)
    {
        if (glucoseMeasArray[i].state & GLUCOSE_MEAS_STATE_VALID)
        {
            count++;
        }

        if (setBits)
        {
            glucoseMeasArray[i].state |= mask;
        }
        else
        {
            glucoseMeasArray[i].state &= ~mask;
        }
    }

    return count;
}

/*********************************************************************
 * @fn      GlucoseSensor_markFirstValidRec
 *
 * @brief   Set the state in the first valid record.
 *
 * @param   setBits - if true bits are set, otherwise cleared
 * @param   mask - bits to set or clear
 *
 * @return  number of valid records set or cleared.
 */
static uint8_t GlucoseSensor_markFirstValidRec(bool setBits, uint8_t mask)
{
    uint8_t i;

    for (i = 0; i < MEAS_IDX_MAX; i++)
    {
        if (glucoseMeasArray[i].state & GLUCOSE_MEAS_STATE_VALID)
        {
            if (setBits)
            {
                glucoseMeasArray[i].state |= mask;
            }
            else
            {
                glucoseMeasArray[i].state &= ~mask;
            }

            return 1;
        }
    }

    return 0;
}

/*********************************************************************
 * @fn      GlucoseSensor_markLastValidRec
 *
 * @brief   Set the state in the last valid record.
 *
 * @param   setBits - if true bits are set, otherwise cleared
 * @param   mask - bits to set or clear
 *
 * @return  number of valid records set or cleared.
 */
static uint8_t GlucoseSensor_markLastValidRec(bool setBits, uint8_t mask)
{
    uint8_t i;

    for (i = MEAS_IDX_MAX; i > 0; i--)
    {
        if (glucoseMeasArray[i - 1].state & GLUCOSE_MEAS_STATE_VALID)
        {
            if (setBits)
            {
                glucoseMeasArray[i - 1].state |= mask;
            }
            else
            {
                glucoseMeasArray[i - 1].state &= ~mask;
            }

            return 1;
        }
    }

    return 0;
}

/*********************************************************************
 * @fn      GlucoseSensor_verifyTime
 *
 * @brief   Verify time values are suitable for filtering.
 *
 * @param   pTime - UTC time struct
 *
 * @return  true if time is ok, false otherwise
 */
static uint8_t GlucoseSensor_verifyTime(UTCTimeStruct *pTime)
{
    // Sanity check year.
    if (pTime->year < 2000 || pTime->year > 2111)
    {
        return false;
    }
    // Check day range.
    if (pTime->day == 0 || pTime->day > 31)
    {
        return false;
    }
    // Check month range
    if (pTime->month == 0 || pTime->month > 12)
    {
        return false;
    }

    // Adjust month and day; utc time uses 0-11 and 0-30,
    // characteristic uses 1-12 and 1-31.
    pTime->day--;
    pTime->month--;

    return true;
}

/*********************************************************************
 * @fn      GlucoseSensor_markAllRecordsEarlierThanTime
 *
 * @brief   Mark all records earlier than a specific time.
 *
 * @param   pTime - time filter
 * @param   set - if true bits are set, otherwise cleared
 * @param   mask - bits to set or clear
 *
 * @return  number of valid records set or cleared.
 */
static uint8_t GlucoseSensor_markAllRecordsEarlierThanTime(UTCTimeStruct *pTime,
                                                           bool set,
                                                           uint8_t mask)
{
    uint8_t i;
    uint8_t count = 0;
    UTCTime testTime;

    if (GlucoseSensor_verifyTime(pTime) == false)
    {
        return 0;
    }

    testTime = UTC_convertUTCSecs(pTime);

    for (i = 0; i < MEAS_IDX_MAX; i++)
    {
        UTCTime recordTime = UTC_convertUTCSecs(&glucoseMeasArray[i].baseTime)
                + glucoseMeasArray[i].timeOffset;

        if (recordTime <= testTime)
        {
            if (glucoseMeasArray[i].state & GLUCOSE_MEAS_STATE_VALID)
            {
                count++;
            }

            if (set)
            {
                glucoseMeasArray[i].state |= mask;
            }
            else
            {
                glucoseMeasArray[i].state &= ~mask;
            }
        }
    }

    return count;
}

/*********************************************************************
 * @fn      GlucoseSensor_markAllRecordsLaterThanTime
 *
 * @brief   Mark all records later than a specific time.
 *
 * @param   pTime - time filter
 * @param   set - if true bits are set, otherwise cleared
 * @param   mask - bits to set or clear
 *
 * @return  number of valid records set or cleared.
 */
static uint8_t GlucoseSensor_markAllRecordsLaterThanTime(UTCTimeStruct *pTime,
                                                         bool set, uint8_t mask)
{
    uint8_t i;
    uint8_t count = 0;
    UTCTime testTime;

    if (GlucoseSensor_verifyTime(pTime) == false)
    {
        return 0;
    }

    testTime = UTC_convertUTCSecs(pTime);

    for (i = 0; i < MEAS_IDX_MAX; i++)
    {
        UTCTime recordTime = UTC_convertUTCSecs(&glucoseMeasArray[i].baseTime)
                + glucoseMeasArray[i].timeOffset;

        if (recordTime >= testTime)
        {
            if (glucoseMeasArray[i].state & GLUCOSE_MEAS_STATE_VALID)
            {
                count++;
            }

            if (set)
            {
                glucoseMeasArray[i].state |= mask;
            }
            else
            {
                glucoseMeasArray[i].state &= ~mask;
            }
        }
    }

    return count;
}

/*********************************************************************
 * @fn      GlucoseSensor_markAllRecordsInRangeTime
 *
 * @brief   Mark all records between two times.
 *
 * @param   pTime1 - time filter low end of range
 * @param   pTime2 - time filter high end of range
 * @param   set - if true bits are set, otherwise cleared
 * @param   mask - bits to set or clear
 *
 * @return  number of valid records set or cleared.
 */
static uint8_t GlucoseSensor_markAllRecordsInRangeTime(UTCTimeStruct *pTime1,
                                                       UTCTimeStruct *pTime2,
                                                       bool set, uint8_t mask)
{
    uint8_t i;
    uint8_t count = 0;
    UTCTime lowEnd, highEnd;

    if (GlucoseSensor_verifyTime(pTime1) == false)
    {
        return 0;
    }

    if (GlucoseSensor_verifyTime(pTime2) == false)
    {
        return 0;
    }

    lowEnd = UTC_convertUTCSecs(pTime1);
    highEnd = UTC_convertUTCSecs(pTime2);

    for (i = 0; i < MEAS_IDX_MAX; i++)
    {
        UTCTime recordTime = UTC_convertUTCSecs(&glucoseMeasArray[i].baseTime)
                + glucoseMeasArray[i].timeOffset;

        if ((recordTime >= lowEnd) && (recordTime <= highEnd))
        {
            if (glucoseMeasArray[i].state & GLUCOSE_MEAS_STATE_VALID)
            {
                count++;
            }

            if (set)
            {
                glucoseMeasArray[i].state |= mask;
            }
            else
            {
                glucoseMeasArray[i].state &= ~mask;
            }
        }
    }

    return count;
}

/*********************************************************************
 * @fn      GlucoseSensor_markAllRecordsEarlierThanSeqNum
 *
 * @brief   Mark all records earlier than a specific sequence number.
 *
 * @param   pSeqNum - filter
 * @param   set - if true bits are set, otherwise cleared
 * @param   mask - bits to set or clear
 *
 * @return  number of valid records set or cleared.
 */
static uint8_t GlucoseSensor_markAllRecordsEarlierThanSeqNum(uint16_t *pSeqNum,
                                                             bool set,
                                                             uint8_t mask)
{
    uint8_t i;
    uint8_t count = 0;

    for (i = 0; i < MEAS_IDX_MAX; i++)
    {
        if (glucoseMeasArray[i].seqNum <= *pSeqNum)
        {
            if (glucoseMeasArray[i].state & GLUCOSE_MEAS_STATE_VALID)
            {
                count++;
            }

            if (set)
            {
                glucoseMeasArray[i].state |= mask;
            }
            else
            {
                glucoseMeasArray[i].state &= ~mask;
            }
        }
    }

    return count;
}

/*********************************************************************
 * @fn      GlucoseSensor_markAllRecordsLaterThanSeqNum
 *
 * @brief   Mark all records later than a specific sequence number.
 *
 * @param   pSeqNum - filter
 * @param   set - if true bits are set, otherwise cleared
 * @param   mask - bits to set or clear
 *
 * @return  number of valid records set or cleared.
 */
static uint8_t GlucoseSensor_markAllRecordsLaterThanSeqNum(uint16_t *pSeqNum,
                                                           bool set,
                                                           uint8_t mask)
{
    uint8_t i;
    uint8_t count = 0;

    for (i = 0; i < MEAS_IDX_MAX; i++)
    {
        if (glucoseMeasArray[i].seqNum >= *pSeqNum)
        {
            if (glucoseMeasArray[i].state & GLUCOSE_MEAS_STATE_VALID)
            {
                count++;
            }

            if (set)
            {
                glucoseMeasArray[i].state |= mask;
            }
            else
            {
                glucoseMeasArray[i].state &= ~mask;
            }
        }
    }

    return count;
}

/*********************************************************************
 * @fn      GlucoseSensor_markAllRecordsInRangeSeqNum
 *
 * @brief   Mark all records between two sequence numbers.
 *
 * @param   pSeqNum1 - filter low end of range
 * @param   pSeqNum2 - filter high end of range
 * @param   set - if true bits are set, otherwise cleared
 * @param   mask - bits to set or clear
 *
 * @return  number of valid records set or cleared.
 */
static uint8_t GlucoseSensor_markAllRecordsInRangeSeqNum(uint16_t *pSeqNum1,
                                                         uint16_t *pSeqNum2,
                                                         bool set, uint8_t mask)
{
    uint8_t i;
    uint8_t count = 0;

    for (i = 0; i < MEAS_IDX_MAX; i++)
    {
        if ((glucoseMeasArray[i].seqNum >= *pSeqNum1)
                && (glucoseMeasArray[i].seqNum <= *pSeqNum2))
        {
            if (glucoseMeasArray[i].state & GLUCOSE_MEAS_STATE_VALID)
            {
                count++;
            }

            if (set)
            {
                glucoseMeasArray[i].state |= mask;
            }
            else
            {
                glucoseMeasArray[i].state &= ~mask;
            }
        }
    }

    return count;
}

/*********************************************************************
 * @fn      GlucoseSensor_filterRecords
 *
 * @brief   Call the correct filter function for a particular operator.
 *
 * @param   oper - control point operator
 * @param   filterType - control point filter type
 * @param   param1 - filter (if applicable), otherwise NULL
 * @param   param2 - filter (if applicable), otherwise NULL
 * @param   set - if true bits are set, otherwise cleared
 * @param   mask - bits to set or clear
 * @param   opcode - control point opcode
 *
 * @return  number of valid records within filter times
 */
static uint8_t GlucoseSensor_filterRecords(uint8_t oper, uint8_t filterType,
                                           void *param1, void *param2, bool set,
                                           uint8_t mask, uint8_t opcode)
{
    uint8_t numFiltered = 0;

    switch (oper)
    {
    case CTL_PNT_OPER_NULL:
        GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_OPER_NOT_SUPPORTED, opcode);
        break;

    case CTL_PNT_OPER_ALL:
        numFiltered = GlucoseSensor_markAllRecords(set, mask);
        break;

    case CTL_PNT_OPER_FIRST:
        numFiltered = GlucoseSensor_markFirstValidRec(set, mask);
        break;

    case CTL_PNT_OPER_LAST:
        numFiltered = GlucoseSensor_markLastValidRec(set, mask);
        break;

    case CTL_PNT_OPER_RANGE:
        if (filterType == CTL_PNT_FILTER_SEQNUM)
        {
            numFiltered = GlucoseSensor_markAllRecordsInRangeSeqNum(param1,
                                                                    param2, set,
                                                                    mask);
        }
        else
        {
            numFiltered = GlucoseSensor_markAllRecordsInRangeTime(param1,
                                                                  param2, set,
                                                                  mask);
        }
        break;

    case CTL_PNT_OPER_LESS_EQUAL:
        if (filterType == CTL_PNT_FILTER_SEQNUM)
        {
            numFiltered = GlucoseSensor_markAllRecordsEarlierThanSeqNum(param1,
                                                                        set,
                                                                        mask);
        }
        else
        {
            numFiltered = GlucoseSensor_markAllRecordsEarlierThanTime(param1,
                                                                      set,
                                                                      mask);
        }
        break;

    case CTL_PNT_OPER_GREATER_EQUAL:
        if (filterType == CTL_PNT_FILTER_SEQNUM)
        {
            numFiltered = GlucoseSensor_markAllRecordsLaterThanSeqNum(param1,
                                                                      set,
                                                                      mask);
        }
        else
        {
            numFiltered = GlucoseSensor_markAllRecordsLaterThanTime(param1, set,
                                                                    mask);
        }
        break;

    default:
        GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_OPER_INVALID, opcode);
        break;
    }

    return numFiltered;
}

/*********************************************************************
 * @fn      GlucoseSensor_findNumValidRecords
 *
 * @brief   Count number of valid records in the array.
 *
 * @return  number of valid records
 */
static uint8_t GlucoseSensor_findNumValidRecords(void)
{
    uint8_t i;
    uint8_t count = 0;

    for (i = 0; i < MEAS_IDX_MAX; i++)
    {
        if (glucoseMeasArray[i].state & GLUCOSE_MEAS_STATE_VALID)
        {
            count++;
        }
    }

    return count;
}

/*********************************************************************
 * @fn      GlucoseSensor_findFirstValidFilteredIdx
 *
 * @brief   Find the first valid entry, that also is in the last filter.
 *
 * @param   startIndex - starting index of search
 *
 * @return  index of next valid and filtered record or MEAS_IDX_MAX if none
 */
static uint8_t GlucoseSensor_findFirstValidFilteredIdx(uint8_t startIndex)
{
    uint8_t i;

    if (startIndex >= MEAS_IDX_MAX)
    {
        return MEAS_IDX_MAX;
    }

    for (i = startIndex; i < MEAS_IDX_MAX; i++)
    {
        if (glucoseMeasArray[i].state == GLUCOSE_MEAS_STATE_ALL)
        {
            break;
        }
    }

    return i;
}

/*********************************************************************
 * @fn      GlucoseSensor_ctlPntHandleOpcode
 *
 * @brief   Handle control point opcodes.
 *
 * @param   opcode - control point opcode
 * @param   oper - control point operator
 * @param   filterType - control point filter type
 * @param   param1 - filter (if applicable), otherwise NULL
 * @param   param2 - filter (if applicable), otherwise NULL
 *
 * @return  none
 */
static void GlucoseSensor_ctlPntHandleOpcode(uint8_t opcode, uint8_t oper,
                                             uint8_t filterType, void *param1,
                                             void *param2)
{
    switch (opcode)
    {
    case CTL_PNT_OP_REQ:
        //Clear all filter bits, before running the new test.
        GlucoseSensor_markAllRecords(false, GLUCOSE_MEAS_STATE_FILTER_PASS);

        if (GlucoseSensor_filterRecords(oper, filterType, param1, param2, true,
        GLUCOSE_MEAS_STATE_FILTER_PASS,
                                        opcode) > 0)
        {
            glucoseSendAllRecords = true;

            glucoseMeasIdx = GlucoseSensor_findFirstValidFilteredIdx(0);

            GlucoseSensor_measSend();
        }
        else
        {
            GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_NO_RECORDS, opcode);
        }

        break;

    case CTL_PNT_OP_CLR:
        if (GlucoseSensor_filterRecords(oper, filterType, param1, param2, false,
        GLUCOSE_MEAS_STATE_VALID,
                                        opcode) > 0)
        {
            GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_SUCCESS, opcode);
        }
        else
        {
            GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_NO_RECORDS, opcode);
        }

        break;

    case CTL_PNT_OP_ABORT:
        glucoseSendAllRecords = false;

        glucoseSendContext = false;

        //Stop notification timer.
        Util_stopClock(&notiTimeoutClock);

        GlucoseSensor_ctlPntRsp(CTL_PNT_RSP_SUCCESS, opcode);

        break;

    case CTL_PNT_OP_GET_NUM:
        // Clear all previous filter bits before running the test.
        GlucoseSensor_markAllRecords(false, GLUCOSE_MEAS_STATE_FILTER_PASS);

        GlucoseSensor_ctlPntNumRecRsp(
                GlucoseSensor_filterRecords(oper, filterType, param1, param2,
                true,
                                            GLUCOSE_MEAS_STATE_FILTER_PASS,
                                            opcode));
        break;

    default:
        //Do nothing.
        break;
    }
}

/*********************************************************************
 * @fn      GlucoseSensor_clockHandler
 *
 * @brief   Clock handle for all clock events.  This function stores an event
 *          flag and wakes up the application's event processor.
 *
 * @param   arg - event flag.
 *
 * @return  none
 */
static void GlucoseSensor_clockHandler(UArg arg)
{
    // Wake up the application.
    Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      GlucoseSensor_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t GlucoseSensor_enqueueMsg(uint16_t event, uint8_t state,
                                        uint8_t *pData)
{
    glucoseEvt_t *pMsg;

    // Create dynamic pointer to message.
    if (pMsg = ICall_malloc(sizeof(glucoseEvt_t)))
    {
        pMsg->hdr.event = event;
        pMsg->hdr.state = state;
        pMsg->pData = pData;

        // Enqueue the message.
        return Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t*) pMsg);
    }

    return FALSE;
}

/*********************************************************************
 * @fn      GlucoseSensor_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void GlucoseSensor_processConnEvt(Gap_ConnEventRpt_t *pReport)
{

    // Get index from handle
    uint8_t connIndex = GlucoseSensor_getConnIndex(pReport->handle);

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

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
/*********************************************************************
 * @fn      GlucoseSensor_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void GlucoseSensor_updateRPA(void)
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

static status_t GlucoseSensor_stopAutoPhyChange(uint16_t connHandle)
{
    // Get connection index from handle
    uint8_t connIndex = GlucoseSensor_getConnIndex(connHandle);
    GLUCOSE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    // Stop connection event notice
    Gap_RegisterConnEventCb(NULL, GAP_CB_UNREGISTER, connHandle);

    // Also update the phychange request status for active RSSI tracking connection
    connList[connIndex].phyCngRq = FALSE;
    connList[connIndex].isAutoPHYEnable = FALSE;

    return SUCCESS;
}

/*********************************************************************
 *********************************************************************/
