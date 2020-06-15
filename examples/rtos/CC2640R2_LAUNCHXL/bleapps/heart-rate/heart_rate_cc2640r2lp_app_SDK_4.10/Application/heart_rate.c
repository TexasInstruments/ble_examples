/******************************************************************************

 @file       heart_rate.c

 @brief This file contains the Heart Rate sample application for use with the
 CC26xx Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2011-2017, Texas Instruments Incorporated
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
#include "att_rsp.h"

#include "peripheral.h"
#include "heartrateservice.h"
#include "devinfoservice.h"
#include "battservice.h"

#include "board_key.h"

#include "board.h"

#include "heart_rate.h"

/*********************************************************************
 * MACROS
 */

// Convert BPM to RR-Interval for data simulation purposes.
#define HEARTRATE_BPM2RR(bpm)            ((uint16) 60 * 1024 / (uint16) (bpm))

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connectionEventRegisterCauseBitMap |= RegisterCause )
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connectionEventRegisterCauseBitMap &= (~RegisterCause) )
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)
// Gets whether the RegisterCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connectionEventRegisterCauseBitMap & RegisterCause )

/*********************************************************************
 * CONSTANTS
 */

// Fast advertising interval in 625us units.
#define DEFAULT_FAST_ADV_INTERVAL                       32

// Duration of fast advertising duration in ms.
#define DEFAULT_FAST_ADV_DURATION                       30000

// Slow advertising interval in 625us units.
#define DEFAULT_SLOW_ADV_INTERVAL                       1600

// Slow advertising duration in ms (set to 0 for continuous advertising).
#define DEFAULT_SLOW_ADV_DURATION                       0

// How often to perform heart rate periodic event.
#define DEFAULT_HEARTRATE_PERIOD                        2000

// Whether to enable automatic parameter update request when a connection is
// formed.
#define DEFAULT_ENABLE_UPDATE_REQUEST                   GAPROLE_LINK_PARAM_UPDATE_WAIT_BOTH_PARAMS

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

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL                     6

// Battery measurement period in ms
#define DEFAULT_BATT_PERIOD                             15000

// Arbitrary values used to simulate measurements.
#define HEARTRATE_BPM_DEFAULT                           73
#define HEARTRATE_BPM_MAX                               80
#define HEARTRATE_ENERGY_INCREMENT                      10
#define HEARTRATE_FLAGS_IDX_MAX                         7

#define HEARTRATE_MEAS_LEN                              9

// Task configuration
#define HEARTRATE_TASK_PRIORITY                         1
#define HEARTRATE_TASK_STACK_SIZE                       644

#define HEARTRATE_STATE_CHANGE_EVT                      0x0001
#define HEARTRATE_KEY_CHANGE_EVT                        0x0002
#define HEARTRATE_MEAS_EVT                              0x0004
#define HEARTRATE_BATT_EVT                              0x0008
#define HEARTRATE_PASSCODE_NEEDED_EVT                   0x0010
#define HEARTRATE_CONN_EVT                              0x0012

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
    FOR_ATT_RSP = 2,
    FOR_AOA_SEND = 4,
    FOR_TOF_SEND = 8
} connectionEventRegisterCause_u;



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

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct HeartRate_task;
Char HeartRate_taskStack[HEARTRATE_TASK_STACK_SIZE];

// Profile state parameter.
static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP Profile - Name attribute for SCAN RSP data (max size = 31 bytes).
static uint8_t scanData[] = {
        // Complete name.
        0x12,// length of this data
        GAP_ADTYPE_LOCAL_NAME_COMPLETE, 'H', 'e', 'a', 'r', 't', ' ', 'R', 'a',
        't', 'e', ' ', 'S', 'e', 'n', 's', 'o', 'r' };

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

// Components of heart rate measurement structure.
static uint8_t heartRateBpm = HEARTRATE_BPM_DEFAULT;
static uint16_t heartRateEnergyLvl = 0;
static uint16_t heartRateRrInterval = HEARTRATE_BPM2RR(HEARTRATE_BPM_DEFAULT);
static uint16_t heartRateRrInterval2 = HEARTRATE_BPM2RR(HEARTRATE_BPM_DEFAULT);

// Advertising user-cancelled state.
static bool advCancelled = FALSE;

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

/*********************************************************************
 * LOCAL FUNCTIONS
 */

// Task functions and message processing.
static void HeartRate_init(void);
static void HeartRate_taskFxn(UArg a0, UArg a1);
static void HeartRate_processStackMsg(ICall_Hdr *pMsg);
static void HeartRate_processGattMsg(gattMsgEvent_t *pMsg);
static void HeartRate_processAppMsg(heartRateEvt_t *pMsg);
static uint8_t HeartRate_enqueueMsg(uint8_t event, uint8_t state,
                                    uint8_t *pData);
static void HeartRate_clockHandler(UArg arg);
static void HeartRate_measPerTask(void);
static void HeartRate_battPerTask(void);
static void HeartRate_measNotify(void);
static bool HeartRate_toggleAdvertising(void);

// Events and callbacks for profiles and keys.
static void HeartRate_battCB(uint8_t event);
static void HeartRate_battEvt(uint8_t event);
static void HeartRate_serviceCB(uint8_t event);
static void HeartRate_heartRateEvt(uint8_t event);
static void HeartRate_keyPressHandler(uint8_t keys);
static void HeartRate_handleKeys(uint8_t shift, uint8_t keys);
static void HeartRate_stateChangeCB(gaprole_States_t newState);
static void HeartRate_stateChangeEvt(gaprole_States_t pEvent);

static void HeartRate_passcodeCB(uint8_t *deviceAddr,
                                 uint16_t connHandle,
                                 uint8_t uiInputs, uint8_t uiOutputs,
                                 uint32_t numComparison);
static void HeartRate_processPasscode(void);

static void HeartRate_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void HeartRate_processConnEvt(Gap_ConnEventRpt_t *pReport);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks.
static gapRolesCBs_t heartRatePeripheralCB = { HeartRate_stateChangeCB // Profile State Change Callbacks.
        };

// Bond Manager Callbacks.
static const gapBondCBs_t heartRateBondCB = {
        (pfnPasscodeCB_t) HeartRate_passcodeCB,    // Passcode callback
        NULL                     // Pairing state callback.
        };

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

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

    // Hard code the DB Address till CC2650 board gets its own IEEE address.
    //uint8_t bdAddress[B_ADDR_LEN] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22 };
    //HCI_EXT_SetBDADDRCmd(bdAddress);

    // Set device's Sleep Clock Accuracy
    //HCI_EXT_SetSCACmd(40);

    // Create an RTOS queue for message from profile to be sent to app.
    appMsgQueue = Util_constructQueue(&appMsg);

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

        // Set the GAP Role Parameters.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &initial_advertising_enable);
        GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                             &gapRole_AdvertOffTime);

        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanData), scanData);
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

    // Set the GAP Characteristics.
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

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

    // Start the Device.
    GAPRole_StartDevice(&heartRatePeripheralCB);

    // Start the Bond Manager.
    GAPBondMgr_Register((gapBondCBs_t* )&heartRateBondCB);

    // Register with GAP for HCI/Host messages
    GAP_RegisterForMsgs(selfEntity);

#if !defined (USE_LL_CONN_PARAM_UPDATE)
    // Get the currently set local supported LE features
    // The HCI will generate an HCI event that will get received in the main
    // loop
    HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

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
                while (!Queue_empty(appMsgQueue))
                {
                    heartRateEvt_t *pMsg = (heartRateEvt_t*) Util_dequeueMsg(
                            appMsgQueue);
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
    case HEARTRATE_STATE_CHANGE_EVT:
        HeartRate_stateChangeEvt((gaprole_States_t) pMsg->hdr.state);
        break;

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
        HeartRate_processPasscode();
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
        if (gapProfileState != GAPROLE_CONNECTED)
        {
            // Set fast advertising interval for user-initiated connections.
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN,
                              DEFAULT_FAST_ADV_INTERVAL);
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX,
                              DEFAULT_FAST_ADV_INTERVAL);
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION);

            // Toggle GAP advertisement status.
            // Set flag if advertising was cancelled.
            if (HeartRate_toggleAdvertising() == FALSE)
            {
                advCancelled = TRUE;
            }
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
    uint8_t advState;

    // Find the current GAP advertisement status.
    GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &advState);

    // Get the opposite state.
    advState = !advState;

    // Change the GAP advertisement status to opposite of current status.
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advState);

    return advState;
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
 * @fn      HeartRate_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void HeartRate_stateChangeCB(gaprole_States_t newState)
{
    // Enqueue the event.
    HeartRate_enqueueMsg(HEARTRATE_STATE_CHANGE_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      HeartRate_stateChangeEvt
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void HeartRate_stateChangeEvt(gaprole_States_t newState)
{
    // If no change to the GAP Role state has occurred
    if (gapProfileState == newState)
    {
        return;
    }

    // If connected
    if (newState == GAPROLE_CONNECTED)
    {
        // Get connection handle.
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
    }
    // If disconnected
    else if (gapProfileState == GAPROLE_CONNECTED
            && newState != GAPROLE_CONNECTED)
    {
        // Stop periodic measurement of heart rate.
        Util_stopClock(&measPerClock);

        if (newState == GAPROLE_WAITING_AFTER_TIMEOUT)
        {
            // Link loss timeout-- use fast advertising
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN,
                              DEFAULT_FAST_ADV_INTERVAL);
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX,
                              DEFAULT_FAST_ADV_INTERVAL);
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION);
        }
        else
        {
            // Else use slow advertising
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN,
                              DEFAULT_SLOW_ADV_INTERVAL);
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX,
                              DEFAULT_SLOW_ADV_INTERVAL);
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION);
        }

        // Enable advertising.
        HeartRate_toggleAdvertising();
    }
    // If advertising stopped
    else if (gapProfileState == GAPROLE_ADVERTISING
            && newState == GAPROLE_WAITING)
    {
        // If advertising stopped by user
        if (advCancelled)
        {
            // Disable advertising.
            advCancelled = FALSE;
        }
        // Else if fast advertising switch to slow
        else if (GAP_GetParamValue(
                TGAP_GEN_DISC_ADV_INT_MIN) == DEFAULT_FAST_ADV_INTERVAL)
        {
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN,
                              DEFAULT_SLOW_ADV_INTERVAL);
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX,
                              DEFAULT_SLOW_ADV_INTERVAL);
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION);

            // Enable advertising.
            HeartRate_toggleAdvertising();
        }
#if AUTO_ADV
    else
    {
      // Test mode: continue advertising.
      HeartRate_toggleAdvertising();
    }
#endif //AUTO_ADV
    }
#if AUTO_ADV
  else if (newState == GAPROLE_WAITING_AFTER_TIMEOUT)
  {
    // Test mode: continue advertising.
    HeartRate_toggleAdvertising();
  }
#endif //AUTO_ADV
    // If started
    else if (newState == GAPROLE_STARTED)
    {
        // Set the system ID from the bd addr.
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];
        GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);

        // Shift three bytes up.
        systemId[7] = systemId[5];
        systemId[6] = systemId[4];
        systemId[5] = systemId[3];

        // Set middle bytes to zero.
        systemId[4] = 0;
        systemId[3] = 0;

        // Pass systemId to the Device Info service.
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
                             systemId);
    }

    // Update GAP profile state.
    gapProfileState = newState;
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
        if (gapProfileState == GAPROLE_CONNECTED)
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
 * @fn      RunningSensor_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void HeartRate_processPasscode(void)
{
    uint16_t connectionHandle;
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

    // This app uses a default passcode. A real-life scenario would handle all
    // pairing scenarios and likely generate this randomly.
    GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
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
        if (gapProfileState == GAPROLE_CONNECTED)
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
 * @fn      HeartRate_perTask
 *
 * @brief   Perform a periodic heart rate measurement.
 *
 * @param   none
 *
 * @return  none
 */
static void HeartRate_measPerTask(void)
{
    if (gapProfileState == GAPROLE_CONNECTED)
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
    if (gapProfileState == GAPROLE_CONNECTED)
    {
        // Perform battery level check.
        Batt_MeasLevel();

        // Restart timer.
        Util_startClock(&battPerClock);
    }
}

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
        return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t*) pMsg);
    }

    return FALSE;
}

bStatus_t HeartRate_UnRegisterToAllConnectionEvent(
        connectionEventRegisterCause_u connectionEventRegisterCause)
{
    bStatus_t status = SUCCESS;

    CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
    // in case  there is no more registration for the connection event than unregister
    if (!CONNECTION_EVENT_IS_REGISTERED)
    {
        GAP_RegisterConnEventCb(HeartRate_connEvtCB, GAP_CB_UNREGISTER,
                                LINKDB_CONNHANDLE_ALL);
    }

    return (status);
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
    if (CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP))
    {
        // The GATT server might have returned a blePending as it was trying
        // to process an ATT Response. Now that we finished with this
        // connection event, let's try sending any remaining ATT Responses
        // on the next connection event.
        // Try to retransmit pending ATT Response (if any)
        if (attRsp_sendAttRsp() == SUCCESS)
        {
            // Disable connection event end notice
            HeartRate_UnRegisterToAllConnectionEvent(FOR_ATT_RSP);
        }
    }

}

/*********************************************************************
 * @fn      HeartRate_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void HeartRate_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
    // Enqueue the event for processing in the app context.
    if (HeartRate_enqueueMsg(HEARTRATE_CONN_EVT, 0, (uint8_t*) pReport) == FALSE)
    {
        ICall_free(pReport);
    }
}

/*********************************************************************
 * @fn      HeartRate_RegistertToAllConnectionEvent()
 *
 * @brief   register to receive connection events for all the connection
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t HeartRate_RegistertToAllConnectionEvent(
        connectionEventRegisterCause_u connectionEventRegisterCause)
{
    bStatus_t status = SUCCESS;

    // in case  there is no registration for the connection event, make the registration
    if (!CONNECTION_EVENT_IS_REGISTERED)
    {
        status = GAP_RegisterConnEventCb(HeartRate_connEvtCB, GAP_CB_REGISTER,
                                         LINKDB_CONNHANDLE_ALL);
    }
    if (status == SUCCESS)
    {
        //add the reason bit to the bitamap.
        CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
    }

    return (status);
}

/*********************************************************************
 * @fn      HeartRate_UnRegisterToAllConnectionEvent()
 *
 * @brief   Unregister connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */


/*********************************************************************
 *********************************************************************/
