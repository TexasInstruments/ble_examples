/******************************************************************************

 @file       simple_serial_socket_client.c

 @brief This file contains the Simple Serial Socket client sample application
        for use  with the CC13xx / CC26xx Bluetooth Low Energy Protocol Stack.

 Group: CMCU, LPRF
 Target Device: CC1352, CC2652, CC2651R3SIPA

 ******************************************************************************

 Copyright (c) 2018-2023, Texas Instruments Incorporated
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

 ******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/UART2.h>
#include <ti/drivers/uart2/UART2CC26X2.h>
#include <ti/drivers/utils/List.h>

#include <ti_drivers_config.h>
#include "ti_ble_config.h"

#include <icall.h>
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"
#include "ble_user_config.h"
#include "util.h"
#include <ti/display/Display.h>
#include <ti/drivers/apps/LED.h>

#include "simple_serial_socket_client.h"
#include "simple_stream_profile_client.h"
#include "simple_service_discovery.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Application events
#define SSSC_EVT_KEY_CHANGE                     0x01
#define SSSC_EVT_SCAN_ENABLED                   0x02
#define SSSC_EVT_SCAN_DISABLED                  0x03
#define SSSC_EVT_ADV_REPORT                     0x04
#define SSSC_EVT_INSUFFICIENT_MEM               0x05
#define SSSC_EVT_CONN                           0x06
#define SSSC_EVT_OUTGOING_DATA                  0x07

// Simple Serial Socket Client Task Events
#define SSSC_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SSSC_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SSSC_START_DISCOVERY_EVT               Event_Id_00
#define SSSC_NOTI_ENABLE_EVT                   Event_Id_01

#define SSSC_ALL_EVENTS                        (SSSC_ICALL_EVT           | \
                                               SSSC_QUEUE_EVT            | \
                                               SSSC_START_DISCOVERY_EVT  | \
                                               SSSC_NOTI_ENABLE_EVT)

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  30

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// Task configuration
#define SSSC_TASK_PRIORITY                     1

#ifndef SSSC_TASK_STACK_SIZE
#define SSSC_TASK_STACK_SIZE                   864
#endif

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connectionEventRegisterCauseBitMap |= RegisterCause )
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connectionEventRegisterCauseBitMap &= (~RegisterCause) )
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)
// Gets whether the RegisterCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connectionEventRegisterCauseBitMap & RegisterCause )

// UART read buffer size
#define UART_MAX_READ_SIZE    (256)

#ifndef CONFIG_LEDCOUNT
#define CONFIG_LEDCOUNT        2
#else
#define CONFIG_LED2            2
#endif

// Application states
enum
{
    BLE_STATE_IDLE,
    BLE_STATE_CONNECTING,
    BLE_STATE_CONNECTED,
    BLE_STATE_DISCONNECTING
};

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
    appEvtHdr_t hdr;    // event header.
    uint8_t *pData;  // event data
    uint16_t arg0;    // Generic argument
} ssscEvt_t;

// UART Write list element
typedef struct
{
    List_Elem elem;
    uint8_t len;
    uint8_t buffer[];
} uartMsg_t;

// The following typedef is used for registration to connection event
typedef enum
{
    NOT_REGISTER = 0, FOR_STREAM = 1
} connectionEventRegisterCause_u;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
LED_Params ledParams;

Display_Handle dispHandle = NULL;
LED_Handle ledHandle[CONFIG_LEDCOUNT];

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Handle the registration and un-registration for the connection event, since only one can be registered.
static uint32_t connectionEventRegisterCauseBitMap = NOT_REGISTER; //see connectionEventRegisterCause_u

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;
static Clock_Struct startNotiEnableClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct ssscTask;
Char ssscTaskStack[SSSC_TASK_STACK_SIZE];

// Scanning state
static bool scanningStarted = FALSE;

// Connection handle of current connection
static uint16_t connHandle = LINKDB_CONNHANDLE_INVALID;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovery done
static bool discoveryDone = FALSE;

// Maximum PDU size (default = 27 octets)
static uint16_t ssscMaxPduSize;

// Server manufacturer data: 0x0D, 0x00 (TI) : 0xC0, 0x00, 0xFF, 0xEE (COFFEE)
static const uint8_t serverManufData[6] = { 0x0D, 0x00, 0xC0, 0xFF, 0xEE };

// UART Interface
static UART2_Handle uartHandle = NULL;

// UART read buffer
static uint8_t uartReadBuffer[UART_MAX_READ_SIZE] = { 0 };

// UART write list
static List_List uartWriteList;
static uartMsg_t *uartCurrentMsg;
static uint8_t uartWriteActive = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void uartReadCallback(UART2_Handle handle, void *rxBuf, size_t size, void *userArg, int_fast16_t status);
static void uartWriteCallback(UART2_Handle handle, void *rxBuf, size_t size, void *userArg, int_fast16_t status);

static void SimpleSerialSocketClient_init(void);
static void SimpleSerialSocketClient_taskFxn(UArg a0, UArg a1);

static void SimpleSerialSocketClient_processStackMsg(ICall_Hdr *pMsg);
static void SimpleSerialSocketClient_processAppMsg(ssscEvt_t *pMsg);
static void SimpleSerialSocketClient_processGapMsg(gapEventHdr_t *pMsg);
static void SimpleSerialSocketClient_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleSerialSocketClient_processConnEvt(
        Gap_ConnEventRpt_t *pReport);
static void SimpleSerialSocketClient_processGATTDiscEvent(gattMsgEvent_t *pMsg);

static void SimpleSerialSocketClient_startDiscovery(void);
static bool SimpleSerialSocketClient_findSerialStreamServer(
        const uint8_t *uuid, const uint8_t *pManData, uint8_t manDataLen,
        uint8_t *pData, uint8_t dataLen);
static void SimpleSerialSocketClient_genericClockHandler(UArg a0);
static void SimpleSerialSocketClient_connEvtCB(Gap_ConnEventRpt_t *pReport);
static uint8_t SimpleSerialSocketClient_enqueueMsg(uint8_t event,
                                                   uint8_t status,
                                                   uint8_t *pData,
                                                   uint16_t arg0);
static void SimpleSerialSocketClient_scanCb(uint32_t evt, void *msg,
                                            uintptr_t arg);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * DRIVER CALLBACKS
 */

/*********************************************************************
 * @fn      uartReadCallback
 *
 * @brief   UART read callback, notify the application that we have new data to handle
 *
 * @param   handle  - UART handle
 *          *rxBuf  - buffer containing the incoming data.
 *          size    - length of the data buffer.
 *
 * @return  None.
 */
static void uartReadCallback(UART2_Handle handle, void *rxBuf, size_t size,  void *userArg, int_fast16_t status)
{
    // Pass the number of read bytes using the arg0 field
    SimpleSerialSocketClient_enqueueMsg(SSSC_EVT_OUTGOING_DATA, NULL, NULL,
                                        size);
}

/*********************************************************************
 * @fn      uartWriteCallback
 *
 * @brief   Process more data if available in the UART write list
 *
 * @param   handle  - UART handle
 *          *txBuf  - buffer containing the written data.
 *          size    - length of the data buffer written.
 *
 * @return  None.
 */
static void uartWriteCallback(UART2_Handle handle, void *rxBuf, size_t size,  void *userArg, int_fast16_t status)
{
    // Free the last printed buffer
    if (NULL != uartCurrentMsg)
    {
        ICall_free(uartCurrentMsg);
        uartCurrentMsg = NULL;
    }

    // If the list is not empty, start another write
    if (!List_empty(&uartWriteList))
    {
        uartCurrentMsg = (uartMsg_t*) List_get(&uartWriteList);
        UART2_write(uartHandle, uartCurrentMsg->buffer, uartCurrentMsg->len, NULL);
    }
    else
    {
        uartWriteActive = 0;
    }
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleSerialSocketClient_RegistertToAllConnectionEvent()
 *
 * @brief   register to receive connection events for all the connection
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t SimpleSerialSocketClient_RegisterToAllConnectionEvent(
        connectionEventRegisterCause_u connectionEventRegisterCause)
{
    bStatus_t status = SUCCESS;

    // in case  there is no registration for the connection event, make the registration
    if (!CONNECTION_EVENT_IS_REGISTERED)
    {
        status = Gap_RegisterConnEventCb(SimpleSerialSocketClient_connEvtCB,
                                         GAP_CB_REGISTER,
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
 * @fn      SimpleSerialSocketClient_UnRegistertToAllConnectionEvent()
 *
 * @brief   Unregister connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t SimpleSerialSocketClient_UnRegisterToAllConnectionEvent(
        connectionEventRegisterCause_u connectionEventRegisterCause)
{
    bStatus_t status = SUCCESS;

    CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
    // in case  there is no more registration for the connection event than unregister
    if (!CONNECTION_EVENT_IS_REGISTERED)
    {
        Gap_RegisterConnEventCb(SimpleSerialSocketClient_connEvtCB,
                                GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
    }

    return (status);
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_createTask
 *
 * @brief   Task creation function for the Simple Serial Socket Client.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleSerialSocketClient_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = ssscTaskStack;
    taskParams.stackSize = SSSC_TASK_STACK_SIZE;
    taskParams.priority = SSSC_TASK_PRIORITY;

    Task_construct(&ssscTask, SimpleSerialSocketClient_taskFxn, &taskParams,
                   NULL);
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_Init
 *
 * @brief   Initialization function for the Simple Serial Socket Client App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleSerialSocketClient_init(void)
{
    // ******************************************************************
    // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);

    // Create an RTOS queue for message from profile to be sent to app.
    appMsgQueue = Util_constructQueue(&appMsg);

    // Setup discovery delay as a one-shot timer
    Util_constructClock(&startDiscClock,
                        SimpleSerialSocketClient_genericClockHandler,
                        DEFAULT_SVC_DISCOVERY_DELAY,
                        0, false, SSSC_START_DISCOVERY_EVT);

    Util_constructClock(&startNotiEnableClock,
                        SimpleSerialSocketClient_genericClockHandler, 200, 0,
                        false, SSSC_NOTI_ENABLE_EVT);

    // Initialize and open PIN driver to use LEDs
    LED_init();
    LED_Params_init(&ledParams);
    ledHandle[CONFIG_LED_0] = LED_open(CONFIG_LED_0, &ledParams);
    ledHandle[CONFIG_LED_1] = LED_open(CONFIG_LED_1, &ledParams);

    // Initialize UART

    // Open UART in callback mode for both read and write
    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.readReturnMode = UART2_ReadReturnMode_PARTIAL;
    uartParams.readMode = UART2_Mode_CALLBACK;
    uartParams.writeMode = UART2_Mode_CALLBACK;
    uartParams.readCallback = uartReadCallback;
    uartParams.writeCallback = uartWriteCallback;
    uartParams.baudRate = 115200;

    uartHandle = UART2_open(CONFIG_DISPLAY_UART, &uartParams);

    if (uartHandle == NULL)
    {
        // UART_open() failed
        while (1)
            ;
    }

    // Setup an initial read
    UART2_read(uartHandle, uartReadBuffer, UART_MAX_READ_SIZE, NULL);

    // Make sure to leave at least MIN_HEAP_HEADROOM of heap
    // for the BLE stack application to operate.
    SimpleStreamClient_setHeadroomLimit(4000);

    // Initialize GATT attributes
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                     (void* )attDeviceName);

    //Set default values for Data Length Extension
    //Extended Data Length Feature is already enabled by default
    //in build_config.opt in stack project.
    {
        //Change initial values of RX/TX PDU and Time, RX is set to max. by default(251 octets, 2120us)
#define APP_SUGGESTED_RX_PDU_SIZE 251     //default is 251 octets(RX)
#define APP_SUGGESTED_RX_TIME     17000   //default is 17000us(RX)
#define APP_SUGGESTED_TX_PDU_SIZE 27      //default is 27 octets(TX)
#define APP_SUGGESTED_TX_TIME     328     //default is 328us(TX)

        //This API is documented in hci.h
        //See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
        //http://software-dl.ti.com/lprf/ble5stack-latest/
        HCI_EXT_SetMaxDataLenCmd(APP_SUGGESTED_TX_PDU_SIZE,
                                 APP_SUGGESTED_TX_TIME,
                                 APP_SUGGESTED_RX_PDU_SIZE,
                                 APP_SUGGESTED_RX_TIME);
    }

    // Initialize GATT Client
    VOID GATT_InitClient("");

    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(selfEntity);

    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);// GAP
    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

    // Register with GAP for HCI/Host messages (for RSSI)
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Accept all parameter update requests
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION,
                      GAP_UPDATE_REQ_ACCEPT_ALL);

    // Initialize GAP layer for Central role and register to receive GAP events
    GAP_DeviceInit(GAP_PROFILE_CENTRAL, selfEntity, ADDRMODE_RP_WITH_PUBLIC_ID,
                   NULL);
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_taskFxn
 *
 * @brief   Application task entry point for the Simple Serial Socket Client.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void SimpleSerialSocketClient_taskFxn(UArg a0, UArg a1)
{
    // Initialize application
    SimpleSerialSocketClient_init();

    // Application main loop
    for (;;)
    {
        uint32_t events;

        events = Event_pend(syncEvent, Event_Id_NONE, SSSC_ALL_EVENTS,
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
                    // Process inter-task message
                    SimpleSerialSocketClient_processStackMsg((ICall_Hdr*) pMsg);
                }

                if (pMsg)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // If RTOS queue is not empty, process app message
            if (events & SSSC_QUEUE_EVT)
            {
                while (!Queue_empty(appMsgQueue))
                {
                    ssscEvt_t *pMsg = (ssscEvt_t*) Util_dequeueMsg(appMsgQueue);
                    if (pMsg)
                    {
                        // Process message
                        SimpleSerialSocketClient_processAppMsg(pMsg);

                        // Free the space from the message
                        ICall_free(pMsg);
                    }
                }
            }

            if (events & SSSC_START_DISCOVERY_EVT)
            {
                SimpleSerialSocketClient_startDiscovery();
            }

            // Enable stream notifications
            if (events & SSSC_NOTI_ENABLE_EVT)
            {
                // Enable notification for incoming data
                bStatus_t retVal = SimpleStreamClient_enableNotifications(
                        connHandle);

                if (retVal == SUCCESS)
                {
                    // Consider notifications active
                    // (This might not be the case, we would have to read them back to make sure)
                    // Green LED indicates notifications is enabled
//                    LED_setOn(ledHandle[CONFIG_LED_1], 100);
                }
            }
        }
    }
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleSerialSocketClient_processStackMsg(ICall_Hdr *pMsg)
{
    hciEvt_HardwareError_t *hardCode;

    switch (pMsg->event)
    {
    case GAP_MSG_EVENT:
        SimpleSerialSocketClient_processGapMsg((gapEventHdr_t*) pMsg);
        break;

    case GATT_MSG_EVENT:
        SimpleSerialSocketClient_processGATTMsg((gattMsgEvent_t*) pMsg);
        break;

    case HCI_GAP_EVENT_EVENT:
    {

        // Process HCI message
        switch (pMsg->status)
        {
        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            hardCode = (hciEvt_HardwareError_t*) pMsg;
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,
                          hardCode->hardwareCode);
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
 * @fn      SimpleSerialSocketClient_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleSerialSocketClient_processAppMsg(ssscEvt_t *pMsg)
{
    bool safeToDealloc = TRUE;

    switch (pMsg->hdr.event)
    {
    case SSSC_EVT_ADV_REPORT:
    {

        GapScan_Evt_AdvRpt_t *pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);
        if (SimpleSerialSocketClient_findSerialStreamServer(
                SimpleStreamServerUUID, serverManufData,
                sizeof(serverManufData), pAdvRpt->pData, pAdvRpt->dataLen))
        {
            // We found our device
            scanningStarted = FALSE;

            // Disable scanning
            GapScan_disable("");

            // Establish a connection to the device
            GapInit_connect(pAdvRpt->addrType & MASK_ADDRTYPE_ID, pAdvRpt->addr,
                            INIT_PHY_1M, 0);
        }

        // Free report payload data
        if (pAdvRpt->pData != NULL)
        {
            ICall_free(pAdvRpt->pData);
        }

        break;
    }

    case SSSC_EVT_SCAN_DISABLED:
    {
        // If the device was not found, continue to scan for it
        if (scanningStarted == TRUE)
        {
            // Re-enable scan
            GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
        }
        break;
    }

    case SSSC_EVT_OUTGOING_DATA:
    {
        // You could do processing of data here ...

        // ... for now, just send the data read from UART over the air
        bStatus_t status = SimpleStreamClient_sendData(connHandle,
                                                       &uartReadBuffer,
                                                       pMsg->arg0);

        // If status is not successful, register for connection events for further processing
        if (status != SUCCESS)
        {
            SimpleSerialSocketClient_RegisterToAllConnectionEvent(FOR_STREAM);
        }

        // Start another read
        UART2_read(uartHandle, uartReadBuffer, UART_MAX_READ_SIZE, NULL);

        break;
    }

        // Connection event
    case SSSC_EVT_CONN:
    {
        SimpleSerialSocketClient_processConnEvt(
                (Gap_ConnEventRpt_t*) (pMsg->pData));

        ICall_free(pMsg->pData);
        break;
    }

    default:
        // Do nothing.
        break;
    }

    if ((safeToDealloc == TRUE) && (pMsg->pData != NULL))
    {
        ICall_free(pMsg->pData);
    }
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleSerialSocketClient_processGapMsg(gapEventHdr_t *pMsg)
{
    switch (pMsg->opcode)
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        uint8_t temp8;
        uint16_t temp16;
        gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t*) pMsg;

        // Setup scanning
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        // Register callback to process Scanner events
        GapScan_registerCb(SimpleSerialSocketClient_scanCb, NULL);

        // Set Scanner Event Mask
        GapScan_setEventMask(
                GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED | GAP_EVT_ADV_REPORT);

        // Set Scan PHY parameters
        GapScan_setPhyParams(SCAN_PRIM_PHY_1M, SCAN_TYPE_ACTIVE,
                             DEFAULT_SCAN_INTERVAL, DEFAULT_SCAN_INTERVAL);

        // Set Advertising report fields to keep
        temp16 = (SCAN_ADVRPT_FLD_ADDRTYPE | SCAN_ADVRPT_FLD_ADDRESS);
        GapScan_setParam(SCAN_PARAM_RPT_FIELDS, &temp16);
        // Set Scanning Primary PHY
        temp8 = ADDRMODE_RP_WITH_PUBLIC_ID;
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

        ssscMaxPduSize = pPkt->dataPktLen;

        // Start the scan
        scanningStarted = TRUE;
        discoveryDone = FALSE;
        GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
        UART2_write(uartHandle, "Info: Done Initializing\n\r", 25, NULL);
    }
        break;

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        if (pMsg->hdr.status == SUCCESS)
        {
            state = BLE_STATE_CONNECTED;
            connHandle = ((gapEstLinkReqEvent_t*) pMsg)->connectionHandle;

            // Red LED indicates connection
            LED_setOn(ledHandle[CONFIG_LED_0], 100);

            // If service discovery not performed initiate service discovery
            if (streamServiceHandle.startHandle == GATT_INVALID_HANDLE)
            {
                Util_startClock(&startDiscClock);
            }

            HCI_LE_ReadRemoteUsedFeaturesCmd(connHandle);

            // Send a message from the client to the server
            UART2_write(uartHandle, "Info: Client Connected. Start typing.\n\r", 39, NULL);
        }
        else
        {
            state = BLE_STATE_IDLE;
            connHandle = LINKDB_CONNHANDLE_INVALID;

            // Reset LEDs
            LED_setOff(ledHandle[CONFIG_LED_0]);
            LED_setOff(ledHandle[CONFIG_LED_1]);
        }
    }
        break;

    case GAP_LINK_TERMINATED_EVENT:
    {
        state = BLE_STATE_IDLE;
        connHandle = LINKDB_CONNHANDLE_INVALID;

        // Reset LEDs
        LED_setOff(ledHandle[CONFIG_LED_0]);
        LED_setOff(ledHandle[CONFIG_LED_1]);
        // Clean up service handles
        streamServiceHandle.startHandle = GATT_INVALID_HANDLE;
        streamServiceHandle.endHandle = GATT_INVALID_HANDLE;

        // Start the scan
        scanningStarted = TRUE;
        discoveryDone = FALSE;
        GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
    }
        break;

    default:
        break;
    }
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void SimpleSerialSocketClient_processGATTMsg(gattMsgEvent_t *pMsg)
{
    if (state == BLE_STATE_CONNECTED)
    {
        // A notification was received, process data received from the steam socket
        if (pMsg->method == ATT_HANDLE_VALUE_NOTI)
        {
            uartMsg_t *newMsg;
            newMsg = ICall_malloc(
                    sizeof(uartMsg_t) + pMsg->msg.handleValueNoti.len);

            if (newMsg)
            {
                newMsg->len = pMsg->msg.handleValueNoti.len;
                memcpy(newMsg->buffer, pMsg->msg.handleValueNoti.pValue,
                       pMsg->msg.handleValueNoti.len);

                List_put(&uartWriteList, (List_Elem*) newMsg);

                if (uartWriteActive == 0)
                {
                    uartWriteActive = 1;
                    uartCurrentMsg = (uartMsg_t*) List_get(&uartWriteList);

                    UART2_write(uartHandle, uartCurrentMsg->buffer,
                               uartCurrentMsg->len, NULL);
                }
            }
        }
        else if (discoveryDone == FALSE)
        {
            SimpleSerialSocketClient_processGATTDiscEvent(pMsg);
        }
    } // else - in case a GATT message came after a connection has dropped, ignore it.

    // Needed only for ATT Protocol messages
    GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimpleSerialSocketClient_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
    if (CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_STREAM))
    {
        // The stream is active, process the stream at every connection event.
        uint8_t status = SimpleStreamClient_processStream();

        if (status == SUCCESS)
        {
            // No more data is awaiting processing, un-register to the connection events
            SimpleSerialSocketClient_UnRegisterToAllConnectionEvent(FOR_STREAM);
        }
    }
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */

static void SimpleSerialSocketClient_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
    uint8_t retVal = SUCCESS;

    retVal = SimpleServiceDiscovery_discoverService(connHandle, selfEntity,
                                                    &streamServiceHandle, pMsg);

    // If all handles is populated or the discovery is complete
    if (retVal == SIMPLE_DISCOVERY_SUCCESSFUL)
    {
        // Try to enable notifications for the stream
        Util_startClock(&startNotiEnableClock);

        discoveryDone = TRUE;
    }
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_startDiscovery
 *
 * @brief   Start service discovery by issuing a MTU exchange.
 *
 * @return  none
 */
static void SimpleSerialSocketClient_startDiscovery(void)
{
    attExchangeMTUReq_t req;

    // Discover GATT Server's Rx MTU size
    req.clientRxMTU = ssscMaxPduSize - L2CAP_HDR_SIZE;

    // ATT MTU size should be set to the minimum of the Client Rx MTU
    // and Server Rx MTU values
    VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_findSerialStreamServer
 *
 * @brief   Find a serial stream server based on given UUID in an advertiser's
 *          service UUID list and manufacturer specific data
 *
 * @return  TRUE if service UUID and matching manufacturer data found
 */
static bool SimpleSerialSocketClient_findSerialStreamServer(
        const uint8_t *uuid, const uint8_t *pManufData, uint8_t manDataLen,
        uint8_t *pData, uint8_t dataLen)
{
    uint8_t matchingIdAndUuid = 0;
    uint8_t adLen;
    uint8_t adType;
    uint8_t *pEnd;

    pEnd = pData + dataLen - 1;

    // While end of data not reached
    while ((dataLen > 0) && (pData < pEnd))
    {
        // Get length of next AD item
        adLen = *pData++;
        if (adLen > 0)
        {
            adType = *pData;

            // If AD type is for 128-bit service UUID
            if ((adType == GAP_ADTYPE_128BIT_MORE)
                    || (adType == GAP_ADTYPE_128BIT_COMPLETE))
            {
                pData++;
                adLen--;

                // For each UUID in list
                while (adLen >= 2 && pData < pEnd)
                {
                    // Check for match
                    if (!memcmp(pData, uuid, ATT_UUID_SIZE))
                    {
                        matchingIdAndUuid++;
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

            // REMOVED FOR TESTING
            if (adType == GAP_ADTYPE_MANUFACTURER_SPECIFIC)
            {
                // Floor to the lowest length
                uint8_t len = adLen;
                if (manDataLen < len)
                {
                    len = manDataLen;
                }

                if (memcmp(pManufData, pData, len))
                {
                    matchingIdAndUuid++;
                }
            }

            if (matchingIdAndUuid < 2)
            {
                // Go to next item
                pData += adLen;
            }
            else
            {
                return TRUE;
            }

        }
    }

    // Match not found
    return FALSE;
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_genericClockHandler
 *
 * @brief   Generic clock handler for events
 *
 * @param   a0 - passed on
 *
 * @return  none
 */
void SimpleSerialSocketClient_genericClockHandler(UArg a0)
{
    Event_post(syncEvent, a0);
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void SimpleSerialSocketClient_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
    // Enqueue the event for processing in the app context.
    if (SimpleSerialSocketClient_enqueueMsg(SSSC_EVT_CONN, 0,
                                            (uint8_t*) pReport, NULL) != SUCCESS)
    {
        ICall_free(pReport);
    }
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleSerialSocketClient_enqueueMsg(uint8_t event, uint8_t state,
                                                   uint8_t *pData,
                                                   uint16_t arg0)
{
    uint8_t success;
    ssscEvt_t *pMsg = ICall_malloc(sizeof(ssscEvt_t));

    // Create dynamic pointer to message.
    if (pMsg)
    {
        pMsg->hdr.event = event;
        pMsg->hdr.state = state;
        pMsg->pData = pData;
        pMsg->arg0 = arg0;

        // Enqueue the message.
        success = Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t*) pMsg);
        return (success) ? SUCCESS : FAILURE;
    }

    return FALSE;
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_scanCb
 *
 * @brief   Callback called by GapScan module
 *
 * @param   evt - event
 * @param   msg - message coming with the event
 * @param   arg - user argument
 *
 * @return  none
 */
void SimpleSerialSocketClient_scanCb(uint32_t evt, void *pMsg, uintptr_t arg)
{
    uint8_t event;

    if (evt & GAP_EVT_ADV_REPORT)
    {
        event = SSSC_EVT_ADV_REPORT;
    }
    else if (evt & GAP_EVT_SCAN_ENABLED)
    {
        event = SSSC_EVT_SCAN_ENABLED;
    }
    else if (evt & GAP_EVT_SCAN_DISABLED)
    {
        event = SSSC_EVT_SCAN_DISABLED;
    }
    else if (evt & GAP_EVT_INSUFFICIENT_MEMORY)
    {
        event = SSSC_EVT_INSUFFICIENT_MEM;
    }
    else
    {
        return;
    }

    if (SimpleSerialSocketClient_enqueueMsg(event, SUCCESS, pMsg,
                                            NULL) != SUCCESS)
    {
        ICall_free(pMsg);
    }
}

/*********************************************************************
 *********************************************************************/
