/******************************************************************************

 @file       simple_serial_socket_client.c

 @brief This file contains the Simple Serial Socket Client sample application for use
        with the CC2640R2 Bluetooth Low Energy Protocol Stack.

 Group: LPRF
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2018, Texas Instruments Incorporated
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
 Release Name: simplelink_cc2640r2_sdk_02_20_00_49
 Release Date: 2018-07-16 13:19:56
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include <ti/drivers/utils/List.h>

#include "board.h"

#include <icall.h>
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"
#include "ble_user_config.h"
#include "central.h"
#include "util.h"

#include "simple_serial_socket_client.h"
#include "simple_stream_profile_client.h"
#include "simple_service_discovery.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SSSC_STATE_CHANGE_EVT                 0x0001
#define SSSC_CONN_EVT                         0x0002
#define SSSC_OUTGOING_DATA                    0x0004

// Simple Central Task Events
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

// After the connection is formed, the central will accept connection parameter
// update requests from the peripheral
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
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

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
  uint8_t    *pData;  // event data
  uint16_t   arg0;    // Generic argument
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
  NOT_REGISTER       = 0,
  FOR_STREAM         = 1
}connectionEventRegisterCause_u;

/*********************************************************************
 * GLOBAL VARIABLES
 */

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

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Serial Socket";

// Scanning state
static bool scanningStarted = FALSE;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovery done
static bool discoveryDone = FALSE;

// Maximum PDU size (default = 27 octets)
static uint16 maxPduSize;

// Server manufacturer data: 0x0D, 0x00 (TI) : 0xC0, 0x00, 0xFF, 0xEE (COFFEE)
static const uint8_t serverManufData[6] = {0x0D, 0x00, 0xC0, 0xFF, 0xEE};

// UART Interface
static UART_Handle uartHandle = NULL;

// UART read buffer
static uint8_t uartReadBuffer[UART_MAX_READ_SIZE] = {0};

// UART write list
static List_List       uartWriteList;
static uartMsg_t*      uartCurrentMsg;
static uint8_t         uartWriteActive = 0;

// Pin driver handle
static PIN_Handle ledPinHandle;
static PIN_State  ledPinState;

// PIN configuration for LEDs
static PIN_Config ledPinTable[] = {
    Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void uartReadCallback(UART_Handle handle, void *rxBuf, size_t size);
static void uartWriteCallback(UART_Handle handle, void *rxBuf, size_t size);
static uint8_t SimpleSerialSocketClient_eventCB(gapCentralRoleEvent_t *pEvent);
static void SimpleSerialSocketClient_connEvtCB(Gap_ConnEventRpt_t *pReport);

static void SimpleSerialSocketClient_init(void);
static void SimpleSerialSocketClient_taskFxn(UArg a0, UArg a1);
static void SimpleSerialSocketClient_startDiscovery(void);
static bool SimpleSerialSocketClient_findSerialStreamServer(const uint8_t *uuid, const uint8_t *pManData, uint8_t manDataLen, uint8_t *pData,
                                                            uint8_t dataLen);
static void SimpleSerialSocketClient_genericClockHandler(UArg a0);
static uint8_t SimpleSerialSocketClient_enqueueMsg(uint8_t event, uint8_t status,
                                                   uint8_t *pData, uint16_t arg0);

static void SimpleSerialSocketClient_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleSerialSocketClient_processStackMsg(ICall_Hdr *pMsg);
static void SimpleSerialSocketClient_processAppMsg(ssscEvt_t *pMsg);
static void SimpleSerialSocketClient_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void SimpleSerialSocketClient_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void SimpleSerialSocketClient_processConnEvt(Gap_ConnEventRpt_t *pReport);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Central GAPRole Callbacks
static gapCentralRoleCB_t SimpleSerialSocketClient_roleCB =
{
  SimpleSerialSocketClient_eventCB     // GAPRole Event Callback
};

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
static void uartReadCallback(UART_Handle handle, void *rxBuf, size_t size)
{
    // Pass the number of read bytes using the arg0 field
    SimpleSerialSocketClient_enqueueMsg(SSSC_OUTGOING_DATA, NULL, NULL, size);
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
static void uartWriteCallback(UART_Handle handle, void *txBuf, size_t size)
{
    // Free the last printed buffer
    if (NULL != uartCurrentMsg)
    {
        ICall_free(uartCurrentMsg);
        uartCurrentMsg = NULL;
    }

    // If the list is not empty, start another write
    if (!List_empty(&uartWriteList)) {
        uartCurrentMsg = (uartMsg_t *) List_get(&uartWriteList);
        UART_write(uartHandle, uartCurrentMsg->buffer, uartCurrentMsg->len);
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
bStatus_t SimpleSerialSocketClient_RegisterToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  // in case  there is no registration for the connection event, make the registration
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    status = GAP_RegisterConnEventCb(SimpleSerialSocketClient_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
  }
  if(status == SUCCESS)
  {
    //add the reason bit to the bitamap.
    CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
  }

  return(status);
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
bStatus_t SimpleSerialSocketClient_UnRegisterToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
  // in case  there is no more registration for the connection event than unregister
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    GAP_RegisterConnEventCb(SimpleSerialSocketClient_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
  }

  return(status);
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_createTask
 *
 * @brief   Task creation function for the Simple Central.
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

  Task_construct(&ssscTask, SimpleSerialSocketClient_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_Init
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
  Util_constructClock(&startDiscClock, SimpleSerialSocketClient_genericClockHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, SSSC_START_DISCOVERY_EVT);

  Util_constructClock(&startNotiEnableClock, SimpleSerialSocketClient_genericClockHandler,
                        200, 0, false, SSSC_NOTI_ENABLE_EVT);

  // Initialize and open PIN driver to use LEDs
  PIN_init(ledPinTable);
  ledPinHandle = PIN_open(&ledPinState, ledPinTable);

  PIN_setOutputValue(ledPinHandle, Board_RLED, 0);
  PIN_setOutputValue(ledPinHandle, Board_GLED, 0);

  // Initialize UART
  UART_Params uartParams;
  UART_init();

  // Open UART in callback mode for both read and write
  UART_Params_init(&uartParams);
  uartParams.writeDataMode  = UART_DATA_BINARY;
  uartParams.readDataMode   = UART_DATA_BINARY;
  uartParams.readReturnMode = UART_RETURN_FULL;
  uartParams.readMode       = UART_MODE_CALLBACK;
  uartParams.writeMode      = UART_MODE_CALLBACK;
  uartParams.readCallback   = uartReadCallback;
  uartParams.writeCallback  = uartWriteCallback;
  uartParams.readEcho       = UART_ECHO_OFF;
  uartParams.baudRate       = 115200;

  uartHandle = UART_open(Board_UART0, &uartParams);

  if (uartHandle == NULL) {
      // UART_open() failed
      while (1);
  }

  // Enable partial return
  UART_control(uartHandle, UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE, NULL);

  // Setup an initial read
  UART_read(uartHandle, uartReadBuffer, UART_MAX_READ_SIZE);

  // Setup the Central GAPRole Profile. For more information see the GAP section
  // in the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
  {
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;

    GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8_t),
                                &scanRes);
  }

  // Set GAP Parameters to set the discovery duration
  // For more information, see the GAP section of the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
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
  VOID GAPCentralRole_StartDevice(&SimpleSerialSocketClient_roleCB);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_taskFxn
 *
 * @brief   Application task entry point for the Simple Central.
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
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleSerialSocketClient_processStackMsg((ICall_Hdr *)pMsg);
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
          ssscEvt_t *pMsg = (ssscEvt_t *)Util_dequeueMsg(appMsgQueue);
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
          bStatus_t retVal = SimpleStreamClient_enableNotifications(connHandle);

          if (retVal == SUCCESS)
          {
              // Consider notifications active
              // (This might not be the case, we would have to read them back to make sure)
              // Green LED indicates notifications is enabled
              PIN_setOutputValue(ledPinHandle, Board_GLED, 1);
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
      SimpleSerialSocketClient_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;

    case GATT_MSG_EVENT:
      SimpleSerialSocketClient_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {

        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            hardCode = (hciEvt_HardwareError_t*) pMsg;
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, hardCode->hardwareCode);
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
  switch (pMsg->hdr.event)
  {
    case SSSC_OUTGOING_DATA:
      {
          // You could do processing of data here ...

          // For now, just Send the data read from UART over the air
          bStatus_t status = SimpleStreamClient_sendData(connHandle, &uartReadBuffer, pMsg->arg0);

          // If status is not successful, register for connection events for further processing
          if (status != SUCCESS) {
              SimpleSerialSocketClient_RegisterToAllConnectionEvent(FOR_STREAM);
          }

          // Start another read
          UART_read(uartHandle, uartReadBuffer, UART_MAX_READ_SIZE);

          break;
      }

    case SSSC_STATE_CHANGE_EVT:
      // This message is passed on from the GAPRole to the app from the stack.
      SimpleSerialSocketClient_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message, as the message origins from the stack,
      // it has to be freed using Icall_freeMsg.
      ICall_freeMsg(pMsg->pData);
      break;

    // Connection event
    case SSSC_CONN_EVT:
      {
        SimpleSerialSocketClient_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));

        ICall_free(pMsg->pData);
        break;
      }

    default:
      // Do nothing.
      break;
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
static void SimpleSerialSocketClient_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        maxPduSize = pEvent->initDone.dataPktLen;

        scanningStarted = TRUE;

        // Start discovery
        GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST);
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
          if (SimpleSerialSocketClient_findSerialStreamServer(SimpleStreamServerUUID,
                                                   serverManufData,
                                                   sizeof(serverManufData),
                                                   pEvent->deviceInfo.pEvtData,
                                                   pEvent->deviceInfo.dataLen))
          {
              // We found our device
              scanningStarted = FALSE;

              // Cancel discovery
              GAPCentralRole_CancelDiscovery();

              // Establish a connection to the device
              GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                           DEFAULT_LINK_WHITE_LIST,
                                           pEvent->deviceInfo.addrType, pEvent->deviceInfo.addr);
          }
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // If the device was not found, continue to scan for it
        if (scanningStarted == TRUE)
        {
            // Start discovery
            GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                          DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                          DEFAULT_DISCOVERY_WHITE_LIST);
        }
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;

          // Red LED indicates connection
          PIN_setOutputValue(ledPinHandle, Board_RLED, 1);

          // If service discovery not performed initiate service discovery
          if (streamServiceHandle.startHandle == GATT_INVALID_HANDLE)
          {
            Util_startClock(&startDiscClock);
          }

          HCI_LE_ReadRemoteUsedFeaturesCmd(connHandle);
        }
        else
        {
          state = BLE_STATE_IDLE;
          connHandle = GAP_CONNHANDLE_INIT;

          // Reset LEDs
          PIN_setOutputValue(ledPinHandle, Board_RLED, 0);
          PIN_setOutputValue(ledPinHandle, Board_GLED, 0);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;

        // Reset LEDs
        PIN_setOutputValue(ledPinHandle, Board_RLED, 0);
        PIN_setOutputValue(ledPinHandle, Board_GLED, 0);

        streamServiceHandle.startHandle = GATT_INVALID_HANDLE;
        streamServiceHandle.endHandle   = GATT_INVALID_HANDLE;
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
    // A notification was received, process data recevied from the steam socket
    if (pMsg->method == ATT_HANDLE_VALUE_NOTI)
    {
        uartMsg_t *newMsg;
        newMsg = ICall_malloc(sizeof(uartMsg_t) + pMsg->msg.handleValueNoti.len);

        if (newMsg) {

            newMsg->len = pMsg->msg.handleValueNoti.len;
            memcpy(newMsg->buffer, pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len);

            List_put(&uartWriteList, (List_Elem *) newMsg);

            if (uartWriteActive == 0) {
                uartWriteActive = 1;
                uartCurrentMsg = (uartMsg_t *) List_get(&uartWriteList);

                UART_write(uartHandle, uartCurrentMsg->buffer, uartCurrentMsg->len);
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
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
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

  retVal = SimpleServiceDiscovery_discoverService(connHandle, selfEntity, &streamServiceHandle, pMsg);

  // If all handles is populated or the discovery is complete
  if (retVal == SIMPLE_DISCOVERY_SUCCESSFUL)
  {
    // Try to enable notifications for the stream
    Util_startClock(&startNotiEnableClock);

    discoveryDone = TRUE;
  }
}

/*********************************************************************
 * @fn      SimpleSerialSocketClient_findSerialStreamServer
 *
 * @brief   Find a serial stream server based on given UUID in an advertiser's
 *          service UUID list and manufacturer specific data
 *
 * @return  TRUE if service UUID and matching manufacturer data found
 */
static bool SimpleSerialSocketClient_findSerialStreamServer(const uint8_t *uuid, const uint8_t *pManufData,
                                                            uint8_t manDataLen, uint8_t *pData, uint8_t dataLen)
{
  uint8_t matchingIdAndUuid = 0;
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

      // If AD type is for 128-bit service UUID
      if ((adType == GAP_ADTYPE_128BIT_MORE) ||
          (adType == GAP_ADTYPE_128BIT_COMPLETE))
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

      if(adType == GAP_ADTYPE_MANUFACTURER_SPECIFIC)
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
 * @fn      SimpleSerialSocketClient_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleSerialSocketClient_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SimpleSerialSocketClient_enqueueMsg(SSSC_STATE_CHANGE_EVT,
                                  SUCCESS, (uint8_t *)pEvent, NULL))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
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
  if( SimpleSerialSocketClient_enqueueMsg(SSSC_CONN_EVT, 0 ,(uint8_t *) pReport, NULL) == FALSE)
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
                                           uint8_t *pData, uint16_t arg0)
{
  ssscEvt_t *pMsg = ICall_malloc(sizeof(ssscEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;
    pMsg->arg0 = arg0;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
*********************************************************************/
