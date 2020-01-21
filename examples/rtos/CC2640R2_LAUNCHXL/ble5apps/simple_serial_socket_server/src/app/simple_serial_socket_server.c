/******************************************************************************

 @file  simple_serial_socket_server.c

 @brief This file contains the Simple Serial Socket server sample application
        for use  with the CC2640R2 Bluetooth Low Energy 5 Protocol Stack.

 Group: CMCU, LPRF
 Target Device: CC2640R2

 ******************************************************************************

 Copyright (c) 2020, Texas Instruments Incorporated
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

#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include <ti/drivers/utils/List.h>

#include "board.h"

#include <icall.h>
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"
#include "devinfoservice.h"
#include "ll_common.h"
#include "util.h"

#include "simple_serial_socket_server.h"
#include "simple_stream_profile_server.h"

/*********************************************************************
 * CONSTANTS
 */

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Task configuration
#define SSSS_TASK_PRIORITY                     1
#ifndef SSSS_TASK_STACK_SIZE
#define SSSS_TASK_STACK_SIZE                   644
#endif

// Application events
#define SSSS_CONN_EVT                          0x0001
#define SSSS_ADV_EVT                           0x0002
#define SSSS_OUTGOING_DATA                     0x0020

// Internal Events for RTOS application
#define SSSS_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SSSS_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Bitwise OR of all events to pend on
#define SSSS_ALL_EVENTS                        (SSSS_ICALL_EVT        | \
                                               SSSS_QUEUE_EVT)

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connectionEventRegisterCauseBitMap |= RegisterCause )
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connectionEventRegisterCauseBitMap &= (~RegisterCause) )
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)
// Gets whether the RegisterCause was registered to receive connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connectionEventRegisterCauseBitMap & RegisterCause )

//  UART read buffer size
#define UART_MAX_READ_SIZE    (256)

// Minimum heap headroom for BLE application
#define MIN_HEAP_HEADROOM     (4000)

// Spin if the expression is not true
#define SimpleSerialSocketServer_ASSERT(expr) if (!(expr)) simple_serial_socket_spin();

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;    // event header.
  uint8_t    *pData;  // event data
  uint16_t   arg0;    // Generic argument
} ssssEvt_t;

// UART msg list element
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
  FOR_ATT_RSP        = 1,
  FOR_STREAM         = 2
}connectionEventRegisterCause_u;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} spGapAdvEventData_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Handle the registration and un-registration for the connection event, since only one can be registered.
static uint32_t       connectionEventRegisterCauseBitMap = NOT_REGISTER; //see connectionEventRegisterCause_u

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct ssssTask;
uint8_t ssssTaskStack[SSSS_TASK_STACK_SIZE];

// Scan response data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x13,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',
  'i',
  'm',
  'p',
  'l',
  'e',
  'S',
  'e',
  'r',
  'i',
  'a',
  'l',
  'S',
  'o',
  'c',
  'k',
  'e',
  't',

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
  // Flags: this field sets the device to use general discoverable
  // mode (advertises indefinitely) instead of general
  // discoverable mode (advertise for 30 seconds at a time)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x11,   // length of this data
  GAP_ADTYPE_128BIT_MORE,      // some of the UUID's, but not all
  TI_BASE_UUID_128(SIMPLESTREAMSERVER_SERV_UUID),

  // ID nugget
  0x06,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  // Texas Instruments company ID
  0x0D,
  0x00,
  // Custom data identifier
  0xC0,
  0xFF,
  0xEE
};

// Advertising handles
static uint8 advHandleLegacy;
static uint8 advHandleLongRange;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Serial Socket";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Global connection handle
static uint16_t connHandle = 0xFFFF;

// UART Interface
static UART_Handle uartHandle = NULL;

// UART read buffer
static uint8_t uartReadBuffer[UART_MAX_READ_SIZE] = {0};

// UART write list
static List_List  uartWriteList;
static uartMsg_t* uartCurrentMsg;
static uint8_t    uartWriteActive = 0;

// Pin driver handle
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

// PIN configuration for LEDs
static PIN_Config ledPinTable[] = {
    Board_PIN_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void uartReadCallback(UART_Handle handle, void *rxBuf, size_t size);
static void uartWriteCallback(UART_Handle handle, void *rxBuf, size_t size);

static void SimpleStreamServer_cccUpdateCB(uint16_t value);
static void SimpleStreamServer_incomingDataCB(uint16_t connHandle,
                              uint8_t paramID, uint16_t len,
                              uint8_t *pValue);

static void SimpleSerialSocketServer_init( void );
static void SimpleSerialSocketServer_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleSerialSocketServer_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleSerialSocketServer_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleSerialSocketServer_processGapMessage(gapEventHdr_t *pMsg);
static void SimpleSerialSocketServer_processConnEvt(Gap_ConnEventRpt_t *pReport);
static void SimpleSerialSocketServer_processAppMsg(ssssEvt_t *pMsg);

static void SimpleSerialSocketServer_sendAttRsp(void);
static void SimpleSerialSocketServer_freeAttRsp(uint8_t status);

static uint8_t SimpleSerialSocketServer_enqueueMsg(uint8_t event, uint8_t state,
                                              uint8_t *pData, uint16_t arg0);

static void SimpleSerialSocketServer_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void SimpleSerialSocketServer_advCB(uint32_t event, void *pBuf, uintptr_t arg);


/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// SimpleStreamServer Profile Callbacks
static SimpleStreamServerCBs_t SimpleStreamServer_SimpleStreamServerProfileCBs =
{
  .pfnCccUpdateCb     = SimpleStreamServer_cccUpdateCB,    // Characteristic configuration update handler
  .pfnIncomingDataCb  = SimpleStreamServer_incomingDataCB, // Incoming data handler
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      simple_serial_socket_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void simple_serial_socket_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      SimpleSerialSocketServer_RegisterToAllConnectionEvent()
 *
 * @brief   register to receive connection events for all the connection
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t SimpleSerialSocketServer_RegisterToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  // in case  there is no registration for the connection event, make the registration
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    status = Gap_RegisterConnEventCb(SimpleSerialSocketServer_connEvtCB, GAP_CB_REGISTER, CONNHANDLE_ALL);
  }
  if(status == SUCCESS)
  {
    //add the reason bit to the bitmap.
    CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
  }

  return(status);
}

/*********************************************************************
 * @fn      SimpleSerialSocketServer_UnRegisterToAllConnectionEvent()
 *
 * @brief   Unregister connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t SimpleSerialSocketServer_UnRegisterToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
  // in case  there is no more registration for the connection event than unregister
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    Gap_RegisterConnEventCb(SimpleSerialSocketServer_connEvtCB, GAP_CB_UNREGISTER, CONNHANDLE_ALL);
  }

  return(status);
}


/*********************************************************************
 * @fn      uartReadCallback
 *
 * @brief   UART read callback, notify the application it needs to handle the data
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
  SimpleSerialSocketServer_enqueueMsg(SSSS_OUTGOING_DATA, NULL, NULL, size);
}

/*********************************************************************
 * @fn      uartWriteCallback
 *
 * @brief   UART write callback, perform additional writes if the UART msg
 *          list is not empty
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
 * @fn      SimpleStreamServer_cccUpdateCB
 *
 * @brief   Callback from SimpleStreamServer Profile indicating state change
 *
 * @param   connHandle - connection handle tied to the state change
 *          state      - New state
 *
 * @return  None.
 */
static void SimpleStreamServer_cccUpdateCB(uint16_t value)
{
  // Notifications is active
  if (value == 0x0001) {
    // Indicate that the stream is enabled
    PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 1);
  }
  else
  {
    // Indicate that the stream is disabled
    PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 0);
  }
}

/*********************************************************************
 * @fn      SimpleStreamServer_incomingDataCB
 *
 * @brief   Callback from SimpleStreamServer Profile indicating incoming data
 *
 * @param   paramId - parameter Id of the value that was changed.
 *          len     - length of the data buffer.
 *          *data - buffer containing the incoming data.
 *
 * @return  None.
 */
static void SimpleStreamServer_incomingDataCB(uint16_t connHandle,
                                            uint8_t paramID, uint16_t len,
                                            uint8_t *data)
{
  // Try to allocate and store the data to our UART write queue
  uartMsg_t *newMsg;
  newMsg = SimpleStreamServer_allocateWithHeadroom(sizeof(uartMsg_t) + len);

  if (newMsg)
  {
    newMsg->len = len;
    memcpy(newMsg->buffer, data, len);

    List_put(&uartWriteList, (List_Elem *) newMsg);

    // If no write is currently active, start a new one
    if (uartWriteActive == 0)
    {
      uartWriteActive = 1;
      uartCurrentMsg = (uartMsg_t *) List_get(&uartWriteList);

      UART_write(uartHandle, uartCurrentMsg->buffer, uartCurrentMsg->len);
    }
  }
}

 /*********************************************************************
 * @fn      SimpleSerialSocketServer_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleSerialSocketServer_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = ssssTaskStack;
  taskParams.stackSize = SSSS_TASK_STACK_SIZE;
  taskParams.priority = SSSS_TASK_PRIORITY;

  Task_construct(&ssssTask, SimpleSerialSocketServer_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleSerialSocketServer_init
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
static void SimpleSerialSocketServer_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Initialize and open PIN driver to use LEDs
  PIN_init(ledPinTable);
  ledPinHandle = PIN_open(&ledPinState, ledPinTable);

  PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, 0);
  PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 0);

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

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = GAP_UPDATE_REQ_ACCEPT_ALL;

    // Accept all parameter update requests
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service

  // Initialize Simple data stream service
  SimpleStreamServer_AddService(GATT_ALL_SERVICES);
  SimpleStreamServer_RegisterAppCBs(&SimpleStreamServer_SimpleStreamServerProfileCBs);

  // Make sure to leave at least MIN_HEAP_HEADROOM of heap
  // for the BLE stack application to operate.
  SimpleStreamServer_setHeadroomLimit(MIN_HEAP_HEADROOM);

  // Register to connection events
  SimpleSerialSocketServer_RegisterToAllConnectionEvent(FOR_STREAM);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html
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
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  GATT_InitClient();

  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, ADDRMODE_RP_WITH_PUBLIC_ID, NULL);
}

/*********************************************************************
 * @fn      SimpleSerialSocketServer_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleSerialSocketServer_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleSerialSocketServer_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, SSSS_ALL_EVENTS,
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

          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = SimpleSerialSocketServer_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SSSS_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
            ssssEvt_t *pMsg = (ssssEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            SimpleSerialSocketServer_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      SimpleSerialSocketServer_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleSerialSocketServer_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleSerialSocketServer_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleSerialSocketServer_processGATTMsg((gattMsgEvent_t *)pMsg);
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
 * @fn      SimpleSerialSocketServer_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleSerialSocketServer_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if ( SimpleSerialSocketServer_RegisterToAllConnectionEvent(FOR_ATT_RSP) == SUCCESS)
    {
      // First free any pending response
      SimpleSerialSocketServer_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimpleSerialSocketServer_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimpleSerialSocketServer_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  if ( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP))
  {
    // The GATT server might have returned a blePending as it was trying
    // to process an ATT Response. Now that we finished with this
    // connection event, let's try sending any remaining ATT Responses
    // on the next connection event.
    // Try to retransmit pending ATT Response (if any)
    SimpleSerialSocketServer_sendAttRsp();
  }

  if (CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_STREAM))
  {
    // The stream is active, process the stream at every connection event.
    bStatus_t status = SimpleStreamServer_processStream();

    // If status is successful there is no data pending for processing
    if (status == SUCCESS)
    {
      SimpleSerialSocketServer_UnRegisterToAllConnectionEvent(FOR_STREAM);
    }
  }
}

/*********************************************************************
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleSerialSocketServer_sendAttRsp(void)
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
      SimpleSerialSocketServer_UnRegisterToAllConnectionEvent (FOR_ATT_RSP);
      // We're done with the response message
      SimpleSerialSocketServer_freeAttRsp(status);
    }
  }
}

/*********************************************************************
 * @fn      SimpleSerialSocketServer_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleSerialSocketServer_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status != SUCCESS)
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SimpleSerialSocketServer_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleSerialSocketServer_processAppMsg(ssssEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
   // Outgoing data event
   case SSSS_OUTGOING_DATA:
     {
       // You could do processing of data here ...

       // For now, just Send the data read from UART over the air
       bStatus_t status = SimpleStreamServer_sendData(connHandle, &uartReadBuffer, pMsg->arg0);

       // If status is not successful, register for connection events for further processing
       if (status != SUCCESS) {
           SimpleSerialSocketServer_RegisterToAllConnectionEvent(FOR_STREAM);
       }

       // Start another read
       UART_read(uartHandle, uartReadBuffer, UART_MAX_READ_SIZE);

       break;
     }

   case SSSS_ADV_EVT:
     {
        // All events have associated memory to free except the insufficient memory
        // event
        if (((spGapAdvEventData_t *)(pMsg->pData))->event != GAP_EVT_INSUFFICIENT_MEMORY)
        {
          ICall_free(((spGapAdvEventData_t *)(pMsg->pData))->pBuf);
        }
        
        ICall_free(pMsg->pData);
        break;
    }

    // Connection event
    case SSSS_CONN_EVT:
      {
        SimpleSerialSocketServer_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));

        ICall_free(pMsg->pData);
        break;
      }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleSerialSocketServer_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void SimpleSerialSocketServer_processGapMessage(gapEventHdr_t *pMsg)
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

        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        // Temporary memory for advertising parameters for set #1. These will be copied
        // by the GapAdv module
        GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&SimpleSerialSocketServer_advCB, &advParamLegacy,
                               &advHandleLegacy);
        SimpleSerialSocketServer_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advertData), advertData);
        SimpleSerialSocketServer_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanRspData), scanRspData);
        SimpleSerialSocketServer_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        SimpleSerialSocketServer_ASSERT(status == SUCCESS);

        // Use long range params to create long range set #2
        GapAdv_params_t advParamLongRange = GAPADV_PARAMS_AE_LONG_RANGE_CONN;

        // Create Advertisement set #2 and assign handle
        status = GapAdv_create(&SimpleSerialSocketServer_advCB, &advParamLongRange,
                               &advHandleLongRange);
        SimpleSerialSocketServer_ASSERT(status == SUCCESS);

        // Load advertising data for set #2 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLongRange, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advertData), advertData);
        SimpleSerialSocketServer_ASSERT(status == SUCCESS);

        // Set event mask for set #2
        status = GapAdv_setEventMask(advHandleLongRange,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable long range advertising for set #2
        status = GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        SimpleSerialSocketServer_ASSERT(status == SUCCESS);
      }

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      uint8_t numActive = 0;

      numActive = linkDB_NumActive();

      // Use numActive to determine the connection handle of the last
      // connection and save it
      if (connHandle == 0xFFFF)
      {
        connHandle = numActive - 1;
      }

      // Red LED indicates connection
      PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, 1);

      // Stop advertising since there is no room for more connections
      GapAdv_disable(advHandleLongRange);
      GapAdv_disable(advHandleLegacy);

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      // Clear connHandle
      connHandle = 0xFFFF;

      // "Disconnect" the stream as well
      SimpleStreamServer_disconnectStream();

      // Indicate disconnection by turning of the LEDs
      PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, 0);
      PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 0);

      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      // The application is setup to accept all params update request
      break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      // Here you could handle params update events
      break;
    }

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleSerialSocketServer_advCB
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void SimpleSerialSocketServer_advCB(uint32_t event, void *pBuf, uintptr_t arg)
{
  spGapAdvEventData_t *pData = ICall_malloc(sizeof(spGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(SimpleSerialSocketServer_enqueueMsg(SSSS_ADV_EVT, NULL, (uint8_t *) pData, NULL) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimpleSerialSocketServer_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void SimpleSerialSocketServer_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if( SimpleSerialSocketServer_enqueueMsg(SSSS_CONN_EVT, 0 ,(uint8_t *) pReport, NULL) != SUCCESS)
  {
    ICall_free(pReport);
  }
}

/*********************************************************************
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 * @param   arg0  - generic argument.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleSerialSocketServer_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData, uint16_t arg0)
{
  uint8_t success;
  ssssEvt_t *pMsg = ICall_malloc(sizeof(ssssEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;
    pMsg->arg0  = arg0;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return FALSE;
}
/*********************************************************************
*********************************************************************/
