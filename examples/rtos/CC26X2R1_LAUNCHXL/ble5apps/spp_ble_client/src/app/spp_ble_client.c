/******************************************************************************

 @file  spp_ble_client.c

 @brief This file contains the SPP BLE Client sample application for use
        with the CC26xx Bluetooth Low Energy Protocol Stack.

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

#include "bcomdef.h"

#include "icall.h"
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "board_key.h"
#include "board.h"

#include "ble_user_config.h"

#include "serial_port_service.h"

#include "spp_ble_client.h"
#include "inc/sdi_task.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Application events
#define SBC_EVT_KEY_CHANGE          0x01
#define SBC_EVT_SCAN_ENABLED        0x02
#define SBC_EVT_SCAN_DISABLED       0x03
#define SBC_EVT_ADV_REPORT          0x04
#define SBC_EVT_SVC_DISC            0x05
#define SBC_EVT_PAIR_STATE          0x06
#define SBC_EVT_PASSCODE_NEEDED     0x07
#define SBC_EVT_READ_RPA            0x08
#define SBC_EVT_INSUFFICIENT_MEM    0x09
#define SBC_START_DISCOVERY_EVT     0x0A
#define SBC_AUTO_CONNECT_EVT        0x0B
#define SBC_NOTI_ENABLE_EVT         0x0C

// SPP BLE Client Task Events
#define SBC_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define SBC_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define SBC_ALL_EVENTS                        (SBC_ICALL_EVT           | \
                                              SBC_QUEUE_EVT)

// Address mode of the local device
#define DEFAULT_ADDRESS_MODE                  ADDRMODE_PUBLIC

// Default PHY for scanning and initiating
#define DEFAULT_SCAN_PHY                     SCAN_PRIM_PHY_1M
#define DEFAULT_INIT_PHY                     INIT_PHY_1M

// Default scan duration in 10 ms
#define DEFAULT_SCAN_DURATION                100 // 1 sec

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

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

// How often to read current current RPA (in ms)
#define SBC_READ_RPA_PERIOD                    3000

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           3000

// Default notification enable timer delay in ms
#define DEFAULT_NOTI_ENABLE_DELAY             200

// TRUE to connect automatically to server to preset address
#ifndef CLIENT_AUTO_CONNECT
#define CLIENT_AUTO_CONNECT                   TRUE
#endif
// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                   1024
#endif

// Advertising report fields to keep in the list
// Interested in only peer address type and peer address
#define SBC_ADV_RPT_FIELDS   (SCAN_ADVRPT_FLD_ADDRTYPE | SCAN_ADVRPT_FLD_ADDRESS)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define SBC_ADDR_STR_SIZE     15

// Spin if the expression is not true
#define SIMPLECENTRAL_ASSERT(expr) if (!(expr)) SPPBLEClient_spin();

/*********************************************************************
 * TYPEDEFS
 */

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

// RTOS queue for profile/app messages.
typedef struct _queueRec_
{
  Queue_Elem _elem;          // queue element
  uint8_t *pData;            // pointer to app data
} queueRec_t;
// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} sbcEvt_t;

// App event passed from profiles.
typedef struct
{
  uint8_t *pData;  // New data
  uint8_t length; // New status
} sbcUARTEvt_t;

// Scanned device information record
typedef struct
{
  uint8_t addrType;         // Peer Device's Address Type
  uint8_t addr[B_ADDR_LEN]; // Peer Device Address
} scanRec_t;

// Connected device information
typedef struct
{
  uint16_t currentConnHandle;        // Connection Handle
  uint8_t  addr[B_ADDR_LEN];  // Peer Device Address
  uint8_t  charHandle;        // Characteristic Handle
} connRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

// Global pin resources
PIN_State pinGpioState;
PIN_Handle hGpioPin;
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;

// Clock object used to signal timeout
static Clock_Struct startNotiEnableClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Queue object used for UART messages
static Queue_Struct appUARTMsg;
static Queue_Handle appUARTMsgQueue;

// Task configuration
Task_Struct sbcTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(sbcTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t sbcTaskStack[SBC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "SPP BLE Client";

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
// Number of scan results filtered by Service UUID
static uint8_t numScanRes = 0;

// Scan results filtered by Service UUID
static scanRec_t scanList[DEFAULT_MAX_SCAN_RES];
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

// Number of connected devices
static uint8_t numConn = 0;

// List of connections
static connRec_t connList[MAX_NUM_BLE_CONNS];

// Connection handle of current connection
static uint16_t sbcConnHandle = CONNHANDLE_INVALID;

// Accept or reject L2CAP connection parameter update request
static bool acceptParamUpdateReq = true;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charDataHdl = 0;

// Discovered characteristic CCCD handle
static uint16_t charCCCDHdl = 0;

//UUID of Serial Port Data Characteristic
static uint8_t uuidDataChar[ATT_UUID_SIZE] = { TI_BASE_UUID_128(SERIALPORTSERVICE_DATA_UUID) };

// Value to write
static uint8_t charVal = 0x41;

// Maximum PDU size (default = 27 octets)
static uint16_t sbcMaxPduSize;

// Maximum MTU size (default = 23 octets)
static uint16 currentMTUSize;

// Pins that are actively used by the application
static PIN_Config SPPBLEAppPinTable[] =
{
    Board_PIN_RLED       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */
    Board_PIN_GLED       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */

    PIN_TERMINATE
};

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

//Current connection handle
static uint16 currentConnHandle = MAX_NUM_BLE_CONNS;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SPPBLEClient_init(void);
static void SPPBLEClient_taskFxn(uintptr_t a0, uintptr_t a1);

static void SPPBLEClient_handleKeys(uint8_t keys);
static uint8_t SPPBLEClient_processStackMsg(ICall_Hdr *pMsg);
static void SPPBLEClient_processGapMsg(gapEventHdr_t *pMsg);
static void SPPBLEClient_processGATTMsg(gattMsgEvent_t *pMsg);
static void SPPBLEClient_processAppMsg(sbcEvt_t *pMsg);
static void SPPBLEClient_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void SPPBLEClient_startSvcDiscovery(void);
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
static bool SPPBLEClient_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                      uint16_t dataLen);
static void SPPBLEClient_addScanInfo(uint8_t *pAddr, uint8_t addrType);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
static uint8_t SPPBLEClient_addConnInfo(uint16_t currentConnHandle, uint8_t *pAddr);
static uint8_t SPPBLEClient_removeConnInfo(uint16_t currentConnHandle);
static char* SPPBLEClient_getConnAddrStr(uint16_t currentConnHandle);
static void SPPBLEClient_processPairState(uint8_t state, uint8_t status);
static void SPPBLEClient_processPasscode(uint16_t currentConnHandle,
                                          uint8_t uiOutputs);
static void SPPBLEClient_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void SPPBLEClient_passcodeCb(uint8_t *deviceAddr, uint16_t currentConnHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs,
                                     uint32_t numComparison);
static void SPPBLEClient_pairStateCb(uint16_t currentConnHandle, uint8_t state,
                                      uint8_t status);

void SPPBLEClient_startDiscHandler(UArg a0);
static void SPPBLEClient_keyChangeHandler(uint8 keys);
static void SPPBLEClient_clockHandler(UArg arg);

static uint8_t SPPBLEClient_enqueueMsg(uint8_t event, uint8_t status,
                                        uint8_t *pData);

static void SPPBLEClient_scanCb(uint32_t evt, void* msg, uintptr_t arg);

void SPPBLEClient_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len);
static void SPPBLEClient_genericHandler(UArg arg);
static void SPPBLEClient_autoConnect(void);
char* convInt32ToText(int32 value);
/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Bond Manager Callbacks
static gapBondCBs_t bondMgrCBs =
{
  SPPBLEClient_passcodeCb, // Passcode callback
  SPPBLEClient_pairStateCb // Pairing/Bonding state Callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/*********************************************************************
 * @fn      SPPBLEClient_autoConnect
 *
 * @brief   Client automatically connects to hardcoded address
 *
 * @return  none
 */
void SPPBLEClient_autoConnect(void)
{
    uint8_t peerAddr[6];

    // connect to hardcoded device address i.e. 0x050403020100
    int x = 0;
    for(x = 0; x<6; x++)
    {
        peerAddr[x] = x;
    }

    DEBUG("Auto connecting..."); DEBUG_NEWLINE();

    GapScan_Evt_AdvRpt_t advRpt;

    advRpt.addrType = ADDRTYPE_PUBLIC;

    memcpy(advRpt.addr, peerAddr, 6);

    GapInit_setPhyParam(INIT_PHY_1M|INIT_PHY_2M, INIT_PHYPARAM_CONN_INT_MIN, 20);
    GapInit_setPhyParam(INIT_PHY_1M|INIT_PHY_2M, INIT_PHYPARAM_CONN_INT_MAX, 20);

    GapInit_connect(advRpt.addrType & MASK_ADDRTYPE_ID,
                    advRpt.addr, DEFAULT_INIT_PHY, 0);

}


/*******************************************************************************
 * @fn      SPPBLEClient_blinkLed
 *
 * @brief   Blinks a led 'n' times, duty-cycle 50-50
 * @param   led - led identifier
 * @param   nBlinks - number of blinks
 *
 * @return  none
 */
void SPPBLEClient_blinkLed(uint8_t led, uint8_t nBlinks)
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
 * @fn      SPPBLEClient_toggleLed
 *
 * @brief   Toggle an LED
 * @param   led - led identifier
 * @param   state - state to change the LED
 *
 * @return  none
 */
void SPPBLEClient_toggleLed(uint8_t led, uint8_t state)
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
 * @fn      SPPBLEClient_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void SPPBLEClient_spin(void)
{
  volatile uint8_t x;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_createTask
 *
 * @brief   Task creation function for the SPP BLE Client.
 *
 * @param   none
 *
 * @return  none
 */
void SPPBLEClient_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbcTaskStack;
  taskParams.stackSize = SBC_TASK_STACK_SIZE;
  taskParams.priority = SBC_TASK_PRIORITY;

  Task_construct(&sbcTask, SPPBLEClient_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SPPBLEClient_Init
 *
 * @brief   Initialization function for the SPP BLE Client App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void SPPBLEClient_init(void)
{
  uint8_t i;

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  hGpioPin = PIN_open(&pinGpioState, SPPBLEAppPinTable);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  appUARTMsgQueue = Util_constructQueue(&appUARTMsg);

  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, SPPBLEClient_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);

  Util_constructClock(&startNotiEnableClock, SPPBLEClient_genericHandler,
                      DEFAULT_NOTI_ENABLE_DELAY, 0, false, SBC_NOTI_ENABLE_EVT);

  Board_initKeys(SPPBLEClient_keyChangeHandler);

  // Initialize internal data
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    connList[i].currentConnHandle = CONNHANDLE_INVALID;
  }

  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void *)attDeviceName);

  //Set default values for Data Length Extension
  //Extended Data Length Feature is already enabled by default
  //in build_config.opt in stack project.
  {
    //Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    #define APP_SUGGESTED_PDU_SIZE 27 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 328 //default is 328us(TX)

    //This API is documented in hci.h
    //See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    //http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  //Register to receive UART messages
  SDITask_registerIncomingRXEventAppCB(SPPBLEClient_enqueueUARTMsg);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set Bond Manager parameters
  {
    // Don't send a pairing request after connecting; the device waits for the
    // application to start pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    // Do not use authenticated pairing
    uint8_t mitm = FALSE;
    // This is a display only device
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    // Create a bond during the pairing process
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Start Bond Manager and register callback
  // This must be done before initialing the GAP layer
  VOID GAPBondMgr_Register(&bondMgrCBs);

  // Accept all parameter update requests
  GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, GAP_UPDATE_REQ_ACCEPT_ALL);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Initialize GAP layer for Central role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_CENTRAL, selfEntity, addrMode, NULL);

  //Display project name and Bluetooth 5 support
  uint8_t hello[] = "Hello from SPP BLE Client! With Bluetooth 5 support!\n\r";
  DEBUG(hello);

  //Blink twice for client
  SPPBLEClient_blinkLed(Board_PIN_GLED, 2);

}

/*********************************************************************
 * @fn      SPPBLEClient_taskFxn
 *
 * @brief   Application task entry point for the SPP BLE Client.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void SPPBLEClient_taskFxn(uintptr_t a0, uintptr_t a1)
{
  // Initialize application
  SPPBLEClient_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, SBC_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = SPPBLEClient_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message
      if (events & SBC_QUEUE_EVT)
      {
        sbcEvt_t *pMsg;

        // If RTOS queue is not empty, process app UART message.
        if (!Queue_empty(appUARTMsgQueue))
        {
          //Get the message at the front of the queue but still keep it in the queue
          queueRec_t *pRec = Queue_head(appUARTMsgQueue);
          sbcUARTEvt_t *pMsg = (sbcUARTEvt_t *)pRec->pData;

          if (pMsg && (linkDB_Up(currentConnHandle)))
          {
            // Process message.
            bStatus_t retVal = FAILURE;

            // Do a write
            attWriteReq_t req;

            // Allocate data bytes to send over the air
            req.pValue = GATT_bm_alloc(currentConnHandle, ATT_WRITE_REQ, pMsg->length, NULL);

            if ( (req.pValue != NULL) && charDataHdl)
            {
              req.handle = charDataHdl; //handle for Value of Data characteristic found during service discovery
              req.len = pMsg->length;
              memcpy(req.pValue, pMsg->pData, pMsg->length);
              req.sig = FALSE;
              req.cmd = TRUE;

              // Send data received from UART terminal using Write No Response over the air
              retVal = GATT_WriteNoRsp(currentConnHandle, &req);

              if ( retVal != SUCCESS )
              {
                GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
                DEBUG("FAIL FROM CLIENT: "); DEBUG((uint8_t*)convInt32ToText((int)retVal)); DEBUG_NEWLINE();
              }
              else
              {
                //Remove from the queue
                Util_dequeueMsg(appUARTMsgQueue);

                // Deallocate data payload being transmitted.
                ICall_freeMsg(pMsg->pData);
                // Free the space from the message.
                ICall_free(pMsg);

                if(!Queue_empty(appUARTMsgQueue))
                {
                  // Wake up the application to flush out any remaining UART data in the queue.
                  Event_post(syncEvent, SBC_QUEUE_EVT);
                }
              }
            }else
            {
              DEBUG("Allocation ERROR!");
            }
          }
        }

        while (pMsg = (sbcEvt_t *)Util_dequeueMsg(appMsgQueue))
        {
          // Process message
          SPPBLEClient_processAppMsg(pMsg);

          // Free the space from the message
          ICall_free(pMsg);
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SPPBLEClient_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SPPBLEClient_processGapMsg((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      SPPBLEClient_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch (pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
          SPPBLEClient_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;

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
                  if (pMyMsg->cmdStatus ==
                      HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
                  {
                    Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0,
                            "PHY Change failure, peer does not support this");
                  }
                  else
                  {
                    Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0,
                                   "PHY Update Status: 0x%02x",
                                   pMyMsg->cmdStatus);
                  }
                }
                break;
              case HCI_DISCONNECT:
                break;

              default:
                {
                  Display_printf(dispHandle, SBC_ROW_NON_CONN, 0,
                                 "Unknown Cmd Status: 0x%04x::0x%02x",
                                 pMyMsg->cmdOpcode, pMyMsg->cmdStatus);
                }
              break;
            }
          }
          break;

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC
            = (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

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
          }

          break;
        }

        default:
          break;
      }

      break;
    }

    case L2CAP_SIGNAL_EVENT:
      // place holder for L2CAP Connection Parameter Reply
      break;

    default:
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SPPBLEClient_processAppMsg
 *
 * @brief   Scanner application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SPPBLEClient_processAppMsg(sbcEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
  case SBC_NOTI_ENABLE_EVT:
    {
      // Process message.
      uint8 retVal;
      attWriteReq_t req;
      uint8 configData[2] = {0x01,0x00};

      req.pValue = GATT_bm_alloc(currentConnHandle, ATT_WRITE_REQ, 2, NULL);

      if ((charCCCDHdl == NULL) && (charDataHdl != NULL)) {charCCCDHdl = charDataHdl + 1;} //Hardcoded
      if ( (req.pValue != NULL) && charCCCDHdl)
      {
        req.handle = charCCCDHdl; //Handle for CCCD of Data characteristic
        req.len = 2;
        memcpy(req.pValue, configData, 2);
        req.cmd = TRUE; //Has to be true for NoRsp from server(command, not request)
        req.sig = FALSE;
        retVal = GATT_WriteNoRsp(currentConnHandle, &req);
        if (retVal != SUCCESS)
        {
          DEBUG("ERROR enabling notification...\n\r");
        }
        else
        {
          DEBUG("Notification enabled...\n\r");
        }
      }
      break;
    }

    case SBC_AUTO_CONNECT_EVT:
      SPPBLEClient_autoConnect();
      break;

    case SBC_EVT_KEY_CHANGE:
      SPPBLEClient_handleKeys(pMsg->hdr.state);
      break;

    case SBC_EVT_ADV_REPORT:
    {
      GapScan_Evt_AdvRpt_t* pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      if (SPPBLEClient_findSvcUuid(SERIALPORTSERVICE_SERV_UUID,
                                    pAdvRpt->pData, pAdvRpt->dataLen))
      {
        SPPBLEClient_addScanInfo(pAdvRpt->addr, pAdvRpt->addrType);
        Display_printf(dispHandle, SBC_ROW_NON_CONN, 0, "Discovered: %s",
                       Util_convertBdAddr2Str(pAdvRpt->addr));
      }
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
      Display_printf(dispHandle, SBC_ROW_NON_CONN, 0, "Discovered: %s",
                     Util_convertBdAddr2Str(pAdvRpt->addr));
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

      if (pAdvRpt->pData != NULL)
      {
        ICall_free(pAdvRpt->pData);
      }
      ICall_free(pAdvRpt);

      break;
    }

    case SBC_EVT_SCAN_ENABLED:
      Display_printf(dispHandle, SBC_ROW_NON_CONN, 0, "Discovering...");
      break;

    case SBC_EVT_SCAN_DISABLED:
    {
      uint8_t numReport;
      uint8_t i;
      static uint8_t* pAddrs = NULL;
      uint8_t* pAddrTemp;
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      numReport = numScanRes;
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
      GapScan_Evt_AdvRpt_t advRpt;

      numReport = ((GapScan_Evt_End_t*) (pMsg->pData))->numReport;
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
      ICall_free(pMsg);

      Display_printf(dispHandle, SBC_ROW_NON_CONN, 0,
                     "%d devices discovered", numReport);

      if (numReport > 0)
      {

      }

      if (numConn > 0)
      {

      }

      if (pAddrs != NULL)
      {
        ICall_free(pAddrs);
      }

      // Allocate buffer to display addresses
      pAddrs = ICall_malloc(numReport * SBC_ADDR_STR_SIZE);

      if (pAddrs == NULL)
      {
        numReport = 0;
      }

      pAddrTemp = pAddrs;
      for (i = 0; i < numReport; i++, pAddrTemp += SBC_ADDR_STR_SIZE)
      {
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        // Get the address from the list, convert it to string, and
        // copy the string to the address buffer
        memcpy(pAddrTemp, Util_convertBdAddr2Str(scanList[i].addr),
               SBC_ADDR_STR_SIZE);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
        // Get the address from the report, convert it to string, and
        // copy the string to the address buffer
        GapScan_getAdvReport(i, &advRpt);
        memcpy(pAddrTemp, Util_convertBdAddr2Str(advRpt.addr),
               SBC_ADDR_STR_SIZE);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

      }

      break;
    }

    case SBC_EVT_SVC_DISC:
      SPPBLEClient_startSvcDiscovery();
      break;

    // Pairing event
    case SBC_EVT_PAIR_STATE:
    {
      SPPBLEClient_processPairState(pMsg->hdr.state, *pMsg->pData);

      ICall_free(pMsg->pData);
      break;
    }

    // Passcode event
    case SBC_EVT_PASSCODE_NEEDED:
    {
      SPPBLEClient_processPasscode(sbcConnHandle, *pMsg->pData);

      ICall_free(pMsg->pData);
      break;
    }

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
    case SBC_EVT_READ_RPA:
    {
      // Read the current RPA.
      // The parameters for the call to HCI_LE_ReadLocalResolvableAddressCmd
      // are not needed to be accurate to retrieve the local resolvable address.
      // The 1st parameter can be any of ADDRMODE_PUBLIC and ADDRMODE_RANDOM.
      // The 2nd parameter only has to be not NULL.
      // The result will come with HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS
      // complete event.
      HCI_LE_ReadLocalResolvableAddressCmd(0, rpa);
      break;
    }
#endif // PRIVACY_1_2_CFG

    // Insufficient memory
    case SBC_EVT_INSUFFICIENT_MEM:
    {
      // We are running out of memory.
      //Display_printf(dispHandle, SBC_ROW_ANY_CONN, 0, "Insufficient Memory");
      DEBUG("Insufficient Memory");
      // We might be in the middle of scanning, try stopping it.
      GapScan_disable();
      break;
    }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_processGapMsg
 *
 * @brief   GAP message processing function.
 *
 * @param   pMsg - pointer to event message structure
 *
 * @return  none
 */
static void SPPBLEClient_processGapMsg(gapEventHdr_t *pMsg)
{
  switch (pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      uint8_t temp8;
      uint16_t temp16;
      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      // Setup scanning
      // For more information, see the GAP section in the User's Guide:
      // http://software-dl.ti.com/lprf/ble5stack-latest/

      // Register callback to process Scanner events
      GapScan_registerCb(SPPBLEClient_scanCb, NULL);

      // Set Scanner Event Mask
      GapScan_setEventMask(GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED |
                           GAP_EVT_ADV_REPORT);

      // Set Scan PHY parameters
      GapScan_setPhyParams(DEFAULT_SCAN_PHY, SCAN_TYPE_ACTIVE,
                           SCAN_PARAM_DFLT_INTERVAL, SCAN_PARAM_DFLT_INTERVAL);

      // Set Advertising report fields to keep
      temp16 = SBC_ADV_RPT_FIELDS;
      GapScan_setParam(SCAN_PARAM_RPT_FIELDS, &temp16);
      // Set Scanning Primary PHY
      temp8 = DEFAULT_SCAN_PHY;
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

      sbcMaxPduSize = pPkt->dataPktLen;

      DEBUG("Initialized...");

#if defined (CLIENT_AUTO_CONNECT) && (CLIENT_AUTO_CONNECT == TRUE)
      SPPBLEClient_enqueueMsg(SBC_AUTO_CONNECT_EVT, 0, NULL);
#endif

      // Display device address
      Display_printf(dispHandle, SBC_ROW_IDA, 0, "%s Addr: %s",
                     (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
                     Util_convertBdAddr2Str(pPkt->devAddr));

      // Display the address of this device
      DEBUG("BD ADDR: "); DEBUG((uint8_t*)Util_convertBdAddr2Str(pPkt->devAddr));
      DEBUG_NEWLINE();

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
      if (addrMode > ADDRMODE_RANDOM)
      {
        // If the local device is supposed to use privacy,
        // read the current RPA and display.
        // The parameters for the call to HCI_LE_ReadLocalResolvableAddressCmd
        // don't have to be accurate to retrieve the local resolvable address.
        // The 1st parameter can be any of ADDRMODE_PUBLIC and ADDRMODE_RANDOM.
        // The 2nd parameter only has to be not NULL.
        // The result will come with HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS
        // complete event.
        HCI_LE_ReadLocalResolvableAddressCmd(0, rpa);

        // Create one-shot clock for RPA check event.
        Util_constructClock(&clkRpaRead, SPPBLEClient_clockHandler,
                            SBC_READ_RPA_PERIOD, 0, true, SBC_EVT_READ_RPA);
      }
#endif // PRIVACY_1_2_CFG
      break;
    }

    case GAP_CONNECTING_CANCELLED_EVENT:
    {
      if (numConn > 0)
      {

      }

      Display_printf(dispHandle, SBC_ROW_NON_CONN, 0,
                     "Conneting attempt cancelled");

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      uint16_t connHandle = ((gapEstLinkReqEvent_t*) pMsg)->connectionHandle;
      uint8_t* pAddr = ((gapEstLinkReqEvent_t*) pMsg)->devAddr;
      uint8_t  connIndex;
      uint8_t* pStrAddr;

      // Add this connection info to the list
      currentConnHandle = connHandle;
      connIndex = SPPBLEClient_addConnInfo(connHandle, pAddr);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      SIMPLECENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      connList[connIndex].charHandle = 0;

      pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

      Display_printf(dispHandle, SBC_ROW_NON_CONN, 0, "Connected to %s", pStrAddr);
      Display_printf(dispHandle, SBC_ROW_NUM_CONN, 0, "Num Conns: %d", numConn);

      DEBUG("Connected to:"); DEBUG(pStrAddr);
      DEBUG_NEWLINE();

      // Toggle LED to indicate connection status
      SPPBLEClient_toggleLed(Board_PIN_GLED, Board_LED_TOGGLE);

      // If service discovery not performed initiate service discovery
      if (charDataHdl == 0)
      {
        Util_startClock(&startDiscClock);
      }

      // If we already have maximum allowed number of connections,
      // disable device discovery and additional connection making.
      if (numConn >= MAX_NUM_BLE_CONNS)
      {

      }

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      uint16_t currentConnHandle = ((gapTerminateLinkEvent_t*) pMsg)->connectionHandle;
      uint8_t connIndex;

      // Mark this connection deleted in the connected device list.
      connIndex = SPPBLEClient_removeConnInfo(currentConnHandle);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      SIMPLECENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      DEBUG("Disconnected");

      //Toggle LED to indicate connection status
      SPPBLEClient_toggleLed(Board_PIN_GLED, Board_LED_TOGGLE);

      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
      if (acceptParamUpdateReq)
      {
        gapUpdateLinkParamReqReply_t rsp;
        gapUpdateLinkParamReq_t *pReq;

        pReq = &((gapUpdateLinkParamReqEvent_t *)pMsg)->req;

        rsp.connectionHandle = pReq->connectionHandle;
        rsp.intervalMin = pReq->intervalMin;
        rsp.intervalMax = pReq->intervalMax;
        rsp.connLatency = pReq->connLatency;
        rsp.connTimeout = pReq->connTimeout;
        rsp.accepted = TRUE;

        // Send application's requested parameters back.
        VOID GAP_UpdateLinkParamReqReply(&rsp);
      }
      else
      {
        gapUpdateLinkParamReqReply_t rsp;
        gapUpdateLinkParamReq_t *pReq;

        pReq = &((gapUpdateLinkParamReqEvent_t *)pMsg)->req;

        rsp.connectionHandle = pReq->connectionHandle;
        rsp.accepted = FALSE;

        // Reject the request.
        VOID GAP_UpdateLinkParamReqReply(&rsp);
      }

      break;

     case GAP_LINK_PARAM_UPDATE_EVENT:
     {
       gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

        // Get the address from the connection handle
        linkDBInfo_t linkInfo;
        if (linkDB_GetInfo(pPkt->connectionHandle, &linkInfo) ==  SUCCESS)
        {

          if(pPkt->status == SUCCESS)
          {
            Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0,
                          "Updated: %s, connTimeout:%d",
                           Util_convertBdAddr2Str(linkInfo.addr),
                           linkInfo.connTimeout);
          }
          else
          {
            // Display the address of the connection update failure
            Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0,
                           "Update Failed 0x%h: %s", pPkt->opcode,
                           Util_convertBdAddr2Str(linkInfo.addr));
          }
        }
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 *
 * @return  none
 */
static void SPPBLEClient_handleKeys(uint8_t keys)
{
  static uint8_t index = 0;

  if (keys & KEY_LEFT)
  {
    // Check if the key is still pressed. Workaround for possible bouncing.
    if (PIN_getInputValue(Board_PIN_BUTTON0) == 0)
    {
      uint8_t status;

      SPPBLEClient_toggleLed(Board_PIN_RLED, Board_LED_TOGGLE);

      if(linkDB_Up(currentConnHandle))
      {
        // Do a write
        attWriteReq_t req;

        charVal = 0x41;
#define NUM_CHARS 42
        req.pValue = GATT_bm_alloc(currentConnHandle, ATT_WRITE_REQ, NUM_CHARS, NULL);
        if ( req.pValue != NULL )
        {
          req.handle = charDataHdl;
          req.len = NUM_CHARS;
          for(int x =0; x< NUM_CHARS; x++)
            req.pValue[x] = charVal++;
          req.sig = 0;
          req.cmd = 1;

          status = GATT_WriteNoRsp(currentConnHandle, &req);
          if ( status != SUCCESS )
          {
            GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
          }
        }
        else
        {
          status = bleMemAllocError;
        }
      }
    }
  }
  else if (keys & KEY_RIGHT)
  {
    // Check if the key is still pressed. Workaround for possible bouncing.
    if (PIN_getInputValue(Board_PIN_BUTTON1) == 0)
    {
      SPPBLEClient_toggleLed(Board_PIN_GLED, Board_LED_TOGGLE);

      if(linkDB_Up(currentConnHandle))
      {
        switch(index)
        {
        case 0:
          DEBUG("Changing PHY to 2M...");
          index = 1; // 2M
          SPPBLEClient_doSetConnPhy(index);
          break;
        case 1:
          DEBUG("Changing PHY to Coded...");
          index = 3; // Coded
          SPPBLEClient_doSetConnPhy(index);
          break;
        case 3:
#define REQUESTED_PDU_SIZE 27
#define REQUESTED_TX_TIME 328
          DEBUG("Changing Data Length Extension..."); DEBUG_NEWLINE();
          index = 4; // Default to 1M
          HCI_LE_SetDataLenCmd(currentConnHandle, REQUESTED_PDU_SIZE, REQUESTED_TX_TIME);
          break;
        default:
          DEBUG("Changing PHY to 1M...");
          index = 0; // 1M
          SPPBLEClient_doSetConnPhy(index);
          break;
        }
      }
    }
  }
}
/*********************************************************************
 * @fn      SPPBLEClient_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void SPPBLEClient_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (linkDB_Up(pMsg->connHandle))
  {
    if(pMsg->method == ATT_HANDLE_VALUE_NOTI)
    {
      // Send received bytes over the air to UART terminal
      SDITask_sendToUART(pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len);
    }
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0,
                     "ATT Rsp dropped %d", pMsg->method);
    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0,
                       "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0,
                       "Read rsp: 0x%02x", pMsg->msg.readRsp.pValue[0]);
      }
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0,
                       "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful write, display the value that was written and
        // increment value
        Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0,
                       "Write sent: 0x%02x", charVal);
      }

    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.

      // Display the opcode of the message that caused the violation.
      Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0,
                     "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      currentMTUSize = pMsg->msg.mtuEvt.MTU;
      SDITask_setAppDataSize(currentMTUSize);

      DEBUG("MTU Size: "); DEBUG((uint8_t*)convInt32ToText((int)currentMTUSize)); DEBUG_NEWLINE();

    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      SPPBLEClient_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.

  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      SPPBLEClient_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SPPBLEClient_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
  case HCI_READ_RSSI:
    {
      uint16_t currentConnHandle = BUILD_UINT16(pMsg->pReturnParam[1],
                                                pMsg->pReturnParam[2]);
      int8 rssi = (int8)pMsg->pReturnParam[3];

      DEBUG((uint8_t*)SPPBLEClient_getConnAddrStr(currentConnHandle));
      DEBUG(":RSSI (-dBm): ");
      DEBUG((uint8_t*)convInt32ToText((int)rssi));
      DEBUG_NEWLINE();
      break;
    }

  case HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS:
    {
      uint8_t* pRpaNew = &(pMsg->pReturnParam[1]);

      if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
      {
        // If the RPA has changed, update the display
        Display_printf(dispHandle, SBC_ROW_RPA, 0, "RP Addr: %s",
                       Util_convertBdAddr2Str(pRpaNew));
        memcpy(rpa, pRpaNew, B_ADDR_LEN);
      }
      break;
    }

  case HCI_LE_SET_DATA_LENGTH:
    HCI_LE_ReadMaxDataLenCmd();
    break;

  case HCI_LE_READ_MAX_DATA_LENGTH:
    DEBUG("Max TX bytes: ");
    DEBUG((uint8_t*)convInt32ToText((int)pMsg->pReturnParam[1] + (pMsg->pReturnParam[2]<<8))); DEBUG_NEWLINE();
    DEBUG("Max TX time: ");
    DEBUG((uint8_t*)convInt32ToText((int)pMsg->pReturnParam[3] + (pMsg->pReturnParam[4]<<8))); DEBUG_NEWLINE();
    DEBUG("Max RX bytes: ");
    DEBUG((uint8_t*)convInt32ToText((int)pMsg->pReturnParam[5] + (pMsg->pReturnParam[6]<<8))); DEBUG_NEWLINE();
    DEBUG("Max RX time: ");
    DEBUG((uint8_t*)convInt32ToText((int)pMsg->pReturnParam[7] + (pMsg->pReturnParam[8]<<8))); DEBUG_NEWLINE();
    break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SPPBLEClient_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0, "Pairing success");
    }
    else
    {
      Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0, "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_ENCRYPTED)
  {
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0, "Encryption success");
    }
    else
    {
      Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0, "Encryption failed: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0, "Bond save success");
    }
    else
    {
      Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0, "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SPPBLEClient_processPasscode(uint16_t currentConnHandle,
                                          uint8_t uiOutputs)
{
  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0, "Passcode: %d",
                   B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(currentConnHandle , SUCCESS, B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      SPPBLEClient_startSvcDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void SPPBLEClient_startSvcDiscovery(void)
{
  uint8 stat;

  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = 0;

  discState = BLE_DISC_STATE_MTU;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = sbcMaxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  stat =  GATT_ExchangeMTU(currentConnHandle, &req, selfEntity);
  if(stat != SUCCESS)
  {
    DEBUG("GATT_ExchangeMTU Fail:")
    DEBUG((uint8_t*)convInt32ToText((int)stat));
    DEBUG_NEWLINE();
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void SPPBLEClient_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  if (discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover simple service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_UUID_SIZE] = { TI_BASE_UUID_128(SERIALPORTSERVICE_SERV_UUID) };

      discState = BLE_DISC_STATE_SVC;

      DEBUG("Discovering services..."); DEBUG_NEWLINE();

      // Discovery simple BLE service
      VOID GATT_DiscPrimaryServiceByUUID(currentConnHandle, uuid, ATT_UUID_SIZE,
                                         selfEntity);
    }
  }
  else if (discState == BLE_DISC_STATE_SVC)
  {
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      DEBUG("Found Serial Port Service...");
    }

    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;
        uint8_t uuid[ATT_UUID_SIZE] = { TI_BASE_UUID_128(SERIALPORTSERVICE_DATA_UUID) };

        // Discover characteristic
        discState = BLE_DISC_STATE_CHAR;

        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_UUID_SIZE;
        memcpy(req.type.uuid,  uuid, ATT_UUID_SIZE);

        // Discover characteristic descriptors
        GATT_DiscAllCharDescs(currentConnHandle,
                              svcStartHdl + 1,
                              svcEndHdl,
                              selfEntity);
      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic descriptors found
    if (pMsg->method == ATT_FIND_INFO_RSP &&
        pMsg->msg.findInfoRsp.numInfo > 0)
    {
      uint8_t i;

      // For each handle/uuid pair
      for (i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++)
      {
        if(pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE)
        {
          // Look for CCCD
          if (ATT_BT_PAIR_UUID(pMsg->msg.findInfoRsp.pInfo, i) ==
              GATT_CLIENT_CHAR_CFG_UUID)
          {
            // CCCD found
            DEBUG("CCCD for Data Char Found..."); DEBUG_NEWLINE();
            charCCCDHdl = ATT_PAIR_HANDLE(pMsg->msg.findInfoRsp.pInfo, i);
            break;
          }
        }
        else if(pMsg->msg.findInfoRsp.format == ATT_HANDLE_UUID_TYPE)
        {
          // Look for Serial Data Char.
          if (memcmp(&(pMsg->msg.findInfoRsp.pInfo[ATT_PAIR_UUID_IDX(i)]), uuidDataChar, ATT_UUID_SIZE) == 0)
          {
            // CCCD found
            DEBUG("Data Char Found...");
            charDataHdl = ATT_PAIR_HANDLE(pMsg->msg.findInfoRsp.pInfo, i);
            break;
          }
        }
      }
    }


    // If procedure complete
    if ((pMsg->method == ATT_FIND_INFO_RSP  &&
         pMsg->hdr.status == bleProcedureComplete) ||
        (pMsg->method == ATT_ERROR_RSP))
    {

      //Enable notification on peripheral(after a few seconds delay, let it finish connection/discovery process)
      {
        Util_startClock(&startNotiEnableClock);
      }

      discState = BLE_DISC_STATE_IDLE;
    }
  }
}



#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
/*********************************************************************
 * @fn      SPPBLEClient_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool SPPBLEClient_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                      uint16_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  if (dataLen > 0)
  {
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
            (adType == GAP_ADTYPE_16BIT_COMPLETE))
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
  }

  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      SPPBLEClient_addScanInfo
 *
 * @brief   Add a device to the scanned device list
 *
 * @return  none
 */
static void SPPBLEClient_addScanInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;

  // If result count not at max
  if (numScanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < numScanRes; i++)
    {
      if (memcmp(pAddr, scanList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(scanList[numScanRes].addr, pAddr, B_ADDR_LEN);
    scanList[numScanRes].addrType = addrType;

    // Increment scan result count
    numScanRes++;
  }
}
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

/*********************************************************************
 * @fn      SPPBLEClient_addConnInfo
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SPPBLEClient_addConnInfo(uint16_t currentConnHandle, uint8_t *pAddr)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].currentConnHandle == CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].currentConnHandle = currentConnHandle;
      memcpy(connList[i].addr, pAddr, B_ADDR_LEN);
      numConn++;

      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      SPPBLEClient_removeConnInfo
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if currentConnHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SPPBLEClient_removeConnInfo(uint16_t currentConnHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].currentConnHandle == currentConnHandle)
    {
      // Found the entry to mark as deleted
      connList[i].currentConnHandle = CONNHANDLE_INVALID;
      numConn--;

      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      SPPBLEClient_getConnAddrStr
 *
 * @brief   Return, in string form, the address of the peer associated with
 *          the currentConnHandle.
 *
 * @return  A null-terminated string of the address.
 *          if there is no match, NULL will be returned.
 */
static char* SPPBLEClient_getConnAddrStr(uint16_t currentConnHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].currentConnHandle == currentConnHandle)
    {
      return Util_convertBdAddr2Str(connList[i].addr);
    }
  }

  return NULL;
}

/*********************************************************************
 * @fn      SPPBLEClient_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SPPBLEClient_pairStateCb(uint16_t currentConnHandle, uint8_t state,
                                      uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    SPPBLEClient_enqueueMsg(SBC_EVT_PAIR_STATE, state, pData);
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SPPBLEClient_passcodeCb(uint8_t *deviceAddr, uint16_t currentConnHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs,
                                     uint32_t numComparison)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    SPPBLEClient_enqueueMsg(SBC_EVT_PASSCODE_NEEDED, 0, pData);
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_genericHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SPPBLEClient_genericHandler(UArg arg)
{
  // Wake up the application.
  SPPBLEClient_enqueueMsg(arg, 0, NULL);
}

/*********************************************************************
 * @fn      SPPBLEClient_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SPPBLEClient_startDiscHandler(UArg a0)
{
  // Initiate service discovery
  SPPBLEClient_enqueueMsg(SBC_EVT_SVC_DISC, 0, NULL);
}

/*********************************************************************
 * @fn      SPPBLEClient_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SPPBLEClient_keyChangeHandler(uint8 keys)
{
  SPPBLEClient_enqueueMsg(SBC_EVT_KEY_CHANGE, keys, NULL);
}

/*********************************************************************
 * @fn      SPPBLEClient_enqueueUARTMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   data - message data pointer.
 * @param   len - message length.
 *
 * @return  TRUE or FALSE
 */
void SPPBLEClient_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len)
{
  sbcUARTEvt_t *pMsg;

  // Enqueue message to be sent over the air
  {
    // Create dynamic pointer to message.
    if (pMsg = ICall_malloc(sizeof(sbcUARTEvt_t)))
    {

      pMsg->pData = (uint8 *)ICall_allocMsg(len);
      if(pMsg->pData)
      {
        // payload
        memcpy(pMsg->pData , data, len);
      }
      pMsg->length = len;

      // Enqueue the message.
      Util_enqueueMsg(appUARTMsgQueue, syncEvent, (uint8_t *)pMsg);
    }
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_clockHandler
 *
 * @brief   clock handler function
 *
 * @param   arg - argument from the clock initiator
 *
 * @return  none
 */
void SPPBLEClient_clockHandler(UArg arg)
{
  uint8_t evtId = (uint8_t) (arg & 0xFF);

  switch (evtId)
  {
    case SBC_EVT_READ_RPA:
      // Restart timer
      Util_startClock(&clkRpaRead);
      // Let the application handle the event
      SPPBLEClient_enqueueMsg(SBC_EVT_READ_RPA, 0, NULL);
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SPPBLEClient_enqueueMsg(uint8_t event, uint8_t state,
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
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return (false);
}

/*********************************************************************
 * @fn      SPPBLEClient_scanCb
 *
 * @brief   Callback called by GapScan module
 *
 * @param   evt - event
 * @param   msg - message coming with the event
 * @param   arg - user argument
 *
 * @return  none
 */
void SPPBLEClient_scanCb(uint32_t evt, void* pMsg, uintptr_t arg)
{
  if (evt & GAP_EVT_ADV_REPORT)
  {
    SPPBLEClient_enqueueMsg(SBC_EVT_ADV_REPORT, SUCCESS, pMsg);
  }
  else if (evt & GAP_EVT_SCAN_ENABLED)
  {
    SPPBLEClient_enqueueMsg(SBC_EVT_SCAN_ENABLED, SUCCESS, pMsg);
  }
  else if (evt & GAP_EVT_SCAN_DISABLED)
  {
    SPPBLEClient_enqueueMsg(SBC_EVT_SCAN_DISABLED, SUCCESS, pMsg);
  }
  else if (evt & GAP_EVT_INSUFFICIENT_MEMORY)
  {
    SPPBLEClient_enqueueMsg(SBC_EVT_INSUFFICIENT_MEM, SUCCESS, pMsg);
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_doSetScanPhy
 *
 * @brief   Set PHYs for scanning.
 *
 * @param   index - 0: 1M PHY
 *                  1: CODED PHY (Long range)
 *
 * @return  always true
 */
bool SPPBLEClient_doSetScanPhy(uint8_t index)
{
  uint8_t temp8;

  if (index == 0)
  {
    temp8 = SCAN_PRIM_PHY_1M;
  }
  else
  {
    temp8 = SCAN_PRIM_PHY_CODED;
  }

  // Set scanning primary PHY
  GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);

  return (true);
}

/*********************************************************************
 * @fn      SPPBLEClient_doDiscoverDevices
 *
 * @brief   Enables scanning
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SPPBLEClient_doDiscoverDevices(uint8_t index)
{
  (void) index;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // The stack does not need to record advertising reports
  // since the application will filter them by Service UUID and save.

  // Reset number of scan results to 0 before starting scan
  numScanRes = 0;
  GapScan_enable(0, DEFAULT_SCAN_DURATION, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // Let the stack record the advertising reports as many as up to DEFAULT_MAX_SCAN_RES.
  GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

  return (true);
}

/*********************************************************************
 * @fn      SPPBLEClient_doStopDiscovering
 *
 * @brief   Stop on-going scanning
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SPPBLEClient_doStopDiscovering(uint8_t index)
{
  (void) index;

  GapScan_disable();

  return (true);
}

/*********************************************************************
 * @fn      SPPBLEClient_doEstablishLink
 *
 * @brief   Establish a link to a peer device
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SPPBLEClient_doConnect(uint8_t index)
{
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  GapInit_connect(scanList[index].addrType & MASK_ADDRTYPE_ID,
                  scanList[index].addr, DEFAULT_INIT_PHY, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  GapScan_Evt_AdvRpt_t advRpt;

  GapScan_getAdvReport(index, &advRpt);

  GapInit_connect(advRpt.addrType & MASK_ADDRTYPE_ID,
                  advRpt.addr, DEFAULT_INIT_PHY, 0);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

  Display_printf(dispHandle, SBC_ROW_NON_CONN, 0, "Connecting...");

  return (true);
}

/*********************************************************************
 * @fn      SPPBLEClient_doCancelConnecting
 *
 * @brief   Cancel on-going connection attempt
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SPPBLEClient_doCancelConnecting(uint8_t index)
{
  (void) index;

  GapInit_cancelConnect();

  return (true);
}

/*********************************************************************
 * @fn      SPPBLEClient_doConnUpdate
 *
 * @brief   Initiate Connection Update procedure
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SPPBLEClient_doConnUpdate(uint8_t index)
{
  gapUpdateLinkParamReq_t params;

  (void) index;

  params.connectionHandle = sbcConnHandle;
  params.intervalMin = DEFAULT_UPDATE_MIN_CONN_INTERVAL;
  params.intervalMax = DEFAULT_UPDATE_MAX_CONN_INTERVAL;
  params.connLatency = DEFAULT_UPDATE_SLAVE_LATENCY;

  linkDBInfo_t linkInfo;
  if (linkDB_GetInfo(sbcConnHandle, &linkInfo) == SUCCESS)
  {
    if (linkInfo.connTimeout == DEFAULT_UPDATE_CONN_TIMEOUT)
    {
      params.connTimeout = DEFAULT_UPDATE_CONN_TIMEOUT + 200;
    }
    else
    {
      params.connTimeout = DEFAULT_UPDATE_CONN_TIMEOUT;
    }
  }
  else
  {
    Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0,
                   "update :%s, Unable to find link information",
                    Util_convertBdAddr2Str(linkInfo.addr));
  }
  GAP_UpdateLinkParamReq(&params);

  Display_printf(dispHandle, SBC_ROW_CUR_CONN, 0, "Param update Request:connTimeout =%d",
                 params.connTimeout);

  return (true);
}

/*********************************************************************
 * @fn      SPPBLEClient_doSetConnPhy
 *
 * @brief   Set Connection PHY preference.
 *
 * @param   index - 0: 1M PHY
 *                  1: 2M PHY
 *                  2: 1M + 2M PHY
 *                  3: CODED PHY (Long range)
 *                  4: 1M + 2M + CODED PHY
 *
 * @return  always true
 */
bool SPPBLEClient_doSetConnPhy(uint8_t index)
{
  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS | HCI_PHY_CODED,
  };

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX. For more information, see the LE 2M PHY section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  // Note PHYs are already enabled by default in build_config.opt in stack project.
  HCI_LE_SetPhyCmd(currentConnHandle, 0, phy[index], phy[index], 0);

  return (true);
}

/*********************************************************************
 * @fn      SPPBLEClient_doDisconnect
 *
 * @brief   Disconnect the specified link
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SPPBLEClient_doDisconnect(uint8_t index)
{
  (void) index;

  GAP_TerminateLinkReq(sbcConnHandle, HCI_DISCONNECT_REMOTE_USER_TERM);

  return (true);
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
