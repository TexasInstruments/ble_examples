/*
 * Filename: spp_ble_client.c
 *
 * Description: This is the simple_peripheral example modified to send
 * data over BLE at a high throughput.
 *
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"

#include "hci_tl.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "gatt_uuid.h"
#include "serial_port_service.h"

#include "spp_ble_client.h"
#include "inc/sdi_task.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include "board_key.h"
#include "board.h"

#include "ble_user_config.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Simple BLE Central Task Events
#define SBC_START_DISCOVERY_EVT               0x0001
#define SBC_PAIRING_STATE_EVT                 0x0002
#define SBC_PASSCODE_NEEDED_EVT               0x0004
#define SBC_RSSI_READ_EVT                     0x0008
#define SBC_KEY_CHANGE_EVT                    0x0010
#define SBC_STATE_CHANGE_EVT                  0x0020
#define SBC_UART_CHANGE_EVT                   0x0040
#define SBC_PERIODIC_EVT                      0x0080
#define SBC_AUTO_CONNECT_EVT                  0x0100

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

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

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

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

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// Default notification enable timer delay in ms
#define DEFAULT_NOTI_ENABLE_DELAY             200

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// TRUE to connect automatically to server to preset address
#ifndef CLIENT_AUTO_CONNECT
#define CLIENT_AUTO_CONNECT                   TRUE
#endif

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                   864
#endif


// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

#define APP_SUGGESTED_PDU_SIZE 251
#define APP_SUGGESTED_TX_TIME 2120

/*********************************************************************
 * TYPEDEFS
 */

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
  uint8_t *pData; // event data pointer
} sbcEvt_t;

// App event passed from profiles.
typedef struct
{
  uint8_t *pData;  // New data
  uint8_t length; // New status
} sbcUARTEvt_t;

// RSSI read data structure
typedef struct
{
  uint16_t period;      // how often to read RSSI
  uint16_t connHandle;  // connection handle
  Clock_Struct *pClock; // pointer to clock struct
} readRssi_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
//Display_Handle dispHandle = NULL;

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

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
//static Clock_Struct periodicClock;

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

// Task pending events
static uint16_t events = 0;

// Task configuration
Task_Struct sbcTask;
Char sbcTaskStack[SBC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "SPP BLE Client";

// Number of scan results and scan result index
static uint8_t scanRes;
static uint8_t scanIdx;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static bool scanningStarted = FALSE;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

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

// GATT read/write procedure state
static bool procedureInProgress = FALSE;

// Maximum PDU size (default = 27 octets)
static uint16 maxPduSize;

// Maximum MTU size (default = 23 octets)
static uint16 currentMTUSize;

// Pins that are actively used by the application
static PIN_Config SPPBLEAppPinTable[] =
{
    Board_RLED       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */
    Board_GLED       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */

    PIN_TERMINATE
};

// Array of RSSI read structures
static readRssi_t readRssi[MAX_NUM_BLE_CONNS];

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SPPBLEClient_init(void);
static void SPPBLEClient_taskFxn(UArg a0, UArg a1);

static void SPPBLEClient_processGATTMsg(gattMsgEvent_t *pMsg);
static void SPPBLEClient_handleKeys(uint8_t shift, uint8_t keys);
static void SPPBLEClient_processStackMsg(ICall_Hdr *pMsg);
static void SPPBLEClient_processAppMsg(sbcEvt_t *pMsg);
static void SPPBLEClient_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void SPPBLEClient_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void SPPBLEClient_startDiscovery(void);
static bool SPPBLEClient_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen);
static void SPPBLEClient_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void SPPBLEClient_processPairState(uint8_t state, uint8_t status);
static void SPPBLEClient_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);

static void SPPBLEClient_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
//static bStatus_t SPPBLEClient_StartRssi(uint16_t connHandle, uint16_t period);
//static bStatus_t SPPBLEClient_CancelRssi(uint16_t connHandle);
//static readRssi_t *SPPBLEClient_RssiAlloc(uint16_t connHandle);
//static readRssi_t *SPPBLEClient_RssiFind(uint16_t connHandle);
//static void SPPBLEClient_RssiFree(uint16_t connHandle);

static uint8_t SPPBLEClient_eventCB(gapCentralRoleEvent_t *pEvent);
static void SPPBLEClient_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs);
static void SPPBLEClient_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status);

void SPPBLEClient_startDiscHandler(UArg a0);
void SPPBLEClient_keyChangeHandler(uint8 keys);
void SPPBLEClient_readRssiHandler(UArg a0);

static uint8_t SPPBLEClient_enqueueMsg(uint8_t event, uint8_t status,
                                           uint8_t *pData);

void SPPBLEClient_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len);
static void SPPBLEClient_genericHandler(UArg arg);
static void SPPBLEClient_autoConnect(void);
char* convInt32ToText(int32 value);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t SPPBLEClient_roleCB =
{
  SPPBLEClient_eventCB     // Event callback
};

// Bond Manager Callbacks
static gapBondCBs_t SPPBLEClient_bondCB =
{
  (pfnPasscodeCB_t)SPPBLEClient_passcodeCB, // Passcode callback
  SPPBLEClient_pairStateCB                  // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
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
    PIN_setOutputValue(hGpioPin, led, 1);
    delay_ms(BLINK_DURATION);
    PIN_setOutputValue(hGpioPin, led, 0);
    delay_ms(BLINK_DURATION);
  }
}

/*******************************************************************************
 * @fn      SPPBLEClient_toggleLed
 *
 * @brief   Blinks a led 'n' times, duty-cycle 50-50
 * @param   led - led identifier
 * @param   nBlinks - number of blinks
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
 * @fn      SPPBLEClient_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
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
  ICall_registerApp(&selfEntity, &sem);

  // Handling of LED
  hGpioPin = PIN_open(&pinGpioState, SPPBLEAppPinTable);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  appUARTMsgQueue = Util_constructQueue(&appUARTMsg);

  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, SPPBLEClient_genericHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, SBC_START_DISCOVERY_EVT);

  Util_constructClock(&startNotiEnableClock, SPPBLEClient_genericHandler,
                      DEFAULT_NOTI_ENABLE_DELAY, 0, false, SBC_UART_CHANGE_EVT);

  Board_initKeys(SPPBLEClient_keyChangeHandler);

//  dispHandle = Display_open(Display_Type_LCD, NULL);

  // Initialize internal data
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    readRssi[i].connHandle = GAP_CONNHANDLE_ALL;
    readRssi[i].pClock = NULL;
  }

  // Setup Central Profile
  {
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;

    GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8_t),
                                &scanRes);
  }

  // Setup GAP
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void *)attDeviceName);

  //Set CI to 20ms
  GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, 16);
  GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, 16);

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = DEFAULT_PASSCODE;
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    uint8_t mitm = DEFAULT_MITM_MODE;
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    uint8_t bonding = DEFAULT_BONDING_MODE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Start the Device
  VOID GAPCentralRole_StartDevice(&SPPBLEClient_roleCB);

  // Register with bond manager after starting device
  GAPBondMgr_Register(&SPPBLEClient_bondCB);

  //Register to receive UART messages
  SDITask_registerIncomingRXEventAppCB(SPPBLEClient_enqueueUARTMsg);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  uint8_t hello[] = "Hello from SPP BLE Client! With Data Length Extension support!\n\r";
  DEBUG(hello);

  Display_print0(dispHandle, 0, 0, "BLE Central");

  //Blink twice for client
  SPPBLEClient_blinkLed(Board_GLED, 2);
}

/*********************************************************************
 * @fn      SPPBLEClient_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void SPPBLEClient_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SPPBLEClient_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
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
          SPPBLEClient_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

      // If RTOS queue is not empty, process app UART message.
      if (!Queue_empty(appUARTMsgQueue))
      {
        //Get the message at the front of the queue but still keep it in the queue
        queueRec_t *pRec = Queue_head(appUARTMsgQueue);
        sbcUARTEvt_t *pMsg = (sbcUARTEvt_t *)pRec->pData;

        if (pMsg && (state == BLE_STATE_CONNECTED))
        {
          // Process message.
          bStatus_t retVal = FAILURE;

            // Do a write
            attWriteReq_t req;

            //Allocate data bytes to send over the air
            req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, pMsg->length, NULL);

            if ( (req.pValue != NULL) && charDataHdl)
            {
              req.handle = charDataHdl; //handle for Value of Data characteristic found during service discovery
              req.len = pMsg->length;
              memcpy(req.pValue, pMsg->pData, pMsg->length);
              req.sig = FALSE;
              req.cmd = TRUE;

            retVal = GATT_WriteNoRsp(connHandle, &req);

            if ( retVal != SUCCESS )
            {
              GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
                DEBUG("FAIL FROM CLIENT: "); DEBUG((uint8_t*)convInt32ToText((int)retVal)); DEBUG_NEWLINE();
            }else
            {
              //Remove from the queue
              Util_dequeueMsg(appUARTMsgQueue);

              //Toggle LED to indicate data received from UART terminal and sent over the air
              //SPPBLEClient_toggleLed(Board_GLED, Board_LED_TOGGLE);

              ICall_freeMsg(pMsg->pData);
              // Free the space from the message.
              ICall_free(pMsg);

              if(!Queue_empty(appUARTMsgQueue))
              {
                // Wake up the application to flush out any remaining UART data in the queue.
                Semaphore_post(sem);
              }
            }
          }

        }

      }

    // If RTOS queue is not empty, process app message
    while (!Queue_empty(appMsgQueue))
    {
      sbcEvt_t *pMsg = (sbcEvt_t *)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message
        SPPBLEClient_processAppMsg(pMsg);

        // Free the space from the message
        ICall_free(pMsg);
      }
    }


    if (events & SBC_UART_CHANGE_EVT)
    {
      // Process message.
      uint8 retVal;
      attWriteReq_t req;
      uint8 configData[2] = {0x01,0x00};

      events &= ~SBC_UART_CHANGE_EVT;

      req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, 2, NULL);

      if ((charCCCDHdl == NULL) && (charDataHdl != NULL)) {charCCCDHdl = charDataHdl + 1;} //Hardcoded
      if ( (req.pValue != NULL) && charCCCDHdl)
      {
        req.handle = charCCCDHdl; //Handle for CCCD of Data characteristic
        req.len = 2;
        memcpy(req.pValue, configData, 2);
        req.cmd = TRUE; //Has to be true for NoRsp from server(command, not request)
        req.sig = FALSE;
        retVal = GATT_WriteNoRsp(connHandle, &req);
        if (retVal != SUCCESS)
        {
          DEBUG("ERROR enabling notification...\n\r");
        }
        else
        {
          DEBUG("Notification enabled...\n\r");
        }
      }
    }


    if (events & SBC_START_DISCOVERY_EVT)
    {
      events &= ~SBC_START_DISCOVERY_EVT;

      if(!scanningStarted)
      SPPBLEClient_startDiscovery();

    }

    if (events & SBC_AUTO_CONNECT_EVT)
    {
      events &= ~SBC_AUTO_CONNECT_EVT;

#if defined (CLIENT_AUTO_CONNECT) && (CLIENT_AUTO_CONNECT == TRUE)
      SPPBLEClient_autoConnect();
#endif
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
 * @return  none
 */
static void SPPBLEClient_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SPPBLEClient_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;

    case GATT_MSG_EVENT:
      SPPBLEClient_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            SPPBLEClient_processCmdCompleteEvt((hciEvt_CmdComplete_t *)pMsg);
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
 * @fn      SPPBLEClient_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SPPBLEClient_processAppMsg(sbcEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBC_STATE_CHANGE_EVT:
      SPPBLEClient_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;

    case SBC_KEY_CHANGE_EVT:
      SPPBLEClient_handleKeys(0, pMsg->hdr.state);
      break;

    case SBC_RSSI_READ_EVT:
      {
        readRssi_t *pRssi = (readRssi_t *)pMsg->pData;

        // If link is up and RSSI reads active
        if (pRssi->connHandle != GAP_CONNHANDLE_ALL &&
            linkDB_Up(pRssi->connHandle))
        {
          // Restart timer
          Util_restartClock(pRssi->pClock, pRssi->period);

          // Read RSSI
          VOID HCI_ReadRssiCmd(pRssi->connHandle);
        }
      }
      break;

    // Pairing event
    case SBC_PAIRING_STATE_EVT:
      {
        SPPBLEClient_processPairState(pMsg->hdr.state, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    // Passcode event
    case SBC_PASSCODE_NEEDED_EVT:
      {
        SPPBLEClient_processPasscode(connHandle, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SPPBLEClient_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        maxPduSize = pEvent->initDone.dataPktLen;

        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
        Display_print0(dispHandle, 2, 0, "Initialized");

#if defined (CLIENT_AUTO_CONNECT) && (CLIENT_AUTO_CONNECT == TRUE)
        SPPBLEClient_genericHandler(SBC_AUTO_CONNECT_EVT);
#endif
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        {
          if (SPPBLEClient_findSvcUuid(SERIALPORTSERVICE_SERV_UUID,
                                           pEvent->deviceInfo.pEvtData,
                                           pEvent->deviceInfo.dataLen))
          {
            SPPBLEClient_addDeviceInfo(pEvent->deviceInfo.addr,
                                           pEvent->deviceInfo.addrType);
          }
        }
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        scanningStarted = FALSE;

        // if not filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE)
        {
          // Copy results
          scanRes = pEvent->discCmpl.numDevs;
          memcpy(devList, pEvent->discCmpl.pDevList,
                 (sizeof(gapDevRec_t) * scanRes));
        }

        Display_print1(dispHandle, 2, 0, "Devices Found %d", scanRes);

        if (scanRes > 0)
        {
          //Display_print0(dispHandle, 3, 0, "<- To Select");
        }

        // initialize scan index to last device
        scanIdx = scanRes;
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;
          procedureInProgress = TRUE;

          SPPBLEClient_toggleLed(Board_GLED, Board_LED_TOGGLE);

          // If service discovery not performed initiate service discovery
          if (charDataHdl == 0)
          {
            Util_startClock(&startDiscClock);
          }

          Display_print0(dispHandle, 2, 0, "Connected");
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));
        }
        else
        {
          state = BLE_STATE_IDLE;
          connHandle = GAP_CONNHANDLE_INIT;
          discState = BLE_DISC_STATE_IDLE;

          Display_print0(dispHandle, 2, 0, "Connect Failed");
          Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->gap.hdr.status);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;
        discState = BLE_DISC_STATE_IDLE;
        charDataHdl = 0;
        procedureInProgress = FALSE;

        // Cancel RSSI reads
        //SPPBLEClient_CancelRssi(pEvent->linkTerminate.connectionHandle);

        Display_print0(dispHandle, 2, 0, "Disconnected");
        Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->linkTerminate.reason);
        //Display_clearLine(dispHandle, 4);
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        Display_print1(dispHandle, 2, 0, "Param Update: %d", pEvent->linkUpdate.status);
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
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void SPPBLEClient_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter


  // Set Packet Length in a Connection
  if (keys & KEY_RIGHT)
  {
    //SPPBLEClient_toggleLed(Board_GLED, Board_LED_TOGGLE);

    if (state == BLE_STATE_CONNECTED )
    {
      //Request max supported size
      uint16_t requestedPDUSize = APP_SUGGESTED_PDU_SIZE;
      uint16_t requestedTxTime = APP_SUGGESTED_TX_TIME;

      //This API is documented in hci.h
      if(SUCCESS != HCI_LE_SetDataLenCmd(connHandle, requestedPDUSize, requestedTxTime))
      {
        DEBUG("Data length update failed");
      }

    }else
    {
      uint8_t addrType;
      uint8_t *peerAddr;

      // Connect or disconnect
      if (state == BLE_STATE_IDLE)
      {
        // if there is a scan result
        if (scanRes > 0)
        {
          // connect to current device in scan result
          peerAddr = devList[scanIdx].addr;
          addrType = devList[scanIdx].addrType;

          state = BLE_STATE_CONNECTING;

          GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                       DEFAULT_LINK_WHITE_LIST,
                                       addrType, peerAddr);
        }
      }
    }
    return;
  }

  if (keys & KEY_LEFT)
  {
    //SPPBLEClient_toggleLed(Board_RLED, Board_LED_TOGGLE);

    // Start or stop discovery
    if (state == BLE_STATE_CONNECTED &&
             charDataHdl != 0  &&
             procedureInProgress == FALSE)
    {
      uint8_t status;

      // Do a read or write as long as no other read or write is in progress
      procedureInProgress = TRUE;
      {
        // Do a write
        attWriteReq_t req;

        req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, 1, NULL);
        if ( req.pValue != NULL )
        {
          req.handle = charDataHdl;
          req.len = 1;
          req.pValue[0] = charVal++;
          req.sig = 0;
          req.cmd = 1;

          status = GATT_WriteNoRsp(connHandle, &req);
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
      procedureInProgress = FALSE;

    }
    else
    {
#if defined (CLIENT_AUTO_CONNECT) && (CLIENT_AUTO_CONNECT == TRUE)
      SPPBLEClient_genericHandler(SBC_AUTO_CONNECT_EVT);
#endif
    }

    return;
  }

}

/*********************************************************************
 * @fn      SPPBLEClient_autoConnect
 *
 * @brief   Client automatically connects to hardcoded address
 *
 * @return  none
 */
void SPPBLEClient_autoConnect(void)
{
    uint8_t addrType;
    uint8_t peerAddr[6];

    // connect to hardcoded device address i.e. 0x050403020100
    int x = 0;
    for(x = 0; x<6; x++)
    {
        peerAddr[x] = x;
    }

    addrType = ADDRTYPE_PUBLIC;

    DEBUG("Auto connecting..."); DEBUG_NEWLINE();

    GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                 DEFAULT_LINK_WHITE_LIST,
                                 addrType, peerAddr);

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
  if (state == BLE_STATE_CONNECTED)
  {

    if(pMsg->method == ATT_HANDLE_VALUE_NOTI)
    {
      //Send received bytes to serial port
      SDITask_sendToUART(pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len);

      //Toggle LED to indicate data received from client
      SPPBLEClient_toggleLed(Board_RLED, Board_LED_TOGGLE);
    }
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      Display_print1(dispHandle, 4, 0, "ATT Rsp dropped %d", pMsg->method);
    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, 4, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Display_print1(dispHandle, 4, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
      }

      procedureInProgress = FALSE;
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, 4, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful write, display the value that was written and
        // increment value
        Display_print1(dispHandle, 4, 0, "Write sent: %d", charVal++);
      }

      procedureInProgress = FALSE;

    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.

      // Display the opcode of the message that caused the violation.
      Display_print1(dispHandle, 4, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      currentMTUSize = pMsg->msg.mtuEvt.MTU;
      SDITask_setAppDataSize(currentMTUSize);

      DEBUG("MTU Size: "); DEBUG((uint8_t*)convInt32ToText((int)currentMTUSize)); DEBUG_NEWLINE();

      // MTU size updated
      Display_print1(dispHandle, 4, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
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
  case HCI_LE_SET_DATA_LENGTH:
    //DEBUG("HCI_LE_SET_DATA_LENGTH");
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

//    case HCI_READ_RSSI:
//      {
//        int8 rssi = (int8)pMsg->pReturnParam[3];
//
//        Display_print1(dispHandle, 4, 0, "RSSI -dB: %d", (uint32_t)(-rssi));
//      }
//      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_StartRssi
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   period - RSSI read period in ms
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
//static bStatus_t SPPBLEClient_StartRssi(uint16_t connHandle, uint16_t period)
//{
//  readRssi_t *pRssi;
//
//  // Verify link is up
//  if (!linkDB_Up(connHandle))
//  {
//    return bleIncorrectMode;
//  }
//
//  // If already allocated
//  if ((pRssi = SPPBLEClient_RssiFind(connHandle)) != NULL)
//  {
//    // Stop timer
//    Util_stopClock(pRssi->pClock);
//
//    pRssi->period = period;
//  }
//  // Allocate structure
//  else if ((pRssi = SPPBLEClient_RssiAlloc(connHandle)) != NULL)
//  {
//    pRssi->period = period;
//  }
//  // Allocate failed
//  else
//  {
//    return bleNoResources;
//  }
//
//  // Start timer
//  Util_restartClock(pRssi->pClock, period);
//
//  return SUCCESS;
//}

/*********************************************************************
 * @fn      SPPBLEClient_CancelRssi
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
//static bStatus_t SPPBLEClient_CancelRssi(uint16_t connHandle)
//{
//  readRssi_t *pRssi;
//
//  if ((pRssi = SPPBLEClient_RssiFind(connHandle)) != NULL)
//  {
//    // Stop timer
//    Util_stopClock(pRssi->pClock);
//
//    // Free RSSI structure
//    SPPBLEClient_RssiFree(connHandle);
//
//    return SUCCESS;
//  }
//
//  // Not found
//  return bleIncorrectMode;
//}

///*********************************************************************
// * @fn      gapCentralRole_RssiAlloc
// *
// * @brief   Allocate an RSSI structure.
// *
// * @param   connHandle - Connection handle
// *
// * @return  pointer to structure or NULL if allocation failed.
// */
//static readRssi_t *SPPBLEClient_RssiAlloc(uint16_t connHandle)
//{
//  uint8_t i;
//
//  // Find free RSSI structure
//  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
//  {
//    if (readRssi[i].connHandle == GAP_CONNHANDLE_ALL)
//    {
//      readRssi_t *pRssi = &readRssi[i];
//
//      pRssi->pClock = (Clock_Struct *)ICall_malloc(sizeof(Clock_Struct));
//      if (pRssi->pClock)
//      {
//        Util_constructClock(pRssi->pClock, SPPBLEClient_readRssiHandler,
//                            0, 0, false, i);
//        pRssi->connHandle = connHandle;
//
//        return pRssi;
//      }
//    }
//  }
//
//  // No free structure found
//  return NULL;
//}

/*********************************************************************
 * @fn      gapCentralRole_RssiFind
 *
 * @brief   Find an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if not found.
 */
//static readRssi_t *SPPBLEClient_RssiFind(uint16_t connHandle)
//{
//  uint8_t i;
//
//  // Find free RSSI structure
//  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
//  {
//    if (readRssi[i].connHandle == connHandle)
//    {
//      return &readRssi[i];
//    }
//  }
//
//  // Not found
//  return NULL;
//}

/*********************************************************************
 * @fn      gapCentralRole_RssiFree
 *
 * @brief   Free an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  none
 */
//static void SPPBLEClient_RssiFree(uint16_t connHandle)
//{
//  uint8_t i;
//
//  // Find RSSI structure
//  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
//  {
//    if (readRssi[i].connHandle == connHandle)
//    {
//      readRssi_t *pRssi = &readRssi[i];
//      if (pRssi->pClock)
//      {
//        Clock_destruct(pRssi->pClock);
//
//        // Free clock struct
//        ICall_free(pRssi->pClock);
//        pRssi->pClock = NULL;
//      }
//
//      pRssi->connHandle = GAP_CONNHANDLE_ALL;
//      break;
//    }
//  }
//}

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
    Display_print0(dispHandle, 2, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Pairing success");
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
      Display_print0(dispHandle, 2, 0, "Bonding success");
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
 * @fn      SPPBLEClient_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SPPBLEClient_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs)
{
  uint32_t  passcode;

  // Create random passcode
  passcode = Util_GetTRNG();
  passcode %= 1000000;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      SPPBLEClient_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void SPPBLEClient_startDiscovery(void)
{
  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = charDataHdl = 0;

  discState = BLE_DISC_STATE_MTU;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
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
    // MTU size response received, discover simple BLE service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_UUID_SIZE] = { TI_BASE_UUID_128(SERIALPORTSERVICE_SERV_UUID) };

      discState = BLE_DISC_STATE_SVC;

      DEBUG("Discovering services..."); DEBUG_NEWLINE();

      // Discovery simple BLE service
      VOID GATT_DiscPrimaryServiceByUUID(connHandle, uuid, ATT_UUID_SIZE,
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

        //DEBUG("Reading UUIDs...");


        // Discover characteristic descriptors
        GATT_DiscAllCharDescs(connHandle,
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
            DEBUG("Data Char Found..."); //DEBUG_NEWLINE();
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

      procedureInProgress = FALSE;
      discState = BLE_DISC_STATE_IDLE;
    }





  }
}

/*********************************************************************
 * @fn      SPPBLEClient_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool SPPBLEClient_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen)
{
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

  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      SPPBLEClient_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SPPBLEClient_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;

  // If result count not at max
  if (scanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < scanRes; i++)
    {
      if (memcmp(pAddr, devList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);
    devList[scanRes].addrType = addrType;

    // Increment scan result count
    scanRes++;
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SPPBLEClient_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SPPBLEClient_enqueueMsg(SBC_STATE_CHANGE_EVT,
                                  SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      SPPBLEClient_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SPPBLEClient_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    SPPBLEClient_enqueueMsg(SBC_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      SPPBLEClient_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SPPBLEClient_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    SPPBLEClient_enqueueMsg(SBC_PASSCODE_NEEDED_EVT, 0, pData);
  }
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
  events |= SBC_START_DISCOVERY_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
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
void SPPBLEClient_keyChangeHandler(uint8 keys)
{
  SPPBLEClient_enqueueMsg(SBC_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SPPBLEClient_readRssiHandler
 *
 * @brief   Read RSSI handler function
 *
 * @param   a0 - read RSSI index
 *
 * @return  none
 */
void SPPBLEClient_readRssiHandler(UArg a0)
{
  SPPBLEClient_enqueueMsg(SBC_RSSI_READ_EVT, SUCCESS,
                              (uint8_t *)&readRssi[a0]);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SPPBLEClient_genericHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SPPBLEClient_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   status - message status.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
void SPPBLEClient_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len)
{
  sbcUARTEvt_t *pMsg;

  //Enqueue message only in a connected state
  if(state == BLE_STATE_CONNECTED)
  {
    // Create dynamic pointer to message.
    if (pMsg = ICall_malloc(sizeof(sbcUARTEvt_t)))
    {

      pMsg->pData = (uint8 *)ICall_allocMsg(len);
      if(pMsg->pData)
      {
        //payload
        memcpy(pMsg->pData , data, len);
      }
      pMsg->length = len;

      // Enqueue the message.
      Util_enqueueMsg(appUARTMsgQueue, sem, (uint8_t *)pMsg);
    }
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
    return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
  }

  return FALSE;
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
