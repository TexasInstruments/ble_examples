/*
 * Filename: simple_peripheral_bidirectional_audio.c
 *
 * Description: This is the simple_peripheral example modified to send
 * audio data over BLE.
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

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#include "hci_tl.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simple_gatt_profile.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_ble_api.h"

#include "util.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC
#include "board_key.h"

#include "board.h"

#include "audio_peripheral.h"

#include "simple_peripheral_bidirectional_audio.h"

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         2

// Task configuration
#define SBP_TASK_PRIORITY                     1


#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   864
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PAIRING_STATE_EVT                 0x0004
#define SBP_PASSCODE_NEEDED_EVT               0x0008
#define SBP_CONN_EVT_END_EVT                  0x0010
#define SBP_KEY_CHANGE_EVT                    0x0020
#define SBP_KEY_CHANGE_EVT                    0x0020
#define SBP_AUDIO_EVT                         0x0040

// Internal Events for RTOS application
#define SBP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBP_PERIODIC_EVT                      Event_Id_00

#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT        | \
                                               SBP_QUEUE_EVT        | \
                                               SBP_PERIODIC_EVT)

#define DLE_MAX_PDU_SIZE 251
#define DLE_MAX_TX_TIME 2120

#define DEFAULT_PDU_SIZE 27
#define DEFAULT_TX_TIME 328

// The combined overhead for L2CAP and ATT notification headers
#define TOTAL_PACKET_OVERHEAD 7

// GATT notifications for throughput example don't require an authenticated link
#define GATT_NO_AUTHENTICATION 0

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
  uint8_t *pData;  // event data
} sbpEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;


typedef struct
{
  // Service and Characteristic discovery variables.
  uint16 audioStartCharValueHandle;
  uint16 audioDataCharValueHandle;
  uint16 audioVolumeCharValueHandle;
  uint8  lastRemoteAddr[B_ADDR_LEN];
} SimpleBLEPeripheral_HandleInfo_t;

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

/*********************************************************************
 * LOCAL VARIABLES
 */
// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

static PIN_Config SBP_configTable[] =
{
 Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 Board_DIO25_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,  /* Debug IO initially high       */
 Board_DIO26_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,  /* Debug IO initially high       */
 Board_DIO27_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,  /* Debug IO initially high       */
 Board_DIO28_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,  /* Debug IO initially high       */
 PIN_TERMINATE
};

static PIN_State sbpPins;
static PIN_Handle hSbpPins;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
 // complete name
 0x16,   // length of this data
 GAP_ADTYPE_LOCAL_NAME_COMPLETE,
 'S',
 'i',
 'm',
 'p',
 'l',
 'e',
 'B',
 'L',
 'E',
 'A',
 'u',
 'd',
 'i',
 'o',
 'T',
 'x',
};

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
 0x03,   // length of this data
 GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
 LO_UINT16(AUDIO_SERV_UUID),
 HI_UINT16(AUDIO_SERV_UUID)
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE AudioTx";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

static uint8 serviceDiscComplete = FALSE;
static uint16 serviceToDiscover = GATT_INVALID_HANDLE;
static uint8 enableCCCDs = TRUE;

// Handle info saved here after connection to skip service discovery.
static SimpleBLEPeripheral_HandleInfo_t remoteHandles;

/* Audio START characteristic */
static uint16 audioStartCharValueHandle   = GATT_INVALID_HANDLE;
static uint16 audioStartCCCHandle         = GATT_INVALID_HANDLE;
/* Audio "Data" characteristic */
static uint16 audioDataCharValueHandle    = GATT_INVALID_HANDLE;
static uint16 audioDataCCCHandle          = GATT_INVALID_HANDLE;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);

static uint8_t SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state,
                                              uint8_t *pData);
static void SimpleBLEPeripheral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                           uint8_t uiInputs, uint8_t uiOutputs);
void SimpleBLEPeripheral_keyChangeHandler(uint8 keys);
static void SimpleBLEPeripheral_handleKeys(uint8_t shift, uint8_t keys);

static void SimpleBLEPeripheral_pairStateCB(uint16_t connHandle, uint8_t state,
                                            uint8_t status);
static void SimpleBLEPeripheral_processPairState(uint8_t state, uint8_t status);
static void SimpleBLEPeripheral_processPasscode(uint8_t uiOutputs);
static void SimpleBLEPeripheral_DiscoverService( uint16 connHandle, uint16 svcUuid );
static void SimpleBLEPeripheral_EnableNotification( uint16 connHandle, uint16 attrHandle );

//static void SimpleBLECentral_SaveHandles( void );
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
 SimpleBLEPeripheral_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
 (pfnPasscodeCB_t) SimpleBLEPeripheral_passcodeCB, // Passcode callback
 SimpleBLEPeripheral_pairStateCB                  // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
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
static void SimpleBLEPeripheral_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  HCI_LE_WriteSuggestedDefaultDataLenCmd(DLE_MAX_PDU_SIZE , DLE_MAX_TX_TIME);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  dispHandle = Display_open(Display_Type_ANY, NULL);
  Display_print0(dispHandle, 0, 0, "\f");

  Board_initKeys(SimpleBLEPeripheral_keyChangeHandler);

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager. For more information see the section in the
  // User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
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

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);
  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes

  // Add Audio Service
  Audio_AddService();

  // Start the Device
  VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  HCI_LE_ReadMaxDataLenCmd();

  Display_print0(dispHandle, 0, 0, "Audio Tx Peripheral with DLE");

  // Open pin structure for use
  hSbpPins = PIN_open(&sbpPins, SBP_configTable);

  AudioPeripheral_init(dispHandle, hSbpPins);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEPeripheral_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread

    events = Event_pend(syncEvent, Event_Id_NONE, SBP_ALL_EVENTS,
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
            if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              SimpleBLEPeripheral_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SimpleBLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }
      // If RTOS queue is not empty, process app message.
      if (events & SBP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            SimpleBLEPeripheral_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
  case GATT_MSG_EVENT:
    // Process GATT message
    safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
    break;

  case HCI_GAP_EVENT_EVENT:
  {
    // Process HCI message
    switch(pMsg->status)
    {
    case HCI_COMMAND_COMPLETE_EVENT_CODE:
      // Process HCI Command Complete Event
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
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (state == BLE_STATE_CONNECTED)
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      Display_print1(dispHandle, 4, 0, "ATT Rsp dropped %d", pMsg->method);
    };

    switch ( pMsg->method )
    {
    case ATT_HANDLE_VALUE_NOTI:
      // Check to see if notification is from audio data or control char
      if (pMsg->msg.handleValueNoti.handle == audioDataCharValueHandle)
      {
        genericAudioData_t pData;
        pData.len = pMsg->msg.handleValueNoti.len;
        pData.pValue = pMsg->msg.handleValueNoti.pValue;
        AudioPeripheral_processData(AUDIO_DATA, &pData);
      }
      break;

    case ATT_FIND_BY_TYPE_VALUE_RSP:
      // Response from GATT_DiscPrimaryServiceByUUID
      // Service found, store handles
      if ( pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        svcStartHdl =
            ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        svcEndHdl =
            ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      }
      // If procedure complete
      else if ( pMsg->hdr.status == bleProcedureComplete )
      {
        if ( svcStartHdl != 0 )
        {
          if ( serviceToDiscover == AUDIO_SERV_UUID)
          {
            // Discover all characteristics
            GATT_DiscAllChars( connHandle, svcStartHdl, svcEndHdl, selfEntity );
          }

        }
      }
      break;

    case ATT_ERROR_RSP:

      if (serviceToDiscover ==  AUDIO_SERV_UUID
          && pMsg->msg.errorRsp.reqOpcode == ATT_FIND_BY_TYPE_VALUE_REQ
          && pMsg->msg.errorRsp.handle == 0x0001)
        //0x0001 is the start attribute handle of 0xfff0, AUDIO_SERV_UUID
      {
        if ( (enableCCCDs == TRUE) && (audioStartCharValueHandle != GATT_INVALID_HANDLE))
        {
          audioStartCCCHandle = audioStartCharValueHandle + 1;
          // Begin configuring the characteristics for notifications
          SimpleBLEPeripheral_EnableNotification( connHandle, audioStartCCCHandle );
        }
      }
      break;

    case ATT_READ_BY_TYPE_RSP:
      // Response from Discover all Characteristics.
      // Success indicates packet with characteristic discoveries.
      if ( pMsg->hdr.status == SUCCESS )
      {
        attReadByTypeRsp_t *pRsp = &pMsg->msg.readByTypeRsp;

        if( serviceToDiscover ==  AUDIO_SERV_UUID )
        {
          uint16 charUUID = GATT_INVALID_HANDLE;
          uint16 *pHandle = &charUUID;
          /* Write into charUUID what Audio Profile char value we're dealing with */
          *pHandle = BUILD_UINT16( pRsp->pDataList[17] , pRsp->pDataList[18]);
          if      (charUUID == AUDIOPROFILE_START_UUID) {
            pHandle = &audioStartCharValueHandle;
            *pHandle = BUILD_UINT16( pRsp->pDataList[3] , pRsp->pDataList[4]);
          }
          else if (charUUID == AUDIOPROFILE_AUDIO_UUID ){
            pHandle = &audioDataCharValueHandle;
            *pHandle = BUILD_UINT16( pRsp->pDataList[3] , pRsp->pDataList[4]);
          }
        }
        break;
      }

      // This indicates that there is no more characteristic data
      // to be discovered within the given handle range.
      else if ( pMsg->hdr.status == bleProcedureComplete )
      {
        if ( serviceToDiscover == AUDIO_SERV_UUID )
        {
          /* This kicks off the enabling the 1st of notification enable event */
          if (audioStartCharValueHandle != GATT_INVALID_HANDLE) {
            audioStartCCCHandle = audioStartCharValueHandle + 1 ;
            SimpleBLEPeripheral_EnableNotification( connHandle, audioStartCCCHandle );
          }
          break;
        }

      }
      break;

    case ATT_WRITE_RSP:
      if ( pMsg->hdr.status == SUCCESS && !serviceDiscComplete )
      {
        uint16 handle = GATT_INVALID_HANDLE;

        // Chain the CCCD enable writes so that a RSP for one triggers the next enable.
        if (audioDataCCCHandle == GATT_INVALID_HANDLE) {
          handle = audioDataCCCHandle = audioDataCharValueHandle + 1;
        }
        else {
          serviceDiscComplete = TRUE;
          break;
        }

        SimpleBLEPeripheral_EnableNotification( connHandle, handle );

        break;

      }

      break;



      // Service Change indication
    case ATT_HANDLE_VALUE_IND:
      // Note: this logic assumes that the only indications that will be sent
      //       will come from that GATT Service Changed Characteristic
      if ( pMsg->hdr.status == SUCCESS )
      {

        // Acknowledge receipt of indication
        ATT_HandleValueCfm( pMsg->connHandle );

      }
      break;

    default:
      // Unknown event
      break;
    } //switch
  } // else - in case a GATT message came after a connection has dropped, ignore it.
  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_sendAttRsp(void)
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
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, 5, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, 5, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, 5, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
  case SBP_STATE_CHANGE_EVT:
  {
    SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                              hdr.state);
    break;
  }

  case SBP_KEY_CHANGE_EVT:
  {
    SimpleBLEPeripheral_handleKeys(0, pMsg->hdr.state);
    break;
  }

    // Pairing event
  case SBP_PAIRING_STATE_EVT:
  {
    SimpleBLEPeripheral_processPairState(pMsg->hdr.state, *pMsg->pData);

    ICall_free(pMsg->pData);
    break;
  }

  // Passcode event
  case SBP_PASSCODE_NEEDED_EVT:
  {
      SimpleBLEPeripheral_processPasscode(*pMsg->pData);

      ICall_free(pMsg->pData);
      break;
  }
  case SBP_AUDIO_EVT:
  {
    AudioPeripheral_eventHandler(pMsg->hdr.state);
    break;
  }

  default:
    // Do nothing.
    break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState, 0);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
  switch ( newState )
  {
  case GAPROLE_STARTED:
  {
    uint8_t ownAddress[B_ADDR_LEN];

    GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

    // Display device address
    Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
    Display_print0(dispHandle, 2, 0, "Initialized");
  }
  break;

  case GAPROLE_ADVERTISING:
    Display_print0(dispHandle, 2, 0, "Advertising");
    break;

  case GAPROLE_CONNECTED:
  {
    state = BLE_STATE_CONNECTED;

    uint8_t peerAddress[B_ADDR_LEN];

    GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);

    Display_print0(dispHandle, 2, 0, "Connected");
    Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));

    if (FALSE == serviceDiscComplete)
    {
      // Begin Service Discovery of AUDIO Service to find out report handles
      serviceToDiscover = AUDIO_SERV_UUID;
      SimpleBLEPeripheral_DiscoverService( connHandle, serviceToDiscover );
    }

  }
  break;

  case GAPROLE_CONNECTED_ADV:
    Display_print0(dispHandle, 2, 0, "Connected Advertising");
    break;

  case GAPROLE_WAITING:
    Util_stopClock(&periodicClock);
    SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

    Display_print0(dispHandle, 2, 0, "Disconnected");

    serviceDiscComplete = FALSE;

    audioStartCharValueHandle  = GATT_INVALID_HANDLE;
    audioStartCCCHandle        = GATT_INVALID_HANDLE;
    audioDataCharValueHandle   = GATT_INVALID_HANDLE;
    audioDataCCCHandle         = GATT_INVALID_HANDLE;

    AudioPeripheral_stopStreaming();

    // Clear remaining lines
    Display_clearLines(dispHandle, 3, 5);
    break;

  case GAPROLE_WAITING_AFTER_TIMEOUT:
    SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

    Display_print0(dispHandle, 2, 0, "Timed Out");

    serviceDiscComplete = FALSE;

    audioStartCharValueHandle  = GATT_INVALID_HANDLE;
    audioStartCCCHandle        = GATT_INVALID_HANDLE;
    audioDataCharValueHandle   = GATT_INVALID_HANDLE;
    audioDataCCCHandle         = GATT_INVALID_HANDLE;

    AudioPeripheral_stopStreaming();

    // Clear remaining lines
    Display_clearLines(dispHandle, 3, 5);
    break;

  case GAPROLE_ERROR:
    Display_print0(dispHandle, 2, 0, "Error");
    break;

  default:
    Display_clearLine(dispHandle, 2);
    break;
  }

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLEPeripheral_keyChangeHandler(uint8 keys)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static uint8_t SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  sbpEvt_t *pMsg = ICall_malloc(sizeof(sbpEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
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
 * @fn      SimpleBLEPeripheral_handleKeys
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
static void SimpleBLEPeripheral_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter
  AudioPeripheral_handleKeys(keys);
}

/*********************************************************************
 * @fn      SimpleBLECentral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimpleBLEPeripheral_pairStateCB(uint16_t connHandle, uint8_t state,
                                            uint8_t status)
{
  uint8_t *pData;
  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;
    // Queue the event.
    SimpleBLEPeripheral_enqueueMsg(SBP_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleBLEPeripheral_processPairState(uint8_t state, uint8_t status)
{
  Display_clearLines(dispHandle, 5, 5);
  switch (state)
  {
  case GAPBOND_PAIRING_STATE_STARTED:
    Display_print0(dispHandle, 2, 0, "Pairing started");
    break;

  case GAPBOND_PAIRING_STATE_COMPLETE:
    if (status == SUCCESS)
    {
      // Enter a GAP Bond manager Paired state
      Display_print0(dispHandle, 2, 0, "Pairing success");

      if (FALSE == serviceDiscComplete)
      {
        // Begin Service Discovery of AUDIO Service to find out report handles
        serviceToDiscover = AUDIO_SERV_UUID;
        SimpleBLEPeripheral_DiscoverService( connHandle, serviceToDiscover );
      }
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
    }
    break;

  case GAPBOND_PAIRING_STATE_BOND_SAVED:
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bond Saved");
    }
    break;

  case GAPBOND_PAIRING_STATE_BONDED:
    if (status == SUCCESS)
    {

      if (
          ( remoteHandles.audioStartCharValueHandle == GATT_INVALID_HANDLE )         ||
          ( remoteHandles.audioDataCharValueHandle == GATT_INVALID_HANDLE )
      )
      {

        serviceDiscComplete = FALSE;
        serviceToDiscover = AUDIO_SERV_UUID;

        // We must perform service discovery again, something might have changed.
        // Begin Service Discovery
        SimpleBLEPeripheral_DiscoverService( connHandle, serviceToDiscover );

      }
      else
      {
        // No change, restore handle info.
        // bonding indicates that we probably already enabled all these characteristics. easy fix if not.
        serviceDiscComplete    = TRUE;

        audioStartCharValueHandle = remoteHandles.audioStartCharValueHandle;
        audioDataCharValueHandle  = remoteHandles.audioDataCharValueHandle;

      }

      Display_print0(dispHandle, 2, 0, "Bond save success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
    }
    break;

  default:
    break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimpleBLEPeripheral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                           uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    SimpleBLEPeripheral_enqueueMsg(SBP_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimpleBLEPeripheral_processPasscode(uint8_t uiOutputs)
{
  // This app uses a default passcode. A real-life scenario would handle all
  // pairing scenarios and likely generate this randomly.
  uint32_t passcode = B_APP_DEFAULT_PASSCODE;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  uint16_t connectionHandle;
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_DiscoverService
 *
 * @brief   Discover service using UUID.
 *
 * @param   connHandle - connection handle to do discovery on
 * @param   svcUuid - service UUID to discover
 *
 * @return  none
 */
static void SimpleBLEPeripheral_DiscoverService( uint16 connHandle, uint16 svcUuid )
{
  if(svcUuid == AUDIO_SERV_UUID) // only take care of Audio Service in this project
  {
    uint8 uuid[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0,
                      0x00, 0x40, 0x51, 0x04, LO_UINT16( svcUuid ), HI_UINT16( svcUuid ), 0x00, 0xF0};

    VOID GATT_DiscPrimaryServiceByUUID( connHandle, uuid, ATT_UUID_SIZE, selfEntity );
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_EnableNotification
 *
 * @brief   Enable notification for a given attribute handle.
 *
 * @param   connHandle - connection handle to send notification on
 * @param   attrHandle - attribute handle to send notification for
 *
 * @return  none
 */
static void SimpleBLEPeripheral_EnableNotification( uint16 connHandle, uint16 attrHandle )
{
  attWriteReq_t req;

  req.pValue = GATT_bm_alloc( connHandle, ATT_WRITE_REQ, 2, NULL );
  if ( req.pValue != NULL )
  {
    uint8 notificationsOn[] = {0x01, 0x00};

    req.handle = attrHandle;

    req.len = 2;
    memcpy(req.pValue, notificationsOn, 2);

    req.sig = 0;
    req.cmd = 0;

    if ( GATT_WriteCharValue( connHandle, &req, selfEntity ) != SUCCESS )
    {
      GATT_bm_free( (gattMsg_t *)&req, ATT_WRITE_REQ );
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_setEvent
 *
 * @brief   Function to set event in the Simple BLE Peripheral task.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_setEvent(uint8_t newEvents)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_AUDIO_EVT, newEvents, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_setEvent
 *
 * @brief   Function to set event in the Simple BLE Peripheral task.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_clearEvent(uint16_t clearEvents)
{

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_isConnected
 *
 * @brief   Function to set event in the Simple BLE Peripheral task.
 *
 * @param   None.
 *
 * @return  TRUE if connected, FALSE if not.
 */
uint8_t SimpleBLEPeripheral_isConnected(void)
{
  uint8_t gapRoleState;
  GAPRole_GetParameter(GAPROLE_STATE, &gapRoleState);

  return (gapRoleState == GAPROLE_CONNECTED);
}

/*********************************************************************
 *********************************************************************/
