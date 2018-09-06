/*
 * Filename: peripheral_bidirectional_audio.c
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

// TI-RTOS Includes
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>


// BLE-Stack return codes
#include <inc/bcomdef.h>
/*
 * All BLE stack APIs come from here, do not include BLE protocol layer files
 * (i.e. gatt.h) directly!
 */
#include <icall/inc/icall_ble_api.h>

// Common code
#include <common/cc26xx/util.h>
#include <common/cc26xx/board_key.h>

// Gateway to board file
#include <target/board.h>

// GAP Role
#include <profiles/roles/cc26xx/peripheral.h>

// Audio Profile and Discovery code
#include <profiles/audio_dle/audio_duplex.h>
#include <profiles/audio_dle/audio_profile_dle.h>
#include <profiles/audio_dle/audio_client_disc.h>


// Application include file
#include "peripheral_bidirectional_audio.h"

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
#define PA_TASK_PRIORITY                     1


#ifndef PA_TASK_STACK_SIZE
#define PA_TASK_STACK_SIZE                   864
#endif

// Internal Events for RTOS application
#define PA_STATE_CHANGE_EVT                  0x0001
#define PA_PAIRING_STATE_EVT                 0x0002
#define PA_PASSCODE_NEEDED_EVT               0x0004
#define PA_CONN_EVT                          0x0008
#define PA_KEY_CHANGE_EVT                    0x0010
#define PA_AUDIO_EVT                         0x0020

// Internal Events for RTOS application
#define PA_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define PA_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define PA_PERIODIC_EVT                      Event_Id_00

#define PA_ALL_EVENTS                        (PA_ICALL_EVT        | \
                                               PA_QUEUE_EVT        | \
                                               PA_PERIODIC_EVT)

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connectionEventRegisterCauseBitMap |= RegisterCause )
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connectionEventRegisterCauseBitMap &= (~RegisterCause) )
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)
// Gets whether the RegisterCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connectionEventRegisterCauseBitMap & RegisterCause )

#define DLE_MAX_PDU_SIZE                      251
#define DLE_MAX_TX_TIME                       2120

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
  uint8_t *pData;  // event data
} sbpEvt_t;

typedef enum
{
   NOT_REGISTER       = 0,
   FOR_AOA_SCAN       = 1,
   FOR_ATT_RSP        = 2,
   FOR_AOA_SEND       = 4,
   FOR_TOF_SEND       = 8
}connectionEventRegisterCause_u;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t       connectionEventRegisterCauseBitMap = NOT_REGISTER; //see connectionEventRegisterCause_u


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
static uint16_t connHandle = INVALID_CONNHANDLE;

static PIN_Config PA_configTable[] =
{
 Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
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
Task_Struct paTask;
Char paTaskStack[PA_TASK_STACK_SIZE];

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
 // complete name
 0x16,   // length of this data
 GAP_ADTYPE_LOCAL_NAME_COMPLETE,
 'S','i','m','p','l','e','B','L','E',
 'A','u','d','i','o','T','x',
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

// Structure to store all handles found by AudioClientDisc module
static AudioClientDisc_handles_t  audioSvcHandles;

// Application state
static uint8_t state = BLE_STATE_IDLE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void PeripheralAudio_init( void );
static void PeripheralAudio_taskFxn(UArg a0, UArg a1);

static uint8_t PeripheralAudio_processStackMsg(ICall_Hdr *pMsg);
static uint8_t PeripheralAudio_processGATTMsg(gattMsgEvent_t *pMsg);
static void PeripheralAudio_processAppMsg(sbpEvt_t *pMsg);
static void PeripheralAudio_processStateChangeEvt(gaprole_States_t newState);
static void PeripheralAudio_sendAttRsp(void);
static void PeripheralAudio_freeAttRsp(uint8_t status);
static void PeripheralAudio_stateChangeCB(gaprole_States_t newState);
static uint8_t PeripheralAudio_enqueueMsg(uint8_t event, uint8_t state,
                                              uint8_t *pData);
static void PeripheralAudio_passcodeCB(uint8_t *deviceAddr,
                                            uint16_t connHandle,
                                            uint8_t uiInputs, uint8_t uiOutputs);
void PeripheralAudio_keyChangeHandler(uint8_t keys);
static void PeripheralAudio_handleKeys(uint8_t shift, uint8_t keys);

static void PeripheralAudio_pairStateCB(uint16_t connHandle, uint8_t state,
                                            uint8_t status);
static void PeripheralAudio_processPairState(uint8_t state, uint8_t status);
static void PeripheralAudio_processPasscode(uint8_t uiOutputs);
static void PeripheralAudio_EnableNotification( uint16_t connHandle,
                                                    uint16_t attrHandle );
static void PeripheralAudio_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void PeripheralAudio_processConnEvt(Gap_ConnEventRpt_t *pReport);
static bStatus_t PeripheralAudio_RegisterToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause);
static bStatus_t PeripheralAudio_UnRegisterToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t PeripheralAudio_gapRoleCBs =
{
 PeripheralAudio_stateChangeCB            // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
 (pfnPasscodeCB_t) PeripheralAudio_passcodeCB,  // Passcode callback
 PeripheralAudio_pairStateCB                    // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      PeripheralAudio_createTask
 *
 * @brief   Task creation function for the Peripheral Bidirectional Audio.
 *
 * @param   None.
 *
 * @return  None.
 */
void PeripheralAudio_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = paTaskStack;
  taskParams.stackSize = PA_TASK_STACK_SIZE;
  taskParams.priority = PA_TASK_PRIORITY;

  Task_construct(&paTask, PeripheralAudio_taskFxn, &taskParams, NULL);
}


/*********************************************************************
 * @fn      PeripheralAudio_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void PeripheralAudio_keyChangeHandler(uint8_t keys)
{
  PeripheralAudio_enqueueMsg(PA_KEY_CHANGE_EVT, keys, NULL);
}


/*********************************************************************
 * @fn      PeripheralAudio_setEvent
 *
 * @brief   Function to set event in the Peripheral Bidirectional Audio task.
 *
 * @param   None.
 *
 * @return  None.
 */
void PeripheralAudio_setEvent(uint8_t newEvents)
{
  PeripheralAudio_enqueueMsg(PA_AUDIO_EVT, newEvents, NULL);
}

/*********************************************************************
 * @fn      PeripheralAudio_init
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
static void PeripheralAudio_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  HCI_LE_WriteSuggestedDefaultDataLenCmd(DLE_MAX_PDU_SIZE , DLE_MAX_TX_TIME);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  dispHandle = Display_open(Display_Type_ANY, NULL);
  Display_printf(dispHandle, 0, 0, "\f");

  Board_initKeys(PeripheralAudio_keyChangeHandler);

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

  // Start the Device
  VOID GAPRole_StartDevice(&PeripheralAudio_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  HCI_LE_ReadMaxDataLenCmd();

  Display_printf(dispHandle, 0, 0, "Audio Peripheral with DLE");

  // Open pin structure for use
  hSbpPins = PIN_open(&sbpPins, PA_configTable);

  AudioDuplex_open(dispHandle, hSbpPins,
                   (pfnAudioDuplexCB_t)PeripheralAudio_setEvent);
}

/*********************************************************************
 * @fn      PeripheralAudio_taskFxn
 *
 * @brief   Application task entry point for the Peripheral Bidirectional Audio.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void PeripheralAudio_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  PeripheralAudio_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread

    events = Event_pend(syncEvent, Event_Id_NONE, PA_ALL_EVENTS,
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
        uint8_t safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = PeripheralAudio_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }
      // If RTOS queue is not empty, process app message.
      if (events & PA_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            PeripheralAudio_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t PeripheralAudio_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = PeripheralAudio_processGATTMsg((gattMsgEvent_t *)pMsg);
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
 * @fn      PeripheralAudio_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t PeripheralAudio_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (state == BLE_STATE_CONNECTED)
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. Let's try to retransmit the response
      // on the next connection event.
      if( PeripheralAudio_RegisterToAllConnectionEvent(FOR_ATT_RSP) == SUCCESS)
      {
        // First free any pending response
        PeripheralAudio_freeAttRsp(FAILURE);

        // Hold on to the response message for retransmission
        pAttRsp = pMsg;

        // Don't free the response message yet
        return (FALSE);
      }
    }

    if(!AudioClientDisc_isComplete())
    {
      AudioClientDisc_status_t status = AudioClientDisc_processGATTDisc(pMsg, selfEntity);

      if (status == AUDIO_CLIENT_DISC_COMPLETE)
      {
        Display_printf(dispHandle, 5, 0,
                        "Service Discovery Complete");
        PeripheralAudio_EnableNotification(pMsg->connHandle,
                                            audioSvcHandles.audioStartCCCHandle);
        PeripheralAudio_EnableNotification(pMsg->connHandle,
                                            audioSvcHandles.audioDataCCCHandle);
      }
    }

    switch ( pMsg->method )
    {
      case ATT_MTU_UPDATED_EVENT:
        // MTU size updated
        Display_printf(dispHandle, 4, 0,
                        "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
        break;
      case ATT_HANDLE_VALUE_NOTI:
        // Check to see if notification is from audio data or control char
        if (pMsg->msg.handleValueNoti.handle == audioSvcHandles.audioDataCharValueHandle)
        {
          AudioDuplex_audioData pData;
          pData.len = pMsg->msg.handleValueNoti.len;
          pData.pValue = pMsg->msg.handleValueNoti.pValue;
          AudioDuplex_processData(AudioDuplex_data, &pData);
        }
        else if (pMsg->msg.handleValueNoti.handle == audioSvcHandles.audioStartCharValueHandle)
        {
          AudioDuplex_audioData pData;
          pData.len = pMsg->msg.handleValueNoti.len;
          pData.pValue = pMsg->msg.handleValueNoti.pValue;
          AudioDuplex_processData(AudioDuplex_start_stop, &pData);
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
        break;
    }
  }

  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      PeripheralAudio_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void PeripheralAudio_sendAttRsp(void)
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
      // Disable connection event end CB
      PeripheralAudio_UnRegisterToAllConnectionEvent(FOR_ATT_RSP);

      // We're done with the response message
      PeripheralAudio_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_printf(dispHandle, 5, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void PeripheralAudio_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, 5, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_printf(dispHandle, 5, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void PeripheralAudio_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case PA_STATE_CHANGE_EVT:
    {
      PeripheralAudio_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;
    }

    case PA_KEY_CHANGE_EVT:
    {
      PeripheralAudio_handleKeys(0, pMsg->hdr.state);
      break;
    }

      // Pairing event
    case PA_PAIRING_STATE_EVT:
    {
      PeripheralAudio_processPairState(pMsg->hdr.state, *pMsg->pData);

      ICall_free(pMsg->pData);
      break;
    }

    // Passcode event
    case PA_PASSCODE_NEEDED_EVT:
    {
        PeripheralAudio_processPasscode(*pMsg->pData);

        ICall_free(pMsg->pData);
        break;
    }
    case PA_AUDIO_EVT:
    {
      AudioDuplex_eventHandler(pMsg->hdr.state);
      break;
    }
    case PA_CONN_EVT:
    {
      PeripheralAudio_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));

      ICall_free(pMsg->pData);
      break;
    }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void PeripheralAudio_stateChangeCB(gaprole_States_t newState)
{
  PeripheralAudio_enqueueMsg(PA_STATE_CHANGE_EVT, newState, 0);
}

/*********************************************************************
 * @fn      PeripheralAudio_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void PeripheralAudio_processStateChangeEvt(gaprole_States_t newState)
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
    {
      uint8_t ownAddress[B_ADDR_LEN];

      GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

      // Display device address
      Display_printf(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
      Display_printf(dispHandle, 2, 0, "Initialized");
      break;
    }
    case GAPROLE_ADVERTISING:
      Display_printf(dispHandle, 2, 0, "Advertising");
      break;

    case GAPROLE_CONNECTED:
    {
      state = BLE_STATE_CONNECTED;

      uint8_t peerAddress[B_ADDR_LEN];

      GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

      GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);

      AudioDuplex_setConnectionHandle(connHandle);

      Display_printf(dispHandle, 2, 0, "Connected");
      Display_printf(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));

      // Startup discovery processing on peer
      AudioClientDisc_open(&audioSvcHandles);

      break;
    }

    case GAPROLE_CONNECTED_ADV:
      Display_printf(dispHandle, 2, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      PeripheralAudio_freeAttRsp(bleNotConnected);

      Display_printf(dispHandle, 2, 0, "Disconnected");

      AudioDuplex_stopStreaming();

      // Reset char handles and disc state
      AudioClientDisc_close();

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);
      break;

      case GAPROLE_WAITING_AFTER_TIMEOUT:
      PeripheralAudio_freeAttRsp(bleNotConnected);

      Display_printf(dispHandle, 2, 0, "Timed Out");

      AudioDuplex_stopStreaming();

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);
      break;

    case GAPROLE_ERROR:
      Display_printf(dispHandle, 2, 0, "Error");
      break;

    default:
      Display_clearLine(dispHandle, 2);
      break;
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static uint8_t PeripheralAudio_enqueueMsg(uint8_t event, uint8_t state,
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
 * @fn      PeripheralAudio_handleKeys
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
static void PeripheralAudio_handleKeys(uint8_t shift, uint8_t keys)
{
  static uint8_t previousKeys = 0;
  uint16_t connectionHandle = AudioDuplex_getConnectionHandle();

  // Only process changes in keys pressed
  if (keys != previousKeys)
  {
    if(connectionHandle != INVALID_CONNHANDLE && linkDB_Up(connectionHandle))
    {
      // Check for both keys first
      if (keys == (KEY_LEFT | KEY_RIGHT))
      {
        // Start chain of events to stop stream
        AudioDuplex_stopStreaming();
      }
      else if (keys & KEY_LEFT)
      {
        AudioDuplex_startStreaming(AUDIO_DUPLEX_STREAM_TYPE_MSBC);
      }
      else if (keys & KEY_RIGHT)
      {
        AudioDuplex_startStreaming(AUDIO_DUPLEX_STREAM_TYPE_ADPCM);
      }
    }
    else
    {
      Display_printf(dispHandle, 2, 0, "Connection required for stream");
    }
  }
  previousKeys = keys;
}

/*********************************************************************
 * @fn      SimpleBLECentral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void PeripheralAudio_pairStateCB(uint16_t connHandle, uint8_t state,
                                            uint8_t status)
{
  uint8_t *pData;
  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;
    // Queue the event.
    PeripheralAudio_enqueueMsg(PA_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void PeripheralAudio_processPairState(uint8_t state, uint8_t status)
{
  Display_clearLines(dispHandle, 5, 5);
  switch (state)
  {
  case GAPBOND_PAIRING_STATE_STARTED:
    Display_printf(dispHandle, 2, 0, "Pairing started");
    break;

  case GAPBOND_PAIRING_STATE_COMPLETE:
    if (status == SUCCESS)
    {
      // Enter a GAP Bond manager Paired state
      Display_printf(dispHandle, 2, 0, "Pairing success");
    }
    else
    {
      Display_printf(dispHandle, 2, 0, "Pairing fail: %d", status);
    }
    break;

  case GAPBOND_PAIRING_STATE_BOND_SAVED:
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, 2, 0, "Bond Saved");
    }
    break;

  case GAPBOND_PAIRING_STATE_BONDED:
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, 2, 0, "Bond save success");
    }
    else
    {
      Display_printf(dispHandle, 2, 0, "Bond save failed: %d", status);
    }
    break;

  default:
    break;
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void PeripheralAudio_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                           uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    PeripheralAudio_enqueueMsg(PA_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void PeripheralAudio_processPasscode(uint8_t uiOutputs)
{
  // This app uses a default passcode. A real-life scenario would handle all
  // pairing scenarios and likely generate this randomly.
  uint32_t passcode = B_APP_DEFAULT_PASSCODE;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_printf(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  uint16_t connectionHandle;
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      PeripheralAudio_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void PeripheralAudio_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  if( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP))
  {
    // The GATT server might have returned a blePending as it was trying
    // to process an ATT Response. Now that we finished with this
    // connection event, let's try sending any remaining ATT Responses
    // on the next connection event.
    // Try to retransmit pending ATT Response (if any)
    PeripheralAudio_sendAttRsp();
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void PeripheralAudio_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if( PeripheralAudio_enqueueMsg(PA_CONN_EVT, 0 ,(uint8_t *) pReport) == FALSE)
  {
    ICall_freeMsg(pReport);
  }
}

/*********************************************************************
 * @fn      PeripheralAudio_RegisterToAllConnectionEvent()
 *
 * @brief   register to receive connection events for all the connection
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
static bStatus_t PeripheralAudio_RegisterToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  // in case  there is no registration for the connection event, make the registration
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    status = GAP_RegisterConnEventCb(PeripheralAudio_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
  }
  if(status == SUCCESS)
  {
    //add the reason bit to the bitamap.
    CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
  }

  return(status);
}

/*********************************************************************
 * @fn      PeripheralAudio_UnRegisterToAllConnectionEvent()
 *
 * @brief   Unregister connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
static bStatus_t PeripheralAudio_UnRegisterToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
  // in case  there is no more registration for the connection event than unregister
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    GAP_RegisterConnEventCb(PeripheralAudio_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
  }

  return(status);
}

/*********************************************************************
 * @fn      PeripheralAudio_EnableNotification
 *
 * @brief   Enable notification for a given attribute handle.
 *
 * @param   connHandle - connection handle to send notification on
 * @param   attrHandle - attribute handle to send notification for
 *
 * @return  none
 */
static void PeripheralAudio_EnableNotification( uint16_t connHandle,
                                                      uint16_t attrHandle )
{
  attWriteReq_t req;

  req.pValue = GATT_bm_alloc( connHandle, ATT_WRITE_REQ, 2, NULL );
  if ( req.pValue != NULL )
  {
    uint8_t notificationsOn[] = {0x01, 0x00};

    req.handle = attrHandle;

    req.len = 2;
    memcpy(req.pValue, notificationsOn, 2);

    req.sig = 0;
    req.cmd = TRUE;

    bStatus_t status = GATT_WriteNoRsp( connHandle, &req);

    if ( status != SUCCESS )
    {
      GATT_bm_free( (gattMsg_t *)&req, ATT_WRITE_REQ );
    }
  }
}

/*********************************************************************
 *********************************************************************/
