/*
 * Filename: central_bidirectional_audio.c
 *
 * Description: This is the audio part of the simple_central example
 * modified to send audio data over BLE.
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

// TI-RTOS Kernel Includes
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

// TI-RTOS Drivers
#include <ti/display/Display.h>
#include <ti/drivers/PIN.h>

// BLE-Stack return codes
#include <inc/bcomdef.h>
/*
 * All BLE stack APIs come from here, do not include BLE protocol layer files
 * (i.e. gatt.h) directly!
 */
#include <icall/inc/icall_ble_api.h>

// GAP Role
#include <profiles/roles/cc26xx/central.h>

// Common code
#include <common/cc26xx/util.h>
#include <common/cc26xx/board_key.h>

// Gateway to board file
#include <target/board.h>

#include <profiles/audio_dle/audio_duplex.h>
#include <profiles/audio_dle/audio_profile_dle.h>
#include <profiles/audio_dle/audio_client_disc.h>



#include "central_bidirectional_audio.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define CA_STATE_CHANGE_EVT                  0x0001
#define CA_KEY_CHANGE_EVT                    0x0002
#define CA_RSSI_READ_EVT                     0x0004
#define CA_PAIRING_STATE_EVT                 0x0008
#define CA_PASSCODE_NEEDED_EVT               0x0010
#define CA_SCANNING_TOGGLE_EVT               0x0020
#define CA_AUDIO_EVT                         0x0040

// Central Bidirectional Audio Task Events
#define CA_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define CA_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define CA_START_DISCOVERY_EVT               Event_Id_00

#define CA_ALL_EVENTS                        (CA_ICALL_EVT           |        \
                                               CA_QUEUE_EVT           |        \
                                               CA_START_DISCOVERY_EVT)

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

// Default passcode
#define DEFAULT_PASSCODE                      123456

#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  FALSE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SCANNING_TOGGLECLOCK          200

// Defines for using data length extension
#define DLE_MAX_PDU_SIZE                      251
#define DLE_MAX_TX_TIME                       2120

// Task configuration
#define CA_TASK_PRIORITY                     1

#ifndef CA_TASK_STACK_SIZE
#define CA_TASK_STACK_SIZE                   864
#endif

#define APP_MIN_CONN_INTERVAL                 8
#define APP_MAX_CONN_INTERVAL                 8

#define APP_SLAVE_LATENCY                     0
#define APP_CONN_TIMEOUT                      200    // 2s supervision timeout

#define CA_DISP_ROW_NAME                     0
#define CA_DISP_ROW_ADDR                     1
#define CA_DISP_ROW_CONN_STATE               2
#define CA_DISP_ROW_PEER_ADDR                3
#define CA_DISP_ROW_GEN_STATUS1              4
#define CA_DISP_ROW_GEN_STATUS2              5

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
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} caEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

bool resultFindRC;
bool resultFindST;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock object used to signal start/end scanning
static Clock_Struct scanningToggleClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct caTask;
Char caTaskStack[CA_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Central Duplex Audio ";

// Number of scan results and scan result index
static uint8_t scanRes = 0;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static bool scanningStarted = FALSE;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Maximum PDU size (default = 27 octets)
static uint16_t maxPduSize;

/* Pin driver handles */
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State allPinState;

static uint8_t peerAddr[B_ADDR_LEN] = {0,0,0,0,0,0};

// Structure to store all handles found by AudioClientDisc module
static AudioClientDisc_handles_t  audioSvcHandles;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void CentralAudio_init(void);
static void CentralAudio_taskFxn(UArg a0, UArg a1);

static void CentralAudio_processGATTMsg(gattMsgEvent_t *pMsg);
static void CentralAudio_handleKeys(uint8_t shift, uint8_t keys);
static void CentralAudio_processStackMsg(ICall_Hdr *pMsg);
static void CentralAudio_processAppMsg(caEvt_t *pMsg);
static void CentralAudio_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void CentralAudio_exchangeMTU(void);
static bool CentralAudio_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen);
static void CentralAudio_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void CentralAudio_processPairState(uint8_t state, uint8_t status);
static void CentralAudio_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);

static uint8_t CentralAudio_eventCB(gapCentralRoleEvent_t *pEvent);
static void CentralAudio_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs);
static void CentralAudio_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status);

void CentralAudio_keyChangeHandler(uint8_t keys);

static uint8_t CentralAudio_enqueueMsg(uint8_t event, uint8_t status,
                                           uint8_t *pData);

static uint8_t CentralAudio_FindHIDRemote( uint8_t* pData, uint8_t length );
static void CentralAudio_SetIdle( void );
static uint8_t CentralAudio_BondCount( void );
static void CentralAudio_EstablishLink( uint8_t whiteList, uint8_t addrType,
                                            uint8_t *peerAddr );
static void CentralAudio_EnableNotification( uint16_t connHandle,
                                                  uint16_t attrHandle );
static void CentralAudio_scanningToggleHandler(UArg a0);

static PIN_Config ledPinTable[] =
{
  // LEDs initially off
  Board_RLED   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_GLED   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t CentralAudio_roleCB =
{
  CentralAudio_eventCB     // Event callback
};

// Bond Manager Callbacks
static gapBondCBs_t CentralAudio_bondCB =
{
  (pfnPasscodeCB_t)CentralAudio_passcodeCB, // Passcode callback
  CentralAudio_pairStateCB                  // Pairing state callback
};

/*********************************************************************
 * @fn      CentralAudio_createTask
 *
 * @brief   Task creation function for the Central Bidirectional Audio.
 *
 * @param   none
 *
 * @return  none
 */
void CentralAudio_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = caTaskStack;
  taskParams.stackSize = CA_TASK_STACK_SIZE;
  taskParams.priority = CA_TASK_PRIORITY;

  Task_construct(&caTask, CentralAudio_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      CentralAudio_setEvent
 *
 * @brief   Function to set event in the Central Bidirectional Audio task.
 *
 * @param   None.
 *
 * @return  None.
 */
void CentralAudio_setEvent(uint8_t newEvents)
{
  CentralAudio_enqueueMsg(CA_AUDIO_EVT, newEvents, NULL);
}

/*********************************************************************
 * @fn      CentralAudio_Init
 *
 * @brief   Initialization function for the Central Bidirectional Audio App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void CentralAudio_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  HCI_LE_WriteSuggestedDefaultDataLenCmd(DLE_MAX_PDU_SIZE , DLE_MAX_TX_TIME);

  // Open all pins
  ledPinHandle = PIN_open(&allPinState, ledPinTable);

  // Turn on Red led to indicate nothing has been connected
  PIN_setOutputValue( ledPinHandle, Board_RLED, 1);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Periodic event to toggle LED to indicate if scanning is in progress
  Util_constructClock(&scanningToggleClock,
                      CentralAudio_scanningToggleHandler,
                      DEFAULT_SCANNING_TOGGLECLOCK, 0, false,
                      CA_SCANNING_TOGGLE_EVT);

  Board_initKeys(CentralAudio_keyChangeHandler);

  // Open Display.
  dispHandle = Display_open(Display_Type_ANY, NULL);
  Display_printf(dispHandle, CA_DISP_ROW_NAME, 0, "\f");

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

  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
  {
    // Don't send a pairing request after connecting; the device waits for the
    // application to start pairing
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    // Do not use authenticated pairing
    uint8_t mitm = DEFAULT_MITM_MODE;
    // This is a display only device
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    // Create a bond during the pairing process
    uint8_t bonding = DEFAULT_BONDING_MODE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE_NO_PAIRING, sizeof(uint8_t),
                            &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT Client
   GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Set connection parameters:
  {
    uint16_t minconnectionInterval = APP_MIN_CONN_INTERVAL;
    uint16_t maxconnectionInterval = APP_MAX_CONN_INTERVAL;

    uint16_t slaveLatency = APP_SLAVE_LATENCY;
    uint16_t timeout = APP_CONN_TIMEOUT;

     GAP_SetParamValue( TGAP_CONN_EST_INT_MIN, minconnectionInterval );
     GAP_SetParamValue( TGAP_CONN_EST_INT_MAX, maxconnectionInterval );
     GAP_SetParamValue( TGAP_CONN_EST_LATENCY, slaveLatency );
     GAP_SetParamValue( TGAP_CONN_EST_SUPERV_TIMEOUT, timeout );

  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Start the Device
   GAPCentralRole_StartDevice(&CentralAudio_roleCB);

  // Register with bond manager after starting device
  GAPBondMgr_Register(&CentralAudio_bondCB);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  Display_printf(dispHandle, CA_DISP_ROW_NAME, 0, "Audio Central with DLE");

  AudioDuplex_open(dispHandle, ledPinHandle,
                  (pfnAudioDuplexCB_t)CentralAudio_setEvent);
}

/*********************************************************************
 * @fn      CentralAudio_taskFxn
 *
 * @brief   Application task entry point for the Central Bidirectional Audio.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void CentralAudio_taskFxn(UArg a0, UArg a1)
{
  uint32_t events;
  // Initialize application
  CentralAudio_init();

  // Application main loop
  for (;;)
  {
    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, CA_ALL_EVENTS,
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
          CentralAudio_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message
      if (events & CA_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          caEvt_t *pMsg = (caEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message
            CentralAudio_processAppMsg(pMsg);

            // Free the space from the message
            ICall_free(pMsg);
          }
        }
      }

      if (events & CA_SCANNING_TOGGLE_EVT)
      {
          Util_startClock(&scanningToggleClock);
          PIN_setOutputValue(ledPinHandle, Board_GLED,
                              !PIN_getOutputValue(Board_GLED));
      }
    }
  }
}

/*********************************************************************
 * @fn      CentralAudio_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void CentralAudio_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      CentralAudio_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;

    case GATT_MSG_EVENT:
      CentralAudio_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
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
 * @fn      CentralAudio_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void CentralAudio_processAppMsg(caEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case CA_STATE_CHANGE_EVT:
    {
      CentralAudio_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;
    }

    case CA_KEY_CHANGE_EVT:
    {
      CentralAudio_handleKeys(0, pMsg->hdr.state);
      break;
    }
    // Pairing event
    case CA_PAIRING_STATE_EVT:
    {
        CentralAudio_processPairState(pMsg->hdr.state, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
    }

    // Passcode event
    case CA_PASSCODE_NEEDED_EVT:
    {
        CentralAudio_processPasscode(connHandle, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
    }

    case CA_AUDIO_EVT:
    {
      AudioDuplex_eventHandler(pMsg->hdr.state);
      break;
    }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      CentralAudio_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void CentralAudio_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  static uint8_t addrType;
  static uint8_t peerDeviceFound = FALSE;

  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        maxPduSize = pEvent->initDone.dataPktLen;

        Display_printf(dispHandle, CA_DISP_ROW_ADDR, 0,
                        Util_convertBdAddr2Str(pEvent->initDone.devAddr));

        Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0, "Initialized");
        if ( CentralAudio_BondCount() > 0 )
        {
          GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, NULL );
          Display_printf(dispHandle, CA_DISP_ROW_GEN_STATUS2, 0,
                          "Erase Bond info ");
        }
        else
        {
          // Sit idle till ask to scan
          CentralAudio_SetIdle();
        }
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if ((CentralAudio_findSvcUuid( AUDIO_SERV_UUID,
                                          pEvent->deviceInfo.pEvtData,
                                          pEvent->deviceInfo.dataLen)) )
        {
          CentralAudio_addDeviceInfo(pEvent->deviceInfo.addr,
                                         pEvent->deviceInfo.addrType);
          addrType = pEvent->deviceInfo.addrType;
          memcpy( peerAddr, pEvent->deviceInfo.addr, B_ADDR_LEN );
        }
        if (( pEvent->deviceInfo.eventType == GAP_ADRPT_SCAN_RSP ) &&
              CentralAudio_FindHIDRemote( pEvent->deviceInfo.pEvtData,
                                  pEvent->deviceInfo.dataLen ) )
        {
          // End device discovery
           GAPCentralRole_CancelDiscovery();
          peerDeviceFound = TRUE;
          addrType = pEvent->deviceInfo.addrType;
          memcpy( peerAddr, pEvent->deviceInfo.addr, B_ADDR_LEN );
        }
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
    {
      Util_stopClock(&scanningToggleClock);
      PIN_setOutputValue(ledPinHandle, Board_GLED, 0);
      // discovery complete
      scanningStarted = FALSE;
      // If we have found a connectable device, establish a connection
      if ( peerDeviceFound == TRUE )
      {

        CentralAudio_EstablishLink( DEFAULT_LINK_WHITE_LIST, addrType,
                                    peerAddr );

        peerDeviceFound = FALSE;
        scanRes = 0;
      }
      else if ( scanRes > 0 )
      {
        scanRes--;

        // Scan again
        // Begin scanning
        GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST );
      }
      else
      {
        // Go idle
        CentralAudio_SetIdle();
      }
    }
    break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;

          // Set Audio Cxn Handle
          AudioDuplex_setConnectionHandle(connHandle);

          Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0, "Connected");
          Display_printf(dispHandle, CA_DISP_ROW_PEER_ADDR, 0,
                          Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));
          PIN_setOutputValue(ledPinHandle, Board_GLED, 1);
          PIN_setOutputValue(ledPinHandle, Board_RLED, 0);

          // Start MTU exchange
          CentralAudio_exchangeMTU();

          // Startup discovery processing on peer
          AudioClientDisc_open(&audioSvcHandles);
        }
        else if ( CentralAudio_BondCount() > 0 )
        {
          // Re-initiate connection
          CentralAudio_EstablishLink( TRUE, addrType, peerAddr );
        }
        else
        {
          connHandle = GAP_CONNHANDLE_INIT;

          // Go idle
          CentralAudio_SetIdle();
          Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0,
                          "Connect Failed");
          Display_printf(dispHandle, CA_DISP_ROW_PEER_ADDR, 0,
                          "Reason: %d", pEvent->gap.hdr.status);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;

        Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0, "Disconnected");
        Display_printf(dispHandle, CA_DISP_ROW_PEER_ADDR, 0,
                        "Reason: %d", pEvent->linkTerminate.reason);
        Display_clearLine(dispHandle, 4);
        Display_clearLine(dispHandle, 6);

        AudioDuplex_stopStreaming();

        PIN_setOutputValue(ledPinHandle, Board_GLED, 0);
        PIN_setOutputValue(ledPinHandle, Board_RLED, 1);

        if ( CentralAudio_BondCount() > 0 )
        {
          // Re-initiate connection
          CentralAudio_EstablishLink( TRUE, addrType, peerAddr );
        }
        else
        {
          // Go idle
          CentralAudio_SetIdle();

          // Reset char handles and disc state
          AudioClientDisc_close();
        }
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0,
                        "Param Update: %d", pEvent->linkUpdate.status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      CentralAudio_handleKeys
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
static void CentralAudio_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter
  static uint8_t previousKeys = 0;
  // Only process changes in keys pressed
  if (keys != previousKeys) {
    if (keys & KEY_LEFT)
    {
        // If not connected
        if (state == BLE_STATE_IDLE)
        {
            // If not currently scanning
            if (!scanningStarted)
            {
                scanningStarted = TRUE;
                scanRes = 0;

                Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0,
                                "Discovering...");
                Display_clearLines(dispHandle, CA_DISP_ROW_PEER_ADDR, 5);
                PIN_setOutputValue( ledPinHandle, Board_RLED, 0);

                GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                              DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                              DEFAULT_DISCOVERY_WHITE_LIST);

                Util_startClock(&scanningToggleClock);
            }
        }
        return;
    }

    if (keys & KEY_RIGHT)
    {
        // If bonds exist, erase all of them
        if ( ( CentralAudio_BondCount() > 0 ) &&
             ( state != BLE_STATE_CONNECTED ) )
        {
            if ( state == BLE_STATE_CONNECTING )
            {
                state = BLE_STATE_DISCONNECTING;
                 GAPCentralRole_TerminateLink( GAP_CONNHANDLE_INIT );
            }

             GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, NULL );
            Display_printf(dispHandle, CA_DISP_ROW_GEN_STATUS2, 0,
                            "Erase Bond info ");

            // Turn on green LED
            PIN_setOutputValue( ledPinHandle, Board_GLED, 1);

            // Wait half a second
            Task_sleep(50000);

            // Turn off green LED
            PIN_setOutputValue( ledPinHandle, Board_GLED, 0);
        }

        return;
    }
  }
  previousKeys = keys;
}

/*********************************************************************
 * @fn      CentralAudio_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void CentralAudio_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (state == BLE_STATE_CONNECTED)
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      Display_printf(dispHandle, CA_DISP_ROW_GEN_STATUS1, 0,
                      "ATT Rsp dropped %d", pMsg->method);
    };

    if(!AudioClientDisc_isComplete())
    {
      AudioClientDisc_status_t status = AudioClientDisc_processGATTDisc(pMsg, selfEntity);

      if (status == AUDIO_CLIENT_DISC_COMPLETE)
      {
        Display_printf(dispHandle, CA_DISP_ROW_GEN_STATUS2, 0,
                        "Service Discovery Complete");
        CentralAudio_EnableNotification(pMsg->connHandle,
                                            audioSvcHandles.audioStartCCCHandle);
        CentralAudio_EnableNotification(pMsg->connHandle,
                                            audioSvcHandles.audioDataCCCHandle);
      }
    }

    switch ( pMsg->method )
    {
      case ATT_MTU_UPDATED_EVENT:
      {
        if (pMsg->method == ATT_MTU_UPDATED_EVENT)
        {
          // MTU size updated
          Display_printf(dispHandle, CA_DISP_ROW_GEN_STATUS1, 0,
                          "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
        }
        break;
      }
      case ATT_HANDLE_VALUE_NOTI:
      {
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
      }
      // Service Change indication
      case ATT_HANDLE_VALUE_IND:
      {
        // Note: this logic assumes that the only indications that will be sent
        //       will come from that GATT Service Changed Characteristic
        if ( pMsg->hdr.status == SUCCESS )
        {
          // Acknowledge receipt of indication
          ATT_HandleValueCfm( pMsg->connHandle );
        }
        break;
      }

      default:
        break;
    }
  }

  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      CentralAudio_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void CentralAudio_processPairState(uint8_t state, uint8_t status)
{
  Display_clearLines(dispHandle, CA_DISP_ROW_GEN_STATUS2, 5);

  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
     if (status == SUCCESS)
    {
      // Enter a GAP Bond manager Paired state
      Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0, "Pairing success");
    }
    else
    {
      Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0,
                      "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0, "Bond Saved");
    }
    else
    {
      Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0,
                      "Bond save failed: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0,
                      "Bond save success");
    }
    else
    {
      Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0,
                      "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      CentralAudio_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void CentralAudio_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs)
{
  uint32_t  passcode;

  // Create random passcode
  passcode = Util_GetTRNG();
  passcode %= 1000000;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_printf(dispHandle, CA_DISP_ROW_GEN_STATUS1, 0,
                    "Passcode: %d", passcode);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      CentralAudio_exchangeMTU
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void CentralAudio_exchangeMTU(void)
{
  attExchangeMTUReq_t req;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
   GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      CentralAudio_EnableNotification
 *
 * @brief   Enable notification for a given attribute handle.
 *
 * @param   connHandle - connection handle to send notification on
 * @param   attrHandle - attribute handle to send notification for
 *
 * @return  none
 */
static void CentralAudio_EnableNotification( uint16_t connHandle,
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
 * @fn      CentralAudio_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool CentralAudio_findSvcUuid(uint16_t uuid, uint8_t *pData,
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
          (adType == GAP_ADTYPE_16BIT_COMPLETE) ||
          (adType == GAP_ADTYPE_MANUFACTURER_SPECIFIC))
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
 * @fn      CentralAudio_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void CentralAudio_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
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
 * @fn      CentralAudio_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t CentralAudio_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (CentralAudio_enqueueMsg(CA_STATE_CHANGE_EVT,
                                  SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      CentralAudio_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void CentralAudio_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    CentralAudio_enqueueMsg(CA_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      CentralAudio_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void CentralAudio_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    CentralAudio_enqueueMsg(CA_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      CentralAudio_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void CentralAudio_keyChangeHandler(uint8_t keys)
{
  CentralAudio_enqueueMsg(CA_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      CentralAudio_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t CentralAudio_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  caEvt_t *pMsg = ICall_malloc(sizeof(caEvt_t));

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
 * @fn      CentralAudio_FindHIDRemote
 *
 * @brief   Search Scan Response data for a "HID AdvRemote"
 *
 * @param   pData - received advertising data
 * @param   dataLen - advertising data length
 *
 * @return  TRUE if found, false otherwise
 */
static uint8_t CentralAudio_FindHIDRemote( uint8_t* pData, uint8_t length )
{
  resultFindRC = FALSE;
  resultFindST = FALSE;
  static uint8_t remoteNameRC[] =
  {
    'C', 'C', '2', '6', '5', '0', ' ', 'R', 'C'
  };
  static uint8_t remoteNameTx[] =
  {
      'S', 'i', 'm', 'p', 'l', 'e',
      'B', 'L', 'E',
      'A', 'u', 'd', 'i', 'o', 'T', 'x'
  };

  // move pointer to the start of the scan response data.
  pData += 2;

  // adjust length as well
  length -= 2;

  if (length == sizeof(remoteNameRC))
  {
      resultFindRC = !(memcmp( remoteNameRC, pData, length ));
  }
  else if (length == sizeof(remoteNameTx))
  {
      resultFindRC = !(memcmp( remoteNameTx, pData, length ));
  }

  // did not find RC, then search for ST
  if (!resultFindRC)
  {

    static uint8_t remoteNameST[] =
    {
      'C', 'C', '2', '6', '5', '0', ' ',
      'S', 'e', 'n',  's',  'o',  'r',  'T',  'a',  'g'
    };

    length -= 9;
    resultFindST = !(memcmp( remoteNameST, pData, length ));
  }
  return (resultFindRC || resultFindST);
}

/*********************************************************************
 * @fn      CentralAudio_BondCount
 *
 * @brief   Gets the total number of bonded devices.
 *
 * @param   none.
 *
 * @return  number of bonded devices.
 */
static uint8_t CentralAudio_BondCount( void )
{
  uint8_t bondCnt = 0;

   GAPBondMgr_GetParameter( GAPBOND_BOND_COUNT, &bondCnt );

  return ( bondCnt );
}

/*********************************************************************
 * @fn      CentralAudio_SetIdle
 *
 * @brief   Set the device to idle.
 *
 * @param   none
 *
 * @return  none
 */
static void CentralAudio_SetIdle( void )
{
  state = BLE_STATE_IDLE;
  Util_stopClock(&scanningToggleClock);
  PIN_setOutputValue( ledPinHandle, Board_GLED, 0);
  PIN_setOutputValue( ledPinHandle, Board_RLED, 1);
  Display_printf(dispHandle, CA_DISP_ROW_CONN_STATE, 0, "Idle...");
}

/*********************************************************************
 * @fn      CentralAudio_EstablishLink
 *
 * @brief   Establish a link to a peer device.
 *
 * @param   whiteList - determines use of the white list
 * @param   addrType - address type of the peer devic
 * @param   peerAddr - peer device address
 *
 * @return  none
 */
static void CentralAudio_EstablishLink( uint8_t whiteList, uint8_t addrType,
                                            uint8_t *peerAddr )
{
  if ( state != BLE_STATE_CONNECTED )
  {
    state = BLE_STATE_CONNECTING;

    // Try to connect to remote device
    GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                 whiteList,
                                 addrType, peerAddr);
  }
}

/*********************************************************************
 * @fn      CentralAudio_scanningToggleHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void CentralAudio_scanningToggleHandler(UArg arg)
{
    Event_post(syncEvent, CA_SCANNING_TOGGLE_EVT);
}

/*********************************************************************
*********************************************************************/
