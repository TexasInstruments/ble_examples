/******************************************************************************

 @file  throughput_peripheral.c

 @brief This file contains the Throughput Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************

 Copyright (c) 2013-2017, Texas Instruments Incorporated
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
 Release Name: simplelink_cc2640r2_sdk_1_35_00_07_eng
 Release Date: 2017-03-23 10:36:21
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
#include "throughput_service.h"

#include "peripheral.h"

#include "board.h"

#include "board_key.h"

#include "throughput_peripheral_menu.h"

#include "throughput_peripheral.h"

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          100

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Application specific event ID for HCI Connection Event End Events
#define SBP_HCI_CONN_EVT_END_EVT              0x0001

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
    #define SBP_DISPLAY_TYPE Display_Type_LCD
  #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
    #define SBP_DISPLAY_TYPE Display_Type_UART
  #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
    #define SBP_DISPLAY_TYPE 0 // Option not supported
  #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#else // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #define SBP_DISPLAY_TYPE 0 // No Display
#endif // !Display_DISABLE_ALL

// Task configuration
#define SBP_TASK_PRIORITY                     1

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

#define SBP_STATE_CHANGE_EVT                  0x0001
//#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_KEY_CHANGE_EVT                    0x0002

// Internal Events for RTOS application
#define SBP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBP_THROUGHPUT_EVT                    Event_Id_00
#define SBP_PDU_CHANGE_EVT                    Event_Id_01
#define SBP_PHY_CHANGE_EVT                    Event_Id_02


#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT        | \
                                               SBP_QUEUE_EVT        | \
                                               SBP_THROUGHPUT_EVT   | \
                                               SBP_PDU_CHANGE_EVT   | \
                                               SBP_PHY_CHANGE_EVT)

// Row numbers
#define SBP_ROW_RESULT        TBM_ROW_APP
#define SBP_ROW_STATUS_1      (TBM_ROW_APP + 1)
#define SBP_ROW_STATUS_2      (TBM_ROW_APP + 2)
#define SBP_ROW_STATUS_3      (TBM_ROW_APP + 3)
#define SBP_ROW_ROLESTATE     (TBM_ROW_APP + 4)
#define SBP_ROW_BDADDR        (TBM_ROW_APP + 5)
#define SBP_ROW_ROLE          (TBM_ROW_APP + 6)

// For DLE
#define DLE_MAX_PDU_SIZE 251
#define DLE_MAX_TX_TIME 17040

#define DEFAULT_PDU_SIZE 27
#define DEFAULT_TX_TIME 328

// The combined overhead for L2CAP and ATT notification headers
#define TOTAL_PACKET_OVERHEAD 7

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

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
  0x12,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'T',
  'h',
  'r',
  'o',
  'u',
  'g',
  'h',
  'p',
  'u',
  't',
  ' ',
  'P',
  'e',
  'r',
  'i',
  'p',
  'h',
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
  LO_UINT16(THROUGHPUT_SERVICE_SERV_UUID),
  HI_UINT16(THROUGHPUT_SERVICE_SERV_UUID)
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Throughput";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Flag to Toggle Throughput Demo
static bool throughputOn = false;

// Message counter for Throughput Demo
static uint32 msg_counter = 1;

// Strings for PHY
static uint8_t* phyName[] = {
  "1 Mbps", "2 Mbps",
  "Coded:S2", "Coded:S8"
};

// PHY Options
static uint16_t phyOptions = HCI_PHY_OPT_NONE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

static void SBP_throughputOn(void);
static void SBP_throughputOff(void);

void SimpleBLEPeripheral_keyChangeHandler(uint8 keys);
static void SimpleBLEPeripheral_handleKeys(uint8_t keys);
static void SimpleBLEPeripheral_blastData();

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
  SimpleBLEPeripheral_stateChangeCB     // Profile State Change Callbacks
};

// Throughput GATT Profile Callbacks
static Throughput_ServiceCBs_t SimpleBLEPeripheral_throughputProfileCBs =
{
  SimpleBLEPeripheral_charValueChangeCB // Characteristic value change callback
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
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  dispHandle = Display_open(SBP_DISPLAY_TYPE, NULL);

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

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service


  // Setup the Throughput Characteristic Values
  {
    // Set Initial Values of Characteristics in GATT Table
    uint8_t pdu_size = DEFAULT_PDU_SIZE;
    uint8_t phy_supported = LL_PHY_1_MBPS;

    Throughput_Service_SetParameter(THROUGHPUT_SERVICE_UPDATE_PDU, sizeof(uint8_t),
                               &pdu_size);
    Throughput_Service_SetParameter(THROUGHPUT_SERVICE_UPDATE_PHY, sizeof(uint8_t),
                               &phy_supported);
  }

  // Initialize the GATT attributes
  Throughput_Service_AddService();      // Throughput Service

  // Register callbacks with Throughput Profile
  Throughput_Service_RegisterAppCBs(&SimpleBLEPeripheral_throughputProfileCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-docs-latest/docs/ble5stack/ble_user_guide/html/ble-stack/hci.html
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Init key debouncer
  Board_initKeys(SimpleBLEPeripheral_keyChangeHandler);

  // Set the title of the main menu
  TBM_SET_TITLE(&sbpMenuMain, "Texas Instruments Bluetooth 5 Demo");

  // Initialize Two-Button Menu module
  tbm_setItemStatus(&sbpMenuMain, TBM_ITEM_NONE, TBM_ITEM_ALL);
  tbm_initTwoBtnMenu(dispHandle, &sbpMenuMain, 3, NULL);

  // By Default Allow Central to support any and all PHYs
  HCI_LE_SetDefaultPhyCmd(LL_PHY_USE_ANY_PHY, LL_PHY_1_MBPS | LL_PHY_2_MBPS| HCI_PHY_CODED, LL_PHY_1_MBPS | LL_PHY_2_MBPS| HCI_PHY_CODED);

  // Set the Transmit Power of the Device to +5dBm
  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);

  // Set the RX Gain to be highest
  HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);

  // Start the Device
  VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);
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
            if (pEvt->event_flag & SBP_HCI_CONN_EVT_END_EVT)
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

      if (events & SBP_THROUGHPUT_EVT)
      {
        // Begin Throughput Demo in App Task
        SimpleBLEPeripheral_blastData();
      }

      if (events & SBP_PDU_CHANGE_EVT)
      {
        // Variables needed
        uint8_t newValue = 0;
        uint16_t connectionHandle = 0;

        // Get Prereq data.
        Throughput_Service_GetParameter(THROUGHPUT_SERVICE_UPDATE_PDU, &newValue);
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

        // DLE HCI command to adjust PDU size
        HCI_LE_SetDataLenCmd(connectionHandle, newValue, DLE_MAX_TX_TIME);
      }

      if (events & SBP_PHY_CHANGE_EVT)
      {
        // Variables needed
        uint8_t newValue = 0;
        uint16_t connectionHandle = 0;

        // Get Prereq data.
        Throughput_Service_GetParameter(THROUGHPUT_SERVICE_UPDATE_PHY, &newValue);
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

        // Which PHY is picked?
        static uint8_t phy[] = {
          HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_CODED, HCI_PHY_CODED
        };

        // Swtich to determine PHY options (needed for coded S2 and S8 mode)
        switch(newValue)
        {
        case 0:
        case 1:
          phyOptions = HCI_PHY_OPT_NONE;
          break;
        case 2:
          phyOptions = HCI_PHY_OPT_S2;
          break;
        case 3:
          phyOptions = HCI_PHY_OPT_S8;
          break;
        }

        // Set this device's Phy Preference on the current connection.
        HCI_LE_SetPhyCmd(connectionHandle, LL_PHY_USE_PHY_PARAM, phy[newValue], phy[newValue], phyOptions);
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

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          case HCI_LE_EVENT_CODE:
            {
              hciEvt_BLEPhyUpdateComplete_t *pPUC
                = (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

              if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
              {
                if (pPUC->status != SUCCESS)
                {
                  Display_print0(dispHandle, SBP_ROW_STATUS_1, 0, "PHY Change failure");
                }
                else
                {
                  Display_print0(dispHandle, SBP_ROW_STATUS_1, 0, "PHY Update Complete");

                  uint8_t index = 0;
                  switch(pPUC->rxPhy)
                  {
                    case HCI_PHY_1_MBPS:
                      index = 0;
                      break;

                    case HCI_PHY_2_MBPS:
                      index = 1;
                      break;

                    case HCI_PHY_CODED:
                      {
                        if(phyOptions == HCI_PHY_OPT_S2)
                          index = 2;
                        else if (phyOptions == HCI_PHY_OPT_S8)
                          index = 3;
                      }
                      break;
                  }
                  Display_print1(dispHandle, SBP_ROW_STATUS_3, 0, "Current PHY: %s",
                                 phyName[index]);

                }

                // Start Throughput
                SBP_throughputOn();
              }

              if (pPUC->BLEEventCode == HCI_BLE_DATA_LENGTH_CHANGE_EVENT)
              {
                // TX PDU Size Updated
                hciEvt_BLEDataLengthChange_t *dleEvt = (hciEvt_BLEDataLengthChange_t *)pMsg;
                Display_print1(dispHandle, SBP_ROW_STATUS_2, 0, "PDU Size: %dB", dleEvt->maxTxOctets);

                // Start Throughput
                SBP_throughputOn();
              }
            }
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
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_HCI_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_print1(dispHandle, SBP_ROW_RESULT, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_print1(dispHandle, SBP_ROW_RESULT, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
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
      Display_print1(dispHandle, SBP_ROW_STATUS_1, 0, "Rsp send retry: %d", rspTxRetry);
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
      Display_print1(dispHandle, SBP_ROW_STATUS_1, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, SBP_ROW_STATUS_1, 0, "Rsp retry failed: %d", rspTxRetry);
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
      SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

//    case SBP_CHAR_CHANGE_EVT:
//      SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
//      break;

    case SBP_KEY_CHANGE_EVT:
      SimpleBLEPeripheral_handleKeys(pMsg->hdr.state);
      break;

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
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
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
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        Display_print1(dispHandle, SBP_ROW_BDADDR, 0, "This Device's BDADDR : %s", Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "GAP STATE: Initialized");
        Display_print0(dispHandle, SBP_ROW_ROLE, 0, "Device GAP Role: Peripheral");
      }
      break;

    case GAPROLE_ADVERTISING:
      Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Advertising");
      break;

    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;

        numActive = linkDB_NumActive();

        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
          Display_print1(dispHandle, SBP_ROW_ROLESTATE, 0, "Num Conns: %d", (uint16_t)numActive);
          Display_print0(dispHandle, SBP_ROW_STATUS_1, 0, Util_convertBdAddr2Str(linkInfo.addr));
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

          Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Connected");
          Display_print0(dispHandle, SBP_ROW_STATUS_1, 0, Util_convertBdAddr2Str(peerAddress));
        }

        tbm_setItemStatus(&sbpMenuMain, TBM_ITEM_ALL, TBM_ITEM_NONE);
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Disconnected");

      // Disable PHY change
      tbm_setItemStatus(&sbpMenuMain, TBM_ITEM_NONE, TBM_ITEM_ALL);

      // Clear remaining lines
      Display_clearLines(dispHandle, SBP_ROW_RESULT, SBP_ROW_STATUS_2);
      Display_clearLines(dispHandle, SBP_ROW_RESULT, SBP_ROW_STATUS_3);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, SBP_ROW_RESULT, 0, "Timed Out");

      // Disable PHY change
      tbm_setItemStatus(&sbpMenuMain, TBM_ITEM_NONE, TBM_ITEM_ALL);

      // Clear remaining lines
      Display_clearLines(dispHandle, SBP_ROW_STATUS_1, SBP_ROW_STATUS_2);
      break;

    case GAPROLE_ERROR:
      Display_print0(dispHandle, SBP_ROW_RESULT, 0, "Error");
      break;

    default:
      Display_clearLines(dispHandle, SBP_ROW_RESULT, SBP_ROW_STATUS_2);
      break;
  }

  // Update the state
  //gapProfileState = newState;
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  // If we wish to process the message in the Application Context
  // We would utilize inter process communications such as below
  //    SimpleBLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID);

  // However, due to the throughput taking up all application task
  // processor time, we'll process the value change in the stack
  // context: here

  SimpleBLEPeripheral_processCharValueChangeEvt(paramID);

  // Note that if this takes too long, BLE connect can drop
  // This needs to be minimized to allow the stack to continue
  // to function correctly
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter index of characteristic changed
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{

  switch(paramID)
  {
    case THROUGHPUT_SERVICE_UPDATE_PDU:

      // Turn off Throughput - to allow application to process profile value change
      SBP_throughputOff();

      // Inform Application to update PDU
      Event_post(syncEvent, SBP_PDU_CHANGE_EVT);

      break;

    case THROUGHPUT_SERVICE_UPDATE_PHY:

      // Turn off Throughput - to allow application to process profile value change
      SBP_throughputOff();

      // Inform Application to update PDU
      Event_post(syncEvent, SBP_PHY_CHANGE_EVT);

      break;

    case THROUGHPUT_SERVICE_TOGGLE_THROUGHPUT:

      // Turn on Throughput - Turns on Throughput
      SimpleBLEPeripheral_doThroughputDemo(0);

      break;

    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys - bitmap of pressed keys
 *
 * @return  none
 */
void SimpleBLEPeripheral_keyChangeHandler(uint8 keys)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_KEY_CHANGE_EVT, keys);
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
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, syncEvent, (uint8*)pMsg);
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 *
 * @return  none
 */
static void SimpleBLEPeripheral_handleKeys(uint8_t keys)
{
  if (keys & KEY_LEFT)
  {
    // Check if the key is still pressed. WA for possible bouncing.
    if (PIN_getInputValue(Board_PIN_BUTTON0) == 0)
    {
      tbm_buttonLeft();
    }
  }
  else if (keys & KEY_RIGHT)
  {
    // Check if the key is still pressed. WA for possible bouncing.
    if (PIN_getInputValue(Board_PIN_BUTTON1) == 0)
    {
      tbm_buttonRight();
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_doSetPhy
 *
 * @brief   Set PHY preference.
 *
 * @param   index - 0, 1, 2(, 3, 4)
 *
 * @return  always true
 */
bool SimpleBLEPeripheral_doSetPhy(uint8 index)
{
  uint8_t gapRoleState;
  uint16_t connectionHandle;

  static uint8_t* phyName[] = {
    "1 Mbps", "2 Mbps",
    "Coded:S2", "Coded:S8",
  };

  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_CODED,
  };

  static uint8_t options[] = {
    HCI_PHY_OPT_NONE, HCI_PHY_OPT_NONE,
    HCI_PHY_OPT_S2, HCI_PHY_OPT_S8,
  };

  // Turn off Throughput while operation is in progress
  SBP_throughputOff();

  // Assign phyOptions
  phyOptions = options[index];

  GAPRole_GetParameter(GAPROLE_STATE, &gapRoleState);
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX.
  HCI_LE_SetPhyCmd(connectionHandle, 0, phy[index], phy[index], phyOptions);

  Display_print1(dispHandle, SBP_ROW_RESULT, 0, "PHY preference: %s", phyName[index]);

  Display_clearLine(dispHandle, SBP_ROW_STATUS_1);

  return true;
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_doSetDLEPDU
 *
 * @brief   Set PDU preference.
 *
 * @param   index - 0, 1
 *
 * @return  always true
 */
bool SimpleBLEPeripheral_doSetDLEPDU(uint8 index)
{
  // Vars to keep track of active packet length settings
  uint16_t txOctets;
  uint16_t txTime;
  uint8_t gapRoleState;
  uint16_t connectionHandle;

  switch (index)
  {
  case 0:
    txOctets = DEFAULT_PDU_SIZE;
    txTime = DEFAULT_TX_TIME;
    break;
  case 1:
    txOctets = DLE_MAX_PDU_SIZE;
    txTime = DLE_MAX_TX_TIME;
    break;
  }

  // Turn off Throughput while operation is in progress
  SBP_throughputOff();

  // Get GAP Params
  GAPRole_GetParameter(GAPROLE_STATE, &gapRoleState);

  if (GAPROLE_CONNECTED == gapRoleState)
  {
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

    // DLE HCI command to adjust PDU size
    HCI_LE_SetDataLenCmd(connectionHandle, txOctets, txTime);
  }
  else
  {
    // if not connected, write suggested default
    HCI_LE_WriteSuggestedDefaultDataLenCmd(txOctets, txTime);

    if (GAPROLE_INIT == gapRoleState)
    {
      // Go to Main Menu
      tbm_goTo(&sbpMenuMain);
    }
  }

  Display_print1(dispHandle, SBP_ROW_RESULT, 0, "PDU Size preference: %s Bytes",
                 (txOctets == DEFAULT_PDU_SIZE) ? "27" : "251");

  Display_clearLine(dispHandle, SBP_ROW_STATUS_1);

  return true;
}

static void SBP_throughputOn(void)
{
  // Turn on Throughput Demo & signal application
  throughputOn = true;

  Display_print0(dispHandle, SBP_ROW_RESULT, 0, "Throughput ON");

  Event_post(syncEvent, SBP_THROUGHPUT_EVT);
}

static void SBP_throughputOff(void)
{
  // Turn off Throughput Demo
  throughputOn = false;

  Display_print0(dispHandle, SBP_ROW_RESULT, 0, "Throughput OFF");
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_doThroughputDemo
 *
 * @brief   Set PDU preference.
 *
 * @param   index - 0, 1
 *
 * @return  always true
 */
bool SimpleBLEPeripheral_doThroughputDemo(uint8 index)
{

  // ignore unused error
  (void) index;

  if(throughputOn)
  {
    // Turn off Throughput Demo
    throughputOn = false;

    Display_print0(dispHandle, SBP_ROW_RESULT, 0, "Throughput OFF");
  }
  else
  {
    // Turn on Throughput Demo & signal application
    throughputOn = true;

    Display_print0(dispHandle, SBP_ROW_RESULT, 0, "Throughput ON");

    Event_post(syncEvent, SBP_THROUGHPUT_EVT);
  }

  return true;
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_blastData
 *
 * @brief   Sends ATT notifications in a tight while loop to demo
 *          throughput
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_blastData()
{
  // Subtract the total packet overhead of ATT and L2CAP layer from notification payload
  uint16_t len = MAX_PDU_SIZE-TOTAL_PACKET_OVERHEAD;
  attHandleValueNoti_t noti;
  bStatus_t status;
  noti.handle = 0x1E;
  noti.len = len;

  // Store hte connection handle for future reference
  uint16_t connectionHandle;
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

  while(throughputOn)
  {
    // If RTOS queue is not empty, process app message.
    // We need to process the app message here in the case of a keypress
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

    noti.pValue = (uint8 *)GATT_bm_alloc( connectionHandle, ATT_HANDLE_VALUE_NOTI, GATT_MAX_MTU, &len );

    if ( noti.pValue != NULL ) //if allocated
    {

      // Place index
      noti.pValue[0] = (msg_counter >> 24) & 0xFF;
      noti.pValue[1] = (msg_counter >> 16) & 0xFF;
      noti.pValue[2] = (msg_counter >> 8) & 0xFF;
      noti.pValue[3] = msg_counter & 0xFF;

      // Attempt to send the notification w/ no authentication
      status = GATT_Notification( connectionHandle, &noti, 0);
      if ( status != SUCCESS ) //if noti not sent
      {
        GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
      }
      else
      {
        // Notification is successfully sent, increment counters
        msg_counter++;
      }
    }
    else
    {
      // bleNoResources was returned
      asm(" NOP ");
    }
  }
}

/*********************************************************************
*********************************************************************/
