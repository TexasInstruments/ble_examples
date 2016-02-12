/*
 * ancsApp.c
 *
 * Application task functionality for the ANCS demo
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
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ICall.h>
#include "bcomdef.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "gatt_profile_uuid.h"
#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
#include "board_key.h"
#include "board_lcd.h"
#include "Board.h"
#include <driverlib/trng.h>

#include "ancsapp.h"
   
#include "ancs.h"
   
#include <ti/drivers/lcd/LCDDogm1286.h>

/*********************************************************************
 * MACROS
 */

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
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Internal events for RTOS application.
#define ANCSAPP_STATE_CHANGE_EVT              0x0001
#define ANCSAPP_PASSCODE_EVT                  0x0002
#define ANCSAPP_PAIRING_COMPLETE              0x0004
#define ANCSAPP_BONDING_COMPLETE              0x0008
#define ANCSAPP_KEY_CHANGE_EVT                0x0010
#define ANCSAPP_START_DISC_EVT                0x0020

// Task configuration
#define ANCSAPP_TASK_PRIORITY                 1
#define ANCSAPP_TASK_STACK_SIZE               612

// ANCS Service: 7905F431-B5CE-4E99-A40F-4B1E122D00D0
#define ANCS_SVC_UUID 0xD0, 0x00, 0x2D, 0x12, 0x1E, 0x4B, 0x0F, 0xA4, 0x99, 0x4E, 0xCE, 0xB5, 0x31, 0xF4, 0x05, 0x79
// Notification Source: UUID 9FBF120D-6301-42D9-8C58-25E699A21DBD (notifiable)
#define ANCS_NOTIF_SRC_CHAR_UUID        0x1DBD // Last 2 bytes of the 128bit-16bytes UUID
// Control point: UUID 69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9 (writeable with response)
#define ANCS_CTRL_PT_CHAR_UUID          0xD9D9
// Data Source: UUID 22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB (notifiable)
#define ANCS_DATA_SRC_CHAR_UUID         0x7BFB
#define CHAR_DESC_HDL_UUID128_LEN        21  // 5 + 16bytes = 21

#define BLINK_DURATION                        20 // Milliseconds

// Application states
enum
{
  ANCS_STATE_IDLE = 0,
  ANCS_STATE_DISCOVERY,
  ANCS_STATE_READY,
};

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct 
{
  uint16_t event;              // Which profile's event.
  uint8_t state;              // New state.
} AncsAppEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint16_t Ancs_handleCache[HDL_CACHE_LEN];
/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t configCCCDState;
extern uint8_t clientFlags;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages.
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread.
static ICall_Semaphore sem;

// Handle for the Message Queue.
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// GAP State.
static gaprole_States_t AncsAppGapState = GAPROLE_INIT;
//Cache of ANCS service handles
static uint8_t ancsAppState = ANCS_STATE_IDLE;
//Subscribed

// Task pending events.
static uint16_t events = 0;

// Task configuration.
Task_Struct AncsApp_task;
Char AncsApp_taskStack[ANCSAPP_TASK_STACK_SIZE];


// GAP Profile - Name attribute for SCAN RSP data.
static uint8_t AncsApp_scanData[] =
{
  // local name
  0x09,
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'A','N','C','S',' ','A','p','p',
  
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

static uint8_t AncsApp_advData[] = 
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02, // length of this data  
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  // Service Solicitation: this peripheral (NC) is looking for the ANCS service
  // on the iOS device. As per Apple Bluetooth Design Guidelines, soliciting
  // the ANCS service will cause the device to show up in the iOS settings app
  0x11, // length of this data
  GAP_ADTYPE_SERVICES_LIST_128BIT,
  // ANCS service UUID
  ANCS_SVC_UUID
};

// Device name attribute value.
static uint8_t AncsApp_deviceName[GAP_DEVICE_NAME_LEN] = "ANCS App";

// Pairing state callback event variables.
static uint8_t passCode_uiOutputs = 0;

//! \brief PIN Config LEDs
static PIN_Config ledPinsCfg[] =
{
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

//! \brief PIN State LEDs
static PIN_State ledPins;

//! \brief PIN Handles for LED pins
static PIN_Handle hledPins;

// LCD parameters
LCD_Handle lcdHandle;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

// Application tasks and event processing.
static void AncsApp_init(void);
static void AncsApp_taskFxn(UArg a0, UArg a1);
static uint8_t AncsApp_enqueueMsg(uint8_t event, uint8_t status);
static void AncsApp_processStackMsg(ICall_Hdr *pMsg);
static void AncsApp_processAppMsg(AncsAppEvt_t *pMsg);
static void AncsApp_processGattMsg(gattMsgEvent_t *pMsg);
static void AncsApp_disconnected(void);

// Peripheral role.
static void AncsApp_stateChangeCB(gaprole_States_t newState);
static void AncsApp_stateChangeEvt(gaprole_States_t newState);

// Passcode.
static void AncsApp_passcodeCB(uint8_t *deviceAddr, uint16_t connectionHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs);
static void AncsApp_passcodeEvt(uint16_t connectionHandle, uint8_t uiOutputs);
// Pair state.
static void AncsApp_pairStateCB(uint16_t connHandle, uint8_t state, 
                                      uint8_t status);
static void AncsApp_pairStateEvt(uint8_t state, uint8_t status);

// Service discovery
void AncsApp_discoverService(gattMsgEvent_t *pMsg);

// Keys.
void AncsApp_keyPressCB(uint8 keys);
static void AncsApp_handleKeysEvt(uint8_t keys);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t AncsApp_peripheralCB =
{
  AncsApp_stateChangeCB,          // Profile State Change Callbacks
};

// Bond Manager Callbacks
static const gapBondCBs_t AncsApp_bondCB =
{
  AncsApp_passcodeCB,             //  Passcode callback
  AncsApp_pairStateCB             //  Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AncsApp_createTask
 *
 * @brief   Task creation function for the Time Application.
 *
 * @param   none
 *
 * @return  none
 */
void AncsApp_createTask(void)
{
  Task_Params taskParams;
    
  // Configure task.
  Task_Params_init(&taskParams);
  taskParams.stack = AncsApp_taskStack;
  taskParams.stackSize = ANCSAPP_TASK_STACK_SIZE;
  taskParams.priority = ANCSAPP_TASK_PRIORITY;
  
  Task_construct(&AncsApp_task, AncsApp_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      AncsApp_init
 *
 * @brief   Initialization function for the Time Application Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ...).
 *
 * @param   None.
 *
 * @return  None.
 */
void AncsApp_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the DB Address till CC2650 board gets its own IEEE address.
  //uint8_t bdAddress[B_ADDR_LEN] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22 };
  //HCI_EXT_SetBDADDRCmd(bdAddress);
  
  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(40);
  
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
  
  Board_initKeys(AncsApp_keyPressCB);
  hledPins = PIN_open(&ledPins, ledPinsCfg);

  Board_openLCD();
  //Send the form feed char to the LCD, this is helpful if using a terminal
  //as it will clear the terminal history
  LCD_WRITE_STRING("\f", LCD_PAGE0);
  /*------------------------ GAP Settings ------------------------------------*/
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts  upon initialization
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

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(AncsApp_scanData),
                         AncsApp_scanData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, 
                         sizeof(AncsApp_advData), AncsApp_advData);

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
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, AncsApp_deviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  /*------------------------ Bond Manager Settings ---------------------------*/
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;
    uint16_t slaveSecurityReqWait = 0;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_INITIATE_WAIT, sizeof(uint16_t), 
                            &slaveSecurityReqWait);
  }
  
  /*------------------------ GATT Client Settings ----------------------------*/
  VOID GATT_InitClient();
  // Register to receive incoming ATT Indications/Notifications.
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes.
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes
  DevInfo_AddService();                      // Device Information Service
  
  // Start the Device.
  VOID GAPRole_StartDevice(&AncsApp_peripheralCB);

  // Register with bond manager after starting device.
  GAPBondMgr_Register((gapBondCBs_t *) &AncsApp_bondCB);
}

/*********************************************************************
 * @fn      AncsApp_taskFxn
 *
 * @brief   Time Application Task entry point.  This function
 *          is called to initialize and then process all events for the task.  
 *          Events include timers, messages and any other user defined events.
 *
 * @param   a0, a1 - not used.
 *
 * @return  none.
 */
void AncsApp_taskFxn(UArg a0, UArg a1)
{
  AncsApp_init();
  
  // Application main loop.
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
          // Process inter-task message.
          AncsApp_processStackMsg((ICall_Hdr *)pMsg);
        }
                
        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }
    // If RTOS queue is not empty, process app message.
    if (!Queue_empty(appMsgQueue))
    {
      AncsAppEvt_t *pMsg = 
        (AncsAppEvt_t*)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message.
        AncsApp_processAppMsg(pMsg);
        
        // Free the space from the message.
        ICall_free(pMsg);
      }
    }
    //If there is an important alert like an incoming call
    //turn on the LED to notify the user
    if(clientFlags & CLIENT_IMPORTANT_ALERT)
    {
      PIN_setOutputValue(hledPins, Board_LED1, Board_LED_ON);
    }
    else
    {
      PIN_setOutputValue(hledPins, Board_LED1, Board_LED_OFF);
    }
    // Service discovery event.
    if (events & ANCSAPP_START_DISC_EVT)
    {
      events &= ~ANCSAPP_START_DISC_EVT;
      //This event will kick off service discovery.
      //The event is only called once, when the GAPROLE is connected
      AncsApp_discoverService(NULL);
    } 
    // Passcode event.
    if (events & ANCSAPP_PASSCODE_EVT)
    {
      events &= ~ANCSAPP_PASSCODE_EVT;
      AncsApp_passcodeEvt(Ancs_connHandle, passCode_uiOutputs);
      // Clear event variables.
      passCode_uiOutputs = 0;
    }
    
  }
}

/*********************************************************************
 * @fn      AncsApp_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void AncsApp_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      AncsApp_processGattMsg((gattMsgEvent_t *) pMsg);
      break;
      
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      AncsApp_processAppMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void AncsApp_processAppMsg(AncsAppEvt_t *pMsg)
{
  switch (pMsg->event)
  {
    case ANCSAPP_STATE_CHANGE_EVT:
      AncsApp_stateChangeEvt((gaprole_States_t)pMsg->state);
      break;
    case ANCSAPP_KEY_CHANGE_EVT:
      AncsApp_handleKeysEvt(pMsg->state);
      break;
    case ANCSAPP_BONDING_COMPLETE:
      AncsApp_pairStateEvt(pMsg->event, pMsg->state);
      break;
    case ANCSAPP_PAIRING_COMPLETE:
      AncsApp_pairStateEvt(pMsg->event, pMsg->state);
      break;
      
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      AncsApp_processGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void AncsApp_processGattMsg(gattMsgEvent_t *pMsg)
{
  //Discovery takes priority
  if(ancsAppState == ANCS_STATE_DISCOVERY)
  {
    AncsApp_discoverService(pMsg);
  }
  //If we have received a notification or indication, assume its ANCS
  //process in ANCS client
  else if (pMsg->method == ATT_HANDLE_VALUE_NOTI ||
       pMsg->method == ATT_HANDLE_VALUE_IND)
  {
    // Now in ancs_client.c
    Ancs_handleNotification(pMsg);
  }
  //ANCS requires authentication, if the NP attempts to read/write chars on the
  //NP without proper authentication, the NP will respond with insufficent_athen
  //error to which we must respond with a slave security request
  else if  (pMsg->method == ATT_ERROR_RSP &&
            pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ &&
            pMsg->msg.errorRsp.errCode == ATT_ERR_INSUFFICIENT_AUTHEN)
  {
    uint16 conn_handle;
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &conn_handle);
    uint8_t mitm;
    uint8_t bonding;
    GAPBondMgr_GetParameter(GAPBOND_MITM_PROTECTION, &mitm);
    GAPBondMgr_GetParameter(GAPBOND_BONDING_ENABLED, &bonding);
    uint8_t authRequest = ((mitm & 0x01) << 2) | ((bonding & 0x01) << 1) |
                          (bonding & 0x01);

    GAP_SendSlaveSecurityRequest(conn_handle, authRequest);
  }
  //If we have received a read or write response, assume that it is related to 
  //CCCD configuration
  else if (pMsg->method == ATT_READ_RSP || pMsg->method == ATT_WRITE_RSP)
  {
  }
  //Otherwise subscribe to notification source if ANCS discovery is complete
  else
  {
  }
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      AncsApp_disconnected
 *
 * @brief   Handle disconnect. 
 *
 * @return  none
 */

static void AncsApp_disconnected(void)
{
  // Initialize state variables
  //AncsApp_discState = DISC_IDLE;

  Ancs_unSubsNotifSrc();
  Ancs_unSubsDataSrc();
  
  // Invalidate connection variables.
  Ancs_connHandle = INVALID_CONNHANDLE;
}

/*********************************************************************
 * @fn      AncsApp_discoverService
 *
 * @brief   Function to handle the discovery of the ANCS service
 *
 * @param   pMsg - GATT message to process, may be NULL in DISC_ANCS_START 
 *
 * @return  none
 */
void AncsApp_discoverService(gattMsgEvent_t *pMsg)
{
  static uint8_t discoveryState = DISC_ANCS_START;
  static uint16_t Ancs_svcStartHdl;
  static uint16_t Ancs_svcEndHdl;
  static uint8_t Ancs_endHdlIdx;
  static uint8_t isNotifCCCD = FALSE;
  
  switch (discoveryState)
  {
    case DISC_ANCS_START:  
      {
        uint8_t uuid[ATT_UUID_SIZE] = {ANCS_SVC_UUID};

        // Initialize service discovery variables
        Ancs_svcStartHdl = Ancs_svcEndHdl = 0;
        Ancs_endHdlIdx = 0;
        
        // Discover ANCS service by UUID
        GATT_DiscPrimaryServiceByUUID(Ancs_connHandle, uuid,
                                      ATT_UUID_SIZE, ICall_getEntityId());      

        discoveryState = DISC_ANCS_SVC;
      }
      break;

    case DISC_ANCS_SVC:
      // Service found, store handles
      if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
          pMsg->msg.findByTypeValueRsp.numInfo > 0)
      {
        Ancs_svcStartHdl = 
          ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        Ancs_svcEndHdl = 
          ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      }
      
      // If procedure complete
      if ((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
           pMsg->hdr.status == bleProcedureComplete) ||
          (pMsg->method == ATT_ERROR_RSP))
      {
        // If service found
        if (Ancs_svcStartHdl != 0)
        {
          // Discover all characteristics
          GATT_DiscAllChars(Ancs_connHandle, Ancs_svcStartHdl,
                            Ancs_svcEndHdl, ICall_getEntityId());
          
          discoveryState = DISC_ANCS_CHAR;
        }
        else
        {
          // Service not found
          discoveryState = DISC_FAILED;
          LCD_WRITE_STRING("ANCS SVC not present", LCD_PAGE6);
        }
      }    
      break;

    case DISC_ANCS_CHAR:
      {
        // Characteristics found, chache them
        uint8_t   *pHandleValuePairList;
        uint16_t  handle;
        uint16_t  uuid;
        if (pMsg->method == ATT_READ_BY_TYPE_RSP &&
            pMsg->msg.readByTypeRsp.numPairs > 0 && 
            pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID128_LEN)
        {
          pHandleValuePairList = pMsg->msg.readByTypeRsp.pDataList;
          uint8_t i;
          // For each handle value pair in the list of chars in ANCS service
          for (i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i--)
          {
            // Parse characteristic declaration
            handle = BUILD_UINT16(pHandleValuePairList[3], 
                                  pHandleValuePairList[4]);
            uuid = BUILD_UINT16(pHandleValuePairList[5], 
                                pHandleValuePairList[6]);
                   
            // If looking for end handle
            if (Ancs_endHdlIdx != 0)
            {
              // End handle is characteristic declaration handle - 1
              Ancs_handleCache[Ancs_endHdlIdx] = 
                BUILD_UINT16(pHandleValuePairList[0], 
                             pHandleValuePairList[1]) - 1;
              
              Ancs_endHdlIdx = 0;
            }

            // If UUID is of interest, cache handle
            switch (uuid)
            {
              case ANCS_NOTIF_SRC_CHAR_UUID:
                Ancs_handleCache[HDL_ANCS_NTF_NOTIF_START] = handle;
                Ancs_endHdlIdx = HDL_ANCS_NTF_NOTIF_END;
                break;
                
              case ANCS_CTRL_PT_CHAR_UUID:
                Ancs_handleCache[HDL_ANCS_CTRL_PT_START] = handle;
                Ancs_endHdlIdx = HDL_ANCS_CTRL_PT_END;
                break;
                
              case ANCS_DATA_SRC_CHAR_UUID:
                Ancs_handleCache[HDL_ANCS_DATA_SRC_START] = handle;
                Ancs_endHdlIdx = HDL_ANCS_DATA_SRC_END;
                break;
                
              default:
                break;
            }
            
            pHandleValuePairList += CHAR_DESC_HDL_UUID128_LEN;
          }
        }
          
        // If procedure complete
        if ((pMsg->method == ATT_READ_BY_TYPE_RSP  && 
             pMsg->hdr.status == bleProcedureComplete) ||
            (pMsg->method == ATT_ERROR_RSP))
        {
          // Special case of end handle at end of service
          if (Ancs_endHdlIdx != 0)
          {
            Ancs_handleCache[Ancs_endHdlIdx] = Ancs_svcEndHdl;
            Ancs_endHdlIdx = 0;
          }
          
          // If notification source char is missing, there is something wrong
          if (Ancs_handleCache[HDL_ANCS_NTF_NOTIF_START] == 0)
          {
            LCD_WRITE_STRING("Notif src not found", LCD_PAGE6);
            discoveryState = DISC_FAILED;
          }
          else if (Ancs_handleCache[HDL_ANCS_NTF_NOTIF_START] <
                   Ancs_handleCache[HDL_ANCS_NTF_NOTIF_END])
          {
            // Discover ANCS Notification Source CCCD
            GATT_DiscAllCharDescs(Ancs_connHandle,
                                  Ancs_handleCache[HDL_ANCS_NTF_NOTIF_START] + 1,
                                  Ancs_handleCache[HDL_ANCS_NTF_NOTIF_END],
                                  ICall_getEntityId());
            isNotifCCCD = TRUE;                            
            discoveryState = DISC_ANCS_CCCD;
          }
          else
          {
            LCD_WRITE_STRING("CCCD missing", LCD_PAGE6);
            // Missing required characteristic descriptor
            Ancs_handleCache[HDL_ANCS_NTF_NOTIF_START] = 0;
            discoveryState = DISC_FAILED;
          }
        }
        // Discover all characteristics of ANCS service until end handle
        else 
        {
          GATT_DiscAllChars(Ancs_connHandle, handle+1, 
                            Ancs_svcEndHdl, ICall_getEntityId());

        }

      }      
      break;

    case DISC_ANCS_CCCD:
      {
        // Characteristic descriptors found
        if (pMsg->method == ATT_FIND_INFO_RSP &&
            pMsg->msg.findInfoRsp.numInfo > 0 && 
            pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE)
        {
          uint8_t i;
          // For each handle/uuid pair
          for (i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++)
          {
            // Look for CCCD
            if (ATT_BT_PAIR_UUID(pMsg->msg.findInfoRsp.pInfo, i) ==
                GATT_CLIENT_CHAR_CFG_UUID)
            {
              // CCCD found
              // if it is Notification Source CCCD
              if (isNotifCCCD == TRUE)
              {
                Ancs_handleCache[HDL_ANCS_NTF_CCCD] =
                  ATT_BT_PAIR_HANDLE(pMsg->msg.findInfoRsp.pInfo, i);
              }
              // else it is Data Source CCCD
              else
              {
                Ancs_handleCache[HDL_ANCS_DATA_SRC_CCCD] =
                  ATT_BT_PAIR_HANDLE(pMsg->msg.findInfoRsp.pInfo, i);
              }
              break;
            }
          }
        }
        
        // If procedure complete
        if ((pMsg->method == ATT_FIND_INFO_RSP  && 
             pMsg->hdr.status == bleProcedureComplete) ||
            (pMsg->method == ATT_ERROR_RSP))
        {
          // Discover ANCS Data Source characteristic descriptors
          if (isNotifCCCD == TRUE && 
              Ancs_handleCache[HDL_ANCS_DATA_SRC_CCCD] == 0)
          {
            GATT_DiscAllCharDescs(Ancs_connHandle,
                                  Ancs_handleCache[HDL_ANCS_DATA_SRC_START] + 1,
                                  Ancs_handleCache[HDL_ANCS_DATA_SRC_END],
                                  ICall_getEntityId());
 
            isNotifCCCD = FALSE;
            
          }
          else
          {
            discoveryState = DISC_IDLE;
            ancsAppState = ANCS_STATE_READY;
            while (SUCCESS != Ancs_subsNotifSrc()){}
            while (SUCCESS != Ancs_subsDataSrc()){}
          }
        }
      }
      break;

    default:
      break;
  }
}

/*------------------------ Event Handler Functions ---------------------------*/
/*********************************************************************
 * @fn      AncsApp_handleKeysEvt
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events.
 *
 * @return  none
 */
static void AncsApp_handleKeysEvt(uint8_t keys)
{
  //If both keys are pressed
  if (keys == KEY_ID_BUTTON_BOTH)
  {
    // If connected, terminate connection
    if (AncsAppGapState == GAPROLE_CONNECTED)
    {
      LCD_WRITE_STRING("Bonds Erased", LCD_PAGE6);
      bStatus_t check = GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, 0, NULL);
      //Now we force disconnect
      if(SUCCESS == check)
        GAPRole_TerminateConnection();
    }
    else
    {
      // Clear all stored bonds.
      LCD_WRITE_STRING("Bonds Erased", LCD_PAGE6);
      GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, 0, 0);
    }
  }
  //else if a single key is pressed
  else
  {
    //Left(BTN-1) key on CC2650LP
    //Up key on SRF06
    if (keys & KEY_ID_BUTTON0)
    {
      Ancs_performPositiveAction();
    }
    //Right(BTN-2) key on CC2650LP
    //Down key on SRF06
    if (keys & KEY_ID_BUTTON1)
    {
      Ancs_performNegativeAction();
    }
  }
}

/*********************************************************************
 * @fn      AncsApp_stateChangeEvt
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void AncsApp_stateChangeEvt(gaprole_States_t newState)
{
  //If we have started the GAPRole, set/disp our own addr
  if (newState == GAPROLE_STARTED)
  {
    PIN_setOutputValue(hledPins, Board_LED2, Board_LED_OFF);
    uint8_t bdAddr[B_ADDR_LEN];
    
    GAPRole_GetParameter(GAPROLE_BD_ADDR, &bdAddr);
    LCD_WRITE_STRING("ANCS App", LCD_PAGE0);
    LCD_WRITE_STRING(Util_convertBdAddr2Str(bdAddr),  LCD_PAGE1);
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
  }
  // if connected, check for stored bonds, update LCD with peer addr
  else if (newState == GAPROLE_CONNECTED)
  {
    PIN_setOutputValue(hledPins, Board_LED2, Board_LED_OFF);
    //resolve address
    uint8 resolv_addr[B_ADDR_LEN];
    uint8 devAddr[B_ADDR_LEN];
    uint16 conn_handle;
    uint8 bond_count;
    uint8 addrType;

    // check for an existing bond, value of zero indicates no stored bonds in NV
    GAPBondMgr_GetParameter(GAPBOND_BOND_COUNT, &bond_count);
    if (bond_count)
    {
      // we have at least one stored bond. 
      // resolve the address before proceeding with the connection
      GAPRole_GetParameter(GAPROLE_CONNHANDLE, &conn_handle);
      GAPRole_GetParameter(GAPROLE_BD_ADDR_TYPE, &addrType);
      GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, &devAddr);
      uint8 idx = GAPBondMgr_ResolveAddr( addrType, devAddr, resolv_addr );
      if (idx != GAP_BONDINGS_MAX) //if address found in bond manager
      {
        asm("NOP");  //continue with connection
      }
      else
      {
        //disconnect, does not match any bonded address in NV
        LCD_WRITE_STRING("Cxn terminate", LCD_PAGE2);
        GAPRole_TerminateConnection();
      }
    }
      
    LCD_WRITE_STRING("ANCS Connected", LCD_PAGE2);
    LCD_WRITE_STRING(Util_convertBdAddr2Str(resolv_addr), LCD_PAGE3);
    events |= ANCSAPP_START_DISC_EVT;
    ancsAppState = ANCS_STATE_DISCOVERY;
  }
  // if advertising stopped, restart adv
  else if (newState == GAPROLE_WAITING)
  {
    PIN_setOutputValue(hledPins, Board_LED2, Board_LED_OFF);
    LCD_WRITE_STRING("Connection Terminated", LCD_PAGE2);
    AncsApp_disconnected();
    // if advertising stopped by user
    uint8_t advState = TRUE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advState);
  }
  // If started, notify user, wait for cxn
  else if (newState == GAPROLE_ADVERTISING)
  {
    PIN_setOutputValue(hledPins, Board_LED2, Board_LED_ON);
    LCD_WRITE_STRING("ANCS Advertising", LCD_PAGE2);
    LCD_WRITE_STRING("", LCD_PAGE3);
    LCD_WRITE_STRING("", LCD_PAGE4);
  }
}

/*********************************************************************
 * @fn      AncsApp_passcodeEvt
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void AncsApp_passcodeEvt(uint16_t connectionHandle, uint8_t uiOutputs)
{
#ifdef TI_DRIVERS_LCD_INCLUDED

  // Create random passcode
  uint32_t  passcode = Util_GetTRNG();
  passcode %= 1000000;
  // Display passcode to user
  if (uiOutputs != 0)
  {
    LCD_WRITE_STRING_VALUE("Passcode:", passcode, 10,  LCD_PAGE4);
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
#endif
}

/*********************************************************************
 * @fn      AncsApp_pairStateEvt
 *
 * @brief   Pairing state event processor.
 *
 * @return  none
 */
static void AncsApp_pairStateEvt(uint8_t state, uint8_t status)
{
  //Device is paired and bonded
  if(state == ANCSAPP_BONDING_COMPLETE)
  {
    if (status == SUCCESS)
    { 
      LCD_WRITE_STRING("Bonding success", LCD_PAGE4);
    }
    else
    {
      LCD_WRITE_STRING_VALUE("Pairing fail", status, 10, LCD_PAGE4);
    }
  }
  //else device has paired
  else if (state == ANCSAPP_PAIRING_COMPLETE)
  {
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING("Pairing success", LCD_PAGE4);
      while (SUCCESS != Ancs_subsNotifSrc()){}
      while (SUCCESS != Ancs_subsDataSrc()){}
      Semaphore_post(sem);
    }
  }
}

/*------------------------ Application Callbacks -----------------------------*/
/*********************************************************************
 * @fn      AncsApp_passcodeCB
 *
 * @brief   Passcode callback from GAP Bond Manager
 *
 * @param   deviceAddr - peer dev addr
 * @param   connectionHandle - handle of device pairing to
 * @param   uiInputs - peer's user interface inputs
 * @param   uiOutputs - peer's user interface outputs
 *
 * @return  none
 */
static void AncsApp_passcodeCB(uint8_t *deviceAddr, uint16_t connectionHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs)
{
  // Set the event.
  events |= ANCSAPP_PASSCODE_EVT;
  
  // Store argument for event processing.
  passCode_uiOutputs = uiOutputs;
  
  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      AncsApp_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @param   connHandle -handle of connected peer
 * @param   state - state of device pairing process
 * @param   status - status of pairing process

 *
 * @return  none
 */
static void AncsApp_pairStateCB(uint16_t connHandle, uint8_t state, 
                                      uint8_t status)
{
  if( GAPBOND_PAIRING_STATE_BONDED == state ||
      GAPBOND_PAIRING_STATE_BOND_SAVED == state)
  {
    AncsApp_enqueueMsg(ANCSAPP_BONDING_COMPLETE, status);
  }
  else if (GAPBOND_PAIRING_STATE_COMPLETE == state)
  {
    AncsApp_enqueueMsg(ANCSAPP_PAIRING_COMPLETE, status);
  }
}

/*********************************************************************
 * @fn      AncsApp_stateChangeCB
 *
 * @brief   Function to handle GAP role state changes
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void AncsApp_stateChangeCB(gaprole_States_t newState)
{
  AncsApp_enqueueMsg(ANCSAPP_STATE_CHANGE_EVT, newState); 
}

/*********************************************************************
 * @fn      AncsApp_keyPressCB
 *
 * @brief   keys - bitfield of pressed keys 
 *
 * @return  none
 */
void AncsApp_keyPressCB(uint8 keys)
{
  // Enqueue the event.
  AncsApp_enqueueMsg(ANCSAPP_KEY_CHANGE_EVT, keys);
}
/*------------------------ Utility Functions ---------------------------------*/
/*********************************************************************
 * @fn      AncsApp_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   status - message status.
 *
 * @return  TRUE or FALSE
 */
static uint8_t AncsApp_enqueueMsg(uint8_t event, uint8_t status)
{
  AncsAppEvt_t *pMsg;
  
  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(AncsAppEvt_t)))
  {
    pMsg->event = event;
    pMsg->state = status;
    
    // Enqueue the message
    uint8_t status = Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
    return status;
  }
  
  return FALSE;
}

/*********************************************************************
*********************************************************************/
