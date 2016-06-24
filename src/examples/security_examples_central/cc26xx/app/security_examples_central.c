/*******************************************************************************
  Filename:       just_works.c
  Revised:        $Date$
  Revision:       $Revision$

  Description:    This file contains the basic sample central application to implement
                  legacy just works pairing for use with the CC2650 Bluetooth 
                  Low Energy Protocol Stack.

 Copyright (c) 2013-2016, Texas Instruments Incorporated
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
*******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"

#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "hci.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include <ti/mw/display/Display.h>
#include "util.h"
#include "board_key.h"
#include "board.h"

#include "security_examples_central.h"

#include <ti/mw/lcd/LCDDogm1286.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Simple BLE Central Task Events
#define SEC_PAIRING_STATE_EVT                 0x0001
#define SEC_KEY_CHANGE_EVT                    0x0002
#define SEC_STATE_CHANGE_EVT                  0x0004
#define SEC_PASSCODE_NEEDED_EVT               0x0008

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

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Task configuration
#define SEC_TASK_PRIORITY                     1

#ifndef SEC_TASK_STACK_SIZE
#define SEC_TASK_STACK_SIZE                   864
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
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data 
} secEvt_t;

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

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct secTask;
Char secTaskStack[SEC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Security Ex Centr";

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

//oob data needed for either type of OOB pairing
#if ((PAIRING == OOB_SC) || (PAIRING == OOB_LE))
// LOCAL OOB DATA
uint8 oobLocal[KEYLEN] = { 0xA3, 0xDE, 0xBB, 0x31, 0xE6, 0x42, 0x4E, 0x2F,
                  0x39, 0x7F, 0xF2, 0xD2, 0xC4, 0x89, 0xC6, 0xA7 };
#endif

//ECC keys needed when using static keys with OOB secure connections pairing
#if ((PAIRING == OOB_SC) && (STATIC_KEYS))
//LOCAL KEYS
gapBondEccKeys_t eccKeys =
{
  .privateKey = {0x15, 0x99, 0x87, 0x83, 0xc7, 0x84, 0x05, 0x92, 0x35, 0x9e, 
                 0x54, 0x2c, 0x77, 0x61, 0xb5, 0xd6, 0x0a, 0x80, 0x67, 0x5d, 
                 0xe8, 0x62, 0xd5, 0xe0, 0xeb, 0xce, 0x76, 0xc7, 0x7b, 0xc2, 
                 0xfb, 0x43},
  .publicKeyX = {0xca, 0x42, 0x2f, 0xc3, 0x4c, 0xe5, 0x03, 0x9a, 0x94, 0x06, 
                 0x26, 0x6d, 0xd8, 0x22, 0x51, 0x30, 0xe6, 0x04, 0xd7, 0x4b, 
                 0x9b, 0xc3, 0x1e, 0x45, 0xde, 0x5e, 0x3d, 0x5d, 0xb0, 0x1a, 
                 0xe4, 0xaa},
  .publicKeyY = {0x03, 0xc8, 0xbf, 0xd1, 0x00, 0xc6, 0x10, 0xb5, 0xec, 0x33, 
                 0x0c, 0x39, 0x8d, 0xa9, 0xcf, 0x87, 0x36, 0x27, 0xe9, 0x02, 
                 0x27, 0x28, 0xad, 0xc1, 0xb0, 0x40, 0xae, 0x97, 0x47, 0x66, 
                 0x8f, 0xb4}
};
#endif

//needed for all types of pairing besides OOB
#if !(((PAIRING == OOB_SC) || (PAIRING == OOB_LE)))
// Passcode variables
static uint8_t judgeNumericComparison = FALSE;
static uint8_t waiting_for_passcode = FALSE;
static uint32_t passcode = 0;
static uint32_t passcode_multiplier = 100000;
static uint16_t passcode_connHandle = 0xFFFF;
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void security_examples_central_init(void);
static void security_examples_central_taskFxn(UArg a0, UArg a1);

static void security_examples_central_handleKeys(uint8_t shift, uint8_t keys);
static void security_examples_central_processStackMsg(ICall_Hdr *pMsg);
static void security_examples_central_processAppMsg(secEvt_t *pMsg);
static void security_examples_central_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void security_examples_central_processPasscode(uint16_t connectionHandle,
                                              gapPasskeyNeededEvent_t *pData);

static void security_examples_central_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void security_examples_central_processPairState(uint8_t state, uint8_t status);

static uint8_t security_examples_central_eventCB(gapCentralRoleEvent_t *pEvent);
static void security_examples_central_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);
static void security_examples_central_pairStateCB(uint16_t connHandle, uint8_t state, 
                                         uint8_t status);

void security_examples_central_keyChangeHandler(uint8 keys);

static uint8_t security_examples_central_enqueueMsg(uint8_t event, uint8_t status, 
                                           uint8_t *pData);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t security_examples_central_roleCB =
{
  security_examples_central_eventCB     // Event callback
};

// Bond Manager Callbacks
static gapBondCBs_t security_examples_central_bondCB =
{
  (pfnPasscodeCB_t)security_examples_central_passcodeCB, // Passcode callback
  security_examples_central_pairStateCB                  // Pairing state callback
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
void security_examples_central_createTask(void)
{
  Task_Params taskParams;
    
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = secTaskStack;
  taskParams.stackSize = SEC_TASK_STACK_SIZE;
  taskParams.priority = SEC_TASK_PRIORITY;
  
  Task_construct(&secTask, security_examples_central_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      security_examples_central_Init
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
static void security_examples_central_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  uint8 bdAddr[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
  HCI_EXT_SetBDADDRCmd(bdAddr);  
  
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
     
  Board_initKeys(security_examples_central_keyChangeHandler);
  
  dispHandle = Display_open(Display_Type_LCD, NULL);
  
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
  
  //Setup the Gap Bond Manager
  {
    //common GAPBondMgr params
    uint8_t pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    uint8_t bonding = FALSE;    
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);      
    
    //initializtion for secure connections OOB
#if (PAIRING == OOB_SC)
    uint8_t scMode = GAPBOND_SECURE_CONNECTION_ONLY;
    uint8_t oobEnabled = TRUE;
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &scMode);
    GAPBondMgr_SetParameter(GAPBOND_LOCAL_OOB_SC_ENABLED, sizeof(uint8_t), &oobEnabled );
    GAPBondMgr_SetParameter(GAPBOND_LOCAL_OOB_SC_DATA, sizeof(uint8_t) * KEYLEN, oobLocal);
    // if using static ECC keys
#if STATIC_KEYS
    GAPBondMgr_SetParameter(GAPBOND_ECC_KEYS, sizeof(gapBondEccKeys_t), &eccKeys);   
#endif //STATIC_KEYS    
  
    //initialization for legacy OOB pairing  
#elif (PAIRING == OOB_LE)
    uint8_t scMode = GAPBOND_SECURE_CONNECTION_NONE;
    uint8_t oobEnabled = TRUE;
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &scMode);    
    GAPBondMgr_SetParameter(GAPBOND_OOB_DATA, sizeof(uint8_t) * KEYLEN, oobLocal);
    GAPBondMgr_SetParameter(GAPBOND_OOB_ENABLED, sizeof(uint8_t), &oobEnabled );    
  
  //initialization for numeric comparison pairing (only possible with secure connections) 
#elif (PAIRING == NUMCOMP)
  uint8_t mitm = TRUE;
  uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_YES_NO;
  uint8_t scMode = GAPBOND_SECURE_CONNECTION_ONLY;
  GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
  GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
  GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &scMode);
  
  //initialization for passcode entry pairing
#elif (PAIRING == PASSCODE)
  uint8_t mitm = TRUE;
  uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
  uint8_t scMode = GAPBOND_SECURE_CONNECTION_ALLOW;
  GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
  GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
  GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &scMode);
  
  //initialization for just works pairing
#elif (PAIRING == JUSTWORKS)
  uint8_t mitm = FALSE;
  uint8_t scMode = GAPBOND_SECURE_CONNECTION_ALLOW;
  GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
  GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &scMode);
#endif 
  }
  
  // Initialize GATT Client
  VOID GATT_InitClient();

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes
  
  // Start the Device
  VOID GAPCentralRole_StartDevice(&security_examples_central_roleCB);

  // Register with bond manager after starting device
  GAPBondMgr_Register(&security_examples_central_bondCB);

  Display_print0(dispHandle, LCD_PAGE0, 0, "Security Ex Centr");
}

/*********************************************************************
 * @fn      security_examples_central_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void security_examples_central_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  security_examples_central_init();
  
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
          security_examples_central_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    // If RTOS queue is not empty, process app message
    while (!Queue_empty(appMsgQueue))
    {
      secEvt_t *pMsg = (secEvt_t *)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message
        security_examples_central_processAppMsg(pMsg);
        
        // Free the space from the message
        ICall_free(pMsg);
      }
    }    
  }
}

/*********************************************************************
 * @fn      security_examples_central_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void security_examples_central_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      security_examples_central_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      security_examples_central_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void security_examples_central_processAppMsg(secEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SEC_STATE_CHANGE_EVT:
      security_examples_central_processStackMsg((ICall_Hdr *)pMsg->pData);
      
      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;
      
    case SEC_KEY_CHANGE_EVT:
      security_examples_central_handleKeys(0, pMsg->hdr.state); 
      break;
      
    // Pairing event  
    case SEC_PAIRING_STATE_EVT:
      {
        security_examples_central_processPairState(pMsg->hdr.state, *pMsg->pData);
        
        ICall_free(pMsg->pData);
        break;
      }
      
    // Passcode event    
    case SEC_PASSCODE_NEEDED_EVT:
      {     
        security_examples_central_processPasscode(connHandle, (gapPasskeyNeededEvent_t *)pMsg->pData);
        
        ICall_free(pMsg->pData);
        break;
      }
      
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      security_examples_central_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void security_examples_central_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {        
        Display_print0(dispHandle, LCD_PAGE1, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
        Display_print0(dispHandle, LCD_PAGE2, 0, "Initialized");
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        security_examples_central_addDeviceInfo(pEvent->deviceInfo.addr, 
                                       pEvent->deviceInfo.addrType);
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        scanningStarted = FALSE;

        // Copy results
        scanRes = pEvent->discCmpl.numDevs;
        memcpy(devList, pEvent->discCmpl.pDevList,
               (sizeof(gapDevRec_t) * scanRes));
        
        Display_print1(dispHandle, LCD_PAGE2, 0, "Devices Found %d", scanRes);
        
        if (scanRes > 0)
        {
          Display_print0(dispHandle, LCD_PAGE3, 0, "<- To Select");
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

          Display_print0(dispHandle, LCD_PAGE2, 0, "Connected");
          Display_print0(dispHandle, LCD_PAGE3, 0, Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));   
        }
        else
        {
          state = BLE_STATE_IDLE;
          connHandle = GAP_CONNHANDLE_INIT;
          
          Display_print0(dispHandle, LCD_PAGE2, 0, "Connect Failed");
          Display_print1(dispHandle, LCD_PAGE3, 0, "Reason: %d", pEvent->gap.hdr.status);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;
        
        Display_print0(dispHandle, LCD_PAGE2, 0, "Disconnected");
        Display_print1(dispHandle, LCD_PAGE3, 0, "Reason: %d", pEvent->linkTerminate.reason);
        Display_print0(dispHandle, LCD_PAGE4, 0, "");
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        Display_print1(dispHandle, LCD_PAGE2, 0, "Param Update: %d", pEvent->linkUpdate.status);
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      security_examples_central_handleKeys
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
static void security_examples_central_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter

  if (keys & KEY_LEFT)
  {
    // Display discovery results
    if (!scanningStarted && scanRes > 0)
    {
      // Increment index of current result (with wraparound)
      scanIdx++;
      if (scanIdx >= scanRes)
      {
        scanIdx = 0;
      }

      Display_print1(dispHandle, LCD_PAGE2, 0, "Device %d", (scanIdx + 1));
      Display_print0(dispHandle, LCD_PAGE3, 0, Util_convertBdAddr2Str(devList[scanIdx].addr));
    }

    return;
  }

  if (keys & KEY_UP)
  {
    // Start or stop discovery
    if (state != BLE_STATE_CONNECTED)
    {
      if (!scanningStarted)
      {
        scanningStarted = TRUE;
        scanRes = 0;
        
        Display_print0(dispHandle, LCD_PAGE2, 0, "Discovering...");
        Display_print0(dispHandle, LCD_PAGE3, 0, "");
        Display_print0(dispHandle, LCD_PAGE4, 0, "");
        
        GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST);      
      }
      else
      {
        GAPCentralRole_CancelDiscovery();
      }
    }
    return;
  }

//needed for all types of pairing besides OOB
#if !(((PAIRING == OOB_SC) || (PAIRING == OOB_LE)))
  if (keys & KEY_RIGHT) 
  {
    if (waiting_for_passcode)
    {
      //increment passcode digit
      passcode += passcode_multiplier;
      Display_print1(dispHandle, LCD_PAGE5, 0, "%d",passcode);
      return;
    }
    else if (judgeNumericComparison)
    {
      judgeNumericComparison = FALSE;
      
      // overload 3rd parameter as TRUE when instead of the passcode when
      // numeric comparisons is used.
      GAPBondMgr_PasscodeRsp(connHandle, SUCCESS, TRUE);      
      Display_print0(dispHandle, LCD_PAGE5, 0, "Codes Match!");
      return;
    }
  }
#endif  

  if (keys & KEY_SELECT)
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
  
        Display_print0(dispHandle, LCD_PAGE2, 0, "Connecting");
        Display_print0(dispHandle, LCD_PAGE3, 0, Util_convertBdAddr2Str(peerAddr));
        Display_print0(dispHandle, LCD_PAGE4, 0, "");
      }
    }
    else if (state == BLE_STATE_CONNECTING ||
              state == BLE_STATE_CONNECTED)
    {
      // disconnect
      state = BLE_STATE_DISCONNECTING;

      GAPCentralRole_TerminateLink(connHandle);
      
      Display_print0(dispHandle, LCD_PAGE2, 0, "Disconnecting");
      Display_print0(dispHandle, LCD_PAGE4, 0, "");
    }

    return;
  }

//needed for all types of pairing besides OOB
#if !(((PAIRING == OOB_SC) || (PAIRING == OOB_LE)))
  if ((keys & KEY_DOWN) && (waiting_for_passcode))
  {
    // incrememnt passcode multiplier
    passcode_multiplier = passcode_multiplier / 10;
    if (passcode_multiplier == 0)
    {
      //send pascode response
      GAPBondMgr_PasscodeRsp(passcode_connHandle, SUCCESS, passcode);      
      //reset variables
      passcode_multiplier = 100000;
      passcode = 0;
      waiting_for_passcode = FALSE;
      passcode_connHandle = 0xFFFF;
    }
    return;
  }
#endif  
}

/*********************************************************************
 * @fn      security_examples_central_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void security_examples_central_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_print0(dispHandle, LCD_PAGE2, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, LCD_PAGE2, 0, "Pairing success");
    }
    else
    {
      Display_print1(dispHandle, LCD_PAGE2, 0, "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, LCD_PAGE2, 0, "Bonding success");
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, LCD_PAGE2, 0, "Bond save success");
    }
    else
    {
      Display_print1(dispHandle, LCD_PAGE2, 0, "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      security_examples_central_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void security_examples_central_processPasscode(uint16_t connectionHandle,
                                              gapPasskeyNeededEvent_t *pData)
{
//needed for all types of pairing besides OOB
#if !(((PAIRING == OOB_SC) || (PAIRING == OOB_LE)))
  if (pData->numComparison) //numeric comparison
  {
#if STATIC_PASSCODE
    // Send passcode response
    GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, TRUE);    
#else    
    judgeNumericComparison = TRUE;
#endif    
    //Display passcode
    Display_print1(dispHandle, LCD_PAGE4, 0, "Num Cmp: %d", pData->numComparison);
  }
  else //passkey entry
  {
    if (pData->uiInputs) // if we are to enter passkey
    {
#if STATIC_PASSCODE
      passcode = 111111;
      // Send passcode response
      GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
#else
      // user will enter passcode
      waiting_for_passcode = TRUE;
      passcode_connHandle = connectionHandle;
#endif      
      Display_print0(dispHandle, LCD_PAGE4, 0, "Enter Passcode:");
      Display_print1(dispHandle, LCD_PAGE5, 0, "%d", passcode);      
    }
    else if (pData->uiOutputs) // if we are to display passkey
    {
#if STATIC_PASSCODE
      passcode = 111111;
#else
      // Create random passcode
      passcode = Util_GetTRNG();
      passcode %= 1000000;
#endif      
      Display_print1(dispHandle, LCD_PAGE4, 0, "Passcode: %d", passcode);
      // Send passcode response
      GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);   
    }
    else
    {
      //shouldn't get here
    }
  }
#endif  
}

/**********************************************************************
 * @fn      security_examples_central_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void security_examples_central_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
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
 * @fn      security_examples_central_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t security_examples_central_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (security_examples_central_enqueueMsg(SEC_STATE_CHANGE_EVT, 
                                  SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }
  
  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      security_examples_central_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void security_examples_central_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  uint8_t *pData;
  
  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;  
  
    // Queue the event.
    security_examples_central_enqueueMsg(SEC_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      security_examples_central_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void security_examples_central_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison)
{
  gapPasskeyNeededEvent_t *pData;
  
  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPasskeyNeededEvent_t))))
  {
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->connectionHandle = connHandle;    
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;
    
    // Enqueue the event.
    security_examples_central_enqueueMsg(SEC_PASSCODE_NEEDED_EVT, 0, (uint8_t *) pData);
  }
}

/***********************************************************************
 * @fn      security_examples_central_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void security_examples_central_keyChangeHandler(uint8 keys)
{
  security_examples_central_enqueueMsg(SEC_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      security_examples_central_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t security_examples_central_enqueueMsg(uint8_t event, uint8_t state, 
                                           uint8_t *pData)
{
  secEvt_t *pMsg = ICall_malloc(sizeof(secEvt_t));
  
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

/*********************************************************************
*********************************************************************/
