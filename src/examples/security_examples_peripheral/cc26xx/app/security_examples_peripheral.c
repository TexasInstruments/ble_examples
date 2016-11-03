/**************************************************************************************************
  Filename:       just_works.c
  Revised:        $Date$
  Revision:       $Revision$

  Description:    This file contains the basic sample peripheral application to implement
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
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "simple_gatt_profile.h"
#include "devinfoservice.h"
#include "hci_tl.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include <ti/mw/display/Display.h>
#include "util.h"
#include "board_key.h"
#include "board.h"

#include "security_examples_peripheral.h"

#include <ti/mw/lcd/LCDDogm1286.h>

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

// Task configuration
#define SEP_TASK_PRIORITY                     1

#ifndef SEP_TASK_STACK_SIZE
#define SEP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define SEP_PAIRING_STATE_EVT                 0x0001
#define SEP_KEY_CHANGE_EVT                    0x0002
#define SEP_STATE_CHANGE_EVT                  0x0004
#define SEP_PASSCODE_NEEDED_EVT               0x0008

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
  uint8_t *pData;  // event data 
} sepEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * LOCAL VARIABLES
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
Task_Struct sepTask;
Char sepTaskStack[SEP_TASK_STACK_SIZE];

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x69,   // 'i'
  0x6d,   // 'm'
  0x70,   // 'p'
  0x6c,   // 'l'
  0x65,   // 'e'
  0x42,   // 'B'
  0x4c,   // 'L'
  0x45,   // 'E'
  0x50,   // 'P'
  0x65,   // 'e'
  0x72,   // 'r'
  0x69,   // 'i'
  0x70,   // 'p'
  0x68,   // 'h'
  0x65,   // 'e'
  0x72,   // 'r'
  0x61,   // 'a'
  0x6c,   // 'l'

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

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED
};

// Connection handle of current connection 
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Security Ex Periph";

//oob data needed for either type of OOB pairing
#if ((PAIRING == OOB_SC) || (PAIRING == OOB_LE))
//OOB data from remote
gapBondOobSC_t oobRemoteData =
{
  .addr = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA},
  .confirm = {0x38, 0xc0, 0x4d, 0x01, 0xe8, 0xb1, 0x7b, 0x90, 0x28, 0xad, 0x99, 
              0x48, 0xad, 0x89, 0x79, 0x4c },
  .oob = {0xA3, 0xDE, 0xBB, 0x31, 0xE6, 0x42, 0x4E, 0x2F, 0x39, 0x7F, 0xF2, 
          0xD2, 0xC4, 0x89, 0xC6, 0xA7}
};
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

static void security_examples_peripheral_init( void );
static void security_examples_peripheral_taskFxn(UArg a0, UArg a1);

static void security_examples_peripheral_handleKeys(uint8_t shift, uint8_t keys);
static void security_examples_peripheral_processAppMsg(sepEvt_t *pMsg);
static void security_examples_peripheral_processPasscode(uint16_t connectionHandle,
                                              gapPasskeyNeededEvent_t *pData);

static void security_examples_peripheral_processStateChangeEvt(gaprole_States_t newState);
static void security_examples_peripheral_stateChangeCB(gaprole_States_t newState);
static uint8_t security_examples_peripheral_enqueueMsg(uint8_t event, uint8_t status,
                                                     uint8_t *pData);

static void security_examples_peripheral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);
static void security_examples_peripheral_pairStateCB(uint16_t connHandle, uint8_t state, 
                                         uint8_t status);
void security_examples_peripheral_keyChangeHandler(uint8 keys);

static void security_examples_peripheral_processPairState(uint8_t state, uint8_t status);
static void security_examples_peripheral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);
static uint8_t security_example_peripheral_processStackMsg(ICall_Hdr *pMsg);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t security_examples_peripheral_gapRoleCBs =
{
  security_examples_peripheral_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t security_examples_peripheral_BondMgrCBs =
{
  (pfnPasscodeCB_t)security_examples_peripheral_passcodeCB, // Passcode callback
  security_examples_peripheral_pairStateCB  // Pairing / Bonding state Callback (not used by application)
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      security_examples_peripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void security_examples_peripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sepTaskStack;
  taskParams.stackSize = SEP_TASK_STACK_SIZE;
  taskParams.priority = SEP_TASK_PRIORITY;

  Task_construct(&sepTask, security_examples_peripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      security_examples_peripheral_init
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
static void security_examples_peripheral_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  uint8 bdAddr[] = {0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB};
  HCI_EXT_SetBDADDRCmd(bdAddr);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  Board_initKeys(security_examples_peripheral_keyChangeHandler);
  
  dispHandle = Display_open(Display_Type_LCD, NULL);
 
  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
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
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &scMode);
#if STATIC_KEYS
    GAPBondMgr_SetParameter(GAPBOND_ECC_KEYS, sizeof(gapBondEccKeys_t), &eccKeys);     
#endif    
    
    //initialization for legacy OOB pairing  
#elif (PAIRING == OOB_LE) 
    uint8_t scMode = GAPBOND_SECURE_CONNECTION_NONE;
    uint8_t oobEnabled = TRUE;
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &scMode);
    GAPBondMgr_SetParameter(GAPBOND_OOB_DATA, sizeof(uint8_t) * KEYLEN, oobRemoteData.oob);
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
  uint8_t ioCap = GAPBOND_IO_CAP_KEYBOARD_ONLY;
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

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  DevInfo_AddService();                        // Device Information Service  
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile    
  
  // Start the Device
  VOID GAPRole_StartDevice(&security_examples_peripheral_gapRoleCBs);
  
  // Start Bond Manager
  VOID GAPBondMgr_Register(&security_examples_peripheral_BondMgrCBs);
  
  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);
  
// pass OOB data to GAPBondMgr for OOB SC pairing
#if (PAIRING == OOB_SC)
#if (STATIC_KEYS)  // We already have the keys...get the confirm value
  SM_GetScConfirmOob(eccKeys.publicKeyX, oobRemoteData.oob, oobRemoteData.confirm);  

  uint8_t oobEnabled = TRUE;
  
  GAPBondMgr_SetParameter(GAPBOND_REMOTE_OOB_SC_ENABLED, sizeof(uint8_t), &oobEnabled );    
  GAPBondMgr_SetParameter(GAPBOND_REMOTE_OOB_SC_DATA, sizeof(gapBondOobSC_t), &oobRemoteData); 
#else //keys will be returned from the stack
  // Register to receive SM messages
  SM_RegisterTask(selfEntity); 
  
  // Get ECC Keys - response comes in through callback.
  SM_GetEccKeys();  
#endif //STATIC_KEYS
#endif //OOB_SC
  
  Display_print0(dispHandle, LCD_PAGE0, 0, "Security Ex Periph");
}

/*********************************************************************
 * @fn      security_examples_peripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void security_examples_peripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  security_examples_peripheral_init();

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
        uint8 safeToDealloc = TRUE;
        
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;
          
          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = security_example_peripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }
        
        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }          
      
      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sepEvt_t *pMsg = (sepEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          security_examples_peripheral_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    } 
  }
}

/*********************************************************************
 * @fn      security_examples_peripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void security_examples_peripheral_processAppMsg(sepEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SEP_STATE_CHANGE_EVT:
      security_examples_peripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case SEP_KEY_CHANGE_EVT:
      security_examples_peripheral_handleKeys(0, pMsg->hdr.state); 
      break;      
      
    // Pairing event  
    case SEP_PAIRING_STATE_EVT:
      {
        security_examples_peripheral_processPairState(pMsg->hdr.state, *pMsg->pData);
        
        ICall_free(pMsg->pData);
        break;
      }  
      
    // Passcode event    
    case SEP_PASSCODE_NEEDED_EVT:
      {     
        security_examples_peripheral_processPasscode(connHandle, (gapPasskeyNeededEvent_t *)pMsg->pData);
        
        ICall_free(pMsg->pData);
        break;
      }      
      
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      security_examples_peripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void security_examples_peripheral_stateChangeCB(gaprole_States_t newState)
{
  security_examples_peripheral_enqueueMsg(SEP_STATE_CHANGE_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      security_examples_peripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void security_examples_peripheral_processStateChangeEvt(gaprole_States_t newState)
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
        Display_print0(dispHandle, LCD_PAGE1, 0, Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, LCD_PAGE2, 0, "Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
      Display_print0(dispHandle, LCD_PAGE2, 0, "Advertising");
      break;

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);

        Display_print0(dispHandle, LCD_PAGE2, 0, "Connected");
        Display_print0(dispHandle, LCD_PAGE3, 0, Util_convertBdAddr2Str(peerAddress));
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Display_print0(dispHandle, LCD_PAGE2, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:
      Display_print0(dispHandle, LCD_PAGE2, 0, "Disconnected");

      // Clear remaining lines
      Display_print0(dispHandle, LCD_PAGE3, 0, "");
      Display_print0(dispHandle, LCD_PAGE4, 0, "");
      Display_print0(dispHandle, LCD_PAGE5, 0, "");
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      Display_print0(dispHandle, LCD_PAGE2, 0, "Timed Out");
      
      // Clear remaining lines
      Display_print0(dispHandle, LCD_PAGE3, 0, "");
      Display_print0(dispHandle, LCD_PAGE4, 0, "");
      Display_print0(dispHandle, LCD_PAGE5, 0, "");
      break;

    case GAPROLE_ERROR:
      Display_print0(dispHandle, LCD_PAGE2, 0, "Error");
      break;

    default:
      Display_print0(dispHandle, LCD_PAGE2, 0, "");
      break;
  }
}

/*********************************************************************
 * @fn      security_examples_peripheral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void security_examples_peripheral_processPairState(uint8_t state, uint8_t status)
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
 * @fn      security_examples_peripheral_handleKeys
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
static void security_examples_peripheral_handleKeys(uint8_t shift, uint8_t keys)
{
//needed for all types of pairing besides OOB
#if !(((PAIRING == OOB_SC) || (PAIRING == OOB_LE)))
  (void)shift;  // Intentionally unreferenced parameter

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
 * @fn      security_examples_peripheral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void security_examples_peripheral_processPasscode(uint16_t connectionHandle,
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

/*********************************************************************
 * @fn      security_examples_peripheral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void security_examples_peripheral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  uint8_t *pData;
  
  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;  
  
    // Queue the event.
    security_examples_peripheral_enqueueMsg(SEP_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      security_examples_peripheral_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void security_examples_peripheral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
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
    security_examples_peripheral_enqueueMsg(SEP_PASSCODE_NEEDED_EVT, 0, (uint8_t *) pData);
  }
}

/***********************************************************************
 * @fn      security_examples_peripheral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void security_examples_peripheral_keyChangeHandler(uint8 keys)
{
  security_examples_peripheral_enqueueMsg(SEP_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      security_examples_peripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static uint8_t security_examples_peripheral_enqueueMsg(uint8_t event, uint8_t state, 
                                           uint8_t *pData)
{
  sepEvt_t *pMsg = ICall_malloc(sizeof(sepEvt_t));

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


/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t security_example_peripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {    
#ifdef BROKEN
  case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;
#endif // Broken      

    //receive keys from stack for OOB SC pairing without static keys
#if ((PAIRING == OOB_SC) && !(STATIC_KEYS))
    case SM_MSG_EVENT:
      {
        //check for correct event
        if (pMsg->status == SM_ECC_KEYS_EVENT)
        {
          smEccKeysEvt_t* eccKeys =  (smEccKeysEvt_t*)pMsg;
          
          uint8_t oobEnabled = TRUE;
          
          // Get the confirm value
          SM_GetScConfirmOob(eccKeyspublicKeyX, oobRemoteData.oob, oobRemoteData.confirm);  
          
          //pass OOB data to GAPBondMgr
          GAPBondMgr_SetParameter(GAPBOND_REMOTE_OOB_SC_ENABLED, sizeof(uint8_t), &oobEnabled );    
          GAPBondMgr_SetParameter(GAPBOND_REMOTE_OOB_SC_DATA, sizeof(gapBondOobSC_t), &oobRemoteData);            
        }
      }
      break;
#endif

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}
