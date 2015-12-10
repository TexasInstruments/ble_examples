/**************************************************************************************************
  Filename:       just_works.c
  Revised:        $Date$
  Revision:       $Revision$

  Description:    This file contains the basic sample peripheral application to implement
                  legacy just works pairing for use with the CC2650 Bluetooth 
                  Low Energy Protocol Stack.

  Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
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
#include "devinfoservice.h"
#include "hci.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include "board_lcd.h"
#include "board_key.h"

#include "Board.h"

#include "security_examples_peripheral.h"

#include <ti/drivers/lcd/LCDDogm1286.h>

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
#define SEP_STATE_CHANGE_EVT                  0x0001
#define SEP_PAIRING_STATE_EVT                 0x0002

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
Task_Struct sbpTask;
Char sbpTaskStack[SEP_TASK_STACK_SIZE];

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

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Security Ex Periph";

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void security_examples_peripheral_init( void );
static void security_examples_peripheral_taskFxn(UArg a0, UArg a1);

static void security_examples_peripheral_processAppMsg(sepEvt_t *pMsg);

static void security_examples_peripheral_processStateChangeEvt(gaprole_States_t newState);
static void security_examples_peripheral_stateChangeCB(gaprole_States_t newState);
static void security_examples_peripheral_enqueueMsg(uint8_t event, uint8_t status,
                                                     uint8_t *pData);

static void security_examples_peripheral_pairStateCB(uint16_t connHandle, uint8_t state, 
                                         uint8_t status);
static void security_examples_peripheral_processPairState(uint8_t state, uint8_t status);

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
  NULL, // Passcode callback (not used by application)
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
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SEP_TASK_STACK_SIZE;
  taskParams.priority = SEP_TASK_PRIORITY;

  Task_construct(&sbpTask, security_examples_peripheral_taskFxn, &taskParams, NULL);
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

  Board_openLCD();
 
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

  // Setup the GAP Bond Manager
  {
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = FALSE;
    uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8_t bonding = FALSE;
    uint8_t scMode = GAPBOND_SECURE_CONNECTION_ONLY;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &scMode);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  DevInfo_AddService();                        // Device Information Service  
  
  // Start the Device
  VOID GAPRole_StartDevice(&security_examples_peripheral_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&security_examples_peripheral_BondMgrCBs);
   
  LCD_WRITE_STRING("Security Ex Periph", LCD_PAGE0);
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

    // Pairing event  
    case SEP_PAIRING_STATE_EVT:
      {
        security_examples_peripheral_processPairState(pMsg->hdr.state, *pMsg->pData);
        
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
        LCD_WRITE_STRING(Util_convertBdAddr2Str(ownAddress), LCD_PAGE1);
        LCD_WRITE_STRING("Initialized", LCD_PAGE2);
      }
      break;

    case GAPROLE_ADVERTISING:
      LCD_WRITE_STRING("Advertising", LCD_PAGE2);
      break;

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        LCD_WRITE_STRING("Connected", LCD_PAGE2);
        LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddress), LCD_PAGE3);
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      LCD_WRITE_STRING("Connected Advertising", LCD_PAGE2);
      break;

    case GAPROLE_WAITING:
      LCD_WRITE_STRING("Disconnected", LCD_PAGE2);

      // Clear remaining lines
      LCD_WRITE_STRING("", LCD_PAGE3);
      LCD_WRITE_STRING("", LCD_PAGE4);
      LCD_WRITE_STRING("", LCD_PAGE5);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      LCD_WRITE_STRING("Timed Out", LCD_PAGE2);
      
      // Clear remaining lines
      LCD_WRITE_STRING("", LCD_PAGE3);
      LCD_WRITE_STRING("", LCD_PAGE4);
      LCD_WRITE_STRING("", LCD_PAGE5);
      break;

    case GAPROLE_ERROR:
      LCD_WRITE_STRING("Error", LCD_PAGE2);
      break;

    default:
      LCD_WRITE_STRING("", LCD_PAGE2);
      break;
  }
}

/*********************************************************************
 * @fn      security_examples_central_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void security_examples_peripheral_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    LCD_WRITE_STRING("Pairing started", LCD_PAGE2);
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING("Pairing success", LCD_PAGE2);
    }
    else
    {
      LCD_WRITE_STRING_VALUE("Pairing fail:", status, 10, LCD_PAGE2);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING("Bonding success", LCD_PAGE2);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING("Bond save success", LCD_PAGE2);
    }
    else
    {
      LCD_WRITE_STRING_VALUE("Bond save failed:", status, 10, LCD_PAGE2);
    }
  }
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
 * @fn      security_examples_peripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void security_examples_peripheral_enqueueMsg(uint8_t event, uint8_t state, 
                                           uint8_t *pData)
{
  sepEvt_t *pMsg = ICall_malloc(sizeof(sepEvt_t));;

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
  }
}

/*********************************************************************
*********************************************************************/
