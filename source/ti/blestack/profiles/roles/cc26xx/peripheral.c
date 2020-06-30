/******************************************************************************

 @file  peripheral.c

 @brief GAP Peripheral Role for RTOS Applications

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2009-2020, Texas Instruments Incorporated
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
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <driverlib/ioc.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "peripheral.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Profile Events
#define GAPROLE_ICALL_EVT             ICALL_MSG_EVENT_ID //Event_Id_31
#define START_ADVERTISING_EVT         Event_Id_00
#define START_CONN_UPDATE_EVT         Event_Id_01
#define CONN_PARAM_TIMEOUT_EVT        Event_Id_02

#define GAPROLE_ALL_EVENTS            (GAPROLE_ICALL_EVT     | \
                                       START_ADVERTISING_EVT | \
                                       START_CONN_UPDATE_EVT | \
                                       CONN_PARAM_TIMEOUT_EVT)

#define DEFAULT_ADVERT_OFF_TIME       30000   // 30 seconds

#define DEFAULT_MIN_CONN_INTERVAL     0x0006  // 100 milliseconds
#define DEFAULT_MAX_CONN_INTERVAL     0x0C80  // 4 seconds

#define MIN_CONN_INTERVAL             0x0006
#define MAX_CONN_INTERVAL             0x0C80

#define DEFAULT_TIMEOUT_MULTIPLIER    1000

#define CONN_INTERVAL_MULTIPLIER      6

#define MIN_SLAVE_LATENCY             0
#define MAX_SLAVE_LATENCY             500

#define MIN_TIMEOUT_MULTIPLIER        0x000a
#define MAX_TIMEOUT_MULTIPLIER        0x0c80

#define MAX_TIMEOUT_VALUE             0xFFFF

// Task configuration
#define GAPROLE_TASK_PRIORITY         3

#ifndef GAPROLE_TASK_STACK_SIZE

#if defined __TI_COMPILER_VERSION__
#define GAPROLE_TASK_STACK_SIZE       480 // multiples of 8 only
#else  // IAR Compiler Used
#define GAPROLE_TASK_STACK_SIZE       440 // multiples of 8 only
#endif // defined __TI_COMPILER_VERSION__

#endif // GAPROLE_TASK_STACK_SIZE

/*********************************************************************
 * TYPEDEFS
 */

typedef struct
{
  uint8_t paramUpdateEnable;
  uint16_t minConnInterval;
  uint16_t maxConnInterval;
  uint16_t slaveLatency;
  uint16_t timeoutMultiplier;
} gapRole_updateConnParams_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Link DB maximum number of connections
#ifndef STACK_LIBRARY
uint8 linkDBNumConns;
#else /* ! STACK_LIBRARY */
extern uint8 linkDBNumConns;
#endif /* STACK_LIBRARY */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

static ICall_SyncHandle syncEvent;

// Clock object used to signal timeout
static Clock_Struct startAdvClock;
static Clock_Struct startUpdateClock;
static Clock_Struct updateTimeoutClock;

// Task setup
Task_Struct gapRoleTask;

#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(gapRoleTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t gapRoleTaskStack[GAPROLE_TASK_STACK_SIZE];

static gaprole_States_t gapRole_state;

/*********************************************************************
 * Profile Parameters - reference GAPROLE_PROFILE_PARAMETERS for
 * descriptions
 */

static uint8_t  gapRole_profileRole;
static uint8_t  gapRole_IRK[KEYLEN];
static uint8_t  gapRole_SRK[KEYLEN];
static uint32_t gapRole_signCounter;
static uint8_t  gapRole_bdAddr[B_ADDR_LEN];
static uint8_t  gapRole_AdvEnabled = FALSE;
static uint8_t  gapRole_AdvNonConnEnabled = FALSE;
static uint16_t gapRole_AdvertOffTime = DEFAULT_ADVERT_OFF_TIME;
static uint8_t  gapRole_AdvertDataLen = 3;

static uint8_t  gapRole_AdvertData[B_MAX_ADV_LEN] =
{
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,   // AD Type = Flags
  // Limited Discoverable & BR/EDR not supported
  (GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED),
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

static uint8_t  gapRole_ScanRspDataLen = 0;
static uint8_t  gapRole_ScanRspData[B_MAX_ADV_LEN] = {0};
static uint8_t  gapRole_AdvEventType;
static uint8_t  gapRole_AdvDirectType;
static uint8_t  gapRole_AdvDirectAddr[B_ADDR_LEN] = {0};
static uint8_t  gapRole_AdvChanMap;
static uint8_t  gapRole_AdvFilterPolicy;

static uint16_t gapRole_ConnectionHandle = INVALID_CONNHANDLE;

static uint8_t  gapRole_ConnectedDevAddr[B_ADDR_LEN] = {0};

// Connection parameter update parameters.
static gapRole_updateConnParams_t gapRole_updateConnParams =
{
  // Default behavior is to accept the remote device's request until application
  // changes local parameters.
  .paramUpdateEnable = GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS,
  .minConnInterval = DEFAULT_MIN_CONN_INTERVAL,
  .maxConnInterval = DEFAULT_MAX_CONN_INTERVAL,
  .slaveLatency = MIN_SLAVE_LATENCY,
  .timeoutMultiplier = DEFAULT_TIMEOUT_MULTIPLIER
};

static uint16_t gapRole_ConnInterval = 0;
static uint16_t gapRole_ConnSlaveLatency = 0;
static uint16_t gapRole_ConnTimeout = 0;
static uint8_t  gapRole_ConnectedDevAddrType = 0;
static uint8_t  gapRole_ConnTermReason = 0;

static uint8_t paramUpdateNoSuccessOption = GAPROLE_NO_ACTION;

// Application callbacks
static gapRolesCBs_t *pGapRoles_AppCGs = NULL;
static gapRolesParamUpdateCB_t *pGapRoles_ParamUpdateCB = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

/*********************************************************************
 * Profile Attributes - Table
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void gapRole_init(void);
static void gapRole_taskFxn(UArg a0, UArg a1);

static void      gapRole_processStackMsg(ICall_Hdr *pMsg);
static void      gapRole_processGAPMsg(gapEventHdr_t *pMsg);
static void      gapRole_SetupGAP(void);
static void      gapRole_HandleParamUpdateNoSuccess(void);
static bStatus_t gapRole_startConnUpdate(uint8_t handleFailure,
                                       gapRole_updateConnParams_t *pConnParams);

static void gapRole_setEvent(uint32_t event);

/*********************************************************************
 * CALLBACKS
 */
void gapRole_clockHandler(UArg a0);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @brief   Set a GAP Role parameter.
 *
 * Public function defined in peripheral.h.
 */
bStatus_t GAPRole_SetParameter(uint16_t param, uint8_t len, void *pValue)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
    case GAPROLE_IRK:
      if (len == KEYLEN)
      {
        VOID memcpy(gapRole_IRK, pValue, KEYLEN) ;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_SRK:
      if (len == KEYLEN)
      {
        VOID memcpy(gapRole_SRK, pValue, KEYLEN) ;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_SIGNCOUNTER:
      if (len == sizeof (uint32_t))
      {
        gapRole_signCounter = *((uint32_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADVERT_ENABLED:
      if (len == sizeof(uint8_t))
      {
        // Non-connectable advertising must be disabled.
        if (gapRole_AdvNonConnEnabled != TRUE)
        {
          uint8_t oldAdvEnabled = gapRole_AdvEnabled;
          gapRole_AdvEnabled = *((uint8_t*)pValue);

          if ((oldAdvEnabled) && (gapRole_AdvEnabled == FALSE))
          {
            // Turn off advertising.
            if ((gapRole_state == GAPROLE_ADVERTISING)
                || (gapRole_state == GAPROLE_WAITING_AFTER_TIMEOUT))
            {
              VOID GAP_EndDiscoverable(selfEntity);
            }
          }
          else if ((oldAdvEnabled == FALSE) && (gapRole_AdvEnabled))
          {
            // Turn on advertising.
            if ((gapRole_state == GAPROLE_STARTED)
                || (gapRole_state == GAPROLE_WAITING)
                || (gapRole_state == GAPROLE_WAITING_AFTER_TIMEOUT))
            {
              gapRole_setEvent(START_ADVERTISING_EVT);
            }
          }
        }
        else
        {
          ret = bleIncorrectMode;
        }
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADV_NONCONN_ENABLED:
      if (len == sizeof(uint8_t))
      {
        // Connectable advertising must be disabled.
        if (gapRole_AdvEnabled != TRUE)
        {
          uint8_t oldAdvEnabled = gapRole_AdvNonConnEnabled;
          gapRole_AdvNonConnEnabled = *((uint8_t*)pValue);

          if ((oldAdvEnabled) && (gapRole_AdvNonConnEnabled == FALSE))
          {
            if ((gapRole_state == GAPROLE_ADVERTISING_NONCONN)
                || (gapRole_state == GAPROLE_CONNECTED_ADV)
                || (gapRole_state == GAPROLE_WAITING_AFTER_TIMEOUT))
            {
              VOID GAP_EndDiscoverable(selfEntity);
            }
          }
          else if ((oldAdvEnabled == FALSE) && (gapRole_AdvNonConnEnabled))
          {
            // Turn on advertising.
            if ((gapRole_state == GAPROLE_STARTED)
                || (gapRole_state == GAPROLE_WAITING)
                || (gapRole_state == GAPROLE_CONNECTED)
                || (gapRole_state == GAPROLE_WAITING_AFTER_TIMEOUT))
            {
              gapRole_setEvent(START_ADVERTISING_EVT);
            }
          }
        }
        else
        {
          ret = bleIncorrectMode;
        }
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADVERT_OFF_TIME:
      if (len == sizeof (uint16_t))
      {
        gapRole_AdvertOffTime = *((uint16_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADVERT_DATA:
      if (len <= B_MAX_ADV_LEN)
      {
        VOID memset(gapRole_AdvertData, 0, B_MAX_ADV_LEN);
        VOID memcpy(gapRole_AdvertData, pValue, len);
        gapRole_AdvertDataLen = len;

        // Update the advertising data
        ret = GAP_UpdateAdvertisingData(selfEntity,
                              TRUE, gapRole_AdvertDataLen, gapRole_AdvertData);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_SCAN_RSP_DATA:
      if (len <= B_MAX_ADV_LEN)
      {
        VOID memset(gapRole_ScanRspData, 0, B_MAX_ADV_LEN);
        VOID memcpy(gapRole_ScanRspData, pValue, len);
        gapRole_ScanRspDataLen = len;

        // Update the Response Data
        ret = GAP_UpdateAdvertisingData(selfEntity,
                              FALSE, gapRole_ScanRspDataLen, gapRole_ScanRspData);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADV_EVENT_TYPE:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= GAP_ADTYPE_ADV_LDC_DIRECT_IND))
      {
        gapRole_AdvEventType = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADV_DIRECT_TYPE:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= ADDRMODE_PRIVATE_RESOLVE))
      {
        gapRole_AdvDirectType = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADV_DIRECT_ADDR:
      if (len == B_ADDR_LEN)
      {
        VOID memcpy(gapRole_AdvDirectAddr, pValue, B_ADDR_LEN) ;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADV_CHANNEL_MAP:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= 0x07))
      {
        gapRole_AdvChanMap = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADV_FILTER_POLICY:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= GAP_FILTER_POLICY_WHITE))
      {
        gapRole_AdvFilterPolicy = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_PARAM_UPDATE_ENABLE:
      if ((len == sizeof (uint8_t)) &&
          (*((uint8_t*)pValue) < GAPROLE_LINK_PARAM_UPDATE_NUM_OPTIONS))
      {
        gapRole_updateConnParams.paramUpdateEnable = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_MIN_CONN_INTERVAL:
      {
        uint16_t newInterval = *((uint16_t*)pValue);
        if (  len == sizeof (uint16_t)           &&
             (newInterval >= MIN_CONN_INTERVAL) &&
             (newInterval <= MAX_CONN_INTERVAL))
        {
          gapRole_updateConnParams.minConnInterval = newInterval;
        }
        else
        {
          ret = bleInvalidRange;
        }
      }
      break;

    case GAPROLE_MAX_CONN_INTERVAL:
      {
        uint16_t newInterval = *((uint16_t*)pValue);
        if (  len == sizeof (uint16_t)          &&
             (newInterval >= MIN_CONN_INTERVAL) &&
             (newInterval <= MAX_CONN_INTERVAL))
        {
          gapRole_updateConnParams.maxConnInterval = newInterval;
        }
        else
        {
          ret = bleInvalidRange;
        }
      }
      break;

    case GAPROLE_SLAVE_LATENCY:
      {
        uint16_t latency = *((uint16_t*)pValue);
        if (len == sizeof (uint16_t) && (latency < MAX_SLAVE_LATENCY))
        {
          gapRole_updateConnParams.slaveLatency = latency;
        }
        else
        {
          ret = bleInvalidRange;
        }
      }
      break;

    case GAPROLE_TIMEOUT_MULTIPLIER:
      {
        uint16_t newTimeout = *((uint16_t*)pValue);
        if (len == sizeof (uint16_t)
            && (newTimeout >= MIN_TIMEOUT_MULTIPLIER) && (newTimeout <= MAX_TIMEOUT_MULTIPLIER))
        {
          gapRole_updateConnParams.timeoutMultiplier = newTimeout;
        }
        else
        {
          ret = bleInvalidRange;
        }
      }
      break;

      case GAPROLE_PARAM_UPDATE_REQ:
        {
          uint8_t req = *((uint8_t*)pValue);
          if (len == sizeof (uint8_t) && (req == TRUE))
          {
            // Make sure we don't send an L2CAP Connection Parameter Update Request
            // command within TGAP(conn_param_timeout) of an L2CAP Connection Parameter
            // Update Response being received.
            if (Util_isActive(&updateTimeoutClock) == FALSE)
            {
              // Start connection update procedure
              ret = gapRole_startConnUpdate(GAPROLE_NO_ACTION, &gapRole_updateConnParams);
              if (ret == SUCCESS)
              {
                // Connection update requested by app, cancel such pending procedure (if active)
                Util_stopClock(&startUpdateClock);
              }
            }
            else
            {
              ret = blePending;
            }
          }
          else
          {
            ret = bleInvalidRange;
          }
        }
        break;

    default:
      // The param value isn't part of this profile, try the GAP.
      if ((param < TGAP_PARAMID_MAX) && (len == sizeof (uint16_t)))
      {
        ret = GAP_SetParamValue(param, *((uint16_t*)pValue));
      }
      else
      {
        ret = INVALIDPARAMETER;
      }
      break;
  }

  return (ret);
}

/*********************************************************************
 * @brief   Get a GAP Role parameter.
 *
 * Public function defined in peripheral.h.
 */
bStatus_t GAPRole_GetParameter(uint16_t param, void *pValue)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
    case GAPROLE_PROFILEROLE:
      *((uint8_t*)pValue) = gapRole_profileRole;
      break;

    case GAPROLE_IRK:
      VOID memcpy(pValue, gapRole_IRK, KEYLEN) ;
      break;

    case GAPROLE_SRK:
      VOID memcpy(pValue, gapRole_SRK, KEYLEN) ;
      break;

    case GAPROLE_SIGNCOUNTER:
      *((uint32_t*)pValue) = gapRole_signCounter;
      break;

    case GAPROLE_BD_ADDR:
      VOID memcpy(pValue, gapRole_bdAddr, B_ADDR_LEN) ;
      break;

    case GAPROLE_ADVERT_ENABLED:
      *((uint8_t*)pValue) = gapRole_AdvEnabled;
      break;

    case GAPROLE_ADV_NONCONN_ENABLED:
      *((uint8_t*)pValue) = gapRole_AdvNonConnEnabled;
      break;

    case GAPROLE_ADVERT_OFF_TIME:
      *((uint16_t*)pValue) = gapRole_AdvertOffTime;
      break;

    case GAPROLE_ADVERT_DATA:
      VOID memcpy(pValue , gapRole_AdvertData, gapRole_AdvertDataLen);
      break;

    case GAPROLE_SCAN_RSP_DATA:
      VOID memcpy(pValue, gapRole_ScanRspData, gapRole_ScanRspDataLen) ;
      break;

    case GAPROLE_ADV_EVENT_TYPE:
      *((uint8_t*)pValue) = gapRole_AdvEventType;
      break;

    case GAPROLE_ADV_DIRECT_TYPE:
      *((uint8_t*)pValue) = gapRole_AdvDirectType;
      break;

    case GAPROLE_ADV_DIRECT_ADDR:
      VOID memcpy(pValue, gapRole_AdvDirectAddr, B_ADDR_LEN) ;
      break;

    case GAPROLE_ADV_CHANNEL_MAP:
      *((uint8_t*)pValue) = gapRole_AdvChanMap;
      break;

    case GAPROLE_ADV_FILTER_POLICY:
      *((uint8_t*)pValue) = gapRole_AdvFilterPolicy;
      break;

    case GAPROLE_CONNHANDLE:
      *((uint16_t*)pValue) = gapRole_ConnectionHandle;
      break;

    case GAPROLE_PARAM_UPDATE_ENABLE:
      *((uint16_t*)pValue) = gapRole_updateConnParams.paramUpdateEnable;
      break;

    case GAPROLE_MIN_CONN_INTERVAL:
      *((uint16_t*)pValue) = gapRole_updateConnParams.minConnInterval;
      break;

    case GAPROLE_MAX_CONN_INTERVAL:
      *((uint16_t*)pValue) = gapRole_updateConnParams.maxConnInterval;
      break;

    case GAPROLE_SLAVE_LATENCY:
      *((uint16_t*)pValue) = gapRole_updateConnParams.slaveLatency;
      break;

    case GAPROLE_TIMEOUT_MULTIPLIER:
      *((uint16_t*)pValue) = gapRole_updateConnParams.timeoutMultiplier;
      break;

    case GAPROLE_CONN_BD_ADDR:
      VOID memcpy(pValue, gapRole_ConnectedDevAddr, B_ADDR_LEN) ;
      break;

    case GAPROLE_CONN_INTERVAL:
      *((uint16_t*)pValue) = gapRole_ConnInterval;
      break;

    case GAPROLE_CONN_LATENCY:
      *((uint16_t*)pValue) = gapRole_ConnSlaveLatency;
      break;

    case GAPROLE_CONN_TIMEOUT:
      *((uint16_t*)pValue) = gapRole_ConnTimeout;
      break;

     case GAPROLE_BD_ADDR_TYPE:
      *((uint8_t*)pValue) = gapRole_ConnectedDevAddrType;
      break;

    case GAPROLE_STATE:
      *((uint8_t*)pValue) = gapRole_state;
      break;

    case GAPROLE_CONN_TERM_REASON:
      *((uint8_t*)pValue) = gapRole_ConnTermReason;
      break;

    default:
      // The param value isn't part of this profile, try the GAP.
      if (param < TGAP_PARAMID_MAX)
      {
        *((uint16_t*)pValue) = GAP_GetParamValue(param);
      }
      else
      {
        ret = INVALIDPARAMETER;
      }
      break;
  }

  return (ret);
}

/*********************************************************************
 * @brief   Does the device initialization.
 *
 * Public function defined in peripheral.h.
 */
bStatus_t GAPRole_StartDevice(gapRolesCBs_t *pAppCallbacks)
{
  if (gapRole_state == GAPROLE_INIT)
  {
    // Clear all of the Application callbacks
    if (pAppCallbacks)
    {
      pGapRoles_AppCGs = pAppCallbacks;
    }

    // Start the GAP
    gapRole_SetupGAP();

    return (SUCCESS);
  }
  else
  {
    return (bleAlreadyInRequestedMode);
  }
}

/*********************************************************************
 * @brief   Register application's callbacks.
 *
 * Public function defined in peripheral.h.
 */
void GAPRole_RegisterAppCBs(gapRolesParamUpdateCB_t *pParamUpdateCB)
{
  if (pParamUpdateCB != NULL)
  {
    pGapRoles_ParamUpdateCB = pParamUpdateCB;
  }
}

/*********************************************************************
 * @brief   Terminates the existing connection.
 *
 * Public function defined in peripheral.h.
 */
bStatus_t GAPRole_TerminateConnection(void)
{
  if ((gapRole_state == GAPROLE_CONNECTED) ||
      (gapRole_state == GAPROLE_CONNECTED_ADV))
  {
    return (GAP_TerminateLinkReq(selfEntity, gapRole_ConnectionHandle,
                                 HCI_DISCONNECT_REMOTE_USER_TERM));
  }
  else
  {
    return (bleIncorrectMode);
  }
}

/*********************************************************************
 * @fn      GAPRole_createTask
 *
 * @brief   Task creation function for the GAP Peripheral Role.
 *
 * @param   none
 *
 * @return  none
 */
void GAPRole_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = gapRoleTaskStack;
  taskParams.stackSize = GAPROLE_TASK_STACK_SIZE;
  taskParams.priority = GAPROLE_TASK_PRIORITY;

  Task_construct(&gapRoleTask, gapRole_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

/*********************************************************************
 * @fn      gapRole_init
 *
 * @brief   Initialization function for the GAP Role Task.
 *
 * @param   none
 *
 * @return  none
 */
static void gapRole_init(void)
{
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  gapRole_state = GAPROLE_INIT;
  gapRole_ConnectionHandle = INVALID_CONNHANDLE;

  // Get link DB maximum number of connections
#ifndef STACK_LIBRARY
  linkDBNumConns = linkDB_NumConns();
#endif /* STACK_LIBRARY */

  // Setup timers as one-shot timers
  Util_constructClock(&startAdvClock, gapRole_clockHandler,
                      0, 0, false, START_ADVERTISING_EVT);
  Util_constructClock(&startUpdateClock, gapRole_clockHandler,
                      0, 0, false, START_CONN_UPDATE_EVT);
  Util_constructClock(&updateTimeoutClock, gapRole_clockHandler,
                      0, 0, false, CONN_PARAM_TIMEOUT_EVT);

  // Initialize the Profile Advertising and Connection Parameters
  gapRole_profileRole = GAP_PROFILE_PERIPHERAL;
  VOID memset(gapRole_IRK, 0, KEYLEN);
  VOID memset(gapRole_SRK, 0, KEYLEN);
  gapRole_signCounter = 0;
  gapRole_AdvEventType = GAP_ADTYPE_ADV_IND;
  gapRole_AdvDirectType = ADDRMODE_PUBLIC;
  gapRole_AdvChanMap = GAP_ADVCHAN_ALL;
  gapRole_AdvFilterPolicy = GAP_FILTER_POLICY_ALL;

  // Restore Items from NV
  VOID osal_snv_read(BLE_NVID_IRK, KEYLEN, gapRole_IRK);
  VOID osal_snv_read(BLE_NVID_CSRK, KEYLEN, gapRole_SRK);
  VOID osal_snv_read(BLE_NVID_SIGNCOUNTER, sizeof(uint32_t),
                     &gapRole_signCounter);
}

/*********************************************************************
 * @fn      gapRole_taskFxn
 *
 * @brief   Task entry point for the GAP Peripheral Role.
 *
 * @param   a0 - first argument
 * @param   a1 - second argument
 *
 * @return  none
 */
static void gapRole_taskFxn(UArg a0, UArg a1)
{
  // Initialize profile
  gapRole_init();

  // Profile main loop
  for (;;)
  {
    uint32_t events;

    // Wait for an event to be posted
    events = Event_pend(syncEvent, Event_Id_NONE, GAPROLE_ALL_EVENTS,
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
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & GAP_EVENT_SIGN_COUNTER_CHANGED)
            {
              // Sign counter changed, save it to NV
              VOID osal_snv_write(BLE_NVID_SIGNCOUNTER, sizeof(uint32_t),
                                  &gapRole_signCounter);
            }
          }
          else
          {
            // Process inter-task message
            gapRole_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      if (events & START_ADVERTISING_EVT)
      {
        if (gapRole_AdvEnabled || gapRole_AdvNonConnEnabled)
        {
          gapAdvertisingParams_t params;

          // Setup advertisement parameters
          if (gapRole_AdvNonConnEnabled)
          {
            // Only advertise non-connectable undirected.
            params.eventType = GAP_ADTYPE_ADV_NONCONN_IND;
          }
          else
          {
            params.eventType = gapRole_AdvEventType;
            params.initiatorAddrType = gapRole_AdvDirectType;
            VOID memcpy(params.initiatorAddr, gapRole_AdvDirectAddr, B_ADDR_LEN);
          }

          params.channelMap = gapRole_AdvChanMap;
          params.filterPolicy = gapRole_AdvFilterPolicy;

          if (GAP_MakeDiscoverable(selfEntity, &params) != SUCCESS)
          {
            gapRole_state = GAPROLE_ERROR;

            // Notify the application with the new state change
            if (pGapRoles_AppCGs && pGapRoles_AppCGs->pfnStateChange)
            {
              pGapRoles_AppCGs->pfnStateChange(gapRole_state);
            }
          }
        }
      }

      if (events & START_CONN_UPDATE_EVT)
      {
        // Start connection update procedure
        gapRole_startConnUpdate(GAPROLE_NO_ACTION, &gapRole_updateConnParams);
      }

      if (events & CONN_PARAM_TIMEOUT_EVT)
      {
        // Unsuccessful in updating connection parameters
        gapRole_HandleParamUpdateNoSuccess();
      }
    }
  } // for
}

/*********************************************************************
 * @fn      gapRole_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void gapRole_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      gapRole_processGAPMsg((gapEventHdr_t *)pMsg);
      break;

    case L2CAP_SIGNAL_EVENT:
      {
        l2capSignalEvent_t *pPkt = (l2capSignalEvent_t *)pMsg;

        // Process the Parameter Update Response
        if (pPkt->opcode == L2CAP_PARAM_UPDATE_RSP)
        {
          l2capParamUpdateRsp_t *pRsp = (l2capParamUpdateRsp_t *)&(pPkt->cmd.updateRsp);

          if ((pRsp->result == L2CAP_CONN_PARAMS_REJECTED) &&
               (paramUpdateNoSuccessOption == GAPROLE_TERMINATE_LINK))
          {
            // Cancel connection param update timeout timer
            Util_stopClock(&updateTimeoutClock);

            // Terminate connection immediately
            GAPRole_TerminateConnection();
          }
          else
          {
            uint16_t timeout = GAP_GetParamValue(TGAP_CONN_PARAM_TIMEOUT);

            // Let's wait for Controller to update connection parameters if they're
            // accepted. Otherwise, decide what to do based on no success option.
            Util_restartClock(&updateTimeoutClock, timeout);
          }
        }
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      gapRole_processGAPMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void gapRole_processGAPMsg(gapEventHdr_t *pMsg)
{
  uint8_t notify = FALSE;   // State changed notify the app? (default no)

  switch (pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;
        bStatus_t stat = pPkt->hdr.status;

        if (stat == SUCCESS)
        {
          // Save off the generated keys
          VOID osal_snv_write(BLE_NVID_IRK, KEYLEN, gapRole_IRK);
          VOID osal_snv_write(BLE_NVID_CSRK, KEYLEN, gapRole_SRK);

          // Save off the information
          VOID memcpy(gapRole_bdAddr, pPkt->devAddr, B_ADDR_LEN);

          gapRole_state = GAPROLE_STARTED;

          // Update the advertising data
          stat = GAP_UpdateAdvertisingData(selfEntity,
                              TRUE, gapRole_AdvertDataLen, gapRole_AdvertData);
        }

        if (stat != SUCCESS)
        {
          gapRole_state = GAPROLE_ERROR;
        }

        notify = TRUE;
      }
      break;

    case GAP_ADV_DATA_UPDATE_DONE_EVENT:
      {
        gapAdvDataUpdateEvent_t *pPkt = (gapAdvDataUpdateEvent_t *)pMsg;

        if (pPkt->hdr.status == SUCCESS)
        {
          if (pPkt->adType)
          {
            // Setup the Response Data
            pPkt->hdr.status = GAP_UpdateAdvertisingData(selfEntity, FALSE,
                                                         gapRole_ScanRspDataLen,
                                                         gapRole_ScanRspData);
          }
          else if ((gapRole_state != GAPROLE_ADVERTISING)         &&
                   (gapRole_state != GAPROLE_CONNECTED_ADV)       &&
                   (gapRole_state != GAPROLE_CONNECTED ||
                    gapRole_AdvNonConnEnabled == TRUE)            &&
                   (gapRole_state != GAPROLE_ADVERTISING_NONCONN) &&
                   (Util_isActive(&startAdvClock) == FALSE))
          {
            // Start advertising
            gapRole_setEvent(START_ADVERTISING_EVT);
          }
        }

        if (pPkt->hdr.status != SUCCESS)
        {
          // Set into Error state
          gapRole_state = GAPROLE_ERROR;
          notify = TRUE;
        }
      }
      break;

    case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
    case GAP_END_DISCOVERABLE_DONE_EVENT:
      {
        gapMakeDiscoverableRspEvent_t *pPkt = (gapMakeDiscoverableRspEvent_t *)pMsg;

        if (pPkt->hdr.status == SUCCESS)
        {
          if (pMsg->opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
          {
            if (gapRole_state == GAPROLE_CONNECTED)
            {
              gapRole_state = GAPROLE_CONNECTED_ADV;
            }
            else if (gapRole_AdvEnabled)
            {
              gapRole_state = GAPROLE_ADVERTISING;
            }
            else
            {
              gapRole_state = GAPROLE_ADVERTISING_NONCONN;
            }
          }
          else // GAP_END_DISCOVERABLE_DONE_EVENT
          {
            if (gapRole_AdvertOffTime != 0)
            {
              if ((gapRole_AdvEnabled) || (gapRole_AdvNonConnEnabled))
              {
                Util_restartClock(&startAdvClock, gapRole_AdvertOffTime);
              }
            }
            else
            {
              // Since gapRole_AdvertOffTime is set to 0, the device should not
              // automatically become discoverable again after a period of time.
              // Set enabler to FALSE; device will become discoverable again when
              // this value gets set to TRUE
              if (gapRole_AdvEnabled == TRUE)
              {
                gapRole_AdvEnabled = FALSE;
              }
              else
              {
                gapRole_AdvNonConnEnabled = FALSE;
              }
            }

            // Update state.
            if (gapRole_state == GAPROLE_CONNECTED_ADV)
            {
              // In the Advertising Off period
              gapRole_state = GAPROLE_CONNECTED;
            }
            else
            {
              // In the Advertising Off period
              gapRole_state = GAPROLE_WAITING;
            }
          }
        }
        else
        {
          gapRole_state = GAPROLE_ERROR;
        }

        notify = TRUE;
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

        if (pPkt->hdr.status == SUCCESS)
        {
          VOID memcpy(gapRole_ConnectedDevAddr, pPkt->devAddr, B_ADDR_LEN);
          gapRole_ConnectionHandle = pPkt->connectionHandle;
          gapRole_state = GAPROLE_CONNECTED;

          // Store connection information
          gapRole_ConnInterval = pPkt->connInterval;
          gapRole_ConnSlaveLatency = pPkt->connLatency;
          gapRole_ConnTimeout = pPkt->connTimeout;
          gapRole_ConnectedDevAddrType = pPkt->devAddrType;

          // Check whether update parameter request is enabled
          if ((gapRole_updateConnParams.paramUpdateEnable ==
               GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS) ||
              (gapRole_updateConnParams.paramUpdateEnable ==
               GAPROLE_LINK_PARAM_UPDATE_INITIATE_APP_PARAMS))
          {
            // Get the minimum time upon connection establishment before the
            // peripheral can start a connection update procedure.
            uint16_t timeout = GAP_GetParamValue(TGAP_CONN_PAUSE_PERIPHERAL);

            Util_restartClock(&startUpdateClock, timeout*1000);
          }

          // Notify the Bond Manager to the connection
          VOID GAPBondMgr_LinkEst(pPkt->devAddrType, pPkt->devAddr,
                                  pPkt->connectionHandle, GAP_PROFILE_PERIPHERAL);
        }
        else if (pPkt->hdr.status == bleGAPConnNotAcceptable)
        {
          // Set enabler to FALSE; device will become discoverable again when
          // this value gets set to TRUE
          gapRole_AdvEnabled = FALSE;

          // Go to WAITING state, and then start advertising
          gapRole_state = GAPROLE_WAITING;
        }
        else
        {
          gapRole_state = GAPROLE_ERROR;
        }
        notify = TRUE;
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

        GAPBondMgr_LinkTerm(pPkt->connectionHandle);

        memset(gapRole_ConnectedDevAddr, 0, B_ADDR_LEN);

        // Erase connection information
        gapRole_ConnInterval = 0;
        gapRole_ConnSlaveLatency = 0;
        gapRole_ConnTimeout = 0;
        gapRole_ConnTermReason = pPkt->reason;

        // Cancel all connection parameter update timers (if any active)
        Util_stopClock(&startUpdateClock);
        Util_stopClock(&updateTimeoutClock);

        notify = TRUE;

        gapRole_ConnectionHandle = INVALID_CONNHANDLE;

        // If device was advertising when connection dropped
        if (gapRole_AdvNonConnEnabled)
        {
          // Continue advertising.
          gapRole_state = GAPROLE_ADVERTISING_NONCONN;
        }
        // Else go to WAITING state.
        else
        {
          if(pPkt->reason == LL_SUPERVISION_TIMEOUT_TERM)
          {
            gapRole_state = GAPROLE_WAITING_AFTER_TIMEOUT;
          }
          else
          {
            gapRole_state = GAPROLE_WAITING;
          }

          // Start advertising, if enabled.
          gapRole_setEvent(START_ADVERTISING_EVT);
        }
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

        // Cancel connection param update timeout timer (if active)
        Util_stopClock(&updateTimeoutClock);

        if (pPkt->hdr.status == SUCCESS)
        {
          // Store new connection parameters
          gapRole_ConnInterval = pPkt->connInterval;
          gapRole_ConnSlaveLatency = pPkt->connLatency;
          gapRole_ConnTimeout = pPkt->connTimeout;

          // Make sure there's no pending connection update procedure
          if(Util_isActive(&startUpdateClock) == FALSE)
          {
            // Notify the application with the new connection parameters
            if (pGapRoles_ParamUpdateCB != NULL)
            {
              (*pGapRoles_ParamUpdateCB)(gapRole_ConnInterval,
                                         gapRole_ConnSlaveLatency,
                                         gapRole_ConnTimeout);
            }
          }
        }
      }
      break;

    case GAP_PAIRING_REQ_EVENT:
      {
        gapPairingReqEvent_t *pPkt = (gapPairingReqEvent_t *)pMsg;

        // Send Pairing Failed Response
        VOID GAP_TerminateAuth(pPkt->connectionHandle,
                               SMP_PAIRING_FAILED_NOT_SUPPORTED);
      }
      break;

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
      {
        gapUpdateLinkParamReqEvent_t *pReq;
        gapUpdateLinkParamReqReply_t rsp;

        pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;

        // Add in connection handle.
        rsp.connectionHandle = pReq->req.connectionHandle;

        // If application provided desired parameters, check against them.
        // Otherwise, use what the remote has requested.
        if (gapRole_updateConnParams.paramUpdateEnable ==
            GAPROLE_LINK_PARAM_UPDATE_REJECT_REQUEST)
        {
          rsp.accepted = FALSE;
        }
        else
        {
          // Link Parameter update is accepted.  set parameter values with the
          // the strategy requested by the application.
          rsp.accepted = TRUE;

          // If an update was scheduled, cancel it.
          Util_stopClock(&startUpdateClock);

          if ((gapRole_updateConnParams.paramUpdateEnable ==
                 GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS) ||
                 (gapRole_updateConnParams.paramUpdateEnable ==
                 GAPROLE_LINK_PARAM_UPDATE_WAIT_BOTH_PARAMS))
          {
            // Volume 3, Part C, Section 9.3.9.2, "Devices should be tolerant of
            // connection parameters given to them by the remote device." The
            // interpretation here is to check if the connection parameters
            // given by the remote device are compatible with the desired values
            // of the local device.  Where a mismatch occurs, the local device
            // attempts to conservatively fix them.  This plays out as described
            // below.

            // Minimum Connection Interval.  the longer is used.
            rsp.intervalMin =
            (pReq->req.intervalMin < gapRole_updateConnParams.minConnInterval) ?
              gapRole_updateConnParams.minConnInterval :
              pReq->req.intervalMin;

            // Maximum Connection Interval.  The longer is used.
            rsp.intervalMax =
            (pReq->req.intervalMax < gapRole_updateConnParams.maxConnInterval) ?
              gapRole_updateConnParams.maxConnInterval :
              pReq->req.intervalMax;

            // Slave Latency.  The longer slave latency is used.
            rsp.connLatency =
               (pReq->req.connLatency < gapRole_updateConnParams.slaveLatency) ?
                 gapRole_updateConnParams.slaveLatency :
                 pReq->req.connLatency;

            // Connection Timeout.  The longer timeout is used.
            rsp.connTimeout =
          (pReq->req.connTimeout < gapRole_updateConnParams.timeoutMultiplier) ?
            gapRole_updateConnParams.timeoutMultiplier :
            pReq->req.connTimeout;
          }
          else if ((gapRole_updateConnParams.paramUpdateEnable ==
                    GAPROLE_LINK_PARAM_UPDATE_WAIT_APP_PARAMS) ||
                   (gapRole_updateConnParams.paramUpdateEnable ==
                    GAPROLE_LINK_PARAM_UPDATE_INITIATE_APP_PARAMS))
          {
            // Only use application requested values.
            rsp.intervalMin = gapRole_updateConnParams.minConnInterval;
            rsp.intervalMax = gapRole_updateConnParams.maxConnInterval;
            rsp.connLatency = gapRole_updateConnParams.slaveLatency;
            rsp.connTimeout = gapRole_updateConnParams.timeoutMultiplier;
          }
          else // GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS
          {
            // Only use remote requested values.
            rsp.intervalMin = pReq->req.intervalMin;
            rsp.intervalMax = pReq->req.intervalMax;
            rsp.connLatency = pReq->req.connLatency;
            rsp.connTimeout = pReq->req.connTimeout;
          }
        }

        // Send application's requested parameters back.
        VOID GAP_UpdateLinkParamReqReply(&rsp);
      }
      break;

    default:
      break;
  }

  if (notify == TRUE)
  {
    // Notify the application with the new state change
    if (pGapRoles_AppCGs && pGapRoles_AppCGs->pfnStateChange)
    {
      pGapRoles_AppCGs->pfnStateChange(gapRole_state);
    }
  }
}

/*********************************************************************
 * @fn      gapRole_SetupGAP
 *
 * @brief   Call the GAP Device Initialization function using the
 *          Profile Parameters.
 *
 * @param   none
 *
 * @return  none
 */
static void gapRole_SetupGAP(void)
{
  VOID GAP_DeviceInit(selfEntity, gapRole_profileRole, 0, gapRole_IRK,
                      gapRole_SRK, (uint32*)&gapRole_signCounter);
}

/*********************************************************************
 * @fn      gapRole_HandleParamUpdateNoSuccess
 *
 * @brief   Handle unsuccessful connection parameters update.
 *
 * @param   none
 *
 * @return  none
 */
static void gapRole_HandleParamUpdateNoSuccess(void)
{
  // See which option was chosen for unsuccessful updates
  switch (paramUpdateNoSuccessOption)
  {
    case GAPROLE_RESEND_PARAM_UPDATE:
      GAPRole_SendUpdateParam(gapRole_updateConnParams.minConnInterval,
                              gapRole_updateConnParams.maxConnInterval,
                              gapRole_updateConnParams.slaveLatency,
                              gapRole_updateConnParams.timeoutMultiplier,
                              GAPROLE_RESEND_PARAM_UPDATE);
      break;

    case GAPROLE_TERMINATE_LINK:
      GAPRole_TerminateConnection();
      break;

    case GAPROLE_NO_ACTION:
      // fall through
    default:
      //do nothing
      break;
  }
}

/********************************************************************
 * @fn          gapRole_startConnUpdate
 *
 * @brief       Start the connection update procedure
 *
 * @param       handleFailure - what to do if the update does not occur.
 *              Method may choose to terminate connection, try again,
 *              or take no action
 * @param       pConnParams   - connection parameters to use
 *
 * @return      SUCCESS: operation was successful.
 *              INVALIDPARAMETER: Data can not fit into one packet.
 *              MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *              bleInvalidRange:
 *              bleIncorrectMode: invalid profile role.
 *              bleAlreadyInRequestedMode: already updating link parameters.
 *              bleNotConnected: Connection is down
 *              bleMemAllocError: Memory allocation error occurred.
 *              bleNoResources: No available resource
 */
static bStatus_t gapRole_startConnUpdate(uint8_t handleFailure,
                                        gapRole_updateConnParams_t *pConnParams)
{
  bStatus_t status;

  // First check the current connection parameters versus the configured parameters
  if ((gapRole_ConnInterval < pConnParams->minConnInterval)   ||
       (gapRole_ConnInterval > pConnParams->maxConnInterval)   ||
       (gapRole_ConnSlaveLatency != pConnParams->slaveLatency) ||
       (gapRole_ConnTimeout  != pConnParams->timeoutMultiplier))
  {
    uint16_t timeout = GAP_GetParamValue(TGAP_CONN_PARAM_TIMEOUT);
#if defined(L2CAP_CONN_UPDATE)
    l2capParamUpdateReq_t updateReq;

    updateReq.intervalMin = pConnParams->minConnInterval;
    updateReq.intervalMax = pConnParams->maxConnInterval;
    updateReq.slaveLatency = pConnParams->slaveLatency;
    updateReq.timeoutMultiplier = pConnParams->timeoutMultiplier;

    status =  L2CAP_ConnParamUpdateReq(gapRole_ConnectionHandle, &updateReq, selfEntity);
#else
    gapUpdateLinkParamReq_t linkParams;

    linkParams.connectionHandle = gapRole_ConnectionHandle;
    linkParams.intervalMin = pConnParams->minConnInterval;
    linkParams.intervalMax = pConnParams->maxConnInterval;
    linkParams.connLatency = pConnParams->slaveLatency;
    linkParams.connTimeout = pConnParams->timeoutMultiplier;

    status = GAP_UpdateLinkParamReq( &linkParams );
#endif // L2CAP_CONN_UPDATE

    if(status == SUCCESS)
    {
      paramUpdateNoSuccessOption = handleFailure;
      // Let's wait either for L2CAP Connection Parameters Update Response or
      // for Controller to update connection parameters
      Util_restartClock(&updateTimeoutClock, timeout);
    }
  }
  else
  {
    status = bleInvalidRange;
  }

  return status;
}

/********************************************************************
 * @fn          GAPRole_SendUpdateParam
 *
 * @brief       Update the parameters of an existing connection
 *
 * @param       minConnInterval - the new min connection interval
 * @param       maxConnInterval - the new max connection interval
 * @param       latency - the new slave latency
 * @param       connTimeout - the new timeout value
 * @param       handleFailure - what to do if the update does not occur.
 *              Method may choose to terminate connection, try again,
 *              or take no action
 *
 * @return      SUCCESS: operation was successful.
 *              INVALIDPARAMETER: Data can not fit into one packet.
 *              MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *              bleInvalidRange:
 *              bleIncorrectMode: invalid profile role.
 *              bleAlreadyInRequestedMode: already updating link parameters.
 *              bleNotConnected: Connection is down
 *              bleMemAllocError: Memory allocation error occurred.
 *              bleNoResources: No available resource
 */
bStatus_t GAPRole_SendUpdateParam(uint16_t minConnInterval,
                                  uint16_t maxConnInterval,
                                  uint16_t latency, uint16_t connTimeout,
                                  uint8_t handleFailure)
{
  // If there is no existing connection no update need be sent
  if (gapRole_state != GAPROLE_CONNECTED)
  {
    return (bleNotConnected);
  }
  else
  {
    gapRole_updateConnParams_t paramUpdate;

    paramUpdate.minConnInterval = minConnInterval;
    paramUpdate.maxConnInterval = maxConnInterval;
    paramUpdate.slaveLatency = latency;
    paramUpdate.timeoutMultiplier = connTimeout;

    // Connection update requested by app, cancel such pending procedure (if active)
    Util_stopClock(&startUpdateClock);

    // Start connection update procedure
    return gapRole_startConnUpdate(handleFailure, &paramUpdate);
  }
}

/*********************************************************************
 * @fn      gapRole_setEvent
 *
 * @brief   Set an event
 *
 * @param   event - event to be set
 *
 * @return  none
 */
static void gapRole_setEvent(uint32_t event)
{
  // Wake up the application thread when it waits for clock event
  Event_post(syncEvent, event);
}

/*********************************************************************
 * @fn      gapRole_clockHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - event
 *
 * @return  none
 */
void gapRole_clockHandler(UArg a0)
{
  gapRole_setEvent(a0);
}

/*********************************************************************
*********************************************************************/
