/******************************************************************************

 @file  hid_adv_remote.c

 @brief HID Advanced Remote Application.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************

 Copyright (c) 2015-2016, Texas Instruments Incorporated
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

 *****************************************************************************/

/******************************************************************************
 * INCLUDES
 */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/mw/sensors/SensorI2C.h>
#include <ti/mw/sensors/SensorMpu9250.h>
#include <ti/mw/extflash/ExtFlash.h>
#include <ti/mw/remotecontrol/buzzer.h>
#include <ti/drivers/pdm/PDMCC26XX.h>

#include <ICall.h>
#include <string.h>
#include "hci.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "hidkbdccservice.h"
#include "hiddev.h"
#include "audio_profile.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "osal_snv.h"
#include "icall_apimsg.h"
#include "util.h"
#include "board.h"
#include "kcb.h"
#include "key_scan.h"
#include "hid_adv_remote.h"

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT              60000

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         50

// Supervision timeout value (units of 10ms, 1000=10s) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         10

// Default passcode
#define DEFAULT_PASSCODE                      0

// Length of MITM passcode
#define PASSCODE_LEN                          6

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     TRUE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_KEYBOARD_ONLY

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6

// Task configuration
#define HAR_TASK_PRIORITY                     1

#ifndef HAR_TASK_STACK_SIZE
#define HAR_TASK_STACK_SIZE                   644
#endif

#define HAR_AUDIO_MAX_ALLOC_BUF               10
#define HAR_MIC_KEY_RELEASE_TIME              500
#define HAR_STREAM_LIMIT_TIME                 30000

// Factory Reset & Image Select
#define EFL_ADDR_RECOVERY                     0x20000
#define EFL_SIZE_RECOVERY                     0x20000
#define BL_OFFSET                             0x1F001

// Internal Events for RTOS application
#define HAR_STATE_CHANGE_EVT                  0x0001
#define HAR_HANDLE_MIC_BUTTON_PRESS           0x0002
#define HAR_START_STREAMING_EVT               0x0004
#define HAR_STOP_STREAMING_EVT                0x0008
#define HAR_SEND_STOP_CMD_EVT                 0x0010
#define HAR_PROCESS_PDM_DATA_EVT              0x0020
#define HAR_KEY_PRESS_EVT                     0x0040

#define BLE_AUDIO_CMD_STOP                    0x00
#define BLE_AUDIO_CMD_START                   0x04

#define RAS_DATA_TIC1_CMD                     0x01

#define LED_DELAY_LENGTH                      100
#define NUM_BLINKS                            3
#define BUZZER_FREQUENCY                      1000

/******************************************************************************
 * TYPEDEFS
 */
// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // Event header
  uint8_t *pData;  // Event data
} harEvt_t;

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * EXTERNAL VARIABLES
 */

/******************************************************************************
 * EXTERNAL FUNCTIONS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */

// Events flag for internal application events
volatile uint16_t events = 0;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Task configuration
Task_Struct harTask;
Char harTaskStack[HAR_TASK_STACK_SIZE];

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  0x0A,                             // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,   // AD Type = Complete local name
  'C',
  'C',
  '2',
  '6',
  '5',
  '0',
  ' ',
  'R',
  'C'
};

// Advertising data
static uint8_t advertData[] =
{
  // flags
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // appearance
  0x03,   // length of this data
  GAP_ADTYPE_APPEARANCE,
  LO_UINT16(GAP_APPEARE_GENERIC_HID),
  HI_UINT16(GAP_APPEARE_GENERIC_HID),

  // service UUIDs
  0x05,   // length of this data
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(HID_SERV_UUID),
  HI_UINT16(HID_SERV_UUID),
  LO_UINT16(BATT_SERV_UUID),
  HI_UINT16(BATT_SERV_UUID)
};

// Device name attribute value
static uint8_t attDeviceName[] = "CC2650 RC";

// HID Dev configuration
static hidDevCfg_t hidAdvRemoteCfg =
{
  DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
  HID_KBD_FLAGS               // HID feature flags
};

// GPIO pin table
static PIN_Config harPinTable[] =
{
  Board_LED_G  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_LED_R  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_LED_IR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

// GPIO pin state
static PIN_State harPinState;

// GPIO Pin handle
PIN_Handle harPinHandle;

// PDM parameters
static PDMCC26XX_Handle pdmHandle;

static void HIDAdvRemote_pdmCB(PDMCC26XX_Handle handle,
                              PDMCC26XX_StreamNotification *streamNotification);

static void *HIDAdvRemote_audioMalloc(uint_least16_t size);
static void HIDAdvRemote_audioFree(void *msg);

PDMCC26XX_Params pdmParams =
{
  .callbackFxn = HIDAdvRemote_pdmCB,
  .useDefaultFilter = true,
  .decimationFilter = NULL,
  .micGain = PDMCC26XX_GAIN_18,
  .micPowerActiveHigh = true,
  .applyCompression = true,
  .startupDelayWithClockInSamples = 512,
  .retBufSizeInBytes = BLEAUDIO_HDRSIZE + BLEAUDIO_BUFSIZE,
  .mallocFxn = (PDMCC26XX_MallocFxn) HIDAdvRemote_audioMalloc,
  .freeFxn = (PDMCC26XX_FreeFxn) HIDAdvRemote_audioFree,
  .custom = NULL
};

// Counter for number of PDM buffers
static uint8_t harAudioBufCount = 0;

// Mic key release clock
static Clock_Struct micKeyReleaseClock;

// Stream limit clock
static Clock_Struct streamLimitClock;

// Lock for PDM streaming
volatile static uint8_t pdmLock = FALSE;

// Current key being pressed
uint8 currKey;

// MITM pairing variables
static uint8_t remainingPasscodeDigits = 0;
static uint32_t passcode;

// Application GAP Role and GAP Bond states
static gaprole_States_t harGapRoleState = GAPROLE_INIT;
static uint8_t harGapBondState = HID_GAPBOND_PAIRING_STATE_NONE;

// Flag is set while MIC button is pressed
volatile static uint8_t streamFlag = FALSE;

/******************************************************************************
 * LOCAL FUNCTIONS
 */

static void HIDAdvRemote_init(void);
static void HIDAdvRemote_taskFxn(UArg a0, UArg a1);
static void HIDAdvRemote_processStackMsg(ICall_Hdr *pMsg);
static void HIDAdvRemote_processGattMsg(gattMsgEvent_t *pMsg);

static uint8_t HIDAdvRemote_transmitAudioStreamCmd(uint8_t cmd);
static void HIDAdvRemote_startStreamingVoice(void);
static void HIDAdvRemote_processPdmData(void);
static void HIDAdvRemote_transmitAudioFrame(uint8_t *buf);
static void HIDAdvRemote_stopStreamingVoice(void);
static void HIDAdvRemote_sendStopCmd(void);
static void HIDAdvRemote_finishStream(void);
static void HIDAdvRemote_micKeyReleaseCB(UArg a0);
static void HIDAdvRemote_signalStopStreamEvt(void);
static void HIDAdvRemote_streamLimitCB(UArg a0);

void HIDAdvRemote_ProcessKeyEvent(uint8 key);
void HIDAdvRemote_ProcessKey(rcKeyId_t harCurrentKeyId,
                             rcKeyState_t harCurrentKeyState);

static void HIDAdvRemote_sendReport(uint8_t keycode);
static void HIDAdvRemoteConCtrl_sendReport(uint8_t keycode);

static void HIDAdvRemote_passcodeHandler(rcKeyId_t harCurrentKeyId,
                                         rcKeyState_t harCurrentKeyState);
static void HIDAdvRemote_buildPasscode(rcKeyId_t harCurrentKeyId);

static void HIDAdvRemote_giveUserFeedback(void);
static void HIDAdvRemote_unpairKeyAction(void);

#ifdef RC_OAD_ENABLE
static void HIDAdvRemote_applyFactoryImage(void);
#endif // RC_OAD_ENABLE

static uint8_t HIDAdvRemote_handleReportCB(uint8_t id, uint8_t type,
                                           uint16_t uuid, uint8_t oper,
                                           uint16_t *pLen, uint8_t *pData);
static void HIDAdvRemote_handleEventCB(uint8_t evt);
static void HIDAdvRemote_passcodeCB(uint8 *deviceAddr, uint16 connectionHandle,
                                    uint8 uiInputs, uint8 uiOutputs);

/******************************************************************************
 * PROFILE CALLBACKS
 */

// HID Dev callbacks
static hidDevCB_t hidAdvRemoteHidCBs =
{
  HIDAdvRemote_handleReportCB,
  HIDAdvRemote_handleEventCB,
  HIDAdvRemote_passcodeCB
};

/******************************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HIDAdvRemote_createTask
 *
 * @brief   Task creation function for the HID Advanced Remote.
 *
 * @param   None.
 *
 * @return  None.
 */
void HIDAdvRemote_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = harTaskStack;
  taskParams.stackSize = HAR_TASK_STACK_SIZE;
  taskParams.priority = HAR_TASK_PRIORITY;

  Task_construct(&harTask, HIDAdvRemote_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      HIDAdvRemote_init
 *
 * @brief   Initialization function for the Advance Remote App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Setup the GAP
  VOID GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL,
                         DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t gapRole_AdvertOffTime = 0;

    uint8_t enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initial_advertising_enable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &gapRole_AdvertOffTime);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData),
                         advertData);
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enable_update_request);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desired_min_interval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desired_max_interval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desired_slave_latency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desired_conn_timeout);
  }

  // Set the GAP Characteristics
  {
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                     (void *) attDeviceName);
  }

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
    uint32_t passkey = DEFAULT_PASSCODE;
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    uint8_t mitm = DEFAULT_MITM_MODE;
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    uint8_t bonding = DEFAULT_BONDING_MODE;
    uint8_t autoSyncWhiteList = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t),
                            &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t),
                            &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t),
                            &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t),
                            &bonding);
    GAPBondMgr_SetParameter(GAPBOND_AUTO_SYNC_WL, sizeof(uint8_t),
                            &autoSyncWhiteList);
  }

  // Setup Battery Characteristic Values
  {
    uint8_t critical = DEFAULT_BATT_CRITICAL_LEVEL;

    Batt_SetParameter(BATT_PARAM_CRITICAL_LEVEL, sizeof(uint8_t),
                      &critical);
  }

  // Open LED PIN driver
  harPinHandle = PIN_open(&harPinState, harPinTable);

  // Initialize PDM Driver
  PDMCC26XX_init((PDMCC26XX_Handle) &(PDMCC26XX_config));

  // Initialize mic key release timer (500ms)
  Util_constructClock(&micKeyReleaseClock, HIDAdvRemote_micKeyReleaseCB,
                      HAR_MIC_KEY_RELEASE_TIME, 0, false, NULL);

  // Initialize stream limit timer (30 sec)
  Util_constructClock(&streamLimitClock, HIDAdvRemote_streamLimitCB,
                      HAR_STREAM_LIMIT_TIME, 0, false, NULL);

  // Initialize and Configure Key Driver
  KeyInit();
  KeyConfig(HIDAdvRemote_ProcessKeyEvent, HAR_KEY_REPEAT_INTERVAL,
            HAR_KEY_DEBOUNCE_TIME, HAR_KEY_POLL_RATE);
  kcb_Init(HIDAdvRemote_passcodeHandler, KCB_ONE_KEYS_SUPPORTED);

  // Initialize unused MPU to sleep
  SensorI2C_open();
  SensorMpu9250_init();

  // Test External Flash and put to sleep
  ExtFlash_test();

  // Add Audio Service
  Audio_AddService();

  // Add HID keyboard service
  HidKbdCC_AddService();

  // Register for HID Dev callback
  HidDev_Register(&hidAdvRemoteCfg, &hidAdvRemoteHidCBs);

  // Start the GAP Role and Register the Bond Manager.
  HidDev_StartDevice();
}

/*********************************************************************
 * @fn      HIDADvRemote_taskFxn
 *
 * @brief   Application task entry point for the HID Advance Remote Application.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void HIDAdvRemote_taskFxn(UArg a0, UArg a1)
{
  uint32_t hwiKey;
  volatile uint16_t localEvents = 0;
  volatile static uint8_t pdmStream = FALSE;

  // Initialize application
  HIDAdvRemote_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signalled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest, (void **)&pMsg) ==
          ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          HIDAdvRemote_processStackMsg((ICall_Hdr *) pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    // Handle events one at a time
    hwiKey = Hwi_disable();
    localEvents = events;
    events = 0;
    Hwi_restore(hwiKey);

    if (localEvents & HAR_HANDLE_MIC_BUTTON_PRESS)
    {
      if ((harGapBondState == GAPBOND_PAIRING_STATE_COMPLETE) ||
          (harGapBondState == GAPBOND_PAIRING_STATE_BONDED) ||
          (harGapBondState == GAPBOND_PAIRING_STATE_BOND_SAVED))
      {
        hwiKey = Hwi_disable();
        events |= HAR_START_STREAMING_EVT;
        Hwi_restore(hwiKey);
        Semaphore_post(sem);
      }
      else if ((harGapRoleState != GAPROLE_ADVERTISING) &&
               (harGapRoleState != GAPROLE_CONNECTED))
      {
        HidDev_StartAdvertising();
      }
    }

    if (localEvents & HAR_START_STREAMING_EVT)
    {
      if ((!pdmLock) && (!pdmStream) && (harGapRoleState == GAPROLE_CONNECTED))
      {
        pdmStream = TRUE;
        HIDAdvRemote_startStreamingVoice();
      }
    }

    if (localEvents & HAR_PROCESS_PDM_DATA_EVT)
    {
      if (harGapRoleState == GAPROLE_CONNECTED)
      {
        if (pdmStream)
        {
          HIDAdvRemote_processPdmData();
        }
      }
      else
      {
        hwiKey = Hwi_disable();
        events |= HAR_STOP_STREAMING_EVT;
        Hwi_restore(hwiKey);
        Semaphore_post(sem);
      }
    }

    if (localEvents & HAR_STOP_STREAMING_EVT)
    {
      if (pdmStream)
      {
        HIDAdvRemote_stopStreamingVoice();
      }
    }

    if (localEvents & HAR_SEND_STOP_CMD_EVT)
    {
      HIDAdvRemote_sendStopCmd();
      pdmStream = FALSE;
    }

    if (localEvents & HAR_KEY_PRESS_EVT)
    {
      kcb_ProcessKey(currKey);
    }
  }
}

/*********************************************************************
 * @fn      HIDAdvRemote_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void HIDAdvRemote_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      HIDAdvRemote_processGattMsg((gattMsgEvent_t *)pMsg);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      HIDAdvRemote_processGattMsg
 *
 * @brief   Process GATT messages
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void HIDAdvRemote_processGattMsg(gattMsgEvent_t *pMsg)
{
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      HIDAdvRemote_transmitAudioStreamCmd
 *
 * @brief   Transmits GATT Notification in order to start or stop stream
 *
 * @param   cmd - command to transmit
 *
 * @return  SUCCESS if successful, FAILURE if not
 */
static uint8_t HIDAdvRemote_transmitAudioStreamCmd(uint8_t cmd)
{
  return Audio_SetParameter(AUDIOPROFILE_START, AUDIOPROFILE_CMD_LEN, &cmd);
}

/*********************************************************************
 * @fn      HIDAdvRemote_audioMalloc
 *
 * @brief   allocates a audio block
 *
 * @param   size - size of the block in bytes
 *
 * @return  pointer to buffer if successful, NULL if failed
 */
static void *HIDAdvRemote_audioMalloc(uint_least16_t size)
{
  uint8_t *rtnAddr = NULL;
  uint_least32_t taskKey;
  uint8_t flag = 0;
  uint32_t hwiKey;

  /* Check number of simultaneous allocated buffers; do not allow more than
   * HAR_AUDIO_MAX_ALLOC_BUF simultaneous allocated buffers
   */
  taskKey = Task_disable();

  if (harAudioBufCount < HAR_AUDIO_MAX_ALLOC_BUF)
  {
    flag = 1;
    harAudioBufCount++;
  }
  else
  {
    // Reached maximum buffer size; must process data
    hwiKey = Hwi_disable();
    events |= HAR_PROCESS_PDM_DATA_EVT;
    Hwi_restore(hwiKey);
    Semaphore_post(sem);
  }

  Task_restore(taskKey);

  if (flag)
  {
    rtnAddr = (uint8_t *) ICall_malloc(size);
  }

  return rtnAddr;
}

/*********************************************************************
 * @fn      HIDAdvRemote_audioFree
 *
 * @brief   Frees an allocated audio block
 *
 * @param   msg - pointer to a memory block to free
 *
 * @return  None.
 */
static void HIDAdvRemote_audioFree(void *msg)
{
  uint8_t *pBuf = (uint8_t *)msg;
  uint_least32_t taskKey;

  taskKey = Task_disable();
  harAudioBufCount--;
  Task_restore(taskKey);

  ICall_free((void *) pBuf);
}

/*********************************************************************
 * @fn      HIDAdvRemote_startStreamingVoice
 *
 * @brief   Starts streaming audio to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_startStreamingVoice(void)
{
  // LED on while streaming
  PIN_setOutputValue(harPinHandle, Board_LED_R, 1);

  // Stop HID Idle timer during stream
  HidDev_StopIdleTimer();

  // Increase TX power during stream
  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);

  /* Send the start cmd. If it doesn't go through, return value is not SUCCESS,
   * and state should not be changed to STREAMING.
   */
  if (HIDAdvRemote_transmitAudioStreamCmd(BLE_AUDIO_CMD_START) == SUCCESS)
  {
    // Open PDM driver
    if (!pdmHandle)
    {
      pdmHandle = PDMCC26XX_open(&pdmParams);

      if (pdmHandle)
      {
        Util_restartClock(&streamLimitClock, HAR_STREAM_LIMIT_TIME);
        PDMCC26XX_startStream(pdmHandle);
      }
    }
  }
}

/*********************************************************************
 * @fn      HIDAdvRemote_processPdmData
 *
 * @brief   Processes data from PDM driver
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_processPdmData(void)
{
  uint32_t hwiKey;
  static PDMCC26XX_BufferRequest bufferRequest;
  uint8_t *pAudioFrame = NULL;
  uint8_t tmpSeqNum;

  // Request new audio frame / buffer
  if (PDMCC26XX_requestBuffer(pdmHandle, &bufferRequest))
  {
    pAudioFrame = ((uint8 *) (bufferRequest.buffer));

    // First audio frame byte: 5 bits seq num, 3 bits data cmd
    tmpSeqNum = (((PDMCC26XX_pcmBuffer *)pAudioFrame)->metaData).seqNum;
    pAudioFrame[0] = (((tmpSeqNum % 32) << 3) | RAS_DATA_TIC1_CMD);

    // Transmit processed audio frame
    HIDAdvRemote_transmitAudioFrame(pAudioFrame);

    // Free audio frame
    HIDAdvRemote_audioFree(pAudioFrame);

    // Process additional audio frames if available
    hwiKey = Hwi_disable();
    if (harAudioBufCount)
    {
      events |= HAR_PROCESS_PDM_DATA_EVT;
      Hwi_restore(hwiKey);
      Semaphore_post(sem);
    }
    else
    {
      Hwi_restore(hwiKey);
    }
  }
}

/*********************************************************************
 * @fn      HIDAdvRemote_transmitAudioFrame
 *
 * @brief   Transmits processed audio frame to connected device
 *
 * @param   buf - pointer to PDM buffer
 *
 * @return  None.
 */
static void HIDAdvRemote_transmitAudioFrame(uint8_t *buf)
{
  // Send 5 GATT notifications for every audio frame
  for (int i = 0; i < BLEAUDIO_NUM_NOT_PER_FRAME; )
  {
    if (Audio_SetParameter(AUDIOPROFILE_AUDIO, BLEAUDIO_NOTSIZE, buf) == SUCCESS)
    {
      // Move on to next section of audio frame
      buf += BLEAUDIO_NOTSIZE;
      i++;
    }
  }
}

/*********************************************************************
 * @fn      HIDAdvRemote_stopStreamingVoice
 *
 * @brief   Stops streaming audio to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_stopStreamingVoice(void)
{
  uint32_t hwiKey;

  if (pdmHandle)
  {
    // stop stream
    PDMCC26XX_stopStream(pdmHandle);

    // close driver
    PDMCC26XX_close(pdmHandle);
    pdmHandle = NULL;
  }

  hwiKey = Hwi_disable();
  events |= HAR_SEND_STOP_CMD_EVT;
  Hwi_restore(hwiKey);
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      HIDAdvRemote_sendStopCmd
 *
 * @brief   Sends a stop command to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_sendStopCmd(void)
{
  uint32_t hwiKey;

  if (harGapRoleState == GAPROLE_CONNECTED)
  {
    if (HIDAdvRemote_transmitAudioStreamCmd(BLE_AUDIO_CMD_STOP) == SUCCESS)
    {
      HIDAdvRemote_finishStream();
    }
    else
    {
      hwiKey = Hwi_disable();
      events |= HAR_SEND_STOP_CMD_EVT;
      Hwi_restore(hwiKey);
      Semaphore_post(sem);
    }
  }
  else
  {
    HIDAdvRemote_finishStream();
  }
}


/*********************************************************************
 * @fn      HIDAdvRemote_finishStream
 *
 * @brief   Helper function that finishes process of streaming
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_finishStream(void)
{
  uint32_t hwiKey;

  // LED off
  PIN_setOutputValue(harPinHandle, Board_LED_R, 0);

  // Restart HID Idle timer after stream
  HidDev_StartIdleTimer();

  // Reset TX power
  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);

  // Clear event
  hwiKey = Hwi_disable();
  events &= ~HAR_SEND_STOP_CMD_EVT;
  Hwi_restore(hwiKey);

  // Temporarily disable streaming
  pdmLock = TRUE;
  Util_restartClock(&micKeyReleaseClock, HAR_MIC_KEY_RELEASE_TIME);
}

/*********************************************************************
 * @fn      HIDAdvRemote_pdmCB
 *
 * @brief   Callback function for the PDM driver
 *
 * @param   handle - handle of PDM object
 *          pStreamNotification - notification of available buffers
 *                                and potential errors
 *
 * @return  None.
 */
static void HIDAdvRemote_pdmCB(PDMCC26XX_Handle handle,
                              PDMCC26XX_StreamNotification *pStreamNotification)
{
  events |= HAR_PROCESS_PDM_DATA_EVT;

  if (pStreamNotification->status != PDMCC26XX_STREAM_BLOCK_READY)
  {
    events |= HAR_STOP_STREAMING_EVT;
  }

  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      HIDAdvRemote_micKeyReleaseCB
 *
 * @brief   Enables voice streaming
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_micKeyReleaseCB(UArg a0)
{
  // Unlock variable - allow PDM streaming to happen again
  pdmLock = FALSE;
}


/*********************************************************************
 * @fn      HIDAdvRemote_signalStopStreamEvt
 *
 * @brief   Sets the Stop Stream Event
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_signalStopStreamEvt(void)
{
  uint32_t hwiKey;

  hwiKey = Hwi_disable();
  events |= HAR_STOP_STREAMING_EVT;
  Hwi_restore(hwiKey);
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      HIDAdvRemote_streamLimitCB
 *
 * @brief   Limits time of voice streaming
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_streamLimitCB(UArg a0)
{
  HIDAdvRemote_signalStopStreamEvt();
}

/*********************************************************************
 *
 * @fn      HIDAdvRemote_ProcessKeyEvent
 *
 * @brief   Process event from keyScan module
 *
 * @param   key - ID of the key that caused event
 *
 * @return  none
 *
 */
void HIDAdvRemote_ProcessKeyEvent(uint8 key)
{
  uint32_t hwiKey;

  // Save key
  currKey = key;

  // Set event
  hwiKey = Hwi_disable();
  events |= HAR_KEY_PRESS_EVT;
  Hwi_restore(hwiKey);
  Semaphore_post(sem);
}

/*********************************************************************
 *
 * @fn      HIDAdvRemote_ProcessKey
 *
 * @brief   Callback service for keys
 *
 * @param   harCurrentKeyId - keys that were pressed
 *          harCurrentKeyState - state that key is in (pressed, released, etc.)
 *
 * @return  void
 *
 */
void HIDAdvRemote_ProcessKey(rcKeyId_t harCurrentKeyId,
                             rcKeyState_t harCurrentKeyState)
{
  static rcKeyId_t    harPrevKeyId    = RC_SW_NA;
  static rcKeyState_t harPrevKeyState = RC_KEY_STATE_NOT_PRESSED;
  uint32_t hwiKey;

  if (harCurrentKeyState == RC_KEY_STATE_PRESSED)
  {
    switch (harCurrentKeyId)
    {
      case RC_SW_PWR:
        HIDAdvRemoteConCtrl_sendReport(HID_CC_RPT_POWER);
        break;

      case RC_SW_TV:
        break;

      case RC_SW_STB:
        break;

      case RC_SW_VOL_UP:
        HIDAdvRemoteConCtrl_sendReport(HID_CC_RPT_VOLUME_UP);
        break;

      case RC_SW_MUTE:
        HIDAdvRemoteConCtrl_sendReport(HID_CC_RPT_MUTE);
        break;

      case RC_SW_VOL_DN:
        HIDAdvRemoteConCtrl_sendReport(HID_CC_RPT_VOLUME_DOWN);
        break;

      case RC_SW_MENU:
        HIDAdvRemote_sendReport(HID_KEYBOARD_LEFT_GUI);
        break;

      case RC_SW_HOME:
        HIDAdvRemote_sendReport(HID_KEYBOARD_RIGHT_GUI);
        break;

      case RC_SW_BACK:
        HIDAdvRemote_sendReport(HID_KEYBOARD_DELETE);
        break;

      case RC_SW_UP_ARW:
        HIDAdvRemote_sendReport(HID_KEYBOARD_UP_ARROW);
        break;

      case RC_SW_ENTER:
        HIDAdvRemote_sendReport(HID_KEYBOARD_RETURN);
        break;

      case RC_SW_LF_ARW:
        HIDAdvRemote_sendReport(HID_KEYBOARD_LEFT_ARROW);
        break;

      case RC_SW_RT_ARW:
        HIDAdvRemote_sendReport(HID_KEYBOARD_RIGHT_ARROW);
        break;

      case RC_SW_DN_ARW:
        HIDAdvRemote_sendReport(HID_KEYBOARD_DOWN_ARROW);
        break;

      case RC_SW_VOICE:
        streamFlag = TRUE;

        hwiKey = Hwi_disable();
        events |= HAR_HANDLE_MIC_BUTTON_PRESS;
        Hwi_restore(hwiKey);
        Semaphore_post(sem);
        break;

      case RC_SW_PAIR:
        break;

      case RC_SW_INFINITY:
        break;

      case RC_SW_DIG_1:
        HIDAdvRemote_sendReport(HID_KEYBOARD_1);
        break;

      case RC_SW_DIG_2:
        HIDAdvRemote_sendReport(HID_KEYBOARD_2);
        break;

      case RC_SW_DIG_3:
        HIDAdvRemote_sendReport(HID_KEYBOARD_3);
        break;

      case RC_SW_DIG_4:
        HIDAdvRemote_sendReport(HID_KEYBOARD_4);
        break;

      case RC_SW_DIG_5:
        HIDAdvRemote_sendReport(HID_KEYBOARD_5);
        break;

      case RC_SW_DIG_6:
        HIDAdvRemote_sendReport(HID_KEYBOARD_6);
        break;

      case RC_SW_DIG_7:
        HIDAdvRemote_sendReport(HID_KEYBOARD_7);
        break;

      case RC_SW_DIG_8:
        HIDAdvRemote_sendReport(HID_KEYBOARD_8);
        break;

      case RC_SW_DIG_9:
        HIDAdvRemote_sendReport(HID_KEYBOARD_9);
        break;

      case RC_SW_AV:
        break;

      case RC_SW_RECORD:
        HIDAdvRemoteConCtrl_sendReport(HID_CC_RPT_RECORD);
        break;

      case RC_SW_DIG_0:
        HIDAdvRemote_sendReport(HID_KEYBOARD_0);
        break;

      case RC_SW_PREV:
        break;

      case RC_SW_NEXT:
        break;

      case RC_SW_PLAY:
        HIDAdvRemoteConCtrl_sendReport(HID_CC_RPT_PLAY_PAUSE);
        break;

      default:
        break;
    }
  }
  else if ((harCurrentKeyState == RC_KEY_STATE_NOT_PRESSED) &&
           (harPrevKeyState    != RC_KEY_STATE_NOT_PRESSED))
  {
    switch(harCurrentKeyId)
    {
      case RC_SW_VOICE:
        if (harPrevKeyId == RC_SW_VOICE)
        {
          streamFlag = FALSE;
          HIDAdvRemote_signalStopStreamEvt();
        }

        break;

      case RC_SW_PREV:
        if ((harPrevKeyState == RC_KEY_STATE_PRESSED) ||
            (harPrevKeyState == RC_KEY_STATE_PRESSED_NOT_YET_REPEATED))
        {
          HIDAdvRemoteConCtrl_sendReport(HID_CC_RPT_SCAN_PREV_TRK);
        }

        HIDAdvRemoteConCtrl_sendReport(KEY_NONE);
        break;

      case RC_SW_NEXT:
        if ((harPrevKeyState == RC_KEY_STATE_PRESSED) ||
            (harPrevKeyState == RC_KEY_STATE_PRESSED_NOT_YET_REPEATED))
        {
          HIDAdvRemoteConCtrl_sendReport(HID_CC_RPT_SCAN_NEXT_TRK);
        }

        HIDAdvRemoteConCtrl_sendReport(KEY_NONE);
        break;

      default:
        if (((harCurrentKeyId >= RC_SW_UP_ARW) &&
             (harCurrentKeyId <= RC_SW_DIG_9)) ||
            ((harCurrentKeyId >= RC_SW_MENU) &&
             (harCurrentKeyId <= RC_SW_ENTER)))
        {
          HIDAdvRemote_sendReport(KEY_NONE);
        }
        else
        {
          HIDAdvRemoteConCtrl_sendReport(KEY_NONE);
        }

        break;
    }
  }
  else if ((harCurrentKeyState == RC_KEY_STATE_REPEATED) &&
           (harPrevKeyState    != RC_KEY_STATE_REPEATED))
  {
    switch(harCurrentKeyId)
    {
      case RC_SW_PAIR:
        HIDAdvRemote_unpairKeyAction();
        break;

#ifdef RC_OAD_ENABLE
      case RC_SW_HOME:
        HIDAdvRemote_applyFactoryImage();
        break;
#endif // RC_OAD_ENABLE

      case RC_SW_PREV:
        HIDAdvRemoteConCtrl_sendReport(HID_CC_RPT_REWIND);
        break;

      case RC_SW_NEXT:
        HIDAdvRemoteConCtrl_sendReport(HID_CC_RPT_FAST_FWD);
        break;

      default:
        break;
    }
  }

  harPrevKeyState = harCurrentKeyState;
  harPrevKeyId = harCurrentKeyId;
}

/*********************************************************************
 * @fn      HIDAdvRemote_sendReport
 *
 * @brief   Build and send a HID Keyboard report
 *
 * @param   keycode - HID keycode
 *
 * @return  None.
 */
static void HIDAdvRemote_sendReport(uint8_t keycode)
{
  uint8_t buf[HID_KEYBOARD_IN_RPT_LEN];

  // No need to include Report Id
  buf[0] = 0;                 // no modifier keys
  buf[1] = 0;                 // Reserved
  buf[2] = keycode;           // Keycode 1
  buf[3] = 0;                 // Keycode 2
  buf[4] = 0;                 // Keycode 3
  buf[5] = 0;                 // Keycode 4
  buf[6] = 0;                 // Keycode 5
  buf[7] = 0;                 // Keycode 6

  HidDev_Report(HID_RPT_ID_KEY_IN,
                HID_REPORT_TYPE_INPUT,
                HID_KEYBOARD_IN_RPT_LEN,
                buf);
}

/*********************************************************************
 * @fn      HIDAdvRemoteConCtrl_sendReport
 *
 * @brief   Build and send a HID Consumer Control report
 *
 * @param   keycode - HID keycode
 *
 * @return  None.
 */
static void HIDAdvRemoteConCtrl_sendReport(uint8_t keycode)
{
  uint8_t buf[HID_CC_IN_RPT_LEN];

  buf[0] = keycode;   // keycode

  HidDev_Report(HID_RPT_ID_CC_IN,
                HID_REPORT_TYPE_INPUT,
                HID_CC_IN_RPT_LEN,
                buf);
}

/*********************************************************************
 * @fn      HIDAdvRemote_passcodeHandler
 *
 * @brief   Build and send a HID Keyboard report
 *
 * @param   keycode - HID keycode
 *
 * @return  None.
 */
static void HIDAdvRemote_passcodeHandler(rcKeyId_t    harCurrentKeyId,
                                         rcKeyState_t harCurrentKeyState)
{
  if (remainingPasscodeDigits == 0)
  {
    HIDAdvRemote_ProcessKey(harCurrentKeyId, harCurrentKeyState);
  }
  else
  {
    if (harCurrentKeyState == RC_KEY_STATE_PRESSED)
    {
      HIDAdvRemote_buildPasscode(harCurrentKeyId);
    }
  }
}

/*********************************************************************
 * @fn      HIDAdvRemote_buildPasscode
 *
 * @brief   Build and send a passcode.
 *
 * @param   harCurrentKeyId - HID keycode
 *
 * @return  none
 */
static void HIDAdvRemote_buildPasscode(rcKeyId_t harCurrentKeyId)
{
  if ((harCurrentKeyId >= RC_SW_DIG_0) && (harCurrentKeyId <= RC_SW_DIG_9))
  {
    // Append new digit to passcode
    passcode *= 10;
    if ((harCurrentKeyId >= RC_SW_DIG_0) && (harCurrentKeyId <= RC_SW_DIG_9))
    {
      passcode += (harCurrentKeyId - RC_SW_DIG_1 + 1);
    }

    if (--remainingPasscodeDigits == 0)
    {
       // Send passcode response
      HidDev_PasscodeRsp(SUCCESS, passcode);
    }
  }
  else
  {
    // Send passcode response
    HidDev_PasscodeRsp(SMP_PAIRING_FAILED_PASSKEY_ENTRY_FAILED, passcode);

    remainingPasscodeDigits = 0;
  }
}

/*********************************************************************
 * @fn      HIDAdvRemote_giveUserFeedback
 *
 * @brief   Blink LED 3 times and emit sound
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_giveUserFeedback(void)
{
  // Turn on sound
  buzzerOpen(harPinHandle);
  buzzerSetFrequency(BUZZER_FREQUENCY);

  // Blink LED
  for (uint8_t i = 0; i <= NUM_BLINKS; i++)
  {
    PIN_setOutputValue(harPinHandle, Board_LED_R, 1);
    DELAY_MS(LED_DELAY_LENGTH);
    PIN_setOutputValue(harPinHandle, Board_LED_R, 0);
    DELAY_MS(LED_DELAY_LENGTH);
  }

  // Turn off sound
  buzzerClose();
}

/*********************************************************************
 * @fn      HIDAdvRemote_unpairKeyAction
 *
 * @brief   Erase bonds and emit sound to confirm
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_unpairKeyAction(void)
{
  HidDev_SetParameter(HIDDEV_ERASE_ALLBONDS, 0, NULL);
  HIDAdvRemote_giveUserFeedback();
}


#ifdef RC_OAD_ENABLE
/*********************************************************************
 * @fn      HIDAdvRemote_applyFactoryImage
 *
 * @brief   Flashes remote with factory image loaded in memory
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_applyFactoryImage(void)
{
  // Load and launch factory image; page 31 must be omitted
  ((void (*)(uint32_t, uint32_t, uint32_t))BL_OFFSET)(EFL_ADDR_RECOVERY,
                                                      EFL_SIZE_RECOVERY-0x1000,
                                                      0);
}
#endif // RC_OAD_ENABLE

/*********************************************************************
 * @fn      HIDAdvRemote_handleReportCB
 *
 * @brief   HID Dev report callback.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   uuid  - attribute uuid.
 * @param   oper  - operation:  read, write, etc.
 * @param   pLen  - Length of report.
 * @param   pData - Report data.
 *
 * @return  SUCCESS
 */
static uint8_t HIDAdvRemote_handleReportCB(uint8_t id, uint8_t type,
                                           uint16_t uuid, uint8_t oper,
                                           uint16_t *pLen, uint8_t *pData)
{
  // Intentionally left blank
  return SUCCESS;
}

/*********************************************************************
 * @fn      HIDAdvRemote_handleEventCB
 *
 * @brief   HID Dev event callback.
 *
 * @param   evt - event ID.
 *
 * @return  None.
 */
static void HIDAdvRemote_handleEventCB(uint8_t evt)
{
  uint32_t hwiKey;

  switch(evt)
  {
    case(HID_DEV_GAPROLE_STATE_CHANGE_EVT):
      // Update application GAP Role state
      HidDev_GetParameter(HIDDEV_GAPROLE_STATE, &harGapRoleState);
      break;

    case(HID_DEV_GAPBOND_STATE_CHANGE_EVT):
      // Update application GAP Bond pairing state
      HidDev_GetParameter(HIDDEV_GAPBOND_STATE, &harGapBondState);

      // Send to MIC press event
      if (streamFlag)
      {
        hwiKey = Hwi_disable();
        events |= HAR_HANDLE_MIC_BUTTON_PRESS;
        Hwi_restore(hwiKey);
        Semaphore_post(sem);
      }

      break;

    default:
      // Do nothing
      break;
  }

  return;
}

/*********************************************************************
 * @fn      HIDAdvRemote_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @param   deviceAddr - address of device to pair with, and could be
 *                       either public or random.
 *          connectionHandle - connection handle
 *          uiInputs - pairing User Interface Inputs - Ask user to
 *                     input passcode
 *          uiOutputs - pairing User Interface Outputs - Display passcode
 *
 * @return  None.
 */
static void HIDAdvRemote_passcodeCB(uint8 *deviceAddr, uint16 connectionHandle,
                                    uint8 uiInputs, uint8 uiOutputs)
{
  remainingPasscodeDigits = PASSCODE_LEN;
  passcode = 0;
}

/******************************************************************************
 ******************************************************************************/
