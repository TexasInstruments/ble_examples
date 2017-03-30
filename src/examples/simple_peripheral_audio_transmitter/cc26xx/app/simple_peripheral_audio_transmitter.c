
/*
 * Filename: simple_peripheral_audio_transmitter.c
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
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hci_tl.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
//#include "devinfoservice.h"
//#include "simple_gatt_profile.h"

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include <ti/mw/display/Display.h>
#include "board_key.h"

#include "board.h"

#include "simple_peripheral.h"

#include "audio_profile.h"

#include <ti/drivers/pdm/Codec1.h>
#include "audiocodec.h"
#include "msbc_library.h"
#include "I2SCC26XX.h"

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     16
#else //!FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

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

// How often to perform periodic event (in msec)
//#define SBP_PERIODIC_EVT_PERIOD               DEFAULT_DESIRED_MIN_CONN_INTERVAL+2
#define SBP_PERIODIC_EVT_PERIOD               8

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1


#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004
#define SBP_CONN_EVT_END_EVT                  0x0008
#define SBP_KEY_CHANGE_EVT                    0x0010
#define SBP_I2S_FRAME_EVENT                   0x0020
#define SBP_I2S_ERROR_EVENT                   0x0040
#define SBP_SEND_STOP_CMD_EVENT               0x0080
#define SBP_SEND_START_CMD_EVENT              0x0100
#define SBP_STOP_I2S_EVENT                    0x0200
#define SBP_START_I2S_EVENT                   0x0400

#define DLE_MAX_PDU_SIZE 251
#define DLE_MAX_TX_TIME 2120

#define DEFAULT_PDU_SIZE 27
#define DEFAULT_TX_TIME 328

// The combined overhead for L2CAP and ATT notification headers
#define TOTAL_PACKET_OVERHEAD 7

// GATT notifications for throughput example don't require an authenticated link
#define GATT_NO_AUTHENTICATION 0

#define BLE_AUDIO_CMD_STOP                    0x00
#define BLE_AUDIO_CMD_START                   0x04
#define BLE_AUDIO_CMD_START_MSBC              0x05
#define BLE_AUDIO_CMD_NONE                    0xFF

#define RAS_DATA_TIC1_CMD                     0x01

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

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x14,   // length of this data
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
#if !defined(FEATURE_OAD) || defined(FEATURE_OAD_ONCHIP)
  0x03,   // length of this data
#else //OAD for external flash
  0x05,  // lenght of this data
#endif //FEATURE_OAD
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
#endif //FEATURE_OAD
#ifndef FEATURE_OAD_ONCHIP
  LO_UINT16(AUDIO_SERV_UUID),
  HI_UINT16(AUDIO_SERV_UUID)
#endif //FEATURE_OAD_ONCHIP
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE AudioTx";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

#define INPUT_OPTION      AUDIO_CODEC_MIC_ONBOARD
#define BLEAUDIO_BUFSIZE_ADPCM            96
#define BLEAUDIO_HDRSIZE_ADPCM            4

#ifdef DLE_ENABLED // Data Length Extension Enable
#define BLEAUDIO_NUM_NOT_PER_FRAME_ADPCM  1
#define BLEAUDIO_NUM_NOT_PER_FRAME_MSBC   1

#else
#define BLEAUDIO_NUM_NOT_PER_FRAME_ADPCM  5
#define BLEAUDIO_NUM_NOT_PER_FRAME_MSBC   3
#endif

#define ADPCM_SAMPLES_PER_FRAME   (BLEAUDIO_BUFSIZE_ADPCM * 2)
#define MSBC_SAMPLES_PER_FRAME    120
#define MSBC_ENCODED_SIZE          57
typedef enum {
  STREAM_STATE_IDLE,
  STREAM_STATE_SEND_START_CMD,
  STREAM_STATE_START_I2S,
  STREAM_STATE_ACTIVE,
  STREAM_STATE_SEND_STOP_CMD,
  STREAM_STATE_STOP_I2S,
} STREAM_STATE_E;
const unsigned char msbc_data[] =
{
	0xad, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x01, 0x12,
	0xe1, 0xeb, 0x31, 0x60, 0x76, 0xcd, 0x61, 0xf3,
	0x40, 0xe5, 0x09, 0x38, 0xc4, 0xba, 0xa3, 0xa2,
	0x38, 0x7b, 0x09, 0xb8, 0x1d, 0xdf, 0x30, 0x7c,
	0xd1, 0xa2, 0x42, 0x4b, 0xe5, 0xae, 0xa9, 0x15,
	0x9e, 0x1e, 0xc1, 0x62, 0x07, 0x6e, 0xb5, 0x1f,
	0x33, 0x56, 0x90, 0x92, 0xf9, 0x7b, 0xaa, 0x35,
	0xe0
};
//int16_t pcmSamples[MSBC_SAMPLES_PER_FRAME * I2SCC26XX_QUEUE_SIZE] = {0};
int16_t *pcmSamples;

uint8_t i2sContMgtBuffer[I2S_BLOCK_OVERHEAD_IN_BYTES * I2SCC26XX_QUEUE_SIZE] = {0};
uint8_t audio_encoded[100] = {0};
sbc_t sbc = {0};
ssize_t written = 0;
struct {
  STREAM_STATE_E streamState;
  STREAM_STATE_E requestedStreamState;
  uint8_t streamType;
  uint8_t requestedStreamType;
  uint8_t samplesPerFrame;
  uint8_t notificationsPerFrame;
  int8_t si;
  int16_t pv;
  uint8_t activeLED;
} streamVariables = {STREAM_STATE_IDLE, STREAM_STATE_IDLE, 0, 0, 0, 0, 0, 0, 0};

static void I2SCC26XX_i2sCallbackFxn(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *notification);
static I2SCC26XX_Handle i2sHandle = NULL;
static I2SCC26XX_StreamNotification i2sStream;
static I2SCC26XX_Params i2sParams = {
  .requestMode            = I2SCC26XX_CALLBACK_MODE,
  .ui32requestTimeout     = BIOS_WAIT_FOREVER,
  .callbackFxn            = I2SCC26XX_i2sCallbackFxn,
  .blockSize              = MSBC_SAMPLES_PER_FRAME,
  .pvContBuffer           = NULL,
  .ui32conBufTotalSize    = 0,
  .pvContMgtBuffer        = (void *) i2sContMgtBuffer,
  .ui32conMgtBufTotalSize = sizeof(i2sContMgtBuffer),
  .currentStream          = &i2sStream
};
static bool i2sStreamInProgress = false;
static uint8_t volume = 0, seqNum = 0;

#include <ti/mw/display/DisplayUart.h>
UART_Handle uartHandle;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimpleBLEPeripheral_clockHandler(UArg arg);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);

static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

void SimpleBLEPeripheral_keyChangeHandler(uint8 keys);
static void SimpleBLEPeripheral_handleKeys(uint8_t shift, uint8_t keys);

static uint8_t SimpleBLEPeripheral_transmitAudioStreamCmd(uint8_t cmd);
static void SimpleBLEPeripheral_startStreaming(void);
//static void SimpleBLEPeripheral_processPdmData(void);
static void SimpleBLEPeripheral_transmitAudioFrame(uint8_t *buf);
static void SimpleBLEPeripheral_stopStreaming(void);
static void SimpleBLEPeripheral_sendStopCmd(void);
static void SimpleBLEPeripheral_sendStartCmd(void);
static void SimpleBLEPeripheral_startI2Sstream(void);
static void SimpleBLEPeripheral_stopI2Sstream(void);
static void SimpleBLEPeripheral_finishStream(void);

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
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
  SimpleBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

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
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the DB Address till CC2650 board gets its own IEEE address
  uint8 bdAddress[B_ADDR_LEN] = { 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };
  HCI_EXT_SetBDADDRCmd(bdAddress);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

#if defined (DLE_ENABLED)
  HCI_LE_WriteSuggestedDefaultDataLenCmd(DLE_MAX_PDU_SIZE , DLE_MAX_TX_TIME);
#endif

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, SimpleBLEPeripheral_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);

  dispHandle = Display_open(Display_Type_LCD, NULL);
  if(dispHandle == NULL)
  {
    dispHandle = Display_open(Display_Type_UART, NULL);

    //Send the form feed char to the LCD, this is helpful if using a terminal
    //as it will clear the terminal history
    Display_print0(dispHandle, 0, 0, "\f");
  }

  // Highjack UART handle
  DisplayUart_Object  *object  = (DisplayUart_Object  *)dispHandle->object;
  uartHandle = object->hUart;

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

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE

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

#if defined FEATURE_OAD
	#if defined (HAL_IMAGE_A)
	  Display_print0(dispHandle, 0, 0, "Audio Tx Peripheral A");
	#else
	  Display_print0(dispHandle, 0, 0, "Audio Tx Peripheral B");
	#endif // HAL_IMAGE_A
#else
	#if defined (DLE_ENABLED)
	  Display_print0(dispHandle, 0, 0, "Audio Tx Peripheral with DLE");
	#else
	  Display_print0(dispHandle, 0, 0, "Audio Tx Peripheral");
	#endif
#endif // FEATURE_OAD

  // Open pin structure for use
  hSbpPins = PIN_open(&sbpPins, SBP_configTable);

  Board_initKeys(SimpleBLEPeripheral_keyChangeHandler);

  /* Then initialize I2S driver */
  i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
  I2SCC26XX_init(i2sHandle);

  // Initialize TLV320AIC3254 Codec on Audio BP
  AudioCodecOpen();
  // Configure Codec
  AudioCodecConfig(AUDIO_CODEC_TI_3254, AUDIO_CODEC_16_BIT, 16000, 2, 0, INPUT_OPTION);
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
  uint32_t hwiKey;
  // Initialize application
  SimpleBLEPeripheral_init();

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
    }

    // If RTOS queue is not empty, process app message.
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

    if (events & SBP_PERIODIC_EVT)
    {
      events &= ~SBP_PERIODIC_EVT;

      PIN_setOutputValue( hSbpPins, Board_DIO27_ANALOG, !(PIN_getOutputValue(Board_DIO27_ANALOG)));

      // Kick off next round, if we're still streaming
      if (streamVariables.streamState == STREAM_STATE_ACTIVE) {
//        static uint8_t period = SBP_PERIODIC_EVT_PERIOD;
//        if (period == SBP_PERIODIC_EVT_PERIOD) {
//          period = SBP_PERIODIC_EVT_PERIOD - 1;
//        } else {
//          period = SBP_PERIODIC_EVT_PERIOD;
//        }
//        Util_rescheduleClock(&periodicClock, period); //7-8ms delay (try to get 7.5ms on average)
        Util_rescheduleClock(&periodicClock, SBP_PERIODIC_EVT_PERIOD); //8ms delay
        Util_startClock(&periodicClock);
      }
    }

    if (events & SBP_I2S_FRAME_EVENT)
    {
      events &= ~SBP_I2S_FRAME_EVENT;
      if (i2sStreamInProgress) {
        I2SCC26XX_BufferRequest bufferRequest;
        I2SCC26XX_BufferRelease bufferRelease;
        bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
        while (gotBuffer) {
          PIN_setOutputValue( hSbpPins, Board_DIO26_ANALOG, 0);
          // Flush on UART
//          UART_write(uartHandle, bufferRequest.bufferIn, streamVariables.samplesPerFrame * sizeof(int16_t));
          if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC) {
            sbc_encode(&sbc, (int16_t *)bufferRequest.bufferIn, streamVariables.samplesPerFrame * sizeof(int16_t), audio_encoded, MSBC_ENCODED_SIZE, &written);
            audio_encoded[1] = seqNum++;
          }
          else {
          audio_encoded[0] = (((seqNum++ % 32) << 3) | RAS_DATA_TIC1_CMD);
            // Send previous PV and SI
            audio_encoded[1] = streamVariables.si;
            audio_encoded[2] = LO_UINT16(streamVariables.pv);
            audio_encoded[3] = HI_UINT16(streamVariables.pv);
            Codec1_encodeBuff((uint8_t *)&audio_encoded[4], (int16_t *)bufferRequest.bufferIn, streamVariables.samplesPerFrame, &streamVariables.si, &streamVariables.pv);
          }
          SimpleBLEPeripheral_transmitAudioFrame(audio_encoded);
          PIN_setOutputValue( hSbpPins, Board_DIO26_ANALOG, 1);
          bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
          bufferRelease.bufferHandleOut = NULL;
          I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
          if (volume <= 70) {
            if ((volume & 0x0F) == 0x04) {
              // Volume control
              AudioCodecMicVolCtrl(AUDIO_CODEC_TI_3254, INPUT_OPTION, volume);
            }
            volume++;
          }
          gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
        }
      }
    }

    if (events & SBP_I2S_ERROR_EVENT)
    {
      events &= ~SBP_I2S_ERROR_EVENT;
      PIN_setOutputValue( hSbpPins, Board_DIO27_ANALOG, 0);

      // Move to stop state
      hwiKey = Hwi_disable();
      streamVariables.streamState = STREAM_STATE_SEND_STOP_CMD;
      events |= SBP_SEND_STOP_CMD_EVENT;
      Hwi_restore(hwiKey);
      PIN_setOutputValue( hSbpPins, Board_DIO27_ANALOG, 1);
    }

    if (events & SBP_SEND_STOP_CMD_EVENT)
    {
      events &= ~SBP_SEND_STOP_CMD_EVENT;
      SimpleBLEPeripheral_sendStopCmd();
    }

    if (events & SBP_STOP_I2S_EVENT)
    {
      events &= ~SBP_STOP_I2S_EVENT;
      SimpleBLEPeripheral_stopI2Sstream();
    }

    if (events & SBP_SEND_START_CMD_EVENT)
    {
      events &= ~SBP_SEND_START_CMD_EVENT;
      SimpleBLEPeripheral_sendStartCmd();
    }

    if (events & SBP_START_I2S_EVENT)
    {
      events &= ~SBP_START_I2S_EVENT;
      SimpleBLEPeripheral_startI2Sstream();
    }

#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
      oadTargetWrite_t *oadWriteEvt = Queue_dequeue(hOadQ);

      // Identify new image.
      if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }
      // Write a next block request.
      else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }

      // Free buffer.
      ICall_free(oadWriteEvt);
    }
#endif //FEATURE_OAD
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
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_CONN_EVT_END_EVT) == SUCCESS)
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
    Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_print1(dispHandle, 5, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
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
      SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case SBP_KEY_CHANGE_EVT:
      SimpleBLEPeripheral_handleKeys(0, pMsg->hdr.state);
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
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

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

#ifdef PLUS_BROADCASTER
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of
     * state to the application.  These are then disabled here so that sending
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        Display_print0(dispHandle, 2, 0, "Connected");
        Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Display_print0(dispHandle, 2, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, 2, 0, "Disconnected");

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, 2, 0, "Timed Out");

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
      Display_print0(dispHandle, 2, 0, "Error");
      break;

    default:
      Display_clearLine(dispHandle, 2);
      break;
  }

  // Update the state
  //gapProfileState = newState;
}


#ifdef FEATURE_OAD
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_enqueue(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
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
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
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

  // Check for both keys first
  if (keys == (KEY_LEFT | KEY_RIGHT))
  {
    if (streamVariables.streamState != STREAM_STATE_IDLE)
    {
      streamVariables.requestedStreamState = STREAM_STATE_IDLE;
      streamVariables.requestedStreamType = BLE_AUDIO_CMD_STOP;
      // Start chain of events to stop stream
      SimpleBLEPeripheral_stopStreaming();
    }
  }
  else if (keys & KEY_LEFT)
  {
    if (streamVariables.streamState == STREAM_STATE_IDLE) {
      // Start MSBC stream, from IDLE
      streamVariables.streamType = BLE_AUDIO_CMD_START_MSBC;
      streamVariables.samplesPerFrame = MSBC_SAMPLES_PER_FRAME;
      streamVariables.notificationsPerFrame = BLEAUDIO_NUM_NOT_PER_FRAME_MSBC;
      streamVariables.activeLED = Board_LED2;
      streamVariables.requestedStreamType = BLE_AUDIO_CMD_NONE;
      streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
      SimpleBLEPeripheral_startStreaming();
    }
    else if (streamVariables.streamType == BLE_AUDIO_CMD_START) {
      // Change stream to mSBC, from ADPCM
      streamVariables.requestedStreamType = BLE_AUDIO_CMD_START_MSBC;
      streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
      // Start chain of events to stop stream
      SimpleBLEPeripheral_stopStreaming();
    }
  }
  else if (keys & KEY_RIGHT)
  {
    if (streamVariables.streamState == STREAM_STATE_IDLE) {
      // Start ADPCM stream, from IDLE
      streamVariables.streamType = BLE_AUDIO_CMD_START;
      streamVariables.samplesPerFrame = ADPCM_SAMPLES_PER_FRAME;
      streamVariables.notificationsPerFrame = BLEAUDIO_NUM_NOT_PER_FRAME_ADPCM;
      streamVariables.activeLED = Board_LED1;
      streamVariables.requestedStreamType = BLE_AUDIO_CMD_NONE;
      streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
      SimpleBLEPeripheral_startStreaming();
    }
    else if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC) {
      // Change stream to ADPCM, from mSBC
      streamVariables.requestedStreamType = BLE_AUDIO_CMD_START;
      streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
      // Start chain of events to stop stream
      SimpleBLEPeripheral_stopStreaming();
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_transmitAudioStreamCmd
 *
 * @brief   Transmits GATT Notification in order to start or stop stream
 *
 * @param   cmd - command to transmit
 *
 * @return  SUCCESS if successful, FAILURE if not
 */
static uint8_t SimpleBLEPeripheral_transmitAudioStreamCmd(uint8_t cmd)
{
  return Audio_SetParameter(AUDIOPROFILE_START, AUDIOPROFILE_CMD_LEN, &cmd);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_transmitAudioFrame
 *
 * @brief   Transmits processed audio frame to connected device
 *
 * @param   buf - pointer to PDM buffer
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_transmitAudioFrame(uint8_t *buf)
{
  PIN_setOutputValue( hSbpPins, Board_DIO28_ANALOG, 0);
  // Send 3 GATT notifications for every audio frame
  for (int i = 0; i < streamVariables.notificationsPerFrame; )
  {
    if (Audio_SetParameter(AUDIOPROFILE_AUDIO, BLEAUDIO_NOTSIZE, buf) == SUCCESS)
    {
      PIN_setOutputValue( hSbpPins, Board_DIO28_ANALOG, !(PIN_getOutputValue(Board_DIO28_ANALOG)));
      // Move on to next section of audio frame
      buf += BLEAUDIO_NOTSIZE;
      i++;
      PIN_setOutputValue(hSbpPins, streamVariables.activeLED, 1);
    }
    else
    {
      PIN_setOutputValue(hSbpPins, streamVariables.activeLED, 0);
    }
  }
  PIN_setOutputValue( hSbpPins, Board_DIO28_ANALOG, 1);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_startStreaming
 *
 * @brief   Starts streaming audio to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_startStreaming(void)
{
  uint32_t hwiKey;
  // LED on while streaming
  PIN_setOutputValue(hSbpPins, streamVariables.activeLED, 1);

  // Increase TX power during stream
  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);

  // Allocate memory for decoded PCM data
  pcmSamples = ICall_malloc(sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE));
  i2sParams.blockSize              = streamVariables.samplesPerFrame;
  i2sParams.pvContBuffer           = (void *) pcmSamples;
  i2sParams.ui32conBufTotalSize    = sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE);
  I2SCC26XX_Handle i2sHandleTmp = I2SCC26XX_open(i2sHandle, &i2sParams);

  Display_print1(dispHandle, 5, 0, "Opened I2S: %d samples/frame", streamVariables.samplesPerFrame);

  if (i2sHandleTmp == i2sHandle) {
    // Move to send start command
    hwiKey = Hwi_disable();
    streamVariables.streamState = STREAM_STATE_SEND_START_CMD;
    events |= SBP_SEND_START_CMD_EVENT;
    Hwi_restore(hwiKey);
  }
  else {
    // Return, or move to IDLE state
    streamVariables.streamState = STREAM_STATE_IDLE;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stopStreaming
 *
 * @brief   Stops streaming audio to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stopStreaming(void)
{
  uint32_t hwiKey;
  PIN_setOutputValue( hSbpPins, Board_DIO27_ANALOG, 0);
  // Check if we're at the right state in the stopping process
  if (streamVariables.streamState == STREAM_STATE_ACTIVE) {
    // Start by sending STOP command
    streamVariables.streamState = STREAM_STATE_SEND_STOP_CMD;
    hwiKey = Hwi_disable();
    events |= SBP_SEND_STOP_CMD_EVENT;
    Hwi_restore(hwiKey);
    Semaphore_post(sem);
  }
  PIN_setOutputValue( hSbpPins, Board_DIO27_ANALOG, 1);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_startI2Sstream
 *
 * @brief   Start I2S stream
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_startI2Sstream(void)
{
  // Check that we're in the correct state
  if (streamVariables.streamState == STREAM_STATE_START_I2S) {
    if (streamVariables.requestedStreamState == STREAM_STATE_ACTIVE) {
      // Try to start I2S stream
      i2sStreamInProgress = I2SCC26XX_startStream(i2sHandle);
      if (i2sStreamInProgress) {
        // Move to ACTIVE as we have completed start sequence
        streamVariables.streamState = STREAM_STATE_ACTIVE;

//        Util_rescheduleClock(&periodicClock, SBP_PERIODIC_EVT_PERIOD); //8ms delay
//        Util_startClock(&periodicClock);

        if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC) {
          Display_print0(dispHandle, 5, 0, "mSBC Stream");
        }
        else if (streamVariables.streamType == BLE_AUDIO_CMD_START) {
          Display_print0(dispHandle, 5, 0, "ADPCM Stream");
        }
      }
      else {
        Display_print0(dispHandle, 5, 0, "Failed to start I2S stream");
      }
    }
    else {
      Display_print0(dispHandle, 5, 0, "Started stream when Active was not requested");
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stopI2Sstream
 *
 * @brief   Stop I2S stream
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stopI2Sstream(void)
{
  // Check that we're in the correct state
  if (streamVariables.streamState == STREAM_STATE_STOP_I2S) {
    // Try to stop I2S stream
    if (I2SCC26XX_stopStream(i2sHandle)) {
      SimpleBLEPeripheral_finishStream();
      if (streamVariables.requestedStreamState == STREAM_STATE_IDLE) {
        // Simply move to IDLE as we have completed stop sequence
        streamVariables.streamState = STREAM_STATE_IDLE;
        Display_print0(dispHandle, 5, 0, "No Stream");
      }
      else if (streamVariables.requestedStreamType == BLE_AUDIO_CMD_START_MSBC) {
        // Start chain of events to start stream again
        streamVariables.streamType = BLE_AUDIO_CMD_START_MSBC;
        streamVariables.samplesPerFrame = MSBC_SAMPLES_PER_FRAME;
        streamVariables.notificationsPerFrame = BLEAUDIO_NUM_NOT_PER_FRAME_MSBC;
        streamVariables.activeLED = Board_LED2;
        streamVariables.requestedStreamType = BLE_AUDIO_CMD_NONE;
        streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
        SimpleBLEPeripheral_startStreaming();
      }
      else if (streamVariables.requestedStreamType == BLE_AUDIO_CMD_START) {
        streamVariables.streamType = BLE_AUDIO_CMD_START;
        streamVariables.samplesPerFrame = ADPCM_SAMPLES_PER_FRAME;
        streamVariables.notificationsPerFrame = BLEAUDIO_NUM_NOT_PER_FRAME_ADPCM;
        streamVariables.activeLED = Board_LED1;
        streamVariables.requestedStreamType = BLE_AUDIO_CMD_NONE;
        streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
        SimpleBLEPeripheral_startStreaming();
      }
      else {
        Display_print2(dispHandle, 5, 0, "Incorrect state %d.%d",
                       streamVariables.requestedStreamState, streamVariables.requestedStreamType);
      }
    }
    else {
      Display_print0(dispHandle, 5, 0, "Failed to stop I2S stream");
    }
  }
  else {
    Display_print1(dispHandle, 5, 0, "Tried to stop I2S stream in state %d", streamVariables.streamState);
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendStartCmd
 *
 * @brief   Sends a start command to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_sendStartCmd(void)
{
  uint32_t hwiKey;
  // Check that we're in the correct state
  if (streamVariables.streamState == STREAM_STATE_SEND_START_CMD) {
    if (SimpleBLEPeripheral_transmitAudioStreamCmd(streamVariables.streamType) == SUCCESS)
    {
      PIN_setOutputValue( hSbpPins, Board_DIO27_ANALOG, 0);
      if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC) {
        // Initialize encoder
        sbc_init_msbc(&sbc, 0);
      }
      else {
        // Initialize encoder
        streamVariables.pv = 0;
        streamVariables.si = 0;
      }
      // Try next state
      streamVariables.streamState = STREAM_STATE_START_I2S;
      hwiKey = Hwi_disable();
      events |= SBP_START_I2S_EVENT;
      Hwi_restore(hwiKey);
      Semaphore_post(sem);
      PIN_setOutputValue( hSbpPins, Board_DIO27_ANALOG, 1);
    }
    else {
      // Try again
      hwiKey = Hwi_disable();
      events |= SBP_SEND_START_CMD_EVENT;
      Hwi_restore(hwiKey);
      Semaphore_post(sem);
    }
  }
  else {
    // Try next state
    hwiKey = Hwi_disable();
    events |= SBP_START_I2S_EVENT;
    Hwi_restore(hwiKey);
    Semaphore_post(sem);
  }

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendStopCmd
 *
 * @brief   Sends a stop command to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_sendStopCmd(void)
{
  uint32_t hwiKey;

  // Check that we're in the correct state
  if (streamVariables.streamState == STREAM_STATE_SEND_STOP_CMD) {
    uint8_t gapRoleState;
    uint8_t retVal = SUCCESS;
    GAPRole_GetParameter(GAPROLE_STATE, &gapRoleState);

    if (gapRoleState == GAPROLE_CONNECTED)
    {
      retVal = SimpleBLEPeripheral_transmitAudioStreamCmd(BLE_AUDIO_CMD_STOP);
      if (retVal == SUCCESS)
      {
        // Move to stop I2S stream
        hwiKey = Hwi_disable();
        // Reset TX power
        HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
        streamVariables.streamState = STREAM_STATE_STOP_I2S;
        events |= SBP_STOP_I2S_EVENT;
        Hwi_restore(hwiKey);
      }
      else
      {
        Display_print1(dispHandle, 5, 0, "Failed to send STOP: %d", retVal);
        // Try again
        hwiKey = Hwi_disable();
        events |= SBP_SEND_STOP_CMD_EVENT;
        Hwi_restore(hwiKey);
        Semaphore_post(sem);
      }
    }
    else
    {
      // Move to stop I2S stream
      hwiKey = Hwi_disable();
      // Reset TX power
      HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
      streamVariables.streamState = STREAM_STATE_STOP_I2S;
      events |= SBP_STOP_I2S_EVENT;
      Hwi_restore(hwiKey);
    }
  }
  else {
    // Try next state
    hwiKey = Hwi_disable();
    events |= SBP_STOP_I2S_EVENT;
    Hwi_restore(hwiKey);
    Semaphore_post(sem);
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_finishStream
 *
 * @brief   Finish stream
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_finishStream(void) {
  // LED off
  PIN_setOutputValue(hSbpPins, streamVariables.activeLED, 0);
  /* Turn output volume back down */
  volume = 0;
  AudioCodecMicVolCtrl(AUDIO_CODEC_TI_3254, INPUT_OPTION, volume);

  // Stop potentially pending events
  events &= ~(SBP_I2S_FRAME_EVENT | SBP_I2S_ERROR_EVENT);

  if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC)
  {
    sbc_finish(&sbc);
  }
  /* Close I2S driver */
  I2SCC26XX_close(i2sHandle);
  /* Free memory */
  ICall_free(pcmSamples);
  pcmSamples = NULL;
}

static void I2SCC26XX_i2sCallbackFxn(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *notification) {
  if (notification->status == I2SCC26XX_STREAM_ERROR) {
    /* Let thread process PDM error */
    events |= SBP_I2S_ERROR_EVENT;
    Semaphore_post(sem);
  }
  else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY) {
    // Provide buffer
    events |= SBP_I2S_FRAME_EVENT;
    Semaphore_post(sem);
    PIN_setOutputValue( hSbpPins, Board_DIO25_ANALOG, !(PIN_getOutputValue(Board_DIO25_ANALOG)));
    PIN_setOutputValue( hSbpPins, (streamVariables.activeLED == Board_LED0) ? Board_LED1 : Board_LED0, 0);
  }
  else {
    PIN_setOutputValue( hSbpPins, (streamVariables.activeLED == Board_LED0) ? Board_LED1 : Board_LED0, 1);
  }
}

/*********************************************************************
*********************************************************************/
