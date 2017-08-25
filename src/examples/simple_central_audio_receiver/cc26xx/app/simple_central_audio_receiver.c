/*
 * Filename: simple_central_audio_receiver.c
 *
 * Description: This is the simple_central example modified to receive
 * data over BLE at a high throughput.
 *
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
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

#include "bcomdef.h"

#include "hci_tl.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simple_gatt_profile.h"
#include "gatt_profile_uuid.h"
#include "gatt.h"
#include "gatt_uuid.h"

#include "OSAL.h"
#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include "board_key.h"
#include <ti/mw/display/Display.h>
#include "board.h"

#include "simple_central.h"

#include "ble_user_config.h"

#include "audio_profile.h"

#ifdef STREAM_TO_AUDBOOST
#include "audiocodec.h"
#include "I2SCC26XX.h"
#endif //STREAM_TO_AUDBOOST
#include "msbc_library.h"

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include <ti/drivers/pdm/Codec1.h>

/*********************************************************************
 * MACROS
 */
uint8 audioConfigEnable = 0;

/*********************************************************************
 * CONSTANTS
 */

#if !defined(BOARD_DISPLAY_EXCLUDE_UART)
  #ifdef STREAM_TO_PC
    #warning Cannot stream to PC and log over UART
    #undef STREAM_TO_PC
  #endif
#endif

// Simple BLE Central Task Events
#define SBC_START_DISCOVERY_EVT               0x0001
#define SBC_PAIRING_STATE_EVT                 0x0002
#define SBC_PASSCODE_NEEDED_EVT               0x0004
#define SBC_RSSI_READ_EVT                     0x0008
#define SBC_KEY_CHANGE_EVT                    0x0010
#define SBC_STATE_CHANGE_EVT                  0x0020
#define SBC_SCANNING_TOGGLE_EVT               0x0040
#define SBC_AUDIO_FORCESTOP_EVT               0x0080
#define SBC_AUDIO_VOLUMEDOWN_EVT              0x0100


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

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      8

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Change it to initialize pairing from central since sensortag audio project does not initial pairing
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000
// Default service discovery timer delay in ms
#define DEFAULT_SCANNING_TOGGLECLOCK          200

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                   864
#endif

#define APP_MIN_CONN_INTERVAL                 8      // 10ms, need short connection interval in order to reach the needed throughput for audio streaming
#define APP_MAX_CONN_INTERVAL                 8

#define APP_SLAVE_LATENCY                     0      // Initially 0 for fast connection. 49 slave latency (500ms effective interval)
#define APP_CONN_TIMEOUT                      200    // 2s supervision timeout

// Gap Bond Manager States
#define UNPAIRED_STATE                        0x00
#define PAIRED_BONDED_STATE                   0x01

#define TI_COMPANY_ID                         0x000D  // To maintain connectivity with SensorTag audio project

// Simple Profile Service UUID
#define SIMPLEPROFILE_SERV_UUID               0xFFF0

// Service Change flags
#define NO_CHANGE                             0x00
#define CHANGE_OCCURED                        0x01

// Copied from hid_adv_remote.c
#define BLE_AUDIO_CMD_STOP                    0x00
#define BLE_AUDIO_CMD_START                   0x04
#define BLE_AUDIO_CMD_START_MSBC              0x05

#define BLE_AUDIO_STREAM_NONE                 0x00
#define BLE_AUDIO_STREAM_ADPCM                0x01
#define BLE_AUDIO_STREAM_MSBC                 0x02

// 4:1 ADPCM compression is used, wrt to int16_t buffer, this is 2x
#define BLE_AUDIO_U16_COMPRESSION_RATE        2

#define BLE_AUDIO_RX_BUF_SIZE                 BLE_AUDIO_U16_COMPRESSION_RATE*BLEAUDIO_NOTSIZE

#define DLE_MAX_PDU_SIZE 251
#define DLE_MAX_TX_TIME 2120

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} sbcEvt_t;

// RSSI read data structure
typedef struct
{
  uint16_t period;      // how often to read RSSI
  uint16_t connHandle;  // connection handle
  Clock_Struct *pClock; // pointer to clock struct
} readRssi_t;

typedef struct
{
  uint8_t       charProps;
  uint8_t       charValueHandleLow;
  uint8_t       charValueHandleHigh;
  uint8_t       charValueUUIDLow;
  uint8_t       charValueUUIDHigh;
}attPrimaryServiceValue_t;

typedef struct
{
  // Service and Characteristic discovery variables.
  uint16 keyCharHandle;
  uint16 keyCCCHandle;
  uint16 audioStartCharValueHandle;
  uint16 audioDataCharValueHandle;
  uint8  lastRemoteAddr[B_ADDR_LEN];
} SimpleBLECentral_HandleInfo_t;


/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

#ifdef STREAM_TO_AUDBOOST
  static uint8_t volume = 0;
#endif
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

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;
// Clock object used to signal start/end scanning
static Clock_Struct scanningToggleClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task pending events
static uint16_t events = 0;

// Task configuration
Task_Struct sbcTask;
Char sbcTaskStack[SBC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8_t scanRes;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static bool scanningStarted = FALSE;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Maximum PDU size (default = 27 octets)
static uint16 maxPduSize;

// Array of RSSI read structures
static readRssi_t readRssi[MAX_NUM_BLE_CONNS];

/* Pin driver handles */
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State allPinState;

static uint8 remoteAddr[B_ADDR_LEN] = {0,0,0,0,0,0};

// Handle info saved here after connection to skip service discovery.
static SimpleBLECentral_HandleInfo_t remoteHandles;

static uint16 serviceToDiscover = GATT_INVALID_HANDLE;

static uint8 enableCCCDs = TRUE;

// Service and Characteristic discovery variables.

static uint16 keyCharHandle          = GATT_INVALID_HANDLE;

/* Audio START characteristic */
static uint16 audioStartCharValueHandle   = GATT_INVALID_HANDLE;
static uint16 audioStartCCCHandle         = GATT_INVALID_HANDLE;
/* Audio "Data" characteristic */
static uint16 audioDataCharValueHandle    = GATT_INVALID_HANDLE;
static uint16 audioDataCCCHandle         = GATT_INVALID_HANDLE;

static uint8 serviceDiscComplete = FALSE;

/*********************************************************************
 * LOCAL VARIABLES
 */
static void SimpleBLECentral_init(void);
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1);

static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg);
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_startDiscovery(void);
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen);
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status);
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);

static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);

static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs);
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status);

void SimpleBLECentral_startDiscHandler(UArg a0);
void SimpleBLECentral_keyChangeHandler(uint8 keys);
void SimpleBLECentral_readRssiHandler(UArg a0);

static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t status,
                                           uint8_t *pData);

static uint8 SimpleBLECentral_FindHIDRemote( uint8* pData, uint8 length );
static void SimpleBLECentral_SetIdle( void );
static uint8 SimpleBLECentral_BondCount( void );
static void SimpleBLECentral_EstablishLink( uint8 whiteList, uint8 addrType, uint8 *remoteAddr );
static void SimpleBLECentral_EnableNotification( uint16 connHandle, uint16 attrHandle );
static void SimpleBLECentral_DiscoverService( uint16 connHandle, uint16 svcUuid );
static void SimpleBLECentral_SaveHandles( void );

//static void SimpleBLECentral_processAudioPkt(uint8_t pkt_counter, uint8_t * pInBuf);

static void SimpleBLECentral_scanningToggleHandler(UArg a0);

PIN_Config ledPinTable[] = {
    Board_RLED   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,         /* LED initially off             */
    Board_GLED   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,         /* LED initially off             */
    Board_DIO25_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,  /* Debug IO initially high       */
    Board_DIO26_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,  /* Debug IO initially high       */
    Board_DIO27_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,  /* Debug IO initially high       */
    Board_DIO28_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,  /* Debug IO initially high       */
  PIN_TERMINATE
};

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
int16_t *audio_decoded;
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

#ifdef STREAM_TO_AUDBOOST
uint8_t i2sContMgtBuffer[I2S_BLOCK_OVERHEAD_IN_BYTES * I2SCC26XX_QUEUE_SIZE] = {0};
#endif //STREAM_TO_AUDBOOST
uint8_t audio_encoded[100] = {0};
sbc_t sbc = {0};
size_t written = 0;
struct {
  uint8_t sourceIsPDM;
  uint8_t streamType;
  uint8_t samplesPerFrame;
  uint8_t notificationsPerFrame;
  int8_t si;
  int16_t pv;
  int8_t maxVolume;
#ifdef STREAM_TO_AUDBOOST
  uint8_t i2sOpened;
#endif //STREAM_TO_AUDBOOST
} streamVariables = {0};

#ifdef STREAM_TO_AUDBOOST
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
#else //STREAM_TO_PC
UART_Handle uartHandle;
#endif //STREAM_TO_AUDBOOST

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t SimpleBLECentral_roleCB =
{
  SimpleBLECentral_eventCB     // Event callback
};

// Bond Manager Callbacks
static gapBondCBs_t SimpleBLECentral_bondCB =
{
  (pfnPasscodeCB_t)SimpleBLECentral_passcodeCB, // Passcode callback
  SimpleBLECentral_pairStateCB                  // Pairing state callback
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
void SimpleBLECentral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbcTaskStack;
  taskParams.stackSize = SBC_TASK_STACK_SIZE;
  taskParams.priority = SBC_TASK_PRIORITY;

  Task_construct(&sbcTask, SimpleBLECentral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_Init
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
static void SimpleBLECentral_init(void)
{
  uint8_t i;

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

#if defined (DLE_ENABLED)
  HCI_LE_WriteSuggestedDefaultDataLenCmd(DLE_MAX_PDU_SIZE , DLE_MAX_TX_TIME);
#endif

  HCI_EXT_SetSCACmd(120);
  // Open all pins
  ledPinHandle = PIN_open(&allPinState, ledPinTable);

  // Turn on Red led to indicate nothing has been connected
  PIN_setOutputValue( ledPinHandle, Board_RLED, 1);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, SimpleBLECentral_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);

  // Periodic event to toggle LED to indicate if scanning is in progress
  Util_constructClock(&scanningToggleClock, SimpleBLECentral_scanningToggleHandler,
                      DEFAULT_SCANNING_TOGGLECLOCK, 0, false, SBC_SCANNING_TOGGLE_EVT);

  Board_initKeys(SimpleBLECentral_keyChangeHandler);

  dispHandle = Display_open(Display_Type_LCD, NULL);
  if(dispHandle == NULL)
  {
    dispHandle = Display_open(Display_Type_UART, NULL);

    //Send the form feed char to the LCD, this is helpful if using a terminal
    //as it will clear the terminal history
    Display_print0(dispHandle, 0, 0, "\f");
  }

  // Initialize internal data
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    readRssi[i].connHandle = GAP_CONNHANDLE_ALL;
    readRssi[i].pClock = NULL;
  }

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

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = DEFAULT_PASSCODE;
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    uint8_t mitm = DEFAULT_MITM_MODE;
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    uint8_t bonding = DEFAULT_BONDING_MODE;
    uint8_t failAction = GAPBOND_FAIL_TERMINATE_ERASE_BONDS;
    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_BOND_FAIL_ACTION,sizeof(uint8_t), &failAction);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Set connection parameters:
  {
    uint16 minconnectionInterval = APP_MIN_CONN_INTERVAL;
    uint16 maxconnectionInterval = APP_MAX_CONN_INTERVAL;

    uint16 slaveLatency = APP_SLAVE_LATENCY;
    uint16 timeout = APP_CONN_TIMEOUT;

    VOID GAP_SetParamValue( TGAP_CONN_EST_INT_MIN, minconnectionInterval );
    VOID GAP_SetParamValue( TGAP_CONN_EST_INT_MAX, maxconnectionInterval );
    VOID GAP_SetParamValue( TGAP_CONN_EST_LATENCY, slaveLatency );
    VOID GAP_SetParamValue( TGAP_CONN_EST_SUPERV_TIMEOUT, timeout );

  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Start the Device
  VOID GAPCentralRole_StartDevice(&SimpleBLECentral_roleCB);

  // Register with bond manager after starting device
  GAPBondMgr_Register(&SimpleBLECentral_bondCB);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);
#if defined (DLE_ENABLED)
  Display_print0(dispHandle, 0, 0, "Audio Central with DLE");
#else
  Display_print0(dispHandle, 0, 0, "Audio Central");
#endif

#ifdef STREAM_TO_AUDBOOST
  /* Then initialize I2S driver */
  i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
  I2SCC26XX_init(i2sHandle);

  // Initialize TLV320AIC3254 Codec on Audio BP
  AudioCodecOpen();
  // Configure Codec
  AudioCodecConfig(AUDIO_CODEC_TI_3254, AUDIO_CODEC_16_BIT, 16000, 2, AUDIO_CODEC_SPEAKER_HP, 0);
#else //STREAM_TO_PC
  UART_Params uartParams;
  UART_Params_init(&uartParams);
#ifdef UART_DUMP_UNCOMPRESSED
  uartParams.baudRate  = 460800 * 4;
#else //!UART_DUMP_UNCOMPRESSED
  uartParams.baudRate  = 460800;
#endif //UART_DUMP_UNCOMPRESSED
  uartParams.writeMode = UART_MODE_BLOCKING;
  uartHandle = UART_open(Board_UART, &uartParams);
#endif //STREAM_TO_AUDBOOST
}

/*********************************************************************
 * @fn      SimpleBLECentral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLECentral_init();

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
          SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg);
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
      sbcEvt_t *pMsg = (sbcEvt_t *)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message
        SimpleBLECentral_processAppMsg(pMsg);

        // Free the space from the message
        ICall_free(pMsg);
      }
    }

    if (events & SBC_START_DISCOVERY_EVT)
    {
      events &= ~SBC_START_DISCOVERY_EVT;

      SimpleBLECentral_startDiscovery();
    }

    if (events & SBC_SCANNING_TOGGLE_EVT)
    {
      events &= ~SBC_SCANNING_TOGGLE_EVT;

      Util_startClock(&scanningToggleClock);
      PIN_setOutputValue(ledPinHandle, Board_GLED, !PIN_getOutputValue(Board_GLED));
    }

    if (events & SBC_AUDIO_FORCESTOP_EVT)
    {
      events &= ~SBC_AUDIO_FORCESTOP_EVT;
      PIN_setOutputValue(ledPinHandle, Board_RLED, Board_LED_OFF);
#ifdef STREAM_TO_AUDBOOST
      if (i2sStreamInProgress) {
        I2SCC26XX_stopStream(i2sHandle);
        i2sStreamInProgress = false;
        Display_print0(dispHandle, 5, 0, "Force Shutdown I2S stream");

        /* Turn output volume back down */ //TODO: Turn off codec
        volume = 0;
        AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, AUDIO_CODEC_SPEAKER_HP, volume);

        if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC)
        {
          sbc_finish(&sbc);
        }
      }
      if (streamVariables.i2sOpened == TRUE)
      {
        I2SCC26XX_close(i2sHandle);
        streamVariables.i2sOpened = FALSE;
        Display_print0(dispHandle, 5, 0, "Closed I2S driver");
        if (audio_decoded) {
          ICall_free(audio_decoded);
          audio_decoded = NULL;
          Display_print0(dispHandle, 5, 0, "Free'd memory for I2S driver");
        }
        else {
          asm(" NOP");
          Display_print0(dispHandle, 5, 0, "Failed to free memory for I2S driver");
        }
      }
#else //STREAM_TO_PC
#ifdef UART_DUMP_UNCOMPRESSED
      if (audio_decoded) {
        ICall_free(audio_decoded);
      }
#endif //UART_DUMP_UNCOMPRESSED
      if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC)
      {
        sbc_finish(&sbc);
      }
#endif //STREAM_TO_AUDBOOST
      streamVariables.streamType = BLE_AUDIO_CMD_STOP;
      Display_print0(dispHandle, 5, 0, "Force Shutdown");
    }

    if (events & SBC_AUDIO_VOLUMEDOWN_EVT)
    {
      events &= ~SBC_AUDIO_VOLUMEDOWN_EVT;
      /* Turn output volume back down */ //TODO: Turn off codec
      volume = 0;
      AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, AUDIO_CODEC_SPEAKER_HP, volume);
      Display_print0(dispHandle, 5, 0, "Turn Volume down to 0");
    }


  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleBLECentral_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;

    case GATT_MSG_EVENT:
      SimpleBLECentral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            SimpleBLECentral_processCmdCompleteEvt((hciEvt_CmdComplete_t *)pMsg);
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
 * @fn      SimpleBLECentral_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBC_STATE_CHANGE_EVT:
      SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;

    case SBC_KEY_CHANGE_EVT:
      SimpleBLECentral_handleKeys(0, pMsg->hdr.state);
      break;

    case SBC_RSSI_READ_EVT:
      {
        readRssi_t *pRssi = (readRssi_t *)pMsg->pData;

        // If link is up and RSSI reads active
        if (pRssi->connHandle != GAP_CONNHANDLE_ALL &&
            linkDB_Up(pRssi->connHandle))
        {
          // Restart timer
          Util_restartClock(pRssi->pClock, pRssi->period);

          // Read RSSI
          VOID HCI_ReadRssiCmd(pRssi->connHandle);
        }
      }
      break;

    // Pairing event
    case SBC_PAIRING_STATE_EVT:
      {
        SimpleBLECentral_processPairState(pMsg->hdr.state, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    // Passcode event
    case SBC_PASSCODE_NEEDED_EVT:
      {
        SimpleBLECentral_processPasscode(connHandle, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  static uint8 addrType;
  static uint8 peerDeviceFound = FALSE;

  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        maxPduSize = pEvent->initDone.dataPktLen;

        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
        Display_print0(dispHandle, 2, 0, "Initialized");
        if ( SimpleBLECentral_BondCount() > 0 )
        {
          VOID GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, NULL );
          Display_print0(dispHandle, 5, 0, "Erase Bond info ");

          if (!scanningStarted)
          {
            scanningStarted = TRUE;
            scanRes = 0;

            Display_print0(dispHandle, 2, 0, "Discovering...");
            Display_clearLines(dispHandle, 3, 5);
            PIN_setOutputValue( ledPinHandle, Board_RLED, 0);

            GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                          DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                          DEFAULT_DISCOVERY_WHITE_LIST);

            Util_startClock(&scanningToggleClock);
          }
        }
        else
        {
          // Sit idle till ask to scan
          SimpleBLECentral_SetIdle();
        }
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        {
          if ((SimpleBLECentral_findSvcUuid( TI_COMPANY_ID,
                                            pEvent->deviceInfo.pEvtData,
                                            pEvent->deviceInfo.dataLen)) ||
              (SimpleBLECentral_findSvcUuid( HID_SERV_UUID,
                                            pEvent->deviceInfo.pEvtData,
                                            pEvent->deviceInfo.dataLen)) ||
              (SimpleBLECentral_findSvcUuid( AUDIO_SERV_UUID,
                                            pEvent->deviceInfo.pEvtData,
                                            pEvent->deviceInfo.dataLen)) )
          {
            SimpleBLECentral_addDeviceInfo(pEvent->deviceInfo.addr,
                                           pEvent->deviceInfo.addrType);
            addrType = pEvent->deviceInfo.addrType;
            osal_memcpy( remoteAddr, pEvent->deviceInfo.addr, B_ADDR_LEN );
          }
        }
        if (( pEvent->deviceInfo.eventType == GAP_ADRPT_SCAN_RSP ) &&
              SimpleBLECentral_FindHIDRemote( pEvent->deviceInfo.pEvtData,
                                  pEvent->deviceInfo.dataLen ) )
        {
          peerDeviceFound = TRUE;
          // End device discovery
          VOID GAPCentralRole_CancelDiscovery();
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

        SimpleBLECentral_EstablishLink( FALSE, addrType, remoteAddr );

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
        SimpleBLECentral_SetIdle();
      }
    }
    break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;

          if (FALSE == serviceDiscComplete)
          {
            // Begin Service Discovery of AUDIO Service to find out report handles
            serviceToDiscover = AUDIO_SERV_UUID;
            SimpleBLECentral_DiscoverService( connHandle, serviceToDiscover );
            serviceDiscComplete = FALSE;
            audioConfigEnable =0; //This will re-trig an audio configuration if needed
          }

          Util_startClock(&startDiscClock);

          Display_print0(dispHandle, 2, 0, "Connected");
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));
          PIN_setOutputValue(ledPinHandle, Board_GLED, 1);
          PIN_setOutputValue(ledPinHandle, Board_RLED, 0);
        }
        else if ( SimpleBLECentral_BondCount() > 0 )
        {
          // Re-initiate connection
          SimpleBLECentral_EstablishLink( TRUE, addrType, remoteAddr );
        }
        else
        {
          connHandle = GAP_CONNHANDLE_INIT;

          // Go idle
          SimpleBLECentral_SetIdle();
          Display_print0(dispHandle, 2, 0, "Connect Failed");
          Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->gap.hdr.status);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {

        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;

        Display_print0(dispHandle, 2, 0, "Disconnected");
        Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->linkTerminate.reason);
        Display_clearLine(dispHandle, 4);

        if ( serviceDiscComplete == TRUE )
        {
          // Remember the address of the last connected remote
          osal_memcpy( remoteHandles.lastRemoteAddr, remoteAddr, B_ADDR_LEN );

          // Save handle information
          SimpleBLECentral_SaveHandles();
        }


        // Invalidate service discovery variables.
        serviceDiscComplete    = FALSE;

        serviceToDiscover      = GATT_INVALID_HANDLE;

        audioStartCharValueHandle  = GATT_INVALID_HANDLE;
        audioStartCCCHandle        = GATT_INVALID_HANDLE;
        audioDataCharValueHandle   = GATT_INVALID_HANDLE;
        audioDataCCCHandle         = GATT_INVALID_HANDLE;

        PIN_setOutputValue(ledPinHandle, Board_GLED, 0);
        PIN_setOutputValue(ledPinHandle, Board_RLED, 1);
        enableCCCDs = TRUE;
        if ( SimpleBLECentral_BondCount() > 0 )
        {
          // Re-initiate connection
          SimpleBLECentral_EstablishLink( TRUE, addrType, remoteAddr );
        }
        else
        {
          // Go idle
          SimpleBLECentral_SetIdle();
        }
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        Display_print1(dispHandle, 2, 0, "Param Update: %d", pEvent->linkUpdate.status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_handleKeys
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
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter

  if (keys & KEY_LEFT)
  {
    // Start or stop discovery
    if  (state == BLE_STATE_IDLE )
    {
      if (!scanningStarted)
      {
        scanningStarted = TRUE;
        scanRes = 0;

        Display_print0(dispHandle, 2, 0, "Discovering...");
        Display_clearLines(dispHandle, 3, 5);
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
    if ( ( SimpleBLECentral_BondCount() > 0 ) && ( state != BLE_STATE_CONNECTED ) )
    {
      if ( state == BLE_STATE_CONNECTING )
      {
        state = BLE_STATE_DISCONNECTING;
        VOID GAPCentralRole_TerminateLink( GAP_CONNHANDLE_INIT );
      }

      VOID GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, NULL );
      Display_print0(dispHandle, 5, 0, "Erase Bond info ");

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

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  static uint16_t prevSeqNum = 0;
  static int numberOfPackets = 0, lostPackets = 0, counterToAccountForSamplingFrequencyDifference = 0;
  static uint8_t audio_pkt_counter = 0, frameReady = FALSE, missedFrames = 0;

  if (state == BLE_STATE_CONNECTED)
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      Display_print1(dispHandle, 4, 0, "ATT Rsp dropped %d", pMsg->method);
    };

    switch ( pMsg->method )
    {
    case ATT_HANDLE_VALUE_NOTI:
      // Check to see if notification is from audio data or control char
      if (pMsg->msg.handleValueNoti.handle == audioDataCharValueHandle)
      {
          PIN_setOutputValue( ledPinHandle, Board_DIO28_ANALOG, 0);

          if (streamVariables.streamType == BLE_AUDIO_CMD_START)
          {
              // Copy to encoded buffer
              memcpy(&audio_encoded[audio_pkt_counter * pMsg->msg.handleValueNoti.len], pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len);

              // Increment the packet counter, handle wrap around logic
              audio_pkt_counter++;
              if (audio_pkt_counter == streamVariables.notificationsPerFrame)
              {
                  audio_pkt_counter = 0;
                  frameReady = TRUE;
                  // Check for missing frames
                  numberOfPackets++;
                  counterToAccountForSamplingFrequencyDifference++;
                  missedFrames = 0;
                  uint8_t curSeqNum = audio_encoded[0] >> 3;
                  if (((prevSeqNum + 1) & 0x001F) != curSeqNum) {
                      if (curSeqNum > prevSeqNum) {
                          missedFrames = curSeqNum - prevSeqNum;
                      } else {
                          missedFrames = (curSeqNum + 31) - prevSeqNum;
                      }
                      numberOfPackets += missedFrames;
                      counterToAccountForSamplingFrequencyDifference += missedFrames;
                      lostPackets += missedFrames;
                      Display_print2(dispHandle, 5, 0, "Missing frame, PER %d/%d", lostPackets, numberOfPackets);
                      // Don't account for too many missed frames, limit to three
                      missedFrames &= 0x03;
                  }
                  prevSeqNum = curSeqNum;
                  // Account for sampling frequency mismatch, in case source is PDM from CC26xx Remote (sampled at ~15.97kHz
                  if ((streamVariables.sourceIsPDM == TRUE) && (counterToAccountForSamplingFrequencyDifference >= 375)) {
                      counterToAccountForSamplingFrequencyDifference -= 375;
                      missedFrames += 1;
                      Display_print1(dispHandle, 5, 0, "Accounting for sampling frequency mismatch, frame %d", numberOfPackets);
                  }
              }
          }
          else if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC)
          {
              if ((pMsg->msg.handleValueNoti.pValue[0] == 0xAD) &&
                  (pMsg->msg.handleValueNoti.pValue[2] == 0x00))
              {
                  audio_pkt_counter = 0;
              }
              // Output the receive packets through UART, and use audio_frame_serial_print.py to decode
              if (audio_pkt_counter == 2) {
                  memcpy(&audio_encoded[audio_pkt_counter * pMsg->msg.handleValueNoti.len], pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len - 3);
                  frameReady = TRUE;
              }
              else
              {
                  memcpy(&audio_encoded[audio_pkt_counter * pMsg->msg.handleValueNoti.len], pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len);
              }
              audio_pkt_counter++;
          }
          if (frameReady == TRUE) {
#ifdef STREAM_TO_AUDBOOST
            if (i2sStreamInProgress) {
              I2SCC26XX_BufferRequest bufferRequest;
              void *bufferCopy;
              I2SCC26XX_BufferRelease bufferRelease;
              bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
              if (gotBuffer) {
                PIN_setOutputValue( ledPinHandle, Board_DIO26_ANALOG, 0);
                if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC) {
                  sbc_decode(&sbc, audio_encoded, MSBC_ENCODED_SIZE, (int16_t *)bufferRequest.bufferOut, streamVariables.samplesPerFrame * sizeof(int16_t), &written);
                }
                else {
                  streamVariables.pv = BUILD_UINT16(audio_encoded[2], audio_encoded[3]);
                  streamVariables.si = audio_encoded[1];
                  Codec1_decodeBuff((int16_t *)bufferRequest.bufferOut, (uint8_t *)&audio_encoded[4], streamVariables.samplesPerFrame * sizeof(int16_t), &streamVariables.si, &streamVariables.pv);
                }
                PIN_setOutputValue( ledPinHandle, Board_DIO26_ANALOG, 1);
              }
              // Repeat to make up for missing frames
              while (missedFrames) {
                // Get new buffer to copy to before releasing
                bufferRelease.bufferHandleIn = NULL;
                bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
                bufferCopy = bufferRequest.bufferOut;
                // Now get new buffer
                gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
                if (gotBuffer) {
                  missedFrames--;
                  // Copy the same samples
                  memcpy(bufferRequest.bufferOut, bufferCopy, streamVariables.samplesPerFrame * sizeof(int16_t));
                }
                // Then release buffer, and try again if there are more missing frames
                I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
              }
              bufferRelease.bufferHandleIn = NULL;
              bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
              I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
              if (gotBuffer) {
                if (volume <= streamVariables.maxVolume) {
                  if (volume < streamVariables.maxVolume) {
                    volume += 5;
                  } else if (volume > streamVariables.maxVolume) {
                    volume -= 5;
                  }
                  // Volume control
                  AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, AUDIO_CODEC_SPEAKER_HP, volume);
                }
              }
            }
            else if (streamVariables.i2sOpened == TRUE) {
              PIN_setOutputValue( ledPinHandle, Board_DIO27_ANALOG, 0);
              // Start I2S stream
              i2sStreamInProgress = I2SCC26XX_startStream(i2sHandle);
              if (!i2sStreamInProgress) {
                Display_print0(dispHandle, 5, 0, "Failed to start I2S stream");
                I2SCC26XX_stopStream(i2sHandle);
              }
              else {
                Display_print0(dispHandle, 5, 0, "Started I2S stream");
              }
              PIN_setOutputValue( ledPinHandle, Board_DIO27_ANALOG, 1);
            }
            else {
              // Failed to open, take the opportunity to try again
              // Allocate memory for decoded PCM data
              audio_decoded = ICall_malloc(sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE));
              if (audio_decoded) {
                i2sParams.blockSize              = streamVariables.samplesPerFrame;
                i2sParams.pvContBuffer           = (void *) audio_decoded;
                memset(audio_decoded, 0, sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE));
                i2sParams.ui32conBufTotalSize    = sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE);
                I2SCC26XX_open(i2sHandle, &i2sParams);
                Display_print0(dispHandle, 5, 0, "Opened I2S driver");
                volume = 40;
                streamVariables.i2sOpened = TRUE;
              }
              else {
                Display_print0(dispHandle, 5, 0, "Failed to allocate mem for I2S driver on frame");
              }
            }
#else //STREAM_TO_PC
#ifdef UART_DUMP_UNCOMPRESSED
            if (audio_decoded) {
              PIN_setOutputValue( ledPinHandle, Board_DIO26_ANALOG, 0);
              if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC) {
                sbc_decode(&sbc, audio_encoded, MSBC_ENCODED_SIZE, audio_decoded, streamVariables.samplesPerFrame * sizeof(int16_t), &written);
              }
              else {
                streamVariables.pv = BUILD_UINT16(audio_encoded[2], audio_encoded[3]);
                streamVariables.si = audio_encoded[1];
                Codec1_decodeBuff(audio_decoded, (uint8_t *)&audio_encoded[4], streamVariables.samplesPerFrame * sizeof(int16_t), &streamVariables.si, &streamVariables.pv);
              }
              PIN_setOutputValue( ledPinHandle, Board_DIO26_ANALOG, 1);
              static uint8_t pcmHeader[3] = {0xAD, 0x00, 0xCC};
              UART_write(uartHandle, pcmHeader, sizeof(pcmHeader));
              pcmHeader[1]++;
              if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC) {
                pcmHeader[2] = 0xCC;
              }
              else {
                pcmHeader[2] = 0xCD;
              }
              PIN_setOutputValue( ledPinHandle, Board_DIO26_ANALOG, 0);
              UART_write(uartHandle, audio_decoded, streamVariables.samplesPerFrame * sizeof(int16_t));
#else //!UART_DUMP_UNCOMPRESSED
            if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC) {
              UART_write(uartHandle, audio_encoded, MSBC_ENCODED_SIZE);
            }
            else {
              UART_write(uartHandle, audio_encoded, BLEAUDIO_BUFSIZE_ADPCM + BLEAUDIO_HDRSIZE_ADPCM);
            }
#endif //UART_DUMP_UNCOMPRESSED
            PIN_setOutputValue( ledPinHandle, Board_DIO26_ANALOG, 1);
            }
            else{
              audio_decoded = ICall_malloc(sizeof(int16_t) * streamVariables.samplesPerFrame);
            }
#endif //STREAM_TO_AUDBOOST
            frameReady = FALSE;
          }
#ifdef STREAM_TO_AUDBOOST
          else if ((streamVariables.i2sOpened == FALSE)&& (streamVariables.streamType != BLE_AUDIO_CMD_STOP)) {
            // Failed to open, take the opportunity to try again
            // Allocate memory for decoded PCM data
           audio_decoded = ICall_malloc(sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE));
            if (audio_decoded) {
              i2sParams.blockSize              = streamVariables.samplesPerFrame;
              i2sParams.pvContBuffer           = (void *) audio_decoded;
              memset(audio_decoded, 0, sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE));
              i2sParams.ui32conBufTotalSize    = sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE);
              I2SCC26XX_open(i2sHandle, &i2sParams);
              volume = 40;
              Display_print0(dispHandle, 5, 0, "Opened I2S driver");
              streamVariables.i2sOpened = TRUE;
            }
            else {
              Display_print0(dispHandle, 5, 0, "Failed to allocate mem for I2S driver on fragment");
            }
          }
#else //STREAM_TO_PC
#ifdef UART_DUMP_UNCOMPRESSED
          if (!audio_decoded) {
              audio_decoded = ICall_malloc(sizeof(int16_t) * streamVariables.samplesPerFrame);
              Display_print0(dispHandle, 5, 0, "Failed to allocate mem for decoding");
          }
#endif //UART_DUMP_UNCOMPRESSED
#endif //STREAM_TO_AUDBOOST
          PIN_setOutputValue( ledPinHandle, Board_DIO28_ANALOG, 1);
      }
      else if(pMsg->msg.handleValueNoti.handle == audioStartCharValueHandle)
      {
        PIN_setOutputValue( ledPinHandle, Board_DIO27_ANALOG, 0);
        // Audio/Voice commands are 1B in length
        if(AUDIOPROFILE_CMD_LEN == pMsg->msg.handleValueNoti.len)
        {
          // If we received a stop command reset the audio_pkt_counter, SI, PV
          if(BLE_AUDIO_CMD_START == *(pMsg->msg.handleValueNoti.pValue))
          {
            if (streamVariables.streamType != BLE_AUDIO_CMD_STOP) {
              Display_print0(dispHandle, 5, 0, "Already started stream");
            }
            else {
                  numberOfPackets = 0;
                  lostPackets = 0;
                  prevSeqNum = 0;
                  counterToAccountForSamplingFrequencyDifference = 0;
                  audio_pkt_counter = 0;
                  PIN_setOutputValue(ledPinHandle, Board_RLED, Board_LED_ON);
                  streamVariables.streamType = BLE_AUDIO_CMD_START;
                  streamVariables.notificationsPerFrame = BLEAUDIO_NUM_NOT_PER_FRAME_ADPCM;
                  streamVariables.samplesPerFrame = ADPCM_SAMPLES_PER_FRAME;
                  streamVariables.maxVolume = 85 - 10;
                  // Allocate memory for decoded PCM data
#ifdef STREAM_TO_AUDBOOST
                  audio_decoded = ICall_malloc(sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE));

                  if (audio_decoded) {
                    i2sParams.blockSize              = streamVariables.samplesPerFrame;
                    i2sParams.pvContBuffer           = (void *) audio_decoded;
                    memset(audio_decoded, 0, sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE));
                    i2sParams.ui32conBufTotalSize    = sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE);
                    I2SCC26XX_open(i2sHandle, &i2sParams);
                    Display_print1(dispHandle, 5, 0, "Opened I2S driver: %d", 1);
                    streamVariables.i2sOpened = TRUE;
                  }
                  else {
                        Display_print0(dispHandle, 5, 0, "Failed to allocate mem for I2S driver on start");
                  }
#else //STREAM_TO_PC
#ifdef UART_DUMP_UNCOMPRESSED
                  audio_decoded = ICall_malloc(sizeof(int16_t) * streamVariables.samplesPerFrame);
#endif //UART_DUMP_UNCOMPRESSED
#endif //STREAM_TO_AUDBOOST
                  Display_print0(dispHandle, 5, 0, "ADPCM Stream");
            }
          }
          else if(BLE_AUDIO_CMD_START_MSBC == *(pMsg->msg.handleValueNoti.pValue))
          {
            if (streamVariables.streamType != BLE_AUDIO_CMD_STOP) {
              Display_print0(dispHandle, 5, 0, "Already started stream");
            }
            else {
                  audio_pkt_counter = 0;
                  PIN_setOutputValue(ledPinHandle, Board_RLED, Board_LED_ON);
                  streamVariables.streamType = BLE_AUDIO_CMD_START_MSBC;
                  streamVariables.notificationsPerFrame = BLEAUDIO_NUM_NOT_PER_FRAME_MSBC;
                  streamVariables.samplesPerFrame = MSBC_SAMPLES_PER_FRAME;
                  streamVariables.maxVolume = 90 - 15;
                  // Allocate memory for decoded PCM data
#ifdef STREAM_TO_AUDBOOST
                  audio_decoded = ICall_malloc(sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE));

                  if (audio_decoded) {
                    i2sParams.blockSize              = streamVariables.samplesPerFrame;
                    i2sParams.pvContBuffer           = (void *) audio_decoded;
                    memset(audio_decoded, 0, sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE));
                    i2sParams.ui32conBufTotalSize    = sizeof(int16_t) * (streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE);
                    I2SCC26XX_open(i2sHandle, &i2sParams);
                    Display_print0(dispHandle, 5, 0, "Opened I2S driver");
                    streamVariables.i2sOpened = TRUE;
                  }
                  else {
                    Display_print0(dispHandle, 5, 0, "Failed to allocate mem for I2S driver on start");
                  }
#else //STREAM_TO_PC
#ifdef UART_DUMP_UNCOMPRESSED
                  audio_decoded = ICall_malloc(sizeof(int16_t) * streamVariables.samplesPerFrame);
#endif //UART_DUMP_UNCOMPRESSED
#endif //STREAM_TO_AUDBOOST
                // Initialize encoder
                sbc_init_msbc(&sbc, 0);
                Display_print0(dispHandle, 5, 0, "mSBC Stream");
            }
          }
          else if(BLE_AUDIO_CMD_STOP == *(pMsg->msg.handleValueNoti.pValue))
          {
            audio_pkt_counter = 0;
            PIN_setOutputValue(ledPinHandle, Board_RLED, Board_LED_OFF);
#ifdef STREAM_TO_AUDBOOST
            if (i2sStreamInProgress) {
              I2SCC26XX_stopStream(i2sHandle);
              i2sStreamInProgress = false;
              Display_print0(dispHandle, 5, 0, "Stopped I2S stream");

              /* Turn output volume back down */ //TODO: Turn off codec
              volume = 0;
              AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, AUDIO_CODEC_SPEAKER_HP, volume);

              if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC)
              {
                sbc_finish(&sbc);
              }
            }
            if (streamVariables.i2sOpened == TRUE)
            {
              I2SCC26XX_close(i2sHandle);
              streamVariables.i2sOpened = FALSE;
              Display_print0(dispHandle, 5, 0, "Closed I2S driver");
              if (audio_decoded) {
                ICall_free(audio_decoded);
                audio_decoded = NULL;
                Display_print0(dispHandle, 5, 0, "Free'd memory for I2S driver");
              }
              else {
                asm(" NOP");
                Display_print0(dispHandle, 5, 0, "Failed to free memory for I2S driver");
              }
            }
#else //STREAM_TO_PC
#ifdef UART_DUMP_UNCOMPRESSED
            if (audio_decoded) {
              ICall_free(audio_decoded);
            }
#endif //UART_DUMP_UNCOMPRESSED
            if (streamVariables.streamType == BLE_AUDIO_CMD_START_MSBC)
            {
              sbc_finish(&sbc);
            }
#endif //STREAM_TO_AUDBOOST
            streamVariables.streamType = BLE_AUDIO_CMD_STOP;
            Display_print0(dispHandle, 5, 0, "No Stream");
          }
          PIN_setOutputValue( ledPinHandle, Board_DIO27_ANALOG, 1);
        }
      }
      break;

    case ATT_FIND_BY_TYPE_VALUE_RSP:
      // Response from GATT_DiscPrimaryServiceByUUID
      // Service found, store handles
      if ( pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        svcStartHdl =
          ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        svcEndHdl =
          ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      }
      // If procedure complete
      else if ( pMsg->hdr.status == bleProcedureComplete )
      {
        if ( svcStartHdl != 0 )
        {
          if ( serviceToDiscover == AUDIO_SERV_UUID)
          {
            // Discover all characteristics
            GATT_DiscAllChars( connHandle, svcStartHdl, svcEndHdl, selfEntity );
          }

        }
      }
      break;

    case ATT_ERROR_RSP:

      if (serviceToDiscover ==  AUDIO_SERV_UUID
            && pMsg->msg.errorRsp.reqOpcode == ATT_FIND_BY_TYPE_VALUE_REQ
            && pMsg->msg.errorRsp.handle == 0x0001)
            //0x0001 is the start attribute handle of 0xfff0, AUDIO_SERV_UUID
      {
          if ( (enableCCCDs == TRUE) && (audioStartCharValueHandle != GATT_INVALID_HANDLE))
          {
              audioStartCCCHandle = audioStartCharValueHandle + 1 ;
              SimpleBLECentral_EnableNotification( connHandle, audioStartCCCHandle );
          }
      }
      break;

    case ATT_READ_BY_TYPE_RSP:
      // Response from Discover all Characteristics.
      // Success indicates packet with characteristic discoveries.
      if ( pMsg->hdr.status == SUCCESS )
      {
        attReadByTypeRsp_t *pRsp = &pMsg->msg.readByTypeRsp;

        if( serviceToDiscover ==  AUDIO_SERV_UUID )
        {
          uint16 charUUID = GATT_INVALID_HANDLE;
          uint16 *pHandle = &charUUID;
          /* Write into charUUID what Audio Profile char value we're dealing with */
          *pHandle = BUILD_UINT16( pRsp->pDataList[17] , pRsp->pDataList[18]);
          if      (charUUID == AUDIOPROFILE_START_UUID) {
            pHandle = &audioStartCharValueHandle;
            *pHandle = BUILD_UINT16( pRsp->pDataList[3] , pRsp->pDataList[4]);
          }
          else if (charUUID == AUDIOPROFILE_AUDIO_UUID ){
            pHandle = &audioDataCharValueHandle;
            *pHandle = BUILD_UINT16( pRsp->pDataList[3] , pRsp->pDataList[4]);
          }
        }
        break;
      }

      // This indicates that there is no more characteristic data
      // to be discovered within the given handle range.
      else if ( pMsg->hdr.status == bleProcedureComplete )
      {
        if ( serviceToDiscover == AUDIO_SERV_UUID )
        {
          /* This kicks off the enabling the 1st of notification enable event */
          if (audioStartCharValueHandle != GATT_INVALID_HANDLE) {
            audioStartCCCHandle = audioStartCharValueHandle + 1 ;
            SimpleBLECentral_EnableNotification( connHandle, audioStartCCCHandle );
          }
          break;
        }

      }
      break;

    case ATT_WRITE_RSP:
      if ( pMsg->hdr.status == SUCCESS && !serviceDiscComplete )
      {
          uint16 handle = GATT_INVALID_HANDLE;

          // Chain the CCCD enable writes so that a RSP for one triggers the next enable.
          if (audioStartCCCHandle == GATT_INVALID_HANDLE) {
            handle = audioStartCCCHandle = audioStartCharValueHandle + 1;
          }
          else if (audioDataCCCHandle == GATT_INVALID_HANDLE) {
            handle = audioDataCCCHandle = audioDataCharValueHandle + 1;
          }
          else {
            serviceDiscComplete = TRUE;
            break;
          }

          SimpleBLECentral_EnableNotification( connHandle, handle );

          break;

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

      // Service Change indication
      case ATT_EXCHANGE_MTU_RSP:
      {
        Display_print1(dispHandle, 10, 0, "server Rx MTU size : %d", pMsg->msg.exchangeMTURsp.serverRxMTU);
      }
        break;

    default:
      // Unknown event
      break;
    } //switch
  } // else - in case a GATT message came after a connection has dropped, ignore it.
  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
      {
        int8 rssi = (int8)pMsg->pReturnParam[3];

        Display_print1(dispHandle, 4, 0, "RSSI -dB: %d", (uint32_t)(-rssi));
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status)
{
  Display_clearLines(dispHandle, 5, 5);
  switch (state)
  {
  case GAPBOND_PAIRING_STATE_STARTED:
    Display_print0(dispHandle, 2, 0, "Pairing started");
    break;

  case GAPBOND_PAIRING_STATE_COMPLETE:
    if (status == SUCCESS)
    {
      // Enter a GAP Bond manager Paired state
      Display_print0(dispHandle, 2, 0, "Pairing success");

      if (FALSE == serviceDiscComplete)
      {
          // Begin Service Discovery of AUDIO Service to find out report handles
          serviceToDiscover = AUDIO_SERV_UUID;
          SimpleBLECentral_DiscoverService( connHandle, serviceToDiscover );
      }
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
    }
    break;

  case GAPBOND_PAIRING_STATE_BOND_SAVED:
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bond Saved");
    }
    break;

  case GAPBOND_PAIRING_STATE_BONDED:
    if (status == SUCCESS)
    {

      if ( osal_memcmp( remoteHandles.lastRemoteAddr, remoteAddr, B_ADDR_LEN ) == TRUE  )
      {

        if (
            ( remoteHandles.audioStartCharValueHandle == GATT_INVALID_HANDLE )         ||
              ( remoteHandles.audioDataCharValueHandle == GATT_INVALID_HANDLE )
                )
        {
          serviceToDiscover = AUDIO_SERV_UUID;

          // We must perform service discovery again, something might have changed.
          // Begin Service Discovery
          SimpleBLECentral_DiscoverService( connHandle, serviceToDiscover );

          serviceDiscComplete = FALSE;
          audioConfigEnable =0; //This will re-trig an audio configuration if needed
        }
        else
        {
          // No change, restore handle info.
          // bonding indicates that we probably already enabled all these characteristics. easy fix if not.
          serviceDiscComplete    = TRUE;

          audioStartCharValueHandle = remoteHandles.audioStartCharValueHandle;
          audioDataCharValueHandle  = remoteHandles.audioDataCharValueHandle;

          //Still, Force update of Audio config for now...
          audioConfigEnable =1;
        }
      }
      else if ( osal_isbufset( remoteHandles.lastRemoteAddr, 0x00, B_ADDR_LEN ) == TRUE )
      {
        // lastRemoteAddr is all 0's, which means the device was bonded before
        // it was power-cycled, and that we probably already enabled all CCCDs.
        // So, we only need to find out attribute report handles.
        enableCCCDs = FALSE;

        // Begin Service Discovery of HID Service to find out report handles
        serviceToDiscover = AUDIO_SERV_UUID;
        SimpleBLECentral_DiscoverService( connHandle, serviceToDiscover );
      }

      Display_print0(dispHandle, 2, 0, "Bond save success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
    }
    break;

  default:
    break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs)
{
  uint32_t  passcode;

  // Create random passcode
  passcode = Util_GetTRNG();
  passcode %= 1000000;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void SimpleBLECentral_startDiscovery(void)
{
  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = 0;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      SimpleBLECentral_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
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
 * @fn      SimpleBLECentral_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
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
 * @fn      SimpleBLECentral_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SimpleBLECentral_enqueueMsg(SBC_STATE_CHANGE_EVT,
                                  SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    SimpleBLECentral_enqueueMsg(SBC_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    SimpleBLECentral_enqueueMsg(SBC_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_startDiscHandler(UArg a0)
{
  events |= SBC_START_DISCOVERY_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLECentral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_keyChangeHandler(uint8 keys)
{
  SimpleBLECentral_enqueueMsg(SBC_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_readRssiHandler
 *
 * @brief   Read RSSI handler function
 *
 * @param   a0 - read RSSI index
 *
 * @return  none
 */
void SimpleBLECentral_readRssiHandler(UArg a0)
{
  SimpleBLECentral_enqueueMsg(SBC_RSSI_READ_EVT, SUCCESS,
                              (uint8_t *)&readRssi[a0]);
}

/*********************************************************************
 * @fn      SimpleBLECentral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  sbcEvt_t *pMsg = ICall_malloc(sizeof(sbcEvt_t));

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
 * @fn      SimpleBLECentral_FindHIDRemote
 *
 * @brief   Search Scan Response data for a "HID AdvRemote"
 *
 * @param   pData - received advertising data
 * @param   dataLen - advertising data length
 *
 * @return  TRUE if found, false otherwise
 */
static uint8 SimpleBLECentral_FindHIDRemote( uint8* pData, uint8 length )
{
  resultFindRC = FALSE;
  resultFindST = FALSE;
  static uint8 remoteNameRC[] =
  {
    'C', 'C', '2', '6', '5', '0', ' ', 'R', 'C'
  };
  static uint8 remoteNameTx[] =
  {
      'S', 'i', 'm', 'p', 'l', 'e',
      'B', 'L', 'E',
      'A', 'u', 'd', 'i', 'o', 'T', 'x',
  };

  // move pointer to the start of the scan response data.
  pData += 2;

  // adjust length as well
  length -= 2;

  streamVariables.sourceIsPDM = FALSE;
  if (length == sizeof(remoteNameRC)) {
      resultFindRC = osal_memcmp( remoteNameRC, pData, length );
      streamVariables.sourceIsPDM = TRUE;
  }
  else if (length == sizeof(remoteNameTx)) {
      resultFindRC = osal_memcmp( remoteNameTx, pData, length );
  }

  // did not find RC, then search for ST
  if (!resultFindRC){

    static uint8 remoteNameST[] =
    {
      'C', 'C', '2', '6', '5', '0', ' ',
      'S', 'e', 'n',  's',  'o',  'r',  'T',  'a',  'g'
    };

    length -= 9;
    resultFindST = osal_memcmp( remoteNameST, pData, length );
  }
  return (resultFindRC || resultFindST);
}

/*********************************************************************
 * @fn      SimpleBLECentral_BondCount
 *
 * @brief   Gets the total number of bonded devices.
 *
 * @param   none.
 *
 * @return  number of bonded devices.
 */
static uint8 SimpleBLECentral_BondCount( void )
{
  uint8 bondCnt = 0;

  VOID GAPBondMgr_GetParameter( GAPBOND_BOND_COUNT, &bondCnt );

  return ( bondCnt );
}

/*********************************************************************
 * @fn      SimpleBLECentral_SetIdle
 *
 * @brief   Set the device to idle.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_SetIdle( void )
{
  state = BLE_STATE_IDLE;
  Util_stopClock(&scanningToggleClock);
  PIN_setOutputValue( ledPinHandle, Board_GLED, 0);
  PIN_setOutputValue( ledPinHandle, Board_RLED, 1);
  Display_print0(dispHandle, 2, 0, "Idle...");
}

/*********************************************************************
 * @fn      SimpleBLECentral_EstablishLink
 *
 * @brief   Establish a link to a peer device.
 *
 * @param   whiteList - determines use of the white list
 * @param   addrType - address type of the peer devic
 * @param   remoteAddr - peer device address
 *
 * @return  none
 */
static void SimpleBLECentral_EstablishLink( uint8 whiteList, uint8 addrType, uint8 *remoteAddr )
{
  if ( state != BLE_STATE_CONNECTED )
  {
    state = BLE_STATE_CONNECTING;

    // Try to connect to remote device
    GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                 DEFAULT_LINK_WHITE_LIST,
                                 addrType, remoteAddr);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_DiscoverService
 *
 * @brief   Discover service using UUID.
 *
 * @param   connHandle - connection handle to do discovery on
 * @param   svcUuid - service UUID to discover
 *
 * @return  none
 */
static void SimpleBLECentral_DiscoverService( uint16 connHandle, uint16 svcUuid )
{
  if(svcUuid == AUDIO_SERV_UUID) // only take care of Audio Service in this project
  {
    uint8 uuid[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0,
                      0x00, 0x40, 0x51, 0x04, LO_UINT16( svcUuid ), HI_UINT16( svcUuid ), 0x00, 0xF0};

    VOID GATT_DiscPrimaryServiceByUUID( connHandle, uuid, ATT_UUID_SIZE, selfEntity );
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_EnableNotification
 *
 * @brief   Enable notification for a given attribute handle.
 *
 * @param   connHandle - connection handle to send notification on
 * @param   attrHandle - attribute handle to send notification for
 *
 * @return  none
 */
static void SimpleBLECentral_EnableNotification( uint16 connHandle, uint16 attrHandle )
{
  attWriteReq_t req;

  req.pValue = GATT_bm_alloc( connHandle, ATT_WRITE_REQ, 2, NULL );
  if ( req.pValue != NULL )
  {
    uint8 notificationsOn[] = {0x01, 0x00};

    req.handle = attrHandle;

    req.len = 2;
    osal_memcpy(req.pValue, notificationsOn, 2);

    req.sig = 0;
    req.cmd = 0;

    if ( GATT_WriteCharValue( connHandle, &req, selfEntity ) != SUCCESS )
    {
      GATT_bm_free( (gattMsg_t *)&req, ATT_WRITE_REQ );
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_SaveHandles
 *
 * @brief   save handle information in case next connection is to the
 *          same bonded device.
 *
 * @param   none.
 *
 * @return  none.
 */
static void SimpleBLECentral_SaveHandles( void )
{
  // Service and Characteristic discovery variables.
  remoteHandles.keyCharHandle          = keyCharHandle;

//  // CCC's of the notifications
//  remoteHandles.keyCCCHandle           = keyCCCHandle;

  remoteHandles.audioStartCharValueHandle = audioStartCharValueHandle;
  remoteHandles.audioDataCharValueHandle  = audioDataCharValueHandle;
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLECentral_scanningToggleHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

#ifdef STREAM_TO_AUDBOOST
static void I2SCC26XX_i2sCallbackFxn(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *notification) {
  static uint8_t lostConsecutively = 0;
  uint32_t hwiKey;
  if (notification->status == I2SCC26XX_STREAM_BUFFER_READY) {
    // Provide buffer
    PIN_setOutputValue( ledPinHandle, Board_DIO25_ANALOG, !(PIN_getOutputValue(Board_DIO25_ANALOG)));
    lostConsecutively = 0;
    streamVariables.maxVolume = 75;
  }
  else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY_BUT_NO_AVAILABLE_BUFFERS) {
    // Provide buffer
    lostConsecutively++;
    PIN_setOutputValue( ledPinHandle, Board_DIO25_ANALOG, !(PIN_getOutputValue(Board_DIO25_ANALOG)));
    if ((lostConsecutively > 5 ) & (lostConsecutively <= 40 )) {
      if( volume > 0 ){
        // hwiKey = Hwi_disable();
        // events |= SBC_AUDIO_VOLUMEDOWN_EVT;
        // Hwi_restore(hwiKey);
        // Semaphore_post(sem);
        streamVariables.maxVolume = 0;
      }
    }
    else if (lostConsecutively > 40 ) {
      hwiKey = Hwi_disable();
      events |= SBC_AUDIO_FORCESTOP_EVT;
      lostConsecutively = 0;
      Hwi_restore(hwiKey);
      Semaphore_post(sem);
    }
  }
  else {
    PIN_setOutputValue( ledPinHandle, Board_RLED, !(PIN_getOutputValue(Board_RLED)));
  }
}
#endif //STREAM_TO_AUDBOOST

/*********************************************************************
*********************************************************************/
