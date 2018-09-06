/******************************************************************************

 @file       uble_bcast_scan.c

 @brief This file contains the implementation of uble_bcast_scan app which is
        capable of advertising and scanning using the micro BLE stack

 Group: WCS BTS
 Target Device: CC2640R2

 ******************************************************************************

 Copyright (c) 2016-2017, Texas Instruments Incorporated
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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <stdlib.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"

#include "board.h"
#include "board_key.h"
#include <ti/display/Display.h>
#include <ti/display/lcd/LCDDogm1286.h>

// DriverLib
#include <driverlib/aon_batmon.h>
#include "uble.h"
#include "ugap.h"
#include "urfc.h"

#include "util.h"
#include "gap.h"

#include <menu/two_btn_menu.h>
#include "uble_bcast_scan_menu.h"
#include "uble_bcast_scan.h"

/*********************************************************************
 * MACROS
 */

// Eddystone Base 128-bit UUID: EE0CXXXX-8786-40BA-AB96-99B91AC981D8
#define EDDYSTONE_BASE_UUID_128( uuid )  0xD8, 0x81, 0xC9, 0x1A, 0xB9, 0x99, \
                                         0x96, 0xAB, 0xBA, 0x40, 0x86, 0x87, \
                           LO_UINT16( uuid ), HI_UINT16( uuid ), 0x0C, 0xEE

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval (units of 0.625 ms)
#define DEFAULT_ADVERTISING_INTERVAL          160 // 100 ms

#define UBS_DISPLAY_TYPE Display_Type_UART

// Task configuration
#define UBS_TASK_PRIORITY                     3

#ifndef UBS_TASK_STACK_SIZE
#define UBS_TASK_STACK_SIZE                   800
#endif

// RTOS Event to queue application events
#define UEB_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Application Events
#define UBS_EVT_KEY_CHANGE      0x0001
#define UBS_EVT_MICROBLESTACK   0x0002

// Pre-generated Random Static Address
#define UBS_PREGEN_RAND_STATIC_ADDR    {0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc}

// Eddystone definitions
#define EDDYSTONE_SERVICE_UUID                  0xFEAA

#define EDDYSTONE_FRAME_TYPE_UID                0x00
#define EDDYSTONE_FRAME_TYPE_URL                0x10
#define EDDYSTONE_FRAME_TYPE_TLM                0x20

#define EDDYSTONE_FRAME_OVERHEAD_LEN            8
#define EDDYSTONE_SVC_DATA_OVERHEAD_LEN         3
#define EDDYSTONE_MAX_URL_LEN                   18

// # of URL Scheme Prefix types
#define EDDYSTONE_URL_PREFIX_MAX        4
// # of encodable URL words
#define EDDYSTONE_URL_ENCODING_MAX      14

#define EDDYSTONE_URI_DATA_DEFAULT      "http://www.ti.com/ble"

// Row numbers
#define UBS_ROW_RESULT        TBM_ROW_APP
#define UBS_ROW_STATUS_1      (TBM_ROW_APP + 1)
#define UBS_ROW_STATUS_2      (TBM_ROW_APP + 2)
#define UBS_ROW_BCAST_STATE   (TBM_ROW_APP + 3)
#define UBS_ROW_SCAN_STATE    (TBM_ROW_APP + 4)
#define UBS_ROW_SCAN_1        (TBM_ROW_APP + 5)
#define UBS_ROW_SCAN_2        (TBM_ROW_APP + 6)
#define UBS_ROW_SCAN_3        (TBM_ROW_APP + 7)
#define UBS_ROW_MONITOR_STATE UBS_ROW_SCAN_STATE
#define UBS_ROW_MONITOR_1     UBS_ROW_SCAN_1
#define UBS_ROW_MONITOR_2     UBS_ROW_SCAN_2
#define UBS_ROW_MONITOR_3     UBS_ROW_SCAN_3

/*********************************************************************
 * TYPEDEFS
 */

// App to App event
typedef struct {
  uint16 event;
  uint8 data;
} ubsEvt_t;

// Eddystone UID frame
typedef struct {
  uint8 frameType;      // UID
  int8 rangingData;
  uint8 namespaceID[10];
  uint8 instanceID[6];
  uint8 reserved[2];
} eddystoneUID_t;

// Eddystone URL frame
typedef struct {
  uint8 frameType;      // URL | Flags
  int8 txPower;
  uint8 encodedURL[EDDYSTONE_MAX_URL_LEN];  // the 1st byte is prefix
} eddystoneURL_t;

// Eddystone TLM frame
typedef struct {
  uint8 frameType;      // TLM
  uint8 version;        // 0x00 for now
  uint8 vBatt[2];       // Battery Voltage, 1mV/bit, Big Endian
  uint8 temp[2];        // Temperature. Signed 8.8 fixed point
  uint8 advCnt[4];      // Adv count since power-up/reboot
  uint8 secCnt[4];      // Time since power-up/reboot
                          // in 0.1 second resolution
} eddystoneTLM_t;

typedef union {
  eddystoneUID_t uid;
  eddystoneURL_t url;
  eddystoneTLM_t tlm;
} eddystoneFrame_t;

typedef struct {
  uint8 length1;        // 2
  uint8 dataType1;      // for Flags data type (0x01)
  uint8 data1;          // for Flags data (0x04)
  uint8 length2;        // 3
  uint8 dataType2;      // for 16-bit Svc UUID list data type (0x03)
  uint8 data2;          // for Eddystone UUID LSB (0xAA)
  uint8 data3;          // for Eddystone UUID MSB (0xFE)
  uint8 length;         // Eddystone service data length
  uint8 dataType3;      // for Svc Data data type (0x16)
  uint8 data4;          // for Eddystone UUID LSB (0xAA)
  uint8 data5;          // for Eddystone UUID MSB (0xFE)
  eddystoneFrame_t frame;
} eddystoneAdvData_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Event globally used to post local events and pend on local events.
static Event_Handle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct ubsTask;
uint8 ubsTaskStack[UBS_TASK_STACK_SIZE];

#if defined(FEATURE_OBSERVER)
static bool UBLEBcastScan_initObserver(void);
#endif /* FEATURE_OBSERVER */

// Current Adv Interval
static uint16 ubsAdvInterval = DEFAULT_ADVERTISING_INTERVAL;

static uint32 advCount = 0;
#if defined(FEATURE_OBSERVER)
static uint32 scanIndCount = 0;
#endif /* FEATURE_OBSERVER */

// Current TX Power
static int8 ubsTxPower = UBLE_PARAM_DFLT_TXPOWER;

// Current Adv Channel Map
static uint8 ubsAdvChanMap = UBLE_PARAM_DFLT_ADVCHANMAP;

// Current Time to Adv
static uint8 ubsTimeToAdv = UBLE_PARAM_DFLT_TIMETOADV;

// Adv Data Update Option
static bool bUpdateUponEvent = false;

// Current Duty Off Time
static uint16 ubsBcastDutyOff = 0;
// Current Duty On Time
static uint16 ubsBcastDutyOn = 0;

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static eddystoneAdvData_t eddystoneAdv = {
    // Flags; this sets the device to use general discoverable mode
    0x02,// length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED | GAP_ADTYPE_FLAGS_GENERAL,

    // Complete list of 16-bit Service UUIDs
    0x03,// length of this data including the data type byte
    GAP_ADTYPE_16BIT_COMPLETE, LO_UINT16(EDDYSTONE_SERVICE_UUID), HI_UINT16(
        EDDYSTONE_SERVICE_UUID),

    // Service Data
    0x03,// to be set properly later
    GAP_ADTYPE_SERVICE_DATA, LO_UINT16(EDDYSTONE_SERVICE_UUID), HI_UINT16(
        EDDYSTONE_SERVICE_UUID)
};

eddystoneUID_t eddystoneUID;
eddystoneURL_t eddystoneURL;
eddystoneTLM_t eddystoneTLM;

uint8 eddystoneURLDataLen;

// Array of URL Scheme Prefices
static char* eddystoneURLPrefix[EDDYSTONE_URL_PREFIX_MAX] = { "http://www.",
    "https://www.", "http://", "https://" };

// Array of URLs to be encoded
static char* eddystoneURLEncoding[EDDYSTONE_URL_ENCODING_MAX] = { ".com/",
    ".org/", ".edu/", ".net/", ".info/", ".biz/", ".gov/", ".com/", ".org/",
    ".edu/", ".net/", ".info/", ".biz/", ".gov/" };

// Eddystone frame type currently used
static uint8 currentFrameType = EDDYSTONE_FRAME_TYPE_UID;

// Pointer to application callback
keysPressedCB_t appKeyChangeHandler_st = NULL;


/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void UBLEBcastScan_init(void);
static void UBLEBcastScan_taskFxn(UArg a0, UArg a1);
static void UBLEBcastScan_initUID(void);
static void UBLEBcastScan_initURL(void);
static void UBLEBcastScan_updateAdvDataWithFrame(uint8 frameType);

static void UBLEBcastScan_keyChangeHandler(uint8 keys);
static void UBLEBcastScan_processAppMsg(ubsEvt_t *pMsg);

static void UBLEBcastScan_bcast_stateChangeCB(ugapBcastState_t newState);
static void UBLEBcastScan_bcast_advPrepareCB(void);
static void UBLEBcastScan_bcast_advDoneCB(bStatus_t status);

#if defined(FEATURE_OBSERVER)
static void UBLEBcastScan_scan_stateChangeCB(ugapObserverScan_State_t newState);
static void UBLEBcastScan_scan_indicationCB(bStatus_t status, uint8_t len, uint8_t *pPayload);
static void UBLEBcastScan_scan_windowCompleteCB(bStatus_t status);
#endif /* FEATURE_OBSERVER */

static bool UBLEBcastScan_initPostProcess(bStatus_t status, uint8* pAddrType);
static bStatus_t UBLEBcastScan_enqueueMsg(uint16 event, uint8 data);

static void UBLEBcastScan_eventProxy(void);

/*********************************************************************
 * CALLBACKS
 */

/*********************************************************************
 * @fn      UBLEBcastScan_menuSwitchCB
 *
 * @brief   Callback function to be called when the menu is switching
 *
 * @param   pCurrMenu - the menu object the focus is leaving
 * @param   pNextMenu - the menu object the focus is moving into
 *
 * @return  None.
 */
void UBLEBcastScan_menuSwitchCB(tbmMenuObj_t pCurrMenu, tbmMenuObj_t pNextMenu)
{
  return;
}


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      UBLEBcastScan_createTask
 *
 * @brief   Task creation function for the Micro Eddystone Beacon.
 *
 * @param   None.
 *
 * @return  None.
 */
void UBLEBcastScan_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = ubsTaskStack;
  taskParams.stackSize = UBS_TASK_STACK_SIZE;
  taskParams.priority = UBS_TASK_PRIORITY;

  Task_construct(&ubsTask, UBLEBcastScan_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      UBLEBcastScan_init
 *
 * @brief   Initialization function for the Micro Eddystone Beacon App
 *          Task. This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
static void UBLEBcastScan_init(void)
{
  ugapBcastCBs_t bcastCBs = {
  UBLEBcastScan_bcast_stateChangeCB,
  UBLEBcastScan_bcast_advPrepareCB,
  UBLEBcastScan_bcast_advDoneCB };

  bStatus_t status;

  // Create an RTOS event used to wake up this application to process events.
  syncEvent = Event_create(NULL, NULL);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Default is not to switch antenna
  uble_registerAntSwitchCB(NULL);

  uble_stackInit(UBLE_ADDRTYPE_PUBLIC, NULL, UBLEBcastScan_eventProxy,
                 RF_TIME_CRITICAL);
  ugap_bcastInit(&bcastCBs);

  // Setup key callback for keys
  Board_initKeys(UBLEBcastScan_keyChangeHandler);

  // Open Display.
  dispHandle = Display_open(UBS_DISPLAY_TYPE, NULL);

  // Initialize UID frame
  UBLEBcastScan_initUID();

  // Initialize URL frame
  UBLEBcastScan_initURL();

  // Initialize the two button menu
  tbm_initTwoBtnMenu(dispHandle, &ubsMenuMain, 3, NULL);

  status = uble_stackInit(UBLE_ADDRTYPE_PUBLIC, NULL, UBLEBcastScan_eventProxy,
                          RF_TIME_CRITICAL);

  UBLEBcastScan_initPostProcess(status, "PubAddr");
}

/*********************************************************************
 * @fn      UBLEBcastScan_initUID
 *
 * @brief   initialize UID frame
 *
 * @param   none
 *
 * @return  none
 */
static void UBLEBcastScan_initUID(void)
{
  // Set Eddystone UID frame with meaningless numbers for example.
  // This need to be replaced with some algorithm-based formula
  // for production.
  eddystoneUID.namespaceID[0] = 0x00;
  eddystoneUID.namespaceID[1] = 0x01;
  eddystoneUID.namespaceID[2] = 0x02;
  eddystoneUID.namespaceID[3] = 0x03;
  eddystoneUID.namespaceID[4] = 0x04;
  eddystoneUID.namespaceID[5] = 0x05;
  eddystoneUID.namespaceID[6] = 0x06;
  eddystoneUID.namespaceID[7] = 0x07;
  eddystoneUID.namespaceID[8] = 0x08;
  eddystoneUID.namespaceID[9] = 0x09;

  eddystoneUID.instanceID[0] = 0x04;
  eddystoneUID.instanceID[1] = 0x51;
  eddystoneUID.instanceID[2] = 0x40;
  eddystoneUID.instanceID[3] = 0x00;
  eddystoneUID.instanceID[4] = 0xB0;
  eddystoneUID.instanceID[5] = 0x00;
}

/*********************************************************************
 * @fn      UBLEBcastScan_encodeURL
 *
 * @brief   Encodes URL in accordance with Eddystone URL frame spec
 *
 * @param   urlOrg - Plain-string URL to be encoded
 *          urlEnc - Encoded URL. Should be URLCFGSVC_CHAR_URI_DATA_LEN-long.
 *
 * @return  0 if the prefix is invalid
 *          The length of the encoded URL including prefix otherwise
 */
uint8 UBLEBcastScan_encodeURL(char* urlOrg, uint8* urlEnc)
{
  uint8 i, j;
  uint8 urlLen;
  uint8 tokenLen;

  urlLen = (uint8) strlen(urlOrg);

  // search for a matching prefix
  for (i = 0; i < EDDYSTONE_URL_PREFIX_MAX; i++)
  {
    tokenLen = strlen(eddystoneURLPrefix[i]);
    if (strncmp(eddystoneURLPrefix[i], urlOrg, tokenLen) == 0)
    {
      break;
    }
  }

  if (i == EDDYSTONE_URL_PREFIX_MAX)
  {
    return 0;       // wrong prefix
  }

  // use the matching prefix number
  urlEnc[0] = i;
  urlOrg += tokenLen;
  urlLen -= tokenLen;

  // search for a token to be encoded
  for (i = 0; i < urlLen; i++)
  {
    for (j = 0; j < EDDYSTONE_URL_ENCODING_MAX; j++)
    {
      tokenLen = strlen(eddystoneURLEncoding[j]);
      if (strncmp(eddystoneURLEncoding[j], urlOrg + i, tokenLen) == 0)
      {
        // matching part found
        break;
      }
    }

    if (j < EDDYSTONE_URL_ENCODING_MAX)
    {
      memcpy(&urlEnc[1], urlOrg, i);
      // use the encoded byte
      urlEnc[i + 1] = j;
      break;
    }
  }

  if (i < urlLen)
  {
    memcpy(&urlEnc[i + 2], urlOrg + i + tokenLen, urlLen - i - tokenLen);
    return urlLen - tokenLen + 2;
  }

  memcpy(&urlEnc[1], urlOrg, urlLen);
  return urlLen + 1;
}

/*********************************************************************
 * @fn      UBLEBcastScan_initURL
 *
 * @brief   initialize URL frame
 *
 * @param   none
 *
 * @return  none
 */
static void UBLEBcastScan_initURL(void)
{
  // Set Eddystone URL frame with the URL of TI BLE site.
  eddystoneURLDataLen = UBLEBcastScan_encodeURL(
      EDDYSTONE_URI_DATA_DEFAULT, eddystoneURL.encodedURL);
}

/*********************************************************************
 * @fn      UBLEBcastScan_updateTLM
 *
 * @brief   Update TLM elements
 *
 * @param   none
 *
 * @return  none
 */
static void UBLEBcastScan_updateTLM(void)
{
  uint32 time100MiliSec;
  uint32 batt;

  // Battery voltage (bit 10:8 - integer, but 7:0 fraction)
  batt = AONBatMonBatteryVoltageGet();
  batt = (batt * 125) >> 5; // convert V to mV
  eddystoneTLM.vBatt[0] = HI_UINT16(batt);
  eddystoneTLM.vBatt[1] = LO_UINT16(batt);

  // Temperature - 19.5 (Celcius) for example
  eddystoneTLM.temp[0] = 19;
  eddystoneTLM.temp[1] = 256 / 2;

  // advertise packet cnt;
  eddystoneTLM.advCnt[0] = BREAK_UINT32(advCount, 3);
  eddystoneTLM.advCnt[1] = BREAK_UINT32(advCount, 2);
  eddystoneTLM.advCnt[2] = BREAK_UINT32(advCount, 1);
  eddystoneTLM.advCnt[3] = BREAK_UINT32(advCount, 0);

  // running time
  // the number of 100-ms periods that have passed since the beginning.
  // no consideration of roll over for now.
  time100MiliSec = Clock_getTicks() / (100000 / Clock_tickPeriod);
  eddystoneTLM.secCnt[0] = BREAK_UINT32(time100MiliSec, 3);
  eddystoneTLM.secCnt[1] = BREAK_UINT32(time100MiliSec, 2);
  eddystoneTLM.secCnt[2] = BREAK_UINT32(time100MiliSec, 1);
  eddystoneTLM.secCnt[3] = BREAK_UINT32(time100MiliSec, 0);
}

/*********************************************************************
 * @fn      UBLEBcastScan_updateAdvDataWithFrame
 *
 * @brief   Selecting the type of frame to be put in the service data
 *
 * @param   frameType - Eddystone frame type
 *
 * @return  none
 */
static void UBLEBcastScan_updateAdvDataWithFrame(uint8 frameType)
{
  if (frameType == EDDYSTONE_FRAME_TYPE_UID
      || frameType == EDDYSTONE_FRAME_TYPE_URL
      || frameType == EDDYSTONE_FRAME_TYPE_TLM)
  {
    eddystoneFrame_t* pFrame;
    uint8 frameSize;

    eddystoneAdv.length = EDDYSTONE_SVC_DATA_OVERHEAD_LEN;
    // Fill with 0s first
    memset((uint8*) &eddystoneAdv.frame, 0x00, sizeof(eddystoneFrame_t));

    switch (frameType) {
    case EDDYSTONE_FRAME_TYPE_UID:
      eddystoneUID.frameType = EDDYSTONE_FRAME_TYPE_UID;
      frameSize = sizeof(eddystoneUID_t);
      pFrame = (eddystoneFrame_t *) &eddystoneUID;
      break;

    case EDDYSTONE_FRAME_TYPE_URL:
      eddystoneURL.frameType = EDDYSTONE_FRAME_TYPE_URL;
      frameSize = sizeof(eddystoneURL_t) - EDDYSTONE_MAX_URL_LEN
          + eddystoneURLDataLen;
      pFrame = (eddystoneFrame_t *) &eddystoneURL;
      break;

    case EDDYSTONE_FRAME_TYPE_TLM:
      eddystoneTLM.frameType = EDDYSTONE_FRAME_TYPE_TLM;
      frameSize = sizeof(eddystoneTLM_t);
      pFrame = (eddystoneFrame_t *) &eddystoneTLM;
      break;
    }

    memcpy((uint8 *) &eddystoneAdv.frame, (uint8 *) pFrame, frameSize);
    eddystoneAdv.length += frameSize;

    uble_setParameter(UBLE_PARAM_ADVDATA,
                      EDDYSTONE_FRAME_OVERHEAD_LEN + eddystoneAdv.length,
                      &eddystoneAdv);
  }
}

/*********************************************************************
 * @fn      UBLEBcastScan_bcast_updateAdvData(void)
 *
 * @brief   Update Adv Data.
 *
 * @param   None.
 *
 * @return  None.
 */
static void UBLEBcastScan_bcast_updateAdvData(void)
{
  if (currentFrameType == EDDYSTONE_FRAME_TYPE_TLM)
  {
    UBLEBcastScan_updateTLM();
  }

  UBLEBcastScan_updateAdvDataWithFrame(currentFrameType);
}


/*********************************************************************
 * @fn      UBLEBcastScan_processEvent
 *
 * @brief   Application task entry point for the Micro Eddystone Beacon.
 *
 * @param   none
 *
 * @return  none
 */
static void UBLEBcastScan_taskFxn(UArg a0, UArg a1)
{
  volatile uint32 keyHwi;

  // Initialize application
  UBLEBcastScan_init();

  for (;;)
  {
    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    Event_pend(syncEvent, Event_Id_NONE, UEB_QUEUE_EVT, BIOS_WAIT_FOREVER);

    // If RTOS queue is not empty, process app message.
    while (!Queue_empty(appMsgQueue))
    {
      ubsEvt_t *pMsg;

      // malloc() is not thread safe. Must disable HWI.
      keyHwi = Hwi_disable();
      pMsg = (ubsEvt_t *) Util_dequeueMsg(appMsgQueue);
      Hwi_restore(keyHwi);

      if (pMsg)
      {
        // Process message.
        UBLEBcastScan_processAppMsg(pMsg);

        // free() is not thread safe. Must disable HWI.
        keyHwi = Hwi_disable();

        // Free the space from the message.
        free(pMsg);
        Hwi_restore(keyHwi);
      }
    }
  }
}

/*********************************************************************
 * @fn      UBLEBcastScan_handleKeys
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
static void UBLEBcastScan_handleKeys(uint8 shift, uint8 keys)
{
  (void) shift;  // Intentionally unreferenced parameter

  if (keys & KEY_LEFT)
  {
    // Check if the key is still pressed.
    if (PIN_getInputValue(Board_PIN_BUTTON0) == 0)
    {
      tbm_buttonLeft();
    }
  }
  else if (keys & KEY_RIGHT)
  {
    // Check if the key is still pressed.
    if (PIN_getInputValue(Board_PIN_BUTTON1) == 0)
    {
      tbm_buttonRight();
    }
  }
}

/*********************************************************************
 * @fn      UBLEBcastScan_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void UBLEBcastScan_keyChangeHandler(uint8 keys)
{
  UBLEBcastScan_enqueueMsg(UBS_EVT_KEY_CHANGE, keys);
}

/*********************************************************************
 * @fn      UBLEBcastScan_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void UBLEBcastScan_processAppMsg(ubsEvt_t *pMsg)
{
  switch (pMsg->event)
  {
  case UBS_EVT_KEY_CHANGE:
    UBLEBcastScan_handleKeys(0, pMsg->data);
    break;

  case UBS_EVT_MICROBLESTACK:
    uble_processMsg();
    break;

  default:
    // Do nothing.
    break;
  }
}

/*********************************************************************
 * @fn      UBLEBcastScan_bcast_stateChange_CB
 *
 * @brief   Callback from Micro Broadcaster indicating a state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void UBLEBcastScan_bcast_stateChangeCB(ugapBcastState_t newState)
{
  switch (newState)
  {
  case UGAP_BCAST_STATE_INITIALIZED:
    Display_print0(dispHandle, UBS_ROW_BCAST_STATE, 0,
                   "Broadcast_State: Initialized");
    break;

  case UGAP_BCAST_STATE_IDLE:
    Display_print0(dispHandle, UBS_ROW_BCAST_STATE, 0,
                   "Broadcast_State: Idle");
    break;

  case UGAP_BCAST_STATE_ADVERTISING:
    Display_print0(dispHandle, UBS_ROW_BCAST_STATE, 0,
                   "Broadcast_State: Advertising");
    break;

  case UGAP_BCAST_STATE_WAITING:
    Display_print0(dispHandle, UBS_ROW_BCAST_STATE, 0,
                   "Broadcast_State: Waiting");
    break;

  default:
    break;
  }
}

/*********************************************************************
 * @fn      UBLEBcastScan_bcast_advPrepareCB
 *
 * @brief   Callback from Micro Broadcaster notifying that the next
 *          advertising event is about to start so it's time to update
 *          the adv payload.
 *
 * @param   None.
 *
 * @return  None.
 */
static void UBLEBcastScan_bcast_advPrepareCB(void)
{
  static uint8* pStr[3] = {".", "..", "..."};
  static uint8 progress = 0;

  UBLEBcastScan_bcast_updateAdvData();

  Display_print1(dispHandle, UBS_ROW_RESULT, 0,
                 "Adv Prepare CB%s", pStr[progress]);

  if (++progress == 3)
  {
    progress = 0;
  }
}

/*********************************************************************
 * @fn      UBLEBcastScan_bcast_advDoneCB
 *
 * @brief   Callback from Micro Broadcaster notifying that an
 *          advertising event has been done.
 *
 * @param   status - How the last event was done. SUCCESS or FAILURE.
 *
 * @return  None.
 */
static void UBLEBcastScan_bcast_advDoneCB(bStatus_t status)
{
  advCount++;

  Display_print1(dispHandle, UBS_ROW_STATUS_2, 0,
                 "%d Adv\'s done", advCount);
}


/*********************************************************************
 * @fn      UBLEBcastScan_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   data - message data.
 *
 * @return  TRUE or FALSE
 */
static bStatus_t UBLEBcastScan_enqueueMsg(uint16 event, uint8 data)
{
  volatile uint32 keyHwi;
  ubsEvt_t *pMsg;
  uint8_t status = FALSE;

  // malloc() is not thread safe. Must disable HWI.
  keyHwi = Hwi_disable();

  // Create dynamic pointer to message.
  pMsg = (ubsEvt_t*) malloc(sizeof(ubsEvt_t));
  if (pMsg != NULL)
  {
    pMsg->event = event;
    pMsg->data = data;

    // Enqueue the message.
    status = Util_enqueueMsg(appMsgQueue, syncEvent, (uint8*) pMsg);
  }
  Hwi_restore(keyHwi);
  return status;
}

/*********************************************************************
 * @fn      UBLEBcastScan_eventProxy
 *
 * @brief   Proxy function for posting an event from the uBLE Stack
 *          to the application
 *
 * @return  None
 */
void UBLEBcastScan_eventProxy(void)
{
  UBLEBcastScan_enqueueMsg(UBS_EVT_MICROBLESTACK, 0);
}

/*********************************************************************
 * @fn      UBLEBcastScan_initBcast
 *
 * @brief   Initialize broadcaster functionality
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_initBcast(void)
{
  ugapBcastCBs_t bcastCBs = {
    UBLEBcastScan_bcast_stateChangeCB,
    UBLEBcastScan_bcast_advPrepareCB,
    UBLEBcastScan_bcast_advDoneCB };

  /* Initilaize Micro GAP Broadcaster Role */
  if (SUCCESS == ugap_bcastInit(&bcastCBs))
  {
    uint8 tempPower;

    Display_print0(dispHandle, UBS_ROW_STATUS_2, 0, "Bcast initialized");

    uble_getParameter(UBLE_PARAM_TXPOWER, &tempPower);
    eddystoneUID.rangingData = tempPower;
    eddystoneURL.txPower = tempPower;

    currentFrameType = EDDYSTONE_FRAME_TYPE_UID;

    UBLEBcastScan_updateAdvDataWithFrame(currentFrameType);

    return true;
  }

  Display_print0(dispHandle, UBS_ROW_STATUS_2, 0, "Bcast init failed");

  return false;
}

#if defined(FEATURE_OBSERVER)
/*********************************************************************
 * @fn      UBLEBcastScan_initObserver
 *
 * @brief   Initialze observer functionality
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_initObserver(void)
{
  ugapObserverScanCBs_t observerCBs = {
    UBLEBcastScan_scan_stateChangeCB,
    UBLEBcastScan_scan_indicationCB,
    UBLEBcastScan_scan_windowCompleteCB };

  /* Initialize Micro GAP Observer Role */
  if (SUCCESS == ugap_scanInit(&observerCBs))
  {
    Display_print0(dispHandle, UBS_ROW_STATUS_2, 0, "Observer initialized");

    /* This is the spot to do scan request without using the keypress control */
    ugap_scanRequest(UBLE_ADV_CHAN_ALL, 160, 320);

    return true;
  }

  Display_print0(dispHandle, UBS_ROW_STATUS_2, 0, "Observer init failed");

  return false;
}
#endif /* FEATURE_OBSERVER */

/*********************************************************************
 * @fn      UBLEBcastScan_initPostProcess
 *
 * @brief   Handle the post initialization process of the micro stack
 *
 * @param   status - status of micro stack init.
 * @param   pAddrType - pointer to address type string.
 *
 * @return  status - true if successful false otherwise
 */
static bool UBLEBcastScan_initPostProcess(bStatus_t status, uint8* pAddrType)
{
  bool bSuccess = false;

  Display_clearLines(dispHandle, UBS_ROW_STATUS_1, UBS_ROW_STATUS_2);

  if (status == SUCCESS)
  {
    uint8 bdAddr[B_ADDR_LEN];

    uble_getAddr(UBLE_ADDRTYPE_BD, bdAddr);

    Display_print1(dispHandle, UBS_ROW_RESULT, 0, "Init\'ed w/ %s", pAddrType);
    Display_print1(dispHandle, UBS_ROW_STATUS_1, 0,
                   "BDAddr=%s", Util_convertBdAddr2Str(bdAddr));

    bSuccess = UBLEBcastScan_initBcast();

  }
  else
  {
    Display_print0(dispHandle, UBS_ROW_RESULT, 0, "Stack init failed");
  }

#if defined(FEATURE_OBSERVER)
  bSuccess = UBLEBcastScan_initObserver();
#endif /* FEATURE_OBSERVER */

  return bSuccess;
}

/*********************************************************************
 * @fn      UBLEBcastScan_displayOutOfRange
 *
 * @brief   Handle the edge case of increment decrement operations
 *
 * @param   inc_dec - increment or decremented
 *
 * @return  None
 */
void UBLEBcastScan_displayOutOfRange(uint8 inc_dec)
{
  Display_print1(dispHandle, UBS_ROW_RESULT, 0, "No more %sc allowed",
                 inc_dec ? "de" : "in");
}

/*********************************************************************
 * @fn      UBLEBcastScan_doTxPower
 *
 * @brief   Increment/Decrement the TX Power level
 *
 * @param   index - index of the menu item used to determine increment/decrement
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doTxPower(uint8 index)
{
  uint8 i;
  bool bSuccess = false;

  /* Get the index of the current TX Power in the power table */
  for (i = 0; i < ubTxPowerTable.numTxPowerVal; i++)
  {
    if (ubTxPowerTable.pTxPowerVals[i].dBm == ubsTxPower)
    {
      break;
    }
  }

  Display_clearLines(dispHandle, UBS_ROW_STATUS_1, UBS_ROW_STATUS_1);

  if (index == 0) /* Increment */
  {
    if (i < ubTxPowerTable.numTxPowerVal - 1) /* If not maximum already? */
    {
      ubsTxPower = ubTxPowerTable.pTxPowerVals[i + 1].dBm;
      bSuccess = true;
    }
  }
  else /* index == 1, Decrement */
  {
    if (i > 0) /* If not minimum already? */
    {
      ubsTxPower = ubTxPowerTable.pTxPowerVals[i - 1].dBm;
      bSuccess = true;
    }
  }

  if (bSuccess)
  {
    if (SUCCESS == uble_setParameter(UBLE_PARAM_TXPOWER,
                                     sizeof(ubsTxPower), &ubsTxPower))
    {
      Display_print1(dispHandle, UBS_ROW_RESULT, 0,
                     "TXPower: %d dBm", ubsTxPower);

      eddystoneUID.rangingData = ubsTxPower;
      eddystoneURL.txPower = ubsTxPower;
    }
    else
    {
      bSuccess = false;
      Display_print0(dispHandle, UBS_ROW_RESULT, 0, "TXPower failed");
      /* Restore ubsTxPower */
      uble_getParameter(UBLE_PARAM_TXPOWER, &ubsTxPower);
    }
  }
  else
  {
    UBLEBcastScan_displayOutOfRange(index);
  }

  return bSuccess;
}

/*********************************************************************
 * @fn      UBLEBcastScan_doAdvInterval
 *
 * @brief   Increment/Decrement the advertising interval
 *
 * @param   index - index of the menu item used to determine increment/decrement
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doAdvInterval(uint8 index)
{
  bool bSuccess = false;

  Display_clearLines(dispHandle, UBS_ROW_STATUS_1, UBS_ROW_STATUS_1);

  if (index == 0) /* Increment */
  {
    if (ubsAdvInterval <= UBLE_MAX_ADV_INTERVAL - 160)
    {
      ubsAdvInterval += 160;  /* Add 100 ms */
      bSuccess = true;
    }
  }
  else /* index == 1, Decrement */
  {
    if (ubsAdvInterval >= UBLE_MIN_ADV_INTERVAL + 160)
    {
      ubsAdvInterval -= 160;  /* Subtract 100 ms */
      bSuccess = true;
    }
  }

  if (bSuccess)
  {
    if (SUCCESS == uble_setParameter(UBLE_PARAM_ADVINTERVAL,
                                     sizeof(ubsAdvInterval), &ubsAdvInterval))
    {
      Display_print1(dispHandle, UBS_ROW_RESULT, 0,
                     "AdvInterval: %d", ubsAdvInterval);
    }
    else
    {
      bSuccess = false;
      Display_print0(dispHandle, UBS_ROW_RESULT, 0, "AdvInterval Failed");
      /* Restore ubsAdvInterval */
      uble_getParameter(UBLE_PARAM_ADVINTERVAL, &ubsAdvInterval);
    }
  }
  else
  {
    UBLEBcastScan_displayOutOfRange(index);
  }

  return bSuccess;
}

/*********************************************************************
 * @fn      UBLEBcastScan_doAdvChanMap
 *
 * @brief   Enable/disable various advertising channels (37, 38, 39)
 *
 * @param   index - index of the menu item used to determine increment/decrement
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doAdvChanMap(uint8 index)
{
  bool bSuccess = false;

  Display_clearLines(dispHandle, UBS_ROW_STATUS_1, UBS_ROW_STATUS_1);

  if (index == 0) /* Increment */
  {
    if (ubsAdvChanMap < UBLE_MAX_CHANNEL_MAP)
    {
      ubsAdvChanMap++;
      bSuccess = true;
    }
  }
  else /* index == 1, Decrement */
  {
    if (ubsAdvChanMap > UBLE_MIN_CHANNEL_MAP)
    {
      ubsAdvChanMap--;
      bSuccess = true;
    }
  }

  if (bSuccess)
  {
    if (SUCCESS == uble_setParameter(UBLE_PARAM_ADVCHANMAP,
                                     sizeof(ubsAdvChanMap), &ubsAdvChanMap))
    {
      Display_print3(dispHandle, UBS_ROW_RESULT, 0, "AdvChan: %s %s %s",
                     (ubsAdvChanMap & UBLE_ADV_CHAN_37) ? "37" : "  ",
                     (ubsAdvChanMap & UBLE_ADV_CHAN_38) ? "38" : "  ",
                     (ubsAdvChanMap & UBLE_ADV_CHAN_39) ? "39" : "  ");
    }
    else
    {
      bSuccess = false;
      Display_print0(dispHandle, UBS_ROW_RESULT, 0, "AdvChan failed");
      /* Restore ubsAdvChanMap */
      uble_getParameter(UBLE_PARAM_ADVCHANMAP, &ubsAdvChanMap);
    }
  }
  else
  {
    UBLEBcastScan_displayOutOfRange(index);
  }

  return bSuccess;
}

/*********************************************************************
 * @fn      UBLEBcastScan_doTimeToPrepare
 *
 * @brief   Increment/Decrement the time when the application is
 *          notified before an advertisement event
 *
 * @param   index - index of the menu item used to determine increment/decrement
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doTimeToPrepare(uint8 index)
{
  bool bSuccess = false;

  Display_clearLines(dispHandle, UBS_ROW_STATUS_1, UBS_ROW_STATUS_1);

  if (index == 0) /* Increment */
  {
    if (ubsTimeToAdv < 254)
    {
      ubsTimeToAdv += 2;
      bSuccess = true;
    }
  }
  else /* index == 1, Decrement */
  {
    if (ubsTimeToAdv > 1)
    {
      ubsTimeToAdv -= 2;
      bSuccess = true;
    }
  }

  if (bSuccess)
  {
    if (SUCCESS == uble_setParameter(UBLE_PARAM_TIMETOADV,
                                     sizeof(ubsTimeToAdv), &ubsTimeToAdv))
    {
      Display_print1(dispHandle, UBS_ROW_RESULT, 0,
                     "TimeToPrepare: %d ms", ubsTimeToAdv);
    }
    else
    {
      bSuccess = false;
      Display_print0(dispHandle, UBS_ROW_RESULT, 0, "TimeToPrepare failed");
      /* Restore ubsTimeToAdv */
      uble_getParameter(UBLE_PARAM_TIMETOADV, &ubsTimeToAdv);
    }
  }
  else
  {
    UBLEBcastScan_displayOutOfRange(index);
  }

  return bSuccess;
}

/*********************************************************************
 * @fn      UBLEBcastScan_doSetFrameType
 *
 * @brief   Update advertisement data with a given Eddystone frame type
 *
 * @param   index - index of the menu item used to frame offset
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doSetFrameType(uint8 index)
{
  static uint8* pType[3] = { "UID", "URL", "TLM" };

  /* Items for this action starts at ROW 2 in ubsMenuAdvData */
  index--;

  Display_print2(dispHandle, UBS_ROW_RESULT, 0,
                 "%sEddystone %s!", (bUpdateUponEvent)? "To be " : "",
                 pType[index]);

  currentFrameType = index << 4;

  if (!bUpdateUponEvent)
  {
    UBLEBcastScan_bcast_updateAdvData();
  }

  return true;
}

/*********************************************************************
 * @fn      UBLEBcastScan_doUpdateOption
 *
 * @brief   Toggle the update of the advertisement data to be immediate
 *          or on an event.
 *
 * @param   index - Used to choose whether immediate update or at event
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doUpdateOption(uint8 index)
{
  Display_print0(dispHandle, UBS_ROW_RESULT, 0,
                 (index == 0) ? "Immediate Change" : "Change upon Event");

  bUpdateUponEvent = (index == 0) ? false : true;

  /* Go to ubsMenuAdvData object */
  tbm_goTo(&ubsMenuAdvData);

  return true;
}

/*********************************************************************
 * @fn      UBLEBcastScan_doBcastDutyOnTime
 *
 * @brief   Change the "on" duty cycle of the broadcast
 *
 * @param   index - index of the menu item used to determine increment/decrement
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doBcastDutyOnTime(uint8 index)
{
  bool bSuccess = false;
  uint16 currentDutyOn = ubsBcastDutyOn;

  Display_clearLines(dispHandle, UBS_ROW_STATUS_1, UBS_ROW_STATUS_1);

  if (index == 0) /* Increment */
  {
    if (ubsBcastDutyOn <= 65535 - 5)
    {
      ubsBcastDutyOn += 5;  /* Add 500 ms */
      bSuccess = true;
    }
  }
  else /* index == 1, Decrement */
  {
    if (ubsBcastDutyOn >= 5)
    {
      ubsBcastDutyOn -= 5;  /* Subsract 500 ms */
      bSuccess = true;
    }
  }

  if (bSuccess)
  {
    if (SUCCESS == ugap_bcastSetDuty(ubsBcastDutyOn, ubsBcastDutyOff))
    {
      Display_print1(dispHandle, UBS_ROW_RESULT, 0, "DutyOnTime: %d ms",
                     ubsBcastDutyOn * 100);
    }
    else
    {
      bSuccess = false;
      Display_print0(dispHandle, UBS_ROW_RESULT, 0, "DutyOnTime failed");
      /* Restore ubsAdvInterval */
      ubsBcastDutyOn = currentDutyOn;
    }
  }
  else
  {
    UBLEBcastScan_displayOutOfRange(index);
  }

  return bSuccess;
}

/*********************************************************************
 * @fn      UBLEBcastScan_doBcastDutyOffTime
 *
 * @brief   Change the "off" duty cycle of the broadcast
 *
 * @param   index - index of the menu item used to determine increment/decrement
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doBcastDutyOffTime(uint8 index)
{
  bool bSuccess = false;
  uint16 currentDutyOff = ubsBcastDutyOff;

  Display_clearLines(dispHandle, UBS_ROW_STATUS_1, UBS_ROW_STATUS_1);

  if (index == 0) /* Increment */
  {
    if (ubsBcastDutyOff <= 65535 - 5)
    {
      ubsBcastDutyOff += 5;  /* Add 500 ms */
      bSuccess = true;
    }
  }
  else /* index == 1, Decrement */
  {
    if (ubsBcastDutyOff >= 5)
    {
      ubsBcastDutyOff -= 5;  /* Subtract 500 ms */
      bSuccess = true;
    }
  }

  if (bSuccess)
  {
    if (SUCCESS == ugap_bcastSetDuty(ubsBcastDutyOn, ubsBcastDutyOff))
    {
      Display_print1(dispHandle, UBS_ROW_RESULT, 0,
                     "DutyOffTime: %d ms",
                     ubsBcastDutyOff * 100);

      if (ubsBcastDutyOff == 0)
      {
        Display_print0(dispHandle, UBS_ROW_STATUS_1, 0, "DutyControl disabled");
      }
    }
    else
    {
      bSuccess = false;
      Display_print0(dispHandle, UBS_ROW_RESULT, 0, "DutyOffTime failed");
      /* Restore ubsAdvInterval */
      ubsBcastDutyOff = currentDutyOff;
    }
  }
  else
  {
    UBLEBcastScan_displayOutOfRange(index);
  }

  return bSuccess;
}

/*********************************************************************
 * @fn      UBLEBcastScan_doBcastStart
 *
 * @brief   Start broadcasting
 *
 * @param   index - Used to select number of adv vs indefinite
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doBcastStart(uint8 index)
{
  bool bSuccess = false;
  uint16 numAdv = index ? 100 : 0;

  Display_clearLines(dispHandle, UBS_ROW_STATUS_1, UBS_ROW_STATUS_2);
  Display_clearLines(dispHandle, UBS_ROW_MONITOR_STATE, UBS_ROW_MONITOR_3);

  if (SUCCESS == ugap_bcastStart(numAdv))
  {
    if (numAdv == 0)
    {
      bSuccess = true;
      Display_print0(dispHandle, UBS_ROW_RESULT, 0, "Bcast - indefinitely");
    }
    else
    {
      Display_print1(dispHandle, UBS_ROW_RESULT, 0, "Bcast - %d Adv\'s", numAdv);
    }

    /* Enable stop action and disable start actions  */
    tbm_setItemStatus(&ubsMenuBcastControl, TBM_ITEM_2,
                      TBM_ITEM_0 | TBM_ITEM_1);
  }
  else
  {
    Display_print0(dispHandle, UBS_ROW_RESULT, 0, "Bcast start failed");
  }

  return bSuccess;
}

/*********************************************************************
 * @fn      UBLEBcastScan_doBcastStop
 *
 * @brief   Stop broadcasting
 *
 * @param   index - unused
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doBcastStop(uint8 index)
{
  bool bSuccess = false;

  (void) index;

  Display_clearLines(dispHandle, UBS_ROW_STATUS_1, UBS_ROW_BCAST_STATE);

  if (SUCCESS == ugap_bcastStop())
  {
    bSuccess = true;
    Display_print0(dispHandle, UBS_ROW_RESULT, 0, "Bcast - stopped");

    /* Enable start actions */
    tbm_setItemStatus(&ubsMenuBcastControl,
                      TBM_ITEM_0 | TBM_ITEM_1, TBM_ITEM_2);
  }
  else
  {
    Display_print0(dispHandle, UBS_ROW_RESULT, 0, "Bcast stop failed");
  }

  return bSuccess;
}

#if defined(FEATURE_OBSERVER)
/*********************************************************************
 * @fn      UBLEBcastScan_scan_stateChangeCB
 *
 * @brief   Callback from Micro Observer indicating a state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void UBLEBcastScan_scan_stateChangeCB(ugapObserverScan_State_t newState)
{
  switch (newState)
  {
  case UGAP_SCAN_STATE_INITIALIZED:
    Display_print0(dispHandle, UBS_ROW_SCAN_STATE, 0,
                   "Observer_State: Initialized");
    break;

  case UGAP_SCAN_STATE_IDLE:
    Display_print0(dispHandle, UBS_ROW_SCAN_STATE, 0,
                   "Observer_State: Idle");
    break;

  case UGAP_SCAN_STATE_SCANNING:
    Display_print0(dispHandle, UBS_ROW_SCAN_STATE, 0,
                   "Observer_State: Scanning");
    break;

  case UGAP_SCAN_STATE_WAITING:
    Display_print0(dispHandle, UBS_ROW_SCAN_STATE, 0,
                   "Observer_State: Waiting");
    break;

  case UGAP_SCAN_STATE_SUSPENDED:
    Display_print0(dispHandle, UBS_ROW_SCAN_STATE, 0,
                   "Observer_State: Suspended");
    break;

  default:
    break;
  }
}

/*********************************************************************
 * @fn      UBLEBcastScan_scan_indicationCB
 *
 * @brief   Callback from Micro observer notifying that a advertising
 *          packet is received.
 *
 * @param   status status of a scan
 * @param   len length of the payload
 * @param   pPayload pointer to payload
 *
 * @return  None.
 */
static void UBLEBcastScan_scan_indicationCB(bStatus_t status, uint8_t len,
                                              uint8_t *pPayload)
{
  static char  *devAddr;
  static uint8  chan;
  static uint32 timeStamp;
  static uint32 rxBufFull = 0;

  /* We have an advertisment packe:
   *
   * | Preamble  | Access Addr | PDU         | CRC     |
   * | 1-2 bytes | 4 bytes     | 2-257 bytes | 3 bytes |
   *
   * The PDU is expended to:
   * | Header  | Payload     |
   * | 2 bytes | 1-255 bytes |
   *
   * The Header is expended to:
   * | PDU Type...RxAdd | Length |
   * | 1 byte           | 1 byte |
   *
   * The Payload is expended to:
   * | AdvA    | AdvData    |
   * | 6 bytes | 0-31 bytes |
   *
   * The radio stripps the CRC and replaces it with the postfix.
   *
   * The Postfix is expended to:
   * | RSSI   | Status | TimeStamp |
   * | 1 byte | 1 byte | 4 bytes   |
   *
   * The Status is expended to:
   * | bCrcErr | bIgnore | channel  |
   * | bit 7   | bit 6   | bit 5..0 |
   *
   * Note the advPke is the beginning of PDU; the dataLen includes
   * the postfix length.
   *
   */
  if (status == SUCCESS)
  {
    devAddr = Util_convertBdAddr2Str(pPayload + 2);
    chan = (*(pPayload + len - 5) & 0x3F);
    timeStamp = *(uint32 *)(pPayload + len - 4);

    Display_print3(dispHandle, UBS_ROW_SCAN_1, 0,
                    "ScanInd DevAddr: %s Chan: %d timeStamp: %ul",
                    devAddr, chan, timeStamp);
  }
  else
  {
    /* Rx buffer is full */
    Display_print1(dispHandle, UBS_ROW_SCAN_3, 0, "Rx Buffer Full: %d",
                    ++rxBufFull);
  }
}

/*********************************************************************
 * @fn      UBLEBcastScan_scan_windowCompleteCB
 *
 * @brief   Callback from Micro Broadcaster notifying that a
 *          scan window is completed.
 *
 * @param   status - How the last event was done. SUCCESS or FAILURE.
 *
 * @return  None.
 */
static void UBLEBcastScan_scan_windowCompleteCB (bStatus_t status)
{
  scanIndCount++;

  Display_print1(dispHandle, UBS_ROW_SCAN_2, 0,
                 "%d Scan Window\'s done", scanIndCount);
}
#endif /* FEATURE_OBSERVER */

/*********************************************************************
 *********************************************************************/
