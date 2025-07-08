/******************************************************************************

@file  app_audio_stream.c

@brief This example file demonstrates how to activate the peripheral role with
the help of BLEAppUtil APIs.

Two structures are used for event handling, one for connection events and one
for advertising events.
In each, eventMask is used to specify the events that will be received
and handled.
In addition, fill the BLEAppUtil_AdvInit_t structure with variables generated
by the Sysconfig.

In the events handler functions, write what actions are done after each event.
In this example, after a connection is made, activation is performed for
re-advertising up to the maximum connections.

In the Peripheral_start() function at the bottom of the file, registration,
initialization and activation are done using the BLEAppUtil API functions,
using the structures defined in the file.

More details on the functions and structures can be seen next to the usage.

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2024, Texas Instruments Incorporated
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

#include <string.h>
#include <time.h>
#include <ti/drivers/GPIO.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>
#include "audio_stream_profile.h"

//*****************************************************************************
//! Defines
//*****************************************************************************
#define AS_CCC_UPDATE_NOTIFICATION_ENABLED  1

//*****************************************************************************
//! Globals
//*****************************************************************************
extern void Application_onSystemInitialized(void);
extern void Application_onBLEConnectionEstablished(void);
extern void Application_onBLEConnectionTerminated(void);
extern void Application_onBLEStreamEnabled(void);
extern void Application_onBLEStreamDisabled(void);
extern void Application_onUARTDataReception(uint8_t *buffer, uint16_t size);
extern void Application_onBLEDataReception(uint8_t *buffer, uint16_t size);
extern void Application_onLeftButtonPress(void);
extern void Application_onRightButtonPress(void);
extern void Application_DoAudioEvent();
//*****************************************************************************
//!LOCAL FUNCTIONS
//*****************************************************************************

static void AS_onCccUpdateCB( uint16 connHandle, uint16 pValue );
static void AS_incomingDataCB( uint16 connHandle, char *pValue, uint16 len );

//*****************************************************************************
//!APPLICATION CALLBACK
//*****************************************************************************
// Data Stream application callback function for incoming data
static ASP_cb_t as_profileCB =
{
  AS_onCccUpdateCB,
  AS_incomingDataCB
};

//*****************************************************************************
//! Functions
//*****************************************************************************
/*********************************************************************
 * @fn      AS_onCccUpdateCB
 *
 * @brief   Callback from Data_Stream_Profile indicating ccc update
 *
 * @param   cccUpdate - pointer to data structure used to store ccc update
 *
 * @return  SUCCESS or stack call status
 */
static void AS_onCccUpdateCB( uint16 connHandle, uint16 pValue )
{
  if ( pValue == AS_CCC_UPDATE_NOTIFICATION_ENABLED)
  {
    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                      "AudioStream status: CCC Update - connectionHandle: "
                      MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                      "Notifications enabled", connHandle);
    Application_onBLEStreamEnabled();
  }
  else
  {
    MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0,
                      "AudioStream status: CCC Update - connectionHandle: "
                      MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                      "Notifications disabled", connHandle);
    Application_onBLEStreamDisabled();
  }
}

/*********************************************************************
 * @fn      AS_incomingDataCB
 *
 * @brief   Callback from Data_Stream_Profile indicating incoming data
 *
 * @param   dataIn - pointer to data structure used to store incoming data
 *
 * @return  SUCCESS or stack call status
 */
static void AS_incomingDataCB( uint16 connHandle, char *pValue, uint16 len )
{
  bStatus_t status = SUCCESS;
  char dataOut[] = "Data size is too long";
  char printData[len+1];
  uint16 i = 0;

  // The incoming data length was too large
  if ( len == 0 )
  {
    // Send error message over GATT notification
    status = ASP_sendData( (uint8 *)dataOut, sizeof( dataOut ) );
  }

  // New data received from peer device
  else
  {
    memcpy (printData, pValue, len );
    printData[len] ='\0';
    // Change upper case to lower case and lower case to upper case
    for ( i = 0; i < len; i++ )
    {
      if ( pValue[i] >= 'a' && pValue[i] <= 'z' )
      {
        pValue[i] = pValue[i] - 32;
      }
      else if ( pValue[i] >= 'A' && pValue[i] <= 'Z' )
      {
        pValue[i] = pValue[i] + 32;
      }
    }

    // Echo the incoming data over GATT notification
    status = ASP_sendData( (uint8 *)pValue, len );
    if ( status == SUCCESS )
    {
      // Copy the changed data to buffer before printing it
      memcpy (printData, pValue, len );
    }
  }
}

/*********************************************************************
 * @fn      AudioStream_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Data Stream profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t AudioStream_start( void )
{
  bStatus_t status = SUCCESS;

  status = ASP_start( &as_profileCB );
  if( status != SUCCESS )
  {
    // Return status value
    return status;
  }

  return ( SUCCESS );
}
