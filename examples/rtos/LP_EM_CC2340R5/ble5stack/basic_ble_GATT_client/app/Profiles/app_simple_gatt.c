/******************************************************************************

@file  app_simple_gatt.c

@brief This file contains the Simple GATT application functionality

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2023, Texas Instruments Incorporated
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

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/profiles/simple_gatt/simple_gatt_profile.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
//*****************************************************************************

static void SimpleGatt_changeCB( uint8_t paramId );
void SimpleGatt_notifyChar4();

// Simple GATT Profile Callbacks
static SimpleGattProfile_CBs_t simpleGatt_profileCBs =
{
  SimpleGatt_changeCB // Simple GATT Characteristic value change callback
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      SimpleGatt_ChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void SimpleGatt_changeCB( uint8_t paramId )
{
  uint8_t newValue = 0;

  switch( paramId )
  {
    case SIMPLEGATTPROFILE_CHAR1:
      {
        SimpleGattProfile_getParameter( SIMPLEGATTPROFILE_CHAR1, &newValue );

        // Print the new value of char 1
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "Profile status: Simple profile - "
                          "Char 1 value = " MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                          newValue);
      }
      break;

    case SIMPLEGATTPROFILE_CHAR3:
      {
        SimpleGattProfile_getParameter(SIMPLEGATTPROFILE_CHAR3, &newValue);

        // Print the new value of char 3
        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "Profile status: Simple profile - "
                          "Char 3 value = " MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                          newValue);

        SimpleGatt_notifyChar4();
      }
      break;
    case SIMPLEGATTPROFILE_CHAR4:
      {
          // Print Notification registration to user
          MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "Profile status: Simple profile - "
                                    "Char 4 = Notification registration");

          SimpleGatt_notifyChar4();
          break;
      }
    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      SimpleGatt_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Simple GATT profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t SimpleGatt_start( void )
{
  bStatus_t status = SUCCESS;

  // Add Simple GATT service
  status = SimpleGattProfile_addService();
  if(status != SUCCESS)
  {
	// Return status value
    return(status);
  }

  // Setup the Simple GATT Characteristic Values
  // For more information, see the GATT and GATTServApp sections in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {
    uint8_t charValue1 = 1;
    uint8_t charValue2 = 2;
    uint8_t charValue3 = 3;
    uint8_t charValue4 = 4;
    uint8_t charValue5[SIMPLEGATTPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

    SimpleGattProfile_setParameter( SIMPLEGATTPROFILE_CHAR1, sizeof(uint8_t),
                                    &charValue1 );
    SimpleGattProfile_setParameter( SIMPLEGATTPROFILE_CHAR2, sizeof(uint8_t),
                                    &charValue2 );
    SimpleGattProfile_setParameter( SIMPLEGATTPROFILE_CHAR3, sizeof(uint8_t),
                                    &charValue3 );
    SimpleGattProfile_setParameter( SIMPLEGATTPROFILE_CHAR4, sizeof(uint8_t),
                                    &charValue4 );
    SimpleGattProfile_setParameter( SIMPLEGATTPROFILE_CHAR5, SIMPLEGATTPROFILE_CHAR5_LEN,
                                    charValue5 );
  }
  // Register callback with SimpleGATTprofile
  status = SimpleGattProfile_registerAppCBs( &simpleGatt_profileCBs );

  // Return status value
  return(status);
}

/*********************************************************************
 * @fn      SimpleGatt_notifyChar4
 *
 * @brief   This function is called when WriteReq has been received to Char 4 or to Char 3.
 *          The purpose of this function is to send notification of Char 3 with the value
 *          of Char 3.
 *
 * @return  void
 */
void SimpleGatt_notifyChar4()
{
  uint8_t value;
  if (SimpleGattProfile_getParameter(SIMPLEGATTPROFILE_CHAR3, &value) == SUCCESS)
    {
      // Call to set that value of the fourth characteristic in the profile.
      // Note that if notifications of the fourth characteristic have been
      // enabled by a GATT client device, then a notification will be sent
      // every time there is a change in Char 3 or Char 4.
      SimpleGattProfile_setParameter(SIMPLEGATTPROFILE_CHAR4, sizeof(uint8_t),
                                 &value);
    }
}
