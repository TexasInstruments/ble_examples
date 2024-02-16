/******************************************************************************

@file  app_dev_info.c

@brief This file contains the device info application functionality

Group: WCS, BTS
$Target Device: DEVICES $

******************************************************************************
$License: BSD3 2022 $
******************************************************************************
$Release Name: PACKAGE NAME $
$Release Date: PACKAGE RELEASE DATE $
*****************************************************************************/

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/services/dev_info/dev_info_service.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
//*****************************************************************************

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      DevInfo_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Device Info service.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t DevInfo_start(void)
{
  bStatus_t status = SUCCESS;
  uint8_t  *devAddr = NULL;
  uint8_t systemId[DEVINFO_SYSTEM_ID_LEN] = {0};

  // Device Information Service
  status = DevInfo_addService();
  if ( status != SUCCESS )
  {
    // Return status value
    return( status );
  }

  devAddr = GAP_GetDevAddress( TRUE );

  memcpy( &systemId[0], &devAddr[0], 3 );   // use 6 bytes of device address for 8 bytes of system ID value
  memset( &systemId[3], 0xFFFE, 2 );        // set middle bytes to 0xFFFE
  memcpy( &systemId[5], &devAddr[3], 3 );   // shift three bytes up

  // Set Device Info Service System ID Parameter
  status = DevInfo_setParameter( DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId );
  if ( status != SUCCESS )
  {
    // Return status value
    return ( status );
  }

  // Set Device Info Service Manufacturer Name Parameter
  status = DevInfo_setParameter( DEVINFO_MANUFACTURER_NAME, DEVINFO_STR_ATTR_LEN, "Texas Instruments" );

  // Return status value
  return ( status );
}
