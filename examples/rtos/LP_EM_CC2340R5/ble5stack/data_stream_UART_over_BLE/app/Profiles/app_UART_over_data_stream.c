/******************************************************************************

@file  app_data.c

@brief This file contains the Data Stream application functionality.

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
#include <time.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
#include <ti/bleapp/profiles/data_stream/data_stream_profile.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <app_main.h>

//*****************************************************************************
//! Defines
//*****************************************************************************
#define DS_CCC_UPDATE_NOTIFICATION_ENABLED  1
#define UART_MAX_READ_SIZE    (128)

//*****************************************************************************
//! Globals
//*****************************************************************************
UART2_Handle uart;
UART2_Params uartParams;
// UART read buffer
static uint8_t uartReadBuffer[UART_MAX_READ_SIZE] = {0};
static uint8_t BufferSize = 0;

//*****************************************************************************
//!LOCAL FUNCTIONS
//*****************************************************************************

static void DS_onCccUpdateCB( uint16 connHandle, uint16 pValue );
static void DS_incomingDataCB( uint16 connHandle, char *pValue, uint16 len );

//*****************************************************************************
//!APPLICATION CALLBACK
//*****************************************************************************
// Data Stream application callback function for incoming data
static DSP_cb_t ds_profileCB =
{
  DS_onCccUpdateCB,
  DS_incomingDataCB
};

//*****************************************************************************
//! Functions
//*****************************************************************************
/*********************************************************************
 * @fn      DS_onCccUpdateCB
 *
 * @brief   Callback from Data_Stream_Profile indicating ccc update
 *
 * @param   cccUpdate - pointer to data structure used to store ccc update
 *
 * @return  SUCCESS or stack call status
 */
static void DS_onCccUpdateCB( uint16 connHandle, uint16 pValue )
{
 
}

/*********************************************************************
 * @fn      DS_incomingDataCB
 *
 * @brief   Callback from Data_Stream_Profile indicating incoming data
 *
 * @param   dataIn - pointer to data structure used to store incoming data
 *
 * @return  SUCCESS or stack call status
 */
static void DS_incomingDataCB( uint16 connHandle, char *pValue, uint16 len )
{
  bStatus_t status = SUCCESS;
  char dataOut[] = "Data size is too long";
  char printData[len+1];
  uint16 i = 0;

  // Toggle LEDs to indicate that data was received
  GPIO_toggle( CONFIG_GPIO_LED_RED );
  GPIO_toggle( CONFIG_GPIO_LED_GREEN );

  // The incoming data length was too large
  if ( len == 0 )
  {
    // Send error message over GATT notification
    status = DSP_sendData( (uint8 *)dataOut, sizeof( dataOut ) );
  }

  // New data received from peer device
  else
  {
    // Copy the incoming data to buffer before printing it
    memcpy (printData, pValue, len );
    printData[len] ='\0';

   //Write incoming data processing here
    UART2_write(uart,printData, len, NULL);
    UART2_write(uart, "\n\r", 2, NULL);
  }
}

void SendUARTOverBLE(){
    DSP_sendData( (uint8 *)uartReadBuffer, BufferSize );
    UART2_read(uart, uartReadBuffer, UART_MAX_READ_SIZE, NULL);
}

/*
 *  ======== callbackFxn ========
 */
void callbackFxn(UART2_Handle handle, void *buffer, size_t count, void *userArg, int_fast16_t status)
{
    BufferSize = count;
    BLEAppUtil_invokeFunctionNoData(SendUARTOverBLE);
}

/*********************************************************************
 * @fn      DataStream_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Data Stream profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t DataStream_start( void )
{
  bStatus_t status = SUCCESS;

  status = DSP_start( &ds_profileCB );
  if( status != SUCCESS )
  {
    // Return status value
    return status;
  }

  /* Create a UART in CALLBACK read mode */
  UART2_Params_init(&uartParams);
  uartParams.readMode     = UART2_Mode_CALLBACK;
  uartParams.readCallback = callbackFxn;
  uartParams.baudRate     = 115200;

  uart = UART2_open(CONFIG_UART2_0, &uartParams);

  UART2_write(uart,"DataStream with UART\n\r", 22, NULL);

  if (uart == NULL)
      {
          /* UART2_open() failed */
          while (1) {}
      }

  UART2_read(uart, uartReadBuffer, UART_MAX_READ_SIZE, NULL);

  // Set LEDs
  GPIO_write( CONFIG_GPIO_LED_RED, CONFIG_LED_OFF );
  GPIO_write( CONFIG_GPIO_LED_GREEN, CONFIG_LED_ON );

  return ( SUCCESS );
}
