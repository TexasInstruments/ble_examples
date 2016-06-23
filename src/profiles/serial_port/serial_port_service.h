/**************************************************************************************************
  Filename:       SerialPortService.h
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Serial Port Profile definitions and
                  prototypes.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

#ifndef SERIALPORTSERVICE_H
#define SERIALPORTSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>   
   
   
/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
 
#define SERIALPORTSERVICE_CHAR_DATA             0  // RW uint8 - Profile Characteristic 2 value
#define SERIALPORTSERVICE_CHAR_STATUS           1  // R uint8 - Profile Characteristic 3 value
#define SERIALPORTSERVICE_CHAR_CONFIG           2  // RW uint8 - Profile Characteristic 4 value

#define SERIALPORTSERVICE_SET_UART_CONFIG       3  // W uint8 - Profile SET_UART_CONFIG value
#define SERIALPORTSERVICE_GET_UART_CONFIG       4  // R uint8 - Profile GET_UART_CONFIG value
   
// Serial Port Service UUID
#define SERIALPORTSERVICE_SERV_UUID             0xC0E0
    
// Key Pressed UUID
#define SERIALPORTSERVICE_DATA_UUID             0xC0E1
#define SERIALPORTSERVICE_STATUS_UUID           0xC0E2
#define SERIALPORTSERVICE_CONFIG_UUID           0xC0E3

// Serial Port Profile Services bit fields
#define SERIALPORTSERVICE_SERVICE               0x00000001

// Length of Status Characteristic in bytes
#define SERIALPORTSERVICE_STATUS_LEN            7

// Length of Config Characteristic in bytes
#define SERIALPORTSERVICE_CONFIG_LEN            3
  
//Length of Data Characteristic in bytes
#define SERIALPORTSERVICE_DATA_LEN              128 //20  
   
#define SERVAPP_NUM_ATTR_SUPPORTED              11   
extern gattAttribute_t SerialPortServiceAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED];      
extern uint8 SerialPortServiceData[SERIALPORTSERVICE_DATA_LEN];

/*********************************************************************
 * TYPEDEFS
 */

  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*SerialPortServiceChange_t)( uint8 paramID );

typedef struct
{
  SerialPortServiceChange_t        pfnSerialPortServiceChange;  // Called when characteristic value changes
} SerialPortServiceCBs_t;

    
/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * SerialPortService_AddService- Initializes the Serial Port Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t SerialPortService_AddService( uint32 services );

/*
 * SerialPortService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t SerialPortService_RegisterAppCBs( SerialPortServiceCBs_t *appCallbacks );

/*
 * SerialPortService_SetParameter - Set a Serial Port Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t SerialPortService_SetParameter( uint8 param, uint8 len, void *value );

/*********************************************************************
 * @fn      SerialPortService_AddStatusErrorCount
 *
 * @brief   Set a Serial Port Profile parameter.
 *
 * @param   errorCode - UART_Status error code 
 *
 * @return  bStatus_t
 */
extern bStatus_t SerialPortService_AddStatusErrorCount( UART_Status errorCode );

extern bStatus_t SerialPortService_AddStatusTXBytes( uint16 count );
extern bStatus_t SerialPortService_AddStatusRXBytes( uint16 count );
extern bStatus_t SerialPortService_GetUartConfig( UART_Params *params );  
extern bStatus_t SerialPortService_SetUartConfig( UART_Params *params );

/*
 * SerialPortService_GetParameter - Get a Serial Port Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t SerialPortService_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SERIALPORTSERVICE_H */
