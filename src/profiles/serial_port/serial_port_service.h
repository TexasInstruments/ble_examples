/*
 * Filename: serial_port_service.h
 *
 * Description: This is the simple_peripheral example modified to send
 * data over BLE at a high throughput.
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
