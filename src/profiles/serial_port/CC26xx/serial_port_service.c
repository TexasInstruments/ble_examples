/*
 * Filename: serial_port_service.c
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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#ifdef SDI_USE_UART
#include "inc/sdi_task.h"
#endif

#include "serial_port_service.h"
   
#ifdef SDI_USE_UART
#include "inc/sdi_tl_uart.h"
#endif

#include "ti/drivers/uart/UARTCC26XX.h"
#include "board.h"
#include "spp_ble_server.h"

/*********************************************************************
 * MACROS
 */
#define AUTO_NOTIFICATION FALSE
/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        11

gattAttribute_t SerialPortServiceAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED];   
/*********************************************************************
 * TYPEDEFS
 */

typedef enum UART_STATUS_CHAR {   
  FRAMING_ERR_BYTE,
  PARITY_ERR_BYTE,
  RF_OVERRUN_BYTE
} UART_STATUS_BYTE_DEF;

/*!
 *  @brief    UART config service settings
 *
 *  This enumeration defines the UART config bits in Byte 2 of config. char.
 */
typedef enum UART_CONFIG_CHAR {
    UART_CONFIG_START = 0,        /*!<Start bit level */
    UART_CONFIG_STOP = 1,         /*!< Stop bit level (must be different from start bit level)*/
    UART_CONFIG_STOP_BITS  = 2,   /*!< Number of stop bits */
    UART_CONFIG_PARITY = 3,       /*!< Parity enable  */
    UART_CONFIG_EVEN  = 4,        /*!< Parity level  */
    UART_CONFIG_FLOW = 5          /*!< Flow control enabled  */
} UART_CONFIG_BIT_DEF;
/*********************************************************************

 * GLOBAL VARIABLES
 */
// Serial Port Service Profile Service UUID: 0xC0E0
CONST uint8 SerialPortServUUID[ATT_UUID_SIZE] =
{ 
  TI_BASE_UUID_128(SERIALPORTSERVICE_SERV_UUID)
};
   
// Characteristic Data UUID: 0xC0E1
CONST uint8 SerialPortServiceDataUUID[ATT_UUID_SIZE] =
{ 
  TI_BASE_UUID_128(SERIALPORTSERVICE_DATA_UUID)
};

// Characteristic Status UUID: 0xC0E2
CONST uint8 SerialPortServiceStatusUUID[ATT_UUID_SIZE] =
{ 
  TI_BASE_UUID_128(SERIALPORTSERVICE_STATUS_UUID)
};

// Characteristic Config UUID: 0xC0E3
CONST uint8 SerialPortServiceConfigUUID[ATT_UUID_SIZE] =
{ 
  TI_BASE_UUID_128(SERIALPORTSERVICE_CONFIG_UUID)
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static SerialPortServiceCBs_t *SerialPortService_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Serial Port Profile Service attribute
static CONST gattAttrType_t SerialPortService = { ATT_UUID_SIZE, SerialPortServUUID };

// Serial Port Profile Characteristic Data Properties
static uint8 SerialPortServiceDataProps = GATT_PROP_WRITE_NO_RSP | GATT_PROP_NOTIFY;

// Serial Port Profile Characteristic Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *SerialPortServiceDataConfig;

// Characteristic Data Value
uint8 SerialPortServiceData[SERIALPORTSERVICE_DATA_LEN] = {0,};

// Serial Port Profile Characteristic Data User Description
static uint8 SerialPortServiceDataUserDesp[21] = "Data Characteristic \0";

// Serial Port Profile Characteristic Status Properties
static uint8 SerialPortServiceStatusProps = GATT_PROP_READ;

// Characteristic Status Value
static uint8 SerialPortServiceStatus[SERIALPORTSERVICE_STATUS_LEN] = {0,0,0,0,0,0,0};

// Serial Port Profile Characteristic Status User Description
static uint8 SerialPortServiceStatusUserDesp[23] = "Status Characteristic \0";

static uint32 numTxBytes = 0;   //received on serial port, sent to central. since last status readout.
static uint32 numRxBytes = 0;   //received from central, sent on serial port. since last status readout.
static uint32 numRFLinkOverRun = 0; //number of bytes lost due to overrun from serial device.
static uint32 numFramingError = 0;
static uint32 numParityError = 0;
static UART_Params SerialPortParams;

// Serial Port Profile Characteristic Config Properties
static uint8 SerialPortServiceConfigProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Config Value
static uint8 SerialPortServiceConfig[SERIALPORTSERVICE_CONFIG_LEN] = {0x2D,0x00,02};
                                     
// Serial Port Profile Characteristic Config User Description
static uint8 SerialPortServiceConfigUserDesp[23] = "Config Characteristic \0";

//Keep track of length
static uint8 charDataValueLen = SERIALPORTSERVICE_DATA_LEN;

/*********************************************************************
 * Profile Attributes - Table
 */

gattAttribute_t SerialPortServiceAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Serial Port Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&SerialPortService            /* pValue */
  },    

    // Characteristic Data Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &SerialPortServiceDataProps 
    },

      // Characteristic Data Value 
      { 
        { ATT_UUID_SIZE, SerialPortServiceDataUUID },
        GATT_PERMIT_WRITE, 
        0, 
        SerialPortServiceData 
      },

       // Characteristic Data configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&SerialPortServiceDataConfig 
      },
      
      // Characteristic Data User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        SerialPortServiceDataUserDesp 
      },           
      
    // Characteristic Status Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &SerialPortServiceStatusProps 
    },

      // Characteristic Status Value 
      { 
        { ATT_UUID_SIZE, SerialPortServiceStatusUUID },
        GATT_PERMIT_READ, 
        0, 
        //&SerialPortServiceChar3 
        SerialPortServiceStatus
      },

      // Characteristic Status User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        SerialPortServiceStatusUserDesp 
      },

    // Characteristic Config Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &SerialPortServiceConfigProps 
    },

      // Characteristic Value Config
      { 
        { ATT_UUID_SIZE, SerialPortServiceConfigUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        SerialPortServiceConfig 
      },

      // Characteristic Config User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        SerialPortServiceConfigUserDesp 
      },
      
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t SerialPortService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t SerialPortService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// Serial Port Profile Service Callbacks
CONST gattServiceCBs_t SerialPortServiceCBs =
{
  SerialPortService_ReadAttrCB,  // Read callback function pointer
  SerialPortService_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SerialPortService_AddService
 *
 * @brief   Initializes the Serial Port Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t SerialPortService_AddService( uint32 services )
{
  uint8 status;

  // Allocate Client Characteristic Configuration table
  SerialPortServiceDataConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                            linkDBNumConns );

  if ( SerialPortServiceDataConfig == NULL )
  {     
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, SerialPortServiceDataConfig );

#if (defined(AUTO_NOTIFICATION)  && (AUTO_NOTIFICATION == TRUE))
  //Hardcode to enable notification in GATT table
   SerialPortServiceDataConfig[0].connHandle = 0x0000; 
   SerialPortServiceDataConfig[0].value = 0x01;
#endif

  if ( services & SERIALPORTSERVICE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( SerialPortServiceAttrTbl, 
                                          GATT_NUM_ATTRS( SerialPortServiceAttrTbl ),
                                          16,
                                          &SerialPortServiceCBs );
  }
  else
  {
    status = SUCCESS;
  }
  
  return ( status );
}

/*********************************************************************
 * @fn      SerialPortService_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t SerialPortService_RegisterAppCBs( SerialPortServiceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    SerialPortService_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      SerialPortService_SetParameter
 *
 * @brief   Set a Serial Port Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SerialPortService_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SERIALPORTSERVICE_CHAR_DATA:
      
      if( len <= SERIALPORTSERVICE_DATA_LEN )
      {
        memset(SerialPortServiceData, 0, SERIALPORTSERVICE_DATA_LEN);
        VOID memcpy( SerialPortServiceData, value, len );
        charDataValueLen = len;
        
        // See if Notification has been enabled
        ret = GATTServApp_ProcessCharCfg( SerialPortServiceDataConfig, SerialPortServiceData, FALSE,
                                    SerialPortServiceAttrTbl, GATT_NUM_ATTRS( SerialPortServiceAttrTbl ),
                                    INVALID_TASK_ID, SerialPortService_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      

    case SERIALPORTSERVICE_CHAR_STATUS:

     if ( len == SERIALPORTSERVICE_STATUS_LEN ) 
      {
        VOID memcpy( SerialPortServiceStatus, value, SERIALPORTSERVICE_STATUS_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case SERIALPORTSERVICE_CHAR_CONFIG:
       
      if ( len == SERIALPORTSERVICE_CONFIG_LEN ) 
      {
        VOID memcpy( SerialPortServiceData, value, SERIALPORTSERVICE_CONFIG_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}



/*********************************************************************
 * @fn      SerialPortService_SetStatusTXBytes
 *
 * @brief   Set a Serial Port Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SerialPortService_AddStatusTXBytes( uint16 count )
{
  bStatus_t ret = SUCCESS;
  
  if( count )
  {
    numTxBytes += count;
    SerialPortServiceStatus[6] = numTxBytes & 0x00ff;
    SerialPortServiceStatus[5] = (numTxBytes & 0xff00) >> 8;

  }
  else
  {
      ret = bleInvalidRange;
  }
      
  return ( ret );
}

/*********************************************************************
 * @fn      SerialPortService_AddStatusErrorCount
 *
 * @brief   Set a Serial Port Profile parameter.
 *
 * @param   errorCode - UART_Status error code 
 *
 * @return  bStatus_t
 */
bStatus_t SerialPortService_AddStatusErrorCount( UART_Status errorCode )
{
  bStatus_t ret = SUCCESS;
  
  switch(errorCode)
  {
  case UART_PARITY_ERROR:
    numParityError++;
    SerialPortServiceStatus[PARITY_ERR_BYTE] = numParityError;
    break;
  case UART_OVERRUN_ERROR:
    numRFLinkOverRun++;
    SerialPortServiceStatus[RF_OVERRUN_BYTE] = numRFLinkOverRun;
    break;
  case UART_FRAMING_ERROR:
    numFramingError++;
    SerialPortServiceStatus[FRAMING_ERR_BYTE] = numFramingError;
    break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      SerialPortService_SetStatusRXBytes
 *
 * @brief   Set a Serial Port Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SerialPortService_AddStatusRXBytes( uint16 count )
{
  bStatus_t ret = SUCCESS;
  
  if( count )
  {
    numRxBytes += count;
    SerialPortServiceStatus[4] = numRxBytes & 0x000000ff;
    SerialPortServiceStatus[3] = (numRxBytes & 0x0000ff00) >> 8;
  }
  else
  {
      ret = bleInvalidRange;
  }
      
  return ( ret );
}


/*********************************************************************
 * @fn      SerialPortService_SetUartConfig
 *
 * @brief   Set a Serial Port Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SerialPortService_SetUartConfig( UART_Params *params )
{
  bStatus_t ret = SUCCESS;
  
    SerialPortParams.readMode       = params->readMode;
    SerialPortParams.writeMode      = params->writeMode;
    SerialPortParams.readTimeout    = params->readTimeout;
    SerialPortParams.writeTimeout   = params->writeTimeout;
    SerialPortParams.readCallback   = params->readCallback;
    SerialPortParams.writeCallback  = params->writeCallback;
    SerialPortParams.readReturnMode = params->readReturnMode;
    SerialPortParams.readDataMode   = params->readDataMode;
    SerialPortParams.writeDataMode  = params->writeDataMode;
    SerialPortParams.baudRate       = params->baudRate;
    SerialPortParams.dataLength     = params->dataLength;
    SerialPortParams.stopBits       = params->stopBits;
    SerialPortParams.parityType     = params->parityType;
    SerialPortParams.readEcho       = params->readEcho;
    SerialPortParams.custom         = params->custom;
  
    DEBUG("*****UART SETTINGS*****"); DEBUG_NEWLINE();
       
    uint16_t count = SerialPortParams.baudRate/10;
    SerialPortServiceConfig[1] = count & 0x00ff;
    SerialPortServiceConfig[0] = (count & 0xff00) >> 8;
    
    //DEBUG("UART Baud Rate: ");
    //DEBUG((uint8_t*)convInt32ToText(SerialPortParams.baudRate)); DEBUG_NEWLINE();
    
    SerialPortServiceConfig[2] &= 0xfe; //Start bit Low
    SerialPortServiceConfig[2] |= 0x02; //Stop bit High
    
    DEBUG("Stop bits: ");
    if(SerialPortParams.stopBits == UART_STOP_TWO)
    {
      SerialPortServiceConfig[2] |= 0x04;   //SPB = 1, 2 stop bit
      DEBUG("2 stop bit");
    }
    else
    {
      SerialPortServiceConfig[2] &= 0xfb;   //SPB = 0, 1 stop bit
      DEBUG("1 stop bit");
    }
    
    DEBUG_NEWLINE();
    DEBUG("Parity: ");
    
    switch(SerialPortParams.parityType)
    {
    case UART_PAR_NONE:
      SerialPortServiceConfig[2] &= 0xf7;   //Parity disabled
      DEBUG("UART_PAR_NONE");
      break;
    case UART_PAR_EVEN:
      SerialPortServiceConfig[2] |= 0x08;   //Parity enabled
      SerialPortServiceConfig[2] |= 0x10;   //Even parity
      DEBUG("UART_PAR_EVEN");
      break;
    case UART_PAR_ODD:
      SerialPortServiceConfig[2] |= 0x08;   //Parity enabled
      SerialPortServiceConfig[2] &= 0xef;   //Even parity
      DEBUG("UART_PAR_ODD");
      break;
    case UART_PAR_ZERO:
      break;
    case UART_PAR_ONE:
      break;
    }
    
    DEBUG_NEWLINE();
    
    //Hardware Flow control is not ready yet.
    SerialPortServiceConfig[2] &= 0xdf;
    
    
  return ( ret );
}

/*********************************************************************
 * @fn      SerialPortService_GetUartConfig
 *
 * @brief   Get a Serial Port Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SerialPortService_GetUartConfig( UART_Params *params )
{
  bStatus_t ret = SUCCESS;
  
  params->readMode         =   SerialPortParams.readMode;      
  params->writeMode        =   SerialPortParams.writeMode;     
  params->readTimeout      =   SerialPortParams.readTimeout;   
  params->writeTimeout     =   SerialPortParams.writeTimeout;  
  params->readCallback     =   SerialPortParams.readCallback;  
  params->writeCallback    =   SerialPortParams.writeCallback; 
  params->readReturnMode   =   SerialPortParams.readReturnMode;
  params->readDataMode     =   SerialPortParams.readDataMode;  
  params->writeDataMode    =   SerialPortParams.writeDataMode; 
  params->baudRate         =   SerialPortParams.baudRate;      
  params->dataLength       =   SerialPortParams.dataLength;    
  params->stopBits         =   SerialPortParams.stopBits;      
  params->parityType       =   SerialPortParams.parityType;    
  params->readEcho         =   SerialPortParams.readEcho;      
  params->custom           =   SerialPortParams.custom;        
    
  return ( ret );
}




/*********************************************************************
 * @fn      SerialPortService_GetParameter
 *
 * @brief   Get a Serial Port Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SerialPortService_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SERIALPORTSERVICE_CHAR_DATA:
      VOID memcpy( value, SerialPortServiceData, SERIALPORTSERVICE_DATA_LEN );
      break; 

    case SERIALPORTSERVICE_CHAR_STATUS:
      VOID memcpy( value, SerialPortServiceStatus, SERIALPORTSERVICE_STATUS_LEN );
      break; 
      
    case SERIALPORTSERVICE_CHAR_CONFIG:
      VOID memcpy( value, SerialPortServiceConfig, SERIALPORTSERVICE_CONFIG_LEN );
      break;  
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          SerialPortService_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t SerialPortService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_UUID_SIZE )
  {
    // Get 16-bit UUID from 128-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[12], pAttr->type.uuid[13]);
    
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics SERIALPORTSERVICE_STATUS_UUID and SERIALPORTSERVICE_CONFIG_UUID have read permissions
      // characteristic SERIALPORTSERVICE_DATA_UUID does not have read permissions, but because it
      //   can be sent as a notification, it is included here

      case SERIALPORTSERVICE_DATA_UUID:
        *pLen = charDataValueLen;
        VOID memcpy( pValue, pAttr->pValue, charDataValueLen );
        break;
        
      case SERIALPORTSERVICE_STATUS_UUID:
        *pLen = SERIALPORTSERVICE_STATUS_LEN;
        VOID memcpy( pValue, pAttr->pValue, SERIALPORTSERVICE_STATUS_LEN );
        
        //Reset all counters
        numTxBytes = 0;
        SerialPortServiceStatus[6] = 0x00;
        SerialPortServiceStatus[5] = 0x00;
        numRxBytes = 0;
        SerialPortServiceStatus[4] = 0x00;
        SerialPortServiceStatus[3] = 0x00;
        numRFLinkOverRun = 0;
        numFramingError = 0;
        numParityError = 0;
        break;        
      case SERIALPORTSERVICE_CONFIG_UUID:
        *pLen = SERIALPORTSERVICE_CONFIG_LEN;
        VOID memcpy( pValue, pAttr->pValue, SERIALPORTSERVICE_CONFIG_LEN );
        break;
   
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else if( pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {        
      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    //neither 16-bit UUID nor 128bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      SerialPortService_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t SerialPortService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[12], pAttr->type.uuid[13]);
    switch ( uuid )
    {
      case SERIALPORTSERVICE_DATA_UUID:
        if ( offset == 0 )
        {
          if ( len > SERIALPORTSERVICE_DATA_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          
          //Copy/Store data to the GATT table entry
          memset(pCurValue, 0, SERIALPORTSERVICE_DATA_LEN);
          memcpy(pCurValue, pValue, len);
          
#ifdef SDI_USE_UART          
          //Send Data to UART 
          SDITask_sendToUART(pCurValue, len);
#else         
          SNP_replyToHost_send(0x55, 0xFF, NULL, len, pCurValue);
#endif
          //Toggle LED to indicate data received from client
          SPPBLEServer_toggleLed(Board_RLED, Board_LED_TOGGLE);
          
          if (len > 0)
          {
           SerialPortService_AddStatusRXBytes( len );
          }
      
          notifyApp = SERIALPORTSERVICE_CHAR_DATA;
        }
             
        break;
        
      case SERIALPORTSERVICE_CONFIG_UUID:

        //Validate the value
        if ( offset == 0 )
        {
         if ( len != SERIALPORTSERVICE_CONFIG_LEN )
         {
            status = ATT_ERR_INVALID_VALUE_SIZE;
         }
        }
        else
        {
         status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          
          memset(pCurValue, 0, SERIALPORTSERVICE_CONFIG_LEN);
          memcpy(pCurValue, pValue, len);
            
          //Configured in higher application layer        
          notifyApp = SERIALPORTSERVICE_CHAR_CONFIG;
        }
             
        break;

      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else if (pAttr->type.len == ATT_BT_UUID_SIZE )
  {  
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

    switch ( uuid )
    {
    case GATT_CLIENT_CHAR_CFG_UUID:
       status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
       break;
    default:
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
      //neither 16bit UUID nor 128bit UUID
      status = ATT_ERR_INVALID_HANDLE;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && SerialPortService_AppCBs && SerialPortService_AppCBs->pfnSerialPortServiceChange )
  {
    SerialPortService_AppCBs->pfnSerialPortServiceChange( notifyApp );  
  }
  
  return ( status );
}

/*********************************************************************
*********************************************************************/
