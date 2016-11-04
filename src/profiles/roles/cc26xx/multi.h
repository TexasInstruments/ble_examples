/*
 * Filename: multi.h
 *
 * Description: multi role profile code header
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


#ifndef PERIPHERAL_H
#define PERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*-------------------------------------------------------------------
 * INCLUDES
 */
#include "gap.h"

/*-------------------------------------------------------------------
 * CONSTANTS
 */
  
/** @defgroup GAPROLE_PROFILE_PARAMETERS GAP Role Parameters
 * @{
 */
#define GAPROLE_PROFILEROLE         0x300  //!< Reading this parameter will return GAP Role type. Read Only. Size is uint8_t.
#define GAPROLE_IRK                 0x301  //!< Identity Resolving Key. Read/Write. Size is uint8_t[KEYLEN]. Default is all 0, which means that the IRK will be randomly generated.
#define GAPROLE_SRK                 0x302  //!< Signature Resolving Key. Read/Write. Size is uint8_t[KEYLEN]. Default is all 0, which means that the SRK will be randomly generated.
#define GAPROLE_SIGNCOUNTER         0x303  //!< Sign Counter. Read/Write. Size is uint32_t. Default is 0.
#define GAPROLE_BD_ADDR             0x304  //!< Device's Address. Read Only. Size is uint8_t[B_ADDR_LEN]. This item is read from the controller.
#define GAPROLE_ADVERT_ENABLED      0x305  //!< Enable/Disable Advertising. Read/Write. Size is uint8_t. Default is TRUE=Enabled.
#define GAPROLE_ADVERT_OFF_TIME     0x306  //!< Advertising Off Time for Limited advertisements (in milliseconds). Read/Write. Size is uint16_t. Default is 30 seconds.
#define GAPROLE_ADVERT_DATA         0x307  //!< Advertisement Data. Read/Write. Max size is uint8_t[B_MAX_ADV_LEN].  Default is "02:01:01", which means that it is a Limited Discoverable Advertisement.
#define GAPROLE_SCAN_RSP_DATA       0x308  //!< Scan Response Data. Read/Write. Max size is uint8_t[B_MAX_ADV_LEN]. Defaults to all 0.
#define GAPROLE_ADV_EVENT_TYPE      0x309  //!< Advertisement Type. Read/Write. Size is uint8_t.  Default is GAP_ADTYPE_ADV_IND (defined in GAP.h).
#define GAPROLE_ADV_DIRECT_TYPE     0x30A  //!< Direct Advertisement Address Type. Read/Write. Size is uint8_t. Default is ADDRTYPE_PUBLIC (defined in GAP.h).
#define GAPROLE_ADV_DIRECT_ADDR     0x30B  //!< Direct Advertisement Address. Read/Write. Size is uint8_t[B_ADDR_LEN]. Default is NULL.
#define GAPROLE_ADV_CHANNEL_MAP     0x30C  //!< Which channels to advertise on. Read/Write Size is uint8_t. Default is GAP_ADVCHAN_ALL (defined in GAP.h)
#define GAPROLE_ADV_FILTER_POLICY   0x30D  //!< Filter Policy. Ignored when directed advertising is used. Read/Write. Size is uint8_t. Default is GAP_FILTER_POLICY_ALL (defined in GAP.h).
#define GAPROLE_RSSI_READ_RATE      0x30F  //!< How often to read the RSSI during a connection. Read/Write. Size is uint16_t. The value is in milliseconds. Default is 0 = OFF.
#define GAPROLE_MIN_CONN_INTERVAL   0x311  //!< Minimum Connection Interval to allow (n * 1.25ms).  Range: 7.5 msec to 4 seconds (0x0006 to 0x0C80). Read/Write. Size is uint16_t. Default is 7.5 milliseconds (0x0006).
#define GAPROLE_MAX_CONN_INTERVAL   0x312  //!< Maximum Connection Interval to allow (n * 1.25ms).  Range: 7.5 msec to 4 seconds (0x0006 to 0x0C80). Read/Write. Size is uint16_t. Default is 4 seconds (0x0C80).
#define GAPROLE_SLAVE_LATENCY       0x313  //!< Update Parameter Slave Latency. Range: 0 - 499. Read/Write. Size is uint16_t. Default is 0.
#define GAPROLE_TIMEOUT_MULTIPLIER  0x314  //!< Update Parameter Timeout Multiplier (n * 10ms). Range: 100ms to 32 seconds (0x000a - 0x0c80). Read/Write. Size is uint16_t. Default is 1000.
#define GAPROLE_PARAM_UPDATE_REQ    0x319  //!< Slave Connection Parameter Update Request. Write. Size is uint8_t. If TRUE then connection parameter update request is sent.
#define GAPROLE_STATE               0x31A  //!< Reading this parameter will return GAP Peripheral Role State. Read Only. Size is uint8_t.
#define GAPROLE_ADV_NONCONN_ENABLED 0x31B  //!< Enable/Disable Non-Connectable Advertising.  Read/Write.  Size is uint8_t.  Default is FALSE=Disabled.
#define GAPROLE_MAX_SCAN_RES        0x404  //!< Maximum number of discover scan results to receive. Default is 0 = unlimited.
/** @} End GAPROLE_PROFILE_PARAMETERS */
   
/*-------------------------------------------------------------------
 * TYPEDEFS
 */
   
/**
 * GAP Peripheral Role States.
 */
typedef enum
{
  GAPROLE_INIT = 0,                       //!< Waiting to be started
  GAPROLE_STARTED,                        //!< Started but not advertising
  GAPROLE_ADVERTISING,                    //!< Currently Advertising
  GAPROLE_ADVERTISING_NONCONN,            //!< Currently using non-connectable Advertising
  GAPROLE_WAITING,                        //!< Device is started but not advertising, is in waiting period before advertising again
  GAPROLE_WAITING_AFTER_TIMEOUT,          //!< Device just timed out from a connection but is not yet advertising, is in waiting period before advertising again
  GAPROLE_CONNECTED,                      //!< In a connection
  GAPROLE_CONNECTED_ADV,                  //!< In a connection + advertising
  GAPROLE_ERROR                           //!< Error occurred - invalid state
} Peripheral_States_t;

/**
 * Multi Event Structure
 */
typedef union
{
  gapEventHdr_t             gap;                //!< GAP_MSG_EVENT and status.
  gapDeviceInitDoneEvent_t  initDone;           //!< GAP initialization done.
  gapDeviceInfoEvent_t      deviceInfo;         //!< Discovery device information event structure.
  gapDevDiscEvent_t         discCmpl;           //!< Discovery complete event structure.
  gapEstLinkReqEvent_t      linkCmpl;           //!< Link complete event structure.
  gapLinkUpdateEvent_t      linkUpdate;         //!< Link update event structure.
  gapTerminateLinkEvent_t   linkTerminate;      //!< Link terminated event structure.
} gapMultiRoleEvent_t;

typedef struct
{
  uint16_t connHandle;
  uint16_t minConnInterval;
  uint16_t maxConnInterval;
  uint16_t slaveLatency;
  uint16_t timeoutMultiplier;
} gapRole_updateConnParams_t;

/**
 *  Possible actions the peripheral device may take if an unsuccessful parameter
 *  update is received.
 *
 *  Parameters for GAPRole_SendUpdateParam() only
 */

#define GAPROLE_NO_ACTION                    0 // Take no action upon unsuccessful parameter updates
#define GAPROLE_RESEND_PARAM_UPDATE          1 // Continue to resend request until successful update
#define GAPROLE_TERMINATE_LINK               2 // Terminate link upon unsuccessful parameter updates

// params for gaprole link functions
#define GAPROLE_ACTIVE_LINKS            1
#define GAPROLE_AVAILABLE_LINKS         2

#define GAPROLE_CONN_JUST_TERMINATED    0xFFFD

/*-------------------------------------------------------------------
 * MACROS
 */

/*-------------------------------------------------------------------
 * Profile Callbacks
 */

/**
 * Callback when the connection parameteres are updated.
 */
typedef void (*gapRolesParamUpdateCB_t)(uint16_t connInterval,
                                        uint16_t connSlaveLatency,
                                        uint16_t connTimeout);

/**
 * Callback when the device has been started.  Callback event to
 * the Notify of a state change.
 */
typedef void (*peripheralStateNotify_t)(Peripheral_States_t newState, uint16_t connHandle);

/**
 * Callback when the event should be processed by the app instead of the GAP Role
 */
typedef uint8_t (*passThroughToApp_t)
(
  gapMultiRoleEvent_t *pEvent         //!< Pointer to event structure.
);

/**
 * Callback structure - must be setup by the application and used when 
 *                      GAPRole_StartDevice() is called.
 */
typedef struct
{
  passThroughToApp_t        pfnPassThrough;  //!< When the event should be processed by the app instead of the GAP Role
} gapRolesCBs_t;

/*-------------------------------------------------------------------
 * API FUNCTIONS
 */

/**
 * @defgroup GAPROLES_PERIPHERAL_API GAP Peripheral Role API Functions
 *
 * @{
 */

/**
 * @brief       Set a GAP Role parameter.
 *
 *  NOTE: You can call this function with a GAP Parameter ID and it will set the
 *        GAP Parameter.  GAP Parameters are defined in (gap.h).  Also,
 *        the "len" field must be set to the size of a "uint16_t" and the
 *        "pValue" field must point to a "uint16_t".
 *
 * @param       param - Profile parameter ID: @ref GAPROLE_PROFILE_PARAMETERS
 * @param       len - length of data to write
 * @param       pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return      SUCCESS or INVALIDPARAMETER (invalid paramID)
 */
extern bStatus_t GAPRole_SetParameter(uint16_t param, uint8_t len, void *pValue, uint8 connHandle);

/**
 * @brief       Get a GAP Role parameter.
 *
 *  NOTE: You can call this function with a GAP Parameter ID and it will get a
 *        GAP Parameter.  GAP Parameters are defined in (gap.h).  Also, the
 *        "pValue" field must point to a "uint16_t".
 *
 * @param       param - Profile parameter ID: @ref GAPROLE_PROFILE_PARAMETERS
 * @param       pValue - pointer to location to get the value.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return      SUCCESS or INVALIDPARAMETER (invalid paramID)
 */
extern bStatus_t GAPRole_GetParameter(uint16_t param, void *pValue, uint8 connHandle);

/**
 * @brief       Does the device initialization.  Only call this function once.
 *
 * @param       pAppCallbacks - pointer to application callbacks.
 *
 * @return      SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t GAPRole_StartDevice(gapRolesCBs_t *pAppCallbacks, uint8_t* numConns);

/**
 * @brief       Terminates the existing connection.
 *
 * @return      SUCCESS or bleIncorrectMode
 */
extern bStatus_t GAPRole_TerminateConnection(uint16_t connHandle);

/**
 * @brief       Update the parameters of an existing connection
 *
 * @param       connInterval - the new connection interval
 * @param       latency - the new slave latency
 * @param       connTimeout - the new timeout value
 * @param       handleFailure - what to do if the update does not occur.
 *              Method may choose to terminate connection, try again, or take no action
 *
 * @return      SUCCESS, bleNotConnected or bleInvalidRange
 */
extern bStatus_t GAPRole_SendUpdateParam(uint16_t minConnInterval, 
                                         uint16_t maxConnInterval,
                                         uint16_t latency, uint16_t connTimeout,
                                         uint8_t handleFailure,
                                         uint16_t connHandle  );

/**
 * @brief       Register application's callbacks.
 *
 * @param       pParamUpdateCB - pointer to param update callback.
 *
 * @return      none
 */
extern void GAPRole_RegisterAppCBs(gapRolesParamUpdateCB_t *pParamUpdateCB);

/**
 * @brief   Start a device discovery scan.
 *
 * @param   mode - discovery mode: @ref GAP_DEVDISC_MODE_DEFINES
 * @param   activeScan - TRUE to perform active scan
 * @param   whiteList - TRUE to only scan for devices in the white list
 *
 * @return  SUCCESS: Discovery scan started.<BR>
 *          bleIncorrectMode: Invalid profile role.<BR>
 *          bleAlreadyInRequestedMode: Not available.<BR>
 */
extern bStatus_t GAPRole_StartDiscovery(uint8_t mode, uint8_t activeScan, uint8_t whiteList);

/**
 * @brief   Cancel a device discovery scan.
 *
 * @return  SUCCESS: Cancel started.<BR>
 *          bleInvalidTaskID: Not the task that started discovery.<BR>
 *          bleIncorrectMode: Not in discovery mode.<BR>
 */
extern bStatus_t GAPRole_CancelDiscovery(void);

/**
 * @brief   Establish a link to a peer device.
 *
 * @param   highDutyCycle -  TRUE to high duty cycle scan, FALSE if not
 * @param   whiteList - determines use of the white list: @ref GAP_WHITELIST_DEFINES
 * @param   addrTypePeer - address type of the peer device: @ref GAP_ADDR_TYPE_DEFINES
 * @param   peerAddr - peer device address
 *
 * @return  SUCCESS: started establish link process.<BR>
 *          bleIncorrectMode: invalid profile role.<BR>
 *          bleNotReady: a scan is in progress.<BR>
 *          bleAlreadyInRequestedMode: can’t process now.<BR>
 *          bleNoResources: too many links.<BR>
 */
extern bStatus_t GAPRole_EstablishLink(uint8_t highDutyCycle, uint8_t whiteList,
                                              uint8_t addrTypePeer, uint8_t *peerAddr);   
                                         
/**
 * @} End GAPROLES_PERIPHERAL_API
 */


extern bStatus_t gapRole_connUpdate(uint8_t handleFailure, 
                                       gapRole_updateConnParams_t *pConnParams);

/*-------------------------------------------------------------------
 * TASK FUNCTIONS - Don't call these. These are system functions.
 */
   
/**
 * @brief       Task creation function for the GAP Peripheral Role.
 *
 * @param       none
 *
 * @return      none
 */
extern void GAPRole_createTask(void);

extern void gapRole_abort(void);

/*-------------------------------------------------------------------
-------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERAL_H */
