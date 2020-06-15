/******************************************************************************

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2009-2020, Texas Instruments Incorporated
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

/**
 *  @defgroup Peripheral GAPRole (Peripheral)
 *  @brief This module implements the Peripheral GAP Role
 *  @{
 *  @file  peripheral.h
 *  @brief      Peripheral layer interface
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

/*-------------------------------------------------------------------
 * CONSTANTS
 */

/** @defgroup Peripheral_Constants Peripheral GAPRole Constants
 * @{
 */

/** @defgroup Peripheral_Params Peripheral GAPRole Parameters
 * @{
 * Parameters set via @ref GAPRole_SetParameter
 */

/**
 * @brief This parameter will return GAP Role type (Read-only)
 *
 * size: uint8_t
 *
 * range: when using the Peripheral GAPRole, this will always be @ref GAP_PROFILE_PERIPHERAL
 */
#define GAPROLE_PROFILEROLE         0x300

/**
 * @brief Identity Resolving Key (Read/Write) Size is uint8_t[KEYLEN].
 *
 * @note If this is set to all 0x00's, the IRK will be randomly generated
 *
 * size: uint8_t[16]
 *
 * default: 0x00000000000000000000000000000000
 *
 * range: 0x00000000000000000000000000000000 - 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
 */
#define GAPROLE_IRK                 0x301

/**
 * @brief Signature Resolving Key (Read/Write)
 *
 * @note If this is set to all 0x00's, the SRK will be randomly generated
 *
 * size: uint8_t[16]
 *
 * default: 0x00000000000000000000000000000000
 *
 * range: 0x00000000000000000000000000000000 - 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
 */
#define GAPROLE_SRK                 0x302

/**
 * @brief Sign Counter (Read/Write)
 *
 * size: uint32_t
 *
 * default: 0x0000
 *
 * range: 0x0000 - 0xFFFF
 */
#define GAPROLE_SIGNCOUNTER         0x303

/**
 * @brief Device Address read from the controller (Read-only)
 *
 * The BDADDR is read, in increasing order of priortiy, from the info page,
 * secondary address from flash, or set from @ref HCI_ReadBDADDRCmd
 *
 * size: uint8_t[6]
 *
 * default: BDADDR from info page
 *
 * range: 0x000000000000 - 0xFFFFFFFFFFFE
 */
#define GAPROLE_BD_ADDR             0x304

/**
 * @brief Enable/Disable Connectable Advertising (Read/Write)
 *
 * @warning @ref GAPROLE_ADV_NONCONN_ENABLED must be set to FALSE in order to enable this
 *
 * size: uint8_t
 *
 * default: TRUE
 *
 * range: TRUE (enabled) or FALSE (disabled)
 */
#define GAPROLE_ADVERT_ENABLED      0x305

/**
 * @brief How long to remain off (in sec) after advertising stops before starting again (Read/Write)
 *
 * If set to 0, advertising will not start again.
 *
 * size: uint16
 *
 * default: 30
 *
 * range: 0-65535
 */
#define GAPROLE_ADVERT_OFF_TIME     0x306

/**
 * @brief Advertisement data (Read/Write)
 *
 * @note The third byte sets limited / general advertising as defined in Vol 3, Part C, section 11.1.3
 * of the BT 4.2 Core Spec.
 *
 * size: a uint8_t array of up to 31 bytes
 *
 * default: 02:01:01 (general advertising)
 */
#define GAPROLE_ADVERT_DATA         0x307

/**
 * @brief Scan Response Data (Read/Write)
 *
 * @note This should be formatted as define d in Vol 3, Part C, section 11.1.3 of the BT 4.2 Core Spec.
 *
 * size: a uint8_t array of up to 31 bytes
 *
 * default: all 0x00's
 */
#define GAPROLE_SCAN_RSP_DATA       0x308

/**
 * @brief Advertisement Types (Read/Write)
 *
 * size: uint8_t
 *
 * default: @ref GAP_ADTYPE_ADV_IND
 *
 * range: @ref GAP_Adv_Types
 */
#define GAPROLE_ADV_EVENT_TYPE      0x309

/**
 * @brief Direct Advertisement Type (Read/Write)
 *
 * size: uint8_t
 *
 * default: @ref ADDRMODE_PUBLIC
 *
 * range: @ref Gap_Addr_Modes
 */
#define GAPROLE_ADV_DIRECT_TYPE     0x30A

/**
 * @brief Direct Advertisement Address (Read/Write)
 *
 * size: uint8_t[6]
 *
 * default: NULL
 *
 * range: 0x000000000000 - 0xFFFFFFFFFFFE
 */
#define GAPROLE_ADV_DIRECT_ADDR     0x30B

/**
 * @brief Which channels to advertise on (Read/Write)
 *
 * Multiple channels can be selected by ORing the bit values below.
 *
 * size: uint8_t
 *
 * default: @ref GAP_ADVCHAN_ALL
 *
 * range: @ref GAP_Adv_Chans
 */
#define GAPROLE_ADV_CHANNEL_MAP     0x30C

/**
 * @brief Policy for filtering advertisements (Read/Write)
 *
 * @note This is ignored for direct advertising.
 *
 * size: uint8_t
 *
 * default: @ref GAP_FILTER_POLICY_ALL
 *
 * range: @ref GAP_Adv_Filter_Policices
 */
#define GAPROLE_ADV_FILTER_POLICY   0x30D

/**
 * @brief Connection Handle of current connected device (Read-only)
 *
 * size: uint16_t
 *
 * range: 0x0000-0xFFFD
 */
#define GAPROLE_CONNHANDLE          0x30E

/// @cond NODOC
/**
 * @brief How often to read the RSSI during a connection (Read/Write)
 *
 * size: uint16_t
 *
 * default: 0x00
 * range:
 */
#define GAPROLE_RSSI_READ_RATE      0x30F
/// @endcond // NODOC

/**
 * @brief Whether to request a connection parameter update upon connection (Read/Write)
 *
 * size: uint8_t
 *
 * default: TRUE
 *
 * range: TRUE (do request) or FALSE (don't request)
 */
#define GAPROLE_PARAM_UPDATE_ENABLE 0x310

/**
 * @brief  Minimum connection interval (n * 1.25 ms) to use when performing param update (Read/Write)
 *
 * size: uint16_t
 *
 * default: 6
 *
 * range: 6 - @ref GAPROLE_MAX_CONN_INTERVAL
 */
#define GAPROLE_MIN_CONN_INTERVAL   0x311

/**
 * @brief Maximum connection interval (n * 1.25 ms) to use when performing param update (Read/Write)
 *
 * size: uint16_t
 *
 * default: 3200
 *
 * range: @ref GAPROLE_MIN_CONN_INTERVAL - 3200
 */
#define GAPROLE_MAX_CONN_INTERVAL   0x312

/**
 * @brief Slave latency to use when performing param update (Read/Write)
 *
 * size: uint16_t
 *
 * default: 0
 *
 * range: 0 - 499
 */
#define GAPROLE_SLAVE_LATENCY       0x313

/**
 * @brief Supervision timeout (n x 10 ms) to use when performing param update (Read/Write)
 *
 * size: uint16_t
 *
 * default: 1000
 *
 * range: 10-3200
 */
#define GAPROLE_TIMEOUT_MULTIPLIER  0x314

/**
 * @brief Address of connected device (Read-only)
 *
 * size: uint8_t[6]
 *
 * range: 0x000000000000 - 0xFFFFFFFFFFFD
 */
#define GAPROLE_CONN_BD_ADDR        0x315

/**
 * @brief Connection Interval (n x 1.25 ms) of current connection (Read-only)
 *
 * size: uint16_t
 *
 * range: 6 - 3200
 */
#define GAPROLE_CONN_INTERVAL       0x316

/**
 * @brief Slave Latency of current connection (Read-only)
 *
 * size: uint16_t
 *
 * range: 0 - 499
 */
#define GAPROLE_CONN_LATENCY        0x317

/**
 * @brief Connection Interval (n x 10 ms) of current connection (Read-only)
 *
 * size: uint16_t
 *
 * range: 10 - 3200
 */
#define GAPROLE_CONN_TIMEOUT        0x318  //!< Current timeout value.  Read only.  size is uint16_t.  Range is 100ms to 32 seconds.  Default is 0 (no connection).

/**
 * @brief Used to send a Parameter Update Request (Write-only)
 *
 * size: uint8_t
 *
 * range: TRUE (send the update)
 */
#define GAPROLE_PARAM_UPDATE_REQ    0x319

/**
 * @brief Current Peripheral GAPRole state (Read-only)
 *
 * size: uint8_t
 *
 * default: @ref GAPROLE_INIT
 *
 * range: @ref gaprole_States_t
 */
#define GAPROLE_STATE               0x31A

/**
 * @brief Enable / Disable non-connectable advertising (Read/Write)
 *
 * @warning @ref GAPROLE_ADVERT_ENABLED must be set to FALSE in order to enable this
 *
 * size: uint8_t
 *
 * default: FALSE
 *
 * range: TRUE (enable) or FALSE (disable)
 */
#define GAPROLE_ADV_NONCONN_ENABLED 0x31B

/**
 * @brief Address type of connected device (Read-only)
 *
 * size: uint8_t
 *
 * range: @ref GAP_Addr_Types
 */
#define GAPROLE_BD_ADDR_TYPE        0x31C

/**
 * @brief Reason of the last connection terminated event
 *
 * size: unit8_t
 *
 * range: error defines in ll.h
 */
#define GAPROLE_CONN_TERM_REASON    0x31D

/** @} End Peripheral_Params */

/*-------------------------------------------------------------------
 * TYPEDEFS
 */

/// @brief GAP Peripheral Role States.
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
} gaprole_States_t;

/** @defgroup Peripheral_Param_Update_Fail_Actions Failed Parameter Update Actions
 * @{
 *  Possible actions the device may take if an unsuccessful parameter
 *  update is received.
 */
#define GAPROLE_NO_ACTION                    0 //!< Take no action upon unsuccessful parameter updates
#define GAPROLE_RESEND_PARAM_UPDATE          1 //!< Continue to resend request until successful update
#define GAPROLE_TERMINATE_LINK               2 //!< Terminate link upon unsuccessful parameter updates
/** @} End Peripheral_Param_Update_Fail_Actions */

/** @defgroup Multi_Param_Update_Options Parameter Update Options
 * @{
 *  Possible actions the device may take when it receives a
 *  Connection Parameter Update Request.
 */
#define GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS   0 //!< Wait for parameter update request, respond with remote's requested parameters.
#define GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS 1 //!< Initiate parameter update request, respond with best combination of local and remote parameters.
#define GAPROLE_LINK_PARAM_UPDATE_INITIATE_APP_PARAMS  2 //!< Initiate parameter update request, respond with local requested parameters only.
#define GAPROLE_LINK_PARAM_UPDATE_WAIT_APP_PARAMS      3 //!< Wait for parameter update request, respond with local requested parameters only.
#define GAPROLE_LINK_PARAM_UPDATE_WAIT_BOTH_PARAMS     4 //!< Wait for parameter update request, respond with best combination of local and remote parameters.
#define GAPROLE_LINK_PARAM_UPDATE_REJECT_REQUEST       5 //!< Reject all parameter update requests.
#define GAPROLE_LINK_PARAM_UPDATE_NUM_OPTIONS          6 //!< Used for parameter checking.
/** @} End Multi_Param_Update_Options */

/** @} End Peripheral_Constants */

/*-------------------------------------------------------------------
 * MACROS
 */

/*-------------------------------------------------------------------
 * Profile Callbacks
 */

/** @defgroup Peripheral_CBs Peripheral GAPRole Callbacks
 * @{
 * These are functions whose pointers are passed from the application
 * to the GAPRole so that the GAPRole can send events to the application.
 */

/**
 * @brief Callback when the connection parameters are updated.
 *
 * @param connInterval new connection interval
 * @param connSlaveLatency new slave latency
 * @param connTimeout new supervision timeout
 */
typedef void (*gapRolesParamUpdateCB_t)(uint16_t connInterval,
                                        uint16_t connSlaveLatency,
                                        uint16_t connTimeout);

/**
 * @brief State Change Callback Type
 *
 * Callback to notify the application of a GAPRole state change
 *
 * @param newState new GAPRole state
 */
typedef void (*gapRolesStateNotify_t)(gaprole_States_t newState);

/**
 * @brief Peripheral GAPRole Callback structure
 *
 * This must be setup by the application and passed to the GAPRole when
 * @ref GAPRole_StartDevice is called.
 */
typedef struct
{
  gapRolesStateNotify_t    pfnStateChange;  //!< Whenever the device changes state
} gapRolesCBs_t;

/** @} End Peripheral_CBs */

/*-------------------------------------------------------------------
 * API FUNCTIONS
 */

/**
 * @brief       Set a GAP Role parameter.
 *
 * @note
 * The "len" field must be set to the size of a "uint16_t" and the
 *        "pValue" field must point to a "uint16_t".
 *
 * @param       param @ref Peripheral_Params
 * @param       len length of data to write
 * @param       pValue pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  @ref SUCCESS
 * @return  @ref INVALIDPARAMETER
 * @return  @ref bleInvalidRange : len is invalid for the given param
 * @return  @ref blePending : previous param update has not been completed
 * @return  @ref bleIncorrectMode : can not start connectable advertising because nonconnectable
 *            advertising is enabled
 */
extern bStatus_t GAPRole_SetParameter(uint16_t param, uint8_t len, void *pValue);

/**
 * @brief       Get a GAP Role parameter.
 *
 * @note
 *  The "pValue" field must point to a "uint16_t".
 *
 * @param       param @ref Peripheral_Params
 * @param       pValue pointer to location to get the value.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  @ref SUCCESS
 * @return  @ref INVALIDPARAMETER
 */
extern bStatus_t GAPRole_GetParameter(uint16_t param, void *pValue);

/**
 * @brief     Initialize the GAP layer
 *
 * @warning   Only call this function once
 *
 * @param     pAppCallbacks pointer to application callbacks.
 *
 * @return    @ref SUCCESS
 * @return    @ref bleAlreadyInRequestedMode : device was already initialized
 */
extern bStatus_t GAPRole_StartDevice(gapRolesCBs_t *pAppCallbacks);

/**
 * @brief       Terminates the existing connection.
 *
 * @return      @ref SUCCESS
 * @return      @ref bleIncorrectMode : there is no active connection
 * @return      @ref HCI_ERROR_CODE_CONTROLLER_BUSY : disconnect is already in process
 */
extern bStatus_t GAPRole_TerminateConnection(void);

/**
 * @brief       Update the parameters of an existing connection
 *
 * @param       minConnInterval the desired min connection interval
 * @param       maxConnInterval the desired min connection interval
 * @param       latency the new slave latency
 * @param       connTimeout the new timeout value
 * @param       handleFailure @ref Peripheral_Param_Update_Fail_Actions
 *
 * @return      @ref SUCCESS : operation was successful.
 * @return      @ref INVALIDPARAMETER : Data can not fit into one packet.
 * @return      @ref MSG_BUFFER_NOT_AVAIL
 * @return      @ref bleInvalidRange : connection parameters violate spec
 * @return      @ref bleIncorrectMode : invalid profile role.
 * @return      @ref bleAlreadyInRequestedMode : already updating link parameters.
 * @return      @ref bleNotConnected : Connection is down
 * @return      @ref bleMemAllocError
 * @return      @ref bleNoResources
 */
extern bStatus_t GAPRole_SendUpdateParam(uint16_t minConnInterval,
                                         uint16_t maxConnInterval,
                                         uint16_t latency, uint16_t connTimeout,
                                         uint8_t handleFailure);

/**
 * @brief       Register application's callbacks.
 *
 * @param       pParamUpdateCB pointer to param update callback.
 */
extern void GAPRole_RegisterAppCBs(gapRolesParamUpdateCB_t *pParamUpdateCB);

/// @cond NODOC

/*-------------------------------------------------------------------
 * TASK FUNCTIONS - Don't call these. These are system functions.
 */

/**
 * @brief       Task creation function for the GAP Peripheral Role.
 *
 */
extern void GAPRole_createTask(void);

/// @endcond // NODOC

/*-------------------------------------------------------------------
-------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERAL_H */

/** @} End Peripheral */
