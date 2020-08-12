/******************************************************************************

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2010-2020, Texas Instruments Incorporated
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
 *  @defgroup GAPBondMgr GAPBondMgr
 *  @brief This module implements the GAP Bond Manager
 *  For a detailed usage section describing how to send these commands and receive events,
 *  see the <a href="../ble-stack/gapbondmngr.html">GAPBondMgr Section</a> of the
 *  User's Guide.
 *  @{
 *  @file  gapbondmgr.h
 *  @brief      GAPBondMgr layer interface
 */

#ifndef GAPBONDMGR_H
#define GAPBONDMGR_H

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

/** @defgroup GAPBondMgr_Constants GAP Bond Manager Constants
 * @{
 */
#if !defined (GAP_BONDINGS_MAX)
  #define GAP_BONDINGS_MAX    10    //!< Maximum number of bonds that can be saved in NV.
#endif

#if !defined (GAP_CHAR_CFG_MAX)
  #define GAP_CHAR_CFG_MAX    4    //!< Maximum number of characteristic configuration that can be saved in NV.
#endif

/** @defgroup GAPBondMgr_Params GAP Bond Manager Parameters
 * @{
 * Parameters set via @ref GAPBondMgr_SetParameter
 */

/**
 * @brief Whether to allow pairing, and if so, whether to initiate pairing. (Read/Write)
 *
 * size: uint8_t
 *
 * default: @ref GAPBOND_PAIRING_MODE_WAIT_FOR_REQ
 *
 * range: @ref GAPBondMgr_Pairing_Modes
 */
#define GAPBOND_PAIRING_MODE          0x400

/**
 * @brief The amount of time (in ms) to wait for a pairing request before sending the
 * slave initiate request (Read/Write)
 *
 * size: uint16_t
 *
 * default: 0 - 65535
 *
 * range: 1000
 */
#define GAPBOND_INITIATE_WAIT         0x401

/**
 * @brief Whether to use authenticated pairing (Read/Write)
 *
 * size: uint8_t
 *
 * default: TRUE
 *
 * range: TRUE (use) or FALSE (do not use)
 */
#define GAPBOND_MITM_PROTECTION       0x402

/**
 * @brief The I/O capabilities of the local device (Read/Write)
 *
 * size: uint8_t
 *
 * default: @ref GAPBOND_IO_CAP_DISPLAY_ONLY
 *
 * range: @ref GAPBondMgr_IO_Caps
 */
#define GAPBOND_IO_CAPABILITIES       0x403

/**
 * @brief Whether to use OOB for pairing (Read/Write)
 *
 * size: uint8_t
 *
 * default: FALSE
 *
 * range: TRUE (use) or FALSE (do not use)
 */
#define GAPBOND_OOB_ENABLED           0x404

/**
 * @brief OOB data to use for pairing (Read/Write)
 *
 * size: uint8_t[16]
 *
 * default: 0x00000000000000000000000000000000 -
 *
 * range: 0x00000000000000000000000000000000 - 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
 */
#define GAPBOND_OOB_DATA              0x405

/**
 * @brief Whether to request bonding during pairing (Read/Write)
 *
 * size: uint8_t
 *
 * default: FALSE
 *
 * range: TRUE (request) or FALSE (do not request)
 */
#define GAPBOND_BONDING_ENABLED       0x406

/**
 * @brief Key distribution list for pairing (Read/Write)
 *
 * Multiple values can be used via bitwise ORing
 *
 * size: uint8_t
 *
 * default: @ref GAPBOND_KEYDIST_SENCKEY | @ref GAPBOND_KEYDIST_SIDKEY
 * | @ref GAPBOND_KEYDIST_MIDKEY | @ref GAPBOND_KEYDIST_MSIGN
 *
 * range: @ref GAPBondMgr_Key_Distr
 */
#define GAPBOND_KEY_DIST_LIST         0x407

/**
 * @brief This parameter is only used by the Host Test app Application (network processor)
 *        The Id of this parameter is used to store the password locally.
 *        the GAP Bond manager is not storing any default password anymore,
 *        everytime a password is needed, pfnPasscodeCB_t will be call.
 *
 * @note an embedded application MUST used @ref pfnPasscodeCB_t
 *
 * size: uint32_t
 *
 * default: 0
 *
 * range: 0 - 999999
 */
#define GAPBOND_DEFAULT_PASSCODE      0x408

/**
 * @brief Erase all bonds from SNV and remove all bonded devices (Write)
 *
 * @note The erase won't happen until there are no active connections.
 */
#define GAPBOND_ERASE_ALLBONDS        0x409

/// @cond NODOC
#define GAPBOND_AUTO_FAIL_PAIRING     0x40A
#define GAPBOND_AUTO_FAIL_REASON      0x40B
/// @endcond // NODOC

/**
 * @brief Key Size used in pairing. (Read/Write)
 *
 * size: uint8_t
 *
 * default: 16
 *
 * range: @ref TGAP_SM_MIN_KEY_LEN - @ref TGAP_SM_MAX_KEY_LEN
 */
#define GAPBOND_KEYSIZE               0x40C

/**
 * @brief Synchronize the whitelist with bonded devices (Read/Write)
 *
 * If TRUE, the whitelist will first be cleared. Then, each unique address stored by bonds in
 * SNV will be synched with the whitelist indefinitely or until this is set to FALSE
 *
 * size: uint8_t
 *
 * default: FALSE
 *
 * range: TRUE (sync) or FALSE (don't sync)
 */
#define GAPBOND_AUTO_SYNC_WL          0x40D

/**
 * @brief Gets the total number of bonds stored in NV (Read-only)
 *
 * size: uint8_t
 *
 * range: 0 - 255
 */
#define GAPBOND_BOND_COUNT            0x40E

/**
 * @brief The action that the device takes after an unsuccessful bonding attempt (Write-only)
 *
 * size: uint8_t
 *
 * default: @ref GAPBOND_FAIL_TERMINATE_LINK
 *
 * range: @ref GAPBondMgr_Bond_Failure_Actions
 */
#define GAPBOND_BOND_FAIL_ACTION      0x40F

/**
 * @brief Erase a single bonded device (Write-only)
 *
 * @note The erase won't happen until there are no active connections.
 *
 * size: uint8_t[9]
 *
 * range: A nine-byte array where the first byte is the @ref GAP_Addr_Types and
 * the next 8 are the device address.
 */
#define GAPBOND_ERASE_SINGLEBOND      0x410

/**
 * @brief Define Secure Connections Usage during Pairing (Read/Write)
 *
 * size: uint8_t
 *
 * default: @ref GAPBOND_SECURE_CONNECTION_ALLOW
 *
 * range: @ref GAPBondMgr_Secure_Cxns
 */
#define GAPBOND_SECURE_CONNECTION     0x411

/**
 * @brief ECC Key Regeneration Policy (Read/Write)
 *
 * Only applicable for Secure Connections. Define reuse of the private and public
 * ECC keys for multiple pairings.The default is to
 * always regenerate the keys upon each new pairing. This parameter has no effect when the
 * application specifies the keys using the @ref GAPBOND_ECC_KEYS parameter. The behavior
 * is that upon each pairing the number of recycles remaining is decremented by 1, but
 * if the pairing fails the count is decremented by 3. The specification recommends that this
 * value be set to no higher than 10 to avoid an attacker from learning too much about a
 * private key before it is regenerated.
 *
 * size: uint8_t
 *
 * default: 2
 *
 * range: 0 - 256
 */
#define GAPBOND_ECCKEY_REGEN_POLICY   0x412

/**
 * @brief The private and public keys to use for Secure Connections pairing (Read/Write)
 *
 * Only applicable for Secure Connections. Allows the application to specify the private and public keys to use
 * pairing. When this is set, the keys are used indefinitely even if a regeneration policy was
 * set with @ref GAPBOND_ECCKEY_REGEN_POLICY. To make the Bond Manager stop using these keys, pass
 * a 1 byte value of 0x00. These keys are stored in RAM and are not retained in non-volatile memory.
 * These keys can be defined by the application, or the application can request them using the
 * @ref SM_GetEccKeys command.
 *
 * size: gapBondEccKeys_t
 *
 * default: by default, the keys are generated using @ref GAPBOND_ECCKEY_REGEN_POLICY
 *
 * range: A valid @ref gapBondEccKeys_t structure <br>
 * 0x00: previously passed keys will no longer be used
 */
#define GAPBOND_ECC_KEYS              0x413

/**
 * @brief Validate Remote OOB Secure Connections Data (Read/Write)
 *
 * Indicate to the Bond Manager that any Secure Connections OOB data that has been received from a
 * remote device, which has been supplied to the Bond Manager by the GAPBOND_REMOTE_OOB_SC_DATA
 * parameter, is valid. Only applicable for Secure Connections
 *
 * size: uint8_t
 *
 * default: FALSE
 *
 * range: TRUE (OOB data is valid) or FALSE (OOB data is not valid)
 */
#define GAPBOND_REMOTE_OOB_SC_ENABLED 0x414

/**
 * @brief Remote OOB Secure Connections Data (Read/Write)
 *
 * Used to pass OOB Secure Connections data to the bond manager that has been received from a remote device.
 * This data is not only the 16 bytes of OOB data, but also the 32-byte ECC Public Key X-Coordinate
 * of the remote device (that it must use when pairing) and a 16-byte confirmation value computed using
 * the said public key, OOB data is used as input to the SM_F4 function. This data can be invalidated by
 * writing 0x00 to the @ref GAPBOND_REMOTE_OOB_SC_ENABLED parameter. Only applicable for Secure Connections.
 *
 * size: gapBondOobSC_t
 *
 * default: all 0x00's
 *
 * range: A valid @ref gapBondOobSC_t structure
 */
#define GAPBOND_REMOTE_OOB_SC_DATA    0x415

/**
 * @brief Validate Local OOB Secure Connections Data (Read/Write)
 *
 * Indicates to the Bond Manager that the local device has valid OOB data it has attempted to send to the
 * remote device, and that the local OOB data supplied to the Bond Manager by the @ref GAPBOND_LOCAL_OOB_SC_DATA
 * parameter is valid. This is needed to determine if the OOB protocol is expected to be used for Secure
 * Connections. Only one device needs to have received OOB data for OOB pairing to be used in Secure
 * Connections. Only applicable for Secure Connections.
 *
 * size: uint8_t
 *
 * default: FALSE
 *
 * range: TRUE (OOB data is valid) or FALSE (OOB data is not valid)
 */
#define GAPBOND_LOCAL_OOB_SC_ENABLED  0x416

/**
 * @brief Local OOB Secure Connections Data (Read/Write)
 *
 * Passes Secure Connections data to the bond manager that was sent to the remote device. This is only the
 * 16 bytes of byte OOB data, which is needed to complete pairing. Only applicable for Secure Connections.
 *
 * size: uint8_t[16]
 *
 * default: all 0x00's
 *
 * range: 0x00000000000000000000000000000000 - 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
 */
#define GAPBOND_LOCAL_OOB_SC_DATA     0x417

/**
 * @brief Enable / Disable LRU Bond Replacement Scheme (Read/Write)
 *
 * Whether to enable the least recently used bond scheme so that, when a  newly bonded device is added and
 * all the entries are full, the least recently used device’s bond is deleted to make room.
 *
 * size: uint8_t
 *
 * default: FALSE
 *
 * range: TRUE (enable) or FALSE (disable)
 */
#define GAPBOND_LRU_BOND_REPLACEMENT  0x418
/** @} End GAPBondMgr_Params */

/**
 * @defgroup GAPBondMgr_Pairing_Modes GAP Bond Manager Pairing Modes
 * @{
 */
#define GAPBOND_PAIRING_MODE_NO_PAIRING          0x00  //!< Pairing is not allowed
#define GAPBOND_PAIRING_MODE_WAIT_FOR_REQ        0x01  //!< Wait for a pairing request or slave security request
#define GAPBOND_PAIRING_MODE_INITIATE            0x02  //!< Don't wait, initiate a pairing request or slave security request
/** @} End GAPBondMgr_Pairing_Modes */

/** @defgroup GAPBondMgr_IO_Caps GAP Bond Manager I/O Capabilities
 * @{
 */
#define GAPBOND_IO_CAP_DISPLAY_ONLY              0x00  //!< Display Only Device
#define GAPBOND_IO_CAP_DISPLAY_YES_NO            0x01  //!< Display and Yes and No Capable
#define GAPBOND_IO_CAP_KEYBOARD_ONLY             0x02  //!< Keyboard Only
#define GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT        0x03  //!< No Display or Input Device
#define GAPBOND_IO_CAP_KEYBOARD_DISPLAY          0x04  //!< Both Keyboard and Display Capable
/** @} End GAPBondMgr_IO_Caps */

/** @defgroup GAPBondMgr_Key_Distr GAP Bond Manager Key Distribution
 * @{
 */
#define GAPBOND_KEYDIST_SENCKEY                  0x01  //!< Slave Encryption Key
#define GAPBOND_KEYDIST_SIDKEY                   0x02  //!< Slave IRK and ID information
#define GAPBOND_KEYDIST_SSIGN                    0x04  //!< Slave CSRK
#define GAPBOND_KEYDIST_SLINK                    0x08  //!< Slave Link Key
#define GAPBOND_KEYDIST_MENCKEY                  0x10  //!< Master Encrypton Key
#define GAPBOND_KEYDIST_MIDKEY                   0x20  //!< Master IRK and ID information
#define GAPBOND_KEYDIST_MSIGN                    0x40  //!< Master CSRK
#define GAPBOND_KEYDIST_MLINK                    0x80  //!< Master Link Key
/** @} End GAPBondMgr_Key_Distr */


/** @defgroup GAPBondMgr_Pairing_States GAP Bond Manager Pairing States
 * @{
 * These are the states that are returned through the @ref pfnPairStateCB_t . See the specific
 * state for its possible statuses.
 */

/**
 * @brief Pairing Started
 *
 * The following statuses are possible for this state:
 * - @ref SUCCESS : pairing has been initiated. A pairing request has been either sent or received.
 */
#define GAPBOND_PAIRING_STATE_STARTED             0x00

/**
 * @brief Pairing Complete
 *
 * The following statuses are possible for this state:
 * - @ref SUCCESS : pairing pairing is complete (Session keys have been exchanged)
 * - @ref SMP_PAIRING_FAILED_PASSKEY_ENTRY_FAILED : user input failed
 * - @ref SMP_PAIRING_FAILED_OOB_NOT_AVAIL : Out-of-band data not available
 * - @ref SMP_PAIRING_FAILED_AUTH_REQ : Input and output capabilities of devices do not
allow for authentication
 * - @ref SMP_PAIRING_FAILED_CONFIRM_VALUE : the confirm value does not match the
calculated compare value
 * - @ref SMP_PAIRING_FAILED_NOT_SUPPORTED : pairing is unsupported
 * - @ref SMP_PAIRING_FAILED_ENC_KEY_SIZE : encryption key size is insufficient
 * - @ref SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED : The SMP command received is
unsupported on this device
 * - @ref SMP_PAIRING_FAILED_UNSPECIFIED : encryption failed to start
 * - @ref bleTimeout : pairing failed to complete before timeout
 * - @ref bleGAPBondRejected : keys did not match
 */
#define GAPBOND_PAIRING_STATE_COMPLETE            0x01

/**
 * @brief Bonding Complete
 *
 * The following statuses are possible for this state:
 * - @ref SUCCESS : pairing has been initiated. A pairing request has been either sent or received.
 * - @ref HCI_ERROR_CODE_PIN_KEY_MISSING : encryption key is missing
 * - @ref HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE : feature is unsupported by the remote
device
 * - @ref HCI_ERROR_CODE_LMP_LL_RESP_TIMEOUT : Timeout waiting for response
 * - @ref bleGAPBondRejected (0x32): this is received due to one of the previous three errors
 */
#define GAPBOND_PAIRING_STATE_BONDED              0x02
#define GAPBOND_PAIRING_STATE_BOND_SAVED          0x03  //!< Bonding record saved in NV
#define GAPBOND_PAIRING_STATE_CAR_READ            0x04  //!< Central Address Resolution Characteristic Read
/** @} End GAPBondMgr_Pairing_States */

/** @defgroup GAPBondMgr_Pairing_Failed Pairing failure status values
 * @{
 */
#define SMP_PAIRING_FAILED_PASSKEY_ENTRY_FAILED   0x01 //!< The user input of the passkey failed, for example, the user cancelled the operation.
#define SMP_PAIRING_FAILED_OOB_NOT_AVAIL          0x02 //!< The OOB data is not available
#define SMP_PAIRING_FAILED_AUTH_REQ               0x03 //!< The pairing procedure can't be performed as authentication requirements can't be met due to IO capabilities of one or both devices
#define SMP_PAIRING_FAILED_CONFIRM_VALUE          0x04 //!< The confirm value doesn't match the calculated compare value
#define SMP_PAIRING_FAILED_NOT_SUPPORTED          0x05 //!< Pairing isn't supported by the device
#define SMP_PAIRING_FAILED_ENC_KEY_SIZE           0x06 //!< The resultant encryption key size is insufficient for the security requirements of this device.
#define SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED      0x07 //!< The SMP command received is not supported on this device.
#define SMP_PAIRING_FAILED_UNSPECIFIED            0x08 //!< Pairing failed due to an unspecified reason
#define SMP_PAIRING_FAILED_REPEATED_ATTEMPTS      0x09 //!< Pairing or authentication procedure is disallowed because too little time has elapsed since the last pairing request or security request.
#define SMP_PAIRING_FAILED_INVALID_PARAMETERS     0x0A //!< Indicates that the command length is invalid or that a parameter is outside of the specified range.
#define SMP_PAIRING_FAILED_DHKEY_CHECK_FAILED     0x0B //!< Indicates to the remote device that the DHKey Check value received does not match the one calculated by the local device.
#define SMP_PAIRING_FAILED_NUM_COMPARISON_FAILED  0x0C //!< Indicates that the confirm value in the numeric comparison protocol do not match.
/** @} End GAPBondMgr_Pairing_Failed */

/** @defgroup GAPBondMgr_Bond_Failure_Actions Bonding Failure Actions
 * @{
 */
#define GAPBOND_FAIL_NO_ACTION                         0x00 //!< Take no action upon unsuccessful bonding
#define GAPBOND_FAIL_INITIATE_PAIRING                  0x01 //!< Initiate pairing upon unsuccessful bonding
#define GAPBOND_FAIL_TERMINATE_LINK                    0x02 //!< Terminate link upon unsuccessful bonding
#define GAPBOND_FAIL_TERMINATE_ERASE_BONDS             0x03 //!< Terminate link and erase all existing bonds on device upon unsuccessful bonding
/** @} End GAPBondMgr_Bond_Failure_Actions */

/** @defgroup GAPBondMgr_Secure_Cxns GAP Bond Manager Secure Connections options
 * @{
 */
#define GAPBOND_SECURE_CONNECTION_NONE                 0x00 //!< Secure Connections not supported
#define GAPBOND_SECURE_CONNECTION_ALLOW                0x01 //!< Secure Connections are supported
#define GAPBOND_SECURE_CONNECTION_ONLY                 0x02 //!< Secure Connections Only Mode
/** @} End GAPBondMgr_Secure_Cxns */

/** @defgroup GAPBondMgr_ECC_Policies GAP Bond Manager ECC Re-use count before regeneration
 * @{
 * Any value in this range is acceptable, but the higher re-use count, the more an attacker can learn about the keys in each attack.
 */
#define GAPBOND_REGENERATE_ALWAYS                       0x00 //!< Regenerate after each usage
#define GAPBOND_REGENERATE_NEVER                        0xFF //!< Never regenerate, always use the same pair.
/** @} End GAPBondMgr_ECC_Policies */

// ECC Key Length
#define ECC_KEYLEN                                     32   //!< 256 bit keys

/** @} End GAPBondMgr_Constants */

/*-------------------------------------------------------------------
 * TYPEDEFS
 */

/** @defgroup GAPBondMgr_CBs GAP Bond Manager Callbacks
 * @{
 * These are functions whose pointers are passed from the application
 * to the GAPBondMgr so that the GAPBondMgr can send events to the application
 */

/**
 * @brief Passcode and Numeric Comparison Callback Function
 *
 * This callback returns to the application the peer device information when a passcode is requested during
 * the paring process or when numeric comparison is used
 *
 * @param deviceAddr Pointer to 6-byte device address which the current pairing process relates to
 * @param connectionHandle Connection handle of the current pairing process
 * @param uiInputs if TRUE, the local device should accept a passcode input.
 * @param uiOutputs if TRUE, the local device should display the passcode.
 * @param numComparison If this is a nonzero value, then it is the code that should be displayed for numeric
 * comparison pairing. If this is zero, then passcode pairing is occurring.
*/
typedef void (*pfnPasscodeCB_t)
(
  uint8_t* deviceAddr,
  uint16_t connectionHandle,
  uint8_t  uiInputs,
  uint8_t  uiOutputs,
  uint32_t numComparison
);

/**
 * @brief Pairing State Callback Function
 *
 * This callback returns the current pairing state to the application whenever the state changes and the
 * current status of the pairing or bonding process associated with the current state
 *
 * @param connectionHandle connection handle of current pairing process
 * @param state @ref GAPBondMgr_Pairing_States
 * @param status pairing status
 */
typedef void (*pfnPairStateCB_t)
(
  uint16_t connectionHandle,
  uint8_t  state,
  uint8_t  status
);

/*
 * @brief GAPBondMgr Callback Structure
 *
 * This must be setup by the application and passed to the GAPRole when
 * @ref GAPBondMgr_Register is called.
 */
typedef struct
{
  pfnPasscodeCB_t     passcodeCB;       //!< Passcode callback
  pfnPairStateCB_t    pairStateCB;      //!< Pairing state callback
} gapBondCBs_t;

/** @} End GAPBondMgr_CBs */

/** @defgroup GAPBondMgr_Structs GAP Bond Manager Structures
 * @{
 */

/// @brief Public and Private ECC Keys
typedef struct
{
  uint8_t privateKey[ECC_KEYLEN];			//!< private key
  uint8_t publicKeyX[ECC_KEYLEN];			//!< public key X
  uint8_t publicKeyY[ECC_KEYLEN];			//!< prublic key Y
} gapBondEccKeys_t;

/// @brief Secure Connections Out of Band
typedef struct
{
  uint8_t addr[B_ADDR_LEN];    //!< BLE connection address
  uint8_t confirm[KEYLEN]; //!< Confirm value F4(PKx, PKx, oob, 0).
  uint8_t oob[KEYLEN];     //!< oob data.
} gapBondOobSC_t;

/** @} End GAPBondMgr_Structs */

/*-------------------------------------------------------------------
 * MACROS
 */

/*-------------------------------------------------------------------
 * API FUNCTIONS
 */

/**
 * @brief       Set a GAP Bond Manager parameter.
 *
 * @note
 * The "len" field must be set to the size of a "uint16_t" and the
 * "pValue" field must point to a "uint16_t".
 *
 * @param param - @ref GAPBondMgr_Params
 * @param len - length of data to write
 * @param pValue - pointer to data to write.  This is dependent on
 *        the parameter ID and WILL be cast to the appropriate
 *        data type (example: data type of uint16_t will be cast to
 *        uint16_t pointer).
 *
 * @return  @ref SUCCESS
 * @return  @ref INVALIDPARAMETER
 */
extern bStatus_t GAPBondMgr_SetParameter(uint16_t param, uint8_t len, void *pValue);

/**
 * @brief       Get a GAP Bond Manager parameter.
 *
 * @note
 * The "pValue" field must point to a "uint16_t".
 *
 * @param param - Profile parameter ID: @ref GAPBondMgr_Params
 * @param pValue - pointer to location to get the value.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return @ref SUCCESS
 * @return @ref INVALIDPARAMETER
 */
extern bStatus_t GAPBondMgr_GetParameter(uint16_t param, void *pValue);

/**
 * @brief       Notify the Bond Manager that a connection has been made.
 *
 * @note
 * The GAP Role profile will call this function
 *
 * @param       addrType - @ref Addr_type
 * @param       pDevAddr - device's address
 * @param       connHandle - connection handle
 * @param       role - @ref GAP_Profile_Roles
 *
 * @return      @ref SUCCESS
 * @return      @ref FAILURE
 */
extern bStatus_t GAPBondMgr_LinkEst(uint8_t addrType, uint8_t *pDevAddr, uint16_t connHandle, uint8_t role);

/**
 * @brief       Notify the Bond Manager that a connection has been terminated.
 *
 * @note
 * The GAP Role profile will call this function
 *
 * @param       connHandle - connection handle
 */
extern void GAPBondMgr_LinkTerm(uint16_t connHandle);

/**
 * @brief       Notify the Bond Manager that a Slave Security Request is received.
 *
 * @note
 * The GAP Role profile will call this function if configured to do so
 *
 * @param       connHandle - connection handle
 * @param       authReq    - slave device's authentication requirements
 */
extern void GAPBondMgr_SlaveReqSecurity(uint16_t connHandle, uint8_t authReq);

/**
 * @brief       Resolve an address from bonding information.
 *
 * @param       addrType - @ref Addr_type
 * @param       pDevAddr - peer's address
 * @param       pResolvedAddr - pointer to buffer to put the resolved address
 *
 * @return      bonding index (0 - (GAP_BONDINGS_MAX-1) if found
 * @return      GAP_BONDINGS_MAX if not found
 */
extern uint8_t GAPBondMgr_ResolveAddr(uint8_t addrType, uint8_t *pDevAddr, uint8_t *pResolvedAddr);

/**
 * @brief       Set/clear the service change indication in a bond record.
 *
 * @param       connectionHandle - connection handle of the connected device or 0xFFFF
 *                                 if all devices in database.
 * @param       setParam - TRUE to set the service change indication, FALSE to clear it.
 *
 * @return      @ref SUCCESS - bond record found and changed,
 * @return      @ref bleNoResources - bond record not found (for 0xFFFF connectionHandle),
 * @return      @ref bleNotConnected - connection not found - connectionHandle is invalid (for non-0xFFFF connectionHandle).
 */
extern bStatus_t GAPBondMgr_ServiceChangeInd(uint16_t connectionHandle, uint8_t setParam);

/**
 * @brief       Update the Characteristic Configuration in a bond record.
 *
 * @param       connectionHandle - connection handle of the connected device or 0xFFFF
 *                                 if all devices in database.
 * @param       attrHandle - attribute handle.
 * @param       value - characteristic configuration value.
 *
 * @return      @ref SUCCESS - bond record found and changed,
 * @return      @ref bleNoResources - bond record not found (for 0xFFFF connectionHandle),
 * @return      @ref bleNotConnected - connection not found - connectionHandle is invalid (for non-0xFFFF connectionHandle).
 */
extern bStatus_t GAPBondMgr_UpdateCharCfg(uint16_t connectionHandle, uint16_t attrHandle, uint16_t value);

/**
 * @brief       Register callback functions with the bond manager.
 *
 * @warning It is the application's responsibility to register a callback
 * function if it is requesting / accepting authenticated pairing.
 *
 * @param       pCB - pointer to callback function structure.
 */
extern void GAPBondMgr_Register(gapBondCBs_t *pCB);

/**
 * @brief       Respond to a passcode request.
 *
 * @param       connectionHandle - connection handle of the connected device or 0xFFFF
 *                                 if all devices in database.
 * @param       status - SUCCESS if passcode is available, otherwise see @ref GAPBondMgr_Pairing_Failed.
 * @param       passcode - integer value containing the passcode.
 *
 * @return      @ref SUCCESS - bond record found and changed,
 * @return      @ref bleIncorrectMode - Link not found.
 * @return      @ref INVALIDPARAMETER : passcode is out of range
 * @return      @ref bleMemAllocError : heap is out of memory
 */
extern bStatus_t GAPBondMgr_PasscodeRsp(uint16_t connectionHandle, uint8_t status, uint32_t passcode);

/**
 * @brief       Read Central Address Resolution Characteristic
 *
 * Send Read By Type Request to get value attribute of Central
 *              Address Resolution characteristic. Value determines if peer
 *              supports Enhanced Privacy. Bond record will automatically be
 *              updated based on peer response
 *
 * @param       connectionHandle - connection handle of the connected device
 *
 * @return      @ref SUCCESS : Request was sent successfully.
 * @return      @ref INVALIDPARAMETER
 * @return      @ref MSG_BUFFER_NOT_AVAIL.
 * @return      @ref bleNotConnected
 * @return      @ref blePending : A response is pending with this server.
 * @return      @ref bleMemAllocError
 * @return      @ref bleTimeout : Previous transaction timed out.
 */
extern bStatus_t GAPBondMgr_ReadCentAddrResChar(uint16_t connectionHandle);

/**
 * @brief       Determine if peer device supports enhanced privacy by checking
 *              the Enhanced Privacy state flag of the bond record that
 *              corresponds to the peer's identity address
 *
 * @param       pPeerIdAddr - peer identity address
 *
 * @return      TRUE: Peer supports enhanced privacy.
 * @return      FALSE: Peer does not support enhanced privacy.
 */
extern uint8_t GAPBondMgr_SupportsEnhancedPriv(uint8_t *pPeerIdAddr);

/**
 * @brief       This is a bypass mechanism to allow the bond manager to process
 *              GAP messages.
 *
 * @warning  This is an advanced feature and shouldn't be called unless
 *              the normal GAP Bond Manager task ID registration is overridden.
 *
 * @param       pMsg - GAP event message
 *
 * @return      TRUE if safe to deallocate incoming GAP message
 * @return      FALSE otherwise.
 */
extern uint8_t GAPBondMgr_ProcessGAPMsg(gapEventHdr_t *pMsg);

/**
 * @brief       This function will check the length of a Bond Manager NV Item.
 *
 * @param       id - NV ID.
 * @param       len - lengths in bytes of item.
 *
 * @return      @ref SUCCESS
 * @return      @ref FAILURE
 */
extern uint8_t GAPBondMgr_CheckNVLen(uint8_t id, uint8_t len);

/**
 * @brief       This function will add all device address and IRK pairs from
 *              bond records to the Controller. Used with PRIVACY_1_2_CFG
 *
 * @return      @ref SUCCESS
 * @return      @ref FAILURE
 */
extern bStatus_t GAPBondMgr_syncResolvingList(void);

/// @cond NODOC

/*-------------------------------------------------------------------
 * TASK FUNCTIONS - Don't call these. These are system functions.
 */

/**
 * @internal
 *
 * @brief       Initialization function for the GAP Bond Manager Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param       the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 */
extern void GAPBondMgr_Init(uint8_t task_id);

/**
 * @internal
 *
 * @brief       GAP Bond Manager Task event processor.
 *          This function is called to process all events for the task.
 *          Events include timers, messages and any other user defined
 *          events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return      events not processed
 */
extern uint16_t GAPBondMgr_ProcessEvent(uint8_t task_id, uint16_t events);

/// @endcond // NODOC

/*-------------------------------------------------------------------
-------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* GAPBONDMGR_H */

/** @} End GAPBondMgr */
