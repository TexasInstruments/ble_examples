/******************************************************************************

 @file  gapbondmgr.c

 @brief GAP peripheral profile manages bonded connections

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2011-2020, Texas Instruments Incorporated
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

#if (HOST_CONFIG & (CENTRAL_CFG | PERIPHERAL_CFG))

#if defined(GAP_BOND_MGR) && (defined(NO_OSAL_SNV) || (defined(OSAL_SNV) && (OSAL_SNV == 0)))
  #error "Bond Manager cannot be used since NO_OSAL_SNV used! Disable in buildConfig.opt"
#endif

/*
 * When GATT_NO_CLIENT is used, the use of GATT Client API is compiled out under
 * GBM_GATT_NO_CLIENT.  This means that, in the context of Privacy 1.2, the Bond
 * Manager of this device will not read the Central Address Resolution
 * Characteristic of the remote device.  If it is desired that this device uses
 * a Private Resolvable Address for Directed Advertisements, comment out the
 * pre-processor logic below.
 */
#ifdef GATT_NO_CLIENT
  #ifndef GBM_GATT_NO_CLIENT
    #define GBM_GATT_NO_CLIENT
  #endif // !GBM_GATT_NO_CLIENT
#endif // GATT_NO_CLIENT

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "osal.h"
#include "osal_snv.h"
#include "l2cap.h"
#include "sm.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "hci.h"
#include "gattservapp.h"
#include "gapgattserver.h"
#include "gapbondmgr.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Task event types
#define GAP_BOND_SYNC_CC_EVT                            0x0001 // Sync char config
#define GAP_BOND_SAVE_REC_EVT                           0x0002 // Save bond record in NV
#define GAP_BOND_SAVE_RCA_EVT                           0x0004 // Save reconnection address in NV
#define GAP_BOND_POP_PAIR_QUEUE_EVT                     0x0008 // Begin pairing with the next queued device
#define GAP_BOND_SYNC_RL_EVT                            0x0010 // Sync Resolving List

// Once NV usage reaches this percentage threshold, NV compaction gets triggered.
#define NV_COMPACT_THRESHOLD                            80

// Secure Connections minimum MTU size
#define SECURECONNECTION_MIN_MTU_SIZE                   65

// Central Device should only contain one Central Address Resolution Characteristic
#define NUM_CENT_ADDR_RES_CHAR                          1

// Central Address Resolution value shall be one octet in length. The read response
// should be 3 total octets: 2 octets for attribute handle and 1 octet for value
#define CENT_ADDR_RES_RSP_LEN                           3

/**
 * GAP Bond Manager NV layout
 *
 * The NV definitions:
 *     BLE_NVID_GAP_BOND_START - starting NV ID
 *     GAP_BONDINGS_MAX - Maximum number of bonding allowed (10 is max for number of NV IDs allocated in bcomdef.h).
 *
 * A single bonding entry consists of 6 components (NV items):
 *     Bond Record - defined as gapBondRec_t and uses GAP_BOND_REC_ID_OFFSET for an NV ID
 *     local LTK Info - defined as gapBondLTK_t and uses GAP_BOND_LOCAL_LTK_OFFSET for an NV ID
 *     device LTK Info - defined as gapBondLTK_t and uses GAP_BOND_DEV_LTK_OFFSET for an NV ID
 *     device IRK - defined as "uint8_t devIRK[KEYLEN]" and uses GAP_BOND_DEV_IRK_OFFSET for an NV ID
 *     device CSRK - defined as "uint8_t devCSRK[KEYLEN]" and uses GAP_BOND_DEV_CSRK_OFFSET for an NV ID
 *     device Sign Counter - defined as a uint32_t and uses GAP_BOND_DEV_SIGN_COUNTER_OFFSET for an NV ID
 *
 * When the device is initialized for the first time, all (GAP_BONDINGS_MAX) NV items are created and
 * initialized to all 0xFF's. A bonding record of all 0xFF's indicates that the bonding record is empty
 * and free to use.
 *
 * The calculation for each bonding records NV IDs:
 *    mainRecordNvID = ((bondIdx * GAP_BOND_REC_IDS) + BLE_NVID_GAP_BOND_START)
 *    localLTKNvID = (((bondIdx * GAP_BOND_REC_IDS) + GAP_BOND_LOCAL_LTK_OFFSET) + BLE_NVID_GAP_BOND_START)
 *
 */
#ifdef SNP_SECURITY
#define GAP_BOND_REC_ID_OFFSET              0 //!< NV ID for the main bonding record
#define GAP_BOND_LOCAL_LTK_OFFSET           1 //!< NV ID for the bonding record's local LTK information
#define GAP_BOND_DEV_IRK_OFFSET             2 //!< NV ID for the bonding records' device IRK

#define GAP_BOND_REC_IDS                    3
#else //!SNP_SECURITY
#define GAP_BOND_REC_ID_OFFSET              0 //!< NV ID for the main bonding record
#define GAP_BOND_LOCAL_LTK_OFFSET           1 //!< NV ID for the bonding record's local LTK information
#define GAP_BOND_DEV_LTK_OFFSET             2 //!< NV ID for the bonding records' device LTK information
#define GAP_BOND_DEV_IRK_OFFSET             3 //!< NV ID for the bonding records' device IRK
#define GAP_BOND_DEV_CSRK_OFFSET            4 //!< NV ID for the bonding records' device CSRK
#define GAP_BOND_DEV_SIGN_COUNTER_OFFSET    5 //!< NV ID for the bonding records' device Sign Counter

#define GAP_BOND_REC_IDS                    6
#endif //SNP_SECURITY

// Macros to calculate the index/offset in to NV space
#define calcNvID(Idx, offset)               (((((Idx) * GAP_BOND_REC_IDS) + (offset))) + BLE_NVID_GAP_BOND_START)
#define mainRecordNvID(bondIdx)             (calcNvID((bondIdx), GAP_BOND_REC_ID_OFFSET))
#define localLTKNvID(bondIdx)               (calcNvID((bondIdx), GAP_BOND_LOCAL_LTK_OFFSET))
#define devLTKNvID(bondIdx)                 (calcNvID((bondIdx), GAP_BOND_DEV_LTK_OFFSET))
#define devIRKNvID(bondIdx)                 (calcNvID((bondIdx), GAP_BOND_DEV_IRK_OFFSET))
#define devCSRKNvID(bondIdx)                (calcNvID((bondIdx), GAP_BOND_DEV_CSRK_OFFSET))
#define devSignCounterNvID(bondIdx)         (calcNvID((bondIdx), GAP_BOND_DEV_SIGN_COUNTER_OFFSET))

// Macros to calculate the GATT index/offset in to NV space
#define gattCfgNvID(Idx)                    ((Idx) + BLE_NVID_GATT_CFG_START)

// Key Size Limits
#define MIN_ENC_KEYSIZE                     7  //!< Minimum number of bytes for the encryption key
#define MAX_ENC_KEYSIZE                     16 //!< Maximum number of bytes for the encryption key

// Bonded State Flags
#define GAP_BONDED_STATE_AUTHENTICATED                  0x0001
#define GAP_BONDED_STATE_SERVICE_CHANGED                0x0002
#define GAP_BONDED_STATE_ENHANCED_PRIVACY               0x0004
#define GAP_BONDED_STATE_SECURECONNECTION               0x0008

/*********************************************************************
 * TYPEDEFS
 */

// Structure of NV data for the connected device's encryption information
typedef struct
{
  uint8_t   LTK[KEYLEN];              // Long Term Key (LTK)
  uint16_t  div;  //lint -e754        // LTK eDiv
  uint8_t   rand[B_RANDOM_NUM_SIZE];  // LTK random number
  uint8_t   keySize;                  // LTK key size
} gapBondLTK_t;

// Structure of NV data for the connected device's address information
typedef struct
{
  uint8_t   publicAddr[B_ADDR_LEN];     // Peer's address
  uint8_t   publicAddrType;             // Peer's address type
  uint8_t   reconnectAddr[B_ADDR_LEN];  // Privacy Reconnection Address
  uint8_t   stateFlags;                 // State flags: @ref GAP_BONDED_STATE_FLAGS
} gapBondRec_t;

// Structure of NV data for the connected device's characteristic configuration
typedef struct
{
  uint16_t attrHandle;  // attribute handle
  uint8_t  value;       // attribute value for this device
} gapBondCharCfg_t;

typedef struct pairQueue
{
  uint16_t           connHandle;
  uint8_t            addrType;
  gapPairingReq_t *pPairReq;
  struct pairQueue *pNext;
} gapBondPairQueueNode_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

extern uint8_t gapState;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t gapBondMgr_TaskID;   // Task ID for internal task/event processing

// GAPBonding Parameters
static uint8_t gapBond_PairingMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
static uint16_t gapBond_InitiateWait = 1000;  // Default to 1 second
static uint8_t gapBond_MITM = FALSE;
static uint8_t gapBond_IOCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
static uint8_t gapBond_OOBDataFlag = FALSE;
static uint8_t gapBond_OOBData[KEYLEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & SECURE_CONNS_CFG)
static uint8_t gapBond_OobScDataFlag = FALSE;
static uint8_t gapBond_localOobScDataFlag = FALSE;
static gapBondOobSC_t gapBond_remoteOobSCData =
{
  {0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};
static uint8_t gapBond_localOobSCData[KEYLEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#endif //(BLE_V42_FEATURES & SECURE_CONNS_CFG)
static uint8_t gapBond_Bonding = FALSE;
static uint8_t gapBond_AutoFail = FALSE;
static uint8_t gapBond_AutoFailReason = SMP_PAIRING_FAILED_NOT_SUPPORTED;
static uint8_t gapBond_KeyDistList =
(
  GAPBOND_KEYDIST_SENCKEY     // sEncKey enabled, to send the encryption key
   | GAPBOND_KEYDIST_SIDKEY   // sIdKey enabled, to send the IRK, and BD_ADDR
   | GAPBOND_KEYDIST_SSIGN    // sSign enabled, to send the CSRK
   | GAPBOND_KEYDIST_MENCKEY  // mEncKey enabled, to get the master's encryption key
   | GAPBOND_KEYDIST_MIDKEY   // mIdKey enabled, to get the master's IRK and BD_ADDR
   | GAPBOND_KEYDIST_MSIGN    // mSign enabled, to get the master's CSRK
);
static uint8_t  gapBond_KeySize = MAX_ENC_KEYSIZE;

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & SECURE_CONNS_CFG)
static uint8_t gapBond_secureConnection = GAPBOND_SECURE_CONNECTION_ALLOW;

#ifdef SC_HOST_DEBUG
static uint8_t gapBond_useEccKeys = TRUE;
// These are the "Debug Mode" keys as defined in
static gapBondEccKeys_t gapBond_eccKeys =
{
  { 0xBD, 0x1A, 0x3C, 0xCD, 0xA6, 0xB8, 0x99, 0x58, 0x99, 0xB7, 0x40, 0xEB,
   0x7B, 0x60, 0xFF, 0x4A, 0x50, 0x3F, 0x10, 0xD2, 0xE3, 0xB3, 0xC9, 0x74,
   0x38, 0x5F, 0xC5, 0xA3, 0xD4, 0xF6, 0x49, 0x3F },

  { 0xE6, 0x9D, 0x35, 0x0E, 0x48, 0x01, 0x03, 0xCC, 0xDB, 0xFD, 0xF4, 0xAC,
    0x11, 0x91, 0xF4, 0xEF, 0xB9, 0xA5, 0xF9, 0xE9, 0xA7, 0x83, 0x2C, 0x5E,
    0x2C, 0xBE, 0x97, 0xF2, 0xD2, 0x03, 0xB0, 0x20 },

  { 0x8B, 0xD2, 0x89, 0x15, 0xD0, 0x8E, 0x1C, 0x74, 0x24, 0x30, 0xED, 0x8F,
    0xC2, 0x45, 0x63, 0x76, 0x5C, 0x15, 0x52, 0x5A, 0xBF, 0x9A, 0x32, 0x63,
    0x6D, 0xEB, 0x2A, 0x65, 0x49, 0x9C, 0x80, 0xDC }
};
#else
static uint8_t gapBond_useEccKeys = FALSE;
static gapBondEccKeys_t gapBond_eccKeys =
{
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};
#endif //SC_HOST_DEBUG

#endif //(BLE_V42_FEATURES & SECURE_CONNS_CFG)

#if (HOST_CONFIG & CENTRAL_CFG)
static uint8_t  gapBond_BondFailOption = GAPBOND_FAIL_TERMINATE_LINK;
#endif

static const gapBondCBs_t *pGapBondCB = NULL;

// Local RAM shadowed bond records
static gapBondRec_t bonds[GAP_BONDINGS_MAX] = {0};

static uint8_t autoSyncWhiteList = FALSE;

static uint8_t eraseAllBonds = FALSE;

static uint8_t bondsToDelete[GAP_BONDINGS_MAX] = {FALSE};

// Globals used for saving bond record and CCC values in NV
static uint8_t bondIdx = GAP_BONDINGS_MAX;
static gapAuthCompleteEvent_t *pAuthEvt = NULL;

#if (HOST_CONFIG & PERIPHERAL_CFG)

#if defined (GAP_PRIVACY_RECONNECT)
// Global used for saving reconnection address in NV
static uint8_t reconnectAddrIdx = GAP_BONDINGS_MAX;
#endif // GAP_PRIVACY_RECONNECT

#endif // PERIPHERAL_CFG

#ifdef  GBM_QUEUE_PAIRINGS
gapBondPairQueueNode_t *pPairingQ = NULL;
#endif //GBM_QUEUE_PAIRINGS

uint8_t gapBond_removeLRUBond = FALSE;
uint8_t gapBond_lruBondList[GAP_BONDINGS_MAX] = {0};

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
// Global used to indicate whether Resolving List must be resynched with
// bond records once controller is no longer adv/init/scanning
uint8_t gapBond_syncRL = FALSE;
#endif // BLE_V42_FEATURES & PRIVACY_1_2_CFG

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8_t gapBondMgrUpdateCharCfg(uint8_t idx, uint16_t attrHandle, uint16_t value);
static gapBondCharCfg_t *gapBondMgrFindCharCfgItem(uint16_t attrHandle,
                                                    gapBondCharCfg_t *charCfgTbl);
static void gapBondMgrInvertCharCfgItem(gapBondCharCfg_t *charCfgTbl);
static uint8_t gapBondMgrAddBond(gapBondRec_t *pBondRec, gapAuthCompleteEvent_t *pPkt);
static uint8_t gapBondMgrGetStateFlags(uint8_t idx);
static bStatus_t gapBondMgrGetPublicAddr(uint8_t idx, uint8_t *pAddr);
static uint8_t gapBondMgrFindReconnectAddr(uint8_t *pReconnectAddr);
static uint8_t gapBondMgrFindAddr(uint8_t *pDevAddr);
static void gapBondMgrReadBonds(void);
static uint8_t gapBondMgrFindEmpty(void);
static uint8_t gapBondMgrBondTotal(void);
static bStatus_t gapBondMgrEraseAllBondings(void);
static bStatus_t gapBondMgrEraseBonding(uint8_t idx);
static uint8_t gapBondMgr_ProcessOSALMsg(osal_event_hdr_t *pMsg);
static void gapBondMgr_ProcessGATTMsg(gattMsgEvent_t *pMsg);
static void gapBondMgr_ProcessGATTServMsg(gattEventHdr_t *pMsg);
static void gapBondSetupPrivFlag(void);
static void gapBondMgrBondReq(uint16_t connHandle, uint8_t idx, uint8_t stateFlags,
                               uint8_t role, uint8_t startEncryption);
static bStatus_t gapBondMgrAuthenticate(uint16_t connHandle, uint8_t addrType,
                                         gapPairingReq_t *pPairReq);
static void gapBondMgr_SyncWhiteList(void);
static uint8_t gapBondMgr_SyncCharCfg(uint16_t connHandle);
static void gapBondFreeAuthEvt(void);
static uint8_t gapBondGetRandomAddrSubType(uint8_t *pAddr);

static void gapBondMgrReadLruBondList(void);
static uint8_t gapBondMgrGetLruBondIndex(void);
static void gapBondMgrUpdateLruBondList(uint8_t bondIndex);

#if !defined (GATT_NO_SERVICE_CHANGED) || \
    (!defined GBM_GATT_NO_CLIENT && \
      ((defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)) && \
        (HOST_CONFIG & PERIPHERAL_CFG)))
static uint8_t gapBondMgrChangeState(uint8_t idx, uint16_t state, uint8_t set);
#endif // !GATT_NO_SERVICE_CHANGED || \
       // (!GBM_GATT_NO_CLIENT && (BLE_V42_FEATURES & PRIVACY_1_2_CFG & PERIPHERAL_CFG))

#if (HOST_CONFIG & PERIPHERAL_CFG)
static void gapBondMgrSlaveSecurityReq(uint16_t connHandle);

#if defined (GAP_PRIVACY_RECONNECT)
static uint8_t gapBondMgrUpdateReconnectAddr(uint8_t idx);
static void gapBondMgrAttrValueChangeCB(uint16_t connHandle, uint8_t attrId);
#endif // GAP_PRIVACY_RECONNECT

#endif // PERIPHERAL_CFG

#ifndef GATT_NO_SERVICE_CHANGED
static void gapBondMgrSendServiceChange(linkDBItem_t *pLinkItem);
#endif // GATT_NO_SERVICE_CHANGED

#if !defined (BLE_V42_FEATURES) || !(BLE_V42_FEATURES & PRIVACY_1_2_CFG)
static uint8_t gapBondMgrResolvePrivateAddr(uint8_t *pAddr);
#endif // ! BLE_V42_FEATURES | ! PRIVACY_1_2_CFG

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
static void gapBondMgr_gapIdle(void);
#if defined (HOST_CONFIG) && (HOST_CONFIG & PERIPHERAL_CFG)
#ifndef GBM_GATT_NO_CLIENT
static void gapBondMgrReadCARRsp(uint16_t connHandle, attReadByTypeRsp_t rsp);
static bStatus_t gapBondMgrSetEnhancedPrivFlag(uint16_t connHandle, uint8_t setParam);
static void gapBondMgr_ProcessAttErrRsp(attErrorRsp_t rsp);
#endif // !GBM_GATT_NO_CLIENT
#endif // HOST_CONFIG & PERIPHERAL_CFG
#endif // BLE_V42_FEATURES & PRIVACY_1_2_CFG

#ifdef GBM_QUEUE_PAIRINGS
static void gapBondMgrQueuePairing(uint16_t connHandle, uint8_t addrType,
                                    gapPairingReq_t *pPairReq);
#endif //GBM_QUEUE_PAIRINGS

/*********************************************************************
 * HOST and GAP CALLBACKS
 */

static gapBondMgrCBs_t gapCBs =
{
#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
  .supportEnhPrivCB = GAPBondMgr_SupportsEnhancedPriv,
  .gapIdleCB = gapBondMgr_gapIdle
#else
  .supportEnhPrivCB = NULL,
  .gapIdleCB = NULL
#endif // BLE_V42_FEATURES & PRIVACY_1_2_CFG
};

#if (HOST_CONFIG & PERIPHERAL_CFG)
#if defined (GAP_PRIVACY_RECONNECT)
// Bond Manager's callback
static ggsAppCBs_t gapBondMgrCB =
{
  gapBondMgrAttrValueChangeCB
};
#endif // GAP_PRIVACY_RECONNECT
#endif // PERIPHERAL_CFG

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @brief   Set a GAP Bond Manager parameter.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_SetParameter(uint16_t param, uint8_t len, void *pValue)
{
  bStatus_t ret = SUCCESS;  // return value

  switch (param)
  {
    case GAPBOND_PAIRING_MODE:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= GAPBOND_PAIRING_MODE_INITIATE))
      {
        gapBond_PairingMode = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_INITIATE_WAIT:
      if (len == sizeof (uint16_t))
      {
        gapBond_InitiateWait = *((uint16_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_MITM_PROTECTION:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= TRUE))
      {
        gapBond_MITM = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_IO_CAPABILITIES:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= GAPBOND_IO_CAP_KEYBOARD_DISPLAY))
      {
        gapBond_IOCap = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_OOB_ENABLED:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= TRUE))
      {
        gapBond_OOBDataFlag = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_OOB_DATA:
      if (len == KEYLEN)
      {
        VOID osal_memcpy(gapBond_OOBData, pValue, KEYLEN) ;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_BONDING_ENABLED:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= TRUE))
      {
        gapBond_Bonding = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_KEY_DIST_LIST:
      if (len == sizeof (uint8_t))
      {
        gapBond_KeyDistList = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_ERASE_ALLBONDS:
      if (len == 0)
      {
        // Make sure there's no active connection
        if (GAP_NumActiveConnections() == 0)
        {
          // Erase all bonding records
          VOID gapBondMgrEraseAllBondings();

          // See if NV needs a compaction
          VOID osal_snv_compact(NV_COMPACT_THRESHOLD);

          // Make sure Bond RAM Shadow is up-to-date
          gapBondMgrReadBonds();
        }
        else
        {
          eraseAllBonds = TRUE;
        }
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_ERASE_SINGLEBOND:
      if (len == (1 + B_ADDR_LEN))
      {
        uint8_t idx;
        uint8_t devAddr[B_ADDR_LEN];

        // Reverse bytes
        VOID osal_revmemcpy(devAddr, (uint8_t *)pValue+1, B_ADDR_LEN);

        // Resolve address and find index
        idx = GAPBondMgr_ResolveAddr(*((uint8_t *)pValue), devAddr, NULL);
        if (idx < GAP_BONDINGS_MAX)
        {
          // Make sure there's no active connection
          if (GAP_NumActiveConnections() == 0)
          {
            // Erase bond
            VOID gapBondMgrEraseBonding(idx);

            // See if NV needs a compaction
            VOID osal_snv_compact(NV_COMPACT_THRESHOLD);

            // Make sure Bond RAM Shadow is up-to-date
            gapBondMgrReadBonds();
          }
          else
          {
            // Mark entry to be deleted when disconnected
            bondsToDelete[idx] = TRUE;
          }
        }
        else
        {
          ret = INVALIDPARAMETER;
        }
      }
      else
      {
        // Parameter is not the correct length
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_AUTO_FAIL_PAIRING:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= TRUE))
      {
        gapBond_AutoFail = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_AUTO_FAIL_REASON:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= SMP_PAIRING_FAILED_REPEATED_ATTEMPTS))
      {
        gapBond_AutoFailReason = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_KEYSIZE:
      if ((len == sizeof (uint8_t))
          && ((*((uint8_t*)pValue) >= MIN_ENC_KEYSIZE) && (*((uint8_t*)pValue) <= MAX_ENC_KEYSIZE)))
      {
        gapBond_KeySize = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_AUTO_SYNC_WL:
      if (len == sizeof(uint8_t))
      {
        uint8_t oldVal = autoSyncWhiteList;

        autoSyncWhiteList = *((uint8_t *)pValue);

        // only call if parameter changes from FALSE to TRUE
        if ((oldVal == FALSE) && (autoSyncWhiteList == TRUE))
        {
          // make sure bond is updated from NV
          gapBondMgrReadBonds();
        }
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

#if (HOST_CONFIG & CENTRAL_CFG)
    case GAPBOND_BOND_FAIL_ACTION:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= GAPBOND_FAIL_TERMINATE_ERASE_BONDS))
      {
        gapBond_BondFailOption = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
#endif

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & SECURE_CONNS_CFG)
    case GAPBOND_SECURE_CONNECTION:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= GAPBOND_SECURE_CONNECTION_ONLY))
      {
        gapBond_secureConnection = *((uint8_t*)pValue);

        // Update linkDB so the security mode can be enforced.
        if (gapBond_secureConnection  == GAPBOND_SECURE_CONNECTION_ONLY)
        {
          linkDB_SecurityModeSCOnly(TRUE);
        }
        else
        {
          linkDB_SecurityModeSCOnly(FALSE);
        }
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_ECCKEY_REGEN_POLICY:
      if (len == sizeof(uint8_t))
      {
        SM_SetECCRegenerationCount(*(uint8_t *)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_ECC_KEYS:
#ifndef SC_HOST_DEBUG
      if ((len == sizeof(gapBondEccKeys_t) && pValue != 0))
      {
        // Copy.
        osal_memcpy(&gapBond_eccKeys, (gapBondEccKeys_t *)pValue,
                    sizeof(gapBondEccKeys_t));

        // Use these keys for future pairings.
        gapBond_useEccKeys = TRUE;
      }
      else if (len == sizeof(uint8_t) && pValue != 0)
      {
        gapBond_useEccKeys = *((uint8_t *)pValue);
      }
      else
#endif //SC_HOST_DEBUG
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_REMOTE_OOB_SC_ENABLED:
      if (len == sizeof(uint8_t) && ((uint8_t *)pValue != NULL))
      {
        gapBond_OobScDataFlag = *(uint8_t *)pValue;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    case GAPBOND_REMOTE_OOB_SC_DATA:
      if (len == sizeof (gapBondOobSC_t) && ((gapBondOobSC_t *)pValue != NULL))
      {
        VOID osal_memcpy(&gapBond_remoteOobSCData, pValue, sizeof(gapBondOobSC_t)) ;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_LOCAL_OOB_SC_ENABLED:
      if (len == sizeof(uint8_t) && ((uint8_t *)pValue != NULL))
      {
        gapBond_localOobScDataFlag = *(uint8_t *)pValue;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    case GAPBOND_LOCAL_OOB_SC_DATA:
      if (len == sizeof(uint8_t) * KEYLEN && ((uint8_t *)pValue != NULL))
      {
        VOID osal_memcpy(gapBond_localOobSCData, pValue, sizeof(uint8_t) * KEYLEN);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
#endif //(BLE_V42_FEATURES & SECURE_CONNS_CFG)

    case GAPBOND_LRU_BOND_REPLACEMENT:
      if (len == sizeof(uint8_t) && ((uint8_t *)pValue != NULL))
      {
        gapBond_removeLRUBond = *(uint8_t *)pValue;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      // The param value isn't part of this profile, try the GAP.
      if ((param < TGAP_PARAMID_MAX) && (len == sizeof (uint16_t)))
      {
        ret = GAP_SetParamValue(param, *((uint16_t*)pValue));
      }
      else
      {
        ret = INVALIDPARAMETER;
      }
      break;
  }

  return (ret);
}

/*********************************************************************
 * @brief   Get a GAP Bond Manager parameter.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_GetParameter(uint16_t param, void *pValue)
{
  bStatus_t ret = SUCCESS;  // return value

  switch (param)
  {
    case GAPBOND_PAIRING_MODE:
      *((uint8_t*)pValue) = gapBond_PairingMode;
      break;

    case GAPBOND_INITIATE_WAIT:
      *((uint16_t*)pValue) = gapBond_InitiateWait;
      break;

    case GAPBOND_MITM_PROTECTION:
      *((uint8_t*)pValue) = gapBond_MITM;
      break;

    case GAPBOND_IO_CAPABILITIES:
      *((uint8_t*)pValue) = gapBond_IOCap;
      break;

    case GAPBOND_OOB_ENABLED:
      *((uint8_t*)pValue) = gapBond_OOBDataFlag;
      break;

    case GAPBOND_OOB_DATA:
      VOID osal_memcpy(pValue, gapBond_OOBData, KEYLEN) ;
      break;

    case GAPBOND_BONDING_ENABLED:
      *((uint8_t*)pValue) = gapBond_Bonding;
      break;

    case GAPBOND_KEY_DIST_LIST:
      *((uint8_t*)pValue) = gapBond_KeyDistList;
      break;

    case GAPBOND_AUTO_FAIL_PAIRING:
      *((uint8_t*)pValue) = gapBond_AutoFail;
      break;

    case GAPBOND_AUTO_FAIL_REASON:
      *((uint8_t*)pValue) = gapBond_AutoFailReason;
      break;

    case GAPBOND_KEYSIZE:
      *((uint8_t*)pValue) = gapBond_KeySize;
      break;

    case GAPBOND_AUTO_SYNC_WL:
      *((uint8_t*)pValue) = autoSyncWhiteList;
      break;

    case GAPBOND_BOND_COUNT:
      *((uint8_t*)pValue) = gapBondMgrBondTotal();
      break;

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & SECURE_CONNS_CFG)
    case GAPBOND_SECURE_CONNECTION:
      *((uint8_t*)pValue) = gapBond_secureConnection;
      break;

    case GAPBOND_ECC_KEYS:
      VOID osal_memcpy(pValue, &gapBond_eccKeys, sizeof(gapBondEccKeys_t));
      break;

    case GAPBOND_REMOTE_OOB_SC_ENABLED:
      *((uint8_t *)pValue) = gapBond_OobScDataFlag;
      break;

    case GAPBOND_REMOTE_OOB_SC_DATA:
      osal_memcpy(pValue, &gapBond_remoteOobSCData, sizeof(gapBondOobSC_t));
      break;

    case GAPBOND_LOCAL_OOB_SC_ENABLED:
      *((uint8_t *)pValue) = gapBond_localOobScDataFlag;
      break;

    case GAPBOND_LOCAL_OOB_SC_DATA:
      VOID osal_memcpy(pValue, gapBond_localOobSCData, KEYLEN);
      break;
#endif //(BLE_V42_FEATURES & SECURE_CONNS_CFG)

    case GAPBOND_LRU_BOND_REPLACEMENT:
      *((uint8_t *)pValue) = gapBond_removeLRUBond;
      break;

    default:
      // The param value isn't part of this profile, try the GAP.
      if (param < TGAP_PARAMID_MAX)
      {
        *((uint16_t*)pValue) = GAP_GetParamValue(param);
      }
      else
      {
        ret = INVALIDPARAMETER;
      }
      break;
  }

  return (ret);
}

/*********************************************************************
 * @brief   Notify the Bond Manager that a connection has been made.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_LinkEst(uint8_t addrType, uint8_t *pDevAddr, uint16_t connHandle, uint8_t role)
{
  uint8_t idx;                          // NV Index
  uint8_t publicAddr[B_ADDR_LEN]        // Place to put the public address
      = {0, 0, 0, 0, 0, 0};

  idx = GAPBondMgr_ResolveAddr(addrType, pDevAddr, publicAddr);
  if (idx < GAP_BONDINGS_MAX)
  {
    uint8_t stateFlags = gapBondMgrGetStateFlags(idx);
    gapBondCharCfg_t charCfg[GAP_CHAR_CFG_MAX]; // Space to read a char cfg record from NV
#ifndef SNP_SECURITY
    smSigningInfo_t signingInfo;
#endif //SNP_SECURITY

    // On peripheral, load the key information for the bonding
    // On central and initiaiting security, load key to initiate encyption
    gapBondMgrBondReq(connHandle, idx, stateFlags, role,
                       ((gapBond_PairingMode == GAPBOND_PAIRING_MODE_INITIATE) ? TRUE : FALSE));

#ifndef SNP_SECURITY
    // Load the Signing Key
    VOID osal_memset(&signingInfo, 0, sizeof (smSigningInfo_t));
    if (osal_snv_read(devCSRKNvID(idx), KEYLEN, signingInfo.srk) == SUCCESS)
    {
      if (osal_isbufset(signingInfo.srk, 0xFF, KEYLEN) == FALSE)
      {
        // Load the signing information for this connection
        VOID osal_snv_read(devSignCounterNvID(idx), sizeof (uint32_t), &(signingInfo.signCounter));
        VOID GAP_Signable(connHandle,
                          ((stateFlags & GAP_BONDED_STATE_AUTHENTICATED) ? TRUE : FALSE),
                          &signingInfo);
      }
    }
#endif //SNP_SECURITY

    // Load the characteristic configuration
    if (osal_snv_read(gattCfgNvID(idx), sizeof (charCfg), charCfg) == SUCCESS)
    {
      uint8_t i;

      gapBondMgrInvertCharCfgItem(charCfg);

      for (i = 0; i < GAP_CHAR_CFG_MAX; i++)
      {
        gapBondCharCfg_t *pItem = &(charCfg[i]);

        // Apply the characteristic configuration for this connection
        if (pItem->attrHandle != GATT_INVALID_HANDLE)
        {
          VOID GATTServApp_UpdateCharCfg(connHandle, pItem->attrHandle,
                                          (uint16_t)(pItem->value));
        }
      }
    }

#ifndef GATT_NO_SERVICE_CHANGED
    // Has there been a service change?
    if (stateFlags & GAP_BONDED_STATE_SERVICE_CHANGED)
    {
      VOID GATTServApp_SendServiceChangedInd(connHandle, gapBondMgr_TaskID);
    }
#endif // GATT_NO_SERVICE_CHANGED

    // Update LRU Bond list
    gapBondMgrUpdateLruBondList(idx);

  }
#if (HOST_CONFIG & CENTRAL_CFG)
  else if (role == GAP_PROFILE_CENTRAL &&
            gapBond_PairingMode == GAPBOND_PAIRING_MODE_INITIATE)
  {
    // If Central and initiating and not bonded, then initiate pairing
    VOID gapBondMgrAuthenticate(connHandle, addrType, NULL);

    // Call app state callback
    if (pGapBondCB && pGapBondCB->pairStateCB)
    {
      pGapBondCB->pairStateCB(connHandle, GAPBOND_PAIRING_STATE_STARTED, SUCCESS);
    }
  }
#endif // HOST_CONFIG & CENTRAL_CFG

#if (HOST_CONFIG & PERIPHERAL_CFG)
  // If Peripheral and initiating, send a slave security request to
  // initiate either pairing or encryption
  if (role == GAP_PROFILE_PERIPHERAL &&
       gapBond_PairingMode == GAPBOND_PAIRING_MODE_INITIATE)
  {
    gapBondMgrSlaveSecurityReq(connHandle);
  }
#endif //HOST_CONFIG & PERIPHERAL_CFG

  return (SUCCESS);
}

/*********************************************************************
 * @brief   Notify the Bond Manager that a connection has been terminated.
 *
 * Public function defined in gapbondmgr.h.
 */
void GAPBondMgr_LinkTerm(uint16_t connHandle)
{
  (void)connHandle;

  if (GAP_NumActiveConnections() == 0)
  {
    // See if we're asked to erase all bonding records
    if (eraseAllBonds == TRUE)
    {
      VOID gapBondMgrEraseAllBondings();
      eraseAllBonds = FALSE;

      // Reset bonds to delete table
      osal_memset(bondsToDelete, FALSE, sizeof(bondsToDelete));
    }
    else
    {
      // See if we're asked to erase any single bonding records
      uint8_t idx;
      for (idx = 0; idx < GAP_BONDINGS_MAX; idx++)
      {
        if (bondsToDelete[idx] == TRUE)
        {
          VOID gapBondMgrEraseBonding(idx);
          bondsToDelete[idx] = FALSE;
        }
      }
    }

    // See if NV needs a compaction
    VOID osal_snv_compact(NV_COMPACT_THRESHOLD);

    // Make sure Bond RAM Shadow is up-to-date
    gapBondMgrReadBonds();
  }
}

#if (HOST_CONFIG & CENTRAL_CFG)
/*********************************************************************
 * @brief   Notify the Bond Manager that a Slave Security Request is received.
 *
 * Public function defined in gapbondmgr.h.
 */
void GAPBondMgr_SlaveReqSecurity(uint16_t connHandle, uint8_t authReq)
{
  uint8_t idx;
  uint8_t publicAddr[B_ADDR_LEN] = {0, 0, 0, 0, 0, 0};
  linkDBItem_t *pLink = linkDB_Find(connHandle);

  // If link found and not already initiating security
  if (pLink != NULL && gapBond_PairingMode != GAPBOND_PAIRING_MODE_INITIATE)
  {
    // If already bonded initiate encryption
    idx = GAPBondMgr_ResolveAddr(pLink->addrType, pLink->addr, publicAddr);
    if (idx < GAP_BONDINGS_MAX)
    {
      uint8_t prevStateFlags = gapBondMgrGetStateFlags(idx) ;

      // Convert state flags into authReq format in the spec
      uint8_t prevAuthReq = 0;
      prevAuthReq |= (prevStateFlags & GAP_BONDED_STATE_AUTHENTICATED) ? SM_AUTH_STATE_AUTHENTICATED : 0 ;
      prevAuthReq |= (prevStateFlags & GAP_BONDED_STATE_SECURECONNECTION) ? SM_AUTH_STATE_SECURECONNECTION : 0 ;

      // Core Specification v4.2 Vol 3 Part H Section 2.4.6 states "If
      // [security info] does not meet the security properties requested by the
      // slave, then the master shall initiate the pairing procedure."
      if (((authReq & SM_AUTH_STATE_AUTHENTICATED) <= (prevAuthReq & SM_AUTH_STATE_AUTHENTICATED)) &&
           ((authReq & SM_AUTH_STATE_SECURECONNECTION) <= (prevAuthReq & SM_AUTH_STATE_SECURECONNECTION)))
      {
        gapBondMgrBondReq(connHandle, idx, gapBondMgrGetStateFlags(idx),
                           GAP_PROFILE_CENTRAL, TRUE);

        return;
      }
    }

    // If no pairing allowed or an authentication attempt fails.
    if (gapBond_PairingMode == GAPBOND_PAIRING_MODE_NO_PAIRING ||
         (gapBondMgrAuthenticate(connHandle, pLink->addrType, NULL)
          != SUCCESS))
    {
      // Send error
      VOID GAP_TerminateAuth(connHandle, SMP_PAIRING_FAILED_NOT_SUPPORTED);

      return;
    }
  }
#ifdef GBM_QUEUE_PAIRINGS
  else if (pLink != NULL)
  {
    // If we are queuing pairings and this device automatically initiates
    // pairing, fail now and send a request later (the request was already
    // stored when GAPBondMgr_LinkEst was called.)
    VOID GAP_TerminateAuth(connHandle, SMP_PAIRING_FAILED_NOT_SUPPORTED);
  }
#endif //GBM_QUEUE_PAIRINGS
}
#endif

/*********************************************************************
 * @brief   Resolve an address from bonding information.
 *
 * Public function defined in gapbondmgr.h.
 */
uint8_t GAPBondMgr_ResolveAddr(uint8_t addrType, uint8_t *pDevAddr, uint8_t *pResolvedAddr)
{
  uint8_t idx = GAP_BONDINGS_MAX;
  uint8_t randomSubType = gapBondGetRandomAddrSubType(pDevAddr);

  if (addrType == ADDRTYPE_PUBLIC ||
       addrType == ADDRTYPE_PUBLIC_ID ||
       addrType == ADDRTYPE_RANDOM_ID ||
       randomSubType == RANDOM_ADDR_SUBTYPE_STATIC)
  {
    idx = gapBondMgrFindAddr(pDevAddr);
    if ((idx < GAP_BONDINGS_MAX) && (pResolvedAddr))
    {
      VOID osal_memcpy(pResolvedAddr, pDevAddr, B_ADDR_LEN);
    }
  }
  else if (randomSubType == RANDOM_ADDR_SUBTYPE_NRPA)
  {
    // This could be a reconnection address
    idx = gapBondMgrFindReconnectAddr(pDevAddr);
    if ((idx < GAP_BONDINGS_MAX) && (pResolvedAddr))
    {
      VOID gapBondMgrGetPublicAddr(idx, pResolvedAddr);
    }
  }
  // If using Privacy 1.2 then controller will have all IRK values stored
  // in the bond records and will try to resolve every incoming RPA. Meaning
  // if the host is given an RPA it will not be able to resolve it. Thus,
  // resolving an address at the host level is only possible if Privacy 1.2
  // is not enabled
#if !defined (BLE_V42_FEATURES) || !(BLE_V42_FEATURES & PRIVACY_1_2_CFG)
  else if (randomSubType == RANDOM_ADDR_SUBTYPE_RPA)
  {
    idx = gapBondMgrResolvePrivateAddr(pDevAddr);
    if ((idx < GAP_BONDINGS_MAX) && (pResolvedAddr))
    {
      VOID gapBondMgrGetPublicAddr(idx, pResolvedAddr);
    }
  }
#endif // ! BLE_V42_FEATURES | ! PRIVACY_1_2_CFG

  return (idx);
}

#ifndef GATT_NO_SERVICE_CHANGED
/*********************************************************************
 * @brief   Set/clear the service change indication in a bond record.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_ServiceChangeInd(uint16_t connectionHandle, uint8_t setParam)
{
  bStatus_t ret = bleNoResources; // return value

  if (connectionHandle == 0xFFFF)
  {
    uint8_t idx;  // loop counter

    // Run through the bond database and update the Service Change indication
    for (idx = 0; idx < GAP_BONDINGS_MAX; idx++)
    {
      if (gapBondMgrChangeState(idx, GAP_BONDED_STATE_SERVICE_CHANGED, setParam))
      {
        ret = SUCCESS;
      }
    }

    // If the service change indication is TRUE, tell the connected devices
    if (setParam)
    {
      // Run connected database
      linkDB_PerformFunc(gapBondMgrSendServiceChange);
    }
  }
  else
  {
    // Find connection information
    linkDBItem_t *pLinkItem = linkDB_Find(connectionHandle);
    if (pLinkItem)
    {
      uint8_t idx; // loop counter
      idx = GAPBondMgr_ResolveAddr(pLinkItem->addrType, pLinkItem->addr, NULL);
      if (idx < GAP_BONDINGS_MAX)
      {
        // Bond found, update it.
        VOID gapBondMgrChangeState(idx, GAP_BONDED_STATE_SERVICE_CHANGED, setParam);
        ret = SUCCESS;
      }

      // If the service change indication is TRUE, tell the connected device
      if (setParam)
      {
        gapBondMgrSendServiceChange(pLinkItem);
      }
    }
    else
    {
      ret = bleNotConnected;
    }
  }

  return (ret);
}
#endif // GATT_NO_SERVICE_CHANGED

/*********************************************************************
 * @brief   Update the Characteristic Configuration in a bond record.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_UpdateCharCfg(uint16_t connectionHandle, uint16_t attrHandle, uint16_t value)
{
  bStatus_t ret = bleNoResources; // return value

  if (connectionHandle == INVALID_CONNHANDLE)
  {
    uint8_t idx;  // loop counter

    // Run through the bond database and update the Characteristic Configuration
    for (idx = 0; idx < GAP_BONDINGS_MAX; idx++)
    {
      if (gapBondMgrUpdateCharCfg(idx, attrHandle, value))
      {
        ret = SUCCESS;
      }
    }
  }
  else
  {
    // Find connection information
    linkDBItem_t *pLinkItem = linkDB_Find(connectionHandle);
    if (pLinkItem)
    {
      uint8_t idx = GAPBondMgr_ResolveAddr(pLinkItem->addrType, pLinkItem->addr, NULL);
      if (idx < GAP_BONDINGS_MAX)
      {
        // Bond found, update it.
        VOID gapBondMgrUpdateCharCfg(idx, attrHandle, value);
        ret = SUCCESS;
      }
    }
    else
    {
      ret = bleNotConnected;
    }
  }

  return (ret);
}

/*********************************************************************
 * @brief   Register callback functions with the bond manager.
 *
 * Public function defined in gapbondmgr.h.
 */
void GAPBondMgr_Register(gapBondCBs_t *pCB)
{
  pGapBondCB = pCB;

  // Take over the processing of Authentication messages
  VOID GAP_SetParamValue(TGAP_AUTH_TASK_ID, gapBondMgr_TaskID);

  // Register with GATT Server App for event messages
  GATTServApp_RegisterForMsg(gapBondMgr_TaskID);
}

/*********************************************************************
 * @brief   Respond to a passcode request.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_PasscodeRsp(uint16_t connectionHandle, uint8_t status, uint32_t passcode)
{
  bStatus_t ret = SUCCESS;

  if (status == SUCCESS)
  {
    // Truncate the passcode
    passcode = passcode % (GAP_PASSCODE_MAX + 1);

    ret = GAP_PasscodeUpdate(passcode, connectionHandle);
    if (ret != SUCCESS)
    {
      VOID GAP_TerminateAuth(connectionHandle, SMP_PAIRING_FAILED_PASSKEY_ENTRY_FAILED);
    }
  }
  else
  {
    VOID GAP_TerminateAuth(connectionHandle, status);
  }

  return ret;
}

#ifndef GBM_GATT_NO_CLIENT
#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
/*********************************************************************
 * @brief   Send Read By Type Request to get value of Central Address
 *          Resolution characteristic
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_ReadCentAddrResChar(uint16_t connectionHandle)
{
  attReadByTypeReq_t req;

  req.startHandle = 0x0001;
  req.endHandle   = 0xFFFF;

  req.type.len = ATT_BT_UUID_SIZE;
  req.type.uuid[0] = LO_UINT16(CENTRAL_ADDRESS_RESOLUTION_UUID);
  req.type.uuid[1] = HI_UINT16(CENTRAL_ADDRESS_RESOLUTION_UUID);

  return GATT_ReadUsingCharUUID(connectionHandle, &req,
                                 gapBondMgr_TaskID);
}
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG
#endif //!GBM_GATT_NO_CLIENT

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
/*********************************************************************
 * @brief   Determine if peer device supports enhanced privacy
 *
 * Public function defined in gapbondmgr.h.
 */
uint8_t GAPBondMgr_SupportsEnhancedPriv(uint8_t *pPeerIdAddr)
{
  uint8_t flags;
  uint8_t idx;

  if (pPeerIdAddr)
  {
    idx = gapBondMgrFindAddr(pPeerIdAddr);

    if (idx < GAP_BONDINGS_MAX)
    {
      flags = gapBondMgrGetStateFlags(idx);

      if (flags & GAP_BONDED_STATE_ENHANCED_PRIVACY)
      {
        return TRUE;
      }
    }
  }

  return FALSE;
}
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG

/*********************************************************************
 * @brief   This is a bypass mechanism to allow the bond manager to process
 *              GAP messages.
 *
 * Public function defined in gapbondmgr.h.
 */
uint8_t GAPBondMgr_ProcessGAPMsg(gapEventHdr_t *pMsg)
{
  switch (pMsg->opcode)
  {
    case GAP_PASSKEY_NEEDED_EVENT:
      {
        gapPasskeyNeededEvent_t *pPkt = (gapPasskeyNeededEvent_t *)pMsg;

        // Check that the pairing is still active.
        if (GAP_isPairing())
        {
          if (pGapBondCB && pGapBondCB->passcodeCB)
          {
            // Ask app for a passcode
            pGapBondCB->passcodeCB(pPkt->deviceAddr, pPkt->connectionHandle, pPkt->uiInputs, pPkt->uiOutputs, pPkt->numComparison);
          }
          else //  There is no registered callback. Terminate pairing.
          {
            VOID GAP_TerminateAuth(pPkt->connectionHandle, SMP_PAIRING_FAILED_PASSKEY_ENTRY_FAILED);
          }
        }
      }
      break;

    case GAP_AUTHENTICATION_COMPLETE_EVENT:
      {
        gapAuthCompleteEvent_t *pPkt = (gapAuthCompleteEvent_t *)pMsg;
        uint8_t saveStatus = SUCCESS;

#ifdef GBM_QUEUE_PAIRINGS
        // If any connected devices are waiting to pair, try now.
        if (pPairingQ)
        {
          osal_set_event(gapBondMgr_TaskID, GAP_BOND_POP_PAIR_QUEUE_EVT);
        }
#endif //GBM_QUEUE_PAIRINGS

        // Should we save bonding information (one save at a time)
        if ((pPkt->hdr.status == SUCCESS)             &&
             (pPkt->authState & SM_AUTH_STATE_BONDING) &&
             (pAuthEvt == NULL))
        {
          gapBondRec_t bondRec;

          VOID osal_memset(&bondRec, 0, sizeof (gapBondRec_t)) ;

          {
            linkDBItem_t *pItem;

            pItem = linkDB_Find(pPkt->connectionHandle);

            if (pItem)
            {
              // Do we have a public address in the data?
              if (pPkt->pIdentityInfo)
              {

                smIdentityInfo_t *pInfo = pPkt->pIdentityInfo;

                bondRec.publicAddrType = pInfo->addrType;
                VOID osal_memcpy(bondRec.publicAddr, pInfo->bd_addr, B_ADDR_LEN);

                // Update LinkDB information
                {
                  // Move currently known address to the private address field
                  osal_memcpy(pItem->addrPriv, pItem->addr, B_ADDR_LEN);

                  // Move the known Identity Address into the public address field.
                  osal_memcpy(pItem->addr, pInfo->bd_addr, B_ADDR_LEN);

                  // Move the Identity Address type into the address type.
                  pItem->addrType = pInfo->addrType | SET_ADDRTYPE_ID;
                }
              }
              else
              {
                // Use current information.
                bondRec.publicAddrType = pItem->addrType;
                VOID osal_memcpy(bondRec.publicAddr, pItem->addr, B_ADDR_LEN);
              }

              // The connection stateFlags are read early in GAP_Bond() stage from the SNV using old connection flags
              // but if the current connection (on AUTH complete event) has different flags, the link flags must be changed
              // before using them and before writing the new flags to the SNV
              // Set the authentication flag
              if (pPkt->authState & SM_AUTH_STATE_AUTHENTICATED)
              {
                pItem->stateFlags |= LINK_AUTHENTICATED;
              }
              else
              {
                pItem->stateFlags &= ~LINK_AUTHENTICATED;
              }
#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & SECURE_CONNS_CFG)
              // Set the secure flag
              if (pPkt->authState & SM_AUTH_STATE_SECURECONNECTION)
              {
                pItem->stateFlags |= LINK_SECURE_CONNECTIONS;
              }
              else
              {
                pItem->stateFlags &= ~LINK_SECURE_CONNECTIONS;
              }
#endif //(BLE_V42_FEATURES & SECURE_CONNS_CFG)
           }
            else
            {
              // We don't have an address, so ignore the message.
              break;
            }
          }

          // Save off of the authentication state
          bondRec.stateFlags |= (pPkt->authState & SM_AUTH_STATE_AUTHENTICATED) ? GAP_BONDED_STATE_AUTHENTICATED : 0;

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & SECURE_CONNS_CFG)
          bondRec.stateFlags |= (pPkt->authState & SM_AUTH_STATE_SECURECONNECTION) ? GAP_BONDED_STATE_SECURECONNECTION : 0;
#endif //(BLE_V42_FEATURES & SECURE_CONNS_CFG)

          if (!gapBondMgrAddBond(&bondRec, pPkt))
          {
            // Notify our task to save bonding information in NV
            osal_set_event(gapBondMgr_TaskID, GAP_BOND_SAVE_REC_EVT);

            // We're not done with this message; it will be freed later
            return (FALSE);
          }

          // Couldn't save bonding information in NV -- it must be full!
          saveStatus = bleNoResources;
        }

        // Call app state callback in the fail case. Success is handled after GAP_BOND_SAVE_REC_EVT.
        if (pGapBondCB && pGapBondCB->pairStateCB)
        {
          pGapBondCB->pairStateCB(pPkt->connectionHandle, GAPBOND_PAIRING_STATE_COMPLETE, pPkt->hdr.status);

          if (saveStatus != SUCCESS)
          {
            // Bonding record couldn't be saved in NV
            pGapBondCB->pairStateCB(pPkt->connectionHandle, GAPBOND_PAIRING_STATE_BOND_SAVED, saveStatus);
          }
        }
      }
      break;

    case GAP_BOND_COMPLETE_EVENT:
      // This message is received when the bonding is complete.  If hdr.status is SUCCESS
      // then call app state callback.  If hdr.status is NOT SUCCESS, the connection will be
      // dropped at the LL because of a MIC failure, so again nothing to do.
      {
        gapBondCompleteEvent_t *pPkt = (gapBondCompleteEvent_t *)pMsg;

#if (HOST_CONFIG & CENTRAL_CFG)
        if (pPkt->hdr.status == LL_ENC_KEY_REQ_REJECTED)
        {
          // LTK not found on peripheral device (Pin or Key Missing). See which
          // option was configured for unsuccessful bonding.
          linkDBItem_t *pLinkItem = linkDB_Find(pPkt->connectionHandle);
          if (pLinkItem)
          {
            switch (gapBond_BondFailOption)
            {
              case GAPBOND_FAIL_INITIATE_PAIRING:
                // Initiate pairing
                gapBondMgrAuthenticate(pPkt->connectionHandle, pLinkItem->addrType, NULL);
                break;

              case GAPBOND_FAIL_TERMINATE_LINK:
                // Drop connection
                GAP_TerminateLinkReq(pLinkItem->taskID, pPkt->connectionHandle, HCI_DISCONNECT_AUTH_FAILURE);
                break;

              case GAPBOND_FAIL_TERMINATE_ERASE_BONDS:
                // Set up bond manager to erase all existing bonds after connection terminates
                VOID GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, 0, NULL);

                // Drop connection
                GAP_TerminateLinkReq(pLinkItem->taskID, pPkt->connectionHandle, HCI_DISCONNECT_AUTH_FAILURE);
                break;

              case GAPBOND_FAIL_NO_ACTION:
                // fall through
              default:
                // do nothing
                break;
            }
          }
        }
#endif
        if (pGapBondCB && pGapBondCB->pairStateCB)
        {
          pGapBondCB->pairStateCB(pPkt->connectionHandle, GAPBOND_PAIRING_STATE_BONDED, pMsg->hdr.status);
        }
      }
      break;

#ifndef SNP_SECURITY
    case GAP_SIGNATURE_UPDATED_EVENT:
      {
        uint8_t idx;
        gapSignUpdateEvent_t *pPkt = (gapSignUpdateEvent_t *)pMsg;

        idx = GAPBondMgr_ResolveAddr(pPkt->addrType, pPkt->devAddr, NULL);
        if (idx < GAP_BONDINGS_MAX)
        {
          // Save the sign counter
          VOID osal_snv_write(devSignCounterNvID(idx), sizeof (uint32_t), &(pPkt->signCounter));
        }
      }
      break;
#endif //SNP_SECURITY

#if (HOST_CONFIG & PERIPHERAL_CFG)
    case GAP_PAIRING_REQ_EVENT:
      {
        gapPairingReqEvent_t *pPkt = (gapPairingReqEvent_t *)pMsg;

        if (gapBond_AutoFail != FALSE)
        {
          // Auto Fail TEST MODE (DON'T USE THIS) - Sends pre-setup reason
          VOID GAP_TerminateAuth(pPkt->connectionHandle, gapBond_AutoFailReason);
        }
        else if (gapBond_PairingMode == GAPBOND_PAIRING_MODE_NO_PAIRING)
        {
          // No Pairing - Send error
          VOID GAP_TerminateAuth(pPkt->connectionHandle, SMP_PAIRING_FAILED_NOT_SUPPORTED);
        }
        else
        {
          linkDBItem_t *pLinkItem = linkDB_Find(pPkt->connectionHandle);
          if (pLinkItem == NULL)
          {
            // Can't find the connection, ignore the message
            break;
          }

          // Requesting bonding?
          if (pPkt->pairReq.authReq & SM_AUTH_STATE_BONDING)
          {
            if ((pLinkItem->addrType != ADDRTYPE_PUBLIC) && (pPkt->pairReq.keyDist.mIdKey == FALSE))
            {
              uint8_t publicAddr[B_ADDR_LEN];

              // Check if we already have the public address in NV
              if (GAPBondMgr_ResolveAddr(pLinkItem->addrType, pLinkItem->addr, publicAddr) == GAP_BONDINGS_MAX)
              {
                // Can't bond to a non-public address if we don't know the public address
                VOID GAP_TerminateAuth(pPkt->connectionHandle, SMP_PAIRING_FAILED_AUTH_REQ);

                // Ignore the message
                break;
              }
            }
          }

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & SECURE_CONNS_CFG)
          // Section 2.3.5.1: "If the key generation method does not result in a key
          // that provides sufficient security properties then the device shall send the
          // Pairing Failed command with the error code "Authentication Requirements.""
          // This supports Secure Connections Only Mode, where if the remote device does
          // not support Secure Connections and the local device requires Secure
          // Connections, then pairing can be aborted here with a Pairing Failed command
          // with error code "Authentication requirements".
          if (gapBond_secureConnection == GAPBOND_SECURE_CONNECTION_ONLY)
          {
            if (!(pPkt->pairReq.authReq & SM_AUTH_STATE_SECURECONNECTION))
            {
              // We are not interested in pairing with this device.
              VOID GAP_TerminateAuth(pPkt->connectionHandle, SMP_PAIRING_FAILED_AUTH_REQ);

              break;
            }
          }
#endif //(BLE_V42_FEATURES & SECURE_CONNS_CFG)

          // Send pairing response
          gapBondMgrAuthenticate(pPkt->connectionHandle, pLinkItem->addrType, &(pPkt->pairReq));

          // Call app state callback
          if (pGapBondCB && pGapBondCB->pairStateCB)
          {
            pGapBondCB->pairStateCB(pPkt->connectionHandle, GAPBOND_PAIRING_STATE_STARTED, SUCCESS);
          }
        }
      }
      break;
#endif

#if (HOST_CONFIG & CENTRAL_CFG)
    case GAP_SLAVE_REQUESTED_SECURITY_EVENT:
      {
        uint16_t connHandle = ((gapSlaveSecurityReqEvent_t *)pMsg)->connectionHandle;
        uint8_t authReq = ((gapSlaveSecurityReqEvent_t *)pMsg)->authReq;

        GAPBondMgr_SlaveReqSecurity(connHandle, authReq);
      }
      break;
#endif

    case GAP_LINK_TERMINATED_EVENT:
      {
        gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

        GAPBondMgr_LinkTerm(pPkt->connectionHandle);
      }
      break;

#ifdef GBM_QUEUE_PAIRINGS
    case GAP_AUTHENTICATION_FAILURE_EVT:
      {
        // If there are any queued devices waiting, try pairing with them
        if (pPairingQ)
        {
          osal_set_event(gapBondMgr_TaskID, GAP_BOND_POP_PAIR_QUEUE_EVT);
        }
      }
      break;
#endif //GBM_QUEUE_PAIRINGS

    default:
      break;
  }

  return (TRUE);
}

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

#if !defined (GATT_NO_SERVICE_CHANGED) || \
    (!defined GBM_GATT_NO_CLIENT && \
      ((defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)) && \
        (HOST_CONFIG & PERIPHERAL_CFG)))
/*********************************************************************
 * @fn      gapBondMgrChangeState
 *
 * @brief   Change a state flag in the stateFlags field of the bond record.
 *
 * @param   idx - Bond NV index
 * @param   state - state flage to set or clear
 * @param   set - TRUE to set the flag, FALSE to clear the flag
 *
 * @return  TRUE if NV Record exists, FALSE if NV Record is empty
 */
static uint8_t gapBondMgrChangeState(uint8_t idx, uint16_t state, uint8_t set)
{
  gapBondRec_t bondRec;   // Space to read a Bond record from NV

  // Look for public address that is used (not all 0xFF's)
  if ((osal_snv_read(mainRecordNvID(idx), sizeof (gapBondRec_t), &bondRec) == SUCCESS)
      && (osal_isbufset(bondRec.publicAddr, 0xFF, B_ADDR_LEN) == FALSE))
  {
    // Update the state of the bonded device.
    uint8_t stateFlags = bondRec.stateFlags;
    if (set)
    {
      stateFlags |= state;
    }
    else
    {
      stateFlags &= ~(state);
    }

    if (stateFlags != bondRec.stateFlags)
    {
      bondRec.stateFlags = stateFlags;
      VOID osal_snv_write(mainRecordNvID(idx), sizeof (gapBondRec_t), &bondRec);
    }

    return (TRUE);
  }

  return (FALSE);
}
#endif // !GATT_NO_SERVICE_CHANGED || \
       // (!GBM_GATT_NO_CLIENT && (BLE_V42_FEATURES & PRIVACY_1_2_CFG & PERIPHERAL_CFG))

/*********************************************************************
 * @fn      gapBondMgrUpdateCharCfg
 *
 * @brief   Update the Characteristic Configuration of the bond record.
 *
 * @param   idx - Bond NV index
 * @param   attrHandle - attribute handle (0 means all handles)
 * @param   value - characteristic configuration value
 *
 * @return  TRUE if NV Record exists, FALSE if NV Record is empty
 */
static uint8_t gapBondMgrUpdateCharCfg(uint8_t idx, uint16_t attrHandle, uint16_t value)
{
  gapBondRec_t bondRec;   // Space to read a Bond record from NV

  // Look for public address that is used (not all 0xFF's)
  if ((osal_snv_read(mainRecordNvID(idx), sizeof (gapBondRec_t), &bondRec) == SUCCESS)
       && (osal_isbufset(bondRec.publicAddr, 0xFF, B_ADDR_LEN) == FALSE))
  {
    gapBondCharCfg_t charCfg[GAP_CHAR_CFG_MAX]; // Space to read a char cfg record from NV

    if (osal_snv_read(gattCfgNvID(idx), sizeof (charCfg), charCfg) == SUCCESS)
    {
      uint8_t update = FALSE;

      gapBondMgrInvertCharCfgItem(charCfg);

      if (attrHandle == GATT_INVALID_HANDLE)
      {
        if (osal_isbufset((uint8_t *)charCfg, 0x00, sizeof (charCfg)) == FALSE)
        {
          // Clear all characteristic configuration for this device
          VOID osal_memset((void *)charCfg, 0x00, sizeof (charCfg));
          update = TRUE;
        }
      }
      else
      {
        gapBondCharCfg_t *pItem = gapBondMgrFindCharCfgItem(attrHandle, charCfg);
        if (pItem == NULL)
        {
          // Must be a new item; ignore if the value is no operation (default)
          if ((value == GATT_CFG_NO_OPERATION) ||
               ((pItem = gapBondMgrFindCharCfgItem(GATT_INVALID_HANDLE, charCfg)) == NULL))
          {
            return (FALSE); // No empty entry found
          }

          pItem->attrHandle = attrHandle;
        }

        if (pItem->value != value)
        {
          // Update characteristic configuration
          pItem->value = (uint8_t)value;
          if (value == GATT_CFG_NO_OPERATION)
          {
            // Erase the item
            pItem->attrHandle = GATT_INVALID_HANDLE;
          }

          update = TRUE;
        }
      }

      // Update the characteristic configuration of the bonded device.
      if (update)
      {
        gapBondMgrInvertCharCfgItem(charCfg);
        VOID osal_snv_write(gattCfgNvID(idx), sizeof(charCfg), charCfg);
      }
    }

    return (TRUE);
  }

  return (FALSE);
}

/*********************************************************************
 * @fn      gapBondMgrFindCharCfgItem
 *
 * @brief   Find the Characteristic Configuration for a given attribute.
 *          Uses the attribute handle to search the characteristic
 *          configuration table of a bonded device.
 *
 * @param   attrHandle - attribute handle.
 * @param   charCfgTbl - characteristic configuration table.
 *
 * @return  pointer to the found item. NULL, otherwise.
 */
static gapBondCharCfg_t *gapBondMgrFindCharCfgItem(uint16_t attrHandle,
                                                    gapBondCharCfg_t *charCfgTbl)
{
  uint8_t i;
  for (i = 0; i < GAP_CHAR_CFG_MAX; i++)
  {
    if (charCfgTbl[i].attrHandle == attrHandle)
    {
      return (&(charCfgTbl[i]));
    }
  }

  return ((gapBondCharCfg_t *)NULL);
}

/*********************************************************************
 * @fn      gapBondMgrInvertCharCfgItem
 *
 * @brief   Invert the Characteristic Configuration for a given client.
 *
 * @param   charCfgTbl - characteristic configuration table.
 *
 * @return  none.
 */
static void gapBondMgrInvertCharCfgItem(gapBondCharCfg_t *charCfgTbl)
{
  uint8_t i;
  for (i = 0; i < GAP_CHAR_CFG_MAX; i++)
  {
    charCfgTbl[i].attrHandle = ~(charCfgTbl[i].attrHandle);
    charCfgTbl[i].value = ~(charCfgTbl[i].value);
  }
}

/*********************************************************************
 * @fn      gapBondMgrAddBond
 *
 * @brief   Save a bond from a GAP Auth Complete Event
 *
 * @param   pBondRec - basic bond record
 * @param   pLocalLTK - LTK used by this device during pairing
 * @param   pDevLTK - LTK used by the connected device during pairing
 * @param   pIRK - IRK used by the connected device during pairing
 * @param   pSRK - SRK used by the connected device during pairing
 * @param   signCounter - Sign counter used by the connected device during pairing
 *
 * @return  TRUE, if done processing bond record. FALSE, otherwise.
 */
static uint8_t gapBondMgrAddBond(gapBondRec_t *pBondRec, gapAuthCompleteEvent_t *pPkt)
{
  // See if this is a new bond record
  if (pAuthEvt == NULL)
  {
    // Make sure we have bonding info
    if ((pBondRec == NULL) || (pPkt == NULL))
    {
      return (TRUE);
    }

    // First see if we already have an existing bond for this device
    bondIdx = gapBondMgrFindAddr(pBondRec->publicAddr);
    if (bondIdx >= GAP_BONDINGS_MAX)
    {
      bondIdx = gapBondMgrFindEmpty();
    }
#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
    else if (pPkt->pIdentityInfo)
    {
      uint8_t oldIrk[KEYLEN];

      // Verify that that a previous bond had an IRK before attempting to
      // remove it from the Controller's resolving list.
      if ((osal_snv_read(devIRKNvID(bondIdx), KEYLEN, oldIrk) == SUCCESS) &&
           (osal_isbufset(oldIrk, 0xFF, KEYLEN) == FALSE))
      {
        // If a current record is simply being updated then erase previous
        // entry in resolving list for this peer. Will subsequently update
        // RL with new ID info for peer
        if (gapState == GAP_STATE_IDLE)
        {
          HCI_LE_RemoveDeviceFromResolvingListCmd((pBondRec->publicAddrType &
                                                     MASK_ADDRTYPE_ID),
                                                   pBondRec->publicAddr);
        }
        else
        {
          // Controller is currently busy so modifying RL is not allowed. Use
          // delayed sync of Bond Records to Resolving List
          gapBond_syncRL = TRUE;
        }
      }
    }
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG
  }

  if (bondIdx < GAP_BONDINGS_MAX)
  {
    // See if this is a new bond record
    if (pAuthEvt == NULL)
    {
      gapBondCharCfg_t charCfg[GAP_CHAR_CFG_MAX];

      // Save the main information
      VOID osal_snv_write(mainRecordNvID(bondIdx), sizeof (gapBondRec_t), pBondRec);

      // Once main information is saved, it qualifies to update the LRU table.
      gapBondMgrUpdateLruBondList(bondIdx);

      // Write out FF's over the characteristic configuration entry, to overwrite
      // any previous bond data that may have been stored
      VOID osal_memset(charCfg, 0xFF, sizeof (charCfg));

      VOID osal_snv_write(gattCfgNvID(bondIdx), sizeof (charCfg), charCfg);

      // Update Bond RAM Shadow just with the newly added bond entry
      VOID osal_memcpy(&(bonds[bondIdx]), pBondRec, sizeof (gapBondRec_t));

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
      if (pPkt->pIdentityInfo)
      {
        // Add device to resolving list
        if (gapState == GAP_STATE_IDLE)
        {
          HCI_LE_AddDeviceToResolvingListCmd((pPkt->pIdentityInfo->addrType &
                                                MASK_ADDRTYPE_ID),
                                              pPkt->pIdentityInfo->bd_addr,
                                              pPkt->pIdentityInfo->irk,
                                              NULL);
        }
        else
        {
          // Controller must be in adv/scan/init state, so set syncRL flag
          // and wait for idle period to sync the bond records and RL
          gapBond_syncRL = TRUE;
        }
      }
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG

      // Keep the OSAL message to store the security keys later - will be freed then
      pAuthEvt = pPkt;
    }
    else
    {
      // If available, save the LTK information
      if (pAuthEvt->pSecurityInfo)
      {
        VOID osal_snv_write(localLTKNvID(bondIdx), sizeof (gapBondLTK_t), pAuthEvt->pSecurityInfo);
        pAuthEvt->pSecurityInfo = NULL;
      }
      // If available, save the connected device's LTK information
      else if (pAuthEvt->pDevSecInfo)
      {
#ifndef SNP_SECURITY
        VOID osal_snv_write(devLTKNvID(bondIdx), sizeof (gapBondLTK_t), pAuthEvt->pDevSecInfo);
#endif //SNP_SECURITY
        pAuthEvt->pDevSecInfo = NULL;
      }
      // If available, save the connected device's IRK
      else if (pAuthEvt->pIdentityInfo)
      {
        VOID osal_snv_write(devIRKNvID(bondIdx), KEYLEN, pAuthEvt->pIdentityInfo->irk);
        pAuthEvt->pIdentityInfo = NULL;
      }
      // If available, save the connected device's Signature information
      else if (pAuthEvt->pSigningInfo)
      {
#ifndef SNP_SECURITY
        VOID osal_snv_write(devCSRKNvID(bondIdx), KEYLEN, pAuthEvt->pSigningInfo->srk);
        VOID osal_snv_write(devSignCounterNvID(bondIdx), sizeof (uint32_t), &(pAuthEvt->pSigningInfo->signCounter));
#endif //SNP_SECURITY
        pAuthEvt->pSigningInfo = NULL;
      }
      else
      {
        if (autoSyncWhiteList)
        {
          gapBondMgr_SyncWhiteList();
        }

        // Update the GAP Privacy Flag Properties
        gapBondSetupPrivFlag();

        return (TRUE);
      }
    }

    // We have more info to store
    return (FALSE);
  }

  return (TRUE);
}

/*********************************************************************
 * @fn      gapBondMgrGetStateFlags
 *
 * @brief   Gets the state flags field of a bond record in NV
 *
 * @param   idx
 *
 * @return  stateFlags field
 */
static uint8_t gapBondMgrGetStateFlags(uint8_t idx)
{
  gapBondRec_t bondRec;

  if (osal_snv_read(mainRecordNvID(idx), sizeof (gapBondRec_t), &bondRec) == SUCCESS)
  {
    return (bondRec.stateFlags);
  }

  return (0);
}

/*********************************************************************
 * @fn      gapBondMgrGetPublicAddr
 *
 * @brief   Copy the public Address from a bonding record
 *
 * @param   idx - Bond record index
 * @param   pAddr - a place to put the public address from NV
 *
 * @return  SUCCESS if successful.
 *          Otherwise failure.
 */
static bStatus_t gapBondMgrGetPublicAddr(uint8_t idx, uint8_t *pAddr)
{
  bStatus_t stat;         // return value
  gapBondRec_t bondRec;   // Work space for main bond record

  // Check parameters
  if ((idx >= GAP_BONDINGS_MAX) || (pAddr == NULL))
  {
    return (INVALIDPARAMETER);
  }

  stat = osal_snv_read(mainRecordNvID(idx), sizeof (gapBondRec_t), &bondRec);

  if (stat == SUCCESS)
  {
    VOID osal_memcpy(pAddr, bondRec.publicAddr, B_ADDR_LEN);
  }

  return (stat);
}

/*********************************************************************
 * @fn      gapBondMgrFindReconnectAddr
 *
 * @brief   Look through the bonding entries to find a
 *          reconnection address.
 *
 * @param   pReconnectAddr - device address to look for
 *
 * @return  index to found bonding (0 - (GAP_BONDINGS_MAX-1),
 *          GAP_BONDINGS_MAX if no empty entries
 */
static uint8_t gapBondMgrFindReconnectAddr(uint8_t *pReconnectAddr)
{
  // Item doesn't exist, so create all the items
  uint8_t idx;
  for (idx = 0; idx < GAP_BONDINGS_MAX; idx++)
  {
    // compare reconnection address
    if (osal_memcmp(bonds[idx].reconnectAddr, pReconnectAddr, B_ADDR_LEN))
    {
      return (idx); // Found it
    }
  }

  return (GAP_BONDINGS_MAX);
}

/*********************************************************************
 * @fn      gapBondMgrFindAddr
 *
 * @brief   Look through the bonding entries to find an address.
 *
 * @param   pDevAddr - device address to look for
 *
 * @return  index to empty bonding (0 - (GAP_BONDINGS_MAX-1),
 *          GAP_BONDINGS_MAX if no empty entries
 */
static uint8_t gapBondMgrFindAddr(uint8_t *pDevAddr)
{
  // Item doesn't exist, so create all the items
  uint8_t idx;
  for (idx = 0; idx < GAP_BONDINGS_MAX; idx++)
  {
    // Read in NV Main Bond Record and compare public address
    if (osal_memcmp(bonds[idx].publicAddr, pDevAddr, B_ADDR_LEN))
    {
      return (idx); // Found it
    }
  }

  return (GAP_BONDINGS_MAX);
}

#if !defined (BLE_V42_FEATURES) || !(BLE_V42_FEATURES & PRIVACY_1_2_CFG)
/*********************************************************************
 * @fn      gapBondMgrResolvePrivateAddr
 *
 * @brief   Look through the NV bonding entries to resolve a private
 *          address.
 *
 * @param   pDevAddr - device address to look for
 *
 * @return  index to found bonding (0 - (GAP_BONDINGS_MAX-1),
 *          GAP_BONDINGS_MAX if no entry found
 */
static uint8_t gapBondMgrResolvePrivateAddr(uint8_t *pDevAddr)
{
  uint8_t idx;
  for (idx = 0; idx < GAP_BONDINGS_MAX; idx++)
  {
    uint8_t IRK[KEYLEN];

    // Read in NV IRK Record and compare resolvable address
    if (osal_snv_read(devIRKNvID(idx), KEYLEN, IRK) == SUCCESS)
    {
      if ((osal_isbufset(IRK, 0xFF, KEYLEN) == FALSE) &&
           (GAP_ResolvePrivateAddr(IRK, pDevAddr) == SUCCESS))
      {
        return (idx); // Found it
      }
    }
  }

  return (GAP_BONDINGS_MAX);
}
#endif // ! BLE_V42_FEATURES | ! PRIVACY_1_2_CFG

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
/*********************************************************************
 * @fn      gapBondMgr_gapIdle
 *
 * @brief   Callback for GAP to notify bond manager whenever it has
 *          finished all adv/scan/init
 *
 * @param   state - GAP state @ref GAP_STATE_FLAGS
 *
 * @return  none
 */
void gapBondMgr_gapIdle(void)
{
  // Callback will be received when GAP is no longer advertising, scanning,
  // or initiating a connection. These are states where the resolving list
  // cannot be updated so once GAP is no longer in these states Bond Mgr
  // should sync resolving list to bond records if there is a pending update
  if (gapBond_syncRL == TRUE)
  {
    GAPBondMgr_syncResolvingList();
  }
}
#endif // BLE_V42_FEATURES & PRIVACY_1_2_CFG

/*********************************************************************
 * @fn      gapBondMgrReadBonds
 *
 * @brief   Read through NV and store them in RAM.
 *
 * @param   none
 *
 * @return  none
 */
static void gapBondMgrReadBonds(void)
{
  uint8_t idx;
  for (idx = 0; idx < GAP_BONDINGS_MAX; idx++)
  {
    // See if the entry exists in NV
    if (osal_snv_read(mainRecordNvID(idx), sizeof(gapBondRec_t), &(bonds[idx])) != SUCCESS)
    {
      // Can't read the entry, assume that it doesn't exist
      VOID osal_memset(bonds[idx].publicAddr, 0xFF, B_ADDR_LEN);
      VOID osal_memset(bonds[idx].reconnectAddr, 0xFF, B_ADDR_LEN);
      bonds[idx].stateFlags = 0;
    }
  }

  if (autoSyncWhiteList)
  {
    gapBondMgr_SyncWhiteList();
  }

  // Update the GAP Privacy Flag Properties
  gapBondSetupPrivFlag();
}

/*********************************************************************
 * @fn      gapBondMgrReadLruBondList
 *
 * @brief   Find List of LRU Bond in NV or create one.
 *
 * @param   none
 *
 * @return  none
 */
static void gapBondMgrReadLruBondList(void)
{
  // See if the LRU list exists in NV
  if (osal_snv_read(BLE_LRU_BOND_LIST, sizeof(uint8_t) * GAP_BONDINGS_MAX,
                      gapBond_lruBondList) != SUCCESS)
  {
    // If it doesn't, add the list using the LRU values
    for (uint8_t idx = 0; idx < GAP_BONDINGS_MAX; idx++)
    {
      // Store in reverse order (bond record 0 will automatically be most recently used)
      gapBond_lruBondList[idx] = GAP_BONDINGS_MAX - idx - 1;
    }
  }
}

/*********************************************************************
 * @fn      gapBondMgrGetLruBondIndex
 *
 * @brief   Get the least recently used bond then set it as the most recently
 *          used bond.  This algorithm expects that this bond will immediately
 *          be used to reference the most recently used bond.
 *
 * @param   none
 *
 * @return  the least recently used bond.
 */
static uint8_t gapBondMgrGetLruBondIndex(void)
{
  // Return bond record index found in the LRU position.
  return gapBond_lruBondList[0];
}

/*********************************************************************
 * @fn      gapBondMgrUpdateLruBondList
 *
 * @brief   Make bond record index the most recently used bond, updating the
 *          list accordingly.
 *
 * @param   the bond record index.
 *
 * @return  none
 */
static void gapBondMgrUpdateLruBondList(uint8_t bondIndex)
{
  uint8_t updateIdx = GAP_BONDINGS_MAX;

  // Find the index
  for (uint8_t i = GAP_BONDINGS_MAX; i > 0; i--)
  {
    if (gapBond_lruBondList[i - 1] == bondIndex)
    {
      // Found.
      updateIdx = i - 1;
      break;
    }
  }

  // Update list to make bondIndex the most recently used.
  for (uint8_t i = updateIdx ; i < GAP_BONDINGS_MAX - 1; i++)
  {
    uint8_t swap = gapBond_lruBondList[i];
    gapBond_lruBondList[i] = gapBond_lruBondList[i + 1];
    gapBond_lruBondList[i + 1] = swap;
  }

  // If there was any change to the list
  if (updateIdx < GAP_BONDINGS_MAX - 1)
  {
    // Store updated list in NV
    osal_snv_write(BLE_LRU_BOND_LIST, sizeof(uint8_t) * GAP_BONDINGS_MAX,
                    gapBond_lruBondList);
  }
}

/*********************************************************************
 * @fn      gapBondMgrFindEmpty
 *
 * @brief   Look through the bonding NV entries to find an empty.
 *
 * @param   none
 *
 * @return  index to empty bonding (0 - (GAP_BONDINGS_MAX-1),
 *          GAP_BONDINGS_MAX if no empty entries
 */
static uint8_t gapBondMgrFindEmpty(void)
{
  // Item doesn't exist, so create all the items
  uint8_t idx;
  for (idx = 0; idx < GAP_BONDINGS_MAX; idx++)
  {
    // Look for public address of all 0xFF's
    if (osal_isbufset(bonds[idx].publicAddr, 0xFF, B_ADDR_LEN))
    {
      return (idx); // Found one
    }
  }

  // If all bonding records are used and LRU bond removal is permitted,
  // use the LRU bond.
  if (gapBond_removeLRUBond)
  {
    return  gapBondMgrGetLruBondIndex();
  }

  return (GAP_BONDINGS_MAX);
}

/*********************************************************************
 * @fn      gapBondMgrBondTotal
 *
 * @brief   Look through the bonding NV entries calculate the number
 *          entries.
 *
 * @param   none
 *
 * @return  total number of bonds found
 */
static uint8_t gapBondMgrBondTotal(void)
{
  uint8_t idx;
  uint8_t numBonds = 0;

  // Item doesn't exist, so create all the items
  for (idx = 0; idx < GAP_BONDINGS_MAX; idx++)
  {
    // Look for public address that are not 0xFF's
    if (osal_isbufset(bonds[idx].publicAddr, 0xFF, B_ADDR_LEN) == FALSE)
    {
      numBonds++; // Found one
    }
  }

  return (numBonds);
}

/*********************************************************************
 * @fn      gapBondMgrEraseAllBondings
 *
 * @brief   Write all 0xFF's to all of the bonding entries
 *
 * @param   none
 *
 * @return  SUCCESS if successful.
 *          Otherwise, NV_OPER_FAILED for failure.
 */
static bStatus_t gapBondMgrEraseAllBondings(void)
{
  uint8_t idx;
  bStatus_t stat = SUCCESS;  // return value

  // Item doesn't exist, so create all the items
  for (idx = 0; (idx < GAP_BONDINGS_MAX) && (stat == SUCCESS); idx++)
  {
    // Erasing will write/create a bonding entry
    stat = gapBondMgrEraseBonding(idx);
  }

  return (stat);
}

/*********************************************************************
 * @fn      gapBondMgrEraseBonding
 *
 * @brief   Write all 0xFF's to the complete bonding record
 *
 * @param   idx - bonding index
 *
 * @return  SUCCESS if successful.
 *          Otherwise, NV_OPER_FAILED for failure.
 */
static bStatus_t gapBondMgrEraseBonding(uint8_t idx)
{
  bStatus_t ret;
  gapBondRec_t bondRec;

  if (idx == bondIdx)
  {
    // Stop ongoing bond store process to prevent any invalid data be written.
    osal_clear_event(gapBondMgr_TaskID, GAP_BOND_SYNC_CC_EVT);
    osal_clear_event(gapBondMgr_TaskID, GAP_BOND_SAVE_REC_EVT);

    gapBondFreeAuthEvt();
  }

#if (HOST_CONFIG & PERIPHERAL_CFG)

#if defined (GAP_PRIVACY_RECONNECT)
  if (idx == reconnectAddrIdx)
  {
    osal_clear_event(gapBondMgr_TaskID, GAP_BOND_SAVE_RCA_EVT);

    reconnectAddrIdx = GAP_BONDINGS_MAX;
  }
#endif // GAP_PRIVACY_RECONNECT

#endif // PERIPHERAL_CFG

  // First see if bonding record exists in NV, then write all 0xFF's to it
  if ((osal_snv_read(mainRecordNvID(idx), sizeof (gapBondRec_t), &bondRec) == SUCCESS)
       && (osal_isbufset(bondRec.publicAddr, 0xFF, B_ADDR_LEN) == FALSE))
  {
    gapBondLTK_t ltk;
    gapBondCharCfg_t charCfg[GAP_CHAR_CFG_MAX];

    // Initialize memory of temporary variables
    VOID osal_memset(&ltk, 0xFF, sizeof (gapBondLTK_t));
    VOID osal_memset(charCfg, 0xFF, sizeof (charCfg));

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
    {
      uint8_t oldIrk[KEYLEN];

      // Verify that that a previous bond had an IRK before attempting to
      // remove it from the Controller's resolving list.
      if ((osal_snv_read(devIRKNvID(idx), KEYLEN, oldIrk) == SUCCESS) &&
           (osal_isbufset(oldIrk, 0xFF, KEYLEN) == FALSE))
      {
        // Remove device from Resolving List prior to erasing bonding info
        if (gapState == GAP_STATE_IDLE)
        {
          HCI_LE_RemoveDeviceFromResolvingListCmd((bondRec.publicAddrType &
                                                     MASK_ADDRTYPE_ID),
                                                   bondRec.publicAddr);
        }
        else
        {
          // Controller is currently busy so modifying RL is not allowed. Use
          // delayed sync of Bond Records to Resolving List
          gapBond_syncRL = TRUE;
        }
      }
    }
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG

    // Erase bond record shadow copy
    VOID osal_memset(&bondRec, 0xFF, sizeof (gapBondRec_t));

    // Write out FF's over the entire bond entry.
    ret = osal_snv_write(mainRecordNvID(idx), sizeof (gapBondRec_t), &bondRec);
    ret |= osal_snv_write(localLTKNvID(idx), sizeof (gapBondLTK_t), &ltk);
#ifndef SNP_SECURITY
    ret |= osal_snv_write(devLTKNvID(idx), sizeof (gapBondLTK_t), &ltk);
#endif //SNP_SECURITY
    ret |= osal_snv_write(devIRKNvID(idx), KEYLEN, ltk.LTK);
#ifndef SNP_SECURITY
    ret |= osal_snv_write(devCSRKNvID(idx), KEYLEN, ltk.LTK);
    ret |= osal_snv_write(devSignCounterNvID(idx), sizeof (uint32_t), ltk.LTK);
#endif //SNP_SECURITY

    // Write out FF's over the characteristic configuration entry.
    ret |= osal_snv_write(gattCfgNvID(idx), sizeof (charCfg), charCfg);
  }
  else
  {
    ret = SUCCESS;
  }

  return (ret);
}

/*********************************************************************
 * @brief   Task Initialization function.
 *
 * Internal function defined in gapbondmgr.h.
 */
void GAPBondMgr_Init(uint8_t task_id)
{
  gapBondMgr_TaskID = task_id;  // Save task ID

  // Register Call Back functions for GAP
  GAP_RegisterBondMgrCBs(&gapCBs);

  // Setup Bond RAM Shadow
  gapBondMgrReadBonds();

  // Setup LRU Bond List
  gapBondMgrReadLruBondList();

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
  osal_set_event(gapBondMgr_TaskID, GAP_BOND_SYNC_RL_EVT);
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG

#if (HOST_CONFIG & PERIPHERAL_CFG)
#if defined (GAP_PRIVACY_RECONNECT)
  GGS_RegisterAppCBs(&gapBondMgrCB);
#endif // GAP_PRIVACY_RECONNECT
#endif // PERIPHERAL_CFG
}

/*********************************************************************
 * @brief   Task Event Processor function.
 *
 * Internal function defined in gapbondmgr.h.
 */
uint16_t GAPBondMgr_ProcessEvent(uint8_t task_id, uint16_t events)
{
  VOID task_id; // OSAL required parameter that isn't used in this function

  if (events & SYS_EVENT_MSG)
  {
    uint8_t *pMsg;

    if ((pMsg = osal_msg_receive(gapBondMgr_TaskID)) != NULL)
    {
      if (gapBondMgr_ProcessOSALMsg((osal_event_hdr_t *)pMsg))
      {
        // Release the OSAL message
        VOID osal_msg_deallocate(pMsg);
      }
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if (events & GAP_BOND_SAVE_REC_EVT)
  {
    // Save bonding record in NV
    if (gapBondMgrAddBond(NULL, NULL))
    {
      // Notify our task to update NV with CCC values stored in GATT database
      osal_set_event(gapBondMgr_TaskID, GAP_BOND_SYNC_CC_EVT);

      return (events ^ GAP_BOND_SAVE_REC_EVT);
    }

    osal_set_event(gapBondMgr_TaskID, GAP_BOND_SAVE_REC_EVT);

    return (events ^ GAP_BOND_SAVE_REC_EVT);
  }

  if (events & GAP_BOND_SYNC_CC_EVT)
  {
    // Update NV to have same CCC values as GATT database
    // Note: pAuthEvt is a global variable used for deferring the storage
    if (gapBondMgr_SyncCharCfg(pAuthEvt->connectionHandle))
    {
      if (pGapBondCB && pGapBondCB->pairStateCB)
      {
        // Assume SUCCESS since we got this far.
        pGapBondCB->pairStateCB(pAuthEvt->connectionHandle, GAPBOND_PAIRING_STATE_COMPLETE, SUCCESS);

        // Bonding record was successfully saved in NV
        pGapBondCB->pairStateCB(pAuthEvt->connectionHandle, GAPBOND_PAIRING_STATE_BOND_SAVED, SUCCESS);
      }

#ifndef GBM_GATT_NO_CLIENT
#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
      // Read peer's Central Address Resolution characteristic
      // Requred before a peripheral device can use RPA for initiator address
      // within directed advertisements. BT 4.2 Vol. 3, Part C, Section 12.4
      if (linkDB_Role(pAuthEvt->connectionHandle) == GAP_PROFILE_PERIPHERAL)
      {
        bStatus_t ret;

        // Read CAR characteristic's value attribute of peer device
        ret = GAPBondMgr_ReadCentAddrResChar(pAuthEvt->connectionHandle);

        // If read fails notify application that read failed
        if (ret != SUCCESS && pGapBondCB && pGapBondCB->pairStateCB)
        {
            pGapBondCB->pairStateCB(pAuthEvt->connectionHandle,
                                     GAPBOND_PAIRING_STATE_CAR_READ,
                                     ret);
        }
      }
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG
#endif //!GBM_GATT_NO_CLIENT

      // We're done storing bond record and CCC values in NV
      gapBondFreeAuthEvt();

      return (events ^ GAP_BOND_SYNC_CC_EVT);
    }

    osal_set_event(gapBondMgr_TaskID, GAP_BOND_SYNC_CC_EVT);

    return (events ^ GAP_BOND_SYNC_CC_EVT);
  }

  if (events & GAP_BOND_SAVE_RCA_EVT)
  {
#if (HOST_CONFIG & PERIPHERAL_CFG)

#if defined (GAP_PRIVACY_RECONNECT)
    if (reconnectAddrIdx < GAP_BONDINGS_MAX)
    {
      // Save the new reconnection address
      VOID gapBondMgrUpdateReconnectAddr(reconnectAddrIdx);

      reconnectAddrIdx = GAP_BONDINGS_MAX;
    }
#endif // GAP_PRIVACY_RECONNECT

#endif // PERIPHERAL_CFG

    return (events ^ GAP_BOND_SAVE_RCA_EVT);
  }

#ifdef GBM_QUEUE_PAIRINGS
  if (events & GAP_BOND_POP_PAIR_QUEUE_EVT)
  {
    if (pPairingQ)
    {
      gapBondPairQueueNode_t *pPair = pPairingQ;

      // begin authentication
      gapBondMgrAuthenticate(pPair->connHandle, pPair->addrType,
                             pPair->pPairReq);

      // Move to the next queue item (or NULL if none exists).
      pPairingQ = pPairingQ->pNext;

      // Free pPairReq parameter (if used)
      if (pPair->pPairReq)
      {
        osal_mem_free(pPair->pPairReq);
      }

      // Free the former queue head.
      osal_mem_free(pPair);
    }

    return (events ^ GAP_BOND_POP_PAIR_QUEUE_EVT);
  }
#endif //GBM_QUEUE_PAIRINGS

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
  if (events & GAP_BOND_SYNC_RL_EVT)
  {
    GAPBondMgr_syncResolvingList();

    return (events ^ GAP_BOND_SYNC_RL_EVT);
  }
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      gapBondMgr_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t gapBondMgr_ProcessOSALMsg(osal_event_hdr_t *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      safeToDealloc = GAPBondMgr_ProcessGAPMsg((gapEventHdr_t *)pMsg);
      break;

    case GATT_MSG_EVENT:
      gapBondMgr_ProcessGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case GATT_SERV_MSG_EVENT:
      gapBondMgr_ProcessGATTServMsg((gattEventHdr_t *)pMsg);
      break;

    default:
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      GAPBondMgr_CheckNVLen
 *
 * @brief   This function will check the length of an NV Item.
 *
 * @param   id - NV ID.
 * @param   len - lengths in bytes of item.
 *
 * @return  SUCCESS or FAILURE
 */
uint8_t GAPBondMgr_CheckNVLen(uint8_t id, uint8_t len)
{
  uint8_t stat = FAILURE;

  // Convert to index
  switch ((id - BLE_NVID_GAP_BOND_START) % GAP_BOND_REC_IDS)
  {
    case GAP_BOND_REC_ID_OFFSET:
      if (len == sizeof (gapBondRec_t))
      {
        stat = SUCCESS;
      }
      break;

    case GAP_BOND_LOCAL_LTK_OFFSET:
#ifndef SNP_SECURITY
    case GAP_BOND_DEV_LTK_OFFSET:
#endif //SNP_SECURITY
      if (len == sizeof (gapBondLTK_t))
      {
        stat = SUCCESS;
      }
      break;

    case GAP_BOND_DEV_IRK_OFFSET:
#ifndef SNP_SECURITY
    case GAP_BOND_DEV_CSRK_OFFSET:
#endif //SNP_SECURITY
      if (len == KEYLEN)
      {
        stat = SUCCESS;
      }
      break;

#ifndef SNP_SECURITY
    case GAP_BOND_DEV_SIGN_COUNTER_OFFSET:
      if (len == sizeof (uint32_t))
      {
        stat = SUCCESS;
      }
      break;
#endif //SNP_SECURITY

    default:
      break;
  }

  return (stat);
}

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
/*********************************************************************
 * @fn      GAPBondMgr_syncResolvingList
 *
 * @brief   Add all devices found in bond record to the resolving list in
 *          the controller.
 *
 * @param   none
 *
 * @return  SUCCESS or FAILURE
 */
bStatus_t GAPBondMgr_syncResolvingList(void)
{
  uint8_t i;

  // If GAP is not idle then modifcations to the Controller Resolving List
  // will not be allowed
  if (gapState != GAP_STATE_IDLE)
  {
    return FAILURE;
  }

  // Clear current contents of resolving list
  HCI_LE_ClearResolvingListCmd();

  // Add device's local IRK
  HCI_LE_AddDeviceToResolvingListCmd(NULL, NULL, NULL, GAP_GetIRK());

  // Write bond addresses into the Resolving List
  for(i = 0; i < GAP_BONDINGS_MAX; i++)
  {
    // Make sure empty addresses are not added to the Resolving List
    if (osal_isbufset(bonds[i].publicAddr, 0xFF, B_ADDR_LEN) == FALSE)
    {
      uint8_t IRK[KEYLEN];

      // Read in NV IRK Record
      if (osal_snv_read(devIRKNvID(i), KEYLEN, IRK) == SUCCESS)
      {
        // Make sure read NV IRK is valid
        if (osal_isbufset(IRK, 0xFF, KEYLEN) == FALSE)
        {
          // Resolving list does not use ID addr types so must mask away bit
          // prior to adding device to list
          HCI_LE_AddDeviceToResolvingListCmd((bonds[i].publicAddrType &
                                                MASK_ADDRTYPE_ID),
                                              bonds[i].publicAddr,
                                              IRK, NULL);
        }
      }
    }
  }

  // Clear any flag noting that the resolving list must be resynched since
  // that has now complete
  gapBond_syncRL = FALSE;

  return SUCCESS;
}
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG

/*********************************************************************
 * @fn          gapBondMgr_ProcessGATTMsg
 *
 * @brief       Process an incoming GATT message.
 *
 * @param       pMsg - pointer to received message
 *
 * @return      none
 */
static void gapBondMgr_ProcessGATTMsg(gattMsgEvent_t *pMsg)
{
  // Process the GATT message
  switch (pMsg->method)
  {
#ifndef GATT_NO_SERVICE_CHANGED
    case ATT_HANDLE_VALUE_CFM:
      // Clear Service Changed flag for this client
      VOID GAPBondMgr_ServiceChangeInd(pMsg->connHandle, 0x00);
      break;
#endif // GATT_NO_SERVICE_CHANGED

#ifndef GBM_GATT_NO_CLIENT
#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG) && \
    (HOST_CONFIG & PERIPHERAL_CFG)
    case ATT_READ_BY_TYPE_RSP:
      // Read of Central Address Resolution complete, store in bond record
      gapBondMgrReadCARRsp(pMsg->connHandle, pMsg->msg.readByTypeRsp);
      break;

    case ATT_ERROR_RSP:
      gapBondMgr_ProcessAttErrRsp(pMsg->msg.errorRsp);
      break;
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG & PERIPHERAL_CFG
#endif //!GBM_GATT_NO_CLIENT

    default:
      // Unknown message
      break;
  }

  GATT_bm_free(&pMsg->msg, pMsg->method);
}

#ifndef GBM_GATT_NO_CLIENT
#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG) && \
    (HOST_CONFIG & PERIPHERAL_CFG)
/*********************************************************************
 * @fn          gapBondMgr_ProcessAttErrRsp
 *
 * @brief       Process an incoming ATT Error Response message
 *
 * @param       rsp - att error message received
 *
 * @return      none
 */
static void gapBondMgr_ProcessAttErrRsp(attErrorRsp_t rsp)
{
  switch(rsp.reqOpcode)
  {
    case ATT_READ_BY_TYPE_REQ:
      {
        // If Central Address Resolution does not exist than Enhanced Privacy
        // is not supported
        bStatus_t ret = bleIncorrectMode;

        if (rsp.errCode == ATT_ERR_ATTR_NOT_FOUND)
        {
          // Set flag in bond record and notify application
          ret = gapBondMgrSetEnhancedPrivFlag(rsp.handle, GAP_PRIVACY_DISABLED);
        }

        // Notify application of unsuccessful read of CAR characteristic
        if (pGapBondCB && pGapBondCB->pairStateCB)
        {
          pGapBondCB->pairStateCB(rsp.handle, GAPBOND_PAIRING_STATE_CAR_READ,
                                   ret);
        }
      }
      break;
    default:
      // Error Response to Unknown Request
      break;
  }
}
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG & PERIPHERAL_CFG
#endif //!GBM_GATT_NO_CLIENT

/*********************************************************************
 * @fn          gapBondMgr_ProcessGATTServMsg
 *
 * @brief       Process an incoming GATT Server App message.
 *
 * @param       pMsg - pointer to received message
 *
 * @return      none
 */
static void gapBondMgr_ProcessGATTServMsg(gattEventHdr_t *pMsg)
{
  // Process the GATT Server App message
  switch (pMsg->method)
  {
    case GATT_CLIENT_CHAR_CFG_UPDATED_EVENT:
      {
        gattClientCharCfgUpdatedEvent_t *pEvent = (gattClientCharCfgUpdatedEvent_t *)pMsg;

        VOID GAPBondMgr_UpdateCharCfg(pEvent->connHandle, pEvent->attrHandle, pEvent->value);
      }
      break;

    default:
      // Unknown message
      break;
  }
}

#ifndef GATT_NO_SERVICE_CHANGED
/*********************************************************************
 * @fn      gapBondMgrSendServiceChange
 *
 * @brief   Tell the GATT that a service change is needed.
 *
 * @param   pLinkItem - pointer to connection information
 *
 * @return  none
 */
static void gapBondMgrSendServiceChange(linkDBItem_t *pLinkItem)
{
  VOID GATTServApp_SendServiceChangedInd(pLinkItem->connectionHandle,
                                          gapBondMgr_TaskID);
}
#endif // GATT_NO_SERVICE_CHANGED

/*********************************************************************
 * @fn      gapBondSetupPrivFlag
 *
 * @brief   Setup the GAP Privacy Flag properties.
 *
 * @param   none
 *
 * @return  none
 */
static void gapBondSetupPrivFlag(void)
{
  uint8_t privFlagProp;

  if (gapBondMgrBondTotal() > 1)
  {
    privFlagProp = GATT_PROP_READ;
  }
  else
  {
    privFlagProp = GATT_PROP_READ | GATT_PROP_WRITE;
  }

  // Setup the
  VOID GGS_SetParameter(GGS_PERI_PRIVACY_FLAG_PROPS, sizeof (uint8_t), &privFlagProp);
}

/*********************************************************************
 * @fn      gapBondMgrAuthenticate
 *
 * @brief   Initiate authentication
 *
 * @param   connHandle - connection handle
 * @param   addrType - peer address type
 * @param   pPairReq - Enter these parameters if the Pairing Request was already received.
 *          NULL, if waiting for Pairing Request or if initiating.
 *
 * @return  SUCCESS
 *          bleIncorrectMode: Not correct profile role,
 *          INVALIDPARAMETER,
 *          bleNotConnected,
 *          bleAlreadyInRequestedMode,
 *          FAILURE - not workable.
 */
static bStatus_t gapBondMgrAuthenticate(uint16_t connHandle, uint8_t addrType,
                                         gapPairingReq_t *pPairReq)
{
  gapAuthParams_t params;
  linkDBItem_t *pLinkItem;

  // Find the connection
  pLinkItem = linkDB_Find(connHandle);
  if (pLinkItem == NULL)
  {
    return bleNotConnected;
  }

  VOID osal_memset(&params, 0, sizeof (gapAuthParams_t));

  // Setup the pairing parameters
  params.connectionHandle = connHandle;
  params.secReqs.ioCaps = gapBond_IOCap;
  params.secReqs.maxEncKeySize = gapBond_KeySize;

  params.secReqs.keyDist.sEncKey = (gapBond_KeyDistList & GAPBOND_KEYDIST_SENCKEY) ? TRUE : FALSE;
  params.secReqs.keyDist.sIdKey = (gapBond_KeyDistList & GAPBOND_KEYDIST_SIDKEY) ? TRUE : FALSE;
  params.secReqs.keyDist.mEncKey = (gapBond_KeyDistList & GAPBOND_KEYDIST_MENCKEY) ? TRUE : FALSE;
  params.secReqs.keyDist.mIdKey = (gapBond_KeyDistList & GAPBOND_KEYDIST_MIDKEY) ? TRUE : FALSE;
  params.secReqs.keyDist.mSign = (gapBond_KeyDistList & GAPBOND_KEYDIST_MSIGN) ? TRUE : FALSE;
  params.secReqs.keyDist.sSign = (gapBond_KeyDistList & GAPBOND_KEYDIST_SSIGN) ? TRUE : FALSE;


#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & SECURE_CONNS_CFG)
  // If Secure Connections Only Mode
  params.secReqs.isSCOnlyMode = (gapBond_secureConnection==GAPBOND_SECURE_CONNECTION_ONLY) ? TRUE : FALSE;

  // Add Secure Connections to the authentication requirements if enabled and
  // if supported by the MTU size.
  if (L2CAP_GetMTU() >= SECURECONNECTION_MIN_MTU_SIZE)
  {
    params.secReqs.authReq |= (gapBond_secureConnection) ?
                               SM_AUTH_STATE_SECURECONNECTION : 0;
  }

  // If we want to specify the keys
  if (gapBond_useEccKeys)
  {
    params.secReqs.eccKeys.isUsed = TRUE;
    osal_memcpy(params.secReqs.eccKeys.sK, gapBond_eccKeys.privateKey, ECC_KEYLEN);
    osal_memcpy(params.secReqs.eccKeys.pK_x, gapBond_eccKeys.publicKeyX, ECC_KEYLEN);
    osal_memcpy(params.secReqs.eccKeys.pK_y, gapBond_eccKeys.publicKeyY, ECC_KEYLEN);
  }
  else
  {
    params.secReqs.eccKeys.isUsed = FALSE;
  }

  // If local OOB data is available
  if (gapBond_localOobScDataFlag)
  {
    params.secReqs.localOobAvailable = gapBond_localOobScDataFlag;
    VOID osal_memcpy(params.secReqs.localOob, gapBond_localOobSCData, KEYLEN);
  }

  // Note: if gapBond_OobScDataFlag is defined gapBond_OOBData is not used.
  // If the bond manager is set up for OOB dats
  if (gapBond_OobScDataFlag && osal_memcmp(gapBond_remoteOobSCData.addr, pLinkItem->addr, B_ADDR_LEN))
  {
    params.secReqs.oobAvailable = gapBond_OobScDataFlag;
    VOID osal_memcpy(params.secReqs.oob, gapBond_remoteOobSCData.oob, KEYLEN);
    VOID osal_memcpy(params.secReqs.oobConfirm, gapBond_remoteOobSCData.confirm, KEYLEN);
  }
  else
#endif //(BLE_V42_FEATURES & SECURE_CONNS_CFG)
       if (gapBond_OOBDataFlag)
  {
    params.secReqs.oobAvailable = gapBond_OOBDataFlag;
    VOID osal_memcpy(params.secReqs.oob, gapBond_OOBData, KEYLEN);
  }
  else
  {
    params.secReqs.oobAvailable = FALSE;
  }

  if (gapBond_Bonding && addrType != ADDRTYPE_PUBLIC)
  {
    // Force a slave ID key
    params.secReqs.keyDist.sIdKey = TRUE;
  }

  params.secReqs.authReq |= (gapBond_Bonding) ? SM_AUTH_STATE_BONDING : 0;
  params.secReqs.authReq |= (gapBond_MITM) ? SM_AUTH_STATE_AUTHENTICATED : 0;

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & SECURE_CONNS_CFG)

#endif //(BLE_V42_FEATURES & SECURE_CONNS_CFG)

#ifdef GBM_QUEUE_PAIRINGS
  {
    uint8_t stat = GAP_Authenticate(&params, pPairReq) ;
    if (stat == bleAlreadyInRequestedMode)
    {
      // We are already pairing with another device.  Queue up the request.
      // need to store connHandle, addrType.
      // Note:  assumes that slaves will not be tied up
      gapBondMgrQueuePairing(connHandle, addrType, pPairReq);
    }
    else if (stat != SUCCESS)
    {
      // This pairing failed, try the next one.
      if (pPairingQ)
      {
        osal_set_event(gapBondMgr_TaskID, GAP_BOND_POP_PAIR_QUEUE_EVT);
      }
    }

    return stat;
  }
#else
  return GAP_Authenticate(&params, pPairReq);
#endif //GBM_QUEUE_PAIRINGS
}

#ifndef GBM_GATT_NO_CLIENT
#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG) && \
    (HOST_CONFIG & PERIPHERAL_CFG)
/*********************************************************************
 * @fn      gapBondMgrReadCARRsp
 *
 * @brief   Determine whether or not Privacy 1.2 is supported on peer
 *          device through Central Address Resolution chracteristic read
 *          response
 *
 * @param   connHandle -  the connection handle.
 * @param   rsp -         Read by Type response message for central
 *                        address resolution UUID
 *
 * @return  None
 */
static void gapBondMgrReadCARRsp(uint16_t connHandle, attReadByTypeRsp_t rsp)
{
  uint8_t value;
  bStatus_t ret = bleNoResources; // return value

  // Parameter Check:
  // Should only exist one Central Address Resolution characteristic on
  // central peer device with a one octet value
  // BT 4.2 Vol. 3, Part C, Section 12.4
  if (rsp.numPairs != NUM_CENT_ADDR_RES_CHAR ||
       rsp.len != CENT_ADDR_RES_RSP_LEN)
  {
    ret = bleInvalidRange;
  }
  else
  {
    // Read By Type response data list should only contain one handle-value pair
    // First two octets are att handle, third octet is value of characteristic
    value = rsp.pDataList[2];

    // Set Bond Record Enhanced Privacy flag for this connection
    ret = gapBondMgrSetEnhancedPrivFlag(connHandle, value);
  }

  // Notify application if CAR was unsuccessfully read or if there
  // was an error when storing in NV
  if (pGapBondCB && pGapBondCB->pairStateCB)
  {
    pGapBondCB->pairStateCB(connHandle, GAPBOND_PAIRING_STATE_CAR_READ,
                             ret);
  }
}
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG & PERIPHERAL_CFG
#endif //!GBM_GATT_NO_CLIENT

#ifndef GBM_GATT_NO_CLIENT
#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG) && \
    (HOST_CONFIG & PERIPHERAL_CFG)
/*********************************************************************
 * @fn      gapBondMgrSetEnhancedPrivFlag
 *
 * @brief   Set the enhanced privacy flag in the bond record corresponding
 *          to the connection handle provided
 *
 * @param   connHandle -  the connection handle.
 * @param   setParam -    value to update enhanced privacy flag with
 *
 * @return      SUCCESS - bond record found and changed,<BR>
 *              bleNoResources - bond record not found (for 0xFFFF connHandle),<BR>
 *              bleNotConnected - connection not found - connHandle is invalid
 *                                (for non-0xFFFF connHandle).
 */
static bStatus_t gapBondMgrSetEnhancedPrivFlag(uint16_t connHandle,
                                                uint8_t setParam)
{
  bStatus_t ret = bleNoResources; // return value

  if (connHandle == INVALID_CONNHANDLE)
  {
    uint8_t idx;  // loop counter

    // Run through the bond database and update the Enhanced Privacy Flag
    for (idx = 0; idx < GAP_BONDINGS_MAX; idx++)
    {
      if (gapBondMgrChangeState(idx, GAP_BONDED_STATE_ENHANCED_PRIVACY,
                                  setParam))
      {
        ret = SUCCESS;
      }
    }
  }
  else
  {
    // Find connection information
    linkDBItem_t *pLinkItem = linkDB_Find(connHandle);
    if (pLinkItem)
    {
      uint8_t idx; // loop counter
      idx = GAPBondMgr_ResolveAddr(pLinkItem->addrType, pLinkItem->addr,
                                    NULL);
      if (idx < GAP_BONDINGS_MAX)
      {
        // Bond found, update it.
        gapBondMgrChangeState(idx, GAP_BONDED_STATE_ENHANCED_PRIVACY,
                               setParam);
        ret = SUCCESS;
      }
    }
    else
    {
      ret = bleNotConnected;
    }
  }

  return (ret);
}
#endif //BLE_V42_FEATURES & PRIVACY_1_2_CFG & PERIPHERAL_CFG
#endif //!GBM_GATT_NO_CLIENT

#if (HOST_CONFIG & PERIPHERAL_CFG)
/*********************************************************************
 * @fn      gapBondMgrSlaveSecurityReq
 *
 * @brief   Send a slave security request
 *
 * @param   connHandle - connection handle
 *
 * @return  none
 */
static void gapBondMgrSlaveSecurityReq(uint16_t connHandle)
{
  uint8_t authReq = 0;

  authReq |= (gapBond_Bonding) ? SM_AUTH_STATE_BONDING : 0;
  authReq |= (gapBond_MITM) ? SM_AUTH_STATE_AUTHENTICATED : 0;

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & SECURE_CONNS_CFG)
  //Secure Connection
  authReq |= (gapBond_secureConnection) ? SM_AUTH_STATE_SECURECONNECTION : 0;
#endif //(BLE_V42_FEATURES & SECURE_CONNS_CFG)

  VOID GAP_SendSlaveSecurityRequest(connHandle, authReq);
}

#if defined (GAP_PRIVACY_RECONNECT)
/*********************************************************************
 * @fn      gapBondMgrUpdateReconnectAddr
 *
 * @brief   Update the reconnection address field of a bond record.
 *
 * @param   idx - Bond NV index
 *
 * @return  TRUE if reconnection address saved in NV. FALSE, otherwise.
 */
static uint8_t gapBondMgrUpdateReconnectAddr(uint8_t idx)
{
  gapBondRec_t bondRec;   // Space to read a Bond record from NV

  // First see if bonding record exists in NV (public address in not all 0xFF's)
  if ((osal_snv_read(mainRecordNvID(idx), sizeof (gapBondRec_t), &bondRec) == SUCCESS)
      && (osal_isbufset(bondRec.publicAddr, 0xFF, B_ADDR_LEN) == FALSE))
  {
    // Update the reconnection address
    VOID osal_memcpy(bondRec.reconnectAddr, bonds[idx].reconnectAddr, B_ADDR_LEN);

    // Write out the entire bond entry
    VOID osal_snv_write(mainRecordNvID(idx), sizeof (gapBondRec_t), &bondRec);

    return (TRUE);
  }

  return (FALSE);
}

/*********************************************************************
 * @fn      gapBondMgrAttrValueChangeCB
 *
 * @brief   Attribute value change callback.
 *
 * @param   connHandle - connection handle
 * @param   attrId - attribute id
 *
 * @return  void
 */
static void gapBondMgrAttrValueChangeCB(uint16_t connHandle, uint8_t attrId)
{
  switch (attrId)
  {
    case GGS_RECONNCT_ADDR_ATT:
    {
      // Reconnection address has been updated
      linkDBItem_t *pLinkItem = linkDB_Find(connHandle);
      if (pLinkItem)
      {
        uint8_t idx = gapBondMgrFindAddr(pLinkItem->addr);
        if (idx < GAP_BONDINGS_MAX)
        {
          uint8_t reconnectAddr[B_ADDR_LEN];

          // Get the new reconnection address
          if (GGS_GetParameter(GGS_RECONNCT_ADDR_ATT, reconnectAddr) == SUCCESS)
          {
            // Reverse bytes before saving the new reconnection address
            VOID osal_revmemcpy(bonds[idx].reconnectAddr, reconnectAddr, B_ADDR_LEN);

            // Remember bond index for the reconnection address
            reconnectAddrIdx = idx;

            // Notify our task to save the reconnection address in NV
            osal_set_event(gapBondMgr_TaskID, GAP_BOND_SAVE_RCA_EVT);
          }
        }
      }
    }
    break;

    default:
      break;
  }
}
#endif // GAP_PRIVACY_RECONNECT

#endif // PERIPHERAL_CFG

/*********************************************************************
 * @fn      gapBondMgrBondReq
 *
 * @brief   Initiate a GAP bond request
 *
 * @param   connHandle - connection handle
 * @param   idx - NV index of bond entry
 * @param   stateFlags - bond state flags
 * @param   role - master or slave role
 * @param   startEncryption - whether or not to start encryption
 *
 * @return  none
 */
static void gapBondMgrBondReq(uint16_t connHandle, uint8_t idx, uint8_t stateFlags,
                               uint8_t role, uint8_t startEncryption)
{
  smSecurityInfo_t ltk;
  osalSnvId_t      nvId;

#ifndef SNP_SECURITY
  if (role == GAP_PROFILE_CENTRAL && !(stateFlags & GAP_BONDED_STATE_SECURECONNECTION))
  {
    nvId = devLTKNvID(idx);
  }
  else
#endif //SNP_SECURITY
  {
    nvId = localLTKNvID(idx);
  }

  // Initialize the NV structures
  VOID osal_memset(&ltk, 0, sizeof (smSecurityInfo_t));

  if (osal_snv_read(nvId, sizeof (smSecurityInfo_t), &ltk) == SUCCESS)
  {
    if ((ltk.keySize >= MIN_ENC_KEYSIZE) && (ltk.keySize <= MAX_ENC_KEYSIZE))
    {
      uint8_t authenticated;
      uint8_t secureConnections;

      authenticated = (stateFlags & GAP_BONDED_STATE_AUTHENTICATED) ? TRUE : FALSE;
#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & SECURE_CONNS_CFG)
      secureConnections = (stateFlags & GAP_BONDED_STATE_SECURECONNECTION) ? TRUE: FALSE;
#else
      secureConnections = FALSE;
#endif //(BLE_V42_FEATURES & SECURE_CONNS_CFG)

      VOID GAP_Bond(connHandle, authenticated, secureConnections, &ltk,
                     startEncryption);
    }
  }
}

/*********************************************************************
 * @fn      gapBondMgr_SyncWhiteList
 *
 * @brief   synchronize the White List with the bonds
 *
 * @param   none
 *
 * @return  none
 */
static void gapBondMgr_SyncWhiteList(void)
{
  uint8_t i;

  //erase the White List
  VOID HCI_LE_ClearWhiteListCmd();

  // Write bond addresses into the White List
  for(i = 0; i < GAP_BONDINGS_MAX; i++)
  {
    // Make sure empty addresses are not added to the White List
    if (osal_isbufset(bonds[i].publicAddr, 0xFF, B_ADDR_LEN) == FALSE)
    {
      VOID HCI_LE_AddWhiteListCmd(HCI_PUBLIC_DEVICE_ADDRESS, bonds[i].publicAddr);
    }
  }
}

/*********************************************************************
 * @fn          gapBondMgr_SyncCharCfg
 *
 * @brief       Update the Bond Manager to have the same configurations as
 *              the GATT database.
 *
 * @param       connHandle - the current connection handle to find client configurations for
 *
 * @return      TRUE if sync done. FALSE, otherwise.
 */
static uint8_t gapBondMgr_SyncCharCfg(uint16_t connHandle)
{
  static gattAttribute_t *pAttr = NULL;
  static uint16_t service;

  // Only attributes with attribute handles between and including the Starting
  // Handle parameter and the Ending Handle parameter that match the requested
  // attribute type and the attribute value will be returned.

  // All attribute types are effectively compared as 128-bit UUIDs,
  // even if a 16-bit UUID is provided in this request or defined
  // for an attribute.
  if (pAttr == NULL)
  {
    pAttr = GATT_FindHandleUUID(GATT_MIN_HANDLE, GATT_MAX_HANDLE,
                                 clientCharCfgUUID, ATT_BT_UUID_SIZE, &service);
  }

  if (pAttr != NULL)
  {
    uint16_t len;
    uint8_t attrVal[ATT_BT_UUID_SIZE];

    // It is not possible to use this request on an attribute that has a value
    // that is longer than 2.
    if (GATTServApp_ReadAttr(connHandle, pAttr, service, attrVal,
                               &len, 0, ATT_BT_UUID_SIZE, 0xFF) == SUCCESS)
    {
      uint16_t value = BUILD_UINT16(attrVal[0], attrVal[1]);

      if (value != GATT_CFG_NO_OPERATION)
      {
        // NV must be updated to meet configuration of the database
        VOID GAPBondMgr_UpdateCharCfg(connHandle, pAttr->handle, value);
      }
    }

    // Try to find the next attribute
    pAttr = GATT_FindNextAttr(pAttr, GATT_MAX_HANDLE, service, NULL);
  }

  return (pAttr == NULL);
}

/*********************************************************************
 * @fn          gapBondFreeAuthEvt
 *
 * @brief       Free GAP Authentication Complete event.
 *
 * @param       none
 *
 * @return      none
 */
static void gapBondFreeAuthEvt(void)
{
  if (pAuthEvt != NULL)
  {
    // Release the OSAL message
    VOID osal_msg_deallocate((uint8_t *)pAuthEvt);
    pAuthEvt = NULL;
  }

  bondIdx = GAP_BONDINGS_MAX;
}

#ifdef GBM_QUEUE_PAIRINGS
static void gapBondMgrQueuePairing(uint16_t connHandle, uint8_t addrType,
                                    gapPairingReq_t *pPairReq)
{
  // Allocated space on the queue
  gapBondPairQueueNode_t * pairing = (gapBondPairQueueNode_t *) osal_mem_alloc(sizeof(gapBondPairQueueNode_t));

  if (pairing)
  {
    pairing->connHandle = connHandle;
    pairing->addrType = addrType;
    pairing->pPairReq = NULL;
    pairing->pNext = NULL;

    // If there was a pairing request message.
    if (pPairReq)
    {
      pairing->pPairReq = (gapPairingReq_t *)osal_memdup(pPairReq, sizeof(gapPairingReq_t));

      if (!pairing->pPairReq)
      {
        // malloc failed, free pairing and return
        osal_mem_free(pairing);

        return;
      }
    }

    // Check if queue is empty
    if (pPairingQ == NULL)
    {
      pPairingQ = pairing;
    }
    else
    {
      gapBondPairQueueNode_t *qNode = pPairingQ;

      while (qNode->pNext != NULL)
      {
        qNode = qNode->pNext;
      }

      qNode->pNext = pairing;
    }
  }
}
#endif //GBM_QUEUE_PAIRINGS


/*********************************************************************
 * @fn          gapGetRandomAddrSubType
 *
 * @brief       Determine the random address subtype type from the address.
 *
 * @param       addr - 6 byte address
 *
 * @return      returns the random subtype of address
 *              @ref RANDOM_ADDR_SUBTYPE_DEFINES
 */
uint8_t gapBondGetRandomAddrSubType(uint8_t *pAddr)
{
  uint8_t addrSubtype = RANDOM_ADDR_SUBTYPE_NRPA;

  // Get just the random address header bits
  uint8_t addrTypeMask = (uint8_t)(pAddr[B_ADDR_LEN-1] & RANDOM_ADDR_HDR_MASK);

  // Get the address type from the address header
  if (addrTypeMask == STATIC_ADDR_HDR)
  {
    addrSubtype = RANDOM_ADDR_SUBTYPE_STATIC;
  }
  else if (addrTypeMask == PRIVATE_RESOLVE_ADDR_HDR)
  {
    addrSubtype = RANDOM_ADDR_SUBTYPE_RPA;
  }

  return addrSubtype;
}

#endif // (CENTRAL_CFG | PERIPHERAL_CFG)

/*********************************************************************
*********************************************************************/
