

#include <bcomdef.h>
#include <gapgattserver.h>
#include <gap_advertiser.h>
#include <app_main.h>
#include <gapbondmgr.h>
#include "ti_ble_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>

// The GAP profile role
uint8_t profileRole = GAP_PROFILE_PERIPHERAL;
// GAP GATT Service (GGS) parameters
uint8_t attDeviceName[GAP_DEVICE_NAME_LEN]= "Data Stream";

uint8_t pRandomAddress[B_ADDR_LEN] = {0};

// Initiate selected profiles
void init_profiles(){
 }
/*********************************************************************
 * Bond Manager Configuration
 */

gapBondParams_t gapBondParams = {
    .pairMode             = GAPBOND_PAIRING_MODE_NO_PAIRING,
    .mitm                 = false,
    .ioCap                = GAPBOND_IO_CAP_DISPLAY_ONLY,
    .bonding              = false,
    .secureConnection     = GAPBOND_SECURE_CONNECTION_ALLOW,
    .authenPairingOnly    = false,
    .autoSyncAL           = false,
    .eccReGenPolicy       = 0,
    .KeySize              = 16,
    .removeLRUBond        = false,
    .KeyDistList          = GAPBOND_KEYDIST_CENCKEY | GAPBOND_KEYDIST_CIDKEY | GAPBOND_KEYDIST_CSIGN | GAPBOND_KEYDIST_PENCKEY | GAPBOND_KEYDIST_PIDKEY | GAPBOND_KEYDIST_PSIGN,
    .eccDebugKeys         = false,
    .eraseBondWhileInConn = false,
    .sameIrkAction        = GAPBOND_SAME_IRK_UPDATE_BOND_REC
};

uint8_t pairMode                =    GAPBOND_PAIRING_MODE_NO_PAIRING;
uint8_t mitm                    =    false;
uint8_t ioCap                   =    GAPBOND_IO_CAP_DISPLAY_ONLY;
uint8_t bonding                 =    false;
uint8_t secureConnection        =    GAPBOND_SECURE_CONNECTION_ALLOW;
uint8_t authenPairingOnly       =    false;
uint8_t autoSyncAL              =    false;
uint8_t eccReGenPolicy          =    0;
uint8_t KeySize                 =    16;
uint8_t removeLRUBond           =    false;
uint8_t KeyDistList             =    GAPBOND_KEYDIST_CENCKEY | GAPBOND_KEYDIST_CIDKEY | GAPBOND_KEYDIST_CSIGN | GAPBOND_KEYDIST_PENCKEY | GAPBOND_KEYDIST_PIDKEY | GAPBOND_KEYDIST_PSIGN;
uint8_t eccDebugKeys            =    false;
uint8_t allowDebugKeys          =    true;
uint8_t eraseBondWhileInConn    =    false;
uint8_t sameIrkAction           =    GAPBOND_SAME_IRK_UPDATE_BOND_REC;

void setBondManagerParameters()
{
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &secureConnection);
    // Set Authenticated Pairing Only mode
    GAPBondMgr_SetParameter(GAPBOND_AUTHEN_PAIRING_ONLY, sizeof(uint8_t), &authenPairingOnly);
    // Set Auto Acceptlist Sync
    GAPBondMgr_SetParameter(GAP_ADV_AL_POLICY_ANY_REQ, sizeof(uint8_t), &autoSyncAL);
    GAPBondMgr_SetParameter(GAPBOND_ECCKEY_REGEN_POLICY, sizeof(uint8_t), &eccReGenPolicy);
    GAPBondMgr_SetParameter(GAPBOND_KEYSIZE, sizeof(uint8_t), &KeySize);
    GAPBondMgr_SetParameter(GAPBOND_LRU_BOND_REPLACEMENT, sizeof(uint8_t), &removeLRUBond);
    GAPBondMgr_SetParameter(GAPBOND_KEY_DIST_LIST, sizeof(uint8_t), &KeyDistList);
    // Set Secure Connection Debug Keys
    GAPBondMgr_SetParameter(GAPBOND_SC_HOST_DEBUG, sizeof(uint8_t), &eccDebugKeys);
    // Set Allow Debug Keys
    GAPBondMgr_SetParameter(GAPBOND_ALLOW_DEBUG_KEYS, sizeof(uint8_t), &allowDebugKeys);
    // Set the Erase bond While in Active Connection Flag
    GAPBondMgr_SetParameter(GAPBOND_ERASE_BOND_IN_CONN, sizeof(uint8_t), &eraseBondWhileInConn);
    GAPBondMgr_SetParameter(GAPBOND_SAME_IRK_OPTION, sizeof(uint8_t), &sameIrkAction);
}


GapAdv_params_t advParams1 = {
  .eventProps =   GAP_ADV_PROP_CONNECTABLE | GAP_ADV_PROP_LEGACY | GAP_ADV_PROP_SCANNABLE,
  .primIntMin =   32,
  .primIntMax =   64,
  .primChanMap =  GAP_ADV_CHAN_ALL,
  .peerAddrType = PEER_ADDRTYPE_PUBLIC_OR_PUBLIC_ID,
  .peerAddr =     { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa },
  .filterPolicy = GAP_ADV_AL_POLICY_ANY_REQ,
  .txPower =      GAP_ADV_TX_POWER_NO_PREFERENCE,
  .primPhy =      GAP_ADV_PRIM_PHY_1_MBPS,
  .secPhy =       GAP_ADV_SEC_PHY_1_MBPS,
  .sid =          0
};

uint8_t advData1[] =
{
  0x02,
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED | GAP_ADTYPE_FLAGS_GENERAL,

  0x11,
  GAP_ADTYPE_128BIT_MORE,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0xb0,
  0x00,
  0x40,
  0x51,
  0x04,
  0xc0,
  0xc0,
  0x00,
  0xf0,




  0x07,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  //Company Identifier
  0x0d,
  0x00,

  //Additional Data
  0xc0,
  0xff,
  0xee,

};

uint8_t scanResData1[] =
{
  0x0c,
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'D',
  'a',
  't',
  'a',
  ' ',
  'S',
  't',
  'r',
  'e',
  'a',
  'm',

  0x02,
  GAP_ADTYPE_POWER_LEVEL,
  0,


  0x05,
  GAP_ADTYPE_PERIPHERAL_CONN_INTERVAL_RANGE,
  LO_UINT16(80),
  HI_UINT16(80),
  LO_UINT16(104),
  HI_UINT16(104),

};

