#include <bcomdef.h>
#include <ti/display/Display.h>
#include <menu/two_btn_menu.h>
#include "throughput_central_menu.h"
#include "throughput_central.h"

/*
 * Menu Lists Initializations
 */

// Menu: Main
// upper: none
MENU_OBJ(tcMenuMain, "Top Menu", 6, NULL)
  MENU_ITEM_SUBMENU(&tcMenuSelectConn)
  MENU_ITEM_SUBMENU(&tcMenuConnect)
  MENU_ITEM_ACTION("Cancel Connecting", ThroughputCentral_doCancelConnecting)
  MENU_ITEM_ACTION("Discover Devices",  ThroughputCentral_doDiscoverDevices)
  MENU_ITEM_ACTION("Stop Discovering",  ThroughputCentral_doStopDiscovering)
  MENU_ITEM_SUBMENU(&tcMenuScanPhy)
MENU_OBJ_END

// Menu: ScanPhy
// upper: Main
MENU_OBJ(tcMenuScanPhy, "Set Scanning PHY", 2, &tcMenuMain)
  MENU_ITEM_ACTION("1 Mbps", ThroughputCentral_doSetScanPhy)
  MENU_ITEM_ACTION("Coded",  ThroughputCentral_doSetScanPhy)
MENU_OBJ_END

// Menu: Connect
// upper: Main
// NOTE: The number of items in this menu object shall be equal to
//       or greater than DEFAULT_MAX_SCAN_RES.
//       The number of items cannot exceed 27 which is the two-button menu's
//       constraint.
MENU_OBJ(tcMenuConnect, "Connect to", DEFAULT_MAX_SCAN_RES, &tcMenuMain)
  MENU_ITEM_ACTION(NULL, ThroughputCentral_doConnect)
  MENU_ITEM_ACTION(NULL, ThroughputCentral_doConnect)
  MENU_ITEM_ACTION(NULL, ThroughputCentral_doConnect)
  MENU_ITEM_ACTION(NULL, ThroughputCentral_doConnect)
  MENU_ITEM_ACTION(NULL, ThroughputCentral_doConnect)
  MENU_ITEM_ACTION(NULL, ThroughputCentral_doConnect)
  MENU_ITEM_ACTION(NULL, ThroughputCentral_doConnect)
  MENU_ITEM_ACTION(NULL, ThroughputCentral_doConnect)
MENU_OBJ_END

// Menu: SelectDev
// upper: Main
// NOTE: The number of items in this menu object shall be
//       equal to or greater than MAX_NUM_BLE_CONNS
MENU_OBJ(tcMenuSelectConn, "Work with", MAX_NUM_BLE_CONNS, &tcMenuMain)
  MENU_ITEM_ACTION(NULL, ThroughputCentral_doSelectConn)
  MENU_ITEM_ACTION(NULL, ThroughputCentral_doSelectConn)
  MENU_ITEM_ACTION(NULL, ThroughputCentral_doSelectConn)
  MENU_ITEM_ACTION(NULL, ThroughputCentral_doSelectConn)
MENU_OBJ_END

// Menu: PerConnection
// upper: SelectDevice
MENU_OBJ(tcMenuPerConn, NULL, 4, &tcMenuSelectConn)
  MENU_ITEM_ACTION("Toggle Throughput",  ThroughputCentral_doToggleThroughput)
  MENU_ITEM_SUBMENU(&tcMenuConnPhy)
  MENU_ITEM_SUBMENU(&tcMenuConnPdu)
  MENU_ITEM_ACTION("Disconnect",         ThroughputCentral_doDisconnect)

MENU_OBJ_END

// Menu: ConnPhy
// upper: Main
MENU_OBJ(tcMenuConnPhy, "Set Conn PHY", 4, &tcMenuPerConn) //Previous: PerConnection
  MENU_ITEM_ACTION("1 Mbps",              ThroughputCentral_doSetConnPhy)
  MENU_ITEM_ACTION("2 Mbps",              ThroughputCentral_doSetConnPhy)
  MENU_ITEM_ACTION("Coded:S2",            ThroughputCentral_doSetConnPhy)
  MENU_ITEM_ACTION("Coded:S8",            ThroughputCentral_doSetConnPhy)
MENU_OBJ_END

// Menu: ConnPdu
// upper: Main
MENU_OBJ(tcMenuConnPdu, "Set Conn PDU", 2, &tcMenuPerConn) //Previous: PerConnection
  MENU_ITEM_ACTION("27 Bytes",            ThroughputCentral_doSetDLEPDU)
  MENU_ITEM_ACTION("251 Bytes",           ThroughputCentral_doSetDLEPDU)
MENU_OBJ_END
