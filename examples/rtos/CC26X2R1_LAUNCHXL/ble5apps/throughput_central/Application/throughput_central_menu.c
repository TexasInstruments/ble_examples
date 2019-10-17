#include <bcomdef.h>
#include <ti/display/Display.h>
#include <menu/two_btn_menu.h>
#include "throughput_central_menu.h"
#include "throughput_central.h"

/* Main Menu Object */
tbmMenuObj_t tcMenuMain;
tbmMenuObj_t tcMenuConnect;
tbmMenuObj_t tcMenuScanPhy;
tbmMenuObj_t tcMenuConnPhy;
tbmMenuObj_t tcMenuConnPdu;
tbmMenuObj_t tcMenuSelectConn;
tbmMenuObj_t tcMenuPerConn;

/*
 * Menu Lists Initializations
 */

void Throughput_buildMenu(void)
{
	// Menu: Main
	// upper: none
	MENU_OBJ(tcMenuMain, "Top Menu", 6, NULL)
	MENU_ITEM_SUBMENU(tcMenuMain,0,&tcMenuSelectConn)
	MENU_ITEM_SUBMENU(tcMenuMain,1,&tcMenuConnect)
	MENU_ITEM_ACTION(tcMenuMain,2,"Cancel Connecting",ThroughputCentral_doCancelConnecting)
	MENU_ITEM_ACTION(tcMenuMain,3,"Discover Devices",ThroughputCentral_doDiscoverDevices)
	MENU_ITEM_ACTION(tcMenuMain,4,"Stop Discovering",ThroughputCentral_doStopDiscovering)
	MENU_ITEM_SUBMENU(tcMenuMain,5,&tcMenuScanPhy)
	MENU_OBJ_END

	// Menu: ScanPhy
	// upper: Main
	MENU_OBJ(tcMenuScanPhy,"Set Scanning PHY",2,&tcMenuMain)
	MENU_ITEM_ACTION(tcMenuScanPhy,0,"1 Mbps",ThroughputCentral_doSetScanPhy)
	MENU_ITEM_ACTION(tcMenuScanPhy,1,"Coded",ThroughputCentral_doSetScanPhy)
	MENU_OBJ_END


// Menu: Connect
// upper: Main
// NOTE: The number of items in this menu object shall be equal to
//       or greater than DEFAULT_MAX_SCAN_RES.
//       The number of items cannot exceed 27 which is the two-button menu's
//       constraint.
  MENU_OBJ(tcMenuConnect,"Connect to",DEFAULT_MAX_SCAN_RES,&tcMenuMain)
  MENU_ITEM_MULTIPLE_ACTIONS(tcMenuConnect, DEFAULT_MAX_SCAN_RES, NULL, ThroughputCentral_doConnect)
  MENU_OBJ_END

// Menu: SelectDev
// upper: Main
// NOTE: The number of items in this menu object shall be
//       equal to or greater than MAX_NUM_BLE_CONNS
  MENU_OBJ(tcMenuSelectConn,"Work with", MAX_NUM_BLE_CONNS, &tcMenuMain)
  MENU_ITEM_MULTIPLE_ACTIONS(tcMenuSelectConn, MAX_NUM_BLE_CONNS, NULL, ThroughputCentral_doSelectConn)
  MENU_OBJ_END
  

// Menu: PerConnection
// upper: SelectDevice
MENU_OBJ(tcMenuPerConn, NULL, 4, &tcMenuSelectConn)
MENU_ITEM_ACTION(tcMenuPerConn, 0, "Toggle Throughput",  ThroughputCentral_doToggleThroughput)
MENU_ITEM_SUBMENU(tcMenuPerConn, 1, &tcMenuConnPhy)
MENU_ITEM_SUBMENU(tcMenuPerConn, 2, &tcMenuConnPdu)
MENU_ITEM_ACTION(tcMenuPerConn, 3, "Disconnect",         ThroughputCentral_doDisconnect)
MENU_OBJ_END

// Menu: ConnPhy
// upper: Main
MENU_OBJ(tcMenuConnPhy, "Set Conn PHY", 4, &tcMenuPerConn)
MENU_ITEM_ACTION(tcMenuConnPhy, 0, "1 Mbps",ThroughputCentral_doSetConnPhy)
MENU_ITEM_ACTION(tcMenuConnPhy, 1, "2 Mbps",ThroughputCentral_doSetConnPhy)
MENU_ITEM_ACTION(tcMenuConnPhy, 2, "Coded S2",ThroughputCentral_doSetConnPhy)
MENU_ITEM_ACTION(tcMenuConnPhy, 3, "Coded S8",ThroughputCentral_doSetConnPhy)
MENU_OBJ_END

// Menu: ConnPdu
// upper: Main
MENU_OBJ(tcMenuConnPdu, "Set Conn PDU", 2, &tcMenuPerConn)
MENU_ITEM_ACTION(tcMenuConnPdu, 0, "27 Bytes",ThroughputCentral_doSetDLEPDU)
MENU_ITEM_ACTION(tcMenuConnPdu, 1, "251 Bytes",ThroughputCentral_doSetDLEPDU)
MENU_OBJ_END
}
