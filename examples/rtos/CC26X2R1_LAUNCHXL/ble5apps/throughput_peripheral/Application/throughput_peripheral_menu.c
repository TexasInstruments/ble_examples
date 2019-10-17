#include <bcomdef.h>
#include <ti/display/Display.h>

#include <menu/two_btn_menu.h>
#include "throughput_peripheral_menu.h"
#include "throughput_peripheral.h"

/* Main Menu Object */
tbmMenuObj_t tpMenuMain;
tbmMenuObj_t tpMenuSelectConn;
tbmMenuObj_t tpMenuPerConn;
tbmMenuObj_t tpMenuConnPhy;

/*
 * Menu Lists Initializations
 */

void Throughput_buildMenu(void)
{
// Menu: Main
// upper: none
MENU_OBJ(tpMenuMain, NULL, 1, NULL)
MENU_ITEM_SUBMENU(tpMenuMain, 0, &tpMenuSelectConn)
MENU_OBJ_END

// Menu: SelectDev
// upper: Main
// NOTE: The number of items in this menu object shall be
//       equal to or greater than MAX_NUM_BLE_CONNS
MENU_OBJ(tpMenuSelectConn, "Work with", 1, &tpMenuMain)
MENU_ITEM_ACTION(tpMenuSelectConn, 0, NULL, ThroughputPeripheral_doSelectConn)
MENU_OBJ_END

// Menu: PerConnection
// upper: SelectDevice
MENU_OBJ(tpMenuPerConn, NULL, 2, &tpMenuSelectConn)
MENU_ITEM_SUBMENU(tpMenuPerConn, 0, &tpMenuConnPhy)
MENU_ITEM_ACTION(tpMenuPerConn, 1, "Toggle Throughput",   ThroughputPeripheral_doToggleThroughput)
MENU_OBJ_END

// Menu: ConnPhy
// upper: Select Device
MENU_OBJ(tpMenuConnPhy, "Set Conn PHY Preference", 4, &tpMenuPerConn)
MENU_ITEM_ACTION(tpMenuConnPhy, 0, "1 Mbps",              ThroughputPeripheral_doSetConnPhy)
MENU_ITEM_ACTION(tpMenuConnPhy, 1, "2 Mbps",              ThroughputPeripheral_doSetConnPhy)
MENU_ITEM_ACTION(tpMenuConnPhy, 2, "Coded:S2",            ThroughputPeripheral_doSetConnPhy)
MENU_ITEM_ACTION(tpMenuConnPhy, 3, "Coded:S8",            ThroughputPeripheral_doSetConnPhy)
MENU_OBJ_END
}
