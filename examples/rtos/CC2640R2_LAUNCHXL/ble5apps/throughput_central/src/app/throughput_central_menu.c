#include <bcomdef.h>
#include <ti/display/Display.h>

#if defined(BLE_V50_FEATURES) && (BLE_V50_FEATURES & PHY_2MBPS_CFG) && !defined(Display_DISABLE_ALL)
#include <menu/two_btn_menu.h>
#include "throughput_central_menu.h"
#include "throughput_central.h"

/*
 * Menu Lists Initializations
 */

/* Menu: Main
     3 submenus, no actions, no upper */
MENU_OBJ(sbcMenuMain, NULL, 5, NULL)
  MENU_ITEM_SUBMENU(&sbcMenuScanandConnect)
  MENU_ITEM_SUBMENU(&sbcMenuSetPhy)
  MENU_ITEM_SUBMENU(&sbcMenuSetDLEPDU)
  MENU_ITEM_ACTION("Toggle RSSI Readings", SimpleBLECentral_doToggleRSSI)
  MENU_ITEM_ACTION("Disconnect", SimpleBLECentral_doDisconnect)
MENU_OBJ_END

/* Menu: Scanning and Connection
     3 actions, upper = sbpMenuMain */
MENU_OBJ(sbcMenuScanandConnect, "Scan/Connect Menu", 3, &sbcMenuMain)
  MENU_ITEM_ACTION("Select Next Device",                SimpleBLECentral_doScanAndConnect)
  MENU_ITEM_ACTION("Connect to Selected Device",        SimpleBLECentral_doScanAndConnect)
  MENU_ITEM_ACTION("Scan for Devices",                  SimpleBLECentral_doScanAndConnect)
MENU_OBJ_END

/* Menu: Set PHY
     5 actions, upper = sbpMenuMain */
MENU_OBJ(sbcMenuSetPhy, "Set PHY", 4, &sbcMenuMain)
  MENU_ITEM_ACTION("1 Mbps",                 SimpleBLECentral_doSetPhy)
  MENU_ITEM_ACTION("2 Mbps",                 SimpleBLECentral_doSetPhy)
  MENU_ITEM_ACTION("Coded:S2",               SimpleBLECentral_doSetPhy)
  MENU_ITEM_ACTION("Coded:S8",               SimpleBLECentral_doSetPhy)
MENU_OBJ_END

/* Menu: Set DLE PDU
     2 actions, upper = sbpMenuMain */
MENU_OBJ(sbcMenuSetDLEPDU, "Set PDU", 2, &sbcMenuMain)
  MENU_ITEM_ACTION("27 Bytes",     SimpleBLECentral_doSetDLEPDU)
  MENU_ITEM_ACTION("251 Bytes",    SimpleBLECentral_doSetDLEPDU)
MENU_OBJ_END

#endif  // PHY_2MBPS_CFG && !Display_DISABLE_ALL

