#include <bcomdef.h>
#include <ti/display/Display.h>

#if defined(BLE_V50_FEATURES) && (BLE_V50_FEATURES & PHY_2MBPS_CFG) && !defined(Display_DISABLE_ALL)
#include <menu/two_btn_menu.h>
#include "longrange_central_menu.h"
#include "longrange_central.h"

/*
 * Menu Lists Initializations
 */

/* Menu: Main
     3 submenus, no actions, no upper */
MENU_OBJ(sbcMenuMain, NULL, 3, NULL)
  MENU_ITEM_SUBMENU(&sbcMenuScanandConnect)
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

#endif  // PHY_2MBPS_CFG && !Display_DISABLE_ALL

