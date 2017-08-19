#include <bcomdef.h>
#include <ti/display/Display.h>

#if defined(BLE_V50_FEATURES) && (BLE_V50_FEATURES & PHY_2MBPS_CFG) && !defined(Display_DISABLE_ALL)
#include <menu/two_btn_menu.h>
#include "throughput_peripheral_menu.h"
#include "throughput_peripheral.h"

/*
 * Menu Lists Initializations
 */

/* Menu: Main
     2 submenus, 1 action, no upper */
MENU_OBJ(sbpMenuMain, NULL, 3, NULL)
  MENU_ITEM_SUBMENU(&sbpMenuSetPhy)
  MENU_ITEM_SUBMENU(&sbpMenuSetDLEPDU)
  MENU_ITEM_ACTION("Toggle Throughput Demo", SimpleBLEPeripheral_doThroughputDemo)
  MENU_ITEM_ACTION("Disconnect from Peer", SimpleBLEPeripheral_doThroughputDemo)
MENU_OBJ_END

/* Menu: Set PHY
     5 actions, upper = sbpMenuMain */
MENU_OBJ(sbpMenuSetPhy, "Set PHY", 4, &sbpMenuMain)
  MENU_ITEM_ACTION("1 Mbps",                 SimpleBLEPeripheral_doSetPhy)
  MENU_ITEM_ACTION("2 Mbps",                 SimpleBLEPeripheral_doSetPhy)
  MENU_ITEM_ACTION("Coded:S2",               SimpleBLEPeripheral_doSetPhy)
  MENU_ITEM_ACTION("Coded:S8",               SimpleBLEPeripheral_doSetPhy)
MENU_OBJ_END

/* Menu: Set DLE PDU
     2 actions, upper = sbpMenuMain */
MENU_OBJ(sbpMenuSetDLEPDU, "Set PDU", 2, &sbpMenuMain)
  MENU_ITEM_ACTION("27 Bytes",     SimpleBLEPeripheral_doSetDLEPDU)
  MENU_ITEM_ACTION("251 Bytes",    SimpleBLEPeripheral_doSetDLEPDU)
MENU_OBJ_END

#endif  // PHY_2MBPS_CFG && !Display_DISABLE_ALL

