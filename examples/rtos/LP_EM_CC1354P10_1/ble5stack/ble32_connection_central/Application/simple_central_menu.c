/******************************************************************************

@file  simple_central_menu.c

Group: WCS, BTS
Target Device: cc13xx cc26xx

******************************************************************************

 Copyright (c) 2022-2024, Texas Instruments Incorporated
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

#include <bcomdef.h>
#include <ti/display/Display.h>
#include <menu/two_btn_menu.h>
#include "simple_central_menu.h"
#include "simple_central.h"
#include "ti_ble_config.h"

/* Main Menu Object */
tbmMenuObj_t scMenuMain;
tbmMenuObj_t scMenuConnect;
tbmMenuObj_t scMenuScanPhy;
tbmMenuObj_t scMenuAutoConnect;
tbmMenuObj_t scMenuConnPhy;
tbmMenuObj_t scMenuSelectConn;
tbmMenuObj_t scMenuPerConn;
tbmMenuObj_t scMenuGattWrite;

/*
 * Menu Lists Initializations
 */
void SimpleCentral_buildMenu(void)
{
// Menu: Main
// upper: none
  MENU_OBJ(scMenuMain, "Simple Central", 7, NULL)
  MENU_ITEM_ACTION(scMenuMain,0,"Discover Devices",  SimpleCentral_doDiscoverDevices)
  MENU_ITEM_ACTION(scMenuMain,1,"Stop Discovering",  SimpleCentral_doStopDiscovering)
  MENU_ITEM_SUBMENU(scMenuMain,2,&scMenuConnect)
  MENU_ITEM_ACTION(scMenuMain,3,"Update MTU Size",  SimpleCentral_doMTUupdate)
  MENU_ITEM_ACTION(scMenuMain,4,"Enable Notifications",  SimpleCentral_enableNotify)
  MENU_ITEM_ACTION(scMenuMain,5,"Start GATT Write cycle",  SimpleCentral_doStartGATT)
  MENU_ITEM_ACTION(scMenuMain,6,"Stop GATT Write cycle",  SimpleCentral_doStopGATT)




  MENU_OBJ_END


// Menu: Connect
// upper: Main
// NOTE: The number of items in this menu object shall be equal to
//       or greater than DEFAULT_MAX_SCAN_RES.
//       The number of items cannot exceed 27 which is the two-button menu's
//       constraint.
  MENU_OBJ(scMenuConnect, "Connect to", DEFAULT_MAX_SCAN_RES, &scMenuMain)
  MENU_ITEM_MULTIPLE_ACTIONS(scMenuConnect, DEFAULT_MAX_SCAN_RES, NULL, SimpleCentral_doConnect)
  MENU_OBJ_END

// Menu: SelectDev
// upper: Main
// NOTE: The number of items in this menu object shall be
//       equal to or greater than MAX_NUM_BLE_CONNS
  MENU_OBJ(scMenuSelectConn, "Work with", MAX_NUM_BLE_CONNS, &scMenuMain)
  MENU_ITEM_MULTIPLE_ACTIONS(scMenuSelectConn, MAX_NUM_BLE_CONNS, NULL, SimpleCentral_doSelectConn)
  MENU_OBJ_END

// Menu: PerConnection
// upper: SelectDevice
  MENU_OBJ(scMenuPerConn, NULL, 7, &scMenuSelectConn)
  MENU_ITEM_ACTION(scMenuPerConn,0,"GATT Read",          SimpleCentral_doGattRead)
  MENU_ITEM_SUBMENU(scMenuPerConn,1,&scMenuGattWrite)
  MENU_ITEM_ACTION(scMenuPerConn,2,"Start RSSI Reading", SimpleCentral_doRssiRead)
  MENU_ITEM_ACTION(scMenuPerConn,3,"Stop RSSI Reading",  SimpleCentral_doRssiRead)
  MENU_ITEM_ACTION(scMenuPerConn,4,"Connection Update",  SimpleCentral_doConnUpdate)
  MENU_ITEM_SUBMENU(scMenuPerConn,5,&scMenuConnPhy)
  MENU_ITEM_ACTION(scMenuPerConn,6,"Disconnect",         SimpleCentral_doDisconnect)
  MENU_OBJ_END

// Menu: GattWrite
// upper: PerConnection
  MENU_OBJ(scMenuGattWrite, "GATT Write", 4, &scMenuPerConn)
  MENU_ITEM_ACTION(scMenuGattWrite,0,"Write 0x00", SimpleCentral_doGattWrite)
  MENU_ITEM_ACTION(scMenuGattWrite,1,"Write 0x55", SimpleCentral_doGattWrite)
  MENU_ITEM_ACTION(scMenuGattWrite,2,"Write 0xAA", SimpleCentral_doGattWrite)
  MENU_ITEM_ACTION(scMenuGattWrite,3,"Write 0xFF", SimpleCentral_doGattWrite)
  MENU_OBJ_END

// Menu: ConnPhy
// upper: Main
  MENU_OBJ(scMenuConnPhy, "Set Conn PHY Preference", 5, &scMenuMain)
  MENU_ITEM_ACTION(scMenuConnPhy,0,"1 Mbps",              SimpleCentral_doSetConnPhy)
  MENU_ITEM_ACTION(scMenuConnPhy,1,"2 Mbps",              SimpleCentral_doSetConnPhy)
  MENU_ITEM_ACTION(scMenuConnPhy,2,"1 & 2 Mbps",          SimpleCentral_doSetConnPhy)
  MENU_ITEM_ACTION(scMenuConnPhy,3,"Coded",               SimpleCentral_doSetConnPhy)
  MENU_ITEM_ACTION(scMenuConnPhy,4,"1 & 2 Mbps, & Coded", SimpleCentral_doSetConnPhy)
  MENU_OBJ_END
}
