/******************************************************************************

@file  app_menu.c

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2023, Texas Instruments Incorporated
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


//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>
#include "ti_ble_config.h"

#if !defined(Display_DISABLE_ALL)
//*****************************************************************************
//! Prototypes
//*****************************************************************************
// Scanning callbacks
void Menu_scanningCB(uint8 index);
void Menu_scanStartCB(uint8 index);
void Menu_scanStopCB(uint8 index);
// Connection callbacks
void Menu_connectionCB(uint8 index);
void Menu_connectCB(uint8 index);
void Menu_connectToDeviceCB(uint8 index);
void Menu_workWithCB(uint8 index);
void Menu_selectedDeviceCB(uint8 index);
void Menu_connPhyCB(uint8 index);
void Menu_connPhyChangeCB(uint8 index);
void Menu_paramUpdateCB(uint8 index);
void Menu_disconnectCB(uint8 index);
//gatt client callbacks
void Menu_GattReadCB(uint8 index);
void Menu_doGattReadCB(uint8 index);
void Menu_GattWriteCB(uint8 index);
void Menu_GattWriteValueCB(uint8 index);
void Menu_doGattWriteCB(uint8 index);
void Menu_GattExchangeMTUCB(uint8 index);
void Menu_GattExchangeMTUValueCB(uint8 index);
void Menu_doRssiRead(uint8 index);
void Menu_doEnableNotification(uint8 index);
void Menu_doDisableNotification(uint8 index);
//*****************************************************************************
//! Globals
//*****************************************************************************
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
// The current connection handle the menu is working with
static uint16 menuCurrentConnHandle;
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )

#if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )
// Scan Menu
const MenuModule_Menu_t scanningMenu[] =
{
 {"Scan", &Menu_scanStartCB, "Scan for devices"},
 {"Stop Scan", &Menu_scanStopCB, "Stop Scanning for devices"}
};

MENU_MODULE_MENU_OBJECT("Scanning Menu", scanningMenu);
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG ) )

#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
// Connection Menu
const MenuModule_Menu_t connectionMenu[] =
{
#if ( HOST_CONFIG & ( CENTRAL_CFG ) )
 {"Connect", &Menu_connectCB, "Connect to a device"},
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG ) )
 {"Work with", &Menu_workWithCB, "Work with a peer device"}
};

MENU_MODULE_MENU_OBJECT("Connection Menu", connectionMenu);

// Work with menu
const MenuModule_Menu_t workWithMenu[] =
{
 {"Change conn phy", &Menu_connPhyCB, "1M, Coded or 2M"},
 {"Param update", &Menu_paramUpdateCB, "Send connection param update req"},
 {"Disconnect", &Menu_disconnectCB, "Disconnect a specific connection"},
 {"GATTread", &Menu_GattReadCB, "Read a specific connection's characteristic"},
 {"GATTwrite", &Menu_GattWriteCB, "Write a specific connection's characteristic"},
 {"Change max ATT_MTU", &Menu_GattExchangeMTUValueCB, "set the ATT_MTU to the maximum possible value that can be supported by both devices"},
 {"Start RSSI Reading", &Menu_doRssiRead, "Start reading RSSI value for a specific connection"},
 {"EnableNotification", &Menu_doEnableNotification, "Enable Notification for a specific connection"},
 {"DisableNotification", &Menu_doDisableNotification, "Disable Notification for a specific connection"}
};

MENU_MODULE_MENU_OBJECT("Work with Menu", workWithMenu);

// Phy selection menu
const MenuModule_Menu_t connPhyMenu[] =
{
 {"1 Mbps", &Menu_connPhyChangeCB, ""},
 {"2 Mbps", &Menu_connPhyChangeCB, ""},
 {"1 & 2 Mbps", &Menu_connPhyChangeCB, ""},
 {"Coded", &Menu_connPhyChangeCB, ""},
 {"1 & 2 Mbps & Coded", &Menu_connPhyChangeCB, ""},
};

MENU_MODULE_MENU_OBJECT("Set Conn PHY Preference", connPhyMenu);

// readCharacteristic selection menu
const MenuModule_Menu_t readCharacteristicMenu[] =
{
 {"Characteristic 1", &Menu_doGattReadCB, ""},
 {"Characteristic 2", &Menu_doGattReadCB, ""},
 {"Characteristic 5", &Menu_doGattReadCB, ""},
};

MENU_MODULE_MENU_OBJECT("Read the Characteristic", readCharacteristicMenu);

// writeCharacteristic selection menu
const MenuModule_Menu_t writeCharacteristicMenu[] =
{
 {"Characteristic 1", &Menu_GattWriteValueCB, ""},
 {"Characteristic 3", &Menu_GattWriteValueCB, ""},
};

MENU_MODULE_MENU_OBJECT("Write the Characteristic", writeCharacteristicMenu);

// writeValueCharacteristic selection menu
const MenuModule_Menu_t writeValueCharacteristicMenu[] =
{
 {"0x00", &Menu_doGattWriteCB, ""},
 {"0x02", &Menu_doGattWriteCB, ""},
 {"0x55", &Menu_doGattWriteCB, ""},
 {"0xFF", &Menu_doGattWriteCB, ""},
};

MENU_MODULE_MENU_OBJECT("Write the Characteristic", writeValueCharacteristicMenu);
attWriteReq_t req;
uint8_t charVal = 0;

// ATT_MTU value selection menu
const MenuModule_Menu_t GattExchangeMTUValueMenu[] =
{
 {"251", &Menu_GattExchangeMTUCB, ""},
 {"100", &Menu_GattExchangeMTUCB, ""},
 {"60", &Menu_GattExchangeMTUCB, ""},
 {"25", &Menu_GattExchangeMTUCB, ""},
};

MENU_MODULE_MENU_OBJECT("Write the Characteristic", GattExchangeMTUValueMenu);
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )

// Main menu
#if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG | PERIPHERAL_CFG ) )
const MenuModule_Menu_t mainMenu[] =
{
#if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG) )
 {"Scanning", &Menu_scanningCB, "Scan menu"},
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
 {"Connection", &Menu_connectionCB, "Connection menu"},
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
};

MENU_MODULE_MENU_OBJECT("Basic BLE GATT Client Menu", mainMenu);
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG | PERIPHERAL_CFG ) )

//*****************************************************************************
//! Functions
//*****************************************************************************

#if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )
/*********************************************************************
 * @fn      Menu_scanningCB
 *
 * @brief   A callback that will be called once the Scanning item in
 *          the main menu is selected.
 *          Calls MenuModule_startSubMenu to display the scanning menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_scanningCB(uint8 index)
{
  MenuModule_startSubMenu(&scanningMenuObject);
}

/*********************************************************************
 * @fn      Menu_scanStartCB
 *
 * @brief   A callback that will be called once the scan item in
 *          the scanningMenu is selected.
 *          Sets the parameters needed for a scan and starts the scan.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_scanStartCB(uint8 index)
{
    bStatus_t status;
    const BLEAppUtil_ScanStart_t centralScanStartParams =
    {
        /*! Zero for continuously scanning */
        .scanPeriod     = DEFAULT_SCAN_PERIOD, /* Units of 1.28sec */

        /*! Scan Duration shall be greater than to scan interval,*/
        /*! Zero continuously scanning. */
        .scanDuration   = DEFAULT_SCAN_DURATION, /* Units of 10ms */

        /*! If non-zero, the list of advertising reports will be */
        /*! generated and come with @ref GAP_EVT_SCAN_DISABLED.  */
        .maxNumReport   = APP_MAX_NUM_OF_ADV_REPORTS
    };

    status = BLEAppUtil_scanStart(&centralScanStartParams);

    // Print the status of the scan
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: ScanStart = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);
}

/*********************************************************************
 * @fn      Menu_scanStopCB
 *
 * @brief   A callback that will be called once the stop scan item in
 *          the scanningMenu is selected.
 *          Calls BLEAppUtil_scanStop and display the returned status.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_scanStopCB(uint8 index)
{
    bStatus_t status;

    status = BLEAppUtil_scanStop();

    // Print the status of the scan
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: ScanStop = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);
}
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )

#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
/*********************************************************************
 * @fn      Menu_connectionCB
 *
 * @brief   A callback that will be called once the connection item in
 *          the main menu is selected.
 *          Calls MenuModule_startSubMenu to display the connection menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_connectionCB(uint8 index)
{
  MenuModule_startSubMenu(&connectionMenuObject);
}

#if ( HOST_CONFIG & ( CENTRAL_CFG ) )
/*********************************************************************
 * @fn      Menu_connectCB
 *
 * @brief   A callback that will be called once the connect item in
 *          the connectionMenu is selected.
 *          Gets the list of scanned devices and calls
 *          MenuModule_printStringList to display the list of addresses.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_connectCB(uint8 index)
{
    uint8 i = 0;
    // Create a static list for the devices addresses strings
    static char addressList[APP_MAX_NUM_OF_ADV_REPORTS][BLEAPPUTIL_ADDR_STR_SIZE] = {0};
    // Create a static menu that will contain the addresses
    static MenuModule_Menu_t peerAddr[APP_MAX_NUM_OF_ADV_REPORTS];

    // Get the scan results list
    App_scanResults *menuScanRes;
    uint8 size = Scan_getScanResList(&menuScanRes);

    // If the scan result list is empty, pring a msg
    if(size == 0)
    {
        MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: No devices in scan list");
    }
    // If the list is not empty, copy the addresses and fill up the menu
    else
    {
        for(i = 0; i < size; i++)
        {
            // Convert the addresses to strings
            memcpy(addressList[i], BLEAppUtil_convertBdAddr2Str(menuScanRes[i].address), BLEAPPUTIL_ADDR_STR_SIZE);
            peerAddr[i].itemName = addressList[i];
            peerAddr[i].itemCallback = &Menu_connectToDeviceCB;
            peerAddr[i].itemHelp = "";
        }
        // Create the menu object
        MENU_MODULE_MENU_OBJECT("Addresses List", peerAddr);
        // Display the list
        MenuModule_printStringList(&peerAddrObject, size);
    }
}

/*********************************************************************
 * @fn      Menu_connectToDeviceCB
 *
 * @brief   A callback that will be called once a device address in
 *          the scan results list is selected.
 *          Gets the list of scanned devices and connect to the address
 *          in the provided index.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_connectToDeviceCB(uint8 index)
{
    bStatus_t status;

    // Get the scan results list
    App_scanResults *menuScanRes;
    uint8 size = Scan_getScanResList(&menuScanRes);

    // Set the connection parameters
    BLEAppUtil_ConnectParams_t connParams =
    {
     .peerAddrType = menuScanRes[index].addressType,
     .phys = INIT_PHY_1M,
     .timeout = 1000
    };

    // Copy the selected address
    memcpy(connParams.pPeerAddress, menuScanRes[index].address, B_ADDR_LEN);
    status = BLEAppUtil_connect(&connParams);

    // Print the status of the connect call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: Connect = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);

    // Go back to the last menu
    MenuModule_goBack();
}
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG ) )

/*********************************************************************
 * @fn      Menu_workWithCB
 *
 * @brief   A callback that will be called once the work with item in
 *          the connectionMenu is selected.
 *          Gets the list of connected devices and calls
 *          MenuModule_printStringList to display the list of addresses.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_workWithCB(uint8 index)
{
    uint8 i;
    uint8 numConns = linkDB_NumActive();

    // If the connection list is empty, pring a msg
    if(numConns == 0)
    {
        MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: No connected devices");
    }
    // If the list is not empty, copy the addresses and fill up the menu
    else
    {
        // Create a static list for the devices addresses strings
        static char connAddrsses[MAX_NUM_BLE_CONNS][BLEAPPUTIL_ADDR_STR_SIZE] = {0};
        // Create a static menu that will contain the addresses
        static MenuModule_Menu_t connAddrList[MAX_NUM_BLE_CONNS];
        // Get the list of connected devices
        App_connInfo * currConnList = Connection_getConnList();
        for(i = 0; i < numConns; i++)
        {
            // Convert the addresses to strings
            memcpy(connAddrsses[i], BLEAppUtil_convertBdAddr2Str(currConnList[i].peerAddress), BLEAPPUTIL_ADDR_STR_SIZE);
            connAddrList[i].itemName = connAddrsses[i];
            connAddrList[i].itemCallback = &Menu_selectedDeviceCB;
            connAddrList[i].itemHelp = "";
        }

        // Create the menu object
        MENU_MODULE_MENU_OBJECT("Connected devices List", connAddrList);
        // Display the list
        MenuModule_printStringList(&connAddrListObject, numConns);
    }
}

/*********************************************************************
 * @fn      Menu_selectedDeviceCB
 *
 * @brief   A callback that will be called once a device address in
 *          the connected devices list is selected.
 *          Go back to the last menu and display the work with menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_selectedDeviceCB(uint8 index)
{
    menuCurrentConnHandle = Connection_getConnhandle(index);
    // Go to the last menu
    MenuModule_goBack();
    // Display the work with menu options
    MenuModule_startSubMenu(&workWithMenuObject);
}

/*********************************************************************
 * @fn      Menu_connPhyCB
 *
 * @brief   A callback that will be called once the Change conn phy item
 *          in the workWithMenu is selected.
 *          Display the connPhy options menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_connPhyCB(uint8 index)
{
  MenuModule_startSubMenu(&connPhyMenuObject);
}

/*********************************************************************
 * @fn      Menu_connPhyChangeCB
 *
 * @brief   A callback that will be called once a phy item in the
 *          connPhyMenu is selected.
 *          Calls BLEAppUtil_setConnPhy to set the selected phy and
 *          go back to the work with menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_connPhyChangeCB(uint8 index)
{
    bStatus_t status;
    static uint8_t phy[] = {
      HCI_PHY_1_MBPS,
      HCI_PHY_2_MBPS,
      HCI_PHY_1_MBPS | HCI_PHY_2_MBPS,
      HCI_PHY_CODED,
      HCI_PHY_1_MBPS | HCI_PHY_2_MBPS | HCI_PHY_CODED
    };

    BLEAppUtil_ConnPhyParams_t phyParams =
    {
     .connHandle = menuCurrentConnHandle,
     .allPhys = 0,
     .txPhy = phy[index],
     .rxPhy = phy[index],
     .phyOpts = 0
    };

    // Set the connection phy selected in the menu
    status = BLEAppUtil_setConnPhy(&phyParams);

    // Print the status of the set conn phy call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: SetConnPhy = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);

    // Go back to the "work with" menu
    MenuModule_goBack();
}

/*********************************************************************
 * @fn      Menu_paramUpdateCB
 *
 * @brief   A callback that will be called once the Param update item
 *          in the workWithMenu is selected.
 *          Calls BLEAppUtil_paramUpdateReq to send the parameters
 *          update request.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_paramUpdateCB(uint8 index)
{
    bStatus_t status;
    gapUpdateLinkParamReq_t pParamUpdateReq =
    {
     .connectionHandle = menuCurrentConnHandle,
     .intervalMin = 400,
     .intervalMax = 800,
     .connLatency = 0,
     .connTimeout = 600
    };

    // Send a connection param update request
    status = BLEAppUtil_paramUpdateReq(&pParamUpdateReq);

    // Print the status of the param update call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: ParamUpdateReq = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);
}

/*********************************************************************
 * @fn      Menu_disconnectCB
 *
 * @brief   A callback that will be called once the Disconnect item
 *          in the workWithMenu is selected.
 *          Calls BLEAppUtil_disconnect to disconnect from the
 *          menuCurrentConnHandle.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_disconnectCB(uint8 index)
{
    bStatus_t status;

    // Disconnect from the selected connection
    status = BLEAppUtil_disconnect(menuCurrentConnHandle);

    // Print the status of the set conn phy call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: Disconnect = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);

    // Go back to the "connection" menu
    MenuModule_goBack();
}

/*********************************************************************
 * @fn      Menu_GattReadCB
 *
 * @brief   A callback that will be called once the Change read characteristic item
 *          in the workWithMenu is selected.
 *          Display the doGattReadCB options menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */

void Menu_GattReadCB(uint8 index)
{
  MenuModule_startSubMenu(&readCharacteristicMenuObject);
}

/*********************************************************************
 * @fn      Menu_doGattRead
 *
 * @brief   A callback that will be called once the GATT Read item
 *          in the workWithMenu is selected.
 *
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_doGattReadCB(uint8 index)
{
    bStatus_t status;
    attReadReq_t req;

    switch(index){
        case 0:
        {
            req.handle = 37; // handle characteristic 1
            break;
        }
        case 1:
        {
            req.handle = 40;// handle characteristic 2
            break;
        }
        case 2:
        {
            req.handle = 50;// handle characteristic 5
            break;
        }
        default:
            break;
    }


    status = GATT_ReadCharValue(menuCurrentConnHandle, &req, BLEAppUtil_getSelfEntity());

    // Print the status of the gatt_readCharValue call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: GATTRead = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);
    // Go back to the last menu
    MenuModule_goBack();
}

/*********************************************************************
 * @fn      Menu_GattWriteCB
 *
 * @brief   A callback that will be called once the Change write characteristic item
 *          in the workWithMenu is selected.
 *          Display the GattWriteValueCB options menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */

void Menu_GattWriteCB(uint8 index)
{
  MenuModule_startSubMenu(&writeCharacteristicMenuObject);
}
/*********************************************************************
 * @fn      Menu_GattWriteValueCB
 *
 * @brief   A callback that will be called once the write value characteristic item
 *          in the workWithMenu is selected.
 *          Display the doGatt-writeCB options menu.
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */

void Menu_GattWriteValueCB(uint8 index)
{
   switch(index){
       case 0:
       {
           req.handle = 37; // handle characteristic 1
           break;
       }
       case 1:
       {
           req.handle = 43;// handle characteristic 3
           break;
       }
       default:
           break;
   }

  MenuModule_startSubMenu(&writeValueCharacteristicMenuObject);
}

/*********************************************************************
 * @fn      Menu_doGattWriteCB
 *
 * @brief   A callback that will be called once the GATT write item
 *          in the workWithMenu is selected.
 *
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_doGattWriteCB(uint8 index)
{
    bStatus_t status;

    uint8_t charVals[4] = { 0x00, 0x02, 0x55, 0xFF };

    req.pValue = GATT_bm_alloc(menuCurrentConnHandle, ATT_WRITE_REQ, 1, NULL);
    req.len = 1;
    charVal = charVals[index];
    req.pValue[0] = charVal;
    req.sig = 0;
    req.cmd = 0;

    status = GATT_WriteCharValue(menuCurrentConnHandle, &req, BLEAppUtil_getSelfEntity());
    if ( status != SUCCESS )
    {
        GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }


    // Print the status of the gatt_WriteCharValue call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: GATTWrite = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);


    if ( status != SUCCESS )
        {
            GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
        }
    // Go back to the last menu
    MenuModule_goBack();
}

void Menu_GattExchangeMTUValueCB(uint8 index)
{
  MenuModule_startSubMenu(&GattExchangeMTUValueMenuObject);
}

/*********************************************************************
 * @fn      Menu_GattExchangeMTUCB
 *
 * @brief   A callback that will be called once the ATT Exchange MTU GATT item
 *          in the workWithMenu is selected.
 *
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_GattExchangeMTUCB(uint8 index){
    bStatus_t status;
    attExchangeMTUReq_t req;
    uint8_t MTUVals[4] = { 251, 100, 60, 25 };

    req.clientRxMTU = MTUVals[index]-L2CAP_HDR_SIZE;
    status = GATT_ExchangeMTU(menuCurrentConnHandle,&req,BLEAppUtil_getSelfEntity());

    // Print the status of the GATT_ExchangeMTU call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: Change max ATT_MTU = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);
    // Go to the last menu
    MenuModule_goBack();
}
/*********************************************************************
 * @fn      Menu_doRssiRead
 *
 * @brief   A callback that will be called once the GATT RssiRead item
 *          in the workWithMenu is selected.
 *
 *
 * @param   index - the index in the menu
 *
 * @return  none
 */
void Menu_doRssiRead(uint8 index){
    bStatus_t status;
    status = HCI_ReadRssiCmd(menuCurrentConnHandle);
    // Print the status of the HCI_ReadRssiCmd call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: Read RSSI = "
                          MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                          status);
}

void Menu_doEnableNotification(uint8 index){
    bStatus_t status;
    attWriteReq_t req;

    uint8 configData[2] = {0x01,0x00};
    req.pValue = GATT_bm_alloc(menuCurrentConnHandle, ATT_WRITE_REQ, 2, NULL);

    // Enable notify for outgoing data
    if (req.pValue != NULL)
    {
        req.handle = 47;
        req.len = 2;
        memcpy(req.pValue, configData, 2);
        req.cmd = TRUE;
        req.sig = FALSE;
        status = GATT_WriteNoRsp(menuCurrentConnHandle, &req);
    }
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: GATT Notification = "
                          MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                          status);
}

void Menu_doDisableNotification(uint8 index){
    bStatus_t status;
    attWriteReq_t req;

    uint8 configData[2] = {0x00,0x00};
    req.pValue = GATT_bm_alloc(menuCurrentConnHandle, ATT_WRITE_REQ, 2, NULL);

    // Enable notify for outgoing data
    if (req.pValue != NULL)
    {
        req.handle = 47;
        req.len = 2;
        memcpy(req.pValue, configData, 2);
        req.cmd = TRUE;
        req.sig = FALSE;
        status = GATT_WriteNoRsp(menuCurrentConnHandle, &req);
    }
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: GATT Notification = "
                          MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                          status);
}

#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )

#endif // #if !defined(Display_DISABLE_ALL)

/*********************************************************************
 * @fn      Menu_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize the
 *          menu
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Menu_start()
{
  bStatus_t status = SUCCESS;

#if !defined(Display_DISABLE_ALL)
  MenuModule_params_t params = {
#if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG | PERIPHERAL_CFG ) )
    .mode = MenuModule_Mode_MENU_WITH_BUTTONS
#else
    .mode = MenuModule_Mode_PRINTS_ONLY
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG | PERIPHERAL_CFG ) )
  };

#if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG | PERIPHERAL_CFG ) )
  status = MenuModule_init(&mainMenuObject, &params);
#else
  status = MenuModule_init(NULL, &params);
  if(status == SUCCESS)
  {

    // Print the application name
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0,
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_CYAN
                      "Basic BLE GATT Client" MENU_MODULE_COLOR_RESET);
  }
#endif // #if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG | PERIPHERAL_CFG ) )

#endif // #if !defined(Display_DISABLE_ALL)
  return status;
}
