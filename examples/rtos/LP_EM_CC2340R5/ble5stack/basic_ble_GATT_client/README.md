# basic_ble GATT client example 

This project implements a simple Bluetooth low energy central device with GATT client functionality. 
The basic_ble_GATT_client application can be configured to filter and connect to peripheral devices, then you'll be able to work 
with a peripheral device if a connection has been established.

So far this example, owns several functionality to work with the peripheral connected device, 
GATTRead > Read a specific characteristic value
GATTWrite > Write a specific characteristic value
GATT_ExchangeMTU > Modify the ATT_MTU max size for the connection
EnableNotification > Enable the notification of a dedicated characteristic
DisableNotification > Disable the notification of a dedicated characteristic
Start RSSI Reading > NOT SUPPORTED

## Limitations
-The example is meant to be connected to a device running basic_ble_GATT_client as a peripheral
  > Connecting to a device with a GATT table not matching the basic_ble_GATT_client GATT table may lead to unexpected behavior as the GATT handles are hardcoded
    in Menu_doGattReadCB() and Menu_GattWriteValueCB() (see inside app/app_menu.c)
-The example does not yet support RSSI read continously 
  > If this feature is needed, update the menu with a timer, displaying the peripheral's RSSI value

## Code's Structure
The majority of the example still unchanged, all the change has been implemented in the app folder. (Expect for the RSSI you probably should have add the 
HCI_READ_RSSI event to the bleapputil_api.h file)

The app_menu.c file contains all the change for the UART display menu, what's new:

## Menus added to the Work With Menu: 
- GATTread
- GATTwrite
- Change max ATT_MTU
- EnableNotification 
- DisableNotification 

### Menus added to the GATTread Menu: 
- Characteristic 1
- Characteristic 2
- Characteristic 5 (This one leads to an error as the pairing is not set by default)

### Menus added to the GATTwrite Menu: 
- Characteristic 1
- Characteristic 3

Inside each Characteristics' menus a new menu is displayed to select the value to write,
for the moment you can choose between 4 different values: 0x00, 0x02, 0x55, 0xFF

### Menus added to the Change max ATT_MTU Menu:
A new menu is displayed to select the ATT_MTU size value to change,
for the moment you can choose between 4 different values: 251, 100, 60, 25 
As these value depends also of the  Max PDU size (parameter in sysconfig) and the ATT_MTU 
value of the peripheral side, some of the value can lead to an error, improvements: 
write the reason why the function leads to an error

The app_menu.c displays also the status of each called GATT functions

The app_data.c file contains all the change to trigger the write events called by each GATT functions
and result displayed in the UART display menu, what's new:

## Comments on the code

4 new events have been added to the GATT_EventHandler triggered by the GATT_MSG_EVENT:

- ATT_READ_RSP, triggered by GATT_ReadCharValue function, displays the read value
- ATT_WRITE_RSP, triggered by GATT_WriteCharValue function, displays the sent write value
- ATT_EXCHANGE_MTU_RSP, triggered by GATT_ExchangeMTU function, displays the server and the client MTU value
- ATT_ERROR_RSP, triggered if an error is detected, displays the error Code value
- ATT_HANDLE_VALUE_NOTI, triggered if characteristic 4 has notification enable


To use the UART display menu, connect the UART display of your launchpad to a UART terminal, 
Scan for peripheral devices and connect to one of them, then go to the connection > work with menu
all change should be directly available there.















