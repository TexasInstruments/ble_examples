Purpose
===============

This example project configures the Simple BLE Central for the CC2650 LaunchPad. The simple central is a Bluetooth low energy central device with GATT client functionality. To accommodate the CC2650 LaunchPad, two buttons are used to navigate through the menu.

By default, the simple central application is configured to filter and connect to peripheral devices with the TI Simple Profile Service UUID, such as Simple Peripheral. To modify this behavior, set DEFAULT_DEV_DISC_BY_SVC_UUID to FALSE in simple_central.c.

The application can be used with a LCD screen, or a terminal program such as PuTTY can be used to display over UART.

Requirements
=============

#### Hardware

- [CC2650 LaunchPad](http://www.ti.com/tool/launchxl-cc2650)
- **OPTIONAL:** [Sharp LCD BoosterPack](http://www.ti.com/tool/430boost-sharp96)
- Device to connect to

##### Software
- simple_central_lp project from this GIT page
- [BLE-Stack v2.2.1](http://www.ti.com/ble-stack)
- **OPTIONAL:** Terminal program such as [PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html)
- CCS 6.2.x **or** IAR 7.70.x


User Interface
================

#### Choosing Display

By default, the project will use a UART display. To use LCD to display (instead of UART), go to the project Predefines (Properties -> Build -> ARM Compiler -> Predefined Symbols) and delete the `BOARD_DISPLAY_EXCLUDE_LCD` entry. Then add `BOARD_DISPLAY_EXCLUDE_UART`.

For instructions on using PuTTY with the CC2650 LaunchPad, please see the [FAQ page](faq.md).

#### Running the Project

Upon powering the device, press the right button (BTN-2) to scan for devices. The central will display both address and name of the devices found. To browse through the found devices press the left button (BTN-1). To scan for devices, press the right button. When browsing through devices, press the right button to connect.

When connected, the bottom row displays the possible actions. Press left to toggle which option is selected, and right to execute.
- Param upd req: Send a parameter update request, toggling between the "initial" and "default" set of connection parameters. (By default, this toggles between a connection interval of 400 and 200 ms.)
- Start/stop RSSI poll: Start or cancel RSSI polling.
- Read/write req: Send a read or write request for the Simple Service characteristic 1.
- Disconnect.

When disconnected, press right to scan for devices.
