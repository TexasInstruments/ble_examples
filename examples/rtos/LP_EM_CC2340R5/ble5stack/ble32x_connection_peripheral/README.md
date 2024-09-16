### Bluetooth Low Energy 32 connection Demo using CC2340R5 and CC1354P10 ###

Purpose / Scope
===============

The purpose of this demo is to demonstrate that the CC2340R5 + CC1354 devices can be used to implement a high capacity (up to 32 connections) network. In this demo, there are 32 CC2340R5 operating as Bluetooth LE peripherals connected to a single CC1354 device. Once established, the connections are stable. The demo then showcases data transmission between the central and peripheral (both ways). The central performs GATT writes on each peripheral and each peripheral sends a notification back. As each peripheral receives and transmit data, the LEDs blink to show that the network is live and communicating. 

Prerequisites
=============

### Hardware Requirements ###

Before running the demo, the user will need the following components:

- Up to 32 [LP-EM-CC2340R5 Launchpad](https://www.ti.com/tool/LP-EM-CC2340R5)
- Up to 33 [LP-XDS110 Debugger](https://www.ti.com/tool/LP-XDS110ET)
- Up to 33 USB type C to USB A cable (included with LP-XDS110ET debugger)
- 1x [LP-EM-CC1354P10 Launchpad](https://www.ti.com/tool/LP-EM-CC1354P10)
- Port USB Hub

### Firmware Requirements ###
Bluetooth LE Peripheral: 

- SDK for the CC2340R5: [SimpleLink Low Power F3 SDK (7.40.00.64)](https://www.ti.com/tool/download/SIMPLELINK-LOWPOWER-F3-SDK)

- SysConfig v1.18.1
- RTOS: FreeRTOS v 202104.00 
- Compiler: TI Clang v2.1.3.LTS

### Tools Requirements ###

- Code Composer Studio: v12.4 or newer

### BLE Peripheral Parameters Default Settings ###
PHY: 1M
Tx Power: 0 dBm
Connection Interval (min/max): 100/130 msec
Max Number of PDUs: 5
Max Size of PDU: 255 bytes
Extended Stack Settings: Guard Time

Running the Demo
================

### Steps ###

![Hardware Photo](<resources/Hardware Test Platform.png>)

1. Obtain the necessary hardware.
2. Connect the board/s to the PC terminal
3. Build and load the launchpads: 
    - Peripheral LP CC2340R5 with the `ble32x_connection_peripheral` project
    - Central LP CC1354P10-1 with the `ble32x_connection_central` project
        *   A pre-compiled binary for the peripheral project has been included with
            this repo. This binary is called "ble32_connection_central_app_LP_EM_CC1354P10_1_tirtos_ticlang.out"
            and may be found in the resources folder.
        *   The source code of the `ble32x_connection_central` project may be
            found in the [simplelink_cc13xx_26xx_sdk-7.10 branch](https://github.com/TexasInstruments/ble_examples/tree/simplelink_cc13xx_26xx_sdk-7.10) of this repo.
4. Open Serial Terminal on CCS (or preferred Terminal Application) for the Central Device 
    - Press Ctrl+Alt+Shift+T on CCS and a window should open
    - Make sure the following parameters are configured:
        - A. In ‘Choose Terminal’ select Serial Terminal
        - Serial Port: Your device’s UART COM Port (can be found on Device Manager which COM Port)
        - Baud Rate: 115200
        - Data Size: 8
        - Parity: None
        - Stop bits: 1
        - Encoding: Default (ISO-8859-1)
5. Run the program, by either using the UART console window or by push button commands

The text-based and push button commands are available in the Central Demo Readme.md file. 

- When the peripherals are disconnected from the cental:
![Peripherals Disconnected](resources/Peripherals_Dis.png)

- When the peripherals are connected to the central:
![Peripherals Connected](resources/Peripherals_Con.png)
    - The LEDs should be toggling every 500ms 