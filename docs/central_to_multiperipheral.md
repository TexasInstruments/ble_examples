
Purpose / Scope
===============

This purpose of this example is to demonstrate connecting multiple peripherals with a central application.
The project is based on the simple_central project and has been enhanced to include connections up to 3 peripherals.

Prerequisites
=============

#### Hardware Requirements

Before running the demo, the user will need the following components:

- 1x [CC2650 DK](https://store.ti.com/cc2650dk.aspx)
- 3x Any CC2650 kits such as [CC2650 LaunchPad](http://www.ti.com/tool/launchxl-cc2650)

#### Firmware Requirements

- [BLE-Stack v2.2.1](http://www.ti.com/ble-stack)


Running the Demo
================

Running the demo is as simple and compiling and loading the code, interacting with buttons on the CC2650DK board. Please see the steps below:

1. Compile and load the code
 - Build and load CC2650DK #1 with the `central_to_multiperipheral` stack project
 - Build and load CC2650DK #1 with the `central_to_multiperipheral` app project
 - Similarly, load the other boards with simple_peripheral project

2. Connect CC2650DK #1 with three other boards
 - Press UP button to start discovery
 - Press LEFT button to browse through the discovered BLE devices with addresses
 - Press SELECT to connect, verify it is connected with message on the LCD display
 - Repeat until up to 3 devices are connected **Note that discovery step can be skipped if the device to be connected was already found in the first discovery**

3. Send some test data to multiple simple_peripheral devices
 - Browse through the connected devices with the RIGHT button
 - Press the DOWN button to read/write to Characteristic 1 of the currently selected connected device
 - Repeat for each of the connected devices
 - To disconnect, press the RIGHT key to select from connected device(s) then press the SELECT key.