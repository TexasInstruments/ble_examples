
Purpose / Scope
===============

This purpose of this example is to demonstrate a UART to BLE bridge using TI's
proprietary Serial Port Profile (SPP). The project is based on the
[UART to Bluetooth® Low Energy (BLE) Bridge Reference Design](http://www.ti.com/tool/TIDC-SPPBLE-SW-RD),
and has been enhanced to include Bluetooth 5.0 features such as LE 2M PHY and
LE Coded PHY.

**Note: This example will be deprecated as of the 2.20 based SDKs. It will
be replaced with the Simple Serial Socket**

Prerequisites
=============

#### Hardware Requirements

Before running the demo, the user will need the following components:

- 2x [CC26x2R LaunchPad](http://www.ti.com/tool/launchxl-cc26x2r1)
- 2x Micro USB cable

#### Firmware Requirements

- [BLE5-Stack](http://www.ti.com/tool/SIMPLELINK-CC26X2-SDK)

Serial Port Service (SPS)
=========================

The serial port service is made to implement a bi-directional UART connection over the
BLE protocol. The service uses a 128 bit UUID: F000C0E0-0451-4000-B000-00000000-0000.
SPS contains three characteristics, they are listed below.

| Characteristic    | UUID                                      |
|:-----------------:|:-----------------------------------------:|
|Data               | F000C0E1-0451-4000-B000-00000000-0000     |
|Status             | F000C0E2-0451-4000-B000-00000000-0000     |
|Config             | F000C0E3-0451-4000-B000-00000000-0000     |

For more information about the Serial Port Profile (SPP), please see the
[TI-Design Guide](http://www.ti.com/tool/TIDC-SPPBLE-SW-RD) or the
[SPS Spec Document](http://www.ti.com/lit/TIDUA63).

Running the Demo
================

Running the demo is as simple and compiling and loading the code, then hooking
up to your PC to send a receive data over UART. Please follow the steps below:

1. Compile and load the code
 - Build LaunchPad #1 with the `ble5_spp_ble_client` project
 - Build LaunchPad #2 with the `ble5_spp_ble_server` project

1. Connect the Boards to the PC terminal
 - You can use the terminal to send data from your PC to the LaunchPad,
   and also display the info sent from one device to another.
 - You will need to open two instances of the terminal program,
   one to communicate with each board.
 - Follow the steps from our [FAQ](faq.md) to connect to the LaunchPad boards
 - **Please note that the SPP project uses the default baud rate of 115200**

1. Power the boards individually and verify they are initialized
 - The client will blink the green LED twice at initialization.
   It will also display `Auto connecting...` on the terminal
 - The server will blink the red LED once at initialization
 - The client will auto connect to the server using a hardcoded BD\_ADDR
 - Upon connecting the client will display:
   `Discovering services...Found Serial Port Service...Data Char Found...Notification enabled...`
 - At this point you can type into either terminal window and watch it being echoed
   to the other terminal via BLE.

In addition to the UART display interface, the project uses the following buttons:

Key         | Description
------------|---------------------------------------------------------
BTN-1       | Send an ASCII character over the air
BTN-2       | Toggle between different PHYs (1M, 2M, Coded, Auto PHY)

The following is a description of the LEDs used in the application:

LED         | Description
------------|-----------------------------------------------------
Green (DIO7)| Indicates connection status, toggles on BTN-2 press
Red (DIO6)  | Toggles on BTN-1 press

References
==========
 * [UART To BLE Bridge Wiki](http://processors.wiki.ti.com/index.php/CC2640_UART_to_BLE_Bridge)
 * [UART To BLE Bridge TI Design](http://www.ti.com/tool/TIDC-SPPBLE-SW-RD)
 * [SPS Spec Document](http://www.ti.com/lit/TIDUA63)
