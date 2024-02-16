
Purpose / Scope
===============

This purpose of this example is to demonstrate a UART over BLE example using
the data_stream example as a base. The project is designed to allow the
actual data sink and source to be easily exchanged for the purpose 
of creating a generic simple stream application.

Prerequisites
=============

#### Hardware Requirements

Before running the demo, the user will need the following components:

- 1x [LP-EM-CC2340R5 Launchpad](https://www.ti.com/tool/LP-EM-CC2340R5)
- 1x [LP-XDS110 Debugger](https://www.ti.com/tool/LP-XDS110ET)
- 1x USB type C to USB A cable (included with LP-XDS110ET debugger)

#### Firmware Requirements

- [SimpleLink Low Power F3 SDK (7.40.00.64)](https://www.ti.com/tool/download/SIMPLELINK-LOWPOWER-F3-SDK)

#### Mobile Application Requirements

- [SimpleLink Connect Application on iOS](https://apps.apple.com/app/simplelink-connect/id6445892658)
- [SimpleLink Connect Application on Android](https://play.google.com/store/apps/details?id=com.ti.connectivity.simplelinkconnect)
- [SimpleLink Connect Application source code](https://www.ti.com/tool/SIMPLELINK-CONNECT-SW-MOBILE-APP)


TI Terminal service
===================

The TI Terminal service is made to implement a bi-directional data stream
connection over the BLE protocol. The service uses a 128 bit UUID:
F000C0C0-0451-4000-B000-00000000-0000. The service contains two
characteristics, they are listed below.

| Characteristic    | UUID                                      |
|:-----------------:|:-----------------------------------------:|
|DataIn             | F000C0C1-0451-4000-B000-00000000-0000     |
|DataOut            | F000C0C2-0451-4000-B000-00000000-0000     |

The `DataIn` characteristic implies  data coming from client to server while
`DataOut` is data outgoing from server to client.

Running the Demo
================

Running the demo is as simple and compiling and loading the code, then hooking
up to your PC to send a receive data over UART. Please see the steps below:

1. Compile and load the code
 - Build and load LaunchPad with the `data_stream_UART_over_BLE_LP_EM_CC2340R5_freertos_ticlang` project


2. Connect the board to the PC terminal
 - You can use the terminal to send data from your PC to the LaunchPad, and
 also display the info sent from one device to another.
 - Follow the steps from our [FAQ](../../../../../docs/faq.md) to connect to the LaunchPad boards
 - **Please note that the project uses the default baud rate of 115200**. This value is set in the 
 function ``DataStream_start`` in the file ``app/Profiles/app_UART_over_data_stream.c`` 

3. Power the board

4. Connect to the `data_stream_UART_over_BLE` board with a smart phone running the SimpleLink Connect app

5. Perform Bluetooth LE scan on SimpleLink Connect App

6. Connect to `DataStream UART` device

7. Open TI Terminal service (UUID: F000C0C0-0451-4000-B000-00000000-0000)

8. Enable notifications on DataOut characteristic (UUID: F000C0C2-0451-4000-B000-00000000-0000)

At this point, any UART data sent over the PC terminal will appear in the DataOut characteristic on the phone.

Any data written to the DataIn (UUID: F000C0C1-0451-4000-B000-00000000-0000) characteristic will
be sent to the CC2340R5 device and be printed to the PC terminal.

Note: Third-party Serial BLE applications may be use to connect to the data_stream_UART_over_ble example,
but you may need to manually configure the serial BLE application to use the characteristics for input
and output.

Improving UART Performance for Large Data Transfers
===================================================

The default board file configures the size of the internal UART ring buffer to
32 bytes. If the amount of data sent over UART to the device exceeds
UART_MAX_READ_SIZE plus the size of the ring buffer, data loss is probable.
This is due to the internal UART ring buffer wrapping before the application
has the chance to process the previous UART buffer. To be able to handle
larger chunks of data, the size of the ring buffer can be increased inside the
sysconfig file in the UART2 tab under Ring Buffer Size.


Enable Adafruit Bluefruit LE Connect "UART mode" support
========================================================

The UUIDs of the TI Terminal service can be changed to enable "UART
mode" support for third-party smart phone applications such as the Adafruit
Bluefruit LE connect.

To enable this support, the UUIDs found inside the
`common/Services/Data_stream/data_stream_server.c` and `common/Services/Data_stream/data_stream_server.h` files has to be changed according to the table below:

| Service / Characteristic | UUID                                      |
|:------------------------:|:-----------------------------------------:|
|TI Terminal               | 6E400001-B5A3-F393-E0A9-E50E24DC-CA9E     |
|DataIn                    | 6E400002-B5A3-F393-E0A9-E50E24DC-CA9E     |
|DataOut                   | 6E400003-B5A3-F393-E0A9-E50E24DC-CA9E     |

Note: the `data_stream_server.c` and `data_stream_server.h` files are included as linked resources in the project by
default. Ensure to replace these with local copies to not modify your SDK files and keep your modifications contained
to your project.

To learn more about the data_stream example, please reference the [SimpleLink data_stream examples SLA](https://dev.ti.com/tirex/content/simplelink_academy_for_cc23xx_7_20_01_00/_build_simplelink_academy_for_cc23xx_7_20_01_00/source/cc2340rx_00_data_stream_examples.html).


