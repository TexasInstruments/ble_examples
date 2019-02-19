
Purpose / Scope
===============

This purpose of this example is to demonstrate a simple serial socket using
TI's Simple Stream Server profile. The example showcase an simple UART over
BLE implementation, the project is designed to allow the actual data sink and
source to be easily exchanged for the purpose of creating a generic simple
stream application.

Prerequisites
=============

#### Hardware Requirements

Before running the demo, the user will need the following components:

- 2x [CC13x2 Launchpad](http://www.ti.com/tool/LAUNCHXL-CC1352R1)
- 2x Micro USB cable

#### Firmware Requirements

- [CC13x2 / CC26x2 SDK](http://www.ti.com/tool/SIMPLELINK-CC13X2-26X2-SDK)

Simple Stream Server service
============================

The simple stream service is made to implement a bi-directional data stream
connection over the BLE protocol. The service uses a 128 bit UUID:
F000C0C0-0451-4000-B000-00000000-0000. simple stream server service contains
two characteristics, they are listed below.

| Characteristic    | UUID                                      |
|:-----------------:|:-----------------------------------------:|
|DataIn             | F000C0C1-0451-4000-B000-00000000-0000     |
|DataOut            | F000C0C2-0451-4000-B000-00000000-0000     |

The `DataIn` characteristic implies data coming from client to server while
`DataOut` is data outgoing from server to client.

For more information on the service, see the Simple Serial Socket Server example readme.

Simple Stream Client profile
============================

The simple stream client profile provides an easy to use interface, similar to
that of the simple stream server service, for interaction with a connected
Simple Stream Server peripheral. Following a connection, the service table
provided by the profile has to be populated with service handles before any
data can be sent from the client to the server.

The service table can be easily populated by utilizing the simple service
discovery API available in the example. To see how this API is used during the
discovery process, refer to the `SimpleSerialSocketClient_processGATTDiscEvent`
function found inside `simple_serial_socket_client`.

The profile provided APIs:
* SimpleStreamClient_sendData(uint16_t* data, uint16_t len)
* SimpleStreamClient_processStream()
* SimpleStreamClient_disconnectStream()
* SimpleStreamClient_enableNotifications(uint16_t connHandle)

The Simple Serial Client keeps an internal FIFO list for data sent to the
profile using `SimpleStreamClient_sendData()`. When new data is sent to the
profile, the internal queue will be processed and as much data as possible
will be sent to the server using writes commands (no-respond).

> Note: The Simple Serial Client profile relies on dynamic memory allocated and
> will use the application ICall heap to store the queued data.

If the profile is unable to send all data inside the current connection event,
`SimpleStreamClient_processStream()` can be called in application space to
process the internal queue again at another time.

If the client disconnects from the server, the internal queue can be flushed
by calling `SimpleStreamClient_disconnectStream()`.

To receive incoming data, the client has to enable notifications. To do this,
the `SimpleStreamClient_enableNotifications()` API can be used to enable
notifications given the connection handle.

Running the Demo
================

Running the demo is as simple and compiling and loading the code, then hooking
up to your PC to send a receive data over UART. Please see the steps below:

1. Compile and load the code
 - Build LaunchPad #1 with the `simple_serial_socket_client`
 cc13x2r1lp_stack-FlashROM_Library project
 - Build and load LaunchPad #1 with the `simple_serial_socket_client`
 cc13x2r1lp_app-FlashROM_StackLibrary project
 - Build LaunchPad #2 with the `simple_serial_socket_server`
 cc13x2r1lp_stack-FlashROM_Library project
 - Build and load LaunchPad #2 with the `simple_serial_socket_server`
 cc13x2r1lp_app-FlashROM_StackLibrary project

2. Connect the Boards to the PC terminal
 - You can use the terminal to send data from your PC to the LaunchPad, and
 also display the info sent from one device to another.
 - You will need to open two instances of the terminal program to, one to
 communicate with each board.
 - Follow the steps from our [FAQ](faq.md) to connect to the LaunchPad boards
 - **Please note that the Simple Serial Socket project uses the default baud
 rate of 115200**

3. Power the boards individually and verify they are initialized
 - The client will auto connect to the server by looking for an advertisement
 containing the SSS service together with a
   matching manufacturer ID. The manufacturer ID can be changed in both
   projects to allow for multiple devices to run
   alongside each other.
 - Upon connecting the client and server will light the red LED. When
 notifications has been enabled and the stream is ready
   for use, the green LED will also be lit.
 - At this point you can type into either terminal window and watch it being
 echoed to the other terminal via BLE.

Improving UART performance for large data transfers
===================================================

The default board file configures the size of the internal UART ring buffer to
32 bytes. If the amount of data sent over UART to the device exceeds
UART_MAX_READ_SIZE plus the size of the ring buffer, data loss is probable.
This is due to the internal UART ring buffer wrapping before the application
has the chance to process the previous UART buffer. To be able to handle
larger chunks of data, the size of the ring buffer can be increased inside the
board file.

In order to modify the size of the board file, the `board.c` file found inside
the `Startup`  folder need to be replaced with copies of the actual board
files. This can be done by performing the following steps:

- Remove the `board.c` file found inside the `Startup` folder.
- Copy the following files from `<SDK DIR>/source/ti/blestack/boards/CC1352R1_LAUNCHXL` directory into the projects `Startup` folder:
    - Board.h
    - CC1352R1_LAUNCHXL.h
    - CC1352R1_LAUNCHXL.c
    - CC1352R1_LAUNCHXL_fxns.c

The ring buffer size can now be changed by resizing the uartCC26XXRingBuffer
array found inside `CC1352R1_LAUNCHXL.c`.

It is also possible to modify the shared `CC1352R1_LAUNCHXL.c` file that
`board.c` imports directly instead of coping the board files into the project.
This is however not recommended as changes will propagate to all BLE examples
that depend on the shared file.
