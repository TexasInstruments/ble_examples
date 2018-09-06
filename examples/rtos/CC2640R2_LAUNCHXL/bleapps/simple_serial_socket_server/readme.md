
Purpose / Scope
===============

This purpose of this example is to demonstrate a simple serial socket using
TI's Simple Stream Server service. The example showcase an simple UART over
BLE implementation, the project is designed to allow the actual data sink and
source to be easily exchanged for the purpose of creating a generic simple
stream application.

Prerequisites
=============

#### Hardware Requirements

Before running the demo, the user will need the following components:

- 2x [CC2640R2 LaunchPad](http://www.ti.com/tool/launchxl-cc2640r2)
- 2x Micro USB cable

#### Firmware Requirements

- [CC2640R2 SDK](http://www.ti.com/tool/SIMPLELINK-CC2640R2-SDK)

Simple Stream Server service (SSS)
=========================

The simple stream service is made to implement a bi-directional data stream
connection over the BLE protocol. The service uses a 128 bit UUID:
F000C0C0-0451-4000-B000-00000000-0000. SSS contains two characteristics, they
are listed below.

| Characteristic    | UUID                                      |
|:-----------------:|:-----------------------------------------:|
|DataIn             | F000C0C1-0451-4000-B000-00000000-0000     |
|DataOut            | F000C0C2-0451-4000-B000-00000000-0000     |

The `DataIn` characteristic implies  data coming from client to server while
`DataOut` is data outgoing from server to client.

The service provide APIs for sending and processing the data stream:
* SimpleStreamServer_sendData(uint16_t* data, uint16_t len)
* SimpleStreamServer_processStream()
* SimpleStreamServer_disconnectStream()

The Simple Serial Server service keeps an internal FIFO list for data sent to
the service using `SimpleStreamServer_sendData()`. When new data is sent to the
service, the internal queue will be processed and as much data as possible
will be sent to the server using notifications (this assumes the client has
enabled notifications).

> Note: The Simple Serial Server service relies on dynamic memory allocated and
> will use the application ICall heap to store the queued data.

If the service is unable to send all data inside the current connection event,
`SimpleStreamServer_processStream()` can be called in application space to
process the internal queue again at another time.

If the server disconnects from the client, the internal queue can be flushed
by calling `SimpleStreamServer_disconnectStream()`.

In order to receive data from the client, callbacks has to be registered
after adding the service. This is done by calling
`SimpleStreamServer_RegisterAppCBs( SimpleStreamServerCBs_t *appCallbacks )`

Running the Demo
================

Running the demo is as simple and compiling and loading the code, then hooking
up to your PC to send a receive data over UART. Please see the steps below:

1. Compile and load the code
 - Build LaunchPad #1 with the `simple_serial_socket_client`
 cc2640r2lp_stack-FlashROM_Library project
 - Build and load LaunchPad #1 with the `simple_serial_socket_client`
 cc2640r2lp_app-FlashROM_StackLibrary project
 - Build LaunchPad #2 with the `simple_serial_socket_server`
 cc2640r2lp_stack-FlashROM_Library project
 - Build and load LaunchPad #2 with the `simple_serial_socket_server`
 cc2640r2lp_app-FlashROM_StackLibrary project

2. Connect the Boards to the PC terminal
 - You can use the terminal to send data from your PC to the LaunchPad, and
 also display the info sent from one device to another.
 - You will need to open two instances of the terminal program to, one to
 communicate with each board.
 - Follow the steps from our [FAQ](faq.md) to connect to the LaunchPad boards
 - **Please note that the Simple Serial Bridge project uses the default baud
 rate of 115200**

3. Power the boards individually  and verify they are initialized
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
