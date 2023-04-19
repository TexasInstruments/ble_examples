
Purpose / Scope
===============

This purpose of this example is to demonstrate a simple serial socket using
TI's Simple Stream Server profile. The example showcases a simple UART over
BLE implementation. The project is designed to allow the actual data sink and
source to be easily exchanged for the purpose of creating a generic simple
stream application.

Prerequisites
=============

#### Hardware Requirements

Before running the demo, the user will need the following components:

- 2x [CC26x2 Launchpad](http://www.ti.com/tool/LAUNCHXL-CC26X2R1)
- 2x Micro USB cable

#### Firmware Requirements

- [CC13xx / CC26xx SDK (6.40.00.13)](http://www.ti.com/tool/SIMPLELINK-CC13XX-CC26XX-SDK)

Simple Stream Server service
============================

The simple stream service is made to implement a bi-directional data stream
connection over the BLE protocol. The service uses a 128 bit UUID:
F000C0C0-0451-4000-B000-00000000-0000. The service contains two
characteristics, they are listed below.

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

The simple stream server service keeps an internal FIFO list for data sent to
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
 - Build and load LaunchPad #1 with the `simple_serial_socket_client` project
 - Build and load LaunchPad #2 with the `simple_serial_socket_server` project

2. Connect the boards to the PC terminal
 - You can use the terminal to send data from your PC to the LaunchPad, and
 also display the info sent from one device to another.
 - You will need to open two instances of the terminal program, one to
 communicate with each board.
 - Follow the steps from our [FAQ](faq.md) to connect to the LaunchPad boards
 - **Please note that the Simple Serial Server project uses the default baud
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

The UUIDs of the simple stream server service can be changed to enable "UART
mode" support for third-party smart phone applications such as the Adafruit
Bluefruit LE connect.

To enable this support, the UUIDs found inside the
`PROFILES/simple_stream_profile_server.c` file has to be changed according to
the table below:

| Service / Characteristic | UUID                                      |
|:------------------------:|:-----------------------------------------:|
|SimpleStreamServer        | 6E400001-B5A3-F393-E0A9-E50E24DC-CA9E     |
|DataIn                    | 6E400002-B5A3-F393-E0A9-E50E24DC-CA9E     |
|DataOut                   | 6E400003-B5A3-F393-E0A9-E50E24DC-CA9E     |

When changing the UUID array, the UUID has to be written with little-endian.
The resulting UUIDs should be as shown below:

```
// SimpleStreamServer Service UUID
CONST uint8_t SimpleStreamServerUUID[ATT_UUID_SIZE] =
{
 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0X93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

// DataIn UUID
CONST uint8_t SimpleStreamServer_DataInUUID[ATT_UUID_SIZE] =
{
 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0X93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
};

// DataOut UUID
CONST uint8_t SimpleStreamServer_DataOutUUID[ATT_UUID_SIZE] =
{
 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0X93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
};
```
Expected Throughput
===================
With default configurations there is an expected throughput of around 75Kbps.
The set Parameters are:

* Max PDU Size: 255
* Max number of PDUs: 15
* Conection Interval: 7.5ms
* Baudrate: 115200

 These parameters except baudrate can be changed in the syscfg file under the BLE Tab. To change baurate update the Simple_serial_socket_server.c file.