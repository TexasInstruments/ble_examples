Bluetooth Low Energy 32 connection Demo using CC2340R5 and CC1354P10

Purpose / Scope
===============

The purpose of this demo is to demonstrate that the CC2340R5 + CC1354 devices can be used to implement a high capacity (32 connections) network. In this demo, there are 32 CC2340R5 operating as Bluetooth LE peripherals connected to a single CC1354 device. Once established, the connections are stable. The demo then showcases data transmission between the central and peripheral (both ways). The central performs GATT writes on each peripheral and each peripheral sends a notification back. As each peripheral receives and transmit data, the LEDs blink to show that the network is live and communicating. 

Prerequisites
=============

#### Hardware Requirements

Before running the demo, the user will need the following components:

- 32x [LP-EM-CC2340R5 Launchpad](https://www.ti.com/tool/LP-EM-CC2340R5)
- 33x [LP-XDS110 Debugger](https://www.ti.com/tool/LP-XDS110ET)
- 33x USB type C to USB A cable (included with LP-XDS110ET debugger)
- 1x [LP-EM-CC1354P10 Launchpad] (https://www.ti.com/tool/LP-EM-CC1354P10)
- 2x 16 Port USB Hub

#### Firmware Requirements

- [SimpleLink Low Power F3 SDK (7.40.00.64)](https://www.ti.com/tool/download/SIMPLELINK-LOWPOWER-F3-SDK)
- [SimpleLink Low Power F2 SDK (7.10.00.98)](https://www.ti.com/tool/download/SIMPLELINK-LOWPOWER-F2-SDK)
- SysConfig v1.16.2
- RTOS: FreeRTOS v 202104.00
- Compiler: TI Clang v2.1.3.LTS

### Tools Requirements ###

- Windows Visual Studio Professional 2013: v12.0.21005.1 REL
- Code Composer Studio: v12.4

### BLE Central Parameters ###

PHY: 1M
Tx Power: 5 dBm
Connection Interval (min/max): 200 msec
Slave Latency: 0
Connection Timeout: 6 seconds
Scan Interval: 1 second
Scan Window: 1 second
Scan Duration: 2 seconds
Max Number of Connections: 32
Max Number of PDUs: 255
Max Size of PDU: 255 bytes
Extended Stack Settings: Guard Time
PHY: 1M
Tx Power: 0 dBm
Connection Interval (min/max): 100/130 msec
Max Number of PDUs: 5
Max Size of PDU: 255 bytes
Extended Stack Settings: Guard Time

[Hardware Photo](<Hardware Test Platform.png>)

Running the Demo
================



### User Guide Commands ###

-[Text-based Console command](<Text Commands.png>)


