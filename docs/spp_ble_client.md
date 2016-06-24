
Purpose / Scope
===============

This sample project is used to connect to proprietary Serial Port profile in a Bluetooth low
energy peripheral device to provide an example of serial BLE bridge application. The project is based on
simple_central. This project is configured to run on the CC2650 Launchpad. 

# Interface

Connect CC2650 LaunchPad's USB port to terminal application on the PC such as TeraTerm using 921600 8N1 serial port setup.

A connection must be established before data is received from the peer
device.

Operation
==========

1. Power up the device
   The device starts auto connect to server device by default using hardcoded address.

1. Use spp_ble_client to connect to spp_ble_server, GREEN LED should be ON when connected for both boards.
   A smartphone app can also act as the client.
   
1. Typing in one terminal should show up on the peer terminal and RED LED should toggle for every packet sent. 
