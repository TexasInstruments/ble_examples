
Purpose / Scope
===============

This sample project implements the HID-Over-GATT profile in a Bluetooth low
energy peripheral device to provide an example of how a HID keyboard can be
emulated with a simple button remote control device. The project is based on
adopted profile and service specifications for HID-Over-GATT and Scan
Parameters. The project also includes the Device Information Service and
Battery Service. This project is configured to run on the CC2650 Launchpad. 

# Interface

When connected, pressing Button 1 and Button 2 will send the "left arrow" and
"right arrow" keys, respectively.

A secure connection must be established before key presses is sent to the peer
device.

Operation
==========

1. Power up the device
   The device starts advertising by default.

1. Initiate a device discovery and connection procedure to discover and connect
   to the HID device from a HID Host peer device.
   An example in doing this is can be done by pairing this example with an
   android device. By pressing Buttons 1 or 2, you can see different items being
   selected on the "home" screen.
