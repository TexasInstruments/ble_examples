
Purpose / Scope
===============

This sample project implements the HID-Over-GATT profile on the CC2650 LaunchPad
The project aims to show how a HID keyboard can be emulated using a simple two button device.

The example is based on adopted profile and service specifications for HID-Over-GATT and Scan
Parameters. Also included are the Device Information Service and Battery Service.

# Interface

When connected, pressing Button 1 and Button 2 will send the "left arrow" and
"right arrow" HID reports, respectively.

A secure connection must be established before key presses is sent to the peer
device.

Running the Demo
================

1. Compile and load the projects:
    - First build and load `cc2650lp_stack`
    - Next build and load `cc2650lp_app`

2. The device will start advertising out of the box, the device name is `'H','I','D',' ','K','e','y','b','o','a','r','d'`

3. Connect to the HID emulated keyboard using a central device that supports the HID-Over-GATT profile. Common examples are:
    - Android mobile devices
    - iOS mobile devices

4. Initiate a device discovery and connection procedure using your phone's settings menu. Pairing/bonding is enabled so you will be prompted to pair on the first time.

5. Open an application that supports keyboard input. (for example internet browser) Notice that the cursor moves to the left or right based on the left and right keypresses on the launchpad.