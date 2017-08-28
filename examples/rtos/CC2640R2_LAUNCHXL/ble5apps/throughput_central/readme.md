# BLE5-Stack Throughput Central

## Functional Overview

This project is meant to be utilized with BLE5-Stack Throughput Peripheral to
demonstrate Bluetooth&trade; 5 features.

This project is based on the Simple Central sample application from the
BLE5-Stack component of the SDK.

BLE5-Stack Throughput Peripheral Project should be used in order to do a full
throughput demo. However, this project can function standalone to demonstrate
the Data Length Extension, 2 Megabit, or Coded PHYs when connected to a peer
device supporting these features.

This list contains the changes from the standard Simple Central example:
- Added Support for a custom profile called Throughput Profile
- Modified existing menu system to utilize the Two Button Menu
- Changed Service and Characteristic Discover Behavior
- Changed MTU Size of project

## Project Hardware
- 1 CC2650R2 Launchpad
- 1 UART Terminal on PC

### Two Button Menu

The Two Button Menu system is designed to utilize the two push buttons available
on the CC2640R2 Launchpad. The left button (BTN1) is always used to cycle
between options. The right button (BTN2) is used to activate the selected action
or item.

The Two Button Menu is set up by default to utilize the back channel UART
present on the launchpad as it's display.

### Running the Demo

1.  Compile and load the projects:
    - First build and load `ble5_throughput_central_cc2640r2lp_stack`
    - Next build and load `ble5_throughput_central_cc2640r2lp_app`

1.  Connect to the LaunchPad via PuTTY (or other serial console emulator). For
    instructions on connecting via PuTTY, please see our [FAQ page](faq.md).

1.  After connecting to PuTTY, you will be presented with the Two Button Menu
    system prompting you for actions. Select 'Scan/Connect Menu' by pressing the
    right button (BTN2).

    ```
    *Texas Instruments Bluetooth 5 Demo

    +Scan/Connect Menu >
    ```

1.  Then select "Scan For Devices" menu option by pressing the right button (BTN2).

    ```
    *Scan/Connect Menu
    < Next Item
      Scan for Devices >
     +Upper Menu
    ```

1.  After Scanning is Performed, choose the desired device to connect to by pressing
    the right button (BTN2). Note: If the peer device supports the Throughput Profile,
    below the peer device's address will contain the text:
    ``-- Throughput Profile Supported --``

    ```
    *Scan/Connect Menu
    < Next Item
      Select Next Device >
      Connect to Selected Device
      Scan for Devices
     +Upper Menu
    2 Devices Found

    Scanned Device 1
    0x98072DAA4E5E
    -- Throughput Profile Supported --
    ```

1.  Press the left button (BTN1) to move to the next action, "Connect to
    Selected Device". Then use the right button (BTN2) to begin connecting to
    the displayed device.

    After a connection is successful, you will be presented with new menu
    options:

    ```
    *Texas Instruments Bluetooth 5 Demo
    < Next Item
     +Set PHY >
     +Set PDU
      Toggle RSSI Readings
      Disconnect
    ```

    As well as connection details in the UART display:

    ```
    Peer Device : 0x98072DAA4E5E
    PHY: 1 Mbps
    Instant Rate (kb/s): 247.904
    Average Rate (kb/s): 264.104 over 10 Samples
    RSSI -dBm: 44
    ```

    As shown, if the Throughput Profile is supported on the peer, throughput information will be displayed.

1.  As desired, BLE5-Stack Throughput Central can modify the PHY and DLE PDU
    size of the connection. Use left button (BTN1) to change selected action,
    and right button (BTN2) to invoke the selected action. The display will
    update if connection PDU or PHY is changed.

    ```
    Peer Device : 0x98072DAA4E5E
    Current PHY: 2 Mbps
    Instant Rate (kb/s): 1366.400
    Average Rate (kb/s): 1366.400 over 10 Samples
    RSSI -dBm: 32

    This Device's BDADDR : 0x00124B005220
    Device GAP Role: Central
    Device RX PDU Size: 251B
    MTU Size: 247B
    ```

    If Throughput Profile is supported on the peer, the displayed throughput
    will update in real time.
