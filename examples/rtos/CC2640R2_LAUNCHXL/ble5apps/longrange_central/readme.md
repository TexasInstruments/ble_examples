# BLE5-Stack Long Range Central

## Functional Overview

This example page will demonstrate the Bluetooth&trade; 5 Long Range
capabilities of the Texas Instruments CC2640R2 SDK's BLE5-Stack.

This project is a modified Simple Central sample application from the
BLE5-Stack component of the SDK.

This list contains the changes from the standard Simple Central example:
- Removed the Simple GATT Profile
- Added Support for a custom profile called Throughput Profile
- Added Support for a custom profile called Temperature Profile
   - Based on the [Sensor Tag's Temperature profile](http://processors.wiki.ti.com/index.php/CC2650_SensorTag_User's_Guide#IR_Temperature_Sensors)

The Long Range Central only connects to peripherals that support the Throughput
and Temperature profiles. To connect, the Central and Peripheral devices must be
within the 1M PHY range. Once connected, the Central takes the follow steps:

1.  The discovery of the Throughput and Temperature Characteristics
1.  A GATT write to the Throughput PHY Characteristic to have the peripheral
    start negotiation the use of the Coded PHY (S8).
1.  Update the LED status indicating that the Central is connected to the
    Peripheral.
1.  Enabling GATT Notifications for the Temperature Data Characteristic

## Project Hardware
- 1 CC2650R2 Launchpad
- 1 UART Terminal on PC

### LED Output

- RED: Solid RED indicates the device is not connected to a peripheral.
- GREEN: Blinking GREEN indicates the device is connected to a peripheral in
  using the Long Range Coded PHY (S8).

### Two Button Menu

The Two Button Menu system is designed to utilize the two push buttons available
on the CC2640R2 Launchpad. The left button (BTN1) is always used to cycle
between options. The right button (BTN2) is used to activate the selected action
or item.

The Two Button Menu is set up by default to utilize the back channel UART
present on the Launchpad as it's display.

### Running the Demo

1.  Compile and load the projects:
    - First build and load `ble5_longrange_central_cc2640r2lp_stack`
    - Next build and load `ble5_longrange_central_cc2640r2lp_app`

1.  Connect to the Launchpad via PuTTY (or other serial console emulator). For
    instructions on connecting via PuTTY, please see our [FAQ page](faq.md).

1.  After connecting to PuTTY, you will be presented with the Two Button Menu
    system prompting you for actions. Select 'Scan/Connect Menu' by pressing the
    right button (BTN2).

    ```
    *Texas Instruments Bluetooth 5 Long Range Demo

    +Scan/Connect Menu >

    This Device's BDADDR : 0x<BDADDR>
    ```

1.  Then select "Scan For Devices" menu option by pressing the right button (BTN2).

    ```
    *Scan/Connect Menu
    < Next Item
      Scan for Devices >
     +Upper Menu
    This Device's BDADDR : 0x<BDADDR>
    ```

1.  After Scanning is performed, choose the desired device to connect to by
    pressing the right button (BTN2). Note: Only peripherals that support both
    the Throughput and Temperature profiles will be listed.

    ```
    *Scan/Connect Menu
    < Next Item
      Select Next Device >
      Connect to Selected Device
      Scan for Devices
     +Upper Menu
    2 Devices Found

    Scanned Device 1
    0x<BDADDR>
    ```

1.  Press the left button (BTN1) to move to the next action, "Connect to
    Selected Device". Then use the right button (BTN2) to begin connecting to
    the displayed device.

    After a connection is successful, you will be presented with new menu
    options:

    ```
    *Texas Instruments Bluetooth 5 Long Range Demo
    < Next Item
      Toggle RSSI Readings
      Disconnect
    PHY Update Complete
    ```

    As well as connection details in the UART display:

    ```
    Peer Device : 0x<BDADDR>
    Current PHY: Coded:S8
    RSSI -dBm: 65
    Object Temperature: 22 (C)
    ```
