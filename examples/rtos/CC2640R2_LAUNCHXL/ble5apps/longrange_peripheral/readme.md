# BLE5-Stack Long Range Peripheral

## Functional Overview

This example page will demonstrate the Bluetooth&trade; 5 Long Range
capabilities of the Texas Instruments CC2640R2 SDK's BLE5-Stack.

This project is a modified Simple Peripheral sample application from the
BLE5-Stack component of the SDK.

This list contains the changes from the standard Simple Peripheral example:
- Removed the Simple GATT Profile
- Added Support for a custom profile called Throughput Profile
- Added Support for a custom profile called Temperature Profile
   - Based on the [Sensor Tag's Temperature profile](http://processors.wiki.ti.com/index.php/CC2650_SensorTag_User's_Guide#IR_Temperature_Sensors)

The Long Range Peripheral is intended to be a head-less device that can used
with a USB-powered battery pack to evaluate range capabilities. All the control
functions are handled by the Long Range Central device. Once connected, the
Central instructs the Peripheral to negotiate the use of the Coded PHY at which
the device will update the LED status and start gather temperature data.
Similarly, when the Central enables notifications on the Temperature Profile's
Data Characteristic, the peripheral will send the temperature data to the
central at a 1s interval. Once the connection is dropped or terminated, the
peripheral must be moved within the range of a 1M PHY.

## Project Hardware
- 1 CC2650R2 Launchpad

### LED Output

- RED: Solid RED indicates the device is not connected to a central.
- GREEN: Blinking GREEN indicates the device is connected to a central in
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
    - First build and load `ble5_longrange_peripheral_cc2640r2lp_stack`
    - Next build and load `ble5_longrange_peripheral_cc2640r2lp_app`
