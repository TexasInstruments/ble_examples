Extra examples for TI BLE-Stack SDK v2.1.x
=========================================

This repository contains additional sample applications and components for the Texas Instruments *Bluetooth&reg;* Low Energy software development kit. All Github sample apps are compatible with BLE-Stack SDK v2.1.1, to use this version simply update the IDE environment variables as described in the Cloning and running section.

To use the examples and tools in this repository, please [download and install the SDK](http://www.ti.com/ble-stack) first, and if necessary [buy an evaluation kit](https://store.ti.com/Search.aspx?k=CC2650).

For other hardware and software resources, [please visit our wiki](http://www.ti.com/ble-wiki).

**Note:** Not all additional sample projects are migrated to GitHub - please refer to the wiki for a complete list.

## Sample Applications

The following Demos can be found on this repository, which is formatted to mirror the BLE-Stack SDK for similarity. Note that some demos may require multiple projects

#### SimpleAP + SimpleNP: Network Processor Interface: UART Software Handshaking
This example shows how you can avoid RTS/CTS flow control lines and still use the power saving features of the device, by using the RX/TX lines to wake up the chip and do a software handshake with an external MCU or computer. For this demo TI's SimpleNP and SimpleAP projects are used.
* [SimpleNP projects and docs](Projects/ble/simple_np_sw_handshaking)
* [SimpleAP projects and docs](Projects/ble/simple_ap_sw_handshaking)
* [SAPLib projects and docs](Projects/ble/sap_lib_sw_handshaking)
* [Modified NPI component files](Components/npi/unified)

#### MultiRole (Link Layer Topology)
This example shows how you can connect as a peripheral and central at the same time to multiple devices.
* [MultiRole projects and docs](Projects/ble/multi_role)
* [Multi GAP Role component](Projects/ble/Profiles/Roles/CC26xx)

#### SimpleBLEPeripheral: Redirect LCD to UART
This example shows how to use the TI-RTOS Log system, how to print Log statements on UART, and how to redirect the `LCD_WRITE_STRING*` macros used by many of the sample apps to UART via Log statements.
* [Modified SimpleBLEPeripheral project and docs](Projects/ble/simple_ble_peripheral_uartdisplay)
* [Log-to-UART component for TI-RTOS LoggerCallback](Components/uart_log)

#### SimpleBLEPeripheral: Porting a project to TI-RTOS 2.15
This example shows how you can port a BLE project to TI-RTOS 2.15. As an example, SimpleBLEPeripheral is ported. The commit log could be instructive for checking what was done exactly.
* [SimpleBLEPeripheral ported to TI-RTOS 2.15 and instructions](Projects/ble/simple_ble_peripheral_tirtos_2p15)

## CC2650 LaunchPad Board files
BLE-Stack SDK 2.1.x compatible version of the TI-RTOS board files for the CC2650 Launchpad can be found here. Addional porting information is available on the CC2650 LaunchPad User's Guide for Bluetooth Smart.
* [CC2650 LaunchPad User's Guide for Bluetooth Smart](http://processors.wiki.ti.com/index.php/CC2650_LaunchPad_User%27s_Guide_for_Bluetooth_Smart)
* [Board files folder](Util/tirtos_2_13_patches/ti/boards)

## Cloning and running
That the example projects are made to reference a BLE-Stack SDK installation at `c:\ti\simplelink\ble_cc26xx_2_01_00_44423`. The repository can be cloned to any location. If using BLE-Stack SDKv2.1.1, be sure to change this value to `c:\ti\simplelink\ble_cc26xx_2_01_01_44627`.

If you have installed the BLE-Stack SDK at another location, you will need to modify the IDE's project files.

#### IAR Embedded Workbench SDK path change
Open the `\*.custom_argvars` file associated with the workspace you want to run, and change the path for `TI_BLE_SDK_BASE` to reflect your path.

#### Code Composer Studio SDK path change
Open the `.project` file associated with the projects you want to run, and change the path for `TI_BLE_SDK_BASE` in the `<variableList>` at the bottom of the file to reflect your path.
