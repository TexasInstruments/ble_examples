Extra examples for TI BLE-Stack SDK
===================================

This repository contains additional sample applications and components for the Texas Instruments *Bluetooth&reg;* Low Energy software development kit. All Github sample apps are compatible with  the latest BLE-Stack SDK.

To use the examples and tools in this repository, please [download and install the SDK](http://www.ti.com/ble-stack) first, and if necessary [buy an evaluation kit](https://store.ti.com/Search.aspx?k=CC2650).

For other hardware and software resources, [please visit our wiki](http://www.ti.com/ble-wiki).

**Note:** Not all additional sample projects are migrated to GitHub - please refer to the wiki for a complete list.

Installation
============

This repository can be cloned and tracked using Git. For instructions on how to clone a repository from Github please refer to this guide: [Clone from Github](https://help.github.com/articles/cloning-a-repository/)

For users who are unfamiliar with Git, there is the option of downloading the contents of the repository as a zip file. See instructions below.

    - Click the green "Clone or download" button
    - Select "Download ZIP" option
    - Zip folder will appear in your Downloads folder

This repository can be cloned/download anywhere on your computer. There is a dependency between this repository and the BLE SDK install location.

By default the BLE SDK will install to:

        C:\ti\simplelink\ble_sdk_2_02_00_31

If the BLE SDK must be installed to a different location, then the examples in this repo will need to have the following environment variable updated

        TI_BLE_SDK_BASE

Follow your IDE specific instructions for changing environment variables or simply do a global search and replace within the ble\_examples folder.

By default, TI\_BLE\_SDK\_BASE points to C:\ti\simplelink\ble\_sdk\_2\_02\_00\_31. If you installed the SDK there, no change is required.

Examples List
=============

The following examples are currently supported:

 * beacon\_rfdriver
 * hid\_emu\_kbd
 * multi\_role
 * security\_examples\_central
 * security\_examples\_peripheral
 * spp\_ble\_client
 * spp\_ble\_server
 * throughput\_example\_central
 * throughput\_example\_peripheral/
