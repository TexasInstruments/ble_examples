Extra examples for TI BLE-Stack SDK
===================================

This repository contains additional sample applications and components for the Texas Instruments *Bluetooth&reg;* Low Energy software development kit. 

To use the examples and tools in this repository, please [download and install the SDK](http://www.ti.com/ble-stack) first, and if necessary [buy an evaluation kit](http://www.ti.com/tool/launchxl-cc2640r2).

For other hardware and software resources, [please visit our wiki](http://www.ti.com/ble-wiki). If you have any questions please refer to the [FAQ page](docs/faq.md).

**Note:** Not all additional sample projects are migrated to GitHub - please refer to the Examples List below.

Installation
============

This repository can be cloned and tracked using Git. For instructions on how to clone a repository from Github please refer to this guide: [Clone from Github](https://help.github.com/articles/cloning-a-repository/)

For users who are unfamiliar with Git, there is the option of downloading the contents of the repository as a zip file. See instructions below.

    - Click the green "Clone or download" button
    - Select "Download ZIP" option
    - Zip folder will appear in your Downloads folder

&nbsp;&nbsp;&nbsp;&nbsp;![Download Github zip](docs/doc_resources/download_zip_github.png)

This repository can be cloned/download anywhere on your computer. There is a dependency between this repository and the BLE SDK install location.

By default the BLE SDK will install to:

        C:\ti\simplelink_cc2640r2_sdk_1_00_00_xx

If the BLE SDK must be installed to a different location, then the examples in this repo will need to have the following environment variable updated

        TI_BLE_SDK_BASE

Follow your IDE specific instructions for changing environment variables or simply do a global search and replace within the ble\_examples folder. See the [FAQ page](docs/faq.md) for more details.

By default, TI\_BLE\_SDK\_BASE points to `C:\ti\simplelink_cc2640r2_sdk_1_00_00_xx`. If you installed the SDK to the default location, no change is required.

Required Tools
==============

Similar to the BLE-Stack SDK, the examples in this repository support the IAR and CCS toolchains. Please pay careful attention to versions of these tools, the supported version are listed below. Using a non supported version is untested and may result in unexpected behavior.

 - IAR for ARM v7.80.1
 - CCS v7.0.0 with TI ARM Compiler v16.09

For more information on toolchain setup, please refer to our [FAQ page](docs/faq.md).

FAQ
===

The [FAQ page](docs/faq.md) will try to address some of the common questions related to the ble_examples repo.

Examples List
=============

The following examples are currently supported:

### project_zero
&nbsp;&nbsp;&nbsp;&nbsp;Quickly connect to smartphone to toggle LEDs, receive
button press notifications, and display messages over UART.
&nbsp;&nbsp;&nbsp;&nbsp;

[docs](docs/project_zero.md) | [project files](examples/rtos/CC2640R2_LAUNCHXL/blestack/cc2640r2_project_zero/tirtos/ccs) | [src](examples/rtos/CC2640R2_LAUNCHXL/blestack/cc2640r2_project_zero/src)

References
==========

The following reference pages may be helpful during general BLE development. New users of the CC26xx platform and BLE are encouraged to read the TI BLE Software Developer's Guide (located in the docs/blestack directory of your SDK install ) and complete the [SimpleLink Academy](http://software-dl.ti.com/lprf/simplelink_academy/overview.html) training.

Other resources can be found below:

* [BLE wiki](http://www.ti.com/ble-wiki)
* [BLE E2E Page](www.ti.com/ble-forum)