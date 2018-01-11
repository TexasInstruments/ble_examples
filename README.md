## Table of Contents

* [Navigating the Repository](#navigating-the-repository)
* [Change Log](#change-log)
* [Installation](#installation)
* [Required Tools](#required-tools)
* [Examples List](#examples--demo-list)
* [Tools](#tools)
* [References](#references)

# Introduction

These examples and demos are for **TI SimpleLink CC2640R2 SDK 1.50.00.58**

This repository contains *Bluetooth&reg;* 4.2 Low Energy sample applications for
Texas Instruments' SimpleLink CC2640R2 SDK. These examples have not been
validated as production-ready.

**Do not** use GitHub's bug tracking feature for support. For inquiries, see the
[Bluetooth&reg; low energy Forum](https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538).

To use the examples and tools in this repository, please download and install
the [SimpleLink CC2640R2 SDK](http://www.ti.com/ble-stack) **first**, and if
necessary [buy an evaluation kit](http://www.ti.com/tool/launchxl-cc2640r2).

For other hardware and software resources,
[please visit our wiki](http://www.ti.com/ble-wiki). If you have any questions
please refer to the [FAQ page](docs/faq.md).

For extra examples for **TI BLE-Stack 2.2.x SDK** (CC26x0R1), see
[Branch: ble_examples-2.2](https://github.com/ti-simplelink/ble_examples/tree/ble_examples-2.2).

# A Note about CC2640R2 SDK 1.50

The CC2640R2 SDK 1.50 is unique in that it's BLE-Stack and BLE5-Stack component
are split into two separate installers, they are both available at the
[CC2640R2 SDK download page](http://www.ti.com/tool/SIMPLELINK-CC2640R2-SDK).

The BLE5-Stack component is now available via My Secure Software, and is
evaulation only. The evaluation SDK now contains native support for Long Range
mode. The evaluation mode sdk is `simplelink_cc2640r2_sdk_1_50_00_71`, this
release is not supported by the Github examples.

The BLE-Stack component installer is `simplelink_cc2640r2_sdk_1_50_00_58`.
This release is fully supported by Github. Since this installer does not support
the BLE5-Stack, the `ble5apps/` has been removed.

Customers wishing to access BLE5-Stack examples should refer to the my secure
software release of the SDK or the SimpleLink CC2640R2 SDK 1.40 release on
Github. This past release is accessible via the `simplelink_sdk-1.40` branch.
See below for more details on how to check out this branch.


# Navigating the Repository

The examples provided on this GitHub page serve as a plugin to a corresponding
BLE-Stack SDK release. The master branch will always point to the latest release.

Older releases can be accessed by checking out/downloading their corresponding
branch. For more information on supported examples
please consult the readme.md of the desired branch/release.

For for more information about different SDK components, please consult the
stack's User Guide.

## Change Log

Note: The version numbers below are related to GitHub ble_examples releases.
The numbering scheme is in the form of M.mm.pp.bb. The fields pp.bb are incremented
as GitHub examples are released, M.mm will map a GitHub release to a SimpleLink
SDK release.

### 1.50.00.00
Initial offering of Simplelink CC2640R2 SDK 1.50.00.58 examples.
* BLE-Stack:  Serial Port Profile (SPP) Server/Client
* BLE-Stack:  Bidirectional Audio Demo (Central/Peripheral)
* BLE-Stack:  Apple Notification Center Service (ANCS) Demo (Peripheral)
* Tools:      voice.py script for Voice-over-HOGP


## Installation

This repository can be cloned and tracked using Git. For instructions on how to
clone a repository from Github please refer to this guide:
[Clone from Github](https://help.github.com/articles/cloning-a-repository/)

For users who are unfamiliar with Git, there is the option of downloading the
contents of the repository as a zip file. See instructions below.

1. Click the green "Clone or download" button
1. Select "Download ZIP" option
1. Zip folder will appear in your Downloads folder

This repository can be cloned/download anywhere on your computer. There is a
dependency between this repository and the SimpleLink CC2640R2 SDK install
location.

By default the SimpleLink CC2640R2 SDK will install to:

    C:\ti\simplelink_cc2640r2_sdk_1_50_00_58

If the Simplelink CC2640R2 SDK must be installed to a different location, then
see the [FAQ page](docs/faq.md) for IDE specific instructions for changing
environment variables.

## Required Tools

Similar to the Simplelink CC2640R2 SDK, the examples in this repository support
the CCS and IAR toolchains. Please pay careful attention to versions of these
tools, the supported version are listed below. Using a non supported version is
untested and may result in unexpected behavior.

* CCS v7.3.0 with TI ARM Compiler v16.09.03 LTS
* IAR for ARM v8.11.2

For more information on toolchain setup, please refer to our
[FAQ page](docs/faq.md).

## FAQ

The [FAQ page](docs/faq.md) will try to address some of the common questions
related to the ble_examples repo.

## Examples / Demo List

### Full Duplex Bidirectional Audio Demo

Encode and transmit a full duplex bidirectional audio stream over BLE using two
CC2640R2 LaunchPads with CC3200AUDBOOST.

* central\_bidirectional\_audio
    * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/bleapps/central_bidirectional_audio/readme.md)
    * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/central_bidirectional_audio/tirtos/ccs)
    * [Source](examples/rtos/CC2640R2_LAUNCHXL/bleapps/central_bidirectional_audio/src)

* peripheral\_bidirectional\_audio
    * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/bleapps/peripheral_bidirectional_audio/readme.md)
    * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/peripheral_bidirectional_audio/tirtos/ccs)
    * [Source](examples/rtos/CC2640R2_LAUNCHXL/bleapps/peripheral_bidirectional_audio/src)

### Serial Port Profile (SPP) BLE Examples

This is a 2 part example where a spp\_ble\_client and spp\_ble\_server connect
with one another and exchange data over UART.

The example also supports Bluetooth 4.2 demonstrating the BLE-Stack's Data Length
Extension with OAD support:

* spp\_ble\_client
    * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/bleapps/spp_ble_client/readme.md)
    * [IAR Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/spp_ble_client/tirtos/iar)
    * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/spp_ble_client/tirtos/ccs)
    * [Source](examples/rtos/CC2640R2_LAUNCHXL/bleapps/spp_ble_client/src)
* spp\_ble\_server
    * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/bleapps/spp_ble_server/readme.md)
    * [IAR Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/spp_ble_server/tirtos/iar)
    * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/spp_ble_server/tirtos/ccs)
    * [Source](examples/rtos/CC2640R2_LAUNCHXL/bleapps/spp_ble_server/src)

### Simple Peripheral Observer BLE Example

This sample project is used to add observer role to a BLE peripheral device to
show scanning functionality in a peripheral based project.

The example supports Bluetooth 4.2:

* simple\_peripheral\_observer
    * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_peripheral_observer/readme.md)
    * [IAR Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_peripheral_observer/tirtos/iar)
    * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_peripheral_observer/tirtos/ccs)
    * [Source](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_peripheral_observer/src)


### Apple Notification Center Service (ANCS) Demo

This is an example that demonstrates the use and functionality of the ANCS. ANCS is a GATT
service present on iOS devices used to retrieve and interact with iOS notifications.

The example supports Bluetooth 4.2:

* ancs
    * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/bleapps/ancs/readme.md)
    * [IAR Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/ancs/tirtos/iar)
    * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/ancs/tirtos/ccs)
    * [Source](examples/rtos/CC2640R2_LAUNCHXL/bleapps/ancs/src)

## Tools

### [Voice.py](tools/scripts/voice/voice.py)
This is script allows developers to evaluate the Voice over
HID-over-GATT-Profile (HoGP) profile which is demonstrated as part of the
`voice_hogp_remote` example. The `voie_hogp_remote` is available as part of the
[Simplelink CC2640R2 SDK BLE Example Pack](http://www.ti.com/tool/download/SIMPLELINK-CC2640R2-SDK-BLE-EXAMPLE-PACK).
The `voice.py` Python script is compatible with Python 2 or Python 3 and depends
on the Windows&reg; only [pywinusb](https://pypi.python.org/pypi/pywinusb/) python
module. The script has been tested to work on a Window 10 host with a BLE 4.0
compatible BLE controller. For help on how to use the `voice.py` script, simply
invoke its help menu `voice.py --help`.

## References

The following reference pages may be helpful during general Bluetooth Low
Energy development.
Users developing Bluetooth 4.2 applications are encouraged to read the
[BLE-Stack for Bluetooth 4.2 User's Guide](http://software-dl.ti.com/simplelink/esd/simplelink_cc2640r2_sdk/1.50.00.58/exports/docs/blestack/ble_user_guide/html/ble-stack-3.x-guide/index.html).

As an additional resource, users are encouraged to complete the
[SimpleLink Academy](http://software-dl.ti.com/lprf/simplelink_academy/overview.html)
training.

Other resources can be found below:

* [BLE wiki](http://www.ti.com/ble-wiki)
* [BLE E2E Page](www.ti.com/ble-forum)
