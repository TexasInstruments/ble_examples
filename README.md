# Introduction

These examples and demos are for **TI SimpleLink CC2640R2 SDK 1.40.00.45**

This repository contains **experimental** *Bluetooth&reg;* 5 and
*Bluetooth&reg;* 4.2 Low Energy sample applications for Texas Instruments'
SimpleLink CC2640R2 SDK. These examples have not been validated as
production-ready.

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

## Change Log

Note: The version numbers below are related to GitHub ble_examples releases.
The numbering scheme is in the form of M.mm.pp.bb. The fields pp.bb are incremented
as GitHub examples are released, M.mm will map a GitHub release to a SimpleLink
SDK release.

### 1.40.00.00
Initial offering of GitHub examples for Simplelink CC2640R2 SDK 1.40.00.45
* BLE5-Stack: Long Range Demo (Central/Peripheral)
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

    C:\ti\simplelink_cc2640r2_sdk_1_40_00_45

If the Simplelink CC2640R2 SDK must be installed to a different location, then
see the [FAQ page](docs/faq.md) for IDE specific instructions for changing
environment variables.

## Required Tools

Similar to the Simplelink CC2640R2 SDK, the examples in this repository support
the CCS and IAR toolchains. Please pay careful attention to versions of these
tools, the supported version are listed below. Using a non supported version is
untested and may result in unexpected behavior.

* CCS v7.2.0 with TI ARM Compiler v16.09
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

* simple\_central\_bidirectional\_audio
    * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_central_bidirectional_audio/readme.md)
    * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_central_bidirectional_audio/tirtos/ccs)
    * [Source](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_central_bidirectional_audio/src)

* simple\_peripheral\_bidirectional\_audio
    * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_peripheral_bidirectional_audio/readme.md)
    * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_peripheral_bidirectional_audio/tirtos/ccs)
    * [Source](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_peripheral_bidirectional_audio/src)

### Bluetooth 5 Long Range Demo

This is a 2 part demo where a longrange\_central and longrange\_peripheral
connect with one another, demonstrating the BLE5-Stack's long range capabilities
using the Coded PHY (S8).

* longrange\_central
    * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/longrange_central/readme.md)
    * [IAR Project Files](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/longrange_central/tirtos/iar)
    * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/longrange_central/tirtos/ccs)
    * [Source](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/longrange_central/src)
* longrange\_peripheral
    * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/longrange_peripheral/readme.md)
    * [IAR Project Files](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/longrange_peripheral/tirtos/iar)
    * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/longrange_peripheral/tirtos/ccs)
    * [Source](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/longrange_peripheral/src)

### Apple Notification Center Service (ANCS) Demo

This is an example that demonstrates the use and functionality of the ANCS. ANCS is a GATT 
service present on iOS devices used to retrieve and interact with iOS notifications. 

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
Energy development. New users of the Simplelink CC2640R2 platform and Bluetooth
5 development are encouraged to read the
[BLE5-Stack User's Guide][BLE5-Stack].
Users developing Bluetooth 4.2 applications are encouraged to read the
[BLE Software Developer's Guide][BLE-Stack].

As an additional resource, users are encouraged to complete the
[SimpleLink Academy](http://software-dl.ti.com/lprf/simplelink_academy/overview.html)
training.

Other resources can be found below:

* [BLE wiki](http://www.ti.com/ble-wiki)
* [BLE E2E Page](www.ti.com/ble-forum)

[BLE5-Stack]: (http://software-dl.ti.com/simplelink/esd/simplelink_cc2640r2_sdk/1.40.00.45/exports/docs/ble5stack/ble_user_guide/html/ble-stack-5.x-guide/index.html)
[BLE-Stack]:  (http://software-dl.ti.com/simplelink/esd/simplelink_cc2640r2_sdk/1.40.00.45/exports/docs/blestack/ble_user_guide/html/ble-stack-3.x-guide/index.html)
