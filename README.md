## Table of Contents

* [Navigating the Repository](#navigating-the-repository)
* [Change Log](#change-log)
* [Installation](#installation)
* [Required Tools](#required-tools)
* [Examples List](#examples--demo-list)
* [Tools](#tools)
* [References](#references)
* [FAQ](docs/faq.md)
* [Versioning BLE-Stack Projects](docs/suggested_workflow.md)

# Introduction

These examples and demos are for **TI SimpleLink CC26x2 SDK 1.60.00.43**

This repository contains *Bluetooth&reg;* 5 sample applications for
Texas Instruments' SimpleLink CC26x2 SDK.

**Do not** use GitHub's bug tracking feature for support. For inquiries, see the
[Bluetooth&reg; low energy Forum](https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538).

To use the examples and tools in this repository, please download and install
the [SimpleLink CC26x2 SDK](http://www.ti.com/tool/SIMPLELINK-CC26X2-SDK) **first**, and if
necessary [buy an evaluation kit](http://www.ti.com/tool/LAUNCHXL-CC26X2R1).

For other hardware and software resources,
[please visit our wiki](http://www.ti.com/ble-wiki). If you have any questions
please refer to the [FAQ page](docs/faq.md).

For extra examples for **TI CC2640R2 SDK**, see
[Branch: simplelink_sdk-1.50](https://github.com/ti-simplelink/ble_examples/tree/simplelink_sdk-1.50).

For extra examples for **TI BLE-Stack 2.2.x SDK** (CC26x0R1), see
[Branch: ble_examples-2.2](https://github.com/ti-simplelink/ble_examples/tree/ble_examples-2.2).

# Navigating the Repository

The examples provided on this GitHub page serve as a plugin to a corresponding
BLE-Stack SDK release. The master branch will always point to the latest release.

Older releases can be accessed by checking out/downloading their corresponding
branch. For more information on supported examples
please consult the readme.md of the desired branch/release.

## Change Log

Note: The version numbers below are related to GitHub ble_examples releases.
The numbering scheme is in the form of M.mm.pp.bb. The fields pp.bb are incremented
as GitHub examples are released, M.mm will map a GitHub release to a SimpleLink
SDK release.

### 1.60.00.00
Initial offering of SimpleLink CC26x2 SDK 1.60.00.43 examples.
* Bidirectional Audio Demo (Central/Peripheral): now on BLE5-Stack
* Created i2secho application that demonstrates audio sampling and playback
  with the CC3200AUDBOOST

## Installation

This repository can be cloned and tracked using Git. For instructions on how to
clone a repository from GitHub please refer to this guide:
[Clone from GitHub](https://help.github.com/articles/cloning-a-repository/)

For users who are unfamiliar with Git, there is the option of downloading the
contents of the repository as a zip file. See instructions below.

1. Click the green "Clone or download" button
1. Select "Download ZIP" option
1. Zip folder will appear in your Downloads folder

This repository can be cloned/download anywhere on your computer. There is a
dependency between this repository and the SimpleLink CC26x2 SDK install
location.

By default the SimpleLink CC26x2 SDK will install to:

    C:\ti\simplelink_cc26x2_sdk_1_60_00_43

If the SimpleLink CC26x2 SDK must be installed to a different location, then
see the [FAQ page](docs/faq.md) for IDE specific instructions for changing
environment variables.

## Required Tools

Similar to the SimpleLink CC26x2 SDK, the examples in this repository support
the CCS and IAR toolchains. Please pay careful attention to versions of these
tools, the supported version are listed below. Using a non supported version is
untested and may result in unexpected behavior.

* CCS v7.04.00 with TI ARM Compiler v16.09.06LTS
* IAR for ARM v8.11.4

For more information on toolchain setup, please refer to our
[FAQ page](docs/faq.md).

## FAQ

The [FAQ page](docs/faq.md) will try to address some of the common questions
related to the ble_examples repo.

## Examples / Demo List

### Full Duplex Bidirectional Audio Demo

Encode and transmit a full duplex bidirectional audio stream over BLE using two
CC26x2 LaunchPads with CC3200AUDBOOST.

* central\_bidirectional\_audio
    * [Documentation](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/central_bidirectional_audio/readme.md)
    * [CCS Project Files](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/central_bidirectional_audio/tirtos/ccs)
    * [IAR Project Files](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/central_bidirectional_audio/tirtos/iar)
    * [Source](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/central_bidirectional_audio/src)

* peripheral\_bidirectional\_audio
    * [Documentation](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/peripheral_bidirectional_audio/readme.md)
    * [CCS Project Files](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/peripheral_bidirectional_audio/tirtos/ccs)
    * [IAR Project Files](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/peripheral_bidirectional_audio/tirtos/iar)
    * [Source](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/peripheral_bidirectional_audio/src)

### I2S Echo

Read audio frames in from an external codec and loop them back using I2S.

* i2secho
    * [Documentation](examples/rtos/CC26X2R1_LAUNCHXL/drivers/i2secho/README.md)
    * [CCS Project Files](examples/rtos/CC26X2R1_LAUNCHXL/drivers/i2secho/tirtos/ccs)
    * [IAR Project Files](examples/rtos/CC26X2R1_LAUNCHXL/drivers/i2secho/tirtos/iar)
    * [GCC Project Files](examples/rtos/CC26X2R1_LAUNCHXL/drivers/i2secho/tirtos/gcc)
    * [Source](examples/rtos/CC26X2R1_LAUNCHXL/drivers/i2secho/)


## References

The following reference pages may be helpful during general Bluetooth Low
Energy development. New users of the SimpleLink CC26x2 platform and Bluetooth
5 development are encouraged to read the
[BLE5-Stack User's Guide](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC26X2%20SDK%2FDocuments%2FBLE5-Stack%2FBLE5-Stack%20User's%20Guide).

As an additional resource, users are encouraged to complete the
[SimpleLink Academy](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC26X2%20SDK%2FSimpleLink%20Academy)
training.

Other resources can be found below:

* [BLE wiki](http://www.ti.com/ble-wiki)
* [BLE E2E Page](www.ti.com/ble-forum)
