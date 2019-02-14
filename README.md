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

These examples and demos are for **TI SimpleLink CC26x2 SDK 2.30.00.34**

This repository contains *Bluetooth&reg;* 5 sample applications for
Texas Instruments' SimpleLink CC26x2 SDK. These examples have not been
validated as production-ready.

**Do not** use GitHub's bug tracking feature for support. For inquiries, see the
[Bluetooth&reg; low energy Forum](https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538).

To use the examples and tools in this repository, please download and install
the [SimpleLink CC26x2 SDK](http://www.ti.com/tool/SIMPLELINK-CC26X2-SDK) **first**, and if
necessary [buy an evaluation kit](http://www.ti.com/tool/LAUNCHXL-CC26X2R1).

If you have any questions please refer to the [FAQ page](docs/faq.md).
For examples for other SDK versions and platforms, see table below.

<table>
  <tbody>
    <tr>
      <th width = 50%>SDK</th>
      <th>Examples</th>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_sdk-2.30">TI SimpleLink CC26x2 SDK 2.30.00.34 (current)</a>
      </td>
      <td>
        <ul>
          <li><a href="#bluetooth-5-throughput-demo">Bluetooth 5 Throughput Demo</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_sdk-2.10">TI SimpleLink CC26x2 SDK 2.10.00.44</a>
      </td>
      <td>
        <ul>
          <li>Full Duplex Bidirectional Audio Demo (Central/Peripheral)</li>
          <li>I2S Echo</li>
          <li>Serial Port Profile</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_sdk-1.60">TI SimpleLink CC26x2 SDK 1.60.00.43</a>
      </td>
      <td>
        <ul>
          <li>Full Duplex Bidirectional Audio Demo (Central/Peripheral)</li>
          <li>I2S Echo</li>
          <li>Serial Port Profile</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_cc2640r2_sdk-2.20">TI SimpleLink CC2640R2 SDK 2.20.00.49</a>
      </td>
      <td>
        <ul>
          <li>Simple Serial Socket</li>
          <li>Micro BLE Stack Broadcaster Observer</li>
          <li>Bidirectional Audio</li>
          <li>Simple Peripheral Observer</li>
          <li>Apple Notification Center Service</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_sdk-1.50">TI SimpleLink CC2640R2 SDK 1.50.00.58</a>
      </td>
      <td>
        <ul>
          <li>Micro BLE Stack Broadcaster + Observer</li>
          <li>Full Duplex Bidirectional Audio Demo</li>
          <li>Serial Port Profile</li>
          <li>Simple Peripheral Observer BLE</li>
          <li>Apple Notification Center Service</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_sdk-1.40">TI SimpleLink CC2640R2 SDK 1.40.00.42</a>
      </td>
      <td>
        <ul>
          <li>Full Duplex Bidirectional Audio Demo</li>
          <li>Bluetooth 5 Throughput Demo</li>
          <li>Bluetooth 5 Long Range Demo</li>
          <li>Serial Port Profile</li>
          <li>Simple Peripheral Observer BLE</li>
          <li>Apple Notification Center Service</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_sdk-1.35">TI SimpleLink CC2640R2 SDK 1.35.00.33</a>
      </td>
      <td>
        <ul>
          <li>Bluetooth 5 Throughput Demo</li>
          <li>Bluetooth 5 Long Range Demo</li>
          <li>Serial Port Profile</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/ble_examples-2.2">TI BLE-Stack SDK v2.2.x</a>
      </td>
      <td>
        <ul>
          <li>beacon_rfdriver</li>
          <li>hid_adv_remote_privacy</li>
          <li>hid_emu_kbd</li>
          <li>multi_role</li>
          <li>security_examples</li>
          <li>simple_beacon</li>
          <li>simple_central_lp</li>
          <li>simple_central_audio_receiver</li>
          <li>simple_peripheral_audio_transmitter</li>
          <li>simple_eddystone</li>
          <li>simple_peripheral_observer</li>
          <li>simple_proprietary_beacon</li>
          <li>spp_over_ble</li>
          <li>throughput_example</li>
          <li>serial_bootloader</li>
          <li>central_to_multiperipheral</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/ble_examples-2.1">TI BLE-Stack SDK v2.1.x</a>
      </td>
      <td>
        <ul>
          <li>SimpleAP</li>
          <li>SimpleNP</li>
          <li>MultiRole</li>
          <li>SimpleBLEPeripheral: LCD to UART</li>
          <li>SimpleBLEPeripheral: porting to TI-RTOS 2.15</li>
          <li>Apple Notification Center Service</li>
        </ul>
      </td>
    </tr>
  </tbody>
</table>

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

### 2.30.00.00
Initial offering of SimpleLink CC26x2 SDK 2.30.00.34 examples.

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

    C:\ti\simplelink_cc26x2_sdk_2_30_00_34

If the SimpleLink CC26x2 SDK must be installed to a different location, then
see the [FAQ page](docs/faq.md) for IDE specific instructions for changing
environment variables.

## Required Tools

Similar to the SimpleLink CC26x2 SDK, the examples in this repository support
the CCS and IAR toolchains. Please pay careful attention to versions of these
tools, please refer to the release notes for supported versions.

For more information on toolchain setup, please refer to our
[FAQ page](docs/faq.md).

## FAQ

The [FAQ page](docs/faq.md) will try to address some of the common questions
related to the ble_examples repo.

## Examples / Demo List

### Full Duplex Bidirectional Audio Demo

**Note: These examples will be deprecated as of the 2.40 SDKs.**

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

**Note: These examples will be deprecated as of the 2.40 SDKs.**

Read audio frames in from an external codec and loop them back using I2S.

* i2secho
    * [Documentation](examples/rtos/CC26X2R1_LAUNCHXL/drivers/i2secho/README.md)
    * [CCS Project Files](examples/rtos/CC26X2R1_LAUNCHXL/drivers/i2secho/tirtos/ccs)
    * [IAR Project Files](examples/rtos/CC26X2R1_LAUNCHXL/drivers/i2secho/tirtos/iar)
    * [GCC Project Files](examples/rtos/CC26X2R1_LAUNCHXL/drivers/i2secho/tirtos/gcc)
    * [Source](examples/rtos/CC26X2R1_LAUNCHXL/drivers/i2secho/)

### Bluetooth 5 Throughput Demo

Showcase a high (near theoretical max) data throughput over Bluetooth 5. The demo
can be run with two CC26x2 Launchpads, or one CC26x2 Launchpad and one smartphone
with the Simplelink Starter app.

* throughput\_peripheral
    * [Documentation](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/throughput_peripheral/readme.md)
    * [CCS Project Files](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/throughput_peripheral/tirtos/ccs)
    * [Source](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/throughput_peripheral/src)
* throughput\_central
    * [Documentation](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/throughput_central/readme.md)
    * [CCS Project Files](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/throughput_central/tirtos/ccs)
    * [Source](examples/rtos/CC26X2R1_LAUNCHXL/ble5apps/throughput_central/src)

## References

The following reference pages may be helpful during general Bluetooth Low
Energy development. New users of the SimpleLink CC26x2 platform and Bluetooth
5 development are encouraged to read the
[BLE5-Stack User's Guide](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC26X2%20SDK%2FDocuments%2FBLE5-Stack%2FBLE5-Stack%20User's%20Guide).

As an additional resource, users are encouraged to complete the
[SimpleLink Academy](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC26X2%20SDK%2FSimpleLink%20Academy)
training.

Other resources can be found below:

* [BLE E2E Page](www.ti.com/ble-forum)
