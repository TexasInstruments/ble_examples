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

This repository contains *Bluetooth&reg; Low Energy* & *Proprietary RF* sample applications for
Texas Instruments' SimpleLink CC13xx / CC26xx SDK, SimpleLink CC2640R2 SDK, and SimpleLink Low Power F3 SDK.

These examples  are each hosted on different branches of this repo. Please reference the table below to find examples for each of these SDKs.

These examples have not been validated as production-ready.

**Do not** use GitHub's bug tracking feature for support. For inquiries, see the
[Bluetooth&reg; low energy Forum](https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538)
and the [Other Wireless Technologies Forum](https://e2e.ti.com/support/wireless-connectivity/other-wireless-group/other-wireless/f/other-wireless-technologies-forum).

To use the examples and tools in this repository, please download and install
the relevant SDK listed in a given branch **first**, and if necessary obtain the required evaluation kit:

If you have any questions please refer to the [FAQ page](docs/faq.md).


<table>
  <tbody>
    <tr>
      <th width = 50%>SDK</th>
      <th>Examples</th>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_low_power_f3_sdk-7.40">TI SimpleLink Low Power F3 SDK 7.40.00.00</a>
      </td>
      <td>
        <ul>
          <li>Data Stream UART over BLE example</li>
          <li>Basic BLE GATT Client example</li>
          <li>UART RF Carrier Wave</li>
          <li>Serial Boot Loader Tool v1.0.0 (SDK version independent)</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_low_power_f3_sdk-7.20">TI SimpleLink Low Power F3 SDK 7.20.00.00</a>
      </td>
      <td>
        <ul>
          <li>Data Stream UART over BLE example</li>
          <li>Basic BLE GATT Client example</li>
          <li>Serial Boot Loader Tool v1.0.0 (SDK version independent)</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_cc13xx_26xx_sdk-6.40">TI SimpleLink CC13xx / CC26xx SDK 6.4x.xx.xx</a>
      </td>
      <td>
        <ul>
          <li>Simple Serial Socket (for CC2651R3SIPA only)</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_cc13x2_26x2_sdk-5.10">TI SimpleLink CC13x2 / CC26x2 SDK 5.10.00.00</a>
      </td>
      <td>
        <ul>
          <li>Simple Serial Socket</li>
          <li>Bluetooth 5 Throughput Demo</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_cc13xx_26xx_sdk-4.10">TI SimpleLink CC13x2 / CC26x2 SDK 4.10.00.00</a>
      </td>
      <td>
        <ul>
          <li>Simple Serial Socket</li>
          <li>Bluetooth 5 Throughput Demo</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_cc13xx_26xx_sdk-3.20">TI SimpleLink CC13x2 / CC26x2 SDK 3.20.00.67</a>
      </td>
      <td>
        <ul>
          <li>Simple Serial Socket</li>
          <li>Bluetooth 5 Throughput Demo</li>
          <li>Tree Structure Network</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_cc13xx_26xx_sdk-2.40">TI SimpleLink CC13x2 / CC26x2 SDK 2.40.00.81</a>
      </td>
      <td>
        <ul>
          <li>Simple Serial Socket</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_cc26x2_sdk-2.30">TI SimpleLink CC26x2 SDK 2.30.00.34</a>
      </td>
      <td>
        <ul>
          <li>Full Duplex Bidirectional Audio Demo (Central/Peripheral)</li>
          <li>I2S Echo</li>
          <li>Serial Port Profile</li>
          <li>Bluetooth 5 Throughput Demo</li>
          <li>Tree Structure Network</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_cc26x2_sdk-2.10">TI SimpleLink CC26x2 SDK 2.10.00.44</a>
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
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_sdk-1.60">TI SimpleLink CC26x2 SDK 1.60.00.43</a>
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
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_cc2640r2_sdk-2.20">TI SimpleLink CC2640R2 SDK 2.20.00.49</a>
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
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_sdk-1.50">TI SimpleLink CC2640R2 SDK 1.50.00.58</a>
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
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_sdk-1.40">TI SimpleLink CC2640R2 SDK 1.40.00.45</a>
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
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/simplelink_sdk-1.35">TI SimpleLink CC2640R2 SDK 1.35.00.33</a>
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
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/ble_examples-2.2">TI BLE-Stack SDK v2.2.x</a>
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
        <a href = "https://github.com/TexasInstruments/ble_examples/tree/ble_examples-2.1">TI BLE-Stack SDK v2.1.x</a>
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

### SimpleLink Low Power F3 SDK Examples 7.40.00.00
Update of the examples for SimpleLinK Low Power F3 SDK (CC23XX) & Serial Boot Loader tool.
Addition of UART 

### 7.20.00.00
First release of the examples for SimpleLinK Low Power F3 SDK (CC23XX) & Serial Boot Loader tool.

### 6.40.00.00
First release of the examples for CC2651R3SIPA.

### 5.10.00.00
Update of the examples to CC13x2 / CC26x2 SDK 5.10.00.00.
Only supports CC26x2R1.

### 4.10.00.00
Update of the examples to CC13x2 / CC26x2 SDK 4.10.00.00.
First release of the examples for CC1352R1.

### 3.20.00.00
Initial offering of SimpleLink CC13x2 / CC26x2 SDK 3.20.00.67 examples, 
supporting the rev. E of the SimpleLinkCC26x2 and CC13x2 MCUs.

### 2.40.00.00
Initial offering of SimpleLink CC13x2 / CC26x2 SDK 2.40.00.81 examples.

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

By default, your chosen SDK will install to ``C:\ti\``

If the SimpleLink SDK must be installed to a different location, 
then see the [FAQ page](docs/faq.md) for IDE specific instructions for changing
environment variables.

## Required Tools

Similar to the SimpleLink SDK, the examples in this repository 
support the CCS and IAR toolchains (not for all the examples).
Please pay careful attention to versions of these tools, please refer to the
release notes for supported versions.

For more information on toolchain setup, please refer to our
[FAQ page](docs/faq.md).

## FAQ

The [FAQ page](docs/faq.md) will try to address some of the common questions
related to the ble_examples repo.


## References

The following reference pages may be helpful during general Bluetooth Low
Energy development. New users of the SimpleLink Low Power F3 platform and Bluetooth
5 development are encouraged to read the
[BLE5-Stack User's Guide](https://dev.ti.com/tirex/content/simplelink_lowpower_f3_sdk_7_20_00_29/docs/ble5stack/ble_user_guide/html/ble-stack-5.x-guide/index-cc23xx.html).

As an additional resource, users are encouraged to complete the
[SimpleLink Academy](https://dev.ti.com/tirex/explore/node?node=A__AEaxXmSXZjp24G7-XUfwSQ__SIMPLELINK-ACADEMY-CC23XX__gsUPh5j__LATEST)
training.

Other resources can be found below:

* [BLE E2E Page](www.ti.com/ble-forum)