## Table of Contents

* [Navigating the Repository](#navigating-the-repository)
* [Change Log](#change-log)
* [Installation](#installation)
* [Required Tools](#required-tools)
* [Examples List](#examples--demo-list)
* [Tools](#tools)
* [References](#references)
* [FAQ](docs/faq.md)
* [Suggested Work Flow](docs/suggested_workflow.md)

# Introduction

These examples and demos are for **TI SimpleLink CC2640R2 SDK 3.40.00.10**

This repository contains *Bluetooth&reg;* Low Energy sample applications for
Texas Instruments' SimpleLink CC2640R2 SDK. These examples have not been
validated as production-ready. Services and profiles in this repository have not 
been validated or certified.

**Do not** use GitHub's bug tracking feature for support. For inquiries, see the
[Bluetooth&reg; low energy Forum](https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538).

To use the examples and tools in this repository, please download and install
the [SimpleLink CC2640R2 SDK](http://www.ti.com/ble-stack) **first**, and if
necessary [buy an evaluation kit](http://www.ti.com/tool/launchxl-cc2640r2).

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
  <a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_cc2640r2_sdk-3.40">TI SimpleLink CC2640R2 SDK 3.40.00.10 (current)</a>
      </td>
      <td>
        <ul>
          <li><a href="#simple-serial-socket">Simple Serial Socket</a></li>
          <li><a href="#simple-serial-socket">Bluetooth 5 Simple Serial Socket</li>
          <li><a href="#simple-peripheral-observer">Simple Peripheral Observer</li>
          <li><a href="#blood-pressure-monitor">Blood Pressure Monitor</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
	<a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_cc2640r2_sdk-3.30">TI SimpleLink CC2640R2 SDK 3.30.00.20</a>
      </td>
      <td>
        <ul>
          <li>Simple Serial Socket</a></li>
          <li>Bluetooth 5 Simple Serial Socket</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_sdk-3.20">TI SimpleLink CC13x2 / CC26x2 SDK 3.20.00.67</a>
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
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_sdk-2.40">TI SimpleLink CC13x2 / CC26x2 SDK 2.40.00.81</a>
      </td>
      <td>
        <ul>
          <li>Simple Serial Socket</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_sdk-2.30">TI SimpleLink CC26x2 SDK 2.30.00.34</a>
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
          <li><a href="#simple-serial-socket">Simple Serial Socket</a></li>
          <li><a href="#micro-ble-stack-broadcaster-observer">Micro BLE Stack Broadcaster Observer</li>
          <li><a href="#bidirectional-audio">Bidirectional Audio</li>
          <li><a href="#simple-peripheral-observer">Simple Peripheral Observer</li>
          <li><a href="#apple-notification-center-service">Apple Notification Center Service</li>
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
        <a href = "https://github.com/ti-simplelink/ble_examples/tree/simplelink_sdk-1.40">TI SimpleLink CC2640R2 SDK 1.40.00.45</a>
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

For for more information about different SDK components, please consult the
BLE-stack User's Guide.

## Change Log

Note: The version numbers below are related to GitHub ble_examples releases.
The numbering scheme is in the form of M.mm.pp.bb. The fields pp.bb are incremented
as GitHub examples are released, M.mm will map a GitHub release to a SimpleLink
SDK release.

### 3.40.00.00
Initial offering of select SimpleLink CC2640R2 SDK 3.40.00.10 examples and one 
new example added:
* Blood Pressure Monitor: This sample project implements the Blood Pressure 
  profiles in a Bluetooth Low Energy peripheral device to provide an example 
  blood pressure monitor (BPM) using simulated measurement data.

### 3.30.00.00
Initial offering of select SimpleLink CC2640R2 SDK 3.30.00.20 examples.

### 3.20.00.00
Initial offering of SimpleLink CC13x2 / CC26x2 SDK 3.20.00.67 examples supporting 
the rev. E of the SimpleLinkCC26x2 and CC13x2 MCUs.

### 2.40.00.00
Initial offering of SimpleLink CC13x2 / CC26x2 SDK 2.40.00.81 examples.

### 2.20.00.00
New examples added.
* BLE-Stack:  Simple Serial Socket Server/Client added

**SPP over BLE is now deprecated and replaced by Simple Serial Socket**

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

    C:\ti\simplelink_cc2640r2_sdk_x_xx_xx_xx

If the SimpleLink CC2640R2 SDK must be installed to a different location, then
see the [FAQ page](docs/faq.md) for IDE specific instructions for changing
environment variables.

## Required Tools

Similar to the SimpleLink CC2640R2 SDK, the examples in this repository support
the CCS and IAR toolchains. Please refer to the release notes for the supported
versions of each toolcahin. Using a non supported version is
untested and may result in unexpected behavior.

For more information on toolchain setup, please refer to our
[FAQ page](docs/faq.md).

## FAQ

The [FAQ page](docs/faq.md) will try to address some of the common questions
related to the ble_examples repo.

## Examples / Demo List

### Simple Serial Socket

This is a UART over BLE bridge example where a Simple Serial Socket Client
(SSSC) and Simple Serial Socket Server (SSSS) connect with one another and
exchange UART data over the air. An generic Simple Stream Service is used
making it easy to exchange data sink and source to create a custom serial
socket over BLE implementation.

**Note: this example replaces the SPP over BLE example**

* Simple Serial Socket Client
    * BLE 4.2
      * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_serial_socket_client/readme.md)
      * [IAR Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_serial_socket_client/tirtos/iar)
      * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_serial_socket_client/tirtos/ccs)
      * [Source](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_serial_socket_client/src)
    * BLE 5.0
      * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/simple_serial_socket_client/readme.md)
      * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/simple_serial_socket_client/tirtos/ccs)
      * [Source](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/simple_serial_socket_client/src)
* Simple Serial Socket Server
    * BLE 4.2
      * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_serial_socket_server/readme.md)
      * [IAR Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_serial_socket_server/tirtos/iar)
      * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_serial_socket_server/tirtos/ccs)
      * [Source](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_serial_socket_server/src)
    * BLE 5.0
      * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/simple_serial_socket_server/readme.md)
      * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/simple_serial_socket_server/tirtos/ccs)
      * [Source](examples/rtos/CC2640R2_LAUNCHXL/ble5apps/simple_serial_socket_server/src)

### Simple Peripheral Observer

This sample project is used to add observer role to a BLE peripheral device to
show scanning functionality in a peripheral based project.

* simple\_peripheral\_observer
    * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_peripheral_observer/readme.md)
    * [IAR Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_peripheral_observer/tirtos/iar)
    * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_peripheral_observer/tirtos/ccs)
    * [Source](examples/rtos/CC2640R2_LAUNCHXL/bleapps/simple_peripheral_observer/src)

### Blood Pressure Monitor

This sample project implements the Blood Pressure profiles in a Bluetooth low 
energy peripheral device to provide an example blood pressure monitor (BPM) 
using simulated measurement data. The application implements the Sensor role of 
the blood pressure profile. The project is based on the adopted profile and 
service specifications for blood pressure.

* blood\_pressure
    * [Documentation](examples/rtos/CC2640R2_LAUNCHXL/bleapps/blood_pressure/readme.md)
    * [CCS Project Files](examples/rtos/CC2640R2_LAUNCHXL/bleapps/blood_pressure/tirtos/ccs)
    * [Source](examples/rtos/CC2640R2_LAUNCHXL/bleapps/blood_pressure/src)

## References

The following reference pages may be helpful during general Bluetooth Low
Energy development.
Users developing Bluetooth applications are encouraged to read the
[BLE-Stack User's Guide](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC2640R2%20SDK%2FDocuments%2FBLE-Stack%2FBLE-Stack%20User's%20Guide).

As an additional resource, users are encouraged to complete the
[SimpleLink Academy](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC2640R2%20SDK%2FSimpleLink%20Academy)
training.

Other resources can be found below:

* [BLE E2E Page](www.ti.com/ble-forum)
