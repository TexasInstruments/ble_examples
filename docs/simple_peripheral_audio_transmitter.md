Purpose / Scope
===============

This page will document how to demonstrate an end to end BLE voice solution using two CC2650 LaunchPads with CC3200AUDBOOST boosterpacks.
Emphasis will be placed on the peripheral device which is responsible for coding and transmitting the voice stream.

TI's voice system has long supported streaming voice data from the Pulse Density Modulation (PDM) microphone on the CC2650 SensorTag and HID Advanced Remote kits. This page documents an addition. The addition is the usage of the CC3200AUDBOOST boosterpack. It also adds another compression algorithm, mSBC.
Voice data is transferred over BLE using the TI audio\_profile which is a Voice Over GATT Profile (VoGP) design. This profile has been update to add an additional start byte. This new start byte indicates that the following stream uses mSBC compression.
In this demo, data flows unidirectionally between a streamer (GATT server) and a receiver (GATT client) device.

Other supported streamer examples from the BLE-Stack SDK are:
 - `hid_adv_remote`
 - `sensortag_audio`

Simple Peripheral Audio Transmitter is designed to demonstrate coding and transmitting the audio stream from the CC3200AUDBOOST boosterpack. This example is based on the simple\_peripheral project from the **BLE-Stack v2.2.1** installer.

The peripheral project was modified as follows:

 - Run on the CC2650 LaunchPad
 - Always advertise and attempt to connect to CC2650 LaunchPad Audio Central
 - Change scan response data to 'SimpleBLEAudioTx' and device name to 'Simple BLE AudioTx'
 - Transmit audio stream using the TI VoGP audio\_profile
 - Switch between ADPCM and mSBC compression
 - [Optional] Stream audio data with IMA-ADPCM mechanism using Data Length Extension feature

Streaming Voice over BLE
========================

For more information about the technical details of TI's Voice Over BLE Solution please see:
 - [Voice Streaming on CC2650](http://processors.wiki.ti.com/index.php/CC2650RC_Getting_Started_with_Development#Voice_Streaming_on_the_CC2650RC)
 - [Voice Over BLE](http://processors.wiki.ti.com/index.php/BLESDK-2.2.x-CC2650RC_Developers_Guide#Voice_Over_BLE)

_Note: The above links also apply to sensortag\_audio projects as well. They employ the same firmware components (PDM driver, audio\_profile, etc) as the Remote Control solutions, but deploy on a different dev board._

Some quick facts about voice over BLE:

 - Input device: `Pulse Density Modulation (PDM) Microphone` or `CC3200AUDBOOST boosterpack`
 - Sample rate: `16kHz`
 - Bit Depth: `16 bits`
 - Compression mechanism: `4:1 IMA-ADPCM` or `mSBC`
 - Required application throughput: `66.67kbps` or `60.8kpbs`
 - Voice quality (IMA-ADPCM) has been qualified by Nuance and is sufficient for voice recognition solutions


Prerequisites
=============

#### Hardware Requirements

_Note: Apple headphones will not work with the CC3200 boosterpack_

Before running the demo, the user will need the following components:

- [CC2650 LaunchPad](http://www.ti.com/tool/launchxl-cc2650)
- [CC3200AUDBOOST](http://www.ti.com/tool/cc3200audboost)
- [BLE-Stack v2.2.1](http://www.ti.com/ble-stack)
- A device to output audio over a 3.5mm jack (connected to the LINE-IN, 3.5mm stereo jack, of the CC3200AUDBOOST boosterpack)
- **OPTIONAL:** A PC to view logging via UART-over-USB, @460800 baudrate
- **OPTIONAL:** [Sharp LCD BoosterPack](http://www.ti.com/tool/430boost-sharp96)

#### Firmware Requirements

1. Load the `simple_peripheral_audio_transmitter` project onto one CC2650 LaunchPad
2. Load the `simple_central_audio_reciever` project onto the other CC2650 LaunchPad


Running the Demo
================

After building the firmware required for the voice transmitter and receiver, you are ready to demo the voice capabilities of the CC2650. Before following the steps below,
be sure that an audio source (i.e. line out from a PC is connected to the LINE_IN jack of the `CC3200AUDBOOST`).

1. Power up the transmitter launchpad
 * If the serial port is attached it will log
 ```
 Audio Tx Peripheral
 <BD_ADDR>
 Initialized
 Advertising
 ```
2. Power up the audio\_receiver device
 * If the serial port is attached it will log
 ```
 Audio Central
 <BD_ADDR>
 Initialized
 Idle...
 ```
3. Start Discovery on the central device by pressing the left key on the LaunchPad. Scanning is indicated by blinking green LED on the LaunchPad.
 * The Central device will scan the peripheral's advertisement data for either the TI\_COMPANY\_ID (SensorTag), the HID\_SERV\_UUID (HID Advanced Remote) or SIMPLEPROFILE\_SERV\_UUID (Audio Tx Peripheral).
 * After finding devices that list these services their advertisement payloads it will scan for the following device names:
 ```c
  static uint8 remoteNameST[] =
  {
    'C', 'C', '2', '6', '5', '0', ' ',
    'S', 'e', 'n',  's',  'o',  'r',  'T',  'a',  'g',
  };
  ...
  static uint8 remoteNameRC[] =
  {
    'H', 'I', 'D', ' ', 'A', 'd', 'v', 'R', 'e', 'm', 'o', 't', 'e'
  };
  ...
  static uint8 remoteNameTx[] =
  {
      'S', 'i', 'm', 'p', 'l', 'e',
      'B', 'L', 'E',
      'A', 'u', 'd', 'i', 'o', 'T', 'x',
  };
 ```
4. If an acceptable voice streaming device is found (STK, CC2650RC or CC2650LP) then the central will connect, pair, and bond to the device. If attached the serial port will log:
 * Audio Central
  ```
  Discovering...
  Pairing started
  Connected
  <PEER_BD_ADDR>
  Pairing success
  Bond Saved OR Param Update: 0
  ```
 * Audio Tx Peripheral
  ```
  Connected
  <PEER_BD_ADDR>
  ```
5. The devices are now ready to stream voice over BLE. On the transmitting launchpad press:
 * Right button to start transmitting with ADPCM compression
 * Left button to start transmitting with mSBC compression
6. The demo is written such that, once a sensortag, remote control or transmitting launchpad is discovered, the audio\_receiver project will pair and bond to it. Scanning/connecting to other devices is not allowed while bonded. (i.e. left button is disabled). In order to "forget the devices" you should:
 * Power off your audio transmitting device. Wait for link to be terminated. Red LED will turn on.
 * Press the right key to erase the bonds from the audio\_receiver project. Green LED will blink 1x while red LED stays on.
 * The bonds are now erased, you can discover and connect to another device.

Optional demo : Stream audio data with IMA-ADPCM mechanism using Data Length Extension feature
------------

The Data Length Extension feature for this stack projet is enabled by adding -DBLE_V42_FEATURES=EXT_DATA_LEN_CFG into build_config.opt file
The needed modification for the application project is wrapped around in the DLE_ENABLED predefine symbol.
The MAX_PDU_SIZE has been modified to be 107.

To enable the application to send 100 bytes audio data in one frame, you simply need to enable DLE_ENABLED in the predefine symbol.

_Note: Data Length Extension feature only supports ADPCM format for now_

Once Data Length Extension feature is enabled, the attached serial port will log:
 * Audio Central with DLE
  ```
  Discovering...
  Pairing started
  Connected
  <PEER_BD_ADDR>
  Pairing success
  Bond Saved
  ```
 * Audio Tx Peripheral with DLE
  ```
  Connected
  <PEER_BD_ADDR>
  ```


Demo LED states
===============

The following states of the device can be described by the red and green LEDs on the LaunchPad.

Audio Central:
* Idle + bonds forgotten: Red LED is on, solid
* Scanning for devices: Green LED is flashing
* Device connected + bond saved: Green LED is on, solid
* Device connected and bonded + streaming voice: Red LED blinks on both LP and STK or Remote.
* Bonds forgotten: Green LED blinks 1x while red LED is on.

Audio Tx Peripheral:
* Transmitting mSBC compressed audio: Red LED is active, and on (Green is inactive LED)
* Transmitting ADPCM compressed audio: Green LED is active, and on (Red is inactive LED)
* Fails to add notification to queue: Active LED flicker
* I2S driver fails to get buffer for next frame: Inactive LED flicker

Useful tip
==========

To make more RAM available to HEAP some variables are placed in AUX_RAM. This ram is available when the Sensor Controller is not in use. The following code in `simple_peripheral_audio_transmitter.c` shows this placement:

IAR:
```
#pragma default_variable_attributes = @ "AUX_RAM_SECTION"
```

CCS:
```
#pragma DATA_SECTION(i2sContMgtBuffer, ".aux_ram")
#pragma DATA_SECTION(audio_encoded, ".aux_ram")
#pragma DATA_SECTION(sbc, ".aux_ram")
#pragma DATA_SECTION(written, ".aux_ram")
#pragma DATA_SECTION(streamVariables, ".aux_ram")
```

References
==========
 * [CC2650 Remote Control User's Guide](http://processors.wiki.ti.com/index.php/CC2650RC_UG)
 * [CC2650 Remote Control Developer's Guide](http://processors.wiki.ti.com/index.php/CC2650RC_Getting_Started_with_Development#Getting_started_with_Development)
 * [CC2650 SensorTag User's Guide](http://processors.wiki.ti.com/index.php/CC2650_SensorTag_User%27s_Guide)
 * [Voice Over BLE](http://processors.wiki.ti.com/index.php/BLESDK-2.2.x-CC2650RC_Developers_Guide#Voice_Over_BLE)
