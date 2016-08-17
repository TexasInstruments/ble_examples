Purpose / Scope
===============

This page will document how to demonstrate an end to end BLE voice solution using two CC2650 devices.
Emphasis will be placed on the central device which is reponsible for recieving and decoding the voice stream.

TI's voice system supports streaming voice data from the Pulse Density Modulation (PDM) microphone on the CC2650 SensorTag and HID Advanced Remote kits.
Voice data is transferred over BLE using the TI audio\_profile which is a Voice Over GATT Profile (VoGP) design.
In this demo, data flows unidirectionally between a streamer (GATT server) and a receiver (GATT client) device.

The supported streamer examples from the BLE-Stack SDK are:
 - `hid_adv_remote`
 - `sensortag_audio`

The streaming devices are the ones containing the PDM microphone on board. These are the devices the user will ultimately be speaking into.
These devkits and firmware examples can be found within the BLE-Stack SDK release 2.02. Documentation on the CC2650RC can be found at [CC2650 Remote Control User's Guide](http://processors.wiki.ti.com/index.php/CC2650RC_UG), and documentation on the SensorTag can be found at [CC2650 SensorTag User's Guide](http://processors.wiki.ti.com/index.php/CC2650_SensorTag_User%27s_Guide).

The supported receiver examples are:
 - `simple_audio_receiver` (hosted in this repo)
 - CC254x based ble\_hidvogatt\_1\_00\_01\_02 (Found on the [CC2650RC's product page](http://www.ti.com/tool/cc2650rc))

The purpose of this page is the document/demo the **`simple_audio_receiver`** example.

This example is based on the simple\_central project
from the **BLE-Stack v2.2.0** installer.

This project is slightly modified to:

 - Run on the CC2650 LaunchPad
 - Automatically connect to CC2650 SensorTag or CC2650RC based on advertisement data
 - Receive voice stream using the TI VoGP audio\_profile
 - Send the voice stream to the PC over UART for post processing (Python script included)


Prerequisites
=============

#### Hardware Requirements

Before running the demo, the user will need the following components:

- CC2650 voice enabled development kit (SensorTag or HID Advanced Remote)
  - [CC2650 RC](http://www.ti.com/tool/cc2650rc)
  - [CC2650 STK](http://www.ti.com/tool/cc2650stk)
- [BLE-Stack v2.2.0](http://www.ti.com/ble-stack)
- [CC2650 LaunchPad](http://www.ti.com/tool/launchxl-cc2650)
- A PC that supports `.wav` file playback.
- **OPTIONAL:** [Sharp LCD BoosterPack](http://www.ti.com/tool/430boost-sharp96)

#### Firmware Requirements

1. Load the voice streaming device with it's proper firmware image from the BLE-Stack SDK
  * For the CC2650 STK this is the `sensortag_audio` project
  * For the CC2650 RC this is the `hid_adv_remote` project
2. Load the `simple_central_audio_reciever` project onto the CC2650 LaunchPad
3. Handle Python dependencies for  `audio_frame_serial_print.py` from [tools/scripts/audio folder](../tools/scripts/audio)
  * Requires [Python 2.7](https://www.python.org/download/releases/2.7/)
  * The script also requires the following Python modules: struct, wave, serial, time, winsound
  * See the [FAQ](faq.md) for more info


Running the Demo
================

After building the firmware required for the voice streamer and receiver, you are ready to demo the voice capabilites of the CC2650.

1. Connect the audio\_receiver (CC2650 LaunchPad) device to your PC. Use windows device manager to note the COM port, it is the User/UART port (COM4 in picture below)

   ![Connecting the COM Port](doc_resources/dev_mgr_xds110.PNG)

2. Make note of the COM port from step #1 and update the Python script to use that port by replacing COM38 the following line:
    ```Python
    ser = Serial("COM38", 400000, timeout=0.1)
    ```
3. Run `audio_frame_serial_print.py`
4. Power up the voice streaming device.
 * The SensorTag will advertise out of the box, this is indicated by the green blinking LED.
 * The HID Advanced Remote will advertise after any button press.
5. Power up the audio\receiver device
 * If the LCD is attached it will display
 ```
 Audio central
 <BD_ADDR>
 Idle...
 ```
6. Start Discovery on the central device by pressing the left key on the LaunchPad. Scanning is indicated by blinking green LED on the LaunchPad.
 * The Central device will scan the peripheral's advertisement data for either the TI_COMPANY_ID (SensorTag) or the HID_SERV_UUID (HID Advanced Remote).
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
 ```
7. If an acceptable voice streaming device is found (STK or CC2650RC) then the central will connect, pair, and bond to the device. If attached the Sharp LCD will display:
  ```
  Audio central
  <BD_ADDR>
  Bond Saved OR Param Update: 0
  <PEER_BD_ADDR>
  ```
8. The devices are now ready to stream voice over BLE
 * Press and hold the MIC button to start streaming voice from the HID Advanced Remote
 * Press and hold the right key on the SensorTag to start streaming voice from the SensorTag

9. The Python script will read the voice frames from the CC2650 and decode them into `.wav` files. These files can be played back on the PC.
* The files are saved in the format: `pdm_test_%Y-%m-%d_%H-%M-%S_adpcm` where Y, m, d, H, M, S are used to store the time stamp when the file was saved.

References
==========
 * [CC2650 Remote Control User's Guide](http://processors.wiki.ti.com/index.php/CC2650RC_UG)
 * [CC2650 Remote Control Developer's Guide](http://processors.wiki.ti.com/index.php/CC2650RC_Getting_Started_with_Development#Getting_started_with_Development)
 * [CC2650 SensorTag User's Guide](http://processors.wiki.ti.com/index.php/CC2650_SensorTag_User%27s_Guide)
 * [Voice Over Remote Control](http://www.ti.com/lit/an/swra506/swra506.pdf)
