
Purpose / Scope
===============

This sample project implements the Blood Pressure profiles in a Bluetooth low 
energy peripheral device to provide an example blood pressure monitor (BPM) 
using simulated measurement data. The application implements the Sensor role of 
the blood pressure profile. The project is based on the adopted profile and 
service specifications for blood pressure. The project also includes the Device 
Information Service. 

Prerequisites
=============

#### Hardware Requirements

Before running the demo, the user will need the following components:

- 1x [CC2640R2 LaunchPad](http://www.ti.com/tool/launchxl-cc2640r2)
- 1x Micro USB cable

#### Firmware Requirements

- [CC2640R2 SDK](http://www.ti.com/tool/SIMPLELINK-CC2640R2-SDK)

Running the Demo
================

Please follow the steps below to run the demo:

1. Compile and load the examples

2. Connect the Boards to the PC terminal
 - Follow the steps from the [FAQ](faq.md) to connect to the LaunchPad

3. Power the board and verify it is initialized
 - When you power on the device the following will be printed out on the display:

Blood Pressure
Started
0x54C60EF048C6

4. Press the right button (BTN-2) to start advertisement. This will also light 
   the red LED.

5. Initiate a device discovery and connection procedure to discover and connect 
   to the blood pressure sensor from a blood pressure collector peer device.

   The peer device discovers the blood pressure service and configures it to 
   enable indication or notifications of the blood pressure measurement. The 
   peer device may also discover the device information service for more 
   information such as the manufacturing and serial number of the device. 

6. When blood pressure measurements have been enabled, the application sends 
   data to the peer containing simulated measurement values. While the device is 
   processing BP measurements, the green LED is on.

7. Press the up button to cycle through different data formats in the following order:
 1) MMHG | TIMESTAMP | PULSE | USER | STATUS
 2) MMHG | TIMESTAMP
 3) MMHG
 4) KPA
 5) KPA | TIMESTAMP
 6) KPA |TIMESTAMP | PULSE

8. If the peer device initiates pairing, the blood pressure sensor requires a
   passcode. The default passcode is 000000. When the connection terminates, the
   BPM does not begin advertising until the button is pressed. The peer device may
   also query the blood pressure for read-only device information. The GATT_DB
   excel sheet for this project lists further details on the supported items (for
   example, model number, serial number, and so forth).

9. When the blood pressure has been measured a fixed number of times (CUFF_MAX, 
   per default set to 40) the device will disconnect from the master.

Configuration
=============

If you want the device to start advertising immediately, uncomment the AUTO_ADV 
define in the predefined symbols list. 

Blood Pressure Service
======================

The blood pressure service is defined by Bluetooth SIG. It's implemented in 
``bpservice.c``. The service uses a 16 bit UUID: 1810. The service contains three
characteristics, they are listed below.

| Characteristic            | UUID | Properties | Permissions |
|:-------------------------:|:----:|:----------:|:-----------:|
|Blood Pressure Measurement | 2A35 | Indicate   | None        |
|Intermediate Cuff Pressure | 2A36 | Notify     | None        |
|Blood Pressure Feature     | 2A49 | Read       | Read        |


