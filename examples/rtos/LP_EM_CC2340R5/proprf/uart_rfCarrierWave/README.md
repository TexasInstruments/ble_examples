Purpose / Scope
===============

The purpose of this example is to demonstrate a UART configurable rfCarrierWave, using 
rfCarrierWave as a base. This project is designed to allow the customization over UART of certain configurations like PHY, frequency, and TX power.  

# Notice

 The following example is provided as-is, with any and all faults,
 this is not meant to be a production quality example in line with
 the SDK examples. The following example is meant to demonstrate one
 way to use UART to select specs for rfCarrierWave.
 
## KNOWN ISSUES:

 The program can become unresponsive (to UART inputs) after idling for enough time.


Pre-requisites:
===============

### Hardware Requirements

- 1x [LP-EM-CC2340R5 Launchpad](https://www.ti.com/tool/LP-EM-CC2340R5)
- 1x [LP-XDS110 Debugger](https://www.ti.com/tool/LP-XDS110ET)
- 1x USB type C to USB A cable (included with LP-XDS110ET debugger)

### Software Requirements

- [SimpleLink Low Power F3 SDK (7.40)](https://www.ti.com/tool/download/SIMPLELINK-LOWPOWER-F3-SDK)
- [PuTTY](https://putty.org/)
- [SmartRF Studio 7/8](https://www.ti.com/tool/SMARTRFTM-STUDIO)
	
#### Important Software Setup

- Locate your device's COM port in device manager. 
- In the PuTTY Session, enter your COM port, 115200 for speed, and set connection type to Serial. 
- **In PuTTY it is required in the "Terminal" section to set Local echo and Local line editing to "Force on".**
- Once all settings are selected open the session and you should see the following menu, if you do not see the menu double check your PuTTY settings, and if the device has been flashed. 

```
rfCarrier Selector
Type in the three letter keywords in CAPS found below to select the following menu options:
Type PHY to select communication protocol
Type FRQ to select frequency
Type TXP to select transmit power
Type SRT to start the test
Type STP to stop the test
Type RST to reset the device
--------------------------------------------------
```

> **_Note:_** If you are using SmartRF Studio you will open either version 7 or 8 depending on your device, then select a PHY that allows you to continuous RX. Remember to match the continuous RX frequency and rfCarrierTX frequency; when you run the program you should see a RSSI reading on the continuous RX device.


Running the Demo
================

**A test run of any new configuration requires a device Reset (`RST`).**

### Command Keyword Descriptions 

* Set the PHY with "PHY" followed by a number to select:
	* "0" for 1 mbps BLE
	* "1" for 250 kbps MSK
	* "2" for 250 kbps FEC
	* "3" for 2 mbps BLE
	* "4" for coded BLE 

* To set the Frequency use "FRQ" followed by a four digit number:
	* If you type in "FRQ 2440" you set the frequency to 2.440 GHz; the number you type in is multiplied by 1,000,000.

* To set the transmit TXPower use "TXP" followed by MAX or MIN:
	* "MAX" for maximum (8 dB) TX power.
	* "MIN" for minimum (-20 dB) TX power.

* Type "SRT" to Start the program, you will also see your selected settings outputted in putty.
* Type "STP" to stop the program, this will keep the values of the previous run.
* Type "RST" to reset the program, allowing you to re-enter all settings, it is recommended to `RST` the program every time you intend to change the settings of the PHY.


### Command example

The following is an example of what to type in to configure and run the program in PuTTY:
* PHY 0
* FRQ 2440
* TXP MAX
* SRT
* STP
* RST

How to enable RX functionality 
==============================

To enable a RX version of the rfCarrierWave follow the following steps:

1)	Start with the **rfpacketRX** example from your SDK
2)	Below the `rxCmd.common.timing.relGracefulStopTime`, and above `RCL_Command_submit` place the following code:
    
	```ccs
	rxCmd.config.repeated = 1;

    rxCmd.config.disableSyncA = 1;
    rxCmd.config.disableSyncB = 1;
    rxCmd.config.discardRxPackets = 1;
	```

3)	Below the `RCL_Command_submit` place something like the following, remember to enable UART2 in a similar manner to the uart2echo example.

	```ccs
	sleep(1);    
	while (1)
    {
		char sbuffer[64];
        gRssi = RCL_readRssi();
        memset(sbuffer, 0, strlen(sbuffer));
        sprintf(sbuffer, "RSSI %d \r\n", gRssi);
        UART2_write(uart, sbuffer, sizeof(sbuffer), &bytesWritten);
        memset(sbuffer, 0, strlen(sbuffer));
        UART2_flushRx(uart);
        sleep(1);
    }
	```

4)	Define the global `int8_t gRssi;` place outside of and above `mainThread`. 
5)	Run the program, and possibly have a second device on hand with SmartRF Studio 7/8 running any Continuous TX test at 2440 MHz. 
