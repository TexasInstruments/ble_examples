# ANCS Demo

## Table of Contents

* [**Introduction**](#Introduction)
* [**Hardware Prerequisites**](#Hardware Prerequisites)
* [**Software Prerequisites**](#Software Prerequisites)
* [**Preprocessor Defines**](#Preprocessor Defines)
	* [**USE\_GUI\_COMPOSER**](#USE_GUI_COMPOSER)
	* [**USE\_WATCHDOG\_TIMER**](#USE_WATCHDOG_TIMER)
	* [**IGNORE\_PREEXISTING\_NOTIFICATIONS**](#IGNORE_PREEXISTING_NOTIFICATIONS)
* [**Usage**](#Usage)
	* [**GUI Composer Demonstration**](#GUI Composer Demonstration)
	* [**UART Demonstration**](#UART Demonstration)
	
## <a name="Introduction"></a>Introduction

The ANCS project implements a simple Bluetooth low energy
peripheral device acting as a GATT client which receives Apple notification data from a mobile Apple device acting as a GATT server. This project is meant to demonstrate a basic use of the Apple Notification Center Service or ANCS, and can be adapted for use in a variety personal electronic applications. The ANCS's specification can be found here:
[**Apple Notification Center Service (ANCS) Specification**](https://developer.apple.com/library/content/documentation/CoreBluetooth/Reference/AppleNotificationCenterServiceSpecification/Introduction/Introduction.html).

This project uses stack and app configurations that are summarized in the table below:

App Build Configuration     | Compatible Stack Configuration | Description
----------------------------|--------------------------------|-------------------------------------------------------------------
FlashROM_StackLibrary       | FlashROM_Library               | Application build configuration linked to Stack library (fully executable)
FlashROM_StackLibrary_RCOSC | FlashROM_Library               | Application build configuration linked to Stack library for 32-kHz crystal-less device configuration

FlashROM configurations use the split image configuration, which is explained
[**here**](../../../../../docs/blestack/ble_user_guide/html/cc2640/platform.html#split-image-configuration).
FlashROM_Library configurations use the stack library configuration, which is
explained [**here**](../../../../../docs/blestack/ble_user_guide/html/cc2640/platform.html#stack-library-configuration).

This document discusses the procedure for using the ANCS
application when the FlashROM or FlashROM_StackLibrary configurations are used.
The FlashROM_StackLibrary_RCOSC configuration is discussed in the
[**RCOSC section of the BLE-Stack User's Guide for Bluetooth 4.2**](../../../../../docs/blestack/ble_user_guide/html/ble-stack/index.html#using-32-khz-crystal-less-mode).

## <a name="Hardware Prerequisites"></a>Hardware Prerequisites

The default ANCS Demo configuration uses the
[**LAUNCHXL-CC2640R2**](http://www.ti.com/tool/launchxl-cc2640r2). This hardware
configuration is shown in the below image:

<img src="resource/hardware_setup.jpg" width="300" height="400" />

For custom hardware, see the [**Running the SDK on Custom Boards section of the
BLE-Stack User's Guide for Bluetooth 4.2**](../../../../../docs/blestack/ble_user_guide/html/ble-stack/index.html#running-the-sdk-on-custom-boards).

This application also requires a mobile Apple device running iOS that supports Bluetooth 4.2. 

## <a name="Software Prerequisites"></a>Software Prerequisites

This project requires either the [**companion GUI Composer application**](https://dev.ti.com/gallery/)
 or a serial terminal application such as [**PuTTY**](http://www.putty.org/) 
 or [**Tera Term**](https://ttssh2.osdn.jp/index.html.en) for its demonstrations.

For information on what versions of Code Composer Studio and IAR Embedded
Workbench to use, see the Release Notes located in the docs/blestack folder. For
information on how to import this project into your IDE workspace and
build/run, please refer to [**The CC2640R2F Platform section in the
BLE-Stack User's Guide for Bluetooth 4.2**](../../../../../docs/blestack/ble_user_guide/html/cc2640/platform.html). 

## <a name="Preprocessor Defines"></a>Preprocessor Defines

This application has optional functionality toggled in the form of preprocessor 
defines. For CCS, you can learn how to add and modify preprocessor 
defines [**Here**](../../../../../docs/blestack/ble_user_guide/html/cc2640/developing_in_ccs.html#accessing-preprocessor-symbols).

#### <a name="USE_GUI_COMPOSER"></a>USE\_GUI\_COMPOSER:

This symbol determines which demonstration mode the application is in.
If USE\_GUI\_COMPOSER is defined, the project will support the 
[GUI Composer 2.0 demonstration](#GUI Composer Demonstration). If instead it is not defined, the
application will support the [UART demonstration](#UART Demonstration) instead.

By default USE\_GUI\_COMPOSER is defined.

#### <a name="USE_WATCHDOG_TIMER"></a>USE\_WATCHDOG\_TIMER:

This symbol determines if a watchdog timer (WDT) is enabled in the application. The WDT is enabled when USE\_WATCHDOG\_TIMER is defined, and disabled when the 
symbol is undefined. The WDT is set to start counting down once the CC2640R2 is 
advertising. The WDT is configured to wait five seconds before being kicked, and the
 periodic task resetting the WDT will trigger every half-second. Both of these values
  can be changed by adjusting WATCHDOG\_TIMER\_TIMEOUT\_PERIOD and 
  ANCSAPP\_PERIODIC\_EVT\_PERIOD found at the top of ancsApp.c respectfully. 

By default USE\_WATCHDOG\_TIMER is undefined.


#### <a name="IGNORE_PREEXISTING_NOTIFICATIONS"></a>IGNORE\_PREEXISTING\_NOTIFICATIONS:

This symbol determines if the application should include notifications
that exist on the mobile Apple device before the session with the CC2640R2 was 
initiated. Such notifications are ignored if IGNORE\_PREEXISTING\_NOTIFICATIONS is
defined, conversely preexisting notifications will be processed by the application if the 
symbol is undefined. 

By default IGNORE\_PREEXISTING\_NOTIFICATIONS is undefined.

## <a name="Usage"></a>Usage

This application has two primary modes of usage, a GUI Composer 2.0 demonstration,
or a UART terminal demonstration. 

For both demonstrations, this document will use an iPhone 6 Plus to serve as the mobile Apple 
device hosting the ANCS. Note that any mobile Apple device that supports Bluetooth 4.2 should work accordingly. 

### <a name="GUI Composer Demonstration"></a>GUI Composer Demonstration

The GUI Composer 2.0 partner application can be accessed [**Here**](https://dev.ti.com/gallery/).
Simply search through the list of applications featured in the gallery for one titled **ANCS Demo**.

<img src="resource/ANCS_demo_figure8.png" width="1000" height="350" />

Once located, click on the project to run the application. You should now be presented with this 
page:

<img src="resource/ANCS_demo_figure9.png" width="1000" height="400" />

You may click okay **CLOSE** on the README.md window, and wait until the prompt in the bottom left 
of the application reads **waiting for data...** as seen in the above picture.

On the mobile Apple device advertising is enabled by default upon startup, so there is no need to
physically press a button to start advertising. Unlock the mobile Apple device 
and open the Settings App. Open the Bluetooth settings tab and notice that "ANCS Demo"
can now be connected to:

<img src="resource/ancs_demo_figure2.png" width="300" height="533" /> 

Next, a prompt asking to pair to "ANCS Demo" should appear mobile Apple device:

<img src="resource/ancs_demo_figure4.png" width="300" height="533" />

Tap "Pair" and the mobile Apple device should complete the bonding process with the ANCS Demo.
The "ANCS Demo" should now appear with an "i" encompassed by a circle signifying
stored bond data:  

<img src="resource/ancs_demo_figure5.png" width="300" height="533" />

The GUI application should now be displaying various notification data as it is retrieved:

<img src="resource/ANCS_demo_figure10.png" width="1000" height="400" />

Once notification data is no longer being retrieved for the existing notifications, you may view 
retrieved notifications by selecting them in the droplist box titled **Select Notification:**

<img src="resource/ANCS_demo_figure11.png" width="1000" height="400" />

If the mobile Apple device receives an incoming call, you may accept or decline the call using the 
CC2640R2's right (Accept) and left (Decline) buttons. This is an image of the GUI application
after accepting a an incoming call:

<img src="resource/ANCS_demo_figure12.png" width="1000" height="400" />

**Note:** If the mobile Apple device generates any new notifications, the ANCS Demo
will automatically retrieve the notification's data and display it respectively.


## <a name="UART Demonstration"></a>UART Demonstration


This document will use PuTTY to serve as the display for the output of the
CC2640R2 LaunchPad for the UART demonstration. Note that any other serial 
terminal application can be used. The following default parameters are used
 for the UART peripheral for display:

  UART Param     |Default Values
  -------------- |----------------
  Baud Rate      |115200
  Data length    |8 bits
  Parity         |None
  Stop bits      |1 bit
  Flow Control   |None

Once the ANCS Demo application starts, the output to the terminal
will report its address and the fact that it is advertising, as shown below:

<img src="resource/ancs_demo_figure1.png" width="637" height="400" />

Advertising is enabled by default upon startup, so there is no need to
physically press a button to start advertising. Unlock the mobile Apple device 
and open the Settings App. Open the Bluetooth settings tab and notice that "ANCS Demo"
can now be connected to:

<img src="resource/ancs_demo_figure2.png" width="300" height="533" />

Tap "ANCS Demo" to connect to it. The terminal will now display "Connected" and 
the address of the connected mobile Apple device as seen below:

<img src="resource/ancs_demo_figure3.png" width="637" height="400" />

Back on the mobile Apple device a prompt asking to pair to "ANCS Demo" should appear:

<img src="resource/ancs_demo_figure4.png" width="300" height="533" />

Tap "Pair" and the mobile Apple device should complete the bonding process with the ANCS Demo.
The "ANCS Demo" should now appear with an "i" encompassed by a circle signifying
stored bond data:  

<img src="resource/ancs_demo_figure5.png" width="300" height="533" />

Back on the terminal a stream of notification data should begin to be displayed.
These notifications can be found in the mobile Apple device's Notification Center.
With each notification seven articles of data will de displayed:

1. The number of notifications left to be processed 
2. The category of the notification
3. The iOS application producing the notification's ID
4. The display name of the iOS application
5. The title of the iOS application
6. The message of the notification
7. The date and time the notification was created

<img src="resource/ancs_demo_figure6.png" width="637" height="400" />

Additionally, if the mobile Apple device receives an incoming call, the call
can be declined by pressing the button on the right side of the device. 
Alternatively, the call can be accepted by pressing the button on the left
side of the device.

This is an example UART output of a call being declined:

<img src="resource/ancs_demo_figure7.png" width="637" height="400" />

**Note:** If the mobile Apple device generates any new notifications, the ANCS Demo
will automatically retrieve the notification's data and display it respectively.
