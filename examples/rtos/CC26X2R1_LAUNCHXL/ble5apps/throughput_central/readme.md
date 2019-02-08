# Bluetooth 5 Throughput Central

## Functional Overview

This project is meant to be used on a SimpleLink CC26x2R1 LaunchPad along with a 
Bluetooth 5 Throughput Peripheral to demonstrate what throughput can be 
achieved using TI SimpleLink Bluetooth 5 devices.

This project is based on the Simple Central application found in the BLE5-stack
component of the SDK.

This demo demonstrates how to achieve a high data throughput over Bluetooth low 
energy. The project also showcases the different settings of Bluetooth 5, such as 
the different PHY-settings: 
- LE 1M PHY
- LE 2M PHY 
- LE Coded PHYs
Two different versions of the Coded PHY are available: S=2 and S=8 coding. In 
addition to these features, the demo allows for selecting two different packet 
sizes (27 B and 251 B), along with RSSI reading.

When connected to the Throughput Peripheral, the user can run the throughput 
demo, transferring data using GATT Notifications. The user can 
control the PHY, PDU size and connection interval used in the connection. 

The Peripheral generates data and sends it to the Central where the average and
instant throughput rates are calculated and shown.

The following changes have been made to the Simple Central example:
- Added support for a custom profile called Throughput Profile
- Updated menu with options to adjust PDU size
- Included throughput calculation
- Removed auto PHY-switch feature
- Changed default MTU size
- Added filtering of discovered devices by UUID
- Added display options for LCD

## Project Hardware
- 1 CC26x2R1 Launchpad (http://www.ti.com/tool/LAUNCHXL-CC26X2R1)
- 1 UART Terminal on PC or 1 BOOSTXL-SHARP128 LCD boosterpack (http://www.ti.com/tool/BOOSTXL-SHARP128)

To use the Launchpad with the LCD boosterpack, add the following predefined 
symbols:
	
	BOARD_DISPLAY_USE_LCD=1
	BOARD_DISPLAY_USE_UART=0
  	BOARD_DISPLAY_USE_UART_ANSI=0

## Two Button Menu

The Two Button Menu system is designed to utilize the two push buttons available
on the CC26X2R1 LaunchPad. The left button (BTN1) is always used to cycle
between options. The right button (BTN2) is used to activate the selected action
or item.

The Two Button Menu is set up by default to utilize the back channel UART
present on the LaunchPad as its display.

## Running the demo

1.  Build and load the projects:
	- First build 'ble5_throughput_central_cc26x2r1lp_app'
	- Next build and load 'ble5_throughput_central_cc26x2r1lp_app'

2. 	Connect to the Launchpad via PuTTY (or some other serial console emulator).

3.	After connecting to the device with the serial console, the Two Button Menu
	will appear. Here, you can either select scanning PHY (the Peripheral 
	advertises on both 1M and Coded PHY) or scan for devices.
	Select "Discover Devices" to scan for devices. By default, the central 
	filters devices by the advertised service UUIDs, looking for the Throughput
	Service.
	
4. 	While discovering, the device lists the address of the discovered devices 
	which advertise with the Throughput Service UUID. After it has finished 
	scanning, the number of devices which fulfill the criteria is displayed.
	
5.	To connect to a discovered device, select "Connect To" followed by the 
	device to connect to.

6.	After connecting, the peripheral sends a connection parameter request to the
	central, updating the connection interval from the default 100 ms to the 
	desired connection interval. By default, this is 200 ms.
	
7.	Select "Work With" followed by the address of the device to communicate with.
	The devices will then exchange MTU, set the initial PDU size and set the 
	initial PHY. After this has been set, the RSSI reading will start 
	printing the current RSSI every second.

8.  Select and press "Toggle Throughput" to start sending data from the Peripheral
	to the Central. The Instant Rate and Average Rate will now be displayed and
	updated to show the calculated throughput.

9. 	While connected, the PDU size and PHY can be changed by selecting their 
	corresponding menus, "Set Conn Phy" or "Set Conn PDU", followed by selecting
	the desired option. This can be done while throughput is off as well as on.
	
10. To disconnect from the device, select Disconnect. 

## Throughput calculation
When the Central receives a ATT_HANDLE_VALUE_NOTI-event, it adds the length of
the notification packet received to a variable called bytesRecvd. This variable
is displayed every second as the Instant Throughput. It is also added 
to a circular buffer for average throughput calculation, and the average throughput
is also displayed every second.

## More information

For more information regarding how the throughput is generated and sent, the different 
connection parameters and how they affect throughput, see the readme for Throughput 
Peripheral: 
[Throughput Peripheral readme](./../throughput_peripheral/readme.md)