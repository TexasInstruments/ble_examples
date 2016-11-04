multi_role
====================
### Functional Overview
This project will provide a demo to demonstrate the capabilities of the Texas Instruments BLE 2.2.1 stack to funciton in multiple simultaneous connection roles. Specifically, the project is capable of functioning as a master and a slave simultaneously, in any combination, while maintaining up to 8 conections. Both CCS and IAR projects are provided.

The project is capable of connecting to any central / peripheral device.  Any type of legacy pairing / bonding is possible. Parameter updates are supported. Connections can be established in any role. The project supports multiple simultaneous GATT discoveries and / or pairings / bondings.

### Heap Considerations
Due to the large amount of functionality in this project, it is rather RAM-constrained.  If the project is configured for too many connections (via the MAX_NUM_BLE_CONNS preprocessor define) and also security, it is possible for heap allocation failures to occur which will break the stack. Therefore, the project should be stress-tested for its intended use case to verify that there are no heap issues by including the HEAPMGR_METRICS preprocessor define. See the debugging section of the software developer's guide for more information on how to do this.

The following table provides an estimate of the amount of additional RAM that is needed past the base configuration of 1 connection without security. This is total additional RAM needed, accounting for both static RAM needed by adding / removing security and max heap size after reaching a steady state.  Note that these are just typical numbers. Run-time analysis should be performed as described above.

|connections|no security|security|
|:----------:|:----------:|:-------:|
|1|0|801|
|2|938|1799|
|3|1848|2601|
|4|2770|3660|
|5|3715|4624|
|6|4652|5398|
|7|5569|6582|
|8|6499|7512|

### Project Configurations
In order to provide more RAM to allow for more connections / security, there is an additional project configuration for both the IAR and CCS projects to use the cache as RAM.  Of course, using the cache as RAM comes at the tradeoff of potential increased processing time and thus higher power consumption.

Coming soon: detailed analysis of power conusmption with Cache as RAM

### Other Considerations

When at least one connection is already formed, in order to allow enough processing time to scan for a new connection, the minimum possible connection interval (in milliseconds) that can be used is:

    12.5 + 5*n

where n is the amount of current connections. For example, if there are currently four connections, all four connections must use a minimum connection interval of 12\*5 + 5\*4 = 32.5 ms in order to allow scanning to occur to establish a new connection.

### Assumptions
For this demo, the terms master / central and slave / peripheral are used synonymously. It is assumed that the master / central devices are GATT clients and slave / peripheral devices are GATT servers. Once the connection limit (set with the MAX_NUM_BLE_CONNS preprocessor define) is reached, the multi_role device won’t be allowed to advertise / scan until there is a disconnection.

When connected in the master role, the multi_role device will query the slave device for the simpleGATTProfile service in order to demo some basic GATT procedures (read / write characteristic).  However, it is not necessary for the slave device to contain this service in order for a  connection to be established. That is, it is possible for the peripheral devices in the system to have differing attribute tables since the multi_role device will perform a service / characteristic discovery after each connection is formed.

### Demo Requirements
##### Hardware
- 2 CC2650 Launchpads
- 1-7 other devices to connect to the multi_role.

##### Software
- multi_role project from this GIT page
- [TI 2.2.1 BLE-SDK](https://www.ti.com/blestack) if it is desired to use the simpleBLEPeripheral project as a slave to expose the simpleGATTProfile to demo GATT procedure functionality

### Multi_role User Interface
The multi_role uses the two launchpad buttons to accept user input and outputs information through the UART.  In order to view this information, a terminal application such as PuTTY can be used. Ensure to configure the serial port to use a baud rate of 115200.
###### Main Menu
Upon powering on or resetting the device, the user will be in the main menu. When in the main menu, the left button will scroll through the possible actions which are:
- Scan
- Connect
- GATT Read / Write
- Connection Update
- Disconnect

The right button is used to select one of these options. When selected, each option is defined below:

###### Scan
Pressing the right button will start a device discovery to search for advertising devices. By default, the application will filter out any devices which do not advertise the simpleGATTProfile UUID (0xFFF0). This can be disabled by setting the following define in multi_role.c to FALSE:

	#define DEFAULT_DEV_DISC_BY_SVC_UUID    FALSE

###### Connect
Pressing the right button from the main menu while "Connect" is shown will enter the connect menu. Here, the left button is used to browse through devices discovered from the last Scan. If a scan has not occurred yet, there won't be any devices to connect to.  In this menu, the right button will select a device to connect to.  It is also possible to browse to and select "Cancel" to return to the main menu.

###### GATT Read / Write
Pressing the right button from the main menu while "GATT Read / Write" is shown will enter the "GATT Read / Write" menu. Here, the left button is used to browse through connected devices. When a device is selected with the right button, a GATT read / write of simple profile characteristic 1 will be sent to the connected device.  Each time a device is chosen, the action toggles between read and write with the write value incrementing by 1.  It is also possible to browse to and select "Cancel" to return to the main menu.

###### Connection Update
Pressing the right button from the main menu while "Connection Update" is shown will enter the "Connection Update" menu. Here, the left button is used to browse through connected devices. When a device is selected with the right button, a connection parameter update is sent to the connected device with the following parameters:
- 100ms minimum connection interval
- 187.5ms maxmimum connection interval
- 0 slave latency
- 2 seconds supervision timeout

It is also possible to browse to and select "Cancel" to return to the main menu.

###### Disconnect
Pressing the right button from the main menu while "Disconnect" is shown will enter the "Disconnect" menu. Here, the left button is used to browse through connected devices. When a device is selected with the right button, a link termiantion request will be sent to the connected device.  In this menu, the right button will select a device to connect to.  It is also possible to browse to and select "Cancel" to return to the main menu.