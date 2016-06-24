MultiRole
====================
### Purpose
This project will provide a demo to demonstrate a MultiRole project using the Texas Instruments BLE 2.2.0 SDK. The project is capable of functioning as a master and a slave simultaneously. Both CCS and IAR projects are provided.

### Functional Overview
As opposed to the 2.1.0 multi_role example, this version is capable of connecting to any central / peripheral device since connection parameter updates and security are now supported.  Connections can be established in any role.

### Limitations / Rules
Due to the large amount of functionality in this project, it is rather RAM-constrained.  If the project is configured for too many connections (via the ) and also security, it is possible for heap allocation failures to occur which will break the stack. Therefore, the project should be stress-tested for its intended use case to verify that there are no heap issues. See the debugging section of the software developer's guide for more information on how to do this.

When at least one connection is already formed, in order to allow enough processing time to scan for a new connection, the minimum possible connection interval (in milliseconds) that can be used is:

    12.5 + 5*n

where n is the amount of current connections. For example, if there are currently four connections, all four connections must use a minimum connection interval of 12\*5 + 5\*4 = 32.5 ms in order to allow scanning to occur to establish a new connection.

### MultiRole User Interface
###### Main Menu
Upon powering on or resetting the device, the user will be in the main menu.

The following actions can be taken from the main menu (listed next to their action key):

- Up button: scan for devices
- Left button: browse through discovered devices
- Right button: turn advertising on / off
- Down button: browse connected devices
- Select button: connect to a discovered device or enter the Device Menu for the given connected device

Note that the select button will function differently depending on whether the last action was browsing through discovered devices or connected devices.
###### Device Menu
Once the device menu is entered, the following actions can be taken:

- Up button: Read / Write Characteristic. This is currently only available for devices connected in the central role.
- Left button: nothing.
- Right button: perform connection parameter update
- Down button: Go back to the Main Menu.
- Select button: disconnect from the given device.
