
Purpose / Scope
===============

This sample project is used to add observer role to a peripheral Bluetooth low
energy peripheral device to show scanning functionality in a peripheral based project. The project is based on
simple_peripheral. This project is configured to run on the CC2650 Launchpad. 


Method
======

1. Compile and load the projects:
    - First build and load `cc2650lp_stack`
    - Next build and load `cc2650lp_app`

2. Connect to the LaunchPad via PuTTY (or other serial console emulator). For instructions on connecting via PuTTY, please see our [FAQ page](faq.md).

3. After connecting you will notice that the device is advertising, the terminal will display as below. (note that your BD_ADDR may be different)

        BLE Peripheral Observer
        0xB0B448D09105
        Initialized
        Advertising

4. You may initiate scanning at any time by pressing KEY\_LEFT (BTN-1), scanning may be canceled via KEY\_RIGHT (BTN-2). Scan reports are displayed on the terminal as

        Scanning On
        Device info 1. 0x247189083301
        Device info 2. 0x247189083A84
        Device info 3. 0xB0B448EE1902
        Device info 4. 0x68C90B07B68B
        Device info 5. 0x9059AF0B7C82
        Devices discovered: 5
        Scanning Off


5. Additionaly the device can support a peripheral role connection while scanning, once connected to a peer, the terminal will show. Note your peer addr will be different

        Num Conns: 1
        0x45C8713837B3
