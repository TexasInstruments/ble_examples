Purpose / Scope
===============

This sample project is used to add observer role to a BLE peripheral device to
show scanning functionality in a peripheral based project. The project is based
on simple_peripheral. This project is configured to run on the CC2640R2
Launchpad. 


Method
======

1. Compile and load the projects:
    - First build `simple_peripheral_observer_cc2640r2lp_stack_library`
    - Next build and load `simple_peripheral_observer_cc2640r2lp_app`

2. Connect to the LaunchPad via PuTTY (or other serial console emulator).

3. You will notice that when the device is advertising, the terminal will display 
   as below. (Note that your BD_ADDR may be different)

        BLE Peripheral Observer
        0xB0B448D09105
        Advertising

4. You may initiate scanning at any time by pressing KEY\_LEFT (BTN-1), scanning 
   may be canceled via KEY\_RIGHT (BTN-2). Scan reports are displayed on the 
   terminal as

        Scanning Off
        Devices discovered: 14
        Advertising Addr: 0x4EFF5168AE58 Advertising Type: Connectable undirected
        Advertising Data: 02:01:1A:0A:FF:4C:00:10:05:01:90:74:DB:90:00:B6:E2:7D:A1:55:68:64:1D:2E:1E:B5:35:2D:34:20:


5. Additionally the device can support a peripheral role connection while 
   scanning. Once connected to a peer, the terminal will show the peer address 
   as below. Note your peer addr will be different.

        Num Conns: 1
        0x45C8713837B3
