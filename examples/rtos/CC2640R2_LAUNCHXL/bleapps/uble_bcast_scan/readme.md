Purpose / Scope
===============

The `uble_bcast_scan` project is intended to demonstrate advertising and
scanning using the TI Micro BLE Stack.

This example demonstrates the following functionality of the Micro Stack:

 - Switching/Updating advertising Data
 - Plugging advertising related callback functions
 - Scanning and processing scan results
 - Alternating between advertising and scanning

Advertising
===========

By default the application will not advertise, advertising can be enabled by
selecting the `Bcast Start/Stop` menu option.

The advertisement data is setup to be in the Eddystone format. In general the
advertisement behavior is similiar to the `micro_eddystone_beacon` example
within the SDK.

There are a number of other menu functions that can change advertising paramters.
These include the following

 - Increment/decrement advertising interval
 - Increment/decrement application notice of advertisement end event
 - Control advertisement duty cycle (on/off time)
 - Set TX Power
 - Set advertising channel map
 - Set advertisement data

Scanning
========

By default the application will be in scanning mode. The default scan settings
are set with scan interval = 100ms and scan window set to 200ms.

The scan parameters can be tweaked by changing the parameters
(in units of 0.0625ms) of `ugap_scanRequest` within
`UBLEBcastScan_initObserver`.

References
==========
 * [BLE-Stack User's Guide, See Micro BLE Stack Section](http://software-dl.ti.com/lprf/blestack-latest/)
