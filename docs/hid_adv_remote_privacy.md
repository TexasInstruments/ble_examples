Purpose / Scope
===============

This example project is based of the BLE SDK's HID Advanced Remote example with Bluetooth Low Energy 4.2's Privacy 1.2 feature enabled. Given the example behaves identically to the example provided in the SDK, refer to the BLE SDK's documentation for the example's behavioral details.

To observe the behavior of Privacy 1.2, a Bluetooth Low Energy sniffer is recommended. With Privacy 1.2 enabled, it is possible to enable resolvable private addressing with a few steps:

1. Configure the GAP bond manager to automatically add the bonded addresses into its white list: `GAPBondMgr_SetParameter(GAPBOND_AUTO_SYNC_WL, sizeof(uint8_t), &autoSyncWhiteList);`

1. Configure GAP to use a resolvable random private (RPA) address. By default, this will cause the device to change its device address every 15 minutes: `GAP_ConfigDeviceAddr(ADDRMODE_PRIVATE_RESOLVE, NULL);`

1. When configured for `ADDRMOD_PRIVATE_RESOLVE`, you change the time interval in which device generates new RPA addresses. This is set via: `GAP_SetParamValue(TGAP_PRIVATE_ADDR_INT, 5);` where '5' is the time interval in minutes.

# Modified source files
The following files can be compared with the original files shipped with the BLE SDK.

| Original in BLE SDK                                          | Modified for Privacy 1.2       |
| -------------------                                          | ------------------------       |
| `src\examples\hid_adv_remote\cc26xx\hid_adv_remote.c`        | `hid_adv_remote_privacy.c`     |
| `src\profiles\roles\peripheral.c`                            | `peripheral_privacy.c`         |

Running the Demo
================

1. Compile and load the projects:
    - First build and load `cc2650rc_stack`
    - Next build and load `cc2650rc_app`

1. Using a Bluetooth Low Energy 4.2 Privacy 1.2 enabled central device (e.g.  iOS), preform a pairing process.

1. Terminate the connection or simply wait 60s for the remote to disconnect (DEFAULT\_HID\_IDLE\_TIMEOUT).

1. As the device will generate a new RPA approximately every 5 minutes, wait for at least 5 minutes before reconnecting to the central device (e.g. pressing a key). Via the sniffer, you should observe a new RPA being used when advertising.  However to the central device, you should see that it can resolve the RPA and reconnect to the previously bonded device without having to undergo the pairing process.
