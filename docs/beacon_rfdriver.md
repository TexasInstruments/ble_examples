Beacon RF Driver
====================
### Purpose
This project will provide a beaconing demo which uses the RTOS RF Driver, not the entire BLE stack.  This will allow for additional power savings (by limiting post-processing time) and a significantly smaller code size.

Both CCS and IAR projects are included.

### Functional Overview
As currently configured, the project will alternate each advertisement between a 31-byte string of 0xAA's and 0xBB's. If the EVERY_OTHER preprocessor define is removed, the aBeacons will be sent for ABEACON_COUNT followed by one bBeacon.

For more detailed information on the RF driver, see the documentation included with the RTOS installer.


