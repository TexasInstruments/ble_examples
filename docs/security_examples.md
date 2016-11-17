Security Examples
====================
### Purpose
This project will demonstrate the various security methods available using the BLE v2.2.0 stack.

This demo is comprised of two projects, representing both sides of a connection:
   - security\_examples\_central
   - security\_examples\_peripheral

Note that there are both CCS and IAR projects for both of theses.

### Instructions
The projects are controlled in the same method as the simpleBLEPeripheral and simpleBLECentral projects. For more information on the user interface, see the "Sample Applications" section of the software developer's guide.

The different security types can be chosen by setting the PAIRING define in security_examples_central.h / security_examples_peripheral.h. The options are:
```c
#define OOB_LE                  0x01    // out of band legacy
#define OOB_SC                  0x02    // out of band secure connections
#define JUSTWORKS               0x03    // just works
#define PASSCODE                0x04    // passcode
#define NUMCOMP                 0x05    // numeric comparison
```

The STATIC_PASSCODE define can be set to:
- 0: passcode will be entered using the keypad
- 1: static passcode will be used, thus mimicking passcode entry

For more detailed information on how the various security modes operate, see the GAPBondMgr section of the software developer's guide
