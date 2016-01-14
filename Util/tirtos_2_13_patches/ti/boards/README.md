Migrating to TI-RTOS 2.15.xx.xx Board File Format
========
There are a number of data structures and conventions that have changed in the latest TI-RTOS 2.15.xx.xx release. This guide will explain how to port TI-RTOS 2.13.00.06 style board files to this new format. The CC2650 Launchpad board files will be used throughout this example, thus this can serve as a porting guide for this evaluation kit.

Board files in the new format have been backported to be compatible with TI-RTOS 2.13.00.06, they are included in Util/board_files directory.

New Drivers
----------
TI-RTOS 2.15.xx.xx features a few new drivers older versions (i.e. 2.13.00.06) are not aware of. Notably these are the:
* RF Driver
* PDM Driver
* Modified Power Driver
* Middleware Display Driver

In order make these files compatible with TI-RTOS 2.13.00.06, all references to the new drivers must be commented out. For example:
```C
/* RF hwi and swi priority */
/*
const RFCC26XX_HWAttrs RFCC26XX_hwAttrs = {
    .hwiCpe0Priority = ~0,
    .hwiHwPriority = ~0,
    .swiCpe0Priority = 0,
    .swiHwPriority = 0,
};
*/
```

HWAttrs Structure Changes
--------------

The HWAttrs data structure is used to configure the peripheral's driver with board specific settings. In the newer RTOS versions, this structure has been updated in a couple of ways:
1. A version system has been added
2. New fields have been added

Since older (i.e. 2.13.00.06) versions of TI-RTOS are not aware of these changes, new fields and version tags must be removed, for example:
```C
//const I2CCC26XX_HWAttrsV1 i2cCC26xxHWAttrs[CC2650STK_I2CCOUNT] = {
const I2CCC26XX_HWAttrs i2cCC26xxHWAttrs[CC2650STK_I2CCOUNT] = {
    {
        .baseAddr = I2C0_BASE,
        .powerMngrId = PERIPH_I2C0, //PowerCC26XX_PERIPH_I2C0,
        .intNum = INT_I2C, //INT_I2C_IRQ,
        //.intPriority = ~0,
        //.swiPriority = 0,
        .sdaPin = Board_I2C0_SDA0,
        .sclPin = Board_I2C0_SCL0,
    }
};
```
Migrating to CC2650 LaunchPad
--------------

The CC2650LP is most similar to the CC2650EM-7ID evaluation module. The main exception is that there is no LCD present by default on the LP. This can be remedied by using the UART logger functionality from the simple_ble_peripheral_uartdisplay project in this repo.
