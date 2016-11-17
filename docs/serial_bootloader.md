Purpose / Scope
===============

This project implements a slightly modified version of the CC26xx/CC13xx ROM Serial Bootloader running from Flash. The purpose is to allow users to modify the boot-loader to suit their application, should the ROM bootloader not fit the needs.

Prerequisites
=============

#### Hardware Requirements

A CC13xx or CC26xx device is needed for this project.

#### Firmware Requirements

To load an image with this bootloader, the linker file for that image must not use Sector 0 or 31. This means the FLASH area of the bootloaded image can be in the range `[0x1000, 0x1F000)`, meaning `0x1F000` is not included, as this is the first address of the last sector (31). 

Serial Bootloading
=========================

A bootloader is an on-chip piece of code that can load a firmware image from an external source into the code memory of the device and execute this code.

The CC26xx/CC13xx devices have a feature in ROM that can do this via UART or SPI on fixed pins. It is always enabled when the device is unprogrammed, but after programming can only be enabled via a 'back-door' pin assert during reset.

See the [CC13xx, CC26xx SimpleLink Wireless MCU Technical Reference Manual](http://www.ti.com/lit/ug/swcu117f/swcu117f.pdf#page=690) Chapter 8 about the Bootloader in ROM. Only changes are described here.

Files / Projects
=========================

* [Project files: examples/util/serial_bootloader](../examples/util/serial_bootloader)
* [Source files: src/util/serial_bootloader](../src/util/serial_bootloader)


Using the Serial Bootloader
=========================

Setting up which PINs to use
-------------------------------

The table below shows the hard-coded pin alternatives for the bootloader signals for the ROM bootloader.

| Signal   | VQFN48, 7 × 7 | VQFN32, 5 × 5 | VQFN32, 4 × 4 |
|----------|---------------|---------------|---------------|
| UART0 RX | DIO2          | DIO1          | DIO1
| UART0 TX | DIO3          | DIO0          | DIO2
| SSI0 Clk | DIO10         | DIO10         | DIO8
| SSI0 Fss | DIO11         | DIO9          | DIO7
| SSI0 RX  | DIO9          | DIO11         | DIO9
| SSI0 TX  | DIO8          | DIO12         | DIO0

This is configurable in the Flash bootloader via precompiler defines. Below is a table of the defines used:

| Signal    | #define             | Default       | Description 
|-----------|---------------------|---------------|-------------
| UART0 RX  | PIN_UART_RXD        | IOID_2        |
| UART0 TX  | PIN_UART_TXD        | IOID_3        |
| SSI0 Clk  | PIN_SSP_CLK         | IOID_10       |
| SSI0 Fss  | PIN_SSP_CSN         | IOID_11       |
| SSI0 RX   | PIN_SSP_MOSI        | IOID_9        |
| SSI0 TX   | PIN_SSP_MISO        | IOID_8        |
| Enter BL  | PIN_BL_ENTER        | IOID_12       | Assert to enter BL on reset
| Enter H/L | PIN_BL_ENTER_ACTIVE | 0             | Assert level (1 = High)

The way the default is configured, the UART should work out of the box on SmartRF06EB with 7x7 EM and on CC2650LAUNCHXL.

The Bootloader can be entered by connecting IOID_12 with GND.
* On CC2650-LAUNCHXL this can be done with a jumper between `DIO12` and `GND`. Change to `IOID_13` to use `BTN-1`.
* On SRF06EB this can be done by holding down the `DOWN` key.

**Note:** The pull direction of the BL pin will automatically be configured as the opposite of the `ACTIVE` value, so if the BL pin is active `LOW (0)` then pull is `PULL_UP` and vice versa. This is different from the ROM bootloader which always configures `PULL_UP`. 

Programming the bootloader
--------------------------
Import either `examples/util/serial_bootloader/ccs/serial_bootloader.projectspec` into CCS or open `examples/util/serial_bootloader/iar/serial_bootloader.eww` with IAR version 7.70.2 or higher.

Build the project and flash onto device.

### **Note on debugging**
When using the bootloader with for example SmartRF Flash Programmer 2 for testing, the `COMMAND_RESET` is issued quite frequently. If the device is not manually reset with a physical PIN reset after programming the bootloader, the debugging subsystem is not completely cleared, and `COMMAND_RESET` will cause the device to hang, waiting for the debugger to tell it to continue.

To avoid this, do a physical PIN reset of the device before sending the `COMMAND_RESET` command to the bootloader.

Making a 'bootloadable' image
-----------------------------

The default operation for `main.c` in this project is to check if the back-door pin is asserted, and if it is not, jump to the user application location.

```c
  // Change the below constant to match the application image intvec start.
  //             vvvvvv
  asm(" MOV R0, #0x1010 ");     // The .resetVecs or .intvecs for the app are
                                // are placed at the constant #0xXXXX address
  asm(" LDR SP, [R0, #0x0] ");  // Load the initial stack pointer
  asm(" LDR R1, [R0, #0x4] ");  // Load the Reset vector address
  asm(" BX R1 ");               // Jump to the application Reset vector
```

The default location it assumes the application image starts is `0x1010`. This is the location used by for example BLE SDK images used for `Over the Air Download (OAD)`. It is 0x1010 instead of 0x1000 because space is reserved for an image header of 16 bytes at the start.

You must configure both the linker file and the TI-RTOS config file to work with the bootloader.

As the CCFG area should not be used by the user application, either exclude `ccfg.c` from the build, or don't tell the linker to place the CCFG section.

Linker files
------------

In order for the application to start at a given location, the linker file used to link the application must specify this.

For reference see these linker files for OAD:
* CCS: `ble_sdk_2_02_01_18/src/common/cc26xx/ccs/cc26xx_app_oad.cmd`
* IAR: `ble_sdk_2_02_01_18/src/common/cc26xx/iar/cc26xx_app_oad.icf`

The pertinent parts are, for CCS:

```
#define FLASH_APP_BASE          0x00001000
#define FLASH_OAD_IMG_HDR_SIZE  0x10
#define FLASH_OAD_IMG_START     FLASH_APP_BASE + FLASH_OAD_IMG_HDR_SIZE
...
MEMORY
{
    FLASH (RX) : origin = FLASH_OAD_IMG_START, length = FLASH_OAD_IMG_MAX_LEN
    IMAGE_HEADER (RX) : origin = FLASH_APP_BASE, length = FLASH_OAD_IMG_HDR_SIZE
...
SECTIONS
{
    .imgHdr         :   > IMAGE_HEADER
    .intvecs        :   > FLASH
    .text           :   > FLASH
...
```

For IAR the linker file example is a bit more complicated, but the essential part is to define FLASH region to be only 0x1000-0x1EFFF. In the below example, the define `APP_IMAGE_START` is provided in the project's extra linker defines as 0x1000.

```
...
define symbol CHKSUM_START        = APP_IMAGE_START;
define symbol FLASH_END           = APP_IMAGE_START + IMAGE_SIZE - 1;
...
// OAD specific
define symbol OAD_HDR_SIZE    = 12; // linker needs word alignment.
define symbol OAD_HDR_START   = CHKSUM_START + 4;
define symbol OAD_HDR_END     = OAD_HDR_START + OAD_HDR_SIZE - 1;

define symbol INT_VEC_SIZE    = 64;
define symbol INT_VEC_START   = OAD_HDR_START + OAD_HDR_SIZE;
define symbol INT_VEC_END     = INT_VEC_START + INT_VEC_SIZE - 1;

define symbol OAD_FLASH_START = INT_VEC_START + INT_VEC_SIZE;
...
define region CHECKSUM        = mem:[from CHKSUM_START  to OAD_HDR_START - 1];
define region FLASH_IMG_HDR   = mem:[from OAD_HDR_START to OAD_HDR_END];
define region INT_VEC         = mem:[from INT_VEC_START to INT_VEC_END];
define region FLASH           = mem:[from OAD_FLASH_START to FLASH_END];
...
place at start of CHECKSUM      { ro section .checksum };
keep                            { ro section .checksum };

// Flash OAD Image Header
place at start of FLASH_IMG_HDR { readonly section IMAGE_HEADER };
keep                            { readonly section IMAGE_HEADER };

// Code and RO Data
place in FLASH { readonly };

// Interrupt Vector Table
place at start of INT_VEC { readonly section .intvec };
keep                      { readonly section .intvec };

```

### TI-RTOS Config file
As mentioned in the Restrictions section, for CC26xx the ROM module of TI-RTOS can't be used.

* Comment out the section related to ROM, for example
```js
/*
 * To use BIOS in flash, comment out the code block below.
 */
//var ROM = xdc.useModule('ti.sysbios.rom.ROM'); <-- comment out
if (Program.cpu.deviceName.match(/CC26/)) {
//    ROM.romName = ROM.CC2650;                  <-- comment out
}
else if (Program.cpu.deviceName.match(/CC13/)) {
    ROM.romName = ROM.CC1350;
}

```

You must also specify where the interrupt vectors should start:
```js
/*
 * Assign an address for the reset vector.
 *
 * Default is 0x0, which is the start of Flash. Ordinarily this setting should
 * not be changed.
 */
m3Hwi.resetVectorAddress = 0x1010; // Changed
```


### Example linker file
As an example, consider the linker file used by the TI-RTOS examples for the Launchpad, `CC2650_LAUNCHXL.cmd`:

```c
#define FLASH_BASE              0x0
#define FLASH_SIZE              0x20000
//...
MEMORY
{
    /* Application stored in and executes from internal flash */
    FLASH (RX) : origin = FLASH_BASE, length = FLASH_SIZE
//...
```

In order to work with the flash-based serial bootloader, change it like this:

```c
#define FLASH_BASE              0x1010   // Changed
#define FLASH_SIZE              0x20000
#define FLASH_PAGE_SIZE         0x1000   // Added
//...
MEMORY
{
    /* Application stored in and executes from internal flash */
    // Length is excluding the last page
    FLASH (RX) : origin = FLASH_BASE, length = FLASH_SIZE - FLASH_BASE - FLASH_PAGE_SIZE // Changed
//...
```

 


Example usage steps
==============
Using a CC2650 LAUNCHXL board.
1. Build and download serial_bootloader to the launchpad
1. Exit the debug session
1. Import the TI-RTOS Empty Example for CC2650 Launchpad 
1. Change the linker file as shown above
1. Exclude `ccfg.c` from the build. For BLE applications this is usually called `ccfg_app_ble.c` or similar.
1. Open the .cfg file and comment out the `ROM configuration` section to not use RTOS in ROM.
1. Change the .cfg file declaration for `m3Hwi.resetVectorAddress` to match your linker file
1. Build the project
1. Place a jumper between the pins `DIO12` and `GND` and press `Reset` on the board to enter the serial bootloader
1. Open `SmartRF Flash Programmer 2` and select the `.out` file under `Debug/` in your project. (Find the location by right clicking on the file, select properties and look at the Resource tab)
1. Make sure to select erase of **only** `"Pages in image"` as `COMMAND_BANK_ERASE` is disabled.
1. Press Play to program
1. Remove `DIO12` jumper and press `Reset` on the board to enter the application.



Restrictions
=========================

Flash usage
------------

The serial bootloader in Flash needs to reside somewhere in Flash to function, and so adds the following restrictions to the bootloaded image:

### Sector 0
Sector 0 cannot be used by the user application. This is because the ROM starts executing user code from address 0x0000, which is in Sector 0.

This also precludes the user application using the ROM'ed SYS/BIOS content on CC26xx, as it expects to find user-data in Sector 0. For CC13xx this data is expected in Sector 1.

### Sector 31

Sector 31 cannot be used by the user application. This is because this sector contains the CCFG table, and it is not safe to bootload this area.

If Sector 31 is erased and the power goes out before new content is written, the ROM bootloader will not start executing Flash code after restart.


Available commands
------------------
All of the commands available for the ROM bootloader are available, except for the Bank Erase command.

Interface speeds
----------------
The max communication speeds for UART/SPI have not been characterized on the Flash version of the bootloader.

Safe operation
--------------
There is no additional parameter checking to ensure safe operation of the bootloader running from Flash as opposed to ROM, so it is perfectly possible to get the bootloader to overwrite or erase itself. **This is not recommended, will cause faulty operation, and should be avoided.**

References
=========================
* [CC13xx, CC26xx SimpleLink Wireless MCU Technical Reference Manual](http://www.ti.com/lit/ug/swcu117f/swcu117f.pdf#page=690)