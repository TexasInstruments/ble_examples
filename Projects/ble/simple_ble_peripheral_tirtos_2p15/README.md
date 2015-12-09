# BLE stack v2.1 - TI RTOS 2.15 Porting Guide


This guide will show how to port the SimpleBLEPeripheral project to use TI RTOS
2.15.00.17 instead of the original 2.13.00.06 version.

## Why upgrade?
- Beta version of PDM (pulse density modulation) driver for microphones
- Bug fixes in several drivers (UART, SPI, I2C)
- RF driver for proprietary modes
- All drivers now have configurable interrupt priority
- Board file fixes for CCS
- Improved calibration routine for oscillators
- Kernel power management is now a separate driver

If you do not have any problems with your current solution using TI RTOS 2.13.00.06,
it is recommended to not update.

This version of TI RTOS has not undergone thorough testing with the BLE stack so
there might still exist unknown issues.

Before starting the porting process, please read the migration guide found at:
- http://processors.wiki.ti.com/index.php/TI-RTOS_Migration_2_15

## Prerequisites
- Clone the ble-sdk-210-extra repository:
 - https://github.com/ti-simplelink/ble-sdk-210-extra.git
- Download and install TI RTOS for SimpleLink Wireless MCUs (2.15.00.17):
  - http://www.ti.com/tool/ti-rtos-mcu

The example projects in ble-sdk-210-extra have split the stack installation path
from the project so that they are movable. All included files and search paths
references the stack installation using the variable *TI_BLE_SDK_BASE* which
should point to your stack installation path.

# Steps

All changes necessary to manually port the project are shown in the Git diff of
this repository.

1. (Optional) Make a copy of the following files:
 - Components/hal/target/_common/cc26xx/mb_PATCH.c
 - Components/icall/ports/tirtos/ICallCC2650.c
 - Projects/ble/common/cc26xx/board_lcd.c
 - Projects/ble/simple_ble_peripheral_tirtos_2p15/CC26xx/Source/Application/main.c
 - Projects/ble/simple_ble_peripheral_tirtos_2p15/CC26xx/Source/Application/simpleBLEPeripheral.c

 The copies can either be renamed to append *_tirtos_2p15_port* as in this
 example or modified directly.

  These files must be added to your workspace and the originals excluded from
  the build.

2. Add a missing compiler search path to your project.
This step is necessary if you are working on the ble-sdk-210-extra repository.
 - IAR: Properties -> C/C++ Compiler -> Preprocessors
    - $TI_BLE_SDK_BASE$/Components/icall/ports/tirtos
    - Update board file to point to: $TI_RTOS_DRIVERS_BASE$/ti/boards/CC2650DK_7ID
 - CCS: Properties -> Build -> ARM Compiler -> Include Options:
    - "${TI_BLE_SDK_BASE}/Components/icall/ports/tirtos"
    - Update board file to point to: "${TI_RTOS_DRIVERS_BASE}/ti/boards/CC2650DK_7ID"

3. In the CC2640Stack project, the unused file hal_rtc_wrapper.h from HAL/Target/CC2650/Drivers.
   This no longer compiles as an interrupt name has changed.

4. Modify mb_PATCH.c in stack(hal/target/_common/cc26xx/mb_PATCH.c)(interrupt names changed)
 - INT_RF_CPE1    -> INT_RFC_CPE_1
 - INT_RF_CPE0    -> INT_RFC_CPE_0
 - INT_RF_HW      -> INT_RFC_HW_COMB
 - INT_RF_CMD_ACK -> INT_RFC_CMD_ACK

5. Modify the includes of the Power module ( ICallCC2650.c, main.c)
This is now a driver located together with the other TI RTOS drivers.
  -  ti/sysbios/family/arm/cc26xx/Power.h -> ti/drivers/Power.h
  -  ti/sysbios/family/arm/cc26xx/PowerCC2650.h -> <ti/drivers/power/PowerCC26XX.h

6. Modify Power related variables and modified API in ICallCC2650.c:
  - Add definition of Power API no longer exposed:
    - extern bool PowerCC26XX_isStableXOSC_HF(void);
  - Power notification API has changed:
    -  -static Power_NotifyResponse ICallPlatform_pwrNotify(Power_Event eventType, UArg clientArg)
    -  +int ICallPlatform_pwrNotify(unsigned int eventType, uintptr_t eventArg, UArg clientArg)
  - Power defines have changed. See diff for details.

7. Update RTOS config file to use the same structure as TI RTOS examples. See
   diff for an example configuration file.

8. Update SimpleBLEPeripheral.c to use correct LCD include path:
  - ti/mw/lcd/LCDDogm1286.h

## CCS specific steps

1.  Properties -> Linked resources:
   - Update CC26XXWARE and TI_RTOS_DRIVERS_BASE paths. CC26XXWARE must be updated in both stack and Application

2. Properties -> General -> RTSC:
   - Select TIRTOS 2.15.0.15 and XDCTools 3.32.0.06.

3. Properties -> Build -> ARM Linker -> File Search Path:
   - Update path to RTOS drivers library to "${TI_RTOS_DRIVERS_BASE}/ti/drivers/lib/drivers_cc26xxware.arm3"

4. Modify path of LCD driver source (now located in ti/mw instead of ti/drivers)

5. Remove path to old Board.c file, add new CC2650DK_7ID.c file to workspace (for 7x7 EVM)

## IAR specific steps

1. Update custom argument variables (SimpleBLEPeripheral.custom_argvars), restart workspace. See diff for updated TI RTOS paths.

2. Modify path of LCD driver source (now located in ti/mw instead of ti/drivers)

3. Remove path to old Board.c file, add new CC2650DK_7ID.c file to workspace (for 7x7 EVM)

4. Update the paths to configPkg as instructed in the TI RTOS Migration 2.15 wiki
