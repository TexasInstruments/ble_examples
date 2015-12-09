Redirect LCD writes to UART
==========================

Most of the sample applications in the BLE SDK use the LCD display on the SmartRF06 evaluation board to show status information.

For boards without an LCD, it's possible to hijack the LCD writes and output them in other ways.

The project files in this folder show how, using helper files in this repository, the `board_lcd.h` interface used by the sample apps can be left alone, but the output can be sent to the UART.

The project files reference `SimpleBLEPeripheral` source files from the BLE SDK, but the steps can be applied to any project.

Steps
-----
The steps below are applied to this project. A further description can be found in the Readme for `Components/uart_log` in this repository.

1. Add `uart_logs.c` from `Components/uart_log` in this repository to the project.

2. Remove `board_lcd.c` from the project, and add `Projects/ble/common/cc26xx/board_lcd_uartlog.c` from this repository instead.

3. Add an include path for `Components/uart_log` in this repository, so `board_lcd_uartlog.c` can initialize the UART logging.

4. Add logging to the TI-RTOS configuration file (see references below)

5. Remove the global Log disable define `xdc_runtime_Log_DISABLE_ALL`

6. (Optional for LCD to UART) Change `__FILE__` behavior to use short path.


Bonus exercise
-----

Observant readers will note that since the method for LCD redirection uses the Log functionality and the project is set up to use this, feel free to throw in a couple of `Log_info0("Hello world!");` in the source files.

Note the limitations in the UART Log readme, and also note that nothing will be output before the "LCD" has been opened, as this is what tells `uart_logs.c` about the UART.

Build configurations
-------------------
The IAR and CCS application projects have two build configurations each, the difference between them being which Board.c and Board.h file to use. This is to let the source files refer to `Board_UART` independently of the board used.

* FlashROM-STK
  - SensorTag rev 1.2+ board files from SensorTag patch files in BLE SDK 2.1
* FlashROM-SRF06EB
  - SmartRF06 Evaluation board 7x7 EM board files from TIRTOS 2.13.00.06

#### Choosing Build Config in IAR
When the CC2640App project is active, select build configuration from the drop-down menu in the workspace pane.

#### Choosing Build Config in CCS
Right click on the SimpleBLEPeripheral project, go to Build Configurations -> Set active, and choose from the list.

#### Debugger connection
Be aware that the debugger connection in some cases must be configured separately from the build configuration. If the wrong debugger connection is active, refer to the Software Developers User Guide in the BLE SDK install folder or the CCS help on how to change this.

References
--------
* [UART Log Readme](../../../Components/uart_log)
* Git diff for the steps: f2b4f0d4ae023dc02111a9aef8f1cdf49ccd5194
